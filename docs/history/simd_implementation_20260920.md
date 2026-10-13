# SIMD 实施与验证记录（2026-09-20）

本轮修改目录是 `RRTstarDFPFF_next`。原始 `RRTstarDFPFF` 和 `reference/` 不参与新实现，也不被覆盖。当前实现复用一套 RRT*、PFF 和 SIMD 接口，没有按机器人复制规划器。

## 计算顺序与不变量

1. Flat output 采样；最近邻/半径搜索通过模型显式声明的加权欧氏或周期距离进入 SoA/AVX2 路径。其他度量回退 scalar。
2. 每批最多 8 个父节点，使用相同终端约束 mask 的 PFF 求解；位置固定、导数自由的 expansion 与全状态固定的 rewire 使用同一个求解器。
3. 批量计算归一化 Gramian 二次型和时间方程；保留递归导数隔离、重根、端点、全部候选时间。临近零残差使用 long double 参考求值决定符号。没有用符号扫描替代原求根器。
4. 用 scalar reference 重建每条候选的 costate、轨迹、终端状态和代价。Top-K 按 `parent.cost + edge.cost` 排序，相同代价按原生成顺序排序。
5. 对选中的候选，先 SIMD 求值多个时刻的状态/控制和球/圆障碍；预筛只提前拒绝。保留下来的边继续调用模型完整验证。LinearModel 保留连续多项式极值与圆障碍距离验证。
6. 插入节点后，rewire 的连接约束为 `Constraint::full(node->state)`。只修改被重连节点的父指针和 incoming edge，节点状态、branch 和后代轨迹保持不变，后代累计代价逐层更新。
7. 删除/清空树会使 NN 的 SoA 缓存失效。单纯 rewire 不改变节点状态，无需重建坐标缓存。

## 本轮文件与接口

| 文件 | 改动 |
|---|---|
| `simd/simd_backend.h/.cpp` | 稳定 ABI、CPU/OS 能力检测、scalar fallback、批量内核调度 |
| `simd/simd_avx2.cpp` | 独立编译的 AVX2 Horner、距离、PFF 二次型、球障碍和基础算术 |
| `simd/simd_roots.h` | 保留重根的批量隔离/二分求根 |
| `simd/simd_pff.h` | `candidatesBatch` 返回所有候选；混合 mask、无效输入和短批次安全回退 |
| `simd/simd_flat_eval.h` | `evaluateSoA`，导数 0…Order，区分 FlatState 与 FlatControl |
| `simd/simd_collision.h` | 时间交错采样、圆/球预筛、状态/控制范围预筛、scalar 确认 |
| `pff/pff.h` | 约束 mask 的归一化 Gramian 分解缓存，统一时间多项式与候选重建接口 |
| `pff/pff_integrator.h`、`pff/pff_cost.h`、`core/node.h` | 将原有 GBK 中文注释转换为 UTF-8，消除 Clang 编码警告；数学与节点语义不变 |
| `core/nearest_neighbor.h` | NN/Near 的增量 SoA 与批量距离；近边界 scalar 复核 |
| `core/tree.h` | 缓存失效 epoch，不改变节点所有权 |
| `core/rrtstar.h` | expansion/rewire 批处理，确定性 Top-K，验证顺序及真实计数 |
| `systems/linear_model.h` | 显式 metric/screen hook，保留连续完整验证 |
| `df/quad3d_df.h` | 修复 Clang 拒绝嵌套 Config 默认参数的问题，保留 Config 别名与原参数值 |
| `common/planner_config.h`、`common/types.h` | 分项开关、计数和耗时字段 |
| `benchmark/benchmark_runner.h`、`benchmark_result.h` | YAML 配置读取与 CSV 输出 |
| `benchmark/config/linear_simd_*.yaml` | 可直接运行的 SIMD smoke/reference 参数 |
| `benchmark/compare_backends.py` | 固定 workload、相同种子、四组 ablation、结果一致性检查 |
| `tests/test_simd.cpp`、`test_end_to_end.cpp` | 多维/多阶/尾部/根/碰撞/树/路径回归 |
| `CMakeLists.txt` | AVX2 仅作用于内核文件；可选离线 Eigen/yaml-cpp 源目录 |

## 数值与平台选择

规划热路径使用 double，8 个候选按两个 AVX2 double4 寄存器处理。`SimdFloat8` 提供基础算术接口，但没有把 PFF 时间根、排序或碰撞边界降为 float。未启用 FMA/fast-math；本轮先保证 scalar/SIMD 判定一致。

标量源文件不带全局 `-mavx2` 或 `/arch:AVX2`。编译包含 AVX2 内核不代表强制执行 AVX2：运行时检查 CPU 和 OS 支持。`simd.enabled: false` 或编译 `RRTSTARDFPFF_ENABLE_AVX2=OFF` 可关闭。

```yaml
simd:
  enabled: true
  width: 8
  pff: true
  nearest_neighbor: true
  collision_screen: true
  collision_samples: 33
  interleaved_samples: true
```

`simd_pff_lanes` 统计批量能量计算处理的候选数；`simd_flat_samples` 统计批量轨迹求值处理的时刻数；`simd_nn_lanes` 统计距离处理的节点数。尾部 scalar 项不冒充 SIMD 项。旧的 `simd_evaluations` 为三项之和，单位不同，不应单独拿它计算吞吐率。

`pff_time` 是整个 PFF 阶段，包含 SIMD 和 scalar 工作。`simd_time` 是走 SIMD 流程的 PFF 调用及预筛耗时，含打包、回退与结果重建，不是硬件指令独占时间；NN 另外报告 `nn_time`。这些时间有包含关系，不能相加当总时间。`screen_time` 和 `validation_time` 用于分离预筛与完整验证。

## 构建与运行

已安装 Eigen3、yaml-cpp、CMake 和 C++17 编译器的环境：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRRTSTARDFPFF_ENABLE_AVX2=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/benchmark --config benchmark/config/linear_simd_smoke.yaml
./build/benchmark --config benchmark/config/linear_simd_reference.yaml
python benchmark/compare_backends.py --exe build/benchmark --iterations 1000 --runs 3
```

Windows 对应执行文件有 `.exe` 后缀。无系统安装的依赖可通过 `RRTPFF_EIGEN_SOURCE` 与 `RRTPFF_YAML_SOURCE` 指向源目录。本次本机便携工具链路径和实测结果补充在下方验证记录。

## 明确未完成的部分

- 三个机器人在真实 DynoBench 中的六环境端到端运行仍未接通，benchmark 继续明确拒绝非 linear 系统。已有参考 adapter 测试不能当作真实 DynoBench 验证。
- Quad 姿态/四元数/推力的 SIMD 逆变换、完整 DynoBench 碰撞 SIMD、AABB/OBB 内核没有实现。
- 完整树节点内存池、分层空间索引和结构全量 SoA 没有实现；本次 SoA 用于 NN 缓存和批量数值数据。
- 候选 FlatTrajectory 仍在排序前由 scalar reference 重建；尚未实现只对 Top-K 延迟生成多项式和完整结果对象。
- 没有以稀疏碰撞采样替换连续验证，也没有通过改变代价或根集合制造速度提升。
- 保留原有 Top-K 和采样策略，不额外声称具有完整 RRT* 渐近最优性证明。

## 快照边界

`backups/simd_before_20260920/manifest.json` 记录了本轮集中改造前的文件快照。该快照中的 `simd_backend.h` 已是本轮较早的中间草稿，不是改造前原始头文件；不能将整个快照误称为完整可运行的旧版。原始项目和 reference 源文件继续独立保留。

`reference/` 的 20 个文件已与既有 `docs/reference_sha256.json` 逐项核对，零差异；本轮核对记录见 `docs/simd_reference_verification.json`。

## 实测验收

环境：Windows x86-64，Intel Core i5-1155G7，Zig 0.13.0 / Clang 18.1.6，Release `-O3`、baseline CPU + 独立 AVX2 内核，Eigen 3.4.0，yaml-cpp 0.8.0，CMake 4.4。

| 检查 | 结果 |
|---|---|
| Windows Release 编译与链接 | 通过；修正了 Quad3D Config 默认参数、GBK 注释和 CPU 探测的工具链兼容问题 |
| AVX2 构建 CTest | 7/7 通过，4.40 s；实际报告 `Compiled AVX2=1 runtime AVX2=1` |
| 完全关闭 AVX2 内核后的构建与 CTest | 7/7 通过，5.20 s |
| Scalar / AVX2 smoke | 各 1/1 成功，代价约 2.74636 |
| 固定 1000 次迭代，3 个相同种子，四种模式 | 全部成功，工作量和结果一致 |
| 固定 10000 次迭代，5 个相同种子，四种模式 | 全部 5/5 成功，工作量和结果一致 |
| Rewire 不变量 | 状态和 branch 固定、子孙边复用、累计 cost 传播测试通过 |
| Continuous validation | 薄障碍不能由稀疏预筛放行；每条插入边完整复验通过 |
| 真实 DynoBench 六环境 | **未完成；不能宣称完整 RRT* + DF + PFF + SIMD 已跑通** |

这里的 CTest `dynobench` 是参考 adapter 测试，不是链接真实 DynoBench 的 ground-truth 测试。

### 性能数据与推荐配置

以下为五圆环境、10000 次迭代、种子 40…44；每项 wall time 为 5 次运行中位数，加速比为同一种子配对加速比的中位数。因此加速比不必等于两列中位时间直接相除。

| 模式 | 中位规划时间 | 配对中位加速比 | 一致性 |
|---|---:|---:|---|
| scalar | 1.901977 s | 1.000× | 基准 |
| SIMD NN | 1.899445 s | 1.001× | 通过 |
| SIMD NN + 碰撞预筛 | 1.593137 s | 1.178× | 通过 |
| SIMD NN + 预筛 + PFF 批处理 | 1.826086 s | 1.059× | 通过 |

一致性检查逐 seed 核对 success、nodes、rewires、iterations、pff_queries；cost、trajectory_duration、path_length 在 `2e-6` 相对/绝对容差内一致。

**推荐使用 `benchmark/config/linear_simd_fast.yaml`**：开启 NN 与预筛，保留有缓存的 scalar PFF。`linear_simd_reference.yaml` 仍开启全部 SIMD，供后续优化 PFF 的批次打包、求根递归和结果物化开销。不能把该 PFF 实现单独宣传为已测得更快：本环境 PFF 阶段中位耗时由约 1.060 s 上升至 1.260 s。

原始 CSV/每组参数/日志和 summary 位于 `results/simd_comparison_10000/`；小工作量对比位于 `results/simd_comparison/`。测试与构建日志位于 `docs/validation_logs/simd_20260920/`。这些是 Windows 当前机器的测量，未声称 Linux、其他 CPU 或三个机器人上的加速比。

### 本机复现命令

便携工具在项目外层 `D:/codex_talk/project_1/tmp/`，不依赖 WSL。可用已生成的构建目录：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR = 'D:/codex_talk/project_1/tmp/zig-cache'
& 'D:/codex_talk/project_1/tmp/simd-build-tools/cmake/data/bin/cmake.exe' --build build_simd_win --parallel 2
& 'D:/codex_talk/project_1/tmp/simd-build-tools/cmake/data/bin/ctest.exe' --test-dir build_simd_win --output-on-failure
./build_simd_win/benchmark.exe --config benchmark/config/linear_simd_fast.yaml
```

不安装到系统目录、不改变系统 PATH。跨目录复制源码时按前文标准 CMake 命令重新配置构建，不能搬用含绝对路径的旧 CMakeCache。

交付前已恢复 AVX2=ON，重新构建并再次通过 7/7 CTest（4.40 s）。最终日志为 `build_final.log` / `ctest_final.log`。
