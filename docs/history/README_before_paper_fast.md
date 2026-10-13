> 2026-09-20 r2：修复 Quad3D 跨工具链回归测试，生产代码/六环境参数不变。最新用户虚拟机实测 3/6（Quad3D 两组、Quad2D Bugtrap 通过；Uni 两组及 Quad2D Hole 未解）。请阅读 [测试修复与当前实验状态](docs/quad3d_baseline_portability_fix.md)。

> 本轮交付：**RRTstarDFPFF_planar_fix_20260920**。Quad3D 配置/DF 冻结，新增搜索策略仅在 Uni/Quad2D 启用。请先阅读 [本轮修改与虚拟机运行说明](docs/planar_fixes_20260920.md)。下文为此前版本记录。

# RRTstarDFPFF

2026-09-20 六环境 0/6 后的最新修改见 [搜索修正说明](docs/search_fixes_20260920.md)：完整目标连接、物理可行时间候选、Uni 有符号/不对称控制限制、平坦预筛、诊断计数与可选引导采样。本轮新版真实六环境结果待 VM 验证；以下真实库接入说明保留为上一阶段记录。

当前独立副本为 `RRTstarDFPFF_dyno_20260920`。原始 `RRTstarDFPFF_next`、此前 SIMD 副本和 `reference/` 保留。真实库接入、虚拟机命令和验证边界见 [六环境实验说明](docs/real_dynobench_20260920.md)。历史 SIMD 工作见 [SIMD 实施说明](docs/simd_implementation_20260920.md)。

当前共用一套 RRT* / PFF 核心，支持 scalar 和运行时调度的 AVX2。已接入批量 PFF、SoA 最近邻、轨迹 Horner 求值、圆/球碰撞预筛、确定性 Top-K 排序，以及 rewire 全状态固定的连接。

## 当前边界

默认构建支持 `system: linear`。启用 `RRTSTARDFPFF_ENABLE_DYNOBENCH=ON` 并提供已有 DynoBench 源码后，统一 `benchmark` 增加 `unicycle`、`quad2d`、`quad3d` 的真实模型入口。新代码使用真实 `robot_factory`、FCL、`step()`、`distance()` 和 `Trajectory::check()`；旧参考 adapter 仅供历史测试。源码接入不等于六环境已经运行成功，真实链接、集成测试和六环境成功率必须在具备依赖的虚拟机确认。

## 构建与运行

需要 CMake 3.16+、C++17 编译器、Eigen3 和 yaml-cpp：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRRTSTARDFPFF_ENABLE_AVX2=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
build/benchmark --config benchmark/config/linear_smoke.yaml
build/benchmark --config benchmark/config/linear_simd_smoke.yaml
build/benchmark --config benchmark/config/linear_simd_fast.yaml
# 全部 SIMD（包含当前开销偏大的 PFF 批处理）
build/benchmark --config benchmark/config/linear_simd_reference.yaml
python benchmark/compare_backends.py --exe build/benchmark --iterations 1000 --runs 3
```

Windows 使用 `.exe` 后缀；多配置生成器需加 `--config Release` 和 `ctest -C Release`，可执行文件位于 `build/Release/`。离线依赖可通过 `-DRRTPFF_EIGEN_SOURCE=<Eigen源码目录>` 与 `-DRRTPFF_YAML_SOURCE=<yaml-cpp源码目录>` 提供。

`simd.enabled: false` 在同一二进制中关闭加速；`RRTSTARDFPFF_ENABLE_AVX2=OFF` 构建不含 AVX2 内核的版本。不支持 AVX2 的 CPU 自动回退。AVX2 编译选项只作用于独立内核，不作用于整个程序。

## 验证原则

碰撞预筛只能拒绝，不能作为可行性证明。LinearModel 继续使用连续多项式极值/圆障碍验证。rewire 固定已有节点的完整 FlatState 和 branch，后代 incoming edge 不重算，只传播累计代价。

`benchmark/compare_backends.py` 固定迭代次数、种子与参数，记录 scalar / NN / NN+screen / 全部 SIMD 四组结果，并核对节点数、rewire 次数、最终代价及轨迹长度。其加速比只适用于该工作负载，不能推广为论文或六环境的加速比。

历史审计见 [修改说明](修改说明.md)、[逐函数迁移表](docs/function_migration.tsv)、[完整需求清单](docs/requirements_checklist.md)。本轮改动文件已统一分行排版，reference 保持原始内容与格式。

2026-09-20 实测：Windows Release 的 AVX2 / 无 AVX2 两种构建均通过 7/7 CTest；五圆环境固定 10000 次迭代、5 个种子结果一致。推荐 NN+预筛配置约 1.178×，全部 SIMD 约 1.059×；详见实施说明与原始 CSV。此结论仅覆盖 Linear 闭环。
