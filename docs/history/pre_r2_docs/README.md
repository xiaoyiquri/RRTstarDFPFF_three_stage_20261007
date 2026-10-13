# RRTstarDFPFF paper SIMD 20260922

独立新版本，原 `RRTstarDFPFF_paper_fast_20260921` 不覆盖。

本版本按照 FLASK Algorithm 3 的**离散检测流程**实现：
PFF 局部轨迹 → 同一轨迹 8 个时间点 → float8 轨迹求值 → float8 DF 逆变换 → float8 物理状态/控制约束 → float8 球体碰撞 → 接受/拒绝边。
参考：[Ultrafast Sampling-based Kinodynamic Planning via Differential Flatness](https://arxiv.org/html/2603.16059v4)，V-D、Algorithm 3、VII-A。
保留本项目 RRT* + PFF，不把整体算法称作论文原版 FLASK。

## 编译（使用虚拟机已有依赖，无需安装或下载）

```bash
cd "$HOME/MYFILE/newfile/RRTstar/RRTstarDFPFF_paper_simd_20260922"
cmake -S . -B build_simd -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRRTSTARDFPFF_ENABLE_AVX2=ON
cmake --build build_simd --parallel 2
ctest --test-dir build_simd --output-on-failure
```

需要已有 C++17、Eigen3、yaml-cpp、CMake，以及支持 AVX2 的 x86-64 CPU/虚拟机。**不需要 FCL 或 DynoBench 编译产物**。若虚拟机未暴露 AVX2，会明确失败，不会自动转为标量检测。
若 Eigen/yaml-cpp 没有安装到搜索路径，可指定已有源码：`-DRRTPFF_EIGEN_SOURCE=/已有/eigen -DRRTPFF_YAML_SOURCE=/已有/yaml-cpp`。不设置旧的 `ENABLE_DYNOBENCH` 参数。

## 六环境实验

```bash
experiment_dir="$PWD/results/simd_$(date +%Y%m%d_%H%M%S_%N)"
python3 tools/run_six.py \
  --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --build-dir "$PWD/build_simd" \
  --output "$experiment_dir" \
  --runs 1 --timeout 60 --seed 41
cat "$experiment_dir/summary.csv"
```

`--cases uni_bugtrap uni_wall quad2d_bugtrap quad2d_hole quad3d_block quad3d_window` 可选择子集；`--runs 20` 连续使用 seed、seed+1、…；`--first-solution` 获得首解后停止，默认继续 RRT* 优化到 timeout。`first_solution_time` 是规划器首解墙钟秒，`planning_time` 是总规划墙钟秒，`path_length` 是导出的最终位置轨迹折线长度，`trajectory_duration` 是运动时间。

单场景可运行 `./build_simd/benchmark --config benchmark/config/quad3d_window.yaml`，先将该 YAML 的 `dynobench.source` 指向已有数据目录。`run_six.py` 自动生成包含绝对路径的新配置，因此推荐使用脚本。

## 明确的实验语义

- 六配置 `validation.max_step: 0.005`，保留原先指定的 5 ms 上限。每边 N=ceil(T/dt)+1，含首尾；步长实际为 T/(N-1)。每批时间索引为 i+j*ceil(N/8)，j=0…7。尾批使用 lane mask，每个采样点只检查一次。
- **权威碰撞模型是球体集合**。Uni 半径 0.125 m，Quad2D 0.1 m，Quad3D 0.25 m，明确写在 YAML；不是声称论文公布了这些半径，也不是旧矩形机体的外包络。使用这个几何后不应直接和旧 FCL 矩形实验比较成功率/时间。
- 地图轴对齐盒分成边长不超过 0.1 m 的小格，每格用外接球覆盖；因此障碍物球体集合会略大于原盒。直接球障碍保持球体。机器人额外膨胀 1e-5 m。每次运行保存实际使用的 `geometry.yaml`，可据此绘图，而不能继续只画旧矩形机体。
- BVH 只减少球体对的访问：包围盒测试和叶球碰撞均批量覆盖 8 个物理位置。最终几何判据为中心距离 <= 两球半径之和，接触视为碰撞。
- 不再调用标量 DF 候选验证、FCL、DynoBench step/check_trajectory，也不进行独立外部检查。输入 lifting、PFF 求解、排序、树结构和结果序列化仍可使用标量；这些不是物理碰撞检测。
- DF 的角度/三角函数、归一化、四元数、角速度、力矩/电机映射都在 AVX2 `__m256` 中计算，不是 8 次标量函数。PFF 的 double 求解及其原有 SIMD 实现保留，不宣称 PFF 全部变成 float。
- 保留标量 DF 作为测试 reference 和起终点显式 lifting，不在候选边验证链调用它。无阻力。
- ChooseParent/Rewire 移除 Top-K；按真实代价从低到高检查，遇到可行改进即采用，或候选耗尽/超时。原来的 Near 集、goal_parent_limit、采样策略仍保留；取消 Top-K 不表示遍历整棵树。
- rewiring 仍固定已有节点的完整 FlatState 和 velocity branch，仅替换 incoming edge、parent，向后代传播累计 cost。
- `sampled_success` 仅表示指定采样分辨率、球体几何、float32 计算下通过；不是连续无碰撞证明，也不是 DynoBench 真值认证。manifest 标记 `dynobench_verified=false`、`external_checks=0`。

结果输出：`results.csv`、`validation.csv`、`trajectory_N.yaml`、`geometry.yaml`、配置副本和 `manifest.json`。旧绘图程序依赖 `dynobench_success` 与 FCL 几何，不应直接用于本版本。

## 绘图

新版绘图程序读取 `sampled_success=1` 和 `validation_backend=simd_discrete_float8`，不会查找旧的 `validated_success` 字段：

```bash
experiment_dir="$PWD/results/simd_20260922_151048_302055533"
plot_dir="$experiment_dir/figures_$(date +%Y%m%d_%H%M%S_%N)"
python3 ./plot_successful_results_paper.py \
  --experiment "$experiment_dir" \
  --cases uni_bugtrap uni_wall quad2d_bugtrap quad2d_hole quad3d_block quad3d_window \
  --run 0 \
  --output "$plot_dir"
cat "$plot_dir/metrics.csv"
xdg-open "$plot_dir/index.html"
```

绘图只读取每个环境目录内的 `trajectory_0.yaml` 和 `geometry.yaml`，不再需要模型目录参数。无桌面环境时运行 `python3 -m http.server 8000 --directory "$plot_dir"`，再在可访问该虚拟机的浏览器打开 `http://虚拟机IP:8000`。

运行器只要求 `--dynobench-source`。环境文件和官方模型从该目录的 `envs/`、`models/` 读取；Uni 和 Quad3D 的论文覆盖参数在六个 benchmark YAML 的 `dynobench.model_file` 中显式指定，Quad2D 直接使用官方 `models/quad2d_v0.yaml`，不再复制一份相同文件。

## 核心修改位置

- `core/rrtstar.h`：删除 Top-K 硬截断，保留排序/rewire/节点状态规则。
- `simd/float8.h`、`df/simd_inverse.h`：八 lane 数学与三系统逆映射。
- `simd/physical_avx2.cpp`：SIMD 轨迹求值、DF、物理约束、BVH/球体检测。
- `simd/discrete_validator.h`：采样、尾 mask、Bernstein 系数转换和输出。
- `dynobench/data_adapter.*`：仅读取 YAML 和模型约定；没有验证 API。
- `systems/dyno_model.h`：原系统搜索接口接入单一 SIMD 验证路径。
- `tests/test_physical_simd.cpp`：数值对照、倒车、奇异点、NaN、尾 mask、碰撞、取消 Top-K 测试。

`reference/` 与不在 CMake 中注册的历史测试属于历史参考，不是此版运行路径。旧 Quad3D 的整树快照基于旧候选截断和 FCL 几何，不适用于此次明确改变验证语义的版本；保留 PFF、DF、树/rewiring 数值和结构回归。

实测结果和逐项完成情况见 `交付说明.md`。
