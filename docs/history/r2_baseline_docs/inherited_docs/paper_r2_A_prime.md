# paper_r2 / 无人工地图 Guidance 修改说明

日期：2026-10-06。此目录是原工程的独立副本，不含原 `.git`，原工程未覆盖。

原工程：`D:\codex_talk\project_1\project_code\RRTstarDFPFF_paper_simd_20260928`

新工程：`D:\codex_talk\project_1\project_code\RRTstarDFPFF_paper_r2_noguidance_20261006`

## 本次授权范围

只实现删除人工地图 Guidance，以及 paper_r2 / boundary lifting / 物理恢复和诊断。
不实施先前讨论的 PFF 代价邻域、全路径 float32、T* 自适应搜索、shortcut、120 s/20 seeds 协议。
RRT* 邻域、rewiring 时机、PFF 时间候选、动力学参数和 sphere 几何保持原实验设置。

## 用户确认的 A′ 契约

| 内容 | paper_r2 | full_df |
|---|---|---|
| Quad 节点状态 | y, dy | y, dy, ddy, jerk |
| Quad PFF 多项式 | 三次 | 七次 |
| 伪控制 | ddy | snap |
| 边界 lifting | 位置、速度；Quad3D yaw/yaw rate | 原有高阶 lifting |
| 边内物理恢复 | acceleration、jerk、snap=0 | acceleration、jerk、snap |
| 边内物理约束/球碰撞 | AVX2 时间维 SIMD 采样验证 | 同一验证器 |
| 节点连接 y/dy | 必须匹配 | 必须匹配所有高阶 flat 坐标 |
| 连接姿态/角速度 | 仅诊断，不因误差拒绝 | 验证，不匹配则拒绝最终路径 |
| 恢复后的物理起终点 | 与原请求作诊断 | 必须匹配 |

Uni 本来就是二阶链，两种模式都是三次曲线；signed velocity lifting 和 branch 不变。
`full_df` 的初始和目标高阶 lifting 仍由原 scalar DF 检查其姿态/角速度一致性，最终路径还经过 SIMD 恢复后的端点/连接一致性检查。

paper_r2 不是完整物理状态连续性认证。位置/速度匹配不能推出 Quad 姿态/角速度连续；输出不能标为 DynoBench runtime 验证通过，也不能标为连续碰撞认证。

## 实现

### Guidance

六份运行配置均显式 `profile: paper_fair`、`sampling.guidance_probability: 0`。
移除运行配置中的人工区域和 tilt hints；paper_fair 加载器拒绝重引入这些字段，即使 probability 为零。
保留普通 goal sampling、均匀采样及 Uni 与地图坐标无关的原 steering 策略。
paper_r2 Quad 不再使用包含高阶边界的 hover/rest candidate。
原六配置存档于 `docs/history/pre_r2_configs/`，仅作历史记录，不是当前运行入口。

### 二阶 PFF 与 lifting

`df/paper_r2_df.h` 定义 Quad2DR2DF / Quad3DR2DF，节点维数为 4 / 8。
它们使用现有 `PFFSolver<Dim,2>`，无需另写一套 BVP；代价仍为 T + pseudo-control energy。
普通扩展保留 PFF 自由终端导数，goal/rewire 固定该模式保存的全部 flat-state 坐标。

Quad2D 从原物理位置/线速度投影到 y/dy。
Quad3D 从正规化四元数获取 yaw，并用 Rdot=R*skew(body omega) 求 Euler yaw rate；不会直接把 body omega_z 当 yaw rate。
当 yaw 坐标在竖直机体 x 轴处奇异时，明确拒绝 lifting。
paper_r2 配置不接受 acceleration/jerk/yaw 高阶 lifting，也不接受人为覆盖 yaw_rate。

### 验证导数

积分器阶数和恢复所需导数阶数分开。
三次 PFF 的 jerk 按真实系数解析求值，snap 显式为零；不会为了降阶把 jerk 一并清零。
Bernstein 转换先清空输出缓冲区，再生成最高四阶导数；SIMD Jet 也显式初始化。
`FlatTrajectory::derivative` 对高于多项式次数的非负阶导数返回精确零，时间越界仍报错。
现有八个交错时间点、尾 lane mask、物理限制和 sphere/BVH 碰撞验证保留。

### 诊断和成功口径

最终路径对每条边重新调用同一 SIMD validator。分别计算原请求起点与恢复起点、每个连接处左右边、恢复终点与原请求终点的误差。
连接误差在拼接丢弃重复时间点之前计算，因此不会掩盖两侧姿态/角速度跳变。
paper_r2 只要求 flat 边界和每条边采样验证通过；物理边界误差仅记录。
full_df 除上述检查外，各物理误差也必须小于当前 endpoint_tolerance，否则 reason=`full_physical_boundary_mismatch`。
比较对象是物理状态，不要求控制输入在连接处连续。

每次运行保存 `boundary_diagnostics_N.csv`：

* kind=start/junction/goal、edge_index、time；
* flat_position_error（包含全部 flat outputs，Quad3D 包含 yaw）、flat_velocity_error；
* physical_position_error、physical_velocity_error；
* attitude_error_rad、omega_error。

Quad3D 姿态误差是四元数符号不变的最短旋转角；Quad2D/Uni 使用 wrapped angle。
full_df 采用原 endpoint_tolerance 对每种误差分别判定，单位为 m、m/s、rad、rad/s；不把它们合并成新的优化目标。

`results.csv` 和轨迹 YAML 标记：

* df_mode、pseudo_control_order、boundary_contract；
* reduced_order_result、physical_boundary_match；
* solver_precision=float64、time_root_precision=long_double、validation_precision=float32。

manifest 记录所选模式、boundary_contract、精度、输入哈希及预算；逐轨迹误差和匹配结果见 CSV。

`physical_boundary_match` 只是当前容差下测量结果；即使为 true，也不是完整连续性证明。
轨迹 YAML 中 requested start/goal 仍是原 benchmark 请求；`states` 是实际恢复的采样状态，可能与请求姿态/角速度不同，需结合诊断文件解释。
拼接后的离散 states 保留上一条边的连接点，另一侧的差异由诊断 CSV 单独记录。

## 配置和运行

### 修改文件索引

| 文件 | 改动 |
|---|---|
| df/paper_r2_df.h（新增） | 二阶 Quad 节点、位置/速度/yaw lifting |
| common/flat_trajectory.h | 三次曲线的四阶导数返回零 |
| simd/physical_batch.h、physical_batch.cpp | 积分器阶数与恢复导数阶数分开 |
| simd/discrete_validator.h、physical_avx2.cpp | Bernstein jerk、零 snap、Jet 初始化 |
| systems/batch_config.h | 为二阶派生 DF 装配物理参数 |
| systems/dyno_model.h | 二阶模式接入、连接/端点诊断、full_df 一致性检查 |
| systems/robot_factory.h | 模式选择、paper_fair 配置拒绝人工 Guidance |
| common/boundary_diagnostics.h（新增）、common/types.h | 误差度量和结果元数据 |
| benchmark/robot_runner.cpp、benchmark_result.h | 模式分派、诊断 CSV、结果列 |
| dynobench/data_adapter.h/.cpp | 导出模式、精度及连续性语义 |
| benchmark/config/*.yaml、full_df/*.yaml（新增） | 两套无地图 Guidance 配置 |
| tools/run_six.py | --df-mode 参数及 manifest |
| CMakeLists.txt、tests/test_paper_r2.cpp（新增）、test_paper_profile.cpp、test_run_six.py | 验证模式、导数、物理跳变及配置 |
| tools/audit_original.py（新增） | 原工程完整性核对 |

默认六配置 `benchmark/config/*.yaml` 为 paper_r2。
`benchmark/config/full_df/*.yaml` 为 full_df，保留高阶 lifting，同时关闭人工地图 Guidance。
两组配置皆沿用原 30 s、runs=1 等默认设置；运行器可显式覆盖。

```bash
python tools/run_six.py --dynobench-source /path/to/dynobench-main \
  --build-dir build_r2 --df-mode paper_r2 --runs 1 --timeout 5 \
  --output results/new_r2_run

python tools/run_six.py --dynobench-source /path/to/dynobench-main \
  --build-dir build_r2 --df-mode full_df --runs 1 --timeout 5 \
  --output results/new_full_run
```

数据源仍只有 YAML，不调用 DynoBench/FCL runtime。
修改后的小预算结果用于运行检查，不保证无 Guidance 后六环境都成功；不得直接继承旧工程成功率或性能数字。

## 验证与原工程保护

新增 `test_paper_r2`：三次曲线 jerk/snap、两种 Quad 的 double-reference/SIMD 对照、yaw-rate 投影、A′ 的诊断跳变、flat 断点拒绝和 full_df 物理端点拒绝。
更新 paper_profile 和运行器测试，检查两个模式的配置及 Guidance 禁用。
本次实际构建/测试结果见根目录 `交付说明.md`，历史 validation_logs 不是本次结果。

`docs/source_before_changes.json` 是原工程 661 个文件（含原 .git）的改前哈希。
执行 `python tools/audit_original.py` 可核对原工程，并写 `docs/original_integrity_check.json`。
核对不写原工程目录；独立副本未包含原 `.git`。
