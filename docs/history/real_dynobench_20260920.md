# 真实 DynoBench 六环境接入与实验说明

本次独立目录：`RRTstarDFPFF_dyno_20260920`。原目录未覆盖，未下载或安装软件。

**状态：真实库接入代码与实验入口已经补齐；虚拟机的真实链接、real_dynobench 测试及六环境规划结果仍待运行确认。不得把编译成功、局部连接测试通过或参考 adapter 测试通过写成六环境规划成功。**

## 1. 修改映射

| 文件 | 修改 |
|---|---|
| `dynobench/real_adapter.h/.cpp` | 新建真实库边界；实际模型工厂、物理 bounds、FCL 碰撞、step/distance 残差、完整 Trajectory 校验、高精度物理轨迹导出 |
| `systems/dyno_model.h` | 三系统共用模型桥接，分离 flat 轨迹重建与真实验证；Unicycle 特化仅负责 branch 契约 |
| `systems/robot_factory.h` | 系统选择、明确 LiftData、读取实例化 DynoBench 模型参数；不含 RRT*/PFF 实现 |
| `core/rrtstar.h` | 增加可选 `expansionConstraint()` 泛型钩子：普通扩展终端导数自由，采到真实目标时固定完整 lifted goal |
| `benchmark/benchmark_runner.h` | 根据编译开关进入真实机器人 runner；关闭真实库时明确报错 |
| `benchmark/robot_runner.cpp` | 共用机器人实验 runner；最终真实校验决定 success；物理 trajectory YAML 与 validation.csv |
| `benchmark/config/{uni_*,quad2d_*,quad3d_*}.yaml` | 六个完整配置，替换此前仅含四五行的占位配置 |
| `CMakeLists.txt` | 引用已有 DynoBench 源码，不抓取依赖；真实库开关可用；增加真实集成测试和离线模板编译检查 |
| `tests/test_end_to_end.cpp` | 增加完整终态约束的 scalar/SIMD 回归 |
| `tests/test_real_dynobench.cpp` | 六环境真实起终点、局部 PFF/DF 动力学、障碍碰撞、非法控制、损坏轨迹拒绝测试 |
| `tools/run_six.py` | 标准库实验脚本：前置真实测试、六环境运行、独立轨迹检查、manifest、日志和汇总；无安装行为 |

`pff/`、`simd/`、`df/` 的数学实现沿用已有副本。RRT* 仍只有一个模板实现。rewire 仍固定目标节点完整 FlatState，仅替换该节点的 incoming edge/parent，并更新子树累计代价。

## 2. 在你的 Linux 虚拟机编译

先把整个新目录复制到 `~/MYFILE/newfile/RRTstar/RRTstarDFPFF_dyno_20260920`，不要只拷贝 YAML，也不要复用 Windows build 缓存。

```bash
cd ~/MYFILE/newfile/RRTstar/RRTstarDFPFF_dyno_20260920
cmake -S . -B build_real_dynobench \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DRRTSTARDFPFF_ENABLE_AVX2=ON \
  -DRRTSTARDFPFF_ENABLE_DYNOBENCH=ON \
  -DRRTPFF_DYNOBENCH_SOURCE="$HOME/MYFILE/newfile/dynobench-main"
cmake --build build_real_dynobench --parallel 2
cmake --build build_real_dynobench --target check_trajectory --parallel 2
ctest --test-dir build_real_dynobench --output-on-failure
```

应包含 `real_dynobench` 测试。仅有历史 7 个测试表示没有启用真实库。CMake 4 如提示旧依赖 minimum compatibility，原配置命令附加 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`；不需要修改或下载依赖。缺少 FCL/Boost/json 等已有依赖时先停止，保留报错日志。

## 3. 先跑六环境单次验证

```bash
python3 tools/run_six.py \
  --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --runs 1 --timeout 30 --seed 40
```

脚本用已有 Python 标准库，不需要 PyYAML。会先执行真实集成测试；失败立即停止。随后每个环境运行一次，将成功物理轨迹交给独立 `check_trajectory` 二次校验。运行中每个环境开始/结束各一条状态，不轮询。执行错误、进程超时或独立校验失败立即停止；规划正常结束但没找到解会记录失败并继续其他环境。

每次自动创建新的 `results/real_时间戳/`，不会覆盖上一轮结果。退出码：`0` 所有运行真实验证成功，`2` 至少一次正常规划未找到有效解，其他非零表示需要检查日志的执行错误。

也可单独启动：

```bash
./build_real_dynobench/benchmark --config benchmark/config/uni_bugtrap.yaml
./build_real_dynobench/benchmark --config benchmark/config/uni_wall.yaml
./build_real_dynobench/benchmark --config benchmark/config/quad2d_bugtrap.yaml
./build_real_dynobench/benchmark --config benchmark/config/quad2d_hole.yaml
./build_real_dynobench/benchmark --config benchmark/config/quad3d_block.yaml
./build_real_dynobench/benchmark --config benchmark/config/quad3d_window.yaml
```

单独运行时 `dynobench.source` 相对 YAML 目录解析，预设对应本说明的 VM 目录布局。`results.csv` 已存在则拒绝覆盖，重复实验请换 output_directory。推荐使用脚本自动管理路径与新目录。

## 4. 通过单次验证后再做统计实验

```bash
# SIMD；50 runs，seed 40..89，每 run 30 秒规划预算
python3 tools/run_six.py --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --runs 50 --timeout 30 --seed 40

# 相同配置/seed 的 scalar 对照
python3 tools/run_six.py --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --runs 50 --timeout 30 --seed 40 --scalar
```

完整两轮最多约 5 小时规划预算，外加验证/导出时间，不应在单次试验尚未通过时直接启动。固定 wall time 下，SIMD 与 scalar 的实际迭代数量可能不同；不要将其结果差异当作同工作量速度比。正式性能比较应额外使用足够长 timeout 与固定 max_iterations，检查两组实际 iterations 是否一致。

## 5. 真实环境映射与物理契约

| 配置 | DynoBench envs 下路径 | 实际模型 |
|---|---|---|
| Uni Bugtrap | `unicycle1_v0/bugtrap_0.yaml` | `unicycle1_v0` |
| Uni Wall | `unicycle1_v2/wall_0.yaml` | `unicycle1_v2` |
| Quad2D Bugtrap | `quad2d_v0/quad_bugtrap.yaml` | `quad2d_v0` |
| Quad2D Hole | `quad2d_v0/fall_through.yaml` | `quad2d_v0` |
| Quad3D Block | `quadrotor_v0/quad_one_obs.yaml` | **`quad3d_v0`** |
| Quad3D Window | `quadrotor_v0/window.yaml` | **`quad3d_v0`** |

start、goal、位置边界、碰撞几何全部来自真实环境文件；m/J/arm/motor bounds 等来自实例化模型。Quad2D 禁用阻力；不兼容的阻力或非 motor-control 模型会报错。

- Uni state `[x,y,theta]`，control `[signed_v,omega]`，每边固定 branch。默认 LiftData 明确给出 +0.30 m/s，普通边不能过零换向。start/goal 不包含速度，lifting 速度是额外实验条件。当前整条路径继承起点 branch，停车换向模式尚未实现。
- Quad2D state `[x,z,theta,vx,vz,omega]`；control 是按 `mg/2` 归一化的两电机推力；升力方向 `[-sin(theta),cos(theta)]`。
- Quad3D state `[px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz]`；速度在世界系，角速度在机体系，升力沿 body +z，重力沿 world -z。control 为按 `mg/4` 归一化的四电机推力，分配符号与真实 B0 一致。
- Quad2D 使用二维四阶链；Quad3D 使用四维四阶链。yaw jerk 必須显式提供，因为 FlatState<4,4> 包含 yaw 的三阶导数。两端 hover 的 acceleration/jerk/yaw 导数零值是 YAML 明确给出的已知实验条件，不是缺失信息的补零。
- Quad3D 默认采样 yaw=0；这是明确的固定航向实验条件，扩大区间需作为另一实验条件记录。高阶 yaw 的四阶惩罚来自既有链定义，不能与采用二阶 yaw 的论文代价直接混同。

## 6. 验证与性能边界

1. 每条候选先 PFF/排序/Top-K，再 DF 重建与真实逐点 state/control/FCL/step 检查。真实机器人尚无保守的 SIMD 体积碰撞预筛，配置必须 `collision_screen: false`。批量 PFF、SoA NN、flat Horner 求值可以用 AVX2。
2. 首个普通边固定初始 full flat state、终端 output；导数自由。目标连接固定 full lifted goal；重连固定既有 full state。PFF 是平坦线性链的无约束时间/控制代价解，经非线性物理约束筛选，并非物理约束下全局最优局部求解器；固定 Top-K 与有限时间搜索也不宣称完整渐近最优证明。
3. `max_step=0.005`，同时不超过模型 ref_dt。使用真实 `step(x,u,dt)` 比较 DF 重建的下一状态，阈值 `0.001`；全路径再检查真实 start/goal、边拼接及 DynoBench Trajectory。这是有时间分辨率和数值容差的验证，不是连续碰撞自由或精确积分证明。
4. 核对的上游 `check_trajectory` 仅在 verbose 分支更新 max_jump。桥接层无条件独立调用真实 step/distance 并使用该残差，防止静默误判。
5. 物理 YAML 使用 17 位有效数字和显式 times，避免上游 6 位格式化改变轨迹；独立工具检查的是实际落盘文件。
6. `results.csv` 的 success 以最终真实校验为准；`validation.csv` 同时保留 planner_success/dynobench_success、最大跳变、起终点误差和失败原因。无解时不得把 CSV 存在当作实验成功。
7. PFF/DF/SIMD/DynoBench 计时含部分嵌套，不能直接相加；planning_time 是搜索时间，最终复核和导出位于其外。CSV 的验证计数和计时统计搜索及 validatePath，不含 writeTrajectory 的再次校验、写盘或外部工具检查；因此不是整段脚本的总调用数或端到端耗时。脚本 summary 是原始成功次数，不预设任何成功率。

## 7. 交付验收清单

| 项目 | 状态 |
|---|---|
| 独立另存、不覆盖原项目、不安装依赖 | 已完成 |
| 真实六环境路径、状态/control/quaternion 约定核对 | 已按本地 DynoBench 源码核对 |
| 单一 RRT*/PFF/SIMD 核心 | 已保持 |
| full-goal 与 frozen-state rewire | 已接入；回归测试见构建记录 |
| 真实 API、collision、step、完整轨迹校验与导出 | 已实现；真实链接和执行待 VM 确认 |
| 六配置、单次/多 seed 脚本、日志、独立检查 | 已提供 |
| Windows 模板编译/原有测试 | Clang 18.1.6、AVX2 ON/真实库 OFF 编译通过；7/7 测试通过，含新增 full-goal scalar/SIMD 回归；见 `validation_logs/` |
| 虚拟机真实库构建与 real_dynobench 测试 | 待用户执行以上命令 |
| 六环境均找到并验证完整路径 | 未确认；不报告成功率 |
| 50 次正式统计、三机器人性能结论 | 未运行 |

若真实测试失败，先提供 `preflight.log` 或 CTest 报错；若测试通过但规划无解，提供对应环境 `.log`、`results.csv` 和 `validation.csv`，再依据拒绝计数和轨迹约束定位，不放宽真实校验来制造成功结果。
