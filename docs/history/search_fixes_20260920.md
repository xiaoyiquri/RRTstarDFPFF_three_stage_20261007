# 六环境 0/6 之后的搜索修正

本轮修改当前独立工作副本 `RRTstarDFPFF_dyno_20260920`。未覆盖 `RRTstarDFPFF_next` 或 SIMD 副本；最初交付的 `RRTstarDFPFF_dyno_20260920.zip` 保留为修改前快照。新交付包用 `RRTstarDFPFF_search_fix_20260920` 命名，便于在虚拟机与旧版并存。

## 依据与修正范围

用户回传：真实 DynoBench 构建和 8/8 CTest 通过；六环境 seed=40 完整规划均 `planner_success=0`。Uni 最终只有 32/20 个节点，大量 DF 拒绝；四旋翼已经有数千节点，但没有完整目标连接。

这些数据确认搜索未找到可行完整路径，不能证明某一个函数就是全部失败的根因。上一轮分析中两点需要纠正：普通扩展 limitExtension 是正常的局部扩展机制；control_bound_rejected 单一计数也不能归因于速度下界，用户日志已经明确显示了不对称角速度越界。因此本轮增加可验证的机制与细分诊断，不把推测写成已证实根因。

## 逐项实施

| 建议 | 本轮结果 | 文件 |
|---|---|---|
| 细分 DF 拒绝原因 | 新增数值、branch/元数据、零速、最小/最大速度、角速度、加速度、jerk、推力、电机、角加速度、奇异点、采样数量上限计数 | common/types.h、df/*.h、benchmark/benchmark_result.h |
| 显式目标连接 | 新增 goalConstraint；goal-biased sample 使用实际目标，不截短；从最多 16 个邻近目标的父节点尝试完整终态连接。普通随机扩展仍使用 range | core/rrtstar.h、core/sampler.h、systems/dyno_model.h |
| Uni 真实控制区间 | 从实际 DynoBench bounds 配置 signed v 和 omega 上下界。Wall 的 v>=0.25 和 omega>=-0.25 同时生效；保留反向语义、禁止过零 | systems/robot_factory.h、df/unicycle_df.h |
| branch-aware 候选 | 保留原输出固定、终端速度自由的候选，额外提供固定终端速度向量候选。方向为父节点到目标的平坦运动方向，速度幅值沿用父节点，物理正反向仅由 metadata 决定 | systems/dyno_model.h；core 仅调用泛型可选接口 |
| Flat bounds 预筛 | 每条 Top-K 候选在重建前做 16 间隔平坦位置/速度/加速度采样检查；复用 scalar/AVX2 Horner。只筛掉已发现违规者，不宣称连续验证 | systems/flat_bounds.h、systems/dyno_model.h |
| 环境引导采样 | 非目标采样中 20% 取 YAML 指定的通道附近区域，80% 保留全域均匀采样。所有区域仅描述 flat output，不采样 free derivatives | core/sampler.h、common/sampling_region.h、six YAML |
| 首解前减少重连 | 六配置 rewire_before_first_solution=false；发现第一个目标后恢复原重连。重连仍固定既有节点完整状态和 branch，子树仅传播 cost | common/planner_config.h、core/rrtstar.h |
| 提前筛选速度/推力 | Uni 检查完整速度区间及不对称 omega；Quad 在样本处分类检查速度、加速度、jerk、力/推力、角速度/角加速度、电机。密集重建使用同一 sampleFeasible，避免只有粗采样检查附加约束 | df/unicycle_df.h、df/quad2d_df.h、df/quad3d_df.h |
| 参数调整 | 增加候选时间样本与采样/连接/重连开关；保留 timeout=30、max_iterations=100000、R=I、goal_tolerance=1e-6，以及真实 dynamics/endpoint=1e-3 | six YAML |
| 失败诊断 | CSV 增加终止原因、目标连接查询/有效次数、最近目标输出距离、真实 dynamics 拒绝；六环境脚本自动输出失败摘要，提供只读 inspect_results.py | common/types.h、benchmark/*、tools/* |

没有放宽目标和物理校验容差：目前没有证据说明 1e-6 的数值残差导致六环境全部无解。完整目标连接仍解固定终端约束。新 `closest_goal_distance` 是 flat output 距离，不能单独用作物理目标通过条件。

## 候选时间：关键的补充

原实现仅尝试无约束 J(T) 的驻点和 `[min_time,max_time]` 两端。物理约束下，可行时间可能只在中间。新增 `pff.additional_time_samples: 12`，把 12 个等间隔内部时间加入已有集合。

每个时间都由同一个 scalar `fixedTime` 求解 Gramian/PMP 最优曲线，scalar 与 SIMD 共享 `resultsAtTimes`，代价仍为 `T + integral(u' R u)`。不是把已生成的轨迹简单缩放时间，起点导数和固定终端约束每次重新满足。

新增时间通常不满足无约束 `H(T)=0`。因此准确表述是“固定时间最优连接 + 有限候选时间的物理可行性搜索”，不能宣称求解了带非线性约束的全局最优 free-final-time 问题。12 个样本也不保证找全可行区间；计算量会增加，需用新实验测量收益。

## 配置与消融

所有新增机制可独立关闭以比较效果：

```yaml
planner:
  explicit_goal_connection: true
  goal_parent_limit: 16
  terminal_velocity_hint: true  # 仅 Uni，作为额外候选
  rewire_before_first_solution: false
pff:
  additional_time_samples: 12  # 0 恢复原时间集合
validation:
  flat_screen: true
  flat_screen_intervals: 16
sampling:
  guidance_probability: 0.20   # 0 恢复全域均匀采样（仍有 goal bias）
  regions: []                  # 六 YAML 提供实际区域
```

实验引导区域来自已知障碍几何，是额外的环境先验。论文比较必须标明并提供 guidance_probability=0 的消融；不能把引导采样效果归为 SIMD 加速。区域既不是已求得路径，也不是碰撞自由证书。

## 验证

新增 `search_improvements` 回归检查：

- 超过 range 的完整目标连接、关闭开关恢复原限制、障碍物不能被目标连接跳过。
- 一个受 Uni Wall 速度区间约束的连接：驻点和端点均不可行，补充内部时间可行；scalar/SIMD 的候选 T/cost 一致。
- 非对称角速度、倒车 signed velocity 与航向、两端合法而中间速度过低的曲线。
- scalar/SIMD 平坦越界预筛、Quad3D 推力分类，以及引导采样仍保留全域均匀探索。

原 PFF、DF、SIMD、树、重连和端到端测试继续保留。Windows 构建会实例化三个真实模型的模板入口，但本机没有 FCL/Boost 开发环境；新版真实 DynoBench 测试仍需虚拟机重跑。先前 8/8 的 VM 成功只适用于上一版，不自动覆盖本轮修改。

本机实际结果：Clang 18.1.6 / Release / AVX2 ON / 真实库 OFF 构建通过，包含三个真实模型模板入口；8/8 测试全部通过，4.30 s。新增 search_improvements 中的时间候选、目标连接和 signed-control 测试均通过。Linear SIMD smoke 成功 1/1，53 列 CSV 表头和数据一致。

日志见 `validation_logs/search_fix_compile.log` 和 `validation_logs/search_fix_ctest.log`。这些结果不等于新版真实 DynoBench 9/9 或六环境成功。

## 在虚拟机重新验证

将新包 `RRTstarDFPFF_search_fix_20260920.zip` 放入虚拟机 Downloads 后执行：

```bash
python3 -m zipfile -e "$HOME/Downloads/RRTstarDFPFF_search_fix_20260920.zip" "$HOME/MYFILE/newfile/RRTstar"
cd "$HOME/MYFILE/newfile/RRTstar/RRTstarDFPFF_search_fix_20260920"
mkdir -p logs
(
  set -e
  set -o pipefail
  cmake -S . -B build_real_dynobench -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DRRTSTARDFPFF_ENABLE_AVX2=ON -DRRTSTARDFPFF_ENABLE_DYNOBENCH=ON \
    -DRRTPFF_DYNOBENCH_SOURCE="$HOME/MYFILE/newfile/dynobench-main" 2>&1 | tee logs/configure.log
  cmake --build build_real_dynobench --parallel 2 2>&1 | tee logs/build.log
  cmake --build build_real_dynobench --target check_trajectory --parallel 2 2>&1 | tee logs/build_checker.log
  (cd build_real_dynobench && ctest --output-on-failure) 2>&1 | tee logs/ctest.log
)
```

真实库开启时应为 9 个测试，包含 real_dynobench 与 search_improvements；全部通过后：

```bash
experiment_dir="$PWD/results/real_$(date +%Y%m%d_%H%M%S)"
python3 tools/run_six.py --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --build-dir "$PWD/build_real_dynobench" --output "$experiment_dir" --runs 1 --timeout 30 --seed 40
python3 tools/inspect_results.py "$experiment_dir"
```

不需要安装依赖。原环境/模型文件、DynoBench 源码和真实校验阈值不修改。新六环境结果尚待 VM 验证，未报告成功率或性能提升。
