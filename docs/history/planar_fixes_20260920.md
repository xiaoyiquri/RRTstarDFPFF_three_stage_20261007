# Uni / Quad2D 定向修正；Quad3D 回归基线冻结

独立交付目录：`RRTstarDFPFF_planar_fix_20260920`。本轮从 `RRTstarDFPFF_search_fix_20260920` 复制，旧版源码和压缩包保留。

## 实验依据

用户虚拟机的上一版结果（seed=40、每组 30 秒）：Uni Bugtrap/Wall、Quad2D Bugtrap/Hole 搜索超时；Quad3D Block/Window 均通过内部与独立 DynoBench 轨迹检查。前四组 `goal_valid=0`。Uni Bugtrap 主要 DF 拒绝为 omega；Uni Wall 主要为最小速度；Quad2D 送检轨迹的碰撞拒绝率约 99.59% / 98.64%。

这些计数不证明所有失败都由某一个函数导致。本轮根据已确认的候选截断顺序和重复验证路径修改，并用具体回归测试验证，不通过放松碰撞、速度、控制或目标容差来增加成功数。

## Quad3D 冻结方式

- `quad3d_block.yaml`、`quad3d_window.yaml`、`quad3d_df.h` 逐字节保持不变。
- scalar PFF、SIMD PFF、SIMD backend 和 flat evaluator 数学实现不变。SHA-256 清单在 `validation_logs/quad3d_frozen_baseline.json`。
- 共用 RRT* 仍只有一套；新策略均默认关闭，只在四个二维 YAML 启用。真实 benchmark 会拒绝在 Quad3D 打开本轮搜索开关。
- `quad3d_baseline` 测试使用上一版核心生成的固定迭代参考数据，比较新版 Quad3D 维度/阶数下的树、父节点、代价和边系数，覆盖 scalar / SIMD。夹具由上一版 include 路径和上一版静态库编译的程序生成，未从新版倒推。
- 这个确定性测试使用 Quad3D DF，可检测共用算法变化，但没有 FCL 环境。真实 Block/Window 回归仍须在虚拟机运行。按墙钟限时的两次实验不能保证节点数或耗时逐位相同。

## 修改清单

| 修改 | 接口与位置 | 验证目标 |
|---|---|---|
| 预筛失败不占用昂贵 Top-K 名额 | `PlannerConfig::precheck_before_top_k`；`core/rrtstar.h` | 低代价但预筛失败的候选不能遮蔽后面的可行时间 |
| 稀疏真实碰撞预检查 | `precheckTrajectory`、`trajectoryValidAfterPrecheck`；`systems/dyno_model.h` | 碰撞终点在密集重建前拒绝；预检查漏掉的小障碍仍由完整验证拒绝 |
| 精确目标候选缓存 | `cache_goal_connections`；`core/rrtstar.h` | 按不可变 node id 和本次 solve 的固定目标/求解器缓存；未检查的候选保留到下轮，不能缓存成失败 |
| 扩展父节点多样性 | `minimum_parent_candidates`；共用 NN 的 nearestK | Near 太小时补足最近的若干完整状态，最终仍按真实边代价和可行性选择 |
| Uni 转向方向目标 | `limitExtension` 可选策略；`systems/dyno_model.h` | 利用平坦速度方向和真实不对称 omega 区间生成位置提议，PFF 和 DF 仍负责生成、验证实际曲线 |
| Uni 终端速度候选 | `additionalExpansionConstraints` | 显式终端速度幅值与转向角；保留原自由终端 PFF、原附加候选及固定 branch |
| Quad2D 悬停终态候选 | `planar_search.rest_hint` | 补充位置固定且 v/a/j 显式为零的最优控制连接，减少惯性状态在障碍附近继续漂移 |
| Quad2D Hole 倾斜终态候选 | `planar_search.tilt_hints` | 在指定洞口区域提供明确的姿态/速度边界，经过同一 PFF 求解与真实验证 |
| 分阶段诊断 | CSV 和 `inspect_results.py` | 预检查次数/拒绝、碰撞预拒绝、目标缓存复用/耗尽、密集重建次数 |
| 分组运行 | `tools/run_six.py --cases ...` | 可单独跑两组 Quad3D 回归或四组二维实验，默认仍运行六组 |

筛选采用“按总代价排序、逐个执行便宜预检查、只有通过者消耗完整验证预算”的惰性执行方式。并未声称所有候选都先通过连续约束证明。完整密集 DF、实际 DynoBench step、状态/控制边界和 FCL 校验保留。Rewire 的节点完整状态/metadata 不变，子树仍仅传播累计代价。

目标缓存只在一个静态环境的 `solve()` 内有效，下次 solve 清空。缓存每个具体时间候选的验证结论；未被 Top-K 验证到的候选仍为待验证。所有时间都确实拒绝后，才跳过该父节点。重连降低父节点代价后，成功连接可重新参与比较。不得把本策略直接用于动态障碍环境而不增加缓存失效机制。

## 二维参数与数学含义

四个二维配置启用上述搜索开关，`minimum_parent_candidates=8`、`top_k=16`、`collision_precheck_samples=9`。这里 Top-K 从“最多检查 64 个原始候选”变为“最多完整验证 16 个已通过预检查的候选”，不是简单缩小原来的候选集合。

Uni Bugtrap 的 range 从 0.7 改为 0.30，补充终端速度幅值 0.15；Uni Wall 的 range 改为 0.5，幅值保持 0.30，满足真实 v>=0.25。两组起终 lifting 的 signed velocity 仍为原来的 0.30。终端转向提议来自平坦运动方向 `atan2(dy,dx)`，不从速度范数猜正反 branch。

转向目标使用平均速度和受限角速度构造弧段终点，仅是输出采样策略；它不被当作已满足非完整约束的轨迹。补充边仍通过原 PFF 的固定边界最优解生成，随后进行全部校验。转向安全系数配置为 0.7。

新回归测试发现，仅使用原有时间网格仍可能漏掉合法转向曲线。因此 Uni 启用 `model_time_hints`，按提议弧长 / 平均速度估计时间，并加入 `[0.9,1.0,1.1]` 倍的候选。它们经过原 scalar `fixedTime()` 求解，遵守 min/max_time；固定终端时间模式不加提示时间，扩展和重连复用同一入口。PFF/SIMD 数学文件不修改，额外 scalar 计算不计入 SIMD lanes。这些时间不一定满足无约束 H(T)=0，仍是有限候选时间的物理可行性搜索。

Quad2D Hole 对应本地真实环境的洞宽 0.6，机器人矩形尺寸 0.6×0.2；水平穿越没有正间隙。本轮在洞口中心附近增加采样区域，并给出 theta=±0.85、速度 [0,-2]、单位质量推力大小 0.35g、jerk=0 的显式终态候选：

```
a_x = -0.35 * g * sin(theta)
a_z =  0.35 * g * cos(theta) - g
```

这与现有 Quad2D DF 的符号一致。jerk=0 是指定边界条件，表示此终态零角速度；不是缺失信息自动补零。悬停/倾斜均属于额外固定终态连接，原自由终端候选保留。完整目标仍使用原 lifting，不接受提示状态替代目标。

倾斜终态的推力和几何投影可行不等于整条穿洞轨迹可行。提示参数是待实测的候选策略，未宣称解决了所有姿态或可达性问题。论文实验应分别关闭提示/引导采样做消融，不能把这些策略的收益归因于 SIMD。

## 验证记录

构建和 CTest 的本轮最终结果见 `validation_logs/planar_verification.md`。只有那里明确列出的实际检查才标记为完成。上一版虚拟机的 9/9 和 Quad3D 两组成功不能自动当作新版六环境成功。

本机不具备真实 FCL/Boost 构建环境；使用现有离线编译器和依赖，不下载/安装软件。`BoundarySpy` 仅用于检查验证调用顺序，不能作为 DynoBench 成功证明；真实测试由 `test_real_dynobench` 和独立 `check_trajectory` 完成。

新增计数中 `collision_rejected` 包括早期和密集碰撞拒绝，`dynobench_validations` 仅计完整轨迹验证。因此不能再直接用两者相除作为送检碰撞率；早期部分单独看 `collision_precheck_rejected`。

## 虚拟机运行

将本轮 zip 放入 Downloads，执行：

```bash
python3 -m zipfile -e "$HOME/Downloads/RRTstarDFPFF_planar_fix_20260920.zip" "$HOME/MYFILE/newfile/RRTstar"
cd "$HOME/MYFILE/newfile/RRTstar/RRTstarDFPFF_planar_fix_20260920"
mkdir -p logs
(
  set -e
  set -o pipefail
  python3 tools/verify_quad3d_baseline.py
  cmake -S . -B build_real_dynobench -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DRRTSTARDFPFF_ENABLE_AVX2=ON -DRRTSTARDFPFF_ENABLE_DYNOBENCH=ON \
    -DRRTPFF_DYNOBENCH_SOURCE="$HOME/MYFILE/newfile/dynobench-main" 2>&1 | tee logs/configure.log
  cmake --build build_real_dynobench --parallel 2 2>&1 | tee logs/build.log
  cmake --build build_real_dynobench --target check_trajectory --parallel 2 2>&1 | tee logs/build_checker.log
  (cd build_real_dynobench && ctest --output-on-failure) 2>&1 | tee logs/ctest.log
)
```

真实库开启时应有 11 项测试。本轮新增 `planar_search`、`quad3d_baseline`。全部通过后先验证冻结的 Quad3D：

```bash
regression_dir="$PWD/results/quad3d_regression_$(date +%Y%m%d_%H%M%S)"
python3 tools/run_six.py --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --build-dir "$PWD/build_real_dynobench" --output "$regression_dir" \
  --cases quad3d_block quad3d_window --runs 1 --timeout 30 --seed 40
python3 tools/inspect_results.py "$regression_dir"
```

确认两组独立轨迹检查通过，再运行四组二维实验：

```bash
experiment_dir="$PWD/results/planar_$(date +%Y%m%d_%H%M%S)"
python3 tools/run_six.py --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --build-dir "$PWD/build_real_dynobench" --output "$experiment_dir" \
  --cases uni_bugtrap uni_wall quad2d_bugtrap quad2d_hole --runs 1 --timeout 30 --seed 40
run_status=$?
printf '退出码：%s\n结果目录：%s\n' "$run_status" "$experiment_dir"
cat "$experiment_dir/summary.csv"
python3 tools/inspect_results.py "$experiment_dir"
```

退出码 2 表示选定实验里仍有未成功项，并不是跳过真实检查。保留 summary、详细计数、manifest 和日志；先看首解与拒绝分布，再决定下一轮修改。
