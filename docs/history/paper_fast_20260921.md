# 论文参数与 PFF / SIMD 优化版（2026-09-21）

这是 `RRTstarDFPFF_planar_fix_20260920_r2` 的独立副本。原工程和外部 DynoBench 不修改。参数来自 [Ultrafast Sampling-based Kinodynamic Planning via Differential Flatness，v4，VII-A](https://arxiv.org/html/2603.16059v4#S7.SS1)。本版仍使用 RRT*，不是论文 FLASK-RRTConnect 的完整复现；新物理参数下的轨迹不能与旧质量/惯量下的轨迹要求逐点相等。

## 配置与物理控制

| 系统 / 两个环境 | 本版实际设置 | 说明 |
|---|---|---|
| Uni / Bugtrap、Wall | 最大线速度 1 m/s；角速度范围 ±1.5 rad/s | 保留两环境原 min_vel=-0.5 / 0.25；论文没有公布下限。端点 lifting 和每边固定 branch 保留 |
| Quad2D / Bugtrap、Hole | m=0.034 kg，I=0.0001 kg m²，l=0.1 m | 每电机 f_i≤0.65mg；DynoBench u_i≤1.3，因 f_i=mg u_i/2 |
| Quad3D / Block、Window | m=1 kg，J=diag(0.1,0.1,0.2) kg m²；速度范数≤4 m/s，角速度范数≤8 rad/s | 总推力≤1.5mg；逐轴绝对力矩≤2 N m |

Quad3D 不再用 Crazyflie 的小惯量和独立电机上限去近似论文的总推力/力矩限制。实际 DynoBench `motor_control: false` 已实现：`[f,τx,τy,τz]=B0*u`。本地源码中的 B0=diag(mg,20,20,20)，所以模型 YAML 设置 u=[0..1.5, ±0.1, ±0.1, ±0.1]。Adapter 读取库实例的真实 B0、检查可逆并缓存其逆；DF 接收这份映射，不在 DF 中猜控制归一化。真实集成测试检查 B0*u 上限和非零力矩 step()。

论文向量 torque 上限未注明范数，本版明确采用逐轴绝对值上限；不把这个解释当作作者额外确认。Quad2D 速度/角速度沿用现有 4/8，VII-A 没有单独公布其值。两个 Quad 的额外 DF 加速度/角加速度上限改为显式 `.inf`，通过实际推力、力矩/电机以及速度约束判定，而非附加未公布的 20/25 上限。模型 YAML 中保留的 max_acc / max_angular_acc 是上游启发式时间下界参数，不是该库的真实 state/control 上限；本工程不将它们额外用作 DF 验收阈值。无阻力。

模型文件位于 `benchmark/models/paper/`，六个 YAML 的 `dynobench.models_directory` 指向这里。`run_six.py` 给规划器与外部 `check_trajectory` 传同一目录，并在 manifest 中记录模型、环境、配置和二进制 SHA256。外部原模型目录不修改。

## PFF：保持数学与候选集，按需生成轨迹

仍求解 J=T+∫uᵀRu dt，六组 R=I；扩展固定 flat output、部分终端导数自由；rewire/目标连接固定完整终端状态。Gramian、costate、H(T)=0 根的数学定义保持一致。额外 12 个固定时间解仍保留，它们是物理可行性候选，不能称为都满足 H(T)=0 的自由时间最优根。

`terminalSolution()` 是共享参考计算。`defer_trajectories: true` 时：

1. 保留原根、区间端点、12 个额外时间和 Unicycle 原时间提示，计算同一精确终端解和 edge cost，并缓存 eta 与能量。
2. 以 parent cost + edge cost 排序。
3. 候选被实际检查时才 `materialize()`，复用缓存终端解生成多项式、检查端点，并附加 branch 元数据。
4. 通过 cheap screen / DF / DynoBench / FCL 后才入树或重连。重连不修改既有节点状态，后代只传播累计代价。
5. 目标连接缓存保留已生成轨迹；未访问时间候选不再提前分配多项式向量。

完整候选集合不删减，避免为速度牺牲窄通道可行解。数值无效候选在生成轨迹时仍被拒绝，不能占掉后续合法候选的 Top-K 配额。`solve()` / `evaluateBatch()` 始终返回已生成轨迹；`candidates()` 在延迟模式下允许返回 `deferred=true` 的候选，调用者必须先物化再读取 trajectory，且不得篡改未物化候选的 boundary/T/缓存字段。显式更换起点时会重新求解，避免使用旧起点缓存。该契约已接入扩展、目标缓存、时间提示和重连。

初版微基准发现重复终端求解会使“全部候选物化”退化，已通过缓存同一终端解消除此重复求解。全部候选都需物化时仍有缓存/排序开销；提供 `--eager-pff` 作相同模型下的对照，附微基准同时测首候选与全部候选访问，不保证每个问题都加速。

## SIMD 与验证

`SIMDBatchWidth=8` 统一 PFF/core 候选批次。论文式碰撞路径另用 `__m256`
处理八个 `float` 时间采样点：多项式位置求值和 AABB 屏幕都走真正的
AVX2 float8 内核。PFF 代价、候选排序和真实 DF/DynoBench 仍保留 double
参考路径；这两个批处理轴不能混为一个 SIMD 宽度。尾部安全处理；非
AVX2 主机自动回退到 scalar float/double 实现。未开启 fast-math。

新增 robot inner sphere 与障碍 AABB 的 SIMD 拒绝：球严格在已知单个机器人几何内部，且中心为该模型实际碰撞中心。只要内接球穿入障碍，就能拒绝轨迹；内接球未碰撞不能证明整个机器人无碰撞。球半径略缩以避免数值接触误拒绝；不支持的几何跳过此筛选。保留现有 box/sphere 真实几何，未猜测论文未公布的障碍球化和机器人球半径。

碰撞屏幕按离散均匀时间采样检查：八个连续采样点一批，任意采样点的
内接球穿入 AABB 即拒绝；通过只表示在这些采样点没有发现碰撞。通过者
仍完整重建 DF，检查真实 state/control bounds、DynoBench step 残差和
FCL；最终拼接轨迹及导出再次验证，外部 checker 独立验证。采样间隔仍
为0.005 s、动力学/端点容差仍0.001。它们是采样检查，不声称连续碰撞
证明。

Quad3D `sampleFeasible()` 复用同一 frame 的 R/Rdot/Rddot 生成状态和控制；真实桥接路径的控制矩阵逆只算一次。旧默认电机模式作为测试参考仍可用。

`results.csv` 增加 `pff_materializations`（成功生成的完整候选轨迹数，eager/lazy 都计数）和 `simd_collision_samples`（真正由向量指令计算的位置样本数）；`screen_rejected` 记录 cheap screen 拒绝，`collision_checks` 仍是真实库碰撞调用，不能混算。各计时列有包含关系，不应直接相加。

## 编译与六环境运行：用户现有虚拟机

解压新目录到 `$HOME/MYFILE/newfile/RRTstar/`。只使用你已有依赖；不执行安装/下载。不要向旧实验目录覆盖；不运行旧 `apply_patch.py` 或旧生产文件冻结哈希脚本。本版参数有意更新；CTest 的 `quad3d_baseline` 用独立旧实现比较默认旧模型，仍保留。

```bash
cd "$HOME/MYFILE/newfile/RRTstar/RRTstarDFPFF_paper_fast_20260921"
mkdir -p logs
(
  set -e
  set -o pipefail
  cmake -S . -B build_real_dynobench -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DRRTSTARDFPFF_ENABLE_AVX2=ON -DRRTSTARDFPFF_ENABLE_DYNOBENCH=ON \
    -DRRTPFF_DYNOBENCH_SOURCE="$HOME/MYFILE/newfile/dynobench-main" 2>&1 | tee logs/configure.log
  cmake --build build_real_dynobench --parallel 2 2>&1 | tee logs/build.log
  cmake --build build_real_dynobench --target check_trajectory --parallel 2 2>&1 | tee logs/checker_build.log
  ctest --test-dir build_real_dynobench --output-on-failure 2>&1 | tee logs/ctest.log
)
```

全部通过后，先运行六组各一次（seed 与之前成功记录相同，仍优化到60秒；重点读 first_solution_time）：

```bash
experiment_dir="$PWD/results/paper_$(date +%Y%m%d_%H%M%S_%N)"
python3 tools/run_six.py \
  --dynobench-source "$HOME/MYFILE/newfile/dynobench-main" \
  --build-dir "$PWD/build_real_dynobench" --output "$experiment_dir" \
  --runs 1 --timeout 60 --seed 41
run_status=$?
printf 'exit=%s\nresults=%s\n' "$run_status" "$experiment_dir"
if [ -f "$experiment_dir/summary.csv" ]; then cat "$experiment_dir/summary.csv"; fi
```

需要首解后停止时，在脚本命令后加 `--first-solution`。其结果长度是首解长度，不能与优化60秒后的最终长度混比。独立验证在停止搜索后仍执行。需与论文 anytime 的180秒预算比较时改 `--timeout 180`；RRTConnect 和 RRT*、几何模型与硬件差异仍须报告。

同模型 A/B：每次选择全新 output，保持 seed/runs/timeout 相同；分别追加 `--eager-pff`、`--scalar`、`--no-collision-screen`。这三个开关独立，不能一次全关后把总差异归因于单一模块。微基准：

```bash
./build_real_dynobench/benchmark_pff > logs/pff_microbenchmark.csv
```

首解时间、最终轨迹长度仍在各 case 的 results.csv；轨迹/绘图脚本接口未改变。旧图不能代表此版更换动力学后的实验结果。

## 验证边界

本机 Windows 缺少真实 FCL 构建依赖；WSL 返回访问拒绝。本轮不安装依赖、不持续轮询；本地离线构建检查通用核心和真实前端模板，新版真实 adapter 链接与六组实测仍需上述 VM 验证。具体本地已通过项、日志与微基准数值见根目录 `修改说明.md`；历史 validation_logs 不作为本轮通过证据。
