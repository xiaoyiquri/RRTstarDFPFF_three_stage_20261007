# Quad3D 回归测试修复与最新实测状态

独立源码副本：`RRTstarDFPFF_planar_fix_20260920_r2`。原 `planar_fix_20260920` 目录和压缩包保留。本次修改测试基础设施，不修改生产 RRT*、DF、PFF、SIMD、DynoBench adapter、benchmark 或六个实验配置。

## 为什么修改 quad3d_baseline

原测试将 Windows Zig/Clang 工具链生成的整棵树数据当作 Linux 工具链必须重现的固定答案。`std::uniform_real_distribution` 的实现和浮点运算在不同标准库/编译器之间可能不同；同一个随机种子不保证跨工具链产生完全一致的 RRT* 树。因此，原测试失败不能独立证明算法回归。用户原始失败日志没有具体差异，当前也不能断言 Linux 上的首个差异一定只是舍入误差。

现在分别编译两个独立测试程序：

1. `quad3d_reference` 使用冻结的 `search_fix_20260920` 旧核心及其全部本地 include 依赖；27 个源文件通过配置阶段 SHA-256 检查，旧 SIMD backend 也独立编译、独立链接。
2. `test_quad3d_baseline` 使用当前核心。两者在同一机器、同一工具链、相同测试参数下运行。
3. CTest 比较 scalar 和 SIMD 两种模式下的统计、节点、父节点、完整 flat state、边时间及多项式系数。整数和树结构必须完全一致；浮点容差仍为原来的 `1e-9 * (1 + abs(expected))`。
4. 首个差异输出模式、字段/节点、expected、actual；所有轨迹保存在构建目录的 `quad3d_baseline_artifacts/`。
5. 原 Windows 轨迹文件完整保留，作为附加诊断。只有同工具链旧核心与新核心匹配，测试才通过；当前核心不能为自己生成通过标准。

冻结代码仅用于测试，不进入 benchmark。冻结来源旧核心的 SHA-256 为 `3280263f7ebcfad8ae27b41a0f509890ad604199241902ad0db02f0dab55a2fd`。该测试没有 FCL，真实环境实验仍单独记录。

## 文件与检查清单

| 项目 | 本次结果 |
|---|---|
| 原版源码保留、新版另存 | 完成 |
| 生产代码和六个 YAML 不变 | 完成；见 `validation_logs/r2_integrity.json` |
| 旧核心测试副本独立编译、哈希锁定 | 完成 |
| 同工具链 scalar / SIMD 新旧对照 | 本机通过 |
| 篡改父节点、状态、系数、统计或 NaN 应失败 | 比较器自检通过 |
| 数值容差保持不变 | 完成 |
| Linux 上修复后的 CTest | 待虚拟机执行，不能用本机结果替代 |
| 六环境全部真实成功 | 未完成；当前用户实测为 3/6 |

本机使用已有 Windows Zig/Clang、Eigen、yaml-cpp 离线编译，无下载或安装。只重新构建两个回归测试及所需库，定向 CTest `quad3d_baseline` 通过；没有把这次结果写成真实 DynoBench 的 11/11。原始编译/测试日志在 `validation_logs/baseline_r2_*.log`。

## 用户虚拟机最新结果

以下来自用户提供的日志，均为 seed=40、每组 30 秒、1 次实验；不是本机执行结果，也不是统计成功率。

| 环境 | 真实验证 | 证据 |
|---|---|---|
| Quad3D Block | 1/1 | `max_jump=2.49625e-5`，起点误差 0，终点误差约 `1.25e-12` |
| Quad3D Window | 1/1 | `max_jump=2.38967e-5`，起点误差 0，终点误差约 `3.44e-13` |
| Quad2D Bugtrap | 1/1 | summary 中独立检查 1 次；`max_jump=4.44079e-5` |
| Uni Bugtrap | 0/1 | 1510 节点，最近目标距离 1.11290，`goal_valid=0` |
| Uni Wall | 0/1 | 1194 节点，最近目标距离 1.88447，`goal_valid=0` |
| Quad2D Hole | 0/1 | 3480 节点，最近目标距离 2.04901，`goal_valid=0` |

三组失败原因均为 `timeout_no_goal`，不是找到轨迹后被最终检查否定。Quad3D 新增平面预筛/缓存计数均为 0，与冻结策略相符。Quad2D Bugtrap 也应作为后续修改的回归用例。

诊断边界与下一轮改进方向：

- Uni 的几何最近邻只比较位置，`minimum_parent_candidates=8` 也按位置补足。不同航向的节点可能缺乏候选覆盖；需要结合树中位置/速度方向分布验证，不能仅从累计拒绝次数确定根因。PFF 分别耗时 18.84 秒、16.97 秒；进一步增加每个父节点的时间/终态候选会有明显代价。
- Uni Bugtrap 的 DF 拒绝集中在角速度（40420），Wall 还包含最小速度（304609）和角速度（123509）。各自 1509/1194 个目标父状态尝试都没有成功目标连接。缓存耗尽表示当前有限候选集合已拒绝，不证明不存在可行连接。
- Hole 的局部倾斜姿态提示只约束终态，不能保证进入、穿过、离开洞口的整段姿态/角速度/推力合法。DF 角加速度拒绝 55802 次、推力拒绝 19945 次，碰撞预筛拒绝 283193 次。应检查洞口附近实际节点和被拒目标轨迹，而不是继续堆积悬停终态或放宽真实约束。
- `collision_rejected` 包含预检查与密集检查，不能除以 `dynobench_validations` 当作失败率。其数值大于密集验证次数是当前统计定义所允许的。

这次测试补丁不声称解决上述三组搜索失败，也不要求为了它重复已通过的三组真实实验。

## 虚拟机增量更新

将 `RRTstarDFPFF_quad3d_baseline_patch_20260920.zip` 放入虚拟机 Downloads。只复制下列代码块，不复制 `cxt@...$` 提示符或程序输出：

```bash
python3 -m zipfile -e "$HOME/Downloads/RRTstarDFPFF_quad3d_baseline_patch_20260920.zip" "$HOME/Downloads"
cd "$HOME/MYFILE/newfile/RRTstar/RRTstarDFPFF_planar_fix_20260920"
(
  set -e
  python3 "$HOME/Downloads/RRTstarDFPFF_quad3d_baseline_patch_20260920/apply_patch.py" --project "$PWD"
  cmake -S . -B build_real_dynobench
  cmake --build build_real_dynobench --target quad3d_reference test_quad3d_baseline --parallel 2
  ctest --test-dir build_real_dynobench -R '^quad3d_baseline$' --output-on-failure -V
  ctest --test-dir build_real_dynobench --output-on-failure
)
```

已有构建目录会保留真实 DynoBench、AVX2 和 source 路径选项。补丁先校验旧文件和全部 payload，遇到未知本地修改则停止，写入前备份至 `backups/quad3d_baseline_时间戳/`。无需删除构建目录或安装软件。完整源码 r2 另行提供，已有 VM 推荐使用增量补丁。

若 Linux 新旧核心比较仍失败，保留 `build_real_dynobench/quad3d_baseline_artifacts/comparison.log` 和两个子目录的轨迹；诊断已能给出首个实际差异，不能再将其直接归为跨平台误差。

日志中的 `Command 'Real' not found`、`DF: command not found` 等来自把输出粘贴回 shell；`File exists` 来自结果目录已存在，脚本拒绝覆盖。这些不会推翻先前成功的实验。后续新实验可省略 `--output`，脚本自动生成包含微秒的新目录，或者使用 `$(date +%Y%m%d_%H%M%S_%N)`。不需要安装 shell 提示的那些无关包。
