# 下一阶段任务逐项检测清单

输出副本：`RRTstarDFPFF_next`。状态含义：DONE=已完成并有源码依据；PARTIAL=部分实现或未运行；NOT_DONE=尚未实现；BLOCKED=外部工具/依赖阻塞。

| 编号 | 要求 | 状态 | 证据/缺口 |
|---:|---|---|---|
| 1 | 必须先确认当前仓库的实际状态 | **DONE** | 源码、reference、DF prototype、DynoBench布局和环境文件已审计 |
| 2 | 总体修改原则 | **PARTIAL** | 顺序与分层已写入实现；后续阶段尚未全部验证 |
| 3 | 第一阶段任务：先彻底验收当前 Phase 1 | **PARTIAL** | 2026-09-20 Windows Release 构建与 7/7 CTest 已通过；旧 OMPL 全实验等价性尚未验收 |
| 4 | 第二阶段：重构 System Abstraction | **PARTIAL** | SystemModel 模板接口已新增，三个完整模型仍未接入 RRT* |
| 5 | SystemModel 的职责 | **PARTIAL** | 职责边界已定义，物理/Dyno验证仍不完整 |
| 6 | 编译期维度原则 | **DONE** | FlatState 编译期维度原则保留 |
| 7 | 必须重新明确 Physical State 与 Flat State | **PARTIAL** | PhysicalTrajectory 和 Flat/Physical 分层已新增 |
| 8 | DifferentialFlatness 基础接口 | **DONE** | 统一 DifferentialFlatness 接口已新增 |
| 9 | Uni DF | **PARTIAL** | Unicycle signed lifting、branch、零速拒绝与 rewire metadata 测试已运行；真实环境待接通 |
| 10 | Quad2D DF | **PARTIAL** | 无阻力 Quad2D DF 测试已运行；真实 DynoBench 环境待接通 |
| 11 | Quad3D DF | **PARTIAL** | Quad3D 解析 Rdot/Rddot、hover/tilted 状态控制测试已运行；真实环境待接通 |
| 12 | Quad3D inverse DF 必须严格根据 DynoBench | **PARTIAL** | 轴向/四元数审计完成；完整高阶控制验证未完成 |
| 13 | DynoBench Adapter | **PARTIAL** | adapter seam 和三个参考兼容 adapter 已新增，尚未链接真实DynoBench |
| 14 | DynoBench ground truth | **PARTIAL** | finite/step接口存在；真实DynoBench ground truth 尚未运行 |
| 15 | Quad3D 特别要求 | **PARTIAL** | 状态布局和 quaternion x,y,z,w 已记录，电机/动力学一致性未验收 |
| 16 | Physical trajectory | **DONE** | PhysicalTrajectory 类型已新增 |
| 17 | PFF 当前代码保留，但需要调整接口 | **DONE** | 现有 PFF 保留并与 DF/validation 分离 |
| 18 | PFF 的数学定义必须统一 | **DONE** | J=integral(1+u^T R u) 已统一 |
| 19 | Terminal Constraint | **DONE** | output/full mask 已保留 |
| 20 | PFF 与 quintic 的关系必须重新明确 | **DONE** | quintic 与 PFF 的区别已明确 |
| 21 | RRT* Core 改造 | **PARTIAL** | Generic RRT* 保留；LocalPlanner 尚未接入主循环 |
| 22 | 必须新增 LocalPlanner 层 | **PARTIAL** | LocalPlanner 已新增，尚未替换 RRTstar 控制流 |
| 23 | LocalPlanResult | **PARTIAL** | LocalPlanResult 已新增，Model 类型契约仍需整理 |
| 24 | RRT* Expansion 流程 | **PARTIAL** | Linear 已接入 SIMD PFF→Top-K→SIMD 预筛→连续验证；三个机器人主循环未接通 |
| 25 | Rewire 流程 | **PARTIAL** | 完整端点固定、branch 不变、后代边不变与代价传播回归通过；真实物理验证待接入 |
| 26 | RRT* 的 Node 不得被重新设计成 Physical State Node | **DONE** | Node 仍只保存 Flat State 和 FlatTrajectory |
| 27 | Sampling 改造 | **PARTIAL** | uniform/goal bias 保留，通用周期 metric 的 scalar/SIMD 对比通过；Quad yaw sampling 未接通 |
| 28 | Nearest / Near 改造 | **DONE** | Model提供距离，Nearest/Near无系统分支 |
| 29 | Flat Metric 必须独立 | **DONE** | 距离由Model提供 |
| 30 | Pruning | **PARTIAL** | Linear bound保留；三个系统尚无可证明bound |
| 31 | SIMD 最终目标 | **PARTIAL** | AVX2 PFF/FlatEval/NN/预筛已接入 Linear；DF SIMD 和六环境未完成 |
| 32 | SIMD backend | **DONE** | 独立 AVX2 内核、CPUID/OS 状态检查、运行时关闭与 fallback 已实现 |
| 33 | SIMD PFF | **PARTIAL** | 能量二次型和时间根批处理；端点/costate/完整轨迹由 scalar reference 重建 |
| 34 | SIMD 与 Scalar 一致性要求 | **DONE** | 2D 二阶、2D 四阶、4D 四阶，多 mask/非对角 R/1..8 尾部/所有时间候选测试通过 |
| 35 | SIMD FlatTrajectory | **DONE** | 真实 AVX2 Horner SoA，state/control 分离与 scalar 一致性测试通过 |
| 36 | SIMD Collision | **DONE** | 圆/球廉价预筛接入 Linear RRT*，连续最终验证保留；薄障碍回归通过 |
| 37 | 最重要的性能结构 | **PARTIAL** | SoA→批量评估→Top-K→预筛→完整验证已运行；FlatTrajectory 延迟生成尚未实现 |
| 38 | Benchmark 工厂化 | **PARTIAL** | benchmark框架有Linear CSV，六系统未接通 |
| 39 | 六个 YAML | **PARTIAL** | 六个YAML已新增，但当前runner明确拒绝非linear |
| 40 | 环境名称必须以 DynoBench 实际文件确认 | **DONE** | 六个目标DynoBench路径已核对并记录 |
| 41 | DynoBench 环境 Adapter | **PARTIAL** | 参考兼容adapter存在，真实DynoBench API尚未链接 |
| 42 | 六环境最终调用 | **NOT_DONE** | 六个命令尚未实际运行 |
| 43 | Benchmark 指标必须保留 | **PARTIAL** | 主要指标字段存在，拒绝分类尚未完整 |
| 44 | 最终输出的轨迹必须是真实保存的轨迹 | **DONE** | Node incoming edge直接恢复，没有为输出重求解 |
| 45 | 物理轨迹验证 | **NOT_DONE** | 完整DF inverse到DynoBench路径尚未运行 |
| 46 | 重要：DynoBench 是最终 ground truth | **NOT_DONE** | 没有宣称已完成ground truth |
| 47 | 单元测试必须扩展 | **DONE** | 7 组 CTest 全部运行通过，包含新 SIMD/固定 workload e2e 回归 |
| 48 | DF Test | **PARTIAL** | 现有 Uni/Quad DF 测试运行通过；未覆盖真实六环境完整动态可行性 |
| 49 | Quad3D DF Test | **PARTIAL** | hover/tilted 解析姿态控制与 lifting 测试运行通过；完整实机模型验证未完成 |
| 50 | SIMD Test | **DONE** | AVX2 实际激活，lane/tail/roots/NN/periodic metric/预筛等回归通过 |
| 51 | End-to-End Test | **PARTIAL** | Linear scalar/SIMD 固定工作量 e2e 与树内每条边复验通过；Uni/Quad e2e 待完成 |
| 52 | 结果正确性优先于性能 | **PARTIAL** | 当前 Linear SIMD 结果一致性实测通过；六系统矩阵未接通 |
| 53 | 最终 Baseline | **PARTIAL** | 新增 scalar/SIMD 固定迭代、同种子分项 ablation；旧 OMPL baseline 未编译比较 |
| 54 | 必须区分三类优化 | **DONE** | 算法改进/计算加速/基线已在文档中区分 |
| 55 | RRT* 理论问题 | **DONE** | 未声称Top-K+DF+SIMD继承完整渐近最优性 |
| 56 | 对 PFF 的理论限制 | **DONE** | PFF约束限制已明确 |
| 57 | 禁止事项 | **DONE** | 禁止事项已遵守；未链接prototype bridge |
| 58 | 与当前仓库的文件迁移原则 | **DONE** | 迁移原则和副本快照已建立 |
| 59 | CMake 修改 | **PARTIAL** | CMake Release/AVX2 构建实际通过；真实 DynoBench 依赖尚未接入 |
| 60 | 版本与依赖 | **PARTIAL** | C++17/Eigen/yaml-cpp/SIMD fallback已声明，DynoBench依赖未配置 |
| 61 | SIMD backend 必须运行时可关闭 | **DONE** | simd.enabled 可运行时切换，并提供 pff/NN/screen 分项开关 |
| 62 | 公平 Benchmark | **NOT_DONE** | 六系统公平benchmark尚未执行 |
| 63 | SIMD speedup 必须单独测 | **PARTIAL** | Linear 五圆场景分项加速比已有实测与 CSV，不能推广为六环境结果 |
| 64 | DF overhead 也必须测 | **NOT_DONE** | 没有DF/Dyno overhead实测 |
| 65 | 最终性能架构 | **PARTIAL** | 数据流接口已分层，实际闭环未接通 |
| 66 | 最终类关系 | **PARTIAL** | RRTstar 已注入通用 SIMD_PFF 与 model screen hook；LocalPlanner 类型迁移尚未完成 |
| 67 | 三个系统的最终结构 | **PARTIAL** | 三类模型头文件已新增，尚无真实环境/adapter闭环 |
| 68 | 最终数学链 | **PARTIAL** | 数学链已体现在DF/PFF接口，未完成端到端运行 |
| 69 | 具体开发顺序 | **PARTIAL** | Phase 1 与当前 DF/树/SIMD 测试完成构建运行；三机器人闭环未完成 |
| 70 | 每完成一个阶段必须输出 | **PARTIAL** | 本文件和README记录修改/命令/失败项；后续阶段待输出 |
| 71 | 每次修改后必须建立验证闭环 | **PARTIAL** | 本轮 Linear/DF/SIMD 构建与测试形成闭环；六环境验证仍缺失 |
| 72 | 最终成功标准 | **NOT_DONE** | 六环境未实际执行 |
| 73 | 最终代码质量要求 | **PARTIAL** | 核心分层和fallback代码已写，真实系统未完成 |
| 74 | 当前仓库最重要的修改优先级 | **DONE** | P0-P8优先级已写入修改说明并按P0/P1推进 |
| 75 | 最终不要破坏当前已经正确的部分 | **DONE** | Linear/PFF/树/root solver/reference均保留 |
| 76 | 最重要的最终判断标准 | **DONE** | 以code+compile+unit+e2e+benchmark为完成标准 |
| 77 | 开始执行前必须先做的工作 | **DONE** | 前置审计表和布局表已形成 |
| 78 | 最终实现原则 | **DONE** | RRT*→PFF→SIMD→DF→DynoBench原则保留 |

## 历史验证事实（2026-09-20 的新记录优先）

- 原始 `RRTstarDFPFF` 和其 `reference/` 未修改。
- `RRTstarDFPFF_next/reference` 的 20 个原文件 SHA-256 应继续与上一份清单一致。
- Python PFF 推导审计已覆盖 27 个组合；这不等价于 C++ 编译。
- 本次复查：`WslService` 为 `Running/Manual`，但 `wsl --status` 和 `wsl -l -v` 均返回 `E_ACCESSDENIED`，且没有可访问的发行版；本机仍缺少 cmake/g++/clang++/cl/ninja。因此不能伪造 CMake、CTest 或六环境运行结果。
- 当前 benchmark 对 `system != linear` 会明确报错，六个 YAML 是配置骨架，不宣称已经支持六环境。

## DF follow-up audit (2026-09-18)

本轮针对 `df/` 要求完成了以下源码级修正：

- `FlatState<Dim, Order>` 只暴露 `0 ... Order-1` 阶；`FlatControl<Dim>` 承载第 `Order` 阶导数。
- `UnicycleLift::signed_velocity`、`Quad2DLift` 和 `Quad3DLift` 均为显式 lifting 输入；缺失 lifting 信息不会静默补零。
- Unicycle edge 使用 `UnicycleTrajectoryMetadata` 保存固定 Forward/Reverse branch；零速、branch 切换和起点速度不一致都会拒绝。
- PFF result、RRT node、tree edge 和 SIMD batch 已增加 Metadata 模板参数，避免 Unicycle branch 在传递过程中丢失。
- Quad2D 采用无阻力解析逆变换；Quad3D 使用 DynoBench 的 `(p, qx, qy, qz, qw, v_world, omega_body)` 布局和解析 `Rdot/Rddot`，不使用有限差分。
- 已增加 DF 单元测试中的 Unicycle branch propagation、Quad2D hover 和 Quad3D analytic attitude/control smoke cases。

这些项目目前属于 **源码完成、编译验证 BLOCKED**：本机没有可用的 C++ 编译器/NMake/Ninja，WSL 命令返回 `E_ACCESSDENIED`，因此不能把上述源码审计报告为 C++ 构建通过。

## SIMD 实测更新（2026-09-20）

已使用项目临时目录中的 Zig 0.13.0 / Clang 18.1.6、CMake 4.4、Eigen 3.4.0、yaml-cpp 0.8.0 完成 Windows Release 构建。AVX2 构建 CTest 为 7/7 通过；`Compiled AVX2=1 runtime AVX2=1`。此前“没有编译器、未运行 C++”的说明属于历史状态，不再适用于本轮。

范围边界：`dynobench` 测试名对应参考 adapter 测试；没有链接真实 DynoBench。因此仍不能宣称六环境成功。

完整记录见 [SIMD 实施说明](simd_implementation_20260920.md)，结果数据位于 `results/simd_comparison*/`。

补充：编译完全关闭 AVX2 内核后，同样 7/7 CTest 通过；六环境仍未完成。
