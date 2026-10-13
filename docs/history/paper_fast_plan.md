# 2026-09-21 修改范围

来源：`RRTstarDFPFF_planar_fix_20260920_r2`；另存本目录，原目录与 DynoBench 源码不修改。

| 原文件 / 接口 | 本轮修改 |
|---|---|
| benchmark/config 六组 YAML | 独立论文动力学模型目录，k=8，延迟 PFF 轨迹构造，保守碰撞预筛 |
| benchmark/models/paper（新增） | VII-A 明示的速度、质量、惯量、推力与力矩约束 |
| pff/pff.h fixedTime/resultsAtTimes | 共用终端最优解代价计算；保留全部时间候选，仅在验证候选时生成轨迹 |
| core/rrtstar.h appendBatch/rewire | 候选排序后物化；rewire 仍固定完整节点状态、branch，后代仅更新 cost |
| simd/ | k=8 双精度 SoA 球-AABB 内核；AVX2 使用两组 double4 |
| systems/dyno_model.h | Flat 碰撞拒绝前置；通过者继续完整 DF/DynoBench/FCL 验证 |
| dynobench/real_adapter.* | 读取实际 B0 控制矩阵，支持归一化 wrench 模式；提取保守内接球和障碍 AABB |
| df/quad3d_df.h | 接受实际控制映射，复用单次 frame 重建；不降阶、不改四元数布局 |
| tools/run_six.py | 规划器与外部 checker 使用相同模型目录，记录哈希 |

数值标准来源：https://arxiv.org/html/2603.16059v4#S7.SS1 。论文未公布球半径、障碍球化分辨率、Uni 反向下限、完整搜索超参数。本版保留原几何和 branch 策略，不宣称严格复现 Table I。Quad3D 的向量 torque 上限按逐轴绝对值 2 Nm 实现；此解释显式记录。Quad2D 保留既有 4 m/s、8 rad/s 上限，论文 VII-A 未单独给出这两个 2D 上限。

不删除额外可行时间：用延迟构造减少重复计算，避免去掉 12 个时间后破坏窄通道可行性。scalar fixedTime 是唯一数学参考。回归比较在相同模型/配置下完成，论文模型的树不要求与旧模型相同。

验证：旧 CTest、延迟/立即 PFF 候选与树等价、k=8/tail/NaN 碰撞一致性、paper 配置和 wrench 约束、真实库本地连接与外部 checker（若依赖可用）。不得下载依赖；不能运行的实测项目如实标注。
