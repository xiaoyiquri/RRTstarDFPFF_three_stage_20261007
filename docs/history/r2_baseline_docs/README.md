# RRTstar + DF + PFF：paper_r2 / 无人工地图 Guidance

此目录为 2026-10-06 的独立修改副本。原目录没有覆盖。

默认六配置使用 `paper_fair`、`df.mode: paper_r2` 和 `guidance_probability: 0`。
Quad 的节点只存 y/dy，PFF 为三次曲线，伪控制为 ddy；物理恢复使用 acceleration、jerk 和 snap=0。
连接 y/dy 必须连续，姿态/角速度只诊断。恢复后的物理起终点也输出误差。

`benchmark/config/full_df/` 保留高阶 DF 及 boundary lifting，关闭人工地图 Guidance，并要求恢复后的物理边界一致。
Uni 原本就是二阶，保留原有 signed-velocity lifting。

详细契约、修改文件和精度说明见 [修改说明](docs/paper_r2_A_prime.md)。
实际验证记录见 [交付说明](交付说明.md)，操作命令见 [运行指令](运行指令.md)。

旧版说明存于 `docs/history/pre_r2_docs/`，旧结果/validation_logs 为复制的历史资料，不能作为本次修改后的结果。

