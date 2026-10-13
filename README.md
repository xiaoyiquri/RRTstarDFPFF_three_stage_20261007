# paper_r2 float32 strict 新副本

本目录由 `RRTstarDFPFF_paper_r2_noguidance_20261006` 复制建立，原目录保持不变。

主模式为 `flask_strict` / `float32_production`：无人工地图 Guidance，r=2 cubic PFF，float 状态、代价、NN、DF 和 AVX2 时间采样检查；保留 sphere union。默认启用 Algorithm 4 风格 shortcut，并分别保存 raw、shortcut、首解及边界诊断。

A′ 语义保持不变：y、dy 连接连续，Quad 姿态和 omega 只诊断。完整物理连续性及连续碰撞没有被认证。单根固定 Unicycle signed branch；未增加双 branch 森林。

- [具体实现与数学公式](docs/float32_strict_implementation.md)
- [Ubuntu 构建、运行及可视化完整命令](运行指令.md)
- [修改文件与交付范围](修改说明.md)
- [本机验证记录](docs/verification_report.md)
- [原目录完整性核验](docs/original_integrity_check.json)

`full_df` 继续使用 `legacy_mixed`；`double_reference` 仅在测试中使用。六个主配置保留原 RRT* / PFF 参数及 12 个额外时间候选。历史文档在 docs/history，现行语义以本 README 和实施说明为准。
