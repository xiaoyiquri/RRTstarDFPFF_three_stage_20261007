# 本次交付摘要

目标：另存论文式 float8 离散物理检测版本，不覆盖原代码。

完成：完整时间批次 DF/状态/控制/球体碰撞，移除 Top-K 与运行时标量/FCL/DynoBench 验证；保留 PFF 与固定状态 rewiring；六 YAML 和运行脚本更新。

验证：C++ Release 构建成功；CTest 12/12；seed41 六环境首解 6/6，持续优化六环境 6/6 且均有 rewiring。214 个原文件 SHA-256 一致。没有下载软件。

限制：本机 Windows 验证，虚拟机尚未复跑；仅新球体模型和 5 ms 离散判据，不是旧 FCL 几何结果、连续认证或 20 次性能统计。详细证据见交付说明及 validation_logs。

## DynoBench 数据去重审计（2026-09-28）

- `benchmark/models/paper/quad2d_v0.yaml` 与 DynoBench 官方 `models/quad2d_v0.yaml` 逐字一致，已从新版本删除；运行器现在直接使用 `dynobench-source/models/quad2d_v0.yaml`。
- `unicycle1_v0.yaml`、`unicycle1_v2.yaml`、`quad3d_v0.yaml` 与官方模型存在论文实验参数差异（速度/角速度或质量、惯量、控制模式），仍保留，但通过配置中的 `dynobench.model_file` 明确标识为实验覆盖，不再使用目录级 `models_directory` 参数。
- `dynobench/data_adapter.*` 不是 DynoBench 验证器的重复实现；它只读取 YAML，并把原始盒体转换成当前 float8 离散验证使用的球体并集。`geometry.yaml`、`robot_radius`、`obstacle_sphere_cell`、`collision_margin` 仍是该验证后端的必要参数。
- `plot_successful_results_paper.py` 仍保留，因为 DynoBench viewer 直接绘制原始盒体，而本版本结果必须绘制实际使用的球体并集；已移除它不使用的外部模型路径参数。
- `reference/`、`tests/reference/`、`docs/history/`、`validation_logs/` 是历史/证据材料，不在 CMake 运行路径中；本次不删除，以免丢失复现实验记录。
