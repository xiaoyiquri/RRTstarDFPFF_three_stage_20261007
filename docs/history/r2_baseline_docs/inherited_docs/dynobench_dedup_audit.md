# DynoBench 数据与配置去重审计

审计对象：

- `D:/codex_talk/project_1/dynoplan_review/dynobench-main`
- 本项目 `RRTstarDFPFF_paper_simd_20260922`

## 结论

本项目不需要复制 DynoBench 的环境文件、viewer 或 FCL/DynoBench 验证器。运行时现在以 `--dynobench-source` 为官方数据根目录：

```text
source/envs/<environment_file>
source/models/<robot_type>.yaml
```

唯一保留的本地模型副本是论文实验的动力学覆盖文件。它们与官方模型不是同一份数据，不能删除或自动替换。

## 已删除或合并

| 项目 | 处理 | 原因 |
|---|---|---|
| `benchmark/models/paper/quad2d_v0.yaml` | 删除 | 与 DynoBench `models/quad2d_v0.yaml` 逐字一致 |
| `dynobench.models_directory` | 删除 | 目录参数重复表达数据根；官方模型可由环境 robot type 和 `source/models` 唯一解析 |
| `simd.enabled`、`simd.width`（六个机器人 YAML） | 删除 | 本版本固定使用 AVX2 `__m256` float8；后端不是逐环境可切换项 |
| `validation.backend`（六个机器人 YAML） | 删除 | 当前机器人入口只有 `simd_discrete_float8`，由编译/运行时后端固定，不再重复配置 |
| 绘图脚本的 `--dynobench-source`、`--models-directory` | 删除 | 绘图只读取结果目录中的 `trajectory_N.yaml` 和实际 `geometry.yaml` |

## 必须保留

| 项目 | 原因 |
|---|---|
| `benchmark/models/paper/unicycle1_v0.yaml`、`unicycle1_v2.yaml` | 速度和角速度限制覆盖了官方模型，属于论文实验条件 |
| `benchmark/models/paper/quad3d_v0.yaml` | 质量、惯量和控制模式覆盖了官方模型，属于论文实验条件 |
| `geometry.robot_radius` | SIMD 球体机器人模型的显式半径，DynoBench `size` 不是本后端的同一语义 |
| `geometry.obstacle_sphere_cell` | 轴对齐盒体到球体并集的离散化分辨率 |
| `geometry.collision_margin` | 球体几何的显式安全膨胀 |
| `validation.max_step`、`max_samples`、`endpoint_tolerance` | 离散采样验证的算法参数，不等价于 DynoBench 模型的积分 `dt` |
| `geometry.yaml` | 记录本次运行真正使用的球体并集；DynoBench viewer 只知道原始盒体 |
| `plot_successful_results_paper.py` | 绘制本版本球体并集和离散结果，不是原始 DynoBench viewer 的重复输出 |

## 历史目录

`reference/`、`tests/reference/`、`docs/history/` 和 `validation_logs/` 不在 CMake 运行路径中，但包含旧版本、基准快照和实验证据。本次不删除，避免破坏可追溯性；正式打包时可将它们排除在源码包之外。

## 验证

- `paper_profile`：通过，确认 Quad2D 使用官方模型、Uni/Quad3D 使用显式论文覆盖模型。
- Python `tests.test_run_six`：2/2 通过。
- CTest：12/12 通过。
- 六环境 3 秒首解烟测：6/6 `sampled_success=1/1`，且没有 FCL、`check_trajectory` 或 DynoBench 标量调用。
