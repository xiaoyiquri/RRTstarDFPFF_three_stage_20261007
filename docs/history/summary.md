# 2026-09-21 论文参数与 PFF/SIMD 优化

- 新副本：`project_code/RRTstarDFPFF_paper_fast_20260921`；原 `RRTstarDFPFF_planar_fix_20260920_r2` 与外部 DynoBench 保持原样。
- 六 YAML 使用独立 paper 模型；Quad3D m=1、J=[.1,.1,.2]、真实 B0 wrench 映射；论文未发表细节明确保留/标记。
- PFF 精确代价排序，缓存终端解，按需生成轨迹；保留12额外时间和原时间提示。RRT*、固定全状态 rewire、branch、目标缓存保持。
- SIMD logical k=8 double；新增内接球/AABB保守拒绝；通过者仍走全部真实验证。Quad3D复用frame及控制映射。
- 本地 Release核心 CTest 12/12、Python 2/2、linear CLI烟雾1/1通过；真实机器人前端模板编译通过；真实FCL链接/六环境/外部checker未在本机执行。
- 微基准首候选物化约1.16–1.35倍；全部物化约0.957–1.076倍，不外推端到端性能。
- 修改报告与VM命令：`修改说明.md`、`docs/paper_fast_20260921.md`。无依赖下载。剩余：用户现有VM真实六环境验证。
