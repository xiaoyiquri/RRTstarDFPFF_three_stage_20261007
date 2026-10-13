# 真实 DynoBench 六环境离线接入检查

日期：2026-09-20。用户要求仅使用现有环境，禁止下载软件。本轮没有下载、安装、启动包管理器或联网获取依赖；没有替换真实 DynoBench 为手写物理/碰撞代码。

## 结果：BLOCKED

已存在 Windows Zig/Clang、CMake、Ninja、Eigen 3.4.0、yaml-cpp 0.8.0 和 DynoBench 源码；但当前环境不能编译真实 DynoBench。

| 检查项 | 结果/证据 |
|---|---|
| DynoBench 原始头文件实际编译 | 失败：`dyno_macros.hpp:7` 的 `boost/stacktrace.hpp` 缺失，详见 header_check.log |
| FCL 头文件实际编译 | 失败：`fcl/fcl.h` 缺失，详见 fcl_check.log |
| FCL 开发包/源码/已编译 DynoBench | 在已检查目录未找到可用版本 |
| Boost | MATLAB 中仅找到部分 vc143 Boost DLL；没有找到所需开发头文件/stacktrace_basic 开发库，不能当作已具备可用依赖 |
| `deps/json` | 源码子模块目录为空，原 CMake 的 `add_subdirectory(deps/json)` 无法直接工作 |
| nlohmann/json 可用线索 | MATLAB 包含一份 json.hpp，可在后续评估本地复用；它不能解决 FCL/Boost 缺失 |
| 六份原始环境 YAML | 都存在；下表给出真实路径 |

DynoBench 原始 `CMakeLists.txt` 明确要求 Boost program_options、serialization、stacktrace_basic，以及 fcl、yaml-cpp、Eigen3；`robot_models_base.hpp` 和 `robot_models_base.cpp` 直接使用 FCL 几何对象与碰撞管理器，不能在保持真实 DynoBench 验证的同时跳过这一依赖。

依赖搜索覆盖当前项目、已有 dynoplan 源码、D:/PROJECT、D:/venvs、MATLAB R2024b、常用开发工具/Python 目录与 Program Files。Program Files 下部分 SQL Server 私有数据目录不可读，未继续访问这些与本任务无关的目录，因此不声称扫描了整台机器的全部文件。

## 六环境状态

| 实验 | DynoBench envs 下的原始路径 | YAML | 真实运行 |
|---|---|---|---|
| Uni Bugtrap | `unicycle1_v0/bugtrap_0.yaml` | 存在 | 未运行：依赖阻塞 |
| Uni Wall | `unicycle1_v2/wall_0.yaml` | 存在 | 未运行：依赖阻塞 |
| Quad2D Bugtrap | `quad2d_v0/quad_bugtrap.yaml` | 存在 | 未运行：依赖阻塞 |
| Quad2D Hole | `quad2d_v0/fall_through.yaml` | 存在 | 未运行：依赖阻塞 |
| Quad3D Block | `quadrotor_v0/quad_one_obs.yaml` | 存在 | 未运行：依赖阻塞 |
| Quad3D Window | `quadrotor_v0/window.yaml` | 存在 | 未运行：依赖阻塞 |

## 当前代码状态

- 此前的 Linear RRT* + PFF + SIMD 及独立 DF 测试结果仍然有效，不扩张为三个机器人的端到端验证结果。
- 本轮仅新增本报告、两个最小编译探针及日志、环境存在性记录；没有改变已通过测试的规划器、DF、SIMD 或 adapter 源码。
- 搜索进程与编译探针均已结束，没有持续轮询或后台下载任务。

继续真实接入需要本机已有且可访问的 FCL（含所需依赖）与 Boost 开发包/源码，或一个已经具备这些依赖的可访问运行环境。仅有 DynoBench GitHub 源码不等于具备可编译的 DynoBench。
