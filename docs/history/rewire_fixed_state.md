# Rewire 修改说明（2026-09-18）

修改位置：`D:/codex_talk/project_1/project_code/RRTstarDFPFF_next`。

本次约束：每次成功重连只重新求解并替换 `added -> node` 的入边，固定两端完整 FlatState。所有已有节点的 state 和 metadata 保持原值；子树内部的父子链接、轨迹系数、边代价及边持续时间保持原值，只传播累计 cost。

## 逐项检查

| 要求 | 本次处理 | 验证状态 |
|---|---|---|
| 只求解 `added -> node` | 保留 `Constraint::full(node->state)` | 源码核对完成 |
| 固定两端速度 | 二阶系统固定位置、速度 | 已编写回归测试，待 C++ 运行 |
| 四阶完整状态不变 | 位置、速度、加速度、jerk 全部固定 | 已编写逐阶端点拒绝测试，待运行 |
| 不覆盖既有节点 state / metadata | 删除 `commit()` 和 `rewire()` 中的 metadata 赋值；无 state 赋值 | 源码核对完成 |
| Unicycle branch 不匹配时拒绝 | 修改树之前调用模型的 `terminalMetadataMatches()` | 正向/倒车拒绝用例已编写，待运行 |
| 比较终点有符号速度 | 根据新边 `end` 和固定 branch 恢复速度，与目标节点 metadata 在配置容差内比较 | 起点/终点速度不同的用例已编写，待运行 |
| 匹配成功仍保留原 metadata | 容差比较仅判断可否连接，不用计算结果替换节点原值 | 精确保留原值的测试已编写，待运行 |
| 子树只更新累计 cost | 保留 `child.cost = parent.cost + child.incoming.cost` 的遍历 | 子节点、孙节点和旁支快照测试已编写，待运行 |
| 拒绝时整树不变 | 元数据检查位于所有树写操作之前 | 分支错误、缺失数据、终点速度错误的快照测试已编写，待运行 |
| 通用 core 不包含 Unicycle 公式 | 自定义 metadata 的比较交给 System Model；`std::monostate` 没有额外约束 | 源码核对完成 |

`added` 插入时的 metadata 初始化仍然必要。本次仅移除重连已有节点时的两处 metadata 覆盖。

## 修改文件和接口

- `core/rewiring.h`：新增 `commit(node, parent, edge, model)`；有实际内容的 metadata 必须通过模型验证。三参数版本仅供 `std::monostate` 使用，防止绕过状态语义检查。
- `core/rrtstar.h`：重连时传入模型，删除重连成功后对目标 metadata 的赋值。
- `systems/unicycle_model.h`：新增 `terminalMetadataMatches(edge, target_metadata)`，校验有效性、branch 和终点有符号速度。
- `tests/test_tree.cpp`：覆盖 `<2,2>`、`<2,4>`、`<4,4>` 的完整状态固定、子树 cost 传播、其他入边保持原值及成环拒绝。修正旧测试使用过短连接时间、实际不能降低 cost 的测试输入。
- `tests/test_rewire_metadata.cpp`：新增正向/倒车语义和拒绝后整树不变的测试。
- `CMakeLists.txt`：注册 `rewire_metadata` 测试。

## 验证结果和运行命令

本机未发现可用的 `cl`、`g++`、`clang++`、`nmake` 或 `ninja`，因此本次 C++ 编译和 CTest **未执行**。没有再次调用此前受阻的 WSL。以下命令用于具备 C++17、Eigen3、yaml-cpp 和 CMake 的环境：

```sh
cmake -S . -B build_rewire -DBUILD_TESTING=ON
cmake --build build_rewire --config Release --target test_tree test_rewire_metadata test_end_to_end
ctest --test-dir build_rewire -C Release --output-on-failure -R "^(tree|rewire_metadata|end_to_end)$"
```

执行过独立 Python 数值核算，确认新测试输入的代价满足改进条件。这只校验测试输入，不代表执行了 C++ 测试：

| 输入 | 重连前累计 cost | 重连后累计 cost |
|---|---:|---:|
| `<2,2>` | 375.0200 | 16.4222 |
| `<2,4>` | 50330276.1000 | 96133.1132 |
| `<4,4>` | 50346567.7000 | 96730.2613 |
| Unicycle | 4.8400 | 1.5025 |

## 备份和独立补丁

修改前文件已保存在 `backups/rewire_before_20260918_202548/`，文件名带 `.before` 后缀，避免作为源码参与构建。

`docs/rewire_fixed_state.patch` 仅包含本次修改，不包含工作区此前已有的其他差异。当前源码已经应用本次修改；补丁用于审阅或迁移到匹配的修改前版本。

本次修正不代表真实 DynoBench 六环境、完整 SIMD 或整个项目已通过验收。
