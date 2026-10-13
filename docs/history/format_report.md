# 代码格式调整报告

日期：2026-09-15

## 范围

- 已格式化活动工程中的 `.h`、`.hpp`、`.cpp`、`.cc`、`.cxx` 文件，共 61 个文件。
- `reference/` 保持原样。它是迁移基线和 SHA-256 快照，不能因为排版改变而失去对照价值。
- YAML、Markdown、CMake 配置没有做语义性重写；只保留原有配置内容。

## 调整内容

- 将同一行中的语句按分号拆开。
- 将 `std::cerr << ...; return ...;` 拆成独立行。
- 将 `try/catch` 体拆成多行。
- 保持字符串字面量、注释内容、数值、标识符、模板参数和表达式顺序不变。
- 删除格式化过程中产生的重复嵌套目录；当前没有 `core/core`、`pff/pff`、`common/common`、`benchmark/benchmark` 或 `tests/tests`。

## 验证

- 活动代码中没有残留 `;return`、`try {return` 或 `catch(...) {std::...` 紧凑模式。
- 原始 `reference/` 文件未修改。
- 本次只做排版调整，没有执行 C++ 编译；WSL 仍返回 `E_ACCESSDENIED`，本机没有可用 C++/CMake 工具链。
