# Klipperx 开发手册

面向贡献者与模块维护者的技术参考。涵盖 `msg`（消息协议）模块的公开 API 用法、内部消息结构、架构设计与性能特性。

## 模块结构

`msg` 模块位于 `src/core/klippy/msg/`：

| 文件 | 职责 |
|------|------|
| `mod.rs` | 模块入口：`MsgBase` / `MsgHandler` / `MsgEntry` 消息结构 + 测试 |
| `parser.rs` | `Parser` 引擎：消息注册、发送、接收、路由 |
| `proto.rs` | 协议编解码：`Payload`、`ArgType` / `ArgValue` |
| `param.rs` | `Param` 函数调用参数 |
| `error.rs` | `MsgError` / `MsgResult` |

## 目录

- [Parser API 参考](parser-api.md) — 消息注册、发送、接收的使用说明
- [消息结构](message-structure.md) — `MsgBase` / `MsgHandler` / `MsgEntry`
- [内部架构](architecture.md) — 合并发送、路由优先级、性能特性
- [Identify 机制](identify.md) — 主机与 MCU 间的数据字典协商流程
- [测试](testing.md) — 测试覆盖与运行方式

---

- [← 文档首页](../../README.md)
