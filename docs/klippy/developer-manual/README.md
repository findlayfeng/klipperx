# Klipperx 开发手册

面向贡献者与模块维护者的技术参考。涵盖消息编解码（`msg`）、MCU 通信与数据字典（`mcu`）、以及功能特征层（`mcu::feature`）的 API 用法、内部结构与设计取舍。

## 分层结构

依赖方向单向向下（`mcu::feature → mcu → msg`）。特征是 `mcu` 最上层的子模块，但它不因此获得特权：下面的传输、字典与编解码代码不提任何特征。

| 层 | 路径 | 职责 | 知道哪些具体命令 |
|----|------|------|------------------|
| 编解码引擎 | `src/core/klippy/msg/` | 格式串 ↔ 字节 | **不知道**：只认 `%u` / `%.*s` |
| MCU 通信 | `src/core/klippy/mcu/` | 帧收发、数据字典、类型化消息访问 | 只知道 `identify` 一对 |
| 功能特征 | `src/core/klippy/mcu/feature/` | 消息语义、对外能力 | 全部 |

`mcu/mod.rs` 只声明 `pub mod feature;`，不引用其中的任何名字，所以反向依赖在编译期就成立不了。所以“子模块”只是目录归属，分层规则与 `mcu` / `msg` 之间一样严。

核心约束：**除 `identify` / `identify_response` 外，主机不定义任何收发命令格式**。其余格式全部来自固件在 identify 阶段下发的数据字典（见 [MCU 协议与数据字典](mcu-protocol.md)）。

## 模块结构

### `msg/` — 消息编解码

| 文件 | 职责 |
|------|------|
| `mod.rs` | `Msg` 消息定义（id / name / 参数表 / 可选回调） |
| `parser.rs` | `Parser` 注册表：注册、编码、解码、按名查找、回调绑定 |
| `proto.rs` | `ArgType` / `ArgValue` / `Payload` / `PayloadParser` 编解码原语 |
| `param.rs` | `Param`（位置 / 命名参数描述，尚未接入 `encode`） |
| `error.rs` | `MsgError` / `MsgResult` |

### `mcu/` — MCU 通信与数据字典

| 文件 | 职责 |
|------|------|
| `mod.rs` | `Mcu`：收发任务、`send` / `call`、字典安装与查询；只声明 `feature` 不引用它 |
| `identify.rs` | 主机侧唯一的格式定义：`IDENTIFY_MESSAGES` |
| `dictionary.rs` | `Dictionary`：解析固件字典、枚举展开、安装进 `Parser` |
| `codec.rs` | `McuCommand` / `McuResponse` / `Params`：类型化消息视图 |
| `pending.rs` | `PendingCalls`：同步请求/响应记账 |
| `error.rs` | `McuError`（总括）、`McuCallError`（`call` 专用） |
| `restart_method.rs` | `McuRestartMethod` 配置枚举 |

### `mcu/feature/` — 功能特征

| 文件 | 职责 |
|------|------|
| `mod.rs` | 特征层说明与再导出 |
| `identify.rs` | 引导特征：握手协议、分块抓取与解压、`connect` 入口 |
| `clock.rs` | `ClockSync` / `McuClock`：`get_clock` ↔ `clock` |

## 目录

- [消息编解码（msg）](message-structure.md) — `Msg` / `ArgType` / `ArgValue` / `Payload`
- [Parser API 参考](parser-api.md) — 注册、编码、解码、回调绑定
- [MCU 协议与数据字典](mcu-protocol.md) — `Mcu`、`Dictionary`、类型化调用、特征层与新增命令流程
- [Identify 机制](identify.md) — 主机与 MCU 间的数据字典协商流程
- [内部架构](architecture.md) — 收发任务、合并发送、路由优先级、性能特性
- [测试](testing.md) — 测试覆盖与运行方式

---

- [← 文档首页](../../README.md)
