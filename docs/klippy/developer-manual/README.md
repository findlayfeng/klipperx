# Klipperx 开发手册

面向贡献者与模块维护者的技术参考。涵盖消息编解码（`msg`）、MCU 传输与数据字典（`mcu`）、命令层（`cmd`）、事件层（`event`）、identify 引导（`identify`）与客户端 API 层（`api`）的内部结构与设计取舍。

> **第三方 API 接口**（G-Code 命令、API 端点等）参见 [第三方开发手册](../third-party-dev/README.md)。

## 分层结构

| 层 | 路径 | 职责 | 知道哪些具体命令 |
|----|------|------|------------------|
| 编解码引擎 | `src/core/klippy/msg/` | 格式串 ↔ 字节 | **不知道**：只认 `%u` / `%.*s` |
| MCU 传输 | `src/core/klippy/mcu/` | 帧收发、`Parser`、数据字典、裸命名访问（`send` / `call`） | 只知道 `identify` 一对（起始 `Parser`） |
| 命令层 | `src/core/klippy/cmd/` | 命令词汇（`McuCommand` / `McuResponse` / `Params`）、类型化调用、各命令模块 | 全部 |
| 事件层 | `src/core/klippy/event/` | 事件词汇（`McuEvent`）、回调注册（`Mcu::bind_event`）、各事件模块 | 事件消息（当前只有 `stats`） |
| Identify 引导 | `src/core/klippy/identify.rs` | 主机自有格式、分块驱动与解压、`connect` / `identify` 入口 | `identify` 一对 |
| 客户端 API | `src/core/klippy/api/` | Unix Domain Socket、`0x03` 分帧、请求分发、端点与推送 | **不涉及**：只认客户端端点 |

`cmd`、`event`、`identify` 与 `mcu` **平级**，不是 `mcu` 的子模块：传输代码（帧、`Parser`、字典、裸 `send` / `call`）不引用任何能力，命令、事件与引导都是建立在它之上的模块。

`api` 与前五者都不同：它不在 MCU 数据通路上，而是客户端一侧的入口（对应 klipper 的 `klippy/webhooks.py`）。它不发送 MCU 消息、也不被 MCU 消息驱动，端点需要数据时向 `printer` / `gcode` 取。

MCU 一侧的依赖边一共只有这五条：

```
  msg  ←──  mcu  ←──  cmd  ←──  event
             ↑          ↑
        identify ───────┘
```

| 边 | 说明 |
|----|------|
| `cmd → mcu → msg` | 正常的向下依赖：命令层用传输，传输用编解码 |
| `event → cmd → mcu` | 事件层复用命令层的 `Params` 与类型化调用风格，不引用具体命令模块 |
| `identify → mcu` | 引导交换要通过传输收发、并把字典装进 `Parser` |
| `identify → cmd::identify` | identify 的命令定义（类型化视图、`count` 参数）也放在命令层 |
| `mcu → identify` | **唯一的反向上行边**，只有一处：构造时取起始 `Parser`（`identify::new_parser`）。`Mcu` 在认识任何消息之前必须先认识 `identify` 这一对，这个先后关系无法用分层表达，只能接受这条边（理由见 [Identify 机制](identify.md)） |

核心约束：**除 `identify` / `identify_response` 外，主机不定义任何收发命令格式**。其余格式全部来自固件在 identify 阶段下发的数据字典（见 [MCU 协议与数据字典](mcu-protocol.md)）。

### 客户端 API 一侧

`api` 不参与上面那张图，它单独构成客户端一侧的一层：

```
                     ┌─────────┐
        客户端 ─────▶ │   api   │ ──▶ printer / gcode
                     └─────────┘
```

| 边 | 状态 |
|----|------|
| `api → error` | 已有：`KlippyError` 是 `Server::bind` / `Server::run` 的错误类型 |
| `api → tokio 运行时` | 已有：accept 循环与每连接一个任务跑在 host 建的 runtime 上（见 `klippy::run`）；API 层自己不建线程，socket 收发是异步的 |
| `api → printer`、`api → gcode` | **计划中**：`info` 的 handler 仍是 `todo!()`，`objects/*` 与 `gcode/*` 尚未开始，所以这两条边还没出现在代码里 |
| `api → mcu` / `cmd` / `event` | **没有**，将来也不应该有：端点经 `printer` / `gcode` 间接使用协议层，不直接碰帧与字典 |

**并发模型**：`api` 用一个任务 accept、一个任务服务一条连接。所以跨连接并行、同连接内的请求保持顺序（客户端 pipeline 时看到的顺序与上游一致）；一个卡住的客户端只占住自己的任务。上游是一个线程 + reactor + 每连接一对 fd 回调，形状等价，只是用任务代替了 greenlet。推送给连接用的是**同步**的 `PushTarget::push`（入队 + `Notify` 唤醒该连接的任务），因此任何任务/线程都能推，不需要持有 runtime；“写不动超过 5 秒就断开”与上游的 `blocking_count` 同义。

`api` 内部自己是分层的（协议 ↔ 注册表 ↔ 连接），与 MCU 一侧的 `msg` / `mcu` / `cmd` 分法同构：底层的分帧不知道任何端点，端点也不知道字节怎么分帧。

### `src/client/` — 自带的客户端

主机自己的客户端**不在** `src/core/klippy/` 下，因为它不是主机的一部分：它是 `klipperx api` / `klipperx console` 两个子命令背后的东西，只对外说话。

| 文件 | 职责 |
|------|------|
| `client.rs` | 两个子命令的参数与入口；`--api-server` 与主机共用同一个解析 |
| `client/connection.rs` | `Connection`：分帧、`id` 分配与回收、应答按 `id` 配对并标注方法名、推送识别 |
| `client/console.rs` | 交互式会话：stdin 与 socket 同时 `select!`、本地命令、推送打印 |

它**复用** api 层的 `protocol`（分帧、请求形状）与 `address`（`ApiTarget` / `Transport`），不把协议再实现一遍：两边对分隔符或 `id` 语义若有分歧，那就不是在验证任何东西。依赖方向只有一条：`client → api`，api 层不知道客户端存在。

## 模块结构

### `msg/` — 消息编解码

| 文件 | 职责 |
|------|------|
| `mod.rs` | `Msg` 消息定义（id / name / 参数表 / 可选回调） |
| `parser.rs` | `Parser` 注册表：注册、编码、解码、按名查找、回调绑定 |
| `proto.rs` | `ArgType` / `ArgValue` / `Payload` / `PayloadParser` 编解码原语 |
| `param.rs` | `Param`（位置 / 命名参数描述，尚未接入 `encode`） |
| `error.rs` | `MsgError` / `MsgResult` |

### `mcu/` — MCU 传输与数据字典

| 文件 | 职责 |
|------|------|
| `mod.rs` | `Mcu`：构造（`new`）、收发任务、`send` / `call`、字典安装与查询、`Drop`；构造时向 `identify` 取起始 `Parser`，本身不引用任何命令 |
| `dictionary.rs` | `Dictionary`：解析固件字典、枚举展开、安装进 `Parser` |
| `pending.rs` | `PendingCalls`：同步请求/响应记账 |
| `error.rs` | `McuError`（总括）、`McuCallError`（`call` 专用） |
| `restart_method.rs` | `McuRestartMethod` 配置枚举 |

### `cmd/` — 命令层

| 文件 | 职责 |
|------|------|
| `mod.rs` | 命令词汇：`McuCommand` / `McuResponse` / `Params`，以及类型化调用 `Mcu::send_msg` / `Mcu::call_msg` |
| `allocate_oids.rs` | `allocate_oids`：预留对象 id（固件 `basecmd.c` 的 Low level allocation） |
| `config.rs` | `get_config` / `finalize_config`：配置 CRC 握手（`basecmd.c` 的 Config CRC） |
| `uptime.rs` | `get_uptime`：读 64 位固件时钟（`basecmd.c` 的 Timing and load stats） |
| `shutdown.rs` | `emergency_stop` / `clear_shutdown`：固件停机与解锁（`basecmd.c` 的 Misc commands） |
| `clock.rs` | `ClockSync` / `McuClock`：`get_clock` ↔ `clock`（**暂不参与编译**：`pub mod clock;` 在 `mod.rs` 里被注释掉，文件与测试原样保留） |
| `identify.rs` | `identify` / `identify_response` 的类型化视图（分片驱动在 `identify.rs`） |

### `event/` — 事件层

与 `cmd` 平级的目录：命令由主机发起，事件由固件发起，两者的注册与投递方式不同，因此分成两层。事件层复用命令层的 `Params`，不引用任何具体命令模块。

| 文件 | 职责 |
|------|------|
| `mod.rs` | 事件词汇：`McuEvent`，以及回调注册 `Mcu::bind_event`（底层 `Mcu::bind_callback` 在 `mcu`） |
| `stats.rs` | `stats` 事件（`basecmd.c` 的 `stats_update` 定时推送）；`register_stats_logging` 为占位订阅（只记日志） |

### `identify.rs` — Identify 引导

与 `mcu`、`cmd` 平级的单文件模块：主机侧的格式定义（`IDENTIFY_MESSAGES`）与起始 `Parser`（`new_parser`）、分块请求与拼装、zlib 解压、以及 `Mcu::connect` / `Mcu::identify` 两个入口。

identify 的命令**定义**（名称、参数、解码）与其它命令一样放在命令层；但**分片驱动**不是命令的一部分——一条 `identify` 只请求一个窗口 `offset..offset+40`，把一串这样的回应拼成负载是链路层的事——所以它与主机自有的格式定义一起留在 `identify.rs`。这也是唯一在字典存在之前运行的交换，那时 `Mcu::call_msg` 还会拒绝执行，只能走 `Mcu::call_msg_ungated`。

### `api/` — 客户端 API 层

客户端一侧的入口，对应 klipper 的 `klippy/webhooks.py`：外部工具（Fluidd / Mainsail / Moonraker 等）连上 API server，发 `0x03` 分隔的 JSON 请求。监听位置由 `-a/--api-server` 给出：默认是 Unix Domain Socket 路径（与上游一致），写成 `tcp:<host>:<port>` 则监听 TCP；**不给这个选项就不起服务**，这一点也与上游一致。线上的形状（请求/应答、无 `id` 不应答、推送模板、错误文案）以 [Klippy API 参考](../third-party-dev/api-reference.md) 为准，两边要一起改。

| 文件 | 职责 |
|------|------|
| `mod.rs` | 层的说明：监听位置、`0x03` 分帧、请求/应答与推送的形状、并发模型、模块表、待实现清单 |
| `address.rs` | `ApiTarget`：把 `--api-server` 的值解析成 socket 路径或 TCP 地址；未知 scheme（比如曾经的 `http://…:7125`）直接报错而不是当成文件名。`Transport` 也在这里：两个方向都只用它一个类型看待 socket |
| `protocol.rs` | `Framing`（粘包 / 拆包）、`Request` / `Response`、`Params` 访问器、`ApiError`、`ResponseTemplate`、`PushTarget`；不认 socket，也不认端点 |
| `registry.rs` | `Endpoint` / `MuxEndpoint` trait、`Api` 注册表与 `dispatch`、mux 的 key 选择、remote method、内建 `list_endpoints`；注册期错误单独用 `RegistrationError` |
| `server.rs` | `Listener`（两种传输）、`Server::bind` / `run`（accept 循环）、`ClientConnection`（分帧状态、发件箱、`Notify` 唤醒、关闭标志，即端点拿到的 `PushTarget`），以及每条连接的读写 `select!` 与 5 秒写超时 |
| `endpoints/` | 一个端点一个文件；目前只有 `info.rs`，且 handler 为 `todo!()`、**尚未注册**，参数与响应形状（`InfoParams` / `InfoResponse`）已定义 |

端点自己不拼应答信封：它只返回 payload 或 `ApiError`，`id` 的回显与「无 `id` 就不应答」由 `protocol.rs` 一处决定，端点无从弄错。

## 二进制

| 二进制 | 入口 | 是什么 |
|--------|------|--------|
| `klipperx` | `src/main.rs` | 项目的 CLI：`klippy`（跑主机）、`api`、`console` |
| `klippy` | `src/bin/klippy/main.rs` | 只有主机，等价于 `klipperx klippy`（名字取自上游的 `klippy.py`） |
| `klippy-client` | `src/bin/klippy-client/main.rs` | 只有客户端，等价于 `klipperx api` / `klipperx console` |

参数定义全在库里（`klippy::AppArgs`、`client::ApiArgs` / `ConsoleArgs`），二进制只做三件事：解析命令行、装日志（`logging::init`）、把错误打成一行并以退出码 1 结束。后两个二进制只装载各自那部分，因此命令行与帮助文本是干净的。

注意 `[[bin]]` 目标**共用同一个库**：单独编译 `klippy-client` 并不会少编一个依赖——这个 crate 的依赖是一套（`reqwest`、`flate2`、`libloading` 等主机要用的东西依旧会一起编），只是命令行是客户端而已。真要一个不带主机依赖的客户端，得把 api 层的 `protocol` / `address` 抽成单独的 crate，让客户端二进制只依赖它；现在没这么做。

## 目录

- [消息编解码（msg）](message-structure.md) — `Msg` / `ArgType` / `ArgValue` / `Payload`
- [Parser API 参考](parser-api.md) — 注册、编码、解码、回调绑定
- [MCU 协议与数据字典](mcu-protocol.md) — `Mcu`、`Dictionary`、类型化调用、命令层与新增命令流程
- [Identify 机制](identify.md) — 主机与 MCU 间的数据字典协商流程
- [内部架构](architecture.md) — 收发任务、合并发送、路由优先级、性能特性
- [测试](testing.md) — 测试覆盖与运行方式

---

- [← 文档首页](../../README.md)
