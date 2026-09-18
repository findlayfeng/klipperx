# Klipperx 开发手册

面向贡献者与模块维护者的技术参考。涵盖消息编解码（`msg`）、MCU 传输与数据字典（`mcu`）、命令层（`cmd`）、事件层（`event`）、identify 引导（`identify`）、机器的时钟与定时器（`reactor`）与客户端 API 层（`api`）的内部结构与设计取舍。

> **第三方 API 接口**（G-Code 命令、API 端点等）参见 [第三方开发手册](../third-party-dev/README.md)。

## 分层结构

| 层 | 路径 | 职责 | 知道哪些具体命令 |
|----|------|------|------------------|
| 编解码引擎 | `src/core/klippy/msg/` | 格式串 ↔ 字节 | **不知道**：只认 `%u` / `%.*s` |
| MCU 传输 | `src/core/klippy/mcu/` | 帧收发、`Parser`、数据字典、裸命名访问（`send` / `call`） | 只知道 `identify` 一对（起始 `Parser`） |
| 命令层 | `src/core/klippy/cmd/` | 命令词汇（`McuCommand` / `McuResponse` / `Params`）、类型化调用、各命令模块 | 全部 |
| 事件层 | `src/core/klippy/event/` | 事件词汇（`McuEvent`）、回调注册（`Mcu::bind_event`）、各事件模块 | 事件消息（当前只有 `stats`） |
| Identify 引导 | `src/core/klippy/identify.rs` | 主机自有格式、分块驱动与解压、`connect` / `identify` 入口 | `identify` 一对 |
| 客户端 API | `crates/klippy-api/` + `src/core/klippy/api/` | Unix Domain Socket、`0x03` 分帧、请求分发、端点与推送 | **不涉及**：只认客户端端点 |

`cmd`、`event`、`identify` 与 `mcu` **平级**，不是 `mcu` 的子模块：传输代码（帧、`Parser`、字典、裸 `send` / `call`）不引用任何能力，命令、事件与引导都是建立在它之上的模块。

`api` 与前五者都不同：它不在 MCU 数据通路上，而是客户端一侧的入口（对应 klipper 的 `klippy/webhooks.py`）。它不发送 MCU 消息、也不被 MCU 消息驱动，端点需要数据时向 `printer` / `gcode` 取。

它也是唯一**跨包**的一层：API 本身（协议、地址、服务端）在 `crates/klippy-api`，因为客户端也要用它，而客户端不能依赖主机；主机这边只剩端点。

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

### `reactor` — 机器的时钟与定时器

`Printer` 不拥有 runtime：它的时间与定时器来自一个被交给它的 reactor（`Printer::reactor()`，上游 `get_reactor()` 全树 121 处）。`reactor` 是个对象安全的 trait（`monotonic` / `register_timer` / `unregister_timer` / `call_later`）：主机把建在自己 runtime 上的 `TokioReactor` 交给它，测试交给它一个能手动拨表的 `ManualReactor`。上游那套 greenlet 的 `pause` / `completion` 在 async/await 世界里就是 Future，所以这里只剩下时间与定时器；与上游的逐项对应见 [时钟与定时器](reactor.md)。

### 客户端 API 一侧

`api` 不参与上面那张图，它单独构成客户端一侧的一层：

```
                     ┌─────────┐
        客户端 ─────▶ │   api   │ ──▶ printer / gcode
                     └─────────┘
```

| 边 | 状态 |
|----|------|
| `klippy-api → TransportError` | 已有：socket 层面的失败用自己的错误类型（`Bind` / `Connect` / `Closed` / `Io`），不认识主机的 `KlippyError` |
| `api → tokio 运行时` | 已有：accept 循环与每连接一个任务跑在 host 建的 runtime 上（见 `klippy::run`）；API 层自己不建线程，socket 收发是异步的 |
| `api → printer` | 已有：`api::register`（`src/core/klippy/api/mod.rs`）把服务器的对象与端点一次装到机器上，`objects/list`、`objects/query` 与 `webhooks` 都由它安装；必须在 bind 之前调用 |
| `api → gcode` | **计划中**：`gcode/*` 尚未开始 |
| `api → mcu` / `cmd` / `event` | **没有**，将来也不应该有：端点经 `printer` / `gcode` 间接使用协议层，不直接碰帧与字典 |

**并发模型**：`api` 用一个任务 accept、一个任务服务一条连接。所以跨连接并行、同连接内的请求保持顺序（客户端 pipeline 时看到的顺序与上游一致）；一个卡住的客户端只占住自己的任务。上游是一个线程 + reactor + 每连接一对 fd 回调，形状等价，只是用任务代替了 greenlet。推送给连接用的是**同步**的 `PushTarget::push`（入队 + `Notify` 唤醒该连接的任务），因此任何任务/线程都能推，不需要持有 runtime；“写不动超过 5 秒就断开”与上游的 `blocking_count` 同义。

`api` 内部自己是分层的（协议 ↔ 注册表 ↔ 连接），与 MCU 一侧的 `msg` / `mcu` / `cmd` 分法同构：底层的分帧不知道任何端点，端点也不知道字节怎么分帧。

### `crates/klippy-client/` — 自带的客户端

客户端**不在**本包里（`klipperx`），因为它不是主机的一部分：它只对外说话。它自成一个包，因此编译它不会编主机的东西。

| 文件 | 职责 |
|------|------|
| `lib.rs` | `klipperx api` / `klipperx console` 以及 `klippy-client` 三处的参数与入口；`--api-server` 与主机共用同一个解析 |
| `connection.rs` | `Connection`：分帧、`id` 分配与回收、应答按 `id` 配对并标注方法名、推送识别 |
| `session.rs` | `Session`：把一行输入解释成请求或本地命令、登记发出去的请求、把收到的东西变成 `Entry`；不打印任何东西 |
| `tui.rs` | 全屏窗口：三块面板、键位、行编辑与历史、日志滚动；把 `Entry` 画出来 |
| `console.rs` | 行模式：一次一行写到 stdout；管道与 `--plain` 走这条 |
| `main.rs` | `klippy-client` 二进制（另有一套二十行的日志初始化，见下） |

两个前端共用 `session.rs`，差别只在怎么画：一个写行、一个开窗。这一层切分让
行模式不需要终端也能测，而窗口里的日志不过是 `Entry` 的列表。

它**复用** `klippy-api` 的 `protocol`（分帧、请求形状）与 `address`（`ApiTarget` / `Transport`），不把协议再实现一遍：两边对分隔符或 `id` 语义若有分歧，那就不是在验证任何东西。依赖方向只有一条：`klippy-client → klippy-api`，API 不知道客户端存在。

### 主机自带的那扇窗口（`klipperx --tui`）

`klipperx klippy printer.cfg --tui`（子命令也可省）打开一个 klippy-client 窗口，
连的是这个进程自己。它**不是主机的功能，而是 `klipperx` 这个 CLI 的**：窗口是
客户端，会拖进一整个终端库，而只负责提供 API 的 `klippy` 二进制根本用不到。

```
klipperx（bin，src/main.rs）
  ├─ logging::to_window(records)   ← 主机日志改道进窗口，不再写 stdout
  ├─ Window                        ← 实现 klippy::Attachment
  └─ klippy::run(AppArgs, Some(Box::new(window)))
       ├─ Api / Server / klippy_process(config)     ← 照常
       └─ Attachment::run(api)
            ├─ tokio::io::duplex(64 KiB)
            │    ├─ 一端：klippy_api::server::serve(ClientConnection::new(api))
            │    └─ 另一端：Session::from_transport(…) → tui::run_session(session, logs)
            └─ 日志记录 (Level, String) → Entry::Log
```

边界：

- 主机库只知道 `klippy::Attachment` —— 一个「给我 API，我跑到我结束为止」的钩子
  （`Pin<Box<dyn Future>>`）。它不知道终端、客户端或 TUI 库存在，`logging` 也只发
  中立的 `(Level, String)`。两套日志等级类型在 bin 里对接：那是两边的词汇相遇的
  地方，也正因为如此，`klippy` 二进制不再链接 ratatui / crossterm
  （release 7.1 → 6.3 MB，`strings` 里一个 ratatui 都不剩），并且不再接受
  `--tui` —— 那本来就是 CLI 的选项。
- 钩子返回就意味着主机停下：附加进来的东西就是这次调用的界面。反过来，主机自己
  的停机条件（信号、配置错误）会让钩子结束：先等一个再停另一个，两者就不会互相
  矛盾。

三处不留神就会错的地方（都有测试或注释守着）：

- **日志要先改道再启动主机**，否则主机最早几行会写到窗口上；`logging::to_window`
  返回的 `WindowGuard` 用 Drop 收回改道，所以窗口关掉而主机还在跑时日志回到 stdout。
- **`record_str` 与 `record_debug` 两个钩子都要实现**：字面量消息（`info!("done")`）
  走前者，带参数的消息走后者，只实现后者会得到一屏空行。
- **终端还原放在 `TerminalGuard` 的 Drop 里**：窗口任务是被 abort 的（主机先退出
  时），futures 被丢弃不会执行后面的清理。

它是唯一**依赖不重合**的包：`cargo tree -p klippy-client` 里没有 `reqwest` / `flate2` / `libloading`（TUI 用的 `ratatui` 是它自己的），实测 debug 56.6 MB / release 3.5 MB，而 `klipperx` 是 95.4 MB。代价是 `main.rs` 里那二十行日志初始化与主机重复——为它单开一个 crate 比重复更糟。

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
| `mod.rs` | `Mcu`：构造（`new`）、收发任务、`send` / `call`、字典安装与查询、`seconds_to_clock`、`Drop`；构造时向 `identify` 取起始 `Parser`，本身不引用任何命令 |
| `object.rs` | `McuObject`：`[mcu]` / `[mcu <name>]` 作为打印机对象，以及工厂 `load_config` / `load_config_prefix`。section 只在 `PrinterObject::connect` 时才解析、开设备、跑 identify，随后把累积的配置交给固件，`get_status` 报 identify 快照 |
| `config.rs` | `ConfigBuilder`：配置期的 oid 发号器、`config` / `restart` / `init` 三张命令表、config 回调、CRC 与 `finalize_config`，以及 `configure()` 的 `get_config` 两段式握手 |
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

### `reactor.rs` — 时钟与定时器

与 `mcu` / `cmd` 都无关的单文件模块：机器的时间来源与定时器表，平级于 `printer.rs`。

| 项 | 职责 |
|------|------|
| `Reactor` trait | `monotonic` 与定时器注册 / 取消；回调收到事件时刻，返回下次唤醒时间或 `None` |
| `TokioReactor` | 主机实现：定时器是 tokio 任务，时钟是 tokio 的 `Instant` |
| `ManualReactor` | 测试实现：`advance(delta)` 手动拨表并逐个跑定时器，不需要 runtime |

### `api/` — 客户端 API 层

客户端一侧的入口，对应 klipper 的 `klippy/webhooks.py`：外部工具（Fluidd / Mainsail / Moonraker 等）连上 API server，发 `0x03` 分隔的 JSON 请求。监听位置由 `-a/--api-server` 给出：默认是 Unix Domain Socket 路径（与上游一致），写成 `tcp:<host>:<port>` 则监听 TCP；**不给这个选项就不起服务**，这一点也与上游一致。线上的形状（请求/应答、无 `id` 不应答、推送模板、错误文案）以 [Klippy API 参考](../third-party-dev/api-reference.md) 为准，两边要一起改。

主机这边是 `mod.rs`（说明 + 转出 `klippy-api` 的类型 + `register`）、`endpoints/`、`webhooks.rs` 与 `start_args.rs`。

| 文件 | 职责 |
|------|------|
| `mod.rs` | 说明主机侧与 API 的分界，把 `klippy-api` 的四个模块转出，并提供 `register`：一次把服务器这一侧（`webhooks` + 端点）装到机器上 |
| `endpoints/` | 一个端点一个文件：`info.rs`、`objects_list.rs`、`objects_query.rs`、`objects_subscribe.rs`；参数、响应形状与 handler 都在各文件里。`objects/query` 与 `objects/subscribe` 共用字段选择（`select_fields` / `status_object`） |
| `webhooks.rs` | 服务器自己的打印机对象：名字与字段对齐上游 `webhooks.get_status`，读的是机器状态 |
| `start_args.rs` | 主机启动参数（`config_file` / `log_file` / `software_version` / `cpu_info`）：上游放在 printer 上（29 处 `get_start_args`），这里归主机侧，`info` 是第一个消费者 |

API 本身在 `crates/klippy-api/src/`：

| 文件 | 职责 |
|------|------|
| `lib.rs` | crate 说明：线上的形状、监听位置、并发模型、模块表 |
| `address.rs` | `ApiTarget`：把 `--api-server` 的值解析成 socket 路径或 TCP 地址；未知 scheme（比如曾经的 `http://…:7125`）直接报错而不是当成文件名。`Transport` 也在这里：两个方向都只用它一个类型看待 socket。`DEFAULT_API_SERVER` / `NO_API_SERVER` 两个常量也在这里 —— 主机与客户端共用同一个默认值，而「空值＝不提供服务」只有主机认 |
| `protocol.rs` | `Framing`（粘包 / 拆包）、`Request` / `Response`、`Params` 访问器、`ApiError`、`ResponseTemplate`、`PushTarget`；不认 socket，也不认端点 |
| `registry.rs` | `Endpoint` / `MuxEndpoint` trait、`Api` 注册表与 `dispatch`、mux 的 key 选择、remote method、内建 `list_endpoints`；注册期错误单独用 `RegistrationError` |
| `server.rs` | `Listener`（两种传输）、`Server::bind` / `run`（accept 循环）、`ClientConnection`（分帧状态、发件箱、`Notify` 唤醒、关闭标志，即端点拿到的 `PushTarget`），以及每条连接的读写 `select!` 与 5 秒写超时 |
| `error.rs` | `TransportError`：socket 层面的失败（`Bind` / `Connect` / `Closed` / `Io`），与请求层面的 `ApiError` 分开 |

端点自己不拼应答信封：它只返回 payload 或 `ApiError`，`id` 的回显与「无 `id` 就不应答」由 `protocol.rs` 一处决定，端点无从弄错。

## 二进制

| 二进制 | 入口 | 是什么 |
|--------|------|--------|
| `klipperx` | `src/main.rs` | 项目的 CLI：跑主机（默认，也写作 `klippy`）、`api`、`console` |
| `klippy` | `src/bin/klippy/main.rs` | 只有主机，等价于 `klipperx klippy`（名字取自上游的 `klippy.py`）；没有 `--tui`，不链接客户端与终端库（release 6.3 MB vs `klipperx` 7.6 MB） |
| `klippy-client` | `crates/klippy-client/src/main.rs` | 只有客户端，等价于 `klipperx api` / `klipperx console`；**自成一个包**，不编主机 |

`klipperx` 的顶层参数里嵌着一份 `AppArgs`（`Option<AppArgs>`，与 `klippy` 子命令同一类型、`args_conflicts_with_subcommands` 保证两者不能混用），所以不带子命令时 `klipperx printer.cfg` 就是 `klipperx klippy printer.cfg`。那个 `Option` 不是为了可空：clap 只有在整组参数可选时才会放过组内必填项（配置文件），否则 `klipperx api …` 会来要一个它根本不需要的配置文件。

参数定义全在库里（`klippy::AppArgs`、`klippy_client::{ApiArgs, ConsoleArgs}`），二进制只做三件事：解析命令行、装日志、把错误打成一行并以退出码 1 结束。后两个二进制只装载各自那部分，因此命令行与帮助文本是干净的。

`klippy` 与 `klipperx` 在同一个包里，共用一套依赖；`klippy-client` 在另一个包里，只依赖 `klippy-api` 与 clap / serde_json / tokio / tracing，所以它既不会编 `reqwest` / `flate2` / `libloading`，产物也小得多（实测 debug 49.7 MB vs 95.4 MB）。

## 目录

- [消息编解码（msg）](message-structure.md) — `Msg` / `ArgType` / `ArgValue` / `Payload`
- [Parser API 参考](parser-api.md) — 注册、编码、解码、回调绑定
- [MCU 协议与数据字典](mcu-protocol.md) — `Mcu`、`Dictionary`、类型化调用、命令层与新增命令流程
- [Identify 机制](identify.md) — 主机与 MCU 间的数据字典协商流程
- [时钟与定时器（reactor）](reactor.md) — 机器的时钟、定时器契约，以及与上游 reactor/greenlet 的对应
- [内部架构](architecture.md) — 收发任务、合并发送、路由优先级、性能特性
- [测试](testing.md) — 测试覆盖与运行方式

---

- [← 文档首页](../../README.md)
