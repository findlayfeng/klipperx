# Klipperx 开发手册

面向贡献者与模块维护者的技术参考。涵盖消息编解码（`msg`）、MCU 传输与数据字典（`mcu`）、MCU 配置构建（`ConfigBuilder`）、引脚解析（`pins`）、G-Code 调度（`gcode`）、命令层（`cmd`）、事件层（`event`）、identify 引导（`identify`）、机器的时钟与定时器（`reactor`）、机器的骨架与装载（`printer` / `load`）与客户端 API 层（`api`）的内部结构与设计取舍。

> **第三方 API 接口**（G-Code 命令、API 端点等）参见 [第三方开发手册](../third-party-dev/README.md)。

## 分层结构

| 层 | 路径 | 职责 | 知道哪些具体命令 |
|----|------|------|------------------|
| 机器 | `src/core/klippy/printer.rs` + `load.rs` | 对象注册表、事件总线、状态与生命周期；`load.rs` 按 `section!` 生成的工厂表把 config 装成对象 | **不涉及**：只先注册 `pins` / `gcode` |
| 编解码引擎 | `src/core/klippy/msg/` | 格式串 ↔ 字节 | **不知道**：只认 `%u` / `%.*s` |
| MCU 传输 | `src/core/klippy/mcu/` | 帧收发、`Parser`、数据字典、裸命名访问（`send` / `call`） | 只知道 `identify` 一对（起始 `Parser`） |
| 命令层 | `src/core/klippy/cmd/` | 命令词汇（`McuCommand` / `McuResponse` / `Params`）、类型化调用、各命令模块 | 全部 |
| 事件层 | `src/core/klippy/event/` | 固件事件词汇（`McuEvent`）、回调注册（`Mcu::bind_event`）；打印机级事件词汇（`KlippyEvent`，由 `build.rs` 生成） | 事件消息（当前 `stats` / `shutdown`）、打印机事件声明 |
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

`Printer` 不拥有 runtime：它的时间与定时器来自一个被交给它的 reactor（`Printer::reactor()`，上游 `get_reactor()` 全树 119 处调用）。`reactor` 是个对象安全的 trait（`monotonic` / `register_timer` / `unregister_timer` / `call_later`）：主机把建在自己 runtime 上的 `TokioReactor` 交给它，测试交给它一个能手动拨表的 `ManualReactor`。上游那套 greenlet 的 `pause` / `completion` 在 async/await 世界里就是 Future，所以这里只剩下时间与定时器；与上游的逐项对应见 [时钟与定时器](reactor.md)。

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
| `api → gcode` | 已有：`gcode` 调度器（G1，`src/core/klippy/gcode.rs`）与五个 `gcode/*` 端点（G3，`api/endpoints/gcode.rs`，按请求查找 `gcode`），含 `gcode/subscribe_output` |
| `api → mcu` / `cmd` / `event` | **没有**，将来也不应该有：端点经 `printer` / `gcode` 间接使用协议层，不直接碰帧与字典 |

**并发模型**：`api` 用一个任务 accept、一个任务服务一条连接。所以跨连接并行、同连接内的请求保持顺序（客户端 pipeline 时看到的顺序与上游一致）；一个卡住的客户端只占住自己的任务。上游是一个线程 + reactor + 每连接一对 fd 回调，形状等价，只是用任务代替了 greenlet。推送给连接用的是**同步**的 `PushTarget::push`（入队 + `Notify` 唤醒该连接的任务），因此任何任务/线程都能推，不需要持有 runtime；“写不动超过 5 秒就断开”与上游的 `blocking_count` 同义。端点的 `Endpoint::handle` 是异步的（返回 `EndpointFuture`），因此 `gcode/script` 可以在一句请求里 `.await` 一次 `G28`/`M109`，而不必把 worker 卡在 `block_in_place` 里等机器。

`api` 内部自己是分层的（协议 ↔ 注册表 ↔ 连接），与 MCU 一侧的 `msg` / `mcu` / `cmd` 分法同构：底层的分帧不知道任何端点，端点也不知道字节怎么分帧。

### `crates/klippy-client/` — 自带的客户端

客户端**不在**本包里（`klipperx`），因为它不是主机的一部分：它只对外说话。它自成一个包，因此编译它不会编主机的东西。

| 文件 | 职责 |
|------|------|
| `lib.rs` | `klipperx api` / `klipperx console` 以及 `klippy-client` 三处的参数与入口；`--api-server` 与主机共用同一个解析 |
| `build.rs` | **构建期新鲜度校验**：仓库内构建时扫描主机 `src/core/klippy`，与签入的内建参数表 `src/gcode_params.rs` 逐条比对（按名字与参数，**不做文本比较**，免得 `cargo fmt` 的换行被当成漂移），不一致就让构建失败并给出重生成命令；主机源码不存在（发布/安装构建）时跳过，签入表即兜底 |
| `connection.rs` | `Connection`：分帧、`id` 分配与回收、应答按 `id` 配对并标注方法名、推送识别；超时的调用方可撤销 id（`forget_pending`），以免离开时白等宽限 |
| `session.rs` | `Session`：把一行输入解释成请求或本地命令（`handle_line`）、把整行当 g-code 发（`handle_gcode_line`）、订阅 g-code 输出（`subscribe_gcode_output`）、登记发出去的请求、把收到的东西变成 `Entry`；不打印任何东西 |
| `tui.rs` | 全屏窗口：三块面板、键位（`^G` / `/gcode` 切请求/g-code 模式，进入时自动订阅 g-code 输出**并拉一次 `objects/query` 的 `gcode.commands`（全部命令名，含无描述的）供补全**）、行编辑与历史、日志滚动；`Tab` 补光标所在的那个词——首位补命令名（`/` 词来自会话+窗口命令表、去重；g-code 模式按大小写不敏感匹配打印机命令名，多候选弹层画在输入行上方），非首位在 `=` 左边补参数名（打印机的 `parameters` 优先，否则用签入的内建表 `gcode_params.rs`），值里不补，候选层开着时其它键先收层再照常生效；g-code 模式把 `gcode/script` 往来剥掉 API 信封直显（`>` 回显脚本、`< ` 标打印机原文，g-code 来往按方向分色；其余 `gcode/*` 管线的成功请求/应答隐藏、失败保留），切回请求模式后恢复信封；把 `Entry` 画出来 |
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
       ├─ Api / Server / klippy_process(config_file)     ← 照常
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
  （`strings target/release/klippy` 里一个 ratatui 都不剩，`klipperx` 则有 227 处），并且不再接受
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

它是唯一**依赖不重合**的包：`cargo tree -p klippy-client` 里没有 `reqwest` / `flate2` / `libloading`（TUI 用的 `ratatui` 是它自己的），实测 debug 63.5 MB / release 3.9 MB，而 `klipperx` 是 debug 242.4 MB / release 18.3 MB（2026-10-03 实测）。代价是 `main.rs` 里那二十行日志初始化与主机重复——为它单开一个 crate 比重复更糟。

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
| `mod.rs` | `Mcu`：构造（`new`）、收发任务、`send` / `call`、字典安装与查询（出站队列 `SEND_QUEUE_CAPACITY` = 512：同步 `send` 在队列满时**有界退避等待**（总 ≤ `SYNC_SEND_WAIT` 1s），超时仍报错且带上命令名与水位；能大量入队的路径走 await 容量的 `send_payload`——剩余 ≤16 格（`SYNC_SEND_HEADROOM`）时暂停入队等排空，给同步 `send` 常备余量）、`seconds_to_clock`、`min_schedule_time`（上游 `mcu.py:13` 的 `MIN_SCHEDULE_TIME = 0.100`，本仓复用 `MIN_REQTIME_DELTA`，不另存第二份数值，供 `GCodeRequestQueue` 对齐 `next_min_flush_time`）、**时钟估计器**（`estimated_clock`：样本 `(sent, received, clock)` 取中点折半程 RTT 作错点，窗口 `CLOCK_FIT_WINDOW=30` 最小二乘拟合频率，无样本时回退字典标称；`get_uptime` 种子无往返则锚在调用瞬间）；**`Mcu::close`**：显式关闭（置 `closed` 旗 → `interface.shutdown()` 放阻塞读 → 中止收发任务），`enqueue`/`flush` 见旗即拒，**不依赖引用计数**（对齐上游 `_disconnect()`，`mcu.py:718-719`），`Drop` 退化为 trace+close 兜底。`Drop`；构造时向 `identify` 取起始 `Parser`，本身不引用任何命令；收发与告警日志行带 `[<MCU 名>]` 前缀（`send` / `recv` / `Unhandled message` / `Decode error` / `Timeout waiting for response`） |
| `object.rs` | `McuObject`：`[mcu]` / `[mcu <name>]` 作为打印机对象，以及工厂 `load_config` / `load_config_prefix`。section 只在 `PrinterObject::connect` 时才解析、开设备、跑 identify，随后把累积的配置交给固件（需要时先复位），再把固件的 `shutdown`/`is_shutdown`/`starting` 绑成打印机停机；`connect` 在时钟种子成功后注册 **1 s 的 `mcu_clock_poll`**（全部 MCU，每秒一次 `get_clock` 喂 C1–C3 估计器，teardown/Drop 取消）；**失联判定**：每轮先计「已发未答」查询数（`ClockEstimator::note_query_sent`，答案折叠时清零），超过上游的 4 条界限（`clocksync.py:156-157`）就按上游文案 `Lost communication with MCU '<name>'` 停机（`MCUConnHelper.check_timeout`，`mcu.py:897-905`），每 MCU 只报一次，file-output 模式不判（无 2026-10-18 之前那种「查询失败只 debug、永远 ready」）；**连接期串口重试**：普通启动的独占 tty 打开按上游 `serialhdl.py:181-204` 的 5 s × 90 s 预算重试（只包打开，握手与配置仍单次；重试在阻塞池上跑，退出请求在睡眠切片内被察觉），`reset` 后的重开仍走自己的 `RECONNECT_ATTEMPTS` 循环不叠预算；**握手期停机**：握手**之前**先绑一对**只记录**的 `shutdown`/`is_shutdown`（写 `connect_shutdown` 槽、`Mcu::abort_pending_calls()` 让在飞 `call` 立刻返回），握手成功后再换成上报语义；就地 `reset` + 重连 + 重跑同一份 `BuiltConfig` 是**有界**的（`MAX_IN_PLACE_RESETS = 3`，每次前 `RESET_SETTLE = 500 ms`），且「复位后回来仍是停机」也计入重试（不再一次失败就报错）；**`reconnect` 三步** = `close` 旧会话 → 重开 → `install_clock` 重绑新会话（对齐上游 `_disconnect()`，`mcu.py:718-719`），两个定时器注册幂等，`poll_clock` 对已关闭会话 debug 跳过；`get_status` 报 identify 快照 |
| `ids.rs` | **MCU 层 id 分配器**（节名 `[mcu_ids]`，历史拼写 `[canbus_ids]` 作别名兼容——两者注册同一个对象、只有一台分配器，历史名经 `object = "mcu_ids"` 挂在规范键下；第一个 `[mcu]` 不论什么接口都会把它拉起）：每个 `[mcu]` 都被发一个**唯一 id**——CAN 用它做节点寻址（`0x100 + 2*id`）；**共享串口**（同一 `serial:` 路径被 ≥2 颗 `[mcu]` 引用，`serial_ports.rs` 分组）用**同一套映射**做帧头寻址、并在分配端口时同步握手一次（映射 `assign.rs`、帧头与握手 `devices/serial.rs`），**独占串口不使用它**，`host_library` 与 `test:` **同样领到 id、只是暂不上线**（`connect` 没有它就不开传输层）。**所有接口共用一个序列**：自动号 = `NODEID_FIRST = 4` 起的**最小空位**、上界 `MAX_MCU_ID = 0xff`（255，由承载它的 CAN admin 报文单字节字段决定）——上游 `canbus_ids.py:7-18` 的 `len(ids)+4` 泛化到全传输，但不再拿「已登记数」当游标；重复 uuid 报 `Duplicate uuid`；**本仓特有**：`canbus_nodeid` 为显式覆盖，装载时由 `reserve_explicit` **预扫全配置**、把每个显式号先保留下来（读节的原始参数、不经 `get_str` 等 accessor，所以不会把该选项标成已读、不影响未定义选项检查），自动号绕开它们——显式号写在文件哪个位置都生效，发号结果与 `[mcu]` 声明顺序无关；装载期**唯一**的撞号错是**两个显式号互撞**（点名双方），号段 `4..=255` 占满时报 `mcu_ids: no free id for MCU '<名>': every id in 4..=255 is already used or reserved`（上游无显式选项、撞号靠固件 `can_id_conflict` 兜底）；每个 `[mcu]` 在 `object.rs::load_config` 登记并把返回的 id 存在 `McuObject` 上，`connect` 打开传输层前必须持有（否则报 Connection「has no id assigned」），所以 `[mcu]` 工厂的 `phase = early` 保证全部 MCU 先登记完 |
| `serial_ports.rs` | **共享串口分组**（无配置节：第一个 `[mcu]` 把它 `ensure` 起来，注册名 `serial_ports`，`is_queryable` 假、不进 `objects/list`）：按 `serial:` 路径把 `[mcu]` 与 `[mcu <name>]` 两族分组——1 颗 = 独占、≥2 颗 = 共享（[串口复用协议](serial-mux-protocol.md)）；**规则 7**：「这条线是否按 id 寻址」（`is_id_addressed`）= **共享 ∨ 下游设备 ≥ 1 ∨ 是桥**（下游台数由 `bridge_ports::bridge_downstream_counts` 预扫，`downstream_devices(path)` 可查；0 台下游的桥仍按 id 寻址属于**刻意保守口径**——被 mux 化的桥自己就有身份、输出带帧头）；同一路径上**显式写出**的 `baud` 不一致时 `warn` 并取**最小值**、未写的节跟着它走（不一致不再是配置错），`effective_baud(path)` 给出该值，`is_shared`/`peers` 供接线查询；读数走节的原始参数、不经 tracked accessor（与 `McuIds::reserve_explicit` 同哲学，不豁免未定义选项检查） |
| `host_libraries.rs` | **host 库路径的复用判定与身份指派**（无配置节，注册名 `host_libraries`）：按 `host_library:` 路径把两族 `[mcu]` 分组，判定该路径的帧是否带 mux 头——1 节既无 `uuid` 也不是桥（路径上没有桥、也没有下游）= 裸帧；1 节有 `uuid`、或该路径的 `[mcu]` 是桥（**含 0 台下游**，规则 7 的保守口径）、或 ≥2 节 = 按 id 寻址（≥2 节时最多一节可省略 `uuid`，它认领桥自身的设备）；≥2 节都省略、或两节写同一个 uuid = 装载期报错并点名涉及的节。校验在**解析层**触发（`config/mcu.rs` 的 host 臂调 `validate`；同一先例：串口臂调 `serial_ports::path_is_shared`），`ensure`/`is_mux`/`identity`/`named_uuid` 由 `mcu/object.rs::load_config` 在装载期消费（只有 `peers` 尚无生产调用点）。**身份指派**：每条路径一个**确定性**身份（`FNV-1a(路径字符串)`，逐个跳过 `[0;6]` 与配置里所有已知 uuid——含 CAN、含其它路径），按路径而非按节、跨重启稳定且永不与配置里任何 uuid 相同；`named_uuid` 在连接时决定 `SET_NODEID` 点名谁——节上写了 `uuid:` 就点名桥下游那台，否则用路径自身份；**多份库可同进程共存（2026-10-16 起）**：`reject_a_second_path` 与运行期同款守卫已删除（库不再装进程级 `SIGUSR1`，主循环唤醒改按实例 `eventfd`）；同路径节一致性校验（≥2 节的可区分性等）仍由 `validate` 负责 |
| `config.rs` | [`ConfigBuilder`](mcu-config.md)：配置期的 oid 发号器、`config` / `restart` / `init` 三张命令表、config 回调、CRC 与 `finalize_config`，`configure()` / `handshake()` 的 `get_config` 两段式握手，以及“停机或 CRC 不一致时先复位（`config_reset` 就地，或 `reset` + 重连）再配置”的复位路径 |
| `bridge_ports.rs` | **桥下游端口节**（节名 `[uart_device <名>]` / `[can_device <名>]`，order 19，前缀工厂）：校验 `*_mcu` 必填且指向存在的 `[mcu]`、`*_oid` 必填、`uart_baud` 正整数、`uart_restart_method` **只认线级方法 `arduino`/`cheetah`**（写其它值解析期报错）；未知选项留给 `check_unused`。工厂返回轻量描述对象（`UartPort`/`CanPort`，`is_queryable` 假）注册在各自节标识下；`bridge_mcus(config)` 零 access 扫描供**规则 7 的桥半边**（桥所在路径按 id 寻址）使用；`load_uart_device` 在配置期把 `config_uart`（带 `slot`）/`config_bridge_port` 排进桥的 `ConfigBuilder` 并记下端口 `index`（= 声明顺序；**`slot` 就是这个 index**，一处计数），`uart_path:` 则随端口表在装载期收集、库 `init` 前交给库（见 `cmd/uart.rs`） |
| `bridge_attach.rs` | **桥下游端口设备的 attach（M1-P3）**：`resolve` 从子节的 `uart:`/`can:` 取端口节与桥（含 oid 与桥名），要求**桥已先连**（否则点名错误，并区分「裸 `[mcu]` 永远先于 prefix 节装载」这一支），端口 `index` 来自该端口节的**声明顺序**（M1-P7 起固件侧没有端口表；配置期由主机发 `config_uart`+`config_bridge_port` 声明，见 `mcu/bridge_ports.rs`），并拒绝 `[can_device]` 端口（属 P5）；`open` 开一个**不握手**的下游句柄（`attach_downstream`/`load_downstream`），**先**向桥发 `SET_PORT_NODEID(index, id, flags)`，命名设备**再**做设备握手；`open_link` 供复位路径复用（一处文案） |
| `port_grouping.rs` | **端口分组与冲突校验**（无配置节，注册名 `port_groups`，形态与 `serial_ports.rs` 同构）：按（引用键类型, sub）把两族 `[mcu]` 分组到已声明的端口节，`classify`/`ensure`/`validate` 照 `host_libraries` 先例；查询 `peers`/`is_shared`/`is_legacy`/`bridge`/`kind`。规则：共享口（≥2 成员）全员互异 uuid、CAN 口成员必有 uuid、桥不能同时是下游子设备；legacy = 恰 1 成员且 uart 且无 uuid。预扫走 `get_text` 不记 access（`classify_records_no_reads` 钉住），校验在解析层触发（`config/mcu.rs` 的 `uart`/`can` 臂调 `validate`） |
| `link_grouping.rs` | **链接分组共享核心**（`pub(crate)`）：`group_by_key` + `distinguishability`（缺 uuid / 重复 uuid 两类，`allowed_uuidless` 参数化）+ `declared_uuid` 归一 + `quoted` 文案——`host_libraries.rs`（路径键）与 `port_grouping.rs`（端口节键）**共用同一实现**，两套共享判定不再并存；host_library 路径形态行为与既有测试逐字不变 |
| `resource/pin.rs` | `McuChip`（MCU 作为 pin chip，实现 `PinChip`）与 `McuDigitalOut`：数字输出的 oid、`config_digital_out` / `update_digital_out` 与运行期的 `queue_digital_out`；pin 名→编号在 config 回调里完成。另提供 `resolve_bus_name`（`BUS_PINS_<bus>` 预留，供 SPI/I2C 调用）与 `resolve_bus_value` |
| `resource/pwm.rs` | `McuPwm`：硬件 `config_pwm_out` / `queue_pwm_out` 与软件 PWM（`config_digital_out` + `set_digital_out_pwm_cycle` + `queue_digital_out`），`set_pwm` / `update_pwm` / `next_aligned_clock` |
| `resource/adc.rs` | `McuAdc` 与 `AdcRegistry`：`config_analog_in` + 周期 `query_analog_in`（新旧两种格式按字典格式串选择），按 oid 路由 `analog_in_state` 上报 |
| `resource/stepper.rs` | MCU 侧的步进器（上游 `MCU_stepper`）：oid、`config_stepper`、运行期发步进批与 `stepper_get_position`；运动层只认它 |
| `resource/endstop.rs` | `McuEndstop`（上游 `MCU_endstop`）：归零时的固件侧限位，持有 `endstop_home` 的触发窗口与查询 |
| `resource/trsync.rs` | `McuTrsync`（上游 `MCU_trsync` / `TriggerDispatch`）：触发组——`trsync_start` 后多个步进器在触发时一起停，并把触发时刻的位置报回；`Completion` 存原始 reason：`wait()` 保持 1-4 的 typed 语义（未知码折叠 `CommsTimeout`），`wait_raw` 供 trigger_analog 的 5+ 原因贯通，`raw_is_failure`（≥4）判失败。**`trsync_state` 路由器（2026-10-18）**：在会话 `attach`（identify 之后、配置握手之前）就装、整会话不解除，**无主 oid 静默丢弃**（对齐上游 `mcu.py:228-240`）；`on_restart` 列表里发 `trsync_start(oid,0,0,0)`，把上一轮（例如被杀的进程）留在该 oid 上的周期报告定时器断掉；同时删掉了挂在 chip 上的单向 `bound` 标志——它会在 `Mcu` 被重建时让绑定永久丢失。归零等待另加 **5 s 兜底**（上游 `TRSYNC_TIMEOUT` 语义）：超时先 `dispatch.stop()`（发 `trsync_trigger`）再以 `Timeout during homing` 结束，**不再占着 gcode 锁把整台机器锁死** |
| `resource/trigger_analog.rs` | `McuTriggerAnalog`（上游 `MCU_trigger_analog`）：`set_raw_range` / `set_trigger` 去重后下发、`home` 挂 trsync 归零、`query_state` 回报 homing/时钟；错误码四类解码（`RAW_RANGE` / `OVERFLOW` / `MONITOR` / `SENSOR_SPECIFIC`，≥`SENSOR_SPECIFIC` 走传感器回调）；含 `MCU_SosFilter`：SOS 段/状态/offset_scale 去重缓存；已实现 `HomingEndstop`（M5b 第二实现通道，eddy 类探针共用 `probing_move`） |
| `resource/spi.rs` | `McuSpi`（上游 `MCU_SPI`）：`config_spi`（连片选一起装）与传输 |
| `resource/i2c.rs` | `McuI2c`（上游 `MCU_I2C`）：`config_i2c` 后配置为硬件或软件（bit-bang）总线，读写字节 |
| `dictionary.rs` | `Dictionary`：解析固件字典、枚举展开、安装进 `Parser` |
| `pending.rs` | `PendingCalls`：同步请求/响应记账；`abort_all` 清空全部在飞调用（丢弃 sender，等待方立刻返回，不等超时） |
| `events.rs` | 按消息 id 索引的入站回调表（`McuEvents`）：`bind` / `callback`，与 `Parser` 分开放——编解码表是纯字典，回调可能反向持有资源 |
| `error.rs` | `McuError`（总括）、`McuCallError`（`call` 专用） |
| `restart.rs` | 固件复位的物理分派：`command` / `arduino` / `cheetah` / `rpi_usb` 四种怎么把板子重置，含连接期门控（`restart_before_bringup`、`check_usb_power`）与 USB 端口切电（`interface/usb.rs`） |
| `restart_method.rs` | `McuRestartMethod` 配置枚举 |

### `cmd/` — 命令层

| 文件 | 职责 |
|------|------|
| `mod.rs` | 命令词汇：`McuCommand` / `McuResponse` / `Params`，以及类型化调用 `Mcu::send_msg` / `Mcu::call_msg` |
| `allocate_oids.rs` | `allocate_oids`：预留对象 id（固件 `basecmd.c` 的 Low level allocation） |
| `config.rs` | `get_config` / `finalize_config`：配置 CRC 握手（`basecmd.c` 的 Config CRC） |
| `uptime.rs` | `get_uptime`：读 64 位固件时钟（`basecmd.c` 的 Timing and load stats） |
| `shutdown.rs` | `emergency_stop` / `clear_shutdown`：固件停机与解锁（`basecmd.c` 的 Misc commands） |
| `clock.rs` | `ClockSync` / `McuClock`：`get_clock` ↔ `clock`（用能力 trait 把时钟同步与 `Mcu` 解耦，测试里用不依赖 MCU 的 `FixedClock`）；每次 `get_clock` 查询同时以三元组喂 `Mcu` 的时钟估计（`test_a_query_feeds_the_mcu_clock_estimate`） |
| `gpio.rs` | `config_digital_out` / `update_digital_out` / `queue_digital_out` / `set_digital_out_pwm_cycle`：数字输出与软件 PWM 周期（固件 `gpiocmds.c`） |
| `pwm.rs` | `config_pwm_out` / `queue_pwm_out`：硬件 PWM（固件 `pwmcmds.c`） |
| `adc.rs` | `config_analog_in` / `query_analog_in`（新旧两种）与 `analog_in_state`（新旧两种）：ADC 周期采样（固件 `adccmds.c`） |
| `stepper.rs` | `config_stepper` / `queue_step` / `reset_step_clock` / `set_next_step_dir` / `stepper_get_position` / `stepper_stop_on_trigger`：步进生成（固件 `stepper.c`）；**配置期/接管期经 restart 列表补 `reset_step_clock clock=0` 重锚固件步进链**（上游 `stepper.py:117-118` `on_restart=True`，新配置与复用两分支 `mcu.py:1014/1069` 均下发——C5 案的修复点） |
| `endstop.rs` | `config_endstop` / `endstop_home` / `endstop_query_state`：归零期的固件侧限位（固件 `endstop.c`） |
| `trsync.rs` | `config_trsync` / `trsync_start` / `trsync_set_timeout` / `trsync_trigger`：触发组——多个步进器同时停（固件 `trsync.c`，回零的停止机制） |
| `trigger_analog.rs` | `config_trigger_analog` / `trigger_analog_set_raw_range` / `trigger_analog_set_trigger` / `trigger_analog_home` / `trigger_analog_query_state` 与响应 `trigger_analog_state`：探针式触发（固件 `trigger_analog.c`，配合 `sos_filter.c`） |
| `sos_filter.rs` | `config_sos_filter` / `sos_filter_set_section` / `sos_filter_set_state` / `sos_filter_set_offset_scale` / `sos_filter_set_active`：SOS 滤波器的段与状态设置（固件 `sos_filter.c`） |
| `spi.rs` | `config_spi` / `spi_set_bus` / `spi_set_sw_bus` / `spi_transfer` / `spi_send`：SPI 总线（固件 `spicmds.c`） |
| `i2c.rs` | `config_i2c` / `i2c_set_bus` / `i2c_set_software_bus` / `i2c_write` / `i2c_read` / `i2c_transfer`：I2C 总线（固件 `i2ccmds.c`） |
| `thermocouple.rs` | `config_thermocouple` / `query_thermocouple` 等：SPI 热电偶/RTD 测温（固件 `thermocouple.c`） |
| `ds18b20.rs` | `config_ds18b20` / `query_ds18b20` 与 `ds18b20_result`：1-wire 温度传感器（固件 `ds18b20.c`） |
| `debug.rs` | `debug_read` / `debug_write` / `debug_ping` / `debug_nop`：寄存器级调试（固件 `debugcmds.c`，供 `temperature_mcu` 标定等用） |
| `identify.rs` | `identify` / `identify_response` 的类型化视图（分片驱动在 `identify.rs`） |

### `event/` — 事件层

与 `cmd` 平级的目录。这里有两套机制：

- **固件事件**由固件发起，经 `Mcu::bind_event` 绑到响应名上（`McuEvent`）；
- **打印机事件**由主机自身触发（生命周期、重启、部件状态变化），词汇是 `KlippyEvent`，
  由 `Printer::register_event_handler` / `send_event` 按事件名注册与分发。

两者分开：固件事件的格式来自字典，打印机事件的名字来自上游 `klippy` 的字符串集合。
固件事件层复用命令层的 `Params`，不引用具体命令模块；打印机事件层的设计与对应关系见
[事件系统](event-system.md)。

| 文件 | 职责 |
|------|------|
| `mod.rs` | 固件事件词汇：`McuEvent`，以及回调注册 `Mcu::bind_event`（底层 `Mcu::bind_callback` 在 `mcu`）；列出 `decl` 与 `printer_bus` |
| `stats.rs` | `stats` 事件（`basecmd.c` 的 `stats_update` 定时推送；id 由字典给出，如 AVR 固件是 -12、host 库是 16——主机不写死）；`register_stats` 注册
|              | 订阅（算 `last_stats` 并记日志），在 `McuObject::connect` 中调用，固件每 5 秒推送一次 |
| `shutdown.rs` | `shutdown` / `is_shutdown` / `starting`：固件停机/重启事件；`static_string_id` 经字典枚举解成原因文本，由 `McuObject` 绑成打印机停机 |
| `printer_bus.rs` | `KlippyEvent`：`include!` 由 `build.rs` 写入 `OUT_DIR` 的生成文件 |
| `test_support.rs` | 事件模块测试共用的夹具（仅 `cfg(test)`）：伪造的字典/帧——格式串在这里是“固件会发什么”的样本，不是主机常量 |
| `decl/` | 打印机事件声明，一个命名空间一个文件，供 `build.rs` 扫描；`mod.rs` 定义空展开的 `event!` 宏 |

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

### `printer.rs` — 机器

一台机器一个实例：它持有配置装载出来的 printer objects，并管理自己的生命周期（bring up、状态、停机、空闲直到退出）。对应上游 `klippy/klippy.py` 的 `Printer`，骨架与装载拆在 `printer.rs` 与 `load.rs` 两个文件。

`PrinterObject::release_cycles()`：`teardown()` 在**丢弃部件之前**对每个部件调用它，用来打断「部件相互强引用」的环——环存在时 `Drop` 永远不会跑，被环拴住的 `Mcu` 会让接收任务的阻塞读一直挂着、共享 runtime 收不了尾（`McuObject` 覆写它清事件表；`Drop` 里再调一次兜底）；`WebhooksStatus` 也覆写它——配置被重建时经 `Api::clear_mux()` 注销该配置的 mux 实例（逐个 `detach`），让下一轮装载重新注册。

| 项 | 职责 |
|------|------|
| `Printer` | 对象注册表（`add_object` / `lookup_object` / `lookup_object_as::<T>`）、事件总线（`register_event_handler` / `send_event`，词汇见 `event/`）、状态（`get_state_message` / `invoke_shutdown`）、生命周期（`bring_up` / `teardown` / `reset_for_restart`） |
| `load.rs` | 装载入口：按各模块顶层 `section!` 声明生成的工厂表，把 config 的每个 section 变成对象并注册；`pins` / `gcode` 由它最先注册（见 [声明式表生成](codegen.md)）。`DEFAULT_MODULES`（上游 `toolhead.py:604-614` 的无条件加载表，目前只有 `idle_timeout`）在阶段走完后为**缺失**的节补建对象：`ConfigSection::new(id, None)` 交给同一个工厂，读默认值（2026-10-18） |
| `reactor` | 不拥有 runtime：时间与定时器来自被交给它的 `Reactor`（`Printer::reactor()`，见 [时钟与定时器](reactor.md)） |

`printer.rs` 不认识任何具体命令或 extras：具体部分由 `cmd/` 与 `extras/` 在装载时接进来，它只定义机器的骨架。

### `pins.rs` — 引脚解析

与 `printer.rs` 平级的单文件模块：把配置里的引脚描述变成 MCU + 引脚名，并记录谁在用哪个引脚。对应上游 `klippy/pins.py`。

| 项 | 职责 |
|------|------|
| `PrinterPins` | 注册成 printer object `pins`：`parse_pin` / `lookup_pin`（共享与重复使用）/ `reset_pin_sharing` / `allow_multi_use_pin` / **`setup_digital_out` / `setup_static_digital_out` / `setup_pwm` / `setup_adc` / `setup_stepper` / `setup_endstop`**（校验后交给 chip 建资源）；**注册但不可查询**（`is_queryable` = false，上游 `objects/list` 也是这样滤掉它的） |
| `PinResolver` | 每个 MCU 一份别名与保留：`reserve_pin` / `alias_pin` / `resolve`（上游 `update_command` 去掉文本改写）；`RESERVE_PINS_*` 在 MCU connect 时预留，`BUS_PINS_<bus>` 由 `McuChip::resolve_bus_name` 预留 |
| `PinChip` / `DigitalOut` / `PwmOut` / `Adc` | chip 侧接口与资源接口；`McuChip` 建出 `McuDigitalOut` / `McuPwm` / `McuAdc`（`mcu/resource/pin.rs`、`mcu/resource/pwm.rs`、`mcu/resource/adc.rs`） |
| `PinType` / `PinParams` / `PinError` | 资源类型决定描述可带哪些修饰（`!` / `^` / `~`）、解析结果、上游原文的错误文案 |

数字从哪来：上游把引脚**名字**留在命令文本里，发送时由 msgparser 查字典的 `pin` 枚举；这里编码器只接受 `ArgValue`，所以名字要在**配置回调**（build 时、有字典）里换成编号，见 [MCU 配置构建](mcu-config.md)。资源的派发（上游 `setup_pin`）在 `PrinterPins` 的一组 `setup_*` 上（`setup_digital_out` / `setup_pwm` / `setup_adc` / `setup_stepper` / `setup_endstop` / `setup_static_digital_out`），由 chip（`McuChip`）建出对应资源。

### `gcode.rs` — G-Code 调度器

与 `printer.rs` / `pins.rs` 平级的单文件模块：把一行 g-code 解析成命令名与参数，查命令表，运行处理器。对应上游 `klippy/gcode.py`。

调度器是 `Printer::new` 建的 **host 对象**（`teardown` 特意保留它跨重启），因此它对 printer 只持 **`Weak<Printer>`**：强引用会闭成
`printer → objects → gcode → printer` 环，整台机器（连同每个 `Mcu`）永不析构。`printer()` 因此返回 `Option<Arc<Printer>>`。

| 项 | 职责 |
|------|------|
| `GCodeDispatch` | printer object `gcode`：`register_command` / `register_mux_command`（`SET_PIN PIN=…` 这类按一个参数选处理器）、`run_script` / `run_script_from_command`（处理器内部入口，宏类模块用）、`create_gcode_command`（合成命令）、`command_exists`（查命令名是否注册，供 `gcode_macro` 装载期检查用）、输出处理器、`get_status` 报命令表（所以它是**可查询**对象）；`command_of_line` 是公开 helper，复用解析逻辑返回一行的命令名，供外部消费者（`gcode_macro`）只取名字 |
| `GcodeCommand` | 交给处理器的已解析命令：通用 `get`（parser + `minval`/`maxval`/`above`/`below`）与 `get_str` / `get_int` / `get_int_bounded` / `get_float` / `get_float_bounded` / `get_float_range`（缺参 / 解析失败 / 超范围都报上游文案的 `CommandError`），`get_command_parameters` / `get_raw_command_parameters`，以及 `respond_info` / `respond_raw` |
| 传统 / 扩展命令 | 传统（`M110`、`G1`）参数是 `S200` 这种“字母+值”；扩展（`SET_PIN`）是 `KEY=VALUE`，带 shell 引号——后者在分派时重解析（上游 `_get_extended_params`） |

它在 `load_config` 里**最先**注册（在 `pins` 之前），因为资源与 `[board_pins]` 建对象时要往它注册命令；按上游，它是 `Printer.__init__` 的早对象。**运动命令不在这里**：`G0`/`G1`/`G20`/`G21`/`G90`/`G91`/`G92`/`M82`/`M83`/`M114`/`M220`/`M221`/`SET_GCODE_OFFSET`/`SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE` 由 `gcode_move` 注册（坐标系那一层读它们，再把工具头坐标交给 toolhead），`G4`/`M400`/`G28`/`SET_KINEMATIC_POSITION` 由 toolhead 注册，温度类（`M104`/`M109`/`M140`/`M190`/`SET_HEATER_TEMPERATURE` 等）由各 extras 注册（见 [G-Code 命令参考](../user-manual/gcode-commands.md)）。`gcode:command_error` 事件（处理器报 `CommandError` 时触发；panic 走停机、不发）已就位；命令错误一律**报告 + 中止整段脚本**并把错误交回调用方 —— 上游 `GCodeIO` 的行协议（`ok` 应答 / `need_ack`）本仓不实现，相关机制已从 `gcode.rs` 删除。见[本项目与上游的偏移](upstream-deviations.md)。

一处**有意偏离**：mux 命令的“值不合法”提示里，上游按 dict 迭代序取最后一个匹配做 `Did you mean`，这里对候选排序后取第一个（消息要稳定）。默认项（注册 `value=None`）与上游一致：不给 key 时命中。（本页其它与上游不一致处统一登记在[本项目与上游的偏移](upstream-deviations.md)。）

#### 越界报错里限制值的写法（与上游不同，有意保留）

命令与配置两边的越界报错都按这一节的说法；只有整数取值的限制值能看出差别（`0.5` 这类
两边写法一样）。

**现象**：报错里的限制值写成 `0`、`1`、`-1` 这种没有小数点的样子（如 `A must be above 0`），
上游同样的报错写成 `0.0`、`1.0`、`-1.0`。

**根因**：上游用 Python 的 `%s` 直接打印传进来的那个数；Python 打印“正好是整数的浮点”会带
小数点（`0.0`），Rust 默认打印不带（`0`）。上游也有调用点传的是整数（例如
`quad_gantry_level.py:30` 的 `above=0`），那里两边都是 `0`——所以不能统一补小数点，要逐处
对着上游调用点才能改。

**后果**：只影响这句话显示成什么样。判断在拼字符串之前就做完了，同一个值两边接受 / 拒绝一致；
数值、配置解析与 `SAVE_CONFIG` 写回的值都不受影响；语料只比结果不比文本（见
[回归测试](regression-tests.md)），所以它也不会被语料抓到。

**决定**：不修，只记录。对齐的收益纯粹是文本（日志与上游文档一字不差），代价是按上游调用点
逐处分类、再重推一批断言。

### `extras/` — 建立在核心之上的 `[<section>]` 模块

对应上游 `klippy/extras/`：它们是核心（pin 层、G-Code 调度器、MCU 配置）的**使用者**，
通过 `load.rs` 的工厂表接入，核心不反过来引用它们。工厂表由各模块顶层的 `section!` 声明
生成（[声明式表生成](codegen.md)），因此新增段落不必改中心表。

| 文件 | 职责 |
|------|------|
| `output_pin.rs` | `[output_pin <name>]`：读 `pin` / `value` / `shutdown_value`，以及 PWM 的 `pwm` / `cycle_time` / `hardware_pwm`；用 `PrinterPins::setup_digital_out` 或 `setup_pwm` 建资源（无条件 `setup_max_duration(0)`，同上游），向 `gcode` 注册 `SET_PIN PIN=<name> VALUE=<0..1>`；`get_status` 报 `value`。**调度**：`cmd_set_pin` 把值经 `register_lookahead_callback` 钉到前瞻时刻、入 `GCodeRequestQueue`，flush 时以 `queue_digital_out(clock)`（软 PWM 先 `next_aligned_clock` 再 `set_pwm`）**按打印时间生效**，同值重复设置 `Discard` 零发帧；**两条立即路径兜底**——无 `[printer]`（lookup 不到 `toolhead`）或资源未连接（`min_schedule_time()` 为 `None`）时走 `update_digital_out`/`update_pwm`；换算经 `DigitalOut`/`PwmOut` 上**带默认实现**的 `print_time_to_clock` / `min_schedule_time`（只有 `McuDigitalOut`/`McuPwm` 覆写）。`TEMPLATE=` **已支持**（2026-10-18：`template_evaluator` 单例，0.5 s 求值后经 `send_async_request` 下发；`TEMPLATE=` 空值 = 取消订阅；`static_value` 只接受并忽略——上游已删该参数）；接线件 `PinSchedule` / `arm_queue` 与 `servo`、`pwm_tool` 共享（2026-10-03） |
| `bed_mesh.rs` | `[bed_mesh]`：床网格标定。认领 `mesh_min`/`mesh_max`/`probe_count`/`speed`/`algorithm`/`horizontal_move_z`/`fade_*`/`mesh_pps`/`bicubic_tension`/`round_probe_count`/`mesh_radius`/`mesh_origin`/`move_check_distance` 与 `faulty_region_<N>_min`/`_max` 对；选项解析按上游 `parse_config_pair`（批 #11：`probe_count`/`mesh_pps` 单值→(n,n)、minval 3/0，非法值文案逐字对齐；round 床不读 `mesh_min`/`mesh_max`，边界由 `mesh_radius` 推导并 floor 到 0.1mm）；按上游生成矩形（行内 zigzag、间距下取整到百分位）或圆床（按半径过滤）探测点；`BED_MESH_CALIBRATE` 逐点移动 + 经 `probe` 会话探测并存下网格（存格按 Y 分行、行内 X 升序，对齐上游 zigzag 探测序），`BED_MESH_CLEAR` 清空，`get_status` 报二维 `probed_matrix`（实测网格）与 `mesh_matrix`（插值网格，由 `ZMesh` 产出并 `round(z,6)`）与 `profile_name`（已探测时为 `default`）。**已做（2026-10-06，`dc1dd1e`）**：`bed_mesh/dump_mesh` 端点（`api/endpoints/bed_mesh_dump.rs`，`mesh_matrix` 已改为插值网格）；插值网格本体——`ZMesh` 的 `lagrange`/`bicubic`/`direct` 三个采样器、`mesh_pps` 细分（边长 `(n-1)*(pps+1)+1`）、`bicubic_tension`、`calc_z` 双线性查表（越界 constrain）、`_verify_algorithm` 的算法/探针数校验与 `mesh_pps` 两轴全 0 时改写为 `direct`、圆床行补齐成方阵。**未做**：faulty 区域替换、`mesh_offsets`/`set_zero_reference`（仅被 `BED_MESH_OFFSET`/`ZERO_REFERENCE` 驱动）、`BED_MESH_PROFILE`/`OUTPUT`/`MAP`/`OFFSET`。**已做（2026-10-06，`05f93f6`）**：**fade + `MoveSplitter` + move transform** —— `fade_start`/`fade_end`/`fade_target` 与 `get_z_factor`（`fade_target` 合法性看网格 Z 范围、`==0` 时免检、另有 `fade_dist` 上界，文案逐字对齐）、`MoveSplitter`（`build_move`/`calc_z_offset`/`set_next_move`/`split`，逐行对齐 `bed_mesh.py:1257-1319`）、`impl MoveTarget for BedMesh` 并在 load 期以 `force=false` 注册（`bed_mesh.py:130-131`）、新增 `split_delta_z`（默认 `.025`、min `0.01`）；**`BED_MESH_CLEAR` 不撤销 transform**（上游只注册一次，clear 即 `set_mesh(None)`，无网格时 `move()` 直通 toolhead） |
| `bed_tilt.rs` | `[bed_tilt]`：床面倾斜补偿。读 `x_adjust`/`y_adjust`/`z_adjust`（默认 0）与可选 `points`；装载时用 `gcode_move::set_move_transform` 把自己装成移动变换（`get_position` 减平面、`move_to` 加回）；`points` 存在才注册 `BED_TILT_CALIBRATE`（≥3 点，经 `ProbePointsHelper` 驱动），回调用 `coordinate_descent` 拟合平面，`update_adjust` 重锚坐标并把三项以 `%.6f` 记入 `configfile` 的 `SAVE_CONFIG` pending（`SAVE_CONFIG` 命令随配置装载注册，写回文件并重启） |
| `z_tilt.rs` | `[z_tilt]`：多 Z 电机调平。`RetryHelper`（`retries`/`retry_tolerance` 与命令覆盖、点高差范围文案）、`ZAdjustStatus`（`applied` 标志，`stepper_enable:motor_off` 复位）、`ZAdjustHelper`（Z 电机名单与 `z_positions` 项数校验、`adjust_steppers` 逐步复刻：脱挂→按 `-a` 排序挂回移动→收尾复挂，异常全挂回重抛）、`ZTilt` 段与 `Z_TILT_ADJUST`（平面残差走 `coordinate_descent`）。还作为 `quad_gantry_level` 的共用件出口 |
| `quad_gantry_level.rs` | `[quad_gantry_level]`：四点龙门调平。`points` 恰好 4、`gantry_corners` >=2、`max_adjust` 中止文案；`linefit`/`plot` 精确两点直线拟合（非最小二乘）求四角高度；复用 `z_tilt` 的 `RetryHelper`/`ZAdjustStatus`/`ZAdjustHelper`；保留上游两处怪癖（`horizontal_move_z` 不跟随命令覆盖、重试用机架相对高度） |
| `bltouch.rs` | `[bltouch]`：BLTouch 探针。`control_pin`（PWM 占空比即单线协议，按上游 `Commands` 表发命令）+ `sensor_pin`（endstop）；connect 按上游 raise+verify；把自己注册为 `probe` 对象与 `probe` 虚拟 chip（复用 `probe.rs` 的会话/chip 件，`endstop` 已是 `Arc<dyn HomingEndstop>`），注册 `BLTOUCH_DEBUG`（`COMMAND`）/`BLTOUCH_STORE`（`MODE`），探针族三命令复用 `probe.rs` 的注册路径；`home_start` 夹 `ENDSTOP_REST_TIME` 并在触发后按 `multi` 抬针 |
| `screws_tilt_adjust.rs` | `[screws_tilt_adjust]` 与 `SCREWS_TILT_CALCULATE`：逐螺丝测床面、按 `screw_thread` 螺距把偏差折成 `HH:MM` 圈数（对应上游 `screws_tilt_adjust.py`） |
| `trigger_analog.rs` | trigger_analog 主机侧 design 半边（上游同名文件 L11-110）：`to_fixed_32` / `calc_frac_bits` 定点转换、`GeneratedSOS` 表生成、`DigitalFilter` 低通/导数设计；无 `section!`（非配置段），与 `cmd/trigger_analog.rs`、`mcu/resource/trigger_analog.rs` 构成三件套（M5b） |
| `bulk_sensor.rs` | 批读框架（工单 H6，对应上游 `bulk_sensor.py`）：`FixedFreqReader`（状态应用/块切片/16 位序号回绕）、`BatchBulkHelper`（首客户端启动批循环、末客户端停）、`ClockSyncRegression`（EMA 时钟回归）、`MuxBatchEndpoint`/`WebhooksBatchClient`（`response_template` 推送；mux 注销时 `MuxBatchEndpoint::detach` → `BatchBulkHelper::stop()` 标记 detached、清客户端、拒绝新客户端并停流，且不再对已消失的配置跑 stop 回调）与按 oid 路由的 `BulkDataRegistry` |
| `motion_report.rs` | 运动数据转储（上游 `motion_report.py` 的两个只读端点）：`dump_stepper`（header `interval,count,add`；批次带 `start_position`/`start_mcu_position`/`step_distance`/`first_clock`/`last_clock` 等，首个 `step_count==0` 处截断）与 `dump_trapq`（header `time,duration,start_velocity,acceleration,start_position,direction`；`accel = 2*half_accel`，重读窗口 `last.time + min(duration, 0.1)`、去重一行）；实例按**名字**在装载期注册（`toolhead`/`extruder*`/每个 stepper），请求时再解析成 trapq id；**未做**：`get_status`（`live_position` 等）、shutdown 诊断、`manual_stepper` 的 trapq 实例、drip 保护 |
| `ldc1612.rs` | LDC1612 传感器库对象（对应上游 `ldc1612.py`，**无配置段**，由 probe_eddy_current 构造——该接线随 M5d 落地生效）：I2C 复用 `setup_i2c`、`config_ldc1612[_with_intb]` + restart `query_ldc1612`、`freq_conv`/`sensor_div` 换算、`convert_samples` 错误分支、`LDC_CALIBRATE_DRIVE_CURRENT`（mux `CHIP=`，`reg_drive_current` 提取）、`dump_ldc1612` 端点与 `setup_trigger_analog`→`ldc1612_attach_trigger_analog` 绑定 |
| `axis_twist_compensation.rs` | `[axis_twist_compensation]`：按 X/Y 位置插值 Z 补偿，经 **`probe:update_results` 载荷**（`ProbeResultsHandle`，批 #36 新增）原地改上报 Z；`AXIS_TWIST_COMPENSATION_CALIBRATE` 校准向导 |
| `gcode_button.rs` | `[gcode_button <name>]` 数字路径（`pin`/`press_gcode`/`release_gcode`/`debounce_delay`、`QUERY_BUTTON`、`get_status`）；gap：固件按钮查询未接、`analog_range` 明确拒绝（无 `query_adc`，批 #37）；**`QUERY_BUTTON` 的 handler 对自身是 `Weak`（2026-10-06）**：理由同 `virtual_sdcard`——它持 `Arc<GCodeDispatch>`，强捕获会成环并把命令表连同表里部件一起跨 `teardown` 钉住 |
| `gcode_arcs.rs` | `[gcode_arcs]` 段落地（`resolution`，语料档位）：G2/G3 与平面选择命令属 H10 未注册（对应上游 `gcode_arcs.py`） |
| `bed_screws.rs` | `[bed_screws]` 手动调平段（screw1..99 缺失即停/名称/fine_adjust/行进默认，≥3 螺丝；对应 `bed_screws.py`）；**`BED_SCREWS_ADJUST` 命令族已移植（2026-10-06）**：`BED_SCREWS_ADJUST` 装载期常驻注册，`ACCEPT`/`ADJUSTED`/`ABORT` 随会话注册、收尾（ABORT/完成/toolhead 拒绝）时注销；粗调→精调两遍、`ADJUSTED` 把 `-1` 归 `0` 重数、提示文案与 status 对齐上游。它**不依赖** `probe`/`manual_probe`/`configfile`（上游同样只用 `toolhead.manual_move` 与 `gcode`）；与 `manual_probe` 的 `ACCEPT`/`ABORT` 同名，按上游「先注册者占坑」 |
| `pwm_cycle_time.rs` | `[pwm_cycle_time <name>]`：软件 PWM 引脚 + `SET_PIN` 的 `CYCLE_TIME=`（固件周期 build 期定，运行期改只记主机账；`pwm_cycle_time.py`）；`SET_PIN` 经 `register_lookahead_callback` 钉 print-time 调度、按 `min_schedule_time` 自间隔（2026-10-03，`a3e1d4c`，匹配上游 `pwm_cycle_time.py:102-122`）；无 `toolhead` 或 MCU 未连接时回退立即路径 |
| `pwm_tool.rs` | `[pwm_tool <name>]`：带 `maximum_mcu_duration` 固件兕底的 PWM 工具引脚（`pwm_tool.py`）；`SET_PIN` 经 `GCodeRequestQueue` **按打印时间生效**（复用 `output_pin` 接线件 + 两条兜底，2026-10-03） |
| `temperature_fan.rs` | `[temperature_fan <name>]`：传感器+风扇+`watermark`/`pid` 双控制环，`SET_TEMPERATURE_FAN_TARGET`（mux，`temperature_fan.py`） |
| `temperature_host.rs` | 裸 `[temperature_host]` 只把 `temperature_host` 工厂注册给 `heaters`（**`phase = early`**，order 30）；传感器在消费节（`[temperature_sensor]`/加热器，`sensor_type: temperature_host`）里建：`sensor_path` 默认 `/sys/class/thermal/thermal_zone0/temp`、1 s 定时采样 ÷1000、min/max 越界 shutdown 逐字、对象 `temperature_host <末词>` 的 `get_status` 只有 `temperature`、`--debugoutput` 跳过开文件与定时器、读失败置 0 并退役定时器（2026-10-03，上游 `temperature_host.py`） |
| `controller_fan.rs` | `[controller_fan <name>]`：`klippy:ready` 起每秒 tick，active→idle→stop（`controller_fan.py`） |
| `gcode_macro.rs` | `[gcode_macro <名>]`：宏即命令（大写注册、description 为 help）、宏体经 `template` 渲染后回派发、`SET_GCODE_VARIABLE` 已注册（mux 按段名）；**装载期静态命令存在性检查**：装载时抽出宏体中字面写死的命令名（`strip_template_tags` 置空 Jinja2 标签后取每行首词），`klippy:ready` 后逐个查 `GCodeDispatch::command_exists`，未注册者 `respond_info` 告警（不拒绝、不阻断加载），动态算出的命令名不抽取故不告警，与变量标签黏连的命令词（`G9{ 0 if ABSOLUTE else 1 }` 渲染成 `G90`/`G91`，`strip_template_tags` 保留字节偏移供比对）也不构成名字同样跳过；`variable_*` 与 `SET_GCODE_VARIABLE` 的 `VALUE` 走 [`py_literal`] 的 CPython `ast.literal_eval` 子集（另收 JSON 的 `true`/`false`/`null`），所以客户端文件里 `{'restore': False, 'temp': 0}` 这类写法能直接加载（2026-10-18，语料 = 入库的 `mainsail.cfg`）；尚缺读 `printer.objects` 的反射能力（`gcode_macro.py`，`PrinterView` 目前不可枚举） |
| `py_literal.rs` | `ast.literal_eval` 子集的**纯解析器**（无配置节，对应 `gcode_macro.py:153-162` 的 `ast.literal_eval`）：`None`/`True`/`False`（另收 `null`/`true`/`false`）、十进制与 `0x`/`0o`/`0b` 整数、浮点（含指数）、单双引号字符串与常见转义、`list`/`tuple`/`dict`（含尾随逗号）与嵌套；拒不表求值的名字/调用/算术。产物是 `serde_json::Value`，数字与布尔 key 按上游 `json.dumps` 的形态转字符串。消费方 = `gcode_macro` 的 `variable_*` 与 `SET_GCODE_VARIABLE` |
| `gcode_request_queue.rs` | 打印时间请求队列（**无配置节、纯逻辑**，移植上游 `output_pin.py:15-90` 的 `GCodeRequestQueue`）：按 `print_time` 排队、覆盖压缩（后一条请求盖过前一条）、`next_min_flush_time` 按 `min_schedule_time` 对齐、`discard`/`reschedule`/`repeat` 三分支与 `send_async_request` 直通；sink 回调在**锁外**执行（上游单线程 reactor 与本仓 gcode/flush 双线程的差异），`flush` 假设单 flusher（由 `extras/toolhead.rs` 的 10 ms tick 单驱动保证）。消费方 `output_pin`、`fan`、`pwm_tool`、`servo` **已接线**（首次可调度的请求懒注册 flush 回调），12 个单测 |
| `template.rs` | 模板引擎——**minijinja 2.24 的适配层**（`custom_syntax` 特性给单花括号定界符 `{`/`{%`/`{#}`、`UndefinedBehavior::Strict`、关自动转义、装载期编译与求值分两段）。公开门面 `Template`/`TemplateError`/`Context`/`Rt`/`Builtin`/`PrinterView` 保持不变，5 个消费方零改动；值桥=只有 printer 状态里的数组暴露 `.x/.y/.z/.e`、`action_*` 与 `range`/`namespace` 显式绑定、自定义 `int`/`float`/`min`/`max` 复刻 Jinja2 形态（可选默认参、大小写不敏感）；错误帧逐字保上游 `Error loading template …`/`Error evaluating …`，行号 1 基。2026-10-18 补三处引擎面：**输出 formatter**（标签输出容器按 Python `str()` 拼写，元素走 repr）、**unknown method callback**（`str` 的 `split`/`strip`/`replace`/`startswith`/`endswith`/`join`/`find`/`count`/`lower`/`upper`、`list` 的 `index`/`count`；`append`/`extend`/`pop` 因值语义降级成 `None`）、以及 **`%` printf 重写**（左侧是字符串字面量的 `"…%s…" % x` 在编译期改写成 `\|format(...)`，`{ … }` 与 `{% … %}` 两种标签都覆盖）。与 Jinja2 的已知差异（`%`/`//` 欧几里得取余、Strict 下缺键在打印/迭代/判真时报错、`~` 拼接容器不走 formatter、列表不可变、部分 detail 措辞）见该模块文档 |
| `template_evaluator.rs` | `TEMPLATE=` 的共享求值器（上游 `output_pin.py:187-193` 的 `lookup_template_eval`）：printer 单例对象 `template_evaluator`；`set_template(gcmd, callback)` 校验模板名与 `PARAM_*`（逐字上游文案），`TEMPLATE=` 空串 = 取消订阅；0.5 s reactor 定时器每 tick 为每个订阅建新 `Context`（`printer` + `action_respond_info`/`action_raise_error`/`range`，**无** `params`/`rawparams`），渲染失败只告警并传空串；测试入口 `render_once(eventtime)` 同步驱动，不留永久定时器 |
| `led.rs` | LED 六段一体（`led`/`neopixel`/`dotstar`/`pca9533`/`pca9632` + `display_template` 惰性单例）：共享 `LEDHelper`，上游仅因 Python 模块布局分文件（`led.py` 等五文件+`display/display.py:168`）；`[display_template]` 的 `text` 在装载期编译成 `Template` 并由 `DisplayTemplates::render` 求值（`template_evaluator` 与 `SET_LED_TEMPLATE` 的渲染后端）；`SET_LED`/`SET_LED_TEMPLATE` 仍未注册（H3/H8） |
| `extruder_stepper.rs` | `[extruder_stepper <name>]` prefix 段：段全读+绑定校验（上游文案）+`motion_queue` 记账+`SET_PRESSURE_ADVANCE` 按名挂值（`extruder_stepper.py:23`）；宿主 step 同步待 toolhead 缝（H10） |
| `exclude_object.rs` | `[exclude_object]` 零选项段 + 四命令（`EXCLUDE_OBJECT[_START/_END/_DEFINE]`）+ 排除区移动变换（`exclude_object.py`）；语料用例已随宏体渲染引擎转绿（2026-09-24） |
| `virtual_sdcard.rs` | `[virtual_sdcard]`：`path` 必填 + `on_error_gcode`（`virtual_sdcard.py:35-36`）；文件管理 + `M20`–`M27`/`M28`–`M30`（`cmd_error`）/`SDCARD_RESET_FILE`/`SDCARD_PRINT_FILE` 命令族已注册；`M24` 经 `do_resume` 注册 reactor 定时器 → spawn 回放 task，逐行 `gcode.run_script` 回放，`M25`/`do_pause` 经 `must_pause_work` 旗标停回放并**同步等待回放 task 退出**（2026-10-03，`106a91a`，匹配上游 `virtual_sdcard.py:123-127`；`cmd_from_sd` 守卫避自死锁），EOF/pause/error 分别 `print_stats.note_complete/pause/error` 收尾。省略：`gcode.get_mutex().test()` 让出（本仓无该 API）、`_handle_analyze_shutdown`/`_handle_debuginput_exit`、`stats`（`PrinterObject` 无 `stats`）；`path` 不做 `expanduser`/`normpath`；语料里 `sdcard_loop.cfg` 的相对 `path` 由**注入配置时绝对化到 `klipper_dir()`** 解决（2026-10-06 起；之前靠语料测试期切进程 CWD，那会与并行用例互踩 CWD，已删）；**注册进 dispatcher 的 handler 对自身是 `Weak`（2026-10-06）**：它自己持 `Arc<GCodeDispatch>`，若 handler 强捕获 `Arc<Self>` 就成环 `对象 → dispatcher → 命令表 → 对象`，该环会跨 `teardown` 钉住表、连带钉住表里各 handler 捕获的所有部件（含 `Mcu`，它的 `Drop` 才是释放阻塞读的那一步） |
| `print_stats.rs` | `print_stats` 打印机对象（`print_stats.py`）：`state`（standby/printing/paused/complete/error/cancelled）、`filename`、`filament_used`、`total_duration`/`print_duration`/`prev_pause_duration`、`info_total_layer`/`info_current_layer`；`set_current_file`/`note_start`/`note_pause`/`note_complete`/`note_error`/`note_cancel`/`reset`；`SET_PRINT_STATS_INFO`（`TOTAL_LAYER`/`CURRENT_LAYER`，0 清空、切换 total 重置 current、current 截断到 total）；`get_status` 三态形状；耗材靠 `gcode_move.get_status` 的 `position[3]`/`extrude_factor`；订阅 `extruder:activate_extruder` 并在切换后重锚 `last_epos`（上游 `print_stats.py:20-22`），换料不再产生假的耗材尖峰 |
| `display_status.rs` | `[display_status]` 裸段（`display_status.py:49`）；`M73`/`M117`/`SET_DISPLAY_TEXT` 已注册（批 #7，`[display]` 会按需创建它） |
| `homing_override.rs` | `[homing_override]`：`axes`/`set_position_*`/`gcode`（`homing_override.py:65`）；G28 包装未装（模板未渲染，H9 共担） |
| `sdcard_loop.rs` | `[sdcard_loop]` 裸段：`SDCARD_LOOP_*` 三命令的栈/索引语义已单测钉住（`sdcard_loop.py:72`），命令本身未注册（H4） |
| `servo.rs` | `[servo <name>]` 舵机段（脉宽几何全选项）+ `SET_SERVO`（mux 键 `SERVO=`，`servo.py`）；**调度**：`SET_SERVO` 经前瞻钉入 `GCodeRequestQueue`，sink 对齐上游 `_set_pwm`（同值 discard、`RESCHEDULE_SLACK=0.000500` 周期对齐、迟到超 slack 则 reschedule 再 `set_pwm`），两条立即兕底（2026-10-03） |
| `idex_modes.rs` | `[dual_carriage]` 段（late/order=55）+ **generic 路径**（批 #12：`register_generic` 收各滑架 `position_endstop`、`GenericDualCarriages` 对象、`Shared` 按滑架记帧；`HomingHomeRailsEnd` 上把该轴各滑架帧记到各自 endstop＝上游 `DualCarriages.home`）+ 三命令两路注册；`CARRIAGE` 先名字、`0/1` 仅恰 2 滑架回退；步进不驱动（C1） |
| `carriage.rs` | `[carriage <name>]`/`[dual_carriage <name>]`/`[extra_carriage <name>]`/`[stepper <name>]` 装载与 generic_cartesian 运动学接线（批 #12）；`build()` 接收 `[printer]` 的 `max_z_velocity/max_z_accel`（原写死 0 即 Z 归零零长 drip 不 fire 的首因）；`CarriageModel` 管滑架/电机/位姿帧；批 #27：`[dual_carriage]` 的 `primary_carriage` 是**可选**（无它即该轴的主动滑架：`axis` 必填、无 `safe_distance`），同主滑架两个 dual 按**主滑架名**（非轴）判重；`idex` 的 `dc_rails` 顺序为「主动滑架 + 从动滑架」 |
| `probe_eddy_current.rs` | eddy 探针对象（约2500行，`[probe_eddy_current <名>]` 工厂，批 #3）：`load_config_prefix` + `PROBE`/`QUERY_PROBE`/`PROBE_ACCURACY` 复用 + `PROBE_EDDY_CURRENT_TAP_CALIBRATE` tap 标定 + 采样点/虚拟端停接 `McuTriggerAnalog`（样本流=ldc1612+bulk_sensor）；**`EddyCalibrationTool` 已移植**（2026-10-03，`4caf425`：`PROBE_EDDY_CURRENT_CALIBRATE` + `Z_OFFSET_APPLY_PROBE`，校准移动采样 + 频率-位置关联 + 噪声过滤 + `configfile.set` 写回）；tap 分析依赖 `mcu_to_commanded_position`（fileoutput 路径同上游走哑数据） |
| `delta_calibrate.rs` | `[delta_calibrate]` 与 `DELTA_CALIBRATE`/`DELTA_ANALYZE`：测量几何 + coordinate descent 拟合（`delta_calibrate.py`，批 #5）；`manual_probe` 消费、SAVE_CONFIG 待写行 |
| `input_shaper.rs` + `shaper_defs.rs` | `[input_shaper]` 与 `SET_INPUT_SHAPER`：整形系数表与上游 `shaper_defs.py` 逐位对齐（wave-2）；**系数未接步进生成**（gap，`recompute_scan_windows` 为显式 no-op） |
| `adxl345.rs` + `cmd/adxl345.rs` | `[adxl345]` 加速度计（SPI、`axes_map`、`rate` 默认 3200）与 `adxl345/dump_adxl345` 端点（wave-2）；bulk 数据通路待共享泛化 |
| `mpu9250.rs` + `cmd/mpu9250.rs` | `[mpu9250]` 加速度计（I2C、默认 0x68/400k、`rate` 4000）与 `mpu9250/dump_mpu9250`（wave-2）；同上的 bulk gap |
| `filament_switch_sensor.rs` + `filament_motion_sensor.rs` + `buttons.rs` | 断料检测两段与 `[buttons]` 依赖对象（wave-2，`extruders.test` 转绿即其验收） |
| `pause_resume.rs` | `[pause_resume]` 节与 `PAUSE`/`RESUME`/`CLEAR_PAUSE`/`CANCEL_PRINT`（批 #15）；`pause_resume/*` 三个 webhooks 端点已注册（2026-10-03，`api/endpoints/pause_resume.rs`）；`virtual_sdcard` 的 `do_pause`/`do_resume`/`do_cancel` 已实现，`is_sd_active` 现可达（`is_active` 反映回放 task）；`do_pause`/`do_cancel` 已改为 async 并等待回放 task 退出（2026-10-03，`106a91a`），SD 分支经 `RegisteredSdCard` 转发真原语（`0890669`），`SD busy` 从 `RESUME` 冒出；**注册进 dispatcher 的四个 handler 对自身是 `Weak`（2026-10-18）**：本对象在 `klippy:connect` 存下 `RegisteredSdCard`（它持 `Arc<GCodeDispatch>`），强捕获就与 `[virtual_sdcard]` 一起成环 `dispatcher → 命令表 → pause_resume → sd 卡 → dispatcher`，跨 `teardown` 钉住整张表并连带钉住各 `Mcu`；单测 `teardown_releases_the_sd_card_and_pause_resume` 用 `Arc::strong_count` 钉死 |
| `heater_fan.rs` | `[heater_fan <name>]`：`Fan` 核心 + `klippy:ready` 起的每秒 tick，任一 heater 有 target 或温度 > `heater_temp` 即为 `fan_speed`，**仅速度变化时写 PWM**（批 #6；`printers.test` 的 run 级收益） |
| `fan_generic.rs` | `[fan_generic <name>]`：全部选项交给 `Fan` 核心（`shutdown_speed` 默认 **0.0**），注册 mux 命令 `SET_FAN_SPEED FAN=<name>`；`TEMPLATE=` **已接**（同一 `template_evaluator`，每次渲染调 `Fan::set_speed`；`SPEED=` 与 `TEMPLATE=` 互斥） |
| `safe_z_home.rs` | `[safe_z_home]`：接管 G28（Z-hop → 按需 `X0 Y0` → 安全位 → `Z0`）；`section!(order = 70, phase = late)` **必须晚于 toolhead（`printer`，order 60 late）**，否则 `unregister_command("G28")` 得 `None`；与 `[homing_override]` 互斥（批 #6） |
| `manual_stepper.rs` + `force_move.rs` | `[manual_stepper <name>]` 与 `MANUAL_STEPPER`（含 `GCODE_AXIS` 动态注册/注销 extra axis）；**2026-10-18 起自有 trapq**：首次需要时 `take_stepper()` 装进运动队列（隐藏对象 `manual_stepper_stepper <name>` 跑它自己的 `connect`）、`process_move` 按上游 append、`MOVE=`/`G1 A…` 真出步、`SET_POSITION` 先 flush 再摆 solver；**仍缺**自带回零（`STOP_ON_ENDSTOP` 拒绝，缺 `drip_update_time`）。`force_move.rs` 另有完整 `ForceMove`：`[force_move] enable_force_move` 门控的 `FORCE_MOVE`、每个电机一条的 `STEPPER_BUZZ`，以及 `manual_move` 的整段时序（换单轴 cartesian solver + 专用 trapq → append → dwell → 生成发步 → 还原 trapq/kinematics → wipe），`calc_move_time` 仍在原处（批 #6；`FORCE_MOVE`/`STEPPER_BUZZ` 2026-10-04） |
| `display/{mod,display,st7920,hd44780,hd44780_spi,uc1701,ssd1306,aip31068_spi}.rs` + vendored `display.cfg` | `[display]` 框架与六驱动（批 #7+#13+#28+#30）；**storage-only**（不渲染、菜单未实现，见 config 手册的 gap）；`display_status` 的 `M73`/`M117`/`SET_DISPLAY_TEXT` 批 #7 落地 |
| `hx71x.rs` + `load_cell.rs` + `load_cell_probe.rs` + `cmd/{hx71x,ads1220,ads131m0x}.rs` | `[load_cell]` 节与 HX711/HX717 驱动（批 #14，走 LC-1 的 `with_format("<i",…)` 接缝）、`ads1220`/`ads131m0x` 传感器、`load_cell_probe` 与 `LOAD_CELL_*` 命令族均已落地（2026-10-06 复核）；`load_cell/dump_force` 的 `detach` 清该 cell 的推送客户端并置 `detached`（竞态中的请求回 `UnknownMuxValue`）。**load-cell 探测三线接线（2026-10-06）**：`[load_cell_probe]` 在 `klippy:mcu_identify` 把 Z 轨电机注册进自己的 trigger dispatch（`register_z_steppers`，对齐上游 `probe.LookupZSteppers`、`probe.py:195-206`）；建完 trigger 后调传感器 `setup_trigger_analog` 抳 init attach（`trigger_analog.py:300-301`，三型 `*_attach_trigger_analog`）；注册 `probe` pin chip，`endstop_pin: probe:z_virtual_endstop` 可用（`load_cell_probe.py:698`），第二个 `probe` chip 报配置错误。余项：部分 `LOAD_CELL_*` 仍回答「not implemented」（`load_cell.rs:20` `:306` `:364`；另 `dump_taps`/`LOAD_CELL_TEST_TAP` 未接） |
| `tmc.rs` + `tmc_uart.rs` + `tmc2208.rs` + `tmc2209.rs` + `tmc_spi.rs` + `tmc2130.rs` + `tmc2240.rs` + `tmc2660.rs` + `tmc5160.rs` | TMC 驱动族（批 #7 + 后续）：单一 `TmcDriver` + `TmcTransport` trait + 表驱动；虚拟端停用装饰器实现；**六型均已落地**：UART 侧 `tmc2208`/`tmc2209`，SPI 侧 `tmc2130`（批 #34）/`tmc2240`/`tmc2660`/`tmc5160`（2026-10-06 复核：六个 `section!` 均在工厂表）。**活线与运行期链路已补齐（2026-10-18）**：UART 走 `tmcuart_send`/`tmcuart_response`（5 次重试、IFCNT 回读校验、共享 UART 的总线锁、`select_pins` 真选通）；SPI 走 `transfer` 读与「`spi_send` 写 + 哑读回读」写（多设备链共享一个 `McuSpi` 与一把锁）；`tmc/stallguard_dump` 端点（六型按能力选读路径）；使能链路（无专用 enable 脚时的 `toff` 虚拟使能）；相位跟踪（`sync_mcu_pos` 写 `mcu_phase_offset`，被 `[endstop_phase]` 消费）；1 s 周期 `DRV_STATUS`/`GSTAT`/`adc_temp` 检查与 `get_status` 合并。余项：both-edge（`dedge`）未启用；SPI 的写+回读是两条消息而非上游一次片选内两帧；`TmcRegister.receive_time` 取往返后时刻而非响应到达时刻（见[与上游的偏移](upstream-deviations.md)） |
| `board_pins.rs` | `[board_pins]` / `[board_pins <name>]`：读 `mcu` 列表与 `aliases` / `aliases_*`（`名=引脚`，值写成 `<...>` 则保留），调用 `PrinterPins::alias_pin` / `reserve_pin`。对象不可查询 |
| `static_digital_output.rs` | `[static_digital_output <name>]`：读 `pins`（引脚列表），一次全部拉到固定电平（上游同名节）；`order = 35` 排在 `board_pins` 后，别名可用 |
| `static_pwm_clock.rs` | `[static_pwm_clock <name>]`：只消费一个 pin、建硬件 PWM，周期 `1/frequency`、占空 50%（start/shutdown 同值）、`max_duration = 0`；**不是 pin 芯片**（不注册 chip、无 `get_status`）；`order = 35`；build 期做 `MCU时钟 % frequency` 整除校验，余数报上游原样文案 `Frequency output must be without remainder`，脉宽越 40%–60% 只告警 |
| `duplicate_pin_override.rs` | `[duplicate_pin_override]`：`pins`（必填）逐个走 `PrinterPins::allow_multi_use_pin`，让同一个 pin 可被多处使用；描述**不得带修饰符**（`parse_pin` 的 `can_invert`/`can_pullup` 关闭，同上游 `pins.py:133`）；无资源/命令/`get_status`；`phase = early`、`order = 17`（在 `[mcu]`/`[multi_pin]`/`[sx1509]` 之后、泛型节之前） |
| `stepper.rs` | `[stepper_x]` / `[stepper_y]` / `[stepper_z]`（`phase = late`，order 50：`endstop_pin` 为虚拟端停时，`position_endstop` 取端停提供的位置——`PinChip::virtual_endstop_position`，上游 `MCU_endstop.get_position_endstop`，探针返回 `z_offset`；`endstop_pin` 可能指向别的段注册的 chip，见 [声明式表生成](codegen.md)）：一个电机在一根轴上。读 `step_pin` / `dir_pin` / `rotation_distance` / `microsteps` / `full_steps_per_rotation` / `gear_ratio` / `step_pulse_duration` 与行程（`position_min` / `position_max` / `position_endstop` / `endstop_pin` / `homing_*`），建 MCU 侧 stepper 资源与 rail ；**同一轨的兄弟电机一起被该轨端停刹车**（2026-10-18）：`Rail::lookup` 把不带自己 `endstop_pin` 的 `[stepper_z1]`… 注册进轨道端停的 `TriggerDispatch`（上游 `stepper.py:433-445`），`McuTrsync::add_stepper` 因此改为幂等（上游 `mcu.py:183-186`） |
| `stepper_enable.rs` | `[stepper_enable]`：读 `enable_pin`，管全部步进器的使能；注册 `M18` / `M84` / `SET_STEPPER_ENABLE STEPPER=… ENABLE=…`，广播 `stepper_enable:motor_off`；**自动使能链**（2026-10-18）：每个被管的 stepper 在装载期向运动队列的 active 回调表登记一条回调（`ActiveCallbacks`），`MotionQueuing::generate` 在生成前用 `itersolve` 的 `check_active` 判「已有未生成的真运动」，命中就把使能按那段运动的 `print_time` 排期下发（`queue_digital_out`，无时钟则回落立即写），并取出该回调（一次性）；`M84`/`M18` 失能后重新武装，所以下一次运动会再使能；`SET_STEPPER_ENABLE` 的日期取 `toolhead.get_last_move_time()`（上游先 `flush_step_generation` + dwell，本仓不 dwell）；`EnableTracking` 的 `register_state_callback`（签名为 `Fn(Option<f64>, bool)`）/ `is_motor_enabled` / `has_dedicated_enable` 为 `pub(crate)`（TMC 驱动的使能链路用；回调在 `set_motors_enable_inner` 特全局锁时就地执行，调用方必须推迟、不得在回调里回查） |
| `extruder.rs` | `[extruder]`（并连带读 `extruder1`…`extruder98` 兄弟节）：热端 + 挤出运动的 E 轴。读 `nozzle_diameter` / `filament_diameter` / `pressure_advance` / `pressure_advance_smooth_time` / `max_extrude_*` 等，经 `heaters::setup_heater` 建加热器，注册 `M104` / `M109` / `SET_PRESSURE_ADVANCE`（mux `EXTRUDER`）/ `ACTIVATE_EXTRUDER`；`M109` 已接等待（2026-10-03，`6900894`：`wait=true` 经 `PrinterHeaters::set_temperature`）；`ACTIVATE_EXTRUDER` 切换成功后发 `extruder:activate_extruder`（无载荷，消费者 `gcode_move` 重锚 E、`print_stats` 重锚耗材基准） |
| `heater_bed.rs` | `[heater_bed]`：读 heater 选项建床加热器（`setup_heater`，`gcode_id: B`），注册 `M140` / `M190`；`M190` 已接等待（2026-10-03，`6745e83`：`wait=true` 经 `PrinterHeaters::set_temperature`） |
| `heater_generic.rs` | `[heater_generic <name>]`：任意命名的加热器，读 `gcode_id`，其余走 `setup_heater` |
| `heaters.rs` | 传感器与加热器的注册表（上游 `[heaters]` 不是配置节）：`add_sensor_factory` / `setup_sensor` / `setup_heater` / `register_sensor`；`get_status` 报 `available_sensors` / `available_heaters` / `available_monitors`。`lookup_heater`（逐字 `Unknown heater '<name>'`）与 `Heater::get_temp` 为 `[verify_heater]` 补（批 #20）；`get_all_heaters`（上游同名，批 #31 为 `[homing_heaters]` 开放）。`ensure` 幂等地建出注册表，并拉起五个传感器工厂：`ds18b20` / `adc_temperature` / `temperature_mcu` / `spi_temperature` / `temperature_combined`；`register_sensor` 同处注册 `TEMPERATURE_WAIT SENSOR=` mux（2026-10-03，`b5da84e`）：`MINIMUM`/`MAXIMUM` 按上游顺序校验（文案逐字）、传感器三级解析（heaters 表 → `PrinterHeaterGeneric` → `lookup_object`）、1 s 轮询（`tokio` 定时器代本仓有意缺失的 `reactor.pause`）；**M105 gcode-id 表**（2026-10-03，`14f5055`）：`register_sensor(config, sensor, gcode_id)` 存 `BTreeMap<String, Arc<dyn GcodeTempSensor>>`，`M105`（`when_not_ready=true`）输出排序后的 `<id>:%.1f /%.1f` 列表（空表/`klippy:ready` 前回 `T:0`），`TEMPERATURE_WAIT` 等待循环与 `set_temperature(.., wait)` 每轮回真 `_get_temp` 报表；重复 gcode-id 报 `G-Code sensor id %s already registered`。`set_temperature(.., wait)` 等待循环、`HeaterControl` trait（`set_control` 交换控制环并清零 target，上游 `heaters.py:127-132`）与首台加热器经 `setup_heater` 装载 `pid_calibrate` 为 2026-10-03 随 `PID_CALIBRATE` 落地 |
| `pid_calibrate.rs` | `PID_CALIBRATE`（由首台加热器的 `setup_heater` 装载，无配置节）：`ControlAutoTune` 自整定（双相摆动 `TUNE_PID_DELTA=5`、12 峰判停、Åström–Hägglund 估 Ku/Tu、Ziegler–Nichols 出参）、respond_info 三行、`configfile.set` 四条暂存、`WRITE_FILE`→`/tmp/heattest.txt`、未跑满报 `pid_calibrate interrupted` 并交还原控制环（2026-10-03，上游 `pid_calibrate.py`） |
| `verify_heater.rs` | `[verify_heater <heater_name>]`（prefix）：`hysteresis` 5 / `max_error` 120 / `heating_gain` 2 / `check_gain_time` 60（bed）· 20（其他）；每秒检查 heater 是否按预期升温，失败 `invoke_shutdown("Heater <name> not heating at expected rate" + HINT_THERMAL)`；节由 `setup_heater` 经 `config.sibling` 认领（配置无该节也有默认检查器，批 #20） |
| `idle_timeout.rs` | `[idle_timeout]`：`timeout`（默认 600、`above 0`）与 `gcode`（默认 `DEFAULT_IDLE_GCODE`）、`SET_IDLE_TIMEOUT`、`get_status` 的 `state`/`printing_time`/`idle_timeout`，发 `idle_timeout:ready\|printing\|idle`（载荷 `{print_time}`，批 #21）；起印识别以 `toolhead:sync_print_time` 为准（`handle_sync_print_time` 按上游的错序签名绑定、`check_time = READY_TIMEOUT + est − print`、载荷 = `print_time + PIN_MIN_TIME`），`observe_print_time` 降为兵底且由 `seen_sync` 防止双发；**没有 `[idle_timeout]` 节也加载（2026-10-18）**：`load.rs` 的 `DEFAULT_MODULES` 按上游 `toolhead.py:604-614` 的无条件加载在阶段走完后补建缺失的默认模块，`timeout` 取默认 600 并进 `configfile.settings`——客户端宏无条件读 `printer.idle_timeout.state` |
| `adc_temperature.rs` | ADC→温度的传感器定义：`[thermistor <name>]`（`resistance1..N` / `temperature1..N` / `beta`）与 `[adc_temperature <name>]`（`voltage1..N`），以及内建的电压/电阻传感器（`PT1000`、`PT100 INA826` 与 `BUILTIN_THERMISTORS`）；裸 `[adc_temperature]` 是上游“装载默认值”的开关。**两族都声明为 `phase = early`**（`thermistor` order 20，批 #22；`adc_temperature` order 25，2026-10-03 `372cd93`）：自定义型号要在装载期被 `[extruder]`/`[heater_bed]`/`[heater_generic]` 的 `sensor_type` 解析前注册，且它们是 prefix-only、降 `order` 无效（见 codegen 相位通则） |
| `adc_scaled.rs` | `[adc_scaled <name>]`（**`phase = early`**）：`vref_pin`/`vssa_pin` 两路平滑参考 ADC（`smooth_time` 默认 2.0、`above=0.`），节名注册为虚拟 pin chip，每个消费者 ADC 包一层 `(raw - vssa)/(vref - vssa)`；`query_adc` 未实现（批 #19） |
| `temperature_sensor.rs` | `[temperature_sensor <name>]`：读 `sensor_type`（交给 `heaters` 查工厂）与 `min_temp` / `max_temp`，`get_status` 报 `temperature` / `measured_min_temp` / `measured_max_temp` |
| `temperature_mcu.rs` | 传感器工厂 `temperature_mcu`：MCU 自带的 ADC 温度通道（`cmd/debug.rs` 的 `debug_read` 读寄存器），标定数据在内；`sensor_mcu` 写**芯片名**（`[mcu downstream]` 的 `downstream`，经 `spi_device::mcu_object_name` 找到对象 `mcu downstream`，与上游 `'%s:ADC_TEMPERATURE' % mcu_name` 同口径） |
| `temperature_combined.rs` | 传感器工厂 `temperature_combined`：把多个传感器合成一个（上游同名），周期定时器在阈值越界时报警 |
| `temperature_probe.rs` | `[temperature_probe <name>]`（`temperature_probe.py`）：内层按 `sensor_type` 建传感器再包成对象（`register_sensor` 同上游），10 个探针选项 + `Polynomial2d`（求值/拟合）、`smooth_time` 平滑读数（默认 2.0 s）与 kick、`get_status` 六键、`stats`、`phase` 为 generic（order 30）；**A 核心**（`f6ea201`）与 **B 命令族**（2026-10-03，`dfaf418`：五条命令 + 标定状态机、手动探针会话、加热等待脚本、升温 kick 下一轮，`in_calibration` 驱动 A 的 `check_kick_next`）均已落地，只剩 `EddyCalibrationTool`（`PROBE_EDDY_CURRENT_CALIBRATE`）调用方缺席；**C 漂移补偿**（2026-10-03，`704ee0d`：`EddyDriftCompensation` 四选项/纯函数族/`finish` 九段 fit 写回，回填 B 留下的 3 处接缝与 ENABLE 接真，接线 `has_sibling("probe_eddy_current <同名>")`；`probe_eddy_current.rs` 新增 `DriftCompensation` trait + `register_drift_compensation`，`apply_calibration`/`freq_to_height`/`height_to_freq` 过 adjust/unadjust） |
| `multi_pin.rs` | `[multi_pin <name>]`（**`phase = early`**）：`pins`（必填、逗号分隔）把调用扇出到多个真实 pin；节名注册为虚拟 pin chip（重复注册的 `DuplicateChip` 吞掉），`multi_pin:<name>` 作为 lookup 值；`update_pwm` 等逐子 pin 转发，`next_aligned_clock` 原样返回（批 #26） |
| `respond.rs` | `[respond]`：`default_type`（`echo`/`command`/`error`，默认 `echo`）与 `default_prefix`；注册就绪前可用的 `M118`（原样透传）与 `RESPOND`（`TYPE`/`PREFIX`/`MSG`，含 `echo_no_space` 不加空格，批 #29） |
| `save_variables.rs` | `[save_variables]`：`filename` 必填（只展开 `~`/`~/`）→ 缺文件建空、`[Variables]` 段按 Python literal 读写（自实现子集：`None`/`True`/`False`/int/float/str/list/dict，字符串键）；`SAVE_VARIABLE` 要求全小写变量名，写盘后重载，`get_status` 返回 `{variables: {...}}`，宏经 `printer.save_variables.variables.<name>` 读取（2026-10-04） |
| `homing_heaters.rs` | `[homing_heaters]`：归零期间把选中 heater 目标置 0、结束后恢复（`heaters`/`steppers` 列表，`steppers` 的资格过滤未实现——本仓 homing 事件无载荷）；批 #31 |
| `firmware_retraction.rs` | `[firmware_retraction]`：`G10`/`G11` 与 `SET_RETRACTION`/`GET_RETRACTION`，经 `gcode_move` 的 `SAVE/RESTORE_GCODE_STATE` + `G1 E…`；`get_status` 报四个参数（批 #32） |
| `spi_temperature.rs` | 传感器工厂 `MAX6675` / `MAX31855` / `MAX31856` / `MAX31865`：SPI 热电偶/RTD，经 `cmd/thermocouple.rs` |
| `ad5206.rs` | `[ad5206 <name>]` 数字电位器（6 通道，SPI mode 0 @ 25 MHz，`enable_pin` 作 CS）：`scale`（默认 1.0、`above=0.`）与 `channel_1..6`（`minval=0.`、`maxval=scale`），写值 `int(val*256/scale+.5)`；写入经 MCU post-init 回调在 bring-up 时发出（批 #16） |
| `mcp4018.rs` | `[mcp4018 <name>]` 单路 I2C 数字电位器：`i2c_address` **默认 `0x2f`**、`scale` 默认 1、`wiper` 必填（`0..=scale`）；写 `int(v*127/scale+.5)` 单字节，`klippy:connect` 时 spawn 首写；`SET_DIGIPOT DIGIPOT=<name> [WIPER=]`（批 #33） |
| `mcp4451.rs` | `[mcp4451 <name>]` I2C 数字电位器（4 路）：`i2c_address` 必填且**仅 44..47**（否则 `mcp4451 address must be between 44 and 47`）、`scale`（默认 1.0）、`wiper_0..3`；先无条件写 `[0x40,0xff]`/`[0xa0,0xff]`，再按 `WiperRegisters=[0,1,6,7]` 写 `int(val*255/scale+.5)`；装载期写经 post-init 回调 spawn 异步 `i2c.write`（批 #25） |
| `dac084s085.rs` | `[dac084S085 <name>]` 四通道 SPI DAC（mode 1 @ 10 MHz，`enable_pin` 作 CS）：`scale`（默认 1.0、`above=0.`）与 `channel_A..D`（`minval=0.`、`maxval=scale`），写值 `int(val*255/scale)`（**截断**，与 ad5206 的 `+0.5` 不同）；`section!` 的 id 逐字 `dac084S085`（节名大小写敏感，批 #23） |
| `ds18b20.rs` | 传感器工厂 `DS18B20`：1-wire 温度传感器，读 `serial_no` / `sensor_mcu` / `ds18_report_time`，周期查询经 `cmd/ds18b20.rs` |
| `fan.rs` | `[fan]`：读 `pin` / `max_power` / `kick_start_time` / `off_below` / `cycle_time` / `hardware_pwm` / `shutdown_speed` / 可选 `enable_pin`；`tachometer_pin` 已接通（批 #8：读 `tachometer_ppr`/`tachometer_poll_interval`，报 `rpm`），注册 `M106` / `M107`；速度变更经前瞻钉入 `GCodeRequestQueue`（flush 按 clock 落 pin，kick-start 尾巴是队列 repeat），`set_speed`（heater_fan/controller_fan/temperature_fan/重启）走 `send_async_request`；两条立即兑底（无 toolhead／无法定时，兑底时 kick 才用 `call_later`）（2026-10-03） |
| `pulse_counter.rs` | `tachometer_pin` 频率计数（批 #8，上游 `pulse_counter.py` 无 section、不进工厂表）：`config_counter`/`query_counter` 装载、`counter_state` 按 oid 路由，`rpm` = Δcount/Δtime 换算 |
| `gcode_move.rs` | G-Code 坐标系（无配置节，由 `[printer]` 的装载拉起）：偏移、G90/G91、M82/M83、速度/挤出系数；注册 `G0`/`G1`/`G92`/`M114`/`GET_POSITION`/`SET_GCODE_OFFSET`/`SAVE_GCODE_STATE` 等，并把工具头坐标交给 toolhead；`axis_map` 由 `toolhead:update_extra_axes` 重建（初始 `{X:0,Y:1,Z:2,E:3}`，额外轴字母从 4 起，slot 0 恒为 E 不参与映射、`F`/`N`/小写/多字母被滤掉），`G1` 按字母寻址额外轴并交给 `MoveTarget::move_to_extended`；`GET_POSITION` 的 `mcu:` 行是「指令位置换算回步数」而非读固件计数，`kinematic:` 某轴不可解时逐轴回落为 `toolhead:` 值（同上游 `homing.py`） |
| `toolhead.rs` | `[printer]`（`object = "toolhead"`，`phase = late`）：读 `kinematics` / `max_velocity` / `max_accel` / `max_z_velocity` / `max_z_accel` / `square_corner_velocity`，建运动栈与 rail，注册 `G4` / `M400` / `G28` / `SET_KINEMATIC_POSITION`；提供命令内冲刷入口 `flush_step_generation`（take/put 独占，成功或失败都归还槽位，返回时先前入队的 move 已生成并交到 transport）、`set_position`（先 flush 再改位置并发 `toolhead:set_position`，同上游顺序）与 `z_stepper_names`（Z 轨电机名单，`z_tilt` 校验用）；提供探针式回零 `probing_move`（供 probe 族消费：`homing_move_begin` 先于采样、无触发报 `No trigger on probe after full movement`、零位移与亚纳米（<1e-9，与 `motion::plan::Move::new` 同口径）直接返回当前位置、不下发 endstop 命令；滴满整段后把工具头指令位置设到移动终点并返回——同上游 file-output 语义；M5d 起 `home_start` **之前**先排空 trapq 规划积压（防首帧 >monitor 窗口），`get_status` 增 `extruder` 键（`toolhead.py:511`），见 testing.md）；拉起 `gcode_move`、`manual_probe`（上游同一批默认模块，`toolhead.py:613-614`）与 `query_endstops`；生成带上游 BGFLUSH **地平线**（`horizon()`：est+0.4/0.7 s 批窗、只升不降，`motion_queuing.py:196-215`）——行为由语料钉住（iqex/itex 的半回绕越界、eddy 的 monitor 断供在无地平线上红，补齐后 237/0）；另提供两个 **connect-safe 注册口**（`GCodeRequestQueue` 的接线点）：`register_lookahead_callback`（lookahead 空则立即以 `get_last_move_time()` 回调，否则挂到最后一班 move、flush 时带其末尾时间）与 `register_flush_callback`（在 `MotionQueuing::generate` 开头按注册序触发，**无 stepper 也照跑**），未连接时挂 pending、`connect` 时一次性安装（同 `set_estimated_print_time_source` 的先例）；另注册 `SET_VELOCITY_LIMIT` 与 `M204`（运行期改四个上限，经 `motion::ToolHead::set_max_velocities` **同时**同步状态与规划器；junction deviation / MCR 公式收敛到 `motion/plan.rs` 的 `MoveLimits::from_velocity_limits` 一处），`get_status` 因此多出 `minimum_cruise_ratio` 与 `square_corner_velocity`；`manual_move` 为 `force_move` 提供 force-move 整段时序；**`M400` 会真的等 MCU 估计时钟追上**（2026-10-06）：`motion::ToolHead::wait_moves_state()` 给快照（`special_queuing_state` + `print_time` + est），等待环在 `extras/toolhead.rs`（每轮短锁读数、解锁后 100 ms sleep；四条有界出口：est 追上 / est 停滞 1 s 报错 / shutdown / 不可暂停），闸 `can_pause = !printer.is_fileoutput()` 对齐上游 `toolhead.py:221-223`、`:425-429` ；**生成地平线不再被回退**（2026-10-18）：后台 flush pass 的起点取 `max(pass 自己那份, ToolHead::last_step_gen_time())`——回零 drip 与探针的 flush 都在运动层抬高后者，只看前者会把 `itersolve.last_flush_time` 拽回去重走已发过的段（表现为 `stepcompress … Invalid sequence` 停机）；失败回零另补上游 `homing_move` 的收尾（`get_last_move_time` → `set_position(haltpos)` → `wipe_trapq`，且**不**标 homed） |
| `manual_probe.rs` | `[manual_probe]`：交互式 Z 高度探测。`MANUAL_PROBE` / `Z_ENDSTOP_CALIBRATE` 启动助手，动态注册 `ACCEPT`/`NEXT`/`ABORT`/`TESTZ`（结束即注销）、Z 先抬 `Z_BOB_MINIMUM` 再落、`TESTZ` 支持 `+`/`++`/`-`/`--` 二分与数值、状态报 `{is_active, z_position, z_position_lower, z_position_upper}`；`verify_no_manual_probe` 用 `unregister_command` 精确判定。**已落地（2026-10-06）**：`Z_OFFSET_APPLY_ENDSTOP`——普通机写 Z 轨 `position_endstop`、delta 机写三塔 `stepper_a/b/c`（上游将 delta 变体重注册到同名命令上覆盖，本仓因 `register_command` 拒绝重名而按 `kinematics: delta` 二选一，结果等价）。**未做**：z 位置取 kinematics 反算（现取指令位置，cartesian 等价） |
| `probe.rs` | `[probe]`：探针与虚拟 Z 端停。读 `pin` / `z_offset` / `x_offset` / `y_offset` / `speed` / `lift_speed` / `samples` / `sample_retract_dist` / `samples_result` / `samples_tolerance` / `samples_tolerance_retries` / `deactivate_on_each_sample` / `activate_gcode` / `deactivate_gcode`；用 `setup_endstop` 建物理探针端停，并以 `probe` 之名 `register_chip`，使 `endstop_pin: probe:z_virtual_endstop` 可解析（`setup_pin` 只认该名字、拒 `!`/`^`，文案同上游）。**`ProbePointsHelper`**（`z_tilt`/`quad_gantry_level`/`bed_tilt` 共用）按 `points` 逐点驱动探测与手动模式（`METHOD=manual` 经 `manual_probe` 的 ACCEPT 链），回调可返回 retry 整轮重来；会话 `ProbeSessionHelper` 做 samples 采样、`samples_tolerance` 重试与 median/average 归并，发 `probe:update_results`，并在 `gcode:command_error` 时收尾会话；注册 `QUERY_PROBE` / `PROBE` / `PROBE_ACCURACY`；`get_status` 报 `{name, last_query, last_z_result}`。**`ProbeSession` trait（M5b）**：`start_probe_session / run_probe / probe_params / pull_probed_results / end_probe_session / offsets`——`PrinterProbe` 为第一实现（委托原会话逻辑），`lookup_probe_session` → `LiveRound` 在 `start_probe` 处分发；**`SampleDelivery` seam**：会话实现、传感器生产者投样（真实生产者已由 M5d 接 eddy）；`HomingEndstop` 第二实现 `McuTriggerAnalog`（对接行在 resource 侧）；`lookup_probe_session` 已同时 downcast eddy 的 `PrinterEddyProbe`（M5d 双对象分发）。**未落地**：`Z_OFFSET_APPLY_PROBE`、endstop wrapper 的 *stow* 半边（`activate_gcode`/`deactivate_gcode` 与多探针状态机；**位置半边已完成**——`ProbeChip::virtual_endstop_position` 返回 `z_offset`，rail 优先采用，`pins.rs:396` / `extras/stepper.rs:374-406`）；注册 `QUERY_PROBE` / `PROBE` / `PROBE_ACCURACY` / `PROBE_CALIBRATE` / **`Z_OFFSET_APPLY_PROBE`（2026-10-06）**，`ProbePointsHelper` 已落地（被 z_tilt/QGL/bed_tilt/screws 四段消费） |
| `query_endstops.rs` | `query_endstops` 对象（由 `[printer]` 装载拉起，无配置节）：登记各 rail 的 endstop，注册 `QUERY_ENDSTOPS` / `M119`，`get_status` 报 `last_query` |
| `i2c_device.rs` | `[i2c_device <name>]`：原始 I2C 设备，经 `McuI2c`；注册 mux `IIC_WRITE` / `IIC_READ`（键 `DEVICE`），十六进制 `DATA=` 经 `bus_debug` |
| `smart_effector.rs` | `[smart_effector]`：`pin`+可选 `control_pin`（`probe_accel`/`z_offset` 等）；`SET_SMART_EFFECTER`（`SENSITIVITY`0..255/`ACCEL`≥0/`RECOVERY_TIME`≥0，缺省取当前值，文案保留上游 `accelartion` 拼写）与 `RESET_SMART_EFFECTOR`（仅配置 `control_pin` 时注册，按 1000 bits/s 发 `[131,131]` 帧）；注册为 `probe` 对象与虚拟 chip。**未做**：`probe_accel`/`recovery_time` 的 `probe_prepare/finish` 往返钩子（语料不触发） |
| `spi_device.rs` | `[spi_device <name>]`：原始 SPI 设备，经 `McuSpi`；注册 mux `SPI_TRANSFER` / `SPI_SEND`（键 `DEVICE`） |
| `bus_debug.rs` | `i2c_device` / `spi_device` 共用的调试命令底座：同步→异步桥与 `DATA=` 的十六进制编解码（无配置节） |
| `error_mcu.rs` | MCU 停机消息的展开（无配置节，第一个 `[mcu]` 拉起）：监听 `klippy:shutdown` / `klippy:analyze_shutdown`，把简短原因扩成原因+提示（上游 `extras/error_mcu.py`） |

`extras/` 的 107 个模块（106 `pub mod` + `pub(crate) bus_debug`）全部在 `extras/mod.rs` 声明；原 `extras/canbus_ids.rs`（CAN 节点号分配器）已**移到 `mcu/ids.rs`** 并泛化为全接口共用的 MCU 层 id 分配器，节名改为 `[mcu_ids]`、历史拼写 `[canbus_ids]` 保留为别名（两个工厂注册同一个对象，各算一个工厂 id）；其中 85 个文件声明了 `section!`（含 `printer`，声明在 `toolhead.rs`；`mcu` 声明在 `mcu/mod.rs`；两个桥端口节声明在 `mcu/bridge_ports.rs`），全仓共 124 个装载 id——与声明一一对应，由 `test_the_factory_table_is_in_load_order` 钉住（本分支实测重核为 85 文件 / 124 装载 id / 107 模块；旧文 104/103/80/117、82/120 与 84/122 均已过时）构成工厂表；`heaters` / `gcode_move` / `query_endstops` / `error_mcu` / `bus_debug` / `serial_ports` 等非节模块由上述模块按需 `ensure`，不占配置节。

### `api/` — 客户端 API 层

客户端一侧的入口，对应 klipper 的 `klippy/webhooks.py`：外部工具（Fluidd / Mainsail / Moonraker 等）连上 API server，发 `0x03` 分隔的 JSON 请求。监听位置由 `-a/--api-server` 给出：默认是 Unix Domain Socket 路径（与上游一致），写成 `tcp:<host>:<port>` 则监听 TCP；**不给这个选项就不起服务**，这一点也与上游一致。线上的形状（请求/应答、无 `id` 不应答、推送模板、错误文案）以 [Klippy API 参考](../third-party-dev/api-reference.md) 为准，两边要一起改。

主机这边是 `mod.rs`（说明 + 转出 `klippy-api` 的类型 + `register`）、`endpoints/`、`webhooks.rs` 与 `start_args.rs`。

| 文件 | 职责 |
|------|------|
| `mod.rs` | 说明主机侧与 API 的分界，把 `klippy-api` 的四个模块转出，并提供 `register`：一次把服务器这一侧（`webhooks` + 端点）装到机器上。端点来自各模块 `endpoint!` 声明生成的安装函数表（[声明式表生成](codegen.md)），`register` 只负责先装 `webhooks` 再遍历该表；mux 注册在建表这一刻从 `webhooks` 的 pending **抽干一次**（此后模块直连表）——所以 `register` 每进程只调一次，`Api`/`Server` 不重建 |
| `endpoints/` | 一个端点一个文件：`info.rs`、`emergency_stop.rs`、`objects_list.rs`、`objects_query.rs`、`objects_subscribe.rs`、`gcode.rs`、`query_endstops.rs`、`register_remote_method.rs`、`pause_resume.rs`、`bed_mesh_dump.rs`（共 16 条注册路径 + 内建 `list_endpoints` = 17 条，见 `api/mod.rs` 的测试断言）；参数、响应形状、handler 与安装函数都在各文件里。`objects/query` 与 `objects/subscribe` 共用字段选择（`select_fields` / `status_object`），`gcode/*` 按请求从 `printer` 里取 `gcode`（例外：`gcode/restart` / `gcode/firmware_restart` 在 `gcode` 缺席时直接 `request_exit`，让配置没装载成功时仍能重启）。未实现的表面（其余 `*/dump_*`）见 `endpoints/mod.rs` 的状态表 |
| `webhooks.rs` | 服务器自己的打印机对象：名字与字段对齐上游 `webhooks.get_status`，读的是机器状态；同时是 mux 表的生命周期驱动者——初始装载缓冲 `pending`、`api::register` 只抽干一次、`set_api` 后直连注册、`release_cycles`（teardown）时 `clear_mux()`，两条路径共用 `check_mux_conflict` 的检查与文案 |
| `start_args.rs` | 主机启动参数（`config_file` / `log_file` / `software_version` / `cpu_info`）：上游放在 printer 上（29 处 `get_start_args`），这里归主机侧，`info` 是第一个消费者；`software_version` 的值取自 `version.rs`（编译期钉死的上游基线 + `-` + crate 版本） |

API 本身在 `crates/klippy-api/src/`：

| 文件 | 职责 |
|------|------|
| `lib.rs` | crate 说明：线上的形状、监听位置、并发模型、模块表 |
| `address.rs` | `ApiTarget`：把 `--api-server` 的值解析成 socket 路径或 TCP 地址；未知 scheme（比如曾经的 `http://…:7125`）直接报错而不是当成文件名。`Transport` 也在这里：两个方向都只用它一个类型看待 socket。`DEFAULT_API_SERVER` / `NO_API_SERVER` 两个常量也在这里 —— 主机与客户端共用同一个默认值，而「空值＝不提供服务」只有主机认 |
| `protocol.rs` | `Framing`（粘包 / 拆包）、`Request` / `Response`、`Params` 访问器、`ApiError`、`ResponseTemplate`、`PushTarget`；不认 socket，也不认端点 |
| `registry.rs` | `Endpoint` / `MuxEndpoint` trait（`MuxEndpoint::detach` 默认空实现，`Api::clear_mux` 在配置重建时对每个实例调用）、`Api` 注册表与 `dispatch`、mux 的 key 选择（mux 表运行期可变：`register_mux(&self)` / `clear_mux`，分派只在读锁下解析、不跨 await 持锁）、`mux_registrations` 供 `webhooks` 查路径已有实例、remote method、内建 `list_endpoints`；注册期错误单独用 `RegistrationError` |
| `server.rs` | `Listener`（两种传输）、`Server::bind` / `run`（accept 循环）、`ClientConnection`（分帧状态、发件箱、`Notify` 唤醒、关闭标志，即端点拿到的 `PushTarget`），以及每条连接的读写 `select!` 与 5 秒写超时 |
| `error.rs` | `TransportError`：socket 层面的失败（`Bind` / `Connect` / `Closed` / `Io`），与请求层面的 `ApiError` 分开 |

端点自己不拼应答信封：它只返回 payload 或 `ApiError`，`id` 的回显与「无 `id` 就不应答」由 `protocol.rs` 一处决定，端点无从弄错。

### `motion/` — 运动栈

规划、梯形队列与步进生成：上游摊在 `toolhead.py` + C `chelper/` + `motion_quuing.py` 三处，
这里是按层拆开的一个模块。只被 `extras/toolhead.rs`（`[printer]`）与 `gcode_move` 消费。

| 文件 | 职责 |
|------|------|
| `plan.rs` | `Move` 与 `LookAheadQueue`：主机侧规划器，决定每个 move 的速度（前瞻、拐角、Z 限速） |
| `trapq.rs` | 梯形速度队列：步进生成器读它的段（含相续填补、过期提取） |
| `itersolve.rs` | 每个 stepper 的位置求解器：把轨迹变成步进时刻（含 cartesian/corexy/corexz 族的位置函数） |
| `stepcompress.rs` | 把步进时刻压成 `queue_step(interval, count, add)` 批（SDS 过滤、方向切换、history 回溯，与上游 C 实现对拍） |
| `stepper.rs` | 运动层眼中的 stepper：一个 stepper 的位置/历史/生成接口，与 MCU 侧资源对接；`trapq` 为 `Option<usize>`（脱挂态，`z_tilt` 的逐电机调整依赖它） |
| `toolhead.rs` | `ToolHead`：print time 跟踪、move 入队、dwell、`drip_move`（回零直灌）与 flush 调度；`Stepper` 可脱挂（`set_trapq(None)`），`MotionQueuing::generate` 跳过脱挂者；`register_lookahead_callback`（空队列立即以 `get_last_move_time()` 回调，否则记队列深度、`process_lookahead` 时挂回该 move）与 `register_flush_callback` 两个转发口 |
| `queuing.rs` | `MotionQueuing`：多个 trapq 的输出队列；`register_flush_callback` 注册的回调在 `generate(flush_time)` 开头按注册序触发（参数即 `flush_time`——上游 `cb(flush_time, step_gen_time)` 只跨缝传前一个），**无 stepper 也照跑**。flush 由 `extras/toolhead.rs` 的 10 ms tick 驱动（`horizon()` 地平线），不是上游的 reactor 定时器 |
| `kinematics.rs` | `Kinematics` / `HomingState` trait、`MoveContext`，以及 cartesian 族（`CartesianTransform`：Standard/CoreXy/CoreXz/Hybrid*）与 `NoneKinematics`（`kinematics: none`） |
| `extra.rs` | 额外轴（挤出机 E 轴）：不属于运动学、有自己 trapq 的轴 |
| `mod.rs` | 模块出口与公共类型 |

### `config/` — 配置解析、记录与校验

| 文件 | 职责 |
|------|------|
| `mod.rs` | INI 风格解析器：节/参数/注释/多行值/空节，与上游 `configparser` 行为对齐（节头行内注释、`:`/`=` 等价、缩进续行等四处曾分歧、已修）；选项名统一小写（`optionxform = str.lower`），节名与值保留原样 |
| `section.rs` | `ConfigSection`：一个节（id + sub + 参数）的存储与遍历（按插入序、按 id 过滤）；`get` / `get_str` / `get_text` / `has` 按小写查询，与存储侧对齐 |
| `value.rs` | `ConfigValue`：单行 / 多行值 |
| `source.rs` | 配置来源（文件路径）的表示 |
| `wrapper.rs` | `ConfigWrapper`：带**读取记录**的类型化视图（`get_*` 家族、`sibling` / `has_sibling`），读取记录就是 schema |
| `access.rs` | `AccessTracking`：每个节/选项谁读过的账本 |
| `validate.rs` | `check_unused`：装载末尾拒绝没人读过的节与选项（上游 `ConfigValidate.check_unused`） |
| `mcu.rs` | `[mcu]` 节的专用解析：传输键二选一（`serial`/`canbus_*`/`host_library`/`uart`/`can`/`test`），其中 `uart:`/`can:` 引用桥下游端口节（`can:` 必带 `uuid:`、`uart:` 无 `uuid:` 即 legacy，跨节规则由 `port_grouping::validate` 单源校验）；`baud`/`restart_method`/`usb_power` 校验，产出 `McuConfig`；host 臂另接受 `uuid:`/`canbus_uuid:`（指明该路径上被寻址的是哪块设备）并触发 host 库路径的复用判定（`mcu/host_libraries.rs`）；新变体 `Transport::BridgePort` 的 `open` 臂只报「走 `mcu/bridge_attach.rs`」的指路文案（真正开口在 `bridge_attach::open`，由 `mcu/object.rs::connect` 分派）；`McuConfig.host_mux` 由装载期（`mcu/object.rs::load_config` 打戳，做法同 `shared_baud`）决定该路径是走 `Interface::host_mux` 还是裸帧 `Interface::host`；`McuConfig.bridge_restart`（`BridgeRestart::{Device,Port}`）也在解析期定下（命名子节 + `restart_method: admin` ⇒ `Device`；legacy 子节 + 端口节 `uart_restart_method` ⇒ `Port`） |
| `object.rs` | `configfile` 打印机对象：面向客户端的配置状态与五种 `warnings` 形状；`set` / `remove_section` 记下待回写的 autosave 值（`save_config_pending` / `save_config_pending_items`），并同步维护块 fileconfig；`SAVE_CONFIG` 命令由装载器注册，把这些值写回文件并重启 |

越界报错的措辞与上游逐字一致（`configfile.py:49-59`）；限制值的数字写法见上面 `gcode.rs`
一节的「越界报错里限制值的写法」，是同一处**有意保留**差异。

回写进 `SAVE_CONFIG` 块的值文本不在此列：浮点走 `py_float_str`（Python 的 `str()` 写法，指数与
NaN 也归一），与上游写出的文件逐字一致，见用户手册 [配置文件参考](../user-manual/config.md)
的「序列化与回写」。

### `interface/` — 传输与设备

`Mcu` 只见 `Device` trait（`send` / 阻塞 `receive` / `shutdown`）；字节怎么走是下面各实现的事。

| 文件 | 职责 |
|------|------|
| `mod.rs` | `Interface`：在 `spawn_blocking` 里跑设备 I/O、持有机器 runtime handle（`with_transport` 是唯一的 ambient 捕获点）、`off_runtime`（取消 = 传输已关：`send` 拒发 `ConnectionLost`、`receive` 返 `None`）；`Interface` 可克隆共享设备 |
| `devices/serial.rs` | `SerialDevice`：tty 字节流（raw 模式、`termios2`、阻塞读）；独占串口唯一使用的设备。**共享串口（串口复用）也在这里**：`SharedSerialPort`/`SharedSerialHandle`——一个物理串口被多颗 MCU 共享，**串口复用协议的帧头与 KXID 伪帧编解码、分配期握手也在这里**（`assign.rs` 只留跨传输共享的仲裁 id 映射）——按路径复用同一个端口、端口自打开起**唯一 reader** 按 2 字节带校验帧头（10 数据位 + 4 检测位，只检不纠）分派到每颗 MCU 各自的 `FrameStream`、写路径由写锁串行化（单元内部不交错）、`shutdown` 只停本句柄、**`shutdown` 与句柄析构都注销自己的槽**（守卫式 `unregister`，见下；同 id 重 attach 无陈旧积压）、最后一个持有者析构才关端口并 join reader（若那次析构正跑在 reader 线程上——读者在 `dispatch` 期间短暂持强引用——则**跳过自 join** 让 reader 自然退出）；`attach(path, baud, mcu_id, timeout, uuid)` 做**分配期同步握手**（默认 500 ms × 3 次，期间在该 MCU 的待握手槽位收 `+1` 方向回执，用尽返回 `InterfaceError::Timeout` 并注销槽位；带 `uuid` 时先写一枚 `SET_KLIPPER_NODEID` 点名再发重确认请求），长度字节或头结构异常时按 `skip_to_sync` 语义重同步；**发现与指派 admin 信道**（`ADMIN_LISTEN` 0x3f0/`ADMIN_REPLY` 0x3f1，命令族 `0x00` 查询/`0x01` 指派/`0x20` 应答逐字镜像 CAN）：`discovery_query()` 拼广播查询、`UnassignedNode::from_unit` 解应答（uuid 大端 + application 三态）、`set_node_id_for(uuid, id)` 拼指派（id 越界返 `None`）、`UnassignedScan`/`serial_query_unassigned` 单次广播 + 截止窗口 + 按 uuid 去重，`klipperx scan --port` 就用它。（注册表与公共肋见 `devices/shared.rs`） |
| `devices/shared.rs` | **两个共享传输共用的复用肋** `resource_for`：按路径查 `Weak` 注册表，无活引用则 open 并登记，资源随最后一个 `Arc` 引用析构而关闭（注册表条目随之失效，下次连接=重新映射）。串口半（`port_for`）与 host 半（`library_for`）各自持有按路径的注册表并调用它；本模块**只**有这一条公共形状，不含任何线上逻辑 |
| `devices/host.rs` | **共享 host 库设备层** `HostLibrary`/`HostDevice`——`dlopen` klipper host 库，输入输出都是协议字节（测试逐字节、发布走整帧接口）；一个路径的库**只加载一次**（一个 runtime + 一个 reader），每颗 `[mcu] host_library:` 拿一个通讯引用共享它，`shutdown` 只停本引用，最后一个引用析构才 `klipper_host_shutdown` + join 线程 + 卸载（registry 里 `Weak` 失效，下次重新映射=断电重启）。**mux 路径（`OutputMode::Mux`）**：库的**唯一**读者就是路由器——按 `assign::mcu_id_from_arb` 把每个单元投到该 id 的引用自己的槽，无 id 的 admin 单元丢弃（与共享串口同构），因此桥自己的引用与下游子设备的引用**互不取字节**；**裸路径（`Default`/`Bytes`）**仍共用同一条输出队列（竞争消费，不是广播，无 id 可路由）（注册表与公共肋见 `devices/shared.rs`）；**字节流模式**：`load_bytes(path, identity)` 让 reader 线程在任何构建下都走 `klipper_host_output_wait`（整帧符号只在默认模式解析）、不喂 `FrameStream`、该模式下 `receive()` 返 `None`，并暴露原始 `read`/`write`（bench/扫描用）；**身份**由 `klipper_host_set_uuid()` 在 `init` 前投递，非全 `0` 身份在旧库上报错；同一路径的库在**模式**/整帧字节流、**身份**或**端口 tty 表**上不一致时拒绝复用（端口表不一致只能点名报错：`klipper_host_port()` 必须在 `init` 前调用；一份库一进程的限制见 `mcu/host_libraries.rs`）；另有 unit 收发接口（`klipper_host_input_unit`/`klipper_host_output_unit`，固件侧需帧接口开关 **且**桥模式）；**mux 路径的 attach**（`HostMux{identity, named}` + `HostDevice::load_mux` → `Interface::host_mux`）：`init` 前投递身份、发 `SET_KLIPPER_NODEID`、走 `assignment_request` 重确认握手（3×500 ms）、之后收发都带帧头并只收解回本 id 的单元 |
| `devices/canserial.rs` | `CanSerialDevice`：SocketCAN——承载的仍是同一份 serial 字节流（8 字节切 CAN 帧；`0x100+2n` 寻址取自 `assign.rs` 的共享映射、不再自带常量，开链路前拒绝越界节点号；admin 报文指派节点号）；**admin 扫描（2026-10-06）**：`CanbusAdminSocket` + `query_unassigned(bus, window, report)` 广播 `0x3f0/[0x00]`（`CMD_QUERY_UNASSIGNED`）并收 `0x3f1` 的 `RESP_NEED_NODEID`，`UnassignedNode`/`UnassignedScan` 按 uuid 去重（`AdminBus` 把收帧与计时抽成可注入接缝，窗口是「从广播起算的截止时间」而不是空闲超时）；**CAN 上游的端口级命令**（M1-P5）：`send_set_port_node_id`/`send_reset_device`/`send_reset_port` 把三条命令作为 mux `KXID` 单元放进同一条字节流（候选 B：零新帧形；单帧 admin 通道 `0x3f0` 与它无关） |
| `scan.rs` | `klipperx scan (--interface can0 \| --port <tty>) [--timeout 2] [--baud 250000]`：**一个工具覆盖两种总线**，传输必须显式给（无默认总线，裸调用打 help）——`--interface` 走 CAN（`canserial::query_unassigned`：`0x3f0/[0x00]` 广播、收 `0x3f1` 的 `RESP_NEED_NODEID`），给了就走串口（`serial::serial_query_unassigned`：线内 `discovery_query` 单元）。输出两者同形：`Found uuid=%012x, Application: …` 与总数行（`0x01`/无字节=Klipper、`0x11`=CanBoot）；两者同给、`--baud` 不给 `--port`、以及一个传输都不给，都在开端口/开 socket 前被拒（裸调用由 `arg_required_else_help` 打 help）。**只读广播，不改板子状态**，也不写配置文件 |；`--host-library <path.so>`（配 `--host-identity <12 位十六进制>`，默认全 `0`，**只在给了 `--host-library` 时可用**）扫一份 host 库——加载它、`init` 前投递身份，再走同一段广播/窗口/去重代码，三种传输输出同形 |
| `devices/simulator.rs` | `SimulatorDevice`（`test: dict=…`）：字典驱动的应答机——分块回 zlib 字典、记 `finalize_config` 的 CRC、回 ack、回答时钟，回归语料的端到端就跑在它上面；**步进时序模型**：per-oid 固件步进链（config/queue/reset 三态、`timer_is_before` u32 回绕、首拍过期/忙拒 → 字典 `static_string_id` 的 shutdown 帧）与固件侧样本续窗的 monitor——两会话复现（C5/Q10）的确定性假件；**多实例连机**：`link_machine(&[…])` 把多块板连成一台机器——本板步进置 `move_pending`、释放自身锁后把信号一跳转发给 peer（不回响，两板同时步进不互锁），peer 用被 arm 的自家时钟触发 armed 检查；未 link 的实例 `peers` 为空、行为与开销零变化（2026-10-03，`d14ce6a`） |
| `devices/responder_mcu.rs` | 响应器式假 MCU 句柄（可多实例）：`section()/sections()` 生成 `[mcu …] test: dict=` 块、bring_up 后 `mcu()/device()` 取回各自连接与实例、`link_machine()` 连机；取物全为 Option/Result，bring_up 阶段不 panic（否则跳过 teardown 会挂住运行时收尾）；两个 `#[cfg(test)]` 取物缝在 `Interface`/`Mcu` 上 |
| `devices/frame_mock.rs` | `FrameMock`：测试夹具——FIFO 精确帧比对 + 预设输出帧 |
| `pty.rs` | `posix_openpt` 开的 pty 对（仅 `cfg(test)`）：串口测试需要真内核 tty 时用 |
| `usb.rs` | `restart_method: rpi_usb` 的 USB 端口切电：sysfs 拓扑发现（tty→hub→端口号）与两种机制（sysfs `disable` / `nusb` 控制传输） |
| `error.rs` | `InterfaceError`：打开失败 / 关闭等传输层错误 |

### 零散顶层文件

| 文件 | 职责 |
|------|------|
| `mod.rs` / `test_util.rs` | 模块根：`trace_enabled()`（`KLIPPERX_TRACE`）与 **`sleep_off_worker(delay)`**——契约上同步的暂停点一律走它（多线程 runtime ⇒ `block_in_place` 交出 worker core，current-thread 或无 runtime ⇒ `std::thread::sleep`；直接 `block_in_place` 在 current-thread 上会 panic），纪律见[测试](testing.md)。`test_util.rs` 是 `#[cfg(test)]` 的测试共用助手：有界轮询 `wait_until(deadline, interval, pred, what)`（替掉「固定 sleep + 正向断言」） |
| `frame.rs` | `Frame` 与 `FrameStream`：Klipper 的帧格式（长度、序号、载荷、CRC、`0x7e` SYNC）与分包重组——**所有字节流设备共用**，新设备不要自己再实现一遍；另有 `Frame::with_wire_seq`/`encode_wire`（按给定 seq 字节原样出帧，供共享串口的**握手伪帧**使用）；`FrameStream` 不再有指派支路——解析失败即 `skip_to_sync` |
| `assign.rs` | **MCU id ↔ 仲裁 id 映射**（CAN 与共享串口共用的唯一定义）：`arb_id` = `0x100 + 2*mcu_id`、`reply_arb_id` = 应答方向（`+1`）、`mcu_id_from_arb` 把两个方向解回同一个 id、`is_reply_arb` 定方向；定义域 `1..=MAX_MCU_ID`（`0xff`），越界与 id 0 回 `None`。**传输专有的东西不在这里**：串口侧的 KXID 伪帧、2 字节带校验帧头（10 数据位 + 4 检测位）编解码与分配期同步握手在 `interface/devices/serial.rs`，CAN 侧的 admin 帧（`0x3f0` / `CMD_SET_NODEID`）在 `interface/devices/canserial.rs`，两者都只从本模块取映射 |
| `error.rs` | 主机错误词汇：`KlippyError`（Connection/Protocol/Request/Parse/Config/Internal）与 `ConfigError`——决定错误把机器带到哪个状态 |
| `mathutil.rs` | 运动栈共用的数值帮助（`Coord` 等，上游散在 `gcode.py`），以及 `coordinate_descent`（上游 `mathutil.py:16-49` 的近似最小二乘：步长 `1.0` 起、`>1e-5` 与万轮上限、改善 `*1.1`/不改善 `*0.9`）——H9 调平族 `z_tilt`/`bed_tilt` 平面拟合的依赖 |
| `upstream.rs` | 上游 `.test` 语料的 harness（`#[cfg(test)]`）：语料解析、缺口报告、`IGNORED` 守卫、端到端运行，见[回归测试](regression-tests.md) |
| `ui/` | 便利启动的两个服务：`klipperx mainsail`（静态托管；站点三层 `--dir` → **编进二进制的那份** → `third_party/mainsail/dist`，**不在本机编译前端**；`build.rs` 扫 `assets/mainsail/` 生成 `EMBEDDED_MAINSAIL`（`include_bytes!` 静态表，目录缺失或为空即 `None`））与 `klipperx moonraker`（自管 venv：`<venv>/klipperx-requirements.sha256` 记依赖表哈希，缺失或不一致才 `pip install`；源码解析 `--source` → `<data-path>/klipperx/moonraker`（首启从检出复制一次）→ 子模块）。HTTP 静态服务用现有 tokio 手写（不引入 crate）。**这是快速起环境的便利通道，不是生产部署路径**；发布时 `cp -r third_party/mainsail/dist/. assets/mainsail/ && cargo build --release`（`assets/mainsail/` 不入库） |
| `version.rs` | 本机上报的版本串 `version::software_version()` = **编译期钉死的上游基线** `UPSTREAM_KLIPPER_VERSION`（`v0.13.0-734-gfe4eb86`，上游 `git describe --tags --long` 的形状：标签 `v0.13.0` + 自标签起 **734** 个上游提交 + 基线提交 `fe4eb86`）+ `-` + `CARGO_PKG_VERSION`（暂不管理，`0.1.0`）。上游是运行时 `git describe --always --tags --long --dirty` 算出来的（`klippy/klippy.py:310-335`、`klippy/util.py:195-225`），本仓 worktree 里子模组是空目录、且本地克隆**一个标签都没有**（本地 describe 只会退回裸 sha），所以改为编译期钉死——重算步骤（基线提交、标签、计数双源核对）写在常量注释里。消费方：`api/start_args.rs`（`info` 的 `software_version`）、`printer.rs` 的默认值、`gcode.rs` 的 `M115` 兜底；启动横幅 `Versions: klipperx …` 仍报 crate 版本（那是 klipperx 自己的版本，不是被复刻的 klipper） |

## 二进制

| 二进制 | 入口 | 是什么 |
|--------|------|--------|
| `klipperx` | `src/main.rs` | 项目的 CLI：跑主机（默认，也写作 `klippy`）、`api`、`console` |
| `klippy` | `src/bin/klippy/main.rs` | 只有主机，等价于 `klipperx klippy`（名字取自上游的 `klippy.py`）；没有 `--tui`，不链接客户端与终端库（release 16.1 MB vs `klipperx` 18.3 MB，2026-10-03 实测） |
| `klippy-client` | `crates/klippy-client/src/main.rs` | 只有客户端，等价于 `klipperx api` / `klipperx console`；**自成一个包**，不编主机 |

`klipperx` 的顶层参数里嵌着一份 `AppArgs`（`Option<AppArgs>`，与 `klippy` 子命令同一类型、`args_conflicts_with_subcommands` 保证两者不能混用），所以不带子命令时 `klipperx printer.cfg` 就是 `klipperx klippy printer.cfg`。那个 `Option` 不是为了可空：clap 只有在整组参数可选时才会放过组内必填项（配置文件），否则 `klipperx api …` 会来要一个它根本不需要的配置文件。

参数定义全在库里（`klippy::AppArgs`、`klippy_client::{ApiArgs, ConsoleArgs}`），二进制只做三件事：解析命令行、装日志、把错误打成一行并以退出码 1 结束。后两个二进制只装载各自那部分，因此命令行与帮助文本是干净的。

`klippy` 与 `klipperx` 在同一个包里，共用一套依赖（模板引擎是 `minijinja 2.24`，启用 `custom_syntax` 与 `json` 特性）；`klippy-client` 在另一个包里，只依赖 `klippy-api` 与 clap / serde_json / tokio / tracing，所以它既不会编 `reqwest` / `flate2` / `libloading`，产物也小得多（实测 debug 63.5 MB vs 242.4 MB、release 3.9 MB vs 18.3 MB，2026-10-03）。

## 目录

- [消息编解码（msg）](message-structure.md) — `Msg` / `ArgType` / `ArgValue` / `Payload`
- [Parser API 参考](parser-api.md) — 注册、编码、解码、回调绑定
- [MCU 协议与数据字典](mcu-protocol.md) — `Mcu`、`Dictionary`、类型化调用、命令层与新增命令流程
- [MCU 配置构建（ConfigBuilder）](mcu-config.md) — 配置期的 oid、三张命令表、设备侧的配置与 CRC 校验（含与上游的差异）、两段式下发
- [Identify 机制](identify.md) — 主机与 MCU 间的数据字典协商流程
- [串口复用协议](serial-mux-protocol.md) — 多 MCU 共用一个串口时的帧头寻址、分配期握手，以及设备端要实现的线上规格
- [时钟与定时器（reactor）](reactor.md) — 机器的时钟、定时器契约，以及与上游 reactor/greenlet 的对应
- [事件系统](event-system.md) — 打印机级事件总线：事件清单、触发时序、编译期代码生成与处理器约定
- [声明式表生成（build.rs）](codegen.md) — 事件/段落/端点三类「声明在模块、聚合在编译期」的表的机制、语法与约束
- [运行时编排（机器与 API）](runtime.md) — 两个 runtime：机器专用、API 一个，以及边界约定与停机顺序
- [延迟与抖动（主机侧）](latency.md) — 抖动从哪来，以及要不要绑核（结论：先度量，不急）
- [压力测试（`klipperx stress`）](stress.md) — 给一块 MCU 逐步加大步进或链路负载，直到它出错
- [内部架构](architecture.md) — 传输层与 TRACE 日志的开启方式、收发任务、合并发送、路由优先级、性能特性
- [测试](testing.md) — 测试覆盖与运行方式
- [klippy 运行机制](klippy-runtime.md) — 对象图、状态机、reactor 回调模型、事件与数据流
- [回归测试（`.test` 与数据字典）](regression-tests.md) — 上游主机回归测试的「文件输出 + 数据字典」模式、语料结构与复用分层
- [本项目与上游的偏移](upstream-deviations.md) — 集中登记本仓与上游不一致之处（不做/不替代、行为偏移、暂缓项、计数清单）

---

- [← 文档首页](../../README.md)
