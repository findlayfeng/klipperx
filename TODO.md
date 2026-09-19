# TODO — 主机（klippy）侧的待办与未决问题

本文件只记**打算做什么**和**还没定的事**，不重复已经定下的设计（那些写在各模块文档和
`docs/klippy/developer-manual/` 里）。括号里的 `klippy/xxx.py:NN` 指上游参考实现
（`third_party/klipper/`），用来在动手前核对行为。已完成的条目只在下面留一行索引，
细节留在各模块自己的文档里。

## 已定

- **Printer 是一台机器，且只有单实现**：`src/core/klippy/printer.rs` 里就是一个 `Printer`
  结构体（无 trait、无工厂），一个主机进程只跑一个；机器拥有它的组成部分（config
  section 对应的 printer object）与生命周期；进程级的东西（命令行、日志、API server、
  runtime、重启循环）是它上面的一层。
- **`Kinematics` trait 与 `kinematics/` 已删**（连同 `load_kinematics` 工厂）：kinematics
  不是 Printer 的分类依据，它是 toolhead 持有的一个对象；**等实现 toolhead 时再加回**。
  上游证据：`klippy/toolhead.py:242` 是唯一装载点，`Printer` 类不知道 kinematics。
- **MCU 一侧的分层已定**：`msg → mcu → cmd`，`event` / `identify` 平级
  （`docs/klippy/developer-manual/architecture.md`、`README.md` 的分层表）。
- **上线（bring-up）是机器的，executor 是调用方的**：`PrinterObject::connect()` 返回
  一个 boxed `Future`（`std::future::Future`，不是 tokio 的），`Printer::bring_up()` 是
  async，同步的 `run()` 只等退出。机器因此不依赖任何 runtime，谁驱动 `bring_up` 谁带
  executor。上游的 `klippy:connect` handler 在这里被 `connect()` 方法取代，事件留给
  观察者。
- **组成部分由配置装载，装载顺序是契约**：`src/core/klippy/load.rs` 的静态工厂表
  （`load_config` / `load_config_prefix`）是唯一的 section → object 入口；主 section 先于
  前缀 section，各自按表序（`klippy/klippy.py:90-113`）。对象两段式构造：工厂只建对象并
  登记（注册键是 section identifier），`PrinterObject::connect()` 才解析 section、开设备。
  `webhooks` 由 `api::register` 在配置装载**之前**登记，所以 `objects/list` 从一而终以它
  开头（`klippy/klippy.py:36-40`）。
- **客户端 API 的线形状**以 `docs/klippy/third-party-dev/api-reference.md` 为准。

## 已完成（留档）

- **对象表与只读端点**：`add_object` / `objects` / `lookup_object` / `status_of`，
  `objects/list` 与 `objects/query`（`printer.rs`、`api/endpoints/objects_{list,query}.rs`），
  以及服务器侧的 `webhooks` 对象（`api/webhooks.rs`）；`api::register` 一次装完
  （`api/mod.rs`）。
- **`[mcu]` 住户**：`mcu::object::McuObject`，section 在 connect 时才解析、开设备、跑
  identify，`get_status` 报 identify 快照（`mcu/object.rs`）。
- **配置驱动装载**：静态工厂表、两段式构造顺序、section 级合法性校验
  （`load.rs`，`load_config` 是 `Printer` 的方法）。
- **主机层串起来**：`klippy_process`（建机器 → `api::register` → bind → `load_config`，
  失败即 `invoke_shutdown` → `bring_up` → `run`）、`info` 端点与 `StartArgs`
  （`src/klippy.rs`、`api/endpoints/info.rs`、`api/start_args.rs`）——重启循环除外，见 D2。
- **`run()` 的形态**：`bring_up()` async、`run()` 同步只等退出、机器只用
  `std::future::Future`，不起 tokio（`printer.rs`）。
- **时钟与定时器**：`reactor` trait（`monotonic` / `register_timer` / `unregister_timer` /
  `call_later`）与两个实现（主机 `TokioReactor`、测试 `ManualReactor`）；`Printer` 持有
  `Arc<dyn Reactor>`，`eventtime` 走它，机器不拥有 runtime（`reactor.rs`、
  `docs/klippy/developer-manual/reactor.md`）。
- **`objects/subscribe`**：请求立即回一份全量快照，随后每 0.25 s（`SUBSCRIPTION_REFRESH_TIME`，
  上游 `klippy/webhooks.py:467`）把变化的字段用 `response_template` 推给连接；连接关闭即
  退订、最后一个退订时定时器自停；`objects/query` 与它共用字段选择
  （`api/endpoints/objects_subscribe.rs`、`objects_query.rs`）。
- **MCU 配置构建层（F1）**：`ConfigBuilder` 的 oid 发号、`config` / `restart` / `init` 三张命令表、
  config 回调、CRC 与 `finalize_config`，以及 `configure()` 的 `get_config` 两段式下发；
  `McuObject` 在 connect 时把累积的配置交给固件（`mcu/config.rs`、`mcu/object.rs`）。
- **引脚解析与 `pins` 对象（F2）**：`PrinterPins` 的 `parse_pin` / `lookup_pin` / 共享与
  多用途，`PinResolver` 的别名与保留，`RESERVE_PINS_*` 在 connect 时预留；`pins` 注册但
  不可查询，为此 `PrinterObject` 加了 `is_queryable` / `queryable_objects`，并加了
  `lookup_object_as`（`src/core/klippy/pins.rs`、`printer.rs`、`mcu/object.rs`）。
- **GPIO 数字输出（F3 的 MCU 部分）**：`PinChip` / `DigitalOut` 接口与 `PrinterPins::setup_digital_out`
  派发；`McuChip` + `McuDigitalOut` 在 config 回调里把 pin 名→编号，发出 `config_digital_out`
  （含 `max_duration`、start/shutdown 约束）与 restart 的 `update_digital_out`，运行期可
  `queue_digital_out` / `update_digital_out`（`cmd/gpio.rs`、`mcu/pin.rs`）。`output_pin`
  消费者与 bus 同步输出等 gcode/运动层。
- **GCODE 调度器（G1）**：`GCodeDispatch`（命令表 / `register_mux_command` / `run_script` /
  输出处理器 / `get_status` 报命令表）与 `GcodeCommand`（传统 `S200` 与扩展 `KEY=VALUE`
  两套参数、`get_*` 的上游文案错误）、内置命令与未 ready 行为（`src/core/klippy/gcode.rs`）。
  在 `load_config` 里最先注册（`pins` 之前）。
- **`output_pin` 与 `SET_PIN`（G2）**：`[output_pin <name>]`（`pin` / `value` / `shutdown_value` /
  `maximum_mcu_duration`）用 `setup_digital_out` 建输出并注册 `SET_PIN PIN=… VALUE=…`；
  从 API 发 `gcode/script` 就能点灯（端点属 G3）。`pwm` 暂拒，`SET_PIN` 先立即生效
  （`extras/output_pin.rs`、`load.rs`）。
- **`gcode/*` 端点（G3 的四个）**：`gcode/help` / `gcode/script` / `gcode/restart` /
  `gcode/firmware_restart`；命令错误用新增的 `ApiError::CommandError`（不关停 klippy）；
  端点按请求从 `printer` 取 `gcode`（`api/endpoints/gcode.rs`、`crates/klippy-api/src/protocol.rs`）。
  这样**从 API 发 `SET_PIN` 点灯已经通了**。

## 待办

**当前选择：GCODE 驱动**。G1（调度器）、G2（`output_pin` + `SET_PIN`）与 G3 的四条端点已完成，
**已可从 API 发 `SET_PIN` 点灯**；剩 G3 的 `gcode/subscribe_output` 与 G4（运动命令，随 C1）。
执行层做不到的地方先用占位：`SET_PIN` 现为立即 `update_digital_out`（不排程），
`gcode/restart` 直接退出进程（无重启循环 D2）。

编号保留旧文件的 T/Q 以便对照，新增项给新号。依赖列的是**工具性前置**，不是自然顺序。

| # | 事项 | 依赖 |
|---|---|---|
| G3 | `gcode/subscribe_output`（其余 `gcode/*` 已落地） | G1 ✓ |
| G4 | 运动命令（G0/G1/G28…） | G1 ✓、C1 |
| A1b | reactor 串行调度器与延迟度量 | A1 ✓ |
| A2 | 错误词汇（`CommandError` / `ConfigError`） | — |
| B2 | MCU 关闭与错误上报（含 `last_stats`） | A2 |
| B4 | 其余端点（estop / remote method / pause_resume / …） | G3 等 |
| F3 | `MCU_bus_digital_out`（命令队列/运动同步输出） | C1 |
| F4 | PWM（硬件 / 软件） | F1 ✓、F2 ✓ |
| F5 | ADC | F1 ✓、F2 ✓ |
| F6 | SPI 总线 | F1 ✓、F2 ✓ |
| F7 | I2C 总线 | F1 ✓、F2 ✓ |
| F8 | endstop / trsync | F1 ✓、F2 ✓、C1 |
| F9 | 输入与外设资源（buttons / pulse_counter / …） | F1–F7 |
| C1 | toolhead 与 kinematics | — |
| C2 | 配置装载收尾（option 校验、第二个住户） | — |
| D1 | 主机层 start args / rollover / `--logfile` | — |
| D2 | 重启循环 | Q7 |
| E1 | 文档 | — |
| E2 | `python_path` 的取消 | 外部项目 |

### A1（旧 T3）reactor 抽象与定时器 —— 已完成

- [x] 最小 trait 与两个实现（`src/core/klippy/reactor.rs`）：`monotonic`、`register_timer`
      （回调收到事件时刻、返回下次唤醒时间或 `None`，上游 `klippy/reactor.py:145`
      `:157-172` 的契约）、`unregister_timer`、`call_later`（上游 `register_callback`
      `:187` 的一次性版）。机器拿 `Arc<dyn Reactor>`，主机传 `TokioReactor`（定时器是
      建在自己 runtime 上的 tokio 任务），测试传可手动拨表的 `ManualReactor`。
- [x] `Printer::eventtime` 改为读 reactor 的钟（`printer.rs`），并加 `Printer::reactor()`
      （上游 `get_reactor()`）；机器不再自己存 `Instant`。
- [x] 没有把上游的 `pause` / `completion` / greenlet 搬过来：在 async/await 里它们就是
      Future。未搬的还有 `update_timer`（武装/解除）、idle / latency 钩子、fd 事件 ——
      其中 **latency 钩子连同串行化排进 A1b**，其余等消费者，理由写在 `reactor.md` 的
      「还没有的」。
- 这一条是 B1（`objects/subscribe` 的 0.25 s 定时器）与将来 `idle_timeout` 的前置。

### A1b（A1 的收尾）reactor 的串行调度器与延迟度量

**为什么单列一条**：打印的**硬实时在 MCU**（步进脉冲由固件发，主机把 move 提前送进 MCU
的步进队列），主机只是**软实时**——只要不把队列喂空。但上游主机确定性的前提是
**单线程、按唤醒时间、一次一个回调、可观测**（`_check_timers`，`klippy/reactor.py:157-172`）。
现状的 `TokioReactor` 是「一个定时器一个 tokio 任务」+ 多线程 runtime，两个同时到期的回调
可以被两个 worker **并行**执行且不保证顺序；将来把 toolhead / trapq 这类运动状态放进定时
回调时，这会重新引入锁与竞态。MCU 队列给了余量，所以这不是「现在会坏」，而是「在把运动
状态交给定时器之前必须先补」。完整分析见 `docs/klippy/developer-manual/reactor.md`。

- [ ] **串行 dispatcher**：所有定时器进**一个 dispatcher 任务**（最小堆 + 一个
      `sleep_until`），按唤醒时间顺序出队、**一次跑一个回调**，复刻上游 `_check_timers`。
      trait 不变、机器代码不动——这正是 A1 把 reactor 做成 trait 的直接收益；可以替换
      `TokioReactor`，也可以作为它的一个变体。
- [ ] **机器与 API 分 runtime**：机器跑在专用 runtime（current-thread 或专用线程），API
      的每连接任务在别处，避免客户端流量影响运动时序。
- [ ] **延迟度量**：补上游的 `set_latency_notifier`（`klippy/reactor.py:316`）——一轮忙
      超过阈值就报「忙了多久、哪些回调拖的」（上游 `extras/garbage_collection.py:20` 的
      `_analyze_callback`）。`_recent_callbacks` 的等价物要在 dispatcher 里维护；我们已有
      `monotonic` 与定时器，挂得上。
- [ ] **关键回调不许 await / 阻塞**：上游用 `assert_no_pause`（`klippy/reactor.py:265`）在
      shutdown / ready 回调里禁止 pause。async 里没有 pause，但「这里不许 await、不许做
      重活」的语义仍要守（先文档约束，必要时再上机制）。
- [ ] **验收**：能测出定时器回调的唤醒延迟（`ManualReactor` 给确定性、真 runtime 给抖动
      数字），并确认并发回调不再可能同时碰同一份打印机状态。

### A2（旧 T6）错误词汇

- [ ] `CommandError` / `ConfigError`（上游 extras 里 94 / 41 处）；`KlippyError` 现在只有
      通信类 5 个变体（`src/core/klippy/error.rs`）。
- [ ] 借 `KlippyError::Internal` 的地方已经出现，是这条的验收点：`Printer::add_object`
      的重复名（`printer.rs` 的 TODO）、`load.rs` 的工厂拒绝与未认领 section（`load.rs`
      的 `TODO`）、`McuObject::connect` 的 section 解析。

### B1（旧 T1 剩余）objects/subscribe —— 已完成

- [x] 0.25 s 轮询 + `response_template`（`klippy/webhooks.py:482` 注册、`:561` 实现），
      推送走 `PushTarget`；`ResponseTemplate` 包住每次推送。端点：
      `src/core/klippy/api/endpoints/objects_subscribe.rs`，由 `api::register` 安装。
- [x] 只推变化：每个订阅自己记一份「上次推给它的值」（`Subscription::last`），tick 里与
      当次 `get_status` 比对；同一 tick 内每个对象只查一次，多个订阅者共用（上游的 `query`
      缓存）。`null` 字段列表展开为对象当时的字段；连接关闭即退订，定时器在最后一个退订时
      自停。
- [x] 两处有意与上游不同，写在模块文档里：回包在**请求时**就发（不等下一个 tick），且
      变化是相对**该连接上次所见**而非全局快照——本主机没有 pending-query tick，这两点
      在稳态下与上游一致。
- 详见 `docs/klippy/developer-manual/testing.md` 的 `objects_subscribe.rs` 一行。

### B2（新）MCU 关闭与错误上报

- [ ] **命令已定义但没人调用**：`emergency_stop` / `clear_shutdown` 在
      `cmd/shutdown.rs` 里，但没有任何 printer object、也没有端点发它们。MCU 停机时主机
      应发 `emergency_stop`，恢复时发 `clear_shutdown`（上游挂在 MCU 的 shutdown 处理上，
      `klippy/mcu.py:801-802` `:883`）。`emergency_stop` 端点本身见 B4。
- [ ] **`last_stats` 仍未报**：`stats` 事件现在只打日志（`event/stats.rs` 的
      `register_stats_logging`），所以 `McuObject::get_status` 只报三个 identify 字段。
      上游由 `MCUStatsHelper` 累计（`klippy/mcu.py:912` `:974-975`），`get_status` 多一个
      `last_stats`（`klippy/mcu.py:1235`）。需要先有 stats 消费者。
- [ ] **错误上报带载荷**：上游 `klippy:notify_mcu_error` 带 `msg` 与 details
      （`klippy/klippy.py:144` `:151`），shutdown 分析走 `klippy:analyze_shutdown`
      （`klippy/klippy.py:216-220`）。当前 `PrinterEvent` 的 handler 无参，表达不了，
      见 Q2 / Q3。
- [ ] **连接层停机标志与 `config_reset`**：上游 `_send_get_config` 先查连接层的
      `conn_helper.is_shutdown()`（收到 shutdown 消息时置位，`klippy/mcu.py:769-910`），
      再查 `get_config` 的 `is_shutdown` 字段；恢复路径用 `config_reset` 清 CRC/oid/运动队列
      （`MCUConfigHelper` 的 restart helper，`klippy/mcu.py:756-770`）。`config_reset` 命令
      类型已在 F1，**发送**它属于这里。

### G（新，先做）GCODE 驱动

上游参考：`klippy/gcode.py`（调度器）、`klippy/extras/output_pin.py`（第一个消费者）、
`klippy/webhooks.py:438-452`（端点）。**不依赖 toolhead**：toolhead 只是注册运动命令的
一个消费者。

#### G1 gcode 调度器 —— 已完成

实现：`src/core/klippy/gcode.rs`（`GCodeDispatch` / `GcodeCommand` / `CommandError`），
在 `load_config` 里**最先**注册（在 `pins` 之前）。上游 `klippy/gcode.py`。

- [x] `gcode` 作为 printer object，在 `load_config`、`pins` 之前注册（上游放
      `Printer.__init__` 早对象；我们 `webhooks` 由 `api::register` 先装，所以
      `objects/list` 的顺序是 webhooks→gcode）。
- [x] 命令表 `register_command(name, handler, desc, when_not_ready)`；非传统名做上游的
      合法性校验；重名报 `gcode command X already registered`。
- [x] `register_mux_command(cmd, key, value, handler, desc)`：一个 key、按值选处理器，
      未注册值报上游文案（选项列表排序以保证稳定）。
- [x] `run_script`：解析（传统 `S200` / 扩展 `KEY=VALUE` 带引号、行号、`;` 注释）、
      一条出错即停并回 `!!`，返回 `CommandError`。
- [x] 输出：`register_output_handler` / `respond_info`（`// ` 前缀）/ `respond_raw` /
      `respond_error`（`!! `）。
- [x] 未知命令报 `Unknown command:"..."`；未 ready 时报状态消息；内置 `M110` / `M112` /
      `M115` / `RESTART` / `FIRMWARE_RESTART` / `ECHO` / `STATUS` / `HELP`（`when_not_ready`）。
- [x] 与上游一致的 `get_status`：`{commands: {名: {help}}}`，所以 `gcode` 是**可查询**对象
      （早先记成不可查询是错的——上游 `GCodeDispatch.get_status` 就返回命令表）。
- 未做（不在 G1 范围）：`ok` 应答（文件输出协议）、`gcode:command_error` 事件（Q2）、
      `run_script` 的 reactor mutex、`M117/M118` 等特殊默认处理。

#### G2 `output_pin` 与 `SET_PIN`（真机点灯的入口）—— 已完成

实现：`src/core/klippy/extras/output_pin.rs`（上游 `klippy/extras/output_pin.py` 的
数字输出子集），工厂由 `load.rs` 的表接入。

- [x] `load.rs` 工厂表加 `[output_pin <name>]`（`load_config_prefix`）；加载测试证明
      真实 section 被认领（`[gcode, pins, mcu, output_pin fan]`）。
- [x] 选项：`pin`（必填）、`value`（默认 0）、`shutdown_value`（默认 0）、
      `maximum_mcu_duration`（默认 2 s），各自校验并报上游风格的配置错误。
- [x] 用 `PrinterPins::setup_digital_out` 建数字输出，把 start/shutdown/max_duration 设进去；
      以 section 的 sub 注册到 `SET_PIN` 的 mux（`PIN=<name>`）。
- [x] `SET_PIN PIN=<name> VALUE=<0..1>`：**现为立即 `update_digital_out`**（`>=0.5` 为开），
      上游的 `GCodeRequestQueue` 排程版留到时钟层与 C1（`queue_digital_out` 已能收绝对时钟）。
- [x] `get_status` 报 `value`（可查询 / 可订阅）。
- 未做：`pwm` / `cycle_time`（F4，现在**显式拒绝**而不是当数字输出）、
      `scale` / `static_value` / `template`（display 模板）。

#### G3 `gcode/*` 端点 —— 四条已落地，剩 `subscribe_output`

实现：`src/core/klippy/api/endpoints/gcode.rs`，在 `api::register` 里装上。端点**按请求**
从 `printer` 取 `gcode`（`load_config` 在 `api::register` 之后才建它；取不到时报打印机状态）。

- [x] `gcode/help`（`klippy/webhooks.py:438`）：返回扁平的 `{命令: 帮助}`。
- [x] `gcode/script`（`:439`）：`run_script`；命令级错误作为 `error` 回（新增
      `ApiError::CommandError`，**不关停 klippy**），成功回 `{}`。
- [x] `gcode/restart` / `gcode/firmware_restart`（`:440-442`）：跑内置 `RESTART` /
      `FIRMWARE_RESTART`（= `request_exit`），主机侧语义接 D2（现在会退出进程）。
- [ ] `gcode/subscribe_output`（`:443-444`）：把输出处理器接到发起请求的连接（`PushTarget`）
      并推 `{response: line}`。需要**可移除的输出处理器**（连接关闭时摘掉），
      现在的 `register_output_handler` 只增不减。

#### G4 运动命令（G0/G1/G28/G92/M114…）

- [ ] 由 toolhead 注册，随 **C1**；gcode 层不需为它们改什么，只要命令表够通用
      （含 `register_mux_command`，给 `SET_PIN` 这类 `PIN=` 选择用）。

### B4（新）其余端点

`api-reference.md` 有、`endpoints/mod.rs` 的表里标「not started」的其余部分，各自等它读的
对象先存在：

- [ ] `emergency_stop`（`klippy/webhooks.py:322` `_handle_estop_request`）。
- [ ] `register_remote_method`：方法表与推送（`klippy/webhooks.py:319` `:323` `:391`
      `:412`）。
- [ ] `pause_resume/{pause,resume,cancel}`：等 `pause_resume` 对象。
- [ ] `query_endstops/status`：等 endstop / homing。
- [ ] `bed_mesh/dump_mesh` 与 `*/dump_*` 多路复用端点（`klippy/webhooks.py:335`
      `_handle_mux`）：等对应 extras（`bed_mesh`、`adxl345` 等）。

### F（新）MCU 基础资源：GPIO / SPI / I2C / ADC / PWM

上游把这些叫 printer objects 下面的「资源」：主机用一个 **oid** 和一个 **pin 描述**
建立资源对象，把 `config_*` 命令攒起来，在 `finalize_config` 之前算一个 CRC 一次性下发，
之后用 `queue_*` / `set_*` / `*_transfer` 命令驱动。命令层（`allocate_oids` / `get_config` /
`finalize_config` / `get_uptime` / `emergency_stop` / `get_clock`）已就位，**F1 已把 oid 发号、
config 命令累积与 CRC、两段式下发补齐**；剩下的缺口是 **pin 解析（F2）与任何一个 `config_*`
资源（F3–F9）**，所以真实 printer.cfg 里带引脚的东西还接不上。

F 组的 **F2–F9 都依赖 F1（已完成）**，F3–F9 还需 F2（pin 解析）才能把引脚填进命令。

#### F1 MCU 配置构建层（oid / config 命令 / CRC）—— 已完成

上游 `MCUConfigHelper`（`klippy/mcu.py:979-1143`）。实现在 `src/core/klippy/mcu/config.rs`
的 `ConfigBuilder`，由 `McuObject` 在**建对象时**持有、在 connect（identify 之后）时
`configure()`：

- [x] **oid 计数**：`create_oid()` 单调从 0 发号（上游 `:1118`），`build` 把
      `allocate_oids count=N` 插在最前（`:1004-1020`）；走完 `MAX_OIDS`（255）报错不回绕，
      定稿后不能再领。
- [x] **config 命令累积**：`add_config_cmd` / `add_restart_cmd` / `add_init_cmd` 三张表
      （`:1125`），`register_config_callback`（在 build 时跑，可继续领 oid/加命令，拿到 `&Mcu`
      故可用 `seconds_to_clock`）与 `register_post_init_callback`。
- [x] **CRC 与 finalize**：`build` 跑回调 → 插 `allocate_oids` → 对 `config` 列表的**编码字节**
      算 CRC-32 → 追加 `finalize_config crc=`。**与上游不同**：上游哈希命令文本，我们没有命令
      文本，改为哈希 wire 字节；固件只存不算，所以只要自洽就行（模块文档里写清楚）。
      上游的 pin 名改写属于 F2。
- [x] **两段式下发**：`configure()` 先 `get_config`；未配置则送 `config + init`，已配置且 CRC
      一致则只送 `restart + init`，CRC 不一致报错（重启路径见 D2）；再问一次，检查
      `move_count`，跑 post-init 回调（`:1047-1085`）。
- [x] **`seconds_to_clock`**：`(seconds * CLOCK_FREQ)`，从字典读常量（`:1140`）。`get_query_slot`
      （`:1136`）需要 print-time 时钟，留给它的消费者（ADC / endstop）一起做。
- [x] **`request_move_queue_slot`**（`:1142`）：预留运动队列槽位，`configure` 用 `move_count` 对账。
- [x] **`config_reset`** 命令类型（无参数；`src/basecmd.c:262`，声明在各板子的 `main.c`，如
      `src/linux/main.c:59`）。发送它的 shutdown 恢复路径仍属于 B2。
- 详见 `docs/klippy/developer-manual/testing.md` 的 `config.rs` 一行。

#### F2 pin 解析与 `pins` 对象 —— 已完成

主机侧的引脚词汇，独立于任何具体资源：上游 `klippy/pins.py`（`PrinterPins` `:60`、
`PinResolver` `:18`）。实现在 `src/core/klippy/pins.rs`，`load_config` 在加载任何 section
之前先注册 `pins`。

- [x] `parse_pin`：`[chip:]pin` 描述，`!` 取反、`^`/`~` 上拉（`:67-95`）；修饰只在
      `PinType` 允许时生效；未知 chip、畸形描述（带上格式提示）报上游原文。
- [x] `lookup_pin`：同一 pin 重复使用要同 `share_type` 且极性一致，否则报
      `pin X used multiple times in config`；`allow_multi_use_pin` / `reset_pin_sharing`
      是例外口子（`:96-119`）。
- [x] `PinResolver`：`reserve_pin` / `alias_pin` / `resolve` —— `resolve` 是上游
      `update_command`（`:41-49`）去掉文本改写后的部分：跟别名、报“pin X is an alias for Y”、
      拒绝保留引脚。`RESERVE_PINS_*` 在 `McuObject::connect` 里预留
      （上游 `klippy/mcu.py:1091-1100`）。
- [x] `pins` 作为 printer object 注册（`pins.py:137`），**但不可查询**：上游
      `objects/list` 只留带 `get_status` 的对象，为此给 `PrinterObject` 加了
      `is_queryable`（默认 true）与 `queryable_objects()`，并顺手加了 `lookup_object_as`
      （按名取回具体类型，上游 `printer.lookup_object('pins')` 的 Rust 写法）。
      报错文案对齐：`Pin 'X' is not a valid pin name on mcu 'Y'`（`mcu.py:1021-1032`）
      属于数字解析那一步（见下）。

**本条未做、已拆分出去的**：

- [x] **`setup_pin` 的资源派发**（`pins.py:114-117`）：已在 **F3** 落地（`PrinterPins::setup_digital_out`
      → `PinChip` → `McuDigitalOut`）；PWM / ADC / endstop 随 F4 / F5 / F8 各自加一个方法。
- [x] **数字解析**：已在 **F3** 落地——`McuDigitalOut` 在 config 回调（build 时、有字典）里用
      `pin` 枚举把名字换成编号。
- [ ] **`[board_pins]`**（`klippy/extras/board_pins.py`）：调用 `alias_pin` / `reserve_pin` 的
      section，需要 config 的 list 解析与一个新工厂项；解析器 API 已就绪，section 随配置装载
      （C2）一起接。
- [ ] **`BUS_PINS_<bus>`**：由 SPI/I2C 在开总线时预留（`klippy/extras/bus.py:9-32`），随 F6/F7。

#### F3 GPIO 数字输出 —— 已完成（bus 同步输出等 C1）

- [x] **接上芯片派发**（F2 留下的）：`PrinterPins::setup_digital_out` 把校验过的 `PinParams`
      交给 chip；chip 接口是 `PinChip`（每种资源一个方法），`McuChip`（`mcu/pin.rs`）实现它，
      对应上游 `klippy/mcu.py:1111-1116` 的 `pcs` 表。
- [x] **pin 名 → 编号**：`McuDigitalOut` 在 config 回调里用字典的 `pin` 枚举把名字换成数字；
      未知名字报 `Pin 'X' is not a valid pin name on mcu 'Y'`（`klippy/mcu.py:1021-1032`）。
      别名/保留的解析在同一个回调里、在枚举之前。
- [x] `MCU_digital_out`（`klippy/mcu.py:408-449`）：`config_digital_out oid=%c pin=%u
      value=%c default_value=%c max_duration=%u` + 重启时的 `update_digital_out
      oid=%c value=%c` + 运行期 `queue_digital_out oid=%c clock=%u on_ticks=%u`。
      命令类型在 `cmd/gpio.rs`，资源在 `mcu/pin.rs`；固件 `src/gpiocmds.c:127` `:174` `:195`。
      `max_duration` 的 start==shutdown 约束与 `MAX_SCHEDULE_TICKS` 上限已实现。
- [ ] **`MCU_bus_digital_out`**（`klippy/extras/bus.py:337` 以后）：挂在命令队列上、与运动
      同步的输出；需要命令队列/运动层（C1）。
- [x] 上位消费者 `output_pin` 的**前置**已具备：`PrinterPins::setup_digital_out` 返回
      `McuDigitalOut`。section 本身移到 **G2**（它是 gcode 驱动的第一个消费者）。
- 运行期 `queue_digital_out` 收的是**绝对固件时钟**；print_time → clock 的换算属于时钟层
      （`cmd/clock.rs` 的 `ClockSync` 现只有 `get_clock`，偏移跟踪未做）。

#### F4 PWM（硬件 / 软件）

上游 `MCU_pwm`（`klippy/mcu.py:451-553`）：

- [ ] **硬件**：`config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu
      max_duration=%u` + `queue_pwm_out oid=%c clock=%u value=%hu`
      （固件 `src/pwmcmds.c:78` `:105`），满量程取常量 `PWM_MAX`。
- [ ] **软件**：没有硬件 PWM 时用 `config_digital_out` + `set_digital_out_pwm_cycle
      oid=%c cycle_ticks=%u` + `queue_digital_out`（固件 `src/gpiocmds.c:141` `:174`），
      满量程是 `cycle_ticks`。
- [ ] `next_aligned_print_time`（`:531`）：软件 PWM 的值变化要对齐到周期边界，不能任意时刻改。
- [ ] `pin_type` 是 `pwm`，可翻转；`shutdown_value` 在软件 PWM 下必须是 0 或 1。

#### F5 ADC

上游 `MCU_adc`（`klippy/mcu.py:555-655`）：

- [ ] `config_analog_in oid=%c pin=%u` + 周期查询 `query_analog_in oid=%c clock=%u
      sample_ticks=%u sample_count=%c rest_ticks=%u bytes_per_report=%c min_value=%hu
      max_value=%hu range_check_count=%c`，回应是 `analog_in_state oid=%c next_clock=%u
      value=%hu`（固件 `src/adccmds.c:75` `:100`）。
- [ ] 采样批处理（`batch_num`）与旧格式兼容分支：上游先试 `bytes_per_report`，拿不到就退回
      一次性 `sample_count`（`:619-655`）。我们的字典是运行期下发的，所以“有没有这条命令”
      可以直接用 `Dictionary::message` 判断。
- [ ] 满量程 `ADC_MAX` 常量、`sample_count * ADC_MAX < 2^16` 的上限、`get_query_slot` 的
      查询相位。消费者：`thermistor` / `adc_temperature` / `temperature_sensor`。

#### F6 SPI 总线

上游 `MCU_SPI`（`klippy/extras/bus.py:42-155`）：

- [ ] 设备侧：`config_spi oid=%c pin=%u cs_active_high=%c`（或 `config_spi_without_cs`），
      总线侧：`spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u`，收发：
      `spi_send oid=%c data=%*s`、`spi_transfer oid=%c data=%*s` /
      `spi_transfer_response oid=%c response=%*s`；还有 `config_spi_shutdown`
      （固件 `src/spicmds.c:37` `:62` `:122` `:157`）。
- [ ] `resolve_bus_name`（`bus.py:9-32`）：从字典的 `spi_bus`（或通用 `bus`）枚举里取总线号；
      没写 `spi_bus` 时要求总线 0 在枚举里，否则报 `Must specify spi_bus on mcu 'X'`；未知总线报
      `Unknown spi_bus 'X'`。
- [ ] 软件 SPI（`spi_software_{miso,mosi,sclk}_pin`）：`spi_set_sw_bus`（新）/ 
      `spi_set_software_bus`（旧），固件 `src/spi_software.c`。
- [ ] `MCU_SPI_from_config`（`bus.py:124`）：从 section 读 `cs_pin` / `spi_speed` /
      `spi_bus` / 软件引脚，`cs_pin=None` 时不占用引脚的共享。

#### F7 I2C 总线

上游 `MCU_I2C`（`klippy/extras/bus.py:161` 以后）：

- [ ] 设备侧：`config_i2c oid=%c`，总线侧：`i2c_set_bus oid=%c i2c_bus=%u rate=%u
      address=%u`；传输：`i2c_transfer oid=%c write=%*s read_len=%u` /
      `i2c_response oid=%c i2c_bus_status=%c response=%*s`，或新式的 `i2c_write` /
      `i2c_read` + `i2c_read_response`（固件 `src/i2ccmds.c:32` `:48` `:107`）。
- [ ] `i2c_bus_status` 不是 `SUCCESS` 时上游会 `invoke_shutdown`
      （`bus.py:295-300`）；`i2c_write` 的 retry 与 `async_write_only` 是可选分支。
- [ ] 软件 I2C（`i2c_software_{scl,sda}_pin`）：`i2c_set_sw_bus`，固件 `src/i2c_software.c`。

#### F8 endstop / trsync（与 C1 共享）

- [ ] `MCU_endstop`（`klippy/mcu.py:340-407`）：`config_endstop oid=%c pin=%c pull_up=%c`、
      回零 `endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u
      pin_value=%c trsync_oid=%c trigger_reason=%c`、查询 `endstop_query_state oid=%c` /
      `endstop_state oid=%c homing=%c next_clock=%u pin_value=%c`（固件 `src/endstop.c:72`
      `:97` `:115`）。
- [ ] `MCU_trsync` / `TriggerDispatch`（`mcu.py:155-339`）：多 MCU 同步触发
      （`src/trsync.c`），回零结束时用来同时停各个轴。这是 C1 回零的直接前置。
- [ ] 消费者是 `homing`（`klippy/extras/homing.py`），所以这条要等 toolhead 的接口
      （`home_rails` / `get_trigger_position`，见 C1）一起定。

#### F9 其他输入与外设资源

建立在 F1–F6 之上，各自一个 `config_*` + 查询/事件：

- [ ] `buttons`（`src/buttons.c`，`config_buttons` / `buttons_add` / `buttons_query` /
      `buttons_ack`）—— 暂停/恢复按钮、耗材检测。
- [ ] `pulse_counter`、`neopixel` / `dotstar` / `led`、`tmcuart`、`sdcard` / `sdio`、
      `lcd_hd44780` / `lcd_st7920`、`sensor_bulk`（批量传感器上报）与各类 SPI/I2C 传感器
      （`sensor_adxl345` / `sensor_lis2dw` / …）。
- 这些是 extras，不阻塞运动；等 F1–F6 完成、真有对应 section 时再逐个接。

### C1（旧 T4）toolhead 与 kinematics

kinematics 已随 Printer 重构删除，从这里重新开始：

- [ ] 先立 **toolhead 对象**：位置记忆（`commanded_pos`）、trapq、速度/加速度上限，
      回零与移动的入口（上游 `klippy/toolhead.py:389` `:400` `:482` `:507` `:522`）。
- [ ] 再加回 **`Kinematics` trait 与 `kinematics/`**：按上游由 toolhead 读
      `[printer] kinematics` 装载（`klippy/toolhead.py:242`），不是交给 Printer。
- [ ] `calc_position` 的返回类型：上游允许逐轴为 `None`
      （`extras/homing.py:245` 判空，`mathutil.py:152` 的 `gaussian_solve` 会返回
      `None`），`Coord { x: f64, .. }`（已随 kinematics 一起删除）需要重新定形状。
- [ ] 回零协议：上游 kinematics 调 `homing_state.home_rails(rails, forcepos, movepos)`、
      `set_homed_position(pos)`、`get_trigger_position`、`set_stepper_adjustment`。
      没有这些，任何真实 kinematics 的 `home()` 都写不出来。
- [ ] stepper 句柄：上游能 `get_commanded_position()` / `get_step_dist()` / `set_trapq()` /
      `setup_itersolve()`；`calc_position` 的输入就从这里来。
- [ ] step 生成层的运动学（上游 `rail.setup_itersolve('cartesian_stepper_alloc', axis)`、
      `kinematics/kinematic_stepper.py`）在我们这儿还没有对应物，运动规划整个未开始。

### C2（旧 T5 剩余）配置装载收尾

- [ ] **option 级校验**：上游拿访问追踪当 schema（`klippy/configfile.py:435-441`），
      `ConfigSection` 还没有访问记录，未做。
- [ ] **住户只有 MCU**：上游在 `_read_config` 里显式加载的 `pins` / `configfile` /
      `toolhead` 还没有入口，所以任何真实 printer.cfg 现在都会在未认领的 section 上报错；
      第二个住户进来时按同一张表补（C1 的 toolhead 就是下一个）。

### D1（旧 T2 剩余）主机层 start args / rollover / 日志

- [ ] `StartArgs` 只有 `info` 需要的四个字段（`api/start_args.rs`）；上游的
      `apiserver`、`start_reason`、debug 输入输出、每个 MCU 的字典路径还没进来。
- [ ] rollover info：上游 `set_rollover_info` 7 处（`klippy/klippy.py:369` 起），给 `info`
      与日志用；归主机层，不进机器。
- [ ] `--logfile`：现在没有，`log_file` 恒为 `null`（`api/start_args.rs`）；先有写文件的
      日志层，rollover 才有意义。

### D2（旧 Q7 的落地）重启循环

- [ ] 现在 `firmware_restart` / `restart` 只记录并退出：`klippy_process` 拿到 `run()` 的
      结果只 `debug!`（`src/klippy.rs`），没有任何东西按结果重建机器。上游的主循环在
      `klippy/klippy.py:355-370` 按 `res` 决定退出还是 `time.sleep(1.)` 后重建。
- [ ] 前置是 Q7（API 与打印机的关系）；`start_reason`（D1）也要跟着这条进来。
- [ ] **CRC 不匹配时重启**：上游发现已配置但 CRC 不一致时，先
      `request_exit('firmware_restart')`（`check_restart_on_crc_mismatch`，
      `klippy/mcu.py:678-685`、`:1057-1059`），**不是**重发配置——`finalize_config`
      已锁住固件（第二次会 `Already finalized`）。重启方法按 `restart_method` 分派
      （`:756-770`），也就是 `McuConfig.restart_method` 的第一个读者。另有
      `start_reason == 'firmware_restart'` 却仍已配置时 raise “Failed automated reset”
      的前置门（`:1053-1056`）。详见 `docs/klippy/developer-manual/mcu-config.md`。

### E1（旧 T7）文档

- [ ] `docs/klippy/developer-manual/`：补 printer 一节，并在 README 的分层表里加
      `printer` 一行（现在只有 msg / mcu / cmd / event / identify / api）。
- [ ] **过期描述**：`printer.rs` 头注释仍写「No part is loaded from the config into it
      yet … so it still runs empty」，而 `load.rs` 已经装载 `[mcu]`；改了代码就要回头改
      这几句。

### E2（旧 T8）`python_path` 的取消（**远期，依赖外部项目**）

- [ ] **现状**：`info` 里这个字段只被 Moonraker 使用，且它把它当 Klipper 的
      virtualenv 解释器（`update_manager/app_deploy.py` 的 `_configure_virtualenv`），
      因此我们的主机目前只能报一个**不存在的路径**（让它的 Klipper 更新项退化为
      no-op），不能报自己的二进制（会让 Moonraker 报 `Invalid virtualenv` 而起不来）。
      字段本身还必须存在：它直接下标取 `self._klippy_info["python_path"]`
      （`klippy_connection.py` 的 `_save_path_info`）。
- [ ] **想做的**：这是一个 Klipper 实现细节，本主机没有解释器也没有 Klipper 源码树，
      报一个不存在（或任何）路径都是在编数据；理想是**取消**这个字段。
- [ ] **阻塞在外部**：取消会让 Moonraker 的 `_save_path_info` 抛 `KeyError`（不在它的
      `except ServerError` 里），连接任务出错、它反复重连。上游把下标取值改成
      `.get()`（或用 `client_info` 判类型）之后才能自由。所以这条要等与 Moonraker 的
      沟通/上游改动，排在很后面。
- [ ] **过渡期的可选做法：按请求认出 Moonraker，只对它发这个字段**（其余客户端不发）。
      可行手段：① `client_info.program == "Moonraker"`
      （`moonraker/components/klippy_apis.py`：首次 info 带 `{'client_info': {'program': "Moonraker", 'version': …}}`）
      —— 自报、可缺、且**只有识别那次请求带**，因此要按连接记住；
      ② `SO_PEERCRED` 取对端 pid，再看 `/proc/<pid>/comm`（Moonraker 现在是 Python 进程，
      comm 未必是 moonraker）—— 更硬但要先把对端身份从 server 层传到
      `EndpointContext`（现在没有）。上游只把 `client_info` 当日志用，故这属于本项目的
      自定义兼容层，要有到期日。

## 未决问题

（编号沿用上一版，从 Q2 起。）

- [ ] **Q2 事件系统的形状**：封闭枚举（现状 `PrinterEvent`）还是开放总线（上游 30+ 个
      自定义事件名 + 各事件自定参数，`klippy/klippy.py:224-227`）。枚举表达不了
      `idle_timeout:ready` 这类名字，也表达不了 B2 的带载荷事件。
- [ ] **Q3 `PrinterEvent` 是否恢复 `McuIdentify` / `AnalyzeShutdown` / `NotifyMcuError`**
      （后两个的 handler 需要 msg + details）。这其实是 Q2 的一个子问题：无参 handler
      装不下它们。
- [ ] **Q4 `get_status` 的返回形状**：`serde_json::Value`（贴上游、客户端零适配）还是
      typed + serde。
- [ ] **Q5 要不要反射式能力**：`lookup_objects(module)` 前缀遍历、`gcode_macro` 的
      `printer.objects`（`klippy/extras/gcode_macro.py:41`）。**部分已做**：F2 为了
      `pins` 加了 `Printer::lookup_object_as::<T>(name)`（单名取回具体类型）；前缀遍历仍未定。
- [ ] **Q6 退出结果的语义**：`"exit" / "error_exit" / "firmware_restart"` 由谁解释、
      `run()` 的返回值怎么变成进程退出码（`klippy/klippy.py:355-370`，`error_exit` 退 -1）。
- [ ] **Q7 重启时 API 与打印机的关系**：`firmware_restart` 要重建机器，但端点与
      `webhooks` 把 `Arc<Printer>` 烤在了自己身上。要么每次重启重建 Api + Server
      （上游每次重新 bind socket），要么一个 Api 配一个可换入的 printer 槽
      （`RwLock<Arc<Printer>>` / ArcSwap）。前者要动 `--tui` 的 in-process server，
      后者要改 `api::register` 与三个端点/对象的构造。定下来之前不做重启循环（D2）。

## 证据索引（上游，供回头分析时查）

| 主题 | 位置 |
|---|---|
| Printer 生命周期、状态、事件 | `klippy/klippy.py:25-236` |
| 事件总线（开放名字 + 参数） | `klippy/klippy.py:224-227` |
| notify / analyze shutdown 的载荷 | `klippy/klippy.py:144-151`、`:216-220` |
| 对象注册表（add / lookup / load） | `klippy/klippy.py:70-113` |
| 主循环与重启、退出码 | `klippy/klippy.py:355-370` |
| `objects/list`、`query`、`subscribe` | `klippy/webhooks.py:480-560` |
| `emergency_stop` / `register_remote_method` / mux | `klippy/webhooks.py:319-340` |
| `gcode/*` 端点 | `klippy/webhooks.py:438-452` |
| gcode 调度器（命令表 / `run_script` / 输出） | `klippy/gcode.py` |
| `output_pin`（`SET_PIN` / `GCodeRequestQueue`） | `klippy/extras/output_pin.py` |
| section 校验用注册表 | `klippy/configfile.py:425-445` |
| mcu 作为 printer object、它的 status | `klippy/mcu.py:1147-1170`、`:1235`、`:938-975` |
| stats 累计与 shutdown 处理 | `klippy/mcu.py:801-802`、`:883`、`:912`、`:974-975` |
| 工厂装载 `load_config` / `load_config_prefix` | `klippy/klippy.py:90-113` |
| reactor 定时器 / 回调 / 时钟 | `klippy/reactor.py:111` `:145` `:187` |
| kinematics 的装载与接缝 | `klippy/toolhead.py:235-252`、`:389` `:400` `:482` `:507` `:522` |
| 各 kinematics 的差异 | `klippy/kinematics/*.py`（`home` / `check_move` / `calc_position` / `get_status`） |
| 通用回零驱动 | `klippy/extras/homing.py:165-300` |
| IDEX / 双滑车 | `klippy/kinematics/idex_modes.py`、`klippy/kinematics/cartesian.py:19-30` |
| step 生成层的运动学 | `klippy/kinematics/kinematic_stepper.py`、`rail.setup_itersolve(...)` |
| 惰性装载 | `klippy/extras/adc_temperature.py:51` `load_object(config, 'query_adc')` |
| MCU 配置构建（oid / config 命令 / CRC / pin 解析） | `klippy/mcu.py:979-1143`、`klippy/pins.py:18-137` |
| 引脚别名 section（`alias_pin` / `reserve_pin` 的调用方） | `klippy/extras/board_pins.py` |
| 固件配置区（allocate_oids / get_config / finalize / config_reset） | `src/basecmd.c:235-380` |
| GPIO 输出 / PWM（软件） | `klippy/mcu.py:408-553`、`src/gpiocmds.c:127-215` |
| 硬件 PWM | `klippy/mcu.py:451-553`、`src/pwmcmds.c:78-130` |
| ADC 采样与周期查询 | `klippy/mcu.py:555-655`、`src/adccmds.c:75-115` |
| SPI / I2C 总线 | `klippy/extras/bus.py:9-336`、`src/spicmds.c`、`src/i2ccmds.c` |
| endstop / trsync 触发 | `klippy/mcu.py:155-407`、`src/endstop.c:72-120`、`src/trsync.c` |
