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

## 待办

编号保留旧文件的 T/Q 以便对照，新增项给新号。依赖列的是**工具性前置**，不是自然顺序。

| # | 事项 | 依赖 |
|---|---|---|
| A1b | reactor 串行调度器与延迟度量 | A1 ✓ |
| A2 | 错误词汇（`CommandError` / `ConfigError`） | — |
| B1 | `objects/subscribe` | A1 ✓ |
| B2 | MCU 关闭与错误上报（含 `last_stats`） | A2 |
| B3 | `gcode` 层与 `gcode/*` 端点 | C1 |
| B4 | 其余端点（estop / remote method / pause_resume / …） | B3 等 |
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

### B1（旧 T1 剩余）objects/subscribe

- [ ] 0.25s 轮询 + `response_template`（`klippy/webhooks.py:482` 注册、`:561` 实现），
      推送走已有的 `PushTarget`（`crates/klippy-api/src/protocol.rs:507`，
      `ResponseTemplate` 已就位）。
- [ ] 需要 A1 的定时器与「只推变化」的比对：上游用全局 `last_query` 与每个订阅自己的
      字段表，两次查询里同一个值变了才推。

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

### B3（新）gcode 层与 gcode/* 端点

- [ ] `gcode` 层本身不存在（`src/core/klippy/` 下没有 `gcode`）：没有 g-code 解释器、
      命令注册表、`run_script`。它是 toolhead 的第一个使用者，也是这一族端点的全部内容：
      `gcode/help`、`gcode/script`、`gcode/restart`、`gcode/firmware_restart`、
      `gcode/subscribe_output`（`klippy/webhooks.py:438-444`）。
- [ ] `gcode/restart` / `gcode/firmware_restart` 只是 `run_script('restart' /
      'firmware_restart')`（`klippy/webhooks.py:449-452`），主机侧语义接 D2。

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
      `printer.objects`（`klippy/extras/gcode_macro.py:41`）。
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
