# 已完成条目归档（截至 2026-09-22）

本文件是 `TODO.md` 原有「已完成（留档）」一节的全文，为了把 TODO 收窄到**只剩未决项**而
单独归档。它不是规范，也不必随实现更新：细节在各模块自己的文档与
`docs/klippy/developer-manual/`，这里每条只留一行索引，最近完成的在前。

- **上游 `.test` 语料框架与 T1（首个用例转绿）**：`src/core/klippy/upstream.rs` 的 harness 与
      runner（按 `CONFIG` 拆运行、字典齐备才启用、`IGNORED` 留档必然失败的用例）；
      `interface/devices/simulator.rs` 的字典驱动应答机（identify / 配置握手 / 时钟 / ack）；
      `crates/test-support/build.rs` 按 `KLIPPERX_ARCHES` / `KLIPPERX_ALL_ARCHES` 编字典；
      并顺带对齐配置解析（多行值 / `=` / 节头与 `;` 注释）、实现 `deprecate`；
      `linuxtest.test` 端到端通过（`kinematics: none` + `heaters` + `temperature_sensor` +
      `ds18b20`）。后续推进见正文 **T**。
- **文档补齐（E1）**：`developer-manual` 补 `printer` 一节与分层表行；清掉 `printer.rs` 头注释里
      「还没有住户」的过期描述。

- **框架队列 FW1–FW9（全部完成）**：FW1 配置装载、FW2 对象模型、FW3 错误词汇、FW4 G-Code、
      FW5 运动（a–f）、FW6 资源与触发、FW7 MCU 与传输、FW8 主机层与重启、FW9 API；验收以
      「最小模块在 host 单测 + 假 MCU 上跑通」为准，真板项见 [`TESTING.md`](../../TESTING.md)。
      两个子项 `[~]` 暂缓：`GCodeIO`（不做 OctoPrint 串口仿真，见正文 G1b）、
      `MCU_bus_digital_out` 包装（能力已由 `DigitalOut::queue_digital_out` 提供，随 H8 显示接）。
      分阶段细节见各 FW 的工作记录（`docs/work-log/2026-09-21-fw*-notes.md` 等）；
      已完结的几篇随收口清理（见目录 README 说明）。
- **toolhead 与 kinematics（C1 / FW5a–f）**：Rust 分层重写（不引 FFI）——`Coord` 与
      `clocksync` 回归、`Move`/`LookAheadQueue`/`trapq`、`itersolve`+`kin_cartesian`、
      `MotionQueuing`/`ToolHead`/`McuStepper`、`Kinematics`+`cartesian`+`[stepper_*]`/`[printer]`+`G1`、
      `stepcompress` 完整压缩；`kinematics: none` 随 T1 补上。设计取舍与逐阶段验收见
      FW5 动工前调查（已完结清理）；回零协议的 `get_trigger_position` /
      `set_stepper_adjustment` 随 `endstop_phase` 后置（H9）。
- **框架队列收尾（FW1/C2、FW3/A2、FW4、FW7、FW8、FW9）**：配置 getter 补 `get_choice`/
      `get_float_bounded`/`get_int_bounded` 并改用上游文案（`config/wrapper.rs`）；`require_object`
      与 connect 失败统一 `set_error_state`（可 RESTART）；`GCodeDispatch::request_restart` +
      `RestartHooks` 让 `RESTART`/`FIRMWARE_RESTART` 先 dwell/wait 再退出；`error_mcu` 模块
      （`invoke_shutdown_with` 载荷 + `update_error_msg` + 停机/protocol/connect 文案）；
      `StartArgs` 补 `apiserver`/`start_reason`/`device` 等并接入 `M115`；
      `register_remote_method` 端点 + `webhooks` 对象转发 `call_remote_method`、mux 注册倒入
      `Api`。细节见框架收尾记录（已完结清理）。

- **G-Code 框架大部（FW4）**：参数访问器补齐（通用 `get` + `minval`/`maxval`/`above`/`below`、
      `get_int_bounded`、`get_float_bounded`）、`get_command_parameters` / `get_raw_command_parameters`
      （剥行号与校验和）、`create_gcode_command`、`run_script_from_command`；`gcode:command_error`
      在 `process_line` 接上（panic 不触发）；`ack` 改为一次性（`Cell`）并按 `need_ack` 决定错误
      是否中止脚本；`gcode:request_restart` 声明补 `print_time` 载荷，生成枚举去掉 `Eq`。
      `GCodeIO`（伪 tty / OctoPrint 串口仿真）**暂缓 `[~]`**，随 C1 的 `request_restart` 触发
      另行跟进；见 [FW4 笔记](2026-09-21-fw4-notes.md)。

- **FW7/FW8 收尾（B2/D2/D3）**：`last_stats`（`event/stats.rs` 换算 + `McuObject` 上报，真板已验）；
      固件 `reset` 优先于 `config_reset`（`mcu/config.rs`）；RTO 定时重传（`mcu/mod.rs` 的
      `Sender::retransmit`，25 ms → 5 s 退避，单测）；`rpi_usb` 连接期门控
      （`restart_before_bringup`，仅单测）；重启后 `gcode/subscribe_output` 订阅由端点
      `watch_restarts` 重挂（`api/endpoints/gcode.rs`）；字典装载前的固件输出不再报 `ERROR`。

- **主机层日志与退出码（FW8 大部）**：`--logfile` 把格式化日志同时写到 stdout 与文件
      （开不了就降级），`info.log_file` 报真实路径；主机层 rollover info（`set/clear/write_rollover`，
      启动与重启写 `versions` + 横幅）；`klippy::run` 返回退出码，`error_exit` 退 `-1`
      （`logging.rs`、`klippy.rs`、`main.rs`、`bin/klippy/main.rs`、`api/start_args.rs`）。

- **MCU 停机与 `emergency_stop`（FW7）**：新增 `emergency_stop` 端点（进 shutdown 并回 `{}`）；
      每个 `McuObject` 由工厂在 `klippy:shutdown` 注册处理器，向固件发 `emergency_stop`；
      新增 `is_shutdown`（`Arc<AtomicBool>`）与 `force_local_shutdown`，固件自报停机时不回发
      （`api/endpoints/emergency_stop.rs`、`mcu/object.rs`）。
- **对象模型反射（FW2）**：Q4 定案 `get_status -> serde_json::Value`；新增
      `Printer::lookup_objects(module)`（前缀遍历）与 `Printer::statuses(eventtime)`（一次取
      全部可查对象状态），供 `gcode_macro` 的 `printer.objects` 等消费者；Q5 定为只读反射
      （`printer.rs`）。

- **错误词汇框架（FW3）**：`ConfigError` + `KlippyError::Config`；`Printer::add_object` 重复名、
      `load.rs` 工厂拒绝/未认领 section、`McuConfig::new` 都改报 config error；`gcode.rs`
      `invoke_handler` 与 `Api::dispatch` 用 `catch_unwind` 兜底（panic → `invoke_shutdown`）；
      API 侧新增 `Api::set_internal_error_hook`；新增 `Printer::set_error_state`，坏配置/connect 期
      config 错报 `error`（可 RESTART）而不是 shutdown（`error.rs`、`printer.rs`、`gcode.rs`、
      `crates/klippy-api/src/registry.rs`、`klippy.rs`、`config/mcu.rs`、`mcu/object.rs`）。
- **配置装载框架（FW1）**：`ConfigWrapper`（类型化 getter 同时记账）+ `AccessTracking`（键小写化、
      值为解析后的 JSON）+ `check_unused`（section/option 未认领报错）；`section!` 增 `phase`/`object`，
      装载器按 early/generic/late 分阶段并支持对象名≠节名；新增 `configfile` 对象（`settings`/`config`）；
      `McuObject` 工厂预解析 `[mcu]` 以便在装载期记账，`connect` 仍只开设备
      （`config/{access,wrapper,validate,object}.rs`、`load.rs`、`build.rs`、`printer.rs`）。
      真板（STM32F103）核对：正常配置 10/10 `ready`（与基线交替），`objects/list` 含 `configfile` 且
      `settings` 形状对，拼错选项报 `error`（可 `RESTART`）、`SET_PIN` 仍可用；
      连续复位抖动的两个新发现记入 D2。

- **I2C 总线错误停机（F7）**：`McuI2c::transfer`/`write` 非 SUCCESS 时按上游
      `invoke_shutdown`；探测用的 `transfer_without_shutdown`/`write_without_shutdown`
      只报错。
- **G-Code 默认处理器与参数解析（G1b）**：`default_handler` 补 `M105`/`M21`/`M140`/`M104`/
      `M107`/`M106` 的“安静忽略”与 `M117`/`M118`/`M23` 的按首 token 路由；`GcodeCommand::ack`
      + `need_ack`（`M115`/`M105` 已用）；`parse_extended` 补反斜杠转义与引号拼接；
      `unregister_command` 注销（对齐上游 `register_command(cmd, None)`）。
- **G-Code 调度器小行为对齐（G1b）**：`ECHO` 改用 `// ` 前缀且不记日志、`HELP`
      未就绪提示与按 active 表遍历、`get_status` 按 active 表构建、未就绪时停机不打印、
      mux 默认项（`value=None`）不再不可达、行号命令行剥尾部 `*<digits>` 校验和；另删掉
      `src/core/parser.rs` 的未引用存根，developer-manual 补 `printer` 一节与分层表行。
- **软件总线命令新旧兼容（F6/F7）**：`cmd::spi::add_software_bus` /
      `cmd::i2c::add_software_bus` 按 `try_lookup_command` 在 `*_set_sw_bus`（host 算好
      `pulse_ticks`）与 `*_set_software_bus`（固件收 `rate` 自行量化）之间二选一，回退时打
      deprecation 警告；resource 只给 pins + speed，不感知新旧。
- **SPI 总线（F6）**：`cmd/spi.rs`、`mcu/resource/spi.rs` 的 `McuSpi`（硬件/软件两条
      路，固件驱动 CS）、`extras/spi_device.rs`（`[spi_device]` 构造器 + `SPI_TRANSFER` /
      `SPI_SEND` 调试命令）；与 `[i2c_device]` 共用 `extras/bus_debug.rs`。
- **I2C 总线（F7）**：`cmd/i2c.rs`、`mcu/resource/i2c.rs` 的 `McuI2c`（硬件/软件两条
      路，旧式 `i2c_transfer` / 新式 `i2c_read`）、`Mcu::try_lookup_command` 与
      `PrinterPins::resolve_bus_value`；消费者 `extras/i2c_device.rs` 与 `IIC_WRITE` /
      `IIC_READ` 调试命令（真机自测用，见 F7）。
- **真板端到端验证（G2b）**：`config.cfg` 中 `[output_pin]` + `SET_PIN` 点亮/熄灭 + PWM
      占空比调整，确认 `config_digital_out` / `config_pwm_out` oid 与初始值、
      `update_digital_out` / `update_pwm` 上线（替代假 chip / 假设备测试）。
- **ADC（F5）**：`cmd/adc.rs` 的 `config_analog_in` / `query_analog_in`（新旧两版）与
      `analog_in_state`，`mcu/resource/adc.rs` 的 `McuAdc` / `AdcRegistry`，以及
      `ConfigBuilder::get_query_slot`（`Mcu::estimated_clock`）；消费者（thermistor 等）未接。
- **PWM（F4）**：`cmd/pwm.rs` 与 `mcu/resource/pwm.rs` 的硬件/软件两条路
      （`set_pwm` / `update_pwm` / `next_aligned_clock`），`pins.rs` 的 `PwmOut` /
      `setup_pwm`；`[output_pin]` 的 `pwm` / `cycle_time` / `hardware_pwm` 已接。
- **`[board_pins]` 与 `BUS_PINS_<bus>`（F2 剩余）**：`extras/board_pins.rs` 与
      `ConfigSection` 的 `get_list` / `get_list_of_lists`；`McuChip::resolve_bus_name`
      按 `BUS_PINS_<bus>` 预留 SPI/I2C 引脚（F6/F7 会用）。
- **重启循环与 `restart_method` 分派（D2 大部）**：`klippy_process` 就地重建
      （`Printer::reset_for_restart` + `load_config` + `bring_up`，同一个 `Arc<Printer>`），
      以及 `command` / `arduino` / `cheetah` / `rpi_usb` 四种物理复位与连接期门控
      （`src/klippy.rs`、`mcu/object.rs`、`mcu/restart.rs`、`interface/usb.rs`）。
- **identify 后的 DEBUG 摘要**：`describe_dictionary` 在 `Mcu::identify` 里打版本对、
      消息条数与常量（`identify.rs`）。
- **客户端的 `firmware_restart`**：`Session::firmware_restart` 与本地命令
      `.firmware_restart`（行模式与 g-code 模式都认），`usage()` 同步更新
      （`crates/klippy-client/src/session.rs`）。
- **`Mcu::flush`**：发送队列的 item 分 `SendItem::Payload | SendItem::Flush(oneshot)`，
      发送任务遇 barrier 立即发走并回报；`FrameMock::recorder()` 让 block 边界可断言
      （`mcu/mod.rs`、`interface/test.rs`）。
- **reset 路径的 P0/P3**：`emergency_stop` 与 `config_reset` 分两个 block，中间用固件的
      `shutdown` 报告作屏障，无 `shutdown` 时 15 ms 兜底并告警（`mcu/config.rs`、
      `mcu/object.rs`）。
- **`gcode/subscribe_output` 与 TUI g-code 模式**：连接包成带 `is_closed` 的
      `OutputHandler` 推 `{response: line}`；`^G` / `.gcode` 整行走 `gcode/script` 并自动
      订阅（`api/endpoints/gcode.rs`、`gcode.rs`、`klippy-client`）。
- **`gcode/*` 端点（G3）**：`gcode/help` / `script` / `restart` / `firmware_restart`；
      命令级错误用 `ApiError::CommandError`，不关停 klippy（`api/endpoints/gcode.rs`）。
- **`output_pin` 与 `SET_PIN`（G2）**：`setup_digital_out` / `setup_pwm` 建资源并注册
      `SET_PIN PIN=… VALUE=…`，无条件 `setup_max_duration(0)`（`extras/output_pin.rs`、
      `load.rs`）；剩余差异见 **G2b**。
- **GCODE 调度器（G1）**：命令表 / `register_mux_command` / `run_script` / 输出处理器 /
      内置命令，`load_config` 里最先注册（`gcode.rs`）；剩余行为差异见 **G1b**。
- **GPIO 数字输出（F3 的 MCU 部分）**：`PinChip` / `DigitalOut`、`config_digital_out` +
      restart 的 `update_digital_out` + 运行期 `queue_digital_out`
      （`cmd/gpio.rs`、`mcu/resource/pin.rs`）。
- **pin 解析与 `pins`（F2）**：`PrinterPins` / `PinResolver` 的别名与保留，
      `RESERVE_PINS_*` 在 connect 预留；`pins` 注册但不可查询（`pins.rs`、`printer.rs`、
      `mcu/object.rs`）。
- **MCU 配置构建层（F1）**：oid 发号、`config` / `restart` / `init` 三张命令表、config 回调、
      CRC + `finalize_config`，`configure()` 的 `get_config` 两段式下发（`mcu/config.rs`）。
- **MCU 停机上报与复位（B2 大部）**：`shutdown` / `is_shutdown` / `starting` 经
      `static_string_id` 解成原因，配置握手**之后**绑成打印机停机；`configure` / `handshake`
      先复位再配置——有 `config_reset` 就地清，只有 `reset` 的固件发 `reset` + 重连 + 重试
      握手（`event/shutdown.rs`、`mcu/config.rs`、`mcu/object.rs`）。
- **`objects/subscribe`（B1）**：请求立即回全量快照，随后每 0.25 s
      （`SUBSCRIPTION_REFRESH_TIME`）推变化字段；连接关闭即退订（`api/endpoints/objects_subscribe.rs`、
      `objects_query.rs`）。
- **reactor 抽象与定时器（A1）**：`Reactor` trait（`monotonic` / `register_timer` /
      `unregister_timer` / `call_later`）与 `TokioReactor` / `ManualReactor`；机器持
      `Arc<dyn Reactor>`，不拥有 runtime（`reactor.rs`）。
- **reactor 串行调度器（A1b 之一）**：`TokioReactor` 改为**一个 dispatcher 任务 + 最小堆**
      （`reactor.rs` 的 `run_dispatcher` / `Dispatcher` / `TimerEntry`）：睡到最早唤醒时间、
      按唤醒时间一次跑一个回调，同时到期按注册顺序（`seq`）；注册 / 取消都 `Notify` 唤醒
      dispatcher，取消的条目在到期时跳过；reactor 析构时置 `closed` 让 dispatcher 退出。
      有两处回归测试（同一时刻按注册顺序、回调里再注册不死锁）。
      （`docs/klippy/developer-manual/reactor.md`）。
- **定时回调不许等待 / 做重活（A1b 之一）**：写进 `reactor.rs` 的模块文档与 `register_timer`
      契约——回调跑在 dispatcher 上，没有地方 `await`，不许阻塞或做重活；`reactor.md` 单列
      一节。对应上游 `assert_no_pause`（`klippy/reactor.py:265`），当前只是约定、无机制。
- **reactor 延迟度量（A1b 之一）**：`Reactor::set_latency_notifier`（对应上游
      `reactor.py:316`）——一轮分发从最早唤醒时间算起忙过阈值，就把该轮回调的
      `LatencyReport`（`busy` + 每个回调的 `name` / `duration` / `lateness`）交回。名字在注册时
      给出（`register_timer_named`；`register_timer` 为无名版），因为 Rust 闭包没有名字可反射。
      trait 默认空实现，`TokioReactor` 实现（`ManualReactor` 不给抖动）；主机在 `src/klippy.rs`
      挂 50 ms 阈值的日志回调（上游 `garbage_collection` 用同一阈值）。测试在真时间下验证慢回调
      被报出、快回调不报、被慢回调挡住的后继定时器 `lateness` 超阈（`reactor.rs`；
      `docs/klippy/developer-manual/reactor.md`）。
- **机器侧 spawn 显式化（A3 前置）**：`Interface` 改为「`handle` 字段 + 私有 `Transport` 枚举」，
      设备 I/O 走 `off_runtime`（用存的 handle）；`Mcu` 从 `interface.handle()` 取 handle 存字段，
      收发任务用它 spawn；`restart.rs` 的 `spawn_blocking` 改成显式 `&Handle` 参数。机器侧不再有
      裸 `tokio::spawn` / `spawn_blocking`，唯一 ambient 捕获点是 `Interface::with_transport`
      （设备打开**之后**，所以打不开的传输不需要 runtime）。行为不变。
      （`interface/mod.rs`、`mcu/mod.rs`、`mcu/restart.rs`、`mcu/object.rs`、`config/mcu.rs`；
      `docs/klippy/developer-manual/runtime.md`）
- **机器与 API 分 runtime（A3）**：机器跑在专用多线程 runtime（`worker_threads(2)`，worker 名
      `klippy-mcu`）上，由一条专用 OS 线程（`klippy-machine`）驱动 `klippy_process`；API 保留
      进程本来的多线程 runtime（`klippy-api`）跑 accept / 每连接 / attachment / `ctrl_c`。reactor
      显式建在机器 handle 上；`load_config` 跑在 `machine_handle.enter()` 下，所以
      `Interface::with_transport` 捕获到的是机器 handle。跨 runtime 只靠 `Arc<Printer>` 与
      `request_exit` 的 `Condvar`。停机顺序：`request_exit` → `run()` 返回 → `teardown`（机器
      runtime 尚在）→ 机器线程 drop runtime → join → abort 监听与 server（`src/klippy.rs`；
      `docs/klippy/developer-manual/runtime.md`）。
- **主机层串起来**：`klippy_process`（建机器 → `api::register` → bind → `load_config`，
      失败即 `invoke_shutdown` → `bring_up` → `run`）、`info` 端点、`StartArgs`
      （`src/klippy.rs`、`api/endpoints/info.rs`、`api/start_args.rs`）。
- **`run()` 的形态**：`bring_up()` async、`run()` 同步只等退出、机器只用
      `std::future::Future`，不起 tokio（`printer.rs`）。
- **配置驱动装载**：静态工厂表、两段式构造顺序、section 级合法性校验（`load.rs`）。
- **`[mcu]` 住户**：`McuObject`，section 到 connect 才解析、开设备、跑 identify，
      `get_status` 报 identify 快照（`mcu/object.rs`）。
- **对象表与只读端点**：`add_object` / `objects` / `lookup_object` / `status_of`、
      `objects/list`、`objects/query`，以及服务器侧的 `webhooks` 对象
      （`printer.rs`、`api/endpoints/objects_{list,query}.rs`、`api/webhooks.rs`）。
