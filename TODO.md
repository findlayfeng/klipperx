# TODO — 主机（klippy）侧的待办与未决问题

本文件只记**打算做什么**和**还没定的事**，不重复已经定下的设计（那些写在各模块文档和
`docs/klippy/developer-manual/` 里）。括号里的 `klippy/xxx.py:NN` 指上游参考实现
（`third_party/klipper/`），用来在动手前核对行为。已完成的条目见文末「已完成（留档）」，
**每条只留一行索引**，细节在各模块自己的文档里；做完一件事就把它从正文挪进那张索引。
上游**全部功能点**的逐项对照（含判为「不适用」的 Python 专属项）见
[上游功能覆盖审计](docs/work-log/2026-09-21-upstream-coverage-audit.md)；本文件只放
**要动手的事**与**还没定的事**。

## 框架优先（先把框架做完，再铺模块）

本项目的固定做法：**先把框架立住 → 用一个最小模块把它跑通 → 再横向铺模块**。
下面把 TODO 里属于**框架级**的事单独排队；H1–H12、G2b/G4、S1 等具体模块都排在
这些之后。判定标准：一项负责**定接口/定生命周期/定数据形状**、被多个 extras 共用，
就是框架；只消费已有接口、自己就是一个 `[section]` 的，是模块。

| # | 框架 | 对应 TODO | 框架边界（定什么） | 首个模块（验收） | 之后铺开 | 依赖 |
|---|---|---|---|---|---|---|
| **FW1** | 配置装载框架收尾 | C2 | option 访问追踪当 schema（`ConfigWrapper` + `AccessTracking`）、住户与阶段（`phase`/`object`，对象名可≠节名）、未认领 section/option 报 `ConfigError`、`configfile` 对象 | `[output_pin fan]` 多一个选项报选项错、`objects/list` 含 `configfile` 且 `settings`/`config` 形状对；`[printer]`→`toolhead` 的晚阶段住户有装载器单测 | 各 extras 的 option schema（H1–H12）；`[printer]`/toolhead 本体随 C1 接入 | FW3（共用 `ConfigError`） |
| **FW2** | 对象模型收尾 | Q4、Q5 | Q4 定案 `Value`；Q5 加 `lookup_objects(module)` 前缀遍历与 `statuses()` 快照 | `objects/query` 形状不变；`lookup_objects("mcu")` 前缀遍历与反射读状态有单测 | `gcode_macro` 的 `printer.objects` 模板视图、display 菜单、宏变量 | — |
| **FW3** | 错误词汇框架 | A2 | `CommandError`/`ConfigError` 分层、`KlippyError::Config`、`Internal` 收敛、handler/endpoint 异常 `catch_unwind` → `invoke_shutdown`、config 错走 `set_error_state`（`PrinterState::Error`） | 参数错报 `CommandError` 且不停机；坏配置 / connect 期 config 错报 `error`（可 RESTART）；panic 的 handler / endpoint 触发 `invoke_shutdown` | 全树的错误分支 | — |
| **FW4** | G-Code 框架收尾（**完成**；`GCodeIO` 暂缓 `[~]`） | G1b（框架部分） | ~~参数访问器、`create_gcode_command`、`run_script_from_command`、`gcode:command_error` 触发~~ ✅；`GCodeIO` 输入抽象（伪 tty / 文件 / `stats gcodein` / `debuginput_exit`）**暂缓 `[~]`**（不做 OctoPrint 串口仿真）；`gcode:request_restart` 触发随 C1 | ~~`create_gcode_command` / 参数访问器~~ ✅；`GCodeIO` 暂缓（见 [FW4 笔记](docs/work-log/2026-09-21-fw4-notes.md)） | 全部 gcode extras（H3、H8…） | FW1、FW3（已满足） |
| **FW5** | 运动框架（最重，**拆 FW5a–FW5f**） | C1（框架部分）、H12 | **FW5a** `Coord` + `clocksync` 回归；**FW5b** `Move`/`LookAheadQueue`/`trapq`；**FW5c** `itersolve` + `kin_cartesian`；**FW5d** `MotionQueuing`/`ToolHead`/`McuStepper`；**FW5e** `Kinematics` + `cartesian` + `[stepper_*]`/`[printer]` + `G1`；**FW5f** `stepcompress` 完整压缩 | host 单测 → 假 MCU → **真板 `G1`（单轴 → 三轴 + `[extruder]`）→ `G28`（与 FW6/F8 联合）** | `kinematics/*` 其余、H9、H10、input shaper | FW1（已满足） |
| **FW6** | 资源与触发框架（**拆 FW6a–FW6f**） | F3、F8 | **FW6a** 命令层（`cmd/endstop.rs`/`cmd/trsync.rs`/`stepper_stop_on_trigger`）+ `MCU_endstop` + 单 MCU `TriggerDispatch`/`MCU_trsync`；**FW6b** `Rail`/`endstop_pin` + `query_endstops` 对象 + `query_endstops/status` 端点 + `M119`；**FW6c** `stepcompress` history/`find_past_position` + stepper 回零句柄；**FW6d** `HomingState` + `ToolHead::drip_move` + `G28`；**FW6e**（后置）多 MCU trsync；**FW6f**（小）`MCU_bus_digital_out` | FW6a/b：一个 endstop + `query_endstops/status`（假 MCU）；FW6d：真板单轴 `G28`（与 FW5 联合） | homing/probe、运动同步 `SET_PIN` | FW5 |
| **FW7** | MCU 与传输框架收尾 | B2、D3 | `emergency_stop` 对象（`klippy:shutdown` → 固件 `emergency_stop`）、本地 shutdown 标志、`emergency_stop` 端点、带载荷错误上报；RTO 定时重传与固件 `reset` 优先未做 | `emergency_stop` 端点使打印机进 shutdown；主机停机向固件发 `emergency_stop`，固件自报停机不回发 | TMC/传感器等资源 | — |
| **FW8** | 主机层与重启框架 | D1、D2、Q6 | `--logfile`/rollover/Q6、`rpi_usb` 门控、CRC 物理复位、重启后订阅均已完成（代码）；剩 `StartArgs` 其余字段与 `rpi_usb` 真机验证 | `--logfile` 落盘、`error_exit` 非零、重启后订阅不断；真板 `last_stats` 已验 | 日志、Moonraker 兼容 | — |
| **FW9** | API 框架收尾 | B4（框架部分） | `register_remote_method` 与推送、mux 端点注册机制、`emergency_stop` 端点 | `register_remote_method` + 推送 | `pause_resume/*`、`*/dump_*` 等消费者 | FW7 |

> **怎么验收**：每个框架都以「最小模块在真机/测试设备上跑通」为准，不以“代码写完”为准。
> 例如 FW5 的验收是 `G1` 真的动了步进（`G28` 与 FW6/F8 联合验收），而不是 `Kinematics`
> trait 编译通过。

> **建议顺序**：**FW1/FW3**（配置与错误，最底层）✅ → **FW2/FW7/FW8** ✅ 大部
> （对象模型 / MCU / 主机层）→ **FW4**（G-Code，依赖 FW1+FW3）✅（`GCodeIO` 暂缓 `[~]`）→ **FW5**（运动，最重，
> 依赖 FW1）→ **FW6** → **FW9**。FW5 与 FW4 都依赖 FW1；FW6 等 FW5 的运动层，但 F8
> （endstop/trsync）的接口与 FW5e 联合定，`G28` 的验收跨两者。
> FW2/FW7/FW8 的剩余点见 FW8（`rpi_usb`/CRC/输出订阅）、B2（`last_stats`/RTO/固件 `reset`）。
> D2 的真板启动抖动已归档为**非阻塞观察项**（板/USB 链路层，复现不了），不再单独排期。

> **未决问题里属于框架决策的**：**Q4**（status 形状，FW2）、**Q5**（反射，FW2）、
> **Q6**（退出语义，FW8）；其余 Q 已解决或属模块。**Q8**（`GCodeIO`）已定为暂缓 `[~]`。

> **不列入框架、可以直接随模块做的**：G2b（`SET_PIN` 时序）、G4（运动命令本体）、
> F6 剩余（`spi_transfer_with_preface`）、F9、H1–H11、S1、E1、E2。

## 已定

- **一台机器，单实现**：`Printer` 是一个结构体（无 trait、无工厂），一个进程只跑一个；机器
  拥有它的组成部分与生命周期。进程级的东西（命令行、日志、API server、runtime、重启循环）
  是它上面的一层。
- **`Kinematics` 不是 Printer 的分类依据**：trait 与 `kinematics/` 已删，实现 toolhead 时再
  加回、由 toolhead 持有（`klippy/toolhead.py:242` 是唯一装载点）。
- **分层**：`msg → mcu → cmd`，`event` / `identify` 平级
  （`docs/klippy/developer-manual/architecture.md`）。
- **bring-up 是机器的，executor 是调用方的**：`PrinterObject::connect()` 返回 boxed
  `std::future::Future`，`bring_up()` async、`run()` 同步只等退出；机器不依赖任何 runtime。
- **配置装载是唯一入口，顺序是契约**：`load.rs` 的静态工厂表是唯一的 section → object 入口，
  主 section 先于前缀 section；两段式构造（工厂只登记，`connect()` 才解析/开设备）；
  `webhooks` 由 `api::register` 先装，所以 `objects/list` 从一而终以它开头
  （`klippy/klippy.py:36-40` `:90-113`）。
- **客户端 API 的线形状**以 `docs/klippy/third-party-dev/api-reference.md` 为准。
- **重启是就地重建**（Q7 的答案）：`restart` / `firmware_restart` 在同一个 `Arc<Printer>` 上
  `reset_for_restart()` → 重载配置 → 再 `bring_up`，端点与 `--tui` 的 in-process server 全程
  有效；不换 printer、不重建 Api/Server。

## 待办

依赖列的是**工具性前置**，不是自然顺序。下表是索引，逐条细节在后面的小节里；
框架级的工作已抽到文首「框架优先」，本表不再区分级别。
H1–H12 是[上游功能覆盖审计](docs/work-log/2026-09-21-upstream-coverage-audit.md)
里**未实现**的 extras 消费者，按域归并。

**核心与架构**

| # | 事项 | 依赖 |
|---|---|---|
| C1 | toolhead 与 kinematics | — |
| C2 | 配置装载剩余（`[printer]`/toolhead 住户、autosave/`SAVE_CONFIG`/`deprecate`、wrapper 的 choice/range 文案） | C1 |
| A2 | 错误词汇剩余（`lookup_object` config error、`Internal` 剩余收敛点） | — |
| B2 | MCU 剩余：`emergency_stop` 对象/端点、`last_stats`、错误载荷、本地 shutdown 标志、`command` 的固件 `reset` | — |
| D1 | 主机层 start args / rollover / `--logfile` | — |
| D2 | 重启循环剩余（`rpi_usb` 连接期门控、CRC 不一致的处理、重启后的输出订阅） | — |
| D3 | `command` 接管运行中的板子：RTO 定时重传 | B2 |

**MCU 资源与总线**

| # | 事项 | 依赖 |
|---|---|---|
| F3 | `MCU_bus_digital_out`（命令队列/运动同步输出） | C1 |
| F6 | SPI 总线剩余：`spi_transfer_with_preface` / `setup_shutdown_msg` | F1、F2 |
| F8 | endstop / trsync | F1、F2、C1 |
| F9 | 固件资源剩余：buttons / pulse_counter / trigger_analog / initial_pins / sdcard / sensor_bulk / lcd / neopixel / thermocouple / tmcuart 等 | F1–F7 |

**G-Code 与端点**

| # | 事项 | 依赖 |
|---|---|---|
| G1b | gcode 调度器与上游的行为差异（参数访问器、事件触发…；`GCodeIO` 暂缓 `[~]`） | C1 |
| G2b | 用 GCODE 控制 GPIO：`SET_PIN` 时序（数字与 PWM 均已可驱动） | C1 |
| G4 | 运动命令（G0/G1/G28…） | G1、C1 |
| B4 | 其余端点（estop / remote method / pause_resume / `*/dump_*` / …） | G3、H4、H9 |

**上游 extras 消费者**（详见「上游 extras 覆盖盘点」）

| # | 事项 | 依赖 |
|---|---|---|
| H1 | 加热与温度（heaters / heater_bed / heater_generic / pid_calibrate / verify_heater / temperature_*） | F4、F5、C1 |
| H2 | 风扇与通用输出（fan / fan_generic / heater_fan / controller_fan / pwm_tool / static_* / multi_pin / servo / led / neopixel / dotstar / 电位器与 LED 驱动） | F3、F4、F6、F7 |
| H3 | G-Code 宏与脚本（gcode_macro / save_variables / delayed_gcode / respond） | G1b、Q5 |
| H4 | 打印流程与 SD 卡（virtual_sdcard / print_stats / display_status / pause_resume / exclude_object / sdcard_loop / firmware_retraction） | F9、C1 |
| H5 | TMC 步进驱动（tmc / tmc_uart / tmc2130…tmc5160） | F6、F7、F9、C1 |
| H6 | 传感器与块状数据（bulk_sensor / 加速度计 / angle / ldc1612 / hx71x / ads* / load_cell / input_shaper / resonance） | F5、F6、F7、F9、C1 |
| H7 | 输入与外设（buttons / gcode_button / pulse_counter / trigger_analog / 断料与线宽传感器 / GPIO 扩展 / DAC） | F3、F5、F9 |
| H8 | LCD 显示与菜单（display/*） | F9、G1b |
| H9 | 探测 / 调平 / 校准（probe / bltouch / bed_mesh / z_tilt / quad_gantry_level / bed_screws / …） | C1、F8 |
| H10 | 运动相关 extras（gcode_move / gcode_arcs / force_move / manual_stepper / stepper_enable / idle_timeout / motion_report / …） | C1 |
| H11 | 主机运行时与调试（statistics / error_mcu / canbus_ids / canbus_stats） | — |
| H12 | 核心工具补齐（mathutil / util 反射 / clocksync / pins 消费侧） | C1 |

**工具与文档**

| # | 事项 | 依赖 |
|---|---|---|
| S1 | 压力测试工具（`klipperx stress`）剩余：stepper 资源、别名解析、端到端测试 | C1 |
| E1 | 文档 | — |
| E2 | `python_path` 的取消 | 外部项目 |

> 判为**不适用**、不进待办的上游模块：`garbage_collection.py`（Python GC 调优）、
> `aio_executor.py`（Python 线程池）、`parsedump.py`（离线开发工具）、
> `debugcmds.c`（固件调试口）。理由见审计文档第 18 节。

### G2b 用 GCODE 控制 GPIO（现状与剩余）

数字与 PWM 两条链都已打通（见文末 F3 / F4 索引）：`[output_pin <name>]`
（`extras/output_pin.rs`）读 `pin` / `value` / `shutdown_value`（PWM 另加 `pwm` /
`cycle_time` / `hardware_pwm`），经 `PrinterPins::setup_digital_out` / `setup_pwm`
（`pins.rs`）建出 `McuDigitalOut` / `McuPwm`，并注册 mux 命令
`SET_PIN PIN=<name> VALUE=<0..1>`；运行时走立即路径（`update_digital_out` / `update_pwm`，
软件 PWM 对齐到周期边界）。剩下的是：

- [x] **真板端到端验证**：`config.cfg` 加一段 `[output_pin <name>]` + `pin: <PAx>`，用
      `SET_PIN PIN=<name> VALUE=1` 点亮、`VALUE=0` 熄灭，并对 `pwm: true` 的脚改占空比，
      确认 `config_digital_out` / `config_pwm_out` 的 oid 与初始值、`update_digital_out` /
      `update_pwm` 都真的上了线（现有测试都在假 chip / 假设备上）。
- [ ] **与运动 / 打印时间同步的 `SET_PIN`**（上游 `GCodeRequestQueue`，
      `klippy/extras/output_pin.py:13-85` `:249-269`）：上游把请求排进 toolhead 的
      lookahead、在 print time 生效，并对移动中的 pin 变化与 MCU 最小调度间隔做对齐；
      我们没有 toolhead / print time，只能立即改值（`output_pin.rs` 头注释）。随 **C1**；
      对一个独立 GPIO 不紧急，但打印中改 pin 不会与 move 同步。
- [ ] **`output_pin` 的其余上游选项**：`scale`（PWM 用，`output_pin.py:207-214`）、
      `TEMPLATE` + `template_evaluator`（display 模板，`output_pin.py:88-170`）——与开关
      GPIO 本身无关，按需再补。

### S1 压力测试工具（`klipperx stress`）剩余

**是什么**：`klipperx stress <config> [mcu]` 给一块 MCU 逐步加大负载到它出错。两个任务：
`--task step`（默认）压步进生成（`queue_step`，到固件 `shutdown`：`Stepper too far in past` /
`Timer too close` / `Move queue overflow`），`--task comm` 压主机↔MCU 链路（`get_clock` 往返速率）。
设计与失败形态见 `docs/klippy/developer-manual/stress.md`。

**已做**：连接 / identify、从 `[stepper_*]` 借 step/dir 引脚并解析（`PA0` / `mcu:PA0` / 尾随
`!`）、用 `ConfigBuilder` 配置一个压力 stepper（**无 `config_reset` 的固件走 `reset` + 重连 +
重试握手**）、按 `--rate-step` 几何 ramp、检测 `is_shutdown` 并报出原因（绑 `shutdown` /
`is_shutdown` 的 `static_string_id`）；`--task comm` 按墙钟配速发 `get_clock`、用 `clock` 回调计数、
以积压/达不到目标速率判定；`cmd/stepper.rs` 补了 typed 命令（`config_stepper` / `queue_step` /
`reset_step_clock` / `set_next_step_dir` / `stepper_get_position`）。

**真板实测**：STM32F103（72 MHz）上 `--task step --rate-step 1.1` 得到 339 623 步/秒存活、
375 000 步/秒 shutdown（`Stepper too far in past`）；`--task comm` 稳定扛住约 3.5k 往返/秒，
4441 req/s 时响应积压被判定为链路顶不住。

- [ ] **stepper 资源（C1）**：工具的 `invert_step` / `step_pulse_ticks` 硬编码为 0，也没读
      `[stepper_*]` 的 `microsteps` / `enable_pin`；真正的 stepper 资源随 C1 做，之后压力工具
      改成复用它。
- [ ] **`[board_pins]` 别名**：现在只解析引脚名本身，别名未展开（`pins.rs` 已有解析器）。
- [ ] **端到端测试**：可照 `identify` 的 `chunked_mappings` 脚本化 identify + config +
      `queue_step`，用 `TestDevice` 覆盖一次加压（及 `ResetRequired` 路径）；`--task comm` 同理。
      目前只测了段计算、引脚解析与命令编码。

### A2 错误词汇（框架 FW3）

- [x] **`ConfigError` 与分层**：`error.rs` 新增 `ConfigError`，`KlippyError` 增 `Config` 变体；
      工厂、`load.rs`、`Printer::add_object`、`McuConfig::new` 从 `Internal` 改为 `ConfigError`
      （上游 `add_object` 重复名也报 config error）。
- [x] **handler 异常 → `invoke_shutdown`**：`gcode.rs` 的 `invoke_handler` 用 `catch_unwind` 包住
      handler，panic 报 `Internal error on command:"X"` 并 `invoke_shutdown`；API 侧 `Api::dispatch`
      同样兜底，经 `Api::set_internal_error_hook`（由 `api::register` 指向 `invoke_shutdown`）
      报 `Internal Error on WebRequest: <method>`。
- [x] **config 错与 shutdown 分离**：新增 `Printer::set_error_state`（用上此前从未赋值的
      `PrinterState::Error`）；`load_config` 失败与 `connect` 期的 `KlippyError::Config` 走它，
      只有真正的内部错才 `invoke_shutdown`。
- [ ] **`lookup_object` 未命中**：上游 `lookup_object` 未命中报 config error；本仓库仍是 `Option`，
      按需加 `lookup_object_or_config_error`（消费者出现时）。
- [ ] **`Internal` 收敛的剩余点**：`mcu/object.rs` 的若干包装（`config.open()` 等）仍按 `Internal`/
      `Connection` 混用，随 FW7 校对。

### B2 MCU 关闭与错误上报（剩余，框架 FW7）

- [x] **`emergency_stop` / `clear_shutdown` 的对象与端点**：`emergency_stop` 端点已加
      （`api/endpoints/emergency_stop.rs`，进 shutdown 并回 `{}`）；每个 `McuObject` 由工厂在
      `klippy:shutdown` 上注册处理器，向固件发 `emergency_stop`（`mcu/object.rs` 的
      `on_host_shutdown`）。`clear_shutdown` 仍只被 `configure` 的复位路径使用。
- [x] **本地 shutdown 标志**：`McuObject::is_shutdown`（`Arc<AtomicBool>`，供 `'static` 事件
      处理器共享）+ `force_local_shutdown`；固件自报 `shutdown`/`is_shutdown` 时置位，
      `on_host_shutdown` 据此不回发。`bind_shutdown` 仍在 `configure` 之后绑定（足够安全）；
      要提前到 identify 之后，再靠标志区分自己发的停止——留作可选项。
- [x] **`last_stats`**：`event/stats.rs` 的 `LastStats::from_report` 按上游算术（`klippy/mcu.py:931-941`）
      把每条 `stats` 换算成 `mcu_tick_avg/stddev/awake`；`register_stats` 存进 `McuObject` 的
      槽，`get_status` 在收到过报告后带上 `last_stats`。真板已确认。
- [ ] **错误上报带载荷**：上游 `klippy:notify_mcu_error` 带 `msg` 与 details
      （`klippy/klippy.py:144` `:151`），shutdown 分析走 `klippy:analyze_shutdown`
      （`klippy/klippy.py:216-220`）。带载荷的变体已就位
      （`KlippyEvent::KlippyNotifyMcuError` / `KlippyEvent::KlippyAnalyzeShutdown`；
      `analyze_shutdown` 已触发并传 `msg`），`notify_mcu_error` 的触发点已接入
      （`Printer::bring_up` 中 MCU 连接失败路径）；`error_mcu` 模块尚未实现，暂无法
      丰富错误信息。
- [x] **`command` 的固件 `reset` 优先**：`reset_firmware` 现在先看固件有没有 `reset`，有就返回
      `ResetRequired`，由 `connect` 发 `reset` + 重连 + 重试握手（`klippy/mcu.py:733-740`
      的 `_reset_cmd` 优先）；只有没有 `reset` 时才用 `config_reset` 就地清（`mcu/config.rs`）。

### G1b gcode 调度器与上游的行为差异（框架部分 FW4）

**为什么单列一条**：G1 的骨架（命令表 / `run_script` / 输出处理器 / 内置命令 / mux）已按
`klippy/gcode.py` 落地，逐行对照后还剩一批**行为差异**。一部分只能随前置模块（GCodeIO /
toolhead / 开放事件）一起补，一部分是现在就独立可补的小行为。命令表本身够通用，G4 运动
命令不必等这条。上游行号以 `third_party/klipper/klippy/gcode.py` 为准。

**随前置一起补（GCodeIO / toolhead / 事件）**

- [~] **`GCodeIO` 未移植（已定：暂缓，不做 OctoPrint 串口仿真）**：伪 tty / 文件输入整块
      缺失——fd 读取与 `partial_input`、`pending_commands` 批量与 20 条阈值、`M112` 乱序检测
      （`m112_r` = `^(?:[nN][0-9]+)?\s*[mM]112(?:\s|$)`）、`input_log`、debuginput EOF 退出、
      `stats gcodein=`（`:390-494`）。现在输入由 API 层的 `gcode/script` 承担，客户端契约走
      Moonraker API；**结论（2026-09-21）：暂不实现**，保留为将来的可选扩展。
      将来要做时的最小路径与前置：① 先给 reactor 补 **fd 事件层**（本仓库只有定时器，无
      `register_fd`/`poll` 对应物），这是最贵的一块；② `util.create_pty`（`openpty` +
      `symlink` 到 `/tmp/printer` + 关 `ECHO` + 非阻塞）与 `GCodeIO` 对象；③ `is_fileinput`
      决定 `request_restart` / `_handle_shutdown` 是否退 `error_exit`（`:355` `:429`）；
      ④ `gcode:debuginput_exit` 需要 `send_event` 收集 handler 返回值（上游 `all(...)`）。
      tty 与 debuginput 共用同一套 `_process_data`，应一起做。详见
      [FW4 笔记](docs/work-log/2026-09-21-fw4-notes.md) 第 3 节。
- [x] **`ack()` / `need_ack`**：`GcodeCommand` 没有 `ack`（`:54-63`），这是文件输入协议的
      一部分。受影响的具体行为：`M115` 应该先 `ack(msg)`、失败才 `respond_info`（`:344-350`）。
      —— 已加 `GcodeCommand::ack`（`M115` / `M105` 已用）；本轮补全协议本身：`ack` 清
      `need_ack`（`Cell`）所以只 ack 一次，`process_line` 末尾按上游调用 `gcmd.ack()`，
      错误传播也按 `need_ack` 分支（`true` 时报告并 ack 而不中止脚本）。仍无 `need_ack=true`
      的生产者（GCodeIO）。
- [x] **事件**：错误分支不发 `gcode:command_error`（`:226`），重启不发
      `gcode:request_restart`（`:358`），debug 输入不发 `gcode:debuginput_exit`（`:433`）。
      事件总线（`KlippyEvent`）已就绪。
      —— `gcode:command_error` 已在 `process_line` 接上（handler 的 `CommandError` 触发；
      panic 走 `invoke_shutdown`、**不**触发，同上游 `:223-234`）。`gcode:request_restart` 的
      声明已补 `print_time` 载荷（上游实际带参数），触发随 C1。
- [~] **`gcode:debuginput_exit` 触发（随 `GCodeIO` 暂缓）**：上游 `_do_debuginput_exit`
      轮询 `all(send_event('gcode:debuginput_exit'))`（`:432-435`），依赖 handler 的返回值；
      本仓库 `Printer::send_event` 丢弃返回值（上游 `klippy/klippy.py:226-227` 是
      `return [cb(...)]`）。要与 `GCodeIO` 一起做（见上一条）。
- [ ] **`request_restart` 的停机前动作**：上游在 ready 时先 `toolhead.dwell(0.500)` +
      `wait_moves()` 再 `request_exit`（`:352-365`），随 **C1**；当前直接 `request_exit`
      （`gcode.rs:515-545`）。
- [ ] **`Coord`**（`:12-17`）：随 toolhead / kinematics（C1）。
- [x] **handler 内部异常 → `invoke_shutdown`**：上游用裸 `except:` 兜底，报
      `Internal error on command:"X"` 并停机（`:229-232`）。已由 `invoke_handler` 的
      `catch_unwind` 实现（A2）；本轮把 `CommandError` 与 panic 分成 `HandlerOutcome` 两支，
      好让 `gcode:command_error` 只对前者触发。

**可独立补的小行为差异**

- [x] **`default_handler` 缩水**（`:283-316`）：缺 `M105` → `ack("T:0")`、`M21`、
      `M140/M104` 且 `S=0`、`M107` / `M106`（S 关或 fileinput）这些「没有该模块时安静忽略」
      的抑制；也缺「命令名里带空格」时按 `realcmd = cmd.split()[0]` 路由到 `M117/M118/M23`
      的分支。后者是实际差异：`M117 123` 这类数字消息在 Rust 里会整串当命令名而报
      `Unknown command`（`parse_line` 只做 trim，`gcode.rs:862-917`）。
- [x] **`ECHO` 前缀**：上游 `respond_info(commandline, log=False)` → 输出 `// <line>`
      （`:368-369`）；Rust 用 `respond_raw`，没有 `// ` 前缀、不记日志（`gcode.rs` 的 `ECHO`）。
- [x] **`HELP` 未就绪提示**：上游未就绪时首行加
      `Printer is not ready - not all commands available.`，并遍历当前 active 表（`:379-388`）；
      Rust 无该提示，遍历 help 表（`gcode.rs:663-678`）。
- [ ] **`M115` 版本号来源**：上游取 `start_args['software_version']`（`:344-350`），Rust 用
      `CARGO_PKG_VERSION`。当前 `StartArgs::collect` 的 `software_version` 本身就填
      `CARGO_PKG_VERSION`，且宿主没把它接到 `GcodeDispatch`（`Printer` 不持有 `StartArgs`），
      所以行为差异要等 **D1** 的 start args wiring 才有意义，一并做。
- [x] **`get_status` 的构建口径**：上游返回缓存的 `status_commands`、按 **active 表**构建
      （未就绪只列 base 的 8 条内置，`:176-184`）；Rust 每次从 `commands.ready` 全量重建
      （`gcode.rs:578-592`）。未就绪阶段 `objects/query` 看到的命令集合不同。
- [x] **未就绪时停机不打印**：上游 `_handle_shutdown` 在 `not is_printer_ready` 时直接
      return（`:186-193`）；Rust 无条件发 `Klipper state: Shutdown`（`gcode.rs:335-343`）。
- [x] **`is_traditional_gcode` 判定**：上游用 `float(cmd[1:])`（`:125-131`），Rust 只看首字母
      大写 + 次字符数字（`gcode.rs:794`）。`M1ABC` 这类上游拒绝注册、Rust 接受。
- [x] **`parse_extended` 的 shlex 保真**：Rust 手写解析只做引号切换 + `#`/`;` 截断，不处理
      反斜杠转义 / 引号拼接等 `shlex` 语义（`gcode.rs:937-983` 对 `:266-281`）。
      —— 已补：单引号内原样、双引号内只转义 `"`/`\`、引号外退格去反斜杠、相邻引号拼接、
      尾部悬空反斜杠报错。
- [x] **校验和 `*123`**：上游 `get_raw_command_parameters` 会剥掉尾部校验和（`:40-51`），
      Rust 的 `raw_parameters` 不剥（`gcode.rs:919-935`）。只在文件 / 串口输入路径上有影响，
      连同 `GCodeIO` 一起看。
- [x] **`register_command(cmd, None)` 注销**：上游支持注销并返回旧 handler（`:133-141`），
      Rust 无注销、重复注册直接报错（`gcode.rs:325-350`）。
      —— 已加 `GCodeDispatch::unregister_command`（返回旧 handler，未知名字返回 `None`）。
- [x] **参数访问器缺口**：缺 `above`/`below`、`get_int` 的 `minval/maxval`、通用
      `get(parser=…)`；缺 `get_command_parameters` / `get_raw_command_parameters`（raw 只在
      内部 `Parsed`）；也没有 `create_gcode_command`（字段私有，外部无法构造 gcmd）
      （`:23-91` `:244`）。
      —— 已补齐：通用 `get`（parser + `minval`/`maxval`/`above`/`below`）、`get_int_bounded`、
      `get_float_bounded`、`get_command_parameters`、`get_raw_command_parameters`（剥行号与
      `*<checksum>`）、`GCodeDispatch::create_gcode_command`。后者的消费者是 `homing` /
      `probe` / `safe_z_home` / `bed_mesh` / `gcode_arcs`（**不是** `gcode_macro`）。
- [x] **`run_script_from_command`**（原先未列）：上游 handler 内部入口（`:237-238`），消费者
      `gcode_macro` / `pause_resume` / `firmware_retraction` / `hall_filament_width_sensor`。
      已加，与 `run_script` 同实现（本主机无 dispatcher mutex），是 H3 的直接前置。
- [ ] **`get_mutex` 等价物**（原先未列）：上游 `gcode.get_mutex()`（`:242-243`）被
      `bed_mesh`（`:307`）与 `idle_timeout`（`:70` `:90`）用来判断「是否有脚本在跑」；
      本主机无 reactor mutex，是否需要等价物（脚本占用标志）等 C1 与那两个模块落地再定。
- [x] **mux 缺省项（`value=None`）不可达**（**优先，含测试**）：`dispatch_mux` 用
      `contains_key(&None)` 认出缺省项，但键缺席时把请求值取成 `""` 再用 `Some("")` 查表，
      永远命中不了 `None`，于是走到「值不合法」错误分支（`gcode.rs:680-733` 对 `:317-342`）。
      实测：注册 `SET_PIN` 的 `PIN=None` 后执行 `SET_PIN VALUE=1`，报
      `The value '' is not valid for PIN. Options: `。当前库里只用 `Some(name)` 注册，未覆盖。
- [x] **mux 错误提示的 `Did you mean`**：上游按 dict 迭代序取「最后一个匹配」（`:317-342`），
      Rust 对 values 排序后取第一个匹配（`gcode.rs:718-733`）——措辞更稳定，属有意偏离；
      要么对齐上游，要么在文档里记一句。
- [x] **清理 `src/core/parser.rs`**：`parse_gcode` / `parse_gcode_line` 是未被引用的存根
      （`#[allow(dead_code)]`），真正的解析在 `gcode.rs`；删除或并入 `gcode.rs` 的测试。

### G4 运动命令（G0/G1/G28/G92/M114…）

- [ ] 由 toolhead 注册，随 **C1**；gcode 层不需为它们改什么，只要命令表够通用
      （含 `register_mux_command`，给 `SET_PIN` 这类 `PIN=` 选择用）。

### B4 其余端点（机制部分 FW9）

`api-reference.md` 有、`endpoints/mod.rs` 的表里标「not started」的其余部分，各自等它读的
对象先存在：

- [ ] `emergency_stop`（`klippy/webhooks.py:322` `_handle_estop_request`）。
- [ ] `register_remote_method`：方法表与推送（`klippy/webhooks.py:319` `:323` `:391`
      `:412`）。
- [ ] `pause_resume/{pause,resume,cancel}`：等 `pause_resume` 对象。
- [ ] `query_endstops/status`：等 endstop / homing。
- [ ] `bed_mesh/dump_mesh` 与 `*/dump_*` 多路复用端点（`klippy/webhooks.py:335`
      `_handle_mux`）：等对应 extras（`bed_mesh`、`adxl345` 等）。

### F MCU 基础资源（F3、F6–F9）

上游把这些叫 printer objects 下面的「资源」：主机用一个 **oid** 和一个 **pin 描述**
建立资源对象，把 `config_*` 命令攒起来，在 `finalize_config` 之前算一个 CRC 一次性下发，
之后用 `queue_*` / `set_*` / `*_transfer` 命令驱动。命令层（`allocate_oids` / `get_config` /
`finalize_config` / `get_uptime` / `emergency_stop` / `get_clock`）已就位，配置构建层（F1）
与 pin 解析（F2）也已完成，数字输出、PWM（F4）与 ADC（F5）三个 `config_*` 资源已落地
（见文末索引），SPI/I2C 总线（F6/F7）也已落地，剩下的缺口是命令队列/运动同步输出（F3）、
endstop/trsync（F8）与其余固件资源（F9）。

#### F3 剩余：`MCU_bus_digital_out`（框架 FW6f，见 [FW6 调查](docs/work-log/2026-09-21-fw6-notes.md)）

- [ ] `MCU_bus_digital_out`（`klippy/extras/bus.py:337` 以后）：挂在命令队列上、与运动
      同步的输出；需要命令队列/运动层（C1）。
- 运行期 `queue_digital_out` 收的是**绝对固件时钟**；print_time → clock 的换算属于时钟层
      （`cmd/clock.rs` 的 `ClockSync` 现只有 `get_clock`，偏移跟踪未做）。
- **不引入 host 侧 serialqueue**（上游靠它把命令压到 `req_clock` 再发）：FW6 的单 MCU
      链路里 `trsync_start`/`endstop_home`/`queue_digital_out` 都带绝对时钟，固件自己调度；
      `MCU_bus_digital_out.update_digital_out(minclock, reqclock)` 改为发
      `queue_digital_out(reqclock, value)`。多 MCU 时再评估。

#### F6 SPI 总线

上游 `MCU_SPI`（`klippy/extras/bus.py:42-155`）：

- [x] 设备侧：`config_spi oid=%c pin=%u cs_active_high=%c`（或 `config_spi_without_cs`），
      总线侧：`spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u`，收发：
      `spi_send oid=%c data=%*s`、`spi_transfer oid=%c data=%*s` /
      `spi_transfer_response oid=%c response=%*s`；还有 `config_spi_shutdown`
      （固件 `src/spicmds.c:37` `:62` `:122` `:157`）。
- [x] 软件 SPI（`spi_software_{miso,mosi,sclk}_pin`）：`spi_set_sw_bus`（新），固件
      `src/spi_software.c`。
- [x] `MCU_SPI_from_config`（`bus.py:124`）：从 section 读 `cs_pin` / `spi_speed` /
      `spi_bus` / 软件引脚，`cs_pin=None` 时不占用引脚的共享。
- [x] 旧式 `spi_set_software_bus`（rate 版）回退：`cmd::spi::add_software_bus` 按
      `try_lookup_command` 二选一（新式传 host 算好的 `pulse_ticks`，旧式传 `rate`），
      回退时打 deprecation 警告。只能单测——vendored 固件只有新式。
- [ ] `spi_transfer_with_preface` 与 `setup_shutdown_msg`：`ConfigSpiShutdown` 命令
      已定义，但资源/消费者未接（设备需要在 shutdown 时发消息时才用得上）。

已落地：`cmd/spi.rs`（命令层，`%*s` 走二进制 `ArgType::Bytes`）、
`mcu/resource/spi.rs`（`McuSpi` 资源 + `SpiMode`，片选由固件驱动）、
`extras/spi_device.rs`（`[spi_device <name>]` 构造器 + `SPI_TRANSFER` / `SPI_SEND`
调试命令；与 `[i2c_device]` 共用 `extras/bus_debug.rs` 的 hex/异步桥）。

真机验证（STM32F103 + W25 flash，CS=PA15，SPI1 重映射 PB3/PB4/PB5）：硬件 `spi1a`
与软件 bit-bang 两条路都读出 JEDEC ID `ef 30 13`、状态寄存器 `0x00` 与地址 0x00 的
数据。

#### F7 I2C 总线

上游 `MCU_I2C`（`klippy/extras/bus.py:161` 以后）：

- [x] 设备侧：`config_i2c oid=%c`，总线侧：`i2c_set_bus` / `i2c_set_sw_bus`；传输：
      `i2c_transfer` + `i2c_response`（旧式）或 `i2c_write` / `i2c_read` +
      `i2c_read_response`（新式）（固件 `src/i2ccmds.c:32` `:48` `:107`）。
- [x] `i2c_bus_status` 不是 `SUCCESS` 时按上游 `invoke_shutdown`（`bus.py:295-300`）：
      `McuI2c::transfer` / `write` 在非 SUCCESS 时停机（消息 `MCU 'x' I2C request to
      addr N reports error S`）并返回 `McuError::I2cBus`；探测用的
      `transfer_without_shutdown` / `write_without_shutdown` 只返回错误，`IIC_READ` /
      `IIC_WRITE` 用后者（探不存在的地址不应停机）。`i2c_write` 的 retry 与
      `async_write_only` 仍是可选分支。
- [x] 软件 I2C（`i2c_software_{scl,sda}_pin`）：`i2c_set_sw_bus`，固件 `src/i2c_software.c`。
- [x] 旧式 `i2c_set_software_bus`（rate 版）回退：`cmd::i2c::add_software_bus` 按
      `try_lookup_command` 二选一，回退时打 deprecation 警告。只能单测。
- [x] 通用构造器与 `[i2c_device <name>]` section（G）：读 `i2c_mcu` / `i2c_address` /
      `i2c_speed` / `i2c_bus` / `i2c_software_{scl,sda}_pin`，经 `McuObject::setup_i2c`
      构造 `McuI2c`；并注册 `IIC_WRITE` / `IIC_READ` 两个调试命令（上游无此 section
      与命令，为真机自测而加：`IIC_READ DEVICE=<n> WRITE=<hex> READ_LEN=<n>`）。
      真正的 sensor 消费者仍属 F9。

已落地：`cmd/i2c.rs`（命令层，`%*s` 走二进制 `ArgType::Bytes`，`i2c_transfer` 用固件的
`write=`）、`mcu/resource/i2c.rs`（`McuI2c` 资源 + `I2cMode`，新式组合传输只用 `i2c_read`）、
`Mcu::try_lookup_command`（按字典原始格式串精确匹配，检测新旧传输风格）、
`PrinterPins::resolve_bus_name` / `resolve_bus_value`（`i2c_bus=%u` 需 host 先解析枚举值）、
`extras/i2c_device.rs`（`[i2c_device]` 构造器 + 调试命令）。软件总线的新旧命令选择
（`i2c_set_sw_bus` ↔ `i2c_set_software_bus`）在 `cmd::i2c::add_software_bus`。

#### F8 endstop / trsync（与 C1 共享，框架 FW6a–FW6d，见 [FW6 调查](docs/work-log/2026-09-21-fw6-notes.md))

- [ ] **FW6a** 命令层与 MCU 触发：`cmd/endstop.rs`、`cmd/trsync.rs`、`StepperStopOnTrigger`；
      `PinChip::setup_endstop` + `PrinterPins::setup_endstop`；`mcu/resource/endstop.rs`
      （`McuEndstop` + `home_start`/`home_wait`/`query_endstop`）；`motion/trsync.rs` 的单 MCU
      `TriggerDispatch`/`McuTrsync`（`bind_event` 收 `trsync_state` 报告并重发
      `trsync_set_timeout`；`trsync_trigger`/`trsync_state` 完成 completion）。
- [ ] **FW6b** 消费者：`[stepper_*]` 的 `endstop_pin`/`homing_*` → `Rail`；`query_endstops`
      对象 + `query_endstops/status` 端点 + `M119`/`QUERY_ENDSTOPS`。（**框架验收**）
- [ ] **FW6c** 回零精度前置：`stepcompress` 的 history / `find_past_position` / `extract_old`；
      `motion::Stepper::mcu_position`/`past_mcu_position`/`note_homing_end`；`McuStepper` 发
      `reset_step_clock`/`stepper_stop_on_trigger`。
- [ ] **FW6d** `HomingState` + `ToolHead::drip_move` + `extras/homing.rs`（`Homing`/`HomingMove`/
      `G28`）；`homing:*` 事件触发。与 FW5 联合验收真板单轴 `G28`。
- [ ] `MCU_endstop`（`klippy/mcu.py:340-407`）：`config_endstop oid=%c pin=%c pull_up=%c`、
      回零 `endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u
      pin_value=%c trsync_oid=%c trigger_reason=%c`、查询 `endstop_query_state oid=%c` /
      `endstop_state oid=%c homing=%c next_clock=%u pin_value=%c`（固件 `src/endstop.c:72`
      `:97` `:115`）。
- [ ] `MCU_trsync` / `TriggerDispatch`（`mcu.py:155-339`）：多 MCU 同步触发
      （`src/trsync.c`），回零结束时用来同时停各个轴。这是 C1 回零的直接前置。
      **FW6 只做单 MCU**；多 MCU（`SecondarySync`/共享轴检查）后置 FW6e。
- [ ] 消费者是 `homing`（`klippy/extras/homing.py`），所以这条要等 toolhead 的接口
      （`home_rails` / `get_trigger_position`，见 C1）一起定。

#### F9 其他输入与外设资源

建立在 F1–F6 之上，各自一个 `config_*` + 查询/事件。这一节只列**固件侧资源**；
在它之上建的**宿主 extras 消费者**（buttons / pulse_counter / neopixel / sdcard / lcd /
sensor_bulk / 各类传感器）按域归到 H5–H8，两边互为前置：

- [ ] `buttons`（`src/buttons.c`，`config_buttons` / `buttons_add` / `buttons_query` /
      `buttons_ack`）—— 暂停/恢复按钮、耗材检测。
- [ ] `pulse_counter`、`neopixel` / `dotstar` / `led`、`tmcuart`、`sdcard` / `sdio`、
      `lcd_hd44780` / `lcd_st7920`、`sensor_bulk`（批量传感器上报）与各类 SPI/I2C 传感器
      （`sensor_adxl345` / `sensor_lis2dw` / …）。
- 这些是 extras，不阻塞运动；等 F1–F6 完成、真有对应 section 时再逐个接。消费者见
      H5（TMC/tmcuart）、H6（sensor_bulk/加速度计）、H7（buttons/pulse_counter/trigger_analog）、
      H8（lcd）。

### C1 toolhead 与 kinematics（框架 FW5）

kinematics 已随 Printer 重构删除，从这里重新开始。动工前调查见
[FW5 笔记](docs/work-log/2026-09-21-fw5-notes.md)，已定的设计取舍：

- **chelper 用 Rust 分层重写**（不用 FFI）；「FFI 复用 C」记为 `[~]` fallback，待真板出现
  性能问题再返工（前瞻：热路径每 flush 约 40 µs，预算 5–10 ms，Rust ≈ C，FFI 无性能收益）。
- **`Coord([f64; 4])`**；`calc_position` 返回 `[Option<f64>; 3]`（`None` 只出现在从 stepper
  位置反推轴位置这一处，见 `extras/homing.py:245`）。
- **`motion_quuing` 照搬上游分层**：`MotionQueuing` 持有 trapq 与每 MCU 的输出，flush 调度
  在它里面。
- **`stepcompress` 先简化、后完整**：FW5c 先简化（每步一条 `queue_step`）让流程能过；
  **FW5f 已完成完整压缩**（`(interval,count,add)` + `max_error` + `CHECK_LINES`，与上游向量对拍）。
- **`Kinematics::check_move` 用窄 context**，不把 `Move` 暴露给 kinematics。
- **验收先到 `G1`**；`G28` 需要 endstop/trsync（F8），与 FW6 联合验收。

阶段拆分（详见笔记）：

| 阶段 | 内容 | 验收 |
|---|---|---|
| FW5a | ✅ `Coord`（`mathutil.rs`）；`clocksync` 回归（`ClockEstimator` + `McuClock`） | host 单测（10 个） |
| FW5b | ✅ `Move`、`LookAheadQueue`、`trapq` | host 单测（13 个） |
| FW5c | ✅ `itersolve` + `kin_cartesian`；`stepcompress` 简化 + `warn_and_wait`（策略可注入，测试传 0） | host 单测（12 个，含告警/宽限） |
| FW5d-1 | ✅ `ToolHead`、`MotionQueuing`、`Stepper`（host 链路） | host 单测（8 个）：`G1` 出正确 `queue_step` |
| FW5d-2 | ✅ `McuStepper` 资源 + `setup_stepper` + `StepCommand`→MCU 命令转换 | host 单测（3 个）；`[stepper_*]` section 注册与真板读回留 FW5e |
| FW5e-1 | ✅ `Kinematics` trait + `CartesianKinematics` + `MoveContext`（窄接口）+ ToolHead 集成 | host 单测（8 个） |
| FW5e-2 | ✅ `[stepper_*]`/`[printer]` section 注册、`G1`/`G0`（`G4`/`M400`/`SET_KINEMATIC_POSITION`）、连接期 `stepper_get_position` 对齐 | host/装载单测；真板读回与 G1 `[~]`（板只知 X 引脚，见 FW5f 记录） |
| FW5f | ✅ `stepcompress` 完整压缩（`(interval,count,add)`/`max_error`/`check_line`/方向翻转/远步重锚） | 与上游 C 向量对拍 + 重构性质单测；真板 `--task motion`：500 步 → 3 条命令，读回 500 |

细节条目：

- [x] 先立 **toolhead 对象**：位置记忆（`commanded_pos`）、trapq、速度/加速度上限，
      回零与移动的入口（上游 `klippy/toolhead.py:389` `:400` `:482` `:507` `:522`）。
      FW5d-1 立骨架，FW5e-2 补 `set_position`/`kinematics` 访问器。
- [x] 再加回 **`Kinematics` trait 与 `kinematics/`**：按上游由 toolhead 读
      `[printer] kinematics` 装载（`klippy/toolhead.py:242`），不是交给 Printer。FW5e-1。
- [ ] 回零协议：上游 kinematics 调 `homing_state.home_rails(rails, forcepos, movepos)`、
      `set_homed_position(pos)`、`get_trigger_position`、`set_stepper_adjustment`。
      没有这些，任何真实 kinematics 的 `home()` 都写不出来。（**FW6d** 与 F8 联合定）
- [x] stepper 句柄：上游能 `get_commanded_position()` / `get_step_dist()` / `set_trapq()` /
      `setup_itersolve()`；`calc_position` 的输入就从这里来。FW5d/FW5e-2：
      `motion::Stepper` + `McuStepper`（oid/引脚/`query_position`）。
- [x] step 生成层的运动学：上游 `rail.setup_itersolve('cartesian_stepper_alloc', axis)`；
      这部分上游是 C（`klippy/chelper/` 的 `stepcompress.c`、`itersolve.c`、`kin_*.c`、
      `trapq.c`、`kin_shaper.c`，见审计文档 §4.1），Rust 侧整体重写（决定见上）。
      FW5c 已落地（`itersolve` + `kin_cartesian`），`stepcompress` 在 FW5f 补完整压缩。

### C2 配置装载收尾（框架 FW1）

- [x] **option 级校验**：访问追踪当 schema 已落地：`ConfigWrapper`（类型化 getter 一处解析并记帐）、
      `AccessTracking`（键小写化，值为解析后的 JSON）、`check_unused`（`config/validate.rs`），
      未认领选项报上游原文 `Option 'x' is not valid in section 'y'`。
- [x] **住户与阶段**：`section!` 新增 `phase = early|generic|late` 与 `object = "<name>"`，
      装载器按阶段遍历、按声明名注册；`configfile` 作为无节对象在 `gcode` 之后注册。
- [x] **`configfile` 对象**：`get_status` 的 `settings`/`config`/`warnings` 已接，`objects/list` 可见。
- [x] **`[printer]` / toolhead 本体**：晚阶段住户已接入（`section!("printer", phase = late,
      object = "toolhead", …)`），消费者是 C1 的 toolhead（FW5e-2）。
- [ ] **autosave / `SAVE_CONFIG` / `deprecate`**：`configfile` 的剩余状态与写入路径，
      属模块而非框架，单列（依赖 FW1）。
- [ ] **`getchoice` 与范围/列表上限**：wrapper 目前只做类型解析 + 两个自定义范围检查；
      上游的 `minval/maxval/above/below/count` 统一文案随各 extras 的 option schema 补。

### D1 主机层 start args / rollover / 日志（框架 FW8）

- [x] **`--logfile`**：`AppArgs.log_file`（`--logfile`）+ `logging::init(verbose, log_file)`：格式化行同时写 stdout 与文件（开窗时跳过 stdout），开不了文件就降级到 stdout；`StartArgs.log_file` 填上，`info` 报真实路径（`logging.rs`、`klippy.rs`、`main.rs`、`bin/klippy/main.rs`）。
- [x] **rollover info**：主机层 `logging` 的 `set/clear/write_rollover_info`，启动与每次重启写
      `versions` 块 + `Log rollover at <asctime>` 横幅（贴 `klippy/queuelogger.py:31-53`）。
      `Printer::set_rollover_info` 那套上游 API 随需要它的模块（toolhead/webhooks）再加。
- [ ] **`StartArgs` 仍只 info 需要的字段**：`apiserver`、`start_reason`、debug 输入输出、
      每个 MCU 的字典路径还没进来（`api/start_args.rs`）；`start_reason` 已在 `Printer` 上，
      不重复搬进 `StartArgs`。
- [ ] **`StartArgs` 接到消费方**：`software_version` 目前只被 `info` 端点读；`M115` 要按上游
      读它（`klippy/gcode.py:344-350`），需要把版本串接到 `GcodeDispatch`（随 G1b 的「`M115`
      版本号来源」一并做）。

### D2 重启循环（剩余，框架 FW8）

循环本身与四种 `restart_method`（`command` / `arduino` / `cheetah` / `rpi_usb`）的物理分派
都已完成（见文末索引；方法与连接期门控见 `docs/klippy/developer-manual/mcu-config.md`）。
剩下的三块：

- [x] **`rpi_usb` 的连接期门控**：`restart::restart_before_bringup` 按上游两个点判断
      （`check_restart_on_attach` / `check_restart_on_send_config`，`klippy/mcu.py:690-700`）：端口不存在
      → “enable power”，否则 “full reset before config”；`McuObject::connect` 据此
      `request_exit("firmware_restart")` 并中止本次 bring-up，重启循环下一轮
      （`is_firmware_restart()`）才断电、开端口、发配置。**本机没有可控 VBUS 的 hub，
      只有决策逻辑的单测（`mcu/restart.rs`），没有真机验证。**
- [x] **CRC 不匹配改走物理复位**：`reset_firmware` 先看固件有没有 `reset`，有就 `ResetRequired`
      → `reset` + 重连 + 重试握手（真重启，清定时器与步进队列）；只有没有 `reset` 时才
      `config_reset` 就地清。`rpi_usb` 的 CRC 不匹配则由上一项的门先请求 firmware_restart。
      上游那一条 `start_reason == 'firmware_restart'` 仍已配置时 raise “Failed automated reset”
      的前置门还没做（`klippy/mcu.py:1053-1056`）。
- [x] **重启后的 g-code 输出订阅**：`GcodeSubscribeOutput` 把
      `(PushTarget, template)` 订阅存在自己（端点在 `Api` 上跨重启存活），并有
      `watch_restarts` 任务每 250 ms 比对当前 `GCodeDispatch` 是否换了实例，换了就把还活着的
      订阅重新挂上去（`api/endpoints/gcode.rs`，单测覆盖）。
- [~] **真板启动抖动（已归档观察项，不再单独处理）**：复查结论（2026-09-21）：
      ① 原先的 `MCU 'mcu' shutdown: Rescheduled timer in the past` / `timeout: no response for
      config` 在当前代码上**复现不出来**（连续启动、stress 种子、留步进队列、杀在 bring-up 中段
      等场景均 `ready`，基线亦同）；原先那条很可能就是就地 `config_reset` 的窗口，已被「`reset` 优先」
      消除。② 另有一个**间歇、与主机实现无关**的现象：在 **identify 握手中**强杀 host 后，板端会
      十几秒不应答（下一个 host 等满 `IDENTIFY_TIMEOUT=10s` 报连接错误，随后板子自行恢复）。
      软件侧干扰源已排除（无 ModemManager/brltty/autosuspend/残留进程）；`dmesg` 受限看不到 USB 层，
      硬件/USB 因素未排除。**判定为板/USB 链路层的已知观察项，不阻塞任何框架任务；除非将来做
      重启相关改动（D2/D3）或用户主动要求，不再为此单独排期/调查。**
- [x] **字典装载前的固件输出不再报错**：`Mcu` 加 `identified` 旗标（接收任务共享）；字典装上之前
      的 decode 失败按预期降到 `debug`，装上之后的未知 id 仍是 `error`（`mcu/mod.rs`）。

### D3 `command` 接管一块还在跑的板子（框架 FW7）

`command` 的 `config_reset` 要连上才能发，而重连时对手的序号接着上一条会话走——这不是边缘
情况：`rpi_usb` 切不了 VBUS 的机器会当场回退到 `command`（`mcu/object.rs`），普通 `RESTART`
也只是重建对象、重新 open + identify，同样要接上一块没被复位的固件。

**传输层的接管已完成**：收发两端共用一个 4 位序号，接收任务把空帧的号报给发送任务
（重复 ack 即 NAK），发送端按「更大 → 采纳并换号重发未确认块 / 不更新 → 原号重发 / 否则
ack」settle，并用 `Mcu::took_over_session()` 让 `rpi_usb` 判断“有没有真重启”
（`mcu/mod.rs`、`mcu/object.rs`）。测试见 `docs/klippy/developer-manual/testing.md` 的
`mod.rs` / `host.rs`（含对着真 host 库的同进程二次连接）。

**还剩**：

- [x] **RTO 定时重传**：`Sender` 加了 `rto`/`retransmit_at`，发送任务在 `select!` 里等它；
      到期就把未确认的块原号重发，并像上游一样把等待翻倍（`serialqueue.c:422-460`，
      `MIN_RTO`=25 ms、`MAX_RTO`=5 s）。单测：不发 ack 的设备能收到重传（`mcu/mod.rs`）。
- [x] **固件 `reset` 优先**：见 B2（已完成）。

### E1 文档

- [x] `docs/klippy/developer-manual/`：补 printer 一节，并在 README 的分层表里加
      `printer` 一行（现在只有 msg / mcu / cmd / event / identify / api）。
- [x] **过期描述**：`printer.rs` 头注释仍写「No part is loaded from the config into it
      yet … so it still runs empty」，而 `load.rs` 已经装载 `[mcu]`；改了代码就要回头改
      这几句。

### E2 `python_path` 的取消（**远期，依赖外部项目**）

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

## 上游 extras 覆盖盘点

上游 133 个 extras（不含 `__init__.py`）里，本仓库目前只有 `board_pins` ✅、
`output_pin` ◐、`bus`（SPI/I2C 框架）✅；其余按域归并成 H1–H12。逐模块的完整对照表
（含固件命令模块、端点、判为不适用者）见
[上游功能覆盖审计](docs/work-log/2026-09-21-upstream-coverage-audit.md)。

### H1 加热与温度

- [ ] `heaters.py` 框架：`get_heater`、PWM 定时器、`verify_heater` 调度
      （`klippy/extras/heaters.py`）。
- [ ] `heater_bed.py` / `heater_generic.py`：section 住户、`M140`/`M190` /
      `SET_HEATER_TEMPERATURE`。
- [ ] `pid_calibrate.py`（`PID_CALIBRATE`）、`verify_heater.py`。
- [ ] 传感器：`temperature_sensor.py` / `thermistor.py` / `adc_temperature.py` /
      `adc_scaled.py` / `spi_temperature.py`（MAX31855/56/65）/ `temperature_combined.py` /
      `temperature_host.py` / `temperature_mcu.py` / `temperature_probe.py` /
      `temperature_fan.py`。
- 依赖 F4（PWM）、F5（ADC）、F6（SPI 温度）、C1（`temperature_fan` 随运动）。

### H2 风扇与通用输出

- [ ] `fan.py`（`[fan]`，`M106`/`M107`）、`fan_generic.py`、`heater_fan.py`、
      `controller_fan.py`。
- [ ] `pwm_tool.py`（队列化 PWM，随运动）、`pwm_cycle_time.py`、`static_digital_output.py`、
      `static_pwm_clock.py`。
- [ ] `multi_pin.py`、`servo.py`、`duplicate_pin_override.py`。
- [ ] `led.py`、`neopixel.py`、`dotstar.py`（固件 `neopixel.c`）。
- [ ] I2C/SPI 数字电位器、DAC、LED 驱动：`ad5206.py`、`mcp4018.py`、`mcp4451.py`、
      `mcp4728.py`、`dac084S085.py`、`pca9533.py`、`pca9632.py`、`sx1509.py`。
- 与 **G2b** 的分工：G2b 只补 `[output_pin]` 的时序；H2 是其余输出类 extras。

### H3 G-Code 宏与脚本

- [ ] `gcode_macro.py`：`[gcode_macro]`、变量、`rename_existing`，以及读
      `printer.objects` 的反射式能力（**Q5**）。
- [ ] `save_variables.py`（`SAVE_VARIABLE` / `[variables]`）。
- [ ] `delayed_gcode.py`（`[delayed_gcode]`）。
- [ ] `respond.py`（`RESPOND` / `M118`）。
- 前置：**G1b** 的 `create_gcode_command` 与参数访问器（宏类模块要构造 gcmd）。

### H4 打印流程与 SD 卡

- [ ] `virtual_sdcard.py`：主机侧文件打印、`M24`/`M25`/`M27`、进度。
- [ ] `print_stats.py`、`display_status.py`（`M73`/`M117`）。
- [ ] `pause_resume.py`（`PAUSE`/`RESUME`/`CANCEL_PRINT` + 三个端点，见 **B4**）。
- [ ] `exclude_object.py`、`sdcard_loop.py`、`firmware_retraction.py`（G10/G11）。
- 依赖 F9（固件 `sdiocmds.c` 的 sdcard 资源）、C1（`gcode_move` 的位置恢复）。

### H5 TMC 步进驱动

- [ ] `tmc.py` 公共框架（寄存器、StallGuard、`DUMP_TMC`/`SET_TMC_*`）。
- [ ] `tmc_uart.py`（固件 `src/tmcuart.c`）。
- [ ] SPI 型：`tmc2130.py`、`tmc5160.py`；UART/SPI 型：`tmc2208.py`、`tmc2209.py`、
      `tmc2240.py`、`tmc2660.py`。
- 依赖 F6/F7、F9（tmcuart）、C1（stepper 对象）。

### H6 传感器与块状数据

- [ ] `bulk_sensor.py` 框架 + 固件 `sensor_bulk.c` + 各 `*/dump_*` 端点（**B4**）。
- [ ] 加速度计：`adxl345.py`、`mpu9250.py`、`icm20948.py`、`lis2dw.py`、`lis3dh.py`、
      `bmi160.py`（固件 `src/sensor_*.c`、`sos_filter.c`）。
- [ ] `angle.py`（磁编码）、`ldc1612.py`（涡流）、`hx71x.py`、`ads1220.py`、
      `ads131m0x.py`、`ads1x1x.py`。
- [ ] `load_cell.py` / `load_cell_probe.py`（称重，配固件 `trigger_analog.c`）。
- [ ] `input_shaper.py` / `resonance_tester.py` / `shaper_calibrate.py` / `shaper_defs.py`。
- 依赖 F5/F6/F7、F9（sensor_bulk）、C1。

### H7 输入与外设

- [ ] `buttons.py` / `gcode_button.py`（固件 `buttons.c`）。
- [ ] `pulse_counter.py`（固件 `pulse_counter.c`）。
- [ ] `trigger_analog.py`（固件 `trigger_analog.c`）。
- [ ] 断料/线宽：`filament_switch_sensor.py`、`filament_motion_sensor.py`、
      `hall_filament_width_sensor.py`、`tsl1401cl_filament_width_sensor.py`。
- [ ] 固件 `initial_pins.c` 的初始引脚状态。
- [ ] 特定板/芯片：`samd_sercom.py`、`replicape.py`、`palette2.py`。
- 依赖 F3（GPIO）、F5（ADC）、F9。

### H8 LCD 显示与菜单

- [ ] `display/display.py` 框架与 `hd44780.py`、`hd44780_spi.py`、`aip31068_spi.py`、
      `st7920.py`、`uc1701.py`。
- [ ] 菜单：`display/menu.py`、`display/menu_keys.py`、`display.cfg`、`menu.cfg`；
      事件 `menu:*`。
- [ ] 固件 `lcd_hd44780.c` / `lcd_st7920.c`。
- 依赖 F9（固件侧 LCD）、G1b（`create_gcode_command`，菜单脚本要构造 gcmd）。

### H9 探测 / 调平 / 校准

- [ ] 探针：`probe.py`、`bltouch.py`、`smart_effector.py`、`manual_probe.py`、
      `safe_z_home.py`、`endstop_phase.py`。
- [ ] 调平：`bed_mesh.py`（含 `bed_mesh/dump_mesh` 端点）、`bed_tilt.py`、
      `quad_gantry_level.py`、`z_tilt.py`。
- [ ] 螺丝：`bed_screws.py`、`screws_tilt_adjust.py`。
- [ ] 校准：`delta_calibrate.py`、`axis_twist_compensation.py`、`skew_correction.py`、
      `z_thermal_adjust.py`、`tuning_tower.py`。
- [ ] 回零周边：`homing_override.py`、`homing_heaters.py`；事件 `homing:*`、
      `probe:update_results`。
- 依赖 C1、F8（endstop/trsync）、H3（宏）、H12（`mathutil`）。

### H10 运动相关 extras

- [ ] `gcode_move.py`（G0/G1/G28/G92/M114…，即 **G4** 的实现体）。
- [ ] `gcode_arcs.py`（G2/G3）、`force_move.py`、`manual_stepper.py`、
      `stepper_enable.py`、`extruder_stepper.py`。
- [ ] `idle_timeout.py`（`idle_timeout:*` 事件）、`motion_queuing.py`、
      `motion_report.py`（`dump_trapq`/`dump_stepper` 端点，见 **B4**）。
- 依赖 C1（toolhead/kinematics）；`gcode_move` 同时是 **G4** 的前置。

### H11 主机运行时与调试

- [ ] `statistics.py`：周期上报主机统计（CPU/内存）。
- [ ] `error_mcu.py`：MCU 错误详情，供 shutdown 分析（接 **B2**）。
- [ ] `canbus_ids.py` / `canbus_stats.py`：CAN 节点分配与状态（接 `[mcu]` 的 canbus 选项）。
- 判为不适用：`garbage_collection.py`、`aio_executor.py`、`parsedump.py`（审计文档第 18 节）。

### H12 核心工具补齐

- [x] `mathutil.py` 的 `Coord`：`mathutil.rs` 的 `Coord([f64; 4])`（FW5a）；几何算法
      （`trilateration`/`gaussian_solve`）随 delta/probe（H9）。
- [ ] `util.py` 的反射与注册表 helper：`get_heater` / `get_sensor` / 前缀式
      `lookup_objects`（**Q5**）。
- [x] `clocksync.py`：`cmd/clock.rs` 的 `ClockEstimator`（EWMA 回归、最小 RTT、`print_time_to_clock`/
      `estimated_print_time`/`clock32_to_clock64`）+ `McuClock` 接入（每个 `get_clock` 采样一次，
      FW5a）。
- [ ] `pins.py` 消费侧接口（`get_pin_type`、重命名等）——随 **H7** 等消费者。

## 未决问题

- [x] **Q2 事件系统的形状**：已选定大枚举：`KlippyEvent`（`src/core/klippy/event/`）
      覆盖上游全部 35 个事件名，声明分散在 `event/decl/`，由 `build.rs` 生成，`Unknown`
      兜底未声明事件名。设计见 [事件系统](docs/klippy/developer-manual/event-system.md)。
- [x] **Q3 `PrinterEvent` 是否恢复 `McuIdentify` / `AnalyzeShutdown` / `NotifyMcuError`**：
      随 Q2 一并解决。处理器签名改为 `Fn(&KlippyEvent)`，带载荷事件读变体字段；
      `mcu_identify`、`analyze_shutdown` 与 `notify_mcu_error` 均已触发。原 `PrinterEvent`
      已删除。
- [x] **Q4 `get_status` 的返回形状**：选定 `serde_json::Value`（贴上游、客户端零适配）。
      typed + serde 会把每个对象的状态变成一套并行类型，而状态本来就是给客户端看的 JSON；
      typed 只在模块内部需要时用（如 `McuConfig`），不作用于 `get_status`。
- [x] **Q5 要不要反射式能力**：要，但只做**读**，不做动态属性。已加
      `Printer::lookup_objects(module)`（前缀遍历，`klippy/klippy.py:81-88`）与
      `Printer::statuses(eventtime)`（一次取全部可查对象的状态），加上已有的
      `lookup_object` / `lookup_object_as::<T>` / `status_of`。`gcode_macro` 的
      `printer.objects` 模板视图（`klippy/extras/gcode_macro.py:13-45`）在其上实现，
      写能力（模板改对象）不做。
- [x] **Q6 退出结果的语义**：`klippy::run` 返回进程退出码，`klippy_process` 把最终的 run
      result 带回来；只有 `error_exit` 是非零（`-1`，同上游 `sys.exit(-1)`），`exit` 与
      “附件结束” 都是 0；两个 main 用 `std::process::exit(code)`（`klippy.rs`、`main.rs`）。
- [~] **Q8 GCodeIO（伪 tty / OctoPrint 串口仿真）补不补**：**已定（2026-09-21）：暂不实现**，
      归档为将来可选项，等需要时再操作。纯 API 主机（Moonraker）不需要它；代价是
      `debuginput_exit`、`is_fileinput`/`error_exit`、`stats gcodein=`、`input_log`、`M112` 乱序
      一直缺。将来做时的前置见 G1b 的 `GCodeIO` 条目（首要是 reactor 的 fd 事件层）。

## 上游事件对照清单（事件总线已就绪，逐项注册处理器）

上游 `Printer` 维护 `event_handlers` 字典（`klippy/klippy.py:36`），通过
`register_event_handler(name, cb)` 注册、`send_event(name, *params)` 分发，共 35 个事件名。

`KlippyEvent` 已声明全部 35 个名字，`Printer::register_event_handler` / `send_event` 按名
注册与分发，处理器签名为 `Fn(&KlippyEvent)`。以下事件可按优先级逐个注册处理器；依赖
关系标注在 `[依赖]` 中，`—` 表示仅依赖事件总线，其他依赖的模块已标记为其他 TODO 条目。
生命周期事件已触发，其余事件的触发点随对应模块落地。

### 生命周期事件（最高优先级）

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `klippy:mcu_identify` | MCU identify 完成后 | 无 | `klippy/klippy.py:131` | — |
| `klippy:connect` | 配置装载完成、打印机即将就绪 | 无 | `klippy/klippy.py:132` | — |
| `klippy:ready` | 打印机进入 ready 状态 | 无 | `klippy/klippy.py:162` | — |
| `klippy:shutdown` | 进入 shutdown 状态 | 无 | `klippy/klippy.py:211` | — |
| `klippy:disconnect` | 运行结束/退出时 | 无 | `klippy/klippy.py:195` | — |
| `klippy:firmware_restart` | firmware restart 前 | 无 | `klippy/klippy.py:194` | — |
| `klippy:notify_mcu_error` | MCU 通信出错时 | `msg: str, details: dict` | `klippy/klippy.py:144,151` | ✅ 已接入 |
| `klippy:analyze_shutdown` | 进入 shutdown 后分析 | `msg: str, details: dict` | `klippy/klippy.py:216-220` | — |

> **说明**：两个事件由带载荷的变体承载（`KlippyEvent::KlippyNotifyMcuError` /
> `KlippyEvent::KlippyAnalyzeShutdown { msg, details }`）。`analyze_shutdown` 已触发并传入
> `msg`，`details` 暂为空表；`notify_mcu_error` 已接入 `bring_up` 中 MCU 连接失败路径，
> `error_mcu` 模块尚未实现。

### MCU 相关事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `klippy:mcu_identify` | 每个 MCU identify 后 | 无 | `klippy/mcu.py:797,929,1001` | — |

> 已由 MCU 层事件 `Starting` 覆盖部分语义，但上游的 `klippy:mcu_identify` 是
> Printer 级事件，供 extras（probe、tmc、temperature_mcu 等）做初始化。

### 运动/回零事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `homing:home_rails_begin` | 回零开始 | `homing_state` | `klippy/extras/homing.py:80` | C1 |
| `homing:home_rails_end` | 回零结束 | `homing_state` | `klippy/extras/homing.py:148` | C1 |
| `homing:homing_move_begin` | 回零移动开始 | `homing_state` | `klippy/extras/homing.py:210` | C1 |
| `homing:homing_move_end` | 回零移动结束 | `homing_state` | `klippy/extras/homing.py:234` | C1 |
| `stepper:sync_mcu_position` | stepper 位置同步 | `stepper` | `klippy/stepper.py:56` | C1 |
| `stepper:set_dir_inverted` | 方向反转设置 | `stepper` | `klippy/stepper.py:153` | C1 |
| `dual_carriage:update_kinematics` | IDEx 双滑车运动学更新 | — | `klippy/kinematics/idex_modes.py:383` | C1 |

> 全部依赖 C1（toolhead + kinematics + homing），回零协议未实现前这些事件无消费者。

### idle_timeout 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `idle_timeout:ready` | idle_timeout 模块就绪 | 无 | `klippy/extras/idle_timeout.py:44` | — |
| `idle_timeout:idle` | 进入空闲状态 | 无 | `klippy/extras/idle_timeout.py:57` | — |
| `idle_timeout:printing` | 开始打印（恢复活动） | 无 | `klippy/extras/idle_timeout.py:95` | — |

> 需 idle_timeout 对象（`[idle_timeout]`），目前未实现。

### 工具头事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `toolhead:manual_move` | 手动移动前 | `positions, speed` | `klippy/toolhead.py:390` | C1 |
| `toolhead:set_position` | 设置位置（G92 等） | `positions, e` | `klippy/toolhead.py:416` | C1 |
| `toolhead:sync_print_time` | print_time 更新 | `print_time` | `klippy/toolhead.py:446` | C1 |
| `toolhead:update_extra_axes` | 额外轴位置更新 | `positions` | `klippy/toolhead.py:455` | C1 |

> 全部依赖 C1（toolhead），无 toolhead 则无消费者。

### gcode 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `gcode:command_error` | gcode 命令错误 | 无 | `klippy/gcode.py:226` | ✅ 已触发（`process_line`） |
| `gcode:debuginput_exit` | debuginput EOF | 无 | `klippy/gcode.py:433` | **暂缓 `[~]`**（随 GCodeIO；需 `send_event` 返回值） |
| `gcode:request_restart` | 请求重启 | `print_time` | `klippy/gcode.py:358` | C1（需 toolhead 的 print time） |

> `gcode:command_error` 已接（handler 的 `CommandError` 触发，panic 不触发）；`gcode:request_restart`
> 的声明已带 `print_time` 载荷，触发点等 C1；`gcode:debuginput_exit` 随 `GCodeIO` **暂缓 `[~]`**
> （不做 OctoPrint 串口仿真），将来做时还要先让 `send_event` 收集 handler 返回值（上游 `all(...)`）。

### 工具/传感器事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `probe:update_results` | probe 测量完成 | `results` | `klippy/extras/probe.py:200` | endstop |
| `extruder:activate_extruder` | 切换 active extruder | `extruder` | `klippy/kinematics/extruder.py:25` | C1 |
| `stepper_enable:motor_off` | stepper 电机关闭 | `stepper_enable` | `klippy/extras/stepper_enable.py:120` | C1 |
| `virtual_sdcard:reset_file` | VSD 文件重置 | 无 | `klippy/extras/virtual_sdcard.py:151` | sdcard |
| `load_cell:calibrate` | 称重传感器校准 | 无 | `klippy/extras/load_cell.py:397` | ADC |
| `load_cell:tare` | 称重传感器归零 | 无 | `klippy/extras/load_cell.py:404` | ADC |

> 依赖各自模块（endstop、sdcard、ADC 等），不阻塞运动。

### 显示/菜单事件（menu.py 内部）

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `menu:`（空名） | 菜单初始化 | `menu` | `klippy/extras/display/menu.py:346` | display |
| `menu:populate` | 菜单填充 | `menu` | `klippy/extras/display/menu.py:754` | display |
| `menu:init` | 菜单初始化 | `menu` | `klippy/extras/display/menu.py:722` | display |
| `menu:begin` | 菜单开始 | `menu` | `klippy/extras/display/menu.py:712` | display |
| `menu:exit` | 菜单退出 | `menu` | `klippy/extras/display/menu.py:913` | display |

> 依赖 display/menu 模块，优先级最低。

### 依赖关系总结

```
事件总线（KlippyEvent，已就绪）
├── klippy:* 生命周期事件（8个）—— 已触发（notify_mcu_error 待接入）
├── stepper:* —— 依赖 C1
├── homing:* —— 依赖 C1
├── toolhead:* —— 依赖 C1
├── idle_timeout:* —— 依赖 idle_timeout 对象
├── gcode:* —— 依赖 G1b
├── probe:* —— 依赖 endstop
├── extruder:* —— 依赖 C1
├── stepper_enable:* —— 依赖 C1
├── virtual_sdcard:* —— 依赖 sdcard
├── load_cell:* —— 依赖 ADC
└── menu:* —— 依赖 display
```

## 已完成（留档）

细节在各模块文档里；这里每条只留一行索引，最近完成的在前。

- **G-Code 框架大部（FW4）**：参数访问器补齐（通用 `get` + `minval`/`maxval`/`above`/`below`、
      `get_int_bounded`、`get_float_bounded`）、`get_command_parameters` / `get_raw_command_parameters`
      （剥行号与校验和）、`create_gcode_command`、`run_script_from_command`；`gcode:command_error`
      在 `process_line` 接上（panic 不触发）；`ack` 改为一次性（`Cell`）并按 `need_ack` 决定错误
      是否中止脚本；`gcode:request_restart` 声明补 `print_time` 载荷，生成枚举去掉 `Eq`。
      `GCodeIO`（伪 tty / OctoPrint 串口仿真）**暂缓 `[~]`**，随 C1 的 `request_restart` 触发
      另行跟进；见 [FW4 笔记](docs/work-log/2026-09-21-fw4-notes.md)。

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
      发送任务遇 barrier 立即发走并回报；`TestDevice::recorder()` 让 block 边界可断言
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

## 证据索引（上游，供回头分析时查）

只列**还没做完**的条目要用的位置；已完成项的参考见各模块文档。

| 主题 | 位置 |
|---|---|
| Printer 生命周期、状态、事件 | `klippy/klippy.py:25-236` |
| 事件总线（开放名字 + 参数） | `klippy/klippy.py:224-227` |
| notify / analyze shutdown 的载荷 | `klippy/klippy.py:144-151`、`:216-220` |
| 对象注册表（add / lookup / load） | `klippy/klippy.py:70-113` |
| 主循环与重启、退出码 | `klippy/klippy.py:355-370` |
| `emergency_stop` / `register_remote_method` / mux | `klippy/webhooks.py:319-340` |
| gcode 调度器（命令表 / `run_script` / 输出） | `klippy/gcode.py:105-388` |
| `GCodeIO`（伪 tty / 文件输入、`ack` 协议） | `klippy/gcode.py:390-494` |
| `output_pin`（`SET_PIN` / `GCodeRequestQueue` / 模板） | `klippy/extras/output_pin.py:13-269` |
| section 校验用注册表 | `klippy/configfile.py:425-445` |
| mcu 作为 printer object、它的 status | `klippy/mcu.py:1147-1170`、`:1235`、`:938-975` |
| stats 累计与 shutdown 处理 | `klippy/mcu.py:801-802`、`:883`、`:912`、`:974-975` |
| 固件停机/重启事件 | `src/sched.c:310` `:318` `:351`、`klippy/mcu.py:813-835` `:880-881` |
| `config_reset` 与 restart helper | `src/basecmd.c:262-272`、`klippy/mcu.py:756-770` |
| reactor 定时器 / 回调 / 时钟 | `klippy/reactor.py:111` `:145` `:187` |
| reactor latency 钩子 | `klippy/reactor.py:316`、`klippy/extras/garbage_collection.py:20` |
| kinematics 的装载与接缝 | `klippy/toolhead.py:235-252`、`:389` `:400` `:482` `:507` `:522` |
| 各 kinematics 的差异 | `klippy/kinematics/*.py`（`home` / `check_move` / `calc_position` / `get_status`） |
| 通用回零驱动 | `klippy/extras/homing.py:165-300` |
| IDEX / 双滑车 | `klippy/kinematics/idex_modes.py`、`klippy/kinematics/cartesian.py:19-30` |
| step 生成层的运动学 | `klippy/kinematics/kinematic_stepper.py`、`rail.setup_itersolve(...)` |
| 惰性装载 | `klippy/extras/adc_temperature.py:51` `load_object(config, 'query_adc')` |
| SPI / I2C 总线 | `klippy/extras/bus.py:9-336`、`src/spicmds.c`、`src/i2ccmds.c` |
| endstop / trsync 触发 | `klippy/mcu.py:155-407`、`src/endstop.c:72-120`、`src/trsync.c` |
| 加热器框架与温度传感器 | `klippy/extras/heaters.py`、`thermistor.py`、`temperature_sensor.py` |
| G-Code 宏 / 变量 / 定时 | `klippy/extras/gcode_macro.py:41`、`save_variables.py`、`delayed_gcode.py` |
| 打印流程与 SD 卡 | `klippy/extras/virtual_sdcard.py`、`print_stats.py`、`pause_resume.py:27-31` |
| TMC 驱动与 UART | `klippy/extras/tmc.py`、`tmc_uart.py`、`src/tmcuart.c` |
| 块状传感器与端点 | `klippy/extras/bulk_sensor.py:100`、`load_cell.py:55`、`src/sensor_bulk.c` |
| LCD 显示与菜单 | `klippy/extras/display/display.py`、`menu.py:346,712,722,754,913` |
| 主机统计与 MCU 错误详情 | `klippy/extras/statistics.py`、`error_mcu.py` |
| **全量模块对照** | [`docs/work-log/2026-09-21-upstream-coverage-audit.md`](docs/work-log/2026-09-21-upstream-coverage-audit.md) |
