# TODO — 主机（klippy）侧的待办与未决问题

本文件只记**打算做什么**和**还没定的事**，不重复已经定下的设计（那些写在各模块文档和
`docs/klippy/developer-manual/` 里）。括号里的 `klippy/xxx.py:NN` 指上游参考实现
（`third_party/klipper/`），用来在动手前核对行为。已完成的条目**从正文彻底删除**（规则见下节；历史在 git 与 `docs/work-log/`，
**大项索引**见文末「已完成（留档）」，细项不留）。
上游**全部功能点**的逐项对照（含判为「不适用」的 Python 专属项）已于 2026-09-21
盘点完毕、结论并入本文件（覆盖审计已完结清理）；本文件只放
**要动手的事**与**还没定的事**。

## 更新规则（本文件怎么维护）

1. **细节不落两处**：条目若已在 `docs/work-log/` 有动工前调查（notes）并按其实现，实现细节、
   步骤、拍板点**只写在那边**（或对应笔记/提交信息），本文件的原条目压缩成**一行引用**：
   `[主题](docs/work-log/<笔记>.md) — 一句话范围 + 依赖 + 状态`。这样实现侧改笔记即可，
   不必两边同步维护，也不会让 TODO 与笔记互相漂移。
2. **完成即彻底删除**：条目做完后从正文**删除**，不保留打勾行（历史在 git；**大项索引留在文末「已完成（留档）」**，细项不留）删除前确认三件事已随同一提交落地：受影响手册已同步、
   回归/守卫数字已实测重核、旧断言（「尚未实现/待办」类）已一并清掉——即 AGENTS「改完必须同步
   手册」与三条硬要求的自查都做完了，才允许删。
3. **归档只留大项**：`docs/work-log/` 的「已完成条目归档」只保留**子系统/框架队列/里程碑级**的
   大项（框架队列 FW*、MCU 配置构建层 F1、pins F2、G-Code 调度器 G1、语料框架与首个转绿、
   toolhead/kinematics、runtime 拆分、重启循环这类能独立命名的能力）；收尾、单命令/单端点/单验证、
   文案对齐、小修等**细项不进归档**，随「完成即删除」直接消失（需要时查 git 历史）。
4. 数字与状态的纪律沿用 AGENTS 硬要求：本节不重复，见仓库根 `AGENTS.md`；现行失败统计以本文件
   「当前失败原因统计」与 `regression-tests.md` 为唯一来源（快照归档不更新）。

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
  `reset_for_restart()` → 从磁盘重读配置 → 再 `bring_up`，端点与 `--tui` 的 in-process server
  全程有效；不换 printer、不重建 Api/Server。

## 待办

依赖列的是**工具性前置**，不是自然顺序。下表是索引，逐条细节在后面的小节里；
框架队列（FW1–FW9）已完成，索引见文末「已完成（留档）」；本表不再区分级别。
H1–H12 是上游 extras 里按域归并的消费者（2026-09-21 全量盘点的结果；其中若干已落地，
逐条见各节；覆盖审计已完结清理）。

**核心与架构**

| # | 事项 | 依赖 |
|---|---|---|
| C1 | 运动层收尾：轴/stepper 抽象 + 多轴（C1a）、extruder 运动（C1b）、运动学族（C1c：delta/polar/generic_cartesian/rotary_delta/winch/deltesian 全落地）、print-time 回调（C1d）——**✅ 收官** | — |
| C2 | 配置装载：框架部分 ✅（FW1，含 choice/range 文案与 `deprecate` 警告）；autosave/`SAVE_CONFIG` 写回 ✅（2026-10-02，详见下） | — |
| D1 | 主机层 start args / rollover / `--logfile` ✅（FW8）；剩余：`debuginput`/`debugoutput` 的命令行接线与每 MCU 字典路径 | — |

**MCU 资源与总线**

| # | 事项 | 依赖 |
|---|---|---|
| F6 | SPI 总线剩余：`spi_transfer_with_preface` / `setup_shutdown_msg` | F1、F2 |
| F8 | endstop / trsync ✅（FW6）；测试侧「响应器式多实例假 MCU」✅（2026-10-03，`d14ce6a`） | F1、F2、C1 |
| F9 | 固件资源剩余：buttons / trigger_analog / initial_pins / sdcard / sensor_bulk / lcd / neopixel / tmcuart 等（已接：`cmd/thermocouple.rs` + `spi_temperature`；pulse_counter 批 #8 落地） | F1–F7 |

**G-Code 与端点**

| # | 事项 | 依赖 |
|---|---|---|
| G1b | gcode 调度器与上游的行为差异（`get_mutex` 等价物等；`GCodeIO` 暂缓 `[~]`；参数访问器与 `M115`/`Coord`/`request_restart` 已完成并归档） | C1 |
| G2b | 用 GCODE 控制 GPIO：数字/PWM 驱动与 `SET_PIN` **已落地**；「随打印时间生效的请求队列」（上游 `GCodeRequestQueue`）**已移植并接入 `output_pin`**；`pwm_cycle_time` 的 `SET_PIN` 已通过 `register_lookahead_callback` + `min_schedule_time` 自间隔调度（2026-10-03，`a3e1d4c`）；余项＝`output_pin` 的 `static_value`/`template`，以及 `heaters` 切到同一队列（`fan`/`servo`/`pwm_tool` 2026-10-03 已切换） | —（C1 已收官，不再是前置） |
| G4 | 运动命令（G0/G1/G28…） | G1、C1 |
| B4 | 其余端点（`*/dump_*` / …；estop、remote method、`pause_resume/*` 与 `bed_mesh/dump_mesh` 已落地） | G3、H4、H9 |

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
| H10 | 运动相关 extras（gcode_arcs / force_move 剩余 / manual_stepper / idle_timeout / motion_report / …；gcode_move、stepper_enable、motion_queuing 已落地） | C1 |
| H11 | 主机运行时与调试（statistics / canbus_ids / canbus_stats；error_mcu 已落地） | — |
| H12 | 核心工具补齐（mathutil / util 反射 / clocksync / pins 消费侧） | C1 |

**工具与文档**

| # | 事项 | 依赖 |
|---|---|---|
| S1 | 压力测试工具（`klipperx stress`）剩余：stepper 资源、别名解析、端到端测试 | C1 |
| E2 | `python_path` 的取消 | 外部项目 |

> 判为**不适用**、不进待办的上游模块：`garbage_collection.py`（Python GC 调优）、
> `aio_executor.py`（Python 线程池）、`parsedump.py`（离线开发工具）、
> `debugcmds.c`（固件调试口）。理由见括注：都是 Python 侧调优或离线/调试工具，与本主机无关。

### G2b 用 GCODE 控制 GPIO（现状与剩余）

数字与 PWM 两条链都已打通（见文末索引）：`[output_pin <name>]`
（`extras/output_pin.rs`）读 `pin` / `value` / `shutdown_value`（PWM 另加 `pwm` /
`cycle_time` / `hardware_pwm`），经 `PrinterPins::setup_digital_out` / `setup_pwm`
（`pins.rs`）建出 `McuDigitalOut` / `McuPwm`，并注册 mux 命令
`SET_PIN PIN=<name> VALUE=<0..1>`；运行时经 `GCodeRequestQueue` 按打印时间生效（`output_pin`/`fan`/`pwm_tool`/`servo` 已接线，`heaters` 侧无队列化消费者；两条兕底见下）。剩下的是：

- [x] **与运动 / 打印时间同步的 `SET_PIN`**（上游 `GCodeRequestQueue`，
      `klippy/extras/output_pin.py:13-85` `:249-269`）：**已落地**——队列本体
      `extras/gcode_request_queue.rs`（覆盖压缩、`next_min_flush_time` 按
      `min_schedule_time` 对齐、`discard`/`reschedule`/`repeat`、sink 回调在锁外、12 测），
      接线在 `output_pin`：`cmd_set_pin` 经 `register_lookahead_callback` 把请求钉到
      前瞻时刻，flush 回调由 `MotionQueuing::generate` 消化，digital 走
      `queue_digital_out(clock)`、软 PWM 先 `next_aligned_clock` 再 `set_pwm`；
      **两条兜底**——没有 `[printer]`（lookup 不到 `toolhead`）或资源不可调度
      （`min_schedule_time()` 为 `None`）时保留立即路径（`update_digital_out`/`update_pwm`）。
      **余项**：`heaters` 无队列化消费者待切（`fan` `d0eb7ce`、`servo`/`pwm_tool` `cf9c884` 已于 2026-10-03 切换）。
- [ ] **`output_pin` 的 `static_value` / `TEMPLATE` + `template_evaluator`**（display 模板，
      `output_pin.py:88-170`）——与开关 GPIO 本身无关；`display_template` 机制已在（工厂表里有
      `display_template`），缺的是把它接到 `SET_PIN TEMPLATE=`。**语料 0 处使用**，可按需再补
      （`scale` 已随 T8 落地并归档）。

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

- [ ] **stepper 资源（C1 已收官，本项待改）**：C1 的 `PrinterStepper`/资源层已落地，但压力工具
      （`src/stress.rs`）仍把 `invert_step` / `step_pulse_ticks` 硬编码为 0，也没读
      `[stepper_*]` 的 `microsteps` / `enable_pin`（`stress.rs:787-788`）；待改成复用真资源，
      之后才谈「压力测试与生产同参」。
- [ ] **`[board_pins]` 别名**：现在只解析引脚名本身，别名未展开（`pins.rs` 已有解析器）。
- [ ] **端到端测试**：可照 `identify` 的 `chunked_mappings` 脚本化 identify + config +
      `queue_step`，用 `TestDevice` 覆盖一次加压（及 `ResetRequired` 路径）；`--task comm` 同理。
      目前只测了段计算、引脚解析与命令编码。

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
      tty 与 debuginput 共用同一套 `_process_data`，应一起做。
- [~] **`gcode:debuginput_exit` 触发（随 `GCodeIO` 暂缓）**：上游 `_do_debuginput_exit`
      轮询 `all(send_event('gcode:debuginput_exit'))`（`:432-435`），依赖 handler 的返回值；
      本仓库 `Printer::send_event` 丢弃返回值（上游 `klippy/klippy.py:226-227` 是
      `return [cb(...)]`）。要与 `GCodeIO` 一起做（见上一条）。
**可独立补的小行为差异**

（`request_restart` 的停机前动作、`Coord`、`M115` 版本号来源三项已完成并归档，
分别见 `gcode.rs:1186-1193`、`mathutil.rs:36`、`gcode.rs:747-767`。）
- [x] **`get_mutex` 等价物**（原先未列）：上游 `gcode.get_mutex()`（`klippy/gcode.py:242-243`）被
      `bed_mesh`（`:307`）与 `idle_timeout`（`:70` `:90`）用来判断「是否有脚本在跑」。
      **结论：不做**——C1 与那两个模块都已落地：`bed_mesh` 不需要该判据（仓内无对应代码路径），
      `idle_timeout` 明示删掉它（模块文档「What is not here」，见 `idle_timeout.rs:57-58`），
      至今没有消费者需要。

### G4 运动命令（G0/G1/G28/G92/M114…）

- [ ] **G4-2 外围**：`GET_POSITION`（要 `kin.get_steppers()` + `calc_position` + MCU 位置）、
      extra-axes 的 `axis_map`（`Coord` 目前固定 4 轴）、`toolhead:manual_move` /
      `toolhead:update_extra_axes` / `extruder:activate_extruder` 的发送方（等它们的 API）、
      `move_transform`（等 `bed_mesh`）。

### B4 其余端点（机制部分 FW9）

`api-reference.md` 有、`endpoints/mod.rs` 的表里标「not started」的其余部分，各自等它读的
对象先存在：

- [ ] `bed_mesh/dump_mesh` 与其余 `*/dump_*`（mux 机制已就绪，实例随各自 extras 落地）。

### F MCU 基础资源（F6、F8、F9）

上游把这些叫 printer objects 下面的「资源」：主机用一个 **oid** 和一个 **pin 描述**
建立资源对象，把 `config_*` 命令攒起来，在 `finalize_config` 之前算一个 CRC 一次性下发，
之后用 `queue_*` / `set_*` / `*_transfer` 命令驱动。命令层（`allocate_oids` / `get_config` /
`finalize_config` / `get_uptime` / `emergency_stop` / `get_clock`）已就位，配置构建层（F1）
与 pin 解析（F2）也已完成，数字输出、PWM（F4）与 ADC（F5）三个 `config_*` 资源已落地
（见文末索引），SPI/I2C 总线（F6/F7）也已落地（细节已归档），剩下的缺口是 endstop/trsync（F8，另有
一条测试侧待办）与其余固件资源（F9）；`MCU_bus_digital_out`（F3）能力已具备，不另做包装。

#### F6 SPI 总线

上游 `MCU_SPI`（`klippy/extras/bus.py:42-155`）：

- [ ] `spi_transfer_with_preface` 与 `setup_shutdown_msg`：`ConfigSpiShutdown` 命令
      已定义，但资源/消费者未接（设备需要在 shutdown 时发消息时才用得上）。

已落地：`cmd/spi.rs`（命令层，`%*s` 走二进制 `ArgType::Bytes`）、
`mcu/resource/spi.rs`（`McuSpi` 资源 + `SpiMode`，片选由固件驱动）、
`extras/spi_device.rs`（`[spi_device <name>]` 构造器 + `SPI_TRANSFER` / `SPI_SEND`
调试命令；与 `[i2c_device]` 共用 `extras/bus_debug.rs` 的 hex/异步桥）。

真机验证（STM32F103 + W25 flash，CS=PA15，SPI1 重映射 PB3/PB4/PB5）：硬件 `spi1a`
与软件 bit-bang 两条路都读出 JEDEC ID `ef 30 13`、状态寄存器 `0x00` 与地址 0x00 的
数据。

#### F8 endstop / trsync（与 C1 共享，框架 FW6a–FW6e 已落地）

##### F8b `Mcu` ↔ 资源的强引用环（阻塞回归）

#### F9 其他输入与外设资源

建立在 F1–F6 之上，各自一个 `config_*` + 查询/事件。这一节只列**固件侧资源**；
在它之上建的**宿主 extras 消费者**（buttons / pulse_counter / neopixel / sdcard / lcd /
sensor_bulk / 各类传感器）按域归到 H5–H8，两边互为前置：

- [ ] `buttons`（`src/buttons.c`，`config_buttons` / `buttons_add` / `buttons_query` /
      `buttons_ack`）—— 暂停/恢复按钮、耗材检测。
- [ ] `neopixel` / `dotstar` / `led`、`tmcuart`、`sdcard` / `sdio`、
      `lcd_hd44780` / `lcd_st7920`、`sensor_bulk`（批量传感器上报）与各类 SPI/I2C 传感器
      （`sensor_adxl345` / `sensor_lis2dw` / …）。
- 这些是 extras，不阻塞运动；等 F1–F6 完成、真有对应 section 时再逐个接。消费者见
      H5（TMC/tmcuart）、H6（sensor_bulk/加速度计）、H7（buttons/pulse_counter/trigger_analog）、
      H8（lcd）。

### C1 运动层收尾 —— ✅ 收官

- **C1a** 轴/stepper 抽象 + 多轴（`[stepper_z1]`…、`Rail`/`LookupMultiRail`）✅
- **C1b** extruder 运动（`extra_axes`/junction 接线）✅
- **C1c** 运动学族：`delta`（批 #5）、`polar`（批 #5）、`generic_cartesian`（批 #12）、
  `rotary_delta`（批 #40）、`winch`、`deltesian` **全部落地**；`mathutil` 的
  `trilateration`/`gaussian_solve` 已到位。
- **C1d** print-time 回调（`ToolHead::register_lookahead_callback` +
  `motion_queuing.register_flush_callback`）✅（`toolhead:sync_print_time` **事件本身仍无发送点**，
  它与本项的「回调」是两件事——`idle_timeout` 因此改为观察 `print_time` 前进）。

**历史归档**：C1 的动工前调查（上游轴/stepper/rail/extruder 模型、七个拍板点、C1a–C1d 拆分）随收官
从工作记录目录清理；需要时 `git log --diff-filter=D -- docs/work-log/` 找回。

### C2 配置装载收尾（框架 FW1）

- [x] **autosave / `SAVE_CONFIG` 写回侧**（2026-10-02）：`#*#` 区块读取合并批 #5 已落地；回写侧由 `%s 待办无` —— `SAVE_CONFIG` 命令在本批完成：`Config` 保留块 fileconfig（`autosave_block`）、`PrinterConfig` 在 `set`/`remove_section` 同步维护它并暴露给写回侧、`config/save_config.rs` 把块序列化回文件（`#*# ` 前缀、正文去重让块获胜、带时间戳备份 + temp/rename 换主文件）、装载器注册 `SAVE_CONFIG` 并 `request_restart` 重启重读。消费者（`PID_CALIBRATE`、`probe_eddy_current`、`bed_tilt` 等）依赖它做完。语料 0 个 `#*#` 块，不引入回归。

### D1 主机层 start args / rollover / 日志（框架 FW8）

- [ ] **`StartArgs` 的剩余接线**：结构体已带 `apiserver`（宿主启动时填，`src/klippy.rs:323`）、
      `start_reason`、`debug_input`/`debug_output`、`device`、`linux_version`；缺的是
      `--debuginput`/`--debugoutput` 的命令行解析（`StartArgs::collect` 里仍是 `None`）与
      每个 MCU 的字典路径。`software_version` 已由宿主 `set_start_args` 注入
      （`src/klippy.rs:324`）并被 `info` 与 `M115` 读取——接线已完成并归档。
- [ ] **rollover 的 `log_config`**：上游每次 `_read_config` 都把整份配置写进 rollover
      （`configfile.py:482-487`、`klippy.py:118`），本仓只有 `versions` 块；重启重读上线后，
      「这次重载的是哪份配置」在日志里看不到。
- [ ] **`StartArgs.start_reason` 已死**：上游主循环每轮 `start_args['start_reason'] = res`
      （`klippy.py:368`）、`mcu.py:682`/`:1060` 从字典读；本仓 MCU 改读
      `Printer::start_reason()`（`reset_for_restart` 每轮更新，语义一致），
      `StartArgs.start_reason` 停在 `"startup"` 且无人读。要么让它随重启更新（对外形状与上游一致），
      要么删掉该字段。

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

### T 上游 `.test` 语料推进 —— ✅ 收官（2026-09-25）

框架：`src/core/klippy/upstream.rs`（字典驱动应答机 + 按 `CONFIG` 拆分的运行）与
`crates/test-support/build.rs`（按架构编字典）。

**结果**：`KLIPPERX_UPSTREAM_ALL=1` **237 条通过 / 0 条失败**，`IGNORED` **清空**（默认构建下另有 2 条
因未编 `pru` 字典跳过，见下方备注）；默认套件（含全部
语料 237 例）约 65s 正常退出。逐批转绿过程见各批提交信息与
[回归测试手册](docs/klippy/developer-manual/regression-tests.md)。

**推进口径（留作方法）**：

- 「首次失败原因」分组只用于定位，**不是工作队列**：`load_config` 遇首个未知 section 即停，修好一个
  缺口只会让运行前进到下一个，总数可能不变，**各组收益不可加**。
- **验收标准**是「对应 `.test` 从 `IGNORED` 移除后通过」，不是「某个错误不再出现」。
- 每例一个独立 runtime + 有界收尾（对齐上游「每个用例一个 `klippy.py` 进程」的隔离）；重型套件
  （`--lib` 全量、`ALL=1`）由 main 在合入点单进程跑，跑完按显式 PID 清残留进程。

**历史归档**：2026-09-22 的失败原因分析（239 次运行按首次失败归类的 T3–T10 工单、完整失败日志与
「计数不是工作量」的复盘）随收官从工作记录目录清理；需要时
`git log --diff-filter=D -- docs/work-log/` 找回。

- 备注：`printers.test` 有 2 条运行声明 `DICTIONARY pru.dict host=linuxprocess.dict`，默认不构建
  `pru`；要跑需 `KLIPPERX_ARCHES=…,pru`（需 `pru-gcc`）。

## 上游 extras 覆盖盘点

上游 133 个 extras（顶层 `*.py` 不含 `__init__.py`、不含 `display/` 子目录；2026-09-23 实数复核
✓）里，本仓库已落地的有：`board_pins` ✅、`output_pin` ◐、`bus`（SPI/I2C 框架）✅、`fan`、
`gcode_move`、`stepper_enable`、`query_endstops`、`error_mcu`、`static_digital_output`、`ds18b20`、
温度传感器族（`temperature_sensor` / `thermistor` / `adc_temperature` / `spi_temperature` /
`temperature_combined` / `temperature_mcu`），以及 `heaters` / `heater_bed` / `heater_generic`
（控制环与住户已落地，等待 / 校准面仍缺，见 H1）；其余按域归并成 H1–H12。逐模块的完整对照表
（含固件命令模块、端点、判为不适用者）随覆盖审计收口清理；落点就是下文 H1–H12
与 F/G/B 各节。

### H1 加热与温度

- [x] **`M105` / gcode_id 表**（heaters 域）：`TEMPERATURE_WAIT` 已落地（2026-10-03，`b5da84e`，
      10 测）；`M105` gcode-id 表已接（2026-10-03，`14f5055`）：`register_sensor` 存 `gcode_id→sensor` 表（`BTreeMap` 排序），
      `M105` 输出 `<id>:%.1f /%.1f` 列表（空表/未 ready 回 `T:0`），`TEMPERATURE_WAIT` 等待循环每轮回真报表（5 新测）。
- [ ] 传感器**剩余**：仅 `EddyCalibrationTool`（`PROBE_EDDY_CURRENT_CALIBRATE`，`note_z_calibration_*` 的唯一调用方）未移植；`temperature_probe.py` A+B+C 已落（`f6ea201` / `dfaf418` / `704ee0d`；前置 `TEMPERATURE_WAIT` 已随 `b5da84e` 落地）。装载序均已修复：`thermistor` 批 #22、`adc_temperature` `372cd93`（都声明 `phase = early`）。已落地并归档：`temperature_host`（2026-10-03，`87d7b01`）/ `temperature_sensor` / `thermistor` / `adc_temperature` / `spi_temperature`（MAX6675/31855/31856/31865） / `temperature_combined` / `temperature_mcu`（T7）；`temperature_fan` 无需再排（2026-10-03 核对：选项/双控制环/命令/get_status 逐项与上游一致，11 单测 + 3 条语料用例全绿；速度抑制窗未移植属低危可选）。
- 依赖 F4（PWM）、F5（ADC）、F6（SPI 温度）；C1 已收官。

### H2 风扇与通用输出

- [ ] **引用（拆分、拍板点与依赖以笔记为准）**：[H2 动工前调查](docs/work-log/2026-09-23-h2-notes.md)
  ——上游 21 个文件的依赖盘点、H2-1…H2-7 拆分与四个拍板点。**已落地**：`fan`/`fan_generic`/
  `heater_fan`/`controller_fan`（含 heater 注册表）、`tachometer_pin`→`pulse_counter`、
  `pwm_tool`/`pwm_cycle_time`/`servo` 模块本体、`multi_pin`（批 #26）、`sx1509`（批 #41）；
  **余项**：`static_pwm_clock.py`（语料 1 处）、`replicape.py`（语料 1 处，另见本文件「特定板/芯片」）、
  `duplicate_pin_override.py`（语料 0 用）；`pwm_tool` 队列化已完成（2026-10-03，`cf9c884`，同 **G2b**）。

### H3 G-Code 宏与脚本

- [ ] `gcode_macro.py`：段与宏注册**已落地**（2026-09-24 集成批 #1，语料绿）；剩余 = `rename_existing` 连接期换名，以及读
      `printer.objects` 的反射式能力（**Q5**）。**U-A7b 已归档**（2026-09-24 批 #4 起的「受控子集引擎」阶段：`exclude_object.test`+`dual_carriage.test` 双翻转、guard 归零，`{% set %}` 批 #9、过滤器参数与 `default`/`float` 批 #17、列表字面量与 `|min`/`|max` 批 #24 相继落地）。**2026-09-29 引擎换为 minijinja 2.24 适配层**（`custom_syntax` 单花括号定界符、Strict undefined、装载期编译与求值分两段）：原子集外的 `namespace()`、关键字实参、`{% block %}`、`|float(默认)` 由此接上（语料 160 绿）。仍缺：`%` 字符串格式化（minijinja 的 `%` 是数值取模）、模板内方法调用（`Coord`/`PrinterView` 未实现 `call_method`；「方法调用白名单」待办消解——不装 `unknown_method_callback` 即天然拒绝）；三元 `x if c else y` 由 minijinja 原生支持但本仓未单列测试。与 Jinja2 的已知差异（`%`/`//` 欧几里得取余、Strict 下缺键在打印/迭代/判真时报错、部分 detail 措辞）见 `template.rs` 模块文档。iqex/itex 的模板阻塞已消，首因前移到 `dual_carriage` 的 `primary_carriage`。
- [ ] `save_variables.py`（`SAVE_VARIABLE` / `[variables]`）。
- [ ] `delayed_gcode.py`（`[delayed_gcode]`）。
- [x] `respond.py`（`RESPOND` / `M118`，批 #29）。
- 前置：**G1b** 的 `create_gcode_command` 与参数访问器（宏类模块要构造 gcmd）。

### H4 打印流程与 SD 卡

- [x] `virtual_sdcard.py`：主机侧文件打印、`M20`–`M27`/`SDCARD_RESET_FILE`/`SDCARD_PRINT_FILE`、`work_handler` 回放、进度（**2026-10-02**；省略 `gcode.get_mutex().test()` 让出、`_handle_analyze_shutdown`/`_handle_debuginput_exit`/`stats`，`path` 不做 `expanduser`/`normpath`）。
- [x] `print_stats.py`（**2026-10-02**；省略 `_handle_activate_extruder`，事件未 fire）、`display_status.py`（`M73`/`M117`）。
- [x] `pause_resume.py` 的节、四条命令与 `pause_resume/*` 三个端点（批 #15 + 2026-10-03）。
- [ ] `exclude_object.py`（段+四命令落地，**2026-09-24 批 #4 随引擎转绿**，含排除区 E 补偿）、`sdcard_loop.py`（段已落地，`SDCARD_LOOP_*` 命令与文件回放未接）、`firmware_retraction.py`（G10/G11，**批 #32 已落地**）。
- 依赖 F9（固件 `sdiocmds.c` 的 sdcard 资源）、C1（`gcode_move` 的位置恢复）。

### H5 TMC 步进驱动

- [ ] `tmc.py` 公共框架（寄存器、StallGuard、`DUMP_TMC`/`SET_TMC_*`）。
- [ ] `tmc_uart.py`（固件 `src/tmcuart.c`）。
- [ ] SPI 型：`tmc5160.py`、`tmc2240.py`、`tmc2660.py`（`tmc2130.py` 批 #34 已落地）；UART/SPI 型：`tmc2208.py`、`tmc2209.py`、
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

- [ ] 固件按钮查询（`buttons.c`）/ `query_adc`——`gcode_button` 的**数字路径**与 `[buttons]` 已落地（批 #37）；`analog_range` 明确拒绝。
- [x] `pulse_counter.py`（批 #8：host 侧 `pulse_counter.rs` 落地并接通 `tachometer_pin`；`config_counter`/`query_counter` 按字典编码，真机验证仍待 T6）。
- [ ] `trigger_analog.py`（固件 `trigger_analog.c`）。
- [ ] 断料/线宽：`filament_switch_sensor.py`、`filament_motion_sensor.py`、
      `hall_filament_width_sensor.py`、`tsl1401cl_filament_width_sensor.py`。
- [ ] 固件 `initial_pins.c` 的初始引脚状态。
- [ ] 特定板/芯片：`samd_sercom.py`、`replicape.py`、`palette2.py`。
- 依赖 F3（GPIO）、F5（ADC）、F9。

### H8 LCD 显示与菜单

- [x] `display/display.py` 框架与 `hd44780.py`、`st7920.py`、`uc1701.py`（`ssd1306` 同文件，批 #7/#13 已落地，均 storage-only）。
- [ ] `hd44780_spi.py`、`sh1106`（`aip31068_spi` 批 #28 已落地）。
- [ ] 菜单：`display/menu.py`、`display/menu_keys.py`、`display.cfg`、`menu.cfg`；
      事件 `menu:*`。
- [ ] 固件 `lcd_hd44780.c` / `lcd_st7920.c`。
- 依赖 F9（固件侧 LCD）、G1b（`create_gcode_command`，菜单脚本要构造 gcmd）。

### H9 探测 / 调平 / 校准

- [ ] 探针：`probe.py` ◐（**已落地**：`[probe]` 段、`probe` 虚拟 chip、选项与偏移、会话采样、
      `QUERY_PROBE`/`PROBE`/`PROBE_ACCURACY`、`probe:update_results` 发送；**待做**：
      `PROBE_CALIBRATE`/`Z_OFFSET_APPLY_PROBE`（需 `manual_probe` + `configfile.set()`）、
      `ProbePointsHelper`（消费者 z_tilt/screws）、endstop wrapper 的 `z_offset`/`query_endstop` 覆盖——
      需先把 `PinChip::setup_endstop` 的返回类型接口化）、`probe_eddy_current.py`、`manual_probe.py`、`safe_z_home.py`、`endstop_phase.py`。
- [ ] **保真单元：bed_mesh 插值 + 调平应用**（排在 H9 模块闭包之后）：`LagrangeMesh`/`BicubicMesh` +
      `mesh_pps` → `mesh_matrix`、`get_z(x, y)`，以及把网格作用到 move（`gcode_move` 的 move-transform
      seam + `MoveSplitter` + `fade_*`）。素材：`abandoned/wip-main-leftovers` 的 `bed_mesh.rs`
      （含 `ZMesh`/lagrange/bicubic 与两个测试），按主线结构重做并补手册。
- [ ] **保真单元：探针精度**（排在 H9 模块闭包之后）：按触发步数反算位置与 `rest_time`
      （上游 `_calc_endstop_rate`）。前置已满足：模拟器步数模型与多实例假 MCU 已落地
      （Q10 + `d14ce6a`，两 MCU 端到端测试在案）。
- **端停位置的缺口（需接口化）**：上游 rail 会优先向 endstop 要位置（`mcu_endstop.get_position_endstop()`，
      探针 wrapper 返回 `z_offset`）；本仓 `position_endstop` 缺省退到 `position_min`，所以用
      `probe:z_virtual_endstop` 的配置虽然在解析上能过，但 Z 回零后的位置不等于上游——`z_virtual_endstop.test`
      的位置断言需要它。
- **已知偏离（探针精度）**：本仓回零与探针移动返回**指令位置**，未按上游 `StepperPosition.note_home_end`
      + `calc_toolhead_pos` 用触发步数反算；`home_start` 的 `rest_time` 也硬编码（上游 `_calc_endstop_rate`
      按 move 距离与步数计算）。两者都影响真实探针 Z 精度，属后续精度单元；模拟器语料不受影响。
- [ ] 调平：`bed_mesh.py` ◐（**已落地**：`[bed_mesh]` 段与全量选项、探测点生成、
      `BED_MESH_CALIBRATE` 逐点探测存格（按 Y 分行、行内 X 升序）、`BED_MESH_CLEAR`、
      `bed_mesh/dump_mesh` 端点（2026-10-03）；**待做**：插值网格
      （lagrange/bicubic、`mesh_pps`）、faulty 区域替换、fade 与 move 的 z 补偿、profile 命令）。
- [ ] 螺丝：`screws_tilt_adjust.py` ✅；`bed_screws.py` **段已落地**（2026-09-24 批 #1），`BED_SCREWS_ADJUST`/`ACCEPT`/`ADJUSTED`/`ABORT` 命令族未移植。
- [ ] 校准：`delta_calibrate.py` ✅（段+`DELTA_CALIBRATE`/`DELTA_ANALYZE` 落地 2026-09-24 批 #5，`delta_calibrate.test` 转绿）、`axis_twist_compensation.py` ✅（批 #36）、`skew_correction.py`、
      `z_thermal_adjust.py`、`tuning_tower.py`。
- [ ] 回零周边：`homing_override.py`（`homing_heaters.py` 批 #31 已落地）；事件 `probe:update_results` 已带载荷并触发（批 #36）
      （`homing:*` 四个已随 toolhead 落地产线触发，见事件清单）。
- 依赖 C1、F8（endstop/trsync）、H3（宏）、H12（`mathutil`）。

### H10 运动相关 extras

- [ ] `gcode_arcs.py`：**段已落地**（2026-09-24 批 #1），G2/G3 弧规划与平面命令仍未接；`manual_stepper.py`；`force_move.py` **部分**（只有
      `SET_KINEMATIC_POSITION`，`FORCE_MOVE`/`STEPPER_BUZZ` 未接）；`extruder_stepper.py` 段已落地（2026-09-24 批 #2，宿主 step 同步的 toolhead 缝仍缺）。
      （`stepper_enable.py` ✅ 已随 T2 落地并归档。）
- [x] `idle_timeout.py`（批 #21：节 + `SET_IDLE_TIMEOUT` + 三个事件带载荷）、
      `motion_report.py`（`dump_trapq`/`dump_stepper` 端点，见 **B4**）。
      （`motion_queuing.py` ✅ 已由 `motion/queuing.rs` 落地并归档。）
- 依赖 C1（toolhead/kinematics）；`gcode_move` 同时是 **G4** 的前置。

### H11 主机运行时与调试

- [ ] `statistics.py`：周期上报主机统计（CPU/内存；`event/stats.rs` 是 MCU 调度时序上报，不是它）。
- （`error_mcu.py` ✅ 已落地并归档：`extras/error_mcu.rs`，消费 `klippy:notify_mcu_error` /
  `klippy:analyze_shutdown`，由 `[mcu]` 工厂 ensure。）
- [ ] `canbus_ids.py` / `canbus_stats.py`：CAN 节点分配与状态（接 `[mcu]` 的 canbus 选项；
      当前 `[mcu]` 直接声明 id，见 `config/mcu.rs:265`）。
- 判为不适用：`garbage_collection.py`、`aio_executor.py`、`parsedump.py`（Python 侧调优/
  离线工具，与本主机无关；完整清单见「待办」表后的不适用段）。

### H12 核心工具补齐

- [ ] `util.py` 的反射与注册表 helper **剩余**：`get_heater` / `get_sensor`
      （前缀式 `lookup_objects`、`statuses`、`get_status -> serde_json::Value` 已随 FW2
      落地并归档，见 `printer.rs:622` `:654` `:676`；**Q5** 已定为只读反射）。
- [ ] `pins.py` 消费侧接口（`get_pin_type` 查询面）——重命名/别名与 `PinType` 已有
      （`pins.rs:78` `:533`），查询入口随 **H7** 等消费者再补。

## 未决问题

- [ ] **Q9 `wait_moves`（FW5d）打印节拍缺失**：上游在运动队列将满时会等待 MCU 追上（`wait_moves`；
      语料另有 `is_fileoutput` 虚拟时钟同款语义），本仓 `motion/toolhead.rs` 文档自述 **FW5d 未实现**
      （2026-09-27 C4 侦察发现）——G-Code 可瞬间把整段运动灌进 trapq，print horizon 相对
      `estimated_clock` 无界。后果分两层：① fake 环境最终以「`test:` 传输放行两道闸」解（探针实证
      闸会退化为墙钟串行；早先批过的 simulator 跳钟方案已撤销，见 `eeb0e79`）；② **真机**上 C4 的 min/req 闸会以「压队=背压」形态工作
      （放行速度由固件消费决定，上游同款），但宿主侧没有上游的节拍保护。**R8 真机观察点**：
      长 gcode 连灌时 print horizon 与压队深度。实现 wait_moves 属独立工单（上游正解），不阻塞 C4。

- [x] **Q10 模拟设备的步进时序模型（两会话固件链）——已完成（2026-09-27，`agents/feat-simulator-step-timing`）**：
      `SimulatorDevice` 落 per-oid `StepChain`（`config_stepper` 归零、`queue_step` 空闲首拍/忙延展/
      首拍过期三态、`reset_step_clock` 忙拒+重锚）、`timer_is_before` u32 回绕（含 2³¹ 假阳边界），
      过期/忙拒经字典 `static_string_id` 发 shutdown 帧并置 `get_config is_shutdown`；5 条单测（两会话
      红/绿对）+ 转红实证（剪过期判定恰红 2 条）。配套：宿主生成地上游 BGFLUSH 地平线（est+0.4/0.7 s
      出生垫——同修语料两类 `Timer too close`：管线迟打戳与半回绕越界）、假件时钟定为纯墙钟（早先
      跳钟方案撤销）、monitor 窗改由固件侧样本续命（`eddy.test`）。语料 `KLIPPERX_UPSTREAM_ALL=1`
      237/0、workspace 2052+ 全绿。**原任务书**（存档）：C5 案暴露的测试缺口——当时假设备（FrameMock /
      SimulatorDevice / 语料）都不维护 `next_step_time` 链、不做 `timer_is_before` 回绕，且语料每例全新
      实例，复现不了「同一假固件跨两次宿主会话」；方案即上述最小时序模型 + 两会话单测（修复前代码转红），
      原则：真机定案、假件固化。

- [~] **Q8 GCodeIO（伪 tty / OctoPrint 串口仿真）补不补**：**已定（2026-09-21）：暂不实现**，
      归档为将来可选项，等需要时再操作。纯 API 主机（Moonraker）不需要它；代价是
      `debuginput_exit`、`is_fileinput`/`error_exit`、`stats gcodein=`、`input_log`、`M112` 乱序
      一直缺。将来做时的前置见 G1b 的 `GCodeIO` 条目（首要是 reactor 的 fd 事件层）。

## 上游事件对照清单（事件总线已就绪，逐项注册处理器）

上游 `Printer` 维护 `event_handlers` 字典（`klippy/klippy.py:36`），通过
`register_event_handler(name, cb)` 注册、`send_event(name, *params)` 分发，共 35 个事件名。

`KlippyEvent` 已声明全部 35 个名字，`Printer::register_event_handler` / `send_event` 按名
注册与分发，处理器签名为 `Fn(&KlippyEvent)`。以下事件可按优先级逐个注册处理器；依赖
关系标注在「实现依赖」列中，`—` 表示仅依赖事件总线，其他依赖的模块已标记为其他 TODO 条目。
生命周期事件已触发，其余事件的触发点随对应模块落地（逐项状态 2026-09-23 复核，见各表）。

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
| `klippy:analyze_shutdown` | 进入 shutdown 后分析 | `msg: str, details: dict` | `klippy/klippy.py:216-220` | ✅ 已接入 |

> **说明**：两个事件由带载荷的变体承载（`KlippyEvent::KlippyNotifyMcuError` /
> `KlippyEvent::KlippyAnalyzeShutdown { msg, details }`）。`analyze_shutdown` 触发时
> `details` 在 MCU 停止路径带 `{mcu, reason, event_type}`；`notify_mcu_error` 已接入
> `bring_up` 中 MCU 连接失败路径；两者都由 `error_mcu` 模块（`extras/error_mcu.rs`）消费。

### MCU 相关事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `klippy:mcu_identify` | 每个 MCU identify 后 | 无 | `klippy/mcu.py:797,929,1001` | — |

> 已由 MCU 层事件 `Starting` 覆盖部分语义，但上游的 `klippy:mcu_identify` 是
> Printer 级事件，供 extras（probe、tmc、temperature_mcu 等）做初始化。

### 运动/回零事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `homing:home_rails_begin` | 回零开始 | `homing_state` | `klippy/extras/homing.py:80` | ✅ 已触发（`extras/toolhead.rs:910`） |
| `homing:home_rails_end` | 回零结束 | `homing_state` | `klippy/extras/homing.py:148` | ✅ 已触发（`extras/toolhead.rs:923`，带 `axes` 载荷） |
| `homing:homing_move_begin` | 回零移动开始 | `homing_state` | `klippy/extras/homing.py:210` | ✅ 已触发（`extras/toolhead.rs:974`） |
| `homing:homing_move_end` | 回零移动结束 | `homing_state` | `klippy/extras/homing.py:234` | ✅ 已触发（`extras/toolhead.rs:1008`） |
| `stepper:sync_mcu_position` | stepper 位置同步 | `stepper` | `klippy/stepper.py:56` | C1 |
| `stepper:set_dir_inverted` | 方向反转设置 | `stepper` | `klippy/stepper.py:153` | C1 |
| `dual_carriage:update_kinematics` | IDEx 双滑车运动学更新 | — | `klippy/kinematics/idex_modes.py:383` | C1 |

> `homing:*` 四个已随 toolhead 产线触发（`extras/toolhead.rs:910` `:923` `:974` `:1008`，
> `G28` 路径）；`stepper:sync_mcu_position`、`stepper:set_dir_inverted`、
> `dual_carriage:update_kinematics` 仍无触发点，分别随 stepper 位置同步、目录反转变更、
> C1c 的 IDEX/双滑车落地。

### idle_timeout 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `idle_timeout:ready` | idle_timeout 模块就绪 | `{print_time: f64}` | `klippy/extras/idle_timeout.py:44` | — |
| `idle_timeout:idle` | 进入空闲状态 | `{print_time: f64}` | `klippy/extras/idle_timeout.py:57` | — |
| `idle_timeout:printing` | 开始打印（恢复活动） | `{print_time: f64}` | `klippy/extras/idle_timeout.py:95` | — |

> 批 #21 已落地：`[idle_timeout]` 装载即注册，三个事件带 `{print_time: f64}` 载荷发出。

### 工具头事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `toolhead:manual_move` | 手动移动前 | `positions, speed` | `klippy/toolhead.py:390` | 尚无触发点（handler 已备，G4-2） |
| `toolhead:set_position` | 设置位置（G92 等） | `positions, e` | `klippy/toolhead.py:416` | ✅ 已触发（`extras/toolhead.rs:1091`，`SET_KINEMATIC_POSITION`） |
| `toolhead:sync_print_time` | print_time 更新 | `print_time` | `klippy/toolhead.py:446` | **尚无发送点**（C1d 的回调已落地，但该事件没有触发方；`idle_timeout` 改为观察 `print_time` 前进） |
| `toolhead:update_extra_axes` | 额外轴位置更新 | `positions` | `klippy/toolhead.py:455` | 尚无触发点（handler 已备，G4-2） |

> `toolhead:set_position` 已产线触发（`gcode_move` 重置链之一）；其余三个尚无触发点：
> `manual_move`/`update_extra_axes` 等 G4-2 的 API；`sync_print_time` 仍无发送点（C1d 已收官）。

### gcode 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `gcode:command_error` | gcode 命令错误 | 无 | `klippy/gcode.py:226` | ✅ 已触发（`process_line`） |
| `gcode:debuginput_exit` | debuginput EOF | 无 | `klippy/gcode.py:433` | **暂缓 `[~]`**（随 GCodeIO；需 `send_event` 返回值） |
| `gcode:request_restart` | 请求重启 | `print_time` | `klippy/gcode.py:358` | ✅ 已触发（`gcode.rs:1191`，`get_last_move_time` 取 print time） |

> `gcode:command_error` 已接（handler 的 `CommandError` 触发，panic 不触发）；`gcode:request_restart`
> 已在 `request_restart` 处理器里产线触发（`gcode.rs:1191`，先 `get_last_move_time` 再
> dwell/wait）；`gcode:debuginput_exit` 随 `GCodeIO` **暂缓 `[~]`**（不做 OctoPrint 串口仿真），
> 将来做时还要先让 `send_event` 收集 handler 返回值（上游 `all(...)`）。

### 工具/传感器事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `probe:update_results` | probe 测量完成 | `ProbeResultsHandle`（可原地改 Z，批 #36） | `klippy/extras/probe.py:200` | endstop |
| `extruder:activate_extruder` | 切换 active extruder | `extruder` | `klippy/kinematics/extruder.py:25` | 尚无发送方（handler 已备，`gcode_move.rs:357`） |
| `stepper_enable:motor_off` | stepper 电机关闭 | `stepper_enable` | `klippy/extras/stepper_enable.py:120` | ✅ 已触发（`stepper_enable.rs:338`） |
| `virtual_sdcard:reset_file` | VSD 文件重置 | 无 | `klippy/extras/virtual_sdcard.py:151` | sdcard |
| `load_cell:calibrate` | 称重传感器校准 | 无 | `klippy/extras/load_cell.py:397` | ADC |
| `load_cell:tare` | 称重传感器归零 | 无 | `klippy/extras/load_cell.py:404` | ADC |

> 依赖各自模块（endstop、sdcard、ADC 等），不阻塞运动。

### 显示/菜单事件（menu.py 内部）

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `menu:populate` | 菜单填充 | `menu` | `klippy/extras/display/menu.py:346` | display |
| `menu:init` | 菜单初始化 | `menu` | `klippy/extras/display/menu.py:722` | display |
| `menu:begin` | 菜单开始 | `menu` | `klippy/extras/display/menu.py:754` | display |
| `menu:exit` | 菜单退出 | `menu` | `klippy/extras/display/menu.py:913` | display |

> 依赖 display/menu 模块，优先级最低。上游 `menu.send_event` 实际发 `"menu:" + <名>`，
> 共四个（2026-09-23 复核行号：populate 346 / init 722 / begin 754 / exit 913）；曾列出的
> 「`menu:`（空名）」行是前缀构造的误拆，已删除，事件总数仍为 35（本仓库 decl 恰好 35 个）。

### 依赖关系总结

```
事件总线（KlippyEvent，已就绪）
├── klippy:* 生命周期事件（8个）—— 已触发（notify_mcu_error / analyze_shutdown 也已接入）
├── stepper:* —— 依赖 C1（位置同步/目录反转，尚无触发点）
├── homing:* —— 已触发（toolhead 回零路径）
├── toolhead:* —— set_position 已触发；其余随 G4-2（C1d 已收官）
├── idle_timeout:* —— 依赖 idle_timeout 对象
├── gcode:* —— command_error / request_restart 已触发；debuginput_exit 随 GCodeIO 暂缓
├── probe:* —— 依赖 endstop
├── extruder:* —— 依赖 C1（activate_extruder 尚无发送方）
├── stepper_enable:* —— 已触发（motor_off）
├── virtual_sdcard:* —— 依赖 sdcard
├── load_cell:* —— 依赖 ADC
└── menu:* —— 依赖 display
```

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
| LCD 显示与菜单 | `klippy/extras/display/display.py`、`menu.py:346,722,754,913` |
| 主机统计与 MCU 错误详情 | `klippy/extras/statistics.py`、`error_mcu.py` |

---

## 已完成（留档）

只保留**大项**（标准见上「更新规则」第 3 条）；细项已完成即从正文删除，
需要时查 git 历史。此节不随实现更新，细节见各模块文档与开发手册。

- **上游 `.test` 语料框架与 T1（首个用例转绿）**：`src/core/klippy/upstream.rs` 的 harness 与
      runner（按 `CONFIG` 拆运行、字典齐备才启用、`IGNORED` 留档必然失败的用例）；
      `interface/devices/simulator.rs` 的字典驱动应答机（identify / 配置握手 / 时钟 / ack）；
      `crates/test-support/build.rs` 按 `KLIPPERX_ARCHES` / `KLIPPERX_ALL_ARCHES` 编字典；
      并顺带对齐配置解析（多行值 / `=` / 节头与 `;` 注释）、实现 `deprecate`；
      `linuxtest.test` 端到端通过（`kinematics: none` + `heaters` + `temperature_sensor` +
      `ds18b20`）。后续推进见正文 **T**。

- **框架队列 FW1–FW9（全部完成）**：FW1 配置装载、FW2 对象模型、FW3 错误词汇、FW4 G-Code、
      FW5 运动（a–f）、FW6 资源与触发、FW7 MCU 与传输、FW8 主机层与重启、FW9 API；验收以
      「最小模块在 host 单测 + 假 MCU 上跑通」为准，真板项见 [`TESTING.md`](TESTING.md)。
      两个子项 `[~]` 暂缓：`GCodeIO`（不做 OctoPrint 串口仿真，见正文 G1b）、
      `MCU_bus_digital_out` 包装（能力已由 `DigitalOut::queue_digital_out` 提供，随 H8 显示接）。
      分阶段细节见各 FW 的工作记录（已随收官清理，`git log --diff-filter=D -- docs/work-log/` 可找回）；
      已完结的几篇随收口清理（见目录 README 说明）。

- **toolhead 与 kinematics（C1 / FW5a–f）**：Rust 分层重写（不引 FFI）——`Coord` 与
      `clocksync` 回归、`Move`/`LookAheadQueue`/`trapq`、`itersolve`+`kin_cartesian`、
      `MotionQueuing`/`ToolHead`/`McuStepper`、`Kinematics`+`cartesian`+`[stepper_*]`/`[printer]`+`G1`、
      `stepcompress` 完整压缩；`kinematics: none` 随 T1 补上。设计取舍与逐阶段验收见
      FW5 动工前调查（已完结清理）；回零协议的 `get_trigger_position` /
      `set_stepper_adjustment` 随 `endstop_phase` 后置（H9）。

- **重启循环与 `restart_method` 分派（D2 大部）**：`klippy_process` 就地重建
      （`Printer::reset_for_restart` + `load_config` + `bring_up`，同一个 `Arc<Printer>`），
      以及 `command` / `arduino` / `cheetah` / `rpi_usb` 四种物理复位与连接期门控
      （`src/klippy.rs`、`mcu/object.rs`、`mcu/restart.rs`、`interface/usb.rs`）。

- **GCODE 调度器（G1）**：命令表 / `register_mux_command` / `run_script` / 输出处理器 /
      内置命令，`load_config` 里最先注册（`gcode.rs`）；剩余行为差异见 **G1b**。

- **pin 解析与 `pins`（F2）**：`PrinterPins` / `PinResolver` 的别名与保留，
      `RESERVE_PINS_*` 在 connect 预留；`pins` 注册但不可查询（`pins.rs`、`printer.rs`、
      `mcu/object.rs`）。

- **MCU 配置构建层（F1）**：oid 发号、`config` / `restart` / `init` 三张命令表、config 回调、
      CRC + `finalize_config`，`configure()` 的 `get_config` 两段式下发（`mcu/config.rs`）。

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

- **配置驱动装载**：静态工厂表、两段式构造顺序、section 级合法性校验（`load.rs`）。
