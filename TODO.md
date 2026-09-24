# TODO — 主机（klippy）侧的待办与未决问题

本文件只记**打算做什么**和**还没定的事**，不重复已经定下的设计（那些写在各模块文档和
`docs/klippy/developer-manual/` 里）。括号里的 `klippy/xxx.py:NN` 指上游参考实现
（`third_party/klipper/`），用来在动手前核对行为。已完成的条目单独归档在
[已完成条目归档](docs/work-log/2026-09-22-completed-archive.md)，做完一件事就把它从正文挪过去。
上游**全部功能点**的逐项对照（含判为「不适用」的 Python 专属项）已于 2026-09-21
盘点完毕、结论并入本文件（覆盖审计已完结清理）；本文件只放
**要动手的事**与**还没定的事**。

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
框架队列（FW1–FW9）已完成，索引见文末「已完成（留档）」；本表不再区分级别。
H1–H12 是上游 extras 里按域归并的消费者（2026-09-21 全量盘点的结果；其中若干已落地，
逐条见各节；覆盖审计已完结清理）。

**核心与架构**

| # | 事项 | 依赖 |
|---|---|---|
| C1 | 运动层收尾：轴/stepper 抽象 + 多轴（C1a）、extruder 运动（C1b）、运动学族（C1c）、`gcode_move`/print-time 回调（C1d）；详见下方 C1 小节与 [C1 调查](docs/work-log/2026-09-22-c1-notes.md) | — |
| C2 | 配置装载：框架部分 ✅（FW1，含 choice/range 文案与 `deprecate` 警告）；autosave/`SAVE_CONFIG` 仍待（属模块） | — |
| D1 | 主机层 start args / rollover / `--logfile` ✅（FW8）；剩余：`debuginput`/`debugoutput` 的命令行接线与每 MCU 字典路径 | — |

**MCU 资源与总线**

| # | 事项 | 依赖 |
|---|---|---|
| F6 | SPI 总线剩余：`spi_transfer_with_preface` / `setup_shutdown_msg` | F1、F2 |
| F8 | endstop / trsync ✅（FW6）；测试侧「响应器式多实例假 MCU」待办（可用 `SimulatorDevice`） | F1、F2、C1 |
| F9 | 固件资源剩余：buttons / pulse_counter / trigger_analog / initial_pins / sdcard / sensor_bulk / lcd / neopixel / tmcuart 等（thermocouple 已接：`cmd/thermocouple.rs` + `spi_temperature`） | F1–F7 |

**G-Code 与端点**

| # | 事项 | 依赖 |
|---|---|---|
| G1b | gcode 调度器与上游的行为差异（`get_mutex` 等价物等；`GCodeIO` 暂缓 `[~]`；参数访问器与 `M115`/`Coord`/`request_restart` 已完成并归档） | C1 |
| G2b | 用 GCODE 控制 GPIO：`SET_PIN` 时序（数字与 PWM 均已可驱动） | C1 |
| G4 | 运动命令（G0/G1/G28…） | G1、C1 |
| B4 | 其余端点（pause_resume / `*/dump_*` / …；estop 与 remote method 已落地） | G3、H4、H9 |

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
`SET_PIN PIN=<name> VALUE=<0..1>`；运行时走立即路径（`update_digital_out` / `update_pwm`，
软件 PWM 对齐到周期边界）。剩下的是：

- [ ] **与运动 / 打印时间同步的 `SET_PIN`**（上游 `GCodeRequestQueue`，
      `klippy/extras/output_pin.py:13-85` `:249-269`）：上游把请求排进 toolhead 的
      lookahead、在 print time 生效，并对移动中的 pin 变化与 MCU 最小调度间隔做对齐；
      我们没有 toolhead / print time，只能立即改值（`output_pin.rs` 头注释）。随 **C1**；
      对一个独立 GPIO 不紧急，但打印中改 pin 不会与 move 同步。
- [ ] **`output_pin` 的 `TEMPLATE` + `template_evaluator`**（display 模板，
      `output_pin.py:88-170`）——与开关 GPIO 本身无关，按需再补（`scale` 已随 T8 落地并归档）。

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
- [~] **`gcode:debuginput_exit` 触发（随 `GCodeIO` 暂缓）**：上游 `_do_debuginput_exit`
      轮询 `all(send_event('gcode:debuginput_exit'))`（`:432-435`），依赖 handler 的返回值；
      本仓库 `Printer::send_event` 丢弃返回值（上游 `klippy/klippy.py:226-227` 是
      `return [cb(...)]`）。要与 `GCodeIO` 一起做（见上一条）。
**可独立补的小行为差异**

（`request_restart` 的停机前动作、`Coord`、`M115` 版本号来源三项已完成并归档，
分别见 `gcode.rs:1186-1193`、`mathutil.rs:36`、`gcode.rs:747-767`。）
- [ ] **`get_mutex` 等价物**（原先未列）：上游 `gcode.get_mutex()`（`:242-243`）被
      `bed_mesh`（`:307`）与 `idle_timeout`（`:70` `:90`）用来判断「是否有脚本在跑」；
      本主机无 reactor mutex，是否需要等价物（脚本占用标志）等 C1 与那两个模块落地再定。

### G4 运动命令（G0/G1/G28/G92/M114…）

- [x] **G4-1 `gcode_move`（坐标系核心）**：✅ 新 `extras/gcode_move.rs`（按名字加载，同
      `toolhead.py:610-613` 的 default modules 列表）；`G0`/`G1` 从 toolhead **搬家**到这一层，
      `toolhead` 只留 `move_to`/`position`（上游 `toolhead.move`/`get_position`）。
      命令：`G0/G1`、`G20/G21`、`G90/G91`、`M82/M83`、`G92`、`M220/M221`、
      `SET_GCODE_OFFSET`、`SAVE`/`RESTORE_GCODE_STATE`、`M114`；`get_status`。
      重置链四条：`klippy:ready`（解析 move target）、`homing:home_rails_end`（**新** `axes`
      载荷，只给回零过的轴重新锚定）、`toolhead:set_position`（**新**发送方：
      `SET_KINEMATIC_POSITION`）、`gcode:command_error`。
      调查与拍板见 [G4 notes](docs/work-log/2026-09-23-gcode-move-notes.md)。
- [ ] **G4-2 外围**：`GET_POSITION`（要 `kin.get_steppers()` + `calc_position` + MCU 位置）、
      extra-axes 的 `axis_map`（`Coord` 目前固定 4 轴）、`toolhead:manual_move` /
      `toolhead:update_extra_axes` / `extruder:activate_extruder` 的发送方（等它们的 API）、
      `move_transform`（等 `bed_mesh`）。
- [x] 命令表够用：由 toolhead/gcode_move 注册，`register_mux_command`（`SET_PIN` 这类）可用。

### B4 其余端点（机制部分 FW9）

`api-reference.md` 有、`endpoints/mod.rs` 的表里标「not started」的其余部分，各自等它读的
对象先存在：

- [ ] `pause_resume/{pause,resume,cancel}`：等 `pause_resume` 对象。

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

- [ ] **FW6a-2** 测试侧加**响应器式假 MCU**（可多实例），并补 `ToolHeadObject::connect` 的
      两 MCU 端到端测试。（挪到 FW6c 一起做；时钟偏移已有 `McuChip` 单测）

##### F8b `Mcu` ↔ 资源的强引用环（阻塞回归）

- [x] **环已断（2026-09-22）**：
      - `McuTrsync` 不再持整份 `McuChip`（后者含 `Arc<TrsyncRegistry>`，与 registry 互为强
        引用），改持 `TrsyncChip`（name + mcu 槽 + clock 槽 + print-time mapping）；注册用的
        registry 只在 config 回调里捕获。
      - `McuEvents::clear` + `Mcu::clear_events`，`McuObject::Drop` 在落下前清空回调表，断开
        `Mcu → events → resource → Arc<Mcu>`。
      - 回零 drip 的 `sleep` 与 `completion.wait()` 用 `select!` 竞赛，避免丢唤醒。
      - 验收：`a_homing_move_runs_against_the_fake_firmware`（带 endstop 的 `G28` 跑通且
        进程干净退出，释放后 `Mcu::Drop` 运行）；默认套件绿。
- [x] **剩余时序问题（2026-09-23，已解决）**：`KLIPPERX_UPSTREAM_ALL=1` 在 `commands.test`
      （`QUERY_ENDSTOPS` → `M18` → `G28`）间歇卡住，根因是「同步 g-code 处理器用
      `block_in_place + Handle::block_on` 驱动异步动作」导致 runtime 的定时器被饿住（连
      `call_msg` 的 1s 超时都不触发）。修法：g-code 处理链与客户端 API 边界**全面 async**
      （`CommandHandler` 返回 boxed future、`Endpoint::handle` 返回 `EndpointFuture`），
      三个机器侧 `block_in_place` 桥与两个 API 侧临时桥（`run_script_blocking` /
      `query_all_blocking`）全部删除，见提交 `1782721`、`e05135b`。
      验收：`KLIPPERX_UPSTREAM_ALL=1` 不再卡（~1.06s 跑完），首次失败分布前移到 H2/H1 的真实
      缺口（`fan` 82、`pid_Kp` 必填 49、`probe` 25 …；`pid_Kp` 那项当时误记为 autosave，
      后证实是选项名大小写，已修复归零）。

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


### C1 运动层收尾

FW5a–f / FW6a–f 已把「cartesian + 假 MCU 的 `G1`/`G28`」跑通并归档；C1 现在是把那条单轴
链**推广成通用运动层**。逐项证据、拍板点与上游对照见
[C1 动工前调查](docs/work-log/2026-09-22-c1-notes.md)；这里只记要做的事。

- [x] **C1a 轴/stepper 抽象 + 多轴**：✅ 已完成。`ConfigWrapper::sibling`（`Printer` 的 loader
      把整个 `Config` 传给工厂，兄弟 section 的 option 读取即认领）；`PrinterStepper::setup_itersolve`
      （solver 由拥有者装，缺省回退到名字对应的 cartesian 轴）；`Rail`（`extras/stepper.rs`，
      多 stepper 一轴 + `LookupMultiRail` 等价物）；toolhead 从 rail 建轴、为每个 stepper 装
      cartesian solver。`[stepper_z1]`/`[stepper_z2]` 与 `[stepper_z]` 一起进 Z rail；同级 section
      只读电机选项（`position_*`/`homing_*` 只在主段）。回归里 `Section 'stepper_z1'…` 消失，
      首次失败前移到 `z_tilt`/`quad_gantry_level`（H9）。**不依赖 H1**；为 C1b/C1c/T6/S1 提供落点。
- [x] **C1b extruder 运动**：✅ 已完成。`MotionQueuing` 多 trapq（C1b-1）；`heaters::setup_heater`
      与 `Heater` 桩（C1b-2）；新增 `extras/extruder.rs`（`PrinterExtruder` + `ExtraAxis`）：
      `[extruder]`/`[extruder1]` 经兄弟 section 读取，自带 trapq 与 `extruder_position_fn`
      solver，注册 `M104`/`M109`/`ACTIVATE_EXTRUDER`/`SET_PRESSURE_ADVANCE`；toolhead 在 connect
      时把每个 extruder 挂成 extra axis 并分配 trapq。验收：`an_extruder_move_runs_against_the_fake_firmware`
      （`G1 E…` 同时驱动运动轴与挤出机）。回归里 `Section 'extruder'…` 消失，首次失败前移到
      `pid_Kp` 必填（选项名大小写，已修复归零）/`heater_fan`（H2-3）/`extruder_stepper`
      （H10）（2026-09-23 实跑）；`heater_bed`/`fan` 段均已落地。
- [x] **C1c-1 corexy 族**：✅ 已完成。`CartesianTransform`（Standard/CoreXy/CoreXz/HybridCoreXy/
      HybridCoreXz）把「rail 位置 → 台面轴」抽成值；`itersolve` 加 `corexy_/corexz_ position_fn`
      （`x±y` / `x±z`，active flags `X|Y` / `X|Z`）；`ToolHeadObject` 的 `KinematicsKind` 分派 solver
      与 endstop 互挂（corexy 双向、hybrid 单向，同上游）。回归里 corexy/corexz/hybrid_* 的首
      次失败由 kinematics 前移到 `extruder`（T3）/`dual_carriage`（T9）。
- [ ] **C1c-2 delta 族**：`delta`/`rotary_delta`/`deltesian`/`winch`（迭代求解 + `mathutil` 的
      `trilateration`/`gaussian_solve`）。
- [ ] **C1c-3 generic_cartesian**。
- [ ] **C1c-4 polar**。
- [ ] **C1d print-time 回调**：`ToolHead::register_lookahead_callback` +
      `motion_queuing.register_flush_callback`（`output_pin` 的 `GCodeRequestQueue` 与
      `fan`/`servo`/`pwm_cycle_time` 等着它）。`gcode_move` 的坐标系部分已由 **G4-1** 落地。

> **T3 的边界**：`[extruder]`/`heater_bed`/`fan` 三段与 C1b 已落地，T3 现在卡在 H2-3 的
> `heater_fan`、H1 的 `verify_heater`/`pid_calibrate`、H10 的 `extruder_stepper`
> （未进装载表）与 F9/H7 的 `pulse_counter`（fan `tachometer_pin`）；
> 原列的 C2 autosave（`pid_Kp` 49 次）是误归因，真因是选项名大小写，已修复归零。
> T5 按 C1c 的族顺序推进。

### C2 配置装载收尾（框架 FW1）

- [ ] **autosave / `SAVE_CONFIG`**：`#*#` 自动保存区块的读取（并入配置、与 include 冲突检查、
      损坏检测）与回写（`SAVE_CONFIG` 命令、备份、重启），属模块而非框架；`bed_tilt` / PID /
      `probe_eddy_current` 等消费者都依赖它（上游 `klippy/configfile.py:248`、`:346`）。
      语料里 **0 个配置带 `#*#` 区块**，本项对当前回归失败数为零——原先「首位失败是
      `pid_Kp` 49 次」的归因有误，那实际是选项名大小写问题，已修复归零（见下方失败原因
      统计）。本项要做的是写回侧：`PID_CALIBRATE` 等调用 `configfile.set()` 后由
      `SAVE_CONFIG` 落盘（重启）。
      （`getchoice` 与 `minval/maxval/above/below/count` 文案已由框架补齐并归档，见 FW1/C2。）

### D1 主机层 start args / rollover / 日志（框架 FW8）

- [ ] **`StartArgs` 的剩余接线**：结构体已带 `apiserver`（宿主启动时填，`src/klippy.rs:323`）、
      `start_reason`、`debug_input`/`debug_output`、`device`、`linux_version`；缺的是
      `--debuginput`/`--debugoutput` 的命令行解析（`StartArgs::collect` 里仍是 `None`）与
      每个 MCU 的字典路径。`software_version` 已由宿主 `set_start_args` 注入
      （`src/klippy.rs:324`）并被 `info` 与 `M115` 读取——接线已完成并归档。




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

### T 上游 `.test` 语料推进（按依赖顺序）

框架已落地：`src/core/klippy/upstream.rs`（字典驱动应答机 + 按 `CONFIG` 拆分的运行）与
`crates/test-support/build.rs`（按架构编字典）；T1（`linuxtest.test`）已完成并转绿。当前 239 次
运行里，默认构建缺 2 条（引用 `pru`）、忽略列表 34 条、
实际执行 3 条（`linuxtest.test`、`out_of_bounds.test`、`commands.test`，均通过）；忽略列表即本节的工单，每步做完
就从 `IGNORED` 移除对应文件（手册见
`docs/klippy/developer-manual/regression-tests.md`）。

**推进口径**：下文的「首次失败原因」分组只用于定位，不是工作队列——`load_config` 遇到第一个
未知 section 就停，修好一个缺口只会让运行前进到下一个缺口，总数可能不变（T7 前后失败总数
几乎不变就是例子），各组收益不可加。进度以**转绿运行数 / `IGNORED` 条目数**衡量（当前 3 / 34）。**验收标准
是「对应 `.test` 从 `IGNORED` 移除后通过」**，不是「某个错误不再出现」。详见
[失败原因分析复盘](docs/work-log/2026-09-22-upstream-regression-failures.md#复盘计数口径与重排后补)。

**阶段 0（都不依赖 C1）**：

- [x] **运行 × 缺口扫描**：`upstream_gap_report`（`upstream.rs`）静态扫描每条运行引用的全部
      section 与 `kinematics:`，打印缺口矩阵。实跑显示真正的公共前缀是 `extruder`（**231** 条
      运行引用它，而非「首次失败」的 102；2026-09-21 时点），印证了按工作队列排期的误导性。用
      `cargo test -p klipperx --lib upstream_gap_report -- --nocapture` 查看。
- [x] **`IGNORED` 守卫测试**：`ignored_cases_still_fail`——某个 `IGNORED` 文件的全部可跑运行都
      通过时报失败并提示移除；`KLIPPERX_UPSTREAM_ALL=1` 时不生效（那时本就是要跑全部）。
- [x] **T8** `output_pin` 的 `value` 上界与 `scale` 选项（6 次失败）：`scale` 只在 PWM 路径读
      （`output_pin.py:207-214`），`value`/`shutdown_value` 上界改为 `scale` 并存成除以
      `scale` 的值；`SET_PIN VALUE` 同样按 `scale` 检查。
- [x] **T9 小段**：`restart_method` 在非串口 MCU 上也读一次（记录为已用，只警告不拒绝，修掉
      `Option 'restart_method' is not valid in section 'mcu'`）；新增 `static_digital_output`
      段（`set_digital_out` 命令 + `PinChip::setup_static_digital_out`）。
      **`pwm_cycle_time` 移出**：它需要 `toolhead.register_lookahead_callback` 的 print-time 调度
      与 `[pwm_tool]`，`pwm.test` 靠它无法转绿，随 **C1**。
- [x] **T2 收尾**：自动装载 `stepper_enable`（`PrinterStepperEnable::ensure`，上游
      `stepper.py:282-285` 对每个 stepper 调 `load_object('stepper_enable')`）+ 注册
      `M18`/`M84`/`SET_STEPPER_ENABLE`，M18/M84 与 `gcode:request_restart` 都走 `motor_off`
      并发 `stepper_enable:motor_off`。

按「闭包最小 → 杠杆最大」推进（T1 之后）：

- [x] **T2. `[stepper_enable]`（`enable_pin`）**：✅ 已完成（阶段 0）。`enable_pin` 选项已读取，
      自动装载、`M18`/`M84`/`SET_STEPPER_ENABLE` 与 `stepper_enable:motor_off` 事件均已就位；
      相关配置的首次失败因此前移到了 extruder/probe，仍留在忽略列表（要它转绿还需那些段）。
- [x] **T7. 温度传感器**：✅ 已完成。`temperature_mcu` 真正读 ADC（真板 STM32F103 ~35 °C），
      `AdcTemperatureBridge` 接上 `Arc<dyn Adc>`，`[thermistor <name>]` / `[adc_temperature <name>]`
      段与 8 个内置热敏电阻就位，新增 `spi_temperature`（MAX6675/31855/31856/31865）与
      `temperature_combined`；`debug_read` 系列改用异步 pre-build 钩子并在虚拟 MCU 上验证。
      回归里不再有 `Unknown temperature sensor`。

**当前失败原因统计**（`KLIPPERX_UPSTREAM_ALL=1` 实跑，2026-09-23：**183 次失败**、54 次通过、
2 条因未构建 `pru` 字典不计，合计 239；下表为**选项名大小写修复后**的分布——49 次
`must be specified` 归零但总数不变、首因整体后移，见[复盘](docs/work-log/2026-09-22-upstream-regression-failures.md#复盘计数口径与重排后补)
的「收益不可加」）：

| 首次失败原因 | 次数 | 对应 TODO |
|---|---|---|
| 温度传感器（`temperature_*`） | **0** | T7（已完成） |
| 选项名大小写（`optionxform`） | **0** | 已对齐（49 → 0） |
| `Unknown pin chip name 'probe'`（余量靠 `[bltouch]`/`[smart_effector]`/eddy 注册 chip） | 20 | T4 / H9（M2/M3/M5） |
| TMC：段 23 + pin chip 7 | 30 | T6 / H5 |
| `Error loading kinematics`（delta / generic_cartesian / rotary_delta / polar / winch / deltesian） | 21 | T5（C1c-2/3/4） |
| `Section 'display'` | 21 | T9 / H8 |
| `Section 'heater_fan …'` | 27 | H2-3 |
| `Section 'filament_switch_sensor …'` 等 | 8 | H7 |
| `sensor_pin: … 'vref_scaled'`（`adc_scaled`） | 4 | H1 |
| H9 其余（`bed_mesh` 6、`safe_z_home` 4、`bed_screws` 3、`quad_gantry_level` 2、`z_tilt`/`endstop_phase` 各 1） | 17 | H9 |
| 板级扩展 section（`mcp4451`/`dac084s085`/`ad5206`/`multi_pin`/`sx1509_duex`/`replicape`） | 8 | H2 / H7 |
| `Option 'tachometer_pin' … pulse_counter` | 2 | F9 / H7 |
| `Section 'extruder_stepper …'` | 2 | H10 |
| `Section 'verify_heater …'` | 2 | H1 |
| `Unknown temperature sensor`（`G2`、`Kingroon_B3950`、`NTCS0603E3104FXT`） | 3 | H1 |
| MCU 引脚映射（`Pin 'PF1'`/`'PF7'`/`'PD6'`） | 3 | F2 |
| 单实例（`dual_carriage` 2、`gcode_macro` 2、`led` 2；`virtual_sdcard`/`temperature_fan`/`pwm_cycle_time`/`manual_stepper`/`input_shaper`/`gcode_arcs`/`fan_generic`/`exclude_object`/`controller_fan` 各 1） | 15 | H1 / H3 / H4 / H9 / H10 |
| 运行期失败（`Move out of range`：`generic-simulavr`） | 1 | 运行期（非装载） |
| `Section 'extruder'`（T3 旧首位） | **0** | T3 已消 |

（本表随每个单元更新：2026-09-23 探针链路单元后实测；除 probe 行外其余各行的拆分仍取 U3a 时点，
后续单元只会把首因往后推、总数不变，逐单元变化见各自提交信息。已转绿的语料：`linuxtest.test`、
`commands.test`、`out_of_bounds.test`、`bed_mesh.test`、`z_virtual_endstop.test` 与
`printers.test → printer-wanhao-duplicator-i3-plus-mark2-2019`。）

**T3 之后按首次失败分组的工单**：

- [ ] **T3. `extruder` + `heater_bed` + `fan`**：三段均已落地，`Section 'extruder'` 已 **0 次**
      （2026-09-23 实跑）；原列的 C2 autosave（`pid_Kp` 49）是误归因——语料 0 个 `#*#` 区块，
      真因是选项名大小写，已修复归零。现在卡在 H2-3 `heater_fan`（22）、H10 `extruder_stepper`
      （2）、H1 `verify_heater`（2）与 F9/H7 `pulse_counter`（2）。
- [ ] **T4. `probe` / `bltouch` / endstop pin chip**（35 次失败，2026-09-23 选项大小写修复后实测）：
      `bed_mesh`、`bltouch`、`eddy`、`screws_tilt_adjust`、`smart_effector`、`z_virtual_endstop` 等。
      依赖 F8（endstop，已 ✅）与 H9（probe 模块）。**施工序列**（scout 静态矩阵测算，每步以「移出
      `IGNORED` 后通过」验收）：
      **U1** 运动底座 `probing_move` ✅（2026-09-23：事件顺序、无触发报错、零位移报错）
      → **U2** `[stepper_x/y/z]` 移到 late 阶段 ✅（解 `probe:`/`tmc*_stepper_x:` 的共同根因）
      → **U3a** `[probe]` 段 + `probe` 虚拟 chip ✅（chip 首因 35 → 22）
      → **U3b** 会话采样 + `QUERY_PROBE`/`PROBE`/`PROBE_ACCURACY`
      → **U4** `manual_probe.rs` 命令族 + `configfile.set()` 记账 ✅（2026-09-23；含 PROBE_CALIBRATE 接线）
      → **U5** `bed_mesh.py`（**首批 2 绿**：`bed_mesh.test`、`z_virtual_endstop.test`，随后验证模拟器上
      的 `G28` via `probe:z_virtual_endstop`）
      → **M6** `z_tilt`/`quad_gantry_level`/`bed_tilt`（+2 绿）→ **M7** `STEPPER_BUZZ`（H10，+1）
      → **M2** `bltouch`（+1）→ **M4** `screws_tilt_adjust`（+1）→ **M3** `smart_effector`（+1）
      → **M5** `probe_eddy_current`（+1，另需 `trigger_analog`，F9/H7）。
      其余 37 条 printers 命中 probe 的配置另压跨域长尾（H2/H3/H4/H5/H7/H8 与 H9 兄弟段），不在本闭包内。
      语料里 **0 个配置带 `#*#`**，autosave 不阻塞本批。
- [ ] **T5. 运动学**（21 次失败，2026-09-23 选项大小写修复后实测）：`delta`（12）、
      `generic_cartesian`（4）、`rotary_delta`（2）、`polar` / `winch` / `deltesian`（各 1）。
      corexy 族已随 **C1c-1** 消失，`none` 已在 T1。
- [ ] **T6. TMC pin chip**（28 次失败，2026-09-23 选项大小写修复后实测）：段 21（`tmc2209 stepper_x` 15、
      `tmc2130` 2、`tmc2208` 2、`tmc2660` 1、`tmc5160` 1）+ pin chip 7（`tmc2209_stepper_x` 4、
      `tmc2130_stepper_x` 3）。依赖 H5（TMC）。
- [x] **T7. 温度传感器**（0 次失败）：✅ 已完成（见上）。
- [x] **T8. `output_pin` 的 `value` 选项**（6 次失败）：✅ 已完成（阶段 0）。`value` 上界改为
      `scale`（PWM 才有 `scale`，默认 1），`scale` 不识别的问题随之消失。
- [ ] **T9. 其余 extras 段**（2026-09-23 选项大小写修复后实跑的散项，按域归入 H1–H10）：`display`（21，H8）、
      `filament_switch_sensor`（8，H7）、`adc_scaled`/`vref_scaled`（4，H1）、板级扩展
      （`mcp4451` 2 + `dac084s085` 2 + `ad5206`/`multi_pin`/`sx1509_duex`/`replicape` 各 1，H2/H7）、
      `bed_screws`（3，H9）、`dual_carriage`/`safe_z_home`/`gcode_macro`（各 2），以及 `virtual_sdcard`/
      `exclude_object`/`gcode_arcs`/`manual_stepper`/`pwm_cycle_time`/`led`/`input_shaper`/`temperature_fan`/
      `fan_generic`/`controller_fan`（各 1）。`static_digital_output` 已在阶段 0 落地，`stepper_z1`（多轴）已由
      **C1a** 打开（相关运行的首次失败前移到 `z_tilt`/`quad_gantry_level`，属 H9）。
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

- [x] **Heater 控制环**（bang-bang/PID）✅ 已落地（`heaters.rs` 的 `ControlBangBang`/
      `ControlPID` 与 `update`，含单测）；**剩余**：`get_heater`/`lookup_heater` 注册表与
      `Heater::get_temp`（同时是 H2-3 的阻塞依赖）、`verify_heater` 周期检查
      （`heaters.rs:152-153` 留位）。
- [x] `heater_bed.py` / `heater_generic.py` ✅ 已落地：section 住户、`M140`/`M190`、
      `SET_HEATER_TEMPERATURE`（`heater_bed.rs:24` `:52`、`heaters.rs:499-516`）；
      **剩余**：M190/M109 的等待循环未接（`heater_bed.rs:8-10`、`extruder.rs:20`）。
- [ ] `pid_calibrate.py`（`PID_CALIBRATE`）与 `verify_heater.py`：仍缺
      （`extruder.rs:21` 明示 still open）。
- [ ] 传感器**剩余**：`adc_scaled.py`（回归 `vref_scaled` 4 次）、`temperature_host.py` /
      `temperature_probe.py` / `temperature_fan.py`，以及 `thermistor` 自定义型号
      （`G2` / `Kingroon_B3950` 2 次）。已落地并归档：`temperature_sensor` / `thermistor` /
      `adc_temperature` / `spi_temperature`（MAX6675/31855/31856/31865） /
      `temperature_combined` / `temperature_mcu`（T7）。
- 依赖 F4（PWM）、F5（ADC）、F6（SPI 温度）、C1（`temperature_fan` 随运动）。

### H2 风扇与通用输出

动工前调查见 [H2 notes](docs/work-log/2026-09-23-h2-notes.md)（上游 21 个文件的依赖盘点、
344 次「运行×缺口」测算、H2-1…H2-7 拆分与四个拍板点）。

- [x] **H2-1 `fan.py`**：✅ `[fan]` 段 + 共享的 `Fan` 核心 + `M106`/`M107` + `get_status`，
      `gcode:request_restart` 停风、`enable_pin` 只在 0↔非0 翻转、kick start 满速后回落。
      回归：静态缺口 `fan` **197 → 0**，单缺口运行 55 → 38（`is_fileoutput` 卡点已随下方
      卡点 1 转绿）。
      拍板（notes §5）：调度先走 immediate（`output_pin` 先例，C1d 后与 `GCodeRequestQueue`
      一起切）；kick start 用 reactor 定时器 + 请求序号；`tachometer_pin` **明确报错**（依赖
      F7 `pulse_counter`，不静默 `rpm: null`）；`TEMPLATE` 不在这一层。
- [ ] **H2-2 `fan_generic.py`**（`SET_FAN_SPEED`，复用 `Fan`；`TEMPLATE` 报错）。
- [ ] **H2-3 `heater_fan.py`**：需要 `heaters::{add_heater,lookup_heater}` + `Heater::get_temp`
      （H1 面），`klippy:ready` 起每秒 timer。
- [ ] **H2-4 `controller_fan.py`**：`stepper_enable::get_steppers`/`lookup_enable` 已有。
- [x] **转绿卡点 1（`is_fileoutput`，T3/H1 共用）**：✅ `Printer::is_fileoutput()`
      （= `start_args['debugoutput']`，`mcu.py:1169`）+ `heaters.py:38-39` 的
      `can_extrude` 初值 + 回归 harness 模拟 `test_klippy.py` 的 `-o`。上游每个用例都带
      `-o`，温度查询永远无人应答，所以 `can_extrude` 必须初值为真。`Extrude below minimum
      temp` **48 → 0**。
- [x] **转绿卡点 2（`gcode_move`，G4/H10）**：✅ `Move out of range` **49 → 0**，回归失败运行
      **当时 235 → 187**（一次多转绿 48 次；2026-09-23 实跑现为 186）。
- [x] **转绿卡点 3（`pid_Kp` 选项名大小写，config 解析）**：✅ 已修复归零——原判「autosave 缺
      `#*#` 读取（C2/H1）」属误归因：语料里 0 个配置带 `#*#` 区块，49 次失败的真因是文件写
      `pid_kp` 而代码查 `pid_Kp`。已按上游 `optionxform = str.lower` 对齐（`mod.rs` 存储侧
      小写 + `section.rs` 查询侧小写；`override_config` 的键同样折叠），`must be specified`
      文案保留调用方大小写，`is not valid` 用存储侧小写。autosave/`SAVE_CONFIG` 仍属 C2，
      为未来 `PID_CALIBRATE` 的写回服务。
- [ ] `pwm_tool.py`（队列化 PWM，随运动）、`pwm_cycle_time.py`、`static_pwm_clock.py`
      （`static_digital_output.py` 已随 T9 阶段 0 落地并归档）。
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

- [ ] 探针：`probe.py` ◐（**已落地**：`[probe]` 段、`probe` 虚拟 chip、选项与偏移、会话采样、
      `QUERY_PROBE`/`PROBE`/`PROBE_ACCURACY`、`probe:update_results` 发送；**待做**：
      `PROBE_CALIBRATE`/`Z_OFFSET_APPLY_PROBE`（需 `manual_probe` + `configfile.set()`）、
      `ProbePointsHelper`（消费者 z_tilt/screws）、endstop wrapper 的 `z_offset`/`query_endstop` 覆盖——
      需先把 `PinChip::setup_endstop` 的返回类型接口化）、`bltouch.py`、`smart_effector.py`、
      `probe_eddy_current.py`、`manual_probe.py`、`safe_z_home.py`、`endstop_phase.py`。
- [ ] **保真单元：bed_mesh 插值 + 调平应用**（排在 H9 模块闭包之后）：`LagrangeMesh`/`BicubicMesh` +
      `mesh_pps` → `mesh_matrix`、`get_z(x, y)`，以及把网格作用到 move（`gcode_move` 的 move-transform
      seam + `MoveSplitter` + `fade_*`）。素材：`abandoned/wip-main-leftovers` 的 `bed_mesh.rs`
      （含 `ZMesh`/lagrange/bicubic 与两个测试），按主线结构重做并补手册。
- [ ] **保真单元：探针精度**（排在 H9 模块闭包之后）：按触发步数反算位置与 `rest_time`
      （上游 `_calc_endstop_rate`）。前置：模拟器步数模型（否则 `stepper_get_position` 恒 0，无法验收），
      见 F8 的「响应器式假 MCU」。
- **端停位置的缺口（需接口化）**：上游 rail 会优先向 endstop 要位置（`mcu_endstop.get_position_endstop()`，
      探针 wrapper 返回 `z_offset`）；本仓 `position_endstop` 缺省退到 `position_min`，所以用
      `probe:z_virtual_endstop` 的配置虽然在解析上能过，但 Z 回零后的位置不等于上游——`z_virtual_endstop.test`
      的位置断言需要它。
- **已知偏离（探针精度）**：本仓回零与探针移动返回**指令位置**，未按上游 `StepperPosition.note_home_end`
      + `calc_toolhead_pos` 用触发步数反算；`home_start` 的 `rest_time` 也硬编码（上游 `_calc_endstop_rate`
      按 move 距离与步数计算）。两者都影响真实探针 Z 精度，属后续精度单元；模拟器语料不受影响。
- [ ] 调平：`bed_mesh.py` ◐（**已落地**：`[bed_mesh]` 段与全量选项、探测点生成、
      `BED_MESH_CALIBRATE` 逐点探测存格、`BED_MESH_CLEAR`；**待做**：插值网格
      （lagrange/bicubic、`mesh_pps`）、faulty 区域替换、fade 与 move 的 z 补偿、profile 命令
      与 `bed_mesh/dump_mesh` 端点）、`bed_tilt.py`、`quad_gantry_level.py`、`z_tilt.py`。
- [x] **探针端到端：时钟纪元**：✅ 已完成（2026-09-24）。探针链路本身已在假 MCU 上验收（`upstream.rs` 的 5 条聚焦
      E2E：普通端停 `G28 Z`、`probe:z_virtual_endstop` 的 `G28 Z`、`G28 + PROBE`、
      `G28 + PROBE_CALIBRATE/TESTZ/ACCEPT`、`G28 + BED_MESH_CALIBRATE`（3×3）），假 MCU 的端停时序也已建模
      （`endstop_home` 只武装，待 `reset_step_clock`/`queue_step`——即移动真的开始——才回 `trsync_state`，
      否则探针初始就是“已触发”）。**剩余拦阻是时钟纪元**：`cmd/clock.rs:298` 的 `clock32_to_clock64` 以
      `last_clock`（假 MCU 报的**墙钟**）为参考选 32 位时钟的纪元，而 arm clock 来自**快进的打印时间**——
      7×7 标定（`bed_mesh.test`）在毫秒级真实时间里推进了 ~136 s 打印时间，超过 2³¹ tick 的半圈窗口，
      于是触发时刻被映射到**前一圈**（差 2³²/16 MHz = 268.44 s）。实测证据：插桩下 `reason=Some(EndstopHit)`
      而 `trigger_time=-132.27`，正确值 `136.17 = -132.27 + 268.44`，因此 `home_wait` 返回负值、
      `probing_move` 报 `No trigger on probe after full movement`。试过的修法（让假 MCU 的 `get_clock`
      跟“已入队时钟”取 max）**未解决**，提示要查 `clock_sync` 的估计路径（或 `home_wait` 的
      `clock32_to_clock64` 调用是否该用别的参考时钟）。
- [ ] 螺丝：`bed_screws.py`、`screws_tilt_adjust.py`。
- [ ] 校准：`delta_calibrate.py`、`axis_twist_compensation.py`、`skew_correction.py`、
      `z_thermal_adjust.py`、`tuning_tower.py`。
- [ ] 回零周边：`homing_override.py`、`homing_heaters.py`；事件 `probe:update_results` 未触发
      （`homing:*` 四个已随 toolhead 落地产线触发，见事件清单）。
- 依赖 C1、F8（endstop/trsync）、H3（宏）、H12（`mathutil`）。

### H10 运动相关 extras

- [x] **`gcode_move.py`（G4-1）**：坐标系核心已落地，见 **G4** 小节；G4-2 外围待做。
- [ ] `gcode_arcs.py`（G2/G3）、`manual_stepper.py`；`force_move.py` **部分**（只有
      `SET_KINEMATIC_POSITION`，`FORCE_MOVE`/`STEPPER_BUZZ` 未接）；`extruder_stepper.py`
      仍缺（`[extruder_stepper <name>]` 未进装载表，回归 2 次）。
      （`stepper_enable.py` ✅ 已随 T2 落地并归档。）
- [ ] `idle_timeout.py`（`idle_timeout:*` 事件已声明未触发）、
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
| `idle_timeout:ready` | idle_timeout 模块就绪 | 无 | `klippy/extras/idle_timeout.py:44` | — |
| `idle_timeout:idle` | 进入空闲状态 | 无 | `klippy/extras/idle_timeout.py:57` | — |
| `idle_timeout:printing` | 开始打印（恢复活动） | 无 | `klippy/extras/idle_timeout.py:95` | — |

> 需 idle_timeout 对象（`[idle_timeout]`），目前未实现。

### 工具头事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `toolhead:manual_move` | 手动移动前 | `positions, speed` | `klippy/toolhead.py:390` | 尚无触发点（handler 已备，G4-2） |
| `toolhead:set_position` | 设置位置（G92 等） | `positions, e` | `klippy/toolhead.py:416` | ✅ 已触发（`extras/toolhead.rs:1091`，`SET_KINEMATIC_POSITION`） |
| `toolhead:sync_print_time` | print_time 更新 | `print_time` | `klippy/toolhead.py:446` | 尚无触发点（随 C1d） |
| `toolhead:update_extra_axes` | 额外轴位置更新 | `positions` | `klippy/toolhead.py:455` | 尚无触发点（handler 已备，G4-2） |

> `toolhead:set_position` 已产线触发（`gcode_move` 重置链之一）；其余三个尚无触发点：
> `manual_move`/`update_extra_axes` 等 G4-2 的 API，`sync_print_time` 随 C1d。

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
| `probe:update_results` | probe 测量完成 | `results` | `klippy/extras/probe.py:200` | endstop |
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
├── toolhead:* —— set_position 已触发；其余随 G4-2 / C1d
├── idle_timeout:* —— 依赖 idle_timeout 对象
├── gcode:* —— command_error / request_restart 已触发；debuginput_exit 随 GCodeIO 暂缓
├── probe:* —— 依赖 endstop
├── extruder:* —— 依赖 C1（activate_extruder 尚无发送方）
├── stepper_enable:* —— 已触发（motor_off）
├── virtual_sdcard:* —— 依赖 sdcard
├── load_cell:* —— 依赖 ADC
└── menu:* —— 依赖 display
```

## 已完成（留档）

全部完成条目已单独归档：[**已完成条目归档**](docs/work-log/2026-09-22-completed-archive.md)
（框架队列 FW1–FW9、各模块与总线的落地索引；细节在各模块自己的文档里）。

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
