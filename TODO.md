# TODO — 主机（klippy）侧的待办与未决问题

本文件只记**打算做什么**和**还没定的事**，不重复已经定下的设计（那些写在各模块文档和
`docs/klippy/developer-manual/` 里）。括号里的 `klippy/xxx.py:NN` 指上游参考实现
（`third_party/klipper/`），用来在动手前核对行为。已完成的条目见文末「已完成（留档）」，
只留一行索引，细节在各模块自己的文档里。

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

依赖列的是**工具性前置**，不是自然顺序。

| # | 事项 | 依赖 |
|---|---|---|
| G4 | 运动命令（G0/G1/G28…） | G1、C1 |
| A1b | reactor 串行调度器与延迟度量 | A1 |
| A2 | 错误词汇（`CommandError` / `ConfigError`） | — |
| B2 | MCU 剩余：`emergency_stop` 对象/端点、`last_stats`、`restart_method` 校验 | — |
| B4 | 其余端点（estop / remote method / pause_resume / …） | G3 等 |
| F3 | `MCU_bus_digital_out`（命令队列/运动同步输出） | C1 |
| F4 | PWM（硬件 / 软件） | F1、F2 |
| F5 | ADC | F1、F2 |
| F6 | SPI 总线 | F1、F2 |
| F7 | I2C 总线 | F1、F2 |
| F8 | endstop / trsync | F1、F2、C1 |
| F9 | 输入与外设资源（buttons / pulse_counter / …） | F1–F7 |
| C1 | toolhead 与 kinematics | — |
| C2 | 配置装载收尾（option 校验、第二个住户） | — |
| D1 | 主机层 start args / rollover / `--logfile` | — |
| D2 | `restart_method` 分派（重启循环已完成） | — |
| E1 | 文档 | — |
| E2 | `python_path` 的取消 | 外部项目 |

### A1b reactor 的串行调度器与延迟度量

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

### A2 错误词汇

- [ ] `CommandError` / `ConfigError`（上游 extras 里 94 / 41 处）；`KlippyError` 现在只有
      通信类 5 个变体（`src/core/klippy/error.rs`）。
- [ ] 借 `KlippyError::Internal` 的地方已经出现，是这条的验收点：`Printer::add_object`
      的重复名（`printer.rs` 的 TODO）、`load.rs` 的工厂拒绝与未认领 section（`load.rs`
      的 `TODO`）、`McuObject::connect` 的 section 解析。

### B2 MCU 关闭与错误上报（剩余）

- [ ] **`emergency_stop` / `clear_shutdown` 的对象与端点**：`cmd/shutdown.rs` 的两个命令现在
      只有 `configure` 的复位路径在用 `emergency_stop`；还缺“主机侧停机时通知 MCU”与
      `emergency_stop` 端点（端点本身见 B4，上游 `klippy/mcu.py:801-802` `:883`）。
- [ ] **`last_stats` 仍未报**：`stats` 事件现在只打日志（`event/stats.rs` 的
      `register_stats_logging`），所以 `McuObject::get_status` 只报三个 identify 字段。
      上游由 `MCUStatsHelper` 累计（`klippy/mcu.py:912` `:974-975`），`get_status` 多一个
      `last_stats`（`klippy/mcu.py:1235`）。需要先有 stats 消费者。
- [ ] **错误上报带载荷**：上游 `klippy:notify_mcu_error` 带 `msg` 与 details
      （`klippy/klippy.py:144` `:151`），shutdown 分析走 `klippy:analyze_shutdown`
      （`klippy/klippy.py:216-220`）。当前 `PrinterEvent` 的 handler 无参，表达不了；
      我们现在的做法是把原因写进状态消息（上游放在 details 里），见 Q2 / Q3。
- [x] **`restart_method` 的校验与默认**：未知值报配置错（不再静默变 Arduino）、非串口
      （CAN / host）恒为 `command` 且不读该项、串口缺省 `arduino`，与上游 `getchoice` +
      `if baud` 对齐（`klippy/mcu.py:666-671`）。“是串口”做成参数以便不依赖真串口测试。
      四种方法的**物理分派**（arduino / cheetah / rpi_usb）见 D2。
- [ ] **不认 `reset` 命令**：`mcu/config.rs` 缺 `config_reset` 就报「断电」。上游
      `_restart_via_command` 优先 `reset`，没有才 `force_local_shutdown` + `config_reset`
      （`klippy/mcu.py:730-746`）。
- [ ] **reset 期间没有本地 shutdown 标志**：现在靠「`configure` 完成后才 `bind_shutdown`」的
      时序规避；隐式、无测试，recv 一旦改成缓冲/异步就会把自发的 `emergency_stop` 误报成
      `MCU … restarted`。上游有 `_is_shutdown`（`klippy/mcu.py:893-895`）。

### G4 运动命令（G0/G1/G28/G92/M114…）

- [ ] 由 toolhead 注册，随 **C1**；gcode 层不需为它们改什么，只要命令表够通用
      （含 `register_mux_command`，给 `SET_PIN` 这类 `PIN=` 选择用）。

### B4 其余端点

`api-reference.md` 有、`endpoints/mod.rs` 的表里标「not started」的其余部分，各自等它读的
对象先存在：

- [ ] `emergency_stop`（`klippy/webhooks.py:322` `_handle_estop_request`）。
- [ ] `register_remote_method`：方法表与推送（`klippy/webhooks.py:319` `:323` `:391`
      `:412`）。
- [ ] `pause_resume/{pause,resume,cancel}`：等 `pause_resume` 对象。
- [ ] `query_endstops/status`：等 endstop / homing。
- [ ] `bed_mesh/dump_mesh` 与 `*/dump_*` 多路复用端点（`klippy/webhooks.py:335`
      `_handle_mux`）：等对应 extras（`bed_mesh`、`adxl345` 等）。

### F MCU 基础资源（F4–F9）

上游把这些叫 printer objects 下面的「资源」：主机用一个 **oid** 和一个 **pin 描述**
建立资源对象，把 `config_*` 命令攒起来，在 `finalize_config` 之前算一个 CRC 一次性下发，
之后用 `queue_*` / `set_*` / `*_transfer` 命令驱动。命令层（`allocate_oids` / `get_config` /
`finalize_config` / `get_uptime` / `emergency_stop` / `get_clock`）已就位，配置构建层（F1）
与 pin 解析（F2）也已完成，剩下的缺口是**任何一个 `config_*` 资源**本身。

#### F3 剩余：`MCU_bus_digital_out`

- [ ] `MCU_bus_digital_out`（`klippy/extras/bus.py:337` 以后）：挂在命令队列上、与运动
      同步的输出；需要命令队列/运动层（C1）。
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

#### F2 剩余：`[board_pins]` 与 `BUS_PINS_<bus>`

- [ ] **`[board_pins]`**（`klippy/extras/board_pins.py`）：调用 `alias_pin` / `reserve_pin` 的
      section，需要 config 的 list 解析与一个新工厂项；解析器 API 已就绪，section 随配置装载
      （C2）一起接。
- [ ] **`BUS_PINS_<bus>`**：由 SPI/I2C 在开总线时预留（`klippy/extras/bus.py:9-32`），随 F6/F7。

### C1 toolhead 与 kinematics

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

### C2 配置装载收尾

- [ ] **option 级校验**：上游拿访问追踪当 schema（`klippy/configfile.py:435-441`），
      `ConfigSection` 还没有访问记录，未做。
- [ ] **住户只有 MCU**：上游在 `_read_config` 里显式加载的 `pins` / `configfile` /
      `toolhead` 还没有入口，所以任何真实 printer.cfg 现在都会在未认领的 section 上报错；
      第二个住户进来时按同一张表补（C1 的 toolhead 就是下一个）。

### D1 主机层 start args / rollover / 日志

- [ ] `StartArgs` 只有 `info` 需要的四个字段（`api/start_args.rs`）；上游的
      `apiserver`、`start_reason`、debug 输入输出、每个 MCU 的字典路径还没进来。
- [ ] rollover info：上游 `set_rollover_info` 7 处（`klippy/klippy.py:369` 起），给 `info`
      与日志用；归主机层，不进机器。
- [ ] `--logfile`：现在没有，`log_file` 恒为 `null`（`api/start_args.rs`）；先有写文件的
      日志层，rollover 才有意义。

### D2 重启循环（剩余）

循环本身已完成（见文末）：`klippy_process` 是循环，`run()` 返回重启类结果就
`reset_for_restart()` + `load_config()` + `RESTART_DELAY` 后重来。剩下的是与上游不同的几块。

#### `restart_method` 分派：分析

上游 `MCURestartHelper` 在 `klippy:firmware_restart` 事件上按方法四选一
（`klippy/mcu.py:746-770`）：

| 方法 | 动作 | 依赖 |
|---|---|---|
| `command` | 有 `reset` 就用它，否则 `force_local_shutdown` + 15 ms + `config_reset`，再 disconnect | 固件命令（B2 已做 `config_reset`，差 `reset`） |
| `arduino`（含未设） | disconnect 后以 2400 打开、`read(1)`、DTR true→false（`serialhdl.py:392`） | tty DTR |
| `cheetah` | disconnect 后 RTS 拉高、DTR 两轮翻转、RTS 拉低（`serialhdl.py:365`） | tty DTR+RTS |
| `rpi_usb` | disconnect 后 `hub-ctrl -h 0 -P 2 -p 0` → 2 s → `-p 1`（`chelper/__init__.py:339`） | 外部 `hub-ctrl` + sudo |

外加三处按方法的**连接期门控**：`rpi_usb` 且串口不存在 → 先启动一次去上电（`:693`）；
`rpi_usb` → 上电复位前不许 configure（`:696`）；`cheetah` → 连接时 RTS 必须拉低（`:703`）。
方法只在**有 baud（串口）**时从配置读，CAN 恒为 `command`（`:668-671`）；`CANBUS_BRIDGE` 的
MCU 默认跳过（`:749`）。

**我们已经有的**：`command`（`config_reset` + 事件屏障，见 B2）；以及“关掉再开”的形状
（`reset_for_restart` 丢对象关设备 → `bring_up` 重开）。**`arduino` 可能被“重开 tty 会拉
DTR”隐式满足，但那是驱动副作用，不算实现。**

**缺口（按依赖）**：

1. **没有 `start_reason`**（D1）：`connect` 分不清“首次启动”与“firmware_restart 后的重连”，
   而 attach 期门控全靠它；循环知道原因，`McuObject` 不知道。
2. **分派点/时序对不上**：`run()` 发 `FirmwareRestart` 时串口还开着，drop 在 `run()` 返回
   之后；自然的落点是 connect **之前**，但 `McuConfig::new` 把解析与打开耦合了（内部
   `create_interface` 直接 `SerialDevice::open`），要先拆开才能在打开前复位/上电/定 RTS。
3. **`serial.rs` 没有 modem 线控制**：要加 DTR/RTS（`TIOCMBIS`/`TIOCMBIC`）并能以 2400 短暂
   打开再关。
4. **`rpi_usb` 要外部程序**：`hub-ctrl.c` 在 `third_party/klipper/lib/hub-ctrl/`，但上游现场
   gcc 编译 + `sudo` 跑；对 Rust 主机是环境/权限问题。
5. **`restart_method` 没被带到分派点**：`Mcu::new` 只取 name+interface，字段被丢
   （`mcu/mod.rs:374-376`）。
6. **校验与默认**（本轮先做）。
7. **`reset` 命令**（B2 单列）。

**推进顺序**：⑥ `restart_method` 校验/默认 ✓ → ② `McuConfig` 解析/打开拆分 ✓ →
① `start_reason`（最小切片）✓ → ③ `serial.rs` DTR/RTS + `arduino` 显式复位 ✓ →
⑤ `cheetah`（含 attach RTS）✓ → ④ `rpi_usb`（sysfs 拓扑 + `nusb` + udev）✓。

- [x] **⑥ `restart_method` 校验/默认**：未知值报配置错，非串口（CAN / host）恒为 `command`、
      且配置了该项时告警，串口缺省为 `arduino`（对齐上游 `getchoice` + `if baud`，
      `klippy/mcu.py:666-671`）；`info` 级日志报出最终命中的方法。
- [x] **② `McuConfig` 解析/打开拆分**：新增 `Transport`（`Serial` / `Can` / `Host` / `Test`）
      描述，`McuConfig::new` 只解析（含全部配置校验），`McuConfig::open` 才开设备；
      `Mcu::new` / `Mcu::connect` 改成吃 `(name, interface)`。测试拆成“路由”与“打开”两段。
- [x] **① `start_reason`（最小切片）**：`Printer` 记下这次上线的原因
      （`reset_for_restart(reason)`），`McuObject` 由此判断是不是 `firmware_restart`。
      把 D1 的 `StartArgs` / `info` 接上仍待做。
- [x] **③ `serial.rs` DTR/RTS + arduino 复位**：`ModemLines`（`set_dtr` / `set_rts`，
      `TIOCMBIS`/`TIOCMBIC`）+ `mcu/restart.rs::reset_firmware`—— `command` 不动、
      `arduino` 以 2400 开、排空、DTR 三拍（`klippy/serialhdl.py:392`）、`rpi_usb` 记 warning
      后继续（仍靠 `config_reset` 恢复）。在 `McuObject::connect` 的 open **之前**、且仅
      `firmware_restart` 时调用。
- [x] **⑤ `cheetah`**：`cheetah_reset`——以 2400 开、RTS 拉高、排空，然后 DTR 两轮翻转、
      中间把 RTS 拉低（`klippy/serialhdl.py:365`）；另加**连接期**的 RTS 拉低
      （`lookup_attach_uart_rts`，`klippy/mcu.py:703-705`）：`McuConfig::open` 对 cheetah
      传 `rts=false`，`Transport::open(rts)` 在打开后立即 `SerialDevice::set_rts(false)`。
- 注（③/⑤ 共有）：pty 不模拟 modem 线（`TIOCMBIS` = `ENOTTY`），所以 DTR/RTS 序列
      **没有端到端测试**，只测了 `ModemLines` 能开 tty + 分派到串口复位；两条路径**实际执行
      时都告警「未在真板上测过」**，手工测试确认后把这两条 warn 删掉。
- [x] **④ `rpi_usb`**：上游 `_restart_rpi_usb`（`klippy/mcu.py:748`）是 `disconnect` →
      `run_hub_ctrl(0)` → 2 s → `run_hub_ctrl(1)`，而 `run_hub_ctrl`
      （`chelper/__init__.py:339`）现场 `gcc` 编译 `lib/hub-ctrl/hub-ctrl.c` 再
      `sudo hub-ctrl -h 0 -P 2 -p {0,1}`。实质只有**一个 USB 控制传输**
      （`bmRequestType=0x23`、`SET/CLEAR_FEATURE`、`wValue=USB_PORT_FEAT_POWER(8)`、
      `wIndex=端口号`；`hub-ctrl.c:396-403`）。Rust 实现：
      - **拓扑发现**（`interface/usb.rs::resolve_tty_port`）：从 `/sys/class/tty/<tty>/device`
        上溯到 USB 设备，取父 hub 的 `busnum`/`devnum` 与端口号（`<hub>.<port>` / 根 hub 的
        `<bus>-<port>`）。纯 sysfs 读取、无需权限，**有单测**（假 sysfs 树）。不抄上游的
        `-h 0 -P 2`：端口从拓扑得出（将来可加一个配置项作覆盖）。
      - **切电**：两条路，`usb_power`（`auto`/`sysfs`/`libusb`）选。`sysfs` 写内核 ≥ 6.0 的
        端口 `disable` 文件（用 glob `*port<N>` 兼容命名，ABI 文档的 `port<X>` 与 7.x 的
        `<hub>-port<X>` 都认）；`libusb` 用 `nusb`（**纯 Rust**，无 libusb）对 hub 发
        `SET_FEATURE`/`CLEAR_FEATURE(PORT_POWER)`。`restart.rs` 里 `spawn_blocking` 跑，
        中间夹 2 s（`USB_POWER_OFF`），再等 tty 回来（`USB_PORT_RETURN_TIMEOUT`，顺带满足
        上游 `check_restart_on_attach`）。
      - **权限（用 udev，不用 sudo）**：两条路都要 root，上游因此 `sudo`；改为限定到该 hub 的
        udev 规则。`scripts/klipperx-usb-udev.sh` 按串口设备从 sysfs 生成并安装它（`--install`
        用 `sudo sh -c "cat > …"` 写 `/etc/udev/rules.d/` 再重载 udev）；
        `usb::recommended_rules` 在告警里给同样两条。
      - **上线探测**：`restart::check_usb_power` 在每次 connect 按 `usb_power` 探一次（含写权限），
        两条都不可用就告警并附上该 hub 的规则，不必等第一次 `FIRMWARE_RESTART`。
      - **已在真板验证**（`0424:2137` 的 hub）：hub 描述符 `wHubCharacteristics lpsm=1`，
        `CLEAR_FEATURE(PORT_POWER)` 后端口状态 `0x0103 → 0x0000`、`SET_FEATURE` 后回到
        `0x0103`，板子确实掉线重连（`rpi_usb_reset` 2 s 断电 + ~0.5 s 等重枚举后返回），
        所以这条**去掉了** `warn_untested`（`arduino`/`cheetah` 仍保留）。
      - 验证时发现并修掉两个真 bug：① 判断“设备回来了”不能用 `/dev` 节点（内核在端口断电
        期间**不拆设备**，节点和 sysfs 链接都还在），要用 USB 设备的 node 号变了
        （实测 `189:40 → 189:41`）；② `resolve_tty_port` 原来用给定路径的 basename 找
        sysfs，`/dev/serial/by-id/…` 这种解析不了，现在先 `canonicalize`。
        等待判定放宽为「node 号变了 **或** 中途见到设备消失过」——内核可能把刚释放的号直接
        再发出来（`choose_devnum` 是游标式，实测连续 42→43→44→45，复用概率低但非零）。
      注：“上电复位前不许 configure”（`:696`）在我们的“先复位、再 open、再 configure”
      顺序下天然成立。
      待真板确认：树莓派 5 的板载 hub 自称 per-port、实为 ganged，只切一个端口切不掉 VBUS，
      需按实际硬件验证。
- [ ] **CRC 不匹配仍走就地复位（有意偏离上游）**：上游发现已配置但 CRC 不一致时先
      `request_exit('firmware_restart')`（`check_restart_on_crc_mismatch`，
      `klippy/mcu.py:678-685`、`:1057-1059`），让重启循环做物理复位；我们用 `configure` 里的
      `emergency_stop` + `config_reset` 就地复位（已在 B2 实现并测试，真板不必断电）。若要
      贴上游，还有 `start_reason == 'firmware_restart'` 却仍已配置时 raise “Failed automated
      reset” 的前置门（`:1053-1056`）。详见 `docs/klippy/developer-manual/mcu-config.md`。
- [ ] **重启后的 g-code 输出订阅**：连接不断，`objects/subscribe` 也自动继续（它按名查新对象），
      但 `gcode/subscribe_output` 的处理器挂在被重建的 `GCodeDispatch` 上，重启后静默失效，
      要客户端重新订阅。上游靠 socket 重绑让客户端重连、重订阅；我们要么在客户端收到
      `klippy:ready` 后重订阅，要么把输出订阅表移到连接上。

### E1 文档

- [ ] `docs/klippy/developer-manual/`：补 printer 一节，并在 README 的分层表里加
      `printer` 一行（现在只有 msg / mcu / cmd / event / identify / api）。
- [ ] **过期描述**：`printer.rs` 头注释仍写「No part is loaded from the config into it
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

## 未决问题

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

## 已完成（留档）

细节在各模块文档里；这里只留索引，最近完成的在前。

- **重启循环（D2）**：`klippy_process` 按 `run()` 的结果决定退出还是重建；重建是**就地**的——
      `Printer::reset_for_restart` 丢掉 config 装载的部件（关设备）并保留 host 的 `webhooks`，
      再 `load_config` + `bring_up`，同一个 `Arc<Printer>` 继续服务（Q7）。为此把
      `pins ↔ McuChip` 的强引用环改成 chip 持 `Weak<PrinterPins>`，否则旧 MCU 不会被释放
      （`src/klippy.rs`、`printer.rs`、`load.rs`、`mcu/pin.rs`）。

- **identify 后的 DEBUG 摘要**：`describe_dictionary` 在 `Mcu::identify` 里打版本对、
      消息条数与常量（`identify.rs`）。
- **客户端的 `firmware_restart`**：`Session::firmware_restart` 与本地命令
      `.firmware_restart`（行模式与 g-code 模式都认），`usage()` 同步更新
      （`crates/klippy-client/src/session.rs`）。
- **`Mcu::flush`**：发送队列的 item 变成 `SendItem::Payload | SendItem::Flush(oneshot)`，
      发送任务遇到 barrier 就把当前 batch 立即发走并回报；`TestDevice::recorder()` 让
      block 边界可断言（`mcu/mod.rs`、`interface/test.rs`）。
- **reset 路径的 P0/P3**：`emergency_stop` 与 `config_reset` 分两个 block，中间用
      `Mcu::call(EmergencyStop, …, Shutdown, …)` 等固件的 `shutdown` 报告（注册先于发送，
      是真屏障）；无 `shutdown` 时 15 ms 兜底并告警，固件上线时 `bind_shutdown` 也会告警
      （`mcu/config.rs`、`mcu/object.rs`）。原 bug：两条命令被合批，固件的 longjmp 掀掉
      block，`config_reset` 被丢。
- **`gcode/subscribe_output` 与 TUI g-code 模式**：连接包成带 `is_closed` 的
      `OutputHandler`，推 `{response: line}`；`^G` / `.gcode` 整行走 `gcode/script` 并自动
      订阅输出，`// …` 与 `!! …` 都可见（`api/endpoints/gcode.rs`、`gcode.rs`、`klippy-client`）。
- **`gcode/*` 端点（G3）**：`gcode/help` / `script` / `restart` / `firmware_restart`；
      命令级错误用 `ApiError::CommandError`，不关停 klippy（`api/endpoints/gcode.rs`）。
- **`output_pin` 与 `SET_PIN`（G2）**：`[output_pin <name>]` 用 `setup_digital_out` 建数字
      输出并注册 `SET_PIN PIN=… VALUE=…`；无条件 `setup_max_duration(0)`（`extras/output_pin.rs`、
      `load.rs`）。`pwm` 暂拒，`SET_PIN` 先立即生效。
- **GCODE 调度器（G1）**：`GCodeDispatch` 的命令表 / `register_mux_command` / `run_script` /
      输出处理器 / 内置命令，`load_config` 里最先注册（`gcode.rs`）。
- **GPIO 数字输出（F3 的 MCU 部分）**：`PinChip` / `DigitalOut`、`config_digital_out` +
      restart 的 `update_digital_out` + 运行期 `queue_digital_out`（`cmd/gpio.rs`、`mcu/pin.rs`）。
- **pin 解析与 `pins`（F2）**：`PrinterPins` / `PinResolver` 的别名与保留，`RESERVE_PINS_*`
      在 connect 预留；`pins` 注册但不可查询（`is_queryable` / `queryable_objects` /
      `lookup_object_as`）（`pins.rs`、`printer.rs`、`mcu/object.rs`）。
- **MCU 配置构建层（F1）**：oid 发号、`config` / `restart` / `init` 三张命令表、config 回调、
      CRC + `finalize_config`，`configure()` 的 `get_config` 两段式下发（`mcu/config.rs`）。
- **MCU 停机上报与就地复位（B2 大部）**：`shutdown` / `is_shutdown` / `starting` 事件经
      `static_string_id` 解成原因，配置握手**之后**绑成打印机停机；`configure` 用
      `emergency_stop` + `config_reset` 就地复位再配置（`event/shutdown.rs`、`mcu/config.rs`、
      `mcu/object.rs`）。
- **`objects/subscribe`（B1）**：请求立即回全量快照，随后每 0.25 s（`SUBSCRIPTION_REFRESH_TIME`）
      推变化字段；连接关闭即退订，最后一个退订时定时器自停；与 `objects/query` 共用字段选择
      （`api/endpoints/objects_subscribe.rs`、`objects_query.rs`）。
- **reactor 抽象与定时器（A1）**：`Reactor` trait（`monotonic` / `register_timer` /
      `unregister_timer` / `call_later`）与 `TokioReactor` / `ManualReactor`；机器持
      `Arc<dyn Reactor>`，不拥有 runtime（`reactor.rs`、`docs/.../reactor.md`）。
- **主机层串起来**：`klippy_process`（建机器 → `api::register` → bind → `load_config`，
      失败即 `invoke_shutdown` → `bring_up` → `run`，重启类结果则就地重建）、`info` 端点、
      `StartArgs`（`src/klippy.rs`、`api/endpoints/info.rs`、`api/start_args.rs`）。
- **`run()` 的形态**：`bring_up()` async、`run()` 同步只等退出、机器只用
      `std::future::Future`，不起 tokio（`printer.rs`）。
- **配置驱动装载**：静态工厂表、两段式构造顺序、section 级合法性校验（`load.rs`）。
- **`[mcu]` 住户**：`McuObject`，section 到 connect 才解析、开设备、跑 identify，
      `get_status` 报 identify 快照（`mcu/object.rs`）。
- **对象表与只读端点**：`add_object` / `objects` / `lookup_object` / `status_of`、
      `objects/list`、`objects/query`，以及服务器侧的 `webhooks` 对象
      （`printer.rs`、`api/endpoints/objects_{list,query}.rs`、`api/webhooks.rs`）。

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
| 固件停机/重启事件 | `src/sched.c:310` `:318` `:351`、`klippy/mcu.py:813-835` `:880-881` |
| `config_reset` 与 restart helper | `src/basecmd.c:262-272`、`klippy/mcu.py:756-770` |
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
