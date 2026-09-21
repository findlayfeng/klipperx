# TODO — 主机（klippy）侧的待办与未决问题

本文件只记**打算做什么**和**还没定的事**，不重复已经定下的设计（那些写在各模块文档和
`docs/klippy/developer-manual/` 里）。括号里的 `klippy/xxx.py:NN` 指上游参考实现
（`third_party/klipper/`），用来在动手前核对行为。已完成的条目见文末「已完成（留档）」，
**每条只留一行索引**，细节在各模块自己的文档里；做完一件事就把它从正文挪进那张索引。

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
| G2b | 用 GCODE 控制 GPIO：`SET_PIN` 时序（数字与 PWM 均已可驱动） | C1 |
| G1b | gcode 调度器与上游的行为差异（ack / cmd_default / ECHO / mux 缺省…） | C1 |
| G4 | 运动命令（G0/G1/G28…） | G1、C1 |
| S1 | 压力测试工具（`klipperx stress`）剩余：stepper 资源、别名解析、端到端测试 | C1 |
| A2 | 错误词汇（`CommandError` / `ConfigError`） | — |
| B2 | MCU 剩余：`emergency_stop` 对象/端点、`last_stats`、错误载荷、本地 shutdown 标志、`command` 的固件 `reset` | — |
| B4 | 其余端点（estop / remote method / pause_resume / …） | G3 等 |
| F3 | `MCU_bus_digital_out`（命令队列/运动同步输出） | C1 |
| F6 | SPI 总线 | F1、F2 |
| F7 | I2C 总线 | F1、F2 |
| F8 | endstop / trsync | F1、F2、C1 |
| F9 | 输入与外设资源（buttons / pulse_counter / …） | F1–F7 |
| C1 | toolhead 与 kinematics | — |
| C2 | 配置装载收尾（option 校验、第二个住户） | — |
| D1 | 主机层 start args / rollover / `--logfile` | — |
| D2 | 重启循环剩余（`rpi_usb` 连接期门控、CRC 不一致的处理、重启后的输出订阅） | — |
| D3 | `command` 接管运行中的板子：RTO 定时重传 | B2 |
| E1 | 文档 | — |
| E2 | `python_path` 的取消 | 外部项目 |

### G2b 用 GCODE 控制 GPIO（现状与剩余）

数字与 PWM 两条链都已打通（见文末 F3 / F4 索引）：`[output_pin <name>]`
（`extras/output_pin.rs`）读 `pin` / `value` / `shutdown_value`（PWM 另加 `pwm` /
`cycle_time` / `hardware_pwm`），经 `PrinterPins::setup_digital_out` / `setup_pwm`
（`pins.rs`）建出 `McuDigitalOut` / `McuPwm`，并注册 mux 命令
`SET_PIN PIN=<name> VALUE=<0..1>`；运行时走立即路径（`update_digital_out` / `update_pwm`，
软件 PWM 对齐到周期边界）。剩下的是：

- [ ] **真板端到端验证**：`config.cfg` 加一段 `[output_pin <name>]` + `pin: <PAx>`，用
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
      （`klippy/klippy.py:216-220`）。带载荷的变体已就位
      （`KlippyEvent::KlippyNotifyMcuError` / `KlippyEvent::KlippyAnalyzeShutdown`；
      `analyze_shutdown` 已触发并传 `msg`），`notify_mcu_error` 的触发点待接入；当前
      MCU 错误路径的做法是把原因写进状态消息（上游放在 details 里）。
- [ ] **`command` 的固件 `reset`**：复位现在优先 `config_reset`（清配置），上游还会优先用
      固件的 `reset`（真重启 MCU，`HF_IN_SHUTDOWN`）。`restart_method == command` 的
      `firmware_restart` 已在**拆机之前**用活连接发 `reset`
      （`McuObject::before_firmware_restart`），但一般的配置握手路径还没有。
- [ ] **reset 期间没有本地 shutdown 标志**：现在靠「`configure` 完成后才 `bind_shutdown`」的
      时序规避；隐式、无测试，recv 一旦改成缓冲/异步就会把自发的 `emergency_stop` 误报成
      `MCU … restarted`。上游有 `_is_shutdown`（`klippy/mcu.py:893-895`）。

### G1b gcode 调度器与上游的行为差异

**为什么单列一条**：G1 的骨架（命令表 / `run_script` / 输出处理器 / 内置命令 / mux）已按
`klippy/gcode.py` 落地，逐行对照后还剩一批**行为差异**。一部分只能随前置模块（GCodeIO /
toolhead / 开放事件）一起补，一部分是现在就独立可补的小行为。命令表本身够通用，G4 运动
命令不必等这条。上游行号以 `third_party/klipper/klippy/gcode.py` 为准。

**随前置一起补（GCodeIO / toolhead / 事件）**

- [ ] **`GCodeIO` 未移植**：伪 tty / 文件输入整块缺失——fd 读取与 `partial_input`、
      `pending_commands` 批量与 20 条阈值、`M112` 乱序检测、`input_log`、debuginput EOF
      退出、`stats gcodein=`（`:390-494`）。现在输入由 API 层的 `gcode/script` 承担；要么
      明确不补（纯 API 主机），要么把串口/文件输入做成一个独立对象。
- [ ] **`ack()` / `need_ack`**：`GcodeCommand` 没有 `ack`（`:54-63`），这是文件输入协议的
      一部分。受影响的具体行为：`M115` 应该先 `ack(msg)`、失败才 `respond_info`（`:344-350`）。
- [ ] **事件**：错误分支不发 `gcode:command_error`（`:226`），重启不发
      `gcode:request_restart`（`:358`），debug 输入不发 `gcode:debuginput_exit`（`:433`）。
      事件总线（`KlippyEvent`）已就绪，待接入这些触发点。
- [ ] **`request_restart` 的停机前动作**：上游在 ready 时先 `toolhead.dwell(0.500)` +
      `wait_moves()` 再 `request_exit`（`:352-365`），随 **C1**；当前直接 `request_exit`
      （`gcode.rs:515-545`）。
- [ ] **`Coord`**（`:12-17`）：随 toolhead / kinematics（C1）。
- [ ] **handler 内部异常 → `invoke_shutdown`**：上游用裸 `except:` 兜底，报
      `Internal error on command:"X"` 并停机（`:230-232`）；Rust 无 panic 捕获。随错误
      词汇（A2）一起定。

**可独立补的小行为差异**

- [ ] **`default_handler` 缩水**（`:283-316`）：缺 `M105` → `ack("T:0")`、`M21`、
      `M140/M104` 且 `S=0`、`M107` / `M106`（S 关或 fileinput）这些「没有该模块时安静忽略」
      的抑制；也缺「命令名里带空格」时按 `realcmd = cmd.split()[0]` 路由到 `M117/M118/M23`
      的分支。后者是实际差异：`M117 123` 这类数字消息在 Rust 里会整串当命令名而报
      `Unknown command`（`parse_line` 只做 trim，`gcode.rs:862-917`）。
- [ ] **`ECHO` 前缀**：上游 `respond_info(commandline, log=False)` → 输出 `// <line>`
      （`:368-369`）；Rust 用 `respond_raw`，没有 `// ` 前缀、不记日志（`gcode.rs` 的 `ECHO`）。
- [ ] **`HELP` 未就绪提示**：上游未就绪时首行加
      `Printer is not ready - not all commands available.`，并遍历当前 active 表（`:379-388`）；
      Rust 无该提示，遍历 help 表（`gcode.rs:663-678`）。
- [ ] **`M115` 版本号来源**：上游取 `start_args['software_version']`（`:344-350`），Rust 用
      `CARGO_PKG_VERSION`。
- [ ] **`get_status` 的构建口径**：上游返回缓存的 `status_commands`、按 **active 表**构建
      （未就绪只列 base 的 8 条内置，`:176-184`）；Rust 每次从 `commands.ready` 全量重建
      （`gcode.rs:578-592`）。未就绪阶段 `objects/query` 看到的命令集合不同。
- [ ] **未就绪时停机不打印**：上游 `_handle_shutdown` 在 `not is_printer_ready` 时直接
      return（`:186-193`）；Rust 无条件发 `Klipper state: Shutdown`（`gcode.rs:335-343`）。
- [ ] **`is_traditional_gcode` 判定**：上游用 `float(cmd[1:])`（`:125-131`），Rust 只看首字母
      大写 + 次字符数字（`gcode.rs:794`）。`M1ABC` 这类上游拒绝注册、Rust 接受。
- [ ] **`parse_extended` 的 shlex 保真**：Rust 手写解析只做引号切换 + `#`/`;` 截断，不处理
      反斜杠转义 / 引号拼接等 `shlex` 语义（`gcode.rs:937-983` 对 `:266-281`）。
- [ ] **校验和 `*123`**：上游 `get_raw_command_parameters` 会剥掉尾部校验和（`:40-51`），
      Rust 的 `raw_parameters` 不剥（`gcode.rs:919-935`）。只在文件 / 串口输入路径上有影响，
      连同 `GCodeIO` 一起看。
- [ ] **`register_command(cmd, None)` 注销**：上游支持注销并返回旧 handler（`:133-141`），
      Rust 无注销、重复注册直接报错（`gcode.rs:325-350`）。
- [ ] **参数访问器缺口**：缺 `above`/`below`、`get_int` 的 `minval/maxval`、通用
      `get(parser=…)`；缺 `get_command_parameters` / `get_raw_command_parameters`（raw 只在
      内部 `Parsed`）；也没有 `create_gcode_command`（字段私有，外部无法构造 gcmd，宏类模块
      会需要）（`:23-91` `:244`）。
- [ ] **mux 缺省项（`value=None`）不可达**（**优先，含测试**）：`dispatch_mux` 用
      `contains_key(&None)` 认出缺省项，但键缺席时把请求值取成 `""` 再用 `Some("")` 查表，
      永远命中不了 `None`，于是走到「值不合法」错误分支（`gcode.rs:680-733` 对 `:317-342`）。
      实测：注册 `SET_PIN` 的 `PIN=None` 后执行 `SET_PIN VALUE=1`，报
      `The value '' is not valid for PIN. Options: `。当前库里只用 `Some(name)` 注册，未覆盖。
- [ ] **mux 错误提示的 `Did you mean`**：上游按 dict 迭代序取「最后一个匹配」（`:317-342`），
      Rust 对 values 排序后取第一个匹配（`gcode.rs:718-733`）——措辞更稳定，属有意偏离；
      要么对齐上游，要么在文档里记一句。
- [ ] **清理 `src/core/parser.rs`**：`parse_gcode` / `parse_gcode_line` 是未被引用的存根
      （`#[allow(dead_code)]`），真正的解析在 `gcode.rs`；删除或并入 `gcode.rs` 的测试。

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

### F MCU 基础资源（F3、F6–F9）

上游把这些叫 printer objects 下面的「资源」：主机用一个 **oid** 和一个 **pin 描述**
建立资源对象，把 `config_*` 命令攒起来，在 `finalize_config` 之前算一个 CRC 一次性下发，
之后用 `queue_*` / `set_*` / `*_transfer` 命令驱动。命令层（`allocate_oids` / `get_config` /
`finalize_config` / `get_uptime` / `emergency_stop` / `get_clock`）已就位，配置构建层（F1）
与 pin 解析（F2）也已完成，数字输出、PWM（F4）与 ADC（F5）三个 `config_*` 资源已落地
（见文末索引），剩下的缺口是命令队列/运动同步输出（F3）、总线（F6/F7）与 endstop（F8）。

#### F3 剩余：`MCU_bus_digital_out`

- [ ] `MCU_bus_digital_out`（`klippy/extras/bus.py:337` 以后）：挂在命令队列上、与运动
      同步的输出；需要命令队列/运动层（C1）。
- 运行期 `queue_digital_out` 收的是**绝对固件时钟**；print_time → clock 的换算属于时钟层
      （`cmd/clock.rs` 的 `ClockSync` 现只有 `get_clock`，偏移跟踪未做）。

#### F6 SPI 总线

上游 `MCU_SPI`（`klippy/extras/bus.py:42-155`）：

- [ ] 设备侧：`config_spi oid=%c pin=%u cs_active_high=%c`（或 `config_spi_without_cs`），
      总线侧：`spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u`，收发：
      `spi_send oid=%c data=%*s`、`spi_transfer oid=%c data=%*s` /
      `spi_transfer_response oid=%c response=%*s`；还有 `config_spi_shutdown`
      （固件 `src/spicmds.c:37` `:62` `:122` `:157`）。
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

循环本身与四种 `restart_method`（`command` / `arduino` / `cheetah` / `rpi_usb`）的物理分派
都已完成（见文末索引；方法与连接期门控见 `docs/klippy/developer-manual/mcu-config.md`）。
剩下的三块：

- [ ] **`rpi_usb` 的连接期门控（有意后置）**：① 串口不存在 → 先请求一次 firmware_restart
      去上电（`check_restart_on_attach`，`klippy/mcu.py:696-700`）；② 未配置时发配置前也先做
      一次 USB 断电（`check_restart_on_send_config`，`:692-694`），保证配置落在一块**本次会话
      断电重启过**的板子上（`Endstop_Phase.md`：rpi_usb 的意义就是断电复位，连未配置的板子
      也要先断电）。现在只在 `firmware_restart` 路径上「先复位、再 open、再 configure」，普通
      启动看到未配置的板子会直接发配置，不看 `restart_method`。
- [ ] **CRC 不匹配仍走就地复位（有意偏离上游）**：上游发现已配置但 CRC 不一致时先
      `request_exit('firmware_restart')`（`check_restart_on_crc_mismatch`，
      `klippy/mcu.py:678-685`、`:1057-1059`），让重启循环做物理复位；我们在 `configure` /
      `handshake` 里就地做：有 `config_reset` 直接清，只有 `reset` 时发 `reset` + 重连 + 重试
      握手。若要贴上游，还有 `start_reason == 'firmware_restart'` 却仍已配置时 raise
      “Failed automated reset” 的前置门（`:1053-1056`）。详见
      `docs/klippy/developer-manual/mcu-config.md`。
- [ ] **重启后的 g-code 输出订阅**：连接不断，`objects/subscribe` 也自动继续（它按名查新对象），
      但 `gcode/subscribe_output` 的处理器挂在被重建的 `GCodeDispatch` 上，重启后静默失效，
      要客户端重新订阅。上游靠 socket 重绑让客户端重连、重订阅；我们要么在客户端收到
      `klippy:ready` 后重订阅，要么把输出订阅表移到连接上。

### D3 `command` 接管一块还在跑的板子

`command` 的 `config_reset` 要连上才能发，而重连时对手的序号接着上一条会话走——这不是边缘
情况：`rpi_usb` 切不了 VBUS 的机器会当场回退到 `command`（`mcu/object.rs`），普通 `RESTART`
也只是重建对象、重新 open + identify，同样要接上一块没被复位的固件。

**传输层的接管已完成**：收发两端共用一个 4 位序号，接收任务把空帧的号报给发送任务
（重复 ack 即 NAK），发送端按「更大 → 采纳并换号重发未确认块 / 不更新 → 原号重发 / 否则
ack」settle，并用 `Mcu::took_over_session()` 让 `rpi_usb` 判断“有没有真重启”
（`mcu/mod.rs`、`mcu/object.rs`）。测试见 `docs/klippy/developer-manual/testing.md` 的
`mod.rs` / `host.rs`（含对着真 host 库的同进程二次连接）。

**还剩**：

- [ ] **RTO 定时重传**：帧丢了、固件也在静等时，现在只能靠对端再发 ack/NAK 触发。
- [ ] **固件 `reset` 优先**：见 B2 的剩余（现在走 `config_reset`）。

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

- [x] **Q2 事件系统的形状**：已选定大枚举：`KlippyEvent`（`src/core/klippy/event/`）
      覆盖上游全部 35 个事件名，声明分散在 `event/decl/`，由 `build.rs` 生成，`Unknown`
      兜底未声明事件名。设计见 [事件系统](docs/klippy/developer-manual/event-system.md)。
- [x] **Q3 `PrinterEvent` 是否恢复 `McuIdentify` / `AnalyzeShutdown` / `NotifyMcuError`**：
      随 Q2 一并解决。处理器签名改为 `Fn(&KlippyEvent)`，带载荷事件读变体字段；
      `mcu_identify` 与 `analyze_shutdown` 已触发，`notify_mcu_error` 待 MCU 错误路径提供
      `details`。原 `PrinterEvent` 已删除。
- [ ] **Q4 `get_status` 的返回形状**：`serde_json::Value`（贴上游、客户端零适配）还是
      typed + serde。
- [ ] **Q5 要不要反射式能力**：`lookup_objects(module)` 前缀遍历、`gcode_macro` 的
      `printer.objects`（`klippy/extras/gcode_macro.py:41`）。**部分已做**：F2 为了
      `pins` 加了 `Printer::lookup_object_as::<T>(name)`（单名取回具体类型）；前缀遍历仍未定。
- [ ] **Q6 退出结果的语义**：`"exit" / "error_exit" / "firmware_restart"` 由谁解释、
      `run()` 的返回值怎么变成进程退出码（`klippy/klippy.py:355-370`，`error_exit` 退 -1）。

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
| `klippy:notify_mcu_error` | MCU 通信出错时 | `msg: str, details: dict` | `klippy/klippy.py:144,151` | — |
| `klippy:analyze_shutdown` | 进入 shutdown 后分析 | `msg: str, details: dict` | `klippy/klippy.py:216-220` | — |

> **说明**：两个事件由带载荷的变体承载（`KlippyEvent::KlippyNotifyMcuError` /
> `KlippyEvent::KlippyAnalyzeShutdown { msg, details }`）。`analyze_shutdown` 已触发并传入
> `msg`，`details` 暂为空表；`notify_mcu_error` 待 MCU 错误路径提供 `details`。

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
| `gcode:command_error` | gcode 命令错误 | `gcode_command` | `klippy/gcode.py:226` | G1b |
| `gcode:debuginput_exit` | debuginput EOF | 无 | `klippy/gcode.py:433` | G1b |
| `gcode:request_restart` | 请求重启 | 无 | `klippy/gcode.py:358` | G1b |

> 依赖 G1b（gcode 调度器行为差异修复）和 GCodeIO（文件/伪 tty 输入）。

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

细节在各模块文档里；这里每条只留一行索引，最近完成的在前。

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
