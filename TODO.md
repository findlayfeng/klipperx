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
H1–H12 是上游 extras 里按域归并的消费者（2026-09-21 全量盘点的结果）。
**2026-10-06 复核**：本表与各节已按源码实测重新对账——2026-10-02~05 的大批落地此前未回写
（TMC 六型、传感器族、断料传感器、面板驱动、`delayed_gcode` 等），现已改为现状。核对口径是
「`section!` 声明 + `load.rs` 工厂表 + 命令/事件注册点」，条目里凡写「已落地」均指这三处实测命中。

**核心与架构**

| # | 事项 | 依赖 |
|---|---|---|
| C1 | 运动层收尾：轴/stepper 抽象 + 多轴（C1a）、extruder 运动（C1b）、运动学族（C1c：delta/polar/generic_cartesian/rotary_delta/winch/deltesian 全落地）、print-time 回调（C1d）——**✅ 收官** | — |
| C2 | 配置装载：框架部分 ✅（FW1，含 choice/range 文案与 `deprecate` 警告）；autosave/`SAVE_CONFIG` 写回 ✅（2026-10-02，详见下） | — |
| D1 | 主机层 start args / rollover / `--logfile` ✅（FW8）；`debuginput`/`debugoutput` 的命令行接线与每 MCU 字典路径**不做**（随 GCodeIO 决策，2026-10-05 结案；`debug_input` 字段与 `debuginput_exit` 事件已随死代码删除） | — |

**MCU 资源与总线**

| # | 事项 | 依赖 |
|---|---|---|
| F6 | SPI 总线剩余：`spi_transfer_with_preface` / `setup_shutdown_msg` | F1、F2 |
| F8 | endstop / trsync ✅（FW6）；测试侧「响应器式多实例假 MCU」✅（2026-10-03，`330c212`） | F1、F2、C1 |
| F9 | 固件资源剩余：buttons / trigger_analog / initial_pins / sdcard / lcd / tmcuart 等（已接：`cmd/thermocouple.rs` + `spi_temperature`；pulse_counter 批 #8 落地）。宿主侧已建：`extras/led.rs` 的 `led`/`neopixel`/`dotstar`/`pca9533`/`pca9632`、`extras/buttons.rs`（固件查询未接）、`cmd/trigger_analog.rs` 资源层（宿主节未建） | F1–F7 |

**G-Code 与端点**

| # | 事项 | 依赖 |
|---|---|---|
| G1b | gcode 调度器与上游的行为差异（`get_mutex` 等价物等；`GCodeIO` **不做**（2026-10-05 结案，`need_ack` 机制已删）；参数访问器与 `M115`/`Coord`/`request_restart` 已完成并归档） | C1 |
| G2b | 用 GCODE 控制 GPIO：数字/PWM 驱动与 `SET_PIN` **已落地**；「随打印时间生效的请求队列」（上游 `GCodeRequestQueue`）**已移植并接入 `output_pin`**；`pwm_cycle_time` 的 `SET_PIN` 已通过 `register_lookahead_callback` + `min_schedule_time` 自间隔调度（2026-10-03，`8a45d25`）；余项＝`output_pin` 的 `static_value`/`template`，以及 `heaters` 切到同一队列（`fan`/`servo`/`pwm_tool` 2026-10-03 已切换） | —（C1 已收官，不再是前置） |
| G4 | 运动命令（G0/G1/G28…） | G1、C1 |
| B4 | 其余端点（`*/dump_*` / …；estop、remote method、`pause_resume/*` 与 `bed_mesh/dump_mesh` 已落地） | G3、H4、H9 |

**上游 extras 消费者**（详见「上游 extras 覆盖盘点」）

| # | 事项 | 依赖 |
|---|---|---|
| H1 | 加热与温度（heaters / heater_bed / heater_generic / pid_calibrate / verify_heater / temperature_*）——**✅ 收官**（传感器族、`M105` gcode-id 表、`TEMPERATURE_WAIT`、`M109`/`M190` 等待、eddy 校准工具均已落地，见 H1 节） | — |
| H2 | 风扇与通用输出（fan / fan_generic / heater_fan / controller_fan / pwm_tool / static_* / multi_pin / servo / led / neopixel / dotstar / 电位器与 LED 驱动）——主体已落地；余项＝`replicape` 与 `pwm_tool` 的 `maximum_mcu_duration` 中间更新（见 H2 节） | F3、F4、F6、F7 |
| H3 | G-Code 宏与脚本（gcode_macro / delayed_gcode / respond）——三者主体**均已落地**（含单测）；`rename_existing` 连接期换名已落地（2026-10-06，`5ba2356`），余项＝`printer.objects` 的可枚举反射（Q5），见 H3 节 | G1b、Q5 |
| H4 | 打印流程与 SD 卡（virtual_sdcard / print_stats / display_status / pause_resume / exclude_object / sdcard_loop / firmware_retraction）——**均已落地**（`sdcard_loop` 的三条命令与 `SdCardFile` 接缝 2026-10-21），见 H4 节 | F9、C1 |
| H5 | TMC 步进驱动（tmc / tmc_uart / tmc2130…tmc5160）——六型（2130/2208/2209/2240/2660/5160）与 UART 框架均已落地；**活线与运行期链路 2026-10-18 补齐**（UART/SPI 读写、`tmc/stallguard_dump`、使能链路、相位跟踪、周期 `DRV_STATUS` 检查），余项＝both-edge 未启用与两条传输层行为偏移，见 H5 节 | F6、F7、C1 |
| H6 | 传感器与块状数据（bulk_sensor / 加速度计 / angle / ldc1612 / hx71x / ads* / load_cell / input_shaper / resonance）——`bulk_sensor` 框架、`adxl345`/`mpu9250`、`ldc1612`、`hx71x`、`ads1220`/`ads131m0x`、`load_cell`(+`_probe`)、`input_shaper`/`resonance_tester` 均已落地；余项＝`angle`、其余加速度计（lis2dw/lis3dh/icm20948/bmi160）、`ads1x1x`、`adxl345` 寄存器 setup+缩放，以及压力提前的三条余项与 `input_shaper` 未作用到运动（窗口机制已就绪），见 H6 节 | F5、F6、F7、C1 |
| H7 | 输入与外设（buttons / gcode_button / pulse_counter / trigger_analog / 断料与线宽传感器 / GPIO 扩展 / DAC）——`gcode_button`、`pulse_counter`、`filament_switch_sensor`/`filament_motion_sensor`、`trigger_analog` 三件套已落地；余项＝固件按钮查询与 `query_adc`、线宽传感器（hall_*/tsl1401cl）、`initial_pins`、板级模块（samd_sercom/replicape/palette2），见 H7 节 | F3、F5、F9 |
| H8 | LCD 显示与菜单（display/*）——面板驱动与固件命令均已落地（hd44780/st7920/uc1701/ssd1306/aip31068_spi/hd44780_spi，含各自的 `config_*`/send 命令）；余项＝菜单族（`menu.py`/`menu_keys.py`/`display.cfg`/`menu.cfg`）、个别驱动的 `set_glyphs`、`emulated_st7920`/`sh1106`，见 H8 节 | F9、G1b |
| H9 | 探测 / 调平 / 校准（probe / bltouch / bed_mesh / z_tilt / quad_gantry_level / bed_screws / …）——探针与其校准命令族、末端位置接口（`virtual_endstop_position`）、z_tilt/qgl/bed_tilt/screws_tilt/axis_twist/delta_calibrate、**bed_mesh 的 profile 命令族与探针触发时刻精度（2026-10-19）** 均已落地；余项＝`bed_mesh` 的 faulty 区域与 `set_zero_reference`、`skew_correction`/`z_thermal_adjust`/`tuning_tower`，见 H9 节 | C1、F8 |
| H10 | 运动相关 extras（gcode_arcs / manual_stepper / idle_timeout / motion_report / …；gcode_move、stepper_enable、motion_queuing、force_move、idle_timeout、GET_POSITION、动态 `axis_map`、`manual_stepper` 自有 trapq 出步、`motion_report` 两端点、**G2/G3 弧与 G17–G19 平面选择（2026-10-21）**已落地）——余项＝`manual_stepper` 回零、`extruder_stepper` 宿主同步缝、`motion_report` 的 `get_status`/shutdown 诊断，见 H10 节 | C1 |
| H11 | 主机运行时与调试（statistics / canbus_stats；`canbus_ids` 与 CAN 扫描已落地） | — |
| H12 | 核心工具补齐（mathutil / util 反射 / clocksync / pins 消费侧） | C1 |

**工具与文档**

| # | 事项 | 依赖 |
|---|---|---|
| S1 | 压力测试工具（`klipperx stress`）：**已收官**——真资源/别名/假板 e2e（含 `--task motion`，2026-10-21）均已落地；余项仅真板双机并发验证 | — |
| S2 | 文档补齐（2026-10-06 对账发现，非新功能）：`config.md` 缺 **`[bed_mesh]` 整节**（约 20 个选项，含 `split_delta_z` / `move_check_distance`）、`gcode-commands.md` 缺 `BED_MESH_*` 命令；开发手册「模块表 / `testing.md` 覆盖行 / `regression-tests.md`」的其余陈旧行未重审（本轮只修了与已落地改动强相关的那些） | — |
| S3 | 真机用例：把 `configure_taking_over`（“连接 ≠ 配置”，遇 `ResetRequired` 则 `reset` + 重开 + 重握手一次）**上提到 `hardware_test`**——现在三份需求（R3/R5/R11）共用一个局部实现（`mcu/resource/endstop.rs` 测试里 + `stress.rs` 自己的），上提后只留一份；同时把 L2（`Machine::bring_up`）已落地这件事写进框架 doc 的模板 | — |
| S4 | 真机用例继续铺（框架两层都已就绪）：R4 归零（无加热，建议先做）→ R9 收尾 / R7 挤出 / R6 温度 / R10 急停（**带加热的最后**）；R8 回放 | S3、Q11 |
| S6 | 运行时临时产物收口（**暂缓，2026-10-19 登记**）：`/tmp` 侧有 `kx-start.log`（每次手动拉 klippy 重新生成）、活连接文件 `klippy_uds` 与 Moonraker 的 `moonraker_instance_ids`、harness 的 `pi-bash-*.log`；仓库侧 `.staging/tmp-2026-10-19/` 存着 13 项旧产物（7 个 `klipperx-log-*` 运行目录 + 6 份闸门/启动日志，3.1 MB，已 gitignore）。要做的：按已定路径约定把日志改道（`klippy` 已有 `--logfile` 落 `<data-path>/logs/`，手动启动的 stdout 也应落那里），并给 `.staging` 定一个清理时机。判据：`klippy_uds` / `moonraker_instance_ids` 是**活 IPC 与第三方状态——不迁移、不进释放链**；`pi-bash-*.log` 归 harness；清 `.staging` 前先确认里面的日志不再作任何结论的证据 | — |
| E2 | `python_path` 的取消 | 外部项目 |

**串口复用 / 桥（跨主机与固件）**

| # | 事项 | 依赖 |
|---|---|---|
| M1 | 桥下游**端口模块化**与**没有 admin 通道的设备**共存：端口按 `[uart_device]`/`[can_device]` 定义、子设备引用端口节名，归属由主机经桥自己的 admin 通道告知桥（分六期：P1 配置面 → P2 固件端口表+端口级命令 → P3 attach 端到端 → P4 判定与传输正交 → P5 CAN 上游/复位 → P6 多站串口（RS-485，后置））。**CAN 下游端口（拓扑 B：CAN 当桥的一个端口，切片载具）已落地并真机验收**，见[协议页](docs/klippy/developer-manual/serial-mux-protocol.md)的 CAN 下游端口一节 | — |
| M2 | 通过 **admin 流程复位固件**（per-device reset）——**已落地**：`restart_method: admin` 在合法值表，桥自身走 `bridge_admin_restart`、下游端口走 `RESET_PORT`/`RESTART_DEVICE`（三传输都实现）；legacy 走端口节 `uart_restart_method`。余项只是真机极性与保持验证 | M1 |

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
      **余项**：`heaters` 无队列化消费者待切（`fan` `f2d9844`、`servo`/`pwm_tool` `e3b12a7` 已于 2026-10-03 切换）。
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

- [x] **stepper 资源（2026-10-06，`5ada3ce`）**：已改成复用真资源——每个压力 stepper 经
      `PrinterPins::setup_stepper` 建出正式 `McuStepper`（一次建一个 `PrinterPins` 注册表，
      每块 `[mcu]` 一个 `McuChip` + 自己的 `ConfigBuilder`），所以固件 `config_stepper` 带的是
      该节自己的 `invert_step`（步进脚前导 `!`）与 `step_pulse_ticks`（`step_pulse_duration`，
      默认 2 µs）；`--task motion` 的步距由 `microsteps`/`rotation_distance` /
      `full_steps_per_rotation`/`gear_ratio` 算出（无几何时退 0.01 mm）；本板自己的 `enable_pin`
      会被读出并置使能（共用线去重）。
- [x] **`[board_pins]` 别名（同上）**：别名经 F2 的正式装载器（`BoardPins` + `PrinterPins::parse_pin`/
      `resolve_pin`）解析；取反是**前导** `!`（`!PA0` / `!mcu:alias`；`mcu:!PA0` 是非法描述）。
- [x] **端到端测试（同上）**：假板（`test: dict=` 的 `SimulatorDevice`）上的 step ramp、comm ramp、
      `ResetRequired` 重连各有用例；另加一条解码线上 `config_stepper` 的用例（验 `invert_step`/
      `step_pulse_ticks` 真从节里来）。**假板 `--task motion` e2e 已落地（2026-10-21）**：假件按 oid 累积有符号步进位置
      （`set_next_step_dir` 换向、`queue_step` 累加、`reset_step_clock` 不清零，对齐 `stepper.c` 的
      `stepper_get_position` 语义），`a_motion_smoke_runs_end_to_end_against_a_fake_board` 断言读回步数 ==
      距离/`step_dist`（回退成恒答 0 即红）。**剩余真板项**：双机并发压测。

### G1b gcode 调度器与上游的行为差异（框架部分 FW4）

**为什么单列一条**：G1 的骨架（命令表 / `run_script` / 输出处理器 / 内置命令 / mux）已按
`klippy/gcode.py` 落地，逐行对照后还剩一批**行为差异**。一部分只能随前置模块（GCodeIO /
toolhead / 开放事件）一起补，一部分是现在就独立可补的小行为。命令表本身够通用，G4 运动
命令不必等这条。上游行号以 `third_party/klipper/klippy/gcode.py` 为准。

**随前置一起补（GCodeIO / toolhead / 事件）**

- [x] **`GCodeIO` 不移植（2026-09-21 暂缓 → 2026-10-05 结案：不做）**：伪 tty / 文件输入整块
      缺失——fd 读取与 `partial_input`、`pending_commands` 批量与 20 条阈值、`M112` 乱序检测
      （`m112_r` = `^(?:[nN][0-9]+)?\s*[mM]112(?:\s|$)`）、`input_log`、debuginput EOF 退出、
      `stats gcodein=`（`:390-494`）。现在输入由 API 层的 `gcode/script` 承担，客户端契约走
      Moonraker API；**结论：不做**（`need_ack`/`ack` 机制已于 2026-10-05 从 `gcode.rs` 删除；偏移登记在
      [本项目与上游的偏移](docs/klippy/developer-manual/upstream-deviations.md)）。
      以下为若要翻案时的最小路径与前置（存档）：① 先给 reactor 补 **fd 事件层**（本仓库只有定时器，无
      `register_fd`/`poll` 对应物），这是最贵的一块；② `util.create_pty`（`openpty` +
      `symlink` 到 `/tmp/printer` + 关 `ECHO` + 非阻塞）与 `GCodeIO` 对象；③ `is_fileinput`
      决定 `request_restart` / `_handle_shutdown` 是否退 `error_exit`（`:355` `:429`）；
      ④ `gcode:debuginput_exit` 需要 `send_event` 收集 handler 返回值（上游 `all(...)`）。
      tty 与 debuginput 共用同一套 `_process_data`，应一起做。
- [x] **`gcode:debuginput_exit` 触发（随 `GCodeIO` 结案：不做）**：上游 `_do_debuginput_exit`
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

- [ ] **G4-2 外围**：extra-axes 的 `axis_map` 已落地（`gcode_move` 随 `toolhead:update_extra_axes` 重建，`G1 A…` 按字母寻址并进入规划器；2026-10-18），
      额外轴出步也已落地（`manual_stepper` 自有 trapq，`MOVE=` 与 `G1 A…` 都真出步）。
      仍缺 **`Coord` 的定容上限**：`MAX_AXES = 8`（E + 4 个字母轴，第 5 个不映射）、
      `G1_PARAMS` 静态表不为额外字母提供补全、`reset_last_position` 的尾巴沿用上一段（toolhead 只报 4 轴）；
      上游 `ToolHead.manual_move` / `ToolHead.add_extra_axis` 的其余调用方（`toolhead:manual_move` 由 `safe_z_home` 发、
      `toolhead:update_extra_axes` 由 `ToolHeadObject` 发，见下表；`extruder:activate_extruder` 已由 `ACTIVATE_EXTRUDER` 发出）、
      `bed_mesh`（`set_move_transform` 已实现，`bed_tilt` / `exclude_object` 在用）。

### B4 其余端点（机制部分 FW9）

`api-reference.md` 有、`endpoints/mod.rs` 的表里标「not started」的其余部分，各自等它读的
对象先存在：

- [ ] **dump 端点家族残余**（mux 机制已就绪，实例随各自 extras 落地）。已落地：
      `bed_mesh/dump_mesh`、`motion_report/dump_trapq` 与 `dump_stepper`、`adxl345`/`ldc1612`/`mpu9250`
      （经 `bulk_sensor` 的 `add_mux_endpoint`）、`load_cell/dump_force`、`tmc/stallguard_dump`。
      仍缺：`load_cell_probe/dump_taps`（tap 事件未实现）、`icm20948`/`bmi160`/`lis2dw`/`angle` 四个
      dump（随未移植的 extras 缺席），以及 `motion_report` 的 `get_status`
      （`live_position`/`live_velocity`/`live_extruder_velocity`，现返回 `{}`）与其 shutdown 诊断窗口转储。

### F MCU 基础资源（F6、F8、F9）
上游把这些叫 printer objects 下面的「资源」：主机用一个 **oid** 和一个 **pin 描述**
建立资源对象，把 `config_*` 命令攒起来，在 `finalize_config` 之前算一个 CRC 一次性下发，
之后用 `queue_*` / `set_*` / `*_transfer` 命令驱动。命令层（`allocate_oids` / `get_config` /
`finalize_config` / `get_uptime` / `emergency_stop` / `get_clock`）已就位，配置构建层（F1）
与 pin 解析（F2）也已完成，数字输出、PWM（F4）与 ADC（F5）三个 `config_*` 资源已落地
（见文末索引），SPI/I2C 总线（F6/F7）也已落地（细节已归档），剩下的缺口是 endstop/trsync（F8，另有
一条测试侧待办）与其余固件资源（F9）；`MCU_bus_digital_out`（F3）能力已具备，不另做包装。

#### F5 ADC 采样时间（余项）

样本时间已于 2026-10-19 统一到 print time（资源层换算，与上游 `mcu.py:636-646` 同形）：

- [ ] **二级 MCU 没有 `get_uptime` 样本时 `recalibrate` 会发布垃圾 offset（2026-10-19 由时钟映射单元发现）**：
      `McuObject::recalibrate` 对二级 MCU 无条件跑 `SecondarySync::calibrate`，而它读
      `secondary.get_clock()`——`ClockEstimator::new` 把三个字段都清零，未 seed 时它返回 0，
      于是 `adjusted_freq = (0 - sync1_clock)/4 s` 为负、`set_mapping` 发出**错的 offset**
      （频率部分现在由 `time_mapping` 回落到字典频）。修法方向：`SecondarySync::calibrate` /
      `recalibrate` 加"已有样本"闸门。落点在 `mcu/object.rs`。

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

#### F8 endstop / trsync（与 C1 共享，框架 FW6a–FW6e 已落地）——**✅ 收官（2026-10-06 复核）**

endstop 资源（`mcu/resource/endstop.rs`，含 `minclock` 门控与 `home_wait` 纪元参考）、trsync、
`query_endstops` 与测试侧多实例假 MCU 均已落地，无余项。

#### F9 其他输入与外设资源

建立在 F1–F6 之上，各自一个 `config_*` + 查询/事件。这一节只列**固件侧资源**；
在它之上建的**宿主 extras 消费者**（buttons / pulse_counter / neopixel / sdcard / lcd /
传感器）按域归到 H5–H8，两边互为前置。

**已落地（2026-10-06 复核）**：`sensor_bulk` 框架（`extras/bulk_sensor.rs` + `cmd/` 各传感器命令层）、
`neopixel`/`dotstar`/`led`（宿主 `extras/led.rs` 六个 section）、`buttons` 宿主注册表（`extras/buttons.rs`）、
各类 SPI/I2C 传感器（`cmd/adxl345.rs`、`cmd/mpu9250.rs`、`cmd/hx71x.rs`、`cmd/ads1220.rs`、
`cmd/ads131m0x.rs`、`cmd/ldc1612.rs`、`cmd/sos_filter.rs`、`cmd/thermocouple.rs`、`cmd/ds18b20.rs`）。

- [ ] 固件 **buttons 查询**（`config_buttons` / `buttons_add` / `buttons_query` / `buttons_ack`）——
      宿主注册表已在，没有发送方。
- [ ] `initial_pins`（固件 `initial_pins.c` 是**编译期**行为，宿主不消费 `INITIAL_PINS` 常量）；
      `sdcard`/`sdio` **不是缺口**——上游 klippy 没有对应宿主模块（`sdiocmds.c` 只有固件侧，
      `virtual_sdcard`/`sdcard_loop` 是纯宿主实现）；`lcd_hd44780`/`lcd_st7920` **已落地**
      （`display/hd44780.rs`、`display/st7920.rs` 的 `config_*` 与 send 命令齐备）。
- 这些是 extras，不阻塞运动；消费者见 H5（TMC/tmcuart）、H6（加速度计）、
      H7（buttons/trigger_analog）、H8（lcd）。

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

- [x] **`StartArgs` 的剩余接线（2026-10-05 结案）**：结构体已带 `apiserver`（宿主启动时填，`src/klippy.rs:323`）、
      `start_reason`、`debug_output`、`device`、`linux_version`（`debug_input` 字段已随死代码删除）；`--debuginput`/`--debugoutput`
      的命令行解析**不做**（随 `GCodeIO` 决策：本仓不实现行协议输入与 batch mode，`StartArgs::collect` 里保持 `None`；
      `debug_output` 字段继续供语料 harness 使用），每个 MCU 的 `-d` 字典路径同样不做（只服务上游 `-o`）。
      `software_version` 已由宿主 `set_start_args` 注入
      （`src/klippy.rs:324`）并被 `info` 与 `M115` 读取——接线已完成并归档。
- [ ] **`StartArgs.start_reason` 已死**：上游主循环每轮 `start_args['start_reason'] = res`
      （`klippy.py:368`）、`mcu.py:682`/`:1060` 从字典读；本仓 MCU 改读
      `Printer::start_reason()`（`reset_for_restart` 每轮更新，语义一致），
      `StartArgs.start_reason` 停在 `"startup"` 且无人读。要么让它随重启更新（对外形状与上游一致），
      要么删掉该字段。

### E2 `python_path` 的取消（**远期，依赖外部项目**）

- [ ] **现状**：`info` 里这个字段只被 Moonraker 使用，且它把它当 Klipper 的
      virtualenv 解释器（`update_manager/app_deploy.py` 的 `_configure_virtualenv`），
      因此 `python_path` 目前只能报一个**不存在的路径**（让它的 Klipper 更新项退化为
      no-op），不能报自己的二进制（会让 Moonraker 报 `Invalid virtualenv` 起不来）。
      字段本身还必须存在：它直接下标取 `self._klippy_info["python_path"]`
      （`klippy_connection.py` 的 `_save_path_info`）。姊妹字段 `klipper_path` 已于
      2026-10-19 起报**真实存在的释放根**（`src/assets.rs`，见 api-reference）。
- [ ] **想做的**：这是一个 Klipper 实现细节，本主机没有解释器也没有 Klipper 源码树，
      报一个不存在（或任何）路径都是在编数据；理想是**取消**这个字段。
- [x] **（2026-10-18 对 Moonraker 源码复核；2026-10-19 随内置资产特性修订）** 两个键都**不能删**：
      `_save_path_info` 对两个键都是**无默认下标**（`klippy_connection.py:352-354`），`KeyError`
      不在 `:418-427` 的 `except ServerError` 内 → 初始化任务静默死、socket 连着但
      `klippy_state` 永远 `startup`、**不重连**。填真值的边界现在**分字段**：`python_path`
      **不能**填真值（真 venv 形状 → `update_manager/app_deploy.py:96-113` 抛
      `Invalid virtualenv` 起不来，非 venv 真解释器同理）；`klipper_path` **可以**填真值，
      但必须是**非 git 的释放根**——`_set_klipper_repo` 的 `AppType.detect` 只看
      `<path>/.git` 且**不看 env**，指向真源码检出会把更新项升成 `GitDeploy`（真
      fetch/reset/重启服务），而 `assets.rs` 释放的根恒无 `.git` ⇒ 仍是 `BaseDeploy`。
      回归守卫已随实现改名（`info.rs` 的
      `test_klipper_path_is_the_start_args_value_and_python_stays_absent`：`klipper_path`
      等于 `StartArgs` 携带的释放根、`python_path` 断言不得等于
      `CARGO_MANIFEST_DIR`/`current_exe()`）。只有当上游把那两处下标改成 `.get()` 时才能动。
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

### M1 桥下游端口模块化与「无 admin 设备」共存

设计、拍板点与六期开发计划原记在 `docs/work-log/2026-10-13-bridge-port-notes.md`（已删，内容已全部进手册；要回溯历史用 `git log -- docs/work-log/`）；现行行为以[串口复用协议](docs/klippy/developer-manual/serial-mux-protocol.md)、[配置手册](docs/klippy/user-manual/config.md)与[MCU 连接方式](docs/klippy/user-manual/mcu-connection.md)为准。

一句话范围：端口按 klipper 一贯的设备节形式定义（`[uart_device <名>]` / `[can_device <名>]`，
`i2c`/`spi` 复用已有节类型），子设备引用**端口节名**；`[bridge]`/`[bridge_port]`/`bridge_admin`
都不要（桥与端口从引用推算，legacy 从「无 `uuid:` + uart 端口」推算）；**归属由主机经桥自己的
admin 通道**用端口级命令 `SET_PORT_NODEID(index, id)` 告知桥，**下游设备完全不参与**——这就是
"与没有 admin 的设备共存"的答案。**依赖**：无；**状态**：✅ 配置面（P1）、固件命令族（P2）、attach 走端口两侧（legacy 与命名，P3）、固件端口对象化（P7）、端口路径运行期化（R1）、一进程一份 host 库（L1）、host 后端按槽多路流（R2）均已落地；**配置驱动的整链端到端已有真固件验收**——真 `klipper.elf` 独立进程当下游（单端口整链 + 双端口同桥并发，覆盖见 [`testing.md`](docs/klippy/developer-manual/testing.md)）。**余项**：L3（linux 控制台接 mux + `-U` 身份）✅、P4 判定与传输正交 ✅（规则 7 显式计入下游台数）、P5 CAN 上游 **部分落地 + 余项挂「待测（硬件）」**（主机侧端口命令的 CAN 单元发送 API + 固件 `MUX_CAN` 载体 + 编译覆盖目标已落；`[can_device]` 下游端口后端 `BRIDGE_PORT_CAN`、CAN 上游 attach 端到端、`vcan` 夹具——**归硬件待测，不阻塞进度**，见 [`TESTING.md`](TESTING.md) 的 M1-4；CAN **下游端口**（拓扑 B，切片载具）已落地并真机验收，见[协议页](docs/klippy/developer-manual/serial-mux-protocol.md)的 CAN 下游端口一节）、P6 多站串口（RS-485）**后端尚未实现**（不是「等硬件」）：主机侧的多站分组已在（`port_flags` + 协议页规则 10），但固件的半双工收发与方向控制、以及宿主侧对应支持**零代码**（`TESTING.md` 自陈；全树无 rs485/half-duplex/direction 实现）——需先写完才谈得上真机验证，且需 485 设备才能验收。**已知缺陷**（留档、不阻塞）：原生 USB CDC 的 legacy 设备没有经桥硬复位手段（CDC 感知方案已被用户否掉，见 [`TESTING.md`](TESTING.md) 的 M2-1）；同进程两份 host 库无法一起退出——已按「一进程一份库」规则化并落两道守卫（配置层 + 运行期按路径），见开发手册模块表的 `host_libraries.rs` 行。（原「命名设备在宿主库桥上握手后库不卸载」的诊断已被推翻：真因是测试用默认 current-thread 运行时，见 `testing.md` 的 `restart.rs` 行。）**真机项**：[`TESTING.md`](TESTING.md) 的「M1/M2」一节。**测试策略（2026-10-14）**：真机/外设项后移——先让全部代码在软件侧说得通，等 M1+M2 落地后统一跑一次真机，清单见 [`TESTING.md`](TESTING.md) 的「M1/M2」一节。

**工程余项（不阻塞，已排期）**：

- **in-place reset 路径内的 child 失败不走恢复**：桥端口 tty 丢失的两条触发（桥自身 bring-up 失败、child 连接期间
  被 `0x05` 重启）都已落有界恢复（`a359eef`），但 `reconnect` / `McuObject::handshake_with_in_place_resets` 里的
  child 失败仍直接报错。
- **恢复预算不归还的边角**：一次恢复请求后若重启始终连不上，该 MCU 的恢复预算不再归还（此后再丢端口不会自动
  恢复），需人工 `FIRMWARE_RESTART`。
- **`a359eef`（child 侧恢复）尚未独立评审**：它是同一套已被评审过的模式的小扩展，列入里程碑末的统一评审。

**其它留档缺陷（不阻塞、非本期范围）**：

- **宿主库的「卸载完成」不可观察**：`wait_for_unload` / `library_unloaded()` 只看到注册表的 `Weak` 变死，**不等**
  `HostLibrary::drop` 里的 `dlclose` 尾巴；「镜像真的重新映射」由 `test_the_library_unloads_with_its_last_reference`
  的会话接管断言钉住。个别 plain-library 测试的偶发 flake（真实首因仍待有界复现）候选出在这里。
- **新主机配旧库没有版本握手**：`config_uart` 比旧库多一个参数 ⇒ 连接期按字典参数个数校验直接报错
  （`expected 2 parameters, got 3`），文案不友好但不会静默走错路径。
- **子模组 `scripts/check_whitespace.sh` 的基线不干净**（`host/` 下多处 >80 列），与本轮改动无关。

### M2 通过 **admin 流程复位固件**（per-device reset）——已并入 M1

复位与端口级命令是**同一套机制**
（都分"面向设备"与"面向端口/桥"两条，现行形状见[串口复用协议](docs/klippy/developer-manual/serial-mux-protocol.md)的命令表
与[MCU 连接方式](docs/klippy/user-manual/mcu-connection.md)的 `restart_method` 节）。✅ 固件命令族已落地
（2026-10-13）：`REQUEST_BOOTLOADER 0x02 [0x02][index]`（进引导，刷写工具用）与 `RESTART_DEVICE 0x05 [0x05][index]`（重启固件）都只转发到该口、`RESET_PORT 0x04
[0x04][index]` 由桥做 DTR/RTS 线脉冲（poll 状态机非阻塞），均无回执；✅ **主机半边也已落地**（P3-D）：命名设备用 `restart_method: admin`（命令经桥转发 `RESTART_DEVICE 0x05`），legacy 由**端口节** `uart_restart_method`（只认 `arduino`/`cheetah`，线级）决定走 `RESET_PORT`；无回执 ⇒ 沿用「复位后重开+重连」确认；旧固件/非桥由 `resolve` 的点名错误自然降级为 `command`。
真机项（复位极性与保持、双板接线）**已按统一策略后移**，见上面 M1 段的测试策略。

范围一句话：今天的复位只有线级手段（`arduino` 的 DTR、`cheetah` 的 RTS、`rpi_usb` 切电），在共享/mux
路径上会打断整条线因而被配置层强制成 `command`，而 `command` 不是真复位；目标是让"点名复位某台设备"
（或由桥替它做线级复位）成为可能，**无 uuid 的 legacy 设备只能走"桥替它做"**那条。

## 上游 extras 覆盖盘点

上游 133 个 extras（顶层 `*.py` 不含 `__init__.py`、不含 `display/` 子目录；2026-09-23 实数复核
✓）里，本仓库已落地的远超最初盘点：`bus`（SPI/I2C）、`fan` 族、`gcode_move`、`stepper_enable`、
`query_endstops`、`error_mcu`、`static_digital_output`、`ds18b20`、温度传感器族、
`heaters`/`heater_bed`/`heater_generic`（含等待 / 校准面）、TMC 六型与 `tmc_uart`、
`bulk_sensor` 与加速度计、`input_shaper`/`resonance_tester`、`virtual_sdcard`/`print_stats`/
`pause_resume`、`filament_*_sensor`、面板四驱动、`led`/`neopixel`/`dotstar`、`delayed_gcode`/`respond`；
未落地的按域归并成 H1–H12。逐模块的完整对照表
（含固件命令模块、端点、判为不适用者）随覆盖审计收口清理；落点就是下文 H1–H12
与 F/G/B 各节。

### H1 加热与温度

- [x] **`TURN_OFF_HEATERS` 与固件加热看门狗（2026-10-18）**：`TURN_OFF_HEATERS` 已注册（ready 表，文案同上游 `heaters.py:319`），遍历全部 heater 置 0；`setup_max_duration` 改回上游的 `MAX_HEAT_TIME = 3.0`（此前是 0.0，宿主挂住时固件不拿口）。**`set_pwm` 的两半已于 2026-10-19 全部到位**：`target_temp <= 0 ⇒ 0` 与 `read_time > verify_mainthread_time ⇒ 0`（截止戳 = 1 s 定时器打的 PWM 芯片 print time + 5 s），另补了 `get_temp` 的 7 s 陈旧压制（`QUELL_STALE_TIME`）——两者都要求读数与 print time 同域，由同批的 ADC 样本时间统一解锁。

- [ ] **`MAX_MAINTHREAD_TIME` 护栏在真实加热器上不可达（2026-10-19 真机结论，留作记录）**：
      klipper 的加热器一律软件 PWM（上游 `heaters.py:61` 也写死 `setup_cycle_time(pwm_cycle_time)`），
      每 0.1 s 需要一个新边沿；宿主一停，固件先报 `Missed scheduling of next digital out event`
      （实测 `SIGSTOP` 6 s 即触发）。因此 5 s 的主线程截止对加热器只是纵深防御，7 s 陈旧压制同样
      只有恢复后第一条读数之前的几毫秒窗口。两者边界由单测钉住，归零时有限速告警可查。若要别的
      场景真验，只能给 `[heater_generic]` 加 `hardware_pwm` 选项（上游没有这个选项，属新增行为，
      需先拍板）或构造「读数在跑、打戳任务被独占」的阻塞段。

- [x] **`M105` / gcode_id 表**（heaters 域）：`TEMPERATURE_WAIT` 已落地（2026-10-03，`a802cdc`，
      10 测）；`M105` gcode-id 表已接（2026-10-03，`c1fecac`）：`register_sensor` 存 `gcode_id→sensor` 表（`BTreeMap` 排序），
      `M105` 输出 `<id>:%.1f /%.1f` 列表（空表/未 ready 回 `T:0`），`TEMPERATURE_WAIT` 等待循环每轮回真报表（5 新测）。
- [x] **传感器全部收官**：`EddyCalibrationTool` 已移植（2026-10-03，`a6004f7`：`PROBE_EDDY_CURRENT_CALIBRATE` + `Z_OFFSET_APPLY_PROBE`，校准移动采样 + 噪声过滤 + `configfile` 写回；`note_z_calibration_*` 唯一调用方到位）；`temperature_probe.py` A+B+C 已落（`19e7eb4` / `65866da` / `1f0d623`；前置 `TEMPERATURE_WAIT` 已随 `a802cdc` 落地）。装载序均已修复：`thermistor` 批 #22、`adc_temperature` `9391bb4`（都声明 `phase = early`）。已落地并归档：`temperature_host`（2026-10-03，`07e65b4`）/ `temperature_sensor` / `thermistor` / `adc_temperature` / `spi_temperature`（MAX6675/31855/31856/31865） / `temperature_combined` / `temperature_mcu`（T7）；`temperature_fan` 无需再排（2026-10-03 核对：选项/双控制环/命令/get_status 逐项与上游一致，11 单测 + 3 条语料用例全绿；速度抑制窗未移植属低危可选）。
- 依赖 F4（PWM）、F5（ADC）、F6（SPI 温度）；C1 已收官。

**本节收官（2026-10-06 复核）**：`M105`/`TEMPERATURE_WAIT`/`M109`/`M190` 与传感器族均已落地，
无余项；后续新传感器（`angle` 等）归 H6。手册同步见加热器 / 温度传感器各页。

### H2 风扇与通用输出

- [ ] **余项**：`replicape.py`（语料 1 处，另见本文件「特定板/芯片」——回归里唯一的静态缺节）。
      **已落地**：`fan`/`fan_generic`/`heater_fan`/`controller_fan`（含 heater 注册表 `lookup_heater`，批 #20）、
      `tachometer_pin`→`pulse_counter`（批 #8）、`pwm_tool`/`pwm_cycle_time`/`servo` 模块本体与队列化、
      `multi_pin`（批 #26）、`sx1509`（批 #41）、电位器/DAC 驱动族 `mcp4018`/`mcp4451`/`ad5206`/`dac084s085`、
      `led`/`neopixel`/`dotstar`/`pca9533`/`pca9632`（`extras/led.rs` 六个 section）、
      `static_pwm_clock`（2026-10-18）、`duplicate_pin_override`（2026-10-18）、
      `SET_PIN`/`SET_FAN_SPEED` 的 `TEMPLATE=`（共享 `template_evaluator`，2026-10-18）。
      `pwm_tool` 另缺 `maximum_mcu_duration` 的**主机侧**中间更新（上游 `MCU_queued_pwm._gen_intermediate_updates`；
      固件 `queue_pwm_out` 已在），见 `pwm_tool.rs` 模块文档。

### H3 G-Code 宏与脚本

- [ ] `gcode_macro.py`：段与宏注册**已落地**（2026-09-24 集成批 #1，语料绿）；**`rename_existing` 连接期换名已落地**
      （2026-10-06，`5ba2356`：装载期只做同型检查、连接期经 `PrinterObject::connect` 把 alias 的旧处理器位移到
      `rename_existing` 名下、宏再接管 alias；缺命令报上游文案，换名目标已被占用时按 `register_command` 报错
      ——上游 `gcode.py:142-144` 同款，**不**先 unregister 静默顶掉）。剩余 = 读 `printer.objects` 的反射式能力（**Q5**）。**U-A7b 已归档**（2026-09-24 批 #4 起的「受控子集引擎」阶段：`exclude_object.test`+`dual_carriage.test` 双翻转、guard 归零，`{% set %}` 批 #9、过滤器参数与 `default`/`float` 批 #17、列表字面量与 `|min`/`|max` 批 #24 相继落地）。**2026-09-29 引擎换为 minijinja 2.24 适配层**（`custom_syntax` 单花括号定界符、Strict undefined、装载期编译与求值分两段）：原子集外的 `namespace()`、关键字实参、`{% block %}`、`|float(默认)` 由此接上（语料 160 绿）。仍缺：读 `printer.objects` 的反射式枚举（**Q5**）；三元 `x if c else y` 由 minijinja 原生支持但本仓未单列测试。**2026-10-18 三轮补齐**（以 Mainsail 客户端宏文件为语料）：① `%` 字符串格式化——编译期把「左侧是字符串字面量」的 `"…%s…" % x` 重写成 `|format(...)`（`{ … }` 与 `{% … %}` 都盖，`{% if n % 2 == 0 %}` 这类数值取模不动，`%(key)s` 映射形态仍不支持）；② 模板内方法调用——装了 unknown-method callback，`str` 的 `split`/`strip`/`replace`/`startswith`/`endswith`/`join`/`find`/`count`/`lower`/`upper` 与 `list` 的 `index`/`count` 是真实现，**`append`/`extend`/`pop` 因值语义只能降级**（返回 `None`、列表不变，收集写法拿到空列表）；③ 标签输出容器的 formatter（按 Python `str()` 拼写）。与 Jinja2 的已知差异（`%`/`//` 欧几里得取余、Strict 下缺键在打印/迭代/判真时报错、`~` 拼接容器不走 formatter、列表不可变、部分 detail 措辞）见 `template.rs` 模块文档。iqex/itex 的模板阻塞已消，首因前移到 `dual_carriage` 的 `primary_carriage`。
- [x] `delayed_gcode.py`（`[delayed_gcode]`）**已落地**（2026-10-06 复核：`extras/delayed_gcode.rs` 已装进
      工厂表 `load.rs`，`initial_duration` / `UPDATE_DELAYED_GCODE` / `klippy:ready` 注册齐全，8 条单测）。
      上游 `test/klippy/` 无对应 `.test`，语料不覆盖。
- [x] `respond.py`（`RESPOND` / `M118`，批 #29）。
- 前置：**G1b** 的 `create_gcode_command` 与参数访问器（宏类模块要构造 gcmd）。
- [ ] 已知偏差（2026-10-18 审出，客服端宏会踩）：无 `[extruder]` 时上游 `printer.toolhead.extruder` 是 `''`（`DummyExtruder.get_name()`），本仓恒为 `"extruder"`，于是 `_TOOLHEAD_PARK_PAUSE_CANCEL`/`_CLIENT_EXTRUDE` 那类 `!= ''` 守卫在本仓拦不住、直接去查未注册的 `printer['extruder']`。有挤出机时两者一致。
- [ ] 上游 `toolhead.py:604-614` 的无条件加载表里，本仓 `DEFAULT_MODULES` 只收 `idle_timeout`
      （`gcode_move`/`manual_probe` 由 toolhead 工厂 `ensure` 无条件建立，不经该表）。逐项现状：
      `tuning_tower` **真缺**（无文件、无 `TUNING_TOWER`）；`statistics` 只缺 1 Hz 汇总日志（对象已落地，
      见 H11）；`garbage_collection` 只缺日志汇总（Rust 无 GC 冻结机制，只有延迟告警）；
      `homing` **不是缺口**（功能在 `motion/kinematics.rs` 的 `Homing` 与 toolhead 内，只是没有同名模块）。

- [x] **归零丢报告不再锁死（2026-10-18 真机）**：`trsync_state` 路由器提前到会话 `attach` 安装、无主 oid 静默、
      `on_restart` 发 `trsync_start(oid,0,0,0)` 断掉被杀进程留下的周期报告；等触发加 5 s 兜底
      （超时先 `dispatch.stop()` 再 `Terminal during homing` 式失败），失败路径释放 gcode 锁。
- [x] **TMC 相位偏移（2026-10-18 真机）**：首因不是 UART 读失败，而是 `probing_move`/`flush_step_generation`
      把 toolhead 的 `Connected` 整个 `take()` 出去，使能回调恰在其间取不到 `mcu_position`（53 µs 早退）；
      改取 solver 位置后与上游一致。
- [ ] **RESTART 对活固件做 takeover**（2026-10-18 真机）：`M112` → `RESTART` 会 identify+takeover 仍在运行的固件，
      串口偶发 `Input/output error` 并重建 `Mcu`（曾连带 trsync 绑定丢失）。上游 `RESTART` 不重置固件、只重发配置——
      本仓应在会话仍健康时复用它，把「重置」留给 `FIRMWARE_RESTART`。
- [ ] `Timer too close`（真机只出现过一次）：前面 5 s 内有一次 `Reactor busy 0.195s`（TMC 单线读同步阻塞反应堆），
      归到「使能路径不得阻塞反应堆」的验收里继续观察。
- [ ] `klipperx mainsail` / `klipperx moonraker`（2026-10-18）：**便利启动**通道，不是生产部署；打包策略暂定
      （Mainsail 只带 `dist/`，Moonraker 走子模块源码 + 自管 venv），将来若要发布再定。

### H4 打印流程与 SD 卡

- [x] `virtual_sdcard.py`：主机侧文件打印、`M20`–`M27`/`SDCARD_RESET_FILE`/`SDCARD_PRINT_FILE`、`work_handler` 回放、进度（**2026-10-02**；省略 `gcode.get_mutex().test()` 让出、`_handle_analyze_shutdown`/`_handle_debuginput_exit`/`stats`，`path` 不做 `expanduser`/`normpath`）。
- [x] `print_stats.py`（**2026-10-02**；省略 `_handle_activate_extruder`，事件未 fire）、`display_status.py`（`M73`/`M117`）。
- [x] `pause_resume.py` 的节、四条命令与 `pause_resume/*` 三个端点（批 #15 + 2026-10-03）；命令表改握 `Weak`（2026-10-18）：它与 `[virtual_sdcard]` 同场时曾成环 `dispatcher → 命令表 → pause_resume → sd 卡 → dispatcher`，跨 `teardown` 钉住整张表（连带各 `Mcu`）——单测 `teardown_releases_the_sd_card_and_pause_resume` 钉死。
- [x] `exclude_object.py`（段+四命令落地，**2026-09-24 批 #4 随引擎转绿**，含排除区 E 补偿）、`firmware_retraction.py`（G10/G11，**批 #32 已落地**）。
- [x] `sdcard_loop.py` —— **已落地（2026-10-21）**：三条命令在 `sdcard_loop::load_config` 注册（`COUNT` 照上游为必填、`minval 0`；上下文错误文案逐字同上游）、
      `virtual_sdcard` 实现 `SdCardFile`（`file_position`= `next_file_position`，`loop_end` 经 `set_file_position` 回卷），
      回放环在派发 `BEGIN` 行前把游标推到循环体首行。三条回放级测试钉住 `COUNT=2` 执行两遍、空栈、上下文规则。
- 依赖 F9（固件 `sdiocmds.c` 的 sdcard 资源）、C1（`gcode_move` 的位置恢复）。

### H5 TMC 步进驱动

**主体已落地（2026-10-06 复核，2026-10-18 补齐活线）**：公共框架（`extras/tmc.rs` + `tmc_spi.rs`
的寄存器/字段表、`DUMP_TMC`/`SET_TMC_*`）与六型驱动都在工厂表里——`tmc2130`（批 #34）、
`tmc2208`、`tmc2209`、`tmc2240`、`tmc2660`、`tmc5160`（各带 `section!`），`tmc_uart` 资源层在 `cmd/`、
`extras/tmc_uart.rs`。

- [x] **`stallguard_dump` 查询**（`tmc/stallguard_dump` 端点，2026-10-18）：`TmcStallguardDump` +
      `BatchBulkHelper`，六型按能力选读路径（`DRV_STATUS.sg_result` / `SG_RESULT` / `SG4_RESULT`）；
      节拍统一 5 ms（上游用 `spi_status` 猜出的 `optimized_spi` 分 1 ms/5 ms）。
- [x] **`tmc_spi` 的总线共享**（2026-10-18）：多设备链复用首个节的 `McuSpi` 与一把 `busy` 锁；
      后续节仍会发一次被丢弃的 `config_spi`/`spi_set_bus`（为了走选项与片选校验），与上游共用同一个
      pin 对象不同，但帧与互斥已共享。
- [x] **`FORCE_MOVE` / `STEPPER_BUZZ` 真机不出步 —— 已归因并修掉前提（2026-10-18）**：假固件上
      host 侧**本来就没有缺口**（`FORCE_MOVE 10 mm@100 mm/s` 生成 +800 步、`STEPPER_BUZZ` +800/−800），
      `manual_move` 的四件（换 solver / 挂专用 trapq / flush 到生成并发送 / 不碰规划器）都在。真机上
      「命令成功但 `MSCNT` 不动」的三个成因分别是：**A** 电机根本没使能（EN 未开或 TMC `toff=0`，已由
      自动使能链修掉）；**B** `[manual_stepper]` 未装队列（已由自有 trapq 修掉）；**C** 队列外电机
      **静默空跑**（推进时间轴、返回 Ok、0 步、无日志）——C 已改成 `warn!`（不报错，见下一条）。
      **仍需真机复验**：`MANUAL_STEPPER MOVE=` / `FORCE_MOVE` / `STEPPER_BUZZ` 三条各自的 `queue_step`。
- [ ] **IDEX 第二滑车根本没有步进生成**（2026-10-18 发现，真缺口）：`idex_modes` 给 `[dual_carriage]`
      建的 `PrinterStepper` 从不 `take_stepper()`、也从不进 `MotionQueuing`，它的 solver 没挂任何 trapq
      ⇒ 不只是 `STEPPER_BUZZ` 不响，这一路**整条驱动链**都不出步。上游每个 `MCU_stepper` 建时即
      `allocate_syncemitter`（`klippy/stepper.py:50-56`），所以 IDEX 机器在那里是正常的；本仓语料此前
      是**假绿**（只要求 `STEPPER_BUZZ STEPPER=dual_carriage` 不报错）。修法参考
      `manual_stepper::ensure_motion`（`take_stepper` → `add_extra_axis_stepper` → 装进队列入生成集），
      但要照顾 kinematics 那一侧（carriage 在 IDEX 里是主 kinematics 的一部分，不是「自有 trapq」那种
      独立轴），属跨 kinematics/queuing seam 的独立单元。到那时 `force_move` 的 `warn!` 可收掉。
- [x] **`admin` 工具的 attach 没有会话 takeover —— 已修（2026-10-18）**：接管本来就是 klippy
      identify 的能力（`mcu/mod.rs` 的重号重发），缺的是「握手无应答就再也走不到 identify」。现在
      重试耗尽后改为「不握手开链路 → identify 接管陈旧会话 → 同一 handle 上补做分配握手」；沉默板子
      仍报 `AttachUnanswered`，只有「板子在应答另一个连接」才报 `SessionHeldByAnotherProcess`
      （点名「通常是 klippy 在驱动，先停掉它」）。
- [x] **失败态后的 `SET_KINEMATIC_POSITION` 会撞 stepcompress —— 已修（2026-10-18）**：首因是**生成
      地平线被回退**——drip 已把步生成到移动末端，而规划器 `print_time` 还停在 drip 起点；失败回零
      没有上游 `homing_move` 的收尾三步（`get_last_move_time` → flush → `set_position(haltpos)`），
      后台 flush pass 又用自己那份 `last_step_gen_time`（0.7）把 `itersolve.last_flush_time` 拽回去、
      重走 drip 段，重走出来的步落在 compressor 的 `last_step_clock` 之前 ⇒ `check_line` 报
      `Invalid sequence`。修法：地平线起点取 `max(两处)`，并在未触发分支补上游的收尾（`&[]` 不标
      homed）。仍待真机复验一次。
- [ ] **both-edge（`dedge`）**：宿主侧不选用——`PrinterStepper::get_pulse_duration()` 第二项恒 `false`，
      `mcu/resource/stepper.rs` 只发 `invert_step` + `step_pulse_ticks`，所以 TMC 从不写 `dedge`。
      **固件是支持的**：`src/stepper.c` 声明 `STEPPER_STEP_BOTH_EDGE=1`、`invert_step < 0` 走单定时器路径，
      `Kconfig` 的 `WANT_STEPPER_OPTIMIZED_BOTH_EDGE` 默认 y。缺的是主机侧按
      `STEPPER_STEP_BOTH_EDGE`/`STEPPER_OPTIMIZED_UNSTEP` + 脉宽算出 `want_both_edges`
      （上游 `stepper.py:86-113`）并未 `invert_step=-1`。属优化项，不影响正确性。
- 依赖 F6/F7、C1（stepper 对象）；`tmcuart` 固件资源仍在 **F9** 名下。

### H6 传感器与块状数据

**已落地（2026-10-06 复核）**：`bulk_sensor` 框架（`extras/bulk_sensor.rs`，被 `adxl345`/`ldc1612`/
`load_cell` 共用，固件命令层在 `cmd/`）、加速度计 `adxl345`/`mpu9250`、`hx71x`、`ads1220`、
`ads131m0x`、`ldc1612`、`load_cell` + `load_cell_probe`、`input_shaper` + `resonance_tester`
（`shaper_defs`/`shaper_calibrate` 的算式在本仓 `shaper_defs.rs` 与 `input_shaper.rs` 内）。

- [x] **load-cell 探测三线接线（2026-10-06，`9a11bf8` + `6251675` + `37698cc`）**：
      `[load_cell_probe]` 在 `klippy:mcu_identify` 把 Z 轨电机注册进自己的 trigger dispatch（对齐上游
      `probe.LookupZSteppers`）；建完 trigger 后调传感器 `setup_trigger_analog`（三型 init attach，
      对齐 `trigger_analog.py:300-301`）；注册 `probe` pin chip，`endstop_pin: probe:z_virtual_endstop` 可用
      （对齐 `load_cell_probe.py:698`）。余下：
  - [ ] **真机 PROBE 用例**（触发即停 + 无 MONITOR）——本机无 load cell，接线后才写得安全且有意义；
  - [ ] **多 MCU 共享 Z 轴**：`add_stepper` 会拒，上游报配置错，本仓因事件总线隔离处理器而只能
        `tracing::error!` 后继续 bring-up（已登记为与上游的差异，见 `upstream-deviations.md`）；
  - [ ] `dump_taps` / `LOAD_CELL_TEST_TAP` 仍未接（`load_cell_probe` 的探测结果族）。

- [ ] **`angle`**（磁编码传感器）——无文件、无 section。
- [ ] **其余加速度计**：`icm20948`、`lis2dw`、`lis3dh`、`bmi160`（固件 `sensor_icm20948.c` /
      `sensor_lis2dw.c` / `sensor_bmi160.c` 已有，宿主未建）。注意 **`lis3dh` 不需要单独的固件支持**：
      上游 `lis3dh.py` 复用 `lis2dw` 的类型分支，`Kconfig` 也把两者合在一个选项。
- [ ] **`ads1x1x`**（`ads1x1x.py`）。
- [ ] **`adxl345` 的寄存器 setup / `_convert_samples` 缩放**（`adxl345.rs:38-40` 自述）。
- [x] **`SET_PRESSURE_ADVANCE` 作用于运动 —— 已落地（2026-10-21）**：ESPA 平滑按 `kin_extruder.c` 的解析积分实现（`motion/pressure_advance.rs`），
      参数是按打印时间的表、逐段查表（改值不改写已生成位置），`SET_PRESSURE_ADVANCE` 写入共享的
      `Arc<ExtruderPressureAdvance>`；`pa = 0` 有逐点回归保护。同批修了两个既有静默错：挤出机 stepper
      被 `add_stepper` 重指回主 trapq（从未读自己的）、`ToolHead::set_position` 把 toolhead 坐标写进 extra axis。
- [ ] **压力提前的余项（2026-10-21 收口时登记）**：
      1. 改值用 `get_last_move_time()` 落表，**不**走上游的 `register_lookahead_callback`；且 `smooth_time`
         变更不 `flush_step_generation`——若无 ≥`hst` 的前瞻余量，可能改写 `hst` 内已生成位置、触发一次原地补步；
      2. `[extruder_stepper <name>]` 自己的 `pressure_advance` 仍只记录/上报，未接求解器（默认 0 与上游一致）；
      3. `find_past_position` 仍是 stub，pa 领先量不经 MCU 计数进 `motion_report`。
- [ ] **`input_shaper` 未作用到运动**（单独一项）：`extras/input_shaper.rs` 现在只做参数校验/上报，
      `update_input_shaping` 是空缝、`recompute_scan_windows()` 是文档化的 no-op——既不碰 trapq 也不碰
      position function（上游 `kin_shaper.c` 的 `input_shaper_alloc`/`set_sk`/`update_sk` 未移植）。
      **但窗口机制已就绪（2026-10-21）**：`StepKinematics::set_generation_window(pre, post)` 已落地，
      shaper 落地时照 `shaper_note_generation_time` 把脉冲跨度写进去即可。
- [ ] 各传感器的 `*/dump_*` 端点仍在 **B4** 名下。
- 依赖 F5/F6/F7、C1。

### H7 输入与外设

**已落地（2026-10-06 复核）**：`gcode_button`（数字路径与 `[buttons]` 注册表，批 #37；`analog_range` 明确拒绝）、
`pulse_counter`（批 #8，接通 `tachometer_pin`）、`filament_switch_sensor`、`filament_motion_sensor`
（各带 `section!`，`SET_FILAMENT_SENSOR` 走 mux 注册）。

- [ ] 固件按钮查询（`buttons.c`）/ `query_adc`——宿主 `[buttons]` 注册表已在但**查询没有发送方**；
      `query_adc` 整个模块不存在（被它挡住的 `gcode_button` 的 `analog_range` 与 `adc_scaled` 已自述）。
- [x] `pulse_counter.py`（批 #8：host 侧 `pulse_counter.rs` 落地并接通 `tachometer_pin`；`config_counter`/`query_counter` 按字典编码，真机验证仍待 T6）。
- [x] `trigger_analog` —— **不是缺口**：上游本就无 `load_config`/配置段（`trigger_analog.py` 只有
      `MCU_trigger_analog` 资源类），本仓三件套齐备：滤波设计（`extras/trigger_analog.rs`）、
      六条报文（`cmd/trigger_analog.rs`）、资源层（`mcu/resource/trigger_analog.rs`），
      消费面 `load_cell_probe` / `probe_eddy_current`。
- [ ] 线宽传感器：`hall_filament_width_sensor`、`tsl1401cl_filament_width_sensor`——无文件、无 section
      （断料两项**已落地**，见上）。
- [ ] 固件 `initial_pins.c` 的初始引脚状态。
- [ ] 特定板/芯片：`samd_sercom.py`、`replicape.py`、`palette2.py`。
- 依赖 F3（GPIO）、F5（ADC）、F9。

### H8 LCD 显示与菜单

- [x] **面板驱动全部落地（2026-10-06 复核）**：`display/display.rs` 的框架与分派里已含
      `hd44780`、`hd44780_spi`、`st7920`（SPI）、`uc1701`、`ssd1306`/`sh1106`、`aip31068_spi`（批 #7/#13/#28 + 后续），
      均为 storage-only。
- [ ] 个别驱动的 `set_glyphs`：`uc1701`、`hd44780_spi`、`aip31068_spi` 自述未移植字形写入。
- [ ] 菜单：`display/menu.py`、`display/menu_keys.py`、`display.cfg`、`menu.cfg`；
      事件 `menu:*`（`display/` 下无 menu 文件）。
- [ ] 固件 `lcd_hd44780.c` / `lcd_st7920.c`。
- 依赖 F9（固件侧 LCD）、G1b（`create_gcode_command`，菜单脚本要构造 gcmd）。

### H9 探测 / 调平 / 校准

- [x] **探针与回零周边全部落地（2026-10-06 复核）**：`[probe]` 段与 `probe` 虚拟 chip、选项与偏移、会话采样、
      `QUERY_PROBE`/`PROBE`/`PROBE_ACCURACY`/`PROBE_CALIBRATE`（`probe.rs:1032` 注册；校准后 `configfile.set`
      回写 `z_offset`）、`ProbePointsHelper`（`probe.rs:1297`；消费者 z_tilt/qgl/bed_tilt/screws_tilt/
      axis_twist/delta_calibrate）、`manual_probe`（`MANUAL_PROBE`/`Z_ENDSTOP_CALIBRATE`/`TESTZ`/`ACCEPT`/
      `NEXT`/`ABORT`）、`safe_z_home`、`endstop_phase`、`bltouch`、`smart_effector`、`probe_eddy_current`
      （含校准工具）均已落地；`probe:update_results` 已带载荷产线触发。
- [x] **`Z_OFFSET_APPLY_*` 小命令——已落地（2026-10-06，`865b583`）**：`[probe]` 的
      `Z_OFFSET_APPLY_PROBE`（`probe.rs`，与 eddy 同名但 section 互斥）与 `manual_probe` 的
      `Z_OFFSET_APPLY_ENDSTOP`（普通机写 Z 轨 `position_endstop`，delta 机为三塔
      `stepper_a/b/c`）均已注册，文案与 `%.3f` 回写对齐上游；与上游的一处等价差异
      （上游用重名覆盖注册、本仓因 `register_command` 拒绝重名而按 `kinematics: delta` 二选一）
      已登记在 `upstream-deviations.md`。**语料 0 处使用**，覆盖靠单测。
- **端停位置接口化：已完成（2026-10-06 复核，此前条目为陈旧断言）**。`PinChip::setup_endstop_dyn` 返回
      `Arc<dyn HomingEndstop>`，`ProbeChip::virtual_endstop_position` 返回 `z_offset`（`pins.rs:386` `:396`、
      `probe.rs:306`），rail 优先采用虚拟位置、否则才退 `position_min`（`extras/stepper.rs:374-406`；
      上游 `stepper.py:336-343`）。另：`z_virtual_endstop.test` **本身没有任何位置断言**（只是一串
      `G28/G1/PROBE/BED_MESH_CALIBRATE/...`），所以「该 test 需要它」的说法不成立——接口正确性靠单测。
- [x] **保真单元 U1：bed_mesh 插值网格——已落地（2026-10-06，`07072b9`）**：`ZMesh` 的 `lagrange`/`bicubic`/`direct`
      三个采样器 + `mesh_pps` 细分 + `bicubic_tension` + `calc_z(x, y)`（双线性查表、越界 constrain）+ `round(z,6)`。
      两个出口（`get_status` 与 `bed_mesh/dump_mesh`）现均报插值后的 `mesh_matrix`（U1 前 dump 端点还是 `probed.clone()`）。
      规格要点（上游 `bed_mesh.py:1321-1539`）：边长 `(count-1)*(mesh_pps+1)+1`——**`mesh_pps: 1` 仍插值**，
      只有两轴 `mesh_pps` 全 0 时 `_verify_algorithm`（`:428-430`）才改写为 `direct`、`mesh_matrix` 才等于 probed。
      ⚠️ 素材分支 `abandoned/wip-main-leftovers` 的 `bed_mesh.rs` **不可复用**（`lagrange_1d` 是恒等复制、
      `bicubic_1d` 是 Catmull-Rom、不按 `mesh_pps` 细分），本单元是按上游重做的；语料对此零覆盖，验收靠自建单测。
- [x] **保真单元 U2/U3：把网格作用到 move——已落地（2026-10-06，`e462770`）**：fade（`fade_start`/`fade_end`/`fade_target`
      与 `get_z_factor`；`fade_target` 的合法性看**网格 Z 范围**、`fade_target == 0` 时免检、另有 `fade_dist` 上界）
      与 `MoveSplitter`（`build_move`/`calc_z_offset`/`set_next_move`/`split`，逐行对齐 `bed_mesh.py:1257-1319`）；
      `impl MoveTarget for BedMesh` 并在 `load_config` 里以 `force=false` 注册（上游 `bed_mesh.py:130-131`）。
      规格校正：**`BED_MESH_CLEAR` 不撤销 transform**——上游只注册一次，clear 就是 `set_mesh(None)`，
      `move()` 在无网格时直通 `toolhead.move`；另新增 `split_delta_z`（默认 `.025`、min `0.01`）、
      `move_check_distance` 补 min `3.`。
      **profile 命令族已落地（2026-10-19）**：`BED_MESH_PROFILE`（LOAD/SAVE/REMOVE，存 `[bed_mesh <name>]`、
      `SAVE_CONFIG` 写回）、`BED_MESH_OUTPUT`（`PGP`）、`BED_MESH_MAP`、`BED_MESH_OFFSET`（`X`/`Y`/**`ZFADE`**），
      `BED_MESH_CALIBRATE PROFILE=` 自动存盘；`bed_mesh/dump_mesh` 的 `profiles` 改读真实表。
- [ ] **`bed_mesh` 余项（唯一登记处）**：
      1. **faulty 区域替换**（上游 `_process_faulty_regions`，`bed_mesh.py:949-1000`）：本仓
         `FaultyRegion` 只解析成字段，插值/探测/`get_status`/dump **全部不看它**；
      2. **`ZERO_REFERENCE`**：`set_zero_reference`（上游 `bed_mesh.py:1409`）没有命令注册；
      3. **dump 端点的 `mesh_params` 界限**：`bed_mesh/dump_mesh` 用配置现算的点算界限，而上游用
         已装载网格的探测点包围盒（`z_mesh.get_mesh_params()`）——`LOAD=` 一个不同规模的 profile 后两者不同。
         （注意 `calibration.config.mesh_min/mesh_max` 取配置**是对的**，上游 `dump_calibration` 同样如此；
         `get_status` 也已用装载网格。）
- [x] 螺丝：`screws_tilt_adjust` ✅；`bed_screws` **段 + 命令族均已落地**（2026-10-06，`55f534a`：`BED_SCREWS_ADJUST`
      常驻注册，`ACCEPT`/`ADJUSTED`/`ABORT` 随会话注册与注销；粗调→精调两遍；`bed_screws.test` 走真命令转绿）。
      有意偏离一处（已登记）：注册中途失败时本仓收回半组命令并复位，上游会留下半个命令组（`upstream-deviations.md`）。
- [ ] 校准：`delta_calibrate` ✅（段+`DELTA_CALIBRATE`/`DELTA_ANALYZE` 落地 2026-09-24 批 #5，`delta_calibrate.test` 转绿）、`axis_twist_compensation` ✅（批 #36）；**未移植**：`skew_correction`、
      `z_thermal_adjust`、`tuning_tower`（三者无文件、无 section，语料不覆盖）。
- [x] 回零周边：`homing_heaters`（批 #31）已落地；事件 `probe:update_results` 已带载荷并触发（批 #36），`homing:*` 四个已随 toolhead 产线触发（见事件清单）。
- 依赖 C1、F8（endstop/trsync）、H3（宏）、H12（`mathutil`）。

### H10 运动相关 extras

- [x] **同轨兄弟电机一起被该轨端停刹车（2026-10-18）**：`Rail::lookup` 把 `[stepper_z1]`/`[stepper_z2]`
      等不带自己 `endstop_pin` 的兄弟注册进轨道端停的 `TriggerDispatch`（上游 `stepper.py:433-445`），
      `McuTrsync::add_stepper` 改幂等（上游 `mcu.py:183-186`）。此前 Trident 的 `G28 Z` 只有
      `stepper_z` 会立刻停，z1/z2 跑完约 50 ms 队列 ⇒ gantry 隐斜且 `set_position` 记账错误。

- [x] `gcode_arcs.py`：**已落地**（2026-10-21）——`G2`/`G3` 按上游 `planArc` 分段（圆心/角度归一化、整圆、螺旋轴、最后一段取目标点），
      `G17`/`G18`/`G19` 选平面；`R` 按上游拒绍（`does not support R moves`）、`P` 不解析、`G91` 下拒绍。
      每段经 `gcode_move` 的 `G1` 缝发出（不自拼 trapq），因此坐标变换/`G92`/因子均生效。18 测 + `gcode_arcs.test` 在真命令下绿。
- [x] `manual_stepper.py` **已落地**（section + 命令；**2026-10-18 补齐自有 trapq**：首次需要时装进运动队列、`MOVE=`/`G1 A…` 真出步、`SET_POSITION` 先 flush 再摆 solver）。
      **余项**：自带回零（`STOP_ON_ENDSTOP` 仍明示拒绝——缺 `drip_update_time`，`drip_move` 只灌主 trapq）。
- [x] `force_move.py` **主体已落地**（2026-10-04：`FORCE_MOVE`/`STEPPER_BUZZ` + 整段 force-move 时序，motion 侧按 handle 的
      trapq 原语；`SET_KINEMATIC_POSITION` 一直在 `toolhead`）。
- [ ] `extruder_stepper.py`：**段已落地**（2026-09-24 批 #2），宿主 step 同步的 toolhead 缝仍缺
      （不跟随 extruder 运动）。
- [ ] **extra-axes 外围**：`GET_POSITION` ✅ 与动态 `axis_map` ✅ 已落地（2026-10-18；`G1 A…` 按字母寻址、进入规划器并有自己 trapq 的 `manual_stepper` 能真出步）。
      余项：`MAX_AXES = 8` 的容量上限（E + 4 个字母轴，第 5 个不映射）；`G1_PARAMS` 静态表不为额外字母提供补全；`reset_last_position` 的尾巴沿用上一段（toolhead 只报 4 轴）。
      （`stepper_enable.py` ✅ 已随 T2 落地并归档。）
- [x] `idle_timeout.py`（批 #21：节 + `SET_IDLE_TIMEOUT` + 三个事件带载荷）、
      `motion_report.py`（`dump_trapq`/`dump_stepper` 端点，见 **B4**）。
      （`motion_queuing.py` ✅ 已由 `motion/queuing.rs` 落地并归档。）
- 依赖 C1（toolhead/kinematics）；`gcode_move` 已随 G4-1 落地（见 G4 节），不再是 H10 的前置。

### H11 主机运行时与调试


- [x] **MCU 失联检测与连接期串口重试（2026-10-18）**：`mcu_clock_poll` 现在按上游判据（未答查询 > 4，`clocksync.py:156-157`）报 `Lost communication with MCU '<name>'` 并停机，每 MCU 一次、file-output 不判；独占串口的连接期打开按 5 s × 90 s 预算重试（只包打开，重试在阻塞池上跑，退出请求可打断）。
- [x] **日志轮转与 rollover 配置块 —— 已修（2026-10-18）**：`--logfile` 现在照上游 `bglogger` 的
      `TimedRotatingFileHandler(when='midnight', backupCount=5)` 轮转（本地午夜后第一条日志触发，
      `klippy.log.YYYY-MM-DD`、保留 5 份），新文件开头写 `rollover_info`（`versions` + `config` 块，
      形状照 `configfile.py:482-487`）。**修正旧断言**：上游**不是**「2 MB × 5」，全仓没有 `maxBytes`。
- [ ] **余项**：`--logfile` 缺省仍不落盘（上游同样需要显式给）；仓内不带 systemd unit 文件；
      按天切不按大小切，刷屏会话单日文件仍可能很大。
- [x] **定时器回调 panic 无兜底 —— 已修（2026-10-18）**：定时器、latency notifier、运动队列的
      flush/active 回调全部 `catch_unwind`；panic 记 `error!`（带回调名与 panic 文本）后丢弃那个定时器，
      其余照常调度（详见 `docs/klippy/developer-manual/architecture.md`）。

- [ ] `statistics.py` 的 **`PrinterStats` 半**：上游每秒打一行主机统计汇总（`statistics.py:47-72`）。
      对象那半已落地（`extras/system_stats.rs` 无条件注册 `system_stats`，报
      `sysload`/`cputime`/`memavail`/`dropped_logs`/`dispatch`），本仓以反应堆延迟告警替代该日志行；
      要补时需要 `PrinterObject` 上的 `stats` 回调位（现在 trait 没有）。
- （`error_mcu.py` ✅ 已落地并归档：`extras/error_mcu.rs`，消费 `klippy:notify_mcu_error` /
  `klippy:analyze_shutdown`，由 `[mcu]` 工厂 ensure。）
- [x] **`canbus_ids` 与 CAN 节点发现——已落地（2026-10-06）**：`extras/canbus_ids.rs`（现迁移至
      `mcu/ids.rs`，并已泛化为全接口共用的 MCU 层 id 分配器）按
      `[mcu]` 声明顺序发号（`NODEID_FIRST = 4`、步长 1、`new_id = len(ids) + 4`），`[mcu]` 的
      `canbus_nodeid` 因此变为**可选**（写了就用、且占号位；与自增号撞车时**装载期报错**——
      本仓有意偏离：上游靠固件 `can_id_conflict` 兜底）；**2026-10-11 修订**：自动号已改为
      `4` 起**最小空位**（上界 255），并改为装载时**预扫全配置**保留全部显式 `canbus_nodeid`
      （`reserve_explicit`）——显式号写在哪个位置都生效、发号与声明顺序无关，自动号不再撞显式
      号，装载期只在两个显式号互撞或号段占满时报错（上面的 `new_id = len(ids) + 4` 游标已不再
      使用）；发现侧新增 `klipperx canbus-scan
      [--interface can0] [--timeout 2]`（广播 `0x3f0/[0x00]`，收 `0x3f1` 的 `RESP_NEED_NODEID`，
      输出可粘进配置的 uuid）——**都不写回配置文件**（与上游 pinned 版一致）。
      **未做**：`canbus_stats.py`（总线错误计数/状态上报）；真总线上的行为未验证（见 testing.md）。
- [ ] **CAN 的 `SET_NODEID` 没有身份校验（2026-10-20 评审发现，未修）**：上游发完 `SET_NODEID` 就调
      `get_canbus_id` 把被测板的 uuid 读回来比对（`serialhdl.py:158-167`，不等就 `disconnect()` 重来，
      90 s 预算）；`get_canbus_id` 是**字典消息**，而字典由 `identify` 交付，所以 open 阶段（那时只有
      `0x3f0` 的 admin 帧、没有 msgid）拿不到应答语义。本仓现在只做**有界重发**兜丢帧：
      `NODEID_ASSIGN_ATTEMPTS = 3`、`NODEID_ASSIGN_INTERVAL = 200 ms`（合计 ≤1 s），发完即开链路，
      身份是否吻合留给 identify（`interface/devices/canserial.rs` 的 `open_with_wire` 自述）。
      **后续项**：在 identify 拿到字典之后补一次 `get_canbus_id` 比对 + 不匹配则重连（上游同一判据），
      届时「发出去的 `SET_NODEID` 到底落在哪块板」才有主机侧证据。
- 判为不适用：`garbage_collection.py`、`aio_executor.py`、`parsedump.py`（Python 侧调优/
  离线工具，与本主机无关；完整清单见「待办」表后的不适用段）。

### H12 核心工具补齐

- [ ] 便捷 helper（**参照系待定**）：`get_heater` / `get_sensor` 这类封装。**注意口径**：语料钉定的上游
      `third_party/klipper/klippy/util.py` **没有**这两个函数——它们是更新版上游才有的东西，
      本仓在语料里无参照。现有能力已足够：前缀式 `lookup_objects`、`statuses`、
      `get_status -> serde_json::Value`（`printer.rs`）。按 AGENTS 「一切对照读 `third_party/klipper`、
      语料是最终仲裁」，这条先当「新上游特性、无覆盖」挂着，不排期。
- [ ] `pins.py` 消费侧 `get_pin_type` 查询面——同样参照系待定：语料钉定的上游 `pins.py` 没有该函数；
      本仓以 typed 入口（`setup_digital_out`/`setup_pwm`/`setup_adc`）代替上游的 `setup_pin(type, desc)`，
      `PinType` 与 `lookup_pin` 已有（`pins.rs`）。

## 未决问题

- [x] **Q9 `wait_moves`（FW5d）节拍：宿主等待已落地（2026-10-06，`a23a92b`）；余项＝送出口径 / 真机验证**：
      本仓 `M400` 现在真的等到 MCU 的**估计时钟**追上 `print_time`（`motion/toolhead.rs` 新增
      `wait_moves_state()` 快照 + `extras/toolhead.rs` 的异步等待环，每轮短锁读数、解锁后 sleep；
      四条有界出口：est 追上 / est 停滞 1s 报错 / shutdown / 不可暂停）。
      **关键语义（上游同形）**：`can_pause = !printer.is_fileoutput()`，不可暂停时**立即退出等待**
      （`toolhead.py:221-223` 与 `:425-429`）——所以语料（`-o` 形态）下不等待，**语料对该特性零覆盖**。
      实测：无 `debug_output` 的 e2e 里 `gap_before=0.4247s → waited=0.5048s → gap_after=-0.0793s`；
      反向对照（去掉闸）会让 file-output 测试实测睡了 500ms，闸由测试钉住。
      **仍缺（后续单元）**：① `send_wait_ack`（`mcu/mod.rs:2639-2650` 自述只是 flush）；
      ② fileoutput 形态下上游 `_flush_handler_debug` 的等价物。
      **已消除的两个“缺口”**（2026-10-06 复核，不要当未完项再排）：
      - **“送出口径会让 step 晚到 0.106–0.115 s”是假结论——该证据的唯一来源是假固件模型漏算首拍 `interval`**
        （`simulator.rs` 的 running 分支，已修）；修后同样的真等待复现从 11 例红变 259/0 绿。
        真机送出口径是否存在同类偏差**无任何证据**，已降为待验假设，由 R11 观察。
      - `MIN_KIN_TIME`/`kin_flush_delay` 地板**已对齐上游**（`calc_print_time`），
        但在本仓 `BUFFER_TIME_START = 0.250` 远大于 `MIN_KIN_TIME + KIN_FLUSH_DELAY = 0.101` 的前提下
        **行为等价（近似 no-op）**，不要把它当成“不晚到”的保障。
      **真机未验证**：等待生效路径（无 `debug_output`）尚未上真板，见 TESTING.md 的 R11。

- [x] **Q11 断环：已修（2026-10-06）** —— `VirtualSdCard` 与 `GCodeButton` 都持 `Arc<GCodeDispatch>`、
      又把**捕获 `Arc<Self>`** 的 handler 注册回它的命令/mux 表，于是命令表反向钉住部件：
      `teardown()` 后 `Mcu::Drop` 不跑 → `interface.shutdown()` 不被调用 → 收尾时那批 `spawn_blocking`
      的阻塞读永远 park。修法照本仓已有约定（`delayed_gcode`/`idle_timeout` 同形、`gcode.rs` 的
      「析构」段落已把它写成约定）：把注册进 dispatcher 的 handler 对自身的捕获改成 `Arc::downgrade`。
      **实测**：撤掉修复 → 改敏感的探针失败并点名 cfg
      `printer-biqu-bx-2021.cfg: … did not shut down within 5s (5.000556956s): a part was leaked`；
      修复在位 → 同一探针 **0.50s** 通过；
      `upstream` 过滤全量 422 条 69.7s、**最慢 0.83s**（此前两条各 5.25s）；`KLIPPERX_TRACE` 下这两条
      现在会打 `MCU DROP`。
      **顺带解决**：`firmware_restart` 时旧连接不再被命令表钉住（新增
      `a_firmware_restart_releases_the_old_session` 钉住它）。
      **前一轮的错误断言已更正**：我曾写「`-o`（fileoutput）下不 park、所以语料不受影响」——
      实测在含 `[virtual_sdcard]`/`[gcode_button]` 的 cfg 上**不成立**（这两条语料就是代价）。
      另：真机（非 `-o`）形态的 park 复现已**修好并转绿**：那条探针当时叫
      `a_live_case_leaves_the_fake_devices_reader_parked`（`#[ignore]`），现改名 `a_live_case_runtime_shuts_down`
      并**取消忽略**——它的 cfg 带 `[extruder]` 加热器，真正首因是下面那条 reactor 定时器环，
      **不是**我当时归因的「整台机器从不说话」（已在源码 doc 订正）。
      **环已断（2026-10-06，`2875ee7`）**：`verify_heater` 的 1s reactor 定时器回调强持 `Arc<Heater>`
      （环 `reactor → 定时器堆 → Heater → McuPwm/chip → McuClock → reactor`）→ 改成 `Weak` + `upgrade()`；
      同批修了 `virtual_sdcard::do_resume` 的一次性定时器强持 `Arc<Self>`。
      实测：同一探针从 `5.000577767s: a part was leaked` → `MCU DROP mcu` + 收尾 **942µs**；
      带加热器的重启用例 `a_live_heated_restart_releases_the_old_session` 收尾 **1.34ms**，
      并断言旧会话的 `Arc<Mcu>` 强计数收敛到 1 后第二次会话才能重新 identify。
      **全集清点**（1 2 处 `register_timer`）后的**最后一处同类环也已断（2026-10-21）**：
      `api/endpoints/objects_subscribe.rs` 的刷新定时器回调改持 `Weak<Inner>`，钉子用例
      `teardown_releases_the_printer_through_the_refresh_timer` 钉死。

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

- [x] **Q8 GCodeIO（伪 tty / OctoPrint 串口仿真）补不补**：**结案（2026-10-05）：不做**（2026-09-21 曾定暂缓）。
      `need_ack`/`ack` 机制已从 `src/core/klippy/gcode.rs` 删除；偏移登记在
      [本项目与上游的偏移](docs/klippy/developer-manual/upstream-deviations.md)。纯 API 主机（Moonraker）不需要它；
      放弃的能力：`debuginput_exit`、`is_fileinput`/`error_exit`、`stats gcodein=`、`input_log`、`M112` 乱序。
      需要 OctoPrint 串口主机或上游 batch mode 的场景请用上游 Klipper。若将来翻案，前置见 G1b 的 `GCodeIO`
      条目（首要是 reactor 的 fd 事件层）。

## 上游事件对照清单（事件总线已就绪，逐项注册处理器）

上游 `Printer` 维护 `event_handlers` 字典（`klippy/klippy.py:36`），通过
`register_event_handler(name, cb)` 注册、`send_event(name, *params)` 分发，共 35 个事件名。

`KlippyEvent` 已声明全部 35 个名字，`Printer::register_event_handler` / `send_event` 按名
注册与分发，处理器签名为 `Fn(&KlippyEvent)`。以下事件可按优先级逐个注册处理器；依赖
关系标注在「实现依赖」列中，`—` 表示仅依赖事件总线，其他依赖的模块已标记为其他 TODO 条目。
生命周期事件已触发，其余事件的触发点随对应模块落地（逐项状态 2026-10-04 复核，见各表）。

> **「参数」列的口径＝本仓载荷**：按 `src/core/klippy/event/decl/*.rs` 的 `event!` 声明写，
> 裸声明一律写「无」，带字段的写 `{字段}`。上游 `send_event` 的实参（对象引用、位置参数等）
> 见「上游位置」列与[事件系统](docs/klippy/developer-manual/event-system.md) 的上游载荷表；
> 上游传对象引用而本仓 `decl` 为裸声明时，本列写「无」（处理器按需自行 `lookup_object`）。

### 生命周期事件（最高优先级）

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `klippy:mcu_identify` | MCU identify 完成后 | 无 | `klippy/klippy.py:131` | — |
| `klippy:connect` | 配置装载完成、打印机即将就绪 | 无 | `klippy/klippy.py:132` | — |
| `klippy:ready` | 打印机进入 ready 状态 | 无 | `klippy/klippy.py:162` | — |
| `klippy:shutdown` | 进入 shutdown 状态 | 无 | `klippy/klippy.py:211` | — |
| `klippy:disconnect` | 运行结束/退出时 | 无 | `klippy/klippy.py:195` | — |
| `klippy:firmware_restart` | firmware restart 前 | 无 | `klippy/klippy.py:194` | — |
| `klippy:notify_mcu_error` | MCU 通信出错时 | `{msg, details}` | `klippy/klippy.py:144,151` | ✅ 已接入 |
| `klippy:analyze_shutdown` | 进入 shutdown 后分析 | `{msg, details}` | `klippy/klippy.py:216-220` | ✅ 已接入 |

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
| `homing:home_rails_begin` | 回零开始 | 无 | `klippy/extras/homing.py:210` | ✅ 已触发（`extras/toolhead.rs:2316`，回零循环与各运动学路径多处） |
| `homing:home_rails_end` | 回零结束 | `{axes, homing}` | `klippy/extras/homing.py:234` | ✅ 已触发（`extras/toolhead.rs:3236`） |
| `homing:homing_move_begin` | 回零移动开始 | 无 | `klippy/extras/homing.py:80` | ✅ 已触发（`extras/toolhead.rs:2734` 等） |
| `homing:homing_move_end` | 回零移动结束 | 无 | `klippy/extras/homing.py:148` | ✅ 已触发（`extras/toolhead.rs:2806` 等） |
| `stepper:sync_mcu_position` | stepper 位置同步 | 无 | `klippy/stepper.py:227` | C1 |
| `stepper:set_dir_inverted` | 方向反转设置 | 无 | `klippy/stepper.py:153` | C1 |
| `dual_carriage:update_kinematics` | IDEx 双滑车运动学更新 | 无 | `klippy/kinematics/idex_modes.py:383` | C1 |

> `homing:*` 四个已随 toolhead 产线触发（`extras/toolhead.rs:2316` `:3236` `:2734` `:2806`，
> `G28` 路径；`homing_heaters` 也会发 move 对）；`stepper:sync_mcu_position`、`stepper:set_dir_inverted`、
> `dual_carriage:update_kinematics` 仍无触发点，分别随 stepper 位置同步、目录反转变更、
> C1c 的 IDEX/双滑车落地。

### idle_timeout 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `idle_timeout:ready` | idle_timeout 模块就绪 | `{print_time: f64}` | `klippy/extras/idle_timeout.py:95` | — |
| `idle_timeout:idle` | 进入空闲状态 | `{print_time: f64}` | `klippy/extras/idle_timeout.py:57` | — |
| `idle_timeout:printing` | 开始打印（恢复活动） | `{print_time: f64}` | `klippy/extras/idle_timeout.py:106` | — |

> 批 #21 已落地：`[idle_timeout]` 装载即注册，三个事件带 `{print_time: f64}` 载荷发出。

### 工具头事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `toolhead:manual_move` | 手动移动前 | 无 | `klippy/toolhead.py:416` | ✅ 已触发（`safe_z_home` 的 `HomeOps::manual_move`，`extras/safe_z_home.rs:215`） |
| `toolhead:set_position` | 设置位置（G92 等） | 无 | `klippy/toolhead.py:390` | ✅ 已触发（`extras/toolhead.rs:1752`，`SET_KINEMATIC_POSITION`） |
| `toolhead:sync_print_time` | print_time 更新 | 无 | `klippy/toolhead.py:267` | **尚无发送点**（C1d 的回调已落地，但该事件没有触发方；`idle_timeout` 改为观察 `print_time` 前进） |
| `toolhead:update_extra_axes` | 额外轴位置更新 | 无 | `klippy/toolhead.py:455` | ✅ 已触发（`ToolHeadObject::add_extra_axis` / `remove_extra_axis`，`extras/toolhead.rs:1876` / `:1892`，`manual_stepper` 调用） |

> `toolhead:set_position`（`gcode_move` 重置链之一）、`toolhead:manual_move`（`safe_z_home`）
> 与 `toolhead:update_extra_axes`（`manual_stepper`）都已产线触发；只剩
> `toolhead:sync_print_time` 无发送点（C1d 已收官）。

### gcode 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `gcode:command_error` | gcode 命令错误 | 无 | `klippy/gcode.py:226` | ✅ 已触发（`process_line`） |
| `gcode:debuginput_exit` | debuginput EOF | 无 | `klippy/gcode.py:433` | **不做 `[x]`**（本仓不实现行协议输入；上游靠 `send_event` 收集返回值） |
| `gcode:request_restart` | 请求重启 | `{print_time: f64}` | `klippy/gcode.py:358` | ✅ 已触发（`gcode.rs:1357`，`get_last_move_time` 取 print time） |

> `gcode:command_error` 已接（handler 的 `CommandError` 触发，panic 不触发）；`gcode:request_restart`
> 已在 `request_restart` 处理器里产线触发（`gcode.rs:1357`，先 `get_last_move_time` 再
> dwell/wait）；`gcode:debuginput_exit` 随 `GCodeIO` **不做 `[x]`**（本仓不实现行协议输入），
> 若将来翻案还要先让 `send_event` 收集 handler 返回值（上游 `all(...)`）。

### 工具/传感器事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `probe:update_results` | probe 测量完成 | `{results: ProbeResultsHandle}`（可原地改 Z，批 #36） | `klippy/extras/probe.py:366` | endstop |
| `extruder:activate_extruder` | 切换 active extruder | 无 | `klippy/kinematics/extruder.py:289` | 尚无发送方（handler 已备，`gcode_move.rs:392`；`ACTIVATE_EXTRUDER` 不触发它） |
| `stepper_enable:motor_off` | stepper 电机关闭 | 无 | `klippy/extras/stepper_enable.py:120` | ✅ 已触发（`stepper_enable.rs:351`） |
| `virtual_sdcard:reset_file` | VSD 文件重置 | 无 | `klippy/extras/virtual_sdcard.py:151` | sdcard |
| `load_cell:calibrate` | 称重传感器校准 | 无 | `klippy/extras/load_cell.py:404` | ADC |
| `load_cell:tare` | 称重传感器归零 | 无 | `klippy/extras/load_cell.py:406` | ADC |

> 依赖各自模块（endstop、sdcard、ADC 等），不阻塞运动。

### 显示/菜单事件（menu.py 内部）

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `menu:populate` | 菜单填充 | 无 | `klippy/extras/display/menu.py:346` | display |
| `menu:init` | 菜单初始化 | 无 | `klippy/extras/display/menu.py:722` | display |
| `menu:begin` | 菜单开始 | 无 | `klippy/extras/display/menu.py:754` | display |
| `menu:exit` | 菜单退出 | 无 | `klippy/extras/display/menu.py:913` | display |

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
├── gcode:* —— command_error / request_restart 已触发；debuginput_exit 不做（本仓无行协议输入）
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
| `GCodeIO`（伪 tty / 文件输入、`ack` 协议；**本仓不做**，`ack`/`need_ack` 已删） | `klippy/gcode.py:390-494` |
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

## 设备操作层剩余 P2（2026-10-20 评审留档，未修）

来源：出真机前的 20 lane 设备代码评审（报告口径与证据见评审会话的 `.staging/device-review/REPORT.md`，不入库）。
每条都复核过 file:line；**均不阻塞真机**，撞上了按这里直接改。

- [x] **`same_file` / 重复 attach / 重开轮次 / 停端口握手 / `answers_for` / reader panic / scan 读窗口 / 排他打开 / 无界 channel —— 均已修（2026-10-21）**：
      共 9 条，按本节的修法方向全部落地（`serial.rs` + `frame.rs`），每条带改前必红的用例。
- [ ] `TunnelRegistry::register` 静默顶替同 id，且生产代码从不 `clear_tunnel_sink`（**后者是有意设计**：
      `bridge_flash.rs` 靠 drop receiver 隐式退订，注释已写明）；`UnitRegistry::has`/`deliver` 之间的
      TOCTOU 会丢单元（失败分支现在有 `debug!`，但没有 warn）。
- [ ] CAN 的 `SET_NODEID` **没有身份校验**（本批已加有界重发 3×200 ms，但 uuid 比对未做）：上游 `serialhdl.py` 的 `get_canbus_id`
      是字典报文，只能在 `identify` 之后做；现挂在 open 阶段拿不到 msgid。后续单元：在 `connect` 装好字典后比对 uuid。
- [ ] 桥端口数上限与固件不符：主机允许 256、固件 `BRIDGE_MAX_PORTS 4`（`bridge_ports.rs` / `third_party/klipper/src/bridge.c`），
      超限只靠固件 shutdown 兜住且报错不点名额；固件把「槽号越界」与「tty 打不开」折叠成同一句 `config_uart: … could not be opened`，
      主机侧当成瞬时丢 tty 去走 10 s 恢复。
- [ ] `pending::cancel` 删的是「第一个同名同 oid 匹配」而非 token：同名并发调用时先超时者会删掉别人的登记；
      `resolve` 对「有同名登记但 oid 不匹配」也返回 `true`（`mcu/pending.rs`）。修法：`register` 返回自增 token，按 token 精确删。
- [ ] `seconds_to_clock` 的 `.max(0.0)` 与上游不符：负数/NaN 静默变成 0（= 立刻执行）而非报错（`mcu/mod.rs`）。
- [ ] `move_clocks` 先记池子再 `enqueue`，`enqueue` 失败会留一条永不解扣的幽灵槽（`mcu/mod.rs`）。
- [ ] `TriggerDispatch::stop()` 把 raw≥5 的失败理由折成 `HostRequest`（`resource/trsync.rs`）；改用 `stop()` 返回值的调用方会把探针错误读成正常结束。
- [ ] `trigger_analog_home` 缺 `req_clock`（上游 `reqclock=clock`）；`home_wait` 在 `clear_home()` 失败时跳过 `dispatch.stop()` 且顺序与上游相反；
      重发缓存「先提交后发送」，发送失败无回滚（`resource/trigger_analog.rs`）。
- [ ] `endstop.rs` 的链路故障用 send 错误覆盖了上游的 "Communication timeout during homing" 文案。
- [ ] `stress --task step` 无行程上界与提示（真打印机上会顶机械端）；`Err(err)` 与固件自报 shutdown 折成同一出口（断线=量到限值、退出码 0）；
      收尾用隐式 `Drop` 而非 `shutdown_timeout`；`wait_for_idle` 以「相隔 5 ms 两次读数相等」判定空闲（`src/stress.rs`）。
- [ ] `hardware_test` 没有用例级总超时（L2 用例体与板锁等待都无界）；teardown 不关加热/输出（用例在设了目标后失败会带着 PWM 值继续跑）；
      `apply_id_addressed_transport` 只复刻 Serial 那一枚戳，`host_mux`/`host_port_paths` 不设（`src/hardware_test.rs`）。
- [ ] 注释/文档与实现不符的存量：`mcu/ids.rs` 称串口 id「无人消费」；`resource/stepper.rs` 称 `step_pulse_duration=0`
      让固件自己挑（实际是零脉宽）；`MoveSlots` 把 `set_next_step_dir` 说成占槽（真正占槽的是 `queue_step`/`queue_digital_out`/
      `queue_pwm_out`/`queue_pca9685_out`）。

