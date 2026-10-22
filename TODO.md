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
3. **归档只留大项**：本文件的「已完成（留档）」只保留**子系统/框架队列/里程碑级**的
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
框架队列（FW1–FW9）与上游 extras 消费者（H1–H12）均已完成，索引见文末「已完成（留档）」。
零散待办（无编号）紧跟索引表，细节见各节或各模块文档。

**索引**

| # | 事项 | 依赖 |
|---|---|---|
| H11 | 主机运行时与调试：两条 2026-10-22 运行时小件，见 [H11 节](#h11-主机运行时与调试) | — |
| S2 | 文档补齐（非新功能）：开发手册「模块表 / `testing.md` 覆盖行 / `regression-tests.md`」的陈旧行未逐条重审（本轮只修了与已删条目强相关的那些） | — |
| S3 | 真机用例：把 `configure_taking_over`（“连接 ≠ 配置”，遇 `ResetRequired` 则 `reset` + 重开 + 重握手一次）**上提到 `hardware_test`**——现在三份需求（R3/R5/R11）共用一个局部实现（`mcu/resource/endstop.rs` 测试里 + `stress.rs` 自己的），上提后只留一份；同时把 L2（`Machine::bring_up`）已落地这件事写进框架 doc 的模板 | — |
| S4 | 真机用例继续铺（框架两层都已就绪）：R4 归零（无加热，建议先做）→ R9 收尾 / R7 挤出 / R6 温度 / R10 急停（**带加热的最后**）；R8 回放 | S3 |
| S6 | 运行时临时产物收口（**暂缓，2026-10-19 登记**）：`/tmp` 侧有 `kx-start.log`（每次手动拉 klippy 重新生成）、活连接文件 `klippy_uds` 与 Moonraker 的 `moonraker_instance_ids`、harness 的 `pi-bash-*.log`；仓库侧 `.staging/tmp-2026-10-19/` 存着 13 项旧产物（7 个 `klipperx-log-*` 运行目录 + 6 份闸门/启动日志，3.1 MB，已 gitignore）。要做的：按已定路径约定把日志改道（`klippy` 已有 `--logfile` 落 `<data-path>/logs/`，手动启动的 stdout 也应落那里），并给 `.staging` 定一个清理时机。判据：`klippy_uds` / `moonraker_instance_ids` 是**活 IPC 与第三方状态——不迁移、不进释放链**；`pi-bash-*.log` 归 harness；清 `.staging` 前先确认里面的日志不再作任何结论的证据 | — |
| E2 | `python_path` 的取消 | 外部项目 |

**零散待办**（无编号；细节见各节与各模块文档）

- `canbus_stats`：总线错误计数/状态上报。上游 `canbus_stats.py`，本仓无对应模块；
  CAN 侧现有能力是 id 分配器（`mcu/ids.rs`）与节点发现（`klipperx admin scan`），见
  [MCU 连接方式](docs/klippy/user-manual/mcu-connection.md)。
- `manual_stepper` 自带回零：`STOP_ON_ENDSTOP` 仍明示拒绝（缺 `drip_update_time`，`drip_move` 只灌主 trapq）。
- `pwm_tool` 的 `maximum_mcu_duration` 中间更新：固件上限照常下发（`setup_max_duration`），上游的
  宿主侧重生成（`MCU_queued_pwm._gen_intermediate_updates`，`pwm_tool.py:119-135`）本仓无对应物，
  见 `extras/pwm_tool.rs` 的模块文档。
- `BED_MESH_OUTPUT PGP=` 补上游的 `(Substitute)` 列（`docs/klippy/developer-manual/README.md` 的 bed_mesh 行尾也记着这条）。
- 真板验证：`S1` 的双机并发压测（`stress.md` 已有单板数据）与 `M2` 的极性与保持。
- TMC 的 SPI 「写 + 回读」切到 `McuSpi::transfer_with_preface`（入口已具备、TMC 未切；收敛判据是
  片选次数不变，见 `docs/klippy/developer-manual/README.md` 的 `tmc_spi.rs` 行与模块文档）。
- CAN 帧的仲裁 id 未校验范围：`CanFrame::new` 只查长度（`interface/devices/canserial.rs:258-275`），
  `to_abi` 直接把 `self.id` 写进内核布局；只有 `from_abi` 侧用 `CAN_SFF_MASK` 掩码。
  写入路径应拒 `id > 0x7ff`（与上游 `canlib` 的行为对齐）。
- ADC 样本时刻的相位：上游 `mcu.py:635,644` 的「每样本时刻比 report 周期晚一拍」是否等价，
  本仓 `mcu/resource/adc.rs` 未逐行对过——待核（不影响当前语料）。

**串口复用 / 桥（跨主机与固件）**

| # | 事项 | 依赖 |
|---|---|---|
| M1 | 桥下游**端口模块化**与**没有 admin 通道的设备**共存：端口按 `[uart_device]`/`[can_device]` 定义、子设备引用端口节名，归属由主机经桥自己的 admin 通道告知桥（分六期：P1 配置面 → P2 固件端口表+端口级命令 → P3 attach 端到端 → P4 判定与传输正交 → P5 CAN 上游/复位 → P6 多站串口（RS-485，后置））。**CAN 下游端口（拓扑 B：CAN 当桥的一个端口，切片载具）已落地并真机验收**，见[协议页](docs/klippy/developer-manual/serial-mux-protocol.md)的 CAN 下游端口一节 | — |
| M2 | 通过 **admin 流程复位固件**（per-device reset）——**已落地**：`restart_method: admin` 在合法值表，桥自身走 `bridge_admin_restart`、下游端口走 `RESET_PORT`/`RESTART_DEVICE`（三传输都实现）；legacy 走端口节 `uart_restart_method`。余项只是真机极性与保持验证 | M1 |

> 判为**不适用**、不进待办的上游模块：`garbage_collection.py`（Python GC 调优）、
> `aio_executor.py`（Python 线程池）、`parsedump.py`（离线开发工具）、
> `debugcmds.c`（固件调试口）。理由见括注：都是 Python 侧调优或离线/调试工具，与本主机无关。

### H11 主机运行时与调试


## 未决问题

- [ ] **`--tui` 下 SIGTERM 不结束宿主（2026-10-22 实测）**：`kill -TERM <pid>` 后进程仍活 ≥30 s
      （要等窗口自己关），只能 SIGKILL。手册说的是「离开窗口就停宿主」，信号应当同样触发退出；
      现在 `request_exit` 置了旗，但 run loop 卡在 attachment 上。方向：attachment 的 await 与
      退出请求 `select`，收到请求即收窗口。
- [ ] **`M400` 等待的两个后续单元**（等待本体与四条出口已落地：`motion/toolhead.rs` 的
      `wait_moves_state()` 快照 + `extras/toolhead.rs` 的异步等待环；`can_pause = !is_fileoutput()`
      时立即退出，所以语料对该特性零覆盖）：
      ① fileoutput 形态下上游 `_flush_handler_debug` 的等价物未移植（上游在 `extras/motion_queuing.py`，
      `motion/queuing.rs` 无对应分支）；
      ② 上游 `send_wait_ack`（`klippy/mcu.py:125-127`，消费方是传感器芯片）本仓以普通 send 代替——
      理由已写在 `extras/adxl345.rs` 的 `finish_measurements`（发送队列保序，所以不等 ack 也够）；
      只有将来真需要 ack 语义时才要补。

## 上游事件对照清单（34 个已声明，逐项核对发送点）

上游 `Printer` 维护 `event_handlers` 字典（`klippy/klippy.py:36`），通过
`register_event_handler(name, cb)` 注册、`send_event(name, *params)` 分发，共 35 个事件名。

`KlippyEvent` 已声明 34 个名字（上游 35 个里的 `gcode:debuginput_exit` **有意不声明**，见下表），
`Printer::register_event_handler` / `send_event` 按名注册与分发，处理器签名为 `Fn(&KlippyEvent)`。
「实现依赖」列记的是**现在还有没有发送点**：34 个已声明事件里 32 个已在生产路径发出，
只剩 `stepper:sync_mcu_position` / `stepper:set_dir_inverted` 两个仍无发送点（逐项状态 2026-10-22 复核）。

> **「参数」列的口径＝本仓载荷**：按 `src/core/klippy/event/decl/*.rs` 的 `event!` 声明写，
> 裸声明一律写「无」，带字段的写 `{字段}`。上游 `send_event` 的实参（对象引用、位置参数等）
> 见「上游位置」列与[事件系统](docs/klippy/developer-manual/event-system.md) 的上游载荷表；
> 上游传对象引用而本仓 `decl` 为裸声明时，本列写「无」（处理器按需自行 `lookup_object`）。

### 生命周期事件（最高优先级）

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `klippy:mcu_identify` | MCU identify 完成后 | 无 | `klippy/klippy.py:131` | ✅ 已触发（`printer.rs:1076`） |
| `klippy:connect` | 配置装载完成、打印机即将就绪 | 无 | `klippy/klippy.py:132` | ✅ 已触发（`printer.rs:1104`） |
| `klippy:ready` | 打印机进入 ready 状态 | 无 | `klippy/klippy.py:162` | ✅ 已触发（`printer.rs:1115`） |
| `klippy:shutdown` | 进入 shutdown 状态 | 无 | `klippy/klippy.py:211` | ✅ 已触发（`printer.rs:885`、`mcu/object.rs:3274,3323`） |
| `klippy:disconnect` | 运行结束/退出时 | 无 | `klippy/klippy.py:195` | ✅ 已触发（`printer.rs:976`） |
| `klippy:firmware_restart` | firmware restart 前 | 无 | `klippy/klippy.py:194` | ✅ 已触发（`printer.rs:974`） |
| `klippy:notify_mcu_error` | MCU 通信出错时 | `{msg, details}` | `klippy/klippy.py:144,151` | ✅ 已触发（`printer.rs:852`、`error_mcu.rs:364,390`） |
| `klippy:analyze_shutdown` | 进入 shutdown 后分析 | `{msg, details}` | `klippy/klippy.py:216-220` | ✅ 已触发（`printer.rs:886`） |

> **说明**：两个事件由带载荷的变体承载（`KlippyEvent::KlippyNotifyMcuError` /
> `KlippyEvent::KlippyAnalyzeShutdown { msg, details }`）。`analyze_shutdown` 触发时
> `details` 在 MCU 停止路径带 `{mcu, reason, event_type}`；`notify_mcu_error` 已接入
> `bring_up` 中 MCU 连接失败路径；两者都由 `error_mcu` 模块（`extras/error_mcu.rs`）消费。

### MCU 相关事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `klippy:mcu_identify` | 每个 MCU identify 后 | 无 | 上游发送点 `klippy/klippy.py:131`（`mcu.py:797,929,1001` 是**注册**处） | ✅ 已触发（`printer.rs:1076`） |

> 已由 MCU 层事件 `Starting` 覆盖部分语义，但上游的 `klippy:mcu_identify` 是
> Printer 级事件，供 extras（probe、tmc、temperature_mcu 等）做初始化。

### 运动/回零事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `homing:home_rails_begin` | 回零开始 | 无 | `klippy/extras/homing.py:210` | ✅ 已触发（`extras/toolhead.rs:3441,3510,3538,3580,3612,3643,3667`，回零循环与各运动学路径多处） |
| `homing:home_rails_end` | 回零结束 | `{axes, homing}` | `klippy/extras/homing.py:234` | ✅ 已触发（`extras/toolhead.rs:4860`、`gcode_move.rs:1433`、`z_thermal_adjust.rs:880`） |
| `homing:homing_move_begin` | 回零移动开始 | 无 | `klippy/extras/homing.py:80` | ✅ 已触发（`extras/toolhead.rs:4045,4207,4334`、`homing_heaters.rs:377,397,416`） |
| `homing:homing_move_end` | 回零移动结束 | 无 | `klippy/extras/homing.py:148` | ✅ 已触发（`extras/toolhead.rs:4150,4264,4288,4448` 等 8 处） |
| `stepper:sync_mcu_position` | stepper 位置同步 | 无 | `klippy/stepper.py:227` | **未触发**：仅声明（`event/decl/stepper.rs:3`），生产路径无发送点 |
| `stepper:set_dir_inverted` | 方向反转设置 | 无 | `klippy/stepper.py:153` | **未触发**：仅声明（`event/decl/stepper.rs:4`），生产路径无发送点 |
| `dual_carriage:update_kinematics` | IDEx 双滑车运动学更新 | 无 | `klippy/kinematics/idex_modes.py:383` | ✅ 已触发（`idex_modes.rs:725`、`input_shaper.rs:718`，锁外） |

> `homing:*` 四个已随 toolhead 产线触发（`G28` 路径；`homing_heaters` 也会发 move 对）；
> `stepper:*` 两个仍无触发点。

### idle_timeout 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `idle_timeout:ready` | idle_timeout 模块就绪 | `{print_time: f64}` | `klippy/extras/idle_timeout.py:95` | ✅ 已触发（`idle_timeout.rs:467`） |
| `idle_timeout:idle` | 进入空闲状态 | `{print_time: f64}` | `klippy/extras/idle_timeout.py:57` | ✅ 已触发（`idle_timeout.rs:602`） |
| `idle_timeout:printing` | 开始打印（恢复活动） | `{print_time: f64}` | `klippy/extras/idle_timeout.py:106` | ✅ 已触发（`idle_timeout.rs:428,518`） |

> 批 #21 已落地：`[idle_timeout]` 装载即注册，三个事件带 `{print_time: f64}` 载荷发出。

### 工具头事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `toolhead:manual_move` | 手动移动前 | 无 | `klippy/toolhead.py:416` | ✅ 已触发（`safe_z_home` 的 `HomeOps::manual_move`，`extras/safe_z_home.rs:215`） |
| `toolhead:set_position` | 设置位置（G92 等） | 无 | `klippy/toolhead.py:390` | ✅ 已触发（`extras/toolhead.rs:2189,4796`、`gcode_move.rs:1267`、`gcode_arcs.rs:576`、`bed_tilt.rs:573`、`firmware_retraction.rs:472`、`z_thermal_adjust.rs:714`） |
| `toolhead:sync_print_time` | print_time 更新 | `{curtime, est_print_time, print_time}` | `klippy/toolhead.py:267` | ✅ 已触发（`extras/toolhead.rs:2866-2873` 的 `flush_sync_print_time`，在 flush 循环内、锁外） |
| `toolhead:update_extra_axes` | 额外轴位置更新 | 无 | `klippy/toolhead.py:455` | ✅ 已触发（`ToolHeadObject::add_extra_axis` / `remove_extra_axis`，`extras/toolhead.rs:2765` / `:2781`，`manual_stepper` 调用） |

> 四个 `toolhead:*` 事件均已在生产路径触发（`sync_print_time` 由 flush 任务在锁外发出，
> 载荷三字段见 `event/decl/toolhead.rs:10`）。

### gcode 事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `gcode:command_error` | gcode 命令错误 | 无 | `klippy/gcode.py:226` | ✅ 已触发（`gcode.rs:1087`，handler 的 `CommandError` 触发、panic 不触发） |
| `gcode:debuginput_exit` | debuginput EOF | 无 | `klippy/gcode.py:433` | **不做（有意偏离）**：本仓不实现行协议输入，`event/decl/gcode.rs` 也没有这个名字；上游靠 `send_event` 收集返回值 |
| `gcode:request_restart` | 请求重启 | `{print_time: f64}` | `klippy/gcode.py:358` | ✅ 已触发（`gcode.rs:1323`，先 `get_last_move_time` 再 dwell/wait） |

> `gcode:debuginput_exit` 随 `GCodeIO` 一并**不做**（本仓不实现行协议输入；若将来翻案，
> 先要让 `send_event` 收集 handler 返回值，上游是 `all(...)`）。

### 工具/传感器事件

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `probe:update_results` | probe 测量完成 | `{results: ProbeResultsHandle}`（可原地改 Z，批 #36） | `klippy/extras/probe.py:366` | ✅ 已触发（`probe.rs:558`、`probe_eddy_current.rs:1455,1581`、`axis_twist_compensation.rs:1078`） |
| `extruder:activate_extruder` | 切换 active extruder | 无 | `klippy/kinematics/extruder.py:289` | ✅ 已触发（`extruder.rs:369`，`ACTIVATE_EXTRUDER` 处理器内） |
| `stepper_enable:motor_off` | stepper 电机关闭 | 无 | `klippy/extras/stepper_enable.py:120` | ✅ 已触发（`stepper_enable.rs:456`） |
| `virtual_sdcard:reset_file` | VSD 文件重置 | 无 | `klippy/extras/virtual_sdcard.py:151` | ✅ 已触发（`virtual_sdcard.rs:404`） |
| `load_cell:calibrate` | 称重传感器校准 | 无 | `klippy/extras/load_cell.py:404` | ✅ 已触发（`load_cell.rs:938`） |
| `load_cell:tare` | 称重传感器归零 | 无 | `klippy/extras/load_cell.py:406` | ✅ 已触发（`load_cell.rs:856,941`） |

> 六条均已在生产路径触发（不再有“等 sdcard / ADC / endstop 落地”的依赖）。

### 显示/菜单事件（menu.py 内部）

| 事件名 | 触发时机 | 参数 | 上游位置 | 实现依赖 |
|---|---|---|---|---|
| `menu:populate` | 菜单填充 | 无（上游传容器对象） | `klippy/extras/display/menu.py:346` | ✅ 已触发（`display/menu.rs:798`，容器 `populate` 末尾，锁外） |
| `menu:init` | 菜单初始化 | 无（上游传 manager） | `klippy/extras/display/menu.py:722` | ✅ 已触发（`display/menu.rs:1155`，`MenuManager` 构建末尾） |
| `menu:begin` | 菜单开始 | 无（上游传 manager） | `klippy/extras/display/menu.py:754` | ✅ 已触发（`display/menu.rs:1586`，`begin` 推根之前） |
| `menu:exit` | 菜单退出 | 无（上游传 manager） | `klippy/extras/display/menu.py:913` | ✅ 已触发（`display/menu.rs:1809`，`exit` 置 `running=false` 之前） |

> 上游 `menu.send_event` 实际发 `"menu:" + <名>` 并附 `self` 作载荷；本仓四个事件无载荷，
> 处理者按对象名 `menu` 查。四条发送点由 `test_the_lifecycle_raises_the_four_menu_events` 钉住
> （`init` → `begin` → `populate` → `exit` 顺序，`display/menu.rs:2771`）。

### 依赖关系总结

```
事件总线（KlippyEvent，34 个名字已声明）
├── klippy:* 生命周期事件（8 个）—— 全部已触发
├── stepper:* —— **未触发**（位置同步/目录反转变更尚无发送点）
├── homing:* —— 已触发（toolhead 回零路径 + homing_heaters）
├── toolhead:* —— 四个均已触发（含 sync_print_time，出 flush 任务）
├── idle_timeout:* —— 已触发（三个都带 print_time 载荷）
├── gcode:* —— command_error / request_restart 已触发；debuginput_exit 有意不声明
├── probe:* —— 已触发（probe / eddy / axis_twist）
├── extruder:* —— 已触发（ACTIVATE_EXTRUDER）
├── stepper_enable:* —— 已触发（motor_off）
├── virtual_sdcard:* —— 已触发（reset_file）
├── load_cell:* —— 已触发（calibrate / tare）
└── menu:* —— 已触发（四条，菜单引擎内）
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
| `output_pin`（`SET_PIN` / `GCodeRequestQueue` / 模板） | `klippy/extras/output_pin.py:14-270` |
| section 校验用注册表 | `klippy/configfile.py:425-445` |
| mcu 作为 printer object、它的 status | `klippy/mcu.py:1147-1170`、`:1235`、`:938-975` |
| stats 累计与 shutdown 处理 | `klippy/mcu.py:801-802`、`:883`、`:912`、`:974-975` |
| 固件停机/重启事件 | `src/sched.c:310` `:318` `:351`、`klippy/mcu.py:813-835` `:880-881` |
| `config_reset` 与 restart helper | `src/basecmd.c:262-272`、`klippy/mcu.py:754-766` |
| reactor 定时器 / 回调 / 时钟 | `klippy/reactor.py:111` `:145` `:187` |
| reactor latency 钩子 | `klippy/reactor.py:316`、`klippy/extras/garbage_collection.py:45`（`klippy:ready` 注册）/ `:70-74`（latency notifier） |
| kinematics 的装载与接缝 | `klippy/toolhead.py:235-252`、`:389` `:400` `:482` `:507` `:522` |
| 各 kinematics 的差异 | `klippy/kinematics/*.py`（`home` / `check_move` / `calc_position` / `get_status`） |
| 通用回零驱动 | `klippy/extras/homing.py:45-161`（`HomingMove`）/ `:164-300`（`Homing` 状态跟踪） |
| IDEX / 双滑车 | `klippy/kinematics/idex_modes.py`、`klippy/kinematics/cartesian.py:19-30` |
| step 生成层的运动学 | `klippy/kinematics/kinematic_stepper.py`、`rail.setup_itersolve(...)` |
| 惰性装载 | `klippy/extras/adc_temperature.py:51` `load_object(config, 'query_adc')` |
| SPI / I2C 总线 | `klippy/extras/bus.py:9-336`、`src/spicmds.c`、`src/i2ccmds.c` |
| endstop / trsync 触发 | `klippy/mcu.py:155-407`、`src/endstop.c:72-115`、`src/trsync.c` |
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
      `ds18b20`）。
- **上游 extras 消费者 H1–H12（按域）**：加热与温度、风扇与通用输出、宏与脚本、打印流程与 SD 卡、
      TMC 六型、传感器与块状数据、输入与外设（含固件按钮查询与模拟按键）、LCD 与菜单引擎、
      探测/调平/校准（含 `bed_mesh`/`skew_correction`/`z_thermal_adjust`/`tuning_tower`）、
      运动相关 extras、主机运行时调试（`statistics`/`canbus_ids`/`SET_NODEID` 读回校验）、
      核心工具——2026-10-22 全部收官；判为不做的项（板级三件、`get_heater`/`get_sensor`/
      `get_pin_type`）登记在[与上游的偏移](docs/klippy/developer-manual/upstream-deviations.md)的「上游有、本仓不做」表。

- **框架队列 FW1–FW9（全部完成）**：FW1 配置装载、FW2 对象模型、FW3 错误词汇、FW4 G-Code、
      FW5 运动（a–f）、FW6 资源与触发、FW7 MCU 与传输、FW8 主机层与重启、FW9 API；验收以
      「最小模块在 host 单测 + 假 MCU 上跑通」为准，真板项见 [`TESTING.md`](TESTING.md)。
      两个子项 `[~]` 暂缓：`GCodeIO`（不做 OctoPrint 串口仿真，见[与上游的偏移](docs/klippy/developer-manual/upstream-deviations.md)）、
      `MCU_bus_digital_out` 包装（能力已由 `DigitalOut::queue_digital_out` 提供，随显示层接入）。
      分阶段细节见各 FW 的工作记录（已随收官清理，`git log --diff-filter=D -- docs/work-log/` 可找回）；
      已完结的几篇随收口清理（见目录 README 说明）。

- **toolhead 与 kinematics（C1 / FW5a–f）**：Rust 分层重写（不引 FFI）——`Coord` 与
      `clocksync` 回归、`Move`/`LookAheadQueue`/`trapq`、`itersolve`+`kin_cartesian`、
      `MotionQueuing`/`ToolHead`/`McuStepper`、`Kinematics`+`cartesian`+`[stepper_*]`/`[printer]`+`G1`、
      `stepcompress` 完整压缩；`kinematics: none` 随 T1 补上。设计取舍与逐阶段验收见
      FW5 动工前调查（已完结清理）；回零协议的 `get_trigger_position` /
      `set_stepper_adjustment` 随 `endstop_phase` 落地（已到）。

- **重启循环与 `restart_method` 分派（D2 大部）**：`klippy_process` 就地重建
      （`Printer::reset_for_restart` + `load_config` + `bring_up`，同一个 `Arc<Printer>`），
      以及 `command` / `arduino` / `cheetah` / `rpi_usb` 四种物理复位与连接期门控
      （`src/klippy.rs`、`mcu/object.rs`、`mcu/restart.rs`、`interface/usb.rs`）。

- **GCODE 调度器（G1）**：命令表 / `register_mux_command` / `run_script` / 输出处理器 /
      内置命令，`load_config` 里最先注册（`gcode.rs`）；与上游的行为差异集中在
      [与上游的偏移](docs/klippy/developer-manual/upstream-deviations.md)（`GCodeIO` 不做、`get_mutex` 无消费者）。

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

- **主机装配与启动参数**（子系统级）：`klippy_process` 的建机 → `api::register` → bind →
      `load_config` → `bring_up` → `run` 顺序，`[mcu]`-only 与 `[printer]` 两种形态的装载段
      （`DEFAULT_MODULES`/`TOOLHEAD_MODULES`），以及 `StartArgs`
      （`src/klippy.rs`、`api/start_args.rs`、`load.rs`）。


