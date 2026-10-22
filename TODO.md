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
框架队列（FW1–FW9）与上游 extras 消费者（H1–H12）均已完成，索引见文末「已完成（留档）」。
零散待办（无编号）紧跟索引表，细节见各节或各模块文档。

**索引**

| # | 事项 | 依赖 |
|---|---|---|
| H11 | 主机运行时与调试：两条 2026-10-22 运行时小件，见 [H11 节](#h11-主机运行时与调试) | — |
| S2 | 文档补齐（非新功能）：`config.md` 缺 **`[bed_mesh]` 整节**（约 20 个选项，含 `split_delta_z` / `move_check_distance`）；开发手册「模块表 / `testing.md` 覆盖行 / `regression-tests.md`」的其余陈旧行未重审 | — |
| S3 | 真机用例：把 `configure_taking_over`（“连接 ≠ 配置”，遇 `ResetRequired` 则 `reset` + 重开 + 重握手一次）**上提到 `hardware_test`**——现在三份需求（R3/R5/R11）共用一个局部实现（`mcu/resource/endstop.rs` 测试里 + `stress.rs` 自己的），上提后只留一份；同时把 L2（`Machine::bring_up`）已落地这件事写进框架 doc 的模板 | — |
| S4 | 真机用例继续铺（框架两层都已就绪）：R4 归零（无加热，建议先做）→ R9 收尾 / R7 挤出 / R6 温度 / R10 急停（**带加热的最后**）；R8 回放 | S3 |
| S6 | 运行时临时产物收口（**暂缓，2026-10-19 登记**）：`/tmp` 侧有 `kx-start.log`（每次手动拉 klippy 重新生成）、活连接文件 `klippy_uds` 与 Moonraker 的 `moonraker_instance_ids`、harness 的 `pi-bash-*.log`；仓库侧 `.staging/tmp-2026-10-19/` 存着 13 项旧产物（7 个 `klipperx-log-*` 运行目录 + 6 份闸门/启动日志，3.1 MB，已 gitignore）。要做的：按已定路径约定把日志改道（`klippy` 已有 `--logfile` 落 `<data-path>/logs/`，手动启动的 stdout 也应落那里），并给 `.staging` 定一个清理时机。判据：`klippy_uds` / `moonraker_instance_ids` 是**活 IPC 与第三方状态——不迁移、不进释放链**；`pi-bash-*.log` 归 harness；清 `.staging` 前先确认里面的日志不再作任何结论的证据 | — |
| E2 | `python_path` 的取消 | 外部项目 |

**零散待办**（无编号；细节见各节与各模块文档）

- `canbus_stats`：总线错误计数/状态上报。上游 `canbus_stats.py`，本仓无对应模块；
  参见 `H11` 节的 CAN 侧现状（`mcu/ids.rs` 只管 id 分配）。
- `heaters` 切到 `GCodeRequestQueue`（`output_pin`/`fan`/`servo`/`pwm_tool` 已接线，`heaters` 侧无队列化
  消费者），见 [G2b 节](#g2b-用-gcode-控制-gpio剩余)。
- `manual_stepper` 自带回零：`STOP_ON_ENDSTOP` 仍明示拒绝（缺 `drip_update_time`，`drip_move` 只灌主 trapq）。
- `pwm_tool` 的 `maximum_mcu_duration` 中间更新：固件上限照常下发（`setup_max_duration`），上游的
  宿主侧重生成（`MCU_queued_pwm._gen_intermediate_updates`，`pwm_tool.py:119-135`）本仓无对应物，
  见 `extras/pwm_tool.rs` 的模块文档。
- `BED_MESH_OUTPUT PGP=` 补上游的 `(Substitute)` 列（`docs/klippy/developer-manual/README.md` 的 bed_mesh 行尾也记着这条）。
- 真板验证：`S1` 的双机并发压测（`stress.md` 已有单板数据）与 `M2` 的极性与保持。
- TMC 的 SPI 「写 + 回读」切到 `McuSpi::transfer_with_preface`（入口已具备、TMC 未切；收敛判据是
  片选次数不变，见 `docs/klippy/developer-manual/README.md` 的 `tmc_spi.rs` 行与模块文档）。

**串口复用 / 桥（跨主机与固件）**

| # | 事项 | 依赖 |
|---|---|---|
| M1 | 桥下游**端口模块化**与**没有 admin 通道的设备**共存：端口按 `[uart_device]`/`[can_device]` 定义、子设备引用端口节名，归属由主机经桥自己的 admin 通道告知桥（分六期：P1 配置面 → P2 固件端口表+端口级命令 → P3 attach 端到端 → P4 判定与传输正交 → P5 CAN 上游/复位 → P6 多站串口（RS-485，后置））。**CAN 下游端口（拓扑 B：CAN 当桥的一个端口，切片载具）已落地并真机验收**，见[协议页](docs/klippy/developer-manual/serial-mux-protocol.md)的 CAN 下游端口一节 | — |
| M2 | 通过 **admin 流程复位固件**（per-device reset）——**已落地**：`restart_method: admin` 在合法值表，桥自身走 `bridge_admin_restart`、下游端口走 `RESET_PORT`/`RESTART_DEVICE`（三传输都实现）；legacy 走端口节 `uart_restart_method`。余项只是真机极性与保持验证 | M1 |

> 判为**不适用**、不进待办的上游模块：`garbage_collection.py`（Python GC 调优）、
> `aio_executor.py`（Python 线程池）、`parsedump.py`（离线开发工具）、
> `debugcmds.c`（固件调试口）。理由见括注：都是 Python 侧调优或离线/调试工具，与本主机无关。

### G2b 用 GCODE 控制 GPIO（现状与剩余）

`[output_pin <name>]`（`extras/output_pin.rs`）与 PWM 两条链都已打通：`SET_PIN PIN=<name> VALUE=<0..1>`
经 `GCodeRequestQueue`（`extras/gcode_request_queue.rs`）按打印时间生效，`output_pin`/`fan`/`servo`/
`pwm_tool` 已接线；无 `[printer]` 或资源不可调度时保留立即路径（两条兜底与选项细节见
`docs/klippy/developer-manual/README.md` 的 `output_pin.rs` 行与 `pwm_tool.rs` 模块文档）。
剩下的只有一条：

- [ ] **`heaters` 切到同一队列**：`heaters` 侧目前无队列化消费者（`fan` `f2d9844`、`servo`/`pwm_tool` `e3b12a7`
      已于 2026-10-03 切换）。

### H11 主机运行时与调试

- [ ] **共享串口被占用时启动 panic（2026-10-22 真机旁证）**：另一个 `klipperx` 已经占着同一个 tty
      时再起一个宿主，进程以 `thread 'main' panicked at … tokio/runtime/blocking/shutdown.rs:51:
      Cannot drop a runtime in a context where blocking is not allowed` 结束，而不是一条「串口被占」
      的干净错误。复现：`klipperx klippy <cfg>` 跑着时再起同一条命令。方向：失败路径上的 runtime
      在 async 上下文里被 drop，改成干净的 `Err` 返回（上游 `serialhdl` 报 `Unable to open serial
      port`）。
- [ ] **`--tui` 下 SIGTERM 不结束宿主（2026-10-22 实测）**：`kill -TERM <pid>` 后进程仍活 ≥30 s
      （要等窗口自己关），只能 SIGKILL。手册说的是「离开窗口就停宿主」，信号应当同样触发退出；
      现在 `request_exit` 置了旗，但 run loop 卡在 attachment 上。方向：attachment 的 await 与
      退出请求 `select`，收到请求即收窗口。

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
      - `MIN_KIN_TIME`/`kin_flush_delay` 地板**已对齐上游**（`calc_print_time`），且 `kin_flush_delay` 现在是**运行期值**
        （2026-10-21：`ToolHead::recompute_scan_windows` 取全部 stepper 的 pre/post 最大值；常量已改名为
        `SDS_CHECK_TIME = 1e-3` 作为地板），但在本仓 `BUFFER_TIME_START = 0.250` 远大于
        `MIN_KIN_TIME + kin_flush_delay`（常见 PA 下 `0.001 + 0.02`）的前提下
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
| `dual_carriage:update_kinematics` | IDEx 双滑车运动学更新 | 无 | `klippy/kinematics/idex_modes.py:383` | ✅ 已触发（`extras/idex_modes.rs` 滑车切换/恢复，锁外） |

> `homing:*` 四个已随 toolhead 产线触发（`extras/toolhead.rs:2316` `:3236` `:2734` `:2806`，
> `G28` 路径；`homing_heaters` 也会发 move 对）；`stepper:sync_mcu_position`、`stepper:set_dir_inverted`
> 仍无触发点，分别随 stepper 位置同步与目录反转变更。

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
| `menu:populate` | 菜单填充 | 无（上游传容器对象） | `klippy/extras/display/menu.py:346` | ✅ 已触发（`menu.rs` 容器 `populate` 末尾，锁外） |
| `menu:init` | 菜单初始化 | 无（上游传 manager） | `klippy/extras/display/menu.py:722` | ✅ 已触发（`MenuManager::new` 末尾） |
| `menu:begin` | 菜单开始 | 无（上游传 manager） | `klippy/extras/display/menu.py:754` | ✅ 已触发（`begin` 推根之前） |
| `menu:exit` | 菜单退出 | 无（上游传 manager） | `klippy/extras/display/menu.py:913` | ✅ 已触发（`exit` 置 `running=false` 之前） |

> 上游 `menu.send_event` 实际发 `"menu:" + <名>` 并附 `self` 作载荷；本仓四个事件无载荷，
> 处理者按对象名 `menu` 查。四条发送点由 `test_the_lifecycle_raises_the_four_menu_events` 铉住
> （`init` → `begin` → `populate` → `exit` 顺序）。

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
- [x] **隧道的静默顶替与丢单元 —— 已修（2026-10-22）**：`TunnelRegistry::register` 与 `UnitRegistry::register`
      同 id 顶替现在都 `warn!`（带 arb）；`UnitRegistry` 的丢单元两条失败臂（receiver 已掉 / `has`→`deliver` 之间离开）
      也 `warn!`（带 arb、原因、单元字节数）。**顶替语义本身保留**（既有契约用例钉着它，改 `Result` 会波及三条链路的
      setter）；`clear_tunnel_sink` 的「从不调用」仍是有意设计（`bridge_flash.rs` 靠 drop receiver 隐式退订）。
- [x] **桥端口数上限对齐固件 —— 已修（2026-10-22）**：`mcu/object.rs` 新增 `BRIDGE_MAX_PORTS = 4`
      （固件没有字典常量，故主机常量 + 注释指回 `third_party/klipper/src/bridge.c:65`，并注明 `:76-77` 的编译期断言），
      `next_bridge_port_index` 超限即报错并**点名槽号与上限**（此前主机允许 256、只靠固件 shutdown 兜住）。
      钉子：配置期第 5 口报错 / 4 口可载 / 主机常量与固件 `#define` 防漂移。
- [x] **设备层小件批 —— 已落地（2026-10-21）**：`seconds_to_clock` 拒绝负值/NaN/∞（钳位下移到 `step_pulse_duration` 一处）、
      `move_clocks` 发送失败退回槽位、`TriggerDispatch::stop()` 保留原始失败理由（探针失败不再读成主机请求）、
      `trigger_analog` 的 arm 带 `reqclock` 且先停再清了、端停链路故障不再吞掉 `Communication timeout during homing`、
      `stress --task step` 加行程上界（`position_max × 0.5`）+ 三种出口（断线 exit 1）+ `shutdown_timeout` + 有界 `wait_for_idle`。
- [x] **待答调用的两条语义 —— 均已落地（2026-10-21 / 2026-10-22）**：
      ① `cancel` 按 token 精确删（`register` 返回 `PendingCallToken`，计数在登记表内 `Arc<AtomicU64>`、随 clone 共享；
      三条清理路径都按自己那笔的 token 取消）；
      ② `resolve` 对「有同名登记但 oid 不匹配」**不再返回 `true`**：不消费、返回 `false`，消息照常走
      「绑定回调 / `Unhandled` 告警」那条路（对齐上游 `handle_default` 的可见性，此前只留一条 `debug!`）。
      钉子：`test_an_answer_for_another_oid_is_not_swallowed`（假 MCU：oid 8 的应答被投递给该消息上绑定的
      回调、call 仍在等 oid 7；用回调而不是日志做证据，避免依赖接收任务跑在哪个线程）。
- [x] **`hardware_test` 的用例级总超时与 teardown 护栏 —— 已落地（2026-10-21）**：
      `hardware_test::case`/`async_case` 把用例体放自管线程、主线程 `recv_timeout(300 s)` 并打
      `HW-TIMEOUT: <用例>: still at <阶段> after <时长>`；板锁 `LOCK_EX|LOCK_NB` 轮询、`LOCK_TIMEOUT=600 s`
      （大于用例预算，慢前驱不误报）；teardown 先关加热/输出再拆机器；`apply_id_addressed_transport`
      补齐 `host_mux`/`host_port_paths`。
- [x] **注释/文档与实现不符的存量 —— 已修（2026-10-22）**：`mcu/ids.rs` 的「串口 id 无人消费」改成实情
      （它落进 `McuConfig.id`，是传输层开链的前置条件，且与 CAN node id 共用一条序列）；`MoveSlots` 的
      move-class 名单去掉 `set_next_step_dir`（固件 `stepper.c:294-301` 只置标志、不 `move_alloc`，
      本仓也确实走普通发送路径）；`resource/stepper.rs` 的「`step_pulse_duration=0` 让固件自己挑」按
      上游 `stepper.py:108-110` 的实情写成「老 MCU 的 both-edge 模式靠零脉宽开启」。

