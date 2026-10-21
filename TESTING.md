# 最小打印任务 · 真实测试待测列表

本文件只放**必须真实硬件才能做的测试**：按「一次最小打印任务」的执行顺序，排成一张待测清单；
每项写清它做的**真实测试**是什么、怎么算过。host 单测 + 上游语料（字典驱动假 MCU）已覆盖的
行为不重复列——这里只留单测测不到的**电气、物理量级、真机时序、接线拓扑**。清单**不阻塞**
`TODO.md` 的开发任务，约定同 [`AGENTS.md`](AGENTS.md)。

**「最小打印任务」的定义**：一块 STM32F103 板 + 下面的最小接线，从上电连接到打完一个
20×20 mm 单壁方框（单层）再安全收尾——连接 → 归零 → 预热 → 逐行喂 gcode 打印 →
关加热、断使能、退出。**待测列表全绿 = 能完整跑一个最小打印任务。**

## 前置（硬件与运行方式）

- **板子**：STM32F103xe（72 MHz，USB `usb-Klipper_stm32f103xe_39FFD7054D47323924610951-if00`
  → `/dev/ttyACM0`），固件 `out/firmware/klipper/klipper.bin`（仓库根 `make klipper` 产出；
  目标板与引导偏移由 `make menuconfig` 决定（**仓库不带种子配置**，台面板的选法见
  [构建 MCU 固件](docs/klippy/user-manual/firmware-build.md) 末节；含 `config_stepper` 等）。
- **最小接线**：X 与 Y 的 `step_pin`/`dir_pin`、X 的 `endstop_pin`、挤出机步进（E 轴）、
  加热棒 + 热敏电阻（`[extruder]`）。**引脚尚未定论**：本节此前写 X = `PB0`/`PB1`，而
  `config.cfg` 的注释写 `PB4`/`PB5`（且那组注释与 `[output_pin] motor_in1..4` 的
  `PB5`–`PB8` 冲突）——两处都未经实测，写 config 前必须逐个上板确认，Z/热床可选。
- **喂 gcode 的方式**：`virtual_sdcard` 的文件回放（`M20`–`M29`/`SDCARD_PRINT_FILE`）与
  `print_stats` 已落地（2026-10-06 复核：`extras/virtual_sdcard.rs:421-429`、`extras/print_stats.rs`），
  所以「上传 → 开打 → 看进度」的工作流可用。最小任务仍可用**客户端 g-code 模式逐行发**
  （`Ctrl+G`）或 `gcode/script` 端点顶替，便于逐步核对。
- **跑起来**：

  ```sh
  cargo run --release -- config.cfg -a /tmp/klippy_uds   # 起宿主（连真板）
  klipperx console -a /tmp/klippy_uds                    # 另开窗口；^G 进 g-code 模式
  ```

## 待测列表（按一次打印的顺序）

> **已有可跑（自动化）的真机用例**（`KLIPPERX_HW_CONFIG=<你的 printer.cfg> cargo hwtest`——仓库根
> `.cargo/config.toml` 的别名，只跑真机 lane；跑法与两层能力见
> [开发手册 → 测试 → 真机测试](docs/klippy/developer-manual/testing.md#真机测试)）：
> R1 帧序 `test_frame_sequence_sync_against_a_real_board`、R2 重启三轮
> `test_firmware_reset_comes_back_three_times_on_a_real_board`、R3 端停电平
> `test_endstop_level_reads_open_and_shorted_on_a_real_board`、R5 单轴位移
> `test_r5_single_axis_move_matches_the_firmware_step_count`、R11 短时 soak
> `test_r11_short_soak_holds_a_safe_step_rate`。
> 其余项仍需按下面的步骤人工验，或用带真实交互/加热的路径（R4/R6–R10 的自动化用例待铺）。

- [ ] **R1 上电连接与配置下发** —— 真实测试：插板起宿主，走完 identify、字典下发、
  `get_config`/配置 CRC 与板端 `finalize_config`、时钟同步；`STATUS`/`M115` 有应答，
  日志无 `!!`。可加跑真机帧序用例：`KLIPPERX_HW_CONFIG=~/printer.cfg cargo hwtest
  test_frame_sequence_sync_against_a_real_board`。
  判定：会话建立、配置被固件接受、无重传/超时报错。

- [ ] **R2 软复位与会话恢复** —— 真实测试：`FIRMWARE_RESTART`（`restart_method: command`）
  连续做 3 轮，每轮重新 identify + 配置 + 订阅。判定：板子每次都能重启回来，
  API 订阅与 g-code 通道不断（主机层已留档，见下），不出现串口打不开或设备节点漂移。

- [ ] **R3 端停电平与极性（`M119`）** —— 真实测试：X endstop 在**断开 / 手动短接**两种
  物理状态下各读一次 `M119`（或 `query_endstops/status`）。判定：`open` ↔ `TRIGGERED`
  随真实电平翻转，`!`/`^` 极性与上拉符合配置。这是 R4 回零的前置。

- [ ] **R4 归零（`G28`）** —— 真实测试：`homing_speed` 先设小值（如 5 mm/s），
  `G28 X`（接线齐后再 `G28`全轴）。判定：向 `position_endstop` 方向移动、触碰即停、
  回抽后二次回零（`homing_retract_dist`）；`homed_axes` 含 `x`、`position` 落在
  `position_endstop`（`max_error` 内），不撞机不越界。

- [ ] **R5 运动对账（`G1` + `M400`）** —— 真实测试：三轴各走一段已知距离（如
  `G1 X10` / `G1 X0 Y10` / `G1 Z1`），`M400` 后读 `stepper_get_position`。判定：
  电机方向/距离与指令一致，固件步数 = 距离 / `step_dist`（单轴 500 步对账已留档，
  见下；三轴待接线）。单轴冒烟可用
  `cargo run --release -- stress <config.cfg> mcu --task motion` 顶替。

- [ ] **R6 温度链路与加热闭环（`M104`）** —— 真实测试：挤出机加热棒 + 热敏电阻上电，
  `temperature_sensor`/`extruder` 的 ADC 读数与接触温度计对照；`M104 S150` 后看温度
  爬升并稳定、`M104 S0` 后回落。判定：读数误差可解释（同量级、单调）、PID 收敛不
  发散、`verify_heater` 不误触发。**注意**：`M109`/`M190` 已会等到目标温度、`M105` 按
  gcode-id 表输出（2026-10-06 复核：`extras/extruder.rs:263`、`extras/heater_bed.rs:53`、
  `extras/heaters.rs:417`；此前本节「不等温、回 `T:0`」的说法已过时）；`TEMPERATURE_WAIT`
  亦可等待（2026-10-03，`a802cdc`）。

- [ ] **R7 挤出机进料（E 轴）** —— 真实测试：热态下 `G1 E10`（配合 `M83`）送料，
  量实际挤出长度；冷态再发同一条。判定：进料量与 `rotation_distance`/齿轮比对账；
  低于 `min_extrude_temp` 时被 `Extrude below minimum temp` 拒绝（保护生效）。

- [ ] **R8 最小任务回放（单壁方框）** —— 真实测试：用 g-code 模式（或循环调
  `gcode/script`）把一段 20×20 mm 单壁方框的 gcode **逐行喂完**（含归零、预热到位后的
  打印段与 `M400` 收尾）。判定：全程无 `!!` 错误、行间流控不卡不丢、
  `objects/query toolhead` 的 `position` 走完全部路径、打完的方框形状/尺寸与指令一致
  （进度可读 `print_stats`，也可用日志与位置核对）。

- [ ] **R9 打印收尾与静止** —— 真实测试：结尾发 `M104 S0`、`M18`（或等 `idle_timeout`）。
  判定：加热器停在关断值、电机失能、`last_stats`/`stats` 正常上报、宿主保持连接仍能
  接受命令、无残留加热。

- [ ] **R10 急停安全（打印中 `M112`）** —— 真实测试：**再打一遍并在打印中**发 `M112`
  （TUI 里 `Esc`×3）。判定：立即急停——加热器断电（`shutdown_value`）、步进停止、
  宿主报错退出且退出码非零；`FIRMWARE_RESTART` 后能恢复并可继续跑任务。
  这是唯一一条**必须在带加热的真实任务里**验的安全路径。

- [ ] **R11 连续运行的真机时序** —— 真实测试：把 R1–R9 的最小任务**循环跑满一段较长
  时间**（分钟到小时级，或用 `stress --task step` 顶着上限压）。判定：无
  `Stepper too far in past` / `Timer too close`、无丢步、USB 不掉线；本项要暴露的
  代码侧缺口：**宿主的 `ToolHead.wait_moves` 等待已落地（2026-10-06，`a23a92b`）**，但它按上游
  `can_pause` 在 fileoutput 下短路，所以**真机路径（无 `debug_output`）仍未验证**——本项就是要验它。
  尚未建模的剩 `send_wait_ack`（消息的 `min_clock`/`req_clock` 闸**已建模**，见 `mcu/mod.rs:149-176`；
  `mcu/mod.rs:2639-2650` 自述只是 flush、不是「块被固件确认」）。
  另注意：**「送出口径会让最后几批 step 晚到」这条旧假设已无证据**（其唯一来源是假固件链条模型的
  漏算首拍 `interval`，已修），只剩「未证实」；真机应把它当观察点而不是已知缺陷。
  （曾另有一句「`-o` 下不 park 阻塞读、所以语料不受影响」也已被证伪并修掉：含 `[virtual_sdcard]`/
  `[gcode_button]` 的 cfg 在 `-o` 下同样泄漏，代价是那两条各 5.25s 的收尾；根因是命令表反向钉住
  部件的强引用环，2026-10-06 已断——现在这两条收尾回到毫秒级、且探针会点名泄漏的 cfg。）

- [ ] **R12 加热的时钟域与主线程护栏** —— 真机部分验过（2026-10-19，裸 STM32F103 +
  `[heater_generic]` 软件 PWM 脚 + 板载 LED + `temperature_mcu`）：
  **① 跨 32 位时钟绕圈 ✅ 通过**：`power` 恒 1.0 连跑 180 s（F103 72 MHz ⇒ 59.6 s 一圈，跨 3 圈），
  每 2 s 采一次共 90 样本 —— 温度 min 36.24 / max 36.51 / **极差 0.27 ℃**、`power` **90/90 全 1.0**、
  分段均值 36.31→36.36→36.45 单调微升、会话全程 `ready`。改动前（ADC 样本时间是裸 tick）绕圈后
  `time_diff` 变负巨值 ⇒ 平滑温度乱跳 ⇒ watermark 把占空比打 0（LED 会每 ~60 s 闪一下）。
  **② 5 s 主线程护栏在真机上「结构上不可达」——由 6 s 停顿实测证明**：`SIGSTOP` 6 s 后固件先报
  `MCU shutdown: Missed scheduling of next digital out event`（软件 PWM 每 0.1 s 需要一个新边沿，
  宿主停下就漏排；上游的加热器同样是软件 PWM，`heaters.py:61` 写死 `setup_cycle_time(pwm_cycle_time)`），
  也就是说对**真实加热器**来说，0.1 s 级的固件看门狗比 5 s 的主线程截止先咬人，护栏只是纵深防御。
  同一次停顿在修「迟到读数排到过去」之前还会先报 `Rescheduled timer in the past`（见下），修后只剩
  固件的 missed-scheduling。护栏归零时现已留限速告警（每秒读时一条），但正常与停顿两条路都到不了它。
  （补记：对齐上游后已加 **PWM 更新棘轮**——`min_pwm_change = max_power×5%` +
  `next_pwm_time = pwm_time + 3.0 − (3×pwm_delay + 0.001)`，PWM 事件从约 3.3 条/秒降到
  约 0.5 条/秒；重跑 6 s `SIGSTOP` **仍然**以 `Missed scheduling of next digital out event`
  结束——这在软件 PWM 下是上游设计行为（宿主必须持续服务），不是本仓缺陷。）
  **③ 7 s 陈旧压制同样未被真机观测**：恢复后第一条读数即刷新 `last_temp_time`，窗口只有其前几毫秒；
  且 `M105` 文本不经 `gcode/script` 应答回传。②③ 的边界由单测钉住
  （`test_a_reading_past_the_mainthread_deadline_sends_no_power`、
  `test_a_quiet_sensor_quells_to_zero_after_seven_seconds`、`test_a_stall_is_reported_at_most_once_a_second`）。
  **前置修复（同批）**：真机首次跑时 `[heater_generic]` 一连接就 `Timer too close`（上电 3.28 s），
  首因是 PWM 变更走无时钟即时路径、撞上固件的 `MAX_HEAT_TIME` auto-off；改成上游的
  `set_pwm(read_time + pwm_delay)` 排期后 `state: ready`、`power 1.0`。
  **第二层（同批后续修）**：排期后又出现 `Rescheduled timer in the past`（上电 10.4 s）——USB CDC
  把多批报告攒在一起送达时，最后一条读数的 `read_time` 已落后宿主 ~0.3 s，`read_time + pwm_delay`
  正好落在固件时钟之后；现在延迟从「读数与芯片 print time 之较晚者」起算，修后同场景只剩固件的
  missed-scheduling（预期行为）。

## 已完成真机验证（留档）

### 2026-10-19 自动化 lane 首次跑通（裸 STM32F103，桥固件 / mux 控制台）

配置：`/tmp/kx-hw.cfg`（单 `[mcu]` + 一个桥端口节让主机按 id 寻址；`[stepper_x]` 用空闲 GPIO
PB4/PB5/`!PB6`，`endstop_pin: ^PD10`，不接任何负载）。`KLIPPERX_HW_CONFIG=<cfg> cargo hwtest`：

| 用例 | 结果 |
|---|---|
| R1 帧序同步 | ✅ `connect #1: took_over=true at sequence 10`、`#2 adopted sequence 3`，0.88 s |
| R2 复位三轮 | ✅ 每轮 `rebooted, identified and answered get_clock`：765 / 763 / 764 ms，共 2.64 s |
| R5 单轴步数 | ✅ `firmware position: 400 step(s), expected 400`（5 mm @ 10 mm/s，0.96 s） |
| R11 短时 soak | ✅ `5000 steps/s for 2.0 s` → 10000 步（≈125 mm），无 shutdown，2.70 s |
| R3 端停电平 | ✅ **交互式跑通（需人手一次）**：`endstop_pin: ^PD10`（上拉）→ `read 1: TRIGGERED`（悬空被上拉为高）；把 PD10 短到 GND 后回车 → `read 2: open`，输出 `电平翻转确认：TRIGGERED -> open`（44 s，含等人） |

> **接线语义（实测确认，与上游同款）**：`^` 只置上拉、`!` 才置 flip（`pins.py:70-76` ↔ `pins.rs:970-972`）；主机
> `query_endstop` 返回 `pin_value ^ invert`（`endstop.rs:146` ↔ `mcu.py:406`），`true` 打印为 `TRIGGERED`。
> 所以 `^PD10` **悬空 = 高 = TRIGGERED**、**短到 GND = 低 = open**——klipper 的 `^` 端停按常闭接法工作。

**同日修复并复验（串口在重新枚举下的身份）**：改一次 `endstop_pin`（PD10 → PD11 → 再回 PD10）⇒ lane 必须
`reset` 板子 ⇒ USB 重新枚举；此前同一场景是 `serial write … Input/output error (os error 5)` 直接失败，
现在两轮都 `the firmware accepted the configuration` 并读到电平（`^PD10` 短到 GND 时 `read 1: open`）。
机制：`port_for` 的复用判据加了「路径当前解析到的 `(dev, ino)` 与已打开描述符一致」(`same_file`；元数据读不到时**沿用**，2026-10-21)，
`attach` 在握手失败且端口已失效时重开（4 × 250 ms；**打不开也计入一轮**，耗尽后报原握手错误）。

**前置修复（同批）**：这份 lane 以前打不通这块板——`hardware_test`/`stress` 各自只拿到一节就
`McuConfig::open()`，而"这条 `serial:` 路径按 id 寻址"是**装载期**在整份配置上判定的，于是 lane 用
独占裸帧对 mux 固件说话、identify 必然超时（日志里 `serial port …` 没有 `shared` 字样）。现在两处都
调用 `apply_id_addressed_transport`（`serial_ports` 同一分类 + `NODEID_FIRST`），并加了两条单测钉住
"单 `[mcu]` 仍独占 / 桥端口节按 id"。


| 项 | 结果 |
|---|---|
| FW5 完整压缩 `--task motion`（单轴） | ✅ 5 mm/10 mm/s、`step_dist=0.01`：500 步压成 3 条命令，`stepper_get_position` 读回 500 |
| `klipperx stress --task step` | ✅ STM32F103 存活到 ~339 623 步/秒，375 000 步/秒时 `Stepper too far in past` |
| F6 SPI（W25 flash，CS=PA15，SPI1 重映射 PB3/PB4/PB5） | ✅ 硬件 `spi1a` 与软件 bit-bang 都读出 JEDEC `ef 30 13`、状态 `0x00`、地址 0x00 数据 |
| `last_stats`（MCU `stats` 上报） | ✅ 真板确认 |
| `--logfile`、`error_exit` 非零、重启后订阅不断 | ✅（主机层，代码 + 真板启动路径） |
| `[output_pin]` 数字/PWM、`SET_PIN` | ✅ 真板端到端（F3/F4 索引） |
| `temperature_mcu`（MCU 内置温度） | ✅ STM32F103 `[temperature_sensor mcu_temp] sensor_type: temperature_mcu` 读数约 **35.1 °C**（校准 `base=357.558 / slope=-767.442`，`<mcu>:ADC_TEMPERATURE`） |

## M1/M2（桥下游端口 + admin 复位）——**统一真机测试（后移）**

> **P5（CAN）与 P6（RS-485）余项统一按「待测（硬件）」处理，不阻塞主线进度**：
> CAN/485 两类设备的全部未落地项（`[can_device]` 端口后端 `BRIDGE_PORT_CAN`、CAN 上游 attach
> 端到端、`vcan` 夹具、P6 多站串口）都已挂在本节，等硬件就绪再跑；软件侧该做的已做完
> （见台账 §10）。

**测试策略（2026-10-14 拍板）**：M1（桥下游端口模块化）与 M2（admin 流程复位）的**真机/外设项
一律后移**——先让全部代码路径在软件侧（pty + 回环夹具 + 假固件/裸帧库）说得通、单测与集成测
全绿，等 **M1+M2 全部落地**后再把下面这几项**一次跑完**。期间不因缺真机而停下任何一期的开发，
也不把「未上真板」当成未完成（与本文开头「不阻塞 `TODO.md` 开发任务」同一条约定）。

- [x] **M1-1 桥下游端口 attach（legacy）** —— 一块板做桥（固件开 `CONFIG_BRIDGE`），其 uart 端口接
  一台**没有 mux 的旧固件**板（无 admin）。真实测试：宿主按 `[uart_device]` + 无 `uuid:` 的 `[mcu]`
  连上，桥收到 `SET_PORT_NODEID(index, id, flags=0)` 后把该 id 的数据单元剥头下发、把下游原始帧
  加上应答头上行。判定：identify + `get_clock` 跑通，下游板全程不参与 admin，日志无 `!!`。
  **2026-10-15 真机结论**：以「宿主库当桥 + 真板（旧/普通固件）当下游」形态验过（见下面「真机结果」）。
  板做桥那一半已不再缺件：stm32/linux 的 UART 端口后端已落地（2026-10-17 经桥刷写已闭环，
  见「范围外」的「经桥」项），剩下的是**配置拓扑**（`[uart_device]` + `[mcu child]`）的真机 identify，
  归在 M1-5。
- [x] **M1-2 桥下游端口 attach（命名设备）** —— 端口上那台**能答 admin** 的设备带 `uuid:`：主机对桥
  发 `SET_PORT_NODEID(index, id, MUX_PORT_UNITS)`，对设备发 `SET_KLIPPER_NODEID` + 重确认握手，
  之后按整单元原样转发。判定：同上跑通，且能与 M1-1 共存于同一桥（一条端口一种形态）。
  **2026-10-15 真机通过**：真板（stm32f103xe，USB CDC 控制台 + `CONFIG_MUX_SERIAL`，身份
  `e512c60ac0e6` = 其 UID 的 chipid 派生值）当命名下游挂在一份宿主库桥后面：`MCU 'bridge'
  identified: 22 messages` / `MCU 'child' identified: 135 messages` / 库 stderr `bridge serial:
  /dev/ttyACM1 open at 250000 baud (fd N, slot 0)`。仍未验：**配置拓扑下的板做桥**（见 M1-5）、
  两台真设备同桥、真板上的 `irq_save()` 时序与冲突处理。
- [x] **M1-3 端口配置期的显式失败与 `index` 一致**（M1-P7 起取代旧的「端口表对账」）—— 端口由主机在
  配置期用 `config_uart`/`config_bridge_port` 声明，`index` = 端口节声明顺序。判定：把端口 tty
  指向不存在的设备（或写一个不可设的 baud）时，主机在**配置阶段**报出点名错误（含桥名与
  `config_uart` 原因），而不是 init 失败或静默；两个端口节时 `index` 0/1 与线上命令一致。
  **2026-10-15 真机通过**（前半）：`uart_path: /dev/klipperx-does-not-exist` ⇒ `MCU 'bridge'
  shutdown during connect: config_uart: the UART device could not be opened`（点名 + 配置阶段）。
  两个端口节的 `index` 0/1 一致性只由单测覆盖（本机只有一块下游板）。
- [ ] **M2-1 `RESET_PORT` 的电气极性与保持** —— 桥对某端口断言 **DTR\|RTS 250 ms** 能否真把下游
  板复位（取决于板子是 DTR 还是 RTS 敏感、高电平还是低电平有效）。判定：下游确实重启并重新
  identify；同时确认**桥自己**没被按在复位上（连续两条 `RESET_PORT` 也不锁存）。
  **部分结论（2026-10-15，代码证据）**：本机这台 `usb-Klipper_stm32f103xe_…-if00` 是**原生 USB CDC**（klipper 自带 USB 栈），
  固件**不把 DTR/RTS 当复位**（`src/generic/usb_cdc.c:452-459,485-494`：只记 `line_control_state`，唯一消费是「1200 波特 + DTR 低」进 bootloader），
  所以 `RESET_PORT` 的 250 ms 脉冲在这类板子上**静默无效**（内核照常接受 ioctl）⇒ 该项在**原生 USB 板**上判「不适用」，
    其判定只对 **USB-串口桥接板**（DTR\|RTS 接复位脚）有意义；原生 USB 的 legacy 设备（不能答 admin）目前**没有**经桥的硬复位手段（命名/mux 设备走 `restart_method: admin` 的 `RESTART_DEVICE (0x05)`），只能断电 / `rpi_usb`。
  **真机观察（2026-10-15）**：在「宿主库当桥 + 真板（原生 USB CDC）当下游」形态下，`RESET_PORT`
  就是桥对该 tty 做 DTR/RTS 脉冲——命令发出、板子**无任何反应**（不重启、不丢会话），与上面结论一致。

- [x] **M2-2 `RESTART_DEVICE (0x05)` 经桥转发** —— 下游是**能答 admin** 的设备，桥只把 `[0x05][index]` 转发给
  它（不扇出、不本地执行）。判定：只有该端口那台重启，其它端口与桥不受影响。
  **2026-10-15 真机通过（已按拆分后的命令号重验）**：命令族已拆为 **`REQUEST_BOOTLOADER (0x02)`（刷写
  工具用：请引导）与 `RESTART_DEVICE (0x05)`（重启固件）**；`restart_method: admin` 走 **0x05**。
  真机：宿主库当桥 + 真板（USB CDC + mux）当命名下游，`FIRMWARE_RESTART` 后日志
  `MCU 'child': resetting bridge port 0 with RESTART_DEVICE`，且**板子确实重启**（USB 重新枚举：
  `/dev/ttyACM2` → `/dev/ttyACM1`）。
  **本轮修掉的真缺陷**：每个 mux 构建自己也是桥构建（`BRIDGE` 是 `default y if MUX_SERIAL`），而
  `bridge_admin_unit` 对 `[0x02]/[0x05][index]` 原本**无条件消费**——板子自己就是设备时，命令被它
  自己的桥半吞掉、永远到不了本机设备半（表现：桥把单元发到了线上、板子却不重启）。修法：
  **只有 index 指到本机在用端口时才消费，否则穿透给本机设备半**（`src/bridge.c`）。
  仍未验：真桥板带下游端口时的「只那台重启、其它不受影响」；无 bootloader 的板子上 `0x02` 仍是
  设计内 no-op（该命令给刷写工具用）。
- [x] **M2-3 复位后的确认路径（无回执）** —— 端口级命令（`0x02`–`0x05`）都没有回执，主机如何知道复位已发生
  （重握手 / 等 `NEED_NODEID` / 重新 identify）。判定：`FIRMWARE_RESTART` 后会话确实重建，不出现
  「命令发了但设备没重启」的静默假成功；旧固件（不能答 admin）仍回退到 `command`。
  **2026-10-15 真机通过**：恰好拿到「设备没真重启」的反例——主机没有假成功，而是点名报出
  `cannot configure MCU: Failed automated reset: MCU 'child' is still configured (CRC 0x890098f7)`，
  并先走会话接管路径（`Firmware is still in an earlier session (sequence 6); taking it over`）。
- [ ] **M2-4 `REQUEST_BOOTLOADER (0x02)` 的正向验收（需一块带 bootloader 的板）** —— 本机这台 stm32f103ze
  是 `FLASH_START_0000` 且未开 CAN，`bootloader_request()` 没有可走的路径 ⇒ 0x02 目前只验到「命令上到
  线上、设备不响应」这一半（见下面「上板发现的前置条件 B」）。真实测试：换一块下游固件带**可软件进入的
  bootloader**（CanBoot / USB-HID）的板，对它的端口发 `0x02`。判定：该板真的停在 bootloader 里（刷写工具
  能连上，或板子自报 bootloader 内容），桥与同桥其它端口不受影响。
- [ ] **M1-4 CAN 上游 attach（P5 余项）** —— 桥经 CAN 连（固件 `CONFIG_MUX_CAN`，主机需走
  `interface/devices/canserial.rs` 的端口级单元发送路径）。真实测试：先经单帧 admin `0x3f0` 指派
  node id，再经**单元**完成 mux 扫描/采纳（`SET_KLIPPER_NODEID`）、确认本节点回帧带应答头、
  `uuid` 冲突时 `shutdown`；然后一台桥经 CAN 上游带 N 台下游设备（UNITS 口/legacy 口两种）并跑
  `SET_PORT_NODEID`/`RESET_PORT`。判定：同 M1-1/M1-2 的 identify + `get_clock`，且**单元不被截断**。
  **缺件（2026-10-17 复核，已缩小）**：固件的 CAN **下游端口**后端（`BRIDGE_PORT_CAN`）与它的
  单帧 admin 代理**已经落地并真机验过**（见下方「经桥刷 CAN 下游」，F446 桥 + F103 下游，含可反复刷）；
  仍未接线的只剩 **CAN 上游 attach**（桥自己经 CAN 被宿主连）；`printer.cfg` 的 `[can_device]` 端口节
  已接线并真机跑通（主机在配置期发 `config_can` + `config_bridge_port`，见
  [MCU 连接方式](docs/klippy/user-manual/mcu-connection.md) 的 CAN 一节）。
  （桥自己经 CAN 被宿主连）。
  **预判风险**：`mux_input` 在 `irq_save()` 段内对 192 B 收缓冲做就地 memmove（与 `serial_irq.c` 同形），
  CAN 侧帧更密，真机需量一次中断关闭时长与丢帧率；`mux_send` 满缓冲丢单元（协议靠重发，
  admin 单元属点火不管）的频率也需观察。
- [ ] **M1-5 板做桥（配置拓扑：`[uart_device]` + `[mcu child]`）** —— M1-1/M1-2 验的都是「宿主库当桥 +
  真板当下游」。端口后端已不再是缺件：`third_party/klipper/src/stm32/bridge_serial.c`
  （总开关 `CONFIG_STM32_BRIDGE_SERIAL` 下多选 USART1/2/3、UART4/5，勾几个就有几个口）、
  `src/linux/bridge.c`、`src/host/bridge.c`（4 槽）都在，
  且 2026-10-17 已用「真板当桥（USB CDC 控制台 + `MUX_SERIAL`）→ USART1 → 下游」的拓扑把**刷写**
  整链跑通（见「范围外」的「经桥」项）。**待验的是配置面**：真板开 `CONFIG_BRIDGE` + 下游端口，
  宿主按 `[uart_device]` + `[mcu child]`（分别 legacy 与命名各一次）连上去。判定：
  同 M1-1/M1-2 的 identify + `get_clock`，命令经真板的桥半转发，且 `irq_save()` 段内时序无丢帧。
- [ ] **P6 多站串口（RS-485 一类）** —— 端口后端（半双工收发 + 方向控制）尚未实现，主机侧「一个串口
  端口节被多颗命名 `[mcu]` 引用」的分组与判定已有（都带头、各自走自己的 `uuid → id`）。真实测试（待后端
  落地 + 485 硬件/收发器）：一条多站线挂 ≥2 台说单元的设备，各自的 id 分配与数据帧都正确路由；对照：
  单设备 legacy 端口仍是剥头/加头。判定：两侧会话都稳定、无串扰。
- **M1-2 的软件侧等价物已永久入库**（真 `klipper.elf` 当命名下游走真桥，`-U` 身份 = 配置 `uuid:`），
  所以上面 M1-2 真机项只需验**真板/真时序**（扫描/指派/冲突、`irq_save()` 时序、两个真设备同桥）。

### 真机结果（2026-10-15，第一次上板）

板子：`usb-Klipper_stm32f103xe_39FFD7054D47323924610951` → `/dev/ttyACM1`（STM32F103ZE，SWD 刷入：
`./venv/bin/pyocd load --target stm32f103ze out/firmware/klipper/klipper.elf`）。

**为这次上板补的固件改动**（子模组）：USB CDC 控制台接 mux（`src/generic/usb_cdc.c` 的 rx/tx 两个
接点与 `mux_send`）、`MUX_SERIAL` 依赖放宽为 `SERIAL || USBSERIAL`（此前 USB 控制台的板子 `SERIAL=n`，
根本选不了 mux）、以及 `Makefile` 的 `src-y := $(call uniq,$(src-y))`（保留首次出现的**保序去重**；
`chipid.c` 被多载体各列一次 ⇒ 重复定义 + 重复 `compile_time_request`）。板子 `.config` 现为 `CONFIG_USBSERIAL=y + CONFIG_MUX_SERIAL=y`。

**跑成的**：① `klipperx admin scan --port /dev/ttyACM1` → `uuid=e512c60ac0e6`（= 该板 UID 的 chipid 派生，
已用仓库自带 C 版 fasthash64 独立复算一致）⇒ 真机 mux 扫描与身份；② M1-2（宿主库当桥 + 真板命名
下游）整链 identify（bridge 22 / child 135）；③ M1-3（点名配置错误）；④ M2-3（主机对「复位没发生」
的显式判定）；⑤ M2-2 控制路径（`RESTART_DEVICE` 经桥转发）。

**上板发现的前置条件（当结论用）**：A) 想当 mux 下游的板，其控制台必须已接 mux 线（含 USB CDC）；
B) `REQUEST_BOOTLOADER (0x02)` 要真生效，下游固件得带**可软件进入的 bootloader**（CanBoot / USB-HID）
——不带就是设计内 no-op，它属**刷写工具**；「重启固件」用 `RESTART_DEVICE (0x05)`（`restart_method: admin`）；
C) 原生 USB CDC 板不看 DTR/RTS，线级 `RESET_PORT` 对它无效；D) 「板做桥」当时还缺 MCU 侧的 **UART 端口后端**
（`CONFIG_SERIAL_UART` 那时只有 host 库实现）——**现已落地**（stm32 `src/stm32/bridge_serial.c`，多选 USART1/2/3 + UART4/5，
见 [MCU 连接方式](docs/klippy/user-manual/mcu-connection.md) 的多端口一节），这句留作上板当天的记录。

**刷写这台板的正确姿势（本轮踩过，写下来）**：`./venv/bin/pyocd load -O connect_mode=halt
-O frequency=1000000 -e chip --target stm32f103ze out/firmware/klipper/klipper.elf`
——默认频率下刷写会**静默失败**（报成功但板上还是旧镜像），chip erase + halt + 1 MHz 才可靠；
另外**核心处于 sleep 时经 SWD 读内存会全返回 `0xFF`**（必须先 `halt`），否则会误判「flash 是空的」。

**三条语义要分清（真机实测）**：
- `REQUEST_BOOTLOADER (0x02)`＝**请求进 bootloader**（面向刷写/恢复，**不属** `restart_method` 的取值）；无 bootloader 的下游它是设计内 no-op。
- `restart_method: admin`＝经桥转发 **`RESTART_DEVICE (0x05)`**，让下游**命名设备重启自己的固件**（不是进引导）。
- `restart_method: command`（下游桥端口的默认）＝**重启固件本身**：主机把固件的 `reset` 命令当普通帧经**桥按 id 转发**给那台设备（固件侧 `command_reset` → `NVIC_SystemReset()`），**不需要 bootloader**。真机验过：`MCU 'child': resetting the firmware with the 'reset' command` ⇒ 板子确实重启（USB 重新枚举，`ttyACM1` → `ttyACM2`），复位后 `klipperx admin scan --port /dev/ttyACM2` 仍报同一身份 `e512c60ac0e6`（身份是 chipid 派生，跨复位稳定）。
- **USB CDC 下游的复位会拖累宿主库桥，但现在会自恢复**：下游重新枚举期间 `/dev/ttyACM1` 消失（可能换号），
  桥那一侧此时 `config_uart` 打不开 tty ⇒ `MCU 'bridge' shutdown during connect: config_uart: the UART
device could not be opened`。主机会**有界等该端口路径回来**（≤10s、100ms 轮询；`uart_path:` 用
  **by-id/by-path** 这种稳定路径），回来就请求一次 **plain restart**（不是 `firmware_restart`，避免把子设备
  再点一次 `RESTART_DEVICE`）——进程级重启会 `klipper_host_shutdown` + 卸载 + 重新映射 + 重新 `init`，
  并按配置重新登记端口，于是下一轮 bring-up 拿到已回来的 tty ⇒ **自动恢复**。每个 MCU 只有一份恢复
  预算（`Printer::claim_bridge_port_recovery`，成功 connect 后归还）：所以「tty 在、但 open 一直失败」
  （目录/无权限/被占/槽越界）只会重启**一次**、随后稳定停在 error，不会重启风暴；路径始终不存在则
  有界等待后仍点名报错（M1-3 行为不变）。**两条触发都覆盖**：桥自己的 bring-up 失败（预算记在桥节）与
  **设备在 child 连接期间被 0x05 重启**（桥已 up、fd 已死 ⇒ child 自己的 connect 失败；预算记在
  **child 节**）——两者都走同一套「有界等路径 + 一次 plain restart」。**真机全链已验**（2026-10-15）：
  `RESTART_DEVICE` ⇒ 设备重新枚举（`ttyACM2`→`ttyACM1`）⇒ child 侧超时失败（照实报一次 error）⇒
  `the bridge port tty came back; restarting the printer to reopen it` ⇒ plain restart ⇒ **bridge 22 /
  child 135 重新起来、state=ready**（全链约 40s，其中 ~35s 是 child 会话自身的超时）；同一条链在
  没有恢复前是 `Printer error` 终点。仍未覆盖：**in-place reset（`reconnect`）内**的 child 失败不走这条路。

### 真机结果（2026-10-16，mux 设备半直连：隧道 / 送引导 / 重启）

目的：**不摆桥**、只把板子当一个 mux 设备半，先验「设备对新指令的应答对不对」。固件 = klipper
子模组 `test/configs/stm32f103-usb-mux.config` 加 `CONFIG_STM32_FLASH_START_2000=y`（应用偏移
`0x8002000`，与 katapult 对齐），**构建外移**（`KCONFIG_CONFIG`/`OUT` 指自建目录
`target/hw-phase1`，不碰子模组的 `.config`/`out/`）；用 `klipperx admin flash --device <by-id>` 直连
USB 刷入（684 块 + verify，rc=0）。夹具：`src/core/klippy/interface/devices/serial.rs` 的两条
`#[ignore]` 探针，tty 由 `KLIPPERX_HW_MUX_TTY=<by-id>` 给（uuid 默认本板 `e512c60ac0e6`，可
`KLIPPERX_HW_MUX_UUID` 覆盖）。

| # | 验项 | 结果 |
|---|------|------|
| 1 | 发现（未指派）：应答落在 admin 应答 id `0x3f1`、`application=None`、`NEED_NODEID` | ✓ |
| 2 | 指派被采纳：再扫**静默**（号只由主机持有） | ✓ |
| 3 | `SET_PORT_NODEID (0x03)`：设备不执行（桥自身命令），无任何应答 | ✓ |
| 4 | 隧道单元（`01 88 …`）：整段丢弃且流恢复 | ✓ |
| 5 | `RESTART_DEVICE (0x05)`：真重启、号丢失、再扫仍应答同一 uuid | ✓ |
| 6 | `REQUEST_BOOTLOADER (0x02)`：板子重枚举为 `usb-katapult_stm32f103xe_devloop1-if00` | ✓ |

实测耗时：第 1 条 8.91 s、第 2 条 2.41 s。**阶段结束时板子停在 katapult**（第 6 条是最后一步）。

两条踩出来的口径（已写进探针注释）：

- **板子会自发 1 Hz `stats` 帧**（固件字典记作 id `-12`，编码首字节 `0x74`），且上一轮残留的应答头
  会先到；所以探针开端口时必须 `tcflush` 清输入缓冲，并且**盲读第一个单元会把两者粘连误判**——
  解析器必须按 arb 分派、必要时 `drop_to_sync`。
- **隧道单元之后至多吞一帧**：设备解析器遇到内层 katapult 帧（首字节 `0x01` 不是合法长度）就进入
  「等下一个 SYNC」态，紧随其后那一帧被整体吃掉、再下一帧恢复——与主机侧「丢到下一个 `0x7e`、至多
  损失一个单元」同性质，刷写会话的重传（`tries=5`）盖得过它。

### 真机结果（2026-10-17，主机直连 CAN：上游 USB-CAN 桥）

做法：把 F446 桥板刷成**上游 klipper 的 `USBCANBUS`**（`src/generic/usb_canbus.c`，USB id
`1d50:606f`，46 500 B，PB8/PB9 @1 Mbit、12 MHz 晶振、katapult 32 KiB 偏移），内核 `gs_usb` 自动绑定后
得到 `can0`（`ip link set dev can0 up type can bitrate 1000000` + `txqueuelen 128`）。这条路径不需要
额外买 USB-CAN 适配器，而且**桥板自己也是总线上的一个 CAN 节点**（固件声明 `CANBUS_BRIDGE=1`）。

节点固件是关键差异：直连 CAN 的节点**不能带 `MUX_CAN`**（裸 canserial；带头会被主机的帧解析器当垃圾），
而挂在 klipperx 桥 CAN 端口上的节点**必须带 `MUX_CAN`**。两者都用 SBW 刷（见下）。

| 项目 | 结果 |
|---|---|
| `admin scan --interface can0` | 两颗：桥板自己 `56ec3127224a`、下游 `12f912ed3873` |
| `klippy` 附着下游 | ✅ `MCU 'child' identified: 139 messages` |
| `admin flash --interface can0`（切片节点 40 824 B） | ✅ 638 块，但需先 `cansend can0 3f0#02<uuid>` 送引导（工具侧缺口） |
| 单节点 `comm` | **2272 req/s**（2842 崩） |
| 单节点 `step` | **145 455 steps/s**（181 818 崩，`Stepper too far in past`） |
| 单节点 `motion` | ✅ |
| **双节点并发**（桥板自身 + 下游，同一条 can0） | 两颗都 **2272 req/s**（聚合 ≈ 4.5k 往返/秒） |

对比：同样 1 Mbit、同样下游板，主机直连的 `step` 是 **145 455**，而 klipperx 桥的 CAN 端口只有
**15 625**（9 倍差）；说明桥端口的 per-frame 转发在突发命令负载下代价更高，待查。

工具侧两个缺口已补齐（2026-10-18）：`admin reset --interface can0 --uuid <12 位> --bootloader`
能直接从主机侧把板子送进引导（不再要手工 `cansend` 一帧 `0x3f0 [0x02][uuid]`）；`stress` 给连
`canbus_interface` 的 MCU 也从同一条 id 序列分配节点号（不再要手写 `canbus_nodeid:`）。两者的
主机侧验收是单测（`flasher/cli` 与 `stress` 的分配器用例），**未上真机重跑**。

### 真机结果（2026-10-17，四台设备压测轮）

拓扑：**UART 桥** STM32F103（2×USART，`e512c60ac0e6`）+ 槽 0 上的下游 `97abe031988e`；
**CAN 桥** STM32F446（CAN1 PB8/PB9 @1 Mbit，`56ec3127224a`）+ 下游 `12f912ed3873`。
工具：`klipperx stress`（release 构建）+ `klippy`；配置留档在仓库根 `bench-uart.cfg` /
`bench-uart-500k.cfg` / `bench-can.cfg` / `bench-all4.cfg`（未入库，手测用）。

**这一轮压出来的四个缺陷（都已修并在真机复验）**：

| 症状 | 根因 | 修复 |
|---|---|---|
| 换配置后的第二次连接必定 `Input/output error`，重连不起来 | 共享/mux 串口 reset 后把**死 port** 交回（旧 `Arc<Mcu>` 还活着，注册表按路径查回同一个 fd） | `resource_for` 跳过 stopped 资源；写失败即标 stopped；`serial::attach` 拿到新 fd（`cce26bf`） |
| UART 下游每次会话后 mux 身份从 `97abe031988e` 变成 `7eabe031988e`，重连再也握不上手，须重新上电 | **上游 console bug**：`encode_acknak.min_size = 0`，`transmit_room()` 压缩后把 5 字节 ack 写到 `transmit_buf[96..100]`，帧尾 `0x7e` 落在 `mux_identity[0]`（250 kbaud 饱和链路几乎每个 ack 都走压缩分支） | `min_size = MESSAGE_MIN` + ack 短路遵守 `buf_size`（`df4ed17c`） |
| CAN 下游 `identify` 永远超时；scan / 指派 / 送引导 / 隧道刷写全通 | CAN 侧 `can_decode_nodeid()` 给的是**仲裁 id**，被当成 node id 交给 mux 设备半 ⇒ 设备发出的帧头是 `mux_encode_header(0)`（`80 00`，没有地址），也拒收发给它的单元 | 交接米跑 wire 上的 node id（`7ca47634`） |
| `[can_device]` 配置期 shutdown：`Pin PB8 reserved for CAN - can't reserve for bridge_can` | 端口型 build 同一对引脚被 `RESERVE_PINS_CAN` 与 `RESERVE_PINS_bridge_can` 各声明一次 | 引脚由 `src/stm32/can.c` 在 `CONFIG_BRIDGE_PORT_CAN` 下只声明一次（`24cc4eba`） |

**吞吐与余量（`stress`，release 构建）**：

| 拓扑 | 下游 comm 单独 | 与桥并发 | 桥自己 comm | 下游 step 上限 | motion |
|---|---|---|---|---|---|
| UART 250 kbaud | 930 req/s | 1160 | 2270(debug)/2834(release) | — | — |
| UART 500 kbaud | 1814 req/s | 1452 | 2834 | 145 455 steps/s | ✅ |
| CAN 1 Mbit（切片） | 2268 req/s | 2270 | 2834（并发时掉到 1816） | 15 625 steps/s | ✅ |

读法（详见 [MCU 连接方式 → 桥下游链路的吞吐与余量](docs/klippy/user-manual/mcu-connection.md#桥下游链路的吞吐与余量真机实测)）：
250 k 是纯线速瓶颈；并发损耗**不对称**（UART 损被中继侧 ~20%，CAN 损桥自己 ~36%）；CAN 的 step 低是
8 B 帧下 `queue_step` 要拆两帧、压测突发下发超过一段的提前量（真实主机是连续投递，不是突发）。

**CAN 端口运行期会话已闭环**：`identify` → 配置握手 → 周期 `get_clock`，同一拓扑连续两次 `klippy` 都成功。
仍未覆盖：CAN 上游 attach（桥经 CAN 被宿主连）。

### 真机结果（2026-10-18，TMC2209 直接挂在桥板上，单线 UART）

拓扑：F446 桥（USB CDC 控制台 + CAN1）；TMC2209 接在**桥板**上——STEP=PC8、DIR=PC7、
EN=PC13、UART=PC6（单线，收发同脚）、DIAG=PC3。下游 F103 照旧挂在 CAN 端口（它的 PC13 是状态灯）。
配置：`kinematics: cartesian` + `[stepper_x]`（Y/Z 是没接电机的占位轴，只为 cartesian 成立）+
`[tmc2209 stepper_x]` + `[endstop_phase stepper_x]`；`run_current 0.35 A`、`driver_SGTHRS 80`。

**验过的（硬证据）**：

| 项 | 证据 |
|---|---|
| UART 读 | `IOIN 21000041 version=0x21`（TMC2209）、`GCONF 1c0`（`pdn_disable`/`mstep_reg_select`/`multistep_filt`）、`CHOPCONF 14030053`（`toff=3 mres=16 intpol=1`）、`DRV_STATUS cs_actual/stealth/stst` |
| UART 写 | `SLAVECONF 200 senddelay=2`、`IHOLD_IRUN` 随 `SET_TMC_CURRENT` 变化、`SET_TMC_FIELD toff` 改 `CHOPCONF` 后读回 |
| 转动 | `MSCNT` 451→911（驱动侧微步计数）、±100 mm @5 mm/s 连续来回多轮不出错；也能用 `G1` 往复做出"嗡鸣" |
| 相位 | `SET_STEPPER_ENABLE` 后 `mcu_phase_offset=1`、`phase_offset_position=0.0013228`（正好 1 微步） |
| 周期检查 | 1 s 一次，`drv_status` 出现在 `tmc2209 stepper_x` 状态里 |
| DIAG 接线 | 接上后空闲读引脚为**低**（`G28 X` 跑满行程报 `No trigger on x after full movement`）；未接时因内部上拉而"立刻触发" |

**这一轮踩出并修掉的三个缺陷**（`3cd3db4`、`a1062b6`）：

1. **写必须等固件自己的应答**：固件对每一条 `tmcuart_send` 都回一个 `tmcuart_response`（写也回，
   载荷为空），那是"这一帧已发完"的完成信号。主机之前 fire-and-forget，于是只能靠 sleep 猜，
   且那条没人收的应答会被下一条查询吃掉，返回上一帧的状态（表现为 `IFCNT` 不前进而寄存器其实已改）。
2. **使能脚要关掉固件看门狗**：`digital_out` 默认带 2 s `max_duration`，使能后 move queue 空时固件
   直接 shutdown（`Missed scheduling of next digital out event`）。上游 `stepper_enable.py` 会
   `setup_max_duration(0.)`，本仓漏了 ⇒ **任何写 `enable_pin` 的轴都会中枪**（此前 bench 配置都没写它）。
3. **周期检查不能阻塞反应堆**：一次 5 s 的 UART 读超时让反应堆忙 5 s，MCU 错过下一个 digital-out
   截止时间而 shutdown。改成 `spawn` 到运行时，并把 UART 交换超时收到 500 ms（正常交换 ~5 ms）。

**两个发现（当日均未修，2026-10-18 当晚已处理，见 `TODO.md`）**：

- `FORCE_MOVE` / `STEPPER_BUZZ` / `[manual_stepper]` 在真机上**不产生任何 `queue_step`**（`MSCNT`
  不动）；只有 `[stepper_x]` + cartesian 的 `G1` 走完整 host 路径才真的出步。
  **重新归因**：假固件上 host 侧本来就出步（`FORCE_MOVE` +800、`STEPPER_BUZZ` ±800），所以真机症状
  的成因是 ① 电机没使能（EN 未开 / TMC `toff=0`）② `[manual_stepper]` 未进队列 ③ 队列外电机静默空跑。
  ①②已修（自动使能链、manual_stepper 自有 trapq），③改为 `warn!`。**这三条仍需上板逐条复验。**
- `G28` 失败（`No trigger …`）之后再做 `SET_KINEMATIC_POSITION` + `G1`，会撞
  `Internal error in stepcompress: … Invalid sequence` 而 shutdown（正常路径与零长度移动都不触发）。
  **已修**：失败回零没有补上游的收尾三步，导致生成地平线被后台 flush pass 回退；现在地平线取
  `max(两处)` + 未触发分支补收尾。仍需上板复验一次。

**仍未验**：无感回零的**真触发**（这台电机是空悬的，不会堵转，需要负载或手捏轴）与
`tmc/stallguard_dump` 的随负载采样；SPI 族（2130/2240/2660/5160）的真板读写本轮没上板。

### 真机结果（2026-10-18，桥的自恢复：admin 重置不再需要 identify）

背景：`restart_method: command` 是"连上之后发固件 `reset`"，所以它**只能在 identify 之后**生效——板子卡在
陈旧会话、认不出来时正好用不上。真机验过两条 admin 路径，它们都不需要 klipper 会话：

```bash
# 1) 工具路径（不读 printer.cfg）：重置桥自己 / 送它进 katapult
klipperx admin reset --bridge /dev/serial/by-id/usb-Klipper_stm32f446xx_… --bridge-uuid 56ec3127224a
klipperx admin reset --bridge … --bootloader

# 2) 配置路径：桥自己的 [mcu] 写 restart_method: admin，之后 FIRMWARE_RESTART 就走同一条 admin 帧
```

实测（F446，`admin reset --bridge …`，0.99 s）：

```
shared serial … open at 250000 baud
Creating MCU: bridge                                  ← 只开 mux 链路，没有 identify
shared serial … open at 250000 baud                   ← 重启之后的回归确认
MCU 'bridge' identified: 150 messages registered
reset: bridge (restarted and identified again)
```

实测（同一块板，`restart_method: admin` + `FIRMWARE_RESTART`）：

```
05:55:12 bridge … restarted over its admin channel (index 0xff)
05:55:13 MCU 'mcu' identified: 150 messages registered
05:55:13 MCU 'downstream' identified: 139 messages registered     → state: ready
```

两者发的都是 `RESTART_DEVICE [0x05][0xFF]`：`0xFF` 超出任何桥的端口数（`BRIDGE_MAX_PORTS = 8`），
固件的 `bridge.c` 因此不把它当"某个端口的下游设备"，而是落给 `mux_admin_unit` → `mcu_restart()`。
`--bootloader` 同位置发 `[0x02][0xFF]` → `bootloader_request()`（未在本轮跑，避免把桥刷成 katapult 后还要重刷）。

## 范围外的真板项（不属最小打印任务，未排期）

- **双板时序/漂移**：第二块板（`[mcu zboard]`）跨 MCU 停轴、小时级运行的实际晶振漂移与
  USB/CAN 抖动（软件侧已用 ±100 ppm/1 h 模拟；`SecondarySync` 周期重校准）。
- **`rpi_usb` 物理复位**：per-port 断电重启、换电后 identify 不误判、拔插（`restart_method: rpi_usb`，
  代码与决策逻辑已就绪）。
- **未接的外设**：固件按钮查询、`trigger_analog` 宿主节、sdcard、LCD——
  各接一个真实外设后验。（宿主模块已落地但尚未上真板的：`pulse_counter`、`neopixel`/`dotstar`、
  `sensor_bulk` 与各类 SPI/I2C 传感器。）
- **TMC 驱动剩下的真板项**：UART 侧已在 2026-10-18 验过（见上一节），剩下三项：① 无感回零的**真触发**
  （需要带负载的机构，空悬电机不会堵转）；② `tmc/stallguard_dump` 端点在**有负载**时采到随推力变化的
  `sg_result`；③ SPI 族（`tmc2130`/`tmc2240`/`tmc2660`/`tmc5160`）的真板读写**一次都还没上板**。
  **注意**：两条传输层的 `block_in_command` 要求多线程 runtime，真板上不能把命令塞进 current-thread runtime。
- **回零精度数字**：`homing_retract_dist` 二次回零与 `endstop_phase` 相位调节的**实际精度**
  （功能已软件落地，真板只差精度测量）。
- **katapult 刷写真机验证（直连）**：带 katapult 引导的板子（应用带偏移）经 `klipperx admin flash` 直刷——
  USB-CDC 路径（`--device /dev/serial/by-id/usb-Katapult…`：prime → 会话 → verify → `COMPLETE`
  后重枚举）与 CAN 路径（`--interface` + `-u`：admin `0x11/0x12` 指派 + uuid 校验）。判定按
  [flash-firmware](.agents/skills/flash-firmware/SKILL.md) 硬证据口径：刷后设备重枚举为应用
  （USB id 从 `1d50:6177` 回到应用 id）、identify 版本与本地构建一致。
  **实测记录（2026-10-16，UART ROM 首装 katapult 后）**：USB-CDC 路径**已验**——经 `flash` 子命令（当时还是独立 bin，语法未变）
  两轮刷 deployer.bin（83/82 块+verify rc=0），第二轮换血后 `usb-katapult_…_devloop1` 活体
  CONNECT + 读回 `boot==新 katapult.bin`/`app==新 deployer.bin` 双 True（手法见
  [katapult-self-update](.agents/skills/katapult-self-update/SKILL.md)）；**CAN 路径仍未真机**。
  两个镜像现在由仓库根 `make katapult` 产出：`out/firmware/katapult/katapult.bin` 与
  `out/firmware/katapult/deployer.bin`（只在共享目标选了非零引导偏移时才有 deployer）。
- **katapult 刷写真机验证（经桥，2026-10-17 闭环）**：拓扑 = 主机 → 桥（USB-CDC 控制台 +
  `CONFIG_MUX_SERIAL=y`）→ UART1（PA10/PA9）@250000 → 下游。
  **硬前提**（缺一条就跑不通，写进拓扑而不是当成「坑」）：
  - 桥的端口在**硬件 UART** 上：`test/configs/stm32f103-usb-mux-bridge.config`（USB CDC 控制台 +
    `CONFIG_STM32_BRIDGE_SERIAL_USART1=y`），口的引脚恒为 PA10（RX）/ PA9（TX）；
  - 下游挂在一条 **UART 控制台**上，且说单元：跑应用的板子带
    `CONFIG_STM32_SERIAL_USART1=y` + `CONFIG_MUX_SERIAL=y`（`test/configs/stm32f103-serial-mux.config`）；
    停在引导里的板子得是 **mux 版 katapult**（构建时选上 `MUX_SERIAL`）——stock katapult 只说裸帧，
    只能挂 `--legacy` 口；
  - 应用带 katapult 的偏移（`CONFIG_STM32_FLASH_START_2000=y` ⇒ 应用在 **0x8002000**）。

  实测（命令与输出逐条，均为 2026-10-17 本轮真机；四步用法见
  [MCU 连接方式 → 经桥刷下游](docs/klippy/user-manual/mcu-connection.md) 一节）：

  | # | 验项 | 命令（`<桥>` = 桥的 tty） | 实测结果 |
  |---|---|---|---|
  | 1 | 列桥的 UART 槽 | `admin bridge config --bridge <桥>` | `available uarts: 2`、`uart 0: 250000 baud default slot (USART1 PA10/PA9)`、`uart 1: … (USART2 PA3/PA2)` |
  | 2 | 启用端口 | `admin bridge port --bridge <桥> --port 0 --port-nodeid 7` | `target: port 0 nodeid 7, units route, tunnel open` |
  | 3 | 经桥发现 | `admin scan --bridge <桥> --ports 0` | `Found uuid=97abe031988e, Application: CanBoot, port 0` + `Total 1 uuids found` |
  | 4 | 经桥刷下游 | `admin flash --bridge <桥> --port 0 --port-nodeid 7 <fw>` | `complete: 43716 bytes in 684 blocks`（verify 全过） |
  | 5 | 经桥刷 **UART 控制台 + mux** 应用 | 同上 | `complete: 41548 bytes in 650 blocks` |
  | 6 | **可反复刷**（目标正跑应用） | 同上 + `--port-reset bootloader` | `complete: 41548 bytes in 650 blocks`，rc=0 |
  | 7 | 经桥把下游引导自更新为 mux 版 katapult | 同上（写 deployer 镜像） | `complete: 4544 bytes in 71 blocks` |

  判定口径：“`complete: N bytes in M blocks`” **只在 verify 逐块比对全过后才打印**
  （`src/flash/session.rs:391-405`），所以那一行就是「写进去的字节 = 读回来的字节」的证据；
  第 6 行另外证明了“**不需要拔插/手按复位**就能再刷一遍”（`0x02` 经 UNITS 路由把正跑应用的设备
  送回引导，会话的 `CONNECT` 重试盖住引导启动时间）。

  **仍未验（写在明处）**：`--legacy` 口上的 stock katapult（只验了单测夹具）；
  host 库当桥时的多槽并行刷写；`scan --bridge` 的漏报率（只有“一片一板”的拓扑）。

  #### 经桥刷 **CAN** 下游（2026-10-17，拓扑：主机 ─USB→ F446 桥 ─CAN 1 Mbit/s→ F103）

  | # | 验项 | 命令 | 实测结果 |
  |---|---|---|---|
  | 1 | CAN-only 桥列槽 | `admin bridge config --bridge <F446>` | `available uarts: 1`、`uart 0: 250000 baud default slot (CAN1 PB8/PB9)` |
  | 2 | 经 CAN 口发现下游 | `admin scan --bridge <F446> --ports 0` | `Found uuid=12f912ed3873, Application: CanBoot, via port 0` |
  | 3 | 指派 | `admin bridge port … --port 0 --port-nodeid 7 --assign-uuid 12f912ed3873` | `assign: … (sent, no confirmation — katapult does not acknowledge it, which is normal)` |
  | 4 | 经 CAN 隧道刷写 | `admin flash --bridge <F446> … --port 0 --port-nodeid 7 <fw>` | `complete: 43284 bytes in 677 blocks` |
  | 5 | **可反复刷**（目标正跑应用） | 同上 + `--port-reset bootloader --assign-uuid …` | `complete: 43284 bytes in 677 blocks`；刷完扫描见 `Application: Klipper` |
  | 6 | 桥自己进引导 | `admin reset --bridge <F446> --bootloader` | 板子以 `usb-katapult_stm32f446xx_<serial>` 重新枚举，随后 `admin flash --device` 写它 |

  CAN 专属判据：① `0x3f1` 的 `[0x20][uuid][app]` 被包成带端口路径的单元上行（第 2 行）；
  ② 隧道帧的 arb = `0x100 + 2*7`（端口 `node_id`），上行按 `reply_arb_id` 回；
  ③ **CAN 的 node id 是 RAM 态**，所以流程必须是「指派 → 送引导 → 再指派 → 隧道」（第 5 行从应用态
  一次成功，就是这三步都在断言内的证据）；④ 送引导走的是 `0x3f0 [0x02][uuid]` 单帧，与载具无关。

  真机踩过、已修/已写进手册的坑：F446 那块板是 **12 MHz 晶振**（必须 `LOW_LEVEL_OPTIONS` +
  `STM32_CLOCK_REF_12M`，否则 PLL 算错、USB 不枚举）；**F4 的 Kconfig 没有 8 KiB 引导选项**，
  默认 32 KiB ⇒ 应用在 `0x8008000`；**F1 上 USB 与 CAN1 RX0 共用中断向量 20**，F103 做 CAN 桥只能
  用 USART 控制台；`0x02` 必须以**带单元头的单元**发到设备自己的 arb（否则 mux 设备不认）。

## 怎么算清零

每项做到「按上面判定通过 → 把结果（命令、关键日志、数字）写回留档表」即可。真板项
**不阻止**任何 `TODO.md` 任务从「待办」移入「已完成（留档）」。历史清单与更细的旧编号
（T1–T7）见 `git log -- TESTING.md`。
