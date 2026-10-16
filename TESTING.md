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
  → `/dev/ttyACM0`），固件 `third_party/klipper/out/klipper.bin`（含 `config_stepper` 等）。
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
  亦可等待（2026-10-03，`b5da84e`）。

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
  代码侧缺口：**宿主的 `ToolHead.wait_moves` 等待已落地（2026-10-06，`46d6e64`）**，但它按上游
  `can_pause` 在 fileoutput 下短路，所以**真机路径（无 `debug_output`）仍未验证**——本项就是要验它。
  尚未建模的剩 `send_wait_ack`（消息的 `min_clock`/`req_clock` 闸**已建模**，见 `mcu/mod.rs:149-176`；
  `mcu/mod.rs:2639-2650` 自述只是 flush、不是「块被固件确认」）。
  另注意：**「送出口径会让最后几批 step 晚到」这条旧假设已无证据**（其唯一来源是假固件链条模型的
  漏算首拍 `interval`，已修），只剩「未证实」；真机应把它当观察点而不是已知缺陷。
  （曾另有一句「`-o` 下不 park 阻塞读、所以语料不受影响」也已被证伪并修掉：含 `[virtual_sdcard]`/
  `[gcode_button]` 的 cfg 在 `-o` 下同样泄漏，代价是那两条各 5.25s 的收尾；根因是命令表反向钉住
  部件的强引用环，2026-10-06 已断——现在这两条收尾回到毫秒级、且探针会点名泄漏的 cfg。）

## 已完成真机验证（留档）

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
  仍未验的是**板做桥**那一半：MCU 侧还没有 UART 端口后端（`CONFIG_SERIAL_UART` 只有 host 库实现），
  所以「一块板做桥、其 uart 接下游」目前无法配置。
- [x] **M1-2 桥下游端口 attach（命名设备）** —— 端口上那台**能答 admin** 的设备带 `uuid:`：主机对桥
  发 `SET_PORT_NODEID(index, id, MUX_PORT_UNITS)`，对设备发 `SET_KLIPPER_NODEID` + 重确认握手，
  之后按整单元原样转发。判定：同上跑通，且能与 M1-1 共存于同一桥（一条端口一种形态）。
  **2026-10-15 真机通过**：真板（stm32f103xe，USB CDC 控制台 + `CONFIG_MUX_SERIAL`，身份
  `e512c60ac0e6` = 其 UID 的 chipid 派生值）当命名下游挂在一份宿主库桥后面：`MCU 'bridge'
  identified: 22 messages` / `MCU 'child' identified: 135 messages` / 库 stderr `bridge serial:
  /dev/ttyACM1 open at 250000 baud (fd N, slot 0)`。仍未验：**板做桥**（同上缺 UART 端口后端）、
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
  **缺件（软件侧也已声明）**：`[can_device]` 下游端口节与固件 `BRIDGE_PORT_CAN` 端口后端未实现
  （配置期点名拒绝）；本环境无 CAN 外设，固件 `MUX_CAN` 目前只有**构建级**验证
  （`test/configs/stm32f103-can-mux.config` 让闸门真的编那段代码）。无真板时可在
  `CAP_NET_ADMIN` 容器里用 `vcan` 搭两节点（本仓无此夹具先例，属可选）。
  **预判风险**：`mux_input` 在 `irq_save()` 段内对 192 B 收缓冲做就地 memmove（与 `serial_irq.c` 同形），
  CAN 侧帧更密，真机需量一次中断关闭时长与丢帧率；`mux_send` 满缓冲丢单元（协议靠重发，
  admin 单元属点火不管）的频率也需观察。
- [ ] **M1-5 板做桥（MCU 侧 UART 端口后端）** —— M1-1/M1-2 验的都是「宿主库当桥 + 真板当下游」，「一块
  真板当桥、其 uart 接下游」今天配置不出来：`CONFIG_SERIAL_UART`（UART 对象层）只有 host 库后端
  （`src/host/bridge.c`），MCU 侧没有 `bridge_port_ops` 实现。真实测试（待端口后端落地后）：真板开
  `CONFIG_BRIDGE` + 一个下游端口，下游分别接旧固件板（legacy）与能答 admin 的板（命名）各一次。判定：
  同 M1-1/M1-2 的 identify + `get_clock`，命令经真板的桥半转发，且 `irq_save()` 段内时序无丢帧。
- [ ] **P6 多站串口（RS-485 一类）** —— 端口后端（半双工收发 + 方向控制）尚未实现，主机侧「一个串口
  端口节被多颗命名 `[mcu]` 引用」的分组与判定已有（都带头、各自走自己的 `uuid → id`）。真实测试（待后端
  落地 + 485 硬件/收发器）：一条多站线挂 ≥2 台说单元的设备，各自的 id 分配与数据帧都正确路由；对照：
  单设备 legacy 端口仍是剥头/加头。判定：两侧会话都稳定、无串扰。
- **M1-2 的软件侧等价物已永久入库**（真 `klipper.elf` 当命名下游走真桥，`-U` 身份 = 配置 `uuid:`），
  所以上面 M1-2 真机项只需验**真板/真时序**（扫描/指派/冲突、`irq_save()` 时序、两个真设备同桥）。

### 真机结果（2026-10-15，第一次上板）

板子：`usb-Klipper_stm32f103xe_39FFD7054D47323924610951` → `/dev/ttyACM1`（STM32F103ZE，SWD 刷入：
`./venv/bin/pyocd load --target stm32f103ze third_party/klipper/out/klipper.elf`）。

**为这次上板补的固件改动**（子模组）：USB CDC 控制台接 mux（`src/generic/usb_cdc.c` 的 rx/tx 两个
接点与 `mux_send`）、`MUX_SERIAL` 依赖放宽为 `SERIAL || USBSERIAL`（此前 USB 控制台的板子 `SERIAL=n`，
根本选不了 mux）、以及 `Makefile` 的 `src-y := $(call uniq,$(src-y))`（保留首次出现的**保序去重**；
`chipid.c` 被多载体各列一次 ⇒ 重复定义 + 重复 `compile_time_request`）。板子 `.config` 现为 `CONFIG_USBSERIAL=y + CONFIG_MUX_SERIAL=y`。

**跑成的**：① `klipperx scan --port /dev/ttyACM1` → `uuid=e512c60ac0e6`（= 该板 UID 的 chipid 派生，
已用仓库自带 C 版 fasthash64 独立复算一致）⇒ 真机 mux 扫描与身份；② M1-2（宿主库当桥 + 真板命名
下游）整链 identify（bridge 22 / child 135）；③ M1-3（点名配置错误）；④ M2-3（主机对「复位没发生」
的显式判定）；⑤ M2-2 控制路径（`RESTART_DEVICE` 经桥转发）。

**上板发现的前置条件（当结论用）**：A) 想当 mux 下游的板，其控制台必须已接 mux 线（含 USB CDC）；
B) `REQUEST_BOOTLOADER (0x02)` 要真生效，下游固件得带**可软件进入的 bootloader**（CanBoot / USB-HID）
——不带就是设计内 no-op，它属**刷写工具**；「重启固件」用 `RESTART_DEVICE (0x05)`（`restart_method: admin`）；
C) 原生 USB CDC 板不看 DTR/RTS，线级 `RESET_PORT` 对它无效；D) 「板做桥」还需 MCU 侧 **UART 端口后端**
（`CONFIG_SERIAL_UART` 目前只有 host 库实现）。

**刷写这台板的正确姿势（本轮踩过，写下来）**：`./venv/bin/pyocd load -O connect_mode=halt
-O frequency=1000000 -e chip --target stm32f103ze third_party/klipper/out/klipper.elf`
——默认频率下刷写会**静默失败**（报成功但板上还是旧镜像），chip erase + halt + 1 MHz 才可靠；
另外**核心处于 sleep 时经 SWD 读内存会全返回 `0xFF`**（必须先 `halt`），否则会误判「flash 是空的」。

**三条语义要分清（真机实测）**：
- `REQUEST_BOOTLOADER (0x02)`＝**请求进 bootloader**（面向刷写/恢复，**不属** `restart_method` 的取值）；无 bootloader 的下游它是设计内 no-op。
- `restart_method: admin`＝经桥转发 **`RESTART_DEVICE (0x05)`**，让下游**命名设备重启自己的固件**（不是进引导）。
- `restart_method: command`（下游桥端口的默认）＝**重启固件本身**：主机把固件的 `reset` 命令当普通帧经**桥按 id 转发**给那台设备（固件侧 `command_reset` → `NVIC_SystemReset()`），**不需要 bootloader**。真机验过：`MCU 'child': resetting the firmware with the 'reset' command` ⇒ 板子确实重启（USB 重新枚举，`ttyACM1` → `ttyACM2`），复位后 `klipperx scan --port /dev/ttyACM2` 仍报同一身份 `e512c60ac0e6`（身份是 chipid 派生，跨复位稳定）。
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

## 范围外的真板项（不属最小打印任务，未排期）

- **双板时序/漂移**：第二块板（`[mcu zboard]`）跨 MCU 停轴、小时级运行的实际晶振漂移与
  USB/CAN 抖动（软件侧已用 ±100 ppm/1 h 模拟；`SecondarySync` 周期重校准）。
- **`rpi_usb` 物理复位**：per-port 断电重启、换电后 identify 不误判、拔插（`restart_method: rpi_usb`，
  代码与决策逻辑已就绪）。
- **未接的外设**：固件按钮查询、`trigger_analog` 宿主节、sdcard、LCD、tmcuart——
  各接一个真实外设后验。（宿主模块已落地但尚未上真板的：`pulse_counter`、`neopixel`/`dotstar`、
  `sensor_bulk` 与各类 SPI/I2C 传感器。）
- **回零精度数字**：`homing_retract_dist` 二次回零与 `endstop_phase` 相位调节的**实际精度**
  （功能已软件落地，真板只差精度测量）。
- **katapult 直连刷写真机验证**：带 katapult 引导的板子（应用带偏移）经 `flash` bin 直刷——
  USB-CDC 路径（`--device /dev/serial/by-id/usb-Katapult…`：prime → 会话 → verify → `COMPLETE`
  后重枚举）与 CAN 路径（`--interface` + `-u`：admin `0x11/0x12` 指派 + uuid 校验）。判定按
  [flash-firmware](.agents/skills/flash-firmware/SKILL.md) 硬证据口径：刷后设备重枚举为应用
  （USB id 从 `1d50:6177` 回到应用 id）、identify 版本与本地构建一致。现有测试板为
  `FLASH_START_0000` 无引导，需先经 SWD 刷入 katapult 并把应用改偏移构建。

## 怎么算清零

每项做到「按上面判定通过 → 把结果（命令、关键日志、数字）写回留档表」即可。真板项
**不阻止**任何 `TODO.md` 任务从「待办」移入「已完成（留档）」。历史清单与更细的旧编号
（T1–T7）见 `git log -- TESTING.md`。
