# 工作记录

本目录存放一次性的**分析 / 盘点 / 审计记录**，不是规范，也不随实现推进而更新。
规范与执行清单以 `docs/klippy/` 与根目录的 `TODO.md` 为准。

正文记的是写作当天的盘点与判断，**不随后续实现改写**；每篇末尾的「实施记录」（§6）是唯一
随实现推进更新的回填节——余项与现状以它为准，仍留在本目录的记录都还有未收口的余项。

下表按**创建时间**（首次入库时间）从早到晚排列。对应任务**已完结**的记录会从本目录
清理掉，需要时从 git 历史找回（`git log --diff-filter=D -- docs/work-log/`）。

| 创建时间 | 记录 |
|---|---|
| 2026-09-23 09:10 | [H2（风扇与通用输出）动工前调查](2026-09-23-h2-notes.md) — 上游 fan/heater_fan/controller_fan 等 21 个文件的依赖盘点、344 次「运行×缺口」测算、GCodeRequestQueue/heater 注册表/tachometer 三处差距与 H2-1…H2-7 拆分 |
| 2026-09-23 10:03 | [G4/H10（gcode_move）动工前调查](2026-09-23-gcode-move-notes.md) — 上游 g-code 坐标系的状态/命令/事件模型、`G0`/`G1` 从 toolhead 搬家的差距、`Move out of range` 49 次的算账、G4-1/G4-2 拆分与六个拍板点 |
| 2026-10-16 07:10 | [CAN ISO-TP / CAN-FD 动工前调查与实施规划](2026-10-16-can-isotp-notes.md) — **按设备类型选路**（FD 设备＝`cans+FD`；标准 CAN 设备＝`isotp+classical`；`isotp+FD` 保留可能性不做）：ISO-TP 字段级规格（含 FD 单帧转义式 62 B、超时取 25 ms 的依据）、**帧数开销对照（classical 下 `queue_step` 档 2×，FD/cans 下 1 帧/消息）**、CAN-FD 落地细节（`struct canbus_msg` 8→64 的逐处清单、三份 `fdcan.c` 寄存器级缺口、哪些架构不能 FD、主机侧 72B `canfd_frame` 落点、六个未验证项）、主机/固件/配置面/linux 四处文件级落点与十四条风险。**实施记录（§10）**：主机侧全部落地（FD 链路 / ISO-TP 纯状态机 / 设备层 / 两个配置键）+ 固件两半落地（`CONFIG_ISOTP` 从站、`LINUX_CANSERIAL` SocketCAN 控制台）+ **vcan0 上真 `klipper.elf` 完整 `[mcu]` 连接跑通**；仅 CAN-FD 固件半边挂起 |
| 2026-10-16 23:20 | [MCU 架构的桥端口后端（`SERIAL_UART` / `uart_backend_*`）规格](2026-10-16-bridge-port-backend-spec.md) — 只读侦察留档：桥端口后端今天只有 host 库实现（`src/host/bridge.c`），`src/Kconfig:200-207` 的注释即「An arch selects this once it can hand a UART object out as a bridge port」；本报告给出契约面（`bridge_port_ops`/`uart_backend_*`，架构无关、不用改）、MCU 侧要补的东西（3 个后端函数 + 端口 ops + 自加 `DECL_TASK`，因为 host 的 `bridge_poll()` 挂在 `irq_wait()` 上）、stm32 现有 UART 资产能否复用、Kconfig 接线（架构只需 `select BRIDGE_PORT_SERIAL`）、槽号↔UART 与 baud 来源、`reset` 无 modem 线只能 `NULL`、**linux 架构可离机端到端**（桥=linux 固件进程 + pty 端口 + 假 bootloader）与建议单元拆分。**状态：实现待派** |
| 2026-10-17 | [桥接 CAN 下游端口（拓扑 B）动工前规划](2026-10-17-bridge-can-plan.md) — 用户拍定拓扑 B（桥仍 USB CDC，CAN 收发器当它的一个下游口），端口上两条载具都要：**CAN 流（8 字节切片）与 ISO-TP（整帧一条消息）**。含事实基线（两条载具都在、`ISOTP` 只换载具不换寻址、STM32 上 CAN 属控制台 `choice` 所以与 USB 互斥、`MUX_SERIAL`/`MUX_CAN` 的 `mux_send` 只能有一份、单 CAN 控制器与只装三个 id 的过滤器）、**五个设计裁定 D1–D5**（端口后端不 select `CANSERIAL`/`MUX_CAN`、CAN 引脚拆出 choice、载具用 `flags` bit1 `MUX_PORT_ISOTP` 声明且隧道恒切片、legacy/tunnel 口的 arb 取自端口 `node_id`、过滤器改为可追加）、**U1–U19 任务单元**（固件切片后端→ISO-TP 端口→主机 attach 放开→`admin` 四命令→测试与文档闸门）、真机设备清单（桥板+收发器 PB8/PB9、下游 CAN 板、120Ω×2、可选 USB-CAN）与预排命令序列。**状态：规划定稿，实现待派；真机另开一轮** |
| 2026-10-16 10:38 | [Katapult 拓扑刷写：动工前调查、决策与任务拆分](2026-10-16-katapult-flash-task.md) — **引入 katapult（子模组已入主线 `d6a66c4`）+ 自研拓扑刷写工具/桥隧道**的决策论证；兼容性六维裁定（帧层三道闸不互混、寻址逐位同值、bootloader 态握手=超时而非误触发、共存安全但送不到、串口/CAN 协议帧同字节非 1:1、上游边界一致）；缺口清单（共享口下行双重封装→unit 形态两步、桥回程刷写隧道模式）与 **T1–T5 任务拆分**、三个拍板点、五条未验证风险；附三路侦察全文（file:line 级）。**阶段 1 已真机通过（六项全绿）；T3、T5a/T5c/T5b′/T5-tool/T5d-1/T5d-2/T5d-3 与 B1/B2/B3 均已验收合入（见 §8）**；余**阶段 2 真机**（`klipperx flash --bridge … --port … --port-nodeid …`，板子停在 katapult 等下游接线） |
