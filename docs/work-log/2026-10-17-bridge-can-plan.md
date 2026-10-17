# 桥接 CAN 下游端口（拓扑 B）：动工前规划

**状态**：**全部单元已落地并真机验收**（F446 桥 + F103 下游、1 Mbit：发现 → 指派 → 送引导 → 隧道刷写 → 反复刷全通；运行期 identify/配置握手/comm 2266 req/s/step 15 625 steps/s/motion 全绿）。**拓扑由用户拍板：B —— CAN 当桥的下游端口**（桥仍是 USB CDC 控制台 + 一个 CAN 收发器当它的一个端口，像今天的 USART 端口那样），端口上跑的就是今天的**CAN 流（8 字节切片）** 载具——**端口只有这一种载具**：曾经的多帧载具已按用户裁定整体删除，代码留在废案台账（[`docs/abandoned.md`](../../abandoned.md)）。

本文件是任务书来源：每个单元（U1…U19）就是一份最小 worker 任务书；实现细节改动先改本文件。

---

## 1. 事实基线（全部有源码依据，动手前不必再查）

### 1.1 载具是 CAN 流（8 字节切片）

| 载具 | 开关 | 固件 | 主机 | 语义 |
|---|---|---|---|---|
| CAN 流（8 字节切片） | `CANSERIAL`（arch 的引脚项 `select`） | `third_party/klipper/src/generic/canserial.c` | `src/core/klippy/interface/devices/canserial.rs`（31 测） | 字节流按 ≤8 B 切帧，无 PCI/序号/长度头 |


### 1.2 三个结构约束（决定了本规划的形态）

1. **CAN 在 STM32 上目前是控制台 `choice` 的一项**：`third_party/klipper/src/stm32/Kconfig:390-450`
   把 USB / 串口 / CAN 三选一（各自 `select USBSERIAL`/`SERIAL`/`CANSERIAL`）。所以「USB CDC 控制台 +
   CAN 下游口」现在**编不出来**——这是拓扑 B 的第一块石头。
2. **一个 build 里 `mux_send` 只能有一份实现**：`MUX_SERIAL` 的实现在
   `src/generic/serial_irq.c`，`MUX_CAN` 的在 `src/generic/canserial.c`；`src/Kconfig:184-189`
   自己写明「两者实践中从不同时成立（arch 的 `choice` 保证）」。
   ⇒ **CAN 端口后端绝不能 `select MUX_CAN`**，否则与 USB 控制台的 `MUX_SERIAL` 撞符号。
   它也不该 `select CANSERIAL`：CAN 端口是**路由器的对端**，不是本节点的字节流。
3. **一个 build 只有一个 CAN 控制器**：`src/stm32/can.c:57-97`、`src/stm32/fdcan.c:63-77`
   都按引脚选一个 `SOC_CAN`，`canhw_set_filter` 每次重写**全部**过滤器（`stm32/can.c:136-172`），
   且过滤器只装 `0x3f0` + 自己的 `id`/`id+1`。桥要收下游 node 的帧，必须新增「追加过滤器」接口。

### 1.3 端口后端的 seam（拓扑 B 要填的就是这两个）

| 层 | 位置 | 要求 |
|---|---|---|
| 路由层 | `third_party/klipper/src/bridge.h:27-43` `struct bridge_port_ops { write, reset, ctx }` | `write` 必填、必须**整块入队永不交错**（`bridge.h:30-32`）；`reset` 可空（CAN 无复位线） |
| UART 对象层 | `src/uart.h:39/45/50` `uart_backend_open` / `uart_backend_port_ops` / `uart_backend_set_port` | 三个都必须存在（`src/uart.c:54/89/94`），**每 arch 只有一份符号** |
| 端口号 | `BRIDGE_UART_COUNT`（`DECL_CONSTANT`） | 主机「列口」的唯一数据源；一个 build 只能声明一次 |

现有三个后端（`src/stm32/bridge_serial.c`、`src/linux/bridge.c`、`src/host/bridge.c`）各占一个 arch，
所以**加 CAN 后端不是「再加一个文件」，而是先要把 arch 的三个符号改成分派层**。

### 1.4 主机侧：连接面缺第三变体，数据面已对称

- 数据面已就绪：`TunnelLink` 三个实现（`src/flash/bridge.rs:52/58/64`，其中有
  `impl TunnelLink for CanSerialDevice`）、端口级 5 条 unit 命令在 CAN 上已有
  （`canserial.rs:764-812`）、隧道上行 sink（`canserial.rs:838`）、`TunnelBridge`/session 与链路无关。
- 缺的集中在三处「按串口/`SharedSerialHandle` 写死」：`flash/cli.rs` 的 `BridgeLinkSource`
  （`:424-441`，只有 Serial/Library）、`flash/bridge_flash.rs` 的 `BridgeDevice`/`BridgeOpen`/
  `discover_bridge_uuid`/`shared_handle`、`scan.rs:376-462` 的 `bridge_scan_pass`。
- **本期这些不动**：拓扑 B 的宿主链路仍是串口/host 库；CAN 只在**下游端口**一侧出现。
  上述第三变体属拓扑 A，留作后续（受益但不阻塞）。

### 1.5 下游固件形态（真机矩阵的根据）

- **CAN 流 + 应用**：klipper `CANSERIAL`（±`MUX_CAN` 让设备说单元）——`test/configs/stm32f103-can-mux.config` 是现成构建闸门。
- **CAN 流 + 引导**：stock katapult 的 CAN 载体就是裸切片（`third_party/katapult/src/generic/canserial.c:56-78` 发、`:225-238` 收），
  **katapult 侧零改动**即可被桥当 legacy/tunnel 口刷——前提是桥的 CAN 端口后端按「裸切片 + 该口 node_id 的 arb」行事。

---

## 2. 四个设计裁定（本规划的前提，若有异议先改这里再动代码）

| # | 裁定 | 理由 | 代价 |
|---|---|---|---|
| **D1** | CAN 端口后端 = 新文件 `src/generic/bridge_can.c`，只依赖 `canbus.h` 的板级钩子；**不 select `CANSERIAL`、不 select `MUX_CAN`**，只 `select CANBUS` 拉驱动 | 避开 `mux_send` 双实现（§1.2-2）；端口不绑本节点 id，才能听多个下游 id | 端口后端自己管过滤器与 rx 分桶 |
| **D2** | CAN 引脚从控制台 `choice` 里解耦：新增 `STM32_BRIDGE_CAN_PB8_PB9`（低层选项）等端口专用引脚项，`select CANBUS` + `select BRIDGE_PORT_CAN`，与 USB 控制台共存 | 拓扑 B 的全部意义就是「USB 控制台 + CAN 端口」；但 **F1 上做不到**：USB 与 CAN1 RX0 是同一个中断向量（20，`USB_LP_CAN1_RX0_IRQn`），上游同一规则（`HAVE_STM32_USBCANBUS depends on !MACH_STM32F1`）。⇒ F103 要 CAN 端口就拿 USART 做控制台（需 `LOW_LEVEL_OPTIONS`）+ USB-UART 转换器；要 USB 控制台就换 F4（USB OTG + CAN1） | 对上游 Kconfig 的偏离，必须进 `upstream-deviations.md`；Kconfig 里写成 `depends on !(MACH_STM32F1 && USBSERIAL)` |
| **D4** | 端口的 CAN arb 来源：**UNITS 口从单元头解**（`mux_decode_header`→`mux_node_id_from_arb`→`mux_arb_id`）；**legacy/tunnel 口用端口记的 `node_id`**（`SET_PORT_NODEID` 两种形态都记 id，协议页 `:298-316` 已如此） | `bridge.h` 的 `write` 拿不到 arb；legacy 剥头后 arb 丢失 | `bridge.h` 加只读访问器 `bridge_port_node_id(int)`（子模组小改，本规划已批准） |
| **D5** | 过滤器扩为**可追加**：新增 `canhw_add_filter(uint32_t arb)`（上限固定，bxCAN 走 bank 池，linux 空实现），`canhw_set_filter` 保持原义 | 下游每个 node 要两条 arb（`0x100+2n`、`+1`），单 id 接口不够 | 改 `generic/canbus.h` + 各 arch `can.c`（子模组改动，本规划已批准） |

**明确不做**：CAN-FD 固件半边（`struct canbus_msg` 8→64，另案）、拓扑 A（CAN 当桥上游）、
RS-485/P6、`[can_device]` 的多设备一端口（一 CAN 口一 node 为第一版上限）。

---

## 3. 任务单元（到真机测试前）

### P1 固件：CAN 端口后端（切片载具）

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U1** | **端口后端分派层**：arch 的 `uart_backend_open`/`uart_backend_port_ops`/`uart_backend_set_port` 与 `BRIDGE_UART_COUNT` 收归一处（新 `src/stm32/bridge_port.c`），按 slot 分派给 UART 后端（`bridge_serial.c` 内改名为 `bridge_serial_open` 等）与 CAN 后端；`BRIDGE_UART_COUNT` = UART 槽数 + CAN 槽数，**只声明一次** | `third_party/klipper/src/stm32/{bridge_port.c,bridge_serial.c,Makefile}` | 现有串口桥构建（`stm32f103-usb-mux-bridge.config`）编过；`BRIDGE_UART_COUNT` 在字典里仍唯一；槽序 = UART 在前、CAN 在后，写进注释 |
| **U2** | **CAN 端口后端本体**：`uart_backend_open`（CAN 忽略 `baud`，只接受「CAN 槽」；占用则 `UART_OPEN_BUSY`）、`port_ops.write`（解单元头→arb→≤8 B 切片→`canbus_send`；发不出去就整块留队重试，永不切块）、`reset = NULL`、`set_port` 记 index | 新 `src/generic/bridge_can.c` | 单元测试级：给定「单元字节 → 期望帧序列（arb+分片）」的纯函数可测；`canbus_send` 返回 -1 时不丢块（桩函数断言重试） |
| **U3** | **Kconfig 解耦 + 构建开关**：新增 `BRIDGE_PORT_CAN`（`select UART`）；STM32 侧端口专用引脚项（D2，PB8/PB9 先做，其余按需）`select CANBUS` + `select BRIDGE_PORT_CAN`；**不 select `CANSERIAL`/`MUX_CAN`** | `src/Kconfig`、`src/stm32/Kconfig`、`src/{Makefile,stm32/Makefile}` | 新 config 能同时开出 `USBSERIAL` + `MUX_SERIAL` + `BRIDGE_PORT_SERIAL` + `BRIDGE_PORT_CAN`，且 `mux_send` 只有一份（链接通过） |
| **U4** | **过滤器追加接口**（D5）：`canhw_add_filter(arb)`；bxCAN 的 bank 分配（`0x3f0`+自身 id 之外追加下游 id 对）、`fdcan.c` 同形、linux 空实现、rp2040/lpc176x 至少编译通过 | `src/generic/canbus.h`、`src/{stm32/can.c,stm32/fdcan.c,linux/can.c,...}` | 每 arch 编译 0 告警；超出上限时报错（`shutdown` 附带原因），不静默丢 |
| **U5b** | （**下一单元，真机的前置**）**CAN 单帧 admin 代理**：桥代发 `0x3f0`（查询 `[0x00]`、指派 `[0x01\|0x11][uuid6][nodeid]`、清号 `[0x12]`），并把 `0x3f1` 的 `NEED_NODEID` 应答（`[0x20][uuid6][app]`）**包成带端口路径的单元上行**（`mux_unit_with_path` 同形）；端口开时无条件装 `0x3f1` 过滤器。没有它，未指派的 CAN 下游设备无法被发现/指派，`admin scan --bridge --ports <can槽>` 与首次刷写均不成立 | `src/generic/bridge_can.c`（下行代发 + 上行包装）、`src/bridge.c`（广播分发时识别 CAN 端口）、`src/stm32/can.c`（过滤器） | 单元级：查询→应答→指派→重扫四步的帧序列测试；未指派设备经桥能取到 uuid 与端口路径；重复指派幂等 |
| **U5** | **收侧分桶 + arb 来源**：`canbus_process_data` 在 `CONFIG_BRIDGE_PORT_CAN` 时把「下游 arb」的帧交给 bridge_can 的 per-node 重组缓冲（**不进** `CanData.receive_buf`），凑齐一个整单元/整块后 `bridge_port_input(index, …)`；`bridge.h` 加 `bridge_port_node_id(int)` 供 legacy/tunnel 口取 arb | `src/generic/canserial.c`（分派点）、`src/generic/bridge_can.c`、`src/bridge.{h,c}` | 纯函数测试：两个 node 的帧交错到达 → 两条独立流各自完整；无 node_id 的 legacy 口收到数据不 panic（丢弃 + 计数） |
| **U6** | **构建闸门配置**：`test/configs/stm32f103-usb-can-bridge.config`（USB CDC + `MUX_SERIAL` + 两个 USART 端口 + CAN 端口） | `third_party/klipper/test/configs/` | 闸门 `cargo test --workspace` 里被编过（现有机制按目录遍历，`crates/test-support/build.rs:151-197`），字典能产出 |
### P3 主机：配置面与 attach

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U10** | **放开 `[can_device]` 端口**：`load_can_device` → `wire_bridge_port`、`CanPort::index()`；`bridge_attach::resolve` 去掉 CAN 拒绝（含错误文案与 `:732-744` 的探针测试改成「可接」） | `src/core/klippy/mcu/{bridge_ports.rs,bridge_attach.rs}` | 配置层测试：`[can_device]` + `[mcu child]` 能构建端口请求；旧的「点名拒绝」断言全部改掉（`grep BRIDGE_PORT_CAN` 归零，文档另行同步） |
| **U11** | **CAN 端口的 `config_uart` 语义**：slot 规则（UART 槽在前、CAN 槽在后，按分派层口径）；`baud` 对 CAN 忽略并在报告里说明 | `src/core/klippy/cmd/uart.rs`、`mcu/bridge_ports.rs` | 端口声明序列测试断言 `slot = uart_count`；`baud` 不影响行为 |

### P4 工具：`klipperx admin` 对 CAN 端口

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U13** | `admin bridge config` 报告 CAN 槽：槽名与「这是 CAN 口」的事实（`BRIDGE_UART_COUNT` 仍是总数，另加一个描述 CAN 槽的常量或按 arch 的 `RESERVE_PINS_*` 类比给出） | `src/flash/{bridge_flash.rs,cli.rs}` | 纯函数测试：报告里 CAN 槽行不与 UART 槽混；旧固件（无该常量）文案不变 |
| **U15** | `admin flash --bridge … --port <can槽> --port-nodeid N`：隧道恒切片，端到端走现有 `BridgeLink::flash` | `src/flash/{cli.rs,bridge_flash.rs}` | 单元级：帧序列 = 裸 katapult 块按 8 B 切片、arb = 该 node；失败重试路径与串口一致 |
| **U16** | `admin scan --bridge … --ports <can槽>`：CAN 口上的单元广播与应答归因 | `src/scan.rs` | 纯逻辑测试：应答按 port 归属（路径字节），无应答时报告「该口无设备」 |

### P5 测试与文档（真机前的闸门）

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U17** | 单测补齐：U2/U5 的纯逻辑用桩（`canbus_send` 记录器、假时钟），不依赖真总线；`BRIDGE_UART_COUNT` 分派与槽序 | 各模块 `#[cfg(test)]` | 新增测试全绿；`vcan` 仍不进闸门（与现状一致，只在真机清单里作为可选） |
| **U19** | 闸门：`cargo test --workspace` 全绿、`cargo fmt --all -- --check`、`cargo build --workspace --all-targets` 0 告警、6 份新老 CAN config 全部编过 | — | 命令与输出贴进收口报告 |

### P6 真机前检查单（**本规划到此为止，真机另开一轮**）

见 §4/§5。软件侧做到「构建产物在、命令能列计划、帧序列有测试钉住」即停，不刷板。

---

## 4. 真机设备清单（按拓扑 B）

| # | 设备 | 用途 | 备注 |
|---|---|---|---|
| 1 | **桥：STM32F446**（已就位）+ **CAN 收发器**（3.3V） | 桥：USB CDC 控制台（PA11/PA12），CAN1 在 **PB8(RX)/PB9(TX)** | F4 上 USB 与 CAN1 不共向量，能同时用；应用地址跟 katapult 的 32 KiB 分区（0x8008000） |
| 2 | **下游：STM32F103**（已就位）+ 同款收发器 | CAN1 在 **PA11/PA12**（因此不能再用 USB，与 D2 一致） | 要能刷应用（`CANSERIAL`±`MUX_CAN`）与引导（stock katapult on CAN，裸切片） |
| 3 | **120 Ω ×2** | 总线两端终端 | 新桥板与新下游板各一端 |
| 4 | 双绞线 + 共地 | 物理层 | 短距离即可，但两端终端不能省 |
| 5 | ~~**USB-CAN 适配器**~~（**本期无，不做前置**） | 直连对照（`admin flash --interface can0`）、`candump` 观察 | 没有它照样能验：新桥板自己就是 CAN 的另一端；代价是「主机直连刷」与总线嗅探两条旁证缺席，真机报告里要写明 |
| 6 | **机上现有两块板（桥 + UART 下游）保持现状、不动** | **UART 拓扑的回归基线**：CAN 工作的每一轮闸门后，都用它们把现有 UART 刷写（含可反复刷）重跑一遍 | 不刷、不改配置；它们是「新固件没把旧拓扑搞坏」的唯一真机证据 |
| 7 | 软件侧替代：`CAP_NET_ADMIN` 容器 + `vcan0` | 无板时先验主机逻辑（非本仓闸门先例） | 真机不是它的替代 |

**同时接两套设备时的纪律**：CAN 测试板与现有两块板会同时挂在主机上（`by-id` 不同、芯片 UID 不同），每条命令都要**显式给 `--bridge <CAN 桥板的 by-id>`**；
新桥板首次使用时是未指派状态，要么先 `admin scan` 取 uuid / 或用 `--bridge-uuid`，**不要拿旧桥的 `e512c60ac0e6` 去开**（会白白重启旧桥）。

**非本清单**：CAN-FD 收发器与 FDCAN 板（固件半边未做）、RS-485。

## 5. 真机揭幕时的命令序列（预排，便于验证产出手感）

```sh
B=/dev/serial/by-id/usb-Klipper_stm32f103xe_<桥>
# ① 桥的口表：应看到 UART 槽 + 一个 CAN 槽
klipperx admin bridge config --bridge $B
klipperx admin bridge port --bridge $B --port <can槽> --port-nodeid 7
# ③ 经 CAN 口发现下游
klipperx admin scan --bridge $B --ports <can槽>
# ④ 经 CAN 口刷下游（隧道恒切片；引导态 = stock katapult）
klipperx admin flash --bridge $B --bridge-uuid <uuid> --port <can槽> --port-nodeid 7 <fw.bin>
```

判定口径沿用现有一套（`complete: N bytes in M blocks` 只在 verify 全过后打印；`Found uuid=…, port N`）。

## 6. 风险（真机前必须先想清）

1. **中断关闭时长**：`mux_input` 在 `irq_save()` 段内就地 memmove，CAN 帧更密（`TESTING.md:200-202`
   已登记）——CAN 端口后端自己也要在 IRQ 里做 per-node 追加，必须复用「关中断小块 + 任务里重组」的
   形状，不能整块在 IRQ 里 memmove。
2. **过滤器不够**：bxCAN F103 有 14 个 bank，`0x3f0` 用掉一个、自身 id 一个，下游多 node 时余量要算清；
   超限必须 `shutdown` 报错（附原因），不能静默听不见。
   `admin bridge config` 要能回读端口状态，误配在报告层暴露。
3. **子模组改动**（D4/D5）：`bridge.h`/`canbus.h` 的接口扩展要在 `upstream-deviations.md` 登记，
   并在 katapult 侧同步评估（katapult 的 `canhw_set_filter` 同形，本期不改 katapult）。
## 8. 实施记录

### 2026-10-17：U1 / U4 落地（固件骨架，已合入子模组 `klipperx`）

| 单元 | 提交（`third_party/klipper`） | 结果 |
|---|---|---|
| U1 端口后端分派层 | `ee7cf27c` + `53676321`，合入 `95007ed4` | 新 `src/stm32/bridge_port.c` 独占 `uart_backend_*` 三个符号与 `DECL_CONSTANT("BRIDGE_UART_COUNT")`；`bridge_serial.c` 改名并暴露 `bridge_serial_slot_count()`；`BRIDGE_SERIAL_SLOT_COUNT` 写成编译期和（`DECL_CONSTANT` 只折常量表达式） |
| U4 过滤器可追加 | `1630f1f5`，合入 `8c746e5e` | `CANBUS_FILTER_MULTI`（无 prompt）+ `canhw_add_filter(arb, mask)`：stm32 bxCAN 用 bank 3..10，上限 8，`canhw_set_filter` 在 `FA1R=0` 之后**重装追加项**（否则节点号一变就静默丢） |


### 结构发现（改了两条口径，后续单元必须照这个写）

1. **D6（新增裁定）**：`generic/canbus.c` 与 arch 驱动今天挂在 `CONFIG_CANSERIAL` 下，而 `stm32/can.c` 的 IRQ 直接调 `canbus_process_data()`/`canbus_notify_tx()`，后者转给 canserial——**「只要驱动不要节点」编不出来**。端口后端要求把 `$(canbus-src-y)` 与 `generic/canbus.c` 移到 `CONFIG_CANBUS` 下，`canbus.c` 里的 canserial 调用加守卫，并加桥端口分派点。**仍不 select `CANSERIAL`/`MUX_CAN`**（D1 不变）。
2. **D4 补强**：`struct bridge_port_ops` 加**可选** `.node_id(ctx, uint8_t)` 回调，由 `bridge.c` 在记下某个 id 时调用——过滤器只能在那一刻才知道要听谁；CAN 后端在回调里装过滤器，`canhw_add_filter` 返回 `-1` 或能力未开时 **`shutdown` 并点名**，不许静默听不见。
3. **`shutdown` 归属**：`canhw_add_filter` 只返回 `-1`，由 `bridge_can.c` 翻成一条总线级 `shutdown` 文案，不引入全局聚合层。
4. **过滤器上限已定**：`CANHW_EXTRA_FILTERS = 8` ⇒ 配合 `BRIDGE_CAN_NODE_MAX = 4` 有一倍余量；一个 node 用「bit0 掩码」一格（`id & ~1` + `0x7FE`）盯 `id`/`id+1` 一对。

### 进行中

**U2** ✅ 已合入（`18e35da4` / `7d578035` / `62ec8298`，合入 `29aaa353`）：`CONFIG_BRIDGE_PORT_CAN` + `src/generic/bridge_can.c`——CAN 管线从 CANserial 节点解绑（`canbus.c` 加守卫、`canbus-src-y` 拆出 `canbus-driver-src-y`）、`bridge_port_ops` 的 `.node_id` 钩子装过滤器、`write` 解单元头得 arb（legacy/隧道口用端口 `node_id`）、收侧按 node 分桶后交路由器；构建闸门 `test/configs/stm32f103-serial-can-bridge.config`。

**审出的 P1（已修）**：`bridge_can_task` 原本在 `irq_save()` 里调 `bridge_port_input()`——那是路由入口（会组帧、路由、写别的端口），不能隔着关中断跑；改成「锁内只拷出 + 腾空间，锁外再交路由器」（`62ec8298`）。


**验证（4 条构建全 rc=0）**：新目标 `BRIDGE_UART_COUNT=2` 且 `bridge_can.o` 有七个 `T bridge_can_*`；`usb-mux-bridge`（count=1）、`can-mux`、真机 `target/hw-bridge/klipper.config`（count=2）均**无** `bridge_can_*` 符号；主仓 `cargo test --workspace` 3206 通过 / 15 ignored。

### 2026-10-17 晚：真机拓扑定稿（F446 桥 + F103 下游）与 CAN-only 构建修复

用户提供真实拓扑：**主机 ─USB→ STM32F446（桥）─CAN→ STM32F103（下游）**；桥的 CAN 在 **PB8/PB9**，下游在 **PA11/PA12**（与 D2 的向量约束一致：下游不用 USB）。两块板都已处于引导态（F446 呈现为 `usb-katapult_stm32f446xx_26004B001050563641363020-if00`）。

**新审出的 P1（已修，`07fa67f4`）**：分派层无条件调 USART 后端，所以 **CAN-only build（F446：USB 控制台 + CAN 口，无 USART 口）链接不过**（LTO 报 `unsupported relocation` 于 `uart_backend_set_port`）。修法：串口分支全部用 `#if CONFIG_BRIDGE_PORT_SERIAL` 包；没有串口后端时 CAN 槽从 0 数、ops/set_port 直接答 CAN 后端。新增构建闸门 `test/configs/stm32f446-usb-can-bridge.config`——它就是发现这个 bug 的目标。

**F4 的闪存分区事实**：F4 的 Kconfig **不提供 8 KiB 引导选项**（`STM32_FLASH_START_2000` 只在 F1/F0/G0/G4）；F4 默认 32 KiB ⇒ 当 katapult 是 32 KiB 时应用在 **0x8008000**。真机配置已按此建成 `target/hw-can-bridge/klipper.config`（`target/` 被 ignore，不入库），产物 `target/hw-can-bridge/out/klipper.bin` = **50584 B**，`BRIDGE_UART_COUNT = 1`（只有 CAN 槽）。

**验证**：F446 闸门 + 三条回归（`usb-mux-bridge` count=1 / `serial-can-bridge` count=2 / `can-mux`）+ 真机旧桥配置（count=2）全 rc=0；CAN-only build 的 `bridge_can.o` 七个 `T bridge_can_*`。

**待确认（真机刷写前）**：① F446 上的 katapult 是否按 **32 KiB bootloader**（→ 应用 0x8008000）编的；② CAN 速率——classical CAN 上限 1 Mbit/s，固件现设 `CANBUS_FREQUENCY=1000000`，用户说的「10MHz」需澄清。

### 2026-10-17 收口：CAN 下游端口真机闭环（`work` 已合入）

| 单元 | 提交（klipper / 主仓） | 结果 |
|---|---|---|
| U2 CAN 切片后端 | `18e35da4`/`7d578035`/`62ec8298` | `CONFIG_BRIDGE_PORT_CAN` + `generic/bridge_can.c`：管线脱离 CANserial、`.node_id` 钩子装过滤器、`write` 解 arb、收侧按 node 分桶 |
| U5b 单帧 admin 代理 | `70dd3a68`/`79ad754e` | 开端口即装 `0x3f1`；`0x00`/`0x01`(+`0x11`) 翻译成 `0x3f0` 单帧；`0x3f1 [0x20][uuid][app]` 包成单元上行（端口路径由桥追加） |
| 设备命令按单元下发 | `7df2df49`/`…` | `0x02`/`0x05` 必须带单元头发到设备自己的 arb；剥头+沿用 `0x3f0` 会让 mux 设备看不懂 |
| `0x02` 带 uuid | `d627e384` | `[0x02][index][uuid]` → `0x3f0 [0x02][uuid]` **且**照旧发单元 |
| R1 端口层改名 + `config_can` + 运行期速率 | `bb31ac7d`/`88ce8bfd` | `port` 层、`config_can oid bitrate slot`、`canhw_set_rate`、`CANBUS=n` 的端口型 build |
| R2/U7/U8/U9 主机 | `3bd0463`…`5100c8a` | `ConfigCan`/`PortConfig`、`--can-bitrate`、`--assign-uuid`、刷写前后各指派一次、attach 重试（4 s/25 s） |

**真机结果**（F446 桥 ─CAN 1M→ F103，见 `TESTING.md`）：发现 → 指派 → 送引导 → 隧道刷写 → 再发现 →
反复刷全部通过；下游应用换一副字节流后**一条命令**照样能刷。

1. **`[can_device]` 配置面：已做**（U10，`2777aee`/`b1e4159`）：`can_bitrate`（默认 1 Mbit/s）、
   配置期排 `config_can`+`config_bridge_port`、`resolve` 放开且**要求 `uuid:`**、CAN 口恒 UNITS。
   连带修掉一个静默 P1（U11a）：原来 slot 同时充当「端口 index」与「USART 选择」，一台桥混排
   UART/CAN 且 CAN 先声明时会互换后端；现在 `config_can` 走 `port_backend_can_open` 不看 slot，
   主机给 `config_uart` 的 slot 改成**该桥上的 UART 序号**（U11b）。
2. **CAN 上游 attach（桥经 CAN 被宿主连）与 `fdcan` 的过滤器实现：挂起（用户决定）**，不属本期。

**已知缺口 → U5b**：CAN 单帧 admin（`0x3f0`/`0x3f1`）未代理，而且端口开时也没有人装 admin 过滤器，所以**经桥扫描/指派一台未指派的 CAN 下游设备现在还不可能**。这是真机测试的前置单元（U5b，§3 已排），不是本轮遗留缺陷。

## 实施记录：四台设备压测轮（2026-10-17 晚）

真机拓扑与数字见 [`TESTING.md`](../../TESTING.md) 的「真机结果（2026-10-17，四台设备压测轮）」。
这里只留三条**根因**，因为它们改变了设计理解：

1. **CAN 侧 node id 有两种表示，不能混用**（`7ca47634`）。`can_decode_nodeid()` 把 admin 帧里的
   wire 字节转成**仲裁 id**（`0x100 + 2n`），`CanData.assigned_id` 与接收过滤器都用这个值；而
   mux 设备半的 `mux_assigned` 要的是**节点号**（1..255，`mux_arb_id()`/`mux_reply_arb_id()` 的上界）。
   把 arb 交给它 ⇒ `mux_reply_arb_id()` 返回 0 ⇒ 设备给自己每一帧写 `mux_encode_header(0)`（字节
   `80 00`，一个谁都没有 slot 的地址），并且拒收发给自己的单元 ⇒ 运行期会话永远 `identify` 超时，
   而 admin 通道（scan/指派/送引导/隧道刷写）全通——因为那条路走的是 CAN 自己的帧，不经过 mux 头。
   定位手段：桥固件的 `debug_read` 读 RAM（端口 flags/route/过滤器/计数器 + 一个 arb 直方图），
   **没有动 SWD**。

2. **单元头不上数据路径**（回退 `031eeb15`，`5275808a`）。一度以为「设备侧 `mux_input` 没有 arb 参数、
   所以必须连头下发」，实测反证：改回去行为完全一样。真机教训写在协议页：
   「一个地址只上线一次」——CAN 上地址由 arb 承载，桥解头取 arb 后必须剥头；设备上行的头是**主机线格式**，
   桥原样上交。

3. **console 的 ack 越界会改掉 mux 身份**（`df4ed17c`）。`encode_acknak` 没声明 `min_size`，发送缓冲
   接近满时 ack 被放到缓冲区外 5 字节，帧尾 `0x7e` 正好砸在 `mux_identity[0]`。250 kbaud 饱和链路
   （930 req/s）几乎每个 ack 都命中 ⇒ 设备身份 `97abe031988e` → `7eabe031988e`，重连握手永远超时，
   只有重新上电恢复。这条是**压测逼出来的**，单次功能测试看不出来。

**压测同时确认的两个结构事实**：并发损耗不对称（UART 损被中继的一侧 ~20%，CAN 损桥自己的控制台 ~36%，
因为 CAN 中继在桥固件里更贵）；CAN 的 `queue_step` 在经典帧下每条拆两帧，压测的**突发下发**会先撞
`Timer too close`（真实主机是随移动队列连续投递，所以现场能力好于这个数字，但高步频板仍建议走 UART 500 k）。
