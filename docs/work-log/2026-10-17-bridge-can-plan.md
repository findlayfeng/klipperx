# 桥接 CAN 下游端口（拓扑 B）：动工前规划

**状态**：规划定稿（未动代码、未上真机）。**拓扑已由用户拍板：B —— CAN 当桥的下游端口**
（桥仍是 USB CDC 控制台 + 一个 CAN 收发器当它的一个端口，像今天的 USART 端口那样），
端口上**两条载具都要**：CAN 流（8 字节切片）与 ISO-TP（整帧一条消息）。

本文件是任务书来源：每个单元（U1…U19）就是一份最小 worker 任务书；实现细节改动先改本文件。

---

## 1. 事实基线（全部有源码依据，动手前不必再查）

### 1.1 两条载具都在，且都在「节点自己的字节流」里

| 载具 | 开关 | 固件 | 主机 | 语义 |
|---|---|---|---|---|
| CAN 流（8 字节切片） | `CANSERIAL`（arch 的引脚项 `select`） | `third_party/klipper/src/generic/canserial.c` | `src/core/klippy/interface/devices/canserial.rs`（31 测） | 字节流按 ≤8 B 切帧，无 PCI/序号/长度头 |
| ISO-TP（整帧一条消息） | `CONFIG_ISOTP`（`depends on CANSERIAL`，默认 n） | `src/generic/isotp.c`（classical only：`ISOTP_DL 8`、SF 7/FF 6/CF 7、25 ms 超时、FC 固定 CTS/BS=0/STmin=0） | `interface/devices/isotp.rs`（8 测）+ `isotp_proto.rs`（22 测） | 一帧 klipper 消息（+ MUX 口时的 2 字节单元头）当一条 ISO-TP 消息分段 |

`ISOTP` **换的是载具，不是寻址**：arb id、`0x3f0` 单帧 admin、uuid、mux 全不变
（`third_party/klipper/src/Kconfig:441-466`、`src/generic/isotp.c:1-15`）。

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
- **ISO-TP + 应用**：klipper `CANSERIAL`+`ISOTP`（`stm32f103-can-isotp.config` / `stm32h743-can-isotp.config`）。
- **ISO-TP + 引导：不存在**。katapult 无 ISO-TP（`third_party/katapult/src/generic/` 无 `isotp.c`），
  设备从「应用 ISO-TP」重启进「引导切片」时，**同一条 arb 上的字节格式会变**。
  ⇒ 隧道刷写恒按切片（见 D3），ISO-TP 只管运行期。

---

## 2. 五个设计裁定（本规划的前提，若有异议先改这里再动代码）

| # | 裁定 | 理由 | 代价 |
|---|---|---|---|
| **D1** | CAN 端口后端 = 新文件 `src/generic/bridge_can.c`，只依赖 `canbus.h` 的板级钩子；**不 select `CANSERIAL`、不 select `MUX_CAN`**，只 `select CANBUS` 拉驱动 | 避开 `mux_send` 双实现（§1.2-2）；端口不绑本节点 id，才能听多个下游 id | 端口后端自己管过滤器与 rx 分桶 |
| **D2** | CAN 引脚从控制台 `choice` 里解耦：新增 `STM32_BRIDGE_CAN_PB8_PB9`（低层选项）等端口专用引脚项，`select CANBUS` + `select BRIDGE_PORT_CAN`，与 USB 控制台共存 | 拓扑 B 的全部意义就是「USB 控制台 + CAN 端口」；用 PB8/PB9 避开 USB 的 PA11/PA12 | 对上游 Kconfig 的偏离，必须进 `upstream-deviations.md` |
| **D3** | 端口载具用 **`SET_PORT_NODEID` 的 `flags` 新增 bit1 `MUX_PORT_ISOTP`（0x02）** 声明，默认切片；**隧道（`0x06`）恒按切片**，与载具标志正交 | flags 已经是「设备说什么」的载体属性；隧道运的是 katapult 裸帧，而 katapult 没有 ISO-TP（§1.5） | 旧桥忽略未知 bit ⇒ 静默按切片处理，需在 `admin bridge config` 报告里回读端口状态以暴露误配 |
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
| **U5** | **收侧分桶 + arb 来源**：`canbus_process_data` 在 `CONFIG_BRIDGE_PORT_CAN` 时把「下游 arb」的帧交给 bridge_can 的 per-node 重组缓冲（**不进** `CanData.receive_buf`），凑齐一个整单元/整块后 `bridge_port_input(index, …)`；`bridge.h` 加 `bridge_port_node_id(int)` 供 legacy/tunnel 口取 arb | `src/generic/canserial.c`（分派点）、`src/generic/bridge_can.c`、`src/bridge.{h,c}` | 纯函数测试：两个 node 的帧交错到达 → 两条独立流各自完整；无 node_id 的 legacy 口收到数据不 panic（丢弃 + 计数） |
| **U6** | **构建闸门配置**：`test/configs/stm32f103-usb-can-bridge.config`（USB CDC + `MUX_SERIAL` + 两个 USART 端口 + CAN 端口） | `third_party/klipper/test/configs/` | 闸门 `cargo test --workspace` 里被编过（现有机制按目录遍历，`crates/test-support/build.rs:151-197`），字典能产出 |

### P2 固件：ISO-TP 端口载具

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U7** | **per-(port,node) ISO-TP 状态**：从 `src/generic/isotp.c` 抽出可多实例化的收发状态（收 SF/FF/CF/FC；发分段 + 等 FC），常量沿用 8 B classical / 25 ms / CTS·BS=0·STmin=0；上限固定（桥 4 端口 × 1 node 起步） | `src/generic/bridge_can_isotp.c`（或 `bridge_can.c` 内分支） | 纯状态机测试：SF/FF+CF/FC 双向、SN 回绕、超时失败；与 `isotp_proto.rs` 的 22 条测试语义对齐（可照抄用例） |
| **U8** | **载具标志落到写/收路径**（D3）：`MUX_PORT_ISOTP` 时 `write` 走 ISO-TP 分段（目标 arb = 端口 node_id），收走 ISO-TP 重组；隧道恒切片 | `src/generic/bridge_can.c`、`src/bridge.c`（flags 记录）、`src/mux.h`（bit 定义） | 同一端口在两种 flags 下的帧序列测试（切片 vs SF/FF/CF）；隧道模式忽略 ISOTP 位 |
| **U9** | **构建闸门配置**：`test/configs/stm32f103-usb-can-bridge-isotp.config`（U6 + `CONFIG_ISOTP` 在端口侧意义下的编译覆盖） | 同上 | 编过、字典产出 |

### P3 主机：配置面与 attach

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U10** | **放开 `[can_device]` 端口**：`load_can_device` → `wire_bridge_port`、`CanPort::index()`；`bridge_attach::resolve` 去掉 CAN 拒绝（含错误文案与 `:732-744` 的探针测试改成「可接」） | `src/core/klippy/mcu/{bridge_ports.rs,bridge_attach.rs}` | 配置层测试：`[can_device]` + `[mcu child]` 能构建端口请求；旧的「点名拒绝」断言全部改掉（`grep BRIDGE_PORT_CAN` 归零，文档另行同步） |
| **U11** | **CAN 端口的 `config_uart` 语义**：slot 规则（UART 槽在前、CAN 槽在后，按分派层口径）；`baud` 对 CAN 忽略并在报告里说明 | `src/core/klippy/cmd/uart.rs`、`mcu/bridge_ports.rs` | 端口声明序列测试断言 `slot = uart_count`；`baud` 不影响行为 |
| **U12** | **载具告知**：`[can_device]` 新键 `can_protocol: cans|isotp`（默认 `cans`），落成 `SET_PORT_NODEID` 的 `MUX_PORT_ISOTP` 位 | `src/core/klippy/mcu/bridge_ports.rs`、`interface/devices/serial.rs`（标志构造） | 配置解析 + 线上字节测试（flags = `UNITS|ISOTP`） |

### P4 工具：`klipperx admin` 对 CAN 端口

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U13** | `admin bridge config` 报告 CAN 槽：槽名与「这是 CAN 口」的事实（`BRIDGE_UART_COUNT` 仍是总数，另加一个描述 CAN 槽的常量或按 arch 的 `RESERVE_PINS_*` 类比给出） | `src/flash/{bridge_flash.rs,cli.rs}` | 纯函数测试：报告里 CAN 槽行不与 UART 槽混；旧固件（无该常量）文案不变 |
| **U14** | `admin bridge port` 对 CAN 端口：默认 UNITS + 可选 `--port-carrier cans|isotp`；拒绝组合（CAN + `--legacy` 且无 node_id；CAN + 非默认 baud 提示忽略） | `src/flash/cli.rs` | plan/参数层测试：合法组合成计划、非法组合点名报错；命令序列断言与串口口一致（`SET_PORT_NODEID` flags 多一位） |
| **U15** | `admin flash --bridge … --port <can槽> --port-nodeid N`：隧道恒切片，端到端走现有 `BridgeLink::flash` | `src/flash/{cli.rs,bridge_flash.rs}` | 单元级：帧序列 = 裸 katapult 块按 8 B 切片、arb = 该 node；失败重试路径与串口一致 |
| **U16** | `admin scan --bridge … --ports <can槽>`：CAN 口上的单元广播与应答归因 | `src/scan.rs` | 纯逻辑测试：应答按 port 归属（路径字节），无应答时报告「该口无设备」 |

### P5 测试与文档（真机前的闸门）

| # | 单元 | 落点 | 验收 |
|---|---|---|---|
| **U17** | 单测补齐：U2/U5/U7/U8 的纯逻辑用桩（`canbus_send` 记录器、假时钟），不依赖真总线；`BRIDGE_UART_COUNT` 分派与槽序 | 各模块 `#[cfg(test)]` | 新增测试全绿；`vcan` 仍不进闸门（与现状一致，只在真机清单里作为可选） |
| **U18** | 文档同步：协议页（`MUX_PORT_ISOTP`、CAN 端口后端与槽序、隧道恒切片）、用户手册（`[can_device]` 可用、四命令对 CAN 口、`can_protocol`）、`config.md`、`upstream-deviations.md`（D2 的 choice 拆解、D5 的过滤器接口）、`testing.md` 测数与覆盖、`TESTING.md` M1-4 改为「待真机，清单见本文件」 | 见左 | 「`BRIDGE_PORT_CAN` 尚缺」类断言全部消失（`grep` 只剩历史 work-log）；测数用 `cargo test --list` 重数 |
| **U19** | 闸门：`cargo test --workspace` 全绿、`cargo fmt --all -- --check`、`cargo build --workspace --all-targets` 0 告警、6 份新老 CAN config 全部编过 | — | 命令与输出贴进收口报告 |

### P6 真机前检查单（**本规划到此为止，真机另开一轮**）

见 §4/§5。软件侧做到「构建产物在、命令能列计划、帧序列有测试钉住」即停，不刷板。

---

## 4. 真机设备清单（按拓扑 B）

| # | 设备 | 用途 | 备注 |
|---|---|---|---|
| 1 | **桥板**：STM32F103ZE（现有那块 devloop1）+ **CAN 收发器模块**（3.3 V：SN65HVD230/TCAN332 一类） | 桥：USB CDC 控制台，CAN 当它的下游口 | CAN 接 **PB8(RX)/PB9(TX)**，避开 USB 的 PA11/PA12（D2） |
| 2 | **下游 CAN 板 ×1**：STM32F103ZE + 同款收发器 | 两条路径的被测设备 | 需要能刷两套固件：klipper `CANSERIAL`(±`MUX_CAN`) 与 klipper `CANSERIAL`+`ISOTP`；引导态用 stock katapult（CAN 载体，裸切片） |
| 3 | **120 Ω ×2** | 总线两端终端 | 桥板与下游板各一端 |
| 4 | 双绞线 + 共地 | 物理层 | 短距离即可，但两端终端不能省 |
| 5 | **USB-CAN 适配器（可选）** | 直连对照（`admin flash --interface can0`）、`candump` 观察 | 桥板的 CAN 已被端口占用，不能当主机侧适配器用 |
| 6 | （可选）第二块下游板 | 「UNITS 口 / 隧道口」两种用法各一块，省得反复刷 | 一端口一 node 是第一版上限 |
| 7 | 软件侧替代：`CAP_NET_ADMIN` 容器 + `vcan0` | 无板时先验主机逻辑（非本仓闸门先例） | 真机不是它的替代 |

**非本清单**：CAN-FD 收发器与 FDCAN 板（固件半边未做）、RS-485。

## 5. 真机揭幕时的命令序列（预排，便于验证产出手感）

```sh
B=/dev/serial/by-id/usb-Klipper_stm32f103xe_<桥>
# ① 桥的口表：应看到 UART 槽 + 一个 CAN 槽
klipperx admin bridge config --bridge $B
# ② 把 CAN 槽启起来：UNITS 载具（cans / isotp 两选一）
klipperx admin bridge port --bridge $B --port <can槽> --port-nodeid 7 --port-carrier cans
# ③ 经 CAN 口发现下游
klipperx admin scan --bridge $B --ports <can槽>
# ④ 经 CAN 口刷下游（隧道恒切片；引导态 = stock katapult）
klipperx admin flash --bridge $B --bridge-uuid <uuid> --port <can槽> --port-nodeid 7 <fw.bin>
# ⑤ ISO-TP 路径：设备跑 CANSERIAL+ISOTP 时，端口用 --port-carrier isotp 再 scan/连
```

判定口径沿用现有一套（`complete: N bytes in M blocks` 只在 verify 全过后打印；`Found uuid=…, port N`），
另加两条本拓扑专属：**CAN 槽的帧序列里 arb = `0x100+2*node`**、**ISO-TP 口上是 SF/FF/CF 而非裸切片**。

## 6. 风险（真机前必须先想清）

1. **中断关闭时长**：`mux_input` 在 `irq_save()` 段内就地 memmove，CAN 帧更密（`TESTING.md:200-202`
   已登记）——CAN 端口后端自己也要在 IRQ 里做 per-node 追加，必须复用「关中断小块 + 任务里重组」的
   形状，不能整块在 IRQ 里 memmove。
2. **过滤器不够**：bxCAN F103 有 14 个 bank，`0x3f0` 用掉一个、自身 id 一个，下游多 node 时余量要算清；
   超限必须 `shutdown` 报错（附原因），不能静默听不见。
3. **载具误配**（D3 的代价）：设备跑 ISO-TP、端口按切片 ⇒ 字节被解释成垃圾且**看起来像 CRC 错**。
   `admin bridge config` 要能回读端口载具，误配在报告层暴露。
4. **引导期格式翻转**（§1.5）：ISO-TP 设备的引导是切片。刷写只走隧道（恒切片）就绕开了；
   若将来要在端口上「拨回应用」再验 ISO-TP，需要一次复位 + 端口载具保持 —— 第一版不验这段。
5. **子模组改动**（D4/D5）：`bridge.h`/`canbus.h` 的接口扩展要在 `upstream-deviations.md` 登记，
   并在 katapult 侧同步评估（katapult 的 `canhw_set_filter` 同形，本期不改 katapult）。
