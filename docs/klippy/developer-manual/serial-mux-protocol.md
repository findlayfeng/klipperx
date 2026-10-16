# 串口复用协议（多 MCU 共用一个串口）

一个物理串口后面**只有一条字节流**，而 Klipper 的帧格式里没有板地址（`frame.rs`）——所以
「多颗 MCU 共用一个串口」必须自己给帧加**地址头**，并且要和设备端谈妥这件事。这份文档就是
那条线上的规格：**设备端（固件）按它实现，主机侧按它发送与分派**。

主机侧代码：`src/core/klippy/interface/devices/serial.rs`（端口、reader、分派、握手，
以及帧头与 KXID 伪帧的编解码）；仲裁 id 的映射单一定义在 `src/core/klippy/assign.rs`
（`arb_id`/`reply_arb_id`/`mcu_id_from_arb`/`is_reply_arb`，私有 `ARB_BASE`），CAN 与共享串口都从那里取。
什么时候用它、什么时候不用，由**配置事实**决定：同一个 `serial:` 路径被
**≥2 颗** `[mcu]` 引用即共享（`mcu/serial_ports.rs` 在装载期分组）；被 1 颗引用且**不是桥**
（没有下游设备挂在它上面）才是真正的独占——独占串口走原来的 `SerialDevice`，**不发任何指派帧、
不带任何头**，线格式与 Klipper 逐字节相同。桥则不论引用几颗都**按 id 寻址**（规则 7，见下节
[两级寻址](#两级寻址上游链路与下游端口正交)）。

> **固件侧：设备半与桥半都已实现，但还没有架构默认启用它们**。`src/mux.c` 有**单元格式与
> admin 单元族**，以及**设备半**：`mux_input()` 就地解析输入流（首字节高位置位 = 带头单元，
> 否则 = 原始帧），未分配时答扫描、按 `0x01` 采纳 node id、被指派到已持有的 id 时照 CAN 的
> `can_id_conflict` 弃号并 `shutdown`；已采纳 id 后自己发出的帧带应答头。`src/bridge.c`
> （`CONFIG_BRIDGE`）有**桥半**：端口表、`uuid → 端口` 与 `id → 端口` 两张路由表、admin 扇出
> （上游广播原样下达、下游应答**逐条**上行、不合并）、单设备路径**剥头/加头**；**M1-P2 起**
> 又加上**端口级命令族**（`SET_PORT_NODEID`/`REQUEST_BOOTLOADER`/`RESTART_DEVICE`/`RESET_PORT`，
> **T3 起加 `SET_PORT_TUNNEL`**，面向桥自己、无回执，
> 见下节「端口级命令族」）；**M1-P7 起端口改为配置期由主机声明的 UART 对象**
> （`config_uart`/`config_bridge_port`，字典表退役），「单设备」由主机下发的 `flags` 运行期告知
> 而非编译期标志。host 库还有
> **下游串口端口**与 **unit 收发接口**（`klipper_host_input_unit`/`klipper_host_output_unit`，
> 需帧接口开关**且**桥模式）。
>
> 但**今天没有架构把它编进共享串口的固件**：`MUX_SERIAL` 只由 host 的 `HOST_BRIDGE_SERIAL`
> 或 `HOST_MUX_SERIAL` 选上（微控制器将来照它们给控制台选 UART 的做法加 `choice` 备选串口是
> 同一件事）。所以**今天的串口固件仍不回执**，共享模式必然**握手超时**——误开到旧固件只会连接
> 失败，不会破坏会话。设备半与桥半都有**永久测试**钉住（见[测试](testing.md)：第二份 mux 库
> 变体 + mux 路径的真端到端 attach）。

## 寻址：与 CAN 逐位同值

沿用 CAN 的映射——单一定义在 `assign.rs`（`arb_id`/`reply_arb_id`，私有 `ARB_BASE = 0x100`）：

| 方向 | 仲裁 id | 说明 |
|------|---------|------|
| 主机 → 设备 | `0x100 + 2*mcu_id` | 主机写这个 id |
| 设备 → 主机 | `0x100 + 2*mcu_id + 1` | 设备在下一个 id 上回答 |

`mcu_id` 是该 MCU 在 `[mcu_ids]` 里分到的 **MCU 层 id**（1..=255，`mcu/ids.rs`）——和 CAN
节点号是同一个号段、同一个序列。主机侧把仲裁 id 解回 `mcu_id` 时**两个方向都解回同一个
id**（`mcu_id_from_arb`），方向本身由奇偶区分（`is_reply_arb`）。

### 两级寻址（上游链路与下游端口正交）

一条链路的寻址分两级，两级**互不相干**：第一级是**上游链路**（主机 ↔ 桥/设备），回答「这条
路径是否按 id 加帧头收发」；第二级是**下游端口**（桥 ↔ 挂在 `[uart_device]`/`[can_device]`
上的设备），由主机用 `SET_PORT_NODEID` 在**桥自己**的 `id → 端口` 表里登记。判定第一级时
**不需要知道下游有哪些设备**，下游设备也全程不参与上游寻址：

| 上游链路 | 第一级：主机 ↔ 桥（上游链路按 id 寻址？） | 第二级：桥 ↔ 下游设备 |
|----------|------------------------------------------|-----------------------|
| 串口（`serial:`） | 规则 7：**共享**（≥2 颗 `[mcu]` 引用）**∨ 下游设备 ≥ 1 ∨ 是桥** ⇒ 用 2 字节帧头；三者都不是 ⇒ 裸帧 | 桥的 `id → 端口` 表（`SET_PORT_NODEID(index, id, flags)` 登记，`index` = 端口节声明顺序） |
| CAN（`canbus`） | 桥的 CAN **node id**：总线按 `0x100+2*id` 仲裁 id 硬件寻址，每颗 MCU 各装一个 filter，**不需要**额外帧头 | 桥下游那条字节流**内部**的 mux id（同一套单元格式），同样经桥的 `id → 端口` 表路由 |
| host 库（`host_library:`） | 与串口同构（`mcu/host_libraries.rs::path_is_mux`）：单节写 `uuid` **∨ 下游设备 ≥ 1 ∨ 是桥 ∨ ≥2 节** ⇒ 按 id 寻址；单节无 `uuid` 且非桥 ⇒ 裸帧 | 同串口：桥的 `id → 端口` 表 |

**规则 7 的判定式**（`mcu/serial_ports.rs::is_id_addressed`，host 库侧同构
`mcu/host_libraries.rs::path_is_mux`）：

```text
按 id 寻址 = 共享 ∨ 下游设备数 ≥ 1 ∨ 是桥
```

「下游设备数」由 `mcu/bridge_ports.rs::bridge_downstream_counts` 零 access 预扫全配置得出：先把
每个 `[uart_device]`/`[can_device]` 的 `*_mcu` 记成「端口节标识 → 桥名」，再把每个 `[mcu]` 的
`uart:`/`can:` 引用映射回端口节、数到对应桥名下。计数项与「是桥」项**各是一条独立理由**，
不互相取代：**0 台下游**的桥仍按 id 寻址——这是**刻意的保守口径**，不是疏漏：桥只要被 mux 化，
`mux_out_header` 就会给它的输出加帧头；若因「0 台下游」判成非 id 寻址，主机会用裸帧去解一个带
身份的 mux 设备输出，直接解析错位。

## 帧头：2 字节带校验的码字

头**不是** Klipper 的 VLQ：它是**固定 2 字节**，装仲裁 id 的 **10 个数据位 + 4 个校验位**
（缩短汉明码：校验位落在 1/2/4/8，数据位占其余位置，码字宽 14 位），外面保留变长编码那
两个定界位——首字节最高位恒 `1`、次字节恒 `0`——所以两字节边界在字节流上无歧义。单一实现在
`serial.rs`（`encode_header`/`decode_header`、`HEADER_*` 常量）。

| `mcu_id` | 主机 → 设备 | 设备 → 主机 |
|----------|------------|------------|
| 1 | `84 42` | `F4 42` |
| 120 | `C0 7E` | `B0 7E` |
| 255 | `AF 7D` | `DF 7D` |
| admin | `E8 3F`（收听） | `98 3F`（应答） |

- **只检不纠**：校验矩阵的 14 个列（每个位置的非零 4 位编码，去掉一个）互异且非零 ⇒ 最小距离
  3 ⇒ **任何 1 位、2 位错误都被检出**（`HeaderScan::Invalid` → 丢单元 + 按既有策略重同步）。
  不做纠错是刻意的：2 位错若被「当作 1 位错纠正」会落到**另一个合法 id** 上，静默把帧投给
  错误 MCU；检测则把它变成一次丢弃。
- **为什么需要**：帧体的 CRC-16 **不覆盖头**（只算 `[len][seq][载荷]`）。头被翻位而仍落在另一个
  合法 id 上时，帧体完好、CRC 通过，而同一根线上各 MCU 的会话序号是同步增长的，收到方很可能
  把它当成本会话的合法帧——发给 A 的命令在 B 上执行。CAN 侧没这个问题（CAN 自己的 CRC 覆盖
  仲裁 id）。丢弃后有 Klipper 帧自身的块级 ack/重传兜底。
- **值域**：10 位数据位容纳 `0x000..=0x3ff`。协议占用的是分配域 `0x102..=0x2ff`（`id` 双向）与
  admin `0x3f0`/`0x3f1`；**`0x3f2..=0x3ff` 保留**，本版本不发出、读到即丢。`0x3ff`
  （`HEADER_ESCAPE`）是将来**扩展头**的起点（其后可接长度/子类型），所以 2 字节格式没有关掉
  「加长的形式」这条路；10 位数据位本可容到 383，但保留带（`0x3f2..=0x3ff`）与 admin 的 `0x3f0`/`0x3f1` 把**现行上限压到 375**（`0x100+2*375 = 0x3ee`、应答 `0x3ef`；376 的对就撞上 admin）——仍远超 CAN admin 报文单字节 `nodeid`
  的 255 上限）。
- **头里仍可出现 `0x7e`**：`id ∈ 120..=127` 时第二字节就是它（`id=120 → C0 7E`）。这不是缺陷——
  上游 [Protocol.md](../../../third_party/klipper/docs/Protocol.md) 明确写着「a sync character is
  **not** exclusive to the framing and **may** be present in the message block content」，帧内容
  本来就允许含 `0x7e`；定界靠长度字节与那个固定两字节边界，不靠「头上不出现 `0x7e`」。Klipper 的
  丢弃/重同步路径（`command_find_block`）也只在**校验失败**时才去扫下一个 `0x7e`。

## 单元 = 头 + 帧体

```
[2 字节头(仲裁 id)] [原始帧字节……]       ← 主机 → 设备，头用 0x100+2*id
[2 字节头(仲裁 id)] [原始帧字节……]       ← 设备 → 主机，头用 0x100+2*id+1
```

**帧体一个字节都不改**：长度、序号、载荷、CRC 的覆盖范围、尾部 `0x7e` 全是原来的
（`frame.rs`）。主机发送时把 `encode_header(arb) ++ frame.raw_bytes()` 一次写完（端口级写锁
串行化，单元内部不会被另一颗 MCU 的写打断）；接收时按「固定 2 字节头（校验通过）→ 取长度
字节 → 凑齐该长度」切出整单元，剥掉头，把帧字节推进该 `mcu_id` 自己的 `FrameStream`
（重组与校验仍是既有那套，`serial.rs`）。

头校验不过（`HeaderScan::Invalid`：定界位错、1/2 位错、或落在保留带）、长度字节不在
`MESSAGE_MIN..=MESSAGE_MAX`、或头结构上不可能成立时，按 `FrameStream` 的重同步语义丢弃到
下一个 `0x7e`。

## 隧道单元：katapult 帧整段作为帧体（两侧均已落地）

刷写会话（katapult 协议）要复用同一条共享链路时，katapult 帧**整帧**作为一个单元的帧体传——
不拆、不改内层一个字节：

```
[2 字节头(仲裁 id)] [01 88 <cmd> <word_cnt> <payload> <crc16> 99 03]
```

- **判据**：帧位（头后）首两字节恰为 `01 88`（katapult `MESSAGE_STX1/STX2`）。与 klipper 帧
  **零冲突**：klipper 帧位首字节是长度（`5..=64`），`0x01` 与 `0x88` 都不在值域。
- **长度**：`word_cnt` 在帧位第 4 字节，帧长 = `word_cnt*4+8`，约束 **8..=128**（katapult 的
  `MESSAGE_MIN/MAX`，`third_party/katapult/src/command.c` 同款校验路径）；缓冲未到齐按整单元
  等待（等价于帧的 NeedMore），越界按原有重同步语义丢到下一个 `0x7e`；`0x01` 后不是 `0x88`
  走原 junk 路径、逐字节不变。
- **绝不分片**：数据块命令帧（Send Block，64 B 块 = 76 B）超过 klipper 单帧载荷上限（59 B），
  跨帧分片会把外层帧尾与下一帧头撕进内层、内层 CRC 必炸——所以隧道单元按整帧承载，128 B
  上限正好盖住全部命令与 64 B 数据块（分片必碎由永久测试钉住，见 [测试](testing.md)）。
- **unit 层不校验内层**：只按 framing 切整段（tag、word-count 定长一致、8..=128），**不校验内层
  CRC、不校验 cmd 字节**——内层校验归刷写会话。双向：主机→设备用 `arb_id`、设备→主机用
  `reply_arb_id`（寻址不变）。
- **消费（T5c 起有投递口）**：主机收线识别到隧道单元即整段消费，并**按 arb 投给注册过该 arb 的监听者**
  （`TunnelSink::channel()` / `set_tunnel_sink(arb)` / `clear_tunnel_sink(arb)`；三条链路同形：共享串口
  `SharedSerialHandle`、host 库 `HostDevice`、CAN `CanSerialDevice`）。投的是**整帧字节**（消息 = 一个 katapult
  块），消费方不必重新分帧；**未注册时仍记 debug 丢弃**，**绝不**落进 `drop_to_sync` 吃流（其后普通单元
  照常切出，有测试钉住）；接收端 drop 后 `send` 失败 ⇒ 自动注销并回到丢弃。主机侧的 `Read+Write`
  适配器在 `src/flash/bridge.rs`（`TunnelBridge::split()` → `TunnelReader`/`TunnelWriter`；writer
  只发**整个 katapult 块**，reader 非阻塞 `WouldBlock`），刷写会话直接就着它跑。
- **与固件侧是共享契约**：判据与常量集中在 `serial.rs` 的「The tunnel unit」一节，固件 `mux.c`
  **已按同一判据镜像落地**（`mux_unit_len` 的 `01 88` 分支：子模组提交 `a5ecd95a`，真库测试
  `test_the_mux_tunnel_units_over_the_real_library` 钉住行为；两侧措辞与数字一致，无差异点）。
- **桥的隧道口（T3，固件 `src/bridge.c`）**：`SET_PORT_TUNNEL (0x06)` 把一个**端口**置成隧道口
  （幂等 toggle，桥自己消费、不转发下游、无回执，越界/短载荷静默）。置位后该口双向按隧道语义走：
  **下行**收到路由到本的隧道单元则剥 2 B 头、把 katapult 帧原样字节写端口；**上行**扫端口字节流
  （`01 88` 起、`frame[3]*4+8` 定长、垃圾逐字节丢），收齐一个整帧就作为一个隧道单元、以
  `reply_arb_id(port.node_id)` 上行（一次 `mux_send`，不交错）；缓冲上限 = **2×128 B**（整块
  总能装下）。**前提**：上行要用该口的 `node_id`，而它只由 **legacy 方式**的 `SET_PORT_NODEID`
  （`flags` 无 `MUX_PORT_UNITS`）记下——刷写目标本来就是不说单元的引导设备，故口径一致；但把口
  先命名成 UNITS 再开隧道会静默丢掉上行。主机侧 `SET_PORT_TUNNEL` 的常量与发送器属 T5。
- 黄金字节（两侧测试共用）：Connect `01 88 11 00 f1 7c 99 03`；Send Block（64 B 块、地址
  `0x1000`）`01 88 12 11 00 10 00 00 + 00..3f + da 1e 99 03`；16 B ACK
  `01 88 a0 02 12 00 00 00 00 10 00 00 bc dc 99 03`。

## 分配期握手（identify 之前）

每颗 MCU 在**分配（打开）这个共享端口时**做一次同步握手，握手之后才轮到 identify：

```
请求（主机）  [头(0x100+2*id)]   [09][20]["KXID"][crc16 hi][crc16 lo][7E]
回执（设备）  [头(0x100+2*id+1)] [09][20]["KXID"][crc16 hi][crc16 lo][7E]
```

- 那个 9 字节伪帧就是 `serial.rs` 的**伪帧**（`ASSIGN_SEQ_BYTE` / `MAGIC`）：与合法帧只差 seq 字节——`0x20` 把
  `MESSAGE_DEST`（`0x10`）清掉、改占 bit5，于是 `Frame::parse` 必然拒收它（`frame.rs`），
  这让它天然区别于数据帧。**id 只在头里出现一次**，载荷只放 `KXID` 标记（不再编一遍号）。
- **请求对旧固件无害**：旧固件的 `command_find_block` 按「长度 → seq 字节 → …」检查，而单元
  出线的第一个字节就是头的首字节（`0x80..=0xff`，不是合法长度），所以它在**第一个字节**处就进
  错误分支、丢弃到下一个 `0x7e`；`id ∈ 120..=127` 时那个 `0x7e` 就是头的第二字节，伪帧留给下一次
  调用再丢一次。两步都**不读载荷、不查 CRC、永不 dispatch、序号不动**（至多回一枚无害空帧后转
  静默）。所以把共享串口误配到不支持复用的板子上，结果是 **握手超时、连接失败**，而不是把已有
  会话搞乱。这条性质由 `interface/devices/serial.rs` 里 `third_party/klipper/src/command.c`
  的**可执行忠实移植测试**钉住（整单元——含新头——也补了一条）。
- 超时与重试：默认 500 ms × 3 次（`ASSIGN_ATTACH_TIMEOUT` / `ASSIGN_ATTACH_ATTEMPTS`）；
  用尽仍无回执 → `InterfaceError::Timeout`，`connect` 失败。
- 端口自打开起就有**唯一 reader**：某颗 MCU 来 attach 时在该 reader 里登记一个
  「待握手槽位」（按它的 `mcu_id`），带 `+1` 头的回执被同一个分派路径投给它——所以
  **已有 MCU 在正常跑时，新 MCU 也能安全 attach**，不需要抢夺 fd。
- 节上写了 `uuid`（或别名 `canbus_uuid`）时，这枚请求**不是第一枚出线的单元**：
  `attach()` 先按 uuid 发一枚 `SET_KLIPPER_NODEID` 点名，再发这枚重确认请求——帧序见
  下一节。不带 `uuid` 的节只发重确认请求，与之前完全一致。

## 发现与指派：admin 信道与命令族

共享串口沿用 CAN 的 admin 语义，命令帧也沿用上一节那枚 `KXID` 伪帧——命令只是标记之后多出
的一个字节，**分配期握手的 9 字节请求一字不变**（它就是标记后不带命令的那枚）。常量单一
定义在 `serial.rs`（`ADMIN_LISTEN`/`ADMIN_REPLY`/`CMD_*`/`RESP_NEED_NODEID`），
CAN 侧的同族常量在 `canserial.rs`。

| 角色 | 仲裁 id | 说明 |
|------|---------|------|
| admin 收听 | `0x3f0` | 未分配设备在此收查询/指派（与 CAN 广播同一个 id） |
| admin 应答 | `0x3f1` | 未分配设备在此回答（沿用 `+1` 奇偶约定） |

两个 id 在已分配号段 `0x102..=0x2ff` 之外，所以 `dispatch` 不会把 admin 单元投给任何 MCU。

### 命令族（帧体 = `[len][0x20]["KXID"]` + 命令字节）

| 命令/应答 | 字节 | 方向 | 单元 |
|-----------|------|------|------|
| `QUERY_UNASSIGNED` | `0x00` | 主机 → 全部 | `[头(0x3f0)][len][0x20]["KXID"][0x00][crc16][0x7e]`（`discovery_query()`） |
| `SET_KLIPPER_NODEID` | `0x01` | 主机 → 点名 | `[头(0x3f0)][len][0x20]["KXID"][0x01][uuid 大端 6 字节][nodeid][crc16][0x7e]`（`set_node_id_for(uuid, id)`，id 越界返回 `None`，校验同 CAN） |
| `REQUEST_BOOTLOADER` | `0x02` | 主机 → 桥 → 单口 | `[头(0x3f0)][len][0x20]["KXID"][0x02][index][crc16][0x7e]`（M1-P2；对照 CAN `CMD_REQUEST_BOOTLOADER` 同值同义）——**刷写工具的命令**：让那台设备去 bootloader |
| `RESTART_DEVICE` | `0x05` | 主机 → 桥 → 单口 | 同上形状，`[0x05][index]`——**重启那台设备的固件**（`mcu_restart()`，即 `reset` 命令的身体），设备自己起来而不是停在引导里 |
| `SET_PORT_NODEID` | `0x03` | 主机 → 桥自己 | `[头(0x3f0)][len][0x20]["KXID"][0x03][index][id][flags][crc16][0x7e]`（`set_port_node_id(index, id, flags)`，见下节） |
| `RESET_PORT` | `0x04` | 主机 → 桥自己 | `[头(0x3f0)][len][0x20]["KXID"][0x04][index][crc16][0x7e]`（M1-P2，见下节） |
| `SET_PORT_TUNNEL` | `0x06` | 主机 → 桥自己 | `[头(0x3f0)][len][0x20]["KXID"][0x06][index][crc16][0x7e]`（**T3**）——把该口置/清为**刷写隧道口**（幂等，见下节与「隧道单元」段） |
| `NEED_NODEID` | `0x20` | 设备 → 主机 | `[头(0x3f1)][len][0x20]["KXID"][0x20][uuid 大端 6 字节][app?][crc16][0x7e]`（`UnassignedNode::from_unit`） |

- uuid **高位字节在前**；应答末尾可带一枚 application 字节：无字节或 `0x01` = Klipper、
  `0x11` = CanBoot、其它 = `Unknown`（`application_name()`）——一字不带的旧形状也认。
- uuid/app 按**帧长字节定位**，所以它们里面出现 `0x7e` 是普通内容（帧内容允许 SYNC）。
- 指派后设备按 nodeid 正常收发（`0x100+2*id` 寻址），**已分配的设备不再响应 `0x00`**。

### 主机侧怎么用

- **扫描**（`klipperx scan --port <tty>`，`src/scan.rs`）：
  `serial_query_unassigned()` 广播一次查询，随后按与 `dispatch` 相同的成帧语义切单元
  （`UnitStream`），窗口是**从广播起算的截止时间**（`--timeout`，默认 2 s），按 uuid
  去重、每块板第一次被听到时回调——输出 `Found uuid=%012x, Application: …` 与总数行，`--interface <bus>` 则扫 CAN（同一个 `klipperx scan`），两者输出同形。CAN 侧的扫描实现在 `canserial.rs` 的 `query_unassigned`。`--host-library <path.so>` 则扫一份
  host 库（配 `--host-identity <12 位十六进制>`，默认全 `0`）：加载它、在
  `klipper_host_init()` **之前**用 `klipper_host_set_uuid()` 把身份交给它，再走同一段广播 /
  窗口 / 去重代码；`--host-identity` 只在给了 `--host-library` 时可用。
- **连接期**：`[mcu]` 节带 `uuid`（或别名 `canbus_uuid`）的共享串口，`attach()` 在发分配
  请求**之前**先写一枚 `SET_KLIPPER_NODEID`（按 uuid 点名 id），再走上一节的重确认
  握手——顺序与 CAN 的 `CanSerialDevice::open`（开链路时发 SET_NODEID）对应。

### 固件端契约（设备半已实现，见 `src/mux.c`）

`src/mux.c` 已实现下面的行为（`mux_input()` 在控制台任务里就地解析，`mux_admin_unit()` 处理
admin 信道，`mux_unit()` 决定归属）。**注意两条不同的寻址**：命令走 **admin id**（`0x3f0`），
而重新确认握手是**按 id 寻址**的（头部 `0x100+2*id`、帧体是空体 `KXID` 伪帧）——设备在自己的
`arb(id)` 上认出它、按 `+1` 回 `assignment_reply`；把两者混为一谈会让握手永远收不到回执
（这条是端到端测试抓出来的），并配有 `mux_query_unassigned` / `mux_set_node_id` /
`mux_need_nodeid` / `mux_assignment_reply` 与解析用的 `mux_parse_kxid`：

- 未分配的设备监听 `0x3f0`：收 `0x00` 在 `0x3f1` 回身份（uuid + application）；收
  `0x01 <uuid6> <id>` 且 uuid 是自己时**采纳 id** 并转入正常寻址。幂等：重复收到同一
  指派不改变已采纳的 id。
- **已分配**的设备对 `0x00` 保持沉默（所以扫描只列未分配的板）。
- 冲突处理留固件端：串口没有 CAN 的总线仲裁，同一根线上多板同时应答会撞字节——由固件
  结合物理接口避免或重试；主机侧只做单次广播 + 截止窗口 + uuid 去重，不重发。

### 端口级命令族（M1-P2 已落地，固件 `src/bridge.c`、主机 builder 在 `serial.rs`）

命令同样走 `0x3f0` 的 `KXID` 伪帧，但**面向桥自己**——桥是端口的拥有者，主机把「哪个 id 的设备
在哪条端口上」直接告诉桥，**下游设备全程不参与、不答任何 admin**（这就是「与没有 admin 的设备
共存」的机制）。全部**无回执**、参数不足/越界**静默丢弃**；除上面两条设备命令外**不下发任何
下游端口**（桥的 admin 处理函数以「是否消费」返回值短路本机设备半，`0x00`/`0x01` 的行为与扇出
逐位不变）。

| 命令 | 载荷 | 语义 |
|------|------|------|
| `SET_PORT_NODEID` | `[0x03][index][id][flags]`（4 字节） | 桥建 `id → 端口` 表并记该口 `flags`。`index` 0 起（= 该端口节的**声明顺序**，配置期由 `config_bridge_port` 登记）；`id` = MCU 层 id（`1..=255`，`mux_arb_id(id)!=0`）；`flags` bit0=`MUX_PORT_UNITS`（`0x01`）：置位 = 该口设备**说单元**（收发整单元原样转发）；清零 = **legacy**（下行剥头、上行按该口记的 id 加应答头）；保留位忽略。**幂等**：重复发送 = 覆盖同值；**旧映射不撤销**——同一口改登记为别的 id 后，旧 id 仍路由到该口（`bridge_map_id` 只增不删），想把下游设备改绑到别的口需另行定义 |
| `REQUEST_BOOTLOADER` / `RESTART_DEVICE` | `[0x02][index]` / `[0x05][index]` | 桥**只**转发到 `index` 口（UNITS 口带头转发、legacy 口剥头转发——legacy 设备的解析器会拒 seq `0x20`，属预期：无 admin 的 legacy 口只能走 `RESET_PORT`），不扇出全体；**但「index 指不到本机任何在用的端口」时不消费、穿透给本机设备半**（`BRIDGE` 是 `default y if MUX_SERIAL` ⇒ 每个 mux 设备自己也是桥构建，而「主机经别的桥找上我」时命令就是该由我执行的）——这正是「单设备 mux 板能被点名重启」的关键：设备半执行 `[0x02]`→`bootloader_request()`（`CONFIG_HAVE_BOOTLOADER_REQUEST` 守卫）、`[0x05]`→`mcu_restart()`，index 字节设备忽略 |
| `RESET_PORT` | `[0x04][index]` | 桥消费后对 `index` 口做**线级复位**：经 `bridge_port_ops.reset` 钩子（NULL=不支持 → 静默）；host 侧实现为 DTR\|RTS 断言 250 ms 的脉冲（**只对 USB-串口桥接板有效**：原生 USB CDC 板记下控制线状态但不当复位，见 `config.md` 的 `uart_restart_method`），用任务循环 idle 处的 poll 状态机延迟恢复（不阻塞不忙等；pty 等无 modem 线的 fd 返回 -1 静默）；**保持窗内重复发送按已完成处理**（不把此刻已断言的线态当成「原状态」重存，否则释放时恢复断言态、设备被永久按住） |
| `SET_PORT_TUNNEL` | `[0x06][index]` | 把 `index` 口置/清为**刷写隧道口**（**T3**）：**幂等 toggle**——置位后该口下行剥头直写 katapult 帧、上行把端口字节按隧道规则组单元回 `reply_arb_id(node_id)`；清位恢复常规模式。桥自己消费、**不转发下游**（与 `0x03` 同族）、无回执；短载荷/越界 index 静默丢弃 |

**端口不再上报字典表**（M1-P7 起）：`BRIDGE_PORT_COUNT`/`BRIDGE_PORT<n>_NAME`/`_TYPE`/`_PATH`
四键已**删除**。端口由**主机在配置期声明**——`config_uart oid=%c baud=%u slot=%c` 建一个 UART 对象（`slot` = 要用哪个端口槽，与下面的 `index` 同源一处计数；该槽没有运行期登记的 tty 时回落库构建期的默认路径），
`config_bridge_port oid=%c uart=%c` 把它接成桥的一个端口；**端口 `index` = 该端口节的声明顺序**
（固件 `bridge_add_port` 取表内首个空位，与主机的下发顺序一致），`SET_PORT_NODEID` 的 `index`
就是它。配置失败（baud 不可设、端口打不开、端口数超限）由固件 `shutdown` ⇒ 主机在**配置阶段**
拿到点名错误。host 侧 UART 后端**按端口槽各开一路流**（≤ `KLIPPER_HOST_PORT_SLOTS`，每槽独立
fd、整帧发送队列与 reset 状态机）：不同 `slot` 可同时打开；**同槽**重复 `config_uart` 报
`config_uart: the port slot is already in use`，slot 越界在后端点名拒绝（不走路径回落）。

**两级寻址**：第一级是「主机 ↔ 桥」——桥在上游链路上有自己的 mux id（或 host 库身份），命令走
admin id `0x3f0`；第二级是「桥 ↔ 下游某口的设备」——用 `SET_PORT_NODEID` 登记的 `id → 端口`
映射，数据单元按同一套 `0x100+2*id` 头路由到口（`bridge_route_unit`），口上再按该口的 `flags`
决定剥/加头。两级互不相干：第一级寻址不需要知道下游有哪些设备，下游也不需要 admin 能力。

## 设计规则总表（从决断提取）

这一节把散在各处的**规范性运行机制**收敛成一张表——每条一行、指向它的实现落点；判据、常量、
线上字节仍以本页其余小节与源码为准，这里不重复叙述。动这块代码前先核对本表：改的是哪条规则。

| # | 规则 | 落点 |
|---|------|------|
| 1 | **两级寻址互不相干**：第一级「上游链路是否按 id 加帧头」由**配置事实**判定，**不需要知道下游有哪些设备**；第二级「桥 ↔ 下游某口」由桥自己的 `id → 端口` 表判定 | `mcu/serial_ports.rs::is_id_addressed`、`mcu/host_libraries.rs::path_is_mux`、`SET_PORT_NODEID` |
| 2 | **端口 `index` = 端口节声明顺序**；`config_uart` 的载槽参数 `slot` 与它**一处计数** | `mcu/bridge_ports.rs::wire_bridge_port`、固件 `bridge_add_port` |
| 3 | `SET_PORT_NODEID` 的 `flags` bit0 = `MUX_PORT_UNITS`（`0x01`）置位 = 该口设备**说单元**；保留位忽略；命令**幂等**；**旧映射不撤销**（`bridge_map_id` 只增不删——同一口改登记别的 id 后旧 id 仍路由到该口） | 固件 `src/bridge.c` |
| 4 | **桥半对设备命令的穿透规则**：`[0x02]/[0x05][index]` 只有 `index` 指到**本机在用端口**才消费，否则**穿透给本机设备半**——每个 mux 构建自己也是桥构建（`BRIDGE` 是 `default y if MUX_SERIAL \|\| MUX_CAN`），「主机经别的桥找上我」时命令本就该由我执行 | 固件 `src/bridge.c::bridge_admin_unit` |
| 5 | **面向桥自己的端口级命令族全无回执**（`0x02`/`0x03`/`0x04`/`0x05`，**T3 起含 `0x06 SET_PORT_TUNNEL`**）：参数不足/越界**静默丢弃**；确认只有两个手段——**复位后能否重新握手/identify**、连不上就按配置阶段报错。设备级的 `0x00`/`0x01` 不在此列（设备用 `0x20` 应答 / 重确认握手回答） | 命令族表、`mcu/bridge_attach.rs` |
| 6 | **`0x02` 与 `0x05` 分工**：`0x02 REQUEST_BOOTLOADER` = 请求**进引导**（刷写工具的命令，**不进** `restart_method`；无 bootloader 的板是设计内 no-op）；`0x05 RESTART_DEVICE` = **重启固件本身**（`mcu_restart()`，`restart_method: admin` 走它）。两者都只转发到 `index` 口、不扇出 | 固件 `src/mux.c`、`src/bridge.c` |
| 7 | **一条链路一个读者、每引用自己的槽**：共享串口与 host 库同构（`SharedSerialPort`/`McuSlot`、`HostLibrary` 的 `Router`）——读者按 `assign::mcu_id_from_arb` 投槽，**无 id 的 admin 单元丢弃**（否则两个活引用会把字节流瓜分、谁都拼不齐帧）；读者**不持**强引用，避免「最后一个 `Drop` 落在读者线程 ⇒ `pthread_join(self)`」 | `devices/serial.rs`、`devices/host.rs` |
| 8 | **多份 host 库可同进程共存（2026-10-16 起）**：库不再装**进程级** `SIGUSR1`（主循环唤醒改按实例 `eventfd`；`SIGALRM` 那处死代码一并删除，见子模组提交 `host: end the task loop through a per-instance wake fd`），配置层与运行期两处「按路径拒绝第二条」的守卫**已去掉**（`host_libraries.rs`、`devices/host.rs`）。要跑两份**独立固件**必须加载**两份不同文件的拷贝**（`dlopen` 按 inode 识别，同一文件仍返回同一映射）——共存测试就是这么做的；同路径多节仍只加载一次并共享引用。**约束仍在**：`HOST_LIBRARY_TEST_LOCK` 之所以保留，是因为它护着进程级 stderr 重定向与 pty 资源，而不是库单例 | `devices/host.rs::test_two_host_library_copies_run_side_by_side` |
| 9 | **端口 tty 路径在库 `init` 前登记、运行期可注入**：`klipper_host_port(slot, path)`（`slot` = 端口 `index`，槽数 `KLIPPER_HOST_PORT_SLOTS = 4`，与固件 `BRIDGE_MAX_PORTS` 编译期绑定）；该槽未登记则回落构建期默认 `CONFIG_HOST_BRIDGE_SERIAL_PATH`（降级为兜底默认）；**同一路径的库在端口表不一致时拒绝复用**（`klipper_host_port()` 必须在 `init` 前调用——不做「表按路径合并」，那会让两条桥的槽号互相覆盖） | 固件 `src/host/host.c`、主机 `mcu/bridge_ports.rs` / `cmd/uart.rs` |
| 10 | **口形态判据是「设备说不说单元」，与台数无关**：legacy（无 `uuid:` + uart）⇒ 该口剥头/加头；命名（写了 `uuid:`）⇒ 整单元原样转发；**多站口**（同一端口节被 ≥2 台命名设备引用，如 RS-485）⇒ 都带头、各自走自己的 `uuid → id`。**legacy 只允许出现在单设备端口上**（它与邻居无法区分 ⇒ 在共享端口上声明 legacy 是配置错） | 两级寻址一节、[MCU 连接方式](../user-manual/mcu-connection.md) |
| 11 | **恢复预算：一次机会、归还条件明确**。桥 bring-up 因端口 `open` 失败、或 child 在连接期间被 `0x05` 重启而连不上 ⇒ **有界等 `uart_path` 回来**（`BRIDGE_PORT_RETURN_TIMEOUT` = 10 s、`BRIDGE_PORT_POLL` = 100 ms），回来就请求一次 **plain** restart（不是 `firmware_restart`，免得把子设备再点一次 `0x05` ⇒ 循环）。预算挂在 **`Printer`** 上**按节名**记（桥侧记桥节、child 侧记 child 节），**成功 connect 后归还**——所以「路径一直在但打不开」（目录/无权限/被占/槽越界）只重启**一次**便稳定 error，不会变成常驻重启风暴 | `mcu/object.rs`、`printer.rs::claim_bridge_port_recovery` / `clear_bridge_port_recovery` |
| 12 | **`MUX_SERIAL`/`MUX_CAN` 的载体条件与「不默认开」**：`MUX_SERIAL depends on SERIAL \|\| USBSERIAL`（载体存在才可见）+ 身份来源 `HAVE_CHIPID \|\| MACH_HOST \|\| MACH_LINUX`；`MUX_CAN depends on CANBUS && HAVE_CHIPID`；两者都 `select MUX`，`BRIDGE` 是 `default y if MUX_SERIAL \|\| MUX_CAN`。**都不默认开**：单元格式不是上游 klippy 主机说的格式，必须是刻意选择（host 库由 `HOST_MUX_SERIAL` 选上）。CAN 的**传输层**另是一个独立开关：`ISOTP`（`depends on CANSERIAL`，把 can-serial 的 CAN 承载换成 ISO 15765-2，与 `MUX_CAN` 正交可叠加） | 固件 `src/Kconfig` |

## 主机侧的行为约定

- **端口只开一次**：按 `serial:` 路径复用（`Weak` 注册表），第二颗 MCU 只是 attach 到同一个
  `SharedSerialPort`。
- **写锁**：`send` 与握手请求共用一把写锁，保证「头 + 帧」整段不被交错。
- **引用计数关闭**：`shutdown` 只停**本句柄**（其它 MCU 还在用）；**`shutdown` 与句柄析构都注销自己的槽**（守卫式 `unregister`：只有表里那个槽就是本句柄的才移除）（同 id 重 attach 不会看到陈旧积压）；最后一个持有者析构时才
  置停、唤醒所有槽位、关 fd、join reader 线程（若这次析构正跑在 reader 线程上，则跳过自 join、由 reader 下一轮 `weak.upgrade()` 拿到 `None` 后自然退出）。
- **baud**：同一路径多颗 MCU 的显式 `baud` 若不一致，装载期 `warn` 并**取最小值**（未写
  `baud` 的节跟着走）；不一致不再是配置错。
- **复位语义**：`restart_method` 的线级复位（`arduino` 的 DTR 翻转、`cheetah`、`rpi_usb`
  切电）是**整条线**上的动作，共享端口上会把所有 MCU 一起打断——所以共享路径在
  **解析层**就被强制成 `command`（`McuConfig::parse_restart_method` 收一个 `shared` 标志，
  判定用 `mcu/serial_ports.rs::path_is_shared`）：显式写了别的值记一条 warn，未写（串口
  默认 `arduino`）静默降级，非法拼写仍是配置错。强制在解析层发生，因此 `connect`、重连、
  `before_firmware_restart`、`reset_firmware` **全都**拿到 `command`，没有旁路；
  `Transport::open` 里对 cheetah 的 RTS 另有一条 warn（防御性，正常路径已不可达）。
- **桥下游设备的 attach 时序**（M1-P3 已落地，只对**给桥自己的**端口级命令）：identify 之后
  才拿得到端口表（§4.3），对账取 `index` 后主机按下面顺序发：
  1. 开一个**不握手**的下游句柄（同一共享资源/同一库的下一个引用，按该设备的 id 寻址）；
  2. 向桥发 `SET_PORT_NODEID(index, id, flags)`——**这一步必须在前**：路由不存在时，下面那条
     命名单元会被桥直接丢掉（`bridge_route_unit` 查不到 `id → 端口`）；
  3. legacy（无 `uuid:`）：到此为止，之后按**原始帧**收发；命名设备：再做
     `SET_KLIPPER_NODEID` + 重确认握手，之后按**整单元**收发（§4.5 的「两条都要发」）。
  顺序由调用方（`mcu/bridge_attach.rs`）持有，`serial.rs`/`host.rs` 只提供入口（`attach_downstream`/
  `load_downstream` + `handshake`/`attach` + `send_set_port_node_id`）。
- **复位/重启命令的主机接线**（M2 主机半边）：命名设备用 `restart_method: admin`——先重申
  `SET_PORT_NODEID`（端口 `flags` 是桥侧状态，未置位时该命令会被桥**剥头**、设备收到
  裸 `KXID` 帧而拒绝执行），再发 `RESTART_DEVICE [0x05][index]`（重启固件；`0x02` 是刷写工具
  的进引导请求，不进 `restart_method`）；legacy 设备由端口节
  `uart_restart_method`（只认 `arduino`/`cheetah`）声明，主机只发 `RESET_PORT [0x04][index]`
  （桥自己消费、做线脉冲，**不下发**）。两条都无回执 ⇒ 确认靠「复位后能否重新握手/identify」。

## 与 CAN 的关系

CAN 不需要这套东西：总线自带仲裁 id 与硬件过滤，每颗 MCU 各开一个 socket、各装一个只收
自己应答 id 的内核过滤器（`canserial.rs`），节点号由 `0x3f0` 的 admin 帧 fire-and-forget
交付。串口没有寻址，所以把同一套「`0x100 + 2*id` / 应答 `+1`」的语义搬到字节流里，多出来
的只有**帧头**和**一次握手确认**。admin 信道与命令族也逐字镜像 CAN（`0x00` 查询 / `0x01`
指派 / `0x20` 应答，见「发现与指派：admin 信道与命令族」一节），差别只在冲突处理：CAN 靠总线仲裁，串口留给固件端结合
物理接口实现。

## 尚未落地

- **微控制器架构的串口 mux 已接线**：`MUX_SERIAL` 现在是公共的、**有 prompt**
  能力（`depends on SERIAL || USBSERIAL`——载体接在哪里，选项就在哪里可见），选上后 MCU 控制台在架构
  无关层 `src/generic/serial_irq.c` 接 `mux_input`/`mux_send`（与 host 逐字同构），身份由各 arch 的
  `chipid_init` 按 `src/generic/chipid_uuid.h`（与 CAN 同一派生）设进 `mux_set_uuid()`。**USB CDC 控制台也已接线**
  （`src/generic/usb_cdc.c`：同样的两个接点 + `mux_send`；已在 stm32f103xe 真板上验过扫描/身份/经桥 identify），
  **linux 进程控制台**同样（`src/linux/console.c` + `src/linux/main.c` 的 `-U <12 hex>` 身份来源，
  `test/configs/linuxprocess-mux.config` 是构建目标）。**默认不开**：单元格式不是上游 klippy 主机说的格式，
  必须刻意选择（host 库由 `HOST_MUX_SERIAL` 选上）。真板上的指派/冲突处理、`irq_save()` 段内时序仍属真机项。
- **CAN 承载也已接线**（`MUX_CAN`）：`src/generic/canserial.c` 的字节流收发接同一套
  `mux_input`/`mux_send`（收端在 `command_find_block()` 之前、`irq_save()` 段内；发端 `console_sendf`
  预留 `MUX_HEADER_BYTES` + `mux_out_header`，`mux_send` 整单元入队），身份由 `canserial_set_uuid` →
  `mux_set_uuid()`（与 CAN uuid 同六个字节）。**CAN 单帧 admin 通道（`id=0x3f0`）不变**——单元跑在
  **节点自己仲裁 id 的字节流**里，与单帧通道是两条不同的通道（`0x02` 不会撞 `REQUEST_BOOTLOADER`）。
  **CAN 上游必须跑单元**：N 台下游设备的帧都复用到这一条字节流上，没有头就说不出哪台发的（串口上游
  可能只有一台，CAN 不会）；故选用「载荷内同一套单元」而非「单帧 admin 命令」（后者承载不了下游数据）。
  `depends on CANBUS && HAVE_CHIPID`、**不默认开**；`src-$(CONFIG_MUX) += mux.c` 让两种载体共用一份
  `mux.c`（`MUX_SERIAL`/`MUX_CAN` 由各 arch 的通信接口 `choice` 天然互斥）。**CAN 上还能换一副传输承载**：
  `CONFIG_ISOTP`（`src/generic/isotp.c`）把 can-serial 的「按帧容量切片」换成 ISO 15765-2 分段
  （一条 Klipper 帧 = 一条 ISO-TP 消息），而单元层与 `mux_input`/`mux_send` 一个字节不改；
  **`depends on CANSERIAL`、且只编 classical**。**`vcan` 上的端到端已落地**：`linux` 架构的
  SocketCAN 控制台（`LINUX_CANSERIAL`）让真 `klipper.elf` 能当 can-serial 节点，主机侧已实测
  跑通完整 `[mcu]` 连接（admin 指派 → identify → `get_clock`/stats）；剩下的未落地项只有
  `[can_device]` 下**游**端口（需 `BRIDGE_PORT_CAN` 端口后端）与真总线。
- **host 模式的数据面**：unit 接口已定义、固件也已按单元发出自己的帧，但**主机侧（klippyx）还没
  改用它**——`HostDevice` 仍走字节流/整帧。**分配段（扫描、`SET_NODEID`、重确认握手）走字节流
  这条已经通了**：mux 路径的真端到端 attach 就是走它。
- **多下游端口**：端口表支持多条，host 侧 UART 后端**按槽多路并行**（每槽独立 fd/发送队列/
  reset 状态机，同槽二次 `config_uart` 才被拒），**端到端已钉住**：同一份库下两个端口各挂一个真
  linux 固件进程、同一次 bring_up 内并发 identify + 各自两次 `get_clock`，库 stderr 断言两行 `… (fd N, slot 0/1)`（声明顺序
  即槽位；覆盖见[测试](testing.md)）。「某单元属于哪个端口」的归属策略**已由主机侧命令
  落地并被探针钉住**（`SET_PORT_NODEID` 建 `id → 端口` 表，见上节；真库探针见[测试](testing.md)）。
- **配置驱动的整链端到端：已通（含真机）**。宿主库的 mux 读端现在是**库级单读者 + 按 id 投槽**
  （与共享串口同构：一个读者按 `assign::mcu_id_from_arb` 把单元投到每引用自己的槽，无 id 的 admin
  单元丢弃），所以桥自己的引用与下游子设备的引用**不再互相吞字节**——此前的竞争消费会让两侧会话
  一起坏（现象：`not a frame` 警告 + identify 超时）。**真机实测**（宿主库当桥、端口接一台**旧固件
  真板**）：桥 `identified: 22 messages`、端口按配置期 `uart_baud` 打开、下游 `identified: 135
  messages`（与把同一块板子直连完全一致）、40s 内 0 错误；后续在 M1-P8 又修掉宿主库读端缺陷（库级单读者 + 按 id 投槽）。
- **host 侧读端口的时机**：`bridge_poll()` 插在任务循环的睡眠处（`src/host/timer.c` 的
  `irq_wait()`），不额外起线程；代价是下游字节要等任务循环醒着时才被捡起。
