# 串口复用协议（多 MCU 共用一个串口）

一个物理串口后面**只有一条字节流**，而 Klipper 的帧格式里没有板地址（`frame.rs`）——所以
「多颗 MCU 共用一个串口」必须自己给帧加**地址头**，并且要和设备端谈妥这件事。这份文档就是
那条线上的规格：**设备端（固件）按它实现，主机侧按它发送与分派**。

主机侧代码：`src/core/klippy/interface/devices/serial.rs`（端口、reader、分派、握手，
以及帧头与 KXID 伪帧的编解码）；仲裁 id 的映射单一定义在 `src/core/klippy/assign.rs`
（`arb_id`/`reply_arb_id`/`mcu_id_from_arb`/`is_reply_arb`，私有 `ARB_BASE`），CAN 与共享串口都从那里取。
什么时候用它、什么时候不用，由**配置事实**决定：同一个 `serial:` 路径被
**≥2 颗** `[mcu]` 引用即共享（`mcu/serial_ports.rs` 在装载期分组），只被 1 颗引用即
独占——独占串口走原来的 `SerialDevice`，**不发任何指派帧、不带任何头**，
线格式与 Klipper 逐字节相同。

> **固件侧：设备半与桥半都已实现，但还没有架构默认启用它们**。`src/mux.c` 有**单元格式与
> admin 单元族**，以及**设备半**：`mux_input()` 就地解析输入流（首字节高位置位 = 带头单元，
> 否则 = 原始帧），未分配时答扫描、按 `0x01` 采纳 node id、被指派到已持有的 id 时照 CAN 的
> `can_id_conflict` 弃号并 `shutdown`；已采纳 id 后自己发出的帧带应答头。`src/bridge.c`
> （`CONFIG_BRIDGE`）有**桥半**：端口表、`uuid → 端口` 与 `id → 端口` 两张路由表、admin 扇出
> （上游广播原样下达、下游应答**逐条**上行、不合并）、单设备路径**剥头/加头**；**M1-P2 起**
> 又加上**端口级命令族**（`SET_PORT_NODEID`/`RESET_DEVICE`/`RESET_PORT`，面向桥自己、无回执，
> 见下节「端口级命令族」）与 **identify 字典的端口表**（`BRIDGE_PORT*` 键），「单设备」由主机
> 下发的 `flags` 运行期告知而非编译期标志。host 库还有
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
| `RESET_DEVICE` | `0x02` | 主机 → 桥 → 单口 | `[头(0x3f0)][len][0x20]["KXID"][0x02][index][crc16][0x7e]`（M1-P2；对照 CAN `CMD_REQUEST_BOOTLOADER` 同值同义，见下节） |
| `SET_PORT_NODEID` | `0x03` | 主机 → 桥自己 | `[头(0x3f0)][len][0x20]["KXID"][0x03][index][id][flags][crc16][0x7e]`（`set_port_node_id(index, id, flags)`，见下节） |
| `RESET_PORT` | `0x04` | 主机 → 桥自己 | `[头(0x3f0)][len][0x20]["KXID"][0x04][index][crc16][0x7e]`（M1-P2，见下节） |
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
共存」的机制）。全部**无回执**、参数不足/越界**静默丢弃**；除 `RESET_DEVICE` 外**不下发任何
下游端口**（桥的 admin 处理函数以「是否消费」返回值短路本机设备半，`0x00`/`0x01` 的行为与扇出
逐位不变）。

| 命令 | 载荷 | 语义 |
|------|------|------|
| `SET_PORT_NODEID` | `[0x03][index][id][flags]`（4 字节） | 桥建 `id → 端口` 表并记该口 `flags`。`index` 0 起（对应 identify 字典端口表 `BRIDGE_PORT<n>_` 的下标）；`id` = MCU 层 id（`1..=255`，`mux_arb_id(id)!=0`）；`flags` bit0=`MUX_PORT_UNITS`（`0x01`）：置位 = 该口设备**说单元**（收发整单元原样转发）；清零 = **legacy**（下行剥头、上行按该口记的 id 加应答头）；保留位忽略。**幂等**：重复发送 = 覆盖同值；**旧映射不撤销**——同一口改登记为别的 id 后，旧 id 仍路由到该口（`bridge_map_id` 只增不删），想把下游设备改绑到别的口需另行定义 |
| `RESET_DEVICE` | `[0x02][index]` | 桥**只**转发到 `index` 口（UNITS 口带头转发、legacy 口剥头转发——legacy 设备的解析器会拒 seq `0x20`，属预期：无 admin 的 legacy 口只能走 `RESET_PORT`），不扇出全体、不触发本机设备半；无桥的板（纯 mux 构建）由设备半自己执行（照 `canserial.c` 的 `can_process_request_bootloader` 模式，`CONFIG_HAVE_BOOTLOADER_REQUEST` 守卫，index 字节设备忽略） |
| `RESET_PORT` | `[0x04][index]` | 桥消费后对 `index` 口做**线级复位**：经 `bridge_port_ops.reset` 钩子（NULL=不支持 → 静默）；host 侧实现为 DTR\|RTS 断言 250 ms 的脉冲，用任务循环 idle 处的 poll 状态机延迟恢复（不阻塞不忙等；pty 等无 modem 线的 fd 返回 -1 静默）；**保持窗内重复发送按已完成处理**（不把此刻已断言的线态当成「原状态」重存，否则释放时恢复断言态、设备被永久按住） |

**identify 字典的端口表**（仅桥构建出现，键名钉死供主机断言）：`BRIDGE_PORT_COUNT`（端口数）、
`BRIDGE_PORT<n>_NAME`（该口 oid 名，host 串口为 `"uart0"`）、`BRIDGE_PORT<n>_TYPE`（`"uart"`）、
`BRIDGE_PORT<n>_PATH`（诊断用，`CONFIG_HOST_BRIDGE_SERIAL_PATH` 的值）。`SET_PORT_NODEID` 里的
`index` 就是这张表的下标；表随 arch 编译期声明，多端口/其他 arch 后端出现时按同样形状扩展
（`BRIDGE_PORT<n>_*`）。

**两级寻址**：第一级是「主机 ↔ 桥」——桥在上游链路上有自己的 mux id（或 host 库身份），命令走
admin id `0x3f0`；第二级是「桥 ↔ 下游某口的设备」——用 `SET_PORT_NODEID` 登记的 `id → 端口`
映射，数据单元按同一套 `0x100+2*id` 头路由到口（`bridge_route_unit`），口上再按该口的 `flags`
决定剥/加头。两级互不相干：第一级寻址不需要知道下游有哪些设备，下游也不需要 admin 能力。

## 主机侧的行为约定

- **端口只开一次**：按 `serial:` 路径复用（`Weak` 注册表），第二颗 MCU 只是 attach 到同一个
  `SharedSerialPort`。
- **写锁**：`send` 与握手请求共用一把写锁，保证「头 + 帧」整段不被交错。
- **引用计数关闭**：`shutdown` 只停**本句柄**（其它 MCU 还在用）；最后一个持有者析构时才
  置停、唤醒所有槽位、关 fd、join reader 线程。
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
- **复位命令的主机接线**（M2 主机半边）：命名设备用 `restart_method: admin`——先重申
  `SET_PORT_NODEID`（端口 `flags` 是桥侧状态，未置位时 `RESET_DEVICE` 会被桥**剥头**、设备收到
  裸 `KXID` 帧而拒绝执行），再发 `RESET_DEVICE [0x02][index]`；legacy 设备由端口节
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

- **微控制器架构的串口 mux 已接线**（M1-P7 的 FW-2/FW-3）：`MUX_SERIAL` 现在是公共的、**有 prompt**
  能力（`depends on SERIAL`），选上后 MCU 控制台在架构无关层 `src/generic/serial_irq.c` 接
  `mux_input`/`mux_send`（与 host 逐字同构），身份由各 arch 的 `chipid_init` 按
  `src/generic/chipid_uuid.h`（与 CAN 同一派生）设进 `mux_set_uuid()`。**默认不开**：单元格式不是
  上游 klippy 主机说的格式，必须刻意选择（host 库由 `HOST_MUX_SERIAL` 选上）。真板扫描/指派/冲突
  与 `irq_save()` 段内时序仍属真机项。
- **host 模式的数据面**：unit 接口已定义、固件也已按单元发出自己的帧，但**主机侧（klippyx）还没
  改用它**——`HostDevice` 仍走字节流/整帧。**分配段（扫描、`SET_NODEID`、重确认握手）走字节流
  这条已经通了**：mux 路径的真端到端 attach 就是走它。
- **多下游端口**：端口表支持多条，但今天只登记一条（host 那条 tty），多端口扇出尚未被测过。
  「某单元属于哪个端口」的归属策略**已由主机侧命令落地并被探针钉住**（`SET_PORT_NODEID` 建
  `id → 端口` 表，见上节；真库探针见[测试](testing.md)）。
- **配置驱动的整链端到端**：夹具（真 `.dict` 假固件经真桥答 identify/`get_clock`）与配置驱动路由
  （子设备裸帧确实到端口）**两半**都已实测，但合起来那条在**单进程内做不到**——宿主库只有一个
  输出队列，桥自己的引用会把子设备的回包偷走（实测：子设备 identify 请求被重发 ~28 次）。要真
  验需下游固件跑**独立进程**（或真机）。详见[工作记录](../work-log/2026-10-13-bridge-port-notes.md) §7 的欠项。
- **host 侧读端口的时机**：`bridge_poll()` 插在任务循环的睡眠处（`src/host/timer.c` 的
  `irq_wait()`），不额外起线程；代价是下游字节要等任务循环醒着时才被捡起。
