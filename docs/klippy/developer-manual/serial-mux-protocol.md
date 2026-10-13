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

> **固件侧部分落地，设备半的状态机仍待做**。已实现的是**单元格式与 admin 单元族**（固件
> `src/mux.c`：2 字节带校验头、`KXID` 伪帧、`QUERY_UNASSIGNED`/`SET_KLIPPER_NODEID`/
> `NEED_NODEID` 的构造与解析）、host 库的**下游串口端口**（`CONFIG_BRIDGE_PORT_SERIAL`）
> 与 **unit 收发接口**（`klipper_host_input_unit`/`klipper_host_output_unit`，需要帧接口开关
> **且**桥模式）。还差**设备半状态机**（应答扫描、采纳 nodeid、按 CAN 规则处理 id 冲突、把
> self 的帧交给本机解析器），所以今天的固件仍不回执：共享模式必然**握手超时**，误开到旧固件
> 只会连接失败，不会破坏会话。

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
  「加长的形式」这条路；保留带同时意味着 `nodeid` 可到 383（远超 CAN admin 报文单字节 `nodeid`
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

### 固件端契约（设备半状态机待实现）

单元格式与 admin 单元族已在固件里（`src/mux.c`，含 `mux_query_unassigned` /
`mux_set_node_id` / `mux_need_nodeid` / `mux_assignment_reply` 与解析用的 `mux_parse_kxid`）；
**下面这条状态机行为**（谁来应答、何时采纳、冲突怎么办）还没接上：

- 未分配的设备监听 `0x3f0`：收 `0x00` 在 `0x3f1` 回身份（uuid + application）；收
  `0x01 <uuid6> <id>` 且 uuid 是自己时**采纳 id** 并转入正常寻址。幂等：重复收到同一
  指派不改变已采纳的 id。
- **已分配**的设备对 `0x00` 保持沉默（所以扫描只列未分配的板）。
- 冲突处理留固件端：串口没有 CAN 的总线仲裁，同一根线上多板同时应答会撞字节——由固件
  结合物理接口避免或重试；主机侧只做单次广播 + 截止窗口 + uuid 去重，不重发。

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

## 与 CAN 的关系

CAN 不需要这套东西：总线自带仲裁 id 与硬件过滤，每颗 MCU 各开一个 socket、各装一个只收
自己应答 id 的内核过滤器（`canserial.rs`），节点号由 `0x3f0` 的 admin 帧 fire-and-forget
交付。串口没有寻址，所以把同一套「`0x100 + 2*id` / 应答 `+1`」的语义搬到字节流里，多出来
的只有**帧头**和**一次握手确认**。admin 信道与命令族也逐字镜像 CAN（`0x00` 查询 / `0x01`
指派 / `0x20` 应答，见「发现与指派：admin 信道与命令族」一节），差别只在冲突处理：CAN 靠总线仲裁，串口留给固件端结合
物理接口实现。

## 尚未落地

- **设备半状态机**：应答 `QUERY_UNASSIGNED`、采纳 `SET_KLIPPER_NODEID`、按 CAN 的
  `can_id_conflict` 处理 id 冲突、以及「首字节高位置位=带头单元，否则=原始帧」的双态解析。
  共享路径在它就绪前必然握手超时，发现扫描也只会等到截止窗口走空（预期行为，不是 bug）；
  单元格式与 admin 单元族本身已在固件里（`src/mux.c`）。
- **host 库路径的 id 寻址**：配置层已能判定一条 `host_library:` 路径是否需要按 id 寻址
  （`mcu/host_libraries.rs`，见[连接方式](../user-manual/mcu-connection.md)一节的规则），但
  **帧头消费**（attach 时按 mux 走、以及把 self 的帧交给本机解析器）尚未接入。独占串口与
  判定为**非 mux** 的 host 库路径、以及 `test:` 也仍不使用 id 寻址——但注意 id 本身**每个 `[mcu]` 都有**（`McuObject::connect` 没有它就拒绝开传输层），对它们只是标识与分配序列的一部分。
- **host 模式的数据面**：`klipper_host_input_unit()` / `klipper_host_output_unit()` 已定义
  （一次一整单元：2 字节头 + 帧，头里就是来源设备），但固件尚未按单元发出上游帧、主机侧也
  还没改用它。**分配与扫描那段对话始终走字节流**（`klipper_host_input` /
  `klipper_host_output_wait`），这套 unit 接口只是 host 的"整单元"快路径。
