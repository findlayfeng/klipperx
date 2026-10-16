# Katapult 拓扑刷写：动工前调查、决策与任务拆分（2026-10-16）

> 状态：**已拍板、已派发（2026-10-16）**——§6 三项拍板点定了 ①A ②A ③A（+独立执行文件），
> T1（双重封装原型）与 T4a（katapult 会话核心）并行在跑；T2 等 T1、T3 等 T2、T4b 等 T4a。
> 规格缺口先查 §8 附录（三路侦察全文，file:line 级）。
> 本篇是一次性调查记录（非规范）；实现落地后按 work-log 惯例补「实施记录」节。

## 1. 需求（用户口径，2026-10-16）

引入 bootloader，目标是**复用 klipperx 的设备拓扑，升级时不重新接线**。流程：

1. 子设备需要升级时，工具先把**整个拓扑复位**；
2. **仅配置**「到目标设备路径上」的桥；
3. 把目标设备**重启进升级模式**；
4. **通过桥转发刷机包**完成刷机；
5. 直连设备（独占串口/USB/CAN）也要支持直接升级（可以不复用上游脚本）。

决策问题：**引入 katapult 合适，还是自行实现合适？** —— 已裁定，见 §2。

## 2. 决策：引入 katapult（bootloader 半）+ 自研「拓扑刷写工具 + 桥隧道」（主机半）

关键认识：**「复用拓扑」的缺口与选哪个 bootloader 基本无关**——缺口在主机工具与桥的转发路径上，
换任何 bootloader 都要做；而 bootloader 本身（按芯片擦写 flash、掉电安全、USB/CAN/UART 栈）才是真正
难且必须二选一的部分，那一半 katapult 已经做完且是 klipper 生态标准件。

支持引入的四条（全部有据，见附录）：

1. **直连路径零代码**：独占串口线格式与上游逐字节相同（侦察B §1.1）、CAN 寻址逐位同值
   （`assign.rs` 与上游同式）、USB 只是 id 不同（katapult `1d50:6177` / klipper `1d50:614e`）——
   上游 `flashtool.py` 对直连设备今天就能用。
2. **协议对隧道最友好**：katapult 是无寻址裸字节流，寻址天然由 mux 层承担；固件解析器靠
   `0x01` 扫描 + CRC 自同步、NACK 有界（`third_party/katapult/src/command.c:99-141`，已逐行核）；
   **双重封装（katapult 帧塞进 klipper 帧载荷）能被它原样捞出来**，隧道设计有余量。
3. **钩子已就位**：本仓固件已有 `CONFIG_HAVE_BOOTLOADER_REQUEST`、端口级 `REQUEST_BOOTLOADER (0x02)`
   与 CAN 同值同义、发现应答已能命名 "CanBoot"（application 字节 `0x11`，`serial.rs:833-835`）；
   katapult 用 klipper 构建系统，与 `third_party/klipper` fork 同源。
4. **自行实现的唯一吸引力**（bootloader 原生说 mux、省掉隧道）换不来整套 bootloader 自研的代价：
   stm32 全系/rp2040/rp2350/lpc176x 的 flash 算法、掉电中断安全、USB/CAN 栈、真机验证——
   全是最高风险、最耗硬件时的部分，且与「拓扑复用」毫无关系。

子模组已入主线：`third_party/katapult`（`https://github.com/findlayfeng/katapult.git` 的
`klipperx` 分支，gitlink `3b973a4`），检出解析 `katapult_dir()`/`KLIPPERX_KATAPULT_DIR` 与
klipper 同款约定（提交 `d6a66c4`，已 ff 进 `work`）。

## 3. 兼容性评估结论（评估对象：katapult 协议 × 本仓通信层改动）

| 维度 | 裁定 | 要点（证据见附录） |
|---|---|---|
| 帧层 | **互不混淆** | katapult `<0x01><0x88>…<CRC16><0x99><0x03>`；klipper 应用帧 `[len 5..64][seq\|0x10]…[crc][0x7e]`；mux 头首字节恒 `0x80` 置位。三道闸：framing bit（`serial.rs:705`、`mux.c:513`）、长度/seq 值域（`0x88` 当 seq 高 4 位是 `0x8`，判非法）、CRC+尾定界互不过对方校验。坏帧只丢弃+重同步、不误执行；katapult NACK 有界（`CF_NEED_VALID`：两次有效帧间至多一次） |
| 寻址 | **逐位同值** | 数据 id `0x100+2*id` / `+1`（`assign.rs:42-56`）；admin `0x3f0/0x3f1`；bootloader 与应用共用 uuid，node id 分区错开（flashtool ≥128 / klippy ≥4）；进 bootloader 不改 id（`canserial.c` 只在 set_nodeid 时改） |
| 握手 | **超时而非误触发** | 主机对 bootloader 态设备：mux 握手收不到 `+1` 回执 → 3 次后 `InterfaceError::Timeout`，连接失败不破坏会话（`serial.rs:1269-1305`）；KXID 的 seq `0x20` 被 `Frame::check` 拒收（`frame.rs:126-130`）；旧固件丢 KXID 有永久测试（`serial.rs:3046-3130`） |
| 共存 | **安全但送不到** | 共享口主机 reader 见无头帧 → `HeaderScan::Invalid` → `drop_to_sync` 整段丢弃（katapult 帧内无 `0x7e`，至多吃掉其后第一个合法单元，可自愈，`serial.rs:1542-1612`）；固件设备半把裸 katapult 帧当非法长度原始帧丢弃（`mux.c:513-516`） |
| 串口↔CAN 帧 | **协议帧逐字节相同，非 1:1** | 同一编码器/解析器（`command.c:24-28`、`command_find_block`）；CAN 只是把字节流切 ≤8 B 片段（`flashtool.py:662-677`、`canserial.c:226-236`），Connect=1 帧、64 B 块的 Send Block=10 帧；admin 通道（query `0x00`/set `0x11`/clear `0x12`/resp `0x20`）是 CAN 独有旁路，不套 katapult 帧 |
| 上游基线 | **边界一致** | klippy 运行时从不发 katapult 帧，katapult 对话只在独立脚本（`lib/katapult/flashtool.py:56-66`）；进 bootloader 三种入口（USB DTR 1200、串口 magic 串、CAN admin）本仓全保留，端口级 `0x02` 是超集扩展 |

## 4. 缺口清单（唯一工程量所在：数据面隧道）

1. **共享口下行**：主机所有共享口写出一律带头、无裸写入口（`serial.rs:1879-1886`，
   侦察B §0②）；`mux_unit_len` 用 klipper 长度语义校验 unit 载荷（`mux.c:484-505`），
   katapult 帧裹进 unit 会被量错长度当 junk 丢头。两条路：
   - **A. 双重封装（零固件改动出原型）**：katapult 帧塞进 klipper 帧载荷发出，桥 legacy 口
     剥头后 bootloader 靠 `0x01` 扫描 + CRC 自同步捞出内层帧；尾部 klipper crc/`0x7e` 被当垃圾重同步掉。
   - **B. unit 加「bootloader 载荷」形态（正式方案）**：`mux_unit_len` 认可 katapult 的
     word_cnt 长度语义（或加 unit 类型位），主机侧 `serial.rs` 对应加一种 unit 构造。
2. **桥回程（最大固件工作）**：下发方向 legacy 口剥头已够用；**回程**桥的端口读端按 klipper
   帧解析，katapult 应答（首字节 `0x01` < `MESSAGE_MIN`）被 `drop_to_sync` 丢掉。需要给桥加
   显式「**刷写隧道模式**」：把端口上的原始字节收拢成单元上行——不要靠解析巧合。
3. **工具（主机侧，自研）**：§1 的状态机——复位拓扑 → `SET_PORT_NODEID` 配路径 →
   `REQUEST_BOOTLOADER` → 隧道刷写（katapult 6 命令 + 重试节拍：`tries=5`、`read_timeout=2s`、
   prime 帧、busy/NACK 处理，`flashtool.py:321-406`）→ `COMPLETE` → 重新握手验收。
   直连路径直接说裸 katapult 帧（CAN/独占口/USB 现成可用）。
4. **验收锚点**：坏帧只丢弃不误投的性质已被钉死（上表「握手」「共存」两行的测试），
   隧道设计沿用同一原则；桥的隧道模式要配自己的永久测试（参照 mux 端口命令族探针
   `test_the_bridge_port_commands_over_the_real_library` 的做法）。

## 5. 已就位的资产

- **控制面四命令族**（`serial-mux-protocol.md:173,224,253`；M1-P2 起）：
  `SET_PORT_NODEID (0x03)` 配路径、`RESTART_DEVICE (0x05)` / `RESET_PORT (0x04)` 复位、
  `REQUEST_BOOTLOADER (0x02)` 进引导——**全部无回执，确认靠「能不能重新握手」**；
  `send_request_bootloader` 至今**无生产调用者**（侦察B 穷举证实）——新工具就是第一个消费者。
- **子模组与解析**：`third_party/katapult @ 3b973a4`、`katapult_dir()`/`KLIPPERX_KATAPULT_DIR`
  （`d6a66c4`）。
- **恢复语义**：设备重启重枚举 USB 后主机会有界等 `uart_path` 回来（≤10s）并自动恢复一次
  （`mcu-connection.md:123`）——刷完重启的验收路径可复用同一机制。

## 6. 任务拆分（最小单元；依赖排序；worker 任务书按六要素 + 防超时三板斧写）

| # | 单元 | 内容 | 依赖 | 验收 |
|---|---|---|---|---|
| T1 | 原型通路（双重封装） | 主机侧测试：klipper 帧载荷裹 katapult 帧 → `mux_input` 半/桥 legacy 口剥头 → katapult 解析器（C 可执行移植，参照 `serial.rs:3046-3130` 的做法）捞出内层帧；只做测试与小改，证明/证伪 §4-A | 无 | 新增永久测试绿；结论回填本篇 §4 |
| T2 | 隧道 unit 形态（固件+主机） | §4-B：`mux_unit_len` 认可 word_cnt 长度语义 + unit 构造/解构（`mux.c`/`serial.rs`）+ 双侧永久测试 | T1 结论 | 双侧测试钉住字节形状；旧固件/旧主机行为不变（现有测试全绿） |
| T3 | 桥刷写隧道模式 | 固件：桥端口读端收拢原始字节上行、下行剥头直写；模式开关（配置期或 admin 命令，需先定）；参照端口命令族探针写真库端到端测试 | T2 | 新探针测试逐字节钉往返；不进模式时行为与现状逐字节相同 |
| T4 | 刷写工具骨架（直连） | 主机：katapult 会话（6 命令/重试/prime/verify/COMPLETE）+ 直连三路（独占串口/CAN/USB）；直连 CAN 与独占串口可直接复用上游 flashtool 语义 | 无（与 T1-T3 并行） | 对假 bootloader（脚本或测试夹具）跑完整刷写用例；直连真机项登记 TESTING.md |
| T5 | 工具拓扑流程 | §4-3 的状态机：复位拓扑 → 配路径 → 进引导 → 隧道刷写 → 验收；消费 T2/T3 | T2、T3、T4 | 端到端（桥夹具 + 假设备）全绿；真机项清单 |

拍板点（2026-10-16 用户已拍，写任务书按此）：
1. **隧道模式开关 = A：新增 admin 命令 `SET_PORT_TUNNEL (0x06)`**，与 `REQUEST_BOOTLOADER (0x02)`
   同族同路由（按 `index` 转发、无回执、确认靠隧道往返通不通）；退出=发命令，不重注册端口。
2. **隧道单元上限 = A：128 B**（= katapult `MESSAGE_MAX`，`third_party/katapult/src/command.h:40`）；
   下行整帧一 unit，回程按整帧或 ≤128 B 片段上行、主机侧重拼（katapult 字节流自同步）。
   单元形态判据（供 T2 任务书）：载荷首两字节 `01 88` ⇒ 隧道单元，长度 = `word_cnt*4+8`（≤128）；
   否则仍按 klipper 长度语义——头格式不变、只改两处长度判据与两处读写路由。
3. **工具形态 = A：独立 `[[bin]]` 执行文件**（项目内样板：`src/bin/klippy/main.rs` 的独立 bin；
   子命令样板 `src/scan.rs` 一并参考），核心会话/流程逻辑放 lib 供入口与测试共享；
   是否再挂 `klipperx flash` 子命令由 T4b 报告后 main 定。

## 7. 风险与未验证项

1. 双重封装在**真 katapult 固件**上未实测（T1 就是干这个的；静态分析结论：`command_find_block`
   的 resync + CRC 足够，`0x01` 出现在内层帧头必然先被找到）。
2. CAN 进 bootloader 后**同一数值 node id 是否保持**：katapult 源已核（进引导不改
   `assigned_id`），但主机重连路径（再次 `SET_NODEID` 的时序）没跑过真机。
3. 桥隧道模式与「端口上还有别的 MCU 在跑」的互斥语义未设计——T3 任务书必须先定。
4. `0x3f0` 广播在 bootloader 存活期间会被 MCU 处理（上游 `Bootloader_Entry.md:77-83`）——
   隧道期间 admin 扫描要避免误清 node id（katapult 的 clear 是全局的）。
5. 工具重试节拍取自 flashtool（5 次 / 2s），隧道增加了桥转发延迟，上限值需在 T5 实测调。

## 8. 附录：三路侦察全文（2026-10-16，只读侦察，file:line 级）

- 附录A：katapult 协议与传输层规格（`~/project/katapult`）
- 附录B：本仓通信层改动与 bootloader 交互落点
- 附录C：上游 klipper 与 katapult 的集成基线

---

# 附录A：katapult 协议与传输层规格（侦察A）

只读仓库：`/home/fengcheng/project/katapult`。未改任何文件、未执行任何 git 写操作、未向下委派。
标注约定：**[证实]** = 直接读到代码/文档；**[推演]** = 由代码静态追踪得出、未运行验证；**[推测]** = 边界内无法核实的外部事实。

---

## 0. 边界与文件清单

- 点名文件已读：`protocol.md`（198 行）、`README.md`（245 行）、`scripts/`（先 ls）、`src/`（先 ls，重点 generic/ 与各芯片目录）。
- `scripts/` 清单：`flash_can.py`、`flashtool.py`、`buildbinary.py`、`buildcommands.py`、`check-gcc.sh`、`make-version.sh`、`test-build.sh`、`uf2_append_boot_signature.py`。**没有独立的 canflash 脚本**；刷写/查询都在 `flash_can.py`。
- **`scripts/flash_can.py` 与 `scripts/flashtool.py` 字节级完全相同**（`diff -q` 退出码 0，[证实]）。下文统称 `flash_can.py`，行号对两者同时有效。
- `src/` 关键文件：`command.c/.h`、`flashcmd.c`、`sched.c`、`bootentry.c`、`generic/{canserial.c/.h, canbus.c/.h, serial_irq.c, usb_cdc.c, armcm_canboot.c}`、`stm32/can.c`、`rp2040/can.c`。

---

## 1. 帧格式（Frame）

**[证实]** 主机↔bootloader 命令与响应共用同一帧：

```
<0x01><0x88><cmd/ack 1B><payload字长 1B><payload(4B倍数)><CRC16-LE 2B><0x99><0x03>
```

| 事实 | 证据 |
|---|---|
| 帧定义（头/尾/字长/CRC 覆盖范围/小端） | `protocol.md:9-24` |
| 命令 0x11..0x16 定义 | `protocol.md:29-158`；`src/command.h:27-32`；`flash_can.py:59-64` |
| 响应 0xA0/0xF1/0xF2/0xF3 | `protocol.md:163-197`；`src/command.h:33-35`（只有 A0/F1/F2）；`flash_can.py:67-70` |
| CRC16-CCITT，init 0xFFFF | `flash_can.py:41-50`（注释注明来源 klipper msgproto.py）；固件校验 `src/command.c:118-122` |
| 主机构帧：CRC 覆盖 cmd+len+payload，CRC 小端 | `flash_can.py:253-263`（`_build_command`） |
| 固件构响应帧：`0x8801 | cmdid<<16 | len<<24` + CRC + `0x0399<<16` | `src/command.c:20-33`（`command_respond`） |
| 帧最小 8 字节、最大 128 字节（`MESSAGE_MAX 128` → 字长 ≤ 30） | `src/command.h:38-50`；校验 `src/command.c:109-111` |
| 协议版本 1.1.0（`PROTO_VERSION 0x00010100`） | `src/command.h:26` |

**[推演→需注意]** 协议文档与主机脚本都定义了 **0xF3 busy**（`protocol.md:191-197`、`flash_can.py:70,383-385` 收到 busy 会 `sleep(1.5)` 后重试），但 **`src/` 全树 grep 未找到任何发送 0xF3 的代码**（只有 `RESPONSE_ACK/NACK/COMMAND_ERROR`，`src/command.h:33-35`）。当前固件源码中 busy 为「文档/主机侧保留、固件不发」。

---

## 2. Q1：传输层与寻址

**三种传输：CAN、USB（CDC-ACM 虚拟串口）、UART** [证实] `README.md:4-8`（"In addition to CAN, Katapult now supports USB and UART interfaces"）、`README.md:35-40`（menuconfig Communication interface）。

### 2.1 CAN（11-bit 标准 id）

- **主机侧发包**：id > 0x7FF 才置 EFF 标志（节点 id ≤ 0x2FF，永远标准帧）；长载荷按 8 字节切片成多帧连续发送，帧内无额外封装 —— `flash_can.py:662-680`；收包格式 `CAN_FMT = "<IB3x8s"`（16 字节 recvmsg）`flash_can.py:52`，`_handle_can_response`/`_process_packet` 按 id 分发到对应节点流 `flash_can.py:629-660`。
- **node id → 仲裁 id 的关系：`can_id = node_id × 2 + 0x100`，应答 id = 该 id + 1**
  - 固件解码：`src/generic/canserial.c:138-141`（`can_decode_nodeid: (nodeid << 1) + 0x100`）
  - 主机编码：`flash_can.py:742`（`decoded_id = node_id * 2 + 0x100`）
  - 固件发送用 `assigned_id + 1`：`src/generic/canserial.c:57-63`；主机监听 `decoded_id + 1`：`flash_can.py:745`
- **node_id 字节由主机在分配时指定**：`node_id = len(self.nodes) + CANBUS_NODEID_OFFSET(=128)`，作为 admin「set node id」载荷的第 8 字节发出 —— `flash_can.py:84,734-743`。（脚本节点表预置了 0x3f1 → 首个节点 node_id=129 → 仲裁 id 0x202，**[证实]** 数值推算。）固件分配时校验 UUID、冲突则清 id：`src/generic/canserial.c:177-191`（`can_check_uuid` 125-131）。
- **广播/查询 id**：
  - admin 广播 id **0x3f0**、admin 应答 id **0x3f1** —— `src/generic/canserial.h:6-7`、`flash_can.py:78-79`
  - 查询未分配节点：主机向 0x3f0 发裸载荷 `[0x00]`（`CANBUS_CMD_QUERY_UNASSIGNED`，`flash_can.py:698-701`、定义 `flash_can.py:80`）；未分配节点应答 0x3f1：`[0x20][6B UUID][app标记]`，app 标记 `0x11`=Katapult、`0x01`=Klipper —— 固件 `src/generic/canserial.c:144-157`（`data[7]=0x11`，仅 `assigned_id==0` 时应答，146-147），主机过滤 `app == "Katapult"` `flash_can.py:711-726`
  - 清除已分配 id：向 0x3f0 发 `[0x12]` —— 主机 `flash_can.py:729-732`，固件 `src/generic/canserial.c:163-167`
  - 让正在跑 Klipper 的节点进 bootloader：向 **0x3f0** 发裸载荷 `[0x02][6B UUID]`（klipper admin reboot）—— `flash_can.py:73-75,692-697`
- **数据通道**：主机把 katapult 帧字节直接塞进 CAN 数据（每帧 ≤8 字节），节点 id = 分配 id —— 无帧头级寻址，**寻址完全由 CAN id 承担** [证实]。

### 2.2 UART（裸字节流，无地址头）

- 固件按字节收进 192B 缓冲，收到 `0x03`（尾同步字节）唤醒解析 —— `src/generic/serial_irq.c:17-37`（`serial_rx_byte`，29-30 唤醒）；`console_task` 直接调 `command_find_block` —— `serial_irq.c:75-87`。**无任何地址头**。
- 主机 `SerialSocket.send(can_id, payload)` **忽略 can_id，直接写串口** —— `flash_can.py:901-907`；`CanNode(0, …)` 节点 id 为 0 —— `flash_can.py:881`。
- 默认波特 250000 —— 主机 `flash_can.py:1145-1148`；固件 `DECL_CONSTANT("SERIAL_BAUD",…)` `src/generic/serial_irq.c:22`。
- UART **没有查询/发现命令**：`SerialSocket.is_query → False` `flash_can.py:884-885`；`-q` 帮助标注 "CANBus Ony" `flash_can.py:1153-1155`。

### 2.3 USB（CDC-ACM，同样裸字节流）

- 固件 bulk-OUT → 128B 缓冲 → `command_find_and_dispatch`（同帧解析）—— `src/generic/usb_cdc.c:89,98-127`（115 行）。**无地址头**，寻址靠 USB 设备本身。
- USB 身份：Katapult `1d50:6177`、Klipper `1d50:614e`、GS-CAN bridge `1d50:606f` —— `flash_can.py:86-88`；主机用 manufacturer `katapult`/`klipper` 判定当前跑的是谁 —— `flash_can.py:1049-1067`、`786-810`。
- **USB-CAN bridge 形态**：脚本先按 CAN 走（检测 GS-CAN + manufacturer `klipper`，序列号换 uuid，`flash_can.py:747-784`、`201-209`），发 reboot 后等设备以 katapult 身份重连（8×0.5s，`flash_can.py:786-810`），随后**改用 SerialSocket 走 bridge 的 tty** —— `flash_can.py:1105-1109`。[证实]

---

## 3. Q2：发现与连接

### 3.1 CAN 发现流程（主机 `run()` 时序，`flash_can.py:812-851`）

1. bind 接口（816-819 失败 → FlashError）。
2. 有刷写/`-r` 请求：向 0x3f0 发 klipper reboot `[0x02]+uuid`（823 → `_jump_to_bootloader` 692-697）；USB-CAN bridge 等重连 ≤8×0.5s=4s（790-810），普通 CAN `sleep(1.0)`（828）；仅 `-r` 则到此为止。
3. 发 `[0x12]` 清所有节点已分配 id（831-832），`sleep(0.5)`。
4. `-q` → `_query_uuids`（833-835 → 698-727）：
   - 命令：0x3f0 + `[0x00]`；**超时窗口 2.0 秒**（703），窗口内动态 `read(8, timeout=min(剩余, …))`（705-712）。
   - 判定：`resp[0]==0x20` 且 7 字节以上（711），`resp[7]==0x11` 才算 Katapult（714-726）。
   - **失败判定：窗口内无应答 → 返回空列表，不抛异常**；`main()` 仍打印 "CANBus UUID Query Complete"（1118-1119）——是否为空由调用方判读 [证实]。
   - README 警告：**只应在总线单节点时查询**，多节点应答冲突可致 bus-off，须复位恢复 —— `README.md:107-110`。
5. `_set_node_id`：0x3f0 + `[0x11][6B uuid][node_id]`（736-743），`sleep(0.5)`（838）。
6. `connect_btl()` 发 Connect（840）→ `verify_canbus_uuid()` 发 0x16 比对 6 字节 UUID（841；314-319，不匹配 → FlashError 319）。**0x16 仅 CAN 固件支持**：`src/command.c:84-90`（非 `CONFIG_CANSERIAL` 落入 default → 回 0xF2）；串口主机脚本不调它（1076-1079 无 verify uuid）[证实]。

**连接/命令级重试**（`send_command`，`flash_can.py:321-407`）：
- 默认 `tries=5`、`read_timeout=2.0s`（325-326）；读到 `0x99 0x03` 为止，丢弃不含 `0x01 0x88` 头的字节逐字节重同步（338-344）。
- 每次重试前：`read(1024, timeout=.25)` 清缓冲（401-404）+ `sleep(0.5)`（406）；**5 次耗尽 → `FlashError`（407）→ main 捕获返回 1**（1110-1113）。
- busy(0xF3)：`sleep(1.5)` 后继续（383-385，计入 tries）；NACK/0xF2/CRC/尾错：仅记日志后重试（369-397）。
- 连接最坏耗时 ≈ 5×(2+0.25+0.5) ≈ 13.75s [推演，参数[证实]]。

### 3.2 Connect 响应字段来源（固件 `src/flashcmd.c:19-34`）

| 字段 | 来源 | 证据 |
|---|---|---|
| `protocol_version` | 编译期 `PROTO_VERSION = 0x000100100→0x00010100`（1.1.0） | `flashcmd.c:25`、`src/command.h:26` |
| `start_address` | `CONFIG_LAUNCH_APP_ADDRESS`（按芯片 Kconfig，如 stm32 默认 0x80020000/0x8008000…） | `flashcmd.c:26`、`src/stm32/Kconfig:509-516`（lpc176x Kconfig:104、rp2040 Kconfig:64 同名） |
| `block_size` | `CONFIG_BLOCK_SIZE`（stm32 默认 64） | `flashcmd.c:27`、`src/stm32/Kconfig:518-520` |
| `mcu_type` 字符串 | `CONFIG_MCU`（每芯片 Kconfig 定义，`DECL_CONSTANT_STR("MCU",…)` 导出） | `flashcmd.c:28`、`src/stm32/Kconfig:134`、`src/generic/armcm_canboot.c:16` |
| `software_version` 字符串 | `CONFIG_KATAPULT_VERSION` ← `make-version.sh:6-12`：`git describe --tags --long --always --dirty`，无 git 则读 `.version`，否则 `"?"` | `flashcmd.c:22,29-31`、`src/Kconfig:155` |

主机解析（`flash_can.py:273-313`）：前 12 字节 `struct.unpack("<4sII")` → version/start_addr/block_size（279-280）；**block_size 白名单 64/128/256/512，否则 FlashError**（284-285）；版本取 3 字节倒序 `(1,1,0)`（282-283）；`≥(1,1,0)` 时按 `\0` 拆 mcu_type 与软件版本（287-293）；若固件是 klipper.bin 且带 klipper_dict，**比对 MCU 名不一致 → FlashError**（305-313）。

### 3.3 UART/USB 的「发现」

- 无查询命令。主机靠 **USB VID/PID + manufacturer 字符串**判定设备当前状态（`flash_can.py:1049-1067`）。
- 请求进入 bootloader：
  - USB：DTR 置位 + 1200bps line coding 后等重连 ≤4s（`_request_usb_bootloader` 962-1011；`TIOCMBIS` 975、`B1200` 977、8×0.5s 983-991）。**固件侧 `usb_cdc.c:445-448 check_reboot()` 是空函数** [证实] → katapult 自身不处理 1200/DTR 请求，**[推测]** 该请求由运行中的应用（klipper）响应。
  - UART：发裸字符串 `b"~ \x1c Request Serial Bootloader!! ~"`（`flash_can.py:90,1013-1019`）。**`src/` 定向 grep 找不到该串的处理器** [证实]（`README.md:184-186` 称 "request Klipper's bootloader"）→ **[推测]** 同样由应用侧处理。
- 固件侧「留在 bootloader」的条件（与发现强相关）：`bootentry_check()` 满足任一才进 bootloader —— 内存签名 `REQUEST_CANBOOT`（应用主动跳）、**应用区 flash 为空**、按键、双击复位（10ms~500ms 窗口）—— `src/bootentry.c:55-68`（26-53），否则 `sched_main` 直接跳应用 —— `src/sched.c:55-59`。**进入后无超时自动退出**（主循环无 timer 跳转路径，只有 `complete` 触发）[证实，见 §4]。`README.md:84-85`：flash 空则自动进 bootloader。

---

## 4. Q3：主机发送节拍与重试策略

| 环节 | 行为 | 证据 |
|---|---|---|
| 块间间隔 | **成功路径无显式 sleep**，严格请求-响应串行（上一命令拿到 ack 才发下一条） | `flash_can.py:409-457`（`send_file` 循环） |
| Send Block 重试 | 每块 `tries=3, read_timeout=5.0`；回显地址不匹配 → `sleep(0.1)` 再试；3 次失败 → FlashError（含地址） | `flash_can.py:427-448`（sleep `.1` 在 443） |
| 末块填充 | 不足 block_size 用 `0xFF` 补齐；`flash_address += block_size` 递增 | `flash_can.py:423-426,449` |
| busy(0xF3) | `sleep(1.5)` 后重试（占一次 try） | `flash_can.py:383-385` |
| NACK(0xF1)/err(0xF2)/CRC/尾错 | 记日志 → 走统一重试：清缓冲 0.25s + `sleep(0.5)` | `flash_can.py:369-406` |
| 命令级失败 | tries 耗尽 → `FlashError("Error sending command …")` → 退出码 1 | `flash_can.py:407`、`1110-1113` |
| EOF(0x13) | ack 载荷 `page_count` 打印 | `flash_can.py:455-457`；固件 `src/flashcmd.c:106-118` |
| verify 流程 | `verify_file`：逐块 `REQUEST_BLOCK(0x14)`（tries=3、默认 2s 超时）、校验回显地址（不匹配 sleep 0.1 重试，3 次失败报错），对读回数据做 **SHA1 与本地固件 SHA1 对比**，不一致 → FlashError | `flash_can.py:459-491`（466-470、476、484-489） |
| Complete 后行为（主机） | `finish()` 发 0x15 并等 ack；**在 `finally` 中只要 `is_flash_req` 就总是尝试发送**（含刷写失败路径——脚本注释：出错也会退出 bootloader，除非通信已断）；随后打印 "Programming Complete" | `flash_can.py:492-493,845-850,1081-1085,1125` |
| Complete 后行为（固件） | ack 后置 `complete=1`，**100ms 定时器到点 → `application_jump()` → 置 `REQUEST_START_APP` + `NVIC_SystemReset()`**，重启后直接跳应用（不再走 bootentry 判定） | `src/flashcmd.c:45-57`（100ms 在 50）、`src/generic/armcm_canboot.c:69-73,113-116` |
| USB 双缓冲 prime | 对 stm32（除 f2/f4/h7）先发**非法命令 0x90** 的合法帧逼设备先回一帧，响应被丢弃 | `flash_can.py:265-271,346-351,1032-1046` |

**[推演]** 由于 bootloader 进入后无超时退出、`complete` 不校验传输是否完成（`flashcmd.c:45-57` 无条件延时跳转），失败刷写 + `finally` 里的 Complete 会让设备复位回（可能仍是旧的/空白的）应用区。

---

## 5. Q4：bootloader 运行期过滤与共存

### 5.1 CAN 层过滤（会忽略非本节点帧）

- **软件过滤（所有芯片生效）** `src/generic/canserial.c:226-249`：
  - `id == assigned_id` → 进命令字节流（229-236）；
  - `id == 0x3f0` 或 `assigned_id+1` → 进 admin 队列（238-241）；
  - **其他 id 一律忽略**（函数尾 `return 0`，不入任何队列）。
- **硬件过滤（stm32，`CONFIG_CANBUS_FILTER` 默认 `y if CANSERIAL`，`src/Kconfig:77-79`）** `src/stm32/can.c:136-169`：bank0=admin 0x3f0 **恒激活**（150,166 `(1<<0) | …`），bank1/2=id+1/id 仅当已分配时激活（152-155,166）；`CONFIG_CANBUS_FILTER` 关闭时 bank0 配成 mask=0（全收）+ 软件过滤兜底（157-159）。
- **rp2040：无硬件过滤**，注释 "Filter not implemented (and not necessary)" —— `src/rp2040/can.c:39-42`，依赖上面的软件过滤。
- **admin 命令白名单** `src/generic/canserial.c:194-209`：只处理 `0x00`(query)/`0x11`(set id)/`0x12`(clear id)。klipper 应用侧的 admin 命令 `0x01`(set-node)/`0x02`(reboot)（定义 `flash_can.py:73-75`）**被 bootloader 静默忽略** [证实]。
- **id 冲突自保**：听到 `assigned_id+1`（别人占用我方 TX id）→ 清自己的分配（`canserial.c:295-296,170-175`）；set-id 携带本 id 但 UUID 不符 → 清分配（`canserial.c:187-189`）。

### 5.2 字节流层过滤（UART/USB/CAN 数据通道）

- `command_find_block`（`src/command.c:100-152`）要求：头 `0x01 0x88`（107-108）→ 字长换算 msglen∈[8,128]（109-111）→ 尾 `0x99 0x03`（115-117）→ CRC16 一致（118-122），**全过才 dispatch**（156-164）。非 katapult 字节被丢弃到下一个 `0x01`（131-139）。**klipper 正常协议帧不会被误执行**（须先通过上述 4 重校验）[证实]。
- NACK 节流：仅在「初始 / 上一个有效帧之后」的第一个坏块回一次 NACK（`CF_NEED_VALID` 门控，`command.c:96,141-145`），连续垃圾不再重复 NACK [证实]。
- **[推演·兼容性关键风险]** 错误路径的重同步是 `memchr(buf, 0x01, buf_len)`（`command.c:134-137`）：当**缓冲区首字节恰为 `0x01` 而整帧不合法**（例如 klipper 应用帧、或头完好但 CRC 损坏的 katapult 帧）时，`memchr` 命中 `buf[0]` → `pop_count = 0` → 调用方弹出 0 字节（`serial_irq.c:78-85`、`usb_cdc.c:115-126`、`canserial.c:304-310`），缓冲永不前进：首轮回一次 NACK 后（`command.c:141-145`）进入**无消费死循环**，后续重试帧只会堆积到缓冲溢出（串口 192B `serial_irq.c:17`，CAN 192B `canserial.c:38`，USB 128B `usb_cdc.c:89`）→ 表现为「NACK 后全部超时」，**只能靠复位恢复**。静态追踪已逐分支核实，未动态复现。
  - 为何实践中少见：CAN 有硬件 CRC15、USB 有 CRC32，传输层几乎不产生损坏字节；UART 250000 低误码 + 主机只发合法帧。**外来协议帧（klipper 帧）是更可能的触发源。**

### 5.3 bootloader 与应用的共存

- **同一 MCU 上二者互斥，永不同时运行** [证实]：复位后 `bootentry_check` 决定去向（`bootentry.c:55-68`），跳应用 = 置签名 + `NVIC_SystemReset`（`armcm_canboot.c:69-73`），Complete 也走同一路径（`flashcmd.c:45-57`）。bootloader 主循环（`sched.c:55-69`）无任何与应用并发的机制。
- **同一总线上多设备**：按 CAN id 隔离（§5.1）。未分配的 bootloader 只应答 0x3f0 查询（`canserial.c:146-147`），已分配的只收自己的 id + admin；klipper 应用流量（其他 id）在硬件/软件过滤层被丢弃 [证实]。
- 主机脚本自身也避免混淆：查询只收 `resp[7]==0x11`（Katapult）的节点（`flash_can.py:714-726`）；刷写前先 `0x12` 清 id 保证已知状态（`flash_can.py:831-832`）。

---

## 6. Q5：与 klipper 应用协议帧的字节级可区分性

> klipper 应用帧布局（0x01 sync + seq + CRC16）**来自任务前提**，本侦察边界（只读 katapult 仓）内无 klipper 源可核对 → **[推测/前提]**；权威核对点应为 klipperx 仓 `third_party/klipper` 的 `src/msgproto.*` / 串口协议实现（边界外，建议后续补一次定向核对）。以下分析以「klipper 帧同样以 `0x01` 起始、第二字节不是常量 `0x88`」为前提。

**katapult 帧的判别特征** [证实]：`[0]=0x01`、`[1]=0x88`、`[3]=字长（≤30，因 MESSAGE_MAX=128）`、`[n-2..n-1]=0x99 0x03`、CRC16-CCITT(0xFFFF) 覆盖 `[2..n-6]` —— `src/command.h:37-50`、`src/command.c:100-127`、`protocol.md:9-24`。

1. **正向（bootloader 收 klipper 帧）**：
   - 第 1 关：字节 1 必须是 `0x88`。klipper 帧的第 2 字节是长度/序号类字段，仅当其值恰为 0x88 才可能过关 → **绝大多数 klipper 帧在 `command.c:107-108` 被拒**。
   - 若碰巧过关：还需 `buf[3]*4+8 ∈ [8,128]`（`buf[3]` 是 klipper 帧的 seq/crc 字节）、`buf[msglen-2..msglen-1]==99 03`、CRC16 相符（联合概率约 2^-16 × 小因子）→ **误收执行的可能性可忽略**（量化为 **[推演]**）。
   - **执行安全 [证实]：绝不会把 klipper 帧当命令执行**（dispatch 只在校验全过后，`command.c:156-164`）。
   - **但可用性受 §5.2 停摆推演影响 [推演]**：klipper 帧以 `0x01` 起始且整体不合法 → `pop_count=0` → 一次 NACK 后解析器原地停摆 → 之后主机（无论 klipper 还是 katapult）都收不到响应，直到复位。**这是兼容性评估的首要风险点。**
2. **反向（klipper 应用收 katapult 帧）**：katapult 帧 `[1]=0x88` 的第二字节是关键差异位（klipper 帧该位为长度字段）；klipper 侧是否会误收/停摆**超出本侦察边界**，未评估。
3. **CAN 层还有一重天然隔离** [证实]：数据通道按 id 分离（§5.1）；且 **admin 通道载荷是裸字节命令（`[0x00]`/`[0x11]+uuid`/`[0x12]`/`[0x02]+uuid`），完全不带 `01 88` 帧头** —— `flash_can.py:692-701`。即同一总线上 admin 载荷与两种「帧协议」在结构上就不同。
4. **响应侧**：katapult 响应 = `01 88 ack len payload crc 99 03`（`src/command.c:20-33`）；klipper 应用的响应帧布局未在边界内核实 [推测]。

---

## 7. 不确定点（汇总）

1. **klipper 应用帧字节布局**未在边界内核实（任务前提）；建议对 klipperx 仓 `third_party/klipper` 做一次定向核对（`src/msgproto.*` + 主机 `klippy` 串口编码）。
2. **§5.2 `pop_count=0` 停摆**是静态追踪结论，未做动态复现（建议：UART 注入 klipper 风格帧或故意坏 CRC 帧，观察是否「一次 NACK 后全部超时」）。
3. **0xF3 busy**：文档与主机有、`src/` 无发送方 —— 推测为保留/历史设计；如需确证需查全仓生成代码或固件实测。
4. **`SERIAL_BL_REQ` 字符串与 USB 1200bps/DTR 请求的处理方**：katapult 固件侧无 handler（`usb_cdc.c:445-448` 为空、src 无该串），**推测**由运行中的应用（klipper）响应；README:184-186 措辞含糊。
5. **bxCAN 在 FA1R 各配置下的接收语义**属硬件手册范畴，仓库内只能证实寄存器写值（`stm32/can.c:146-166`）；软件过滤路径确定不受影响。
6. stm32 USBFS 双缓冲需 prime 的机理依据脚本注释（`flash_can.py:1033-1046`），固件侧未逐寄存器核实。
7. `node_id=129→id 0x202` 的首节点取值是按 `len(nodes)=1` 静态推算 [证实数值推算，未运行]。

## 8. 停止条件声明

- 5 个规格问题均已逐条作答并附 file:line；「代码证实 / 静态推演 / 推测」已分类标注。
- 只读操作完成：ls + 点名文件精读 + 针对性符号 grep（含 `Request Serial`、`0xf3`、`CANBUS_FILTER`、`BLOCK_SIZE` 等精确字面验证）；**未做探索式全仓 grep，未改文件，未执行 git 写操作，未委派**。
- `flash_can.py` ≡ `flashtool.py` 已用 `diff -q` 核实。
- 边界外事项（klipper 源核对、动态复现）已列入不确定点，按停止条件不再扩展。
---

# 附录B（侦察B 原文）：klipperx 字节流/帧层相对上游的改动清单 + 主机侧 bootloader 交互落点

只读侦察（当前分支 work，未改任何文件、未执行 git 写操作、未再委派）。
标注约定：**[代码证实]** = 由本仓源码/文档直接读到；**[推测]** = 由已读证据推断、仓库内无直接代码可证。
katapult 帧格式（首字节 `0x01`、尾 `0x99 0x03`）为任务书/父级给定前提，仓库内无 katapult 源可核。

---

## 0. 收口专题（父级点名的两个问题）

### ① mux 帧头 vs katapult 帧：字节级交互全路径

mux 头判据（两侧同构）：**首字节高位=1、次字节高位=0**，再过 14 位汉明校验、arb 落在
`0x102..=0x2ff`（保留带 `≥0x3f2` 拒收）——`serial.rs:662-664`（`header_bytes` 强制 `0x80 | …`）、
`serial.rs:697-722`（`decode_header`，定界位检查 `:705`）；固件侧
`third_party/klipper/src/mux.c:101-113`（`mux_encode_header`）、`:125-137`（`mux_decode_header`）、
`:484-506`（`mux_unit_len`）、`:508-535`（`mux_input`）。

**katapult 帧（首字节 `0x01`，高位=0）在各解析路径上的处理：**

| 路径 | 判定点 | 行为 | 结果 |
|---|---|---|---|
| 主机共享串口 reader `dispatch` | `decode_header` 首字节 `0x01 & 0x80 == 0` → `HeaderScan::Invalid`（`serial.rs:705,712-715`） | `drop_to_sync`（`serial.rs:1557-1561` 调用点、`1600-1612` 实现）：找下一个 `0x7e`；**katapult 尾是 `0x99 0x03`、帧内无 `0x7e`** → `pending.clear()` + `needs_sync=true` | **整段当 junk 丢弃、不投任何槽位**；之后进入失步态，后续字节**丢到第一个 `0x7e`**（`serial.rs:1542-1554`） |
| 失步态的代价 | `needs_sync` 分支吃掉到下一个 `0x7e` 为止的所有字节 | 若后面紧跟一台在跑的 MCU 的合法单元，**该单元从头到尾（含其收尾 `0x7e`）被当垃圾吃掉一个**，再下一个单元恢复 [代码证实的行为 + 结论推断] | 每段 bootloader 字节流**至多损失其后第一个合法单元**，无状态污染、可自愈 |
| admin 扫描流 `UnitStream::take_unit_with_arb` | 同一 `decode_header`/`drop_to_sync`（`serial.rs:1086-1123`，`:1098-1107` Invalid 分支；文档 `:1074-1077`「Exactly the dispatch policy」） | 同上：`0x01` 开头 → Invalid → 丢到同步 | 扫描**不会**把 bootloader 字节当应答；最多本轮扫不到板子 |
| 被"桥 legacy 口加头"后的形态（bootloader 字节藏在合法头之后） | 头解析通过 → 读长度字节 = `0x01` → 不在 `MESSAGE_MIN..=MESSAGE_MAX(5..=64)`（`serial.rs:1571-1575`） | `drop_to_sync`：同样丢到下一个 `0x7e`；katapult 帧无 `0x7e` → 清空 + 失步 → **其后第一个合法单元再被吃掉一次** | 不误投；代价同上（1 个单元） |
| 固件设备半 `mux_input` | 首字节高位清 → `return 1`：「No header: an ordinary frame … The parser owns it」（`mux.c:513-516`） | 交给 klipper 帧解析器：长度字节 `0x01 < MESSAGE_MIN` → 进错误分支、丢到下一个 `0x7e`、不 dispatch、序号不动——与旧固件丢 KXID 同一条被测试钉住的性质（`serial.rs:3046-3130` 可执行移植测试；文档 `serial-mux-protocol.md:136-141`） | 固件侧把 katapult 帧当**非法长度的原始帧**整体丢弃，不会 dispatch |

**"误解析"的门槛（要骗过主机需要同时满足）**[代码证实的判据 + 概率推断]：
某个字节对要被当单元收下，必须 ①首字节≥0x80、②次字节<0x80、③汉明校验和=0（1/2 位错必拒，
`serial.rs:630-645`、测试 `:3321-3373`）、④arb 落在可路由区间、⑤长度字节 5..64、
⑥帧体 CRC-16 过（`push_frame` 后由槽位 `FrameStream::parse` 的 CRC 检查兜底，`serial.rs:1613-1628`）。
katapult 帧是「命令字节 + 地址/数据 + `0x99 0x03` + CRC」，随机凑齐 ①–⑥ 的概率可忽略
[推测：无 katapult 源，无法给出精确概率]。**结论：junk 丢弃是主行为，误解析被 6 道门拦住。**

### ② 向"bootloader 态设备"发帧走不走帧头

先厘清：**帧头是"主机↔桥/应用固件"的线上契约，bootloader（katapult）不实现它**——
mux 只存在于应用固件（`third_party/klipper/src/mux.c`，`MUX_SERIAL` 不默认开，
`serial-mux-protocol.md:26-35`）。逐条：

- **主机经共享串口的所有写出：一律带头，无裸写入口** [代码证实]
  - 数据帧：`SharedSerialHandle::send` 每次 `encode_header(arb_id(mcu_id)) + frame.raw_bytes()`
    （`serial.rs:1879-1886`）；
  - 握手/命令：`send_unit`（`serial.rs:1657-1671`）、`send_admin`（`:1677-1685`）——
    `SET_NODEID`/KXID/`SET_PORT_NODEID`/`RESTART_DEVICE`/`RESET_PORT`/`REQUEST_BOOTLOADER` 全部
    先 `encode_header(0x3f0)`（构造在 `serial.rs:884-887`，剥头重贴在 `:1700,1715,1728,1743`）。
  - 即：**假如有调用者对共享口上已进 bootloader 的设备发任何东西，字节都是 `[0x80..][0x7f..][len]…`
    开头**，bootloader 首字节就撞不上它的 `0x01` 命令字节 [推断，前提同上]。
- **独占串口：永远无头** [代码证实]——`SerialDevice::send` 直写裸帧（`serial.rs:183-198`）；
  配置层独占口不发指派帧（`config/mcu.rs:533-540`；`serial-mux-protocol.md:11-14`）。
- **host 库：mux 引用带 `encode_header(mux.arb)`，裸引用不带；admin 单元恒带头**
  （`host.rs:1376-1390` `Device::send`，`:1149-1161` `send_admin`，注释明言与帧路径的差别）。
- **CAN：线上没有字节头**，寻址靠 CAN 仲裁 id（`canserial.rs:340-358` `frames()` 只切片不加头）；
  但**端口级 admin 单元在节点字节流内部仍带 0x3f0 头**（`canserial.rs:717-740`、
  `serial-mux-protocol.md:316`）——那是发给**桥（应用固件）**的，bootloader 不在场。
- **桥 → 下游端口方向**：`MUX_PORT_UNITS` 口整单元带头转发、legacy 口**剥头**下发
  （`serial.rs:890-895` 标志语义；转发规则 `serial-mux-protocol.md:224`）。
  即：已进 bootloader 的下游设备若还能收到主机流量，legacy 口送的是**裸 klipper 帧**、
  UNITS 口送的是**带头单元**——两种都与 katapult 格式不相识，bootloader 侧只会拒收 [代码证实的
  转发语义 + 对 bootloader 行为的推断]。
- **主机自己根本不与 bootloader 对话**：`send_request_bootloader` 三处定义（`serial.rs:1741-1744`、
  `host.rs:1215-1218`、`canserial.rs:768-771`）**没有任何生产调用者**（对 `src/`+`crates/`+`docs/`+
  `scripts/` 穷举命中仅定义与测试 `serial.rs:2221-2225`、`host.rs:2613-2626`、`canserial.rs:1887-1891`），
  函数文档写明「Nothing in the restart path sends this」（`serial.rs:1736-1738`）。
- **外部刷写工具（不在本仓）**[推测]：直接裸写 tty/socket、不带 mux 头——所以共享口上：
  工具的裸字节会被主机 reader 按 ① 丢弃，工具也拿不到主机的头格式；本仓文档只把 `0x02` 称为
  「刷写工具的命令」（`serial-mux-protocol.md:173`），**没有**任何"主机替工具向 bootloader 发帧"的通路。

---

## 1. 帧层（frame.rs）与上游 klipper 线格式是否逐字节一致

### 1.1 逐字节相同的部分 [代码证实]

对照：`third_party/klipper/klippy/chelper/msgblock.h` + `msgblock.c`（仓内上游检出）。

| 项 | 本仓 | 上游 | 结论 |
|---|---|---|---|
| 常量（MIN=5/MAX=64/HEADER=2/TRAILER=3/POS_LEN=0/POS_SEQ=1/TRAILER_CRC=3/TRAILER_SYNC=1/SEQ_MASK=0x0f/DEST=0x10/SYNC=0x7e） | `src/core/klippy/frame.rs:17-47` | `msgblock.h:7-17` | 完全相同 |
| CRC-16-CCITT（初值 0xFFFF；上游 `data ^= data<<4` 在 uint8_t 下 ≡ 本仓 `data_byte ^= (data_byte & 0x0F) << 4`） | `frame.rs:61-78`（`crc16_ccitt!`） | `msgblock.c:20-33` | 逐位相同 |
| 收帧校验顺序与语义（长度范围 → DEST 位 → SYNC 定位 → CRC 覆盖 `[len,seq,payload]`） | `frame.rs:117-151`（`Frame::check`） | `msgblock.c:36-75`（`msgblock_check`） | 语义相同、顺序一致 |
| 出线编码 `[len][seq\|0x10][payload][crc_hi][crc_lo][0x7e]` | `frame.rs:226-240`（`encode_wire` 正常路径） | 固件 `command_add_frame` 同构 | 逐字节相同 |
| 错误恢复（丢到下一个 `0x7e`；无 SYNC 整段丢弃并失步） | `frame.rs:259-345` | `msgblock.c:56-74` | 策略相同 |

**独占串口线格式 = 上游逐字节相同 [代码证实]**：`SerialDevice::send` 直写 `frame.raw_bytes()` 无前缀
（`serial.rs:183-198`）；独占口不发指派帧（`config/mcu.rs:533-540`）；文档
（`serial-mux-protocol.md:11-14`「独占……线格式与 Klipper 逐字节相同」）。

### 1.2 差异点（全清单）[代码证实]

1. **`wire_seq` 旁路帧**（帧层唯一自增语义）：`frame.rs:176-190`（字段）、`:190-197`（`with_wire_seq`）、
   `:209-214`（`raw_bytes` 分派）、`:216-232`（`encode` vs `encode_wire`）。允许把不在 `0x10..=0x1f`
   的 seq 字节（实际只用 `0x20`）原样送出；上游固件产不出，本仓 `Frame::check` 自己也拒收
   （`frame.rs:126-130`，测试 `:446-460`）。唯一生产者 `serial.rs:845-849`（`kxid_unit`）。
2. **共享/桥路径每帧前加 2 字节校验帧头**（mux 包裹层）：`send_unit` 写锁下整段写
   （`serial.rs:1657-1671`）；`header_bytes` 首字节 `0x80 | …`、次字节高位恒 0（`serial.rs:662-664`）；
   10 数据位 + 4 校验位缩短汉明码（只检不纠，`serial.rs:589-636`）。**帧体 CRC 不覆盖帧头**，
   故另加这 4 位（`upstream-deviations.md:102`）。
3. **KXID 伪帧/命令单元**：`[len=9][0x20]["KXID"][crc][0x7e]`（`serial.rs:552-569,724-729,845-849`），
   命令帧 = 载荷尾追命令字节（`serial.rs:837-886`）。
4. **admin/寻址信道进串口线**：`0x3f0`/`0x3f1` 与 `0x100+2*id`/`+1`（`serial.rs:805-832`、
   `assign.rs:42-70`）——上游串口线内没有（`upstream-deviations.md:102-103`）。
5. **收端按头分派**：单 reader 切单元投槽（`serial.rs:1540-1596` `dispatch`、`:1600+` `drop_to_sync`）；
   host 库同构（`host.rs:1149-1161`、`:1376-1390`）。

明确回答「头/尾/CRC/seq」：**头（len+seq）、尾（crc+sync）、CRC 算式与覆盖范围、seq 低 4 位 + DEST 位
语义、帧长上下限——均与上游逐字节一致**；差异只有 (a) seq 可被旁路成 `0x20`，(b) 共享/桥路径在帧前多
2 字节头，(c) 载荷层多出 KXID 伪帧与命令字节。

---

## 2. 主机侧进入 bootloader 交互的场景与代码路径

### 2.1 restart.rs 的 restart_method 分支 [代码证实]

`reset_firmware`（`restart.rs:74-111`）：`Command` → 无前置动作（之后 `mcu/config.rs:612,734` 的
`config_reset`）；`Admin` → 本传输无动作（`:84-87`），真正动作是 `reset_bridge_firmware`
（`restart.rs:141-176`）发 `send_set_port_node_id + send_restart_device`（`device_reset`
`restart.rs:188-200`；`Port` 变体 `:156` 发 `send_reset_port`）；`Arduino` → DTR@2400
（`:338-357`）；`Cheetah` → RTS/DTR 舞步@2400（`:359-385`，注释 `:354-356`：**解除**板上 stateful
bootloader 电路的武装）；`RpiUsb` → 断复电（`:392+`）。调用场景：firmware_restart 后、开传输前
（`mcu/object.rs:1536-1566`）。关联：`McuConfig::open` 对 Cheetah 整条连接撤销 RTS，否则板子掉进
bootloader（`config/mcu.rs:543-548`；`serial.rs:287-290`）——**都是线级避/触 bootloader，不是协议对话**。

### 2.2 REQUEST_BOOTLOADER 的发送者与接收者 [代码证实]

- 定义与值：`serial.rs:812-817`（`CMD_REQUEST_BOOTLOADER = 0x02`，注释点明「CAN 同值同义，且是**刷写
  工具**的命令」）；发送函数 `serial.rs:1741-1744`、`host.rs:1215-1218`、`canserial.rs:768-771`。
- **无生产调用者**（穷举见 §0②）。函数文档「Nothing in the restart path sends this」
  （`serial.rs:1736-1738`、`host.rs:1211-1213`）。
- 路径：**主机 → 桥（admin 通道，头 `0x3f0` 的 KXID 单元）→ 桥按 `index` 转发到该端口**；`index`
  指不到本机在用端口时桥不消费、穿透给本机设备半（设备半 `bootloader_request()`，`CONFIG_HAVE_BOOTLOADER_REQUEST`
  守卫）——`serial-mux-protocol.md:173,224,253`；固件 `third_party/klipper/src/mux.c:347-350,413-414`、
  `src/bridge.c:456`、`src/host/host.c:222`。
- CAN 上两条通道分离：端口级命令走**节点自己仲裁 id 的字节流**（`canserial.rs:717-740`、
  `serial-mux-protocol.md:316`）；单帧 CAN admin 的 `0x02`（上游 `third_party/klipper/src/generic/canserial.c:236,305-310`）
  按 **UUID** 校验后 `bootloader_request()`。

### 2.3 主机自己是否实现 katapult 协议 [代码证实：没有]

`src/`+`crates/` 检索 `katapult`/`0x88`：无 bootloader 协议实现（`0x88` 命中均为显示/编码无关处）。
`0x11` 在本仓唯一协议身份是发现应答的 application 字节 `CMD_SET_CANBOOT_NODEID`
（`serial.rs:833-835`、`canserial.rs:105`），仅用于把应答命名为 "CanBoot"
（`serial.rs:935-941`、`canserial.rs:452-460`），不是主机发起的命令。
**结论：主机完全不与 bootloader 对话**——只有「让设备去 bootloader」的 API（`0x02`，无调用者）和线级复位；
无 UUID 查询/读 flash/擦写/跳转任何 katapult 命令，刷写须由外部工具完成。

---

## 3. CAN 路径：与 bootloader 是否共用 arb id / 节点 id 是否变化

[代码证实]
- 单一映射 `assign.rs:42-56`（`0x100+2*id` / `+1`，域 `1..=0xff`）；CAN 链路直接用
  （`canserial.rs:316-344`），socket 过滤器只收 `rx_id`（`canserial.rs:630,1171`），软件 `accept`
  再拒非 `rx_id`（`canserial.rs:364-371`）。
- 节点号由主机在开链路时经 `0x3f0` 单帧下发，「The node id lives in the host, not in the MCU」
  （`canserial.rs:585-593`），payload `[0x01][uuid][nodeid]`（`:292-298`）；全仓无第二处指派。
- 进 bootloader 不改 id：`send_request_bootloader` 只把 `admin_unit([0x02, index])` 塞进字节流
  （`canserial.rs:768-771`）；固件 `can_process_request_bootloader` 只核 UUID 后调
  `bootloader_request()`（`canserial.c:305-310`），不改 `assigned_id`（改 id 的只有
  `can_process_set_klipper_nodeid`，`canserial.c:283-296`）。
- bootloader 可在总线上被发现并自报：应答 id `0x3f1`、application 字节 `0x11` = CanBoot
  （`canserial.rs:399-460`）。

[推测] 共用性：bootloader 与应用**共用同一 arb id 号段**（信道逐字镜像 CAN 上游协议）；进 bootloader 后
**同一数值 node id 是否保持**无法在仓库内证实——katapult 源不在仓内，上游 `docs/CANBUS_protocol.md`
grep 无 bootloader 条目；主机侧没有任何重发 `SET_NODEID` 的代码路径（仅 `open_with_wire` 开链路时发一次），
重新拿号只能来自刷写工具/主机再次 `SET_NODEID`（若 bootloader 实现它）。主机在 bootloader 存活期间重连：
照常发 `SET_NODEID` + identify → identify 必超时，属**连接失败而非总线冲突**（过滤器仍只认 `rx_id`）。

---

## 4. 串口路径：mux 头 vs katapult 头（字节级）

详见 §0①。要点（带行码）：
- mux 首字节恒 `0x80..=0xff`（`serial.rs:662-664`），解码强制定界位（`serial.rs:705`）；文档
  `serial-mux-protocol.md:79`。
- katapult 首字节 `0x01`（高位 0）→ `HeaderScan::Invalid` → `drop_to_sync`：katapult 帧内无 `0x7e`
  → 清空 + `needs_sync`（`serial.rs:1600-1612`）→ 后续丢到第一个 `0x7e`（`:1542-1554`），
  **至多再吃掉后面第一个合法单元**，不投槽、不改序号。
- 头两字节可含 `0x7e`（id∈120..=127）的边界已被测试钉住（`serial.rs:3280-3318`、
  `serial-mux-protocol.md:138-140`）。
- 固件半 `mux_input`：高位清的首字节当**原始帧**交给帧解析器（`mux.c:513-516`），katapult 的 `0x01`
  长度非法 → 丢到下一个 `0x7e`、不 dispatch（与旧固件丢 KXID 同性质，测试 `serial.rs:3046-3130`）。
- 独占口无头：bootloader 与主机格式互不相识，但主机无 bootloader 读写路径（§2.3），不会对话。

---

## 5. identify 握手 / mux 握手（KXID 伪帧）在 bootloader 存在时的误触发风险

[代码证实]
- **时序**：共享口 attach 先做指派握手（`SET_NODEID` 点名 + KXID 重确认，500ms×3，`serial.rs:536-542`、
  `attach :1269+`、`run_handshake :1779-1830`），**之后**才 identify（`serial-mux-protocol.md:125-141`）；
  独占口不发指派帧（`config/mcu.rs:533-540`）。
- **KXID 对 klipper 式解析器必然无害**：seq `0x20` 清掉 DEST → `Frame::check` 拒收（`frame.rs:126-130`）；
  旧固件在**第一字节**（头首字节 `0x80..`，非合法长度）即进错误分支、整单元丢弃、序号不动——
  command.c 可执行移植测试钉住（`serial.rs:3046-3130` 三条：整单元丢弃 / 请求夹在 identify 前 /
  identify 前后各一次），文档 `serial-mux-protocol.md:136-141`、`upstream-deviations.md:102`。
- **bootloader 存在时**：主机不为 bootloader 发任何东西（§2.3）；共享线上 identify/KXID/命令单元都带头
  （首字节 `0x80..` ≠ `0x01`）→ 推断 bootloader 拒收/忽略，**无"被当 katapult 命令误触发"的字节通路**
  [推断，bootloader 源不在仓内]。独占口出线是裸 klipper 帧（首字节=长度 5..64）——bootloader 若把某字节
  读作命令，主机无法证伪 [无法证实，列为风险]。
- **真正发生的是超时而非误触发** [代码证实]：板子在 bootloader 时握手收不到 `+1` 头回执 → 3 次后
  `InterfaceError::Timeout`、`connect` 失败（`serial.rs:1269-1305,536-542`）；文档确认误配到不支持复用的
  固件「握手超时、连接失败，不会破坏已有会话」（`serial-mux-protocol.md:138-141`），且今天的微控制器
  串口固件默认不编 mux、根本不回执（`serial-mux-protocol.md:26-35`）。
- 反向（bootloader 回的数据被主机误登记）：扫描收端 `UnitStream` 只出「头 + 整帧」，`UnassignedScan`
  再核 CRC/形状（`serial.rs:947-1004,1023-1041`；不合形直接丢，测试 `:3749-3762`）→ 最多"找不到板子"。
  数据侧 `push_frame` 后还有槽位 `FrameStream::parse` 的 CRC 兜底（`serial.rs:1613-1628`）。

---

## 不确定点（仓库内无法证实）

1. katapult 确切字节格式（`0x01` 头、`0x99 0x03` 尾、命令集）：源不在本仓，§0/§4/§5 依赖给定前提。
2. bootloader 收到 mux 头/identify/KXID 时的具体行为（拒收 vs 计数 vs 复位）：只能由首字节不相交推断。
3. CAN 进 bootloader 后同一数值 node id 是否保持：无代码路径、上游协议文档未覆盖。
4. `serial-mux-protocol.md:26-35`（串口固件不回执、共享必超时）与 `:336-341`（宿主库桥真机端到端成功）
   分属不同载体，边界（哪些载体已回执）未逐一核实。
5. "至多再吃掉后面第一个合法单元"的代价是失步策略的确定行为（`serial.rs:1542-1561`）推出的**确定性
   结论**，但未跑实验验证（只读约束）。

## 停止条件声明

按任务书只读点名文件 + 回答 5 问题与父级追加两问所必需的窄范围定位检索（`send_request_bootloader`
调用方穷举、上游 `msgblock.c/h`、`canserial.c`、`mux.c` 对照、`katapult|0x88` 穷举、文档行号核对）；
未扩展到其他文件，未改任何文件，未执行 git 写操作，未向下委派，未使用任何上下文压缩工具。
---

# 附录C（侦察C 原文）：上游 klipper × katapult 集成基线（对照组）

范围：只读 `third_party/klipper/` 子模组检出（唯一合法上游参照）。所有 file:line 均相对该检出根。
标注约定：**[代码]** = 直接读到代码/文档原文；**[推测]** = 由多处证据推断、未见显式声明。

---

## Q1. 上游主机何时与 bootloader 对话？

**结论 [代码]：klippy 运行时从不与 bootloader 对话；与 bootloader 的交互只发生在独立的 flash/查询脚本里（flashtool.py、canbus_query.py、flash_usb.py）。**

- **启动/连接时：只跟"应用"说话，不查 bootloader。**
  - `klippy/mcu.py:857-864` `_attach()`：CAN 走 `connect_canbus(uuid, nodeid, iface)`，UART 走 `connect_uart`，pipe 走 `connect_pipe`；没有任何 bootloader 分支。
  - `klippy/serialhdl.py:119-167` `connect_canbus()`：开 socketcan → 发 admin `SET_NODEID`(0x01)（`serialhdl.py:131-136`）→ `_start_session()` 做 **identify 数据字典抓取**（`serialhdl.py:73` 的 `identify offset=... count=40`）→ 用 `get_canbus_id` 校验 uuid↔nodeid 映射（`serialhdl.py:156-161`）。整条链只说 klipper MCU 协议。
  - 连接失败是**无限重试到 90 秒上限**（`serialhdl.py:140`、`172`、`190`），identify 单轮 5 秒超时即 disconnect 再来（`serialhdl.py:96-103`）。**不会**降级去尝试 bootloader 协议 [代码+推断：全 klippy 树无 bootloader 分支，见 Q2]。
- **restart_method 路径：全部是"让应用复位"，没有一条是"进 bootloader"。**
  - `klippy/mcu.py:666` 选项表 `[None, 'arduino', 'cheetah', 'command', 'rpi_usb']`。
  - `mcu.py:730-747` `_restart_via_command` 发 **应用命令** `reset`/`config_reset`；`mcu.py:720-728` `_restart_arduino`/`_restart_cheetah` 只做串口 DTR/RTS 电平序列（`serialhdl.py:366-403` 的 `arduino_reset`/`cheetah_reset`）；`mcu.py:748-752` `_restart_rpi_usb` 断 USB 供电。
  - `mcu.py:703-705` 注释：cheetah 需要保持 RTS 否则 reset 会触发**板载（硬件）bootloader**——即上游把"进 bootloader"视为副作用而非目标。
- **真正进 bootloader 的入口（都在 klippy 之外/固件侧）：**
  - 文档 `docs/Bootloader_Entry.md:6-97` 列三种方式：USB-ACM 1200baud+DTR（`:8-11`）、物理串口发 `~ \x1c Request Serial Bootloader!! ~`（`:43-57`）、CAN admin 消息（`:75-83`，明确"即使设备已有 nodeid、即使 MCU 已 shutdown 也处理"）。
  - CAN 侧用 flashtool：`docs/Bootloader_Entry.md:85-90` 给出 `flashtool.py -i <iface> -u <uuid> -r`。
  - 固件实现：`src/generic/serial_irq.c:128-130`（串口 magic 串 → `bootloader_request()`）、`src/generic/usb_cdc.c:522-526`（1200 baud → `bootloader_request()`）、`src/generic/canserial.c:236,305-310,325-326`（admin `CANBUS_CMD_REQUEST_BOOTLOADER 0x02` + uuid 校验 → `bootloader_request()`）、`src/generic/armcm_reset.c:13-52`（`try_request_canboot()` 用 `CANBOOT_REQUEST/BYPASS` 签名让 katapult 保持/退出，`CANBOOT_SIGNATURE 0x21746f6f426e6143`）。
  - `scripts/flash_usb.py:13-20` `enter_bootloader()`（1200baud+DTR），且 `flash_usb.py:100-105` 直接 **spawn `lib/katapult/flashtool.py`** 完成刷写——脚本层的组合是上游唯一"主机编排 bootloader 会话"的地方。
- **结论口径**：启动查询=否；仅 flash/查询工具=是；restart_method 路径=全部复位应用，不进 bootloader（进 bootloader 由固件侧请求处理 + 外部工具发起）。

## Q2. 上游主机自己是否实现 katapult 帧协议（0x11 Connect 那套）？

**结论 [代码]：klippy 运行时（klippy/ 树）完全不实现、不发送 katapult 帧；katapult 帧协议只存在于 `lib/katapult/flashtool.py` 这个独立脚本。**

- katapult 帧定义只在 `lib/katapult/flashtool.py:56-66`：
  `CMD_HEADER = b'\x01\x88'`、`CMD_TRAILER = b'\x99\x03'`、`BOOTLOADER_CMDS = {'CONNECT':0x11,'SEND_BLOCK':0x12,'SEND_EOF':0x13,'REQUEST_BLOCK':0x14,'COMPLETE':0x15,'GET_CANBUS_ID':0x16}`，ACK/NACK 常量 `:67-70`。
- 帧组装/校验：`flashtool.py:253-263` `_build_command()`（header + cmd + word_cnt + payload + crc16(从 cmd 起) + trailer）；`flashtool.py:273-316` `connect_btl()` 解析 CONNECT 应答（4s 版本 + 起始地址 + block size + MCU 信息）；`flashtool.py:327-406` `send_command()` 重试/校验；`flashtool.py:42-49` crc16-ccitt（注明抄自 klipper msgproto.py）。
- klippy 侧反证 [代码]：
  - 全 klippy 树 grep `bootloader` 只有 2 处**注释**：`klippy/mcu.py:704`、`klippy/serialhdl.py:367`。
  - grep `0x11|0x99|0x88|CMD_HEADER` 在 `klippy/` 无任何帧协议命中（命中的都是无关驱动常量，如 `ads131m0x.py:16`、`tmc2130.py:13`）。
  - `klippy/` 中 admin id `0x3f0` 只出现一次：`serialhdl.py:131`，用途是**连接时 SET_NODEID(0x01)**，不是 bootloader 命令。
- 唯一的"klipper 知道 katapult"痕迹在**文档与脚本**：`docs/Bootloader_Entry.md:100-112`（klipper 收到进 bootloader 请求后"让 Katapult 保持激活再复位"，由固件 `armcm_reset.c` 完成），以及 `scripts/flash_usb.py:100` 调 flashtool。
- **推测 [推测]**：klippy 若面对停在 katapult 的设备，只会 identify 超时→disconnect→重试 90s 后报 `Unable to connect`（`serialhdl.py:96-103,140`），不会识别"这是 bootloader"。

## Q3. CAN 地址约定（node id / 仲裁 id）

**结论 [代码]：应用与 bootloader 共用同一 6 字节 `canbus_uuid` 和同一套 id 公式，但 node id 分区不同（应用 ≥4 由 klippy 分配；bootloader ≥128 由 flashtool 分配），互不感知。**

- 公式（应用数据流）：`docs/CANBUS_protocol.md:13-17` — host→mcu `canbus_nodeid*2+256`，mcu→host `canbus_nodeid*2+256+1`。代码同式：`klippy/serialhdl.py:121` `txid = canbus_nodeid * 2 + 256`，filter `txid+1`（`:122`）。
- admin 仲裁 id：host→mcu `0x3f0`，mcu→host `0x3f1`（`docs/CANBUS_protocol.md:26-29`；`serialhdl.py:131`；`scripts/canbus_query.py:8`；`flashtool.py:73,78`）。`0x3f0` 是广播地址（`CANBUS_protocol.md:29-31`）。
- 查询 bootloader（未分配节点）用的仲裁 id = **`0x3f0` 发 `0x00`，在 `0x3f1` 收 `0x20` RESP_NEED_NODEID**：
  - `scripts/canbus_query.py:8-24`（`CANBUS_ID_ADMIN=0x3f0`、`CMD_QUERY_UNASSIGNED=0x00`、发包 `:23-24`），应答 `:30-44`，**第 8 字节是 app_id**：`0x01`=Klipper、`0x11`=CanBoot/Katapult（`canbus_query.py:13-14,37-44`）。默认 app_id=Klipper（`canbus_query.py:42`）。
  - 同逻辑在 `flashtool.py:78-84,697-727`（`CANBUS_RESP_NEED_NODEID=0x20`，只把 `app == "Katapult"` 的 uuid 收进列表 `:724-726`）。
- 应用 node id 分配：`klippy/extras/canbus_ids.py:7,14-19` `NODEID_FIRST = 4`，按配置顺序 `len(ids)+4`；host 连接时用 `0x3f0` + `CMD_SET_KLIPPER_NODEID 0x01` 下发（`serialhdl.py:133-136`；消息格式 `CANBUS_protocol.md:41-47`）。
- bootloader node id 分配：`flashtool.py:84` `CANBUS_NODEID_OFFSET = 128`，`flashtool.py:734-746` `_set_node_id()` 用 `CANBUS_CMD_SET_NODEID = 0x11`（`flashtool.py:81`）下发 `node_id = len(self.nodes)+128`，实际仲裁 id `node_id*2+0x100`。
- **同 uuid、不同 node id 分区 [代码]**：进 bootloader 前 flashtool 先广播 `CANBUS_CMD_CLEAR_NODE_ID 0x12` 清掉**所有 bootloader node id**（`flashtool.py:82,729-732,831`），再对目标 uuid 重新分配 128+ 的 id；应用侧 id（≥4）由 klippy 自己的 `canbus_ids` 表管理，两条路径只通过 uuid 关联。
- 固件侧 uuid 校验：`src/generic/canserial.c:241-247,294,307` `can_check_uuid()` —— 0x01/0x02 admin 命令都要求带正确 6 字节 uuid。
- **[推测]** 上游"bootloader 与应用共用 node id"并不成立——它们共用的是 uuid 与公式，node id 显式错开（4+ vs 128+），避免与已分配的应用 id 撞号。

## Q4. 「应用帧 vs bootloader 帧」共存时，主机会话怎么处理

**结论 [代码]：上游假设同一时刻只有一个会话存在；进 bootloader 前先彻底断开，bootloader 会话由独立工具新起，回来后 klippy 走全新 identify 会话。无跨会话的序列号/状态延续。**

- **断开 [代码]**：`klippy/mcu.py:754-764` `_firmware_restart()` 先按 restart_method 复位，复位路径里一律先 `self._disconnect()`（`:722,727,738,748`）→ `serialhdl.py:217-231` `disconnect()` 释放 serialqueue、join 后台线程、关串口/socket、清 pending notifications。
- **新会话 = 全新序列/字典状态 [代码]**：`serialhdl.py:86-118` `_start_session()` 每次重新 `serialqueue_alloc`、重新跑 identify（5s 超时，`:96-103`）、重新 `msgparser.process_identify`。klipper 帧的 4-bit seq 是**每块字段、仅会话内校验**（`docs/Protocol.md:169-172`、`klippy/msgproto.py:18-24,249-259`），无持久状态 → 重连不产生"序列号错位"问题 [代码+推断]。
- **CAN 特有 [代码]**：bootloader 会话前 `flashtool.py:831` `_reset_nodes()` 广播清 id；bootloader 数据仍走"无状态字节流"假设（`CANBUS_protocol.md:56-64`：CAN 包与 mcu message block 不要求对齐）。CAN 的 0x3f0 广播在 bootloader 里同样被处理（`Bootloader_Entry.md:77-83` 明确"即使已分配 nodeid、即使 MCU shutdown 也处理"），因此**进 bootloader 的 admin 消息可以在应用会话存在时被 MCU 处理**——这是上游唯一显式的"共存"场景。
- **USB 重枚举 [代码]**：katapult 与 klipper 是不同 USB id：`flashtool.py:87-88` `KATAPULT_USB_ID="1d50:6177"` / `KLIPPER_USB_ID="1d50:614e"`（另有 GS-CAN `1d50:606f` `:89`）。flashtool 等待设备重枚举后再开串口：`flashtool.py:962-996` `_request_usb_bootloader()`、`flashtool.py:786-812` `_wait_canbridge_reset()`；串口占用检测防止与 klippy 抢口 `flashtool.py:933-961` `validate_device()`。
- **进入 bootloader 的串口前置动作**：物理串口发 `~ \x1c Request Serial Bootloader!! ~`（`flashtool.py:90,1016`；固件识别 `src/generic/serial_irq.c:128-130`；文档 `Bootloader_Entry.md:43-57`，注明该串不是合法 MCU 协议消息但仍受 sync 字符约束）。
- **[推测]** 上游没有"同时与 bootloader 和应用通信"的设计；对同一端口的并发访问仅被 `validate_device()` 事后检测并报错退出。

## Q5. klipper 正常 MCU 协议帧线格式（用于与 katapult 对照）

**[代码/文档] 均来自 `docs/Protocol.md:157-207` + `klippy/msgproto.py`：**

| 项 | klipper 应用帧 | katapult 帧（对照） |
|---|---|---|
| 结构 | `<1B length><1B seq><nB content><2B crc><1B sync>`（`Protocol.md:161-165`） | `01 88` header + `<1B cmd><1B word_cnt><payload><2B crc>` + `99 03` trailer（`flashtool.py:56-57,253-263`） |
| sync | 尾部 sync 字节 **0x7e**，块首可额外加一个 sync；sync 不排他（`Protocol.md:176-183`；`msgproto.py:24 MESSAGE_SYNC=0x7e`） | 无 0x7e；用固定头 `01 88` / 尾 `99 03` 定界，接收侧靠 `readuntil(b"\x03")` + 头校验滑窗对齐（`flashtool.py:336-347`） |
| 长度 | length 字节 = **整块**字节数，含头尾，最小 5、最大 64（`Protocol.md:165-168`） | `word_cnt = len(payload)//4`（`flashtool.py:254`），接收按 `recd_len = data[3]*4` 反推总长（`flashtool.py:342-345`） |
| seq | 低 4 位序号，高 4 位恒 `0x10`（`Protocol.md:169-172`；`msgproto.py:22-23 MESSAGE_DEST=0x10`，`msgproto.py:292-297`） | **无 seq 字段** |
| CRC | 16-bit CCITT，覆盖**含头不含尾**（`Protocol.md:173-176`；`msgproto.py:29-35,296`） | 16-bit CCITT，`crc16_ccitt(out_cmd[2:])` = **从 cmd 字节起、不含 2B header、不含 trailer**（`flashtool.py:42-49,260`） |
| content | 一串 VLQ 编码的 `<cmd-id><params...>`（`Protocol.md:189-207`） | 单个 1B cmd + 4B 对齐 payload（CONNECT/SEND_BLOCK/...，`flashtool.py:58-66`） |
| 应答语义 | 命令/响应消息 + ack/notify 块（`Protocol.md:308-350`） | `0xa0` ACK / `0xf1` NACK / `0xf2` ERROR / `0xf3` BUSY 在响应块第 3 字节（`flashtool.py:67-70,367,381-388`） |
| CAN 承载 | message block 跨 8 字节 CAN 包字节流拼接，不需对齐（`CANBUS_protocol.md:56-64`；CAN 包格式 `flashtool.py:52 CAN_FMT="<IB3x8s"`） | 同样把 katapult 帧按 8 字节切片走 node 仲裁 id（`flashtool.py:662-677 send()`，≤8B/包） |

**风险提示 [代码]**：两种帧的 CRC 算法相同但**覆盖范围与帧定界完全不同**，且 katapult 帧的 header `0x01 0x88` 中 `0x88` 恰好可被误当作 klipper seq（`0x88` 高 4 位是 0x8，不满足 `0x10` 校验 → `msgproto.py:250` 会判非法），反之 klipper 的 `length` 字节值域 5..64 不会撞 katapult header 第一字节 0x01。这是本仓做兼容性评估的核心对照数据。

---

## 不确定点（未在上游找到证据的）

1. [未证] 上游是否存在任何"识别到对端是 bootloader 就改走 flashtool"的代码路径——**没有找到**；仅有文档建议手动跑 flashtool（`Bootloader_Entry.md:85-90`、`CANBUS.md:61,73`）。klippy 侧 connect 超时行为（90s 后 `Unable to connect`）是从 `serialhdl.py:140` 推断的终局，未实测。
2. [未证] CANBridge 模式下进 bootloader 后 usb-can 桥的 USB id 变化细节：flashtool 用 `GS_CAN_USB_ID 1d50:606f` 匹配 + manufacturer `klipper`→`katapult` 判断（`flashtool.py:747-785,786-812`），但桥固件侧枚举切换代码未读（属 `src/generic/usb_canbus.c`，不在点名清单内）。
3. [未证] RESP_NEED_NODEID 第 8 字节 app_id 由**固件**（klipper 应用侧）还是 katapult 填充：klipper 侧格式 `CANBUS_protocol.md:52` 显示应用回 `0x01`，katapult 回 `0x11`（`canbus_query.py:37-44` 的映射）；**bootloader 固件源码不在本仓 `third_party/klipper` 内**（katapult 是独立仓库，本检出只有 `lib/katapult/flashtool.py`）。
4. [未证] `docs/CANBUS.md` 全文与 `CANBUS_Troubleshooting.md` 中是否还有 bootloader 相关约定（只做了 bootloader 关键词定向 grep，命中 `CANBUS.md:13,61,73,96`）。
5. klippy/extras/canbus.py **不存在**（点名文件假设不成立）；CAN 逻辑实际在 `klippy/serialhdl.py:119-167` + `klippy/extras/canbus_ids.py` + `src/generic/canserial.c`。

## 停止条件声明

- 已完成：5 个规格问题逐条作答并附 file:line；点名文件全部读到或确认不存在（`klippy/extras/canbus.py` 不存在）。
- 停止原因：单元目标（上游集成基线）已闭合，继续深挖（`usb_canbus.c`、katapult 独立仓库固件、CANBUS.md 全文）超出本单元边界且下游暂无消费方。
- 本产出为只读侦察：未修改任何文件、未执行任何 git 写操作、未向下委派、未使用压缩工具。
