# CAN ISO-TP 动工前调查与实施规划

写作当天：2026-10-16。**规格未知已侦察完毕**，本文是设计定稿 + 阶段拆分 + 文件级落点。
末尾「实施记录」是唯一随实现推进更新的回填节。

---

## 1. 范围定稿

给 klipperx 新增**一种与 `CanSerial` 并存的 `[mcu]` 传输**：CAN 层走 ISO 15765-2（ISO-TP）
分段/流控，上面仍然是现有的 Klipper 字节流协议（`length/seq/payload/crc16/sync`，
`MESSAGE_MIN=5`／`MESSAGE_MAX=64`，`src/core/klippy/frame.rs:19-24`）。**不替换**
`canserial` 的自定义 8 字节切片；同一块板子按配置二选一。

已拍板（用户 2026-10-16）：

| # | 决策 | 取值 |
|---|---|---|
| D1 | 主机侧 ISO-TP 谁实现 | **自研用户态状态机**（不用内核 `can-isotp`、不引第三方 crate）。理由：本仓测试环境无 CAN 外设，纯状态机零 IO 才能做逐字节单测（`AdminBus`/`test_wire` 范式）；内核路只作阶段 3 的**参照对端** |
| D2 | 配置键形状 | CAN 家族内加协议选择键 **`canbus_protocol: cans \| isotp`**（默认 `cans`，现状零变）；`uuid`/`canbus_uuid`/`canbus_interface`/`canbus_nodeid`/`[mcu_ids]` 全部沿用 |
| D3 | ISO-TP 参数 | 11 位正常寻址；`tx = 0x100 + 2*id`、`rx = tx + 1`；**BS=0**、STmin=0；**发送不 padding，DLC = 实际使用字节数**；接收容忍任意 ≥ 所需长度的 DLC |
| D4 | 消息粒度 | **一条 Klipper 帧 = 一条 ISO-TP 消息**（消息导向，不做字节流式） |
| D5 | 阶段 2.5 | **做**：给 `linux` 目标加 SocketCAN，让真固件能在 `vcan` 上当从站跑端到端 |
| D6 | CAN-FD 在配置里怎么摆 | **同一 CAN 家族、两个正交轴**：`canbus_protocol: cans\|isotp`（传输层）× `canbus_fd: true`（链路层），不是互斥枚举（详见 §8） |
| D7 | 状态机是否参数化 | **是**：ISO-TP 状态机从一开始就把「每帧数据上限」当参数（classical SF=7/CF=7，CAN-FD SF=62/CF=63），不把 7 硬编码——这样 CAN-FD 落地时状态机零改动 |
| D8 | `struct canbus_msg` 8→64 | **单独一期、单独分支**，不与 ISO-TP 混在同一次提交；它改的是上游共享结构体（`third_party/klipper/src/generic/canbus.h:7-14`），是全仓风险最高的一类偏移 |
| D9 | CAN-FD 首发目标板 | 待定：必须是有 FDCAN 外设 + FD 收发器的板子（stm32 H7/G4 / atsam / atsamd 系）。**它决定改哪个 `fdcan.c`** |

---

## 2. 环境实测（2026-10-16 本机）

| 项 | 结果 |
|---|---|
| 内核 `can-isotp` | ✅ `7.2.9-zen1` 有 `can-isotp.ko.zst`，`alias: can-proto-6`，`max_pdu_size` 默认 8300 → 首次开 ISO-TP socket 自动加载，**无需特权** |
| `vcan` 模块 | ✅ `vcan.ko.zst`（带 `echo` 参数） |
| can-utils | ✅ `isotpsend` / `isotprecv` / `candump` / `cangen` / `ip` / `tc` 均在 |
| 建 vcan 的权限 | ❌ uid=1000，无 `CAP_NET_ADMIN`，`sudo` 需密码（`ip link add ... type vcan` → `Operation not permitted`） |
| 上游对照 | 上游 Klipper **无** ISO-TP（issue/PR 文本检索三组查询 total_count 均为 0）；3D 打印生态无先例（ISO-TP 的主场是 UDS 诊断）。**纯新增，无固件实现可抄** |

**结论**：vcan 需要用户**一次性**执行 `sudo ip link add dev vcan0 type vcan` + `ip link set up vcan0`；
之后测试进程全程免特权。CI/容器要 `--cap-add=NET_ADMIN`。夹具脚本必须先探测 vcan0 是否存在，
缺了就给出明确提示，不要静默失败。

---

## 3. 寻址与帧格式（可直接抄进任务书）

### 3.1 寻址

沿用 Klipper 的映射（`src/core/klippy/assign.rs:31-39`，`ARB_BASE = 0x100`）：
主机写 `0x100 + 2*id`，从站回 `+1`。`id ∈ 1..=255` ⇒ 仲裁 id 落在 `0x102..0x2FE`，
天然避开 admin 的 `0x3f0/0x3f1`。

因此固件侧现有的硬件 filter 集合 `{CANBUS_ID_ADMIN, id, id+1}`
（`third_party/klipper/src/stm32/can.c:136-175`）**不需要改**：ISO-TP 的数据与 FC 两向正好落在
`id` 与 `id+1` 上。同理 `uuid` 身份、admin `0x3f0` 节点指派、`[mcu_ids]` 分配、`scan` 全部可复用。

> ⚠️ 「主机给 `0x100+2*id`、从站回 `+1`」**不是 ISO 标准规定**（11 位正常寻址的 ID 里不含源/目标
> 地址，非 OBD 场景没有统一惯例），是**本项目约定**，写文档时必须标明。

### 3.2 帧格式（byte0 高 4 位 = PCI 类型）

| 帧型 | byte0 | byte1 | byte2… | 数据量 |
|---|---|---|---|---|
| SF | `0x0L`（L = 长度 1..7） | 数据 | … | ≤7 B |
| FF | `0x1H`（H = 长度高 4 位） | 长度低 8 位 | 数据 | 首帧 6 B；总长 12 位 ≤4095 |
| CF | `0x2N`（N = SN） | 数据 | … | 7 B |
| FC | `0x3F`（F：0=CTS / 1=WT / 2=OVFLW） | **BS** | **STmin** | 其余 padding |

- **SN**：从 1 起、+1 递增、模 16（…F,0,1…）。内核实现为准（`isotp.c` 的 `sn++ % 16`）。
- **STmin**：`0x00–0x7F` = 0–127 ms；`0xF1–0xF9` = 100–900 µs；其余保留。
- **BS=0** = 剩余帧一口气发完、不再等 FC。
- **超时**：N_As / N_Ar / N_Bs / N_Cr = **1000 ms**；N_Br / N_Cs 不是定时器而是性能不等式
  （`N_Br+N_Ar < 0.9·N_Bs`、`N_Cs+N_As < 0.9·N_Cr`）。
  ⚠️ 这组数值来自二手 UDS 知识库（ISO 原文付费且不可检索）——**写进手册要标「非一手」**。
- **内核的两个硬语义（固件从站必须对齐）**：① 内核**不发 WAIT 帧**，收端在收到 FF 时一次性决定
  能否收下整包；② 内核发送侧等 FC 超时 1 s 后报 `-ECOMM`。我们的消息上限 64 B，天然满足「FF 处
  一次性分配」。
- **内核 API 常量（已核对，勿用错名）**：`CAN_ISOTP = 6`、`SOL_CAN_ISOTP = SOL_CAN_BASE + CAN_ISOTP`
  （= 106）；sockopt 是 `CAN_ISOTP_OPTS=1` / `CAN_ISOTP_RECV_FC=2` / `CAN_ISOTP_TX_STMIN=3` /
  `CAN_ISOTP_RX_STMIN=4` / `CAN_ISOTP_LL_OPTS=5`；结构体
  `can_isotp_options{flags, frame_txtime(=N_As/N_Ar, ns), ext_address, txpad_content, rxpad_content, rx_ext_address}`、
  `can_isotp_fc_options{bs, stmin, wftmax}`、`can_isotp_ll_options{mtu, tx_dl, tx_flags}`；
  `sockaddr_can.can_addr.tp` **只有 `rx_id`/`tx_id`**（没有 `rx_ext/tx_ext`，扩展地址走 options）；
  `frame_txtime = 0` 表示「不改动」，真要 0 得传 `0xFFFFFFFF`。

---

## 4. 帧数与总线开销：ISO-TP vs 原 `canserial`（本规划的关键量化节）

**两条路的分帧语义根本不同**：

- `canserial` 是**字节流**：把整个待发缓冲按 8 字节切片，**块与块之间共享帧边界**，所以帧数 =
  `ceil(总字节数 / 8)`，**没有每消息的取整损失**。
- ISO-TP 是**消息**协议：每条消息的最后一帧要补齐到帧边界，并且**每条多帧消息额外要一个 FC 帧**
  （反向）+ 一个首帧 FF。所以：`N ≤ 7` ⇒ **1** 帧；`N > 7` ⇒ **2 + ceil((N-6)/7)** 帧。

其中 `N` = Klipper 块长度 = payload + 5（`MESSAGE_MIN=5`）。典型块 `queue_step oid/interval/count/add`
= 11 B payload ⇒ **N = 16**。

### 4.1 逐长度对照（单条消息）

| N（块字节） | payload | canserial（流式） | ISO-TP | 倍率 | 备注 |
|---|---|---|---|---|---|
| 5–7 | 0–2 | 1 | 1 | 1.00× | 单帧，**完全等价** |
| 8 | 3 | 1 | 3 | **3.00×** | 最坏点：为 3 字节多花 2 帧 |
| 9–13 | 4–8 | 2 | 3 | 1.50× | |
| 14–16 | 9–11 | 2 | 4 | **2.00×** | **`queue_step` 落在这里（N=16）** |
| 17–24 | 12–19 | 3 | 4–5 | 1.33–1.67× | |
| 25–32 | 20–27 | 4 | 5–6 | 1.25–1.50× | |
| 40 | 35 | 5 | 8 | 1.60× | |
| 48 | 43 | 6 | 8 | 1.33× | |
| 56 | 51 | 7 | 10 | 1.43× | |
| 64（上限） | 59 | 8 | 11 | **1.375×** | FF(6 B) + 9×CF + 1×FC |

两点规律：

1. **小块（N ≤ 7）完全等价**，一帧对一帧。
2. **代价集中在 N ≥ 8**，而且**越小越贵**：`N=8` 是 3×，`N=16` 是 2×，到 `N=64` 收敛到 1.375×。
   渐近极限是 `8/7 ≈ 1.143`（CF 每帧 7 B vs 切片每帧 8 B），**固定开销 +2 帧/消息**在中小消息上占主导。
3. 折算成**总线时间**（1 Mbit/s，标准帧：8 字节数据 ≈ 111 bit ≈ 111 µs；3 字节 FC ≈ 71 µs）：
   `N=16` 一条消息 canserial ≈ 222 µs，ISO-TP ≈ FF 111 + FC 71 + 2×CF 222 = **404 µs（1.82×）**。

### 4.2 混合负载估算

Klipper CAN 的流量构成大致是：主机→从站 = 大量小命令（`queue_step`、`set_next_step_dir`、
`reset_step_clock`…）+ 少量配置消息；从站→主机 = 定期 `ack`（约 N=10）+ 时钟/状态。按这个构成：

- 小命令（N ≤ 7）不涨价；
- `queue_step` 类（N ≈ 16）翻倍；
- `ack` 类（N ≈ 10）由 2 帧变 3 帧；
- 配置/大消息（N → 64）约 1.4×。

**这比单纯的「+14% 帧」严重得多**，因为 Klipper 的热路径恰好落在 `N = 14–16` 那一档（2×）。

### 4.3 除帧数之外的三个影响

1. **每消息一个往返延迟**：ISO-TP 发送方发完 FF 要**等 FC** 才能发 CF。BS=0 时每条多帧消息
   只多一个往返（≈ 2 帧时间 ≈ 260 µs @1 Mbit），但它是**变长的**——取决于对端固件什么时候
   调度出 FC（从站可能在步进中断里）。这是时间敏感路径（`queue_step`、`trsync`）的主要抖动来源。
2. **流控不是端到端的**：ISO-TP 的 FC 保护的是**分段重装缓冲**（我们只需 64 B），
   **不保护** klipper 的 192 B 控制台缓冲（`RECEIVE_WINDOW`）。也就是说今天「控制台缓冲满 →
   丢字节」的行为**照旧存在**。反过来，将来可以把 FC 做成真正的背压门（缓冲不足时回 WAIT /
   延后 CTS）——那是比 canserial 更优雅的一档，但**不在本次范围**（D3 已定 FF 处一次性决定，不发 WAIT）。
3. **失败模式变了**：`canserial` 丢一帧 = 字节流错位（整个会话 CRC 失效）；ISO-TP 丢一帧 =
   该条消息中止（`N_Cr` 超时），其余消息不受影响。可靠性判据不同，测试要分别覆盖。

**结论**：ISO-TP 在这条链路上是**功能/互操作**选择，不是性能选择。阶段 3/4 必须以实测抖动与
积压作为门禁（见 §6 R1）——若 `queue_step` 路径的抖动或延迟不可接受，退路是「ISO-TP 只用于
非实时通道，主协议仍走 canserial」。

---

## 5. 文件级落点

### 5.1 主机侧（`src/`，无需硬件可验收）

| 文件 | 改动 |
|---|---|
| `src/core/klippy/interface/devices/isotp_proto.rs` | **新增**：纯 ISO-TP 状态机（零 IO）。发送侧 SF/FF/CF 切分、SN 回绕、等 FC、BS/STmin、N_As/N_Bs 超时；接收侧重组、FC 决策、SN 校验、长度/PCI 合法性 |
| `src/core/klippy/interface/devices/isotp.rs` | **新增**：`IsotpDevice`（`Device` impl），复用 `CanFrame`、`wait_readable`（`canserial.rs:878-911`）、`stopped` 停止标志、`off_runtime` 阻塞模型 |
| `src/core/klippy/interface/mod.rs` | `Transport::Isotp(Arc<IsotpDevice>)` 变体（`:70-96`）；`Interface::isotp(...)`（参照 `:170-177`）；`send`/`receive`/`shutdown` 三个 arm（`:358-459`） |
| `src/core/klippy/config/mcu.rs` | `canbus_protocol` 键加入 `interface_keys()`（`:207-223`）与 `present()` 的 CAN 家族链（`:639-651`）；`Transport::Isotp` 变体（`:121-178`）；解析分支插在 CAN 兜底（`:845-868`）之前；`Transport::open()` 新 arm（`:938-1020`）；错误文案（`:877-884`） |
| `src/core/klippy/mcu/object.rs` | `load_config` 的 id/grouping 分派（`:1889-1916`、`:1930-1985`）——ISO-TP 走 `register_can`（与 `Transport::Can` 同一条） |
| `src/scan.rs` | 视需要：ISO-TP 设备是否纳入 `--interface` 扫描（v1 可不动，`uuid` 发现走 admin 通道，与协议无关） |

`identify.rs` 与消息层**不需要改**——`Mcu::connect` 只依赖 `Interface::send/receive`。

### 5.2 固件侧（`third_party/klipper`，需 main 裁决后动）

形状推荐：**把 ISO-TP 做成 canserial 的「CAN 承载层可切换」**，而不是另起独立节点。
`CONFIG_ISOTP=y` 时：

- `canserial.c:66-90` 的 8 字节切片 → 交给 `isotp.c` 的发送分段；
- `canserial.c:296-320` 的按 id 拼接 → 交给 `isotp.c` 的重组 + FC 回发；解出的 payload 仍写进
  **同一个 `receive_buf[192]`**（`canserial.c:46-52`），下游 `console_pop_input` /
  `command_find_block` / `command_dispatch`（`command.c:276,337`）**一个字节不改**；
- 于是 **admin `0x3f0`、`uuid` 身份、节点指派、`MUX_CAN`/`bridge` 全部原样可用**，
  不需要「ISO-TP 模式没有发现机制」这种退让。

| 位置 | 改动 |
|---|---|
| `src/generic/isotp.c` / `isotp.h` | **新增**：从站状态机（重组、FC 回发、SN、超时用 `sched.h` 承载）+ 发送分段 |
| `src/generic/canbus.c:29` | `canbus_process_data()` 是**所有架构 IRQ 的唯一汇聚点**，在这里加 PCI 分派，**不必改 5 个架构的 `can.c` IRQ** |
| `src/generic/canserial.c:66-90,296-320` | 承载层在这两处切到 isotp（目标是最小 `#ifdef` 面） |
| `src/Kconfig:429-441` | `config ISOTP`，`depends on CANBUS` |
| `src/Makefile:7-11` | `src-$(CONFIG_ISOTP) += generic/isotp.c`（本仓已在此处加 `mux.c`/`bridge.c`） |
| `test/configs/*-can-isotp.config` | **新增**构建级覆盖（照 `stm32f103-can-mux.config` 先例） |

硬件 filter **不用改**（见 §3.1）。

### 5.3 阶段 2.5：`linux` 目标加 SocketCAN

新增 `src/linux/can.c`（实现 `canhw_send` / `canhw_set_filter` / `canhw_get_status`，SocketCAN 版）
+ `src/linux/Kconfig` 的 `CANBUS` + `src/linux/Makefile`。`generic/usb_canbus.c` 已有「非 MCU 方式
实现 canbus 后端」的先例。

**一次解锁三件事**：① 真 `klipper.elf`（linux 目标）当 ISO-TP 从站在 `vcan` 上跑，无需硬件就能
端到端；② 现有 canserial 的 vcan 端到端（正好补掉 `TESTING.md` 的 M1-4 挂起项）；③ CI 级 CAN 测试。

---

## 6. 阶段划分与门禁

| 阶段 | 内容 | 验收 |
|---|---|---|
| 0 | 分支 + 本文档 + `TODO.md` 条目 | 文档入库；基线 `cargo test --workspace` 绿 |
| 1 | 主机侧：`isotp_proto.rs`（纯状态机）→ `isotp.rs`（设备层）→ 配置面/装载期接线 | 无 CAN 硬件下：状态机全项单测 + socketpair 假总线双向 + 配置解析/互斥测试；`cargo test --workspace` 全绿 |
| 2 | 固件侧 `isotp.c` + Kconfig + 构建级覆盖 | `test/configs/*-can-isotp.config` 编过、闸门绿、`upstream-deviations.md` 已登记 |
| 2.5 | `linux` 目标 SocketCAN | `vcan0` 上真 `klipper.elf` 当从站，与主机完成 identify |
| 2.6 | **CAN-FD 链路层**（独立分支，见 §7）：`struct canbus_msg` 8→64、`CANMSG_DATA_LEN` 截断、`canserial.c` 切片、各架构 `canhw_*`、FDCAN 驱动真的用 FD 帧、数据段波特率 Kconfig | FD 目标板上 `cans+FD` 与 `isotp+FD` 均能 identify；记录帧数下降实测 |
| 3 | vcan 端到端 + 互操作矩阵（我方实现 ↔ 内核 `can-isotp` ↔ `can-utils`） | 矩阵全绿；**记录每消息帧数实测值**，与 §4 的推算对账 |
| 4 | 真机（按 `flash-firmware` skill 刷写） | `identify` + `get_clock` + 一次真实动作；**量化 R1**（`get_clock` 抖动 / `trsync` 误差 / 队列积压），写进 `TESTING.md` |
| 5 | 文档收口（main，随代码同批提交） | 按 `doc-sync` 映射逐条；写死的测试名/计数回核 |

### 风险

| # | 风险 | 处置 |
|---|---|---|
| **R1** | ISO-TP 的 FC 往返给每条多帧消息插入变长延迟（§4.3-1），而 CAN 上跑的是时间敏感的步进指令 | BS=0 + STmin=0 压到最小；阶段 3/4 **量化门禁**：抖动/延迟需与 `cans` 可比；不达标则退回「ISO-TP 只用于非实时通道」 |
| R2 | 帧数开销：`queue_step` 档 **2×**，上限消息 1.375×（§4） | 阶段 3 实测记录；据此决定是否限制 ISI-TP 模式下的消息频率 |
| R3 | 内核 `can-isotp` 与本实现的分歧（padding、SF 长度边界、无 WAIT、DLC 规则） | 阶段 3 拿内核 + can-utils 当**严格参照对端**做矩阵；差异写进文档 |
| R4 | 与 `canserial` 在同一 `uuid`/id 上冲突 | 配置期点名拒绝：同一 `uuid` 被两家同时引用即报错（阶段 1 的互斥测试覆盖） |
| R5 | `third_party` 偏移（`CONFIG_ISOTP` + linux SocketCAN） | 先报 main 裁决；`upstream-deviations.md` 登记；构建级覆盖目标进闸门 |
| R6 | 目标主机内核无 `CAN_ISOTP` | D1 选自研用户态已规避（只需 raw CAN） |
| R7 | vcan 需一次性特权准备；CI 需 `NET_ADMIN` | 夹具脚本先探测并在缺失时给出明确提示 |
| R8 | `CAN_RAW` 与 `CAN_ISOTP` socket 同 ID 并存是否互扰（调研标记「未证实」） | 阶段 3 实测；它决定 ISO-TP 设备能否与 `canserial`/`candump` 共存 |

| R8 | `CAN_RAW` 与 `CAN_ISOTP` socket 同 ID 并存是否互扰（调研标记「未证实」） | 阶段 3 实测；它决定 ISO-TP 设备能否与 `canserial`/`candump` 共存 |
| **R9** | CAN-FD 改的是上游共享结构体（`canbus_msg`）与 5 个架构的 `canhw_*`，回归面远大于 ISO-TP | 单独一期、单独分支（D8）；构建级覆盖目标进闸门；`upstream-deviations.md` 重点登记 |
| R10 | CAN-FD 硬件受限：`rp2040`（can2040）与 `lpc176x` 是 classical-only，FD 只能落 FDCAN 架构；还需 FD 收发器 | 首发板选型前先确认（D9）；文档里写明“哪些架构能开 FD” |

---

## 7. 补充：CAN-FD 与 ISO-TP 的关系（2026-10-16 追加）

### 7.1 结论：同一家族，**两个正交轴**，不是二选一

| 轴 | 取值 | 管什么 |
|---|---|---|
| 链路层 | classical CAN / CAN-FD | 单帧容量 8→64 B、是否双比特率（BRS）、帧格式（FDF/BRS/ESI、DLC 编码 0–15、CRC 变长） |
| 传输层 | can-serial 的裸 8 字节切片 / ISO-TP 分段 | 一条 >帧容量 的消息怎么切、怎么重组、要不要流控 |

四个组合都成立且都有意义，所以配置上应该是**两个独立键**（D6），而不是把 FD 塞进现有的互斥矩阵：

```ini
[mcu canbed]
canbus_interface: can1
canbus_protocol: isotp      # cans | isotp
canbus_fd: true             # 链路层；默认 false
uuid: 0123456789ab
```

固件侧对应两个独立 Kconfig（`CONFIG_ISOTP` / `CONFIG_CANFD`），可以分别开关、分别验收。

### 7.2 两者一起做的**红利**（这才是「同时实现」的硬理由）

ISO-TP over CAN-FD（`CAN_ISOTP_LL_OPTS` 的 `mtu = CANFD_MTU`、`tx_dl = 64`）：

| 项 | classical + canserial | classical + ISO-TP | **FD + ISO-TP** |
|---|---|---|---|
| `queue_step`（N = 16）| 2 帧 | 4 帧（FF + FC + 2×CF） | **1 帧**（SF） |
| 上限消息（N = 64）| 8 帧 | 11 帧 | 3 帧（N > 62 → FF + FC + CF）|
| N ≤ 62 的任何消息 | ceil(N/8) 帧 | 2 + ceil((N-6)/7) | **1 帧，且无 FC** |

原因是 ISO 15765-2:2016 的单帧有两种形式：低 4 位 SF_DL（长度 1..7）与**首个字节 `0x00` 的转义式**（长度 >7），
后者允许 `TX_DL − 2` 字节载荷（内核源码常量 `SF_PCI_SZ4 = 1` / `SF_PCI_SZ8 = 2`；`isotpsend -L 72:64:1` 可复现）。
`tx_dl = 64` ⇒ **单帧最大 62 B 载荷**。Klipper 的块上限是 64 B，所以只有最大的那一档会退化成多帧。

**推论（对 §4 与风险表的修正）**：

- §4 的「`queue_step` 档 2×」是 **classical CAN 下才有**的现象。FD 一旦落地，
  **R1（FC 往返延迟）与 R2（帧数开销）双双消失**——N ≤ 62 全是单帧、没有任何 FC 握手，
  每消息只 1 帧，**比 canserial 还少**。届时 ISO-TP 就从「有性能代价的互操作方案」
  变成「没有代价的互操作方案」。
- 反过来：**如果目标只是「少帧数/低中断负载」，`cans + FD` 就够了，而且它比「ISO-TP + classical」简单得多**
  （不需要任何 ISO-TP 状态机）。ISO-TP 的不可替代价值在**互操作**。这两个目标必须分开谈，
  否则会把「想要 FD 的性能」错当成「需要 ISO-TP」。

### 7.3 成本极不对称（必须先知道再拍 D8/D9）

| 项 | ISO-TP（classical） | CAN-FD |
|---|---|---|
| 软件面 | 纯用户态/纯固件新模块，**不碰任何现有数据结构** | **要改上游共享结构**：`struct canbus_msg{data[8]}`（`generic/canbus.h:7-14`）、`CANMSG_DATA_LEN` 的 `dlc > 8 ? 8 : dlc` 截断、`canserial.c` 的 8 字节切片、**5 个架构的 `canhw_send`/`canhw_set_filter`**、FDCAN 驱动要真的用 FD 帧 |
| 现状核实 | 全仓零 ISO-TP 痕迹 | 固件全仓 `FDF`/`BRS`/`CANFD` **零命中**（只有 `AHBRSTR` 之类误匹配）；`stm32/fdcan.c:141-147,245-247` 只填 `data[0]`/`data[1]`、`dlc & 0x0f`——**是「用 FDCAN 外设跑 classical 帧」** |
| 硬件面 | 任何现有 CAN 硬件 | 只要 FDCAN 外设 + **FD 收发器**；`rp2040`(can2040)/`lpc176x` classical-only |
| 上游偏移风险 | 新增模块 + `#ifdef`，可控 | **最高**：改共享结构体会波及 admin 队列、mux、bridge、USB-CAN 路径 |

### 7.4 排期建议

1. **现在**：ISO-TP 在 classical CAN 上做完（阶段 1–2.5 **与 FD 无关**），唯一要提前做的是 **D7 参数化**
   （状态机的每帧数据上限当参数），成本几乎为 0。
2. **阶段 2.6**：CAN-FD 单独一期、单独分支（D8），先只做**链路层 8→64**（`cans + FD` 先跑通就验证了整条链路），
   再把 `isotp + FD` 组合上去。
3. **阶段 3/4**：用 `isotp + FD` 组合收口；此时 §4 的帧数/抖动门禁是以 FD 的数字计的。

---

## 8. 实施记录

（随实现推进回填：各阶段完成状态、提交号、实测帧数/抖动数字、与推算的偏差、余项。）
