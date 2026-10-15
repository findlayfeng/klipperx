# CAN ISO-TP 动工前调查与实施规划

写作当天：2026-10-16。**规格未知已侦察完毕**，本文是设计定稿 + 阶段拆分 + 文件级落点。
末尾「实施记录」是唯一随实现推进更新的回填节。

> ## ⚑ 架构定稿（2026-10-16，用户两条指示合并）
>
> **按设备类型选路，两种模式互不叠加**（不搞 `isotp + FD`）：
>
> | 设备 | 链路层 | 传输层 | 状态 |
> |---|---|---|---|
> | **支持 FD 的设备** | CAN-FD（64 B/帧） | **`cans`（can-serial，保持当前帧模式）** | 落地（不走 ISO-TP；但**保留可能性**） |
> | **标准 CAN 设备** | classical（8 B/帧） | **`isotp`** | 落地 |
> | 今天 | classical | `cans` | 默认，零变 |
> | `isotp + FD` | CAN-FD | ISO-TP | **保留可能性，不做/不验证** |
>
> 理由（用户口径）：Klipper 消息块可达 64 B，**大于一张 CAN 帧**。FD 用 64 B 帧就直接装下了，
> 不需要额外协议（保持现状最简）；标准 CAN 只有 8 B，需要 ISO-TP 来承载大于一帧的 Klipper 帧。
>
> **事实核对（写进手册前要弄清）**：canser 的**字节流本已能承载 >帧 的消息**（`canserial.c:79` 按 8
> 字节切片、接收侧拼接）。所以 ISO-TP 在 classical 上的实质增益是
> **消息边界 + 逐消息错误隔离 + 与标准工具/设备的互操作**，而不是「能不能装下大消息」。
> 如果背后还有一台具体的标准 CAN 第三方设备要对接，请告知型号/协议，会影响阶段 5 的验收基线。
>
> 因此：
> - §7.4 的排期作废，用 §9。
> - §9 的「ISO-TP 降级」**推翻**：ISO-TP 回到主线（**限 classical**），**固件 ISO-TP 从站要做**。
> - §4 的帧数对照就是 ISO-TP 在 classical 上的真实代价（`queue_step` 档 2×）；FD 在 `cans` 下
>   一律 1 帧/消息。

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
  **保留值已与内核对过**：内核 `net/can/isotp.c` 的 `isotp_rcv_fc()` 把保留区夹到 `0x7F`
  （`if (stmin > 0x7F && (stmin < 0xF1 || stmin > 0xF9)) stmin = 0x7F;`），与我们 `decode_stmin`
  的 `_ => 127 ms` 逐字节一致 ⇒ 本仓发/收两侧与内核在对端都同意该分支。
- **对端耐心值的边界条件**：内核自己的 ISO-TP 定时器是 **1 s**（`ISOTP_FC_TIMEOUT 1`，等 FC/CF）
  与 2 s（`ISOTP_ECHO_TIMEOUT`）；我们取 **25 ms** 比内核严 40 倍。它是**单方面的**——只约束我们等
  对端的耐心，不改对端行为。所以 25 ms 只在**两端都守时**时安全：我们自己的固件守时（且我们在
  发送方一侧），而内核作为对端也守时（它收到 FF 立即回 FC）⇒ 两种互操作都成立。但只要将来出现
  **一个慢对端**（比如某台真 ECU），这 25 ms 就会误判——那时应把它变成配置键。
- **BS=0** = 剩余帧一口气发完、不再等 FC。
- **超时**：ISO 默认 N_As / N_Ar / N_Bs / N_Cr = **1000 ms**；N_Br / N_Cs 不是定时器而是性能不等式
  （`N_Br+N_Ar < 0.9·N_Bs`、`N_Cs+N_As < 0.9·N_Cr`）。
  ⚠️ 这组数值来自二手 UDS 知识库（ISO 原文付费且不可检索）。
  **本项目采用 25 ms（不是 1000 ms）**——对齐本仓既有的「一个 CAN 往返」耐心值：
  `klippy/chelper/serialqueue.c:107` 的 `MIN_RTO 0.025`（本仓镜像 `src/core/klippy/mcu/mod.rs` 的
  `MIN_RTO: Duration = Duration::from_millis(25)`），而同一份文件以 `#define SQT_CAN 'c'`
  分派 CAN 链路 ⇒ 25 ms 本来就是这个项目在 CAN 上的取值。依据：两端都是我们自己的固件，
  最大 64 B 消息约 11 帧（1 Mbit/s 下 ~1.2 ms），25 ms 留 ~20 倍余量又能快速失败。
  固件侧的 `N_Cs` / `N_Br` 须分别 < `0.9 × 25 ms`。
  （另一个 25 ms 与之同名但不同事：`klippy/mcu.py:279` 的 `TRSYNC_TIMEOUT`，本仓镜像在
  `mcu/resource/trsync.rs:42`，是多 MCU 同步派发的截止时间。）
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

> **本条已被 §9 取代**（用户 2026-10-16 追加指示：`cans + FD` 为主，ISO-TP 降级为验证）。以下原文保留作决策遗迹。

1. **现在**：ISO-TP 在 classical CAN 上做完（阶段 1–2.5 **与 FD 无关**），唯一要提前做的是 **D7 参数化**
   （状态机的每帧数据上限当参数），成本几乎为 0。
2. **阶段 2.6**：CAN-FD 单独一期、单独分支（D8），先只做**链路层 8→64**（`cans + FD` 先跑通就验证了整条链路），
   再把 `isotp + FD` 组合上去。
3. **阶段 3/4**：用 `isotp + FD` 组合收口；此时 §4 的帧数/抖动门禁是以 FD 的数字计的。

---

## 8. CAN-FD 落地细节（侦察结果，2026-10-16）

> 全部来自仓库内只读侦察，`third_party/klipper` 未改动。

### 8.1 关键结论：**必须动 `struct canbus_msg`**

若要在 classical 载荷上只用 FD 帧格式（FDF=1 但 DLC≤8），结构可以不改——但**那样几乎没有吞吐收益**。
真 FD 数据段（>8 B/帧）⇒ 必须改：

| 位置 | 内容 |
|---|---|
| `generic/canbus.h:6-13` | `struct canbus_msg`：`data[8]`/`data32[2]` → `data[64]`/`data32[16]` |
| `generic/canbus.h:18` | `CANMSG_DATA_LEN(msg)` 的 `dlc > 8 ? 8 : dlc` 截断——**全局唯一的「一张帧最多 8 字节」钉点** |
| `generic/canserial.c:79` | `now = avail > 8 ? 8 : avail` —— 发送切片的硬编码 8 |
| `generic/canserial.c:49` | `admin_queue[8]` 按整个结构存盘 ⇒ 内存 8×（8×56B≈448B vs 现在 128B） |
| `generic/usb_canbus.c:81-97,124,139-147,274-278` | gs_usb 信封固定 8 字节 + `canhw_queue[32]` 按结构存 ⇒ 内存 32×；**若不做 gs_usb 的 FD 扩展，USB-CAN 桥保持 classical** |
| `generic/canserial.c:216-221,241-243,191-194` | admin 应答/`can_check_uuid` 的固定 8 字节布局——**admin 继续走 classical 小帧，不要跟着变长** |
| `generic/canserial.c:50-51,292` | `transmit_buf[192]`/`receive_buf[192]` 与 `RECEIVE_WINDOW=192`：语义上仍够用（一条 message ≤ 64 B），切片逻辑改后**不必扩** |

**好消息**：`mux.c`/`mux.h`/`bridge.c` 全程按字节流工作，用 `MUX_HEADER_BYTES`/`MESSAGE_MAX`
而不是 8（`bridge.c:96` 的 `in[2*(MUX_HEADER_BYTES+MESSAGE_MAX)]`）⇒ **FD 不波及 mux/bridge**。
`Makefile` 也**不必动**（改动全在既有文件内，不新增源文件，**也不建议把 FD 切片拆成新文件**）。
`MESSAGE_MAX=64` 不变——FD 只是把「一条 message 跨多帧」变成「可能 1 帧」。

### 8.2 三份 FDCAN 驱动的缺口（驱动层）

三份驱动现在都是「**用 FDCAN 外设跑 classical 帧**」：`(msg->dlc & 0x0f) << 16` 埋 DLC，
只写 `data[0]/data[1]`（=8 B），**FDF/BRS/ESI 全为 0**。逐处：

| 驱动 | TX 缺口 | RX 缺口 |
|---|---|---|
| `stm32/fdcan.c` | `:141` 补 BRS(bit20)/FDF(bit21) + FD DLC 编码（9..15→12/16/20/24/32/48/64）；`:142-143` 写满 `data[0..15]`；`:138-140` 补 ESI(bit31) | `:245` FD DLC 解码 + FDF 判帧型；`:246-247` 读 `data[2..15]` |
| `atsam/fdcan.c` | `:103` / `:104-105` / `:96-99` | `:204` / `:205-206` |
| `atsamd/fdcan.c` | `:120` / `:121-122` / `:113-116` | `:220` / `:221-222` |

- 比特率：三份都**只写 nominal**（`NBTP`：`stm32/fdcan.c:350`、`atsam/fdcan.c:322`、`atsamd/fdcan.c:334`），
  **均未写 `DBTP`** ⇒ 需要新的数据段波特率 Kconfig。
- 消息 RAM：`data[64/4]` 已预留，RXESC/TXESC=7 在 H7（`stm32/fdcan.c:356-359`）、atsam（`:332-338`）、
  atsamd（`:342-345`）已设；**stm32 G0/G4 未设**（`:355` 的 ifdef 只含 H7）—— 需补，见未验证项。
- `canhw_set_filter` 只看 id，**不用改**。

### 8.3 哪些架构不能 FD（硬件限制，必须在 Kconfig 上挡住）

| 架构 | 控制器 | FD |
|---|---|---|
| stm32 F0/F1/F2/F4 | bxCAN | ❌ 经典 CAN 控制器，无 FDF/BRS 寄存器 |
| lpc176x | LPC CAN（SJA1000 血统） | ❌ DLC 只到 8 |
| rp2040 / rp235x | can2040（PIO 位敲） | ❌ 库结构就是 `data[8]`（`lib/can2040/can2040.h:6-12`） |
| stm32 H7/G0B1/G4 | FDCAN | ✅ 驱动已就位 |
| atsam SAME70 | MCAN | ✅ |
| atsamd SAMC21/SAME51/SAME54 | MCAN | ✅（**SAMD51 不在 CANBUS 列表内**） |

### 8.4 Kconfig（建议名，待拍板）

- `CANBUS_FD`（bool，**default n**）：总开关。建议默认关以保持与上游 classical 网络兼容。
- `CANBUS_DATA_FREQUENCY`（int）：FD 数据段波特率，喂 `DBTP`；仅 FDCAN 架构可见。
- `CANBUS_FD` 必须 `depends on` 三家的 FDCAN 符号（`HAVE_STM32_FDCANBUS` `src/stm32/Kconfig:165-166`、
  `HAVE_SAM_CANBUS` `src/atsam/Kconfig:53`、`HAVE_SAMD_CANBUS` `src/atsamd/Kconfig:89`），
  **不得**在 lpc176x/rp2040 的 CAN choice 上暴露。
- 现有 `CANSERIAL`/`CANBUS`/`CANBUS_FREQUENCY`/`CANBUS_FILTER` 在 `src/Kconfig:429-440`。

### 8.5 主机侧落点（`src/core/klippy/interface/devices/canserial.rs`）

| 位置 | 改动 |
|---|---|
| `:929-940` `open_socket()` 之后、`bind_interface()` 之前 | `setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES=5, &1, 4)`（libc 未导出，需自定常量，紧邻 `:101-104` 现有常量） |
| `:81` `CAN_FRAME_SIZE = 16` | FD 的 `struct canfd_frame` 是 **72 字节**（id4 + len1 + flags1 + 保留2 + data64）；`len` 仍在 offset 4，**data 仍在 offset 8** |
| `:121-125,141-148` `CanFrame` | `data: [u8; 64]`、`new()` 上限 64 |
| `:156-175` `to_abi/from_abi` | 按 72 字节重排（`raw[4]=len`、`raw[8..]`） |
| `:250-258` `CanSerialLink::frames()` | `chunks(CAN_DATA_BYTES)` 的切帧上限 |
| `:662,683-691,754-772,841` | 读写缓冲 `[u8; CAN_FRAME_SIZE]` 与 `>= CAN_FRAME_SIZE` 断言 |
| `:802-853` `CanbusAdminSocket` | 同经 `open_socket`/`to_abi`，一并受影响 |

**会连带失效的测试**（必须重写，否则闸门红）：`:1150-1170` `test_can_frame_layout_matches_the_kernel`
（断 `CAN_FRAME_SIZE == 16` 与 `size_of::<SockAddrCan>() == CAN_FRAME_SIZE`——FD 下两者不再相等，
要拆成两个独立断言）、`:1143-1148` `test_a_frame_carries_at_most_eight_bytes`、
`:1237-1245` `test_the_query_frame_is_the_admin_broadcast`（`&raw[9..]` 的 no-padding 断言）、
`:1557` 与 `:1609-1610` 的切片断言。

### 8.6 硬件现实（仓库内可确证者）

- FDCAN 芯片目标：`test/configs/` 里有 `stm32h743/h723/g0b1/g431/g474/same70q20b`（**均未启用 CANBUS**，
  纯 MCU 选择），`samd51p20` 不在 CANBUS 列表内。
- **FD 收发器在哪块板：仓库内无任何依据**（无原理图/板级文档；`test/configs` 里 CAN 相关只有 classical 的
  `stm32f103-can-mux.config`）。
- 从代码成熟度看，**`stm32/fdcan.c` 最完整**（H7 分支已设 RXESC/TXESC），是首发首选；atsam/atsamd 次之。

### 8.7 六个未验证项（落地前必须收口）

1. **stm32 G0/G4 的 RXESC/TXESC 默认元素尺寸**是否够 64 B（`stm32/fdcan.c:355-359` 只在 H7 设）——需查参考手册。
2. **FD 收发器在哪些板上**——仓库内无依据，需你给型号/原理图。
3. **can2040 是否有 FD 实现**——仓库内无依据（上游需查）。
4. **`CAN_RAW_FD_FRAMES` 在 classical 接口上的行为**（与古典固件混跑）——需 vcan/真机实测。
5. **RAM 预算**：`admin_queue[8]`（8×）与 `canhw_queue[32]`（32×）会同步放大，小 RAM 的 MCU 要核算。
6. **vcan 是否支持 FD**（`ip link set vcan0 type can fd on`）——主机侧无硬件验收路径的前提。

---

## 9. 阶段与支持矩阵（取代 §6 与 §7.4）

### 9.1 支持矩阵（配置的两个正交键，但只验证三个组合）

```ini
canbus_protocol: cans | isotp     # 传输层；默认 cans
canbus_fd: true | false           # 链路层；默认 false
```

| 组合 | 用途 | 验收 |
|---|---|---|
| `cans` + classical | 今天的行为（默认） | 现有测试，零回归 |
| **`isotp` + classical** | 标准 CAN 设备 | 主机 ISO-TP 状态机单测 + 固件从站 + vcan/真机端到端 |
| **`cans` + FD** | 支持 FD 的设备 | 主机 72 B `canfd_frame` 单测 + 固件 FD 链路 + vcan/真机端到端 |
| `isotp` + FD | **保留可能性，不做** | 配置两轴正交所以能表达，但**有已知的实现清单**：ISO-TP over CAN-FD 必须**强制 padding**（内核 `net/can/isotp.c` 的 `padlen()`：发送方对 `len > 8` 的帧强制补到 DLC 台阶 12/16/20/24/32/48/64，接收方要求 FF 的 `cf->len == padlen(cf->len)` 否则丢弃）⇒ 我们 D3 定的「发送不 padding、DLC＝实际字节数」**在 classical 上正确、在 FD 上会与内核不互通**。要做时补 `padlen` 即可（约 3 行）；在那之前手册里写「未验证」 |

### 9.2 阶段

| 阶段 | 内容 | 验收 | 依赖 |
|---|---|---|---|
| 0 | 分支 + 本文档 + `TODO.md` 条目 | ✅ `33afc7a` / `19b885d` / `a2db7c0` | — |
| **1a** | **主机侧 CAN-FD 链路**（`canserial.rs`：72 B `canfd_frame`、`CAN_RAW_FD_FRAMES`、按 wire 切帧） | ✅ `7ea3c51`；can serial 30 测 / interface 171 测 / scan 22 测全绿 | — |
| **1b** | **主机侧 ISO-TP 纯状态机**（`isotp_proto.rs`） | ✅ `0085b2c`（22 测）+ `af1b87d`（超时改 25 ms） | — |
| **2a** | 固件 FD 链路层（`canbus_msg` 8→64 + 三份 `fdcan.c` + 两 Kconfig） | ⏸ **用户 2026-10-16 指示挂起**（「FD 的只先做主机侧实现，后续等条件」） | — |
| **2b** | **固件 ISO-TP 从站**（can-serial 的 CAN 承载层可切换：`generic/isotp.c` + `canbus.c:29` 分派 + `canserial.c` 两处承载点） | 待开工。**不依赖 2a**：classical 上每帧仍 8 B，不碰 `struct canbus_msg`/驱动/mux/bridge；重装缓冲自成一块 | — |
| **3a** | 配置面：`canbus_protocol` + `canbus_fd` 两个键 + `config::Transport` 变体 + `Interface` 接线 | 等 1a + 1b（✅ 已就绪） | — |
| **3b** | 固件侧接线与非法组合点名拒绝 | 等 2b | — |
| 4 | `linux` 目标加 SocketCAN（两种模式都要能跑） | 需先侦察控制台选择机制 | — |
| 5 | vcan + 真机 | FD 板跑 `cans+FD`；**classical 板跑 `isotp+classical`（对端就是我们自己的固件）**；记录帧数/抖动 | — |
| 6 | 文档收口 | 按 `doc-sync` 逐条 | — |

### 新增/变更风险

| # | 风险 | 处置 |
|---|---|---|
| **R11** | `struct canbus_msg` 变大会波及 `admin_queue`（8×）与 `canhw_queue`（32×）的内存预算 | 阶段 2 先核算 RAM；必要时把 admin 队列改成独立的 8 字节小结构 |
| **R12** | `CANBUS_FD` 默认关，但一旦打开就必须**两端同时是 FD**（一 classical 一 FD 会直接通信失败） | 文档写清；主机侧 `canbus_fd` 与固件 `CONFIG_CANBUS_FD` 必须配对；错误文案要点名 |
| **R13** | 三份 `fdcan.c` 同时改，回归面广（但它们已被现有古典 CAN 测试覆盖不到——都是构建级） | 三份改动分开提交；构建级覆盖目标覆盖 stm32/atsam/atsamd 三路径 |
| R14 | DLC 编码 9..15 → 12/16/20/24/32/48/64 的换算出错会造成长度错位（静默数据损坏） | 沿用上游口径：**`msg->dlc` 始终表示原始字节数**（不是 DLC 编码），编码/解码只在驱动层；加单测钉住映射表 |

---

## 10. 实施记录

### 2026-10-16：阶段 1a + 1b 完成（已合并进 `agents/feat-can-isotp`）

**1b — 主机侧 ISO-TP 纯状态机**（`0085b2c`，22 测；`af1b87d` 把超时改成 25 ms）

- 新增 `src/core/klippy/interface/devices/isotp_proto.rs`（1155 行含测试）+ `devices/mod.rs` 一行。
- 公开 API：`IsotpConfig{frame_bytes, max_wait_frames}`（`classical()` 8 B / `can_fd()` 64 B，
  `sf_max`/`ff_max`/`cf_max`）；`Input{Poll, Send(&[u8]), Frame(&[u8])}`；
  `Action{Transmit(Vec<u8>), Received(Vec<u8>), Wait{next_poll}, Failed(IsotpError)}`；
  `IsotpState::new/reset/step(now, input)`。**`Transmit` 可能一次给多张，调用方必须按序发。**
- 实现者自己定、规格未钉死的选择：STmin 保留值（`0x80–0xF0`/`0xFA–0xFF`）当作 127 ms 而非报错；
  空消息 ⇒ `EmptyMessage`；上一消息未发完时 `Send` ⇒ `TxBusy`（不排队）；FF 声明长度 ≤ `sf_max`
  或为 0 ⇒ `InvalidPci`；classical 上出现转义形式 ⇒ `InvalidPci`；`max_wait_frames` 默认 4；
  收端固定回 `CTS/BS=0/STmin=0`。
- **超时改成 25 ms**（见 §3.2 的依据）；测试不写死该值（从常量派生），所以改常量不会假红。
- 未接线：设备层（socket + `Device` trait）、配置键、固件从站——都在后续阶段。

**1a — 主机侧 CAN-FD 链路**（`7ea3c51`，仅 `canserial.rs`）

- 帧容量从全局常量变成每设备持有的 `CanWire{Classical, Fd}`（`data_bytes()` = 8/64）；
  `to_abi`/`from_abi`、读写缓冲与数据报长度断言全部按 wire 走；FD 在 bind 前
  `setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES)`。
- **签名变更**：`CanSerialLink::frames()` 多一个 `wire` 参数；`CanFrame::new` 名/形不变但上限 8→64
  （宽度检查在 `to_abi(wire)`，超宽返 `Err`、绝不截断）。
- 兼容性：`CanSerialDevice::open()` 与 `CanbusAdminSocket::open()` 语义不变（classical），
  FD 走新增的 `open_with_wire()`；`test_wire`/`read_wire` 保留为 classical 包装 ⇒
  `interface/mod.rs`、`scan.rs` 零改动。
- 验证：canserial 30 测、interface 171 测、scan 22 测、`cargo test -p klipperx --lib` = **3018 passed / 0 failed / 13 ignored**（99 s，main 合并后复跑）。
- 下游要知道的坑：**FD socket 与 classical 接口混跑**尚无实测（开 `CAN_RAW_FD_FRAMES` 后读到的
  数据报形态、以及对端仍为 classical 时的行为），归阶段 5 实测。

**2a — 固件 FD 链路层：搁置**

- 用户 2026-10-16 指示：「FD 的只先做主机侧实现，后续等条件」。
- 已派出并已停止；worktree 干净、分支零提交（**一个字节未写**），故无 `abandoned/` 需要留档，
  只回收 worktree。改动清单一字未动地留在 §8.1–§8.4，恢复时直接照它下任务书。
- **挂起只影响 FD 的固件半边**；2b 不受影响（见 §9.2 的阶段表）。
