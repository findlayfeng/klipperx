# 桥下游端口模块化与「无 admin 设备」共存（M1）动工前调查（2026-10-13）

本文为 `TODO.md` 的 **M1** 做的动工前调查与开发计划，也是这一轮设计讨论的拍板记录。

- 本地基线：`abf5d0d`（串口复用 mux 与桥的首批实现已并入 `work`）
- 上游基线：`third_party/klipper/`（子模组，`klipperx` 分支）
- 相关规范：[串口复用协议](../klippy/developer-manual/serial-mux-protocol.md)（单元/帧头/admin 命令族）、
  [MCU 连接方式](../klippy/user-manual/mcu-connection.md)、[配置文件参考](../klippy/user-manual/config.md)
- 相关记录：本轮已落地的实现（子模组 `src/mux.c`、`src/bridge.c`；外层 `mcu/host_libraries.rs`、
  `interface/devices/host.rs` 的 `load_mux`、`crates/test-support` 的第二份 mux 库变体）

> **本文是 2026-10-13 当时的拍板记录，不是规范。** 正文 §1–§5 记的是那天的设计与分期，
> 随实现推进会过时；**余项与现状以 §6「实施记录」为准**，`TODO.md` 是权威台账。

---

## 0. 结论摘要

1. **问题**：复用（mux）与桥的 id/路由归属今天**依赖设备答扫描**（桥从下游 `NEED_NODEID` 答里
   学 `uuid → 端口`）。**原版 klipper 串口固件**既不答扫描也无法被点名 ⇒ 单设备下游能靠"剥头"
   工作，但归属无从建立；共享路径上更无法区分。
2. **答案**：把归属从「线上学习」换成「**配置声明 + 主机告知桥**」——配置声明拓扑，主机通过
   **桥自己的 admin 通道**（端口级命令）把 `(端口, id)` 直接告诉桥，**下游设备完全不参与**。
3. **配置面**：下游端口按 klipper 一贯的设备节形式定义（`[uart_device <名>]` / `[can_device <名>]`，
   `i2c`/`spi` 直接复用已有节类型），子设备引用**端口节名**；`[bridge]`/`[bridge_port]`/`bridge_admin`
   都不需要——桥与端口从引用推算，"没有 admin"从「无 `uuid:` + uart 端口」推算。
4. **顺带解决**：判定（独占/共享）从「按 `host_library:` 路径」**下移到端口**；端口可被**非 MCU
   消费者**引用（桥的 i2c/spi 端口上挂传感器等）；上游传输（串口 / CAN）与下游端口类型正交。
5. **M2（经 admin 复位固件）并入本计划**：它与端口级命令是同一套机制——复位同样分"**面向设备**"
   （设备自己执行，走它自己的 admin）与"**面向端口/桥**"（桥替它做线级复位，无 uuid 的 uart 端口
   **只能**走这条）。固件半边（命令族）落 **P2**，主机半边（`restart_method` 接入）落 **P3**。

---

## 1. 动机与范围

**要解决的**（M1）：下游接的是**没有 admin 通道的旧设备**（原版 klipper 串口固件，或任何不实现
mux 的设备）时，主机仍要能把它当普通 `[mcu]` 连上并跑通 identify + `get_clock`。

**在范围内、但是并入执行的**：M2（经 admin 复位固件）——与端口级命令同属"面向谁"的分界（§0.5），
固件半边进 P2、主机半边进 P3；**不在本文范围**：CAN 上游的完整后端实现（本文只要求接口与判定
正交，见 §4.4），以及 i2c/spi 端口的写法（§5.6 未决 a）。

**一条硬约束**：legacy（无 admin）设备**只能出现在单设备端口**上。它与邻居无法区分，所以共享
端口上"声明为 legacy"是配置错。

---

## 2. 已定案的配置面（拍板点）

```ini
# 端口：按一贯的设备节形式定义；类型作节名、名字供引用，选项带类型前缀
[uart_device uartxx]
uart_mcu: bridge_mcu        # 这颗 uart 在哪颗 MCU 上（= 桥）
uart_oid: uart0             # 该 MCU 上对应的 oid 名（固件上报的那个）——单独一项，不折进上一行
uart_baud: 250000           # 端口自身的线参数（可选）

[can_device canxx]
can_mcu: bridge_mcu
can_oid: can0

# i2c/spi 端口：复用 klipper 已有的节类型（`[i2c <名>]`/`[spi <名>]`），消费者按节名引用。
# 如何表达"沿用桥固件已有的那个 oid"（klipper 的 `[i2c]` 是引脚式、自己建 oid）见 §5.6 未决 (a)。

# 桥自身那颗设备（上游传输照旧：host_library: 或 CAN 的 uuid:/canbus_*）
[mcu bridge_mcu]
host_library: /usr/local/lib/libklipper_host.so

# 子设备：引用端口节名
[mcu child_old]
uart: uartxx                # 无 uuid + uart ⇒ 旧思路（原始帧、不发 SET、不等握手）

[mcu child_new]
uart: uartxx
uuid: 11aa22cc33dd          # 同端口第二台必须写 uuid

[mcu child_can]
can: canxx
uuid: 44dd55ee66ff          # CAN 端口必须有 uuid
```

**拍板点**（逐条来自这轮讨论）：

1. 删掉 `[bridge]` / `[bridge_port]`：桥与端口由引用推算，不需要自己的节。
2. 删掉 `bridge_admin`：legacy 由「无 `uuid:` + uart 端口」**推算**，不声明。这条对**新固件也安全**——
   单设备路径上原始帧永远是合法形态，设备只看到裸帧就按老串口 MCU 方式工作。
3. **保留** `[uart_device]` / `[can_device]`：它们是"可被节名引用的端口"，正是非 MCU 消费者
   （i2c/spi 那类 `*_bus:` 引用）能挂到桥上的载体。
4. `uart_oid` / `can_oid` **单独一项**（不与 `uart_mcu` 折成 `mcu:oid` 字符串）。
5. `i2c`/`spi` 端口复用 klipper 已有节类型（`[i2c <名>]`/`[spi <名>]`），只在选项上表达"属于桥的哪个 oid"。

---

## 3. 推算规则（写死七条）

1. **桥** = 端口节里 `*_mcu` 指向的那颗 `[mcu]`；子设备引用端口节时推算出的桥必须与它一致
   （不一致 = 配置错）。
2. **端口** = 端口**节**（`uart_device uartxx`）；它在固件里的身份 = `(mcu 名, oid 名)`。
3. **类型** = 节类型（`uart`/`can`/`i2c`/`spi`），必须与固件为该 oid 报出的类型一致。
4. **"剥不剥头"看的是设备说不说单元，不是这条线有几台**：
   - 设备**被命名过**（有 id——所有非 legacy 的 CAN/串口设备，见规则 8/9）⇒ 该端口**收发都带头**，
     上行头由设备自己按 `+1` 加；
   - **legacy** 设备（无 `uuid:` + uart，规则 5）⇒ 该端口**剥头/加头**，设备只看到原始帧。
   参考组：**端口节被几颗 `[mcu]` 引用 ≥2 时，每台都必须写互异 `uuid:`**（都命名 ⇒ 都带头）；
   legacy **只允许**出现在单设备端口上（与 §1 的硬约束一致）。
   **串口端口不是天然的 1 对 1**：RS-485 一类多站线路上可以挂多台设备——它们都必须是说单元的
   设备（各自走自己的 `uuid → id` 分配）。
5. **无 `uuid:` + uart 端口 ⇒ 走旧思路**：不发 `SET_NODEID`、不等重确认握手、按原始帧对话。
6. **CAN 端口必须有 `uuid:`**（CAN 的寻址本来就靠它）；无 uuid 的 CAN 端口 = 配置错。
7. **上游路径是否按 id 寻址** = 该路径被几颗 `[mcu]` 引用 **+ 其上各桥的下游设备数**——下游设备的
   帧也走同一条流。只有一颗 `[mcu]`（桥）但带 N 台下游时，这条线**必须**按 id 寻址。
   host 库那条（上游是宿主 API 而非 tty）同理：由"这座桥有没有下游"决定。
8. **每一台 CAN / 串口设备都要被分配 id**（`[mcu_ids]` 的序列照旧覆盖它们，下游设备也一样）。
9. **非 legacy 的设备仍走它自己的 `uuid → id` 分配流程**（`SET_KLIPPER_NODEID` + 重确认握手）——
   端口级命令只负责"给桥建表"，**不代替给设备命名**。legacy 设备**略过**这一步（它没有 admin）。

---

## 4. 机制

### 4.1 端口表由固件报出

桥在 identify 字典里报一张端口表，每条含 **`index` + `oid 名` + 类型 + 是否单设备**（路径可选，
供诊断）。名字↔物理路径的对应留在**固件 Kconfig**（`HOST_BRIDGE_SERIAL_PATH` 那类），**主机不猜路径**。
`index` 是必须的：端口级命令在线上只能带紧凑整数，名字留在主机侧做人机对照。

### 4.2 端口级命令：归属由主机经桥自己的 admin 通道告知桥

新增 admin 命令 **`SET_PORT_NODEID(index, id, flags)`**：面向**桥自己**、**不下发**给下游、**不等回执**、
**幂等**（可选带 uuid 只为桥日志/校验）。桥收到就建 `id ↔ 端口` 两张表，并按 `flags` 记下
**这条端口的设备说不说单元**（决定收发是否剥/加头，见 §4.6）。

于是：**下游设备完全不参与** ⇒ 不再需要"未知 uuid 先探一次"的兜底，也不需要设备答扫描。
带 `uuid:` 的设备仍走它自己的 admin（`SET_NODEID` + 重确认握手）——那是"给设备命名"，
与"给桥的端口建表"是两件事，两条都发即可（统一、不特判）。

### 4.3 时序约束（必须写进实现）

oid 名/端口表**只有 identify 之后**才拿得到，而配置解析发生在 connect 之前 ⇒ 对
`(uart_mcu, uart_oid)` 的校验（端口存在？类型对得上？单设备？CAN 端口有 uuid？）**只能排在
identify 之后**；配置解析期先"待验放行"，identify 时补验并报错。

### 4.4 两级寻址（上游传输与端口类型正交）

| 上游 | 主机怎么找**桥** | 主机怎么找**桥下游的某台设备** |
|---|---|---|
| 串口（mux） | 桥在共享串口上的 mux id | 流内**另一个 mux id**（= 桥的一条端口） |
| CAN | 桥的 CAN node id（`uuid:` + `canbus_*`） | CAN 载荷里那条字节流**内部**的 mux id（同一套单元格式） |

所以端口节与"桥经串口还是 CAN 连"无关。

### 4.5 id 分配保留：端口级命令与设备命名是**两件事**

建桥的路由表**不取代**设备自己的 `uuid → id` 分配：

| 流程 | 谁执行 | 谁接收 | 谁需要它 |
|---|---|---|---|
| `SET_KLIPPER_NODEID(uuid, id)` + 重确认握手 | 设备自己 | **设备** | 非 legacy 的每台 CAN/串口设备（含下游设备）——它要知道自己答在哪个号上 |
| `SET_PORT_NODEID(index, id)` | **桥** | **桥自己**（不下发） | 桥需要知道"这个 id 的帧往哪条端口送" |

两条**都要发**（统一、不特判）；**legacy 设备只发后者**（它没有 admin，略过前者）。于是：
**所有 CAN 与串口设备都被分配 id**（`[mcu_ids]` 照旧），legacy 设备的号只存在于主机的账与桥的表里、
设备自己不知道——这正是"与没有 admin 的设备共存"的形状。

### 4.6 剥头/带头由**设备说不说单元**决定，并由主机告知桥（不能按台数推）

桥必须知道**每条端口的设备说不说单元**，否则两端会错配。**不能用"映射到该端口的 id 数"推**：

| 该端口的情况 | id 数 | 正确行为 |
|---|---|---|
| 单设备 + 被命名（有 `uuid:`，或本就是要走 mux 的新固件） | 1 | **带头**（设备自己按 `+1` 加上行头，桥原样转发） |
| 单设备 + legacy（无 `uuid:` + uart） | 1 | **剥头/加头**（设备只看到原始帧） |
| 多站线路（RS-485 等）≥2 台，都被命名 | ≥2 | **带头** |

⇒ 判据是"**说单元 / 不说单元**"，与台数无关（前两行 id 数都是 1）。**主机知道**这件事（规则 4/5
推出来），所以由主机在**端口级命令的 `flags`** 里告诉桥（§4.2）。于是固件今天那个编译期的
`single_device` 标志退场：多站线路不需要额外声明（它天然是"都命名 ⇒ 都带头"），legacy 也不需要
额外声明（"无 uuid + uart" 就是它的定义）。

### 4.7 桥的三份存在（避免把它当纯配置概念）

| 位置 | 形态 | 职责 |
|---|---|---|
| 固件 | `CONFIG_BRIDGE` 转发体（`src/bridge.c`） | 上游那颗 mux 设备；下游端口的**所有者**（打开/持有/线级复位）；**转发与路由**（两张表、admin 扇出、单设备剥头/加头）；端口表上报 + 执行端口级命令 |
| 主机 | 由引用**推算**出的对象（无配置节，形态同 `SerialPorts`/`HostLibraries`） | 记住每端口的设备数/寻址方式/端口 index；发 `SET_PORT_NODEID`；identify 后校验 |
| 配置 | **省略** | — |

---

## 5. 开发计划（六期，每期可独立交付与验收）

### P1 配置面（klipperx，不动固件）

- 新节类型：`[uart_device <名>]`（`uart_mcu` / `uart_oid` / `uart_baud` `uart_restart_method`…）、
  `[can_device <名>]`（`can_mcu` / `can_oid`）；`i2c`/`spi` 复用已有节类型。
- `[mcu]` 的传输键扩成 `uart:` / `can:`（引用端口节名），与 `serial:` / `canbus_*` /
  `host_library:` / `test:` **并存**，保持"只准一个"的冲突校验。
- **端口分组**（键 = 端口节名，形状与 `mcu/serial_ports.rs` 同构）：挂在每个端口上的 `[mcu]` 列表
  ⇒ 独占（1 台，原始帧）/ 共享（≥2 台，按 id 寻址、各自必须互异 `uuid:`）。
- **legacy 推断**（规则 5/6）与冲突报错（共享端口缺 uuid、CAN 端口缺 uuid、端口绑的桥与引用
  推出的桥不一致、legacy 出现在共享端口）。
- **改造**：`mcu/host_libraries.rs` 的判定从"`host_library:` 路径"泛化成"端口节名"（旧的路径判定
  只保留给既有 `host_library:` 多节形态，别让两套共享判定长期并存）；`config/mcu.rs` 的
  `interface_keys()`/`transport_for()` 扩键。
- **验收**：解析/分组/校验的单测（独占、共享、legacy 推断、CAN 无 uuid 报错、键冲突、桥不一致）；
  `cargo test --workspace` 全绿；`cargo doc` 告警不新增。
- **文档**：`config.md`（新节与新键）、`mcu-connection.md`（连接方式表）、`upstream-deviations.md`
  （`uart_device`/`can_device` 与 `mcu:oid` 引用是本仓独有形式）。

### P2 固件端口表 + 端口级命令（子模组）

- 端口表报进 identify 字典（`index` / `oid 名` / 类型；"这台说不说单元"由主机的命令带 flags
  告知，见 §4.6，固件不再需要编译期 `single_device`）；`SET_PORT_NODEID(index, id, flags)` 的处理
  （建表、幂等、不下发、无回执）。
- **M2 的固件半边（同一期，共用同一套命令机制）**：复位命令族分两条——
  **面向设备**（设备自己执行；对照 CAN 侧已有的 `CMD_REQUEST_BOOTLOADER = 0x02`）与
  **面向端口/桥**（桥执行线级动作：对该端口的 DTR/RTS 之类；无 uuid 的 uart 端口**只能**走这条）。
  两条都沿用 `KXID` 帧形与端口 `index`。
- **验收**：三种配置（仅 mux / mux+桥 / 全关）0 告警；**探针**证明"主机发一条端口级命令后，
  该 id 的数据单元被转到该端口、该端口的原始帧被加上该 id 的应答头上行"，**全程不要求下游答
  admin**；端口表能在 identify 字典里读到。
- **文档**：固件 `src/host/README.md`、`serial-mux-protocol.md`（命令族表 + 两级寻址）。

### P3 attach 走端口 + M2 主机半边（klipperx，端到端）

- 装载期：端口节名 → `(mcu, oid)`；identify 后与固件端口表对账 → 取 `index`；按规则 4/5 分岔：
  有 `uuid:` ⇒ 现有 `load_mux`（`SET_NODEID` + 重确认握手）；无 `uuid:` + uart ⇒ **跳过**，
  只对桥发 `SET_PORT_NODEID(index, id)`，之后按原始帧收发。
- **验收（端到端）**：桥下游挂"**没有 mux 的固件**"（`test-support` 那份无 mux 的库，或现有假响应器
  夹具），主机把它当普通 `[mcu]` 连上并跑通 identify + `get_clock`；另一条回归：带 `uuid:` 的设备
  走原路不退化。
- **测试策略（2026-10-14 拍板）**：本期验收**只走软件路径**（pty + 回环夹具/假响应器 + 假固件），
  真机项（DTR/RTS 极性、双板接线、旧固件板）**后移**，与 M2 一起排在 **M1+M2 全部落地后**统一跑，
  清单见 `TESTING.md` 的「M1/M2（桥下游端口 + admin 复位）」一节；不因缺真机停下开发。
- **M2 的主机半边（同一期）**：把复位命令接进 `restart_method`（新增一种方法，或让共享/桥路径内部
  改走它）；"面向端口/桥"的复位用于无 uuid 的 uart 端口（那条线只有它一台，切 DTR/RTS 不打断别人），
  带 uuid 的设备走它自己的 admin。要定：复位到哪一层（重进 `sched_main` / 清 shutdown / 真复位 MCU /
  进 bootloader）、幂等与回执（admin 流程没有回执）、旧固件降级（不能答 admin 的设备继续 `command`）。
- **风险**：**测试夹具体积**——要在 pty 上跑一份"旧固件"（或等价的假响应），这可能是 P3 里最花时间
  的一块；先做夹具、再做判定。

### P4 判定与传输正交

- 规则 7：上游路径"是否按 id 寻址"要把**下游设备数**算进去（`mcu/serial_ports.rs` 的分组要扩）。
- **验收**：单测覆盖"一颗 `[mcu]` 但带 3 台下游 ⇒ 该线按 id 寻址"。
- **文档**：两级寻址表进 `serial-mux-protocol.md` 与 `mcu-connection.md`。

### P5 CAN 上游（后置）

- 桥经 CAN 连接时**这三条命令**（`SET_PORT_NODEID` + 复位两条）与端口表的帧形（CAN admin 帧 /
  CAN 载荷内的同一单元）。

### P6 多站串口（RS-485 一类，后置）

- 新增端口类型/后端（`src/bridge_port_485.c` 之类：半双工收发与方向控制归它）；主机侧则是"一个
  串口端口节被多颗 `[mcu]` 引用"⇒ 保留帧头、每台设备走自己的 `uuid → id` 分配（§4.5）；legacy
  不允许出现在这类端口上（§3 规则 4）。
- **验收**：一条多站端口上挂 ≥2 台说单元的设备，各自的 id 分配与数据帧都正确路由（都带头）；
  另一条对照：单设备 + legacy 端口仍是剥头/加头（§4.6 两行判据各覆盖一次）。

### 5.6 未决（开工时定）

a. **i2c/spi 端口的写法**：klipper 的 `[i2c <名>]`/`[spi <名>]` 是**引脚式**（自己在该 MCU 上建 oid），
   而桥的 i2c/spi 端口是**固件里已有的 oid** ⇒ 需要一个选项表达"沿用已有 oid 而不是按引脚新建"。
   本文只要求"端口能被非 MCU 消费者按节名引用"，具体键名与语义留到 P1 定。
b. **端口 `index` 的稳定性**：固件重排端口会让 index 漂移。稳定的标识是 **oid 名**（identify 表里
   两个都报）；index 只用于线上命令，不需要稳定。
c. **端口级命令的帧形**：`SET_PORT_NODEID` 的参数集（`index` + `id` + `flags`；要不要带 uuid 做校验）、
   `flags` 的取值（至少"设备说不说单元"；将来可能还有"半双工/方向控制"之类）、以及它在 CAN 上游下的
   帧形（CAN admin 帧 vs 载荷内的同一单元）。
d. **M2 的复位命令**：名字与参数（要不要带"复位到哪一层"的选择）、面向设备 vs 面向端口/桥两条却
   只有后者能服务 legacy、以及**没有回执**下主机如何确认复位已发生（P3 接入 `restart_method` 时定）。

### 改造清单（汇总）

| 侧 | 文件 |
|---|---|
| klipperx | `config/mcu.rs`（新节与新键）、`mcu/`（端口分组，泛化 `host_libraries.rs`）、`interface/devices/host.rs`（attach 分岔）、`mcu/restart.rs`（M2 主机半边：接入 admin 复位）、`assign.rs`（不动） |
| 固件 | `src/bridge.{c,h}`（端口表 + `SET_PORT_NODEID`）、`src/host/Kconfig`（端口路径留在固件）、`src/mux.c`（不动） |
| 手册 | `serial-mux-protocol.md`、`mcu-connection.md`、`config.md`、`upstream-deviations.md` |

---

## 6. 实施记录

**P1（配置面）与 P2（固件端口表 + 端口级命令）已于 2026-10-13 落地。** 本节同时记两期；
为避免 P1/P2 两条分支对同一段落的合并冲突，记录统一写在 P1 分支
（`agents/feat-m1-port-config`），P2 分支只带 `serial-mux-protocol.md`、`testing.md` 的覆盖行
与探针相关改动。

### P1 配置面（分支 `agents/feat-m1-port-config`，7 提交：4 代码 + 3 文档）

- **落地范围**：`[uart_device <名>]`/`[can_device <名>]` 端口节（`mcu/bridge_ports.rs`，order 19，
  10 测）；端口分组与跨节冲突校验（`mcu/port_grouping.rs`，注册名 `port_groups`，14 测）；
  `[mcu]` 传输键 `uart:`/`can:` + `Transport::BridgePort`（`config/mcu.rs`，+17 测）；共享判定
  核心抽取（`mcu/link_grouping.rs`，host_libraries 与 port_grouping 共用，口径 =
  `host_libraries.rs` 自身 17 测 + 共享核心 `link_grouping.rs` 7 测，断言 0 改动）。装载 id：85 个声明文件 / 124 个（实测重核）。
- **验收实测**：`cargo test --workspace` **3177 passed / 0 failed / 13 ignored**；
  `cargo doc --no-deps -p klipperx` 告警 160→160（0 新增）；`cargo fmt --all` 干净。
- **与 §5 的偏差**：① §5.6(a) 的 i2c/spi 端口写法**未在本期定案**——本期只实现 uart/can，
  i2c/spi 复用已有节类型、其端口引用写法留待接入非 MCU 消费者的期（验收清单本就未含）；
  ② 规则1 的「引用推出的桥与绑的桥不一致」经核查在本配置面**结构上不可达**（桥的唯一声明位点
  就是端口节 `*_mcu`），实现的是可达形态：`*_mcu` 必须存在 + 桥不能同时是下游子设备；
  ③ `canbus_nodeid` 扩到 CAN 端口子（id 覆盖与链路正交，非 CAN 链路仍拒）；
  ④ `uart_restart_method` 本期仅拼写校验，语义随 P3。
- **未收口**：经端口 attach（`Transport::BridgePort::open` 明确报 M1-P3）、识别后与固件
  端口表对账、`uart_restart_method` 语义——均属 §5 P3。

### P2 固件端口表 + 端口级命令 + M2 固件半边（子模组分支 `agents/feat-m1-port-firmware`）

- **落地范围**（子模组前 2 提交）：`SET_PORT_NODEID [0x03][index][id][flags]`（0x03，
  `MUX_PORT_UNITS=0x01`；建表、幂等、无回执、不下发；`bridge_admin_unit` 返回消费标志，
  `mux_unit` 据此短路本机设备半，0x00/0x01 行为逐位不变）；`single_device` 编译期标志退役为
  端口运行期 `flags`（默认 legacy = 现行为，逐点语义反映射核对）；identify 字典端口表
  `BRIDGE_PORT_COUNT`/`BRIDGE_PORT0_NAME`/`_TYPE`/`_PATH` 四键（仅桥构建出现）；复位命令族
  `RESET_DEVICE 0x02 [0x02][index]`（桥只转发到该口；无桥板设备半照 canserial 模式执行）与
  `RESET_PORT 0x04 [0x04][index]`（桥消费，经 `bridge_port_ops.reset` 钩子做 DTR/RTS 线脉冲；
  host 侧用延迟恢复的 poll 状态机保持任务循环非阻塞）。
- **子模组共 4 提交**：前两提交为上述端口命令族与复位族；第三提交 `bdb42f89` 是
  `src/host/README.md` 的端口级命令文档；第四提交 `host: guard the reset pulse against re-entry
  during its hold` 修掉评审发现的锁存缺陷——保持窗内再来一次 `RESET_PORT` 会把已断言的
  DTR\|RTS 读回当「原状态」，释放时恢复断言态、设备被永久按住，现在入口
  `if (bridge_reset_active) return 0;` 把在飞脉冲内的请求按已完成处理。子模组最终 HEAD =
  `2f1635e1`。
- **主仓侧（6 提交）**：1 代码（`serial.rs` 的 `set_port_node_id` builder 与命令字节常量，+1 测）
  + 1 探针测试（`host.rs` 的真库探针 `test_the_bridge_port_commands_over_the_real_library`，+1 测：
  字典端口表可读、legacy 剥头/加头逐字节、UNITS 原样转发、未登记 id 不路由、无回执不下发、
  `RESET_PORT` 连发两次均只消费无输出、`SET_PORT_NODEID` 越界 index 与短载荷静默丢弃、
  库不被误复位）+ 1 文档（`serial-mux-protocol.md` 命令族与字典端口表、`testing.md` 覆盖行）
  + 2 次 gitlink 同步 + 1 评审修复（复位脉冲重入锁存修复与探针负例/常量断言，
  终值 gitlink = 子模组最终 HEAD）。
- **验收实测**：m1-p2 `cargo test --workspace` **3131 passed / 0 failed**（含新固件构建的全部
  字典——即「新固件不破坏既有测试」的回归证明）；三配置（全关/仅 mux/mux+桥）构建基线与改后
  均 **0 告警**；dict 四键在 mux+桥构建、另两配置不含（护栏生效）；非 host 配置
 （stm32f103 等）0 告警（sam4s8c 的 2 条为既有 ARM RWX 段告警，该配置不编译本次改动文件）。
- **与 §5 的偏差**：① 字典端口表暂写死 1 个 host 串口（NAME 字面量 `"uart0"`，未加
  `HOST_BRIDGE_SERIAL_NAME` Kconfig 项——单一 host 串口无变体，留待多端口/其他 arch 后端时
  宏化）；② 探针中断言 `[0x02][0]` 的头形态依赖同测试内先行把端口切到 UNITS（顺序已注释）。
- **存量 flake 已定位并收窄**（分支 `agents/fix-host-library-unload-race`）：`klipper_host_set_uuid()
  returned -1` 的根因是 mux 库路径的测试只在等**裸帧库**的卸载——`wait_for_unload` 写死了
  `library_path()`，而 muxed-attach 那条异步测试干脆没等——于是下一个用 mux 库的测试可能遇上
  一份已 `init` 的镜像（身份必须在 `init` 前给，故 `set_uuid` 答 -1）。现在按**路径**等卸载：
  异步测试用 `wait_for_unload(path)`、同步测试用 `wait_for_unload_blocking(path)`，五条装载真实库
  的测试（`host_tests` 8 测里的 5 条）返回前都等到本路径的 `Weak` 变死，另两条靠同步析构。
  **残余**：`Weak` 变死只等价于「最后一个引用已释放」，镜像的 unmap 在 `HostLibrary::drop` 里
  紧随其后（`dlclose` 在 join 之后）；要让竞态**证到**归零，得让「卸载完成」可观察（测试侧把
  谓词收紧为「间隔一拍仍为死」，或生产侧在 `Drop` 里显式取走镜像后置完成标志）。「真的重新
  映射」的功能探针是 `test_the_library_unloads_with_its_last_reference` 的 `took_over_session`
  断言。
- **实测**：同一检出（主检出、`cargo test --workspace`）——修前 6 次 4 次红（失败总落在 mux 库
  的测试上）、概率上明显高于更早记的「基线 21 次 2 次」（新增的 mux 库探针多了一个暴露面）；
  修后 `host_tests` 单跑 **20/20 绿**，全量闸门 **15/15 绿**（每次均 3179 passed / 0 failed /
  13 ignored）。
- **未收口**：复位层次与桥自身复位留 P3（§5.6d）；主机侧何时发 SET_PORT_NODEID（attach 时序）
  留 P3。
- **真机项已后移**（2026-10-14 拍板，见 §5 P3 的测试策略）：复位极性/保持时长、桥下游 attach
  的双板接线与旧固件板、`RESET_DEVICE` 转发、复位后确认路径——一律不在本期验，等 M1+M2 全部
  落地后统一跑一次真机，清单在 `TESTING.md` 的「M1/M2（桥下游端口 + admin 复位）」一节。

---

## 7. P3 实施记录（attach 走端口 + M2 主机半边）

范围：① 桥下游端口 attach（legacy 与命名两条形态）；② M2 主机半边（经桥复位接进
`restart_method`）；③ 规则 7 的**桥半边**（它是前两条的前置条件）。测试策略见 §5 P3 与
`TESTING.md` 的「M1/M2」一节：真机项一律后移。分支 `agents/feat-m1-p3-attach` 的基底是 `ed2565f`。

### 单元与提交

| 单元 | 提交 | 内容 |
|---|---|---|
| P3-A | `d33f241` | 下游设备（无 admin）**不握手 attach** + 桥 admin 单元入口：`serial::attach_downstream`、`SharedSerialHandle::{send_unit,send_admin,send_set_port_node_id}`、`HostDevice::{load_downstream,send_admin,send_set_port_node_id}` |
| P3-A2 | `1471177` | 已打开句柄上的握手：`SharedSerialHandle::handshake`（与 `attach` 共用 `run_handshake`）、`HostDevice::attach` 开放 |
| P3-B1 | `fd7355a`,`e6f4b4a` | **规则 7 桥半边**（`host_libraries::path_is_mux` 的 `has_bridge`、`serial_ports::is_id_addressed`、`bridge_ports::bridge_mcus` 零 access 预扫）+ 配置驱动 legacy attach（新 `mcu/bridge_attach.rs`：`resolve`/`open`/端口表对账/四类点名错误/`object.rs` 的连接钩子） |
| P3-B2 | `d4a391d` | 命名设备：**先** `SET_PORT_NODEID(index, id, MUX_PORT_UNITS)`、**再** `SET_KLIPPER_NODEID` + 重确认（§4.5 的「两条都要发」） |
| P3-E1 | `528b8ff` | 夹具 `DownstreamResponder`（真 `.dict` 装的假固件 + pty 双向泵）：经真桥跑通 identify（101×40B + 终止块、解压后与 `linuxprocess.dict` 逐字节相等）与一次 `get_clock` 往返 |
| P3-D | `344886c` | M2 主机半边：`McuRestartMethod::Admin`、`BridgeRestart::{Device,Port}`、`restart::reset_bridge_firmware`/`device_reset`、`uart_restart_method` 真语义（端口侧只认 `arduino`/`cheetah`） |
| 集成 | `54f0f91`,`b381ba4`,`6d22e1e` | 合流修复（B2 新增 `BridgePortLink.named` 与 D 的手搭 link 的语义冲突）、D13（**设备复位先重申路由**）、host 库**写锁**（debug 构建逐字节写会让两个引用的单元交错）、接口层诚实性（三个桥命令拒绝**裸引用**）、两 lane 复审 findings 收口 |

### 验收实测（分支 HEAD `6d22e1e`）

- `cargo test --workspace` → **3220 passed / 0 failed / 14 ignored**；`cargo fmt --all --check` 干净；
  `cargo build --workspace --all-targets` **0 告警**。
- 覆盖计数（`cargo test --lib -- --list` 逐模块数）：`mcu/bridge_attach.rs` **14**、
  `mcu/restart.rs` **14**（其中 1 条 `#[ignore]`，见缺陷②）、`devices/host.rs` **11**、
  `devices/serial.rs` **53**、`mcu/host_libraries.rs` **20**、`mcu/serial_ports.rs` **19**、
  `config/mcu.rs` **75**。

### 决断台账（D0–D14）

| # | 决断 | 理由 | 被否方案 |
|---|---|---|---|
| D0 | M1/M2 真机项**后移**，先让代码在软件侧说得通 | 用户拍板；清单落 `TESTING.md`（M1-1..M2-3） | 逐期等真机 —— 会空转 |
| D1 | 下游设备**通过既有共享资源单例自开句柄**，不把桥的 `Interface` 到处传 | `port_for`/`library_registry` 已保证「一条链路一个资源、句柄按 id 分」 | 给 `Mcu`/`McuObject` 加原始单元发送口 —— 改动面更大 |
| D2 | attach **顺序**：先建路由、再设备握手；桥必须先前连（顺序不对→点名错误，不自愈） | 路由不存在时握手单元会被桥丢掉；`connect` 是串行 await，没有拓扑等待机制 | 在 loader 里加依赖等待 —— 只为这一处引入拓扑排序 |
| D3 | 夹具 = **真 `.dict` 假固件 + 真 pty + 真桥** | 真正要保真的部分（桥路由/剥头/加头、identify、`get_clock`）都是真的 | 手写最小响应器（更假） |
| D4 | 规则 7 桥半边**提前到 P3** | 否则桥在自己的链路上根本不可达（单节无 uuid 会被判裸帧） | 人为加同路径第二节骗分类；或推迟到 P4 —— 两者都让 M1 本期不可用 |
| D5 | 命名设备**两条命令都要发**（桥表 + 设备命名），路由在前 | §4.5；且握手单元要走该路由 | 只发 `SET_KLIPPER_NODEID`（桥不知道往哪送） |
| D6 | M2 配置面：新增 `restart_method: admin`（命名设备经桥转发）；legacy 由**端口节** `uart_restart_method` 决定 | 复位能力属于端口/桥，命名设备的 admin 属于设备 | 把线级方法挂在子节上；或新造一套回执机制 |
| D7 | 无回执的确认 = **沿用「复位后重开+重连」重试路径**，不新造确认 | 三条端口级命令都无回执；重连成功本身就是证据 | 假装有回执 / 轮询桥状态 |
| D8 | 设备复位**先重申路由**（D13 落地为 `device_reset`） | 端口 flags 是桥侧状态：未置位时 `RESET_DEVICE` 会被**剥头**，设备收到裸 KXID 帧（其解析器拒绝）⇒ 复位不会发生；路由命令幂等 | 假定桥还记得 flags |
| D9 | `RESET_PORT` 的观测点在**桥链路**，不在下游端口 | 固件 `bridge.c`：0x04 由桥消费、从不扇出；只有 0x02 会转发 | 我最初任务书写的「下游 pty 收到 0x04」——事实错误，已纠 |
| D10 | 命名字节的复位字节测**后移**并由 main 补 | B2 未合入时 `resolve` 必拒；在 `restart.rs` 复刻端口表扫描会造成两个真源 | 复刻扫描（被否）/ 留 `#[ignore]` 空壳（被否） |
| D11 | host 库写路径**加锁串行化单元** | debug 构建逐字节写；桥自己的引用与下游引用同时写是可达情形 ⇒ 交错会毁两个单元（无回执的 `SET_PORT_NODEID` 静默丢失） | 只在文档里声明 debug 限制 |
| D12 | 三个桥命令**拒绝裸引用**（非 mux 的宿主库/非桥链路） | 文档承诺「不是桥就报错而不是写到没人听的链路上」 | 放宽文档措辞 |
| D13 | 配置驱动的**整链端到端在本期不做**（见欠项） | 单进程内要么两份库（被缺陷②挡），要么一库两读者偷帧（E1 夹具已实测：子设备 identify 请求被重发 ~28 次、回包被桥的引用偷走） | 放宽断言凑绿 / 留 `#[ignore]` |
| D14 | 上游/下游的**两端观测点分开写测试**（桥链路 byte 断言 + 真库端口 byte 断言） | 单条命令的可见面不同（消费 vs 转发） | 只写一条「整链」断言 |

### 发现（本期不修，留档）

1. **两份 host 库同进程无法一起退出**：`host.c:40-46/76-81/104-114` 每次 `klipper_host_init` 都覆盖
   进程级 `SIGUSR1` handler，而 handler 用的是**每库各自**的 `g_shutdown_requested`/`g_main_jmp`，
   `sched.c` 的 `run_tasks` 唯一退出路径就是那次 `longjmp`。⇒ 先卸裸帧库 → **SIGSEGV**（跳进已 unmap
   的映射）；先卸桥库 → `HostLibrary::drop` 的 `join()` **无限挂住**。最小复现：同进程 `load_downstream`
   载入 mux 库 + `load_bytes` 载入裸帧库，不发流量，依次 drop（日志 `/tmp/m1p3fx-t2.log`、`t3.log`）。
   影响面：仅「同进程两份 host 库」的嵌入/测试场景（生产桥拓扑是两块板）。
2. **命名设备的握手在宿主库桥上不释放库**：`HostDevice::attach`/`wait_for_assignment` 之后，即使
   `teardown` + drop 掉全部引用，库在 10s 后仍报「already loaded」。已保留一条
   `#[ignore]` 的复现与其验收测试（`a_named_childs_reset_is_forwarded_to_its_port_whole`）——
   它就是修好后的验收测试。

### 欠项

- **P3 验收：配置驱动的整链端到端**（D13 的结论）。现状：夹具（假固件经真桥答 identify/`get_clock`）
  与配置驱动路由（子设备裸帧确实到端口）**两半**都已实测；缺的是合起来那条。要真做需要
  「下游固件跑**独立进程**」（桥引用与子引用不再是同一输出队列的两个活读者）或真机。
- **命名子节的复位字节测**：D10 的后移项，随 D13 的整链一起补。
- **规则 7 的 0 台下游口径**（F-6）：端口节声明了桥但没有任何 `[mcu] uart:` 引用时，桥的路径仍按
  id 寻址——方向保守（桥本就是 mux 设备）但与规则 7 字面「下游设备数」不同口径，P4 一并重核。
- **桥走 `serial:` + `cheetah`**（F-9）：`is_id_addressed` 让它落共享臂，因此**不再驱 RTS**（有告警）。
  手册已写明。
- **P4/P5/P6**：P4（下游台数泛化）、P5（CAN 上游这三条命令与端口表的帧形）、P6（多站串口）未动。
- **真机项**：`TESTING.md` 的 M1-1..M2-3（复位极性/保持、双板接线、旧固件板、无回执确认）。

---

## 8. 固件对象化（2026-10-15）：端口不再是编译期表，而是配置期的 UART 对象列表

**要求（用户拍板）**：① 串口多路复用不是 host 独有，应是**通用配置项**（其他架构尚未实现接线）；
② 「作为桥设备」也是通用能力，**打开后允许主机把这个 MCU 配置成桥**，且依赖多路复用能力；
③ **波特率（含串口其它配置）不在固件里写死**，配置期由**主机**下发，**失败要显式**；
④ 按 klipper 惯例，**资源够就默认启用**，桥的开关不需要用户设置（真正应用由主机配置）；
⑤ 端口不是「桥专用表」，而是**与桥无关的 UART 对象列表**，端口只是「把某个 UART 对象接给桥」。

### 决断（D15–D19）

| # | 决断 | 理由 | 被否方案 |
|---|---|---|---|
| D15 | 固件新增**配置期 UART 对象**（`config_uart oid=%c baud=%u`），oid 由主机发 | klipper 惯例：外设 oid 配置期分配（`config_stepper`/`config_tmcuart` 同形，`src/tmcuart.c:185`） | 端口继续编译期写死（原状：`BRIDGE_PORT_COUNT/_NAME/_TYPE/_PATH`） |
| D16 | 桥端口 = 主机把某个 UART 对象**接给桥**（`config_bridge_port oid=%c uart=%c`），端口 **index = 声明顺序** | 端口与「是否用于桥」解耦；index 语义（`SET_PORT_NODEID`）不变 | 给桥一张自己声明的端口表；或端口 oid 与 uart oid 合一（桥就不再是「用别人的对象」） |
| D17 | 参数（baud 等）随对象在配置期下发，**失败走 `shutdown()`** ⇒ 主机在配置阶段看到点名错误 | 端口级命令族（0x02/0x03/0x04）刻意无回执，不该被改；`shutdown` 是固件唯一既有的显式失败通道（`src/command.h:52` → `sched.c:310`） | 给 admin 命令补回执（动协议、动两端）；或静默沿用旧 baud |
| D18 | `BRIDGE` 改「资源够就默认具备」：`default y if MUX_SERIAL`（`select MUX` 与 `MUX &&` 会成环，故写单条件，等价） | 照本树 `CANBUS`/`WANT_*` 的 `default y if …` 惯例 | 让它保持 host 独占 select（那样「通用能力」不成立） |
| D19 | `MUX_SERIAL` **不设为 MCU 默认开**：它改变控制台线上格式，与上游 klippy 主机不兼容，必须是刻意选择（host 库由 `HOST_MUX_SERIAL` 选上即可） | 兼容性优先；「资源够」在 host（库）与 MCU（板）含义不同 | MCU 也 `default y if SERIAL`（会破坏所有既有 MCU 固件与 klippy 的互操作） |

### FW-1 落地（子模组 `9b43cc10`/`26987b0a` + 主仓 `56a3830`）

- 固件：新 `src/uart.c`/`uart.h`（`config_uart`、`config_bridge_port`、后端 seam `uart_backend_open`/`uart_backend_port_ops`/`uart_backend_set_port`）；`src/Kconfig` 删 `BRIDGE_PORT_SERIAL_BAUD`、`BRIDGE` 自动开、新增 `UART`/`SERIAL_UART`；`src/host/bridge.c` 重写成 UART 后端（**路径仍是部署事实** `HOST_BRIDGE_SERIAL_PATH`，baud 改配置期）；`src/host/host.c` 的 `klipper_host_init()` **不再因 tty 打不开而失败**；四个 `BRIDGE_PORT_*` 字典键删除。
- 主机：新 `cmd/uart.rs`（`ConfigUart`/`ConfigBridgePort` + `DEFAULT_UART_BAUD = 250_000`）；`bridge_ports.rs` 把两条命令在配置期排进桥的 `ConfigBuilder` 并取端口 `index`（`UartPort::index()/set_index()`）；`object.rs` 每桥一个端口计数器；`bridge_attach.rs` 去掉字典对账，`[can_device]` 端口**点名拒绝**（属 P5）。
- 失败文案（固件 `shutdown`，逐字）：`config_uart: baud must be nonzero`、`… the UART cannot be set to the requested baud`、`… this build's UART is already in use`、`… the UART device could not be opened`、`config_bridge_port: no room for another bridge port`。
- 验收：主仓 `cargo test --workspace` **3220 passed / 0 failed / 14 ignored**、fmt 干净、`--all-targets` 0 告警；子模组 `host`/`host-mux`/`stm32f103-serial` 三配置 `make` rc=0、0 告警。
- 新测试：`bridge_attach::port_sections_take_their_declaration_order`（声明序⇒index 0/1）、`…::a_missing_port_tty_fails_the_configuration`（**配置期显式失败**，含 `MCU 'bridge'` 与 `config_uart` 文案）、`…::a_can_port_is_refused_by_name`、`bridge_ports::the_port_sections_load_and_hold_no_unused_options` 补 `index` 断言。

### 已知面/限制（写手提出，我采纳）

1. `uart_oid` 目前**没有消费者**（index 由声明序给出，oid 名只用于查端口节）——**保留**，作为 MCU 后端落地时「固件 UART 实例名」的挂点；手册要写明「当前只作文档/查节用」。
2. host 后端**一条流一个 UART 对象**：`host_library` 桥最多一个端口（板级后端可有多个）；「两个端口节 ⇒ index 0/1」的验收在**串口桥**路径上做。
3. baud「真的生效」只验到下限（配置期无错 + 端口可用；pty 的 `c_ospeed` 回读不可靠），不可设的波特率会由 `TCSETS2` 失败转成 `UART_OPEN_BAD_BAUD` 的显式 `shutdown`。
4. ~~MCU 侧的 mux 接线是 FW-2 的正身~~ → **FW-2/FW-3 已落地**（见下）。

### FW-2/FW-3 落地（子模组 `4279e45b`、`5266e875`；README `9c144cd9`）

- **FW-2（`4279e45b`）串口 mux 接进 MCU 控制台**：接缝在**架构无关层**——收：`console_task()`
  → `mux_input(receive_buf, &count)`（`src/generic/serial_irq.c:100`，在 `irq_save()` 内读，因 RX ISR
  正往同一缓冲 append）；发：`console_sendf()` 经 `mux_out_header()` 带头、新增 `mux_send()`
  排整单元，两者共用抽出的 `transmit_room()`。**行为与 host 逐字同构**（裸帧原样透传、已指派带头、
  未指派不带头）；admin 通道（扫描应答/采纳 id/冲突 shutdown）都在 elf 里可达（`nm` 证据）。
- **FW-2 顺手修掉的真问题**：`MUX_SERIAL` 原来是**无 prompt 符号**，MCU 上写
  `CONFIG_MUX_SERIAL=y` 会被**静默丢弃**（实测）⇒「通用能力」在 MCU 上根本打不开。现在是正经
  `bool` + `depends on SERIAL` + `select MUX`（无 `SERIAL` 的 linux/ar100/pru 会被拒）；**仍不默认开**，
  理由写进 help：单元格式不是上游 klippy 主机说的格式，必须是刻意选择。
- **FW-3（`5266e875`）MCU 侧身份**：新增 `src/generic/chipid_uuid.h`（`fasthash64` 芯片 id → 6 字节，
  **与 CAN 的 uuid 同一实现同一 seed**，`canserial.c` 改为调它）；5 个有 `chipid.c` 的 arch 在
  `chipid_init` 里按 `CONFIG_MUX_SERIAL` 调 `mux_set_uuid()`（四个文件原有 early-return 补
  `&& !CONFIG_MUX_SERIAL`，否则 mux-only 构建读不到芯片 id）；`mux.c`/`mux.h` 的**协议语义与
  `mux_set_uuid([u8;6])` 未动**（host 仍用它）。`nm` 证据：`chipid.o` 有 `U mux_set_uuid`/`U fasthash64`、
  `mux.o` 有 `T mux_set_uuid`，**非 CAN 固件里不出现 canserial**，host `.so` 里不出现 `fasthash`。
- **构建矩阵（本次实测）**：`stm32f103-serial`（原样 / `+MUX_SERIAL=y`）、`rp2040`(+mux)、
  `atsamd`/`atsam`/`lpc176x`(+mux)、`stm32f103` CAN、`host`、`host-mux` 全部 rc=0；stm32/rp2040
  与 host **0 告警**（samd51/atsam/lpc176x 的告警是 baseline 就有的 `armcm_reset.c` array-bounds 与
  atsam 的 `.ramfunc` RWX，均已对比基线复现）。
- **只能真机验**（已挂 `TESTING.md`）：真板 **扫描 → 指派 → 采纳 → id 冲突 shutdown** 的串口 mux
  端到端；`irq_save()` 段内的 UART 时序。

### 复审（P7）与 FW-4：一个真 blocker 的发现与修复

fresh-context 只读复审对「①–⑤ 都成立」给了逐条证据（端口对象化、参数主机化 + 显式失败、
桥能力自动且不成环、MCU 接线与 host 逐字同构、`chipid_uuid` 派生与 CAN 逐字节等价、
`mux.c` 与基线完全相同），并抓到一个 **blocker**：

- **F1（内存安全）**：`src/generic/serial_irq.c` 的 mux 分支把**剥头前的 `rpos`** 交给帧解析器
  （真实有效长度是 mux 之后的 `count`）。带头正常收发时 `count < rpos` **恒成立**，而
  `command_find_block` 的错误分支不 clamp `pop_count`（`command.c:322-329`），于是
  `console_pop_input` 的 `uint_fast8_t needcopy = rpos - len` 在 `len > rpos` 时**回绕** ⇒
  对 192 字节的 `receive_buf` 越界 `memmove`。**触发条件很轻**：一个尾 SYNC 被噪声打坏的单元。
  host 侧恰恰有这一步（`host/console.c:464-472` 传 mux 后的长度并 clamp）⇒ MCU 这份是**漏了**。
- **修法（FW-4 `a6adb68a`）**：`irq_save()` 段内 `receive_pos = count; rpos = count;`（把不变式
  写进注释）+ `console_pop_input` 入口**防御性 clamp**（`if (len > buffered) len = buffered;`，
  不动 `command.c`）；顺带把 `MUX_SERIAL` 的可用性收紧为 `depends on HAVE_CHIPID || MACH_HOST`
  （avr/hc32f460/simulator 无身份来源 ⇒ 不能选；host 库身份由 `klipper_host_set_uuid` 给）。
  验收：`stm32f103-serial`（±MUX）、`rp2040`+mux、`MACH_HOST`（±`HOST_MUX_SERIAL`）全 rc=0、
  0 告警；对 20 份 `test/configs/*.config` 跑 `olddefconfig` 无 kconfig 警告。
- 复审的其余 finding 由 main 当场修掉（`8bdf79f`）：协议页端口表段退役、`TESTING.md` M1-3 改写、
  `host.rs` 五处过时注释与四键断言。
- **MCU 侧仍只能构建 + 不变式论证**（无运行期驱动）⇒ 真板验证照旧挂 `TESTING.md`。
  `scripts/check_whitespace.sh` 在本子模组**基线就不干净**（host/ 下多处 >80 列），P7 新引入的三处
  已顺手修掉（`6a418e63`），其余属既有面、不在本期范围。

### P8（2026-10-15）：真机实测暴露的宿主库读端缺陷 + 修复

**真机场景**（用户提供：`/dev/serial/by-id/usb-Klipper_stm32f103xe_39FFD7054D47323924610951-if00`
= `/dev/ttyACM0`，一台**旧固件**板子）：宿主库当桥（以其 tty 路径构建、`CONFIG_BRIDGE`/`MUX_SERIAL`
/`UART` 自动具备），配置 `[mcu bridge] host_library:` + `[uart_device uart0]` + `[mcu child] uart: uart0`
（legacy 下游）。

- **修前**：桥起来了（`identified: 22 messages`、端口按配置期 baud 在真板上打开），但下游
  `identify_response` 超时、桥自己的 `clock` 也超时，日志出现 `klipper sent 17/10 bytes under a mux
  header that are not a frame`；45s 内该类告警/超时 **54 条**。
- **对照**：① 同一块板子**直连**串口 ⇒ `identified: 135 messages registered` ⇒ 板子/旧固件无辜；
  ② **只起桥**（去掉下游）⇒ 25s 内 **0 条**告警/超时。
- **根因**：`interface/devices/host.rs` 里每个 `HostDevice` 引用各自 `recv_timeout` 读**同一份库输出**
  （`output.clone()`），`receive_mux` 把不属于自己 id 的单元 `continue` **丢掉** ⇒ 两个活引用把字节流
  瓜分、谁都拼不齐帧（10/17 字节是「半截流」的痕迹）。共享串口那侧本来是**单读者 + 按 id 投槽**
  （`SharedSerialPort`/`McuSlot`/`push_frame`），宿主库缺这层。
- **修复（`7ae15c7`）**：`Router`（槽表 + 模式 + 路径）挂在 `HostLibrary` 上，`start_reader` 起**每库
  一个**读者（两种形态共用：帧 API 与字节流），`reader_loop` 在 mux 模式喂 `UnitStream` →
  `take_unit_with_arb` → `route_unit`（按 `assign::mcu_id_from_arb`）→ `push_unit` 投槽；admin id 丢弃
  （与 `serial.rs` 的 `dispatch` 同策略）；引用侧只读自己的槽，旧的 id 过滤分支删除；裸路径语义不变。
  读者**不持有**库的强引用（避免「最后一个 Drop 落在读者线程 ⇒ `pthread_join(self)` ⇒ panic」）。
- **真机复跑（修后，同一条链路）**：桥 `identified: 22 messages` → 端口打开 → **下游 `identified: 135
  messages`**（与直连一致），随后双机 stats 稳定，**40s 内 0 条**告警/超时、0 错误。⇒ **D13 欠项
  「配置驱动的整链端到端」在真机上兑现**（此前记为「单进程内做不到」，根因其实是这个缺陷）。
- 验收：`cargo test --workspace` **3221 passed / 0 failed / 14 ignored**（两遍）；新测
  `test_two_live_references_on_a_shared_library_do_not_steal_each_others_units`（两条引用交错收，
  各自只拿到自己的两帧且有序）单跑 3 次绿、回到旧行为即红；fmt 干净；`--all-targets` 0 告警；
  `cargo check --release -p klipperx --lib` 覆盖帧 API 分支。
- 复审（fresh context）：**可合入**（代码无 blocker），只要求三处手册同步（已由 main 一起改：
  `serial-mux-protocol.md` 的「单进程做不到」段、`developer-manual/README.md` 的 `host.rs` 行、
  `testing.md` 的 11→12 测与清单）；另报两处**既有**缺陷（下）。

### 新发现的既有缺陷（报而不改，待排期）

1. **`serial.rs` 的读者可能在读者线程上 join 自己**：`reader_loop` 在 `weak.upgrade()`（`serial.rs:1467`）
   到 `drop(port)`（`:1471`）之间短暂持强引用；若此刻最后一个 `SharedSerialHandle` 析构，`SharedSerialPort::drop`
   （`:1440-1452` 的 `handle.join()`）会跑到读者线程上 ⇒ `pthread_join(self)` = `EDEADLK` ⇒ std panic。
   最小修法（复审给出）：`Drop` 里加自 join 守卫（比 `std::thread::current().id()`）。**窗口小但真实**
   （链路繁忙时 `dispatch` 占的时间就是窗口）。
2. **`SharedSerialPort.slots` 只在握手失败时移除**（`serial.rs:1288`）：句柄析构后槽仍在表里，
   `push_frame` 会一直往无人读的槽里堆字节（host 半无此问题：引用析构后 `SEND` 失败即丢）。
3. 复审顺带修正：P8 写手把两条 plain-library 测试的 flake 归到缺陷 1**不成立**（那是 host 侧代码），
   真实首因仍待有界复现——候选是 `library_unloaded()` 只观察 `Weak`、不等 `dlclose` 尾巴（`host.rs:1511-1516`）。
