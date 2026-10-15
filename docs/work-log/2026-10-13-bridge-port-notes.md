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

1. **两份 host 库同进程无法一起退出**（**已按用户拍板改成「一进程一份库」的明确规则并落成守卫**，见下「L1」一节；它**不是** M1 的缺陷——M1 的产品拓扑最多一份库：桥是一块板，或桥是唯一那份库 + 真板下游）。

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

**真机场景**（用户提供：`/dev/serial/by-id/usb-Klipper_stm32f103xe_…-if00`
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

### P9（2026-10-15）：共享串口两处既有缺陷修复 + 一个真机结论

**P9 修复（`cd85007`）**——上一轮复审报出的两处既有缺陷：

1. **读者可能在读者线程上 join 自己**：`reader_loop` 在 `weak.upgrade()`→`drop(port)` 之间短暂持强引用，
   若此刻最后一个 `SharedSerialHandle` 析构，`SharedSerialPort::drop` 的 `handle.join()` 就跑在 reader
   线程上 ⇒ `pthread_join(self)` = `EDEADLK` ⇒ std panic。**修法**：`Drop` 里自 join 守卫
   （`thread::current().id()` 比自己就跳过，`JoinHandle` 被 detach），reader 随后在下一轮
   `weak.upgrade()` 拿到 `None` 自然退出（不依赖 `stopped`、不持锁、不永活）。写手给了**确定性反证**：
   持住目标槽的 `incoming` 锁把读者钉在 `dispatch` 里 ⇒ 必然命中窗口；临时改成无条件 join 即复现
   `failed to join thread: Resource deadlock avoided (os error 35)`。
2. **槽不回收**：`slots` 只在 `attach` 握手失败时移除，句柄正常析构后槽仍在、`push_frame` 一直往无人读的槽
   堆字节。**修法**：新增 `impl Drop for SharedSerialHandle`，锁下用 `Arc::ptr_eq` 只移除**本句柄**的槽
   （与 attach 的 insert、失败路径的 remove 都不打架）；新测断言「析构后该 id 槽消失、同 id 重 attach
   无陈旧积压、之后的新单元照收」。

验收：`cargo test --workspace` **3223 passed / 0 failed / 14 ignored**；fmt 干净；`--all-targets` 0 告警；
复审（fresh context）**可合入**，要求的三处手册数字/措辞（`testing.md` 41/8/53 → **43/10/55**、
dev-manual `README.md` 与 `serial-mux-protocol.md` 的引用计数关闭段）已由 main 同批补上。
复审另报三条**report-only**（`shutdown()` 不注销槽 ⇒ shutdown-未析构期间该槽无界增长；`attach` 失败路径
无条件 remove 与 Drop 的 ptr_eq 不对称；`SharedSerialHandle::shutdown` 无测试）——建议合并成一个小单元，
把注销统一到 ptr_eq 守卫的 `unregister()` 并补 shutdown 路径测试。

**真机结论（DTR/RTS）**——回答「这块板是 USB 虚拟串口，DTR/RTS 到底有没有被处理」：

- 这台 `usb-Klipper_stm32f103xe_…-if00` 是**原生 USB CDC**（klipper 自带 USB 栈）。
- 固件侧（`third_party/klipper/src/generic/usb_cdc.c`）：`USB_CDC_REQ_SET_CONTROL_LINE_STATE` →
  `usb_req_set_line()`（`:485-494`）**只是把 `wValue` 记进 `line_control_state` 并 ACK**，不做复位；
  唯一消费者是 `check_reboot()`（`:452-459`）：**波特率 1200 且 DTR 为低**时才 `bootloader_request()`
  （需 `CONFIG_HAVE_BOOTLOADER_REQUEST`）。
- ⇒ **DTR\|RTS 在 250000 下的 250ms 脉冲对这类板子静默无效**（内核照常接受 `TIOCMSET`，桥也拿不到任何反馈）
  ⇒ `RESET_PORT` 的线级复位**只对 USB-串口桥接板**（DTR\|RTS 接复位脚）有意义；
  **原生 USB 的 legacy 设备目前没有经桥的硬复位手段**（要走命名设备 `restart_method: admin` 的
  `RESET_DEVICE`，或断电/`rpi_usb`）。
- 已落文档：`config.md` 的 `uart_restart_method` 行、`serial-mux-protocol.md` 的 `RESET_PORT` 行、
  `TESTING.md` 的 **M2-1** 判定（该项在原生 USB 板上判「不适用」，其真机验证只对桥接板有意义）。
- **留下的设计缺口**（记为欠项）：既没有「CDC 感知」的端口复位（例如把端口切到 1200 + DTR 低进 bootloader、
  或明确报「此端口不支持线级复位」），也没有按 `by-id` 命名识别原生 USB 并给出告警。

### P10（2026-10-15）：注销槽位收敛 + 用户否掉的方案

- **P10（`e5c91b7`）**：把「注销槽位」收敛成一个私有、`Arc::ptr_eq` 守卫的 `unregister()`
  （`serial.rs:1811`），三处共用：`attach` 失败路径（原为无条件 `remove`）、`Drop for SharedSerialHandle`、
  **`SharedSerialHandle::shutdown()`（新增）**。`shutdown` 原先只置 `stopped`，于是「已 shutdown 但句柄未析构」
  期间 `push_frame` 仍往无人读的槽 `Vec::extend`（`FrameStream` 无界）⇒ 现在找不到槽即丢帧，与 host 侧
  「Sender 失败即丢」同构。`receive` 仍先交付积压再返 `None`；`handshake` 失败**不**注销（与 attach 失败相反）
  的既有区分不变。验收：`--workspace` **3224 passed / 0 failed / 14 ignored**、fmt 干净、0 告警；复审**可合入**。
- 复审要求的文档同步（同批）：`serial-mux-protocol.md` 与 dev-manual `README.md` 的「引用计数关闭」段
  （`shutdown` 与析构**都**注销槽）、`testing.md` 计数 43/10/55 → **44/11/56** 与新测清单。
- 复审另报两条「加守卫后才可达」的小洞，我已同批补：`slots` 字段注释（注销主体）、`handshake` 文档的
  绝对表述，并**新增守卫分支的判别性测试** `a_stale_handle_leaves_the_newer_attachs_slot_alone`
  （同 id 二次 attach ⇒ 旧句柄 `shutdown`/`Drop` 不得摘掉新槽；**去掉守卫即红**，已实测）。
- **被否方案（用户 2026-10-15 明确否掉）**：CDC 感知的端口复位（把端口切到 1200 + DTR 低以进 bootloader，
  或显式报「此端口不支持线级复位」）与按 `by-id` 命名（`usb-Klipper_*` = 原生 USB）给出告警——**不改**。
  结论仍保留在手册里（`config.md` 的 `uart_restart_method`、`serial-mux-protocol.md` 的 `RESET_PORT`、
  `TESTING.md` M2-1），供后来人参考；实现缺口本身作为**已知缺口**留档：原生 USB 的 legacy 设备目前没有
  经桥的硬复位手段（只能命名设备 `restart_method: admin` 或断电）。

### P11（2026-10-15）：`#[ignore]` 那条验收测——**推翻原诊断**，真因是测试的运行时口径

**原诊断（错）**：以为是「命名设备在宿主库桥上握手后库不卸载」（缺陷②），并据此挂了 `#[ignore]`。
**P11 的诊断（对）**：与「命名」无关，真正变量是 **tokio 运行时口径**：

- 桥自己的两个传输任务（`Mcu::from_parts` 的 `interface_for_send`/`interface_for_recv`，
  `mcu/mod.rs:1669`/`:1920`）各持一个 `Interface` 克隆 ⇒ `Arc<HostDevice>` ⇒ `Arc<HostLibrary>`。
- `Mcu::close`（`mcu/mod.rs:2867`）会 abort 这些任务，但 `abort()` 只在运行时**下次跑任务时**才生效；
  而那条测试的 `wait_for_mux_library_unload` 是**阻塞式**等待（`std::thread::sleep` + `HostDevice::load_bytes`），
  在**current-thread** 运行时上它把运行时线程占住 ⇒ 被 abort 的任务永远没机会释放 `Arc<HostDevice>`，
  库的最后一个引用只能等到**测试结束、运行时析构**时才掉。生产用 `new_multi_thread`（`src/klippy.rs:292`），
  而**同族的 legacy 测试本来就是 `flavor = "multi_thread"`**（`restart.rs:886`）——这就是它「通过」、命名那条
  「不通过」的真实差别（不是命名、更不是 P8 的读者）。
- **插桩证据**（临时 `HostDevice`/`HostLibrary` 计数，已还原）：current-thread 下 `teardown`+`drop(printer)`
  之后 `MCU DROP bridge` 触发但 ref=1（桥的任务）仍在、探针 10s 超时；加一次运行时 poll（临时 `sleep(500ms).await`）
  即卸载；`flavor = "multi_thread"` 直接通过（×3）。下游句柄（`load_downstream`）与复位临时句柄
  （`open_bridge_link`）**每次都干净释放** ⇒ `attach`/`wait_for_assignment`/`open_bridge_link` **不泄漏**。

**修法（`ae6a7af`，只动测试）**：`#[tokio::test]` → `#[tokio::test(flavor = "multi_thread")]`、
**解除 `#[ignore]`**、把测试注释里那段错解释换成真解释。**反证**：把属性改回 `#[tokio::test]` 即复现
`the mux host library never unloaded`；改回 multi_thread 即绿。生产语义无需改动。
验收：`cargo test --workspace` **3226 passed / 0 failed / 13 ignored**（ignored 14→13）、
`mcu::restart::tests` 14 passed / 0 ignored、fmt 干净、`--all-targets` 0 告警。

**可迁移的教训**（已写进 `testing.md` 的对应行）：**测试里若用阻塞式等待去观察「被 abort 的任务应释放的资源」，
必须用 `multi_thread` 运行时**——current-thread 下运行时线程被阻塞，abort 永远不生效；这类症状会被
误读成「资源生命周期缺陷」（我们这次就误读了）。

**仍然开着的另一条（与本次无关）**：缺陷①「两份 host 库同进程无法一起退出」（`host.c` 进程级 SIGUSR1 与
`sched.c` 的 longjmp 退出路径），本轮未动。

### L1（2026-10-15）：一进程一份 host 库（规则 + 两道守卫）

用户拍板：**明确 host 只能存在一份**；需要两个固件时改用**同一份库的多引用（mux）**或**独立进程**（固件侧 `linux` 目标可独立运行）。

**落地（`f85c1cc`，主仓）**：

- **配置层**：`HostLibraries::reject_a_second_path`（`mcu/host_libraries.rs:221`）在 `validate`（`:263-264`，由 `[mcu]` 的 host 传输解析调用，`config/mcu.rs:687`）里点名拒绝「≥2 条不同 `host_library:` 路径」。
- **运行期**：`refuse_a_second_library`（`interface/devices/host.rs:213`）在 `library_for`（`:183`）最前面按**路径**守卫——读的是既有注册表的 `Weak`（死条目永不被当成活库，**不引入第二个静态表**）；同路径复用（mux/下游引用）不受影响，**先后载入**（先卸 A 再载 B）照旧允许。
- **文案（逐字，点名两条路径 + 原因）**：配置层 `host library: one host library per process, but the config names '<a>', '<b>'. klipper stops its main loop through a process-wide SIGUSR1 handler that each library installs over the last one, so two libraries in one process meet either a jump into unloaded code (a crash) or a join whose signal another library consumed (a hang); name one path from several sections, or run the firmware images in separate processes`；运行期同口径、把 `<requested>`/`<loaded>` 换进去。
- **测试**（3 条，判别性）：`host_libraries::validate_rejects_a_second_host_library_path`、`config::mcu::a_config_with_two_host_library_paths_is_rejected`、`host::host_tests::test_a_second_host_library_path_is_refused_while_one_is_live`（同路径两引用**仍装载**→不同路径**被拒**→卸掉 A 后 B **可载**）。
  **反证**：去掉两道守卫 ⇒ 三条新测全红，且运行期那条**进程直接 SIGSEGV**（正是缺陷本身的表现）。
- 既有测试必要调整 1 处（已说明）：`test_load_reports_a_missing_library` 现在也取 `HOST_LIBRARY_TEST_LOCK`——装载现在会查进程级注册表，否则「载不存在的路径」会与别的测试的活库竞速、报成「一进程一份库」而不是「failed to load」。
- 验收：`cargo test --workspace` **3229 passed / 0 failed / 13 ignored**、fmt 干净、`--all-targets` 0 告警；「不误伤」面（同路径 mux、两条引用共用一份库、先后载入、多路径身份互异）全部既有测试绿。

### R1/R1b（2026-10-15）：桥端口 tty 改为**运行期**由主机按端口槽登记

**动机**：真机那次为了指到板子的 tty，专门重建了一份库（把 by-id 路径烘进 Kconfig）。改成一份库 + 运行期给路径。

**R1（子模组 `a96368d9` + 主仓 `e13b937`）**

- 固件：`int klipper_host_port(int slot, const char *path)`（`init` 前调用，槽数 `KLIPPER_HOST_PORT_SLOTS = 4`）、内部 `klipper_host_port_path(slot)`；`config_uart oid=%c baud=%u slot=%c`；
  后端 `src/host/bridge.c` 按槽取路径、**没登记就回落** `CONFIG_HOST_BRIDGE_SERIAL_PATH`（该 Kconfig 从「唯一事实」降级为**兜底默认**）；仍**只服务一路流**（第二路 `UART_OPEN_BUSY`）。
- 主机：`[uart_device] uart_path:`（字符串，host-only）；桥那台 `[mcu]` 在 `klipper_host_init` **之前**把各端口 `(slot=端口 index, path)` 交给库——`slot`/`index`/`SET_PORT_NODEID` **一处计数**（`bridge_ports::wire_bridge_port`）。非宿主库桥写 `uart_path:` ⇒ 装载期点名报错。
- 验收：主仓 **3237 passed / 0 failed / 13 ignored**、fmt 干净、0 告警；子模组 `host`/`host-mux` 0 告警；反证（后端改回只读 Kconfig）两条新测均红。
- 最有判别力的一条测试：**把构建默认路径做成活 pty**，仍然失败 ⇒ 证明采纳的是运行期路径（并断言库 stderr 含用户写的那条路径）。

**R1b（子模组 `573c3eaf`/`5dbd8d0c` + 主仓 `8769f4a`）**——收 R1 复审 findings：

1. **P1 缺口（已修）**：同一条 `host_library:` 路径上若有**非桥成员先连接**（`host_libraries` 允许的 mux 组合），首个 `load_mux` 会带**空表**装载并 `init`，桥的 `(slot,path)` 再也登不进去 ⇒ **静默回落**。修法：`HostLibrary` 记下开库时的端口表，`library_for` 复用活库时**表不一致（且本次非空）就点名报错**（`klipper_host_port() 必须在 init 前调用`）。**不做**「表按路径合并」——复审论证那会**制造**新的静默错路径（固件槽表是全局的，槽号却是每颗桥自己的端口序，两条桥求并集会「同槽两值」后者覆盖前者）。附带收益：上一个测试泄漏的活库被下一条测试复用时，身份/模式相同、只有表能识破 ⇒ 从「怪异失败」变成点名错误。
2. **P2（已修）**：桥测试的收尾改 `BridgeTeardown` RAII guard（`teardown → drop printer → 等卸载 → 清 symlink`）——R1 那次 240s 挂起的真因就是**断言在 `teardown()` 前 panic**、机器与已加载库被泄漏、后续测试静默复用陈旧固件；现在 panic 展开也收干净（写手用「临时 panic」验证过：该测红、其余 15 条全绿）。
3. **P2（已修）**：`BRIDGE_MAX_PORTS`（generic `src/bridge.c:58`）与 `KLIPPER_HOST_PORT_SLOTS`（`klipper_host.h`）加**编译期 `#error` 绑定**（`#if CONFIG_MACH_HOST` 守卫，别把 host 头拖进真板构建）；槽越界/路径过长的文案补说明。
4. **P2（已做）**：线上参数名 `path=%c` → **`slot=%c`**（它承载的是槽号，避免误导下一个读协议的人）。
- 验收：主仓 **3238 passed / 0 failed / 13 ignored**、fmt 干净、0 告警；子模组两配置 0 告警；三条反证（编译期 `#error` 触发、表检查去掉即红、RAII 收尾的 panic 探针）。

**仍然记着的事实（不是缺陷）**：`config_uart` 多了一个参数 ⇒ **新主机配旧库**在连接期**显式报错**（按字典校验参数个数：`expected 2 parameters, got 3`），只是文案不友好、没有版本握手；`KLIPPER_HOST_PORT_SLOTS = 4` 与 `BRIDGE_MAX_PORTS = 4` 现已编译期绑定；~~固件仍单流 ⇒ `slot != 0` 的端到端路径不可能被测到，N 路并行流是下一个单元（R2）~~（**已被 §9 的 R2 推翻：多流已落地并双端口整链验收**）。

## 9. R2/L2（2026-10-15）：host 后端多路流 + 真 linux 固件当下游

### 决断（D20–D23）

- **D20（R2-b 排队）**：双端口端到端测试**排在 L2-b 之后**串行——L2-b 是同一条链 ×1 的先行单元，复用其布线可避免重复踩坑、`mod.rs`/辅助代码写冲突与双份 CPU 争用（两单元本身无代码依赖，但有知识依赖）。
- **D21（夹具落点）**：`LinuxFirmware` 夹具放 **dev-dep `klipperx-test-support`**（build.rs 暴露 elf + RAII spawn/收割），而非内联进单个测试——L2-b 与 R2-b 两测试共用，`env!` 经 test-support 自身 build.rs 传递正好绕开 dev-dep `env!` 不可见的限制。
- **D22（换棒策略）**：子代理被 provider `content_filter` 误杀或反复超时**零产出**时，先分析会话状态再决定 resume 还是 fresh 重开：会话历史大（>66k 输入）且两次被过滤 → 放弃续跑、fresh context 重开；上下文已就位但侦察过度 → 同会话 resume 并强制落笔。
- **D23（防超时三板斧）**：任务书直接写入「立即落笔、10 分钟内必须首提交、每步增量提交、只读错误点名的文件」——L2-b 第四次尝试（resume + 强制三步）用 30 分钟内 2 个提交的节奏完成了前三次 30 分钟都写不出一个函数的任务。

### L2-a 夹具（主仓 `bc48281` + 文档 `17ac58d`，合入 `534f279`）

- `build.rs` 断言 `linuxprocess` 的 `out/klipper.elf` 并以 `KLIPPER_LINUX_FIRMWARE` 暴露；`linux_firmware_path()` + `LinuxFirmware`（`-I` 独立临时路径、以「软链指向 `/dev/pts/*`」为就绪信号（≤5s 有界，失败带固件 stderr 尾部）、Drop 必定 kill+有界收尸+删目录；夹具从不向固件发字节（`config_reset` 会 `execv`）。
- 自测试 `linux_firmware_spawn_and_drop` 两跑 0.04s；残留自查 pgrep 无匹配。
- provider 曾把该子任务首跑**内容过滤误杀**（`finish_reason: content_filter`，零文件改动），resume 后一次通过——**不是任务问题**。

### L2-b 真固件整链（`eff45af`/`c1ab57f`/文档 `97195d1`，合入 `5ac33af`）

- 新测 `a_real_linux_firmware_completes_identify_and_clock_behind_the_bridge_port`：真 `klipper.elf` 独立进程当 legacy 下游，`uart_path:` 直指固件自己的 pty slave（R1 语义），bring_up **完整**跑到 Ready（E1 是有界中断）→ identify 快照 + `CLOCK_FREQ==50_000_000`（与 mux 库 20 MHz 可区分 ⇒ 数据来自真固件）→ 两次 `get_clock` 按 32 位回绕差值断言推进 → 按 pid 确认固件被收割。两跑 0.81s。
- **坑**：`McuClock` 与 `Arc<Mcu>` 持 child interface 拖住 mux 库引用，teardown 不先显式 drop 就报「库 never unloaded」（与 P11 同型教训的具象化）。
- **执行教训**：前三次尝试（1 次 content_filter、2 次 30 分钟超时）全部零产出，根因是**侦察无限挤占预算**；第 4 次 resume 强制「落笔→增量提交→按错误点名修复」节奏后一次通过。

### R2-a host 后端按槽多流（子模组 `49ca236f`；主仓 gitlink+手册 `5a723c6`）

- `src/host/bridge.c`：全局单流（`bridge_fd/bridge_port/bridge_tx/bridge_reset_*`）→ `struct bridge_stream[KLIPPER_HOST_PORT_SLOTS]` 每槽一条（fd、owner、router index、整帧 tx 队列、reset 状态机）；hooks 经 ctx=`uart_s*` 按 `owner` 回查自己的槽；**开放判据用 owner 而非 fd>=0**（静态表零初始化 fd==0，用 fd 判会让首槽永远 BUSY——写手自查发现并修掉的唯一实质 bug）；同槽二次 open 才 `UART_OPEN_BUSY`（uart.c 文案改 `the port slot is already in use`）、越界显式拒绝不走回落、失败不留半开、`bridge_poll` 遍历全部；C 注释旧单流断言清零。
- 评审 12 条：代码全过、无 P0；**修复后可合入** → 合入前补修 `Kconfig:86` 与 `README:198` 两处旧单流断言 + `internal.h`/`timer.c`/`bridge.c` 注释微偏（amend 进单提交 `49ca236f`）；评审环境无 shell，文件清单由 main 用 `git show --stat` 闭合（仅 bridge.c/uart.c/uart.h + 4 处文档注释）。
- 三配置构建 0 告警（plain host / mux host / stm32f103，产物全在 worktree 自有 `out/`）；合入后 mux 配置复跑 0 告警。

### R2-b 双端口端到端（`1000766` + 文档 `85e98bc`，合入 `ce48af0`）

- 新测 `two_real_linux_firmware_ports_come_up_on_one_bridge`：一份 mux 库 + 两个 `[uart_device]`（声明顺序即 slot、互异 `uart_path` 各指一个真固件进程）+ 两个 legacy child；**双口 Ready 即多流主证明**（旧单流后端第二个 `config_uart` 会 `UART_OPEN_BUSY` 到不了 Ready）；库 stderr 断言两行 `… (fd N, slot 0/1)` 直证声明顺序→槽位；双 child 各自 identify + `CLOCK_FREQ` + 回绕差值 `get_clock`；双 pid 有界收割。两跑 1.32s，main 复跑同绿。
- 执行上一次通过（模板双份化 + 防超时策略生效）。

### 本段验收

- 主仓 `work` = `ce48af0`：全量闸门 **3241 passed / 0 failed / 13 ignored**、`cargo fmt --all --check` 干净、`--all-targets` 0 告警；工作区干净；两条任务 worktree 已全部销毁。
- 手册同步（随代码）：`testing.md`（夹具小节 + bridge_attach 18 测两条新覆盖）、`serial-mux-protocol.md`（配置期按槽多流语义 + 「多下游端口」段改为「端到端已钉住」）、子模组 `README.md`/`Kconfig` 旧断言清理。
- 两测试的独立评审均已完成：L2-b **OK（P0/P1 无，三条 P2 当场跟进进 `2690ded`）**；R2-b **无须返工（P2 三条：串行 get_clock 措辞、251M 余量、文件清单——均已在后续提交闭合）**；L3（linux console 接 mux + `-U` 身份 → 免真机验命名设备经真桥）与 P4/P5/P6 仍未动。

## 10. L3/P4/P5（2026-10-15，同一轮）：linux 当命名下游、规则 7 计数、CAN 帧形

### 决断（D24–D27）

- **D24（L3 就绪：can“命名设备经真桥”从真机项降为软件可验）**：给 linux 进程控制台接 mux（`src/linux/console.c` 收/发两个接点 + 自己的有界 `mux_send`）并新增 `-U <12 hex>` 身份来源（`src/linux/main.c`，在 `console_setup()` 前 `mux_set_uuid`），Kconfig 把 `MUX_SERIAL` 身份依赖放宽为 `HAVE_CHIPID || MACH_HOST || MACH_LINUX`（linux 新增 `config SERIAL default y` 仅本机型生效）。于是 M1-2 的“命名设备经真桥”有了**真固件独立进程**的整链测试；真板项（扫描/指派/冲突、`irq_save()` 时序、真桥上的双设备冲突）仍属真机。
- **D25（新配置不污染 legacy 固件）**：新增 `test/configs/linuxprocess-mux.config`（`MACH_LINUX + MUX_SERIAL`），**不动** `linuxprocess.config`——后者是 L2-b/R2-b 夹具的字节基准；两份 elf 分别以 `KLIPPER_LINUX_FIRMWARE` / `KLIPPER_LINUX_MUX_FIRMWARE` 暴露，夹具新增 `spawn_named(uuid)`（现有 `spawn()` 逐字不变）。
- **D26（F-6 裁决：规则 7 计入下游台数，但 0 台下游仍按 id 寻址）**：`is_id_addressed = 共享 ∨ 下游设备数 ≥ 1 ∨ 是桥`（`host_libraries::path_is_mux` 同口径）。保留 bridge 项的理由是**机制而非保守**：桥/宿主库只要被 mux 化就有身份，`mux_out_header` 会给它的输出加帧头；若因“0 台下游”判成非 id 寻址，主机会用裸帧去解带身份 mux 设备的输出 ⇒ 解析错位。字面计数项作为**独立且可审计**的判据加入，台数由 `bridge_ports::bridge_downstream_counts` 零 access 预扫提供（`downstream_devices(path)` 可查）；因 `下游 ≥ 1 ⇒ 是桥`，此项在现实中不可观测地改变行为——价值 = 字面判据 + 可查计数 + 文档（评审确认）。
- **D27（P5 帧形：CAN 上游走 mux 单元，候选 B）**：桥经 CAN 连接时，三条端口级命令与下游设备数据**都作为 mux `KXID` 单元跑在字节流里**（经典 CAN 8 字节切帧由 `CanSerialLink`/`canserial.c` 承担），新增 `CONFIG_MUX_CAN`（与 `MUX_SERIAL` 同形，`BRIDGE` 改为 `default y if MUX_SERIAL || MUX_CAN`）。**候选 A（CAN 单帧 admin 命令）被否**：① 它只能承载三条命令、无法承载下游设备数据（N 台下游复用到一条字节流，无头就分不清归属——正是规则 7 的本质）；② `0x02` 与 CAN 既有 `REQUEST_BOOTLOADER` 载荷位冲突，需另造帧形。候选 B 零新帧形、零冲突（单帧 admin 通道 `0x3f0` 与字节流内头字段是两条通道），且串口/CAN/host 库三种上游统一为一套单元。

### L3 落地（子模组 `77019792` + 评审跟进 `628438d2`；主仓 gitlink/手册 `158dd1c`；测试 `afa3dec`）

- 子模组：Kconfig（linux `SERIAL default y`、身份依赖放宽、注释/help 同步）、`src/linux/console.c`（读后先 `mux_input`、发端预留 `MUX_HEADER_BYTES` + `mux_out_header`、`mux_send` 走 4KB 有界整单元队列，满时丢新单元并报 stderr）、`src/linux/main.c`（`-U`，非法即 usage + `-1`，`#if` 守卫写法保证非 mux 构建逐字回退）、新配置。构建三例零告警；worker 自建探针证明 `-U` 三态（指定/大写/缺省）应答 uuid 正确。
- 主仓测试（`afa3dec`）：`a_named_real_linux_firmware_completes_handshake_behind_the_bridge_port`（两跑 1.28s/0.74s，main 复跑 0.93s 绿）——身份常量一处定义、同时喂固件 `-U` 与配置 `uuid:`（防漂移）；断言含库 stderr `open at … slot 0`（桥真在固件 pty 上开口）、Ready、真固件快照 `CLOCK_FREQ==50_000_000`、`get_clock` 回绕差值、pid 收割。
- 评审（`77019792`）：**OK with notes**，无 P0/P1 代码缺陷；P1 属主仓手册（`regression-tests.md` 目标 40→41、linux 行 1→2、字典 37→38、没被引用 10→11），四条 P2 一行修（console.c 注释口径、`main.c` 返回码、`src/Kconfig:77` 措辞、`mux.h` 承载枚举、README 行号）已随 `628438d2` 落地。

### P4 落地（`6815a08`）

- `bridge_ports::bridge_downstream_counts` 零 access 预扫（与 `port_grouping::port_reference` 逐项同构：裸/前缀两种 `[mcu]` 拼写、`uart:`/`can:` 两键、端口节不存在则跳过、同节双键先写胜）+ `PathGroup.downstream` + `serial_ports::is_id_addressed` 显式含计数项 + `host_libraries::path_is_mux` 同口径；新测 `one_mcu_with_three_downstream_devices_is_id_addressed`（**钉住计数 = 3**，不只钉布尔）。
- 评审：**无 P0/P1**；P2 三条已收口——`host_libraries` 新测是**空证**（`has_bridge` 分支恒真，删新项仍绿）已删并改在既有桥测旁注释说明“计数项在此不可隔离、计数在 `serial_ports` 侧钉住”；`testing.md` 的 `host_libraries.rs` 计数修正为 21（删空证后与实现一致）；开发手册**模块表**两行旧断言（`serial_ports.rs` 的“1 颗 = 独占”、`host_libraries.rs` 的“1 节无 uuid = 裸帧”）补上桥例外（源：上一次提交漏同步）。

### P5 落地（本轮 = 免真机可验收的三件；其余归真机/后续）

- **P5-C（主仓 `78a6c9c`）**：`CanSerialDevice` 新增 `send_set_port_node_id`/`send_reset_device`/`send_reset_port`（纯委派到串口侧同一构造器 ⇒ 与串口**逐字节相同**；私有 `send_bytes` 抽出、公开 `send` 行为不变）、`interface/mod.rs` 三条 `match` 加 CAN 臂；`_` 臂「需要桥链路」文案改为含三种链路（含三条逐字断言）。测试 socket-free（socket pair 冒充 CAN socket，`#[cfg(test)]` 接缝），含跨 8 字节帧重组。评审 **OK with notes**（无 P0/P1）。
- **P5-D（子模组 `50fda1cf` + 收口 `3b23c8ea`）**：`config MUX_CAN`（`depends on CANBUS && HAVE_CHIPID`、`select MUX`、`不默认开`）、`BRIDGE` 改 `default y if MUX_SERIAL || MUX_CAN`、`src/Makefile` 改 `src-$(CONFIG_MUX) += mux.c`（两种载体共用一份；互斥是各 arch 通信接口 `choice` 的源码事实，已写进 Kconfig 注释）、`canserial.c` 收端（`command_find_block()` 前、`irq_save()` 段内）接 `mux_input`、发端 `mux_out_header` + 整单元 `mux_send`、`canserial_set_uuid` → `mux_set_uuid`。评审 **可合入**（无 P0/P1）；收口含注释口径修正、`src/host/README.md` 旧断言、新增 `test/configs/stm32f103-can-mux.config`（让闸门真的编 `CONFIG_MUX_CAN` 的代码路径——此前全仓无任何 CAN 目标）。
- **诚实边界**：本环境无 CAN 外设，P5-D 只有**构建级**验证（候选 B 的帧形是设计一致性结论，不是对已跑通 CAN 桥的实测）；主仓侧新臂在**当前产品路径不可达**（`bridge_attach` 仍拒绝 CAN 上游与 `[can_device]` 端口）。
- **P5 尚余（真机/后续）**：`[can_device]` 下游端口节（`CanPort` 拿 `index` + 配置期下发）与固件 **`BRIDGE_PORT_CAN` 端口后端**（新的 `bridge_port_ops`：CAN 收发 + 按下游 id/uuid 过滤）；CAN 上游 attach 端到端（先 `0x3f0` 单帧指派 node id，再经单元扫描/采纳/路由；脚本化总线可做一部分，真总线必需真机）；`vcan` 夹具（需 `CAP_NET_ADMIN`，本仓无先例）；`send_bytes` 目前无写锁（每器件单写者；P5 接线时需决定单写者或加锁——与 D11 在 host 侧同类）。

## 11. 第一次上板（2026-10-15）：USB CDC 接 mux + M1/M2 真机项

用户提供硬件：STM32F103ZE 板（SWD 经 ST-Link，刷写命令 `./venv/bin/pyocd load --target stm32f103ze
third_party/klipper/out/klipper.elf`，构建配置在子模组 `.config`）。用户同时拍板：**P5/P6 全部挂
「待测（硬件）」，不阻塞进度**。

### 决断（D28–D30）

- **D28（USB CDC 控制台接 mux）**：此前 mux 接线只在 `serial_irq.c`（USART）、`canserial.c`、linux、host
  四处，且 `MUX_SERIAL depends on SERIAL`——而 stm32 的通信接口是 choice，USB CDC 控制台 ⇒ `SERIAL=n`
  ⇒ **这块板根本选不了 mux**。先在 `src/generic/usb_cdc.c` 补上同样的两个接点 + `mux_send`，并把依赖
  放宽为 `SERIAL || USBSERIAL`（载体的存在就是依赖）。这是 M1-2 真机的前置。
- **D29（Makefile 去重）**：放宽后 `stm32f103 + USBSERIAL + MUX_SERIAL` 链接失败（`chipid.o` 重复定义）
  ——`chipid.c` 被 USB/CAN/USBCANBUS/MUX 四条 `src-$(CONFIG_*) +=` 各列一次。修法不是逐 arch 打补丁，
  而是在 arch include 之后一行 `src-y := $(sort $(src-y))`（同时修掉重复链接与重复
  `compile_time_request` 两个问题）。代价：丢掉了「重复列入」这个本来就不会编译报错的信号（记入代价）。
- **D30（`RESET_DEVICE` 的前置条件）**：真机上控制路径全通（主机重申 `SET_PORT_NODEID(0,id,1)` → 桥按
  端口转发整单元），但 stm32 的 `bootloader_request()`（`src/stm32/stm32f1.c:247`）只有 CanBoot 与
  `FLASH_START_800/2000` 两种 USB bootloader 路径；本板 `FLASH_START_0000` 且未开 CAN ⇒ **设计内 no-op**。
  结论：per-device 硬复位要求下游固件带**可软件进入的 bootloader**；否则只能 `RESET_PORT`（线级，对原生
  USB CDC 板无效——与既有结论一致）或断电。

### 真机结果

| 项 | 结果 | 关键证据 |
|---|---|---|
| mux 扫描/身份 | ✅ | `klipperx scan --port /dev/ttyACM1` → `uuid=e512c60ac0e6`；= 板 UID（`39ffd705…`）的 `chipid_uuid` 派生，用仓库自带 C 版 fasthash64 独立复算一致 |
| M1-2 命名下游经真桥 | ✅ | 宿主库当桥：`bridge identified: 22`、`child identified: 135`、库 stderr `open at 250000 baud (fd N, slot 0)`（`/dev/ttyACM1`） |
| M1-3 配置期点名失败 | ✅ | `uart_path` 指向不存在设备 ⇒ `MCU 'bridge' shutdown during connect: config_uart: the UART device could not be opened` |
| M2-2 控制路径 | ✅（设备侧 no-op） | `MCU 'child': resetting bridge port 0 with RESET_DEVICE`；设备未重启（见 D30） |
| M2-3 无回执确认路径 | ✅ | 主机没有假成功：`Failed automated reset: MCU 'child' is still configured (CRC 0x890098f7)`，并先走 `Firmware is still in an earlier session (sequence 6); taking it over` |
| M1-1 legacy 下游 | ✅（宿主机当桥形态，早前已验；**板做桥**未验） | 见 §7 P8 与本节；板做桥需 MCU 侧 UART 端口后端 |

### 11.1 M2-2 续（2026-10-15 晚）：命令拆分 + 真机重启打通

用户拍板：**`0x02` 保留为「进引导」**（属刷写工具，不进配置面），**新增 `0x05 RESTART_DEVICE`** 用于「重启固件」，
`restart_method: admin` 改走 0x05。子模组 `b7856e0d`（命令表、`mcu_restart()` 成为 `command_reset` 的唯一实现；
linux 用 `execv`、host 库 no-op）+ `3bd1df7d`（两个 MCU mux 矩阵目标）+ `5919539f`（下一条）；主仓 `ed0aaec`
（主机侧改名/新发发送/测试断言）。

**排障链（值得复用）**：
1. 真机反复「日志说重启了、板子却没重启」→ 用 `socat` 中继抓桥↔板的字节，证明 **0x05 单元确实上了线**；
   直接向板子 tty 写同一单元也无效 ⇒ 问题在设备侧。
2. 怀疑固件没刷上 → pyocd 默认频率下 `load` **静默失败**（报成功但板上是旧镜像）；用
   `-O connect_mode=halt -O frequency=1000000 -e chip` 才真刷上（此后设备自报 `mcu_version=609f1732` 与
   本地构建一致）。另发现 **sleep 态经 SWD 读内存全返回 0xFF**（必先 `halt`）——一度误判「flash 是空的」。
3. 真正的设计缺陷：**每个 mux 构建自己也是桥构建**（`BRIDGE` 是 `default y if MUX_SERIAL`），而
   `bridge_admin_unit` 对 `[0x02]/[0x05][index]` **无条件消费**——板子自己就是设备时命令被它自己的桥半吞掉，
   永远到不了本机设备半。修法（`5919539f`）：**只有 index 指到本机在用端口才消费，否则穿透给本机设备半**。
4. 修后真机验证：`FIRMWARE_RESTART` → `resetting bridge port 0 with RESTART_DEVICE` → 板子 USB
   **重新枚举**（`ttyACM2` → `ttyACM1`）⇒ 点名重启真的生效了。

### 11.2 桥端口 tty 丢失后的自动恢复（2026-10-15 晚）

背景：11.1 的真机验证里，设备被 0x05 重启后**USB 重新枚举**，宿主库桥这一刻 `config_uart` 打不开 tty
⇒ 固件锁存 shutdown ⇒ 整台打印机进 error（旧结论：「这个拓扑下不可能在同一会话内恢复」）。用户要求：
**给 host 也提供重启能力（重走 init），并用固定 by-id/by-path 打开**。

**D31（选 B）**：不做新缝口、不改 `third_party/`。宿主库的「重走 init」就是既有生命周期——最后一个
引用析构 → `klipper_host_shutdown` + 卸载 → 下次装载**重新映射 + 重新 `init`**，端口表每次 bring-up
都会按 `uart_path:` 重新登记；所以只需：**桥 bring-up 因端口打不开而失败时，有界等路径回来（≤10s/100ms），
再请求一次 plain restart**（plain 而非 `firmware_restart`，避免把子设备再点一次 0x05 ⇒ 循环）。
否掉 A（给 `McuChip` 加 `detach` seam 做「只重启桥会话」：价值不足、跨层新缝口）；C（库侧补 `config_reset`
让 host 能原地复活会话）**记为独立后续议题**——库字典里既无 `reset` 也无 `config_reset`（只有
`clear_shutdown`）、`oid_alloc` 拒重复 oid ⇒ 同映射确实无法原地重配置。

**提交**：主机 `76161c4` + P1 修复 `f79aab7`（合入 `49ea8fb`）。

**评审抓出的 P1（真问题，已修）**：恢复**没有预算** ⇒ 「路径**存在**但 `open` 失败」（`uart_path` 指到
目录 EISDIR / 无权限 / 被占 / 槽越界——固件一律同一条 NO_DEVICE 文案）会变成 **~1s 周期永不停歇的
restart 循环**（比改前的稳定 error 更糟）。修法：恢复预算挂在 **`Printer`** 上按节名记
（`claim_bridge_port_recovery` / `clear_bridge_port_recovery`——不能挂每轮重建的 `McuObject`），请求重启前
claim、该 MCU **成功 connect 后归还**：真实丢端口仍能恢复一次，“一直在但打不开”只重启一次便稳定 error。
新增对照测试（`uart_path` 指目录 ⇒ 至多一次重启）4.8s 绿。

**验收**：三条新测试串行 1.8s / 12.3s / 4.8s（全程有界，不再有上次那种挂死）；`bridge_attach` 22 测、
`object` 36、`restart` 14；全量闸门 3252 passed / 0 failed / 13 ignored；fmt 干净、`--all-targets` 0 告警。

**仍未覆盖（登记为后续）**：in-place reset（`reconnect` 在 `handshake_with_in_place_resets` 内）里的 child 失败不走这条恢复。

**11.2.1 child 侧触发（已补，缺口关闭）**：真机复验发现 `FIRMWARE_RESTART` 的顺序是「桥先起 → 再发 child 的
`RESTART_DEVICE`」，于是失败落在 **child 侧**（`SET_KLIPPER_NODEID handshake … Receive timeout` + `write
failed: (5)IO`）——上面的缺口。补法（`3e42390`，合入 `2b24696`）：**同一套手段扩到 child 触发**——
`bridge_attach::host_bridge_port_path` 只认「`Transport::BridgePort` + Uart + 端口节有 `uart_path` +
桥是 host 库」，有界等该路径 + **按 child 自己的节名**领预算 + 一次 plain restart；桥侧逻辑与桥节预算不变。
三条新测试（4.8s 恢复 / 13.8s 路径不回来不重启 / 9.3s 路径在但连不上只重启一次）均在 main 侧复跑绿。
**真机全链通过**（`host-recover2.log`）：`RESTART_DEVICE` ⇒ 设备重新枚举（ttyACM2→ttyACM1）⇒ child 侧超时
失败（照实报一次 error）⇒ `the bridge port tty came back; restarting the printer to reopen it` ⇒
plain restart ⇒ `bridge 22` / `child 135` 重新起来、**state=ready**（全链约 40s，~35s 是 child 会话自身超时）。
**待办**：本轮 child 侧改动 `3e42390` 尚未经独立评审（同一套已被评审的模式的小扩展），列入里程碑末的
统一评审；另：若一次恢复请求后重启始终连不上，预算不归还（需人工 `FIRMWARE_RESTART`）。

### 仍未验 / 缺件（真机与后续）

1. **板做桥**（M1-1/M1-2 的另一半）：MCU 侧没有 UART 端口后端（`CONFIG_SERIAL_UART` 只有 host 库实现）⇒
   「一块板当桥、其 uart 接下游」今天配置不出来。
2. `RESET_DEVICE` 的**正向**验收：需要一块带 CanBoot/USB-HID bootloader 的下游板。
3. 两台真设备同桥、真板上的指派/冲突、`irq_save()` 段内时序与丢帧率。
4. P5（CAN）/P6（485）全部待硬件（见 §10）。
