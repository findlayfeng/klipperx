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

---

## 1. 动机与范围

**要解决的**（M1）：下游接的是**没有 admin 通道的旧设备**（原版 klipper 串口固件，或任何不实现
mux 的设备）时，主机仍要能把它当普通 `[mcu]` 连上并跑通 identify + `get_clock`。

**不在本文范围**：M2（经 admin 复位固件）——只在 §5 P5 与它对接；CAN 上游的完整后端实现
（本文只要求接口与判定正交，见 §4.4）。

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
- **验收**：三种配置（仅 mux / mux+桥 / 全关）0 告警；**探针**证明"主机发一条端口级命令后，
  该 id 的数据单元被转到该端口、该端口的原始帧被加上该 id 的应答头上行"，**全程不要求下游答
  admin**；端口表能在 identify 字典里读到。
- **文档**：固件 `src/host/README.md`、`serial-mux-protocol.md`（命令族表 + 两级寻址）。

### P3 attach 走端口（klipperx，端到端）

- 装载期：端口节名 → `(mcu, oid)`；identify 后与固件端口表对账 → 取 `index`；按规则 4/5 分岔：
  有 `uuid:` ⇒ 现有 `load_mux`（`SET_NODEID` + 重确认握手）；无 `uuid:` + uart ⇒ **跳过**，
  只对桥发 `SET_PORT_NODEID(index, id)`，之后按原始帧收发。
- **验收（端到端）**：桥下游挂"**没有 mux 的固件**"（`test-support` 那份无 mux 的库，或现有假响应器
  夹具），主机把它当普通 `[mcu]` 连上并跑通 identify + `get_clock`；另一条回归：带 `uuid:` 的设备
  走原路不退化。
- **风险**：**测试夹具体积**——要在 pty 上跑一份"旧固件"（或等价的假响应），这可能是 P3 里最花时间
  的一块；先做夹具、再做判定。

### P4 判定与传输正交

- 规则 7：上游路径"是否按 id 寻址"要把**下游设备数**算进去（`mcu/serial_ports.rs` 的分组要扩）。
- **验收**：单测覆盖"一颗 `[mcu]` 但带 3 台下游 ⇒ 该线按 id 寻址"。
- **文档**：两级寻址表进 `serial-mux-protocol.md` 与 `mcu-connection.md`。

### P5 CAN 上游与复位（后置）

- 桥经 CAN 连接时端口表与 `SET_PORT_NODEID` 的帧形（CAN admin 帧 / CAN 载荷内的同一单元）。
- M2（经 admin 复位）与本文对接：**面向设备**（带 uuid 的设备走它的 admin）与**面向端口/桥**
  （桥执行线级复位——无 uuid 的 uart 端口只能走这条）两类命令，复用 P2 的端口 `index`。

### 5.6 未决（开工时定）

a. **i2c/spi 端口的写法**：klipper 的 `[i2c <名>]`/`[spi <名>]` 是**引脚式**（自己在该 MCU 上建 oid），
   而桥的 i2c/spi 端口是**固件里已有的 oid** ⇒ 需要一个选项表达"沿用已有 oid 而不是按引脚新建"。
   本文只要求"端口能被非 MCU 消费者按节名引用"，具体键名与语义留到 P1 定。
b. **端口 `index` 的稳定性**：固件重排端口会让 index 漂移。稳定的标识是 **oid 名**（identify 表里
   两个都报）；index 只用于线上命令，不需要稳定。
c. **端口级命令的帧形**：`SET_PORT_NODEID` 的参数集（`index` + `id` + `flags`；要不要带 uuid 做校验）、
   `flags` 的取值（至少"设备说不说单元"；将来可能还有"半双工/方向控制"之类）、以及它在 CAN 上游下的
   帧形（CAN admin 帧 vs 载荷内的同一单元）。

### P6 多站串口（RS-485 一类，后置）

- 新增端口类型/后端（`src/bridge_port_485.c` 之类：半双工收发与方向控制归它）；主机侧则是"一个
  串口端口节被多颗 `[mcu]` 引用"⇒ 保留帧头、每台设备走自己的 `uuid → id` 分配（§4.5）；legacy
  不允许出现在这类端口上（§3 规则 4）。
- **验收**：一条多站端口上挂 ≥2 台说单元的设备，各自的 id 分配与数据帧都正确路由（都带头）；
  另一条对照：单设备 + legacy 端口仍是剥头/加头（§4.6 两行判据各覆盖一次）。

### 改造清单（汇总）

| 侧 | 文件 |
|---|---|
| klipperx | `config/mcu.rs`（新节与新键）、`mcu/`（端口分组，泛化 `host_libraries.rs`）、`interface/devices/host.rs`（attach 分岔）、`assign.rs`（不动） |
| 固件 | `src/bridge.{c,h}`（端口表 + `SET_PORT_NODEID`）、`src/host/Kconfig`（端口路径留在固件）、`src/mux.c`（不动） |
| 手册 | `serial-mux-protocol.md`、`mcu-connection.md`、`config.md`、`upstream-deviations.md` |

---

## 6. 实施记录

**尚未开工。** 开工后本节的每次回填写：当期的实际落地范围、验收实测数字（命令 + 结果）、
与 §5 的偏差（以及为什么）、以及仍未收口的余项。
