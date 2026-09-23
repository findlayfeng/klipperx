# MCU 配置构建（ConfigBuilder）

一台真实 MCU 的配置是**一次性**下发的：每个资源（引脚、总线、传感器）先领一个 **oid**，再用一条 `config_*` 命令登记自己；主机把这些命令攒起来，算一个 CRC，随 `finalize_config` 一起发出去。之后固件靠 oid 引用这些对象，不再需要重复引脚与总线参数。

本文说明这一层为什么存在、设备侧保存了什么、怎么校验，以及**我们的 CRC 与上游算的不是同一样东西**——这是本层唯一一处有意偏离，展开在 [CRC 一节](#crc我们与上游算的不是同一样东西)。

上游参考：`klippy/mcu.py` 的 `MCUConfigHelper`（`:979-1143`）与 `src/basecmd.c` 的分配/配置区。

## 为什么需要配置期

固件是几乎不做动态分配的 C。`allocate_oids count=N` 是**唯一一次**动态分配，它一次性为整张对象表留出空间（`src/basecmd.c:227`）：

```c
oids = alloc_chunk(sizeof(oids[0]) * count);
oid_count = count;
```

每个资源随后用 `oid_alloc(oid, type, size)` 占据自己的槽（`:201`），运行期命令用 `oid_lookup(oid, type)` 取回数据（`:193`）——类型对不上就 `shutdown("Invalid oid type")`。所以：

- 主机必须在发配置命令**之前**知道一共有多少对象（`allocate_oids` 最前）；
- 配置命令只在固件「没有这份配置」时才重发，判断依据就是 CRC。

两者的先后决定了这一层的形状：**先攒，后发**。

## 三个命令列表

上游维护三张表（`klippy/mcu.py:1125`），我们的 `ConfigBuilder` 一一对应：

| 列表 | 什么时候发 | 作用 | 例子 |
|---|---|---|---|
| `config` | 只有固件不是这份 CRC 时才发 | 建立对象 | `config_digital_out`、`config_spi`、`finalize_config` |
| `restart` | 每次连接都发 | 恢复起始值 | `update_digital_out`（上电默认电平） |
| `init` | 每次连接都发，在其它之后 | 武装周期查询、启动时传输 | 周期 `query_analog_in`、启动 SPI 写 |

`build` 的过程是：

1. 跑 **config 回调**——资源在这里补上它在构造时加不进来的命令（那时还没有字典和 `CLOCK_FREQ`）；
2. 把 `allocate_oids count=N` 插到 `config` 最前；
3. 编码 `config` 列表并算出 **CRC**；
4. 把 `finalize_config crc=…` 追加到 `config` 末尾。

## 我们的实现

`src/core/klippy/mcu/config.rs`，类型 `ConfigBuilder`（`mcu/` 下，与传输层的 `Mcu` 分开）。它按 `[mcu]` / `[mcu <name>]` 一份，`Arc` 共享。

| 方法 | 作用 |
|---|---|
| `create_oid()` | 领下一个 oid（从 0 单调，不复用） |
| `add_config_cmd` / `add_restart_cmd` / `add_init_cmd` | 往三张表加命令 |
| `register_config_callback` | 注册 `build` 时要跑的回调（可继续领 oid / 加命令，拿到 `&Mcu`） |
| `register_pre_build_callback` / `run_pre_build` | **async 预建回调**：在 `build` 之前、字典已装且连接已活时跑（需要 `call_msg` 的钩子，如 `temperature_mcu` 的 `debug_read` 标定） |
| `register_post_init_callback` | 注册固件接受配置后要跑的回调 |
| `request_move_queue_slot()` | 预留运动队列槽位 |
| `build(&Mcu)` | 跑 config 回调、插 `allocate_oids`、算 CRC、追加 `finalize_config`；返回编码好的三张表（`BuiltConfig`） |
| `handshake(&Mcu, &mut BuiltConfig, expect_unconfigured)` | 与固件的单轮握手：`get_config` → 停机/CRC 判断（必要时复位）→ 发表 → 再 `get_config` 确认 → 跑 post-init。单独暴露是因为 `reset` 路径会断开连接：调用方用**同一份** `BuiltConfig` 重连重跑（`mcu/object.rs` 的循环） |
| `configure(&Mcu)` | `build` + 一次 `handshake(…, false)` 的便捷入口（不需要重试循环的调用方用它） |

生命周期挂在 `McuObject` 上（`mcu/object.rs`）：**`ConfigBuilder` 在 `McuObject::new` 时就建好**（配置装载期资源就要往里加命令），`PrinterObject::connect` 里先 `Mcu::connect`（identify 装字典），再跑 `run_pre_build` → `build` → `handshake` 循环：

```
load_config                      connect
─────────────                    ───────
McuObject::new
  └─ ConfigBuilder::new
资源: create_oid / add_config_cmd
                                 Mcu::connect  → identify 装字典
                                 run_pre_build（async 预建回调）
                                 build（跑 config 回调、编码、算 CRC）
                                 handshake 循环:
                                   get_config
                                   停机/CRC 不一致 ? 先复位 : 直接发
                                   发表（未配置 ? config+init : restart+init）
                                   get_config（确认 + move_count 校验）
                                   post-init 回调
                                   ResetRequired ? 重连重跑同一份 BuiltConfig
```

失败统一是 `McuError::Config`：固件停机且无法复位、CRC 不一致且无法复位、固件拒绝配置、运动队列槽位不够。`McuObject::connect` 把它转成 `KlippyError::Connection`。另有 `McuError::ResetRequired`：复位需要重连（固件有 `reset`）时由它把控制权交回 `connect`。（停机与 CRC 不一致**能复位时**不会报错：先 `reset` + 重连，或 `config_reset` 就地清，见下。）

## oid

**固件侧**是一张 `struct oid_s { void *type, *data; }` 的表（`src/basecmd.c:187`），下标就是 oid。`oid=%c` 一个字节，所以每台 MCU 最多 256 个对象。

**主机侧的「发号器」**是一个单调计数器（上游 `create_oid()`，`klippy/mcu.py:1118`）：

- 从 0 开始，依次 0,1,2,…，**不复用、不回收**；
- 只在配置期可用，`finalize_config` 之后禁止再发；
- 终值就是 `allocate_oids count=N` 里的 `N`；
- **每台 MCU 一个**：两个 `[mcu]` 各自从 0 开始。

我们与之对应：`create_oid()` 返回 `Result<u8, McuError>`，走完 `MAX_OIDS`（255）报错而**不回绕**（`%c` 装不下 256），定稿后调用报错。

oid 的**分配顺序 = 对象创建顺序 = config 里的顺序**，因此配置命令的字节序列是确定的——这正是下面 CRC 能当缓存键的前提。

## 设备侧的配置与校验

前面讲的是主机侧的构建。设备侧保存的是这次构建的**结果**，以及一个用来判断它是否仍是当前这份配置的校验值。

### 设备侧的配置是什么

固件在配置期建立、此后只读的状态：

| 状态 | 固件位置 | 建立 / 清除 |
|---|---|---|
| 对象表 `oids`（`oid_count` 个槽） | `src/basecmd.c:189-190` | `allocate_oids` 一次分配（`:227`），`oid_alloc` 逐槽占据（`:200`） |
| 运动队列（`move_count`） | `src/basecmd.c:170` 一带 | `finalize_config` 调 `move_finalize()` 定稿（`:255`） |
| 配置校验值 `config_crc` | `src/basecmd.c:242` | `finalize_config crc=%u` 写入（`:256`）；`config_reset` 清零（`:267`） |

`finalize_config` 终结配置期：之后 `oid_alloc` 报 `Can't assign oid`（`src/basecmd.c:203`），第二次 `finalize_config` 报 `Already finalized`（`:173`）。所以设备上的一份配置在**一次上电周期内只能建立一次**，要换成另一份必须先复位。

### 如何校验

`get_config` 把这三样以 `config is_config=%c crc=%u is_shutdown=%c move_count=%hu` 报回主机（`src/basecmd.c:244-250`）。主机连接时的流程是：

1. `ConfigBuilder::build` 把这次要下发的 `config` 列表（含 `allocate_oids`）编码并算出 CRC；
2. 发 `get_config`，读回设备侧的 `is_config` / `crc`；
3. 按两者决定：
   - `!is_config`：设备未配置（新上电，或刚被复位）→ 发 `config` 列表 + `init`；
   - `is_config && crc == 主机值`：设备上留的正是这份配置 → 只发 `restart` + `init`；
   - `is_config && crc != 主机值`：设备上是另一份配置 → 先复位（优先 `reset` + 重连，没有才 `config_reset` 就地；见 [CRC 一节](#crc我们与上游算的不是同一样东西)），再发 `config` + `init`。

固件**不重算、也不校验**这个值，只是原样存取（`src/basecmd.c:256` `:248`）。它是主机的缓存键：命中省掉一次配置下发，未命中就必须复位重配（配置期已由 `finalize_config` 锁住，不能就地重发）。因此它只需满足两条：

- **确定性**：同一份配置，两次运行算出同一个值；
- **敏感性**：配置变了，值几乎必然不同（CRC32 抗意外碰撞约 2⁻³²）。

### 校验值的存续

`config_crc` 是固件的 RAM 全局（`src/basecmd.c:242`），只在三处变化：`finalize_config` 写入、`config_reset` 清零、上电（BSS 归零）。主机断开、重连都不改变它，所以设备会带着上一次连接留下的配置和校验值等待下一次握手。

至于校验的输入具体是什么字节，见下一节：我们与上游选的不是同一样东西。

## CRC：我们与上游算的不是同一样东西

### 上游算的是命令文本

```python
# klippy/mcu.py:1004-1020
self._config_cmds.insert(0, "allocate_oids count=%d" % (self._oid_count,))
# …先用 pin resolver 把 pin=别名 改写掉…
encoded_config = '\n'.join(self._config_cmds).encode()   # 命令「文本」，换行拼接
self._config_crc = zlib.crc32(encoded_config) & 0xffffffff
self._config_cmds.append("finalize_config crc=%d" % (self._config_crc,))
```

哈希的是字符串，例如：

```
allocate_oids count=1
config_digital_out oid=0 pin=PA1 value=0 default_value=0 max_duration=2000000
```

上游整条管线都以文本为载体：`add_config_cmd` 存字符串，引脚别名用正则改写文本，枚举名（`pin=PA1`）直到**发送时**才由 `msgproto` 用字典解析成编号。

### 我们算的是编码后的字节

我们的命令是**类型化**的（`McuCommand { NAME, args() }`），没有命令文本这层。`build` 直接哈希每条命令编码出来的 wire 字节，首尾相接：

```rust
// src/core/klippy/mcu/config.rs
let mut hashed: Vec<u8> = Vec::new();
encode_into(mcu, AllocateOids::NAME, &allocate.args(), …, &mut hashed)?;
for command in &config {
    encode_into(mcu, command.name, &command.args, …, &mut hashed)?;
}
let crc = crc32(&hashed);
```

每条命令 = `[消息 id（VLQ）] ++ [按声明顺序编码的参数]`，所以上面那条在线上大致是 `0a 00 <PA1 的枚举值> 00 00 <2000000>`（示意，整数编码细节取决于 `%c`/`%u`）。**哈希的就是真正要发出去的字节。**

### 为什么这样做不错：固件不关心这个值

（设备侧怎么存、怎么校验见上一节。）固件收到 CRC 只是存起来：

```c
void command_finalize_config(uint32_t *args) {
    move_finalize();
    config_crc = args[0];          // src/basecmd.c:256
}
```

`get_config` 再原样回给主机（`src/basecmd.c:248`）。哈希方案只需满足上一节的确定性与敏感性两条；用文本还是用字节都满足，下面比较两者在边界情形下的取舍。

### 差异只会在哪儿显形

| 情形 | 上游（文本 CRC） | 我们（字节 CRC） | 谁更合理 |
|---|---|---|---|
| 同一份配置再来一次 | 相同 → 复用 | 相同 → 复用 | 平 |
| 用户改了引脚 PA1 → PA2 | 文本变 → 重配 | 字节变 → 重配 | 平 |
| 同一引脚，换个别名写法 | 文本变 → 多一次重配 | 解析后编号相同 → 复用 | 我们更精确 |
| 固件升级导致命令 id / 参数编码变了 | 文本没变 → 复用旧配置 | 字节变 → 重配 | 我们更安全 |
| 换主机实现（上游 Klipper ↔ 我们） | — | CRC 不同 | 见下 |
| CRC32 意外碰撞 | 2⁻³² | 2⁻³² | 平 |

只有在「文本变了但字节没变」（别名）和「字节变了但文本没变」（固件字典变了）这两格上分道扬镳，而**两个方向我们的行为都不比上游差**：别名那次省掉一次无意义的重配，固件那次把可能不兼容的旧配置重发一遍。

### 拼接为什么没有歧义

上游用 `\n` 连接，天然自定界；我们把多条 payload 首尾相接，看起来没分隔符。这不构成问题：Klipper 的 wire 格式本来就是「一帧里可以解码出多条消息」——每条以消息 id 开头，参数个数与类型由字典的格式串决定，按声明依次消费。收发侧的合并批次（`Payload::try_merge`）走的就是同一条路。我们哈希的正是固件会解码的那串字节，因此无歧义。

（若想让这件事不依赖「格式可解码」这个性质，可以加长度前缀或域分隔再哈希。可选加固，非必需。）

### 过程上的差异（连接握手）

除了「哈希什么」，两边在 **CRC 周围的流程** 也不同。上游 `_connect`
（`klippy/mcu.py:1047-1085`）在 CRC 前后还做了几件我们没做的事：

| 上游步骤 | 位置 | 我们的现状 |
|---|---|---|
| `_send_get_config` 先查**连接层**的 shutdown 标志（`conn_helper.is_shutdown()`），再查 `get_config` 的 `is_shutdown` 字段，两者都 raise | `:1039-1046` | 只查 `get_config` 回的 `is_shutdown` 字段；连接层标志（`McuObject::is_shutdown`）由握手**之后**绑的固件事件维护，握手时还不成立 |
| 未配置时先 `check_restart_on_send_config()`：`restart_method == 'rpi_usb'` 要先做一次 USB 断电重启才发配置 | `:686-689`、`:1052` | 已做：`restart::restart_before_bringup` 在 `connect` 里据此 `request_exit('firmware_restart')` |
| 已配置时先看 `start_reason == 'firmware_restart'`，是则 raise “Failed automated reset”（说明复位没生效），**再**算 CRC | `:1053-1056` | 已做：`handshake(…, expect_unconfigured)` 在 `McuObject::connect` 传入 `is_firmware_restart()`，命中就报 `Failed automated reset` |
| **CRC 不匹配时先 `check_restart_on_crc_mismatch()`：请求一次 `request_exit('firmware_restart')`、pause 2 s、然后才 raise** | `:678-685`、`:1057-1059` | 改成**优先真重启**：固件有 `reset` 就发 `reset` + 重连 + 重试握手（`McuObject::connect` 的循环）；只有没有 `reset` 时才 `emergency_stop` + `config_reset` 就地清；`rpi_usb` 的 CRC 不匹配由上一行的门先请求 firmware_restart |
| pin 名在 `_finalize_config` 里改写；非法 pin 的错误到**发送时**才被 `_send_cfg_init_commands` 捕获并转成 config error | `:1009-1013`、`:1021-1032` | F2 会在**加入命令时**就改写/报错，编码在 build 时，错误也在 build 暴露 |
| 发送后第二次 `get_config`：`fileoutput` 模式下跳过 `is_config` 断言 | `:1066-1068` | 总是断言 |
| `move_count` 与预留槽：把 `move_count - reserved` 交给 `steppersync` | `:1070-1078` | 只校验 `move_count >= reserved`（无运动层，C1） |
| `_finalize_config` 不带重入保护（重复调用会插两次 `allocate_oids`、追加两次 `finalize_config`） | `:1004` | `build` 二次调用直接报错 |

两边**相同**的部分也值得记下：都先 `get_config`，都**无论复用与否都算一次 CRC**（所以复用路径也要 build），都只在 `config` 列表上算、都排除 `restart`/`init`、都把 `allocate_oids` 计入、都把 `finalize_config` 排除。

### 唯一的实际代价

把一个「上游 Klipper 刚用同一份 printer.cfg 配好」的 MCU 交给我们时，上游存的 CRC 和我们算的对不上。`handshake`（`configure` 的握手部分）遇到「已配置且 CRC 不一致」时先尝试复位（见下），不能复位才报错：

```
MCU 'mcu' is configured with CRC 0x…, the host computed 0x…
```

**CRC 不匹配不能靠重发配置来修**：固件一旦 `finalize_config` 就把配置锁住——之后 `oid_alloc` 报 `Can't assign oid`（`src/basecmd.c:204`），再发一次 `finalize_config` 报 `Already finalized`（`src/basecmd.c:173`）。所以重发不只是无效，还会把 MCU 直接打进 shutdown。要换配置只能**复位**：停机时用 `config_reset`（`src/basecmd.c:262`，清 CRC/oid/运动队列），或者重启固件。这正是不匹配时上游**先请求 firmware_restart 再 raise** 的原因——它不重发，它重启。

我们现在也复位，而且**优先真重启**：`handshake` 发现固件已停机或 CRC 不一致时，`reset_firmware`
先看固件有没有 `reset`（`basecmd.c` 之外的板级命令）——有就返回 `McuError::ResetRequired`，
`McuObject::connect` 发 `reset`、重开连接、用同一份 `BuiltConfig` 重跑握手（得到一次真正的重启，
把定时器与步进队列也清掉，同上游 `_reset_cmd` 优先，`klippy/mcu.py:733-740`）。只有没有 `reset`
时才走 `config_reset` 就地清（运行中的固件先 `emergency_stop`，因为它只在停机时可跑），再重新
`get_config`、发这份配置。因为自己发起的 `emergency_stop` 不能被当成意外停机，"固件的停机事件"
只在握手**之后**才绑（`mcu/object.rs`）。

两者都没有的固件仍然报错，提示断电。上游在没有重启 helper 可用时（如 `start_reason ==
'firmware_restart'`）也是直接 raise。

### 对 pin 解析（F2，已实现）的约束

因为哈希的是**编码后的字节**，pin 的枚举名（`PA1`）必须在编码时解析成编号，所以顺序和上游相反：

- 上游：先写文本 → **先**改写别名/查保留 → **再**哈希文本 → 发送时才解析枚举。
- 我们：**先**解析别名/枚举成编号 → 造 `ArgValue` → build 时编码 → 哈希字节。

后果：

1. `PinResolver`（别名、`RESERVE_PINS_*` / `BUS_PINS_*`、重复使用检查）必须在**加入命令之前**作用于参数，而不是像上游那样在 finalize 时改写文本。非法引脚名 / 被保留引脚的错误会在**建配置时**就报出来。
2. 哈希天然覆盖**解析后的编号**，所以「换别名」不触发重配（见上表）。

### 可选加固

哈希方案将来若改变（例如把 pin 名字也纳入），可以给输入加一个域分隔/版本前缀（如 `b"klipperx-mcu-config-v1\0"`），把「新方案恰好撞上旧方案的 CRC」从 2⁻³² 的碰撞概率里彻底摘出去。当前只做 CRC32，尚未加。

## 与上游对应

| 上游（`MCUConfigHelper`） | 位置 | 我们 |
|---|---|---|
| `create_oid` | `klippy/mcu.py:1118` | `ConfigBuilder::create_oid` |
| `add_config_cmd(is_init, on_restart)` | `:1125` | `add_config_cmd` / `add_restart_cmd` / `add_init_cmd` |
| `register_config_callback` | `:1122` | `register_config_callback` |
| `register_post_init_callback` | `:1130` | `register_post_init_callback` |
| `_finalize_config` | `:1004-1020` | `build` |
| `_connect`（两段式） | `:1047-1085` | `build` + `handshake`（`configure` 是二者的便捷组合） |
| `seconds_to_clock` | `:1140` | `Mcu::seconds_to_clock` |
| `request_move_queue_slot` | `:1142` | `ConfigBuilder::request_move_queue_slot` |
| `get_query_slot` | `:1136` | `ConfigBuilder::get_query_slot`（用 `Mcu::estimated_clock`，见下） |
| — | `src/basecmd.c:235` | `AllocateOids`（`cmd/allocate_oids.rs`） |
| — | `src/basecmd.c:250` | `GetConfig` / `ConfigState`（`cmd/config.rs`） |
| — | `src/basecmd.c:258` | `FinalizeConfig`（`cmd/config.rs`） |
| — | `src/basecmd.c:262` | `ConfigReset`（`cmd/config.rs`；发送在 `reset_firmware`，`mcu/config.rs`） |

## 状态清单（均已完成）

> 本节原为「还没有的」，所列各项均已落地，保留为逐项说明与上游的对应关系。

- **pin 名改写**：已完成（F2）。`PinResolver` 在资源**加入命令之前**把别名/保留作用于参数，与上游“finalize 时改写文本”顺序相反，原因见上一节。`[board_pins]` 是它的装载入口（`extras/board_pins.rs`）。
- **`get_query_slot`**：已完成。`ConfigBuilder::get_query_slot` 返回 `现在的估计时钟 + 1.5 s + oid*0.01 s`；它用 `Mcu::estimated_clock`——connect 时一次 `get_uptime` 加上主机时间外推的最小估计，不跟踪漂移、也没有 print time。没有 `get_uptime` 的固件拿不到估计，`get_query_slot` 报 `McuError::Config`。
- **固件复位（`config_reset` / `reset`）**：已完成。`firmware_restart` 且 `restart_method: command` 时，`reset` 在**拆机之前**由 `McuObject::before_firmware_restart` 在**活连接**上发出（上游把它放在 `klippy:firmware_restart` 事件里，`klippy/mcu.py:754`），所以重连后只需 identify + 配置一次，而不用先连上去告诉它重启。配置握手路径上，`config_reset` 就地清；只有 `reset` 的固件发 `reset` + 重连 + 重试握手；两者都没有才报错。
- **`rpi_usb` 的连接期门控**：已完成。`restart::restart_before_bringup` 对应上游 `check_restart_on_attach` / `check_restart_on_send_config`（串口不在先上电、未配置先断电再配置），`check_usb_power` 在每次 connect 探测 hub 能否切电（不能则降级 `command`）。
- **`restart_method` 分派**：已完成。`McuConfig.restart_method` 被三处读取：`McuObject::connect` 挑 `command` 的 `reset`、`McuConfig::open` 给 `cheetah` 定 RTS、物理分派在 `mcu/restart.rs`（`command` / `arduino` / `cheetah` / `rpi_usb`）。

---

- [← 开发手册首页](README.md)
- [MCU 协议与数据字典 ←](mcu-protocol.md) · [Identify 机制 →](identify.md)
