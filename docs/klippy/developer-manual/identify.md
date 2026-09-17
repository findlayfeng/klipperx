# Identify 机制

Identify 是 Klipper 主机端（klippy）与 MCU 端（固件）之间建立通信时的**数据字典协商流程**。其核心目的是：在物理连接建立后，从 MCU 获取一份 JSON 格式的配置与命令描述数据，使主机能够了解 MCU 支持的命令、响应、枚举常量和配置参数。

这也是主机唯一被允许硬编码格式串的地方：字典本身就要靠这条消息传过来。

## 协议定义

| 消息 ID | 格式 | 方向 |
|---------|------|------|
| `0` | `identify_response offset=%u data=%.*s` | MCU → 主机 |
| `1` | `identify offset=%u count=%c` | 主机 → MCU |

两条定义见 `identify::IDENTIFY_MESSAGES`，与 Klipper 的 `msgproto.DefaultMessages` 一致。它们在固件字典里也会原样出现一次，所以 `Dictionary::install` 必须跳过已注册的消息，否则会因 id / name 重复而失败。

**工作流程**（`Identify::fetch`）：

1. 主机发送 `identify offset=N count=40`，请求从偏移量 N 开始、最多 40 字节的数据
2. MCU 回复 `identify_response`，携带当前偏移量和数据块
3. 当固件发回**空数据块**时表示传输完成（固件在 `offset >= isize` 时把 `count` 置 0）
4. 把完整负载做 **zlib 解压**，再解析成 JSON，得到 `Identify { data }`

与 Klipper 的一个差异：Klipper 在 offset 不匹配时不追加数据、继续用同一 offset 重试（可能无限循环）；本实现直接报 `McuError::IdentifyProtocol`，避免死循环，也避免把错位的数据拼成一份看似合法的字典。

### 固件侧行为（源码依据）

主机这边的几个假设都能在 `third_party/klipper` 里找到出处，列出来是为了让它们不被当成巧合：

| 事实 | 出处 |
|------|------|
| 字典是**先压缩再编进固件**的：`command_identify_data` 来自 `zlib.compress`（本仓库的样本构建：699 字节压缩 → 1323 字节解压，头部 `0x78 0xda`，即 zlib 包装 + 最高压缩级别） | `scripts/buildcommands.py` 生成的 `out/compile_time_request.c` |
| 请求的 `count` 是 `%c`（单字节）；`offset + count` 越过末尾时固件把它截断，`offset >= isize` 时直接发 0 字节 | `src/basecmd.c::command_identify` |
| 所以**末块可能短于 40 字节**，主机必须按实际收到的长度推进偏移（`fetch_compressed` 用 `payload.len()`），不能假定每块都是 40；而「0 字节」就是完成标记 | 同上 |

因此解压必须用 **zlib**（`ZlibDecoder`）：固件带的头部就是 zlib 的，裸 deflate 虽然数据相同却少了它。`identify.rs::test_decompress_requires_the_zlib_wrapper` 专门钉住这一点；短末块与完成标记由 `test_fetch_single_chunk` / `test_fetch_multiple_chunks_wrapping_sequence_numbers` 覆盖。

## 代码位置

`identify` 模块与 `mcu`、`cmd` 平级（都在 `src/core/klippy/` 下），**不在**命令层：

| 内容 | 位置 |
|------|------|
| 命令定义（类型化视图、`count` 参数） | `cmd::identify`：`IdentifyRequest` / `IdentifyChunk` |
| 主机侧格式定义（唯一的硬编码例外） | `identify::IDENTIFY_MESSAGES` |
| 格式注册（构造时 `Mcu` 拿到的起始 `Parser`） | `identify::new_parser` |
| 分块请求、拼接、解压、JSON 解析 | `identify::Identify::fetch` |
| 抓取 + 建字典 + 安装 | `Mcu::identify` |
| 建连 + 握手（常用入口） | `Mcu::connect` |

`Mcu::identify` 与 `Mcu::connect` 是写在 `identify.rs` 里的固有 `impl Mcu`，所以整条引导流程与它的格式定义、分块驱动在同一个文件里。

拆成两处的理由：**命令的定义**（名称、参数、解码）与其它命令一样放在命令层 `cmd`；**分片驱动**不是命令的一部分——一条 `identify` 只请求一个窗口 `offset..offset+40`，把一串这样的回应拼成负载是链路层的事——所以它和主机自有的格式定义一起留在 `identify`。

### 为什么 `identify` 与 `mcu` 平级，而不是它的子模块

它曾经是 `mcu/identify.rs`，只能作为传输的内部细节被引用。它与 `mcu`、`cmd` 是同等地位的模块：一个协议关注点，只不过恰好是主机自己定义的那个。

代价是 `mcu` 与 `identify` 互相引用——传输在构造时调用 `identify::new_parser`，`identify` 反过来用 `Mcu::call_msg_ungated` 与 `Mcu::install_dictionary` 驱动传输。这个环是引导交换的真实形状：谁也不能排在前面。`identify` 还从 `cmd` 取两个类型化视图，因为命令的定义按统一规则归命令层。

## Rust API

```rust
// 正常入口：建连 + 握手 + 安装字典，返回可共享的句柄
let mcu: Arc<Mcu> = Mcu::connect(config).await?;

// 需要自定义超时或重试时，分两步
let mcu = Mcu::new(config);   // 只起传输，尚未识别
let installed = mcu.identify(Duration::from_secs(5)).await?;  // 抓取 + 建字典 + 安装

// 只要原始负载：标识符名未建模，未知字段不丢失
// （`Identify::fetch` 是 crate 内部接口，由 `Mcu::identify` 调用）
```

- `Mcu::connect` 使用固定的 `IDENTIFY_TIMEOUT`（10 秒，覆盖整个握手）；`Mcu::identify(timeout)` 可覆盖。
- `Mcu::identify` 是「抓取 → `Dictionary::from_json` → `Mcu::install_dictionary`」的连写；重复调用会替换字典，已注册的消息被跳过，因此握手中断后可以重试。
- `Identify` 只保存完整 JSON（`data: serde_json::Value`），不建模具体字段，未知或未来的字段不会丢失、也不会导致拒绝。结构化视图是 `Dictionary`。
- identify 没有能力 trait：它是一次性引导，只有一个实现、没有替代后端，且没有任何命令模块会持有它。

## 数据字典字段

解压后的 JSON 包含以下字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `enumerations` | Object | 枚举常量映射表。包含 `pin`（引脚名→ID）、`static_string_id`（静态字符串→ID）等枚举集合，供主机端解析消息参数 |
| `commands` | Object | MCU 可接收的命令格式字符串 → 命令 ID 映射。格式如 `get_clock`，主机据此构造出站消息 |
| `responses` | Object | MCU 发出的响应格式字符串 → 响应 ID 映射。如 `clock clock=%u`，主机据此解析入站消息 |
| `output` | Object | 输出格式字符串 → ID 映射。用于无请求方的异步输出消息；**没有前导消息名**，当前不注册进 `Parser` |
| `config` | Object | 固件编译期常量，如 `CLOCK_FREQ`（MCU 主频）、`MCU`（芯片型号）、`SERIAL_BAUD`（波特率）、`INITIAL_PINS` 等 |
| `version` | String | 固件版本，通常由 git describe 生成（含 tag、commit、dirty 标记、构建时间） |
| `build_versions` | String | 交叉编译工具链版本，如 `gcc: 12.3.1 binutils: 2.41` |
| `app` | String | 固件类型标识，固定为 `Klipper` |
| `license` | String | 许可证标识，固定为 `GNU GPLv3` |

各字段如何被解析与安装，见 [MCU 协议与数据字典](mcu-protocol.md#dictionary--固件自述的协议)。

## 安全限制

| 限制 | 值 | 说明 |
|------|----|------|
| 分块大小 | 40 字节 | `IDENTIFY_CHUNK_SIZE`，与 Klipper 一致 |
| 负载上限 | 1 MB | `MAX_IDENTIFY_DATA_SIZE`，同时约束压缩后与解压后的数据 |
| 解压方式 | `ZlibDecoder` + `Read::take` | 边读边截断，zip bomb 不会先被完整展开再检查 |

## 错误

| 变体 | 触发条件 |
|------|----------|
| `McuError::Call` | 收发失败或分块请求超时 |
| `McuError::IdentifyProtocol` | 分块 offset 错位，或负载超过上限（含解压后超限） |
| `McuError::IdentifyCompression` | zlib 解压失败 |
| `McuError::IdentifyJson` | 解压后的内容不是合法 JSON |
| `McuError::Dictionary` | JSON 合法但字段形状不符（如 `commands` 不是对象） |

---

- [← 开发手册首页](README.md)
- [MCU 协议与数据字典 ←](mcu-protocol.md) · [内部架构 →](architecture.md)
