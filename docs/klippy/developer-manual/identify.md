# Identify 机制

Identify 是 Klipper 主机端（klippy）与 MCU 端（固件）之间建立通信时的**数据字典协商流程**。其核心目的是：在物理连接建立后，从 MCU 获取一份 JSON 格式的配置与命令描述数据，使主机能够了解 MCU 支持的命令、响应、枚举常量和配置参数。

这也是主机唯一被允许硬编码格式串的地方：字典本身就要靠这条消息传过来。

## 协议定义

| 消息 ID | 格式 | 方向 |
|---------|------|------|
| `0` | `identify_response offset=%u data=%.*s` | MCU → 主机 |
| `1` | `identify offset=%u count=%c` | 主机 → MCU |

两条定义见 `mcu::identify::IDENTIFY_MESSAGES`，与 Klipper 的 `msgproto.DefaultMessages` 一致。它们在固件字典里也会原样出现一次，所以 `Dictionary::install` 必须跳过已注册的消息，否则会因 id / name 重复而失败。

**工作流程**（`Identify::fetch`）：

1. 主机发送 `identify offset=N count=40`，请求从偏移量 N 开始、最多 40 字节的数据
2. MCU 回复 `identify_response`，携带当前偏移量和数据块
3. 当 `offset == len(identify_data)` 且数据为空时，表示传输完成
4. 把完整负载做 **zlib 解压**，再解析成 JSON，得到 `Identify { data }`

与 Klipper 的一个差异：Klipper 在 offset 不匹配时不追加数据、继续用同一 offset 重试（可能无限循环）；本实现直接报 `McuError::IdentifyProtocol`，避免死循环，也避免把错位的数据拼成一份看似合法的字典。

## 代码位置

| 内容 | 位置 |
|------|------|
| 主机侧格式定义（唯一的硬编码例外） | `mcu::identify::IDENTIFY_MESSAGES` |
| 握手协议、分块、解压、安装字典 | `mcu::cmd::identify` |

字典属于协议知识，而 identify 是唯一在字典存在之前运行的交换，因此它按命令模块的方式书写：类型化消息视图（`IdentifyRequest` / `IdentifyChunk`）加协议逻辑，只是走 [`Mcu::call_msg_ungated`](mcu-protocol.md) 而不是普通的 `call_msg`。

## Rust API

```rust
// 正常入口：建连 + 握手 + 安装字典，返回可共享的句柄
let mcu: Arc<Mcu> = mcu::cmd::identify::connect(config).await?;

// 需要自定义超时、重试，或先看原始负载
let mcu = Arc::new(Mcu::from(config));
let identify = McuIdentify::new(Arc::clone(&mcu))
    .with_timeout(Duration::from_secs(5));

let installed = identify.identify().await?;        // 抓取 + 解析 + 安装
let raw: Identify = identify.fetch().await?;        // 只抓取
let dictionary = Dictionary::from_json(raw.data)?;  // 自行处置
```

- `connect` 与 `McuIdentify::new` 都使用固定的 `IDENTIFY_TIMEOUT`（10 秒，覆盖整个握手）；`with_timeout` 可覆盖。
- `McuIdentify::identify` 是「抓取 → `Dictionary::from_json` → `Mcu::install_dictionary`」的连写；重复调用会替换字典，已注册的消息被跳过，因此握手中断后可以重试。
- `Identify` 只保存完整 JSON（`data: serde_json::Value`），不建模具体字段，未知或未来的字段不会丢失、也不会导致拒绝。结构化视图是 `Dictionary`。
- `McuIdentify` 没有对应的能力 trait：握手是一次性引导，只有一个实现、没有替代后端。

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
