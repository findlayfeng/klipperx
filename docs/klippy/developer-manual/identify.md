# Identify 机制

Identify 是 Klipper 主机端（klippy）与 MCU 端（固件）之间建立通信时的**数据字典协商流程**。其核心目的是：在物理连接建立后，从 MCU 获取一份 JSON 格式的配置与命令描述数据，使主机能够了解 MCU 支持的命令、响应、枚举常量和配置参数。

## 协议定义

| 消息 ID | 格式 | 方向 |
|---------|------|------|
| `0` | `identify_response offset=%u data=%.*s` | MCU → 主机 |
| `1` | `identify offset=%u count=%c` | 主机 → MCU |

**工作流程**：

1. 主机循环发送 `identify offset=N count=40`，请求从偏移量 N 开始、最多 40 字节的数据
2. MCU 回复 `identify_response`，携带当前偏移量和数据块
3. 当 `offset == len(identify_data)` 且数据为空时，表示传输完成

获取的数据是经过 **zlib 压缩**的 JSON，包含以下字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `enumerations` | Object | 枚举常量映射表。包含 `pin`（引脚名→ID）、`static_string_id`（静态字符串→ID）等枚举集合，供主机端解析消息参数 |
| `commands` | Object | MCU 可接收的命令格式字符串 → 命令 ID 映射。格式如 `G1 X=%u Y=%u`，主机据此构造出站消息 |
| `responses` | Object | MCU 发出的响应格式字符串 → 响应 ID 映射。如 `temperature_sensor temp=X`，主机据此解析入站消息 |
| `output` | Object | 输出格式字符串 → ID 映射。用于无请求方的异步输出消息 |
| `config` | Object | 固件编译期常量，如 `CLOCK_FREQ`（MCU 主频）、`MCU`（芯片型号）、`SERIAL_BAUD`（波特率）、`INITIAL_PINS` 等 |
| `version` | String | 固件版本，通常由 git describe 生成（含 tag、commit、dirty 标记、构建时间） |
| `build_versions` | String | 交叉编译工具链版本，如 `gcc: 12.3.1 binutils: 2.41` |
| `app` | String | 固件类型标识，固定为 `Klipper` |
| `license` | String | 许可证标识，固定为 `GNU GPLv3` |

---

- [← 开发手册首页](README.md)
- [内部架构 ←](architecture.md) · [← 返回消息结构](message-structure.md)
