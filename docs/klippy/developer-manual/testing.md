# 测试

测试与被测代码同文件，位于各模块的 `#[cfg(test)] mod tests`，不需要外部进程或真实串口。底层 IO 由 `interface::test::TestDevice` 模拟：它按 FIFO 逐条比对收到的帧，并把预设的输出帧排队给 `receive()`。

## 运行

```bash
cargo test --lib                      # 全部单元测试
cargo test --lib mcu                  # mcu 层
cargo test --lib mcu::cmd::clock     # 单个命令模块
cargo test --lib test_install_skips   # 单个用例（按名过滤）
```

## 覆盖范围

### `msg`

| 模块 | 覆盖 |
|------|------|
| `proto.rs` | 各类型 push/pop 往返、VLQ 有符号编码与 Klipper 对齐、边界长度、`try_convert_to` 范围检查、`try_merge` 超限不动原值 |
| `mod.rs` | `Msg::parse` 全部类型、错误路径、`format()` 往返、`Hash`/`Eq` |
| `parser.rs` | 注册（重复 id / name 拒绝）、`register_all`、`lookup`、编码/解码往返、批量解码、未知 id、回调绑定与替换、`Arc` 共享 |

### `mcu`

| 模块 | 覆盖 |
|------|------|
| `pending.rs` | 注册/配对/取消、未知名字不消费、先到先得、接收端已关闭、只取消一条 |
| `dictionary.rs` | 三张消息表的解析（含 `output` 原样保留）、枚举单值与区间展开、常量、各类畸形输入、`install` 的跳过语义与不注册 `output` |
| `codec.rs` | `Params` 按名取参、无损转换与拒绝收窄、未声明参数报已声明列表、类型不符报两侧类型、字符串/字节互换与非法 UTF-8、`get_enum` 三种路径；`send_msg` / `call_msg` 的握手门禁、未知消息、参数不匹配、往返解码、超时、解码失败 |
| `identify.rs` | 单块与多块拼装（含 4 位序号回绕）、offset 错位、zlib 损坏、非 JSON、MCU 静默、zip bomb 上限、`Mcu::identify` 与 `Mcu::connect` 全流程 |
| `mod.rs` | MCU 构造、发送错误路径、`Drop` 中止接收任务并释放阻塞读 |

### `mcu::cmd`

| 模块 | 覆盖 |
|------|------|
| `identify.rs` | 无独立测试：两个视图由 `mcu::identify` 的端到端测试覆盖（手工构造的请求帧会校验 `args()` 的 id / offset / count，回应帧走 `IdentifyChunk::decode`） |
| `clock.rs` | 读取时钟、32 位回绕值、握手前失败、超时；另有不依赖 MCU 的 `ClockSync` 实现，验证 trait 作为测试缝可用 |

## 写 MCU 相关测试的两个要点

1. **帧要比得完整**：`TestDevice` 比对的是 `Frame`（seq + payload）。请求 payload 可以直接用 `Payload::push_*` 拼，或 `Parser::encode` 得到。
2. **序号必须对齐**：发送任务每批 +1、接收任务要求收到的帧从 0 开始递增，两者都只取低 4 位。第 i 次交换的请求与响应帧序号都应为 `i & 0xf`；超过 16 次交换会回绕，忘记 `& 0x0f` 会让第 17 帧起被静默丢弃（表现为超时）。

握手测试的现成写法见 `mcu/cmd/identify.rs` 的 `chunked_mappings`：它按块大小生成「请求帧 → 响应帧」映射，并用 `flate2` 现场压缩字典内容。

## 文档同步

改动 `msg` / `mcu` / `mcu::cmd` 的公开 API 或分层职责时，请同时更新本手册对应页面（见 [开发手册首页](README.md) 的目录）。

`cargo doc --no-deps --lib` 的警告数应与改动前一致（目前库里已有 10 条残留于 `frame.rs` / `kinematics` / `msg/parser.rs` / `traits.rs`）。新增模块时注意一个陷阱：**模块的文档链接是在它的 `mod` 声明所在作用域里解析的**，所以给 `mcu/mod.rs` 里的 `pub mod cmd;` 写 `///` 文档，会把 `cmd` 自己文档中的裸路径拉到 `mcu` 作用域（在那里 `identify` 指的是私有的传输模块）。`mcu/mod.rs` 因此用 `//` 而不是 `///` 注释这一行。

---

- [← 开发手册首页](README.md)
- [内部架构 ←](architecture.md)
