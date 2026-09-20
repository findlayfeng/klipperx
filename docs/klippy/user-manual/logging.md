# 日志与调试

KlipperX 的日志系统基于 Rust 的 `tracing` 框架，支持多种级别和多种开启方式。

## 日志级别

| 级别 | 含义 | 典型输出 |
|------|------|----------|
| `ERROR` | 严重错误，导致操作失败 | 连接断开、配置解析错误 |
| `WARN` | 警告，主机仍可继续运行 | 请求格式异常、超时 |
| `INFO` | 正常信息（**默认级别**） | 启动完成、API 监听地址 |
| `DEBUG` | 调试细节，仅在排查问题时开启 | 命令与参数、配置节数、内部状态变化 |
| `TRACE` | 最详细的追踪信息 | 接口帧的字节内容 |

默认情况下只显示 `INFO` 及以上级别的日志。

## 开启详细日志

有三种方式，按优先级从高到低排列：

### 1. `--verbose` 命令行参数

最简单的方式，适用于 `--tui` 窗口模式：

```console
$ klipperx ~/printer.cfg --tui --verbose
```

这会强制将所有日志设为 `DEBUG` 级别。**优先级最高**，会忽略 `RUST_LOG` 环境变量。

### 2. `RUST_LOG` 环境变量

更灵活的方式，支持按模块精细控制：

```bash
# 全部设为 debug
RUST_LOG=debug klipperx ~/printer.cfg

# 只开启 klipperx 主库的 debug
RUST_LOG=klipperx=debug klipperx ~/printer.cfg

# 开启特定模块
RUST_LOG=klipperx=debug,klippy_client=info klipperx ~/printer.cfg

# 最详细的 trace 级别
RUST_LOG=trace klipperx ~/printer.cfg
```

> **注意**：`RUST_LOG` 只在没有 `--verbose` 参数时生效。如果传了 `--verbose`，环境变量会被忽略。

### 3. 配置文件（`.env` 或 shell 导出）

如果经常需要调试，可以把 `RUST_LOG` 写到配置文件中：

```bash
# 在 ~/.bashrc 或 ~/.zshrc 中
export RUST_LOG=debug

# 或在 systemd 服务中
[Service]
Environment=RUST_LOG=debug
```

## `--tui` 模式下的日志

当使用 `--tui` 启动时，日志的行为有所不同：

- 主机日志不再输出到终端 stdout，而是写入窗口内的**日志面板**
- 客户端请求和主机日志按时间顺序混合显示
- `--verbose` 会让 DEBUG 日志也出现在日志面板中

```text
◌ state unknown
INFO  API server listening on unix:/tmp/klippy_uds
DEBUG Klippy process started with 1 sections
Connected to this host (in-process).
1 > {"id":1,"method":"info","params":{}}
info: webhooks: No registered callback for path 'info'
WARN  api: dropping malformed request (invalid JSON …): not json
```

## 日志输出说明

日志采用 `LEVEL  消息内容` 的格式：

- `INFO` — 正常操作信息，如服务启动、配置加载完成
- `WARN` — 异常情况但主机仍在运行，如请求格式错误
- `ERROR` — 严重错误，可能导致功能不可用
- `DEBUG` — 内部调试信息，如进出的命令与参数、配置节数量、状态变化等
- `TRACE` — 接口层帧的字节内容（按帧结构分段，十六进制）

在 `DEBUG` 级别下，日志会包含模块名称前缀，如 `klippy`、`api`、`webhooks` 等，方便定位问题来源。

### 接口帧日志

接口层（`interface`）的日志按级别递增显示不同细节：

| 级别 | 接口日志示例 |
|------|-------------|
| `INFO` | `serial port /dev/ttyACM0 open at 250000 baud` |
| `DEBUG` | `sent 15 bytes to /dev/ttyACM0`、`received 32 bytes from /dev/ttyACM0` |
| `TRACE` | `tx frame [serial0]: 0a11 \| 01020304 05 \| 31d87e` |

`DEBUG` 级别只显示字节数量，`TRACE` 级别才会打印帧的实际字节内容。

`TRACE` 的字节按帧的结构分成三段，用 `|` 隔开：

```text
tx frame [serial0]: 0a11 | 01020304 05 | 31d87e
```

- `0a11` —— 头：长度 `0x0a`（10 字节，含头尾）与序号字节 `0x11`（低 4 位是序号，`0x10` 是 DEST 标志）
- `01020304 05` —— 载荷
- `31d87e` —— 尾：CRC（2 字节）与 SYNC（`0x7e`）

每一段内每 4 字节为一组、组间一个空格；不足 4 字节的余数整段连写（如上面的
`01020304 05`）。不是完整一帧的字节（读到半帧、两帧粘在一起）就只按 4 字节
一组打印。

CAN 承载的就是同一份 serial 字节流，所以**整块**也按上面的格式打印；但它另外
给每个 CAN 帧单独打一行**分片**，带上仲裁 id：

```text
rx can frame [can0:0x30a]: id 0x30b data 01020304 05060708
rx frame [can0:0x30a]: 0a11 | 01020304 05 | 31d87e
```

接收时先逐片打分片行、凑齐后打整块行；发送时先打整块行、再逐片打分片行（顺序
相反，内容相同）。整块只占一个 CAN 帧时两行也都会打。用同一套格式而不是另造一套
CAN 专用的打印，原因见[内部架构](../developer-manual/architecture.md) 的「传输层」
一节。

### 命令日志

`mcu` 层在 `DEBUG` 下把进出的每条命令连同参数打成一行，参数名取自固件字典
（也就是报文格式串里的名字）：

```text
send get_clock oid=1
recv clock clock=1234567
send set_pin oid=3 value=1
```

值的写法：整数十进制、字符串原样、字节按 4 字节一组的十六进制（与帧日志同一套）。
字典里没有的消息只打名字。只有 `DEBUG` 及以上才会查字典、拼字符串。

### 窗口里的级别前缀

窗口（`--tui`）的日志面板只有 `DEBUG` / `INFO` / `WARN` / `ERROR` 四种前缀，
**`TRACE` 归入 `DEBUG`**，所以帧字节那种 `TRACE` 行在窗口里前缀是 `DEBUG`。
想拼一拼时看时间戳或模块名可以区分，但只看前缀区分不了。

还有一个容易踩的点：`--verbose` 把过滤器设成 `debug`，**TRACE 事件因此被过滤掉**，
在窗口里看不到帧字节。要看帧字节用 `RUST_LOG`：

```bash
RUST_LOG=klipperx=trace klipperx ~/printer.cfg --tui
```

## 常见问题

### 我想看更详细的日志但不知道设什么

```bash
RUST_LOG=debug klipperx ~/printer.cfg
```

这是最通用的做法，会显示所有模块的调试信息。

### 我只想跟踪某个模块的日志

```bash
RUST_LOG=klipperx=debug klipperx ~/printer.cfg
```

常见的模块名包括 `klippy`（主机核心）、`api`（API 服务器）、`webhooks` 等。

### `--verbose` 和 `RUST_LOG` 有什么区别？

- `--verbose` 是硬编码的 `debug` 级别，适用于 `--tui` 窗口模式，最简单直接
- `RUST_LOG` 更灵活，可以按模块设置不同级别，也适合 systemd 等服务场景
- 两者冲突时 `--verbose` 优先
