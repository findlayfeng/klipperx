# 日志与调试

KlipperX 的日志系统基于 Rust 的 `tracing` 框架，支持多种级别和多种开启方式。

## 日志级别

| 级别 | 含义 | 典型输出 |
|------|------|----------|
| `ERROR` | 严重错误，导致操作失败 | 连接断开、配置解析错误 |
| `WARN` | 警告，主机仍可继续运行 | 请求格式异常、超时 |
| `INFO` | 正常信息（**默认级别**） | 启动完成、API 监听地址 |
| `DEBUG` | 调试细节，仅在排查问题时开启 | 配置节数、内部状态变化 |
| `TRACE` | 最详细的追踪信息 | 内部函数调用链 |

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
- `DEBUG` — 内部调试信息，如配置节数量、状态变化等

在 `DEBUG` 级别下，日志会包含模块名称前缀，如 `klippy`、`api`、`webhooks` 等，方便定位问题来源。

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
