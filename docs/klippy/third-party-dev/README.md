# Klipper 第三方开发手册

面向外部客户端开发者（如 Fluidd、Mainsail、Moonraker、KlipperScreen 等）。描述 Klipper 主机端（klippy）暴露的 API 接口，供第三方工具与打印机通信。

## 目录

- [Klippy API 参考](api-reference.md) — Klipper API Server 完整接口定义

---

## 动手试一下

仓库里自带两个客户端子命令，用的是与本文档相同的协议（`0x03` 分隔的 JSON），
可以直接拿来对照实现。两者都接受与主机相同的 `-a/--api-server` 写法：socket 路径，
或 `tcp:<host>:<port>`。

下面用 `klipperx` 写；只想装客户端的那台机器可以用独立二进制 `klippy-client`，
子命令与参数完全一致（`klippy-client api …`、`klippy-client console …`）。

发一条请求并打印应答：

```console
$ klipperx api -a /tmp/klippy_uds list_endpoints
{
  "endpoints": [
    "emergency_stop",
    "gcode/firmware_restart",
    "gcode/help",
    "gcode/restart",
    "gcode/script",
    "gcode/subscribe_output",
    "info",
    "list_endpoints",
    "objects/list",
    "objects/query",
    "objects/subscribe",
    "query_endstops/status",
    "register_remote_method"
  ]
}
$ klipperx api -a /tmp/klippy_uds 'objects/query' '{"objects": {"toolhead": ["position"]}}'
$ klipperx api -a tcp:127.0.0.1:7125 info
```

进入交互式会话（在终端里是一扇窗口，下面是把它管到管道里的样子，两者同一次会话）：

```console
$ klipperx console -a /tmp/klippy_uds
Connected to unix:/tmp/klippy_uds.
Printer is ready — Printer is ready (v0.12.0-123-gabcdef, 4 core ARMv7 Processor rev 4 (v7l))
Type a request: a method name (`info`), a method and parameters
(`objects/query {"objects": {"toolhead": null}}`), or a whole JSON object.
An `id` is added when you leave it out; `"id": null` sends it unanswered.

Local commands:
  .help          this text
  .subscribe     watch every object (`objects/list` + `objects/subscribe`)
  .subscribe a b watch only the named objects
  .quit          leave (also ^D)

Replies print as `<id> (<method>) <result>`; pushes print as `< <message>`.
klippy> objects/query {"objects": {"toolhead": ["position"]}}
2 (objects/query)
  {
    "eventtime": 1234.5,
    "status": {
      "toolhead": {
        "position": [20.0, 30.0, 5.0, 0.0]
      }
    }
  }
klippy> .subscribe toolhead
Subscribed to 1 object(s); updates print as `<`.
< {"id": null, "method": "klippy:status", "params": {"eventtime": 1234.5, "status": {...}}}
klippy> .quit
Disconnected from unix:/tmp/klippy_uds.
```

会话里输入的三条形式（越往下越接近线上格式）：

| 输入 | 发出的请求 |
|------|-----------|
| `info` | `{"id": n, "method": "info", "params": {}}` |
| `objects/query {"objects": {"toolhead": null}}` | 同上，`params` 由方括号里的 JSON 对象给出 |
| `{"id": 9, "method": "gcode/script", "params": {"script": "M115"}}` | 原样发送 |

`id` 由客户端补上（除非你已经写了，包括写成 `null`）；`"id": null` 会被原样发送，
也就是按协议约定不要应答。回包按 `id` 与发出的方法名配对后打印，服务端主动推来的
消息（无 `id`）以 `<` 开头单独打印 —— 这也是 `.subscribe` 之后能一直看到状态更新
的原因。

> `klipperx console` 在终端里开的是全屏窗口（状态行 + 日志 + 输入行），要行式
> 输出用 `--plain` 或直接接管道，用法见[用户手册](../user-manual/client.md)。
>
> 上游自带的两个客户端（`scripts/whconsole.py`、`scripts/motan/data_logger.py`）
> 只支持 Unix socket，也只看自己那一件事，不能用来验证 TCP 监听。

---

- [← 文档首页](../../README.md)
