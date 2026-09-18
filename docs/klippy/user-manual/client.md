# 客户端使用

不用开 Fluidd / Mainsail，也能从命令行看打印机状态、跑 G-Code、盯着温度变化。
本页面向设备操作者，讲怎么把主机的 API 打开、怎么用它。

客户端叫 `klippy-client`，主机的 `klipperx` 里也带着同样两个命令
（`klipperx api` / `klipperx console`），两者的参数完全一致，下面用
`klippy-client` 写。

## 一、先让主机把 API 开起来

客户端连的是**主机的 API**，不是别的什么东西。启动主机时用 `-a` 指定监听位置：

```console
$ klipperx klippy ~/printer.cfg -a /tmp/klippy_uds        # Unix socket（默认形式）
$ klipperx klippy ~/printer.cfg -a tcp:127.0.0.1:7125     # TCP，供别的机器连
```

| `-a` 的写法 | 含义 |
|-------------|------|
| `/tmp/klippy_uds` | 在该路径建一个 Unix Domain Socket（与上游 Klipper 一致） |
| `unix:/tmp/klippy_uds` | 同上，写得更明确 |
| `tcp:127.0.0.1:7125` | 监听 TCP |
| `127.0.0.1:7125` | 同上（裸 `主机:端口` 简写） |

**不给 `-a` 就不启动 API**，客户端也就无从连接 —— 这与上游一致，是有意的：
一个默认在某处静默监听的程序会让人意外。

> **安全提醒**：API 没有任何认证，能连上的人就能操作打印机。TCP 监听只应开在
> 可信网络上（本机 `127.0.0.1` 或内网），不要直接暴露到公网。Unix socket 也
> 一样，靠文件权限保护。

## 二、拿客户端

从源码构建一个独立的客户端二进制（不依赖主机，也不需要配置文件）：

```console
$ cargo build --release -p klippy-client
$ ./target/release/klippy-client --help
Klipper API client
```

这个包不带主机的依赖，所以编得很快（几秒），产物也只有几 MB —— 拷到别的机器上
用它连打印机是可行的。

也可以直接用主机 CLI 里的同名命令（功能相同，只是那个二进制里还带着主机）：

```console
$ cargo run --release -- api -a /tmp/klippy_uds list_endpoints
$ cargo run --release -- console -a /tmp/klippy_uds
```

## 三、发一条请求：`klippy-client api`

适合脚本、定时任务、以及"就想看一眼"的场合。用法是
`klippy-client api -a <地址> <方法> [参数]`，参数是一个 JSON 对象：

```console
$ klippy-client api -a /tmp/klippy_uds list_endpoints
{
  "endpoints": [
    "list_endpoints"
  ]
}
```

| 选项 | 说明 |
|------|------|
| `-a, --api-server <ADDR>` | API 地址，写法同上一节的 `-a`（必填） |
| `--timeout <SECS>` | 等应答的上限，默认 10 秒。超时不会一直挂着 |

应答里只有 `result` 的内容；请求失败时，错误信息打到标准错误、退出码为 1，
所以脚本里可以直接 `if ! klippy-client api …; then …`。

## 四、交互式：`klippy-client console`

打开就进入一个会话，连上后先自我介绍（问一次主机的 `info`）：

```console
$ klippy-client console -a /tmp/klippy_uds
Connected to unix:/tmp/klippy_uds.
Printer is ready — Printer is ready (v0.12.0-123-gabcdef, 4 core ARMv7 Processor rev 4 (v7l))
Type a request: a method name (`info`), a method and parameters
(`objects/query {"objects": {"toolhead": null}}`), or a whole JSON object.
An `id` is added when you leave it out; `"id": null` sends it unanswered.

Local commands:
  .help          this text
  .subscribe     watch every object (`objects/list` + `objects/subscribe`)
  .subscribe a b watch only the named objects
  .quit          leave, after printing any reply still owed (also ^D)

Replies print as `<id> (<method>) <result>`; pushes print as `< <message>`.
klippy> list_endpoints
2 (list_endpoints)
  {
    "endpoints": [
      "list_endpoints"
    ]
  }
klippy> .quit
Disconnected from unix:/tmp/klippy_uds.
```

> 上面第一行 `Printer is ready — …` 是问 `info` 的结果；编号从 2 起是因为
> `info` 已经用掉了 1。等 `info` 端点写好后就是这个样子，目前那里会打印一行
> `info: webhooks: No registered callback for path 'info'`（见第五节的说明）。

### 一行就是一个请求

| 你敲的 | 发出去的东西 |
|--------|--------------|
| `info` | `{"id": 1, "method": "info", "params": {}}` |
| `objects/query {"objects": {"toolhead": ["position"]}}` | 同上，`params` 取自后面的 JSON |
| `{"id": 9, "method": "gcode/script", "params": {"script": "M115"}}` | 原样发送 |

`id` 是请求的编号，缺省时由客户端补上（这样应答能对上号）。写 `"id": null`
则按协议原样发送"不要应答"的消息 —— 这是协议本身的语义，不是客户端在偷懒。

### 本地命令

以 `.` 开头的行由客户端自己处理，不会发给主机：

| 命令 | 作用 |
|------|------|
| `.help` | 打印这一行的说明 |
| `.subscribe` | 先问主机有哪些对象（`objects/list`），再订阅全部，之后状态变化会持续打出来 |
| `.subscribe <对象> …` | 只订阅指定的对象，例如 `.subscribe toolhead extruder heater_bed` |
| `.quit`（或 `.exit`） | 退出会话；已经在路上的应答会先打出来 |

想要停止订阅，直接 `.quit` 即可：协议规定客户端靠断开连接来取消订阅。

### 输出怎么读

| 前缀 | 含义 |
|------|------|
| `1 (list_endpoints) { … }` | 编号为 1 的请求的应答，括号里是当初发的方法名 |
| `! 2 (objects/query) Missing Argument [objects]` | 请求失败（这里是缺参数），后面是主机的错误说明 |
| `< {"id": null, "method": "klippy:status", …}` | 主机主动推来的消息，不是你问了才有的 —— `.subscribe` 之后就会看到 |

`^D`（或让管道结束）与 `.quit` 等效，只是前者在输入用完后还会等一小会儿，
把欠着的应答打完再退出。所以 `printf 'list_endpoints\n' | klippy-client console -a …`
也能用；不过真要写脚本，用 `klippy-client api` 更直接。

## 五、能做什么

客户端会什么，取决于**主机实现了哪些端点**。完整清单与每个端点的参数、返回字段见
[第三方开发手册的 API 参考](../third-party-dev/api-reference.md)。

> **当前状态**：主机只实现了 `list_endpoints`，所以下表里除它以外的操作现在都会
> 得到 `webhooks: No registered callback for path '…'`。那不是客户端的问题，是那
> 些端点还没写（清单见 [开发手册](../developer-manual/README.md)）。表里先列出
> 各操作**将来**的敲法。

| 想做什么 | 在 `console` 里敲 |
|----------|-------------------|
| 看有哪些端点和对象 | `list_endpoints`、`objects/list` |
| 看打印机状态、版本、CPU | `info` |
| 看当前坐标 | `objects/query {"objects": {"toolhead": ["position"]}}` |
| 看温度 | `objects/query {"objects": {"extruder": ["temperature", "target"]}}` |
| 实时盯状态 | `.subscribe toolhead extruder heater_bed` |
| 跑 G-Code | `gcode/script {"script": "M115"}` |
| 发一条不等待的 G-Code | `gcode/script {"script": "M104 S200"}` |
| 紧急停止 | `emergency_stop` |
| 重载配置并重启主机 | `gcode/restart` |
| 重启固件与主机 | `gcode/firmware_restart` |
| 暂停 / 恢复 / 取消 | `pause_resume/pause`、`pause_resume/resume`、`pause_resume/cancel` |

对应的单条命令形式（方便写脚本）：

```console
$ klippy-client api -a /tmp/klippy_uds objects/query '{"objects": {"toolhead": ["position"]}}'
$ klippy-client api -a /tmp/klippy_uds gcode/script '{"script": "M115"}'
```

## 六、连不上怎么办

| 现象 | 多半是 | 处理 |
|------|--------|------|
| `cannot connect to unix socket /tmp/klippy_uds: Connection refused`（或文件不存在） | 主机没在跑，或者启动时没给 `-a` | 看主机日志里有没有 `API server listening on …` 那一行 |
| 连接时 `Permission denied` | socket 文件的权限 | 用与主机相同的用户运行客户端 |
| `--api-server: unknown scheme 'http://'` | 把 Moonraker 的 HTTP 端口当成 API 地址了 | 换成 socket 路径或 `tcp:主机:端口`；`7125` 是 Moonraker 的，不是这里的 |
| 应答是 `webhooks: No registered callback for path '…'` | 主机没有这个端点（见上一节的状态说明） | 用 `list_endpoints` 看主机现在有什么 |
| 敲了请求，一直没有任何输出 | 请求里写了 `"id": null`，按约定不会有应答 | 去掉 `id`，或让客户端自己补 |
| `the API server closed the connection` | 主机重启或关机了（`RESTART`、`FIRMWARE_RESTART`、故障停机） | 客户端不会自动重连，重新打开一次即可 |
| `no reply to '…' within 10s` | 主机在，但那个端点没应答 | 多半是端点卡住或没实现；`--timeout` 可以调 |

---

- [← 用户手册首页](README.md)
