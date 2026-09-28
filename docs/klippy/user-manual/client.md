# 客户端使用

不用开 Fluidd / Mainsail，也能从命令行看打印机状态、跑 G-Code、盯着温度变化。
本页面向设备操作者，讲怎么把主机的 API 打开、怎么用它。

客户端叫 `klippy-client`，主机的 `klipperx` 里也带着同样两个命令
（`klipperx api` / `klipperx console`），两者的参数完全一致，下面用
`klippy-client` 写。

## 一、先让主机把 API 开起来

客户端连的是**主机的 API**，不是别的什么东西。启动主机时用 `-a` 指定监听位置：

```console
$ klipperx ~/printer.cfg                                  # 默认：/tmp/klippy_uds
$ klipperx ~/printer.cfg -a /run/klipper/api              # 自己指定路径
$ klipperx ~/printer.cfg -a tcp:127.0.0.1:7125            # TCP，供别的机器连
```

跑主机是 `klipperx` 的**默认动作**，所以 `klippy` 这个子命令名可以省：上面两行与
`klipperx klippy ~/printer.cfg -a …` 完全等价（文档里两种写法都会出现）。其余
子命令（`api`、`console`）不能省。

| `-a` 的写法 | 含义 |
|-------------|------|
| **不给** | 用默认路径 `/tmp/klippy_uds`（主机与客户端共用同一个默认值） |
| `/tmp/klippy_uds` | 在该路径建一个 Unix Domain Socket（上游 Klipper 文档里的示例路径） |
| `unix:/tmp/klippy_uds` | 同上，写得更明确 |
| `tcp:127.0.0.1:7125` | 监听 TCP |
| `127.0.0.1:7125` | 同上（裸 `主机:端口` 简写） |
| `''`（空值） | **不启动 API**（只给主机用：客户端必须有地址） |

**默认值是统一的**：主机不给 `-a` 就监听 `/tmp/klippy_uds`，而 `klipperx api`、
`klipperx console`、`klippy-client` 不给 `-a` 就去找同一个路径 —— 所以两边都不用
被交代两次，`klipperx console` 一条命令就能连上刚起的主机。

不想要 API 就给空值（`-a ''`）：API 没有认证，一个不需要它的主机不该被迫监听。

> 从上游 Klipper 过来的话：`-I/--input-tty`（把 G-Code 输入挂在一个 pty 上，
> 上游默认 `/tmp/printer`）以及 `-l`（日志文件）、`-i`/`-o`（调试输入输出）都还
> 没有实现，所以这里也没有这些选项 —— 与其留一个什么都不做的开关，不如没有。

### 顺便开个窗口

主机可以自己开一扇客户端窗口，不用另外起进程、也不用 `-a`：

```console
$ klipperx klippy ~/printer.cfg --tui
```

它会对着**自己**跑一个客户端（走进程内的管道，不经过 socket），于是窗口里既有
客户端发出去的请求和主机的应答，也有主机自己的日志 —— 两者按发生顺序混在同一
条日志里，这是把两边放在一起看的唯一办法：

```text
◌ state unknown
INFO  API server listening on unix:/tmp/klippy_uds
DEBUG Klippy process started with 1 sections
Connected to this host (in-process).
1 > {"id":1,"method":"info","params":{}}
info: webhooks: No registered callback for path 'info'
WARN  api: dropping malformed request (invalid JSON …): not json
```

要点：

- 窗口就是这次运行的界面：**关掉窗口，主机也跟着停**。想让它一直跑，就别用
  `--tui`（或另开一个终端跑 `klippy-client console` 连它）。
- 加了 `--tui` 之后主机自己的日志不再往 stdout 写（否则会糊在窗口上），全都
  进窗口的日志区；窗口关掉后如果主机还在跑，日志会回到 stdout。
- `--verbose` 打开 DEBUG，所以主机更啰嗦时窗口也会显示那些细节。
  更多日志控制方式见 [日志与调试](logging.md)。
- 没有终端（比如 systemd 里）时 `--tui` 只打印一行警告，主机照常无窗口运行。
- 这个选项属于 `klipperx`，**独立二进制 `klippy` 没有它**：窗口是客户端，会带进
  一整套终端界面库，而只负责提供 API 的 `klippy` 用不到 —— 不装它的 `klippy`
  因此小一号（release 9.1 MB，带窗口的 `klipperx` 是 11.2 MB，2026-09-23 实测）。要在独立主机上加
  窗口，另开一个终端跑 `klippy-client console -a …` 即可。

> **安全提醒**：API 没有任何认证，能连上的人就能操作打印机。TCP 监听只应开在
> 可信网络上（本机 `127.0.0.1` 或内网），不要直接暴露到公网。Unix socket 靠文件
> 权限保护，而默认路径在 `/tmp`（沿用上游的示例路径）—— 共享目录、路径可猜，
> 同一个机器上的其他用户因此也能连上。多用户机器请改成自己的路径，或者用
> `-a ''` 干脆不开。

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
```

| 选项 | 说明 |
|------|------|
| `-a, --api-server <ADDR>` | API 地址，写法同上一节的 `-a`（必填） |
| `--timeout <SECS>` | 等应答的上限，默认 10 秒。超时不会一直挂着 |

应答里只有 `result` 的内容；请求失败时，错误信息打到标准错误、退出码为 1，
所以脚本里可以直接 `if ! klippy-client api …; then …`。

## 四、交互式：`klippy-client console`

在终端里打开一扇窗口：上面是打印机状态，中间是实时日志，下面是你敲命令的地方。
**推送**（订阅来的状态更新）会一直往日志里追加，不跟你抢提示符 —— 这正是窗口
存在的理由。

```text
● ready · Printer is ready
Connected to unix:/tmp/klippy_uds.
> id: 1
  method: info
  params: {}
> id: 2
  method: list_endpoints
  params: {}
< id: 2
  result:
    endpoints:
    - emergency_stop
    - gcode/firmware_restart
    - …（完整清单随实现变化，以 `list_endpoints` 实际应答为准）
klippy> objects/query {objects: {toolhead: [position]}}
Enter send · ↑↓ history · PgUp/PgDn/Home/End log · ^G g-code · Esc×3 stop · .help · ^C quit
```

第一行是状态行（左边那个符号：`●` 就绪、`◌` 启动中或状态未知、`▲` 出错/停机、
`✕` 已断开），最后一行是按键提示，中间是日志，倒数第二行是你的输入。

> 每条消息以 `<`（收到）或 `>`（发出）开头、空一格再写正文，正文默认是 **YAML**
> （树状、不用给每个键加引号）；`.json` / `.yaml` 在窗口里切换正文格式，标记与
> 空格不变。相邻两条消息用两种颜色交替（错误仍是红色）。行模式（`--plain` 或管道）不这样：
> 它保持“每件事一行”的紧凑 JSON，便于重定向和 grep。

### 按键

| 按键 | 作用 |
|------|------|
| `Enter` | 发送这一行 |
| `↑` / `↓` | 翻之前敲过的命令；日志回滚后改为逐行滚动（`Ctrl+↑` / `Ctrl+↓` 始终逐行） |
| `PgUp` / `PgDn` | 日志往上 / 往下翻一页（`Ctrl+↑` / `Ctrl+↓` 一次一行） |
| `←` `→` `Backspace` `Delete` | 行内编辑 |
| `Home` / `End` | 日志最顶 / 最新 |
| `Ctrl+A` / `Ctrl+E` | 跳到行首 / 行尾 |
| `Ctrl+U` | 清掉这一行（不记进历史） |
| `Ctrl+G` | 在请求模式与 g-code 模式之间切换（同 `.gcode`） |
| `Ctrl+S` | 把鼠标交给终端——可以拖选、复制文字；按任意键收回（同 `.mouse`） |
| `Ctrl+L` | 清空日志 |
| `Ctrl+C` / `Ctrl+D` | 立刻退出（欠着的应答会先打完） |
| `Esc` | **急停**：连按 3 下（0.8 s 内）发 `emergency_stop`，打印机停机直到重启；**单击不做任何事**——退出请用 `Ctrl+C` / `Ctrl+D` |

`Esc` 在这台窗口里**只**做急停——急停必须能在键盘上盲打出来，而"退出"有更明确的
键（`Ctrl+C` / `Ctrl+D`，且欠着的应答会先打完），所以两者不共用一个键。按 1–2 下时
下方提示会数 `emergency stop 1/3 · Esc again · any other key cancels`；**按任意其它
键即作废**，窗口内不再按也一样（计数过期即消失，单击本身不做任何事）。

翻看旧日志时，最下面那行会提示 `viewing older entries · N lines back`（回滚了
多少行）；按 `End` 或 `PgDn` 回到底部，按 `Home` 回到顶端，或者直接敲下一条
命令也会回到最新处。回滚之后 `↑` / `↓` 也能逐行滚动（`Ctrl+↑` / `Ctrl+↓`
在任何时候都逐行）。滚动的单位是**换行后的一行**，所以一条被折成好几行的长
消息也能逐行看。

日志区最右边一列是滚动条：占满一屏时显示滑块（`█`）与轨道（`│`），停在底部时
滑块贴底、`Home` 时回到顶端；不足一屏时那列留空。这一列始终给滚动条留着，所以
日志正文比窗口窄一列，也不会在超过一屏的瞬间左右跳。鼠标点或拖这一列可以直接
跳到日志对应的位置（鼠标滚轮则是逐行）；拖动时指针移出这一列、甚至移出日志区，
拖动也不会中断，只按指针所在的行定位。

滚轮和滚动条要占用鼠标，所以窗口默认把鼠标抓在手里，终端的拖选用不了。要选文字：
**在日志正文上点一下**（或按 `Ctrl+S` / 敲 `.mouse`）把鼠标交回终端，然后拖选、用终端
自己的方式复制；之后**按任意键**（翻页、打字都算）鼠标就自动收回，不用记快捷键。
交出鼠标期间**日志会冻住**（新日志不再推动画面），否则终端的选区会跟着文字跑偏；
收回后画面停在原处，按 `End` 回到底部继续跟随。注意点的那一下只用来交出鼠标，选区
要下一次拖动才开始（终端只认从头看到的那次拖拽）。窗口用的是备用屏幕，能选的是屏
幕上现在看得到的那些字（日志比一屏长时，先滚到你要看的那段再点）。

窗口用的是终端的备用屏幕，所以**退出之后你终端原本的 scrollback 里没有这些
内容**。想要能滚回去、能重定向、能 grep 的输出，就用下一节的行模式。

### G-code 模式

按 `Ctrl+G`（或敲 `.gcode`）切换「请求模式」与「g-code 模式」，输入提示符会从
`klippy>` 变成 `gcode>`。g-code 模式下整行都当作脚本发给 `gcode/script`，所以直接敲命令：

```
gcode> SET_PIN PIN=fan VALUE=1
```

第一次进入 g-code 模式时会自动订阅一次 `gcode/subscribe_output`，于是命令的
`// …` 信息与 `!! …` 错误会以推送的形式出现在日志里（不订阅的话只能看到命令的应答，
`respond_info` 看不到）。`.` 开头的本地命令在两种模式下都有效（例如 `.reload`、`.quit`、`.help`），所以
`.quit`、`.help` 不会被当成 g-code 发出去。

### 一行就是一个请求

参数与整条请求都写 YAML（JSON 是 YAML 的子集，所以原来那样写也照样能用）：

| 你敲的 | 发出去的东西 |
|--------|--------------|
| `info` | `{"id": 1, "method": "info", "params": {}}` |
| `objects/query {objects: {toolhead: [position]}}` | 同上，`params` 取自后面的 YAML |
| `{id: 9, method: gcode/script, params: {script: M115}}` | 原样发送 |

`id` 是请求的编号，缺省时由客户端补上（这样应答能对上号）。写 `{id: null, …}`
则按协议原样发送"不要应答"的消息 —— 这是协议本身的语义，不是客户端在偷懒。

### 本地命令

以 `.` 开头的行由客户端自己处理，不会发给主机。这类行在输入时提示符会变成
`local>`：

| 命令 | 作用 |
|------|------|
| `.help` | 把这一行的说明打进日志 |
| `.subscribe` | 先问主机有哪些对象（`objects/list`），再订阅全部，之后状态变化会持续出现 |
| `.subscribe <对象> …` | 只订阅指定的对象，例如 `.subscribe toolhead extruder heater_bed` |
| `.firmware_restart` | 重启固件并重载配置（`gcode/firmware_restart`）；连接不断，随后会有一段 `startup` |
| `.reload`（或 `.reload_config`） | 从磁盘重新读取配置文件并重启主机（`gcode/restart`）；`printer.cfg` 改完用它生效 |
| `.quit`（或 `.exit`） | 退出；已经在路上的应答会先打出来 |
| `.yaml` / `.json` | **仅窗口**：把消息正文在 YAML（默认）与紧凑 JSON 之间切换，`<`/`>` 标记不变 |

想要停止订阅，直接退出即可：协议规定客户端靠断开连接来取消订阅。

`.reload` 与 `.firmware_restart` 都会让主机**重新从磁盘读配置文件**（改完文件再发一次即可，
不必重启主机进程）。mux 端点（`*/dump_*`）的实例随配置一起注销并在下一轮重新注册，所以
含 `[load_cell]`、`[adxl345]` 等节的配置反复重载也不会累积、不会卡在 `already registered`。

### 日志怎么读

| 前缀 | 含义 |
|------|------|
| `> id: 2` | 客户端**发出去**的请求，正文是 YAML；消息行用青 / 白两色交替，便于逐条分辨（见[日志与调试](logging.md)） |
| `< id: 2` | 编号为 2 的请求的应答；应答里带 `id`，按它与上面那条对应（`method` 不再重复显示） |
| `! 3 (objects/query) Missing Argument [objects]` | 请求失败（这里是缺参数），后面是主机的错误说明（红色） |
| `< method: klippy:status` | 主机主动推来的消息（没有 `id`）—— 订阅之后就会看到 |

`.json` 只改正文的写法：上面的前缀变成 `> {"id":2,…}` / `< {"id":2,…}`，
标记、空格与颜色交替都不变。

### 没有终端的时候

窗口需要终端。输入或输出只要有一头是管道（`printf … | klippy-client console`），
就自动换成**行模式**：一次一行、没有提示符、没有窗口，输出可以直接重定向。
在终端里想强制用它，加 `--plain`：

```console
$ printf 'list_endpoints\n' | klippy-client console -a /tmp/klippy_uds
Connected to unix:/tmp/klippy_uds.
info: webhooks: No registered callback for path 'info'
2 (list_endpoints) {"endpoints":["emergency_stop","gcode/firmware_restart",…]}
Disconnected from unix:/tmp/klippy_uds.
$ klippy-client console --plain -a /tmp/klippy_uds
```

输入用完之后，客户端会再多等一小会儿（最多一秒）把欠着的应答打完再退出 ——
管道会把所有行一次性送到，主机还没看见第一条，不等就会把答案丢掉。

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
| 重载配置并重启主机（从磁盘重读） | `gcode/restart` |
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
| `the API server closed the connection` | 主机进程没了（`^C`/`SIGTERM`、`--tui` 窗口关闭、致命错误退出） | 客户端不会自动重连，重新打开一次即可。注意 `RESTART`/`FIRMWARE_RESTART` **不断连接**：主机就地重建打印机，连接与订阅全程有效 |
| `no reply to '…' within 10s` | 主机在，但那个端点没应答 | 多半是端点卡住或没实现；`--timeout` 可以调 |

---

- [← 用户手册首页](README.md)
