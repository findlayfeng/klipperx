# Klippy API 参考文档

本文档描述 Klipper 主机端（klippy）通过 Unix Domain Socket 暴露的 API 接口。外部客户端（如 Fluidd、Mainsail、Moonraker）通过这些接口与打印机通信。

## 通信协议概述

### 连接方式

- **传输层**：Unix Domain Socket（默认）；本实现也支持 TCP 监听，两种传输承载完全相同的协议
- **消息格式**：JSON
- **消息分隔符**：`\x03` (ASCII ETX)
- **编码**：JSON（紧凑格式，字段间无空格）

> 监听位置由主机启动参数 `-a/--api-server` 决定：不给就用统一的默认路径
> `/tmp/klippy_uds`（Unix Domain Socket），写成 `tcp:<host>:<port>` 则监听 TCP，
> 给空值则**不启动服务**。主机与客户端用的是同一个默认值，所以两边都不必被交代
> 两次。TCP 监听没有认证，只应开在可信网络上。

### 请求格式

```json
{"id": <string|null>, "method": "<endpoint_path>", "params": { ... }}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | string/null | 请求标识符，用于响应匹配；设为 `null` 表示不期望响应 |
| `method` | string | 端点路径，如 `info`、`objects/query`、`gcode/script` |
| `params` | object | 请求参数，具体取决于端点 |

### 响应格式

成功响应：
```json
{"id": "<请求id>", "result": { ... }}
```

错误响应：
```json
{"id": "<请求id>", "error": {"error": "WebRequestError", "message": "错误信息"}}
```

无 id 的请求（`id` 为 `null` 或省略）：

服务端**不发送任何响应**——收到 `id` 为 `null` 的请求后只执行、不回包。这类请求用于「即发即忘」场景，例如 `gcode/script`、`emergency_stop`；需要结果时应显式传入 `id`。

### 服务端推送（Subscription）

客户端可以通过 `response_template` 参数订阅服务端推送。推送消息由服务端把 `params` 合并进客户端提供的模板生成：`params` 放入 `params` 字段，模板中的其他字段（如 `id`、`method`）原样保留。因此推送格式**完全取决于模板**，服务端不会自动补充 `id`，也不会覆盖模板中的 `method`。推荐模板：

```json
{"id": null, "method": "<method>"}
```

合并后即：

```json
{"id": null, "method": "<method>", "params": { ... }}
```

> 若模板中不含 `method`，推送将不带 `method` 字段。

---

## 端点列表

> **实现状态**（截至 2026-09-24，权威清单见主机侧 `api/endpoints/mod.rs` 的状态表）：
> 下文第 1–12、16 节（`info` / `emergency_stop` / `list_endpoints` / `register_remote_method` /
> `objects/*` / 五个 `gcode/*` / `query_endstops/status`）**已实现**；第 13–15 节
> （`pause_resume/*`）与第 17–18 节（`bed_mesh/dump_mesh`、`*/dump_*`）**部分落地**——截至
> 2026-09-24（wave-2）：`pause_resume` 对象与 `[buttons]` 最小实现已落地，但 `pause_resume/*`
> 端点仍未注册；`*/dump_*` 的 mux 机制与 `ldc1612` / `adxl345` / `mpu9250` 三个消费者已落地，
> `bed_mesh/dump_mesh` 与其余 dump 端点随各自的 extras 落地。调用未实现的端点会得到 `unknown method` 错误。本文描述的是目标形状，
> 实现随模块推进。

### 1. `info` — 获取打印机状态信息

获取打印机的当前状态、版本和系统信息。

**请求：**
```json
{"method": "info", "params": {"client_info": {"name": "Moonraker", "client_version": "v0.8.0"}}}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `client_info` | object | 否 | 客户端身份信息，用于服务端日志记录 |

**响应：**
```json
{
  "state": "ready",
  "state_message": "Printer is ready",
  "hostname": "klipper",
  "klipper_path": "/home/pi/klipper",
  "python_path": "/usr/bin/python3",
  "process_id": 12345,
  "user_id": 1000,
  "group_id": 1000,
  "log_file": "/tmp/klippy.log",
  "config_file": "/home/pi/printer.cfg",
  "software_version": "v0.12.0-123-gabcdef",
  "cpu_info": "4 core ARMv7 Processor rev 4 (v7l)"
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 打印机状态：`startup` / `ready` / `shutdown` / `error` |
| `state_message` | string | 状态描述信息 |
| `hostname` | string | 主机名 |
| `klipper_path` | string | Klipper 安装路径；本主机没有 Klipper 源码树，报一个**不存在的路径**，理由见下 |
| `python_path` | string | 解释型主机就报自己的解释器路径；本主机不是解释型的，报一个**不存在的路径**，理由见下 |
| `process_id` | int | 进程 ID |
| `user_id` / `group_id` | int | 运行用户/组 ID |
| `log_file` | string/null | 日志文件路径；未指定日志文件时为 `null` |
| `config_file` | string | 配置文件路径 |
| `software_version` | string | Klipper 软件版本 |
| `cpu_info` | string | CPU 描述字符串，如 `"4 core ARMv7 Processor rev 4 (v7l)"`（非对象，客户端直接展示） |

> **`klipper_path` / `python_path` 为什么是不存在的路径**：这两个键 Moonraker 都在 `_save_path_info` 里直接下标取值（缺了会 `KeyError`），而它只在 `klipper_path` 与 `python_path` 的父目录**都存在**时才给 Klipper 更新项装上真正的 deploy 类（`components/update_manager/update_manager.py`：`os.path.exists(kcfg["path"]) and os.path.exists(kcfg["env"])`），否则退回 `BaseDeploy`（no-op）。本主机没有 Klipper 源码树也没有解释器，报不存在的路径正好让它退化为 no-op。`python_path` 另有一层约束：这个字段只有 Moonraker 在用（它自己的文档也标注为 "moonraker use only"），而它把它当作 Klipper 安装的 **virtualenv 解释器**：填进更新项的 `env`，再推导出 `<venv>/bin/python` 跑 `-m pip`（`update_manager/app_deploy.py` 的 `_configure_virtualenv`）。所以存在且可执行、但不是 `<venv>/bin/python` 形状的路径会让 Moonraker **启动失败**（`Invalid virtualenv at path …`，因为 `<parent>/bin/activate` 不存在）—— 这正是不能报本主机自己二进制的原因。

---

### 2. `emergency_stop` — 紧急停止

立即将打印机切换到 shutdown 状态。

**请求：**
```json
{"method": "emergency_stop"}
```

**响应：** 无参数响应（`{}`）

---

### 3. `list_endpoints` — 列出所有可用端点

获取当前注册的所有 API 端点路径。

**请求：**
```json
{"method": "list_endpoints"}
```

**响应：**
```json
{
  "endpoints": ["info", "emergency_stop", "objects/list", "objects/query", ...]
}
```

---

### 4. `register_remote_method` — 注册远程方法

客户端注册一个远程方法，服务端可通过此方法向该客户端推送事件。

**请求：**
```json
{
  "method": "register_remote_method",
  "params": {
    "remote_method": "on_printer_state_change",
    "response_template": {
      "id": null,
      "method": "printer:state_change"
    }
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `remote_method` | string | 是 | 远程方法名称 |
| `response_template` | object | 是 | 推送消息的模板格式 |

> 服务端推送时，消息为 `{"params": {...}}` 与 `response_template` 合并（同 Subscription 机制）。若注册后连接已关闭，服务端会清理该注册；当某方法已无活动连接时，服务端内部调用会报 `No active connections for method '<method>'`。

---

## Objects 相关端点

### 5. `objects/list` — 列出所有可查询对象

获取所有可查询状态的打印机对象列表。列表随已加载模块动态变化，下列仅为示例：

**请求：**
```json
{"method": "objects/list"}
```

**响应：**
```json
{
  "objects": [
    "configfile", "gcode_move", "toolhead", "webhooks", "fan",
    "heater_bed", "extruder", "bed_mesh", "print_stats",
    "pause_resume", "query_endstops", "virtual_sdcard", ...
  ]
}
```

---

### 6. `objects/query` — 查询对象状态

一次性查询一个或多个对象的状态。

**请求：**
```json
{
  "method": "objects/query",
  "params": {
    "objects": {
      "toolhead": ["position", "max_velocity", "max_accel"],
      "webhooks": null,
      "extruder": ["temperature", "target", "power"]
    }
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `objects` | object | 是 | 对象名到字段列表的映射；设为 `null` 表示查询全部字段 |

**响应：**
```json
{
  "eventtime": 12345.678,
  "status": {
    "toolhead": {
      "position": [20.0, 30.0, 5.0, 0.0],
      "max_velocity": 300.0,
      "max_accel": 3000.0
    },
    "webhooks": {
      "state": "ready",
      "state_message": "Printer is ready"
    },
    "extruder": {
      "temperature": 200.5,
      "target": 200.0,
      "power": 0.667
    }
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `eventtime` | float | 查询事件时间戳 |
| `status` | object | 对象状态数据，键为对象名 |

三条例外行为（与上游一致，客户端需要知道）：

- **未注册的对象不报错**：它在 `status` 里回一个空对象；若请求里给了字段名，那些字段各回 `null`。
- **对象没有的字段回 `null`**，而不是省略，因此「没请求」与「不存在」可以区分。
- **`null` 字段列表**取该对象当下的全部字段；每个对象的状态只被问一次，同一次查询里的所有对象共用同一个 `eventtime`。

---

### 7. `objects/subscribe` — 订阅对象状态

订阅一个或多个对象的状态变更。服务端每 0.25 秒推送一次状态更新（仅在数据变化时推送）。

**请求：**
```json
{
  "method": "objects/subscribe",
  "params": {
    "objects": {
      "toolhead": ["position", "print_time"],
      "extruder": ["temperature", "target"]
    },
    "response_template": {
      "id": null,
      "method": "printer:status"
    }
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `objects` | object | 是 | 对象名到字段列表的映射 |
| `response_template` | object | 否 | 推送消息模板，默认 `{}` |

> **注意**：`objects/subscribe` 除了周期推送外，还会**立即把当前状态作为本次请求的响应返回**（等价于一次 `objects/query` 全量查询，所有字段都会返回，不受「仅变化时推送」限制）。

**推送格式：**
```json
{
  "method": "printer:status",
  "params": {
    "eventtime": 12345.678,
    "status": {
      "toolhead": {
        "position": [21.0, 30.0, 5.0, 0.0],
        "print_time": 120.5
      }
    }
  }
}
```

> **注意**：取消订阅时，客户端应关闭连接。服务端会自动清理已关闭连接上的订阅。

---

## G-Code 相关端点

### 8. `gcode/help` — 获取可用 G-Code 命令列表

获取所有注册的 G-Code 命令及其帮助信息。返回值为扁平的 `{命令名: 帮助文本}` 字典，内容随已加载模块动态变化，下列仅为示例：

**请求：**
```json
{"method": "gcode/help"}
```

**响应：**
```json
{
  "G1": "G1 X... Y... Z... E... F... - Linear move",
  "G28": "G28 - Homing all axes",
  "M104": "M104 S... - Set extruder temperature (non-blocking)",
  "M109": "M109 S... - Set extruder temperature and wait",
  "M140": "M140 S... - Set bed temperature (non-blocking)",
  "M190": "M190 S... - Set bed temperature and wait",
  "M114": "M114 - Get current position",
  "M220": "M220 S... - Set feedrate percentage",
  "M221": "M221 S... - Set extrude percentage",
  "M204": "M204 S.../P... T... - Set acceleration",
  "SET_VELOCITY_LIMIT": "SET_VELOCITY_LIMIT VELOCITY=... ACCEL=... - Set velocity limits",
  "PAUSE": "Pauses the current print",
  "RESUME": "Resumes the print from a pause",
  ...
}
```

---

### 9. `gcode/script` — 执行 G-Code 脚本

同步解析并执行脚本；命令级错误会立即作为 `error` 返回，但**运动不会被等待完成**。

**请求：**
```json
{
  "method": "gcode/script",
  "params": {
    "script": "M104 S190\nM140 S60"
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `script` | string | 是 | G-Code 脚本，支持多行 |

**响应：** 无参数响应（`{}`）

> **注意**：脚本会被同步解析并执行，因此**命令级错误是立即返回的**（未知命令、参数错误等会作为 `error` 响应返回）。但**运动不会被等待完成**——移动入队后即返回。如需等待运动结束，通过 `objects/query` 轮询 `toolhead.print_time`、`print_stats.state` 等状态。

---

### 10. `gcode/restart` — 重启 Klipper 主机

重新加载配置文件并重启 Klipper 主机进程。

**请求：**
```json
{"method": "gcode/restart"}
```

**响应：** 无参数响应（`{}`）

---

### 11. `gcode/firmware_restart` — 重启固件和主机

重启固件和主机进程，重新加载配置文件。

**请求：**
```json
{"method": "gcode/firmware_restart"}
```

**响应：** 无参数响应（`{}`）

---

### 12. `gcode/subscribe_output` — 订阅 G-Code 输出

订阅 G-Code 命令的输出（包括 `ok` 响应、信息输出和错误信息）。

**请求：**
```json
{
  "method": "gcode/subscribe_output",
  "params": {
    "response_template": {
      "id": null,
      "method": "gcode:output"
    }
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `response_template` | object | 否 | 推送消息模板，默认 `{}` |

**推送格式：**
```json
{
  "method": "gcode:output",
  "params": {
    "response": "ok"
  }
}
```

或：
```json
{
  "method": "gcode:output",
  "params": {
    "response": "// Max Temperatures: extruder=280 bed=120"
  }
}
```

---

## Pause/Resume 相关端点

### 13. `pause_resume/pause` — 暂停打印

**请求：**
```json
{"method": "pause_resume/pause"}
```

**响应：** 无参数响应（`{}`）

---

### 14. `pause_resume/resume` — 恢复打印

**请求：**
```json
{"method": "pause_resume/resume"}
```

**响应：** 无参数响应（`{}`）

---

### 15. `pause_resume/cancel` — 取消打印

**请求：**
```json
{"method": "pause_resume/cancel"}
```

**响应：** 无参数响应（`{}`）

---

## 查询相关端点

### 16. `query_endstops/status` — 查询所有限位开关状态

查询所有已配置的限位开关（endstop）的当前状态。

**请求：**
```json
{"method": "query_endstops/status"}
```

**响应：** 直接返回所有限位开关的当前状态：

```json
{
  "endstop_x": "open",
  "endstop_y": "TRIGGERED",
  "endstop_z": "open"
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `<endstop_name>` | string | `"open"` 或 `"TRIGGERED"` |

> 查询结果同时缓存在 `query_endstops` 对象的 `last_query` 字段，可通过 `objects/query` 获取：
> ```json
> {"objects": {"query_endstops": ["last_query"]}}
> ```
> 返回格式：`{"last_query": {"endstop_x": 0, "endstop_y": 1}}`。注意 `last_query` 的值是限位开关的原始电平（`0` / `1` **整数**），与端点响应中的字符串不同。

---

### 17. `bed_mesh/dump_mesh` — 获取床面网格数据

获取当前激活的床面补偿网格数据。

**请求：**
```json
{"method": "bed_mesh/dump_mesh"}
```

**响应：** 返回当前网格、所有已保存 profile 以及可选的标定数据：

```json
{
  "current_mesh": {
    "name": "default",
    "probed_matrix": [[0.1, 0.0], [0.0, -0.1]],
    "mesh_matrix": [[0.1, 0.0], [0.0, -0.1]],
    "mesh_params": { ... }
  },
  "profiles": { "default": { ... } },
  "calibration": { ... }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `mesh_args` | object | 否 | 传入时额外返回 `calibration` 字段（手动探测参数） |

| 字段 | 类型 | 说明 |
|------|------|------|
| `current_mesh` | object | 当前激活网格；未探测时为 `{}` |
| `profiles` | object | 所有已保存的 profile |
| `calibration` | object | 仅在传入 `mesh_args` 时存在 |

> 运行期状态也可通过 `objects/query` 查询 `bed_mesh` 对象：
> ```json
> {"objects": {"bed_mesh": ["mesh_max", "mesh_min", "probed_matrix", "mesh_matrix", "profile_name", "profiles"]}}
> ```

---

## Bulk Sensor / 数据转储相关端点（多路复用）

以下端点使用 mux 机制：同一条路径可以为多个实例注册，调用时通过**普通请求参数**（非路径参数）指定实例名。例如：

- 路径 `adxl345/dump_adxl345` 的 key 为 `sensor`，因此需传 `"sensor": "adxl345"`
- 路径 `motion_report/dump_stepper` 的 key 为 `name`，因此需传 `"name": "stepper_x"`

若该 key 下注册了默认实例（其取值为 `null`），则该参数可省略；否则缺失或取值非法会报错。

### 18. 数据转储端点一览

| 端点路径 | mux key | 典型 value | 说明 |
|----------|---------|-----------|------|
| `adxl345/dump_adxl345` | `sensor` | 配置节名，如 `adxl345` | ADXL345 加速度数据 |
| `angle/dump_angle` | `sensor` | 配置节名，如 `my_angle` | 磁编码角度传感器数据 |
| `bmi160/dump_bmi160` | `sensor` | 配置节名 | BMI160 加速度数据 |
| `icm20948/dump_icm20948` | `sensor` | 配置节名 | ICM20948 加速度数据 |
| `ldc1612/dump_ldc1612` | `sensor` | 配置节名 | LDC1612 涡流传感器数据 |
| `lis2dw/dump_lis2dw` | `sensor` | 配置节名 | LIS2DW 加速度数据 |
| `mpu9250/dump_mpu9250` | `sensor` | 配置节名 | MPU9250 加速度数据 |
| `motion_report/dump_stepper` | `name` | 步进电机名 | 步进脉冲时间戳 |
| `motion_report/dump_trapq` | `name` | trapq 名 | 梯形运动队列 |
| `tmc/stallguard_dump` | `name` | 步进电机名 | TMC StallGuard 数据 |
| `load_cell/dump_force` | `load_cell` | load cell 名 | 称重传感器采样流 |
| `load_cell_probe/dump_taps` | `load_cell_probe` | 名称 | 探针敲击事件 |

**请求示例（订阅 ADXL345 数据）：**

```json
{
  "method": "adxl345/dump_adxl345",
  "params": {
    "sensor": "adxl345",
    "response_template": {
      "id": null,
      "method": "sensor:bulk_data"
    }
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `<key>` | string | 视注册情况 | 实例名（如 `sensor` / `name` / `load_cell`），见上表 |
| `response_template` | object | 否 | 推送消息模板，默认 `{}` |

**响应（同步）：** 本次请求会立即返回数据头（header），供客户端解析后续二进制/数组数据：

```json
{"header": ["time", "x_acceleration", "y_acceleration", "z_acceleration"]}
```

**推送：** 每个批次以 `params` 推送：

```json
{
  "method": "sensor:bulk_data",
  "params": {
    "data": [[...], [...]],
    "errors": 0,
    "overflows": 0
  }
}
```

> `params` 的具体键由各个传感器决定（常见为 `data` / `errors` / `overflows` / `interval` / `count`）。客户端应使用同步响应里的 `header` 来解析 `data`。

---

## 打印机对象状态说明

以下打印机对象提供状态信息，可通过 `objects/query` 或 `objects/subscribe` 获取：

### `toolhead`

| 字段 | 类型 | 说明 |
|------|------|------|
| `position` | Coord | 当前工具头位置 `[x, y, z, e]` |
| `print_time` | float | 当前打印时间（秒） |
| `stalls` | int | 打印停滞次数 |
| `estimated_print_time` | float | 估计打印时间 |
| `extruder` | string | 当前激活的挤出机名称 |
| `max_velocity` | float | 最大速度（mm/s） |
| `max_accel` | float | 最大加速度（mm/s²） |
| `minimum_cruise_ratio` | float | 最小巡航比例 |
| `square_corner_velocity` | float | 方角速度（mm/s） |
| `extra_axes_status` | object | 额外轴映射 `{轴名: 索引}` |
| `homed_axes` | string | 已归位轴组成的字符串，如 `"xyz"`（来自运动学） |
| `axis_minimum` | Coord | 运动学最小坐标（来自运动学） |
| `axis_maximum` | Coord | 运动学最大坐标（来自运动学） |
| `cone_start_z` | float | 仅 delta 运动学额外提供 |

### `webhooks`

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 打印机状态：`startup` / `ready` / `shutdown` / `error` |
| `state_message` | string | 状态描述 |

### `gcode`

| 字段 | 类型 | 说明 |
|------|------|------|
| `commands` | object | 可用 G-Code 命令列表，键为命令名，值为命令详情（含 `help`） |

### `gcode_move`

| 字段 | 类型 | 说明 |
|------|------|------|
| `absolute_coordinates` | bool | 绝对坐标模式 |
| `absolute_extrude` | bool | 绝对挤出模式 |
| `speed` | float | 当前速度（mm/s） |
| `speed_factor` | float | 速度系数（默认 1.0） |
| `extrude_factor` | float | 挤出系数（默认 1.0） |
| `position` | Coord | 上次设置的位置 `[x, y, z, e]` |
| `gcode_position` | Coord | 含着所有偏移后的当前 G-Code 坐标 |
| `homing_origin` | Coord | 归位原点偏移（`SET_GCODE_OFFSET` 的结果） |
| `axis_map` | array | 轴映射，元素为 `[轴字母, 轴索引]` |

### `configfile`

| 字段 | 类型 | 说明 |
|------|------|------|
| `config` | object | 原始配置内容（节名 → 选项映射） |
| `warnings` | array | 配置警告/弃用信息列表 |
| `save_config_pending` | bool | 是否有待保存的配置变更 |
| `save_config_pending_items` | object | 待保存的变更详情 |
| `settings` | object | 已使用的配置项设置 |

### `heater_bed`

| 字段 | 类型 | 说明 |
|------|------|------|
| `temperature` | float | 当前热床温度（°C） |
| `target` | float | 目标温度（°C） |
| `power` | float | 当前加热功率（0.0 ~ 1.0） |

### `extruder`

| 字段 | 类型 | 说明 |
|------|------|------|
| `temperature` | float | 当前挤出机温度（°C） |
| `target` | float | 目标温度（°C） |
| `power` | float | 当前加热功率（0.0 ~ 1.0） |

### `fan`

| 字段 | 类型 | 说明 |
|------|------|------|
| `speed` | float | 风扇转速（0.0 ~ 1.0） |
| `rpm` | float/null | 风扇转速（RPM），如无测速则为 null |

### `pause_resume`

| 字段 | 类型 | 说明 |
|------|------|------|
| `is_paused` | bool | 打印是否处于暂停状态 |

### `print_stats`

| 字段 | 类型 | 说明 |
|------|------|------|
| `filename` | string | 当前打印文件名 |
| `total_duration` | float | 总打印时长（秒） |
| `print_duration` | float | 实际打印时长（秒） |
| `filament_used` | float | 已使用耗材长度（mm） |
| `state` | string | 打印状态：`standby` / `printing` / `paused` / `complete` / `cancelled` / `error` |
| `message` | string | 状态消息 |
| `info` | object | 当前/总层数：`{total_layer, current_layer}`（可能为 null） |

### `bed_mesh`

| 字段 | 类型 | 说明 |
|------|------|------|
| `profile_name` | string | 当前网格配置文件名（未探测时为空串） |
| `mesh_min` | Coord | 网格最小坐标 `[x, y]` |
| `mesh_max` | Coord | 网格最大坐标 `[x, y]` |
| `probed_matrix` | array | 实测 Z 值矩阵 |
| `mesh_matrix` | array | 插值/补偿后 Z 值矩阵 |
| `profiles` | object | 所有已保存的网格 profile |

### `query_endstops`

| 字段 | 类型 | 说明 |
|------|------|------|
| `last_query` | object | 上次查询结果 `{endstop_name: 0 或 1}`（原始电平整数） |

### `virtual_sdcard`

| 字段 | 类型 | 说明 |
|------|------|------|
| `file_path` | string/null | 当前文件路径，未打印时为 null |
| `progress` | float | 打印进度（0.0 ~ 1.0） |
| `is_active` | bool | 是否正在从虚拟 SD 卡打印 |
| `file_position` | int | 当前文件读取位置（字节） |
| `file_size` | int | 文件总大小（字节） |

### `idle_timeout`

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 空闲超时状态：`Ready` / `Printing` / `Idle` |
| `printing_time` | float | 本次打印已持续时长（秒），非 `Printing` 状态为 0 |
| `idle_timeout` | float | 配置的空闲超时时间（秒） |

---

## G-Code 命令速查

以下是 klippy 注册的核心 G-Code 命令：

### 运动控制
| 命令 | 说明 |
|------|------|
| `G0` / `G1` | 直线移动（Klipper 中两者等价，均受速度限制） |
| `G2` / `G3` | 圆弧插补（需 `gcode_arcs` 模块） |
| `G28` | 归位所有轴（可指定 `X` / `Y` / `Z`） |
| `G90` | 绝对坐标模式 |
| `G91` | 相对坐标模式 |
| `G92` | 设置当前位置 |
| `G4` | 等待（P=毫秒） |
| `M400` | 等待所有移动完成 |
| `SET_GCODE_OFFSET` | 设置坐标偏移 |
| `SAVE_GCODE_STATE` / `RESTORE_GCODE_STATE` | 保存/恢复 G-Code 状态 |

### 温度控制
| 命令 | 说明 |
|------|------|
| `M104` | 设置挤出机温度（不等待） |
| `M109` | 设置挤出机温度并等待 |
| `M140` | 设置热床温度（不等待） |
| `M190` | 设置热床温度并等待 |
| `M105` | 获取温度 |
| `SET_HEATER_TEMPERATURE` | 设置任意加热器温度 |
| `PID_CALIBRATE` | PID 自整定 |

### 速度与加速度
| 命令 | 说明 |
|------|------|
| `M204` | 设置加速度（`S` / `P` / `T` 参数） |
| `SET_VELOCITY_LIMIT` | 设置速度限制（`VELOCITY=` / `ACCEL=` / `SQUARE_CORNER_VELOCITY=` / `MINIMUM_CRUISE_RATIO=`） |
| `M220` | 设置进给率百分比 |
| `M221` | 设置挤出百分比 |

### 风扇与灯光
| 命令 | 说明 |
|------|------|
| `M106` | 开启风扇（`P` 指定风扇，`S` 速度） |
| `M107` | 关闭风扇 |

### 系统控制
| 命令 | 说明 |
|------|------|
| `M112` | 紧急停止 |
| `M115` | 获取固件版本与能力 |
| `RESTART` | 重启 Klipper 主机 |
| `FIRMWARE_RESTART` | 重启固件和主机 |
| `SAVE_CONFIG` | 保存配置并重写配置文件 |
| `PAUSE` | 暂停打印 |
| `RESUME` | 恢复打印 |
| `CANCEL_PRINT` | 取消打印 |
| `CLEAR_PAUSE` | 清除暂停状态 |
| `STATUS` | 报告打印机状态 |
| `HELP` | 显示可用扩展命令 |
| `M114` | 获取当前位置 |
| `M119` | 查询限位开关状态 |
| `BED_MESH_CALIBRATE` | 执行床面探测并生成网格（`bed_mesh`） |
| `BED_MESH_CLEAR` | 清除床面网格 |
| `BED_MESH_PROFILE` | 加载/保存/删除床面网格 profile |
| `BED_MESH_OFFSET` | 调整床面网格偏移 |
| `BED_MESH_OUTPUT` | 输出床面网格数据 |
| `PROBE_CALIBRATE` | 探针校准（`probe`） |
| `QUAD_GANTRY_LEVEL` | 四梁水平校准（`quad_gantry_level`） |
| `Z_TILT_ADJUST` | Z 轴倾斜调整（`z_tilt`） |
| `BED_SCREWS_ADJUST` | 床面螺丝调整（`bed_screws`）——**段已落地、命令未注册**（2026-09-24 批 #1，调用会收到未知命令响应） |
| `SET_SERVO` | 舵机控制（mux 键 `SERVO=`，由 `[servo <名>]` 注册，2026-09-24 批 #2） |
| `SET_DUAL_CARRIAGE` | IDEX 滑架切换（由 `[dual_carriage]` 注册，2026-09-24；轨间坐标交接已实现于批 #4，步进仍仅主轨=C1 缺口入档） |
| `SAVE_DUAL_CARRIAGE_STATE` / `RESTORE_DUAL_CARRIAGE_STATE` | 滑架状态保存/恢复（同上；恢复移动语义未完全移植） |
| `EXCLUDE_OBJECT` / `EXCLUDE_OBJECT_START` / `EXCLUDE_OBJECT_END` / `EXCLUDE_OBJECT_DEFINE` | 打印对象排除四命令（由 `[exclude_object]` 注册，2026-09-24 批 #2；`M486` 宏体用例已随批 #4 引擎转绿，含排除区 E 补偿） |
| `PROBE_EDDY_CURRENT_TAP_CALIBRATE` | eddy tap 标定（由 `[probe_eddy_current]` 注册，批 #3；`TAP=` 子模式对上游；静态 `CALIBRATE=enable` 与 `Z_OFFSET_APPLY_PROBE` 未实现） |
| `DELTA_CALIBRATE` / `DELTA_ANALYZE` | delta 校准（由 `[delta_calibrate]` 注册，批 #5；结果以 SAVE_CONFIG 待写行交回写侧，回写命令未做） |
| `SCREWS_TILT_CALCULATE` | 螺丝倾斜计算（`screws_tilt_adjust`） |
| `SHAPER_CALIBRATE` | 输入整形校准（`resonance_tester`） |
| `QUERY_ENDSTOPS` | 查询限位开关 |
| `FORCE_MOVE` | 强制移动轴（`force_move`，调试用） |
| `MANUAL_PROBE` | 手动探测（`manual_probe`） |

> 上表只列出常见命令，实际可用命令以 `gcode/help` 的返回（或 `objects/query` 查询 `gcode.commands`）为准；部分命令需要先启用对应模块（如 `gcode_arcs`、`bed_mesh`、`probe`、`z_tilt`）。

---

## 错误处理

所有端点在出错时返回统一的错误格式：

```json
{
  "error": {
    "error": "WebRequestError",
    "message": "具体错误描述"
  }
}
```

常见错误场景：

| 错误 | 触发条件 |
|------|----------|
| `webhooks: No registered callback for path '<path>'` | 请求的端点路径不存在 |
| `Missing Argument [<name>]` | 缺少必需的请求参数 |
| `Invalid Argument Type [<name>]` | 参数类型与预期不符 |
| `Multiple calls to send not allowed` | 同一个请求被重复应答 |
| `Internal Error on WebRequest: <method>` | 端点处理逻辑抛出预期外的异常；会记录日志并**触发 klippy shutdown** |
| gcode 层异常文本（如 `Unknown command:"XXX"`、`Must home axis first`） | 命令执行失败或打印机未就绪 |

> 打印机处于 shutdown/error 时的文本（如 `Printer is halted`）出现在 `info` 响应的 `state_message` 字段中，**不是**错误响应。

> 若推送/响应的 JSON 无法序列化，会记录 `json encoding error` 并触发 klippy shutdown。

---

## 通信流程示例

### 客户端连接与初始化

```
1. 客户端连接 Unix Domain Socket
2. 发送 {"id": "1", "method": "info", "params": {"client_info": {"name": "Moonraker"}}}
3. 服务端返回状态信息
4. 客户端订阅状态：{"id": "2", "method": "objects/subscribe", "params": {...}}
5. 客户端订阅 G-Code 输出：{"id": "3", "method": "gcode/subscribe_output", "params": {...}}
```

### 执行 G-Code 并获取结果

```
1. 发送 {"id": "4", "method": "gcode/script", "params": {"script": "M105"}}
2. 通过 gcode/subscribe_output 接收原始输出（M105 返回形如 "T:200.5 /200.0"，
   经 gcode/script 执行时不会带 "ok " 前缀）
3. 或通过 objects/query 查询 extruder 温度
```

### 查询打印机状态

```
1. 发送 {"id": "5", "method": "objects/query", "params": {"objects": {"toolhead": ["position"], "webhooks": null}}}
2. 服务端返回 {"id": "5", "result": {"status": {"toolhead": {...}, "webhooks": {...}}}}
```

---

- [← 第三方开发手册首页](README.md)
