# Klippy Web API 参考文档

本文档描述 Klipper 主机端（klippy）通过 Unix Domain Socket 暴露的 Web API 接口。外部客户端（如 Fluidd、Mainsail、Moonraker）通过这些接口与打印机通信。

## 通信协议概述

### 连接方式

- **传输层**：Unix Domain Socket
- **消息格式**：JSON
- **消息分隔符**：`\x03` (ASCII ETX)
- **编码**：`msgspec.json.encode`（优先）/ `json.dumps`（备选），字段间无空格

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

无 id 的请求：
```json
{"result": { ... }}
```

### 服务端推送（Subscription）

客户端可以通过 `response_template` 参数订阅服务端推送。服务端推送格式：
```json
{"id": null, "method": "<method>", "params": { ... }}
```

---

## 端点列表

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
  "software_version": "v0.12.0-xxx",
  "cpu_info": {"model": "Raspberry Pi 4", "cores": 4}
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 打印机状态：`startup` / `ready` / `shutdown` / `error` |
| `state_message` | string | 状态描述信息 |
| `hostname` | string | 主机名 |
| `klipper_path` | string | Klipper 安装路径 |
| `python_path` | string | Python 解释器路径 |
| `process_id` | int | 进程 ID |
| `user_id` / `group_id` | int | 运行用户/组 ID |
| `log_file` | string | 日志文件路径 |
| `config_file` | string | 配置文件路径 |
| `software_version` | string | Klipper 软件版本 |
| `cpu_info` | object | CPU 信息 |

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

---

## Objects 相关端点

### 5. `objects/list` — 列出所有可查询对象

获取所有注册了 `get_status()` 方法的打印机对象列表。

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
| `response_template` | object | 是 | 推送消息模板 |

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

获取所有注册的 G-Code 命令及其帮助信息。

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

异步执行 G-Code 脚本（不等待响应）。

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

> **注意**：此方法不等待命令执行完成，也不返回执行结果。如需同步执行，通过 `objects/query` 轮询相关状态。

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
| `response_template` | object | 是 | 推送消息模板 |

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

**响应：** 无参数响应（`{}`）

> 查询结果可通过 `objects/query` 查询 `query_endstops` 对象的 `last_query` 字段获取：
> ```json
> {"objects": {"query_endstops": ["last_query"]}}
> ```
> 返回格式：`{"last_query": {"endstop_x": true, "endstop_y": false, ...}}`

---

### 17. `bed_mesh/dump_mesh` — 获取床面网格数据

获取当前激活的床面补偿网格数据。

**请求：**
```json
{"method": "bed_mesh/dump_mesh"}
```

**响应：** 无参数响应（`{}`）

> 网格数据可通过 `objects/query` 查询 `bed_mesh` 对象获取：
> ```json
> {"objects": {"bed_mesh": ["mesh_max", "mesh_min", "probed_matrix", "mesh", "profile_name"]}}
> ```

---

## Bulk Sensor 相关端点（多路复用）

以下端点使用 mux 机制，通过路径中的 `key` 参数区分不同实例。

### 18. `sensor_bulk_data/{sensor_type}` — 批量传感器数据

用于批量传感器（bulk sensor）数据订阅，如振动传感器等。

**请求：**
```json
{
  "method": "sensor_bulk_data/adxl345",
  "params": {
    "response_template": {
      "id": null,
      "method": "sensor:bulk_data"
    }
  }
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `sensor_type` | string | 是 | 传感器名称（路径参数） |
| `response_template` | object | 是 | 推送消息模板 |

---

## Load Cell 相关端点（多路复用）

### 19. `load_cell/{load_cell_name}/...` — 称重传感器操作

用于 load cell 称重传感器的数据订阅。

**请求：**
```json
{
  "method": "load_cell/my_loadcell",
  "params": {
    "response_template": {
      "id": null,
      "method": "load_cell:data"
    }
  }
}
```

---

## 打印机对象状态说明

以下对象注册了 `get_status(eventtime)` 方法，可通过 `objects/query` 或 `objects/subscribe` 获取状态：

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
| `extra_axes` | object | 额外轴映射 `{轴名: 索引}` |

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
| `last_position` | Coord | 上次位置 `[x, y, z, e]` |

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
| `rpm` | int/null | 风扇转速（RPM），如无测速则为 null |

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
| `state` | string | 打印状态：`offline` / `printing` / `paused` / `error` / `cancelled` / `completed` |
| `message` | string | 状态消息 |

### `bed_mesh`

| 字段 | 类型 | 说明 |
|------|------|------|
| `profile_name` | string | 当前网格配置文件名 |
| `mesh_max` | Coord | 网格最大坐标 `[x, y]` |
| `mesh_min` | Coord | 网格最小坐标 `[x, y]` |
| `probed_matrix` | array | 实测 Z 值矩阵 |
| `mesh` | array | 插值后 Z 值矩阵 |
| `z_mesh` | bool/null | 是否启用网格补偿 |
| `fade_start` | float | Fade 起始 Z 值 |
| `fade_end` | float | Fade 结束 Z 值 |
| `fade_target` | float/null | Fade 目标 Z 值 |

### `query_endstops`

| 字段 | 类型 | 说明 |
|------|------|------|
| `last_query` | object | 上次查询结果 `{endstop_name: value}` |

### `virtual_sdcard`

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | SD 卡状态：`off` / `printing` / `paused` |
| `file` | string/null | 当前打印文件路径 |

### `idle_timeout`

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 空闲超时状态：`Ready` / `Idle` / `Timeout` |

---

## G-Code 命令速查

以下是 klippy 注册的核心 G-Code 命令：

### 运动控制
| 命令 | 说明 |
|------|------|
| `G0` / `G1` | 直线移动（G0 快速，G1 受速度限制） |
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
| `BED_MESH_CLEAR` | 清除床面网格 |
| `BED_MESH_PROFILE` | 加载床面网格配置 |
| `BED_MESH_ADJUST` | 调整床面网格偏移 |
| `BED_MESH_OUTPUT` | 输出床面网格数据 |
| `PROBE_CALIBRATE` | 探针校准 |
| `QUAD_GANTRY_LEVEL` | 四梁水平校准 |
| `LEVEL_LINERS` / `LEVEL_CORNERS` | 直线/角点调平 |
| `SCREWS_TILT_ADJUST` | 螺丝倾斜调整 |
| `INPUT_SHAPER` | 输入整形校准 |
| `QUERY_ENDSTOPS` | 查询限位开关 |
| `FORCE_MOVE` | 强制移动轴（调试用） |
| `MANUAL_PROBE` | 手动探测 |

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
| `No registered callback for path` | 请求的端点路径不存在 |
| `Missing Argument [...]` | 缺少必需的请求参数 |
| `Invalid Argument Type [...]` | 参数类型不匹配 |
| `Printer is halted` | 打印机处于 shutdown/error 状态 |

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
2. 通过 gcode/subscribe_output 接收 "ok T:200.5"
3. 或通过 objects/query 查询 extruder 温度
```

### 查询打印机状态

```
1. 发送 {"id": "5", "method": "objects/query", "params": {"objects": {"toolhead": ["position"], "webhooks": null}}}
2. 服务端返回 {"id": "5", "result": {"status": {"toolhead": {...}, "webhooks": {...}}}}
```

---

- [← 第三方开发手册首页](README.md)
