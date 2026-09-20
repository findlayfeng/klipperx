# G-Code 命令参考

本章节描述 KlipperX 已实现的 G-Code 命令格式与用法。

## 命令格式总览

KlipperX 支持两类 G-Code 命令：**传统命令**（Traditional）和**扩展命令**（Extended）。

| 类型 | 格式 | 参数格式 | 示例 |
|------|------|----------|------|
| 传统命令 | 大写字母 + 数字 | 字母 + 值（空格分隔） | `G1 X10.5 Y20 F1000` |
| 扩展命令 | 全大写字母/数字/下划线 | `KEY=VALUE`（等号分隔） | `SET_PIN PIN=fan VALUE=1` |

### 公共语法

无论哪种类型，命令行都支持以下语法：

| 语法 | 说明 | 示例 |
|------|------|------|
| `N<digits>` | 行号前缀（会被解析器跳过） | `N5 G1 X10` |
| `;` 或 `#` | 注释起始符，其后内容被忽略 | `G1 ; 移动到 X10` |
| `'...'` | 单引号包裹的值（保留空格和特殊字符） | `SET_PIN MESSAGE='hello world'` |
| `"..."` | 双引号包裹的值（保留空格和特殊字符） | `SET_PIN MESSAGE="hello world"` |
| 大小写不敏感 | 命令名和参数名均不区分大小写 | `g1` 与 `G1` 等价 |

---

## 传统 G-Code 命令

传统命令的格式为**一个大写字母后接一个数字**，参数以"字母 + 值"的空格分隔形式给出。

### M110 — 设置当前行号

```
M110 [S<line_number>]
```

设置当前行号。KlipperX 中该命令被接受但忽略，不产生任何效果。

**参数：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `S` | 整数 | 行号（可选，被忽略） |

**示例：**
```
M110 S100
```

---

### M112 — 紧急停止

```
M112
```

立即触发打印机关机。所有运动停止，加热器关闭。

**参数：** 无。

**示例：**
```
M112
```

---

### M115 — 获取固件版本

```
M115
```

返回固件名称和版本号。

**参数：** 无。

**输出示例：**
```
// FIRMWARE_NAME:Klipper FIRMWARE_VERSION:0.1.0
```

---

## 扩展命令

扩展命令的格式为**全大写字母、数字和下划线的组合**，不以字母开头，不以"字母+数字"开头（否则会被识别为传统命令）。参数以 `KEY=VALUE` 形式给出。

### ECHO — 回显命令

```
ECHO [text]
```

将命令行的其余部分原样输出。

**参数：** 无参数，命令后跟随的文本会被回显。

**示例：**
```
ECHO Hello, KlipperX!
```

**输出：**
```
ECHO Hello, KlipperX!
```

---

### HELP — 列出可用命令

```
HELP
```

列出所有已注册的扩展命令及其描述。

**参数：** 无。

**输出示例：**
```
// Available extended commands:
// ECHO             : Echo the command line
// FIRMWARE_RESTART : Restart firmware, host, and reload config
// HELP             : Report the list of available extended G-Code commands
// RESTART          : Reload config file and restart host software
// SET_PIN          : Set the value of a pin
// STATUS           : Report the printer status
```

---

### RESTART — 重启主机

```
RESTART
```

重新加载配置文件并重启主机软件。

**参数：** 无。

**示例：**
```
RESTART
```

---

### FIRMWARE_RESTART — 重启固件

```
FIRMWARE_RESTART
```

重启固件、主机并重新加载配置。

**参数：** 无。

**示例：**
```
FIRMWARE_RESTART
```

---

### STATUS — 打印状态

```
STATUS
```

报告打印机的当前状态（Ready / Shutdown / Disconnect 等）。

**参数：** 无。

**输出示例：**
```
// Klipper state: Ready
```

---

### SET_PIN — 设置引脚状态

```
SET_PIN PIN=<name> VALUE=<0..1>
```

设置已配置的 `output_pin` 引脚的值。该命令是一个**多路命令**（mux command），每个 `[output_pin <name>]` 配置节会自动注册一个值。

**参数：**

| 参数 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `PIN` | 字符串 | 是 | 引脚名称，对应 `[output_pin <name>]` 中的 `<name>` |
| `VALUE` | 0.0 ~ 1.0 | 是 | 输出值。数字输出：`>= 0.5` 为高电平；PWM（`pwm: true`）：作为占空比 |

**前提条件：** 配置文件中需定义至少一个 `[output_pin <name>]` 节，例如：

```ini
[output_pin my_fan]
pin: PA0
value: 0
shutdown_value: 0

[output_pin pwm_fan]
pin: PA1
pwm: true
cycle_time: 0.02
```

**示例：**
```
SET_PIN PIN=my_fan VALUE=1      ; 数字输出：打开
SET_PIN PIN=my_fan VALUE=0      ; 数字输出：关闭
SET_PIN PIN=pwm_fan VALUE=0.25  ; PWM：25% 占空比
```

**错误处理：** 如果指定的 PIN 值未注册，会返回可用选项列表：

```
The value 'unknown' is not valid for PIN. Options: 'my_fan', 'my_light'
```

---

## 命令注册说明

KlipperX 的命令通过注册机制动态加载：

- **内置命令**（M110、M112、M115、RESTART、FIRMWARE_RESTART、ECHO、STATUS、HELP）在启动时自动注册，其中 M110、M112、M115、RESTART、FIRMWARE_RESTART、ECHO、STATUS、HELP 在打印机就绪前即可使用。
- **扩展命令**（如 SET_PIN）由对应的功能模块在配置加载时注册。只有当配置文件中存在相应的配置节（如 `[output_pin ...]`）时，对应的命令值才会可用。
- **运动相关命令**（G0、G1、G28 等）由工具头模块注册，不在本 dispatcher 中实现。

---

## 命令解析细节

### 传统命令解析

传统命令的参数按"字母 + 值"配对解析，空格分隔：

| 输入 | 命令 | 参数 |
|------|------|------|
| `G1 X10.5 Y20 F1000` | `G1` | `X=10.5`, `Y=20`, `F=1000` |
| `M115` | `M115` | 无 |
| `G4 P2000` | `G4` | `P=2000` |

### 扩展命令解析

扩展命令的参数按 `KEY=VALUE` 解析，支持 shell 风格的引号和注释：

| 输入 | 命令 | 参数 |
|------|------|------|
| `SET_PIN PIN=fan VALUE=1` | `SET_PIN` | `PIN=fan`, `VALUE=1` |
| `SET_PIN PIN='my pin' VALUE=0` | `SET_PIN` | `PIN=my pin`, `VALUE=0` |
| `SET_PIN PIN=fan VALUE=1 ; turn on` | `SET_PIN` | `PIN=fan`, `VALUE=1` |
| `SET_PIN PIN=fan VALUE=1 # 注释` | `SET_PIN` | `PIN=fan`, `VALUE=1` |

### 行号处理

行号 `N<digits>` 作为命令的前缀被解析器跳过，不影响命令执行：

| 输入 | 实际命令 |
|------|----------|
| `N5 G1 X10` | `G1 X10` |
| `N100 M110 S50` | `M110 S50` |

---

## 命令错误

命令执行失败时，KlipperX 会以 `!! ` 前缀返回错误信息：

```
!! Error on 'SET_PIN PIN=unknown VALUE=1': The value 'unknown' is not valid for PIN. Options: 'my_fan'
!! Error on 'G1 Xabc': unable to parse abc
!! Error on 'MY_CMD': missing VALUE
```

---

- [← 用户手册首页](README.md)
