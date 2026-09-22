# 上游功能覆盖审计（2026-09-21）

本文是一次**盘点记录**，不是规范：它把上游 Klipper 的全部功能点逐项对照本仓库当前实现，
给出「已实现 / 部分实现 / 未实现 / 不适用」的判据与结果。落地用的待办清单在
[`TODO.md`](../../TODO.md) 的「上游 extras 覆盖盘点」一节；本文只负责**证据与口径**，
保持日期与上游 commit 不变，不随实现推进而更新（要更新就再写一篇）。

## 1. 标的与基线

| 项 | 值 |
|---|---|
| 上游路径 | `third_party/klipper/`（submodule，分支 `klipperx`） |
| 上游 commit | `02e71b9fe87787f233d9a3c332cf0a75ae1161dd`（2026-09-18 00:38:34 +0800） |
| 本仓库基线 | `744369b`（2026-09-21，I2C 总线错误停机 F7） |
| 审计日期 | 2026-09-21 |

上游的「功能点」由三层构成，本审计三层都覆盖：

| 层 | 上游位置 | 数量 | 与本仓库的对应 |
|---|---|---|---|
| 主程序核心 | `klippy/*.py` | 17 | `src/core/klippy/`、`src/klippy.rs` |
| 运动学 | `klippy/kinematics/*.py` | 16（含 `__init__`） | 无（`Kinematics` trait 已删，见 TODO C1） |
| extras（配置节） | `klippy/extras/*.py` | 134（含 `__init__`） | `src/core/klippy/extras/`（6 个文件） |
| 显示子包 | `klippy/extras/display/*.py` | 10（含 `__init__`） | 无 |
| 固件命令模块 | `src/*.c` | 36 | `src/core/klippy/cmd/` + `mcu/resource/` |
| C 加速器 | `klippy/chelper/*.c` | 21 | 运动/步进生成与传输辅助，见 §4.1 |

## 2. 方法（可复现）

### 2.1 枚举上游功能点

上游 `Printer._read_config` / `load_object` 用 **section 名 import
`extras/<name>.py`**，再调 `load_config`（匿名节）或 `load_config_prefix`（命名节）
（`klippy/klippy.py:90-113`）。所以「模块文件 = 配置节 = 功能点」基本成立；没有
`load_config*` 的模块（`bus`/`tmc`/`bulk_sensor`/`shaper_defs`/…）是被别的 extras 依赖的
**基础设施**，也计入盘点，单独归类。

```sh
# 每个 extras 模块有没有匿名/命名节入口
cd third_party/klipper/klippy/extras
for f in *.py; do
  printf "%-28s bare=%s prefix=%s\n" "${f%.py}" \
    "$(grep -c '^def load_config(' "$f")" "$(grep -c '^def load_config_prefix(' "$f")"
done
# 模块用途（首行注释）
for f in *.py display/*.py; do printf "%-40s %s\n" "$f" "$(head -1 "$f" | sed 's/^# *//')"; done
# 固件命令模块
ls ../../src/*.c
```

### 2.2 对照本仓库

| 手段 | 位置 |
|---|---|
| 已注册的配置节 | `src/**/section!` 声明（`build.rs` 收集成 `section_factories.rs`） |
| 已实现的 MCU 命令 | `src/core/klippy/cmd/*.rs`（命令词汇 + 类型化调用）、`mcu/resource/*.rs`（资源对象） |
| 已注册的 API 端点 | `src/core/klippy/api/endpoints/*.rs` 的 `endpoint!` 声明与模块头状态表 |
| 已注册的 G-Code 命令 | `src/core/klippy/gcode.rs` 的 `register_builtins` 与各 extras 的 `register_*` |
| 已推送的打印机事件 | `src/core/klippy/event/decl/*.rs`（`build.rs` 生成 `KlippyEvent`） |

### 2.3 状态口径

| 记号 | 含义 |
|---|---|
| ✅ | 已实现，且与上游行为基本对齐 |
| ◐ | 部分实现：主体可用，缺口逐条列出（缺口本身进 TODO） |
| ⬜ | 未实现 |
| N/A | 对 Rust 主机无意义（Python 运行时专属，或特定硬件开发脚本） |

## 3. 结论摘要

- **配置节**：本仓库当前只认 5 个节 —— `[mcu]`、`[output_pin]`、`[board_pins]`、
  `[i2c_device]`、`[spi_device]`。其中 `[i2c_device]` / `[spi_device]` 是**上游没有的
  自建调试节**（上游各传感器各自 `MCU_*_from_config` 读同样选项）；`[printer]` 还没有住户。
- **extras**：133 个上游模块（不含 `__init__.py`）里，`board_pins` ✅、`output_pin` ◐、
  `bus` ✅（SPI/I2C 框架），其余 ⬜。
- **kinematics**：15 个模块全部 ⬜（`Kinematics` trait 已按「实现 toolhead 时再加回」删除）。
- **核心**：传输/协议/配置/G-Code/API/事件/指纹识别已成形；缺 `toolhead` / `stepper`
  对象 / `clocksync`（print_time↔clock 偏移）/ `mathutil` / `util` 的一部分，以及
  `chelper/` 的步进生成与运动数学（§4.1）。
- **MCU 命令**：数字输出、PWM、ADC、SPI、I2C 五类资源已落地；步进生成、endstop/trsync、
  buttons、pulse_counter、sdcard、sensor_bulk、各类传感器、lcd、neopixel、thermocouple、
  tmcuart、trigger_analog、initial_pins 仍缺。
- **API 端点**：`info` / `list_endpoints` / `objects/*` / `gcode/*`（5 个）已完成；
  `emergency_stop`、`register_remote_method`、`pause_resume/*`、`query_endstops/status`、
  `bed_mesh/dump_mesh`、各 `*/dump_*` mux 未做（B4）。
- **Python 专属、判为 N/A**：`garbage_collection`（GC 调优）、`aio_executor`（线程池）；
  `parsedump`（开发工具，可选）。

## 4. 核心主程序（klippy/*.py）

### 4.1 `chelper/`：C 加速器

上游把延迟敏感的内层循环用 C 写、经 `pyhelper` 暴露给 Python。Rust 主机没有这层 FFI
边界，等价能力要么已在 `msg`/`mcu` 里，要么随 **C1** 用 Rust 重写：

| 上游文件 | 用途 | 本仓库对应 |
|---|---|---|
| `stepcompress.c` | 步进压缩/队列下发（`queue_step` 编码） | ⬜ C1 |
| `itersolve.c` + `kin_*.c`（cartesian/corexy/corexz/delta/deltesian/extruder/generic/idex/polar/rotary_delta/shaper/winch） | 逆解与步进分配 | ⬜ C1 |
| `trapq.c` | 梯形运动队列 | ⬜ C1 |
| `kin_shaper.c` | 输入整形（与 `input_shaper.py` 成对） | ⬜ C1/H6 |
| `trdispatch.c` | trsync 触发分发（主机侧） | ⬜ F8 |
| `serialqueue.c` / `msgblock.c` / `steppersync.c` / `pollreactor.c` | 发送队列、旧版 msgblock 协议、步进同步、轮询 reactor | ◐ `mcu/` 的发送任务与 `msg/`；`msgblock` 属于旧固件兼容，未做 |
| `pyhelper.c` / `compiler.h` / `list.h` | FFI 与容器样板 | N/A（Rust） |

| 模块 | 用途 | 状态 | 本仓库对应 / 缺口 |
|---|---|---|---|
| `klippy.py` | Printer 骨架、配置装载、主循环、重启 | ◐ | `src/klippy.rs`、`printer.rs`、`load.rs`；缺 start args/rollover/日志（D1）、退出语义（Q6） |
| `gcode.py` | G-Code 调度器 | ◐ | `gcode.rs`；`GCodeIO`（伪 tty/文件/ack/`stats gcodein`）与一批行为差异（G1b） |
| `configfile.py` | 配置解析与校验 | ◐ | `config/`；缺 option 级访问追踪校验（C2） |
| `mcu.py` | MCU 传输对象、`MCU_*` 资源、`TriggerDispatch` | ◐ | `mcu/` + `cmd/`；缺 `last_stats`、`emergency_stop` 对象、endstop/trsync（B2/F8） |
| `msgproto.py` | 格式串 ↔ 字节 | ✅ | `msg/` |
| `pins.py` | 引脚描述解析与保留 | ◐ | `pins.rs`；`get_pin_type`/rename 等人为用法待补（F9 消费者时） |
| `reactor.py` | 定时器/回调/延迟度量 | ✅ | `reactor.rs`（A1/A1b） |
| `serialhdl.py` | 串口帧收发、序号、RTO | ◐ | `interface/`；RTO 定时重传未做（D3） |
| `clocksync.py` | `print_time` ↔ MCU clock 偏移估计 | ⬜ | `cmd/clock.rs` 只有 `get_clock`；偏移跟踪属于运动层（C1/F3） |
| `stepper.py` | `Stepper` 运动对象、`setup_itersolve` | ⬜ | `cmd/stepper.rs` 只有命令定义；对象与 step 生成未做（C1） |
| `toolhead.py` | 运动队列、trapq、lookahead、kinematics 装载 | ⬜ | C1 |
| `webhooks.py` | 客户端 API（UDS + 分帧 + 端点 + 推送） | ◐ | `api/`；其余端点（B4） |
| `util.py` | `Coord`、`get_heater`/`get_sensor`、`load_object`、错误类 | ◐ | 零散 helper；`Coord`/`get_*`/反射式 `lookup_object` 未做（C1/F9/Q5） |
| `mathutil.py` | 几何/线性代数（kinematics、probe、mesh 用） | ⬜ | C1 |
| `console.py` | 虚拟控制台（stdout 重定向、debug 输入） | ◐ | `src/logging.rs` 覆盖日志侧；伪 tty 输入随 G1b |
| `queuelogger.py` | 后台日志队列 | ◐ | `src/logging.rs` |
| `parsedump.py` | 解析 serial dump 的开发脚本 | N/A | 可选开发工具，不阻塞 |

## 5. 运动学（kinematics/）

全部 ⬜，统一随 **C1**（toolhead 持有并装载）。`kinematic_stepper.py` 是 step 生成层的
接缝，`idex_modes.py` 被 cartesian/hybrid 复用，`extruder.py` 是 `[extruder]` 的运动学部分。

| 模块 | 用途 |
|---|---|
| `cartesian.py` | 直角坐标（XYZ 各一 stepper） |
| `corexy.py` / `corexz.py` | CoreXY / CoreXZ |
| `hybrid_corexy.py` / `hybrid_corexz.py` | 混合型（一路直驱） |
| `generic_cartesian.py` | 通用仿射组合 |
| `delta.py` / `deltesian.py` | Delta / Deltesian |
| `polar.py` / `rotary_delta.py` | 极坐标 / 旋转 Delta |
| `winch.py` | 绞盘 |
| `none.py` | 无运动学（仅外设） |
| `idex_modes.py` | IDEX 双滑车共享 |
| `extruder.py` | 挤出机运动学 |
| `kinematic_stepper.py` | `setup_itersolve` 的 stepper 分配 |
| `__init__.py` | 包定义 |

## 6. 运动 / 回零 / 探测 / 调平（依赖 C1、F8）

| 模块 | 用途 | 依赖 |
|---|---|---|
| `gcode_move.py` | `[gcode_move]`：G0/G1/G28/G92/M114… 与坐标偏移 | C1、G4 |
| `homing.py` | 通用回零驱动 | C1、F8 |
| `homing_override.py` | 用宏替换 G28 | C1、H3 |
| `homing_heaters.py` | 回零时开/关加热器 | C1 |
| `safe_z_home.py` | 指定 XY 位置再 Z 回零 | C1、F8 |
| `manual_probe.py` | 手动 Z 高度探测 | C1、F8 |
| `probe.py` | Z 探针 | C1、F8 |
| `bltouch.py` | BLTouch | C1、F8 |
| `smart_effector.py` | SmartEffector 探针 | C1、F8 |
| `endstop_phase.py` | 用步进相位提高 endstop 精度 | C1、F8 |
| `force_move.py` | `FORCE_MOVE` 等诊断移动 | C1 |
| `manual_stepper.py` | `[manual_stepper]` | C1 |
| `stepper_enable.py` | enable 引脚管理、`M18/M84` | ✅ 已实现（T2） |
| `extruder_stepper.py` | 多 stepper 共用挤出机 | C1 |
| `input_shaper.py` | 输入整形 | C1、H6 |
| `motion_queuing.py` | 低级运动排队/刷新辅助 | C1 |
| `motion_report.py` | `dump_trapq` / `dump_stepper` 端点 | C1、B4 |
| `skew_correction.py` | 歪斜校正 | C1 |
| `gcode_arcs.py` | G2/G3 圆弧 | C1、G4 |
| `axis_twist_compensation.py` | 轴扭转补偿 | C1、F8 |
| `z_thermal_adjust.py` | Z 热漂移补偿 | C1、H1 |
| `bed_mesh.py` | 网床调平（含 `dump_mesh` 端点） | C1、F8、B4 |
| `bed_tilt.py` | 床倾斜补偿（已废弃，仍保留） | C1、F8 |
| `quad_gantry_level.py` | 四 Z 龙门调平 | C1、F8 |
| `z_tilt.py` | 多 Z 倾斜调平 | C1、F8 |
| `bed_screws.py` | `BED_SCREWS_ADJUST` | C1、F8 |
| `screws_tilt_adjust.py` | 螺丝倾斜调整 | C1、F8 |
| `delta_calibrate.py` | Delta 校准 | C1、F8 |
| `tuning_tower.py` | 按 Z 调参 | C1、H3 |
| `firmware_retraction.py` | G10/G11 固件回抽 | C1 |
| `exclude_object.py` | 排除对象 | H4 |
| `idle_timeout.py` | 空闲超时（事件 `idle_timeout:*`） | C1 |
| `load_cell.py` | 称重传感器 + `load_cell/dump_force` | H6、B4 |
| `load_cell_probe.py` | 称重探针 + `dump_taps` | H6、B4 |
| `probe_eddy_current.py` | 涡流 Z 探针 | H6、F8 |
| `resonance_tester.py` / `shaper_calibrate.py` / `shaper_defs.py` | 共振测试与整形自动标定 | H6、C1 |

## 7. 加热与温度（H1）

| 模块 | 用途 |
|---|---|
| `heaters.py` | 加热器框架（`get_heater`、PWM 控制、`verify_heater` 调度） |
| `heater_bed.py` | `[heater_bed]` |
| `heater_generic.py` | `[heater_generic]` |
| `pid_calibrate.py` | `PID_CALIBRATE` |
| `verify_heater.py` | 加热器校验 |
| `temperature_sensor.py` | 通用温度传感器 |
| `thermistor.py` | 热敏电阻（ADC） |
| `adc_temperature.py` | ADC 线性插值温度 |
| `adc_scaled.py` | 用 VREF/VSSA 缩放 ADC |
| `spi_temperature.py` | SPI 热电偶/RTD（MAX31855/MAX31856/MAX31865） |
| `temperature_combined.py` | 多传感器合成 |
| `temperature_host.py` | 主机温度 |
| `temperature_mcu.py` | MCU 片内温度 |
| `temperature_probe.py` | 探针温度漂移补偿 |
| `temperature_fan.py` | 温度风扇（也算输出） |

## 8. 风扇与通用输出（H2）

| 模块 | 用途 | 状态 |
|---|---|---|
| `output_pin.py` | `[output_pin]` | ◐ 数字/PWM 已做；`scale`/`TEMPLATE`/GCodeRequestQueue 缺（G2b） |
| `fan.py` | `[fan]` 打印风扇（`M106/M107`） | ⬜ |
| `fan_generic.py` | `[fan_generic]` | ⬜ |
| `heater_fan.py` | 随加热器开的风扇 | ⬜ |
| `controller_fan.py` | MCU/步进散热风扇 | ⬜ |
| `pwm_tool.py` | 队列化 PWM GPIO 输出 | ⬜ |
| `pwm_cycle_time.py` | 可变频率 PWM | ⬜ |
| `static_digital_output.py` | 固定数字输出 | ⬜ |
| `static_pwm_clock.py` | 固定 PWM 时钟输出 | ⬜ |
| `multi_pin.py` | 一改多引脚 | ⬜ |
| `servo.py` | 舵机（`SET_SERVO`） | ⬜ |
| `duplicate_pin_override.py` | 允许重复引脚 | ⬜ |
| `led.py` | PWM LED | ⬜ |
| `neopixel.py` / `dotstar.py` | 可寻址 LED | ⬜ |
| `pca9533.py` / `pca9632.py` | I2C LED 驱动 | ⬜ |

## 9. 输入与外设（H7）

| 模块 | 用途 | 固件侧 |
|---|---|---|
| `buttons.py` | 按钮检测与回调 | `src/buttons.c` |
| `gcode_button.py` | 按钮触发 G-Code | `src/buttons.c` |
| `pulse_counter.py` | GPIO 边沿计数 | `src/pulse_counter.c` |
| `trigger_analog.py` | 模拟触发（load cell 等） | `src/trigger_analog.c` |
| `filament_switch_sensor.py` | 断料开关 | — |
| `filament_motion_sensor.py` | 运动式断料检测 | `src/buttons.c`/ADC |
| `hall_filament_width_sensor.py` | 霍尔线宽 | ADC |
| `tsl1401cl_filament_width_sensor.py` | TSL1401 线宽 | ADC |
| `ad5206.py` / `mcp4018.py` / `mcp4451.py` / `mcp4728.py` / `dac084S085.py` | 数字电位器 / DAC | SPI/I2C |
| `sx1509.py` | GPIO 扩展 | I2C |
| `samd_sercom.py` | SAMD SERCOM 复用 | 特定固件 |
| `replicape.py` | Replicape 杂项芯片 | 特定板 |
| `palette2.py` | Palette2 MMU | 串口协议 |
| （固件）`initial_pins.c` | 上电初始引脚状态 | — |

## 10. 步进驱动 TMC（H5）

| 模块 | 用途 |
|---|---|
| `tmc.py` | TMC 公共框架（寄存器、StallGuard、`DUMP_TMC`/`SET_TMC_*`） |
| `tmc_uart.py` | TMC UART 传输（固件 `src/tmcuart.c`） |
| `tmc2130.py` / `tmc5160.py` | SPI TMC |
| `tmc2208.py` / `tmc2209.py` / `tmc2240.py` / `tmc2660.py` | UART/SPI TMC |

## 11. 传感器与块状数据（H6）

| 模块 | 用途 | 固件侧 |
|---|---|---|
| `bulk_sensor.py` | 块状传感器框架 + mux 端点 | `src/sensor_bulk.c` |
| `adxl345.py` | ADXL345 加速度 | `src/sensor_adxl345.c` |
| `mpu9250.py` / `icm20948.py` | MPU/ICM 加速度 | `src/sensor_mpu9250.c` 等 |
| `lis2dw.py` / `lis3dh.py` / `bmi160.py` | LIS/BMI 加速度 | `src/sensor_lis2dw.c` 等 |
| `angle.py` | 磁编码角度 | `src/sensor_angle.c` |
| `ldc1612.py` | 涡流位移 | `src/sensor_ldc1612.c` |
| `hx71x.py` | HX711/HX717 | `src/sensor_hx71x.c` |
| `ads1220.py` / `ads131m0x.py` / `ads1x1x.py` | 外部 ADC | `src/sensor_ads1220.c` 等 |
| `load_cell.py` / `load_cell_probe.py` | 称重传感器/探针 | `src/sensor_bulk.c`、`trigger_analog` |
| `ds18b20.py` | 1-Wire 温度 | `src/thermocouple.c`?（实际走 `ds18b20` 命令） |
| `aht10.py` / `bme280.py` / `sht3x.py` / `htu21d.py` / `lm75.py` | I2C 温湿度 | I2C |
| `temperature_*` | 见 H1 | — |
| `canbus_stats.py` / `canbus_ids.py` | CAN 状态与节点分配 | CAN |

## 12. G-Code 宏与脚本（H3）

| 模块 | 用途 |
|---|---|
| `gcode_macro.py` | `[gcode_macro]` 自定义命令与变量 |
| `save_variables.py` | `SAVE_VARIABLE` 跨重启变量 |
| `delayed_gcode.py` | `[delayed_gcode]` 定时脚本 |
| `respond.py` | `RESPOND` / `M118` |
| （核心缺口）`GCodeIO` | 伪 tty / 文件输入、`ack` 协议、`stats gcodein`（G1b） |
| （核心缺口）`create_gcode_command` | 宏类模块构造 gcmd 的入口（G1b 参数访问器缺口） |

## 13. 打印流程与 SD 卡（H4）

| 模块 | 用途 |
|---|---|
| `virtual_sdcard.py` | 主机侧虚拟 SD 卡（`SDCARD_PRINT_FILE`、`M24/M25`） |
| `print_stats.py` | 打印统计 |
| `display_status.py` | `M73` 进度与 `M117` 状态 |
| `pause_resume.py` | `PAUSE`/`RESUME`/`CANCEL_PRINT` + 三个端点 |
| `exclude_object.py` | 排除对象 |
| `sdcard_loop.py` | SD 循环打印 |
| `firmware_retraction.py` | G10/G11（见第 6 节） |
| （固件）`sdiocmds.c` | SDIO/SD 卡固件侧 |

## 14. 显示与菜单（H8）

| 模块 | 用途 |
|---|---|
| `display/display.py` | LCD 框架、`M117`/状态显示 |
| `display/hd44780.py` | HD44780 并口 |
| `display/hd44780_spi.py` | HD44780 SPI |
| `display/aip31068_spi.py` | AiP31068 |
| `display/st7920.py` | ST7920 图形 |
| `display/uc1701.py` | UC1701 图形 |
| `display/menu.py` + `display/menu_keys.py` + `display/display.cfg` + `display/menu.cfg` | 菜单系统 |
| `display/font8x14.py` | 字库数据 |
| （固件）`lcd_hd44780.c` / `lcd_st7920.c` | 固件侧 |

## 15. 主机运行时与调试（H11）

| 模块 | 用途 | 状态 |
|---|---|---|
| `statistics.py` | 周期上报主机统计（`/proc` 等） | ⬜ |
| `error_mcu.py` | MCU 错误更详细信息（供 shutdown 分析） | ⬜（B2） |
| `garbage_collection.py` | Python GC 调优 | N/A（Rust 无 GC） |
| `aio_executor.py` | Python 线程池 executor | N/A（Rust 用 `spawn_blocking`） |
| `parsedump.py` | 开发期解析 serial dump | N/A（可选工具） |

## 16. 固件命令模块对照

| 固件模块 | 主机侧状态 | TODO |
|---|---|---|
| `adccmds.c` | ✅ `cmd/adc.rs`、`mcu/resource/adc.rs` | F5 |
| `basecmd.c` | ✅ `config_reset` 等 | B2 |
| `gpiocmds.c` | ✅ 数字输出 | F3 |
| `pwmcmds.c` | ✅ 硬件/软件 PWM | F4 |
| `spicmds.c` / `spi_software.c` | ✅ | F6 |
| `i2ccmds.c` / `i2c_software.c` | ✅ | F7 |
| `command.c` / `sched.c` | ✅ 协议/帧（主机侧 `msg`/`mcu`） | — |
| `stepper.c` | ◐ 只有命令定义，无 step 生成 | C1 |
| `trsync.c` / `endstop.c` | ⬜ | F8 |
| `buttons.c` | ⬜ | F9 |
| `pulse_counter.c` | ⬜ | F9 |
| `trigger_analog.c` | ⬜ | F9 |
| `initial_pins.c` | ⬜ | F9 |
| `sdiocmds.c` | ⬜ | F9/H4 |
| `neopixel.c` | ⬜ | F9/H2 |
| `lcd_hd44780.c` / `lcd_st7920.c` | ⬜ | F9/H8 |
| `sensor_ads1220.c` / `sensor_ads131m0x.c` / `sensor_adxl345.c` / `sensor_angle.c` / `sensor_bmi160.c` / `sensor_hx71x.c` / `sensor_icm20948.c` / `sensor_ldc1612.c` / `sensor_lis2dw.c` / `sensor_mpu9250.c` | ⬜ | F9/H6 |
| `sensor_bulk.c` | ⬜ | F9/H6 |
| `sos_filter.c` | ⬜（加速度计滤波） | F9/H6 |
| `thermocouple.c` | ⬜ | F9/H1 |
| `tmcuart.c` | ⬜ | F9/H5 |
| `debugcmds.c` | N/A（DEBUG 调试口） | — |

## 17. API 端点对照

见 `docs/klippy/third-party-dev/api-reference.md` 与 `api/endpoints/mod.rs` 的状态表。
已完成：`info`、`list_endpoints`、`objects/list`、`objects/query`、`objects/subscribe`、
`gcode/help`、`gcode/script`、`gcode/restart`、`gcode/firmware_restart`、
`gcode/subscribe_output`。未做：`emergency_stop`、`register_remote_method`、
`pause_resume/{pause,resume,cancel}`、`query_endstops/status`、`bed_mesh/dump_mesh`、
其余 `*/dump_*` mux（B4）。

## 18. 判为 N/A 的说明

- **`garbage_collection.py`**：在长 move 期间关闭 CPython 分代 GC 以压延迟抖动
  （上游 `garbage_collection.py:20` 的阈值与 `reactor.py` 的 latency 钩子同源）。Rust
  无 GC，延迟钩子已由 `reactor.rs` 的 `set_latency_notifier` 承接（A1b），模块本身不移植。
- **`aio_executor.py`**：把阻塞的 Python 调用丢进线程池，避免占住 greenlet 主循环。
  Rust 侧对应 `spawn_blocking`，且机器侧已显式化（A3），无对应模块。
- **`parsedump.py`**：离线解析固件串口 dump 的开发脚本，等价于调试工具；可将来做成
  `klipperx` 子命令，但不是功能缺口。
- **`samd_sercom.py` / `replicape.py`**：与具体芯片/主板绑定的引脚复用与杂项芯片配置，
  等对应固件与硬件进入支持范围再评估。
- **`sos_filter.c`**：二阶 sigma-delta 滤波器，是加速度计/resonance 的固件侧细节，
  随 H6。

## 19. 记录

- 本次审计只做「盘点 + 落 TODO」，**未改任何 Rust 代码**。
- 顺带发现一处过期文档：`docs/klippy/developer-manual/README.md:62` 仍写
  `gcode/subscribe_output` 未做，实际已完成（`api/endpoints/gcode.rs`、`endpoints/mod.rs`
  状态表）；已在本轮修正。
- **同日追加**：按「框架优先」把待办重新分级，框架级抽到 `TODO.md` 文首
  「框架优先（FW1…FW9）」。判定口径：负责**定接口 / 定生命周期 / 定数据形状**、被多个
  extras 共用的是框架；只消费已有接口、自己就是一个 `[section]` 的是模块。本审计只负责
  「有什么」，不负责「先做哪个」。
- 本文件的分类若与 `TODO.md` 冲突，以 `TODO.md` 为准（那里是执行清单）。
