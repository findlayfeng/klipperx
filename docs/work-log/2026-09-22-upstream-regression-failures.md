# 上游回归测试失败原因分析

> **来源**：从 `docs/klippy/developer-manual/regression-tests.md` 的「忽略列表 → 失败原因分析」、
> `TODO.md` 的「当前失败原因统计」与 `KLIPPERX_UPSTREAM_ALL=1` 的实跑结果整理去重。
>
> **运行方式**：`KLIPPERX_UPSTREAM_ALL=1 cargo test -p klipperx --lib upstream_test_cases_run`
>
> **快照**：2026-09-22（T1 → T2 `stepper_enable` → T7 温度传感器全部落地、真板与虚拟 MCU 验证之后）
> **实跑结果**：236 次失败 / 1 次通过 / 2 次因字典未构建跳过（`pru`）/ 0 条无字典（共 239 次运行）

> **T7 已全部解决**：`Unknown temperature sensor` 类失败归零。总失败数仍是 236，因为修好一个
> 缺口只是把首次失败点往后移——原来止步于温度传感器的 4 条运行，现在前进到 `[extruder]`（T3）。

---

## 失败原因总览

按**首次失败原因**归类（一次运行只计入第一个错误）：

| 首次失败原因 | 条数 | 对应工单 |
|--------------|------|----------|
| `Section 'extruder' is not a valid config section` | 102 | T3 |
| `Unknown pin chip name 'probe'` | 36 | T4 |
| 运动学未实现（`delta` 13、`corexy` 11、`generic_cartesian` 4、`hybrid_corexy` 2、`rotary_delta` 2、`polar` / `corexz` / `hybrid_corexz` / `deltesian` / `winch` 各 1） | 37 | T5 |
| TMC 相关（段未实现 21 + pin chip 未知 9） | 30 | T6 |
| `Section 'stepper_z1' is not a valid config section` | 5 | T10 |
| `output_pin` 选项（`value` 超限 5 + `scale` 不识别 1） | 6 | T8 |
| 其余单实例段（`safe_z_home` 3、`restart_method` 3、`static_digital_output` 2，及其余 12 个各 1） | 20 | T9 |
| 温度传感器 | **0** | T7 ✅ |

（102 + 36 + 37 + 30 + 5 + 6 + 20 = 236。）

### T6 / T5 细分

T6 由两类错误组成：

| 失败原因 | 条数 |
|----------|------|
| `Section 'tmc2209 stepper_x' is not a valid config section` | 13 |
| `Unknown pin chip name 'tmc2209_stepper_x'` | 6 |
| `Section 'tmc2208 stepper_x' is not a valid config section` | 3 |
| `Unknown pin chip name 'tmc2130_stepper_x'` | 3 |
| `Section 'tmc5160 stepper_x'` / `tmc2130 stepper_x` / `tmc2660 stepper_x` 段未实现 | 2 / 2 / 1 |

T5 按 `[printer] kinematics` 取值细分（`cartesian` / `none` 已实现）：

| `kinematics` | 条数 |
|--------------|------|
| `delta` | 13 |
| `corexy` | 11 |
| `generic_cartesian` | 4 |
| `rotary_delta` | 2 |
| `hybrid_corexy` | 2 |
| `polar` / `corexz` / `hybrid_corexz` / `deltesian` / `winch` | 各 1 |

---

## 按领域分组工单

### T3：`extruder` + `heater_bed` + `fan`（102 次失败）

- **失败原因**：`Section 'extruder' is not a valid config section`
- **影响**：`commands.test`、`out_of_bounds.test`、`pressure_advance.test`、`temperature.test`
  与 `printers.test` 的绝大多数运行（Voron2、BigTreeTech Octopus/SKR 系列、Prusa Mini+、
  Creality Ender 系列等）
- **依赖**：H1（heaters / heater_bed / fan）
- **说明**：配置在最前面的 `[extruder]` 处就停下，是当前数量最大的单点。温度传感器本身
  （T7）已不再是任何运行的首次失败原因

### T4：`probe` / `bltouch` / endstop pin chip（36 次失败）

- **失败原因**：`Unknown pin chip name 'probe'`
- **影响文件**：`bed_mesh.test`、`bltouch.test`、`eddy.test`、`screws_tilt_adjust.test`、
  `smart_effector.test`、`z_virtual_endstop.test`，以及 `printers.test` 里大量
  `stepper_z: endstop_pin: probe:...` 的配置
- **依赖**：F8（endstop / trsync）
- **说明**：`probe` 是虚拟 pin chip，端停/探针落地后才会被注册

### T5：运动学（37 次失败）

- **失败原因**：`Error loading kinematics '<name>' (only 'cartesian' and 'none' are implemented)`
- **细分**：见上表（`delta` 13、`corexy` 11、`generic_cartesian` 4、`rotary_delta` 2、
  `hybrid_corexy` 2、其余各 1）
- **依赖**：C1
- **说明**：当前仅支持 `cartesian` 与 `none`；`none` 已在 T1 通过

### T6：TMC 段与 TMC pin chip（30 次失败）

- **失败原因**：`[tmc2209 stepper_x]` 等段未实现（21），或 `Unknown pin chip name
  'tmc2209_stepper_x'` / `'tmc2130_stepper_x'`（9）
- **细分**：见上表
- **依赖**：H5（TMC 步进驱动）
- **说明**：段未实现是「配置里直接写 `[tmc...]`」，pin chip 未知是「用 TMC 作为 pin 的 chip
  名」；两者都随 TMC 模块落地一起解决

### T7：温度传感器 ✅ 已完成

T7 的落地缺口已全部补齐，回归里**没有任何运行**再以温度传感器为首个失败原因：

| 缺口 | 修法 |
|------|------|
| `temperature_mcu` 只注册工厂、从不出读数 | 真正建 `<mcu>:ADC_TEMPERATURE` 的 ADC、按 MCU 型号设校准（纯算术型号 + `debug_read` 型号）、读值走 `analog_in_state` |
| `AdcTemperatureBridge` 没有 ADC，`setup_adc_sample`/`handle_adc_report` 是死代码 | 真正持有 `Arc<dyn Adc>`、在 `setup_minmax` 里 `setup_adc_sample`、`Weak` 绑回调 |
| `temperature_sensor` 不持有 sensor → ADC 回调的 `Weak` 永远失败 | 对象持有 `Arc<dyn Sensor>`（上游也存 `self.sensor`） |
| `[thermistor <name>]` / `[adc_temperature <name>]` 段缺失 | 新增两个前缀段工厂，按子名注册传感器工厂 |
| 内置热敏电阻（`EPCOS 100K B57560G104F`、`TDK NTCG104LH104JT1` 等 8 个）缺失 | 把 `temperature_sensors.cfg` 的定义做成 `BUILTIN_THERMISTORS` 表 |
| `MAX6675`/`MAX31855`/`MAX31856`/`MAX31865` 缺失 | 新增 `extras/spi_temperature.rs` + `cmd/thermocouple.rs`（SPI 框架已有） |
| `temperature_combined` 缺失 | 新增 `extras/temperature_combined.rs`（定时轮询各传感器、组合、偏差/越界检查） |

验证：

- **真板**：STM32F103 `[temperature_sensor mcu_temp] sensor_type: temperature_mcu` 读数约
  **35.1 °C**（校准 `base=357.558 / slope=-767.442`）。
- **虚拟 MCU**：`debug_read` 系列（`debug_read`/`debug_write`/`debug_ping`/`debug_nop` + `pong`）
  经 `FrameMock` 往返；`thermocouple_result` 经 `FrameMock` 到达回调并解码；各芯片的
  `calc_temp`/`calc_adc` 有单测。

### T8：`output_pin` 的选项（6 次失败）

- **失败原因**：`Option 'value' in section 'output_pin stepper_xy_current' must have maximum
  of 1`（5 次）与 `Option 'scale' is not valid in section 'output_pin motor_x_pwm'`（1 次）
- **依赖**：H2（风扇与通用输出）
- **说明**：`output_pin` 的 `value` 需要支持大于 1 的取值；`scale` 选项尚未读取

### T9：其余 extras 段（20 次失败）

- **涉及**：`safe_z_home`（3）、`restart_method`（3）、`static_digital_output`（2），以及
  `bed_screws`、`dual_carriage`、`gcode_arcs`、`led`、`manual_stepper`、`display`、
  `endstop_phase`、`adc_scaled`、`replicape`、`pwm_cycle_time`、`virtual_sdcard`、
  `sx1509_duex`（pin chip）各 1
- **说明**：各 1–3 条，按域归入 H1–H12。`restart_method` 已在 `McuConfig` 里解析，但目前只在
  `connect` 阶段读取，装载期的未定义选项检查仍会拦下它

### T10：多轴 stepper（5 次失败）

- **失败原因**：`Section 'stepper_z1' is not a valid config section`
- **影响文件**：`multi_z.test`、`quad_gantry_level.test`、`z_tilt.test`，以及 `printers.test`
  里带 `[stepper_z1]` 的配置（Anycubic Vyper 等）
- **依赖**：C1
- **说明**：`stepper_z1` / `stepper_z2` 这类多轴需要 stepper 的额外轴支持

---

## 完整失败日志

以下日志来自 `KLIPPERX_UPSTREAM_ALL=1 cargo test -p klipperx --lib upstream_test_cases_run`：

```
bed_mesh.test (test/klippy/bed_mesh.cfg): test/klippy/bed_mesh.cfg: stepper_z: Unknown pin chip name 'probe'
bed_screws.test (test/klippy/bed_screws.cfg): test/klippy/bed_screws.cfg: Section 'bed_screws' is not a valid config section
bltouch.test (test/klippy/bltouch.cfg): test/klippy/bltouch.cfg: stepper_z: Unknown pin chip name 'probe'
commands.test (test/klippy/../../config/example-cartesian.cfg): test/klippy/../../config/example-cartesian.cfg: Section 'extruder' is not a valid config section
corexyuv.test (test/klippy/corexyuv.cfg): test/klippy/corexyuv.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
delta.test (test/klippy/../../config/example-delta.cfg): test/klippy/../../config/example-delta.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
delta_calibrate.test (test/klippy/delta_calibrate.cfg): test/klippy/delta_calibrate.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
dual_carriage.test (test/klippy/dual_carriage.cfg): test/klippy/dual_carriage.cfg: Section 'dual_carriage' is not a valid config section
eddy.test (test/klippy/eddy.cfg): test/klippy/eddy.cfg: stepper_z: Unknown pin chip name 'probe'
exclude_object.test (test/klippy/exclude_object.cfg): test/klippy/exclude_object.cfg: Section 'extruder' is not a valid config section
extruders.test (test/klippy/extruders.cfg): test/klippy/extruders.cfg: Section 'extruder' is not a valid config section
gcode_arcs.test (test/klippy/gcode_arcs.cfg): test/klippy/gcode_arcs.cfg: Section 'gcode_arcs' is not a valid config section
generic_cartesian.test (test/klippy/generic_cartesian.cfg): test/klippy/generic_cartesian.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
generic_cartesian_iqex.test (test/klippy/generic_cartesian_iqex.cfg): test/klippy/generic_cartesian_iqex.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
generic_cartesian_itex.test (test/klippy/generic_cartesian_itex.cfg): test/klippy/generic_cartesian_itex.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
hybrid_corexy_dual_carriage.test (test/klippy/hybrid_corexy_dual_carriage.cfg): test/klippy/hybrid_corexy_dual_carriage.cfg: Error loading kinematics 'hybrid_corexy' (only 'cartesian' and 'none' are implemented)
input_shaper.test (test/klippy/input_shaper.cfg): test/klippy/input_shaper.cfg: Section 'extruder' is not a valid config section
led.test (test/klippy/led.cfg): test/klippy/led.cfg: Section 'led lled' is not a valid config section
load_cell.test (test/klippy/load_cell.cfg): test/klippy/load_cell.cfg: Section 'extruder' is not a valid config section
macros.test (test/klippy/macros.cfg): test/klippy/macros.cfg: Section 'extruder' is not a valid config section
manual_stepper.test (test/klippy/manual_stepper.cfg): test/klippy/manual_stepper.cfg: Section 'manual_stepper basic_stepper' is not a valid config section
multi_z.test (test/klippy/multi_z.cfg): test/klippy/multi_z.cfg: Section 'stepper_z1' is not a valid config section
out_of_bounds.test (test/klippy/../../config/example-cartesian.cfg): test/klippy/../../config/example-cartesian.cfg: Section 'extruder' is not a valid config section
polar.test (test/klippy/../../config/example-polar.cfg): test/klippy/../../config/example-polar.cfg: Error loading kinematics 'polar' (only 'cartesian' and 'none' are implemented)
pressure_advance.test (test/klippy/pressure_advance.cfg): test/klippy/pressure_advance.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/example-cartesian.cfg): test/klippy/../../config/example-cartesian.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/example-corexy.cfg): test/klippy/../../config/example-corexy.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-corexz.cfg): test/klippy/../../config/example-corexz.cfg: Error loading kinematics 'corexz' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-hybrid-corexy.cfg): test/klippy/../../config/example-hybrid-corexy.cfg: Error loading kinematics 'hybrid_corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-hybrid-corexz.cfg): test/klippy/../../config/example-hybrid-corexz.cfg: Error loading kinematics 'hybrid_corexz' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-delta.cfg): test/klippy/../../config/example-delta.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-deltesian.cfg): test/klippy/../../config/example-deltesian.cfg: Error loading kinematics 'deltesian' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-rotary-delta.cfg): test/klippy/../../config/example-rotary-delta.cfg: Error loading kinematics 'rotary_delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-winch.cfg): test/klippy/../../config/example-winch.cfg: Error loading kinematics 'winch' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/generic-einsy-rambo.cfg): test/klippy/../../config/generic-einsy-rambo.cfg: Section 'tmc2130 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-fysetc-f6.cfg): test/klippy/../../config/generic-fysetc-f6.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-gt2560.cfg): test/klippy/../../config/generic-gt2560.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mini-rambo.cfg): test/klippy/../../config/generic-mini-rambo.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/generic-rambo.cfg): test/klippy/../../config/generic-rambo.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-ramps.cfg): test/klippy/../../config/generic-ramps.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-rumba.cfg): test/klippy/../../config/generic-rumba.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-ultimaker-ultimainboard-v2.cfg): test/klippy/../../config/generic-ultimaker-ultimainboard-v2.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/kit-zav3d-2019.cfg): test/klippy/../../config/kit-zav3d-2019.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-adimlab-2018.cfg): test/klippy/../../config/printer-adimlab-2018.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/printer-anycubic-4max-2018.cfg): test/klippy/../../config/printer-anycubic-4max-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-4maxpro-2.0-2021.cfg): test/klippy/../../config/printer-anycubic-4maxpro-2.0-2021.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-i3-mega-2017.cfg): test/klippy/../../config/printer-anycubic-i3-mega-2017.cfg: Section 'stepper_z1' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-kossel-2016.cfg): test/klippy/../../config/printer-anycubic-kossel-2016.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-anycubic-kossel-plus-2017.cfg): test/klippy/../../config/printer-anycubic-kossel-plus-2017.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-bq-hephestos-2014.cfg): test/klippy/../../config/printer-bq-hephestos-2014.cfg: Section 'display' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-cr5pro-ht-2022.cfg): test/klippy/../../config/printer-creality-cr5pro-ht-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-cr10-v3-2020.cfg): test/klippy/../../config/printer-creality-cr10-v3-2020.cfg: Section 'safe_z_home' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-cr10s-2017.cfg): test/klippy/../../config/printer-creality-cr10s-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-cr10s-pro-v2-2020.cfg): test/klippy/../../config/printer-creality-cr10s-pro-v2-2020.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-cr20-2018.cfg): test/klippy/../../config/printer-creality-cr20-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-cr20-pro-2019.cfg): test/klippy/../../config/printer-creality-cr20-pro-2019.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-ender5plus-2019.cfg): test/klippy/../../config/printer-creality-ender5plus-2019.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-eryone-thinker-series-v2-2020.cfg): test/klippy/../../config/printer-eryone-thinker-series-v2-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-flashforge-creator-pro-2018.cfg): test/klippy/../../config/printer-flashforge-creator-pro-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-geeetech-A10T-A20T-2021.cfg): test/klippy/../../config/printer-geeetech-A10T-A20T-2021.cfg: Section 'safe_z_home' is not a valid config section
printers.test (test/klippy/../../config/printer-hiprecy-leo-2019.cfg): test/klippy/../../config/printer-hiprecy-leo-2019.cfg: stepper_x: Unknown pin chip name 'tmc2130_stepper_x'
printers.test (test/klippy/../../config/printer-longer-lk4-pro-2019.cfg): test/klippy/../../config/printer-longer-lk4-pro-2019.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-lulzbot-mini1-2016.cfg): test/klippy/../../config/printer-lulzbot-mini1-2016.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/printer-lulzbot-mini2-2018.cfg): test/klippy/../../config/printer-lulzbot-mini2-2018.cfg: stepper_x: Unknown pin chip name 'tmc2130_stepper_x'
printers.test (test/klippy/../../config/printer-lulzbot-taz6-2017.cfg): test/klippy/../../config/printer-lulzbot-taz6-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-lulzbot-taz6-dual-v3-2017.cfg): test/klippy/../../config/printer-lulzbot-taz6-dual-v3-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-makergear-m2-2012.cfg): test/klippy/../../config/printer-makergear-m2-2012.cfg: Section 'endstop_phase stepper_x' is not a valid config section
printers.test (test/klippy/../../config/printer-makergear-m2-2016.cfg): test/klippy/../../config/printer-makergear-m2-2016.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-micromake-d1-2016.cfg): test/klippy/../../config/printer-micromake-d1-2016.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-mtw-create-2015.cfg): test/klippy/../../config/printer-mtw-create-2015.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-robo3d-r2-2017.cfg): test/klippy/../../config/printer-robo3d-r2-2017.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-seemecnc-rostock-max-v2-2015.cfg): test/klippy/../../config/printer-seemecnc-rostock-max-v2-2015.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-sovol-sv01-2020.cfg): test/klippy/../../config/printer-sovol-sv01-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-sunlu-s8-2020.cfg): test/klippy/../../config/printer-sunlu-s8-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-tevo-flash-2018.cfg): test/klippy/../../config/printer-tevo-flash-2018.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-tevo-tarantula-pro-2020.cfg): test/klippy/../../config/printer-tevo-tarantula-pro-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-velleman-k8200-2013.cfg): test/klippy/../../config/printer-velleman-k8200-2013.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-velleman-k8800-2017.cfg): test/klippy/../../config/printer-velleman-k8800-2017.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-wanhao-duplicator-i3-mini-2017.cfg): test/klippy/../../config/printer-wanhao-duplicator-i3-mini-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-wanhao-duplicator-i3-plus-2017.cfg): test/klippy/../../config/printer-wanhao-duplicator-i3-plus-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-wanhao-duplicator-i3-plus-mark2-2019.cfg): test/klippy/../../config/printer-wanhao-duplicator-i3-plus-mark2-2019.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-wanhao-duplicator-6-2016.cfg): test/klippy/../../config/printer-wanhao-duplicator-6-2016.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/printer-wanhao-duplicator-9-2018.cfg): test/klippy/../../config/printer-wanhao-duplicator-9-2018.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/generic-mightyboard.cfg): test/klippy/../../config/generic-mightyboard.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-minitronics1.cfg): test/klippy/../../config/generic-minitronics1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-melzi.cfg): test/klippy/../../config/generic-melzi.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anet-a4-2018.cfg): test/klippy/../../config/printer-anet-a4-2018.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-anet-a8-2017.cfg): test/klippy/../../config/printer-anet-a8-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anet-a8-2019.cfg): test/klippy/../../config/printer-anet-a8-2019.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anet-e10-2018.cfg): test/klippy/../../config/printer-anet-e10-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anet-e16-2019.cfg): test/klippy/../../config/printer-anet-e16-2019.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-cr10-2017.cfg): test/klippy/../../config/printer-creality-cr10-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-cr10mini-2017.cfg): test/klippy/../../config/printer-creality-cr10mini-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender2-2017.cfg): test/klippy/../../config/printer-creality-ender2-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender3-2018.cfg): test/klippy/../../config/printer-creality-ender3-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender5-2019.cfg): test/klippy/../../config/printer-creality-ender5-2019.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-tronxy-p802e-2020.cfg): test/klippy/../../config/printer-tronxy-p802e-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-tronxy-p802m-2020.cfg): test/klippy/../../config/printer-tronxy-p802m-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-tronxy-x5s-2018.cfg): test/klippy/../../config/printer-tronxy-x5s-2018.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-tronxy-x8-2018.cfg): test/klippy/../../config/printer-tronxy-x8-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-wanhao-duplicator-i3-v2.1-2017.cfg): test/klippy/../../config/printer-wanhao-duplicator-i3-v2.1-2017.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-simulavr.cfg): test/klippy/../../config/generic-simulavr.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-printrboard.cfg): test/klippy/../../config/generic-printrboard.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-printrboard-g2.cfg): test/klippy/../../config/generic-printrboard-g2.cfg: Option 'scale' is not valid in section 'output_pin motor_x_pwm'
printers.test (test/klippy/../../config/generic-alligator-r2.cfg): test/klippy/../../config/generic-alligator-r2.cfg: Section 'static_digital_output drv8825_microstepping' is not a valid config section
printers.test (test/klippy/../../config/generic-alligator-r3.cfg): test/klippy/../../config/generic-alligator-r3.cfg: Section 'static_digital_output drv8825_microstepping' is not a valid config section
printers.test (test/klippy/../../config/generic-archim2.cfg): test/klippy/../../config/generic-archim2.cfg: Section 'tmc2130 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-radds.cfg): test/klippy/../../config/generic-radds.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-ruramps-v1.3.cfg): test/klippy/../../config/generic-ruramps-v1.3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-duet2-maestro.cfg): test/klippy/../../config/generic-duet2-maestro.cfg: Section 'tmc2208 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-duet2.cfg): test/klippy/../../config/generic-duet2.cfg: Section 'tmc2660 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-duet2-duex.cfg): test/klippy/../../config/generic-duet2-duex.cfg: output_pin FAN3: Unknown pin chip name 'sx1509_duex'
printers.test (test/klippy/../../config/printer-modix-big60-2020.cfg): test/klippy/../../config/printer-modix-big60-2020.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/generic-duet3-mini.cfg): test/klippy/../../config/generic-duet3-mini.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-duet3-6hc.cfg): test/klippy/../../config/generic-duet3-6hc.cfg: Section 'tmc5160 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-duet3-6xd.cfg): test/klippy/../../config/generic-duet3-6xd.cfg: Section 'adc_scaled vref_scaled' is not a valid config section
printers.test (test/klippy/../../config/generic-azteeg-x5-mini-v3.cfg): test/klippy/../../config/generic-azteeg-x5-mini-v3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-e3-turbo.cfg): test/klippy/../../config/generic-bigtreetech-skr-e3-turbo.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-skr-v1.1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-v1.3.cfg): test/klippy/../../config/generic-bigtreetech-skr-v1.3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-v1.4.cfg): test/klippy/../../config/generic-bigtreetech-skr-v1.4.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mks-sgenl.cfg): test/klippy/../../config/generic-mks-sgenl.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-re-arm.cfg): test/klippy/../../config/generic-re-arm.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-smoothieboard.cfg): test/klippy/../../config/generic-smoothieboard.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-th3d-ezboard-lite-v1.2.cfg): test/klippy/../../config/generic-th3d-ezboard-lite-v1.2.cfg: Section 'tmc2208 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/printer-monoprice-mini-delta-2017.cfg): test/klippy/../../config/printer-monoprice-mini-delta-2017.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-monoprice-select-mini-v2-2018.cfg): test/klippy/../../config/printer-monoprice-select-mini-v2-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-cr6-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-skr-cr6-v1.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-e3-dip.cfg): test/klippy/../../config/generic-bigtreetech-skr-e3-dip.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-mini.cfg): test/klippy/../../config/generic-bigtreetech-skr-mini.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v1.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v1.2.cfg): test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v1.2.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v2.0.cfg): test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v2.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-mini-mz.cfg): test/klippy/../../config/generic-bigtreetech-skr-mini-mz.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-vyper-2021.cfg): test/klippy/../../config/printer-anycubic-vyper-2021.cfg: Section 'stepper_z1' is not a valid config section
printers.test (test/klippy/../../config/printer-monoprice-select-mini-v1-2016.cfg): test/klippy/../../config/printer-monoprice-select-mini-v1-2016.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-sovol-sv05-2022.cfg): test/klippy/../../config/printer-sovol-sv05-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-sovol-sv06-2022.cfg): test/klippy/../../config/printer-sovol-sv06-2022.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/printer-sovol-sv06-plus-2023.cfg): test/klippy/../../config/printer-sovol-sv06-plus-2023.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/printer-sunlu-t3-2022.cfg): test/klippy/../../config/printer-sunlu-t3-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/generic-creality-v4.2.7.cfg): test/klippy/../../config/generic-creality-v4.2.7.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-creality-v4.2.10.cfg): test/klippy/../../config/generic-creality-v4.2.10.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-fysetc-cheetah-v1.1.cfg): test/klippy/../../config/generic-fysetc-cheetah-v1.1.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-fysetc-cheetah-v1.2.cfg): test/klippy/../../config/generic-fysetc-cheetah-v1.2.cfg: Section 'tmc2208 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-mks-robin-e3.cfg): test/klippy/../../config/generic-mks-robin-e3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mks-robin-nano-v1.cfg): test/klippy/../../config/generic-mks-robin-nano-v1.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/generic-mks-robin-nano-v2.cfg): test/klippy/../../config/generic-mks-robin-nano-v2.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-alfawise-u30-2018.cfg): test/klippy/../../config/printer-alfawise-u30-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-cr10-smart-pro-2022.cfg): test/klippy/../../config/printer-creality-cr10-smart-pro-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-cr30-2021.cfg): test/klippy/../../config/printer-creality-cr30-2021.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-creality-cr6se-2020.cfg): test/klippy/../../config/printer-creality-cr6se-2020.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-cr6se-2021.cfg): test/klippy/../../config/printer-creality-cr6se-2021.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-ender2pro-2021.cfg): test/klippy/../../config/printer-creality-ender2pro-2021.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender3-s1-2021.cfg): test/klippy/../../config/printer-creality-ender3-s1-2021.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-ender3-s1plus-2022.cfg): test/klippy/../../config/printer-creality-ender3-s1plus-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-ender3-v2-2020.cfg): test/klippy/../../config/printer-creality-ender3-v2-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender3-v2-neo-2022.cfg): test/klippy/../../config/printer-creality-ender3-v2-neo-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-ender3max-2021.cfg): test/klippy/../../config/printer-creality-ender3max-2021.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender3pro-2020.cfg): test/klippy/../../config/printer-creality-ender3pro-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender5pro-2020.cfg): test/klippy/../../config/printer-creality-ender5pro-2020.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-ender6-2020.cfg): test/klippy/../../config/printer-creality-ender6-2020.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-creality-sermoonD1-2021.cfg): test/klippy/../../config/printer-creality-sermoonD1-2021.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-creality-sermoonV1-2022.cfg): test/klippy/../../config/printer-creality-sermoonV1-2022.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-elegoo-neptune2-2021.cfg): test/klippy/../../config/printer-elegoo-neptune2-2021.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-eryone-er20-2021.cfg): test/klippy/../../config/printer-eryone-er20-2021.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/printer-flsun-q5-2020.cfg): test/klippy/../../config/printer-flsun-q5-2020.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-flsun-qqs-2020.cfg): test/klippy/../../config/printer-flsun-qqs-2020.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-fokoos-odin5-f3-2021.cfg): test/klippy/../../config/printer-fokoos-odin5-f3-2021.cfg: Option 'restart_method' is not valid in section 'mcu'
printers.test (test/klippy/../../config/printer-geeetech-301-2019.cfg): test/klippy/../../config/printer-geeetech-301-2019.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-kingroon-kp3s-2020.cfg): test/klippy/../../config/printer-kingroon-kp3s-2020.cfg: Section 'safe_z_home' is not a valid config section
printers.test (test/klippy/../../config/printer-longer-lk4x-2022.cfg): test/klippy/../../config/printer-longer-lk4x-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-tronxy-x5sa-v6-2019.cfg): test/klippy/../../config/printer-tronxy-x5sa-v6-2019.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-tronxy-x5sa-pro-2020.cfg): test/klippy/../../config/printer-tronxy-x5sa-pro-2020.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-tronxy-xy-2-Pro-2020.cfg): test/klippy/../../config/printer-tronxy-xy-2-Pro-2020.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-twotrees-sapphire-plus-sp-5-v1-2020.cfg): test/klippy/../../config/printer-twotrees-sapphire-plus-sp-5-v1-2020.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-twotrees-sapphire-plus-sp-5-v1.1-2021.cfg): test/klippy/../../config/printer-twotrees-sapphire-plus-sp-5-v1.1-2021.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-twotrees-sapphire-pro-sp-3-2020.cfg): test/klippy/../../config/printer-twotrees-sapphire-pro-sp-3-2020.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-voxelab-aquila-2021.cfg): test/klippy/../../config/printer-voxelab-aquila-2021.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-fysetc-cheetah-v2.0.cfg): test/klippy/../../config/generic-fysetc-cheetah-v2.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/printer-artillery-genius-pro-2022.cfg): test/klippy/../../config/printer-artillery-genius-pro-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-artillery-sidewinder-x2-2022.cfg): test/klippy/../../config/printer-artillery-sidewinder-x2-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-artillery-sidewinder-x3-plus-2024.cfg): test/klippy/../../config/printer-artillery-sidewinder-x3-plus-2024.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-creality-ender5-s1-2023.cfg): test/klippy/../../config/printer-creality-ender5-s1-2023.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-elegoo-neptune3-pro-2023.cfg): test/klippy/../../config/printer-elegoo-neptune3-pro-2023.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/generic-mellow-fly-gemini-v1.cfg): test/klippy/../../config/generic-mellow-fly-gemini-v1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mellow-fly-gemini-v2.cfg): test/klippy/../../config/generic-mellow-fly-gemini-v2.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-e3-rrf-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-e3-rrf-v1.1.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-gtr.cfg): test/klippy/../../config/generic-bigtreetech-gtr.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-pro.cfg): test/klippy/../../config/generic-bigtreetech-skr-pro.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-2.cfg): test/klippy/../../config/generic-bigtreetech-skr-2.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-flyboard.cfg): test/klippy/../../config/generic-flyboard.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/generic-I3DBEEZ9.cfg): test/klippy/../../config/generic-I3DBEEZ9.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mellow-fly-cdy-v3.cfg): test/klippy/../../config/generic-mellow-fly-cdy-v3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mellow-fly-e3-v2.cfg): test/klippy/../../config/generic-mellow-fly-e3-v2.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mellow-super-infinty-hv.cfg): test/klippy/../../config/generic-mellow-super-infinty-hv.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/generic-mks-monster8.cfg): test/klippy/../../config/generic-mks-monster8.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mks-robin-nano-v3.cfg): test/klippy/../../config/generic-mks-robin-nano-v3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-prusa-buddy.cfg): test/klippy/../../config/generic-prusa-buddy.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/generic-th3d-ezboard-v2.0.cfg): test/klippy/../../config/generic-th3d-ezboard-v2.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/printer-biqu-b1-se-plus-2022.cfg): test/klippy/../../config/printer-biqu-b1-se-plus-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-prusa-mini-plus-2020.cfg): test/klippy/../../config/printer-prusa-mini-plus-2020.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-octopus-v1.1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.0.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-fysetc-s6.cfg): test/klippy/../../config/generic-fysetc-s6.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-fysetc-s6-v2.cfg): test/klippy/../../config/generic-fysetc-s6-v2.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-fysetc-spider.cfg): test/klippy/../../config/generic-fysetc-spider.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-ldo-leviathan-v1.2.cfg): test/klippy/../../config/generic-ldo-leviathan-v1.2.cfg: Section 'tmc5160 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-mks-rumba32-v1.0.cfg): test/klippy/../../config/generic-mks-rumba32-v1.0.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-ratrig-v-minion-2021.cfg): test/klippy/../../config/printer-ratrig-v-minion-2021.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-tronxy-crux1-2022.cfg): test/klippy/../../config/printer-tronxy-crux1-2022.cfg: Option 'restart_method' is not valid in section 'mcu'
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-max-ez.cfg): test/klippy/../../config/generic-bigtreetech-octopus-max-ez.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-biqu-bx-2021.cfg): test/klippy/../../config/printer-biqu-bx-2021.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/generic-bigtreetech-skr-3.cfg): test/klippy/../../config/generic-bigtreetech-skr-3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m4p.cfg): test/klippy/../../config/generic-bigtreetech-manta-m4p.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m5p.cfg): test/klippy/../../config/generic-bigtreetech-manta-m5p.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.0.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-e3ez.cfg): test/klippy/../../config/generic-bigtreetech-manta-e3ez.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v3.0.cfg): test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v3.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-pico-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-skr-pico-v1.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-kobra-go-2022.cfg): test/klippy/../../config/printer-anycubic-kobra-go-2022.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-kobra-plus-2022.cfg): test/klippy/../../config/printer-anycubic-kobra-plus-2022.cfg: Option 'restart_method' is not valid in section 'mcu'
printers.test (test/klippy/../../config/generic-replicape.cfg): test/klippy/../../config/generic-replicape.cfg: Section 'replicape' is not a valid config section
printers.test (test/klippy/../../config/sample-multi-mcu.cfg): test/klippy/../../config/sample-multi-mcu.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/kit-voron2-250mm.cfg): test/klippy/../../config/kit-voron2-250mm.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
pwm.test (test/klippy/pwm.cfg): test/klippy/pwm.cfg: Section 'pwm_cycle_time cycle_pwm_pin' is not a valid config section
quad_gantry_level.test (test/klippy/z_tilt.cfg): test/klippy/z_tilt.cfg: Section 'stepper_z1' is not a valid config section
rotary_delta_calibrate.test (test/klippy/rotary_delta_calibrate.cfg): test/klippy/rotary_delta_calibrate.cfg: Error loading kinematics 'rotary_delta' (only 'cartesian' and 'none' are implemented)
screws_tilt_adjust.test (test/klippy/screws_tilt_adjust.cfg): test/klippy/screws_tilt_adjust.cfg: stepper_z: Unknown pin chip name 'probe'
sdcard_loop.test (test/klippy/sdcard_loop.cfg): test/klippy/sdcard_loop.cfg: Section 'virtual_sdcard' is not a valid config section
smart_effector.test (test/klippy/smart_effector.cfg): test/klippy/smart_effector.cfg: stepper_z: Unknown pin chip name 'probe'
temperature.test (test/klippy/temperature.cfg): test/klippy/temperature.cfg: Section 'extruder' is not a valid config section
tmc.test (test/klippy/tmc.cfg): test/klippy/tmc.cfg: stepper_x: Unknown pin chip name 'tmc2130_stepper_x'
z_tilt.test (test/klippy/z_tilt.cfg): test/klippy/z_tilt.cfg: Section 'stepper_z1' is not a valid config section
z_virtual_endstop.test (test/klippy/z_virtual_endstop.cfg): test/klippy/z_virtual_endstop.cfg: stepper_z: Unknown pin chip name 'probe'
```

---

## 推进策略

按「闭包最小 → 杠杆最大」顺序推进：

| 顺序 | 事项 | 失败次数 | 依赖 | 状态 |
|------|------|----------|------|------|
| T1 | `linuxtest.test` | — | 无 | ✅ 已通过 |
| T2 | `[stepper_enable]`（`enable_pin`） | — | 无 | ✅ 已实现；相关文件仍因后续缺节留在忽略列表 |
| T3 | `extruder` + `heater_bed` + `fan` | 102 | H1 | 待做 |
| T4 | `probe` / `bltouch` / endstop pin chip | 36 | F8 | 待做 |
| T5 | 运动学（delta / corexy / …） | 37 | C1 | 待做 |
| T6 | TMC 段与 pin chip | 30 | H5 | 待做 |
| T7 | 温度传感器 | 0 | H1 | ✅ 已完成（含真板与虚拟 MCU 验证） |
| T8 | `output_pin` 的 `value` / `scale` | 6 | H2 | 待做 |
| T9 | 其余 extras 段 | 20 | 各域 | 待做 |
| T10 | 多轴 stepper | 5 | C1 | 待做 |

---

## 复盘：计数口径与重排（后补）

> 本节是对上面快照的**读法修正**，不改写原始统计——那些数字是按「首次失败原因」数出来的，
> 保留原样以便对照。

### 计数不是工作量：别按 102 / 36 / 37 排期

上面的分组只统计**每条运行的第一个**错误，而 `load_config` 遇到第一个未知 section 就停。
因此：

- 修好一个缺口只会让该运行的首次失败点**后移**，总失败数可以纹丝不动。T7 就是例子：4 条
  运行前进到 `[extruder]`，236 → 236。
- 各组的收益**不可加**：T4（`probe` pin chip）的 36 条修完后多半前移到 `[extruder]`（T3）
  或 `bed_mesh` / `z_tilt`（H9），T4 本身拿不到 36。

### 该用的两个指标

1. **运行 × 缺口矩阵**：对每条运行列出**全部**缺口（引用的 section / `kinematics:` / pin chip
   与已实现集合的差集），而不只是第一个。据此才能看出「只差 1 个缺口」的用例与公共前缀。
   落点：`src/core/klippy/upstream.rs` 的测试模块（非致命的静态扫描，不动生产加载器）。
2. **转绿运行数 / `IGNORED` 条目数**：只降不升，每完成一个闭环必然变化。当前 1 / 36。

### 重排后的顺序

| 阶段 | 事项 | 说明 |
|------|------|------|
| 0 | 运行×缺口扫描；`IGNORED` 守卫测试；T8（`output_pin value`/`scale`）；T9 小段（`restart_method`、`static_digital_output`、`pwm_cycle_time`）；T2 收尾（自动装载 + M18/M84/SET_STEPPER_ENABLE） | 都不依赖 C1 |
| 1 | T4 `probe` 虚拟 pin chip | F8 已 ✅，toolhead/homing 已在 |
| 2 | C1 的**轴 / stepper 资源**重构（上游 `extras/stepper.py` 的 `PrinterStepper` + toolhead rails/axes） | T3/T5/T10/T6 的公共前置，避免同一处改四遍 |
| 3 | T3（extruder/heater_bed/fan，先做 H1 加热控制环）；T5（corexy 族 → delta 族 → generic_cartesian → polar）；T10 | 主战场 |
| 4 | T6（TMC，另需 F9 tmcuart）；T9 剩余 | 明确后置 |

### 收尾纪律

- **验收标准是「`.test` 从 `IGNORED` 移除后通过」**，不是「某个错误不再出现」。
- 加 **`IGNORED` 守卫测试**：某条已能通过却仍在 `IGNORED` 里时测试失败，提示移除，防止条目漂移。
- `out_of_bounds.test`（唯一 `SHOULD_FAIL`）必须在配置能装载后才能移出，否则越界检查会被配置
  错误「喂饱」；建议在该条目旁固化这个条件。

---

## 备注

- 忽略列表（`upstream.rs` 的 `IGNORED`）当前按**文件**登记，共 36 条，即除 `linuxtest.test`
  外的全部 `.test`；`KLIPPERX_UPSTREAM_ALL=1` 只绕过这一层，不影响字典是否构建。
- 总失败数 236 在 T7 完成后**没有下降**：温度传感器的 4 条运行前进到了 `[extruder]`（T3）。
  修好一个缺口只有在整条运行再无其它缺节时才会计数下降，`linuxtest.test` 是唯一的例子。
- `printers.test` 有 2 条运行声明 `DICTIONARY pru.dict host=linuxprocess.dict`，默认不构建
  `pru`；要跑需 `KLIPPERX_ARCHES=…,pru`（需 `pru-gcc`）。这 2 条计入「因字典未构建跳过」，
  不算失败。
- `out_of_bounds.test` 是唯一声明 `SHOULD_FAIL` 的用例，它期望的是**运行期**错误
  （`G1 Y9999` 越界），不是配置错误。它的首次失败是 `Section 'extruder' is not a valid
  config section`，属于 T3；等 `example-cartesian.cfg` 所需的节全部落地并从忽略列表移出后，
  反转才会真的去验越界检查，而不是被配置装载错误「喂饱」。

---

- [← work-log 首页](README.md)
- [上游功能覆盖审计 →](2026-09-21-upstream-coverage-audit.md)
- [已完成条目归档 →](2026-09-22-completed-archive.md)
