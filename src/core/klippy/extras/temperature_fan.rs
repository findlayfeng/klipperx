//! `[temperature_fan <name>]` — a fan whose speed follows a temperature sensor.
//!
//! Upstream's `klippy/extras/temperature_fan.py`: the section is a
//! [`Fan`](crate::core::klippy::extras::fan) core plus a sensor from `heaters`
//! and a control loop (`watermark` or `pid`) that turns each reading into a
//! fan speed. `SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=<name>` retargets it.
//!
//! | option | meaning |
//!|---|---|
//! | `pin` | the fan's PWM pin, required (via the `Fan` core) |
//! | `min_temp` / `max_temp` | the sensor's allowed range, required (`> -273.15`, `> min_temp`) |
//! | `max_speed` | duty ceiling of the control loop (default 1, `0 < .. ≤ 1`) |
//! | `min_speed` | duty floor while the loop is running (default 0.3, `0 ..= 1`) |
//! | `target_temp` | initial target (default 40, or `max_temp` when that is below 40; within `min_temp ..= max_temp`) |
//! | `control` | `watermark` or `pid`, required |
//! | `max_delta` | `control: watermark` hysteresis (default 2.0, `> 0`) |
//! | `pid_Kp` / `pid_Ki` / `pid_Kd` | `control: pid` constants, required, divided by 255 |
//! | `pid_deriv_time` | `control: pid` derivative horizon (default 2.0, `> 0`) |
//! | `sensor_type` / `sensor_pin` / … | read by the `heaters` sensor factory |
//!
//! # What is not here
//!
//! * **Speed-update suppression.** Upstream's `set_tf_speed` skips a fan write
//!   that changed little within `0.75 * MAX_FAN_TIME` of the last one, measured
//!   against the sensor's report period (`speed_delay`). The [`Sensor`](crate::core::klippy::extras::heaters::Sensor)
//!   interface here does not carry a report period, so every computed speed is
//!   written; the `Fan` core itself already drops an unchanged request.
//! * Print-time scheduling of the write — C1d, see [`fan`](crate::core::klippy::extras::fan).

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::fan::Fan;
use crate::core::klippy::extras::heaters::{self, Sensor};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[temperature_fan <name>]`) exists upstream
// (`temperature_fan.py:184`).
section!("temperature_fan", order = 30, prefix = load_config_prefix);

/// The default minimum a temperature option accepts (`KELVIN_TO_CELSIUS`).
const KELVIN_TO_CELSIUS: f64 = -273.15;
/// The temperature a PID's first derivative starts from (`AMBIENT_TEMP`).
const AMBIENT_TEMP: f64 = 25.0;
/// The divisor upstream stores the PID constants over (`PID_PARAM_BASE`).
const PID_PARAM_BASE: f64 = 255.0;

/// The control algorithm, as upstream's `ControlBangBang` / `ControlPID`.
enum Control {
    /// `control: watermark`.
    BangBang { max_delta: f64, heating: bool },
    /// `control: pid`.
    Pid {
        kp: f64,
        ki: f64,
        kd: f64,
        min_deriv_time: f64,
        temp_integ_max: f64,
        prev_temp: f64,
        prev_temp_time: f64,
        prev_temp_deriv: f64,
        prev_temp_integ: f64,
    },
}

impl Control {
    /// The fan speed one reading asks for (`temperature_callback`).
    ///
    /// Bang-bang turns the fan fully on above `target + max_delta` and off
    /// below `target - max_delta`; PID computes a bounded correction and maps
    /// it onto `min_speed ..= max_speed` the way upstream does.
    fn speed(
        &mut self,
        read_time: f64,
        temp: f64,
        target: f64,
        min_speed: f64,
        max_speed: f64,
    ) -> f64 {
        match self {
            Control::BangBang { max_delta, heating } => {
                if *heating && temp >= target + *max_delta {
                    *heating = false;
                } else if !*heating && temp <= target - *max_delta {
                    *heating = true;
                }
                if *heating {
                    0.0
                } else {
                    max_speed
                }
            }
            Control::Pid {
                kp,
                ki,
                kd,
                min_deriv_time,
                temp_integ_max,
                prev_temp,
                prev_temp_time,
                prev_temp_deriv,
                prev_temp_integ,
            } => {
                let time_diff = read_time - *prev_temp_time;
                let temp_diff = temp - *prev_temp;
                let temp_deriv = if time_diff >= *min_deriv_time {
                    temp_diff / time_diff
                } else {
                    (*prev_temp_deriv * (*min_deriv_time - time_diff) + temp_diff) / *min_deriv_time
                };
                let temp_err = target - temp;
                let temp_integ =
                    (*prev_temp_integ + temp_err * time_diff).clamp(0.0, *temp_integ_max);
                let co = *kp * temp_err + *ki * temp_integ - *kd * temp_deriv;
                let bounded = co.clamp(0.0, max_speed);
                *prev_temp = temp;
                *prev_temp_time = read_time;
                *prev_temp_deriv = temp_deriv;
                if co == bounded {
                    *prev_temp_integ = temp_integ;
                }
                min_speed.max(max_speed - bounded)
            }
        }
    }
}

/// The mutable half of a [`TemperatureFan`].
struct State {
    last_temp: f64,
    target_temp: f64,
    /// The target the config asked for, and the command's `TARGET` default.
    target_temp_conf: f64,
    min_temp: f64,
    max_temp: f64,
    min_speed: f64,
    max_speed: f64,
    control: Control,
}

/// One `[temperature_fan <name>]`.
pub struct TemperatureFan {
    /// The section's sub-name — the mux key of `SET_TEMPERATURE_FAN_TARGET`.
    #[allow(dead_code)]
    name: String,
    fan: Arc<Fan>,
    /// The sensor the readings come from. Held so the callback chain that
    /// drives it stays alive, as upstream's `TemperatureFan.sensor` does.
    #[allow(dead_code)]
    sensor: Arc<dyn Sensor>,
    state: Mutex<State>,
}

impl TemperatureFan {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The command's `TARGET` default and the current speed limits
    /// (upstream's `cmd_SET_TEMPERATURE_FAN_TARGET` defaults).
    fn command_defaults(&self) -> (f64, f64, f64) {
        let state = self.lock();
        (state.target_temp_conf, state.min_speed, state.max_speed)
    }

    /// One sensor reading: record it, run the control loop, drive the fan
    /// (`TemperatureFan.temperature_callback`).
    fn temperature_callback(&self, read_time: f64, temp: f64) {
        let speed = {
            let mut state = self.lock();
            state.last_temp = temp;
            let target = state.target_temp;
            let (min_speed, max_speed) = (state.min_speed, state.max_speed);
            state
                .control
                .speed(read_time, temp, target, min_speed, max_speed)
        };
        self.set_tf_speed(speed);
    }

    /// Apply a computed speed: floor it at `min_speed`, and turn the fan off
    /// entirely while the target is not positive (`TemperatureFan.set_tf_speed`,
    /// without upstream's report-period suppression — see the module docs).
    fn set_tf_speed(&self, value: f64) {
        // Upstream: `value <= 0.` snaps to 0, anything positive is floored at
        // `min_speed`.
        let mut value = value.max(0.0);
        {
            let state = self.lock();
            if value > 0.0 && value < state.min_speed {
                value = state.min_speed;
            }
            if state.target_temp <= 0.0 {
                value = 0.0;
            }
        }
        let _ = self.fan.set_speed(value);
    }

    /// `SET_TEMPERATURE_FAN_TARGET` (`TemperatureFan.cmd_SET_TEMPERATURE_FAN_TARGET`).
    fn cmd_set_target(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let (target_conf, min_speed_default, max_speed_default) = self.command_defaults();
        let temp = gcmd.get_float_default("TARGET", target_conf)?;
        self.set_temp(temp)?;
        let min_speed = gcmd.get_float_default("MIN_SPEED", min_speed_default)?;
        let max_speed = gcmd.get_float_default("MAX_SPEED", max_speed_default)?;
        if min_speed > max_speed {
            return Err(CommandError::new(format!(
                "Requested min speed ({min_speed:.1}) is greater than max speed ({max_speed:.1})"
            )));
        }
        self.set_min_speed(min_speed)?;
        self.set_max_speed(max_speed)
    }

    /// `TemperatureFan.set_temp`: the bounds error is upstream's, colons and
    /// all (`out of range (%.1f:%.1f)`).
    fn set_temp(&self, degrees: f64) -> Result<(), CommandError> {
        let mut state = self.lock();
        if degrees != 0.0 && (degrees < state.min_temp || degrees > state.max_temp) {
            return Err(CommandError::new(format!(
                "Requested temperature ({degrees:.1}) out of range ({:.1}:{:.1})",
                state.min_temp, state.max_temp
            )));
        }
        state.target_temp = degrees;
        Ok(())
    }

    /// `TemperatureFan.set_min_speed`.
    fn set_min_speed(&self, speed: f64) -> Result<(), CommandError> {
        if speed != 0.0 && (speed < 0.0 || speed > 1.0) {
            return Err(CommandError::new(format!(
                "Requested min speed ({speed:.1}) out of range (0.0 : 1.0)"
            )));
        }
        self.lock().min_speed = speed;
        Ok(())
    }

    /// `TemperatureFan.set_max_speed`.
    fn set_max_speed(&self, speed: f64) -> Result<(), CommandError> {
        if speed != 0.0 && (speed < 0.0 || speed > 1.0) {
            return Err(CommandError::new(format!(
                "Requested max speed ({speed:.1}) out of range (0.0 : 1.0)"
            )));
        }
        self.lock().max_speed = speed;
        Ok(())
    }
}

impl PrinterObject for TemperatureFan {
    /// Upstream's `TemperatureFan.get_status`: the fan's own status plus the
    /// last reading and the target.
    fn get_status(&self, eventtime: f64) -> Value {
        let mut status = self.fan.get_status(eventtime);
        let state = self.lock();
        if let Value::Object(map) = &mut status {
            map.insert("temperature".to_string(), json!(round2(state.last_temp)));
            map.insert("target".to_string(), json!(state.target_temp));
        }
        status
    }
}

impl std::fmt::Debug for TemperatureFan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TemperatureFan")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// Upstream's `load_config_prefix` for `[temperature_fan <name>]`
/// (`temperature_fan.py:184`).
///
/// # Errors
/// A missing or invalid option, a pin that cannot be set up, an unknown
/// sensor, or a g-code registration failure.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let name = config.section().sub.clone().unwrap_or_default();

    // The fan core first, as upstream constructs it (`temperature_fan.py:20`),
    // with `default_shutdown_speed=1.` — a temperature fan that keeps running
    // when klippy dies.
    let fan = Fan::new(config, printer, 1.0)?;

    let min_temp =
        config.get_float_bounded("min_temp", None, Some(KELVIN_TO_CELSIUS), None, None, None)?;
    let max_temp = config.get_float_bounded("max_temp", None, None, None, Some(min_temp), None)?;

    let heaters = heaters::ensure(printer)?;
    let sensor = heaters.setup_sensor(config, printer)?;
    sensor.setup_minmax(min_temp, max_temp);

    let max_speed =
        config.get_float_bounded("max_speed", Some(1.0), None, Some(1.0), Some(0.0), None)?;
    let min_speed =
        config.get_float_bounded("min_speed", Some(0.3), Some(0.0), Some(1.0), None, None)?;
    // Upstream: 40 °C when the range allows it, otherwise the top of the range
    // (`temperature_fan.py:35`).
    let target_default = if max_temp > 40.0 { 40.0 } else { max_temp };
    let target_temp_conf = config.get_float_bounded(
        "target_temp",
        Some(target_default),
        Some(min_temp),
        Some(max_temp),
        None,
        None,
    )?;

    let control = match config
        .get_choice("control", &["watermark", "pid"], None)?
        .as_str()
    {
        "watermark" => Control::BangBang {
            max_delta: config.get_float_bounded(
                "max_delta",
                Some(2.0),
                None,
                None,
                Some(0.0),
                None,
            )?,
            heating: false,
        },
        _ => {
            let kp = config.get_float("pid_Kp", None)? / PID_PARAM_BASE;
            let ki = config.get_float("pid_Ki", None)? / PID_PARAM_BASE;
            let kd = config.get_float("pid_Kd", None)? / PID_PARAM_BASE;
            let min_deriv_time = config.get_float_bounded(
                "pid_deriv_time",
                Some(2.0),
                None,
                None,
                Some(0.0),
                None,
            )?;
            let temp_integ_max = if ki != 0.0 { max_speed / ki } else { 0.0 };
            Control::Pid {
                kp,
                ki,
                kd,
                min_deriv_time,
                temp_integ_max,
                prev_temp: AMBIENT_TEMP,
                prev_temp_time: 0.0,
                prev_temp_deriv: 0.0,
                prev_temp_integ: 0.0,
            }
        }
    };

    heaters.register_sensor(config)?;

    let temperature_fan = Arc::new(TemperatureFan {
        name,
        fan,
        sensor,
        state: Mutex::new(State {
            last_temp: 0.0,
            target_temp: target_temp_conf,
            target_temp_conf,
            min_temp,
            max_temp,
            min_speed,
            max_speed,
            control,
        }),
    });

    // Deliver each reading through a weak handle, so the sensor does not keep
    // the section alive (the pattern `heaters::setup_heater` uses).
    let weak = Arc::downgrade(&temperature_fan);
    temperature_fan
        .sensor
        .setup_callback(Box::new(move |read_time, temp| {
            if let Some(tf) = weak.upgrade() {
                tf.temperature_callback(read_time, temp);
            }
        }));

    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` before any section");
    let this = Arc::clone(&temperature_fan);
    let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| this.cmd_set_target(gcmd));
    let mux_key = temperature_fan.name.clone();
    gcode
        .register_mux_command(
            "SET_TEMPERATURE_FAN_TARGET",
            "TEMPERATURE_FAN",
            Some(mux_key.as_str()),
            handler,
            Some("Sets a temperature fan target and fan speed limits"),
        )
        .map_err(ConfigError::new)?;

    Ok(temperature_fan)
}

/// Upstream's `round(value, 2)`.
fn round2(value: f64) -> f64 {
    (value * 100.0).round() / 100.0
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::heaters::SensorCallback;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
    };
    use crate::core::klippy::reactor::ManualReactor;

    /// A PWM that records what it was told.
    #[derive(Default)]
    struct FakePwm {
        updates: Mutex<Vec<f64>>,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: bool) {}
        fn setup_start_value(&self, _start: f64, _shutdown: f64) {}
        fn set_pwm(&self, _clock: u32, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn update_pwm(&self, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    /// A chip that hands out a [`FakePwm`] per setup and refuses everything else.
    #[derive(Default)]
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }
        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm::default());
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A sensor that accepts everything, for `setup_sensor`.
    #[derive(Debug)]
    struct FakeSensor;

    impl heaters::Sensor for FakeSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}
        fn setup_callback(&self, _callback: SensorCallback) {}
    }

    /// A printer with `gcode` and `pins` over a fake chip, and `heaters` with
    /// a `Fake` sensor type registered.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        // As `fan`'s tests do: the dispatcher turns ready when the printer is.
        printer.send_event(&KlippyEvent::KlippyReady);
        let heaters = heaters::ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|config, _printer| {
                // Claim `sensor_pin` like the real sensor factories do, so the
                // option-matrix test can require it when it is present.
                let _ = config.get_str("sensor_pin");
                Ok(Arc::new(FakeSensor) as Arc<dyn heaters::Sensor>)
            }),
        );
        (printer, chip)
    }

    /// A `[temperature_fan <name>]` section with the corpus option set plus
    /// `options`. Keys are folded to lowercase the way the parser stores them.
    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("temperature_fan", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// The option set each `temperature_fan` instance in the upstream
    /// `temperature.cfg` corpus uses, with `sensor_type` swapped for `Fake`.
    fn corpus_section(name: &str) -> ConfigSection {
        section(
            name,
            &[
                ("pin", "PH6"),
                ("min_temp", "0"),
                ("max_temp", "100"),
                ("control", "watermark"),
                ("sensor_type", "Fake"),
            ],
        )
    }

    fn load(
        printer: &Arc<Printer>,
        section: &ConfigSection,
    ) -> Result<Arc<TemperatureFan>, ConfigError> {
        let identifier = section.identifier();
        let object = load_config_prefix(&wrap(section), printer)?;
        printer
            .add_object(identifier.as_str(), object)
            .expect("one object per section in a fresh printer");
        Ok(printer
            .lookup_object_as::<TemperatureFan>(&identifier)
            .expect("the factory builds a TemperatureFan"))
    }

    fn pwm(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    fn updates(pwm: &FakePwm) -> Vec<f64> {
        pwm.updates.lock().unwrap().clone()
    }

    #[test]
    fn test_the_corpus_option_sets_load_with_upstream_defaults() {
        // Every instance in `test/klippy/temperature.cfg` shares this option
        // set (six sections: max6675/max31855/max31856/max31865/custom
        // thermistor/custom adc); only `sensor_type` differs, and `Fake`
        // stands in for those.
        for name in [
            "test_max6675",
            "test_max31855",
            "test_max31856",
            "test_max31865",
            "test_custom_thermistor",
            "test_custom_adc",
        ] {
            let (printer, _chip) = printer();
            let tf = load(&printer, &corpus_section(name)).unwrap();
            let state = tf.lock();
            assert_eq!(state.min_speed, 0.3, "{name}");
            assert_eq!(state.max_speed, 1.0, "{name}");
            // `max_temp: 100` allows the 40 °C default target.
            assert_eq!(state.target_temp_conf, 40.0, "{name}");
            assert_eq!(state.target_temp, 40.0, "{name}");
            drop(state);
            let status = tf.get_status(0.0);
            assert_eq!(status["target"], json!(40.0));
            assert_eq!(status["temperature"], json!(0.0));
            assert_eq!(status["rpm"], Value::Null);
        }
    }

    #[test]
    fn test_a_low_max_temp_takes_the_target_from_the_range() {
        // Upstream: the default target is 40 only when `max_temp` allows it
        // (`temperature_fan.py:35`).
        let (printer, _chip) = printer();
        let tf = load(
            &printer,
            &section(
                "test_cool",
                &[
                    ("pin", "PH6"),
                    ("min_temp", "0"),
                    ("max_temp", "30"),
                    ("control", "watermark"),
                    ("sensor_type", "Fake"),
                ],
            ),
        )
        .unwrap();

        assert_eq!(tf.lock().target_temp_conf, 30.0);
    }

    #[test]
    fn test_the_pid_control_reads_its_options() {
        let (printer, _chip) = printer();
        let tf = load(
            &printer,
            &section(
                "test_pid",
                &[
                    ("pin", "PH6"),
                    ("min_temp", "0"),
                    ("max_temp", "100"),
                    ("control", "pid"),
                    ("pid_Kp", "255"),
                    ("pid_Ki", "25.5"),
                    ("pid_Kd", "127.5"),
                    ("pid_deriv_time", "1.5"),
                    ("sensor_type", "Fake"),
                ],
            ),
        )
        .unwrap();

        let state = tf.lock();
        assert!(matches!(state.control, Control::Pid { .. }));
        assert_eq!(state.min_speed, 0.3);
        assert_eq!(state.max_speed, 1.0);
    }

    #[test]
    fn test_the_bang_bang_control_drives_the_fan() {
        let (printer, chip) = printer();
        // `kick_start_time: 0` keeps the recorded duties exactly the control
        // loop's requests.
        let tf = load(
            &printer,
            &section(
                "test_fan",
                &[
                    ("pin", "PH6"),
                    ("kick_start_time", "0"),
                    ("min_temp", "0"),
                    ("max_temp", "100"),
                    ("control", "watermark"),
                    ("max_delta", "2"),
                    ("sensor_type", "Fake"),
                ],
            ),
        )
        .unwrap();

        // 25 °C is below target - max_delta: the fan settles off (an initial
        // speed of 0 is an unchanged request, so the fan writes nothing).
        tf.temperature_callback(0.0, 25.0);
        assert!(updates(&pwm(&chip, 0)).is_empty());
        // 50 °C is above target + max_delta: full speed.
        tf.temperature_callback(1.0, 50.0);
        assert_eq!(updates(&pwm(&chip, 0)), [1.0]);
        // Back below the low watermark: off again.
        tf.temperature_callback(2.0, 30.0);
        assert_eq!(updates(&pwm(&chip, 0)), [1.0, 0.0]);
    }

    #[test]
    fn test_the_pid_output_stays_within_the_speed_bounds() {
        let (printer, chip) = printer();
        let tf = load(
            &printer,
            &section(
                "test_pid",
                &[
                    ("pin", "PH6"),
                    ("kick_start_time", "0"),
                    ("min_temp", "0"),
                    ("max_temp", "250"),
                    ("control", "pid"),
                    ("pid_Kp", "64"),
                    ("pid_Ki", "1.4"),
                    ("pid_Kd", "128"),
                    ("sensor_type", "Fake"),
                ],
            ),
        )
        .unwrap();

        // Upstream maps the bounded correction as `max(min_speed, max_speed
        // - bounded)`: a reading *below* target (positive error) saturates the
        // correction, so the fan settles on its floor; a reading *above*
        // target drives it to full speed. Either way the duty stays inside
        // `min_speed ..= max_speed`.
        tf.temperature_callback(0.0, 25.0);
        assert_eq!(updates(&pwm(&chip, 0)), [0.3]);
        tf.temperature_callback(3.0, 60.0);
        assert_eq!(updates(&pwm(&chip, 0)), [0.3, 1.0]);
    }

    #[test]
    fn test_set_temperature_fan_target_updates_the_target() {
        let (printer, _chip) = printer();
        let tf = load(&printer, &corpus_section("test_fan")).unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();

        gcode
            .run_script_sync(
                "SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=test_fan TARGET=50 MIN_SPEED=0.2",
            )
            .unwrap();

        let state = tf.lock();
        assert_eq!(state.target_temp, 50.0);
        assert_eq!(state.min_speed, 0.2);
        // The configured default survives as the no-argument `TARGET` default.
        assert_eq!(state.target_temp_conf, 40.0);
    }

    #[test]
    fn test_set_temperature_fan_target_rejects_out_of_range_values() {
        let (printer, _chip) = printer();
        let tf = load(&printer, &corpus_section("test_fan")).unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();

        let err = gcode
            .run_script_sync("SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=test_fan TARGET=300")
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Requested temperature (300.0) out of range (0.0:100.0)"
        );

        let err = gcode
            .run_script_sync(
                "SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=test_fan MIN_SPEED=0.9 MAX_SPEED=0.5",
            )
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Requested min speed (0.9) is greater than max speed (0.5)"
        );

        let err = gcode
            .run_script_sync("SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=test_fan MIN_SPEED=-0.5")
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Requested min speed (-0.5) out of range (0.0 : 1.0)"
        );

        // The mux key is per-section: another name is not this fan's command.
        let err = gcode
            .run_script_sync("SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=other TARGET=50")
            .unwrap_err();
        assert!(err.to_string().contains("TEMPERATURE_FAN"), "{err}");

        // A target of 0 (fan off) is always in range.
        gcode
            .run_script_sync("SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=test_fan TARGET=0")
            .unwrap();
        assert_eq!(tf.lock().target_temp, 0.0);
    }

    #[test]
    fn test_an_unknown_sensor_is_reported_the_way_heaters_reports_it() {
        let (printer, _chip) = printer();

        let err = load(
            &printer,
            &section(
                "test_fan",
                &[
                    ("pin", "PH6"),
                    ("min_temp", "0"),
                    ("max_temp", "100"),
                    ("control", "watermark"),
                    ("sensor_type", "nope"),
                ],
            ),
        )
        .unwrap_err();

        assert_eq!(err.to_string(), "Unknown temperature sensor 'nope'");
    }

    #[test]
    fn test_an_invalid_control_choice_names_the_option_and_section() {
        let (printer, _chip) = printer();

        let err = load(
            &printer,
            &section(
                "test_fan",
                &[
                    ("pin", "PH6"),
                    ("min_temp", "0"),
                    ("max_temp", "100"),
                    ("control", "linear"),
                    ("sensor_type", "Fake"),
                ],
            ),
        )
        .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice 'linear' for option 'control' in section 'temperature_fan test_fan' \
             is not a valid choice"
        );
    }

    #[test]
    fn test_a_missing_pin_names_the_prefixed_section() {
        let (printer, _chip) = printer();

        let err = load(
            &printer,
            &section(
                "test_fan",
                &[
                    ("min_temp", "0"),
                    ("max_temp", "100"),
                    ("control", "watermark"),
                    ("sensor_type", "Fake"),
                ],
            ),
        )
        .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'temperature_fan test_fan' must be specified"
        );
    }

    #[test]
    fn test_every_option_the_corpus_sets_is_read_through_the_tracker() {
        // The option matrix in one assert: `validate.rs:48` rejects any option
        // the factory did not read, so run the corpus option set through a
        // *tracked* wrapper and require every key to be recorded.
        let (printer, _chip) = printer();
        let text = "[temperature_fan test_fan]\n\
                    pin: PH6\n\
                    min_temp: 0\n\
                    max_temp: 100\n\
                    control: watermark\n\
                    sensor_type: Fake\n\
                    sensor_pin: PE4\n";
        let config = Config::from_text(text).expect("parses").0;
        let section = config
            .get_section("temperature_fan test_fan")
            .expect("the section parses");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        load_config_prefix(&wrapper, &printer).expect("the option set loads");

        for option in section.parameters.keys() {
            assert!(
                access.contains("temperature_fan test_fan", option),
                "option '{option}' was not read"
            );
        }
    }
}
