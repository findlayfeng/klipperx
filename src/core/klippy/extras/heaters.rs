//! `[heaters]` — the temperature-sensor registry.
//!
//! Upstream's `heaters.py` is two things: the registry every
//! `[temperature_sensor]`, `[extruder]` and `[heater_bed]` sets its sensor up
//! through, and the heater control loops. Only the registry is here so far; the
//! control loops arrive with `[extruder]` / `[heater_bed]`.
//!
//! The object has no `[heaters]` section of its own — upstream loads it by name
//! (`printer.load_object(config, 'heaters')`) and so does [`ensure`], which is
//! also where the built-in sensor modules are brought in (upstream reads
//! `temperature_sensors.cfg` for that).

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::adc_temperature;
use crate::core::klippy::extras::ds18b20;
use crate::core::klippy::extras::spi_temperature;
use crate::core::klippy::extras::temperature_combined;
use crate::core::klippy::extras::temperature_mcu;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name other modules look the registry up by.
pub const HEATERS_OBJECT: &str = "heaters";

/// The longest a heater PWM change may sit before the firmware falls back
/// (upstream `MAX_HEAT_TIME`, `heaters.py:15`).
#[allow(dead_code)]
const MAX_HEAT_TIME: f64 = 3.0;
/// The temperature the PID's first derivative is measured against
/// (upstream `AMBIENT_TEMP`, `heaters.py:17`).
const AMBIENT_TEMP: f64 = 25.0;
/// The divisor upstream stores PID constants over (`PID_PARAM_BASE`).
const PID_PARAM_BASE: f64 = 255.0;
/// How close a PID must settle before `TEMPERATURE_WAIT` returns
/// (`PID_SETTLE_DELTA`/`PID_SETTLE_SLOPE`).
const PID_SETTLE_DELTA: f64 = 1.0;
const PID_SETTLE_SLOPE: f64 = 0.1;

/// Called with `(read_time, temperature)` for every reading.
pub type SensorCallback = Box<dyn Fn(f64, f64) + Send + Sync>;

/// What a temperature sensor provides — the part of upstream's sensor interface
/// the registry and its consumers need.
pub trait Sensor: Send + Sync + std::fmt::Debug {
    /// The range a reading is allowed to fall in.
    fn setup_minmax(&self, min_temp: f64, max_temp: f64);

    /// Where readings are delivered.
    fn setup_callback(&self, callback: SensorCallback);
}

/// Builds one sensor from its section (upstream's `sensor_factories` entry).
pub type SensorFactory = Arc<
    dyn Fn(&ConfigWrapper, &Arc<Printer>) -> Result<Arc<dyn Sensor>, ConfigError> + Send + Sync,
>;

/// The heater control algorithms (upstream's `ControlBangBang` / `ControlPID`).
#[derive(Debug)]
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
    /// Compute the PWM value for one reading (`temperature_update`).
    fn update(&mut self, read_time: f64, temp: f64, target: f64, max_power: f64) -> f64 {
        match self {
            Control::BangBang { max_delta, heating } => {
                if *heating && temp >= target + *max_delta {
                    *heating = false;
                } else if !*heating && temp <= target - *max_delta {
                    *heating = true;
                }
                if *heating {
                    max_power
                } else {
                    0.0
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
                let bounded = co.clamp(0.0, max_power);
                *prev_temp = temp;
                *prev_temp_time = read_time;
                *prev_temp_deriv = temp_deriv;
                if co == bounded {
                    *prev_temp_integ = temp_integ;
                }
                bounded
            }
        }
    }

    /// Whether a requested temperature has not been reached yet
    /// (`check_busy`).
    fn check_busy(&self, smoothed_temp: f64, target: f64) -> bool {
        match self {
            Control::BangBang { max_delta, .. } => smoothed_temp < target - *max_delta,
            Control::Pid {
                prev_temp_deriv, ..
            } => {
                (target - smoothed_temp).abs() > PID_SETTLE_DELTA
                    || prev_temp_deriv.abs() > PID_SETTLE_SLOPE
            }
        }
    }
}

/// One configured heater (an extruder hotend, a bed, a generic heater).
///
/// Upstream's `Heater` (`klippy/extras/heaters.py:14-160`): it owns the sensor
/// callback, the bang-bang/PID control loop and the PWM output. The periodic
/// `verify_heater` check is not wired yet (upstream `verify_heater.py`).
pub struct Heater {
    /// The section's short name (`extruder`, `heater_bed`).
    name: String,
    /// The sensor built from the heater's section.
    sensor: Arc<dyn Sensor>,
    /// The heater's PWM output, when the pin was set up.
    pwm: Option<Arc<dyn PwmOut>>,
    min_temp: f64,
    max_temp: f64,
    /// `min_extrude_temp`: the reading below which extrusion is refused.
    min_extrude_temp: f64,
    max_power: f64,
    /// `1 / smooth_time`, for the smoothed temperature.
    inv_smooth_time: f64,
    state: Mutex<HeaterState>,
}

/// The mutable half of a [`Heater`].
struct HeaterState {
    target_temp: f64,
    last_temp: f64,
    smoothed_temp: f64,
    last_temp_time: f64,
    can_extrude: bool,
    last_pwm_value: f64,
    control: Control,
}

impl Heater {
    /// The heater's short name (`extruder`, `heater_bed`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The sensor this heater reads.
    pub fn sensor(&self) -> &Arc<dyn Sensor> {
        &self.sensor
    }

    /// Whether a move may extrude (`PrinterExtruder.check_move`).
    pub fn can_extrude(&self) -> bool {
        self.lock().can_extrude
    }

    /// The configured temperature range.
    pub fn temperature_range(&self) -> (f64, f64) {
        (self.min_temp, self.max_temp)
    }

    /// Set the target temperature (`Heater.set_temp`).
    ///
    /// # Errors
    /// The requested temperature is outside the configured range, as
    /// upstream's `SET_HEATER_TEMPERATURE` checks.
    pub fn set_temp(&self, degrees: f64) -> Result<(), CommandError> {
        if degrees != 0.0 && (degrees < self.min_temp || degrees > self.max_temp) {
            return Err(CommandError::new(format!(
                "Requested temperature ({:.1}) out of range ({:.1}, {:.1})",
                degrees, self.min_temp, self.max_temp
            )));
        }
        self.lock().target_temp = degrees;
        Ok(())
    }

    /// One sensor reading: run the control loop and update the smoothed
    /// temperature (`Heater.temperature_callback`).
    pub fn temperature_callback(&self, read_time: f64, temp: f64) {
        let mut state = self.lock();
        let time_diff = read_time - state.last_temp_time;
        state.last_temp = temp;
        state.last_temp_time = read_time;
        let target = state.target_temp;
        let value = state
            .control
            .update(read_time, temp, target, self.max_power);
        // Upstream schedules the change at `read_time + pwm_delay`; print-time
        // scheduling is C1d, so the output goes out immediately.
        if let Some(pwm) = &self.pwm {
            let _ = pwm.update_pwm(value);
        }
        let temp_diff = temp - state.smoothed_temp;
        let adj_time = (time_diff * self.inv_smooth_time).min(1.0);
        state.smoothed_temp += temp_diff * adj_time;
        state.can_extrude = state.smoothed_temp >= self.min_extrude_temp;
        state.last_pwm_value = value;
    }

    /// Whether a `TEMPERATURE_WAIT` for `target` must keep waiting
    /// (`Heater.check_busy`).
    pub fn check_busy(&self, target: f64) -> bool {
        let state = self.lock();
        state.control.check_busy(state.smoothed_temp, target)
    }

    /// `Heater.get_status`.
    pub fn get_status(&self) -> Value {
        let state = self.lock();
        json!({
            "temperature": (state.smoothed_temp * 100.0).round() / 100.0,
            "target": state.target_temp,
            "power": state.last_pwm_value,
        })
    }

    fn lock(&self) -> MutexGuard<'_, HeaterState> {
        self.state.lock().unwrap_or_else(|p| p.into_inner())
    }
}

impl std::fmt::Debug for Heater {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Heater").field("name", &self.name).finish()
    }
}

/// The `heaters` object: the sensor factory table and what is registered.
pub struct PrinterHeaters {
    factories: Mutex<BTreeMap<String, SensorFactory>>,
    sensors: Mutex<Vec<String>>,
    monitors: Mutex<Vec<String>>,
    heaters: Mutex<Vec<String>>,
}

impl PrinterHeaters {
    fn new() -> Self {
        Self {
            factories: Mutex::new(BTreeMap::new()),
            sensors: Mutex::new(Vec::new()),
            monitors: Mutex::new(Vec::new()),
            heaters: Mutex::new(Vec::new()),
        }
    }

    /// Register a sensor type, upstream's `add_sensor_factory`.
    pub fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(sensor_type.to_string(), factory);
    }

    /// Build the sensor a section asks for, upstream's `setup_sensor`.
    ///
    /// # Errors
    /// A missing `sensor_type`, or one no module has registered.
    pub fn setup_sensor(
        &self,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<Arc<dyn Sensor>, ConfigError> {
        let sensor_type = config.get("sensor_type", None)?;
        let factory = self
            .factories
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&sensor_type)
            .cloned()
            .ok_or_else(|| {
                ConfigError::new(format!("Unknown temperature sensor '{sensor_type}'"))
            })?;
        factory(config, printer)
    }

    /// Note a sensor that was set up, upstream's `register_sensor`.
    ///
    /// The `TEMPERATURE_WAIT` command and the `M105` g-code-id table are not
    /// wired yet; `gcode_id` is still read so the option is claimed.
    pub fn register_sensor(&self, config: &ConfigWrapper) -> Result<(), ConfigError> {
        let _ = config.get_str("gcode_id");
        self.sensors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(config.identifier());
        Ok(())
    }

    /// The registered monitor sections, for `get_status`.
    pub fn register_monitor(&self, config: &ConfigWrapper) {
        self.monitors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(config.identifier());
    }

    /// Build a heater from its section (`PrinterHeaters.setup_heater`).
    ///
    /// Reads and claims the heater options, builds the sensor and wires each of
    /// its readings into the control loop (`Heater::temperature_callback`),
    /// which is what drives the PWM: there is no timer of its own, so a heater
    /// with no readings never changes its output.
    ///
    /// `can_extrude` starts as upstream's `min_extrude_temp <= 0. or
    /// is_fileoutput` (`heaters.py:38-39`) and every reading recomputes it.
    /// File-output mode is how upstream runs its own test cases, where nothing
    /// answers the temperature queries — see [`Printer::is_fileoutput`].
    ///
    /// # Errors
    /// A duplicate heater name, an unknown sensor, or an invalid option.
    pub fn setup_heater(
        &self,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        gcode_id: Option<&str>,
    ) -> Result<Arc<Heater>, ConfigError> {
        let identifier = config.identifier();
        let short_name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        if self
            .heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .contains(&short_name)
        {
            return Err(ConfigError::new(format!(
                "Heater {short_name} already registered"
            )));
        }

        let sensor = self.setup_sensor(config, printer)?;
        let min_temp = config.get_float("min_temp", None)?;
        let max_temp =
            config.get_float_bounded("max_temp", None, None, None, Some(min_temp), None)?;
        // Upstream returns the default without range-checking it
        // (`configfile._get_wrapper`), which matters here: a bed's
        // `max_temp` (e.g. 130) is below the default `min_extrude_temp` (170).
        let min_extrude_temp = match config.get_optional_float("min_extrude_temp")? {
            Some(value) => {
                if value < min_temp {
                    return Err(ConfigError::new(format!(
                        "Option 'min_extrude_temp' in section '{identifier}' must have minimum of {min_temp}"
                    )));
                }
                if value > max_temp {
                    return Err(ConfigError::new(format!(
                        "Option 'min_extrude_temp' in section '{identifier}' must have maximum of {max_temp}"
                    )));
                }
                value
            }
            None => 170.0,
        };
        let max_power =
            config.get_float_bounded("max_power", Some(1.0), None, Some(1.0), Some(0.0), None)?;
        let smooth_time =
            config.get_float_bounded("smooth_time", Some(1.0), None, None, Some(0.0), None)?;
        let pwm_cycle_time =
            config.get_float_bounded("pwm_cycle_time", Some(0.100), None, None, Some(0.0), None)?;

        // Set up the heater pin as a PWM output, as upstream does
        // (`heaters.py:56-61`). The `max_duration` limit is off: the host
        // drives the pin immediately, not on a print-time schedule (C1d).
        let heater_pin = config.get("heater_pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let pwm = pins
            .setup_pwm(&heater_pin, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        pwm.setup_cycle_time(pwm_cycle_time, false);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.0, 0.0);

        // Build the control algorithm.
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
                Control::Pid {
                    kp,
                    ki,
                    kd,
                    min_deriv_time: smooth_time,
                    temp_integ_max: if ki != 0.0 { max_power / ki } else { 0.0 },
                    prev_temp: AMBIENT_TEMP,
                    prev_temp_time: 0.0,
                    prev_temp_deriv: 0.0,
                    prev_temp_integ: 0.0,
                }
            }
        };

        sensor.setup_minmax(min_temp, max_temp);
        self.register_sensor(config)?;
        let _ = gcode_id; // TODO H1: the M105 g-code id table

        let heater = Arc::new(Heater {
            name: short_name.clone(),
            sensor,
            pwm: Some(pwm),
            min_temp,
            max_temp,
            min_extrude_temp,
            max_power,
            inv_smooth_time: 1.0 / smooth_time,
            state: Mutex::new(HeaterState {
                target_temp: 0.0,
                last_temp: 0.0,
                smoothed_temp: 0.0,
                last_temp_time: 0.0,
                // Upstream: `min_extrude_temp <= 0. or is_fileoutput`
                // (`heaters.py:38-39`). File-output mode is how upstream runs
                // its own cases, where the temperature queries are never
                // answered and so no reading ever flips this on again.
                can_extrude: min_extrude_temp <= 0.0 || printer.is_fileoutput(),
                last_pwm_value: 0.0,
                control,
            }),
        });
        // The sensor delivers each reading to the control loop through a weak
        // handle, so the sensor does not keep the heater alive.
        let weak = Arc::downgrade(&heater);
        heater
            .sensor
            .setup_callback(Box::new(move |read_time, temp| {
                if let Some(heater) = weak.upgrade() {
                    heater.temperature_callback(read_time, temp);
                }
            }));
        self.heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(short_name.clone());
        self.register_heater_command(printer, &short_name, Arc::clone(&heater))?;
        Ok(heater)
    }

    /// Register `SET_HEATER_TEMPERATURE` for one heater
    /// (`heaters.py:63-66`, `:362`).
    fn register_heater_command(
        &self,
        printer: &Arc<Printer>,
        short_name: &str,
        heater: Arc<Heater>,
    ) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let target = gcmd.get_float_default("TARGET", 0.0)?;
            heater.set_temp(target)
        });
        gcode
            .register_mux_command(
                "SET_HEATER_TEMPERATURE",
                "HEATER",
                Some(short_name),
                handler,
                Some("Set a heater temperature"),
            )
            .map_err(ConfigError::new)
    }

    fn available_heaters(&self) -> Vec<String> {
        self.heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    fn available_sensors(&self) -> Vec<String> {
        self.sensors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }
}

impl PrinterObject for PrinterHeaters {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "available_heaters": self.available_heaters(),
            "available_sensors": self.available_sensors(),
            "available_monitors": self
                .monitors
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone(),
        })
    }
}

impl std::fmt::Debug for PrinterHeaters {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterHeaters")
            .field("sensors", &self.available_sensors())
            .finish_non_exhaustive()
    }
}

/// The single `heaters` object; the first caller creates it.
///
/// This is where the built-in sensor modules are loaded, since upstream reaches
/// them through `temperature_sensors.cfg` when the registry is first used.
///
/// # Errors
/// Registering the object, or bringing in a sensor module.
pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrinterHeaters>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<PrinterHeaters>(HEATERS_OBJECT) {
        return Ok(existing);
    }
    let heaters = Arc::new(PrinterHeaters::new());
    printer.add_object(
        HEATERS_OBJECT,
        Arc::clone(&heaters) as Arc<dyn PrinterObject>,
    )?;
    ds18b20::ensure(&heaters)?;
    adc_temperature::ensure(&heaters)?;
    temperature_mcu::ensure(&heaters)?;
    spi_temperature::ensure(&heaters)?;
    temperature_combined::ensure(&heaters)?;
    Ok(heaters)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
    };
    use crate::core::klippy::reactor::ManualReactor;

    /// A sensor that accepts everything, for `setup_heater` tests.
    #[derive(Debug)]
    struct FakeSensor;

    impl Sensor for FakeSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}
        fn setup_callback(&self, _callback: SensorCallback) {}
    }

    /// A PWM that accepts everything.
    #[derive(Debug)]
    struct FakePwm;

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_cycle_time(&self, _cycle_time: f64, _hardware: bool) {}
        fn setup_start_value(&self, _start: f64, _shutdown: f64) {}
        fn set_pwm(&self, _clock: u32, _value: f64) -> Result<(), McuError> {
            Ok(())
        }
        fn update_pwm(&self, _value: f64) -> Result<(), McuError> {
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    /// A chip that only exists so a pin description resolves.
    #[derive(Debug)]
    struct NoopChip;

    impl PinChip for NoopChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            Ok(Arc::new(FakePwm))
        }
    }

    fn section(sensor_type: &str) -> ConfigSection {
        let mut section = ConfigSection::new("temperature_sensor", Some("probe"));
        section.parameters.insert(
            "sensor_type".to_string(),
            ConfigValue::Single(sensor_type.to_string()),
        );
        section
    }

    /// A printer with `gcode` and `pins` over a no-op chip.
    fn ready_printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        pins.register_chip("mcu", Arc::new(NoopChip)).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer
    }

    #[test]
    fn test_a_factory_builds_the_sensor_it_registered() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let heaters = ensure(&printer).unwrap();

        assert!(heaters
            .setup_sensor(&ConfigWrapper::untracked(&section("made_up")), &printer)
            .unwrap_err()
            .to_string()
            .contains("Unknown temperature sensor 'made_up'"));
        // DS18B20 is brought in by `ensure`, as upstream's config does.
        let sensor = heaters
            .setup_sensor(&ConfigWrapper::untracked(&section("DS18B20")), &printer)
            .unwrap_err();
        assert!(sensor.to_string().contains("serial_no"), "{sensor}");
    }

    #[test]
    fn test_the_status_lists_what_was_registered() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let heaters = ensure(&printer).unwrap();
        heaters
            .register_sensor(&ConfigWrapper::untracked(&section("Fake")))
            .unwrap();

        assert_eq!(
            heaters.get_status(0.0)["available_sensors"],
            json!(["temperature_sensor probe"])
        );
    }

    #[test]
    fn test_setup_heater_claims_its_options_and_registers() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );

        let mut heater_section = ConfigSection::new("extruder", None);
        for (key, value) in [
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("control", "pid"),
            ("pid_kp", "1"),
            ("pid_ki", "0.1"),
            ("pid_kd", "10"),
            ("min_extrude_temp", "0"),
        ] {
            heater_section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }

        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&heater_section), &printer, None)
            .unwrap();

        assert!(heater.can_extrude());
        heater.set_temp(200.0).unwrap();
        assert_eq!(heater.get_status()["target"], 200.0);
        assert!(heater.set_temp(300.0).is_err());
        assert_eq!(
            heaters.get_status(0.0)["available_heaters"],
            json!(["extruder"])
        );
    }

    /// `heaters.py:38-39`: file-output mode (upstream's `-o`, which
    /// `test_klippy.py` runs every case with) lets a heater extrude from a cold
    /// start — such a run never answers its temperature queries, so no reading
    /// would ever turn the flag on again.
    #[test]
    fn test_file_output_may_extrude_without_a_reading() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let mut args = crate::core::klippy::api::StartArgs::collect("/tmp/printer.cfg", None);
        args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(args));

        // No `min_extrude_temp`: the 170 default is far above anything this
        // run will ever read, and it reads nothing at all.
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("control", "pid"),
            ("pid_kp", "1"),
            ("pid_ki", "0.1"),
            ("pid_kd", "10"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();

        assert!(heater.can_extrude());
    }

    /// A `[extruder]`-style heater section with `options`.
    fn heater_section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("extruder", None);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    #[test]
    fn test_the_bang_bang_control_toggles_the_output() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "50"),
            ("control", "watermark"),
            ("max_delta", "2"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();

        // Below `min_extrude_temp`: no extrusion.
        heater.temperature_callback(0.0, 25.0);
        assert!(!heater.can_extrude());

        heater.set_temp(100.0).unwrap();
        // 90 is below target - max_delta, so the heater turns on.
        heater.temperature_callback(1.0, 90.0);
        assert_eq!(heater.get_status()["power"], 1.0);
        assert!(heater.can_extrude());
        // 103 is above target + max_delta, so it turns off.
        heater.temperature_callback(2.0, 103.0);
        assert_eq!(heater.get_status()["power"], 0.0);
    }

    #[test]
    fn test_the_pid_control_output_is_bounded() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "0"),
            ("control", "pid"),
            ("pid_kp", "64"),
            ("pid_ki", "1.4"),
            ("pid_kd", "128"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();
        heater.set_temp(200.0).unwrap();

        heater.temperature_callback(0.0, 25.0);
        let power = heater.get_status()["power"].as_f64().unwrap();
        assert!((0.0..=1.0).contains(&power), "{power}");
        // Far below target, the PID output should be saturated high.
        assert!(power > 0.5, "{power}");
    }
}
