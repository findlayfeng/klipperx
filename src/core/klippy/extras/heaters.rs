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
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::adc_temperature;
use crate::core::klippy::extras::ds18b20;
use crate::core::klippy::extras::spi_temperature;
use crate::core::klippy::extras::temperature_combined;
use crate::core::klippy::extras::temperature_mcu;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name other modules look the registry up by.
pub const HEATERS_OBJECT: &str = "heaters";

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

/// One configured heater (an extruder hotend, a bed, a generic heater).
///
/// Upstream's `Heater` (`klippy/extras/heaters.py:14-160`). This is the **stub**
/// C1b needs: it reads and claims every heater option, sets up the sensor, and
/// reports `can_extrude = true` so motion is not blocked. The control loop —
/// PID/bang-bang, the PWM output and the periodic timer — is H1; until then a
/// `M104`/`SET_HEATER_TEMPERATURE` records the target and no heat is applied.
pub struct Heater {
    /// The section's short name (`extruder`, `heater_bed`).
    name: String,
    /// The sensor built from the heater's section.
    sensor: Arc<dyn Sensor>,
    min_temp: f64,
    max_temp: f64,
    /// Whether extrusion is allowed. Stubbed to `true` (see the type docs).
    can_extrude: bool,
    target_temp: Mutex<f64>,
    last_temp: Mutex<f64>,
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
        self.can_extrude
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
        *self.target_temp.lock().unwrap_or_else(|p| p.into_inner()) = degrees;
        Ok(())
    }

    /// `Heater.get_status`.
    pub fn get_status(&self) -> Value {
        json!({
            "temperature": *self.last_temp.lock().unwrap_or_else(|p| p.into_inner()),
            "target": *self.target_temp.lock().unwrap_or_else(|p| p.into_inner()),
            // No control loop yet, so no PWM output is ever applied.
            "power": 0.0,
        })
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
    /// Reads and claims the heater options and builds the sensor, but does not
    /// start a control loop (H1). `can_extrude` is stubbed to `true` so an
    /// `[extruder]` can move before H1 lands; the PWM output is reserved but
    /// not driven.
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
        let _ = config.get_float_bounded(
            "min_extrude_temp",
            Some(170.0),
            Some(min_temp),
            Some(max_temp),
            None,
            None,
        )?;
        let _ =
            config.get_float_bounded("max_power", Some(1.0), None, Some(1.0), Some(0.0), None)?;
        let _ = config.get_float_bounded("smooth_time", Some(1.0), None, None, Some(0.0), None)?;
        let _ =
            config.get_float_bounded("pwm_cycle_time", Some(0.100), None, None, Some(0.0), None)?;

        // Reserve the heater pin. Upstream builds a PWM here; H1 switches to
        // `setup_pwm` and drives it, so the pin is only claimed for now.
        let heater_pin = config.get("heater_pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        pins.lookup_pin(&heater_pin, true, false, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        match config
            .get_choice("control", &["watermark", "pid"], None)?
            .as_str()
        {
            "watermark" => {
                let _ = config.get_float_bounded(
                    "max_delta",
                    Some(2.0),
                    None,
                    None,
                    Some(0.0),
                    None,
                )?;
            }
            _ => {
                let _ = config.get_float("pid_Kp", None)?;
                let _ = config.get_float("pid_Ki", None)?;
                let _ = config.get_float("pid_Kd", None)?;
            }
        }

        sensor.setup_minmax(min_temp, max_temp);
        self.register_sensor(config)?;
        let _ = gcode_id; // TODO H1: the M105 g-code id table

        let heater = Arc::new(Heater {
            name: short_name.clone(),
            sensor,
            min_temp,
            max_temp,
            // No control loop: allow extrusion until H1 decides from the
            // reading. Upstream allows it when `min_extrude_temp <= 0` or in
            // file-output mode; the fake-MCU harness is the latter in spirit.
            can_extrude: true,
            target_temp: Mutex::new(0.0),
            last_temp: Mutex::new(0.0),
        });
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
        let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
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
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PINS_OBJECT,
    };
    use crate::core::klippy::reactor::ManualReactor;

    /// A sensor that accepts everything, for `setup_heater` tests.
    #[derive(Debug)]
    struct FakeSensor;

    impl Sensor for FakeSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}
        fn setup_callback(&self, _callback: SensorCallback) {}
    }

    /// A chip that only exists so a pin description resolves.
    #[derive(Debug)]
    struct NoopChip;

    impl PinChip for NoopChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
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
            ("pid_Kp", "1"),
            ("pid_Ki", "0.1"),
            ("pid_Kd", "10"),
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
}
