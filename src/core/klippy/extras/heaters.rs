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
use crate::core::klippy::extras::ds18b20;
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

/// The `heaters` object: the sensor factory table and what is registered.
pub struct PrinterHeaters {
    factories: Mutex<BTreeMap<String, SensorFactory>>,
    sensors: Mutex<Vec<String>>,
    monitors: Mutex<Vec<String>>,
}

impl PrinterHeaters {
    fn new() -> Self {
        Self {
            factories: Mutex::new(BTreeMap::new()),
            sensors: Mutex::new(Vec::new()),
            monitors: Mutex::new(Vec::new()),
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
            "available_heaters": Vec::<String>::new(),
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
    Ok(heaters)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    fn section(sensor_type: &str) -> ConfigSection {
        let mut section = ConfigSection::new("temperature_sensor", Some("probe"));
        section.parameters.insert(
            "sensor_type".to_string(),
            ConfigValue::Single(sensor_type.to_string()),
        );
        section
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
}
