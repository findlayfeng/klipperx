//! `[temperature_sensor <name>]` — a named sensor a client can read.
//!
//! Upstream's `temperature_sensor.py`: it asks `heaters` to build the sensor the
//! section's `sensor_type` names, applies the section's `min_temp`/`max_temp`,
//! and reports the last reading. The sensor itself may be anything `heaters`
//! knows (`DS18B20` here, more as they arrive).

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!(
    "temperature_sensor",
    order = 30,
    prefix = load_config_prefix
);

/// The readings this sensor has seen, shared with the callback.
#[derive(Debug)]
struct Reading {
    last_temp: f64,
    measured_min: f64,
    measured_max: f64,
}

/// A `[temperature_sensor <name>]`.
pub struct TemperatureSensor {
    reading: Arc<Mutex<Reading>>,
}

impl TemperatureSensor {
    fn lock(&self) -> MutexGuard<'_, Reading> {
        self.reading.lock().unwrap_or_else(|p| p.into_inner())
    }
}

impl PrinterObject for TemperatureSensor {
    /// Upstream's `PrinterSensorGeneric.get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        let reading = self.lock();
        json!({
            "temperature": round2(reading.last_temp),
            "measured_min_temp": round2(reading.measured_min),
            "measured_max_temp": round2(reading.measured_max),
        })
    }
}

impl std::fmt::Debug for TemperatureSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TemperatureSensor").finish_non_exhaustive()
    }
}

/// Upstream's `load_config_prefix` for `[temperature_sensor <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let heaters = heaters::ensure(printer)?;
    let sensor = heaters.setup_sensor(config, printer)?;

    // `KELVIN_TO_CELSIUS` is upstream's default minimum.
    let min_temp =
        config.get_float_bounded("min_temp", Some(-273.15), Some(-273.15), None, None, None)?;
    let max_temp = config.get_float_bounded(
        "max_temp",
        Some(99_999_999.9),
        None,
        None,
        Some(min_temp),
        None,
    )?;
    sensor.setup_minmax(min_temp, max_temp);

    let reading = Arc::new(Mutex::new(Reading {
        last_temp: 0.0,
        measured_min: 99_999_999.0,
        measured_max: 0.0,
    }));
    let callback_reading = Arc::clone(&reading);
    // Upstream keeps a reading of 0 out of the measured range, so a sensor that
    // has never reported does not look like it measured 0 °C.
    sensor.setup_callback(Box::new(move |_read_time, temp| {
        let mut reading = callback_reading.lock().unwrap_or_else(|p| p.into_inner());
        reading.last_temp = temp;
        if temp != 0.0 {
            reading.measured_min = reading.measured_min.min(temp);
            reading.measured_max = reading.measured_max.max(temp);
        }
    }));

    heaters.register_sensor(config)?;
    Ok(Arc::new(TemperatureSensor { reading }))
}

/// Upstream's `round(value, 2)`.
fn round2(value: f64) -> f64 {
    (value * 100.0).round() / 100.0
}
