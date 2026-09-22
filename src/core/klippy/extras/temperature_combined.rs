//! `temperature_combined` — a sensor that combines several others.
//!
//! Upstream's `temperature_combined.py`: a `[temperature_sensor]` whose
//! `sensor_type` is `temperature_combined` names other sensors in `sensor_list`
//! and reports their `min`, `max` or `mean`. A `maximum_deviation` guards
//! against the sensors disagreeing — a disagreement shuts the printer down.
//!
//! The combined sensor is itself a printer object
//! (`temperature_combined <name>`), so a later combined sensor may reference it.
//! It starts a reactor timer once connected and polls the referenced sensors'
//! `get_status` each period.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{PrinterHeaters, Sensor, SensorCallback};
use crate::core::klippy::mcu::McuObject;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

/// Upstream's `REPORT_TIME`.
const REPORT_TIME: f64 = 0.300;
/// How long after connect before the first poll (`_handle_ready` waits 1 s for
/// the underlying sensors to have a reading).
const START_DELAY: f64 = 1.0;

/// How the sensor values are combined (`combination_method`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Combine {
    Min,
    Max,
    Mean,
}

impl Combine {
    fn apply(self, values: &[f64]) -> f64 {
        match self {
            Combine::Min => values.iter().copied().fold(f64::INFINITY, f64::min),
            Combine::Max => values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            Combine::Mean => values.iter().sum::<f64>() / values.len() as f64,
        }
    }
}

/// One `temperature_combined` sensor.
pub struct TemperatureCombined {
    /// The section's sub-name (`[temperature_sensor <name>]`).
    name: String,
    /// The object names to read.
    sensor_names: Vec<String>,
    /// The largest spread allowed between the sensors.
    max_deviation: f64,
    method: Combine,
    /// The referenced sensors, resolved in `connect`.
    sensors: Mutex<Vec<Arc<dyn PrinterObject>>>,
    /// The most recent combined reading.
    last_temp: Mutex<f64>,
    min_temp: Mutex<f64>,
    max_temp: Mutex<f64>,
    callback: Mutex<Option<SensorCallback>>,
    printer: Weak<Printer>,
    mcu: Weak<McuObject>,
    timer: Mutex<Option<TimerHandle>>,
    /// A weak handle to this object, for the reactor timer.
    self_ref: Weak<TemperatureCombined>,
}

impl TemperatureCombined {
    /// Read the section and register the object.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let name = config.section().sub.clone().unwrap_or_default();
        let sensor_names = config.get_list("sensor_list", ',').ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{}' must set 'sensor_list'",
                config.identifier()
            ))
        })?;
        let max_deviation =
            config.get_float_bounded("maximum_deviation", None, None, None, Some(0.0), None)?;
        let method = match config
            .get_choice("combination_method", &["min", "max", "mean"], None)?
            .as_str()
        {
            "min" => Combine::Min,
            "max" => Combine::Max,
            "mean" => Combine::Mean,
            _ => unreachable!("get_choice validated the value"),
        };

        // The MCU converted eventtime into print time for the callback.
        let mcu = printer
            .lookup_object_as::<McuObject>("mcu")
            .ok_or_else(|| ConfigError::new("temperature_combined needs an 'mcu' object"))?;

        let object = Arc::new_cyclic(|weak| Self {
            self_ref: weak.clone(),
            name: name.clone(),
            sensor_names,
            max_deviation,
            method,
            sensors: Mutex::new(Vec::new()),
            last_temp: Mutex::new(0.0),
            min_temp: Mutex::new(0.0),
            max_temp: Mutex::new(0.0),
            callback: Mutex::new(None),
            printer: Arc::downgrade(printer),
            mcu: Arc::downgrade(&mcu),
            timer: Mutex::new(None),
        });

        printer
            .add_object(
                &format!("temperature_combined {name}"),
                Arc::clone(&object) as Arc<dyn PrinterObject>,
            )
            .map_err(|err| ConfigError::new(err.to_string()))?;
        Ok(object)
    }

    /// Resolve `sensor_list` against the object registry.
    ///
    /// Upstream does this in a `klippy:connect` handler; here it is the object's
    /// own `connect`, so a bad reference is reported the same way as any other
    /// object that cannot come up.
    fn resolve_sensors(&self) -> Result<(), ConfigError> {
        let Some(printer) = self.printer.upgrade() else {
            return Ok(());
        };
        let mut resolved = Vec::with_capacity(self.sensor_names.len());
        for name in &self.sensor_names {
            let sensor = printer
                .lookup_object(name)
                .ok_or_else(|| ConfigError::new(format!("'{}' does not have a status.", name)))?;
            let status = sensor.get_status(0.0);
            let has_temperature = status
                .get("temperature")
                .map(|value| !value.is_null())
                .unwrap_or(false);
            if !has_temperature {
                if status.get("temperature").is_some() {
                    return Err(ConfigError::new(format!(
                        "Temperature monitor '{name}' is not supported"
                    )));
                }
                return Err(ConfigError::new(format!(
                    "'{name}' does not report a temperature."
                )));
            }
            resolved.push(sensor);
        }
        *self.sensors.lock().unwrap_or_else(|p| p.into_inner()) = resolved;
        Ok(())
    }

    /// Start the periodic poll.
    fn start_timer(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "temperature_combined",
            Box::new(move |eventtime| {
                let Some(sensor) = weak.upgrade() else {
                    return None;
                };
                sensor.update(eventtime)
            }),
            reactor.monotonic() + START_DELAY,
        );
        *self.timer.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    }

    /// One poll: read the sensors, check the spread and range, report.
    fn update(&self, eventtime: f64) -> Option<f64> {
        let values: Vec<f64> = self
            .sensors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .filter_map(|sensor| sensor.get_status(eventtime).get("temperature")?.as_f64())
            .collect();
        if values.is_empty() {
            return Some(eventtime + REPORT_TIME);
        }

        let Some(printer) = self.printer.upgrade() else {
            return None;
        };
        let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min = values.iter().copied().fold(f64::INFINITY, f64::min);
        if max - min > self.max_deviation {
            printer.invoke_shutdown(&format!(
                "COMBINED SENSOR maximum deviation exceeded limit of {:.1}, \
                 max sensor value {:.1}, min sensor value {:.1}.",
                self.max_deviation, max, min
            ));
            return Some(eventtime + REPORT_TIME);
        }

        let temp = self.method.apply(&values);
        // Upstream keeps a zero out of `last_temp`.
        if temp != 0.0 {
            *self.last_temp.lock().unwrap_or_else(|p| p.into_inner()) = temp;
        }
        let last = *self.last_temp.lock().unwrap_or_else(|p| p.into_inner());
        let min_temp = *self.min_temp.lock().unwrap_or_else(|p| p.into_inner());
        let max_temp = *self.max_temp.lock().unwrap_or_else(|p| p.into_inner());
        if last < min_temp {
            printer.invoke_shutdown(&format!(
                "COMBINED SENSOR temperature {last:.1} below minimum temperature of {min_temp:.1}."
            ));
        }
        if last > max_temp {
            printer.invoke_shutdown(&format!(
                "COMBINED SENSOR temperature {last:.1} above maximum temperature of {max_temp:.1}."
            ));
        }

        let print_time = self
            .mcu
            .upgrade()
            .and_then(|mcu| mcu.estimated_print_time(eventtime))
            .unwrap_or(eventtime);
        if let Some(cb) = self
            .callback
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_mut()
        {
            cb(print_time, last);
        }
        Some(eventtime + REPORT_TIME)
    }
}

/// Upstream's `round(value, 2)`.
fn round2(value: f64) -> f64 {
    (value * 100.0).round() / 100.0
}

impl Sensor for TemperatureCombined {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        *self.min_temp.lock().unwrap_or_else(|p| p.into_inner()) = min_temp;
        *self.max_temp.lock().unwrap_or_else(|p| p.into_inner()) = max_temp;
    }

    fn setup_callback(&self, callback: SensorCallback) {
        *self.callback.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
    }
}

impl PrinterObject for TemperatureCombined {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "temperature": round2(*self.last_temp.lock().unwrap_or_else(|p| p.into_inner())) })
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            self.resolve_sensors()
                .map_err(crate::core::klippy::error::KlippyError::Config)?;
            self.start_timer();
            Ok(())
        })
    }
}

impl std::fmt::Debug for TemperatureCombined {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TemperatureCombined")
            .field("name", &self.name)
            .finish()
    }
}

impl Drop for TemperatureCombined {
    fn drop(&mut self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

/// Register the `temperature_combined` sensor factory.
pub fn ensure(heaters: &Arc<PrinterHeaters>) -> Result<(), ConfigError> {
    heaters.add_sensor_factory(
        "temperature_combined",
        Arc::new(|config: &ConfigWrapper, printer: &Arc<Printer>| {
            Ok(TemperatureCombined::new(config, printer)? as Arc<dyn Sensor>)
        }),
    );
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_combination_methods() {
        let values = [10.0, 20.0, 30.0];
        assert_eq!(Combine::Min.apply(&values), 10.0);
        assert_eq!(Combine::Max.apply(&values), 30.0);
        assert_eq!(Combine::Mean.apply(&values), 20.0);
    }

    #[test]
    fn test_rounding() {
        assert_eq!(round2(1.234), 1.23);
        assert_eq!(round2(1.235), 1.24);
    }
}
