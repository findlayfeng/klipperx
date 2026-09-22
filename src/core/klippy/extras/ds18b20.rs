//! `[ds18b20]` — DS18B20 (1-wire) temperature sensors.
//!
//! Upstream's `ds18b20.py`: the module registers the `DS18B20` sensor factory
//! with `heaters` (its `[ds18b20]` section exists only to be loaded), and each
//! `[temperature_sensor …]` with that `sensor_type` builds one of these.
//!
//! The firmware polls the bus on a timer armed by `query_ds18b20` (an **init**
//! command) and pushes `ds18b20_result`; a result carries the firmware clock, so
//! the reading is dated by mapping it back to print time.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock, Weak};

use tracing::warn;

use crate::core::klippy::cmd::ds18b20::{ConfigDs18b20, Ds18b20Result, QueryDs18b20};
use crate::core::klippy::cmd::McuResponse;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{PrinterHeaters, Sensor, SensorCallback};
use crate::core::klippy::mcu::{query_slot, ConfigBuilder, Mcu, McuError, McuObject};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// Default time between readings, upstream's `DS18_REPORT_TIME`.
const REPORT_TIME: f64 = 3.0;
/// The firmware's conversion takes ~750 ms, so the period cannot be shorter
/// (`DS18_MIN_REPORT_TIME`).
const MIN_REPORT_TIME: f64 = 1.0;
/// Consecutive errors the firmware tolerates before shutting down.
const MAX_CONSECUTIVE_ERRORS: u8 = 4;

/// The sensor builder the `heaters` factory table holds.
///
/// Upstream's `ds18b20.load_config` only registers the factory; the `[ds18b20]`
/// section itself carries no options.
pub fn ensure(heaters: &Arc<PrinterHeaters>) -> Result<(), ConfigError> {
    heaters.add_sensor_factory(
        "DS18B20",
        Arc::new(|config: &ConfigWrapper, printer: &Arc<Printer>| {
            Ok(Arc::new(Ds18b20::new(config, printer)?) as Arc<dyn Sensor>)
        }),
    );
    Ok(())
}

/// One `[temperature_sensor]` backed by a DS18B20.
pub struct Ds18b20 {
    state: Arc<Ds18b20State>,
}

/// Everything the callbacks share: the build-time parameters and the readings.
struct Ds18b20State {
    /// The 1-wire serial number as hex text.
    serial: String,
    /// The object id the firmware allocated for this sensor.
    oid: u8,
    /// Seconds between readings.
    report_time: f64,
    /// The firmware ticks between readings, computed at build time.
    report_clock: Mutex<u32>,
    min_temp: Mutex<f64>,
    max_temp: Mutex<f64>,
    callback: Mutex<Option<SensorCallback>>,
    /// The MCU the sensor hangs off, for the clock↔print-time mapping.
    mcu: Weak<McuObject>,
}

impl Ds18b20 {
    /// Read the section and register this sensor's config callbacks.
    ///
    /// # Errors
    /// A missing or unparseable option, an unknown `sensor_mcu`, or an oid
    /// shortage on that MCU.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let serial_no = config.get("serial_no", None)?;
        let report_time = config.get_float_bounded(
            "ds18_report_time",
            Some(REPORT_TIME),
            Some(MIN_REPORT_TIME),
            None,
            None,
            None,
        )?;
        let mcu_name = config.get("sensor_mcu", None)?;
        let mcu = printer
            .lookup_object_as::<McuObject>(&mcu_name)
            .ok_or_else(|| ConfigError::new(format!("Unknown mcu '{mcu_name}'")))?;
        let oid = mcu
            .config()
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(Ds18b20State {
            serial: hex(serial_no.as_bytes()),
            oid,
            report_time,
            report_clock: Mutex::new(0),
            min_temp: Mutex::new(0.0),
            max_temp: Mutex::new(0.0),
            callback: Mutex::new(None),
            mcu: Arc::downgrade(&mcu),
        });

        let builder = mcu.config();
        let build_state = Arc::downgrade(&state);
        builder
            .register_config_callback(Box::new(move |builder, mcu| match build_state.upgrade() {
                Some(state) => state.build(builder, mcu),
                None => Ok(()),
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // The response can only be bound once the firmware has accepted the
        // configuration, and it must be bound on the connection that will
        // report — post-init runs on exactly that one.
        let bind_state = Arc::downgrade(&state);
        builder
            .register_post_init_callback(Box::new(move |mcu| {
                let Some(state) = bind_state.upgrade() else {
                    return;
                };
                // Arm the periodic query first, with a clock from this
                // connection; then bind so the first report has a handler.
                if let Err(err) = state.arm_query(mcu) {
                    warn!("MCU '{}': could not arm DS18B20 query: {err}", mcu.name());
                    return;
                }
                let registry = registry_for(mcu.name());
                if let Err(err) = registry.bind(mcu, state.oid, Arc::downgrade(&state)) {
                    warn!(
                        "MCU '{}': could not bind DS18B20 response: {err}",
                        mcu.name()
                    );
                }
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        Ok(Self { state })
    }
}

impl Sensor for Ds18b20 {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        *self
            .state
            .min_temp
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = min_temp;
        *self
            .state
            .max_temp
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = max_temp;
    }

    fn setup_callback(&self, callback: SensorCallback) {
        *self
            .state
            .callback
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(callback);
    }
}

impl PrinterObject for Ds18b20 {
    /// Upstream's `DS18B20.get_status`.
    fn get_status(&self, _eventtime: f64) -> serde_json::Value {
        serde_json::json!({ "temperature": 0.0 })
    }
}

impl std::fmt::Debug for Ds18b20 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ds18b20")
            .field("oid", &self.state.oid)
            .finish_non_exhaustive()
    }
}

impl Ds18b20State {
    /// The build-time half: add the configuration.
    ///
    /// Upstream's `DS18B20._build_config`. The query is **not** added here: it
    /// carries an absolute clock, and `built` is reused if the firmware is reset
    /// mid-connect, so arming it is left to the post-init callback
    /// ([`Ds18b20State::arm_query`]).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        builder.add_config_cmd(&ConfigDs18b20 {
            oid: self.oid,
            serial: self.serial.clone(),
            max_error_count: MAX_CONSECUTIVE_ERRORS,
        })?;

        let rest_ticks = mcu.seconds_to_clock(self.report_time)? as u32;
        *self.report_clock.lock().unwrap_or_else(|p| p.into_inner()) = rest_ticks;
        Ok(())
    }

    /// Send this sensor's `query_ds18b20`, with a clock read from `mcu` now.
    fn arm_query(&self, mcu: &Mcu) -> Result<(), McuError> {
        let clock = query_slot(mcu, self.oid)?;
        let rest_ticks = *self.report_clock.lock().unwrap_or_else(|p| p.into_inner());
        // The range is in millidegrees on the wire.
        let min_value = (self.lock_min_temp() * 1000.0) as i32;
        let max_value = (self.lock_max_temp() * 1000.0) as i32;
        mcu.send_msg(&QueryDs18b20 {
            oid: self.oid,
            clock,
            rest_ticks,
            min_value,
            max_value,
        })
    }

    fn lock_min_temp(&self) -> f64 {
        *self.min_temp.lock().unwrap_or_else(|p| p.into_inner())
    }

    fn lock_max_temp(&self) -> f64 {
        *self.max_temp.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Handle one `ds18b20_result`, dating the reading by the firmware clock.
    ///
    /// Upstream's `_handle_ds18b20_response`: a fault is logged and dropped, and
    /// the read time is `next_clock - report_clock` mapped to print time.
    fn handle(&self, values: &[ArgValue]) {
        let (next_clock, raw, fault) = match (values.get(1), values.get(2), values.get(3)) {
            (
                Some(ArgValue::UInt32(next_clock)),
                Some(ArgValue::Int32(raw)),
                Some(ArgValue::UInt32(fault)),
            ) => (*next_clock, *raw, *fault),
            _ => return,
        };
        if fault != 0 {
            return;
        }

        let Some(mcu) = self.mcu.upgrade() else {
            return;
        };
        let report_clock = i64::from(*self.report_clock.lock().unwrap_or_else(|p| p.into_inner()));
        let Some(next) = mcu.clock32_to_clock64(next_clock) else {
            return;
        };
        let Some(clock) = mcu.clock() else {
            return;
        };
        let read_time = clock.clock_to_print_time(next - report_clock);

        let callback = self.callback.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(callback) = callback.as_ref() {
            callback(read_time, f64::from(raw) / 1000.0);
        }
    }
}

/// Per-oid routing for `ds18b20_result`, one registry per MCU.
///
/// One callback can be bound per message name, so the sensors on an MCU share a
/// registry and one bound closure routes by oid — the same thing upstream's
/// `register_serial_response(..., oid=…)` does. Registries are keyed by MCU name
/// because a sensor is built before its MCU is connectable.
#[derive(Default)]
struct Ds18b20Registry {
    sensors: Mutex<HashMap<u8, Weak<Ds18b20State>>>,
}

impl Ds18b20Registry {
    fn bind(
        self: &Arc<Self>,
        mcu: &Mcu,
        oid: u8,
        state: Weak<Ds18b20State>,
    ) -> Result<(), McuError> {
        self.sensors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(oid, state);

        let registry = Arc::clone(self);
        mcu.bind_callback(Ds18b20Result::NAME, move |values| {
            // Every result starts with `oid=%c`; anything else is not ours.
            let oid = match values.first() {
                Some(ArgValue::UInt8(oid)) => *oid,
                _ => return,
            };
            let state = registry
                .sensors
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .get(&oid)
                .and_then(Weak::upgrade);
            if let Some(state) = state {
                state.handle(values);
            }
        })
    }
}

/// The registry for one MCU name, created on first use.
fn registry_for(name: &str) -> Arc<Ds18b20Registry> {
    static REGISTRIES: OnceLock<Mutex<HashMap<String, Arc<Ds18b20Registry>>>> = OnceLock::new();
    let registries = REGISTRIES.get_or_init(|| Mutex::new(HashMap::new()));
    let mut registries = registries.lock().unwrap_or_else(|p| p.into_inner());
    Arc::clone(
        registries
            .entry(name.to_string())
            .or_insert_with(|| Arc::new(Ds18b20Registry::default())),
    )
}

/// The serial number as upstream's `"%02x"` join.
fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_serial_number_is_lowercase_hex() {
        assert_eq!(hex(b"12345678"), "3132333435363738");
        assert_eq!(hex(&[0x00, 0xff, 0x0a]), "00ff0a");
    }
}
