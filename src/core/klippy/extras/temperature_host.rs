//! `[temperature_host]` — the host's own thermal-zone temperature sensor.
//!
//! Upstream's `temperature_host.py`: the bare `[temperature_host]` section only
//! registers the `temperature_host` sensor factory with `heaters` (upstream
//! reaches that section through `temperature_sensors.cfg`), and each
//! `[temperature_sensor …]` — or heater — with that `sensor_type` builds one of
//! these. The sensor opens `sensor_path` once, then on a 1-second reactor timer
//! reads the millidegree value the kernel reports, checks it against the
//! section's `min_temp`/`max_temp` (an out-of-range reading shuts the printer
//! down) and delivers it through the sensor callback.
//!
//! The sensor is itself a printer object (`temperature_host <name>`, upstream's
//! `add_object("temperature_host " + name, self)`), so a client can read its
//! `temperature` directly.

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{self, Sensor, SensorCallback};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuObject;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

/// Seconds between readings, upstream's `HOST_REPORT_TIME`.
const REPORT_TIME: f64 = 1.0;
/// The kernel file upstream reads when `sensor_path` is unset
/// (`RPI_PROC_TEMP_FILE`, the Raspberry Pi thermal zone).
const DEFAULT_SENSOR_PATH: &str = "/sys/class/thermal/thermal_zone0/temp";

/// One `temperature_host` sensor.
pub struct TemperatureHost {
    /// The section's last word, upstream's `config.get_name().split()[-1]` —
    /// the object is registered as `temperature_host <name>`.
    name: String,
    /// The file the readings come from (`sensor_path`).
    path: String,
    /// The handle opened at construction; `None` only in a `debugoutput` run,
    /// where upstream opens nothing and starts no timer either.
    file: Mutex<Option<File>>,
    /// The last reading, in degrees Celsius.
    temp: Mutex<f64>,
    min_temp: Mutex<f64>,
    max_temp: Mutex<f64>,
    callback: Mutex<Option<SensorCallback>>,
    timer: Mutex<Option<TimerHandle>>,
    printer: Weak<Printer>,
    /// A weak handle to this object, for the reactor timer.
    self_ref: Weak<TemperatureHost>,
}

impl TemperatureHost {
    /// Read the section, register the object and open the temperature file.
    ///
    /// # Errors
    /// An option is missing or the `sensor_path` file cannot be opened — the
    /// latter is upstream's `Unable to open temperature file '<path>'`
    /// (`temperature_host.py:25-27`).
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let name = last_word(&config.identifier());
        let path = sensor_path(config)?;

        let object = Arc::new_cyclic(|weak| Self {
            name,
            path,
            file: Mutex::new(None),
            temp: Mutex::new(0.0),
            min_temp: Mutex::new(0.0),
            max_temp: Mutex::new(0.0),
            callback: Mutex::new(None),
            timer: Mutex::new(None),
            printer: Arc::downgrade(printer),
            self_ref: weak.clone(),
        });

        // Upstream registers the object before it opens anything
        // (`temperature_host.py:21`), under `temperature_host <name>`.
        printer
            .add_object(
                &format!("temperature_host {}", object.name),
                Arc::clone(&object) as Arc<dyn PrinterObject>,
            )
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // Upstream returns early in a `--debugoutput` run: no file, no timer
        // (`temperature_host.py:22-23`). `is_fileoutput` is that mode here.
        if !printer.is_fileoutput() {
            let file = File::open(&object.path).map_err(|_| {
                ConfigError::new(format!("Unable to open temperature file '{}'", object.path))
            })?;
            *object.file.lock().unwrap_or_else(|p| p.into_inner()) = Some(file);
        }
        Ok(object)
    }

    /// Start the periodic poll, upstream's `klippy:connect` handler arming the
    /// timer with `NOW`.
    fn start_timer(&self) {
        // A `debugoutput` run never opened the file and has no timer.
        if self
            .file
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none()
        {
            return;
        }
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "temperature_host",
            Box::new(move |eventtime| weak.upgrade().and_then(|sensor| sensor.sample(eventtime))),
            reactor.monotonic(),
        );
        *self.timer.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    }

    /// One poll: read the file, bound-check, report — upstream's
    /// `_sample_pi_temperature`.
    ///
    /// `eventtime` is the reactor's clock, which is upstream's
    /// `measured_time = self.reactor.monotonic()`. A read failure logs, keeps
    /// `temperature` at 0.0 and returns `None` — upstream's `NEVER`, which
    /// retires the timer.
    fn sample(&self, eventtime: f64) -> Option<f64> {
        let temp = match self.read_sample() {
            Ok(temp) => temp,
            Err(err) => {
                warn!("temperature_host: Error reading data: {err}");
                *self.temp.lock().unwrap_or_else(|p| p.into_inner()) = 0.0;
                return None;
            }
        };
        *self.temp.lock().unwrap_or_else(|p| p.into_inner()) = temp;

        let printer = self.printer.upgrade()?;
        let min_temp = *self.min_temp.lock().unwrap_or_else(|p| p.into_inner());
        let max_temp = *self.max_temp.lock().unwrap_or_else(|p| p.into_inner());
        if temp < min_temp {
            printer.invoke_shutdown(&format!(
                "HOST temperature {temp:.1} below minimum temperature of {min_temp:.1}."
            ));
        }
        if temp > max_temp {
            printer.invoke_shutdown(&format!(
                "HOST temperature {temp:.1} above maximum temperature of {max_temp:.1}."
            ));
        }

        // Upstream dates the reading with `mcu.estimated_print_time`; a machine
        // whose MCU clock is not mapped yet (or a host-only test) falls back to
        // the event time, the way `temperature_combined` does.
        let read_time = printer
            .lookup_object_as::<McuObject>("mcu")
            .and_then(|mcu| mcu.estimated_print_time(eventtime))
            .unwrap_or(eventtime);
        if let Some(callback) = self
            .callback
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
        {
            callback(read_time, temp);
        }
        Some(eventtime + REPORT_TIME)
    }

    /// `seek(0)` + read + degrees — upstream's `_get_sample` and
    /// `float(raw_value) / 1000.0`, inside the one `try` that a failure bails
    /// out of (`temperature_host.py:60-69`).
    fn read_sample(&self) -> Result<f64, String> {
        let mut handle = self.file.lock().unwrap_or_else(|p| p.into_inner());
        let file = handle
            .as_mut()
            .ok_or_else(|| "the temperature file is not open".to_string())?;
        file.seek(SeekFrom::Start(0))
            .map_err(|err| err.to_string())?;
        let mut raw = String::new();
        file.read_to_string(&mut raw)
            .map_err(|err| err.to_string())?;
        let millidegrees = raw.trim().parse::<f64>().map_err(|err| err.to_string())?;
        Ok(millidegrees / 1000.0)
    }
}

impl Sensor for TemperatureHost {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        *self.min_temp.lock().unwrap_or_else(|p| p.into_inner()) = min_temp;
        *self.max_temp.lock().unwrap_or_else(|p| p.into_inner()) = max_temp;
    }

    fn setup_callback(&self, callback: SensorCallback) {
        *self.callback.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
    }
}

impl PrinterObject for TemperatureHost {
    /// Upstream's `get_status`: the last reading only.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "temperature": round2(*self.temp.lock().unwrap_or_else(|p| p.into_inner())) })
    }

    /// Start polling once the machine is up (`klippy:connect`).
    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            self.start_timer();
            Ok::<(), crate::core::klippy::error::KlippyError>(())
        })
    }
}

impl std::fmt::Debug for TemperatureHost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TemperatureHost")
            .field("name", &self.name)
            .field("path", &self.path)
            .finish_non_exhaustive()
    }
}

impl Drop for TemperatureHost {
    fn drop(&mut self) {
        // The file handle closes here, which is upstream's `handle_disconnect`.
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

/// The file to read, upstream's `config.get("sensor_path", RPI_PROC_TEMP_FILE)`
/// — an option of the section that names the `temperature_host` `sensor_type`,
/// not of the bare `[temperature_host]`.
fn sensor_path(config: &ConfigWrapper) -> Result<String, ConfigError> {
    config.get("sensor_path", Some(DEFAULT_SENSOR_PATH))
}

/// Upstream's `config.get_name().split()[-1]`: `temperature_sensor host` →
/// `host`, a bare `extruder` → `extruder`.
fn last_word(identifier: &str) -> String {
    identifier
        .split_whitespace()
        .last()
        .unwrap_or(identifier)
        .to_string()
}

/// Upstream's `round(value, 2)`.
fn round2(value: f64) -> f64 {
    (value * 100.0).round() / 100.0
}

/// Bare `[temperature_host]`'s printer object.
///
/// The section's whole job is registering the factory — upstream's
/// `load_config` returns `None`, so upstream has no queryable object under
/// `temperature_host` either. This placeholder claims the section for the
/// loader and stays out of `objects/list`.
struct SensorSection;

impl PrinterObject for SensorSection {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// Upstream's `load_config`: bring in `heaters` and register the
/// `temperature_host` factory (`temperature_host.py:86-90`).
///
/// # Errors
/// Bringing in the `heaters` registry.
pub fn load_config(
    _config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let heaters = heaters::ensure(printer)?;
    heaters.add_sensor_factory(
        "temperature_host",
        Arc::new(|config: &ConfigWrapper, printer: &Arc<Printer>| {
            Ok(TemperatureHost::new(config, printer)? as Arc<dyn Sensor>)
        }),
    );
    Ok(Arc::new(SensorSection))
}

// The bare `[temperature_host]` is upstream's "load the module" section (it
// ships in `temperature_sensors.cfg`). `phase = early` is load-bearing: the
// factory is consumed by generic sections of every kind — the prefix
// `[temperature_sensor …]` **and** the main `[extruder]`/`[heater_bed]`/
// `[temperature_fan]` — and within a phase the main sections load first in
// `order`, so at the default phase an order-20 consumer would come before an
// order-30 definition. Loading before the generic walk makes the factory
// available to every consumer, the same reason `[thermistor …]` sits early.
section!(
    "temperature_host",
    order = 30,
    phase = early,
    load = load_config
);

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// A `[temperature_sensor host]` that names this sensor and a path.
    fn consumer_section(path: &str) -> ConfigSection {
        let mut section = ConfigSection::new("temperature_sensor", Some("host"));
        section.parameters.insert(
            "sensor_type".to_string(),
            ConfigValue::Single("temperature_host".to_string()),
        );
        section.parameters.insert(
            "sensor_path".to_string(),
            ConfigValue::Single(path.to_string()),
        );
        section
    }

    /// A file under the system temp directory, with `contents` in it.
    fn temp_file(name: &str, contents: &str) -> String {
        let path =
            std::env::temp_dir().join(format!("temperature_host_{}_{}", std::process::id(), name));
        std::fs::write(&path, contents).expect("the temp file is written");
        path.to_string_lossy().into_owned()
    }

    /// Remove a file a test created, best effort.
    fn remove(path: &str) {
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn test_sensor_path_defaults_to_the_raspberry_pi_thermal_file() {
        let section = ConfigSection::new("temperature_sensor", Some("host"));
        assert_eq!(
            sensor_path(&ConfigWrapper::untracked(&section)).unwrap(),
            "/sys/class/thermal/thermal_zone0/temp"
        );

        let section = consumer_section("/sys/class/thermal/thermal_zone1/temp");
        assert_eq!(
            sensor_path(&ConfigWrapper::untracked(&section)).unwrap(),
            "/sys/class/thermal/thermal_zone1/temp"
        );
    }

    /// Upstream raises `config.error` when the file cannot be opened, word for
    /// word (`temperature_host.py:25-27`).
    #[test]
    fn test_an_unopenable_sensor_path_is_a_config_error() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let path = temp_file("unopenable", "");
        remove(&path);

        let section = consumer_section(&path);
        let error = TemperatureHost::new(&ConfigWrapper::untracked(&section), &printer)
            .expect_err("a missing file is an error");
        assert_eq!(
            error.to_string(),
            format!("Unable to open temperature file '{path}'")
        );
    }

    #[test]
    fn test_the_object_name_is_the_last_word_of_the_section_name() {
        assert_eq!(last_word("temperature_sensor host"), "host");
        assert_eq!(last_word("extruder"), "extruder");
        assert_eq!(last_word("temperature_sensor my sensor"), "sensor");
    }

    /// The timer reads millidegrees once a second and the status rounds them,
    /// with a callback reading per poll (`temperature_host.py:57-83`).
    #[test]
    fn test_the_timer_polls_the_file_and_reports_degrees() {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let path = temp_file("poll", "42500");

        let section = consumer_section(&path);
        let sensor = TemperatureHost::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the file exists");
        sensor.setup_minmax(0.0, 100.0);
        let readings = Arc::new(Mutex::new(Vec::new()));
        let recorded = Arc::clone(&readings);
        sensor.setup_callback(Box::new(move |_read_time, temp| {
            recorded
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(temp);
        }));

        sensor.start_timer();
        // Registered at NOW: the first poll runs immediately.
        reactor.run_due();
        assert_eq!(sensor.get_status(0.0), json!({ "temperature": 42.5 }));

        // The sensor keeps its handle and seeks back to 0, so rewriting the
        // file is the next reading — one poll per second.
        std::fs::write(&path, "43100").expect("the temp file is rewritten");
        assert_eq!(reactor.advance(1.0), 1);
        assert_eq!(sensor.get_status(1.0), json!({ "temperature": 43.1 }));

        let readings = readings.lock().unwrap_or_else(|p| p.into_inner());
        assert_eq!(*readings, vec![42.5, 43.1]);
        drop(readings);
        remove(&path);
    }

    /// A failed read logs, keeps 0.0 and retires the timer — upstream's
    /// `return self.reactor.NEVER` (`temperature_host.py:64-69`).
    #[test]
    fn test_a_failed_read_keeps_zero_and_retires_the_timer() {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let path = temp_file("read_error", "not-a-number");

        let section = consumer_section(&path);
        let sensor = TemperatureHost::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the file exists");
        let readings = Arc::new(Mutex::new(Vec::new()));
        let recorded = Arc::clone(&readings);
        sensor.setup_callback(Box::new(move |_read_time, temp| {
            recorded
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(temp);
        }));

        sensor.start_timer();
        reactor.run_due();
        assert_eq!(sensor.get_status(0.0), json!({ "temperature": 0.0 }));
        assert!(readings
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_empty());

        // The timer returned NEVER: nothing fires again.
        assert_eq!(reactor.advance(60.0), 0);
        remove(&path);
    }

    /// Both bounds shut the printer down with upstream's message
    /// (`temperature_host.py:71-77`).
    #[test]
    fn test_a_reading_out_of_bounds_shuts_the_printer_down() {
        for (contents, expected) in [
            (
                "5000",
                "HOST temperature 5.0 below minimum temperature of 10.0.",
            ),
            (
                "150000",
                "HOST temperature 150.0 above maximum temperature of 100.0.",
            ),
        ] {
            let reactor = Arc::new(ManualReactor::new());
            let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
            let path = temp_file("bounds", contents);

            let section = consumer_section(&path);
            let sensor = TemperatureHost::new(&ConfigWrapper::untracked(&section), &printer)
                .expect("the file exists");
            sensor.setup_minmax(10.0, 100.0);

            sensor.start_timer();
            reactor.run_due();

            let state = printer.get_state_message();
            assert_eq!(
                state.category,
                crate::core::klippy::printer::PrinterState::Shutdown
            );
            assert_eq!(state.message, expected);
            remove(&path);
        }
    }

    /// The bare section loads, registers the factory and claims its own
    /// section; the consumer's object answers under `temperature_host host`.
    #[test]
    fn test_the_bare_section_registers_the_factory_and_the_sensor_object() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let path = temp_file("load", "41000");
        let text = format!(
            "[temperature_host]\n\
             [temperature_sensor host]\n\
             sensor_type: temperature_host\n\
             sensor_path: {path}\n"
        );
        let (config, _) = Config::from_text(&text).expect("the config parses");
        printer.load_config(&config).expect("the config loads");

        let host = printer
            .lookup_object("temperature_host host")
            .expect("the sensor registered its object");
        assert_eq!(host.get_status(0.0), json!({ "temperature": 0.0 }));

        // The bare section's placeholder exists but stays out of `objects/list`.
        let bare = printer
            .lookup_object("temperature_host")
            .expect("the bare section is claimed");
        assert!(!bare.is_queryable());

        let consumer = printer
            .lookup_object("temperature_sensor host")
            .expect("the consumer loaded");
        assert_eq!(
            consumer.get_status(0.0),
            json!({
                "temperature": 0.0,
                "measured_min_temp": 99999999.0,
                "measured_max_temp": 0.0,
            })
        );
        remove(&path);
    }

    /// `sensor_path` belongs to the section that names the `sensor_type`, not
    /// to the bare `[temperature_host]` — upstream's `load_config` reads
    /// nothing, so `check_unused` rejects the option there.
    #[test]
    fn test_the_bare_section_accepts_no_options() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[temperature_host]\nsensor_path: /tmp/somewhere\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        let error = printer
            .load_config(&config)
            .expect_err("the bare section takes no options");
        assert!(
            error
                .to_string()
                .contains("Option 'sensor_path' is not valid in section 'temperature_host'"),
            "{error}"
        );
    }
}
