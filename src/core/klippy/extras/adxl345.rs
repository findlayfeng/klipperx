//! `[adxl345]` — the ADXL345 accelerometer (upstream's
//! `klippy/extras/adxl345.py`).
//!
//! | piece | upstream |
//! |---|---|
//! | [`Adxl345`] — `cs_pin`, `axes_map`, `rate`; SPI via `MCU_SPI_from_config` | `ADXL345.__init__` |
//! | [`read_axes_map`] | `adxl345.read_axes_map` (reused by `[mpu9250]`) |
//! | [`AccelQueryHelper`] / [`AccelMeasurement`] | `adxl345.AccelQueryHelper` (reused by `[mpu9250]`) |
//! | [`Adxl345::start_internal_client`] | `ADXL345.start_internal_client` — the accelerometer interface `resonance_tester` looks for |
//! | `adxl345/dump_adxl345` mux endpoint (key `sensor`) | `batch_bulk.add_mux_endpoint(…)` |
//! | `config_adxl345` / `query_adxl345` | [`crate::core::klippy::cmd::adxl345`] |
//!
//! # Accelerometer interface
//!
//! `resonance_tester` measures with an accelerometer by name: it looks the
//! object up and uses `start_internal_client()`, exactly as upstream's
//! `hasattr(chip, 'start_internal_client')` check does. Here that is
//!
//! ```text
//! Adxl345::start_internal_client(&self) -> Arc<AccelQueryHelper>
//! ```
//!
//! and the returned client carries upstream's four methods — `handle_batch`,
//! `finish_measurements`, `has_valid_samples`, `get_samples` — plus
//! `write_to_file`. `[mpu9250]` shares [`read_axes_map`] and
//! [`AccelQueryHelper`] the same way upstream's `mpu9250.py` imports them from
//! `adxl345.py`.
//!
//! # Gap: the bulk sample path is not wired
//!
//! Everything above registers, but **no samples flow**: this host's shared
//! [`FixedFreqReader`](super::bulk_sensor::FixedFreqReader) is currently
//! ldc1612-specific — it decodes 4-byte `">I"` samples and queries the fixed
//! `query_status_ldc1612` — while the ADXL345 needs 5-byte `"BBBBB"` samples
//! and `query_adxl345_status` (and `[mpu9250]` needs 6-byte `">hhh"` and
//! `query_mpu9250_status`). Generalizing the reader is shared infrastructure and
//! is left to the parent to do once the accelerometer lanes have landed; until
//! then the batch helper's start/stop/batch callbacks are deliberately inert,
//! [`Adxl345::start_internal_client`]'s client stays empty, and the chip's
//! register setup / `_convert_samples` scale conversion are not implemented
//! here. What is real: section loading, option validation, the SPI resource,
//! the two config commands, the mux endpoint, and the client-side windowing.

use std::fs;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::adxl345::{ConfigAdxl345, QueryAdxl345};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, LifecycleCb, BATCH_INTERVAL,
};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuObject;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Loaded after `[board_pins]` (order 30) so an alias or a software bus can name
// the CS pin, alongside the other SPI devices.
section!(
    "adxl345",
    order = 40,
    load = load_config,
    prefix = load_config_prefix
);

/// The endpoint clients stream samples from (`adxl345/dump_adxl345`).
pub const DUMP_ENDPOINT: &str = "adxl345/dump_adxl345";

/// The mux key that selects a sensor instance.
pub const DUMP_KEY: &str = "sensor";

/// The columns the dump endpoint's start response advertises.
const DUMP_HEADER: [&str; 4] = ["time", "x_acceleration", "y_acceleration", "z_acceleration"];

/// Default sample rate (`rate`, upstream's `config.getint('rate', 3200)`).
const DEFAULT_RATE: i64 = 3200;

/// The ADXL345's fixed SPI mode (upstream's `MCU_SPI_from_config(config, 3)`).
const DEFAULT_SPI_MODE: u8 = 3;

/// Default SPI clock in Hz (`default_speed=5000000` upstream).
const DEFAULT_SPI_SPEED: u32 = 5_000_000;

/// The sample rates the chip supports, mapped to its `BW_RATE` code
/// (`QUERY_RATES`; the membership check is upstream's `if self.data_rate not in
/// QUERY_RATES`).
const QUERY_RATES: [u32; 8] = [25, 50, 100, 200, 400, 800, 1600, 3200];

/// Gravity in mm/s² (`FREEFALL_ACCEL`), folded into the axis scales below.
const FREEFALL_ACCEL: f64 = 9.80665 * 1000.;

/// Full-scale sensitivity on X/Y (`SCALE_XY`, 1/265 g/LSB at 3.3V).
const SCALE_XY: f64 = 0.003774 * FREEFALL_ACCEL;

/// Full-scale sensitivity on Z (`SCALE_Z`, 1/256 g/LSB at 3.3V).
const SCALE_Z: f64 = 0.003906 * FREEFALL_ACCEL;

/// The toolhead's registered name (upstream `lookup_object('toolhead')`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// Cap on the batch messages one client holds (`len(self.msgs) >= 10000`).
const MAX_MESSAGES: usize = 10000;

/// Read the `axes_map` option into `(source axis, signed scale)` triples
/// (upstream `adxl345.read_axes_map`).
///
/// `scale_{x,y,z}` are the chip's full-scale sensitivities; a leading `-` on an
/// axis flips the measured sign. A missing option defaults to `x,y,z`; an
/// unknown axis is the upstream error `Invalid axes_map parameter`.
///
/// # Errors
/// [`ConfigError`] when the option does not hold exactly three known axes.
pub fn read_axes_map(
    config: &ConfigWrapper,
    scale_x: f64,
    scale_y: f64,
    scale_z: f64,
) -> Result<Vec<(usize, f64)>, ConfigError> {
    let axis = |name: &str| -> Option<(usize, f64)> {
        Some(match name {
            "x" => (0, scale_x),
            "y" => (1, scale_y),
            "z" => (2, scale_z),
            "-x" => (0, -scale_x),
            "-y" => (1, -scale_y),
            "-z" => (2, -scale_z),
            _ => return None,
        })
    };
    let axes = config
        .get_list("axes_map", ',')
        .unwrap_or_else(|| vec!["x".to_string(), "y".to_string(), "z".to_string()]);
    if axes.len() != 3 {
        return Err(ConfigError::new(format!(
            "Option 'axes_map' in section '{}' must have 3 elements",
            config.identifier()
        )));
    }
    let mut map = Vec::with_capacity(3);
    for name in &axes {
        let entry = axis(name).ok_or_else(|| ConfigError::new("Invalid axes_map parameter"))?;
        map.push(entry);
    }
    Ok(map)
}

// ===========================================================================
// Measurement client
// ===========================================================================

/// One accelerometer reading: print time plus the three axis values, in mm/s²
/// (upstream's `Accel_Measurement` namedtuple).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelMeasurement {
    /// The sample's print time.
    pub time: f64,
    /// X acceleration.
    pub accel_x: f64,
    /// Y acceleration.
    pub accel_y: f64,
    /// Z acceleration.
    pub accel_z: f64,
}

/// A client of an accelerometer's sample stream (upstream's
/// `adxl345.AccelQueryHelper`).
///
/// `resonance_tester` opens one per measurement, feeds it batches through
/// [`handle_batch`](Self::handle_batch), then closes the window with
/// [`finish_measurements`](Self::finish_measurements) and reads the samples
/// back with [`get_samples`](Self::get_samples) /
/// [`has_valid_samples`](Self::has_valid_samples).
pub struct AccelQueryHelper {
    /// The machine, for the toolhead's clock (a `Weak`: the client never keeps
    /// the printer alive).
    printer: Weak<Printer>,
    /// Whether the measurement window was closed (`is_finished`).
    is_finished: AtomicBool,
    /// The window's start (`request_start_time`).
    request_start_time: f64,
    /// The window's end (`request_end_time`), set by
    /// [`finish_measurements`](Self::finish_measurements).
    request_end_time: Mutex<f64>,
    /// The batch messages received so far (`msgs`).
    msgs: Mutex<Vec<Value>>,
    /// The samples [`get_samples`](Self::get_samples) last produced.
    samples: Mutex<Vec<AccelMeasurement>>,
}

impl AccelQueryHelper {
    /// Start a measurement; the window opens at the toolhead's current move time
    /// (upstream `AccelQueryHelper.__init__`).
    ///
    /// `printer` is `None` only in tests, where the window starts at `0.0`.
    pub fn new(printer: Option<&Arc<Printer>>) -> Self {
        let request_start_time = printer.and_then(toolhead_last_move_time).unwrap_or(0.);
        Self {
            printer: printer.map(Arc::downgrade).unwrap_or_default(),
            is_finished: AtomicBool::new(false),
            request_start_time,
            request_end_time: Mutex::new(request_start_time),
            msgs: Mutex::new(Vec::new()),
            samples: Mutex::new(Vec::new()),
        }
    }

    /// Close the measurement window (`finish_measurements`).
    ///
    /// Upstream also calls `toolhead.wait_moves()`; this host has no synchronous
    /// `wait_moves` seam on [`ToolHeadObject`], and the bulk stream that would
    /// populate `msgs` is not wired yet (see the module gap), so only the
    /// window's end is recorded.
    pub fn finish_measurements(&self) {
        let end = self
            .printer
            .upgrade()
            .and_then(|printer| toolhead_last_move_time(&printer))
            .unwrap_or(self.request_start_time);
        *self
            .request_end_time
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = end;
        self.is_finished.store(true, Ordering::SeqCst);
    }

    /// Take one batch message; `false` unregisters the client
    /// (`AccelQueryHelper.handle_batch`).
    pub fn handle_batch(&self, msg: &Value) -> bool {
        if self.is_finished.load(Ordering::SeqCst) {
            return false;
        }
        let mut msgs = self.msgs.lock().unwrap_or_else(|p| p.into_inner());
        if msgs.len() >= MAX_MESSAGES {
            // Avoid filling up memory with too many samples (upstream).
            return false;
        }
        msgs.push(msg.clone());
        true
    }

    /// Whether any received batch intersects the measurement window
    /// (`has_valid_samples`).
    pub fn has_valid_samples(&self) -> bool {
        let end = self.request_end_time();
        let msgs = self.msgs.lock().unwrap_or_else(|p| p.into_inner());
        for msg in msgs.iter() {
            let Some((first, last)) = first_last_sample_time(msg) else {
                continue;
            };
            // The time intervals [first, last] and [start, end] have a
            // non-zero intersection (upstream's check).
            if first > end || last < self.request_start_time {
                continue;
            }
            return true;
        }
        false
    }

    /// The samples inside the measurement window (`get_samples`).
    pub fn get_samples(&self) -> Vec<AccelMeasurement> {
        let msgs = self.msgs.lock().unwrap_or_else(|p| p.into_inner());
        if msgs.is_empty() {
            return self
                .samples
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone();
        }
        let end = self.request_end_time();
        let mut out = Vec::new();
        for msg in msgs.iter() {
            let Some(data) = msg.get("data").and_then(Value::as_array) else {
                continue;
            };
            for row in data {
                let Some(row) = row.as_array() else {
                    continue;
                };
                if row.len() < 4 {
                    continue;
                }
                let (Some(time), Some(x), Some(y), Some(z)) = (
                    row[0].as_f64(),
                    row[1].as_f64(),
                    row[2].as_f64(),
                    row[3].as_f64(),
                ) else {
                    continue;
                };
                if time < self.request_start_time {
                    continue;
                }
                if time > end {
                    break;
                }
                out.push(AccelMeasurement {
                    time,
                    accel_x: x,
                    accel_y: y,
                    accel_z: z,
                });
            }
        }
        *self.samples.lock().unwrap_or_else(|p| p.into_inner()) = out.clone();
        out
    }

    /// Write the samples as CSV (`write_to_file`).
    ///
    /// Upstream forks a helper process and re-nices it; this host writes inline,
    /// which nothing here is latency-sensitive enough to avoid.
    ///
    /// # Errors
    /// Propagates the filesystem error.
    pub fn write_to_file(&self, filename: &str) -> std::io::Result<()> {
        let samples = self.get_samples();
        let mut text = String::from("#time,accel_x,accel_y,accel_z\n");
        for sample in &samples {
            text.push_str(&format!(
                "{:.6},{:.6},{:.6},{:.6}\n",
                sample.time, sample.accel_x, sample.accel_y, sample.accel_z
            ));
        }
        fs::write(filename, text)
    }

    /// The window's current end.
    fn request_end_time(&self) -> f64 {
        *self
            .request_end_time
            .lock()
            .unwrap_or_else(|p| p.into_inner())
    }
}

/// The first and last sample times of a batch message, or `None` when it
/// carries no samples.
fn first_last_sample_time(msg: &Value) -> Option<(f64, f64)> {
    let data = msg.get("data")?.as_array()?;
    let first = data.first()?.as_array()?.first()?.as_f64()?;
    let last = data.last()?.as_array()?.first()?.as_f64()?;
    Some((first, last))
}

/// The toolhead's current move time, or `None` before `[printer]` is up.
fn toolhead_last_move_time(printer: &Arc<Printer>) -> Option<f64> {
    printer
        .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
        .map(|toolhead| toolhead.get_last_move_time())
}

// ===========================================================================
// The sensor
// ===========================================================================

/// One configured ADXL345 (an `[adxl345]` or `[adxl345 <name>]` section).
pub struct Adxl345 {
    state: Arc<Adxl345State>,
}

impl Adxl345 {
    /// Read the section and wire the chip up (`ADXL345.__init__`).
    ///
    /// # Errors
    /// A bad `axes_map` or `rate`, a missing MCU or CS pin, an oid shortage, or
    /// a mux endpoint collision.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| identifier.clone());
        let axes_map = read_axes_map(config, SCALE_XY, SCALE_XY, SCALE_Z)?;
        let rate = config.get_int("rate", Some(DEFAULT_RATE))?;
        if !QUERY_RATES
            .iter()
            .any(|supported| i64::from(*supported) == rate)
        {
            return Err(ConfigError::new(format!("Invalid rate parameter: {rate}")));
        }
        let data_rate = rate as u32;

        // SPI, via the same `MCU_SPI_from_config` a `[spi_device]` uses.
        let setup = mcu_spi_from_config(
            config,
            printer,
            DEFAULT_SPI_MODE,
            "cs_pin",
            DEFAULT_SPI_SPEED,
        )?;
        let spi = setup.device;

        let mcu_name = config
            .get_str("spi_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&mcu_name))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
            })?;
        let builder = mcu_object.config();

        let state = Arc::new(Adxl345State {
            name,
            axes_map,
            data_rate,
            oid: Mutex::new(None),
            printer: Arc::downgrade(printer),
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the SPI resource's own callback: the
        // device oid only exists once that has run.
        let build_state = Arc::downgrade(&state);
        let build_spi = Arc::clone(&spi);
        let build_identifier = identifier.clone();
        builder
            .register_config_callback(Box::new(move |builder, _mcu| {
                let spi_oid = build_spi.oid()?;
                let oid = builder.create_oid()?;
                builder.add_config_cmd(&ConfigAdxl345 { oid, spi_oid })?;
                // Upstream arms nothing at config time: `query_adxl345` is added
                // `on_restart`, so a firmware restart leaves the chip disarmed.
                builder.add_restart_cmd(&QueryAdxl345 { oid, rest_ticks: 0 })?;
                if let Some(state) = build_state.upgrade() {
                    *state.oid.lock().unwrap_or_else(|p| p.into_inner()) = Some(oid);
                }
                Ok(())
            }))
            .map_err(|err| ConfigError::new(format!("{build_identifier}: {err}")))?;

        // The batch helper owns the mux endpoint. Its callbacks are inert until
        // the sample path is wired (see the module gap).
        let batch_cb: BatchCb = Arc::new(|_eventtime| Box::pin(async { Ok(None) }));
        let start_cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        let stop_cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        let batch_bulk = BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, BATCH_INTERVAL);
        batch_bulk.add_mux_endpoint(
            DUMP_ENDPOINT,
            DUMP_KEY,
            &state.name,
            json!({ "header": DUMP_HEADER }),
        )?;
        *state.batch_bulk.lock().unwrap_or_else(|p| p.into_inner()) = Some(batch_bulk);

        Ok(Self { state })
    }

    /// Open a measurement client (`ADXL345.start_internal_client`).
    ///
    /// This is the accelerometer interface `resonance_tester` looks for; see the
    /// module docs. Until the bulk path is wired the client receives no batches.
    pub fn start_internal_client(&self) -> Arc<AccelQueryHelper> {
        let printer = self.state.printer.upgrade();
        let helper = Arc::new(AccelQueryHelper::new(printer.as_ref()));
        let bulk = self
            .state
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        if let Some(bulk) = bulk {
            let client = Arc::clone(&helper);
            bulk.add_client(Arc::new(move |msg: &Value| client.handle_batch(msg)));
        }
        helper
    }

    /// The sensor's name: the section's sub, or the section id when unnamed
    /// (`config.get_name().split()[-1]`).
    pub fn name(&self) -> &str {
        &self.state.name
    }

    /// The configured sample rate (`data_rate`).
    pub fn data_rate(&self) -> u32 {
        self.state.data_rate
    }

    /// The parsed `axes_map` (`(source axis, signed scale)` triples).
    pub fn axes_map(&self) -> &[(usize, f64)] {
        &self.state.axes_map
    }
}

impl PrinterObject for Adxl345 {
    /// Never called through the API: [`PrinterObject::is_queryable`] is false
    /// (upstream's `ADXL345` defines no `get_status`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Adxl345 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Adxl345")
            .field("name", &self.state.name)
            .field("data_rate", &self.state.data_rate)
            .field("axes_map", &self.state.axes_map)
            .finish_non_exhaustive()
    }
}

/// Everything the callbacks and the interface share.
struct Adxl345State {
    /// The sensor's name (the section's sub, or the section id).
    name: String,
    /// `(source axis, signed scale)` per measured axis.
    axes_map: Vec<(usize, f64)>,
    /// Samples per second the chip is configured for.
    data_rate: u32,
    /// The oid the firmware assigned, set at build time.
    oid: Mutex<Option<u8>>,
    /// The machine, for [`Adxl345::start_internal_client`].
    printer: Weak<Printer>,
    /// The batch helper that owns the mux endpoint.
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

/// Upstream's `load_config` for a bare `[adxl345]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Adxl345::new(config, printer)?))
}

/// Upstream's `load_config_prefix` for `[adxl345 <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Adxl345::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::webhooks;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("adxl345", name);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A section and a tracked wrapper for it, leaked so the borrow outlives the
    /// call (tests only).
    fn wrap(name: Option<&str>, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
        ConfigWrapper::new(
            Box::leak(Box::new(section(name, options))),
            AccessTracking::shared(),
        )
    }

    /// A ready printer with `pins` and one registered `[mcu]`.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mcu = McuObject::new(ConfigSection::new("mcu", None), &printer).unwrap();
        printer.add_object("mcu", Arc::new(mcu)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    #[test]
    fn test_the_default_axes_map_is_the_identity() {
        let map = read_axes_map(&wrap(None, &[]), SCALE_XY, SCALE_XY, SCALE_Z).unwrap();
        let axes: Vec<usize> = map.iter().map(|(axis, _)| *axis).collect();
        assert_eq!(axes, vec![0, 1, 2]);
        assert_eq!(map[0].1, SCALE_XY);
        assert_eq!(map[1].1, SCALE_XY);
        assert_eq!(map[2].1, SCALE_Z);
    }

    #[test]
    fn test_a_negated_axes_map_flips_the_named_axes() {
        // The corpus `input_shaper.cfg` setting: `-x,-y,z`.
        let map = read_axes_map(
            &wrap(None, &[("axes_map", "-x,-y,z")]),
            SCALE_XY,
            SCALE_XY,
            SCALE_Z,
        )
        .unwrap();
        let shape: Vec<(usize, f64)> = map
            .iter()
            .map(|(axis, scale)| (*axis, scale.signum()))
            .collect();
        assert_eq!(shape, vec![(0, -1.), (1, -1.), (2, 1.)]);
        assert_eq!(map[0].1, -SCALE_XY);
        assert_eq!(map[2].1, SCALE_Z);
    }

    #[test]
    fn test_an_unknown_axis_is_refused() {
        let err = read_axes_map(&wrap(None, &[("axes_map", "x,y,w")]), 1., 1., 1.).unwrap_err();
        assert_eq!(err.to_string(), "Invalid axes_map parameter");
    }

    #[test]
    fn test_a_bare_section_loads_with_the_default_rate() {
        // Exactly the corpus `test/klippy/input_shaper.cfg` [adxl345] options.
        let printer = printer();
        let adxl = Adxl345::new(
            &wrap(None, &[("cs_pin", "PK7"), ("axes_map", "-x,-y,z")]),
            &printer,
        )
        .unwrap();
        assert_eq!(adxl.name(), "adxl345");
        assert_eq!(adxl.data_rate(), 3200);
        let shape: Vec<(usize, f64)> = adxl
            .axes_map()
            .iter()
            .map(|(axis, scale)| (*axis, scale.signum()))
            .collect();
        assert_eq!(shape, vec![(0, -1.), (1, -1.), (2, 1.)]);
    }

    #[test]
    fn test_a_named_section_loads_under_its_sub() {
        let printer = printer();
        let adxl = Adxl345::new(
            &wrap(Some("second"), &[("cs_pin", "PK7"), ("rate", "1600")]),
            &printer,
        )
        .unwrap();
        assert_eq!(adxl.name(), "second");
        assert_eq!(adxl.data_rate(), 1600);
    }

    #[test]
    fn test_an_unsupported_rate_is_refused() {
        let printer = printer();
        let err = Adxl345::new(
            &wrap(None, &[("cs_pin", "PK7"), ("rate", "1234")]),
            &printer,
        )
        .unwrap_err();
        assert_eq!(err.to_string(), "Invalid rate parameter: 1234");
    }

    #[tokio::test]
    async fn test_the_dump_endpoint_is_registered_under_the_sensor_key() {
        let printer = printer();
        Adxl345::new(&wrap(Some("second"), &[("cs_pin", "PK7")]), &printer).unwrap();

        let webhooks = webhooks::install(&printer).unwrap();
        let registrations = webhooks.take_mux_endpoints();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].path, DUMP_ENDPOINT);
        assert_eq!(registrations[0].key, DUMP_KEY);
        assert_eq!(registrations[0].value.as_deref(), Some("second"));
    }

    #[test]
    fn test_the_client_windows_samples_by_time() {
        let helper = AccelQueryHelper::new(None);
        // The window is [0, 0] until finish_measurements; force an end by
        // finishing without a toolhead, then the samples must fall inside.
        assert!(helper.handle_batch(&json!({"data": [[0.0, 1.0, 2.0, 3.0]]})));
        assert!(helper.has_valid_samples());
        let samples = helper.get_samples();
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].accel_x, 1.0);
    }
}
