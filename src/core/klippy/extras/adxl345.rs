//! `[adxl345]` — the ADXL345 accelerometer (upstream's
//! `klippy/extras/adxl345.py`).
//!
//! | piece | upstream |
//! |---|---|
//! | [`Adxl345`] — `cs_pin`, `axes_map`, `rate`; SPI via `MCU_SPI_from_config` | `ADXL345.__init__` |
//! | [`read_axes_map`] | `adxl345.read_axes_map` (reused by `[mpu9250]`) |
//! | [`AccelQueryHelper`] / [`AccelMeasurement`] | `adxl345.AccelQueryHelper` (reused by `[mpu9250]`) |
//! | [`Adxl345::start_internal_client`] | `ADXL345.start_internal_client` — the accelerometer interface `resonance_tester` looks for |
//! | [`convert_samples`] | `ADXL345._convert_samples` |
//! | [`setup_writes`] / `read_reg_frame` / `set_reg_frame` | `_start_measurements`' register traffic |
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
//! # The sample path
//!
//! Samples flow through the shared
//! [`FixedFreqReader`](super::bulk_sensor::FixedFreqReader), parameterized with
//! the chip's 5-byte `"BBBBB"` layout and the `query_adxl345_status` clock
//! query — same reader as the ldc1612's, only the payload differs (`[i32; 5]`
//! instead of the single-word `u32`). `_start_measurements`' register sequence
//! ([`setup_writes`]), the read/write frames, and `_convert_samples`' ±2g
//! ten-bit packing plus `axes_map` ([`convert_samples`]) follow upstream.
//!
//! One batch message carries `data` rows of `[time, x, y, z]` in mm/s², the
//! accumulated `errors` count (samples the chip flagged and the conversion
//! dropped) and `overflows`.

use std::fs;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::adxl345::{ConfigAdxl345, QueryAdxl345, QueryAdxl345Status};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, FixedFreqReader, LifecycleCb,
};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError, McuObject, McuSpi};
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

// The chip's register map (`adxl345.py`'s `REG_*`).
const REG_DEVID: u8 = 0x00;
const REG_BW_RATE: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_DATA_FORMAT: u8 = 0x31;
const REG_FIFO_CTL: u8 = 0x38;

/// The read bit of a register address (`REG_MOD_READ`).
const REG_MOD_READ: u8 = 0x80;

/// The id a working ADXL345 answers with (`ADXL345_DEV_ID`).
const ADXL345_DEV_ID: u8 = 0xe5;

/// `DATA_FORMAT` for the ±2g full-resolution ten-bit samples
/// (`set_reg(REG_DATA_FORMAT, 0x0B)`).
const SET_DATA_FORMAT: u8 = 0x0B;

/// `FIFO_CTL` in stream mode (`SET_FIFO_CTL`).
const SET_FIFO_CTL: u8 = 0x90;

/// `POWER_CTL` bit 3: leave standby and measure.
const POWER_CTL_MEASURE: u8 = 0x08;

/// Seconds between batch callbacks and the smoothing span's unit
/// (upstream `adxl345.BATCH_UPDATES` — the accelerometers process batches five
/// times as often as the shared [`BATCH_INTERVAL`] default).
const ADXL_BATCH_UPDATES: f64 = 0.100;

/// The fields one sample unpacks (upstream's 5-byte `"BBBBB"`).
const SAMPLE_FIELDS: usize = 5;

/// One converted reading: `[time, x, y, z]`, the `data` row of a batch
/// message (upstream's `Accel_Measurement` as a JSON row).
pub type AccelSample = [f64; 4];

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
// Sample conversion and chip setup (unit-tested directly)
// ===========================================================================

/// Python's `round(value, digits)` for the sample timestamps/values.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// The `BW_RATE` register code for a supported `rate` (`QUERY_RATES[rate]`).
fn bw_rate_code(data_rate: u32) -> Option<u8> {
    QUERY_RATES
        .iter()
        .position(|rate| *rate == data_rate)
        .map(|index| 0x8 + index as u8)
}

/// The register writes `_start_measurements` performs before arming the bulk
/// stream, in order: standby, ten-bit full resolution, an empty FIFO, the
/// requested sample rate, then FIFO stream mode.
///
/// `None` when `data_rate` is not one of [`QUERY_RATES`] — [`Adxl345::new`]
/// refuses those, so the start path only sees this if a section was built
/// some other way.
fn setup_writes(data_rate: u32) -> Option<[(u8, u8); 5]> {
    Some([
        (REG_POWER_CTL, 0x00),
        (REG_DATA_FORMAT, SET_DATA_FORMAT),
        (REG_FIFO_CTL, 0x00),
        (REG_BW_RATE, bw_rate_code(data_rate)?),
        (REG_FIFO_CTL, SET_FIFO_CTL),
    ])
}

/// The bytes `read_reg` clocks out: the address with its read bit set, then a
/// dummy byte whose reply carries the value (upstream's
/// `spi_transfer([reg | REG_MOD_READ, 0x00])`).
fn read_reg_frame(reg: u8) -> [u8; 2] {
    [reg | REG_MOD_READ, 0x00]
}

/// The value a [`read_reg_frame`] reply carries: the byte after the echoed
/// address, or `None` when the reply is short.
fn read_reg_value(response: &[u8]) -> Option<u8> {
    response.get(1).copied()
}

/// The bytes `set_reg` shifts out (upstream's `spi_send([reg, val & 0xFF])`).
fn set_reg_frame(reg: u8, value: u8) -> [u8; 2] {
    [reg, value]
}

/// Map raw ADXL345 samples to scaled acceleration (`ADXL345._convert_samples`).
///
/// One raw sample is the five fields the `"BBBBB"` format unpacks — `xlow`,
/// `ylow`, `zlow`, `xzhigh`, `yzhigh` — which pack into the chip's three
/// ten-bit axis words. A sample with bit 7 of `yzhigh` set is the chip's error
/// flag: it is dropped and counted into `error_count` (upstream's
/// `last_error_count`, accumulated across a measurement). `axes_map` gives
/// each output axis its source component and signed scale.
pub fn convert_samples(
    axes_map: &[(usize, f64)],
    raw: &[(f64, [i32; SAMPLE_FIELDS])],
    error_count: &mut u64,
) -> Vec<AccelSample> {
    let (x_pos, x_scale) = axes_map[0];
    let (y_pos, y_scale) = axes_map[1];
    let (z_pos, z_scale) = axes_map[2];
    let mut out = Vec::with_capacity(raw.len());
    for (ptime, fields) in raw {
        let [xlow, ylow, zlow, xzhigh, yzhigh] = *fields;
        if yzhigh & 0x80 != 0 {
            *error_count += 1;
            continue;
        }
        // Each axis is ten bits, the high bits split across the two trailing
        // bytes; subtracting the sign bit's weight sign-extends the word.
        let raw_xyz = [
            (xlow | ((xzhigh & 0x1f) << 8)) - ((xzhigh & 0x10) << 9),
            (ylow | ((yzhigh & 0x1f) << 8)) - ((yzhigh & 0x10) << 9),
            (zlow | ((xzhigh & 0xe0) << 3) | ((yzhigh & 0xe0) << 6)) - ((yzhigh & 0x40) << 7),
        ];
        out.push([
            round(*ptime, 6),
            round(f64::from(raw_xyz[x_pos]) * x_scale, 6),
            round(f64::from(raw_xyz[y_pos]) * y_scale, 6),
            round(f64::from(raw_xyz[z_pos]) * z_scale, 6),
        ]);
    }
    out
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
    /// `wait_moves` seam on [`ToolHeadObject`], so only the window's end is
    /// recorded.
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
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // Samples per second the clock regression expects, smoothed over two
        // batch windows (`chip_smooth = self.data_rate * BATCH_UPDATES * 2`).
        // Upstream's `BATCH_UPDATES` is 0.100 for the accelerometers.
        let chip_smooth = f64::from(data_rate) * ADXL_BATCH_UPDATES * 2.;
        let state = Arc::new(Adxl345State {
            name,
            axes_map,
            data_rate,
            oid,
            spi,
            mcu_object: Arc::clone(&mcu_object),
            ffreader: FixedFreqReader::with_payload_format(
                chip_smooth,
                "BBBBB",
                &format!("{} oid=%c", QueryAdxl345Status::NAME),
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            last_error_count: Mutex::new(0),
            printer: Arc::downgrade(printer),
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the SPI resource's own callback: the
        // device oid only exists once that has run.
        let build_state = Arc::downgrade(&state);
        let build_identifier = identifier.clone();
        builder
            .register_config_callback(Box::new(move |builder, mcu| {
                if let Some(state) = build_state.upgrade() {
                    state.build(builder, mcu)?;
                }
                Ok(())
            }))
            .map_err(|err| ConfigError::new(format!("{build_identifier}: {err}")))?;

        // The batch helper owns the mux endpoint and drives start/stop/batch.
        let start_state = Arc::downgrade(&state);
        let start_cb: LifecycleCb = Arc::new(move || {
            let start_state = start_state.clone();
            Box::pin(async move {
                let state = start_state
                    .upgrade()
                    .ok_or_else(|| "adxl345 is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "adxl345 is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "adxl345 is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, ADXL_BATCH_UPDATES);
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
    /// module docs. Registering the client starts the chip's measurement stream
    /// (upstream's `batch_bulk.add_client`, whose first client runs
    /// `_start_measurements`) and dropping it — by returning `false` from
    /// [`AccelQueryHelper::handle_batch`] — stops it.
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
    /// The oid the firmware assigned.
    oid: u8,
    /// The SPI bus, shared with the chip's register traffic.
    spi: Arc<McuSpi>,
    /// The MCU the chip is on (the SPI bus's).
    mcu_object: Arc<McuObject>,
    /// The reader: 5-byte `"BBBBB"` samples, `query_adxl345_status` queries.
    ffreader: FixedFreqReader<[i32; SAMPLE_FIELDS]>,
    /// Samples the chip flagged as errors since the last start
    /// (`last_error_count`).
    last_error_count: Mutex<u64>,
    /// The machine, for [`Adxl345::start_internal_client`].
    printer: Weak<Printer>,
    /// The batch helper that owns the mux endpoint.
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Adxl345State {
    /// The build-time half: the chip's config command, its disarmed-at-restart
    /// query, and the reader's data binding (`_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        builder.add_config_cmd(&ConfigAdxl345 {
            oid: self.oid,
            spi_oid: self.spi.oid()?,
        })?;
        // Upstream arms nothing at config time: `query_adxl345` is added
        // `on_restart`, so a firmware restart leaves the chip disarmed.
        builder.add_restart_cmd(&QueryAdxl345 {
            oid: self.oid,
            rest_ticks: 0,
        })?;
        self.ffreader.bind(mcu, &self.mcu_object, self.oid)?;
        Ok(())
    }

    /// Read one register (`read_reg`): the byte the reply carries after the
    /// echoed address.
    async fn read_reg(&self, reg: u8) -> Result<u8, String> {
        let response = self
            .spi
            .transfer(&read_reg_frame(reg))
            .await
            .map_err(|err| err.to_string())?;
        read_reg_value(&response).ok_or_else(|| {
            format!(
                "adxl345 register 0x{reg:02x}: short read ({} byte(s))",
                response.len()
            )
        })
    }

    /// Write one register and read it back (`set_reg`): a mismatch means the
    /// chip did not take the setting.
    async fn set_reg(&self, reg: u8, value: u8) -> Result<(), String> {
        self.spi
            .send(&set_reg_frame(reg, value))
            .map_err(|err| err.to_string())?;
        let stored = self.read_reg(reg).await?;
        if stored != value {
            return Err(format!(
                "Failed to set ADXL345 register [0x{reg:02x}] to 0x{value:02x}: got 0x{stored:02x}. \
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty adxl345 chip."
            ));
        }
        Ok(())
    }

    /// The connected MCU, or why the sensor cannot reach it.
    fn connected_mcu(&self) -> Result<Arc<Mcu>, String> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| "adxl345's MCU is not connected".to_string())
    }

    /// Bring the chip up and arm the bulk stream (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        // In case of miswiring, the device id tells a real chip from noise.
        let dev_id = self.read_reg(REG_DEVID).await?;
        if dev_id != ADXL345_DEV_ID {
            return Err(format!(
                "Invalid adxl345 id (got {dev_id:x} vs {ADXL345_DEV_ID:x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty adxl345 chip."
            ));
        }
        let writes = setup_writes(self.data_rate)
            .ok_or_else(|| format!("Invalid rate parameter: {}", self.data_rate))?;
        for (reg, value) in writes {
            self.set_reg(reg, value).await?;
        }

        // Start bulk reading: one report every four samples.
        let mcu = self.connected_mcu()?;
        let rest_ticks = mcu
            .seconds_to_clock(4. / f64::from(self.data_rate))
            .map_err(|err| err.to_string())? as u32;
        mcu.send_msg(&QueryAdxl345 {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(|err| err.to_string())?;
        self.set_reg(REG_POWER_CTL, POWER_CTL_MEASURE).await?;

        tracing::info!("ADXL345 starting '{}' measurements", self.name);
        // Initialize clock tracking.
        self.ffreader
            .note_start()
            .await
            .map_err(|err| err.to_string())?;
        *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 0;
        Ok(())
    }

    /// Halt the bulk stream (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        self.set_reg(REG_POWER_CTL, 0x00).await?;
        // Upstream's `send_wait_ack` blocks for the ack; the queued messages
        // ahead of this one (including the register write above) preserve the
        // order, so the fire-and-forget send is enough here.
        let mcu = self.connected_mcu()?;
        mcu.send_msg(&QueryAdxl345 {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(|err| err.to_string())?;
        self.ffreader.note_end();
        tracing::info!("ADXL345 finished '{}' measurements", self.name);
        Ok(())
    }

    /// One batch: pull, convert, report (`_process_batch`).
    async fn process_batch(&self, _eventtime: f64) -> Result<Option<Value>, String> {
        let raw = self
            .ffreader
            .pull_samples()
            .await
            .map_err(|err| err.to_string())?;
        let mut error_count = self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let data = convert_samples(&self.axes_map, &raw, &mut error_count);
        let errors = *error_count;
        if data.is_empty() {
            return Ok(None);
        }
        let overflows = self.ffreader.get_last_overflows();
        Ok(Some(json!({
            "data": data,
            "errors": errors,
            "overflows": overflows,
        })))
    }
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

    /// The identity `axes_map` with unit scales, so a conversion asserts the
    /// ten-bit packing alone.
    fn unit_axes() -> Vec<(usize, f64)> {
        vec![(0, 1.0), (1, 1.0), (2, 1.0)]
    }

    #[test]
    fn test_convert_samples_unpacks_the_ten_bit_axes() {
        let mut errors = 0;
        // xlow=0x34, ylow=0x12, zlow=0x56, xzhigh=0x03 (x bits 8..12).
        let raw = vec![(1.5, [0x34, 0x12, 0x56, 0x03, 0x00])];
        let rows = convert_samples(&unit_axes(), &raw, &mut errors);
        assert_eq!(rows, vec![[1.5, 820.0, 18.0, 86.0]]);
        assert_eq!(errors, 0);

        // All bits set: each axis word is signed, so -1 on every axis.
        let raw = vec![(2.0, [0xff, 0xff, 0xff, 0xff, 0x7f])];
        let rows = convert_samples(&unit_axes(), &raw, &mut errors);
        assert_eq!(rows, vec![[2.0, -1.0, -1.0, -1.0]]);
    }

    #[test]
    fn test_convert_samples_applies_the_axes_map_and_scale() {
        // The corpus `input_shaper.cfg` setting, `-x,-y,z`, at the chip's ±2g
        // full scale (x/y 1/265 g/LSB, z 1/256 g/LSB).
        let axes_map = vec![(0, -SCALE_XY), (1, -SCALE_XY), (2, SCALE_Z)];
        let mut errors = 0;
        let raw = vec![(1.0, [0x34, 0x12, 0x56, 0x03, 0x00])];
        let rows = convert_samples(&axes_map, &raw, &mut errors);
        assert_eq!(rows[0][0], 1.0);
        assert!((rows[0][1] + 820.0 * SCALE_XY).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][2] + 18.0 * SCALE_XY).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][3] - 86.0 * SCALE_Z).abs() < 1e-6, "{rows:?}");

        // A permuted map reads the components from other axes.
        let swapped = vec![(2, SCALE_Z), (0, SCALE_XY), (1, SCALE_XY)];
        let rows = convert_samples(&swapped, &raw, &mut errors);
        assert!((rows[0][1] - 86.0 * SCALE_Z).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][2] - 820.0 * SCALE_XY).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][3] - 18.0 * SCALE_XY).abs() < 1e-6, "{rows:?}");
    }

    #[test]
    fn test_convert_samples_drops_error_flagged_samples() {
        let mut errors = 0;
        // yzhigh bit 7 is the chip's error flag; the next sample is fine.
        let raw = vec![
            (1.0, [0x10, 0x00, 0x00, 0x00, 0x80]),
            (2.0, [0x01, 0x00, 0x00, 0x00, 0x00]),
        ];
        let rows = convert_samples(&unit_axes(), &raw, &mut errors);
        assert_eq!(rows, vec![[2.0, 1.0, 0.0, 0.0]]);
        assert_eq!(errors, 1);
        // The count accumulates over batches (upstream's `last_error_count`).
        convert_samples(&unit_axes(), &raw, &mut errors);
        assert_eq!(errors, 2);
    }

    #[test]
    fn test_the_register_setup_sequence_is_upstreams() {
        assert_eq!(
            setup_writes(3200).expect("a supported rate"),
            [
                (0x2d, 0x00), // POWER_CTL: standby
                (0x31, 0x0b), // DATA_FORMAT: ±2g full resolution
                (0x38, 0x00), // FIFO_CTL: empty the FIFO
                (0x2c, 0x0f), // BW_RATE: 3200 Hz
                (0x38, 0x90), // FIFO_CTL: stream mode
            ]
        );
        // The rate codes are upstream's `QUERY_RATES` table.
        for (rate, code) in [
            (25, 0x8),
            (50, 0x9),
            (100, 0xa),
            (200, 0xb),
            (400, 0xc),
            (800, 0xd),
            (1600, 0xe),
            (3200, 0xf),
        ] {
            assert_eq!(bw_rate_code(rate), Some(code), "rate {rate}");
            assert_eq!(setup_writes(rate).expect("supported")[3], (0x2c, code));
        }
        assert_eq!(bw_rate_code(1234), None);
        assert!(setup_writes(1234).is_none());
    }

    #[test]
    fn test_register_frames_set_and_clear_the_read_bit() {
        // `read_reg` clocks the address with `REG_MOD_READ` set plus a dummy
        // byte out; the reply's second byte is the register's value.
        assert_eq!(read_reg_frame(REG_DEVID), [0x80, 0x00]);
        assert_eq!(read_reg_frame(REG_POWER_CTL), [0xad, 0x00]);
        assert_eq!(read_reg_value(&[0x80, 0xe5]), Some(0xe5));
        assert_eq!(read_reg_value(&[0x80]), None);
        // `set_reg` shifts the plain address and the value.
        assert_eq!(set_reg_frame(REG_DATA_FORMAT, 0x0b), [0x31, 0x0b]);
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
