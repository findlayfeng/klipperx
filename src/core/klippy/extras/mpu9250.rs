//! `[mpu9250 <name>]` — the MPU9250 and its pin-compatible siblings
//! (upstream's `klippy/extras/mpu9250.py`).
//!
//! An MPU-family accelerometer on an I2C bus. The section reads the I2C
//! mechanism options (`i2c_address` / `i2c_speed` / `i2c_mcu` / `i2c_bus` /
//! the software-pin pair), asks its MCU for an [`McuI2c`] (the same resource
//! `[i2c_device]` uses), and reserves an oid for the firmware's
//! `config_mpu9250`.
//!
//! | piece | upstream |
//! |---|---|
//! | [`Mpu9250`] | `MPU9250.__init__` |
//! | [`read_axes_map`] / [`convert_samples`] | `adxl345.read_axes_map` / `MPU9250._convert_samples` |
//! | [`setup_writes`] / `config_writes` | `_start_measurements`' register sequence |
//! | [`AccelQueryHelper`] | `adxl345.AccelQueryHelper` |
//! | `mpu9250/dump_mpu9250` mux endpoint | `batch_bulk.add_mux_endpoint(…)` |
//! | [`ConfigMpu9250`] / [`QueryMpu9250`] | `config_mpu9250` / `query_mpu9250` (in [`crate::core::klippy::cmd::mpu9250`]) |
//!
//! # The accelerometer interface
//!
//! A consumer (`resonance_tester`) finds the chip by name and asks it for a
//! measurement client: [`Mpu9250::start_internal_client`] returns an
//! [`AccelQueryHelper`], the same shape upstream's `MPU9250.start_internal_client`
//! returns. The helper is registered on the shared [`BatchBulkHelper`] and
//! collects the batches the dump endpoint also streams. `[adxl345]` exposes the
//! same pair of names, so a caller can treat both chips alike.
//!
//! # The sample path
//!
//! Samples flow through the shared
//! [`FixedFreqReader`](crate::core::klippy::extras::bulk_sensor::FixedFreqReader)
//! parameterized with the chip's 6-byte `">hhh"` layout — three signed 16-bit
//! components, 51 // 6 = 8 samples per message — and the
//! `query_mpu9250_status` clock query. `_start_measurements`' register sequence
//! ([`setup_writes`], [`config_writes`]) and `_convert_samples` ([`convert_samples`])
//! follow upstream.
//!
//! One batch message carries `data` rows of `[time, x, y, z]` in mm/s², an
//! `errors` count (the MPU9250 flags none, so it stays zero) and `overflows`.
//!
//! The G-Code commands upstream registers through `adxl345.AccelCommandHelper`
//! (`ACCELEROMETER_MEASURE` / `ACCELEROMETER_QUERY` / `ACCELEROMETER_DEBUG_READ`
//! / `ACCELEROMETER_DEBUG_WRITE`) are not registered by either section yet.

use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use serde_json::{json, Value};

use crate::core::klippy::cmd::mpu9250::{
    ConfigMpu9250, QueryMpu9250, QUERY_MPU9250_STATUS_MSGFORMAT,
};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, FixedFreqReader, LifecycleCb,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{ConfigBuilder, I2cMode, Mcu, McuError, McuI2c, McuObject};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Both `[mpu9250]` and `[mpu9250 <name>]` are valid upstream. Loaded with the
// other I2C consumers (order 40), after `[board_pins]` (30), because a software
// bus may name its pins through an alias.
section!(
    "mpu9250",
    order = 40,
    load = load_config,
    prefix = load_config_prefix
);

/// The path the dump endpoint registers (`mpu9250.py`).
pub const DUMP_ENDPOINT: &str = "mpu9250/dump_mpu9250";

/// The mux key that selects a sensor instance.
pub const DUMP_KEY: &str = "sensor";

/// The default 7-bit device address (`MPU9250_ADDR`).
pub const DEFAULT_ADDR: i64 = 0x68;

/// The default I2C clock (`default_speed` at the `MCU_I2C_from_config` call).
pub const DEFAULT_SPEED: i64 = 400_000;

/// The lowest clock `MCU_I2C_from_config` accepts (`bus.py`'s `minval`).
const MIN_SPEED: i64 = 100_000;

/// The one sample rate the chip's register map offers (`SAMPLE_RATE_DIVS`).
pub const SUPPORTED_RATE: i64 = 4000;

/// Earth gravity in mm/s² (`FREEFALL_ACCEL`).
const FREEFALL_ACCEL: f64 = 9.80665 * 1000.;

/// 1/4096 g/LSB at the 8g full scale, in mm/s² (`SCALE`).
pub const SCALE: f64 = 0.000244140625 * FREEFALL_ACCEL;

/// The report header a connecting dump client receives
/// (`('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')`).
pub const DUMP_HEADER: [&str; 4] = ["time", "x_acceleration", "y_acceleration", "z_acceleration"];

/// Seconds between batch callbacks and the smoothing span's unit (upstream
/// `mpu9250.BATCH_UPDATES`).
const MPU_BATCH_UPDATES: f64 = 0.100;

/// The three signed 16-bit fields one sample unpacks (upstream's `">hhh"`).
const SAMPLE_FIELDS: usize = 3;

// The chip's register map (`mpu9250.py`'s `REG_*`).
const REG_DEVID: u8 = 0x75;
const REG_FIFO_EN: u8 = 0x23;
const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_CONFIG2: u8 = 0x1D;
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;
const REG_INT_STATUS: u8 = 0x3A;

/// The one sample-rate divider the section accepts (`SAMPLE_RATE_DIVS`).
const SET_SMPLRT_DIV: u8 = 0x00;

/// FIFO in stream mode (`SET_CONFIG`).
const SET_CONFIG: u8 = 0x01;

/// 8g full scale (`SET_ACCEL_CONFIG`).
const SET_ACCEL_CONFIG: u8 = 0x10;

/// 1046 Hz bandwidth, 0.503 ms delay, 4 kHz sample rate (`SET_ACCEL_CONFIG2`).
const SET_ACCEL_CONFIG2: u8 = 0x08;

/// Leave sleep mode (`SET_PWR_MGMT_1_WAKE`).
const SET_PWR_MGMT_1_WAKE: u8 = 0x00;

/// Back to sleep (`SET_PWR_MGMT_1_SLEEP`).
const SET_PWR_MGMT_1_SLEEP: u8 = 0x40;

/// Accelerometer axes on (`SET_PWR_MGMT_2_ACCEL_ON`).
const SET_PWR_MGMT_2_ACCEL_ON: u8 = 0x07;

/// Everything off (`SET_PWR_MGMT_2_OFF`).
const SET_PWR_MGMT_2_OFF: u8 = 0x3F;

/// Reset the FIFO (`SET_USER_FIFO_RESET`).
const SET_USER_FIFO_RESET: u8 = 0x04;

/// Leave the FIFO enabled (`SET_USER_FIFO_EN`).
const SET_USER_FIFO_EN: u8 = 0x40;

/// Let samples into the FIFO (`SET_ENABLE_FIFO`).
const SET_ENABLE_FIFO: u8 = 0x08;

/// Stop samples entering the FIFO (`SET_DISABLE_FIFO`).
const SET_DISABLE_FIFO: u8 = 0x00;

/// One axis of the sensor's `axes_map`: which raw component it reads and the
/// sign/scale applied.
pub type AxesMap = [(usize, f64); 3];

/// One converted reading: `[time, x, y, z]`, the `data` row of a dump message.
pub type AccelSample = [f64; 4];

/// One accelerometer reading a query collected (`adxl345.Accel_Measurement`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelMeasurement {
    /// Print time of the sample.
    pub time: f64,
    /// Acceleration along the mapped X axis, in mm/s².
    pub accel_x: f64,
    /// Acceleration along the mapped Y axis.
    pub accel_y: f64,
    /// Acceleration along the mapped Z axis.
    pub accel_z: f64,
}

// ===========================================================================
// Pure helpers (unit-tested directly)
// ===========================================================================

/// Python's `round(value, digits)` for the sample timestamps/values.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// Read the `axes_map` option into per-axis `(raw index, scale)` pairs
/// (`adxl345.read_axes_map`).
///
/// The default is the identity map `x, y, z`. Each entry names a raw component
/// (`x`/`y`/`z`) with an optional sign; anything else is refused with
/// upstream's message.
///
/// # Errors
/// [`ConfigError`] when the list does not have three entries or names an axis
/// other than `x`/`y`/`z` (optionally negated).
pub fn read_axes_map(config: &ConfigWrapper, scale: f64) -> Result<AxesMap, ConfigError> {
    let default = vec!["x".to_string(), "y".to_string(), "z".to_string()];
    let axes = config.get_list("axes_map", ',').unwrap_or(default);
    if axes.len() != 3 {
        return Err(ConfigError::new(format!(
            "Option 'axes_map' in section '{}' must have 3 elements",
            config.identifier()
        )));
    }
    let mut map = [(0_usize, scale); 3];
    for (index, axis) in axes.iter().enumerate() {
        map[index] = match axis.trim() {
            "x" => (0, scale),
            "-x" => (0, -scale),
            "y" => (1, scale),
            "-y" => (1, -scale),
            "z" => (2, scale),
            "-z" => (2, -scale),
            _ => return Err(ConfigError::new("Invalid axes_map parameter")),
        };
    }
    Ok(map)
}

/// Map raw sensor counts to scaled acceleration
/// (`MPU9250._convert_samples`).
///
/// `raw` is one `(print time, [x, y, z])` sample per row — the MPU9250's
/// `">hhh"` wire format, signed 16-bit components — and the result is the dump
/// message's `[time, x, y, z]` rows with upstream's 6-digit rounding.
pub fn convert_samples(
    axes_map: &AxesMap,
    raw: &[(f64, [i32; SAMPLE_FIELDS])],
) -> Vec<AccelSample> {
    let (x_pos, x_scale) = axes_map[0];
    let (y_pos, y_scale) = axes_map[1];
    let (z_pos, z_scale) = axes_map[2];
    raw.iter()
        .map(|(ptime, fields)| {
            let raw_xyz = [
                f64::from(fields[0]),
                f64::from(fields[1]),
                f64::from(fields[2]),
            ];
            [
                round(*ptime, 6),
                round(raw_xyz[x_pos] * x_scale, 6),
                round(raw_xyz[y_pos] * y_scale, 6),
                round(raw_xyz[z_pos] * z_scale, 6),
            ]
        })
        .collect()
}

/// The device names the chip's id byte maps to (`MPU_DEV_IDS`); the two odd ids
/// are marked defective upstream too.
fn device_name(dev_id: u8) -> Option<&'static str> {
    Some(match dev_id {
        0x74 => "mpu-9515",
        0x73 => "mpu-9255",
        0x71 => "mpu-9250",
        0x70 => "mpu-6500",
        0x68 => "mpu-6050",
        0x75 => "mpu-unknown (DEFECTIVE! USE WITH CAUTION!)",
        0x69 => "mpu-unknown (DEFECTIVE! USE WITH CAUTION!)",
        _ => return None,
    })
}

/// The writes `_start_measurements` issues first: wake the chip and turn the
/// accelerometer axes on.
fn setup_writes() -> [(u8, u8); 2] {
    [
        (REG_PWR_MGMT_1, SET_PWR_MGMT_1_WAKE),
        (REG_PWR_MGMT_2, SET_PWR_MGMT_2_ACCEL_ON),
    ]
}

/// The writes that configure the sample rate and the accelerometer, then empty
/// and enable the FIFO (`_start_measurements`, after the 20 ms wake pause).
fn config_writes() -> [(u8, u8); 7] {
    [
        (REG_SMPLRT_DIV, SET_SMPLRT_DIV),
        (REG_CONFIG, SET_CONFIG),
        (REG_ACCEL_CONFIG, SET_ACCEL_CONFIG),
        (REG_ACCEL_CONFIG2, SET_ACCEL_CONFIG2),
        (REG_FIFO_EN, SET_DISABLE_FIFO),
        (REG_USER_CTRL, SET_USER_FIFO_RESET),
        (REG_USER_CTRL, SET_USER_FIFO_EN),
    ]
}

/// The writes `_finish_measurements` issues once the bulk query is stopped.
fn shutdown_writes() -> [(u8, u8); 2] {
    [
        (REG_PWR_MGMT_1, SET_PWR_MGMT_1_SLEEP),
        (REG_PWR_MGMT_2, SET_PWR_MGMT_2_OFF),
    ]
}

// ===========================================================================
// AccelQueryHelper
// ===========================================================================

/// The state the query helper collects under its lock.
struct QueryState {
    is_finished: bool,
    request_start_time: f64,
    request_end_time: f64,
    msgs: Vec<Value>,
}

/// Collects a sensor's batch messages for one query window
/// (`adxl345.AccelQueryHelper`).
///
/// [`Mpu9250::start_internal_client`] builds one, registers it on the batch
/// helper, and hands it back: the caller ends the window with
/// [`finish_measurements`](Self::finish_measurements) and reads the readings
/// with [`get_samples`](Self::get_samples).
pub struct AccelQueryHelper {
    printer: Weak<Printer>,
    state: Mutex<QueryState>,
}

impl AccelQueryHelper {
    /// The largest number of batches collected before the helper detaches
    /// (`len(self.msgs) >= 10000`).
    const MAX_MSGS: usize = 10_000;

    /// Build a helper whose window opens at the toolhead's last move time.
    pub fn new(printer: Weak<Printer>) -> Arc<Self> {
        let start = last_move_time(&printer);
        Arc::new(Self {
            printer,
            state: Mutex::new(QueryState {
                is_finished: false,
                request_start_time: start,
                request_end_time: start,
                msgs: Vec::new(),
            }),
        })
    }

    /// Close the collection window (`finish_measurements`).
    ///
    /// Upstream additionally waits for the queued moves (`toolhead.wait_moves`)
    /// so the window's end time covers every move; here the caller can await
    /// [`ToolHeadObject::flush_step_generation`] itself before finishing.
    pub fn finish_measurements(&self) {
        let end = last_move_time(&self.printer);
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        state.request_end_time = end;
        state.is_finished = true;
    }

    /// Take one batch; `false` detaches the helper from the batch loop
    /// (`handle_batch`).
    pub fn handle_batch(&self, message: &Value) -> bool {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        if state.is_finished || state.msgs.len() >= Self::MAX_MSGS {
            return false;
        }
        state.msgs.push(message.clone());
        true
    }

    /// Whether any collected batch overlaps the request window
    /// (`has_valid_samples`).
    pub fn has_valid_samples(&self) -> bool {
        let state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        for message in &state.msgs {
            let Some(data) = message.get("data").and_then(Value::as_array) else {
                continue;
            };
            let (Some(first), Some(last)) = (data.first(), data.last()) else {
                continue;
            };
            let (Some(first_time), Some(last_time)) = (row_time(first), row_time(last)) else {
                continue;
            };
            if first_time > state.request_end_time || last_time < state.request_start_time {
                continue;
            }
            return true;
        }
        false
    }

    /// The readings inside the request window, in time order (`get_samples`).
    pub fn get_samples(&self) -> Vec<AccelMeasurement> {
        let state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        let mut samples = Vec::new();
        for message in &state.msgs {
            let Some(data) = message.get("data").and_then(Value::as_array) else {
                continue;
            };
            for row in data {
                let Some(row) = accel_row(row) else {
                    continue;
                };
                if row.time < state.request_start_time {
                    continue;
                }
                if row.time > state.request_end_time {
                    break;
                }
                samples.push(row);
            }
        }
        samples
    }

    /// Seed the request window, for tests that have no `toolhead`
    /// (`AccelQueryHelper` reads the times from it).
    #[cfg(test)]
    pub(crate) fn force_request_window(&self, start: f64, end: f64) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        state.request_start_time = start;
        state.request_end_time = end;
    }
}

/// The toolhead's last move time, or `0.0` before the machine has one.
fn last_move_time(printer: &Weak<Printer>) -> f64 {
    printer
        .upgrade()
        .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))
        .map(|toolhead| toolhead.get_last_move_time())
        .unwrap_or(0.0)
}

/// The time of a `[time, x, y, z]` row, if it has one.
fn row_time(row: &Value) -> Option<f64> {
    row.as_array()?.first()?.as_f64()
}

/// One `[time, x, y, z]` row as a reading.
fn accel_row(row: &Value) -> Option<AccelMeasurement> {
    let row = row.as_array()?;
    if row.len() < 4 {
        return None;
    }
    Some(AccelMeasurement {
        time: row[0].as_f64()?,
        accel_x: row[1].as_f64()?,
        accel_y: row[2].as_f64()?,
        accel_z: row[3].as_f64()?,
    })
}

// ===========================================================================
// The sensor
// ===========================================================================

/// One configured MPU-family accelerometer.
pub struct Mpu9250 {
    state: Arc<Mpu9250State>,
}

impl Mpu9250 {
    /// Read the section and wire the sensor up (`MPU9250.__init__`).
    ///
    /// # Errors
    /// A missing or out-of-range option, an unsupported `rate`, an unknown MCU
    /// or pin, an oid shortage, or a dump endpoint already registered under
    /// this name.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().unwrap_or_else(|| {
            identifier
                .split_whitespace()
                .last()
                .unwrap_or(&identifier)
                .to_string()
        });

        // --- rate ---------------------------------------------------------
        let rate = config.get_int("rate", Some(SUPPORTED_RATE))?;
        if rate != SUPPORTED_RATE {
            return Err(ConfigError::new(format!("Invalid rate parameter: {rate}")));
        }

        // --- axes map -----------------------------------------------------
        let axes_map = read_axes_map(config, SCALE)?;

        // --- MCU + I2C (MCU_I2C_from_config) ------------------------------
        let mcu_name = config
            .get_str("i2c_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());
        let object_name = if mcu_name == "mcu" {
            "mcu".to_string()
        } else {
            format!("mcu {mcu_name}")
        };
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&object_name)
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
            })?;

        let address =
            config.get_int_bounded("i2c_address", Some(DEFAULT_ADDR), Some(0), Some(127))? as u8;
        let speed =
            config.get_int_bounded("i2c_speed", Some(DEFAULT_SPEED), Some(MIN_SPEED), None)? as u32;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let mode = match (
            config.get_str("i2c_software_scl_pin"),
            config.get_str("i2c_software_sda_pin"),
        ) {
            (Some(scl), Some(sda)) => {
                let scl_params = pins
                    .lookup_pin(&scl, false, false, Some("scl"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                let sda_params = pins
                    .lookup_pin(&sda, false, false, Some("sda"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                if scl_params.chip_name != mcu_name || sda_params.chip_name != mcu_name {
                    return Err(ConfigError::new(format!(
                        "Section '{identifier}': i2c pins must be on the same mcu '{mcu_name}'"
                    )));
                }
                I2cMode::Software {
                    scl_pin: scl_params.pin,
                    sda_pin: sda_params.pin,
                    speed,
                }
            }
            (None, None) => I2cMode::Hardware {
                bus: config.get_str("i2c_bus"),
                speed,
            },
            _ => {
                return Err(ConfigError::new(format!(
                    "Section '{identifier}': both 'i2c_software_scl_pin' and \
                     'i2c_software_sda_pin' must be set"
                )));
            }
        };
        let i2c = mcu_object.setup_i2c(mode, address);

        // --- oid + build callback -----------------------------------------
        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(Mpu9250State {
            name,
            oid,
            rate,
            address,
            speed,
            axes_map,
            printer: Arc::downgrade(printer),
            i2c,
            mcu_object: Arc::clone(&mcu_object),
            // Samples per second the clock regression expects, smoothed over
            // two batch windows (`chip_smooth = self.data_rate * BATCH_UPDATES * 2`).
            ffreader: FixedFreqReader::with_payload_format(
                rate as f64 * MPU_BATCH_UPDATES * 2.,
                ">hhh",
                QUERY_MPU9250_STATUS_MSGFORMAT,
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            last_error_count: Mutex::new(0),
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the I2C resource's own callback so the
        // device oid exists when `config_mpu9250` is added (the resource
        // allocates it in its config callback).
        let callback_state = Arc::downgrade(&state);
        builder
            .register_config_callback(Box::new(move |builder, mcu| {
                if let Some(state) = callback_state.upgrade() {
                    state.build(builder, mcu)?;
                }
                Ok(())
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // --- batch helper (BatchBulkHelper + mux endpoint) ----------------
        let start_state = Arc::downgrade(&state);
        let start_cb: LifecycleCb = Arc::new(move || {
            let start_state = start_state.clone();
            Box::pin(async move {
                let state = start_state
                    .upgrade()
                    .ok_or_else(|| "mpu9250 is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "mpu9250 is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "mpu9250 is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, MPU_BATCH_UPDATES);
        batch_bulk.add_mux_endpoint(
            DUMP_ENDPOINT,
            DUMP_KEY,
            &state.name,
            json!({ "header": DUMP_HEADER }),
        )?;
        *state.batch_bulk.lock().unwrap_or_else(|p| p.into_inner()) = Some(batch_bulk);

        Ok(Self { state })
    }

    /// The sensor's name (the section's sub, or the bare section id).
    pub fn name(&self) -> &str {
        &self.state.name
    }

    /// The configured sample rate in Hz (`data_rate`).
    pub fn data_rate(&self) -> i64 {
        self.state.rate
    }

    /// The 7-bit I2C address the section configured (`i2c_address`).
    pub fn i2c_address(&self) -> u8 {
        self.state.address
    }

    /// The I2C clock the section configured (`i2c_speed`).
    pub fn i2c_speed(&self) -> u32 {
        self.state.speed
    }

    /// The sensor's object id.
    pub fn oid(&self) -> u8 {
        self.state.oid
    }

    /// Register a measurement client and hand it back
    /// (`MPU9250.start_internal_client`).
    ///
    /// Registering it starts the chip's stream (the helper's first client runs
    /// `_start_measurements`); the caller ends the window with
    /// [`AccelQueryHelper::finish_measurements`], which detaches the client and
    /// runs the chip's own `_finish_measurements`.
    pub fn start_internal_client(&self) -> Arc<AccelQueryHelper> {
        let helper = AccelQueryHelper::new(self.state.printer.clone());
        if let Some(bulk) = self
            .state
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
        {
            let client = Arc::clone(&helper);
            bulk.add_client(Arc::new(move |message: &Value| {
                client.handle_batch(message)
            }));
        }
        helper
    }

    /// How many clients the batch helper holds (tests and bookkeeping).
    #[cfg(test)]
    pub(crate) fn client_count(&self) -> usize {
        self.state
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
            .map(|bulk| bulk.client_count())
            .unwrap_or(0)
    }
}

impl PrinterObject for Mpu9250 {
    /// Never reached through the API: [`PrinterObject::is_queryable`] is false,
    /// as upstream's object defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Mpu9250 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Mpu9250")
            .field("name", &self.state.name)
            .field("rate", &self.state.rate)
            .field("address", &self.state.address)
            .field("speed", &self.state.speed)
            .finish_non_exhaustive()
    }
}

/// Everything the config callback and the client interface share.
struct Mpu9250State {
    name: String,
    oid: u8,
    rate: i64,
    address: u8,
    speed: u32,
    /// Read by [`convert_samples`], the chip's decode helper.
    axes_map: AxesMap,
    printer: Weak<Printer>,
    i2c: Arc<McuI2c>,
    /// The MCU the I2C bus is on (the reader's data binding needs it).
    mcu_object: Arc<McuObject>,
    /// The reader: 6-byte `">hhh"` samples, `query_mpu9250_status` queries.
    ffreader: FixedFreqReader<[i32; SAMPLE_FIELDS]>,
    /// The MPU-family chips flag no per-sample errors, so this stays zero
    /// (`last_error_count`, kept for the batch message's shape).
    last_error_count: Mutex<u64>,
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Mpu9250State {
    /// The build-time half: the chip's config command, its armed-at-restart
    /// disarmed query, and the reader's data binding (`_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let i2c_oid = self.i2c.oid()?;
        builder.add_config_cmd(&ConfigMpu9250 {
            oid: self.oid,
            i2c_oid,
        })?;
        // Upstream: `query_mpu9250 oid rest_ticks=0` with `on_restart=True`.
        builder.add_restart_cmd(&QueryMpu9250 {
            oid: self.oid,
            rest_ticks: 0,
        })?;
        self.ffreader.bind(mcu, &self.mcu_object, self.oid)?;
        Ok(())
    }

    /// Read one register (`read_reg`), or fail when the read is short.
    async fn read_reg(&self, reg: u8) -> Result<u8, String> {
        let response = self
            .i2c
            .transfer(&[reg], 1)
            .await
            .map_err(|err| err.to_string())?;
        response.first().copied().ok_or_else(|| {
            format!(
                "mpu9250 register 0x{reg:02x}: short read ({} byte(s))",
                response.len()
            )
        })
    }

    /// Write one register (`set_reg`; upstream does not read it back).
    async fn set_reg(&self, reg: u8, value: u8) -> Result<(), String> {
        self.i2c
            .write(&[reg, value])
            .await
            .map_err(|err| err.to_string())
    }

    /// The connected MCU, or why the sensor cannot reach it.
    fn connected_mcu(&self) -> Result<Arc<Mcu>, String> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| "mpu9250's MCU is not connected".to_string())
    }

    /// Wake the chip, configure it, and arm the bulk stream
    /// (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        // In case of miswiring, the device id tells a real chip from noise.
        let dev_id = self.read_reg(REG_DEVID).await?;
        let chip = device_name(dev_id).ok_or_else(|| {
            format!(
                "Invalid mpu id (got {dev_id:x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty chip."
            )
        })?;
        tracing::info!("Found {chip} with id {dev_id:x}");

        for (reg, value) in setup_writes() {
            self.set_reg(reg, value).await?;
        }
        // Dummy read to ensure the queues are flushed, then pause for the
        // accelerometer to wake. Upstream gates the rate write on an MCU clock
        // 20 ms in the future instead; the pause is the same wait.
        self.read_reg(REG_DEVID).await?;
        tokio::time::sleep(Duration::from_millis(20)).await;
        for (reg, value) in config_writes() {
            self.set_reg(reg, value).await?;
        }
        // Clear the FIFO overflow flag.
        self.read_reg(REG_INT_STATUS).await?;

        // Start bulk reading: one report every four samples.
        let mcu = self.connected_mcu()?;
        let rest_ticks = mcu
            .seconds_to_clock(4. / self.rate as f64)
            .map_err(|err| err.to_string())? as u32;
        mcu.send_msg(&QueryMpu9250 {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(|err| err.to_string())?;
        self.set_reg(REG_FIFO_EN, SET_ENABLE_FIFO).await?;

        tracing::info!("MPU9250 starting '{}' measurements", self.name);
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

    /// Halt the bulk stream and put the chip back to sleep
    /// (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        self.set_reg(REG_FIFO_EN, SET_DISABLE_FIFO).await?;
        let mcu = self.connected_mcu()?;
        mcu.send_msg(&QueryMpu9250 {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(|err| err.to_string())?;
        self.ffreader.note_end();
        tracing::info!("MPU9250 finished '{}' measurements", self.name);
        for (reg, value) in shutdown_writes() {
            self.set_reg(reg, value).await?;
        }
        Ok(())
    }

    /// One batch: pull, convert, report (`_process_batch`).
    async fn process_batch(&self, _eventtime: f64) -> Result<Option<Value>, String> {
        let raw = self
            .ffreader
            .pull_samples()
            .await
            .map_err(|err| err.to_string())?;
        let data = convert_samples(&self.axes_map, &raw);
        if data.is_empty() {
            return Ok(None);
        }
        let errors = *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let overflows = self.ffreader.get_last_overflows();
        Ok(Some(json!({
            "data": data,
            "errors": errors,
            "overflows": overflows,
        })))
    }
}

/// Upstream's `load_config` for `[mpu9250]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Mpu9250::new(config, printer)?))
}

/// Upstream's `load_config_prefix` for `[mpu9250 <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Mpu9250::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::webhooks;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("mpu9250", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A section and a tracked wrapper for it, leaked so the borrow outlives
    /// the call (tests only).
    fn wrap(name: &str, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
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
        printer
    }

    #[test]
    fn test_a_named_section_opens_the_dump_endpoint_under_its_name() {
        let printer = printer();

        let mpu = Mpu9250::new(&wrap("my_mpu", &[]), &printer).unwrap();

        assert_eq!(mpu.name(), "my_mpu");
        let registrations = webhooks::install(&printer).unwrap().take_mux_endpoints();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].path, DUMP_ENDPOINT);
        assert_eq!(registrations[0].key, DUMP_KEY);
        assert_eq!(registrations[0].value.as_deref(), Some("my_mpu"));
    }

    #[test]
    fn test_a_bare_section_is_named_after_its_section_id() {
        let printer = printer();

        let mut bare = ConfigSection::new("mpu9250", None);
        bare.parameters.clear();
        let config = ConfigWrapper::new(Box::leak(Box::new(bare)), AccessTracking::shared());
        let mpu = Mpu9250::new(&config, &printer).unwrap();

        assert_eq!(mpu.name(), "mpu9250");
    }

    #[test]
    fn test_the_defaults_are_the_upstream_ones() {
        let printer = printer();

        let mpu = Mpu9250::new(&wrap("my_mpu", &[]), &printer).unwrap();

        assert_eq!(mpu.data_rate(), 4000);
        assert_eq!(mpu.i2c_address(), 0x68);
        assert_eq!(mpu.i2c_speed(), 400_000);
    }

    #[test]
    fn test_the_default_options_are_recorded_as_read() {
        let printer = printer();
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(
            Box::leak(Box::new(section("my_mpu", &[]))),
            Arc::clone(&access),
        );

        Mpu9250::new(&config, &printer).unwrap();

        for option in ["rate", "i2c_address", "i2c_speed"] {
            assert!(
                access.contains("mpu9250 my_mpu", option),
                "option {option} recorded"
            );
        }
    }

    #[test]
    fn test_an_unsupported_rate_is_refused() {
        let printer = printer();

        let err = Mpu9250::new(&wrap("my_mpu", &[("rate", "3200")]), &printer).unwrap_err();

        assert_eq!(err.to_string(), "Invalid rate parameter: 3200");
    }

    #[test]
    fn test_an_address_out_of_range_is_refused() {
        let printer = printer();

        let err = Mpu9250::new(&wrap("my_mpu", &[("i2c_address", "128")]), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'mpu9250 my_mpu' must have maximum of 127"
        );
    }

    #[test]
    fn test_the_default_axes_map_is_the_identity() {
        let map = read_axes_map(&wrap("my_mpu", &[]), SCALE).unwrap();

        assert_eq!(map, [(0, SCALE), (1, SCALE), (2, SCALE)]);
    }

    #[test]
    fn test_an_invalid_axes_map_is_refused() {
        let err = read_axes_map(&wrap("my_mpu", &[("axes_map", "x,q,z")]), SCALE).unwrap_err();

        assert_eq!(err.to_string(), "Invalid axes_map parameter");
    }

    #[test]
    fn test_an_axes_map_shorter_than_three_is_refused() {
        let err = read_axes_map(&wrap("my_mpu", &[("axes_map", "x,y")]), SCALE).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'axes_map' in section 'mpu9250 my_mpu' must have 3 elements"
        );
    }

    #[test]
    fn test_convert_samples_maps_axes_and_scales() {
        // X reads raw Z, Y reads raw X negated, Z reads raw Y — all at SCALE.
        let map = [(2, SCALE), (0, -SCALE), (1, SCALE)];

        let samples = convert_samples(&map, &[(1.000_000_49, [100, 200, 300])]);

        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0][0], 1.000_000, "time rounded to 6 digits");
        assert_eq!(samples[0][1], round(300.0 * SCALE, 6));
        assert_eq!(samples[0][2], round(-100.0 * SCALE, 6));
        assert_eq!(samples[0][3], round(200.0 * SCALE, 6));
    }

    #[test]
    fn test_the_register_sequence_is_upstreams() {
        // Wake, accelerometer axes on; then rate, FIFO stream mode and the
        // accelerometer's bandwidth/full scale; then empty and enable the FIFO.
        assert_eq!(setup_writes(), [(0x6b, 0x00), (0x6c, 0x07)]);
        assert_eq!(
            config_writes(),
            [
                (0x19, 0x00), // SMPLRT_DIV: 4 kHz
                (0x1a, 0x01), // CONFIG: FIFO stream mode
                (0x1c, 0x10), // ACCEL_CONFIG: 8g full scale
                (0x1d, 0x08), // ACCEL_CONFIG2: 1046 Hz bandwidth
                (0x23, 0x00), // FIFO_EN: disabled while the FIFO is reset
                (0x6a, 0x04), // USER_CTRL: reset the FIFO
                (0x6a, 0x40), // USER_CTRL: FIFO enabled
            ]
        );
        assert_eq!(shutdown_writes(), [(0x6b, 0x40), (0x6c, 0x3f)]);

        // The ids upstream accepts, both defective ones included.
        assert_eq!(device_name(0x71), Some("mpu-9250"));
        assert_eq!(device_name(0x70), Some("mpu-6500"));
        assert_eq!(device_name(0x68), Some("mpu-6050"));
        assert!(device_name(0x75).expect("a known id").contains("DEFECTIVE"));
        assert_eq!(device_name(0x00), None);
    }

    #[test]
    fn test_convert_samples_of_nothing_is_empty() {
        let map = [(0, SCALE), (1, SCALE), (2, SCALE)];

        assert!(convert_samples(&map, &[]).is_empty());
    }

    #[test]
    fn test_a_software_bus_with_one_pin_is_refused() {
        let printer = printer();

        let err = Mpu9250::new(
            &wrap("my_mpu", &[("i2c_software_scl_pin", "PA0")]),
            &printer,
        )
        .unwrap_err();

        assert!(
            err.to_string().contains("both 'i2c_software_scl_pin'"),
            "{err}"
        );
    }

    #[test]
    fn test_software_pins_must_share_the_sensor_mcu() {
        let printer = printer();
        // Register a second MCU so a pin can name a different chip.
        let other = McuObject::new(ConfigSection::new("mcu", Some("other")), &printer).unwrap();
        printer.add_object("mcu other", Arc::new(other)).unwrap();

        let err = Mpu9250::new(
            &wrap(
                "my_mpu",
                &[
                    ("i2c_software_scl_pin", "PA0"),
                    ("i2c_software_sda_pin", "other:PA1"),
                ],
            ),
            &printer,
        )
        .unwrap_err();

        assert!(
            err.to_string().contains("must be on the same mcu 'mcu'"),
            "{err}"
        );
    }

    #[test]
    fn test_the_query_helper_filters_to_the_request_window() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let helper = AccelQueryHelper::new(Arc::downgrade(&printer));
        // No `toolhead` in this printer, so seed the window directly.
        helper.force_request_window(0.5, 1.5);

        assert!(helper.handle_batch(&json!({
            "data": [
                [0.0, 1.0, 2.0, 3.0],
                [0.6, 4.0, 5.0, 6.0],
                [1.4, 7.0, 8.0, 9.0],
                [2.0, 10.0, 11.0, 12.0]
            ]
        })));
        assert!(helper.has_valid_samples());

        let samples = helper.get_samples();
        assert_eq!(samples.len(), 2, "only the in-window rows: {samples:?}");
        assert_eq!(
            samples[0],
            AccelMeasurement {
                time: 0.6,
                accel_x: 4.0,
                accel_y: 5.0,
                accel_z: 6.0
            }
        );
        assert_eq!(samples[1].accel_x, 7.0);

        // A finished helper detaches from the batch loop.
        helper.finish_measurements();
        assert!(!helper.handle_batch(&json!({ "data": [] })));
    }

    #[test]
    fn test_the_query_helper_reports_no_valid_samples_outside_the_window() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let helper = AccelQueryHelper::new(Arc::downgrade(&printer));
        helper.force_request_window(0.5, 1.5);

        // Entirely before and entirely after the window.
        helper.handle_batch(&json!({ "data": [[0.0, 1.0, 2.0, 3.0]] }));
        helper.handle_batch(&json!({ "data": [[2.0, 1.0, 2.0, 3.0]] }));

        assert!(!helper.has_valid_samples());
        assert!(helper.get_samples().is_empty());
    }

    #[test]
    fn test_the_loader_registers_the_section_under_its_identifier() {
        // The end-to-end shape criterion 1 asks for: the real loader claims
        // `[mpu9250 my_mpu]` and registers the object under the section's
        // identifier, so a consumer (`resonance_tester`) finds it by name.
        use crate::core::klippy::config::Config;
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = Config::from_text("[mcu]\nserial: /dev/a\n[mpu9250 my_mpu]\n")
            .expect("the test config parses")
            .0;

        printer.load_config(&config).unwrap();

        assert!(printer
            .lookup_object_as::<Mpu9250>("mpu9250 my_mpu")
            .is_some());
    }

    #[tokio::test]
    async fn test_starting_a_measurement_registers_a_batch_client() {
        // The interface now drives the real path: asking for a client subscribes
        // it to the batch stream (whose first client runs the chip's start
        // callback), and nothing has arrived yet.
        let printer = printer();
        let mpu = Mpu9250::new(&wrap("my_mpu", &[]), &printer).unwrap();

        let client = mpu.start_internal_client();

        assert_eq!(mpu.client_count(), 1);
        assert!(client.get_samples().is_empty());
    }
}
