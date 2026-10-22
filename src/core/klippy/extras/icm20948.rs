//! `[icm20948 <name>]` — the ICM20948 accelerometer (upstream's
//! `klippy/extras/icm20948.py`).
//!
//! An ICM20948 on an I2C bus, read the same way `[mpu9250]` is: the section
//! takes the I2C mechanism options (`i2c_address` / `i2c_speed` / `i2c_mcu` /
//! `i2c_bus` / the software-pin pair), asks its MCU for an [`McuI2c`] (the same
//! resource `[i2c_device]` uses), and reserves an oid for the firmware's
//! `config_icm20948`.
//!
//! | piece | upstream |
//! |---|---|
//! | [`Icm20948`] | `ICM20948.__init__` |
//! | [`read_axes_map`] | `adxl345.read_axes_map` (called with `SCALE` three times) |
//! | [`convert_samples`] | `ICM20948._convert_samples` |
//! | [`setup_writes`] / [`config_writes`] / [`shutdown_writes`] | `_start_measurements` / `_finish_measurements` register sequences |
//! | [`AccelQueryHelper`] | `adxl345.AccelQueryHelper` |
//! | `icm20948/dump_icm20948` mux endpoint (key `sensor`) | `batch_bulk.add_mux_endpoint(…)` |
//! | [`ConfigIcm20948`] / [`QueryIcm20948`] | `config_icm20948` / `query_icm20948` (in [`crate::core::klippy::cmd::icm20948`]) |
//!
//! # Accelerometer only
//!
//! Upstream's `icm20948.py` reads the accelerometer and nothing else: the FIFO
//! entries it configures hold the three accel axes, `_convert_samples` unpacks
//! exactly those three words, and the dump endpoint's header is the
//! accelerometer's four columns. There is no gyro path in upstream: the chip is
//! left with upstream's `SET_PWR_MGMT_2_ACCEL_ON` write (`0x07`, the
//! accelerometer side selected), the FIFO is only ever read as accel entries,
//! and `resonance_tester`, the consumer of this interface, measures
//! acceleration only. This port adds no gyro stream of its own.
//!
//! # The accelerometer interface
//!
//! [`Icm20948::start_internal_client`] returns an [`AccelQueryHelper`], the
//! shape upstream's `ICM20948.start_internal_client` returns; the helper
//! subscribes to the shared batch stream the dump endpoint also fans out.
//!
//! # The sample path
//!
//! Samples flow through the shared
//! [`FixedFreqReader`](crate::core::klippy::extras::bulk_sensor::FixedFreqReader)
//! parameterized with the chip's 6-byte `">hhh"` layout — three signed 16-bit
//! words per FIFO entry, 51 // 6 = 8 entries per bulk report (the firmware
//! reads `BYTES_PER_BLOCK` = 48 bytes per `sensor_bulk_data` message) — and the
//! `query_icm20948_status` clock query. The register sequence
//! ([`setup_writes`], [`config_writes`], [`shutdown_writes`]) and
//! `_convert_samples` ([`convert_samples`]) follow upstream.
//!
//! One batch message carries `data` rows of `[time, x, y, z]` in mm/s², an
//! `errors` count (the chip flags no per-sample errors, so it stays zero) and
//! `overflows`.
//!
//! The G-Code commands upstream registers through `adxl345.AccelCommandHelper`
//! (`ACCELEROMETER_MEASURE` / `ACCELEROMETER_QUERY` / `ACCELEROMETER_DEBUG_READ`
//! / `ACCELEROMETER_DEBUG_WRITE`) are not registered by this section, same as
//! `[adxl345]` and `[mpu9250]` here.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::icm20948::{
    ConfigIcm20948, QueryIcm20948, QUERY_ICM20948_STATUS_MSGFORMAT,
};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::adxl345::{read_axes_map, AccelQueryHelper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, FixedFreqReader, LifecycleCb,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{ConfigBuilder, I2cMode, Mcu, McuError, McuI2c, McuObject};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Both `[icm20948]` and `[icm20948 <name>]` are valid upstream. Loaded with the
// other I2C consumers (order 40), after `[board_pins]` (30), because a software
// bus may name its pins through an alias.
section!(
    "icm20948",
    order = 40,
    load = load_config,
    prefix = load_config_prefix
);

/// The path the dump endpoint registers (`icm20948.py`).
pub const DUMP_ENDPOINT: &str = "icm20948/dump_icm20948";

/// The mux key that selects a sensor instance.
pub const DUMP_KEY: &str = "sensor";

/// The default 7-bit device address (`ICM20948_ADDR`).
pub const DEFAULT_ADDR: i64 = 0x68;

/// The default I2C clock (`default_speed` at the `MCU_I2C_from_config` call).
pub const DEFAULT_SPEED: i64 = 400_000;

/// The lowest clock `MCU_I2C_from_config` accepts (`bus.py`'s `minval`).
const MIN_SPEED: i64 = 100_000;

/// The one sample rate the chip's register map offers (`SAMPLE_RATE_DIVS`).
pub const SUPPORTED_RATE: i64 = 4500;

/// Earth gravity in mm/s² (`FREEFALL_ACCEL`).
const FREEFALL_ACCEL: f64 = 9.80665 * 1000.;

/// 1/2048 g/LSB at the 16g full scale, in mm/s² (`SCALE`).
pub const SCALE: f64 = 0.00048828125 * FREEFALL_ACCEL;

/// The report header a connecting dump client receives
/// (`('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')`).
pub const DUMP_HEADER: [&str; 4] = ["time", "x_acceleration", "y_acceleration", "z_acceleration"];

/// Seconds between batch callbacks and the smoothing span's unit (upstream
/// `icm20948.BATCH_UPDATES`).
const ICM_BATCH_UPDATES: f64 = 0.100;

/// The three signed 16-bit fields one sample unpacks (upstream's `">hhh"`).
const SAMPLE_FIELDS: usize = 3;

// The chip's register map (`icm20948.py`'s `REG_*`).
const REG_DEVID: u8 = 0x00;
const REG_FIFO_EN: u8 = 0x67;
const REG_ACCEL_SMPLRT_DIV1: u8 = 0x10;
const REG_ACCEL_SMPLRT_DIV2: u8 = 0x11;
const REG_ACCEL_CONFIG: u8 = 0x14;
const REG_USER_CTRL: u8 = 0x03;
const REG_PWR_MGMT_1: u8 = 0x06;
const REG_PWR_MGMT_2: u8 = 0x07;
const REG_INT_STATUS: u8 = 0x19;
const REG_BANK_SEL: u8 = 0x7F;

/// The one accel sample-rate divider the section accepts (`SAMPLE_RATE_DIVS`).
const SMPLRT_DIV_4_5KHZ: u8 = 0x00;

/// Register bank 0 (`SET_BANK_0`) — the default bank, holding `PWR_MGMT_*`,
/// `FIFO_EN` and the sample-rate dividers.
const SET_BANK_0: u8 = 0x00;

/// Register bank 2 (`SET_BANK_2`), which holds `ACCEL_CONFIG`.
const SET_BANK_2: u8 = 0x20;

/// 16g full scale, 1209 Hz bandwidth, 4.5 kHz sample rate (`SET_ACCEL_CONFIG`).
const SET_ACCEL_CONFIG: u8 = 0x06;

/// Leave sleep mode (`SET_PWR_MGMT_1_WAKE`).
const SET_PWR_MGMT_1_WAKE: u8 = 0x01;

/// Back to sleep (`SET_PWR_MGMT_1_SLEEP`).
const SET_PWR_MGMT_1_SLEEP: u8 = 0x41;

/// Upstream's `SET_PWR_MGMT_2_ACCEL_ON` (the accelerometer side selected).
const SET_PWR_MGMT_2_ACCEL_ON: u8 = 0x07;

/// Everything off (`SET_PWR_MGMT_2_OFF`).
const SET_PWR_MGMT_2_OFF: u8 = 0x3F;

/// Reset the FIFO (`SET_USER_FIFO_RESET`).
const SET_USER_FIFO_RESET: u8 = 0x0E;

/// Leave the FIFO enabled (`SET_USER_FIFO_EN`).
const SET_USER_FIFO_EN: u8 = 0x40;

/// Let accelerometer samples into the FIFO (`SET_ENABLE_FIFO`).
const SET_ENABLE_FIFO: u8 = 0x10;

/// Stop samples entering the FIFO (`SET_DISABLE_FIFO`).
const SET_DISABLE_FIFO: u8 = 0x00;

/// The sensor's `axes_map`: for each output axis, which raw component it reads
/// and the sign/scale applied (`adxl345.read_axes_map`'s three triples).
pub type AxesMap = Vec<(usize, f64)>;

/// One converted reading: `[time, x, y, z]`, the `data` row of a dump message.
pub type AccelSample = [f64; 4];

// ===========================================================================
// Pure helpers (unit-tested directly)
// ===========================================================================

/// Python's `round(value, digits)` for the sample timestamps/values.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// The `ACCEL_SMPLRT_DIV` byte for a supported `rate` (`SAMPLE_RATE_DIVS`).
fn sample_rate_div(data_rate: i64) -> Option<u8> {
    if data_rate == SUPPORTED_RATE {
        Some(SMPLRT_DIV_4_5KHZ)
    } else {
        None
    }
}

/// The device names the chip's id byte maps to (`ICM_DEV_IDS`).
fn device_name(dev_id: u8) -> Option<&'static str> {
    Some(match dev_id {
        0xEA => "icm-20948",
        _ => return None,
    })
}

/// The writes `_start_measurements` issues first: wake the chip, then leave
/// the accelerometer side selected (`SET_PWR_MGMT_2_ACCEL_ON`).
fn setup_writes() -> [(u8, u8); 2] {
    [
        (REG_PWR_MGMT_1, SET_PWR_MGMT_1_WAKE),
        (REG_PWR_MGMT_2, SET_PWR_MGMT_2_ACCEL_ON),
    ]
}

/// The writes that set the sample rate, the accelerometer's full scale, and
/// then reset and enable the FIFO (`_start_measurements`, after the dummy
/// device-id read that flushes the queues).
///
/// The full-scale write is in bank 2, so the sequence selects that bank and
/// returns to bank 0 around it. `None` when `data_rate` is not one of
/// [`SUPPORTED_RATE`] — [`Icm20948::new`] refuses those, so the start path only
/// sees this if a section was built some other way.
fn config_writes(data_rate: i64) -> Option<[(u8, u8); 8]> {
    let div = sample_rate_div(data_rate)?;
    Some([
        (REG_ACCEL_SMPLRT_DIV1, div),         // rate divider, MSB
        (REG_ACCEL_SMPLRT_DIV2, div),         // rate divider, LSB
        (REG_BANK_SEL, SET_BANK_2),           // ACCEL_CONFIG lives in bank 2
        (REG_ACCEL_CONFIG, SET_ACCEL_CONFIG), // 16g, 1209 Hz BW
        (REG_BANK_SEL, SET_BANK_0),           // back to the default bank
        (REG_FIFO_EN, SET_DISABLE_FIFO),      // no samples while the FIFO is reset
        (REG_USER_CTRL, SET_USER_FIFO_RESET),
        (REG_USER_CTRL, SET_USER_FIFO_EN),
    ])
}

/// The writes `_finish_measurements` issues once the bulk query is stopped.
fn shutdown_writes() -> [(u8, u8); 2] {
    [
        (REG_PWR_MGMT_1, SET_PWR_MGMT_1_SLEEP),
        (REG_PWR_MGMT_2, SET_PWR_MGMT_2_OFF),
    ]
}

/// Map raw sensor counts to scaled acceleration (`ICM20948._convert_samples`).
///
/// `raw` is one `(print time, [x, y, z])` sample per row — the chip's `">hhh"`
/// wire format, signed 16-bit components — and the result is the dump message's
/// `[time, x, y, z]` rows with upstream's 6-digit rounding. The chip flags no
/// per-sample errors, so every row converts.
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

// ===========================================================================
// The sensor
// ===========================================================================

/// One configured ICM20948 accelerometer.
pub struct Icm20948 {
    state: Arc<Icm20948State>,
}

impl Icm20948 {
    /// Read the section and wire the sensor up (`ICM20948.__init__`).
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
        let axes_map = read_axes_map(config, SCALE, SCALE, SCALE)?;

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

        let state = Arc::new(Icm20948State {
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
                rate as f64 * ICM_BATCH_UPDATES * 2.,
                ">hhh",
                QUERY_ICM20948_STATUS_MSGFORMAT,
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            last_error_count: Mutex::new(0),
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the I2C resource's own callback so the
        // device oid exists when `config_icm20948` is added (the resource
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
                    .ok_or_else(|| "icm20948 is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "icm20948 is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "icm20948 is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, ICM_BATCH_UPDATES);
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

    /// The `axes_map` the section configured, as `(raw index, scale)` triples.
    pub fn axes_map(&self) -> &AxesMap {
        &self.state.axes_map
    }

    /// Register a measurement client and hand it back
    /// (`ICM20948.start_internal_client`).
    ///
    /// Registering it starts the chip's stream (the helper's first client runs
    /// `_start_measurements`); the caller ends the window with
    /// [`AccelQueryHelper::finish_measurements`], which detaches the client and
    /// runs the chip's own `_finish_measurements`.
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

impl PrinterObject for Icm20948 {
    /// Never reached through the API: [`PrinterObject::is_queryable`] is false,
    /// as upstream's object defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Icm20948 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Icm20948")
            .field("name", &self.state.name)
            .field("rate", &self.state.rate)
            .field("address", &self.state.address)
            .field("speed", &self.state.speed)
            .finish_non_exhaustive()
    }
}

/// Everything the config callback and the client interface share.
struct Icm20948State {
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
    /// The reader: 6-byte `">hhh"` samples, `query_icm20948_status` queries.
    ffreader: FixedFreqReader<[i32; SAMPLE_FIELDS]>,
    /// The chip flags no per-sample errors, so this stays zero
    /// (`last_error_count`, kept for the batch message's shape).
    last_error_count: Mutex<u64>,
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Icm20948State {
    /// The build-time half: the chip's config command, its armed-at-restart
    /// disarmed query, and the reader's data binding (`_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let i2c_oid = self.i2c.oid()?;
        builder.add_config_cmd(&ConfigIcm20948 {
            oid: self.oid,
            i2c_oid,
        })?;
        // Upstream: `query_icm20948 oid rest_ticks=0` with `on_restart=True`.
        builder.add_restart_cmd(&QueryIcm20948 {
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
                "icm20948 register 0x{reg:02x}: short read ({} byte(s))",
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
            .ok_or_else(|| "icm20948's MCU is not connected".to_string())
    }

    /// Wake the chip, configure it, and arm the bulk stream
    /// (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        // In case of miswiring, the device id tells a real chip from noise.
        let dev_id = self.read_reg(REG_DEVID).await?;
        let chip = device_name(dev_id).ok_or_else(|| {
            format!(
                "Invalid icm20948 id (got {dev_id:x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty chip."
            )
        })?;
        tracing::info!("Found {chip} with id {dev_id:x}");

        for (reg, value) in setup_writes() {
            self.set_reg(reg, value).await?;
        }
        // Dummy read to ensure the queues are flushed. Upstream adds no pause
        // for the accelerometer's wake-up here (`icm20948.py` comments the
        // missing 20 ms on purpose).
        self.read_reg(REG_DEVID).await?;
        let writes = config_writes(self.rate)
            .ok_or_else(|| format!("Invalid rate parameter: {}", self.rate))?;
        for (reg, value) in writes {
            self.set_reg(reg, value).await?;
        }
        // Clear the FIFO overflow flag.
        self.read_reg(REG_INT_STATUS).await?;

        // Start bulk reading: one report every four samples.
        let mcu = self.connected_mcu()?;
        let rest_ticks = mcu
            .seconds_to_clock(4. / self.rate as f64)
            .map_err(|err| err.to_string())? as u32;
        mcu.send_msg(&QueryIcm20948 {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(|err| err.to_string())?;
        self.set_reg(REG_FIFO_EN, SET_ENABLE_FIFO).await?;

        tracing::info!("ICM20948 starting '{}' measurements", self.name);
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
        mcu.send_msg(&QueryIcm20948 {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(|err| err.to_string())?;
        self.ffreader.note_end();
        tracing::info!("ICM20948 finished '{}' measurements", self.name);
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

/// Upstream's `load_config` for `[icm20948]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Icm20948::new(config, printer)?))
}

/// Upstream's `load_config_prefix` for `[icm20948 <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Icm20948::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::protocol::Request;
    use crate::core::klippy::api::registry::{Api, EndpointContext};
    use crate::core::klippy::api::{test_support, webhooks};
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("icm20948", Some(name));
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

    // --- section options and defaults ------------------------------------

    #[test]
    fn test_the_defaults_are_the_upstream_ones() {
        let printer = printer();

        let icm = Icm20948::new(&wrap("my_icm", &[]), &printer).unwrap();

        assert_eq!(icm.name(), "my_icm");
        assert_eq!(icm.data_rate(), 4500);
        assert_eq!(icm.i2c_address(), 0x68);
        assert_eq!(icm.i2c_speed(), 400_000);
        assert_eq!(icm.axes_map(), &vec![(0, SCALE), (1, SCALE), (2, SCALE)]);
    }

    #[test]
    fn test_the_default_options_are_recorded_as_read() {
        let printer = printer();
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(
            Box::leak(Box::new(section("my_icm", &[]))),
            Arc::clone(&access),
        );

        Icm20948::new(&config, &printer).unwrap();

        for option in ["rate", "i2c_address", "i2c_speed"] {
            assert!(
                access.contains("icm20948 my_icm", option),
                "option {option} recorded"
            );
        }
    }

    #[test]
    fn test_the_section_options_are_taken_from_the_config() {
        let printer = printer();

        let icm = Icm20948::new(
            &wrap(
                "my_icm",
                &[
                    ("i2c_address", "105"),
                    ("i2c_speed", "100000"),
                    ("axes_map", "z,-x,y"),
                ],
            ),
            &printer,
        )
        .unwrap();

        assert_eq!(icm.i2c_address(), 0x69);
        assert_eq!(icm.i2c_speed(), 100_000);
        assert_eq!(icm.axes_map(), &vec![(2, SCALE), (0, -SCALE), (1, SCALE)]);
    }

    #[test]
    fn test_an_unsupported_rate_is_refused() {
        let printer = printer();

        let err = Icm20948::new(&wrap("my_icm", &[("rate", "3200")]), &printer).unwrap_err();

        assert_eq!(err.to_string(), "Invalid rate parameter: 3200");
    }

    #[test]
    fn test_an_address_out_of_range_is_refused() {
        let printer = printer();

        let err = Icm20948::new(&wrap("my_icm", &[("i2c_address", "128")]), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'icm20948 my_icm' must have maximum of 127"
        );
    }

    #[test]
    fn test_a_bare_section_is_named_after_its_section_id() {
        let printer = printer();
        let mut bare = ConfigSection::new("icm20948", None);
        bare.parameters.clear();
        let config = ConfigWrapper::new(Box::leak(Box::new(bare)), AccessTracking::shared());

        let icm = Icm20948::new(&config, &printer).unwrap();

        assert_eq!(icm.name(), "icm20948");
    }

    #[test]
    fn test_the_loader_registers_the_section_under_its_identifier() {
        use crate::core::klippy::config::Config;
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = Config::from_text("[mcu]\nserial: /dev/a\n[icm20948 my_icm]\n")
            .expect("the test config parses")
            .0;

        printer.load_config(&config).unwrap();

        assert!(printer
            .lookup_object_as::<Icm20948>("icm20948 my_icm")
            .is_some());
    }

    // --- sample decoding and scaling -------------------------------------

    #[test]
    fn test_convert_samples_maps_axes_and_scales() {
        // X reads raw Z, Y reads raw X negated, Z reads raw Y — all at SCALE.
        let map = vec![(2, SCALE), (0, -SCALE), (1, SCALE)];

        let samples = convert_samples(&map, &[(1.000_000_49, [100, 200, 300])]);

        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0][0], 1.000_000, "time rounded to 6 digits");
        assert_eq!(samples[0][1], round(300.0 * SCALE, 6));
        assert_eq!(samples[0][2], round(-100.0 * SCALE, 6));
        assert_eq!(samples[0][3], round(200.0 * SCALE, 6));
    }

    #[test]
    fn test_the_16g_scale_is_upstreams_one_gravity_lsb() {
        // 1/2048 g/LSB at 16g, in mm/s²: one raw count is half a milli-g.
        assert_eq!(SCALE, 0.00048828125 * 9.80665 * 1000.);
        // One g of raw counts (2048 LSB) converts back to one g.
        let map = vec![(0, SCALE), (1, SCALE), (2, SCALE)];
        let samples = convert_samples(&map, &[(0.5, [2048, -2048, 0])]);
        assert_eq!(samples[0][1], round(9.80665 * 1000., 6));
        assert_eq!(samples[0][2], round(-9.80665 * 1000., 6));
        assert_eq!(samples[0][3], 0.0);
    }

    #[test]
    fn test_convert_samples_of_nothing_is_empty() {
        let map = vec![(0, SCALE), (1, SCALE), (2, SCALE)];

        assert!(convert_samples(&map, &[]).is_empty());
    }

    // --- dump endpoint ----------------------------------------------------

    #[tokio::test]
    async fn test_the_dump_endpoint_is_registered_under_the_sections_name() {
        let printer = printer();

        Icm20948::new(&wrap("my_icm", &[]), &printer).unwrap();

        let registrations = webhooks::install(&printer).unwrap().take_mux_endpoints();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].path, DUMP_ENDPOINT);
        assert_eq!(registrations[0].key, DUMP_KEY);
        assert_eq!(registrations[0].value.as_deref(), Some("my_icm"));
    }

    #[tokio::test]
    async fn test_the_dump_endpoint_greets_a_client_with_the_accel_header() {
        let printer = printer();
        Icm20948::new(&wrap("my_icm", &[]), &printer).unwrap();
        let registrations = webhooks::install(&printer).unwrap().take_mux_endpoints();

        let api = Api::new();
        let request =
            Request::parse(br#"{"method":"icm20948/dump_icm20948","params":{"sensor":"my_icm"}}"#)
                .expect("a valid request");
        let context = EndpointContext {
            api: &api,
            client: test_support::silent_target(),
        };
        let response = registrations[0]
            .handler
            .handle(&request, &context)
            .await
            .expect("the instance takes the client");

        assert_eq!(
            response,
            json!({ "header": ["time", "x_acceleration", "y_acceleration", "z_acceleration"] })
        );
    }

    // --- register setup sequences -----------------------------------------

    #[test]
    fn test_the_register_sequence_is_upstreams() {
        // Wake, then upstream's accelerometer-on write.
        assert_eq!(setup_writes(), [(0x06, 0x01), (0x07, 0x07)]);
        assert_eq!(
            config_writes(4500).expect("the supported rate"),
            [
                (0x10, 0x00), // ACCEL_SMPLRT_DIV1: 4.5 kHz
                (0x11, 0x00), // ACCEL_SMPLRT_DIV2: 4.5 kHz
                (0x7f, 0x20), // BANK_SEL: bank 2, where ACCEL_CONFIG lives
                (0x14, 0x06), // ACCEL_CONFIG: 16g, 1209 Hz bandwidth
                (0x7f, 0x00), // BANK_SEL: back to bank 0
                (0x67, 0x00), // FIFO_EN: disabled while the FIFO is reset
                (0x03, 0x0e), // USER_CTRL: reset the FIFO
                (0x03, 0x40), // USER_CTRL: FIFO enabled
            ]
        );
        // The rate is fixed: nothing else has a divider.
        assert_eq!(config_writes(3200), None);
        assert_eq!(sample_rate_div(4500), Some(0x00));
        assert_eq!(sample_rate_div(4000), None);
        // Back to sleep, everything off.
        assert_eq!(shutdown_writes(), [(0x06, 0x41), (0x07, 0x3f)]);
        // The one id upstream accepts.
        assert_eq!(device_name(0xea), Some("icm-20948"));
        assert_eq!(device_name(0x00), None);
    }

    // --- the accelerometer interface --------------------------------------

    #[tokio::test]
    async fn test_starting_a_measurement_registers_a_batch_client() {
        // Asking for a client subscribes it to the batch stream (whose first
        // client runs the chip's start callback); nothing has arrived yet.
        let printer = printer();
        let icm = Icm20948::new(&wrap("my_icm", &[]), &printer).unwrap();

        let client = icm.start_internal_client();

        assert_eq!(icm.client_count(), 1);
        assert!(client.get_samples().is_empty());
    }

    #[test]
    fn test_a_software_bus_with_one_pin_is_refused() {
        let printer = printer();

        let err = Icm20948::new(
            &wrap("my_icm", &[("i2c_software_scl_pin", "PA0")]),
            &printer,
        )
        .unwrap_err();

        assert!(
            err.to_string().contains("both 'i2c_software_scl_pin'"),
            "{err}"
        );
    }
}
