//! `[lis2dw]` / `[lis3dh]` — the LIS2DW and LIS3DH accelerometers (upstream's
//! `klippy/extras/lis2dw.py`, whose `lis3dh.py` is a two-line reuse of the same
//! class with `LIS3DH_TYPE`).
//!
//! | piece | upstream |
//! |---|---|
//! | [`Lis2dw`] — `cs_pin` selects SPI, its absence I2C; `axes_map`; the chip's fixed rate | `LIS2DW.__init__` |
//! | [`load_config_lis3dh`] / [`load_config_prefix_lis3dh`] | `lis3dh.py`'s `load_config` / `load_config_prefix` |
//! | [`read_axes_map`] / [`AccelQueryHelper`] / [`AccelSample`] | `adxl345.read_axes_map` / `AccelQueryHelper` (reused, as upstream imports them) |
//! | [`Lis2dw::start_internal_client`] | `LIS2DW.start_internal_client` |
//! | [`convert_samples`] | `LIS2DW._convert_samples` |
//! | [`setup_writes`] / [`fifo_arm_write`] / [`disable_i2c_write`] | `_start_measurements`' register traffic |
//! | `lis2dw/dump_lis2dw` mux endpoint (key `sensor`) | `batch_bulk.add_mux_endpoint(…)` |
//! | `config_lis2dw` / `query_lis2dw` / `query_lis2dw_status` | [`crate::core::klippy::cmd::lis2dw`] |
//!
//! # One driver, two section names
//!
//! Upstream's `lis3dh.py` is `lis2dw.LIS2DW(config, lis2dw.LIS3DH_TYPE)` and
//! nothing else, so both sections are built here by [`Lis2dw::new`] with a
//! [`LisChipType`] the factory picks. The type decides the device id, the
//! sample scale, the fixed output data rate, and the register/FIFO layout; the
//! firmware gets it as `config_lis2dw`'s `lis_chip_type`.
//!
//! The section id is the only difference on the host side, and it stops at the
//! mux endpoint: a `[lis3dh]` sensor also streams on `lis2dw/dump_lis2dw`, as
//! upstream's single `add_mux_endpoint` call means it must.
//!
//! The G-Code commands upstream registers through `adxl345.AccelCommandHelper`
//! (`ACCELEROMETER_MEASURE` / `ACCELEROMETER_QUERY` / `ACCELEROMETER_DEBUG_READ`
//! / `ACCELEROMETER_DEBUG_WRITE`) are not registered by this section, exactly as
//! `[adxl345]`/`[mpu9250]` leave them out.
//!
//! # The sample path
//!
//! Samples flow through the shared
//! [`FixedFreqReader`](super::bulk_sensor::FixedFreqReader) parameterized with
//! the chip's 6-byte `"<hhh"` layout — three little-endian signed 16-bit words,
//! 51 // 6 = 8 samples per message, as `src/sensor_lis2dw.c` packs
//! `BYTES_PER_BLOCK` — and the `query_lis2dw_status` clock query. Both chips
//! left-justify their 14-bit (LIS2DW) / 12-bit (LIS3DH) readings in the 16-bit
//! output words, so the sign extension is the wire format's, exactly as
//! upstream's `struct` unpack does it.
//!
//! One batch message carries `data` rows of `[time, x, y, z]`, an `errors` count
//! (zero — neither chip flags a per-sample error, so upstream's
//! `last_error_count` is never incremented) and `overflows`.
//!
//! # Units
//!
//! Upstream's `lis2dw.py` defines its own `FREEFALL_ACCEL = 9.80665` (not
//! `9.80665 * 1000.` as `adxl345.py` and `mpu9250.py` do) because its per-LSB
//! sensitivities are in **mg** rather than g: 1 mg is 9.80665 mm/s². The
//! samples are therefore mm/s², the same unit the other two accelerometers
//! report — 4096 LSB at the LIS2DW's ±4g scale is ~19602 mm/s² (2 g).

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::lis2dw::{
    ConfigLis2dw, LisBusType, LisChipType, QueryLis2dw, QUERY_LIS2DW_STATUS_MSGFORMAT,
};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, FixedFreqReader, LifecycleCb,
};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{ConfigBuilder, I2cMode, Mcu, McuError, McuI2c, McuObject, McuSpi};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::adxl345::{read_axes_map, AccelQueryHelper, AccelSample};

// Loaded like `[adxl345]`/`[mpu9250]` (order 40), after `[board_pins]` (30):
// a `cs_pin`, a software I2C pair, or an alias may name pins.
section!(
    "lis2dw",
    order = 40,
    load = load_config,
    prefix = load_config_prefix
);
// Upstream's `lis3dh` module: a separate section id whose factories build the
// same driver with the other chip type.
section!(
    "lis3dh",
    order = 41,
    load = load_config_lis3dh,
    prefix = load_config_prefix_lis3dh
);

/// The endpoint clients stream samples from (`lis2dw/dump_lis2dw`).
pub const DUMP_ENDPOINT: &str = "lis2dw/dump_lis2dw";

/// The mux key that selects a sensor instance.
pub const DUMP_KEY: &str = "sensor";

/// The columns the dump endpoint's start response advertises
/// (`('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')`).
const DUMP_HEADER: [&str; 4] = ["time", "x_acceleration", "y_acceleration", "z_acceleration"];

/// The SPI mode the section fixes (upstream's `MCU_SPI_from_config(config, 3)`).
const DEFAULT_SPI_MODE: u8 = 3;

/// Default SPI clock in Hz (`default_speed=5000000`).
const DEFAULT_SPI_SPEED: u32 = 5_000_000;

/// The chip's 7-bit I2C address unless the section sets one (`LIS_I2C_ADDR`).
const DEFAULT_I2C_ADDR: i64 = 0x19;

/// Default I2C clock in Hz (`default_speed=400000`).
const DEFAULT_I2C_SPEED: i64 = 400_000;

/// The lowest I2C clock `MCU_I2C_from_config` accepts (`bus.py`'s `minval`).
const MIN_I2C_SPEED: i64 = 100_000;

/// Seconds between batch callbacks and the smoothing span's unit (upstream
/// `lis2dw.BATCH_UPDATES`).
const LIS_BATCH_UPDATES: f64 = 0.100;

/// The fields one sample unpacks (upstream's 6-byte `"<hhh"`).
const SAMPLE_FIELDS: usize = 3;

/// One sample's wire format: three little-endian signed 16-bit words, the
/// chips' left-justified output registers (`src/sensor_lis2dw.c`'s
/// `BYTES_PER_SAMPLE`, upstream's `"<hhh"`).
const SAMPLE_FORMAT: &str = "<hhh";

/// Upstream's gravity factor for the two sensitivities below
/// (`FREEFALL_ACCEL` in `lis2dw.py`).
///
/// It is 9.80665 rather than `adxl345.py`'s `9.80665 * 1000.` because the
/// per-LSB constants folded into it are in mg (1 mg = 9.80665 mm/s²), so the
/// two scales come out in mm/s² per raw LSB — the unit the samples are
/// reported in (`LIS2DW_SCALE` / `LIS3DH_SCALE`).
const FREEFALL_ACCEL: f64 = 9.80665;
const LIS2DW_SCALE: f64 = FREEFALL_ACCEL * 1.952 / 4.;
const LIS3DH_SCALE: f64 = FREEFALL_ACCEL * 11.718 / 16.;

// The chips' register map (`lis2dw.py`'s `REG_*`).
const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL_REG1: u8 = 0x20;
const REG_CTRL_REG2: u8 = 0x21;
const REG_CTRL_REG4: u8 = 0x23;
const REG_CTRL_REG5: u8 = 0x24;
const REG_CTRL_REG6: u8 = 0x25;
const REG_FIFO_CTRL: u8 = 0x2E;

/// The read bit of a register address (`REG_MOD_READ`).
const REG_MOD_READ: u8 = 0x80;

/// The ids a working chip answers with on `WHO_AM_I` (`LIS2DW_DEV_ID` /
/// `LIS3DH_DEV_ID`).
const LIS2DW_DEV_ID: u8 = 0x44;
const LIS3DH_DEV_ID: u8 = 0x33;

// ===========================================================================
// The chip type
// ===========================================================================

/// The id a working chip answers with (`LIS2DW_DEV_ID` / `LIS3DH_DEV_ID`).
fn dev_id(chip: LisChipType) -> u8 {
    match chip {
        LisChipType::Lis2dw => LIS2DW_DEV_ID,
        LisChipType::Lis3dh => LIS3DH_DEV_ID,
    }
}

/// Full-scale sensitivity per raw LSB in mm/s² (`LIS2DW_SCALE` /
/// `LIS3DH_SCALE`).
fn sample_scale(chip: LisChipType) -> f64 {
    match chip {
        LisChipType::Lis2dw => LIS2DW_SCALE,
        LisChipType::Lis3dh => LIS3DH_SCALE,
    }
}

/// The chips' fixed output data rate (`self.data_rate`): 1600 Hz for the
/// LIS2DW, 1344 Hz for the LIS3DH.
fn data_rate(chip: LisChipType) -> u32 {
    match chip {
        LisChipType::Lis2dw => 1600,
        LisChipType::Lis3dh => 1344,
    }
}

// ===========================================================================
// Register traffic and chip setup (unit-tested directly)
// ===========================================================================

/// Python's `round(value, digits)` for the sample timestamps/values.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// The bytes `read_reg` clocks out on SPI: the address with its read bit set
/// plus a dummy byte (`spi_transfer([reg | REG_MOD_READ, 0x00])`). The value is
/// in the reply's second byte.
fn spi_read_frame(reg: u8) -> [u8; 2] {
    [reg | REG_MOD_READ, 0x00]
}

/// The bytes a [`spi_read_frame`] reply carries: the byte after the echoed
/// address, or `None` when the reply is short.
fn read_reg_value(response: &[u8]) -> Option<u8> {
    response.get(1).copied()
}

/// The bytes `set_reg` shifts out — the plain address and the value, on SPI and
/// I2C alike (`spi_send([reg, val & 0xFF])` / `i2c_write([reg, val & 0xFF])`).
fn write_frame(reg: u8, value: u8) -> [u8; 2] {
    [reg, value]
}

/// The register writes `_start_measurements` performs before the bulk stream is
/// armed, in order.
///
/// LIS2DW: ODR/2 with the low-pass filter and low noise on, continuous FIFO
/// mode, then high-performance 1600 Hz mode. LIS3DH: 1344 Hz high-resolution
/// with all axes, no filtering, ±16g high resolution, FIFO enabled, stream
/// mode. The LIS2DW's extra I2C-disable write on SPI is
/// [`disable_i2c_write`], issued before these.
fn setup_writes(chip: LisChipType) -> &'static [(u8, u8)] {
    match chip {
        LisChipType::Lis2dw => &[
            // ODR/2, +-16g, low-pass filter, low noise enabled.
            (REG_CTRL_REG6, 0x34),
            // Continuous mode: a new sample overwrites the oldest when full.
            (REG_FIFO_CTRL, 0xC0),
            // High-performance / 1600 Hz, high-performance mode (14-bit).
            (REG_CTRL_REG1, 0x94),
        ],
        LisChipType::Lis3dh => &[
            // High resolution / 1344 Hz, high resolution mode, XYZ enabled.
            (REG_CTRL_REG1, 0x97),
            // Disable all filtering.
            (REG_CTRL_REG2, 0x00),
            // +-16g, high resolution mode.
            (REG_CTRL_REG4, 0x38),
            // Enable FIFO.
            (REG_CTRL_REG5, 0x40),
            // Stream mode.
            (REG_FIFO_CTRL, 0x80),
        ],
    }
}

/// The LIS2DW's SPI-only write: disable the chip's I2C interface
/// (`set_reg(REG_LIS2DW_CTRL_REG2_ADDR, 0x06)`).
fn disable_i2c_write() -> (u8, u8) {
    (REG_CTRL_REG2, 0x06)
}

/// The FIFO write issued once the bulk query is armed (`_start_measurements`'s
/// "start bulk reading" tail): continuous mode for the LIS2DW, stream mode for
/// the LIS3DH. `_finish_measurements` clears the same register.
fn fifo_arm_write(chip: LisChipType) -> (u8, u8) {
    match chip {
        LisChipType::Lis2dw => (REG_FIFO_CTRL, 0xC0),
        LisChipType::Lis3dh => (REG_FIFO_CTRL, 0x80),
    }
}

/// The register write that stops the FIFO (`set_reg(REG_LIS2DW_FIFO_CTRL, 0x00)`).
fn fifo_off_write() -> (u8, u8) {
    (REG_FIFO_CTRL, 0x00)
}

/// Map raw counts to scaled acceleration (`LIS2DW._convert_samples`).
///
/// `raw` is one `(print time, [x, y, z])` sample per row — the chip's
/// `"<hhh"` wire format, already sign-extended — and the result is the dump
/// message's `[time, x, y, z]` rows with upstream's 6-digit rounding.
/// `axes_map` gives each output axis its source component and signed scale.
pub fn convert_samples(
    axes_map: &[(usize, f64)],
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

/// The bus the chip hangs off, with the register traffic each one needs
/// (`LIS2DW.read_reg` / `set_reg`).
enum LisBus {
    /// An SPI device; reads clock the address out and take the next byte.
    Spi(Arc<McuSpi>),
    /// An I2C device; reads ask for one byte after the register address.
    I2c(Arc<McuI2c>),
}

impl LisBus {
    /// Which `bus_oid_type` the firmware should look `bus_oid` up as.
    fn bus_type(&self) -> LisBusType {
        match self {
            Self::Spi(_) => LisBusType::Spi,
            Self::I2c(_) => LisBusType::I2c,
        }
    }

    /// The bus device's oid, as `config_lis2dw` names it.
    fn oid(&self) -> Result<u8, McuError> {
        match self {
            Self::Spi(spi) => spi.oid(),
            Self::I2c(i2c) => i2c.oid(),
        }
    }

    /// Read one register (`read_reg`), or fail when the read is short.
    async fn read_reg(&self, reg: u8) -> Result<u8, String> {
        match self {
            Self::Spi(spi) => {
                let response = spi
                    .transfer(&spi_read_frame(reg))
                    .await
                    .map_err(|err| err.to_string())?;
                read_reg_value(&response).ok_or_else(|| {
                    format!(
                        "lis2dw register 0x{reg:02x}: short read ({} byte(s))",
                        response.len()
                    )
                })
            }
            Self::I2c(i2c) => {
                let response = i2c
                    .transfer(&[reg], 1)
                    .await
                    .map_err(|err| err.to_string())?;
                response.first().copied().ok_or_else(|| {
                    format!(
                        "lis2dw register 0x{reg:02x}: short read ({} byte(s))",
                        response.len()
                    )
                })
            }
        }
    }

    /// Write one register and read it back (`set_reg`): a mismatch means the
    /// chip did not take the setting.
    async fn set_reg(&self, reg: u8, value: u8, chip_name: &str) -> Result<(), String> {
        match self {
            Self::Spi(spi) => spi
                .send(&write_frame(reg, value))
                .map_err(|err| err.to_string())?,
            Self::I2c(i2c) => i2c
                .write(&write_frame(reg, value))
                .await
                .map_err(|err| err.to_string())?,
        }
        let stored = self.read_reg(reg).await?;
        if stored != value {
            return Err(format!(
                "Failed to set {chip_name} register [0x{reg:x}] to 0x{value:x}: got 0x{stored:x}. \
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty {chip_name} chip."
            ));
        }
        Ok(())
    }
}

/// One configured LIS2DW or LIS3DH (a `[lis2dw]`/`[lis3dh]` section, bare or
/// named).
pub struct Lis2dw {
    state: Arc<Lis2dwState>,
}

impl Lis2dw {
    /// Read the section and wire the chip up (`LIS2DW.__init__`).
    ///
    /// `chip` is the type the section's factory passes (upstream's
    /// `LIS2DW(config, LIS2DW_TYPE|LIS3DH_TYPE)`); everything else comes from
    /// the options.
    ///
    /// # Errors
    /// A bad `axes_map`, a missing MCU/CS pin/bus option, an oid shortage, or a
    /// dump endpoint already registered under this name.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        chip: LisChipType,
    ) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| identifier.clone());
        let scale = sample_scale(chip);
        let axes_map = read_axes_map(config, scale, scale, scale)?;
        let data_rate = data_rate(chip);

        // SPI when the section names a chip select pin, I2C otherwise
        // (`if config.get('cs_pin', None) is not None`).
        let (bus, mcu_object) = if config.get_str("cs_pin").is_some() {
            let setup = mcu_spi_from_config(
                config,
                printer,
                DEFAULT_SPI_MODE,
                "cs_pin",
                DEFAULT_SPI_SPEED,
            )?;
            let mcu_name = config
                .get_str("spi_mcu")
                .map(|text| text.trim().to_string())
                .unwrap_or_else(|| "mcu".to_string());
            let mcu_object = lookup_mcu(printer, &mcu_name, &identifier)?;
            (LisBus::Spi(setup.device), mcu_object)
        } else {
            let (i2c, mcu_object) = setup_i2c(config, printer)?;
            (LisBus::I2c(i2c), mcu_object)
        };

        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(Lis2dwState {
            name,
            chip,
            axes_map,
            data_rate,
            oid,
            bus,
            mcu_object: Arc::clone(&mcu_object),
            // Samples per second the clock regression expects, smoothed over
            // two batch windows (`chip_smooth = self.data_rate * BATCH_UPDATES * 2`).
            ffreader: FixedFreqReader::with_payload_format(
                f64::from(data_rate) * LIS_BATCH_UPDATES * 2.,
                SAMPLE_FORMAT,
                QUERY_LIS2DW_STATUS_MSGFORMAT,
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            printer: Arc::downgrade(printer),
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the bus resource's own callback: the
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
                    .ok_or_else(|| "lis2dw is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "lis2dw is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "lis2dw is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, LIS_BATCH_UPDATES);
        batch_bulk.add_mux_endpoint(
            DUMP_ENDPOINT,
            DUMP_KEY,
            &state.name,
            json!({ "header": DUMP_HEADER }),
        )?;
        *state.batch_bulk.lock().unwrap_or_else(|p| p.into_inner()) = Some(batch_bulk);

        Ok(Self { state })
    }

    /// Open a measurement client (`LIS2DW.start_internal_client`).
    ///
    /// The same client `[adxl345]`/`[mpu9250]` hand out: registering it starts
    /// the chip's stream (the helper's first client runs
    /// `_start_measurements`), and returning `false` from
    /// [`AccelQueryHelper::handle_batch`] — which
    /// [`AccelQueryHelper::finish_measurements`] arranges — stops it.
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

    /// Which chip this section is driving.
    pub fn chip(&self) -> LisChipType {
        self.state.chip
    }

    /// The chip's fixed output data rate (`data_rate`).
    pub fn data_rate(&self) -> u32 {
        self.state.data_rate
    }

    /// The parsed `axes_map` (`(source axis, signed scale)` triples).
    pub fn axes_map(&self) -> &[(usize, f64)] {
        &self.state.axes_map
    }

    /// Whether the section picked SPI or I2C (`bus_oid_type`).
    pub fn bus_type(&self) -> LisBusType {
        self.state.bus.bus_type()
    }
}

impl PrinterObject for Lis2dw {
    /// Never called through the API: [`PrinterObject::is_queryable`] is false
    /// (upstream's `LIS2DW` defines no `get_status`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Lis2dw {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Lis2dw")
            .field("name", &self.state.name)
            .field("chip", &self.state.chip)
            .field("data_rate", &self.state.data_rate)
            .field("bus_type", &self.state.bus.bus_type())
            .field("axes_map", &self.state.axes_map)
            .finish_non_exhaustive()
    }
}

/// Everything the callbacks and the client interface share.
struct Lis2dwState {
    /// The sensor's name (the section's sub, or the section id).
    name: String,
    /// The chip type the section selected.
    chip: LisChipType,
    /// `(source axis, signed scale)` per measured axis.
    axes_map: Vec<(usize, f64)>,
    /// Samples per second the chip is configured for.
    data_rate: u32,
    /// The oid the firmware assigned.
    oid: u8,
    /// The bus the chip is on, shared with the register traffic.
    bus: LisBus,
    /// The MCU the chip is on (the bus device's).
    mcu_object: Arc<McuObject>,
    /// The reader: 6-byte `"<hhh"` samples, `query_lis2dw_status` queries.
    ffreader: FixedFreqReader<[i32; SAMPLE_FIELDS]>,
    /// The machine, for [`Lis2dw::start_internal_client`].
    printer: Weak<Printer>,
    /// The batch helper that owns the mux endpoint.
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Lis2dwState {
    /// The build-time half: the chip's config command, its disarmed-at-restart
    /// query, and the reader's data binding (`_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        builder.add_config_cmd(&ConfigLis2dw {
            oid: self.oid,
            bus_oid: self.bus.oid()?,
            bus_type: self.bus.bus_type(),
            chip_type: self.chip,
        })?;
        // Upstream arms nothing at config time: `query_lis2dw` is added
        // `on_restart`, so a firmware restart leaves the chip disarmed.
        builder.add_restart_cmd(&QueryLis2dw {
            oid: self.oid,
            rest_ticks: 0,
        })?;
        self.ffreader.bind(mcu, &self.mcu_object, self.oid)?;
        Ok(())
    }

    /// The connected MCU, or why the sensor cannot reach it.
    fn connected_mcu(&self) -> Result<Arc<Mcu>, String> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| "lis2dw's MCU is not connected".to_string())
    }

    /// Bring the chip up and arm the bulk stream (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        let chip_name = self.chip.name();
        // In case of miswiring, the device id tells a real chip from noise.
        let expected = dev_id(self.chip);
        let dev_id = self.bus.read_reg(REG_WHO_AM_I).await?;
        tracing::info!("{chip_name}_dev_id: {dev_id:x}");
        if dev_id != expected {
            return Err(format!(
                "Invalid {chip_name} id (got {dev_id:x} vs {expected:x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty {chip_name} chip."
            ));
        }
        // On SPI the LIS2DW would otherwise keep its I2C interface enabled.
        if self.chip == LisChipType::Lis2dw && self.bus.bus_type() == LisBusType::Spi {
            let (reg, value) = disable_i2c_write();
            self.bus.set_reg(reg, value, chip_name).await?;
        }
        for &(reg, value) in setup_writes(self.chip) {
            self.bus.set_reg(reg, value, chip_name).await?;
        }

        // Start bulk reading: one report every four samples.
        let mcu = self.connected_mcu()?;
        let rest_ticks = mcu
            .seconds_to_clock(4. / f64::from(self.data_rate))
            .map_err(|err| err.to_string())? as u32;
        mcu.send_msg(&QueryLis2dw {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(|err| err.to_string())?;
        let (reg, value) = fifo_arm_write(self.chip);
        self.bus.set_reg(reg, value, chip_name).await?;

        tracing::info!("{} starting '{}' measurements", chip_name, self.name);
        // Initialize clock tracking.
        self.ffreader
            .note_start()
            .await
            .map_err(|err| err.to_string())?;
        Ok(())
    }

    /// Halt the bulk stream (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        let chip_name = self.chip.name();
        let (reg, value) = fifo_off_write();
        self.bus.set_reg(reg, value, chip_name).await?;
        // Upstream's `send_wait_ack` blocks for the ack; the queued messages
        // ahead of this one preserve the order, so the fire-and-forget send is
        // enough here.
        let mcu = self.connected_mcu()?;
        mcu.send_msg(&QueryLis2dw {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(|err| err.to_string())?;
        self.ffreader.note_end();
        tracing::info!("{} finished '{}' measurements", chip_name, self.name);
        self.bus.set_reg(reg, value, chip_name).await?;
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
        let overflows = self.ffreader.get_last_overflows();
        // Neither chip flags a per-sample error, so upstream's
        // `last_error_count` stays at zero.
        Ok(Some(json!({
            "data": data,
            "errors": 0,
            "overflows": overflows,
        })))
    }
}

/// Look one MCU up by its chip name (`spi_mcu` / `i2c_mcu`).
fn lookup_mcu(
    printer: &Arc<Printer>,
    mcu_name: &str,
    identifier: &str,
) -> Result<Arc<McuObject>, ConfigError> {
    printer
        .lookup_object_as::<McuObject>(&mcu_object_name(mcu_name))
        .ok_or_else(|| {
            ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
        })
}

/// Build the I2C half of `LIS2DW.__init__`'s bus selection: the `i2c_*`
/// options, a hardware or software bus, and the device on it.
fn setup_i2c(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<(Arc<McuI2c>, Arc<McuObject>), ConfigError> {
    let identifier = config.identifier();
    let mcu_name = config
        .get_str("i2c_mcu")
        .map(|text| text.trim().to_string())
        .unwrap_or_else(|| "mcu".to_string());
    let mcu_object = lookup_mcu(printer, &mcu_name, &identifier)?;

    let address =
        config.get_int_bounded("i2c_address", Some(DEFAULT_I2C_ADDR), Some(0), Some(127))? as u8;
    let speed = config.get_int_bounded(
        "i2c_speed",
        Some(DEFAULT_I2C_SPEED),
        Some(MIN_I2C_SPEED),
        None,
    )? as u32;

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
    Ok((i2c, mcu_object))
}

/// Upstream's `lis2dw.load_config` for `[lis2dw]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Lis2dw::new(config, printer, LisChipType::Lis2dw)?))
}

/// Upstream's `lis2dw.load_config_prefix` for `[lis2dw <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    load_config(config, printer)
}

/// Upstream's `lis3dh.load_config` for `[lis3dh]` — the same driver with
/// `LIS3DH_TYPE`.
pub fn load_config_lis3dh(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Lis2dw::new(config, printer, LisChipType::Lis3dh)?))
}

/// The prefix form of [`load_config_lis3dh`] (`[lis3dh <name>]`).
pub fn load_config_prefix_lis3dh(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    load_config_lis3dh(config, printer)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::any::Any;

    use crate::core::klippy::api::protocol::{PushTarget, Request};
    use crate::core::klippy::api::registry::{Api, EndpointContext};
    use crate::core::klippy::api::webhooks;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::bulk_sensor::{SampleFormat, SamplePayload};
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: &str, sub: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new(name, sub);
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
    fn wrap(name: &str, sub: Option<&str>, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
        ConfigWrapper::new(
            Box::leak(Box::new(section(name, sub, options))),
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

    /// The driver a factory built, downcast from its registered object.
    fn driver(object: Arc<dyn PrinterObject>) -> Arc<Lis2dw> {
        let any: Arc<dyn Any + Send + Sync> = object;
        any.downcast::<Lis2dw>()
            .expect("the factories build the lis2dw driver")
    }

    // -----------------------------------------------------------------
    // Section options and the chip-type branch
    // -----------------------------------------------------------------

    #[test]
    fn test_a_lis2dw_section_builds_the_lis2dw_chip_on_spi() {
        let printer = printer();
        // `cs_pin` present → SPI (`if config.get('cs_pin', None) is not None`).
        let lis =
            driver(load_config(&wrap("lis2dw", None, &[("cs_pin", "PK7")]), &printer).unwrap());

        assert_eq!(lis.name(), "lis2dw");
        assert_eq!(lis.chip(), LisChipType::Lis2dw);
        assert_eq!(lis.data_rate(), 1600);
        assert_eq!(lis.bus_type(), LisBusType::Spi);
        // The default `axes_map` is the identity, scaled by the chip's
        // sensitivity (`read_axes_map(config, LIS2DW_SCALE, …)`).
        assert_eq!(
            lis.axes_map(),
            [(0, LIS2DW_SCALE), (1, LIS2DW_SCALE), (2, LIS2DW_SCALE)]
        );
    }

    #[test]
    fn test_a_lis3dh_section_builds_the_lis3dh_chip_on_i2c() {
        let printer = printer();
        // No `cs_pin` → I2C (`MCU_I2C_from_config` with `LIS_I2C_ADDR`).
        let lis = driver(
            load_config_prefix_lis3dh(&wrap("lis3dh", Some("my_lis"), &[]), &printer).unwrap(),
        );

        assert_eq!(lis.name(), "my_lis");
        assert_eq!(lis.chip(), LisChipType::Lis3dh);
        assert_eq!(lis.data_rate(), 1344);
        assert_eq!(lis.bus_type(), LisBusType::I2c);
        assert_eq!(lis.axes_map()[0].1, LIS3DH_SCALE);
        assert_ne!(
            LIS2DW_SCALE, LIS3DH_SCALE,
            "the two chips scale differently"
        );
    }

    #[test]
    fn test_each_factory_hardcodes_its_chip_type() {
        // `lis3dh.py` is `lis2dw.LIS2DW(config, lis2dw.LIS3DH_TYPE)` and
        // nothing else: the section id picks the type, not an option.
        let printer = printer();
        let plain = driver(load_config(&wrap("lis2dw", None, &[]), &printer).unwrap());
        let three = driver(load_config_lis3dh(&wrap("lis3dh", None, &[]), &printer).unwrap());

        assert_eq!(plain.chip(), LisChipType::Lis2dw);
        assert_eq!(three.chip(), LisChipType::Lis3dh);
        assert_eq!((plain.data_rate(), three.data_rate()), (1600, 1344));
        assert_eq!(dev_id(plain.chip()), LIS2DW_DEV_ID);
        assert_eq!(dev_id(three.chip()), LIS3DH_DEV_ID);
    }

    #[test]
    fn test_an_unknown_axis_is_refused() {
        let printer = printer();
        let err = Lis2dw::new(
            &wrap("lis2dw", None, &[("cs_pin", "PK7"), ("axes_map", "x,y,w")]),
            &printer,
            LisChipType::Lis2dw,
        )
        .unwrap_err();
        assert_eq!(err.to_string(), "Invalid axes_map parameter");
    }

    // -----------------------------------------------------------------
    // Sample decoding and scaling
    // -----------------------------------------------------------------

    /// The identity `axes_map` at one chip's sensitivity.
    fn unit_axes(scale: f64) -> [(usize, f64); 3] {
        [(0, scale), (1, scale), (2, scale)]
    }

    #[test]
    fn test_the_sample_format_is_the_firmwares_six_byte_words() {
        let format = SampleFormat::parse(SAMPLE_FORMAT).expect("a valid struct format");
        assert_eq!(format.bytes_per_sample(), 6);
        // `BYTES_PER_BLOCK` (48) fits 8 samples in one report, and 51 // 6 is
        // upstream's `MAX_BULK_MSG_SIZE // bytes_per_sample`.
        assert_eq!(format.samples_per_block(), 8);
    }

    #[test]
    fn test_raw_sample_bytes_decode_and_convert_to_lis2dw_physical_values() {
        // One sample as `src/sensor_lis2dw.c` reports it: three little-endian
        // 16-bit words. The chip left-justifies its 14-bit reading, so the
        // signed word is the reading (`0x1000`, `-1`, `-0x1000` here).
        let format = SampleFormat::parse(SAMPLE_FORMAT).expect("a valid struct format");
        let bytes = [0x00, 0x10, 0xff, 0xff, 0x00, 0xf0];
        let payload = <[i32; SAMPLE_FIELDS]>::decode(&format, &bytes).expect("six good bytes");
        assert_eq!(payload, [0x1000, -1, -0x1000], "sign-extended words");

        let rows = convert_samples(&unit_axes(LIS2DW_SCALE), &[(1.5, payload)]);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0][0], 1.5);
        // 4096 LSB * (9.80665 * 1.952 / 4) mm/s² per LSB, rounded to 6 digits
        // (~2 g, the chip's full-scale reading for 4096 of its 14-bit counts).
        assert!((rows[0][1] - 19602.002739).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][2] - -4.785645).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][3] - -19602.002739).abs() < 1e-6, "{rows:?}");
    }

    #[test]
    fn test_lis3dh_raw_bytes_decode_and_convert_to_physical_values() {
        // The LIS3DH left-justifies its 12-bit reading in the same 16-bit word:
        // 0x123 → 0x1230 (+4656) and 0x8ab → 0x8ab0 (-30032, the 12-bit sign
        // bit carried through the word's sign extension).
        let format = SampleFormat::parse(SAMPLE_FORMAT).expect("a valid struct format");
        let bytes = [0x30, 0x12, 0x00, 0x00, 0xb0, 0x8a];
        let payload = <[i32; SAMPLE_FIELDS]>::decode(&format, &bytes).expect("six good bytes");
        assert_eq!(payload, [0x1230, 0, -30032]);

        let rows = convert_samples(&unit_axes(LIS3DH_SCALE), &[(0.25, payload)]);
        assert_eq!(rows[0][0], 0.25);
        // 4656 / -30032 LSB * (9.80665 * 11.718 / 16) mm/s² per LSB.
        assert!((rows[0][1] - 33440.068488).abs() < 1e-6, "{rows:?}");
        assert_eq!(rows[0][2], 0.0);
        assert!((rows[0][3] - -215694.187462).abs() < 1e-6, "{rows:?}");
    }

    #[test]
    fn test_convert_samples_applies_the_axes_map() {
        // The corpus accelerometer setting, `-x,-y,z`: negation flips the sign,
        // a permuted map reads the components from other axes.
        let negated = [(0, -LIS2DW_SCALE), (1, -LIS2DW_SCALE), (2, LIS2DW_SCALE)];
        let raw = [(1.0, [0x1000, -1, -0x1000])];
        let rows = convert_samples(&negated, &raw);
        assert!((rows[0][1] - -19602.002739).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][2] - 4.785645).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][3] - -19602.002739).abs() < 1e-6, "{rows:?}");

        let swapped = [(2, LIS2DW_SCALE), (0, LIS2DW_SCALE), (1, LIS2DW_SCALE)];
        let rows = convert_samples(&swapped, &raw);
        assert!((rows[0][1] - -19602.002739).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][2] - 19602.002739).abs() < 1e-6, "{rows:?}");
        assert!((rows[0][3] - -4.785645).abs() < 1e-6, "{rows:?}");
    }

    // -----------------------------------------------------------------
    // Register traffic
    // -----------------------------------------------------------------

    #[test]
    fn test_the_lis2dw_register_setup_sequence_is_upstreams() {
        assert_eq!(
            setup_writes(LisChipType::Lis2dw),
            [
                (0x25, 0x34), // CTRL_REG6: ODR/2, +-16g, low-pass, low noise
                (0x2e, 0xc0), // FIFO_CTRL: continuous mode
                (0x20, 0x94), // CTRL_REG1: high-performance 1600 Hz, 14-bit
            ]
        );
        // SPI only: disable the chip's I2C interface first.
        assert_eq!(disable_i2c_write(), (0x21, 0x06));
        // Re-armed after the bulk query starts, and cleared on both sides of
        // `_finish_measurements`.
        assert_eq!(fifo_arm_write(LisChipType::Lis2dw), (0x2e, 0xc0));
        assert_eq!(fifo_off_write(), (0x2e, 0x00));
    }

    #[test]
    fn test_the_lis3dh_register_setup_sequence_is_upstreams() {
        assert_eq!(
            setup_writes(LisChipType::Lis3dh),
            [
                (0x20, 0x97), // CTRL_REG1: 1344 Hz, high resolution, XYZ on
                (0x21, 0x00), // CTRL_REG2: no filtering
                (0x23, 0x38), // CTRL_REG4: +-16g, high resolution
                (0x24, 0x40), // CTRL_REG5: FIFO enabled
                (0x2e, 0x80), // FIFO_CTRL: stream mode
            ]
        );
        assert_eq!(fifo_arm_write(LisChipType::Lis3dh), (0x2e, 0x80));
    }

    #[test]
    fn test_register_frames_set_and_clear_the_read_bit() {
        // `read_reg` clocks the address with `REG_MOD_READ` set plus a dummy
        // byte out; the reply's second byte is the register's value.
        assert_eq!(spi_read_frame(REG_WHO_AM_I), [0x8f, 0x00]);
        assert_eq!(spi_read_frame(REG_FIFO_CTRL), [0xae, 0x00]);
        assert_eq!(read_reg_value(&[0x8f, 0x44]), Some(0x44));
        assert_eq!(read_reg_value(&[0x8f]), None);
        // Writes shift the plain address and the value, on both buses.
        assert_eq!(write_frame(REG_CTRL_REG1, 0x94), [0x20, 0x94]);
    }

    // -----------------------------------------------------------------
    // The dump endpoint
    // -----------------------------------------------------------------

    /// A recording push target for the endpoint test.
    #[derive(Default)]
    struct Recorder {
        messages: Mutex<Vec<Value>>,
    }

    impl PushTarget for Recorder {
        fn is_closed(&self) -> bool {
            false
        }
        fn push(&self, message: Value) {
            self.messages
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(message);
        }
    }

    #[tokio::test]
    async fn test_the_dump_endpoint_registers_the_sensor_key_and_header() {
        let printer = printer();
        let lis = driver(
            load_config_prefix(
                &wrap("lis2dw", Some("my_lis"), &[("cs_pin", "PK7")]),
                &printer,
            )
            .unwrap(),
        );

        let registrations = webhooks::install(&printer).unwrap().take_mux_endpoints();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].path, DUMP_ENDPOINT);
        assert_eq!(registrations[0].key, DUMP_KEY);
        assert_eq!(registrations[0].value.as_deref(), Some("my_lis"));

        // The start response is the column header a connecting client gets
        // (`{'header': hdr}` upstream); this answers before any sample moves,
        // so the chip's stream is irrelevant here.
        let request = Request::parse(
            br#"{"method":"lis2dw/dump_lis2dw","params":{"sensor":"my_lis","response_template":{"method":"dump"}}}"#,
        )
        .unwrap();
        let api = Api::new();
        let recorder = Arc::new(Recorder::default());
        let context = EndpointContext {
            api: &api,
            client: Arc::clone(&recorder) as Arc<dyn PushTarget>,
        };
        let reply = Arc::clone(&registrations[0].handler)
            .handle(&request, &context)
            .await
            .unwrap();
        assert_eq!(
            reply,
            json!({"header": [
                "time",
                "x_acceleration",
                "y_acceleration",
                "z_acceleration"
            ]})
        );
        assert_eq!(lis.name(), "my_lis");
    }
}
