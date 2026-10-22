//! `[bmi160 <name>]` — the BMI160 accelerometer (upstream's
//! `klippy/extras/bmi160.py`).
//!
//! A BMI160 on an I2C bus. The section reads the I2C mechanism options
//! (`i2c_address` / `i2c_speed` / `i2c_mcu` / `i2c_bus` / the software-pin
//! pair), asks its MCU for an [`McuI2c`] (the same resource `[i2c_device]`
//! uses), and reserves an oid for the firmware's `config_bmi160`.
//!
//! | piece | upstream |
//! |---|---|
//! | [`Bmi160`] — `axes_map`; I2C via `MCU_I2C_from_config` | `BMI160.__init__` |
//! | [`read_axes_map`] / [`convert_samples`] | the `adxl345.py` helpers `bmi160.py` imports |
//! | [`AccelQueryHelper`] | `adxl345.AccelQueryHelper` (what `start_internal_client` returns) |
//! | [`setup_writes`] / [`verify_after_write`] | `_start_measurements` / `set_reg` |
//! | `bmi160/dump_bmi160` mux endpoint (key `sensor`) | `batch_bulk.add_mux_endpoint(…)` |
//! | [`ConfigBmi160`] / [`QueryBmi160`] | `config_bmi160` / `query_bmi160` (in [`crate::core::klippy::cmd::bmi160`]) |
//!
//! Upstream picks SPI when the section has a `cs_pin` and I2C otherwise; this
//! section implements the I2C half, so `config_bmi160` always carries
//! [`BUS_OID_TYPE_I2C`].
//!
//! # The accelerometer interface
//!
//! A consumer finds the chip by name and asks it for a measurement client:
//! [`Bmi160::start_internal_client`] returns an [`AccelQueryHelper`], the same
//! helper `[adxl345]` and `[mpu9250]` hand out (upstream's `bmi160.py` builds
//! `adxl345.AccelQueryHelper` the same way). The helper is registered on the
//! shared [`BatchBulkHelper`] and collects the batches the dump endpoint also
//! streams.
//!
//! # The sample path
//!
//! Samples flow through the shared [`FixedFreqReader`] parameterized with the
//! chip's 6-byte `"<hhh"` layout — three signed 16-bit components, 51 // 6 = 8
//! samples per message, which is the firmware's `BYTES_PER_BLOCK` (48 bytes,
//! `src/sensor_bmi160.c`) — and the `query_bmi160_status` clock query.
//! `_start_measurements`' register sequence ([`setup_writes`], with
//! [`verify_after_write`] deciding which writes are read back) and
//! `_convert_samples` ([`convert_samples`], the ±16 g scale) follow upstream.
//!
//! One batch message carries `data` rows of `[time, x, y, z]` in mm/s², an
//! `errors` count (the BMI160 flags no per-sample errors, so it stays zero) and
//! `overflows`.

use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use serde_json::{json, Value};

use crate::core::klippy::cmd::bmi160::{
    ConfigBmi160, QueryBmi160, BUS_OID_TYPE_I2C, QUERY_BMI160_STATUS_MSGFORMAT,
};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, FixedFreqReader, LifecycleCb,
};
use crate::core::klippy::extras::mpu9250::{
    convert_samples, read_axes_map, AccelQueryHelper, AxesMap,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{
    ConfigBuilder, I2cMode, Mcu, McuError, McuI2c, McuObject, SEND_WAIT_ACK_TIMEOUT,
};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Upstream accepts both `[bmi160]` and `[bmi160 <name>]`. Loaded with the other
// I2C consumers (order 40), after `[board_pins]` (30), because a software bus
// may name its pins through an alias.
section!(
    "bmi160",
    order = 40,
    load = load_config,
    prefix = load_config_prefix
);

/// The path the dump endpoint registers (`bmi160.py`).
pub const DUMP_ENDPOINT: &str = "bmi160/dump_bmi160";

/// The mux key that selects a sensor instance.
pub const DUMP_KEY: &str = "sensor";

/// The default 7-bit device address (`BMI_I2C_ADDR`).
pub const DEFAULT_ADDR: i64 = 0x69;

/// The default I2C clock (`default_speed` at the `MCU_I2C_from_config` call).
pub const DEFAULT_SPEED: i64 = 400_000;

/// The lowest clock `MCU_I2C_from_config` accepts (`bus.py`'s `minval`).
const MIN_SPEED: i64 = 100_000;

/// Samples per second the chip is configured for (`self.data_rate = 1600`
/// upstream; the section has no `rate` option).
pub const DATA_RATE: u32 = 1600;

/// Earth gravity in mm/s² (`FREEFALL_ACCEL`).
const FREEFALL_ACCEL: f64 = 9.80665 * 1000.;

/// 2048 LSB/g at the ±16 g full scale, in mm/s² (`SCALE`).
pub const SCALE: f64 = FREEFALL_ACCEL / 2048.;

/// The report header a connecting dump client receives
/// (`('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')`).
pub const DUMP_HEADER: [&str; 4] = ["time", "x_acceleration", "y_acceleration", "z_acceleration"];

/// Seconds between batch callbacks and the smoothing span's unit (upstream
/// `bmi160.BATCH_UPDATES`).
const BMI_BATCH_UPDATES: f64 = 0.100;

/// The three signed 16-bit fields one sample unpacks (upstream's `"<hhh"`).
const SAMPLE_FIELDS: usize = 3;

// The chip's register map (`bmi160.py`'s `REG_*`).
const REG_CHIPID: u8 = 0x00;
const REG_ACC_CONF: u8 = 0x40;
const REG_ACC_RANGE: u8 = 0x41;
const REG_FIFO_DOWNS: u8 = 0x45;
const REG_FIFO_CONFIG_1: u8 = 0x47;
const REG_CMD: u8 = 0x7E;

/// Accelerometer normal mode (`CMD_ACC_PM_NORMAL`).
const CMD_ACC_PM_NORMAL: u8 = 0x11;

/// Accelerometer suspend mode (`CMD_ACC_PM_SUSPEND`).
const CMD_ACC_PM_SUSPEND: u8 = 0x10;

/// Empty the FIFO (`CMD_FIFO_FLUSH`).
const CMD_FIFO_FLUSH: u8 = 0xB0;

/// The id a working BMI160 answers with (`BMI160_DEV_ID`).
const BMI160_DEV_ID: u8 = 0xD1;

/// 1600 Hz output data rate, normal bandwidth, no undersampling
/// (`SET_ACC_CONF_1600HZ`).
const SET_ACC_CONF_1600HZ: u8 = 0x2C;

/// ±16 g full scale (`SET_ACC_RANGE_16G`).
const SET_ACC_RANGE_16G: u8 = 0x0C;

/// No FIFO downsampling (`SET_FIFO_DOWNS`).
const SET_FIFO_DOWNS: u8 = 0x00;

/// Accelerometer FIFO on, headerless mode (`SET_FIFO_CONFIG_1`).
const SET_FIFO_CONFIG_1: u8 = 0x40;

/// The lowest register `set_reg` reads back (`if reg >= 0x40 and reg != REG_CMD`
/// upstream — the command register and the read-only low registers are
/// write-only or unmapped).
const VERIFY_MIN_REG: u8 = 0x40;

// ===========================================================================
// Pure helpers (unit-tested directly)
// ===========================================================================

/// The register writes `_start_measurements` performs after the accelerometer
/// is awake, in order: output data rate, full scale, FIFO downsampling, then
/// the accelerometer FIFO.
fn setup_writes() -> [(u8, u8); 4] {
    [
        (REG_ACC_CONF, SET_ACC_CONF_1600HZ),
        (REG_ACC_RANGE, SET_ACC_RANGE_16G),
        (REG_FIFO_DOWNS, SET_FIFO_DOWNS),
        (REG_FIFO_CONFIG_1, SET_FIFO_CONFIG_1),
    ]
}

/// Whether [`Bmi160State::set_reg`] reads `reg` back after writing it
/// (upstream's `if reg >= 0x40 and reg != REG_CMD`).
///
/// The `CMD` register holds a command, not a stored value, and the registers
/// below 0x40 are the chip's read-only data/id block, so neither can be
/// verified.
fn verify_after_write(reg: u8) -> bool {
    reg >= VERIFY_MIN_REG && reg != REG_CMD
}

// ===========================================================================
// The sensor
// ===========================================================================

/// One configured BMI160.
pub struct Bmi160 {
    state: Arc<Bmi160State>,
}

impl Bmi160 {
    /// Read the section and wire the sensor up (`BMI160.__init__`).
    ///
    /// # Errors
    /// A bad `axes_map` or I2C option, an unknown MCU or pin, an oid shortage,
    /// or a dump endpoint already registered under this name.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().unwrap_or_else(|| {
            identifier
                .split_whitespace()
                .last()
                .unwrap_or(&identifier)
                .to_string()
        });

        // `adxl345.read_axes_map(config, SCALE, SCALE, SCALE)`: every axis
        // shares the one full-scale sensitivity.
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

        let state = Arc::new(Bmi160State {
            name,
            oid,
            address,
            speed,
            axes_map,
            printer: Arc::downgrade(printer),
            i2c,
            mcu_object: Arc::clone(&mcu_object),
            // Samples per second the clock regression expects, smoothed over
            // two batch windows (`chip_smooth = self.data_rate * BATCH_UPDATES * 2`).
            ffreader: FixedFreqReader::with_payload_format(
                f64::from(DATA_RATE) * BMI_BATCH_UPDATES * 2.,
                "<hhh",
                QUERY_BMI160_STATUS_MSGFORMAT,
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            last_error_count: Mutex::new(0),
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the I2C resource's own callback so the
        // device oid exists when `config_bmi160` is added (the resource
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
                    .ok_or_else(|| "bmi160 is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "bmi160 is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "bmi160 is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, BMI_BATCH_UPDATES);
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

    /// The sample rate the chip is configured for (`data_rate`, always 1600).
    pub fn data_rate(&self) -> u32 {
        DATA_RATE
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

    /// The parsed `axes_map` (`(source axis, signed scale)` triples).
    pub fn axes_map(&self) -> &AxesMap {
        &self.state.axes_map
    }

    /// Register a measurement client and hand it back
    /// (`BMI160.start_internal_client`).
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

impl PrinterObject for Bmi160 {
    /// Never reached through the API: [`PrinterObject::is_queryable`] is false,
    /// as upstream's object defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Bmi160 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Bmi160")
            .field("name", &self.state.name)
            .field("data_rate", &DATA_RATE)
            .field("address", &self.state.address)
            .field("speed", &self.state.speed)
            .finish_non_exhaustive()
    }
}

/// Everything the config callback and the client interface share.
struct Bmi160State {
    name: String,
    oid: u8,
    address: u8,
    speed: u32,
    /// Read by [`convert_samples`], the chip's decode helper.
    axes_map: AxesMap,
    /// The machine, for [`Bmi160::start_internal_client`].
    printer: Weak<Printer>,
    i2c: Arc<McuI2c>,
    /// The MCU the I2C bus is on (the reader's data binding needs it).
    mcu_object: Arc<McuObject>,
    /// The reader: 6-byte `"<hhh"` samples, `query_bmi160_status` queries.
    ffreader: FixedFreqReader<[i32; SAMPLE_FIELDS]>,
    /// The BMI160 flags no per-sample errors, so this stays zero
    /// (`last_error_count`, kept for the batch message's shape).
    last_error_count: Mutex<u64>,
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Bmi160State {
    /// The build-time half: the chip's config command, its disarmed-at-restart
    /// query, and the reader's data binding (`_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let i2c_oid = self.i2c.oid()?;
        builder.add_config_cmd(&ConfigBmi160 {
            oid: self.oid,
            bus_oid: i2c_oid,
            bus_oid_type: BUS_OID_TYPE_I2C,
        })?;
        // Upstream: `query_bmi160 oid rest_ticks=0` with `on_restart=True`, so
        // a firmware restart leaves the chip disarmed.
        builder.add_restart_cmd(&QueryBmi160 {
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
                "bmi160 register 0x{reg:02x}: short read ({} byte(s))",
                response.len()
            )
        })
    }

    /// Write one register, then read the configured ones back (`set_reg`): a
    /// mismatch means the chip did not take the setting.
    ///
    /// Upstream pauses 2 ms after every write "for stability"; the read-back is
    /// what actually proves the write landed, so the pause keeps the same place
    /// it has there.
    async fn set_reg(&self, reg: u8, value: u8) -> Result<(), String> {
        self.i2c
            .write(&[reg, value])
            .await
            .map_err(|err| err.to_string())?;
        tokio::time::sleep(Duration::from_millis(2)).await;
        if !verify_after_write(reg) {
            return Ok(());
        }
        let stored = self.read_reg(reg).await?;
        if stored != value {
            return Err(format!(
                "Failed to set BMI160 register [0x{reg:02x}] to 0x{value:02x}: got 0x{stored:02x}. \
                 This is generally indicative of connection problems (e.g. faulty wiring) or a \
                 faulty bmi160 chip."
            ));
        }
        Ok(())
    }

    /// The connected MCU, or why the sensor cannot reach it.
    fn connected_mcu(&self) -> Result<Arc<Mcu>, String> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| "bmi160's MCU is not connected".to_string())
    }

    /// Wake the chip, configure it, and arm the bulk stream
    /// (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        // In case of miswiring, the device id tells a real chip from noise.
        let dev_id = self.read_reg(REG_CHIPID).await?;
        if dev_id != BMI160_DEV_ID {
            return Err(format!(
                "Invalid bmi160 id (got {dev_id:x} vs {BMI160_DEV_ID:x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty bmi160 chip."
            ));
        }

        // Wake the accelerometer first: the chip needs ~50 ms to start up and
        // lock its PLL before the configuration writes take.
        self.set_reg(REG_CMD, CMD_ACC_PM_NORMAL).await?;
        tokio::time::sleep(Duration::from_millis(50)).await;

        for (reg, value) in setup_writes() {
            self.set_reg(reg, value).await?;
        }

        // Drop whatever the FIFO collected while the chip was awake and
        // unconfigured; the flush needs a moment to complete.
        self.set_reg(REG_CMD, CMD_FIFO_FLUSH).await?;
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Start bulk reading: one report every four samples.
        let mcu = self.connected_mcu()?;
        let rest_ticks = mcu
            .seconds_to_clock(4. / f64::from(DATA_RATE))
            .map_err(|err| err.to_string())? as u32;
        mcu.send_msg(&QueryBmi160 {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(|err| err.to_string())?;

        tracing::info!("BMI160 starting '{}' measurements", self.name);
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

    /// Halt the bulk stream and suspend the accelerometer
    /// (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        self.set_reg(REG_CMD, CMD_ACC_PM_SUSPEND).await?;
        // Upstream's `send_wait_ack`: the host moves on (`note_end`) only once
        // the firmware has taken the command that halts the stream.
        let mcu = self.connected_mcu()?;
        mcu.send_msg_wait_ack(
            &QueryBmi160 {
                oid: self.oid,
                rest_ticks: 0,
            },
            SEND_WAIT_ACK_TIMEOUT,
        )
        .await
        .map_err(|err| err.to_string())?;
        self.ffreader.note_end();
        tracing::info!("BMI160 finished '{}' measurements", self.name);
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

/// Upstream's `load_config` for `[bmi160]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Bmi160::new(config, printer)?))
}

/// Upstream's `load_config_prefix` for `[bmi160 <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Bmi160::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::protocol::Request;
    use crate::core::klippy::api::registry::{Api, EndpointContext};
    use crate::core::klippy::api::webhooks;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::extras::bulk_sensor::SampleFormat;
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("bmi160", name);
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
        printer
    }

    #[test]
    fn test_the_defaults_are_the_upstream_ones() {
        let printer = printer();

        let bmi = Bmi160::new(&wrap(Some("my_bmi"), &[]), &printer).unwrap();

        // `self.data_rate = 1600`, `BMI_I2C_ADDR`, `default_speed=400000`.
        assert_eq!(bmi.data_rate(), 1600);
        assert_eq!(bmi.i2c_address(), 0x69);
        assert_eq!(bmi.i2c_speed(), 400_000);
        assert_eq!(bmi.name(), "my_bmi");
    }

    #[test]
    fn test_the_default_options_are_recorded_as_read() {
        let printer = printer();
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(
            Box::leak(Box::new(section(Some("my_bmi"), &[]))),
            Arc::clone(&access),
        );

        Bmi160::new(&config, &printer).unwrap();

        for option in ["i2c_address", "i2c_speed"] {
            assert!(
                access.contains("bmi160 my_bmi", option),
                "option {option} recorded"
            );
        }
    }

    #[test]
    fn test_a_bare_section_is_named_after_its_section_id() {
        let printer = printer();

        let bmi = Bmi160::new(&wrap(None, &[]), &printer).unwrap();

        assert_eq!(bmi.name(), "bmi160");
    }

    #[test]
    fn test_the_default_axes_map_is_the_identity_at_the_16g_scale() {
        let map = read_axes_map(&wrap(Some("my_bmi"), &[]), SCALE).unwrap();

        assert_eq!(map, [(0, SCALE), (1, SCALE), (2, SCALE)]);
        // 2048 LSB/g at ±16 g, i.e. one count is 1/2048 g in mm/s².
        assert_eq!(SCALE * 2048., FREEFALL_ACCEL);
    }

    #[test]
    fn test_an_axes_map_at_the_16g_scale_negates_the_named_axes() {
        let map = read_axes_map(&wrap(Some("my_bmi"), &[("axes_map", "-x,-y,z")]), SCALE).unwrap();

        assert_eq!(map, [(0, -SCALE), (1, -SCALE), (2, SCALE)]);
    }

    #[test]
    fn test_the_wire_format_is_three_little_endian_signed_words() {
        // Upstream's `FixedFreqReader(mcu, chip_smooth, "<hhh")`: 6 bytes per
        // sample, 51 // 6 = 8 per message — the firmware's `BYTES_PER_BLOCK`
        // (8 samples of 6 bytes).
        let format = SampleFormat::parse("<hhh").expect("<hhh is a supported format");
        assert_eq!(format.bytes_per_sample(), 6);
        assert_eq!(format.samples_per_block(), 8);
        assert_eq!(8 * format.bytes_per_sample(), 48);
        // …and the reader the section builds accepts exactly that payload.
        assert!(
            FixedFreqReader::<[i32; SAMPLE_FIELDS]>::with_payload_format(
                320.,
                "<hhh",
                QUERY_BMI160_STATUS_MSGFORMAT
            )
            .is_ok()
        );
    }

    #[test]
    fn test_convert_samples_scales_the_signed_16_bit_counts() {
        // The identity map: ±2048 counts is ±1 g (9806.65 mm/s²), 1024 counts
        // is 4903.325 mm/s².
        let axes_map = [(0, SCALE), (1, SCALE), (2, SCALE)];

        let samples = convert_samples(&axes_map, &[(1.5, [2048, -2048, 1024])]);

        assert_eq!(samples, vec![[1.5, 9806.65, -9806.65, 4903.325]]);
    }

    #[test]
    fn test_convert_samples_applies_the_axes_map() {
        // X reads raw Z, Y reads raw X negated, Z reads raw Y — every axis at
        // the same ±16 g sensitivity.
        let axes_map = [(2, SCALE), (0, -SCALE), (1, SCALE)];

        let samples = convert_samples(&axes_map, &[(1.000_000_49, [2048, 1024, -1024])]);

        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0][0], 1.0, "time rounded to 6 digits");
        assert_eq!(samples[0][1], -4903.325, "x reads raw Z");
        assert_eq!(samples[0][2], -9806.65, "y reads raw X negated");
        assert_eq!(samples[0][3], 4903.325, "z reads raw Y");
    }

    #[test]
    fn test_convert_samples_of_nothing_is_empty() {
        let axes_map = [(0, SCALE), (1, SCALE), (2, SCALE)];

        assert!(convert_samples(&axes_map, &[]).is_empty());
    }

    #[test]
    fn test_the_register_setup_sequence_is_upstreams() {
        // `_start_measurements` after the wake command: output data rate, full
        // scale, FIFO downsampling, accelerometer FIFO.
        assert_eq!(
            setup_writes(),
            [
                (0x40, 0x2c), // ACC_CONF: 1600 Hz, normal bandwidth
                (0x41, 0x0c), // ACC_RANGE: ±16 g
                (0x45, 0x00), // FIFO_DOWNS: no downsampling
                (0x47, 0x40), // FIFO_CONFIG_1: accelerometer FIFO, headerless
            ]
        );
        // The two command bytes `_start_measurements` / `_finish_measurements`
        // send to `REG_CMD` around them.
        assert_eq!((REG_CMD, CMD_ACC_PM_NORMAL), (0x7e, 0x11));
        assert_eq!((REG_CMD, CMD_FIFO_FLUSH), (0x7e, 0xb0));
        assert_eq!((REG_CMD, CMD_ACC_PM_SUSPEND), (0x7e, 0x10));

        // Only the configured registers are read back: not the command
        // register, not the read-only block below 0x40.
        for (reg, _) in setup_writes() {
            assert!(verify_after_write(reg), "0x{reg:02x} is read back");
        }
        assert!(!verify_after_write(REG_CMD));
        assert!(!verify_after_write(REG_CHIPID));
        // Upstream's boundary is `reg >= 0x40` minus the command register, so
        // both ends of the configured block are read back.
        assert!(verify_after_write(0x40));
        assert!(verify_after_write(0x7f));
    }

    #[tokio::test]
    async fn test_the_dump_endpoint_is_registered_with_the_accelerometer_header() {
        let printer = printer();
        Bmi160::new(&wrap(Some("my_bmi"), &[]), &printer).unwrap();

        let webhooks = webhooks::install(&printer).unwrap();
        let registrations = webhooks.take_mux_endpoints();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].path, DUMP_ENDPOINT);
        assert_eq!(registrations[0].key, DUMP_KEY);
        assert_eq!(registrations[0].value.as_deref(), Some("my_bmi"));

        // Subscribing returns the start response: the columns a dump client
        // streams by (`hdr = ('time', 'x_acceleration', …)`).
        let api = Api::new();
        let context = EndpointContext {
            api: &api,
            client: crate::core::klippy::api::test_support::silent_target(),
        };
        let request =
            Request::parse(br#"{"method":"bmi160/dump_bmi160","params":{"sensor":"my_bmi"}}"#)
                .expect("test body is a valid request");
        let reply = registrations[0]
            .handler
            .handle(&request, &context)
            .await
            .expect("the endpoint accepts a subscriber");
        assert_eq!(reply, json!({ "header": DUMP_HEADER }));
    }

    #[tokio::test]
    async fn test_starting_a_measurement_registers_a_batch_client() {
        // Asking for a client subscribes it to the batch stream (whose first
        // client runs the chip's start callback), and nothing has arrived yet.
        let printer = printer();
        let bmi = Bmi160::new(&wrap(Some("my_bmi"), &[]), &printer).unwrap();

        let client = bmi.start_internal_client();

        assert_eq!(bmi.client_count(), 1);
        assert!(client.get_samples().is_empty());
    }

    #[test]
    fn test_a_software_bus_with_one_pin_is_refused() {
        let printer = printer();

        let err = Bmi160::new(
            &wrap(Some("my_bmi"), &[("i2c_software_scl_pin", "PA0")]),
            &printer,
        )
        .unwrap_err();

        assert!(
            err.to_string().contains("both 'i2c_software_scl_pin'"),
            "{err}"
        );
    }
}
