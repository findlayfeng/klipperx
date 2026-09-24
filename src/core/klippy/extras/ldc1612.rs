//! `ldc1612` — the LDC1612 inductive sensor (upstream's
//! `klippy/extras/ldc1612.py`).
//!
//! Upstream has no `[ldc1612]` config section: the module is a *library*
//! object constructed by its consumer (`probe_eddy_current`) from that
//! consumer's section, so this file exposes [`Ldc1612::new`] and registers no
//! factory. What it owns:
//!
//! | piece | upstream |
//! |---|---|
//! | [`Ldc1612`] / configuration + bulk stream | `LDC1612.__init__` / `_build_config` |
//! | `LDC_CALIBRATE_DRIVE_CURRENT` (mux `CHIP=<name>`) | `DriveCurrentCalibrate` |
//! | [`Ldc1612::setup_trigger_analog`] | `LDC1612.setup_trigger_analog` (init command) |
//! | `ldc1612/dump_ldc1612` mux endpoint | `batch_bulk.add_mux_endpoint(…)` |
//! | [`Calibration`] | the `calibration` object passed to `LDC1612(config, calibration)` |
//!
//! The batch machinery lives in [`super::bulk_sensor`] (H6); the wire commands
//! in [`crate::core::klippy::cmd::ldc1612`].

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::ldc1612::{
    ConfigLdc1612, ConfigLdc1612WithIntb, Ldc1612AttachTriggerAnalog, QueryLdc1612,
};
use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, FixedFreqReader, LifecycleCb, Sample, BATCH_INTERVAL,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError, McuI2c, McuObject};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::Printer;

/// The path the dump endpoint registers (`ldc1612.py`).
pub const DUMP_ENDPOINT: &str = "ldc1612/dump_ldc1612";

/// The mux key that selects a sensor instance.
pub const DUMP_KEY: &str = "sensor";

const BATCH_UPDATES: f64 = 0.100;
const LDC1612_ADDR: i64 = 0x2a;
const DEFAULT_LDC1612_FREQ: i64 = 12_000_000;
const SETTLETIME: f64 = 0.005;
const DRIVECUR: i64 = 15;
const DEGLITCH: u16 = 0x05;

const LDC1612_MANUF_ID: u16 = 0x5449;
const LDC1612_DEV_ID: u16 = 0x3055;

const REG_RCOUNT0: u16 = 0x08;
const REG_OFFSET0: u16 = 0x0c;
const REG_SETTLECOUNT0: u16 = 0x10;
const REG_CLOCK_DIVIDERS0: u16 = 0x14;
const REG_ERROR_CONFIG: u16 = 0x19;
const REG_CONFIG: u16 = 0x1a;
const REG_MUX_CONFIG: u16 = 0x1b;
const REG_DRIVE_CURRENT0: u16 = 0x1e;
const REG_MANUFACTURER_ID: u16 = 0x7e;
const REG_DEVICE_ID: u16 = 0x7f;

/// Samples per second the firmware produces (`LDC1612.data_rate`).
pub const DATA_RATE: u32 = 400;

/// The frequency→height calibration the consumer passes in
/// (`probe_eddy_current.EddyCalibration` by name at the upstream call site).
pub trait Calibration: Send + Sync {
    /// `(calibration frequencies, calibration heights)` — read once to warn
    /// about `max_sensor_hz` (`get_calibration`).
    fn get_calibration(&self) -> (Vec<f64>, Vec<f64>);
    /// Add each sample's `z` (`apply_calibration`); `data` rows are
    /// `[time, frequency, z]`.
    fn apply_calibration(&self, data: &mut [[f64; 3]]);
}

// ===========================================================================
// Pure helpers (unit-tested directly)
// ===========================================================================

/// The sensor clock divider: `4 * max_hz` must stay below the reference
/// (`int(math.ceil(4. * max_hz / self.clock_freq))`).
pub(crate) fn sensor_divider(clock_freq: i64, max_hz: f64) -> u32 {
    (4. * max_hz / clock_freq as f64).ceil() as u32
}

/// Raw value ↔ hertz: `float(clock_freq * sensor_div) / (1 << 28)`.
pub(crate) fn freq_conversion(clock_freq: i64, sensor_div: u32) -> f64 {
    f64::from(clock_freq as u32) * f64::from(sensor_div) / (1_u32 << 28) as f64
}

/// `reg_drive_current0 >> 6 & 0x1f` — the value `LDC_CALIBRATE_DRIVE_CURRENT`
/// reports and SAVE_CONFIG stores.
pub(crate) fn drive_current_from_register(reg_drive_current0: u16) -> u8 {
    ((reg_drive_current0 >> 6) & 0x1f) as u8
}

/// Python's `round(value, digits)` for the sample timestamps/frequencies.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// What one [`convert_samples`] pass produced.
pub(crate) struct ConvertedBatch {
    /// `[time, frequency, z]` rows — `z` stays upstream's `999.9` placeholder
    /// until [`Calibration::apply_calibration`] fills it.
    pub data: Vec<[f64; 3]>,
    /// `(message, occurrences)`, first-seen order, for logging.
    pub errors: Vec<(String, u32)>,
    /// Samples that took the error branch this pass (`last_error_count`).
    pub error_count: u64,
}

/// Decode raw register values into frequencies, classifying error values the
/// way upstream's `_convert_samples` does.
///
/// `error_name` names a firmware-encoded error (`0xffffxxxx` carries the
/// `ldc1612_error:` enumeration's value in the low bits); values the sensor
/// itself flags (error bits 28..32) are logged by message and the sample is
/// **kept**, exactly as upstream — only a firmware-encoded error drops it.
pub(crate) fn convert_samples(
    freq_conv: f64,
    error_name: &dyn Fn(u16) -> Option<String>,
    raw: &[Sample],
) -> ConvertedBatch {
    let mut data = Vec::with_capacity(raw.len());
    let mut errors: Vec<(String, u32)> = Vec::new();
    let mut error_count: u64 = 0;
    fn log_once(message: String, errors: &mut Vec<(String, u32)>) {
        match errors.iter_mut().find(|(known, _)| *known == message) {
            Some((_, count)) => *count += 1,
            None => errors.push((message, 1)),
        }
    }
    for (ptime, value) in raw {
        let value = *value;
        let mut mv = value & 0x0fff_ffff;
        if value > 0x03ff_ffff || value == 0 {
            error_count += 1;
            if (value >> 16 & 0xffff) == 0xffff {
                // Encoded error from `sensor_ldc1612.c` — no sample.
                let name = error_name((value & 0xffff) as u16)
                    .unwrap_or_else(|| "Unknown ldc1612 error".to_string());
                log_once(name, &mut errors);
                continue;
            }
            let error_bits = (value >> 28) & 0x0f;
            if error_bits & 0x8 != 0 || mv == 0 {
                log_once("Frequency under valid range".to_string(), &mut errors);
            }
            if error_bits & 0x4 != 0 || mv > 0x03ff_ffff {
                let kind = if error_bits & 0x4 != 0 {
                    "hard"
                } else {
                    "soft"
                };
                log_once(format!("Frequency over valid {kind} range"), &mut errors);
            }
            if error_bits & 0x2 != 0 {
                log_once("Conversion Watchdog timeout".to_string(), &mut errors);
            }
            if error_bits & 0x1 != 0 {
                log_once("Amplitude Low/High warning".to_string(), &mut errors);
            }
            mv &= 0x0fff_ffff;
        }
        data.push([round(*ptime, 6), round(freq_conv * mv as f64, 3), 999.9]);
    }
    ConvertedBatch {
        data,
        errors,
        error_count,
    }
}

// ===========================================================================
// The sensor
// ===========================================================================

/// One configured LDC1612, built from a consumer's config section.
pub struct Ldc1612 {
    state: Arc<Ldc1612State>,
}

impl Ldc1612 {
    /// Read the section and wire the sensor up (`LDC1612.__init__`).
    ///
    /// `calibration` is the consumer's frequency→height calibration, or
    /// `None` (`LDC1612(config, calibration=None)`).
    ///
    /// # Errors
    /// A missing or out-of-range option, an unknown MCU or pin, an oid
    /// shortage, or an endpoint/key already registered.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        calibration: Option<Arc<dyn Calibration>>,
    ) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().unwrap_or_else(|| {
            identifier
                .split_whitespace()
                .last()
                .unwrap_or(&identifier)
                .to_string()
        });

        // --- clock and divider -------------------------------------------
        let clock_freq = config.get_int_bounded(
            "frequency",
            Some(DEFAULT_LDC1612_FREQ),
            Some(2_000_000),
            Some(40_000_000),
        )?;
        let max_hz = config.get_float_bounded(
            "max_sensor_hz",
            Some(5_000_000.),
            Some(3_000_000.),
            Some(20_000_000.),
            None,
            None,
        )?;
        let sensor_div = sensor_divider(clock_freq, max_hz);
        let freq_conv = freq_conversion(clock_freq, sensor_div);
        if let Some(calibration) = &calibration {
            let (frequencies, _) = calibration.get_calibration();
            let peak = frequencies
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);
            if peak > max_hz {
                if let Some(configfile) =
                    printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
                {
                    configfile.runtime_warning(&format!(
                        "ldc1612 {name}: Should set 'max_sensor_hz' to at least {}",
                        peak.ceil()
                    ));
                }
            }
        }

        // --- drive current (DriveCurrentCalibrate) ------------------------
        let drive_cur =
            config.get_int_bounded("reg_drive_current", Some(DRIVECUR), Some(0), Some(31))? as u8;

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
            config.get_int_bounded("i2c_address", Some(LDC1612_ADDR), Some(0), Some(127))? as u8;
        let speed = config.get_int_bounded("i2c_speed", Some(400_000), Some(100_000), None)? as u32;

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
                crate::core::klippy::mcu::I2cMode::Software {
                    scl_pin: scl_params.pin,
                    sda_pin: sda_params.pin,
                    speed,
                }
            }
            (None, None) => crate::core::klippy::mcu::I2cMode::Hardware {
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

        // --- intb pin (validated as upstream does, at config read) --------
        let intb_pin = match config.get_str("intb_pin") {
            None => None,
            Some(text) => {
                let params = pins
                    .lookup_pin(&text, false, false, None)
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                if params.chip_name != mcu_name {
                    return Err(ConfigError::new("ldc1612 intb_pin must be on same mcu"));
                }
                Some(params.pin)
            }
        };

        // --- oids, reader, batch helper -----------------------------------
        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let ffreader = FixedFreqReader::new(f64::from(DATA_RATE) * BATCH_UPDATES * 2.);

        let state = Arc::new(Ldc1612State {
            name,
            oid,
            printer: Arc::downgrade(printer),
            mcu_object,
            builder: Arc::clone(&builder),
            i2c,
            pins: Arc::clone(&pins),
            chip: mcu_name,
            intb_pin,
            clock_freq,
            sensor_div,
            freq_conv,
            drive_cur: Mutex::new(drive_cur),
            last_error_count: Mutex::new(0),
            calibration,
            ffreader,
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the I2C resource's own callback so
        // the device oid exists when `config_ldc1612` is added (the I2C
        // device allocates it in its config callback).
        let callback_state = Arc::downgrade(&state);
        builder
            .register_config_callback(Box::new(move |builder, mcu| {
                if let Some(state) = callback_state.upgrade() {
                    state.build(builder, mcu)?;
                }
                Ok(())
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // --- batch helper (BatchBulkHelper + LDC1612 start/stop/process) --
        let start_state = Arc::downgrade(&state);
        let start_cb: LifecycleCb = Arc::new(move || {
            let start_state = start_state.clone();
            Box::pin(async move {
                let state = start_state
                    .upgrade()
                    .ok_or_else(|| "ldc1612 is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "ldc1612 is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "ldc1612 is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk = BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, BATCH_INTERVAL);
        batch_bulk.add_mux_endpoint(
            DUMP_ENDPOINT,
            DUMP_KEY,
            &state.name,
            json!({"header": ["time", "frequency", "z"]}),
        )?;
        *state.batch_bulk.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::clone(&batch_bulk));

        // --- LDC_CALIBRATE_DRIVE_CURRENT (mux CHIP=<name>) ----------------
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let command_state = Arc::clone(&state);
        let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
            let state = Arc::clone(&command_state);
            Box::pin(async move { state.calibrate_drive_current(gcmd).await })
        });
        gcode
            .register_mux_command(
                "LDC_CALIBRATE_DRIVE_CURRENT",
                "CHIP",
                Some(&state.name),
                handler,
                Some("Calibrate LDC1612 DRIVE_CURRENT register"),
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self { state })
    }

    /// Queue `ldc1612_attach_trigger_analog` so this sensor's samples reach
    /// the `trigger_analog` object `trigger_analog_oid` (`setup_trigger_analog`,
    /// an **init** command — upstream `is_init=True`).
    ///
    /// # Errors
    /// [`McuError::Config`] if the configuration is already built.
    pub fn setup_trigger_analog(&self, trigger_analog_oid: u8) -> Result<(), McuError> {
        add_attach_trigger_analog(&self.state.builder, self.state.oid, trigger_analog_oid)
    }

    /// The sensor's name (the consumer section's sub).
    pub fn name(&self) -> &str {
        &self.state.name
    }

    /// The sensor's object id.
    pub fn oid(&self) -> u8 {
        self.state.oid
    }

    /// Samples per second the firmware produces.
    pub fn data_rate(&self) -> u32 {
        DATA_RATE
    }

    /// Register a batch client (the consumer's `add_client`).
    pub fn add_client(&self, client: impl Fn(&Value) -> bool + Send + Sync + 'static) {
        let bulk = self
            .state
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        if let Some(bulk) = bulk {
            bulk.add_client(Arc::new(client));
        }
    }

    /// Raw value → hertz (`convert_raw_to_frequency`).
    pub fn convert_raw_to_frequency(&self, raw_value: u32) -> f64 {
        raw_value as f64 * self.state.freq_conv
    }

    /// Hertz → raw value (`convert_frequency_to_raw`).
    pub fn convert_frequency_to_raw(&self, freq: f64) -> u32 {
        (freq / self.state.freq_conv + 0.5) as u32
    }

    /// A firmware-encoded sample error's name (`lookup_sensor_error`).
    pub fn lookup_sensor_error(&self, error: u16) -> String {
        error_text(&self.state.mcu_object, error)
    }
}

/// Queue `ldc1612_attach_trigger_analog` on `builder` (the free form, so a
/// test can bind it to a bare builder without a whole machine).
///
/// # Errors
/// [`McuError::Config`] if the configuration is already built.
pub fn add_attach_trigger_analog(
    builder: &ConfigBuilder,
    oid: u8,
    trigger_analog_oid: u8,
) -> Result<(), McuError> {
    builder.add_init_cmd(&Ldc1612AttachTriggerAnalog {
        oid,
        trigger_analog_oid,
    })
}

/// Name a `ldc1612_error:` value through the dictionary enumeration.
fn error_text(mcu_object: &Arc<McuObject>, error: u16) -> String {
    mcu_object
        .mcu()
        .and_then(|mcu| mcu.dictionary())
        .and_then(|dictionary| {
            dictionary
                .enumeration("ldc1612_error:")
                .map(|e| e.name(i64::from(error)).map(str::to_string))
        })
        .flatten()
        .unwrap_or_else(|| "Unknown ldc1612 error".to_string())
}

/// Everything the callbacks share (`LDC1612`'s fields).
struct Ldc1612State {
    name: String,
    oid: u8,
    printer: Weak<Printer>,
    mcu_object: Arc<McuObject>,
    builder: Arc<ConfigBuilder>,
    i2c: Arc<McuI2c>,
    pins: Arc<PrinterPins>,
    /// The MCU the chip is on (`i2c_mcu`), for the `intb_pin` check.
    chip: String,
    intb_pin: Option<String>,
    clock_freq: i64,
    #[allow(dead_code)]
    sensor_div: u32,
    freq_conv: f64,
    drive_cur: Mutex<u8>,
    last_error_count: Mutex<u64>,
    calibration: Option<Arc<dyn Calibration>>,
    ffreader: FixedFreqReader,
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Ldc1612State {
    /// The build-time half (`__init__`'s config commands + `_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let i2c_oid = self.i2c.oid()?;
        match &self.intb_pin {
            None => builder.add_config_cmd(&ConfigLdc1612 {
                oid: self.oid,
                i2c_oid,
            })?,
            Some(pin_name) => {
                // Alias-resolve the pin, then read its firmware number from
                // the `pin` enumeration — what `pin_number` does internally.
                let resolved = self
                    .pins
                    .resolve_pin(&self.chip, pin_name)
                    .map_err(|err| McuError::Config(err.to_string()))?;
                let dictionary = mcu.dictionary().ok_or_else(|| {
                    McuError::Config("the firmware dictionary is missing".to_string())
                })?;
                let number = dictionary
                    .enumeration("pin")
                    .and_then(|enumeration| enumeration.value(&resolved))
                    .ok_or_else(|| {
                        McuError::Config(format!(
                            "Pin '{resolved}' is not a valid pin name on mcu '{}'",
                            mcu.name()
                        ))
                    })?;
                builder.add_config_cmd(&ConfigLdc1612WithIntb {
                    oid: self.oid,
                    i2c_oid,
                    intb_pin: u8::try_from(number).map_err(|_| {
                        McuError::Config(format!("Pin '{resolved}' does not fit a byte"))
                    })?,
                })?;
            }
        }
        // Upstream: `query_ldc1612 oid rest_ticks=0` with `on_restart=True`.
        builder.add_restart_cmd(&QueryLdc1612 {
            oid: self.oid,
            rest_ticks: 0,
        })?;
        self.ffreader.bind(mcu, &self.mcu_object, self.oid)?;
        Ok(())
    }

    /// Read a two-byte register (`read_reg`).
    async fn read_reg(&self, reg: u16) -> Result<u16, McuError> {
        let response = self.i2c.transfer(&[reg as u8], 2).await?;
        if response.len() < 2 {
            return Err(McuError::Config(format!(
                "ldc1612 register 0x{reg:x}: short read ({} bytes)",
                response.len()
            )));
        }
        Ok(u16::from_be_bytes([response[0], response[1]]))
    }

    /// Write a register (`set_reg`).
    async fn set_reg(&self, reg: u16, value: u16) -> Result<(), McuError> {
        self.i2c
            .write(&[reg as u8, ((value >> 8) & 0xff) as u8, (value & 0xff) as u8])
            .await
    }

    fn connected_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| McuError::Config("the sensor's MCU is not connected".to_string()))
    }

    /// Bring the chip up and start the bulk stream (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        let manuf_id = self.read_reg(REG_MANUFACTURER_ID).await.map_err(err)?;
        let dev_id = self.read_reg(REG_DEVICE_ID).await.map_err(err)?;
        if manuf_id != LDC1612_MANUF_ID || dev_id != LDC1612_DEV_ID {
            return Err(format!(
                "Invalid ldc1612 id (got {manuf_id:x},{dev_id:x} vs \
                 {LDC1612_MANUF_ID:x},{LDC1612_DEV_ID:x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty ldc1612 chip."
            ));
        }
        let rcount0 = self.clock_freq as f64 / (16. * DATA_RATE as f64);
        self.set_reg(REG_RCOUNT0, (rcount0 + 0.5) as u16)
            .await
            .map_err(err)?;
        self.set_reg(REG_OFFSET0, 0).await.map_err(err)?;
        self.set_reg(
            REG_SETTLECOUNT0,
            (SETTLETIME * self.clock_freq as f64 / 16. + 0.5) as u16,
        )
        .await
        .map_err(err)?;
        self.set_reg(REG_CLOCK_DIVIDERS0, (self.sensor_div as u16) << 12 | 1)
            .await
            .map_err(err)?;
        self.set_reg(REG_ERROR_CONFIG, (0x1f << 11) | 1)
            .await
            .map_err(err)?;
        self.set_reg(REG_MUX_CONFIG, 0x0208 | DEGLITCH)
            .await
            .map_err(err)?;
        self.set_reg(REG_CONFIG, 0x001 | (1 << 12) | (1 << 10) | (1 << 9))
            .await
            .map_err(err)?;
        let drive_cur = *self.drive_cur.lock().unwrap_or_else(|p| p.into_inner());
        self.set_reg(REG_DRIVE_CURRENT0, u16::from(drive_cur) << 11)
            .await
            .map_err(err)?;

        // Start bulk reading.
        let mcu = self.connected_mcu().map_err(err)?;
        let rest_ticks = mcu.seconds_to_clock(0.5 / DATA_RATE as f64).map_err(err)? as u32;
        mcu.send_msg(&QueryLdc1612 {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(err)?;
        self.ffreader.note_start().await.map_err(err)?;
        *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 0;
        Ok(())
    }

    /// Halt the bulk stream (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        let mcu = self.connected_mcu().map_err(err)?;
        mcu.send_msg(&QueryLdc1612 {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(err)?;
        self.ffreader.note_end();
        Ok(())
    }

    /// One batch: pull, convert, calibrate, report (`_process_batch`).
    async fn process_batch(&self, _eventtime: f64) -> Result<Option<Value>, String> {
        let raw = self.ffreader.pull_samples().await.map_err(err)?;
        if raw.is_empty() {
            return Ok(None);
        }
        let mcu_object = Arc::clone(&self.mcu_object);
        let converted = convert_samples(
            self.freq_conv,
            &|code| {
                mcu_object
                    .mcu()
                    .and_then(|mcu| mcu.dictionary())
                    .and_then(|dictionary| {
                        dictionary
                            .enumeration("ldc1612_error:")
                            .and_then(|enumeration| {
                                enumeration.name(i64::from(code)).map(str::to_string)
                            })
                    })
            },
            &raw,
        );
        for (message, count) in &converted.errors {
            tracing::error!("{}: {} ({count})", self.name, message);
        }
        let mut error_count = self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        *error_count += converted.error_count;
        let error_count = *error_count;

        let mut data = converted.data;
        if let Some(calibration) = &self.calibration {
            calibration.apply_calibration(&mut data);
        }
        let overflows = self.ffreader.get_last_overflows();
        Ok(Some(json!({
            "data": data,
            "errors": error_count,
            "overflows": overflows,
        })))
    }

    /// `LDC_CALIBRATE_DRIVE_CURRENT` (`DriveCurrentCalibrate.cmd_LDC_CALIBRATE`).
    async fn calibrate_drive_current(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let bulk = self
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
            .ok_or_else(|| CommandError::new("ldc1612 is not initialized"))?;
        // Keep the batch stream alive while calibrating; the client leaves
        // when the flag drops (`handle_batch` returning `is_in_progress`).
        let in_progress = Arc::new(AtomicBool::new(true));
        let client_flag = Arc::clone(&in_progress);
        bulk.add_client(Arc::new(move |_message: &Value| {
            client_flag.load(Ordering::SeqCst)
        }));

        if let Some(printer) = self.printer.upgrade() {
            if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>("toolhead") {
                toolhead.dwell(0.100);
            }
        }
        let old_config = self.read_reg(REG_CONFIG).await.map_err(map_command)?;
        self.set_reg(REG_CONFIG, 0x001 | (1 << 9))
            .await
            .map_err(map_command)?;
        let reg_drive_current0 = self
            .read_reg(REG_DRIVE_CURRENT0)
            .await
            .map_err(map_command)?;
        self.set_reg(REG_CONFIG, old_config)
            .await
            .map_err(map_command)?;
        in_progress.store(false, Ordering::SeqCst);

        let drive_cur = drive_current_from_register(reg_drive_current0);
        gcmd.respond_info(&format!(
            "{}: reg_drive_current: {drive_cur}\n\
             The SAVE_CONFIG command will update the printer config file\n\
             with the above and restart the printer.",
            self.name
        ));
        *self.drive_cur.lock().unwrap_or_else(|p| p.into_inner()) = drive_cur;
        if let Some(printer) = self.printer.upgrade() {
            if let Some(configfile) = printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT) {
                configfile.set(&self.name, "reg_drive_current", &drive_cur.to_string());
            }
        }
        Ok(())
    }
}

/// Transport error → callback message.
fn err(error: McuError) -> String {
    error.to_string()
}

/// Transport error → command error.
fn map_command(error: McuError) -> CommandError {
    CommandError::new(error.to_string())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::protocol::{PushTarget, Request};
    use crate::core::klippy::api::registry::{Api, EndpointContext, MuxEndpoint};
    use crate::core::klippy::api::webhooks;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::reactor::ManualReactor;
    use std::collections::HashMap;
    use std::sync::atomic::AtomicBool;

    #[test]
    fn test_sensor_divider_and_freq_conversion() {
        // 5 MHz limit on a 12 MHz clock: 4*5/12 = 1.67 → 2.
        assert_eq!(sensor_divider(12_000_000, 5_000_000.), 2);
        assert_eq!(
            freq_conversion(12_000_000, 2),
            24_000_000. / (1u32 << 28) as f64
        );
        // 3 MHz limit on a 40 MHz clock: 4*3/40 < 1 → ceil → 1.
        assert_eq!(sensor_divider(40_000_000, 3_000_000.), 1);
        // Round-trip raw ↔ frequency.
        let conv = freq_conversion(12_000_000, 2);
        let sensor = TestSensor { conv };
        let raw = sensor.convert_frequency_to_raw(1_000_000.);
        assert!((sensor.convert_raw_to_frequency(raw) - 1_000_000.).abs() < conv * 0.5);
    }

    /// The two conversion methods, over one `freq_conv`.
    struct TestSensor {
        conv: f64,
    }

    impl TestSensor {
        fn convert_raw_to_frequency(&self, raw: u32) -> f64 {
            raw as f64 * self.conv
        }
        fn convert_frequency_to_raw(&self, freq: f64) -> u32 {
            (freq / self.conv + 0.5) as u32
        }
    }

    #[test]
    fn test_convert_samples_error_branches() {
        let conv = 0.1;
        let named = |code: u16| Some(format!("FW_ERROR_{code}"));

        // A valid value, an encoded firmware error (dropped), an error-bits
        // value (kept), and a zero (kept, logged under-range).
        let raw: Vec<Sample> = vec![
            (1.000_000_4, 0x00ab_cdef),
            (2.0, 0xffff_0003),
            (3.0, 0x8000_0042),
            (4.0, 0),
        ];
        let converted = convert_samples(conv, &named, &raw);

        // Only the encoded error drops its sample.
        assert_eq!(converted.data.len(), 3);
        assert_eq!(
            converted.error_count, 3,
            "three values took the error branch"
        );
        assert_eq!(converted.data[0][0], 1.000_000, "time rounded to 6 digits");
        assert_eq!(
            converted.data[0][1],
            (0x00ab_cdef as f64 * conv * 1000.).round() / 1000.
        );
        assert_eq!(converted.data[0][2], 999.9, "z stays the placeholder");

        let errors: HashMap<&str, u32> = converted
            .errors
            .iter()
            .map(|(message, count)| (message.as_str(), *count))
            .collect();
        assert_eq!(errors.get("FW_ERROR_3"), Some(&1));
        assert_eq!(errors.get("Frequency under valid range"), Some(&2));
        assert!(
            errors.contains_key("Conversion Watchdog timeout"),
            "{errors:?}"
        );
    }

    #[test]
    fn test_drive_current_register_extraction() {
        // (reg >> 6) & 0x1f, upstream's extraction from REG_DRIVE_CURRENT0.
        assert_eq!(drive_current_from_register(0), 0);
        assert_eq!(drive_current_from_register(15 << 6), 15);
        assert_eq!(drive_current_from_register(31 << 6), 31);
        assert_eq!(drive_current_from_register(0b1010_1100_0000), 0b10101);
        // Bits above the 5-bit field are ignored.
        assert_eq!(drive_current_from_register(0xffff << 6), 31);
    }

    /// An identified test MCU over the corpus dictionary, for build tests.
    fn test_mcu() -> Arc<Mcu> {
        let mcu = Arc::new(Mcu::for_test(
            "mcu",
            Interface::new(FrameMock::new(Vec::new())),
        ));
        let path = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let raw = std::fs::read(&path).expect("the corpus dictionary");
        let value: serde_json::Value = serde_json::from_slice(&raw).expect("valid JSON");
        mcu.install_dictionary(crate::core::klippy::mcu::Dictionary::from_json(value).unwrap())
            .unwrap();
        mcu
    }

    /// Decode an encoded payload list into `(name, args)` (as the
    /// `trigger_analog` tests do for the config list).
    fn decoded(
        mcu: &Mcu,
        payloads: &[crate::core::klippy::msg::Payload],
    ) -> Vec<(String, Vec<crate::core::klippy::msg::proto::ArgValue>)> {
        let mut parser = Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        payloads
            .iter()
            .map(|payload| {
                let frame = crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).unwrap();
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    #[test]
    fn test_attach_trigger_analog_is_an_init_command_binding_the_trigger_oid() {
        let mcu = test_mcu();
        let builder = ConfigBuilder::new();
        let ld_oid = builder.create_oid().unwrap();
        // The M5a resource's oid is whatever it created — 7 here.
        add_attach_trigger_analog(&builder, ld_oid, 7).unwrap();

        let built = builder.build(&mcu).unwrap();
        let init = decoded(&mcu, &built.init);
        assert_eq!(init.len(), 1, "exactly one init command: {init:?}");
        assert_eq!(init[0].0, "ldc1612_attach_trigger_analog");
        assert_eq!(
            init[0].1,
            vec![
                crate::core::klippy::msg::proto::ArgValue::UInt8(ld_oid),
                crate::core::klippy::msg::proto::ArgValue::UInt8(7),
            ]
        );
    }

    /// A recording push target for endpoint tests.
    #[derive(Default)]
    struct Recorder {
        closed: AtomicBool,
        messages: Mutex<Vec<Value>>,
    }

    impl PushTarget for Recorder {
        fn is_closed(&self) -> bool {
            self.closed.load(Ordering::SeqCst)
        }
        fn push(&self, message: Value) {
            self.messages.lock().unwrap().push(message);
        }
    }

    fn no_op_helper(printer: &Arc<Printer>) -> Arc<BatchBulkHelper> {
        let batch_cb: BatchCb = Arc::new(|_| Box::pin(async { Ok(None) }));
        let cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        BatchBulkHelper::new(printer, batch_cb, cb, cb, BATCH_INTERVAL)
    }

    #[tokio::test]
    async fn test_dump_endpoint_registration_is_unique_and_routes_by_sensor() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        webhooks::install(&printer).unwrap();
        let helper_a = no_op_helper(&printer);
        let helper_b = no_op_helper(&printer);

        helper_a
            .add_mux_endpoint(DUMP_ENDPOINT, DUMP_KEY, "eddy", json!({"i": "a"}))
            .unwrap();
        // Same instance twice: a config error (registration is not doubled).
        let duplicate =
            helper_a.add_mux_endpoint(DUMP_ENDPOINT, DUMP_KEY, "eddy", json!({"i": "a"}));
        assert!(duplicate.is_err(), "duplicate instance rejected");

        helper_b
            .add_mux_endpoint(DUMP_ENDPOINT, DUMP_KEY, "carto", json!({"i": "b"}))
            .unwrap();

        // The two instances' handlers keep their own start responses — the
        // association the mux table routes on.
        let wh = webhooks::install(&printer).unwrap();
        let registrations = wh.take_mux_endpoints();
        assert_eq!(registrations.len(), 2);
        assert!(registrations
            .iter()
            .all(|r| r.path == DUMP_ENDPOINT && r.key == DUMP_KEY));
        let handler_for = |value: &str| {
            registrations
                .iter()
                .find(|r| r.value.as_deref() == Some(value))
                .map(|r| Arc::clone(&r.handler))
                .expect("registered instance")
        };

        let request = Request::parse(
            br#"{"method":"ldc1612/dump_ldc1612","params":{"sensor":"carto","response_template":{"method":"dump"}}}"#,
        )
        .unwrap();
        let api = Api::new();
        let recorder = Arc::new(Recorder::default());
        let context = EndpointContext {
            api: &api,
            client: Arc::clone(&recorder) as Arc<dyn PushTarget>,
        };
        let reply = handler_for("carto")
            .handle(&request, &context)
            .await
            .unwrap();
        assert_eq!(
            reply,
            json!({"i": "b"}),
            "the sensor key selected its instance"
        );

        // One batch reaches the connection, wrapped in the template.
        helper_b.process_one(&json!({"data": [[1.0, 2.0, 999.9]], "errors": 0, "overflows": 0}));
        let messages = recorder.messages.lock().unwrap().clone();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0]["method"], "dump");
        assert_eq!(messages[0]["params"]["data"][0][1], 2.0);

        // A closed connection unregisters its client.
        recorder.closed.store(true, Ordering::SeqCst);
        helper_b.process_one(&json!({"data": []}));
        assert_eq!(helper_b.client_count(), 0, "closed client dropped");
    }
}
