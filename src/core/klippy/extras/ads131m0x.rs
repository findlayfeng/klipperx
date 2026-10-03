//! `ads131m0x` — the ADS131M02/ADS131M04 load-cell ADC (upstream's
//! `klippy/extras/ads131m0x.py`).
//!
//! Upstream has no `[ads131m0x]` config section: the chip is constructed by
//! [`load_cell`](crate::core::klippy::extras::load_cell) from the *same*
//! `[load_cell …]` section, so this module exposes [`Ads131M0x::new`] and
//! registers no factory. What it owns:
//!
//! | point | upstream |
//! |---|---|
//! | [`Ads131M0x`] / configuration + bulk stream | `ADS131M0X.__init__` |
//! | [`ADS131M02`] / [`ADS131M04`] option tables | the `ADS131M0X_SENSOR_TYPES` factories |
//! | [`convert_samples`] | `ADS131M0X._convert_samples` |
//! | `reset_chip` / `setup_chip` (on the private state) | `reset_chip` / `setup_chip` |
//! | `errors` / `overflows` / `sample_rate` status | `ADS131M0X.get_status` |
//!
//! The sample stream runs through [`FixedFreqReader`] parameterized with the
//! `"<i"` sample layout and the `query_ads131m0x_status oid=%c` status query
//! (upstream's `FixedFreqReader(mcu, chip_smooth, "<i")` +
//! `setup_query_command("query_ads131m0x_status oid=%c", …)`); the wire
//! commands live in [`crate::core::klippy::cmd::ads131m0x`].
//!
//! The dump endpoint is **not** this chip's: upstream converts counts to grams
//! in `load_cell` and exposes `load_cell/dump_force` from there.
//!
//! # Known gaps
//!
//! * `setup_trigger_analog` (the `ads131m0x_attach_trigger_analog` init
//!   command, for `[load_cell_probe]` / `trigger_analog`) is not wired here —
//!   [`crate::core::klippy::cmd::ads131m0x::Ads131M0xAttachTriggerAnalog`] is
//!   declared but nothing calls it yet.
//! * The `query_ads131m0x` halts in `_start_measurements` / `_finish_measurements`
//!   are plain sends: upstream's `send_wait_ack` (and the `spi_transfer`
//!   `minclock` that holds the reset acknowledgement past `T_REGACQ`) have no
//!   counterpart in this port's `spi_send` / `spi_transfer` pair.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::ads131m0x::{
    ConfigAds131M0x, QueryAds131M0x, QUERY_ADS131M0X_STATUS_MSGFORMAT,
};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, ClientCb, FixedFreqReader, LifecycleCb, Sample, BATCH_INTERVAL,
};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::mcu::{pin_number, ConfigBuilder, Mcu, McuError, McuObject, McuSpi};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::Printer;

// ===========================================================================
// Constants (upstream's module constants)
// ===========================================================================

/// SPI `NULL` command word.
const NULL_CMD: u16 = 0x00;
/// SPI `RESET` command word.
const RESET_CMD: u16 = 0x11;
/// The word an ADS131M0x returns after a reset.
const RESET_ACK: u16 = 0xFF22;

/// `ID` register address.
const REG_ID: u16 = 0x00;
/// `STATUS` register address.
const REG_STATUS: u16 = 0x01;
/// `MODE` register address.
const REG_MODE: u16 = 0x02;
/// `CLOCK` register address.
const REG_CLOCK: u16 = 0x03;
/// `GAIN1` register address.
const REG_GAIN1: u16 = 0x04;
/// `CFG` register address.
const REG_CFG: u16 = 0x06;

/// `MODE` value selecting 24-bit words.
const WORD24_MODE: u16 = 0x100;
/// `CLOCK` power mode: high resolution.
const PWR_MODE: u16 = 0x2;
/// `CFG` bit enabling global-chop mode.
const GC_MODE: u16 = 0x100;
/// `STATUS` field mask; `F_RESYNC` and the `DRDYx` bits are ignored.
const STATUS_REG_MASK: u16 = 0xBFFC;

/// OSR setting → `CLOCK` register code (`OSR_TO_REG`).
const OSR_TO_REG: &[(i64, u16)] = &[
    (64, 8),
    (128, 0),
    (256, 1),
    (512, 2),
    (1024, 3),
    (2048, 4),
    (4096, 5),
    (8192, 6),
    (16384, 7),
];

/// Gain setting → `GAIN1` nibble (`GAIN_TO_REG`).
const GAIN_TO_REG: &[(i64, u16)] = &[
    (1, 0x00),
    (2, 0x01),
    (4, 0x02),
    (8, 0x03),
    (16, 0x04),
    (32, 0x05),
    (64, 0x06),
    (128, 0x07),
];

/// Global-chop delay → `CFG` code (`GC_DLY_TO_REG`).
const GC_DLY_TO_REG: &[(i64, u16)] = &[
    (2, 0x00),
    (4, 0x01),
    (8, 0x02),
    (16, 0x03),
    (32, 0x04),
    (64, 0x05),
    (128, 0x06),
    (256, 0x07),
    (512, 0x08),
    (1024, 0x09),
    (2048, 0x0a),
    (4096, 0x0b),
    (8192, 0x0c),
    (16384, 0x0d),
    (32768, 0x0e),
    (65536, 0x0f),
];

/// The lowest clock frequency a config may ask for (`MINIMUM_CLOCK_FREQ`).
const MINIMUM_CLOCK_FREQ: i64 = 300_000;
/// The highest clock frequency a config may ask for (`MAXIMUM_CLOCK_FREQ`).
const MAXIMUM_CLOCK_FREQ: i64 = 8_400_000;
/// How far the nearest available sample rate may deviate
/// (`MAX_SAMPLE_RATE_DEVIATION`).
const MAX_SAMPLE_RATE_DEVIATION: f64 = 0.5;

/// The ADC's full-scale fraction (`1. / (1 << 23)`).
const ADC_FACTOR: f64 = 1. / (1 << 23) as f64;

/// The saturated bounds the load cell reports for its 24-bit samples
/// (`get_range`).
pub const RANGE: (i64, i64) = (-0x80_0000, 0x7F_FFFF);

// ===========================================================================
// Sensor type tables (ADS131M02 / ADS131M04)
// ===========================================================================

/// One chip's identity (`ADS131M02(config)` / `ADS131M04(config)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorParams {
    /// The upstream spelling (`sensor_type`, the log prefix).
    pub sensor_type: &'static str,
    /// How many ADC channels the package has.
    pub num_channels: u8,
    /// The `ID` register's upper byte this chip reports (`0x20 | num_channels`).
    pub sensor_id: u8,
}

/// ADS131M02: two channels (`ads131m0x.py:399-400`).
pub const ADS131M02: SensorParams = SensorParams {
    sensor_type: "ADS131M02",
    num_channels: 2,
    sensor_id: 0x22,
};

/// ADS131M04: four channels (`ads131m0x.py:402-403`).
pub const ADS131M04: SensorParams = SensorParams {
    sensor_type: "ADS131M04",
    num_channels: 4,
    sensor_id: 0x24,
};

/// The `sensor_type` values this chip module answers for
/// (`ADS131M0X_SENSOR_TYPES`).
pub const SENSOR_TYPES: [&str; 2] = ["ads131m02", "ads131m04"];

/// A `sensor_type`'s option tables (`ADS131M0X_SENSOR_TYPES[sensor_type]`).
pub fn params_for(sensor_type: &str) -> Option<SensorParams> {
    match sensor_type {
        "ads131m02" => Some(ADS131M02),
        "ads131m04" => Some(ADS131M04),
        _ => None,
    }
}

/// The section's last name segment (`config.get_name().split()[-1]`).
fn section_name(config: &ConfigWrapper) -> String {
    config
        .identifier()
        .split_whitespace()
        .last()
        .unwrap_or_default()
        .to_string()
}

// ===========================================================================
// Option reading (unit-tested directly)
// ===========================================================================

/// `adc_channel` — the input the load cell is wired to
/// (`getint('adc_channel', 0, minval=0, maxval=num_channels-1)`).
///
/// # Errors
/// Upstream's bounds wording for an out-of-range channel.
pub fn read_channel(config: &ConfigWrapper, num_channels: u8) -> Result<u8, ConfigError> {
    let channel = config.get_int_bounded(
        "adc_channel",
        Some(0),
        Some(0),
        Some(i64::from(num_channels) - 1),
    )?;
    Ok(channel as u8)
}

/// `pwm_clock` / `clock_freq` — the modulator clock, either borrowed from
/// another section's `frequency` or given directly.
///
/// # Errors
/// Upstream's `pwm_clock '%s' must support and specify a 'frequency' parameter`
/// (also used when the named section does not exist), the referenced section's
/// `frequency` bounds, and `clock_freq`'s own required/bounded wording.
pub fn read_clock_freq(config: &ConfigWrapper) -> Result<i64, ConfigError> {
    let Some(name) = config.get_str("pwm_clock") else {
        return config.get_int_bounded(
            "clock_freq",
            None,
            Some(MINIMUM_CLOCK_FREQ),
            Some(MAXIMUM_CLOCK_FREQ),
        );
    };
    let identifier = name.split_whitespace().collect::<Vec<_>>().join(" ");
    let Some(pwm_config) = config.sibling(&identifier) else {
        return Err(pwm_clock_error(&name));
    };
    let Some(frequency) = pwm_config.get_optional_float("frequency")? else {
        return Err(pwm_clock_error(&name));
    };
    let identifier = pwm_config.identifier();
    if frequency < MINIMUM_CLOCK_FREQ as f64 {
        return Err(ConfigError::new(format!(
            "Option 'frequency' in section '{identifier}' must have minimum of {MINIMUM_CLOCK_FREQ}"
        )));
    }
    if frequency > MAXIMUM_CLOCK_FREQ as f64 {
        return Err(ConfigError::new(format!(
            "Option 'frequency' in section '{identifier}' must have maximum of {MAXIMUM_CLOCK_FREQ}"
        )));
    }
    Ok(frequency as i64)
}

/// The missing-`frequency` complaint (`pwm_clock '%s' must support and specify
/// a 'frequency' parameter`).
fn pwm_clock_error(name: &str) -> ConfigError {
    ConfigError::new(format!(
        "pwm_clock '{name}' must support and specify a 'frequency' parameter"
    ))
}

/// `gain` — an **int-keyed** `getchoice`, returning the gain multiplier the
/// config named (1..128, default 128).
///
/// # Errors
/// Upstream's `Choice '<value>' for option 'gain' in section '<section>'
/// is not a valid choice`, or `get`'s message for an absent option.
pub fn read_gain(config: &ConfigWrapper) -> Result<i64, ConfigError> {
    let gain = config.get_int("gain", Some(128))?;
    if !GAIN_TO_REG.iter().any(|(setting, _)| *setting == gain) {
        return Err(ConfigError::new(format!(
            "Choice '{gain}' for option 'gain' in section '{}' is not a valid choice",
            config.identifier()
        )));
    }
    Ok(gain)
}

/// `enable_global_chop` / `global_chop_delay` — the global-chop setting, or
/// `None` when it is off.
///
/// # Errors
/// Upstream's `Choice '<value>' for option 'global_chop_delay' in section
/// '<section>' is not a valid choice`.
pub fn read_global_chop(config: &ConfigWrapper) -> Result<Option<i64>, ConfigError> {
    if !config.get_bool("enable_global_chop", Some(false))? {
        return Ok(None);
    }
    let delay = config.get_int("global_chop_delay", Some(16))?;
    if !GC_DLY_TO_REG.iter().any(|(setting, _)| *setting == delay) {
        return Err(ConfigError::new(format!(
            "Choice '{delay}' for option 'global_chop_delay' in section '{}' is not a valid choice",
            config.identifier()
        )));
    }
    Ok(Some(delay))
}

/// `sample_rate` — the requested output rate
/// (`getfloat('sample_rate', above=0.0, default=500.0)`).
///
/// # Errors
/// Upstream's `above` wording for a non-positive rate.
pub fn read_sample_rate(config: &ConfigWrapper) -> Result<f64, ConfigError> {
    config.get_float_bounded("sample_rate", Some(500.0), None, None, Some(0.0), None)
}

/// The output rate one OSR produces (`_calc_sample_rate`).
fn calc_sample_rate(clock_freq: i64, osr: i64, global_chop_delay: Option<i64>) -> f64 {
    let divisor = match global_chop_delay {
        Some(delay) => 2. * (delay as f64 + 3. * osr as f64),
        None => 2. * osr as f64,
    };
    clock_freq as f64 / divisor
}

/// The `(osr, sps)` nearest the requested rate, or upstream's
/// `Requested sample rate %.1f Hz is not available with the configured
/// parameters` when even the nearest is off by half the request
/// (`__init__`'s OSR search). Ties keep the smaller OSR, as the strict `<`
/// comparison in the loop does.
///
/// # Errors
/// The unavailable-rate complaint above.
pub fn fit_osr(
    clock_freq: i64,
    config_sample_rate: f64,
    global_chop_delay: Option<i64>,
) -> Result<(i64, f64), ConfigError> {
    let mut best: Option<(i64, f64)> = None;
    for (osr, _code) in OSR_TO_REG {
        let sps = calc_sample_rate(clock_freq, *osr, global_chop_delay);
        if best.is_none_or(|(_, best_sps)| {
            (sps - config_sample_rate).abs() < (best_sps - config_sample_rate).abs()
        }) {
            best = Some((*osr, sps));
        }
    }
    let (osr, sps) = best.expect("OSR_TO_REG is not empty");
    if (sps - config_sample_rate).abs() >= MAX_SAMPLE_RATE_DEVIATION * config_sample_rate {
        return Err(ConfigError::new(format!(
            "Requested sample rate {:.1} Hz is not available with the configured parameters",
            config_sample_rate
        )));
    }
    Ok((osr, sps))
}

// ===========================================================================
// Sample conversion (unit-tested directly)
// ===========================================================================

/// One converted sample: `(print time, raw counts, ADC fraction)`
/// (`(round(ptime, 6), val, round(val * adc_factor, 9))`).
pub type ConvertedSample = (f64, i64, f64);

/// Decode raw samples, dropping the ones whose top byte is neither `0x00` nor
/// `0xFF` (`_convert_samples`). Each dropped sample is named through
/// `lookup_error`; the returned names are the ones to log, one per drop.
pub fn convert_samples(
    samples: &[Sample],
    lookup_error: &dyn Fn(i64) -> String,
) -> (Vec<ConvertedSample>, Vec<String>) {
    let mut converted = Vec::with_capacity(samples.len());
    let mut errors = Vec::new();
    for (ptime, raw) in samples {
        let top_byte = (raw >> 24) & 0xFF;
        if top_byte != 0x00 && top_byte != 0xFF {
            errors.push(lookup_error(i64::from(top_byte)));
            continue;
        }
        let counts = i64::from(*raw as i32);
        converted.push((
            round(*ptime, 6),
            counts,
            round(counts as f64 * ADC_FACTOR, 9),
        ));
    }
    (converted, errors)
}

/// Python's `round(value, digits)`.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// A 16-bit SPI command/data word as the chip's 24-bit frame
/// (`_convert_to_spi_frame`): the word in the top two bytes, padded to the
/// 4-word minimum frame with null words.
fn convert_to_spi_frame(values: &[u16]) -> Vec<u8> {
    let word_count = values.len().max(4);
    let mut frame = Vec::with_capacity(word_count * 3);
    for index in 0..word_count {
        let value = values.get(index).copied().unwrap_or(NULL_CMD);
        frame.push(((value & 0xFF00) >> 8) as u8);
        frame.push((value & 0xFF) as u8);
        frame.push(0x00);
    }
    frame
}

// ===========================================================================
// The sensor
// ===========================================================================

/// The ADS131M02/ADS131M04 sensor: configuration, the bulk stream, and the
/// `BulkSensorAdc` interface `load_cell` needs
/// (`get_samples_per_second` / `get_range` / `get_status` / `add_client` /
/// `lookup_sensor_error`).
pub struct Ads131M0x {
    state: Arc<Ads131M0xState>,
}

/// Everything the callbacks and the interface share (`ADS131M0X`'s fields).
struct Ads131M0xState {
    /// The section's last name segment.
    name: String,
    /// Upstream's spelling, `ADS131M02` or `ADS131M04` (the log prefix).
    sensor_type: &'static str,
    /// The `ID` register's expected upper byte.
    sensor_id: u8,
    /// The chip's channel count.
    num_channels: u8,
    /// The selected ADC channel.
    channel: u8,
    /// The gain multiplier the config named.
    gain: i64,
    /// The global-chop delay, or `None` when global chop is off.
    gc_dly: Option<i64>,
    /// The OSR that fits `sample_rate`.
    osr: i64,
    /// The sample rate the fitted OSR produces.
    sps: f64,
    /// The oid the firmware assigned at build time.
    oid: u8,
    /// The MCU the SPI bus and the data-ready pin are on.
    mcu_object: Arc<McuObject>,
    /// The MCU's name (`spi_mcu`), for pin resolution.
    chip: String,
    /// The data-ready pin as written, resolved when the config is built.
    data_ready_pin: String,
    /// The SPI resource the chip talks over.
    spi: Arc<McuSpi>,
    /// The pin registry, for the build-time pin resolution.
    pins: Arc<PrinterPins>,
    /// The machine, for `is_fileoutput` (upstream `self.mcu.is_fileoutput`).
    printer: Weak<Printer>,
    /// Samples the current batch dropped as errors (`last_error_count`).
    last_error_count: Mutex<u64>,
    /// Firmware error code → name (`_sensor_errors`), filled at build time.
    sensor_errors: Mutex<HashMap<i64, String>>,
    /// The reader: little-endian 4-byte samples, `query_ads131m0x_status`
    /// status queries.
    ffreader: FixedFreqReader,
    /// The batch helper that runs start/stop/batch for this chip; set as the
    /// last construction step.
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Ads131M0x {
    /// Build the sensor from the consumer's `[load_cell …]` section
    /// (`ADS131M0X.__init__`).
    ///
    /// # Errors
    /// Any option, pin, SPI or MCU complaint from the readers above, or an oid
    /// the MCU cannot hand out.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        params: &SensorParams,
    ) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        // Option order follows upstream: channel, clock, gain, global chop,
        // sample rate — all before the SPI resource is built.
        let channel = read_channel(config, params.num_channels)?;
        let clock_freq = read_clock_freq(config)?;
        let gain = read_gain(config)?;
        let gc_dly = read_global_chop(config)?;
        let config_sample_rate = read_sample_rate(config)?;
        let (osr, sps) = fit_osr(clock_freq, config_sample_rate, gc_dly)?;
        tracing::info!(
            "{} '{}' configured sample_rate = {:.1} SPS",
            params.sensor_type,
            section_name(config),
            sps
        );

        // SPI, via the same `MCU_SPI_from_config` a `[spi_device]` uses.
        let setup = mcu_spi_from_config(config, printer, 1, "cs_pin", 4_000_000)?;
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

        // Data-ready pin: same MCU as the SPI bus.
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let data_ready_pin = config.get("data_ready_pin", None)?;
        let drdy = pins
            .lookup_pin(&data_ready_pin, false, false, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        if drdy.chip_name != mcu_name {
            return Err(ConfigError::new(format!(
                "{} config error: SPI communication and data_ready_pin must be on the same MCU",
                params.sensor_type
            )));
        }

        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(Ads131M0xState {
            name: section_name(config),
            sensor_type: params.sensor_type,
            sensor_id: params.sensor_id,
            num_channels: params.num_channels,
            channel,
            gain,
            gc_dly,
            osr,
            sps,
            oid,
            mcu_object,
            chip: mcu_name,
            data_ready_pin,
            spi,
            pins,
            printer: Arc::downgrade(printer),
            last_error_count: Mutex::new(0),
            sensor_errors: Mutex::new(HashMap::new()),
            ffreader: FixedFreqReader::with_format(
                sps * BATCH_INTERVAL * 2.,
                "<i",
                QUERY_ADS131M0X_STATUS_MSGFORMAT,
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            batch_bulk: Mutex::new(None),
        });

        // The config commands and the reader's data binding belong to the
        // build: the SPI oid and the pin numbers only exist once the
        // dictionary is installed.
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

        let start_state = Arc::downgrade(&state);
        let start_cb: LifecycleCb = Arc::new(move || {
            let start_state = start_state.clone();
            Box::pin(async move {
                let state = start_state
                    .upgrade()
                    .ok_or_else(|| "ads131m0x is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "ads131m0x is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "ads131m0x is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk = BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, BATCH_INTERVAL);
        *state.batch_bulk.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::clone(&batch_bulk));

        Ok(Self { state })
    }

    /// The sensor's name (the section's last segment).
    pub fn name(&self) -> &str {
        &self.state.name
    }

    /// The MCU chip this sensor lives on (`spi_mcu`, default `mcu`), as the
    /// `pins` registry keys it (`[load_cell_probe]` looks the `McuChip` up by
    /// this name for the trigger analog).
    pub fn mcu_chip_name(&self) -> &str {
        &self.state.chip
    }

    /// The sensor's object id.
    pub fn oid(&self) -> u8 {
        self.state.oid
    }

    /// The chip's channel count.
    pub fn num_channels(&self) -> u8 {
        self.state.num_channels
    }

    /// The selected ADC channel.
    pub fn channel(&self) -> u8 {
        self.state.channel
    }

    /// Samples per second the firmware produces
    /// (`get_samples_per_second`; the fitted OSR's rate, a float).
    pub fn samples_per_second(&self) -> f64 {
        self.state.sps
    }

    /// The saturated bounds of the 24-bit samples (`get_range`).
    pub fn range(&self) -> (i64, i64) {
        RANGE
    }

    /// The `BulkSensorAdc` status (`ADS131M0X.get_status`): error and overflow
    /// counters plus the configured sample rate.
    pub fn status(&self, _eventtime: f64) -> Value {
        json!({
            "errors": *self.state.last_error_count.lock().unwrap_or_else(|p| p.into_inner()),
            "overflows": self.state.ffreader.get_last_overflows(),
            "sample_rate": self.state.sps,
        })
    }

    /// A firmware error's name (`lookup_sensor_error`; the `ads131m0x_error:`
    /// enumeration's name for `error_code`, or upstream's unknown fallback).
    pub fn lookup_sensor_error(&self, error_code: i64) -> String {
        self.state
            .sensor_errors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&error_code)
            .cloned()
            .unwrap_or_else(|| format!("Unknown {} error", self.state.sensor_type))
    }

    /// Register a batch client (`add_client`, a direct pass through to the
    /// bulk helper — the first client starts the stream).
    ///
    /// # Panics
    /// Panics when called outside a Tokio runtime (the helper spawns its
    /// loop); every caller here runs inside one.
    pub fn add_client(&self, client: ClientCb) {
        if let Some(bulk) = self
            .state
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
        {
            bulk.add_client(client);
        }
    }

    /// How many clients the chip's bulk helper holds (tests and bookkeeping).
    #[cfg(test)]
    pub fn client_count(&self) -> usize {
        self.state
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
            .map_or(0, |bulk| bulk.client_count())
    }
}

impl Ads131M0xState {
    /// The build-time half (`__init__`'s config commands + `_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let spi_oid = self.spi.oid()?;
        let resolved = self
            .pins
            .resolve_pin(&self.chip, &self.data_ready_pin)
            .map_err(|err| McuError::Config(err.to_string()))?;
        let data_ready_pin = pin_number(mcu, &resolved, &self.chip)?;
        builder.add_config_cmd(&ConfigAds131M0x {
            oid: self.oid,
            spi_oid,
            channel: self.channel,
            num_channels: self.num_channels,
            data_ready_pin,
        })?;
        // Upstream: `query_ads131m0x oid rest_ticks=0` with `on_restart=True`.
        builder.add_restart_cmd(&QueryAds131M0x {
            oid: self.oid,
            rest_ticks: 0,
        })?;
        self.ffreader.bind(mcu, &self.mcu_object, self.oid)?;
        // `mcu.get_enumerations()['ads131m0x_error:']`, inverted to
        // code → name.
        let errors: HashMap<i64, String> = mcu
            .dictionary()
            .and_then(|dictionary| {
                dictionary
                    .enumeration("ads131m0x_error:")
                    .map(|enumeration| {
                        enumeration
                            .iter()
                            .map(|(name, value)| (value, name.to_string()))
                            .collect()
                    })
            })
            .unwrap_or_default();
        *self.sensor_errors.lock().unwrap_or_else(|p| p.into_inner()) = errors;
        Ok(())
    }

    fn connected_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| McuError::Config("the sensor's MCU is not connected".to_string()))
    }

    /// Whether the host runs in file-output mode (upstream
    /// `self.mcu.is_fileoutput()`, which skips the register read-backs).
    fn is_fileoutput(&self) -> bool {
        self.printer
            .upgrade()
            .is_some_and(|printer| printer.is_fileoutput())
    }

    /// Send one command word and read the chip's answer
    /// (`send_command_16`: one full-frame `spi_transfer`).
    async fn send_command_16(&self, cmd: u16) -> Result<u16, String> {
        let response = self
            .spi
            .transfer(&convert_to_spi_frame(&[cmd]))
            .await
            .map_err(to_string)?;
        if response.len() < 2 {
            return Err(format!(
                "ads131m0x: the SPI transfer for command {cmd:#x} returned {} byte(s)",
                response.len()
            ));
        }
        Ok((u16::from(response[0]) << 8) | u16::from(response[1]))
    }

    /// Read a register (`read_reg`): the `RREG` command as one transaction,
    /// then a `NULL` word whose answer carries the value.
    async fn read_reg(&self, reg: u16) -> Result<u16, String> {
        // RREG command: 101a aaaa annn nnnn.
        let cmd = 0xA000 | (reg << 7);
        self.spi
            .send(&convert_to_spi_frame(&[cmd]))
            .map_err(to_string)?;
        self.send_command_16(NULL_CMD).await
    }

    /// Write a register (`write_reg`): a `WREG` command followed by the value,
    /// as one write-only transaction.
    fn write_reg(&self, reg: u16, value: u16) -> Result<(), String> {
        // WREG command: 011a aaaa annn nnnn.
        let cmd = 0x6000 | (reg << 7);
        self.spi
            .send(&convert_to_spi_frame(&[cmd, value]))
            .map_err(to_string)
    }

    /// Validate the chip's identity and reset it (`reset_chip`).
    ///
    /// # Errors
    /// Upstream's `Invalid %s ID register …` and `Failed to reset %s …`
    /// wording, both skipped in file-output mode.
    async fn reset_chip(&self) -> Result<(), String> {
        let is_batch_mode = self.is_fileoutput();
        let id_val = self.read_reg(REG_ID).await?;
        if !is_batch_mode && (id_val >> 8) != u16::from(self.sensor_id) {
            return Err(format!(
                "Invalid {} ID register (got {:#x} vs {:#x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty chip.",
                self.sensor_type, id_val, self.sensor_id
            ));
        }
        self.send_command_16(RESET_CMD).await?;
        // Upstream waits `T_REGACQ` through the transfer's `minclock` before
        // reading the acknowledgement; this port's `spi_transfer` has no
        // `minclock`, so the read follows immediately.
        let ack = self.send_command_16(NULL_CMD).await?;
        if !is_batch_mode && ack != RESET_ACK {
            return Err(format!(
                "Failed to reset {} (got {:#x} vs {:#x}).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty chip.",
                self.sensor_type, ack, RESET_ACK
            ));
        }
        Ok(())
    }

    /// Write the `MODE` / `CLOCK` / `GAIN1` / `CFG` registers, read each back,
    /// and validate `STATUS` (`setup_chip`).
    ///
    /// # Errors
    /// Upstream's `Failed to set <REG> register to %x, got %x` and
    /// `Invalid STATUS register value %x`, all skipped in file-output mode.
    async fn setup_chip(&self) -> Result<(), String> {
        let is_batch_mode = self.is_fileoutput();

        let mode_val = WORD24_MODE;
        self.write_reg(REG_MODE, mode_val)?;
        let actual = self.read_reg(REG_MODE).await?;
        if !is_batch_mode && actual != mode_val {
            return Err(format!(
                "Failed to set MODE register to {mode_val:x}, got {actual:x}"
            ));
        }

        // Bits 11-8: channel mask, bit 5: TBM, bits 4:2: OSR, bits 1:0: PWR.
        let osr_code = code_of(OSR_TO_REG, self.osr);
        let ch_en = 1u16 << (self.channel + 8);
        let clock_val = (osr_code << 2) | ch_en | PWR_MODE;
        self.write_reg(REG_CLOCK, clock_val)?;
        let actual = self.read_reg(REG_CLOCK).await?;
        if !is_batch_mode && actual != clock_val {
            return Err(format!(
                "Failed to set CLOCK register to {clock_val:x}, got {actual:x}"
            ));
        }

        // One gain nibble per channel.
        let gain_code = code_of(GAIN_TO_REG, self.gain);
        let mut gain_val = 0u16;
        for index in 0..self.num_channels {
            gain_val |= gain_code << (index * 4);
        }
        self.write_reg(REG_GAIN1, gain_val)?;
        let actual = self.read_reg(REG_GAIN1).await?;
        if !is_batch_mode && actual != gain_val {
            return Err(format!(
                "Failed to set GAIN1 register to {gain_val:x}, got {actual:x}"
            ));
        }

        let cfg_val = match self.gc_dly {
            Some(delay) => (code_of(GC_DLY_TO_REG, delay) << 9) | GC_MODE,
            None => 0,
        };
        self.write_reg(REG_CFG, cfg_val)?;
        let actual = self.read_reg(REG_CFG).await?;
        if !is_batch_mode && actual != cfg_val {
            return Err(format!(
                "Failed to set CFG register to {cfg_val:x}, got {actual:x}"
            ));
        }

        // NULL_CMD being the last command sent leaves the next data reads
        // pointing at STATUS.
        let status_val = self.read_reg(REG_STATUS).await?;
        if !is_batch_mode && (status_val & STATUS_REG_MASK) != WORD24_MODE {
            return Err(format!("Invalid STATUS register value {status_val:x}"));
        }
        Ok(())
    }

    /// Start the bulk stream (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 0;
        let mcu = self.connected_mcu().map_err(to_string)?;
        // Be sure to halt bulk reading before resetting (upstream uses
        // `send_wait_ack` here; this port has no acknowledgement to wait for).
        mcu.send_msg(&QueryAds131M0x {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(to_string)?;
        self.reset_chip().await?;
        self.setup_chip().await?;
        let rest_ticks = mcu
            .seconds_to_clock(1. / (10. * self.sps))
            .map_err(to_string)? as u32;
        mcu.send_msg(&QueryAds131M0x {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(to_string)?;
        tracing::info!("{} starting '{}' measurements", self.sensor_type, self.name);
        self.ffreader.note_start().await.map_err(to_string)
    }

    /// Halt the bulk stream (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        // Don't use the serial connection after shutdown.
        if self.mcu_object.is_shutdown() {
            return Ok(());
        }
        let mcu = self.connected_mcu().map_err(to_string)?;
        mcu.send_msg(&QueryAds131M0x {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(to_string)?;
        self.ffreader.note_end();
        tracing::info!("{} finished '{}' measurements", self.sensor_type, self.name);
        Ok(())
    }

    /// One batch: pull, convert, report (`_process_batch`). Unlike the hx71x
    /// this chip does not restart on errors or overflows.
    async fn process_batch(&self, _eventtime: f64) -> Result<Option<Value>, String> {
        let samples = self.ffreader.pull_samples().await.map_err(to_string)?;
        let lookup = |code: i64| {
            self.sensor_errors
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .get(&code)
                .cloned()
                .unwrap_or_else(|| format!("Unknown {} error", self.sensor_type))
        };
        let (data, error_names) = convert_samples(&samples, &lookup);
        for name in &error_names {
            tracing::error!("'{}' sample error: {}", self.name, name);
        }
        if !error_names.is_empty() {
            let mut errors = self
                .last_error_count
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            *errors += error_names.len() as u64;
        }
        let error_count = *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        Ok(Some(json!({
            "data": data,
            "errors": error_count,
            "overflows": self.ffreader.get_last_overflows(),
        })))
    }
}

/// The register code for `setting` in one of the option tables.
fn code_of(table: &[(i64, u16)], setting: i64) -> u16 {
    table
        .iter()
        .find(|(key, _)| *key == setting)
        .map(|(_, code)| *code)
        .expect("every stored setting came from this table")
}

/// [`McuError`] → the batch callbacks' `String`.
fn to_string(error: McuError) -> String {
    error.to_string()
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::extras::bulk_sensor::SampleFormat;

    fn section(name: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("load_cell", name);
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
    fn wrap(name: Option<&str>, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
        ConfigWrapper::new(
            Box::leak(Box::new(section(name, options))),
            AccessTracking::shared(),
        )
    }

    /// A ready printer with `pins` and one registered `[mcu]`.
    fn printer() -> Arc<Printer> {
        use crate::core::klippy::event::KlippyEvent;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mcu = McuObject::new(ConfigSection::new("mcu", None), &printer).unwrap();
        printer.add_object("mcu", Arc::new(mcu)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    /// A minimal ADS131M02 cell's chip options (the corpus's
    /// `my_ads131m02`).
    const ADS131M02_CHIP: [(&str, &str); 4] = [
        ("sensor_type", "ads131m02"),
        ("cs_pin", "PB5"),
        ("data_ready_pin", "PB6"),
        ("clock_freq", "8192000"),
    ];

    #[test]
    fn test_the_sensor_type_tables_match_upstream() {
        assert_eq!(
            ADS131M02,
            SensorParams {
                sensor_type: "ADS131M02",
                num_channels: 2,
                sensor_id: 0x22,
            }
        );
        assert_eq!(
            ADS131M04,
            SensorParams {
                sensor_type: "ADS131M04",
                num_channels: 4,
                sensor_id: 0x24,
            }
        );
        assert_eq!(params_for("ads131m02"), Some(ADS131M02));
        assert_eq!(params_for("ads131m04"), Some(ADS131M04));
        // The `sensor_id` rule is `0x20 | num_channels`.
        assert_eq!(ADS131M02.sensor_id, 0x20 | ADS131M02.num_channels);
        assert_eq!(ADS131M04.sensor_id, 0x20 | ADS131M04.num_channels);
        assert_eq!(params_for("ads1220"), None);
        assert_eq!(SENSOR_TYPES, ["ads131m02", "ads131m04"]);
    }

    #[test]
    fn test_adc_channel_defaults_to_zero_and_is_bounded() {
        let config = wrap(Some("x"), &[]);
        assert_eq!(read_channel(&config, 2).unwrap(), 0);
        let config = wrap(Some("x"), &[("adc_channel", "1")]);
        assert_eq!(read_channel(&config, 2).unwrap(), 1);
        // The bound is `num_channels - 1`: channel 2 exists on an M04, not on
        // an M02.
        let config = wrap(Some("x"), &[("adc_channel", "2")]);
        assert_eq!(read_channel(&config, 4).unwrap(), 2);
        let err = read_channel(&config, 2).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'adc_channel' in section 'load_cell x' must have maximum of 1"
        );
    }

    #[test]
    fn test_clock_freq_is_required_and_bounded() {
        let config = wrap(Some("x"), &[]);
        let err = read_clock_freq(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'clock_freq' in section 'load_cell x' must be specified"
        );

        let config = wrap(Some("x"), &[("clock_freq", "100000")]);
        let err = read_clock_freq(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'clock_freq' in section 'load_cell x' must have minimum of 300000"
        );

        let config = wrap(Some("x"), &[("clock_freq", "9000000")]);
        let err = read_clock_freq(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'clock_freq' in section 'load_cell x' must have maximum of 8400000"
        );

        let config = wrap(Some("x"), &[("clock_freq", "8192000")]);
        assert_eq!(read_clock_freq(&config).unwrap(), 8_192_000);
    }

    #[test]
    fn test_pwm_clock_borrows_the_referenced_sections_frequency() {
        let (config, _) = Config::from_text(
            "[load_cell x]\n\
             pwm_clock: static_pwm_clock my_pin\n\
             [static_pwm_clock my_pin]\n\
             frequency: 8192000\n",
        )
        .expect("parses");
        let cell = config.get_section("load_cell x").expect("the cell");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(cell, Arc::clone(&access), None, &config);
        assert_eq!(read_clock_freq(&wrapper).unwrap(), 8_192_000);
        assert!(
            access.contains("static_pwm_clock my_pin", "frequency"),
            "the borrowed option is recorded"
        );
    }

    #[test]
    fn test_a_pwm_clock_without_a_frequency_is_refused_upstream_wording() {
        let (config, _) = Config::from_text(
            "[load_cell x]\n\
             pwm_clock: my_clock\n\
             [my_clock]\n",
        )
        .expect("parses");
        let cell = config.get_section("load_cell x").expect("the cell");
        let wrapper = ConfigWrapper::with_config(cell, AccessTracking::shared(), None, &config);
        let err = read_clock_freq(&wrapper).unwrap_err();
        assert_eq!(
            err.to_string(),
            "pwm_clock 'my_clock' must support and specify a 'frequency' parameter"
        );
    }

    #[test]
    fn test_gain_defaults_to_128_and_is_an_int_keyed_choice() {
        let config = wrap(Some("x"), &[]);
        assert_eq!(read_gain(&config).unwrap(), 128);
        let config = wrap(Some("x"), &[("gain", "64")]);
        assert_eq!(read_gain(&config).unwrap(), 64);
        let config = wrap(Some("x"), &[("gain", "3")]);
        let err = read_gain(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice '3' for option 'gain' in section 'load_cell x' is not a valid choice"
        );
    }

    #[test]
    fn test_global_chop_is_off_by_default_and_its_delay_is_an_int_choice() {
        let config = wrap(Some("x"), &[]);
        assert_eq!(read_global_chop(&config).unwrap(), None);
        let config = wrap(Some("x"), &[("enable_global_chop", "True")]);
        assert_eq!(read_global_chop(&config).unwrap(), Some(16));
        let config = wrap(
            Some("x"),
            &[("enable_global_chop", "True"), ("global_chop_delay", "3")],
        );
        let err = read_global_chop(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice '3' for option 'global_chop_delay' in section 'load_cell x' is not a valid choice"
        );
        let config = wrap(
            Some("x"),
            &[("enable_global_chop", "True"), ("global_chop_delay", "512")],
        );
        assert_eq!(read_global_chop(&config).unwrap(), Some(512));
    }

    #[test]
    fn test_sample_rate_defaults_to_500_and_fits_an_osr() {
        let config = wrap(Some("x"), &[]);
        assert_eq!(read_sample_rate(&config).unwrap(), 500.0);
        // 8.192 MHz / (2 * 8192) = 500 SPS.
        assert_eq!(fit_osr(8_192_000, 500.0, None).unwrap(), (8192, 500.0));
        // The nearest rate may not deviate by half the requested one.
        let err = fit_osr(8_192_000, 100.0, None).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Requested sample rate 100.0 Hz is not available with the configured parameters"
        );
        // Global chop divides the rate by `2 * (delay + 3 * osr)`.
        let (osr, sps) = fit_osr(8_192_000, 665.0, Some(16)).unwrap();
        assert_eq!(osr, 2048);
        assert!((sps - 665.0).abs() < 1.0, "got {sps}");
    }

    #[test]
    fn test_the_spi_frame_is_the_word_in_the_top_two_bytes() {
        // One word, padded to the four-word minimum frame: `hi lo 00` per
        // word (`_convert_to_spi_frame`).
        assert_eq!(
            convert_to_spi_frame(&[0x1234]),
            [0x12, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
        // Two words, still padded.
        assert_eq!(
            convert_to_spi_frame(&[0xA000, 0x0100]),
            [0xA0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn test_the_reader_format_is_little_endian_4_byte_samples() {
        // Upstream's `FixedFreqReader(mcu, chip_smooth, "<i")`: four
        // little-endian bytes a sample, hence `51 // 4 == 12` samples a
        // `sensor_bulk_data` message.
        let format = SampleFormat::parse("<i").expect("\"<i\" parses");
        assert_eq!(format.bytes_per_sample(), 4);
        assert_eq!(format.samples_per_block(), 12);
        assert_eq!(
            format.decode(&[0x34, 0x12, 0x00, 0x00]).unwrap(),
            0x0000_1234
        );
        // …and the reader the sensor builds takes the format plus this
        // chip's status query (`query_ads131m0x_status oid=%c`).
        let reader = FixedFreqReader::with_format(
            500. * BATCH_INTERVAL * 2.,
            "<i",
            QUERY_ADS131M0X_STATUS_MSGFORMAT,
        )
        .expect("the ads131m0x reader parameterization");
        drop(reader);
    }

    #[test]
    fn test_convert_samples_keeps_sign_extended_counts_and_names_dropped_ones() {
        let lookup = |code: i64| match code {
            1 => "SENSOR_ERROR_CRC".to_string(),
            other => format!("Unknown ADS131M02 error {other}"),
        };
        let (rows, errors) = convert_samples(
            &[
                (1.0, 0x00FF_FFFF),
                (2.0, 0xFFFF_FFFF),
                (3.0, 0x0000_0001),
                (4.0, 0x0100_0000),
                (5.0, 0x0080_0000),
            ],
            &lookup,
        );
        // The `0x01` top byte is the only dropped sample; the rest keep their
        // order.
        assert_eq!(errors, ["SENSOR_ERROR_CRC"]);
        assert_eq!(rows.len(), 4);
        assert_eq!(rows[0].0, 1.0);
        assert_eq!(rows[0].1, 0xFF_FFFF);
        assert_eq!(rows[1].1, -1);
        assert_eq!(rows[2].1, 1);
        assert_eq!(rows[3].1, 0x80_0000);
        // The fraction is `round(val * 1 / 2^23, 9)`.
        assert_eq!(rows[1].2, round(-ADC_FACTOR, 9));

        // `0xFF` and `0x00` top bytes are the valid sign extensions.
        let (rows, errors) = convert_samples(
            &[(1.0, 0xFF80_0000), (2.0, 0x007F_FFFF), (3.0, 0x0002_0000)],
            &lookup,
        );
        assert!(errors.is_empty());
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].1, -0x80_0000);
        assert_eq!(rows[1].1, 0x7F_FFFF);
    }

    #[test]
    fn test_a_sensor_builds_from_its_section_and_names_its_errors() {
        let printer = printer();
        let config = wrap(Some("my_ads131m02"), &ADS131M02_CHIP);
        let sensor = Ads131M0x::new(&config, &printer, &ADS131M02).unwrap();
        assert_eq!(sensor.name(), "my_ads131m02");
        assert_eq!(sensor.num_channels(), 2);
        assert_eq!(sensor.channel(), 0);
        assert_eq!(sensor.samples_per_second(), 500.0);
        assert_eq!(sensor.range(), RANGE);
        // Before the build fills the map, every code is unknown.
        assert_eq!(sensor.lookup_sensor_error(1), "Unknown ADS131M02 error");

        // The build's inversion: code → name, from the `ads131m0x_error:`
        // enumeration the firmware ships.
        *sensor.state.sensor_errors.lock().unwrap() = HashMap::from([
            (1, "SENSOR_ERROR_CRC".to_string()),
            (2, "SENSOR_ERROR_RESET".to_string()),
        ]);
        assert_eq!(sensor.lookup_sensor_error(1), "SENSOR_ERROR_CRC");
        assert_eq!(sensor.lookup_sensor_error(2), "SENSOR_ERROR_RESET");
        assert_eq!(sensor.lookup_sensor_error(3), "Unknown ADS131M02 error");

        let status = sensor.status(0.);
        assert_eq!(status["errors"], 0);
        assert_eq!(status["overflows"], 0);
        assert_eq!(status["sample_rate"], 500.0);
    }

    #[test]
    fn test_the_error_enumeration_is_the_firmwares() {
        // The corpus dictionary's `ads131m0x_error:` is the source of the
        // names `lookup_sensor_error` returns.
        let path = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let raw = std::fs::read(&path).expect("the dictionary is readable");
        let value: serde_json::Value = serde_json::from_slice(&raw).expect("JSON");
        let dictionary =
            crate::core::klippy::mcu::Dictionary::from_json(value).expect("a data dictionary");
        let errors: HashMap<i64, String> = dictionary
            .enumeration("ads131m0x_error:")
            .expect("the firmware names its errors")
            .iter()
            .map(|(name, code)| (code, name.to_string()))
            .collect();
        assert_eq!(
            errors,
            HashMap::from([
                (1, "SENSOR_ERROR_CRC".to_string()),
                (2, "SENSOR_ERROR_RESET".to_string()),
            ])
        );
    }

    #[test]
    fn test_pins_on_different_mcus_are_refused() {
        use crate::core::klippy::event::KlippyEvent;

        let printer = printer();
        let aux = McuObject::new(ConfigSection::new("mcu", Some("aux")), &printer).unwrap();
        printer.add_object("mcu aux", Arc::new(aux)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);

        let config = wrap(
            Some("my_ads131m02"),
            &[
                ("sensor_type", "ads131m02"),
                ("cs_pin", "PB5"),
                ("data_ready_pin", "aux:PB6"),
                ("clock_freq", "8192000"),
            ],
        );
        let err = match Ads131M0x::new(&config, &printer, &ADS131M02) {
            Err(err) => err,
            Ok(_) => panic!("expected the cross-MCU pins to be refused"),
        };
        assert_eq!(
            err.to_string(),
            "ADS131M02 config error: SPI communication and data_ready_pin must be on the same MCU"
        );
    }
}
