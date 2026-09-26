//! `ads1220` — the ADS1220 load-cell ADC (upstream's
//! `klippy/extras/ads1220.py`).
//!
//! Upstream has no `[ads1220]` config section: the chip is constructed by
//! [`load_cell`](crate::core::klippy::extras::load_cell) from the *same*
//! `[load_cell …]` section, so this module exposes [`Ads1220::new`] and
//! registers no factory. What it owns:
//!
//! | piece | upstream |
//! |---|---|
//! | [`Ads1220`] / configuration + bulk stream | `ADS1220.__init__` |
//! | [`read_gain`] / [`read_sample_rate`] / [`read_input_mux`] / [`read_vref`] | the option tables of `ADS1220` |
//! | [`register_values`] + [`read_reg_command`] / [`write_reg_command`] | `setup_chip` / `read_reg` / `write_reg` |
//! | [`convert_samples`] | `ADS1220._convert_samples` |
//! | `errors` / `overflows` / `sample_rate` status | `ADS1220.get_status` |
//!
//! The sample stream runs through [`FixedFreqReader`] parameterized with the
//! `"<i"` sample layout and the `query_ads1220_status oid=%c` status query
//! (upstream's `FixedFreqReader(mcu, chip_smooth, "<i")` +
//! `setup_query_command("query_ads1220_status oid=%c", …)`); the wire commands
//! live in [`crate::core::klippy::cmd::ads1220`]. The SPI bus itself comes from
//! the same `spi_*` options a `[spi_device]` reads
//! ([`mcu_spi_from_config`], upstream's `bus.MCU_SPI_from_config(config, 1 …)`).
//!
//! The dump endpoint is **not** this chip's: upstream converts counts to grams
//! in `load_cell` and exposes `load_cell/dump_force` from there.
//!
//! # What differs from upstream, and why
//!
//! * The chip's register validation in `_start_measurements` reports a
//!   `command_error` upstream, which shuts the machine down. Here it is an
//!   error from the batch helper's start callback, so the failure is logged and
//!   the stream stays down — same "the chip did not answer as expected" signal,
//!   without taking the host with it.
//! * `_finish_measurements` upstream waits for the firmware's ack
//!   (`send_wait_ack`); this port sends the same `query_ads1220 … rest_ticks=0`
//!   without waiting, as the other bulk sensors here do.
//!
//! # Known gaps
//!
//! `setup_trigger_analog` (the `ads1220_attach_trigger_analog` init command,
//! for `[load_cell_probe]` / `trigger_analog`) is not wired here, and the SPI
//! register traffic is only covered through the pure byte helpers above — the
//! transfer path itself needs hardware.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::ads1220::{
    ConfigAds1220, QueryAds1220, QUERY_ADS1220_STATUS_MSGFORMAT,
};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, ClientCb, FixedFreqReader, LifecycleCb, Sample, MAX_BULK_MSG_SIZE,
};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::mcu::{pin_number, ConfigBuilder, Mcu, McuError, McuObject, McuSpi};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::Printer;

// `ads1220` declares no `section!`: `[load_cell …]` owns the section and reads
// the chip options from it (`load_cell.py:load_config`).

/// Bytes one sample occupies (`BYTES_PER_SAMPLE = 4`).
pub const BYTES_PER_SAMPLE: usize = 4;

/// Samples one full `sensor_bulk_data` message carries
/// (`MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE`, and what the firmware's
/// `data_count + 4 > 51` flush produces).
pub const MAX_SAMPLES_PER_MESSAGE: usize = MAX_BULK_MSG_SIZE / BYTES_PER_SAMPLE;

/// Seconds between batches, and the span the clock regression smooths over
/// (`ads1220.py:UPDATE_INTERVAL` — the chip's own value, not the shared
/// default).
const UPDATE_INTERVAL: f64 = 0.10;

/// `RESET_CMD`: the reset command byte.
const RESET_CMD: u8 = 0x06;

/// `START_SYNC_CMD`: start conversions in the configured mode.
const START_SYNC_CMD: u8 = 0x08;

/// `RREG_CMD`: read registers.
const RREG_CMD: u8 = 0x20;

/// `WREG_CMD`: write registers.
const WREG_CMD: u8 = 0x40;

/// `NOOP_CMD`: the byte that clocks a register read out.
const NOOP_CMD: u8 = 0x00;

/// `RESET_STATE`: what register 0 reads after a reset.
const RESET_STATE: [u8; 4] = [0x0, 0x0, 0x0, 0x0];

/// Raw counts → ADC fraction (`1. / (1 << 23)`).
const ADC_FACTOR: f64 = 1. / (1 << 23) as f64;

/// `RESET_STATE` as the hex text the error message compares against.
const RESET_STATE_TEXT: &str = "[0x0, 0x0, 0x0, 0x0]";

/// The saturated bounds the load cell reports for its 24-bit samples
/// (`get_range`).
pub const RANGE: (i64, i64) = (-0x80_0000, 0x7F_FFFF);

// ===========================================================================
// Option tables
// ===========================================================================

/// The `gain` choices: name → the register value (`{'1': 0x0, …, '128': 0x7}`).
pub const GAINS: [(&str, u8); 8] = [
    ("1", 0x0),
    ("2", 0x1),
    ("4", 0x2),
    ("8", 0x3),
    ("16", 0x4),
    ("32", 0x5),
    ("64", 0x6),
    ("128", 0x7),
];

/// The normal-mode `sample_rate` choices (`sps_normal`); the position of a rate
/// in this table is the `data_rate` register field.
pub const SPS_NORMAL: [(&str, i32); 7] = [
    ("20", 20),
    ("45", 45),
    ("90", 90),
    ("175", 175),
    ("330", 330),
    ("600", 600),
    ("1000", 1000),
];

/// The turbo-mode `sample_rate` choices (`sps_turbo`).
pub const SPS_TURBO: [(&str, i32); 7] = [
    ("40", 40),
    ("90", 90),
    ("180", 180),
    ("350", 350),
    ("660", 660),
    ("1200", 1200),
    ("2000", 2000),
];

/// The `input_mux` choices: name → the MUX register field.
pub const MUXES: [(&str, u8); 12] = [
    ("AIN0_AIN1", 0b0000),
    ("AIN0_AIN2", 0b0001),
    ("AIN0_AIN3", 0b0010),
    ("AIN1_AIN2", 0b0011),
    ("AIN1_AIN3", 0b0100),
    ("AIN2_AIN3", 0b0101),
    ("AIN1_AIN0", 0b0110),
    ("AIN3_AIN2", 0b0111),
    ("AIN0_AVSS", 0b1000),
    ("AIN1_AVSS", 0b1001),
    ("AIN2_AVSS", 0b1010),
    ("AIN3_AVSS", 0b1011),
];

/// The `vref` choices: name → the VREF register field.
pub const VREFS: [(&str, u8); 4] = [
    ("internal", 0b0),
    ("REF0", 0b01),
    ("REF1", 0b10),
    ("analog_supply", 0b11),
];

/// The `input_mux` values that conflict with `vref: REF1` (`mux_conflict`:
/// every mux that uses AIN0/REFP1 or AIN3/REFN1 as an input, plus
/// `AIN0_AVSS`).
const MUX_VREF_CONFLICTS: [u8; 9] = [
    0b0000, 0b0001, 0b0010, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1011,
];

/// The `vref` value that takes its reference from the AIN0/AIN3 pins
/// (`REF1 = 0b10`).
const VREF_REF1: u8 = 0b10;

/// Default `gain` (`default='128'`).
pub const DEFAULT_GAIN: &str = "128";

/// Default `sample_rate` (`default='660'`).
pub const DEFAULT_SAMPLE_RATE: &str = "660";

/// Default `input_mux` (`default='AIN0_AIN1'`).
pub const DEFAULT_INPUT_MUX: &str = "AIN0_AIN1";

/// Default `vref` (`default='internal'`).
pub const DEFAULT_VREF: &str = "internal";

/// The default SPI clock: 512000 Hz in turbo mode, 256000 Hz otherwise
/// (`spi_speed = 512000 if self.is_turbo else 256000`).
pub fn default_spi_speed(is_turbo: bool) -> u32 {
    if is_turbo {
        512_000
    } else {
        256_000
    }
}

/// Whether a `sample_rate` name is one of the turbo-mode choices
/// (`self.is_turbo = str(self.sps) in self.sps_turbo`).
pub fn is_turbo(sample_rate: &str) -> bool {
    SPS_TURBO.iter().any(|(name, _)| *name == sample_rate)
}

/// The `sample_rate` table a mode configures (`sps_turbo` / `sps_normal`).
pub fn sps_table(is_turbo: bool) -> &'static [(&'static str, i32)] {
    if is_turbo {
        &SPS_TURBO
    } else {
        &SPS_NORMAL
    }
}

/// The samples per second a `sample_rate` name denotes (the value side of
/// whichever table holds it).
pub fn samples_per_second(sample_rate: &str) -> Option<i32> {
    SPS_NORMAL
        .iter()
        .chain(SPS_TURBO.iter())
        .find(|(name, _)| *name == sample_rate)
        .map(|(_, sps)| *sps)
}

/// The `data_rate` register field of a rate: its position in the mode's own
/// table (`list(sps_list.keys()).index(str(self.sps))`).
pub fn data_rate(sample_rate: &str, is_turbo: bool) -> Option<u8> {
    sps_table(is_turbo)
        .iter()
        .position(|(name, _)| *name == sample_rate)
        .and_then(|index| u8::try_from(index).ok())
}

/// Every `sample_rate` name both tables accept (`sps_options`, the union that
/// `getchoice` checks against).
fn sample_rate_choices() -> Vec<&'static str> {
    SPS_NORMAL
        .iter()
        .chain(SPS_TURBO.iter())
        .map(|(name, _)| *name)
        .collect()
}

/// The value a string-keyed choice table holds for `name`.
fn choice_value(table: &[(&'static str, u8)], name: &str) -> u8 {
    table
        .iter()
        .find(|(choice, _)| *choice == name)
        .map(|(_, value)| *value)
        .expect("get_choice checked the name against this table")
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

/// The section an option error names when there is nothing to quote
/// (`ADS1220 config error: …`).
fn chip_error(message: &str) -> ConfigError {
    ConfigError::new(format!("ADS1220 config error: {message}"))
}

// ===========================================================================
// Option reading (unit-tested directly)
// ===========================================================================

/// `gain` — a string-keyed `getchoice`, returning the name and the register
/// field the firmware wants.
///
/// # Errors
/// Upstream's `Choice '<value>' for option 'gain' in section '<section>'
/// is not a valid choice`, or `get`'s message for an absent option.
pub fn read_gain(config: &ConfigWrapper) -> Result<(&'static str, u8), ConfigError> {
    let names: Vec<&str> = GAINS.iter().map(|(name, _)| *name).collect();
    let name = config.get_choice("gain", &names, Some(DEFAULT_GAIN))?;
    let (gain, _) = GAINS
        .iter()
        .find(|(choice, _)| *choice == name)
        .expect("get_choice checked the name against this table");
    Ok((gain, choice_value(&GAINS, gain)))
}

/// `sample_rate` — a string-keyed `getchoice` over the union of both mode
/// tables, returning the name and the samples per second it denotes.
///
/// # Errors
/// Upstream's `Choice '<value>' for option 'sample_rate' in section
/// '<section>' is not a valid choice`, or `get`'s message for an absent
/// option.
pub fn read_sample_rate(config: &ConfigWrapper) -> Result<(&'static str, i32), ConfigError> {
    let choices = sample_rate_choices();
    let name = config.get_choice("sample_rate", &choices, Some(DEFAULT_SAMPLE_RATE))?;
    let name = choices
        .into_iter()
        .find(|choice| *choice == name)
        .expect("get_choice checked the name against these choices");
    Ok((
        name,
        samples_per_second(name).expect("the table holds every rate"),
    ))
}

/// `input_mux` — a string-keyed `getchoice`, returning the name and the MUX
/// register field.
///
/// # Errors
/// As [`read_gain`], naming `input_mux`.
pub fn read_input_mux(config: &ConfigWrapper) -> Result<(&'static str, u8), ConfigError> {
    let names: Vec<&str> = MUXES.iter().map(|(name, _)| *name).collect();
    let name = config.get_choice("input_mux", &names, Some(DEFAULT_INPUT_MUX))?;
    let (name, _) = MUXES
        .iter()
        .find(|(choice, _)| *choice == name)
        .expect("get_choice checked the name against this table");
    Ok((name, choice_value(&MUXES, name)))
}

/// `vref` — a string-keyed `getchoice`, returning the name and the VREF
/// register field.
///
/// # Errors
/// As [`read_gain`], naming `vref`.
pub fn read_vref(config: &ConfigWrapper) -> Result<(&'static str, u8), ConfigError> {
    let names: Vec<&str> = VREFS.iter().map(|(name, _)| *name).collect();
    let name = config.get_choice("vref", &names, Some(DEFAULT_VREF))?;
    let (name, _) = VREFS
        .iter()
        .find(|(choice, _)| *choice == name)
        .expect("get_choice checked the name against this table");
    Ok((name, choice_value(&VREFS, name)))
}

/// `pga_bypass` — the option, or forced on when the negative input is AVSS
/// (`force_pga_bypass = self.mux >= 0b1000`).
///
/// # Errors
/// `getboolean`'s message for a malformed value.
pub fn read_pga_bypass(config: &ConfigWrapper, mux: u8) -> Result<bool, ConfigError> {
    let pga_bypass = config.get_bool("pga_bypass", Some(false))?;
    Ok(mux >= 0b1000 || pga_bypass)
}

/// The `vref: REF1` / input-mux conflict check.
///
/// # Errors
/// Upstream's `ADS1220 config error: AIN0/REFP1 and AIN3/REFN1 cant be used as
/// a voltage reference and an input at the same time`.
pub fn check_vref_mux(mux: u8, vref: u8) -> Result<(), ConfigError> {
    if vref == VREF_REF1 && MUX_VREF_CONFLICTS.contains(&mux) {
        return Err(chip_error(
            "AIN0/REFP1 and AIN3/REFN1 cant be used as a voltage reference and an input at the \
             same time",
        ));
    }
    Ok(())
}

// ===========================================================================
// Register traffic (unit-tested directly)
// ===========================================================================

/// `hexify`: a byte string as Python renders it (`[0x0, 0xff]`).
pub fn hexify(bytes: &[u8]) -> String {
    let rendered: Vec<String> = bytes.iter().map(|byte| format!("0x{byte:x}")).collect();
    format!("[{}]", rendered.join(", "))
}

/// A register read: `RREG_CMD | (reg << 2) | (byte_count - 1)` followed by
/// `byte_count` `NOOP_CMD`s (`read_reg`).
pub fn read_reg_command(reg: u8, byte_count: u8) -> Vec<u8> {
    let mut command = vec![RREG_CMD | (reg << 2) | (byte_count - 1)];
    command.resize(1 + usize::from(byte_count), NOOP_CMD);
    command
}

/// A register write: `WREG_CMD | (reg << 2) | (len - 1)` followed by the
/// register bytes (`write_reg`).
pub fn write_reg_command(reg: u8, register_bytes: &[u8]) -> Vec<u8> {
    let mut command = vec![WREG_CMD | (reg << 2) | (register_bytes.len() as u8 - 1)];
    command.extend_from_slice(register_bytes);
    command
}

/// `setup_chip`'s four register values for a configuration: MUX/gain/bypass,
/// data rate/mode/continuous, VREF, and the unused fourth register.
pub fn register_values(
    mux: u8,
    gain: u8,
    pga_bypass: bool,
    data_rate: u8,
    is_turbo: bool,
    vref: u8,
) -> [u8; 4] {
    let mode = if is_turbo { 0x2 } else { 0x0 };
    let continuous = 0x1;
    [
        (mux << 4) | (gain << 1) | u8::from(pga_bypass),
        (data_rate << 5) | (mode << 3) | (continuous << 2),
        vref << 6,
        0x0,
    ]
}

// ===========================================================================
// Sample conversion (unit-tested directly)
// ===========================================================================

/// One converted sample: `(print time, raw counts, ADC fraction)`
/// (`(round(ptime, 6), val, round(val * adc_factor, 9))`).
pub type ConvertedSample = (f64, i64, f64);

/// Decode raw samples (`_convert_samples`). The ADS1220 firmware reports no
/// error markers, so unlike the hx71x there is no sentinel to cut the batch at:
/// every value is a sample.
pub fn convert_samples(samples: &[Sample]) -> Vec<ConvertedSample> {
    samples
        .iter()
        .map(|(ptime, raw)| {
            let counts = i64::from(*raw as i32);
            (
                round(*ptime, 6),
                counts,
                round(counts as f64 * ADC_FACTOR, 9),
            )
        })
        .collect()
}

/// Python's `round(value, digits)`.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

// ===========================================================================
// The sensor
// ===========================================================================

/// The ADS1220 sensor: configuration, the SPI chip setup, the bulk stream, and
/// the `BulkSensorAdc` interface `load_cell` needs
/// (`get_samples_per_second` / `get_range` / `get_status` / `add_client` /
/// `lookup_sensor_error`).
pub struct Ads1220 {
    state: Arc<Ads1220State>,
}

/// Everything the callbacks and the interface share (`ADS1220`'s fields).
struct Ads1220State {
    /// The section's last name segment.
    name: String,
    /// The machine, for `is_fileoutput` and the batch helper.
    printer: Weak<Printer>,
    /// Configured samples per second (`self.sps`).
    sample_rate: i32,
    /// The rate's name (`str(self.sps)`, the mode tables are keyed by it).
    sample_rate_name: &'static str,
    /// Whether the rate is a turbo-mode one (`self.is_turbo`).
    is_turbo: bool,
    /// The gain register field (`self.gain`).
    gain: u8,
    /// The MUX register field (`self.mux`).
    mux: u8,
    /// The VREF register field (`self.vref`).
    vref: u8,
    /// Whether the PGA is bypassed (`self.pga_bypass`).
    pga_bypass: bool,
    /// The oid the firmware assigned at build time.
    oid: u8,
    /// The SPI bus, shared with the chip's register traffic.
    spi: Arc<McuSpi>,
    /// The MCU the chip is on (the SPI bus's, which the DRDY pin must share).
    mcu_object: Arc<McuObject>,
    /// The MCU's name (`spi_mcu`).
    chip: String,
    /// The data-ready pin as written, resolved when the config is built.
    data_ready_pin: String,
    /// The pin registry, for the build-time pin resolution.
    pins: Arc<PrinterPins>,
    /// Samples that hit an error marker since the last start
    /// (`last_error_count`; the firmware reports none, so it stays zero).
    last_error_count: Mutex<u64>,
    /// The reader: `"<i"` samples, `query_ads1220_status` status queries.
    ffreader: FixedFreqReader,
    /// The batch helper that runs start/stop/batch for this chip; set as the
    /// last construction step.
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Ads1220 {
    /// Build the sensor from the consumer's `[load_cell …]` section
    /// (`ADS1220.__init__`).
    ///
    /// # Errors
    /// Any option, SPI, pin or MCU complaint from the readers above, or an oid
    /// the MCU cannot hand out.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        // Chip options, in upstream's order.
        let (_gain_name, gain) = read_gain(config)?;
        let (sample_rate_name, sample_rate) = read_sample_rate(config)?;
        let turbo = is_turbo(sample_rate_name);
        let (_mux_name, mux) = read_input_mux(config)?;
        let pga_bypass = read_pga_bypass(config, mux)?;
        let (_vref_name, vref) = read_vref(config)?;
        check_vref_mux(mux, vref)?;

        // SPI, via the same `MCU_SPI_from_config` a `[spi_device]` uses — mode
        // 1, at the speed the sample rate implies. Upstream's
        // `MCU_SPI_from_config` requires the select pin (`config.get`); the
        // shared helper here tolerates a missing one for `[spi_device]`, so the
        // requirement is stated first.
        config.get("cs_pin", None)?;
        let setup = mcu_spi_from_config(config, printer, 1, "cs_pin", default_spi_speed(turbo))?;
        let spi = setup.device;

        let chip = config
            .get_str("spi_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&chip))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{chip}'"))
            })?;
        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // The data-ready pin, which must be on the SPI bus's MCU.
        let data_ready_pin = config.get("data_ready_pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let drdy = pins
            .lookup_pin(&data_ready_pin, false, false, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        if drdy.chip_name != chip {
            return Err(chip_error(
                "SPI communication and data_ready_pin must be on the same MCU",
            ));
        }

        let state = Arc::new(Ads1220State {
            name: section_name(config),
            printer: Arc::downgrade(printer),
            sample_rate,
            sample_rate_name,
            is_turbo: turbo,
            gain,
            mux,
            vref,
            pga_bypass,
            oid,
            spi,
            mcu_object,
            chip,
            data_ready_pin,
            pins,
            last_error_count: Mutex::new(0),
            // `chip_smooth = self.sps * UPDATE_INTERVAL * 2`.
            ffreader: FixedFreqReader::with_format(
                sample_rate as f64 * UPDATE_INTERVAL * 2.,
                "<i",
                QUERY_ADS1220_STATUS_MSGFORMAT,
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            batch_bulk: Mutex::new(None),
        });

        // The config commands and the reader's data binding belong to the
        // build: the pins and the SPI bus only become firmware numbers once the
        // dictionary is installed.
        let build_state = Arc::downgrade(&state);
        let build_identifier = identifier.to_string();
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
                    .ok_or_else(|| "ads1220 is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "ads1220 is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "ads1220 is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, UPDATE_INTERVAL);
        *state.batch_bulk.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::clone(&batch_bulk));

        Ok(Self { state })
    }

    /// The sensor's name (the section's last segment).
    pub fn name(&self) -> &str {
        &self.state.name
    }

    /// The sensor's object id.
    pub fn oid(&self) -> u8 {
        self.state.oid
    }

    /// Samples per second the firmware produces (`get_samples_per_second`).
    pub fn samples_per_second(&self) -> i64 {
        i64::from(self.state.sample_rate)
    }

    /// The saturated bounds of the 24-bit samples (`get_range`).
    pub fn range(&self) -> (i64, i64) {
        RANGE
    }

    /// The `BulkSensorAdc` status (`ADS1220.get_status`): error and overflow
    /// counters plus the configured sample rate.
    pub fn status(&self, _eventtime: f64) -> Value {
        json!({
            "errors": *self.state.last_error_count.lock().unwrap_or_else(|p| p.into_inner()),
            "overflows": self.state.ffreader.get_last_overflows(),
            "sample_rate": self.state.sample_rate,
        })
    }

    /// A firmware error's name (`lookup_sensor_error`). Upstream's format
    /// string has no placeholder — `"Unknown ads1220 error" % (error_code,)` —
    /// so the code never reaches the text.
    pub fn lookup_sensor_error(&self, _error_code: i64) -> String {
        "Unknown ads1220 error".to_string()
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

    /// How many clients the bulk helper holds (tests and bookkeeping).
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

impl Ads1220State {
    /// The build-time half (`__init__`'s config commands + `_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let resolved = self
            .pins
            .resolve_pin(&self.chip, &self.data_ready_pin)
            .map_err(|err| McuError::Config(err.to_string()))?;
        builder.add_config_cmd(&ConfigAds1220 {
            oid: self.oid,
            spi_oid: self.spi.oid()?,
            data_ready_pin: pin_number(mcu, &resolved, &self.chip)?,
        })?;
        // Upstream arms nothing at config time: `query_ads1220` is added
        // `on_restart`, so a firmware restart leaves the chip disarmed.
        builder.add_restart_cmd(&QueryAds1220 {
            oid: self.oid,
            rest_ticks: 0,
        })?;
        self.ffreader.bind(mcu, &self.mcu_object, self.oid)?;
        Ok(())
    }

    fn connected_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| McuError::Config("the sensor's MCU is not connected".to_string()))
    }

    /// Whether the host is writing to a file instead of talking to a board
    /// (`mcu.is_fileoutput()`); the register comparisons are skipped then.
    fn is_fileoutput(&self) -> bool {
        self.printer
            .upgrade()
            .map(|printer| printer.is_fileoutput())
            .unwrap_or(false)
    }

    /// Start the bulk stream (`_start_measurements`): reset the chip,
    /// configure its registers, then arm the periodic reports.
    ///
    /// Upstream also clears `consecutive_fails` here; nothing in `ads1220.py`
    /// reads it, so this port does not carry the field.
    async fn start_measurements(&self) -> Result<(), String> {
        *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 0;
        self.reset_chip().await?;
        self.setup_chip().await?;
        let mcu = self.connected_mcu().map_err(to_string)?;
        let rest_ticks = mcu
            .seconds_to_clock(1. / (10. * f64::from(self.sample_rate)))
            .map_err(to_string)? as u32;
        mcu.send_msg(&QueryAds1220 {
            oid: self.oid,
            rest_ticks,
        })
        .map_err(to_string)?;
        tracing::info!("ADS1220 starting '{}' measurements", self.name);
        self.ffreader.note_start().await.map_err(to_string)
    }

    /// Halt the bulk stream (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        // Don't use the serial connection after shutdown.
        if self.mcu_object.is_shutdown() {
            return Ok(());
        }
        let mcu = self.connected_mcu().map_err(to_string)?;
        mcu.send_msg(&QueryAds1220 {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(to_string)?;
        self.ffreader.note_end();
        tracing::info!("ADS1220 finished '{}' measurements", self.name);
        Ok(())
    }

    /// One batch (`_process_batch`): pull and convert. Unlike the hx71x, this
    /// chip has no error markers and no restart-on-failure path.
    async fn process_batch(&self, _eventtime: f64) -> Result<Option<Value>, String> {
        let samples = self.ffreader.pull_samples().await.map_err(to_string)?;
        let data = convert_samples(&samples);
        let errors = *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        Ok(Some(json!({
            "data": data,
            "errors": errors,
            "overflows": self.ffreader.get_last_overflows(),
        })))
    }

    /// Reset the chip and validate the state it comes back in (`reset_chip`).
    async fn reset_chip(&self) -> Result<(), String> {
        // The reset command takes 50us to complete.
        self.send_command(RESET_CMD)?;
        // Read the startup register state and validate it.
        let value = self.read_reg(0x0, 4).await?;
        if value != RESET_STATE {
            if self.is_fileoutput() {
                return Ok(());
            }
            return Err(format!(
                "Invalid ads1220 reset state (got {} vs {RESET_STATE_TEXT}).\nThis is generally \
                 indicative of connection problems\n(e.g. faulty wiring) or a faulty ADS1220 chip.",
                hexify(&value)
            ));
        }
        Ok(())
    }

    /// Write the configured registers and start conversions (`setup_chip`).
    async fn setup_chip(&self) -> Result<(), String> {
        let data_rate = data_rate(self.sample_rate_name, self.is_turbo).ok_or_else(|| {
            format!(
                "sample rate '{}' is not in the mode table",
                self.sample_rate_name
            )
        })?;
        let reg_values = register_values(
            self.mux,
            self.gain,
            self.pga_bypass,
            data_rate,
            self.is_turbo,
            self.vref,
        );
        self.write_reg(0x0, &reg_values).await?;
        // Start measurements immediately.
        self.send_command(START_SYNC_CMD)
    }

    /// Read `byte_count` bytes from register `reg` (`read_reg`): the reply's
    /// first byte is the echoed command, the rest are the register contents.
    async fn read_reg(&self, reg: u8, byte_count: u8) -> Result<Vec<u8>, String> {
        let response = self
            .spi
            .transfer(&read_reg_command(reg, byte_count))
            .await
            .map_err(to_string)?;
        Ok(response.into_iter().skip(1).collect())
    }

    /// Shift one command byte out (`send_command`).
    fn send_command(&self, command: u8) -> Result<(), String> {
        self.spi.send(&[command]).map_err(to_string)
    }

    /// Write a register and read it back (`write_reg`); a mismatch means the
    /// chip did not take the setting.
    async fn write_reg(&self, reg: u8, register_bytes: &[u8]) -> Result<(), String> {
        self.spi
            .send(&write_reg_command(reg, register_bytes))
            .map_err(to_string)?;
        let stored = self.read_reg(reg, register_bytes.len() as u8).await?;
        if stored != register_bytes {
            if self.is_fileoutput() {
                return Ok(());
            }
            return Err(format!(
                "Failed to set ADS1220 register [0x{reg:x}] to {}: got {}. This may be a \
                 connection problem (e.g. faulty wiring)",
                hexify(register_bytes),
                hexify(&stored)
            ));
        }
        Ok(())
    }
}

/// [`McuError`] → the batch callbacks' `String`.
fn to_string(error: McuError) -> String {
    error.to_string()
}

impl std::fmt::Debug for Ads1220 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ads1220")
            .field("name", &self.state.name)
            .field("oid", &self.state.oid)
            .finish_non_exhaustive()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};

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

    /// The corpus `[load_cell my_ads1220]` options.
    const CHIP: [(&str, &str); 2] = [("cs_pin", "PA0"), ("data_ready_pin", "PA1")];

    #[test]
    fn test_the_option_tables_are_the_upstream_tables() {
        assert_eq!(GAINS.len(), 8);
        assert_eq!(GAINS[7], ("128", 0x7));
        assert_eq!(SPS_NORMAL.len(), 7);
        assert_eq!(SPS_TURBO.len(), 7);
        assert_eq!(MUXES.len(), 12);
        assert_eq!(MUXES[11], ("AIN3_AVSS", 0b1011));
        assert_eq!(
            VREFS,
            [
                ("internal", 0b0),
                ("REF0", 0b01),
                ("REF1", 0b10),
                ("analog_supply", 0b11)
            ]
        );
        assert_eq!(MAX_SAMPLES_PER_MESSAGE, 12);
        assert_eq!(BYTES_PER_SAMPLE, 4);
        // The chip's own batch period (`ads1220.py:UPDATE_INTERVAL`), which
        // also smooths the clock regression.
        assert_eq!(UPDATE_INTERVAL, 0.10);
    }

    #[test]
    fn test_the_defaults_are_gain_128_and_a_turbo_660_sps_rate() {
        let config = wrap(Some("my_ads1220"), &[]);
        assert_eq!(read_gain(&config).unwrap(), ("128", 0x7));
        assert_eq!(read_sample_rate(&config).unwrap(), ("660", 660));
        assert!(is_turbo("660"));
        assert_eq!(read_input_mux(&config).unwrap(), ("AIN0_AIN1", 0b0000));
        assert_eq!(read_vref(&config).unwrap(), ("internal", 0b0));
        assert!(!read_pga_bypass(&config, 0b0000).unwrap());
        assert_eq!(default_spi_speed(true), 512_000);
        assert_eq!(default_spi_speed(false), 256_000);
    }

    #[test]
    fn test_the_mode_tables_split_the_rates_the_upstream_way() {
        // 660, 2000 are turbo-only; 175, 20 normal-only; 90 is in both.
        assert!(is_turbo("660") && is_turbo("2000"));
        assert!(!is_turbo("175") && !is_turbo("20"));
        assert!(sps_table(false).contains(&("90", 90)));
        assert!(sps_table(true).contains(&("90", 90)));
        assert!(!sps_table(false).contains(&("660", 660)));
        assert!(!sps_table(true).contains(&("175", 175)));
        // The data rate field is the position in the *mode's* table.
        assert_eq!(data_rate("660", true), Some(4));
        assert_eq!(data_rate("175", false), Some(3));
        assert_eq!(data_rate("175", true), None);
        assert_eq!(samples_per_second("175"), Some(175));
    }

    #[test]
    fn test_a_rate_outside_the_choices_is_refused() {
        let config = wrap(Some("my_ads1220"), &[("sample_rate", "77")]);
        let err = read_sample_rate(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice '77' for option 'sample_rate' in section 'load_cell my_ads1220' is not a valid choice"
        );
    }

    #[test]
    fn test_gain_mux_and_vref_choices_are_checked() {
        let config = wrap(Some("x"), &[("gain", "1")]);
        assert_eq!(read_gain(&config).unwrap(), ("1", 0x0));
        let config = wrap(Some("x"), &[("gain", "3")]);
        assert_eq!(
            read_gain(&config).unwrap_err().to_string(),
            "Choice '3' for option 'gain' in section 'load_cell x' is not a valid choice"
        );
        let config = wrap(Some("x"), &[("input_mux", "AIN3_AIN0")]);
        assert_eq!(
            read_input_mux(&config).unwrap_err().to_string(),
            "Choice 'AIN3_AIN0' for option 'input_mux' in section 'load_cell x' is not a valid choice"
        );
        let config = wrap(Some("x"), &[("vref", "ref1")]);
        assert_eq!(
            read_vref(&config).unwrap_err().to_string(),
            "Choice 'ref1' for option 'vref' in section 'load_cell x' is not a valid choice"
        );
    }

    #[test]
    fn test_the_pga_is_bypassed_when_avss_is_the_negative_input() {
        // `AIN0_AVSS` and the other AVSS inputs force the bypass on.
        assert!(read_pga_bypass(&wrap(Some("x"), &[]), 0b1000).unwrap());
        assert!(read_pga_bypass(&wrap(Some("x"), &[]), 0b1011).unwrap());
        // A differential input leaves the option in charge.
        assert!(!read_pga_bypass(&wrap(Some("x"), &[]), 0b0000).unwrap());
        let config = wrap(Some("x"), &[("pga_bypass", "true")]);
        assert!(read_pga_bypass(&config, 0b0000).unwrap());
    }

    #[test]
    fn test_ref1_and_a_shared_input_pin_is_refused() {
        // REF1 (`0b10`) takes the reference from AIN0/AIN3, so a mux that uses
        // either as an input collides.
        let err = check_vref_mux(0b0010, VREF_REF1).unwrap_err();
        assert_eq!(
            err.to_string(),
            "ADS1220 config error: AIN0/REFP1 and AIN3/REFN1 cant be used as a voltage reference and an input at the same time"
        );
        assert!(check_vref_mux(0b0000, VREF_REF1).is_err());
        assert!(check_vref_mux(0b1011, VREF_REF1).is_err());
        // `AIN1_AVSS` touches neither pin, and REF1 is fine with any other
        // reference choice.
        assert!(check_vref_mux(0b1001, VREF_REF1).is_ok());
        assert!(check_vref_mux(0b0010, 0b0).is_ok());
    }

    #[test]
    fn test_the_register_commands_and_values_match_the_datasheet_layout() {
        // Defaults: mux AIN0_AIN1, gain 128, no bypass, data rate 4 (660 sps
        // in turbo), turbo mode + continuous, internal vref.
        assert_eq!(
            register_values(0b0000, 0x7, false, 4, true, 0b0),
            [0x0E, 0x94, 0x0, 0x0]
        );
        // AIN0_AVSS is 0b1000 with the bypass forced on; normal mode drops the
        // turbo bit.
        assert_eq!(
            register_values(0b1000, 0x0, true, 3, false, 0b10),
            [0x81, 0x64, 0x80, 0x0]
        );
        // A read of register 0 with 4 bytes: RREG | count-1, then NOOPs.
        assert_eq!(read_reg_command(0x0, 4), vec![0x23, 0x0, 0x0, 0x0, 0x0]);
        assert_eq!(read_reg_command(0x3, 1), vec![0x2C, 0x0]);
        // A write of all four registers.
        assert_eq!(
            write_reg_command(0x0, &[0x0E, 0x94, 0x0, 0x0]),
            vec![0x43, 0x0E, 0x94, 0x0, 0x0]
        );
        assert_eq!(hexify(&[0x0, 0xff]), "[0x0, 0xff]");
        assert_eq!(hexify(&RESET_STATE), RESET_STATE_TEXT);
    }

    #[test]
    fn test_the_reader_format_is_little_endian_4_byte_samples() {
        use crate::core::klippy::extras::bulk_sensor::SampleFormat;

        // Upstream's `FixedFreqReader(mcu, chip_smooth, "<i")`: four
        // little-endian bytes a sample, hence `51 // 4 == 12` samples a
        // `sensor_bulk_data` message (LC-1's `with_format` seam).
        let format = SampleFormat::parse("<i").expect("\"<i\" parses");
        assert_eq!(format.bytes_per_sample(), BYTES_PER_SAMPLE);
        assert_eq!(format.samples_per_block(), MAX_SAMPLES_PER_MESSAGE);
        // Little-endian: the low address is the least significant byte.
        assert_eq!(
            format.decode(&[0x34, 0x12, 0x00, 0x00]).unwrap(),
            0x0000_1234
        );
        // …and the reader the sensor builds takes the format plus this chip's
        // status query (`query_ads1220_status oid=%c`).
        let reader = FixedFreqReader::with_format(
            660. * UPDATE_INTERVAL * 2.,
            "<i",
            QUERY_ADS1220_STATUS_MSGFORMAT,
        )
        .expect("the ads1220 reader parameterization");
        drop(reader);
    }

    #[test]
    fn test_convert_samples_has_no_error_sentinels() {
        // The ADS1220 firmware sends no error markers: `0x80000000` is a
        // sample like any other (the full-scale negative count), and the whole
        // batch survives.
        let rows = convert_samples(&[(1.25, 0x00FF_FFFF), (2.5, 1), (3.5, 0x8000_0000)]);
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].0, 1.25);
        assert_eq!(rows[0].1, 0xFF_FFFF);
        assert_eq!(rows[1].1, 1);
        assert_eq!(rows[1].2, round(ADC_FACTOR, 9));
        assert_eq!(rows[2].1, -0x8000_0000);
        // Timestamps round to six places, fractions to nine.
        let rows = convert_samples(&[(1.234567891, 0xFFFF_FFFF)]);
        assert_eq!(rows[0].0, 1.234568);
        assert_eq!(rows[0].1, -1);
        assert_eq!(rows[0].2, round(-ADC_FACTOR, 9));
        assert!(convert_samples(&[]).is_empty());
    }

    #[test]
    fn test_a_sensor_builds_from_its_section() {
        let printer = printer();
        let config = wrap(
            Some("my_ads1220"),
            &[
                ("sensor_type", "ads1220"),
                ("cs_pin", "PA0"),
                ("data_ready_pin", "PA1"),
            ],
        );
        let sensor = Ads1220::new(&config, &printer).unwrap();
        assert_eq!(sensor.name(), "my_ads1220");
        assert_eq!(sensor.samples_per_second(), 660);
        assert_eq!(sensor.range(), RANGE);
        // Upstream's format string carries no placeholder, so the code never
        // reaches the text.
        assert_eq!(sensor.lookup_sensor_error(3), "Unknown ads1220 error");
        let status = sensor.status(0.);
        assert_eq!(status["errors"], 0);
        assert_eq!(status["overflows"], 0);
        assert_eq!(status["sample_rate"], 660);
    }

    #[test]
    fn test_missing_chip_options_are_refused_upstream_wording() {
        let printer = printer();
        let config = wrap(Some("my_ads1220"), &[("cs_pin", "PA0")]);
        let err = Ads1220::new(&config, &printer).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'data_ready_pin' in section 'load_cell my_ads1220' must be specified"
        );

        let config = wrap(Some("my_ads1220"), &[("data_ready_pin", "PA1")]);
        let err = Ads1220::new(&config, &printer).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'cs_pin' in section 'load_cell my_ads1220' must be specified"
        );
    }

    #[test]
    fn test_the_data_ready_pin_and_the_spi_bus_must_share_an_mcu() {
        use crate::core::klippy::event::KlippyEvent;

        let printer = printer();
        let aux = McuObject::new(ConfigSection::new("mcu", Some("aux")), &printer).unwrap();
        printer.add_object("mcu aux", Arc::new(aux)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);

        let config = wrap(
            Some("my_ads1220"),
            &[("cs_pin", "PA0"), ("data_ready_pin", "aux:PA1")],
        );
        let err = Ads1220::new(&config, &printer).unwrap_err();
        assert_eq!(
            err.to_string(),
            "ADS1220 config error: SPI communication and data_ready_pin must be on the same MCU"
        );
    }

    #[tokio::test]
    async fn test_the_first_client_registers_with_the_bulk_helper() {
        let printer = printer();
        let config = wrap(Some("my_ads1220"), &CHIP);
        let sensor = Ads1220::new(&config, &printer).unwrap();
        assert_eq!(sensor.client_count(), 0);
        // `load_cell`'s client, and the pass-through to `BatchBulkHelper`
        // (`add_client`). The spawned batch loop has not been polled yet, so
        // the count is the registration this call just made.
        sensor.add_client(Arc::new(|_message: &Value| true));
        assert_eq!(sensor.client_count(), 1);
    }
}
