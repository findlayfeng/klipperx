//! `hx71x` — the HX711/HX717 load-cell ADC (upstream's
//! `klippy/extras/hx71x.py`).
//!
//! Upstream has no `[hx71x]` config section: the chip is constructed by
//! [`load_cell`](crate::core::klippy::extras::load_cell) from the *same*
//! `[load_cell …]` section, so this module exposes [`Hx71x::new`] and
//! registers no factory. What it owns:
//!
//! | piece | upstream |
//! |---|---|
//! | [`Hx71x`] / configuration + bulk stream | `HX71xBase.__init__` |
//! | [`read_sample_rate`] / [`read_gain`] choices | `HX711` / `HX717` option tables |
//! | [`convert_samples`] | `HX71xBase._convert_samples` |
//! | `errors` / `overflows` / `sample_rate` status | `HX71xBase.get_status` |
//!
//! The sample stream runs through [`FixedFreqReader`] parameterized with the
//! `"<i"` sample layout and the `query_hx71x_status oid=%c` status query
//! (upstream's `FixedFreqReader(mcu, chip_clock_smooth, "<i")` +
//! `setup_query_command("query_hx71x_status oid=%c", …)`); the wire commands
//! live in [`crate::core::klippy::cmd::hx71x`].
//!
//! The dump endpoint is **not** this chip's: upstream converts counts to grams
//! in `load_cell` and exposes `load_cell/dump_force` from there.
//!
//! # Known gaps
//!
//! `setup_trigger_analog` (the `hx71x_attach_trigger_analog` init command,
//! for `[load_cell_probe]` / `trigger_analog`) is not wired here.

use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::cmd::hx71x::{ConfigHx71x, QueryHx71x, QUERY_HX71X_STATUS_MSGFORMAT};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, ClientCb, FixedFreqReader, LifecycleCb, Sample, BATCH_INTERVAL,
};
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::mcu::{pin_number, ConfigBuilder, Mcu, McuError, McuObject};
use crate::core::klippy::pins::{PinParams, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::Printer;

// `hx71x` declares no `section!`: `[load_cell …]` owns the section and reads
// the chip options from it (`load_cell.py:load_config`).

/// Firmware sample error: the desync marker (`SAMPLE_ERROR_DESYNC =
/// -0x80000000`, the signed value upstream compares against).
const SAMPLE_ERROR_DESYNC: u32 = 0x8000_0000;

/// Firmware sample error: the read ran too long (`SAMPLE_ERROR_LONG_READ =
/// 0x40000000`).
const SAMPLE_ERROR_LONG_READ: u32 = 0x4000_0000;

/// Raw counts → ADC fraction (`1. / (1 << 23)`).
const ADC_FACTOR: f64 = 1. / (1 << 23) as f64;

/// The saturated bounds the load cell reports for its 24-bit samples
/// (`get_range`).
pub const RANGE: (i64, i64) = (-0x80_0000, 0x7F_FFFF);

// ===========================================================================
// Sensor type tables (HX711 / HX717)
// ===========================================================================

/// One chip's option tables (`HX711(config)` / `HX717(config)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorParams {
    /// `hx711` or `hx717` (`sensor_type` / the log prefix).
    pub sensor_type: &'static str,
    /// The integer `sample_rate` choices (`getchoice`'s int-keyed dict).
    pub sample_rates: &'static [i64],
    /// The default `sample_rate`.
    pub default_sample_rate: i64,
    /// The `(gain name, gain channel)` choices.
    pub gains: &'static [(&'static str, u8)],
    /// The default gain name.
    pub default_gain: &'static str,
}

/// HX711: 80/10 sps, gain channels A-128/B-32/A-64 (`hx71x.py:158-163`).
pub const HX711: SensorParams = SensorParams {
    sensor_type: "hx711",
    sample_rates: &[80, 10],
    default_sample_rate: 80,
    gains: &[("A-128", 1), ("B-32", 2), ("A-64", 3)],
    default_gain: "A-128",
};

/// HX717: 320/80/20/10 sps, gain channels A-128/B-64/A-64/B-8
/// (`hx71x.py:166-172`).
pub const HX717: SensorParams = SensorParams {
    sensor_type: "hx717",
    sample_rates: &[320, 80, 20, 10],
    default_sample_rate: 320,
    gains: &[("A-128", 1), ("B-64", 2), ("A-64", 3), ("B-8", 4)],
    default_gain: "A-128",
};

/// The `sensor_type` values this chip module answers for
/// (`HX71X_SENSOR_TYPES`).
pub const SENSOR_TYPES: [&str; 2] = ["hx711", "hx717"];

/// A `sensor_type`'s option tables (`HX71X_SENSOR_TYPES[sensor_type]`).
pub fn params_for(sensor_type: &str) -> Option<SensorParams> {
    match sensor_type {
        "hx711" => Some(HX711),
        "hx717" => Some(HX717),
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

/// The two chip pins, read as written (`dout_pin`, `sclk_pin`).
///
/// # Errors
/// Upstream's `config.get(option)`: an absent option is
/// `Option '<name>' in section '<section>' must be specified`.
pub fn read_pins(config: &ConfigWrapper) -> Result<(String, String), ConfigError> {
    let dout_pin = config.get("dout_pin", None)?;
    let sclk_pin = config.get("sclk_pin", None)?;
    Ok((dout_pin, sclk_pin))
}

/// Look both pins up (`ppins.lookup_pin(dout_pin_name)` /
/// `ppins.lookup_pin(sclk_pin_name)`).
///
/// # Errors
/// An unparseable pin description, or pins on different MCUs — upstream's
/// `"%s config error: All pins must be connected to the same MCU"`.
pub fn lookup_pins(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
    dout_pin: &str,
    sclk_pin: &str,
) -> Result<(PinParams, PinParams), ConfigError> {
    let identifier = config.identifier();
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");
    let dout = pins
        .lookup_pin(dout_pin, false, false, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    let sclk = pins
        .lookup_pin(sclk_pin, false, false, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    if sclk.chip_name != dout.chip_name {
        return Err(ConfigError::new(format!(
            "{} config error: All pins must be connected to the same MCU",
            section_name(config)
        )));
    }
    Ok((dout, sclk))
}

/// `sample_rate` — an **int-keyed** `getchoice`
/// (`config.getchoice('sample_rate', {80: 80, …})`), so the membership check
/// runs on the integer the option parses to.
///
/// # Errors
/// Upstream's `Choice '<value>' for option '<option>' in section '<section>'
/// is not a valid choice`, or `get`'s message for an absent option.
pub fn read_sample_rate(config: &ConfigWrapper, params: &SensorParams) -> Result<i64, ConfigError> {
    let value = config.get_int("sample_rate", Some(params.default_sample_rate))?;
    if !params.sample_rates.contains(&value) {
        return Err(ConfigError::new(format!(
            "Choice '{value}' for option 'sample_rate' in section '{}' is not a valid choice",
            config.identifier()
        )));
    }
    Ok(value)
}

/// `gain` — a string-keyed `getchoice` (`{'A-128': 1, …}`), returning the
/// name and the channel number the firmware wants.
///
/// # Errors
/// Upstream's `Choice '<value>' for option 'gain' in section '<section>'
/// is not a valid choice`, or `get`'s message for an absent option.
pub fn read_gain(
    config: &ConfigWrapper,
    params: &SensorParams,
) -> Result<(&'static str, u8), ConfigError> {
    let choices: Vec<&str> = params.gains.iter().map(|(name, _)| *name).collect();
    let name = config.get_choice("gain", &choices, Some(params.default_gain))?;
    let (gain, channel) = params
        .gains
        .iter()
        .find(|(gain, _)| *gain == name)
        .expect("get_choice checked the name against these choices");
    Ok((*gain, *channel))
}

// ===========================================================================
// Sample conversion (unit-tested directly)
// ===========================================================================

/// One converted sample: `(print time, raw counts, ADC fraction)`
/// (`(round(ptime, 6), val, round(val * adc_factor, 9))`).
pub type ConvertedSample = (f64, i64, f64);

/// Decode raw samples and cut the batch at the first firmware error marker
/// (`_convert_samples`: the error counts once, and the rest of the batch is
/// dropped as duplicates).
///
/// Returns the converted rows and whether an error marker was hit.
pub fn convert_samples(samples: &[Sample]) -> (Vec<ConvertedSample>, bool) {
    let mut converted = Vec::with_capacity(samples.len());
    for (ptime, raw) in samples {
        if *raw == SAMPLE_ERROR_DESYNC || *raw == SAMPLE_ERROR_LONG_READ {
            return (converted, true);
        }
        let counts = i64::from(*raw as i32);
        converted.push((
            round(*ptime, 6),
            counts,
            round(counts as f64 * ADC_FACTOR, 9),
        ));
    }
    (converted, false)
}

/// Python's `round(value, digits)`.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

// ===========================================================================
// The sensor
// ===========================================================================

/// The HX711/HX717 sensor: configuration, the bulk stream, and the
/// `BulkSensorAdc` interface `load_cell` needs
/// (`get_samples_per_second` / `get_range` / `get_status` / `add_client` /
/// `lookup_sensor_error`).
pub struct Hx71x {
    state: Arc<Hx71xState>,
}

/// Everything the callbacks and the interface share (`HX71xBase`'s fields).
struct Hx71xState {
    /// The section's last name segment.
    name: String,
    /// `hx711` or `hx717` (`sensor_type`, the log prefix).
    sensor_type: &'static str,
    /// Configured samples per second (`self.sps`).
    sample_rate: i64,
    /// The gain/channel select (`self.gain_channel`).
    gain_channel: u8,
    /// The oid the firmware assigned at build time.
    oid: u8,
    /// The MCU the chip is on (both pins must be here).
    mcu_object: Arc<McuObject>,
    /// The pin registry, for the build-time pin resolution.
    pins: Arc<PrinterPins>,
    /// The MCU's name (`dout_pin`'s chip).
    chip: String,
    /// The pins as written, resolved when the config is built.
    dout_pin: String,
    sclk_pin: String,
    /// Samples that hit an error marker since the last start
    /// (`last_error_count`).
    last_error_count: Mutex<u64>,
    /// Batches with overflows since the last clean batch
    /// (`consecutive_fails`).
    consecutive_fails: Mutex<u32>,
    /// The reader: `"<i"` samples, `query_hx71x_status` status queries.
    ffreader: FixedFreqReader,
    /// The batch helper that runs start/stop/batch for this chip; set as the
    /// last construction step.
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl Hx71x {
    /// Build the sensor from the consumer's `[load_cell …]` section
    /// (`HX71xBase.__init__`).
    ///
    /// # Errors
    /// Any option, pin or MCU complaint from the readers above, or an oid the
    /// MCU cannot hand out.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        params: &SensorParams,
    ) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        // Upstream reads the pins (and takes the MCU from `dout_pin`) before
        // the rate/gain choices.
        let (dout_pin, sclk_pin) = read_pins(config)?;
        let (dout, _sclk) = lookup_pins(config, printer, &dout_pin, &sclk_pin)?;
        let chip = dout.chip_name.clone();
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&chip))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{chip}'"))
            })?;
        let sample_rate = read_sample_rate(config, params)?;
        let (_, gain_channel) = read_gain(config, params)?;

        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(Hx71xState {
            name: section_name(config),
            sensor_type: params.sensor_type,
            sample_rate,
            gain_channel,
            oid,
            mcu_object,
            pins: printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("the loader registers `pins` before any section"),
            chip,
            dout_pin,
            sclk_pin,
            last_error_count: Mutex::new(0),
            consecutive_fails: Mutex::new(0),
            ffreader: FixedFreqReader::with_format(
                sample_rate as f64 * BATCH_INTERVAL * 2.,
                "<i",
                QUERY_HX71X_STATUS_MSGFORMAT,
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            batch_bulk: Mutex::new(None),
        });

        // The config commands and the reader's data binding belong to the
        // build: the pins only become firmware numbers once the dictionary
        // is installed.
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
                    .ok_or_else(|| "hx71x is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "hx71x is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "hx71x is gone".to_string())?;
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

    /// The sensor's object id.
    pub fn oid(&self) -> u8 {
        self.state.oid
    }

    /// Samples per second the firmware produces (`get_samples_per_second`).
    pub fn samples_per_second(&self) -> i64 {
        self.state.sample_rate
    }

    /// The saturated bounds of the 24-bit samples (`get_range`).
    pub fn range(&self) -> (i64, i64) {
        RANGE
    }

    /// The `BulkSensorAdc` status (`HX71xBase.get_status`): error and
    /// overflow counters plus the configured sample rate.
    pub fn status(&self, _eventtime: f64) -> Value {
        json!({
            "errors": *self.state.last_error_count.lock().unwrap_or_else(|p| p.into_inner()),
            "overflows": self.state.ffreader.get_last_overflows(),
            "sample_rate": self.state.sample_rate,
        })
    }

    /// A firmware error's name (`lookup_sensor_error`; the hx71x firmware
    /// reports errors as sample values, so the name is always unknown).
    pub fn lookup_sensor_error(&self, error_code: i64) -> String {
        format!("Unknown hx71x error {error_code}")
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

    /// Whether the stream has a client (tests and bookkeeping).
    #[cfg(test)]
    fn has_bulk(&self) -> bool {
        self.state
            .batch_bulk
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_some()
    }
}

impl Hx71xState {
    /// The build-time half (`__init__`'s config commands + `_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let resolved_dout = self
            .pins
            .resolve_pin(&self.chip, &self.dout_pin)
            .map_err(|err| McuError::Config(err.to_string()))?;
        let resolved_sclk = self
            .pins
            .resolve_pin(&self.chip, &self.sclk_pin)
            .map_err(|err| McuError::Config(err.to_string()))?;
        builder.add_config_cmd(&ConfigHx71x {
            oid: self.oid,
            gain_channel: self.gain_channel,
            dout_pin: pin_number(mcu, &resolved_dout, &self.chip)?,
            sclk_pin: pin_number(mcu, &resolved_sclk, &self.chip)?,
        })?;
        // Upstream: `query_hx71x oid rest_ticks=0` with `on_restart=True` —
        // a reused firmware leaves the chip powered down.
        builder.add_restart_cmd(&QueryHx71x {
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

    /// Start the bulk stream (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        *self
            .consecutive_fails
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 0;
        *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 0;

        let mcu = self.connected_mcu().map_err(to_string)?;
        let rest_ticks = mcu
            .seconds_to_clock(1. / (10. * self.sample_rate as f64))
            .map_err(to_string)? as u32;
        mcu.send_msg(&QueryHx71x {
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
        mcu.send_msg(&QueryHx71x {
            oid: self.oid,
            rest_ticks: 0,
        })
        .map_err(to_string)?;
        self.ffreader.note_end();
        tracing::info!("{} finished '{}' measurements", self.sensor_type, self.name);
        Ok(())
    }

    /// One batch: pull, convert, and restart the chip on errors or repeated
    /// overflows (`_process_batch`).
    async fn process_batch(&self, _eventtime: f64) -> Result<Option<Value>, String> {
        let prev_overflows = self.ffreader.get_last_overflows();
        let prev_error_count = *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let samples = self.ffreader.pull_samples().await.map_err(to_string)?;
        let (data, hit_error) = convert_samples(&samples);
        if hit_error {
            let mut errors = self
                .last_error_count
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            *errors += 1;
        }
        let overflows = self.ffreader.get_last_overflows() - prev_overflows;
        let errors = *self
            .last_error_count
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            - prev_error_count;
        // Decide the restart while no guard is held: the awaits below must
        // not carry a `MutexGuard` (the future has to stay `Send`).
        let restart = if errors > 0 {
            tracing::error!("{}: Forced sensor restart due to error", self.name);
            true
        } else if overflows > 0 {
            let mut fails = self
                .consecutive_fails
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            *fails += 1;
            let over_flowing = *fails > 4;
            if over_flowing {
                tracing::error!("{}: Forced sensor restart due to overflows", self.name);
            }
            over_flowing
        } else {
            *self
                .consecutive_fails
                .lock()
                .unwrap_or_else(|p| p.into_inner()) = 0;
            false
        };
        if restart {
            self.finish_measurements().await?;
            self.start_measurements().await?;
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

    #[test]
    fn test_hx711_defaults_are_80_sps_and_gain_a_128() {
        let config = wrap(Some("my_hx711"), &[]);
        assert_eq!(read_sample_rate(&config, &HX711).unwrap(), 80);
        assert_eq!(read_gain(&config, &HX711).unwrap(), ("A-128", 1));
    }

    #[test]
    fn test_hx717_defaults_are_320_sps_and_gain_a_128() {
        let config = wrap(Some("my_hx717"), &[]);
        assert_eq!(read_sample_rate(&config, &HX717).unwrap(), 320);
        assert_eq!(read_gain(&config, &HX717).unwrap(), ("A-128", 1));
    }

    #[test]
    fn test_the_choices_of_both_chips_are_the_upstream_tables() {
        assert_eq!(HX711.sample_rates, &[80, 10]);
        assert_eq!(HX717.sample_rates, &[320, 80, 20, 10]);
        assert_eq!(HX711.gains, &[("A-128", 1), ("B-32", 2), ("A-64", 3)]);
        assert_eq!(
            HX717.gains,
            &[("A-128", 1), ("B-64", 2), ("A-64", 3), ("B-8", 4)]
        );
        assert_eq!(params_for("hx711"), Some(HX711));
        assert_eq!(params_for("hx717"), Some(HX717));
        assert_eq!(params_for("ads1220"), None);
    }

    #[test]
    fn test_a_sample_rate_outside_the_chip_choices_is_refused() {
        // The corpus reads `sample_rate` as an int-keyed choice, so 77 is an
        // integer here (upstream `configfile.getchoice` takes `getint`).
        let config = wrap(Some("x"), &[("sample_rate", "77")]);
        let err = read_sample_rate(&config, &HX711).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice '77' for option 'sample_rate' in section 'load_cell x' is not a valid choice"
        );
        // 10 is only valid for one of HX717's table entries too — and 77 is
        // refused there as well.
        let config = wrap(Some("x"), &[("sample_rate", "77")]);
        assert!(read_sample_rate(&config, &HX717).is_err());
    }

    #[test]
    fn test_a_gain_outside_the_chip_choices_is_refused() {
        let config = wrap(Some("x"), &[("gain", "B-64")]);
        // B-64 is an HX717 channel, not an HX711 one.
        let err = read_gain(&config, &HX711).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice 'B-64' for option 'gain' in section 'load_cell x' is not a valid choice"
        );
        assert_eq!(read_gain(&config, &HX717).unwrap(), ("B-64", 2));
    }

    #[test]
    fn test_the_chips_choice_values_are_accepted() {
        let config = wrap(Some("x"), &[("sample_rate", "10"), ("gain", "A-64")]);
        assert_eq!(read_sample_rate(&config, &HX711).unwrap(), 10);
        assert_eq!(read_gain(&config, &HX711).unwrap(), ("A-64", 3));
        let config = wrap(Some("x"), &[("sample_rate", "20"), ("gain", "B-8")]);
        assert_eq!(read_sample_rate(&config, &HX717).unwrap(), 20);
        assert_eq!(read_gain(&config, &HX717).unwrap(), ("B-8", 4));
    }

    #[test]
    fn test_missing_chip_pins_are_refused_upstream_wording() {
        let config = wrap(Some("my_hx711"), &[("sclk_pin", "PA3")]);
        let err = read_pins(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'dout_pin' in section 'load_cell my_hx711' must be specified"
        );
    }

    #[test]
    fn test_pins_on_different_mcus_are_refused() {
        use crate::core::klippy::event::KlippyEvent;

        let printer = printer();
        // A second MCU: `[mcu aux]`, whose pins are written `aux:<pin>`.
        let aux = McuObject::new(ConfigSection::new("mcu", Some("aux")), &printer).unwrap();
        printer.add_object("mcu aux", Arc::new(aux)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);

        let config = wrap(
            Some("my_hx711"),
            &[("dout_pin", "PA5"), ("sclk_pin", "aux:PA5")],
        );
        let err = lookup_pins(&config, &printer, "PA5", "aux:PA5").unwrap_err();
        assert_eq!(
            err.to_string(),
            "my_hx711 config error: All pins must be connected to the same MCU"
        );
    }

    #[test]
    fn test_a_sensor_builds_from_its_section() {
        let printer = printer();
        let config = wrap(
            Some("my_hx717"),
            &[
                ("sensor_type", "hx717"),
                ("sclk_pin", "PA7"),
                ("dout_pin", "PJ0"),
            ],
        );
        let sensor = Hx71x::new(&config, &printer, &HX717).unwrap();
        assert_eq!(sensor.name(), "my_hx717");
        assert_eq!(sensor.samples_per_second(), 320);
        assert_eq!(sensor.range(), RANGE);
        assert_eq!(sensor.lookup_sensor_error(3), "Unknown hx71x error 3");
        assert!(sensor.has_bulk());
        let status = sensor.status(0.);
        assert_eq!(status["errors"], 0);
        assert_eq!(status["overflows"], 0);
        assert_eq!(status["sample_rate"], 320);
    }

    #[test]
    fn test_the_reader_format_is_little_endian_4_byte_samples() {
        use crate::core::klippy::extras::bulk_sensor::SampleFormat;

        // Upstream's `FixedFreqReader(mcu, chip_clock_smooth, "<i")`: four
        // little-endian bytes a sample, hence `51 // 4 == 12` samples a
        // `sensor_bulk_data` message (LC-1's `with_format` seam).
        let format = SampleFormat::parse("<i").expect("\"<i\" parses");
        assert_eq!(format.bytes_per_sample(), 4);
        assert_eq!(format.samples_per_block(), 12);
        // Little-endian: the low address is the least significant byte.
        assert_eq!(
            format.decode(&[0x34, 0x12, 0x00, 0x00]).unwrap(),
            0x0000_1234
        );
        // …and the reader the sensor builds takes the format plus this
        // chip's status query (`query_hx71x_status oid=%c`).
        let reader = FixedFreqReader::with_format(
            80. * BATCH_INTERVAL * 2.,
            "<i",
            QUERY_HX71X_STATUS_MSGFORMAT,
        )
        .expect("the hx71x reader parameterization");
        drop(reader);
    }

    #[test]
    fn test_convert_samples_decodes_little_endian_counts() {
        let (rows, hit_error) = convert_samples(&[(1.25, 0x00FF_FFFF), (2.5, 1)]);
        assert!(!hit_error);
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0].0, 1.25);
        // 24-bit sign-extended: 0x00ffffff is the top of the positive range.
        assert_eq!(rows[0].1, 0xFF_FFFF);
        assert_eq!(rows[1].1, 1);
        // The fraction is rounded to nine places (`round(val * adc_factor, 9)`).
        assert_eq!(rows[1].2, round(ADC_FACTOR, 9));
        // Timestamps are rounded to six places, the fraction to nine.
        let (rows, _) = convert_samples(&[(1.234567891, 0xFFFF_FFFF)]);
        assert_eq!(rows[0].0, 1.234568);
        // 0xffffffff is -1 as i32 — the sign extension the firmware does.
        assert_eq!(rows[0].1, -1);
        assert_eq!(rows[0].2, round(-ADC_FACTOR, 9));
    }

    #[test]
    fn test_convert_samples_cuts_the_batch_at_a_firmware_error() {
        // SAMPLE_ERROR_DESYNC (-0x80000000) and SAMPLE_ERROR_LONG_READ
        // (0x40000000) end the batch: the rows before it stay, the rest go.
        let (rows, hit) = convert_samples(&[(1.0, 5), (2.0, 0x8000_0000), (3.0, 6)]);
        assert!(hit);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].1, 5);

        let (rows, hit) = convert_samples(&[(2.0, 0x4000_0000), (3.0, 6)]);
        assert!(hit);
        assert!(rows.is_empty());

        let (rows, hit) = convert_samples(&[]);
        assert!(!hit);
        assert!(rows.is_empty());
    }
}
