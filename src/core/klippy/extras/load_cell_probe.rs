//! `load_cell_probe` — the load cell as a Z probe (`[load_cell_probe]`,
//! upstream's `klippy/extras/load_cell_probe.py`).
//!
//! The section is the probe's *and* the cell's: it carries `sensor_type` and
//! the chip's own options like `[load_cell]`, plus the probe's `z_offset` /
//! `speed` and the continuous-tare filter, and it registers the `probe` object.
//! This module owns:
//!
//! | piece | upstream |
//! |---|---|
//! | [`section!`](crate::core::klippy::load) + option reading | `load_config` / `LoadCellPrinterProbe.__init__` |
//! | [`LoadCellProbeConfigHelper`] (tare/trigger/safety) | `LoadCellProbeConfigHelper` |
//! | [`ProbeParameters`] (`LoadCellParameterHelper`) | `LoadCellParameterHelper` |
//! | the continuous-tare filter design | `ContinuousTareFilterHelper` / `ContinuousTareFilter` |
//! | the trigger analog + SOS filter | `MCU_trigger_analog` / `MCU_SosFilter` |
//! | the `probe` object | `LoadCellPrinterProbe` |
//!
//! The cell itself (chips, calibration, `LOAD_CELL_*`, `load_cell/dump_force`)
//! is [`load_cell`](crate::core::klippy::extras::load_cell): `LoadCell::new`
//! reads `sensor_type` from this same section, exactly as upstream's
//! `load_config` hands `sensor_class(config)` to `LoadCell(config, sensor)`.
//!
//! # Known gaps (the probe run path)
//!
//! The section **loads** and registers the `probe` object, but the probing
//! machinery upstream adds on top is not wired yet, so `PROBE` /
//! `BED_MESH_CALIBRATE` do not reach it:
//!
//! * the `probe` session (`LoadCellProbingMove` / `TappingMove` / `TapSession`
//!   / `SampleAveragingHelper`) and `LOAD_CELL_TEST_TAP`; wiring it needs
//!   `probe.rs`'s [`lookup_probe_session`] to answer for this object, which
//!   means editing `probe.rs` — out of this unit's boundary.
//! * the sample collector (`LoadCellSampleCollector`) and the sensor-side
//!   `setup_trigger_analog` attach (`ads1220`/`hx71x`/`ads131m0x`, each of
//!   which documents that gap too), plus the ascent piecewise least-squares
//!   fit (`LCBestFit`).
//! * the `load_cell_probe/dump_taps` mux endpoint.
//!
//! Until those land the corpus case `load_cell.test` still fails at run time
//! and stays on `upstream.rs`'s `IGNORED` list.
//!
//! [`lookup_probe_session`]: crate::core::klippy::extras::probe

use std::sync::Arc;

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::load_cell::LoadCell;
use crate::core::klippy::extras::probe::{ProbeOffsets, ProbeParams};
use crate::core::klippy::extras::trigger_analog::{calc_frac_bits, to_fixed_32, DigitalFilter};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuChip, McuTriggerAnalog, SosFilter, SosFilterDesign};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("load_cell_probe", order = 40, load = load_config);

/// The object the probe is looked up under (upstream's
/// `printer.add_object('probe', self)`).
const PROBE_OBJECT: &str = "probe";

/// The largest number of SOS sections the continuous-tare filter may use
/// (`MCU_SosFilter(self._mcu, cmd_queue, 4)`).
const MAX_FILTER_SECTIONS: u8 = 4;

/// `ContinuousTareFilterHelper`'s `drift_filter_delay` bound (`minval=1,
/// maxval=2`).
const DRIFT_DELAY: (i64, i64) = (1, 2);

/// `buzz_filter_delay`'s bound, as `drift_filter_delay`'s.
const BUZZ_DELAY: (i64, i64) = (1, 2);

/// The most notches `notch_filter_frequencies` may carry (`max_len=2`).
const MAX_NOTCHES: usize = 2;

// ===========================================================================
// The configuration helpers (unit-tested directly)
// ===========================================================================

/// `LoadCellProbeConfigHelper`: the tare/trigger/safety options and the
/// conversions that depend on the sensor's rate and the cell's calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadCellProbeConfigHelper {
    /// `tare_time`, in seconds (default four 60 Hz power cycles).
    tare_time: f64,
    /// `trigger_force`, in grams.
    trigger_force: f64,
    /// `force_safety_limit`, in grams.
    force_safety_limit: f64,
}

impl LoadCellProbeConfigHelper {
    /// Read the section's probe options (`LoadCellProbeConfigHelper.__init__`).
    ///
    /// # Errors
    /// As the bounded readers: `tare_time` in `[0.01, 1.0]`, `trigger_force`
    /// in `[10, 250]`, `force_safety_limit` in `[100, 10000]`.
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        Ok(Self {
            tare_time: config.get_float_bounded(
                "tare_time",
                Some(4. / 60.),
                Some(0.01),
                Some(1.0),
                None,
                None,
            )?,
            trigger_force: config.get_float_bounded(
                "trigger_force",
                Some(75.),
                Some(10.),
                Some(250.),
                None,
                None,
            )?,
            force_safety_limit: config.get_float_bounded(
                "force_safety_limit",
                Some(2000.),
                Some(100.),
                Some(10000.),
                None,
                None,
            )?,
        })
    }

    /// The configured `tare_time`.
    pub fn tare_time(&self) -> f64 {
        self.tare_time
    }

    /// `get_tare_samples`: the tare window in samples, never below two
    /// (`max(2, math.ceil(tare_time * sps))`).
    pub fn tare_samples(&self, samples_per_second: f64) -> i64 {
        let samples = (self.tare_time * samples_per_second).ceil() as i64;
        samples.max(2)
    }

    /// The configured `trigger_force`, in grams (`get_trigger_force_grams`).
    pub fn trigger_force_grams(&self) -> f64 {
        self.trigger_force
    }

    /// The configured `force_safety_limit`, in grams (`get_safety_limit_grams`).
    pub fn safety_limit_grams(&self) -> f64 {
        self.force_safety_limit
    }

    /// `get_safety_range`: the raw-count band the trigger analog enforces,
    /// computed from the calibration and clamped to the sensor's real range.
    ///
    /// # Errors
    /// `Load cell force_safety_limit exceeds sensor range!` when the band would
    /// leave the sensor's own range (`get_safety_range`).
    pub fn safety_range(
        &self,
        counts_per_gram: f64,
        reference_tare_counts: i64,
        sensor_range: (i64, i64),
    ) -> Result<(i32, i32), ConfigError> {
        let safety_counts = (counts_per_gram * self.force_safety_limit) as i64;
        let safety_min = reference_tare_counts - safety_counts;
        let safety_max = reference_tare_counts + safety_counts;
        let (sensor_min, sensor_max) = sensor_range;
        if safety_min <= sensor_min || safety_max >= sensor_max {
            return Err(ConfigError::new(
                "Load cell force_safety_limit exceeds sensor range!".to_string(),
            ));
        }
        // `int()` of an in-range value; the sensor's range is far inside i32.
        Ok((safety_min as i32, safety_max as i32))
    }
}

/// `get_grams_per_count`: `1 / counts_per_gram`.
///
/// # Errors
/// `counts_per_gram could be so large` is upstream's `OverflowError` guard:
/// a value at or above `1 << 29` cannot be filtered.
pub fn grams_per_count(counts_per_gram: f64) -> Result<f64, ConfigError> {
    if counts_per_gram >= (1u64 << 29) as f64 {
        return Err(ConfigError::new(
            "counts_per_gram value is too large to filter".to_string(),
        ));
    }
    Ok(1. / counts_per_gram)
}

/// `LoadCellParameterHelper`: the probe's motion parameters, with the lift the
/// tap process does itself folded into `load_cell_retract_dist`.
pub struct ProbeParameters {
    /// The section's probe parameters, before a command overrides them.
    defaults: ProbeParams,
}

impl ProbeParameters {
    /// Read the section's probe parameters (`LoadCellParameterHelper.__init__`
    /// → `ProbeParameterHelper`).
    ///
    /// # Errors
    /// As the bounded readers; note `samples_result` defaults to `average`
    /// here, as upstream's `ProbeParameterHelper` does.
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let probe_speed =
            config.get_float_bounded("speed", Some(5.0), None, None, Some(0.0), None)?;
        let lift_speed = config.get_float_bounded(
            "lift_speed",
            Some(probe_speed),
            None,
            None,
            Some(0.0),
            None,
        )?;
        let samples = config.get_int_bounded("samples", Some(1), Some(1), None)?;
        let sample_retract_dist = config.get_float_bounded(
            "sample_retract_dist",
            Some(2.0),
            None,
            None,
            Some(0.0),
            None,
        )?;
        let samples_result =
            config.get_choice("samples_result", &["median", "average"], Some("average"))?;
        let samples_tolerance = config.get_float_bounded(
            "samples_tolerance",
            Some(0.100),
            Some(0.0),
            None,
            None,
            None,
        )?;
        let samples_tolerance_retries =
            config.get_int_bounded("samples_tolerance_retries", Some(0), Some(0), None)?;
        Ok(Self {
            defaults: ProbeParams {
                probe_speed,
                lift_speed,
                samples,
                sample_retract_dist,
                samples_tolerance,
                samples_tolerance_retries,
                samples_result,
            },
        })
    }

    /// The section's parameters (`get_probe_params` / `defaults`).
    pub fn params(&self) -> &ProbeParams {
        &self.defaults
    }
}

/// `ProbeOffsetsHelper`: the probe's XYZ offsets (`get_offsets`).
///
/// # Errors
/// `z_offset` is required; `x_offset`/`y_offset` default to zero.
pub fn read_offsets(config: &ConfigWrapper) -> Result<ProbeOffsets, ConfigError> {
    Ok(ProbeOffsets {
        x: config.get_float("x_offset", Some(0.))?,
        y: config.get_float("y_offset", Some(0.))?,
        z: config.get_float("z_offset", None)?,
    })
}

/// The continuous-tare filter's options (`ContinuousTareFilterHelper`).
#[derive(Debug, Clone, PartialEq)]
pub struct ContinuousTareOptions {
    /// `drift_filter_cutoff_frequency` (the highpass).
    pub drift: Option<f64>,
    /// `drift_filter_delay` (the highpass order).
    pub drift_delay: i64,
    /// `buzz_filter_cutoff_frequency` (the lowpass).
    pub buzz: Option<f64>,
    /// `buzz_filter_delay` (the lowpass order).
    pub buzz_delay: i64,
    /// `notch_filter_frequencies`.
    pub notches: Vec<f64>,
    /// `notch_filter_quality`.
    pub notch_quality: f64,
}

impl ContinuousTareOptions {
    /// Read the filter options and design the section's filter
    /// (`ContinuousTareFilterHelper.__init__`).
    ///
    /// The bounds move with the sensor's rate: `max_filter_frequency` is the
    /// Nyquist floor, and `buzz_filter_cutoff_frequency` may not reach it.
    ///
    /// # Errors
    /// As the bounded readers, plus the list checks upstream applies element by
    /// element (`_validate_float_list`) and the SciPy refusal for a butter
    /// design the table misses.
    pub fn read(
        config: &ConfigWrapper,
        samples_per_second: f64,
    ) -> Result<(Self, DigitalFilter), ConfigError> {
        let max_filter_frequency = (samples_per_second / 2.).floor();
        let drift = optional_bounded(
            config,
            "drift_filter_cutoff_frequency",
            Some(0.1),
            Some(20.0),
        )?;
        let drift_delay = config.get_int_bounded(
            "drift_filter_delay",
            Some(2),
            Some(DRIFT_DELAY.0),
            Some(DRIFT_DELAY.1),
        )?;
        let buzz = optional_bounded(config, "buzz_filter_cutoff_frequency", None, None)?
            .map(|value| {
                let above = 80_f64.min(max_filter_frequency - 1.0);
                if value <= above {
                    return Err(ConfigError::new(format!(
                    "Option 'buzz_filter_cutoff_frequency' in section '{}' must be above {above}",
                    config.identifier()
                )));
                }
                if value >= max_filter_frequency {
                    return Err(ConfigError::new(format!(
                    "Option 'buzz_filter_cutoff_frequency' in section '{}' must be below {max_filter_frequency}",
                    config.identifier()
                )));
                }
                Ok(value)
            })
            .transpose()?;
        let buzz_delay = config.get_int_bounded(
            "buzz_filter_delay",
            Some(2),
            Some(BUZZ_DELAY.0),
            Some(BUZZ_DELAY.1),
        )?;
        let notches = read_float_list(config, "notch_filter_frequencies", max_filter_frequency)?;
        let notch_quality = config.get_float_bounded(
            "notch_filter_quality",
            Some(2.0),
            Some(0.5),
            Some(6.0),
            None,
            None,
        )?;

        let options = Self {
            drift,
            drift_delay,
            buzz,
            buzz_delay,
            notches,
            notch_quality,
        };
        let design = options.design(samples_per_second)?;
        Ok((options, design))
    }

    /// Design the filter from the options (`ContinuousTareFilter.design_filter`
    /// + `ContinuousTareFilterHelper.__init__`'s `design_filter(config.error)`).
    ///
    /// # Errors
    /// The SciPy refusal when a butter design is not in
    /// [`GENERATED_SOS`](crate::core::klippy::extras::trigger_analog).
    pub fn design(&self, samples_per_second: f64) -> Result<DigitalFilter, ConfigError> {
        let mut design = DigitalFilter::new(samples_per_second);
        if let Some(drift) = self.drift {
            design.add_highpass(drift, u32::try_from(self.drift_delay).unwrap_or(1))?;
        }
        if let Some(buzz) = self.buzz {
            design.add_lowpass(buzz, u32::try_from(self.buzz_delay).unwrap_or(1))?;
        }
        for notch in &self.notches {
            design.add_notch(*notch, self.notch_quality);
        }
        Ok(design)
    }
}

/// `floatListParamHelper`'s option: a comma list of floats, bounded per element
/// and capped at [`MAX_NOTCHES`].
///
/// # Errors
/// Upstream's `unable to parse` wording for a non-number, and
/// `_validate_float_list`'s two checks (`above` 0, `below` the Nyquist floor,
/// maximum length).
fn read_float_list(
    config: &ConfigWrapper,
    option: &str,
    below: f64,
) -> Result<Vec<f64>, ConfigError> {
    let Some(items) = config.get_list(option, ',') else {
        return Ok(Vec::new());
    };
    let identifier = config.identifier();
    let values: Vec<f64> = items
        .iter()
        .map(|item| {
            item.trim().parse::<f64>().map_err(|_| {
                ConfigError::new(format!(
                    "Error on '{option}' in section '{identifier}': unable to parse {item}"
                ))
            })
        })
        .collect::<Result<_, _>>()?;
    if values.len() > MAX_NOTCHES {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{identifier}' has maximum length {MAX_NOTCHES}"
        )));
    }
    for value in &values {
        if *value <= 0. {
            return Err(ConfigError::new(format!(
                "Option '{option}' in section '{identifier}' must be above 0.0"
            )));
        }
        if *value >= below {
            return Err(ConfigError::new(format!(
                "Option '{option}' in section '{identifier}' must be below {below}"
            )));
        }
    }
    Ok(values)
}

/// An optional float with `minval`/`maxval`, as `floatParamHelper`'s
/// `_get_float` reads it when the option is present.
///
/// # Errors
/// The wrapper's own bound wording (`Option 'x' in section 'y' must have
/// minimum/maximum of …`).
fn optional_bounded(
    config: &ConfigWrapper,
    option: &str,
    minval: Option<f64>,
    maxval: Option<f64>,
) -> Result<Option<f64>, ConfigError> {
    let Some(value) = config.get_optional_float(option)? else {
        return Ok(None);
    };
    let identifier = config.identifier();
    if let Some(min) = minval {
        if value < min {
            return Err(ConfigError::new(format!(
                "Option '{option}' in section '{identifier}' must have minimum of {min}"
            )));
        }
    }
    if let Some(max) = maxval {
        if value > max {
            return Err(ConfigError::new(format!(
                "Option '{option}' in section '{identifier}' must have maximum of {max}"
            )));
        }
    }
    Ok(Some(value))
}

/// The fixed-point SOS design the trigger analog runs, from a
/// [`DigitalFilter`] (`MCU_SosFilter._convert_filter` / `_convert_state` at the
/// section's start value `0.` and scale `1.`).
///
/// # Errors
/// As [`DigitalFilter::to_fixed_sections`] / [`to_fixed_32`].
pub fn sos_design(design: &DigitalFilter) -> Result<SosFilterDesign, ConfigError> {
    let coeff_frac_bits = design.coeff_frac_bits();
    let sections = design.to_fixed_sections(coeff_frac_bits)?;
    let states = design.to_fixed_state(0.0)?;
    // The config-time scale is the pass-through default (`scale=1.`), whose
    // fractional bits are none.
    let scale_frac_bits = calc_frac_bits(&[1.0]);
    let scale =
        to_fixed_32(1.0, scale_frac_bits).map_err(|err| ConfigError::new(err.to_string()))?;
    Ok(SosFilterDesign {
        sections,
        offset: 0,
        scale,
        scale_frac_bits: u8::try_from(scale_frac_bits)
            .map_err(|_| ConfigError::new("scale_frac_bits does not fit a byte"))?,
        auto_offset: false,
        coeff_frac_bits: u8::try_from(coeff_frac_bits)
            .map_err(|_| ConfigError::new("coeff_frac_bits does not fit a byte"))?,
        states,
    })
}

// ===========================================================================
// The section
// ===========================================================================

/// One configured `[load_cell_probe]` (`load_cell_probe.LoadCellPrinterProbe`).
pub struct LoadCellProbe {
    /// The section's identifier, for logging.
    identifier: String,
    /// The cell the section also configures (`self._load_cell`).
    load_cell: Arc<LoadCell>,
    /// The tare/trigger/safety options.
    config_helper: LoadCellProbeConfigHelper,
    /// The probe's motion parameters.
    parameters: ProbeParameters,
    /// The probe's XYZ offsets.
    offsets: ProbeOffsets,
    /// The SOS filter the trigger analog runs.
    sos_filter: Arc<SosFilter>,
    /// The trigger the probing move would arm
    /// (`self._mcu_trigger_analog`).
    trigger_analog: Arc<McuTriggerAnalog>,
}

impl LoadCellProbe {
    /// Read the section and build the probe (`load_config` /
    /// `LoadCellPrinterProbe.__init__`).
    ///
    /// # Errors
    /// Any option, chip, or MCU complaint — including the SciPy refusal when
    /// the continuous-tare filter's butter design is not in the pre-generated
    /// table, and an unknown MCU for the trigger analog.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        // The cell (chips, calibration, `LOAD_CELL_*`, `dump_force`): upstream
        // hands `load_cell.LoadCell(config, sensor)` the sensor
        // `load_cell_probe.load_config` builds from this same section.
        let load_cell = Arc::new(LoadCell::new(config, printer)?);
        let sensor = load_cell.sensor();
        let samples_per_second = sensor.samples_per_second();

        let config_helper = LoadCellProbeConfigHelper::new(config)?;
        let parameters = ProbeParameters::new(config)?;
        let offsets = read_offsets(config)?;
        let (_options, design) = ContinuousTareOptions::read(config, samples_per_second)?;

        // The trigger analog builds on the sensor's MCU chip.
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let chip = pins
            .chip_as::<McuChip>(sensor.mcu_chip_name())
            .ok_or_else(|| {
                ConfigError::new(format!(
                    "Section '{identifier}': unknown MCU '{}'",
                    sensor.mcu_chip_name()
                ))
            })?;

        // `setup_sos_filter` then the design (`ContinuousTareFilterHelper`).
        let sos_filter = SosFilter::new(&chip, MAX_FILTER_SECTIONS)
            .map_err(|err| ConfigError::new(err.to_string()))?;
        sos_filter.set_filter_design(sos_design(&design)?);
        let trigger_analog = Arc::new(
            McuTriggerAnalog::new(
                (*chip).clone(),
                samples_per_second,
                Some(Arc::clone(&sos_filter)),
            )
            .map_err(|err| ConfigError::new(err.to_string()))?,
        );

        Ok(Self {
            identifier,
            load_cell,
            config_helper,
            parameters,
            offsets,
            sos_filter,
            trigger_analog,
        })
    }

    /// The section identifier.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }

    /// The load cell the section also configures (`self._load_cell`).
    pub fn load_cell(&self) -> &Arc<LoadCell> {
        &self.load_cell
    }

    /// The tare/trigger/safety options.
    pub fn config_helper(&self) -> &LoadCellProbeConfigHelper {
        &self.config_helper
    }

    /// The configured probe parameters (`get_probe_params` → the section's
    /// defaults).
    pub fn parameters(&self) -> &ProbeParameters {
        &self.parameters
    }

    /// The probe's offsets (`get_offsets`).
    pub fn offsets(&self) -> ProbeOffsets {
        self.offsets
    }

    /// The SOS filter the trigger analog runs (`get_sos_filter`).
    pub fn sos_filter(&self) -> &Arc<SosFilter> {
        &self.sos_filter
    }

    /// The trigger the probing move would arm (`self._mcu_trigger_analog`).
    pub fn trigger_analog(&self) -> &Arc<McuTriggerAnalog> {
        &self.trigger_analog
    }
}

impl PrinterObject for LoadCellProbe {
    /// The cell's status (`LoadCellPrinterProbe.get_status`'s first
    /// `status.update(self._load_cell.get_status(eventtime))`).
    ///
    /// The tap fields (`last_z_result` / `is_last_tap_valid`) and the command
    /// helper's `name` / `last_query` join it once the probe session lands (see
    /// the module gaps).
    fn get_status(&self, eventtime: f64) -> Value {
        self.load_cell.get_status(eventtime)
    }
}

impl std::fmt::Debug for LoadCellProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LoadCellProbe")
            .field("identifier", &self.identifier)
            .field("z_offset", &self.offsets.z)
            .field("sections", &self.sos_filter.max_sections())
            .finish()
    }
}

/// Upstream's `load_config` for `[load_cell_probe]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let probe = Arc::new(LoadCellProbe::new(config, printer)?);
    // `self._printer.add_object('probe', self)`.
    let object: Arc<dyn PrinterObject> = Arc::clone(&probe) as Arc<dyn PrinterObject>;
    let _ = &object;
    printer.add_object(PROBE_OBJECT, object)?;
    Ok(probe)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::pins::PrinterPins;
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("load_cell_probe", name);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap(name: Option<&str>, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
        ConfigWrapper::new(
            Box::leak(Box::new(section(name, options))),
            AccessTracking::shared(),
        )
    }

    /// A ready printer with `pins`, `gcode` and one registered `[mcu]`.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let mcu = McuObject::new(ConfigSection::new("mcu", None), &printer).unwrap();
        printer.add_object("mcu", Arc::new(mcu)).unwrap();
        printer
    }

    /// The corpus `[load_cell_probe]` section verbatim
    /// (`test/klippy/load_cell.cfg`).
    const CORPUS: [(&str, &str); 10] = [
        ("z_offset", "0"),
        ("sensor_type", "ads1220"),
        ("speed", "10.0"),
        ("cs_pin", "PJ2"),
        ("data_ready_pin", "PJ3"),
        ("counts_per_gram", "100"),
        ("reference_tare_counts", "1000"),
        ("drift_filter_cutoff_frequency", "0.8"),
        ("buzz_filter_cutoff_frequency", "100.0"),
        ("notch_filter_frequencies", "50, 60"),
    ];

    fn corpus_probe() -> (Arc<Printer>, Arc<dyn PrinterObject>) {
        let machine = printer();
        let probe = load_config(&wrap(None, &CORPUS), &machine).unwrap();
        (machine, probe)
    }

    #[test]
    fn the_corpus_section_loads_and_reads_every_option() {
        // The whole corpus file, so the option contract is the file's.
        let path = klipperx_test_support::klipper_dir().join("test/klippy/load_cell.cfg");
        let text = std::fs::read_to_string(&path).expect("load_cell.cfg is readable");
        let config = Config::from_text(&text).expect("load_cell.cfg parses").0;
        let machine = printer();
        let probe_section = config
            .sections()
            .into_iter()
            .find(|sect| sect.id == "load_cell_probe")
            .expect("the corpus carries [load_cell_probe]");

        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(probe_section, Arc::clone(&access));
        let probe = load_config(&wrapper, &machine).expect("the section loads");
        let _ = probe;
        for option in probe_section.parameters.keys() {
            assert!(
                access.contains(&probe_section.identifier(), option),
                "unread option '{option}' in '{}'",
                probe_section.identifier()
            );
        }
    }

    #[test]
    fn the_corpus_probe_reports_the_cells_status() {
        let (_machine, probe) = corpus_probe();
        let status = probe.get_status(0.);
        // The load cell's status comes through (calibrated + tared from the
        // section's own `counts_per_gram` / `reference_tare_counts`).
        assert_eq!(status["is_calibrated"], true);
        assert_eq!(status["reference_tare_counts"], 1000);
        assert_eq!(status["counts_per_gram"], 100.0);
    }

    #[test]
    fn the_continuous_tare_design_matches_the_corpus_options() {
        let machine = printer();
        let wrapper = wrap(None, &CORPUS);
        let (options, design) = ContinuousTareOptions::read(&wrapper, 660.0).unwrap();
        assert_eq!(options.drift, Some(0.8));
        assert_eq!(options.drift_delay, 2);
        assert_eq!(options.buzz, Some(100.0));
        assert_eq!(options.buzz_delay, 2);
        assert_eq!(options.notches, vec![50.0, 60.0]);
        assert_eq!(options.notch_quality, 2.0);
        // 1 highpass + 1 lowpass + 2 notches.
        assert_eq!(design.get_size(), 4);
        // The fixed-point conversion the trigger would be armed with.
        let sos = sos_design(&design).unwrap();
        assert_eq!(sos.sections.len(), 4);
        assert_eq!(sos.states.len(), 4);
        // The widest coefficient is the highpass's `b1` ≈ 1.989, so the words
        // carry 30 fractional bits.
        assert_eq!(sos.coeff_frac_bits, 30);
        assert_eq!(sos.scale, 1);
        assert_eq!(sos.scale_frac_bits, 0);
        assert!(!sos.auto_offset);
        // The cell and the trigger both exist on the machine.
        let probe = LoadCellProbe::new(&wrapper, &machine).unwrap();
        assert_eq!(probe.offsets().z, 0.0);
        assert_eq!(probe.parameters().params().probe_speed, 10.0);
        assert_eq!(probe.sos_filter().max_sections(), 4);
    }

    #[test]
    fn the_trigger_and_tare_options_carry_upstream_defaults() {
        let options = LoadCellProbeConfigHelper::new(&wrap(None, &CORPUS)).unwrap();
        assert_eq!(options.tare_time(), 4. / 60.);
        assert_eq!(options.trigger_force_grams(), 75.0);
        assert_eq!(options.safety_limit_grams(), 2000.0);
        // `max(2, ceil(tare_time * sps))`: four 60 Hz cycles at 660 SPS.
        assert_eq!(options.tare_samples(660.0), 44);
        // A rate so low the window rounds to nothing still takes two samples.
        assert_eq!(options.tare_samples(1.0), 2);
    }

    #[test]
    fn the_trigger_force_and_safety_limit_are_bounded_upstream_wording() {
        let mut options = CORPUS.to_vec();
        options.push(("trigger_force", "5"));
        let err = LoadCellProbeConfigHelper::new(&wrap(None, &options)).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'trigger_force' in section 'load_cell_probe' must have minimum of 10"
        );

        options.pop();
        options.push(("force_safety_limit", "50"));
        let err = LoadCellProbeConfigHelper::new(&wrap(None, &options)).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'force_safety_limit' in section 'load_cell_probe' must have minimum of 100"
        );
    }

    #[test]
    fn the_safety_range_is_the_calibrated_band_or_an_error() {
        let options = LoadCellProbeConfigHelper {
            tare_time: 4. / 60.,
            trigger_force: 75.,
            force_safety_limit: 2000.,
        };
        // 100 counts/gram around a 1000-count tare: 200000 counts either way,
        // inside the ads1220's 24-bit range.
        let (min, max) = options
            .safety_range(100.0, 1000, (-0x80_0000, 0x7F_FFFF))
            .unwrap();
        assert_eq!(min, 1000 - 200_000);
        assert_eq!(max, 1000 + 200_000);
        // A tiny sensor range leaves the band outside it.
        let err = options
            .safety_range(100.0, 1000, (-1000, 1000))
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Load cell force_safety_limit exceeds sensor range!"
        );
    }

    #[test]
    fn grams_per_count_inverts_the_scale_and_refuses_a_huge_one() {
        assert_eq!(grams_per_count(100.0).unwrap(), 0.01);
        let err = grams_per_count((1u64 << 29) as f64).unwrap_err();
        assert_eq!(
            err.to_string(),
            "counts_per_gram value is too large to filter"
        );
    }

    #[test]
    fn the_notch_frequency_list_is_bounded_element_by_element() {
        let mut options = CORPUS.to_vec();
        options.pop();
        options.push(("notch_filter_frequencies", "50, 60, 70"));
        let err = ContinuousTareOptions::read(&wrap(None, &options), 660.0).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'notch_filter_frequencies' in section 'load_cell_probe' has maximum length 2"
        );

        options.pop();
        options.push(("notch_filter_frequencies", "0"));
        let err = ContinuousTareOptions::read(&wrap(None, &options), 660.0).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'notch_filter_frequencies' in section 'load_cell_probe' must be above 0.0"
        );

        options.pop();
        options.push(("notch_filter_frequencies", "400"));
        let err = ContinuousTareOptions::read(&wrap(None, &options), 660.0).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'notch_filter_frequencies' in section 'load_cell_probe' must be below 330"
        );
    }

    #[test]
    fn the_buzz_cutoff_is_bounded_by_the_sensor_rate() {
        let mut options = CORPUS.to_vec();
        options.retain(|(key, _)| *key != "buzz_filter_cutoff_frequency");
        options.push(("buzz_filter_cutoff_frequency", "80"));
        let err = ContinuousTareOptions::read(&wrap(None, &options), 660.0).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'buzz_filter_cutoff_frequency' in section 'load_cell_probe' must be above 80"
        );

        options.pop();
        options.push(("buzz_filter_cutoff_frequency", "330"));
        let err = ContinuousTareOptions::read(&wrap(None, &options), 660.0).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'buzz_filter_cutoff_frequency' in section 'load_cell_probe' must be below 330"
        );
    }

    #[test]
    fn the_probe_registers_the_probe_object_and_the_load_cell_commands() {
        let (machine, _probe) = corpus_probe();
        assert!(
            machine.lookup_object("probe").is_some(),
            "the probe object is registered"
        );
        let gcode = machine
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        let help = gcode.command_help();
        // The cell's own commands come along with the shared section.
        assert_eq!(
            help.get("LOAD_CELL_TARE").map(String::as_str),
            Some("Set the Zero point of the load cell")
        );
    }
}
