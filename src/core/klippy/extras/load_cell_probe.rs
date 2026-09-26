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
//! # The probe run path
//!
//! The section registers the `probe` object under the name `bed_mesh` /
//! `PROBE` look it up by ([`probe::lookup_probe_session`](crate::core::klippy::extras::probe)),
//! and [`LoadCellProbe`] implements the `ProbeSession` surface: a tap is one
//! `LoadCellProbingMove.probing_move` (tare via the sample collector, then a
//! homing move on the trigger analog) followed by `TappingMove.run_tap`'s
//! ascent piecewise fit ([`LCBestFit`]). `PROBE` and `BED_MESH_CALIBRATE`
//! reach it, so the corpus case `load_cell.test`'s g-code runs through
//! (verified against the fake firmware).
//!
//! What upstream adds around that and is **not** wired yet:
//!
//! * the `load_cell_probe/dump_taps` mux endpoint (`tap` events are not
//!   broadcast).
//! * the `probe:z_virtual_endstop` chip (`probe.HomingViaProbeHelper`): a
//!   config that homes via the load cell has no virtual endstop pin.
//! * `ContinuousTareFilterHelper.update_from_command`: a `PROBE`/`G28` that
//!   overrides the drift/buzz/notch options at the command line is not read
//!   (the corpus passes none).
//! * `LoadCellProbingMove`'s `LookupZSteppers` and the sensors'
//!   `setup_trigger_analog` attach, so the trigger analog's trsync is not
//!   told which Z steppers to stop on a real MCU (the fake firmware fires on
//!   the move's first step regardless).
//! * `LOAD_CELL_TEST_TAP`.
//!
//! [`lookup_probe_session`]: crate::core::klippy::extras::probe

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::trigger_analog::TriggerAnalogType;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::load_cell::{LoadCell, LoadCellSampleCollector};
use crate::core::klippy::extras::probe::{
    calc_probe_z_average, ProbeOffsets, ProbeParams, ProbeSession,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::extras::trigger_analog::{calc_frac_bits, to_fixed_32, DigitalFilter};
use crate::core::klippy::gcode::{
    CommandError, CommandFuture, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{gaussian_solve, mat_mat_mul, mat_transp, Coord};
use crate::core::klippy::mcu::{McuChip, McuTriggerAnalog, SosFilter, SosFilterDesign};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("load_cell_probe", order = 40, load = load_config);

/// The object the probe is looked up under (upstream's
/// `printer.add_object('probe', self)`).
const PROBE_OBJECT: &str = "probe";

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The Z axis index in a [`Coord`].
const Z_AXIS: usize = 2;

/// `FRAC_GRAMS_CONV`: the MCU SOS filter's scale, in "fractional grams".
const FRAC_GRAMS_CONV: f64 = 32768.0;

/// The minimum ascent samples per side of the fit (`FIT_MIN_POINTS`).
const FIT_MIN_POINTS: usize = 3;

/// The ascent data window (`ASCENT_DATA_WINDOW_SECONDS`).
const ASCENT_DATA_WINDOW_SECONDS: f64 = 0.3;

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
    /// The machine, for the toolhead and the probe:tap event.
    printer: Weak<Printer>,
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
    /// The trigger the probing move arms (`self._mcu_trigger_analog`).
    trigger_analog: Arc<McuTriggerAnalog>,
    /// The base filter design, whose offset/scale the tare rewrites.
    base_design: Mutex<SosFilterDesign>,
    /// The Z the probing move descends to (`probe.lookup_minimum_z`).
    z_min_position: f64,
    /// Whether a probe session is open (`SampleAveragingHelper.hw_probe_session`).
    active: AtomicBool,
    /// The completed sample sets (`SampleAveragingHelper.results`).
    results: Mutex<Vec<Coord>>,
    /// The last tap's contact Z (`TappingMove.get_status`).
    last_result: Mutex<f64>,
    /// Whether the last tap was valid (`TappingMove.get_status`).
    is_last_tap_valid: AtomicBool,
    /// The live tare in counts (`LoadCellProbingMove.get_status`).
    tare_counts: Mutex<f64>,
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
        let base_design = sos_design(&design)?;
        sos_filter.set_filter_design(base_design.clone());
        let trigger_analog = Arc::new(
            McuTriggerAnalog::new(
                (*chip).clone(),
                samples_per_second,
                Some(Arc::clone(&sos_filter)),
            )
            .map_err(|err| ConfigError::new(err.to_string()))?,
        );

        // `probe.lookup_minimum_z`: `[stepper_z] position_min`, else
        // `[printer] minimum_z_position`, else 0.
        let z_min_position = match config.sibling("stepper_z") {
            Some(sibling) => sibling.get_float("position_min", Some(0.0))?,
            None => match config.sibling("printer") {
                Some(sibling) => sibling.get_float("minimum_z_position", Some(0.0))?,
                None => 0.0,
            },
        };

        Ok(Self {
            identifier,
            printer: Arc::downgrade(printer),
            load_cell,
            config_helper,
            parameters,
            offsets,
            sos_filter,
            trigger_analog,
            base_design: Mutex::new(base_design),
            z_min_position,
            active: AtomicBool::new(false),
            results: Mutex::new(Vec::new()),
            last_result: Mutex::new(0.0),
            is_last_tap_valid: AtomicBool::new(false),
            tare_counts: Mutex::new(0.0),
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

    /// The toolhead, or "Printer is not ready".
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// Upstream's `_probe_state_error`.
    fn state_error() -> CommandError {
        CommandError::new("Internal probe error - start/end probe session mismatch")
    }

    /// Whether this run writes its MCU output to a file (`MCU.is_fileoutput`).
    fn fileoutput(&self) -> bool {
        self.printer
            .upgrade()
            .is_some_and(|printer| printer.is_fileoutput())
    }

    /// `LoadCellParameterHelper.get_probe_params`: the command's parameters
    /// with the tap's lift folded into `load_cell_retract_dist` and the
    /// inter-sample retract zeroed.
    fn load_cell_params(&self, gcmd: &GcodeCommand) -> Result<(ProbeParams, f64), CommandError> {
        let params = self.parameters.params().from_command(gcmd)?;
        let load_cell_retract_dist = params.sample_retract_dist;
        Ok((
            ProbeParams {
                sample_retract_dist: 0.0,
                ..params
            },
            load_cell_retract_dist,
        ))
    }

    /// `_start_collector`: a fresh collector started at the last move time.
    fn start_collector(&self) -> LoadCellSampleCollector {
        let toolhead = self.toolhead().ok();
        let print_time = toolhead
            .as_ref()
            .map(|toolhead| toolhead.get_last_move_time());
        let collector = self.load_cell.get_collector();
        collector.start_collecting(print_time);
        collector
    }

    /// `ContinuousTareFilterHelper.update_from_command`.
    ///
    /// The command's filter overrides are read by `ContinuousTareOptions`
    /// today only from the config; a command that rewrites them is a known
    /// gap (the corpus passes none).
    fn update_filter_from_command(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        Ok(())
    }

    /// `_pause_and_tare`: collect the tare window, re-tare, and arm the
    /// trigger analog for the coming move.
    async fn pause_and_tare(
        &self,
        gcmd: &GcodeCommand,
        toolhead: &ToolHeadObject,
    ) -> Result<(), CommandError> {
        let collector = self.start_collector();
        let num_samples = self
            .config_helper
            .tare_samples(self.load_cell.sensor().samples_per_second());
        let (samples, errors) = collector.collect_min(num_samples.max(1) as usize).await?;
        if let Some((errs, overflows)) = errors {
            return Err(CommandError::new(format!(
                "Load cell sensor reported errors while probing: {errs} errors, {overflows} overflows"
            )));
        }
        let tare_counts = if samples.is_empty() {
            0.0
        } else {
            samples.iter().map(|sample| sample[2]).sum::<f64>() / samples.len() as f64
        };
        self.update_filter_from_command(gcmd)?;
        self.load_cell.tare(tare_counts as i64);
        *self.tare_counts.lock().unwrap_or_else(|p| p.into_inner()) = tare_counts;

        let counts_per_gram = self.load_cell.counts_per_gram().unwrap_or(1.0);
        let reference = self.load_cell.reference_tare_counts().unwrap_or(0);
        let (safety_min, safety_max) = self
            .config_helper
            .safety_range(
                counts_per_gram,
                reference,
                self.load_cell.saturation_range(),
            )
            .map_err(|err| CommandError::new(err.to_string()))?;
        self.trigger_analog.set_raw_range(safety_min, safety_max);

        // `gpc = get_grams_per_count() * FRAC_GRAMS_CONV` folded into the
        // filter's offset/scale (`sos_filter.set_offset_scale`).
        let gpc = grams_per_count(counts_per_gram)
            .map_err(|err| CommandError::new(err.to_string()))?
            * FRAC_GRAMS_CONV;
        let scale_frac_bits = calc_frac_bits(&[gpc]);
        let mut design = self
            .base_design
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        design.offset = -(tare_counts as i32);
        design.scale =
            to_fixed_32(gpc, scale_frac_bits).map_err(|err| CommandError::new(err.to_string()))?;
        design.scale_frac_bits = u8::try_from(scale_frac_bits).unwrap_or(0);
        self.sos_filter.set_filter_design(design);

        let trigger_val = self.config_helper.trigger_force_grams();
        let trigger_frac_grams = (trigger_val * FRAC_GRAMS_CONV) as i32;
        self.trigger_analog
            .set_trigger(TriggerAnalogType::AbsGe, trigger_frac_grams);
        let _ = toolhead;
        Ok(())
    }

    /// `LoadCellProbingMove.probing_move`: tare, then a probing move to
    /// `z_min_position` that stops on the trigger analog.
    async fn probing_move(
        &self,
        gcmd: &GcodeCommand,
    ) -> Result<(Coord, LoadCellSampleCollector), CommandError> {
        if !self.load_cell.is_calibrated() {
            return Err(CommandError::new("Load Cell not calibrated"));
        }
        let toolhead = self.toolhead()?;
        self.pause_and_tare(gcmd, &toolhead).await?;
        let params = self.parameters.params().from_command(gcmd)?;
        let mut pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        pos.set_axis(Z_AXIS, self.z_min_position);
        let epos = toolhead
            .probing_move(&*self.trigger_analog, pos, params.probe_speed)
            .await?;
        let collector = self.start_collector();
        Ok((epos, collector))
    }

    /// `TappingMove.run_tap`: one descend, the ascent samples, and the
    /// piecewise ascent fit that replaces the raw trigger Z.
    async fn run_tap(&self, gcmd: &GcodeCommand) -> Result<Coord, CommandError> {
        let toolhead = self.toolhead()?;
        let (mut epos, collector) = self.probing_move(gcmd).await?;
        let ascent_start_time = toolhead.get_last_move_time();
        let params = self.parameters.params().from_command(gcmd)?;
        let mut lift_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        lift_pos.set_axis(Z_AXIS, lift_pos.z() + params.sample_retract_dist);
        toolhead.move_to(lift_pos, params.lift_speed)?;
        let move_end = toolhead.get_last_move_time();
        let (samples, errors) = collector.collect_until(move_end).await?;
        if let Some((errs, overflows)) = errors {
            return Err(CommandError::new(format!(
                "Load cell sensor reported errors while probing: {errs} errors, {overflows} overflows"
            )));
        }
        let corrected_z =
            self.analyze_ascent(gcmd, &samples, ascent_start_time, &toolhead, epos.z())?;
        epos.set_axis(Z_AXIS, corrected_z);
        *self.last_result.lock().unwrap_or_else(|p| p.into_inner()) = corrected_z;
        self.is_last_tap_valid.store(true, Ordering::SeqCst);
        Ok(epos)
    }

    /// `TappingMove._analyze_ascent`: the piecewise fit over the ascent
    /// window, with upstream's file-output dummy data and self-check.
    fn analyze_ascent(
        &self,
        gcmd: &GcodeCommand,
        all_samples: &[[f64; 3]],
        ascent_start_time: f64,
        toolhead: &ToolHeadObject,
        raw_z: f64,
    ) -> Result<f64, CommandError> {
        let mut data: Vec<(f64, f64)> = Vec::new();
        for sample in all_samples {
            if sample[0] >= ascent_start_time
                && sample[0] <= ascent_start_time + ASCENT_DATA_WINDOW_SECONDS
            {
                let z = toolhead.kinematic_z_at(sample[0]).unwrap_or(0.0);
                data.push((sample[1], z));
            }
        }
        let fileoutput = self.fileoutput();
        if fileoutput {
            data = vec![
                (0.0, 0.0),
                (10.0, 0.1),
                (20.0, 0.2),
                (25.0, 0.3),
                (25.0, 0.4),
                (25.0, 0.5),
            ];
        }
        if data.len() < 2 * FIT_MIN_POINTS {
            return Err(CommandError::new(format!(
                "Insufficient ascent samples ({} total, need >= {} each) for piecewise fit",
                data.len(),
                2 * FIT_MIN_POINTS
            )));
        }
        let (z_contact, below_count, above_count, depress_slope) = LCBestFit::find_best_fit(&data)?;
        if below_count < FIT_MIN_POINTS || above_count < FIT_MIN_POINTS {
            return Err(CommandError::new(format!(
                "Insufficient ascent samples ({below_count} below, {above_count} above, need >= {FIT_MIN_POINTS} each) for piecewise fit"
            )));
        }
        gcmd.respond_info(&format!(
            "Load cell probe fit: n_below={below_count} n_above={above_count} z_contact={z_contact:.4} raw={raw_z:.4} delta={:.4} depress_slope={depress_slope:.4}",
            raw_z - z_contact
        ));
        if fileoutput && (z_contact - 0.25).abs() > 0.01 {
            return Err(CommandError::new("Load cell probe fit result incorrect"));
        }
        Ok(z_contact)
    }

    /// Register `PROBE` / `QUERY_PROBE` (`probe.ProbeCommandHelper`).
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        // QUERY_PROBE: an analog trigger has no pin to sample
        // (`ProbeCommandHelper(query_endstop=None)`).
        gcode
            .register_command(
                "QUERY_PROBE",
                Arc::new(|_gcmd| {
                    Box::pin(async { Err(CommandError::new("Probe does not support QUERY_PROBE")) })
                }),
                Some("Return the status of the z-probe"),
                false,
            )
            .map_err(ConfigError::new)?;
        let probe = Arc::clone(self);
        gcode
            .register_command(
                "PROBE",
                Arc::new(move |gcmd| {
                    let probe = Arc::clone(&probe);
                    Box::pin(async move {
                        probe.start_probe_session(gcmd)?;
                        probe.run_probe(gcmd).await?;
                        let pos = probe
                            .pull_probed_results()
                            .into_iter()
                            .next()
                            .ok_or_else(LoadCellProbe::state_error)?;
                        probe.end_probe_session()?;
                        gcmd.respond_info(&format!(
                            "Result: at {:.3},{:.3} estimate contact at z={:.6}",
                            pos.x(),
                            pos.y(),
                            pos.z()
                        ));
                        Ok(())
                    })
                }),
                Some("Probe Z-height at current XY position"),
                false,
            )
            .map_err(ConfigError::new)?;
        Ok(())
    }
}

/// `LCBestFit`: the piecewise least-squares fit of the ascent data, using
/// SciPy's `mathutil` helpers as upstream's `LCBestFit` does.
struct LCBestFit;

impl LCBestFit {
    /// `_calc_least_squares`: the relative error and coefficients for one
    /// candidate `est_z_contact`.
    fn calc_least_squares(samples: &[(f64, f64)], est_z_contact: f64) -> (f64, [[f64; 1]; 2]) {
        let mut eqs: Vec<Vec<f64>> = Vec::with_capacity(samples.len());
        let mut ans: Vec<Vec<f64>> = Vec::with_capacity(samples.len());
        for (step_z, sensor_grams) in samples {
            if *step_z <= est_z_contact {
                eqs.push(vec![1.0, step_z - est_z_contact]);
            } else {
                eqs.push(vec![1.0, 0.0]);
            }
            ans.push(vec![*sensor_grams]);
        }
        let eqst = mat_transp(&eqs);
        let eqst_eqs = mat_mat_mul(&eqst, &eqs).expect("least-squares shapes line up");
        let eqst_ans = mat_mat_mul(&eqst, &ans).expect("least-squares shapes line up");
        match gaussian_solve(&eqst_eqs, &eqst_ans, false) {
            None => (f64::MAX, [[0.0], [0.0]]),
            Some(coeffs) => {
                let rel_err = -coeffs
                    .iter()
                    .zip(eqst_ans.iter())
                    .map(|(c, a)| c[0] * a[0])
                    .sum::<f64>();
                (rel_err, [[coeffs[0][0]], [coeffs[1][0]]])
            }
        }
    }

    /// `_run_fit`: the binary search over the split point.
    fn run_fit(samples: &[(f64, f64)]) -> (f64, [[f64; 1]; 2]) {
        let mut min_z = samples[0].0;
        let mut best_z = samples[0].0;
        let mut max_z = samples[samples.len() - 1].0;
        let mut best_err = f64::MAX;
        let mut best_coeffs = [[0.0], [0.0]];
        while max_z - min_z > 0.000050 {
            let mid_z = (min_z + max_z) * 0.5;
            let guess_z = if best_z < mid_z {
                (best_z + max_z) * 0.5
            } else {
                (min_z + best_z) * 0.5
            };
            let (guess_err, guess_coeffs) = Self::calc_least_squares(samples, guess_z);
            if guess_err < best_err {
                if guess_z > best_z {
                    min_z = best_z;
                } else {
                    max_z = best_z;
                }
                best_z = guess_z;
                best_err = guess_err;
                best_coeffs = guess_coeffs;
            } else if guess_z > best_z {
                max_z = guess_z;
            } else {
                min_z = guess_z;
            }
        }
        (best_z, best_coeffs)
    }

    /// `find_best_fit`: returns `(z_contact, below, above, depress_slope)`.
    fn find_best_fit(data: &[(f64, f64)]) -> Result<(f64, usize, usize, f64), CommandError> {
        if data.is_empty() {
            return Err(CommandError::new("no ascent data"));
        }
        let base_z = 0.5 * (data[0].1 + data[data.len() - 1].1);
        let base_grams = 0.5 * (data[0].0 + data[data.len() - 1].0);
        let samples: Vec<(f64, f64)> = data
            .iter()
            .map(|(grams, z)| (z - base_z, grams - base_grams))
            .collect();
        let (est_z, coeffs) = Self::run_fit(&samples);
        let n_below = samples.iter().filter(|sample| sample.0 <= est_z).count();
        let depress_slope = coeffs[1][0];
        Ok((
            base_z + est_z,
            n_below,
            samples.len() - n_below,
            depress_slope,
        ))
    }
}

impl ProbeSession for LoadCellProbe {
    fn start_probe_session(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        if self.active.swap(true, Ordering::SeqCst) {
            return Err(Self::state_error());
        }
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
        Ok(())
    }

    fn run_probe<'a>(&'a self, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
        Box::pin(async move {
            if !self.active.load(Ordering::SeqCst) {
                return Err(Self::state_error());
            }
            let toolhead = self.toolhead()?;
            let homed = toolhead.get_status(0.0)["homed_axes"]
                .as_str()
                .unwrap_or("")
                .to_string();
            if !homed.contains('z') {
                return Err(CommandError::new("Must home before probe"));
            }
            let (params, _load_cell_retract) = self.load_cell_params(gcmd)?;
            let mut retries = 0i64;
            let mut positions: Vec<Coord> = Vec::new();
            while (positions.len() as i64) < params.samples {
                let epos = self.run_tap(gcmd).await?;
                positions.push(epos);
                let spread = {
                    let max = positions
                        .iter()
                        .map(Coord::z)
                        .fold(f64::NEG_INFINITY, f64::max);
                    let min = positions.iter().map(Coord::z).fold(f64::INFINITY, f64::min);
                    max - min
                };
                if spread > params.samples_tolerance {
                    if retries >= params.samples_tolerance_retries {
                        return Err(CommandError::new("Probe samples exceed samples_tolerance"));
                    }
                    gcmd.respond_info("Probe samples exceed tolerance. Retrying...");
                    retries += 1;
                    positions.clear();
                }
                if (positions.len() as i64) < params.samples {
                    let mut lift = toolhead
                        .position()
                        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                    let z = positions.last().map(Coord::z).unwrap_or(0.0);
                    lift.set_axis(Z_AXIS, z + params.sample_retract_dist);
                    toolhead.move_to(lift, params.lift_speed)?;
                }
            }
            let epos = calc_probe_z_average(&positions, &params.samples_result);
            self.results
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(epos);
            Ok(())
        })
    }

    fn probe_params(&self, gcmd: &GcodeCommand) -> Result<ProbeParams, CommandError> {
        Ok(self.load_cell_params(gcmd)?.0)
    }

    fn pull_probed_results(&self) -> Vec<Coord> {
        std::mem::take(&mut *self.results.lock().unwrap_or_else(|p| p.into_inner()))
    }

    fn end_probe_session(&self) -> Result<(), CommandError> {
        if !self.active.swap(false, Ordering::SeqCst) {
            return Err(Self::state_error());
        }
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
        Ok(())
    }

    fn offsets(&self) -> ProbeOffsets {
        self.offsets
    }
}

impl PrinterObject for LoadCellProbe {
    /// The cell's status plus the tap fields
    /// (`LoadCellPrinterProbe.get_status`).
    fn get_status(&self, eventtime: f64) -> Value {
        let mut status = self.load_cell.get_status(eventtime);
        if let Some(object) = status.as_object_mut() {
            object.insert(
                "last_z_result".to_string(),
                json!(*self.last_result.lock().unwrap_or_else(|p| p.into_inner())),
            );
            object.insert(
                "is_last_tap_valid".to_string(),
                json!(self.is_last_tap_valid.load(Ordering::SeqCst)),
            );
        }
        status
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
    // `probe.ProbeCommandHelper(config, self)`.
    probe.register_commands(printer)?;
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
        // The probe commands the load cell probe registers.
        assert_eq!(
            help.get("PROBE").map(String::as_str),
            Some("Probe Z-height at current XY position")
        );
        assert_eq!(
            help.get("QUERY_PROBE").map(String::as_str),
            Some("Return the status of the z-probe")
        );
    }

    /// The wiring the corpus needs: `[load_cell_probe]` registers the `probe`
    /// object under the name `probe.rs`'s points round looks it up by, so
    /// `bed_mesh` / `PROBE` reach it (`probe.py`'s
    /// `add_object('probe', self)`).
    #[test]
    fn the_registered_probe_object_answers_the_probe_lookup() {
        let (machine, _probe) = corpus_probe();
        let session = crate::core::klippy::extras::probe::lookup_probe_session(&machine);
        assert!(
            session.is_some(),
            "the load cell probe answers the `probe` lookup"
        );
    }

    /// `LCBestFit.find_best_fit` on the file-output dummy ascent data returns
    /// the contact Z upstream self-checks for (`z_contact` ≈ 0.25).
    #[test]
    fn the_piecewise_ascent_fit_finds_the_dummy_contact_z() {
        let data = [
            (0.0, 0.0),
            (10.0, 0.1),
            (20.0, 0.2),
            (25.0, 0.3),
            (25.0, 0.4),
            (25.0, 0.5),
        ];
        let (z_contact, below, above, _slope) = LCBestFit::find_best_fit(&data).unwrap();
        assert!((z_contact - 0.25).abs() <= 0.01, "z_contact={z_contact}");
        assert!(below >= FIT_MIN_POINTS, "below={below}");
        assert!(above >= FIT_MIN_POINTS, "above={above}");
    }
}
