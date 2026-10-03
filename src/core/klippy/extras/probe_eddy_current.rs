//! `probe_eddy_current` — the eddy-current Z probe (upstream's
//! `klippy/extras/probe_eddy_current.py`).
//!
//! Upstream's object layout, one row per piece this port keeps:
//!
//! | here | upstream |
//! |---|---|
//! | [`EddyCalibration`] | `EddyCalibration` (the `calibrate = z:freq,…` table, `freq_to_height` / `height_to_freq`) |
//! | [`DriftCompensation`] and [`EddyCalibration::register_drift_compensation`] | the `drift_comp` chain: upstream's `DummyDriftCompensation` default, the duck-typed helper, and the registration `temperature_probe` drives (`:18-54`, `:1073-1074`) |
//! | [`EddyGatherSamples`] | `EddyGatherSamples` (samples in a time window → one probe result) |
//! | [`PrinterEddyProbe`] | `PrinterEddyProbe` — registered as **`probe`**, dispatching `METHOD` over descend / tap / scan sessions |
//! | [`EddyTapCalibration`] | `EddyTapCalibration` (`PROBE_EDDY_CURRENT_TAP_CALIBRATE`) |
//! | [`EddyCalibrationTool`] | `EddyCalibrationTool` (`PROBE_EDDY_CURRENT_CALIBRATE` + `Z_OFFSET_APPLY_PROBE`) |
//! | [`EddyProbeChip`] | `HomingViaProbeHelper` — what makes `probe:z_virtual_endstop` resolve, with `descend_z` as the rail's `position_endstop` |
//!
//! The probe registers the `probe` object itself (the bltouch precedent: the
//! factory calls `printer.add_object("probe", …)` while the loader files the
//! section under its own name), so `PROBE` / `QUERY_PROBE` / `PROBE_ACCURACY`
//! bind to this object and `probe::lookup_probe_session` answers for the
//! points rounds (`bed_mesh`, `ProbePointsHelper`).
//!
//! # Not here (M5d residuals, reported to main)
//!
//! * The `tap` analysis (`TapBestFit`, `_analyze_pullback`) and scan's
//!   per-sample toolhead position lookup need
//!   `mcu_to_commanded_position`-style time-position conversion, which has not
//!   been ported. Under file-output mode (which every upstream corpus case
//!   runs in) upstream skips that analysis too — a dummy result replaces it —
//!   so the corpus path is byte-identical; on real hardware `PROBE METHOD=tap`
//!   stops with an explicit error instead of inventing numbers.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::Value;

use crate::core::klippy::cmd::TriggerAnalogType;
use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::printer_bus::ProbeResultsHandle;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_move::{GCodeMove, GCODE_MOVE_OBJECT};
use crate::core::klippy::extras::ldc1612::{self, Calibration, Ldc1612};
use crate::core::klippy::extras::manual_probe::{
    FinalizeCallback, ManualProbe, MANUAL_PROBE_OBJECT,
};
use crate::core::klippy::extras::probe::{
    calc_probe_z_average, check_virtual_endstop, command_status, lookup_probe_session,
    ProbeCommandState, ProbeOffsets, ProbeParams, ProbeSession, SampleDelivery,
};
use crate::core::klippy::extras::toolhead::{HomingEndstop, ToolHeadObject};
use crate::core::klippy::extras::trigger_analog::{calc_frac_bits, to_fixed_32, DigitalFilter};
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{solve_linear_equations, Coord, Z_AXIS};
use crate::core::klippy::mcu::{McuChip, McuTriggerAnalog, SosFilter, SosFilterDesign};
use crate::core::klippy::pins::{
    DigitalOut, PinChip, PinError, PinParams, PrinterPins, PINS_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!(
    "probe_eddy_current",
    order = 30,
    prefix = load_config_prefix
);

/// Upstream's `load_config_prefix(config)`: build the probe, wire its
/// commands, and register it under `probe` (what `PROBE` / the points rounds
/// resolve); the loader registers the returned object under the section name
/// itself (`klippy.py:load_object` stores the same object both ways).
///
/// # Errors
/// A malformed option, an unknown MCU, or a `probe` name already taken.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let probe = Arc::new(PrinterEddyProbe::new(config, printer)?);
    probe.register_commands()?;
    EddyTapCalibration::register(printer, Arc::clone(&probe.calibration))?;
    EddyCalibrationTool::register(
        printer,
        probe.identifier().to_string(),
        Arc::clone(&probe.calibration),
    )?;
    printer.add_object(PROBE_OBJECT, Arc::clone(&probe) as Arc<dyn PrinterObject>)?;
    Ok(probe)
}

/// The object consumers find this probe under (`probe.py`'s
/// `printer.add_object('probe', self)`).
const PROBE_OBJECT: &str = "probe";

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// Samples per second the ldc1612 produces (`LDC1612.data_rate`).
const SPS: f64 = ldc1612::DATA_RATE as f64;

/// Outside the calibrated range, a frequency maps to this sentinel height
/// (`OUT_OF_RANGE`).
const OUT_OF_RANGE: f64 = 99.9;

/// The raw sample range the firmware may accept while homing
/// (`MAX_VALID_RAW_VALUE`).
const MAX_VALID_RAW_VALUE: i32 = 0x03ff_ffff;

/// Upstream's `FRAC_HZ`: the tap filter scales millihertz.
const FRAC_HZ: f64 = 1_000.0;

/// Upstream's `_probe_state_error` / `SampleAveragingHelper._probe_state_error`.
fn state_error() -> CommandError {
    CommandError::new("Internal probe error - start/end probe session mismatch")
}

/// Count of values `<= needle` in an ascending slice — Python's
/// `bisect.bisect` (right) on the same slice.
fn bisect_right(ascending: &[f64], needle: f64) -> usize {
    ascending.iter().filter(|&&v| v <= needle).count()
}

// ===========================================================================
// Calibration
// ===========================================================================

/// Upstream's duck-typed `drift_comp` (`probe_eddy_current.py:18-54`): what
/// [`EddyCalibration`] asks of the temperature drift helper that
/// `temperature_probe` builds (`EddyDriftCompensation`,
/// `temperature_probe.py:479+`).
///
/// Before anything registers one, upstream serves these calls from
/// `DummyDriftCompensation` (`probe_eddy_current.py:19-28`): pass the
/// frequency through, report `0.` temperature. Here that default is the
/// `None` around the registered `Arc` — every forwarding method falls back to
/// exactly what the dummy does.
pub trait DriftCompensation: Send + Sync {
    /// The temperature the corrections are read at (`get_temperature`).
    fn get_temperature(&self) -> f64;

    /// Move a frequency measured at `origin_temp` (or now, when it is `None`)
    /// toward the calibration temperature (`adjust_freq(freq, temp=None)`).
    fn adjust_freq(&self, freq: f64, origin_temp: Option<f64>) -> f64;

    /// Move a frequency that belongs to the calibration temperature out to
    /// `dest_temp` (or now, when it is `None`)
    /// (`unadjust_freq(freq, temp=None)`).
    fn unadjust_freq(&self, freq: f64, dest_temp: Option<f64>) -> f64;
}

/// The frequency→height table (`probe_eddy_current.EddyCalibration`), storage
/// for the `calibrate = z:freq,z:freq,…` option and the two conversions every
/// session runs samples through.
pub struct EddyCalibration {
    /// Calibration frequencies, ascending (the pairs sorted by frequency).
    cal_freqs: Vec<f64>,
    /// The height belonging to [`Self::cal_freqs`] at the same index
    /// (descending: frequency falls as the probe rises).
    cal_zpos: Vec<f64>,
    /// The registered drift helper (`self.drift_comp`); `None` is upstream's
    /// `DummyDriftCompensation` standing in until `temperature_probe`
    /// registers the real one.
    drift_comp: Mutex<Option<Arc<dyn DriftCompensation>>>,
}

impl EddyCalibration {
    /// Read `calibrate` (`EddyCalibration.__init__` + `_load_calibration`).
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let mut calibration = Self {
            cal_freqs: Vec::new(),
            cal_zpos: Vec::new(),
            drift_comp: Mutex::new(None),
        };
        let Some(raw) = config.get_str("calibrate") else {
            return Ok(calibration);
        };
        let mut pairs = Vec::new();
        for entry in raw.split(',') {
            let entry = entry.trim();
            if entry.is_empty() {
                continue;
            }
            let (z, freq) = entry.split_once(':').ok_or_else(|| {
                ConfigError::new(format!(
                    "Option 'calibrate' in section '{}' must be z:freq pairs",
                    config.identifier()
                ))
            })?;
            let z: f64 = z.trim().parse().map_err(|err| {
                ConfigError::new(format!("Option 'calibrate': bad z '{z}': {err}"))
            })?;
            let freq: f64 = freq.trim().parse().map_err(|err| {
                ConfigError::new(format!("Option 'calibrate': bad frequency '{freq}': {err}"))
            })?;
            pairs.push((freq, z));
        }
        // Sorted by frequency, as upstream's `sorted([(c[1], c[0]) …])`.
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (freq, z) in pairs {
            calibration.cal_freqs.push(freq);
            calibration.cal_zpos.push(z);
        }
        Ok(calibration)
    }

    /// The same table built directly, for tests.
    #[cfg(test)]
    pub fn from_pairs(pairs: &[(f64, f64)]) -> Self {
        let mut pairs: Vec<(f64, f64)> = pairs.to_vec();
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        Self {
            cal_freqs: pairs.iter().map(|(f, _)| *f).collect(),
            cal_zpos: pairs.iter().map(|(_, z)| *z).collect(),
            drift_comp: Mutex::new(None),
        }
    }

    /// Hand the table the temperature drift helper `temperature_probe` built
    /// (`register_drift_compensation`, `probe_eddy_current.py:53-54`). Any
    /// previously registered helper is replaced, as upstream's assignment is.
    pub fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensation>) {
        *self.drift_comp.lock().unwrap_or_else(|p| p.into_inner()) = Some(comp);
    }

    /// The registered helper (`self.drift_comp`), `None` while the dummy
    /// stands in.
    fn drift_comp(&self) -> Option<Arc<dyn DriftCompensation>> {
        self.drift_comp
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// The helper's temperature (`drift_comp.get_temperature()`), `0.` with
    /// none — the dummy's answer (`probe_eddy_current.py:20-21`).
    fn drift_temperature(&self) -> f64 {
        self.drift_comp()
            .map(|comp| comp.get_temperature())
            .unwrap_or(0.)
    }

    /// `drift_comp.adjust_freq(freq, temp)`, or `freq` with no helper.
    fn adjust(&self, freq: f64, origin_temp: Option<f64>) -> f64 {
        self.drift_comp()
            .map(|comp| comp.adjust_freq(freq, origin_temp))
            .unwrap_or(freq)
    }

    /// `drift_comp.unadjust_freq(freq, temp)`, or `freq` with no helper.
    fn unadjust(&self, freq: f64, dest_temp: Option<f64>) -> f64 {
        self.drift_comp()
            .map(|comp| comp.unadjust_freq(freq, dest_temp))
            .unwrap_or(freq)
    }

    /// More than two points, or a refusal (`verify_calibrated`).
    pub fn verify_calibrated(&self) -> Result<(), CommandError> {
        if self.cal_freqs.len() <= 2 {
            return Err(CommandError::new("Must calibrate probe_eddy_current first"));
        }
        Ok(())
    }

    /// Signal the drift helper that a Z calibration is starting
    /// (`note_z_calibration_start`, `probe_eddy_current.py:49-50`). A no-op
    /// while the `DummyDriftCompensation` stands in (the `None` here); the
    /// real `EddyDriftCompensation` from `temperature_probe` would forward.
    pub fn note_z_calibration_start(&self) {
        if let Some(comp) = self.drift_comp() {
            // The real helper's hook is not ported yet; when `temperature_probe`
            // lands its `EddyDriftCompensation`, this is where the call goes.
            let _ = comp;
        }
    }

    /// Signal the drift helper that a Z calibration has finished
    /// (`note_z_calibration_finish`, `probe_eddy_current.py:51-52`). Same
    /// no-op semantics as [`Self::note_z_calibration_start`].
    pub fn note_z_calibration_finish(&self) {
        if let Some(comp) = self.drift_comp() {
            let _ = comp;
        }
    }

    /// The `(frequencies, heights)` pair (`get_calibration`).
    pub fn get_calibration(&self) -> (Vec<f64>, Vec<f64>) {
        (self.cal_freqs.clone(), self.cal_zpos.clone())
    }

    /// Fill each row's `z` from its `frequency` (`apply_calibration`,
    /// `probe_eddy_current.py:61-79`): the frequency is adjusted toward the
    /// current temperature **once** — `cur_temp =
    /// drift_comp.get_temperature()` — before the table is read, and the row
    /// keeps its original frequency.
    pub fn apply_calibration(&self, data: &mut [[f64; 3]]) {
        let cur_temp = self.drift_temperature();
        for row in data.iter_mut() {
            let adj_freq = self.adjust(row[1], Some(cur_temp));
            row[2] = round6(self.table_height(adj_freq));
        }
    }

    /// Frequency → height (`freq_to_height`): the adjusted frequency walked
    /// through the table — upstream sends a one-row sample through
    /// `apply_calibration` (`probe_eddy_current.py:80-83`), so the helper's
    /// temperature is read once for the one row.
    pub fn freq_to_height(&self, freq: f64) -> f64 {
        let cur_temp = self.drift_temperature();
        let adj_freq = self.adjust(freq, Some(cur_temp));
        round6(self.table_height(adj_freq))
    }

    /// The table walk itself (`freq_to_height`'s interpolation):
    /// piecewise-linear between the bracketing calibration points,
    /// `[±OUT_OF_RANGE]` outside them.
    fn table_height(&self, freq: f64) -> f64 {
        let pos = bisect_right(&self.cal_freqs, freq);
        let zpos = if pos >= self.cal_zpos.len() {
            -OUT_OF_RANGE
        } else if pos == 0 {
            OUT_OF_RANGE
        } else {
            let this_freq = self.cal_freqs[pos];
            let prev_freq = self.cal_freqs[pos - 1];
            let this_zpos = self.cal_zpos[pos];
            let prev_zpos = self.cal_zpos[pos - 1];
            let gain = (this_zpos - prev_zpos) / (this_freq - prev_freq);
            let offset = prev_zpos - prev_freq * gain;
            freq * gain + offset
        };
        zpos
    }

    /// Height → frequency (`height_to_freq`, `probe_eddy_current.py:84-99`):
    /// the same interpolation walked the other way, then handed to
    /// `drift_comp.unadjust_freq` with no destination — the *current*
    /// temperature, as upstream passes nothing (`:99`); an uncalibrated
    /// height is an error ("Invalid probe_eddy_current height").
    pub fn height_to_freq(&self, height: f64) -> Result<f64, CommandError> {
        // The table is ascending in frequency = descending in height, so the
        // reversed views are ascending in height.
        let rev_zpos: Vec<f64> = self.cal_zpos.iter().rev().copied().collect();
        let rev_freqs: Vec<f64> = self.cal_freqs.iter().rev().copied().collect();
        let pos = bisect_right(&rev_zpos, height);
        if pos == 0 || pos >= rev_zpos.len() {
            return Err(CommandError::new("Invalid probe_eddy_current height"));
        }
        let this_freq = rev_freqs[pos];
        let prev_freq = rev_freqs[pos - 1];
        let this_zpos = rev_zpos[pos];
        let prev_zpos = rev_zpos[pos - 1];
        let gain = (this_freq - prev_freq) / (this_zpos - prev_zpos);
        let offset = prev_freq - prev_zpos * gain;
        Ok(self.unadjust(height * gain + offset, None))
    }
}

impl Calibration for EddyCalibration {
    fn get_calibration(&self) -> (Vec<f64>, Vec<f64>) {
        EddyCalibration::get_calibration(self)
    }

    fn apply_calibration(&self, data: &mut [[f64; 3]]) {
        EddyCalibration::apply_calibration(self, data);
    }
}

/// Python's `round(value, 6)` (upstream rounds every converted z).
fn round6(value: f64) -> f64 {
    (value * 1_000_000.0).round() / 1_000_000.0
}

// ===========================================================================
// Section options
// ===========================================================================

/// The `[probe_eddy_current]` options this port reads, split so every option
/// in the section is accounted for (`check_unused`) and the defaults follow
/// upstream.
struct EddyOptions {
    /// The Z the descend probe triggers at (`descend_z`, above 0).
    descend_z: f64,
    /// Probe-to-nozzle offsets (`EddyProbeOffsets`; the z axis stays 0).
    offsets: ProbeOffsets,
    /// The `probe.ProbeParameterHelper` defaults.
    params: ProbeParams,
    /// The stored tap threshold (`tap_threshold`, above 0 when present;
    /// absent is 0 = "tap not configured").
    tap_threshold: f64,
    /// The MCU the sensor (and its trigger) sits on (`i2c_mcu`).
    mcu_name: String,
}

impl EddyOptions {
    /// Read the section (`EddyProbeOffsets.__init__`,
    /// `probe.ProbeParameterHelper.__init__`, `EddyTap.__init__`, the
    /// `descend_z` / deprecated `z_offset` pair, and `sensor_type`).
    fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        // `sensor_type` is the one choice the section takes (currently only
        // `ldc1612`).
        let sensor_type = config.get_choice("sensor_type", &["ldc1612"], None)?;
        if sensor_type != "ldc1612" {
            return Err(ConfigError::new(format!(
                "Section '{}': unsupported sensor_type '{sensor_type}'",
                config.identifier()
            )));
        }

        // `descend_z`, with upstream's deprecated-`z_offset` fallback.
        let descend_z =
            if config.get_str("z_offset").is_some() && config.get_str("descend_z").is_none() {
                config.deprecate("z_offset", None);
                config.get_float_bounded("z_offset", None, None, None, Some(0.0), None)?
            } else {
                config.get_float_bounded("descend_z", None, None, None, Some(0.0), None)?
            };

        let offsets = ProbeOffsets {
            x: config.get_float("x_offset", Some(0.0))?,
            y: config.get_float("y_offset", Some(0.0))?,
            // `EddyProbeOffsets.get_offsets` reports 0 for z.
            z: 0.0,
        };

        // `probe.ProbeParameterHelper`'s option set, on the eddy section.
        let speed = config.get_float_bounded("speed", Some(5.0), None, None, Some(0.0), None)?;
        let lift_speed = config.get_optional_float("lift_speed")?;
        let params = ProbeParams {
            probe_speed: speed,
            lift_speed: lift_speed.unwrap_or(speed),
            samples: config.get_int_bounded("samples", Some(1), Some(1), None)? as i64,
            sample_retract_dist: config.get_float_bounded(
                "sample_retract_dist",
                Some(2.0),
                None,
                None,
                Some(0.0),
                None,
            )?,
            samples_result: config.get_choice(
                "samples_result",
                &["median", "average"],
                Some("average"),
            )?,
            samples_tolerance: config.get_float_bounded(
                "samples_tolerance",
                Some(0.100),
                Some(0.0),
                None,
                None,
                None,
            )?,
            samples_tolerance_retries: config.get_int_bounded(
                "samples_tolerance_retries",
                Some(0),
                Some(0),
                None,
            )? as i64,
        };

        // `tap_z_offset` adjusts the tap contact height (`adj_z_contact`),
        // which the tap analysis — not yet ported (module header) — would
        // consume; read here so the option stays accounted for
        // (`check_unused`).
        let _tap_z_offset = config.get_float("tap_z_offset", Some(0.0))?;
        // Upstream reads `tap_threshold` with `above=0.` but a default of 0:
        // the bound applies to a written value, not to an absent one, so the
        // presence of the option decides how it is read here too.
        let tap_threshold = if config.get_str("tap_threshold").is_some() {
            config.get_float_bounded("tap_threshold", None, None, None, Some(0.0), None)?
        } else {
            0.0
        };

        let mcu_name = config
            .get_str("i2c_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());

        Ok(Self {
            descend_z,
            offsets,
            params,
            tap_threshold,
            mcu_name,
        })
    }
}

/// The `KEY` names the eddy `PROBE` / `PROBE_ACCURACY` handlers read.
///
/// `METHOD` leads because `PrinterEddyProbe::start_probe_session` reads it
/// before anything else to pick the move; the parameter-helper keys follow in
/// [`probe_params`]'s read order; the last three are the keys the move the
/// method picked reads itself (`TAP_THRESHOLD` in `prep_tap`,
/// `SAMPLE_RETRACT_DIST` in `tap_once`, `SAMPLE_TIME` in `scan_once`).
const EDDY_PROBE_PARAMS: &[&str] = &[
    "METHOD",
    "SAMPLES_TOLERANCE",
    "SAMPLES",
    "PROBE_SPEED",
    "LIFT_SPEED",
    "SAMPLES_TOLERANCE_RETRIES",
    "SAMPLES_RESULT",
    "TAP_THRESHOLD",
    "SAMPLE_RETRACT_DIST",
    "SAMPLE_TIME",
];

/// `EddyParameterHelper.get_probe_params`: the plain parameters for the
/// descend path, upstream's forced set for `scan` / `rapid_scan` / `tap`
/// (one sample, no retract).
fn probe_params(defaults: &ProbeParams, gcmd: &GcodeCommand) -> Result<ProbeParams, CommandError> {
    let method = gcmd.get_str_default("METHOD", "").to_lowercase();
    if !matches!(method.as_str(), "scan" | "rapid_scan" | "tap") {
        return defaults.from_command(gcmd);
    }
    let samples_tolerance = gcmd.get_float_default("SAMPLES_TOLERANCE", 0.100)?;
    if samples_tolerance < 0.0 {
        return Err(CommandError::new(
            "Option 'SAMPLES_TOLERANCE' must have minimum of 0",
        ));
    }
    let samples = gcmd.get_int_default("SAMPLES", 1)?;
    if samples < 1 {
        return Err(CommandError::new("Option 'SAMPLES' must have minimum of 1"));
    }
    let probe_speed = gcmd.get_float_default("PROBE_SPEED", 5.0)?;
    if probe_speed <= 0.0 {
        return Err(CommandError::new("Option 'PROBE_SPEED' must be above 0.0"));
    }
    let lift_speed = gcmd.get_float_default("LIFT_SPEED", 5.0)?;
    if lift_speed <= 0.0 {
        return Err(CommandError::new("Option 'LIFT_SPEED' must be above 0.0"));
    }
    let retries = gcmd.get_int_default("SAMPLES_TOLERANCE_RETRIES", 0)?;
    if retries < 0 {
        return Err(CommandError::new(
            "Option 'SAMPLES_TOLERANCE_RETRIES' must have minimum of 0",
        ));
    }
    Ok(ProbeParams {
        probe_speed,
        lift_speed,
        samples: samples as i64,
        // scan/tap never retract between samples (`sample_retract_dist = 0.`).
        sample_retract_dist: 0.0,
        samples_tolerance,
        samples_tolerance_retries: retries,
        samples_result: gcmd.get_str_default("SAMPLES_RESULT", "average"),
    })
}

// ===========================================================================
// Measurement collection
// ===========================================================================

/// One probe request: the time window the analysis reads and the callback
/// that turns the measurements in it into a result (`EddyGatherSamples`'
/// `_probe_requests`, with the arguments already captured).
type GatherRequest = (
    f64,
    f64,
    Arc<dyn Fn(&[(f64, f64)]) -> Result<Coord, CommandError> + Send + Sync>,
);

/// Gather the sensor's samples and turn them into probe positions
/// (`probe_eddy_current.EddyGatherSamples`).
///
/// The producer side of [`SampleDelivery`] holds the open session's gather:
/// the ldc1612 batch stream arrives at [`PrinterEddyProbe`]'s forwarding
/// client and is delivered here sample by sample
/// (`sensor_helper.add_client(self._add_sensor_message)` upstream, one row at
/// a time here).
pub struct EddyGatherSamples {
    /// The machine, for file-output mode and the clock estimate.
    printer: Weak<Printer>,
    /// The sensor's clock, for the outage check.
    sensor: Arc<Ldc1612>,
    /// Measurements received so far, `(print time, frequency)`, oldest first.
    measurements: Mutex<Vec<(f64, f64)>>,
    /// Requests waiting for a window of samples, FIFO.
    requests: Mutex<Vec<GatherRequest>>,
    /// The deferred results, in request order (`(res, errmsg)` upstream —
    /// an analysis error is raised at pull time, not at arrival).
    results: Mutex<Vec<Result<Coord, String>>>,
    /// Set by [`EddyGatherSamples::finish`]: later deliveries stop.
    finished: AtomicBool,
}

impl EddyGatherSamples {
    /// Bind a gather to the machine and its sensor (`__init__`, whose
    /// `add_client` this port wires one level up — see
    /// [`PrinterEddyProbe`]'s forwarding client).
    fn new(printer: &Weak<Printer>, sensor: Arc<Ldc1612>) -> Self {
        Self {
            printer: printer.clone(),
            sensor,
            measurements: Mutex::new(Vec::new()),
            requests: Mutex::new(Vec::new()),
            results: Mutex::new(Vec::new()),
            finished: AtomicBool::new(false),
        }
    }

    /// Stop collecting (`finish`).
    fn finish(&self) {
        self.finished.store(true, Ordering::SeqCst);
    }

    /// Queue an analysis for `[start_time, end_time]`
    /// (`add_probe_request`).
    fn add_probe_request(
        &self,
        start_time: f64,
        end_time: f64,
        cb: Arc<dyn Fn(&[(f64, f64)]) -> Result<Coord, CommandError> + Send + Sync>,
    ) {
        self.requests
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push((start_time, end_time, cb));
    }

    /// Run every request whose window the received samples already cover
    /// (`_check_sensor_messages`). Skipped under file-output mode, where
    /// upstream's fake serial delivers no messages at all and every result
    /// comes from the dummy branch in
    /// [`EddyGatherSamples::pull_probed`] instead — this port's simulator
    /// *does* push batch messages, so the same outcome is reached by the same
    /// explicit branch rather than by silence.
    fn check_requests(&self, fileoutput: bool) {
        if fileoutput {
            return;
        }
        loop {
            let next = {
                let requests = self.requests.lock().unwrap_or_else(|p| p.into_inner());
                let measurements = self.measurements.lock().unwrap_or_else(|p| p.into_inner());
                let Some((start, end, _)) = requests.first() else {
                    return;
                };
                match measurements.last() {
                    // Not enough data yet (`self._sensor_messages[-1] … < end_time`).
                    Some((last_time, _)) if *last_time < *end => return,
                    None => return,
                    _ => (*start, *end),
                }
            };
            // Pull the measurements in the window, discarding what no later
            // request can need (`_pull_measurements` + the `del` of the
            // consumed prefix).
            let window = {
                let mut measurements = self.measurements.lock().unwrap_or_else(|p| p.into_inner());
                let (start, end) = next;
                let window: Vec<(f64, f64)> = measurements
                    .iter()
                    .copied()
                    .filter(|(time, _)| *time >= start && *time <= end)
                    .collect();
                measurements.retain(|(time, _)| *time >= start);
                window
            };
            let cb = {
                let mut requests = self.requests.lock().unwrap_or_else(|p| p.into_inner());
                requests.remove(0).2
            };
            // Defer errors to the pull, as upstream does (`errmsg`).
            let result = cb(&window).map_err(|err| err.to_string());
            self.results
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(result);
        }
    }

    /// Wait for the queued requests and take their results
    /// (`pull_probed` + `_await_sensor_messages`).
    ///
    /// Under file-output mode every pending request answers with upstream's
    /// dummy `create_probe_result((0., 0., 0.))` without running its analysis.
    async fn pull_probed(&self) -> Result<Vec<Coord>, CommandError> {
        let fileoutput = self
            .printer
            .upgrade()
            .map(|printer| printer.is_fileoutput())
            .unwrap_or(false);
        loop {
            let pending = self
                .requests
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .len();
            if pending == 0 {
                break;
            }
            if fileoutput {
                let mut requests = self.requests.lock().unwrap_or_else(|p| p.into_inner());
                requests.clear();
                let mut results = self.results.lock().unwrap_or_else(|p| p.into_inner());
                for _ in 0..pending {
                    results.push(Ok(Coord::new(0.0, 0.0, 0.0, 0.0)));
                }
                break;
            }
            // Outage: the machine's print time has passed the window by more
            // than a second with no samples to show for it.
            let end = self
                .requests
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .first()
                .map(|(_, end, _)| *end)
                .unwrap_or(0.0);
            let printer = self
                .printer
                .upgrade()
                .ok_or_else(|| CommandError::new("probe_eddy_current sensor outage"))?;
            let now = printer.reactor().monotonic();
            let est = self.sensor.estimated_print_time(now);
            match est {
                Some(est) if est > end + 1.0 => {
                    return Err(CommandError::new("probe_eddy_current sensor outage"))
                }
                _ => {}
            }
            self.check_requests(fileoutput);
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
        }
        let results = std::mem::take(&mut *self.results.lock().unwrap_or_else(|p| p.into_inner()));
        let mut pulled = Vec::with_capacity(results.len());
        for result in results {
            pulled.push(result.map_err(CommandError::new)?);
        }
        Ok(pulled)
    }
}

impl SampleDelivery for EddyGatherSamples {
    /// One sample at `time` whose sensor value (frequency) is `value`
    /// (`_add_sensor_message`'s row append, one row per call).
    fn deliver_sample(&self, time: f64, value: f64) {
        if self.finished.load(Ordering::SeqCst) {
            return;
        }
        let fileoutput = self
            .printer
            .upgrade()
            .map(|printer| printer.is_fileoutput())
            .unwrap_or(false);
        if fileoutput {
            // Upstream's file-output serial never delivers messages either.
            return;
        }
        self.measurements
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push((time, value));
        self.check_requests(fileoutput);
    }
}

/// Generate a probe result from the average of a set of measurements
/// (`probe_results_from_avg` + upstream's `manual_probe.create_probe_result`:
/// `bed = (test.x + x_offset, test.y + y_offset, test.z − sensor_z)`).
fn probe_results_from_avg(
    measures: &[(f64, f64)],
    toolhead_pos: Coord,
    calibration: &EddyCalibration,
    offsets: ProbeOffsets,
) -> Result<Coord, CommandError> {
    if measures.is_empty() {
        return Err(CommandError::new(
            "Unable to obtain probe_eddy_current sensor readings",
        ));
    }
    let freq_avg = measures.iter().map(|(_, freq)| *freq).sum::<f64>() / measures.len() as f64;
    let sensor_z = calibration.freq_to_height(freq_avg);
    if sensor_z <= -OUT_OF_RANGE || sensor_z >= OUT_OF_RANGE {
        return Err(CommandError::new(
            "probe_eddy_current sensor not in valid range",
        ));
    }
    Ok(Coord::new(
        toolhead_pos.x() + offsets.x,
        toolhead_pos.y() + offsets.y,
        toolhead_pos.z() - sensor_z,
        toolhead_pos.e(),
    ))
}

// ===========================================================================
// Virtual endstop chip
// ===========================================================================

/// Upstream's `HomingViaProbeHelper` for the eddy probe: the `probe` pin chip
/// whose `z_virtual_endstop` resolves to the probe's `trigger_analog` object,
/// with `descend_z` standing in as `get_position_endstop`.
pub struct EddyProbeChip {
    /// The trigger the rail homes through.
    endstop: Arc<McuTriggerAnalog>,
    /// What a `probe:z_virtual_endstop` rail uses as `position_endstop`
    /// (`descend_z`, from `HomingViaProbeHelper(config, descend_z)`).
    position: f64,
}

impl PinChip for EddyProbeChip {
    fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        Err(PinError::Unsupported("digital_out".to_string()))
    }

    fn setup_endstop_dyn(&self, params: &PinParams) -> Result<Arc<dyn HomingEndstop>, PinError> {
        check_virtual_endstop(params)?;
        let endstop: Arc<dyn HomingEndstop> = self.endstop.clone();
        Ok(endstop)
    }

    fn virtual_endstop_position(&self, params: &PinParams) -> Option<f64> {
        check_virtual_endstop(params).ok()?;
        Some(self.position)
    }
}

// ===========================================================================
// Probe sessions
// ===========================================================================

/// Which session the open round runs (`start_probe_session`'s `METHOD`
/// dispatch: `EddyDescend` / `EddyTap` / `EddyScanningProbe`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ActiveSession {
    /// Descend until trigger, wrapped in sample averaging upstream
    /// (`SampleAveragingHelper` over `EddyDescend`).
    Descend,
    /// The tap probe (`SampleAveragingHelper` over `EddyTap`).
    Tap,
    /// `METHOD=scan` / `METHOD=rapid_scan` (`EddyScanningProbe`).
    Scan,
}

/// The `probe` object the eddy section registers (`PrinterEddyProbe`).
pub struct PrinterEddyProbe {
    /// The section's identifier, for `get_status` and `Debug`.
    identifier: String,
    /// The machine, for the toolhead, events and file-output mode.
    printer: Weak<Printer>,
    /// The frequency→height table.
    calibration: Arc<EddyCalibration>,
    /// The sensor: batches, conversions, the trigger's attach point.
    sensor: Arc<Ldc1612>,
    /// The firmware trigger the probe homes and probes through.
    trigger: Arc<McuTriggerAnalog>,
    /// The tap filter design (`EddyTap._setup_tap`'s lowpass + derivative,
    /// fixed-point sections computed once; the millihertz scale rides along).
    tap_design: SosFilterDesign,
    /// Descend target (`descend_z`).
    descend_z: f64,
    /// Probe-to-nozzle offsets (`get_offsets`; z is always 0 here).
    offsets: ProbeOffsets,
    /// The `probe.ProbeParameterHelper` defaults.
    params: ProbeParams,
    /// The stored tap threshold (`tap_threshold`; 0 = not configured).
    tap_threshold: f64,
    /// The Z a probing move descends to (`probe.lookup_minimum_z`).
    z_min_position: f64,
    /// What `get_status` reports (`ProbeCommandHelper.get_status`).
    state: Arc<ProbeCommandState>,
    /// The open session: its kind and the gather collecting its samples.
    /// Shared (`Arc`) so the sensor-side forwarding client — which cannot
    /// hold the probe itself — reaches the open session by clone.
    active: Arc<Mutex<Option<(ActiveSession, Arc<EddyGatherSamples>)>>>,
    /// Completed sample sets (`SampleAveragingHelper.results`).
    results: Mutex<Vec<Coord>>,
    /// Whether the ldc1612 forwarding client was installed (once, at the
    /// first session — clients must not start the batch loop before the MCU
    /// connects).
    client_installed: AtomicBool,
}

impl PrinterEddyProbe {
    /// Build the section (`PrinterEddyProbe.__init__`): calibration, sensor,
    /// trigger, tap design, the `probe` virtual chip.
    ///
    /// # Errors
    /// A missing/malformed option, an unknown MCU, an oid shortage, or a
    /// `probe` chip already registered (`[probe]` / `[bltouch]` alongside
    /// this section — upstream refuses the same way).
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let calibration = Arc::new(EddyCalibration::read(config)?);
        let options = EddyOptions::read(config)?;

        // The sensor (`sensors = {"ldc1612": ldc1612.LDC1612}`).
        let sensor = Arc::new(Ldc1612::new(
            config,
            printer,
            Some(Arc::clone(&calibration) as Arc<dyn Calibration>),
        )?);

        // The MCU handle the trigger and its SOS filter build on.
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let chip = pins.chip_as::<McuChip>(&options.mcu_name).ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}': unknown MCU '{}'",
                options.mcu_name
            ))
        })?;

        // The tap filter design (`EddyTap._setup_tap`): a 25 Hz 4th-order
        // lowpass plus its derivative at the ldc1612's rate, in fixed point —
        // the one entry the pre-generated table carries, so no SciPy path is
        // ever reached.
        let mut filter = DigitalFilter::new(SPS);
        filter.add_lowpass(25.0, 4)?;
        filter.add_derivative();
        let coeff_frac_bits = filter.coeff_frac_bits();
        let sections = filter.to_fixed_sections(coeff_frac_bits)?;
        let states = filter.to_fixed_state(0.0)?;
        let scale_value = FRAC_HZ * sensor.convert_raw_to_frequency(1);
        let scale_frac_bits = calc_frac_bits(&[scale_value]);
        let scale = to_fixed_32(scale_value, scale_frac_bits)
            .map_err(|err| ConfigError::new(err.to_string()))?;
        let tap_design = SosFilterDesign {
            sections,
            states,
            offset: 0,
            scale,
            scale_frac_bits: u8::try_from(scale_frac_bits)
                .map_err(|_| ConfigError::new("tap filter scale_frac_bits does not fit a byte"))?,
            auto_offset: true,
            coeff_frac_bits: u8::try_from(coeff_frac_bits)
                .map_err(|_| ConfigError::new("tap filter coeff_frac_bits does not fit a byte"))?,
        };

        // The trigger object (`trigger_analog.MCU_trigger_analog`), the
        // sensor→trigger attach (`LDC1612.setup_trigger_analog`) and the
        // sensor-side error decoder.
        let sos = SosFilter::new(&chip, tap_design.sections.len() as u8)
            .map_err(|err| ConfigError::new(err.to_string()))?;
        let trigger = Arc::new(
            McuTriggerAnalog::new((*chip).clone(), SPS, Some(sos))
                .map_err(|err| ConfigError::new(err.to_string()))?,
        );
        sensor
            .setup_trigger_analog(trigger.oid())
            .map_err(|err| ConfigError::new(err.to_string()))?;
        trigger.set_sensor_error_lookup({
            // The decoder must not pin the sensor: `Ldc1612State` keeps
            // `pins`, whose `probe` chip holds this trigger, so a strong
            // `sensor` here closes a cycle (trigger → sensor → state → pins →
            // chip → trigger) that outlives the machine — its `Mcu` never drops,
            // its blocking device read stays parked, and the process hangs at
            // shutdown. A gone sensor means a gone machine: the same fallback
            // `error_text` uses.
            let sensor = Arc::downgrade(&sensor);
            move |code| {
                sensor
                    .upgrade()
                    .map(|sensor| sensor.lookup_sensor_error(u16::from(code)))
                    .unwrap_or_else(|| "Unknown ldc1612 error".to_string())
            }
        });

        // `probe.lookup_minimum_z`: `[stepper_z] position_min`, else
        // `[printer] minimum_z_position`, else 0.
        let z_min_position = match config.sibling("stepper_z") {
            Some(sibling) => sibling.get_float("position_min", Some(0.0))?,
            None => match config.sibling("printer") {
                Some(sibling) => sibling.get_float("minimum_z_position", Some(0.0))?,
                None => 0.0,
            },
        };

        // Upstream: `HomingViaProbeHelper(config, descend_z)` registers the
        // `probe` chip, which is what resolves
        // `endstop_pin: probe:z_virtual_endstop` to this trigger.
        pins.register_chip(
            "probe",
            Arc::new(EddyProbeChip {
                endstop: Arc::clone(&trigger),
                position: options.descend_z,
            }),
        )
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self {
            identifier,
            printer: Arc::downgrade(printer),
            calibration,
            sensor,
            trigger,
            tap_design,
            descend_z: options.descend_z,
            offsets: options.offsets,
            params: options.params,
            tap_threshold: options.tap_threshold,
            z_min_position,
            state: Arc::new(ProbeCommandState::default()),
            active: Arc::new(Mutex::new(None)),
            results: Mutex::new(Vec::new()),
            client_installed: AtomicBool::new(false),
        })
    }

    /// Register `PROBE` / `QUERY_PROBE` / `PROBE_ACCURACY`
    /// (`probe.ProbeCommandHelper(config, self, can_set_z_offset=False)`), the
    /// `gcode:command_error` session cleanup, and the tap-calibrate command.
    fn register_commands(self: &Arc<Self>) -> Result<(), ConfigError> {
        let gcode = self
            .printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT))
            .expect("the loader registers `gcode` before any section");

        // A failing command must not leave the session open
        // (`probe.py:_handle_command_error`).
        {
            let weak = Arc::downgrade(self);
            self.printer
                .upgrade()
                .expect("the printer outlives its section")
                .register_event_handler(
                    KlippyEvent::GcodeCommandError,
                    Box::new(move |_event| {
                        if let Some(probe) = weak.upgrade() {
                            probe.abort_session();
                        }
                    }),
                );
        }

        // QUERY_PROBE: upstream binds no query callback for an eddy probe
        // (`ProbeCommandHelper(…, query_endstop=None)`).
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

        // PROBE: one sample set at the current XY.
        {
            let probe = Arc::clone(self);
            gcode
                .register_command_with_params(
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
                                .ok_or_else(state_error)?;
                            probe.end_probe_session()?;
                            gcmd.respond_info(&format!(
                                "Result: at {:.3},{:.3} estimate contact at z={:.6}",
                                pos.x(),
                                pos.y(),
                                pos.z()
                            ));
                            *probe
                                .state
                                .last_z_result
                                .lock()
                                .unwrap_or_else(|p| p.into_inner()) = pos.z() + probe.offsets.z;
                            Ok(())
                        })
                    }),
                    Some("Probe Z-height at current XY position"),
                    EDDY_PROBE_PARAMS,
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        // PROBE_ACCURACY: `SAMPLES` single-sample probes, then the spread
        // (`probe.py:cmd_PROBE_ACCURACY`, driving this object's sessions the
        // way upstream drives `self.probe`).
        {
            let probe = Arc::clone(self);
            // It re-drives the same session through `start_probe_session` /
            // `run_probe`, so it accepts the same names as `PROBE`.
            gcode
                .register_command_with_params(
                    "PROBE_ACCURACY",
                    Arc::new(move |gcmd| {
                        let probe = Arc::clone(&probe);
                        Box::pin(async move {
                            let params = probe.probe_params(gcmd)?;
                            let sample_count = gcmd.get_int_default("SAMPLES", 10)?;
                            if sample_count < 1 {
                                return Err(CommandError::new(
                                    "Option 'SAMPLES' must have minimum of 1",
                                ));
                            }
                            let toolhead = probe.toolhead()?;
                            let pos = toolhead
                                .position()
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            gcmd.respond_info(&format!(
                                "PROBE_ACCURACY at X:{:.3} Y:{:.3} Z:{:.3} (samples={} retract={:.3} speed={:.1} lift_speed={:.1})",
                                pos.x(), pos.y(), pos.z(), sample_count,
                                params.sample_retract_dist, params.probe_speed, params.lift_speed,
                            ));
                            // The accuracy loop probes one sample at a time
                            // (`fo_params['SAMPLES'] = '1'`).
                            let mut fo_params = gcmd.get_command_parameters().clone();
                            fo_params.insert("SAMPLES".to_string(), "1".to_string());
                            let gcode = probe
                                .printer
                                .upgrade()
                                .and_then(|printer| {
                                    printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                                })
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            let fo_gcmd = gcode.create_gcode_command("", "", fo_params);
                            probe.start_probe_session(&fo_gcmd)?;
                            for _ in 0..sample_count {
                                probe.run_probe(&fo_gcmd).await?;
                                let pos = toolhead.position().ok_or_else(|| {
                                    CommandError::new("Printer is not ready")
                                })?;
                                let mut lift = pos;
                                lift.set_axis(
                                    crate::core::klippy::mathutil::Z_AXIS,
                                    pos.z() + params.sample_retract_dist,
                                );
                                toolhead.move_to(lift, params.lift_speed)?;
                            }
                            let positions = probe.pull_probed_results();
                            probe.end_probe_session()?;

                            let max_value = positions
                                .iter()
                                .map(Coord::z)
                                .fold(f64::NEG_INFINITY, f64::max);
                            let min_value = positions
                                .iter()
                                .map(Coord::z)
                                .fold(f64::INFINITY, f64::min);
                            let range_value = max_value - min_value;
                            let avg_value = calc_probe_z_average(&positions, "average").z();
                            let median = calc_probe_z_average(&positions, "median").z();
                            let deviation_sum: f64 = positions
                                .iter()
                                .map(|p| (p.z() - avg_value).powi(2))
                                .sum();
                            let sigma = (deviation_sum / positions.len() as f64).sqrt();
                            gcmd.respond_info(&format!(
                                "probe accuracy results: maximum {max_value:.6}, minimum {min_value:.6}, range {range_value:.6}, average {avg_value:.6}, median {median:.6}, standard deviation {sigma:.6}"
                            ));
                            Ok(())
                        })
                    }),
                    Some("Probe Z-height accuracy at current XY position"),
                    EDDY_PROBE_PARAMS,
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        Ok(())
    }

    /// The toolhead (`PrinterSessionHelper::toolhead`).
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// Close the open session after a command error
    /// (`SampleAveragingHelper._handle_command_error`).
    fn abort_session(&self) {
        if let Some((_, gather)) = self.active.lock().unwrap_or_else(|p| p.into_inner()).take() {
            gather.finish();
        }
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
    }

    /// Install the ldc1612 forwarding client once, on the first session
    /// (`EddyGatherSamples.__init__`'s `add_client`, which upstream also only
    /// reaches while the machine runs). The client stays registered for the
    /// machine's lifetime and forwards rows only into an *open* session's
    /// gather — no accumulation between sessions.
    fn ensure_forwarding_client(&self) {
        if self.client_installed.swap(true, Ordering::SeqCst) {
            return;
        }
        let active = Arc::clone(&self.active);
        self.sensor.add_client(move |message| {
            let gather = {
                let active = active.lock().unwrap_or_else(|p| p.into_inner());
                active.as_ref().map(|(_, gather)| Arc::clone(gather))
            };
            if let Some(gather) = gather {
                if let Some(rows) = message.get("data").and_then(Value::as_array) {
                    for row in rows {
                        if let (Some(time), Some(freq)) = (
                            row.get(0).and_then(Value::as_f64),
                            row.get(1).and_then(Value::as_f64),
                        ) {
                            gather.deliver_sample(time, freq);
                        }
                    }
                }
            }
            true
        });
    }

    /// `EddyDescend._prep_trigger_analog`: pass-through filter, full raw
    /// range, trigger `gt` the descend height's frequency.
    fn prep_descend(&self) -> Result<(), CommandError> {
        let sos_filter = self.trigger.sos_filter();
        // Upstream: `set_filter_design(None)` + `set_offset_scale(0, 1.)` —
        // the default design is the pass-through at scale 1.
        sos_filter.set_filter_design(SosFilterDesign::default());
        self.trigger.set_raw_range(0, MAX_VALID_RAW_VALUE);
        let trigger_freq = self.calibration.height_to_freq(self.descend_z)?;
        let raw = self.sensor.convert_frequency_to_raw(trigger_freq);
        self.trigger.set_trigger(TriggerAnalogType::Gt, raw as i32);
        Ok(())
    }

    /// `EddyTap._prep_trigger_analog_tap`: the lowpass+derivative design in
    /// millihertz, and a `diff_peak_gt` threshold scaled to the probe speed.
    fn prep_tap(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let tap_threshold = gcmd.get_float_default("TAP_THRESHOLD", self.tap_threshold)?;
        if tap_threshold <= 0.0 {
            // Upstream reports a written non-positive value as a parameter
            // bound and an absent one as "Tap not configured"; both end here
            // as the same refusal.
            return Err(CommandError::new("Tap not configured"));
        }
        let params = probe_params(&self.params, gcmd)?;
        let sos_filter = self.trigger.sos_filter();
        sos_filter.set_filter_design(self.tap_design.clone());
        self.trigger.set_raw_range(0, MAX_VALID_RAW_VALUE);
        let adj_thresh = tap_threshold * params.probe_speed / SPS;
        let samp_thresh = (FRAC_HZ * adj_thresh + 0.5) as i32;
        self.trigger
            .set_trigger(TriggerAnalogType::DiffPeakGt, samp_thresh);
        Ok(())
    }

    /// One descend probing move and its analysis request
    /// (`EddyDescend.run_probe`).
    async fn descend_once(
        &self,
        speed: f64,
        gather: &Arc<EddyGatherSamples>,
        toolhead: &Arc<ToolHeadObject>,
    ) -> Result<Coord, CommandError> {
        let mut target = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        target.set_axis(crate::core::klippy::mathutil::Z_AXIS, self.z_min_position);
        toolhead.probing_move(&*self.trigger, target, speed).await?;

        let start_time = self.trigger.last_trigger_time() + 0.050;
        let end_time = start_time + 0.100;
        let toolhead_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let offsets = self.offsets;
        let calibration = Arc::clone(&self.calibration);
        gather.add_probe_request(
            start_time,
            end_time,
            Arc::new(move |window| {
                probe_results_from_avg(window, toolhead_pos, &calibration, offsets)
            }),
        );
        gather
            .pull_probed()
            .await?
            .into_iter()
            .next()
            .ok_or_else(state_error)
    }

    /// One tap probing move: descend, lift, and analyze the retract window
    /// (`EddyTap.run_probe`).
    async fn tap_once(
        &self,
        gcmd: &GcodeCommand,
        speed: f64,
        lift_speed: f64,
        gather: &Arc<EddyGatherSamples>,
        toolhead: &Arc<ToolHeadObject>,
    ) -> Result<Coord, CommandError> {
        let mut target = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        target.set_axis(crate::core::klippy::mathutil::Z_AXIS, self.z_min_position);
        toolhead.probing_move(&*self.trigger, target, speed).await?;

        let lift_dist = gcmd.get_float_default("SAMPLE_RETRACT_DIST", 4.0)?;
        if lift_dist <= 0.0 {
            return Err(CommandError::new(
                "Option 'SAMPLE_RETRACT_DIST' must be above 0.0",
            ));
        }
        let mut haltpos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        haltpos.set_axis(
            crate::core::klippy::mathutil::Z_AXIS,
            haltpos.z() + lift_dist,
        );
        let retract_start = toolhead.get_last_move_time();
        toolhead.move_to(haltpos, lift_speed)?;

        let start_time = retract_start - 0.010;
        let end_time = retract_start + 0.150;
        gather.add_probe_request(
            start_time,
            end_time,
            Arc::new(|_window| {
                Err(CommandError::new(
                    "Eddy tap analysis needs the stepper time-position conversion \
                     (mcu_to_commanded_position), which this port has not shipped yet",
                ))
            }),
        );
        gather
            .pull_probed()
            .await?
            .into_iter()
            .next()
            .ok_or_else(state_error)
    }

    /// One `METHOD=scan` sample window (`EddyScanningProbe.run_probe`).
    ///
    /// `rapid_scan` takes the same single-window path: upstream schedules
    /// one request per lookahead flush, which needs a callback surface this
    /// port has not shipped. Under file-output mode (every corpus case) both
    /// answer with upstream's identical dummy results.
    async fn scan_once(
        &self,
        gcmd: &GcodeCommand,
        gather: &Arc<EddyGatherSamples>,
        toolhead: &Arc<ToolHeadObject>,
    ) -> Result<(), CommandError> {
        let sample_time = gcmd.get_float_default("SAMPLE_TIME", 0.100)?;
        if sample_time <= 0.0 {
            return Err(CommandError::new("Option 'SAMPLE_TIME' must be above 0.0"));
        }
        let printtime = toolhead.get_last_move_time();
        toolhead.dwell(0.050 + sample_time);
        let start_time = printtime + 0.050;
        let end_time = start_time + sample_time;
        // The position the window samples is where the toolhead stands while
        // the dwell runs (`_analyze_scan`'s lookup at `pos_time`).
        let toolhead_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let offsets = self.offsets;
        let calibration = Arc::clone(&self.calibration);
        gather.add_probe_request(
            start_time,
            end_time,
            Arc::new(move |window| {
                probe_results_from_avg(window, toolhead_pos, &calibration, offsets)
            }),
        );
        let results = gather.pull_probed().await?;
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .extend(results);
        Ok(())
    }

    /// The averaged multi-sample run for descend/tap
    /// (`SampleAveragingHelper.run_probe`).
    async fn run_multisample(
        &self,
        gcmd: &GcodeCommand,
        params: &ProbeParams,
        kind: ActiveSession,
        gather: &Arc<EddyGatherSamples>,
        toolhead: &Arc<ToolHeadObject>,
    ) -> Result<(), CommandError> {
        let probexy = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let mut positions: Vec<Coord> = Vec::new();
        let mut retries = 0i64;
        while (positions.len() as i64) < params.samples {
            let pos = match kind {
                ActiveSession::Tap => {
                    self.tap_once(
                        gcmd,
                        params.probe_speed,
                        params.lift_speed,
                        gather,
                        toolhead,
                    )
                    .await?
                }
                _ => {
                    self.descend_once(params.probe_speed, gather, toolhead)
                        .await?
                }
            };
            positions.push(pos);
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
                let mut lift = probexy;
                lift.set_axis(
                    crate::core::klippy::mathutil::Z_AXIS,
                    pos.z() + params.sample_retract_dist,
                );
                toolhead.move_to(lift, params.lift_speed)?;
            }
        }
        let epos = calc_probe_z_average(&positions, &params.samples_result);
        // A consumer edits the averaged result in place before it is stored; the
        // stored value is what the handlers left (`probe.py:329`).
        let results = ProbeResultsHandle::new(vec![epos]);
        if let Some(printer) = self.printer.upgrade() {
            printer.send_event(&KlippyEvent::ProbeUpdateResults {
                results: results.clone(),
            });
        }
        let epos = results.to_vec().first().copied().unwrap_or(epos);
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(epos);
        if gcmd.command() != "G28" {
            gcmd.respond_info(&format!(
                "probe: at {:.3},{:.3} bed will contact at z={:.6}",
                epos.x(),
                epos.y(),
                epos.z()
            ));
        }
        Ok(())
    }
}

impl PrinterObject for PrinterEddyProbe {
    fn get_status(&self, _eventtime: f64) -> Value {
        command_status(&self.identifier, &self.state)
    }
}

impl std::fmt::Debug for PrinterEddyProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterEddyProbe")
            .field("identifier", &self.identifier)
            .field("descend_z", &self.descend_z)
            .finish_non_exhaustive()
    }
}

impl ProbeSession for PrinterEddyProbe {
    /// Open a session, dispatching on `METHOD`
    /// (`PrinterEddyProbe.start_probe_session`).
    fn start_probe_session(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        {
            let active = self.active.lock().unwrap_or_else(|p| p.into_inner());
            if active.is_some() {
                return Err(state_error());
            }
        }
        self.calibration.verify_calibrated()?;
        let method = gcmd.get_str_default("METHOD", "").to_lowercase();
        let kind = match method.as_str() {
            "scan" | "rapid_scan" => ActiveSession::Scan,
            "tap" => {
                self.prep_tap(gcmd)?;
                ActiveSession::Tap
            }
            _ => {
                self.prep_descend()?;
                ActiveSession::Descend
            }
        };
        // The producer side installs once, at the first session — the MCU is
        // connected by then (the client must not start the batch loop before).
        self.ensure_forwarding_client();
        let gather = Arc::new(EddyGatherSamples::new(
            &self.printer,
            Arc::clone(&self.sensor),
        ));
        *self.results.lock().unwrap_or_else(|p| p.into_inner()) = Vec::new();
        *self.active.lock().unwrap_or_else(|p| p.into_inner()) = Some((kind, gather));
        Ok(())
    }

    /// One sample set in the open session (`run_probe`).
    fn run_probe<'a>(
        &'a self,
        gcmd: &'a GcodeCommand,
    ) -> crate::core::klippy::gcode::CommandFuture<'a> {
        Box::pin(async move {
            let (kind, gather) = {
                let active = self.active.lock().unwrap_or_else(|p| p.into_inner());
                match active.as_ref() {
                    Some((kind, gather)) => (*kind, Arc::clone(gather)),
                    None => return Err(state_error()),
                }
            };
            let toolhead = self.toolhead()?;
            // `SampleAveragingHelper._probe`'s homed check.
            let homed = toolhead.get_status(0.0)["homed_axes"]
                .as_str()
                .unwrap_or("")
                .to_string();
            if !homed.contains('z') {
                return Err(CommandError::new("Must home before probe"));
            }
            match kind {
                ActiveSession::Scan => self.scan_once(gcmd, &gather, &toolhead).await,
                _ => {
                    let params = probe_params(&self.params, gcmd)?;
                    self.run_multisample(gcmd, &params, kind, &gather, &toolhead)
                        .await
                }
            }
        })
    }

    /// The parameters a command asks for (`EddyParameterHelper`).
    fn probe_params(&self, gcmd: &GcodeCommand) -> Result<ProbeParams, CommandError> {
        probe_params(&self.params, gcmd)
    }

    /// Take the completed sample sets (`pull_probed_results`).
    fn pull_probed_results(&self) -> Vec<Coord> {
        let mut taken =
            std::mem::take(&mut *self.results.lock().unwrap_or_else(|p| p.into_inner()));
        // The scan path reports its results here upstream
        // (`EddyScanningProbe.pull_probed_results`); descend/tap already
        // sent the event when their set completed. A consumer edits every
        // result in place before they are returned (`probe.py:329`).
        let is_scan = self
            .active
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
            .is_some_and(|(kind, _)| *kind == ActiveSession::Scan);
        if is_scan && !taken.is_empty() {
            let results = ProbeResultsHandle::new(taken);
            if let Some(printer) = self.printer.upgrade() {
                printer.send_event(&KlippyEvent::ProbeUpdateResults {
                    results: results.clone(),
                });
            }
            taken = results.to_vec();
        }
        taken
    }

    /// Close the session (`end_probe_session`).
    fn end_probe_session(&self) -> Result<(), CommandError> {
        let gather = self
            .active
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
            .map(|(_, gather)| gather)
            .ok_or_else(state_error)?;
        gather.finish();
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
        Ok(())
    }

    /// The probe's offsets (`get_offsets`): x/y from the section, z always 0
    /// (`EddyProbeOffsets` / the `METHOD=tap` zeroes).
    fn offsets(&self) -> ProbeOffsets {
        self.offsets
    }
}

impl PrinterEddyProbe {
    /// A batch client at the sensor level (`add_client` upstream) — the
    /// external consumer surface (the calibration tool's data gather and the
    /// `ldc1612/dump_ldc1612` path share the stream).
    pub fn add_client(&self, client: impl Fn(&Value) -> bool + Send + Sync + 'static) {
        self.sensor.add_client(client);
    }

    /// Give the probe `temperature_probe`'s drift helper
    /// (`PrinterEddyProbe.register_drift_compensation`,
    /// `probe_eddy_current.py:1073-1074`): every frequency the table converts
    /// is adjusted through it from here on.
    pub fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensation>) {
        self.calibration.register_drift_compensation(comp);
    }

    /// The section identifier.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }
}

// ===========================================================================
// TAP_CALIBRATE
// ===========================================================================

/// `PROBE_EDDY_CURRENT_TAP_CALIBRATE` (`probe_eddy_current.EddyTapCalibration`):
/// the technical readout over the main calibration, and the `TAP=` trial runs
/// that drive a `METHOD=tap` session through the registered `probe` object.
struct EddyTapCalibration;

impl EddyTapCalibration {
    /// Register the command (`__init__`).
    ///
    /// # Errors
    /// A g-code registration clash.
    fn register(
        printer: &Arc<Printer>,
        calibration: Arc<EddyCalibration>,
    ) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let handler: CommandHandler = {
            let calibration = Arc::clone(&calibration);
            let this_printer = Arc::downgrade(printer);
            Arc::new(move |gcmd| {
                let calibration = Arc::clone(&calibration);
                let this_printer = this_printer.clone();
                Box::pin(async move {
                    let printer = this_printer
                        .upgrade()
                        .ok_or_else(|| CommandError::new("Printer is not ready"))?;

                    // `_analyze_main_calibration`: the best-fit quadratic
                    // through the calibration points at or below 0.750.
                    let (freqs, zpos) = calibration.get_calibration();
                    let coeffs = if freqs.len() < 2 {
                        None
                    } else {
                        let mut eqs: Vec<Vec<f64>> = Vec::new();
                        let mut ans: Vec<Vec<f64>> = Vec::new();
                        for (freq, z) in freqs.iter().zip(zpos.iter()) {
                            if *z <= 0.750 {
                                ans.push(vec![*freq]);
                                eqs.push(vec![1.0, *z, z * z]);
                            }
                        }
                        solve_linear_equations(&eqs, &ans)
                    };

                    match gcmd.get_str_default("TAP", "").as_str() {
                        // No `TAP`: the technical readout — the calibration
                        // fit, then the last-tap line.
                        "" => {
                            let fit_line = match &coeffs {
                                Some(c) => format!(
                                    "Calibration: f={:.3} s={:.3} q={:.3}",
                                    c[0][0], c[1][0], c[2][0]
                                ),
                                None => "Main calibration data not available.".to_string(),
                            };
                            // The last-tap analysis (`_analyze_pullback`)
                            // has not been ported (module header), so there
                            // is none to report — exactly what upstream
                            // answers under file-output mode, where that
                            // analysis never runs either.
                            gcmd.respond_info(&format!(
                                "{fit_line}\n\nRun tap probe for last tap analysis."
                            ));
                            Ok(())
                        }
                        // `TAP=guess`: the trial tap at a threshold derived
                        // from the calibration slope (`_try_tap`).
                        "guess" => {
                            let coeffs = coeffs.ok_or_else(|| {
                                CommandError::new(
                                    "Must complete PROBE_EDDY_CURRENT_CALIBRATE first",
                                )
                            })?;
                            Self::try_tap(&printer, gcmd, coeffs[1][0] * -0.10, 1).await
                        }
                        // `TAP=refine` / `TAP=verify` gate on tap-analysis
                        // state this port has not shipped; while it is unset
                        // — as under upstream's file-output runs, where the
                        // analysis never runs — the upstream refusals stand.
                        "refine" => Err(CommandError::new("Must complete valid 'tap' probe first")),
                        "verify" => {
                            Err(CommandError::new("Must complete valid 'refine' step first"))
                        }
                        _ => Err(CommandError::new("Please provide a valid TAP parameter")),
                    }
                })
            })
        };
        gcode
            // `TAP` picks the step (`TAP=guess` / `refine` / `verify`); the
            // threshold and sample count it needs are computed, not read.
            .register_command_with_params(
                "PROBE_EDDY_CURRENT_TAP_CALIBRATE",
                handler,
                Some("Calibrate tap_threshold for 'tap' probing"),
                &["TAP"],
                false,
            )
            .map_err(ConfigError::new)?;
        Ok(())
    }

    /// One `_try_tap` round: a `METHOD=tap` session through the `probe` object
    /// the section registered (upstream re-enters through `lookup_object`).
    async fn try_tap(
        printer: &Arc<Printer>,
        gcmd: &GcodeCommand,
        tap_threshold: f64,
        samples: i64,
    ) -> Result<(), CommandError> {
        let session = lookup_probe_session(printer)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let threshold = format!("{tap_threshold:.3}");
        let mut fo_params = gcmd.get_command_parameters().clone();
        fo_params.insert("METHOD".to_string(), "tap".to_string());
        fo_params.insert("TAP_THRESHOLD".to_string(), threshold.clone());
        fo_params.insert("SAMPLES".to_string(), samples.to_string());
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let fo_gcmd = gcode.create_gcode_command("", "", fo_params);
        gcmd.respond_info(&format!(
            "Tap probing with TAP_THRESHOLD={threshold} SAMPLES={samples}"
        ));
        session.start_probe_session(&fo_gcmd)?;
        session.run_probe(&fo_gcmd).await?;
        let positions = session.pull_probed_results();
        session.end_probe_session()?;
        let z = positions.first().map(Coord::z).ok_or_else(state_error)?;
        gcmd.respond_info(&format!("Tap probing reports z={z:.6}"));
        Ok(())
    }
}

// ===========================================================================
// PROBE_EDDY_CURRENT_CALIBRATE + Z_OFFSET_APPLY_PROBE
// ===========================================================================

/// `PROBE_EDDY_CURRENT_CALIBRATE` and `Z_OFFSET_APPLY_PROBE`
/// (`probe_eddy_current.EddyCalibrationTool`): the interactive calibration
/// that walks the probe down in 40 µm steps, correlates each step's
/// frequency to its kinematic Z, filters noisy or non-monotone points, and
/// writes the result back as `calibrate = z:freq,…`.
struct EddyCalibrationTool {
    /// The machine, for the toolhead / gcode / configfile / probe / manual_probe.
    printer: Weak<Printer>,
    /// The full section name (e.g. `"probe_eddy_current my_eddy"`).
    name: String,
    /// The frequency→height table the calibration writes into.
    calibration: Arc<EddyCalibration>,
    /// The probe speed `cmd_EDDY_CALIBRATE` read, carried to `post_manual_probe`.
    probe_speed: Mutex<f64>,
}

/// One collected batch message from the sensor (`handle_batch`'s `msg`).
#[derive(Clone)]
struct BatchMessage {
    /// `(query_time, freq, old_z)` rows.
    data: Vec<(f64, f64, f64)>,
}

/// A position's computed frequency statistics (`calc_freqs`'s value).
struct FreqStats {
    freq_avg: f64,
    mad: f64,
    count: usize,
}

/// A row that passed `validate_calibration_data`'s filter.
struct FilteredRow {
    pos: f64,
    freq_avg: f64,
    mad_hz: f64,
    mad_mm: f64,
}

impl EddyCalibrationTool {
    /// Register the two commands (`__init__`).
    ///
    /// # Errors
    /// A g-code registration clash.
    fn register(
        printer: &Arc<Printer>,
        name: String,
        calibration: Arc<EddyCalibration>,
    ) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        // `cname = self.name.split()[-1]` — the sub-name after the section
        // prefix (e.g. `"my_eddy"` from `"probe_eddy_current my_eddy"`).
        let cname = name
            .split_whitespace()
            .next_back()
            .unwrap_or(&name)
            .to_string();

        let tool = Arc::new(Self {
            printer: Arc::downgrade(printer),
            name: name.clone(),
            calibration: Arc::clone(&calibration),
            probe_speed: Mutex::new(0.0),
        });

        // PROBE_EDDY_CURRENT_CALIBRATE (mux, CHIP=<cname>)
        let calib_tool = Arc::clone(&tool);
        let calib_handler: CommandHandler = Arc::new(move |gcmd| {
            let tool = Arc::clone(&calib_tool);
            Box::pin(async move { tool.cmd_eddy_calibrate(gcmd).await })
        });
        gcode
            .register_mux_command_with_params(
                "PROBE_EDDY_CURRENT_CALIBRATE",
                "CHIP",
                Some(&cname),
                calib_handler,
                Some("Calibrate eddy current probe"),
                &["PROBE_SPEED"],
            )
            .map_err(ConfigError::new)?;

        // Z_OFFSET_APPLY_PROBE (plain command)
        let zoffset_tool = Arc::clone(&tool);
        let zoffset_handler: CommandHandler = Arc::new(move |gcmd| {
            let tool = Arc::clone(&zoffset_tool);
            Box::pin(async move { tool.cmd_z_offset_apply_probe(gcmd).await })
        });
        gcode
            .register_command_with_params(
                "Z_OFFSET_APPLY_PROBE",
                zoffset_handler,
                Some("Adjust the probe's z_offset"),
                &["METHOD"],
                false,
            )
            .map_err(ConfigError::new)?;

        Ok(())
    }

    /// The printer, or an error if it has been torn down.
    fn printer(&self) -> Result<Arc<Printer>, CommandError> {
        self.printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// `cmd_EDDY_CALIBRATE`: read `PROBE_SPEED` and start the manual probe
    /// helper (`probe_eddy_current.py:311-316`).
    async fn cmd_eddy_calibrate(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let printer = self.printer()?;
        // `gcmd.get_float("PROBE_SPEED", 5., above=0.)`
        let probe_speed = gcmd.get(
            "PROBE_SPEED",
            Some(5.0),
            |s| s.parse().ok(),
            None,
            None,
            Some(0.0),
            None,
        )?;
        *self.probe_speed.lock().unwrap_or_else(|p| p.into_inner()) = probe_speed;

        let manual_probe = printer
            .lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT)
            .ok_or_else(|| CommandError::new("manual_probe is not available"))?;

        // The finalize callback (`post_manual_probe`).
        let cb_printer = Arc::downgrade(&printer);
        let cb_name = self.name.clone();
        let cb_calibration = Arc::clone(&self.calibration);
        let cb_probe_speed = {
            // Share the probe_speed cell with the callback.
            // We can't move the Mutex out of self, so we read the value and
            // pass a fresh cell — the callback runs after this handler
            // returns, and nothing else reads probe_speed in between.
            Arc::new(Mutex::new(probe_speed))
        };
        let callback: FinalizeCallback = Arc::new(move |mpresult: Option<Coord>| {
            let Some(pos) = mpresult else { return };
            let Some(printer) = cb_printer.upgrade() else {
                return;
            };
            let name = cb_name.clone();
            let calibration = Arc::clone(&cb_calibration);
            let probe_speed = *cb_probe_speed.lock().unwrap_or_else(|p| p.into_inner());
            if let Err(err) = EddyCalibrationTool::post_manual_probe(
                &printer,
                &name,
                &calibration,
                probe_speed,
                pos,
            ) {
                if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                    gcode.respond_info(&format!("Eddy calibration failed: {err}"), true);
                }
            }
        });
        manual_probe.start_helper(&printer, gcmd, callback)?;
        Ok(())
    }

    /// `post_manual_probe`: the manual-probe result drives the calibration
    /// moves and saves the table (`probe_eddy_current.py:268-296`).
    ///
    /// Sync because `FinalizeCallback` is `Fn` (not async): all toolhead
    /// operations use the sync `move_to` / `dwell` / `position` surface.
    fn post_manual_probe(
        printer: &Arc<Printer>,
        name: &str,
        calibration: &Arc<EddyCalibration>,
        probe_speed: f64,
        mpresult: Coord,
    ) -> Result<(), CommandError> {
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let probe_calibrate_z = mpresult.z();
        // Move away from the bed (up 5 mm).
        let mut curpos = mpresult;
        curpos.set_axis(Z_AXIS, curpos.z() + 5.0);
        toolhead.move_to(curpos, probe_speed)?;
        // Move sensor over nozzle position (reverse probe offsets).
        let session = lookup_probe_session(printer)
            .ok_or_else(|| CommandError::new("Probe is not available"))?;
        let offsets = session.offsets();
        curpos.set_axis(0, curpos.x() - offsets.x);
        curpos.set_axis(1, curpos.y() - offsets.y);
        toolhead.move_to(curpos, probe_speed)?;
        // Descend back to bed.
        curpos.set_axis(Z_AXIS, curpos.z() - 5.0 - 0.050);
        toolhead.move_to(curpos, probe_speed)?;
        // Perform calibration movement and capture.
        let cal = Self::do_calibration_moves(printer, name, calibration, &toolhead, probe_speed)?;
        // Calculate each sample position average and variance.
        let mut positions = Self::calc_freqs(&cal);
        // Fix Z position offset.
        for (k, _) in &mut positions {
            *k -= probe_calibrate_z;
        }
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        let filtered = Self::validate_calibration_data(&gcode, &mut positions);
        if filtered.len() <= 8 {
            return Err(CommandError::new("Failed calibration - No usable data"));
        }
        let z_freq_pairs: Vec<(f64, f64)> = filtered.iter().map(|r| (r.pos, r.freq_avg)).collect();
        Self::save_calibration(printer, name, &z_freq_pairs);
        Ok(())
    }

    /// `do_calibration_moves`: walk the probe down in 40 µm steps, collecting
    /// sensor batches and correlating each to its kinematic Z
    /// (`probe_eddy_current.py:137-191`).
    ///
    /// Sync: uses `move_to` / `dwell` / `position` (all sync on
    /// `ToolHeadObject`). Upstream's `flush_step_generation` +
    /// `kin.calc_position` is replaced by `toolhead.position()` — the
    /// commanded position, which is the kinematic position for cartesian
    /// kinematics. Upstream's `wait_moves` is not exposed on
    /// `ToolHeadObject`; in file-output mode (no real hardware) it is a
    /// no-op, and the moves are already queued via `move_to`.
    fn do_calibration_moves(
        printer: &Arc<Printer>,
        name: &str,
        calibration: &Arc<EddyCalibration>,
        toolhead: &Arc<ToolHeadObject>,
        move_speed: f64,
    ) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
        // Start data collection.
        let msgs: Arc<Mutex<Vec<BatchMessage>>> = Arc::new(Mutex::new(Vec::new()));
        let is_finished = Arc::new(AtomicBool::new(false));

        let client_msgs = Arc::clone(&msgs);
        let client_finished = Arc::clone(&is_finished);
        let client_fn = move |msg: &Value| -> bool {
            if client_finished.load(Ordering::SeqCst) {
                return false;
            }
            let data = msg
                .get("data")
                .and_then(|d| d.as_array())
                .unwrap_or(&Vec::new())
                .iter()
                .filter_map(|row| row.as_array())
                .filter_map(|r| {
                    let t = r.get(0).and_then(|v| v.as_f64())?;
                    let f = r.get(1).and_then(|v| v.as_f64())?;
                    let z = r.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0);
                    Some((t, f, z))
                })
                .collect::<Vec<_>>();
            client_msgs
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(BatchMessage { data });
            true
        };

        // `self.printer.lookup_object(self.name).add_client(handle_batch)`.
        let probe = printer
            .lookup_object_as::<PrinterEddyProbe>(PROBE_OBJECT)
            .or_else(|| printer.lookup_object_as::<PrinterEddyProbe>(name))
            .ok_or_else(|| CommandError::new("probe_eddy_current is not available"))?;
        probe.add_client(client_fn);

        toolhead.dwell(1.0);
        calibration.note_z_calibration_start();

        // Move to each 40 µm position.
        let max_z = 4.0;
        let samp_dist = 0.040;
        let start_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let mut times: Vec<(f64, f64, f64)> = Vec::new();
        let n_steps = (max_z / samp_dist) as i64;
        for i in 0..=n_steps {
            let zpos = i as f64 * samp_dist;
            // Move to next position (always descending to reduce backlash).
            let mut hop_pos = start_pos;
            hop_pos.set_axis(Z_AXIS, start_pos.z() + zpos + 0.500);
            toolhead.move_to(hop_pos, move_speed)?;
            let mut next_pos = start_pos;
            next_pos.set_axis(Z_AXIS, start_pos.z() + zpos);
            toolhead.move_to(next_pos, move_speed)?;
            // Note sample timing.
            let start_query_time = toolhead.get_last_move_time() + 0.050;
            let end_query_time = start_query_time + 0.100;
            toolhead.dwell(0.200);
            // Find Z position based on actual commanded position.
            let kin_z = toolhead
                .position()
                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
            times.push((start_query_time, end_query_time, kin_z.z()));
        }
        toolhead.dwell(1.0);
        // `wait_moves` is not exposed on `ToolHeadObject`; moves are queued
        // via `move_to` and the dwell above provides the timing buffer.
        calibration.note_z_calibration_finish();

        // Finish data collection.
        is_finished.store(true, Ordering::SeqCst);

        // Correlate query responses.
        let msgs = msgs.lock().unwrap_or_else(|p| p.into_inner()).clone();
        let mut cal: Vec<(f64, Vec<f64>)> = Vec::new();
        let mut step = 0usize;
        for msg in &msgs {
            for &(query_time, freq, _old_z) in &msg.data {
                while step < times.len() && query_time > times[step].1 {
                    step += 1;
                }
                if step < times.len() && query_time >= times[step].0 {
                    let kin_z = times[step].2;
                    match cal
                        .iter_mut()
                        .find(|(k, _)| (*k - kin_z).abs() < f64::EPSILON)
                    {
                        Some((_, freqs)) => freqs.push(freq),
                        None => cal.push((kin_z, vec![freq])),
                    }
                }
            }
        }
        if cal.len() != times.len() {
            return Err(CommandError::new(
                "Failed calibration - incomplete sensor data",
            ));
        }
        Ok(cal)
    }

    /// `_median`: the median of a slice (sorted copy, odd/even handling)
    /// (`probe_eddy_current.py:193-197`).
    fn median(values: &[f64]) -> f64 {
        let mut sorted = values.to_vec();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let n = sorted.len();
        if n % 2 == 0 {
            (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
        } else {
            sorted[n / 2]
        }
    }

    /// `calc_freqs`: per-position frequency average, MAD (median absolute
    /// deviation), and sample count (`probe_eddy_current.py:199-205`).
    fn calc_freqs(meas: &[(f64, Vec<f64>)]) -> Vec<(f64, FreqStats)> {
        let mut positions = Vec::new();
        for (pos, freqs) in meas {
            let count = freqs.len();
            let freq_avg = freqs.iter().sum::<f64>() / count as f64;
            let mads: Vec<f64> = freqs.iter().map(|f| (f - freq_avg).abs()).collect();
            let mad = Self::median(&mads);
            positions.push((
                *pos,
                FreqStats {
                    freq_avg,
                    mad,
                    count,
                },
            ));
        }
        positions
    }

    /// `validate_calibration_data`: filter non-monotone or noisy points,
    /// report noise stats, and return the surviving rows
    /// (`probe_eddy_current.py:207-243`).
    fn validate_calibration_data(
        gcode: &GCodeDispatch,
        positions: &mut [(f64, FreqStats)],
    ) -> Vec<FilteredRow> {
        // Sort by position (upstream's `sorted(positions.items())`).
        positions.sort_by(|a, b| a.0.total_cmp(&b.0));

        let mut last_freq: f64 = 40_000_000.0;
        let mut last_pos: f64 = 0.0;
        let mut last_mad: f64 = 0.0;
        let mut filtered: Vec<FilteredRow> = Vec::new();
        let mut mad_hz_total = 0.0;
        let mut mad_mm_total = 0.0;
        let mut samples_count = 0usize;

        for (pos, stats) in positions.iter() {
            let pos = *pos;
            let freq_avg = stats.freq_avg;
            let mad_hz = stats.mad;
            let count = stats.count;

            if freq_avg > last_freq {
                gcode.respond_info(
                    &format!("Frequency stops decreasing at step {pos:.3}"),
                    true,
                );
                break;
            }
            let diff_mad = (last_mad.powi(2) + mad_hz.powi(2)).sqrt();
            let freq_diff = last_freq - freq_avg;
            last_freq = freq_avg;
            if freq_diff < 2.5 * diff_mad {
                gcode.respond_info(
                    &format!("Frequency too noisy at step {last_pos:.3} -> {pos:.3}"),
                    true,
                );
                gcode.respond_info(
                    &format!(
                        "Frequency diff: {freq_diff:.3}, MAD_Hz: {last_mad:.3} -> MAD_Hz: {mad_hz:.3}"
                    ),
                    true,
                );
                break;
            }
            last_mad = mad_hz;
            let delta_dist = pos - last_pos;
            last_pos = pos;
            let mad_mm = mad_hz * delta_dist / freq_diff;
            filtered.push(FilteredRow {
                pos,
                freq_avg,
                mad_hz,
                mad_mm,
            });
            mad_hz_total += mad_hz;
            mad_mm_total += mad_mm;
            samples_count += count;
        }

        if filtered.is_empty() {
            return filtered;
        }

        let avg_mad = mad_hz_total / filtered.len() as f64;
        let avg_mad_mm = mad_mm_total / filtered.len() as f64;
        gcode.respond_info(
            &format!(
                "probe_eddy_current: noise {avg_mad_mm:.6}mm, MAD_Hz={avg_mad:.3} in {samples_count} queries\n"
            ),
            true,
        );
        let freq_list: Vec<f64> = filtered.iter().map(|r| r.freq_avg).collect();
        let freq_diff = freq_list.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
            - freq_list.iter().cloned().fold(f64::INFINITY, f64::min);
        gcode.respond_info(&format!("Total frequency range: {freq_diff:.3} Hz\n"), true);
        let mut points = vec![0.25, 0.5, 1.0, 2.0, 3.0];
        for row in &filtered {
            if !points.is_empty() && points[0] <= row.pos {
                points.remove(0);
                gcode.respond_info(
                    &format!(
                        "z: {:.3} # noise {:.6}mm, MAD_Hz={:.3}\n",
                        row.pos, row.mad_mm, row.mad_hz
                    ),
                    true,
                );
            }
        }
        filtered
    }

    /// `_save_calibration`: format the pairs and write them to `configfile`
    /// (`probe_eddy_current.py:117-131`).
    fn save_calibration(printer: &Arc<Printer>, name: &str, z_freq_pairs: &[(f64, f64)]) {
        let gcode = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT);
        if let Some(gcode) = gcode {
            gcode.respond_info(
                "The SAVE_CONFIG command will update the printer config file\n\
                 and restart the printer.",
                true,
            );
        }
        // Save results: `%.6f:%.3f` per pair, comma-separated, newline every 3.
        let mut cal_contents = String::new();
        for (i, (pos, freq)) in z_freq_pairs.iter().enumerate() {
            if i % 3 == 0 {
                cal_contents.push('\n');
            }
            cal_contents.push_str(&format!("{pos:.6}:{freq:.3}"));
            cal_contents.push(',');
        }
        // Remove trailing comma.
        if cal_contents.ends_with(',') {
            cal_contents.pop();
        }
        if let Some(configfile) = printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT) {
            configfile.set(name, "calibrate", &cal_contents);
        }
    }

    /// `_save_tap_z_offset`: read current `tap_z_offset` from configfile
    /// settings, subtract the homing Z, and write back
    /// (`probe_eddy_current.py:318-330`).
    fn save_tap_z_offset(printer: &Arc<Printer>, name: &str, gcmd: &GcodeCommand, homing_z: f64) {
        let tap_z_offset = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .and_then(|configfile| {
                let status = configfile.get_status(0.0);
                status
                    .get("settings")
                    .and_then(|s| s.get(name))
                    .and_then(|s| s.get("tap_z_offset"))
                    .and_then(|v| v.as_f64())
            })
            .unwrap_or(0.0);
        let new_calibrate = tap_z_offset - homing_z;
        gcmd.respond_info(&format!(
            "{name}: tap_z_offset: {new_calibrate:.3}\n\
             The SAVE_CONFIG command will update the printer config file\n\
             with the above and restart the printer."
        ));
        if let Some(configfile) = printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT) {
            configfile.set(name, "tap_z_offset", &format!("{new_calibrate:.3}"));
        }
    }

    /// `cmd_Z_OFFSET_APPLY_PROBE`: read the gcode_move homing origin Z,
    /// then either save the tap z offset (METHOD=tap) or shift the
    /// calibration table and save it (`probe_eddy_current.py:336-354`).
    async fn cmd_z_offset_apply_probe(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let printer = self.printer()?;
        let gcode_move = printer
            .lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT)
            .ok_or_else(|| CommandError::new("gcode_move is not available"))?;
        let offset = gcode_move
            .status()
            .get("homing_origin")
            .and_then(|h| h.as_array())
            .and_then(|a| a.get(Z_AXIS))
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0);
        if offset == 0.0 {
            gcmd.respond_info("Nothing to do: Z Offset is 0");
            return Ok(());
        }
        let method = gcmd.get_str_default("METHOD", "").to_lowercase();
        if method == "tap" {
            Self::save_tap_z_offset(&printer, &self.name, gcmd, offset);
            return Ok(());
        }
        let (cal_freqs, cal_zpos) = self.calibration.get_calibration();
        let mut z_freq_pairs: Vec<(f64, f64)> = cal_zpos
            .iter()
            .zip(cal_freqs.iter())
            .map(|(z, f)| (z - offset, *f))
            .collect();
        z_freq_pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        Self::save_calibration(&printer, &self.name, &z_freq_pairs);
        Ok(())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Which way a [`RecordingDrift`] call came in.
    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Direction {
        Adjust,
        Unadjust,
    }

    /// A drift helper that shifts every frequency by `add` and writes down
    /// each call — the registered stand-in for `temperature_probe`'s
    /// `EddyDriftCompensation`.
    struct RecordingDrift {
        temperature: f64,
        add: f64,
        calls: Mutex<Vec<(Direction, f64, Option<f64>)>>,
    }

    impl RecordingDrift {
        fn new(temperature: f64, add: f64) -> Self {
            Self {
                temperature,
                add,
                calls: Mutex::new(Vec::new()),
            }
        }

        /// Every call recorded, in order.
        fn calls(&self) -> Vec<(Direction, f64, Option<f64>)> {
            self.calls.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    impl DriftCompensation for RecordingDrift {
        fn get_temperature(&self) -> f64 {
            self.temperature
        }

        fn adjust_freq(&self, freq: f64, origin_temp: Option<f64>) -> f64 {
            self.calls.lock().unwrap_or_else(|p| p.into_inner()).push((
                Direction::Adjust,
                freq,
                origin_temp,
            ));
            freq + self.add
        }

        fn unadjust_freq(&self, freq: f64, dest_temp: Option<f64>) -> f64 {
            self.calls.lock().unwrap_or_else(|p| p.into_inner()).push((
                Direction::Unadjust,
                freq,
                dest_temp,
            ));
            freq + self.add
        }
    }

    /// The table the tests convert through: frequency 1000/800/600 at Z 0/1/2
    /// (sorted ascending by frequency, as `read` stores it).
    fn table() -> EddyCalibration {
        EddyCalibration::from_pairs(&[(1000., 0.), (800., 1.), (600., 2.)])
    }

    /// With nothing registered the conversions are the plain table — upstream's
    /// `DummyDriftCompensation` passes frequencies through and reports `0.`
    /// temperature, so the result is byte-for-byte the pre-drift behaviour.
    #[test]
    fn without_a_registered_helper_the_table_walks_alone() {
        let calibration = table();
        // Between 600 (Z 2) and 800 (Z 1): -0.005·f + 5 = 1.5.
        assert_eq!(calibration.freq_to_height(700.), 1.5);
        // The other direction: Z 1 sits at exactly 800.
        assert_eq!(calibration.height_to_freq(1.0).expect("in range"), 800.);
        // `apply_calibration` fills the z column and keeps the frequency.
        let mut rows = [[0., 700., 99.]];
        calibration.apply_calibration(&mut rows);
        assert_eq!(rows[0], [0., 700., 1.5]);
        // The dummy's temperature: the drift path is a pass-through, so a
        // helper that never gets asked shows no calls at all.
        assert_eq!(calibration.drift_temperature(), 0.);
    }

    /// `apply_calibration` runs each frequency through `adjust_freq` at the
    /// helper's *current* temperature — read once per call — before the table,
    /// and leaves the row's own frequency untouched
    /// (`probe_eddy_current.py:61-79`).
    #[test]
    fn apply_calibration_adjusts_through_the_registered_helper() {
        let calibration = table();
        let drift = Arc::new(RecordingDrift::new(42., 100.));
        calibration.register_drift_compensation(Arc::clone(&drift) as Arc<dyn DriftCompensation>);

        let mut rows = [[0., 700., 99.]];
        calibration.apply_calibration(&mut rows);
        // 700 + 100 = 800, which the table maps to Z 1.0 (unadjusted: 1.5).
        assert_eq!(rows[0][2], 1.0);
        // The row keeps the frequency the sensor measured.
        assert_eq!(rows[0][1], 700.);
        // The helper was asked at the temperature it reported — read once for
        // the whole call, as upstream's `cur_temp` is.
        assert_eq!(drift.calls(), vec![(Direction::Adjust, 700., Some(42.))]);
    }

    /// `height_to_freq` hands its table result to `unadjust_freq` with **no**
    /// destination temperature — the current one, upstream's `:99`.
    #[test]
    fn height_to_freq_unadjusts_through_the_registered_helper() {
        let calibration = table();
        let drift = Arc::new(RecordingDrift::new(42., 100.));
        calibration.register_drift_compensation(Arc::clone(&drift) as Arc<dyn DriftCompensation>);

        // Z 1 → table 800, then +100.
        assert_eq!(calibration.height_to_freq(1.0).expect("in range"), 900.);
        assert_eq!(drift.calls(), vec![(Direction::Unadjust, 800., None)]);
        // And the helper's temperature is what `adjust` sees through
        // `freq_to_height`.
        assert_eq!(calibration.freq_to_height(700.), 1.0);
        assert_eq!(drift.calls()[1], (Direction::Adjust, 700., Some(42.)));
    }

    /// A second registration replaces the first, as upstream's assignment
    /// does (`probe_eddy_current.py:54`).
    #[test]
    fn registering_a_helper_replaces_the_previous_one() {
        let calibration = table();
        calibration.register_drift_compensation(Arc::new(RecordingDrift::new(42., 0.)));
        let drift = Arc::new(RecordingDrift::new(42., 100.));
        calibration.register_drift_compensation(Arc::clone(&drift) as Arc<dyn DriftCompensation>);

        assert_eq!(calibration.freq_to_height(700.), 1.0);
        assert_eq!(drift.calls().len(), 1);
    }

    /// Outside the table the sentinel heights still apply after adjustment —
    /// the helper moves the frequency, the table's range check stays the table's.
    #[test]
    fn adjustment_does_not_move_the_out_of_range_guards() {
        let calibration = table();
        calibration.register_drift_compensation(Arc::new(RecordingDrift::new(42., 100.)));
        // 1200 + 100 = 1300: past the top of the table → -OUT_OF_RANGE.
        assert_eq!(calibration.freq_to_height(1200.), -OUT_OF_RANGE);
        // 100 + 100 = 200: below the bottom → +OUT_OF_RANGE.
        assert_eq!(calibration.freq_to_height(100.), OUT_OF_RANGE);
    }

    // ----- EddyCalibrationTool tests -----

    use crate::core::klippy::reactor::ManualReactor;

    /// `_median` with an odd-length slice returns the middle element
    /// (`probe_eddy_current.py:193-197`).
    #[test]
    fn median_odd_length() {
        assert_eq!(EddyCalibrationTool::median(&[3.0, 1.0, 2.0]), 2.0);
        assert_eq!(EddyCalibrationTool::median(&[5.0]), 5.0);
        assert_eq!(
            EddyCalibrationTool::median(&[10.0, 20.0, 30.0, 40.0, 50.0]),
            30.0
        );
    }

    /// `_median` with an even-length slice returns the average of the two
    /// middle elements (`probe_eddy_current.py:193-197`).
    #[test]
    fn median_even_length() {
        assert_eq!(EddyCalibrationTool::median(&[4.0, 1.0, 3.0, 2.0]), 2.5);
        assert_eq!(EddyCalibrationTool::median(&[1.0, 2.0]), 1.5);
        assert_eq!(EddyCalibrationTool::median(&[10.0, 20.0, 30.0, 40.0]), 25.0);
    }

    /// `validate_calibration_data` keeps monotonically decreasing, low-noise
    /// points and reports them all (`probe_eddy_current.py:207-243`).
    #[test]
    fn validate_keeps_clean_monotone_data() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let gcode = GCodeDispatch::new(Arc::clone(&printer));
        // 9 positions, frequencies decreasing by 1000 Hz each step, tiny MAD.
        let mut positions: Vec<(f64, FreqStats)> = (0..9)
            .map(|i| {
                let pos = i as f64 * 0.5;
                let freq = 10_000_000.0 - i as f64 * 1000.0;
                (
                    pos,
                    FreqStats {
                        freq_avg: freq,
                        mad: 1.0,
                        count: 10,
                    },
                )
            })
            .collect();
        let filtered = EddyCalibrationTool::validate_calibration_data(&gcode, &mut positions);
        assert_eq!(filtered.len(), 9);
        // All positions preserved in order.
        for (i, row) in filtered.iter().enumerate() {
            assert!((row.pos - i as f64 * 0.5).abs() < 1e-9);
        }
    }

    /// `validate_calibration_data` stops at the first position where the
    /// frequency stops decreasing (freq_avg > last_freq)
    /// (`probe_eddy_current.py:217-219`).
    #[test]
    fn validate_stops_at_non_monotone_frequency() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let gcode = GCodeDispatch::new(Arc::clone(&printer));
        // Position 3 has a higher frequency than position 2 → stops there.
        let mut positions = vec![
            (
                0.0,
                FreqStats {
                    freq_avg: 10_000_000.0,
                    mad: 1.0,
                    count: 10,
                },
            ),
            (
                0.5,
                FreqStats {
                    freq_avg: 9_999_000.0,
                    mad: 1.0,
                    count: 10,
                },
            ),
            (
                1.0,
                FreqStats {
                    freq_avg: 9_998_000.0,
                    mad: 1.0,
                    count: 10,
                },
            ),
            (
                1.5,
                FreqStats {
                    freq_avg: 9_999_500.0,
                    mad: 1.0,
                    count: 10,
                },
            ), // increases!
            (
                2.0,
                FreqStats {
                    freq_avg: 9_996_000.0,
                    mad: 1.0,
                    count: 10,
                },
            ),
        ];
        let filtered = EddyCalibrationTool::validate_calibration_data(&gcode, &mut positions);
        // Only the first 3 pass (the 4th triggers the break before it's added).
        assert_eq!(filtered.len(), 3);
        assert!((filtered[0].pos - 0.0).abs() < 1e-9);
        assert!((filtered[2].pos - 1.0).abs() < 1e-9);
    }

    /// `validate_calibration_data` stops when the frequency difference is
    /// too noisy (freq_diff < 2.5 * diff_mad)
    /// (`probe_eddy_current.py:226-236`).
    #[test]
    fn validate_stops_at_noisy_data() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let gcode = GCodeDispatch::new(Arc::clone(&printer));
        // Large MAD on the 3rd position makes freq_diff < 2.5 * diff_mad.
        let mut positions = vec![
            (
                0.0,
                FreqStats {
                    freq_avg: 10_000_000.0,
                    mad: 1.0,
                    count: 10,
                },
            ),
            (
                0.5,
                FreqStats {
                    freq_avg: 9_990_000.0,
                    mad: 1.0,
                    count: 10,
                },
            ),
            // freq_diff = 9900000 - 9000000 = 900000, diff_mad = sqrt(1+1000^2) ≈ 1000
            // 2.5 * 1000 = 2500, 900000 > 2500 so this passes.
            (
                1.0,
                FreqStats {
                    freq_avg: 9_000_000.0,
                    mad: 1000.0,
                    count: 10,
                },
            ),
            // freq_diff = 9000000 - 8999990 = 10, diff_mad = sqrt(1000^2+1000^2) ≈ 1414
            // 2.5 * 1414 = 3535, 10 < 3535 → too noisy, breaks.
            (
                1.5,
                FreqStats {
                    freq_avg: 8_999_990.0,
                    mad: 1000.0,
                    count: 10,
                },
            ),
        ];
        let filtered = EddyCalibrationTool::validate_calibration_data(&gcode, &mut positions);
        // First 3 pass, 4th is too noisy.
        assert_eq!(filtered.len(), 3);
    }
}
