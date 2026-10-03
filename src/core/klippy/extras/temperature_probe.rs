//! `[temperature_probe <name>]` — the probe's own temperature sensor.
//!
//! Upstream's `temperature_probe.py` is two halves in one file: a smoothed
//! temperature sensor that joins the `heaters` registry, and the interactive
//! thermal-drift calibration of an eddy probe that is driven *from* that
//! sensor. This module is that file — the section reads its options, `heaters`
//! builds and delivers readings, `_temp_callback`'s smoothing lands in
//! `get_status`, the `TEMPERATURE_PROBE_*` family runs the calibration
//! state machine, and `EddyDriftCompensation` turns each run's samples into
//! the drift correction the eddy probe applies.
//!
//! | here | upstream |
//! |---|---|
//! | [`Polynomial2d`] | `Polynomial2d` (`temperature_probe.py:16-57`) |
//! | [`TemperatureProbeOptions`] | `TemperatureProbe.__init__`'s option reads (`:63-97`) |
//! | [`TemperatureProbe`] | `TemperatureProbe.__init__` (`:60-111`) |
//! | [`sensor_callback`] | `_temp_callback` (`:140-153`) |
//! | [`check_kick_next`] | `_check_kick_next` (`:155-159`) |
//! | [`TemperatureProbe::register_commands`] and the flow behind it | the `TEMPERATURE_PROBE_*` family: registration (`:112-123`), flow (`:164-337`), command bodies (`:338-448`) |
//! | [`TemperatureProbe::get_status`] | `get_status` (`:453-465`) |
//! | [`EddyDriftCompensation`] | the drift helper (`:479-714`) and its `probe_eddy_current` registration (`:125-137`) |
//!
//! The family drives a calibration state machine over the same section's
//! options: `in_calibration` in the shared state is what [`check_kick_next`]
//! needs before it can run `TEMPERATURE_PROBE_NEXT`, so that script is live as
//! soon as a calibration starts.
//!
//! # Not here (deferred; the upstream line ranges are the gap list)
//!
//! * **`EddyCalibrationTool`'s two calls into the helper**
//!   (`note_z_calibration_start` / `_finish`, `temperature_probe.py:544-558`):
//!   the methods are here, their upstream caller
//!   (`EddyCalibrationTool.do_calibration_moves`, `probe_eddy_current.py:134`
//!   and `:157`) is part of the not-yet-ported calibration tool — see
//!   [`probe_eddy_current`]'s gap list.
//! * **`stats`** (`temperature_probe.py:467-468`): the port has no
//!   `Printer`-level walker that collects object `stats` yet;
//!   [`TemperatureProbe::stats`] is upstream's shape, ready for one.
//!
//! [`probe_eddy_current`]: crate::core::klippy::extras::probe_eddy_current

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::object::CONFIGFILE_OBJECT;
use crate::core::klippy::config::{ConfigError, ConfigWrapper, PrinterConfig};
use crate::core::klippy::extras::heaters;
use crate::core::klippy::extras::manual_probe::{
    self, FinalizeCallback, ManualProbe, MANUAL_PROBE_OBJECT,
};
use crate::core::klippy::extras::probe::{lookup_probe_session, ProbeSession};
use crate::core::klippy::extras::probe_eddy_current::{DriftCompensation, PrinterEddyProbe};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{solve_linear_equations, Coord};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("temperature_probe", order = 30, prefix = load_config_prefix);

/// Upstream's `KELVIN_TO_CELSIUS` (`temperature_probe.py:10`): the `min_temp`
/// default *and* its `minval`.
const KELVIN_TO_CELSIUS: f64 = -273.15;

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The probe object, which `probe.py` registers under this name and the
/// calibration looks every probe up by.
const PROBE_OBJECT: &str = "probe";

/// The Z axis index, as [`Coord`] numbers them.
const Z_AXIS: usize = 2;
/// The X axis index, as [`Coord`] numbers them.
const X_AXIS: usize = 0;
/// The Y axis index, as [`Coord`] numbers them.
const Y_AXIS: usize = 1;

/// The sample windows one drift sweep takes (`DRIFT_SAMPLE_COUNT`), each a
/// half-millimetre of Z apart (`temperature_probe.py:476`).
const DRIFT_SAMPLE_COUNT: usize = 9;

// ===========================================================================
// Polynomial2d
// ===========================================================================

/// The second-order polynomial a drift calibration is stored as
/// (`temperature_probe.py:16-57`): `y(x) = c·x² + b·x + a`.
#[derive(Clone, Copy, PartialEq)]
pub struct Polynomial2d {
    /// The constant term (`self.a`).
    pub a: f64,
    /// The linear coefficient (`self.b`).
    pub b: f64,
    /// The quadratic coefficient (`self.c`).
    pub c: f64,
}

impl Polynomial2d {
    /// Build the curve (`Polynomial2d.__init__`).
    pub fn new(a: f64, b: f64, c: f64) -> Self {
        Self { a, b, c }
    }

    /// Evaluate at `xval` (`Polynomial2d.__call__`): `c·x² + b·x + a`.
    pub fn eval(&self, x: f64) -> f64 {
        self.c * x * x + self.b * x + self.a
    }

    /// The coefficients upstream hands back (`Polynomial2d.get_coefs`).
    pub fn get_coefs(&self) -> (f64, f64, f64) {
        (self.a, self.b, self.c)
    }

    /// Best fit of `a + b·x + c·x² = y` over `coords`
    /// (`Polynomial2d.fit`, `:49-57`).
    ///
    /// Upstream solves the normal equations through
    /// `mathutil.solve_linear_equations` and then indexes the result; with no
    /// points that raises, and with too few or collinear ones the solve comes
    /// back empty. Here both are `None` — the caller turns it into the command
    /// error upstream's exception becomes.
    ///
    /// # Panics
    /// Never: an empty `coords` returns `None` before the solve, whose
    /// transposes index the first row (upstream's `mat_transp` does the same
    /// and raises `IndexError`).
    pub fn fit(coords: &[(f64, f64)]) -> Option<Self> {
        if coords.is_empty() {
            return None;
        }
        let eqs: Vec<Vec<f64>> = coords.iter().map(|&(x, _)| vec![1.0, x, x * x]).collect();
        let ans: Vec<Vec<f64>> = coords.iter().map(|&(_, y)| vec![y]).collect();
        let res = solve_linear_equations(&eqs, &ans)?;
        Some(Self {
            a: res[0][0],
            b: res[1][0],
            c: res[2][0],
        })
    }
}

/// Python's `round(value, 8)` — half-to-even, which is what
/// `Polynomial2d.__repr__` compares against `int(coef)` (`:33`).
fn round8(value: f64) -> f64 {
    let scaled = value * 1e8;
    let floor = scaled.floor();
    let frac = scaled - floor;
    let up = if frac > 0.5 {
        true
    } else if frac < 0.5 {
        false
    } else {
        // Exactly halfway: land on the even neighbour, as Python does — an odd
        // `floor` moves up to the even `floor + 1`.
        (floor as i64) % 2 != 0
    };
    (if up { floor + 1.0 } else { floor }) / 1e8
}

impl std::fmt::Display for Polynomial2d {
    /// Upstream's `__str__` (`:28-29`): `"%f, %f, %f"` — six decimals each.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:.6}, {:.6}, {:.6}", self.a, self.b, self.c)
    }
}

impl std::fmt::Debug for Polynomial2d {
    /// Upstream's `__repr__` (`:31-47`), the string its log lines print:
    /// `y(x) = 2.000000x^2 + 1.000000`, skipping terms that round away.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut parts = vec!["y(x) =".to_string()];
        for (i, raw) in [self.c, self.b, self.a].into_iter().enumerate() {
            let mut coef = raw;
            if round8(coef) == coef.trunc() {
                coef = coef.trunc();
            }
            if coef.abs() < 1e-10 {
                continue;
            }
            let degree = 2 - i;
            let term = match degree {
                2 => "x^2".to_string(),
                1 => "x".to_string(),
                _ => String::new(),
            };
            if parts.len() == 1 {
                parts.push(format!("{coef:.6}{term}"));
            } else {
                let sign = if coef < 0.0 { "-" } else { "+" };
                parts.push(format!("{sign} {:.6}{term}", coef.abs()));
            }
        }
        f.write_str(&parts.join(" "))
    }
}

// ===========================================================================
// Options
// ===========================================================================

/// The `[temperature_probe <name>]` options, as `TemperatureProbe.__init__`
/// reads them (`temperature_probe.py:63-97`).
///
/// Parsed apart from the object so the reads — defaults, bounds and their
/// error wording — can be held to upstream without a printer behind them.
#[derive(Debug, Clone, PartialEq)]
pub struct TemperatureProbeOptions {
    /// `speed`: the travel speed between the probe and the nozzle, `None`
    /// when unset (`getfloat("speed", None, above=0.)`).
    pub speed: Option<f64>,
    /// `horizontal_move_z` (default `2.`, above `0.`).
    pub horizontal_move_z: f64,
    /// `resting_z` (default `.4`, above `0.`).
    pub resting_z: f64,
    /// `calibration_position`: where the nozzle heats up, `None` when unset
    /// (`getfloatlist("calibration_position", None, count=3)`).
    pub cal_pos: Option<Vec<f64>>,
    /// `calibration_bed_temp` (default `None`, above `50.`).
    pub cal_bed_temp: Option<f64>,
    /// `calibration_extruder_temp` (default `None`, above `50.`).
    pub cal_extruder_temp: Option<f64>,
    /// `extruder_heating_z` (default `50.`, above `0.`).
    pub cal_extruder_z: f64,
    /// `1 / smooth_time`: the smoothing rate `_temp_callback` applies
    /// (`smooth_time` itself is read once and not kept upstream either).
    pub inv_smooth_time: f64,
    /// `min_temp` (default and `minval` `KELVIN_TO_CELSIUS`).
    pub min_temp: f64,
    /// `max_temp` (default `99999999.9`, above `min_temp`).
    pub max_temp: f64,
}

impl TemperatureProbeOptions {
    /// Read the section (`TemperatureProbe.__init__`, `:63-97`), in upstream's
    /// order.
    ///
    /// # Errors
    /// Upstream's bound and count wording for a value that fails it
    /// (`klippy/configfile.py:48-59`, `:98-101`).
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let speed = optional_above(config, "speed", 0.)?;
        let horizontal_move_z =
            config.get_float_bounded("horizontal_move_z", Some(2.), None, None, Some(0.), None)?;
        let resting_z =
            config.get_float_bounded("resting_z", Some(0.4), None, None, Some(0.), None)?;
        let cal_pos = optional_float_list(config, "calibration_position", 3)?;
        let cal_bed_temp = optional_above(config, "calibration_bed_temp", 50.)?;
        let cal_extruder_temp = optional_above(config, "calibration_extruder_temp", 50.)?;
        let cal_extruder_z = config.get_float_bounded(
            "extruder_heating_z",
            Some(50.),
            None,
            None,
            Some(0.),
            None,
        )?;
        let smooth_time =
            config.get_float_bounded("smooth_time", Some(2.), None, None, Some(0.), None)?;
        let inv_smooth_time = 1. / smooth_time;
        let min_temp = config.get_float_bounded(
            "min_temp",
            Some(KELVIN_TO_CELSIUS),
            Some(KELVIN_TO_CELSIUS),
            None,
            None,
            None,
        )?;
        let max_temp = config.get_float_bounded(
            "max_temp",
            Some(99999999.9),
            None,
            None,
            Some(min_temp),
            None,
        )?;
        Ok(Self {
            speed,
            horizontal_move_z,
            resting_z,
            cal_pos,
            cal_bed_temp,
            cal_extruder_temp,
            cal_extruder_z,
            inv_smooth_time,
            min_temp,
            max_temp,
        })
    }
}

/// An optional float with an upstream `above` bound — `getfloat(option, None,
/// above=…)` (`temperature_probe.py:63,71,73`).
///
/// Absent stays `None` and is not recorded, as upstream's `_get_wrapper` does
/// for a `None` default (`klippy/configfile.py:32-35`); present is read through
/// the tracker and must clear the bound.
///
/// # Errors
/// `Option '<option>' in section '<id>' must be above <above>` when it does not.
fn optional_above(
    config: &ConfigWrapper,
    option: &str,
    above: f64,
) -> Result<Option<f64>, ConfigError> {
    if !config.has(option) {
        return Ok(None);
    }
    config
        .get_float_bounded(option, None, None, None, Some(above), None)
        .map(Some)
}

/// An optional `getfloatlist(option, None, count)` (`klippy/configfile.py:115`)
/// — `calibration_position` at `temperature_probe.py:69-70`.
///
/// Every item is parsed *before* the count is checked, as upstream's
/// `getlists` does (`klippy/configfile.py:98-101`); an absent option is `None`.
///
/// # Errors
/// `Unable to parse option '<option>' in section '<id>'` for a non-number, or
/// `… must have <count> elements` for the wrong count.
fn optional_float_list(
    config: &ConfigWrapper,
    option: &str,
    count: usize,
) -> Result<Option<Vec<f64>>, ConfigError> {
    let Some(text) = config.get_str(option) else {
        return Ok(None);
    };
    let identifier = config.identifier();
    // Upstream returns `[]` for a blank value and keeps empty items otherwise
    // (`klippy/configfile.py:91-101`), so only the whole-value blank collapses.
    let parts: Vec<&str> = if text.trim().is_empty() {
        Vec::new()
    } else {
        text.split(',').map(str::trim).collect()
    };
    let mut values = Vec::with_capacity(parts.len());
    for part in parts {
        values.push(part.parse::<f64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{identifier}'"
            ))
        })?);
    }
    if values.len() != count {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{identifier}' must have {count} elements"
        )));
    }
    Ok(Some(values))
}

// ===========================================================================
// Shared state
// ===========================================================================

/// The readings and calibration flags the sensor callback and the object share
/// (upstream's `last_temp_read_time` / `last_measurement` / `in_calibration` /
/// `next_auto_temp` / `target_temp` / `total_expansion` / `expected_count` /
/// `sample_count` / `step` / `last_zero_pos` / `start_pos` / `_method`,
/// `:64-67,98-110`).
///
/// Shared rather than owned by the object because `_temp_callback` is bound as
/// a plain closure while the object is still being built. The calibration
/// counters live here too so one lock covers the state the sensor callback
/// checks against the state a command writes.
#[derive(Debug)]
struct State {
    /// The clock the last reading was dated from (`last_temp_read_time`).
    last_temp_read_time: f64,
    /// Upstream's `last_measurement`: `(smoothed, measured_min, measured_max)`.
    measurement: (f64, f64, f64),
    /// Whether a drift calibration is running (`in_calibration`).
    in_calibration: bool,
    /// The smoothed temperature the next sample waits for (`next_auto_temp`).
    next_auto_temp: f64,
    /// The calibration's target temperature (`target_temp`).
    target_temp: f64,
    /// The expansion the calibration has estimated (`total_expansion`).
    total_expansion: f64,
    /// The probing method the calibration runs with (`_method`, `"manual"`).
    method: String,
    /// How many samples the calibration expects (`expected_count`).
    expected_count: i64,
    /// How many samples it has taken (`sample_count`).
    sample_count: i64,
    /// The temperature step between two samples (`step`).
    step: f64,
    /// The Z the previous sample was taken at (`last_zero_pos`).
    last_zero_pos: Option<f64>,
    /// The XY the calibration started from (`start_pos`; `[]` upstream, so
    /// `None` until `TEMPERATURE_PROBE_CALIBRATE` captures it).
    start_pos: Option<[f64; 2]>,
}

impl State {
    /// Upstream's `__init__` starting values (`:64-67,98-110`).
    fn new() -> Self {
        Self {
            last_temp_read_time: 0.,
            measurement: (0., 99999999., 0.),
            in_calibration: false,
            next_auto_temp: 99999999.,
            target_temp: 0.,
            total_expansion: 0.,
            method: "manual".to_string(),
            expected_count: 0,
            sample_count: 0,
            step: 2.,
            last_zero_pos: None,
            start_pos: None,
        }
    }

    /// Fold one reading into the smoothed measurement
    /// (`_temp_callback`, `temperature_probe.py:141-149`).
    fn apply_reading(&mut self, read_time: f64, temp: f64, inv_smooth_time: f64) {
        let time_diff = read_time - self.last_temp_read_time;
        self.last_temp_read_time = read_time;
        let adj_time = (time_diff * inv_smooth_time).min(1.);
        let (smoothed, measured_min, measured_max) = self.measurement;
        let smoothed = smoothed + (temp - smoothed) * adj_time;
        self.measurement = (
            smoothed,
            measured_min.min(smoothed),
            measured_max.max(smoothed),
        );
    }
}

/// Take the shared state, poisoned or not.
fn lock_state(state: &Mutex<State>) -> MutexGuard<'_, State> {
    state.lock().unwrap_or_else(|p| p.into_inner())
}

// ===========================================================================
// The sensor callback
// ===========================================================================

/// The callback the sensor delivers readings to — upstream's
/// `_temp_callback` (`temperature_probe.py:140-153`), bound at load.
///
/// It smooths the reading into [`State`], then — while a calibration is
/// running and the smoothed temperature has reached `next_auto_temp` — asks
/// the reactor to run upstream's `_check_kick_next` on the next dispatch
/// (`register_async_callback`, `:150-152`).
fn sensor_callback(
    state: Arc<Mutex<State>>,
    inv_smooth_time: f64,
    printer: Weak<Printer>,
) -> heaters::SensorCallback {
    Box::new(move |read_time, temp| {
        let kick = {
            let mut state = lock_state(&state);
            state.apply_reading(read_time, temp, inv_smooth_time);
            state.in_calibration && state.measurement.0 >= state.next_auto_temp
        };
        if !kick {
            return;
        }
        let Some(printer) = printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let state = Arc::clone(&state);
        let printer = Arc::downgrade(&printer);
        // Upstream's `register_async_callback`: the check runs on the reactor
        // rather than inside the sensor's read path.
        reactor.call_later(0., Box::new(move |_| check_kick_next(&state, &printer)));
    })
}

/// Upstream's `_check_kick_next` (`temperature_probe.py:155-159`): re-check the
/// threshold the reading crossed, clear it, and take the next sample.
///
/// The script runs on the host's runtime rather than blocking the reactor's
/// dispatcher, as `delayed_gcode` runs its own script (`delayed_gcode.rs`).
fn check_kick_next(state: &Mutex<State>, printer: &Weak<Printer>) {
    {
        let mut state = lock_state(state);
        if !(state.in_calibration && state.measurement.0 >= state.next_auto_temp) {
            return;
        }
        state.next_auto_temp = 99999999.;
    }
    let Some(printer) = printer.upgrade() else {
        return;
    };
    let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) else {
        return;
    };
    match tokio::runtime::Handle::try_current() {
        Ok(handle) => {
            handle.spawn(async move {
                if let Err(error) = gcode.run_script("TEMPERATURE_PROBE_NEXT").await {
                    tracing::error!("Script running error: {error}");
                }
            });
        }
        Err(_) => tracing::warn!(
            "temperature_probe: running TEMPERATURE_PROBE_NEXT needs an async runtime"
        ),
    }
}

// ===========================================================================
// The object
// ===========================================================================

/// A `[temperature_probe <name>]` (`temperature_probe.py:TemperatureProbe`).
pub struct TemperatureProbe {
    /// The section id upstream calls `self.name` (`temperature_probe <name>`).
    pub name: String,
    /// The section's parsed options.
    pub options: TemperatureProbeOptions,
    /// The machine upstream keeps as `self.printer`: the command family looks
    /// the toolhead, the probe and the dispatcher up through it. `Weak` so the
    /// object adds no cycle of its own.
    printer: Weak<Printer>,
    /// The sensor the readings come from. Held (not just a weak handle) so the
    /// ADC/report callback that drives it keeps upgrading — the reason
    /// `temperature_sensor.rs` holds its own.
    #[allow(dead_code)]
    sensor: Arc<dyn heaters::Sensor>,
    /// The readings and calibration flags the sensor callback shares.
    state: Arc<Mutex<State>>,
    /// The drift helper upstream builds in `__init__`
    /// (`self.cal_helper = EddyDriftCompensation(…)`, `temperature_probe.py:125-137`):
    /// `None` while the config names no `probe_eddy_current <name>` section,
    /// which is upstream's `cal_helper is None`.
    cal_helper: Mutex<Option<Arc<EddyDriftCompensation>>>,
}

impl TemperatureProbe {
    /// The registered drift helper, cloned out of its slot — upstream's
    /// `self.cal_helper` read.
    fn drift_helper(&self) -> Option<Arc<EddyDriftCompensation>> {
        self.cal_helper
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// Upstream's `get_temp` (`:161-162`): the smoothed temperature and the
    /// calibration's target.
    pub fn get_temp(&self) -> (f64, f64) {
        let state = lock_state(&self.state);
        (state.measurement.0, state.target_temp)
    }

    /// Upstream's `is_in_calibration` (`:450-451`).
    pub fn is_in_calibration(&self) -> bool {
        lock_state(&self.state).in_calibration
    }

    /// Upstream's `stats` (`:467-468`).
    ///
    /// Nothing calls it yet: the port has no `Printer`-level walker that
    /// collects object `stats` (see the module docs).
    pub fn stats(&self) -> (bool, String) {
        let temperature = lock_state(&self.state).measurement.0;
        (false, format!("{}: temp={temperature:.1}", self.name))
    }
}

impl PrinterObject for TemperatureProbe {
    /// Upstream's `get_status` (`:453-465`). `temperature` is reported raw —
    /// upstream rounds only the two measured extremes.
    fn get_status(&self, _eventtime: f64) -> Value {
        // Upstream reads this off its `cal_helper` (`:457-459`): `false` when
        // no helper is registered, the helper's flag when one is.
        let compensation_enabled = self
            .drift_helper()
            .is_some_and(|helper| helper.is_enabled());
        let state = lock_state(&self.state);
        let (smoothed, measured_min, measured_max) = state.measurement;
        json!({
            "temperature": smoothed,
            "measured_min_temp": round2(measured_min),
            "measured_max_temp": round2(measured_max),
            "in_calibration": state.in_calibration,
            "estimated_expansion": state.total_expansion,
            "compensation_enabled": compensation_enabled,
        })
    }
}

impl std::fmt::Debug for TemperatureProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TemperatureProbe")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// Upstream's `round(value, 2)` (`temperature_probe.py:460-461`).
///
/// Python's `round` is half-to-even (banker's rounding), not Rust's
/// half-away-from-zero `.round()`. The logic mirrors [`round8`] but scales
/// to two decimal places.
fn round2(value: f64) -> f64 {
    let scaled = value * 100.0;
    let floor = scaled.floor();
    let frac = scaled - floor;
    let up = if frac > 0.5 {
        true
    } else if frac < 0.5 {
        false
    } else {
        // Exactly halfway: land on the even neighbour, as Python does — an
        // odd `floor` moves up to the even `floor + 1`.
        (floor as i64) % 2 != 0
    };
    (if up { floor + 1.0 } else { floor }) / 100.0
}

// ===========================================================================
// The calibration command family
// ===========================================================================

/// `TEMPERATURE_PROBE_CALIBRATE`'s help (`temperature_probe.py:324-325`).
const CALIBRATE_HELP: &str = "Calibrate probe temperature drift compensation";
/// `TEMPERATURE_PROBE_NEXT`'s help (`temperature_probe.py:409`).
///
/// It is also what the registration gives `TEMPERATURE_PROBE_COMPLETE`,
/// which upstream does although it defines
/// `cmd_TEMPERATURE_PROBE_COMPLETE_help` itself (`:434` vs `:377-381`);
/// the registration is kept verbatim, quirk included.
const NEXT_HELP: &str = "Sample next probe drift temperature";
/// `TEMPERATURE_PROBE_ABORT`'s help (`temperature_probe.py:439`).
const ABORT_HELP: &str = "Abort Probe Drift Calibration";
/// `TEMPERATURE_PROBE_ENABLE`'s help (`temperature_probe.py:443-445`).
const ENABLE_HELP: &str = "Set adjustment factor applied to drift correction";

/// The word after a section's prefix — upstream's `name.split(None, 1)[-1]`.
/// It is the mux value `TEMPERATURE_PROBE_CALIBRATE` / `_ENABLE` register
/// their `PROBE` key under (`temperature_probe.py:111`) and the word the
/// calibration compares against the probe's own name (`:347-348`).
fn short_name(name: &str) -> &str {
    match name.split_once(char::is_whitespace) {
        Some((_, rest)) => rest.trim_start(),
        None => name,
    }
}

impl TemperatureProbe {
    /// Register the two commands the section owns, muxed on the section's
    /// probe name (`temperature_probe.py:112-123`).
    ///
    /// # Errors
    /// A command name or mux value already taken, reported at config load the
    /// way upstream's `register_mux_command` reports it.
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        let pname = short_name(&self.name);
        {
            let this = Arc::clone(self);
            gcode
                .register_mux_command_with_params(
                    "TEMPERATURE_PROBE_CALIBRATE",
                    "PROBE",
                    Some(pname),
                    Arc::new(move |gcmd| {
                        let this = Arc::clone(&this);
                        Box::pin(async move { this.cmd_calibrate(gcmd).await })
                    }),
                    Some(CALIBRATE_HELP),
                    &["METHOD", "TARGET", "STEP"],
                )
                .map_err(ConfigError::new)?;
        }
        {
            let this = Arc::clone(self);
            gcode
                .register_mux_command_with_params(
                    "TEMPERATURE_PROBE_ENABLE",
                    "PROBE",
                    Some(pname),
                    sync(move |gcmd| this.cmd_enable(gcmd)),
                    Some(ENABLE_HELP),
                    // What `EddyDriftCompensation.set_enabled` reads
                    // (`temperature_probe.py:530`).
                    &["ENABLE"],
                )
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    /// The live machine, or the standing "not ready" error.
    fn live_printer(&self) -> Result<Arc<Printer>, CommandError> {
        self.printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// The dispatcher, which upstream reaches as `self.gcode`.
    fn gcode(&self) -> Result<Arc<GCodeDispatch>, CommandError> {
        let printer = self.live_printer()?;
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// The toolhead every move of the calibration goes through.
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        let printer = self.live_printer()?;
        printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// Upstream's `_get_probe` (`temperature_probe.py:259-263`): the probe
    /// the calibration drives, or its refusal.
    ///
    /// # Errors
    /// "No probe configured" when no probe section registered the object.
    fn get_probe(&self) -> Result<Arc<dyn ProbeSession>, CommandError> {
        let printer = self.live_printer()?;
        lookup_probe_session(&printer).ok_or_else(|| CommandError::new("No probe configured"))
    }

    /// The name the `probe` object reports — upstream reads
    /// `probe.get_status(None)["name"]` (`:346`). `None` when the probe's
    /// status carries no name (upstream's probe command helper always adds
    /// one, so this only happens for a probe whose status is its own).
    fn probe_name(&self) -> Option<String> {
        let printer = self.printer.upgrade()?;
        let probe = printer.lookup_object(PROBE_OBJECT)?;
        probe
            .get_status(printer.reactor().monotonic())
            .get("name")
            .and_then(Value::as_str)
            .map(str::to_string)
    }

    /// Upstream's `_check_homed` (`temperature_probe.py:289-297`): every axis
    /// must be homed before the calibration moves anything.
    ///
    /// # Errors
    /// "Printer must be homed before calibration" for the first axis that is
    /// not homed.
    fn check_homed(&self) -> Result<(), CommandError> {
        let printer = self.live_printer()?;
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let status = toolhead.get_status(printer.reactor().monotonic());
        // A toolhead that has not connected reports no status at all; taking
        // that as "nothing homed" keeps the refusal instead of a missing key.
        let homed = status
            .get("homed_axes")
            .and_then(Value::as_str)
            .unwrap_or("");
        if ["x", "y", "z"].iter().all(|axis| homed.contains(*axis)) {
            return Ok(());
        }
        Err(CommandError::new(
            "Printer must be homed before calibration",
        ))
    }

    /// Upstream's `_get_speeds` (`temperature_probe.py:317-322`):
    /// `(lift_speed, probe_speed, move_speed)`.
    ///
    /// # Errors
    /// "No probe configured", or whatever the probe's parameter read reports.
    fn get_speeds(&self) -> Result<(f64, f64, f64), CommandError> {
        let probe = self.get_probe()?;
        // Upstream asks the probe for its params with no command in hand
        // (`get_probe_params()`), so an empty command reads the same defaults
        // the section configured.
        let gcmd = self
            .gcode()?
            .create_gcode_command("PROBE", "", HashMap::new());
        let params = probe.probe_params(&gcmd)?;
        let move_speed = self
            .options
            .speed
            .unwrap_or_else(|| params.probe_speed.max(params.lift_speed));
        Ok((params.lift_speed, params.probe_speed, move_speed))
    }

    /// Upstream's `_move_to_start` (`temperature_probe.py:300-315`): park the
    /// nozzle where the calibration heats it up.
    ///
    /// # Errors
    /// Whatever the moves, the speeds or the heating wait report.
    async fn move_to_start(&self) -> Result<(), CommandError> {
        let toolhead = self.toolhead()?;
        let mut position = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let move_speed = self.get_speeds()?.2;
        if let Some(cal_pos) = self.options.cal_pos.clone() {
            if self.options.cal_extruder_temp.is_some() {
                // Move to the extruder heating z position.
                position.set_axis(Z_AXIS, self.options.cal_extruder_z);
                toolhead.move_to(position, move_speed)?;
            }
            position.set_axis(X_AXIS, cal_pos[0]);
            position.set_axis(Y_AXIS, cal_pos[1]);
            toolhead.move_to(position, move_speed)?;
            if let Some(temp) = self.options.cal_extruder_temp {
                self.set_extruder_temp(temp, true).await?;
            }
            position.set_axis(Z_AXIS, cal_pos[2]);
            toolhead.move_to(position, move_speed)?;
        } else if let Some(temp) = self.options.cal_extruder_temp {
            position.set_axis(Z_AXIS, self.options.cal_extruder_z);
            toolhead.move_to(position, move_speed)?;
            self.set_extruder_temp(temp, true).await?;
        }
        Ok(())
    }

    /// Upstream's `_set_extruder_temp` (`temperature_probe.py:265-278`): the
    /// heater script, and the `TEMPERATURE_WAIT` that follows it when asked
    /// to wait.
    ///
    /// Nothing to run when `calibration_extruder_temp` is not configured —
    /// the early return upstream starts with.
    ///
    /// # Errors
    /// The toolhead lookup, or the scripts themselves.
    async fn set_extruder_temp(&self, temp: f64, wait: bool) -> Result<(), CommandError> {
        if self.options.cal_extruder_temp.is_none() {
            return Ok(());
        }
        let extruder = self.toolhead()?.active_extruder();
        let gcode = self.gcode()?;
        // `%f` upstream: six decimals.
        gcode
            .run_script_from_command(&format!(
                "SET_HEATER_TEMPERATURE HEATER={extruder} TARGET={temp:.6}"
            ))
            .await?;
        if wait {
            gcode
                .run_script_from_command(&format!(
                    "TEMPERATURE_WAIT SENSOR={extruder} MINIMUM={temp:.6}"
                ))
                .await?;
        }
        Ok(())
    }

    /// Upstream's `_set_bed_temp` (`temperature_probe.py:280-287`), with the
    /// same "not configured, nothing to run" guard.
    ///
    /// # Errors
    /// The heater script itself.
    async fn set_bed_temp(&self, temp: f64) -> Result<(), CommandError> {
        if self.options.cal_bed_temp.is_none() {
            return Ok(());
        }
        self.gcode()?
            .run_script_from_command(&format!(
                "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET={temp:.6}"
            ))
            .await
    }

    /// Upstream's `_collect_sample` (`temperature_probe.py:164-178`): lift,
    /// move over the probe point, then hand the sample to the drift helper —
    /// which is what upstream returns unconditionally (`:178`).
    ///
    /// # Errors
    /// "No probe configured", whatever the moves report, or whatever the
    /// helper's sweep reports.
    async fn collect_sample(
        &self,
        mpresult: &Coord,
        tool_zero_z: f64,
    ) -> Result<f64, CommandError> {
        let probe = self.get_probe()?;
        let offsets = probe.offsets();
        let speeds = self.get_speeds()?;
        let (lift_speed, _, move_speed) = speeds;
        let toolhead = self.toolhead()?;
        let mut cur_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        // Move to the probe to sample collection position.
        cur_pos.set_axis(Z_AXIS, cur_pos.z() + self.options.horizontal_move_z);
        toolhead.move_to(cur_pos, lift_speed)?;
        cur_pos.set_axis(X_AXIS, cur_pos.x() - offsets.x);
        cur_pos.set_axis(Y_AXIS, cur_pos.y() - offsets.y);
        toolhead.move_to(cur_pos, move_speed)?;
        if let Some(helper) = self.drift_helper() {
            // Upstream's `return self.cal_helper.collect_sample(mpresult,
            // tool_zero_z, speeds)` (`:178`): the temperature the drift sweep
            // recorded, averaged over one sweep of the probe (`:617-618` reads
            // this same sensor).
            return helper.collect_sample(mpresult, tool_zero_z, speeds).await;
        }
        // Upstream never gets here: `cmd_TEMPERATURE_PROBE_CALIBRATE` refuses
        // a run with no helper registered first (`:355-358`). What a run
        // without one returns is this sensor's current reading, so the state
        // machine still climbs `step` by `step` towards `target_temp`.
        Ok(lock_state(&self.state).measurement.0)
    }

    /// Upstream's `_prepare_next_sample` (`temperature_probe.py:179-197`):
    /// take `ABORT` back from the finished manual probe, settle at
    /// `resting_z` and schedule the next sample's temperature.
    ///
    /// # Errors
    /// The registration, the speeds or the move.
    fn prepare_next_sample(
        self: &Arc<Self>,
        last_temp: f64,
        tool_zero_z: f64,
    ) -> Result<(), CommandError> {
        let gcode = self.gcode()?;
        // Register our own abort command now that the manual probe has
        // finished and unregistered (`:181-186`).
        {
            let this = Arc::clone(self);
            let handler: CommandHandler = Arc::new(move |gcmd| {
                let this = Arc::clone(&this);
                Box::pin(async move { this.cmd_abort(gcmd).await })
            });
            gcode
                .register_command("ABORT", handler, Some(ABORT_HELP), false)
                .map_err(CommandError::new)?;
        }
        let probe_speed = self.get_speeds()?.1;
        let toolhead = self.toolhead()?;
        let mut cur_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        // Move down to the resting position.
        cur_pos.set_axis(Z_AXIS, tool_zero_z + self.options.resting_z);
        toolhead.move_to(cur_pos, probe_speed)?;
        let (cnt, exp_cnt, next_auto_temp) = {
            let mut state = lock_state(&self.state);
            let next_auto_temp = last_temp + state.step;
            state.next_auto_temp = next_auto_temp;
            (state.sample_count, state.expected_count, next_auto_temp)
        };
        gcode.respond_info(
            &format!(
                "{}: collected sample {cnt}/{exp_cnt} at temp {last_temp:.2}C, \
                 next sample scheduled at temp {next_auto_temp:.2}C",
                self.name
            ),
            true,
        );
        Ok(())
    }

    /// Upstream's `_manual_probe_finalize` (`temperature_probe.py:200-232`):
    /// fold the accepted Z into the expansion estimate, take the sample and
    /// either finish or schedule the next one.
    ///
    /// # Errors
    /// Whatever the sample, the moves or the finalization report — after
    /// finalizing, as upstream's `except …: finalize(False); raise` does.
    async fn manual_probe_finalize(
        self: &Arc<Self>,
        mpresult: Option<Coord>,
    ) -> Result<(), CommandError> {
        let Some(mpresult) = mpresult else {
            // Calibration aborted.
            return self.finalize_drift_cal(false, None).await;
        };
        let bed_z = mpresult.z();
        {
            let mut state = lock_state(&self.state);
            if let Some(last_zero_z) = state.last_zero_pos {
                state.total_expansion += last_zero_z - bed_z;
                tracing::info!(
                    "Estimated Total Thermal Expansion: {:.6}",
                    state.total_expansion
                );
            }
            state.last_zero_pos = Some(bed_z);
        }
        let tool_zero_z = self
            .toolhead()?
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?
            .z();
        let last_temp = match self.collect_sample(&mpresult, tool_zero_z).await {
            Ok(temp) => temp,
            Err(err) => {
                self.finalize_drift_cal(false, None).await?;
                return Err(err);
            }
        };
        let (sample_count, target_temp) = {
            let mut state = lock_state(&self.state);
            state.sample_count += 1;
            (state.sample_count, state.target_temp)
        };
        if last_temp >= target_temp {
            // Calibration done.
            return self.finalize_drift_cal(true, None).await;
        }
        if let Err(err) = self.prepare_next_sample(last_temp, tool_zero_z) {
            self.finalize_drift_cal(false, None).await?;
            return Err(err);
        }
        if sample_count == 1 {
            if let Some(temp) = self.options.cal_bed_temp {
                if let Err(err) = self.set_bed_temp(temp).await {
                    self.finalize_drift_cal(false, None).await?;
                    return Err(err);
                }
            }
        }
        Ok(())
    }

    /// `_manual_probe_finalize` from the finalize callback, which is not
    /// itself async: the continuation runs as a spawned task when a runtime is
    /// driving the host (`axis_twist_compensation`'s manual-probe callback
    /// runs the same way).
    fn spawn_manual_probe_finalize(self: &Arc<Self>, mpresult: Option<Coord>) {
        let Ok(handle) = tokio::runtime::Handle::try_current() else {
            tracing::warn!(
                "temperature_probe: continuing a manual probe sample needs an async runtime"
            );
            return;
        };
        let this = Arc::clone(self);
        handle.spawn(async move {
            if let Err(err) = this.manual_probe_finalize(mpresult).await {
                // Upstream re-raises into the command that ended the manual
                // probe; from a continuation there is no command left to
                // carry it, so the line is reported the way one reports a
                // failed command.
                if let Ok(gcode) = this.gcode() {
                    gcode.respond_raw(&format!("!! {err}"));
                }
            }
        });
    }

    /// Upstream's `_finalize_drift_cal` (`temperature_probe.py:233-257`):
    /// clear the state, take the temporary commands away, switch the heaters
    /// off and report an aborted run.
    ///
    /// # Errors
    /// The heater scripts — the same point at which upstream's own
    /// `run_script_from_command` would stop it.
    async fn finalize_drift_cal(
        &self,
        success: bool,
        msg: Option<&str>,
    ) -> Result<(), CommandError> {
        {
            let mut state = lock_state(&self.state);
            state.next_auto_temp = 99999999.;
            state.target_temp = 0.;
            state.expected_count = 0;
            state.sample_count = 0;
            state.step = 2.;
            state.in_calibration = false;
            state.last_zero_pos = None;
            state.total_expansion = 0.;
            state.start_pos = None;
        }
        let gcode = self.gcode()?;
        // Unregister the temporary commands (`:244-246`); a name that is not
        // registered is upstream's no-op here too.
        gcode.unregister_command("ABORT");
        gcode.unregister_command("TEMPERATURE_PROBE_NEXT");
        gcode.unregister_command("TEMPERATURE_PROBE_COMPLETE");
        // Turn off the heaters (`:247-248`).
        self.set_extruder_temp(0., false).await?;
        self.set_bed_temp(0.).await?;
        // The helper's close-out (`:249-254`): upstream calls
        // `self.cal_helper.finish_calibration(success)` and turns a
        // `gcode.error` from it into `success = False` + its message.
        let mut success = success;
        let mut msg = msg.map(str::to_string);
        if let Some(helper) = self.drift_helper() {
            if let Err(err) = helper.finish_calibration(success) {
                success = false;
                msg = Some(err.to_string());
            }
        }
        if !success {
            let msg = msg.unwrap_or_else(|| format!("{}: calibration aborted", self.name));
            gcode.respond_info(&msg, true);
        }
        Ok(())
    }

    /// Upstream's `_auto_probe` (`temperature_probe.py:327-337`): one `PROBE`
    /// round through the probe session, then the same finalize the manual
    /// probe would run.
    ///
    /// # Errors
    /// "No probe configured", whatever the session reports, or the finalize.
    async fn auto_probe(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let method = lock_state(&self.state).method.clone();
        let mut fo_params = gcmd.get_command_parameters().clone();
        fo_params.insert("METHOD".to_string(), method);
        let fo_gcmd = self
            .gcode()?
            .create_gcode_command("PROBE", "PROBE", fo_params);
        let probe = self.get_probe()?;
        probe.start_probe_session(&fo_gcmd)?;
        probe.run_probe(&fo_gcmd).await?;
        let pos = probe
            .pull_probed_results()
            .into_iter()
            .next()
            .ok_or_else(|| {
                CommandError::new("Internal probe error - probe session returned no result")
            })?;
        probe.end_probe_session()?;
        self.manual_probe_finalize(Some(pos)).await
    }

    /// `TEMPERATURE_PROBE_CALIBRATE` (`temperature_probe.py:338-409`).
    ///
    /// # Errors
    /// In upstream's order: not homed, then whatever
    /// [`Self::start_calibration`] reports.
    async fn cmd_calibrate(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let method = gcmd.get_str_default("METHOD", "manual").to_lowercase();
        // Upstream reads `METHOD` first (`:354`), then refuses right here
        // when no drift-compensation helper is registered (`:355-358`,
        // "No calibration helper registered for [%s]").
        if self.drift_helper().is_none() {
            return Err(CommandError::new(format!(
                "No calibration helper registered for [{}]",
                self.name
            )));
        }
        self.check_homed()?;
        self.start_calibration(gcmd, &method).await
    }

    /// Everything `TEMPERATURE_PROBE_CALIBRATE` does after the homed gate
    /// (`temperature_probe.py:345-408`): link the probe, refuse a calibration
    /// that is already running, read `TARGET`/`STEP`, register the two
    /// temporary commands, move to the start and hand over to the
    /// interactive probe.
    ///
    /// Split from [`Self::cmd_calibrate`] only at that gate, so the error
    /// paths behind it are reachable on a test machine whose axes are never
    /// homed; the order inside is upstream's.
    ///
    /// # Errors
    /// "No probe configured", a link or conflict refusal, a `TARGET`/`STEP`
    /// bound, "too few expected samples", taken temporary commands, or the
    /// initial move.
    async fn start_calibration(
        self: &Arc<Self>,
        gcmd: &GcodeCommand,
        method: &str,
    ) -> Result<(), CommandError> {
        let printer = self.live_printer()?;
        let gcode = self.gcode()?;
        // Upstream's `_get_probe` refusal sits here, before the link check
        // (`:345`); the session itself is reached again where it is driven.
        let _ = self.get_probe()?;
        // `[temperature_probe <name>]` has to be the section its probe reports
        // (`:345-352`). A probe whose status carries no `name` has nothing to
        // compare against, so the check stands aside for it rather than
        // failing on a missing key.
        if let Some(probe_name) = self.probe_name() {
            if short_name(&probe_name) != short_name(&self.name) {
                return Err(CommandError::new(format!(
                    "[{}] not linked to registered probe [{}].",
                    self.name, probe_name
                )));
            }
        }
        manual_probe::verify_no_manual_probe(&printer, &gcode)?;
        if lock_state(&self.state).in_calibration {
            return Err(CommandError::new(
                "Already in probe drift calibration. Use TEMPERATURE_PROBE_COMPLETE or ABORT \
                 to exit.",
            ));
        }
        let cur_temp = lock_state(&self.state).measurement.0;
        // `TARGET` and `STEP` (`:361-362`): both parse through the shared
        // `get`, whose missing/parse wording is upstream's. The `above` bound
        // is checked here because this port's `get` prints it as "must have
        // above of …" (`gcode.rs`) where upstream prints "must be above …" —
        // and the error text is upstream's.
        let target_temp = gcmd.get(
            "TARGET",
            None,
            |raw| raw.parse::<f64>().ok(),
            None,
            None,
            None,
            None,
        )?;
        if target_temp <= cur_temp {
            return Err(CommandError::new(format!(
                "Error on '{}': TARGET must be above {:?}",
                gcmd.commandline(),
                cur_temp
            )));
        }
        let step = gcmd.get(
            "STEP",
            Some(2.),
            |raw| raw.parse::<f64>().ok(),
            Some(1.),
            None,
            None,
            None,
        )?;
        let expected_count = ((target_temp - cur_temp) / step + 0.5) as i64;
        if expected_count < 3 {
            return Err(CommandError::new(format!(
                "Invalid STEP and/or TARGET parameters resulted in too few expected samples: \
                 {expected_count}"
            )));
        }
        // The two temporary commands (`:372-386`): NEXT is registered first,
        // and a failure leaves it registered exactly as upstream's `try`
        // does.
        let next: CommandHandler = {
            let this = Arc::clone(self);
            Arc::new(move |gcmd| {
                let this = Arc::clone(&this);
                Box::pin(async move { this.cmd_next(gcmd).await })
            })
        };
        let complete: CommandHandler = {
            let this = Arc::clone(self);
            Arc::new(move |gcmd| {
                let this = Arc::clone(&this);
                Box::pin(async move { this.cmd_complete(gcmd).await })
            })
        };
        let registered = gcode
            .register_command("TEMPERATURE_PROBE_NEXT", next, Some(NEXT_HELP), false)
            .and_then(|()| {
                // Upstream passes NEXT's help for COMPLETE (`:377-381`);
                // see [`NEXT_HELP`].
                gcode.register_command(
                    "TEMPERATURE_PROBE_COMPLETE",
                    complete,
                    Some(NEXT_HELP),
                    false,
                )
            });
        if registered.is_err() {
            return Err(CommandError::new(
                "Auxiliary Probe Drift Commands already registered. Use \
                 TEMPERATURE_PROBE_COMPLETE or ABORT to exit.",
            ));
        }
        {
            let mut state = lock_state(&self.state);
            state.method = method.to_string();
            // Upstream sets `in_calibration` and then calls
            // `self.cal_helper.start_calibration()` (`:387-388`), which
            // switches the correction off and empties the helper's sample
            // buckets. The state lock is dropped first: the helper reads the
            // sensor's state of its own, and the two locks stay unordered.
            state.in_calibration = true;
        }
        if let Some(helper) = self.drift_helper() {
            helper.start_calibration();
        }
        {
            let mut state = lock_state(&self.state);
            state.target_temp = target_temp;
            state.step = step;
            state.sample_count = 0;
            state.expected_count = expected_count;
        }
        // If configured, move to the heating position and turn on the
        // extruder (`:394-398`).
        if let Err(err) = self.move_to_start().await {
            self.finalize_drift_cal(false, Some("Error during initial move"))
                .await?;
            return Err(err);
        }
        // Capture the start position and begin the initial probe (`:399-408`).
        let start = self
            .toolhead()?
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        lock_state(&self.state).start_pos = Some([start.x(), start.y()]);
        if method == "tap" {
            return self.auto_probe(gcmd).await;
        }
        self.start_manual_probe(gcmd)
    }

    /// `TEMPERATURE_PROBE_NEXT` (`temperature_probe.py:410-433`).
    ///
    /// # Errors
    /// A manual probe is running, or whatever the moves report.
    async fn cmd_next(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let printer = self.live_printer()?;
        let gcode = self.gcode()?;
        manual_probe::verify_no_manual_probe(&printer, &gcode)?;
        lock_state(&self.state).next_auto_temp = 99999999.;
        let toolhead = self.toolhead()?;
        let mut cur_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let start_z = cur_pos.z();
        let (lift_speed, probe_speed, move_speed) = self.get_speeds()?;
        // Lift, move the nozzle back to the start position, and come back
        // down to where the probe started (`:416-425`).
        cur_pos.set_axis(Z_AXIS, cur_pos.z() + self.options.horizontal_move_z);
        toolhead.move_to(cur_pos, lift_speed)?;
        let [start_x, start_y] = lock_state(&self.state)
            .start_pos
            .ok_or_else(|| CommandError::new("No calibration start position"))?;
        cur_pos.set_axis(X_AXIS, start_x);
        cur_pos.set_axis(Y_AXIS, start_y);
        toolhead.move_to(cur_pos, move_speed)?;
        cur_pos.set_axis(Z_AXIS, start_z);
        toolhead.move_to(cur_pos, probe_speed)?;
        // The manual probe registers its own `ABORT` (`:426`).
        gcode.unregister_command("ABORT");
        if lock_state(&self.state).method == "tap" {
            return self.auto_probe(gcmd).await;
        }
        self.start_manual_probe(gcmd)
    }

    /// `TEMPERATURE_PROBE_COMPLETE` (`temperature_probe.py:435-437`).
    ///
    /// # Errors
    /// A manual probe is running, or the heater scripts the finalize runs.
    async fn cmd_complete(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let printer = self.live_printer()?;
        let gcode = self.gcode()?;
        manual_probe::verify_no_manual_probe(&printer, &gcode)?;
        let sample_count = lock_state(&self.state).sample_count;
        self.finalize_drift_cal(sample_count >= 3, None).await
    }

    /// `TEMPERATURE_PROBE_ABORT` (`temperature_probe.py:440-441`).
    ///
    /// # Errors
    /// The heater scripts the finalize runs.
    async fn cmd_abort(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.finalize_drift_cal(false, None).await
    }

    /// `TEMPERATURE_PROBE_ENABLE` (`temperature_probe.py:446-448`): forward to
    /// the helper's `set_enabled`, which does the parameter read and both
    /// refusals — with no helper registered this is the no-op upstream runs.
    ///
    /// # Errors
    /// Whatever `EddyDriftCompensation::set_enabled` reports.
    fn cmd_enable(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        if let Some(helper) = self.drift_helper() {
            helper.set_enabled(gcmd)?;
        }
        Ok(())
    }

    /// Hand one sample to the interactive probe — upstream's
    /// `manual_probe.ManualProbeHelper` at `:405-408` and `:430-433`.
    ///
    /// # Errors
    /// No `manual_probe` object, or an already running manual probe.
    fn start_manual_probe(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let printer = self.live_printer()?;
        let manual = printer
            .lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let this = Arc::clone(self);
        let callback: FinalizeCallback = Arc::new(move |mpresult| {
            this.spawn_manual_probe_finalize(mpresult);
        });
        manual.start_helper(&printer, gcmd, callback)
    }
}

// ===========================================================================
// EddyDriftCompensation
// ===========================================================================

/// `config.getlists("drift_calibration", None, seps=(',', '\n'), parser=float)`
/// (`temperature_probe.py:488-490`): split on newlines first — blank lines
/// drop out, as upstream's nested parser filters them — then each line on
/// commas with **no** empty filter, so a stray separator is a parse failure of
/// the whole option. Every group is parsed before any group's length is
/// checked, as upstream's two passes are ordered.
///
/// `None` when the option is absent — and when it is present but empty, where
/// upstream's empty tuple walks on into `_check_calibration` and raises
/// `IndexError`; here that reads as "no curves configured".
///
/// # Errors
/// "Unable to parse option 'drift_calibration' in section '\<id\>'" for a
/// non-number, or upstream's "Invalid polynomial in drift calibration" for a
/// group that is not three coefficients (`:492-495`).
fn read_drift_calibration(
    config: &ConfigWrapper,
) -> Result<Option<Vec<Polynomial2d>>, ConfigError> {
    let Some(text) = config.get_str("drift_calibration") else {
        return Ok(None);
    };
    let identifier = config.identifier();
    let mut groups: Vec<Vec<f64>> = Vec::new();
    for line in text.split('\n') {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let mut coefs = Vec::new();
        for raw in line.split(',') {
            let coef = raw.trim().parse::<f64>().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option 'drift_calibration' in section '{identifier}'"
                ))
            })?;
            coefs.push(coef);
        }
        groups.push(coefs);
    }
    if groups.is_empty() {
        return Ok(None);
    }
    for coefs in &groups {
        if coefs.len() != 3 {
            return Err(ConfigError::new(
                "Invalid polynomial in drift calibration".to_string(),
            ));
        }
    }
    Ok(Some(
        groups
            .into_iter()
            .map(|coefs| Polynomial2d::new(coefs[0], coefs[1], coefs[2]))
            .collect(),
    ))
}

/// The eddy probe's temperature-drift correction (`EddyDriftCompensation`,
/// `temperature_probe.py:479-714`): the stored `drift_calibration` curves say
/// how the probe's resonance drifts with temperature, and every frequency the
/// probe converts is moved back toward the temperature its Z calibration was
/// taken at.
///
/// The loader builds it when the config names a `probe_eddy_current <name>`
/// section and registers it with that probe on the spot (`:125-137`) — which
/// is what [`TemperatureProbe`]'s `cal_helper` slot holds. With no such
/// section there is no helper: the command family refuses to calibrate
/// (`:355-358`) and `get_status` reports `compensation_enabled: false`.
pub struct EddyDriftCompensation {
    /// The section id upstream calls `self.name`.
    name: String,
    /// The machine upstream keeps as `self.printer`: the sweep looks the
    /// toolhead and the probe up through it, the close-outs the `gcode` and
    /// `configfile` objects. `Weak`, as [`TemperatureProbe`] holds it.
    printer: Weak<Printer>,
    /// The sensor every correction is dated from (upstream's
    /// `self.temp_sensor`, the `TemperatureProbe` itself) — shared as its
    /// readings state, so the helper reads the temperature without pinning
    /// the object that owns the helper.
    temp: Arc<Mutex<State>>,
    /// The helper's own fields under one lock (upstream's `cal_temp` /
    /// `drift_calibration` / `calibration_samples` / `max_valid_temp` /
    /// `dc_min_temp` / `min_freq` / `enabled`, `:484-525`).
    inner: Mutex<DriftState>,
}

/// What [`EddyDriftCompensation`] keeps of upstream's `__init__` fields.
#[derive(Debug)]
struct DriftState {
    /// The temperature the Z calibration was taken at (`cal_temp`, `0.`).
    cal_temp: f64,
    /// The highest temperature the curves are validated over
    /// (`max_validation_temp`, default `60.`).
    max_valid_temp: f64,
    /// The stored curves, highest frequency first (`drift_calibration`);
    /// `None` when none are configured.
    drift_calibration: Option<Vec<Polynomial2d>>,
    /// The lowest frequency the lowest curve reaches over `0..=120` °C
    /// (`min_freq`, `999999999999.` until a calibration exists — `:486, :499`).
    min_freq: f64,
    /// The current run's samples: one `(temperature, frequency)` list per
    /// window (`calibration_samples`); `None` between runs.
    calibration_samples: Option<Vec<Vec<(f64, f64)>>>,
    /// Whether the correction is applied (`enabled`).
    enabled: bool,
}

impl EddyDriftCompensation {
    /// Upstream's `__init__` (`:481-525`): read the section's four options,
    /// load and validate the drift curves, and start enabled exactly when
    /// they can be used.
    ///
    /// # Errors
    /// The option reads' parse wording, `Invalid polynomial in drift
    /// calibration` for a curve that is not three coefficients, or
    /// [`Self::check_calibration`]'s crossing message.
    fn read(
        config: &ConfigWrapper,
        printer: Weak<Printer>,
        temp: Arc<Mutex<State>>,
    ) -> Result<Self, ConfigError> {
        let name = config.identifier();
        // Upstream's read order (`:484-488`).
        let cal_temp = config.get_float("calibration_temp", Some(0.))?;
        let max_valid_temp = config.get_float("max_validation_temp", Some(60.))?;
        let dc_min_temp = config.get_float("drift_calibration_min_temp", Some(0.))?;
        let drift_calibration = read_drift_calibration(config)?;
        let mut min_freq = 999999999999.;
        if let Some(calibration) = &drift_calibration {
            // Validate before the curves are used for anything (`:496`).
            Self::check_calibration(calibration, &name, dc_min_temp, max_valid_temp)
                .map_err(ConfigError::new)?;
            // `low_poly = self.drift_calibration[-1]; min([low_poly(temp) for
            // temp in range(121)])` (`:498-499`).
            let low_poly = calibration
                .last()
                .expect("a configured drift calibration is never empty");
            min_freq = (0..121)
                .map(|temp| low_poly.eval(temp as f64))
                .fold(f64::INFINITY, f64::min);
            let curves = calibration
                .iter()
                .map(|poly| format!("{poly:?}"))
                .collect::<Vec<_>>()
                .join("\n");
            tracing::info!(
                "{name}: loaded temperature drift calibration. Min Temp: {dc_min_temp:.2}, \
                 Min Freq: {min_freq:.6}\n{curves}"
            );
        } else {
            tracing::info!(
                "{name}: No drift calibration configured, disabling temperature drift \
                 compensation"
            );
        }
        let mut enabled = drift_calibration.is_some();
        if cal_temp < 1e-6 && enabled {
            // No saved Z-calibration temperature to correct toward (`:517-523`).
            enabled = false;
            tracing::info!(
                "{name}: No temperature saved for eddy probe calibration, disabling temperature \
                 drift compensation."
            );
        }
        Ok(Self {
            name,
            printer,
            temp,
            inner: Mutex::new(DriftState {
                cal_temp,
                max_valid_temp,
                drift_calibration,
                min_freq,
                calibration_samples: None,
                enabled,
            }),
        })
    }

    /// The helper's own lock, poisoned or not.
    fn lock(&self) -> MutexGuard<'_, DriftState> {
        self.inner.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The machine, or the standing "not ready" error (the lookup failure
    /// upstream's `self.printer.lookup_object(...)` becomes).
    fn live_printer(&self) -> Result<Arc<Printer>, CommandError> {
        self.printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// `_check_calibration` (`:657-673`): every curve must sit strictly below
    /// the one before it, degree by degree, over `start_temp..=end_temp`.
    /// Upstream raises through the `error` argument it is handed —
    /// `config.error` at load, `gcode.error` at finish — so the message comes
    /// back here for the caller to raise as its own error type.
    ///
    /// # Panics
    /// Never: both call sites pass a non-empty slice — a loaded calibration
    /// has curves, and a finished run fits one per window.
    fn check_calibration(
        calibration: &[Polynomial2d],
        name: &str,
        start_temp: f64,
        end_temp: f64,
    ) -> Result<(), String> {
        // Python's `int()` truncates toward zero, as the cast does.
        let mut temp = start_temp as i64;
        let end = end_temp as i64 + 1;
        while temp < end {
            let mut last_freq = calibration[0].eval(temp as f64);
            for (i, poly) in calibration[1..].iter().enumerate() {
                let next_freq = poly.eval(temp as f64);
                if next_freq >= last_freq {
                    return Err(format!(
                        "{name}: invalid calibration detected, curve at index {} overlaps \
                         previous curve at temp {temp}C.",
                        i + 1
                    ));
                }
                last_freq = next_freq;
            }
            temp += 1;
        }
        Ok(())
    }

    /// `is_enabled` (`:526-527`).
    pub fn is_enabled(&self) -> bool {
        self.lock().enabled
    }

    /// `set_enabled` (`:528-543`): read `ENABLE`, refuse an enable that could
    /// never apply, then switch the flag.
    ///
    /// # Errors
    /// "Error on '\<commandline\>': missing ENABLE" without the word, or
    /// upstream's two refusals (`:534-541`).
    pub fn set_enabled(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let enabled = gcmd.get_int("ENABLE")? != 0;
        let mut state = self.lock();
        if enabled {
            if state.drift_calibration.is_none() {
                return Err(CommandError::new(
                    "No drift calibration configured, cannot enable temperature drift \
                     compensation",
                ));
            }
            if state.cal_temp < 1e-6 {
                return Err(CommandError::new(
                    "Z Calibration temperature not configured, cannot enable temperature drift \
                     compensation",
                ));
            }
        }
        state.enabled = enabled;
        Ok(())
    }

    /// `note_z_calibration_start` (`:544-546`): the Z calibration begins, so
    /// the saved calibration temperature starts at the sensor's reading.
    ///
    /// No caller yet: upstream's is `EddyCalibrationTool.do_calibration_moves`
    /// (`probe_eddy_current.py:134`), which this port has not brought over
    /// (see the module docs).
    pub fn note_z_calibration_start(&self) {
        let temperature = self.get_temperature();
        self.lock().cal_temp = temperature;
    }

    /// `note_z_calibration_finish` (`:547-558`): the run's temperature is the
    /// midpoint of its start and finish readings, written back as
    /// `calibration_temp` for `SAVE_CONFIG` and reported.
    ///
    /// No caller yet — see [`Self::note_z_calibration_start`].
    pub fn note_z_calibration_finish(&self) {
        let temperature = self.get_temperature();
        let cal_temp = {
            let mut state = self.lock();
            state.cal_temp = (state.cal_temp + temperature) / 2.0;
            state.cal_temp
        };
        let Some(printer) = self.printer.upgrade() else {
            tracing::warn!("{}: printer is gone, calibration_temp not saved", self.name);
            return;
        };
        let Some(configfile) = printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT) else {
            tracing::warn!(
                "{}: no configfile object, calibration_temp not saved",
                self.name
            );
            return;
        };
        // `"%.6f "` upstream — trailing space and all (`:554`).
        configfile.set(&self.name, "calibration_temp", &format!("{cal_temp:.6} "));
        if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
            gcode.respond_info(
                &format!(
                    "{}: Z Calibration Temperature set to {cal_temp:.2}. The SAVE_CONFIG command \
                     will update the printer config file and restart the printer.",
                    self.name
                ),
                true,
            );
        }
    }

    /// `start_calibration` (`:622-625`): switch the correction off and open
    /// fresh sample buckets for the run.
    pub fn start_calibration(&self) {
        let mut state = self.lock();
        state.enabled = false;
        state.calibration_samples = Some(vec![Vec::new(); DRIFT_SAMPLE_COUNT]);
    }

    /// `finish_calibration` (`:626-656`): fit one curve per sample window once
    /// the run is closed out, check the curves do not cross, and save them.
    ///
    /// # Errors
    /// "calibration error, not enough samples" with no run behind it (`:633-636`),
    /// [`Self::check_calibration`]'s crossing message, or a window set
    /// `Polynomial2d::fit` cannot solve — upstream's `fit` would raise out of
    /// the solve instead.
    pub fn finish_calibration(&self, success: bool) -> Result<(), CommandError> {
        let cal_samples = self.lock().calibration_samples.take();
        if !success {
            return Ok(());
        }
        let printer = self.live_printer()?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let Some(cal_samples) = cal_samples else {
            return Err(CommandError::new("calibration error, not enough samples"));
        };
        if cal_samples.len() < 3 {
            return Err(CommandError::new("calibration error, not enough samples"));
        }
        // `min_temp, _ = cal_samples[0][0]` / `max_temp, _ = cal_samples[-1][0]`
        // (`:637-638`): every sample in a window carries that window's
        // temperature, so the first tuple of the first and last window bounds
        // the validation range.
        let min_temp = cal_samples
            .first()
            .and_then(|window| window.first())
            .map(|(temp, _)| *temp)
            .ok_or_else(|| CommandError::new("calibration error, not enough samples"))?;
        let max_temp = cal_samples
            .last()
            .and_then(|window| window.first())
            .map(|(temp, _)| *temp)
            .ok_or_else(|| CommandError::new("calibration error, not enough samples"))?;
        // One fit per window at its Z height (`:639-644`).
        let mut polynomials = Vec::with_capacity(cal_samples.len());
        for (i, coords) in cal_samples.iter().enumerate() {
            let height = 0.05 + i as f64 * 0.5;
            let poly = Polynomial2d::fit(coords).ok_or_else(|| {
                CommandError::new("calibration error, unable to fit a drift calibration polynomial")
            })?;
            tracing::info!("Polynomial at Z={height:.2}: {poly:?}");
            polynomials.push(poly);
        }
        let end_vld_temp = self.lock().max_valid_temp.max(max_temp);
        Self::check_calibration(&polynomials, &self.name, min_temp, end_vld_temp)
            .map_err(CommandError::new)?;
        // The two `configfile.set` writes and the report (`:645-656`).
        let curves = polynomials
            .iter()
            .map(|poly| poly.to_string())
            .collect::<Vec<_>>()
            .join("\n");
        // Upstream's `"\n" + "\n".join([str(p) …])` (`:646`): the option's
        // first line stays empty, every curve a continuation below it.
        let coef_cfg = format!("\n{curves}");
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        configfile.set(&self.name, "drift_calibration", &coef_cfg);
        configfile.set(
            &self.name,
            "drift_calibration_min_temp",
            // Upstream stores the float as-is; `{:?}` prints it the way
            // Python's `str()` would for a temperature.
            &format!("{min_temp:?}"),
        );
        gcode.respond_info(
            &format!(
                "{}: generated {} 2D polynomials\nThe SAVE_CONFIG command will update the printer \
                 config file and restart the printer.",
                self.name,
                polynomials.len()
            ),
            true,
        );
        Ok(())
    }

    /// `collect_sample` (`:559-621`): one drift sweep — put a client on the
    /// probe's batch stream, walk nine half-millimetre Z windows up from the
    /// probe point, wait for the stream to fill them, and average each window
    /// into `calibration_samples`.
    ///
    /// Waits: upstream's `toolhead.wait_moves()` becomes this port's
    /// `flush_step_generation` (the queued moves generated and sent), and its
    /// `reactor.pause` loop (`:626-628`) becomes a `tokio` sleep polling the
    /// same condition — the window list emptying as the probe's data arrives.
    /// Like upstream's, the wait is unbounded: a sensor that stops delivering
    /// leaves the sweep waiting, where a reactor that never sees the data
    /// would.
    ///
    /// # Errors
    /// "Unknown config object 'probe_eddy_current \<name\>'" (upstream's
    /// `lookup_object`), a refused move or flush, or a window the stream left
    /// empty — where upstream divides by its length and raises (`:619-621`).
    pub async fn collect_sample(
        &self,
        mpresult: &Coord,
        tool_zero_z: f64,
        speeds: (f64, f64, f64),
    ) -> Result<f64, CommandError> {
        let printer = self.live_printer()?;
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let (lift_speed, probe_speed, _) = speeds;
        // Upstream's `sect_name = "probe_eddy_current " + self.name.split(None,
        // 1)[-1]`, then `lookup_object(sect_name).add_client(...)` (`:604-606`).
        let sect_name = format!("probe_eddy_current {}", short_name(&self.name));
        let probe = printer
            .lookup_object_as::<PrinterEddyProbe>(&sect_name)
            .ok_or_else(|| CommandError::new(format!("Unknown config object '{sect_name}'")))?;

        let mut cur_pos = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let sweep = Arc::new(Mutex::new(SweepState::default()));
        {
            // The client keeps its own handle to the sweep — it outlives this
            // call, as upstream's closure does.
            let sweep = Arc::clone(&sweep);
            let temp = Arc::clone(&self.temp);
            probe.add_client(move |msg| {
                let Some(rows) = batch_rows(msg) else {
                    return true;
                };
                let mut sweep = sweep.lock().unwrap_or_else(|p| p.into_inner());
                if sweep.move_times.is_empty() {
                    return true;
                }
                let cur_temp = lock_state(&temp).measurement.0;
                sweep.absorb(&rows, cur_temp)
            });
        }
        for i in 0..DRIFT_SAMPLE_COUNT {
            if i == 0 {
                // Move down to the first sample location (`:607-609`).
                cur_pos.set_axis(Z_AXIS, tool_zero_z + 0.05);
            } else {
                // Sample each .5mm in z: hop up 1mm, descend .5 (`:610-615`).
                cur_pos.set_axis(Z_AXIS, cur_pos.z() + 1.0);
                toolhead.move_to(cur_pos, lift_speed)?;
                cur_pos.set_axis(Z_AXIS, cur_pos.z() - 0.5);
            }
            toolhead.move_to(cur_pos, probe_speed)?;
            // The window this sample's data must land in (`:616-619`).
            let start = toolhead.get_last_move_time() + 0.05;
            let end = start + 0.1;
            sweep
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .move_times
                .push((i, start, end));
            toolhead.dwell(0.2);
        }
        // Upstream's `toolhead.wait_moves()`.
        toolhead.flush_step_generation().await?;
        // "Wait for sample collection to finish" (`:626-628`): upstream polls
        // the window list on its reactor — the condition is the same one.
        loop {
            let drained = sweep
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .move_times
                .is_empty();
            if drained {
                break;
            }
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
        let taken = std::mem::take(&mut *sweep.lock().unwrap_or_else(|p| p.into_inner()));
        let (sample_temp, windows) = taken.into_samples(mpresult.z())?;
        let mut state = self.lock();
        let samples = state
            .calibration_samples
            .get_or_insert_with(|| vec![Vec::new(); DRIFT_SAMPLE_COUNT]);
        for (bucket, sample) in samples.iter_mut().zip(windows) {
            bucket.push(sample);
        }
        Ok(sample_temp)
    }

    /// `get_temperature` (`:710-712`): this sensor's smoothed reading — what
    /// upstream reaches as `self.temp_sensor.get_temp()[0]`.
    pub fn get_temperature(&self) -> f64 {
        lock_state(&self.temp).measurement.0
    }

    /// `adjust_freq` (`:674-686`): a measured frequency moved from its origin
    /// temperature toward the stored calibration temperature.
    pub fn adjust_freq(&self, freq: f64, origin_temp: Option<f64>) -> f64 {
        let (enabled, min_freq, cal_temp) = {
            let state = self.lock();
            (state.enabled, state.min_freq, state.cal_temp)
        };
        if !enabled || freq < min_freq {
            return freq;
        }
        let origin_temp = origin_temp.unwrap_or_else(|| self.get_temperature());
        self.calc_freq(freq, origin_temp, cal_temp)
    }

    /// `unadjust_freq` (`:687-698`): the other direction — a frequency stored
    /// at the calibration temperature moved out to the destination.
    pub fn unadjust_freq(&self, freq: f64, dest_temp: Option<f64>) -> f64 {
        let (enabled, min_freq, cal_temp) = {
            let state = self.lock();
            (state.enabled, state.min_freq, state.cal_temp)
        };
        if !enabled || freq < min_freq {
            return freq;
        }
        let dest_temp = dest_temp.unwrap_or_else(|| self.get_temperature());
        self.calc_freq(freq, cal_temp, dest_temp)
    }

    /// `_calc_freq` (`:699-710`): walk the curves from the highest down until
    /// `freq` sits at or above one, then interpolate the move to `dest_temp`
    /// between that curve and the one above; above every curve, correct by how
    /// much the highest curve itself moves. Below them all, untouched.
    fn calc_freq(&self, freq: f64, origin_temp: f64, dest_temp: f64) -> f64 {
        let Some(calibration) = self.lock().drift_calibration.clone() else {
            return freq;
        };
        let mut high_freq: Option<f64> = None;
        for (pos, poly) in calibration.iter().enumerate() {
            let low_freq = poly.eval(origin_temp);
            if freq >= low_freq {
                let Some(high_freq) = high_freq else {
                    // Frequency above the max calibration value: correct by
                    // the top curve's own drift (`:702-704`).
                    return freq + (poly.eval(dest_temp) - low_freq);
                };
                // Piecewise interpolation toward `dest_temp` (`:705-709`).
                // `max` before `min`, as upstream's `min(1., max(0., …))`.
                let t = ((freq - low_freq) / (high_freq - low_freq)).max(0.).min(1.);
                let low_tgt_freq = poly.eval(dest_temp);
                let high_tgt_freq = calibration[pos - 1].eval(dest_temp);
                return (1. - t) * low_tgt_freq + t * high_tgt_freq;
            }
            high_freq = Some(low_freq);
        }
        // Frequency below the minimum: no correction.
        freq
    }
}

/// The probe-facing view of the helper: upstream hands `EddyCalibration` a
/// duck-typed object, here a trait (`probe_eddy_current::DriftCompensation`).
/// Each method is the inherent one above, reached through the name so the two
/// spellings cannot shadow each other.
impl DriftCompensation for EddyDriftCompensation {
    fn get_temperature(&self) -> f64 {
        EddyDriftCompensation::get_temperature(self)
    }

    fn adjust_freq(&self, freq: f64, origin_temp: Option<f64>) -> f64 {
        EddyDriftCompensation::adjust_freq(self, freq, origin_temp)
    }

    fn unadjust_freq(&self, freq: f64, dest_temp: Option<f64>) -> f64 {
        EddyDriftCompensation::unadjust_freq(self, freq, dest_temp)
    }
}

/// One drift sweep's live data (`temperature_probe.py:561-565`): upstream
/// keeps `move_times`, `temps` and `probe_samples` as locals the batch client
/// `_on_bulk_data_recd` closes over; here the client and the sweep share them
/// through this.
struct SweepState {
    /// The windows still to be filled: `(index, start, end)` in print time.
    move_times: Vec<(usize, f64, f64)>,
    /// The temperature each window recorded (`temps`).
    temps: Vec<f64>,
    /// Each window's `(frequency, measured z)` rows (`probe_samples`).
    probe_samples: Vec<Vec<(f64, f64)>>,
}

impl Default for SweepState {
    fn default() -> Self {
        Self {
            move_times: Vec::new(),
            temps: vec![0.; DRIFT_SAMPLE_COUNT],
            probe_samples: vec![Vec::new(); DRIFT_SAMPLE_COUNT],
        }
    }
}

impl SweepState {
    /// One batch message through upstream's `_on_bulk_data_recd` (`:586-603`):
    /// each row lands in the window its time falls in, windows retire as the
    /// stream passes them, and the answer is whether the client stays
    /// registered.
    fn absorb(&mut self, rows: &[[f64; 3]], cur_temp: f64) -> bool {
        let Some(&(mut idx, mut start_time, mut end_time)) = self.move_times.first() else {
            // No window open: upstream falls through to `True`.
            return true;
        };
        for row in rows {
            let ptime = row[0];
            while ptime > end_time {
                self.move_times.remove(0);
                let Some(&next) = self.move_times.first() else {
                    // The last window just retired (`:592-594`).
                    return idx >= DRIFT_SAMPLE_COUNT - 1;
                };
                (idx, start_time, end_time) = next;
            }
            if ptime < start_time {
                continue;
            }
            self.temps[idx] = cur_temp;
            self.probe_samples[idx].push((row[1], row[2]));
        }
        true
    }

    /// The sweep's results (`:617-624`): the average of all nine window
    /// temperatures, then each window's `(sample_temp, average frequency)`
    /// with upstream's log line in front of it.
    ///
    /// # Errors
    /// "Failed calibration - incomplete sensor data" for a window the stream
    /// left empty — upstream divides by its length there and raises.
    fn into_samples(self, bed_z: f64) -> Result<(f64, Vec<(f64, f64)>), CommandError> {
        let sample_temp = self.temps.iter().sum::<f64>() / self.temps.len() as f64;
        let mut windows = Vec::with_capacity(DRIFT_SAMPLE_COUNT);
        for (i, data) in self.probe_samples.iter().enumerate() {
            if data.is_empty() {
                return Err(CommandError::new(
                    "Failed calibration - incomplete sensor data",
                ));
            }
            let avg_freq = data.iter().map(|(freq, _)| freq).sum::<f64>() / data.len() as f64;
            let avg_z = data.iter().map(|(_, z)| z).sum::<f64>() / data.len() as f64;
            let kin_z = i as f64 * 0.5 + 0.05 + bed_z;
            tracing::info!(
                "Probe Values at Temp {sample_temp:.2}C, Z {kin_z:.4}mm: Avg Freq = \
                 {avg_freq:.6}, Avg Measured Z = {avg_z:.6}"
            );
            windows.push((sample_temp, avg_freq));
        }
        Ok((sample_temp, windows))
    }
}

/// A batch message's rows (`{"data": [[time, frequency, z], …]}`), `None`
/// when the message carries no usable `data`.
fn batch_rows(msg: &Value) -> Option<Vec<[f64; 3]>> {
    let rows = msg.get("data")?.as_array()?;
    Some(
        rows.iter()
            .filter_map(|row| {
                let row = row.as_array()?;
                Some([
                    row.first()?.as_f64()?,
                    row.get(1)?.as_f64()?,
                    row.get(2)?.as_f64()?,
                ])
            })
            .collect(),
    )
}

// ===========================================================================
// Loading
// ===========================================================================

/// Upstream's `load_config_prefix` for `[temperature_probe <name>]`
/// (`temperature_probe.py:714-715`): build the sensor object, which registers
/// itself with `heaters` the way upstream's `__init__` does.
///
/// # Errors
/// A malformed option, a `sensor_type` no module knows, or a sensor factory
/// that refuses the section.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let probe = build_temperature_probe(config, printer)?;
    Ok(probe)
}

/// The object [`load_config_prefix`] returns, kept separate so a test can hold
/// the concrete type.
fn build_temperature_probe(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<TemperatureProbe>, ConfigError> {
    let name = config.identifier();
    // Upstream reads the section's options before reaching for `heaters`
    // (`temperature_probe.py:63-97`), so a bad option fails first.
    let options = TemperatureProbeOptions::read(config)?;
    let heaters = heaters::ensure(printer)?;
    let sensor = heaters.setup_sensor(config, printer)?;
    sensor.setup_minmax(options.min_temp, options.max_temp);
    let state = Arc::new(Mutex::new(State::new()));
    sensor.setup_callback(sensor_callback(
        Arc::clone(&state),
        options.inv_smooth_time,
        Arc::downgrade(printer),
    ));
    heaters.register_sensor(config, None, None)?;
    let probe = Arc::new(TemperatureProbe {
        name,
        printer: Arc::downgrade(printer),
        options,
        sensor,
        state: Arc::clone(&state),
        cal_helper: Mutex::new(None),
    });
    // Upstream registers both mux commands in `__init__` (`:112-123`), so a
    // duplicate registration fails at config load rather than at first use.
    probe.register_commands(printer)?;
    // Register the drift helper with the eddy probe (`:125-137`): upstream
    // `load_object`s `probe_eddy_current <pname>`, builds the helper from
    // *this* section's options and hands it over. The loader walks sections by
    // `order` then name — `probe_eddy_current` before `temperature_probe` — so
    // the object is registered by the time this factory runs, which is what
    // upstream's on-demand load guarantees.
    let probe_sect = format!("probe_eddy_current {}", short_name(&probe.name));
    if config.has_sibling(&probe_sect) {
        let pprobe = printer
            .lookup_object_as::<PrinterEddyProbe>(&probe_sect)
            .ok_or_else(|| ConfigError::new(format!("Unknown config object '{probe_sect}'")))?;
        let helper = Arc::new(EddyDriftCompensation::read(
            config,
            Arc::downgrade(printer),
            Arc::clone(&state),
        )?);
        pprobe.register_drift_compensation(Arc::clone(&helper) as Arc<dyn DriftCompensation>);
        tracing::info!(
            "{}: registered drift compensation with probe [{probe_sect}]",
            probe.name
        );
        *probe.cal_helper.lock().unwrap_or_else(|p| p.into_inner()) = Some(helper);
    } else {
        tracing::info!(
            "{}: No probe named {} configured, thermal drift compensation disabled.",
            probe.name,
            short_name(&probe.name)
        );
    }
    Ok(probe)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::probe::{
        PrinterProbe, ProbeCommandState, ProbeOptions, ProbeSessionHelper,
    };
    use crate::core::klippy::extras::toolhead::HomingEndstop;
    use crate::core::klippy::mcu::{ConfigBuilder, McuChip, McuEndstop};
    use crate::core::klippy::pins::{PinParams, PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// A `[temperature_probe <name>]` section with the given options, as the
    /// parser would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("temperature_probe", Some("name"));
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// Read a hand-built section, untracked (no `check_unused` to satisfy).
    fn read(options: &[(&str, &str)]) -> Result<TemperatureProbeOptions, ConfigError> {
        TemperatureProbeOptions::read(&ConfigWrapper::untracked(&section(options)))
    }

    /// The error a hand-built section fails with.
    fn error(options: &[(&str, &str)]) -> String {
        read(options).unwrap_err().to_string()
    }

    /// A sensor whose readings the test delivers by hand.
    #[derive(Default)]
    struct ScriptedSensor(Mutex<Option<heaters::SensorCallback>>);

    impl std::fmt::Debug for ScriptedSensor {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("ScriptedSensor").finish_non_exhaustive()
        }
    }

    impl heaters::Sensor for ScriptedSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}

        fn setup_callback(&self, callback: heaters::SensorCallback) {
            *self.0.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
        }
    }

    impl ScriptedSensor {
        /// Deliver one reading, as the sensor layer would.
        fn read(&self, read_time: f64, temp: f64) {
            let callback = self.0.lock().unwrap_or_else(|p| p.into_inner());
            if let Some(callback) = callback.as_ref() {
                callback(read_time, temp);
            }
        }
    }

    /// A printer with the dispatcher, ready for commands the way `klippy:
    /// ready` makes it, on a reactor the test can advance by hand.
    fn machine() -> (Arc<Printer>, Arc<ManualReactor>, Arc<GCodeDispatch>) {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode is free");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher is registered");
        // The `configfile` object the drift helper's `SAVE_CONFIG` writes go
        // through (`configfile.set`).
        printer
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::new(PrinterConfig::new(
                    AccessTracking::shared(),
                    serde_json::Map::new(),
                )),
            )
            .expect("configfile is free");
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, reactor, gcode)
    }

    /// The `[printer]` section a `kinematics: none` toolhead builds from.
    fn printer_section() -> ConfigSection {
        let mut section = ConfigSection::new("printer", None);
        for (key, value) in [
            ("kinematics", "none"),
            ("max_velocity", "300"),
            ("max_accel", "3000"),
        ] {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        section
    }

    /// A `[probe]` assembled from its parts the way `PrinterProbe::new` does
    /// once the pin layer built the endstop — no machine needed for the
    /// session's command surface (probe.rs's own session tests build one the
    /// same way).
    fn real_z_probe(printer: &Arc<Printer>) -> Arc<PrinterProbe> {
        let mut section = ConfigSection::new("probe", None);
        section
            .parameters
            .insert("pin".to_string(), ConfigValue::Single("PA0".to_string()));
        section.parameters.insert(
            "z_offset".to_string(),
            ConfigValue::Single("1.5".to_string()),
        );
        let config = ConfigWrapper::untracked(&section);
        let options = ProbeOptions::read(&config).expect("the probe options read");
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
        let params = PinParams {
            chip_name: "mcu".to_string(),
            pin: "PA0".to_string(),
            invert: false,
            pullup: 0,
            share_type: None,
        };
        let endstop = Arc::new(McuEndstop::new(chip, &params).expect("the endstop builds"));
        let session = Arc::new(
            ProbeSessionHelper::new(
                &config,
                printer,
                Arc::clone(&endstop) as Arc<dyn HomingEndstop>,
                Arc::clone(&endstop),
                &options,
                None,
            )
            .expect("the session builds"),
        );
        Arc::new(PrinterProbe::from_parts(
            "probe".to_string(),
            options,
            Arc::clone(&endstop),
            session,
            Arc::new(ProbeCommandState::default()),
        ))
    }

    /// Build the section's object on `printer` with a scripted sensor,
    /// returning it with the slot its readings come from.
    fn load_on(
        printer: &Arc<Printer>,
        text: &str,
        section_name: &str,
    ) -> (Arc<TemperatureProbe>, Arc<ScriptedSensor>) {
        let heaters = heaters::ensure(printer).expect("heaters registers");
        let slot = Arc::new(ScriptedSensor::default());
        let sensor = Arc::clone(&slot);
        heaters.add_sensor_factory(
            "Scripted",
            Arc::new(move |_, _| Ok(Arc::clone(&sensor) as Arc<dyn heaters::Sensor>)),
        );

        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config.get_section(section_name).expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(section, Arc::clone(&access));
        let probe = build_temperature_probe(&wrapper, printer).expect("the section loads");
        check_unused(&config, &access, &[]).expect("no option is left unread");
        (probe, slot)
    }

    /// Load a `[temperature_probe name]` from `text` on a printer of its own.
    fn load(text: &str) -> (Arc<TemperatureProbe>, Arc<ScriptedSensor>) {
        let (printer, _, _) = machine();
        load_on(&printer, text, "temperature_probe name")
    }

    /// The minimal section text: `sensor_type` is what `setup_sensor` reads.
    const BASE: &str = "[temperature_probe name]\nsensor_type: Scripted\n";

    // --- Polynomial2d ------------------------------------------------------

    /// The evaluation is upstream's `__call__`: `c·x² + b·x + a`.
    #[test]
    fn polynomial_evaluates_its_quadratic() {
        let poly = Polynomial2d::new(1.0, -3.0, 2.0);
        assert_eq!(poly.get_coefs(), (1.0, -3.0, 2.0));
        // 2·0² - 3·0 + 1
        assert_eq!(poly.eval(0.0), 1.0);
        // 2·2² - 3·2 + 1
        assert_eq!(poly.eval(2.0), 3.0);
        // 2·(-1)² - 3·(-1) + 1
        assert_eq!(poly.eval(-1.0), 6.0);
    }

    /// `fit` recovers the coefficients of an exact quadratic, upstream's
    /// `Polynomial2d.fit` through `mathutil.solve_linear_equations`.
    #[test]
    fn polynomial_fit_recovers_the_coefficients() {
        let f = |x: f64| 4.0 - 1.5 * x + 0.25 * x * x;
        let coords: Vec<(f64, f64)> = [-3.0, -1.0, 0.0, 2.0, 5.0]
            .into_iter()
            .map(|x| (x, f(x)))
            .collect();
        let fitted = Polynomial2d::fit(&coords).expect("five points fit");
        let [a, b, c] = [fitted.a, fitted.b, fitted.c];
        for (coef, want) in [a, b, c].into_iter().zip([4.0, -1.5, 0.25]) {
            assert!((coef - want).abs() < 1e-9, "{coef} vs {want}");
        }
        // Every point sits back on the fitted curve.
        for (x, y) in coords {
            assert!((fitted.eval(x) - y).abs() < 1e-9, "at {x}");
        }
    }

    /// With no points there is nothing to fit — upstream's solve indexes the
    /// empty system and raises; here it is `None`.
    #[test]
    fn polynomial_fit_without_points_has_no_answer() {
        assert!(Polynomial2d::fit(&[]).is_none());
    }

    /// `__str__` is `"%f, %f, %f"` and `__repr__` is the `y(x) = …` form,
    /// skipping terms that round away (`temperature_probe.py:28-47`).
    #[test]
    fn polynomial_formats_upstreams_way() {
        let poly = Polynomial2d::new(1.5, -2.25, 3.0);
        assert_eq!(poly.to_string(), "1.500000, -2.250000, 3.000000");
        assert_eq!(
            format!("{poly:?}"),
            "y(x) = 3.000000x^2 - 2.250000x + 1.500000"
        );
        // A zero linear term drops out; a constant term carries no `x`.
        assert_eq!(
            format!("{:?}", Polynomial2d::new(5.0, 0.0, -2.0)),
            "y(x) = -2.000000x^2 + 5.000000"
        );
        assert_eq!(
            format!("{:?}", Polynomial2d::new(0.0, -3.0, 0.0)),
            "y(x) = -3.000000x"
        );
    }

    // --- Option parsing ----------------------------------------------------

    /// The bare section reads upstream's defaults
    /// (`temperature_probe.py:63-97`).
    #[test]
    fn the_bare_section_reads_upstreams_defaults() {
        let options = read(&[]).expect("the bare section reads");

        assert_eq!(options.speed, None);
        assert_eq!(options.horizontal_move_z, 2.);
        assert_eq!(options.resting_z, 0.4);
        assert_eq!(options.cal_pos, None);
        assert_eq!(options.cal_bed_temp, None);
        assert_eq!(options.cal_extruder_temp, None);
        assert_eq!(options.cal_extruder_z, 50.);
        assert_eq!(options.inv_smooth_time, 1. / 2.);
        assert_eq!(options.min_temp, KELVIN_TO_CELSIUS);
        assert_eq!(options.max_temp, 99999999.9);
    }

    /// Every option the section writes is read back through the tracker, so
    /// `check_unused` accepts the section — including the two `heaters` reads
    /// (`sensor_type`, `gcode_id`), which is why this loads the object.
    #[test]
    fn every_option_the_section_writes_is_recorded_as_read() {
        let text = "\
[temperature_probe name]
sensor_type: Scripted
speed: 50
horizontal_move_z: 3
resting_z: 0.5
calibration_position: 100, 100, 5
calibration_bed_temp: 60
calibration_extruder_temp: 150
extruder_heating_z: 40
smooth_time: 1.5
min_temp: -100
max_temp: 300
gcode_id: T0
";
        let (printer, _, _) = machine();
        let heaters = heaters::ensure(&printer).expect("heaters registers");
        heaters.add_sensor_factory(
            "Scripted",
            Arc::new(|_, _| Ok(Arc::new(ScriptedSensor::default()) as Arc<dyn heaters::Sensor>)),
        );

        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .get_section("temperature_probe name")
            .expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(section, Arc::clone(&access));
        let probe = build_temperature_probe(&wrapper, &printer).expect("the section loads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(probe.options.speed, Some(50.));
        assert_eq!(probe.options.horizontal_move_z, 3.);
        assert_eq!(probe.options.resting_z, 0.5);
        assert_eq!(probe.options.cal_pos, Some(vec![100., 100., 5.]));
        assert_eq!(probe.options.cal_bed_temp, Some(60.));
        assert_eq!(probe.options.cal_extruder_temp, Some(150.));
        assert_eq!(probe.options.cal_extruder_z, 40.);
        assert_eq!(probe.options.inv_smooth_time, 1. / 1.5);
        assert_eq!(probe.options.min_temp, -100.);
        assert_eq!(probe.options.max_temp, 300.);
    }

    /// Each bounded option keeps upstream's bound wording
    /// (`klippy/configfile.py:54-56`), printed the way this port's
    /// `get_float_bounded` prints a limit.
    #[test]
    fn bounded_options_keep_upstreams_bound_wording() {
        assert_eq!(
            error(&[("speed", "0")]),
            "Option 'speed' in section 'temperature_probe name' must be above 0"
        );
        assert_eq!(
            error(&[("horizontal_move_z", "-1")]),
            "Option 'horizontal_move_z' in section 'temperature_probe name' must be above 0"
        );
        assert_eq!(
            error(&[("resting_z", "0")]),
            "Option 'resting_z' in section 'temperature_probe name' must be above 0"
        );
        assert_eq!(
            error(&[("calibration_bed_temp", "50")]),
            "Option 'calibration_bed_temp' in section 'temperature_probe name' must be above 50"
        );
        assert_eq!(
            error(&[("calibration_extruder_temp", "40")]),
            "Option 'calibration_extruder_temp' in section 'temperature_probe name' must be above 50"
        );
        assert_eq!(
            error(&[("extruder_heating_z", "0")]),
            "Option 'extruder_heating_z' in section 'temperature_probe name' must be above 0"
        );
        assert_eq!(
            error(&[("smooth_time", "0")]),
            "Option 'smooth_time' in section 'temperature_probe name' must be above 0"
        );
        assert_eq!(
            error(&[("min_temp", "-300")]),
            "Option 'min_temp' in section 'temperature_probe name' must have minimum of -273.15"
        );
        // `max_temp` is checked against the configured `min_temp`, not its own
        // default (`temperature_probe.py:87-88`).
        assert_eq!(
            error(&[("min_temp", "200"), ("max_temp", "100")]),
            "Option 'max_temp' in section 'temperature_probe name' must be above 200"
        );
    }

    /// `calibration_position` is `getfloatlist(…, count=3)`: every item parses
    /// first, then the count is checked (`klippy/configfile.py:98-101`).
    #[test]
    fn calibration_position_keeps_upstreams_count_wording() {
        assert_eq!(
            error(&[("calibration_position", "100, 100")]),
            "Option 'calibration_position' in section 'temperature_probe name' must have 3 elements"
        );
        assert_eq!(
            error(&[("calibration_position", "left, 100, 5")]),
            "Unable to parse option 'calibration_position' in section 'temperature_probe name'"
        );
        // A blank value is an empty list upstream, which then fails the count.
        assert_eq!(
            error(&[("calibration_position", "")]),
            "Option 'calibration_position' in section 'temperature_probe name' must have 3 elements"
        );
    }

    // --- Readings and status ---------------------------------------------

    /// `_temp_callback`'s smoothing (`temperature_probe.py:141-149`): the first
    /// reading arrives against a zero clock (no movement), each later one moves
    /// the smoothed value by `time_diff / smooth_time`, capped at a full step.
    #[test]
    fn readings_smooth_the_way_upstream_does() {
        let (probe, sensor) = load(BASE);

        // time_diff 0 → adj_time 0: the smoothed value does not move.
        sensor.read(0., 100.);
        // time_diff 1, smooth_time 2 → half the gap.
        sensor.read(1., 100.);
        assert_eq!(probe.get_temp(), (50., 0.));
        // Another half of the remaining gap.
        sensor.read(2., 100.);
        assert_eq!(probe.get_temp(), (75., 0.));
        // A long silence caps the adjustment at a full step: straight to the
        // new reading (`min(time_diff * inv_smooth_time, 1.)`).
        sensor.read(1000., 42.);
        assert_eq!(probe.get_temp(), (42., 0.));
    }

    /// `get_status` reports upstream's six keys (`:453-465`): `temperature`
    /// raw, the measured extremes rounded to two places, and the calibration
    /// flags at their starting values.
    #[test]
    fn get_status_reports_upstreams_shape() {
        let (probe, sensor) = load(BASE);

        sensor.read(0., 100.);
        sensor.read(1., 100.);
        sensor.read(2., 80.);

        let status = probe.get_status(0.0);
        assert_eq!(
            status,
            json!({
                // Half a `smooth_time` of every step from 0: 0 → 50 toward
                // 100, then 50 → 65 toward 80.
                "temperature": 65.0,
                "measured_min_temp": 0.0,
                "measured_max_temp": 65.0,
                "in_calibration": false,
                "estimated_expansion": 0.0,
                "compensation_enabled": false,
            })
        );
        assert!(!probe.is_in_calibration());
    }

    /// The `stats` line is upstream's `'%s: temp=%.1f'`
    /// (`temperature_probe.py:467-468`).
    #[test]
    fn stats_is_upstreams_line() {
        let (probe, sensor) = load(BASE);
        // A long gap caps the adjustment at a full step: the reading lands as
        // given.
        sensor.read(1000., 41.);

        assert_eq!(
            probe.stats(),
            (false, "temperature_probe name: temp=41.0".to_string())
        );
    }

    /// The loader hands back the object under its own section name.
    #[test]
    fn the_loader_registers_the_object_under_its_section_name() {
        let (printer, _, _) = machine();
        let heaters = heaters::ensure(&printer).expect("heaters registers");
        heaters.add_sensor_factory(
            "Scripted",
            Arc::new(|_, _| Ok(Arc::new(ScriptedSensor::default()) as Arc<dyn heaters::Sensor>)),
        );

        let (config, _) = Config::from_text(BASE).expect("the test config parses");
        let section = config
            .get_section("temperature_probe name")
            .expect("the section");
        let wrapper = ConfigWrapper::untracked(section);
        let object = load_config_prefix(&wrapper, &printer).expect("the section loads");

        printer
            .add_object("temperature_probe name", object)
            .expect("the name is free");
        let found = printer
            .lookup_object_as::<TemperatureProbe>("temperature_probe name")
            .expect("the object is there");
        assert_eq!(found.name, "temperature_probe name");
    }

    // --- The command family ------------------------------------------------

    /// A machine the command family runs on: the dispatcher, a
    /// `kinematics: none` toolhead, the `manual_probe` object, a real
    /// `[probe]`, and the `[temperature_probe …]` section loaded on top. Every
    /// line the dispatcher emits is kept in `log`.
    struct Machine {
        /// Held so the `Weak<Printer>` the section keeps stays alive.
        #[allow(dead_code)]
        printer: Arc<Printer>,
        /// The clock the sensor callback arms `call_later` on.
        reactor: Arc<ManualReactor>,
        /// The dispatcher every command runs through.
        gcode: Arc<GCodeDispatch>,
        /// The section under test.
        probe: Arc<TemperatureProbe>,
        /// Where its readings are delivered from.
        sensor: Arc<ScriptedSensor>,
        /// The lines the dispatcher has emitted.
        log: Arc<Mutex<Vec<String>>>,
    }

    impl Machine {
        /// The standard machine: a connected toolhead and a `[probe]`.
        async fn new(section_name: &str) -> Self {
            let text = format!("[{section_name}]\nsensor_type: Scripted\n");
            Self::build(&text, section_name, true, true).await
        }

        /// The standard machine with the section's own options; `connect` says
        /// whether the toolhead has a planner (a toolhead before `connect` has
        /// no position to move from).
        async fn with_options(text: &str, section_name: &str, connect: bool) -> Self {
            Self::build(text, section_name, connect, true).await
        }

        /// The standard machine without a `probe` object, so `_get_probe` has
        /// nothing to find.
        async fn without_probe(section_name: &str) -> Self {
            let text = format!("[{section_name}]\nsensor_type: Scripted\n");
            Self::build(&text, section_name, true, false).await
        }

        async fn build(text: &str, section_name: &str, connect: bool, with_probe: bool) -> Self {
            let (printer, reactor, gcode) = machine();
            let log = Arc::new(Mutex::new(Vec::new()));
            {
                let log = Arc::clone(&log);
                gcode.register_output_handler(Arc::new(move |line: &str| {
                    log.lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .push(line.to_string());
                }));
            }
            printer
                .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
                .expect("pins is free");
            let toolhead = Arc::new(
                ToolHeadObject::new(&ConfigWrapper::untracked(&printer_section()), &printer)
                    .expect("kinematics: none builds"),
            );
            if connect {
                toolhead.connect().await.expect("the toolhead connects");
            }
            printer
                .add_object(TOOLHEAD_OBJECT, toolhead)
                .expect("toolhead is free");
            manual_probe::ensure(&printer, &ConfigWrapper::untracked(&printer_section()))
                .expect("manual_probe");
            if with_probe {
                printer
                    .add_object(PROBE_OBJECT, real_z_probe(&printer))
                    .expect("probe is free");
            }
            let (probe, sensor) = load_on(&printer, text, section_name);
            Self {
                printer,
                reactor,
                gcode,
                probe,
                sensor,
                log,
            }
        }

        /// Run one line through the dispatcher.
        async fn run(&self, line: &str) -> Result<(), CommandError> {
            self.gcode.run_script(line).await
        }

        /// A command with the given parameters, as the dispatcher would hand
        /// it to a handler.
        fn command(&self, name: &str, params: &[(&str, &str)]) -> GcodeCommand {
            let mut words = HashMap::new();
            let mut line = name.to_string();
            for (key, value) in params {
                words.insert((*key).to_string(), (*value).to_string());
                line.push_str(&format!(" {key}={value}"));
            }
            self.gcode.create_gcode_command(name, &line, words)
        }

        /// Every line the dispatcher has emitted.
        fn replies(&self) -> Vec<String> {
            self.log.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }

        /// Let a spawned continuation run to completion (`ManualReactor` runs
        /// no tasks, so the test drives the runtime itself).
        async fn settle() {
            for _ in 0..16 {
                tokio::task::yield_now().await;
            }
        }
    }

    /// The section registers both mux commands under its probe name with
    /// upstream's help, and the temporary trio waits for a calibration
    /// (`temperature_probe.py:112-123, :372-381`).
    #[tokio::test]
    async fn the_commands_are_registered_upstreams_way() {
        let m = Machine::new("temperature_probe probe").await;
        let help = m.gcode.command_help();
        assert_eq!(
            help.get("TEMPERATURE_PROBE_CALIBRATE").map(String::as_str),
            Some("Calibrate probe temperature drift compensation")
        );
        assert_eq!(
            help.get("TEMPERATURE_PROBE_ENABLE").map(String::as_str),
            Some("Set adjustment factor applied to drift correction")
        );
        for name in [
            "ABORT",
            "TEMPERATURE_PROBE_NEXT",
            "TEMPERATURE_PROBE_COMPLETE",
        ] {
            assert!(
                !m.gcode.command_exists(name),
                "{name} waits for a calibration"
            );
        }
        // `PROBE=probe` is the mux value the section registered, and the
        // handler stops at upstream's helper gate first (`:355-358`) — `METHOD`
        // has already been read with its default (`:354`).
        let err = m
            .run("TEMPERATURE_PROBE_CALIBRATE PROBE=probe")
            .await
            .expect_err("the helper gate refuses");
        assert_eq!(
            err.to_string(),
            "No calibration helper registered for [temperature_probe probe]"
        );
        // With a helper registered the homed gate is next (`:359`) — the axes
        // of a `kinematics: none` machine are never homed.
        drift_helper(&m.printer, &m.probe, &[]);
        let err = m
            .run("TEMPERATURE_PROBE_CALIBRATE PROBE=probe")
            .await
            .expect_err("the homed gate refuses");
        assert_eq!(err.to_string(), "Printer must be homed before calibration");
        // Both gates come before TARGET is read (`:354-359`), so a command
        // with no TARGET reports the gate, not the missing value.
        assert!(
            !m.replies()
                .iter()
                .any(|line| line.contains("missing TARGET")),
            "TARGET is read after the gates: {:?}",
            m.replies()
        );
    }

    /// `_get_probe`'s refusal (`:259-263`) and the link check that follows it
    /// (`:345-352`).
    #[tokio::test]
    async fn calibrate_refuses_a_missing_or_unlinked_probe() {
        // No probe at all.
        let m = Machine::without_probe("temperature_probe probe").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("no probe");
        assert_eq!(err.to_string(), "No probe configured");

        // A section the probe is not named after.
        let m = Machine::new("temperature_probe name").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("not linked");
        assert_eq!(
            err.to_string(),
            "[temperature_probe name] not linked to registered probe [probe]."
        );
    }

    /// A running manual probe refuses the calibration (`:354`).
    #[tokio::test]
    async fn calibrate_refuses_while_a_manual_probe_runs() {
        let m = Machine::new("temperature_probe probe").await;
        m.run("MANUAL_PROBE")
            .await
            .expect("the manual probe starts");
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("busy");
        assert_eq!(
            err.to_string(),
            "Already in a manual Z probe. Use ABORT to abort it."
        );
        // Nothing of the calibration was set up behind the refusal.
        assert!(!m.probe.is_in_calibration());
        assert!(!m.gcode.command_exists("TEMPERATURE_PROBE_NEXT"));
    }

    /// `TARGET`, `STEP` and the sample count keep upstream's wording
    /// (`:361-371`).
    #[tokio::test]
    async fn calibrate_validates_target_step_and_sample_count() {
        let m = Machine::new("temperature_probe probe").await;
        // The sensor starts at 0 °C, so a TARGET at or below it is refused.
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "0"), ("STEP", "2")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("at the current temperature");
        assert_eq!(
            err.to_string(),
            "Error on 'TEMPERATURE_PROBE_CALIBRATE TARGET=0 STEP=2': TARGET must be above 0.0"
        );

        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "0.5")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("a step below the minimum");
        assert_eq!(
            err.to_string(),
            "Error on 'TEMPERATURE_PROBE_CALIBRATE TARGET=10 STEP=0.5': STEP must have minimum of 1"
        );

        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "1"), ("STEP", "2")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("too few samples");
        assert_eq!(
            err.to_string(),
            "Invalid STEP and/or TARGET parameters resulted in too few expected samples: 1"
        );
    }

    /// Taken temporary commands are refused with upstream's text, at the same
    /// point of the flow (`:372-386`).
    #[tokio::test]
    async fn calibrate_refuses_when_the_temporary_commands_are_taken() {
        let m = Machine::new("temperature_probe probe").await;
        m.gcode
            .register_command("TEMPERATURE_PROBE_NEXT", sync(|_| Ok(())), None, false)
            .expect("registered");
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("taken");
        assert_eq!(
            err.to_string(),
            "Auxiliary Probe Drift Commands already registered. Use TEMPERATURE_PROBE_COMPLETE or ABORT to exit."
        );
        // Upstream's `try` leaves the command it managed to register standing,
        // and nothing of the calibration started.
        assert!(m.gcode.command_exists("TEMPERATURE_PROBE_NEXT"));
        assert!(!m.probe.is_in_calibration());
    }

    /// The body behind the gate: register the two temporary commands, arm the
    /// state machine and hand the first sample to the interactive probe
    /// (`:387-408`).
    #[tokio::test]
    async fn calibrate_starts_the_state_machine_and_the_manual_probe() {
        let m = Machine::new("temperature_probe probe").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        m.probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect("the calibration starts");

        assert!(m.gcode.command_exists("TEMPERATURE_PROBE_NEXT"));
        assert!(m.gcode.command_exists("TEMPERATURE_PROBE_COMPLETE"));
        // Upstream registers NEXT's help for COMPLETE too (`:377-381`).
        assert_eq!(
            m.gcode
                .command_help()
                .get("TEMPERATURE_PROBE_COMPLETE")
                .map(String::as_str),
            Some("Sample next probe drift temperature")
        );
        // The manual probe took the interactive commands over (`:405-408`).
        for name in ["ACCEPT", "NEXT", "TESTZ", "ABORT"] {
            assert!(m.gcode.command_exists(name), "{name} is the manual probe's");
        }
        let status = m.probe.get_status(0.0);
        assert_eq!(status["in_calibration"], json!(true));
        assert!(
            m.replies().iter().any(|line| line
                .contains("Starting manual Z probe. Use TESTZ to adjust position.")),
            "{:?}",
            m.replies()
        );
        // What the command wrote (`:387-392`).
        let state = lock_state(&m.probe.state);
        assert_eq!(state.method, "manual");
        assert_eq!(state.target_temp, 10.);
        assert_eq!(state.step, 2.);
        assert_eq!(state.sample_count, 0);
        assert_eq!(state.expected_count, 5);
        assert!(state.start_pos.is_some());
    }

    /// One sample round: `ACCEPT` takes the sample and schedules the next
    /// temperature, and the reading reaching it kicks
    /// `TEMPERATURE_PROBE_NEXT`, which opens the following round
    /// (`:155-159, :179-197, :410-433`).
    #[tokio::test]
    async fn a_sample_cycle_kicks_the_next_one_when_the_temperature_rises() {
        let m = Machine::new("temperature_probe probe").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        m.probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect("the calibration starts");

        // The interactive part: move the nozzle down and accept it.
        m.run("TESTZ Z=-0.5").await.expect("the manual probe moves");
        m.run("ACCEPT").await.expect("the sample is accepted");
        Machine::settle().await;

        assert!(
            m.replies().iter().any(|line| line.contains(
                "temperature_probe probe: collected sample 1/5 at temp 0.00C, \
                 next sample scheduled at temp 2.00C"
            )),
            "{:?}",
            m.replies()
        );
        // The finished manual probe handed `ABORT` back to the calibration
        // (`:182-186`), and the helper's own commands are gone.
        assert!(m.gcode.command_exists("ABORT"));
        assert_eq!(
            m.gcode.command_help().get("ABORT").map(String::as_str),
            Some("Abort Probe Drift Calibration")
        );
        assert!(!m.gcode.command_exists("ACCEPT"));
        assert_eq!(lock_state(&m.probe.state).next_auto_temp, 2.);
        assert!(m.probe.is_in_calibration());

        // The reading reaches the scheduled temperature: the sensor callback
        // arms `_check_kick_next`, which runs `TEMPERATURE_PROBE_NEXT`.
        m.sensor.read(1000., 5.);
        m.reactor.advance(1.0);
        Machine::settle().await;

        // The next round is open again. It can only be: `cmd_TEMPERATURE_PROBE_NEXT`
        // unregisters `ABORT` before the helper registers its own, so a
        // leftover would have made the helper fail to start.
        assert!(m.gcode.command_exists("ACCEPT"));
        assert!(m.gcode.command_exists("TESTZ"));
        assert!(
            m.replies()
                .iter()
                .any(|line| line.contains("Starting manual Z probe.")),
            "{:?}",
            m.replies()
        );
        assert!(m.probe.is_in_calibration());
    }

    /// `TEMPERATURE_PROBE_COMPLETE` closes a short run as an abort and takes
    /// the temporary commands with it (`:435-437`, `:233-257`).
    #[tokio::test]
    async fn complete_short_of_three_samples_aborts_the_calibration() {
        let m = Machine::new("temperature_probe probe").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        m.probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect("the calibration starts");
        m.run("TESTZ Z=-0.5").await.expect("the manual probe moves");
        m.run("ACCEPT").await.expect("the sample is accepted");
        Machine::settle().await;

        m.run("TEMPERATURE_PROBE_COMPLETE")
            .await
            .expect("the command runs");
        assert!(
            m.replies()
                .iter()
                .any(|line| line.contains("temperature_probe probe: calibration aborted")),
            "{:?}",
            m.replies()
        );
        for name in [
            "ABORT",
            "TEMPERATURE_PROBE_NEXT",
            "TEMPERATURE_PROBE_COMPLETE",
        ] {
            assert!(!m.gcode.command_exists(name), "{name} is gone");
        }
        let status = m.probe.get_status(0.0);
        assert_eq!(status["in_calibration"], json!(false));
        assert_eq!(status["estimated_expansion"], json!(0.0));
    }

    /// A run with three samples or more completes silently (`:435-437`).
    #[tokio::test]
    async fn complete_with_three_samples_finishes_the_calibration() {
        let m = Machine::new("temperature_probe probe").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        m.probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect("the calibration starts");
        m.run("TESTZ Z=-0.5").await.expect("the manual probe moves");
        m.run("ACCEPT").await.expect("the sample is accepted");
        Machine::settle().await;
        // Three samples collected — the count `cmd_TEMPERATURE_PROBE_COMPLETE`
        // needs (`sample_count >= 3`).
        lock_state(&m.probe.state).sample_count = 3;

        m.run("TEMPERATURE_PROBE_COMPLETE")
            .await
            .expect("the command runs");
        assert!(
            !m.replies()
                .iter()
                .any(|line| line.contains("calibration aborted")),
            "{:?}",
            m.replies()
        );
        assert!(!m.gcode.command_exists("TEMPERATURE_PROBE_NEXT"));
        assert!(!m.probe.is_in_calibration());
    }

    /// The `ABORT` the calibration registers between samples takes the run
    /// down (`:440-441`): upstream's `cmd_TEMPERATURE_PROBE_ABORT` answers as
    /// `ABORT`, the name `_prepare_next_sample` registers it under (`:182-186`).
    #[tokio::test]
    async fn abort_takes_the_calibration_down() {
        let m = Machine::new("temperature_probe probe").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        m.probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect("the calibration starts");
        m.run("TESTZ Z=-0.5").await.expect("the manual probe moves");
        m.run("ACCEPT").await.expect("the sample is accepted");
        Machine::settle().await;

        m.run("ABORT").await.expect("the command runs");
        assert!(
            m.replies()
                .iter()
                .any(|line| line.contains("temperature_probe probe: calibration aborted")),
            "{:?}",
            m.replies()
        );
        for name in [
            "ABORT",
            "TEMPERATURE_PROBE_NEXT",
            "TEMPERATURE_PROBE_COMPLETE",
        ] {
            assert!(!m.gcode.command_exists(name), "{name} is gone");
        }
        assert!(!m.probe.is_in_calibration());
    }

    /// An initial move that fails aborts the calibration where upstream does
    /// (`:394-398`): the run is reported, taken down, and the failure itself
    /// is what the command returns.
    #[tokio::test]
    async fn a_failed_initial_move_aborts_the_calibration() {
        let text = "\
[temperature_probe probe]
sensor_type: Scripted
calibration_position: 100, 100, 5
";
        // A toolhead before `connect` has no position to move from.
        let m = Machine::with_options(text, "temperature_probe probe", false).await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        let err = m
            .probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect_err("the move refuses");
        assert_eq!(err.to_string(), "Printer is not ready");
        assert!(
            m.replies()
                .iter()
                .any(|line| line.contains("Error during initial move")),
            "{:?}",
            m.replies()
        );
        assert!(!m.gcode.command_exists("TEMPERATURE_PROBE_NEXT"));
        assert!(!m.gcode.command_exists("TEMPERATURE_PROBE_COMPLETE"));
        assert!(!m.probe.is_in_calibration());
    }

    /// Two samples: the Z the second one accepts becomes the expansion
    /// estimate, `TEMPERATURE_PROBE_NEXT` opens the round by hand, and
    /// `ABORT` puts the status back (`:200-210, :410-433, :440-441`).
    #[tokio::test]
    async fn the_expansion_estimate_accumulates_and_resets() {
        let m = Machine::new("temperature_probe probe").await;
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        m.probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect("the calibration starts");

        // First sample: there is nothing to compare the Z against yet.
        m.run("TESTZ Z=-0.5").await.expect("the manual probe moves");
        m.run("ACCEPT").await.expect("the first sample is accepted");
        Machine::settle().await;
        assert_eq!(m.probe.get_status(0.0)["estimated_expansion"], json!(0.0));

        // The temperature never reached the schedule, so the next round is
        // started by hand — upstream's `TEMPERATURE_PROBE_NEXT`.
        let next = m.command("TEMPERATURE_PROBE_NEXT", &[]);
        m.probe
            .cmd_next(&next)
            .await
            .expect("the next round starts");
        // `TESTZ` is a relative move (`manual_probe.py:280`), and the nozzle
        // sits at the resting Z: -0.5 + 0.4 = -0.1. Six tenths down puts the
        // second sample at -0.7.
        m.run("TESTZ Z=-0.6").await.expect("the manual probe moves");
        m.run("ACCEPT")
            .await
            .expect("the second sample is accepted");
        Machine::settle().await;

        // `last_zero_pos - bed_z` = -0.5 - (-0.7) (`:204-208`).
        let expansion = m.probe.get_status(0.0)["estimated_expansion"]
            .as_f64()
            .expect("a number");
        assert!((expansion - 0.2).abs() < 1e-9, "{expansion}");

        // `ABORT` is the calibration's own again (the helper gave it back on
        // `ACCEPT`) and the finalize cleared the estimate with the rest.
        m.run("ABORT").await.expect("the command runs");
        assert_eq!(m.probe.get_status(0.0)["in_calibration"], json!(false));
        assert_eq!(m.probe.get_status(0.0)["estimated_expansion"], json!(0.0));
        assert!(!m.gcode.command_exists("TEMPERATURE_PROBE_NEXT"));
    }

    /// The heater scripts are upstream's strings, `%f` and all
    /// (`temperature_probe.py:265-287`), and with no calibration temperature
    /// configured neither runs.
    #[tokio::test]
    async fn the_heater_scripts_are_upstreams_verbatim() {
        let text = "\
[temperature_probe probe]
sensor_type: Scripted
calibration_extruder_temp: 150
calibration_bed_temp: 60
";
        let m = Machine::with_options(text, "temperature_probe probe", true).await;
        let lines = Arc::new(Mutex::new(Vec::new()));
        for (cmd, key, value) in [
            ("SET_HEATER_TEMPERATURE", "HEATER", "extruder"),
            ("SET_HEATER_TEMPERATURE", "HEATER", "heater_bed"),
            ("TEMPERATURE_WAIT", "SENSOR", "extruder"),
        ] {
            let lines = Arc::clone(&lines);
            m.gcode
                .register_mux_command_with_params(
                    cmd,
                    key,
                    Some(value),
                    sync(move |gcmd| {
                        lines
                            .lock()
                            .unwrap_or_else(|p| p.into_inner())
                            .push(gcmd.commandline().to_string());
                        Ok(())
                    }),
                    None,
                    &[],
                )
                .expect("the value is free");
        }

        m.probe
            .set_extruder_temp(150., true)
            .await
            .expect("the heater script runs");
        m.probe
            .set_bed_temp(60.)
            .await
            .expect("the bed script runs");
        assert_eq!(
            *lines.lock().unwrap_or_else(|p| p.into_inner()),
            vec![
                "SET_HEATER_TEMPERATURE HEATER=extruder TARGET=150.000000",
                "TEMPERATURE_WAIT SENSOR=extruder MINIMUM=150.000000",
                "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=60.000000",
            ]
        );
    }

    /// `TEMPERATURE_PROBE_ENABLE` takes the parameter and leaves the status
    /// alone: upstream's run with no drift helper registered (`:446-448`).
    #[tokio::test]
    async fn enable_is_the_no_op_upstream_runs_without_a_helper() {
        let m = Machine::new("temperature_probe probe").await;
        m.run("TEMPERATURE_PROBE_ENABLE PROBE=probe ENABLE=1")
            .await
            .expect("the command runs");
        assert_eq!(
            m.probe.get_status(0.0)["compensation_enabled"],
            json!(false)
        );
    }

    // --- The drift helper ------------------------------------------------

    /// The drift options of a correction that can work: three curves that
    /// never cross (`pᵢ(T) = aᵢ`) and a saved calibration temperature to
    /// correct toward.
    const USABLE_DRIFT: &[(&str, &str)] = &[
        ("calibration_temp", "25"),
        ("drift_calibration", "300, 0, 0\n200, 0, 0\n100, 0, 0"),
    ];

    /// A helper read from a hand-built section against a fresh sensor state —
    /// `EddyDriftCompensation::read` alone, as the wiring calls it, with no
    /// probe to register it with.
    fn read_drift(
        printer: &Arc<Printer>,
        options: &[(&str, &str)],
    ) -> (Arc<EddyDriftCompensation>, Arc<Mutex<State>>) {
        let section = section(options);
        let state = Arc::new(Mutex::new(State::new()));
        let helper = Arc::new(
            EddyDriftCompensation::read(
                &ConfigWrapper::untracked(&section),
                Arc::downgrade(printer),
                Arc::clone(&state),
            )
            .expect("the drift options read"),
        );
        (helper, state)
    }

    /// The same, installed the way the wiring installs it: built from a
    /// section named after `probe`'s own, dropped into `cal_helper`.
    fn drift_helper(
        printer: &Arc<Printer>,
        probe: &Arc<TemperatureProbe>,
        options: &[(&str, &str)],
    ) -> Arc<EddyDriftCompensation> {
        let mut built = ConfigSection::new("temperature_probe", Some(short_name(&probe.name)));
        for (option, value) in options {
            built.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        let helper = Arc::new(
            EddyDriftCompensation::read(
                &ConfigWrapper::untracked(&built),
                Arc::downgrade(printer),
                Arc::clone(&probe.state),
            )
            .expect("the drift options read"),
        );
        *probe.cal_helper.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::clone(&helper));
        helper
    }

    /// The four options read with upstream's defaults, and `enabled` exactly
    /// where upstream leaves it (`temperature_probe.py:481-525`).
    #[test]
    fn drift_options_read_upstreams_way() {
        let (printer, _, _) = machine();

        // No curves: nothing to correct with, and `min_freq` keeps its
        // starting sentinel (`:486`).
        let (helper, _) = read_drift(&printer, &[]);
        assert!(!helper.is_enabled());
        assert!(helper.lock().drift_calibration.is_none());
        assert_eq!(helper.lock().min_freq, 999999999999.);
        assert_eq!(helper.lock().cal_temp, 0.);
        assert_eq!(helper.lock().max_valid_temp, 60.);

        // Curves plus a saved calibration temperature: usable, and `min_freq`
        // is the lowest curve's minimum over `range(121)` (`:498-499`) — a
        // constant 100 everywhere.
        let (helper, _) = read_drift(&printer, USABLE_DRIFT);
        assert!(helper.is_enabled());
        assert_eq!(helper.lock().min_freq, 100.);
        assert_eq!(helper.lock().cal_temp, 25.);
        assert_eq!(
            helper.lock().drift_calibration.as_ref().map(Vec::len),
            Some(3)
        );

        // Curves but no saved temperature: loaded, then switched off again
        // (`:517-523`).
        let (helper, _) = read_drift(
            &printer,
            &[("drift_calibration", "300, 0, 0\n200, 0, 0\n100, 0, 0")],
        );
        assert!(!helper.is_enabled());
        assert!(helper.lock().drift_calibration.is_some());
    }

    /// The load-time refusals: parse first, then the per-curve length, then
    /// the crossing check — upstream's order and wording (`:488-496`).
    #[test]
    fn drift_calibration_refuses_what_upstream_refuses() {
        let (printer, _, _) = machine();
        let read = |options: &[(&str, &str)]| {
            let section = section(options);
            EddyDriftCompensation::read(
                &ConfigWrapper::untracked(&section),
                Arc::downgrade(&printer),
                Arc::new(Mutex::new(State::new())),
            )
            .map(|_| ())
        };

        // The whole option parses before any curve's length is checked
        // (`:488-490`): a bad number in the second curve wins.
        let err = read(&[("drift_calibration", "300, 0, 0\n150, x, 0")])
            .expect_err("a coefficient does not parse");
        assert_eq!(
            err.to_string(),
            "Unable to parse option 'drift_calibration' in section 'temperature_probe name'"
        );
        // A stray separator is a non-number too (upstream does not filter
        // empty items at the inner level).
        let err = read(&[("drift_calibration", "300, 0,")]).expect_err("trailing separator");
        assert_eq!(
            err.to_string(),
            "Unable to parse option 'drift_calibration' in section 'temperature_probe name'"
        );

        // Not three coefficients (`:492-495`).
        let err = read(&[("drift_calibration", "300, 0")]).expect_err("two coefficients");
        assert_eq!(err.to_string(), "Invalid polynomial in drift calibration");

        // Adjacent curves must not meet over `drift_calibration_min_temp ..
        // max_validation_temp` (`:657-673`): `200 - T` reaches the constant
        // 150 exactly at 50 °C.
        let err =
            read(&[("drift_calibration", "200, -1, 0\n150, 0, 0")]).expect_err("the curves cross");
        assert_eq!(
            err.to_string(),
            "temperature_probe name: invalid calibration detected, curve at index 1 overlaps \
             previous curve at temp 50C."
        );
    }

    /// `adjust_freq` / `unadjust_freq` through `_calc_freq`: both gates, all
    /// three interpolation branches, and the round trip
    /// (`temperature_probe.py:674-710`).
    #[test]
    fn adjust_and_unadjust_walk_the_curves_upstreams_way() {
        let (printer, _, _) = machine();
        // Parallel curves `pᵢ(T) = aᵢ - T`, so every branch moves.
        let (helper, state) = read_drift(
            &printer,
            &[
                ("calibration_temp", "25"),
                ("drift_calibration", "300, -1, 0\n200, -1, 0\n100, -1, 0"),
            ],
        );
        assert!(helper.is_enabled());

        // Above the highest curve at the origin (260 at 40 °C): correct by
        // the curve's own drift, 275 - 260 (`:702-704`).
        assert_eq!(helper.adjust_freq(300., Some(40.)), 315.);
        // Between two curves: interpolate — 110 sits halfway between 60 and
        // 160 at 40 °C, and lands halfway between 75 and 175 at 25 °C.
        assert_eq!(helper.adjust_freq(110., Some(40.)), 125.);
        // Below every curve: untouched (`:709-710`).
        assert_eq!(helper.adjust_freq(50., Some(40.)), 50.);
        // …and back out to the origin temperature, exactly.
        assert_eq!(helper.unadjust_freq(125., Some(40.)), 110.);

        // With no temperature given the sensor's reading is the origin —
        // shared state set to 40 °C, the same answer as above.
        lock_state(&state).measurement.0 = 40.;
        assert_eq!(helper.adjust_freq(110., None), 125.);
        assert_eq!(helper.unadjust_freq(125., None), 110.);

        // `freq < min_freq` short-circuits: `min_freq` is -20 (the lowest
        // curve at 120 °C), and below it nothing is corrected — even where
        // the curves at an out-of-range origin would have moved it.
        assert_eq!(helper.adjust_freq(-30., Some(200.)), -30.);
        assert_eq!(helper.adjust_freq(0., Some(200.)), 175.);

        // A stopped run switches the correction off wholesale (`:622-624`).
        helper.start_calibration();
        assert!(!helper.is_enabled());
        assert_eq!(helper.adjust_freq(300., Some(40.)), 300.);
        assert_eq!(helper.unadjust_freq(300., Some(40.)), 300.);
    }

    /// `finish_calibration` with no run behind it (`:627-636`): upstream's
    /// `calibration_samples` is `None` here; the port reports the length
    /// check's refusal rather than the `TypeError` upstream walks into.
    #[test]
    fn finish_without_a_run_reports_not_enough_samples() {
        let (printer, _, _) = machine();
        let (helper, _) = read_drift(&printer, USABLE_DRIFT);

        let err = helper
            .finish_calibration(true)
            .expect_err("there were no samples");
        assert_eq!(err.to_string(), "calibration error, not enough samples");
        // An aborted run is silent, whatever is behind it (`:630-631`).
        assert!(helper.finish_calibration(false).is_ok());
    }

    /// A finished run: nine fits at their window heights, the crossing check,
    /// the two `configfile.set` writes and the report
    /// (`temperature_probe.py:639-656`).
    #[tokio::test]
    async fn finish_calibration_fits_and_saves_the_curves() {
        let m = Machine::new("temperature_probe probe").await;
        let helper = drift_helper(&m.printer, &m.probe, USABLE_DRIFT);
        helper.start_calibration();
        {
            let mut state = helper.lock();
            let samples = state
                .calibration_samples
                .as_mut()
                .expect("start_calibration opened the buckets");
            // Nine constant curves, 300 down to 140, over three temperatures
            // each — every adjacent pair stays strictly ordered.
            for (i, window) in samples.iter_mut().enumerate() {
                let level = 300. - 20. * i as f64;
                for temp in [20., 30., 40.] {
                    window.push((temp, level));
                }
            }
        }
        helper.finish_calibration(true).expect("the run fits");

        // The buckets went with the run (`:627-629`).
        assert!(helper.lock().calibration_samples.is_none());

        // The two writes (`:645-649`): the window temperatures bound the
        // range, so the saved minimum is the first sample's 20 °C.
        let configfile = m
            .printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("configfile is registered");
        let status = configfile.get_status(0.);
        let pending = &status["save_config_pending_items"]["temperature_probe probe"];
        assert_eq!(pending["drift_calibration_min_temp"], json!("20.0"));
        let saved = pending["drift_calibration"]
            .as_str()
            .expect("the curves are saved as text");
        // Upstream's `"\n" + "\n".join([str(p) …])` (`:646`): the option's
        // first line is empty and the curves hang below it, so the split
        // starts with an empty string.
        let lines: Vec<&str> = saved.split('\n').collect();
        assert_eq!(lines.len(), DRIFT_SAMPLE_COUNT + 1, "{saved}");
        assert_eq!(lines[0], "", "the first line is empty: {saved}");
        for (i, line) in lines.iter().skip(1).enumerate() {
            let coefs: Vec<f64> = line
                .split(',')
                .map(|coef| coef.trim().parse().expect("a number"))
                .collect();
            assert_eq!(coefs.len(), 3, "line {i}: {line}");
            // The fit recovers each constant; the flat coefficients only have
            // to round away — the solve may leave a hair of one behind.
            assert!(
                (coefs[0] - (300. - 20. * i as f64)).abs() < 1e-6,
                "line {i}: {line}"
            );
            assert!(
                coefs[1].abs() < 1e-6 && coefs[2].abs() < 1e-6,
                "line {i}: {line}"
            );
        }

        // The report (`:650-656`).
        assert!(
            m.replies()
                .iter()
                .any(|line| line.contains("temperature_probe probe: generated 9 2D polynomials")),
            "{:?}",
            m.replies()
        );
        assert!(
            m.replies().iter().any(|line| line.contains(
                "The SAVE_CONFIG command will update the printer config file and restart the \
                 printer."
            )),
            "{:?}",
            m.replies()
        );
    }

    /// The write-back end to end: the curves `finish_calibration` queues go
    /// through `SAVE_CONFIG`'s file rewrite, and the file a restart reads
    /// yields the pending value verbatim — then still loads as nine curves.
    #[tokio::test]
    async fn finish_calibrations_curves_survive_a_save_config_round_trip() {
        let m = Machine::new("temperature_probe probe").await;
        let helper = drift_helper(&m.printer, &m.probe, USABLE_DRIFT);
        helper.start_calibration();
        {
            let mut state = helper.lock();
            let samples = state
                .calibration_samples
                .as_mut()
                .expect("start_calibration opened the buckets");
            for (i, window) in samples.iter_mut().enumerate() {
                let level = 300. - 20. * i as f64;
                for temp in [20., 30., 40.] {
                    window.push((temp, level));
                }
            }
        }
        helper.finish_calibration(true).expect("the run fits");

        let configfile = m
            .printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("configfile is registered");
        let pending = configfile.get_status(0.)["save_config_pending_items"]
            ["temperature_probe probe"]["drift_calibration"]
            .as_str()
            .expect("the curves are saved as text")
            .to_string();

        let prefix = format!("drift_save_roundtrip_{}", std::process::id());
        let path = std::env::temp_dir().join(format!("{prefix}.cfg"));
        std::fs::write(&path, "[temperature_probe probe]\nsensor_type: Scripted\n")
            .expect("the body config is written");
        let cfgname = path.to_str().unwrap().to_string();
        crate::core::klippy::config::save_config::write_config(&configfile, &cfgname)
            .expect("SAVE_CONFIG writes");

        // The block's shape is upstream's: `drift_calibration =` with an
        // empty first line, each curve a tab-indented continuation.
        let written = std::fs::read_to_string(&path).expect("rewritten file readable");
        assert!(
            written.contains("#*# drift_calibration =\n#*# \t"),
            "{written}"
        );

        // What a restart reads back: parseable, and byte-for-byte the value
        // the run queued.
        let (reloaded, _) = Config::from_file(&path).expect("the rewritten config parses");
        let section = reloaded
            .get_section("temperature_probe probe")
            .expect("the section");
        assert_eq!(
            section.get_text("drift_calibration").expect("read back"),
            pending,
            "the pending value round trips verbatim"
        );

        // …and it still loads as the nine curves the loader reads.
        let reloaded_helper = EddyDriftCompensation::read(
            &ConfigWrapper::untracked(section),
            Arc::downgrade(&m.printer),
            Arc::new(Mutex::new(State::new())),
        )
        .expect("the drift options read");
        assert_eq!(
            reloaded_helper
                .lock()
                .drift_calibration
                .as_ref()
                .map(Vec::len),
            Some(DRIFT_SAMPLE_COUNT)
        );

        // The rewrite left a timestamped backup behind with the prefix too.
        let leftovers = std::fs::read_dir(std::env::temp_dir())
            .expect("the temp dir lists")
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|candidate| {
                candidate
                    .file_name()
                    .is_some_and(|name| name.to_string_lossy().starts_with(&prefix))
            });
        for leftover in leftovers {
            let _ = std::fs::remove_file(leftover);
        }
    }

    /// Curves that would cross are refused before anything is saved —
    /// `_check_calibration` through `gcode.error` (`:645`, `:657-673`).
    #[test]
    fn finish_calibration_refuses_curves_that_cross() {
        let (printer, _, _) = machine();
        let (helper, _) = read_drift(&printer, USABLE_DRIFT);
        helper.start_calibration();
        {
            let mut state = helper.lock();
            let samples = state
                .calibration_samples
                .as_mut()
                .expect("start_calibration opened the buckets");
            // Window 0 below the rest: the pair meets at the first
            // validation temperature, the first sample's 20 °C.
            for (i, window) in samples.iter_mut().enumerate() {
                let level = if i == 0 { 100. } else { 200. };
                for temp in [20., 30., 40.] {
                    window.push((temp, level));
                }
            }
        }
        let err = helper
            .finish_calibration(true)
            .expect_err("the curves cross");
        assert_eq!(
            err.to_string(),
            "temperature_probe name: invalid calibration detected, curve at index 1 overlaps \
             previous curve at temp 20C."
        );

        // The check runs before `configfile.set` (`:645`), so nothing was
        // queued for `SAVE_CONFIG`.
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("configfile is registered");
        assert_eq!(
            configfile.get_status(0.)["save_config_pending"],
            json!(false)
        );
    }

    /// `TEMPERATURE_PROBE_ENABLE` reaches `set_enabled` and `get_status`
    /// reads the flag back — the command wired to the helper (`:446-448`,
    /// `:457-459`).
    #[tokio::test]
    async fn enable_drives_the_registered_helper() {
        let m = Machine::new("temperature_probe probe").await;
        let helper = drift_helper(&m.printer, &m.probe, USABLE_DRIFT);
        assert!(helper.is_enabled());
        assert_eq!(m.probe.get_status(0.0)["compensation_enabled"], json!(true));

        m.run("TEMPERATURE_PROBE_ENABLE PROBE=probe ENABLE=0")
            .await
            .expect("the command runs");
        assert!(!helper.is_enabled());
        assert_eq!(
            m.probe.get_status(0.0)["compensation_enabled"],
            json!(false)
        );

        m.run("TEMPERATURE_PROBE_ENABLE PROBE=probe ENABLE=1")
            .await
            .expect("the command runs");
        assert!(helper.is_enabled());
        assert_eq!(m.probe.get_status(0.0)["compensation_enabled"], json!(true));

        // The word itself is required (`gcmd.get_int("ENABLE")`, `:530`).
        let err = m
            .run("TEMPERATURE_PROBE_ENABLE PROBE=probe")
            .await
            .expect_err("ENABLE is read");
        assert_eq!(
            err.to_string(),
            "Error on 'TEMPERATURE_PROBE_ENABLE PROBE=probe': missing ENABLE"
        );
    }

    /// The two refusals `set_enabled` guards an impossible enable with
    /// (`temperature_probe.py:533-541`).
    #[tokio::test]
    async fn enable_refuses_what_could_never_work() {
        // Curves, but no saved Z-calibration temperature.
        let m = Machine::new("temperature_probe probe").await;
        drift_helper(
            &m.printer,
            &m.probe,
            &[("drift_calibration", "300, 0, 0\n200, 0, 0\n100, 0, 0")],
        );
        let err = m
            .run("TEMPERATURE_PROBE_ENABLE PROBE=probe ENABLE=1")
            .await
            .expect_err("there is no temperature to correct toward");
        assert_eq!(
            err.to_string(),
            "Z Calibration temperature not configured, cannot enable temperature drift \
             compensation"
        );
        assert_eq!(
            m.probe.get_status(0.0)["compensation_enabled"],
            json!(false)
        );

        // No curves at all.
        let m = Machine::new("temperature_probe probe").await;
        drift_helper(&m.printer, &m.probe, &[("calibration_temp", "25")]);
        let err = m
            .run("TEMPERATURE_PROBE_ENABLE PROBE=probe ENABLE=1")
            .await
            .expect_err("there is nothing to apply");
        assert_eq!(
            err.to_string(),
            "No drift calibration configured, cannot enable temperature drift compensation"
        );
    }

    /// `_collect_sample`'s split (`temperature_probe.py:164-177`): the sensor's
    /// reading with no helper, the helper's sweep with one — which here stops
    /// at the probe lookup, since no `probe_eddy_current` object is on this
    /// machine.
    #[tokio::test]
    async fn collect_sample_follows_the_helper_or_the_sensor() {
        let m = Machine::new("temperature_probe probe").await;
        let coord = Coord::new(0., 0., 0., 0.);

        m.sensor.read(1000., 41.);
        let temp = m
            .probe
            .collect_sample(&coord, 0.)
            .await
            .expect("the sensor's reading");
        assert_eq!(temp, 41.);

        drift_helper(&m.printer, &m.probe, USABLE_DRIFT);
        let err = m
            .probe
            .collect_sample(&coord, 0.)
            .await
            .expect_err("the eddy probe object is missing");
        assert_eq!(
            err.to_string(),
            "Unknown config object 'probe_eddy_current probe'"
        );
    }

    /// The helper rides the state machine where upstream puts it: reset when
    /// the run starts (`:388`), closed out when it ends (`:249-254`).
    #[tokio::test]
    async fn the_state_machine_drives_the_helper_where_upstream_does() {
        let m = Machine::new("temperature_probe probe").await;
        let helper = drift_helper(&m.printer, &m.probe, USABLE_DRIFT);
        assert!(helper.is_enabled());

        // `:387-388`: the run switches the correction off and opens the
        // buckets.
        let gcmd = m.command(
            "TEMPERATURE_PROBE_CALIBRATE",
            &[("TARGET", "10"), ("STEP", "2")],
        );
        m.probe
            .start_calibration(&gcmd, "manual")
            .await
            .expect("the run starts");
        assert!(!helper.is_enabled());
        assert!(helper.lock().calibration_samples.is_some());

        // Close the interactive probe without sampling: the manual probe's
        // `ABORT` runs `_finalize_drift_cal(False)`, whose helper call comes
        // back silent with the buckets taken (`:249-254`).
        m.run("ABORT").await.expect("the run aborts");
        Machine::settle().await;
        assert!(helper.lock().calibration_samples.is_none());
        assert!(
            m.replies()
                .iter()
                .any(|line| line.contains("temperature_probe probe: calibration aborted")),
            "{:?}",
            m.replies()
        );

        // The successful close-out: fill a valid run and let COMPLETE carry
        // it through `finalize` into `finish_calibration(true)`.
        helper.start_calibration();
        {
            let mut state = helper.lock();
            let samples = state
                .calibration_samples
                .as_mut()
                .expect("start_calibration opened the buckets");
            for (i, window) in samples.iter_mut().enumerate() {
                let level = 300. - 20. * i as f64;
                for temp in [20., 30., 40.] {
                    window.push((temp, level));
                }
            }
        }
        lock_state(&m.probe.state).sample_count = 3;
        let complete = m.command("TEMPERATURE_PROBE_COMPLETE", &[]);
        m.probe
            .cmd_complete(&complete)
            .await
            .expect("the run completes");
        assert!(
            m.replies()
                .iter()
                .any(|line| line.contains("temperature_probe probe: generated 9 2D polynomials")),
            "{:?}",
            m.replies()
        );
        assert!(helper.lock().calibration_samples.is_none());
    }

    /// The batch client's window bookkeeping (`_on_bulk_data_recd`,
    /// `temperature_probe.py:586-603`): rows land in their window, rows that
    /// pass one retire it, and the last retirement lets the client stay.
    #[test]
    fn the_sweep_buckets_samples_and_retires_windows_upstreams_way() {
        let mut sweep = SweepState::default();
        for i in 0..DRIFT_SAMPLE_COUNT {
            let start = 10. + i as f64;
            sweep.move_times.push((i, start, start + 0.1));
        }
        let mut rows = Vec::new();
        // Before the first window: recorded nowhere (`:599-600`).
        rows.push([9.5, 555., 555.]);
        // Window 0's sample, then a row in the gap that retires it without
        // landing anywhere (`:591-597`).
        rows.push([10.05, 1000., 0.]);
        rows.push([10.5, 555., 555.]);
        // One row per remaining window.
        for i in 1..DRIFT_SAMPLE_COUNT {
            rows.push([10. + i as f64 + 0.05, 1000. - i as f64, i as f64]);
        }
        // Past the last window: the sweep is done, and the answer is
        // `idx >= DRIFT_SAMPLE_COUNT - 1` — the client may stay (`:592-594`).
        rows.push([18.2, 0., 0.]);
        assert!(sweep.absorb(&rows, 42.));
        assert!(sweep.move_times.is_empty());
        for i in 0..DRIFT_SAMPLE_COUNT {
            assert_eq!(
                sweep.probe_samples[i],
                vec![(1000. - i as f64, i as f64)],
                "window {i}"
            );
            assert_eq!(sweep.temps[i], 42.);
        }

        // With no window open every message is a keep-alive (`:586-587`).
        let mut idle = SweepState::default();
        assert!(idle.absorb(&[[10.05, 1000., 0.]], 42.));
    }

    /// The sweep's results (`:617-624`): the averaged temperature over all
    /// nine windows, each window's average, and the refusal for a window the
    /// stream never filled (upstream divides by its length there and raises).
    #[test]
    fn the_sweep_averages_each_window_and_refuses_an_empty_one() {
        let mut sweep = SweepState::default();
        for i in 0..DRIFT_SAMPLE_COUNT {
            let start = 10. + i as f64;
            sweep.move_times.push((i, start, start + 0.1));
            sweep.temps[i] = 42.;
            sweep.probe_samples[i].push((1000. - i as f64, i as f64));
        }
        let (sample_temp, windows) = sweep.into_samples(0.).expect("every window has data");
        // `sum(temps) / len(temps)` over the nine (`:617-618`).
        assert_eq!(sample_temp, 42.);
        assert_eq!(windows.len(), DRIFT_SAMPLE_COUNT);
        for (i, (temp, freq)) in windows.iter().enumerate() {
            assert_eq!(*temp, 42.);
            assert_eq!(*freq, 1000. - i as f64);
        }

        let mut incomplete = SweepState::default();
        incomplete.temps[0] = 42.;
        incomplete.probe_samples[0].push((1000., 0.));
        let err = incomplete.into_samples(0.).expect_err("window 1 is empty");
        assert_eq!(
            err.to_string(),
            "Failed calibration - incomplete sensor data"
        );
    }

    /// The wiring gate (`temperature_probe.py:125-137`): a section that names
    /// `probe_eddy_current <name>` must find its object — the loader
    /// registers it before this factory runs — and one that does not runs
    /// without a helper, upstream's `else`.
    #[test]
    fn the_drift_wiring_follows_the_probe_section() {
        // The sibling section exists but its object does not (a machine
        // where nothing registered the eddy probe): upstream's
        // `printer.load_object(...)` would have built it; the port reports
        // the lookup it does instead.
        let (printer, _, _) = machine();
        let heaters = heaters::ensure(&printer).expect("heaters registers");
        heaters.add_sensor_factory(
            "Scripted",
            Arc::new(|_, _| Ok(Arc::new(ScriptedSensor::default()) as Arc<dyn heaters::Sensor>)),
        );
        let text = "\
[temperature_probe probe]
sensor_type: Scripted

[probe_eddy_current probe]
sensor_type: ldc1612
";
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .get_section("temperature_probe probe")
            .expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);
        let err = build_temperature_probe(&wrapper, &printer)
            .expect_err("the eddy probe object is not registered");
        assert_eq!(
            err.to_string(),
            "Unknown config object 'probe_eddy_current probe'"
        );

        // No sibling section: the helper stays absent (`:135-137`), and the
        // drift options are not read — they would not be valid here either.
        let (printer, _, _) = machine();
        let heaters = heaters::ensure(&printer).expect("heaters registers");
        heaters.add_sensor_factory(
            "Scripted",
            Arc::new(|_, _| Ok(Arc::new(ScriptedSensor::default()) as Arc<dyn heaters::Sensor>)),
        );
        let text = "[temperature_probe probe]\nsensor_type: Scripted\n";
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .get_section("temperature_probe probe")
            .expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);
        let probe = build_temperature_probe(&wrapper, &printer).expect("the section loads");
        assert!(probe.drift_helper().is_none());
    }

    // ---- round2 half-to-even ----

    /// `round2` matches Python's `round(v, 2)` — half-to-even, not
    /// half-away-from-zero. The cases below are genuine half-way points
    /// (where `v * 100.0` is exactly `xxx.5` in IEEE 754) and were verified
    /// against CPython's `round(v, 2)`.
    #[test]
    fn round2_half_to_even_matches_python() {
        // 2.005 * 100 = 200.5 — halfway. Even neighbour is 200 → 2.00.
        // Rust's .round() (half-away-from-zero) would give 2.01.
        assert_eq!(round2(2.005), 2.0);
        // 3.445 * 100 = 344.5 — halfway. Even neighbour is 344 → 3.44.
        // Rust's .round() would give 3.45.
        assert_eq!(round2(3.445), 3.44);
        // 3.455 * 100 = 345.5 — halfway. Even neighbour is 346 → 3.46
        // (floor is odd, so we round up to even).
        assert_eq!(round2(3.455), 3.46);
        // 0.055 * 100 = 5.5 — halfway. Even neighbour is 6 → 0.06
        // (floor is odd, so we round up to even).
        assert_eq!(round2(0.055), 0.06);
    }

    /// Values that are not exact halves round normally in both modes —
    /// `round2` must still produce the standard nearest-neighbour result.
    #[test]
    fn round2_non_half_values_round_normally() {
        assert_eq!(round2(2.344), 2.34);
        assert_eq!(round2(2.346), 2.35);
        assert_eq!(round2(0.0), 0.0);
        assert_eq!(round2(65.0), 65.0);
        // Negative: -3.445 * 100 = -344.5, floor = -345 (odd in absolute
        // terms), even neighbour is -344 → -3.44.
        assert_eq!(round2(-3.445), -3.44);
    }
}
