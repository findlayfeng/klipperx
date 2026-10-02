//! `[temperature_probe <name>]` — the probe's own temperature sensor.
//!
//! Upstream's `temperature_probe.py` is two halves in one file: a smoothed
//! temperature sensor that joins the `heaters` registry, and the interactive
//! thermal-drift calibration of an eddy probe that is driven *from* that
//! sensor. This module is the sensor half — the section reads its options,
//! `heaters` builds and delivers readings, and `_temp_callback`'s smoothing
//! lands in `get_status`.
//!
//! | here | upstream |
//! |---|---|
//! | [`Polynomial2d`] | `Polynomial2d` (`temperature_probe.py:16-57`) |
//! | [`TemperatureProbeOptions`] | `TemperatureProbe.__init__`'s option reads (`:63-97`) |
//! | [`TemperatureProbe`] | `TemperatureProbe.__init__` (`:60-111`) |
//! | [`sensor_callback`] | `_temp_callback` (`:140-153`) |
//! | [`check_kick_next`] | `_check_kick_next` (`:155-159`) |
//! | [`TemperatureProbe::get_status`] | `get_status` (`:453-465`) |
//!
//! # Not here (deferred; the upstream line ranges are the gap list)
//!
//! * **The command family and the calibration state machine behind it.**
//!   `TEMPERATURE_PROBE_CALIBRATE` / `_NEXT` / `_COMPLETE` / `_ABORT` /
//!   `_ENABLE` registration (`temperature_probe.py:112-123`), the flow they
//!   drive (`:164-337`) and the command bodies (`:338-448`). The section
//!   already parses every option that flow consumes — `speed`,
//!   `horizontal_move_z`, `resting_z`, `calibration_position`,
//!   `calibration_bed_temp`, `calibration_extruder_temp`,
//!   `extruder_heating_z` — and `get_status` keeps `in_calibration` /
//!   `estimated_expansion`, so the status shape is upstream's and only the
//!   flow is missing. [`check_kick_next`] is ported and runs the upstream
//!   script, but nothing sets `in_calibration` until that flow lands, so it
//!   cannot fire yet.
//! * **`EddyDriftCompensation`** (`temperature_probe.py:479-714`) and the
//!   `probe_eddy_current` registration it needs (`:125-137`): until it exists,
//!   `get_status`'s `compensation_enabled` is always `false` — which is exactly
//!   what upstream reports when no drift compensation was registered.
//! * **`stats`** (`temperature_probe.py:467-468`): the port has no
//!   `Printer`-level walker that collects object `stats` yet;
//!   [`TemperatureProbe::stats`] is upstream's shape, ready for one.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters;
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::solve_linear_equations;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("temperature_probe", order = 30, prefix = load_config_prefix);

/// Upstream's `KELVIN_TO_CELSIUS` (`temperature_probe.py:10`): the `min_temp`
/// default *and* its `minval`.
const KELVIN_TO_CELSIUS: f64 = -273.15;

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
/// `next_auto_temp` / `target_temp` / `total_expansion`, `:64-67,104-110`).
///
/// Shared rather than owned by the object because `_temp_callback` is bound as
/// a plain closure while the object is still being built.
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
}

impl State {
    /// Upstream's `__init__` starting values (`:64-67,104-110`).
    fn new() -> Self {
        Self {
            last_temp_read_time: 0.,
            measurement: (0., 99999999., 0.),
            in_calibration: false,
            next_auto_temp: 99999999.,
            target_temp: 0.,
            total_expansion: 0.,
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
    /// The sensor the readings come from. Held (not just a weak handle) so the
    /// ADC/report callback that drives it keeps upgrading — the reason
    /// `temperature_sensor.rs` holds its own.
    #[allow(dead_code)]
    sensor: Arc<dyn heaters::Sensor>,
    /// The readings and calibration flags the sensor callback shares.
    state: Arc<Mutex<State>>,
}

impl TemperatureProbe {
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
        let state = lock_state(&self.state);
        let (smoothed, measured_min, measured_max) = state.measurement;
        json!({
            "temperature": smoothed,
            "measured_min_temp": round2(measured_min),
            "measured_max_temp": round2(measured_max),
            "in_calibration": state.in_calibration,
            "estimated_expansion": state.total_expansion,
            // Upstream reads this off its `cal_helper` (`EddyDriftCompensation`,
            // not ported — module docs); without one it is `false` too.
            "compensation_enabled": false,
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
fn round2(value: f64) -> f64 {
    (value * 100.0).round() / 100.0
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
    heaters.register_sensor(config)?;
    Ok(Arc::new(TemperatureProbe {
        name,
        options,
        sensor,
        state,
    }))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

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

    /// Load a `[temperature_probe name]` from `text` with a scripted sensor,
    /// returning the object and the slot its readings come from.
    fn load(text: &str) -> (Arc<TemperatureProbe>, Arc<ScriptedSensor>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let heaters = heaters::ensure(&printer).expect("heaters registers");
        let slot = Arc::new(ScriptedSensor::default());
        let sensor = Arc::clone(&slot);
        heaters.add_sensor_factory(
            "Scripted",
            Arc::new(move |_, _| Ok(Arc::clone(&sensor) as Arc<dyn heaters::Sensor>)),
        );

        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .get_section("temperature_probe name")
            .expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(section, Arc::clone(&access));
        let probe = build_temperature_probe(&wrapper, &printer).expect("the section loads");
        check_unused(&config, &access, &[]).expect("no option is left unread");
        (probe, slot)
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
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
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
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
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
}
