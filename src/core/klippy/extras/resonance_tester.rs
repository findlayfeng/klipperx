//! `[resonance_tester]` — the resonance test section and its three commands
//! (upstream `klippy/extras/resonance_tester.py`).
//!
//! | option | default | bounds |
//! |---|---|---|
//! | `move_speed` | `50.` | above `0` |
//! | `min_freq` | `5.` | minimum `1.` |
//! | `max_freq` | `135.` | `min_freq`…`300.` |
//! | `max_freq_z` | `100.` | `min_freq`…`300.` |
//! | `accel_per_hz` | `60.` | above `0` |
//! | `accel_per_hz_z` | `15.` | above `0` |
//! | `hz_per_sec` | `1.` | `0.1`…`2.` |
//! | `sweeping_accel` | `400.` | above `0` |
//! | `sweeping_accel_z` | `50.` | above `0` |
//! | `sweeping_period` | `1.2` | minimum `0.` |
//! | `probe_points` | `[]` | three floats per line |
//! | `max_smoothing` | unset | minimum `0.05` |
//! | `accel_chip_x` | unset | — |
//! | `accel_chip_y` | unset | — |
//! | `accel_chip_z` | `''` | — |
//! | `accel_chip` | — | required when `accel_chip_x` is unset |
//!
//! The accelerometer chips are read as up to three axis names and grouped by
//! chip: `accel_chip_x` unset means one `accel_chip` covers `x` and `y`, with
//! `accel_chip_z` optional; `accel_chip_x` set means `accel_chip_x`/`accel_chip_y`
//! are both required, `accel_chip_z` still optional (`resonance_tester.py:262-283`).
//! The chips are then sorted by name and the axes of one chip joined, so the
//! corpus's `input_shaper.cfg` yields `[("xz", "adxl345"), ("y", "mpu9250 my_mpu")]`.
//!
//! `connect` resolves each named chip — skipping an empty name — and refuses a
//! name it cannot find (`Unknown config object '<name>'`) or one that is not a
//! [`Adxl345`] or [`Mpu9250`] (`'<name>' is not an accelerometer`), the two
//! messages `connect` gets from `hasattr(chip, 'start_internal_client')`
//! upstream.
//!
//! # Differences from upstream
//!
//! The measurement data path is not ported: upstream streams accelerometer
//! samples through a chip client, drives a vibration test with the toolhead, and
//! fits a PSD with `shaper_calibrate` to recommend input-shaper parameters. This
//! host's chip clients do not carry samples yet
//! ([`Adxl345`](super::adxl345#gap-the-bulk-sample-path-is-not-wired)), there is
//! no `shaper_calibrate`, and running a real motion test against the regression
//! harness's fake MCU would only stall. The three commands therefore register
//! and validate their parameters exactly as upstream parses them, then report
//! the gap in place of the run.
//!
//! What is real: section loading, every option above (read and bounded as
//! upstream), the chip grouping, the `connect` checks, and the three command
//! names and help strings.

use std::sync::{Arc, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::adxl345::Adxl345;
use crate::core::klippy::extras::mpu9250::Mpu9250;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("resonance_tester", order = 30, load = load_config);

/// What every command reports in place of a run: the section and the command
/// surface are wired, but the measurement data path behind them is not (module
/// docs).
const DATA_PATH_GAP: &str = "Resonance testing is not available in this host: \
the accelerometer sample collection and PSD fitting data path is not wired.";

/// `[resonance_tester]`:
///
/// ```text
/// [resonance_tester]
/// probe_points: 20,20,20
/// accel_chip_x: adxl345
/// accel_chip_y: mpu9250 my_mpu
/// accel_chip_z: adxl345
/// ```
///
/// The parsed values stay with the object so the measurement path can start
/// from them; nothing reads them while the commands only report the gap, hence
/// the field-level `allow`.
#[allow(dead_code)]
pub struct ResonanceTester {
    printer: Weak<Printer>,
    /// `move_speed`, default `50.`, above `0` (`resonance_tester.py:265`).
    move_speed: f64,
    /// `min_freq`, default `5.`, minimum `1.` (`resonance_tester.py:59`).
    min_freq: f64,
    /// `max_freq`, default `135.`, in `min_freq`…`300.`.
    max_freq: f64,
    /// `max_freq_z`, default `100.`, in `min_freq`…`300.`.
    max_freq_z: f64,
    /// `accel_per_hz`, default `60.`, above `0`.
    accel_per_hz: f64,
    /// `accel_per_hz_z`, default `15.`, above `0`.
    accel_per_hz_z: f64,
    /// `hz_per_sec`, default `1.`, in `0.1`…`2.`
    /// (`resonance_tester.py:66`).
    hz_per_sec: f64,
    /// `sweeping_accel`, default `400.`, above `0`
    /// (`resonance_tester.py:100`).
    sweeping_accel: f64,
    /// `sweeping_accel_z`, default `50.`, above `0`.
    sweeping_accel_z: f64,
    /// `sweeping_period`, default `1.2`, minimum `0.`
    /// (`resonance_tester.py:102`).
    sweeping_period: f64,
    /// `probe_points`, three floats per line; empty when unset
    /// (`resonance_tester.py:290-291`).
    probe_points: Vec<[f64; 3]>,
    /// `max_smoothing`, optional, minimum `0.05`
    /// (`resonance_tester.py:284`).
    max_smoothing: Option<f64>,
    /// The chips to measure with, as `(axis letters, chip name)`: grouped by
    /// chip name and sorted, the axis letters of one chip joined
    /// (`resonance_tester.py:279-283`).
    accel_chip_names: Vec<(String, String)>,
}

impl ResonanceTester {
    /// Read the section (`resonance_tester.py:263-298`).
    ///
    /// # Errors
    /// [`ConfigError`] for an option outside its bounds, a `probe_points` line
    /// that is not three floats, or a missing `accel_chip`.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let move_speed =
            config.get_float_bounded("move_speed", Some(50.0), None, None, Some(0.0), None)?;
        // `SweepingVibrationsTestGenerator.__init__` builds the pulse generator
        // first, then the sweeping options (`resonance_tester.py:57-103`).
        let min_freq =
            config.get_float_bounded("min_freq", Some(5.0), Some(1.0), None, None, None)?;
        let max_freq = config.get_float_bounded(
            "max_freq",
            Some(135.0),
            Some(min_freq),
            Some(300.0),
            None,
            None,
        )?;
        let max_freq_z = config.get_float_bounded(
            "max_freq_z",
            Some(100.0),
            Some(min_freq),
            Some(300.0),
            None,
            None,
        )?;
        let accel_per_hz =
            config.get_float_bounded("accel_per_hz", Some(60.0), None, None, Some(0.0), None)?;
        let accel_per_hz_z =
            config.get_float_bounded("accel_per_hz_z", Some(15.0), None, None, Some(0.0), None)?;
        let hz_per_sec =
            config.get_float_bounded("hz_per_sec", Some(1.0), Some(0.1), Some(2.0), None, None)?;
        let sweeping_accel =
            config.get_float_bounded("sweeping_accel", Some(400.0), None, None, Some(0.0), None)?;
        let sweeping_accel_z = config.get_float_bounded(
            "sweeping_accel_z",
            Some(50.0),
            None,
            None,
            Some(0.0),
            None,
        )?;
        let sweeping_period =
            config.get_float_bounded("sweeping_period", Some(1.2), Some(0.0), None, None, None)?;

        let accel_chip_names = read_accel_chip_names(config)?;

        // `max_smoothing` may be absent: upstream reads it with a `None`
        // default, so it must be guarded rather than read as a required,
        // bounded option (`resonance_tester.py:289`).
        let max_smoothing = if config.has("max_smoothing") {
            Some(config.get_float_bounded("max_smoothing", None, Some(0.05), None, None, None)?)
        } else {
            None
        };

        let probe_points = read_probe_points(config)?;

        Ok(Self {
            printer: Arc::downgrade(printer),
            move_speed,
            min_freq,
            max_freq,
            max_freq_z,
            accel_per_hz,
            accel_per_hz_z,
            hz_per_sec,
            sweeping_accel,
            sweeping_accel_z,
            sweeping_period,
            probe_points,
            max_smoothing,
            accel_chip_names,
        })
    }

    /// Upstream's `connect` (`resonance_tester.py:300-309`): resolve every
    /// named chip, skipping the empty names a missing per-axis option leaves
    /// behind.
    ///
    /// The first failure sets the printer's error state and stops, as upstream
    /// raises on the first one.
    fn connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        for (_axes, chip_name) in &self.accel_chip_names {
            if chip_name.is_empty() {
                continue;
            }
            if printer.lookup_object(chip_name).is_none() {
                printer.set_error_state(&format!("Unknown config object '{chip_name}'"));
                return;
            }
            // Upstream checks `hasattr(chip, 'start_internal_client')`; the two
            // accelerometer types are what carry that client here.
            let is_accelerometer = printer.lookup_object_as::<Adxl345>(chip_name).is_some()
                || printer.lookup_object_as::<Mpu9250>(chip_name).is_some();
            if !is_accelerometer {
                printer.set_error_state(&format!("'{chip_name}' is not an accelerometer"));
                return;
            }
        }
    }

    /// Upstream's `_parse_chips` (`resonance_tester.py:382-393`): each
    /// comma-separated name must resolve to an accelerometer. The parsed chips
    /// are unused while the run is a gap, so only the validation runs.
    ///
    /// # Errors
    /// A name that resolves to nothing (`Name '<name>' is not valid for CHIPS
    /// parameter`) or to something that is not an accelerometer.
    fn parse_chips(&self, chips: &str) -> Result<(), CommandError> {
        let Some(printer) = self.printer.upgrade() else {
            return Ok(());
        };
        for chip_name in chips.split(',') {
            let name = chip_name.trim();
            if printer.lookup_object(name).is_none() {
                return Err(CommandError::new(format!(
                    "Name '{chip_name}' is not valid for CHIPS parameter"
                )));
            }
            let is_accelerometer = printer.lookup_object_as::<Adxl345>(name).is_some()
                || printer.lookup_object_as::<Mpu9250>(name).is_some();
            if !is_accelerometer {
                return Err(CommandError::new(format!(
                    "'{chip_name}' is not an accelerometer"
                )));
            }
        }
        Ok(())
    }

    /// Upstream's `cmd_TEST_RESONANCES` (`resonance_tester.py:397-444`): parse
    /// every parameter, then report the gap.
    ///
    /// # Errors
    /// As upstream's parse does, before the run.
    fn cmd_test_resonances(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let axis = gcmd.get_str("AXIS")?.to_lowercase();
        parse_axis(&axis)?;
        // An empty `CHIPS`/`POINT` is upstream's unset (`if chips_str:`),
        // not an empty name to resolve (`resonance_tester.py:349-360`).
        if let Ok(chips) = gcmd.get_str("CHIPS") {
            if !chips.is_empty() {
                self.parse_chips(&chips)?;
            }
        }
        if let Ok(point) = gcmd.get_str("POINT") {
            if !point.is_empty() {
                parse_point(&point)?;
            }
        }
        parse_outputs(&gcmd.get_str_default("OUTPUT", "resonances"))?;
        if let Ok(name) = gcmd.get_str("NAME") {
            check_name_suffix(&name)?;
        }
        Err(CommandError::new(DATA_PATH_GAP))
    }

    /// Upstream's `cmd_SHAPER_CALIBRATE` (`resonance_tester.py:447-505`): parse
    /// every parameter, then report the gap.
    ///
    /// # Errors
    /// As upstream's parse does, before the run.
    fn cmd_shaper_calibrate(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        if let Ok(axis) = gcmd.get_str("AXIS") {
            // Upstream spells this `axis.lower() not in 'xyz'`, a substring
            // test, so a multi-character `AXIS` (`xy`) is accepted here too;
            // the error quotes the raw parameter (`resonance_tester.py:452-453`).
            if !"xyz".contains(&axis.to_lowercase()) {
                return Err(CommandError::new(format!("Unsupported axis '{axis}'")));
            }
        }
        if let Ok(chips) = gcmd.get_str("CHIPS") {
            if !chips.is_empty() {
                self.parse_chips(&chips)?;
            }
        }
        // `MAX_SMOOTHING` falls back to the section's `max_smoothing`, which may
        // itself be unset; only a *present* value is parsed and bounded here
        // (`resonance_tester.py:346-347`).
        if gcmd.get_str("MAX_SMOOTHING").is_ok() {
            gcmd.get(
                "MAX_SMOOTHING",
                Some(0.05),
                parse_float,
                Some(0.05),
                None,
                None,
                None,
            )?;
        }
        if let Ok(name) = gcmd.get_str("NAME") {
            check_name_suffix(&name)?;
        }
        Err(CommandError::new(DATA_PATH_GAP))
    }

    /// Upstream's `cmd_MEASURE_AXES_NOISE` (`resonance_tester.py:508-527`):
    /// parse `MEAS_TIME`, then report the gap.
    ///
    /// # Errors
    /// As upstream's `get_float` does for a `MEAS_TIME` that is not above `0`.
    fn cmd_measure_axes_noise(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        gcmd.get(
            "MEAS_TIME",
            Some(2.0),
            parse_float,
            None,
            None,
            Some(0.0),
            None,
        )?;
        Err(CommandError::new(DATA_PATH_GAP))
    }

    /// The commands and events upstream's `__init__` registers
    /// (`resonance_tester.py:288-298`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.connect()
            }),
        );
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        // Each entry's parameter list is the `KEY`s that command's handler
        // reads, in read order (`cmd_measure_axes_noise` / `cmd_test_resonances`
        // / `cmd_shaper_calibrate`).
        let commands: [(&str, CommandHandler, &str, &[&str]); 3] = [
            (
                "MEASURE_AXES_NOISE",
                sync({
                    let object = Arc::clone(self);
                    move |gcmd| object.cmd_measure_axes_noise(gcmd)
                }),
                "Measures noise of all enabled accelerometer chips",
                &["MEAS_TIME"],
            ),
            (
                "TEST_RESONANCES",
                sync({
                    let object = Arc::clone(self);
                    move |gcmd| object.cmd_test_resonances(gcmd)
                }),
                "Runs the resonance test for a specified axis",
                &["AXIS", "CHIPS", "POINT", "OUTPUT", "NAME"],
            ),
            (
                "SHAPER_CALIBRATE",
                sync({
                    let object = Arc::clone(self);
                    move |gcmd| object.cmd_shaper_calibrate(gcmd)
                }),
                "Similar to TEST_RESONANCES but suggest input shaper config",
                &["AXIS", "CHIPS", "MAX_SMOOTHING", "NAME"],
            ),
        ];
        for (name, handler, desc, params) in commands {
            gcode
                .register_command_with_params(name, handler, Some(desc), params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }
}

impl PrinterObject for ResonanceTester {
    fn get_status(&self, _eventtime: f64) -> Value {
        // Upstream's `ResonanceTester` has no `get_status`, and `is_queryable`
        // keeps the object out of `objects/list` either way.
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// Read the chip names and group them by accelerometer
/// (`resonance_tester.py:262-283`).
///
/// # Errors
/// [`ConfigError`] when `accel_chip_x` is unset and `accel_chip` is missing, or
/// when `accel_chip_x` is set and `accel_chip_y` is missing.
fn read_accel_chip_names(config: &ConfigWrapper) -> Result<Vec<(String, String)>, ConfigError> {
    let accel_chip_x = config.get_str("accel_chip_x");
    let chips = if accel_chip_x.as_deref().map_or(true, str::is_empty) {
        vec![
            (
                "xy".to_string(),
                config.get("accel_chip", None)?.trim().to_string(),
            ),
            (
                "z".to_string(),
                config.get("accel_chip_z", Some(""))?.trim().to_string(),
            ),
        ]
    } else {
        vec![
            ("x".to_string(), accel_chip_x.unwrap().trim().to_string()),
            (
                "y".to_string(),
                config.get("accel_chip_y", None)?.trim().to_string(),
            ),
            (
                "z".to_string(),
                config.get("accel_chip_z", Some(""))?.trim().to_string(),
            ),
        ]
    };
    Ok(group_chips(chips))
}

/// Group entries that name the same chip, joining their axis letters
/// (`resonance_tester.py:277-283`).
///
/// The entries are sorted by chip name (stably, as Python's `sorted` is), then
/// consecutive same-name entries are merged and their axis letters sorted, so
/// the result is one `(axis letters, chip name)` per chip.
fn group_chips(mut chips: Vec<(String, String)>) -> Vec<(String, String)> {
    chips.sort_by(|a, b| a.1.cmp(&b.1));
    let mut groups: Vec<(String, String)> = Vec::new();
    for (axis, name) in chips {
        match groups.last_mut() {
            Some(last) if last.1 == name => last.0.push_str(&axis),
            _ => groups.push((axis, name)),
        }
    }
    for (axes, _) in &mut groups {
        let mut letters: Vec<char> = axes.chars().collect();
        letters.sort_unstable();
        *axes = letters.into_iter().collect();
    }
    groups
}

/// Read `probe_points` (`resonance_tester.py:285-286`): three floats per line.
///
/// # Errors
/// [`ConfigError`] for a line whose parsed group is not three floats, or a
/// value that is not a number.
fn read_probe_points(config: &ConfigWrapper) -> Result<Vec<[f64; 3]>, ConfigError> {
    let groups = config.get_list_of_lists("probe_points", '\n', ',', 3)?;
    let identifier = config.identifier();
    let mut points = Vec::with_capacity(groups.len());
    for group in groups {
        let mut point = [0.0; 3];
        for (index, item) in group.iter().enumerate() {
            point[index] = item.trim().parse::<f64>().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option 'probe_points' in section '{identifier}'"
                ))
            })?;
        }
        points.push(point);
    }
    Ok(points)
}

/// Parse an `AXIS` parameter (`_parse_axis`, `resonance_tester.py:39-55`): an
/// axis letter, or a two- or three-component vibration direction.
///
/// The direction itself is unused while the run is a gap, so only the format is
/// checked.
///
/// # Errors
/// `Invalid format of axis '<axis>'` for a direction with the wrong number of
/// components, `Unable to parse axis direction '<axis>'` for a component that
/// is not a number.
fn parse_axis(raw_axis: &str) -> Result<(), CommandError> {
    if matches!(raw_axis, "x" | "y" | "z") {
        return Ok(());
    }
    let dirs: Vec<&str> = raw_axis.split(',').collect();
    if dirs.len() != 2 && dirs.len() != 3 {
        return Err(CommandError::new(format!(
            "Invalid format of axis '{raw_axis}'"
        )));
    }
    for dir in &dirs {
        if parse_float(dir.trim()).is_none() {
            return Err(CommandError::new(format!(
                "Unable to parse axis direction '{raw_axis}'"
            )));
        }
    }
    Ok(())
}

/// Parse a `POINT` parameter (`resonance_tester.py:350-358`).
///
/// # Errors
/// `Invalid POINT parameter, must be 'x,y,z'` for the wrong number of
/// components, or the longer message for a component that is not a number.
fn parse_point(point: &str) -> Result<(), CommandError> {
    let coords: Vec<&str> = point.split(',').collect();
    if coords.len() != 3 {
        return Err(CommandError::new(
            "Invalid POINT parameter, must be 'x,y,z'",
        ));
    }
    for coord in &coords {
        if parse_float(coord.trim()).is_none() {
            return Err(CommandError::new(
                "Invalid POINT parameter, must be 'x,y,z' where x, y and z are \
valid floating point numbers",
            ));
        }
    }
    Ok(())
}

/// Parse an `OUTPUT` parameter (`resonance_tester.py:362-368`).
///
/// # Errors
/// `Unsupported output '<output>', only 'resonances' and 'raw_data' are
/// supported` for any other output.
fn parse_outputs(outputs: &str) -> Result<(), CommandError> {
    let outputs = outputs.to_lowercase();
    for output in outputs.split(',') {
        if !matches!(output, "resonances" | "raw_data") {
            return Err(CommandError::new(format!(
                "Unsupported output '{output}', only 'resonances' and 'raw_data' are supported"
            )));
        }
    }
    // `split` never yields an empty vector, so upstream's "No output
    // specified" branch is unreachable here as well; the default `resonances`
    // also means it is only reached from an explicit `OUTPUT=`, which then
    // fails the loop above with `Unsupported output ''`.
    Ok(())
}

/// Check a `NAME` suffix (`is_valid_name_suffix`, `resonance_tester.py:529-530`):
/// alphanumeric once `-` and `_` are dropped.
///
/// Only a written `NAME` reaches this; upstream's default is a timestamp, which
/// is always valid.
///
/// # Errors
/// `Invalid NAME parameter` for a suffix that is not alphanumeric.
fn check_name_suffix(name_suffix: &str) -> Result<(), CommandError> {
    let stripped: String = name_suffix
        .chars()
        .filter(|c| *c != '-' && *c != '_')
        .collect();
    // `str.isalnum()` is false for the empty string, so an empty `NAME` is
    // invalid too (`resonance_tester.py:529-530`).
    if stripped.is_empty() || !stripped.chars().all(char::is_alphanumeric) {
        return Err(CommandError::new("Invalid NAME parameter"));
    }
    Ok(())
}

/// The factory `section!` names (`resonance_tester.py:552-553 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(ResonanceTester::new(config, printer)?);
    object.register_handlers(printer)?;
    Ok(object)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{
        access::AccessTracking, check_unused, Config, ConfigSection, ConfigValue,
    };
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[resonance_tester]` section with `options`.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("resonance_tester", None);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with a g-code dispatcher, so a section can register commands.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer
    }

    /// A section loaded and its handlers registered on a fresh printer.
    fn loaded(options: &[(&str, &str)]) -> (Arc<Printer>, Arc<ResonanceTester>) {
        let printer = printer();
        let section = section(options);
        let object = Arc::new(
            ResonanceTester::new(&ConfigWrapper::untracked(&section), &printer)
                .expect("the section loads"),
        );
        object
            .register_handlers(&printer)
            .expect("the handlers register");
        (printer, object)
    }

    /// The corpus's `input_shaper.cfg` `[resonance_tester]` options.
    const CORPUS_OPTIONS: &[(&str, &str)] = &[
        ("probe_points", "20,20,20"),
        ("accel_chip_x", "adxl345"),
        ("accel_chip_y", "mpu9250 my_mpu"),
        ("accel_chip_z", "adxl345"),
    ];

    /// The corpus's section reads every one of its options, and its chips group
    /// as upstream's do.
    #[test]
    fn the_corpus_section_reads_every_option_and_groups_the_chips() {
        let mut text = String::from("[resonance_tester]\n");
        for (key, value) in CORPUS_OPTIONS {
            text.push_str(&format!("{key}: {value}\n"));
        }
        let (config, _) = Config::from_text(&text).expect("the section parses");
        let section = config.get_section("resonance_tester").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(section, Arc::clone(&access));

        let object = ResonanceTester::new(&wrapper, &printer()).expect("the section loads");

        assert_eq!(
            object.accel_chip_names,
            [
                ("xz".to_string(), "adxl345".to_string()),
                ("y".to_string(), "mpu9250 my_mpu".to_string()),
            ]
        );
        check_unused(&config, &access, &["resonance_tester".to_string()])
            .expect("no option is left unread");
    }

    /// `max_smoothing` is optional: a section without it loads, and the field
    /// stays unset (`resonance_tester.py:289`).
    #[test]
    fn the_section_loads_without_max_smoothing() {
        let object = ResonanceTester::new(
            &ConfigWrapper::untracked(&section(&[("accel_chip", "adxl345")])),
            &printer(),
        )
        .expect("the section loads without max_smoothing");

        assert_eq!(object.max_smoothing, None);
    }

    /// With `accel_chip_x` unset, `accel_chip` covers `x` and `y` and
    /// `accel_chip_y` is not required (`resonance_tester.py:264-273`).
    #[test]
    fn the_bare_accel_chip_does_not_need_the_per_axis_names() {
        let object = ResonanceTester::new(
            &ConfigWrapper::untracked(&section(&[("accel_chip", "adxl345")])),
            &printer(),
        )
        .expect("the section loads without accel_chip_y");

        // Grouped by chip name: the empty `accel_chip_z` sorts first.
        assert_eq!(
            object.accel_chip_names,
            [
                ("z".to_string(), String::new()),
                ("xy".to_string(), "adxl345".to_string()),
            ]
        );
    }

    /// A `probe_points` line that is not three floats is a config error.
    #[test]
    fn probe_points_must_have_three_elements_per_line() {
        let err = ResonanceTester::new(
            &ConfigWrapper::untracked(&section(&[
                ("accel_chip", "adxl345"),
                ("probe_points", "1,2"),
            ])),
            &printer(),
        )
        .err()
        .expect("the section is rejected");

        assert!(err.to_string().contains("must have 3 elements"), "{err}");
    }

    /// With `accel_chip_z` unset, the empty name it leaves behind is skipped by
    /// `connect`, which then reports the first real chip instead.
    #[test]
    fn connect_skips_the_empty_chip_name() {
        let (printer, _object) = loaded(&[
            ("accel_chip_x", "adxl345"),
            ("accel_chip_y", "mpu9250 my_mpu"),
        ]);

        printer.send_event(&KlippyEvent::KlippyConnect);

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert_eq!(state.message, "Unknown config object 'adxl345'");
    }

    /// A chip name that resolves to something that is not an accelerometer is
    /// refused (`resonance_tester.py:300-309`).
    #[test]
    fn connect_rejects_a_name_that_is_not_an_accelerometer() {
        /// Any registered object will do here; it is not an accelerometer.
        struct NotAChip;

        impl PrinterObject for NotAChip {
            fn get_status(&self, _eventtime: f64) -> Value {
                json!({})
            }
        }

        let (printer, _object) = loaded(&[("accel_chip", "not_a_chip")]);
        printer
            .add_object("not_a_chip", Arc::new(NotAChip))
            .unwrap();

        printer.send_event(&KlippyEvent::KlippyConnect);

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert_eq!(state.message, "'not_a_chip' is not an accelerometer");
    }

    /// A chip name nobody registered is refused with upstream's
    /// `Unknown config object '<name>'`.
    #[test]
    fn connect_reports_an_unknown_chip() {
        let (printer, _object) = loaded(&[("accel_chip", "ghost")]);

        printer.send_event(&KlippyEvent::KlippyConnect);

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert_eq!(state.message, "Unknown config object 'ghost'");
    }

    /// The three commands register under upstream's names, with upstream's help
    /// strings (`resonance_tester.py:292-298`).
    #[test]
    fn the_three_commands_register_with_upstreams_help() {
        let (printer, _object) = loaded(&[("accel_chip", "adxl345")]);
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");

        let help = gcode.command_help();
        assert_eq!(
            help.get("MEASURE_AXES_NOISE").map(String::as_str),
            Some("Measures noise of all enabled accelerometer chips")
        );
        assert_eq!(
            help.get("TEST_RESONANCES").map(String::as_str),
            Some("Runs the resonance test for a specified axis")
        );
        assert_eq!(
            help.get("SHAPER_CALIBRATE").map(String::as_str),
            Some("Similar to TEST_RESONANCES but suggest input shaper config")
        );
    }

    /// Each command accepts its parameters and then reports the data-path gap.
    #[test]
    fn the_commands_report_the_data_path_gap() {
        let (printer, _object) = loaded(&[("accel_chip", "adxl345")]);
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");

        for script in [
            "MEASURE_AXES_NOISE",
            "TEST_RESONANCES AXIS=X",
            "SHAPER_CALIBRATE",
        ] {
            let err = gcode.run_script_sync(script).unwrap_err();
            assert_eq!(err.message(), DATA_PATH_GAP, "{script}");
        }
    }

    /// The parameters upstream rejects are rejected here with upstream's
    /// wording, before the gap is reported.
    #[test]
    fn the_commands_keep_upstreams_parameter_errors() {
        let (printer, _object) = loaded(&[("accel_chip", "adxl345")]);
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");

        let cases = [
            (
                "TEST_RESONANCES AXIS=X POINT=1,2",
                "Invalid POINT parameter, must be 'x,y,z'",
            ),
            (
                "TEST_RESONANCES AXIS=X OUTPUT=bogus",
                "Unsupported output 'bogus', only 'resonances' and 'raw_data' are supported",
            ),
            // The axis is quoted as written, as upstream's message does.
            ("SHAPER_CALIBRATE AXIS=Q", "Unsupported axis 'Q'"),
            (
                "TEST_RESONANCES AXIS=X NAME=no/good",
                "Invalid NAME parameter",
            ),
        ];
        for (script, message) in cases {
            let err = gcode.run_script_sync(script).unwrap_err();
            assert_eq!(err.message(), message, "{script}");
        }
    }
}
