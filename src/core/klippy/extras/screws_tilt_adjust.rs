//! `[screws_tilt_adjust]` — calculate bed-screw turns from probed points
//! (upstream `klippy/extras/screws_tilt_adjust.py`).
//!
//! The section reads `screw1`..`screw99` (stopping at the first missing one),
//! refuses fewer than three screws, and hands the screw coordinates to
//! [`ProbePointsHelper`] as the default probe points. `SCREWS_TILT_CALCULATE`
//! probes them and [`ScrewsTiltAdjust::run_finalize`] turns the probed Z into
//! one report line per screw: the base screw (the first one, or the extreme
//! `DIRECTION` names) plus every other screw's turn adjustment as `HH:MM`,
//! where a full turn is `60` minutes of the thread's pitch
//! (`threads_factor`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `screw1`..`screw99` | — | two floats each; **stops at the first missing index** |
//! | `screwN_name` | `screw at %.3f,%.3f` | the report line's name |
//! | `screw_thread` | `CW-M3` | the pitch/direction table (below) |
//! | `points` | the screws | probe points ([`ProbePointsHelper`]) |
//! | `horizontal_move_z` / `speed` | `5.` / `50.` | the travels ([`ProbePointsHelper`]) |
//!
//! `SCREWS_TILT_CALCULATE` takes `MAX_DEVIATION` (a limit whose breach is the
//! round's deferred error — see [`ScrewsTiltAdjust::probe_finalize`]) and
//! `DIRECTION` (`CW`/`CCW`, which also picks the base screw as the highest or
//! lowest Z depending on the thread's own direction).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock, Weak};

use serde_json::{json, Map, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::probe::{
    probe_points_params, ProbeOffsets, ProbePointsFinalize, ProbePointsHelper,
};
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("screws_tilt_adjust", order = 30, load = load_config);

/// The command this section registers (`screws_tilt_adjust.py:43`).
const COMMAND: &str = "SCREWS_TILT_CALCULATE";

/// The `KEY` names `SCREWS_TILT_CALCULATE` reads, in read order: the
/// deviation limit and direction the handler reads first, then the probe
/// round's (`screws_tilt_adjust.py:47-54`).
fn command_params() -> Vec<&'static str> {
    let mut params = vec!["MAX_DEVIATION", "DIRECTION"];
    params.extend(probe_points_params());
    params
}

/// The `screw_thread` choices mapped to their upstream index
/// (`screws_tilt_adjust.py:30-32`).
const THREADS: &[(&str, u8)] = &[
    ("CW-M3", 0),
    ("CCW-M3", 1),
    ("CW-M4", 2),
    ("CCW-M4", 3),
    ("CW-M5", 4),
    ("CCW-M5", 5),
    ("CW-M6", 6),
    ("CCW-M6", 7),
];

/// The `screw_thread` choice list, in table order (`getchoice`'s `choices`).
fn thread_choices() -> Vec<&'static str> {
    THREADS.iter().map(|(name, _)| *name).collect()
}

/// Millimetres per full turn for a thread index
/// (`screws_tilt_adjust.py:69-71`, including upstream's `0.5` fallback for a
/// value outside the table — unreachable, since the choice validates it).
fn threads_factor(thread: u8) -> f64 {
    const FACTORS: [f64; 8] = [0.5, 0.5, 0.7, 0.7, 0.8, 0.8, 1.0, 1.0];
    FACTORS.get(thread as usize).copied().unwrap_or(0.5)
}

/// The `[screws_tilt_adjust]` section: the screws, the thread, the probe
/// helper behind `SCREWS_TILT_CALCULATE`, and the status
/// (`screws_tilt_adjust.py:5-74`).
pub struct ScrewsTiltAdjust {
    /// The machine, for the report lines and errors (`gcode`).
    printer: Weak<Printer>,
    /// `(coordinate, name)` per `screwN`, in order.
    screws: Vec<((f64, f64), String)>,
    /// The `screw_thread` index into [`THREADS`].
    thread: u8,
    /// Wired after construction: the finalize callback holds a `Weak` to this
    /// section, so the helper cannot be a field built before the callback
    /// exists (the [`ZTilt`](crate::core::klippy::extras::z_tilt) pattern).
    probe_helper: OnceLock<Arc<ProbePointsHelper>>,
    /// The running command's `MAX_DEVIATION` (`None` when it was absent).
    max_diff: Mutex<Option<f64>>,
    /// The running command's `DIRECTION` (`None` when it was absent).
    direction: Mutex<Option<&'static str>>,
    /// The `results` half of `get_status`, rebuilt every round.
    results: Mutex<Value>,
    /// Whether the last round breached `MAX_DEVIATION`
    /// (`screws_tilt_adjust.py: max_diff_error`).
    max_diff_error: AtomicBool,
    /// The first finalize error, for the command to report — an automatic
    /// round's callback cannot raise into the probing loop itself (see
    /// [`ScrewsTiltAdjust::probe_finalize`]).
    last_error: Mutex<Option<CommandError>>,
}

impl ScrewsTiltAdjust {
    /// Read the section, wire the probe helper's callback, and register
    /// `SCREWS_TILT_CALCULATE` (`screws_tilt_adjust.py:6-45`).
    ///
    /// # Errors
    /// A malformed option, fewer than three screws, fewer than three probe
    /// points (`probe.py:minimum_points`), or a command name that is taken.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let screws = read_screws(config)?;
        let choices = thread_choices();
        let thread_name = config.get_choice("screw_thread", &choices, Some("CW-M3"))?;
        let thread = THREADS
            .iter()
            .find(|(name, _)| *name == thread_name)
            .map(|(_, index)| *index)
            .expect("get_choice validated the name against the same table");
        let points: Vec<(f64, f64)> = screws.iter().map(|(coord, _)| *coord).collect();

        let this = Arc::new(Self {
            printer: Arc::downgrade(printer),
            screws,
            thread,
            probe_helper: OnceLock::new(),
            max_diff: Mutex::new(None),
            direction: Mutex::new(None),
            results: Mutex::new(json!({})),
            max_diff_error: AtomicBool::new(false),
            last_error: Mutex::new(None),
        });

        // The callback reaches the section through a weak reference: the
        // helper it builds would otherwise outlive nothing while the section
        // holds the helper (a reference circle).
        let finalize: ProbePointsFinalize = Arc::new({
            let weak = Arc::downgrade(&this);
            move |offsets, positions| {
                let Some(this) = weak.upgrade() else {
                    warn!("SCREWS_TILT_CALCULATE finalize: the screws_tilt_adjust object is gone");
                    return None;
                };
                this.probe_finalize(offsets, positions)
            }
        });
        let probe_helper =
            ProbePointsHelper::with_default_points(config, printer, finalize, Some(points))?;
        probe_helper.minimum_points(3)?;
        this.probe_helper
            .set(probe_helper)
            .unwrap_or_else(|_| unreachable!("the probe helper is wired once"));

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        let weak = Arc::downgrade(&this);
        let handler: CommandHandler = Arc::new(move |gcmd| {
            let weak = weak.clone();
            Box::pin(async move {
                let this = weak
                    .upgrade()
                    .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                this.cmd_screws_tilt_calculate(gcmd).await
            })
        });
        gcode
            .register_command_with_params(
                COMMAND,
                handler,
                Some(
                    "Tool to help adjust bed leveling screws by calculating \
                     the number of turns to level it.",
                ),
                &command_params(),
                false,
            )
            .map_err(ConfigError::new)?;

        Ok(this)
    }

    /// `SCREWS_TILT_CALCULATE` (`screws_tilt_adjust.py:47-54`): read
    /// `MAX_DEVIATION` and `DIRECTION`, then run a probe round.
    ///
    /// # Errors
    /// An unparsable `MAX_DEVIATION`, a `DIRECTION` that is neither `CW` nor
    /// `CCW`, whatever the round reports — including the first error the
    /// finalize callback recorded (an automatic round's callback cannot raise
    /// into the loop itself; see [`ScrewsTiltAdjust::probe_finalize`]).
    async fn cmd_screws_tilt_calculate(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        // Upstream's `gcmd.get_float("MAX_DEVIATION", None)`: absent means
        // "no limit", which the required-parameter getter cannot spell — its
        // `None` default is a missing-parameter error.
        let max_diff = if gcmd.get_command_parameters().contains_key("MAX_DEVIATION") {
            Some(gcmd.get_float("MAX_DEVIATION")?)
        } else {
            None
        };
        let direction = match gcmd.get_command_parameters().get("DIRECTION") {
            None => None,
            Some(raw) => match raw.to_uppercase().as_str() {
                "CW" => Some("CW"),
                "CCW" => Some("CCW"),
                _ => {
                    return Err(CommandError::new(format!(
                        "Error on '{}': DIRECTION must be either CW or CCW",
                        gcmd.commandline()
                    )));
                }
            },
        };
        *self.max_diff.lock().unwrap_or_else(|p| p.into_inner()) = max_diff;
        *self.direction.lock().unwrap_or_else(|p| p.into_inner()) = direction;
        *self.last_error.lock().unwrap_or_else(|p| p.into_inner()) = None;

        let probe_helper = self
            .probe_helper
            .get()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        probe_helper.start_probe(gcmd).await?;
        if let Some(error) = self
            .last_error
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
        {
            return Err(error);
        }
        Ok(())
    }

    /// The probe helper's finalize callback: report the round, and on error
    /// record the first one for [`ScrewsTiltAdjust::cmd_screws_tilt_calculate`]
    /// to report, because this port's callback signature cannot raise into the
    /// probing loop the way upstream's exception does.
    fn probe_finalize(&self, offsets: ProbeOffsets, positions: &[Coord]) -> Option<&'static str> {
        let _ = offsets; // Upstream's callback reads only `positions`.
        match self.run_finalize(positions) {
            Ok(()) => None,
            Err(error) => {
                warn!("SCREWS_TILT_CALCULATE: {error}");
                let mut slot = self.last_error.lock().unwrap_or_else(|p| p.into_inner());
                if slot.is_none() {
                    *slot = Some(error);
                }
                None
            }
        }
    }

    /// One round's work (`screws_tilt_adjust.py:66-127`): compute the report
    /// ([`round_report`]), send its lines, store `results`, then fail with the
    /// deferred `MAX_DEVIATION` error when the round breached the limit.
    fn run_finalize(&self, positions: &[Coord]) -> Result<(), CommandError> {
        let zs: Vec<f64> = positions.iter().map(Coord::z).collect();
        let direction = *self.direction.lock().unwrap_or_else(|p| p.into_inner());
        let max_diff = *self.max_diff.lock().unwrap_or_else(|p| p.into_inner());
        let outcome = round_report(&self.screws, self.thread, direction, &zs, max_diff)?;

        if let Some(printer) = self.printer.upgrade() {
            if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                for line in &outcome.lines {
                    gcode.respond_info(line, true);
                }
            }
        }
        *self.results.lock().unwrap_or_else(|p| p.into_inner()) = Value::Object(outcome.results);
        self.max_diff_error
            .store(outcome.error.is_some(), Ordering::SeqCst);
        match outcome.error {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    /// The status dict (`screws_tilt_adjust.py:69-72`).
    fn status(&self) -> Value {
        json!({
            "error": self.max_diff_error.load(Ordering::SeqCst),
            "max_deviation": *self.max_diff.lock().unwrap_or_else(|p| p.into_inner()),
            "results": *self.results.lock().unwrap_or_else(|p| p.into_inner()),
        })
    }
}

/// Read `screw1`..`screw99`, stopping at the first missing index
/// (`screws_tilt_adjust.py:15-29`).
///
/// # Errors
/// "Unable to parse option …" for a coordinate that is not a number, "must
/// have 2 elements" for a coordinate list of another length, and
/// "\<section\>: Must have at least three screws" below three.
fn read_screws(config: &ConfigWrapper) -> Result<Vec<((f64, f64), String)>, ConfigError> {
    let identifier = config.identifier();
    let mut screws = Vec::new();
    for index in 1..=99 {
        let prefix = format!("screw{index}");
        let Some(raw) = config.get_str(&prefix) else {
            break;
        };
        let parts: Vec<&str> = raw.split(',').map(str::trim).collect();
        let parsed: Vec<f64> = parts
            .iter()
            .map(|part| {
                part.parse::<f64>().map_err(|_| {
                    ConfigError::new(format!(
                        "Unable to parse option '{prefix}' in section '{identifier}'"
                    ))
                })
            })
            .collect::<Result<_, _>>()?;
        if parsed.len() != 2 {
            return Err(ConfigError::new(format!(
                "Option '{prefix}' in section '{identifier}' must have 2 elements"
            )));
        }
        let coord = (parsed[0], parsed[1]);
        let default_name = format!("screw at {:.3},{:.3}", coord.0, coord.1);
        let name = config.get(&format!("{prefix}_name"), Some(&default_name))?;
        screws.push((coord, name));
    }
    if screws.len() < 3 {
        return Err(ConfigError::new(format!(
            "{identifier}: Must have at least three screws"
        )));
    }
    Ok(screws)
}

/// One round's report lines and `results` entries
/// (`screws_tilt_adjust.py:66-127`).
#[derive(Debug)]
struct RoundReport {
    /// Everything to `respond_info`, in order: the reading hint, the base
    /// screw, then every other screw's adjustment.
    lines: Vec<String>,
    /// The `results` status map, keyed `screwN`.
    results: Map<String, Value>,
    /// The deferred `MAX_DEVIATION` error, when the round breached it.
    error: Option<CommandError>,
}

/// Turn the probed Z of every screw into the report (`screws_tilt_adjust.py:
/// 66-133`): pick the base screw, format each line, and check `max_diff`.
///
/// `zs` holds one Z per probe point — upstream's `bed_z`, which carries the
/// probe offsets netted out; this port hands the positions' Z through, and
/// the offsets are one constant across the round, so every difference and the
/// `MAX_DEVIATION` check read the same either way.
///
/// # Errors
/// Fewer positions than screws (upstream would index past the list), or a
/// breach of `max_diff` — the latter also set on the returned outcome.
fn round_report(
    screws: &[((f64, f64), String)],
    thread: u8,
    direction: Option<&'static str>,
    zs: &[f64],
    max_diff: Option<f64>,
) -> Result<RoundReport, CommandError> {
    if zs.len() < screws.len() {
        return Err(CommandError::new(format!(
            "Internal probe error - {} positions for {} screws",
            zs.len(),
            screws.len()
        )));
    }
    let is_clockwise_thread = (thread & 1) == 0;

    // The base position: the extreme Z a `DIRECTION` names — highest for a
    // clockwise thread turned CW (or a counter-clockwise thread turned CCW),
    // lowest otherwise — or the first screw (`screws_tilt_adjust.py:73-86`).
    let (i_base, z_base) = match direction {
        Some(direction) => {
            let use_max = (is_clockwise_thread && direction == "CW")
                || (!is_clockwise_thread && direction == "CCW");
            let mut best = (0usize, zs[0]);
            for (index, &z) in zs.iter().enumerate().skip(1) {
                if (use_max && z > best.1) || (!use_max && z < best.1) {
                    best = (index, z);
                }
            }
            best
        }
        None => (0, zs[0]),
    };

    let mut lines = vec!["01:20 means 1 full turn and 20 minutes, CW=clockwise, \
         CCW=counter-clockwise"
        .to_string()];
    let mut results = Map::new();
    let mut screw_diff: Vec<f64> = Vec::new();
    for (i, (coord, name)) in screws.iter().enumerate() {
        let z = zs[i];
        let key = format!("screw{}", i + 1);
        if i == i_base {
            lines.push(format!(
                "{} : x={:.1}, y={:.1}, z={:.5}",
                format!("{name} (base)"),
                coord.0,
                coord.1,
                z
            ));
            let sign = if is_clockwise_thread { "CW" } else { "CCW" };
            results.insert(
                key,
                json!({ "z": z, "sign": sign, "adjust": "00:00", "is_base": true }),
            );
            continue;
        }
        let diff = z_base - z;
        screw_diff.push(diff.abs());
        // Below a micrometre the screw stands level (`screws_tilt_adjust.py:
        // 100-104`); the sign reads the *unclamped* adjust, so a nearly-level
        // screw counts as turned positively.
        let signed_adjust = if diff.abs() < 0.001 {
            0.0
        } else {
            diff / threads_factor(thread)
        };
        let sign = match (is_clockwise_thread, signed_adjust >= 0.) {
            (true, true) | (false, false) => "CW",
            (true, false) | (false, true) => "CCW",
        };
        let adjust = signed_adjust.abs();
        let full_turns = adjust.trunc() as i64;
        // Upstream: `minutes = round(decimal_part * 60, 0)` — Python's
        // round-half-to-even on the exact binary value.
        let minutes = round_half_even((adjust - full_turns as f64) * 60.0);
        let stamp = format!("{full_turns:02}:{minutes:02}");
        lines.push(format!(
            "{} : x={:.1}, y={:.1}, z={:.5} : adjust {sign} {stamp}",
            name, coord.0, coord.1, z
        ));
        results.insert(
            key,
            json!({ "z": z, "sign": sign, "adjust": stamp, "is_base": false }),
        );
    }

    // `if self.max_diff and …`: upstream's truth test skips `0.0` as well as
    // `None`, and only the non-base screws carry a difference.
    let breach = max_diff
        .filter(|limit| *limit != 0.0)
        .is_some_and(|limit| screw_diff.iter().any(|diff| *diff > limit));
    let error = breach.then(|| {
        CommandError::new(format!(
            "bed level exceeds configured limits ({}mm)! \
             Adjust screws and restart print.",
            max_diff.unwrap_or_default()
        ))
    });
    Ok(RoundReport {
        lines,
        results,
        error,
    })
}

/// Python's `round(value, 0)` (`round-half-to-even`, returning the integer
/// it lands on) for the non-negative minute counts upstream formats.
fn round_half_even(value: f64) -> i64 {
    let floor = value.floor();
    let base = floor as i64;
    match value - floor {
        fraction if fraction > 0.5 => base + 1,
        fraction if fraction < 0.5 => base,
        // Exactly half: keep the even neighbour.
        _ if base % 2 == 0 => base,
        _ => base + 1,
    }
}

impl PrinterObject for ScrewsTiltAdjust {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.status()
    }
}

/// The factory `section!` names (`screws_tilt_adjust.py:load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = ScrewsTiltAdjust::new(config, printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[screws_tilt_adjust]` section with the given options, as the parser
    /// would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("screws_tilt_adjust", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `gcode` registered, as the loader builds it.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode registers");
        printer
    }

    /// Three screws, the section's minimum.
    fn three_screws() -> Vec<((f64, f64), String)> {
        [(10., 30.), (155., 30.), (155., 190.)]
            .iter()
            .map(|&(x, y)| ((x, y), format!("screw at {x:.3},{y:.3}")))
            .collect()
    }

    // --- Config parsing ---------------------------------------------------

    /// `screwN` is read from 1 up to the first missing index, and a gap stops
    /// the scan even when a higher screw exists (`screws_tilt_adjust.py:16-23`).
    #[test]
    fn screws_stop_at_the_first_missing_index() {
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw1_name", "front left"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
            ("screw5", "10,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let screws = read_screws(&config).expect("three screws read");
        assert_eq!(screws.len(), 3, "screw5 must not be read past the gap");
        assert_eq!(screws[0].1, "front left");
        // No `screwN_name`: upstream's default from the coordinates.
        assert_eq!(screws[1].1, "screw at 155.000,30.000");
        assert_eq!(screws[2].0, (155., 190.));

        // A section whose `screw1`..`screw2` stop early has a gap at 3 while
        // `screw4` exists: two screws is still "fewer than three".
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw4", "10,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let err = read_screws(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "screws_tilt_adjust: Must have at least three screws"
        );
    }

    /// Fewer than three screws is a config error with upstream's wording
    /// (`screws_tilt_adjust.py:27-29`), and a coordinate list of any other
    /// length or a non-number keeps the parser's wording.
    #[test]
    fn fewer_than_three_screws_and_malformed_coordinates_are_refused() {
        let sect = section(&[("screw1", "10,30"), ("screw2", "155,30")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            read_screws(&config).unwrap_err().to_string(),
            "screws_tilt_adjust: Must have at least three screws"
        );

        let sect = section(&[("screw1", "10")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            read_screws(&config).unwrap_err().to_string(),
            "Option 'screw1' in section 'screws_tilt_adjust' must have 2 elements"
        );

        let sect = section(&[("screw1", "left,30")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            read_screws(&config).unwrap_err().to_string(),
            "Unable to parse option 'screw1' in section 'screws_tilt_adjust'"
        );
    }

    /// The `screw_thread` choice table and its `threads_factor`, plus the
    /// invalid-choice wording (`screws_tilt_adjust.py:30-37, 69-71`).
    #[test]
    fn the_thread_choice_table_and_its_factors_match_upstream() {
        let choices = thread_choices();
        assert_eq!(
            choices,
            ["CW-M3", "CCW-M3", "CW-M4", "CCW-M4", "CW-M5", "CCW-M5", "CW-M6", "CCW-M6"]
        );
        assert_eq!(
            [
                threads_factor(0),
                threads_factor(1),
                threads_factor(2),
                threads_factor(3),
                threads_factor(4),
                threads_factor(5),
                threads_factor(6),
                threads_factor(7)
            ],
            [0.5, 0.5, 0.7, 0.7, 0.8, 0.8, 1.0, 1.0]
        );
        // A thread index is even exactly for the `CW` names.
        for (index, (name, _)) in THREADS.iter().enumerate() {
            assert_eq!(((index as u8) & 1) == 0, name.starts_with("CW"));
        }

        let sect = section(&[
            ("screw1", "0,0"),
            ("screw2", "1,0"),
            ("screw3", "0,1"),
            ("screw_thread", "CW-M7"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let err = config
            .get_choice("screw_thread", &thread_choices(), Some("CW-M3"))
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice 'CW-M7' for option 'screw_thread' in section 'screws_tilt_adjust' \
             is not a valid choice"
        );
    }

    // --- The report -------------------------------------------------------

    /// The base screw is the first one without `DIRECTION`
    /// (`screws_tilt_adjust.py:84-86`), and its line carries ` (base)` with
    /// the thread's own direction as the sign (`:93-97`).
    #[test]
    fn without_direction_the_first_screw_is_the_base() {
        let screws = three_screws();
        let zs = [2.0, 2.1, 1.9];
        let report = round_report(&screws, 0, None, &zs, None).expect("a report");
        assert!(report.error.is_none());
        assert_eq!(
            report.lines,
            [
                "01:20 means 1 full turn and 20 minutes, CW=clockwise, \
                 CCW=counter-clockwise",
                "screw at 10.000,30.000 (base) : x=10.0, y=30.0, z=2.00000",
                // diff = 2.0 - 2.1 = -0.1 → adjust -0.2 turns → CCW 00:12.
                "screw at 155.000,30.000 : x=155.0, y=30.0, z=2.10000 \
                 : adjust CCW 00:12",
                // diff = 0.1 → adjust 0.2 turns → CW 00:12.
                "screw at 155.000,190.000 : x=155.0, y=190.0, z=1.90000 \
                 : adjust CW 00:12",
            ]
        );
        let base = &report.results["screw1"];
        assert_eq!(base["is_base"], json!(true));
        assert_eq!(base["sign"], json!("CW"));
        assert_eq!(base["adjust"], json!("00:00"));
        assert_eq!(report.results["screw2"]["sign"], json!("CCW"));
        assert_eq!(report.results["screw2"]["adjust"], json!("00:12"));
    }

    /// `DIRECTION` picks the base as the extreme Z: highest when it agrees
    /// with the thread's own direction, lowest otherwise
    /// (`screws_tilt_adjust.py:76-82`); a counter-clockwise thread flips the
    /// agreement (`sign` starts at `CCW` too).
    #[test]
    fn direction_picks_the_extreme_screw_as_the_base() {
        let screws = three_screws();
        let zs = [2.0, 2.1, 1.9];

        // CW thread, DIRECTION=CW → highest z is screw2 (2.1); the base line
        // sits at its own screw's place in the loop, so lines[2].
        let report = round_report(&screws, 0, Some("CW"), &zs, None).expect("a report");
        assert_eq!(
            report.lines[2],
            "screw at 155.000,30.000 (base) : x=155.0, y=30.0, z=2.10000"
        );
        assert_eq!(report.results["screw2"]["is_base"], json!(true));

        // CW thread, DIRECTION=CCW → lowest z is screw3 (1.9), lines[3].
        let report = round_report(&screws, 0, Some("CCW"), &zs, None).expect("a report");
        assert_eq!(
            report.lines[3],
            "screw at 155.000,190.000 (base) : x=155.0, y=190.0, z=1.90000"
        );

        // CCW thread (index 1), DIRECTION=CCW → still the highest.
        let report = round_report(&screws, 1, Some("CCW"), &zs, None).expect("a report");
        assert_eq!(report.results["screw2"]["is_base"], json!(true));
        // Its base sign is the thread's own direction.
        assert_eq!(report.results["screw2"]["sign"], json!("CCW"));

        // CCW thread, DIRECTION=CW → the lowest.
        let report = round_report(&screws, 1, Some("CW"), &zs, None).expect("a report");
        assert_eq!(report.results["screw3"]["is_base"], json!(true));
    }

    /// Turn math: `HH:MM` zero-padded (`%02d:%02d`), Python's round-half-to-
    /// even on the minutes, and the `0.001` threshold that clamps the adjust
    /// but not the sign's raw direction (`screws_tilt_adjust.py:98-121`).
    #[test]
    fn the_adjust_is_padded_and_rounded_the_way_python_rounds() {
        let screws = three_screws();

        // Base 2.000; diff 0.75 / 0.5 (CW-M3) = 1.5 turns → "01:30".
        let report = round_report(&screws, 0, None, &[2.0, 1.25, 2.0], None).expect("a report");
        assert_eq!(
            report.lines[2],
            "screw at 155.000,30.000 : x=155.0, y=30.0, z=1.25000 : adjust CW 01:30"
        );
        assert_eq!(report.results["screw2"]["adjust"], json!("01:30"));

        // Minutes land on exact halves: 7.5 → 08 (even), 22.5 → 22 (even),
        // as Python's `round(x, 0)` does.
        let report = round_report(&screws, 0, None, &[2.0, 2.0 - 0.0625, 2.0 - 0.1875], None)
            .expect("a report");
        assert_eq!(report.results["screw2"]["adjust"], json!("00:08"));
        assert_eq!(report.results["screw3"]["adjust"], json!("00:22"));

        // Below 0.001 the adjust clamps to zero — even a negative difference
        // signs as positive (`adjust = 0` passes the `>= 0` test).
        let report =
            round_report(&screws, 0, None, &[2.0, 2.0005, 1.9995], None).expect("a report");
        assert_eq!(report.results["screw2"]["adjust"], json!("00:00"));
        assert_eq!(report.results["screw2"]["sign"], json!("CW"));
        assert_eq!(report.results["screw3"]["adjust"], json!("00:00"));
        assert_eq!(report.results["screw3"]["sign"], json!("CW"));

        // Exactly 0.001 is *not* below the threshold: bit-exact here as
        // `0.0 - 0.001` (subtracting from zero cannot shift the value), so it
        // adjusts — still 0 minutes, but the sign follows the difference.
        let report = round_report(&screws, 0, None, &[0.0, 0.001, 0.0], None).expect("a report");
        assert_eq!(report.results["screw2"]["sign"], json!("CCW"));
        assert_eq!(report.results["screw2"]["adjust"], json!("00:00"));

        // A counter-clockwise thread flips every sign (same factor, same
        // minutes).
        let report = round_report(&screws, 1, None, &[2.0, 1.9, 2.0], None).expect("a report");
        assert_eq!(report.results["screw2"]["sign"], json!("CCW"));
        assert_eq!(report.results["screw2"]["adjust"], json!("00:12"));
    }

    /// `MAX_DEVIATION` fails the round with upstream's wording, comparing
    /// only the non-base screws' raw differences — truthy `max_diff` only,
    /// so `0.0` disables the check like Python's `if self.max_diff`
    /// (`screws_tilt_adjust.py:123-127`).
    #[test]
    fn max_deviation_fails_the_round_with_upstream_wording() {
        let screws = three_screws();
        let zs = [2.0, 2.05, 2.0];

        let report = round_report(&screws, 0, None, &zs, Some(0.01)).expect("computed");
        let error = report.error.expect("0.05 exceeds 0.01");
        assert_eq!(
            error.to_string(),
            "bed level exceeds configured limits (0.01mm)! \
             Adjust screws and restart print."
        );
        // The report itself was produced before the raise
        // (`screws_tilt_adjust.py:93-121` runs first).
        assert_eq!(report.results.len(), 3);
        assert_eq!(report.lines.len(), 4);

        // A difference exactly at the limit does not breach (`d > max_diff`):
        // the limit *is* the round's own difference, bit for bit.
        let zs: [f64; 3] = [2.0, 1.97, 2.0];
        let exact = (zs[0] - zs[1]).abs();
        let report = round_report(&screws, 0, None, &zs, Some(exact)).expect("computed");
        assert!(report.error.is_none());
        // Just under it, the same round fails.
        let report = round_report(&screws, 0, None, &zs, Some(exact * 0.5)).expect("computed");
        assert!(report.error.is_some());

        // `0.0` is falsy in Python: no check at all.
        let report = round_report(&screws, 0, None, &zs, Some(0.0)).expect("computed");
        assert!(report.error.is_none());

        let report = round_report(&screws, 0, None, &zs, None).expect("computed");
        assert!(report.error.is_none());
    }

    /// Fewer positions than screws is refused rather than indexed past
    /// (upstream would raise `IndexError` inside the callback).
    #[test]
    fn fewer_positions_than_screws_is_refused() {
        let screws = three_screws();
        let err = round_report(&screws, 0, None, &[2.0, 2.1], None).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Internal probe error - 2 positions for 3 screws"
        );
    }

    // --- Wiring ------------------------------------------------------------

    /// The section wires up and reports the status shape upstream's
    /// `get_status` answers: `error`, `max_deviation`, `results`
    /// (`screws_tilt_adjust.py:69-72`), before any round and after one.
    #[test]
    fn the_status_shape_matches_upstream_before_and_after_a_round() {
        let printer = printer();
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw1_name", "front left"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
            ("screw_thread", "CW-M3"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let this = ScrewsTiltAdjust::new(&config, &printer).expect("the section loads");

        assert_eq!(
            this.status(),
            json!({ "error": false, "max_deviation": null, "results": {} })
        );
        // The command registered with upstream's help text.
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        assert_eq!(
            gcode.command_help().get(COMMAND).map(String::as_str),
            Some(
                "Tool to help adjust bed leveling screws by calculating \
                 the number of turns to level it."
            )
        );

        // One round as the command would arm it: a breach flips `error` and
        // keeps the results the lines reported.
        *this.max_diff.lock().unwrap_or_else(|p| p.into_inner()) = Some(0.01);
        let positions = [
            Coord::new(10., 30., 2.0, 0.),
            Coord::new(155., 30., 2.05, 0.),
            Coord::new(155., 190., 2.0, 0.),
        ];
        let err = this.run_finalize(&positions).unwrap_err();
        assert_eq!(
            err.to_string(),
            "bed level exceeds configured limits (0.01mm)! \
             Adjust screws and restart print."
        );
        let status = this.status();
        assert_eq!(status["error"], json!(true));
        assert_eq!(status["max_deviation"], json!(0.01));
        assert_eq!(status["results"]["screw1"]["is_base"], json!(true));
        // diff = 2.0 - 2.05 → a negative adjust signs CCW on a CW thread.
        assert_eq!(status["results"]["screw2"]["sign"], json!("CCW"));

        // The callback defers the error instead of raising into the loop:
        // it answers `None` (no retry) and records the first error for the
        // command to report.
        *this.last_error.lock().unwrap_or_else(|p| p.into_inner()) = None;
        let answer = this.probe_finalize(
            ProbeOffsets {
                x: 0.,
                y: 0.,
                z: 1.15,
            },
            &positions,
        );
        assert_eq!(answer, None);
        let recorded = this.last_error.lock().unwrap_or_else(|p| p.into_inner());
        assert_eq!(
            recorded.as_ref().map(std::string::ToString::to_string),
            Some(
                "bed level exceeds configured limits (0.01mm)! \
                 Adjust screws and restart print."
                    .to_string()
            )
        );
    }

    /// A `DIRECTION` that is neither `CW` nor `CCW` fails the command with
    /// upstream's wording before any probing starts
    /// (`screws_tilt_adjust.py:49-53`).
    #[tokio::test]
    async fn a_bad_direction_fails_the_command_before_probing() {
        let printer = printer();
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let this = ScrewsTiltAdjust::new(&config, &printer).expect("the section loads");

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let mut params = std::collections::HashMap::new();
        params.insert("DIRECTION".to_string(), "sideways".to_string());
        let gcmd =
            gcode.create_gcode_command(COMMAND, "SCREWS_TILT_CALCULATE DIRECTION=sideways", params);

        let err = this.cmd_screws_tilt_calculate(&gcmd).await.unwrap_err();
        assert_eq!(
            err.to_string(),
            "Error on 'SCREWS_TILT_CALCULATE DIRECTION=sideways': \
             DIRECTION must be either CW or CCW"
        );
        // Nothing was armed: the status still reports no limit.
        assert_eq!(this.status()["max_deviation"], json!(null));
    }
}
