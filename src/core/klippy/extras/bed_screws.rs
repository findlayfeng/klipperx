//! `[bed_screws]` — the bed-screws helper's section
//! (upstream `klippy/extras/bed_screws.py`).
//!
//! The section reads `screw1`..`screw99`, stopping at the first missing one
//! (`bed_screws.py:15-18`); each screw carries an optional `screwN_name`
//! (default `screw at %.3f,%.3f` from its own coordinates) and an optional
//! `screwN_fine_adjust` coordinate recorded under the *coarse* screw's name
//! (`:19-26`). Fewer than three screws is a config error (`:27-28`), then the
//! travels are read: `speed` (`50.`), `probe_speed` (`5.`, upstream's
//! `lift_speed`), `horizontal_move_z` (`5.`) and `probe_height` (`0.`,
//! upstream's `probe_z`) — `speed`/`probe_speed` must be above `0`
//! (`:31-34`). Every option is read through the tracking wrapper, which is
//! what lets [`check_unused`](crate::core::klippy::config::check_unused)
//! accept the section.
//!
//! | option | default | role |
//! |---|---|---|
//! | `screw1`..`screw99` | — | two floats each; **stops at the first missing index** |
//! | `screwN_name` | `screw at %.3f,%.3f` | the screw's display name |
//! | `screwN_fine_adjust` | — | the fine pass's coordinate for that screw |
//! | `speed` / `probe_speed` | `50.` / `5.` | the travels, both above `0` |
//! | `horizontal_move_z` / `probe_height` | `5.` / `0.` | the lift and the descent |
//!
//! `BED_SCREWS_ADJUST` is registered at load (`:37-39`). Running it starts a
//! session: the toolhead lifts to `horizontal_move_z`, moves over screw 1 and
//! descends to `probe_height`, and the three session commands `ACCEPT` /
//! `ADJUSTED` / `ABORT` are registered (`:44-73`). `ACCEPT` walks to the next
//! coarse screw; once every screw is accepted and the section has a
//! `screwN_fine_adjust` pass, the count resets and the fine pass runs the same
//! way; a completed pass resets the session, lifts the toolhead and reports
//! `Bed screws tool completed successfully` (`:81-105`). `ADJUSTED` is
//! `ACCEPT` with the accepted count dropped to `-1` first, so a significant
//! screw turn starts the count over (`:111-115`). `ABORT` ends the session
//! without a report (`:117-120`). Any toolhead refusal during a move drops the
//! session — the commands are unregistered and the state reset — before the
//! error is re-raised (`:44-51`).
//!
//! The section loads in the generic phase, *before* `toolhead` (late phase), so
//! the toolhead is looked up when a command runs, not at load.
//!
//! `ACCEPT`/`ABORT` are the same names `manual_probe` registers for its own
//! session. Upstream lets the first registrant win; this port keeps that, and
//! additionally takes back the commands it did manage to register when the
//! second one collides, so a refused session never leaves a half-registered
//! trio behind (upstream leaves the earlier ones registered).

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("bed_screws", order = 30, load = load_config);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The command that starts a session, registered at load
/// (`bed_screws.py:37-39`).
const BED_SCREWS_ADJUST: &str = "BED_SCREWS_ADJUST";

/// The three commands a running session registers (`bed_screws.py:64-73`).
const ACCEPT: &str = "ACCEPT";
const ADJUSTED: &str = "ADJUSTED";
const ABORT: &str = "ABORT";

/// `cmd_BED_SCREWS_ADJUST_help` (`bed_screws.py:80`).
const BED_SCREWS_ADJUST_HELP: &str = "Tool to help adjust bed leveling screws";
/// `cmd_ACCEPT_help` (`bed_screws.py:83`).
const ACCEPT_HELP: &str = "Accept bed screw position";
/// `cmd_ADJUSTED_help` (`bed_screws.py:110`).
const ADJUSTED_HELP: &str = "Accept bed screw position after notable adjustment";
/// `cmd_ABORT_help` (`bed_screws.py:116`).
const ABORT_HELP: &str = "Abort bed screws tool";

/// Which screw list a session walks — upstream's `self.state`, `'adjust'` or
/// `'fine'` (`bed_screws.py:40-41`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    Adjust,
    Fine,
}

impl State {
    /// The name `get_status` reports (`bed_screws.py:76`).
    const fn name(self) -> &'static str {
        match self {
            State::Adjust => "adjust",
            State::Fine => "fine",
        }
    }
}

/// The per-session state (`bed_screws.py:40-43`).
#[derive(Clone, Copy, Debug, Default)]
struct Session {
    /// `state`: `None` while no session runs.
    state: Option<State>,
    /// `current_screw`.
    current_screw: usize,
    /// `accepted_screws`; `cmd_ADJUSTED` drops it to `-1` before `cmd_ACCEPT`
    /// increments it (`bed_screws.py:112`), so it is signed.
    accepted_screws: i64,
}

/// The parsed `[bed_screws]` options (`bed_screws.py:13-34`).
#[derive(Debug)]
struct Settings {
    /// `(coordinate, name)` per `screwN`, in order (`bed_screws.py:15-22`).
    screws: Vec<((f64, f64), String)>,
    /// `(coordinate, name)` per `screwN_fine_adjust`, in order; the name is
    /// the coarse screw's own (`bed_screws.py:23-26`).
    fine_adjust: Vec<((f64, f64), String)>,
    /// The `speed` option, default `50.`, above `0` (`bed_screws.py:31`).
    speed: f64,
    /// The `probe_speed` option — upstream's `lift_speed`, default `5.`,
    /// above `0` (`bed_screws.py:32`).
    probe_speed: f64,
    /// The `horizontal_move_z` option, default `5.` (`bed_screws.py:33`).
    horizontal_move_z: f64,
    /// The `probe_height` option — upstream's `probe_z`, default `0.`
    /// (`bed_screws.py:34`).
    probe_height: f64,
}

impl Settings {
    /// Read the section in upstream's order (`bed_screws.py:13-34`): the
    /// screws (each with its name and fine coordinate), the three-screws
    /// floor, then the travels.
    ///
    /// # Errors
    /// A coordinate that is not two numbers (upstream's `getfloatlist` with
    /// `count=2`), fewer than three screws, or a travel outside its bound —
    /// all with upstream's wording.
    fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let mut screws = Vec::new();
        let mut fine_adjust = Vec::new();
        for index in 1..=99 {
            let prefix = format!("screw{index}");
            // `config.get(prefix, None) is None` breaks the scan
            // (`bed_screws.py:17-18`): a gap stops it even when a higher
            // screw exists.
            let Some(raw) = config.get_str(&prefix) else {
                break;
            };
            let coord = parse_coord(&prefix, &identifier, &raw)?;
            let default_name = format!("screw at {:.3},{:.3}", coord.0, coord.1);
            let name = config.get(&format!("{prefix}_name"), Some(&default_name))?;
            let fine_option = format!("{prefix}_fine_adjust");
            if let Some(fine_raw) = config.get_str(&fine_option) {
                let fine = parse_coord(&fine_option, &identifier, &fine_raw)?;
                fine_adjust.push((fine, name.clone()));
            }
            screws.push((coord, name));
        }
        if screws.len() < 3 {
            return Err(ConfigError::new(
                "bed_screws: Must have at least three screws".to_string(),
            ));
        }
        Ok(Self {
            screws,
            fine_adjust,
            speed: config.get_float_bounded("speed", Some(50.), None, None, Some(0.), None)?,
            probe_speed: config.get_float_bounded(
                "probe_speed",
                Some(5.),
                None,
                None,
                Some(0.),
                None,
            )?,
            horizontal_move_z: config.get_float("horizontal_move_z", Some(5.))?,
            probe_height: config.get_float("probe_height", Some(0.))?,
        })
    }
}

/// One coordinate option: two comma-separated floats — upstream's
/// `getfloatlist(option, count=2)` (`bed_screws.py:19/25`), whose parse runs
/// before the count check (`configfile.py:88-102`).
///
/// # Errors
/// `Unable to parse option …` for a value that is not a number, `must have 2
/// elements` for any other length.
fn parse_coord(option: &str, identifier: &str, raw: &str) -> Result<(f64, f64), ConfigError> {
    let parts: Vec<&str> = raw.split(',').map(str::trim).collect();
    let parsed: Vec<f64> = parts
        .iter()
        .map(|part| {
            part.parse::<f64>().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option '{option}' in section '{identifier}'"
                ))
            })
        })
        .collect::<Result<_, _>>()?;
    if parsed.len() != 2 {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{identifier}' must have 2 elements"
        )));
    }
    Ok((parsed[0], parsed[1]))
}

/// What the helper needs from the machine: the toolhead move behind
/// `toolhead.manual_move` and the G-Code dispatch its commands register with
/// (the [`Adjust`](crate::core::klippy::extras::z_tilt) pattern, so the walk
/// runs against a fake in tests).
trait ScrewOps: Send + Sync {
    /// `toolhead.get_position()`.
    ///
    /// # Errors
    /// "Printer is not ready" before connect.
    fn position(&self) -> Result<Coord, CommandError>;
    /// `toolhead.manual_move(coord, speed)`'s move half.
    ///
    /// # Errors
    /// Whatever the planner refuses: an unhomed axis, a move out of range.
    fn manual_move(&self, position: Coord, speed: f64) -> Result<(), CommandError>;
    /// `gcode.respond_info`.
    fn respond_info(&self, message: &str);
    /// `gcode.register_command(name, handler, desc=...)`.
    ///
    /// # Errors
    /// Upstream's "gcode command \<name\> already registered" when a name is
    /// taken — the `ACCEPT`/`ABORT` overlap with `manual_probe`.
    fn register_command(
        &self,
        name: &str,
        handler: CommandHandler,
        desc: &str,
    ) -> Result<(), CommandError>;
    /// `gcode.register_command(name, None)`.
    fn unregister_command(&self, name: &str);
}

/// The [`ScrewOps`] wiring against the real machine (`bed_screws.py:44-51`).
///
/// The toolhead and the dispatch are looked up per call: the section loads
/// before `toolhead` does, so neither can be held at load.
struct LiveScrewOps {
    printer: Weak<Printer>,
}

impl LiveScrewOps {
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    fn gcode(&self) -> Option<Arc<GCodeDispatch>> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT))
    }
}

impl ScrewOps for LiveScrewOps {
    fn position(&self) -> Result<Coord, CommandError> {
        self.toolhead()?
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    fn manual_move(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.toolhead()?.move_to(position, speed)?;
        // Upstream's `manual_move` fires this (`toolhead.py:416`); `gcode_move`
        // re-anchors its `last_position` on it.
        if let Some(printer) = self.printer.upgrade() {
            printer.send_event(&KlippyEvent::ToolheadManualMove);
        }
        Ok(())
    }

    fn respond_info(&self, message: &str) {
        if let Some(gcode) = self.gcode() {
            gcode.respond_info(message, true);
        }
    }

    fn register_command(
        &self,
        name: &str,
        handler: CommandHandler,
        desc: &str,
    ) -> Result<(), CommandError> {
        let gcode = self
            .gcode()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        gcode
            .register_command(name, handler, Some(desc), false)
            .map_err(CommandError::new)
    }

    fn unregister_command(&self, name: &str) {
        if let Some(gcode) = self.gcode() {
            gcode.unregister_command(name);
        }
    }
}

/// Upstream's `ToolHead.manual_move` (`toolhead.py:410-416`): the non-`None`
/// entries of `coord` replace the commanded position's, then the move is
/// queued.
///
/// # Errors
/// As [`ScrewOps::position`] and [`ScrewOps::manual_move`].
fn manual_move(
    ops: &dyn ScrewOps,
    coord: [Option<f64>; 3],
    speed: f64,
) -> Result<(), CommandError> {
    let mut position = ops.position()?;
    for (axis, value) in coord.iter().enumerate() {
        if let Some(value) = value {
            position.set_axis(axis, *value);
        }
    }
    ops.manual_move(position, speed)
}

/// The owning object for a session handler.
///
/// # Errors
/// "Printer is not ready" when the object has been dropped while its handler
/// outlived it.
fn object(weak: &Weak<BedScrews>) -> Result<Arc<BedScrews>, CommandError> {
    weak.upgrade()
        .ok_or_else(|| CommandError::new("Printer is not ready"))
}

/// A session command handler: upgrade the object, then run `dispatch`
/// (`bed_screws.py:64-73`).
fn command_handler<F>(weak: Weak<BedScrews>, dispatch: F) -> CommandHandler
where
    F: Fn(Arc<BedScrews>, &GcodeCommand) -> Result<(), CommandError> + Send + Sync + 'static,
{
    let dispatch = Arc::new(dispatch);
    Arc::new(move |gcmd| {
        let weak = weak.clone();
        let dispatch = Arc::clone(&dispatch);
        Box::pin(async move {
            let this = object(&weak)?;
            dispatch(this, gcmd)
        })
    })
}

/// The `[bed_screws]` section and the `BED_SCREWS_ADJUST` session it runs
/// (`bed_screws.py:BedScrews`).
pub struct BedScrews {
    /// The machine the session moves and registers with.
    ops: Arc<dyn ScrewOps>,
    /// The parsed options.
    settings: Settings,
    /// The running session's state, or the rest state.
    session: Mutex<Session>,
}

impl BedScrews {
    /// Read the section and register `BED_SCREWS_ADJUST` (`bed_screws.py:8-39`).
    ///
    /// # Errors
    /// A malformed option, fewer than three screws, or a taken command name.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let ops: Arc<dyn ScrewOps> = Arc::new(LiveScrewOps {
            printer: Arc::downgrade(printer),
        });
        Self::with_ops(config, ops)
    }

    /// Build the object against a given [`ScrewOps`] (`new` supplies the live
    /// one; tests a fake).
    ///
    /// # Errors
    /// As [`BedScrews::new`].
    fn with_ops(config: &ConfigWrapper, ops: Arc<dyn ScrewOps>) -> Result<Arc<Self>, ConfigError> {
        let this = Arc::new(Self {
            ops,
            settings: Settings::read(config)?,
            session: Mutex::new(Session::default()),
        });
        this.register_adjust_command()?;
        Ok(this)
    }

    fn lock_session(&self) -> std::sync::MutexGuard<'_, Session> {
        self.session
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The list a state walks (`bed_screws.py`'s `self.states[state]`).
    fn state_screws(&self, state: State) -> &[((f64, f64), String)] {
        match state {
            State::Adjust => &self.settings.screws,
            State::Fine => &self.settings.fine_adjust,
        }
    }

    /// `self.number_of_screws` (`bed_screws.py:29`).
    fn number_of_screws(&self) -> i64 {
        self.settings.screws.len() as i64
    }

    /// `reset()` (`bed_screws.py:40-43`).
    fn reset(&self) {
        let mut session = self.lock_session();
        *session = Session::default();
    }

    /// Upstream's `move` (`bed_screws.py:44-51`): a manual move that drops
    /// the session — unregisters the commands and resets — when the toolhead
    /// refuses.
    ///
    /// # Errors
    /// As [`manual_move`], after the session has been dropped.
    fn move_toolhead(&self, coord: [Option<f64>; 3], speed: f64) -> Result<(), CommandError> {
        match manual_move(self.ops.as_ref(), coord, speed) {
            Ok(()) => Ok(()),
            Err(error) => {
                self.unregister_commands();
                self.reset();
                Err(error)
            }
        }
    }

    /// `unregister_commands` (`bed_screws.py:60-63`).
    fn unregister_commands(&self) {
        self.ops.unregister_command(ACCEPT);
        self.ops.unregister_command(ADJUSTED);
        self.ops.unregister_command(ABORT);
    }

    /// `move_to_screw` (`bed_screws.py:53-73`): lift, move over the screw,
    /// descend, then announce it and register the session commands.
    ///
    /// # Errors
    /// As [`BedScrews::move_toolhead`], or a taken session command name.
    fn move_to_screw(self: &Arc<Self>, state: State, screw: usize) -> Result<(), CommandError> {
        // Move up, over, and then down (`bed_screws.py:54-56`).
        self.move_toolhead(
            [None, None, Some(self.settings.horizontal_move_z)],
            self.settings.probe_speed,
        )?;
        let (coord, name) = {
            let screws = self.state_screws(state);
            (screws[screw].0, screws[screw].1.clone())
        };
        self.move_toolhead(
            [
                Some(coord.0),
                Some(coord.1),
                Some(self.settings.horizontal_move_z),
            ],
            self.settings.speed,
        )?;
        self.move_toolhead(
            [
                Some(coord.0),
                Some(coord.1),
                Some(self.settings.probe_height),
            ],
            self.settings.probe_speed,
        )?;
        // Update state (`bed_screws.py:57-59`).
        {
            let mut session = self.lock_session();
            session.state = Some(state);
            session.current_screw = screw;
        }
        self.ops.respond_info(&format!(
            "Adjust {name}. Then run ACCEPT, ADJUSTED, or ABORT\n\
             Use ADJUSTED if a significant screw adjustment is made"
        ));
        // Register commands (`bed_screws.py:64-73`). A name `manual_probe`
        // already holds refuses the session; take back whatever did register
        // rather than leave a partial trio, and drop the session state.
        if let Err(error) = self.register_session_commands() {
            self.reset();
            return Err(error);
        }
        Ok(())
    }

    /// Register `ACCEPT` / `ADJUSTED` / `ABORT` (`bed_screws.py:64-73`).
    ///
    /// # Errors
    /// "gcode command \<name\> already registered" when one is taken. The
    /// commands that did register are removed first, so a collision never
    /// leaves a partial trio (upstream leaves them behind).
    fn register_session_commands(self: &Arc<Self>) -> Result<(), CommandError> {
        let weak = Arc::downgrade(self);
        self.ops.register_command(
            ACCEPT,
            command_handler(weak.clone(), |this, gcmd| this.cmd_accept(gcmd)),
            ACCEPT_HELP,
        )?;
        if let Err(error) = self.ops.register_command(
            ADJUSTED,
            command_handler(weak.clone(), |this, gcmd| this.cmd_adjusted(gcmd)),
            ADJUSTED_HELP,
        ) {
            self.ops.unregister_command(ACCEPT);
            return Err(error);
        }
        if let Err(error) = self.ops.register_command(
            ABORT,
            command_handler(weak, |this, gcmd| this.cmd_abort(gcmd)),
            ABORT_HELP,
        ) {
            self.ops.unregister_command(ACCEPT);
            self.ops.unregister_command(ADJUSTED);
            return Err(error);
        }
        Ok(())
    }

    /// Register `BED_SCREWS_ADJUST` (`bed_screws.py:37-39`).
    ///
    /// # Errors
    /// "gcode command BED_SCREWS_ADJUST already registered" when the name is
    /// taken.
    fn register_adjust_command(self: &Arc<Self>) -> Result<(), ConfigError> {
        let handler = command_handler(Arc::downgrade(self), |this, gcmd| {
            this.cmd_bed_screws_adjust(gcmd)
        });
        self.ops
            .register_command(BED_SCREWS_ADJUST, handler, BED_SCREWS_ADJUST_HELP)
            .map_err(|error| ConfigError::new(error.to_string()))
    }

    /// `cmd_BED_SCREWS_ADJUST` (`bed_screws.py:81-86`): start a session on the
    /// first coarse screw.
    ///
    /// # Errors
    /// "Already in bed_screws helper; use ABORT to exit" while one is running,
    /// or a toolhead refusal from the first moves.
    fn cmd_bed_screws_adjust(self: &Arc<Self>, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        if self.lock_session().state.is_some() {
            return Err(CommandError::new(
                "Already in bed_screws helper; use ABORT to exit",
            ));
        }
        // Reset the accepted count (`bed_screws.py:83-85`).
        {
            let mut session = self.lock_session();
            session.accepted_screws = 0;
        }
        self.move_toolhead(
            [None, None, Some(self.settings.horizontal_move_z)],
            self.settings.speed,
        )?;
        self.move_to_screw(State::Adjust, 0)
    }

    /// `cmd_ACCEPT` (`bed_screws.py:84-105`): record the screw, then advance
    /// to the next one, retry the coarse pass, start the fine pass, or finish.
    ///
    /// # Errors
    /// A toolhead refusal from the next move (the session is dropped first), or
    /// "[not in a] bed_screws helper" when no session is running.
    fn cmd_accept(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.unregister_commands();
        let (state, current_screw, accepted_screws) = {
            let mut session = self.lock_session();
            session.accepted_screws += 1;
            (
                session.state,
                session.current_screw,
                session.accepted_screws,
            )
        };
        // `ACCEPT` only exists during a session; this is unreachable through
        // the command table.
        let Some(state) = state else {
            return Err(CommandError::new("Not in bed_screws helper"));
        };
        let number_of_screws = self.number_of_screws();
        if current_screw + 1 < self.state_screws(state).len() && accepted_screws < number_of_screws
        {
            // Continue with the next screw (`bed_screws.py:86-90`).
            return self.move_to_screw(state, current_screw + 1);
        }
        if accepted_screws < number_of_screws {
            // Retry the coarse adjustments (`bed_screws.py:91-94`).
            return self.move_to_screw(State::Adjust, 0);
        }
        if state == State::Adjust && !self.settings.fine_adjust.is_empty() {
            // Reset the accepted count and run the fine pass (`:95-100`).
            {
                let mut session = self.lock_session();
                session.accepted_screws = 0;
            }
            return self.move_to_screw(State::Fine, 0);
        }
        // Done (`bed_screws.py:101-105`).
        self.reset();
        self.move_toolhead(
            [None, None, Some(self.settings.horizontal_move_z)],
            self.settings.probe_speed,
        )?;
        gcmd.respond_info("Bed screws tool completed successfully");
        Ok(())
    }

    /// `cmd_ADJUSTED` (`bed_screws.py:111-115`): drop the accepted count to
    /// `-1` so `cmd_ACCEPT`'s increment restarts it, then accept.
    ///
    /// # Errors
    /// As [`BedScrews::cmd_accept`].
    fn cmd_adjusted(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.unregister_commands();
        {
            let mut session = self.lock_session();
            session.accepted_screws = -1;
        }
        self.cmd_accept(gcmd)
    }

    /// `cmd_ABORT` (`bed_screws.py:117-120`): end the session.
    ///
    /// # Errors
    /// None — a move is not involved.
    fn cmd_abort(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.unregister_commands();
        self.reset();
        Ok(())
    }
}

impl PrinterObject for BedScrews {
    /// The status (`bed_screws.py:74-79`): inactive with an empty session
    /// until `BED_SCREWS_ADJUST` runs.
    fn get_status(&self, _eventtime: f64) -> Value {
        let session = self.lock_session();
        json!({
            "is_active": session.state.is_some(),
            "state": session.state.map(State::name),
            "current_screw": session.current_screw,
            "accepted_screws": session.accepted_screws,
        })
    }
}

/// The factory `section!` names (`bed_screws.py:122 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(BedScrews::new(config, printer)?)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicBool, Ordering};

    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::gcode::OutputHandler;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[bed_screws]` section with the given options, as the parser would
    /// build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("bed_screws", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// Three screws, each with a fine coordinate, named `s1`..`s3`.
    fn three_screws() -> ConfigSection {
        section(&[
            ("screw1", "10,30"),
            ("screw1_name", "s1"),
            ("screw1_fine_adjust", "12,32"),
            ("screw2", "155,30"),
            ("screw2_name", "s2"),
            ("screw2_fine_adjust", "157,32"),
            ("screw3", "155,190"),
            ("screw3_name", "s3"),
            ("screw3_fine_adjust", "157,192"),
        ])
    }

    /// A printer with `gcode` registered and an output sink that captures
    /// every line the dispatcher emits.
    fn printer_with_gcode() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<Mutex<Vec<String>>>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let gcode = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
        printer
            .add_object(GCODE_OBJECT, gcode.clone())
            .expect("gcode registers");
        let log = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        let handler: Arc<dyn OutputHandler> = Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        });
        gcode.register_output_handler(handler);
        (printer, gcode, log)
    }

    /// A [`ScrewOps`] over a real G-Code dispatch with a fake toolhead: moves
    /// are recorded (and can be made to fail), while the command table is the
    /// real one.
    struct FakeScrews {
        gcode: Arc<GCodeDispatch>,
        position: Mutex<Coord>,
        moves: Mutex<Vec<Coord>>,
        fail_move: AtomicBool,
    }

    impl FakeScrews {
        fn new(gcode: Arc<GCodeDispatch>) -> Self {
            Self {
                gcode,
                position: Mutex::new(Coord::new(0., 0., 0., 0.)),
                moves: Mutex::new(Vec::new()),
                fail_move: AtomicBool::new(false),
            }
        }

        fn moves(&self) -> Vec<Coord> {
            self.moves.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    impl ScrewOps for FakeScrews {
        fn position(&self) -> Result<Coord, CommandError> {
            Ok(*self.position.lock().unwrap_or_else(|p| p.into_inner()))
        }

        fn manual_move(&self, position: Coord, _speed: f64) -> Result<(), CommandError> {
            if self.fail_move.load(Ordering::SeqCst) {
                return Err(CommandError::new("move refused"));
            }
            self.moves
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(position);
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            Ok(())
        }

        fn respond_info(&self, message: &str) {
            self.gcode.respond_info(message, true);
        }

        fn register_command(
            &self,
            name: &str,
            handler: CommandHandler,
            desc: &str,
        ) -> Result<(), CommandError> {
            self.gcode
                .register_command(name, handler, Some(desc), false)
                .map_err(CommandError::new)
        }

        fn unregister_command(&self, name: &str) {
            self.gcode.unregister_command(name);
        }
    }

    /// A g-code command with no parameters, for the handlers under test.
    fn gcmd(gcode: &GCodeDispatch, command: &str) -> GcodeCommand {
        gcode.create_gcode_command(command, command, HashMap::new())
    }

    // --- Config parsing ---------------------------------------------------

    /// Every option the corpus writes is read back through the tracker, so
    /// `check_unused` accepts the section — the option-level half of the
    /// loader's validation (`config/validate.rs:48`).
    #[test]
    fn every_option_the_section_writes_is_recorded_as_read() {
        let text = "\
[bed_screws]
screw1: 100,50
screw1_name: Front right
screw1_fine_adjust: 200,50
screw2: 75,75
screw2_fine_adjust: 200,75
screw3: 75,75
screw3_name: Last
screw3_fine_adjust: 75,90
speed: 20.
probe_speed: 2.
horizontal_move_z: 2.
probe_height: 0.5
";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("bed_screws").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = Settings::read(&wrapper).expect("the section reads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(
            parsed.screws,
            [
                ((100., 50.), "Front right".to_string()),
                ((75., 75.), "screw at 75.000,75.000".to_string()),
                ((75., 75.), "Last".to_string()),
            ]
        );
        // The fine pass pairs each coordinate with the coarse screw's name.
        assert_eq!(
            parsed.fine_adjust,
            [
                ((200., 50.), "Front right".to_string()),
                ((200., 75.), "screw at 75.000,75.000".to_string()),
                ((75., 90.), "Last".to_string()),
            ]
        );
        assert_eq!(parsed.speed, 20.);
        assert_eq!(parsed.probe_speed, 2.);
        assert_eq!(parsed.horizontal_move_z, 2.);
        assert_eq!(parsed.probe_height, 0.5);
    }

    /// With nothing but the screws, the four travels answer upstream's
    /// defaults and the names fall back to `screw at %.3f,%.3f`
    /// (`bed_screws.py:20,31-34`).
    #[test]
    fn the_travels_fall_back_to_their_upstream_defaults() {
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let parsed = Settings::read(&config).expect("three screws read");

        assert_eq!(parsed.speed, 50.);
        assert_eq!(parsed.probe_speed, 5.);
        assert_eq!(parsed.horizontal_move_z, 5.);
        assert_eq!(parsed.probe_height, 0.);
        assert_eq!(parsed.screws[0].1, "screw at 10.000,30.000");
        assert!(parsed.fine_adjust.is_empty());
    }

    /// `screwN` is read from 1 up to the first missing index: a gap stops
    /// the scan even when a higher screw exists, and a `screwN_fine_adjust`
    /// behind the gap is never read (`bed_screws.py:15-18`).
    #[test]
    fn screws_stop_at_the_first_missing_index() {
        let access = AccessTracking::shared();
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
            ("screw5", "10,190"),
            ("screw5_fine_adjust", "12,192"),
        ]);
        let config = ConfigWrapper::new(&sect, Arc::clone(&access));
        let parsed = Settings::read(&config).expect("three screws read");

        assert_eq!(
            parsed.screws.len(),
            3,
            "screw5 must not be read past the gap"
        );
        let settings = access.settings();
        let options = &settings["bed_screws"];
        assert!(options.get("screw4").is_none());
        assert!(options.get("screw5").is_none());
        assert!(options.get("screw5_fine_adjust").is_none());
    }

    /// Fewer than three screws is config error with upstream's wording, a
    /// coordinate of any other length keeps the `getfloatlist` count wording,
    /// and a non-number keeps the parser's wording (`bed_screws.py:27-28`,
    /// `configfile.py:49/100-102`).
    #[test]
    fn malformed_screws_keep_upstream_wording() {
        let sect = section(&[("screw1", "10,30"), ("screw2", "155,30")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            Settings::read(&config).unwrap_err().to_string(),
            "bed_screws: Must have at least three screws"
        );

        let sect = section(&[
            ("screw1", "10"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            Settings::read(&config).unwrap_err().to_string(),
            "Option 'screw1' in section 'bed_screws' must have 2 elements"
        );

        let sect = section(&[
            ("screw1", "left,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            Settings::read(&config).unwrap_err().to_string(),
            "Unable to parse option 'screw1' in section 'bed_screws'"
        );

        // The fine coordinate is parsed the same way.
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
            ("screw2_fine_adjust", "200"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            Settings::read(&config).unwrap_err().to_string(),
            "Option 'screw2_fine_adjust' in section 'bed_screws' must have 2 elements"
        );
    }

    /// `speed`/`probe_speed` must be above `0` with upstream's bound wording,
    /// and an unparseable travel keeps the parser's wording
    /// (`bed_screws.py:31-32`, `configfile.py:49`).
    #[test]
    fn the_travels_are_bounded_the_way_upstream_bounds_them() {
        // The three-screws floor is checked first (`bed_screws.py:27-31`),
        // so every case keeps a full screw list and varies a travel.
        let screws: [(&str, &str); 3] = [
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ];
        let with = |travel: (&'static str, &'static str)| {
            section(&[screws[0], screws[1], screws[2], travel])
        };

        let sect = with(("speed", "0"));
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            Settings::read(&config).unwrap_err().to_string(),
            "Option 'speed' in section 'bed_screws' must be above 0"
        );

        let sect = with(("probe_speed", "-1"));
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            Settings::read(&config).unwrap_err().to_string(),
            "Option 'probe_speed' in section 'bed_screws' must be above 0"
        );

        let sect = with(("horizontal_move_z", "above"));
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            Settings::read(&config).unwrap_err().to_string(),
            "Unable to parse option 'horizontal_move_z' in section 'bed_screws'"
        );
    }

    // --- Commands ---------------------------------------------------------

    /// The object is built over a fresh printer and fake toolhead; the printer
    /// is returned so the command table stays alive with it.
    fn bed_and_fake(sect: &ConfigSection) -> (Arc<Printer>, Arc<BedScrews>, Arc<FakeScrews>) {
        let (printer, gcode, _log) = printer_with_gcode();
        let fake = Arc::new(FakeScrews::new(gcode));
        let ops: Arc<dyn ScrewOps> = fake.clone();
        let config = ConfigWrapper::untracked(sect);
        let bed = BedScrews::with_ops(&config, ops).expect("the section builds");
        (printer, bed, fake)
    }

    /// `BED_SCREWS_ADJUST` is registered at load, and the session trio is not
    /// (`bed_screws.py:37-39/64-73`).
    #[test]
    fn bed_screws_adjust_is_registered_and_the_session_trio_is_not() {
        let sect = three_screws();
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();

        assert!(gcode.command_exists(BED_SCREWS_ADJUST));
        assert!(!gcode.command_exists(ACCEPT));
        assert!(!gcode.command_exists(ADJUSTED));
        assert!(!gcode.command_exists(ABORT));
        // The rest state upstream's `reset()` leaves (`bed_screws.py:40-43`).
        assert_eq!(
            bed.get_status(0.),
            json!({
                "is_active": false,
                "state": null,
                "current_screw": 0,
                "accepted_screws": 0,
            })
        );
    }

    /// Starting a session registers `ACCEPT` / `ADJUSTED` / `ABORT` and moves
    /// to the first coarse screw (`bed_screws.py:53-73/81-86`); `ABORT`
    /// unregisters them again and resets (`:117-120`).
    #[test]
    fn a_session_registers_the_trio_and_abort_removes_it() {
        let sect = three_screws();
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();

        bed.cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .expect("the session starts");
        assert!(gcode.command_exists(ACCEPT));
        assert!(gcode.command_exists(ADJUSTED));
        assert!(gcode.command_exists(ABORT));
        assert_eq!(bed.get_status(0.)["state"], json!("adjust"));
        assert_eq!(bed.get_status(0.)["current_screw"], json!(0));

        // Lift twice — `cmd_BED_SCREWS_ADJUST` (`bed_screws.py:85`) and then
        // `move_to_screw` (`:54`) — then over screw 1 and down.
        assert_eq!(
            fake.moves(),
            [
                Coord::new(0., 0., 5., 0.),
                Coord::new(0., 0., 5., 0.),
                Coord::new(10., 30., 5., 0.),
                Coord::new(10., 30., 0., 0.),
            ]
        );

        bed.cmd_abort(&gcmd(&gcode, ABORT)).expect("abort ends it");
        assert!(!gcode.command_exists(ACCEPT));
        assert!(!gcode.command_exists(ADJUSTED));
        assert!(!gcode.command_exists(ABORT));
        assert_eq!(bed.get_status(0.)["is_active"], json!(false));
    }

    /// A second `BED_SCREWS_ADJUST` while one runs is refused
    /// (`bed_screws.py:82`).
    #[test]
    fn a_second_session_is_refused_while_one_is_active() {
        let sect = three_screws();
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();

        bed.cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .expect("the session starts");
        let error = bed
            .cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .unwrap_err();
        assert_eq!(
            error.to_string(),
            "Already in bed_screws helper; use ABORT to exit"
        );
    }

    /// `ACCEPT` walks the coarse screws; once all are accepted the fine pass
    /// runs, and its completion resets, lifts and reports
    /// (`bed_screws.py:84-105`).
    #[test]
    fn accept_walks_the_coarse_screws_then_the_fine_pass() {
        let sect = three_screws();
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();
        let log = Arc::new(Mutex::new(Vec::new()));
        {
            let sink = Arc::clone(&log);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                sink.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string());
            }));
        }

        bed.cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .expect("the session starts");
        // Two accepts still walk the coarse list.
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("accept 1");
        assert_eq!(bed.get_status(0.)["current_screw"], json!(1));
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("accept 2");
        assert_eq!(bed.get_status(0.)["current_screw"], json!(2));
        // The third accept completes the coarse pass and enters the fine one.
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("accept 3");
        assert_eq!(bed.get_status(0.)["state"], json!("fine"));
        assert_eq!(bed.get_status(0.)["current_screw"], json!(0));
        assert_eq!(bed.get_status(0.)["accepted_screws"], json!(0));

        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("fine 1");
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("fine 2");
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("fine 3");
        // Done: the session is gone and the trio is unregistered.
        assert_eq!(bed.get_status(0.)["is_active"], json!(false));
        assert!(!gcode.command_exists(ACCEPT));
        assert!(!gcode.command_exists(ADJUSTED));
        assert!(!gcode.command_exists(ABORT));

        let lines = log.lock().unwrap_or_else(|p| p.into_inner()).clone();
        assert!(
            lines.iter().any(|line| line
                == "// Adjust s3. Then run ACCEPT, ADJUSTED, or ABORT\n\
                    // Use ADJUSTED if a significant screw adjustment is made"),
            "the adjust prompt is reported: {lines:?}"
        );
        assert!(
            lines
                .iter()
                .any(|line| line == "// Bed screws tool completed successfully"),
            "the completion is reported: {lines:?}"
        );
    }

    /// Without a `screwN_fine_adjust` pass, the coarse pass already completes
    /// the session (`bed_screws.py:95-105`).
    #[test]
    fn a_section_without_fine_adjust_finishes_after_the_coarse_pass() {
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();

        bed.cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .expect("the session starts");
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("accept 1");
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("accept 2");
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("accept 3");

        assert_eq!(bed.get_status(0.)["is_active"], json!(false));
        assert!(!gcode.command_exists(ACCEPT));
    }

    /// `ADJUSTED` drops the accepted count to `-1` before accepting, so the
    /// count restarts for the next screw (`bed_screws.py:111-115`).
    #[test]
    fn adjusted_restarts_the_accept_count_for_the_next_screw() {
        let sect = three_screws();
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();

        bed.cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .expect("the session starts");
        bed.cmd_accept(&gcmd(&gcode, ACCEPT)).expect("accept");
        assert_eq!(bed.get_status(0.)["accepted_screws"], json!(1));
        assert_eq!(bed.get_status(0.)["current_screw"], json!(1));

        bed.cmd_adjusted(&gcmd(&gcode, ADJUSTED)).expect("adjusted");
        // -1 + 1 = 0, then the walk advanced to the next screw.
        assert_eq!(bed.get_status(0.)["accepted_screws"], json!(0));
        assert_eq!(bed.get_status(0.)["current_screw"], json!(2));
    }

    /// A toolhead refusal mid-session drops the session — the trio is
    /// unregistered and the state reset — and re-raises
    /// (`bed_screws.py:44-51`).
    #[test]
    fn a_failed_move_unregisters_the_trio_and_resets_the_session() {
        let sect = three_screws();
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();

        bed.cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .expect("the session starts");
        assert!(gcode.command_exists(ACCEPT));

        fake.fail_move.store(true, Ordering::SeqCst);
        let error = bed.cmd_accept(&gcmd(&gcode, ACCEPT)).unwrap_err();
        assert_eq!(error.to_string(), "move refused");
        assert!(!gcode.command_exists(ACCEPT));
        assert!(!gcode.command_exists(ADJUSTED));
        assert!(!gcode.command_exists(ABORT));
        assert_eq!(bed.get_status(0.)["is_active"], json!(false));
    }

    /// When `ACCEPT` is already taken — `manual_probe`'s own session
    /// (`manual_probe.py:139-144`) — the bed-screws session is refused and
    /// takes back the commands it registered, leaving neither a partial trio
    /// nor a dangling active state; the pre-existing `ACCEPT` is untouched.
    #[test]
    fn a_taken_accept_name_refuses_the_session_without_a_half_registration() {
        let sect = three_screws();
        let (_printer, bed, fake) = bed_and_fake(&sect);
        let gcode = fake.gcode.clone();
        // The other feature's `ACCEPT`, registered first.
        let dummy: CommandHandler = Arc::new(|_gcmd| Box::pin(async { Ok(()) }));
        gcode
            .register_command(ACCEPT, dummy, None, false)
            .expect("the other ACCEPT registers");

        let error = bed
            .cmd_bed_screws_adjust(&gcmd(&gcode, BED_SCREWS_ADJUST))
            .unwrap_err();
        assert_eq!(error.to_string(), "gcode command ACCEPT already registered");
        // The other feature keeps its command; the session's own two are gone.
        assert!(gcode.command_exists(ACCEPT));
        assert!(!gcode.command_exists(ADJUSTED));
        assert!(!gcode.command_exists(ABORT));
        assert_eq!(bed.get_status(0.)["is_active"], json!(false));
    }
}
