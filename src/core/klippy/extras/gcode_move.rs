//! `gcode_move` — the G-Code coordinate system: offsets, modes and factors.
//!
//! Upstream's `klippy/extras/gcode_move.py`. It is **not** a config section:
//! `toolhead.py:610-613` loads it by name as one of the toolhead's default
//! modules, and so does [`ensure`] (called from `toolhead`'s factory, the same
//! place upstream loads it).
//!
//! Everything a slicer means by "where" lives here: whether a word is absolute
//! or relative (`G90`/`G91` for XYZ, `M82`/`M83` for E — *separately*), which
//! anchor `G92` and `SET_GCODE_OFFSET` moved to, and the speed and extrude
//! factors `M220`/`M221` scale by. `G0`/`G1` turn all that into a **toolhead**
//! coordinate and hand it to [`MoveTarget::move_to`] — upstream's
//! `move_with_transform` — so the toolhead never sees a g-code coordinate.
//!
//! | command | meaning |
//! |---|---|
//! | `G0` / `G1` | move: each axis word in the current mode, `F` in mm/min |
//! | `G20` / `G21` | inches (refused) / millimeters (the only unit) |
//! | `G90` / `G91` | XYZ absolute / relative |
//! | `M82` / `M83` | E absolute / relative |
//! | `G92` | re-anchor: set `base_position`, so the g-code position reads what you name |
//! | `M220` / `M221` | speed / extrude factor, percent |
//! | `SET_GCODE_OFFSET` | a virtual offset, with `MOVE=` to take it now |
//! | `SAVE_GCODE_STATE` / `RESTORE_GCODE_STATE` | park the whole state under a name |
//! | `M114` | report the g-code position |
//!
//! # Keeping `last_position` honest
//!
//! `last_position` is in **toolhead** coordinates and `G1` only touches the
//! axes it names — an unnamed axis is passed through as it is. So whenever the
//! toolhead moves outside a `G1` (homing, `SET_KINEMATIC_POSITION`, a manual
//! move, a command that failed part way) the state has to be re-anchored to
//! where the toolhead actually is, or the next `G1` drags a stale axis along.
//! Upstream does that with events (`gcode_move.py:44-56`); this port handles
//!
//! | event | sent here by |
//! |---|---|
//! | `klippy:ready` | the printer — also when the move target is resolved |
//! | `homing:home_rails_end` (with `axes`) | `toolhead`'s homing loop |
//! | `toolhead:set_position` | `SET_KINEMATIC_POSITION` |
//! | `gcode:command_error` | the dispatcher |
//! | `toolhead:manual_move` | `safe_z_home`'s `HomeOps::manual_move` (`safe_z_home.rs:215`) |
//! | `toolhead:update_extra_axes` | `ToolHeadObject::add_extra_axis` / `remove_extra_axis`
//!   (`toolhead.rs:1876` / `:1892`, called by `manual_stepper`) |
//! | `extruder:activate_extruder` | registered, no sender yet (`ACTIVATE_EXTRUDER` does not fire it) |
//!
//! # What is not here
//!
//! * `GET_POSITION` needs the kinematics' stepper positions and
//!   `calc_position` (G4-2); it stays an unknown command until then, which
//!   costs a run nothing — klipper answers unknown commands rather than
//!   failing them.
//! * `axis_map` grows for extra axes upstream (`_update_extra_axes`); a
//!   [`Coord`] is four axes here, so the mapping stops at `E` (G4-2).
//! * `set_move_transform` is written and both `bed_tilt` (`bed_tilt.rs:376`) and
//!   `exclude_object` (`exclude_object.rs:212`) already swap the target through
//!   it; what is missing is upstream's `bed_mesh` itself.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::mathutil::{Coord, AXES, E_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The object's name, as upstream registers it.
pub const GCODE_MOVE_OBJECT: &str = "gcode_move";

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The speed a move runs at before any `F` (upstream's `self.speed = 25.`).
const DEFAULT_SPEED: f64 = 25.0;

/// What `gcode_move` drives: queue a move, read back where the toolhead is.
///
/// Upstream resolves `move_with_transform` / `position_with_transform` to
/// `toolhead.move` / `toolhead.get_position` at ready, and `bed_mesh` swaps
/// them for its own transform later. One trait keeps that swap possible and
/// lets the coordinate math be tested without a planner.
pub trait MoveTarget: Send + Sync {
    /// Queue a move to `position` at `speed` mm/s.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError>;

    /// The toolhead's current position.
    fn position(&self) -> Coord;
}

/// The toolhead, as [`MoveTarget`] (upstream's ready-time resolution).
struct ToolheadTarget(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadTarget {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn position(&self) -> Coord {
        self.0.position().unwrap_or_default()
    }
}

/// The g-code coordinate state (`gcode_move.py:21-37`).
#[derive(Debug)]
struct MoveState {
    /// `G90`/`G91`: are X/Y/Z words absolute?
    absolute_coord: bool,
    /// `M82`/`M83`: is an E word absolute? (Independent of the axes.)
    absolute_extrude: bool,
    /// The anchor `G92` / `SET_GCODE_OFFSET` moved: g-code = `last - base`.
    base_position: [f64; AXES],
    /// Where the **toolhead** is, per the last `G1` or re-anchor.
    last_position: Coord,
    /// What `SET_GCODE_OFFSET` remembers, re-applied to `base_position` after
    /// homing (`gcode_move.py:245-247`).
    homing_position: [f64; AXES],
    /// mm/s (`F * speed_factor`).
    speed: f64,
    /// `1/60` at start: `F` is mm/min and `speed` is mm/s; `M220` changes it.
    speed_factor: f64,
    /// `M221`, applied to E words and to `G92 E`.
    extrude_factor: f64,
    /// `SAVE_GCODE_STATE` / `RESTORE_GCODE_STATE`, by name.
    saved_states: HashMap<String, SavedState>,
}

impl Default for MoveState {
    fn default() -> Self {
        Self {
            absolute_coord: true,
            absolute_extrude: true,
            base_position: [0.0; AXES],
            last_position: Coord::default(),
            homing_position: [0.0; AXES],
            speed: DEFAULT_SPEED,
            speed_factor: 1.0 / 60.0,
            extrude_factor: 1.0,
            saved_states: HashMap::new(),
        }
    }
}

/// One parked g-code state (`SAVE_GCODE_STATE`).
#[derive(Debug, Clone)]
struct SavedState {
    absolute_coord: bool,
    absolute_extrude: bool,
    base_position: [f64; AXES],
    last_position: Coord,
    homing_position: [f64; AXES],
    speed: f64,
    speed_factor: f64,
    extrude_factor: f64,
}

/// One `gcode_move`, as upstream's `GCodeMove`.
pub struct GCodeMove {
    printer: Weak<Printer>,
    state: Mutex<MoveState>,
    /// Upstream's `move_with_transform` / `position_with_transform`, resolved
    /// at ready (and settable beforehand — `set_move_transform`).
    target: Mutex<Option<Arc<dyn MoveTarget>>>,
}

/// The single `gcode_move`; the first caller creates it (upstream's
/// `printer.load_object(config, "gcode_move")`).
///
/// # Errors
/// A duplicate registration or a g-code name this dispatcher refuses.
pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<GCodeMove>, ConfigError> {
    GCodeMove::ensure(printer)
}

impl GCodeMove {
    fn new(printer: Weak<Printer>) -> Self {
        Self {
            printer,
            state: Mutex::new(MoveState::default()),
            target: Mutex::new(None),
        }
    }

    /// The single `gcode_move`; the first caller creates it, as upstream's
    /// `printer.load_object(config, "gcode_move")` does.
    ///
    /// # Errors
    /// A duplicate registration or a g-code name this dispatcher refuses.
    pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<GCodeMove>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(Self::new(Arc::downgrade(printer)));
        printer.add_object(
            GCODE_MOVE_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        object.register_commands(printer)?;
        object.register_handlers(printer);
        Ok(object)
    }

    /// Upstream's `set_move_transform`: take the slot `bed_mesh` fills, so
    /// moves go through a transform instead of the toolhead.
    ///
    /// # Errors
    /// When something already took the slot and `force` is not set.
    pub fn set_move_transform(
        &self,
        target: Arc<dyn MoveTarget>,
        force: bool,
    ) -> Result<(), ConfigError> {
        let mut slot = self
            .target
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if slot.is_some() && !force {
            return Err(ConfigError::new("G-Code move transform already specified"));
        }
        *slot = Some(target);
        Ok(())
    }

    /// Upstream's `_handle_ready` (`gcode_move.py:58-64`): resolve the move
    /// target to the toolhead when nothing else claimed the slot, then anchor
    /// `last_position` to where the toolhead is.
    fn handle_ready(&self) {
        let claimed = self
            .target
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .is_some();
        if !claimed {
            let Some(printer) = self.printer.upgrade() else {
                return;
            };
            let toolhead = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT);
            if let Some(toolhead) = toolhead {
                *self.target.lock().unwrap_or_else(|p| p.into_inner()) =
                    Some(Arc::new(ToolheadTarget(toolhead)));
            }
        }
        // With no toolhead there is nothing to anchor to yet; upstream has the
        // same ordering (the object exists by ready, so this does not bite).
        self.reset_last_position();
    }

    /// Upstream's `reset_last_position`: `last_position` follows the toolhead,
    /// but only once there is one to ask.
    pub fn reset_last_position(&self) {
        let Some(target) = self.target() else {
            return;
        };
        self.lock().last_position = target.position();
    }

    fn target(&self) -> Option<Arc<dyn MoveTarget>> {
        self.target
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    fn lock(&self) -> MutexGuard<'_, MoveState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl GCodeMove {
    /// Register the commands, capturing a handle to this object.
    ///
    /// Each row also declares the parameter names its command reads, in the
    /// order the handler reads them. They are **not** what the handler parses;
    /// they reach a client as `status.gcode.commands[<name>]["parameters"]`,
    /// which is what completes the left of `=`. A command that reads no named
    /// parameter declares an empty list.
    ///
    /// The last field of each row is upstream's `when_not_ready`: `M114`
    /// answers before the printer is ready, the rest do not.
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        type Command = fn(&GCodeMove, &GcodeCommand) -> Result<(), CommandError>;
        /// The words `cmd_g1` reads, shared by both spellings of the move.
        const G1_PARAMS: &[&str] = &["X", "Y", "Z", "E", "F"];
        /// The axes `cmd_g92` anchors on.
        const G92_PARAMS: &[&str] = &["X", "Y", "Z", "E"];
        /// `cmd_m220` and `cmd_m221` each read their factor as a percentage, in
        /// `S`.
        const PERCENT_PARAMS: &[&str] = &["S"];
        /// `cmd_set_gcode_offset` reads every axis and then that same axis's
        /// `_ADJUST` form — the loop reads one axis's two words before it moves
        /// on, which is why each `*_ADJUST` follows its axis here.
        const GCODE_OFFSET_PARAMS: &[&str] = &[
            "X",
            "X_ADJUST",
            "Y",
            "Y_ADJUST",
            "Z",
            "Z_ADJUST",
            "E",
            "E_ADJUST",
            "MOVE",
            "MOVE_SPEED",
        ];
        const COMMANDS: &[(&str, Command, Option<&str>, &[&str], bool)] = &[
            ("G0", cmd_g1, None, G1_PARAMS, false),
            ("G1", cmd_g1, None, G1_PARAMS, false),
            ("G20", cmd_g20, None, &[], false),
            ("G21", cmd_g21, None, &[], false),
            ("G90", cmd_g90, None, &[], false),
            ("G91", cmd_g91, None, &[], false),
            ("G92", cmd_g92, None, G92_PARAMS, false),
            ("M82", cmd_m82, None, &[], false),
            ("M83", cmd_m83, None, &[], false),
            ("M114", cmd_m114, None, &[], true),
            ("M220", cmd_m220, None, PERCENT_PARAMS, false),
            ("M221", cmd_m221, None, PERCENT_PARAMS, false),
            (
                "SET_GCODE_OFFSET",
                cmd_set_gcode_offset,
                Some("Set a virtual offset to g-code positions"),
                GCODE_OFFSET_PARAMS,
                false,
            ),
            (
                "SAVE_GCODE_STATE",
                cmd_save_gcode_state,
                Some("Save G-Code coordinate state"),
                &["NAME"],
                false,
            ),
            (
                "RESTORE_GCODE_STATE",
                cmd_restore_gcode_state,
                Some("Restore a previously saved G-Code state"),
                &["NAME", "MOVE", "MOVE_SPEED"],
                false,
            ),
        ];
        for (name, command, desc, params, when_not_ready) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                sync(move |gcmd| command(&object, gcmd))
            };
            gcode
                .register_command_with_params(name, handler, *desc, params, *when_not_ready)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    /// The events upstream's `__init__` subscribes to (`gcode_move.py:44-56`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.handle_ready()
            }),
        );
        printer.register_event_handler(
            KlippyEvent::GcodeCommandError,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.reset_last_position()
            }),
        );
        printer.register_event_handler(
            KlippyEvent::ToolheadSetPosition,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.reset_last_position()
            }),
        );
        printer.register_event_handler(
            KlippyEvent::ToolheadManualMove,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.reset_last_position()
            }),
        );
        printer.register_event_handler(
            KlippyEvent::ExtruderActivateExtruder,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.handle_activate_extruder()
            }),
        );
        printer.register_event_handler(
            KlippyEvent::HomingHomeRailsEnd {
                axes: Vec::new(),
                homing: crate::core::klippy::motion::HomingHandle::new(),
            },
            Box::new({
                let object = Arc::clone(self);
                move |event| {
                    let axes = match event {
                        KlippyEvent::HomingHomeRailsEnd { axes, .. } => axes.as_slice(),
                        _ => &[],
                    };
                    object.handle_home_rails_end(axes);
                }
            }),
        );
    }

    /// Upstream's `_handle_home_rails_end`: re-anchor the axes that just homed
    /// to `homing_position`, on top of the reset every toolhead move causes.
    fn handle_home_rails_end(&self, axes: &[usize]) {
        self.reset_last_position();
        let mut state = self.lock();
        for &axis in axes {
            if let Some(homing) = state.homing_position.get(axis).copied() {
                state.base_position[axis] = homing;
            }
        }
    }

    /// Upstream's `_handle_activate_extruder`.
    fn handle_activate_extruder(&self) {
        self.reset_last_position();
        let mut state = self.lock();
        state.extrude_factor = 1.0;
        state.base_position[E_AXIS] = state.last_position.axis(E_AXIS);
    }

    /// Upstream's `GCodeMove.get_status`.
    pub fn status(&self) -> Value {
        let state = self.lock();
        json!({
            "speed_factor": state.speed_factor * 60.0,
            "speed": state.speed / state.speed_factor,
            "extrude_factor": state.extrude_factor,
            "absolute_coordinates": state.absolute_coord,
            "absolute_extrude": state.absolute_extrude,
            "homing_origin": state.homing_position.to_vec(),
            "position": state.last_position.axes().collect::<Vec<_>>(),
            "gcode_position": gcode_position(&state).axes().collect::<Vec<_>>(),
            // Upstream rebuilds this from the toolhead's extra axes; here it
            // stops at `E` (see the module docs).
            "axis_map": {"X": 0, "Y": 1, "Z": 2, "E": 3},
        })
    }
}

impl PrinterObject for GCodeMove {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.status()
    }
}

impl std::fmt::Debug for GCodeMove {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.lock();
        f.debug_struct("GCodeMove")
            .field("absolute_coord", &state.absolute_coord)
            .field("absolute_extrude", &state.absolute_extrude)
            .field("base_position", &state.base_position)
            .field("last_position", &state.last_position)
            .field("speed", &state.speed)
            .finish_non_exhaustive()
    }
}

/// The g-code position of a state (`gcode_move.py:_get_gcode_position`).
fn gcode_position(state: &MoveState) -> Coord {
    let mut position = state.last_position;
    for axis in 0..AXES {
        position.set_axis(axis, position.axis(axis) - state.base_position[axis]);
    }
    position.set_axis(E_AXIS, position.axis(E_AXIS) / state.extrude_factor);
    position
}

// ===========================================================================
// Commands
// ===========================================================================

/// An optional float word: upstream's `gcmd.get_float(name, None)` — an absent
/// word is `None`, one that does not parse is an error.
fn optional_float(gcmd: &GcodeCommand, name: &str) -> Result<Option<f64>, CommandError> {
    match gcmd.get_command_parameters().get(name) {
        None => Ok(None),
        Some(raw) => parse_float(raw).map(Some).ok_or_else(|| {
            CommandError::new(format!(
                "Error on '{}': unable to parse {}",
                gcmd.commandline(),
                raw
            ))
        }),
    }
}

/// A float word inside a move: upstream's `ValueError` there is
/// `Unable to parse move '…'`.
fn move_float(gcmd: &GcodeCommand, name: &str) -> Result<f64, CommandError> {
    let Some(raw) = gcmd.get_command_parameters().get(name) else {
        return Err(CommandError::new(format!(
            "Unable to parse move '{}'",
            gcmd.commandline()
        )));
    };
    parse_float(raw)
        .ok_or_else(|| CommandError::new(format!("Unable to parse move '{}'", gcmd.commandline())))
}

/// The move target, or "not ready" the way the toolhead says it.
fn target(object: &GCodeMove) -> Result<Arc<dyn MoveTarget>, CommandError> {
    object
        .target()
        .ok_or_else(|| CommandError::new("Printer is not ready"))
}

/// `G0`/`G1` (`gcode_move.py:117-141`): each axis word lands in
/// `last_position` — added to it in relative mode, set against `base_position`
/// in absolute — and the result is a toolhead coordinate.
fn cmd_g1(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let target = target(object)?;
    let (position, speed) = {
        let mut state = object.lock();
        let params = gcmd.get_command_parameters();
        for (word, axis) in [("X", 0usize), ("Y", 1), ("Z", 2), ("E", E_AXIS)] {
            if !params.contains_key(word) {
                continue;
            }
            let mut value = move_float(gcmd, word)?;
            let mut absolute = state.absolute_coord;
            if word == "E" {
                value *= state.extrude_factor;
                if !state.absolute_extrude {
                    absolute = false;
                }
            }
            let axis_position = if absolute {
                value + state.base_position[axis]
            } else {
                state.last_position.axis(axis) + value
            };
            state.last_position.set_axis(axis, axis_position);
        }
        if params.contains_key("F") {
            let feed = move_float(gcmd, "F")?;
            if feed <= 0.0 {
                return Err(CommandError::new(format!(
                    "Invalid speed in '{}'",
                    gcmd.commandline()
                )));
            }
            state.speed = feed * state.speed_factor;
        }
        (state.last_position, state.speed)
    };
    target.move_to(position, speed)
}

/// `G20`: inches. The port speaks millimeters only, as upstream refuses it.
fn cmd_g20(_object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    Err(CommandError::new(format!(
        "Machine does not support G20 (inches) command ({})",
        gcmd.commandline()
    )))
}

/// `G21`: millimeters — the only unit, so nothing to do.
fn cmd_g21(_object: &GCodeMove, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    Ok(())
}

/// `G90`: XYZ words are absolute.
fn cmd_g90(object: &GCodeMove, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.lock().absolute_coord = true;
    Ok(())
}

/// `G91`: XYZ words are relative to the last move.
fn cmd_g91(object: &GCodeMove, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.lock().absolute_coord = false;
    Ok(())
}

/// `M82`: E words are absolute.
fn cmd_m82(object: &GCodeMove, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.lock().absolute_extrude = true;
    Ok(())
}

/// `M83`: E words are relative to the last move.
fn cmd_m83(object: &GCodeMove, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.lock().absolute_extrude = false;
    Ok(())
}

/// `G92` (`gcode_move.py:185-196`): move the anchor so the g-code position
/// reads what you name. With no word at all, all four axes read zero.
fn cmd_g92(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let offsets = [
        optional_float(gcmd, "X")?,
        optional_float(gcmd, "Y")?,
        optional_float(gcmd, "Z")?,
        optional_float(gcmd, "E")?,
    ];
    let mut state = object.lock();
    if offsets.iter().all(Option::is_none) {
        // A bare `G92` re-reads as zero everywhere.
        for axis in 0..AXES {
            state.base_position[axis] = state.last_position.axis(axis);
        }
        return Ok(());
    }
    for (axis, offset) in offsets.into_iter().enumerate() {
        let Some(offset) = offset else { continue };
        let offset = if axis == E_AXIS {
            offset * state.extrude_factor
        } else {
            offset
        };
        state.base_position[axis] = state.last_position.axis(axis) - offset;
    }
    Ok(())
}

/// `M220`: the speed factor, percent of what was asked for.
fn cmd_m220(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let value =
        gcmd.get("S", Some(100.0), parse_float, None, None, Some(0.0), None)? / (60.0 * 100.0);
    let mut state = object.lock();
    let current = state.speed / state.speed_factor;
    state.speed = current * value;
    state.speed_factor = value;
    Ok(())
}

/// `M221`: the extrude factor, percent of what was asked for.
fn cmd_m221(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let new_factor = gcmd.get("S", Some(100.0), parse_float, None, None, Some(0.0), None)? / 100.0;
    let mut state = object.lock();
    let last_e = state.last_position.axis(E_AXIS);
    let e_value = (last_e - state.base_position[E_AXIS]) / state.extrude_factor;
    state.base_position[E_AXIS] = last_e - e_value * new_factor;
    state.extrude_factor = new_factor;
    Ok(())
}

/// `SET_GCODE_OFFSET` (`gcode_move.py:208-229`): move the anchor, remembering
/// each axis in `homing_position` so homing re-applies it. `MOVE=1` takes the
/// toolhead along at once.
fn cmd_set_gcode_offset(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let mut move_delta = [0.0; AXES];
    let (target, position, speed) = {
        let mut state = object.lock();
        for (axis, word) in [(0usize, "X"), (1, "Y"), (2, "Z"), (E_AXIS, "E")] {
            let offset = match optional_float(gcmd, word)? {
                Some(offset) => offset,
                None => {
                    let Some(adjust) = optional_float(gcmd, &format!("{word}_ADJUST"))? else {
                        continue;
                    };
                    adjust + state.homing_position[axis]
                }
            };
            let delta = offset - state.homing_position[axis];
            move_delta[axis] = delta;
            state.base_position[axis] += delta;
            state.homing_position[axis] = offset;
        }
        if gcmd.get_int_default("MOVE", 0)? == 0 {
            return Ok(());
        }
        // Only a move that takes the toolhead along needs a target.
        let target = target(object)?;
        let speed = gcmd.get(
            "MOVE_SPEED",
            Some(state.speed),
            parse_float,
            None,
            None,
            Some(0.0),
            None,
        )?;
        for (axis, delta) in move_delta.into_iter().enumerate() {
            let position = state.last_position.axis(axis) + delta;
            state.last_position.set_axis(axis, position);
        }
        (target, state.last_position, speed)
    };
    target.move_to(position, speed)
}

/// `SAVE_GCODE_STATE` (`gcode_move.py:232-245`): park the whole state under
/// `NAME` (default `default`).
fn cmd_save_gcode_state(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let name = gcmd.get_str_default("NAME", "default");
    let mut state = object.lock();
    let saved = SavedState {
        absolute_coord: state.absolute_coord,
        absolute_extrude: state.absolute_extrude,
        base_position: state.base_position,
        last_position: state.last_position,
        homing_position: state.homing_position,
        speed: state.speed,
        speed_factor: state.speed_factor,
        extrude_factor: state.extrude_factor,
    };
    state.saved_states.insert(name, saved);
    Ok(())
}

/// `RESTORE_GCODE_STATE` (`gcode_move.py:247-271`): take a parked state back,
/// keeping the extruder where it is (`base_position[E]` carries the
/// difference) and moving back with `MOVE=1`.
fn cmd_restore_gcode_state(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let name = gcmd.get_str_default("NAME", "default");
    let (saved, move_back, speed) = {
        let state = object.lock();
        let Some(saved) = state.saved_states.get(&name).cloned() else {
            return Err(CommandError::new(format!("Unknown g-code state: {name}")));
        };
        if gcmd.get_int_default("MOVE", 0)? == 0 {
            (saved, false, 0.0)
        } else {
            let speed = gcmd.get(
                "MOVE_SPEED",
                Some(state.speed),
                parse_float,
                None,
                None,
                Some(0.0),
                None,
            )?;
            (saved, true, speed)
        }
    };
    let position = {
        let mut state = object.lock();
        state.absolute_coord = saved.absolute_coord;
        state.absolute_extrude = saved.absolute_extrude;
        state.base_position = saved.base_position;
        state.homing_position = saved.homing_position;
        state.speed = saved.speed;
        state.speed_factor = saved.speed_factor;
        state.extrude_factor = saved.extrude_factor;
        // The extruder stays where it is: the difference goes into the anchor.
        let e_diff = state.last_position.axis(E_AXIS) - saved.last_position.axis(E_AXIS);
        state.base_position[E_AXIS] += e_diff;
        if move_back {
            for axis in 0..3 {
                state
                    .last_position
                    .set_axis(axis, saved.last_position.axis(axis));
            }
        }
        state.last_position
    };
    if move_back {
        target(object)?.move_to(position, speed)?;
    }
    Ok(())
}

/// `M114`: report the g-code position (`gcode_move.py:198-201`).
fn cmd_m114(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let state = object.lock();
    let position = gcode_position(&state);
    drop(state);
    gcmd.respond_raw(&format!(
        "X:{:.3} Y:{:.3} Z:{:.3} E:{:.3}",
        position.x(),
        position.y(),
        position.z(),
        position.axis(E_AXIS)
    ));
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::Z_AXIS;
    use crate::core::klippy::reactor::ManualReactor;

    /// A move target that records what it was asked and stands where the last
    /// move left it — the toolhead, as far as the coordinate math cares.
    struct FakeTarget {
        position: Mutex<Coord>,
        moves: Mutex<Vec<(Coord, f64)>>,
    }

    impl FakeTarget {
        fn new(position: Coord) -> Self {
            Self {
                position: Mutex::new(position),
                moves: Mutex::new(Vec::new()),
            }
        }

        fn moves(&self) -> Vec<(Coord, f64)> {
            self.moves
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone()
        }

        fn position(&self) -> Coord {
            *self
                .position
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
        }
    }

    impl MoveTarget for FakeTarget {
        fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
            *self
                .position
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()) = position;
            self.moves
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push((position, speed));
            Ok(())
        }

        fn position(&self) -> Coord {
            FakeTarget::position(self)
        }
    }

    /// A ready printer with `gcode` and `gcode_move` on it. There is no
    /// `toolhead` object, so ready leaves the target as the test set it.
    fn printer() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<GCodeMove>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let object = ensure(&printer).unwrap();
        let dispatch = gcode(&printer);
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, dispatch, object)
    }

    /// The machine with its move target standing at `position` and the state
    /// anchored to it — what ready leaves behind.
    fn machine(
        position: Coord,
    ) -> (
        Arc<Printer>,
        Arc<GCodeDispatch>,
        Arc<GCodeMove>,
        Arc<FakeTarget>,
    ) {
        let (printer, gcode, object) = printer();
        let target = Arc::new(FakeTarget::new(position));
        object
            .set_move_transform(Arc::clone(&target) as Arc<dyn MoveTarget>, true)
            .unwrap();
        printer.send_event(&KlippyEvent::ToolheadSetPosition);
        (printer, gcode, object, target)
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    /// The state's g-code position, as the four numbers a test compares.
    fn read_gcode_position(object: &GCodeMove) -> [f64; AXES] {
        let state = object.lock();
        let position = gcode_position(&state);
        position.axes().collect::<Vec<_>>().try_into().unwrap()
    }

    #[test]
    fn test_g1_parses_axes_and_speed_into_a_move() {
        let (_printer, gcode, _object, target) = machine(Coord::default());

        gcode.run_script_sync("G1 X10 F600").unwrap();

        // `F` is mm/min and a move is mm/s.
        assert_eq!(target.moves(), [(Coord::new(10.0, 0.0, 0.0, 0.0), 10.0)]);
    }

    #[test]
    fn test_a_g1_leaves_the_axes_it_does_not_name_where_they_were() {
        // Anchored away from the origin: an unnamed axis passes through.
        let (_printer, gcode, _object, target) = machine(Coord::new(1.0, 2.0, 3.0, 4.0));

        gcode.run_script_sync("G1 X10").unwrap();

        assert_eq!(
            target.position(),
            Coord::new(10.0, 2.0, 3.0, 4.0),
            "Y, Z and E keep their last position"
        );
    }

    #[test]
    fn test_g1_remembers_the_last_speed() {
        let (_printer, gcode, _object, target) = machine(Coord::default());

        gcode.run_script_sync("G1 X10 F600").unwrap();
        // The second move has no `F` and must reuse the first one.
        gcode.run_script_sync("G1 X20").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 2);
        assert_eq!(moves[1], (Coord::new(20.0, 0.0, 0.0, 0.0), 10.0));
    }

    #[test]
    fn test_a_non_positive_feedrate_is_rejected() {
        let (_printer, gcode, _object, _target) = machine(Coord::default());

        let err = gcode.run_script_sync("G1 X10 F0").unwrap_err();

        assert!(err.to_string().contains("Invalid speed"), "{err}");
    }

    #[test]
    fn test_g90_and_g91_switch_between_absolute_and_relative() {
        let (_printer, gcode, object, target) = machine(Coord::default());

        gcode.run_script_sync("G91").unwrap();
        gcode.run_script_sync("G1 X10").unwrap();
        gcode.run_script_sync("G1 X10").unwrap();
        assert_eq!(target.position().x(), 20.0, "relative adds");

        gcode.run_script_sync("G90").unwrap();
        gcode.run_script_sync("G1 X10").unwrap();
        assert_eq!(target.position().x(), 10.0, "absolute sets");
        assert_eq!(object.status()["absolute_coordinates"], true);
    }

    /// The shape of `move.gcode`: `G92 Y-3` re-anchors, so the absolute
    /// `G1 Y-2` that follows lands at toolhead Y=+1 (upstream) rather than
    /// walking out of range at Y=-2.
    #[test]
    fn test_g92_keeps_the_following_move_in_range() {
        let (_printer, gcode, object, target) = machine(Coord::new(0.0, 1.5, 0.0, 0.0));

        gcode
            .run_script_sync("G90\nG1 F6000\nG92 Y-3\nG1 Y-2")
            .unwrap();
        assert_eq!(
            target.position().y(),
            2.5,
            "`base_position[Y]` is 1.5 - (-3) = 4.5, so -2 + 4.5 = 2.5"
        );
        assert_eq!(read_gcode_position(&object)[1], -2.0);

        gcode.run_script_sync("G91\nG1 Y-1").unwrap();
        assert_eq!(
            target.position().y(),
            1.5,
            "relative from the anchored value"
        );
        assert_eq!(read_gcode_position(&object)[1], -3.0);
    }

    #[test]
    fn test_a_bare_g92_reads_every_axis_as_zero() {
        let (_printer, gcode, object, target) = machine(Coord::new(4.0, 5.0, 6.0, 7.0));

        gcode.run_script_sync("G92").unwrap();

        assert_eq!(read_gcode_position(&object), [0.0; AXES]);
        assert_eq!(target.position(), Coord::new(4.0, 5.0, 6.0, 7.0));
    }

    #[test]
    fn test_m83_makes_e_relative_while_the_axes_stay_absolute() {
        let (_printer, gcode, _object, target) = machine(Coord::default());

        gcode.run_script_sync("G1 E1").unwrap();
        assert_eq!(target.position().e(), 1.0);

        gcode.run_script_sync("M83").unwrap();
        gcode.run_script_sync("G1 E1 X10").unwrap();
        assert_eq!(target.position().e(), 2.0, "E accumulates");
        assert_eq!(target.position().x(), 10.0, "X is still absolute");
    }

    #[test]
    fn test_m220_and_m221_scale_speed_and_extrusion() {
        let (_printer, gcode, object, target) = machine(Coord::default());

        gcode.run_script_sync("G1 X10 F6000").unwrap();
        gcode.run_script_sync("M220 S50").unwrap();
        assert_eq!(object.status()["speed_factor"], 0.5);

        gcode.run_script_sync("G1 E20").unwrap();
        assert_eq!(target.position().e(), 20.0);

        gcode.run_script_sync("M221 S50").unwrap();
        // The g-code reading does not move; the toolhead halves what is asked.
        gcode.run_script_sync("G1 E40").unwrap();
        assert_eq!(target.position().e(), 30.0, "half of 20 more, from 20");
        assert_eq!(read_gcode_position(&object)[3], 40.0);
    }

    /// `SET_GCODE_OFFSET` remembers the axis so homing puts the anchor back
    /// (`_handle_home_rails_end`), which is what the `homing:home_rails_end`
    /// payload carries.
    #[test]
    fn test_the_anchor_is_reapplied_to_the_axes_that_homed() {
        let (printer, gcode, object, _target) = machine(Coord::new(0.0, 0.0, 5.0, 0.0));

        gcode.run_script_sync("SET_GCODE_OFFSET Z=2").unwrap();
        assert_eq!(object.status()["homing_origin"][2], 2.0);
        assert_eq!(
            read_gcode_position(&object)[2],
            3.0,
            "toolhead 5 - offset 2"
        );

        // `G92 Z0` moves the anchor off the homing value: the g-code position
        // reads zero from here on.
        gcode.run_script_sync("G92 Z0").unwrap();
        assert_eq!(read_gcode_position(&object)[2], 0.0);

        // Homing puts it back for the axis that homed — and only that axis.
        printer.send_event(&KlippyEvent::HomingHomeRailsEnd {
            axes: vec![Z_AXIS],
            homing: crate::core::klippy::motion::HomingHandle::new(),
        });

        assert_eq!(read_gcode_position(&object)[2], 3.0, "5 - homing 2");
        assert_eq!(object.status()["homing_origin"][2], 2.0);
    }

    #[test]
    fn test_save_and_restore_the_g_code_state() {
        let (_printer, gcode, object, target) = machine(Coord::default());

        gcode
            .run_script_sync("G90\nG1 X10\nSAVE_GCODE_STATE NAME=a")
            .unwrap();
        gcode.run_script_sync("G91\nG1 X5").unwrap();
        assert_eq!(target.position().x(), 15.0);

        // Without MOVE=1 the toolhead is left alone; the mode comes back.
        gcode.run_script_sync("RESTORE_GCODE_STATE NAME=a").unwrap();
        assert_eq!(target.position().x(), 15.0);
        assert_eq!(object.status()["absolute_coordinates"], true);

        // With it, the toolhead goes back to where it was parked.
        gcode.run_script_sync("G91\nG1 X5").unwrap();
        gcode
            .run_script_sync("RESTORE_GCODE_STATE NAME=a MOVE=1")
            .unwrap();
        assert_eq!(target.position().x(), 10.0);
    }

    #[test]
    fn test_a_restored_state_needs_a_name_that_was_saved() {
        let (_printer, gcode, _object, _target) = machine(Coord::default());

        let err = gcode
            .run_script_sync("RESTORE_GCODE_STATE NAME=nope")
            .unwrap_err();

        assert!(err.to_string().contains("Unknown g-code state"), "{err}");
    }

    #[test]
    fn test_m114_reports_the_g_code_position() {
        let (_printer, gcode, _object, _target) = machine(Coord::new(0.0, 1.5, 0.0, 0.0));
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string());
            }));
        }

        gcode.run_script_sync("M114").unwrap();

        let lines: Vec<String> = output.lock().unwrap().clone();
        assert_eq!(lines, ["X:0.000 Y:1.500 Z:0.000 E:0.000"]);
    }

    #[test]
    fn test_the_status_is_upstreams_get_status() {
        let (_printer, gcode, object, _target) = machine(Coord::default());

        gcode.run_script_sync("G1 X10 F600").unwrap();
        let status = object.status();

        assert_eq!(
            status["speed_factor"], 1.0,
            "`F 600` with the default factor"
        );
        assert_eq!(status["speed"], 600.0, "mm/min, as g-code reads it");
        assert_eq!(status["extrude_factor"], 1.0);
        assert_eq!(status["absolute_coordinates"], true);
        assert_eq!(status["absolute_extrude"], true);
        assert_eq!(status["homing_origin"], json!([0.0, 0.0, 0.0, 0.0]));
        assert_eq!(status["position"], json!([10.0, 0.0, 0.0, 0.0]));
        assert_eq!(status["gcode_position"], json!([10.0, 0.0, 0.0, 0.0]));
        assert_eq!(status["axis_map"], json!({"X": 0, "Y": 1, "Z": 2, "E": 3}));
    }

    #[test]
    fn test_a_second_transform_cannot_take_the_slot_silently() {
        let (_printer, _gcode, object, _target) = machine(Coord::default());

        let second = Arc::new(FakeTarget::new(Coord::default()));
        let err = object.set_move_transform(second, false).unwrap_err();

        assert_eq!(err.to_string(), "G-Code move transform already specified");
    }

    #[test]
    fn test_inches_are_refused() {
        let (_printer, gcode, _object, _target) = machine(Coord::default());

        let err = gcode.run_script_sync("G20").unwrap_err();

        assert!(err.to_string().contains("G20"), "{err}");
    }

    /// The declarations a client completes `KEY=` from, as they appear on
    /// `status.gcode.commands`: `G0` and `G1` answer with the one list their
    /// shared handler reads, `G20` declares nothing, and `SET_GCODE_OFFSET`
    /// pairs each axis with its `_ADJUST`.
    #[test]
    fn test_every_command_declares_the_parameters_it_reads() {
        let (_printer, gcode, _object, _target) = machine(Coord::default());

        let commands = gcode.get_status(0.0)["commands"].clone();
        assert_eq!(
            commands["G1"]["parameters"],
            json!(["X", "Y", "Z", "E", "F"])
        );
        assert_eq!(commands["G0"]["parameters"], commands["G1"]["parameters"]);
        assert_eq!(commands["G92"]["parameters"], json!(["X", "Y", "Z", "E"]));
        assert_eq!(commands["M220"]["parameters"], json!(["S"]));
        assert_eq!(commands["M221"]["parameters"], json!(["S"]));
        assert_eq!(
            commands["SET_GCODE_OFFSET"]["parameters"],
            json!([
                "X",
                "X_ADJUST",
                "Y",
                "Y_ADJUST",
                "Z",
                "Z_ADJUST",
                "E",
                "E_ADJUST",
                "MOVE",
                "MOVE_SPEED"
            ])
        );
        assert_eq!(commands["SAVE_GCODE_STATE"]["parameters"], json!(["NAME"]));
        assert_eq!(
            commands["RESTORE_GCODE_STATE"]["parameters"],
            json!(["NAME", "MOVE", "MOVE_SPEED"])
        );
        // No named word at all: the object a client saw before.
        assert!(commands["G20"].get("parameters").is_none());
    }
}
