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
//!   (`toolhead.rs:2333` / `:2349`, called by `manual_stepper`) |
//! | `extruder:activate_extruder` | registered, no sender yet (`ACTIVATE_EXTRUDER` does not fire it) |
//!
//! # Extra axes
//!
//! A `GCODE_AXIS` manual stepper (or a second extruder) adds an axis past `E`:
//! `toolhead:update_extra_axes` rebuilds [`MoveState::axis_map`], so the
//! letter registers address the extra position slots exactly as `X`/`Y`/`Z`/`E`
//! address the first four (`gcode_move.py:118-131`). `G1 A10` writes the extra
//! slot and hands the tail to [`MoveTarget::move_to_extended`], so it reaches
//! the planner as `Move::extra_start`/`extra_end` and the extra axis' own
//! `check_move`/`process_move` see their distance. Whether that becomes steps
//! is the axis' business: the extruder already queues its own trapq, a
//! `GCODE_AXIS` manual stepper does not yet (its module docs register that
//! gap).
//!
//! The move-transform slot itself is complete: `bed_mesh`
//! (`bed_mesh.rs:1851`), `bed_tilt` (`bed_tilt.rs:376`) and `exclude_object`
//! all take it through [`GCodeMove::set_move_transform`], and the last one
//! chains over whatever it displaced.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::toolhead::{PositionReport, ToolHeadObject};
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::mathutil::{Coord, AXES, E_AXIS, MAX_AXES};
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

    /// Queue a move that also drives the extra axes.
    ///
    /// `extras` is the tail of `last_position` — the positions of the extra
    /// axes, index 0 being the first one past `E`. A transform that does not
    /// model extra axes (`bed_tilt`, `exclude_object`, a test double) keeps the
    /// four-axis move by default.
    fn move_to_extended(
        &self,
        position: Coord,
        _extras: &[f64],
        speed: f64,
    ) -> Result<(), CommandError> {
        self.move_to(position, speed)
    }

    /// The toolhead's current position.
    fn position(&self) -> Coord;
}

/// The toolhead, as [`MoveTarget`] (upstream's ready-time resolution).
struct ToolheadTarget(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadTarget {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn move_to_extended(
        &self,
        position: Coord,
        extras: &[f64],
        speed: f64,
    ) -> Result<(), CommandError> {
        self.0.move_to_extended(position, extras, speed)
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
    ///
    /// Four axes plus one anchor per extra axis (`gcode_move.py:33` is a
    /// four-long list that `_update_extra_axes` extends at `:131`); only the
    /// first four are read or written by any command upstream.
    base_position: Coord,
    /// Where the **toolhead** is, per the last `G1` or re-anchor, four axes
    /// plus the extra-axis slots.
    last_position: Coord,
    /// What `SET_GCODE_OFFSET` remembers, re-applied to `base_position` after
    /// homing (`gcode_move.py:245-247`).
    homing_position: [f64; AXES],
    /// The g-code letters, mapped to their position index
    /// (`gcode_move.py:33`/`:118-131`): `X`…`E` always, then a letter per
    /// registered extra axis.
    axis_map: BTreeMap<String, usize>,
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
            base_position: Coord::default(),
            last_position: Coord::default(),
            homing_position: [0.0; AXES],
            axis_map: four_axis_map(),
            speed: DEFAULT_SPEED,
            speed_factor: 1.0 / 60.0,
            extrude_factor: 1.0,
            saved_states: HashMap::new(),
        }
    }
}

/// Upstream's `self.axis_map` initial value (`gcode_move.py:33`).
fn four_axis_map() -> BTreeMap<String, usize> {
    [("X", 0), ("Y", 1), ("Z", 2), ("E", E_AXIS)]
        .into_iter()
        .map(|(word, index)| (word.to_string(), index))
        .collect()
}

impl MoveState {
    /// Upstream's `_update_extra_axes` body (`gcode_move.py:118-133`): rebuild
    /// `axis_map` from the toolhead's extra axes, then resize the anchors.
    ///
    /// `extra_axes` are the g-code ids of the non-kinematic axes in position
    /// order — the extruder at index 0 (position [`E_AXIS`]), each
    /// `GCODE_AXIS` manual stepper after it. Upstream enumerates
    /// `get_extra_axes()` = `[None, None, None] + self.extra_axes`, so the
    /// padded index of `extra_axes[i]` is `E_AXIS + i` — the same position
    /// index the planner uses.
    fn update_extra_axes(&mut self, extra_axes: &[Option<String>]) {
        let mut axis_map = four_axis_map();
        for (index, gcode_id) in extra_axes.iter().enumerate() {
            // Slot 0 is the extrude slot ([`E_AXIS`]) in this port —
            // `ToolHead::add_extra_axis` puts the extruder there, and a lone
            // `GCODE_AXIS` stepper would take it when no extruder is
            // configured. Mapping its letter onto that slot would let `G1 A`
            // write the E position, so an index-0 axis never gets a letter;
            // upstream reserves the slot for the extruder and starts at 4.
            if index == 0 {
                continue;
            }
            let Some(gcode_id) = gcode_id else { continue };
            // The filter is upstream's, in its order (`gcode_move.py:126-128`):
            // one uppercase letter, not one of the four axes, not `F` (feed)
            // or `N` (line number).
            let mut letters = gcode_id.chars();
            let Some(letter) = letters.next() else {
                continue;
            };
            if letters.next().is_some() || !letter.is_ascii_uppercase() {
                continue;
            }
            if axis_map.contains_key(gcode_id) || letter == 'F' || letter == 'N' {
                continue;
            }
            // Past the container's capacity a letter would address a slot that
            // cannot exist; upstream's list has no such bound. With
            // [`MAX_AXES`] eight, indices 1..=4 map (positions 4..=7).
            if E_AXIS + index >= MAX_AXES {
                continue;
            }
            axis_map.insert(gcode_id.clone(), E_AXIS + index);
        }
        self.axis_map = axis_map;
        // `self.base_position[4:] = [0.] * (len(extra_axes) - 4)`: the anchors
        // of the extra axes start at zero and the list grows with them. The
        // list already holds the extruder at position [`E_AXIS`].
        let len = (E_AXIS + extra_axes.len()).min(MAX_AXES);
        let base_position = self
            .base_position
            .axes()
            .take(AXES)
            .chain(std::iter::repeat(0.0))
            .take(len);
        self.base_position = Coord::from_axes(base_position);
    }

    /// Grow or trim `last_position` to `len` axes, keeping the values it has.
    fn resize_last_position(&mut self, len: usize) {
        let axes: Vec<f64> = self
            .last_position
            .axes()
            .chain(std::iter::repeat(0.0))
            .take(len)
            .collect();
        self.last_position = Coord::from_axes(axes);
    }
}

/// One parked g-code state (`SAVE_GCODE_STATE`).
#[derive(Debug, Clone)]
struct SavedState {
    absolute_coord: bool,
    absolute_extrude: bool,
    base_position: Coord,
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

    /// Upstream's `set_move_transform` (`gcode_move.py:83-93`): take the slot
    /// `bed_mesh` fills, so moves go through a transform instead of the
    /// toolhead, and hand back the transform it displaced.
    ///
    /// The displaced occupant is what a chaining transform forwards to
    /// (`exclude_object`'s `next_transform`, `exclude_object.py:38-57`).
    /// Upstream returns the toolhead when the slot was still empty; here the
    /// empty slot stays `None`, because a caller that chains already holds the
    /// toolhead it captured at `klippy:connect`.
    ///
    /// # Errors
    /// When something already took the slot and `force` is not set; the
    /// occupant then stays put.
    pub fn set_move_transform(
        &self,
        target: Arc<dyn MoveTarget>,
        force: bool,
    ) -> Result<Option<Arc<dyn MoveTarget>>, ConfigError> {
        let mut slot = self
            .target
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if slot.is_some() && !force {
            return Err(ConfigError::new("G-Code move transform already specified"));
        }
        Ok(slot.replace(target))
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
    ///
    /// The toolhead reports the four modelled axes; the extra-axis tail is
    /// carried over from the last move, because upstream reads it back from the
    /// toolhead's own `commanded_pos`, which does not track extra axes in this
    /// port yet (see the module docs).
    pub fn reset_last_position(&self) {
        let Some(target) = self.target() else {
            return;
        };
        let position = target.position();
        let mut state = self.lock();
        let len = state.base_position.len();
        // `toolhead.position()` carries the four modelled axes, so the
        // extra-axis tail keeps the values of the last move until the toolhead
        // reports the tail itself (upstream reads it out of `commanded_pos`).
        let tail = state.last_position.as_slice().get(AXES..).unwrap_or(&[]);
        let axes = position.axes().chain(tail.iter().copied());
        state.last_position = Coord::from_axes(axes);
        state.resize_last_position(len);
    }

    /// Upstream's `_update_extra_axes` (`gcode_move.py:118-133`), driven by the
    /// toolhead's `toolhead:update_extra_axes` event.
    fn update_extra_axes(&self) {
        let extra_axes: Vec<Option<String>> = self
            .printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .map(|toolhead| {
                toolhead
                    .get_extra_axes()
                    .iter()
                    .map(|axis| axis.axis_gcode_id())
                    .collect()
            })
            .unwrap_or_default();
        self.update_extra_axes_from(&extra_axes);
    }

    /// [`Self::update_extra_axes`] with the axes handed in — the event's math,
    /// testable without a toolhead.
    fn update_extra_axes_from(&self, extra_axes: &[Option<String>]) {
        self.lock().update_extra_axes(extra_axes);
        self.reset_last_position();
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
        ///
        /// A registered `GCODE_AXIS` letter is read too, but the table is built
        /// here, before any extra axis can register, so a client is offered the
        /// four axes only.
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
            (
                "GET_POSITION",
                cmd_get_position,
                Some("Return information on the current location of the toolhead"),
                &[],
                true,
            ),
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
            KlippyEvent::ToolheadUpdateExtraAxes,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.update_extra_axes()
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
                state.base_position.set_axis(axis, homing);
            }
        }
    }

    /// Upstream's `_handle_activate_extruder`.
    fn handle_activate_extruder(&self) {
        self.reset_last_position();
        let mut state = self.lock();
        state.extrude_factor = 1.0;
        let base = state.last_position.axis(E_AXIS);
        state.base_position.set_axis(E_AXIS, base);
    }

    /// Upstream's `gcode_move.cmd_G1(gcmd)`: run a move whose command was
    /// built with [`GCodeDispatch::create_gcode_command`] through the `G1`
    /// handler, which is how `gcode_arcs` sends each arc segment
    /// (`gcode_arcs.py:179-180`).
    ///
    /// # Errors
    /// As `G1`: an unparseable word, a non-positive `F`, or whatever the move
    /// target refuses.
    pub fn cmd_g1(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        cmd_g1(self, gcmd)
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
            // Built from the toolhead's extra axes (`gcode_move.py:118-131`),
            // not from a constant: a `GCODE_AXIS` letter is in here.
            "axis_map": state.axis_map,
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
    // `zip(self.last_position, self.base_position)`: every axis the state
    // carries, the extra ones included.
    for axis in 0..position.len() {
        let base = state.base_position.axis(axis);
        position.set_axis(axis, position.axis(axis) - base);
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

/// `G0`/`G1` (`gcode_move.py:134-157`): each axis word lands in
/// `last_position` — added to it in relative mode, set against `base_position`
/// in absolute — and the result is a toolhead coordinate.
///
/// The words come from [`MoveState::axis_map`], so a registered `GCODE_AXIS`
/// letter moves the extra axis exactly as `X`/`Y`/`Z` move theirs; only `E` has
/// an exception (the extrude factor and `M82`/`M83`).
fn cmd_g1(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let target = target(object)?;
    let (position, speed) = {
        let mut guard = object.lock();
        // The fields are destructured so the words can be walked while
        // `last_position` is written; a per-move copy of the map would
        // allocate on every `G1`.
        let MoveState {
            absolute_coord,
            absolute_extrude,
            base_position,
            last_position,
            axis_map,
            speed,
            speed_factor,
            extrude_factor,
            ..
        } = &mut *guard;
        let params = gcmd.get_command_parameters();
        for (word, axis) in axis_map.iter() {
            if !params.contains_key(word) {
                continue;
            }
            let mut value = move_float(gcmd, word)?;
            let mut absolute = *absolute_coord;
            if word.as_str() == "E" {
                value *= *extrude_factor;
                if !*absolute_extrude {
                    absolute = false;
                }
            }
            let axis_position = if absolute {
                value + base_position.axis(*axis)
            } else {
                last_position.axis(*axis) + value
            };
            last_position.set_axis(*axis, axis_position);
        }
        if params.contains_key("F") {
            let feed = move_float(gcmd, "F")?;
            if feed <= 0.0 {
                return Err(CommandError::new(format!(
                    "Invalid speed in '{}'",
                    gcmd.commandline()
                )));
            }
            *speed = feed * *speed_factor;
        }
        (*last_position, *speed)
    };
    move_with_transform(&target, position, speed)
}

/// Upstream's `move_with_transform`: queue a toolhead move to `position`.
///
/// A [`Coord`] holds the four modelled axes and carries the extra axes behind
/// them, so the tail rides beside the move (`Move::extra_start`/`extra_end`) —
/// the planner and every [`ExtraAxis`](crate::core::klippy::motion::extra::ExtraAxis)
/// then see the extra axis' distance.
fn move_with_transform(
    target: &Arc<dyn MoveTarget>,
    position: Coord,
    speed: f64,
) -> Result<(), CommandError> {
    let extras = position.as_slice().get(AXES..).unwrap_or(&[]);
    let toolhead = Coord::new(position.x(), position.y(), position.z(), position.e());
    target.move_to_extended(toolhead, extras, speed)
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
        // A bare `G92` re-reads as zero everywhere — over the four axes only
        // (`gcode_move.py:190`); the extra axes keep their anchors.
        for axis in 0..AXES {
            let last = state.last_position.axis(axis);
            state.base_position.set_axis(axis, last);
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
        let base = state.last_position.axis(axis) - offset;
        state.base_position.set_axis(axis, base);
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
    let e_value = (last_e - state.base_position.axis(E_AXIS)) / state.extrude_factor;
    state
        .base_position
        .set_axis(E_AXIS, last_e - e_value * new_factor);
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
            let base = state.base_position.axis(axis) + delta;
            state.base_position.set_axis(axis, base);
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
    move_with_transform(&target, position, speed)
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
        // Upstream restores the four anchors only (`gcode_move.py:248`): the
        // extra axes keep whatever anchor they have now.
        for axis in 0..AXES {
            let saved = saved.base_position.axis(axis);
            state.base_position.set_axis(axis, saved);
        }
        state.homing_position = saved.homing_position;
        state.speed = saved.speed;
        state.speed_factor = saved.speed_factor;
        state.extrude_factor = saved.extrude_factor;
        // The extruder stays where it is: the difference goes into the anchor.
        let e_diff = state.last_position.axis(E_AXIS) - saved.last_position.axis(E_AXIS);
        let base_e = state.base_position.axis(E_AXIS) + e_diff;
        state.base_position.set_axis(E_AXIS, base_e);
        if move_back {
            for axis in 0..3 {
                let saved = saved.last_position.axis(axis);
                state.last_position.set_axis(axis, saved);
            }
        }
        state.last_position
    };
    if move_back {
        move_with_transform(&target(object)?, position, speed)?;
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

/// `GET_POSITION` (`gcode_move.py:261-291`): where the machine thinks it is,
/// from the MCU step counts down to the g-code offset.
///
/// It answers the three questions a motion problem splits into — are the
/// steps right, does the kinematics invert them right, are the offsets right —
/// in one reply. Unlike `M114` it reads the **toolhead**, not the move
/// transform, so a `bed_mesh` compensation shows up in `M114` but not in the
/// `toolhead:` line here.
fn cmd_get_position(object: &GCodeMove, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let toolhead = object
        .printer
        .upgrade()
        .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT));
    let Some(toolhead) = toolhead else {
        return Err(CommandError::new("Printer not ready"));
    };
    // A toolhead that exists but has not connected has nothing to report
    // either (`gcode_move.py:265-266` is the same refusal).
    let Some(report) = toolhead.position_report() else {
        return Err(CommandError::new("Printer not ready"));
    };
    let position = toolhead.position().unwrap_or_default();
    let lines = position_lines(&report, position, &object.lock());
    gcmd.respond_info(&lines);
    Ok(())
}

/// The seven lines `GET_POSITION` answers with (`gcode_move.py:283-291`),
/// verbatim: the two stepper lines in `kin.get_steppers()` order,
/// `kinematic:` and `gcode homing:` for X/Y/Z only, everything six decimals
/// except the MCU step counts, which are integers.
fn position_lines(report: &PositionReport, toolhead_position: Coord, state: &MoveState) -> String {
    let mcu_pos = report
        .steppers
        .iter()
        .map(|stepper| format!("{}:{}", stepper.name, stepper.mcu_position))
        .collect::<Vec<_>>()
        .join(" ");
    let stepper_pos = report
        .steppers
        .iter()
        .map(|stepper| format!("{}:{:.6}", stepper.name, stepper.commanded_position))
        .collect::<Vec<_>>()
        .join(" ");
    // An axis the inverse solver cannot recover falls back to the commanded
    // position, which is what `calc_toolhead_pos` does for the same case
    // (`homing.py:74-75`: `cp if cp is not None else tp`). Upstream's
    // `GET_POSITION` has no answer there — it formats the `None` and raises.
    let kin_pos = report
        .kinematic
        .into_iter()
        .enumerate()
        .map(|(axis, value)| {
            format!(
                "{}:{:.6}",
                ["X", "Y", "Z"][axis],
                value.unwrap_or(toolhead_position.axis(axis))
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    // `homing_position` is four slots here (`gcode_move.py:32`'s has three);
    // upstream prints the three axes it hands the kinematics.
    let homing_pos = ["X", "Y", "Z"]
        .into_iter()
        .zip(state.homing_position)
        .map(|(axis, value)| format!("{axis}:{value:.6}"))
        .collect::<Vec<_>>()
        .join(" ");
    format!(
        "mcu: {mcu_pos}\n\
         stepper: {stepper_pos}\n\
         kinematic: {kin_pos}\n\
         toolhead: {}\n\
         gcode: {}\n\
         gcode base: {}\n\
         gcode homing: {homing_pos}",
        xyze_line(toolhead_position.as_slice()),
        xyze_line(state.last_position.as_slice()),
        xyze_line(state.base_position.as_slice()),
    )
}

/// `X:` … `E:` with six decimals — the shape of `GET_POSITION`'s four-axis
/// lines (`zip("XYZE", position)`, `gcode_move.py:275-280`).
fn xyze_line(position: &[f64]) -> String {
    ["X", "Y", "Z", "E"]
        .into_iter()
        .zip(position)
        .map(|(axis, value)| format!("{axis}:{value:.6}"))
        .collect::<Vec<_>>()
        .join(" ")
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::extras::toolhead::StepperReport;
    use crate::core::klippy::mathutil::{X_AXIS, Z_AXIS};
    use crate::core::klippy::reactor::ManualReactor;

    /// A move target that records what it was asked and stands where the last
    /// move left it — the toolhead, as far as the coordinate math cares.
    struct FakeTarget {
        position: Mutex<Coord>,
        moves: Mutex<Vec<(Coord, f64)>>,
        extras: Mutex<Vec<Vec<f64>>>,
    }

    impl FakeTarget {
        fn new(position: Coord) -> Self {
            Self {
                position: Mutex::new(position),
                moves: Mutex::new(Vec::new()),
                extras: Mutex::new(Vec::new()),
            }
        }

        fn moves(&self) -> Vec<(Coord, f64)> {
            self.moves
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone()
        }

        /// The extra-axis tails the moves carried, one per move.
        fn extras(&self) -> Vec<Vec<f64>> {
            self.extras
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

        fn move_to_extended(
            &self,
            position: Coord,
            extras: &[f64],
            speed: f64,
        ) -> Result<(), CommandError> {
            self.extras
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push(extras.to_vec());
            self.move_to(position, speed)
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

    /// The report a cartesian machine standing at X12.5 Y-25 gives — the
    /// shape upstream's own example has (`gcode_move.py:269-282`).
    fn cartesian_report() -> PositionReport {
        PositionReport {
            steppers: vec![
                StepperReport {
                    name: "stepper_x".to_string(),
                    mcu_position: 100,
                    commanded_position: 12.5,
                },
                StepperReport {
                    name: "stepper_y".to_string(),
                    mcu_position: -200,
                    commanded_position: -25.0,
                },
                StepperReport {
                    name: "stepper_z".to_string(),
                    mcu_position: 0,
                    commanded_position: 0.0,
                },
            ],
            kinematic: [Some(12.5), Some(-25.0), Some(0.0)],
        }
    }

    /// The whole reply: seven lines, the stepper lines in the order the report
    /// gives them, `mcu:` in integers and everything else six decimals,
    /// `kinematic:`/`gcode homing:` for three axes only.
    #[test]
    fn test_get_position_prints_the_seven_lines() {
        let position = Coord::new(12.5, -25.0, 0.0, 0.0);
        let state = MoveState {
            last_position: position,
            ..Default::default()
        };

        let lines = position_lines(&cartesian_report(), position, &state);

        assert_eq!(
            lines,
            "mcu: stepper_x:100 stepper_y:-200 stepper_z:0\n\
             stepper: stepper_x:12.500000 stepper_y:-25.000000 stepper_z:0.000000\n\
             kinematic: X:12.500000 Y:-25.000000 Z:0.000000\n\
             toolhead: X:12.500000 Y:-25.000000 Z:0.000000 E:0.000000\n\
             gcode: X:12.500000 Y:-25.000000 Z:0.000000 E:0.000000\n\
             gcode base: X:0.000000 Y:0.000000 Z:0.000000 E:0.000000\n\
             gcode homing: X:0.000000 Y:0.000000 Z:0.000000"
        );
    }

    /// `M114` answers `last - base` (and E divided by the factor), but
    /// `GET_POSITION` prints the two arrays as they are — that is the one
    /// difference a reader of both rows has to know (`gcode_move.py:277-280`).
    #[test]
    fn test_get_position_reports_gcode_and_base_as_they_are() {
        let state = MoveState {
            last_position: Coord::new(10.0, 20.0, 30.0, 40.0),
            base_position: Coord::new(1.0, 2.0, 3.0, 4.0),
            homing_position: [0.5, 0.5, 0.5, 0.5],
            extrude_factor: 2.0,
            ..Default::default()
        };

        let lines = position_lines(&cartesian_report(), Coord::default(), &state);
        let reported: Vec<&str> = lines
            .lines()
            .filter(|line| line.starts_with("gcode"))
            .collect();

        assert_eq!(
            reported,
            [
                "gcode: X:10.000000 Y:20.000000 Z:30.000000 E:40.000000",
                "gcode base: X:1.000000 Y:2.000000 Z:3.000000 E:4.000000",
                "gcode homing: X:0.500000 Y:0.500000 Z:0.500000",
            ]
        );
    }

    /// An axis the inverse solver cannot recover is printed from the commanded
    /// position, **per axis** — `calc_toolhead_pos` does the same
    /// (`homing.py:74-75`: `cp if cp is not None else tp`), and upstream's
    /// `GET_POSITION` has no answer at all there (it formats the `None` and
    /// raises).
    #[test]
    fn test_get_position_falls_back_to_the_commanded_position() {
        let mut report = cartesian_report();
        // Only Y is unrecoverable, and the two that are keep their solved
        // values — neither the commanded ones nor a stand-in zero.
        report.kinematic = [Some(11.0), None, Some(1.0)];
        let position = Coord::new(12.5, -25.0, 3.0, 0.0);
        let state = MoveState::default();

        let lines = position_lines(&report, position, &state);

        assert_eq!(
            lines.lines().nth(2),
            Some("kinematic: X:11.000000 Y:-25.000000 Z:1.000000")
        );
    }

    /// Without a `toolhead` object the command refuses with upstream's exact
    /// wording (`gcode_move.py:265-266`) — `M114` answers in the same state.
    #[test]
    fn test_get_position_needs_a_toolhead() {
        let (_printer, gcode, _object, _target) = machine(Coord::default());

        let err = gcode.run_script_sync("GET_POSITION").unwrap_err();

        assert_eq!(err.to_string(), "Printer not ready");
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
        let err = object
            .set_move_transform(second, false)
            .err()
            .expect("the slot is taken");

        assert_eq!(err.to_string(), "G-Code move transform already specified");
    }

    /// Upstream's `set_move_transform` returns the occupant it displaced
    /// (`gcode_move.py:83-93`) — `exclude_object` chains on that value — and a
    /// refused (`force = false`) registration leaves the occupant in place.
    #[test]
    fn test_set_move_transform_returns_the_displaced_occupant() {
        let (_printer, _gcode, object) = printer();

        let first = Arc::new(FakeTarget::new(Coord::default()));
        assert!(
            object
                .set_move_transform(Arc::clone(&first) as Arc<dyn MoveTarget>, false)
                .unwrap()
                .is_none(),
            "an empty slot displaces nothing"
        );

        let second = Arc::new(FakeTarget::new(Coord::default()));
        assert!(
            object
                .set_move_transform(Arc::clone(&second) as Arc<dyn MoveTarget>, false)
                .is_err(),
            "the slot is taken"
        );
        let displaced = object
            .set_move_transform(Arc::clone(&second) as Arc<dyn MoveTarget>, true)
            .unwrap()
            .expect("the refused call left the first occupant in place");
        assert!(Arc::ptr_eq(
            &displaced,
            &(Arc::clone(&first) as Arc<dyn MoveTarget>)
        ));
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
        // `GET_POSITION` reads no word either, and is in the table at all (it
        // answers before ready, like `M114`).
        assert!(commands["GET_POSITION"].get("parameters").is_none());
    }

    // -------------------------------------------------------------------
    // Extra axes (`gcode_move.py:118-131`): a `GCODE_AXIS` letter is a
    // position word like `X` is. The machine below registers a stand-in
    // extruder at position 3 and an `A` axis at position 4, which is the order
    // a real config has (`ToolHead.connect` adds the extruders first).
    // -------------------------------------------------------------------
    fn extra_axes() -> Vec<Option<String>> {
        vec![Some("E".to_string()), Some("A".to_string())]
    }

    fn axis_map(object: &GCodeMove) -> Value {
        object.status()["axis_map"].clone()
    }

    #[test]
    fn test_axis_map_grows_from_the_toolheads_extra_axes() {
        let (_printer, _gcode, object, _target) = machine(Coord::default());

        object.update_extra_axes_from(&extra_axes());

        assert_eq!(
            axis_map(&object),
            json!({"X": 0, "Y": 1, "Z": 2, "E": 3, "A": 4})
        );
    }

    #[test]
    fn test_axis_map_skips_the_letters_upstream_refuses() {
        let (_printer, _gcode, object, _target) = machine(Coord::default());

        // `None` (no id), empty, two letters, lowercase, `F`, `N`, and a letter
        // that is already an axis (`gcode_move.py:126-128`).
        object.update_extra_axes_from(&[
            None,
            Some(String::new()),
            Some("ab".to_string()),
            Some("a".to_string()),
            Some("F".to_string()),
            Some("N".to_string()),
            Some("E".to_string()),
        ]);
        assert_eq!(axis_map(&object), json!({"X": 0, "Y": 1, "Z": 2, "E": 3}));

        // A letter already in the map keeps its first position.
        object.update_extra_axes_from(&[
            Some("E".to_string()),
            Some("A".to_string()),
            Some("A".to_string()),
        ]);
        assert_eq!(axis_map(&object)["A"], json!(4));
    }

    #[test]
    fn test_g1_addresses_an_extra_axis_by_its_letter() {
        let (_printer, gcode, object, target) = machine(Coord::default());
        object.update_extra_axes_from(&extra_axes());

        gcode.run_script_sync("G1 A5").unwrap();
        assert_eq!(
            object.status()["position"],
            json!([0.0, 0.0, 0.0, 0.0, 5.0]),
            "absolute: the letter writes position slot 4"
        );

        gcode.run_script_sync("G91").unwrap();
        gcode.run_script_sync("G1 A2").unwrap();
        assert_eq!(object.status()["position"][4], json!(7.0), "relative adds");

        // The four modelled axes behave exactly as before.
        gcode.run_script_sync("G90").unwrap();
        gcode.run_script_sync("G1 X10").unwrap();
        assert_eq!(
            target.position(),
            Coord::new(10.0, 0.0, 0.0, 0.0),
            "the move out of this module carries four axes"
        );
        assert_eq!(
            object.status()["position"],
            json!([10.0, 0.0, 0.0, 0.0, 7.0])
        );
    }

    #[test]
    fn test_g1_hands_the_extra_axis_tail_to_the_planner() {
        let (_printer, gcode, object, target) = machine(Coord::default());
        object.update_extra_axes_from(&extra_axes());

        gcode.run_script_sync("G1 X10 A10").unwrap();

        // The four axes stay the toolhead coordinate; the extra axis' position
        // travels beside it as the move's tail (`Move::extra_end`).
        assert_eq!(target.extras(), [vec![10.0]]);
        assert_eq!(
            target.moves(),
            [(Coord::new(10.0, 0.0, 0.0, 0.0), DEFAULT_SPEED)]
        );

        // A move that does not name the extra axis keeps where it is.
        gcode.run_script_sync("G1 X20").unwrap();
        assert_eq!(target.extras(), [vec![10.0], vec![10.0]]);
    }

    #[test]
    fn test_a_bare_g92_leaves_the_extra_axis_anchor_alone() {
        let (_printer, gcode, object, _target) = machine(Coord::default());
        object.update_extra_axes_from(&extra_axes());
        // As if the extra axis' anchor had been set somehow.
        object.lock().base_position.set_axis(4, 2.0);

        gcode.run_script_sync("G1 A5").unwrap();
        gcode.run_script_sync("G92").unwrap();

        // Upstream's bare `G92` re-anchors `base_position[:4]` only
        // (`gcode_move.py:190`), so the A axis still reads its g-code value.
        let state = object.lock();
        assert_eq!(state.base_position.len(), 5, "the tail survives");
        assert_eq!(
            state.base_position.axis(4),
            2.0,
            "the A anchor is untouched"
        );
        drop(state);
        assert_eq!(object.status()["gcode_position"][4], json!(5.0));
    }

    #[test]
    fn test_restore_gcode_state_restores_only_the_first_four_anchors() {
        let (_printer, gcode, object, _target) = machine(Coord::default());
        object.update_extra_axes_from(&extra_axes());
        {
            let mut state = object.lock();
            state.base_position.set_axis(X_AXIS, 1.0);
            state.base_position.set_axis(4, 2.0);
        }

        gcode
            .run_script_sync("SAVE_GCODE_STATE NAME=parked")
            .unwrap();
        {
            let mut state = object.lock();
            state.base_position.set_axis(X_AXIS, 9.0);
            state.base_position.set_axis(4, 8.0);
        }
        gcode
            .run_script_sync("RESTORE_GCODE_STATE NAME=parked")
            .unwrap();

        let state = object.lock();
        assert_eq!(state.base_position.axis(X_AXIS), 1.0, "X is restored");
        assert_eq!(
            state.base_position.axis(4),
            8.0,
            "`base_position[:4] = state['base_position'][:4]`, upstream :248"
        );
        assert_eq!(state.base_position.len(), 5);
    }

    #[test]
    fn test_update_extra_axes_forgets_an_unregistered_axis() {
        let (_printer, _gcode, object, _target) = machine(Coord::default());
        object.update_extra_axes_from(&extra_axes());

        object.update_extra_axes_from(&[Some("E".to_string())]);

        assert_eq!(axis_map(&object), json!({"X": 0, "Y": 1, "Z": 2, "E": 3}));
        assert_eq!(object.status()["position"], json!([0.0, 0.0, 0.0, 0.0]));
    }

    /// A lone `GCODE_AXIS` stepper (no extruder configured) sits in slot [`E_AXIS`]
    /// in this port, so it must **not** get a letter: `G1 A` would land in the
    /// E position.
    #[test]
    fn test_a_lone_manual_stepper_is_not_mapped_onto_the_e_slot() {
        let (_printer, gcode, object, _target) = machine(Coord::default());
        object.update_extra_axes_from(&[Some("A".to_string())]);

        assert_eq!(
            axis_map(&object),
            json!({"X": 0, "Y": 1, "Z": 2, "E": 3}),
            "slot 0 is the extruder slot; an axis there gets no letter"
        );

        gcode.run_script_sync("G1 E5").unwrap();
        gcode.run_script_sync("G1 A10").unwrap();
        assert_eq!(
            object.status()["position"],
            json!([0.0, 0.0, 0.0, 5.0]),
            "A must not write the E slot"
        );
    }

    /// The capacity cap, pinned: with `MAX_AXES` eight the four extra axes
    /// after the extruder map (positions 4..=7) and a fifth does not. Raising
    /// `MAX_AXES` has to revisit this assertion.
    #[test]
    fn test_a_fifth_extra_axis_is_not_mapped() {
        let (_printer, _gcode, object, _target) = machine(Coord::default());

        object.update_extra_axes_from(&[
            Some("E".to_string()),
            Some("A".to_string()),
            Some("B".to_string()),
            Some("C".to_string()),
            Some("D".to_string()),
            Some("G".to_string()),
        ]);

        let map = axis_map(&object);
        assert_eq!(map["A"], json!(4));
        assert_eq!(map["B"], json!(5));
        assert_eq!(map["C"], json!(6));
        assert_eq!(map["D"], json!(7));
        assert!(
            map.get("G").is_none(),
            "the fifth extra axis has no slot: {map}"
        );
        assert_eq!(map.as_object().unwrap().len(), MAX_AXES);
        assert_eq!(
            object.status()["position"].as_array().unwrap().len(),
            MAX_AXES,
            "the position is capped at the container's capacity"
        );
    }
}
