//! `[exclude_object]` — define, track and exclude print objects
//! (upstream `klippy/extras/exclude_object.py`).
//!
//! The section reads **no options** (`exclude_object.py:12-36`): what it does
//! at load is register the four commands and keep the object state the
//! `[gcode_macro M486]` body drives through them.
//!
//! | command | role (`exclude_object.py`) |
//! |---|---|
//! | `EXCLUDE_OBJECT_START` | mark the current object (`:190-198`) |
//! | `EXCLUDE_OBJECT_END` | clear the current object (`:200-213`) |
//! | `EXCLUDE_OBJECT` | reset / exclude by name or current / list (`:215-238`) |
//! | `EXCLUDE_OBJECT_DEFINE` | define an object, or reset the file (`:240-267`) |
//!
//! [`ExcludeObject`] also implements [`MoveTarget`]: while the current object
//! is excluded, moves are dropped instead of forwarded to the toolhead —
//! that is what keeps `G0 X-11` inside an excluded object from being a
//! "Move out of range" (`exclude_object.py:102-172`, the transform).
//!
//! # Gaps this port does not close yet (H4)
//!
//! - ~~The corpus could not turn green on this file alone.~~ `gcode_macro`
//!   renders the `M486` body now (`gcode_macro.rs`), so the `EXCLUDE_*` lines
//!   run; the extrusion-offset compensation above is what keeps the moves
//!   after an excluded region inside the extrusion limits.
//! - **Extrusion offsets are keyed once, not per extruder.** The excluded
//!   region's E compensation — `offset[3]`, `extruder_adj`,
//!   `last_position_extruded` / `last_position_excluded`,
//!   `initial_extrusion_moves` and the XY catch-up on the way out
//!   (`exclude_object.py:102-172`) — is ported in full; upstream stores the
//!   offsets in a map keyed by the active extruder's name
//!   (`_get_extrusion_offsets`, `:94-101`) and this port keeps one array,
//!   which is the same value while one extruder prints.
//! - **The transform does not chain over other transforms.** Upstream keeps
//!   the previous slot occupant (`next_transform`, `exclude_object.py:38-57`)
//!   because Python's `set_move_transform` returns it; this port's
//!   `gcode_move::set_move_transform` does not, so the transform forwards to
//!   the toolhead captured at `klippy:connect`. A config that combines this
//!   with `bed_mesh`/`bed_tilt` would lose their adjustment while excluding.
//!   The `tuning_tower` guard (`:47-53`) has no ported tuning tower to consult.
//! - **`virtual_sdcard:reset_file` is not wired** (`:29-30`): upstream resets
//!   on a new file; here `EXCLUDE_OBJECT_DEFINE RESET=1` is the only reset.
//!   `was_excluded_at_start` (`:197`) is not tracked either.
//! - Malformed `CENTER=`/`POLYGON=` JSON reports this crate's parser wording,
//!   not Python's `json.JSONDecodeError`.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_move::{self, GCodeMove, MoveTarget, GCODE_MOVE_OBJECT};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, E_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

section!("exclude_object", order = 30, load = load_config);

/// The object state upstream keeps in `_reset_state` (`exclude_object.py:75-79`).
#[derive(Debug, Default)]
struct State {
    /// Defined objects, sorted by name (`exclude_object.py:_add_object_definition`).
    objects: Vec<Value>,
    /// Excluded object names, sorted (`exclude_object.py:_exclude_object`).
    excluded_objects: Vec<String>,
    /// The name of the object currently being printed, if any
    /// (`exclude_object.py:cmd_EXCLUDE_OBJECT_START`).
    current_object: Option<String>,
    /// Whether the transform has taken `gcode_move`'s slot
    /// (`exclude_object.py:_register_transform`).
    transform_registered: bool,
    /// The excluded-region move bookkeeping, armed when the transform
    /// registers (`exclude_object.py:_register_transform:47-64`); `None`
    /// while no exclusion has claimed the slot, when the transform forwards
    /// raw positions as before.
    motion: Option<ExcludedMotion>,
}

/// The transform's position bookkeeping (`exclude_object.py:_register_transform`
/// and `_normal_move`/`_ignore_move`, `:103-146`).
///
/// While an object is excluded its moves are dropped, but their **E** still
/// accumulates in `offset[3]`, so the first move out of the region subtracts
/// it again — the toolhead never extrudes the cancelled filament, and the
/// gcode coordinate keeps counting it. `offset[0..2]` carry the transient XY
/// correction so the first XY move away from the excluded end starts from the
/// last *extruded* position; `extruder_adj` compensates a retraction
/// difference across the boundary.
#[derive(Debug, Default)]
struct ExcludedMotion {
    /// The gcode-side position the transform reports (`get_position`).
    last_position: Coord,
    /// The last position that actually extruded (`_normal_move`).
    last_position_extruded: Coord,
    /// The last position inside an excluded region (`_ignore_move`).
    last_position_excluded: Coord,
    /// The furthest E reached printed / excluded (`max_position_*`).
    max_position_extruded: f64,
    max_position_excluded: f64,
    /// Retraction compensation carried across a region boundary
    /// (`_move_from_excluded_region`).
    extruder_adj: f64,
    /// The per-axis offsets subtracted from every forwarded position
    /// (`_get_extrusion_offsets`; upstream keys these per extruder — module
    /// docs).
    offset: [f64; 4],
    /// Upstream arms the transform with five tracked extrusion moves before
    /// exclusions apply (`_register_transform`, `initial_extrusion_moves = 5`).
    initial_extrusion_moves: i32,
    /// Whether the last move was inside the excluded region
    /// (`move()`, `exclude_object.py:182-195`).
    in_excluded_region: bool,
}

/// The `[exclude_object]` section (`exclude_object.py:ExcludeObject`).
pub struct ExcludeObject {
    /// The object state, behind a lock the command handlers and the move
    /// transform both reach.
    state: Mutex<State>,
    /// The transform chain below this one — the toolhead, captured at
    /// `klippy:connect` (`exclude_object.py:_handle_connect`).
    target: Mutex<Option<Arc<dyn MoveTarget>>>,
    /// The machine this object belongs to, for looking up `gcode_move` when
    /// the transform registers.
    printer: Weak<Printer>,
}

impl std::fmt::Debug for ExcludeObject {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.lock();
        f.debug_struct("ExcludeObject")
            .field("objects", &state.objects)
            .field("excluded_objects", &state.excluded_objects)
            .field("current_object", &state.current_object)
            .field("transform_registered", &state.transform_registered)
            .finish()
    }
}

impl ExcludeObject {
    /// Read the section — upstream reads no options
    /// (`exclude_object.py:12-36`), so the bare `[exclude_object]` in the
    /// corpus's `exclude_object.cfg:70` is accepted by the factory claiming
    /// it (`config/validate.rs:26-33`).
    pub fn new(_config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        Ok(Self {
            state: Mutex::new(State::default()),
            target: Mutex::new(None),
            printer: Arc::downgrade(printer),
        })
    }

    /// The state lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The transform chain below this one, copied out of its lock.
    fn target(&self) -> Option<Arc<dyn MoveTarget>> {
        self.target
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// Upstream's `_handle_connect`: the toolhead exists by `klippy:connect`
    /// (`exclude_object.py:59-60`).
    fn handle_connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
            *self.target.lock().unwrap_or_else(|p| p.into_inner()) =
                Some(Arc::new(ToolheadMove(toolhead)));
        }
    }

    /// Upstream's `_register_transform` (`exclude_object.py:38-57`): take
    /// `gcode_move`'s slot on the first exclusion.
    ///
    /// Needs the `Arc` because the transform registers itself. Upstream keeps
    /// the previous occupant; here the chain below is the toolhead captured
    /// at connect (module docs).
    fn register_transform(object: &Arc<ExcludeObject>) -> Result<(), CommandError> {
        if object.lock().transform_registered {
            return Ok(());
        }
        if object.target().is_none() {
            // No toolhead yet (`klippy:connect` has not run): upstream would
            // crash on `self.toolhead = None`; here nothing forwards, so the
            // registration waits for the next exclusion.
            return Ok(());
        }
        let printer = object.printer.upgrade().ok_or_else(|| {
            CommandError::new("The printer is gone; exclude_object cannot register its transform")
        })?;
        let gcode_move = printer
            .lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT)
            .ok_or_else(|| CommandError::new("gcode_move is not loaded"))?;
        gcode_move
            .set_move_transform(Arc::clone(object) as Arc<dyn MoveTarget>, true)
            .map_err(|error| CommandError::new(error.to_string()))?;
        // `_register_transform` (`exclude_object.py:38-57`): the offsets start
        // empty and the three tracked positions start at the toolhead's.
        let pos = object
            .target()
            .map(|target| target.position())
            .unwrap_or_default();
        let mut state = object.lock();
        state.transform_registered = true;
        state.motion = Some(ExcludedMotion {
            last_position: pos,
            last_position_extruded: pos,
            last_position_excluded: pos,
            initial_extrusion_moves: 5,
            ..Default::default()
        });
        Ok(())
    }

    /// Upstream's `_unregister_transform` (`exclude_object.py:62-73`): hand
    /// the slot back to the toolhead below this transform.
    fn unregister_transform(&self) {
        if !self.lock().transform_registered {
            return;
        }
        let (Some(target), Some(printer)) = (self.target(), self.printer.upgrade()) else {
            return;
        };
        if let Some(gcode_move) = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT) {
            // Restoring the toolhead rather than the previous occupant is the
            // chaining gap in the module docs; `force` keeps the write exact.
            let _ = gcode_move.set_move_transform(target, true);
            let mut state = self.lock();
            state.transform_registered = false;
            state.motion = None;
        }
    }

    /// Upstream's `_reset_file` (`exclude_object.py:81-83`): the state starts
    /// over and the transform lets go of the slot.
    fn reset_file(&self) {
        {
            let mut state = self.lock();
            state.objects.clear();
            state.excluded_objects.clear();
            state.current_object = None;
        }
        self.unregister_transform();
    }

    /// Upstream's `_exclude_object` (`exclude_object.py:279-283`): claim the
    /// transform, report, then remember the name (sorted).
    fn exclude_object(
        object: &Arc<ExcludeObject>,
        gcmd: &GcodeCommand,
        name: &str,
    ) -> Result<(), CommandError> {
        Self::register_transform(object)?;
        gcmd.respond_info(&format!("Excluding object {name}"));
        let mut state = object.lock();
        if !state
            .excluded_objects
            .iter()
            .any(|excluded| excluded == name)
        {
            state.excluded_objects.push(name.to_string());
            state.excluded_objects.sort();
        }
        Ok(())
    }

    /// Upstream's `_unexclude_object` (`exclude_object.py:285-290`).
    fn unexclude_object(gcmd: &GcodeCommand, state: &mut State, name: &str) {
        gcmd.respond_info(&format!("Unexcluding object {name}"));
        state.excluded_objects.retain(|excluded| excluded != name);
    }

    /// Register the four commands, capturing a handle to this object.
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        type Command = fn(&Arc<ExcludeObject>, &GcodeCommand) -> Result<(), CommandError>;
        // Each command reads its own set of `KEY=` words, so every entry
        // carries its own list rather than sharing one (the arrays' whole
        // point is that one registration call covers four different commands).
        const COMMANDS: &[(&str, Command, &str, &[&str])] = &[
            (
                "EXCLUDE_OBJECT_START",
                cmd_exclude_object_start,
                "Marks the beginning the current object as labeled",
                &["NAME"],
            ),
            (
                "EXCLUDE_OBJECT_END",
                cmd_exclude_object_end,
                "Marks the end the current object",
                &["NAME"],
            ),
            (
                "EXCLUDE_OBJECT",
                cmd_exclude_object,
                "Cancel moves inside a specified objects",
                &["RESET", "CURRENT", "NAME"],
            ),
            (
                "EXCLUDE_OBJECT_DEFINE",
                cmd_exclude_object_define,
                "Provides a summary of an object",
                &["RESET", "NAME", "JSON", "CENTER", "POLYGON"],
            ),
        ];
        for (name, command, desc, params) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                sync(move |gcmd| command(&object, gcmd))
            };
            gcode
                .register_command_with_params(name, handler, Some(desc), params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    /// The events upstream subscribes to that this port can wire
    /// (`exclude_object.py:16-19`): `virtual_sdcard:reset_file` has no sender
    /// while `virtual_sdcard` is unported (module docs).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.handle_connect()
            }),
        );
    }
}

impl PrinterObject for ExcludeObject {
    /// Upstream's `get_status`: `objects`, `excluded_objects`,
    /// `current_object` (`exclude_object.py:174-180`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let state = self.lock();
        json!({
            "objects": state.objects,
            "excluded_objects": state.excluded_objects,
            "current_object": state.current_object,
        })
    }
}

/// The toolhead as the transform's downstream, the way `gcode_move`'s own
/// `ToolheadTarget` wraps it (`gcode_move.rs:96-105`).
struct ToolheadMove(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadMove {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn position(&self) -> Coord {
        self.0.position().unwrap_or_default()
    }
}

/// `_ignore_move` (`exclude_object.py:145-153`): record the move without
/// forwarding it — the XY/Z drift lands in the offsets, the extrusion in
/// `offset[3]`, so the compensation on the way out subtracts it again.
fn ignore_move(motion: &mut ExcludedMotion, newpos: Coord) {
    for axis in 0..4 {
        if axis != E_AXIS {
            motion.offset[axis] = newpos.axis(axis) - motion.last_position_extruded.axis(axis);
        }
    }
    motion.offset[E_AXIS] += newpos.axis(E_AXIS) - motion.last_position.axis(E_AXIS);
    motion.last_position = newpos;
    motion.last_position_excluded = newpos;
    motion.max_position_excluded = motion.max_position_excluded.max(newpos.axis(E_AXIS));
}

/// `_normal_move` (`exclude_object.py:102-143`): track the move, settle the
/// boundary corrections, and return the position to forward — `newpos` minus
/// the standing offsets.
fn normal_move(motion: &mut ExcludedMotion, newpos: Coord) -> Coord {
    if motion.initial_extrusion_moves > 0
        && motion.last_position.axis(E_AXIS) != newpos.axis(E_AXIS)
    {
        motion.initial_extrusion_moves -= 1;
    }
    motion.last_position = newpos;
    motion.last_position_extruded = newpos;
    motion.max_position_extruded = motion.max_position_extruded.max(newpos.axis(E_AXIS));

    // The first XY move away from an excluded end settles the transient
    // catch-up and folds the pending `extruder_adj` into the E offset.
    if (motion.offset[0] != 0.0 || motion.offset[1] != 0.0)
        && (newpos.axis(0) != motion.last_position_excluded.axis(0)
            || newpos.axis(1) != motion.last_position_excluded.axis(1))
    {
        for axis in 0..4 {
            if axis != E_AXIS {
                motion.offset[axis] = 0.0;
            }
        }
        motion.offset[E_AXIS] += motion.extruder_adj;
        motion.extruder_adj = 0.0;
    }
    if motion.offset[2] != 0.0 && newpos.axis(2) != motion.last_position_excluded.axis(2) {
        motion.offset[2] = 0.0;
    }
    if motion.extruder_adj != 0.0
        && newpos.axis(E_AXIS) != motion.last_position_excluded.axis(E_AXIS)
    {
        motion.offset[E_AXIS] += motion.extruder_adj;
        motion.extruder_adj = 0.0;
    }

    let mut forwarded = newpos;
    for axis in 0..4 {
        forwarded.set_axis(axis, newpos.axis(axis) - motion.offset[axis]);
    }
    forwarded
}

impl MoveTarget for ExcludeObject {
    /// Upstream's `move` (`exclude_object.py:182-195`): a move inside an
    /// excluded object is dropped, anything else passes on — with the
    /// extrusion offsets applied, so the cancelled filament is never
    /// forwarded (`_normal_move`/`_ignore_move`, `:117-146`).
    ///
    /// Before the transform is armed (no exclusion registered it) the raw
    /// drop-or-forward of the first port stands; the corpus arms it through
    /// `register_transform` the moment anything is excluded.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let forward = {
            let mut state = self.lock();
            let current_excluded = state.current_object.as_deref().is_some_and(|current| {
                state
                    .excluded_objects
                    .iter()
                    .any(|excluded| excluded == current)
            });
            match state.motion.as_mut() {
                // Not armed: drop or forward raw, as the first port did.
                None => {
                    if current_excluded {
                        None
                    } else {
                        Some(position)
                    }
                }
                Some(motion) => {
                    // `_test_in_excluded_region`: the first five tracked
                    // extrusion moves after registration still pass
                    // (`initial_extrusion_moves`).
                    if current_excluded && motion.initial_extrusion_moves == 0 {
                        if !motion.in_excluded_region {
                            // `_move_into_excluded_region`.
                            motion.in_excluded_region = true;
                        }
                        ignore_move(motion, position);
                        None
                    } else if motion.in_excluded_region {
                        // `_move_from_excluded_region`: carry the retraction
                        // difference into the compensation, then move normally.
                        motion.in_excluded_region = false;
                        motion.extruder_adj = motion.max_position_excluded
                            - motion.last_position_excluded[E_AXIS]
                            - (motion.max_position_extruded
                                - motion.last_position_extruded[E_AXIS]);
                        Some(normal_move(motion, position))
                    } else {
                        Some(normal_move(motion, position))
                    }
                }
            }
        };
        let Some(forward) = forward else {
            return Ok(());
        };
        match self.target() {
            Some(target) => target.move_to(forward, speed),
            // No toolhead yet (`klippy:connect` has not run): no move runs
            // before ready, so there is nothing to forward to.
            None => Ok(()),
        }
    }

    /// Upstream's `get_position` (`exclude_object.py:95-100`): the toolhead's
    /// position plus the standing extrusion offset, so the gcode coordinate
    /// keeps counting filament the toolhead never extruded.
    fn position(&self) -> Coord {
        let Some(target) = self.target() else {
            return Coord::default();
        };
        let position = target.position();
        let mut state = self.lock();
        let Some(motion) = state.motion.as_mut() else {
            return position;
        };
        let mut gcode = position;
        for axis in 0..4 {
            gcode.set_axis(axis, position.axis(axis) + motion.offset[axis]);
        }
        motion.last_position = gcode;
        gcode
    }
}

/// The factory `section!` names (`exclude_object.py:303`).
///
/// `gcode_move::ensure` stands in for upstream's
/// `printer.load_object(config, 'gcode_move')` (`exclude_object.py:15`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(ExcludeObject::new(config, printer)?);
    object.register_commands(printer)?;
    object.register_handlers(printer);
    gcode_move::ensure(printer)?;
    Ok(object)
}

/// `EXCLUDE_OBJECT_START` (`exclude_object.py:199-204`): remember the name —
/// defined if it was not — and make it current.
fn cmd_exclude_object_start(
    object: &Arc<ExcludeObject>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let name = gcmd.get_str("NAME")?.to_uppercase();
    let mut state = object.lock();
    let defined = state
        .objects
        .iter()
        .any(|entry| entry.get("name").and_then(Value::as_str) == Some(name.as_str()));
    if !defined {
        state.objects.push(json!({ "name": name.clone() }));
        state.objects.sort_by_key(|entry| {
            entry
                .get("name")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string()
        });
    }
    state.current_object = Some(name);
    Ok(())
}

/// `EXCLUDE_OBJECT_END` (`exclude_object.py:207-218`): clear the current
/// object, reporting the two upstream mismatches but never failing.
fn cmd_exclude_object_end(
    object: &Arc<ExcludeObject>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let mut state = object.lock();
    if state.current_object.is_none() && state.transform_registered {
        gcmd.respond_info("EXCLUDE_OBJECT_END called, but no object is currently active");
        return Ok(());
    }
    let name = gcmd.get_str_default("NAME", "");
    if !name.is_empty() {
        let name = name.to_uppercase();
        if state.current_object.as_deref() != Some(name.as_str()) {
            gcmd.respond_info(&format!(
                "EXCLUDE_OBJECT_END NAME={name} does not match the current object NAME={}",
                state.current_object.as_deref().unwrap_or("None")
            ));
        }
    }
    state.current_object = None;
    Ok(())
}

/// `EXCLUDE_OBJECT` (`exclude_object.py:221-245`): reset, exclude by name or
/// by the current object, or list what is excluded.
fn cmd_exclude_object(
    object: &Arc<ExcludeObject>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let parameters = gcmd.get_command_parameters();
    let reset = parameters.contains_key("RESET");
    let current = parameters.contains_key("CURRENT");
    let name = gcmd.get_str_default("NAME", "").to_uppercase();

    if reset {
        if name.is_empty() {
            object.lock().excluded_objects.clear();
        } else {
            let mut state = object.lock();
            ExcludeObject::unexclude_object(gcmd, &mut state, &name);
        }
    } else if !name.is_empty() {
        let already = object
            .lock()
            .excluded_objects
            .iter()
            .any(|excluded| *excluded == name);
        if !already {
            ExcludeObject::exclude_object(object, gcmd, &name)?;
        }
    } else if current {
        let current = object
            .lock()
            .current_object
            .clone()
            .ok_or_else(|| CommandError::new("There is no current object to cancel"))?;
        ExcludeObject::exclude_object(object, gcmd, &current)?;
    } else {
        let state = object.lock();
        gcmd.respond_info(&format!(
            "Excluded objects: {}",
            state.excluded_objects.join(" ")
        ));
    }
    Ok(())
}

/// `EXCLUDE_OBJECT_DEFINE` (`exclude_object.py:248-273`): reset the file,
/// define an object (with its `CENTER`/`POLYGON` parsed as JSON), or list
/// what is known.
fn cmd_exclude_object_define(
    object: &Arc<ExcludeObject>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let parameters = gcmd.get_command_parameters().clone();
    let reset = parameters.contains_key("RESET");
    let name = gcmd.get_str_default("NAME", "").to_uppercase();

    if reset {
        object.reset_file();
        return Ok(());
    }

    if name.is_empty() {
        let state = object.lock();
        let list = if parameters.contains_key("JSON") {
            serde_json::to_string(&state.objects)
                .map_err(|error| CommandError::new(error.to_string()))?
        } else {
            state
                .objects
                .iter()
                .filter_map(|entry| entry.get("name").and_then(Value::as_str))
                .collect::<Vec<_>>()
                .join(" ")
        };
        gcmd.respond_info(&format!("Known objects: {list}"));
        return Ok(());
    }

    let mut entry = json!({ "name": name });
    // Upstream pops only `NAME`, `CENTER` and `POLYGON`; everything else —
    // `JSON` included — lands in the definition as a string.
    for (option, value) in &parameters {
        if matches!(option.as_str(), "NAME" | "CENTER" | "POLYGON") {
            continue;
        }
        entry[option.as_str()] = Value::String(value.clone());
    }
    if let Some(center) = parameters.get("CENTER") {
        // Upstream: `json.loads('[%s]' % center)` (`exclude_object.py:265`).
        let center: Value = serde_json::from_str(&format!("[{center}]"))
            .map_err(|error| CommandError::new(error.to_string()))?;
        entry["center"] = center;
    }
    if let Some(polygon) = parameters.get("POLYGON") {
        let polygon: Value =
            serde_json::from_str(polygon).map_err(|error| CommandError::new(error.to_string()))?;
        entry["polygon"] = polygon;
    }

    let mut state = object.lock();
    state.objects.push(entry);
    state.objects.sort_by_key(|entry| {
        entry
            .get("name")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string()
    });
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[exclude_object]` section with the given options, as the parser would
    /// build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("exclude_object", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A wrapper that records into `access`, as the loader builds it.
    fn wrapper<'a>(section: &'a ConfigSection, access: &Arc<AccessTracking>) -> ConfigWrapper<'a> {
        ConfigWrapper::new(section, Arc::clone(access))
    }

    /// The machine with just `[exclude_object]`, as the loader builds it:
    /// the factory claims the section, the commands register, and the object
    /// lands under the section's own name.
    fn machine() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<ExcludeObject>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text("[exclude_object]\n").expect("the section parses");
        printer.load_config(&config).expect("the section loads");
        // The dispatcher refuses scripts before ready (`gcode.rs` state
        // check); `fan.rs`'s machine lights the ready lamp the same way.
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher is registered");
        let object = printer
            .lookup_object_as::<ExcludeObject>("exclude_object")
            .expect("the object is registered");
        (printer, gcode, object)
    }

    /// A move target that records what it was asked and stands still.
    struct FakeTarget {
        moves: Mutex<Vec<(Coord, f64)>>,
        position: Mutex<Coord>,
    }

    impl FakeTarget {
        fn new() -> Self {
            Self {
                moves: Mutex::new(Vec::new()),
                position: Mutex::new(Coord::default()),
            }
        }

        fn moves(&self) -> Vec<(Coord, f64)> {
            self.moves.lock().unwrap().clone()
        }
    }

    impl MoveTarget for FakeTarget {
        fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
            *self.position.lock().unwrap() = position;
            self.moves.lock().unwrap().push((position, speed));
            Ok(())
        }

        fn position(&self) -> Coord {
            *self.position.lock().unwrap()
        }
    }

    /// Upstream reads no options (`exclude_object.py:12-36`): the bare
    /// `[exclude_object]` in the corpus's `exclude_object.cfg:70` loads, and
    /// the factory claiming the section is what `check_unused` needs
    /// (`config/validate.rs:26-33`).
    #[test]
    fn a_bare_section_loads_with_the_reset_state() {
        let sect = section(&[]);
        let access = AccessTracking::shared();
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let exclude = ExcludeObject::new(&wrapper(&sect, &access), &printer)
            .expect("the empty section loads");
        assert_eq!(
            exclude.get_status(0.0),
            json!({
                "objects": [],
                "excluded_objects": [],
                "current_object": serde_json::Value::Null,
            })
        );
    }

    /// The status keys are exactly upstream's three
    /// (`exclude_object.py:174-180`) — the M486 macro reads
    /// `printer.exclude_object.current_object`.
    #[test]
    fn get_status_exposes_exactly_the_upstream_keys() {
        let (_printer, _gcode, object) = machine();
        let status = object.get_status(0.0);
        let mut keys: Vec<&str> = status
            .as_object()
            .unwrap()
            .keys()
            .map(|key| key.as_str())
            .collect();
        keys.sort_unstable();
        assert_eq!(keys, ["current_object", "excluded_objects", "objects"]);
    }

    /// `START`/`END` track the current object with upstream's uppercasing,
    /// defining an unknown name on the way (`exclude_object.py:199-218`).
    #[test]
    fn start_tracks_and_end_clears_the_current_object() {
        let (_printer, gcode, object) = machine();

        gcode
            .run_script_sync("EXCLUDE_OBJECT_START NAME=alpha")
            .unwrap();
        assert_eq!(
            object.get_status(0.0),
            json!({
                "objects": [{"name": "ALPHA"}],
                "excluded_objects": [],
                "current_object": "ALPHA",
            })
        );

        gcode
            .run_script_sync("EXCLUDE_OBJECT_END NAME=alpha")
            .unwrap();
        assert!(object.get_status(0.0)["current_object"].is_null());
        assert_eq!(
            object.get_status(0.0)["objects"],
            json!([{"name": "ALPHA"}])
        );
    }

    /// `EXCLUDE_OBJECT` excludes by name, unexcludes with `RESET=NAME=…`,
    /// resets wholesale with `RESET=1`, and lists the rest
    /// (`exclude_object.py:221-245`).
    #[test]
    fn exclude_resets_by_name_and_wholesale() {
        let (_printer, gcode, object) = machine();

        gcode.run_script_sync("EXCLUDE_OBJECT NAME=beta").unwrap();
        gcode.run_script_sync("EXCLUDE_OBJECT NAME=alpha").unwrap();
        assert_eq!(
            object.get_status(0.0)["excluded_objects"],
            json!(["ALPHA", "BETA"]),
            "the list stays sorted"
        );

        gcode
            .run_script_sync("EXCLUDE_OBJECT RESET=1 NAME=alpha")
            .unwrap();
        assert_eq!(object.get_status(0.0)["excluded_objects"], json!(["BETA"]));

        gcode.run_script_sync("EXCLUDE_OBJECT RESET=1").unwrap();
        assert_eq!(object.get_status(0.0)["excluded_objects"], json!([]));
    }

    /// `EXCLUDE_OBJECT CURRENT=1` without a current object raises upstream's
    /// wording (`exclude_object.py:238-239`); with one it excludes it.
    #[test]
    fn exclude_current_without_one_is_upstreams_error() {
        let (_printer, gcode, object) = machine();

        let error = gcode
            .run_script_sync("EXCLUDE_OBJECT CURRENT=1")
            .unwrap_err();
        assert_eq!(error.to_string(), "There is no current object to cancel");

        gcode
            .run_script_sync("EXCLUDE_OBJECT_START NAME=gamma")
            .unwrap();
        gcode.run_script_sync("EXCLUDE_OBJECT CURRENT=1").unwrap();
        assert_eq!(object.get_status(0.0)["excluded_objects"], json!(["GAMMA"]));
    }

    /// `DEFINE` parses `CENTER` as the list upstream wraps in brackets and
    /// `POLYGON` as the list it is, keeping any other parameter as a string
    /// (`exclude_object.py:256-268`).
    #[test]
    fn define_parses_center_and_polygon_as_json() {
        let (_printer, gcode, object) = machine();

        gcode
            .run_script_sync("EXCLUDE_OBJECT_DEFINE NAME=thing CENTER=1.5,2.5 OTHER=hi")
            .unwrap();
        gcode
            .run_script_sync("EXCLUDE_OBJECT_DEFINE NAME=shape POLYGON=[[0,0],[1,0],[1,1]]")
            .unwrap();
        assert_eq!(
            object.get_status(0.0)["objects"],
            json!([
                {"name": "SHAPE", "polygon": [[0,0],[1,0],[1,1]]},
                {"name": "THING", "center": [1.5, 2.5], "OTHER": "hi"},
            ])
        );
    }

    /// `DEFINE RESET=1` is upstream's `_reset_file`: the whole state starts
    /// over (`exclude_object.py:81-83`).
    #[test]
    fn define_reset_clears_the_whole_state() {
        let (_printer, gcode, object) = machine();
        gcode
            .run_script_sync("EXCLUDE_OBJECT_DEFINE NAME=thing")
            .unwrap();
        gcode.run_script_sync("EXCLUDE_OBJECT NAME=thing").unwrap();
        gcode
            .run_script_sync("EXCLUDE_OBJECT_START NAME=thing")
            .unwrap();

        gcode
            .run_script_sync("EXCLUDE_OBJECT_DEFINE RESET=1")
            .unwrap();
        assert_eq!(
            object.get_status(0.0),
            json!({
                "objects": [],
                "excluded_objects": [],
                "current_object": serde_json::Value::Null,
            })
        );
    }

    /// The transform drops a move while the current object is excluded and
    /// forwards everything else to the chain below (`exclude_object.py:182-195`).
    #[test]
    fn a_move_in_an_excluded_region_is_dropped_and_the_rest_forwarded() {
        let (_printer, _gcode, object) = machine();
        let fake = Arc::new(FakeTarget::new());
        *object.target.lock().unwrap() = Some(Arc::clone(&fake) as Arc<dyn MoveTarget>);

        // Nothing excluded: every move passes on.
        object
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 50.0)
            .unwrap();
        assert_eq!(fake.moves().len(), 1);

        // Current object 1 is excluded — upstream's `_test_in_excluded_region`.
        {
            let mut state = object.lock();
            state.current_object = Some("1".to_string());
            state.excluded_objects = vec!["1".to_string()];
        }
        // The out-of-range `G0 X-11` the corpus issues inside object 1.
        object
            .move_to(Coord::new(-11.0, 0.0, 0.0, 0.0), 50.0)
            .unwrap();
        assert_eq!(fake.moves().len(), 1, "the excluded move was dropped");

        // Current object 2 is not excluded: the move passes.
        object.lock().current_object = Some("2".to_string());
        object
            .move_to(Coord::new(13.0, 0.0, 0.0, 0.0), 50.0)
            .unwrap();
        assert_eq!(fake.moves().len(), 2);
        assert_eq!(fake.moves()[1].0, Coord::new(13.0, 0.0, 0.0, 0.0));
    }

    /// The compensation math, against the corpus' own shape: a prime block
    /// printed *inside* the cancelled object is dropped, and the first move
    /// out subtracts the whole cancelled extrusion — the forwarded `ΔE` is
    /// zero, which is what keeps `G0 X0` after the prime block inside
    /// `max_extrude_cross_section` (`exclude_object.py:102-153`).
    #[test]
    fn excluded_extrusion_is_never_forwarded_after_leaving_the_region() {
        let (_printer, _gcode, object) = machine();
        let fake = Arc::new(FakeTarget::new());
        *object.target.lock().unwrap() = Some(Arc::clone(&fake) as Arc<dyn MoveTarget>);

        // Armed as `register_transform` arms it, at the toolhead's position.
        let start = Coord::new(11.0, 0.0, 0.0, 0.0);
        {
            let mut state = object.lock();
            state.current_object = Some("1".to_string());
            state.excluded_objects = vec!["1".to_string()];
            state.motion = Some(ExcludedMotion {
                last_position: start,
                last_position_extruded: start,
                last_position_excluded: start,
                initial_extrusion_moves: 0,
                ..Default::default()
            });
        }

        // Two prime moves inside the cancelled object: dropped, but tracked.
        object
            .move_to(Coord::new(140.0, 0.0, 0.0, 0.5), 50.0)
            .unwrap();
        object
            .move_to(Coord::new(160.0, 0.0, 0.0, 1.0), 50.0)
            .unwrap();
        assert!(fake.moves().is_empty(), "excluded moves are dropped");

        // Leave the region: the forwarded E carries no cancelled filament.
        object.lock().current_object = Some("2".to_string());
        object
            .move_to(Coord::new(0.0, 0.0, 0.0, 1.0), 50.0)
            .unwrap();
        let moves = fake.moves();
        assert_eq!(moves.len(), 1, "the first move out is forwarded");
        assert_eq!(
            moves[0].0.axis(E_AXIS),
            0.0,
            "ΔE=0: the 1.0mm cancelled extrusion is compensated out"
        );
        assert_eq!(moves[0].0.axis(0), 0.0, "the XY catch-up settles too");

        // The offset stands for the rest of the run: only new filament goes.
        object
            .move_to(Coord::new(10.0, 0.0, 0.0, 1.5), 50.0)
            .unwrap();
        assert_eq!(fake.moves()[1].0.axis(E_AXIS), 0.5, "new extrusion only");

        // `get_position` reports the gcode side: toolhead + standing offset.
        let gcode = object.position();
        assert_eq!(
            gcode.axis(E_AXIS),
            fake.moves()[1].0.axis(E_AXIS) + 1.0,
            "the gcode coordinate keeps counting the cancelled filament"
        );
    }

    /// The zero-exclusion path: an armed transform that never cancels
    /// anything forwards every position byte-for-byte (`offset` stays empty),
    /// and the initial five tracked extrusions pass like upstream's
    /// `initial_extrusion_moves` window.
    #[test]
    fn without_an_exclusion_positions_forward_untouched() {
        let (_printer, _gcode, object) = machine();
        let fake = Arc::new(FakeTarget::new());
        *object.target.lock().unwrap() = Some(Arc::clone(&fake) as Arc<dyn MoveTarget>);
        let start = Coord::new(0.0, 0.0, 0.0, 0.0);
        {
            let mut state = object.lock();
            state.motion = Some(ExcludedMotion {
                last_position: start,
                last_position_extruded: start,
                last_position_excluded: start,
                initial_extrusion_moves: 5,
                ..Default::default()
            });
        }

        for (index, position) in [
            Coord::new(140.0, 0.0, 0.0, 0.5),
            Coord::new(160.0, 0.0, 0.0, 1.0),
            Coord::new(140.0, 0.0, 0.0, 1.5),
            Coord::new(10.0, 0.0, 0.0, 1.5),
            Coord::new(0.0, 0.0, 0.0, 0.0),
        ]
        .into_iter()
        .enumerate()
        {
            object.move_to(position, 50.0).unwrap();
            assert_eq!(
                fake.moves()[index].0,
                position,
                "move {index} forwarded untouched"
            );
            assert_eq!(object.position(), position, "get_position tracks it");
        }
    }

    /// The four commands register with upstream's help text — a duplicate or
    /// invalid name would already have failed the load
    /// (`exclude_object.py:197-198,206,220,247`).
    #[test]
    fn the_four_commands_are_registered_with_upstreams_help_text() {
        let (_printer, gcode, _object) = machine();
        let help = gcode.command_help();
        assert_eq!(
            help.get("EXCLUDE_OBJECT_START").map(String::as_str),
            Some("Marks the beginning the current object as labeled")
        );
        assert_eq!(
            help.get("EXCLUDE_OBJECT_END").map(String::as_str),
            Some("Marks the end the current object")
        );
        assert_eq!(
            help.get("EXCLUDE_OBJECT").map(String::as_str),
            Some("Cancel moves inside a specified objects")
        );
        assert_eq!(
            help.get("EXCLUDE_OBJECT_DEFINE").map(String::as_str),
            Some("Provides a summary of an object")
        );
    }
}
