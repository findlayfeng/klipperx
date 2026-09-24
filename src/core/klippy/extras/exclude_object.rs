//! `[exclude_object]` — define, track and exclude print objects
//! (upstream `klippy/extras/exclude_object.py`).
//!
//! The section reads **no options** (`exclude_object.py:16-46`): what it does
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
//! "Move out of range" (`exclude_object.py:103-172`, the transform).
//!
//! # Gaps this port does not close yet (H4)
//!
//! - **The corpus cannot turn green on this file alone.** `exclude_object.test`
//!   drives everything through the `M486` macro body, and `gcode_macro` does
//!   not render bodies yet (`gcode_macro.rs`, module docs): the `EXCLUDE_*`
//!   lines never run, so nothing is excluded and the out-of-range moves reach
//!   the toolhead. The section, commands and transform landed here are the
//!   other half; template rendering is the remaining gate.
//! - **Move bookkeeping is simplified.** Upstream tracks extrusion offsets,
//!   `initial_extrusion_moves` priming and `extruder_adj` compensation across
//!   the region boundaries (`exclude_object.py:60-172`); here a move is
//!   dropped while the current object is excluded and forwarded otherwise,
//!   with no offset compensation on the way out.
//! - **The transform does not chain over other transforms.** Upstream keeps
//!   the previous slot occupant (`next_transform`, `exclude_object.py:47-59`)
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
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

section!("exclude_object", order = 30, load = load_config);

/// The object state upstream keeps in `_reset_state` (`exclude_object.py:70-75`).
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
    /// (`exclude_object.py:16-46`), so the bare `[exclude_object]` in the
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
    /// (`exclude_object.py:30-31`).
    fn handle_connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
            *self.target.lock().unwrap_or_else(|p| p.into_inner()) =
                Some(Arc::new(ToolheadMove(toolhead)));
        }
    }

    /// Upstream's `_register_transform` (`exclude_object.py:47-77`): take
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
        object.lock().transform_registered = true;
        Ok(())
    }

    /// Upstream's `_unregister_transform` (`exclude_object.py:79-91`): hand
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
            self.lock().transform_registered = false;
        }
    }

    /// Upstream's `_reset_file` (`exclude_object.py:70-77`): the state starts
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

    /// Upstream's `_exclude_object` (`exclude_object.py:219-224`): claim the
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

    /// Upstream's `_unexclude_object` (`exclude_object.py:226-231`).
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
        const COMMANDS: &[(&str, Command, &str)] = &[
            (
                "EXCLUDE_OBJECT_START",
                cmd_exclude_object_start,
                "Marks the beginning the current object as labeled",
            ),
            (
                "EXCLUDE_OBJECT_END",
                cmd_exclude_object_end,
                "Marks the end the current object",
            ),
            (
                "EXCLUDE_OBJECT",
                cmd_exclude_object,
                "Cancel moves inside a specified objects",
            ),
            (
                "EXCLUDE_OBJECT_DEFINE",
                cmd_exclude_object_define,
                "Provides a summary of an object",
            ),
        ];
        for (name, command, desc) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                sync(move |gcmd| command(&object, gcmd))
            };
            gcode
                .register_command(name, handler, Some(desc), false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    /// The events upstream subscribes to that this port can wire
    /// (`exclude_object.py:29-31`): `virtual_sdcard:reset_file` has no sender
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
    /// `current_object` (`exclude_object.py:174-181`).
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
/// `ToolheadTarget` wraps it (`gcode_move.rs:92-101`).
struct ToolheadMove(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadMove {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn position(&self) -> Coord {
        self.0.position().unwrap_or_default()
    }
}

impl MoveTarget for ExcludeObject {
    /// Upstream's `move` (`exclude_object.py:161-172`): a move inside an
    /// excluded object is dropped, anything else passes on. The region
    /// bookkeeping is simplified — see the module docs.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let excluded_now = {
            let state = self.lock();
            state.current_object.as_deref().is_some_and(|current| {
                state
                    .excluded_objects
                    .iter()
                    .any(|excluded| excluded == current)
            })
        };
        if excluded_now {
            return Ok(());
        }
        match self.target() {
            Some(target) => target.move_to(position, speed),
            // No toolhead yet (`klippy:connect` has not run): no move runs
            // before ready, so there is nothing to forward to.
            None => Ok(()),
        }
    }

    /// Upstream's `get_position` (`exclude_object.py:88-93`), without the
    /// extrusion offsets the module docs list as a gap.
    fn position(&self) -> Coord {
        self.target()
            .map(|target| target.position())
            .unwrap_or_default()
    }
}

/// The factory `section!` names (`exclude_object.py:303`).
///
/// `gcode_move::ensure` stands in for upstream's
/// `printer.load_object(config, 'gcode_move')` (`exclude_object.py:18`).
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

/// `EXCLUDE_OBJECT_START` (`exclude_object.py:190-198`): remember the name —
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

/// `EXCLUDE_OBJECT_END` (`exclude_object.py:200-213`): clear the current
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

/// `EXCLUDE_OBJECT` (`exclude_object.py:215-238`): reset, exclude by name or
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

/// `EXCLUDE_OBJECT_DEFINE` (`exclude_object.py:240-267`): reset the file,
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
        // Upstream: `json.loads('[%s]' % center)` (`exclude_object.py:258-260`).
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

    /// Upstream reads no options (`exclude_object.py:16-46`): the bare
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
    /// (`exclude_object.py:174-181`) — the M486 macro reads
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
    /// defining an unknown name on the way (`exclude_object.py:190-213`).
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
    /// (`exclude_object.py:215-238`).
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
    /// wording (`exclude_object.py:229-230`); with one it excludes it.
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
    /// (`exclude_object.py:250-264`).
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
    /// over (`exclude_object.py:244-247`).
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
    /// forwards everything else to the chain below (`exclude_object.py:161-172`).
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

    /// The four commands register with upstream's help text — a duplicate or
    /// invalid name would already have failed the load
    /// (`exclude_object.py:184-189,191,202,216,242`).
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
