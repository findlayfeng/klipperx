//! `[dual_carriage]` — the IDEX second carriage, and its three commands
//! (upstream `klippy/kinematics/idex_modes.py`).
//!
//! Upstream the section is claimed by the kinematics: `CartKinematics` sees
//! `config.has_section('dual_carriage')`, reads `axis` and `safe_distance`,
//! looks the section up as a multi-stepper rail, and builds the
//! `DualCarriages` module with it (`kinematics/cartesian.py:24-34`); the
//! module registers itself as the `dual_carriage` object and the commands
//! `SET_DUAL_CARRIAGE`, `SAVE_DUAL_CARRIAGE_STATE`,
//! `RESTORE_DUAL_CARRIAGE_STATE` (`idex_modes.py:46-59`).
//!
//! Here the split follows the loader instead: the **section factory** (this
//! module, loaded `late` like the kinematics that consumes it) parses `axis` /
//! `safe_distance` and builds the second carriage's
//! [`PrinterStepper`] — which reads the section's stepper options exactly as
//! upstream's `LookupMultiRail(dc_config)` does — then registers the object
//! and the three commands. The **claim** is one call from the toolhead:
//! [`claim`] hands the cartesian kinematics' primary rail for the carriage
//! axis to the module, the seam upstream wires in `cartesian.py:31-34`.
//!
//! The stepper is built but **not driven**: it never joins the toolhead's
//! rails, so no steps are generated for it. What the three commands *do*
//! carry across is the **gcode coordinate**: switching or restoring parks the
//! departing carriage's axis coordinate and re-anchors the toolhead on the
//! arriving carriage's frame, the way upstream's `toggle_active_dc_rail`
//! calls `toolhead.set_position(newpos)` (`idex_modes.py:101-114`) — the
//! `toolhead:set_position` event re-anchors `gcode_move` with it, so the next
//! move lands in the new carriage's frame and its range. The upstream
//! behaviour still missing is listed below; the hybrid dual carriage unit
//! builds on this seam.
//!
//! The generic-cartesian family reaches the same module from the other side:
//! upstream builds `idex_modes.DualCarriages` inside
//! `GenericCartesianKinematics.__init__` when the config carries
//! `[dual_carriage <name>]` sections (`generic_cartesian.py:137-146`), so
//! there the object and the three commands come from the kinematics, not from
//! a section. Here that constructor is [`register_generic`], called by
//! [`build`](crate::core::klippy::extras::carriage::build) with the primary
//! and dual carriages it collected: the same command state, holding every
//! carriage of the machine instead of the section's two.
//!
//! # Gaps this port does not close yet
//!
//! * **No carriage switching on the motion layer**: the active rail's trapq
//!   swap, the scale/offset transform and `update_limits`
//!   (`idex_modes.py:224-231`, `DualCarriagesRail.activate/inactivate`) are
//!   not implemented — the coordinate handover teleports the toolhead onto
//!   the new frame, but the steps still go to the primary rail, and the
//!   active carriage's **range** stays the primary rail's (identical in the
//!   corpus, where both carriages ride `[0, 200]`).
//! * **`COPY` / `MIRROR` / `INACTIVE` modes are validated, not applied**
//!   (`idex_modes.py:15-17`); upstream's homing/`PRIMARY`-first checks are
//!   not enforced either.
//! * **Saved states store the active index and the axis frames**: carriage
//!   modes (`PRIMARY`/`COPY`/`MIRROR`) are not modelled, the restore
//!   re-anchors the coordinate instead of physically moving the carriages,
//!   and `MOVE_SPEED` is read but unused (`idex_modes.py:285-310`).
//! * **The second carriage's endstop is not in `query_endstops`**, and
//!   `STEPPER_BUZZ STEPPER=dual_carriage` is not registered (both are silent
//!   unknown-command answers, so the corpus does not care).
//! * **A config with two extruders ([`dual_carriage.cfg`](crate and its T0/T1
//!   macros) runs with the second extruder **out of the motion path**: this
//!   port's `Move.axes_d` has four slots (X/Y/Z/E), so an extra axis past the
//!   first is skipped by guarded no-ops in the planner instead of receiving
//!   its own slot — a defensive downgrade of the pre-existing multi-extruder
//!   port gap (it used to panic on the first `G1`). Full dynamic multi-extruder
//!   positions (`Coord`/`axes_d` growing per axis, as upstream's
//!   `gcode_move.py:118-131` does) is a separate unit and a prerequisite of
//!   upstream's `extruders.test`.
//! * **The generic-cartesian module tracks one active carriage**, the last
//!   `SET_DUAL_CARRIAGE` picked, while upstream keeps a mode per rail
//!   (`idex_modes.py:37-45,133-139`): selecting the dual carriage of a
//!   second axis leaves the first axis' carriage where it was.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::stepper::{axis_index, PrinterStepper, Rail};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::motion::Axis;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Both forms load in the generic walk, in the order upstream's
// `_load_kinematics` walks them (`generic_cartesian.py:173-212`): the bare
// `[dual_carriage]` is the cartesian IDEX module below, and the prefix form
// (`[dual_carriage <name>]`) belongs to `kinematics: generic_cartesian`, which
// `extras::carriage` builds. The prefix form has to load before the
// `[stepper <name>]` sections that name it, hence the order here.
section!(
    "dual_carriage",
    order = 53,
    phase = generic,
    load = load_config,
    prefix = crate::core::klippy::extras::carriage::load_dual_carriage
);

/// The name the object (and the section) go by upstream
/// (`idex_modes.py:46 add_object('dual_carriage', self)`).
pub const DUAL_CARRIAGE_OBJECT: &str = "dual_carriage";

/// The toolhead's object name (`[printer]` registers it), where
/// `toggle_active_dc_rail` re-anchors the coordinate
/// (`idex_modes.py:102`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The `MODE` values `SET_DUAL_CARRIAGE` accepts (`idex_modes.py:16`).
const VALID_MODES: [&str; 4] = ["INACTIVE", "PRIMARY", "COPY", "MIRROR"];

/// What the three commands share: the carriage index that is active, each
/// carriage's axis frame, and the states `SAVE_DUAL_CARRIAGE_STATE` wrote.
struct Shared {
    /// The active carriage: for the bare `[dual_carriage]` 0 is the primary
    /// rail, 1 the second carriage; for `kinematics: generic_cartesian` the
    /// index of the carriage `SET_DUAL_CARRIAGE` last picked.
    active: usize,
    /// The carriage-axis coordinate each carriage's frame holds. A frame is
    /// recorded when its carriage is left and carried to when it is
    /// re-entered — the coordinate half of upstream's
    /// `toggle_active_dc_rail` (`idex_modes.py:101-114`), where upstream
    /// instead reads it off the scale/offset transform of a second rail that
    /// this port does not drive (module docs) — and set to the carriage's own
    /// `position_endstop` when its axis homes ([`Shared::homed`]).
    axis_position: Vec<f64>,
    /// The names `CARRIAGE=` takes, in carriage order — upstream's `dc_rails`
    /// keys, each carriage's `rail.get_name(short=True)`
    /// (`idex_modes.py:37-39`). This module's own name is known when the
    /// section loads; the primary rail's short name is filled in by [`claim`],
    /// which is when the cartesian kinematics hands the rail over. A generic
    /// cartesian machine knows all of them when the kinematics builds the
    /// module ([`register_generic`]).
    names: Vec<Option<String>>,
    /// The axis each carriage rides on (`self.axes` upstream), so a switch
    /// re-anchors exactly the arriving carriage's axis.
    axes: Vec<usize>,
    /// Where each carriage sits once its axis homes: its own
    /// `position_endstop`. Upstream homes every carriage of the axis there
    /// (`DualCarriages.home`, `idex_modes.py:116-131`, which toggles each
    /// rail and homes it) and the frames follow the physical carriages;
    /// [`Shared::homed`] is this port's copy of that bookkeeping, and the
    /// bare `[dual_carriage]` section never calls it (its entries stay `0.0`,
    /// like its frames before any homing).
    endstops: Vec<f64>,
    /// `SAVE_DUAL_CARRIAGE_STATE NAME=…` states: the active index and the
    /// axis frames (`idex_modes.py:285-293`; modes are not modelled — module
    /// docs).
    saved: HashMap<String, SavedState>,
}

impl Shared {
    /// The bare `[dual_carriage]` section's two carriages on one `axis`: index
    /// 0 is the primary rail the kinematics claims (name filled by [`claim`]),
    /// index 1 this section's own carriage `own_name`.
    fn for_section(axis: usize, own_name: String) -> Self {
        Self {
            active: 0,
            axis_position: vec![0.0; 2],
            names: vec![None, Some(own_name)],
            axes: vec![axis; 2],
            endstops: vec![0.0; 2],
            saved: HashMap::new(),
        }
    }

    /// A `kinematics: generic_cartesian` machine's carriages, in upstream's
    /// `dc_rails` order (`generic_cartesian.py:137-142`): the primary carriage
    /// of every dual axis, then the dual carriages themselves.
    fn for_carriages(carriages: &[GenericCarriage]) -> Self {
        Self {
            active: 0,
            axis_position: vec![0.0; carriages.len()],
            names: carriages
                .iter()
                .map(|carriage| Some(carriage.name.clone()))
                .collect(),
            axes: carriages
                .iter()
                .map(|carriage| axis_index(carriage.axis))
                .collect(),
            endstops: carriages
                .iter()
                .map(|carriage| carriage.position_endstop)
                .collect(),
            saved: HashMap::new(),
        }
    }

    /// An axis finished homing (`HomingHomeRailsEnd`): every carriage of that
    /// axis now sits at its own `position_endstop`, as upstream's
    /// `DualCarriages.home` leaves them (`idex_modes.py:116-131`) — without
    /// this a dual carriage's frame stays at `0.0` forever, and the first
    /// switch onto it teleports the toolhead to a coordinate it never earned
    /// (`Move out of range` on the corpus' first `G1 X-10`).
    fn homed(&mut self, axes: &[usize]) {
        for (index, axis) in self.axes.iter().enumerate() {
            if axes.contains(axis) {
                self.axis_position[index] = self.endstops[index];
            }
        }
    }
}

/// One carriage of a generic-cartesian dual axis, as [`register_generic`]
/// takes it: the name `SET_DUAL_CARRIAGE CARRIAGE=` matches, the axis it
/// rides, and the coordinate it sits at once that axis homes (its section's
/// `position_endstop`).
pub struct GenericCarriage {
    /// The carriage's short name (`carriage_u`).
    pub name: String,
    /// The axis it rides on.
    pub axis: Axis,
    /// Its `position_endstop`, the coordinate homing leaves it at.
    pub position_endstop: f64,
}

/// One `SAVE_DUAL_CARRIAGE_STATE` snapshot.
#[derive(Debug, Clone, PartialEq)]
struct SavedState {
    /// The carriage that was active.
    active: usize,
    /// Every carriage's axis frame at the save.
    axis_position: Vec<f64>,
}

/// The `dual_carriage` object: the second carriage's section and state.
pub struct DualCarriageModule {
    /// The axis the second carriage rides on (`axis: x` → X, `y` → Y).
    axis: Axis,
    /// `safe_distance` between the carriages, for the units that will move
    /// them (`cartesian.py:33-34` reads it for upstream's constructor).
    #[allow(dead_code)]
    safe_distance: Option<f64>,
    /// The second carriage's motor, built from this section the way upstream's
    /// `LookupMultiRail(dc_config)` is (`cartesian.py:30`). Held so the
    /// firmware resource stays alive; not driven yet (module docs).
    #[allow(dead_code)]
    stepper: PrinterStepper,
    /// The primary rail for `axis`, once the cartesian kinematics claims it
    /// (`cartesian.py:31-34`).
    primary_rail: Mutex<Option<Arc<Rail>>>,
    /// Command-visible state, shared with the three handlers.
    shared: Arc<Mutex<Shared>>,
}

impl DualCarriageModule {
    fn shared_lock(&self) -> MutexGuard<'_, Shared> {
        self.shared
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The axis the second carriage rides on.
    pub fn axis(&self) -> Axis {
        self.axis
    }

    /// The active carriage index: 0 until a `SET_DUAL_CARRIAGE` says otherwise.
    pub fn active_carriage(&self) -> usize {
        self.shared_lock().active
    }

    /// The carriages' axis frames, in carriage order — what the handover
    /// carries across switches and restores.
    pub fn axis_frames(&self) -> Vec<f64> {
        self.shared_lock().axis_position.clone()
    }

    /// The primary rail the cartesian kinematics claimed, by name — `None`
    /// until [`claim`] ran (or for a non-cartesian kinematics).
    pub fn claimed_primary_rail(&self) -> Option<String> {
        self.primary_rail
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_ref()
            .map(|rail| rail.name().to_string())
    }
}

impl PrinterObject for DualCarriageModule {
    /// The active carriage and whether the kinematics claimed the module.
    ///
    /// Upstream reports each carriage's mode instead
    /// (`idex_modes.py:133-139`); modes are not modelled yet (module docs).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "active_carriage": self.shared_lock().active,
            "claimed": self.claimed_primary_rail().is_some(),
        })
    }
}

impl std::fmt::Debug for DualCarriageModule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DualCarriageModule")
            .field("axis", &self.axis)
            .finish_non_exhaustive()
    }
}

/// Wire the cartesian kinematics' rails into the module
/// (`kinematics/cartesian.py:24-34`).
///
/// The toolhead calls this once when `[printer] kinematics` is `cartesian`;
/// without a `[dual_carriage]` section there is no module and this is a
/// no-op, as upstream's `has_section` check is.
pub fn claim(rails: &[Arc<Rail>], printer: &Arc<Printer>) {
    let Some(module) = printer.lookup_object_as::<DualCarriageModule>(DUAL_CARRIAGE_OBJECT) else {
        return;
    };
    let Some(rail) = rails.get(axis_index(module.axis)) else {
        return;
    };
    *module
        .primary_rail
        .lock()
        .unwrap_or_else(|poison| poison.into_inner()) = Some(Arc::clone(rail));
    // The primary carriage's name is the rail's short name
    // (`rail.get_name(short=True)`, `stepper.py:388-393`), the key upstream's
    // `dc_rails` uses for `CARRIAGE=` (`idex_modes.py:37-39`).
    module.shared_lock().names[0] = Some(short_rail_name(rail.name()).to_string());
}

/// A rail's short name (`GenericPrinterRail.get_name(short=True)`,
/// `stepper.py:388-393`): a `stepper_x` rail is `x`, `stepper_z1` is `z1`,
/// and anything else is its last whitespace-separated word.
fn short_rail_name(name: &str) -> &str {
    if let Some(rest) = name.strip_prefix("stepper") {
        // `get_name(short=True)` skips the `stepper` prefix and the symbol
        // after it.
        return rest.strip_prefix('_').unwrap_or(rest);
    }
    name.rsplit(' ').next().unwrap_or(name)
}

/// The factory `section!` names for the bare `[dual_carriage]` section: the
/// module upstream builds inside `CartKinematics.__init__`
/// (`cartesian.py:24-34` + `idex_modes.py:46-59`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let identifier = config.identifier();
    // `dc_config.getchoice('axis', ['x', 'y'])` (`cartesian.py:27`).
    let axis = match config.get("axis", None)?.trim().to_lowercase().as_str() {
        "x" => Axis::X,
        "y" => Axis::Y,
        _ => {
            return Err(ConfigError::new(format!(
                "Option 'axis' in section '{identifier}' must be one of 'x', 'y'"
            )))
        }
    };
    // `dc_config.getfloat('safe_distance', None, minval=0.)`
    // (`cartesian.py:33-34`): absent is allowed, a written value is bounded
    // (the same present-gate `pwm_tool` applies to its optional figure).
    let safe_distance = if config.section().has("safe_distance") {
        Some(config.get_float_bounded("safe_distance", None, Some(0.), None, None, None)?)
    } else {
        None
    };
    // The section doubles as a stepper section: upstream reads it through
    // `stepper.LookupMultiRail(dc_config)` (`cartesian.py:30`), which is
    // exactly what building the primary stepper here reads.
    let stepper = PrinterStepper::new(config, printer, axis, true)?;

    let shared = Arc::new(Mutex::new(Shared::for_section(
        axis_index(axis),
        // This module's own carriage name is the section's short name
        // (`rail.get_name(short=True)`, `stepper.py:388-393`), the key upstream's
        // `dc_rails` uses for `CARRIAGE=` (`idex_modes.py:37-39`); the bare
        // `[dual_carriage]` section's short name is its identifier.
        identifier.clone(),
    )));
    register_commands(&shared, printer)?;

    Ok(Arc::new(DualCarriageModule {
        axis,
        safe_distance,
        stepper,
        primary_rail: Mutex::new(None),
        shared,
    }))
}

/// The `dual_carriage` object of `kinematics: generic_cartesian`.
///
/// Upstream builds `idex_modes.DualCarriages` inside
/// `GenericCartesianKinematics.__init__` whenever the config carries
/// `[dual_carriage <name>]` sections (`generic_cartesian.py:137-146`), and
/// that constructor registers the object and the three commands — for this
/// family there is no bare `[dual_carriage]` section to hang them on. This is
/// that module, built once by [`register_generic`] with every carriage pair
/// of the machine.
pub struct GenericDualCarriages {
    /// The command state, shared with the three handlers.
    shared: Arc<Mutex<Shared>>,
}

impl GenericDualCarriages {
    /// The active carriage's index into the registered carriages: the last
    /// `SET_DUAL_CARRIAGE` pick (module docs — modes are not modelled).
    pub fn active_carriage(&self) -> usize {
        self.shared
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .active
    }

    /// The carriage names `CARRIAGE=` accepts, in upstream's `dc_rails` order
    /// (`generic_cartesian.py:138-142`).
    pub fn carriage_names(&self) -> Vec<String> {
        self.shared
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .names
            .iter()
            .flatten()
            .cloned()
            .collect()
    }
}

impl PrinterObject for GenericDualCarriages {
    /// The active carriage and the carriages themselves; upstream reports a
    /// mode per carriage instead (`idex_modes.py:133-139`), modes are not
    /// modelled yet (module docs).
    fn get_status(&self, _eventtime: f64) -> Value {
        let state = self
            .shared
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        json!({
            "active_carriage": state.active,
            "carriages": state.names.iter().flatten().cloned().collect::<Vec<_>>(),
        })
    }
}

/// Build the generic-cartesian `dual_carriage` module, register its three
/// commands, and let the frames follow homing (`generic_cartesian.py:137-146`,
/// `idex_modes.py:46-59,116-131`).
///
/// `carriages` is every carriage upstream's `dc_rails` carries, in its order:
/// the primary carriage of each dual axis, then the dual carriages. Called by
/// [`build`](crate::core::klippy::extras::carriage::build) once, when the
/// kinematics sees at least one `[dual_carriage <name>]` section.
///
/// # Errors
/// When the `dual_carriage` object or one of the three commands is already
/// registered — a config that also carries a bare `[dual_carriage]`.
pub fn register_generic(
    printer: &Arc<Printer>,
    carriages: &[GenericCarriage],
) -> Result<(), ConfigError> {
    let shared = Arc::new(Mutex::new(Shared::for_carriages(carriages)));
    printer.add_object(
        DUAL_CARRIAGE_OBJECT,
        Arc::new(GenericDualCarriages {
            shared: Arc::clone(&shared),
        }),
    )?;
    register_commands(&shared, printer)?;

    // Upstream's `DualCarriages.home` (`idex_modes.py:116-131`) homes every
    // carriage of the axis to its own endstop and the frames follow the
    // physical carriages there; this port homes the axis through the
    // kinematics instead, so the same refresh hangs off the homing event.
    printer.register_event_handler(
        KlippyEvent::HomingHomeRailsEnd { axes: Vec::new() },
        Box::new({
            let shared = Arc::clone(&shared);
            move |event| {
                if let KlippyEvent::HomingHomeRailsEnd { axes } = event {
                    shared
                        .lock()
                        .unwrap_or_else(|poison| poison.into_inner())
                        .homed(axes);
                }
            }
        }),
    );
    Ok(())
}

/// Register `SET_DUAL_CARRIAGE` / `SAVE_DUAL_CARRIAGE_STATE` /
/// `RESTORE_DUAL_CARRIAGE_STATE` on `shared` (`idex_modes.py:46-59`): the
/// same three handlers whether the module came from the bare section or from
/// the generic-cartesian kinematics.
fn register_commands(
    shared: &Arc<Mutex<Shared>>,
    printer: &Arc<Printer>,
) -> Result<(), ConfigError> {
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` before any section");

    // `SAVE_DUAL_CARRIAGE_STATE`: snapshot the frames — synchronous, it only
    // reads the toolhead's position if there is one.
    {
        let shared = Arc::clone(shared);
        let weak = Arc::downgrade(printer);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let printer = weak.upgrade();
            cmd_save_dual_carriage_state(&shared, gcmd, printer.as_ref())
        });
        gcode
            .register_command(
                "SAVE_DUAL_CARRIAGE_STATE",
                handler,
                Some("Save dual carriages modes and positions"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    // `SET_DUAL_CARRIAGE` / `RESTORE_DUAL_CARRIAGE_STATE` re-anchor the
    // toolhead, so they are async handlers — upstream's
    // `toolhead.set_position` (`idex_modes.py:114,348`) has the same job.
    for (name, desc, restoring) in [
        (
            "SET_DUAL_CARRIAGE",
            "Configure the dual carriages mode",
            false,
        ),
        (
            "RESTORE_DUAL_CARRIAGE_STATE",
            "Restore dual carriages modes and positions",
            true,
        ),
    ] {
        let shared = Arc::clone(shared);
        let weak = Arc::downgrade(printer);
        let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
            let shared = Arc::clone(&shared);
            let weak = weak.clone();
            Box::pin(async move {
                let printer = weak
                    .upgrade()
                    .ok_or_else(|| CommandError::new("printer is gone"))?;
                let plan = if restoring {
                    Plan::Restore(cmd_restore_dual_carriage_state(&shared, gcmd)?)
                } else {
                    Plan::Switch(select_carriage(&shared, gcmd)?)
                };
                apply(&shared, &printer, plan).await
            })
        });
        gcode
            .register_command(name, handler, Some(desc), false)
            .map_err(ConfigError::new)?;
    }
    Ok(())
}

/// What one command asks the frames to do: a switch carries the coordinates
/// across, a restore applies a snapshot wholesale.
enum Plan {
    /// `SET_DUAL_CARRIAGE CARRIAGE=<name|index>`.
    Switch(usize),
    /// `RESTORE_DUAL_CARRIAGE_STATE`.
    Restore(SavedState),
}

/// The arriving carriage's axis frame — the bookkeeping half of upstream's
/// `toggle_active_dc_rail` (`idex_modes.py:101-114`): record the departing
/// frame from `current` along the departing carriage's own axis (when there
/// is a position to record), select the arriving carriage, and hand back the
/// coordinate the toolhead should be re-anchored on. A restore overwrites
/// all frames first (`idex_modes.py:303-348`); without a position only the
/// active index moves, which is the whole of what the commands did before
/// this seam existed.
fn carry_frame(state: &mut Shared, plan: &Plan, current: Option<Coord>) -> f64 {
    match plan {
        Plan::Switch(to) => {
            if let Some(current) = current {
                let departing = state.active;
                let axis = state.axes[departing];
                state.axis_position[departing] = current.axis(axis);
            }
            state.active = *to;
            state.axis_position[*to]
        }
        Plan::Restore(saved) => {
            state.axis_position = saved.axis_position.clone();
            state.active = saved.active;
            state.axis_position[saved.active]
        }
    }
}

/// The toolhead's current position, when a toolhead exists and is connected
/// (`ToolHeadObject::position`).
fn current_position(printer: &Arc<Printer>) -> Option<Coord> {
    printer
        .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)?
        .position()
}

/// Re-anchor the gcode coordinate on the arriving frame: bookkeeping first,
/// then `toolhead.set_position(newpos)` — upstream calls it right after the
/// rail swap (`idex_modes.py:113-114`), and the `toolhead:set_position` event
/// re-anchors `gcode_move` behind it. A machine with no connected toolhead
/// keeps the frames only; there is no coordinate to move.
///
/// # Errors
/// A failure from the toolhead's flush (`set_position`).
async fn apply(
    shared: &Arc<Mutex<Shared>>,
    printer: &Arc<Printer>,
    plan: Plan,
) -> Result<(), CommandError> {
    let toolhead = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT);
    let current = toolhead.as_ref().and_then(|toolhead| toolhead.position());
    let (axis, arriving) = {
        let mut state = shared.lock().unwrap_or_else(|poison| poison.into_inner());
        let arriving = carry_frame(&mut state, &plan, current);
        // The arriving carriage rides its own axis — for a machine whose dual
        // carriages span two axes (`SET_DUAL_CARRIAGE CARRIAGE=carriage_v`)
        // that is not the departing one's.
        (state.axes[state.active], arriving)
    };
    let (Some(toolhead), Some(current)) = (toolhead, current) else {
        return Ok(());
    };
    let mut newpos = current;
    newpos.set_axis(axis, arriving);
    toolhead.set_position(newpos, &[]).await
}

/// `SET_DUAL_CARRIAGE CARRIAGE=<name|0|1> [MODE=…]`: validate and pick the
/// active carriage — the coordinate handover happens in `apply`
/// (`idex_modes.py:240-262`).
///
/// The carriage **name** is looked up first; the `0`/`1` index form is only a
/// fallback when the name does not match. Upstream keys `self.dc_rails` by
/// each carriage's short name and only tries `int()` when there are exactly
/// two carriages (`idex_modes.py:243-254`); the bare `[dual_carriage]` module
/// always carries two (the claimed primary rail and the second carriage), so
/// the fallback is available there, while a generic-cartesian machine's
/// carriage pairs leave it off exactly as upstream does.
///
/// `MODE` is validated then only recorded — applying it is the motion-layer
/// gap above (`idex_modes.py:240-262`).
///
/// # Errors
/// Upstream's argument wordings for a missing/invalid `CARRIAGE` or `MODE`.
fn select_carriage(
    shared: &Arc<Mutex<Shared>>,
    gcmd: &GcodeCommand,
) -> Result<usize, CommandError> {
    let carriage = match gcmd.get_command_parameters().get("CARRIAGE") {
        Some(raw) => raw.clone(),
        None => return Err(CommandError::new("CARRIAGE must be specified")),
    };
    // A carriage by name wins; the key is the carriage's short name.
    let names = shared
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .names
        .clone();
    let index = match names
        .iter()
        .position(|name| name.as_deref() == Some(carriage.trim()))
    {
        Some(index) => index,
        // The index fallback: the corpus passes `CARRIAGE=0` / `CARRIAGE=1`,
        // and upstream offers it only for a machine with exactly two carriages
        // (`idex_modes.py:247-254`). Anything outside `0..=1` keeps upstream's
        // index wording; a name that matched nothing is the `specified`
        // wording.
        None => match carriage.trim().parse::<i64>() {
            Ok(index) if names.len() == 2 && (0..=1).contains(&index) => index as usize,
            Ok(index) if names.len() == 2 => {
                return Err(CommandError::new(format!("Invalid CARRIAGE={index} index")))
            }
            _ => {
                return Err(CommandError::new(format!(
                    "Invalid CARRIAGE={carriage} specified"
                )))
            }
        },
    };
    let mode = gcmd.get_str_default("MODE", "PRIMARY").to_uppercase();
    if !VALID_MODES.contains(&mode.as_str()) {
        return Err(CommandError::new(format!("Invalid mode={mode} specified")));
    }
    Ok(index)
}

/// `SAVE_DUAL_CARRIAGE_STATE [NAME=…]`: remember the active carriage and
/// both frames under `NAME` (default `default`) — upstream saves the axis
/// positions too (`idex_modes.py:283-293`), so the active frame is refreshed
/// from the toolhead first.
fn cmd_save_dual_carriage_state(
    shared: &Arc<Mutex<Shared>>,
    gcmd: &GcodeCommand,
    printer: Option<&Arc<Printer>>,
) -> Result<(), CommandError> {
    let name = gcmd.get_str_default("NAME", "default");
    let mut state = shared.lock().unwrap_or_else(|poison| poison.into_inner());
    if let Some(current) = printer.and_then(current_position) {
        let active = state.active;
        let axis = state.axes[active];
        state.axis_position[active] = current.axis(axis);
    }
    let saved = SavedState {
        active: state.active,
        axis_position: state.axis_position.clone(),
    };
    state.saved.insert(name, saved);
    Ok(())
}

/// `RESTORE_DUAL_CARRIAGE_STATE [NAME=…]`: fetch the snapshot to apply,
/// refusing an unknown state with upstream's wording
/// (`idex_modes.py:295-302`). `MOVE_SPEED` and `MOVE` are still read — the
/// option check demands it — but physically moving the carriages is the
/// motion-layer gap in the module docs, so the coordinate is re-anchored
/// instead (`apply`).
///
/// # Errors
/// Upstream's unknown-state wording, or an unparsable option.
fn cmd_restore_dual_carriage_state(
    shared: &Arc<Mutex<Shared>>,
    gcmd: &GcodeCommand,
) -> Result<SavedState, CommandError> {
    let name = gcmd.get_str_default("NAME", "default");
    // Upstream moves the carriages with these (`idex_modes.py:300-301`);
    // here they are read and validated, not acted on (module docs).
    let _move_speed = gcmd.get_float_default("MOVE_SPEED", 0.)?;
    let _move = gcmd.get_int_default("MOVE", 1)?;
    let state = shared.lock().unwrap_or_else(|poison| poison.into_inner());
    state
        .saved
        .get(&name)
        .cloned()
        .ok_or_else(|| CommandError::new(format!("Unknown DUAL_CARRIAGE state: {name}")))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::mathutil::Coord;
    use crate::core::klippy::motion::extra::ExtraAxis;
    use crate::core::klippy::motion::kinematics::MoveContext;
    use crate::core::klippy::motion::plan::{Move, MoveLimits};
    use crate::core::klippy::motion::queuing::MotionQueuing;
    use crate::core::klippy::motion::toolhead::ToolHead;
    use crate::core::klippy::reactor::ManualReactor;

    /// A printer with the given sections loaded (load only, no connect).
    /// A successful load fires `klippy:ready` so the dispatcher runs scripts,
    /// as temperature_fan's tests do.
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = Config::from_text(text).expect("the test config parses").0;
        let result = printer.load_config(&config);
        if result.is_ok() {
            printer.send_event(&KlippyEvent::KlippyReady);
        }
        (printer, result)
    }

    /// The cartesian machine the corpus runs, minus the heaters: three
    /// stepper rails plus the second carriage on X.
    fn cartesian_config(kinematics: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [dual_carriage]\naxis: x\nsafe_distance: 50\nstep_pin: PH1\ndir_pin: PH0\nenable_pin: !PA7\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PE4\nposition_endstop: 200\n\
             position_max: 200\nhoming_speed: 50\n\
             [printer]\nkinematics: {kinematics}\nmax_velocity: 300\nmax_accel: 3000\n"
        )
    }

    fn module(printer: &Arc<Printer>) -> Arc<DualCarriageModule> {
        printer
            .lookup_object_as::<DualCarriageModule>(DUAL_CARRIAGE_OBJECT)
            .expect("the dual_carriage object is registered")
    }

    /// The cartesian claim path (`cartesian.py:24-34`): the section loads,
    /// every option it carries lands in the option check's access record, the
    /// toolhead hands the module its primary rail, and the carriage index
    /// starts at 0.
    #[test]
    fn cartesian_claims_the_dual_carriage_section() {
        let (printer, result) = load(&cartesian_config("cartesian"));
        result.unwrap();

        let module = module(&printer);
        assert_eq!(module.axis(), Axis::X);
        assert_eq!(
            module.claimed_primary_rail().as_deref(),
            Some("stepper_x"),
            "the kinematics claims the rail the carriage rides on"
        );
        assert_eq!(module.active_carriage(), 0, "the initial carriage index");
        assert_eq!(
            module.get_status(0.0),
            json!({ "active_carriage": 0, "claimed": true })
        );

        // Every option of the real corpus section was read
        // (`config/validate.rs:44-51`); `load_config` already proves it by
        // passing `check_unused`, this pins the set.
        let config = Config::from_text(&cartesian_config("cartesian"))
            .expect("the test config parses")
            .0;
        let section = config.get_section("dual_carriage").expect("the section");
        let access = printer.access_tracking();
        for option in section.parameters.keys() {
            assert!(
                access.contains("dual_carriage", option),
                "unread option '{option}'"
            );
        }
    }

    /// The claim is the cartesian kinematics' alone: another kinematics loads
    /// the same section (the option check still sees every read) but never
    /// gets the rail handed over.
    #[test]
    fn a_non_cartesian_kinematics_loads_the_section_without_claiming_it() {
        let (printer, result) = load(&cartesian_config("corexy"));
        result.unwrap();

        let module = module(&printer);
        assert_eq!(module.claimed_primary_rail(), None);
        assert_eq!(
            module.get_status(0.0),
            json!({ "active_carriage": 0, "claimed": false })
        );
    }

    /// The object and the three upstream commands register
    /// (`idex_modes.py:46-59`), and `SET_DUAL_CARRIAGE` / `SAVE` / `RESTORE`
    /// move the recorded index the way the corpus script drives them.
    #[test]
    fn the_object_registers_the_three_commands_and_tracks_the_active_carriage() {
        let (printer, result) = load(&cartesian_config("cartesian"));
        result.unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let help = gcode.command_help();
        for (name, desc) in [
            ("SET_DUAL_CARRIAGE", "Configure the dual carriages mode"),
            (
                "SAVE_DUAL_CARRIAGE_STATE",
                "Save dual carriages modes and positions",
            ),
            (
                "RESTORE_DUAL_CARRIAGE_STATE",
                "Restore dual carriages modes and positions",
            ),
        ] {
            assert_eq!(help.get(name).map(String::as_str), Some(desc), "{name}");
        }

        let module = module(&printer);
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=1")
            .unwrap();
        assert_eq!(module.active_carriage(), 1);
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY")
            .unwrap();
        assert_eq!(module.active_carriage(), 0);

        gcode.run_script_sync("SAVE_DUAL_CARRIAGE_STATE").unwrap();
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=1")
            .unwrap();
        gcode
            .run_script_sync("RESTORE_DUAL_CARRIAGE_STATE")
            .unwrap();
        assert_eq!(module.active_carriage(), 0, "the saved index is restored");
    }

    /// `CARRIAGE=` takes a carriage **name** first (`idex_modes.py:243-254`):
    /// the primary rail's short name and this module's own short name, while
    /// the `0`/`1` index form stays usable as the fallback the corpus passes.
    /// A name that matches nothing is the `specified` wording, never the
    /// index one.
    #[test]
    fn the_carriage_name_selects_the_carriage_before_the_index() {
        let (printer, result) = load(&cartesian_config("cartesian"));
        result.unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        let module = module(&printer);

        // `dual_carriage` is this module's own short name; `x` is the claimed
        // primary rail's (`stepper_x` → `x`).
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=dual_carriage")
            .unwrap();
        assert_eq!(module.active_carriage(), 1);
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=x")
            .unwrap();
        assert_eq!(module.active_carriage(), 0);

        // The name the caller wanted is not one this machine carries.
        let err = gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=carriage_u")
            .unwrap_err();
        assert_eq!(err.to_string(), "Invalid CARRIAGE=carriage_u specified");

        // The index fallback still works alongside the names.
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=1")
            .unwrap();
        assert_eq!(module.active_carriage(), 1);
    }

    /// The handover bookkeeping (`toggle_active_dc_rail`,
    /// `idex_modes.py:101-114`): a switch records the departing frame and
    /// arrives on the other carriage's; a restore applies the snapshot
    /// wholesale; without a toolhead position only the active index moves —
    /// the pre-handover behaviour the load-only tests see.
    #[test]
    fn the_frames_carry_across_switches_and_restores() {
        let mut state = Shared::for_section(axis_index(Axis::X), "dual_carriage".to_string());

        // On carriage 0 at X=50: switch to 1, whose frame is still 0.
        let arriving = carry_frame(
            &mut state,
            &Plan::Switch(1),
            Some(Coord::new(50.0, 0.0, 0.0, 1.5)),
        );
        assert_eq!(state.active, 1);
        assert_eq!(state.axis_position, [50.0, 0.0]);
        assert_eq!(arriving, 0.0, "the arriving frame's coordinate");

        // On carriage 1 at X=190: switch back, c0's 50 is carried over.
        let arriving = carry_frame(
            &mut state,
            &Plan::Switch(0),
            Some(Coord::new(190.0, 0.0, 0.0, 1.5)),
        );
        assert_eq!(state.axis_position, [50.0, 190.0]);
        assert_eq!(arriving, 50.0, "the gcode coordinate lands on c0's frame");

        // A snapshot applies wholesale on restore, whatever the toolhead says.
        let saved = SavedState {
            active: 1,
            axis_position: vec![10.0, 170.0],
        };
        let arriving = carry_frame(&mut state, &Plan::Restore(saved), None);
        assert_eq!(state.axis_position, [10.0, 170.0]);
        assert_eq!(state.active, 1);
        assert_eq!(arriving, 170.0);

        // No toolhead to read: the frames stay put, the index still moves.
        state.axis_position = vec![7.0, 9.0];
        let arriving = carry_frame(&mut state, &Plan::Switch(0), None);
        assert_eq!(state.active, 0);
        assert_eq!(state.axis_position, [7.0, 9.0]);
        assert_eq!(arriving, 7.0);
    }

    /// A homed axis carries every carriage of it to its own
    /// `position_endstop`: upstream's `DualCarriages.home` homes each carriage
    /// there and the scale/offset transform records where it stopped
    /// (`idex_modes.py:116-131`). Without this refresh a dual carriage's
    /// frame stays `0.0` forever, and the first switch onto it re-anchors the
    /// toolhead at the origin — `corexyuv.test`'s `G1 X-10` then ends at
    /// X=-10 (`Move out of range`).
    #[test]
    fn the_frames_follow_the_homed_carriages_to_their_endstops() {
        let carriages = [
            GenericCarriage {
                name: "carriage_x".to_string(),
                axis: Axis::X,
                position_endstop: 0.0,
            },
            GenericCarriage {
                name: "carriage_u".to_string(),
                axis: Axis::X,
                position_endstop: 300.0,
            },
            GenericCarriage {
                name: "carriage_v".to_string(),
                axis: Axis::Y,
                position_endstop: 200.0,
            },
        ];
        let mut state = Shared::for_carriages(&carriages);
        assert_eq!(state.axis_position, [0.0, 0.0, 0.0]);

        // `G28 X`: both X carriages sit at their endstops; Y is untouched.
        state.homed(&[0]);
        assert_eq!(state.axis_position, [0.0, 300.0, 0.0]);

        // `G28 Y`: the Y carriage follows, the homed X frames stay put.
        state.homed(&[1]);
        assert_eq!(state.axis_position, [0.0, 300.0, 200.0]);
    }

    /// The command errors the corpus' neighbourhood can reach, in upstream's
    /// wording (`idex_modes.py:244-262`, `:301-302`).
    #[test]
    fn the_commands_refuse_bad_arguments_upstream_style() {
        let (printer, result) = load(&cartesian_config("cartesian"));
        result.unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();

        let err = gcode.run_script_sync("SET_DUAL_CARRIAGE").unwrap_err();
        assert_eq!(err.to_string(), "CARRIAGE must be specified");

        let err = gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=5")
            .unwrap_err();
        assert_eq!(err.to_string(), "Invalid CARRIAGE=5 index");

        let err = gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=front")
            .unwrap_err();
        assert_eq!(err.to_string(), "Invalid CARRIAGE=front specified");

        let err = gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=1 MODE=teleport")
            .unwrap_err();
        assert_eq!(err.to_string(), "Invalid mode=TELEPORT specified");

        let err = gcode
            .run_script_sync("RESTORE_DUAL_CARRIAGE_STATE NAME=nope")
            .unwrap_err();
        assert_eq!(err.to_string(), "Unknown DUAL_CARRIAGE state: nope");
    }

    /// A `safe_distance` below zero is refused with upstream's bound
    /// (`cartesian.py:33-34`).
    #[test]
    fn a_negative_safe_distance_is_refused() {
        let (_, result) =
            load(&cartesian_config("cartesian").replace("safe_distance: 50", "safe_distance: -1"));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must have minimum of 0"), "{err}");
    }

    // -----------------------------------------------------------------------
    // The multi-extruder guard (module docs): the corpus config carries two
    // extruders, and a move has four slots — the second extra axis must be
    // skipped, not indexed. The recording axis reads `axes_d[ea_index]` like
    // the real extruder does, so an unguarded planner panics here exactly as
    // `dual_carriage.test` did on its first `G1`.
    // -----------------------------------------------------------------------

    #[derive(Debug)]
    struct RecordingAxis {
        name: &'static str,
        checked: Mutex<Vec<usize>>,
        junctions: Mutex<Vec<usize>>,
        queued: Mutex<Vec<usize>>,
    }

    impl RecordingAxis {
        fn new(name: &'static str) -> Self {
            Self {
                name,
                checked: Mutex::new(Vec::new()),
                junctions: Mutex::new(Vec::new()),
                queued: Mutex::new(Vec::new()),
            }
        }
    }

    impl ExtraAxis for RecordingAxis {
        fn name(&self) -> &str {
            self.name
        }

        fn check_move(
            &self,
            ctx: &mut MoveContext<'_>,
            ea_index: usize,
        ) -> Result<(), CommandError> {
            let _ = ctx.axes_d()[ea_index];
            self.checked.lock().unwrap().push(ea_index);
            Ok(())
        }

        fn calc_junction(&self, prev: &Move, cur: &Move, ea_index: usize) -> f64 {
            let _ = prev.axes_d[ea_index];
            let _ = cur.axes_d[ea_index];
            self.junctions.lock().unwrap().push(ea_index);
            1234.0
        }

        fn process_move(
            &self,
            _queuing: &mut MotionQueuing,
            _print_time: f64,
            move_: &Move,
            ea_index: usize,
        ) {
            let _ = move_.axes_d[ea_index];
            self.queued.lock().unwrap().push(ea_index);
        }

        fn find_past_position(&self, _print_time: f64) -> f64 {
            0.0
        }

        fn get_status(&self) -> Value {
            json!({})
        }
    }

    /// The guard's two sides, pinned: with two extra axes the planner skips
    /// the out-of-slot second one in **all three** paths (per-move check,
    /// junction fold-in, trapq queueing) without panicking, while the first
    /// extra axis keeps its slot-3 behaviour unchanged.
    #[test]
    fn a_second_extra_axis_is_skipped_and_the_first_keeps_its_slot() {
        let mut toolhead = ToolHead::new(MoveLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            junction_deviation: 0.05,
            mcr_pseudo_accel: 1500.0,
        });
        let first = Arc::new(RecordingAxis::new("extruder"));
        let second = Arc::new(RecordingAxis::new("extruder1"));
        toolhead.add_extra_axis(Arc::clone(&first) as Arc<dyn ExtraAxis>);
        toolhead.add_extra_axis(Arc::clone(&second) as Arc<dyn ExtraAxis>);

        // Two extrude-only moves: the check path runs for both moves, the
        // junction path pairs the second move with the first.
        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 5.0), 10.0)
            .unwrap();
        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 6.0), 10.0)
            .unwrap();
        toolhead.flush_step_generation(1.0).unwrap();

        // Slot 3 (the first extra axis) is checked once per move, queued once
        // per move, and folds one junction limit in — as before the guard.
        assert_eq!(*first.checked.lock().unwrap(), [3, 3]);
        assert_eq!(*first.junctions.lock().unwrap(), [3]);
        assert_eq!(*first.queued.lock().unwrap(), [3, 3]);
        // Slot 4 does not exist: the second extra axis never runs a hook.
        assert!(second.checked.lock().unwrap().is_empty());
        assert!(second.junctions.lock().unwrap().is_empty());
        assert!(second.queued.lock().unwrap().is_empty());
    }
}
