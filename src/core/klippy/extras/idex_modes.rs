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
//! # The carriage's motor is a rail of the kinematics, not an extra axis
//!
//! The second carriage is driven the way upstream drives it: as a motor of the
//! **cartesian kinematics**, reading the main trapq. Upstream appends the
//! carriage rail to `CartKinematics.rails` (`cartesian.py:28-30`) and points
//! every rail at `toolhead.get_trapq()` (`cartesian.py:37-38`), so the carriage
//! motor and `stepper_x` generate from the same queue — each with its own
//! solver, `cartesian_stepper_alloc(dc_axis)` for the carriage (`:29-30`), the
//! same position function this port's [`PrinterStepper`] defaults to for its
//! `axis`. It is **not** an extra axis: an extra axis carries a coordinate slot
//! of its own and its own trapq
//! ([`add_extra_axis_stepper`](crate::core::klippy::extras::toolhead::ToolHeadObject::add_extra_axis_stepper)),
//! while this motor follows the toolhead's X (or Y) coordinate, which is
//! exactly what the main trapq holds. Upstream's per-rail transform (the
//! `dual_carriage` c-helper's scale/offset, `chelper/kin_idex.c`) is the mode
//! layer listed under the gaps below.
//!
//! The handover happens at the toolhead's connect, gated on the claim: the bare
//! `[dual_carriage]` section loads for every kinematics here, while upstream
//! builds the carriage rail only inside `CartKinematics`, so
//! [`DualCarriageModule::take_stepper`] answers only once a kinematics claimed
//! the section.
//!
//! What the three commands *do* carry across in this port is the **gcode
//! coordinate**: switching or restoring parks the departing carriage's axis
//! coordinate and re-anchors the toolhead on the arriving carriage's frame, the
//! way upstream's `toggle_active_dc_rail` calls `toolhead.set_position(newpos)`
//! (`idex_modes.py:101-115`) — the `toolhead:set_position` event re-anchors
//! `gcode_move` with it, so the next move lands in the new carriage's frame and
//! its range. The upstream behaviour still missing is listed below; the hybrid
//! dual carriage unit builds on this seam.
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
//! * **No carriage switching on the motion layer**: the scale/offset transform
//!   and `update_limits` (`idex_modes.py:224-235`,
//!   `DualCarriagesRail.activate/inactivate`) are not implemented — the
//!   coordinate handover teleports the toolhead onto the new frame, and both
//!   carriage motors step off the main trapq (so the inactive carriage travels
//!   with the active one instead of holding its stop), and the active
//!   carriage's **range** stays the primary rail's (identical in the corpus,
//!   where both carriages ride `[0, 200]`).
//! * **The carriage rail is the cartesian family's alone**: a `[dual_carriage]`
//!   under another kinematics builds the module (this loader loads the section
//!   regardless) but no kinematics claims it, so its motor steps nowhere. The
//!   hybrid-corexy family is the concrete case: upstream builds the rail with
//!   `corexy_stepper_alloc(b'+')` (`kinematics/hybrid_corexy.py:31-36`), which
//!   this port does not install for the section's stepper.
//! * **`COPY` / `MIRROR` / `INACTIVE` modes are validated, not applied**
//!   (`idex_modes.py:15-17`); upstream's homing/`PRIMARY`-first checks are
//!   not enforced either.
//! * **Saved states store the active index and the axis frames**: carriage
//!   modes (`PRIMARY`/`COPY`/`MIRROR`) are not modelled, the restore
//!   re-anchors the coordinate instead of physically moving the carriages,
//!   and `MOVE_SPEED` is read but unused (`idex_modes.py:285-310`).
//! * **The second carriage's endstop is not in `query_endstops`**: upstream
//!   builds the rail with its own `endstop_pin` (`cartesian.py:25-29`), so
//!   `QUERY_ENDSTOPS` reports it and `G28` homes both carriages
//!   (`DualCarriages.home`, `idex_modes.py:116-132`). Here the section's
//!   endstop drives its trigger but is not queryable, and only the primary
//!   carriage homes.
//! * **`G28` drives the second carriage without its endstop stopping it**: the
//!   homing move is the X (or Y) coordinate, and both carriage motors read it
//!   off the main trapq, so the second carriage travels the whole move while
//!   only the primary rail's endstop is in the trigger (`stepper_x`). Upstream
//!   homes the carriages **one at a time**, each with its own endstop armed
//!   (`idex_modes.py:116-132`). On a real IDEX machine the untriggered carriage
//!   therefore runs the full homing move instead of stopping on its own
//!   `position_endstop`.
//! * **A config with two extruders ([`dual_carriage.cfg`](crate and its T0/T1
//!   macros) puts **both** extruders in the motion path**: a `Move` carries one
//!   position slot per extra axis (`Move::extra_start`/`extra_end`, with
//!   `axes_d`/`axes_r` as long as the toolhead's axis list), so each extruder
//!   checks and queues its own trapezoid, and `gcode_move`'s `axis_map` offers
//!   a `G1` letter for each registered axis.
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
use crate::core::klippy::mcu::McuStepper;
use crate::core::klippy::motion::{Axis, Stepper};
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
    /// `toggle_active_dc_rail` (`idex_modes.py:101-115`), where upstream
    /// instead reads it off the scale/offset transform of a second rail that
    /// this port does not drive (module docs) — and set to the carriage's own
    /// `position_endstop` when its axis homes ([`Shared::homed`]).
    axis_position: Vec<f64>,
    /// The names `CARRIAGE=` takes, in carriage order — upstream's `dc_rails`
    /// keys, each carriage's `rail.get_name(short=True)`
    /// (`idex_modes.py:40-43`). This module's own name is known when the
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
    /// (`DualCarriages.home`, `idex_modes.py:116-132`, which toggles each
    /// rail and homes it) and the frames follow the physical carriages;
    /// [`Shared::homed`] is this port's copy of that bookkeeping, and the
    /// bare `[dual_carriage]` section never calls it (its entries stay `0.0`,
    /// like its frames before any homing).
    endstops: Vec<f64>,
    /// `SAVE_DUAL_CARRIAGE_STATE NAME=…` states: the active index and the
    /// axis frames (`idex_modes.py:285-292`; modes are not modelled — module
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
    /// `DualCarriages.home` leaves them (`idex_modes.py:116-132`) — without
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
    /// `LookupMultiRail(dc_config)` is (`cartesian.py:25-29`). The kinematics'
    /// claim is what makes it a motor of the machine: the toolhead takes it
    /// through [`DualCarriageModule::take_stepper`] (module docs).
    stepper: Arc<PrinterStepper>,
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

    /// Take the second carriage's host stepper, for the toolhead to drive from
    /// the main trapq (`cartesian.py:28-30,37-38`; module docs).
    ///
    /// `None` before connect and after the toolhead took it, as
    /// [`PrinterStepper::take_stepper`] answers. The caller decides *whether*
    /// the carriage is a motor of the machine at all: only the cartesian family
    /// claims the section ([`claim`]), which is upstream's gate for building the
    /// rail at all.
    pub fn take_stepper(&self) -> Option<Stepper> {
        self.stepper.take_stepper()
    }

    /// The firmware stepper resource of the second carriage (the name the
    /// toolhead's `mcu_steppers` map and `queue_step` batches go by is the
    /// section's: `dual_carriage`).
    pub fn mcu_stepper(&self) -> Arc<McuStepper> {
        Arc::clone(self.stepper.mcu_stepper())
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
    // (`rail.get_name(short=True)`, `stepper.py:388-394`), the key upstream's
    // `dc_rails` uses for `CARRIAGE=` (`idex_modes.py:40-43`).
    module.shared_lock().names[0] = Some(short_rail_name(rail.name()).to_string());
}

/// A rail's short name (`GenericPrinterRail.get_name(short=True)`,
/// `stepper.py:388-394`): a `stepper_x` rail is `x`, `stepper_z1` is `z1`,
/// and anything else is its last whitespace-separated word.
///
/// `pub(crate)` because the homing driver's "No trigger on … after full
/// movement" message names rails the same way upstream does
/// (`extras/homing.py:119` uses this short name, while `M119` labels use the
/// full stepper name — see `toolhead`'s module docs).
pub(crate) fn short_rail_name(name: &str) -> &str {
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
    // `dc_config.getchoice('axis', ['x', 'y'])` (`cartesian.py:25-26`).
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
    // `stepper.LookupMultiRail(dc_config)` (`cartesian.py:25-29`), which is
    // exactly what building the primary stepper here reads.
    let stepper = Arc::new(PrinterStepper::new(config, printer, axis, true)?);
    // A hidden object so the stepper's own `connect` runs and builds the host
    // `Stepper` the toolhead takes at its own connect — the same shape
    // `[extruder]` uses for its stepper (`extruder_stepper <name>`) and
    // `[manual_stepper]` for its own (`manual_stepper_stepper <name>`). Without
    // it the section's `PrinterStepper` is never connected, so its host solver
    // (and with it any step generation) does not exist at all.
    printer.add_object(
        &format!("dual_carriage_stepper {identifier}"),
        Arc::clone(&stepper) as Arc<dyn PrinterObject>,
    )?;

    let shared = Arc::new(Mutex::new(Shared::for_section(
        axis_index(axis),
        // This module's own carriage name is the section's short name
        // (`rail.get_name(short=True)`, `stepper.py:388-394`), the key upstream's
        // `dc_rails` uses for `CARRIAGE=` (`idex_modes.py:40-43`); the bare
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
/// `idex_modes.py:46-59,116-132`).
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

    // Upstream's `DualCarriages.home` (`idex_modes.py:116-132`) homes every
    // carriage of the axis to its own endstop and the frames follow the
    // physical carriages there; this port homes the axis through the
    // kinematics instead, so the same refresh hangs off the homing event.
    printer.register_event_handler(
        KlippyEvent::HomingHomeRailsEnd {
            axes: Vec::new(),
            homing: crate::core::klippy::motion::HomingHandle::new(),
        },
        Box::new({
            let shared = Arc::clone(&shared);
            move |event| {
                if let KlippyEvent::HomingHomeRailsEnd { axes, .. } = event {
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

/// The words `select_carriage` reads (`idex_modes.py:240-262`), in read order.
const SET_DUAL_CARRIAGE_PARAMS: &[&str] = &["CARRIAGE", "MODE"];

/// The words `cmd_restore_dual_carriage_state` reads
/// (`idex_modes.py:295-302`), in read order.
const RESTORE_DUAL_CARRIAGE_STATE_PARAMS: &[&str] = &["NAME", "MOVE_SPEED", "MOVE"];

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
            .register_command_with_params(
                "SAVE_DUAL_CARRIAGE_STATE",
                handler,
                Some("Save dual carriages modes and positions"),
                // The state's name, read by `cmd_save_dual_carriage_state`
                // (`idex_modes.py:283-292`).
                &["NAME"],
                false,
            )
            .map_err(ConfigError::new)?;
    }

    // `SET_DUAL_CARRIAGE` / `RESTORE_DUAL_CARRIAGE_STATE` re-anchor the
    // toolhead, so they are async handlers — upstream's
    // `toolhead.set_position` (`idex_modes.py:114,348`) has the same job.
    for (name, desc, restoring, params) in [
        (
            "SET_DUAL_CARRIAGE",
            "Configure the dual carriages mode",
            false,
            SET_DUAL_CARRIAGE_PARAMS,
        ),
        (
            "RESTORE_DUAL_CARRIAGE_STATE",
            "Restore dual carriages modes and positions",
            true,
            RESTORE_DUAL_CARRIAGE_STATE_PARAMS,
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
            .register_command_with_params(name, handler, Some(desc), params, false)
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
/// `toggle_active_dc_rail` (`idex_modes.py:101-115`): record the departing
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
/// The switch also fires `dual_carriage:update_kinematics`, upstream's
/// `DualCarriagesRail.apply_transform` event (`idex_modes.py:383`): the shaping
/// layer holds impulse-derived step generation windows that have to follow the
/// kinematics the carriages left behind. Upstream sends it from inside
/// `activate`/`inactivate`, i.e. before the position is re-anchored and after
/// `toggle_active_dc_rail`'s own flush; here it fires once per switch, right
/// after the bookkeeping and **outside** the `Shared` lock — the handlers flush
/// step generation, which must not run under it.
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
    printer.send_event(&KlippyEvent::DualCarriageUpdateKinematics);
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
/// positions too (`idex_modes.py:283-292`), so the active frame is refreshed
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
    use std::sync::atomic::{AtomicUsize, Ordering};

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

    /// Every switch and restore fires `dual_carriage:update_kinematics` once —
    /// upstream's `DualCarriagesRail.apply_transform` event
    /// (`idex_modes.py:383`), which is how the shaping layer learns that the
    /// kinematics under it changed (`input_shaper.py:153-168`) — and never while
    /// the carriage state is locked: the handler flushes step generation, which
    /// must not run under a state lock the flush path could reach back into.
    /// The probe takes the lock with `try_lock`, so a regression to firing the
    /// event inside the lock fails instead of deadlocking the gate.
    #[test]
    fn a_switch_fires_the_update_kinematics_event_outside_the_lock() {
        let (printer, result) = load(&cartesian_config("cartesian"));
        result.unwrap();
        let module = module(&printer);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let fired = Arc::new(AtomicUsize::new(0));
        let unlocked = Arc::new(AtomicUsize::new(0));
        printer.register_event_handler(
            KlippyEvent::DualCarriageUpdateKinematics,
            Box::new({
                let module = Arc::clone(&module);
                let fired = Arc::clone(&fired);
                let unlocked = Arc::clone(&unlocked);
                move |_| {
                    fired.fetch_add(1, Ordering::SeqCst);
                    if module.shared.try_lock().is_ok() {
                        unlocked.fetch_add(1, Ordering::SeqCst);
                    }
                }
            }),
        );

        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=1")
            .unwrap();
        assert_eq!(fired.load(Ordering::SeqCst), 1, "a switch re-points");
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=0")
            .unwrap();
        gcode
            .run_script_sync("SAVE_DUAL_CARRIAGE_STATE NAME=s")
            .unwrap();
        gcode
            .run_script_sync("RESTORE_DUAL_CARRIAGE_STATE NAME=s")
            .unwrap();
        assert_eq!(
            fired.load(Ordering::SeqCst),
            3,
            "every switch and every restore fires the event once"
        );
        assert_eq!(
            unlocked.load(Ordering::SeqCst),
            3,
            "the event must not fire with the carriage state locked"
        );
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
    /// `idex_modes.py:101-115`): a switch records the departing frame and
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
    /// (`idex_modes.py:116-132`). Without this refresh a dual carriage's
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
    // The extra-axis slots: a move is as long as the toolhead's axis list, so
    // a second extra axis has a position slot of its own instead of being
    // skipped. The recording axis reads `axes_d[ea_index]` the way the real
    // extruder reads it, so an out-of-slot index panics here exactly as it
    // would in the extruder.
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

    /// Both slots, pinned: every extra axis has a position slot now that a
    /// move is as long as the toolhead's axis list, so each one runs its own
    /// hooks — the per-move check and the trapq queueing where its distance is
    /// non-zero, and the junction fold-in on every pair of moves.
    #[test]
    fn a_second_extra_axis_gets_its_own_slot() {
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

        // Two extrude-only moves, each naming both extra axes: the check path
        // runs for both moves, the junction path pairs the second with the
        // first, and queueing sees both.
        toolhead
            .move_to_extended(Coord::new(0.0, 0.0, 0.0, 5.0), &[1.0, 2.0], 10.0)
            .unwrap();
        toolhead
            .move_to_extended(Coord::new(0.0, 0.0, 0.0, 6.0), &[1.5, 2.5], 10.0)
            .unwrap();
        toolhead.flush_step_generation(1.0).unwrap();

        // Slot 3 (the first extra axis) and slot 4 (the second) each run their
        // hooks once per move.
        assert_eq!(*first.checked.lock().unwrap(), [3, 3]);
        assert_eq!(*first.junctions.lock().unwrap(), [3]);
        assert_eq!(*first.queued.lock().unwrap(), [3, 3]);
        assert_eq!(*second.checked.lock().unwrap(), [4, 4]);
        assert_eq!(*second.junctions.lock().unwrap(), [4]);
        assert_eq!(*second.queued.lock().unwrap(), [4, 4]);
    }

    // -----------------------------------------------------------------------
    // The second carriage on the motion layer: end to end, fake firmware
    // -----------------------------------------------------------------------

    /// The corpus' `dual_carriage.cfg` machine over the fake firmware
    /// (`[mcu] test: dict=…`), minus the heaters, the second extruder and the
    /// servo: the cartesian IDEX shape whose second carriage used to be built
    /// and then left out of every queue.
    fn idex_machine_config(dict: &std::path::Path) -> Config {
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PE5\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 50\n\
             [dual_carriage]\naxis: x\nsafe_distance: 50\nstep_pin: PH1\ndir_pin: PH0\nenable_pin: !PA1\n\
             microsteps: 16\nrotation_distance: 40\nendstop_pin: ^PE4\nposition_endstop: 200\n\
             position_max: 200\nhoming_speed: 50\n\
             [stepper_y]\nstep_pin: PF6\ndir_pin: !PF7\nenable_pin: !PF2\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PJ1\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PL3\ndir_pin: PL1\nenable_pin: !PK0\nmicrosteps: 16\n\
             rotation_distance: 8\nendstop_pin: ^PD3\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 20\n",
            dict.display()
        );
        Config::from_text(&text).expect("the config parses").0
    }

    /// Bring that machine up against the fake firmware, as the corpus harness
    /// does (`upstream::run_case` → `printer.bring_up`).
    async fn up_idex_machine(dict: &std::path::Path) -> (Arc<Printer>, Result<(), String>) {
        let config = idex_machine_config(dict);
        let reactor = Arc::new(crate::core::klippy::reactor::TokioReactor::new(
            tokio::runtime::Handle::current(),
        ));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect(
            "dual_carriage.cfg",
            None,
            crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
        );
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));
        let setup = async {
            printer.load_config(&config).map_err(|e| e.to_string())?;
            if tokio::time::timeout(std::time::Duration::from_secs(10), printer.bring_up())
                .await
                .is_err()
            {
                return Err("bring_up timed out".to_string());
            }
            let state = printer.get_state_message();
            if state.category != crate::core::klippy::printer::PrinterState::Ready {
                return Err(format!("not ready: {}", state.message));
            }
            Ok(())
        }
        .await;
        (printer, setup)
    }

    /// The `queue_step` batches this stepper's compressor has emitted so far.
    fn step_batches(toolhead: &Arc<ToolHeadObject>, name: &str) -> usize {
        toolhead
            .stepper_step_history(name, 1000, 0, u64::MAX)
            .map(|history| history.steps.len())
            .unwrap_or(0)
    }

    /// The net steps this stepper's compressor has emitted.
    fn net_steps(toolhead: &Arc<ToolHeadObject>, name: &str) -> i64 {
        toolhead
            .stepper_step_history(name, 1000, 0, u64::MAX)
            .map(|history| history.steps.iter().map(|step| step.step_count).sum())
            .unwrap_or(0)
    }

    /// Poll `pred` until it holds, for at most `deadline`; `false` when the
    /// budget runs out.
    ///
    /// A planned move's steps are generated on the background flush tick, not
    /// while the `G1` is being read, so a test has to wait for them — polling to
    /// a deadline rather than sleeping a fixed window, the shape
    /// `test_util::wait_until` uses, but answering instead of panicking so the
    /// test can still tear the printer down.
    async fn wait_for<F: FnMut() -> bool>(deadline: std::time::Duration, mut pred: F) -> bool {
        let limit = std::time::Instant::now() + deadline;
        while !pred() {
            if std::time::Instant::now() >= limit {
                return false;
            }
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
        }
        true
    }

    /// The second carriage is a motor of the machine, not a dangling
    /// `PrinterStepper`: it is in the toolhead's stepper set with its own oid,
    /// and a move on its carriage axis produces `queue_step` batches for it —
    /// the same count `stepper_x` produces, because both read the carriage's
    /// coordinate off the main trapq with the same step distance.
    ///
    /// A move on the *other* axis steps it not at all: that is what pins the
    /// wiring to the main trapq and to the carriage axis' solver, rather than to
    /// some queue that happens to `queue_step` whenever the machine moves.
    ///
    /// Skipped when the dictionary was not built (`KLIPPERX_ARCHES`).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_second_carriage_generates_steps_on_its_carriage_axis() {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        if !dict.is_file() {
            return;
        }
        let (printer, setup) = up_idex_machine(&dict).await;
        let outcome = async {
            setup?;
            let toolhead = printer
                .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
                .ok_or_else(|| "the toolhead is not registered".to_string())?;
            // In the toolhead's stepper set, which is the set the motion queue
            // generates and sends for — with the carriage's own oid, not a
            // share of the primary rail's.
            let carriage = toolhead.mcu_stepper("dual_carriage").ok_or_else(|| {
                "the second carriage is not in the toolhead's stepper set".to_string()
            })?;
            let primary = toolhead
                .mcu_stepper("stepper_x")
                .ok_or_else(|| "stepper_x is not in the toolhead's stepper set".to_string())?;
            if carriage.oid().map_err(|err| err.to_string())?
                == primary.oid().map_err(|err| err.to_string())?
            {
                return Err("the second carriage shares stepper_x's oid".to_string());
            }
            if net_steps(&toolhead, "dual_carriage") != 0 {
                return Err("the second carriage stepped before the machine moved".to_string());
            }

            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .ok_or_else(|| "the g-code dispatcher is not registered".to_string())?;
            gcode
                .run_script("SET_KINEMATIC_POSITION X=0 Y=0 Z=0")
                .await
                .map_err(|err| err.to_string())?;

            // A move off the carriage's axis leaves it alone.
            let y_before = net_steps(&toolhead, "stepper_y");
            let idle_batches = step_batches(&toolhead, "dual_carriage");
            gcode
                .run_script("G1 Y5 F600")
                .await
                .map_err(|err| format!("G1 Y5: {err}"))?;
            if !wait_for(std::time::Duration::from_secs(5), || {
                net_steps(&toolhead, "stepper_y") != y_before
            })
            .await
            {
                return Err("the Y move generated no steps on stepper_y".to_string());
            }
            if step_batches(&toolhead, "dual_carriage") != idle_batches {
                return Err("a Y move stepped the X carriage".to_string());
            }

            // A move along the carriage's axis steps it, one for one with
            // `stepper_x`: 10 mm at 40 mm / 16 microsteps = 400 steps each.
            let before = (
                net_steps(&toolhead, "stepper_x"),
                net_steps(&toolhead, "dual_carriage"),
                step_batches(&toolhead, "dual_carriage"),
            );
            gcode
                .run_script("G1 X10 F600")
                .await
                .map_err(|err| format!("G1 X10: {err}"))?;
            if !wait_for(std::time::Duration::from_secs(5), || {
                net_steps(&toolhead, "dual_carriage") != before.1
            })
            .await
            {
                return Err(format!(
                    "the X move produced no queue_step for the second carriage \
                     (stepper_x stepped {})",
                    net_steps(&toolhead, "stepper_x") - before.0
                ));
            }
            let after = (
                net_steps(&toolhead, "stepper_x"),
                net_steps(&toolhead, "dual_carriage"),
                step_batches(&toolhead, "dual_carriage"),
            );
            let primary_steps = after.0 - before.0;
            let carriage_steps = after.1 - before.1;
            if carriage_steps <= 0 {
                return Err(format!(
                    "the X move produced no queue_step for the second carriage \
                     (stepper_x stepped {primary_steps})"
                ));
            }
            if primary_steps != carriage_steps {
                return Err(format!(
                    "the carriages stepped differently on one move: stepper_x \
                     {primary_steps}, dual_carriage {carriage_steps}"
                ));
            }
            if after.2 == before.2 {
                return Err("no queue_step batch was emitted for the second carriage".to_string());
            }
            Ok::<(), String>(())
        }
        .await;
        printer.teardown();
        outcome.unwrap();
    }
}
