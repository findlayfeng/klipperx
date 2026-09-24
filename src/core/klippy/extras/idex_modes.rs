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
//! rails, so no steps are generated for it, and `SET_DUAL_CARRIAGE` only
//! updates the bookkeeping. The upstream behaviour still missing is listed
//! below; the idex family's next units (hybrid dual carriage, generic
//! cartesian) build on this seam.
//!
//! # Gaps this port does not close yet
//!
//! * **No carriage switching on the motion layer**: the active rail's trapq
//!   swap, position tracking and `update_limits` (`idex_modes.py:224-231`,
//!   `DualCarriagesRail.activate/inactivate`) are not implemented — the
//!   recorded carriage index only feeds `get_status` and the saved states.
//! * **`COPY` / `MIRROR` / `INACTIVE` modes are validated, not applied**
//!   (`idex_modes.py:15-17`); upstream's homing/`PRIMARY`-first checks are
//!   not enforced either.
//! * **Saved states store the active index only**: upstream saves carriage
//!   modes, axis positions and the toolhead position, and the restore moves
//!   the carriages (`idex_modes.py:285-310`).
//! * **The second carriage's endstop is not in `query_endstops`**, and
//!   `STEPPER_BUZZ STEPPER=dual_carriage` is not registered (both are silent
//!   unknown-command answers, so the corpus does not care).

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::stepper::{axis_index, PrinterStepper, Rail};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::motion::Axis;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Read during the late walk, after the `[stepper_*]` sections and before
// `[printer]` builds the kinematics that claims it.
section!(
    "dual_carriage",
    order = 55,
    phase = late,
    load = load_config
);

/// The name the object (and the section) go by upstream
/// (`idex_modes.py:46 add_object('dual_carriage', self)`).
pub const DUAL_CARRIAGE_OBJECT: &str = "dual_carriage";

/// The `MODE` values `SET_DUAL_CARRIAGE` accepts (`idex_modes.py:16`).
const VALID_MODES: [&str; 4] = ["INACTIVE", "PRIMARY", "COPY", "MIRROR"];

/// What the three commands share: the carriage index that is active, and the
/// states `SAVE_DUAL_CARRIAGE_STATE` wrote.
#[derive(Default)]
struct Shared {
    /// The active carriage: 0 is the primary rail, 1 the second carriage.
    active: usize,
    /// `SAVE_DUAL_CARRIAGE_STATE NAME=…` states, each the saved index for now
    /// (upstream saves modes and positions too; see the module docs).
    saved: HashMap<String, usize>,
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

    let shared = Arc::new(Mutex::new(Shared::default()));
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` before any section");
    let commands: [(
        &str,
        &str,
        fn(&Arc<Mutex<Shared>>, &GcodeCommand) -> Result<(), CommandError>,
    ); 3] = [
        (
            "SET_DUAL_CARRIAGE",
            "Configure the dual carriages mode",
            cmd_set_dual_carriage,
        ),
        (
            "SAVE_DUAL_CARRIAGE_STATE",
            "Save dual carriages modes and positions",
            cmd_save_dual_carriage_state,
        ),
        (
            "RESTORE_DUAL_CARRIAGE_STATE",
            "Restore dual carriages modes and positions",
            cmd_restore_dual_carriage_state,
        ),
    ];
    for (name, desc, body) in commands {
        let shared = Arc::clone(&shared);
        let handler: CommandHandler = sync(move |gcmd| body(&shared, gcmd));
        gcode
            .register_command(name, handler, Some(desc), false)
            .map_err(ConfigError::new)?;
    }

    Ok(Arc::new(DualCarriageModule {
        axis,
        safe_distance,
        stepper,
        primary_rail: Mutex::new(None),
        shared,
    }))
}

/// `SET_DUAL_CARRIAGE CARRIAGE=<0|1> [MODE=…]`: pick the active carriage.
///
/// `CARRIAGE` accepts the numeric index (upstream also accepts a rail name;
/// see the module docs) and `MODE` is validated then only recorded — applying
/// it is the motion-layer gap above (`idex_modes.py:240-262`).
fn cmd_set_dual_carriage(
    shared: &Arc<Mutex<Shared>>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let carriage = match gcmd.get_command_parameters().get("CARRIAGE") {
        Some(raw) => raw.clone(),
        None => return Err(CommandError::new("CARRIAGE must be specified")),
    };
    let index = match carriage.trim().parse::<i64>() {
        // The corpus passes `CARRIAGE=0` / `CARRIAGE=1`; upstream rejects
        // anything outside `0..=1` with the index wording
        // (`idex_modes.py:250-252`).
        Ok(index) if (0..=1).contains(&index) => index as usize,
        Ok(index) => return Err(CommandError::new(format!("Invalid CARRIAGE={index} index"))),
        Err(_) => {
            return Err(CommandError::new(format!(
                "Invalid CARRIAGE={carriage} specified"
            )))
        }
    };
    let mode = gcmd.get_str_default("MODE", "PRIMARY").to_uppercase();
    if !VALID_MODES.contains(&mode.as_str()) {
        return Err(CommandError::new(format!("Invalid mode={mode} specified")));
    }
    let mut state = shared.lock().unwrap_or_else(|poison| poison.into_inner());
    state.active = index;
    Ok(())
}

/// `SAVE_DUAL_CARRIAGE_STATE [NAME=…]`: remember the active carriage under
/// `NAME` (default `default`) (`idex_modes.py:283-284`).
fn cmd_save_dual_carriage_state(
    shared: &Arc<Mutex<Shared>>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let name = gcmd.get_str_default("NAME", "default");
    let mut state = shared.lock().unwrap_or_else(|poison| poison.into_inner());
    let active = state.active;
    state.saved.insert(name, active);
    Ok(())
}

/// `RESTORE_DUAL_CARRIAGE_STATE [NAME=…]`: re-activate the carriage saved
/// under `NAME`, refusing an unknown state with upstream's wording
/// (`idex_modes.py:295-302`).
fn cmd_restore_dual_carriage_state(
    shared: &Arc<Mutex<Shared>>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let name = gcmd.get_str_default("NAME", "default");
    // Upstream moves the carriages with these (`idex_modes.py:300-301`);
    // here they are read and validated, not acted on (module docs).
    let _move_speed = gcmd.get_float_default("MOVE_SPEED", 0.)?;
    let _move = gcmd.get_int_default("MOVE", 1)?;
    let mut state = shared.lock().unwrap_or_else(|poison| poison.into_inner());
    let saved = state
        .saved
        .get(&name)
        .ok_or_else(|| CommandError::new(format!("Unknown DUAL_CARRIAGE state: {name}")))?;
    state.active = *saved;
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    /// A printer with the given sections loaded (load only, no connect).
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = Config::from_text(text).expect("the test config parses").0;
        let result = printer.load_config(&config);
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
}
