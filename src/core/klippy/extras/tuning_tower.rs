//! `[tuning_tower]` — adjust a parameter at each Z height (upstream
//! `klippy/extras/tuning_tower.py`).
//!
//! The section reads **no options** (`tuning_tower.py:14-23`); what it does at
//! load is register `TUNING_TOWER` and subscribe to the events below.
//!
//! `TUNING_TOWER COMMAND=… PARAMETER=…` starts a test: the module takes
//! `gcode_move`'s move-transform slot and, on every extrusion move at a new Z,
//! re-evaluates the parameter from the current **g-code** Z and runs the
//! command when the value changed (`tuning_tower.py:71-92`).
//!
//! | word | meaning (`tuning_tower.py:27-38`) |
//! |---|---|
//! | `COMMAND` | required; the command to run, e.g. `SET_VELOCITY_LIMIT` |
//! | `PARAMETER` | required; the word that command's value goes in, e.g. `ACCEL` |
//! | `START` | value at Z 0 (default 0) |
//! | `FACTOR` | value added per mm of Z (default 0) |
//! | `BAND` | with `FACTOR`: evaluate the middle of each `BAND`-tall band (default 0, `≥ 0`) |
//! | `STEP_DELTA` | with `STEP_HEIGHT`: value added per step (default 0) |
//! | `STEP_HEIGHT` | height of one step (default 0, `≥ 0`) |
//! | `SKIP` | Z distance to ignore at the start (default 0, `≥ 0`) |
//!
//! The emitted line is upstream's `command_fmt`: a traditional command gets
//! `COMMAND PARAMETER<value>`, an extended one `COMMAND PARAMETER=<value>`,
//! the value with nine decimals (`tuning_tower.py:41-44`).
//!
//! A move that extrudes more than 2mm of Z **below** the last one is the start
//! of another print, so the test ends (`CANCEL_Z_DELTA`,
//! `tuning_tower.py:6,75-78`): the slot goes back to the transform the tower
//! displaced and `Ending tuning test mode` is reported.
//!
//! # What is not here
//!
//! - **The parameter command runs detached.** Upstream runs it synchronously
//!   inside `move()` (`tuning_tower.py:89-90`) so the boundary move is already
//!   planned with the new value. `GCodeDispatch::run_script_from_command` is
//!   `async` here while a move transform is not, so the line is run on the
//!   host's runtime (`tokio::spawn`, the pattern of `delayed_gcode` and
//!   `idle_timeout`) and the first move of a band may still see the previous
//!   value. A host with no runtime to spawn on logs a warning instead.
//! - **`exclude_object`'s tuning-tower guard is not wired.** Upstream refuses
//!   to register the exclusion transform while a tower is active
//!   (`exclude_object.py:38-42`); [`TuningTower::is_active`] is the answer that
//!   guard asks for, and `exclude_object.rs`'s module docs still record the
//!   gap.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_move::{self, GCodeMove, MoveTarget};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    is_traditional_gcode, parse_float, sync, CommandError, CommandHandler, GCodeDispatch,
    GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, E_AXIS, Z_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// A move this far below the last one is the start of another print
/// (`tuning_tower.py:6`).
const CANCEL_Z_DELTA: f64 = 2.0;

/// The Z the first extrusion move has to clear (`tuning_tower.py:48`).
const NEVER_Z: f64 = -99999999.9;

/// `TUNING_TOWER`'s help text, verbatim (`tuning_tower.py:26`).
const TUNING_TOWER_HELP: &str = "Tool to adjust a parameter at each Z height";

/// The words `TUNING_TOWER` reads, in upstream's order
/// (`tuning_tower.py:29-38`).
const TUNING_TOWER_PARAMS: &[&str] = &[
    "COMMAND",
    "PARAMETER",
    "START",
    "FACTOR",
    "BAND",
    "STEP_DELTA",
    "STEP_HEIGHT",
    "SKIP",
];

section!("tuning_tower", order = 30, load = load_config);

/// What one move asks the tower to do besides forwarding it
/// (`tuning_tower.py:71-92`).
enum Step {
    /// Nothing to do: not an extrusion move at a new Z, or the value did not
    /// change.
    Nothing,
    /// Run this parameter command.
    Command(String),
    /// The move is at a Z far below the last one: end the test.
    End,
}

/// The test state, behind one lock the move transform and the command share
/// (`tuning_tower.py:15-22`).
struct TowerState {
    /// Whether a test is running — upstream's `normal_transform is not None`
    /// (`tuning_tower.py:97-98`).
    active: bool,
    /// The transform the tower displaced from `gcode_move`'s slot (upstream
    /// `normal_transform`); `None` when the slot was empty.
    displaced: Option<Arc<dyn MoveTarget>>,
    /// The toolhead captured at `klippy:connect`, the chain's end when the
    /// slot was empty (`tuning_tower.py:18-19`).
    toolhead: Option<Arc<dyn MoveTarget>>,
    /// The position of the last move or query, toolhead coordinates
    /// (`tuning_tower.py:17`).
    last_position: Coord,
    /// The Z of the last extrusion move (`tuning_tower.py:18`).
    last_z: f64,
    start: f64,
    factor: f64,
    band: f64,
    step_delta: f64,
    step_height: f64,
    skip: f64,
    /// The value last handed to the command (`tuning_tower.py:20`).
    last_command_value: Option<f64>,
    /// Upstream's `command_fmt` without the `%.9f`
    /// (`tuning_tower.py:41-44`).
    command_prefix: String,
}

impl Default for TowerState {
    fn default() -> Self {
        Self {
            active: false,
            displaced: None,
            toolhead: None,
            last_position: Coord::new(0.0, 0.0, 0.0, 0.0),
            last_z: 0.0,
            start: 0.0,
            factor: 0.0,
            band: 0.0,
            step_delta: 0.0,
            step_height: 0.0,
            skip: 0.0,
            last_command_value: None,
            command_prefix: String::new(),
        }
    }
}

impl TowerState {
    /// The transform below this one: the displaced occupant once a test is
    /// running, the connect-time toolhead otherwise.
    fn chain(&self) -> Option<Arc<dyn MoveTarget>> {
        self.displaced.clone().or_else(|| self.toolhead.clone())
    }

    /// Upstream's `calc_value` (`tuning_tower.py:63-69`): the parameter value
    /// for a g-code Z.
    fn calc_value(&self, z: f64) -> f64 {
        let z = if self.skip != 0.0 {
            (z - self.skip).max(0.0)
        } else {
            z
        };
        if self.step_height != 0.0 {
            return self.start + self.step_delta * (z / self.step_height).floor();
        }
        let z = if self.band != 0.0 {
            // The middle of the band: a step function, not a ramp.
            ((z / self.band).floor() + 0.5) * self.band
        } else {
            z
        };
        self.start + z * self.factor
    }

    /// Upstream's `move()` test (`tuning_tower.py:73-74`): an extrusion move
    /// (E grew) at a Z height the test has not seen, with the position moved.
    fn is_tuning_move(&self, newpos: &Coord) -> bool {
        newpos.axis(E_AXIS) > self.last_position.axis(E_AXIS)
            && newpos.axis(Z_AXIS) != self.last_z
            && (0..=Z_AXIS).any(|axis| newpos.axis(axis) != self.last_position.axis(axis))
    }

    /// Upstream's `move()` body (`tuning_tower.py:71-92`): decide what the
    /// move does to the test, updating `last_z`/`last_command_value`.
    ///
    /// `last_position` is left to the caller, which sets it after the decision
    /// (`tuning_tower.py:91`).
    fn plan_move(&mut self, newpos: &Coord, gcode_move: &GCodeMove) -> Step {
        if !self.is_tuning_move(newpos) {
            return Step::Nothing;
        }
        let z = newpos.axis(Z_AXIS);
        if z < self.last_z - CANCEL_Z_DELTA {
            // Extrude at a lower z height - probably start of new print
            return Step::End;
        }
        let gcode_z = gcode_position_z(gcode_move);
        let newval = self.calc_value(gcode_z);
        self.last_z = z;
        if self.last_command_value == Some(newval) {
            return Step::Nothing;
        }
        self.last_command_value = Some(newval);
        Step::Command(format!("{}{newval:.9}", self.command_prefix))
    }
}

/// Upstream's `self.gcode_move.get_status()['gcode_position'].z`
/// (`tuning_tower.py:80`).
fn gcode_position_z(gcode_move: &GCodeMove) -> f64 {
    gcode_move.status()["gcode_position"]
        .get(Z_AXIS)
        .and_then(Value::as_f64)
        .unwrap_or(0.0)
}

/// The `[tuning_tower]` section (`tuning_tower.py:TuningTower`).
pub struct TuningTower {
    printer: Weak<Printer>,
    gcode: Arc<GCodeDispatch>,
    gcode_move: Arc<GCodeMove>,
    state: Mutex<TowerState>,
}

impl std::fmt::Debug for TuningTower {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.lock();
        f.debug_struct("TuningTower")
            .field("active", &state.active)
            .field("last_z", &state.last_z)
            .field("last_command_value", &state.last_command_value)
            .finish()
    }
}

impl TuningTower {
    /// Read the section — upstream reads no options (`tuning_tower.py:14-23`),
    /// so the bare `[tuning_tower]` is all a config carries.
    fn new(
        _config: &ConfigWrapper,
        printer: &Arc<Printer>,
        gcode: Arc<GCodeDispatch>,
        gcode_move: Arc<GCodeMove>,
    ) -> Self {
        Self {
            printer: Arc::downgrade(printer),
            gcode,
            gcode_move,
            state: Mutex::new(TowerState::default()),
        }
    }

    /// The state lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock(&self) -> MutexGuard<'_, TowerState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Whether a test is running — upstream's `is_active`
    /// (`tuning_tower.py:97-98`), which `exclude_object` consults.
    pub fn is_active(&self) -> bool {
        self.lock().active
    }

    /// Upstream's `_handle_connect` (`tuning_tower.py:18-19` resolves the
    /// toolhead at `printer.lookup_object` time): capture the toolhead, the
    /// chain's end when `gcode_move`'s slot was still empty.
    fn handle_connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
            self.lock().toolhead = Some(Arc::new(ToolheadMove(toolhead)));
        }
    }

    /// Upstream's `end_test` (`tuning_tower.py:93-96`): hand the move-transform
    /// slot back to the transform the tower displaced and say so.
    fn end_test(&self) {
        let chain = self.lock().chain();
        self.gcode.respond_info("Ending tuning test mode", true);
        if let Some(chain) = chain {
            let _ = self.gcode_move.set_move_transform(chain, true);
        }
        let mut state = self.lock();
        state.active = false;
        state.displaced = None;
    }

    /// Run one emitted parameter line on the host's runtime (module docs: the
    /// move transform cannot await).
    fn run_command(&self, script: &str) {
        let gcode = Arc::clone(&self.gcode);
        let script = script.to_string();
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(error) = gcode.run_script_from_command(&script).await {
                        tracing::error!("tuning tower command execution: {error}");
                    }
                });
            }
            Err(_) => {
                tracing::warn!("tuning_tower: the parameter command needs a runtime to run");
            }
        }
    }

    /// Register `TUNING_TOWER` (`tuning_tower.py:23-25`).
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let handler: CommandHandler = {
            let object = Arc::clone(self);
            sync(move |gcmd| cmd_tuning_tower(&object, gcmd))
        };
        gcode
            .register_command_with_params(
                "TUNING_TOWER",
                handler,
                Some(TUNING_TOWER_HELP),
                TUNING_TOWER_PARAMS,
                false,
            )
            .map_err(ConfigError::new)
    }

    /// Upstream's `TUNING_TOWER` handler (`tuning_tower.py:27-61`).
    fn start_test(object: &Arc<TuningTower>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        if object.is_active() {
            object.end_test();
        }
        let command = gcmd.get_str("COMMAND")?;
        let parameter = gcmd.get_str("PARAMETER")?;
        // `gcmd.get_float('START', 0.)` and friends; the `minval=0.` bounds are
        // the three heights and steps (`tuning_tower.py:29-37`).
        let start = gcmd.get("START", Some(0.0), parse_float, None, None, None, None)?;
        let factor = gcmd.get("FACTOR", Some(0.0), parse_float, None, None, None, None)?;
        let band = gcmd.get("BAND", Some(0.0), parse_float, Some(0.0), None, None, None)?;
        let step_delta = gcmd.get("STEP_DELTA", Some(0.0), parse_float, None, None, None, None)?;
        let step_height = gcmd.get(
            "STEP_HEIGHT",
            Some(0.0),
            parse_float,
            Some(0.0),
            None,
            None,
            None,
        )?;
        let skip = gcmd.get("SKIP", Some(0.0), parse_float, Some(0.0), None, None, None)?;
        if factor != 0.0 && (step_height != 0.0 || step_delta != 0.0) {
            return Err(CommandError::new(
                "Cannot specify both FACTOR and STEP_DELTA/STEP_HEIGHT",
            ));
        }
        if (step_delta != 0.0) != (step_height != 0.0) {
            return Err(CommandError::new(
                "Must specify both STEP_DELTA and STEP_HEIGHT",
            ));
        }
        // `"%s %s%%.9f"` for a traditional command, `"%s %s=%%.9f"` for an
        // extended one (`tuning_tower.py:41-44`).
        let command_prefix = if is_traditional_gcode(&command) {
            format!("{command} {parameter}")
        } else {
            format!("{command} {parameter}=")
        };
        // Enable test mode: take the transform slot, keeping what it displaced.
        let displaced = object
            .gcode_move
            .set_move_transform(Arc::clone(object) as Arc<dyn MoveTarget>, true)
            .map_err(|error| CommandError::new(error.to_string()))?;
        {
            let mut state = object.lock();
            state.active = true;
            state.displaced = displaced;
            state.start = start;
            state.factor = factor;
            state.band = band;
            state.step_delta = step_delta;
            state.step_height = step_height;
            state.skip = skip;
            state.last_z = NEVER_Z;
            state.last_command_value = None;
            state.command_prefix = command_prefix;
        }
        // Upstream's `self.get_position()` (`tuning_tower.py:49`): anchor
        // `last_position` to where the chain below says the toolhead is.
        object.position();

        let mut message_parts = vec![format!("start={start:.6}")];
        if factor != 0.0 {
            message_parts.push(format!("factor={factor:.6}"));
            if band != 0.0 {
                message_parts.push(format!("band={band:.6}"));
            }
        } else {
            message_parts.push(format!("step_delta={step_delta:.6}"));
            message_parts.push(format!("step_height={step_height:.6}"));
        }
        if skip != 0.0 {
            message_parts.push(format!("skip={skip:.6}"));
        }
        gcmd.respond_info(&format!(
            "Starting tuning test ({})",
            message_parts.join(" ")
        ));
        Ok(())
    }
}

impl PrinterObject for TuningTower {
    /// Upstream defines no `get_status` for this object.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl MoveTarget for TuningTower {
    /// Upstream's `move` (`tuning_tower.py:71-92`): update the parameter on an
    /// extrusion move at a new Z, then forward the move down the chain.
    ///
    /// The chain below is captured before `end_test` can clear it, so the move
    /// that ends a test still reaches the transform the tower displaced.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let chain = self.lock().chain();
        let step = {
            let mut state = self.lock();
            let step = state.plan_move(&position, &self.gcode_move);
            state.last_position = position;
            step
        };
        match step {
            Step::Command(script) => self.run_command(&script),
            Step::End => self.end_test(),
            Step::Nothing => {}
        }
        match chain {
            Some(chain) => chain.move_to(position, speed),
            // No toolhead yet (`klippy:connect` has not run): nothing forwards.
            None => Ok(()),
        }
    }

    /// Upstream's `get_position` (`tuning_tower.py:57-60`): the chain below's
    /// position, which is also what the test tracks as `last_position`.
    fn position(&self) -> Coord {
        let Some(chain) = self.lock().chain() else {
            return Coord::new(0.0, 0.0, 0.0, 0.0);
        };
        let position = chain.position();
        self.lock().last_position = position;
        position
    }
}

/// The toolhead as the transform chain's end, the way `gcode_move`'s own
/// `ToolheadTarget` wraps it (`gcode_move.rs`).
struct ToolheadMove(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadMove {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn position(&self) -> Coord {
        self.0.position().unwrap_or_default()
    }
}

/// The factory `section!` names (`tuning_tower.py:100-101`).
///
/// `gcode_move::ensure` stands in for upstream's
/// `printer.load_object(config, 'gcode_move')` (`tuning_tower.py:19`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .ok_or_else(|| ConfigError::new("the gcode dispatcher is not registered".to_string()))?;
    let gcode_move = gcode_move::ensure(printer)?;
    let object = Arc::new(TuningTower::new(config, printer, gcode, gcode_move));
    object.register_commands(printer)?;
    printer.register_event_handler(
        KlippyEvent::KlippyConnect,
        Box::new({
            let object = Arc::clone(&object);
            move |_| object.handle_connect()
        }),
    );
    Ok(object)
}

/// `TUNING_TOWER` (`tuning_tower.py:27-61`).
fn cmd_tuning_tower(object: &Arc<TuningTower>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    TuningTower::start_test(object, gcmd)
}
