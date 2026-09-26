//! `[stepper_x]` / `[stepper_y]` / `[stepper_z]` — one motor on one axis.
//!
//! Upstream builds these from `klippy/stepper.py`: `PrinterStepper` looks up the
//! step and direction pins, `MCU_stepper` owns the oid and the wire commands, and
//! `GenericPrinterRail` adds the axis range the kinematics needs. A stepper
//! section is **not** a printer object there (`objects/list` does not show
//! `stepper_x`); the toolhead reads the sections straight from the config.
//!
//! This port's loader needs every section claimed, so the same piece of work is
//! split across two layers:
//!
//! * this module is the `[stepper_*]` section: it parses the motor geometry,
//!   builds the [`McuStepper`] resource (which registers `config_stepper`), and
//!   exposes the rail range for the kinematics. The object is registered but
//!   [`PrinterObject::is_queryable`] is false, so `objects/list` still leaves it
//!   out, as upstream does;
//! * [`ToolHeadObject`](crate::core::klippy::extras::toolhead) takes the
//!   host-side [`Stepper`] this module builds at connect and drives it.
//!
//! # What is here
//!
//! | option | meaning |
//! |---|---|
//! | `step_pin` | the step pin (required) |
//! | `dir_pin` | the direction pin, same MCU as the step pin (required) |
//! | `rotation_distance` | millimetres per full motor rotation (required unless `gear_ratio` implies radians, see below) |
//! | `microsteps` | microsteps per full step (required) |
//! | `full_steps_per_rotation` | full steps per rotation (default 200) |
//! | `gear_ratio` | `g1:g2` pairs multiplied into the step distance |
//! | `step_pulse_duration` | step pulse width in seconds (default 2 µs) |
//! | `position_min` / `position_max` | the axis range (required) |
//! | `position_endstop` | where the endstop sits (stored for FW6 homing) |
//!
//! Homing is wired: `endstop_pin` arms a firmware endstop through
//! `trsync`/`stepper_stop_on_trigger`, the options below feed `HomingInfo`, and
//! `G28` drives the move (see `extras/toolhead.rs`). What is still open is the
//! **second pass** — `homing_retract_dist` / `second_homing_speed` are read
//! into `HomingInfo` but the retract + re-approach is not driven yet — and
//! `endstop_phase` refinement (H9).
//!
//! # Polar's two sections
//!
//! `kinematics: polar` (`klippy/kinematics/polar.py`) reads two sections of
//! its own: `[stepper_arm]` is a normal rail (the arm homes toolhead X with Y
//! pinned to 0) and `[stepper_bed]` is a **bare** stepper — no rail geometry
//! (`PrinterStepper(config, units_in_radians=True)`, `polar.py:26-27`): no
//! `position_*` options, and its step distance is in **radians** when
//! `rotation_distance` is absent but `gear_ratio` is present (upstream's own
//! inference in `parse_step_distance`, `stepper.py:302-304`).

use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::Duration;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::extras::stepper_enable::PrinterStepperEnable;
use crate::core::klippy::extras::toolhead::HomingEndstop;
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::mcu::McuStepper;
use crate::core::klippy::motion::itersolve::{
    cartesian_active_flags, cartesian_position_fn, AxisFlags, PositionFn, PositionPost,
    PositionUnwrap,
};
use crate::core::klippy::motion::{Axis, HomingInfo, Stepper};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

// The three cartesian axes. Aligned with upstream: these are **not** standalone
// printer objects — the toolhead (late, order=60) builds Rail + endstop from
// the config via `Rail::lookup`, which reads `PrinterStepper` registered here.
section!("stepper_x", order = 50, phase = late, load = load_config);
section!("stepper_y", order = 50, phase = late, load = load_config);
section!("stepper_z", order = 50, phase = late, load = load_config);
// The polar sections (`kinematics/polar.py:26-28`): the arm is a rail that
// homes toolhead X (so it carries the rail geometry), the bed is a bare
// radians-mode stepper with no geometry. Same phase/order as the cartesian
// steppers: both must exist before `[printer]` (order=60) claims them.
section!(
    "stepper_arm",
    order = 50,
    phase = late,
    load = load_config_arm
);
section!(
    "stepper_bed",
    order = 50,
    phase = late,
    load = load_config_bed
);
// The letter sections `stepper_a`…`stepper_z`. Upstream claims them two
// ways and here they need factories of their own either way, so the
// undefined-option check accepts the sections: the delta towers are
// `config.getsection('stepper_' + a) for a in 'abc'` (`kinematics/delta.py:15`)
// and the winch anchors are `config.getsection('stepper_' + chr(a + i))
// for i in range(26)` (`kinematics/winch.py:14-17`). The winch factory reads
// them as bare anchor motors (no `position_endstop`) when `[printer]` says
// `kinematics: winch`; see `load_config`.
section!("stepper_a", order = 50, phase = late, load = load_config);
section!("stepper_b", order = 50, phase = late, load = load_config);
section!("stepper_c", order = 50, phase = late, load = load_config);
section!("stepper_d", order = 50, phase = late, load = load_config);
section!("stepper_e", order = 50, phase = late, load = load_config);
section!("stepper_f", order = 50, phase = late, load = load_config);
section!("stepper_g", order = 50, phase = late, load = load_config);
section!("stepper_h", order = 50, phase = late, load = load_config);
section!("stepper_i", order = 50, phase = late, load = load_config);
section!("stepper_j", order = 50, phase = late, load = load_config);
section!("stepper_k", order = 50, phase = late, load = load_config);
section!("stepper_l", order = 50, phase = late, load = load_config);
section!("stepper_m", order = 50, phase = late, load = load_config);
section!("stepper_n", order = 50, phase = late, load = load_config);
section!("stepper_o", order = 50, phase = late, load = load_config);
section!("stepper_p", order = 50, phase = late, load = load_config);
section!("stepper_q", order = 50, phase = late, load = load_config);
section!("stepper_r", order = 50, phase = late, load = load_config);
section!("stepper_s", order = 50, phase = late, load = load_config);
section!("stepper_t", order = 50, phase = late, load = load_config);
section!("stepper_u", order = 50, phase = late, load = load_config);
section!("stepper_v", order = 50, phase = late, load = load_config);
section!("stepper_w", order = 50, phase = late, load = load_config);
// The deltesian sections (`kinematics/deltesian.py:15-17`): two arm rails (no
// `position_max`; `stepper_right` inherits `stepper_left`'s endstop) and the
// straight Y rail. Same phase/order as the other steppers: all must exist
// before `[printer]` (order=60) claims them.
section!("stepper_left", order = 50, phase = late, load = load_config);
section!(
    "stepper_right",
    order = 50,
    phase = late,
    load = load_config
);

/// The default pulse width upstream uses when the option is absent
/// (`klippy/stepper.py:80`).
const DEFAULT_STEP_PULSE_DURATION: f64 = 0.000_002;

/// How long the connect-time position read may take.
///
/// The same order as the other connect-time reads; a board that does not answer
/// only loses the alignment, not the connection.
const POSITION_TIMEOUT: Duration = Duration::from_secs(1);

/// Which rail geometry a section carries — upstream's
/// `GenericPrinterRail(config, need_position_minmax, default_position_endstop)`
/// arguments (`klippy/stepper.py:327-354`).
#[derive(Debug, Clone, Copy)]
pub enum RailGeometry {
    /// `[stepper_x/y/z]`: a full axis — `position_min`/`position_max` are
    /// read, and `position_endstop` defaults to the minimum
    /// (`need_position_minmax=True`).
    Axis,
    /// `[stepper_a/b/c]`: a delta tower — there is no `position_max` option;
    /// the range runs `0..position_endstop` (`stepper.py:352-354`) and
    /// `position_endstop` falls back to `default_position_endstop`
    /// (`stepper_b/c` inherit `stepper_a`'s, `delta.py:16-22`).
    DeltaTower {
        /// The endstop to fall back on when the section omits its own.
        default_position_endstop: Option<f64>,
    },
    /// A numbered sibling (`[stepper_z1]`): a bare motor with no rail
    /// geometry (`LookupMultiRail`'s extra stepper). The winch kinematics'
    /// `[stepper_a]`…`[stepper_z]` anchors use the same geometry
    /// (`kinematics/winch.py:18`, a bare `PrinterStepper` with no rail range).
    BareMotor,
}

/// One `[stepper_*]` section's rail parameters.
///
/// The subset of upstream's `GenericPrinterRail` the cartesian kinematics needs
/// today: the travel limits. The homing fields (`position_endstop` and the
/// speeds) are parsed and kept for FW6.
#[derive(Debug, Clone, Copy, Default)]
pub struct RailParams {
    /// Minimum axis position.
    pub position_min: f64,
    /// Maximum axis position.
    pub position_max: f64,
    /// Where the endstop trips, used by homing (FW6).
    pub position_endstop: f64,
}

/// The solver a stepper's owning rail installs
/// (`MCU_stepper.setup_itersolve`, `klippy/stepper.py:74`): the position
/// function the solver evaluates and the axes it moves.
#[derive(Debug, Clone, Copy)]
pub struct SolverSpec {
    /// Where the stepper is `t` seconds into a segment, in millimetres.
    pub position: PositionFn,
    /// Which of the trapq's axes this stepper follows.
    pub active_flags: AxisFlags,
}

/// The solver hooks a stepper's position function needs beyond the raw
/// evaluation (`StepKinematics::set_hooks`): only polar's bed angle solver
/// has them (`kin_polar.c`'s `post_cb` + the `commanded_pos` unwrap).
#[derive(Debug, Clone, Copy)]
pub struct SolverHooks {
    /// Correct a raw position-fn result against `commanded_pos`.
    pub unwrap: PositionUnwrap,
    /// Renormalize `commanded_pos` after each generated range.
    pub post: PositionPost,
}

/// One configured `[stepper_x]` / `[stepper_y]` / `[stepper_z]`.
pub struct PrinterStepper {
    name: String,
    axis: Axis,
    /// Microsteps per full step, as configured (`microsteps`).
    microsteps: i64,
    /// Millimetres per step, after rotation distance, microsteps and gearing.
    step_dist: f64,
    /// The rotation distance the section wrote, and the full steps that make
    /// one rotation (`get_rotation_distance`, `klippy/stepper.py:136-137`).
    ///
    /// `SET_EXTRUDER_ROTATION_DISTANCE` rewrites the distance; the host solver
    /// is not rebuilt here, so the new value is recorded for reporting (the H10
    /// motion-sync gap the extruder sections document).
    rotation_distance: Mutex<f64>,
    steps_per_rotation: f64,
    /// Whether the direction pin was written with `!` (`orig_dir_inverted`,
    /// `klippy/stepper.py:42`), and the runtime flag `set_dir_inverted` flips.
    orig_dir_inverted: bool,
    dir_inverted: Mutex<bool>,
    /// The range and homing point the kinematics reads.
    params: RailParams,
    /// The firmware side: oid, pins and the wire commands.
    mcu_stepper: Arc<McuStepper>,
    /// The endstop this rail homes to, when the section names one (a pin
    /// `McuEndstop` or the eddy probe's `McuTriggerAnalog`, both as
    /// [`HomingEndstop`]).
    endstop: Option<Arc<dyn HomingEndstop>>,
    /// The homing parameters (`homing.py`'s input).
    homing: HomingInfo,
    /// The machine, to find this MCU's clock/offset at connect. `Weak` because
    /// the printer's registry owns this object.
    printer: Weak<Printer>,
    /// The host solver and compressor, built at connect when the oid and the MCU
    /// frequency exist.
    ///
    /// The toolhead's connect takes it out and owns it from then on, so this is
    /// `None` after the machine is up.
    inner: Mutex<Option<Stepper>>,
    /// The solver function the owning rail installed at load
    /// (`setup_itersolve`). The stepper no longer decides this from its name:
    /// an extruder or a delta stepper reads the same trapq differently.
    solver: Mutex<Option<SolverSpec>>,
    /// The solver hooks the owner installed at load (`setup_hooks`); applied
    /// to the host solver at connect. `None` for every solver upstream runs
    /// without a `post_cb` (cartesian, corexy, extruder, polar's arm/z).
    hooks: Mutex<Option<SolverHooks>>,
}

impl PrinterStepper {
    /// Build the section: parse the motor geometry and register the firmware
    /// stepper.
    ///
    /// `geometry` is the [`RailGeometry`] shorthand the cartesian sections and
    /// bare siblings use; delta towers are built through
    /// [`PrinterStepper::with_geometry`].
    ///
    /// # Errors
    /// Returns a config error naming the section when an option is missing,
    /// malformed, out of range, or names a pin the `pins` layer refuses.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        axis: Axis,
        geometry: bool,
    ) -> Result<Self, ConfigError> {
        let geometry = if geometry {
            RailGeometry::Axis
        } else {
            RailGeometry::BareMotor
        };
        Self::with_geometry(config, printer, axis, geometry)
    }

    /// [`PrinterStepper::new`] with the section's explicit [`RailGeometry`].
    ///
    /// # Errors
    /// As [`PrinterStepper::new`].
    pub fn with_geometry(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        axis: Axis,
        geometry: RailGeometry,
    ) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        // The section identifier, not the bare id: a generic-cartesian motor is
        // `[stepper <name>]`, so its id alone (`stepper`) names no single
        // motor (`config.get_name()`, `klippy/stepper.py:60-64`).
        let name = identifier.clone();

        let step_pin = config.get("step_pin", None)?;
        let dir_pin = config.get("dir_pin", None)?;
        // `rotation_distance` is parsed below, next to the step-distance
        // math: it is optional when `gear_ratio` implies radians mode.
        let microsteps = config.get_int_bounded("microsteps", None, Some(1), None)?;
        let full_steps = config.get_int("full_steps_per_rotation", Some(200))?;
        if full_steps < 1 || full_steps % 4 != 0 {
            return Err(ConfigError::new(format!(
                "full_steps_per_rotation invalid in section '{identifier}'"
            )));
        }
        let gear_ratio = config
            .get_list_of_lists("gear_ratio", ',', ':', 2)?
            .into_iter()
            .map(|pair| {
                let first = pair[0].trim().parse::<f64>().map_err(|_| {
                    ConfigError::new(format!(
                        "Unable to parse option 'gear_ratio' in section '{identifier}'"
                    ))
                })?;
                let second = pair[1].trim().parse::<f64>().map_err(|_| {
                    ConfigError::new(format!(
                        "Unable to parse option 'gear_ratio' in section '{identifier}'"
                    ))
                })?;
                if second == 0.0 {
                    return Err(ConfigError::new(format!(
                        "Option 'gear_ratio' in section '{identifier}' must not divide by zero"
                    )));
                }
                Ok(first / second)
            })
            .collect::<Result<Vec<f64>, ConfigError>>()?
            .into_iter()
            .product::<f64>()
            .max(f64::MIN_POSITIVE);
        let step_pulse_duration = config.get_float_bounded(
            "step_pulse_duration",
            Some(DEFAULT_STEP_PULSE_DURATION),
            Some(0.0),
            Some(0.001),
            None,
            None,
        )?;

        // `rotation_distance` is millimetres per full rotation; the divisor is
        // full steps times microsteps times any gearing
        // (`parse_step_distance`, `klippy/stepper.py:307-323`).
        //
        // Radians mode (upstream's own inference when the caller does not say,
        // `stepper.py:302-304`: no `rotation_distance` but a `gear_ratio`):
        // the rotation is one turn in radians, so the step distance comes out
        // in radians — `[stepper_bed]` of a polar printer is exactly this
        // (`polar.py:26` passes `units_in_radians=True`). A section with
        // neither option still fails on `rotation_distance`, as upstream does.
        let rotation_distance = if !config.has("rotation_distance") && config.has("gear_ratio") {
            std::f64::consts::TAU
        } else {
            config.get_float_bounded("rotation_distance", None, None, None, Some(0.0), None)?
        };
        let steps_per_rotation = full_steps as f64 * microsteps as f64 * gear_ratio;
        let step_dist = rotation_distance / steps_per_rotation;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        // The endstop is built first: when it is a virtual one (the probe's
        // `z_virtual_endstop`), it supplies `position_endstop`, and upstream
        // prefers that over the section's option
        // (`klippy/stepper.py:336-343`, `MCU_endstop.get_position_endstop`).
        let endstop = match config.get_str("endstop_pin") {
            Some(pin) => Some(
                pins.setup_endstop_dyn(&pin, None)
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            ),
            None => None,
        };

        // The rail geometry (`position_min/max`, `position_endstop`, the homing
        // speeds) belongs to the rail's **primary** section. A numbered sibling
        // (`[stepper_z1]`) is a bare motor: `LookupMultiRail` adds it to the
        // primary's rail without reading any of this
        // (`klippy/stepper.py:327-360`, `:455-462`).
        let (params, homing) = match geometry {
            RailGeometry::Axis => {
                let position_min = config.get_float("position_min", Some(0.0))?;
                let position_max = config.get_float_bounded(
                    "position_max",
                    None,
                    None,
                    None,
                    Some(position_min),
                    None,
                )?;
                let position_endstop = match endstop
                    .as_ref()
                    .and_then(|endstop| pins.virtual_endstop_position(endstop))
                {
                    Some(virtual_position) => virtual_position,
                    None => config.get_float("position_endstop", Some(position_min))?,
                };
                if position_endstop < position_min || position_endstop > position_max {
                    return Err(ConfigError::new(format!(
                        "position_endstop in section '{identifier}' must be between position_min and position_max"
                    )));
                }
                let homing = read_homing_info(
                    config,
                    &identifier,
                    position_min,
                    position_max,
                    position_endstop,
                )?;
                (
                    RailParams {
                        position_min,
                        position_max,
                        position_endstop,
                    },
                    homing,
                )
            }
            RailGeometry::DeltaTower {
                default_position_endstop,
            } => {
                // A delta tower has no `position_max` option: the range runs
                // from zero to the endstop (`stepper.py:348-354`), which is
                // also what leaves `homing_positive_dir` inferable (the
                // endstop sits at the top of that range).
                let position_min = 0.0;
                let position_endstop = match endstop
                    .as_ref()
                    .and_then(|endstop| pins.virtual_endstop_position(endstop))
                {
                    Some(virtual_position) => virtual_position,
                    None => config.get_float("position_endstop", default_position_endstop)?,
                };
                let position_max = position_endstop;
                if position_endstop < position_min || position_endstop > position_max {
                    return Err(ConfigError::new(format!(
                        "position_endstop in section '{identifier}' must be between position_min and position_max"
                    )));
                }
                let homing = read_homing_info(
                    config,
                    &identifier,
                    position_min,
                    position_max,
                    position_endstop,
                )?;
                (
                    RailParams {
                        position_min,
                        position_max,
                        position_endstop,
                    },
                    homing,
                )
            }
            RailGeometry::BareMotor => (RailParams::default(), HomingInfo::default()),
        };

        // The step pin's `!` is upstream's `invert_step` (`0`/`1`); the direction
        // pin's `!` is applied on the wire by the resource.
        let mcu_stepper = pins
            .setup_stepper(&step_pin, &dir_pin, step_pulse_duration)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        // Register the stepper with the endstop's trigger dispatch now, at load:
        // the dispatch creates a per-MCU trsync (and the config callback that
        // reserves its oid) before the configuration is built. This is also
        // where upstream rejects a shared axis whose steppers are on different
        // MCUs (`TriggerDispatch.add_stepper`).
        if let Some(endstop) = &endstop {
            let dispatch = endstop.dispatch().ok_or_else(|| {
                ConfigError::new(format!(
                    "{identifier}: the rail's endstop must drive a trigger dispatch"
                ))
            })?;
            dispatch
                .add_stepper(
                    mcu_stepper.chip().clone(),
                    Arc::downgrade(&mcu_stepper),
                    &name,
                )
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        }

        // Register with stepper_enable. Upstream's `PrinterStepper` loads
        // `stepper_enable` for every stepper (`klippy/stepper.py:282-285`), so a
        // config that only writes `enable_pin` still gets the object and its
        // M18/M84 commands; `ensure` creates it on first use.
        let stepper_enable = PrinterStepperEnable::ensure(printer);
        stepper_enable.register_stepper(config, &name)?;

        Ok(Self {
            name,
            axis,
            microsteps,
            step_dist,
            rotation_distance: Mutex::new(rotation_distance),
            steps_per_rotation,
            orig_dir_inverted: mcu_stepper.invert_dir(),
            dir_inverted: Mutex::new(mcu_stepper.invert_dir()),
            params,
            mcu_stepper,
            endstop,
            homing,
            printer: Arc::downgrade(printer),
            inner: Mutex::new(None),
            solver: Mutex::new(None),
            hooks: Mutex::new(None),
        })
    }

    /// The section's name (`stepper_x`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The axis this stepper drives.
    pub fn axis(&self) -> Axis {
        self.axis
    }

    /// Install the solver the owning rail wants this stepper to run
    /// (`MCU_stepper.setup_itersolve`, `klippy/stepper.py:74`).
    ///
    /// Called at load by the rail / kinematics that owns the stepper. Without
    /// it [`PrinterStepper::connect`] falls back to the cartesian axis its name
    /// implies, which keeps a standalone `[stepper_x]` working.
    pub fn setup_itersolve(&self, position: PositionFn, active_flags: AxisFlags) {
        *self
            .solver
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(SolverSpec {
            position,
            active_flags,
        });
    }

    /// Install the solver's position hooks (`StepKinematics::set_hooks`), to
    /// be applied to the host solver at connect.
    ///
    /// Only `kin_polar.c`'s angle solver has hooks: the ±2π unwrap against
    /// `commanded_pos` and the renormalization after each generated range
    /// (upstream's callback reads `sk->commanded_pos` and its `post_cb`).
    pub fn setup_hooks(&self, unwrap: PositionUnwrap, post: PositionPost) {
        *self
            .hooks
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(SolverHooks { unwrap, post });
    }

    /// Millimetres per step.
    pub fn step_dist(&self) -> f64 {
        self.step_dist
    }

    /// The rotation distance and the steps that make one rotation
    /// (upstream `get_rotation_distance`, `klippy/stepper.py:136-137`).
    pub fn get_rotation_distance(&self) -> (f64, f64) {
        (
            *self
                .rotation_distance
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()),
            self.steps_per_rotation,
        )
    }

    /// Upstream's `set_rotation_distance` (`klippy/stepper.py:138-143`).
    ///
    /// The distance is recorded; the host solver and `step_dist` are not
    /// rebuilt, so motion does not yet follow the change (the H10 gap).
    pub fn set_rotation_distance(&self, rotation_distance: f64) {
        *self
            .rotation_distance
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = rotation_distance;
    }

    /// `(dir_inverted, orig_dir_inverted)` (upstream `get_dir_inverted`,
    /// `klippy/stepper.py:144-145`).
    pub fn get_dir_inverted(&self) -> (bool, bool) {
        (
            *self
                .dir_inverted
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()),
            self.orig_dir_inverted,
        )
    }

    /// Upstream's `set_dir_inverted` (`klippy/stepper.py:146-153`); the runtime
    /// flag is recorded (the step generator is not rewritten, the H10 gap).
    pub fn set_dir_inverted(&self, invert_dir: bool) {
        *self
            .dir_inverted
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = invert_dir;
    }

    /// Microsteps per full step, as configured (upstream's
    /// `config.getint("microsteps")`). A phase count is `microsteps × 4`
    /// (`endstop_phase.py:58`).
    pub fn microsteps(&self) -> i64 {
        self.microsteps
    }

    /// The rail range and homing point.
    pub fn params(&self) -> RailParams {
        self.params
    }

    /// The endstop this rail homes to, if the section named one.
    pub fn endstop(&self) -> Option<&Arc<dyn HomingEndstop>> {
        self.endstop.as_ref()
    }

    /// The homing parameters (`homing.py`'s input).
    pub fn homing_info(&self) -> HomingInfo {
        self.homing
    }

    /// The firmware stepper resource.
    pub fn mcu_stepper(&self) -> &Arc<McuStepper> {
        &self.mcu_stepper
    }

    /// Take the host-side stepper the toolhead will drive.
    ///
    /// `None` before connect, or after the toolhead has taken it. The toolhead
    /// calls this once, in its own connect.
    pub fn take_stepper(&self) -> Option<Stepper> {
        self.lock().take()
    }

    /// This MCU's print-time-to-clock offset, found through the chip name.
    ///
    /// Zero when the MCU object cannot be found (a standalone stepper with no
    /// `[mcu]` object), which is the primary's offset anyway.
    /// This MCU's print-time `(offset, frequency)` mapping, found through the
    /// chip name.
    ///
    /// `None` when the MCU object cannot be found (a standalone stepper with no
    /// `[mcu]` object); the caller then uses `(0.0, mcu_freq)`.
    fn time_mapping(&self, mcu: &Arc<crate::core::klippy::mcu::Mcu>) -> Option<(f64, f64)> {
        self.printer.upgrade().and_then(|printer| {
            printer
                .lookup_objects_as::<crate::core::klippy::mcu::McuObject>(Some("mcu"))
                .into_iter()
                .find(|(_, object)| object.name() == mcu.name())
                .map(|(_, object)| object.time_mapping())
        })
    }

    fn lock(&self) -> MutexGuard<'_, Option<Stepper>> {
        self.inner
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for PrinterStepper {
    /// Never called through the API: a stepper section is not a printer object
    /// upstream, so it is not queryable here either.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            let config_error = |message: String| {
                KlippyError::Config(ConfigError::new(format!("{}: {message}", self.name)))
            };
            let mcu = self
                .mcu_stepper
                .mcu()
                .ok_or_else(|| config_error("MCU is not connected".to_string()))?;
            let freq = mcu
                .clock_freq()
                .map_err(|err| config_error(err.to_string()))?;
            let oid = self
                .mcu_stepper
                .oid()
                .map_err(|err| config_error(err.to_string()))?;

            let solver = *self
                .solver
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .as_ref()
                .unwrap_or(&SolverSpec {
                    position: cartesian_position_fn(self.axis),
                    active_flags: cartesian_active_flags(self.axis),
                });
            let mut stepper = Stepper::new(
                self.name.clone(),
                u32::from(oid),
                self.step_dist,
                solver.position,
                solver.active_flags,
                freq,
            );
            // Apply the owner's hooks (polar's bed angle solver) to the host
            // solver, mirroring upstream where `setup_itersolve` and the
            // callback/`post_cb` are installed together.
            if let Some(hooks) = *self
                .hooks
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
            {
                stepper
                    .kinematics_mut()
                    .set_hooks(Some(hooks.unwrap), Some(hooks.post));
            }
            // Read the board's step counter and align the solver with it, as
            // upstream's `_query_mcu_position` does at connect
            // (`klippy/stepper.py:212-228`). This is what makes the host's
            // position agree with the firmware's after a restart or a reset.
            let steps = self
                .mcu_stepper
                .query_position(POSITION_TIMEOUT)
                .await
                .map_err(|err| config_error(err.to_string()))?;
            stepper.kinematics_mut().commanded_pos = f64::from(steps) * self.step_dist;
            // Point the compressor at this MCU's clock domain. The `[mcu]`
            // object already built the estimate and the `SecondarySync` offset at
            // its own connect (which runs before any stepper); find it by chip
            // name and use it. A secondary's mapping is recalibrated later, and
            // the toolhead's flush loop re-reads it before generating.
            let (offset, mapping_freq) = self.time_mapping(&mcu).unwrap_or((0.0, freq));
            stepper.compressor_mut().set_time(offset, mapping_freq);
            // Record where the firmware's counter is (`set_last_position` only
            // flushes the pending step — none yet — and records the position).
            if let Some(clock) = mcu.estimated_clock() {
                stepper
                    .compressor_mut()
                    .set_last_position(clock, i64::from(steps))
                    .map_err(|err| config_error(err.to_string()))?;
            }

            *self.lock() = Some(stepper);
            Ok(())
        })
    }
}

impl std::fmt::Debug for PrinterStepper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterStepper")
            .field("name", &self.name)
            .field("step_dist", &self.step_dist)
            .finish_non_exhaustive()
    }
}

/// One axis' rail: its steppers, and the range/homing info the kinematics
/// reads.
///
/// Upstream's `GenericPrinterRail` (`klippy/stepper.py:326`) as a multi-stepper
/// group (`LookupMultiRail`, `:455`). The primary section (`[stepper_z]`) has a
/// factory of its own; its numbered siblings (`[stepper_z1]`, `[stepper_z2]`…)
/// do not, so they are read here through the primary's wrapper and registered as
/// steppers too — their options are recorded as they are read, which is what
/// makes a section with no factory valid to the undefined-option check, exactly
/// as upstream's `config.getsection` does.
///
/// The range and homing info come from the **primary** stepper, as upstream's
/// `get_range`/`get_homing_info` read the rail's own (`[stepper_z]`) values.
/// Homing all steppers of a multi-stepper rail from one endstop is the
/// `endstop_phase`/trsync refinement (H9); this group only drives them.
pub struct Rail {
    /// The base section name (`stepper_z`).
    name: String,
    /// The primary first, then `stepper_z1`, `stepper_z2`… in order.
    steppers: Vec<Arc<PrinterStepper>>,
}

impl Rail {
    /// Build the rail named by `base_id`, reading numbered siblings until one is
    /// missing (`LookupMultiRail`).
    ///
    /// `config` is the primary section's wrapper; it carries the whole config,
    /// so the siblings can be read. Each sibling is built with the same `axis`
    /// as the primary (`stepper_z1` is still the Z axis).
    ///
    /// # Errors
    /// Returns the primary's config error, or the first sibling's.
    pub fn lookup(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        base_id: &str,
        axis: Axis,
    ) -> Result<Arc<Self>, ConfigError> {
        let primary = printer
            .lookup_object_as::<PrinterStepper>(base_id)
            .ok_or_else(|| {
                ConfigError::new(format!(
                    "Section '{}' needs a '[{base_id}]' section",
                    config.identifier()
                ))
            })?;
        let mut steppers = vec![primary];
        for index in 1..99 {
            let identifier = format!("{base_id}{index}");
            let Some(sibling) = config.sibling(&identifier) else {
                break;
            };
            let stepper = Arc::new(PrinterStepper::new(&sibling, printer, axis, false)?);
            printer.add_object(&identifier, stepper.clone())?;
            steppers.push(stepper);
        }
        Ok(Arc::new(Self {
            name: base_id.to_string(),
            steppers,
        }))
    }

    /// The base section name (`stepper_z`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The first (primary) stepper; its values are the rail's.
    pub fn primary(&self) -> &Arc<PrinterStepper> {
        &self.steppers[0]
    }

    /// Every stepper on the rail, primary first.
    pub fn steppers(&self) -> &[Arc<PrinterStepper>] {
        &self.steppers
    }

    /// The primary's travel range.
    pub fn params(&self) -> RailParams {
        self.primary().params()
    }

    /// The primary's homing parameters.
    pub fn homing_info(&self) -> HomingInfo {
        self.primary().homing_info()
    }

    /// The endstop this rail homes to, when the section names one.
    pub fn endstop(&self) -> Option<&Arc<dyn HomingEndstop>> {
        self.primary().endstop()
    }

    /// The primary's millimetres per step.
    pub fn step_dist(&self) -> f64 {
        self.primary().step_dist()
    }
}

impl std::fmt::Debug for Rail {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Rail")
            .field("name", &self.name)
            .field("steppers", &self.steppers.len())
            .finish()
    }
}

/// The factory the section declarations name: the geometry follows the name
/// (`stepper_a/b/c` are delta towers, `stepper_x/y/z` full axes) — except under
/// `kinematics: winch`, where every letter section is a bare anchor motor.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let identifier = config.identifier();
    // A winch claims `stepper_a`…`stepper_z` as bare anchor motors
    // (`kinematics/winch.py:13-24`): each carries `anchor_x/y/z` and a cable
    // solver, but no rail range and no `position_endstop`. The shared letter
    // sections cannot tell winch from delta on their own, so the factory reads
    // the one option that does.
    if printer_kinematics_is_winch(config) {
        return Ok(Arc::new(PrinterStepper::with_geometry(
            config,
            printer,
            Axis::X,
            RailGeometry::BareMotor,
        )?));
    }
    let axis = axis_from_name(&identifier)?;
    let geometry = match identifier.strip_prefix("stepper_") {
        Some("a" | "b" | "c") => RailGeometry::DeltaTower {
            default_position_endstop: tower_default_endstop(config)?,
        },
        // The deltesian arms are rails too; the right arm falls back on the
        // left's endstop (`deltesian.py:17-21`).
        Some("left") => RailGeometry::DeltaTower {
            default_position_endstop: None,
        },
        Some("right") => RailGeometry::DeltaTower {
            default_position_endstop: deltesian_arm_default_endstop(config)?,
        },
        _ => RailGeometry::Axis,
    };
    Ok(Arc::new(PrinterStepper::with_geometry(
        config, printer, axis, geometry,
    )?))
}

/// Whether `[printer] kinematics` names the cable-winch family.
///
/// Read here because the shared letter sections (`stepper_a`…`stepper_z`) are
/// claimed before `[printer]` loads (order 50 against 60), and each family
/// reads them differently: delta's `stepper_a/b/c` are towers with a rail range
/// and a `position_endstop` (`kinematics/delta.py:15`), while winch's are bare
/// anchor motors (`kinematics/winch.py:13-24`). A wrapper built without the
/// whole config (or a config with no `[printer]` section) answers `false`, as
/// upstream would have no kinematics to load in that case.
fn printer_kinematics_is_winch(config: &ConfigWrapper) -> bool {
    config
        .sibling("printer")
        .and_then(|printer| printer.get_str("kinematics"))
        .is_some_and(|name| name == "winch")
}

/// The endstop a delta tower falls back on when its section omits one:
/// `stepper_b` and `stepper_c` take `stepper_a`'s `position_endstop`, as
/// `LookupMultiRail(..., default_position_endstop=a_endstop)` does
/// (`kinematics/delta.py:16-22`); `stepper_a` itself has no fallback
/// (`stepper.py:342-343` requires its own).
///
/// # Errors
/// When `stepper_a`'s endstop is missing (or unreadable) while a sibling
/// needs the default — the same config error `stepper_a`'s own load raises
/// first.
fn tower_default_endstop(config: &ConfigWrapper) -> Result<Option<f64>, ConfigError> {
    if config.section().id == "stepper_a" {
        return Ok(None);
    }
    let primary = config.sibling("stepper_a").ok_or_else(|| {
        ConfigError::new(format!(
            "Section '{}' needs a '[stepper_a]' section",
            config.identifier()
        ))
    })?;
    primary.get_float("position_endstop", None).map(Some)
}

/// The endstop a deltesian right arm falls back on: `stepper_left`'s
/// `position_endstop` (`deltesian.py:18-21`, `default_position_endstop=def_pos_es`).
///
/// # Errors
/// When `stepper_left` is missing while `stepper_right` needs the default —
/// the same config error `stepper_left`'s own load raises first.
fn deltesian_arm_default_endstop(config: &ConfigWrapper) -> Result<Option<f64>, ConfigError> {
    let primary = config.sibling("stepper_left").ok_or_else(|| {
        ConfigError::new(format!(
            "Section '{}' needs a '[stepper_left]' section",
            config.identifier()
        ))
    })?;
    primary.get_float("position_endstop", None).map(Some)
}

/// The factory `[stepper_arm]` names (`kinematics/polar.py:27`'s
/// `stepper.LookupRail`): a rail with geometry. Its stepper homes toolhead X
/// (upstream homes axis 0 on this rail, with Y pinned to 0), so it carries
/// [`Axis::X`].
pub(crate) fn load_config_arm(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterStepper::new(
        config,
        printer,
        Axis::X,
        true,
    )?))
}

/// The factory `[stepper_bed]` names (`kinematics/polar.py:26`'s
/// `stepper.PrinterStepper(config, units_in_radians=True)`): a **bare** motor
/// — no rail geometry (no `position_*` options), step distance in radians
/// (inferred from `gear_ratio` without `rotation_distance`).
///
/// The axis is inert: the polar toolhead installs the angle solver at load
/// (`setup_itersolve`), so the cartesian fallback this value would select
/// never runs.
pub(crate) fn load_config_bed(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterStepper::new(
        config,
        printer,
        Axis::X,
        false,
    )?))
}

/// Parse the homing parameters of a `[stepper_*]` rail
/// (`GenericPrinterRail.__init__`, `klippy/stepper.py:347-390`).
pub(crate) fn read_homing_info(
    config: &ConfigWrapper,
    identifier: &str,
    position_min: f64,
    position_max: f64,
    position_endstop: f64,
) -> Result<HomingInfo, ConfigError> {
    let speed = config.get_float_bounded("homing_speed", Some(5.0), None, None, Some(0.0), None)?;
    let second_homing_speed = config.get_float("second_homing_speed", Some(speed / 2.0))?;
    let retract_speed = config.get_float("homing_retract_speed", Some(speed))?;
    let retract_dist = config.get_float("homing_retract_dist", Some(5.0))?;
    let positive_dir = match config.get_optional_bool("homing_positive_dir")? {
        Some(positive) => positive,
        None => {
            // Infer from where the endstop sits: near the low end means homing
            // moves negative, near the high end positive, and anywhere in the
            // middle is ambiguous.
            let axis_len = position_max - position_min;
            if position_endstop <= position_min + axis_len / 4.0 {
                false
            } else if position_endstop >= position_max - axis_len / 4.0 {
                true
            } else {
                return Err(ConfigError::new(format!(
                    "Unable to infer homing_positive_dir in section '{identifier}'"
                )));
            }
        }
    };
    if (positive_dir && position_endstop == position_min)
        || (!positive_dir && position_endstop == position_max)
    {
        return Err(ConfigError::new(format!(
            "Invalid homing_positive_dir / position_endstop in '{identifier}'"
        )));
    }
    Ok(HomingInfo {
        speed,
        position_endstop,
        retract_speed,
        retract_dist,
        positive_dir,
        second_homing_speed,
    })
}

/// The axis a section name selects: `stepper_x` → [`Axis::X`], and a delta
/// tower → its rail-order slot (`stepper_a` → X, …), which is an index into
/// the toolhead's rails, not a cartesian meaning: the tower's real solver is
/// the delta one the kinematics installs.
fn axis_from_name(identifier: &str) -> Result<Axis, ConfigError> {
    match identifier.strip_prefix("stepper_") {
        Some("x" | "a") => Ok(Axis::X),
        Some("y" | "b") => Ok(Axis::Y),
        Some("z" | "c") => Ok(Axis::Z),
        // The deltesian arms: the labels are inert (the deltesian kinematics
        // installs their solvers at load), they only have to map to something.
        Some("left") => Ok(Axis::X),
        Some("right") => Ok(Axis::Y),
        _ => Err(ConfigError::new(format!(
            "Unable to map section '{identifier}' to a cartesian axis"
        ))),
    }
}

/// The axis index [`PrinterStepper`] reports (`mathutil`'s constants).
pub fn axis_index(axis: Axis) -> usize {
    match axis {
        Axis::X => X_AXIS,
        Axis::Y => Y_AXIS,
        Axis::Z => Z_AXIS,
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;

    /// Load a config text the way the host does.
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = crate::core::klippy::config::Config::from_text(text)
            .expect("the test config parses")
            .0;
        let result = printer.load_config(&config);
        (printer, result)
    }

    /// An `[mcu]` plus an X stepper with the given extra options.
    fn config_with_x(extra: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n{extra}"
        )
    }

    #[test]
    fn test_section_names_map_to_axes() {
        assert_eq!(axis_from_name("stepper_x").unwrap(), Axis::X);
        assert_eq!(axis_from_name("stepper_y").unwrap(), Axis::Y);
        assert_eq!(axis_from_name("stepper_z").unwrap(), Axis::Z);
        // Delta towers take their rail-order slots (see `axis_from_name`).
        assert_eq!(axis_from_name("stepper_a").unwrap(), Axis::X);
        assert_eq!(axis_from_name("stepper_b").unwrap(), Axis::Y);
        assert_eq!(axis_from_name("stepper_c").unwrap(), Axis::Z);
        assert!(axis_from_name("stepper_e").is_err());
    }

    #[test]
    fn test_axis_indexes_are_the_mathutil_ones() {
        assert_eq!(axis_index(Axis::X), X_AXIS);
        assert_eq!(axis_index(Axis::Y), Y_AXIS);
        assert_eq!(axis_index(Axis::Z), Z_AXIS);
    }

    #[test]
    fn test_the_step_distance_follows_the_geometry() {
        // 40 mm per rotation, 200 full steps, 16 microsteps, no gearing:
        // 40 / (200 * 16) = 0.0125 mm per step.
        let step_dist: f64 = 40.0 / (200.0 * 16.0 * 1.0);
        assert!((step_dist - 0.0125).abs() < 1e-12);
    }

    #[test]
    fn test_a_stepper_section_loads_as_a_stepper_object() {
        let (printer, result) = load(&config_with_x(""));

        result.unwrap();
        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_x")
            .expect("the section registered a stepper object");
        assert_eq!(stepper.name(), "stepper_x");
        assert_eq!(stepper.axis(), Axis::X);
        assert!((stepper.step_dist() - 0.0125).abs() < 1e-12);
        assert_eq!(stepper.params().position_min, 0.0);
        assert_eq!(stepper.params().position_max, 200.0);
        // Registered but not queryable, as upstream's non-object stepper is.
        assert!(!printer
            .queryable_objects()
            .contains(&"stepper_x".to_string()));
        // No `endstop_pin`: the rail has no endstop yet.
        assert!(stepper.endstop().is_none());
    }

    #[test]
    fn test_an_endstop_pin_builds_the_rail_endstop_and_homing_info() {
        let (printer, result) = load(&config_with_x("endstop_pin: PA2\n"));

        result.unwrap();
        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_x")
            .unwrap();
        assert!(stepper.endstop().is_some());
        let info = stepper.homing_info();
        assert_eq!(info.position_endstop, 0.0);
        // The endstop sits at the low end, so homing moves negative.
        assert!(!info.positive_dir);
        assert_eq!(info.speed, 5.0);
        assert_eq!(info.second_homing_speed, 2.5);
        assert_eq!(info.retract_dist, 5.0);
    }

    #[test]
    fn test_an_endstop_in_the_middle_cannot_infer_the_direction() {
        let (_, result) = load(&config_with_x("endstop_pin: PA2\nposition_endstop: 100\n"));

        let err = result.unwrap_err().to_string();
        assert!(err.contains("Unable to infer homing_positive_dir"), "{err}");
    }

    #[test]
    fn test_a_gear_ratio_divides_into_the_step_distance() {
        let (printer, result) = load(&config_with_x("gear_ratio: 2:1\n"));

        result.unwrap();
        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_x")
            .unwrap();
        assert!((stepper.step_dist() - 0.00625).abs() < 1e-12);
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (_, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n",
        );

        let err = result.unwrap_err().to_string();
        assert!(err.contains("dir_pin"), "{err}");
    }

    #[test]
    fn test_pins_on_different_mcus_are_refused() {
        let (_, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [mcu zboard]\nserial: /dev/b\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: zboard:PA1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n",
        );

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Stepper dir pin must be on same mcu as step pin"),
            "{err}"
        );
    }

    #[test]
    fn test_a_position_endstop_outside_the_range_is_refused() {
        let (_, result) = load(&config_with_x("position_endstop: 250\n"));

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("must be between position_min and position_max"),
            "{err}"
        );
    }

    // ----------------------------------------------------------------------
    // Polar's sections (`kinematics/polar.py:26-28`)
    // ----------------------------------------------------------------------

    /// An `[mcu]` plus `[stepper_bed]` with the given extra options: the
    /// bare radians-mode motor (no `rotation_distance`, no `position_*`).
    fn config_with_bed(extra: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_bed]\nstep_pin: PA0\ndir_pin: PA1\n\
             microsteps: 16\ngear_ratio: 80:16\n{extra}"
        )
    }

    /// An `[mcu]` plus `[stepper_arm]` (the polar rail) with extras.
    fn config_with_arm(extra: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_arm]\nstep_pin: PA0\ndir_pin: PA1\n\
             rotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^PA2\nposition_endstop: 300\n\
             position_max: 300\nhoming_speed: 50\n{extra}"
        )
    }
    // -----------------------------------------------------------------------
    // Delta towers: the option matrix `kinematics/delta.py:15-22` implies
    // -----------------------------------------------------------------------

    /// The three delta-tower sections of `config/example-delta.cfg` (minus
    /// `arm_length`, which the delta kinematics reads, not the stepper).
    fn delta_towers(extra_a: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_a]\nstep_pin: PA0\ndir_pin: PA1\nenable_pin: !PA2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PA3\n\
             homing_speed: 50\n{extra_a}\
             [stepper_b]\nstep_pin: PB0\ndir_pin: PB1\nenable_pin: !PB2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PB3\n\
             [stepper_c]\nstep_pin: PC0\ndir_pin: PC1\nenable_pin: !PC2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PC3\n"
        )
    }

    #[test]
    fn test_stepper_bed_claims_in_radians_mode_without_geometry() {
        let (printer, result) = load(&config_with_bed(""));
        result.unwrap();

        // The section is claimed: the object is registered (and, like every
        // stepper, stays out of `objects/list`).
        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_bed")
            .expect("stepper_bed registered");
        assert_eq!(stepper.name(), "stepper_bed");
        assert!(!printer
            .queryable_objects()
            .contains(&"stepper_bed".to_string()));

        // Radians mode: no `rotation_distance`, a `gear_ratio` → the step
        // distance is 2π / (full_steps · microsteps · gearing)
        // = 2π / (200 · 16 · 5) (`parse_step_distance`'s inference).
        let expected = std::f64::consts::TAU / (200.0 * 16.0 * 5.0);
        assert!(
            (stepper.step_dist() - expected).abs() < 1e-12,
            "{}",
            stepper.step_dist()
        );
        // Bare motor: no rail geometry was read, so no range or homing
        // info exists (`polar.py:26` passes no rail).
        assert_eq!(stepper.params().position_min, 0.0);
        assert_eq!(stepper.params().position_max, 0.0);
        assert!(stepper.endstop().is_none());
    }

    #[test]
    fn test_a_bed_with_rotation_distance_stays_in_millimetres() {
        // The inference only fires without `rotation_distance`; with one,
        // gearing multiplies as on any other stepper.
        let (printer, result) = load(&config_with_bed("rotation_distance: 40\n"));
        result.unwrap();

        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_bed")
            .unwrap();
        let expected = 40.0 / (200.0 * 16.0 * 5.0);
        assert!((stepper.step_dist() - expected).abs() < 1e-12);
    }

    #[test]
    fn test_a_stepper_with_neither_rotation_distance_nor_gearing_is_refused() {
        // No `rotation_distance` and no `gear_ratio` is not radians mode —
        // upstream fails on the missing `rotation_distance` too.
        let (_, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_bed]\nstep_pin: PA0\ndir_pin: PA1\nmicrosteps: 16\n",
        );

        let err = result.unwrap_err().to_string();
        assert!(err.contains("rotation_distance"), "{err}");
    }

    #[test]
    fn test_stepper_arm_claims_as_a_rail_with_homing_info() {
        let (printer, result) = load(&config_with_arm(""));
        result.unwrap();

        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_arm")
            .expect("stepper_arm registered");
        // Rail geometry: range 0..300 mm, endstop at the max.
        assert_eq!(stepper.params().position_min, 0.0);
        assert_eq!(stepper.params().position_max, 300.0);
        assert_eq!(stepper.params().position_endstop, 300.0);
        // The endstop built from `endstop_pin`…
        assert!(stepper.endstop().is_some());
        // …and homing inferred toward the max (endstop in the top quarter).
        let info = stepper.homing_info();
        assert!(info.positive_dir, "{}", info.positive_dir);
        assert_eq!(info.position_endstop, 300.0);
        assert_eq!(info.speed, 50.0);
        // Millimetre mode: 40 mm / (200 · 16) per step.
        let expected = 40.0 / (200.0 * 16.0);
        assert!((stepper.step_dist() - expected).abs() < 1e-12);
    }

    #[test]
    fn test_an_arm_without_position_max_is_refused() {
        // The arm is a rail, so `position_max` is as required as it is for
        // `[stepper_x]`.
        let (_, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_arm]\nstep_pin: PA0\ndir_pin: PA1\n\
             rotation_distance: 40\nmicrosteps: 16\n\
             position_endstop: 300\n",
        );

        let err = result.unwrap_err().to_string();
        assert!(err.contains("position_max"), "{err}");
    }

    #[test]
    fn test_delta_towers_need_no_position_max_and_claim_the_sections() {
        // `example-delta.cfg` writes no `position_max`: the range is
        // `0..position_endstop` (`stepper.py:352-354`).
        let (printer, result) = load(&delta_towers("position_endstop: 297.05\n"));
        result.unwrap();
        for name in ["stepper_a", "stepper_b", "stepper_c"] {
            let stepper = printer
                .lookup_object_as::<PrinterStepper>(name)
                .unwrap_or_else(|| panic!("{name} registered"));
            assert_eq!(stepper.params().position_min, 0.0);
            assert_eq!(stepper.params().position_max, 297.05);
            assert!(stepper.endstop().is_some());
            // The endstop sits at the top of the range: homing moves up.
            assert!(stepper.homing_info().positive_dir);
        }
        // `stepper_a`'s homing speed is its own (`delta.py` homes with
        // `rails[0]`'s); `stepper_b/c` fall back to the default.
        assert_eq!(
            printer
                .lookup_object_as::<PrinterStepper>("stepper_a")
                .unwrap()
                .homing_info()
                .speed,
            50.0
        );
        assert_eq!(
            printer
                .lookup_object_as::<PrinterStepper>("stepper_b")
                .unwrap()
                .homing_info()
                .speed,
            5.0
        );
    }

    #[test]
    fn test_delta_tower_b_and_c_inherit_stepper_a_endstop() {
        // `LookupMultiRail(…, default_position_endstop=a_endstop)`:
        // only `stepper_a` carries its own `position_endstop`.
        let (printer, result) = load(&delta_towers("position_endstop: 297.05\n"));
        result.unwrap();
        for name in ["stepper_b", "stepper_c"] {
            let stepper = printer.lookup_object_as::<PrinterStepper>(name).unwrap();
            assert_eq!(stepper.params().position_endstop, 297.05, "{name}");
        }
    }

    #[test]
    fn test_a_delta_tower_without_any_endstop_is_refused() {
        // `stepper_a` has no fallback (`stepper.py:342-343`).
        let text = delta_towers("");
        let (_, result) = load(&text);
        let err = result.unwrap_err().to_string();
        assert!(err.contains("position_endstop"), "{err}");
    }
}
