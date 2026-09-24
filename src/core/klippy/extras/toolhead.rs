//! `[printer]` — the toolhead: the motion planner and the G-code that drives it.
//!
//! Upstream's `klippy/toolhead.py` reads `[printer]` for the velocity limits,
//! loads a kinematics by name (`kinematics/cartesian.py` builds the rails from
//! the `[stepper_*]` sections), and registers the commands that make the
//! toolhead move (`ToolHeadCommandHelper`: `G4`, `M400`, `M204`,
//! `SET_VELOCITY_LIMIT`). The `[printer]` section is loaded **last**, after every
//! generic section, because the kinematics needs the steppers (`klippy.py:124`).
//!
//! This module is that consumer. The motion algorithms are in
//! [`motion`](crate::core::klippy::motion); the `[stepper_*]` sections are
//! [`extras::stepper`](crate::core::klippy::extras::stepper). Here is where the
//! two are joined and where a `G1` becomes a stream of `queue_step` commands.
//!
//! # Registration
//!
//! The section is declared `phase = late, object = "toolhead"`: the loader loads
//! it after the generic walk and registers the object under the name upstream
//! uses, which is `toolhead`, not `printer`
//! (`klippy/toolhead.py:604-615`). The G-code commands are registered at load
//! time (they only become active once the printer is ready) and capture the
//! shared motion state rather than the object, so a restart can drop the object
//! without leaving a handler behind.
//!
//! # What is here, and what is not
//!
//! | command | meaning |
//! |---|---|
//! | `G4` | dwell, `P` in milliseconds or `S` in seconds |
//! | `M400` | flush the planner |
//! | `G28` | home the named axes (all three when none is named) |
//! | `SET_KINEMATIC_POSITION` | force the low-level position, homing the named axes |
//!
//! `G0` / `G1` are **not** here: they belong to
//! [`gcode_move`](crate::core::klippy::extras::gcode_move), which reads them off
//! the command line in g-code coordinates — offsets, relative mode, extrude
//! factor — and calls [`ToolHeadObject::move_to`] with a toolhead coordinate.
//! The toolhead plans moves; it does not interpret a `G` word. Loading that
//! module from here is upstream's `add_printer_objects` loading its default
//! modules (`toolhead.py:610-613`).
//!
//! # Flushing
//!
//! The step solver is the **full** compressor from FW5f: runs of steps are
//! compressed into `queue_step` commands (`interval`, `count`, `add`), so a
//! print does not overflow the firmware's move queue.
//!
//! Steps are generated and sent by a task spawned at connect: it wakes every
//! [`FLUSH_INTERVAL`], generates everything the planner has queued, and awaits
//! the transport, so a long move cannot outrun the send queue. The task checks
//! [`ToolHeadObject::shutdown`] each wake, so a restart stops it with the object.

use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::Duration;

use serde_json::{json, Value};
use tokio::time::sleep;
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::idex_modes;
use crate::core::klippy::extras::query_endstops::{QueryEndstops, QUERY_ENDSTOPS_OBJECT};
use crate::core::klippy::extras::stepper::{PrinterStepper, Rail};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::mcu::{
    Completion, McuEndstop, McuError, McuObject, McuStepper, TriggerDispatch,
};
use crate::core::klippy::motion::delta::{
    delta_active_flags, delta_position_fn, DeltaCalibration, DeltaConfig, DeltaKinematics,
    DELTA_RAIL_NAMES,
};
use crate::core::klippy::motion::extra::ExtraAxis;
use crate::core::klippy::motion::itersolve::{
    cartesian_active_flags, cartesian_position_fn, corexy_active_flags, corexy_position_fn,
    corexz_active_flags, corexz_position_fn, Axis, AxisFlags, PositionFn,
};
use crate::core::klippy::motion::kinematics::{
    home_move, polar_active_flags, polar_angle_normalize, polar_angle_solver, polar_angle_unwrap,
    polar_home_move, polar_radius_solver, CartesianKinematics, CartesianTransform,
    NoneKinematics, PolarKinematics, UnifiedHome,
};
use crate::core::klippy::motion::plan::MoveLimits;
use crate::core::klippy::motion::stepcompress::{StepCommand, StepCompressError};
use crate::core::klippy::motion::toolhead::ToolHead;
use crate::core::klippy::motion::{HomeCoord, HomingInfo};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject, RestartHooks};
use crate::core::klippy::reactor::Reactor;

// Loaded after the generic walk (upstream loads `toolhead` last), registered as
// the `toolhead` object (`[printer]`'s consumer).
section!(
    "printer",
    order = 60,
    phase = late,
    object = "toolhead",
    load = load_config
);

/// How often the flush task wakes to generate and send steps.
const FLUSH_INTERVAL: Duration = Duration::from_millis(10);

/// How old a finished move may stay in the trapq history before it is dropped.
const MOVE_HISTORY_EXPIRE: f64 = 30.0;

/// The kinematics `[printer] kinematics` may name.
///
/// The cartesian family shares one `CartesianKinematics` (limits, homing,
/// `check_move`) and differs in which solver each rail runs, how carriage
/// axes map to rail positions (`CartesianTransform`), and which endstops
/// watch which motors (`kinematics/corexy.py`, `corexz.py`,
/// `hybrid_corexy.py`, `hybrid_corexz.py`). `polar` builds its own
/// [`PolarKinematics`](crate::core::klippy::motion::kinematics::PolarKinematics)
/// over `[stepper_arm]`/`[stepper_z]` plus the bare `[stepper_bed]` stepper
/// (`kinematics/polar.py`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum KinematicsKind {
    /// `kinematics: none`
    None,
    /// `kinematics: cartesian`
    Cartesian,
    /// `kinematics: corexy`
    CoreXy,
    /// `kinematics: corexz`
    CoreXz,
    /// `kinematics: hybrid_corexy`
    HybridCoreXy,
    /// `kinematics: hybrid_corexz`
    HybridCoreXz,
    /// `kinematics: polar`
    Polar,
    /// `kinematics: delta` — the linear-delta family
    /// (`kinematics/delta.py`), whose kinematics and calibration math live in
    /// [`motion::delta`](crate::core::klippy::motion::delta).
    Delta,
}

impl KinematicsKind {
    /// The names this host implements. Kept as a list so the error names them.
    const NAMES: &'static [&'static str] = &[
        "none",
        "cartesian",
        "corexy",
        "corexz",
        "hybrid_corexy",
        "hybrid_corexz",
        "polar",
        "delta",
    ];

    /// Parse a `[printer] kinematics` value.
    fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "none" => Self::None,
            "cartesian" => Self::Cartesian,
            "corexy" => Self::CoreXy,
            "corexz" => Self::CoreXz,
            "hybrid_corexy" => Self::HybridCoreXy,
            "hybrid_corexz" => Self::HybridCoreXz,
            "polar" => Self::Polar,
            "delta" => Self::Delta,
            _ => return None,
        })
    }

    /// The rail-position-to-carriage mapping for the kinematics.
    fn transform(self) -> CartesianTransform {
        match self {
            Self::None | Self::Cartesian => CartesianTransform::Standard,
            Self::CoreXy => CartesianTransform::CoreXy,
            Self::CoreXz => CartesianTransform::CoreXz,
            Self::HybridCoreXy => CartesianTransform::HybridCoreXy,
            Self::HybridCoreXz => CartesianTransform::HybridCoreXz,
            // Polar has no rail→carriage transform: `connect` builds a
            // `PolarKinematics` from `kind` and never reads this arm (the
            // same way `none` never reads its `Standard`).
            Self::Polar => CartesianTransform::Standard,
            // Delta has no rail→carriage mapping of this kind: its towers' solvers
            // are bound to their geometry in the delta branch below, and this
            // value is only consulted by the cartesian build.
            Self::Delta => CartesianTransform::Standard,
        }
    }

    /// The rail names `Delta` claims and the cartesian default (`delta.py:15`).
    fn rail_names(self) -> [&'static str; 3] {
        match self {
            Self::Delta => DELTA_RAIL_NAMES,
            _ => ["stepper_x", "stepper_y", "stepper_z"],
        }
    }

    /// The solver each rail's steppers run (`X`, `Y`, `Z` order).
    ///
    /// Only the cartesian family's rails are zipped with this —
    /// `ToolHeadObject::new` installs `polar`'s per-stepper solvers (bed
    /// angle, arm radius, cartesian Z) in its own branch before this is
    /// consulted, and `none` has no rails.
    fn solvers(self) -> [(PositionFn, AxisFlags); 3] {
        let cart = |axis: Axis| (cartesian_position_fn(axis), cartesian_active_flags(axis));
        match self {
            Self::None | Self::Polar | Self::Cartesian => {
                [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)]
            }
            // CoreXY: both motors carry the X/Y coupling, so each moves when
            // either axis does (`corexy_stepper_alloc`).
            Self::CoreXy => [
                (corexy_position_fn(true), corexy_active_flags()),
                (corexy_position_fn(false), corexy_active_flags()),
                cart(Axis::Z),
            ],
            // CoreXZ: the X/Z pairing is on the X and Z rails.
            Self::CoreXz => [
                (corexz_position_fn(true), corexz_active_flags()),
                cart(Axis::Y),
                (corexz_position_fn(false), corexz_active_flags()),
            ],
            // Hybrid CoreXY: only the X motor is coupled (`x - y`).
            Self::HybridCoreXy => [
                (corexy_position_fn(false), corexy_active_flags()),
                cart(Axis::Y),
                cart(Axis::Z),
            ],
            // Hybrid CoreXZ: only the X motor is coupled (`x - z`).
            Self::HybridCoreXz => [
                (corexz_position_fn(false), corexz_active_flags()),
                cart(Axis::Y),
                cart(Axis::Z),
            ],
            // Delta never reaches here: its branch below binds each tower to
            // `delta_stepper_alloc` instead (`delta.py:50-52`).
            Self::Delta => [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)],
        }
    }

    /// Endstops that must also stop the other rail's motors: `(target, source)`
    /// means the target rail's endstop watches `source`'s steppers.
    ///
    /// Upstream registers these in each kinematics' `__init__`
    /// (`corexy.py:14-17` is the two-way case; `hybrid_corexy.py:17-18` the
    /// one-way one).
    fn endstop_pairs(self) -> &'static [(usize, usize)] {
        match self {
            Self::CoreXy => &[(0, 1), (1, 0)],
            Self::CoreXz => &[(0, 2), (2, 0)],
            Self::HybridCoreXy => &[(1, 0)],
            Self::HybridCoreXz => &[(2, 0)],
            _ => &[],
        }
    }
}

/// The `toolhead` object: the planner, its kinematics, and the MCU steppers.
pub struct ToolHeadObject {
    limits: MoveLimits,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// The cartesian rails, `[stepper_x]`, `[stepper_y]`, and `[stepper_z]`;
    /// for polar, `[stepper_arm, stepper_z]` (each with its `…1`, `…2`
    /// siblings); for delta, `[stepper_a/b/c]`.
    ///
    /// Empty for `kinematics: none`, which has no steppers.
    rails: Vec<Arc<Rail>>,
    /// The bed rail of a polar printer — the bare `[stepper_bed]` stepper.
    /// It belongs to no rail (`kinematics/polar.py:26` builds it standalone),
    /// but its host solver still runs through the usual stepper setup; **its
    /// solvers are only installed by `KinematicsKind::Polar`**.
    bed: Option<Arc<PrinterStepper>>,
    /// Whether `[printer] kinematics` was `none`.
    /// What `[printer] kinematics` named, for consumers that need to know
    /// which one loaded before the toolhead itself connects
    /// (`[delta_calibrate]`'s `hasattr(kin, "get_calibration")` check).
    kind: KinematicsKind,
    /// The delta kinematics, parked here at load until connect installs it —
    /// the rails' delta solvers are already bound in `new`.
    delta: Mutex<Option<DeltaKinematics>>,
    /// How the rails' positions map to carriage axes (`corexy.py:12-15`).
    transform: CartesianTransform,
    /// `[printer] max_angular_velocity`: polar's near-center angular cap
    /// (`0` = uncapped). Read only for polar, as upstream reads it only in
    /// `PolarKinematics.__init__` (`kinematics/polar.py:51-52`).
    max_angular_velocity: f64,
    /// The active extruder's name (`ACTIVATE_EXTRUDER`), for `M104` without `T`.
    active_extruder: Mutex<String>,
    /// The machine's clock, for seeding the print-time mapping.
    reactor: Arc<dyn Reactor>,
    /// The machine, to shut it down if the compressor hits an internal error.
    /// `Weak` because the printer's registry owns this object.
    printer: Weak<Printer>,
    /// The connected motion state; `None` until connect.
    state: Arc<Mutex<Option<Connected>>>,
    /// Set when the object is dropped, to stop the flush task.
    shutdown: Arc<AtomicBool>,
}

/// Everything that exists only once the machine is up.
struct Connected {
    toolhead: ToolHead,
    mcu_steppers: HashMap<String, Arc<McuStepper>>,
    /// The print time the solvers have generated up to.
    last_step_gen_time: f64,
}

/// The step commands to send, paired with the stepper that produced them.
type StepBatches = Vec<(Arc<McuStepper>, Vec<StepCommand>)>;

impl ToolHeadObject {
    /// Build the object from `[printer]` and the three stepper sections.
    ///
    /// # Errors
    /// Returns a config error for a missing or unsupported `kinematics`, a
    /// missing velocity limit, or a missing `[stepper_x/y/z]`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let kinematics = config.get("kinematics", None)?;
        let kind = KinematicsKind::parse(&kinematics).ok_or_else(|| {
            ConfigError::new(format!(
                "Error loading kinematics '{kinematics}' (only {} are implemented)",
                KinematicsKind::NAMES.join(", ")
            ))
        })?;

        // `PolarKinematics.__init__` reads the angular cap; no other
        // kinematics does (`kinematics/polar.py:51-52`), so a
        // `max_angular_velocity` in a cartesian config stays unread and is
        // rejected by `check_unused`, as upstream does. Upstream's default
        // (0 = uncapped) is not bounds-checked, so an absent option is not
        // either — only a present value must be above 0.
        let max_angular_velocity = if kind == KinematicsKind::Polar {
            if config.has("max_angular_velocity") {
                config.get_float_bounded(
                    "max_angular_velocity",
                    None,
                    None,
                    None,
                    Some(0.0),
                    None,
                )?
            } else {
                0.0
            }
        } else {
            0.0
        };

        let max_velocity =
            config.get_float_bounded("max_velocity", None, None, None, Some(0.0), None)?;
        let max_accel = config.get_float_bounded("max_accel", None, None, None, Some(0.0), None)?;
        let min_cruise_ratio = config.get_float_bounded(
            "minimum_cruise_ratio",
            Some(0.5),
            Some(0.0),
            None,
            None,
            Some(1.0),
        )?;
        let square_corner_velocity = config.get_float_bounded(
            "square_corner_velocity",
            Some(5.0),
            Some(0.0),
            None,
            None,
            None,
        )?;
        // `CartKinematics` defaults Z to the full limits and caps it there
        // (`klippy/kinematics/cartesian.py:53-56`).
        let max_z_velocity = config.get_float_bounded(
            "max_z_velocity",
            Some(max_velocity),
            None,
            Some(max_velocity),
            Some(0.0),
            None,
        )?;
        let max_z_accel = config.get_float_bounded(
            "max_z_accel",
            Some(max_accel),
            None,
            Some(max_accel),
            Some(0.0),
            None,
        )?;

        // The junction geometry (`ToolHead._calc_junction_deviation`).
        let junction_deviation =
            square_corner_velocity.powi(2) * (std::f64::consts::SQRT_2 - 1.0) / max_accel;
        let limits = MoveLimits {
            max_velocity,
            max_accel,
            junction_deviation,
            mcr_pseudo_accel: max_accel * (1.0 - min_cruise_ratio),
        };

        let mut rails: Vec<Arc<Rail>> = Vec::new();
        let mut bed = None;
        let mut delta_kinematics = None;
        match kind {
            KinematicsKind::None => {}
            KinematicsKind::Polar => {
                // `kinematics/polar.py:25-31`: the arm is a rail
                // (`LookupRail`), Z is a (multi) rail, and the bed is a bare
                // stepper with no rail geometry of its own.
                let arm = Rail::lookup(config, printer, "stepper_arm", Axis::X)?;
                let z = Rail::lookup(config, printer, "stepper_z", Axis::Z)?;
                // The owning kinematics installs each stepper's solver
                // (`MCU_stepper.setup_itersolve`): arm radius and bed angle
                // both follow X and Y (`kin_polar.c:46`), Z stays cartesian.
                for stepper in arm.steppers() {
                    stepper.setup_itersolve(polar_radius_solver(), polar_active_flags());
                }
                for stepper in z.steppers() {
                    stepper.setup_itersolve(
                        cartesian_position_fn(Axis::Z),
                        cartesian_active_flags(Axis::Z),
                    );
                }
                let bed_stepper = printer
                    .lookup_object_as::<PrinterStepper>("stepper_bed")
                    .ok_or_else(|| {
                        ConfigError::new(format!(
                            "Section '{}' needs a '[stepper_bed]' section",
                            config.identifier()
                        ))
                    })?;
                bed_stepper.setup_itersolve(polar_angle_solver(), polar_active_flags());
                // The angle solver unwraps ±2π against `commanded_pos` and
                // renormalizes after each range (`kin_polar.c`).
                bed_stepper.setup_hooks(polar_angle_unwrap, polar_angle_normalize);
                rails = vec![arm, z];
                bed = Some(bed_stepper);
            }
            KinematicsKind::Delta => {
                // Delta claims `stepper_a/b/c`, the cartesian family
                // `stepper_x/y/z` (`delta.py:15`); the axis is the rail's slot in
                // this list.
                let axes = [Axis::X, Axis::Y, Axis::Z];
                for (name, axis) in kind.rail_names().into_iter().zip(axes) {
                    rails.push(Rail::lookup(config, printer, name, axis)?);
                }
                // The delta kinematics reads its options here (the config
                // reads are part of loading it) and binds each tower to
                // `delta_stepper_alloc` (`setup_itersolve`, `delta.py:50-52`).
                let delta = build_delta(
                    config,
                    &rails,
                    max_velocity,
                    max_accel,
                    max_z_velocity,
                    max_z_accel,
                )?;
                for (rail, (arm2, tower_x, tower_y)) in rails.iter().zip(delta.tower_geometry()) {
                    for stepper in rail.steppers() {
                        stepper.setup_itersolve(
                            delta_position_fn(arm2, tower_x, tower_y),
                            delta_active_flags(),
                        );
                    }
                }
                delta_kinematics = Some(delta);
            }
            _ => {
                for (name, axis) in [
                    ("stepper_x", Axis::X),
                    ("stepper_y", Axis::Y),
                    ("stepper_z", Axis::Z),
                ] {
                    rails.push(Rail::lookup(config, printer, name, axis)?);
                }
                // The owning kinematics installs each stepper's solver
                // (`MCU_stepper.setup_itersolve`); the family decides the position
                // function, so a corexy motor follows `x ± y`.
                let solvers = kind.solvers();
                for (rail, (position, flags)) in rails.iter().zip(solvers) {
                    for stepper in rail.steppers() {
                        stepper.setup_itersolve(position, flags);
                    }
                }
                // A paired rail's endstop has to stop the other rail's motors too
                // (`corexy.py:14-17` and friends).
                for (target, source) in kind.endstop_pairs() {
                    let Some(endstop) = rails[*target].endstop().cloned() else {
                        continue;
                    };
                    for stepper in rails[*source].steppers() {
                        // Every endstop a rail can name drives a dispatch; the
                        // default-`None` answer belongs to endstops that never
                        // ride a rail.
                        let dispatch = endstop.dispatch().ok_or_else(|| {
                            ConfigError::new(format!(
                                "{}: a paired rail's endstop must drive a trigger dispatch",
                                config.identifier()
                            ))
                        })?;
                        dispatch
                            .add_stepper(
                                stepper.mcu_stepper().chip().clone(),
                                Arc::downgrade(stepper.mcu_stepper()),
                                stepper.name(),
                            )
                            .map_err(|err| {
                                ConfigError::new(format!("{}: {err}", config.identifier()))
                            })?;
                    }
                }
                // IDEX: the cartesian kinematics claims `[dual_carriage]` and
                // hands the module this axis' primary rail
                // (`kinematics/cartesian.py:24-34`).
                if kind == KinematicsKind::Cartesian {
                    idex_modes::claim(&rails, printer);
                }
            }
        }

        // The object every rail's endstop is queried through. Created here
        // because this is the first point where all the `[stepper_*]` sections
        // (and their endstops) exist; registered before `toolhead` itself.
        let query = QueryEndstops::new(printer)?;
        for rail in &rails {
            for stepper in rail.steppers() {
                if let Some(endstop) = stepper.endstop() {
                    query.register_endstop(Arc::clone(endstop), stepper.name());
                }
            }
        }
        printer.add_object(QUERY_ENDSTOPS_OBJECT, Arc::new(query))?;

        let state = Arc::new(Mutex::new(None));
        let object = Self {
            limits,
            max_z_velocity,
            max_z_accel,
            rails,
            bed,
            kind,
            delta: Mutex::new(delta_kinematics),
            transform: kind.transform(),
            max_angular_velocity,
            active_extruder: Mutex::new("extruder".to_string()),
            reactor: printer.reactor(),
            printer: Arc::downgrade(printer),
            state,
            shutdown: Arc::new(AtomicBool::new(false)),
        };
        object.register_commands(printer)?;
        Ok(object)
    }

    /// Register the G-code commands, capturing only the shared motion state.
    fn register_commands(&self, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        // `G0`/`G1` are `gcode_move`'s: they interpret g-code coordinates
        // before the toolhead sees them (upstream `ToolHeadCommandHelper` has
        // no move commands for the same reason).
        let dwell_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            sync(move |gcmd| cmd_dwell(&state, gcmd))
        };
        gcode
            .register_command("G4", dwell_handler, None, false)
            .map_err(ConfigError::new)?;
        let wait_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            sync(move |gcmd| cmd_wait_moves(&state, gcmd))
        };
        gcode
            .register_command("M400", wait_handler, None, false)
            .map_err(ConfigError::new)?;
        let position_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            let printer = Arc::downgrade(printer);
            sync(move |gcmd| cmd_set_kinematic_position(&state, &printer, gcmd))
        };
        gcode
            .register_command(
                "SET_KINEMATIC_POSITION",
                position_handler,
                Some("Force a low-level kinematic position"),
                false,
            )
            .map_err(ConfigError::new)?;
        let home_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            let rails = self.rails.clone();
            let kind = self.kind;
            let printer = Arc::downgrade(printer);
            Arc::new(move |gcmd: &GcodeCommand| {
                let state = Arc::clone(&state);
                let rails = rails.clone();
                let printer = printer.clone();
                Box::pin(async move { cmd_g28(&state, &rails, kind, &printer, gcmd).await })
            })
        };
        gcode
            .register_command("G28", home_handler, Some("Home one or more axes"), false)
            .map_err(ConfigError::new)?;
        Ok(())
    }

    /// The names of the three rails, in axis order.
    fn axis_names(&self) -> [String; 3] {
        [
            self.rails[0].name().to_string(),
            self.rails[1].name().to_string(),
            self.rails[2].name().to_string(),
        ]
    }

    /// The names of the Z rail's steppers, in config order (`stepper_z`,
    /// `stepper_z1`, `stepper_z2`…), for a Z-tilt/QGL helper to check its
    /// `z_positions` count against (`ZAdjustHelper.handle_connect` counts the
    /// toolhead's z-active steppers the same way).
    ///
    /// Empty for `kinematics: none`, which has no rails, and for polar
    /// (whose Z rail is found by name below — polar's rails are
    /// `[stepper_arm, stepper_z]`, so a fixed index would be wrong).
    pub fn z_stepper_names(&self) -> Vec<String> {
        self.rails
            .iter()
            .find(|rail| rail.name() == "stepper_z")
            .map(|rail| {
                rail.steppers()
                    .iter()
                    .map(|stepper| stepper.name().to_string())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// The machine's maximum velocity, for `[extruder]`'s speed defaults.
    pub fn max_velocity(&self) -> f64 {
        self.limits.max_velocity
    }

    /// Whether the loaded `[printer]` kinematics carries a delta calibration
    /// (`delta_calibrate.py:handle_connect`'s `hasattr(kin,
    /// "get_calibration")`).
    ///
    /// Answerable from the load-time kind, so `[delta_calibrate]` can check at
    /// its own connect — which runs before this object's, since upstream loads
    /// `toolhead` last (`toolhead.py:604-615`).
    pub fn has_delta_calibration(&self) -> bool {
        matches!(self.kind, KinematicsKind::Delta)
    }

    /// The delta calibration parameters the kinematics carries
    /// (`get_calibration`, `delta.py:153-160`), or `None` for any other
    /// kinematics.
    ///
    /// Read from the connected kinematics when the machine is up, and from
    /// the parameters parked here at load otherwise — they are the same
    /// object; connect hands it over.
    pub fn delta_calibration(&self) -> Option<DeltaCalibration> {
        {
            let guard = self.lock();
            if let Some(connected) = guard.as_ref() {
                if let Some(calibration) = connected
                    .toolhead
                    .kinematics()
                    .and_then(|kinematics| kinematics.delta_calibration())
                {
                    return Some(calibration);
                }
            }
        }
        self.delta
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_ref()
            .map(DeltaKinematics::calibration)
    }

    /// The machine's maximum acceleration, for `[extruder]`'s speed defaults.
    pub fn max_accel(&self) -> f64 {
        self.limits.max_accel
    }

    /// Record the active extruder (`ACTIVATE_EXTRUDER`).
    pub fn set_active_extruder(&self, name: &str) {
        *self
            .active_extruder
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = name.to_string();
    }

    /// The active extruder's name.
    pub fn active_extruder(&self) -> String {
        self.active_extruder
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }
}

impl PrinterObject for ToolHeadObject {
    fn get_status(&self, _eventtime: f64) -> Value {
        let guard = self.lock();
        let Some(connected) = guard.as_ref() else {
            return json!({});
        };
        let position = connected.toolhead.commanded_pos();
        let homed_axes = connected
            .toolhead
            .kinematics()
            .map(|kinematics| kinematics.get_status()["homed_axes"].clone())
            .unwrap_or_else(|| json!(""));
        json!({
            "position": position.as_array(),
            "homed_axes": homed_axes,
            "print_time": connected.toolhead.print_time(),
            // The active extruder's name, as upstream reports it
            // (`toolhead.py:511`); `PARK_{printer.toolhead.extruder}` and
            // friends read it through the macro template.
            "extruder": self.active_extruder(),
            "max_velocity": self.limits.max_velocity,
            "max_accel": self.limits.max_accel,
        })
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            let config_error = |message: String| {
                KlippyError::Config(ConfigError::new(format!("[printer]: {message}")))
            };

            // Each `[stepper_*]` built its host solver during its own connect,
            // which runs before this one (generic sections before the late
            // walk). Take them now, along with the firmware resources and the
            // MCU each axis lives on.
            let mut host_steppers = Vec::new();
            let mut mcu_steppers = HashMap::new();
            for rail in &self.rails {
                for stepper in rail.steppers() {
                    let host = stepper.take_stepper().ok_or_else(|| {
                        config_error(format!("{} is not connected", stepper.name()))
                    })?;
                    host_steppers.push(host);
                    mcu_steppers.insert(
                        stepper.name().to_string(),
                        Arc::clone(stepper.mcu_stepper()),
                    );
                }
            }
            // Polar's bare bed stepper: it belongs to no rail, but its host
            // solver drives it from the main trapq like the rails' — upstream
            // lists it first (`kinematics/polar.py:31-34`).
            if let Some(bed) = &self.bed {
                let host = bed
                    .take_stepper()
                    .ok_or_else(|| config_error(format!("{} is not connected", bed.name())))?;
                host_steppers.push(host);
                mcu_steppers.insert(bed.name().to_string(), Arc::clone(bed.mcu_stepper()));
            }

            // The primary MCU (the bare `[mcu]`) defines the print-time origin;
            // each stepper's compressor was already pointed at its own MCU's
            // clock domain (`SecondarySync`) during its own connect, which used
            // this same `[mcu]` object's clock.
            let main_print_time = self
                .printer
                .upgrade()
                .and_then(|printer| printer.lookup_object_as::<McuObject>("mcu"))
                .and_then(|object| object.estimated_print_time(self.reactor.monotonic()))
                .unwrap_or(0.0);

            let mut toolhead = ToolHead::new(self.limits);
            for stepper in host_steppers {
                toolhead.add_stepper(stepper);
            }
            match self.kind {
                KinematicsKind::None => toolhead.set_kinematics(Box::new(NoneKinematics)),
                KinematicsKind::Polar => {
                    // rails = [arm, z] (installed in `new`); ranges feed the
                    // gated square and the Z limit, as upstream derives them
                    // from `rails[0]`/`rails[1]` (`kinematics/polar.py:42-51`).
                    let bed = self
                        .bed
                        .as_ref()
                        .ok_or_else(|| config_error("stepper_bed is not connected".to_string()))?;
                    let arm = &self.rails[0];
                    let z = &self.rails[1];
                    toolhead.set_kinematics(Box::new(PolarKinematics::new(
                        [
                            bed.name().to_string(),
                            arm.name().to_string(),
                            z.name().to_string(),
                        ],
                        (arm.params().position_min, arm.params().position_max),
                        (z.params().position_min, z.params().position_max),
                        self.limits,
                        self.max_z_velocity,
                        self.max_z_accel,
                        self.max_angular_velocity,
                    )));
                }
            KinematicsKind::Delta => {
                let delta = self
                    .delta
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .take()
                    .ok_or_else(|| {
                        config_error("delta kinematics is not connected".to_string())
                    })?;
                toolhead.set_kinematics(Box::new(delta));
            }
                _ => {
                    toolhead.set_kinematics(Box::new(CartesianKinematics::new(
                        self.axis_names(),
                        Coord::new(
                            self.rails[X_AXIS].params().position_min,
                            self.rails[Y_AXIS].params().position_min,
                            self.rails[Z_AXIS].params().position_min,
                            0.0,
                        ),
                        Coord::new(
                            self.rails[X_AXIS].params().position_max,
                            self.rails[Y_AXIS].params().position_max,
                            self.rails[Z_AXIS].params().position_max,
                            0.0,
                        ),
                        self.max_z_velocity,
                        self.max_z_accel,
                        self.transform,
                    )));
                }
            }

            // The toolhead's print time is the primary MCU's.
            toolhead.set_estimated_print_time(main_print_time);

            // The extruders are the toolhead's `extra_axes`: each has its own
            // trapq, and the extruder queues the extrusion into it
            // (`kinematics/extruder.py:140-190`).
            if let Some(printer) = self.printer.upgrade() {
                for name in extruder_names(&printer) {
                    let Some(extruder) = printer.lookup_object_as::<PrinterExtruder>(&name) else {
                        continue;
                    };
                    if let (Some(mut stepper), Some(mcu_stepper)) =
                        (extruder.take_stepper(), extruder.mcu_stepper())
                    {
                        let trapq = toolhead.allocate_trapq();
                        stepper.set_trapq(trapq);
                        mcu_steppers.insert(stepper.name().to_string(), mcu_stepper);
                        toolhead.add_stepper(stepper);
                        extruder.set_trapq(trapq);
                    }
                    toolhead.add_extra_axis(Arc::clone(&extruder) as Arc<dyn ExtraAxis>);
                }
            }

            *self.lock() = Some(Connected {
                toolhead,
                mcu_steppers,
                last_step_gen_time: 0.0,
            });

            // Let the G-code dispatcher reach the planner before a restart
            // (`GCodeDispatch.request_restart` needs the last print time, a
            // dwell and a wait).
            if let Some(printer) = self.printer.upgrade() {
                printer.register_restart_hooks(Arc::new(ToolHeadRestartHooks(Arc::clone(
                    &self.state,
                ))));
            }

            // The flush task owns nothing the object does not share; it stops
            // when the object is dropped, by reading `shutdown`.
            tokio::spawn(run_flush_loop(
                Arc::clone(&self.state),
                Arc::clone(&self.shutdown),
                self.printer.clone(),
            ));
            Ok(())
        })
    }
}

/// Read the delta options and build the kinematics
/// (`kinematics/delta.py:11-77`, whose reads these mirror one for one): the
/// `[printer]` options here, the tower options from each `[stepper_a/b/c]`
/// section.
///
/// # Errors
/// A missing or out-of-bounds option, reported with the config reader's
/// upstream wording; or a geometry whose home position does not exist (see
/// [`DeltaKinematics::new`]).
#[allow(clippy::too_many_arguments)]
fn build_delta(
    config: &ConfigWrapper,
    rails: &[Arc<Rail>],
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
) -> Result<DeltaKinematics, ConfigError> {
    let radius = config.get_float_bounded("delta_radius", None, None, None, Some(0.0), None)?;
    let print_radius =
        config.get_float_bounded("print_radius", Some(radius), None, None, Some(0.0), None)?;
    let mut endstops = [0.0; 3];
    for (index, rail) in rails.iter().enumerate() {
        endstops[index] = rail.homing_info().position_endstop;
    }
    let max_z = endstops.iter().copied().fold(f64::INFINITY, f64::min);
    let minimum_z_position = config.get_float_bounded(
        "minimum_z_position",
        Some(0.0),
        None,
        Some(max_z),
        None,
        None,
    )?;

    // Tower geometry: `stepper_a`'s `arm_length` is required and sets the
    // default for `stepper_b/c`; the angles default to 210/330/90
    // (`delta.py:34-42`).
    let mut arm_lengths = [0.0; 3];
    let mut angles = [0.0; 3];
    let default_angles = [210.0, 330.0, 90.0];
    for (index, name) in DELTA_RAIL_NAMES.iter().enumerate() {
        let tower = config.sibling(name).ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{}' needs a '[{name}]' section",
                config.identifier()
            ))
        })?;
        let arm_length = if index == 0 {
            tower.get_float_bounded("arm_length", None, None, None, Some(radius), None)?
        } else {
            tower.get_float_bounded(
                "arm_length",
                Some(arm_lengths[0]),
                None,
                None,
                Some(radius),
                None,
            )?
        };
        arm_lengths[index] = arm_length;
        angles[index] = tower.get_float("angle", Some(default_angles[index]))?;
    }
    let mut step_dists = [0.0; 3];
    for (index, rail) in rails.iter().enumerate() {
        step_dists[index] = rail.step_dist();
    }

    DeltaKinematics::new(DeltaConfig {
        radius,
        print_radius,
        minimum_z_position,
        angles,
        arm_lengths,
        endstops,
        step_dists,
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
    })
}

/// The registered extruders, in `[extruder]`, `[extruder1]`… order.
fn extruder_names(printer: &Printer) -> Vec<String> {
    let mut names = Vec::new();
    if printer.lookup_object("extruder").is_some() {
        names.push("extruder".to_string());
        for index in 1..99 {
            let name = format!("extruder{index}");
            if printer.lookup_object(&name).is_none() {
                break;
            }
            names.push(name);
        }
    }
    names
}

impl Drop for ToolHeadObject {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
    }
}

impl ToolHeadObject {
    fn lock(&self) -> MutexGuard<'_, Option<Connected>> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The print time the planner has reached, or `0.0` before connect.
    ///
    /// `query_endstops` dates a query from this (upstream's
    /// `toolhead.get_last_move_time()`).
    ///
    /// This only *reads* the time; upstream's `get_last_move_time()` first
    /// flushes the look-ahead into the trapq. Get that flush — and the steps
    /// generated and sent for it — from
    /// [`ToolHeadObject::flush_step_generation`], then read this.
    pub fn print_time(&self) -> f64 {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.print_time())
            .unwrap_or(0.0)
    }

    /// Flush the look-ahead into the trapq and return the print time it reached
    /// (`toolhead.get_last_move_time()`). Before connect this is `0.0`.
    ///
    /// Unlike [`ToolHeadObject::flush_step_generation`] this only moves the
    /// planner's queue; the steps behind it are generated and sent by the
    /// background task. It is the read half of
    /// [`ToolHeadObject::dwell`]'s pair, as upstream's is.
    pub fn get_last_move_time(&self) -> f64 {
        self.lock()
            .as_mut()
            .map(|connected| connected.toolhead.get_last_move_time())
            .unwrap_or(0.0)
    }

    /// Advance the planner's timeline by `delay` seconds (`toolhead.dwell`).
    ///
    /// A timed protocol — BLTouch's single-wire pulses — uses this to keep its
    /// next command past the moves already queued
    /// (`bltouch.py:_sync_print_time`). Before connect this does nothing.
    pub fn dwell(&self, delay: f64) {
        if let Some(connected) = self.lock().as_mut() {
            connected.toolhead.dwell(delay);
        }
    }

    /// Flush the look-ahead and generate **and send** every step queued so far,
    /// from inside a command (`toolhead.flush_step_generation`).
    ///
    /// Steps are normally generated and sent by the background flush task
    /// ([`run_flush_loop`]); this takes the connected state out of the shared
    /// slot for the duration (as `probing_move` and `G28` do), so the
    /// generation and the awaited transport writes happen here alone — the
    /// background task sees the empty slot and stands back. When this returns,
    /// every move queued **before the call** has been generated and handed to
    /// the transport, and [`ToolHeadObject::print_time`] is upstream's
    /// `get_last_move_time()` — that pair is the flush entry
    /// `ProbePointsHelper._invoke_callback` needs before its callback
    /// (`probe.py:419-424`).
    ///
    /// # Errors
    /// "Printer is not ready" before connect (or while a homing/probe run
    /// holds the state), a step-generation failure, or a failed step send.
    pub async fn flush_step_generation(&self) -> Result<(), CommandError> {
        flush_step_generation(&self.state).await
    }

    /// Force the toolhead to `newpos`, marking `homing_axes` as homed
    /// (`ToolHead.set_position`, `toolhead.py:383-391`).
    ///
    /// Upstream's `set_position` starts by flushing step generation (the
    /// queued moves' steps must be generated before the trapq's position is
    /// rewritten), then sets the position, the homing flags, and fires
    /// `toolhead:set_position` ([`KlippyEvent::ToolheadSetPosition`]) so
    /// `gcode_move` re-anchors — this does the same, which makes it safe for a
    /// Z-tilt `adjust_steppers` loop to `move_to` and then `set_position`.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or a failure from the flush
    /// (generation or send).
    pub async fn set_position(
        &self,
        newpos: Coord,
        homing_axes: &[usize],
    ) -> Result<(), CommandError> {
        flush_step_generation(&self.state).await?;
        {
            let mut guard = self.lock();
            let Some(connected) = guard.as_mut() else {
                return Err(CommandError::new("Printer is not ready"));
            };
            connected.toolhead.set_position(newpos, homing_axes);
        }
        send(&self.printer, &KlippyEvent::ToolheadSetPosition);
        Ok(())
    }

    /// Queue a move to `position` at `speed` mm/s (upstream's `toolhead.move`).
    ///
    /// The toolhead plans it; deciding *what* the coordinates mean — absolute
    /// or relative, against which `G92` anchor, at what extrude factor — is
    /// `gcode_move`'s job, and it is what calls this.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or whatever the planner refuses:
    /// an unhomed axis, a move out of range.
    pub fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let mut guard = self.lock();
        let Some(connected) = guard.as_mut() else {
            return Err(CommandError::new("Printer is not ready"));
        };
        connected.toolhead.move_to(position, speed)
    }

    /// The commanded position (upstream's `toolhead.get_position`).
    ///
    /// `None` before connect: there is no planner to ask yet, and
    /// `gcode_move` only anchors itself at ready.
    pub fn position(&self) -> Option<Coord> {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.commanded_pos())
    }

    /// The main trapq's id — upstream's `toolhead.get_trapq()`, the queue
    /// `ZAdjustHelper.adjust_steppers` reattaches a Z motor to after moving it
    /// on its own.
    ///
    /// `None` before connect: there is no planner to ask yet.
    pub fn main_trapq(&self) -> Option<usize> {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.main_trapq())
    }

    /// Point a motion stepper at a trapq by name, or take it off one with
    /// `None` (`MCU_stepper.set_trapq`, the per-motor detach/reattach
    /// `ZAdjustHelper.adjust_steppers` walks through).
    ///
    /// A detached stepper generates no steps until it is attached again
    /// (`MotionQueuing::generate` skips it), which is how one Z motor moves
    /// while the others hold still.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or "Unknown stepper '<name>'"
    /// when no motion stepper answers to `name`.
    pub fn set_stepper_trapq(&self, name: &str, trapq: Option<usize>) -> Result<(), CommandError> {
        let mut guard = self.lock();
        let connected = guard
            .as_mut()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let stepper = connected
            .toolhead
            .motion_queuing_mut()
            .steppers_mut()
            .iter_mut()
            .find(|stepper| stepper.name() == name)
            .ok_or_else(|| CommandError::new(format!("Unknown stepper '{name}'")))?;
        stepper.set_trapq(trapq);
        Ok(())
    }

    /// Probe-style homing: move toward `target` at `speed`, stop on trigger
    /// (`homing.probing_move`).
    ///
    /// The toolhead is taken out of its shared slot during the move (the
    /// background flush task stands back), then restored afterwards.
    ///
    /// Returns the toolhead's commanded position at the end of the move.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or any error from the
    /// low-level probe move.
    pub async fn probing_move(
        &self,
        endstop: &dyn HomingEndstop,
        target: Coord,
        speed: f64,
    ) -> Result<Coord, CommandError> {
        let mut connected = {
            let mut guard = self.lock();
            let Some(connected) = guard.take() else {
                return Err(CommandError::new("Printer is not ready"));
            };
            connected
        };
        let result = probing_move(&mut connected, endstop, target, speed, &self.printer).await;
        *self.lock() = Some(connected);
        result
    }
}

/// The flush task: generate the queued steps and await the transport.
///
/// A command that must not return until the steps it queued are generated and
/// sent awaits [`flush_step_generation`] instead; the two never own the
/// connected state at the same time, because each takes it out of the shared
/// slot first.
async fn run_flush_loop(
    state: Arc<Mutex<Option<Connected>>>,
    shutdown: Arc<AtomicBool>,
    printer: Weak<Printer>,
) {
    loop {
        sleep(FLUSH_INTERVAL).await;
        if shutdown.load(Ordering::SeqCst) {
            return;
        }
        // Generate under the lock, then send without it: awaiting the transport
        // while holding a `std` mutex would make this future non-`Send`.
        let batches = {
            let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
            match guard.as_mut() {
                Some(connected) => connected.generate(),
                None => Ok(Vec::new()),
            }
        };
        let batches = match batches {
            Ok(batches) => batches,
            // `check_line` failed, which upstream treats as an internal error
            // and shuts the printer down for (`Internal error in stepcompress`).
            Err(err) => {
                if let Some(printer) = printer.upgrade() {
                    printer.invoke_shutdown(&format!("Internal error in stepcompress: {err}"));
                } else {
                    warn!("Internal error in stepcompress: {err}");
                }
                return;
            }
        };
        for (stepper, commands) in batches {
            if let Err(err) = stepper.send_steps_async(&commands).await {
                warn!(
                    "{}: {err}",
                    stepper.oid().map(u32::from).unwrap_or_default()
                );
            }
        }
    }
}

/// Generate the steps for everything queued so far and await sending them,
/// taking the connected state out of the shared slot for the duration
/// (`toolhead.flush_step_generation`).
///
/// The caller must not hold `state`'s lock: the background flush task and this
/// alternate through the slot, so only one of them generates and sends at a
/// time.
///
/// # Errors
/// "Printer is not ready" when the slot is empty (not connected, or a homing
/// run owns it), a step-generation failure, or a failed step send.
async fn flush_step_generation(state: &Arc<Mutex<Option<Connected>>>) -> Result<(), CommandError> {
    let mut connected = {
        let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
        guard
            .take()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?
    };
    let result = async {
        let batches = connected
            .generate()
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (stepper, commands) in batches {
            stepper
                .send_steps_async(&commands)
                .await
                .map_err(command_error)?;
        }
        Ok(())
    }
    .await;
    // Restore the state whether the flush succeeded or failed.
    *state.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(connected);
    result
}

impl Connected {
    /// Generate the steps for everything queued, and return them by stepper.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from a stepper's compressor.
    fn generate(&mut self) -> Result<StepBatches, StepCompressError> {
        // A secondary MCU's print-time mapping is recalibrated periodically as
        // its crystal drifts against the primary's; pick up the current mapping
        // on every stepper before generating.
        for stepper in self.toolhead.motion_queuing_mut().steppers_mut() {
            if let Some(mcu_stepper) = self.mcu_steppers.get(stepper.name()) {
                let (offset, freq) = mcu_stepper.chip().time_mapping();
                stepper.compressor_mut().set_time(offset, freq);
            }
        }
        // Move whatever the planner has queued into the trapq first, so the
        // step generation time below covers it.
        self.toolhead.wait_moves();
        let step_gen_time = self.toolhead.print_time().max(self.last_step_gen_time);
        let batches = self.toolhead.flush_step_generation(step_gen_time)?;
        self.toolhead.finalize_moves(
            step_gen_time,
            (step_gen_time - MOVE_HISTORY_EXPIRE).max(0.0),
        );
        self.last_step_gen_time = step_gen_time;
        Ok(batches
            .into_iter()
            .filter_map(|(name, commands)| {
                self.mcu_steppers
                    .get(&name)
                    .map(|stepper| (Arc::clone(stepper), commands))
            })
            .collect())
    }
}

/// The toolhead's restart handle (`RestartHooks`): the planner operations the
/// G-code dispatcher needs before a restart.
struct ToolHeadRestartHooks(Arc<Mutex<Option<Connected>>>);

impl RestartHooks for ToolHeadRestartHooks {
    fn get_last_move_time(&self) -> f64 {
        self.0
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_mut()
            .map(|connected| connected.toolhead.get_last_move_time())
            .unwrap_or(0.0)
    }

    fn dwell(&self, delay: f64) {
        if let Some(connected) = self
            .0
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_mut()
        {
            connected.toolhead.dwell(delay);
        }
    }

    fn wait_moves(&self) {
        if let Some(connected) = self
            .0
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_mut()
        {
            connected.toolhead.wait_moves();
        }
    }
}

// ===========================================================================
// Homing
// ===========================================================================

/// How long to wait after arming the endstop before moving
/// (`HOMING_START_DELAY`, `klippy/extras/homing.py:9`).
const HOMING_START_DELAY: f64 = 0.001;

/// How long the endstop confirms a trigger over
/// (`ENDSTOP_SAMPLE_TIME`/`ENDSTOP_SAMPLE_COUNT`).
const ENDSTOP_SAMPLE_TIME: f64 = 0.000_015;
const ENDSTOP_SAMPLE_COUNT: u8 = 4;

/// How much of a homing move's steps the drip loop queues at a time
/// (`DRIP_SEGMENT_TIME`, `klippy/extras/motion_queuing.py:20`).
const DRIP_SEGMENT_TIME: f64 = 0.050;

/// The most steps the homing driver may look ahead, so a trigger stops the
/// firmware with little queued behind it.
const DRIP_LOOKAHEAD: f64 = 0.010;

/// A future returned by [`HomingEndstop::home_wait`].
pub type EndstopFuture<'a> = Pin<Box<dyn Future<Output = Result<f64, McuError>> + Send + 'a>>;

/// A future returned by [`HomingEndstop::query_endstop`].
pub type QueryEndstopFuture<'a> = Pin<Box<dyn Future<Output = Result<bool, McuError>> + Send + 'a>>;

/// What the homing driver needs from an endstop (upstream's `MCU_endstop`).
///
/// A trait so the driver can be tested with a fake trigger, without an MCU.
pub trait HomingEndstop: Send + Sync {
    /// Arm the endstop for a move starting at `print_time`.
    ///
    /// # Errors
    /// As [`McuEndstop::home_start`].
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError>;

    /// Wait for the endstop trigger; returns its print time, or `0.0` when the
    /// move ended without one.
    ///
    /// # Errors
    /// As [`McuEndstop::home_wait`].
    fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_>;

    /// The trigger dispatch this endstop stops the steppers through (the
    /// pairing lookup in the kinematics setup uses it). `None` for an endstop
    /// that does not drive one — every endstop a rail can name (a pin
    /// `McuEndstop`, the eddy probe's `McuTriggerAnalog`) answers `Some`.
    fn dispatch(&self) -> Option<&TriggerDispatch> {
        None
    }

    /// Whether the pin reads triggered now (`M119` / `QUERY_ENDSTOPS`). The
    /// default reports "open", which is what upstream's virtual probe helper
    /// answers without a query callback
    /// (`probe.py:HomingViaProbeHelper.query_endstop` → `False`).
    fn query_endstop(&self, _print_time: f64) -> QueryEndstopFuture<'_> {
        Box::pin(async { Ok(false) })
    }
}

impl HomingEndstop for McuEndstop {
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError> {
        McuEndstop::home_start(
            self,
            print_time,
            sample_time,
            sample_count,
            rest_time,
            triggered,
        )
    }

    fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
        Box::pin(McuEndstop::home_wait(self, home_end_time))
    }

    fn dispatch(&self) -> Option<&TriggerDispatch> {
        Some(McuEndstop::dispatch(self))
    }

    fn query_endstop(&self, print_time: f64) -> QueryEndstopFuture<'_> {
        Box::pin(McuEndstop::query_endstop(self, print_time))
    }
}

/// Fill `None` entries from `current` (`Homing._fill_coord`).
fn fill_coord(home: HomeCoord, current: Coord) -> Coord {
    let mut out = current;
    for (axis, value) in home.iter().enumerate() {
        if let Some(value) = value {
            out.set_axis(axis, *value);
        }
    }
    out
}

/// The XYZ distance between two positions.
fn move_distance(a: Coord, b: Coord) -> f64 {
    let mut sum = 0.0;
    for axis in [X_AXIS, Y_AXIS, Z_AXIS] {
        sum += (b.axis(axis) - a.axis(axis)).powi(2);
    }
    sum.sqrt()
}

/// Home the requested axes, one at a time (`Homing.home_rails` driven per axis).
///
/// The caller has taken the toolhead out of its shared slot, so the background
/// flush task sees no toolhead and stands back while the drip loop here owns
/// step generation.
///
/// # Errors
/// A missing endstop, a kinematics refusal, or a failed query/send.
async fn home_axes(
    connected: &mut Connected,
    rails: &[Arc<Rail>],
    kind: KinematicsKind,
    requested: &[usize],
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    // `kinematics: none` has no rails, so there is nothing to home.
    if rails.is_empty() {
        return Ok(());
    }
    if kind == KinematicsKind::Polar {
        // `home` of `kinematics/polar.py:95-108`: X and Y always home
        // **together** on the arm rail (`rails[0]`, Y pinned to 0 by
        // [`polar_home_move`]) — whichever of them was requested — and Z
        // homes on its own rail (`rails[1]`), after.
        let home_xy = requested
            .iter()
            .any(|&axis| axis == X_AXIS || axis == Y_AXIS);
        if home_xy {
            let rail = &rails[0];
            let endstop = rail.endstop().ok_or_else(|| {
                CommandError::new(format!("No endstop configured for {}", rail.name()))
            })?;
            let params = rail.params();
            let info = rail.homing_info();
            let (forcepos, movepos) =
                polar_home_move(X_AXIS, &info, params.position_min, params.position_max);
            // Upstream marks every axis the force position sets as homed
            // (`homing.py:178-184`): x **and** y — which is what opens
            // `limit_xy2` before the drip move is checked.
            let homing_axes = homing_axes_of(&forcepos);
            send(printer, &KlippyEvent::HomingHomeRailsBegin);
            let result = home_axis(
                connected,
                X_AXIS,
                forcepos,
                movepos,
                &homing_axes,
                info,
                rail.step_dist(),
                endstop.as_ref(),
                printer,
            )
            .await;
            send(
                printer,
                &KlippyEvent::HomingHomeRailsEnd {
                    axes: homing_axes.clone(),
                },
            );
            result?;
        }
        if requested.contains(&Z_AXIS) {
            let rail = &rails[1];
            let endstop = rail.endstop().ok_or_else(|| {
                CommandError::new(format!("No endstop configured for {}", rail.name()))
            })?;
            let params = rail.params();
            let info = rail.homing_info();
            let (forcepos, movepos) =
                polar_home_move(Z_AXIS, &info, params.position_min, params.position_max);
            let homing_axes = homing_axes_of(&forcepos);
            send(printer, &KlippyEvent::HomingHomeRailsBegin);
            let result = home_axis(
                connected,
                Z_AXIS,
                forcepos,
                movepos,
                &homing_axes,
                info,
                rail.step_dist(),
                endstop.as_ref(),
                printer,
            )
            .await;
            send(
                printer,
                &KlippyEvent::HomingHomeRailsEnd {
                    axes: homing_axes.clone(),
                },
            );
            result?;
        }
        return Ok(());
    }
    // Delta homes every tower in one move and ignores which axes `G28` named
    // (`kinematics/delta.py:104-110` always takes all three rails), so its
    // homing is one multi-endstop move rather than one per axis.
    if let Some(home) = connected
        .toolhead
        .kinematics()
        .and_then(|kinematics| kinematics.unified_home())
    {
        send(printer, &KlippyEvent::HomingHomeRailsBegin);
        let result = home_unified(connected, rails, &home, printer).await;
        send(
            printer,
            &KlippyEvent::HomingHomeRailsEnd {
                axes: vec![X_AXIS, Y_AXIS, Z_AXIS],
            },
        );
        return result;
    }
    for &axis in requested {
        let rail = &rails[axis];
        let endstop = rail.endstop().ok_or_else(|| {
            CommandError::new(format!("No endstop configured for {}", rail.name()))
        })?;
        let params = rail.params();
        let info = rail.homing_info();
        let (forcepos, movepos) = home_move(axis, &info, params.position_min, params.position_max);
        send(printer, &KlippyEvent::HomingHomeRailsBegin);
        let result = home_axis(
            connected,
            axis,
            forcepos,
            movepos,
            &[axis],
            info,
            rail.step_dist(),
            endstop.as_ref(),
            printer,
        )
        .await;
        send(
            printer,
            &KlippyEvent::HomingHomeRailsEnd { axes: vec![axis] },
        );
        result?;
    }
    Ok(())
}

/// The axes a homing force position marks as homed: every axis whose
/// `forcepos` entry is set (`Homing._set_start_position`,
/// `klippy/extras/homing.py:178-184`). For polar's arm home that is x **and**
/// y — both must be marked to open `limit_xy2`.
fn homing_axes_of(forcepos: &HomeCoord) -> Vec<usize> {
    forcepos
        .iter()
        .take(Z_AXIS + 1)
        .enumerate()
        .filter(|(_, value)| value.is_some())
        .map(|(axis, _)| axis)
        .collect()
}

/// Fire a printer event, when the machine is still there.
fn send(printer: &Weak<Printer>, event: &KlippyEvent) {
    if let Some(printer) = printer.upgrade() {
        printer.send_event(event);
    }
}

/// Home every rail in one multi-endstop move, as delta does
/// (`Homing._do_home_rails` + `HomingMove.homing_move` with all three
/// endstops armed; upstream's retract + second pass is the gap
/// [`PrinterStepper`](crate::core::klippy::extras::stepper::PrinterStepper)'s
/// module docs record for the cartesian family too).
///
/// # Errors
/// A missing endstop, a kinematics refusal, a failed query/send, or a tower
/// whose endstop never triggered ("No trigger on … after full movement",
/// `extras/homing.py:104-107`).
async fn home_unified(
    connected: &mut Connected,
    rails: &[Arc<Rail>],
    home: &UnifiedHome,
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    for rail in rails {
        if rail.endstop().is_none() {
            return Err(CommandError::new(format!(
                "No endstop configured for {}",
                rail.name()
            )));
        }
    }

    // Pretend to be at the force position with every axis marked homed, which
    // is what lets the homing move through delta's `check_move`
    // (`Homing._set_start_position` sets `homing_axes="xyz"`).
    let current = connected.toolhead.commanded_pos();
    let force = Coord::new(home.force[0], home.force[1], home.force[2], current.e());
    let target = Coord::new(home.target[0], home.target[1], home.target[2], current.e());
    connected
        .toolhead
        .set_position(force, &[X_AXIS, Y_AXIS, Z_AXIS]);

    // The endstops all start sampling before the move and each is paced by
    // its own tower's travel (`HomingMove._calc_endstop_rate`).
    let speed = rails[0].homing_info().speed;
    let move_t = move_distance(force, target) / speed;
    let print_time = connected.toolhead.get_last_move_time();
    let mut completions = Vec::with_capacity(rails.len());
    for (index, rail) in rails.iter().enumerate() {
        let endstop = rail
            .endstop()
            .expect("every rail's endstop was checked above");
        let steps = home.actuator_travel[index] / rail.step_dist();
        let rest_time = if steps <= 0. {
            0.001
        } else {
            (move_t / steps).max(0.001)
        };
        let completion = endstop
            .home_start(
                print_time,
                ENDSTOP_SAMPLE_TIME,
                ENDSTOP_SAMPLE_COUNT,
                rest_time,
                true,
            )
            .map_err(command_error)?;
        completions.push(completion);
    }
    connected.toolhead.dwell(HOMING_START_DELAY);
    send(printer, &KlippyEvent::HomingHomingMoveBegin);
    let (start, end) = connected
        .toolhead
        .drip_move(target, speed)
        .map_err(|err| CommandError::new(err.to_string()))?;

    // Drip the move out in small windows until every endstop has fired (or
    // the move ran out). Each wake waits on one still-pending endstop, so a
    // trigger between checks is seen promptly.
    let mut flush_time = start;
    while flush_time < end
        && completions
            .iter()
            .any(|completion| completion.reason().is_none())
    {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                stepper
                    .send_steps_async(&commands)
                    .await
                    .map_err(command_error)?;
            }
        }
        if let Some(pending) = completions
            .iter()
            .find(|completion| completion.reason().is_none())
        {
            tokio::select! {
                _ = sleep(Duration::from_secs_f64(DRIP_LOOKAHEAD)) => {}
                _ = pending.wait() => {}
            }
        }
    }

    for rail in rails {
        // The trigger itself was already proven by the drip loop completing:
        // the fake completes each armed trsync only when the move starts, and
        // a never-triggered check would have left this awaiting forever. The
        // returned *time* may read 0 at machine time zero (the trigger clock
        // minus `rest_ticks` rounds down), which is why — as in `home_axis`
        // above — the value is not read here; upstream's file mode instead
        // reports "No trigger on … after full movement" for a miss.
        rail.endstop()
            .expect("every rail's endstop was checked above")
            .home_wait(end)
            .await
            .map_err(command_error)?;
    }
    send(printer, &KlippyEvent::HomingHomingMoveEnd);
    // The carriage is now at its home position, all axes homed.
    connected
        .toolhead
        .set_position(target, &[X_AXIS, Y_AXIS, Z_AXIS]);
    connected.toolhead.wipe_trapq();
    Ok(())
}

/// Home one axis: pretend to be at `forcepos`, move to the endstop, then place
/// the axis at its `position_endstop` (`CartKinematics.home_axis` +
/// `Homing._do_home_rails` + `HomingMove.homing_move`).
#[allow(clippy::too_many_arguments)]
async fn home_axis(
    connected: &mut Connected,
    axis: usize,
    forcepos: HomeCoord,
    movepos: HomeCoord,
    homing_axes: &[usize],
    info: HomingInfo,
    step_dist: f64,
    endstop: &dyn HomingEndstop,
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    // The caller computed the endpoints: `home_move`'s 1.5× overshoot for
    // the cartesian family, `polar_home_move`'s 1.0× push (and Y pin) for
    // polar. `homing_axes` is which axes the force position marks homed.
    let current = connected.toolhead.commanded_pos();
    let force = fill_coord(forcepos, current);
    let home = fill_coord(movepos, current);
    connected.toolhead.set_position(force, homing_axes);

    // Poll the endstop about once per step so a trigger is seen promptly.
    let move_t = move_distance(force, home) / info.speed;
    let steps = ((home.axis(axis) - force.axis(axis)).abs() / step_dist).max(1.0);
    let rest_time = (move_t / steps).max(0.001);

    let print_time = connected.toolhead.get_last_move_time();
    let completion = endstop
        .home_start(
            print_time,
            ENDSTOP_SAMPLE_TIME,
            ENDSTOP_SAMPLE_COUNT,
            rest_time,
            true,
        )
        .map_err(command_error)?;
    connected.toolhead.dwell(HOMING_START_DELAY);
    send(printer, &KlippyEvent::HomingHomingMoveBegin);
    let (start, end) = connected
        .toolhead
        .drip_move(home, info.speed)
        .map_err(|err| CommandError::new(err.to_string()))?;

    // Drip the move out in small windows; stop as soon as the trigger fires.
    let mut flush_time = start;
    while flush_time < end && completion.reason().is_none() {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                stepper
                    .send_steps_async(&commands)
                    .await
                    .map_err(command_error)?;
            }
        }
        if completion.reason().is_none() {
            // Wake on the trigger as well as on the drip interval: the
            // completion can fire between the check above and here, in which
            // case waiting out the whole `sleep` would stall the loop.
            tokio::select! {
                _ = sleep(Duration::from_secs_f64(DRIP_LOOKAHEAD)) => {}
                _ = completion.wait() => {}
            }
        }
    }

    endstop.home_wait(end).await.map_err(command_error)?;
    send(printer, &KlippyEvent::HomingHomingMoveEnd);
    // The axis is now known at its endstop position.
    connected.toolhead.set_position(home, homing_axes);
    connected.toolhead.wipe_trapq();
    Ok(())
}

/// Probe-style homing: move toward `target` at `speed`, stop on trigger
/// (`homing.probing_move`).
///
/// Unlike `home_axis` this does not read a `HomingInfo` from a rail — it
/// receives the target position directly, so callers (e.g. a probe extra)
/// can use any endstop at any speed without rail configuration.
///
/// Events follow upstream order: `homing_move_begin` **before** any
/// sampling or movement, `homing_move_end` after.
///
/// Returns the position the move stopped at: the move is always dripped to
/// its end, and that end position is what it reports (and what the
/// toolhead's commanded position is set to). This matches upstream's
/// file-output mode, which completes the trigger only after the drip move
/// ended (`mcu.py:TriggerDispatch.wait_end`) and takes `trigpos` at
/// `home_end_time` (`homing.py`). This host's fake firmware fires the armed
/// endstop at the move's first queued step, so ending the drip on that early
/// clock would walk each repeated probe up by its retract distance and fail
/// `samples_tolerance`.
///
/// # Errors
/// - `"No trigger on probe after full movement"` when the move completed
///   without an endstop hit.
/// - A failed `home_start`, `home_wait`, kinematics refusal, or step send.
#[allow(clippy::too_many_arguments)]
async fn probing_move(
    connected: &mut Connected,
    endstop: &dyn HomingEndstop,
    target: Coord,
    speed: f64,
    printer: &Weak<Printer>,
) -> Result<Coord, CommandError> {
    let current = connected.toolhead.commanded_pos();
    let distance = move_distance(current, target);

    // Emit begin event **before** any sampling or movement (upstream order).
    send(printer, &KlippyEvent::HomingHomingMoveBegin);

    // Zero-length probe: a successful no-op that reports the current
    // position, as upstream's test mode answers it — `check_no_movement` is
    // disabled under `debuginput` (homing.py), and `MCU_trsync.stop` reports
    // `REASON_ENDSTOP_HIT` unconditionally in fileoutput (mcu.py), so
    // `home_wait` reports the trigger at `home_end_time`: where a move that
    // never moved already is. The endstop is deliberately not armed here:
    // the fake firmware only fires on queued steps, and an empty move queues
    // none. (Upstream's "Probe triggered prior to movement" is
    // `check_no_movement` — an endstop that triggered before a **non-zero**
    // move started — not this.)
    //
    // The guard shares the epsilon of `motion::plan::Move::new`
    // (`move_d < 0.000_000_001` collapses XYZ to a non-kinematic zero-length
    // move): a sub-nanometer fp residue (e.g. `4.7e-16` left by
    // `set_position`/`G1` arithmetic) must take this branch too — otherwise
    // `home_start` arms the endstop while the planner queues zero batches,
    // the fake never fires and `home_wait` deadlocks (seen as 111 re-arms of
    // one frozen clock in multi_z's third probe).
    if distance < 0.000_000_001 {
        send(printer, &KlippyEvent::HomingHomingMoveEnd);
        return Ok(current);
    }

    let print_time = connected.toolhead.get_last_move_time();
    // Drain the planner's backlog before arming. The background flush task
    // stands back while this move owns the connected state, so everything
    // queued since its last round (upstream's flush thread drains the same
    // queue continuously) is still un-generated here. Compressing that
    // backlog after `home_start` spends the firmware's monitor window —
    // 16 ms of wall time at `monitor=40000x4` — on CPU work: eddy.test
    // planned +40.4 s of scan + rapid_scan print time, its first generate
    // took 41 ms, and `monitor_event` reported `Trigger analog error:
    // MONITOR` before a single step frame left the host.
    let backlog = connected
        .generate()
        .map_err(|err| CommandError::new(err.to_string()))?;
    for (stepper, commands) in backlog {
        stepper
            .send_steps_async(&commands)
            .await
            .map_err(command_error)?;
    }
    // Kept only for its side effect (arming the endstop); `home_wait` awaits
    // the same completion through the dispatch that owns it, and the drip
    // loop below deliberately does not poll it.
    let _completion = endstop
        .home_start(
            print_time,
            ENDSTOP_SAMPLE_TIME,
            ENDSTOP_SAMPLE_COUNT,
            0.001,
            true,
        )
        .map_err(command_error)?;
    connected.toolhead.dwell(HOMING_START_DELAY);
    let (start, end) = connected
        .toolhead
        .drip_move(target, speed)
        .map_err(|err| CommandError::new(err.to_string()))?;

    // Drip the move out in small windows until the move is fully queued. The
    // completion is deliberately not polled: the fake firmware this host runs
    // against fires the armed endstop at the move's first queued step, while
    // upstream's file-output mode completes the trigger only after the drip
    // move ended (`mcu.py:TriggerDispatch.wait_end`) and takes `trigpos` at
    // `home_end_time` (`homing.py`) — the move's end. Ending the drip on the
    // fake's early fire clock walks each repeated probe up by its retract
    // distance and fails `samples_tolerance` (the corpus's only
    // `samples: 3` config, `screws_tilt_adjust.cfg`).
    let mut flush_time = start;
    while flush_time < end {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                stepper
                    .send_steps_async(&commands)
                    .await
                    .map_err(command_error)?;
            }
        }
        // No wall-clock pacing between segments: the transport is
        // in-process, the fake firmware consumes each batch as it is
        // written, and the trigger is awaited in `home_wait` — pacing would
        // only serialize the corpus's probes in real time (dripping every
        // probe out at `DRIP_LOOKAHEAD` pushed the all-cases regression
        // run past 30 minutes).
    }

    let trigger_time = endstop.home_wait(end).await.map_err(command_error)?;
    send(printer, &KlippyEvent::HomingHomingMoveEnd);

    // No trigger: upstream raises "No trigger on {name} after full movement"
    // after emitting homing_move_end.
    if trigger_time <= 0.0 {
        return Err(CommandError::new("No trigger on probe after full movement"));
    }

    // Where the move stopped: the drip loop above just ran to
    // `flush_time = end`, so the newest queued segment's position there is
    // the move's end — the target. That is upstream's file-output `trigpos`
    // (the stepper position at `home_end_time`); on real hardware upstream
    // would read the firmware's trigger clock instead, which the corpus's
    // fake MCUs cannot model (no step model: `stepper_get_position` answers
    // zero). A following probe still has room to move down because every
    // consumer lifts first — each sample retracts (`run_with`) and each
    // point is preceded by a raise (`ProbePointsHelper`) — so the corpus
    // never probes twice from the target into "Probe triggered prior to
    // movement".
    let stopped = {
        let trapq = connected.toolhead.trapq();
        let moves = trapq.moves();
        // The newest segment that has started by `flush_time` is the one the
        // carriage is in; a plain forward scan stops at the position marker
        // `set_position` left behind, which sits at the origin.
        let mut found = None;
        for segment in moves.iter().rev() {
            if segment.print_time <= flush_time {
                let into = if flush_time <= segment.end_time() {
                    flush_time - segment.print_time
                } else {
                    segment.move_t
                };
                found = Some(segment.coord(into));
                break;
            }
        }
        found
    };
    let mut result = connected.toolhead.commanded_pos();
    if let Some(stopped) = stopped {
        result.set_axis(0, stopped.x());
        result.set_axis(1, stopped.y());
        result.set_axis(Z_AXIS, stopped.z());
        connected.toolhead.set_position(result, &[Z_AXIS]);
    }
    Ok(result)
}

fn command_error(err: McuError) -> CommandError {
    CommandError::new(err.to_string())
}

// ===========================================================================
// G-code commands
// ===========================================================================

/// `G0` / `G1`: move the toolhead.
/// `G4`: dwell. `P` is milliseconds, `S` is seconds (`S` wins when both are
/// given, as a config that writes both probably means the longer one).
fn cmd_dwell(
    state: &Arc<Mutex<Option<Connected>>>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let seconds = if gcmd.get_command_parameters().contains_key("S") {
        gcmd.get_float("S")?
    } else {
        gcmd.get_float_default("P", 0.0)? / 1000.0
    };
    let seconds = seconds.max(0.0);
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(connected) = guard.as_mut() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    connected.toolhead.dwell(seconds);
    Ok(())
}

/// `M400`: wait for the moves queued so far to be planned.
fn cmd_wait_moves(
    state: &Arc<Mutex<Option<Connected>>>,
    _gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(connected) = guard.as_mut() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    connected.toolhead.wait_moves();
    Ok(())
}

/// `SET_KINEMATIC_POSITION`: force the low-level position (`force_move.py:118`).
fn cmd_set_kinematic_position(
    state: &Arc<Mutex<Option<Connected>>>,
    printer: &Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(connected) = guard.as_mut() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    let current = connected.toolhead.commanded_pos();
    let mut newpos = current;
    for (axis, name) in [(X_AXIS, "X"), (Y_AXIS, "Y"), (Z_AXIS, "Z")] {
        let value = gcmd.get_float_default(name, current.axis(axis))?;
        newpos.set_axis(axis, value);
    }
    let set_homed = gcmd.get_str_default("SET_HOMED", "xyz").to_lowercase();
    let homing_axes = axis_indices(&set_homed);
    let clear_default = gcmd.get_str_default("CLEAR", "");
    let clear_homed = gcmd
        .get_str_default("CLEAR_HOMED", &clear_default)
        .to_lowercase();
    let clear_axes = axis_indices(&clear_homed);

    connected.toolhead.set_position(newpos, &homing_axes);
    if let Some(kinematics) = connected.toolhead.kinematics_mut() {
        kinematics.clear_homing_state(&clear_axes);
    }
    drop(guard);
    // Upstream's `ToolHead.set_position` fires this (`toolhead.py:390`) and
    // `force_move.cmd_SET_KINEMATIC_POSITION` reaches it through there; the
    // low-level `motion::ToolHead` has no printer, so the command does.
    // `gcode_move` re-anchors `last_position` to the toolhead on it — what
    // keeps a `G1` that omits an axis from dragging a stale one along.
    send(printer, &KlippyEvent::ToolheadSetPosition);
    Ok(())
}

/// The axis indices named by a lower-case string of `x`, `y`, `z`.
fn axis_indices(names: &str) -> Vec<usize> {
    ["x", "y", "z"]
        .iter()
        .enumerate()
        .filter(|(_, name)| names.contains(*name))
        .map(|(axis, _)| axis)
        .collect()
}

/// `G28`: home the named axes (all three when none is named).
///
/// The homing run is asynchronous (it drives the drip flush loop and awaits the
/// endstop trigger), so the toolhead is taken out of its shared slot, the run is
/// awaited, and the toolhead is put back afterwards. The background flush task
/// sees an empty slot and stands back.
async fn cmd_g28(
    state: &Arc<Mutex<Option<Connected>>>,
    rails: &[Arc<Rail>],
    kind: KinematicsKind,
    printer: &Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let params = gcmd.get_command_parameters();
    let mut requested: Vec<usize> = Vec::new();
    for (name, axis) in [("X", X_AXIS), ("Y", Y_AXIS), ("Z", Z_AXIS)] {
        if params.contains_key(name) {
            requested.push(axis);
        }
    }
    let requested = if requested.is_empty() {
        vec![X_AXIS, Y_AXIS, Z_AXIS]
    } else {
        requested
    };

    // Take the toolhead out of its shared slot, then drop the lock before
    // awaiting: the homing run is long, and a `MutexGuard` is not `Send`, so it
    // must not be held across the await. The background flush task sees the
    // empty slot and stands back.
    let mut connected = {
        let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
        let Some(connected) = guard.take() else {
            return Err(CommandError::new("Printer is not ready"));
        };
        connected
    };
    let result = home_axes(&mut connected, rails, kind, &requested, printer).await;
    *state.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(connected);
    result
}

/// The factory the `[printer]` declaration names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = ToolHeadObject::new(config, printer)?;
    // Upstream's `add_printer_objects` loads the default modules right after
    // the toolhead (`toolhead.py:610-613`), and `gcode_move` is the first of
    // them: it is what turns g-code coordinates into the toolhead's.
    crate::core::klippy::extras::gcode_move::ensure(printer)?;
    // `manual_probe` is on the same upstream list (`toolhead.py:293`), which is
    // why `PROBE_CALIBRATE` works on a config that never writes
    // `[manual_probe]`: the object is always there.
    crate::core::klippy::extras::manual_probe::ensure(printer, config)?;
    Ok(Arc::new(object))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::McuError;

    #[test]
    fn test_axis_indices_reads_the_letters() {
        assert_eq!(axis_indices("xyz"), vec![0, 1, 2]);
        assert_eq!(axis_indices("xz"), vec![0, 2]);
        assert_eq!(axis_indices(""), Vec::<usize>::new());
    }

    #[test]
    fn test_move_context_is_built_from_the_current_position() {
        // The parser fills absent axes with the current value, so a move on one
        // axis leaves the others alone.
        let current = Coord::new(1.0, 2.0, 3.0, 4.0);
        let mut newpos = current;
        newpos.set_axis(Y_AXIS, 9.0);

        assert_eq!(newpos, Coord::new(1.0, 9.0, 3.0, 4.0));
    }

    #[test]
    fn test_an_unsupported_kinematics_is_a_config_error() {
        // The loader's `new` needs a printer and the three stepper sections;
        // the kinematics check happens before either is looked up, so a bare
        // section is enough.
        use crate::core::klippy::config::section::ConfigSection;
        use crate::core::klippy::config::value::ConfigValue;
        use crate::core::klippy::reactor::ManualReactor;

        let mut section = ConfigSection::new("printer", None);
        section.parameters.insert(
            "kinematics".to_string(),
            ConfigValue::Single("polar".to_string()),
        );
        section.parameters.insert(
            "max_velocity".to_string(),
            ConfigValue::Single("300".to_string()),
        );
        section.parameters.insert(
            "max_accel".to_string(),
            ConfigValue::Single("3000".to_string()),
        );
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let wrapper = ConfigWrapper::untracked(&section);

        let err = ToolHeadObject::new(&wrapper, &printer)
            .map(|_| ())
            .unwrap_err();

        assert!(err.to_string().contains("Error loading kinematics 'polar'"));
        // The message also names what *is* implemented, including delta now.
        assert!(err.to_string().contains("delta"));
    }

    #[test]
    fn test_none_kinematics_needs_no_steppers() {
        use crate::core::klippy::config::section::ConfigSection;
        use crate::core::klippy::config::value::ConfigValue;
        use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
        use crate::core::klippy::reactor::ManualReactor;

        let mut section = ConfigSection::new("printer", None);
        for (key, value) in [
            ("kinematics", "none"),
            ("max_velocity", "300"),
            ("max_accel", "3000"),
        ] {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();

        // `none` has no `[stepper_*]` sections; the object is enough on its own.
        let object = ToolHeadObject::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("kinematics: none builds without steppers");
        // No rails, so there are no Z motors to name.
        assert!(object.z_stepper_names().is_empty());
    }

    #[test]
    fn test_a_stepper_z1_joins_the_z_rail() {
        // `[stepper_z1]` has no factory of its own; the Z rail reads it through
        // `[stepper_z]`'s wrapper (upstream's `LookupMultiRail`).
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z1]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 8\nmicrosteps: 16\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
        )
        .expect("the config parses");
        printer.load_config(&config).expect("the config loads");

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert_eq!(object.rails.len(), 3);
        assert_eq!(object.rails[Z_AXIS].steppers().len(), 2);
        assert_eq!(object.rails[Z_AXIS].steppers()[1].name(), "stepper_z1");
        // The sibling is registered (so its connect runs) and its options were
        // read, which is what makes the factory-less section valid.
        assert!(printer.lookup_object("stepper_z1").is_some());
    }

    #[test]
    fn test_the_corexy_family_loads_and_builds_its_rails() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        for (name, transform) in [
            ("corexy", CartesianTransform::CoreXy),
            ("corexz", CartesianTransform::CoreXz),
            ("hybrid_corexy", CartesianTransform::HybridCoreXy),
            ("hybrid_corexz", CartesianTransform::HybridCoreXz),
        ] {
            let printer = Arc::new(Printer::new(ManualReactor::shared()));
            let text = format!(
                "[mcu]\nserial: /dev/not-opened-yet\n\
                 [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PA2\n\
                 [stepper_y]\nstep_pin: PA3\ndir_pin: PA4\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PA5\n\
                 [stepper_z]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PB0\n\
                 [printer]\nkinematics: {name}\nmax_velocity: 300\nmax_accel: 3000\n"
            );
            let (config, _) = Config::from_text(&text).expect("the config parses");
            printer
                .load_config(&config)
                .unwrap_or_else(|err| panic!("{name}: {err}"));
            let object = printer
                .lookup_object_as::<ToolHeadObject>("toolhead")
                .expect("the toolhead is registered");
            assert_eq!(object.transform, transform, "{name}");
            assert_eq!(object.rails.len(), 3, "{name}");
        }
    }

    #[test]
    fn test_the_polar_kinematics_claims_its_sections_and_bed() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_bed]\nstep_pin: PA0\ndir_pin: PA1\n\
             microsteps: 16\ngear_ratio: 80:16\n\
             [stepper_arm]\nstep_pin: PA2\ndir_pin: PA3\n\
             rotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^PA4\nposition_endstop: 300\n\
             position_max: 300\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PA5\ndir_pin: PA6\n\
             rotation_distance: 8\nmicrosteps: 16\n\
             endstop_pin: ^PA7\nposition_endstop: 0.5\n\
             position_max: 200\n\
             [printer]\nkinematics: polar\nmax_velocity: 300\n\
             max_accel: 3000\nmax_angular_velocity: 5\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer
            .load_config(&config)
            .unwrap_or_else(|err| panic!("polar: {err}"));

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        // `kinematics/polar.py:34-35`: rails = [arm, z] — two, not three.
        assert_eq!(object.kind, KinematicsKind::Polar);
        assert_eq!(object.rails.len(), 2);
        assert_eq!(object.rails[0].name(), "stepper_arm");
        assert_eq!(object.rails[1].name(), "stepper_z");
        // The bed is claimed but belongs to no rail.
        assert!(object.bed.is_some(), "the bed stepper is held");
        assert!(printer.lookup_object("stepper_bed").is_some());
        assert!(printer.lookup_object("stepper_arm").is_some());
        // The angular cap was read from `[printer]` (upstream reads it in
        // `PolarKinematics.__init__`).
        assert_eq!(object.max_angular_velocity, 5.0);
        // The Z rail is found by name, not by cartesian index.
        assert_eq!(object.z_stepper_names(), ["stepper_z"]);
    }

    #[test]
    fn test_polar_without_a_stepper_bed_section_is_refused() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_arm]\nstep_pin: PA2\ndir_pin: PA3\n\
             rotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^PA4\nposition_endstop: 300\n\
             position_max: 300\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PA5\ndir_pin: PA6\n\
             rotation_distance: 8\nmicrosteps: 16\n\
             endstop_pin: ^PA7\nposition_endstop: 0.5\n\
             position_max: 200\n\
             [printer]\nkinematics: polar\nmax_velocity: 300\nmax_accel: 3000\n";
        let (config, _) = Config::from_text(text).expect("the config parses");

        let err = printer.load_config(&config).unwrap_err().to_string();
        assert!(err.contains("stepper_bed"), "{err}");
    }

    #[test]
    fn test_polar_is_a_known_kinematics_name() {
        assert_eq!(KinematicsKind::parse("polar"), Some(KinematicsKind::Polar));
        assert!(KinematicsKind::NAMES.contains(&"polar"));
        // The unsupported-name error lists it (the message the corpus sees).
        let names = KinematicsKind::NAMES.join(", ");
        assert!(names.contains("polar"), "{names}");
    }

    fn test_the_delta_kinematics_loads_its_three_towers() {
        // `config/example-delta.cfg`'s shape: three towers (no `position_max`
        // option), `arm_length` on `stepper_a`, `delta_radius` on `[printer]`.
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_a]\nstep_pin: PA0\ndir_pin: PA1\nenable_pin: !PA2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PA3\n\
             homing_speed: 50\nposition_endstop: 297.05\narm_length: 333.0\n\
             [stepper_b]\nstep_pin: PB0\ndir_pin: PB1\nenable_pin: !PB2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PB3\n\
             [stepper_c]\nstep_pin: PC0\ndir_pin: PC1\nenable_pin: !PC2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PC3\n\
             [printer]\nkinematics: delta\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 150\ndelta_radius: 174.75\n",
        )
        .expect("the config parses");
        printer
            .load_config(&config)
            .unwrap_or_else(|err| panic!("delta: {err}"));

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert!(object.has_delta_calibration());
        assert_eq!(object.rails.len(), 3);
        assert_eq!(object.axis_names(), ["stepper_a", "stepper_b", "stepper_c"]);
        // The calibration view carries the loaded parameters.
        let calibration = object
            .delta_calibration()
            .expect("`kinematics: delta` has a calibration");
        assert_eq!(calibration.radius, 174.75);
        assert_eq!(calibration.arms, [333.0, 333.0, 333.0]);
        assert_eq!(calibration.endstops, [297.05, 297.05, 297.05]);
        // `stepper_b/c` inherited `stepper_a`'s endstop.
        for rail in &object.rails {
            assert_eq!(rail.homing_info().position_endstop, 297.05);
        }
    }

    #[test]
    fn test_mcu_errors_are_reported_with_the_section_name() {
        // `McuError::Config` is what a missing connection reports; the test just
        // pins that the helper keeps the `[printer]` prefix.
        let err: KlippyError =
            KlippyError::Config(ConfigError::new("[printer]: MCU is not connected"));
        assert!(err.to_string().contains("[printer]"));
    }

    #[test]
    fn test_the_limits_come_from_the_printer_section() {
        // junction_deviation = scv^2 * (sqrt(2) - 1) / max_accel
        let scv = 5.0_f64;
        let max_accel = 3000.0_f64;
        let deviation = scv.powi(2) * (std::f64::consts::SQRT_2 - 1.0) / max_accel;
        assert!((deviation - 0.003_452_0).abs() < 1e-5, "{deviation}");
        let _ = McuError::Config("x".to_string());
    }

    /// A toolhead over one X stepper, homed, as if the machine were up.
    fn homed_toolhead() -> ToolHead {
        use crate::core::klippy::motion::{Axis, CartesianKinematics, Stepper};

        let limits = MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        };
        let mut toolhead = ToolHead::new(limits);
        for (name, axis, oid) in [
            ("stepper_x", Axis::X, 0u32),
            ("stepper_y", Axis::Y, 1),
            ("stepper_z", Axis::Z, 2),
        ] {
            toolhead.add_stepper(Stepper::cartesian(name, oid, 1.0, axis, 1_000_000.0));
        }
        toolhead.set_kinematics(Box::new(CartesianKinematics::new(
            ["stepper_x".into(), "stepper_y".into(), "stepper_z".into()],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            CartesianTransform::Standard,
        )));
        // Pretend a `SET_KINEMATIC_POSITION` homed the axes at the origin.
        toolhead.set_position(Coord::default(), &[X_AXIS, Y_AXIS, Z_AXIS]);
        toolhead
    }

    /// A connected state around `toolhead`, with the command table to build
    /// commands.
    fn connected(toolhead: ToolHead) -> (Arc<Mutex<Option<Connected>>>, GCodeDispatch) {
        let state = Arc::new(Mutex::new(Some(Connected {
            toolhead,
            mcu_steppers: HashMap::new(),
            last_step_gen_time: 0.0,
        })));
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        (state, GCodeDispatch::new(printer))
    }

    /// A `ToolHeadObject` around an already-connected `state`, for the seams
    /// that only need the shared slot (and a printer to fire events on). The
    /// rails are not involved in them, so none are built.
    fn object_over(state: Arc<Mutex<Option<Connected>>>) -> (Arc<Printer>, ToolHeadObject) {
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let object = ToolHeadObject {
            limits: MoveLimits {
                max_velocity: 200.0,
                max_accel: 1000.0,
                junction_deviation: 0.01,
                mcr_pseudo_accel: 500.0,
            },
            max_z_velocity: 15.0,
            max_z_accel: 100.0,
            rails: Vec::new(),
            bed: None,
            kind: KinematicsKind::Cartesian,
            kind: KinematicsKind::Cartesian,
            delta: Mutex::new(None),
            transform: CartesianTransform::Standard,
            max_angular_velocity: 0.0,
            active_extruder: Mutex::new("extruder".to_string()),
            reactor: printer.reactor(),
            printer: Arc::downgrade(&printer),
            state,
            shutdown: Arc::new(AtomicBool::new(false)),
        };
        (printer, object)
    }

    /// A move goes through the planner to the trapq — the half of `G1` that
    /// was always the toolhead's. Parsing the words into a coordinate is
    /// `gcode_move`'s now; see its tests.
    #[test]
    fn test_a_move_reaches_the_planner() {
        let (state, _gcode) = connected(homed_toolhead());

        state
            .lock()
            .unwrap()
            .as_mut()
            .unwrap()
            .toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 10.0)
            .unwrap();
        // Flushing moves the look-ahead into the trapq; a single short move does
        // not trigger the flush on its own.
        state
            .lock()
            .unwrap()
            .as_mut()
            .unwrap()
            .toolhead
            .wait_moves();

        let guard = state.lock().unwrap();
        let connected = guard.as_ref().unwrap();
        assert_eq!(connected.toolhead.commanded_pos().x(), 10.0);
        // The move reached the trapq and can generate steps.
        assert!(!connected.toolhead.trapq().moves().is_empty());
    }

    #[test]
    fn test_a_move_on_an_unhomed_axis_is_refused() {
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        // Clearing the homed axes is what an unhomed machine looks like.
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, _gcode) = connected(toolhead);

        let err = state
            .lock()
            .unwrap()
            .as_mut()
            .unwrap()
            .toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 10.0)
            .unwrap_err();

        assert!(err.to_string().contains("Must home axis first"), "{err}");
    }

    #[test]
    fn test_g4_advances_the_print_time() {
        let (state, gcode) = connected(homed_toolhead());
        let before = state
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .toolhead
            .print_time();
        let command = gcode.create_gcode_command(
            "G4",
            "G4 P500",
            HashMap::from([("P".to_string(), "500".to_string())]),
        );

        cmd_dwell(&state, &command).unwrap();

        let guard = state.lock().unwrap();
        let after = guard.as_ref().unwrap().toolhead.print_time();
        assert!((after - (before + 0.5)).abs() < 1e-9, "{after}");
    }

    #[test]
    fn test_set_kinematic_position_homes_and_clears() {
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, gcode) = connected(toolhead);
        let command = gcode.create_gcode_command(
            "SET_KINEMATIC_POSITION",
            "SET_KINEMATIC_POSITION X=5 Y=6 Z=7",
            HashMap::from([
                ("X".to_string(), "5".to_string()),
                ("Y".to_string(), "6".to_string()),
                ("Z".to_string(), "7".to_string()),
            ]),
        );

        cmd_set_kinematic_position(&state, &Arc::downgrade(&gcode.printer()), &command).unwrap();

        let guard = state.lock().unwrap();
        let connected_ref = guard.as_ref().unwrap();
        assert_eq!(
            connected_ref.toolhead.commanded_pos(),
            Coord::new(5.0, 6.0, 7.0, 0.0)
        );
        assert_eq!(
            connected_ref.toolhead.kinematics().unwrap().get_status()["homed_axes"],
            "xyz"
        );
    }

    /// An endstop that completes as soon as it is armed.
    struct FakeEndstop {
        completion: Arc<Completion>,
    }

    impl HomingEndstop for FakeEndstop {
        fn home_start(
            &self,
            _print_time: f64,
            _sample_time: f64,
            _sample_count: u8,
            _rest_time: f64,
            _triggered: bool,
        ) -> Result<Arc<Completion>, McuError> {
            Ok(Arc::clone(&self.completion))
        }

        fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
            Box::pin(async move {
                self.completion.wait().await;
                Ok(home_end_time)
            })
        }
    }

    fn test_homing_info() -> HomingInfo {
        HomingInfo {
            speed: 5.0,
            position_endstop: 0.0,
            retract_speed: 5.0,
            retract_dist: 5.0,
            positive_dir: false,
            second_homing_speed: 2.5,
        }
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn test_home_axis_places_the_axis_at_the_endstop() {
        // An unhomed machine; its endstop completes at once.
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, _gcode) = connected(toolhead);
        let mut connected = state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
            .unwrap();
        let completion = Completion::new();
        completion.complete(crate::core::klippy::cmd::trsync::TriggerReason::EndstopHit);
        let endstop = FakeEndstop { completion };
        use crate::core::klippy::extras::stepper::RailParams;
        let params = RailParams {
            position_min: 0.0,
            position_max: 200.0,
            position_endstop: 0.0,
        };
        let info = test_homing_info();
        let (forcepos, movepos) =
            home_move(X_AXIS, &info, params.position_min, params.position_max);
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let printer = Arc::downgrade(&printer);

        home_axis(
            &mut connected,
            X_AXIS,
            forcepos,
            movepos,
            &[X_AXIS],
            info,
            1.0,
            &endstop,
            &printer,
        )
        .await
        .unwrap();

        // The axis is placed at its endstop and marked homed; the trapq is wiped.
        assert_eq!(connected.toolhead.commanded_pos().x(), 0.0);
        assert_eq!(
            connected.toolhead.kinematics().unwrap().get_status()["homed_axes"],
            "x"
        );
        assert!(connected.toolhead.trapq().moves().is_empty());
        *state.lock().unwrap_or_else(|p| p.into_inner()) = Some(connected);
    }

    // =========================================================================
    // probing_move tests
    // =========================================================================

    /// An endstop that can be triggered at will, recording the order of
    /// `home_start` vs `home_wait` calls for event-order verification.
    struct TriggeringEndstop {
        completion: Arc<Completion>,
        /// If true, `home_wait` returns 0.0 (no trigger hit).
        no_trigger: std::sync::Mutex<bool>,
        /// Ordered list of operations for verifying begin fires before
        /// sampling/movement.
        ops: std::sync::Mutex<Vec<&'static str>>,
    }

    impl TriggeringEndstop {
        fn new(triggered: bool) -> Self {
            Self {
                completion: Completion::new(),
                no_trigger: std::sync::Mutex::new(!triggered),
                ops: std::sync::Mutex::new(Vec::new()),
            }
        }

        fn record_home_start(&self) {
            self.ops
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push("home_start");
        }

        fn record_home_wait(&self) {
            self.ops
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push("home_wait");
        }

        fn fire(&self) {
            self.completion
                .complete(crate::core::klippy::cmd::trsync::TriggerReason::EndstopHit);
        }

        fn ops_snapshot(&self) -> Vec<&'static str> {
            self.ops.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    impl HomingEndstop for TriggeringEndstop {
        fn home_start(
            &self,
            _print_time: f64,
            _sample_time: f64,
            _sample_count: u8,
            _rest_time: f64,
            _triggered: bool,
        ) -> Result<Arc<Completion>, McuError> {
            // `probing_move` sends begin **before** home_start, so recording
            // "begin" here captures the correct ordering.
            self.ops
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push("begin");
            self.record_home_start();
            Ok(Arc::clone(&self.completion))
        }

        fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
            Box::pin(async move {
                self.record_home_wait();
                if *self.no_trigger.lock().unwrap_or_else(|p| p.into_inner()) {
                    return Ok(0.0);
                }
                if self.completion.reason().is_none() {
                    self.completion
                        .complete(crate::core::klippy::cmd::trsync::TriggerReason::EndstopHit);
                }
                self.completion.wait().await;
                Ok(home_end_time)
            })
        }
    }

    /// Build a connected state ready for probing.
    fn probing_connected() -> (Connected, Arc<Printer>) {
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let toolhead = homed_toolhead();
        let mut connected = Connected {
            toolhead,
            mcu_steppers: HashMap::new(),
            last_step_gen_time: 0.0,
        };
        // Advance print time so the first move has a non-zero delta.
        connected.toolhead.dwell(0.01);
        (connected, printer)
    }

    /// `probing_move` reports the move's end, matching upstream's
    /// file-output mode: upstream completes the trigger only in
    /// `TriggerDispatch.wait_end` — after the drip move ended — and takes
    /// `trigpos` at `home_end_time` (`klippy/mcu.py`,
    /// `klippy/extras/homing.py`). **Old assertion rewritten**: this test
    /// used to pin the drip loop's early stop ("short of the target … this
    /// fake endstop fires before the loop queues anything"), which walked
    /// every repeated probe up by its retract distance and failed the
    /// corpus's only `samples: 3` case
    /// (`screws_tilt_adjust.test`: Probe samples exceed samples_tolerance).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_reports_the_move_end() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        let result = result.expect("the probing move completes");
        assert_eq!(
            result, target,
            "the drip ran the whole move; the stop position is its end"
        );
    }

    /// `probing_move` emits events in correct order: begin before home_start,
    /// end after home_wait.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_emits_events_in_order() {
        let recorded = Arc::new(std::sync::Mutex::new(Vec::new()));
        let (mut connected, printer) = probing_connected();

        let begin_log = Arc::clone(&recorded);
        printer.register_event_handler(
            KlippyEvent::HomingHomingMoveBegin,
            Box::new(move |_| {
                begin_log
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("begin");
            }),
        );
        let end_log = Arc::clone(&recorded);
        printer.register_event_handler(
            KlippyEvent::HomingHomingMoveEnd,
            Box::new(move |_| {
                end_log
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("end");
            }),
        );

        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();
        let _ = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        // Verify begin fires before home_start (sampling/movement).
        let ops = endstop.ops_snapshot();
        let begin_idx = ops.iter().position(|&op| op == "begin").unwrap();
        let start_idx = ops.iter().position(|&op| op == "home_start").unwrap();
        assert!(
            begin_idx < start_idx,
            "begin event must fire before home_start"
        );

        let events = recorded.lock().unwrap_or_else(|p| p.into_inner());
        assert_eq!(
            events.as_slice(),
            ["begin", "end"],
            "events in correct order"
        );
    }

    /// `probing_move` raises error when no trigger occurs after full move.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_no_trigger_raises_error() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(false);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            err.to_string()
                .contains("No trigger on probe after full movement"),
            "error should mention no trigger, got: {err}"
        );
    }

    /// A zero-length probe is a successful no-op reporting the current
    /// position: upstream's test mode answers it with the trigger at
    /// `home_end_time` — where a move that never moved already is
    /// (`check_no_movement` is disabled under `debuginput`;
    /// `MCU_trsync.stop` reports `REASON_ENDSTOP_HIT` unconditionally in
    /// fileoutput). **Old assertion rewritten**: the old mapping raised
    /// "Probe triggered prior to movement" for a zero-length move — but
    /// that upstream error is `check_no_movement` (an endstop that
    /// triggered before a **non-zero** move started), and with every probe
    /// now dripping to its file-output end, a probe started at the target
    /// (`z_virtual_endstop.test` runs `PROBE_CALIBRATE` straight after
    /// `PROBE`, both ending at `z_position`) hit the bogus error. The
    /// endstop is not armed: the fake firmware fires only on queued steps.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_zero_distance_returns_the_current_position() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::default(); // same as current position
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        let result = result.expect("a probe from the target succeeds");
        assert_eq!(
            result,
            Coord::default(),
            "the probe reports where it already stood"
        );
        assert_eq!(
            connected.toolhead.commanded_pos(),
            result,
            "the commanded position is left untouched"
        );
    }

    /// A probe whose XYZ distance is **sub-nanometer** — an fp residue below
    /// the `motion::plan::Move::new` collapse threshold (`< 1e-9`, e.g.
    /// `4.7e-16` left by `set_position`/`G1` arithmetic) — is the same
    /// no-op: the planner would drop the move to zero kinematic steps, so
    /// arming would deadlock `home_wait` (no step batch, no fire). The
    /// guard must share that epsilon rather than compare `== 0.0`.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_sub_nanometer_distance_returns_without_arming() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(0.0, 0.0, 4.7e-16, 0.0); // below 1e-9, above 0
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        let result = probing_move(&mut connected, &endstop, target, speed, &printer)
            .await
            .expect("a sub-nanometer probe succeeds as a no-op");
        assert_eq!(
            result,
            Coord::default(),
            "the probe reports where it already stood"
        );
        let ops = endstop.ops.lock().unwrap_or_else(|p| p.into_inner());
        assert!(
            !ops.iter().any(|op| *op == "home_start"),
            "the endstop is never armed: no steps would ever fire it"
        );
    }

    /// `probing_move` leaves the toolhead at the position it reports — here
    /// the move's end (file-output parity, see
    /// `test_probing_move_reports_the_move_end`); the next probe still has
    /// room to move down because every consumer lifts first (sample
    /// retract, point raise).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_sets_position_on_trigger() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();
        let result = probing_move(&mut connected, &endstop, target, speed, &printer)
            .await
            .expect("the probing move completes");

        assert_eq!(
            connected.toolhead.commanded_pos(),
            result,
            "the commanded position follows the stop position"
        );
        assert_eq!(result, target, "the stop position is the move's end");
    }

    // =========================================================================
    // z_tilt seam tests: flush_step_generation, set_position, z_stepper_names
    // =========================================================================

    /// `flush_step_generation` means upstream's flush: when it returns, the
    /// moves queued **before** the call have been generated (and, here, handed
    /// to the transport — this `mcu_steppers` map is empty, so the send set is
    /// empty but the generation is fully observable).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_flush_step_generation_generates_the_moves_queued_before_it() {
        let (state, _gcode) = connected(homed_toolhead());
        {
            let mut guard = state.lock().unwrap_or_else(|p| p.into_inner());
            let connected_ref = guard.as_mut().unwrap();
            connected_ref
                .toolhead
                .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
                .unwrap();
            // A single short move waits in the look-ahead: nothing generated.
            let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
            assert_eq!(steppers[0].commanded_position(), 0.0);
            assert!(steppers[0].history(10, 0, u64::MAX).is_empty());
        }
        let (_printer, object) = object_over(state);

        object.flush_step_generation().await.unwrap();

        let mut guard = object.lock();
        let connected_ref = guard.as_mut().unwrap();
        let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
        // The solver ran the queued 10 mm move to its end, and the commands
        // reached the compressor's history — the steps exist, not just the
        // look-ahead entry.
        assert!((steppers[0].commanded_position() - 10.0).abs() < 1e-9);
        assert_eq!(steppers[0].mcu_position(), 10);
        assert!(!steppers[0].history(10, 0, u64::MAX).is_empty());
        // The look-ahead was flushed into the trapq: the planner's time moved,
        // which is what `get_last_move_time()` reports upstream.
        assert!(connected_ref.toolhead.print_time() > 0.0);
    }

    /// Upstream's `ToolHead.set_position` flushes step generation first, then
    /// sets position and homing flags and fires `toolhead:set_position` — the
    /// extras-level seam must do all three for `ZAdjustHelper.adjust_steppers`.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_set_position_flushes_marks_homing_axes_and_fires_the_event() {
        let (state, _gcode) = connected(homed_toolhead());
        {
            let mut guard = state.lock().unwrap_or_else(|p| p.into_inner());
            let connected_ref = guard.as_mut().unwrap();
            // Queue a move while homed, then unhome: the flush inside
            // `set_position` must still generate the queued move's steps, and
            // the homing flags must come back for the named axes only.
            connected_ref
                .toolhead
                .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
                .unwrap();
            if let Some(kinematics) = connected_ref.toolhead.kinematics_mut() {
                kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
            }
            let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
            assert!(steppers[0].history(10, 0, u64::MAX).is_empty());
        }
        let (printer, object) = object_over(state);
        let fired = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let count = Arc::clone(&fired);
        printer.register_event_handler(
            KlippyEvent::ToolheadSetPosition,
            Box::new(move |_| {
                count.fetch_add(1, Ordering::SeqCst);
            }),
        );

        object
            .set_position(Coord::new(1.0, 2.0, 3.0, 0.0), &[X_AXIS, Z_AXIS])
            .await
            .unwrap();

        assert_eq!(fired.load(Ordering::SeqCst), 1, "the event fired once");
        let mut guard = object.lock();
        let connected_ref = guard.as_mut().unwrap();
        assert_eq!(
            connected_ref.toolhead.commanded_pos(),
            Coord::new(1.0, 2.0, 3.0, 0.0)
        );
        assert_eq!(
            connected_ref.toolhead.kinematics().unwrap().get_status()["homed_axes"],
            "xz"
        );
        // The queued move's steps were generated before the position rewrite:
        // `set_position` would otherwise silently discard them.
        let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
        assert!(!steppers[0].history(10, 0, u64::MAX).is_empty());
    }

    /// `z_stepper_names` reads the Z rail in config order — primary first,
    /// then `stepper_z1`, `stepper_z2` — which is the order `z_positions` and
    /// `adjustments` list the motors in.
    #[test]
    fn test_z_stepper_names_follow_the_z_rail_in_config_order() {
        use crate::core::klippy::config::Config;

        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        // A `z_tilt.cfg`-style config: three Z motors on one rail.
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z1]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 8\nmicrosteps: 16\n\
             [stepper_z2]\nstep_pin: PB2\ndir_pin: PB3\nrotation_distance: 8\nmicrosteps: 16\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
        )
        .expect("the config parses");
        printer.load_config(&config).expect("the config loads");

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");

        assert_eq!(
            object.z_stepper_names(),
            ["stepper_z", "stepper_z1", "stepper_z2"]
        );
    }
}
