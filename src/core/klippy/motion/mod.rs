//! The motion stack: planning, the trapezoid queue, and (later) step generation.
//!
//! Upstream spreads this over `klippy/toolhead.py` (the planner), the C
//! `chelper/` (the trapq and the step generators) and
//! `klippy/extras/motion_quuing.py` (the output scheduler). Here it is one
//! module, split by layer:
//!
//! * [`plan`] — [`Move`](plan::Move) and [`LookAheadQueue`](plan::LookAheadQueue):
//!   the host-side planner that decides the velocity of every move.
//! * [`trapq`] — the trapezoidal velocity queue the step generators read.
//! * [`itersolve`] — the per-stepper position solver that finds step times.
//! * [`stepcompress`] — turning those step times into `queue_step` commands
//!   (the **simplified** one for now; the compressor is FW5f).
//! * [`stepper`] — one stepper: its solver and compressor.
//! * [`queuing`] — the trapq plus the steppers reading it.
//! * [`toolhead`] — the planner's print time and the moves it feeds in.
//!
//! The MCU side (the oid's `config_stepper`, sending `queue_step` over the
//! wire) and the kinematics trait are FW5e; nothing here talks to an MCU.

pub mod delta;
pub mod extra;
pub mod generic_cartesian;
pub mod itersolve;
pub mod kinematics;
pub mod plan;
pub mod queuing;
pub mod stepcompress;
pub mod stepper;
pub mod toolhead;
pub mod trapq;
pub mod winch;

pub use extra::ExtraAxis;
pub use itersolve::{
    cartesian_active_flags, cartesian_position_fn, corexy_active_flags, corexy_position_fn,
    corexz_active_flags, corexz_position_fn, extruder_position_fn, Axis, AxisFlags, StepKinematics,
};
pub use kinematics::{
    CartesianKinematics, CartesianTransform, HomeCoord, Homing, HomingHandle, HomingInfo,
    HomingState, Kinematics, MoveContext, NoneKinematics,
};
pub use plan::{LookAheadQueue, Move, MoveLimits};
pub use queuing::MotionQueuing;
pub use stepcompress::{GracePolicy, HistoryStep, StepCommand, StepCompressError, StepCompressor};
pub use stepper::Stepper;
pub use toolhead::ToolHead;
pub use trapq::{MoveSegment, Trapq};
pub use winch::{winch_active_flags, winch_position_fn, WinchKinematics};
