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
//!
//! The output scheduler (`MotionQueuing`) is FW5d; nothing here talks to an MCU.

pub mod itersolve;
pub mod plan;
pub mod stepcompress;
pub mod trapq;

pub use itersolve::{
    cartesian_active_flags, cartesian_position_fn, Axis, AxisFlags, StepKinematics,
};
pub use plan::{LookAheadQueue, Move, MoveLimits};
pub use stepcompress::{GracePolicy, StepCommand, StepCompressor};
pub use trapq::{MoveSegment, Trapq};
