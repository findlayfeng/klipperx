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
//!
//! The step generators (`itersolve`, `stepcompress`) and the output scheduler
//! (`MotionQueuing`) are FW5c/FW5d; nothing here talks to an MCU.

pub mod plan;
pub mod trapq;

pub use plan::{LookAheadQueue, Move, MoveLimits};
pub use trapq::{MoveSegment, Trapq};
