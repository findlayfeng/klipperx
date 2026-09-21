//! The motion output queue.
//!
//! Upstream's `klippy/extras/motion_quuing.py`: it owns the trapq and the
//! steppers, and knows how to turn the queued moves into step commands. The
//! real one paces itself with a reactor timer and sends over the MCU; this one
//! is driven explicitly (`generate`) so the host logic can be tested without a
//! device, and the MCU side attaches on top of it.

use super::plan::Move;
use super::stepcompress::{StepCommand, StepCompressError};
use super::stepper::Stepper;
use super::trapq::Trapq;
use crate::core::klippy::mathutil::Xyz;

/// The trapq and the steppers reading it.
#[derive(Debug, Default)]
pub struct MotionQueuing {
    trapq: Trapq,
    steppers: Vec<Stepper>,
    mcu_freq: f64,
}

impl MotionQueuing {
    /// An empty queue converting with `mcu_freq` ticks per second.
    pub fn new(mcu_freq: f64) -> Self {
        Self {
            trapq: Trapq::new(),
            steppers: Vec::new(),
            mcu_freq,
        }
    }

    /// The trapezoid queue.
    pub fn trapq(&self) -> &Trapq {
        &self.trapq
    }

    /// The trapezoid queue, to move the current position in it
    /// ([`Trapq::set_position`]).
    pub fn trapq_mut(&mut self) -> &mut Trapq {
        &mut self.trapq
    }

    /// Drop finished segments from the live queue into the history
    /// (`trapq_finalize_moves`).
    ///
    /// `print_time` is how far the step solvers have generated: anything ending
    /// before it can never be read again. `clear_history_time` is how old a
    /// history entry may be before it is dropped.
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        self.trapq.finalize_moves(print_time, clear_history_time);
    }

    /// Add a stepper to generate for.
    pub fn add_stepper(&mut self, stepper: Stepper) {
        self.steppers.push(stepper);
    }

    /// The steppers.
    pub fn steppers(&self) -> &[Stepper] {
        &self.steppers
    }

    /// The steppers, to set positions on.
    pub fn steppers_mut(&mut self) -> &mut [Stepper] {
        &mut self.steppers
    }

    /// Append one planned move's trapezoid
    /// (`trapq_append` via `ToolHead._process_lookahead`).
    pub fn append_move(&mut self, print_time: f64, move_: &Move) {
        self.trapq.append(
            print_time,
            move_.accel_t,
            move_.cruise_t,
            move_.decel_t,
            Xyz::new(
                move_.start_pos.x(),
                move_.start_pos.y(),
                move_.start_pos.z(),
            ),
            Xyz::new(move_.axes_r[0], move_.axes_r[1], move_.axes_r[2]),
            move_.start_v,
            move_.cruise_v,
            move_.accel,
        );
    }

    /// Generate steps for every stepper up to `flush_time`.
    ///
    /// Returns one entry per stepper that produced commands, in the order the
    /// steppers were added.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from a stepper's compressor.
    pub fn generate(
        &mut self,
        flush_time: f64,
    ) -> Result<Vec<(String, Vec<StepCommand>)>, StepCompressError> {
        let Self {
            trapq,
            steppers,
            mcu_freq,
        } = self;
        let move_clock = (flush_time.max(0.0) * *mcu_freq) as u64;
        let mut out = Vec::new();
        for stepper in steppers.iter_mut() {
            let commands = stepper.generate(trapq, flush_time, move_clock)?;
            if !commands.is_empty() {
                out.push((stepper.name().to_string(), commands));
            }
        }
        Ok(out)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::Coord;
    use crate::core::klippy::motion::itersolve::Axis;
    use crate::core::klippy::motion::plan::MoveLimits;

    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    /// A 10 mm move along X at 100 mm/s, already planned.
    fn move_(start: f64, end: f64) -> Move {
        Move::new(
            Coord::new(start, 0.0, 0.0, 0.0),
            Coord::new(end, 0.0, 0.0, 0.0),
            100.0,
            &limits(),
        )
    }

    #[test]
    fn test_append_and_generate_use_the_same_trapq() {
        let mut queuing = MotionQueuing::new(1_000_000.0);
        queuing.add_stepper(Stepper::cartesian(
            "stepper_x",
            0,
            1.0,
            Axis::X,
            1_000_000.0,
        ));
        let mut move_ = move_(0.0, 10.0);
        // Give it a profile: accelerate and decelerate over 10 mm.
        move_.set_junction(0.0, 10_000.0, 0.0);
        queuing.append_move(0.0, &move_);

        let batches = queuing.generate(0.2).unwrap();

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, "stepper_x");
        let steps: u32 = batches[0]
            .1
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum();
        assert_eq!(steps, 10);
    }

    #[test]
    fn test_a_stepper_with_nothing_to_do_is_silent() {
        let mut queuing = MotionQueuing::new(1_000_000.0);
        queuing.add_stepper(Stepper::cartesian(
            "stepper_y",
            1,
            1.0,
            Axis::Y,
            1_000_000.0,
        ));
        let mut move_ = move_(0.0, 10.0); // X only
        move_.set_junction(0.0, 10_000.0, 0.0);
        queuing.append_move(0.0, &move_);

        // The Y stepper produces no commands, so only the trapq had work.
        assert!(queuing.generate(0.2).unwrap().is_empty());
    }
}
