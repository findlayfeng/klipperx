//! The toolhead: where moves are planned and print time is tracked.
//!
//! Upstream's `klippy/toolhead.py` `ToolHead`: it holds the look-ahead queue,
//! the commanded position and the print time, and feeds the motion queue. The
//! kinematics — which positions are legal, how to home — is FW5e; this is the
//! skeleton a `G1` drives.

use super::plan::{LookAheadQueue, Move, MoveLimits};
use super::queuing::MotionQueuing;
use super::stepcompress::StepCommand;
use super::stepper::Stepper;
use super::trapq::Trapq;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::Coord;

/// How far ahead of the MCU the planner starts, in seconds
/// (`BUFFER_TIME_START`, `klippy/toolhead.py:196`).
pub const BUFFER_TIME_START: f64 = 0.250;

/// The toolhead: commanded position, print time, and the motion queue.
#[derive(Debug)]
pub struct ToolHead {
    limits: MoveLimits,
    lookahead: LookAheadQueue,
    commanded_pos: Coord,
    print_time: f64,
    estimated_print_time: f64,
    special_queuing_state: bool,
    motion_queuing: MotionQueuing,
}

impl ToolHead {
    /// A toolhead with `limits`, whose MCU ticks at `mcu_freq`.
    pub fn new(limits: MoveLimits, mcu_freq: f64) -> Self {
        Self {
            limits,
            lookahead: LookAheadQueue::new(),
            commanded_pos: Coord::default(),
            print_time: 0.0,
            estimated_print_time: 0.0,
            // Upstream starts in "NeedPrime" and resyncs the print time on the
            // first planned move (`klippy/toolhead.py:224`).
            special_queuing_state: true,
            motion_queuing: MotionQueuing::new(mcu_freq),
        }
    }

    /// Add a stepper to drive.
    pub fn add_stepper(&mut self, stepper: Stepper) {
        self.motion_queuing.add_stepper(stepper);
    }

    /// Where the toolhead has been commanded to.
    pub fn commanded_pos(&self) -> Coord {
        self.commanded_pos
    }

    /// The print time the planner has reached.
    pub fn print_time(&self) -> f64 {
        self.print_time
    }

    /// The trapezoid queue.
    pub fn trapq(&self) -> &Trapq {
        self.motion_queuing.trapq()
    }

    /// The motion queue, for setting stepper positions.
    pub fn motion_queuing_mut(&mut self) -> &mut MotionQueuing {
        &mut self.motion_queuing
    }

    /// The MCU's estimated print time, used when planning starts
    /// (`MCU.estimated_print_time`).
    pub fn set_estimated_print_time(&mut self, print_time: f64) {
        self.estimated_print_time = print_time;
    }

    /// Queue a move (`ToolHead.move`, `klippy/toolhead.py:395-408`).
    ///
    /// # Errors
    /// A zero-length move is ignored; the kinematics' own `check_move`
    /// (bounds, per-axis speed limits) is FW5e.
    pub fn move_to(&mut self, newpos: Coord, speed: f64) -> Result<(), CommandError> {
        let move_ = Move::new(self.commanded_pos, newpos, speed, &self.limits);
        if move_.move_d == 0.0 {
            return Ok(());
        }
        self.commanded_pos = move_.end_pos;
        let want_flush = self.lookahead.add_move(move_);
        if want_flush {
            self.process_lookahead();
        }
        Ok(())
    }

    /// Flush the look-ahead into the trapq and advance the print time
    /// (`ToolHead._process_lookahead`, `klippy/toolhead.py:269-298`).
    fn process_lookahead(&mut self) {
        let moves = self.lookahead.flush(false);
        if moves.is_empty() {
            return;
        }
        if self.special_queuing_state {
            // Leaving "NeedPrime": start the print time a buffer ahead of the
            // MCU so the queue is never empty when motion starts.
            self.special_queuing_state = false;
            let min_print_time = self.estimated_print_time + BUFFER_TIME_START;
            if min_print_time > self.print_time {
                self.print_time = min_print_time;
            }
        }
        let mut next_move_time = self.print_time;
        for mut move_ in moves {
            if move_.is_kinematic_move {
                self.motion_queuing.append_move(next_move_time, &move_);
            }
            next_move_time += move_.accel_t + move_.cruise_t + move_.decel_t;
            for callback in move_.timing_callbacks.drain(..) {
                callback(next_move_time);
            }
        }
        self.print_time = next_move_time;
    }

    /// Flush the look-ahead and generate steps up to `step_gen_time`.
    ///
    /// Returns one entry per stepper that produced commands.
    pub fn flush_step_generation(&mut self, step_gen_time: f64) -> Vec<(String, Vec<StepCommand>)> {
        self.process_lookahead();
        self.motion_queuing.generate(step_gen_time)
    }

    /// Plan everything queued so far (`ToolHead.wait_moves`,
    /// `klippy/toolhead.py:422-428`): upstream then waits for the MCU to catch
    /// up, which needs the clock estimate and is FW5d's MCU side.
    pub fn wait_moves(&mut self) {
        self.process_lookahead();
    }

    /// Wait `delay` seconds without moving (`ToolHead.dwell`,
    /// `klippy/toolhead.py:417-420`).
    pub fn dwell(&mut self, delay: f64) {
        self.process_lookahead();
        self.print_time += delay.max(0.0);
    }

    /// Append a move's trapezoid directly, for tests and `drip_move`.
    pub fn append_move(&mut self, print_time: f64, move_: &Move) {
        self.motion_queuing.append_move(print_time, move_);
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::motion::itersolve::Axis;
    use crate::core::klippy::motion::stepcompress::StepCommand;
    use crate::core::klippy::motion::stepper::Stepper;

    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    /// A toolhead with one X stepper, 1 mm per step.
    fn toolhead() -> ToolHead {
        let mut toolhead = ToolHead::new(limits(), 1_000_000.0);
        toolhead.add_stepper(Stepper::cartesian(
            "stepper_x",
            0,
            1.0,
            Axis::X,
            1_000_000.0,
        ));
        toolhead
    }

    /// The number of `queue_step` commands across all steppers.
    fn step_count(batches: &[(String, Vec<StepCommand>)]) -> usize {
        batches
            .iter()
            .flat_map(|(_, commands)| commands)
            .filter(|command| matches!(command, StepCommand::QueueStep { .. }))
            .count()
    }

    #[test]
    fn test_a_move_reaches_the_trapq_and_generates_queue_steps() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0);

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, "stepper_x");
        // 10 mm of travel at 1 mm per step.
        assert_eq!(step_count(&batches), 10);
        assert_eq!(toolhead.commanded_pos(), Coord::new(10.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn test_two_collinear_moves_keep_their_junction_speed() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead
            .move_to(Coord::new(20.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0);

        // No stop between the moves: 20 mm at 1 mm per step.
        assert_eq!(step_count(&batches), 20);
    }

    #[test]
    fn test_dwell_advances_print_time() {
        let mut toolhead = toolhead();
        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead.flush_step_generation(1.0);
        let before = toolhead.print_time();

        toolhead.dwell(0.5);

        assert!((toolhead.print_time() - (before + 0.5)).abs() < 1e-9);
    }

    #[test]
    fn test_a_zero_length_move_is_ignored() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();

        assert!(toolhead.trapq().moves().is_empty());
    }
}
