//! The motion output queue.
//!
//! Upstream's `klippy/extras/motion_quuing.py`: it owns the trapq and the
//! steppers, and knows how to turn the queued moves into step commands. The
//! real one paces itself with a reactor timer and sends over the MCU; this one
//! is driven explicitly (`generate`) so the host logic can be tested without a
//! device, and the MCU side attaches on top of it.

use super::stepcompress::{StepCommand, StepCompressError};
use super::stepper::Stepper;
use super::trapq::Trapq;

/// The trapqs and the steppers reading them.
///
/// Upstream's `MotionQueuing` owns a list of trapqs (`allocate_trapq`,
/// `motion_queuing.py:63-67`): the toolhead has one, each extruder its own. A
/// stepper is bound to one of them (`MCU_stepper.set_trapq`), and only reads
/// that one when generating steps.
#[derive(Default)]
pub struct MotionQueuing {
    trapqs: Vec<Trapq>,
    steppers: Vec<Stepper>,
    /// The flush callbacks (`register_flush_callback`), invoked with the
    /// flush time at the start of every [`MotionQueuing::generate`]
    /// (`motion_queuing.py:147-150`).
    flush_callbacks: Vec<Box<dyn Fn(f64) + Send>>,
}

impl std::fmt::Debug for MotionQueuing {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotionQueuing")
            .field("trapqs", &self.trapqs)
            .field("steppers", &self.steppers)
            .finish_non_exhaustive()
    }
}

impl MotionQueuing {
    /// An empty queue.
    ///
    /// The queue holds no clock of its own: each [`Stepper`] carries its MCU's
    /// frequency and print-time offset, so a stepper on a secondary MCU flushes
    /// on that MCU's clock.
    pub fn new() -> Self {
        Self {
            trapqs: Vec::new(),
            steppers: Vec::new(),
            flush_callbacks: Vec::new(),
        }
    }

    /// Register a flush callback (`MotionQueuing.register_flush_callback`):
    /// it is called with the flush time at the start of every
    /// [`MotionQueuing::generate`], in registration order, and stays
    /// registered for the later ones. Upstream's `_advance_flush_time` calls
    /// each with `flush_time` and `step_gen_time` (`motion_queuing.py:147-150`);
    /// only the flush time crosses this seam.
    pub fn register_flush_callback(&mut self, callback: Box<dyn Fn(f64) + Send + 'static>) {
        self.flush_callbacks.push(callback);
    }

    /// Create a trapq and return its id (`MotionQueuing.allocate_trapq`).
    pub fn allocate_trapq(&mut self) -> usize {
        self.trapqs.push(Trapq::new());
        self.trapqs.len() - 1
    }

    /// A trapq by id.
    pub fn trapq(&self, id: usize) -> &Trapq {
        &self.trapqs[id]
    }

    /// A trapq by id, to append to or move the current position in it.
    pub fn trapq_mut(&mut self, id: usize) -> &mut Trapq {
        &mut self.trapqs[id]
    }

    /// Drop finished segments from every live queue into its history
    /// (`trapq_finalize_moves`).
    ///
    /// `print_time` is how far the step solvers have generated: anything ending
    /// before it can never be read again. `clear_history_time` is how old a
    /// history entry may be before it is dropped.
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        for trapq in &mut self.trapqs {
            trapq.finalize_moves(print_time, clear_history_time);
        }
    }

    /// Add a stepper to generate for. Its trapq id defaults to 0 (the main
    /// trapq); use [`Stepper::set_trapq`] before adding when it differs.
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

    /// Generate steps for every stepper up to `flush_time`.
    ///
    /// Each stepper reads its own trapq, so the toolhead's steppers and an
    /// extruder's generate from different queues. A detached stepper
    /// (`Stepper::set_trapq(None)`) is skipped: no steps are generated for
    /// it until it is attached again (`MCU_stepper.set_trapq`).
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
            trapqs,
            steppers,
            flush_callbacks,
        } = self;
        // Flush callbacks run first (`_advance_flush_time` invokes them before
        // generating, `motion_queuing.py:149-150`) — and they run even when
        // there is no stepper to generate for: a macro-driven dwell timeline
        // queues no steps, but its flush time still advances.
        for callback in flush_callbacks.iter() {
            callback(flush_time);
        }
        let mut out = Vec::new();
        for stepper in steppers.iter_mut() {
            // Detached (`set_trapq(None)`): generate nothing for it.
            let Some(trapq_id) = stepper.trapq_id() else {
                continue;
            };
            let trapq = &trapqs[trapq_id];
            let commands = stepper.generate(trapq, flush_time)?;
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
    use crate::core::klippy::mathutil::{Coord, Xyz};
    use crate::core::klippy::motion::itersolve::Axis;
    use crate::core::klippy::motion::plan::{Move, MoveLimits};
    use std::sync::{Arc, Mutex};

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

    /// Append a move's kinematic (xyz) trapezoid, as the toolhead does.
    fn append(queuing: &mut MotionQueuing, trapq: usize, move_: &Move) {
        queuing.trapq_mut(trapq).append(
            0.0,
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

    #[test]
    fn test_append_and_generate_use_the_same_trapq() {
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
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
        append(&mut queuing, trapq, &move_);

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
    fn test_a_detached_stepper_generates_no_steps_until_reattached() {
        // `ZAdjustHelper.adjust_steppers` takes every Z motor off the trapq,
        // moves with them detached, and attaches them one by one; a detached
        // stepper must produce no commands until it is attached again.
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        queuing.add_stepper(Stepper::cartesian(
            "stepper_z",
            0,
            1.0,
            Axis::Z,
            1_000_000.0,
        ));
        let mut move_ = move_(0.0, 10.0);
        move_.set_junction(0.0, 10_000.0, 0.0);
        // The same 10 mm, queued along Z (the `append` helper is X-only).
        queuing.trapq_mut(trapq).append(
            0.0,
            move_.accel_t,
            move_.cruise_t,
            move_.decel_t,
            Xyz::new(0.0, 0.0, move_.start_pos.x()),
            Xyz::new(0.0, 0.0, 1.0),
            move_.start_v,
            move_.cruise_v,
            move_.accel,
        );
        // Detach (`set_trapq(None)`): the queued move generates nothing.
        queuing.steppers_mut()[0].set_trapq(None);
        assert_eq!(queuing.steppers()[0].trapq_id(), None);
        assert!(queuing.generate(0.2).unwrap().is_empty());

        // Reattach: the same move now generates its steps.
        queuing.steppers_mut()[0].set_trapq(trapq);
        let batches = queuing.generate(0.2).unwrap();

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, "stepper_z");
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
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        queuing.add_stepper(Stepper::cartesian(
            "stepper_y",
            1,
            1.0,
            Axis::Y,
            1_000_000.0,
        ));
        let mut move_ = move_(0.0, 10.0); // X only
        move_.set_junction(0.0, 10_000.0, 0.0);
        append(&mut queuing, trapq, &move_);

        // The Y stepper produces no commands, so only the trapq had work.
        assert!(queuing.generate(0.2).unwrap().is_empty());
    }

    #[test]
    fn test_flush_callbacks_run_in_registration_order_even_without_steppers() {
        // A dwell-only timeline — no stepper reads the trapq, so `generate`
        // produces nothing — still advances the flush time, and the flush
        // callbacks registered on it still run (`motion_queuing.py:147-150`).
        let mut queuing = MotionQueuing::new();
        queuing.allocate_trapq();
        let calls = Arc::new(Mutex::new(Vec::new()));
        for name in ["first", "second"] {
            let calls = Arc::clone(&calls);
            queuing.register_flush_callback(Box::new(move |flush_time| {
                calls.lock().unwrap().push((name, flush_time));
            }));
        }

        assert!(queuing.generate(1.5).unwrap().is_empty());
        assert_eq!(
            *calls.lock().unwrap(),
            [("first", 1.5), ("second", 1.5)],
            "in registration order, each with the flush time"
        );

        // The callbacks stay registered: the next generate runs them again.
        queuing.generate(2.0).unwrap();
        assert_eq!(
            *calls.lock().unwrap(),
            [
                ("first", 1.5),
                ("second", 1.5),
                ("first", 2.0),
                ("second", 2.0)
            ]
        );
    }

    #[test]
    fn test_two_steppers_on_one_mcu_both_generate() {
        // The common case: one MCU drives several motors. The steppers share the
        // MCU's clock/offset but have independent compressors; a diagonal move
        // must produce commands for both.
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        let mut x = Stepper::cartesian("stepper_x", 0, 1.0, Axis::X, 1_000_000.0);
        x.compressor_mut().set_time(0.0, 1_000_000.0);
        let mut y = Stepper::cartesian("stepper_y", 1, 1.0, Axis::Y, 1_000_000.0);
        y.compressor_mut().set_time(0.0, 1_000_000.0);
        queuing.add_stepper(x);
        queuing.add_stepper(y);
        let mut move_ = Move::new(
            Coord::default(),
            Coord::new(10.0, 10.0, 0.0, 0.0),
            100.0,
            &limits(),
        );
        move_.set_junction(0.0, 10_000.0, 0.0);
        append(&mut queuing, trapq, &move_);

        let batches = queuing.generate(0.2).unwrap();

        let names: Vec<&str> = batches.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(names, ["stepper_x", "stepper_y"]);
        let counts: Vec<u32> = batches
            .iter()
            .map(|(_, commands)| {
                commands
                    .iter()
                    .filter_map(|command| match command {
                        StepCommand::QueueStep { count, .. } => Some(*count),
                        StepCommand::SetNextStepDir { .. } => None,
                    })
                    .sum()
            })
            .collect();
        // The diagonal is symmetric, so both axes take the same number of
        // steps on the shared clock.
        assert_eq!(counts[0], counts[1]);
        assert!(counts[0] >= 9, "{counts:?}");
    }
}
