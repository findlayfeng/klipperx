//! A stepper as the motion stack sees it.
//!
//! Upstream's `klippy/stepper.py` `MCU_stepper`: it owns the step distance and
//! the `stepper_kinematics`/`stepcompress` pair, and knows how to ask the
//! solver for the steps up to a print time. Here it is host-only — the MCU side
//! (the oid's `config_stepper`, the `queue_step` commands down the wire) is the
//! next step.

use super::itersolve::{
    cartesian_active_flags, cartesian_position_fn, Axis, AxisFlags, PositionFn, StepKinematics,
};
use super::stepcompress::{HistoryStep, StepCommand, StepCompressError, StepCompressor};
use super::trapq::Trapq;
use crate::core::klippy::mathutil::Xyz;

/// One stepper: its solver state and its step compressor.
#[derive(Debug)]
pub struct Stepper {
    name: String,
    oid: u32,
    step_dist: f64,
    kinematics: StepKinematics,
    compressor: StepCompressor,
}

impl Stepper {
    /// A stepper with `step_dist` millimetres per step, moving `active_flags`.
    pub fn new(
        name: impl Into<String>,
        oid: u32,
        step_dist: f64,
        position: PositionFn,
        active_flags: AxisFlags,
        mcu_freq: f64,
    ) -> Self {
        Self {
            name: name.into(),
            oid,
            step_dist,
            kinematics: StepKinematics::new(step_dist, position, active_flags),
            compressor: StepCompressor::new(oid, mcu_freq),
        }
    }

    /// A cartesian axis stepper (`stepper_x`, `stepper_y`, `stepper_z`).
    pub fn cartesian(
        name: impl Into<String>,
        oid: u32,
        step_dist: f64,
        axis: Axis,
        mcu_freq: f64,
    ) -> Self {
        Self::new(
            name,
            oid,
            step_dist,
            cartesian_position_fn(axis),
            cartesian_active_flags(axis),
            mcu_freq,
        )
    }

    /// The stepper's name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The oid the firmware assigned it.
    pub fn oid(&self) -> u32 {
        self.oid
    }

    /// Millimetres per step.
    pub fn step_dist(&self) -> f64 {
        self.step_dist
    }

    /// The solver state.
    pub fn kinematics(&self) -> &StepKinematics {
        &self.kinematics
    }

    /// The solver state, to set a position on.
    pub fn kinematics_mut(&mut self) -> &mut StepKinematics {
        &mut self.kinematics
    }

    /// The stepper position the solver has reached.
    pub fn commanded_position(&self) -> f64 {
        self.kinematics.commanded_pos()
    }

    /// The firmware step counter the solver's position corresponds to
    /// (`MCU_stepper.get_mcu_position`).
    pub fn mcu_position(&self) -> i64 {
        (self.kinematics.commanded_pos() / self.step_dist).round() as i64
    }

    /// The step position at a past print time
    /// (`MCU_stepper.get_past_mcu_position`).
    pub fn past_mcu_position(&self, print_time: f64) -> i64 {
        self.compressor
            .find_past_position(self.compressor.print_time_to_clock(print_time))
    }

    /// The recently sent runs, newest first, for a motion-report consumer.
    pub fn history(&self, max: usize, start_clock: u64, end_clock: u64) -> Vec<HistoryStep> {
        self.compressor.extract_old(max, start_clock, end_clock)
    }

    /// Reset the compressor's clock, as homing does (`stepcompress_reset`).
    ///
    /// # Errors
    /// As [`StepCompressor::reset`].
    pub fn reset_compressor(&mut self, last_step_clock: u64) -> Result<(), StepCompressError> {
        self.compressor.reset(last_step_clock)
    }

    /// Set the stepper position from a toolhead position.
    pub fn set_position(&mut self, pos: Xyz) {
        self.kinematics.set_position(pos);
    }

    /// The step compressor (for tests and for the MCU side).
    pub fn compressor_mut(&mut self) -> &mut StepCompressor {
        &mut self.compressor
    }

    /// Generate the steps up to `flush_time` and return the commands to send.
    ///
    /// The flush bound is computed from this stepper's own compressor: each MCU
    /// has its own clock frequency and print-time offset, so a secondary MCU's
    /// bound differs from the primary's (`stepcompress_flush`).
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from the compressor.
    pub fn generate(
        &mut self,
        trapq: &Trapq,
        flush_time: f64,
    ) -> Result<Vec<StepCommand>, StepCompressError> {
        self.kinematics
            .generate_steps(trapq, &mut self.compressor, flush_time)?;
        self.compressor
            .flush(self.compressor.print_time_to_clock(flush_time))?;
        Ok(self.compressor.take_commands())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::motion::trapq::MoveSegment;

    #[test]
    fn test_a_cartesian_stepper_only_moves_on_its_axis() {
        let stepper = Stepper::cartesian("stepper_x", 0, 1.0, Axis::X, 1_000_000.0);

        assert_eq!(stepper.name(), "stepper_x");
        assert_eq!(stepper.oid(), 0);
        assert_eq!(stepper.step_dist(), 1.0);
        // A move along Y leaves the X stepper alone.
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1.0,
            start_v: 10.0,
            half_accel: 0.0,
            start_pos: Xyz::default(),
            axes_r: Xyz::new(0.0, 1.0, 0.0),
        };
        assert!(!stepper.kinematics().is_active(&segment));
    }

    #[test]
    fn test_each_stepper_generates_in_its_own_mcu_clock() {
        // A move at print time 10.0..10.1, 10 mm at 100 mm/s (1 mm per step).
        let mut trapq = Trapq::new();
        trapq.append(
            10.0,
            0.0,
            0.1,
            0.0,
            Xyz::default(),
            Xyz::new(1.0, 0.0, 0.0),
            100.0,
            100.0,
            0.0,
        );
        // A secondary MCU: 2 MHz, its clock zero at print time 10.
        let mut stepper = Stepper::cartesian("stepper_x", 0, 1.0, Axis::X, 2_000_000.0);
        stepper.compressor_mut().set_time(10.0, 2_000_000.0);

        let commands = stepper.generate(&trapq, 10.1).unwrap();

        let steps: u32 = commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum();
        assert_eq!(steps, 10);
        // 100 mm/s at 1 mm per step is one step per 10 ms, which is 20 000 ticks
        // on this 2 MHz MCU — not the ~20 000 000 a missing offset would give.
        let intervals: Vec<u32> = commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep { interval, .. } => Some(*interval),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .collect();
        assert!(
            intervals.iter().all(|interval| *interval < 100_000),
            "{intervals:?}"
        );
        assert!(
            intervals
                .iter()
                .any(|interval| (19_000..=21_000).contains(interval)),
            "{intervals:?}"
        );
    }

    #[test]
    fn test_mcu_position_and_past_position() {
        let mut trapq = Trapq::new();
        // 10 mm along X at 100 mm/s.
        trapq.append(
            0.0,
            0.0,
            0.1,
            0.0,
            Xyz::default(),
            Xyz::new(1.0, 0.0, 0.0),
            100.0,
            100.0,
            0.0,
        );
        let mut stepper = Stepper::cartesian("stepper_x", 0, 1.0, Axis::X, 1_000_000.0);

        stepper.generate(&trapq, 0.1).unwrap();

        // The solver has commanded the whole 10 mm, so the firmware counter's
        // equivalent is 10.
        assert_eq!(stepper.mcu_position(), 10);
        // Halfway into the move the history says roughly five steps.
        let past = stepper.past_mcu_position(0.05);
        assert!((4..=6).contains(&past), "{past}");
        // The emitted history is available for a motion report.
        assert!(!stepper.history(10, 0, u64::MAX).is_empty());
        stepper.reset_compressor(0).unwrap();
    }

    #[test]
    fn test_generate_returns_the_steppers_commands() {
        let mut trapq = Trapq::new();
        // 10 mm along X at 100 mm/s.
        trapq.append(
            0.0,
            0.0,
            0.1,
            0.0,
            Xyz::default(),
            Xyz::new(1.0, 0.0, 0.0),
            100.0,
            100.0,
            0.0,
        );
        let mut stepper = Stepper::cartesian("stepper_x", 0, 1.0, Axis::X, 1_000_000.0);

        let commands = stepper.generate(&trapq, 0.1).unwrap();

        let steps: u32 = commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum();
        assert_eq!(steps, 10);
    }
}
