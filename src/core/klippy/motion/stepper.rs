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
use super::stepcompress::{StepCommand, StepCompressError, StepCompressor};
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
    /// `move_clock` is the firmware clock `flush_time` corresponds to; it
    /// releases the step the compressor holds back
    /// (`stepcompress_flush`).
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from the compressor.
    pub fn generate(
        &mut self,
        trapq: &Trapq,
        flush_time: f64,
        move_clock: u64,
    ) -> Result<Vec<StepCommand>, StepCompressError> {
        self.kinematics
            .generate_steps(trapq, &mut self.compressor, flush_time)?;
        self.compressor.flush(move_clock)?;
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

        let commands = stepper.generate(&trapq, 0.1, 100_000).unwrap();

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
