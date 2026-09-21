//! The MCU side of a stepper.
//!
//! Upstream's `MCU_stepper` (`klippy/stepper.py:22-260`): it owns the oid and
//! the step/dir pins, adds `config_stepper` while the config is built, and
//! sends `queue_step`/`set_next_step_dir` at runtime. The host-side solver and
//! compressor live in `motion::Stepper`; this is the wire half, and the two are
//! joined when `[stepper_*]` sections land (FW5e).

use std::sync::{Arc, Mutex};

use super::pin::pin_number;
use crate::core::klippy::cmd::stepper::{ConfigStepper, QueueStep, SetNextStepDir};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::motion::stepcompress::StepCommand;
use crate::core::klippy::pins::{PinParams, PrinterPins};

/// The oid the config callback assigned.
#[derive(Debug, Default)]
struct StepperState {
    oid: Mutex<Option<u8>>,
}

/// One stepper's firmware side: its oid, and the commands it sends.
#[derive(Debug)]
pub struct McuStepper {
    state: Arc<StepperState>,
    /// Shared with the chip, so runtime sends reach the connected device.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
}

impl McuStepper {
    /// Build the resource and register its config callback.
    ///
    /// `invert_step` is `0` normal, `1` inverted, `-1` single-schedule;
    /// `step_pulse_ticks` is the pulse width in clock ticks, `0` letting the
    /// firmware choose (`config_stepper`).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        config: Arc<ConfigBuilder>,
        pins: Arc<PrinterPins>,
        chip_name: String,
        mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
        step_pin: PinParams,
        dir_pin: PinParams,
        invert_step: i8,
        step_pulse_ticks: u32,
    ) -> Self {
        let state = Arc::new(StepperState::default());
        let callback_state = Arc::clone(&state);
        // Weak, not Arc: the registry owns the chip, and the chip owns this
        // callback's `ConfigBuilder` (`mcu/resource/pin.rs` explains the cycle).
        let callback_pins = Arc::downgrade(&pins);
        config
            .register_config_callback(Box::new(move |builder, mcu| {
                let pins = callback_pins
                    .upgrade()
                    .expect("the pins registry outlives the resources it built");
                callback_state.build(
                    builder,
                    mcu,
                    &pins,
                    &chip_name,
                    &step_pin,
                    &dir_pin,
                    invert_step,
                    step_pulse_ticks,
                )
            }))
            .expect("a resource is always built before the configuration is");
        Self { state, mcu }
    }

    /// The oid the config callback assigned.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] before the configuration has been built.
    fn oid(&self) -> Result<u8, McuError> {
        self.state
            .oid
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .ok_or_else(|| McuError::Config("stepper is not configured yet".to_string()))
    }

    /// Send a runtime command to the connected device.
    fn send<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        let mcu = self
            .mcu
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        mcu.send_msg(cmd)
    }

    /// Queue `count` steps (`queue_step`).
    ///
    /// # Errors
    /// As [`McuStepper::send`].
    pub fn queue_step(&self, interval: u32, count: u16, add: i16) -> Result<(), McuError> {
        self.send(&QueueStep {
            oid: self.oid()?,
            interval,
            count,
            add,
        })
    }

    /// Set the direction of the next queued move (`set_next_step_dir`).
    ///
    /// # Errors
    /// As [`McuStepper::send`].
    pub fn set_next_step_dir(&self, direction: bool) -> Result<(), McuError> {
        self.send(&SetNextStepDir {
            oid: self.oid()?,
            dir: u8::from(direction),
        })
    }

    /// Send the commands a [`motion::Stepper`](crate::core::klippy::motion::Stepper)
    /// produced.
    ///
    /// # Errors
    /// As [`McuStepper::send`].
    pub fn send_steps(&self, commands: &[StepCommand]) -> Result<(), McuError> {
        let oid = self.oid()?;
        for command in commands {
            match step_command_to_mcu(oid, command) {
                McuStepCommand::Dir(cmd) => self.send(&cmd)?,
                McuStepCommand::Step(cmd) => self.send(&cmd)?,
            }
        }
        Ok(())
    }
}

impl StepperState {
    /// Resolve the pins and add the configuration
    /// (`MCU_stepper._build_config`, `klippy/stepper.py:78-131`).
    #[allow(clippy::too_many_arguments)]
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
        chip_name: &str,
        step_pin: &PinParams,
        dir_pin: &PinParams,
        invert_step: i8,
        step_pulse_ticks: u32,
    ) -> Result<(), McuError> {
        let step_number = pin_number(mcu, &pins.resolve_pin(chip_name, &step_pin.pin)?, chip_name)?;
        let dir_number = pin_number(mcu, &pins.resolve_pin(chip_name, &dir_pin.pin)?, chip_name)?;
        let oid = builder.create_oid()?;
        *self.oid.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(oid);
        builder.add_config_cmd(&ConfigStepper {
            oid,
            step_pin: step_number as u8,
            dir_pin: dir_number as u8,
            invert_step,
            step_pulse_ticks,
        })?;
        Ok(())
    }
}

/// The firmware command a [`StepCommand`] becomes for `oid`.
///
/// The step solver works in `u32`/`i32` (its own arithmetic), while the wire
/// messages are `%hu`/`%hi`; this is where the two meet.
pub fn step_command_to_mcu(oid: u8, command: &StepCommand) -> McuStepCommand {
    match *command {
        StepCommand::SetNextStepDir { direction, .. } => McuStepCommand::Dir(SetNextStepDir {
            oid,
            dir: u8::from(direction),
        }),
        StepCommand::QueueStep {
            interval,
            count,
            add,
            ..
        } => McuStepCommand::Step(QueueStep {
            oid,
            interval,
            count: count as u16,
            add: add as i16,
        }),
    }
}

/// A step command with its oid resolved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McuStepCommand {
    /// A direction change.
    Dir(SetNextStepDir),
    /// A run of steps.
    Step(QueueStep),
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_a_direction_change_becomes_set_next_step_dir() {
        let command = StepCommand::SetNextStepDir {
            oid: 99,
            direction: true,
        };

        assert_eq!(
            step_command_to_mcu(3, &command),
            McuStepCommand::Dir(SetNextStepDir { oid: 3, dir: 1 })
        );
        // The command's own oid is ignored: the resource owns the real one.
    }

    #[test]
    fn test_a_run_of_steps_becomes_queue_step() {
        let command = StepCommand::QueueStep {
            oid: 99,
            interval: 1000,
            count: 1,
            add: 0,
        };

        assert_eq!(
            step_command_to_mcu(3, &command),
            McuStepCommand::Step(QueueStep {
                oid: 3,
                interval: 1000,
                count: 1,
                add: 0,
            })
        );
    }

    #[test]
    fn test_a_negative_add_survives_the_narrowing() {
        // The full compressor emits accelerating runs; the add is signed.
        let command = StepCommand::QueueStep {
            oid: 0,
            interval: 900,
            count: 40,
            add: -3,
        };

        let McuStepCommand::Step(step) = step_command_to_mcu(0, &command) else {
            panic!("a queue_step command becomes a step");
        };
        assert_eq!(step.add, -3);
        assert_eq!(step.count, 40);
    }
}
