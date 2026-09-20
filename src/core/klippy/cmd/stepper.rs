//! Stepper commands — step generation, the MCU's core work.
//!
//! Host view of `src/stepper.c`. A stepper is created once with
//! `config_stepper`, then moved by queueing `queue_step` entries: one entry is
//! `count` steps `interval` ticks apart, with `add` added to the interval after
//! every step (a constant rate when `add` is 0).
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u` |
//! | host → MCU | `queue_step oid=%c interval=%u count=%hu add=%hi` |
//! | host → MCU | `set_next_step_dir oid=%c dir=%c` |
//! | host → MCU | `reset_step_clock oid=%c clock=%u` |
//! | host → MCU | `stepper_get_position oid=%c` |
//! | MCU → host | `stepper_position oid=%c pos=%i` |
//!
//! `invert_step` is signed: `0` normal, `1` inverted, `-1` "single schedule"
//! (`src/stepper.c:224-231`). The firmware encodes it as one byte, so a `-1`
//! goes out as `0xff`.
//!
//! This is also where an MCU's real limit shows: queueing more moves than its
//! move queue holds shuts it down with `Move queue overflow`
//! (`src/basecmd.c:90`), and scheduling steps faster than the timer task can
//! service them shuts it down with `Timer too close` (`src/sched.c:94`). The
//! `stress` subcommand ramps exactly this load.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigStepper {
    /// The object id the firmware allocated for this stepper.
    pub oid: u8,
    /// Numeric step pin (the firmware's `pin` enumeration value).
    pub step_pin: u8,
    /// Numeric direction pin.
    pub dir_pin: u8,
    /// `0` normal, `1` inverted, `-1` single-schedule.
    pub invert_step: i8,
    /// Step pulse width in clock ticks; `0` lets the firmware choose.
    pub step_pulse_ticks: u32,
}

impl McuCommand for ConfigStepper {
    const NAME: &'static str = "config_stepper";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.step_pin),
            ArgValue::UInt8(self.dir_pin),
            ArgValue::UInt8(self.invert_step as u8),
            ArgValue::UInt32(self.step_pulse_ticks),
        ]
    }
}

/// `queue_step oid=%c interval=%u count=%hu add=%hi` — queue `count` steps.
///
/// The first step happens `interval` ticks after the previous move ended (or
/// after `reset_step_clock`), and each following step is `interval` ticks after
/// the one before, with `add` added to the interval each time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueueStep {
    /// The stepper's oid.
    pub oid: u8,
    /// Ticks between steps.
    pub interval: u32,
    /// Number of steps (`0` is refused by the firmware: `Invalid count parameter`).
    pub count: u16,
    /// Ticks added to `interval` after each step (negative to accelerate).
    pub add: i16,
}

impl McuCommand for QueueStep {
    const NAME: &'static str = "queue_step";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.interval),
            ArgValue::UInt16(self.count),
            ArgValue::Int16(self.add),
        ]
    }
}

/// `set_next_step_dir oid=%c dir=%c` — the direction of the next queued move.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetNextStepDir {
    /// The stepper's oid.
    pub oid: u8,
    /// The direction for the next `queue_step`.
    pub dir: u8,
}

impl McuCommand for SetNextStepDir {
    const NAME: &'static str = "set_next_step_dir";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.dir)]
    }
}

/// `reset_step_clock oid=%c clock=%u` — set the stepper's clock to `clock`.
///
/// The stepper must be idle; the next `queue_step`'s first step is scheduled
/// relative to this.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResetStepClock {
    /// The stepper's oid.
    pub oid: u8,
    /// The firmware clock the stepper's time base is set to.
    pub clock: u32,
}

impl McuCommand for ResetStepClock {
    const NAME: &'static str = "reset_step_clock";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.clock)]
    }
}

/// `stepper_get_position oid=%c` — ask how many steps the stepper has taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepperGetPosition {
    /// The stepper's oid.
    pub oid: u8,
}

impl McuCommand for StepperGetPosition {
    const NAME: &'static str = "stepper_get_position";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `stepper_position oid=%c pos=%i` — the stepped position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepperPosition {
    /// The stepper's oid.
    pub oid: u8,
    /// Steps taken since the stepper was configured, direction-signed.
    pub pos: i32,
}

impl McuResponse for StepperPosition {
    const NAME: &'static str = "stepper_position";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            pos: params.get_i32("pos")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_stepper_args_follow_the_firmware_order() {
        let cmd = ConfigStepper {
            oid: 3,
            step_pin: 12,
            dir_pin: 13,
            invert_step: -1,
            step_pulse_ticks: 40,
        };
        assert_eq!(
            cmd.args(),
            vec![
                ArgValue::UInt8(3),
                ArgValue::UInt8(12),
                ArgValue::UInt8(13),
                ArgValue::UInt8(0xff),
                ArgValue::UInt32(40),
            ]
        );
    }

    #[test]
    fn queue_step_args_follow_the_firmware_order() {
        let cmd = QueueStep {
            oid: 1,
            interval: 2500,
            count: 60000,
            add: -3,
        };
        assert_eq!(
            cmd.args(),
            vec![
                ArgValue::UInt8(1),
                ArgValue::UInt32(2500),
                ArgValue::UInt16(60000),
                ArgValue::Int16(-3),
            ]
        );
    }
}
