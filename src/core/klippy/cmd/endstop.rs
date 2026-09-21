//! `endstop` commands — a pin the firmware watches for a stop.
//!
//! Host view of `src/endstop.c`. An endstop is a GPIO input the firmware samples
//! during a homing move; when it sees the expected level it fires its `trsync`
//! (which stops the steppers at once). Outside a move the host can ask for the
//! pin's current level.
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_endstop oid=%c pin=%c pull_up=%c` |
//! | host → MCU | `endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c` |
//! | host → MCU | `endstop_query_state oid=%c` |
//! | MCU → host | `endstop_state oid=%c homing=%c next_clock=%u pin_value=%c` |
//!
//! An all-zero `endstop_home` (no clock, no sample count) disables the check, as
//! upstream does when a homing move ends (`klippy/mcu.py:383`).

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_endstop oid=%c pin=%c pull_up=%c` — allocate the endstop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigEndstop {
    /// The oid the config callback assigned.
    pub oid: u8,
    /// Numeric pin (the firmware's `pin` enumeration value).
    pub pin: u8,
    /// `1` for a pull-up, `-1` for a pull-down, `0` for none, as `pins` parsed
    /// the `^`/`~` decorations.
    pub pull_up: i8,
}

impl McuCommand for ConfigEndstop {
    const NAME: &'static str = "config_endstop";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.pin),
            ArgValue::UInt8(self.pull_up as u8),
        ]
    }
}

/// `endstop_home ...` — arm (or, all-zero, disable) the endstop check.
///
/// The firmware waits until `clock`, then samples every `rest_ticks` until the
/// pin matches `pin_value`; when it does it confirms over `sample_count` samples
/// `sample_ticks` apart and fires `trsync_oid` with `trigger_reason`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndstopHome {
    /// The endstop's oid.
    pub oid: u8,
    /// The clock the check starts at.
    pub clock: u32,
    /// Ticks between the confirmation samples.
    pub sample_ticks: u32,
    /// How many samples confirm a trigger; `0` disables the check.
    pub sample_count: u8,
    /// Ticks between poll attempts.
    pub rest_ticks: u32,
    /// The pin level that counts as triggered, already XORed with the pin's
    /// inversion.
    pub pin_value: u8,
    /// The trigger group to fire.
    pub trsync_oid: u8,
    /// The reason to fire with.
    pub trigger_reason: u8,
}

impl EndstopHome {
    /// The all-zero message that disables checking
    /// (`home_wait`'s cleanup, `klippy/mcu.py:383`).
    pub fn disable(oid: u8) -> Self {
        Self {
            oid,
            clock: 0,
            sample_ticks: 0,
            sample_count: 0,
            rest_ticks: 0,
            pin_value: 0,
            trsync_oid: 0,
            trigger_reason: 0,
        }
    }
}

impl McuCommand for EndstopHome {
    const NAME: &'static str = "endstop_home";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.sample_ticks),
            ArgValue::UInt8(self.sample_count),
            ArgValue::UInt32(self.rest_ticks),
            ArgValue::UInt8(self.pin_value),
            ArgValue::UInt8(self.trsync_oid),
            ArgValue::UInt8(self.trigger_reason),
        ]
    }
}

/// `endstop_query_state oid=%c` — ask for the current state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndstopQueryState {
    /// The endstop's oid.
    pub oid: u8,
}

impl McuCommand for EndstopQueryState {
    const NAME: &'static str = "endstop_query_state";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `endstop_state oid=%c homing=%c next_clock=%u pin_value=%c` — the current
/// state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndstopState {
    /// The endstop's oid.
    pub oid: u8,
    /// Whether a homing check is armed.
    pub homing: bool,
    /// The clock of the next poll while homing.
    pub next_clock: u32,
    /// The raw pin level (before the host applies the `!` inversion).
    pub pin_value: u8,
}

impl McuResponse for EndstopState {
    const NAME: &'static str = "endstop_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            homing: params.get_u8("homing")? != 0,
            next_clock: params.get_u32("next_clock")?,
            pin_value: params.get_u8("pin_value")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endstop_home_args_follow_the_firmware_order() {
        let cmd = EndstopHome {
            oid: 3,
            clock: 1000,
            sample_ticks: 1500,
            sample_count: 4,
            rest_ticks: 75000,
            pin_value: 1,
            trsync_oid: 2,
            trigger_reason: 1,
        };
        assert_eq!(
            cmd.args(),
            vec![
                ArgValue::UInt8(3),
                ArgValue::UInt32(1000),
                ArgValue::UInt32(1500),
                ArgValue::UInt8(4),
                ArgValue::UInt32(75000),
                ArgValue::UInt8(1),
                ArgValue::UInt8(2),
                ArgValue::UInt8(1),
            ]
        );
    }

    #[test]
    fn config_endstop_encodes_a_negative_pullup_as_a_byte() {
        let cmd = ConfigEndstop {
            oid: 1,
            pin: 16,
            pull_up: -1,
        };
        assert_eq!(
            cmd.args(),
            vec![
                ArgValue::UInt8(1),
                ArgValue::UInt8(16),
                ArgValue::UInt8(0xff),
            ]
        );
    }

    #[test]
    fn disable_is_all_zero() {
        assert_eq!(
            EndstopHome::disable(5).args(),
            vec![
                ArgValue::UInt8(5),
                ArgValue::UInt32(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
            ]
        );
    }
}
