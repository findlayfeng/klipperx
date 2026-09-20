//! Hardware-PWM commands.
//!
//! Host view of the "Hardware PWM" section of the firmware's `pwmcmds.c`. A
//! hardware PWM output is created once with `config_pwm_out` and then driven on
//! a clock:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu max_duration=%u` |
//! | host → MCU | `queue_pwm_out oid=%c clock=%u value=%hu` |
//!
//! `cycle_ticks` is the period of one PWM cycle; `value`/`default_value` are a
//! duty in `0..PWM_MAX` (`PWM_MAX` is a firmware constant, read by the resource
//! that builds the command). `max_duration` is the longest a queued value may
//! sit before the firmware falls back to `default_value`, in clock ticks (`0`
//! meaning no limit).
//!
//! Software PWM reuses the GPIO commands instead: `config_digital_out` +
//! `set_digital_out_pwm_cycle` + `queue_digital_out` ([`super::gpio`]). Which one
//! a `[pwm]` pin uses is decided by [`McuPwm`](crate::core::klippy::mcu::McuPwm)
//! at build time.
//!
//! The pin name in a config file is resolved to the number here by the resource
//! that builds the command (`mcu/resource/pwm.rs`), for the same reason as the digital
//! output: this host's encoder takes [`ArgValue`]s, not command text.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu max_duration=%u`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigPwmOut {
    /// The object id the firmware allocated for this output.
    pub oid: u8,
    /// Numeric pin (the `pin` enumeration value).
    pub pin: u32,
    /// One PWM period, in clock ticks.
    pub cycle_ticks: u32,
    /// Duty to drive now, in `0..PWM_MAX`.
    pub value: u16,
    /// Duty the firmware falls back to on shutdown, in `0..PWM_MAX`.
    pub default_value: u16,
    /// Longest a queued value may be outstanding, in clock ticks (`0` = no
    /// limit).
    pub max_duration: u32,
}

impl McuCommand for ConfigPwmOut {
    const NAME: &'static str = "config_pwm_out";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.pin),
            ArgValue::UInt32(self.cycle_ticks),
            ArgValue::UInt16(self.value),
            ArgValue::UInt16(self.default_value),
            ArgValue::UInt32(self.max_duration),
        ]
    }
}

/// `queue_pwm_out oid=%c clock=%u value=%hu` — change the duty at an absolute
/// firmware clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueuePwmOut {
    /// The output's oid.
    pub oid: u8,
    /// Firmware clock at which the change happens.
    pub clock: u32,
    /// Duty, in `0..PWM_MAX`.
    pub value: u16,
}

impl McuCommand for QueuePwmOut {
    const NAME: &'static str = "queue_pwm_out";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt16(self.value),
        ]
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use serde_json::json;

    fn parser() -> Parser {
        let dictionary = Dictionary::from_json(json!({
            "commands": {
                "config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu max_duration=%u": 20,
                "queue_pwm_out oid=%c clock=%u value=%hu": 21
            }
        }))
        .unwrap();
        let mut parser = Parser::new();
        dictionary.install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn test_config_pwm_out_matches_the_firmware_format() {
        let command = ConfigPwmOut {
            oid: 3,
            pin: 42,
            cycle_ticks: 2_000_000,
            value: 1234,
            default_value: 0,
            max_duration: 40_000_000,
        };

        let encoded = parser()
            .encode(ConfigPwmOut::NAME, &command.args())
            .unwrap();
        let decoded = parser().decode(encoded).unwrap();

        assert_eq!(decoded[0].0.name, "config_pwm_out");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_queue_pwm_out_matches_the_firmware_format() {
        let command = QueuePwmOut {
            oid: 1,
            clock: 123_456,
            value: 4095,
        };

        let encoded = parser().encode(QueuePwmOut::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();

        assert_eq!(decoded[0].0.name, "queue_pwm_out");
        assert_eq!(decoded[0].1, command.args());
    }
}
