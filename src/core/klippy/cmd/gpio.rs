//! GPIO commands — digital outputs, and the software-PWM cycle on one.
//!
//! Host view of the "GPIO out pins" and "Hardware PWM" sections of the
//! firmware's `basecmd.c` / `gpiocmds.c`. A digital output is created once with
//! `config_digital_out` and then driven three ways:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u` |
//! | host → MCU | `update_digital_out oid=%c value=%c` |
//! | host → MCU | `queue_digital_out oid=%c clock=%u on_ticks=%u` |
//! | host → MCU | `set_digital_out_pwm_cycle oid=%c cycle_ticks=%u` |
//!
//! `config_digital_out` allocates the oid; `pin` is the firmware's numeric pin
//! (the `pin` enumeration), `value` the level to drive now, `default_value` the
//! level to fall back to on shutdown, and `max_duration` the longest a
//! scheduled change may sit before the firmware forces the default — in clock
//! ticks, `0` meaning no limit.
//!
//! `update_digital_out` changes the level immediately, and is also the command
//! replayed on every connect (`add_restart_cmd`) to restore the start level.
//! `queue_digital_out` changes it at an absolute firmware clock: `on_ticks` is
//! the level for a plain output (`gpiocmds.c:174`), and the on-duration within
//! the cycle once `set_digital_out_pwm_cycle` has made the pin a software PWM
//! (`gpiocmds.c:141`).
//!
//! The pin name in a config file is resolved to the number here by the resource
//! that builds the command (`mcu/resource/pin.rs`), because this host's encoder takes
//! [`ArgValue`]s rather than command text.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigDigitalOut {
    /// The object id the firmware allocated for this output.
    pub oid: u8,
    /// Numeric pin (the `pin` enumeration value).
    pub pin: u32,
    /// Level to drive now.
    pub value: u8,
    /// Level the firmware returns to on shutdown.
    pub default_value: u8,
    /// Longest a scheduled change may be outstanding, in clock ticks (`0` = no
    /// limit).
    pub max_duration: u32,
}

impl McuCommand for ConfigDigitalOut {
    const NAME: &'static str = "config_digital_out";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.pin),
            ArgValue::UInt8(self.value),
            ArgValue::UInt8(self.default_value),
            ArgValue::UInt32(self.max_duration),
        ]
    }
}

/// `update_digital_out oid=%c value=%c` — change the level now.
///
/// The firmware refuses it while scheduled changes are queued
/// (`gpiocmds.c:195`), so it is for pins that are not being driven on a clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateDigitalOut {
    /// The output's oid.
    pub oid: u8,
    /// Level to drive.
    pub value: u8,
}

impl McuCommand for UpdateDigitalOut {
    const NAME: &'static str = "update_digital_out";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.value)]
    }
}

/// `queue_digital_out oid=%c clock=%u on_ticks=%u` — change at an absolute
/// firmware clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueueDigitalOut {
    /// The output's oid.
    pub oid: u8,
    /// Firmware clock at which the change happens.
    pub clock: u32,
    /// The level for a plain output; the on-duration within the cycle for a
    /// software PWM.
    pub on_ticks: u32,
}

impl McuCommand for QueueDigitalOut {
    const NAME: &'static str = "queue_digital_out";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.on_ticks),
        ]
    }
}

/// `set_digital_out_pwm_cycle oid=%c cycle_ticks=%u` — turn a digital output
/// into a software PWM of `cycle_ticks`.
///
/// The firmware refuses it while scheduled changes are queued
/// (`gpiocmds.c:141`), so it belongs in the config or init phase, before the pin
/// is driven.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetDigitalOutPwmCycle {
    /// The output's oid.
    pub oid: u8,
    /// The period of the software PWM, in clock ticks.
    pub cycle_ticks: u32,
}

impl McuCommand for SetDigitalOutPwmCycle {
    const NAME: &'static str = "set_digital_out_pwm_cycle";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.cycle_ticks),
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

    /// The messages this module needs, as the firmware would publish them.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 10,
                "update_digital_out oid=%c value=%c": 11,
                "queue_digital_out oid=%c clock=%u on_ticks=%u": 12,
                "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u": 13
            }
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn test_config_digital_out_matches_the_firmware_format() {
        let command = ConfigDigitalOut {
            oid: 3,
            pin: 42,
            value: 1,
            default_value: 0,
            max_duration: 40_000_000,
        };

        let encoded = parser()
            .encode(ConfigDigitalOut::NAME, &command.args())
            .unwrap();
        let decoded = parser().decode(encoded).unwrap();

        assert_eq!(decoded[0].0.name, "config_digital_out");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_update_and_queue_and_cycle_match_the_firmware_format() {
        let parser = parser();

        for (name, args) in [
            (
                UpdateDigitalOut::NAME,
                UpdateDigitalOut { oid: 1, value: 0 }.args(),
            ),
            (
                QueueDigitalOut::NAME,
                QueueDigitalOut {
                    oid: 1,
                    clock: 1234,
                    on_ticks: 1,
                }
                .args(),
            ),
            (
                SetDigitalOutPwmCycle::NAME,
                SetDigitalOutPwmCycle {
                    oid: 1,
                    cycle_ticks: 2_000_000,
                }
                .args(),
            ),
        ] {
            let encoded = parser.encode(name, &args).unwrap();
            let decoded = parser.decode(encoded).unwrap();
            assert_eq!(decoded[0].0.name, name);
            assert_eq!(decoded[0].1, args);
        }
    }
}
