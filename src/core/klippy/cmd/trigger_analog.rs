//! `trigger_analog` commands — a sensor sample the firmware watches for a stop.
//!
//! Host view of `src/trigger_analog.c`. A `trigger_analog` object filters one
//! sensor's raw samples through an `sos_filter` and fires a `trsync` when a
//! configured trigger matches — the analog-sensor counterpart of an endstop,
//! used for homing on an inductive/eddy probe.
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_trigger_analog oid=%c sos_filter_oid=%c` |
//! | host → MCU | `trigger_analog_set_raw_range oid=%c raw_min=%i raw_max=%i` |
//! | host → MCU | `trigger_analog_set_trigger oid=%c trigger_analog_type=%c trigger_value=%i` |
//! | host → MCU | `trigger_analog_home oid=%c trsync_oid=%c trigger_reason=%c error_reason=%c clock=%u monitor_ticks=%u monitor_max=%u` |
//! | host → MCU | `trigger_analog_query_state oid=%c` |
//! | MCU → host | `trigger_analog_state oid=%c homing=%c homing_clock=%u` |
//!
//! The reason numbers travel over `trsync`: [`REASON_TRIGGER_ANALOG`] is the
//! base, and a failure adds the `trigger_analog_error:` enumeration value —
//! the same decode `MCU_trigger_analog.home_wait` performs upstream
//! (`klippy/extras/trigger_analog.py:374-395`).

use crate::core::klippy::cmd::trsync::TriggerReason;
use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// The first trsync reason a `trigger_analog` failure carries
/// (`MCU_trigger_analog.REASON_TRIGGER_ANALOG`, `trigger_analog.py:273`):
/// `REASON_COMMS_TIMEOUT + 1`, plus the `trigger_analog_error:` value.
pub const REASON_TRIGGER_ANALOG: u8 = TriggerReason::CommsTimeout as u8 + 1;

/// Which comparison the firmware applies to each filtered sample
/// (`trigger_analog_type`, `src/trigger_analog.c:45-47`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TriggerAnalogType {
    /// `abs_ge` — `abs(sample) >= trigger_value`.
    AbsGe = 0,
    /// `gt` — `sample > trigger_value`.
    Gt = 1,
    /// `diff_peak_gt` — the drop from the running peak exceeds `trigger_value`.
    DiffPeakGt = 2,
}

impl TriggerAnalogType {
    /// The name the firmware's enumeration gives this type.
    pub fn name(self) -> &'static str {
        match self {
            Self::AbsGe => "abs_ge",
            Self::Gt => "gt",
            Self::DiffPeakGt => "diff_peak_gt",
        }
    }
}

/// `config_trigger_analog oid=%c sos_filter_oid=%c` — allocate the object and
/// bind it to its filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigTriggerAnalog {
    /// The oid the config callback assigned.
    pub oid: u8,
    /// The `sos_filter` object's oid.
    pub sos_filter_oid: u8,
}

impl McuCommand for ConfigTriggerAnalog {
    const NAME: &'static str = "config_trigger_analog";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.sos_filter_oid),
        ]
    }
}

/// `trigger_analog_set_raw_range oid=%c raw_min=%i raw_max=%i` — the raw
/// samples outside which homing fails with `RAW_RANGE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerAnalogSetRawRange {
    /// The object's oid.
    pub oid: u8,
    /// Lowest acceptable raw sample.
    pub raw_min: i32,
    /// Highest acceptable raw sample.
    pub raw_max: i32,
}

impl McuCommand for TriggerAnalogSetRawRange {
    const NAME: &'static str = "trigger_analog_set_raw_range";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Int32(self.raw_min),
            ArgValue::Int32(self.raw_max),
        ]
    }
}

/// `trigger_analog_set_trigger oid=%c trigger_analog_type=%c trigger_value=%i`
/// — the comparison each filtered sample is checked against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerAnalogSetTrigger {
    /// The object's oid.
    pub oid: u8,
    /// Which comparison to apply.
    pub trigger_analog_type: TriggerAnalogType,
    /// The value the comparison uses.
    pub trigger_value: i32,
}

impl McuCommand for TriggerAnalogSetTrigger {
    const NAME: &'static str = "trigger_analog_set_trigger";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.trigger_analog_type as u8),
            ArgValue::Int32(self.trigger_value),
        ]
    }
}

/// `trigger_analog_home ...` — arm (or, all-zero, disable) the trigger check.
///
/// The firmware waits until `clock`, then accepts samples: each one is
/// range-checked, run through the `sos_filter`, and compared against the
/// trigger; a match fires `trsync_oid` with `trigger_reason`. Samples arriving
/// more than `monitor_max` windows of `monitor_ticks` apart cancel homing with
/// `error_reason` + `MONITOR` — the sensor went quiet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerAnalogHome {
    /// The object's oid.
    pub oid: u8,
    /// The trigger group to fire.
    pub trsync_oid: u8,
    /// The reason to fire with on a trigger match.
    pub trigger_reason: u8,
    /// The first reason a failure fires with; the `trigger_analog_error:`
    /// value is added to it.
    pub error_reason: u8,
    /// The clock the check starts at.
    pub clock: u32,
    /// Ticks between expected samples (the sensor's update period).
    pub monitor_ticks: u32,
    /// How many missed sample windows cancel homing; `0` disables the check,
    /// as does a zero `monitor_ticks`.
    pub monitor_max: u32,
}

impl TriggerAnalogHome {
    /// The all-zero message that disables checking — upstream's `_clear_home`
    /// sends it before reading the trigger clock back
    /// (`trigger_analog.py:300-304`).
    pub fn disable(oid: u8) -> Self {
        Self {
            oid,
            trsync_oid: 0,
            trigger_reason: 0,
            error_reason: 0,
            clock: 0,
            monitor_ticks: 0,
            monitor_max: 0,
        }
    }
}

impl McuCommand for TriggerAnalogHome {
    const NAME: &'static str = "trigger_analog_home";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.trsync_oid),
            ArgValue::UInt8(self.trigger_reason),
            ArgValue::UInt8(self.error_reason),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.monitor_ticks),
            ArgValue::UInt32(self.monitor_max),
        ]
    }
}

/// `trigger_analog_query_state oid=%c` — ask whether homing is armed and at
/// which clock the last (arm or trigger) event happened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerAnalogQueryState {
    /// The object's oid.
    pub oid: u8,
}

impl McuCommand for TriggerAnalogQueryState {
    const NAME: &'static str = "trigger_analog_query_state";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `trigger_analog_state oid=%c homing=%c homing_clock=%u` — the current
/// state, the answer to [`TriggerAnalogQueryState`].
///
/// After a trigger `homing_clock` is the firmware clock the trigger fired at;
/// while armed it is the arm clock (`src/trigger_analog.c:241-277`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerAnalogState {
    /// The object's oid.
    pub oid: u8,
    /// Whether a homing check is armed.
    pub homing: bool,
    /// The arm clock, or the trigger clock once fired.
    pub homing_clock: u32,
}

impl McuResponse for TriggerAnalogState {
    const NAME: &'static str = "trigger_analog_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            homing: params.get_u8("homing")? != 0,
            homing_clock: params.get_u32("homing_clock")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgType;

    /// The corpus dictionary: the field names, order and types every command
    /// here must match, straight from `src/trigger_analog.c`.
    fn atmega2560() -> Dictionary {
        let path = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let raw =
            std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        let value: serde_json::Value =
            serde_json::from_slice(&raw).expect("atmega2560.dict is JSON");
        Dictionary::from_json(value).expect("a valid data dictionary")
    }

    /// `name` must be in the dictionary with exactly `params`, and `cmd`'s
    /// arguments must encode to those declared types in that order.
    fn assert_matches_dictionary<C: McuCommand>(
        dict: &Dictionary,
        cmd: &C,
        params: &[(&str, ArgType)],
    ) {
        let mut parser = Parser::new();
        dict.install(&mut parser).expect("install the dictionary");
        let msg = parser
            .lookup(C::NAME)
            .unwrap_or_else(|| panic!("dictionary has no command '{}'", C::NAME));
        let declared: Vec<(&str, ArgType)> = msg
            .params
            .iter()
            .map(|(name, atype)| (name.as_str(), *atype))
            .collect();
        assert_eq!(declared, params, "'{}' parameters drifted", C::NAME);
        let args = cmd.args();
        assert_eq!(args.len(), params.len(), "'{}' argument count", C::NAME);
        for (index, (value, (_, atype))) in args.iter().zip(params).enumerate() {
            assert_eq!(
                value.arg_type(),
                *atype,
                "'{}' argument {index} ({:?})",
                C::NAME,
                declared[index].0
            );
        }
    }

    #[test]
    fn test_five_commands_and_the_state_response_match_the_dictionary() {
        let dict = atmega2560();

        assert_matches_dictionary(
            &dict,
            &ConfigTriggerAnalog {
                oid: 1,
                sos_filter_oid: 2,
            },
            &[("oid", ArgType::UInt8), ("sos_filter_oid", ArgType::UInt8)],
        );
        assert_matches_dictionary(
            &dict,
            &TriggerAnalogSetRawRange {
                oid: 1,
                raw_min: -1_000_000,
                raw_max: 1_000_000,
            },
            &[
                ("oid", ArgType::UInt8),
                ("raw_min", ArgType::Int32),
                ("raw_max", ArgType::Int32),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &TriggerAnalogSetTrigger {
                oid: 1,
                trigger_analog_type: TriggerAnalogType::Gt,
                trigger_value: 1500,
            },
            &[
                ("oid", ArgType::UInt8),
                ("trigger_analog_type", ArgType::UInt8),
                ("trigger_value", ArgType::Int32),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &TriggerAnalogHome {
                oid: 1,
                trsync_oid: 3,
                trigger_reason: 1,
                error_reason: REASON_TRIGGER_ANALOG,
                clock: 0x1234_5678,
                monitor_ticks: 40_000,
                monitor_max: 3,
            },
            &[
                ("oid", ArgType::UInt8),
                ("trsync_oid", ArgType::UInt8),
                ("trigger_reason", ArgType::UInt8),
                ("error_reason", ArgType::UInt8),
                ("clock", ArgType::UInt32),
                ("monitor_ticks", ArgType::UInt32),
                ("monitor_max", ArgType::UInt32),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &TriggerAnalogQueryState { oid: 1 },
            &[("oid", ArgType::UInt8)],
        );

        // The response decodes through the dictionary's own format string.
        let def = dict
            .message(TriggerAnalogState::NAME)
            .expect("trigger_analog_state in the dictionary");
        assert_eq!(
            def.format,
            "trigger_analog_state oid=%c homing=%c homing_clock=%u",
        );
        let mut parser = Parser::new();
        parser.register(def.id, &def.format).unwrap();
        let encoded = parser
            .encode(
                TriggerAnalogState::NAME,
                &[
                    ArgValue::UInt8(2),
                    ArgValue::UInt8(1),
                    ArgValue::UInt32(0x89AB_CDEF),
                ],
            )
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let msg = parser.lookup(TriggerAnalogState::NAME).unwrap();
        let state = TriggerAnalogState::decode(&Params::new(msg, &decoded[0].1)).unwrap();
        assert_eq!(
            state,
            TriggerAnalogState {
                oid: 2,
                homing: true,
                homing_clock: 0x89AB_CDEF,
            }
        );
    }

    #[test]
    fn test_trigger_analog_type_enum_matches_the_dictionary() {
        let dict = atmega2560();
        let enumeration = dict
            .enumeration("trigger_analog_type")
            .expect("trigger_analog_type in the dictionary");
        for kind in [
            TriggerAnalogType::AbsGe,
            TriggerAnalogType::Gt,
            TriggerAnalogType::DiffPeakGt,
        ] {
            assert_eq!(
                enumeration.value(kind.name()),
                Some(i64::from(kind as u8)),
                "'{}' wire value drifted",
                kind.name()
            );
        }
    }

    #[test]
    fn test_trigger_analog_error_reason_is_the_first_trsync_failure() {
        // `REASON_COMMS_TIMEOUT + 1`: the four error codes ride reasons 5..8,
        // all of which count as failures.
        assert_eq!(REASON_TRIGGER_ANALOG, 5);
        assert!(crate::core::klippy::cmd::trsync::raw_is_failure(
            REASON_TRIGGER_ANALOG
        ));
        assert!(!crate::core::klippy::cmd::trsync::raw_is_failure(
            TriggerReason::EndstopHit as u8
        ));
    }

    #[test]
    fn test_home_args_follow_the_firmware_order() {
        let cmd = TriggerAnalogHome {
            oid: 7,
            trsync_oid: 4,
            trigger_reason: 1,
            error_reason: REASON_TRIGGER_ANALOG,
            clock: 1000,
            monitor_ticks: 40_000,
            monitor_max: 3,
        };
        assert_eq!(
            cmd.args(),
            vec![
                ArgValue::UInt8(7),
                ArgValue::UInt8(4),
                ArgValue::UInt8(1),
                ArgValue::UInt8(5),
                ArgValue::UInt32(1000),
                ArgValue::UInt32(40_000),
                ArgValue::UInt32(3),
            ]
        );
    }

    #[test]
    fn test_disable_is_all_zero() {
        let args = TriggerAnalogHome::disable(9).args();
        assert_eq!(
            args,
            vec![
                ArgValue::UInt8(9),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt32(0),
                ArgValue::UInt32(0),
            ]
        );
    }
}
