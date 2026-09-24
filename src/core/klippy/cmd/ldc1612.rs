//! LDC1612 commands: configuration, bulk queries, and the bulk data channel.
//!
//! Host view of the firmware's `src/extras/sensor_ldc1612.c` plus the shared
//! `sensor_bulk_*` reporting channel (`src/sensor_bulk.c`). Three groups:
//!
//! | group | commands |
//! |---|---|
//! | chip setup | `config_ldc1612`, `config_ldc1612_with_intb` |
//! | sampling | `query_ldc1612` (arm/disarm), `query_status_ldc1612` → `sensor_bulk_status` |
//! | trigger attach | `ldc1612_attach_trigger_analog` |
//! | bulk reports | `sensor_bulk_data` (unsolicited), `sensor_bulk_status` (query reply) |
//!
//! Every name and field order here is checked against the corpus dictionary
//! (`atmega2560.dict`) by the tests at the bottom.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::msg::proto::ArgValue;

// ===========================================================================
// Configuration
// ===========================================================================

/// `config_ldc1612 oid=%c i2c_oid=%c` — create the sensor on an I2C device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigLdc1612 {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the I2C device the chip hangs off.
    pub i2c_oid: u8,
}

impl McuCommand for ConfigLdc1612 {
    const NAME: &'static str = "config_ldc1612";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.i2c_oid)]
    }
}

/// `config_ldc1612_with_intb oid=%c i2c_oid=%c intb_pin=%c` — as
/// [`ConfigLdc1612`], plus the data-ready interrupt pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigLdc1612WithIntb {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the I2C device the chip hangs off.
    pub i2c_oid: u8,
    /// The firmware number of the `intb_pin`.
    pub intb_pin: u8,
}

impl McuCommand for ConfigLdc1612WithIntb {
    const NAME: &'static str = "config_ldc1612_with_intb";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.i2c_oid),
            ArgValue::UInt8(self.intb_pin),
        ]
    }
}

// ===========================================================================
// Sampling
// ===========================================================================

/// `query_ldc1612 oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`ldc1612.py:_start_measurements` / `_finish`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryLdc1612 {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms the sensor.
    pub rest_ticks: u32,
}

impl McuCommand for QueryLdc1612 {
    const NAME: &'static str = "query_ldc1612";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// `query_status_ldc1612 oid=%c` — ask for the bulk channel's clock state;
/// the firmware answers with [`SensorBulkStatus`]
/// (`FixedFreqReader.setup_query_command`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryStatusLdc1612 {
    /// The sensor's object id.
    pub oid: u8,
}

impl McuCommand for QueryStatusLdc1612 {
    const NAME: &'static str = "query_status_ldc1612";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `ldc1612_attach_trigger_analog oid=%c trigger_analog_oid=%c` — hand this
/// sensor's samples to a `trigger_analog` object (`ldc1612.py:setup_trigger_analog`,
/// an init command).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ldc1612AttachTriggerAnalog {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the `trigger_analog` object that will watch this sensor
    /// (the M5a resource).
    pub trigger_analog_oid: u8,
}

impl McuCommand for Ldc1612AttachTriggerAnalog {
    const NAME: &'static str = "ldc1612_attach_trigger_analog";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.trigger_analog_oid),
        ]
    }
}

// ===========================================================================
// Responses
// ===========================================================================

/// `sensor_bulk_status oid=%c clock=%u query_ticks=%u next_sequence=%hu
/// buffered=%u possible_overflows=%hu` — the reply to
/// [`QueryStatusLdc1612`], carrying everything clock synchronization needs
/// (`bulk_sensor.FixedFreqReader._update_clock`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorBulkStatus {
    /// The sensor's object id.
    pub oid: u8,
    /// The firmware clock when the status was sampled.
    pub clock: u32,
    /// How long the status query itself took, in ticks.
    pub query_ticks: u32,
    /// The sequence number the *next* `sensor_bulk_data` message will carry.
    pub next_sequence: u16,
    /// Bytes buffered for the next report, still short of a full message.
    pub buffered: u32,
    /// Total messages the firmware could not send (wrapped at 16 bits).
    pub possible_overflows: u16,
}

impl McuResponse for SensorBulkStatus {
    const NAME: &'static str = "sensor_bulk_status";

    fn decode(params: &Params<'_>) -> Result<Self, McuError2> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            clock: params.get_u32("clock")?,
            query_ticks: params.get_u32("query_ticks")?,
            next_sequence: params.get_u16("next_sequence")?,
            buffered: params.get_u32("buffered")?,
            possible_overflows: params.get_u16("possible_overflows")?,
        })
    }
}

/// `sensor_bulk_data oid=%c sequence=%hu data=%*s` — one block of packed
/// samples, sent unsolicited while `query_ldc1612` is armed
/// (`bulk_sensor.BulkDataQueue`'s message).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SensorBulkData {
    /// The sensor's object id.
    pub oid: u8,
    /// This message's sequence number (wraps at 16 bits).
    pub sequence: u16,
    /// The packed samples: 4 bytes each, big-endian (`unpack ">I"`).
    pub data: Vec<u8>,
}

impl McuResponse for SensorBulkData {
    const NAME: &'static str = "sensor_bulk_data";

    fn decode(params: &Params<'_>) -> Result<Self, McuError2> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            sequence: params.get_u16("sequence")?,
            data: params.get_bytes("data")?,
        })
    }
}

/// The error type [`McuResponse::decode`] reports; named for import hygiene
/// with the transport's own error.
type McuError2 = crate::core::klippy::mcu::McuError;

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgType;

    /// The corpus dictionary: the field names, order and types every command
    /// here must match, straight from the firmware.
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

    /// A response type must decode from the dictionary's declared parameters.
    fn assert_response_matches_dictionary<R: McuResponse>(
        dict: &Dictionary,
        _response: &R,
        params: &[(&str, ArgType)],
    ) {
        let mut parser = Parser::new();
        dict.install(&mut parser).expect("install the dictionary");
        let msg = parser
            .lookup(R::NAME)
            .unwrap_or_else(|| panic!("dictionary has no response '{}'", R::NAME));
        let declared: Vec<(&str, ArgType)> = msg
            .params
            .iter()
            .map(|(name, atype)| (name.as_str(), *atype))
            .collect();
        assert_eq!(declared, params, "'{}' parameters drifted", R::NAME);
    }

    #[test]
    fn test_commands_match_the_corpus_dictionary() {
        let dict = atmega2560();

        assert_matches_dictionary(
            &dict,
            &ConfigLdc1612 { oid: 1, i2c_oid: 2 },
            &[("oid", ArgType::UInt8), ("i2c_oid", ArgType::UInt8)],
        );
        assert_matches_dictionary(
            &dict,
            &ConfigLdc1612WithIntb {
                oid: 1,
                i2c_oid: 2,
                intb_pin: 3,
            },
            &[
                ("oid", ArgType::UInt8),
                ("i2c_oid", ArgType::UInt8),
                ("intb_pin", ArgType::UInt8),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &QueryLdc1612 {
                oid: 4,
                rest_ticks: 5,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryStatusLdc1612 { oid: 4 },
            &[("oid", ArgType::UInt8)],
        );
        assert_matches_dictionary(
            &dict,
            &Ldc1612AttachTriggerAnalog {
                oid: 6,
                trigger_analog_oid: 7,
            },
            &[
                ("oid", ArgType::UInt8),
                ("trigger_analog_oid", ArgType::UInt8),
            ],
        );
    }

    #[test]
    fn test_responses_match_the_corpus_dictionary() {
        let dict = atmega2560();

        assert_response_matches_dictionary(
            &dict,
            &SensorBulkStatus {
                oid: 0,
                clock: 0,
                query_ticks: 0,
                next_sequence: 0,
                buffered: 0,
                possible_overflows: 0,
            },
            &[
                ("oid", ArgType::UInt8),
                ("clock", ArgType::UInt32),
                ("query_ticks", ArgType::UInt32),
                ("next_sequence", ArgType::UInt16),
                ("buffered", ArgType::UInt32),
                ("possible_overflows", ArgType::UInt16),
            ],
        );
        assert_response_matches_dictionary(
            &dict,
            &SensorBulkData {
                oid: 0,
                sequence: 0,
                data: Vec::new(),
            },
            &[
                ("oid", ArgType::UInt8),
                ("sequence", ArgType::UInt16),
                ("data", ArgType::Bytes),
            ],
        );
    }

    #[test]
    fn test_sensor_bulk_status_decodes_named_fields() {
        let dict = atmega2560();
        let mut parser = Parser::new();
        dict.install(&mut parser).unwrap();
        let msg = parser.lookup("sensor_bulk_status").unwrap().clone();
        let params = Params::new(
            std::sync::Arc::new(msg),
            &[
                ArgValue::UInt8(3),
                ArgValue::UInt32(1_000_000),
                ArgValue::UInt32(64),
                ArgValue::UInt16(7),
                ArgValue::UInt32(48),
                ArgValue::UInt16(1),
            ],
        );
        let status = SensorBulkStatus::decode(&params).expect("decodes");
        assert_eq!(
            status,
            SensorBulkStatus {
                oid: 3,
                clock: 1_000_000,
                query_ticks: 64,
                next_sequence: 7,
                buffered: 48,
                possible_overflows: 1,
            }
        );
    }
}
