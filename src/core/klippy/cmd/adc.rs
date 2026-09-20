//! ADC commands.
//!
//! Host view of the "ADC" section of the firmware's `adccmds.c`. An analog input
//! is created once with `config_analog_in`, then the firmware samples it on a
//! periodic query and pushes each result:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_analog_in oid=%c pin=%u` |
//! | host → MCU | `query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u bytes_per_report=%c min_value=%hu max_value=%hu range_check_count=%c` |
//! | MCU → host | `analog_in_state oid=%c next_clock=%u values=%*s` |
//!
//! The query arms a timer: `sample_count` samples are averaged over
//! `sample_ticks` once, `rest_ticks` apart from the previous report, starting at
//! the absolute `clock`. `min_value`/`max_value` bound the averaged result (a
//! reading outside them, `range_check_count` times in a row, stops the firmware).
//!
//! # Two wire formats
//!
//! Older firmware answers with one `value=%hu`; newer firmware batches up to
//! `batch_num` samples into `values=%*s` (little-endian `u16`s), which is what
//! `bytes_per_report` sizes. Both formats ship under the same `analog_in_state`
//! name and are told apart by their **format string** in the dictionary, so the
//! resource that builds the query picks the matching one
//! (`mcu/resource/adc.rs`).

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_analog_in oid=%c pin=%u`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigAnalogIn {
    /// The object id the firmware allocated for this input.
    pub oid: u8,
    /// Numeric pin (the `pin` enumeration value).
    pub pin: u32,
}

impl McuCommand for ConfigAnalogIn {
    const NAME: &'static str = "config_analog_in";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.pin)]
    }
}

/// `query_analog_in` with the batched `bytes_per_report` parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAnalogIn {
    /// The input's oid.
    pub oid: u8,
    /// Absolute firmware clock of the first report.
    pub clock: u32,
    /// Time between samples in one report, in clock ticks.
    pub sample_ticks: u32,
    /// How many samples each report averages.
    pub sample_count: u8,
    /// Time between reports, in clock ticks.
    pub rest_ticks: u32,
    /// Bytes in one `values=%*s` report (`batch_num * 2`).
    pub bytes_per_report: u8,
    /// Lowest accepted average, in ADC counts.
    pub min_value: u16,
    /// Highest accepted average, in ADC counts.
    pub max_value: u16,
    /// How many out-of-range reports in a row stop the firmware.
    pub range_check_count: u8,
}

impl McuCommand for QueryAnalogIn {
    const NAME: &'static str = "query_analog_in";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.sample_ticks),
            ArgValue::UInt8(self.sample_count),
            ArgValue::UInt32(self.rest_ticks),
            ArgValue::UInt8(self.bytes_per_report),
            ArgValue::UInt16(self.min_value),
            ArgValue::UInt16(self.max_value),
            ArgValue::UInt8(self.range_check_count),
        ]
    }
}

/// `query_analog_in` without `bytes_per_report` (older firmware).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAnalogInOld {
    /// The input's oid.
    pub oid: u8,
    /// Absolute firmware clock of the first report.
    pub clock: u32,
    /// Time between samples in one report, in clock ticks.
    pub sample_ticks: u32,
    /// How many samples each report averages.
    pub sample_count: u8,
    /// Time between reports, in clock ticks.
    pub rest_ticks: u32,
    /// Lowest accepted average, in ADC counts.
    pub min_value: u16,
    /// Highest accepted average, in ADC counts.
    pub max_value: u16,
    /// How many out-of-range reports in a row stop the firmware.
    pub range_check_count: u8,
}

impl McuCommand for QueryAnalogInOld {
    const NAME: &'static str = "query_analog_in";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.sample_ticks),
            ArgValue::UInt8(self.sample_count),
            ArgValue::UInt32(self.rest_ticks),
            ArgValue::UInt16(self.min_value),
            ArgValue::UInt16(self.max_value),
            ArgValue::UInt8(self.range_check_count),
        ]
    }
}

/// `analog_in_state oid=%c next_clock=%u values=%*s` — a batch of samples.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AnalogInState {
    /// The input's oid.
    pub oid: u8,
    /// Firmware clock of the report after this one.
    pub next_clock: u32,
    /// Little-endian `u16` ADC samples, oldest first.
    pub values: Vec<u8>,
}

impl AnalogInState {
    /// The batch as `u16` samples.
    ///
    /// A trailing odd byte (a malformed report) is ignored.
    pub fn samples(&self) -> Vec<u16> {
        self.values
            .chunks_exact(2)
            .map(|pair| u16::from_le_bytes([pair[0], pair[1]]))
            .collect()
    }
}

impl McuResponse for AnalogInState {
    const NAME: &'static str = "analog_in_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            next_clock: params.get_u32("next_clock")?,
            values: params.get_bytes("values")?,
        })
    }
}

/// `analog_in_state oid=%c next_clock=%u value=%hu` — a single averaged sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnalogInStateOld {
    /// The input's oid.
    pub oid: u8,
    /// Firmware clock of the report after this one.
    pub next_clock: u32,
    /// The averaged ADC value.
    pub value: u16,
}

impl McuResponse for AnalogInStateOld {
    const NAME: &'static str = "analog_in_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            next_clock: params.get_u32("next_clock")?,
            value: params.get_u16("value")?,
        })
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
                "config_analog_in oid=%c pin=%u": 30,
                "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u bytes_per_report=%c min_value=%hu max_value=%hu range_check_count=%c": 31,
                "query_analog_in_old oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c": 32
            },
            "responses": {
                "analog_in_state oid=%c next_clock=%u values=%*s": 33
            }
        }))
        .unwrap();
        let mut parser = Parser::new();
        dictionary.install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn test_config_and_query_match_the_firmware_format() {
        let parser = parser();

        let config = ConfigAnalogIn { oid: 2, pin: 7 };
        let encoded = parser.encode(ConfigAnalogIn::NAME, &config.args()).unwrap();
        assert_eq!(parser.decode(encoded).unwrap()[0].1, config.args());

        let query = QueryAnalogIn {
            oid: 2,
            clock: 1_000,
            sample_ticks: 20,
            sample_count: 8,
            rest_ticks: 1_000_000,
            bytes_per_report: 4,
            min_value: 0,
            max_value: 4000,
            range_check_count: 3,
        };
        let encoded = parser.encode(QueryAnalogIn::NAME, &query.args()).unwrap();
        assert_eq!(parser.decode(encoded).unwrap()[0].1, query.args());

        let old = QueryAnalogInOld {
            oid: 2,
            clock: 1_000,
            sample_ticks: 20,
            sample_count: 8,
            rest_ticks: 1_000_000,
            min_value: 0,
            max_value: 4000,
            range_check_count: 3,
        };
        let encoded = parser.encode("query_analog_in_old", &old.args()).unwrap();
        assert_eq!(parser.decode(encoded).unwrap()[0].1, old.args());
    }

    #[test]
    fn test_analog_in_state_decodes_a_batch() {
        let parser = parser();
        let encoded = parser
            .encode(
                AnalogInState::NAME,
                &[
                    ArgValue::UInt8(2),
                    ArgValue::UInt32(500),
                    ArgValue::Bytes(vec![0x10, 0x27, 0x0f, 0x27]),
                ],
            )
            .unwrap();
        let msg = parser.decode(encoded).unwrap().remove(0);
        let params = Params::new(msg.0, &msg.1);

        let state = AnalogInState::decode(&params).unwrap();

        assert_eq!(state.oid, 2);
        assert_eq!(state.next_clock, 500);
        // 0x2710 = 10000, 0x270f = 9999.
        assert_eq!(state.samples(), vec![10000, 9999]);
    }

    #[test]
    fn test_a_mistyped_parameter_is_a_decode_error() {
        let parser = parser();
        let encoded = parser
            .encode(
                AnalogInState::NAME,
                &[
                    ArgValue::UInt8(2),
                    ArgValue::UInt32(500),
                    ArgValue::Bytes(vec![0, 0]),
                ],
            )
            .unwrap();
        let msg = parser.decode(encoded).unwrap().remove(0);
        // Asking for the batch as a single `value` is what the old format does;
        // the new declaration has no such parameter.
        let params = Params::new(msg.0, &msg.1);
        assert!(matches!(
            AnalogInStateOld::decode(&params),
            Err(McuError::Decode(_))
        ));
    }
}
