//! ADS131M0x commands: chip configuration, bulk queries, status query.
//!
//! Host view of the firmware's `src/sensor_ads131m0x.c`. Three groups:
//!
//! | group | commands |
//! |---|---|
//! | chip setup | [`ConfigAds131M0x`] |
//! | bus sharing | [`Ads131M0xAttachTriggerAnalog`] |
//! | sampling | [`QueryAds131M0x`] (arm/disarm); [`QueryAds131M0xStatus`] → `sensor_bulk_status` |
//!
//! The sample reports (`sensor_bulk_data`) and the status reply
//! (`sensor_bulk_status`) are shared with the LDC1612 and live in
//! [`crate::core::klippy::cmd::ldc1612`]; this module adds only the ADS131M0x's
//! own request names. Every name and field order here is checked against the
//! corpus dictionary (`atmega2560.dict`) by the tests at the bottom — the chip
//! is not built into the HC32F460 configuration, so the hc32 test asserts its
//! absence instead.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_ads131m0x oid=%c spi_oid=%c channel=%c num_channels=%c
/// data_ready_pin=%u` — create the sensor on an SPI bus plus a data-ready GPIO
/// (`ads131m0x.py:__init__`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigAds131M0x {
    /// The sensor's object id.
    pub oid: u8,
    /// The SPI device oid the sensor reads through.
    pub spi_oid: u8,
    /// The ADC channel the load cell is wired to.
    pub channel: u8,
    /// The chip's channel count (2 for an ADS131M02, 4 for an ADS131M04).
    pub num_channels: u8,
    /// The firmware number of the `data_ready_pin`.
    pub data_ready_pin: u32,
}

impl McuCommand for ConfigAds131M0x {
    const NAME: &'static str = "config_ads131m0x";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.spi_oid),
            ArgValue::UInt8(self.channel),
            ArgValue::UInt8(self.num_channels),
            ArgValue::UInt32(self.data_ready_pin),
        ]
    }
}

/// `ads131m0x_attach_trigger_analog oid=%c trigger_analog_oid=%c` — hand the
/// sensor's data-ready line to a `trigger_analog` consumer
/// (`ads131m0x.py:setup_trigger_analog`, added `is_init`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ads131M0xAttachTriggerAnalog {
    /// The sensor's object id.
    pub oid: u8,
    /// The `trigger_analog` object's oid.
    pub trigger_analog_oid: u8,
}

impl McuCommand for Ads131M0xAttachTriggerAnalog {
    const NAME: &'static str = "ads131m0x_attach_trigger_analog";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.trigger_analog_oid),
        ]
    }
}

/// `query_ads131m0x oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`ads131m0x.py:_start_measurements` /
/// `_finish_measurements`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAds131M0x {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms the chip.
    pub rest_ticks: u32,
}

impl McuCommand for QueryAds131M0x {
    const NAME: &'static str = "query_ads131m0x";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// `query_ads131m0x_status oid=%c` — ask for the bulk channel's clock state;
/// the firmware answers with
/// [`crate::core::klippy::cmd::ldc1612::SensorBulkStatus`]
/// (`FixedFreqReader.setup_query_command`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAds131M0xStatus {
    /// The sensor's object id.
    pub oid: u8,
}

impl McuCommand for QueryAds131M0xStatus {
    const NAME: &'static str = "query_ads131m0x_status";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// The status-query message format the reader is parameterized with
/// (`setup_query_command("query_ads131m0x_status oid=%c", …)` upstream) — the
/// `msgformat` half of
/// [`crate::core::klippy::extras::bulk_sensor::FixedFreqReader::with_format`].
pub const QUERY_ADS131M0X_STATUS_MSGFORMAT: &str = "query_ads131m0x_status oid=%c";

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::McuResponse;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgType;

    /// One corpus dictionary: the field names, order and types every command
    /// here must match, straight from the firmware.
    fn dictionary(name: &str) -> Dictionary {
        let path = klipperx_test_support::test_dicts_dir().join(name);
        let raw =
            std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        let value: serde_json::Value =
            serde_json::from_slice(&raw).unwrap_or_else(|e| panic!("{name} is JSON: {e}"));
        Dictionary::from_json(value).unwrap_or_else(|e| panic!("{name} is a data dictionary: {e}"))
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

    /// The four ADS131M0x commands, on the AVR corpus dictionary — the only
    /// one that builds the chip's firmware.
    fn assert_ads131m0x_commands(name: &str) {
        let dict = dictionary(name);
        assert_matches_dictionary(
            &dict,
            &ConfigAds131M0x {
                oid: 1,
                spi_oid: 2,
                channel: 3,
                num_channels: 4,
                data_ready_pin: 5,
            },
            &[
                ("oid", ArgType::UInt8),
                ("spi_oid", ArgType::UInt8),
                ("channel", ArgType::UInt8),
                ("num_channels", ArgType::UInt8),
                ("data_ready_pin", ArgType::UInt32),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &Ads131M0xAttachTriggerAnalog {
                oid: 6,
                trigger_analog_oid: 7,
            },
            &[
                ("oid", ArgType::UInt8),
                ("trigger_analog_oid", ArgType::UInt8),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &QueryAds131M0x {
                oid: 8,
                rest_ticks: 9,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryAds131M0xStatus { oid: 10 },
            &[("oid", ArgType::UInt8)],
        );

        // The reader's message format names a command the firmware has, and
        // `SensorBulkStatus` is the reply both the request and the reader
        // depend on.
        let mut parser = Parser::new();
        dict.install(&mut parser).expect("install the dictionary");
        let request = QUERY_ADS131M0X_STATUS_MSGFORMAT
            .split_whitespace()
            .next()
            .expect("the format names a command");
        assert_eq!(request, QueryAds131M0xStatus::NAME);
        assert!(
            parser.lookup(request).is_some(),
            "{name} has no '{request}'"
        );
        assert!(
            parser
                .lookup(crate::core::klippy::cmd::ldc1612::SensorBulkStatus::NAME)
                .is_some(),
            "{name} has no 'sensor_bulk_status'"
        );
    }

    #[test]
    fn test_commands_match_the_atmega2560_dictionary() {
        assert_ads131m0x_commands("atmega2560.dict");
    }

    #[test]
    fn test_the_hc32f460_dictionary_does_not_build_the_ads131m0x() {
        // The corpus's HC32F460 config leaves `WANT_ADS131M0X` unset, so the
        // firmware (and its commands) are absent there: the same suite must
        // not claim coverage the target does not have.
        let dict = dictionary("hc32f460-serial-PA3PA2.dict");
        for name in [
            ConfigAds131M0x::NAME,
            Ads131M0xAttachTriggerAnalog::NAME,
            QueryAds131M0x::NAME,
            QueryAds131M0xStatus::NAME,
        ] {
            assert!(
                !dict.messages().any(|message| message.name == name),
                "{name} must be absent from hc32f460-serial-PA3PA2.dict"
            );
        }
    }
}
