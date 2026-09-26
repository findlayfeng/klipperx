//! ADS1220 commands: chip configuration, bulk queries, status query.
//!
//! Host view of the firmware's `src/sensor_ads1220.c`. Two groups:
//!
//! | group | commands |
//! |---|---|
//! | chip setup | [`ConfigAds1220`]; [`Ads1220AttachTriggerAnalog`] (optional, for `[load_cell_probe]`) |
//! | sampling | [`QueryAds1220`] (arm/disarm); [`QueryAds1220Status`] → `sensor_bulk_status` |
//!
//! The sample reports (`sensor_bulk_data`) and the status reply
//! (`sensor_bulk_status`) are shared with the LDC1612 and live in
//! [`crate::core::klippy::cmd::ldc1612`]; this module adds only the ADS1220's
//! own request names. The ADS1220 is an AVR-only chip (`sensor_ads1220.c` joins
//! the AVR build), so the corpus dictionaries that carry these four commands are
//! the `atmega*.dict` family — the HC32 dictionaries do not, which the tests at
//! the bottom pin down both ways.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_ads1220 oid=%c spi_oid=%c data_ready_pin=%u` — create the sensor on
/// an SPI bus plus its data-ready pin (`ads1220.py:ADS1220.__init__`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigAds1220 {
    /// The sensor's object id.
    pub oid: u8,
    /// The SPI bus resource's object id.
    pub spi_oid: u8,
    /// The firmware number of the `data_ready_pin` (DRDY).
    pub data_ready_pin: u32,
}

impl McuCommand for ConfigAds1220 {
    const NAME: &'static str = "config_ads1220";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.spi_oid),
            ArgValue::UInt32(self.data_ready_pin),
        ]
    }
}

/// `query_ads1220 oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`ads1220.py:_start_measurements` /
/// `_finish_measurements`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAds1220 {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms the chip.
    pub rest_ticks: u32,
}

impl McuCommand for QueryAds1220 {
    const NAME: &'static str = "query_ads1220";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// `query_ads1220_status oid=%c` — ask for the bulk channel's clock state; the
/// firmware answers with [`crate::core::klippy::cmd::ldc1612::SensorBulkStatus`]
/// (`FixedFreqReader.setup_query_command`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAds1220Status {
    /// The sensor's object id.
    pub oid: u8,
}

impl McuCommand for QueryAds1220Status {
    const NAME: &'static str = "query_ads1220_status";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `ads1220_attach_trigger_analog oid=%c trigger_analog_oid=%c` — hand this
/// chip's samples to `[trigger_analog]` (`ads1220.py:setup_trigger_analog`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ads1220AttachTriggerAnalog {
    /// The sensor's object id.
    pub oid: u8,
    /// The `[trigger_analog]` instance's object id.
    pub trigger_analog_oid: u8,
}

impl McuCommand for Ads1220AttachTriggerAnalog {
    const NAME: &'static str = "ads1220_attach_trigger_analog";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.trigger_analog_oid),
        ]
    }
}

/// The status-query message format the reader is parameterized with
/// (`setup_query_command("query_ads1220_status oid=%c", …)` upstream) — the
/// `msgformat` half of
/// [`crate::core::klippy::extras::bulk_sensor::FixedFreqReader::with_format`].
pub const QUERY_ADS1220_STATUS_MSGFORMAT: &str = "query_ads1220_status oid=%c";

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

    /// A dictionary plus its installed parser.
    fn parser_for(name: &str) -> (Dictionary, Parser) {
        let dict = dictionary(name);
        let mut parser = Parser::new();
        dict.install(&mut parser).expect("install the dictionary");
        (dict, parser)
    }

    #[test]
    fn test_commands_match_the_atmega2560_dictionary() {
        let (dict, parser) = parser_for("atmega2560.dict");
        assert_matches_dictionary(
            &dict,
            &ConfigAds1220 {
                oid: 1,
                spi_oid: 2,
                data_ready_pin: 3,
            },
            &[
                ("oid", ArgType::UInt8),
                ("spi_oid", ArgType::UInt8),
                ("data_ready_pin", ArgType::UInt32),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &QueryAds1220 {
                oid: 4,
                rest_ticks: 5,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryAds1220Status { oid: 7 },
            &[("oid", ArgType::UInt8)],
        );
        assert_matches_dictionary(
            &dict,
            &Ads1220AttachTriggerAnalog {
                oid: 8,
                trigger_analog_oid: 9,
            },
            &[
                ("oid", ArgType::UInt8),
                ("trigger_analog_oid", ArgType::UInt8),
            ],
        );

        // The reader's message format names a command the firmware has, and
        // `SensorBulkStatus` is the reply both the request and the reader
        // depend on.
        let request = QUERY_ADS1220_STATUS_MSGFORMAT
            .split_whitespace()
            .next()
            .expect("the format names a command");
        assert_eq!(request, QueryAds1220Status::NAME);
        assert!(parser.lookup(request).is_some(), "no '{request}'");
        assert!(
            parser
                .lookup(crate::core::klippy::cmd::ldc1612::SensorBulkStatus::NAME)
                .is_some(),
            "no 'sensor_bulk_status'"
        );
    }

    #[test]
    fn test_the_hc32f460_dictionary_carries_no_ads1220() {
        // The ADS1220 firmware is AVR-only, so the HC32 dictionaries have none
        // of the four commands — a driver that assumed otherwise would only
        // fail on the second board. The shared bulk replies stay, though: they
        // are what a load-cell chip of any family answers with.
        let (_, parser) = parser_for("hc32f460-serial-PA3PA2.dict");
        for name in [
            ConfigAds1220::NAME,
            QueryAds1220::NAME,
            QueryAds1220Status::NAME,
            Ads1220AttachTriggerAnalog::NAME,
        ] {
            assert!(parser.lookup(name).is_none(), "{name} is AVR-only");
        }
        assert!(
            parser
                .lookup(crate::core::klippy::cmd::ldc1612::SensorBulkStatus::NAME)
                .is_some(),
            "the shared status reply is not AVR-only"
        );
    }
}
