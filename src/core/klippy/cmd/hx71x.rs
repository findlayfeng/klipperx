//! HX711/HX717 commands: chip configuration, bulk queries, status query.
//!
//! Host view of the firmware's `src/sensor_hx71x.c`. Two groups:
//!
//! | group | commands |
//! |---|---|
//! | chip setup | [`ConfigHx71x`] |
//! | sampling | [`QueryHx71x`] (arm/disarm); [`QueryHx71xStatus`] → `sensor_bulk_status` |
//!
//! The sample reports (`sensor_bulk_data`) and the status reply
//! (`sensor_bulk_status`) are shared with the LDC1612 and live in
//! [`crate::core::klippy::cmd::ldc1612`]; this module adds only the HX71x's own
//! request names. Every name and field order here is checked against the
//! corpus dictionaries (`atmega2560.dict`, `hc32f460-serial-PA3PA2.dict`) by
//! the tests at the bottom.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_hx71x oid=%c gain_channel=%c dout_pin=%u sclk_pin=%u` — create the
/// sensor on two GPIO pins (`hx71x.py:_HX71xBase__init__`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigHx71x {
    /// The sensor's object id.
    pub oid: u8,
    /// The gain/channel select (HX711: 1-3, HX717: 1-4).
    pub gain_channel: u8,
    /// The firmware number of the `dout_pin`.
    pub dout_pin: u32,
    /// The firmware number of the `sclk_pin`.
    pub sclk_pin: u32,
}

impl McuCommand for ConfigHx71x {
    const NAME: &'static str = "config_hx71x";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.gain_channel),
            ArgValue::UInt32(self.dout_pin),
            ArgValue::UInt32(self.sclk_pin),
        ]
    }
}

/// `query_hx71x oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`hx71x.py:_start_measurements` / `_finish_measurements`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryHx71x {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms (and power-cycles) the
    /// chip.
    pub rest_ticks: u32,
}

impl McuCommand for QueryHx71x {
    const NAME: &'static str = "query_hx71x";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// `query_hx71x_status oid=%c` — ask for the bulk channel's clock state; the
/// firmware answers with [`crate::core::klippy::cmd::ldc1612::SensorBulkStatus`]
/// (`FixedFreqReader.setup_query_command`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryHx71xStatus {
    /// The sensor's object id.
    pub oid: u8,
}

impl McuCommand for QueryHx71xStatus {
    const NAME: &'static str = "query_hx71x_status";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// The status-query message format the reader is parameterized with
/// (`setup_query_command("query_hx71x_status oid=%c", …)` upstream) — the
/// `msgformat` half of [`crate::core::klippy::extras::bulk_sensor::FixedFreqReader::with_format`].
pub const QUERY_HX71X_STATUS_MSGFORMAT: &str = "query_hx71x_status oid=%c";

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

    /// The three HX71x commands, on both dictionaries that carry them: the
    /// AVR corpus and the HC32 (which has no `*_attach_trigger_analog` for
    /// hx71x's sibling chips but does have this trio).
    fn assert_hx71x_commands(name: &str) {
        let dict = dictionary(name);
        assert_matches_dictionary(
            &dict,
            &ConfigHx71x {
                oid: 1,
                gain_channel: 2,
                dout_pin: 3,
                sclk_pin: 4,
            },
            &[
                ("oid", ArgType::UInt8),
                ("gain_channel", ArgType::UInt8),
                ("dout_pin", ArgType::UInt32),
                ("sclk_pin", ArgType::UInt32),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &QueryHx71x {
                oid: 4,
                rest_ticks: 5,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryHx71xStatus { oid: 7 },
            &[("oid", ArgType::UInt8)],
        );

        // The reader's message format names a command the firmware has, and
        // `SensorBulkStatus` is the reply both the request and the reader
        // depend on.
        let mut parser = Parser::new();
        dict.install(&mut parser).expect("install the dictionary");
        let request = QUERY_HX71X_STATUS_MSGFORMAT
            .split_whitespace()
            .next()
            .expect("the format names a command");
        assert_eq!(request, QueryHx71xStatus::NAME);
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
        assert_hx71x_commands("atmega2560.dict");
    }

    #[test]
    fn test_commands_match_the_hc32f460_dictionary() {
        assert_hx71x_commands("hc32f460-serial-PA3PA2.dict");
    }
}
