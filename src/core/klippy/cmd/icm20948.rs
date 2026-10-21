//! ICM20948 commands: chip configuration and the bulk query arm/disarm.
//!
//! Host view of the firmware's `src/sensor_icm20948.c`. The ICM20948 is the
//! I2C sibling of the MPU9250: the chip is created on an I2C device
//! ([`ConfigIcm20948`]), samples are reported on the shared `sensor_bulk_*`
//! channel, and the periodic report is armed with [`QueryIcm20948`].
//!
//! | group | commands |
//! |---|---|
//! | chip setup | [`ConfigIcm20948`] |
//! | sampling | [`QueryIcm20948`] (arm/disarm); [`QUERY_ICM20948_STATUS`] → `sensor_bulk_status` |
//!
//! The sample reports (`sensor_bulk_data`) and the status reply
//! (`sensor_bulk_status`) are shared with the LDC1612 and live in
//! [`crate::core::klippy::cmd::ldc1612`]; this module adds only the ICM20948's
//! own request names. Every name and field order here is checked against the
//! corpus dictionary (`atmega2560.dict`) by the tests at the bottom.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_icm20948 oid=%c i2c_oid=%c` — create the sensor on an I2C device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigIcm20948 {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the I2C device the chip hangs off.
    pub i2c_oid: u8,
}

impl McuCommand for ConfigIcm20948 {
    const NAME: &'static str = "config_icm20948";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.i2c_oid)]
    }
}

/// `query_icm20948 oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`icm20948.py:_start_measurements` / `_finish`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryIcm20948 {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms the sensor.
    pub rest_ticks: u32,
}

impl McuCommand for QueryIcm20948 {
    const NAME: &'static str = "query_icm20948";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// The bulk status query command name (`query_icm20948_status oid=%c`); the
/// reply is the shared `sensor_bulk_status`.
///
/// Upstream's `bulk_sensor.FixedFreqReader.setup_query_command` takes this as a
/// string, so each sensor names its own status request; `[icm20948]` builds the
/// reader with this name (see `extras::icm20948`).
pub const QUERY_ICM20948_STATUS: &str = "query_icm20948_status";

/// The status query's message format (`query_icm20948_status oid=%c`), the
/// `msgformat` half of upstream's `setup_query_command`.
pub const QUERY_ICM20948_STATUS_MSGFORMAT: &str = "query_icm20948_status oid=%c";

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

    #[test]
    fn test_commands_match_the_corpus_dictionary() {
        let dict = atmega2560();

        assert_matches_dictionary(
            &dict,
            &ConfigIcm20948 { oid: 1, i2c_oid: 2 },
            &[("oid", ArgType::UInt8), ("i2c_oid", ArgType::UInt8)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryIcm20948 {
                oid: 4,
                rest_ticks: 5,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
    }

    #[test]
    fn test_the_status_query_name_is_declared_by_the_firmware() {
        let dict = atmega2560();
        let mut parser = Parser::new();
        dict.install(&mut parser).unwrap();
        let msg = parser
            .lookup(QUERY_ICM20948_STATUS)
            .unwrap_or_else(|| panic!("dictionary has no command '{QUERY_ICM20948_STATUS}'"));
        let declared: Vec<(&str, ArgType)> = msg
            .params
            .iter()
            .map(|(name, atype)| (name.as_str(), *atype))
            .collect();
        assert_eq!(declared, [("oid", ArgType::UInt8)]);
    }
}
