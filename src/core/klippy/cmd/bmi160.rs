//! BMI160 commands: chip configuration on an I2C device and the bulk query
//! arm/disarm (upstream's `klippy/extras/bmi160.py`; firmware
//! `src/sensor_bmi160.c`).
//!
//! | command | fields | upstream |
//! |---|---|---|
//! | `config_bmi160` | `oid=%c bus_oid=%c bus_oid_type=%c` | `BMI160.__init__` |
//! | `query_bmi160` | `oid=%c rest_ticks=%u` | `BMI160._start_measurements` / `_finish_measurements` |
//! | `query_bmi160_status` | `oid=%c` | `FixedFreqReader.setup_query_command` |
//!
//! The reply to `query_bmi160_status` is the shared
//! [`SensorBulkStatus`](super::ldc1612::SensorBulkStatus) message, not a name of
//! its own — every bulk sensor reports through the same channel
//! (`src/sensor_bulk.c`).
//!
//! `bus_oid_type` is the firmware's `DECL_ENUMERATION("bus_oid_type", …)`: the
//! host sends the enumeration's integer ([`BUS_OID_TYPE_I2C`] for the I2C
//! device this section creates, upstream's `self.bus_type` string).
//!
//! Every name and parameter order here is checked against the corpus dictionary
//! (`atmega2560.dict`) by the tests at the bottom.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// The `bus_oid_type` value the firmware declares for an I2C device
/// (`DECL_ENUMERATION("bus_oid_type", "i2c", I2C_SERIAL)` in `sensor_bmi160.c`).
pub const BUS_OID_TYPE_I2C: u8 = 1;

/// `config_bmi160 oid=%c bus_oid=%c bus_oid_type=%c` — create the sensor on a
/// bus device (`BMI160.__init__`'s `mcu.add_config_cmd`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigBmi160 {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the bus device (I2C or SPI) the chip hangs off.
    pub bus_oid: u8,
    /// Which of the two the `bus_oid` names ([`BUS_OID_TYPE_I2C`] here).
    pub bus_oid_type: u8,
}

impl McuCommand for ConfigBmi160 {
    const NAME: &'static str = "config_bmi160";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.bus_oid),
            ArgValue::UInt8(self.bus_oid_type),
        ]
    }
}

/// `query_bmi160 oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`BMI160._start_measurements` /
/// `_finish_measurements`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryBmi160 {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms the sensor.
    pub rest_ticks: u32,
}

impl McuCommand for QueryBmi160 {
    const NAME: &'static str = "query_bmi160";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// `query_bmi160_status oid=%c` — ask for the bulk channel's clock state; the
/// firmware answers with
/// [`SensorBulkStatus`](super::ldc1612::SensorBulkStatus)
/// (`FixedFreqReader.setup_query_command`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryBmi160Status {
    /// The sensor's object id.
    pub oid: u8,
}

impl McuCommand for QueryBmi160Status {
    const NAME: &'static str = "query_bmi160_status";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// The status query's name (`query_bmi160_status`), the command half of the
/// `msgformat` upstream's `setup_query_command` takes.
pub const QUERY_BMI160_STATUS: &str = "query_bmi160_status";

/// The status query's message format (`query_bmi160_status oid=%c`), passed to
/// the reader builder in `extras::bmi160`.
pub const QUERY_BMI160_STATUS_MSGFORMAT: &str = "query_bmi160_status oid=%c";

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

    /// `C::NAME` must be in the dictionary with exactly `params`, and `cmd`'s
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
            &ConfigBmi160 {
                oid: 1,
                bus_oid: 2,
                bus_oid_type: BUS_OID_TYPE_I2C,
            },
            &[
                ("oid", ArgType::UInt8),
                ("bus_oid", ArgType::UInt8),
                ("bus_oid_type", ArgType::UInt8),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &QueryBmi160 {
                oid: 4,
                rest_ticks: 5,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryBmi160Status { oid: 4 },
            &[("oid", ArgType::UInt8)],
        );
    }

    #[test]
    fn test_the_status_query_name_is_declared_by_the_firmware() {
        let dict = atmega2560();
        let mut parser = Parser::new();
        dict.install(&mut parser).unwrap();
        let msg = parser
            .lookup(QUERY_BMI160_STATUS)
            .unwrap_or_else(|| panic!("dictionary has no command '{QUERY_BMI160_STATUS}'"));
        let declared: Vec<(&str, ArgType)> = msg
            .params
            .iter()
            .map(|(name, atype)| (name.as_str(), *atype))
            .collect();
        assert_eq!(declared, [("oid", ArgType::UInt8)]);
        // The reader builder parses the command name out of this format.
        assert_eq!(
            QUERY_BMI160_STATUS_MSGFORMAT.split_whitespace().next(),
            Some(QUERY_BMI160_STATUS)
        );
    }
}
