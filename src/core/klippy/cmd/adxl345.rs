//! ADXL345 commands: chip setup, the bulk stream arm/disarm, and the status
//! query (upstream's `klippy/extras/adxl345.py`).
//!
//! | command | fields | upstream |
//! |---|---|---|
//! | `config_adxl345` | `oid=%c spi_oid=%c` | `mcu.add_config_cmd(...)` in `ADXL345.__init__` |
//! | `query_adxl345` | `oid=%c rest_ticks=%u` | `ADXL345._start_measurements` / `_finish_measurements` |
//! | `query_adxl345_status` | `oid=%c` | `FixedFreqReader.setup_query_command` |
//!
//! The reply to `query_adxl345_status` is the shared
//! [`SensorBulkStatus`](super::ldc1612::SensorBulkStatus) message, not a name of
//! its own — every bulk sensor reports through the same channel
//! (`src/sensor_bulk.c`).
//!
//! Every name and parameter order here is checked against the corpus dictionary
//! (`atmega2560.dict`) by the tests at the bottom.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_adxl345 oid=%c spi_oid=%c` — create the sensor on an SPI device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigAdxl345 {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the SPI device the chip hangs off.
    pub spi_oid: u8,
}

impl McuCommand for ConfigAdxl345 {
    const NAME: &'static str = "config_adxl345";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.spi_oid)]
    }
}

/// `query_adxl345 oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`ADXL345._start_measurements` /
/// `_finish_measurements`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAdxl345 {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms the sensor.
    pub rest_ticks: u32,
}

impl McuCommand for QueryAdxl345 {
    const NAME: &'static str = "query_adxl345";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// `query_adxl345_status oid=%c` — ask for the bulk channel's clock state; the
/// firmware answers with
/// [`SensorBulkStatus`](super::ldc1612::SensorBulkStatus)
/// (`FixedFreqReader.setup_query_command`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryAdxl345Status {
    /// The sensor's object id.
    pub oid: u8,
}

impl McuCommand for QueryAdxl345Status {
    const NAME: &'static str = "query_adxl345_status";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
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
            &ConfigAdxl345 { oid: 1, spi_oid: 2 },
            &[("oid", ArgType::UInt8), ("spi_oid", ArgType::UInt8)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryAdxl345 {
                oid: 4,
                rest_ticks: 5,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryAdxl345Status { oid: 4 },
            &[("oid", ArgType::UInt8)],
        );
    }
}
