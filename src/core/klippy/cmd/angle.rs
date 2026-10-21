//! Angle sensor commands: chip setup and the bulk stream arm/disarm
//! (upstream's `klippy/extras/angle.py` with `src/sensor_angle.c`).
//!
//! | command | fields | upstream |
//! |---|---|---|
//! | `config_spi_angle` | `oid=%c spi_oid=%c spi_angle_type=%c` | `Angle.__init__`'s `mcu.add_config_cmd` |
//! | `query_spi_angle` | `oid=%c clock=%u rest_ticks=%u time_shift=%c` | `_start_measurements` / `_finish_measurements` |
//!
//! The chip selector travels as the `spi_angle_type` enumeration's ordinal
//! (`src/sensor_angle.c`: `a1333`, `as5047d`, `tle5012b`, `mt6816`, `mt6826s`).
//! Upstream arms nothing at config time: `query_spi_angle` is added with
//! `clock=0 rest_ticks=0 time_shift=0` `on_restart`, so a firmware restart
//! leaves the sensor disarmed.
//!
//! Every name and parameter order here is checked against the corpus dictionary
//! (`atmega2560.dict`) by the tests at the bottom.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_spi_angle oid=%c spi_oid=%c spi_angle_type=%c` — create the sensor
/// on an SPI device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigSpiAngle {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the SPI device the chip hangs off.
    pub spi_oid: u8,
    /// The `spi_angle_type` ordinal — the firmware picks its query routine by
    /// it, so the host never speaks the chip's protocol for the bulk stream.
    pub chip_type: u8,
}

impl McuCommand for ConfigSpiAngle {
    const NAME: &'static str = "config_spi_angle";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.spi_oid),
            ArgValue::UInt8(self.chip_type),
        ]
    }
}

/// `query_spi_angle oid=%c clock=%u rest_ticks=%u time_shift=%c` — arm the
/// periodic query at `clock`, or disarm it with `rest_ticks=0`
/// (`Angle._start_measurements` / `_finish_measurements`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuerySpiAngle {
    /// The sensor's object id.
    pub oid: u8,
    /// The firmware clock the periodic sampling starts at; `0` disarms.
    pub clock: u32,
    /// Firmware ticks between samples.
    pub rest_ticks: u32,
    /// How many bits of the sample's time-code are its clock offset.
    pub time_shift: u8,
}

impl McuCommand for QuerySpiAngle {
    const NAME: &'static str = "query_spi_angle";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.rest_ticks),
            ArgValue::UInt8(self.time_shift),
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
            &ConfigSpiAngle {
                oid: 1,
                spi_oid: 2,
                chip_type: 1,
            },
            &[
                ("oid", ArgType::UInt8),
                ("spi_oid", ArgType::UInt8),
                ("spi_angle_type", ArgType::UInt8),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &QuerySpiAngle {
                oid: 4,
                clock: 5,
                rest_ticks: 6,
                time_shift: 3,
            },
            &[
                ("oid", ArgType::UInt8),
                ("clock", ArgType::UInt32),
                ("rest_ticks", ArgType::UInt32),
                ("time_shift", ArgType::UInt8),
            ],
        );
    }

    #[test]
    fn test_the_chip_type_ordinals_are_the_firmwares_enumeration() {
        // `src/sensor_angle.c`'s `SA_CHIP_*` order, which is what the
        // `spi_angle_type` enumeration sends.
        let dict = atmega2560();
        let enumeration = dict
            .enumeration("spi_angle_type")
            .expect("the dictionary declares spi_angle_type");
        let names: Vec<&str> = (0..5)
            .map(|value| enumeration.name(value).expect("a declared chip"))
            .collect();
        assert_eq!(names, ["a1333", "as5047d", "tle5012b", "mt6816", "mt6826s"]);
    }
}
