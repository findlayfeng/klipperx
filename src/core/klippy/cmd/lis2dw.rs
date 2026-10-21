//! LIS2DW / LIS3DH commands: chip creation, the bulk stream arm/disarm, and the
//! status query (upstream's `klippy/extras/lis2dw.py`, whose `[lis3dh]` section
//! reuses the same driver).
//!
//! | command | fields | upstream |
//! |---|---|---|
//! | `config_lis2dw` | `oid=%c bus_oid=%c bus_oid_type=%c lis_chip_type=%c` | `mcu.add_config_cmd(...)` in `LIS2DW.__init__` |
//! | `query_lis2dw` | `oid=%c rest_ticks=%u` | `_start_measurements` / `_finish_measurements` |
//! | `query_lis2dw_status` | `oid=%c` | `FixedFreqReader.setup_query_command` |
//!
//! The chip is created on whichever bus the section picked: `bus_oid` is either
//! an SPI or an I2C device oid, and `bus_oid_type` says which (`src/sensor_lis2dw.c`
//! looks the oid up as the type names). `lis_chip_type` selects the register
//! set and the FIFO status layout — the one place upstream's two section names
//! differ.
//!
//! The reply to `query_lis2dw_status` is the shared
//! [`SensorBulkStatus`](super::ldc1612::SensorBulkStatus) message, not a name of
//! its own — every bulk sensor reports through the same channel
//! (`src/sensor_bulk.c`).
//!
//! Every name, parameter order, and enumeration value here is checked against
//! the corpus dictionary (`atmega2560.dict`) by the tests at the bottom.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// The bus `config_lis2dw`'s `bus_oid` belongs to
/// (`DECL_ENUMERATION("bus_oid_type", …)` in `src/sensor_lis2dw.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LisBusType {
    /// An SPI device oid.
    Spi,
    /// An I2C device oid.
    I2c,
}

impl LisBusType {
    /// The firmware's value for this bus (`SPI_SERIAL` / `I2C_SERIAL`).
    pub fn code(self) -> u8 {
        match self {
            Self::Spi => 0,
            Self::I2c => 1,
        }
    }

    /// The name upstream logs and passes as `bus_oid_type`
    /// (`SPI_SERIAL_TYPE` / `I2C_SERIAL_TYPE`).
    pub fn name(self) -> &'static str {
        match self {
            Self::Spi => "spi",
            Self::I2c => "i2c",
        }
    }
}

/// Which chip the driver is talking to (`DECL_ENUMERATION("lis_chip_type", …)`).
///
/// Upstream selects one of these per section: `[lis2dw]` builds
/// `LIS2DW.__init__` with `LIS2DW_TYPE`, `[lis3dh]` with `LIS3DH_TYPE`
/// (`lis3dh.py`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LisChipType {
    /// The LIS2DW (`LIS2DW_TYPE`).
    Lis2dw,
    /// The LIS3DH (`LIS3DH_TYPE`).
    Lis3dh,
}

impl LisChipType {
    /// The firmware's value for this chip (`LIS2DW` / `LIS3DH`).
    pub fn code(self) -> u8 {
        match self {
            Self::Lis2dw => 0,
            Self::Lis3dh => 1,
        }
    }

    /// The section name that selects this chip, and the name upstream logs
    /// (`lis2dw` / `lis3dh`).
    pub fn name(self) -> &'static str {
        match self {
            Self::Lis2dw => "lis2dw",
            Self::Lis3dh => "lis3dh",
        }
    }
}

/// `config_lis2dw oid=%c bus_oid=%c bus_oid_type=%c lis_chip_type=%c` — create
/// the sensor on an SPI or I2C device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigLis2dw {
    /// The sensor's object id.
    pub oid: u8,
    /// The oid of the bus device the chip hangs off.
    pub bus_oid: u8,
    /// Which kind of device `bus_oid` names.
    pub bus_type: LisBusType,
    /// Which chip's register set the firmware should use.
    pub chip_type: LisChipType,
}

impl McuCommand for ConfigLis2dw {
    const NAME: &'static str = "config_lis2dw";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.bus_oid),
            ArgValue::UInt8(self.bus_type.code()),
            ArgValue::UInt8(self.chip_type.code()),
        ]
    }
}

/// `query_lis2dw oid=%c rest_ticks=%u` — start (nonzero) or stop (zero) the
/// periodic bulk reports (`LIS2DW._start_measurements` / `_finish_measurements`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryLis2dw {
    /// The sensor's object id.
    pub oid: u8,
    /// Firmware ticks between reports; `0` disarms the sensor.
    pub rest_ticks: u32,
}

impl McuCommand for QueryLis2dw {
    const NAME: &'static str = "query_lis2dw";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.rest_ticks)]
    }
}

/// `query_lis2dw_status oid=%c` — ask for the bulk channel's clock state; the
/// firmware answers with
/// [`SensorBulkStatus`](super::ldc1612::SensorBulkStatus)
/// (`FixedFreqReader.setup_query_command`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryLis2dwStatus {
    /// The sensor's object id.
    pub oid: u8,
}

impl McuCommand for QueryLis2dwStatus {
    const NAME: &'static str = "query_lis2dw_status";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// The status query's message format (`query_lis2dw_status oid=%c`), the
/// `msgformat` half of upstream's `setup_query_command`.
pub const QUERY_LIS2DW_STATUS_MSGFORMAT: &str = "query_lis2dw_status oid=%c";

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
            &ConfigLis2dw {
                oid: 1,
                bus_oid: 2,
                bus_type: LisBusType::Spi,
                chip_type: LisChipType::Lis2dw,
            },
            &[
                ("oid", ArgType::UInt8),
                ("bus_oid", ArgType::UInt8),
                ("bus_oid_type", ArgType::UInt8),
                ("lis_chip_type", ArgType::UInt8),
            ],
        );
        assert_matches_dictionary(
            &dict,
            &QueryLis2dw {
                oid: 4,
                rest_ticks: 5,
            },
            &[("oid", ArgType::UInt8), ("rest_ticks", ArgType::UInt32)],
        );
        assert_matches_dictionary(
            &dict,
            &QueryLis2dwStatus { oid: 4 },
            &[("oid", ArgType::UInt8)],
        );
    }

    #[test]
    fn test_the_enumeration_values_match_the_firmware() {
        // `DECL_ENUMERATION("bus_oid_type", "spi", SPI_SERIAL)` /
        // `("i2c", I2C_SERIAL)`, and `("lis_chip_type", "LIS2DW", LIS2DW)` /
        // `("LIS3DH", LIS3DH)` (src/sensor_lis2dw.c).
        let dict = atmega2560();
        let bus = dict
            .enumeration("bus_oid_type")
            .expect("the dictionary has bus_oid_type");
        assert_eq!(bus.value("spi"), Some(i64::from(LisBusType::Spi.code())));
        assert_eq!(bus.value("i2c"), Some(i64::from(LisBusType::I2c.code())));
        let chip = dict
            .enumeration("lis_chip_type")
            .expect("the dictionary has lis_chip_type");
        assert_eq!(
            chip.value("LIS2DW"),
            Some(i64::from(LisChipType::Lis2dw.code()))
        );
        assert_eq!(
            chip.value("LIS3DH"),
            Some(i64::from(LisChipType::Lis3dh.code()))
        );
    }
}
