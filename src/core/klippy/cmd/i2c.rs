//! I2C commands.
//!
//! Host view of the "I2C bus" section of the firmware's `i2ccmds.c`. An I2C
//! device is created once with `config_i2c` and then configured with either a
//! hardware bus (`i2c_set_bus`) or software GPIO pins (`i2c_set_sw_bus`).
//! After that, data is sent with `i2c_transfer` (legacy) or the split
//! `i2c_write` + `i2c_read` (new).
//!
//! # Two configuration paths
//!
//! | Path | Command |
//! |---|---|
//! | hardware | `i2c_set_bus oid=%c i2c_bus=%u rate=%u address=%u` |
//! | software | `i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u` |
//!
//! The firmware's `i2c_set_sw_bus` embeds the device address; the hardware
//! path leaves it for the transfer commands. Both paths converge on the same
//! transfer commands.
//!
//! # Two transfer styles
//!
//! Older firmware uses `i2c_transfer` / `i2c_response` (combined write + read).
//! Newer firmware splits them into `i2c_write` / `i2c_read` + matching
//! responses. The resource detects which at runtime via `try_lookup_command`.

use tracing::warn;

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::msg::proto::ArgValue;

// ===========================================================================
// Configuration commands
// ===========================================================================

/// `config_i2c oid=%c` — allocate an oid for an I2C device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigI2c {
    /// The object id the firmware allocated.
    pub oid: u8,
}

impl McuCommand for ConfigI2c {
    const NAME: &'static str = "config_i2c";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `i2c_set_bus oid=%c i2c_bus=%u rate=%u address=%u` — configure a hardware
/// I2C bus.
///
/// `i2c_bus` is the firmware enumeration's **value** for the bus. The firmware
/// declares it `%u`, so the name a config wrote is resolved by the pin layer
/// ([`PrinterPins::resolve_bus_value`](crate::core::klippy::pins::PrinterPins::resolve_bus_value))
/// — this port encodes config commands instead of sending text. `rate` is the
/// clock frequency in Hz. `address` is the 7-bit device address (shifted to
/// 8-bit by the firmware).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I2cSetBus {
    /// The I2C device oid.
    pub oid: u8,
    /// Hardware bus enumeration value (e.g. `1` for `"i2c_1"`).
    pub bus: u32,
    /// Clock frequency in Hz.
    pub rate: u32,
    /// 7-bit device address (0..=127).
    pub address: u8,
}

impl McuCommand for I2cSetBus {
    const NAME: &'static str = "i2c_set_bus";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.bus),
            ArgValue::UInt32(self.rate),
            ArgValue::UInt32(u32::from(self.address)),
        ]
    }
}

/// `i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u` —
/// configure software (bit-banged) I2C on GPIO pins.
///
/// `pulse_ticks` is the delay per bit in firmware clock ticks (half the I2C
/// period — one tick for SCL low, one for SCL high). `address` is embedded
/// here because the firmware stores it in the software I2C state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cSetSwBus {
    /// The I2C device oid.
    pub oid: u8,
    /// SCL pin number (firmware enumeration).
    pub scl_pin: u32,
    /// SDA pin number (firmware enumeration).
    pub sda_pin: u32,
    /// Pulse width in clock ticks (half the I2C period).
    pub pulse_ticks: u32,
    /// 7-bit device address (0..=127).
    pub address: u8,
}

impl McuCommand for I2cSetSwBus {
    const NAME: &'static str = "i2c_set_sw_bus";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.scl_pin),
            ArgValue::UInt32(self.sda_pin),
            ArgValue::UInt32(self.pulse_ticks),
            ArgValue::UInt32(u32::from(self.address)),
        ]
    }
}

// ===========================================================================
// Software-bus compatibility
// ===========================================================================

/// The newer software-bus command's full declaration, for
/// [`Mcu::try_lookup_command`].
const SET_SW_BUS: &str = "i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u";

/// The older `i2c_set_software_bus` that `i2c_set_sw_bus` replaced.
const SET_SOFTWARE_BUS: &str =
    "i2c_set_software_bus oid=%c scl_pin=%u sda_pin=%u rate=%u address=%u";

/// `i2c_set_software_bus oid=%c scl_pin=%u sda_pin=%u rate=%u address=%u` — the
/// software-bus command firmware from before upstream commit `a9b04e85`
/// (2025-04) uses.
///
/// It takes the wanted **rate** and quantizes it to a power-of-two delay in the
/// firmware (base 100 kHz); [`I2cSetSwBus`] replaced it with a host-computed
/// `pulse_ticks`. [`add_software_bus`] picks between them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cSetSoftwareBus {
    /// The I2C device oid.
    pub oid: u8,
    /// SCL pin number (firmware enumeration).
    pub scl_pin: u32,
    /// SDA pin number.
    pub sda_pin: u32,
    /// Clock frequency in Hz (the firmware quantizes it).
    pub rate: u32,
    /// 7-bit device address (0..=127).
    pub address: u8,
}

impl McuCommand for I2cSetSoftwareBus {
    const NAME: &'static str = "i2c_set_software_bus";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.scl_pin),
            ArgValue::UInt32(self.sda_pin),
            ArgValue::UInt32(self.rate),
            ArgValue::UInt32(u32::from(self.address)),
        ]
    }
}

/// One software (bit-banged) I2C bus, as [`add_software_bus`] needs it.
///
/// The pins are already resolved to firmware numbers; the caller gives the
/// wanted **speed in Hz** and does not choose a command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SoftwareI2cBus {
    /// The I2C device oid.
    pub oid: u8,
    /// SCL pin number (firmware enumeration).
    pub scl_pin: u32,
    /// SDA pin number.
    pub sda_pin: u32,
    /// Clock frequency in Hz.
    pub speed: u32,
    /// 7-bit device address (0..=127).
    pub address: u8,
}

/// Add the software-I2C bus config, in whichever spelling the firmware has.
///
/// `i2c_set_sw_bus` is preferred: it carries half the bit period in ticks, which
/// the host computes from the measured clock ([`Mcu::seconds_to_clock`]), so the
/// rate is exact. Firmware from before upstream commit `a9b04e85` (2025-04) has
/// only `i2c_set_software_bus`; that one goes out with a deprecation warning,
/// matching upstream's `deprecate_mcu_code`.
///
/// The caller passes pins and a speed and does not care which command is sent.
///
/// # Errors
/// Returns [`McuError::Config`] when the firmware has neither command (the bus
/// cannot be configured), or whatever the builder reports.
pub fn add_software_bus(
    builder: &ConfigBuilder,
    mcu: &Mcu,
    bus: &SoftwareI2cBus,
) -> Result<(), McuError> {
    if mcu.try_lookup_command(SET_SW_BUS).is_some() {
        let pulse_ticks = mcu.seconds_to_clock(1.0 / bus.speed as f64 / 2.0)? as u32;
        builder.add_config_cmd(&I2cSetSwBus {
            oid: bus.oid,
            scl_pin: bus.scl_pin,
            sda_pin: bus.sda_pin,
            pulse_ticks,
            address: bus.address,
        })
    } else if mcu.try_lookup_command(SET_SOFTWARE_BUS).is_some() {
        warn!(
            "MCU '{}' has deprecated code (it is missing feature 'i2c_set_sw_bus'). \
             Recompiling and flashing is recommended.",
            mcu.name()
        );
        builder.add_config_cmd(&I2cSetSoftwareBus {
            oid: bus.oid,
            scl_pin: bus.scl_pin,
            sda_pin: bus.sda_pin,
            rate: bus.speed,
            address: bus.address,
        })
    } else {
        Err(McuError::Config(
            "firmware has neither 'i2c_set_sw_bus' nor 'i2c_set_software_bus'".to_string(),
        ))
    }
}

// ===========================================================================
// Transfer commands (legacy: combined)
// ===========================================================================

/// `i2c_transfer oid=%c write=%*s read_len=%u` — write then read in one call.
///
/// This is the legacy combined transfer. The firmware sends back
/// `i2c_response` with the read data. The parameter is `write`, not `data`:
/// [`Mcu::try_lookup_command`](crate::core::klippy::mcu::Mcu::try_lookup_command)
/// matches the firmware's declaration exactly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I2cTransfer {
    /// The I2C device oid.
    pub oid: u8,
    /// Bytes to write, sent as the firmware's binary `%*s` parameter.
    pub write_data: Vec<u8>,
    /// Number of bytes to read back.
    pub read_len: u32,
}

impl McuCommand for I2cTransfer {
    const NAME: &'static str = "i2c_transfer";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.write_data.clone()),
            ArgValue::UInt32(self.read_len),
        ]
    }
}

/// `i2c_response oid=%c i2c_bus_status=%c response=%*s` — response to
/// `i2c_transfer`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I2cResponse {
    /// The I2C device oid.
    pub oid: u8,
    /// Bus status: `SUCCESS`, `NACK`, `START_NACK`, etc.
    pub bus_status: I2cBusStatus,
    /// Read data (empty if the transfer failed).
    pub response: Vec<u8>,
}

impl McuResponse for I2cResponse {
    const NAME: &'static str = "i2c_response";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        let oid = params.get_u8("oid")?;
        let status_name = params.get_enum("i2c_bus_status", "i2c_bus_status")?;
        let bus_status = match status_name.as_str() {
            "SUCCESS" => I2cBusStatus::Success,
            "NACK" => I2cBusStatus::Nack,
            "START_NACK" => I2cBusStatus::StartNack,
            "START_READ_NACK" => I2cBusStatus::StartReadNack,
            "BUS_TIMEOUT" => I2cBusStatus::BusTimeout,
            other => I2cBusStatus::Unknown(other.trim_start_matches('?').parse().unwrap_or(0)),
        };
        let response = params.get_bytes("response")?;
        Ok(Self {
            oid,
            bus_status,
            response,
        })
    }
}

// ===========================================================================
// Transfer commands (new: split write + read)
// ===========================================================================

/// `i2c_write oid=%c data=%*s` — write-only, no response expected.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I2cWrite {
    /// The I2C device oid.
    pub oid: u8,
    /// Bytes to write.
    pub data: Vec<u8>,
}

impl McuCommand for I2cWrite {
    const NAME: &'static str = "i2c_write";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.data.clone()),
        ]
    }
}

/// `i2c_read oid=%c reg=%*s read_len=%u` — read from a register address.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I2cRead {
    /// The I2C device oid.
    pub oid: u8,
    /// Register/address bytes to write before reading.
    pub reg: Vec<u8>,
    /// Number of bytes to read.
    pub read_len: u32,
}

impl McuCommand for I2cRead {
    const NAME: &'static str = "i2c_read";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.reg.clone()),
            ArgValue::UInt32(self.read_len),
        ]
    }
}

/// `i2c_read_response oid=%c response=%*s` — response to `i2c_read`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I2cReadResponse {
    /// The I2C device oid.
    pub oid: u8,
    /// Read data.
    pub response: Vec<u8>,
}

impl McuResponse for I2cReadResponse {
    const NAME: &'static str = "i2c_read_response";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        let oid = params.get_u8("oid")?;
        let response = params.get_bytes("response")?;
        Ok(Self { oid, response })
    }
}

// ===========================================================================
// I2C bus status enumeration
// ===========================================================================

/// I2C bus status codes returned in `i2c_response.i2c_bus_status`.
///
/// Matches the firmware's `DECL_ENUMERATION("i2c_bus_status", ...)` in
/// `i2ccmds.c`. Parsed from the string name returned by `Params::get_enum`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum I2cBusStatus {
    #[default]
    Success,
    Nack,
    StartNack,
    StartReadNack,
    BusTimeout,
    Unknown(u8),
}

impl I2cBusStatus {
    /// Whether the transfer succeeded without errors.
    pub fn is_ok(&self) -> bool {
        matches!(self, I2cBusStatus::Success)
    }

    /// Human-readable name for error reporting.
    pub fn name(&self) -> String {
        match self {
            I2cBusStatus::Success => "SUCCESS".to_string(),
            I2cBusStatus::Nack => "NACK".to_string(),
            I2cBusStatus::StartNack => "START_NACK".to_string(),
            I2cBusStatus::StartReadNack => "START_READ_NACK".to_string(),
            I2cBusStatus::BusTimeout => "BUS_TIMEOUT".to_string(),
            I2cBusStatus::Unknown(v) => format!("UNKNOWN({v})"),
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::test::TestDevice;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{BuiltConfig, Dictionary};
    use crate::core::klippy::msg::parser::Parser;
    use serde_json::json;
    use std::sync::Arc;

    fn parser() -> Parser {
        let dictionary = Dictionary::from_json(json!({
            "commands": {
                "config_i2c oid=%c": 40,
                "i2c_set_bus oid=%c i2c_bus=%u rate=%u address=%u": 41,
                "i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u": 42,
                "i2c_transfer oid=%c write=%*s read_len=%u": 43,
                "i2c_write oid=%c data=%*s": 44,
                "i2c_read oid=%c reg=%*s read_len=%u": 45
            },
            "responses": {
                "i2c_response oid=%c i2c_bus_status=%c response=%*s": 46,
                "i2c_read_response oid=%c response=%*s": 47
            },
            "enumerations": {
                "i2c_bus_status": {
                    "SUCCESS": 0,
                    "NACK": 1,
                    "START_NACK": 2,
                    "START_READ_NACK": 3,
                    "BUS_TIMEOUT": 4
                }
            }
        }))
        .unwrap();
        let mut parser = Parser::new();
        dictionary.install(&mut parser).unwrap();
        parser
    }

    /// A dictionary with the config-phase messages plus `commands`, so a
    /// wrapper's choice can be built and inspected.
    fn config_dictionary(commands: &[(&str, i64)]) -> Dictionary {
        let mut map = serde_json::Map::new();
        map.insert("allocate_oids count=%c".to_string(), json!(2));
        map.insert("finalize_config crc=%u".to_string(), json!(6));
        for (format, id) in commands {
            map.insert((*format).to_string(), json!(id));
        }
        Dictionary::from_json(json!({
            "commands": map,
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
    }

    /// An identified MCU that never talks to anything.
    fn mcu_with(dictionary: Dictionary) -> Mcu {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(Vec::new())));
        mcu.install_dictionary(dictionary).unwrap();
        mcu
    }

    /// The name and arguments of the config command at `index`.
    fn built_command(mcu: &Mcu, built: &BuiltConfig, index: usize) -> (String, Vec<ArgValue>) {
        let mut parser = Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        let frame = Frame::new(0, built.config[index].payload().to_vec());
        let decoded = parser.decode(frame.into()).unwrap();
        (decoded[0].0.name.clone(), decoded[0].1.clone())
    }

    #[test]
    fn test_config_i2c() {
        let command = ConfigI2c { oid: 1 };
        let encoded = parser().encode(ConfigI2c::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "config_i2c");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_i2c_set_bus() {
        let command = I2cSetBus {
            oid: 1,
            bus: 1,
            rate: 400_000,
            address: 0x68,
        };
        let encoded = parser().encode(I2cSetBus::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "i2c_set_bus");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_i2c_set_sw_bus() {
        let command = I2cSetSwBus {
            oid: 1,
            scl_pin: 5,
            sda_pin: 6,
            pulse_ticks: 5000,
            address: 0x68,
        };
        let encoded = parser().encode(I2cSetSwBus::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "i2c_set_sw_bus");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_i2c_set_software_bus_encode() {
        let command = I2cSetSoftwareBus {
            oid: 1,
            scl_pin: 5,
            sda_pin: 6,
            rate: 400_000,
            address: 0x68,
        };
        // The old command is declared only by old firmware.
        let parser = {
            let dictionary = config_dictionary(&[(SET_SOFTWARE_BUS, 71)]);
            let mut parser = Parser::new();
            dictionary.install(&mut parser).unwrap();
            parser
        };
        let encoded = parser
            .encode(I2cSetSoftwareBus::NAME, &command.args())
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "i2c_set_software_bus");
        assert_eq!(decoded[0].1, command.args());
    }

    #[tokio::test]
    async fn test_add_software_bus_prefers_the_new_command() {
        let mcu = mcu_with(config_dictionary(&[
            (SET_SW_BUS, 40),
            (SET_SOFTWARE_BUS, 41),
        ]));
        let builder = ConfigBuilder::new();

        add_software_bus(
            &builder,
            &mcu,
            &SoftwareI2cBus {
                oid: 1,
                scl_pin: 5,
                sda_pin: 6,
                speed: 400_000,
                address: 0x68,
            },
        )
        .unwrap();

        let built = builder.build(&mcu).unwrap();
        let (name, args) = built_command(&mcu, &built, 1);
        assert_eq!(name, "i2c_set_sw_bus");
        // `pulse_ticks` is `CLOCK_FREQ / speed / 2` = 20_000_000 / 400_000 / 2.
        assert_eq!(args[3], ArgValue::UInt32(25));
    }

    #[tokio::test]
    async fn test_add_software_bus_falls_back_to_the_old_command() {
        let mcu = mcu_with(config_dictionary(&[(SET_SOFTWARE_BUS, 41)]));
        let builder = ConfigBuilder::new();

        add_software_bus(
            &builder,
            &mcu,
            &SoftwareI2cBus {
                oid: 1,
                scl_pin: 5,
                sda_pin: 6,
                speed: 400_000,
                address: 0x68,
            },
        )
        .unwrap();

        let built = builder.build(&mcu).unwrap();
        let (name, args) = built_command(&mcu, &built, 1);
        assert_eq!(name, "i2c_set_software_bus");
        // The raw rate goes out; the firmware quantizes it.
        assert_eq!(args[3], ArgValue::UInt32(400_000));
    }

    #[tokio::test]
    async fn test_add_software_bus_without_either_command_is_an_error() {
        let mcu = mcu_with(config_dictionary(&[]));
        let builder = ConfigBuilder::new();

        let err = add_software_bus(
            &builder,
            &mcu,
            &SoftwareI2cBus {
                oid: 1,
                scl_pin: 5,
                sda_pin: 6,
                speed: 400_000,
                address: 0x68,
            },
        )
        .unwrap_err();

        assert!(err.to_string().contains("neither"), "{err}");
    }

    #[test]
    fn test_i2c_transfer() {
        let command = I2cTransfer {
            oid: 1,
            write_data: vec![0x01, 0xF4, 0x45],
            read_len: 4,
        };
        let encoded = parser().encode(I2cTransfer::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "i2c_transfer");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_i2c_write() {
        let command = I2cWrite {
            oid: 1,
            data: vec![0xAA, 0xBB],
        };
        let encoded = parser().encode(I2cWrite::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "i2c_write");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_i2c_read() {
        let command = I2cRead {
            oid: 1,
            reg: vec![0xD0],
            read_len: 6,
        };
        let encoded = parser().encode(I2cRead::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "i2c_read");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_i2c_response_decode() {
        let dictionary = Dictionary::from_json(json!({
            "enumerations": {
                "i2c_bus_status": {
                    "SUCCESS": 0,
                    "NACK": 1,
                    "START_NACK": 2,
                    "START_READ_NACK": 3,
                    "BUS_TIMEOUT": 4
                }
            }
        }))
        .unwrap();
        let encoded = parser()
            .encode(
                I2cResponse::NAME,
                &[
                    ArgValue::UInt8(1),
                    ArgValue::UInt8(1), // NACK (enumerations are wire-encoded as plain integers)
                    ArgValue::Bytes(vec![0xDE, 0xAD]),
                ],
            )
            .unwrap();
        let decoded = parser().decode(encoded).unwrap();
        let params =
            Params::new(decoded[0].0.clone(), &decoded[0].1).with_dictionary(Arc::new(dictionary));
        let response = I2cResponse::decode(&params).unwrap();
        assert_eq!(response.oid, 1);
        assert_eq!(response.bus_status, I2cBusStatus::Nack);
        assert_eq!(response.response, vec![0xDE, 0xAD]);
    }

    #[test]
    fn test_i2c_read_response_decode() {
        let parser = parser();
        let encoded = parser
            .encode(
                I2cReadResponse::NAME,
                &[ArgValue::UInt8(2), ArgValue::Bytes(vec![0x01, 0x02, 0x03])],
            )
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let response =
            I2cReadResponse::decode(&Params::new(decoded[0].0.clone(), &decoded[0].1)).unwrap();
        assert_eq!(response.oid, 2);
        assert_eq!(response.response, vec![0x01, 0x02, 0x03]);
    }

    #[test]
    fn test_i2c_bus_status() {
        assert!(I2cBusStatus::Success.is_ok());
        assert!(!I2cBusStatus::Nack.is_ok());
        assert_eq!(I2cBusStatus::Success.name(), "SUCCESS");
        assert_eq!(I2cBusStatus::Nack.name(), "NACK");
    }
}
