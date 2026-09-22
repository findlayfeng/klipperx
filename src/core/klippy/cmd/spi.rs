//! SPI commands.
//!
//! Host view of the "SPI bus" section of the firmware's `spicmds.c`. A device
//! is created once with `config_spi` (which also allocates its chip-select pin)
//! or `config_spi_without_cs`, then configured with either a hardware bus
//! (`spi_set_bus`) or software GPIO pins (`spi_set_sw_bus`). After that, data
//! moves with `spi_send` (write only) or `spi_transfer` (full duplex).
//!
//! # Chip select
//!
//! Unlike I2C, the firmware drives the chip-select pin itself: `config_spi`
//! installs it, and every transfer asserts it, shifts the bytes, and releases
//! it. A device with no dedicated CS pin (`cs_pin: None`) uses
//! `config_spi_without_cs` and drives the pin from the host side.
//!
//! # Two configuration paths
//!
//! | Path | Command |
//! |---|---|
//! | hardware | `spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u` |
//! | software | `spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u` |
//!
//! Both the `mode` (0..=3, CPOL/CPHA) and, for software, `pulse_ticks` matter:
//! every device on the bus must agree on the mode, and the bit period is half
//! the SPI clock period in firmware ticks.

use tracing::warn;

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::msg::proto::ArgValue;

// ===========================================================================
// Configuration commands
// ===========================================================================

/// `config_spi oid=%c pin=%u cs_active_high=%c` — allocate an oid and the
/// chip-select pin the firmware drives around each transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigSpi {
    /// The SPI device oid.
    pub oid: u8,
    /// Chip-select pin number (firmware enumeration).
    pub pin: u32,
    /// Whether the CS pin is active high (default low).
    pub cs_active_high: bool,
}

impl McuCommand for ConfigSpi {
    const NAME: &'static str = "config_spi";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.pin),
            ArgValue::UInt8(u8::from(self.cs_active_high)),
        ]
    }
}

/// `config_spi_without_cs oid=%c` — a device whose CS is driven from the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigSpiWithoutCs {
    /// The SPI device oid.
    pub oid: u8,
}

impl McuCommand for ConfigSpiWithoutCs {
    const NAME: &'static str = "config_spi_without_cs";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u` — configure a hardware SPI
/// bus.
///
/// `spi_bus` is the firmware enumeration's value for the bus (resolved from the
/// config's name by the pin layer). `mode` is the SPI mode, 0..=3. `rate` is
/// the clock frequency in Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiSetBus {
    /// The SPI device oid.
    pub oid: u8,
    /// Hardware bus enumeration value.
    pub bus: u32,
    /// SPI mode (CPOL/CPHA), 0..=3.
    pub mode: u8,
    /// Clock frequency in Hz.
    pub rate: u32,
}

impl McuCommand for SpiSetBus {
    const NAME: &'static str = "spi_set_bus";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.bus),
            ArgValue::UInt32(u32::from(self.mode)),
            ArgValue::UInt32(self.rate),
        ]
    }
}

/// `spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u
/// pulse_ticks=%u` — configure software (bit-banged) SPI on GPIO pins.
///
/// `pulse_ticks` is half the SPI clock period in firmware ticks (the firmware
/// toggles SCLK twice per bit). The three pins must be on the same MCU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiSetSwBus {
    /// The SPI device oid.
    pub oid: u8,
    /// MISO pin number (firmware enumeration).
    pub miso_pin: u32,
    /// MOSI pin number.
    pub mosi_pin: u32,
    /// SCLK pin number.
    pub sclk_pin: u32,
    /// SPI mode (CPOL/CPHA), 0..=3.
    pub mode: u8,
    /// Bit period in clock ticks (half the SPI clock period).
    pub pulse_ticks: u32,
}

impl McuCommand for SpiSetSwBus {
    const NAME: &'static str = "spi_set_sw_bus";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.miso_pin),
            ArgValue::UInt32(self.mosi_pin),
            ArgValue::UInt32(self.sclk_pin),
            ArgValue::UInt32(u32::from(self.mode)),
            ArgValue::UInt32(self.pulse_ticks),
        ]
    }
}

// ===========================================================================
// Software-bus compatibility
// ===========================================================================

/// The newer software-bus command's full declaration, for
/// [`Mcu::try_lookup_command`].
const SET_SW_BUS: &str =
    "spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u";

/// The older `spi_set_software_bus` that `spi_set_sw_bus` replaced.
const SET_SOFTWARE_BUS: &str =
    "spi_set_software_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u rate=%u";

/// `spi_set_software_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u rate=%u`
/// — the software-bus command firmware from before upstream commit `abc76ee9`
/// (2025-03) uses.
///
/// It takes the wanted **rate** and quantizes it to a power-of-two divider in
/// the firmware; [`SpiSetSwBus`] replaced it with a host-computed `pulse_ticks`.
/// [`add_software_bus`] picks between them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiSetSoftwareBus {
    /// The SPI device oid.
    pub oid: u8,
    /// MISO pin number (firmware enumeration).
    pub miso_pin: u32,
    /// MOSI pin number.
    pub mosi_pin: u32,
    /// SCLK pin number.
    pub sclk_pin: u32,
    /// SPI mode (CPOL/CPHA), 0..=3.
    pub mode: u8,
    /// Clock frequency in Hz (the firmware quantizes it).
    pub rate: u32,
}

impl McuCommand for SpiSetSoftwareBus {
    const NAME: &'static str = "spi_set_software_bus";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.miso_pin),
            ArgValue::UInt32(self.mosi_pin),
            ArgValue::UInt32(self.sclk_pin),
            ArgValue::UInt32(u32::from(self.mode)),
            ArgValue::UInt32(self.rate),
        ]
    }
}

/// One software (bit-banged) SPI bus, as [`add_software_bus`] needs it.
///
/// The pins are already resolved to firmware numbers; the caller gives the
/// wanted **speed in Hz** and does not choose a command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SoftwareSpiBus {
    /// The SPI device oid.
    pub oid: u8,
    /// MISO pin number (firmware enumeration).
    pub miso_pin: u32,
    /// MOSI pin number.
    pub mosi_pin: u32,
    /// SCLK pin number.
    pub sclk_pin: u32,
    /// SPI mode (CPOL/CPHA), 0..=3.
    pub mode: u8,
    /// Clock frequency in Hz.
    pub speed: u32,
}

/// Add the software-SPI bus config, in whichever spelling the firmware has.
///
/// `spi_set_sw_bus` is preferred: it carries the bit period in ticks, which the
/// host computes from the measured clock ([`Mcu::seconds_to_clock`]), so the
/// rate is exact. Firmware from before upstream commit `abc76ee9` (2025-03) has
/// only `spi_set_software_bus`; that one goes out with a deprecation warning,
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
    bus: &SoftwareSpiBus,
) -> Result<(), McuError> {
    if mcu.try_lookup_command(SET_SW_BUS).is_some() {
        let pulse_ticks = mcu.seconds_to_clock(1.0 / bus.speed as f64)? as u32;
        builder.add_config_cmd(&SpiSetSwBus {
            oid: bus.oid,
            miso_pin: bus.miso_pin,
            mosi_pin: bus.mosi_pin,
            sclk_pin: bus.sclk_pin,
            mode: bus.mode,
            pulse_ticks,
        })
    } else if mcu.try_lookup_command(SET_SOFTWARE_BUS).is_some() {
        warn!(
            "MCU '{}' has deprecated code (it is missing feature 'spi_set_sw_bus'). \
             Recompiling and flashing is recommended.",
            mcu.name()
        );
        builder.add_config_cmd(&SpiSetSoftwareBus {
            oid: bus.oid,
            miso_pin: bus.miso_pin,
            mosi_pin: bus.mosi_pin,
            sclk_pin: bus.sclk_pin,
            mode: bus.mode,
            rate: bus.speed,
        })
    } else {
        Err(McuError::Config(
            "firmware has neither 'spi_set_sw_bus' nor 'spi_set_software_bus'".to_string(),
        ))
    }
}

// ===========================================================================
// Transfer commands
// ===========================================================================

/// `spi_send oid=%c data=%*s` — shift bytes out, ignore what comes back.
///
/// The firmware sends no response for this command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpiSend {
    /// The SPI device oid.
    pub oid: u8,
    /// Bytes to send, as the firmware's binary `%*s` parameter.
    pub data: Vec<u8>,
}

impl McuCommand for SpiSend {
    const NAME: &'static str = "spi_send";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.data.clone()),
        ]
    }
}

/// `spi_transfer oid=%c data=%*s` — full-duplex shift; the response carries the
/// bytes clocked in, the same length as `data`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpiTransfer {
    /// The SPI device oid.
    pub oid: u8,
    /// Bytes to send (the return bytes arrive in `spi_transfer_response`).
    pub data: Vec<u8>,
}

impl McuCommand for SpiTransfer {
    const NAME: &'static str = "spi_transfer";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.data.clone()),
        ]
    }
}

/// `spi_transfer_response oid=%c response=%*s` — response to `spi_transfer`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpiTransferResponse {
    /// The SPI device oid.
    pub oid: u8,
    /// Bytes clocked in, the same length as the transfer's `data`.
    pub response: Vec<u8>,
}

impl McuResponse for SpiTransferResponse {
    const NAME: &'static str = "spi_transfer_response";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            response: params.get_bytes("response")?,
        })
    }
}

// ===========================================================================
// Shutdown handling
// ===========================================================================

/// `config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s` — register a message
/// the firmware shifts out on the bus when it shuts down.
///
/// A driver that must disable its hardware on shutdown (a stepper driver, say)
/// sends it here instead of racing the host. The firmware asserts the device's
/// CS and clocks `shutdown_msg` out during its shutdown sequence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigSpiShutdown {
    /// The shutdown handler's own oid.
    pub oid: u8,
    /// The SPI device whose CS the message is sent on.
    pub spi_oid: u8,
    /// Bytes to clock out on shutdown.
    pub shutdown_msg: Vec<u8>,
}

impl McuCommand for ConfigSpiShutdown {
    const NAME: &'static str = "config_spi_shutdown";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.spi_oid),
            ArgValue::Bytes(self.shutdown_msg.clone()),
        ]
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{BuiltConfig, Dictionary};
    use crate::core::klippy::msg::parser::Parser;
    use serde_json::json;

    fn parser() -> Parser {
        let dictionary = Dictionary::from_json(json!({
            "commands": {
                "config_spi oid=%c pin=%u cs_active_high=%c": 50,
                "config_spi_without_cs oid=%c": 51,
                "spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u": 52,
                "spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u": 53,
                "spi_send oid=%c data=%*s": 54,
                "spi_transfer oid=%c data=%*s": 55
            },
            "responses": {
                "spi_transfer_response oid=%c response=%*s": 56
            }
        }))
        .unwrap();
        let mut parser = Parser::new();
        dictionary.install(&mut parser).unwrap();
        parser
    }

    /// Encode then decode a command, returning what came back.
    fn roundtrip<C: McuCommand>(command: &C) -> Vec<ArgValue> {
        let parser = parser();
        let encoded = parser.encode(C::NAME, &command.args()).unwrap();
        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, C::NAME);
        decoded[0].1.clone()
    }

    #[test]
    fn test_config_spi() {
        let command = ConfigSpi {
            oid: 1,
            pin: 15,
            cs_active_high: false,
        };
        assert_eq!(roundtrip(&command), command.args());
    }

    #[test]
    fn test_config_spi_active_high() {
        let command = ConfigSpi {
            oid: 1,
            pin: 15,
            cs_active_high: true,
        };
        assert_eq!(roundtrip(&command), command.args());
    }

    #[test]
    fn test_config_spi_without_cs() {
        let command = ConfigSpiWithoutCs { oid: 2 };
        assert_eq!(roundtrip(&command), command.args());
    }

    #[test]
    fn test_spi_set_bus() {
        let command = SpiSetBus {
            oid: 1,
            bus: 2,
            mode: 0,
            rate: 1_000_000,
        };
        assert_eq!(roundtrip(&command), command.args());
    }

    #[test]
    fn test_spi_set_sw_bus() {
        let command = SpiSetSwBus {
            oid: 1,
            miso_pin: 4,
            mosi_pin: 5,
            sclk_pin: 3,
            mode: 3,
            pulse_ticks: 720,
        };
        assert_eq!(roundtrip(&command), command.args());
    }

    #[test]
    fn test_spi_set_software_bus_encode() {
        let command = SpiSetSoftwareBus {
            oid: 1,
            miso_pin: 4,
            mosi_pin: 5,
            sclk_pin: 3,
            mode: 0,
            rate: 1_000_000,
        };
        // The old command is declared only by old firmware.
        let parser = {
            let dictionary = config_dictionary(&[(SET_SOFTWARE_BUS, 70)]);
            let mut parser = Parser::new();
            dictionary.install(&mut parser).unwrap();
            parser
        };
        let encoded = parser
            .encode(SpiSetSoftwareBus::NAME, &command.args())
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "spi_set_software_bus");
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
            &SoftwareSpiBus {
                oid: 1,
                miso_pin: 4,
                mosi_pin: 5,
                sclk_pin: 3,
                mode: 0,
                speed: 1_000_000,
            },
        )
        .unwrap();

        let built = builder.build(&mcu).unwrap();
        let (name, args) = built_command(&mcu, &built, 1);
        assert_eq!(name, "spi_set_sw_bus");
        // `pulse_ticks` is `CLOCK_FREQ / speed` = 20_000_000 / 1_000_000.
        assert_eq!(args.last(), Some(&ArgValue::UInt32(20)));
    }

    #[tokio::test]
    async fn test_add_software_bus_falls_back_to_the_old_command() {
        let mcu = mcu_with(config_dictionary(&[(SET_SOFTWARE_BUS, 41)]));
        let builder = ConfigBuilder::new();

        add_software_bus(
            &builder,
            &mcu,
            &SoftwareSpiBus {
                oid: 1,
                miso_pin: 4,
                mosi_pin: 5,
                sclk_pin: 3,
                mode: 0,
                speed: 1_000_000,
            },
        )
        .unwrap();

        let built = builder.build(&mcu).unwrap();
        let (name, args) = built_command(&mcu, &built, 1);
        assert_eq!(name, "spi_set_software_bus");
        // The raw rate goes out; the firmware quantizes it.
        assert_eq!(args.last(), Some(&ArgValue::UInt32(1_000_000)));
    }

    #[tokio::test]
    async fn test_add_software_bus_without_either_command_is_an_error() {
        let mcu = mcu_with(config_dictionary(&[]));
        let builder = ConfigBuilder::new();

        let err = add_software_bus(
            &builder,
            &mcu,
            &SoftwareSpiBus {
                oid: 1,
                miso_pin: 4,
                mosi_pin: 5,
                sclk_pin: 3,
                mode: 0,
                speed: 1_000_000,
            },
        )
        .unwrap_err();

        assert!(err.to_string().contains("neither"), "{err}");
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
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(Vec::new())));
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
    fn test_spi_send() {
        let command = SpiSend {
            oid: 1,
            data: vec![0x06],
        };
        assert_eq!(roundtrip(&command), command.args());
    }

    #[test]
    fn test_spi_transfer() {
        // A JEDEC-ID read: the command plus three dummy bytes; the high byte
        // proves the parameter is binary, not text.
        let command = SpiTransfer {
            oid: 1,
            data: vec![0x9f, 0x00, 0x00, 0x00],
        };
        assert_eq!(roundtrip(&command), command.args());

        let command = SpiTransfer {
            oid: 1,
            data: vec![0x03, 0x00, 0x00, 0x00, 0xf4],
        };
        assert_eq!(roundtrip(&command), command.args());
    }

    #[test]
    fn test_spi_transfer_response_decode() {
        let parser = parser();
        let encoded = parser
            .encode(
                SpiTransferResponse::NAME,
                &[
                    ArgValue::UInt8(1),
                    ArgValue::Bytes(vec![0x00, 0xef, 0x40, 0x17]),
                ],
            )
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let response =
            SpiTransferResponse::decode(&Params::new(decoded[0].0.clone(), &decoded[0].1)).unwrap();
        assert_eq!(response.oid, 1);
        assert_eq!(response.response, vec![0x00, 0xef, 0x40, 0x17]);
    }

    #[test]
    fn test_config_spi_shutdown() {
        let command = ConfigSpiShutdown {
            oid: 9,
            spi_oid: 1,
            shutdown_msg: vec![0x80, 0xff],
        };
        // `config_spi_shutdown` is only declared by some firmwares; encode it
        // against a dictionary that has it.
        let parser = {
            let dictionary = Dictionary::from_json(json!({
                "commands": {
                    "config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s": 60
                }
            }))
            .unwrap();
            let mut parser = Parser::new();
            dictionary.install(&mut parser).unwrap();
            parser
        };
        let encoded = parser
            .encode(ConfigSpiShutdown::NAME, &command.args())
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "config_spi_shutdown");
        assert_eq!(decoded[0].1, command.args());
    }
}
