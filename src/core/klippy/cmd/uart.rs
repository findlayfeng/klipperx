//! Port streams — the byte streams a host hands to a bridge as its ports.
//!
//! The bridge has no compile-time port list: which byte streams it owns is a
//! configuration fact. The host creates one stream object per downstream port at
//! configuration time, and hands it to the bridge in the same pass:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_uart oid=%c baud=%u slot=%c` |
//! | host → MCU | `config_can oid=%c bitrate=%u slot=%c` |
//! | host → MCU | `config_bridge_port oid=%c port=%c` |
//!
//! Which of the two `config_*` commands creates the stream depends on the bus
//! the port is (`mcu/bridge_ports.rs` for a port the config declares,
//! [`PortConfig`] on the `flash` side): `config_uart` for a serial port, at
//! `baud`, `config_can` for a CAN bus port, at `bitrate`. Either way the command creates
//! the object and opens the bus behind it (the firmware's `src/uart.c`, and the
//! CAN object's own module); a stream it cannot open, or a rate it cannot set,
//! shuts the firmware down with a message naming why, so a port that cannot be
//! brought up fails the configuration instead of the bridge coming up without
//! it. `config_uart`'s `slot` is the port's **UART slot**: which UART of the
//! bridge it is, which is also the slot the host named that port's bus with
//! before the library came up (`Interface::host_mux`) — so the firmware opens
//! the port the host meant, not a path baked into the build. It is not the
//! port's index (that counts every port, CAN ones included): the firmware picks
//! a USART by this slot alone (`src/stm32/bridge_port.c`). `config_can`'s slot is
//! the index, a placeholder the firmware ignores — a CAN port names no slot. The
//! `config_bridge_port` command that follows hands each object to the router as
//! a downstream port: the bridge numbers ports by the order these
//! commands arrive, and the `index` of the port-level `SET_PORT_NODEID`
//! (`interface/devices::serial::set_port_node_id`) is that number, so the host
//! queues the pairs in the order the port sections are declared
//! (`mcu/bridge_ports.rs`).
//!
//! Neither command has a response of its own: the firmware's acceptance is the
//! whole answer, and a refusal is a shutdown, not a reply.

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// The line rate a port gets when the config writes no `uart_baud:`.
///
/// The same 250000 every serial link in klipper runs at, and the value the
/// removed `BRIDGE_PORT_SERIAL_BAUD` Kconfig option defaulted to, so a config
/// that never wrote a rate keeps the line it had.
pub const DEFAULT_UART_BAUD: u32 = 250_000;

/// The bus rate a port gets when the config writes no `can_bitrate:`.
///
/// 1 Mbit/s: the rate the CAN plans in this repository write for a bridge's CAN
/// slot (`flash/bridge_session.rs`), so a config that never wrote a rate keeps
/// the bus it had.
pub const DEFAULT_CAN_BITRATE: u32 = 1_000_000;

/// `config_uart oid=%c baud=%u slot=%c` — create a UART object and open its
/// stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigUart {
    /// The object id the firmware allocates for this UART.
    pub oid: u8,
    /// The line rate the stream is opened at.
    pub baud: u32,
    /// The UART slot the stream belongs to (`third_party/klipper/src/port.c`,
    /// `src/stm32/bridge_port.c`): which UART of the bridge this port is
    /// (`mcu/bridge_ports.rs`), **not** the port's index — the index counts
    /// every port on the bridge, CAN ones included, while the firmware picks a
    /// USART by this number alone. It is also the number the host named that
    /// port's tty with (`klipper_host_port`, before the library comes up). The
    /// firmware opens the tty registered for this slot, or the build's default
    /// when the host named none.
    pub slot: u8,
}

impl McuCommand for ConfigUart {
    const NAME: &'static str = "config_uart";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.baud),
            ArgValue::UInt8(self.slot),
        ]
    }
}

/// `config_can oid=%c bitrate=%u slot=%c` — create a CAN bus object and open it
/// as a downstream port's stream.
///
/// The CAN twin of [`ConfigUart`]: same three parameters, `bitrate` in place of
/// the line rate. Two callers queue it: [`crate::core::klippy::mcu::bridge_ports`]
/// for a `[can_device]` port at load, and [`PortConfig`] on the `flash` side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigCan {
    /// The object id the firmware allocates for this CAN bus object.
    pub oid: u8,
    /// The bus rate the object is opened at, in bits per second.
    pub bitrate: u32,
    /// The port slot the object belongs to. A CAN port names no slot: the
    /// firmware opens the build's one CAN controller and ignores this field
    /// (`src/stm32/bridge_port.c`'s `port_backend_can_open`), so what is sent is
    /// the port's **index** as a placeholder. [`ConfigUart`]'s slot is a real
    /// number by contrast — the port's UART slot.
    pub slot: u8,
}

impl McuCommand for ConfigCan {
    const NAME: &'static str = "config_can";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.bitrate),
            ArgValue::UInt8(self.slot),
        ]
    }
}

/// `config_bridge_port oid=%c port=%c` — hand a port's stream object to the
/// bridge as a downstream port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigBridgePort {
    /// The object id of the port itself.
    pub oid: u8,
    /// The oid of the stream object the port carries — the UART or CAN bus
    /// object created by [`ConfigUart`] / [`ConfigCan`].
    pub port: u8,
}

impl McuCommand for ConfigBridgePort {
    const NAME: &'static str = "config_bridge_port";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.port)]
    }
}

/// Which bus a bridge port carries, and the rate its stream is opened at.
///
/// The two arms differ only in the `config_*` command that creates the stream —
/// [`stream_command`](PortConfig::stream_command) turns either one into that
/// command's name and arguments, so the declaration path can be handed a port
/// without spelling its bus out twice. Everything downstream of the stream
/// (`config_bridge_port`, the port's slot) is the same for both.
///
/// Only [`Uart`](PortConfig::Uart) has a declaration path today
/// (`mcu/bridge_ports.rs` still refuses a `[can_device]` port); the
/// [`Can`](PortConfig::Can) arm is covered by unit tests until that path lands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortConfig {
    /// A serial port, opened at `baud`.
    Uart {
        /// The line rate the stream is opened at.
        baud: u32,
    },
    /// A CAN bus port, opened at `bitrate`.
    Can {
        /// The bus rate the stream is opened at, in bits per second.
        bitrate: u32,
    },
}

impl PortConfig {
    /// The `config_*` command that creates this port's stream: its name, and the
    /// arguments for the object `oid` on port slot `slot`.
    pub fn stream_command(&self, oid: u8, slot: u8) -> (&'static str, Vec<ArgValue>) {
        match *self {
            Self::Uart { baud } => (ConfigUart::NAME, ConfigUart { oid, baud, slot }.args()),
            Self::Can { bitrate } => (ConfigCan::NAME, ConfigCan { oid, bitrate, slot }.args()),
        }
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
    use crate::core::klippy::msg::proto::Payload;
    use serde_json::json;

    /// The three messages this module declares, as the firmware publishes them.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "config_uart oid=%c baud=%u slot=%c": 2,
                "config_can oid=%c bitrate=%u slot=%c": 3,
                "config_bridge_port oid=%c port=%c": 4
            }
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn test_config_can_matches_the_firmware_format() {
        let command = ConfigCan {
            oid: 3,
            bitrate: 1_000_000,
            slot: 1,
        };
        let parser = parser();

        let encoded = parser.encode(ConfigCan::NAME, &command.args()).unwrap();

        // Spelled out against the wire: id 3 as a VLQ, then the oid as a single
        // byte (`%c`), the bitrate as a VLQ (`%u`, so 1000000 is three bytes),
        // and the slot as a single byte. Each of the three parameters is
        // distinct and non-zero, so a reordered or dropped argument cannot
        // encode to the same payload.
        let mut expected = Payload::new();
        expected.push_i16(3).unwrap();
        expected.push_u8(3).unwrap();
        expected.push_u32(1_000_000).unwrap();
        expected.push_u8(1).unwrap();
        assert_eq!(encoded.payload(), expected.payload());

        // And it decodes back to the arguments, so the view and the firmware
        // format cannot drift apart unnoticed.
        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, ConfigCan::NAME);
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_config_bridge_port_carries_the_stream_oid_as_port() {
        let command = ConfigBridgePort { oid: 5, port: 4 };
        let parser = parser();

        let encoded = parser
            .encode(ConfigBridgePort::NAME, &command.args())
            .unwrap();

        let mut expected = Payload::new();
        expected.push_i16(4).unwrap();
        expected.push_u8(5).unwrap();
        expected.push_u8(4).unwrap();
        assert_eq!(encoded.payload(), expected.payload());
    }

    #[test]
    fn test_port_config_uart_is_the_config_uart_command() {
        let parser = parser();
        let uart = ConfigUart {
            oid: 2,
            baud: DEFAULT_UART_BAUD,
            slot: 3,
        };

        assert_eq!(ConfigUart::NAME, "config_uart");
        assert_eq!(
            PortConfig::Uart {
                baud: DEFAULT_UART_BAUD
            }
            .stream_command(uart.oid, uart.slot),
            (ConfigUart::NAME, uart.args())
        );
        // The arguments the enum produces are the ones the firmware's format
        // accepts, not just a matching vector.
        let (name, args) = PortConfig::Uart {
            baud: DEFAULT_UART_BAUD,
        }
        .stream_command(uart.oid, uart.slot);
        assert!(parser.encode(name, &args).is_ok());
    }

    #[test]
    fn test_port_config_can_is_the_config_can_command() {
        let parser = parser();
        let can = ConfigCan {
            oid: 2,
            bitrate: 1_000_000,
            slot: 3,
        };

        assert_eq!(ConfigCan::NAME, "config_can");
        assert_eq!(
            PortConfig::Can { bitrate: 1_000_000 }.stream_command(can.oid, can.slot),
            (ConfigCan::NAME, can.args())
        );
        let (name, args) = PortConfig::Can { bitrate: 1_000_000 }.stream_command(can.oid, can.slot);
        assert!(parser.encode(name, &args).is_ok());
    }
}
