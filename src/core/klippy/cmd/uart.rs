//! UART objects — the byte streams a host hands to a bridge as its ports.
//!
//! The bridge has no compile-time port list: which byte streams it owns is a
//! configuration fact. The host creates a UART object per downstream port at
//! configuration time, and hands it to the bridge in the same pass:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_uart oid=%c baud=%u path=%c` |
//! | host → MCU | `config_bridge_port oid=%c uart=%c` |
//!
//! `config_uart` creates the object and opens the stream behind it at `baud`
//! (the firmware's `src/uart.c`); a stream it cannot open, or a rate it cannot
//! set, shuts the firmware down with a message naming why, so a port that
//! cannot be brought up fails the configuration instead of the bridge coming up
//! without it. `path` is the port **slot** — the port's index on the bridge,
//! which is also the slot the host named that port's tty with before the library
//! came up (`Interface::host_mux`) — so the firmware opens the stream the host
//! meant, not a path baked into the build. `config_bridge_port` hands that object
//! to the router as a downstream port: the bridge numbers ports by the order
//! these commands
//! arrive, and the `index` of the port-level `SET_PORT_NODEID`
//! (`interface/devices::serial::set_port_node_id`) is that number, so the host
//! queues the pairs in the order the `[uart_device]` sections are declared
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

/// `config_uart oid=%c baud=%u path=%c` — create a UART object and open its
/// stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigUart {
    /// The object id the firmware allocates for this UART.
    pub oid: u8,
    /// The line rate the stream is opened at.
    pub baud: u32,
    /// The port slot the stream belongs to, named `path` on the wire
    /// (`third_party/klipper/src/uart.c`): the port's **index** on its bridge
    /// (`mcu/bridge_ports.rs`), which is the same number the host named that
    /// port's tty with (`klipper_host_port`, before the library comes up). The
    /// firmware opens the tty registered for this slot, or the build's default
    /// when the host named none.
    pub path: u8,
}

impl McuCommand for ConfigUart {
    const NAME: &'static str = "config_uart";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.baud),
            ArgValue::UInt8(self.path),
        ]
    }
}

/// `config_bridge_port oid=%c uart=%c` — hand a UART object to the bridge as a
/// downstream port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigBridgePort {
    /// The object id of the port itself.
    pub oid: u8,
    /// The oid of the UART object the port carries.
    pub uart: u8,
}

impl McuCommand for ConfigBridgePort {
    const NAME: &'static str = "config_bridge_port";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.uart)]
    }
}
