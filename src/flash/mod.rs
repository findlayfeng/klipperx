//! katapult bootloader flashing — frame codec and transport-agnostic session.
//!
//! The katapult bootloader speaks one fixed frame layout on any byte stream:
//!
//! ```text
//! <01><88><cmd><word_cnt><payload (word_cnt × 4 bytes)><crc16-LE><99><03>
//! ```
//!
//! All integers are little-endian. The CRC16-CCITT (init `0xFFFF`) covers
//! `cmd` through the payload — not the two header bytes, not the CRC itself,
//! not the trailer — matching `third_party/katapult/scripts/flashtool.py`
//! (`_build_command`) and the firmware check in
//! `third_party/katapult/src/command.c`.
//!
//! Note this differs from this repo's klipper frame layer
//! (`crate::core::klippy::frame`), whose CRC starts at the length byte; the
//! two codecs are not interchangeable and share no helpers.
//!
//! Modules:
//! - [`bridge_session`] — the bridge half of a flash run: declare a bridge's
//!   downstream ports, reset their devices, and open the flashing tunnel on
//!   the target's port.
//! - [`frame`] — pure encode/decode plus a sliding-window scanner that finds
//!   frames in a stream, skipping garbage one byte at a time.
//! - [`session`] — the transport-agnostic flash session: connect →
//!   send_block × N → eof → verify → complete, over any `Read + Write`.
//! - [`bridge`] — the `Read + Write` face of one bridge flashing tunnel:
//!   whole katapult blocks through a bridge port in tunnel mode.
//! - [`transport`] — the two direct byte streams: an exclusive serial
//!   tty/USB link and a SocketCAN node session.
//! - [`cli`] — the standalone `flash` binary's arguments and actions.
//! - `fake` (tests only) — the in-process scripted bootloader the session
//!   scenarios run against.

pub mod bridge;
pub mod bridge_session;
pub mod cli;
pub mod frame;
pub mod session;
pub mod transport;

#[cfg(test)]
pub(crate) mod fake;
#[cfg(test)]
mod tests;

pub use bridge_session::{
    open_bridge, BridgeError, BridgeLink, BridgePort, BridgeReady, BridgeTarget, PortReset,
};
pub use frame::{build_frame, crc16_ccitt, Frame, FrameScanner};
pub use session::{
    command, complete, connect, eof, flash, flash_with_config, get_canbus_id, request_block,
    send_block, FlashError, SessionConfig, Summary,
};
pub use transport::{open_can, open_tty, CanBus, TransportError, TtyLink};
