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
//! - [`frame`] — pure encode/decode plus a sliding-window scanner that finds
//!   frames in a stream, skipping garbage one byte at a time.

pub mod frame;

pub use frame::{build_frame, crc16_ccitt, Frame, FrameScanner};
