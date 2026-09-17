//! The identify command pair — the one command whose formats the host owns.
//!
//! `identify` / `identify_response` move the firmware's data dictionary to the
//! host. Because a dictionary cannot describe the exchange that delivers it, this
//! pair is the only command whose wire formats the host hard-codes; those two
//! entries stay next to the transport that has to register them before anything
//! else is known (they are `mcu::identify::IDENTIFY_MESSAGES`).
//!
//! What is defined here is what every other command module defines: the typed
//! view. `IdentifyRequest` says "send me bytes `offset..offset+40`", and
//! `IdentifyChunk` carries one answer — the offset it is answering for, plus a
//! slice of the compressed payload.
//!
//! Driving those two is the chunked transfer: following the offset, capping the
//! size, decompressing, decoding. That is not a command concern — the command
//! only ever sends one window — so it lives with the transport in `mcu::identify`,
//! which is also where the public entry points are
//! ([`Mcu::connect`](super::super::Mcu::connect) /
//! [`Mcu::identify`](super::super::Mcu::identify)).
//!
//! Both views are crate-internal: nothing outside needs to name the identify
//! messages, and the driver in `mcu::identify` is their only caller.

use crate::core::klippy::mcu::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// Number of bytes requested per chunk — the `count` argument of `identify`.
///
/// Matches Klipper's hard-coded `count=40`.
pub(crate) const IDENTIFY_CHUNK_SIZE: u8 = 40;

/// `identify offset=%u count=%c` — host → MCU request for one chunk.
pub(crate) struct IdentifyRequest {
    /// Offset of the first byte wanted, which is also the payload length so far.
    pub(crate) offset: u32,
}

impl McuCommand for IdentifyRequest {
    const NAME: &'static str = "identify";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt32(self.offset),
            ArgValue::UInt8(IDENTIFY_CHUNK_SIZE),
        ]
    }
}

/// `identify_response offset=%u data=%.*s` — MCU → host chunk.
pub(crate) struct IdentifyChunk {
    /// Offset the firmware is answering for; must match what was requested.
    pub(crate) offset: u32,
    /// Payload slice; empty means the transfer is complete.
    pub(crate) data: Vec<u8>,
}

impl McuResponse for IdentifyChunk {
    const NAME: &'static str = "identify_response";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            offset: params.get_u32("offset")?,
            data: params.get_bytes("data")?,
        })
    }
}
