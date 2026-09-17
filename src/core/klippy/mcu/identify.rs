//! Identify protocol: data dictionary negotiation between host (klippy) and MCU (firmware).
//!
//! The identify mechanism establishes communication by exchanging a JSON-formatted
//! configuration and command description from the MCU to the host. The data is
//! zlib-compressed and transferred in chunks via the identify request/response
//! message pair.
//!
//! # Protocol
//!
//! | Message ID | Format | Direction |
//! |------------|--------|-----------|
//! | `0` | `identify_response offset=%u data=%.*s` | MCU → host |
//! | `1` | `identify offset=%u count=%c` | host → MCU |
//!
//! The host sends `identify offset=N count=40` repeatedly, and the MCU responds
//! with `identify_response` carrying the offset and data chunk. When the offset
//! equals the total data length and the data is empty, the exchange is complete.
//!
//! # Security
//!
//! The implementation enforces a maximum decompressed data size (1 MB) to prevent
//! zip-bomb style attacks where a tiny compressed payload expands to enormous data.

/// The identify request/response message formats defined by the host.
///
/// These two messages are the **only** formats the host is allowed to
/// hard-code: they are needed to bootstrap the connection, because the MCU's
/// data dictionary (which carries every other message format) is itself
/// transferred through them. All remaining formats are installed from the
/// dictionary after the handshake — see [`Dictionary`](super::Dictionary).
///
/// The ids (0 and 1) and formats match Klipper's `msgproto.DefaultMessages`;
/// both entries are repeated verbatim in the MCU-provided dictionary, so
/// installation must skip messages that are already registered.
///
/// These are registered when creating a new [`Mcu`](super::Mcu).
pub const IDENTIFY_MESSAGES: &[(i16, &str)] = &[
    (0, "identify_response offset=%u data=%.*s"),
    (1, "identify offset=%u count=%c"),
];

/// Size of each identify data chunk (bytes).
#[allow(dead_code)]
const IDENTIFY_CHUNK_SIZE: u32 = 40;

/// Maximum allowed decompressed data size (1 MB).
/// Prevents zip-bomb style attacks where a tiny compressed payload
/// expands to enormous data, causing OOM.
#[allow(dead_code)]
const MAX_IDENTIFY_DATA_SIZE: usize = 1024 * 1024;

/// Identify data sent by the MCU firmware.
///
/// The payload is a JSON object whose schema is owned by the firmware; keys such
/// as `version`, `build_versions`, `config`, `enumerations`, `commands`,
/// `responses` or `output` are *not* modelled here. The complete JSON body is
/// preserved verbatim so that unknown or future fields are never lost or
/// rejected.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Identify {
    /// Complete JSON body as received from the MCU, uncompressed and decoded.
    pub data: serde_json::Value,
}
