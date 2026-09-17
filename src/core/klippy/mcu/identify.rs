//! The host-side identify formats — the single exception to "formats come from
//! the firmware".
//!
//! Every other message format is learned from the MCU's data dictionary, but the
//! dictionary is itself transferred through these two messages. They therefore
//! have to be known before the handshake, which is why they live in the transport
//! layer: [`Mcu::from_parts`](super::Mcu) registers them at construction time.
//!
//! The protocol that drives them — chunked requests, decompression, and
//! dictionary installation — lives in the command layer, since it is host-side
//! protocol logic rather than transport: see
//! [`cmd::identify`](super::cmd::identify).
//!
//! Both entries are repeated verbatim in the firmware-provided dictionary, so
//! installing that dictionary must skip messages which are already registered
//! (see [`Dictionary::install`](super::Dictionary::install)).

/// The identify request/response message formats defined by the host.
///
/// The ids (0 and 1) and formats match Klipper's `msgproto.DefaultMessages`.
/// These two entries are the only formats the host is allowed to hard-code; every
/// other format is installed from the MCU data dictionary after the handshake.
pub const IDENTIFY_MESSAGES: &[(i16, &str)] = &[
    (0, "identify_response offset=%u data=%.*s"),
    (1, "identify offset=%u count=%c"),
];
