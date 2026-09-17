//! The identify command pair — the one command whose formats the host owns.
//!
//! `identify` / `identify_response` move the firmware's data dictionary to the
//! host. Because a dictionary cannot describe the exchange that delivers it, this
//! pair is the only command whose wire formats the host hard-codes; those two
//! entries stay next to the transport that has to register them before anything
//! else is known (they are `identify::IDENTIFY_MESSAGES`).
//!
//! What is defined here is what every other command module defines: the typed
//! view. `IdentifyRequest` says "send me bytes `offset..offset+40`", and
//! `IdentifyChunk` carries one answer — the offset it is answering for, plus a
//! slice of the compressed payload.
//!
//! Driving those two is the chunked transfer: following the offset, capping the
//! size, decompressing, decoding. That is not a command concern — the command
//! only ever sends one window — so it lives in the sibling `identify` module,
//! which is also where the public entry points are
//! ([`Mcu::connect`](crate::core::klippy::mcu::Mcu::connect) /
//! [`Mcu::identify`](crate::core::klippy::mcu::Mcu::identify)).
//!
//! Both views are crate-internal: nothing outside needs to name the identify
//! messages, and the driver in `identify` is their only caller.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// Number of bytes requested per chunk — the `count` argument of `identify`.
///
/// Matches Klipper's hard-coded `count=40`.
pub(crate) const IDENTIFY_CHUNK_SIZE: u8 = 40;

/// `identify offset=%u count=%c` — host → MCU request for one chunk.
#[derive(Debug)]
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
#[derive(Debug)]
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

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::identify::IDENTIFY_MESSAGES;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::Payload;
    use std::sync::Arc;

    /// The transfer module's `IDENTIFY_MESSAGES` are the only description of
    /// these two messages that exists before a dictionary does, so the views
    /// above are checked against *them* rather than against a copy of the
    /// format string. (This import points the other way on purpose; it is a
    /// test-only edge.)
    fn parser() -> Parser {
        let mut parser = Parser::new();
        parser.register_all(IDENTIFY_MESSAGES).unwrap();
        parser
    }

    /// Decode one response payload through the registered format and into the
    /// typed view.
    fn decode_chunk(parser: &Parser, payload: Payload) -> Result<IdentifyChunk, McuError> {
        let decoded = parser.decode(payload).expect("payload must decode");
        let (msg, values) = &decoded[0];
        IdentifyChunk::decode(&Params::new(Arc::clone(msg), values))
    }

    #[test]
    fn test_chunk_size_matches_klipper() {
        // Klipper hard-codes `count=40` in its identify client; a different
        // value here is not wrong, but it must be a deliberate change.
        assert_eq!(IDENTIFY_CHUNK_SIZE, 40);
    }

    #[test]
    fn test_request_matches_the_identify_format() {
        let parser = parser();
        let args = IdentifyRequest { offset: 0x1234 }.args();

        let payload = parser.encode(IdentifyRequest::NAME, &args).unwrap();

        // Spelled out against the wire: id 1, offset as a VLQ, then the chunk
        // size as a single byte (`%c`).
        let mut expected = Payload::new();
        expected.push_i16(1).unwrap();
        expected.push_u32(0x1234).unwrap();
        expected.push_u8(IDENTIFY_CHUNK_SIZE).unwrap();
        assert_eq!(payload.payload(), expected.payload());

        // Decoding what was just encoded yields the arguments again, so the view
        // and the host-owned format cannot drift apart unnoticed.
        let decoded = parser.decode(payload).unwrap();
        assert_eq!(decoded[0].0.name, IdentifyRequest::NAME);
        assert_eq!(decoded[0].1, args);
    }

    #[test]
    fn test_chunk_matches_the_identify_response_format() {
        let parser = parser();

        // Arbitrary bytes, not just text: the payload is zlib output.
        let data = vec![0x78, 0x9c, 0xff, 0xfe];
        let payload = parser
            .encode(
                IdentifyChunk::NAME,
                &[ArgValue::UInt32(80), ArgValue::Bytes(data.clone())],
            )
            .unwrap();

        let chunk = decode_chunk(&parser, payload).unwrap();
        assert_eq!(chunk.offset, 80);
        assert_eq!(chunk.data, data);
    }

    #[test]
    fn test_chunk_decodes_the_completion_marker() {
        let parser = parser();
        let payload = parser
            .encode(
                IdentifyChunk::NAME,
                &[ArgValue::UInt32(0), ArgValue::Bytes(Vec::new())],
            )
            .unwrap();

        // An empty payload at the requested offset is how the transfer ends;
        // the view reports it as-is and leaves the decision to the driver.
        let chunk = decode_chunk(&parser, payload).unwrap();
        assert_eq!(chunk.offset, 0);
        assert!(chunk.data.is_empty());
    }

    #[test]
    fn test_chunk_decode_reports_parameter_mismatches() {
        // Same message, but `data` declared as an integer: the view asks for
        // bytes and must refuse rather than reinterpret the number.
        let mut parser = Parser::new();
        parser
            .register(0, "identify_response offset=%u data=%u")
            .unwrap();
        let payload = parser
            .encode(
                IdentifyChunk::NAME,
                &[ArgValue::UInt32(1), ArgValue::UInt32(2)],
            )
            .unwrap();
        let err = decode_chunk(&parser, payload).unwrap_err();
        assert!(matches!(err, McuError::Decode(_)), "{err:?}");

        // Same message, but the parameter is named differently: the view is
        // bound to the firmware-visible name, so this is an undeclared lookup.
        let mut parser = Parser::new();
        parser
            .register(0, "identify_response offset=%u payload=%.*s")
            .unwrap();
        let payload = parser
            .encode(
                IdentifyChunk::NAME,
                &[ArgValue::UInt32(1), ArgValue::Bytes(b"x".to_vec())],
            )
            .unwrap();
        let err = decode_chunk(&parser, payload).unwrap_err();
        assert!(matches!(err, McuError::Decode(_)), "{err:?}");
    }
}
