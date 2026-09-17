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
//! This is the one exchange that happens *before* a data dictionary exists, which
//! is why these two formats are the only ones the host hard-codes — see
//! [`IDENTIFY_MESSAGES`].
//!
//! # Security
//!
//! The implementation enforces a maximum decompressed data size (1 MB) to prevent
//! zip-bomb style attacks where a tiny compressed payload expands to enormous data.

use super::codec::{McuCommand, McuResponse, Params};
use super::error::McuError;
use super::Mcu;
use crate::core::klippy::msg::proto::ArgValue;
use flate2::read::ZlibDecoder;
use std::io::Read;
use tokio::time::Duration;
use tracing::debug;

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

/// Timeout for the complete identify handshake, used by [`Mcu::connect`].
///
/// Individual chunk requests get the whole budget: a healthy MCU answers in
/// microseconds, so a shorter per-chunk timeout would only add tuning knobs.
pub const IDENTIFY_TIMEOUT: Duration = Duration::from_secs(10);

/// Number of bytes requested per identify chunk (bytes).
///
/// Matches Klipper's hard-coded `count=40`.
const IDENTIFY_CHUNK_SIZE: u32 = 40;

/// Maximum allowed payload size (1 MB), applied both to the compressed bytes
/// received and to the decompressed body.
///
/// Prevents zip-bomb style attacks where a tiny compressed payload expands to
/// enormous data, causing OOM.
const MAX_IDENTIFY_DATA_SIZE: usize = 1024 * 1024;

/// Identify data sent by the MCU firmware.
///
/// The payload is a JSON object whose schema is owned by the firmware; keys such
/// as `version`, `build_versions`, `config`, `enumerations`, `commands`,
/// `responses` or `output` are *not* modelled here. The complete JSON body is
/// preserved verbatim so that unknown or future fields are never lost or
/// rejected.
///
/// [`Dictionary`](super::Dictionary) is the structured view built from it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Identify {
    /// Complete JSON body as received from the MCU, uncompressed and decoded.
    pub data: serde_json::Value,
}

impl Identify {
    /// Fetch the identify payload from a connected MCU.
    ///
    /// Requests chunks until the MCU answers with an empty payload at the
    /// current offset, then decompresses the result and parses it as JSON.
    ///
    /// This runs before any dictionary is installed, so it uses
    /// [`Mcu::call_msg_ungated`] rather than the ordinary typed call.
    ///
    /// # Errors
    /// * [`McuError::Call`] — transport failure or timeout while waiting for a
    ///   chunk.
    /// * [`McuError::IdentifyProtocol`] — a chunk arrived out of order, or the
    ///   payload grew beyond [`MAX_IDENTIFY_DATA_SIZE`].
    /// * [`McuError::IdentifyCompression`] — the payload is not valid zlib data.
    /// * [`McuError::IdentifyJson`] — the decompressed body is not valid JSON.
    pub(crate) async fn fetch(mcu: &Mcu, timeout: Duration) -> Result<Self, McuError> {
        let compressed = Self::fetch_compressed(mcu, timeout).await?;
        debug!(
            "Identify payload received: {} compressed bytes",
            compressed.len()
        );

        let body = Self::decompress(&compressed)?;
        debug!("Identify payload decompressed: {} bytes", body.len());

        let data =
            serde_json::from_slice(&body).map_err(|e| McuError::IdentifyJson(e.to_string()))?;
        Ok(Self { data })
    }

    /// Request chunks until the MCU reports the payload is complete.
    ///
    /// Returns the still-compressed payload exactly as the MCU sent it.
    async fn fetch_compressed(mcu: &Mcu, timeout: Duration) -> Result<Vec<u8>, McuError> {
        let mut payload: Vec<u8> = Vec::new();

        loop {
            let request = IdentifyRequest {
                offset: payload.len() as u32,
            };
            let chunk = mcu
                .call_msg_ungated::<IdentifyRequest, IdentifyChunk>(&request, timeout)
                .await?;

            // The MCU echoes the offset it is answering for. A mismatch means the
            // stream is out of sync; Klipper would silently retry the same offset
            // forever, so surface it instead of looping or corrupting the body.
            if chunk.offset as usize != payload.len() {
                return Err(McuError::IdentifyProtocol(format!(
                    "identify chunk offset mismatch: expected {}, got {}",
                    payload.len(),
                    chunk.offset
                )));
            }

            if chunk.data.is_empty() {
                return Ok(payload);
            }

            if payload.len() + chunk.data.len() > MAX_IDENTIFY_DATA_SIZE {
                return Err(McuError::IdentifyProtocol(format!(
                    "identify payload exceeds {} bytes",
                    MAX_IDENTIFY_DATA_SIZE
                )));
            }

            payload.extend_from_slice(&chunk.data);
        }
    }

    /// Decompress the payload, refusing to expand beyond the size limit.
    ///
    /// The decoder is wrapped in [`Read::take`] so a zip bomb is cut off while
    /// being read rather than after it has already been materialised.
    fn decompress(compressed: &[u8]) -> Result<Vec<u8>, McuError> {
        let mut decoder = ZlibDecoder::new(compressed).take(MAX_IDENTIFY_DATA_SIZE as u64 + 1);
        let mut body = Vec::new();
        decoder
            .read_to_end(&mut body)
            .map_err(|e| McuError::IdentifyCompression(e.to_string()))?;

        if body.len() > MAX_IDENTIFY_DATA_SIZE {
            return Err(McuError::IdentifyProtocol(format!(
                "identify payload decompresses beyond {} bytes",
                MAX_IDENTIFY_DATA_SIZE
            )));
        }
        Ok(body)
    }
}

/// `identify offset=%u count=%c` — host → MCU request for one chunk.
struct IdentifyRequest {
    offset: u32,
}

impl McuCommand for IdentifyRequest {
    const NAME: &'static str = "identify";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt32(self.offset),
            ArgValue::UInt8(IDENTIFY_CHUNK_SIZE as u8),
        ]
    }
}

/// `identify_response offset=%u data=%.*s` — MCU → host chunk.
struct IdentifyChunk {
    offset: u32,
    data: Vec<u8>,
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
    use crate::core::klippy::config::mcu::McuConfig;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::McuRestartMethod;
    use crate::core::klippy::msg::proto::Payload;
    use flate2::write::ZlibEncoder;
    use flate2::Compression;
    use std::io::Write;

    /// A dictionary with the same shape as a real firmware's, trimmed down.
    const DICTIONARY_JSON: &str = r#"{
        "app": "Klipper",
        "version": "v0.12.0-1-g1234567",
        "commands": {
            "identify offset=%u count=%c": 1,
            "get_clock": 5,
            "get_uptime": 4
        },
        "responses": {
            "identify_response offset=%u data=%.*s": 0,
            "clock clock=%u": 18,
            "uptime high=%u clock=%u": 17
        },
        "config": {"CLOCK_FREQ": 20000000}
    }"#;

    fn compress(body: &[u8]) -> Vec<u8> {
        let mut encoder = ZlibEncoder::new(Vec::new(), Compression::default());
        encoder.write_all(body).unwrap();
        encoder.finish().unwrap()
    }

    /// Payload of an `identify offset=%u count=%c` request.
    fn request_payload(offset: u32) -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(1).unwrap();
        payload.push_u32(offset).unwrap();
        payload.push_u8(IDENTIFY_CHUNK_SIZE as u8).unwrap();
        payload.into_raw()
    }

    /// Payload of an `identify_response offset=%u data=%.*s` answer.
    fn response_payload(offset: u32, data: &[u8]) -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(0).unwrap();
        payload.push_u32(offset).unwrap();
        payload.push_bytes(data).unwrap();
        payload.into_raw()
    }

    /// Build the full chunked exchange for `compressed` and return the mappings.
    ///
    /// Frame sequence numbers line up on both sides: the send task numbers each
    /// batch from 0 and the receive loop consumes frames numbered from 0, and a
    /// synchronous call sends exactly one frame per exchange. Both counters live
    /// in the low 4 bits of the sequence byte, so they wrap at 16 — relevant as
    /// soon as a payload needs more than 16 chunks.
    fn chunked_mappings(compressed: &[u8], chunk_size: usize) -> Vec<MappingEntry> {
        let mut mappings = Vec::new();
        let mut offset = 0usize;
        let mut seq = 0u8;

        loop {
            let end = (offset + chunk_size).min(compressed.len());
            let data = &compressed[offset..end];
            mappings.push(MappingEntry {
                input: Frame::new(seq, request_payload(offset as u32)),
                outputs: vec![Frame::new(seq, response_payload(offset as u32, data))],
            });
            seq = seq.wrapping_add(1) & 0x0f;
            offset = end;

            if data.is_empty() {
                break;
            }
        }
        mappings
    }

    fn config(mappings: Vec<MappingEntry>) -> McuConfig {
        McuConfig {
            name: "test_mcu".to_string(),
            restart_method: McuRestartMethod::Command,
            interface: Interface::new(TestDevice::new(mappings)),
        }
    }

    fn mcu_with(mappings: Vec<MappingEntry>) -> Mcu {
        Mcu::from((
            "test_mcu".to_string(),
            Interface::new(TestDevice::new(mappings)),
        ))
    }

    // -----------------------------------------------------------------------
    // Chunk assembly
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_fetch_single_chunk() {
        let body = DICTIONARY_JSON.as_bytes();
        let compressed = compress(body);
        let mappings = chunked_mappings(&compressed, 40);
        // ceil(len / 40) data chunks plus the terminating empty chunk.
        assert_eq!(mappings.len(), compressed.len().div_ceil(40) + 1);

        let identify = Identify::fetch(&mcu_with(mappings), Duration::from_secs(1))
            .await
            .unwrap();

        assert_eq!(identify.data["app"], "Klipper");
        assert_eq!(identify.data["config"]["CLOCK_FREQ"], 20_000_000);
    }

    #[tokio::test]
    async fn test_fetch_multiple_chunks_wrapping_sequence_numbers() {
        let body = DICTIONARY_JSON.as_bytes();
        let compressed = compress(body);
        // Small chunks force several round trips — more than 16, so the 4-bit
        // sequence counter wraps mid-transfer.
        let mappings = chunked_mappings(&compressed, 8);
        assert_eq!(mappings.len(), compressed.len().div_ceil(8) + 1);
        assert!(mappings.len() > 16, "expected the sequence counter to wrap");

        let identify = Identify::fetch(&mcu_with(mappings), Duration::from_secs(5))
            .await
            .unwrap();

        assert_eq!(identify.data["version"], "v0.12.0-1-g1234567");
    }

    #[tokio::test]
    async fn test_fetch_rejects_offset_mismatch() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let mappings = vec![MappingEntry {
            // The MCU answers a different offset than requested.
            input: Frame::new(0, request_payload(0)),
            outputs: vec![Frame::new(0, response_payload(40, &compressed[..8]))],
        }];

        let err = Identify::fetch(&mcu_with(mappings), Duration::from_secs(1))
            .await
            .unwrap_err();

        match err {
            McuError::IdentifyProtocol(msg) => {
                assert!(msg.contains("offset mismatch"), "{msg}");
            }
            other => panic!("expected a protocol error, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn test_fetch_rejects_corrupt_compressed_data() {
        let garbage = b"this is not zlib data".to_vec();
        let mappings = chunked_mappings(&garbage, 40);

        let err = Identify::fetch(&mcu_with(mappings), Duration::from_secs(1))
            .await
            .unwrap_err();

        assert!(matches!(err, McuError::IdentifyCompression(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_fetch_rejects_non_json_body() {
        let mappings = chunked_mappings(&compress(b"not json at all"), 40);

        let err = Identify::fetch(&mcu_with(mappings), Duration::from_secs(1))
            .await
            .unwrap_err();

        assert!(matches!(err, McuError::IdentifyJson(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_fetch_times_out_when_mcu_stays_silent() {
        // The request is mapped to no output at all.
        let mappings = vec![MappingEntry {
            input: Frame::new(0, request_payload(0)),
            outputs: Vec::new(),
        }];

        let err = Identify::fetch(&mcu_with(mappings), Duration::from_millis(50))
            .await
            .unwrap_err();

        assert!(matches!(err, McuError::Call(_)), "{err:?}");
    }

    // -----------------------------------------------------------------------
    // Decompression limits
    // -----------------------------------------------------------------------

    #[test]
    fn test_decompress_rejects_zip_bomb() {
        // Highly compressible 4 MB of zeros, well past the 1 MB limit.
        let bomb = compress(&vec![0u8; 4 * 1024 * 1024]);
        assert!(bomb.len() < MAX_IDENTIFY_DATA_SIZE);

        let err = Identify::decompress(&bomb).unwrap_err();
        match err {
            McuError::IdentifyProtocol(msg) => {
                assert!(msg.contains("decompresses beyond"), "{msg}");
            }
            other => panic!("expected a protocol error, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------------
    // Mcu::identify / Mcu::connect
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_identify_installs_dictionary() {
        let mappings = chunked_mappings(&compress(DICTIONARY_JSON.as_bytes()), 40);
        let mcu = mcu_with(mappings);

        let installed = mcu.identify(Duration::from_secs(1)).await.unwrap();

        // 3 commands + 3 responses, minus the 2 identify messages that were
        // already registered when the Mcu was created.
        assert_eq!(installed, 4);
        assert!(mcu.is_identified());

        let dictionary = mcu.dictionary().unwrap();
        assert_eq!(dictionary.message("get_clock").unwrap().id, 5);
        assert_eq!(dictionary.constant_f64("CLOCK_FREQ"), Some(20_000_000.0));
    }

    #[tokio::test]
    async fn test_connect_returns_identified_mcu() {
        let mappings = chunked_mappings(&compress(DICTIONARY_JSON.as_bytes()), 40);

        let mcu = Mcu::connect(config(mappings)).await.unwrap();

        assert_eq!(mcu.name(), "test_mcu");
        assert!(mcu.is_identified());
        assert!(mcu.dictionary().unwrap().message("get_uptime").is_some());
    }

    #[tokio::test]
    async fn test_identify_propagates_handshake_failure() {
        // No mappings at all: the first chunk request cannot even be sent, so the
        // exchange fails. `Mcu::connect` would report the same error, but it uses
        // the fixed `IDENTIFY_TIMEOUT`, so drive the short timeout directly.
        let mcu = mcu_with(Vec::new());

        let err = mcu.identify(Duration::from_millis(50)).await.unwrap_err();

        assert!(matches!(err, McuError::Call(_)), "{err:?}");
        assert!(!mcu.is_identified());
    }
}
