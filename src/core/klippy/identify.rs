//! Identify — the bootstrap exchange that hands the firmware's data dictionary
//! to the host.
//!
//! This module owns the transfer:
//!
//! * [`IDENTIFY_MESSAGES`] — the two formats the host is allowed to hard-code —
//!   and `new_parser`, which hands out the registry that already knows them
//!   ([`Mcu::new`](crate::core::klippy::mcu::Mcu::new) calls it at construction
//!   time).
//! * the chunked transfer itself: request a chunk, append it, stop at the empty
//!   terminator, then decompress and decode the body (`Identify::fetch`).
//!
//! The typed views of those two messages — and the chunk size, which is the
//! command's own argument — come from the command layer like every other
//! command's, in [`cmd::identify`](super::cmd::identify).
//!
//! [`Mcu::connect`] and [`Mcu::identify`] are defined here too, so the whole
//! bootstrap reads in one place: bring the transport up ([`Mcu::new`]), transfer
//! the payload, decode it, install the dictionary.
//!
//! # Why the transfer lives here and not in the command layer
//!
//! [`cmd`](super::cmd) holds the commands that describe firmware
//! capabilities: their formats come from the dictionary, and they run through
//! the ordinary typed call path. Identify is the opposite on both counts.
//!
//! * The host owns the formats. A dictionary cannot describe the exchange that
//!   delivers it, so these two formats are the single exception to "formats come
//!   from the firmware", and the transport has to know them at construction time.
//! * It runs *before* any dictionary exists, so [`Mcu::call_msg`](crate::core::klippy::mcu::Mcu::call_msg)
//!   — which refuses to run unidentified — is unavailable; the exchange uses
//!   `Mcu::call_msg_ungated`.
//!
//! The chunk loop is also not a capability: a request only ever asks for one
//! window of bytes, and assembling a stream of those into a payload is the
//! counterpart of the framing this layer already hard-codes.
//!
//! Both entries are repeated verbatim in the firmware-provided dictionary, so
//! installing that dictionary must skip messages which are already registered
//! (see [`Dictionary::install`](crate::core::klippy::mcu::Dictionary::install)).
//!
//! # Security
//!
//! The implementation enforces a maximum decompressed data size (1 MB) to prevent
//! zip-bomb style attacks where a tiny compressed payload expands to enormous data.

use super::cmd::identify::{IdentifyChunk, IdentifyRequest};
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::{Dictionary, Mcu, McuError};
use crate::core::klippy::msg::parser::Parser;
use flate2::read::ZlibDecoder;
use std::io::Read;
use std::sync::Arc;
use tokio::time::Duration;
use tracing::{debug, info};

/// The identify request/response message formats defined by the host.
///
/// The ids (0 and 1) and formats match Klipper's `msgproto.DefaultMessages`.
/// These two entries are the only formats the host is allowed to hard-code; every
/// other format is installed from the MCU data dictionary after the handshake.
pub const IDENTIFY_MESSAGES: &[(i16, &str)] = &[
    (0, "identify_response offset=%u data=%.*s"),
    (1, "identify offset=%u count=%c"),
];

/// The parser an [`Mcu`](crate::core::klippy::mcu::Mcu) starts with: an empty registry that already
/// knows [`IDENTIFY_MESSAGES`].
///
/// Registration belongs next to the formats it registers, and giving out the
/// finished registry rather than a `register_formats(&mut parser)` step means a
/// future construction path cannot forget to call it. Every other format arrives
/// with the firmware dictionary (see
/// [`Mcu::install_dictionary`](crate::core::klippy::mcu::Mcu::install_dictionary)).
pub(crate) fn new_parser() -> Parser {
    let mut parser = Parser::new();
    parser
        .register_all(IDENTIFY_MESSAGES)
        .expect("the host-owned identify formats must be valid");
    parser
}

/// Timeout for the complete identify handshake, used by
/// [`Mcu::identify`](crate::core::klippy::mcu::Mcu::identify) and
/// [`Mcu::connect`](crate::core::klippy::mcu::Mcu::connect).
///
/// Each chunk request gets the whole budget: a healthy MCU answers in
/// microseconds, so a shorter per-chunk timeout would only add tuning knobs.
pub const IDENTIFY_TIMEOUT: Duration = Duration::from_secs(10);

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
/// [`Dictionary`] is the structured view built from it.
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
    /// # Errors
    /// * [`McuError::Call`] — transport failure or timeout while waiting for a
    ///   chunk.
    /// * [`McuError::IdentifyProtocol`] — a chunk arrived out of order, or the
    ///   payload grew beyond `MAX_IDENTIFY_DATA_SIZE`.
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

// The typed views of the two messages (`IdentifyRequest`, `IdentifyChunk`) and the
// chunk size — the command's own argument — are defined with the other commands,
// in [`cmd::identify`](super::cmd::identify).

// ===========================================================================
// The bootstrap, step by step
// ===========================================================================

impl Mcu {
    /// Bring up an MCU and complete the identify handshake.
    ///
    /// This is the normal entry point. The flow is deliberately linear:
    ///
    /// 1. [`Mcu::new`] — transport up, parser knows only the identify formats;
    /// 2. [`Mcu::identify`] — transfer the payload, turn it into a
    ///    [`Dictionary`], and install it;
    /// 3. hand back the shared [`Arc`] that command modules take.
    ///
    /// Step 3 is why the handle is an `Arc`: `Mcu` shuts the device down when the
    /// last handle is dropped, and command modules each hold one.
    ///
    /// # Errors
    /// Returns [`McuError`] if any step of the handshake fails, and
    /// [`McuError::OldSession`] when the firmware turned out to be mid-session —
    /// a board that kept running rather than one that just came up. The partially
    /// initialized MCU is dropped on the way out, which shuts the interface down
    /// again.
    pub async fn connect(
        name: impl Into<String>,
        interface: Interface,
    ) -> Result<Arc<Mcu>, McuError> {
        let mcu = Arc::new(Mcu::new(name, interface));
        if let Err(err) = mcu.identify(IDENTIFY_TIMEOUT).await {
            return Err(connect_error(&mcu, err));
        }
        Ok(mcu)
    }

    /// Transfer the firmware data dictionary and install it.
    ///
    /// Returns the number of messages newly registered from the dictionary.
    /// Calling this on an MCU that is already identified replaces the dictionary;
    /// messages already registered are skipped, so an interrupted handshake can be
    /// retried.
    ///
    /// Three things happen, in order: the compressed payload is fetched by
    /// `Identify::fetch`, decoded from JSON into a [`Dictionary`], and handed to
    /// [`Mcu::install_dictionary`](crate::core::klippy::mcu::Mcu::install_dictionary). Nothing is
    /// registered until the payload has decoded successfully, so a garbled
    /// dictionary leaves the MCU exactly as unidentified as it was.
    ///
    /// Use [`Mcu::connect`] instead unless the default [`IDENTIFY_TIMEOUT`] is
    /// wrong, the handshake has to be retried, or the payload is wanted before the
    /// dictionary is installed.
    ///
    /// # Errors
    /// Returns [`McuError`] if the exchange fails, the payload cannot be decoded,
    /// or the dictionary cannot be installed.
    pub async fn identify(&self, timeout: Duration) -> Result<usize, McuError> {
        let identify = Identify::fetch(self, timeout).await?;
        let dictionary = Dictionary::from_json(identify.data)?;
        let installed = self.install_dictionary(dictionary)?;

        info!(
            "MCU '{}' identified: {} messages registered",
            self.name(),
            installed
        );
        if let Some(dictionary) = self.dictionary() {
            debug!("{}", describe_dictionary(self.name(), &dictionary));
        }
        Ok(installed)
    }
}

/// The error a failed [`Mcu::connect`] reports.
///
/// A firmware that answered from an older session never rebooted, and that is the
/// whole reason the handshake failed: report it as what it is instead of as the
/// timeout it causes. The caller is the one that can act on it — this is how an
/// `rpi_usb` reset finds out that switching the port's power did not reset the
/// board (`mcu/object.rs`).
fn connect_error(mcu: &Mcu, err: McuError) -> McuError {
    if mcu.answered_from_old_session() {
        McuError::OldSession(err.to_string())
    } else {
        err
    }
}

/// What the firmware reported about itself, as one DEBUG record.
///
/// The shape upstream's `MCUConnectHelper.log_info` prints (`klippy/mcu.py:845`):
/// the version pair, how many messages the firmware declared, and its
/// compile-time constants. Constants are sorted because they arrive in a
/// `HashMap`, which has no order — two runs of the same firmware should read the
/// same.
fn describe_dictionary(name: &str, dictionary: &Dictionary) -> String {
    let raw = dictionary.raw();
    let field = |key: &str| raw.get(key).and_then(|value| value.as_str()).unwrap_or("?");
    let mut constants: Vec<String> = dictionary
        .constants()
        .iter()
        .map(|(key, value)| format!("{key}={value}"))
        .collect();
    constants.sort();
    format!(
        "MCU '{name}' firmware: {} {} / {} ({} commands, {} responses)\n\
         MCU '{name}' constants: {}",
        field("app"),
        field("version"),
        field("build_versions"),
        dictionary.commands().len(),
        dictionary.responses().len(),
        constants.join(" ")
    )
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::identify::IDENTIFY_CHUNK_SIZE;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::msg::proto::Payload;
    use flate2::write::{DeflateEncoder, ZlibEncoder};
    use flate2::Compression;
    use std::io::Write;
    use std::sync::Arc;

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
        payload.push_u8(IDENTIFY_CHUNK_SIZE).unwrap();
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

    fn interface(mappings: Vec<MappingEntry>) -> Interface {
        Interface::new(TestDevice::new(mappings))
    }

    fn mcu_with(mappings: Vec<MappingEntry>) -> Mcu {
        Mcu::for_test("test_mcu", Interface::new(TestDevice::new(mappings)))
    }

    /// Fetch with a short timeout — the default is 10 s, too slow for tests that
    /// are supposed to fail.
    async fn fetch(mappings: Vec<MappingEntry>, timeout: Duration) -> Result<Identify, McuError> {
        let mcu = mcu_with(mappings);
        Identify::fetch(&mcu, timeout).await
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

        let identify = fetch(mappings, Duration::from_secs(1)).await.unwrap();

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

        let identify = fetch(mappings, Duration::from_secs(5)).await.unwrap();

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

        let err = fetch(mappings, Duration::from_secs(1)).await.unwrap_err();

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

        let err = fetch(mappings, Duration::from_secs(1)).await.unwrap_err();

        assert!(matches!(err, McuError::IdentifyCompression(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_fetch_rejects_non_json_body() {
        let mappings = chunked_mappings(&compress(b"not json at all"), 40);

        let err = fetch(mappings, Duration::from_secs(1)).await.unwrap_err();

        assert!(matches!(err, McuError::IdentifyJson(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_fetch_times_out_when_mcu_stays_silent() {
        // The request is mapped to no output at all.
        let mappings = vec![MappingEntry {
            input: Frame::new(0, request_payload(0)),
            outputs: Vec::new(),
        }];

        let err = fetch(mappings, Duration::from_millis(50))
            .await
            .unwrap_err();

        assert!(matches!(err, McuError::Call(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_a_failed_handshake_from_an_older_session_says_so() {
        // The firmware answers the identify request with a sequence from a
        // session this host never opened: the receive task drops it — and notes
        // it — so the handshake times out. What the caller is told names the real
        // cause, because a board that answers like this never rebooted.
        let mcu = mcu_with(vec![MappingEntry {
            input: Frame::new(0, request_payload(0)),
            outputs: vec![Frame::new(9, response_payload(0, &compress(b"{}")))],
        }]);

        let err = Identify::fetch(&mcu, Duration::from_millis(50))
            .await
            .unwrap_err();
        assert!(matches!(err, McuError::Call(_)), "{err:?}");
        assert!(mcu.answered_from_old_session());

        let err = connect_error(&mcu, err);
        assert!(matches!(err, McuError::OldSession(_)), "{err:?}");
        assert!(
            err.to_string().contains("answered from an older session"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_a_failed_handshake_from_a_fresh_firmware_keeps_its_error() {
        // A board that just came up answers nothing at all here: the handshake
        // fails the same way, but nothing says the firmware was already running,
        // so the error is reported as it is.
        let mcu = mcu_with(vec![]);

        let err = Identify::fetch(&mcu, Duration::from_millis(50))
            .await
            .unwrap_err();
        assert!(!mcu.answered_from_old_session());
        assert!(matches!(connect_error(&mcu, err), McuError::Call(_)));
    }

    // -----------------------------------------------------------------------
    // Decompression limits
    // -----------------------------------------------------------------------

    #[test]
    fn test_decompress_requires_the_zlib_wrapper() {
        // The firmware stores the dictionary as `zlib.compress(...)` output, so
        // the payload starts with a zlib header (`0x78 0xda` at the default
        // level — verified against a build's `compile_time_request.c`, where the
        // 699-byte blob decompresses to the 1323-byte dictionary). Raw deflate
        // carries the same data without that header, and `DeflateDecoder` would
        // happily accept it, so this pins the wrapper we require.
        let mut encoder = DeflateEncoder::new(Vec::new(), Compression::default());
        encoder.write_all(b"{}").unwrap();
        let raw = encoder.finish().unwrap();
        assert!(!raw.starts_with(&[0x78]));

        let err = Identify::decompress(&raw).unwrap_err();
        assert!(matches!(err, McuError::IdentifyCompression(_)), "{err:?}");
    }

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

    #[test]
    fn test_describe_dictionary_summarises_the_firmware() {
        let dictionary =
            Dictionary::from_json(serde_json::from_str(DICTIONARY_JSON).unwrap()).unwrap();

        let summary = describe_dictionary("mcu", &dictionary);

        assert!(summary.contains("Klipper v0.12.0-1-g1234567"), "{summary}");
        assert!(summary.contains("3 commands, 3 responses"), "{summary}");
        assert!(summary.contains("CLOCK_FREQ=20000000"), "{summary}");
    }

    #[tokio::test]
    async fn test_connect_returns_identified_mcu() {
        let mappings = chunked_mappings(&compress(DICTIONARY_JSON.as_bytes()), 40);

        let mcu: Arc<Mcu> = Mcu::connect("test_mcu", interface(mappings)).await.unwrap();

        assert_eq!(mcu.name(), "test_mcu");
        assert!(mcu.is_identified());
        assert!(mcu.dictionary().unwrap().message("get_uptime").is_some());
    }

    #[tokio::test]
    async fn test_identify_propagates_handshake_failure() {
        // No mappings at all: the first chunk request cannot even be sent, so the
        // exchange fails. `connect` would report the same error, but it uses the
        // fixed `IDENTIFY_TIMEOUT`, so drive the short timeout directly.
        let mcu = mcu_with(Vec::new());

        let err = mcu.identify(Duration::from_millis(50)).await.unwrap_err();

        assert!(matches!(err, McuError::Call(_)), "{err:?}");
        assert!(!mcu.is_identified());
    }
}
