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
use crate::core::klippy::mcu::{Dictionary, Mcu, McuCallError, McuError};
use crate::core::klippy::msg::parser::Parser;
use flate2::read::ZlibDecoder;
use std::io::Read;
use std::sync::Arc;
use tokio::time::{sleep, Duration, Instant};
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
/// Each chunk request gets the whole budget, and its attempts share it rather
/// than stack on top of it (see [`IDENTIFY_ATTEMPT_TIMEOUT`]): a healthy MCU
/// answers in microseconds, while the bounded attempts inside the budget are
/// what turn silence into a nak — the chunk still ends exactly where it used
/// to.
pub const IDENTIFY_TIMEOUT: Duration = Duration::from_secs(10);

/// How long one identify attempt waits for its response before the silence is
/// read as a nak and the chunk is renumbered and requested again.
///
/// What the retry exists for is the numbering mismatch: a 5-byte empty frame is
/// both the ack of a healthy block and the nak of one the firmware never took,
/// and this host starts at 0 where some firmwares wait at 1 — so a firmware
/// stuck one ahead naks the first request and the exchange would otherwise end
/// there. That root cause is fixed by renumbering, not by waiting longer, so the
/// first attempts give up early: a healthy firmware answers a chunk in
/// milliseconds, out of a buffer it compressed at build time, and a spurious
/// retry is harmless — the request is idempotent (same offset) and whichever
/// response arrives first is accepted. Only the **last** attempt waits out the
/// rest of the chunk's budget, so a board that is merely slow is never cut off
/// before the `timeout` its caller gave (see [`IDENTIFY_TIMEOUT`]).
const IDENTIFY_ATTEMPT_TIMEOUT: Duration = Duration::from_millis(500);

/// Retries after the first attempt, each preceded by renumbering the send
/// window onto the sequence the firmware reported (see
/// [`Mcu::renumber_to_firmware`](crate::core::klippy::mcu::Mcu::renumber_to_firmware)).
///
/// The first renumber is the fix; rounds after it only cover a frame lost on a
/// line that is already misbehaving, so they are a bounded fallback rather than
/// a cure: three keeps a dead port failing inside one chunk's budget instead of
/// stalling every chunk of the transfer, and the attempt that follows them
/// still runs to the deadline. Retries, windows and pauses all share the one
/// budget the caller passed, so no retry makes a chunk take longer than the
/// timeout it has today.
const IDENTIFY_MAX_RETRIES: u32 = 3;

/// The pause before each fallback retry: 50 ms, doubling (50/100/200 ms, 350 ms
/// across all retries — a fraction of any budget a caller passes).
///
/// Long enough for the firmware to have taken the renumbered request (the
/// transport's own retransmit floor is 25 ms, `MIN_RTO`, and no retransmit is
/// pending after a renumber — it cleared the window), and doubling keeps a
/// broken line from being hammered.
const IDENTIFY_RETRY_BACKOFF: Duration = Duration::from_millis(50);

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
            let chunk = Self::request_chunk(mcu, &request, timeout).await?;

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

    /// One chunk request, under the rule that silence is a nak.
    ///
    /// `timeout` is the whole budget for this chunk — the caller's
    /// [`IDENTIFY_TIMEOUT`] semantics — and every attempt shares it: windows and
    /// backoff pauses are carved out of it, so retries never make a chunk take
    /// longer than it does without them.
    ///
    /// A timeout means the line went quiet exactly where the firmware should
    /// have spoken. The empty frame that may have come back cannot say whether
    /// the request was taken or refused — both carry the firmware's expected
    /// sequence — so quiet is taken as the refusal: the send window is
    /// renumbered onto the sequence the firmware reported (connection init,
    /// `serialqueue.c:196-201`) and the same chunk is requested again
    /// ([`Mcu::renumber_to_firmware`](crate::core::klippy::mcu::Mcu::renumber_to_firmware)).
    /// Any other error is not silence: it is returned as it is.
    async fn request_chunk(
        mcu: &Mcu,
        request: &IdentifyRequest,
        timeout: Duration,
    ) -> Result<IdentifyChunk, McuError> {
        let deadline = Instant::now() + timeout;
        let mut retries_left = IDENTIFY_MAX_RETRIES;
        let mut backoff = IDENTIFY_RETRY_BACKOFF;
        let mut timed_out: Option<McuError> = None;

        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            // The early attempts get their own window; the last one runs the
            // budget out, so a slow firmware still gets the whole `timeout`.
            let window = if retries_left == 0 {
                remaining
            } else {
                IDENTIFY_ATTEMPT_TIMEOUT.min(remaining / (1 + retries_left))
            };
            match mcu
                .call_msg_ungated::<IdentifyRequest, IdentifyChunk>(request, window)
                .await
            {
                Ok(chunk) => return Ok(chunk),
                Err(err @ McuError::Call(McuCallError::Timeout(_))) => timed_out = Some(err),
                Err(other) => return Err(other),
            }

            if retries_left == 0 {
                break;
            }
            retries_left -= 1;
            debug!(
                "identify offset={} was silent for {window:?}: read as a nak, renumbering to \
                 the firmware's sequence and requesting the chunk again",
                request.offset
            );
            mcu.renumber_to_firmware().await?;
            let pause = backoff.min(deadline.saturating_duration_since(Instant::now()));
            if pause.is_zero() {
                break;
            }
            sleep(pause).await;
            backoff = backoff.saturating_mul(2);
        }

        Err(timed_out.unwrap_or_else(|| {
            McuError::Call(McuCallError::Timeout(format!(
                "no response for an identify chunk within {timeout:?}"
            )))
        }))
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
/// A connection that had to take over a session already running is a board nothing
/// reset; when its handshake *still* fails, say so, because the timeout that
/// follows is only a symptom of it (`mcu/object.rs` is the caller that asked for
/// the reset).
fn connect_error(mcu: &Mcu, err: McuError) -> McuError {
    if mcu.took_over_session() {
        McuError::OldSession(err.to_string())
    } else {
        err
    }
}

/// What the firmware reported about itself, as one DEBUG record.
///
/// The shape upstream's `MCUConnectHelper.log_info` prints (`klippy/mcu.py:830-840`):
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
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, MappingEntry};
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
    /// batch from `first_seq` and the receive loop consumes frames numbered the
    /// same way, and a synchronous call sends exactly one frame per exchange.
    /// Both counters live in the low 4 bits of the sequence byte, so they wrap at
    /// 16 — relevant as soon as a payload needs more than 16 chunks.
    ///
    /// `first_seq` is the number the firmware is waiting for when the transfer
    /// starts: `0` for a board that takes this connection's first block, `1` for
    /// one whose counter is already one ahead of it.
    fn chunked_mappings(compressed: &[u8], chunk_size: usize, first_seq: u8) -> Vec<MappingEntry> {
        chunked_mappings_from(compressed, chunk_size, first_seq, 0)
    }

    /// [`chunked_mappings`] for a transfer that starts part-way in: the first
    /// entry answers `start_offset` under `first_seq`, and every chunk after it
    /// follows in order. The prefix of the transfer is scripted by the caller —
    /// a nak, a takeover, a silence that costs a renumber — and this is what
    /// continues from wherever that left the exchange.
    fn chunked_mappings_from(
        compressed: &[u8],
        chunk_size: usize,
        first_seq: u8,
        start_offset: usize,
    ) -> Vec<MappingEntry> {
        let mut mappings = Vec::new();
        let mut offset = start_offset;
        let mut seq = first_seq;

        loop {
            let end = (offset + chunk_size).min(compressed.len());
            let data = &compressed[offset..end];
            mappings.push(MappingEntry {
                input: Frame::new(seq, request_payload(offset as u32)),
                // What a real firmware sends while handling a block: the response,
                // and then the empty ack. Both are stamped with the counter
                // *after* taking the block (`src/command.c:301-305`), which is the
                // number the host's send window reads to drop the block it took —
                // without the ack a transfer of more than `MAX_PENDING_BLOCKS`
                // chunks would wait for room that never comes.
                outputs: vec![
                    Frame::new(
                        seq.wrapping_add(1) & 0x0f,
                        response_payload(offset as u32, data),
                    ),
                    Frame::new(seq.wrapping_add(1) & 0x0f, Vec::new()),
                ],
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
        Interface::new(FrameMock::new(mappings))
    }

    fn mcu_with(mappings: Vec<MappingEntry>) -> Mcu {
        Mcu::for_test("test_mcu", Interface::new(FrameMock::new(mappings)))
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
        let mappings = chunked_mappings(&compressed, 40, 0);
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
        let mappings = chunked_mappings(&compressed, 8, 0);
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
        let mappings = chunked_mappings(&garbage, 40, 0);

        let err = fetch(mappings, Duration::from_secs(1)).await.unwrap_err();

        assert!(matches!(err, McuError::IdentifyCompression(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_fetch_rejects_non_json_body() {
        let mappings = chunked_mappings(&compress(b"not json at all"), 40, 0);

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

    // -----------------------------------------------------------------------
    // Silence is a nak: renumber and request again
    // -----------------------------------------------------------------------

    /// A firmware whose `next_sequence` sits at 1 naks this connection's first
    /// request (sequence 0) with an empty frame carrying 1 — byte-for-byte the
    /// ack an accepted block gets — and then says nothing for the rest of the
    /// budget. Read as an ack, the exchange dies there and identify never
    /// completes; the handshake has to take the silence as the nak it cannot be
    /// proved not to be, adopt the number the firmware reported, and request the
    /// chunk again under that number.
    #[tokio::test]
    async fn test_a_firmware_one_ahead_is_renumbered_and_identifies() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let mut mappings = vec![MappingEntry {
            // The nak: an empty frame stamped with the sequence the firmware is
            // still waiting for, then silence.
            input: Frame::new(0, request_payload(0)),
            outputs: vec![Frame::new(1, Vec::new())],
        }];
        // Only a request numbered 1 is taken, and the transfer runs from there.
        mappings.extend(chunked_mappings(&compressed, 40, 1));

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        let identify = Identify::fetch(&mcu, Duration::from_secs(1))
            .await
            .expect("the nak'd request is renumbered and the transfer completes");

        assert_eq!(identify.data["app"], "Klipper");
        assert!(
            !mcu.took_over_session(),
            "one ahead of our own first block is the ambiguity band, not a session to take over"
        );
        // The first request went out at 0, the renumbered one at the firmware's
        // 1, and every chunk after it exactly once.
        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "the retry carries the sequence the firmware asked for, and nothing is sent twice"
        );
        assert_eq!(
            sent[0].seq(),
            0,
            "the first try is under this connection's 0"
        );
        assert_eq!(
            sent[1].seq(),
            1,
            "the retry is renumbered onto the firmware's 1"
        );
    }

    /// Payload of the periodic `stats` a running firmware interleaves with the
    /// identify exchange: message id -12 as a signed VLQ, then 8 parameter
    /// bytes. The host-owned parser does not know that id before the dictionary
    /// arrives, so the receive task drops the frame.
    fn stats_payload() -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(-12).unwrap();
        payload.extend(&[0; 8]).unwrap();
        payload.into_raw()
    }

    /// The noise a real line carries must leave the nak/renumber exchange
    /// exactly where it is. The firmware answers the first request with a
    /// `stats` frame whose sequence is **congruent** with what this host has
    /// seen (`delta == 0`), so it must move no number — the nak behind it is
    /// still the session's first new sequence, and the window adopts that
    /// number only once the first attempt's window has run out in silence.
    /// The frame's id is unknown before the dictionary, so it must not disturb
    /// the pending call either. Nothing goes out early, and the retry under the
    /// firmware's number completes the transfer.
    #[tokio::test]
    async fn test_a_congruent_stats_frame_does_not_disturb_the_renumber() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let mut mappings = vec![MappingEntry {
            // What the firmware sends instead of an answer: a `stats` frame
            // stamped with the sequence this host has already seen (delta 0),
            // then the nak carrying the number the firmware waits for, then
            // silence for the rest of the first attempt's window.
            input: Frame::new(0, request_payload(0)),
            outputs: vec![Frame::new(0, stats_payload()), Frame::new(1, Vec::new())],
        }];
        // Only a request numbered 1 is taken, and the transfer runs from there.
        mappings.extend(chunked_mappings(&compressed, 40, 1));

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        // A 2.5 s budget makes the first attempt's window the full
        // IDENTIFY_ATTEMPT_TIMEOUT (the smaller of that and a quarter of the
        // budget), so the retry's timing is this constant, not the budget.
        let started = Instant::now();
        let identify = Identify::fetch(&mcu, Duration::from_millis(2500))
            .await
            .expect("the congruent noise neither blocks the answer nor prevents the renumber");
        let elapsed = started.elapsed();

        assert_eq!(identify.data["app"], "Klipper");
        assert!(
            !mcu.took_over_session(),
            "a congruent noise frame takes over no session"
        );
        assert!(
            elapsed >= IDENTIFY_ATTEMPT_TIMEOUT,
            "the retry waits out the first window instead of firing on the noise: {elapsed:?}"
        );
        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "the noise disturbs nothing: one request at 0, the renumbered retry at the \
             firmware's 1, then exactly one per chunk"
        );
        assert_eq!(
            sent[0].seq(),
            0,
            "the first try is under this connection's 0"
        );
        assert_eq!(
            sent[1].seq(),
            1,
            "the retry carries the number the firmware asked for"
        );
    }

    // -----------------------------------------------------------------------
    // reset → reconnect: 对任意残余数的接收侧对齐（B5）
    //
    // 真机失败现场（/tmp/vA.log）：reset 后的重连会话里，固件的回答被接收侧
    // 判成「本连接从未发过的块」连环丢弃，identify 四轮全部超时。三条按现场
    // 时序复刻：同余首帧、空 ack 首帧、改号后的响应帧。
    // -----------------------------------------------------------------------

    /// （a）固件的 `next_sequence` 停在 16 的倍数上（一块没重启的板子），它的
    /// 首帧与本会话 `seen = 0` **同余**（`delta == 0`）：既不动号、不消耗首帧
    /// 豁免，也不宣称接管——传输照常完成，每块一个请求，一个不多。
    #[tokio::test]
    async fn test_a_congruent_first_frame_completes_the_transfer_in_phase() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let chunk = IDENTIFY_CHUNK_SIZE as usize;
        assert!(
            compressed.len() > chunk,
            "the transfer needs a second chunk"
        );
        let mut mappings = vec![MappingEntry {
            input: Frame::new(0, request_payload(0)),
            outputs: vec![
                // The leftover ack of the session before this one: stamped with
                // the firmware's counter, which sits at a multiple of 16 —
                // congruent with this host's `seen = 0`, so it moves no number.
                Frame::new(0, Vec::new()),
                // This connection's block 0 matches the firmware's own low nibble
                // and is taken, so the answer carries the counter after it.
                Frame::new(
                    1,
                    response_payload(0, &compressed[..chunk.min(compressed.len())]),
                ),
                Frame::new(1, Vec::new()),
            ],
        }];
        // From chunk 1 on the exchange runs in phase: the answer put `seen` at 1,
        // so the next request goes out under 1.
        mappings.extend(chunked_mappings_from(&compressed, chunk, 1, chunk));

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        let identify = Identify::fetch(&mcu, Duration::from_secs(2))
            .await
            .expect("a congruent first frame leaves the transfer in phase");

        assert_eq!(identify.data["app"], "Klipper");
        assert!(
            !mcu.took_over_session(),
            "a congruent frame names no session to take over"
        );
        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "one request per chunk from 0 on: no renumber, no retransmit"
        );
    }

    /// （b）本会话的首帧是**空 ack**（携带固件正在等的号），它消耗掉首帧豁免；
    /// 紧随其后的响应帧号更高——接收侧必须照样采纳（不能判 never-sent），
    /// identify 一次完成、不重试。
    #[tokio::test]
    async fn test_an_empty_ack_first_frame_adopts_the_response_behind_it() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let chunk = IDENTIFY_CHUNK_SIZE as usize;
        assert!(
            compressed.len() > chunk,
            "the transfer needs a second chunk"
        );
        let mut mappings = vec![
            // The nak: the firmware waits for 5, this connection's first block
            // is 0. The empty frame carries 5 — the session's first *new*
            // number, which the adoption below spends.
            MappingEntry {
                input: Frame::new(0, request_payload(0)),
                outputs: vec![Frame::new(5, Vec::new())],
            },
            // The same request again under the adopted number. Its answer is
            // stamped 6 — a number this connection has not sent anything at
            // yet (it sent 5), so it only lands if the receive side takes it
            // with the window it just announced.
            MappingEntry {
                input: Frame::new(5, request_payload(0)),
                outputs: vec![
                    Frame::new(
                        6,
                        response_payload(0, &compressed[..chunk.min(compressed.len())]),
                    ),
                    Frame::new(6, Vec::new()),
                ],
            },
        ];
        // The answer put `seen` at 6, so chunk 1 goes out under 6.
        mappings.extend(chunked_mappings_from(&compressed, chunk, 6, chunk));

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        let identify = Identify::fetch(&mcu, Duration::from_secs(2))
            .await
            .expect("the number behind a spent first-frame exemption is still adopted");

        assert_eq!(identify.data["app"], "Klipper");
        assert!(mcu.took_over_session(), "the nak named a running session");
        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "the first block goes out at 0, the adopted number carries the rest; \
             no attempt is burned"
        );
        assert_eq!(sent[0].seq(), 0);
        assert_eq!(sent[1].seq(), 5, "the retry carries the firmware's 5");
    }

    /// （c）改号（identify 读到静默就改号重发）之后，固件的回答必须仍被接收侧
    /// 采纳。改号把发送窗退回到 `seen`，接收侧若因此把固件的号判成「本连接从未
    /// 发过」，回答永远进不来，identify 只能一轮轮超时到死。
    #[tokio::test]
    async fn test_a_response_behind_a_renumber_is_still_accepted() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let chunk = IDENTIFY_CHUNK_SIZE as usize;
        assert!(
            compressed.len() > chunk,
            "the transfer needs a second chunk"
        );
        let mut mappings = vec![
            // The nak: the firmware waits for 5. Adoption → resend at 5.
            MappingEntry {
                input: Frame::new(0, request_payload(0)),
                outputs: vec![Frame::new(5, Vec::new())],
            },
            // The firmware takes the resent block (its counter moves to 6) but
            // its answer is still on the wire: the first attempt's window runs
            // out in silence, and identify reads that as a nak — the window is
            // renumbered back onto 5 and the chunk is asked for again.
            MappingEntry {
                input: Frame::new(5, request_payload(0)),
                outputs: Vec::new(),
            },
            // The renumbered retry, and the answer behind it: stamped 6, one
            // past the number the window was rewound to.
            MappingEntry {
                input: Frame::new(5, request_payload(0)),
                outputs: vec![
                    Frame::new(
                        6,
                        response_payload(0, &compressed[..chunk.min(compressed.len())]),
                    ),
                    Frame::new(6, Vec::new()),
                ],
            },
        ];
        mappings.extend(chunked_mappings_from(&compressed, chunk, 6, chunk));

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        let identify = Identify::fetch(&mcu, Duration::from_millis(2500))
            .await
            .expect("the answer behind the renumbered window is accepted");

        assert_eq!(identify.data["app"], "Klipper");
        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "0 → 5 (adoption) → 5 (renumbered retry) → one per chunk after it"
        );
        assert_eq!(sent[1].seq(), 5, "the adoption carries the firmware's 5");
        assert_eq!(sent[2].seq(), 5, "the renumber puts the retry back on 5");
    }

    /// The healthy path is untouched: the first answer arrives, so the send
    /// window is neither renumbered nor asked to send anything again.
    #[tokio::test]
    async fn test_a_healthy_first_answer_is_neither_renumbered_nor_repeated() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let mappings = chunked_mappings(&compressed, 40, 0);

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        Identify::fetch(&mcu, Duration::from_secs(1))
            .await
            .expect("a firmware that answers on the first try identifies as always");

        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "one request per chunk at its own number: no renumbering, no retry"
        );
    }

    /// Mid-session, an empty frame with the data frame right behind it is the
    /// normal ack it has always been: the silence rule must not read a nak into
    /// it, and the transfer goes on one request per chunk.
    #[tokio::test]
    async fn test_an_ack_with_the_response_right_behind_it_stays_an_ack() {
        let compressed = compress(DICTIONARY_JSON.as_bytes());
        let mut mappings = chunked_mappings(&compressed, 40, 0);
        assert!(mappings.len() > 2, "the exchange has to run mid-transfer");
        // The second chunk's answer comes out in the other order: the empty ack
        // first, its data frame immediately behind it.
        mappings[1].outputs.swap(0, 1);

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        Identify::fetch(&mcu, Duration::from_secs(1))
            .await
            .expect("an ack whose response follows is answered like any other");

        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "the empty frame changed nothing: one request per chunk, at its own number"
        );
    }

    /// The cap, not the clock, ends a hopeless transfer: a firmware that keeps
    /// answering with its number but never sends the response gets the renumbered
    /// request `IDENTIFY_MAX_RETRIES` more times and then the chunk gives up —
    /// with most of the budget unspent, so it is the cap that stopped it.
    #[tokio::test]
    async fn test_identify_gives_up_after_its_retry_cap() {
        let attempts = 1 + IDENTIFY_MAX_RETRIES as usize;
        let mappings: Vec<MappingEntry> = (0..attempts)
            .map(|i| MappingEntry {
                input: Frame::new(i as u8, request_payload(0)),
                // Ack-shaped both ways: stamped one past the block, no response.
                outputs: vec![Frame::new(i as u8 + 1, Vec::new())],
            })
            .collect();

        let device = FrameMock::new(mappings.clone());
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        let err = Identify::fetch(&mcu, Duration::from_secs(3))
            .await
            .unwrap_err();
        assert!(matches!(err, McuError::Call(_)), "{err:?}");

        let sent = recorder.frames();
        let expected: Vec<Frame> = mappings.iter().map(|entry| entry.input.clone()).collect();
        assert_eq!(
            sent, expected,
            "exactly one attempt per round, {attempts} in all, each under the number the \
             firmware reported"
        );
    }

    #[tokio::test]
    async fn test_a_failed_handshake_from_an_older_session_says_so() {
        // The firmware answers the identify request with a sequence from a
        // session this host never opened: the connection takes it over and sends
        // the request again (there is no mapping for that second attempt here), so
        // the handshake times out. What the caller is told names the real cause,
        // because a board that answers like this never rebooted.
        let mcu = mcu_with(vec![MappingEntry {
            input: Frame::new(0, request_payload(0)),
            outputs: vec![Frame::new(9, response_payload(0, &compress(b"{}")))],
        }]);

        let err = Identify::fetch(&mcu, Duration::from_millis(50))
            .await
            .unwrap_err();
        assert!(matches!(err, McuError::Call(_)), "{err:?}");
        assert!(mcu.took_over_session());

        let err = connect_error(&mcu, err);
        assert!(matches!(err, McuError::OldSession(_)), "{err:?}");
        assert!(
            err.to_string().contains("still in an earlier session"),
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
        assert!(!mcu.took_over_session());
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
        let mappings = chunked_mappings(&compress(DICTIONARY_JSON.as_bytes()), 40, 0);
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
        let mappings = chunked_mappings(&compress(DICTIONARY_JSON.as_bytes()), 40, 0);

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
