//! Frame-level id assignment: handing the MCU its id **before identify**, on
//! the wire's own format rather than through the message system.
//!
//! CAN gets its id from an admin frame on `0x3f0` — a bus-level conversation
//! that exists beside the byte stream, before anything else is said. Serial has
//! no side-channel, so this module defines one *inside* the frame format: a
//! frame whose sequence byte carries no `MESSAGE_DEST` bit. The pattern is
//! chosen so that old firmware **discards it by default**, a firmware built to
//! parse it **answers**, and neither outcome can disturb the session that
//! follows.
//!
//! # The pattern
//!
//! ```text
//! [len][0x20]["K" "X" "I" "D"][ascii id][crc16 hi][crc16 lo][0x7e]
//!  \___________________/ \____________________________________/
//!   fails the dest check      ordinary payload + trailer
//! ```
//!
//! * `len` is in `[MESSAGE_MIN, MESSAGE_MAX]`, the payload never contains
//!   `0x7e`, the CRC is the ordinary CRC-16-CCITT over `[len, seq, payload]`.
//!   Everything except the sequence byte is a well-formed frame — so the frame
//!   is self-delimiting (its own trailing `0x7e`) and the payload could be
//!   validated if anyone chose to.
//! * `seq = 0x20`: `(0x20 & !MESSAGE_SEQ_MASK) != MESSAGE_DEST`
//!   (`0x20 != 0x10`). The sequence byte is the only field that differs.
//!
//! # What old firmware does with it
//!
//! `command_find_block` (`third_party/klipper/src/command.c:276`) checks, in
//! order: length range (`:284`), **sequence byte (`:287`)**, SYNC, CRC, then
//! the sequence number. Our frame passes the length check and bails at the
//! sequence-byte check — *before* the payload is ever read, before the CRC is
//! ever computed, before dispatch could run. The error path (`:306-323`)
//! discards bytes up to and including the next `0x7e`, which is our own
//! trailer: exactly this frame is consumed, nothing behind it. Consequences,
//! all of them by construction:
//!
//! * no dispatch → no `shutdown("Invalid command")`, no command executed;
//! * `next_sequence` untouched → the identify exchange that follows starts
//!   exactly where it would have;
//! * at most **one** empty ack-shaped frame in reply (`need_sync` sends a nak
//!   only when `CF_NEED_VALID` was clear — `:320-322`) — an empty frame stamped
//!   with the firmware's current `next_sequence`, which the receive task reads
//!   as the protocol-consistent no-progress notice it is. Every later
//!   assignment frame is discarded **silently** (the flag stays set until a
//!   valid block clears it).
//!
//! So a capable device and an old one are told apart by what comes back: an
//! answer shaped the way only a parser of this pattern could produce (below),
//! or nothing / an ordinary empty frame. Both old-firmware outcomes are
//! harmless; neither advances anything.
//!
//! # The answer (specification)
//!
//! A capable firmware replies with the same shape in the opposite direction:
//!
//! ```text
//! [len][0x20]["K" "X" "I" "D"][echoed id][crc16][0x7e]
//! ```
//!
//! Old firmware can never emit it: every frame it sends is built by
//! `command_add_frame` (`command.c:207-212`), whose sequence byte is
//! `next_sequence`, always in `0x10..=0x1f` (`:16`, `:305`). [`parse_ack`]
//! recognizes the reply where the host's [`FrameStream`] would otherwise
//! discard it — same check, opposite direction — and the id is recorded
//! through `Mcu::assignment_confirmed`.
//!
//! # Scope
//!
//! * Sent by the serial transport at open, before `Mcu::connect` runs identify
//!   (`config/mcu.rs`), mirroring CAN's admin frame inside
//!   `CanSerialDevice::open`. Fire-and-forget: no wait, so an old device costs
//!   nothing but the (harmless) frame itself.
//! * `host_library` and the `test:` simulator do not send it (the simulator's
//!   scripted frames would refuse an unmapped send); CAN keeps its own admin
//!   frame. [Bodies that answer]: devices reassembling a byte stream recognize
//!   the reply wherever [`FrameStream`] sees raw bytes.
//! * The frame-header addressing scheme this feeds is **not** built yet; today
//!   every frame on the wire is the current format, assigned or not. A device
//!   that was never assigned an id communicates exactly as before.
//!
//! [`FrameStream`]: crate::core::klippy::frame::FrameStream

use crate::core::klippy::frame::{
    Frame, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_POS_LEN, MESSAGE_POS_SEQ, MESSAGE_SYNC,
    MESSAGE_TRAILER_SIZE, MESSAGE_TRAILER_SYNC,
};

/// The sequence byte this pattern uses, as a whole byte: bit 5 set, the bits
/// `MESSAGE_DEST` owns clear, the low nibble free.
///
/// Legitimate frames — in either direction — carry `0x10..=0x1f`: bit 4 set by
/// `MESSAGE_DEST`, bits 5-7 zero. `0x20` occupies a bit nothing else uses, so
/// neither direction's ordinary traffic can produce it by accident, and the
/// low nibble stays available for a sequence number should one matter.
pub const ASSIGN_SEQ_BYTE: u8 = 0x20;

/// The mask over the sequence byte's high nibble that identifies the pattern.
const ASSIGN_SEQ_MASK: u8 = 0xf0;

/// The payload marker: fixed bytes every device built to parse this pattern
/// looks for before accepting an id.
///
/// `0x7e` never appears in it, and neither do the ASCII digits of an id, which
/// is what keeps the frame free of an interior SYNC byte (see the module docs:
/// the discard path stops at the first `0x7e`).
pub const MAGIC: &[u8; 4] = b"KXID";

/// The id as it goes on the wire: `MAGIC` followed by decimal digits.
///
/// Digits are `0x30..=0x39` — never `MESSAGE_SYNC` — so the encoding needs no
/// escaping for any id a `u32` can hold.
fn payload(id: u32) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(MAGIC.len() + 10);
    bytes.extend_from_slice(MAGIC);
    bytes.extend_from_slice(id.to_string().as_bytes());
    bytes
}

/// The assignment frame: `id` addressed to the device on the open serial port.
///
/// Sent before identify; the receiving firmware either discards it (module
/// docs) or answers with the shape [`parse_ack`] reads.
pub fn request_frame(id: u32) -> Frame {
    Frame::with_wire_seq(ASSIGN_SEQ_BYTE, payload(id))
}

/// Decode an assignment payload: `MAGIC` followed by non-empty decimal digits.
fn parse_payload(bytes: &[u8]) -> Option<u32> {
    let digits = bytes.strip_prefix(MAGIC)?;
    if digits.is_empty() || !digits.iter().all(u8::is_ascii_digit) {
        return None;
    }
    std::str::from_utf8(digits).ok()?.parse().ok()
}

/// Recognize the answer a device built for [`request_frame`] sends back:
/// `MAGIC` + echoed id in a frame whose sequence byte carries the pattern.
///
/// Every ordinary frame check runs here — length range, the pattern bits,
/// the trailing `0x7e`, the CRC — because the caller is looking at bytes the
/// host's own [`FrameStream`] had already rejected as malformed. The id is
/// returned only when the whole frame is consistent.
///
/// [`FrameStream`]: crate::core::klippy::frame::FrameStream
pub fn parse_ack(raw: &[u8]) -> Option<u32> {
    if raw.len() < MESSAGE_MIN {
        return None;
    }
    let len = raw[MESSAGE_POS_LEN] as usize;
    if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&len) || raw.len() < len {
        return None;
    }
    if raw[MESSAGE_POS_SEQ] & ASSIGN_SEQ_MASK != ASSIGN_SEQ_BYTE {
        return None;
    }
    if raw[len - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
        return None;
    }
    let expected = crate::crc16_ccitt!(&raw[..len - MESSAGE_TRAILER_SIZE]);
    let actual = ((raw[len - 3] as u16) << 8) | (raw[len - 2] as u16);
    if expected != actual {
        return None;
    }
    parse_payload(&raw[2..len - MESSAGE_TRAILER_SIZE])
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::{MESSAGE_DEST, MESSAGE_SEQ_MASK};

    /// The patterned frame for `id`, as raw wire bytes.
    fn raw(id: u32) -> Vec<u8> {
        request_frame(id).raw_bytes()
    }

    #[test]
    fn the_request_is_a_well_formed_frame_with_only_the_seq_byte_wrong() {
        let bytes = raw(4);
        // len = header(2) + payload("KXID4" = 5) + trailer(3) = 10.
        assert_eq!(bytes.len(), 10);
        assert_eq!(bytes[MESSAGE_POS_LEN], 10);
        assert_eq!(bytes[MESSAGE_POS_SEQ], ASSIGN_SEQ_BYTE);
        // The one field that differs from an ordinary frame: bit 4 clear.
        assert_ne!(
            bytes[MESSAGE_POS_SEQ] & !MESSAGE_SEQ_MASK,
            crate::core::klippy::frame::MESSAGE_DEST
        );
        assert_eq!(&bytes[2..7], b"KXID4");
        assert_eq!(*bytes.last().unwrap(), MESSAGE_SYNC);
        // The CRC is the ordinary one over [len, seq, payload] — the shape a
        // capable firmware can validate without inventing a checksum.
        let expected = crate::crc16_ccitt!(&bytes[..bytes.len() - MESSAGE_TRAILER_SIZE]);
        let actual = ((bytes[bytes.len() - 3] as u16) << 8) | bytes[bytes.len() - 2] as u16;
        assert_eq!(expected, actual);
    }

    #[test]
    fn no_payload_byte_is_an_interior_sync() {
        // The firmware's error path discards through the first 0x7e: with the
        // payload (and header) free of it, the frame is consumed in one step.
        // The two CRC bytes are computed, not chosen — one of them can be
        // 0x7e (id 81 produces it), which only splits the discard into two
        // steps; `a_crc_byte_that_is_a_sync_splits_the_discard...` proves the
        // split still never reaches dispatch.
        for id in (0..=999).chain(0x7e..=0x7e + 20) {
            let bytes = raw(id);
            let header_and_payload = &bytes[..bytes.len() - MESSAGE_TRAILER_SIZE];
            assert!(
                header_and_payload.iter().all(|&b| b != MESSAGE_SYNC),
                "id {id} put a SYNC in header/payload: {bytes:02x?}"
            );
            assert!((MESSAGE_MIN..=MESSAGE_MAX).contains(&bytes.len()));
        }
        // A large id still fits (len = 2 + 4 + digits + 3 ≤ 19 for a u32).
        assert!((MESSAGE_MIN..=MESSAGE_MAX).contains(&raw(u32::MAX).len()));
    }

    #[test]
    fn a_crc_byte_that_is_a_sync_splits_the_discard_but_never_reaches_dispatch() {
        // id 81's CRC low byte is 0x7e: an interior SYNC sits in the trailer.
        let request = raw(81);
        assert!(
            request[..request.len() - 1]
                .iter()
                .any(|&b| b == MESSAGE_SYNC),
            "this id should exercise the interior-SYNC path: {request:02x?}"
        );

        // The wire order at connect: our frame, then identify. Walk the
        // firmware's finder to the end: every step that consumes our bytes is
        // a discard (never a dispatch), the request is consumed exactly, and
        // identify dispatches behind it.
        let identify = identify_frame(MESSAGE_DEST);
        let identify_offset = request.len();
        let mut stream = request;
        stream.extend_from_slice(&identify);

        let mut st = FwState::default();
        let mut pos = 0usize;
        let mut dispatched = false;
        while pos < stream.len() {
            match fw_find_block(&stream[pos..], &mut st) {
                Outcome::Dispatch => {
                    assert!(
                        pos >= identify_offset,
                        "our frame was dispatched at offset {pos}"
                    );
                    dispatched = true;
                    break;
                }
                Outcome::SeqNak { consumed } => {
                    assert!(pos >= identify_offset);
                    pos += consumed;
                }
                Outcome::Discarded { consumed, .. } => {
                    assert!(pos < identify_offset, "discard at {pos} ran past our frame");
                    assert!(consumed > 0, "discard consumed nothing; livelock");
                    pos += consumed;
                }
                Outcome::NeedMore => break,
            }
        }
        assert!(dispatched, "identify never dispatched (pos={pos})");
        assert_eq!(pos, identify_offset, "our frame is consumed exactly");
    }

    #[test]
    fn the_pattern_round_trips_through_the_ack_parser() {
        // The answer has the same shape as the request (opposite direction),
        // so the parser accepting the request's bytes is the round trip.
        assert_eq!(parse_ack(&raw(4)), Some(4));
        assert_eq!(parse_ack(&raw(0)), Some(0));
        assert_eq!(parse_ack(&raw(895)), Some(895));
        assert_eq!(parse_ack(&raw(u32::MAX)), Some(u32::MAX));
    }

    #[test]
    fn ordinary_frames_never_parse_as_an_ack() {
        // Every frame old firmware can emit: `command_add_frame` stamps
        // `next_sequence`, in 0x10..=0x1f (`command.c:16,207-212`).
        for seq in 0x10u8..=0x1f {
            let empty = {
                // An empty ack: len = MESSAGE_MIN, ordinary seq byte, CRC.
                let mut frame = vec![MESSAGE_MIN as u8, seq, 0, 0, MESSAGE_SYNC];
                let crc = crate::crc16_ccitt!(&frame[..MESSAGE_MIN - MESSAGE_TRAILER_SIZE]);
                frame[MESSAGE_MIN - 3] = (crc >> 8) as u8;
                frame[MESSAGE_MIN - 2] = crc as u8;
                frame
            };
            assert_eq!(parse_ack(&empty), None, "seq {seq:#04x}");
            let response = Frame::encode(seq & MESSAGE_SEQ_MASK, b"\x01identify_response");
            assert_eq!(parse_ack(&response), None, "seq {seq:#04x}");
        }
    }

    #[test]
    fn a_corrupted_ack_is_rejected() {
        let mut bytes = raw(7);
        // Wrong CRC.
        let mut broken = bytes.clone();
        broken[3] ^= 0xff;
        assert_eq!(parse_ack(&broken), None);
        // Pattern bits cleared — an ordinary frame now.
        let ordinary = {
            let mut b = bytes.clone();
            b[MESSAGE_POS_SEQ] = 0x10;
            let crc = crate::crc16_ccitt!(&b[..b.len() - MESSAGE_TRAILER_SIZE]);
            let len = b.len();
            b[len - 3] = (crc >> 8) as u8;
            b[len - 2] = crc as u8;
            b
        };
        assert_eq!(parse_ack(&ordinary), None);
        // Truncated / short.
        assert_eq!(parse_ack(&bytes[..MESSAGE_MIN - 1]), None);
        bytes.truncate(bytes.len() - 1);
        assert_eq!(parse_ack(&bytes), None);
        // Payload without the marker.
        assert_eq!(parse_payload(b"hello"), None);
        assert_eq!(parse_payload(b"KXID"), None);
        assert_eq!(parse_payload(b"KXID12x"), None);
    }

    // -----------------------------------------------------------------------
    // Old-firmware behavior: a faithful port of `command_find_block`
    // (`third_party/klipper/src/command.c:276-323`), so the compatibility
    // claim above is executable. Structure and citation, not an approximation:
    // the checks run in the C order and the error path consumes through the
    // first SYNC exactly as `memchr` does.
    // -----------------------------------------------------------------------

    /// What one call to the firmware's block finder decides.
    #[derive(Debug, PartialEq, Eq)]
    enum Outcome {
        /// Not enough bytes yet; the buffer waits.
        NeedMore,
        /// A block passed structure, CRC and sequence: dispatch follows.
        Dispatch,
        /// Structure failed; bytes consumed through the SYNC (`pop_count`),
        /// and (only on the first failure) an empty ack frame is sent.
        Discarded { consumed: usize, nacked: bool },
        /// Structure and CRC passed but the sequence was wrong: nak, and the
        /// block is consumed (`*pop_count = msglen` before the check).
        SeqNak { consumed: usize },
    }

    /// The firmware's static state across calls (`command.c:276-281,299,307`).
    struct FwState {
        /// `next_sequence`, initialised to `MESSAGE_DEST` (`command.c:16`).
        next_sequence: u8,
        /// `CF_NEED_SYNC`.
        need_sync: bool,
        /// `CF_NEED_VALID`.
        need_valid: bool,
    }

    impl Default for FwState {
        fn default() -> Self {
            // `static uint8_t next_sequence = MESSAGE_DEST;` (`command.c:16`).
            Self {
                next_sequence: crate::core::klippy::frame::MESSAGE_DEST,
                need_sync: false,
                need_valid: false,
            }
        }
    }

    fn fw_find_block(buf: &[u8], st: &mut FwState) -> Outcome {
        // `if (buf_len && sync_state & CF_NEED_SYNC) goto need_sync;`
        if !buf.is_empty() && st.need_sync {
            return fw_need_sync(buf, st);
        }
        if buf.len() < MESSAGE_MIN {
            return Outcome::NeedMore;
        }
        let msglen = buf[MESSAGE_POS_LEN] as usize;
        // `:284` — length range.
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) {
            return fw_error(buf, st);
        }
        let msgseq = buf[MESSAGE_POS_SEQ];
        // `:287` — the check this pattern is built to meet.
        if (msgseq & !MESSAGE_SEQ_MASK) != crate::core::klippy::frame::MESSAGE_DEST {
            return fw_error(buf, st);
        }
        if buf.len() < msglen {
            return Outcome::NeedMore;
        }
        // `:291` — trailer SYNC.
        if buf[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
            return fw_error(buf, st);
        }
        // `:293-297` — CRC.
        let msgcrc = ((buf[msglen - 3] as u16) << 8) | buf[msglen - 2] as u16;
        let crc = crate::crc16_ccitt!(&buf[..msglen - MESSAGE_TRAILER_SIZE]);
        if crc != msgcrc {
            return fw_error(buf, st);
        }
        // `:299` — a structure-valid block clears NEED_VALID.
        st.need_valid = false;
        // `:301-303` — sequence.
        if msgseq != st.next_sequence {
            return Outcome::SeqNak { consumed: msglen };
        }
        st.next_sequence =
            ((msgseq + 1) & MESSAGE_SEQ_MASK) | crate::core::klippy::frame::MESSAGE_DEST;
        Outcome::Dispatch
    }

    fn fw_error(buf: &[u8], st: &mut FwState) -> Outcome {
        // `:310-313` — a leading SYNC is dropped alone, silently.
        if buf.first() == Some(&MESSAGE_SYNC) {
            return Outcome::Discarded {
                consumed: 1,
                nacked: false,
            };
        }
        // `:314` then fall into need_sync (`:315`).
        st.need_sync = true;
        fw_need_sync(buf, st)
    }

    fn fw_need_sync(buf: &[u8], st: &mut FwState) -> Outcome {
        // `:317-323`: discard through the first SYNC (or the whole buffer),
        // then the one-time nak.
        let consumed = match buf.iter().position(|&b| b == MESSAGE_SYNC) {
            Some(position) => {
                st.need_sync = false;
                position + 1
            }
            None => buf.len(),
        };
        if st.need_valid {
            return Outcome::Discarded {
                consumed,
                nacked: false,
            };
        }
        st.need_valid = true;
        Outcome::Discarded {
            consumed,
            nacked: true,
        }
    }

    /// An ordinary, correctly-sequenced host block (identify's request frame).
    fn identify_frame(seq_nibble: u8) -> Vec<u8> {
        let seq = (seq_nibble & MESSAGE_SEQ_MASK) | crate::core::klippy::frame::MESSAGE_DEST;
        Frame::encode(seq, b"\x01identify offset=0 count=112")
    }

    #[test]
    fn the_firmware_discards_the_request_without_dispatching_it() {
        let request = raw(4);
        let mut st = FwState::default();
        // First failure: consumed exactly through our own trailer, one ack
        // frame in reply, no dispatch.
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        // `next_sequence` never moved — identify starts at its initial value.
        assert_eq!(st.next_sequence, MESSAGE_DEST);
        // A second assignment frame is discarded silently (NEED_VALID holds
        // until a valid block clears it).
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: false
            }
        );
        // And the identify frame that follows dispatches: the pattern leaves
        // the stream exactly where it found it.
        let identify = identify_frame(MESSAGE_DEST);
        assert_eq!(fw_find_block(&identify, &mut st), Outcome::Dispatch);
    }

    #[test]
    fn a_request_sandwiched_before_identify_leaves_identify_intact() {
        // The wire order at connect: our frame, then identify's block. The
        // firmware consumes only ours; identify dispatches on the next call.
        let mut stream = raw(4);
        let identify = identify_frame(MESSAGE_DEST);
        stream.extend_from_slice(&identify);

        let mut st = FwState::default();
        let (first, consumed) = match fw_find_block(&stream, &mut st) {
            outcome @ Outcome::Discarded { consumed, .. } => (outcome, consumed),
            other => panic!("the request must be discarded, got {other:?}"),
        };
        assert!(matches!(first, Outcome::Discarded { nacked: true, .. }));
        assert_eq!(consumed, 10, "only our frame is consumed");
        assert_eq!(
            fw_find_block(&stream[consumed..], &mut st),
            Outcome::Dispatch
        );
    }

    #[test]
    fn the_firmware_would_dispatch_garbage_in_the_pattern_but_never_our_bytes() {
        // The payload is never read on the error path — prove it by putting a
        // syntactically valid command in there with a *correct* CRC over it:
        // the sequence-byte check has already decided, so nothing downstream
        // is reached. (If the dest check were removed, this block would
        // dispatch — which is exactly what the pattern prevents.)
        let mut bytes = raw(4);
        bytes[2..7].copy_from_slice(b"\x00stat"); // msgid 0 + text, CRC fixed:
        let crc = crate::crc16_ccitt!(&bytes[..bytes.len() - MESSAGE_TRAILER_SIZE]);
        let len = bytes.len();
        bytes[len - 3] = (crc >> 8) as u8;
        bytes[len - 2] = crc as u8;
        let mut st = FwState::default();
        assert!(matches!(
            fw_find_block(&bytes, &mut st),
            Outcome::Discarded { .. }
        ));
        assert_eq!(st.next_sequence, MESSAGE_DEST);
    }
}
