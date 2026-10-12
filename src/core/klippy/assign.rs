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
//! ever computed, before dispatch could run. The error path (`:311-332`)
//! discards bytes up to and including the next `0x7e`, which is our own
//! trailer: exactly this frame is consumed, nothing behind it. Consequences,
//! all of them by construction:
//!
//! * no dispatch → no `shutdown("Invalid command")`, no command executed;
//! * `next_sequence` untouched → the identify exchange that follows starts
//!   exactly where it would have;
//! * at most **one** empty ack-shaped frame in reply (`need_sync` sends a nak
//!   only when `CF_NEED_VALID` was clear — `:327-331`) — an empty frame stamped
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
//! * The frame-header addressing scheme this feeds has its codec here (below),
//!   but is **not wired to any transport yet**; today every frame on the wire is
//!   the current format, assigned or not. A device that was never assigned an
//!   id communicates exactly as before.
//!
//! # The serial multiplexing protocol
//!
//! The frame-header scheme this module's id assignment feeds: every frame is
//! prefixed with its sender's arbitration id, so one byte stream can carry
//! several MCUs. The codec below is pure — nothing on the wire uses it yet.
//!
//! * **Arbitration id** — the mapping CAN already uses
//!   (`interface/devices/canserial.rs`): a device's *listener* id is
//!   `0x100 + 2 * mcu_id`, its *answer* id is one higher, both for `mcu_id` in
//!   `1..=MAX_MCU_ID` (`0xff`). [`arb_id`] and [`reply_arb_id`] build them,
//!   [`mcu_id_from_arb`] decodes either direction to the same `mcu_id`, and
//!   [`is_reply_arb`] tells the directions apart.
//! * **Frame header** — a bare Klipper VLQ of the arbitration id, *not* the
//!   `KXID` envelope below. Over the whole assigned range (`0x102..=0x2ff`,
//!   ids `1..=255` in both directions) the header is always exactly two bytes.
//!   [`encode_header`] / [`decode_header`] are the codec; the VLQ itself is the
//!   message system's own ([`crate::core::klippy::msg::proto`]), not a copy.
//! * **Assignment unit** — the header, then a `KXID` pseudo-frame carrying
//!   [`MAGIC`] and nothing else:
//!
//!   ```text
//!   [vlq(arb)][len=9][0x20]["KXID"][crc16 hi][crc16 lo][0x7e]
//!   ```
//!
//!   The id appears **once**, in the header; the pseudo-frame body is fixed
//!   and identical in both directions. [`assignment_request`] addresses a
//!   device, [`assignment_reply`] is the answer it sends, and
//!   [`parse_assignment_ack`] recognizes that answer.
//!
//! A VLQ of an id may end in `0x7e` (`mcu_id = 63, 127, 191, 255` all do). That
//! is not a problem: frame *content* is allowed to contain SYNC — upstream,
//! `docs/Protocol.md:178-183`: “a sync character is not exclusive to the
//! framing and may be present in the message block content” — so a SYNC inside
//! the header is ordinary data.
//!
//! [`FrameStream`]: crate::core::klippy::frame::FrameStream

use crate::core::klippy::config::mcu::MAX_MCU_ID;
use crate::core::klippy::frame::{
    Frame, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_POS_LEN, MESSAGE_POS_SEQ,
    MESSAGE_SYNC, MESSAGE_TRAILER_SIZE, MESSAGE_TRAILER_SYNC,
};
use crate::core::klippy::msg::proto::{vlq_decode, vlq_encode, VlqDecode};

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
// Serial multiplexing protocol
// ===========================================================================

/// Base of the arbitration-id range, matching CAN's node mapping
/// (`interface/devices/canserial.rs`): `0x100 + 2 * mcu_id`.
const ARB_BASE: u32 = 0x100;

/// The arbitration id an MCU listens on: `0x100 + 2 * mcu_id`.
///
/// `None` for an id outside `1..=MAX_MCU_ID` (`0xff`), so a caller cannot ask
/// for an id the wire format cannot carry.
pub fn arb_id(mcu_id: u32) -> Option<u32> {
    if (1..=MAX_MCU_ID).contains(&mcu_id) {
        Some(ARB_BASE + 2 * mcu_id)
    } else {
        None
    }
}

/// The arbitration id an MCU answers on: [`arb_id`] plus one.
pub fn reply_arb_id(mcu_id: u32) -> Option<u32> {
    arb_id(mcu_id).map(|arb| arb + 1)
}

/// Decode an arbitration id back to the `mcu_id` it names.
///
/// Both directions decode to the same id: the listener id (`0x100 + 2n`, even)
/// and the answer id (`0x101 + 2n`, odd) differ only in the low bit, so
/// `offset / 2` recovers `n` either way. `None` outside the assigned range —
/// in particular an id that would name `0` (`0x100`, `0x101`) or one past
/// `MAX_MCU_ID` (`0x300` and up).
pub fn mcu_id_from_arb(arb: u32) -> Option<u32> {
    let offset = arb.checked_sub(ARB_BASE)?;
    if !(2..=2 * MAX_MCU_ID + 1).contains(&offset) {
        return None;
    }
    Some(offset / 2)
}

/// Whether `arb` is an answer id — the odd half of the mapping.
///
/// The device→host direction uses [`reply_arb_id`]; this is how a router
/// reading a header tells that apart from the host→device [`arb_id`].
pub fn is_reply_arb(arb: u32) -> bool {
    arb.checked_sub(ARB_BASE)
        .is_some_and(|offset| offset % 2 == 1)
}

/// Encode a frame header: the arbitration id as a bare Klipper VLQ.
///
/// Over the assigned range (`0x102..=0x2ff`) this is always two bytes.
pub fn encode_header(arb: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(2);
    vlq_encode(arb, &mut out);
    out
}

/// The outcome of reading a frame header off the front of a byte slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderScan {
    /// A whole arbitration id, occupying `consumed` bytes.
    Complete { arb: u32, consumed: usize },
    /// The slice ends inside the id; more bytes could complete it.
    NeedMore,
    /// The bytes cannot be a `u32` VLQ (more than five continuation bytes).
    Invalid,
}

/// Decode the frame header at the front of `buf`.
pub fn decode_header(buf: &[u8]) -> HeaderScan {
    match vlq_decode(buf) {
        VlqDecode::Complete { value, consumed } => HeaderScan::Complete {
            arb: value,
            consumed,
        },
        VlqDecode::NeedMore => HeaderScan::NeedMore,
        VlqDecode::Invalid => HeaderScan::Invalid,
    }
}

/// The pseudo-frame an assignment unit carries after its header: the `KXID`
/// pattern with a payload of [`MAGIC`] alone.
///
/// The id is *not* in here — it is the unit's header — so the same body goes in
/// both directions, and the frame is
/// `[len=9][0x20]["KXID"][crc16 hi][crc16 lo][0x7e]`.
fn assignment_frame() -> Vec<u8> {
    Frame::with_wire_seq(ASSIGN_SEQ_BYTE, MAGIC.to_vec()).raw_bytes()
}

/// The assignment unit addressed to `mcu_id`: the header on its listener
/// arbitration id, then the pseudo-frame.
pub fn assignment_request(mcu_id: u32) -> Option<Vec<u8>> {
    let mut unit = encode_header(arb_id(mcu_id)?);
    unit.extend_from_slice(&assignment_frame());
    Some(unit)
}

/// The answer `mcu_id` sends for [`assignment_request`]: the header on its
/// answer arbitration id, then the same pseudo-frame.
pub fn assignment_reply(mcu_id: u32) -> Option<Vec<u8>> {
    let mut unit = encode_header(reply_arb_id(mcu_id)?);
    unit.extend_from_slice(&assignment_frame());
    Some(unit)
}

/// Recognize an assignment answer and return the id it names.
///
/// Everything must be self-consistent: a whole VLQ header whose arbitration id
/// is the reply direction ([`is_reply_arb`]) and decodes to an [`mcu_id`], then
/// a pseudo-frame whose length byte matches the bytes present, whose sequence
/// byte is [`ASSIGN_SEQ_BYTE`], whose trailer and CRC check out, and whose
/// payload is exactly [`MAGIC`]. A request (even header) is rejected by the
/// direction check.
///
/// [`mcu_id`]: mcu_id_from_arb
pub fn parse_assignment_ack(unit: &[u8]) -> Option<u32> {
    let (arb, consumed) = match decode_header(unit) {
        HeaderScan::Complete { arb, consumed } => (arb, consumed),
        HeaderScan::NeedMore | HeaderScan::Invalid => return None,
    };
    if !is_reply_arb(arb) {
        return None;
    }
    let mcu_id = mcu_id_from_arb(arb)?;
    let frame = &unit[consumed..];
    if frame.len() < MESSAGE_MIN {
        return None;
    }
    let len = frame[MESSAGE_POS_LEN] as usize;
    if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&len) || frame.len() != len {
        return None;
    }
    if frame[MESSAGE_POS_SEQ] != ASSIGN_SEQ_BYTE {
        return None;
    }
    if frame[len - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
        return None;
    }
    let expected = crate::crc16_ccitt!(&frame[..len - MESSAGE_TRAILER_SIZE]);
    let actual = ((frame[len - 3] as u16) << 8) | (frame[len - 2] as u16);
    if expected != actual {
        return None;
    }
    let payload = &frame[MESSAGE_HEADER_SIZE..len - MESSAGE_TRAILER_SIZE];
    if payload != &MAGIC[..] {
        return None;
    }
    Some(mcu_id)
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
    // (`third_party/klipper/src/command.c:276-333`), so the compatibility
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
        // `:298` — a structure-valid block clears NEED_VALID.
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
        // `:311-316` — a leading SYNC is dropped alone, silently.
        if buf.first() == Some(&MESSAGE_SYNC) {
            return Outcome::Discarded {
                consumed: 1,
                nacked: false,
            };
        }
        // `:317` then fall into need_sync (`:318`).
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
        // is reached. (Removing the dest check would not make it dispatch
        // either: the sequence check runs next, `next_sequence` only ever holds
        // `0x10..=0x1f` and never `0x20`, so the block would be nakaed — both
        // checks refuse it.)
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

    #[test]
    fn a_valid_block_rearms_the_one_nak_gate() {
        // `sync_state &= ~CF_NEED_VALID` (`command.c:298`) is the only thing
        // that re-arms the one-time nak: after it, the next structural error
        // nakks again, and the one after that is silent again.
        let mut st = FwState::default();
        let request = raw(4);
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: true, .. }
        ));
        // A valid identify block clears the flag on its way to dispatch.
        assert_eq!(
            fw_find_block(&identify_frame(MESSAGE_DEST), &mut st),
            Outcome::Dispatch
        );
        // So the *next* structural error nakks once more — re-armed …
        let request = raw(5);
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: true, .. }
        ));
        // … and damping resumes immediately after.
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: false, .. }
        ));
        assert_eq!(st.next_sequence, MESSAGE_DEST | 1);
    }

    #[test]
    fn a_frame_split_across_reads_is_still_discarded_once_complete() {
        // Real links deliver bytes in arbitrary slices: the firmware buffers
        // (`need_more_data`, silent — `command.c:281-282`) until MESSAGE_MIN
        // bytes are present, then runs the same checks on the accumulated
        // buffer — our frame is discarded whole on the completing read.
        let request = raw(4);
        let mut st = FwState::default();
        assert_eq!(fw_find_block(&request[..4], &mut st), Outcome::NeedMore);
        let mut buffered = request[..4].to_vec();
        buffered.extend_from_slice(&request[4..]);
        assert_eq!(
            fw_find_block(&buffered, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        assert_eq!(st.next_sequence, MESSAGE_DEST);
    }

    #[test]
    fn a_request_after_identify_is_discarded_without_disturbing_the_session() {
        // The other half of the README's “before and after identify” claim:
        // arriving *after* a valid block, the flag was just cleared by it, so
        // this error nakks once (the harmless empty frame), then damps — never
        // dispatch, and the sequence identify advanced to stays put.
        let mut st = FwState::default();
        assert_eq!(
            fw_find_block(&identify_frame(MESSAGE_DEST), &mut st),
            Outcome::Dispatch
        );
        assert_eq!(st.next_sequence, MESSAGE_DEST | 1);

        let request = raw(4);
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        assert_eq!(
            st.next_sequence,
            MESSAGE_DEST | 1,
            "the request must not move the sequence identify left behind"
        );
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: false, .. }
        ));
    }

    // -----------------------------------------------------------------------
    // Serial multiplexing protocol
    // -----------------------------------------------------------------------

    #[test]
    fn arb_ids_map_like_can_and_round_trip() {
        // The mapping CAN uses (`interface/devices/canserial.rs`).
        assert_eq!(arb_id(1), Some(0x102));
        assert_eq!(reply_arb_id(1), Some(0x103));
        assert_eq!(arb_id(255), Some(0x2fe));
        assert_eq!(reply_arb_id(255), Some(0x2ff));

        // Both directions decode to the same id, over the whole range.
        for id in 1..=MAX_MCU_ID {
            let tx = arb_id(id).unwrap();
            let rx = reply_arb_id(id).unwrap();
            assert_eq!(mcu_id_from_arb(tx), Some(id), "listener id for {id}");
            assert_eq!(mcu_id_from_arb(rx), Some(id), "answer id for {id}");
            assert!(!is_reply_arb(tx), "listener id for {id} is not a reply");
            assert!(is_reply_arb(rx), "answer id for {id} is a reply");
        }

        // Ids that would name 0 (`0x100`, `0x101`) and ids past the bound
        // (`0x300` up) do not decode.
        assert_eq!(mcu_id_from_arb(0x100), None);
        assert_eq!(mcu_id_from_arb(0x101), None);
        assert_eq!(mcu_id_from_arb(0x300), None);
        assert_eq!(mcu_id_from_arb(0x301), None);
        assert_eq!(mcu_id_from_arb(0), None);

        // Out-of-range ids cannot be built either.
        assert_eq!(arb_id(0), None);
        assert_eq!(arb_id(256), None);
        assert_eq!(reply_arb_id(0), None);
        assert_eq!(reply_arb_id(256), None);

        // The direction flag, on a concrete pair and a below-base id.
        assert!(is_reply_arb(0x103));
        assert!(!is_reply_arb(0x102));
        assert!(!is_reply_arb(0));
    }

    #[test]
    fn headers_are_two_bytes_across_the_whole_id_range() {
        for id in 1..=MAX_MCU_ID {
            for arb in [arb_id(id).unwrap(), reply_arb_id(id).unwrap()] {
                let header = encode_header(arb);
                assert_eq!(header.len(), 2, "arb {arb:#x}");
                assert_eq!(
                    decode_header(&header),
                    HeaderScan::Complete { arb, consumed: 2 }
                );
            }
        }
        // The endpoints of the range are the two-byte values below.
        assert_eq!(encode_header(0x102), vec![0x82, 0x02]);
        assert_eq!(encode_header(0x2ff), vec![0x85, 0x7f]);

        // A truncated header waits for the rest instead of guessing.
        assert_eq!(decode_header(&[]), HeaderScan::NeedMore);
        assert_eq!(decode_header(&[0x82]), HeaderScan::NeedMore);
        let full = encode_header(0x2ff);
        assert_eq!(decode_header(&full[..1]), HeaderScan::NeedMore);
        assert_eq!(
            decode_header(&full),
            HeaderScan::Complete {
                arb: 0x2ff,
                consumed: 2
            }
        );
    }

    #[test]
    fn boundary_id_headers_may_contain_a_sync_byte() {
        // The protocol allows 0x7e in frame *content* (`docs/Protocol.md:178-183`:
        // “a sync character is not exclusive to the framing and may be present
        // in the message block content”). A VLQ whose last byte is 0x7e is
        // therefore acceptable, and these values pin that behavior on purpose.
        for (id, expected) in [
            (63u32, [0x82u8, 0x7e]),
            (127, [0x83, 0x7e]),
            (191, [0x84, 0x7e]),
            (255, [0x85, 0x7e]),
        ] {
            let arb = arb_id(id).unwrap();
            let header = encode_header(arb);
            assert_eq!(header, expected, "id {id}");
            assert!(
                header.contains(&MESSAGE_SYNC),
                "id {id} should exercise the 0x7e content byte"
            );
            assert_eq!(
                decode_header(&header),
                HeaderScan::Complete { arb, consumed: 2 }
            );
        }
    }

    #[test]
    fn assignment_units_round_trip_through_the_ack_parser() {
        let request = assignment_request(42).unwrap();
        let reply = assignment_reply(42).unwrap();

        // Request: the listener header, then the fixed pseudo-frame.
        assert_eq!(&request[..2], encode_header(arb_id(42).unwrap()).as_slice());
        assert_eq!(request.len(), 2 + 9);
        assert_eq!(request[2], 9); // frame length byte
        assert_eq!(request[3], ASSIGN_SEQ_BYTE);
        assert_eq!(&request[4..8], MAGIC);
        assert_eq!(*request.last().unwrap(), MESSAGE_SYNC);

        // Reply: the answer header; the body is byte-for-byte the request's.
        assert_eq!(
            &reply[..2],
            encode_header(reply_arb_id(42).unwrap()).as_slice()
        );
        assert_eq!(&reply[2..], &request[2..]);

        // The parser recognizes the reply and rejects the request's direction.
        assert_eq!(parse_assignment_ack(&reply), Some(42));
        assert_eq!(parse_assignment_ack(&request), None);

        for id in [1u32, 42, 200, 255] {
            assert_eq!(
                parse_assignment_ack(&assignment_reply(id).unwrap()),
                Some(id),
                "id {id}"
            );
        }

        // Out-of-range ids have no unit at all.
        assert_eq!(assignment_request(0), None);
        assert_eq!(assignment_request(256), None);
        assert_eq!(assignment_reply(0), None);
        assert_eq!(assignment_reply(256), None);
    }

    #[test]
    fn a_malformed_assignment_ack_is_rejected() {
        let reply = assignment_reply(7).unwrap();
        assert_eq!(parse_assignment_ack(&reply), Some(7));

        // Bad CRC — flip the trailer's high CRC byte.
        let mut bad_crc = reply.clone();
        let crc_at = bad_crc.len() - 3;
        bad_crc[crc_at] ^= 0xff;
        assert_eq!(parse_assignment_ack(&bad_crc), None);

        // Wrong sequence byte.
        let mut bad_seq = reply.clone();
        bad_seq[2 + MESSAGE_POS_SEQ] = 0x10;
        assert_eq!(parse_assignment_ack(&bad_seq), None);

        // Truncation: the trailer's SYNC removed, and the header cut short.
        assert_eq!(parse_assignment_ack(&reply[..reply.len() - 1]), None);
        assert_eq!(parse_assignment_ack(&reply[..1]), None);

        // No MAGIC: a structurally valid pseudo-frame with different payload.
        let mut wrong_magic = encode_header(reply_arb_id(7).unwrap());
        wrong_magic.extend_from_slice(
            &Frame::with_wire_seq(ASSIGN_SEQ_BYTE, b"XXXX".to_vec()).raw_bytes(),
        );
        assert_eq!(parse_assignment_ack(&wrong_magic), None);

        // Wrong trailer SYNC byte.
        let mut bad_trailer = reply.clone();
        *bad_trailer.last_mut().unwrap() = 0x00;
        assert_eq!(parse_assignment_ack(&bad_trailer), None);

        // Trailing bytes after the frame: not a self-consistent unit.
        let mut extra = reply.clone();
        extra.push(0x00);
        assert_eq!(parse_assignment_ack(&extra), None);
    }
}
