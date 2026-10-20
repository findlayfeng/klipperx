//! Pure katapult frame codec: encode a command, decode responses, and scan a
//! byte stream for frames.
//!
//! Layout (all integers little-endian):
//!
//! ```text
//! <01><88><cmd/ack><word_cnt><payload = word_cnt × 4 B><crc16-LE><99><03>
//! ```
//!
//! The CRC16-CCITT (init `0xFFFF`) covers `cmd` through the payload — the two
//! header bytes, the CRC itself and the trailer are excluded. This matches
//! `third_party/katapult/scripts/flashtool.py` (`_build_command`, response
//! verification) and `third_party/katapult/src/command.c`; it does **not**
//! match this repo's klipper frame layer, whose CRC starts at the length byte.

use std::convert::TryInto;

/// Frame start marker.
pub const HEADER: [u8; 2] = [0x01, 0x88];
/// Frame end marker.
pub const TRAILER: [u8; 2] = [0x99, 0x03];

/// Host → bootloader commands.
pub mod command {
    pub const CONNECT: u8 = 0x11;
    pub const SEND_BLOCK: u8 = 0x12;
    pub const EOF: u8 = 0x13;
    pub const REQUEST_BLOCK: u8 = 0x14;
    pub const COMPLETE: u8 = 0x15;
    pub const GET_CANBUS_ID: u8 = 0x16;
}

/// Response codes carried in a frame's third byte.
pub const ACK_SUCCESS: u8 = 0xA0;
pub const NACK: u8 = 0xF1;
pub const ACK_ERROR: u8 = 0xF2;
pub const ACK_BUSY: u8 = 0xF3;

/// Upper bound on a frame payload, checked against the word count a frame
/// declares. The count is one byte, so a declared payload is at most 255 words
/// = 1020 B — a 4096 B cap could never be reached and its check was dead. 256 B
/// is above every frame katapult can build (its `MESSAGE_MAX` is 128 B, so 120 B
/// of payload) with room to spare, and low enough that a bogus header claiming a
/// huge payload slides past instead of holding the scanner for a frame that
/// never comes.
const MAX_PAYLOAD: usize = 256;

/// CRC16-CCITT (init `0xFFFF`, reflected), the algorithm both `flashtool.py`
/// and the katapult firmware use. Verified against an independent
/// line-by-line translation of `flashtool.py` in the tests.
pub fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in buf {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8408
            } else {
                crc >> 1
            };
        }
    }
    crc
}

/// Build one frame. `payload` must be a whole number of words (a multiple of
/// 4 bytes), as on the wire.
pub fn build_frame(code: u8, payload: &[u8]) -> Vec<u8> {
    debug_assert_eq!(payload.len() % 4, 0, "payload must be whole 32-bit words");
    let word_cnt = ((payload.len() / 4) & 0xFF) as u8;
    let mut out = Vec::with_capacity(payload.len() + 8);
    out.extend_from_slice(&HEADER);
    out.push(code);
    out.push(word_cnt);
    out.extend_from_slice(payload);
    let crc = crc16_ccitt(&out[2..]);
    out.extend_from_slice(&crc.to_le_bytes());
    out.extend_from_slice(&TRAILER);
    out
}

/// One decoded frame: the third byte (`cmd` host→device, ack code
/// device→host) plus the word-aligned payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub code: u8,
    pub payload: Vec<u8>,
}

/// A decoded response frame split into its parts: the ack code, the first
/// payload word (the command being acknowledged) and everything after it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResponseFrame {
    pub ack: u8,
    pub orig_cmd: u32,
    pub data: Vec<u8>,
}

impl Frame {
    /// Split a response frame. Returns `None` when the payload is shorter
    /// than the mandatory first word — a malformed response.
    pub fn as_response(&self) -> Option<ResponseFrame> {
        let (orig, data) = self.payload.split_at_checked(4)?;
        Some(ResponseFrame {
            ack: self.code,
            orig_cmd: u32::from_le_bytes(orig.try_into().ok()?),
            data: data.to_vec(),
        })
    }
}

/// Information from a `CONNECT` ack's `data` (payload after `orig_cmd`):
/// protocol version, application start address, block size and MCU identity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectInfo {
    /// e.g. `0x00010002` → (1, 0, 2).
    pub proto_version: (u8, u8, u8),
    pub start_addr: u32,
    pub block_size: u32,
    pub mcu_type: String,
    pub software_version: Option<String>,
}

impl ConnectInfo {
    /// Parse the `data` of a successful `CONNECT` response, following
    /// `flashtool.py`'s `connect_btl()`: the first 12 bytes are
    /// `<proto_ver:4><start_addr:4><block_size:4>`, the rest is a NUL-padded
    /// MCU string (build info from a NUL split once the protocol is ≥ 1.1.0).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 12 {
            return None;
        }
        let proto_version = (data[2], data[1], data[0]);
        let start_addr = u32::from_le_bytes(data[4..8].try_into().ok()?);
        let block_size = u32::from_le_bytes(data[8..12].try_into().ok()?);
        let info = trim_nuls(&data[12..]);
        let (mcu_type, software_version) = if proto_version >= (1, 1, 0) {
            let mut parts = info.splitn(2, |&b| b == 0);
            let mcu = parts.next().unwrap_or(b"");
            let build = parts.next();
            (
                String::from_utf8_lossy(mcu).into_owned(),
                build.map(|b| String::from_utf8_lossy(b).into_owned()),
            )
        } else {
            (String::from_utf8_lossy(info).into_owned(), None)
        };
        Some(Self {
            proto_version,
            start_addr,
            block_size,
            mcu_type,
            software_version,
        })
    }
}

fn trim_nuls(mut bytes: &[u8]) -> &[u8] {
    while bytes.last() == Some(&0) {
        bytes = &bytes[..bytes.len() - 1];
    }
    bytes
}

/// Incremental scanner: feed bytes as they arrive, pull complete frames off
/// the front. Garbage before a header, a bad CRC or a bad trailer all slide
/// the window forward one byte and resume searching, the way `flashtool.py`
/// hunts for `01 88`.
#[derive(Debug, Default, Clone)]
pub struct FrameScanner {
    buf: Vec<u8>,
}

impl FrameScanner {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn feed(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Drop any partial frame (between commands, `flashtool.py` clears its
    /// read buffer the same way).
    pub fn clear(&mut self) {
        self.buf.clear();
    }

    /// Next valid frame, or `None` if more bytes are needed.
    pub fn next_frame(&mut self) -> Option<Frame> {
        loop {
            let start = match self.buf.windows(2).position(|w| w == HEADER) {
                Some(i) => i,
                None => {
                    // No header: keep a lone leading 0x01, it may be the
                    // first half of a header split across two reads.
                    let keep = usize::from(self.buf.last() == Some(&HEADER[0]));
                    let drop = self.buf.len() - keep;
                    if drop > 0 {
                        self.buf.drain(..drop);
                    }
                    return None;
                }
            };
            if start > 0 {
                self.buf.drain(..start);
            }
            if self.buf.len() < 4 {
                return None;
            }
            let payload_len = usize::from(self.buf[3]) * 4;
            if payload_len > MAX_PAYLOAD {
                // Bogus word count: slide past this header.
                self.buf.drain(..1);
                continue;
            }
            let total = payload_len + 8;
            if self.buf.len() < total {
                return None;
            }
            let trailer_ok = self.buf[total - 2] == TRAILER[0] && self.buf[total - 1] == TRAILER[1];
            let want = u16::from_le_bytes([self.buf[total - 4], self.buf[total - 3]]);
            let got = crc16_ccitt(&self.buf[2..total - 4]);
            if trailer_ok && want == got {
                let frame = Frame {
                    code: self.buf[2],
                    payload: self.buf[4..total - 4].to_vec(),
                };
                self.buf.drain(..total);
                return Some(frame);
            }
            // Bad CRC or trailer: slide one byte and keep searching.
            self.buf.drain(..1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN_CONNECT: [u8; 8] = [0x01, 0x88, 0x11, 0x00, 0xF1, 0x7C, 0x99, 0x03];
    const GOLDEN_SEND_BLOCK: [u8; 76] = [
        0x01, 0x88, 0x12, 0x11, 0x00, 0x20, 0x00, 0x08, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
        0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
        0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24,
        0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0xD2, 0xF3, 0x99,
        0x03,
    ];
    const GOLDEN_CONNECT_ACK: [u8; 36] = [
        0x01, 0x88, 0xA0, 0x07, 0x11, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x20, 0x00,
        0x08, 0x40, 0x00, 0x00, 0x00, 0x53, 0x54, 0x4D, 0x33, 0x32, 0x46, 0x31, 0x30, 0x33, 0x00,
        0x00, 0x00, 0xE3, 0x7F, 0x99, 0x03,
    ];
    const GOLDEN_SEND_BLOCK_ACK: [u8; 16] = [
        0x01, 0x88, 0xA0, 0x02, 0x12, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x08, 0x5A, 0xD6, 0x99,
        0x03,
    ];
    const GOLDEN_EOF_ACK: [u8; 16] = [
        0x01, 0x88, 0xA0, 0x02, 0x13, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xE0, 0xE1, 0x99,
        0x03,
    ];
    const GOLDEN_REQUEST_BLOCK_ACK: [u8; 80] = [
        0x01, 0x88, 0xA0, 0x12, 0x14, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x08, 0x00, 0x01, 0x02,
        0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
        0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20,
        0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
        0x3F, 0xF2, 0x6B, 0x99, 0x03,
    ];
    const GOLDEN_COMPLETE_ACK: [u8; 12] = [
        0x01, 0x88, 0xA0, 0x01, 0x15, 0x00, 0x00, 0x00, 0x00, 0x2E, 0x99, 0x03,
    ];
    const GOLDEN_GET_CANBUS_ID_ACK: [u8; 20] = [
        0x01, 0x88, 0xA0, 0x03, 0x16, 0x00, 0x00, 0x00, 0x11, 0xAA, 0x22, 0xBB, 0x33, 0xCC, 0x00,
        0x00, 0x40, 0xED, 0x99, 0x03,
    ];
    const GOLDEN_NACK: [u8; 12] = [
        0x01, 0x88, 0xF1, 0x01, 0x12, 0x00, 0x00, 0x00, 0x6B, 0x3D, 0x99, 0x03,
    ];
    const GOLDEN_BUSY: [u8; 12] = [
        0x01, 0x88, 0xF3, 0x01, 0x12, 0x00, 0x00, 0x00, 0x3D, 0x35, 0x99, 0x03,
    ];
    const GOLDEN_ERROR: [u8; 12] = [
        0x01, 0x88, 0xF2, 0x01, 0x12, 0x00, 0x00, 0x00, 0x16, 0x31, 0x99, 0x03,
    ];

    /// Line-by-line Rust translation of `crc16_ccitt` from
    /// `third_party/katapult/scripts/flashtool.py:42-49` — an implementation
    /// independent of [`crc16_ccitt`], used to cross-check it.
    fn flashtool_crc16(buf: &[u8]) -> u16 {
        let mut crc: u16 = 0xffff;
        for &data0 in buf {
            let mut data = data0 as u16 ^ (crc & 0xff);
            data ^= (data & 0x0f) << 4;
            crc = ((data << 8) | (crc >> 8)) ^ (data >> 4) ^ (data << 3);
        }
        crc & 0xffff
    }

    fn sample_payload() -> Vec<u8> {
        let mut payload = 0x08002000u32.to_le_bytes().to_vec();
        payload.extend((0u8..64).collect::<Vec<u8>>());
        payload
    }

    fn scan_one(bytes: &[u8]) -> Frame {
        let mut scanner = FrameScanner::new();
        scanner.feed(bytes);
        scanner.next_frame().expect("frame should decode")
    }

    #[test]
    fn test_connect_frame_is_eight_golden_bytes() {
        let frame = build_frame(command::CONNECT, &[]);
        assert_eq!(frame.len(), 8);
        assert_eq!(frame, GOLDEN_CONNECT);
    }

    #[test]
    fn test_send_block_frame_matches_golden_vector() {
        let frame = build_frame(command::SEND_BLOCK, &sample_payload());
        assert_eq!(frame.len(), 76);
        assert_eq!(frame, GOLDEN_SEND_BLOCK);
    }

    #[test]
    fn test_crc16_cross_validates_against_flashtool_translation() {
        let mut cases: Vec<Vec<u8>> = vec![
            Vec::new(),
            b"123456789".to_vec(),
            GOLDEN_CONNECT[2..4].to_vec(),
            GOLDEN_SEND_BLOCK[2..72].to_vec(),
            GOLDEN_CONNECT_ACK[2..32].to_vec(),
        ];
        cases.push((0u8..=255).collect::<Vec<u8>>());
        for input in &cases {
            assert_eq!(
                crc16_ccitt(input),
                flashtool_crc16(input),
                "CRC mismatch on {input:?}"
            );
        }
        // Fixed anchors: "123456789" and the two command-body CRCs from the
        // golden frames (connect body → 0x7CF1, send-block body → 0xF3D2).
        assert_eq!(crc16_ccitt(b"123456789"), 0x6F91);
        assert_eq!(flashtool_crc16(b"123456789"), 0x6F91);
        assert_eq!(crc16_ccitt(&GOLDEN_CONNECT[2..4]), 0x7CF1);
        assert_eq!(crc16_ccitt(&GOLDEN_SEND_BLOCK[2..72]), 0xF3D2);
    }

    #[test]
    fn test_decode_connect_ack() {
        let resp = scan_one(&GOLDEN_CONNECT_ACK)
            .as_response()
            .expect("response");
        assert_eq!(resp.ack, ACK_SUCCESS);
        assert_eq!(resp.orig_cmd, u32::from(command::CONNECT));
        let info = ConnectInfo::parse(&resp.data).expect("connect info");
        assert_eq!(info.proto_version, (1, 0, 2));
        assert_eq!(info.start_addr, 0x0800_2000);
        assert_eq!(info.block_size, 64);
        assert_eq!(info.mcu_type, "STM32F103");
        assert_eq!(info.software_version, None);
    }

    #[test]
    fn test_decode_send_block_ack() {
        let resp = scan_one(&GOLDEN_SEND_BLOCK_ACK)
            .as_response()
            .expect("response");
        assert_eq!(resp.ack, ACK_SUCCESS);
        assert_eq!(resp.orig_cmd, u32::from(command::SEND_BLOCK));
        assert_eq!(resp.data, 0x0800_2000u32.to_le_bytes());
    }

    #[test]
    fn test_decode_eof_ack_reports_page_count() {
        let resp = scan_one(&GOLDEN_EOF_ACK).as_response().expect("response");
        assert_eq!(resp.ack, ACK_SUCCESS);
        assert_eq!(resp.orig_cmd, u32::from(command::EOF));
        assert_eq!(
            u32::from_le_bytes(resp.data[..4].try_into().unwrap()),
            2,
            "page count"
        );
    }

    #[test]
    fn test_decode_request_block_ack_carries_address_and_data() {
        let resp = scan_one(&GOLDEN_REQUEST_BLOCK_ACK)
            .as_response()
            .expect("response");
        assert_eq!(resp.ack, ACK_SUCCESS);
        assert_eq!(resp.orig_cmd, u32::from(command::REQUEST_BLOCK));
        assert_eq!(&resp.data[..4], &0x0800_2000u32.to_le_bytes());
        assert_eq!(&resp.data[4..], &(0u8..64).collect::<Vec<u8>>()[..]);
    }

    #[test]
    fn test_decode_complete_ack() {
        let resp = scan_one(&GOLDEN_COMPLETE_ACK)
            .as_response()
            .expect("response");
        assert_eq!(resp.ack, ACK_SUCCESS);
        assert_eq!(resp.orig_cmd, u32::from(command::COMPLETE));
        assert!(resp.data.is_empty());
    }

    #[test]
    fn test_decode_get_canbus_id_ack_carries_uuid() {
        let resp = scan_one(&GOLDEN_GET_CANBUS_ID_ACK)
            .as_response()
            .expect("response");
        assert_eq!(resp.ack, ACK_SUCCESS);
        assert_eq!(resp.orig_cmd, u32::from(command::GET_CANBUS_ID));
        assert_eq!(&resp.data[..6], &[0x11, 0xAA, 0x22, 0xBB, 0x33, 0xCC]);
        assert_eq!(&resp.data[6..], &[0x00, 0x00]);
    }

    #[test]
    fn test_decode_nack_busy_and_error_frames() {
        for (bytes, want_ack) in [
            (&GOLDEN_NACK[..], NACK),
            (&GOLDEN_BUSY[..], ACK_BUSY),
            (&GOLDEN_ERROR[..], ACK_ERROR),
        ] {
            let resp = scan_one(bytes).as_response().expect("response");
            assert_eq!(resp.ack, want_ack);
            assert_eq!(resp.orig_cmd, u32::from(command::SEND_BLOCK));
        }
    }

    #[test]
    fn test_scanner_skips_garbage_before_a_header() {
        let mut scanner = FrameScanner::new();
        scanner.feed(&[0xDE, 0xAD, 0xBE, 0xEF]);
        scanner.feed(&GOLDEN_CONNECT);
        assert_eq!(scanner.next_frame().expect("frame").code, command::CONNECT);
    }

    #[test]
    fn test_scanner_skips_a_frame_with_bad_crc() {
        let mut bad = GOLDEN_SEND_BLOCK_ACK;
        bad[6] ^= 0xFF; // corrupt a payload byte → CRC no longer matches
        let mut scanner = FrameScanner::new();
        scanner.feed(&bad);
        scanner.feed(&GOLDEN_SEND_BLOCK_ACK);
        let frame = scanner.next_frame().expect("good frame after bad one");
        assert_eq!(frame, scan_one(&GOLDEN_SEND_BLOCK_ACK));
        assert!(scanner.next_frame().is_none());
    }

    #[test]
    fn test_scanner_skips_a_frame_with_bad_trailer() {
        let mut bad = GOLDEN_EOF_ACK;
        bad[14] = 0x00; // trailer byte 99 → 00
        let mut scanner = FrameScanner::new();
        scanner.feed(&bad);
        scanner.feed(&GOLDEN_EOF_ACK);
        let frame = scanner.next_frame().expect("good frame after bad one");
        let resp = frame.as_response().expect("response");
        assert_eq!(resp.ack, ACK_SUCCESS);
        assert_eq!(resp.orig_cmd, u32::from(command::EOF));
    }

    #[test]
    fn test_scanner_reassembles_a_frame_split_across_feeds() {
        let mut scanner = FrameScanner::new();
        for chunk in GOLDEN_REQUEST_BLOCK_ACK.chunks(7) {
            assert!(
                scanner.next_frame().is_none(),
                "no frame before the frame is whole"
            );
            scanner.feed(chunk);
        }
        let frame = scanner.next_frame().expect("frame after last feed");
        assert_eq!(frame, scan_one(&GOLDEN_REQUEST_BLOCK_ACK));
    }

    #[test]
    fn test_scanner_clear_drops_partial_frame() {
        let mut scanner = FrameScanner::new();
        scanner.feed(&GOLDEN_REQUEST_BLOCK_ACK[..20]);
        scanner.clear();
        scanner.feed(&GOLDEN_CONNECT);
        assert_eq!(scanner.next_frame().expect("frame").code, command::CONNECT);
    }
}
