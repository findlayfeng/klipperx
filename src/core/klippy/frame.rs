// Frame format handling for Klipper MCU protocol.
//
// This module handles the wire frame format independently of payload content:
//   - Frame length validation
//   - Sequence number handling
//   - CRC-16-CCITT checksum
//   - SYNC byte (0x7e) framing
//
// Payload content is opaque to this module.

// ===========================================================================
// Frame constants
// ===========================================================================

use crate::core::klippy::Payload;

/// Minimum frame length.
pub const MESSAGE_MIN: usize = 5;
/// Maximum frame length.
pub const MESSAGE_MAX: usize = 64;
/// Header size (length byte + sequence byte).
pub const MESSAGE_HEADER_SIZE: usize = 2;
/// Trailer size (2-byte CRC + 1-byte SYNC).
pub const MESSAGE_TRAILER_SIZE: usize = 3;
/// Maximum payload size.
pub const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;

/// Position of length byte in frame.
pub const MESSAGE_POS_LEN: usize = 0;
/// Position of sequence byte in frame.
pub const MESSAGE_POS_SEQ: usize = 1;

/// Position of CRC in trailer.
pub const MESSAGE_TRAILER_CRC: usize = 3;
/// Position of SYNC byte in trailer.
pub const MESSAGE_TRAILER_SYNC: usize = 1;

/// Sequence mask for destination framing.
pub const MESSAGE_SEQ_MASK: u8 = 0x0f;
/// Destination flag in sequence byte.
pub const MESSAGE_DEST: u8 = 0x10;
/// SYNC marker byte.
pub const MESSAGE_SYNC: u8 = 0x7e;

// ===========================================================================
// CRC-16-CCITT
// ===========================================================================

/// Compute CRC-16-CCITT over a variable number of byte slices.
///
/// All referenced slices are concatenated in order for CRC computation.
/// Initial CRC value is always `0xFFFF`.
///
/// # Examples
///
/// ```
/// # use klipperx::crc16_ccitt;
/// let data = b"hello";
/// let crc = crc16_ccitt!(data);
/// let header = b"hdr";
/// let payload = b"pld";
/// let trailer = b"trl";
/// let crc = crc16_ccitt!(header, payload, trailer);
/// ```
#[macro_export]
macro_rules! crc16_ccitt {
    ($($data:expr),+ $(,)?) => {{
        let mut crc: u16 = 0xFFFF;
        $({
            for &byte in $data {
                let mut data_byte = (byte as u16) ^ (crc & 0xFF);
                data_byte ^= (data_byte & 0x0F) << 4;
                crc = ((data_byte << 8) | (crc >> 8)) ^ ((data_byte >> 4) ^ (data_byte << 3));
            }
        })+
        crc
    }};
}

// ===========================================================================
// Frame struct
// ===========================================================================

/// A parsed MCU protocol frame.
///
/// Stores only `seq` and `payload`; the wire format (`raw_bytes()`) is
/// computed on demand via [`encode`].
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Sequence number (lower 4 bits).
    seq: u8,
    /// Payload data between header and trailer.
    payload: Vec<u8>,
}

impl Into<Payload> for Frame {
    fn into(self) -> Payload {
        Payload::from_raw(self.payload)
    }
}

impl Frame {
    /// Return the frame length, computed from payload size plus header and trailer.
    pub fn frame_length(&self) -> usize {
        MESSAGE_MIN + self.payload.len()
    }

    /// Check if a raw byte buffer is a valid frame.
    ///
    /// Returns:
    ///   - `Ok(())` if valid
    ///   - `Err(reason)` if invalid
    pub fn check(raw: &[u8]) -> Result<(), &'static str> {
        // Need at least a minimum frame
        if raw.len() < MESSAGE_MIN {
            return Err("frame too short");
        }

        let msglen = raw[MESSAGE_POS_LEN] as usize;

        // Length must be within valid range
        if msglen < MESSAGE_MIN || msglen > MESSAGE_MAX {
            return Err("invalid frame length");
        }

        // Sequence byte must have DEST flag
        let msgseq = raw[MESSAGE_POS_SEQ];
        if (msgseq & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
            return Err("invalid sequence byte");
        }

        // Buffer must contain full frame
        if raw.len() < msglen {
            return Err("incomplete frame");
        }

        // SYNC byte must be present at trailer position
        if raw[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
            return Err("missing SYNC byte");
        }

        // CRC check
        let expected_crc = crc16_ccitt!(&raw[..msglen - MESSAGE_TRAILER_SIZE]);
        let actual_crc = ((raw[msglen - MESSAGE_TRAILER_CRC] as u16) << 8)
            | (raw[msglen - MESSAGE_TRAILER_CRC + 1] as u16);
        if expected_crc != actual_crc {
            return Err("CRC mismatch");
        }

        Ok(())
    }

    /// Parse a frame from raw bytes.
    ///
    /// Returns `Some(Frame)` if the buffer contains a valid frame, `None` otherwise.
    pub fn parse(raw: &[u8]) -> Option<Frame> {
        Self::check(raw).ok()?;
        let msglen = raw[MESSAGE_POS_LEN] as usize;
        let seq = raw[MESSAGE_POS_SEQ] & MESSAGE_SEQ_MASK;
        let payload = raw[MESSAGE_HEADER_SIZE..msglen - MESSAGE_TRAILER_SIZE].to_vec();
        Some(Self::new(seq, payload))
    }

    /// Create a Frame from a sequence number and payload.
    pub fn new(seq: u8, payload: Vec<u8>) -> Self {
        Self { seq, payload }
    }

    /// Get the sequence number (lower 4 bits of the sequence byte).
    pub fn seq(&self) -> u8 {
        self.seq
    }

    /// Get the payload data (between header and trailer).
    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    /// Encode this frame into raw wire bytes (header + payload + trailer).
    pub fn raw_bytes(&self) -> Vec<u8> {
        Self::encode(self.seq, &self.payload)
    }

    /// Encode a frame with the given sequence number and payload.
    pub fn encode(seq: u8, payload: &[u8]) -> Vec<u8> {
        let msglen = MESSAGE_MIN + payload.len();
        let seq_byte = (seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
        let mut out: Vec<u8> = vec![msglen as u8, seq_byte];
        out.extend_from_slice(payload);

        // CRC-16 over [length, seq, payload]
        let crc = crc16_ccitt!(&out);
        out.push((crc >> 8) as u8);
        out.push(crc as u8);

        // SYNC byte
        out.push(MESSAGE_SYNC);

        out
    }
}

// ===========================================================================
// Stream reassembly
// ===========================================================================

/// Reassembles frames out of a byte stream.
///
/// [`Frame::parse`] handles one frame at a time; a device that transports bytes
/// needs the stateful half of the same job, because a read can stop mid-frame,
/// carry several frames, or contain bytes that were corrupted on the way. Every
/// byte-stream device shares this type — the serial port and the host library
/// today, a socket next — so no device has to invent its own idea of where a
/// frame ends.
///
/// The recovery policy is Klipper's own (`msgblock_check` in
/// `klippy/chelper/msgblock.c`), and it is about where to look next rather than
/// how much to guess: a frame ends with a SYNC byte, so after anything invalid the
/// next SYNC is the cheapest safe place to start again. Dropping one byte at a
/// time would be worse — the following length byte can then demand more bytes than
/// will ever arrive, stalling a perfectly good frame behind it.
///
/// Two consequences of that policy, both inherited from Klipper:
///
/// * junk before a SYNC is discarded together with it, so a frame that arrives
///   while the stream is desynchronised can be skipped as well;
/// * a read containing no SYNC at all is dropped entirely, and the stream stays
///   desynchronised until a SYNC shows up.
///
/// # Examples
///
/// ```
/// # use klipperx::core::klippy::frame::{Frame, FrameStream};
/// let mut stream = FrameStream::new();
/// stream.push(&Frame::encode(1, b"hello"));
/// assert_eq!(stream.next(), Some(Frame::new(1, b"hello".to_vec())));
/// assert_eq!(stream.next(), None); // nothing buffered, nothing claimed
/// ```
#[derive(Debug, Default)]
pub struct FrameStream {
    buffer: Vec<u8>,
    /// Set when an error was seen and no SYNC followed it: until the next SYNC,
    /// every byte in the stream is garbage.
    needs_sync: bool,
}

impl FrameStream {
    /// An empty stream.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add bytes as they arrive.
    pub fn push(&mut self, bytes: &[u8]) {
        self.buffer.extend_from_slice(bytes);
    }

    /// Take the next complete frame, if one has arrived.
    ///
    /// Returns `None` both when more bytes are needed and when the buffer is
    /// unusable, so callers keep feeding the stream until a frame appears or the
    /// input ends.
    pub fn next(&mut self) -> Option<Frame> {
        loop {
            if self.needs_sync {
                self.skip_to_sync();
            }
            if self.buffer.len() < MESSAGE_MIN {
                return None;
            }

            let length = self.buffer[MESSAGE_POS_LEN] as usize;
            if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
                self.skip_to_sync();
                continue;
            }
            if self.buffer.len() < length {
                return None; // the rest of the frame is still in flight
            }

            match Frame::parse(&self.buffer[..length]) {
                Some(frame) => {
                    self.buffer.drain(..length);
                    return Some(frame);
                }
                // SYNC byte or CRC did not check out.
                None => self.skip_to_sync(),
            }
        }
    }

    /// Discard everything up to and including the next SYNC byte.
    ///
    /// When there is none, the buffer is entirely garbage: drop it and stay
    /// desynchronised, so the next read is scanned for a SYNC instead of being
    /// appended to bytes that can never frame anything.
    fn skip_to_sync(&mut self) {
        match self.buffer.iter().position(|&byte| byte == MESSAGE_SYNC) {
            Some(position) => {
                self.buffer.drain(..=position);
                self.needs_sync = false;
            }
            None => {
                self.buffer.clear();
                self.needs_sync = true;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_ccitt() {
        let crc = crc16_ccitt!(b"123456789");
        // Custom CRC-16-CCITT variant
        assert_eq!(crc, 0x6F91);
    }

    #[test]
    fn test_frame_encode_decode() {
        let payload = b"hello world";
        let frame = Frame::encode(5, payload);

        // Check frame structure
        assert!(frame.len() >= MESSAGE_MIN);
        assert_eq!(frame[MESSAGE_POS_LEN] as usize, frame.len());
        assert_eq!(frame[MESSAGE_POS_SEQ] & !MESSAGE_SEQ_MASK, MESSAGE_DEST);
        assert_eq!(frame[frame.len() - 1], MESSAGE_SYNC);

        // Parse it back
        let parsed = Frame::parse(&frame).unwrap();
        assert_eq!(parsed.seq(), 5);
        assert_eq!(parsed.payload(), payload);
        assert_eq!(parsed.frame_length(), frame.len());
    }

    #[test]
    fn test_frame_check_invalid_length() {
        assert!(Frame::check(&[0x02, 0x00]).is_err()); // too short
        assert!(Frame::check(&[0x01, 0x10, 0x00, 0x00, 0x7e]).is_err()); // length < MIN
    }

    #[test]
    fn test_frame_check_bad_sync() {
        let mut frame = Frame::encode(0, b"test");
        // Corrupt SYNC byte
        let end = frame.len();
        frame[end - 1] = 0xFF;
        assert!(Frame::check(&frame).is_err());
    }

    #[test]
    fn test_frame_check_bad_crc() {
        let mut frame = Frame::encode(0, b"test");
        // Corrupt a payload byte
        frame[MESSAGE_HEADER_SIZE] ^= 0xFF;
        assert!(Frame::check(&frame).is_err());
    }

    #[test]
    fn test_frame_check_bad_seq() {
        let mut frame = Frame::encode(0, b"test");
        // Remove DEST flag
        frame[MESSAGE_POS_SEQ] &= MESSAGE_SEQ_MASK;
        assert!(Frame::check(&frame).is_err());
    }

    #[test]
    fn test_frame_incomplete() {
        let full_frame = Frame::encode(0, b"hello");
        // Truncate the frame
        assert!(Frame::parse(&full_frame[..5]).is_none());
    }

    #[test]
    fn test_frame_multiple_in_buffer() {
        let f1 = Frame::encode(1, b"abc");
        let f2 = Frame::encode(2, b"def");
        let mut buf = Vec::new();
        buf.extend_from_slice(&f1);
        buf.extend_from_slice(&f2);

        // Parse first frame
        let frame1 = Frame::parse(&buf).unwrap();
        assert_eq!(frame1.seq(), 1);
        assert_eq!(frame1.payload(), b"abc");
        let consumed1 = frame1.frame_length();
        assert_eq!(consumed1, f1.len());

        // Parse second frame from remaining
        let frame2 = Frame::parse(&buf[consumed1..]).unwrap();
        assert_eq!(frame2.seq(), 2);
        assert_eq!(frame2.payload(), b"def");
        let consumed2 = frame2.frame_length();
        assert_eq!(consumed2, f2.len());
    }

    // -----------------------------------------------------------------------
    // FrameStream
    // -----------------------------------------------------------------------

    /// Feed bytes to a stream the way a device would: in chunks, pulling out
    /// whatever that completes.
    fn frames_from(chunks: &[&[u8]]) -> Vec<Frame> {
        let mut stream = FrameStream::new();
        let mut frames = Vec::new();
        for chunk in chunks {
            stream.push(chunk);
            while let Some(frame) = stream.next() {
                frames.push(frame);
            }
        }
        frames
    }

    #[test]
    fn test_frame_stream_waits_for_a_whole_frame() {
        let encoded = Frame::encode(3, b"hello");

        // Nothing until the last byte arrives.
        for split in 1..encoded.len() {
            let frames = frames_from(&[&encoded[..split]]);
            assert!(frames.is_empty(), "framed at {split} of {}", encoded.len());
        }
        assert_eq!(
            frames_from(&[&encoded]),
            vec![Frame::new(3, b"hello".to_vec())]
        );
    }

    #[test]
    fn test_frame_stream_splits_a_concatenated_stream() {
        let first = Frame::encode(1, b"abc");
        let second = Frame::encode(2, b"def");
        let mut stream = first.clone();
        stream.extend_from_slice(&second);

        let expected = vec![
            Frame::new(1, b"abc".to_vec()),
            Frame::new(2, b"def".to_vec()),
        ];
        assert_eq!(frames_from(&[&stream]), expected);

        // Same result when the read boundary falls inside the first frame.
        let split = first.len() - 1;
        assert_eq!(frames_from(&[&stream[..split], &stream[split..]]), expected);
    }

    #[test]
    fn test_frame_stream_resynchronizes_on_the_next_sync() {
        let good = Frame::encode(4, b"payload");
        let mut stream = vec![0xff, 0x00, MESSAGE_SYNC]; // not a frame
        stream.extend_from_slice(&good);

        assert_eq!(
            frames_from(&[&stream]),
            vec![Frame::new(4, b"payload".to_vec())]
        );
    }

    #[test]
    fn test_frame_stream_drops_a_corrupt_frame_and_keeps_the_next() {
        let mut corrupt = Frame::encode(5, b"payload");
        corrupt[MESSAGE_MIN] ^= 0xff; // flip a payload byte: CRC no longer matches
        let good = Frame::encode(6, b"next");

        let mut stream = corrupt;
        stream.extend_from_slice(&good);

        // The corrupt frame is skipped up to its own trailing SYNC, so the frame
        // behind it survives — worth pinning, because dropping one byte at a time
        // instead would resynchronise on a length byte that may never come.
        assert_eq!(
            frames_from(&[&stream]),
            vec![Frame::new(6, b"next".to_vec())]
        );
    }

    #[test]
    fn test_frame_stream_survives_a_read_without_sync() {
        let good = Frame::encode(7, b"later");
        let mut stream = FrameStream::new();

        // Too short to judge: nothing is framed and nothing is discarded.
        stream.push(&[0x01, 0x02, 0x03]);
        assert_eq!(stream.next(), None);
        assert!(!stream.needs_sync);

        // A read long enough to hold a frame but with an impossible length byte,
        // and no SYNC to resynchronise on, is dropped whole: the stream stays
        // desynchronised instead of trying to frame the junk.
        stream.push(&[0xff, 0x00, 0x01, 0x02, 0x03]);
        assert_eq!(stream.next(), None);
        assert!(stream.needs_sync);

        // The next SYNC ends the resynchronisation: bytes before it are dropped,
        // and a frame read after it is framed normally.
        stream.push(&good);
        assert_eq!(stream.next(), None, "expected to skip the first frame");

        stream.push(&good);
        assert_eq!(stream.next(), Some(Frame::new(7, b"later".to_vec())));
        assert!(!stream.needs_sync);
    }
}
