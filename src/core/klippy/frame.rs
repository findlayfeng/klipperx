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
// Frame constants (mirrors msgproto.py)
// ===========================================================================

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
/// Mirrors `crc16_ccitt()` in msgproto.py, returning a `u16` CRC value.
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
    ///
    /// Mirrors `check_packet()` in msgproto.py.
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
    ///
    /// Mirrors `encode_msgblock()` in msgproto.py.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_ccitt() {
        let crc = crc16_ccitt!(b"123456789");
        // Custom CRC-16-CCITT variant from msgproto.py
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
}
