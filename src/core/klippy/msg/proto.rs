use super::super::frame::MESSAGE_PAYLOAD_MAX;
use super::error::{MsgError, MsgResult};

/// Parameter type enum.
#[derive(Debug, Clone, PartialEq, Copy, Eq, Hash)]
pub enum ArgType {
    UInt8,
    UInt16,
    Int16,
    UInt32,
    Int32,
    Str,
    Bytes,
}

/// Parameter value enum carrying the actual parameter data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ArgValue {
    UInt8(u8),
    UInt16(u16),
    Int16(i16),
    UInt32(u32),
    Int32(i32),
    Str(String),
    Bytes(Vec<u8>),
}

impl ArgValue {
    /// Get the ArgType corresponding to this ArgValue.
    pub fn arg_type(&self) -> ArgType {
        match self {
            ArgValue::UInt8(_) => ArgType::UInt8,
            ArgValue::UInt16(_) => ArgType::UInt16,
            ArgValue::Int16(_) => ArgType::Int16,
            ArgValue::UInt32(_) => ArgType::UInt32,
            ArgValue::Int32(_) => ArgType::Int32,
            ArgValue::Str(_) => ArgType::Str,
            ArgValue::Bytes(_) => ArgType::Bytes,
        }
    }

    /// Attempt to convert this ArgValue to the target ArgType.
    ///
    /// The conversion is only performed when it is lossless:
    /// - Any integer → any integer, provided the value is representable in the
    ///   target type's range (this covers both widening and checked narrowing;
    ///   sign-losing or truncating conversions are rejected).
    /// - `Str` ↔ `Bytes`, which share the same wire encoding (Bytes → Str also
    ///   requires valid UTF-8).
    ///
    /// Everything else (numbers ↔ strings/buffers) is rejected with `None`.
    pub fn try_convert_to(&self, target: ArgType) -> Option<ArgValue> {
        // Same type — no conversion needed
        if self.arg_type() == target {
            return Some(self.clone());
        }

        // `%s` and `%.*s`/`%*s` share the same length-prefixed wire encoding.
        match (self, target) {
            (ArgValue::Str(s), ArgType::Bytes) => {
                return Some(ArgValue::Bytes(s.as_bytes().to_vec()));
            }
            (ArgValue::Bytes(b), ArgType::Str) => {
                return std::str::from_utf8(b)
                    .ok()
                    .map(|s| ArgValue::Str(s.to_string()));
            }
            _ => {}
        }

        // Numeric conversions, range checked so no value is silently wrapped.
        let n = match self {
            ArgValue::UInt8(v) => i64::from(*v),
            ArgValue::UInt16(v) => i64::from(*v),
            ArgValue::Int16(v) => i64::from(*v),
            ArgValue::UInt32(v) => i64::from(*v),
            ArgValue::Int32(v) => i64::from(*v),
            _ => return None,
        };
        match target {
            ArgType::UInt8 if (0..=u8::MAX as i64).contains(&n) => Some(ArgValue::UInt8(n as u8)),
            ArgType::UInt16 if (0..=u16::MAX as i64).contains(&n) => {
                Some(ArgValue::UInt16(n as u16))
            }
            ArgType::Int16 if (i16::MIN as i64..=i16::MAX as i64).contains(&n) => {
                Some(ArgValue::Int16(n as i16))
            }
            ArgType::UInt32 if (0..=u32::MAX as i64).contains(&n) => {
                Some(ArgValue::UInt32(n as u32))
            }
            ArgType::Int32 if (i32::MIN as i64..=i32::MAX as i64).contains(&n) => {
                Some(ArgValue::Int32(n as i32))
            }
            _ => None,
        }
    }
}

/// Format string specifiers used in Klipper message format strings.
/// These map between [`ArgType`] variants and their string representations.
impl ArgType {
    /// Parse a format string specifier into an [`ArgType`].
    ///
    /// Returns `None` if the specifier is not recognized.
    pub fn parse_format(specifier: &str) -> Option<Self> {
        match specifier {
            "%u" => Some(ArgType::UInt32),
            "%i" => Some(ArgType::Int32),
            "%hu" => Some(ArgType::UInt16),
            "%hi" => Some(ArgType::Int16),
            "%c" => Some(ArgType::UInt8),
            "%s" => Some(ArgType::Str),
            // `%*s` and `%.*s` both carry a length-prefixed byte buffer upstream
            // (`PT_buffer` and `PT_progmem_buffer` are empty `PT_string`
            // subclasses, `klippy/msgproto.py:70-80`), so both stay binary-safe
            // here. [`ArgType::format_str`] cannot tell them apart and prints
            // `%.*s`; exact format matching reads the dictionary's raw string
            // (`Mcu::try_lookup_command`), never this reconstruction.
            "%*s" | "%.*s" => Some(ArgType::Bytes),
            _ => None,
        }
    }

    /// Convert this [`ArgType`] to its format string specifier.
    pub fn format_str(&self) -> &'static str {
        match self {
            ArgType::UInt32 => "%u",
            ArgType::Int32 => "%i",
            ArgType::UInt16 => "%hu",
            ArgType::Int16 => "%hi",
            ArgType::UInt8 => "%c",
            ArgType::Str => "%s",
            ArgType::Bytes => "%.*s",
        }
    }
}

/// Message payload for encoding and decoding parameters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Payload {
    /// Raw payload bytes. Private so that the [`MESSAGE_PAYLOAD_MAX`] limit
    /// cannot be bypassed; use [`Payload::payload`] for read access.
    raw: Vec<u8>,
}

impl Default for Payload {
    fn default() -> Self {
        Self::new()
    }
}

/// Number of bytes needed to encode `v` using Klipper's variable-length
/// integer (VLQ) encoding.
///
/// This mirrors `encode_int()` in the firmware (`src/command.c`) and
/// `PT_uint32.encode()` in `msgproto.py`: `v` is interpreted as a signed
/// 32-bit value, so negative numbers use a compact sign-extended form and
/// share the encoder with unsigned values.
fn vlq_len(v: u32) -> usize {
    let sv = v as i32;
    if (-(1 << 5)..(3 << 5)).contains(&sv) {
        1
    } else if (-(1 << 12)..(3 << 12)).contains(&sv) {
        2
    } else if (-(1 << 19)..(3 << 19)).contains(&sv) {
        3
    } else if (-(1 << 26)..(3 << 26)).contains(&sv) {
        4
    } else {
        5
    }
}

impl Payload {
    pub fn new() -> Self {
        Self {
            raw: Vec::with_capacity(MESSAGE_PAYLOAD_MAX),
        }
    }

    /// Create a Payload from raw bytes.
    pub fn from_raw(raw: Vec<u8>) -> Self {
        Self { raw }
    }

    pub fn is_empty(&self) -> bool {
        self.raw.is_empty()
    }

    pub fn len(&self) -> usize {
        self.raw.len()
    }

    /// Get a reference to the raw payload bytes.
    pub fn payload(&self) -> &[u8] {
        &self.raw
    }

    /// Append a single byte to the payload.
    pub fn push(&mut self, byte: u8) -> MsgResult<()> {
        if self.len() >= MESSAGE_PAYLOAD_MAX {
            return Err(MsgError::new("payload exceeds maximum length"));
        }
        self.raw.push(byte);
        Ok(())
    }

    /// Extend the payload with multiple bytes.
    pub fn extend(&mut self, bytes: &[u8]) -> MsgResult<()> {
        if self.len() + bytes.len() > MESSAGE_PAYLOAD_MAX {
            return Err(MsgError::new("payload exceeds maximum length"));
        }
        self.raw.extend_from_slice(bytes);
        Ok(())
    }

    /// Try to merge `other` into this payload.
    ///
    /// If the combined length would not exceed [`MESSAGE_PAYLOAD_MAX`], the
    /// bytes of `other` are appended to this payload and `Ok(())` is returned.
    /// Otherwise this payload is left unchanged and an error is returned.
    pub fn try_merge(&mut self, other: &Payload) -> MsgResult<()> {
        self.extend(other.payload())
    }

    /// Append `v` as a Klipper VLQ.
    ///
    /// The most significant 7-bit group is written first; every byte except
    /// the last has its high bit set as a continuation marker. Negative
    /// values (and unsigned values with bit 31 set) are written in the same
    /// compact sign-extended form the firmware's `parse_int()` expects.
    fn push_vlq(&mut self, v: u32) -> MsgResult<()> {
        let len = vlq_len(v);
        if self.len() + len > MESSAGE_PAYLOAD_MAX {
            return Err(MsgError::new("payload exceeds maximum length"));
        }
        for i in (0..len).rev() {
            let mut byte = ((v >> (i * 7)) & 0x7f) as u8;
            if i > 0 {
                byte |= 0x80;
            }
            self.raw.push(byte);
        }
        Ok(())
    }

    /// Push an unsigned 8-bit value (`%c`).
    pub fn push_u8(&mut self, v: u8) -> MsgResult<()> {
        self.push_vlq(v as u32)
    }

    /// Push an unsigned 16-bit value (`%hu`).
    pub fn push_u16(&mut self, v: u16) -> MsgResult<()> {
        self.push_vlq(v as u32)
    }

    /// Push an unsigned 32-bit value (`%u`).
    pub fn push_u32(&mut self, v: u32) -> MsgResult<()> {
        self.push_vlq(v)
    }

    /// Push a signed 16-bit value (`%hi`).
    pub fn push_i16(&mut self, v: i16) -> MsgResult<()> {
        self.push_vlq(v as i32 as u32)
    }

    /// Push a signed 32-bit value (`%i`, and message ids).
    pub fn push_i32(&mut self, v: i32) -> MsgResult<()> {
        self.push_vlq(v as u32)
    }

    /// Push a byte array (length prefix followed by data).
    pub fn push_bytes(&mut self, bytes: &[u8]) -> MsgResult<()> {
        if self.len() + 1 + bytes.len() > MESSAGE_PAYLOAD_MAX {
            return Err(MsgError::new("payload exceeds maximum length"));
        }

        self.raw.push(bytes.len() as u8);
        self.raw.extend(bytes);

        Ok(())
    }

    /// Push an ArgValue (dispatches to appropriate push method based on variant).
    pub fn push_value(&mut self, value: &ArgValue) -> MsgResult<()> {
        match value {
            ArgValue::UInt8(v) => self.push_u8(*v),
            ArgValue::UInt16(v) => self.push_u16(*v),
            ArgValue::Int16(v) => self.push_i16(*v),
            ArgValue::UInt32(v) => self.push_u32(*v),
            ArgValue::Int32(v) => self.push_i32(*v),
            ArgValue::Str(v) => self.push_bytes(v.as_bytes()),
            ArgValue::Bytes(v) => self.push_bytes(v),
        }
    }

    /// Extend with multiple ArgValue.
    pub fn extend_values(&mut self, values: &[ArgValue]) -> MsgResult<()> {
        for v in values {
            self.push_value(v)?;
        }
        Ok(())
    }

    /// Create a payload parser.
    pub fn as_parser(&self) -> PayloadParser<'_> {
        PayloadParser { raw: &self.raw }
    }

    /// Consume the payload and return the raw bytes.
    pub fn into_raw(self) -> Vec<u8> {
        self.raw
    }
}

/// Payload parser for decoding parameters from byte stream.
pub struct PayloadParser<'a> {
    raw: &'a [u8],
}

impl PayloadParser<'_> {
    /// Returns `true` if all bytes have been consumed.
    pub fn is_empty(&self) -> bool {
        self.raw.is_empty()
    }

    /// Number of bytes left to parse.
    pub fn remaining(&self) -> usize {
        self.raw.len()
    }

    /// Pop a single byte.
    pub fn pop(&mut self) -> MsgResult<u8> {
        if self.raw.is_empty() {
            return Err(MsgError::new("payload underflow"));
        }
        let byte = self.raw[0];
        self.raw = &self.raw[1..];
        Ok(byte)
    }

    /// Pop a byte array (length prefix followed by data).
    pub fn pop_bytes(&mut self) -> MsgResult<Vec<u8>> {
        let len = self.pop()? as usize;
        if self.raw.len() < len {
            return Err(MsgError::new("payload underflow"));
        }
        let bytes = self.raw[..len].to_vec();
        self.raw = &self.raw[len..];
        Ok(bytes)
    }

    /// Pop a UTF-8 string.
    pub fn pop_string(&mut self) -> MsgResult<String> {
        let bytes = self.pop_bytes()?;
        match String::from_utf8(bytes) {
            Ok(s) => Ok(s),
            Err(_) => Err(MsgError::new("invalid UTF-8 string")),
        }
    }

    /// Pop a Klipper VLQ as an unsigned 32-bit value.
    ///
    /// This mirrors `parse_int()` in the firmware (`src/command.c`). The first
    /// byte carries a sign bit (bits 5 and 6), so values that were encoded
    /// from negative numbers come back as their two's-complement form.
    pub fn pop_u32(&mut self) -> MsgResult<u32> {
        let mut byte = self.pop()?;
        let mut val = (byte & 0x7f) as u32;
        if byte & 0x60 == 0x60 {
            val |= 0xffff_ffe0;
        }
        while byte & 0x80 != 0 {
            byte = self.pop()?;
            val = (val << 7) | (byte & 0x7f) as u32;
        }
        Ok(val)
    }

    /// Pop a signed 32-bit value (`%i`).
    pub fn pop_i32(&mut self) -> MsgResult<i32> {
        Ok(self.pop_u32()? as i32)
    }

    /// Pop an unsigned 16-bit value (`%hu`), rejecting out-of-range encodings.
    pub fn pop_u16(&mut self) -> MsgResult<u16> {
        let val: u32 = self.pop_u32()?;
        if val > u16::MAX as u32 {
            return Err(MsgError::new("u16 encoding too long"));
        }

        Ok(val as u16)
    }

    /// Pop a signed 16-bit value (`%hi`).
    pub fn pop_i16(&mut self) -> MsgResult<i16> {
        Ok(self.pop_u32()? as i32 as i16)
    }

    /// Pop an unsigned 8-bit value (`%c`), rejecting out-of-range encodings.
    pub fn pop_u8(&mut self) -> MsgResult<u8> {
        let val: u32 = self.pop_u32()?;
        if val > u8::MAX as u32 {
            return Err(MsgError::new("u8 encoding too long"));
        }
        Ok(val as u8)
    }

    /// Pop a signed 8-bit value.
    pub fn pop_i8(&mut self) -> MsgResult<i8> {
        Ok(self.pop_u32()? as i32 as i8)
    }

    /// Pop a value according to the given ArgType.
    pub fn pop_value(&mut self, arg_type: ArgType) -> MsgResult<ArgValue> {
        match arg_type {
            ArgType::UInt8 => Ok(ArgValue::UInt8(self.pop_u8()?)),
            ArgType::UInt16 => Ok(ArgValue::UInt16(self.pop_u16()?)),
            ArgType::Int16 => Ok(ArgValue::Int16(self.pop_i16()?)),
            ArgType::UInt32 => Ok(ArgValue::UInt32(self.pop_u32()?)),
            ArgType::Int32 => Ok(ArgValue::Int32(self.pop_i32()?)),
            ArgType::Str => Ok(ArgValue::Str(self.pop_string()?)),
            ArgType::Bytes => Ok(ArgValue::Bytes(self.pop_bytes()?)),
        }
    }

    /// Pop multiple values according to the given ArgTypes.
    pub fn pop_values(&mut self, arg_types: &[ArgType]) -> MsgResult<Vec<ArgValue>> {
        let mut values = Vec::with_capacity(arg_types.len());
        for arg_type in arg_types {
            values.push(self.pop_value(*arg_type)?);
        }
        Ok(values)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Klipper VLQ u8 encoding roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn test_push_pop_u8_roundtrip() {
        let values = [0u8, 1, 127, 128, 255];
        for &v in &values {
            let mut payload = Payload::new();
            payload.push_u8(v).unwrap();
            let popped = payload.as_parser().pop_u8().unwrap();
            assert_eq!(popped, v);
        }
    }

    #[test]
    fn test_push_pop_u8_boundary_6bit() {
        // Klipper VLQ boundary: 0-95 encode as 1 byte, 96+ need 2 bytes
        let mut payload = Payload::new();
        payload.push_u8(95).unwrap();
        assert_eq!(payload.len(), 1);
        assert_eq!(payload.as_parser().pop_u8().unwrap(), 95);

        let mut payload = Payload::new();
        payload.push_u8(96).unwrap();
        assert_eq!(payload.len(), 2);
        assert_eq!(payload.as_parser().pop_u8().unwrap(), 96);
    }

    // -----------------------------------------------------------------------
    // Klipper VLQ u16 encoding roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn test_push_pop_u16_roundtrip() {
        let values = [0u16, 1, 127, 128, 255, 256, 16383, 16384, u16::MAX];
        for &v in &values {
            let mut payload = Payload::new();
            payload.push_u16(v).unwrap();
            let popped = payload.as_parser().pop_u16().unwrap();
            assert_eq!(popped, v);
        }
    }

    #[test]
    fn test_push_pop_u16_boundary_6bit() {
        // Klipper VLQ boundary: 0-95 encode as 1 byte, 96+ need 2 bytes
        let mut payload = Payload::new();
        payload.push_u16(95).unwrap();
        assert_eq!(payload.len(), 1);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 95);

        let mut payload = Payload::new();
        payload.push_u16(96).unwrap();
        assert_eq!(payload.len(), 2);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 96);
    }

    #[test]
    fn test_push_pop_u16_boundary_12bit() {
        // Second boundary: 0-12287 encode as 2 bytes, 12288+ needs 3 bytes
        let mut payload = Payload::new();
        payload.push_u16(12287).unwrap();
        assert_eq!(payload.len(), 2);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 12287);

        let mut payload = Payload::new();
        payload.push_u16(12288).unwrap();
        assert_eq!(payload.len(), 3);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 12288);
    }

    #[test]
    fn test_signed_encoding_matches_klipper() {
        // Byte vectors produced by Klipper's `encode_int()` (`src/command.c`)
        // and `PT_uint32.encode()` (`msgproto.py`).
        for (value, expected) in [
            (0i32, &[0x00][..]),
            (95, &[0x5f][..]),
            (96, &[0x80, 0x60][..]),
            (127, &[0x80, 0x7f][..]),
            (128, &[0x81, 0x00][..]),
            (12287, &[0xdf, 0x7f][..]),
            (12288, &[0x80, 0xe0, 0x00][..]),
            (-1, &[0x7f][..]),
            (-32, &[0x60][..]),
            (-33, &[0xff, 0x5f][..]),
            (-567, &[0xfb, 0x49][..]),
            (-4096, &[0xe0, 0x00][..]),
            (-4097, &[0xff, 0xdf, 0x7f][..]),
            (-65536, &[0xfc, 0x80, 0x00][..]),
            (-67108864, &[0xe0, 0x80, 0x80, 0x00][..]),
            (-67108865, &[0x8f, 0xdf, 0xff, 0xff, 0x7f][..]),
            (-100000000, &[0x8f, 0xd0, 0xa8, 0xbe, 0x00][..]),
            (i32::MAX, &[0x87, 0xff, 0xff, 0xff, 0x7f][..]),
            (i32::MIN, &[0x88, 0x80, 0x80, 0x80, 0x00][..]),
        ] {
            let mut payload = Payload::new();
            payload.push_i32(value).unwrap();
            assert_eq!(payload.payload(), expected, "i32 value {value}");
            assert_eq!(payload.as_parser().pop_i32().unwrap(), value);
        }

        // Unsigned values share the encoder for the non-negative range.
        for (value, expected) in [(96u32, &[0x80, 0x60][..]), (12345, &[0x80, 0xe0, 0x39][..])] {
            let mut payload = Payload::new();
            payload.push_u32(value).unwrap();
            assert_eq!(payload.payload(), expected, "u32 value {value}");
            assert_eq!(payload.as_parser().pop_u32().unwrap(), value);
        }
    }

    // -----------------------------------------------------------------------
    // Klipper VLQ u32 encoding roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn test_push_pop_u32_roundtrip() {
        let values = [
            0u32,
            1,
            127,
            128,
            200,
            16383,
            16384,
            2097151,
            2097152,
            268435455,
            268435456,
            u32::MAX,
        ];
        for &v in &values {
            let mut payload = Payload::new();
            payload.push_u32(v).unwrap();
            let popped = payload.as_parser().pop_u32().unwrap();
            assert_eq!(popped, v);
        }
    }

    #[test]
    fn test_push_pop_u32_byte_counts() {
        // Verify byte counts at each Klipper VLQ boundary
        for (value, len) in [
            (0u32, 1),
            (95, 1),
            (96, 2),
            (12287, 2),
            (12288, 3),
            (1572863, 3),
            (1572864, 4),
            (201326591, 4),
            (201326592, 5),
            (268435456, 5),
        ] {
            let mut p = Payload::new();
            p.push_u32(value).unwrap();
            assert_eq!(p.len(), len, "value {value}");
            assert_eq!(p.as_parser().pop_u32().unwrap(), value);
        }

        // Unsigned values with bit 31 set are encoded like the negative i32
        // with the same bit pattern and still decode back to the same u32.
        let mut p = Payload::new();
        p.push_u32(u32::MAX).unwrap();
        assert_eq!(p.payload(), &[0x7f]);
        assert_eq!(p.as_parser().pop_u32().unwrap(), u32::MAX);
    }

    #[test]
    fn test_push_pop_u32_signed_values() {
        let mut payload = Payload::new();
        payload.push_value(&ArgValue::Int32(-1)).unwrap();
        let popped = payload.as_parser().pop_value(ArgType::UInt32).unwrap();
        assert_eq!(popped, ArgValue::UInt32(u32::MAX));
    }

    // -----------------------------------------------------------------------
    // VLQ decoding errors
    // -----------------------------------------------------------------------

    #[test]
    fn test_pop_u16_overflow() {
        // Encode a value larger than u16::MAX (needs more than 2 bytes for u16 range)
        let mut payload = Payload::new();
        payload.push_u32(65536).unwrap(); // 0x1_0000, requires 3 bytes
        let result = payload.as_parser().pop_u16();
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("too long"));
    }

    #[test]
    fn test_pop_u32_five_byte_encoding() {
        // Five-byte encoding of u32::MAX as produced by Klipper's
        // `PT_uint32.encode()` (unsigned path).
        let mut payload = Payload::new();
        payload
            .raw
            .extend_from_slice(&[0x8F, 0xFF, 0xFF, 0xFF, 0x7F]);
        assert_eq!(payload.as_parser().pop_u32().unwrap(), u32::MAX);

        // Five-byte encoding of i32::MIN (C `encode_int`).
        let mut payload = Payload::new();
        payload
            .raw
            .extend_from_slice(&[0x88, 0x80, 0x80, 0x80, 0x00]);
        assert_eq!(payload.as_parser().pop_i32().unwrap(), i32::MIN);
    }

    #[test]
    fn test_pop_underflow() {
        let payload = Payload::new();
        let result = payload.as_parser().pop();
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("underflow"));
    }

    // -----------------------------------------------------------------------
    // Bytes encoding roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn test_push_pop_bytes_roundtrip() {
        let test_cases = [b"".as_slice(), b"hello", b"\x00\x01\x02", &[0u8; 50]];
        for data in test_cases {
            let mut payload = Payload::new();
            payload.push_bytes(data).unwrap();
            let popped = payload.as_parser().pop_bytes().unwrap();
            assert_eq!(popped, data);
        }
    }

    #[test]
    fn test_push_pop_string_roundtrip() {
        let test_cases = [
            "",
            "hello world",
            "\u{00E9}\u{00E8}\u{00EA}", // accented characters
        ];
        for s in test_cases {
            let mut payload = Payload::new();
            payload.push_bytes(s.as_bytes()).unwrap();
            let popped = payload.as_parser().pop_string().unwrap();
            assert_eq!(popped, s);
        }
    }

    #[test]
    fn test_pop_invalid_utf8_string() {
        let mut payload = Payload::new();
        payload.raw.push(3); // length = 3
        payload.raw.extend_from_slice(&[0xFF, 0xFE, 0xFD]); // invalid UTF-8
        let result = payload.as_parser().pop_string();
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("UTF-8"));
    }

    // -----------------------------------------------------------------------
    // ArgValue push/pop roundtrip for all types
    // -----------------------------------------------------------------------

    #[test]
    fn test_push_pop_all_arg_values() {
        let mut payload = Payload::new();

        payload.push_value(&ArgValue::UInt8(42)).unwrap();
        payload.push_value(&ArgValue::UInt16(1234)).unwrap();
        payload.push_value(&ArgValue::Int16(-567)).unwrap();
        payload.push_value(&ArgValue::UInt32(12345)).unwrap();
        payload.push_value(&ArgValue::Int32(-12345)).unwrap();
        payload
            .push_value(&ArgValue::Str("hello".to_string()))
            .unwrap();
        payload.push_value(&ArgValue::Bytes(vec![1, 2, 3])).unwrap();

        let mut parser = payload.as_parser();
        assert_eq!(
            parser.pop_value(ArgType::UInt8).unwrap(),
            ArgValue::UInt8(42)
        );
        assert_eq!(
            parser.pop_value(ArgType::UInt16).unwrap(),
            ArgValue::UInt16(1234)
        );
        assert_eq!(
            parser.pop_value(ArgType::Int16).unwrap(),
            ArgValue::Int16(-567)
        );
        assert_eq!(
            parser.pop_value(ArgType::UInt32).unwrap(),
            ArgValue::UInt32(12345)
        );
        assert_eq!(
            parser.pop_value(ArgType::Int32).unwrap(),
            ArgValue::Int32(-12345)
        );
        assert_eq!(
            parser.pop_value(ArgType::Str).unwrap(),
            ArgValue::Str("hello".to_string())
        );
        assert_eq!(
            parser.pop_value(ArgType::Bytes).unwrap(),
            ArgValue::Bytes(vec![1, 2, 3])
        );
    }

    #[test]
    fn test_pop_values_multiple() {
        let mut payload = Payload::new();
        payload.push_u32(1).unwrap();
        payload.push_u32(2).unwrap();
        payload.push_u32(3).unwrap();

        let types = vec![ArgType::UInt32, ArgType::UInt32, ArgType::UInt32];
        let values = payload.as_parser().pop_values(&types).unwrap();
        assert_eq!(values.len(), 3);
        assert_eq!(values[0], ArgValue::UInt32(1));
        assert_eq!(values[1], ArgValue::UInt32(2));
        assert_eq!(values[2], ArgValue::UInt32(3));
    }

    #[test]
    fn test_try_convert_to_range_checks() {
        // Widening conversions succeed.
        assert_eq!(
            ArgValue::UInt8(255).try_convert_to(ArgType::Int16).unwrap(),
            ArgValue::Int16(255)
        );
        assert_eq!(
            ArgValue::UInt8(100)
                .try_convert_to(ArgType::UInt16)
                .unwrap(),
            ArgValue::UInt16(100)
        );
        assert_eq!(
            ArgValue::UInt16(32767)
                .try_convert_to(ArgType::Int16)
                .unwrap(),
            ArgValue::Int16(32767)
        );

        // Narrowing is allowed when the value still fits.
        assert_eq!(
            ArgValue::UInt32(100)
                .try_convert_to(ArgType::UInt16)
                .unwrap(),
            ArgValue::UInt16(100)
        );
        assert_eq!(
            ArgValue::Int32(-1).try_convert_to(ArgType::Int16).unwrap(),
            ArgValue::Int16(-1)
        );
        assert_eq!(
            ArgValue::Int16(-1).try_convert_to(ArgType::Int32).unwrap(),
            ArgValue::Int32(-1)
        );

        // Sign-losing or truncating conversions are rejected instead of
        // wrapping.
        assert!(ArgValue::Int16(-1)
            .try_convert_to(ArgType::UInt16)
            .is_none());
        assert!(ArgValue::UInt16(40000)
            .try_convert_to(ArgType::Int16)
            .is_none());
        assert!(ArgValue::Int32(-1)
            .try_convert_to(ArgType::UInt32)
            .is_none());
        assert!(ArgValue::UInt32(i32::MAX as u32 + 1)
            .try_convert_to(ArgType::Int32)
            .is_none());
        assert!(ArgValue::Int32(0x1_0000)
            .try_convert_to(ArgType::UInt16)
            .is_none());

        // `%s` and `%.*s` share a wire format and may be converted, but
        // numbers never convert to or from strings/buffers.
        assert_eq!(
            ArgValue::Str("hi".to_string())
                .try_convert_to(ArgType::Bytes)
                .unwrap(),
            ArgValue::Bytes(b"hi".to_vec())
        );
        assert_eq!(
            ArgValue::Bytes(b"hi".to_vec())
                .try_convert_to(ArgType::Str)
                .unwrap(),
            ArgValue::Str("hi".to_string())
        );
        assert!(ArgValue::Bytes(vec![0xFF, 0xFE])
            .try_convert_to(ArgType::Str)
            .is_none());
        assert!(ArgValue::Str("x".to_string())
            .try_convert_to(ArgType::UInt32)
            .is_none());
        assert!(ArgValue::UInt32(1).try_convert_to(ArgType::Str).is_none());
    }

    #[test]
    fn test_push_u16_exact_size_bound() {
        // With one byte left, a 1-byte varint still fits…
        let mut p = Payload::new();
        for _ in 0..MESSAGE_PAYLOAD_MAX - 1 {
            p.push(0).unwrap();
        }
        assert!(p.push_u16(0).is_ok());
        assert_eq!(p.len(), MESSAGE_PAYLOAD_MAX);

        // …but a 2-byte varint does not.
        let mut p = Payload::new();
        for _ in 0..MESSAGE_PAYLOAD_MAX - 1 {
            p.push(0).unwrap();
        }
        assert!(p.push_u16(1000).is_err());
        assert_eq!(p.len(), MESSAGE_PAYLOAD_MAX - 1);
    }

    #[test]
    fn test_push_u32_exact_size_bound() {
        // With two bytes left, a 2-byte varint still fits…
        let mut p = Payload::new();
        for _ in 0..MESSAGE_PAYLOAD_MAX - 2 {
            p.push(0).unwrap();
        }
        assert!(p.push_u32(200).is_ok()); // 200 > 127 → 2 bytes
        assert_eq!(p.len(), MESSAGE_PAYLOAD_MAX);

        // …but a 3-byte varint does not.
        let mut p = Payload::new();
        for _ in 0..MESSAGE_PAYLOAD_MAX - 2 {
            p.push(0).unwrap();
        }
        assert!(p.push_u32(20000).is_err()); // 20000 > 16383 → 3 bytes
    }

    #[test]
    fn test_parser_is_empty_and_remaining() {
        let mut p = Payload::new();
        p.push_u32(1).unwrap();
        let mut parser = p.as_parser();
        assert!(!parser.is_empty());
        assert_eq!(parser.remaining(), 1);
        parser.pop_u32().unwrap();
        assert!(parser.is_empty());
        assert_eq!(parser.remaining(), 0);
    }

    #[test]
    fn test_into_raw() {
        let mut p = Payload::new();
        p.push_u16(300).unwrap();
        let raw = p.into_raw();
        assert_eq!(raw.len(), 2);
    }

    #[test]
    fn test_try_merge_within_limit() {
        let mut a = Payload::new();
        a.push_u32(95).unwrap(); // 1 byte (Klipper VLQ)
        let b = build_bytes(b"hello"); // 5 raw bytes

        a.try_merge(&b).unwrap();
        assert_eq!(a.len(), 6);
    }

    #[test]
    fn test_try_merge_over_limit_leaves_unchanged() {
        let mut a = Payload::new();
        // Fill to the maximum length.
        for _ in 0..MESSAGE_PAYLOAD_MAX {
            a.push(0).unwrap();
        }
        let b = build_bytes(b"x");

        // Merging would exceed the maximum length: error and no modification.
        assert!(a.try_merge(&b).is_err());
        assert_eq!(a.len(), MESSAGE_PAYLOAD_MAX);
    }

    /// Helper building a payload from raw bytes.
    fn build_bytes(bytes: &[u8]) -> Payload {
        let mut p = Payload::new();
        p.extend(bytes).unwrap();
        p
    }

    #[test]
    fn test_extend_values() {
        let values = vec![
            ArgValue::UInt32(1),
            ArgValue::Str("test".to_string()),
            ArgValue::Bytes(vec![0xDE, 0xAD]),
        ];
        let mut payload = Payload::new();
        payload.extend_values(&values).unwrap();

        let mut parser = payload.as_parser();
        assert_eq!(
            parser.pop_value(ArgType::UInt32).unwrap(),
            ArgValue::UInt32(1)
        );
        assert_eq!(
            parser.pop_value(ArgType::Str).unwrap(),
            ArgValue::Str("test".to_string())
        );
        assert_eq!(
            parser.pop_value(ArgType::Bytes).unwrap(),
            ArgValue::Bytes(vec![0xDE, 0xAD])
        );
    }

    // -----------------------------------------------------------------------
    // Payload size limits
    // -----------------------------------------------------------------------

    #[test]
    fn test_payload_exceeds_max() {
        let mut payload = Payload::new();
        // Fill to near the limit
        for _ in 0..MESSAGE_PAYLOAD_MAX {
            payload.push(0).unwrap();
        }
        assert_eq!(payload.len(), MESSAGE_PAYLOAD_MAX);
        // Next push should fail
        assert!(payload.push(0).is_err());
    }

    #[test]
    fn test_extend_exceeds_max() {
        let mut payload = Payload::new();
        // Fill to near the limit
        for _ in 0..MESSAGE_PAYLOAD_MAX {
            payload.push(0).unwrap();
        }
        // Extend by 1 should fail
        assert!(payload.extend(&[0]).is_err());
    }

    #[test]
    fn test_push_bytes_exceeds_max() {
        let mut payload = Payload::new();
        // Fill almost to the limit (1 byte for length prefix + data)
        for _ in 0..(MESSAGE_PAYLOAD_MAX - 1) {
            payload.push(0).unwrap();
        }
        // Pushing 2 bytes of data (1 length + 1 data) should fail
        assert!(payload.push_bytes(&[1, 2]).is_err());
    }

    #[test]
    fn test_payload_is_empty_and_len() {
        let payload = Payload::new();
        assert!(payload.is_empty());
        assert_eq!(payload.len(), 0);

        let mut payload = Payload::new();
        payload.push(1).unwrap();
        assert!(!payload.is_empty());
        assert_eq!(payload.len(), 1);
    }
}
