use super::super::frame::MESSAGE_PAYLOAD_MAX;

/// Protocol error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProtoError {
    pub msg: String,
}

impl ProtoError {
    /// Create a new protocol error.
    pub fn new(msg: impl Into<String>) -> Self {
        Self { msg: msg.into() }
    }
}

impl std::fmt::Display for ProtoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.msg)
    }
}

impl std::error::Error for ProtoError {}

impl From<serde_json::Error> for ProtoError {
    fn from(e: serde_json::Error) -> Self {
        ProtoError::new(e.to_string())
    }
}

impl From<super::MsgError> for ProtoError {
    fn from(e: super::MsgError) -> Self {
        ProtoError::new(e.msg)
    }
}

/// Result type used for protocol operations.
pub type ProtoResult<T> = Result<T, ProtoError>;

/// Parameter type enum.
#[derive(Debug, Clone, PartialEq, Copy, Eq, Hash)]
pub enum ArgType {
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
    /// Only lossless conversions are supported (numeric types only):
    /// - Widening: Int16 → Int32, UInt16 → UInt32, UInt16 → Int32
    /// - Same width with a range check: Int16 ↔ UInt16, Int32 ↔ UInt32
    ///
    /// Conversions that would wrap or lose information are rejected with
    /// `Err(())`: negative values cannot convert to unsigned types, and
    /// unsigned values above the target's maximum cannot convert to signed
    /// types. Str/Bytes never convert to or from numeric types.
    pub fn try_convert_to(&self, target: ArgType) -> Result<ArgValue, ()> {
        // Same type — no conversion needed
        if self.arg_type() == target {
            return Ok(self.clone());
        }

        match (self, target) {
            // Int16 ↔ UInt16 (negative → unsigned is rejected)
            (ArgValue::Int16(v), ArgType::UInt16) if *v >= 0 => {
                Ok(ArgValue::UInt16(*v as u16))
            }
            (ArgValue::UInt16(v), ArgType::Int16) if *v <= i16::MAX as u16 => {
                Ok(ArgValue::Int16(*v as i16))
            }

            // Int32 ↔ UInt32 (negative → unsigned is rejected)
            (ArgValue::Int32(v), ArgType::UInt32) if *v >= 0 => {
                Ok(ArgValue::UInt32(*v as u32))
            }
            (ArgValue::UInt32(v), ArgType::Int32) if *v <= i32::MAX as u32 => {
                Ok(ArgValue::Int32(*v as i32))
            }

            // Widening conversions are always lossless
            (ArgValue::Int16(v), ArgType::Int32) => Ok(ArgValue::Int32(*v as i32)),
            (ArgValue::UInt16(v), ArgType::UInt32) => Ok(ArgValue::UInt32(*v as u32)),
            (ArgValue::UInt16(v), ArgType::Int32) => Ok(ArgValue::Int32(*v as i32)),

            // Unsupported: Str/Bytes to numeric, numeric to Str/Bytes,
            // narrowing, and out-of-range signedness changes.
            _ => Err(()),
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

const fn mask(bits: u32) -> u32 {
    (1u32 << bits) - 1
}

const U16_MASK7: u16 = mask(7) as u16;
const U16_MASK14: u16 = mask(14) as u16;
const U32_MASK7: u32 = mask(7);
const U32_MASK14: u32 = mask(14);
const U32_MASK21: u32 = mask(21);
const U32_MASK25: u32 = mask(25);
const U32_MASK28: u32 = mask(28);

impl Payload {
    pub fn new() -> Self {
        Self {
            raw: Vec::with_capacity(MESSAGE_PAYLOAD_MAX),
        }
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
    pub fn push(&mut self, byte: u8) -> ProtoResult<()> {
        if self.len() >= MESSAGE_PAYLOAD_MAX {
            return Err(ProtoError::new("payload exceeds maximum length"));
        }
        self.raw.push(byte);
        Ok(())
    }

    /// Extend the payload with multiple bytes.
    pub fn extend(&mut self, bytes: &[u8]) -> ProtoResult<()> {
        if self.len() + bytes.len() > MESSAGE_PAYLOAD_MAX {
            return Err(ProtoError::new("payload exceeds maximum length"));
        }
        self.raw.extend_from_slice(bytes);
        Ok(())
    }

    /// Try to merge `other` into this payload.
    ///
    /// If the combined length would not exceed [`MESSAGE_PAYLOAD_MAX`], the
    /// bytes of `other` are appended to this payload and `Ok(())` is returned.
    /// Otherwise this payload is left unchanged and an error is returned.
    pub fn try_merge(&mut self, other: &Payload) -> ProtoResult<()> {
        self.extend(other.payload())
    }

    /// Push a u16 value in 7-bit varint format.
    pub fn push_u16(&mut self, v: u16) -> ProtoResult<()> {
        let needed = if v > U16_MASK14 {
            3
        } else if v > U16_MASK7 {
            2
        } else {
            1
        };
        if self.len() + needed > MESSAGE_PAYLOAD_MAX {
            return Err(ProtoError::new("payload exceeds maximum length"));
        }

        if v > U16_MASK14 {
            self.raw.push(((v >> 14) & 0x7f | 0x80) as u8);
        }

        if v > U16_MASK7 {
            self.raw.push(((v >> 7) & 0x7f | 0x80) as u8);
        }

        self.raw.push((v & 0x7f) as u8);

        Ok(())
    }

    /// Push a u32 value in 7-bit varint format.
    pub fn push_u32(&mut self, v: u32) -> ProtoResult<()> {
        let needed = if v > U32_MASK28 {
            5
        } else if v > U32_MASK21 {
            4
        } else if v > U32_MASK14 {
            3
        } else if v > U32_MASK7 {
            2
        } else {
            1
        };
        if self.len() + needed > MESSAGE_PAYLOAD_MAX {
            return Err(ProtoError::new("payload exceeds maximum length"));
        }

        if v > U32_MASK28 {
            self.raw.push(((v >> 28) & 0x7f | 0x80) as u8);
        }

        if v > U32_MASK21 {
            self.raw.push(((v >> 21) & 0x7f | 0x80) as u8);
        }

        if v > U32_MASK14 {
            self.raw.push(((v >> 14) & 0x7f | 0x80) as u8);
        }

        if v > U32_MASK7 {
            self.raw.push(((v >> 7) & 0x7f | 0x80) as u8);
        }

        self.raw.push((v & 0x7f) as u8);

        Ok(())
    }

    /// Push a byte array (length prefix followed by data).
    pub fn push_bytes(&mut self, bytes: &[u8]) -> ProtoResult<()> {
        if self.len() + 1 + bytes.len() > MESSAGE_PAYLOAD_MAX {
            return Err(ProtoError::new("payload exceeds maximum length"));
        }

        self.raw.push(bytes.len() as u8);
        self.raw.extend(bytes);

        Ok(())
    }

    /// Push an ArgValue (dispatches to appropriate push method based on variant).
    pub fn push_value(&mut self, value: &ArgValue) -> ProtoResult<()> {
        match value {
            ArgValue::UInt16(v) => self.push_u16(*v),
            ArgValue::Int16(v) => self.push_u16(*v as u16),
            ArgValue::UInt32(v) => self.push_u32(*v),
            ArgValue::Int32(v) => self.push_u32(*v as u32),
            ArgValue::Str(v) => self.push_bytes(v.as_bytes()),
            ArgValue::Bytes(v) => self.push_bytes(v),
        }
    }

    /// Extend with multiple ArgValue.
    pub fn extend_values(&mut self, values: &[ArgValue]) -> ProtoResult<()> {
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
    pub fn pop(&mut self) -> ProtoResult<u8> {
        if self.raw.is_empty() {
            return Err(ProtoError::new("payload underflow"));
        }
        let byte = self.raw[0];
        self.raw = &self.raw[1..];
        Ok(byte)
    }

    /// Pop a byte array (length prefix followed by data).
    pub fn pop_bytes(&mut self) -> ProtoResult<Vec<u8>> {
        let len = self.pop()? as usize;
        if self.raw.len() < len {
            return Err(ProtoError::new("payload underflow"));
        }
        let bytes = self.raw[..len].to_vec();
        self.raw = &self.raw[len..];
        Ok(bytes)
    }

    /// Pop a UTF-8 string.
    pub fn pop_string(&mut self) -> ProtoResult<String> {
        let bytes = self.pop_bytes()?;
        match String::from_utf8(bytes) {
            Ok(s) => Ok(s),
            Err(_) => Err(ProtoError::new("invalid UTF-8 string")),
        }
    }

    /// Pop a u32 value in 7-bit varint format.
    pub fn pop_u32(&mut self) -> ProtoResult<u32> {
        let mut val: u32 = 0;
        loop {
            let byte = self.pop()?;

            val |= (byte & 0x7F) as u32;

            if byte & 0x80 == 0 {
                break;
            }

            if val > U32_MASK25 {
                return Err(ProtoError::new("u32 encoding too long"));
            }

            val <<= 7;
        }
        Ok(val)
    }

    /// Pop an i32 value (as u32 internally).
    pub fn pop_i32(&mut self) -> ProtoResult<i32> {
        Ok(self.pop_u32()? as i32)
    }

    /// Pop a u16 value in 7-bit varint format.
    pub fn pop_u16(&mut self) -> ProtoResult<u16> {
        let val: u32 = self.pop_u32()?;
        if val > u16::MAX as u32 {
            return Err(ProtoError::new("u16 encoding too long"));
        }

        Ok(val as u16)
    }

    /// Pop an i16 value (as u16 internally).
    pub fn pop_i16(&mut self) -> ProtoResult<i16> {
        Ok(self.pop_u16()? as i16)
    }

    /// Pop a value according to the given ArgType.
    pub fn pop_value(&mut self, arg_type: ArgType) -> ProtoResult<ArgValue> {
        match arg_type {
            ArgType::UInt16 => Ok(ArgValue::UInt16(self.pop_u16()?)),
            ArgType::Int16 => Ok(ArgValue::Int16(self.pop_i16()?)),
            ArgType::UInt32 => Ok(ArgValue::UInt32(self.pop_u32()?)),
            ArgType::Int32 => Ok(ArgValue::Int32(self.pop_i32()?)),
            ArgType::Str => Ok(ArgValue::Str(self.pop_string()?)),
            ArgType::Bytes => Ok(ArgValue::Bytes(self.pop_bytes()?)),
        }
    }

    /// Pop multiple values according to the given ArgTypes.
    pub fn pop_values(&mut self, arg_types: &[ArgType]) -> ProtoResult<Vec<ArgValue>> {
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
    // Big-endian 7-bit varint u16 encoding roundtrip
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
    fn test_push_pop_u16_boundary_7bit() {
        // 7-bit boundary: 0-127 encode as 1 byte, 128+ needs 2 bytes
        let mut payload = Payload::new();
        payload.push_u16(127).unwrap();
        assert_eq!(payload.len(), 1);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 127);

        let mut payload = Payload::new();
        payload.push_u16(128).unwrap();
        assert_eq!(payload.len(), 2);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 128);
    }

    #[test]
    fn test_push_pop_u16_boundary_14bit() {
        // 14-bit boundary: 0-16383 encode as 2 bytes, 16384+ needs 3 bytes
        let mut payload = Payload::new();
        payload.push_u16(16383).unwrap();
        assert_eq!(payload.len(), 2);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 16383);

        let mut payload = Payload::new();
        payload.push_u16(16384).unwrap();
        assert_eq!(payload.len(), 3);
        assert_eq!(payload.as_parser().pop_u16().unwrap(), 16384);
    }

    #[test]
    fn test_push_pop_u16_signed_values() {
        // Int16/UInt16 push uses u16 encoding, pop returns u16
        let mut payload = Payload::new();
        payload.push_value(&ArgValue::Int16(-1)).unwrap();
        let popped = payload.as_parser().pop_value(ArgType::UInt16).unwrap();
        assert_eq!(popped, ArgValue::UInt16(u16::MAX));
    }

    // -----------------------------------------------------------------------
    // Big-endian 7-bit varint u32 encoding roundtrip
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
        // Verify byte counts at each 7-bit varint boundary
        let mut p = Payload::new();
        p.push_u32(0).unwrap();
        assert_eq!(p.len(), 1); // 0 takes 1 byte (0x00)

        let mut p = Payload::new();
        p.push_u32(127).unwrap();
        assert_eq!(p.len(), 1); // 7 bits fit in 1 byte

        let mut p = Payload::new();
        p.push_u32(128).unwrap();
        assert_eq!(p.len(), 2); // 8 bits need 2 bytes

        let mut p = Payload::new();
        p.push_u32(16383).unwrap();
        assert_eq!(p.len(), 2); // 14 bits fit in 2 bytes

        let mut p = Payload::new();
        p.push_u32(16384).unwrap();
        assert_eq!(p.len(), 3); // 15 bits need 3 bytes

        let mut p = Payload::new();
        p.push_u32(2097151).unwrap();
        assert_eq!(p.len(), 3); // 21 bits fit in 3 bytes

        let mut p = Payload::new();
        p.push_u32(2097152).unwrap();
        assert_eq!(p.len(), 4); // 22 bits need 4 bytes

        let mut p = Payload::new();
        p.push_u32(268435455).unwrap();
        assert_eq!(p.len(), 4); // 28 bits fit in 4 bytes

        let mut p = Payload::new();
        p.push_u32(268435456).unwrap();
        assert_eq!(p.len(), 5); // 29 bits need 5 bytes (max for u32)
    }

    #[test]
    fn test_push_pop_u32_signed_values() {
        let mut payload = Payload::new();
        payload.push_value(&ArgValue::Int32(-1)).unwrap();
        let popped = payload.as_parser().pop_value(ArgType::UInt32).unwrap();
        assert_eq!(popped, ArgValue::UInt32(u32::MAX));
    }

    // -----------------------------------------------------------------------
    // 7-bit varint decoding errors
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
    fn test_pop_u32_overflow() {
        // pop_u32 is big-endian 7-bit: first byte → highest 7 bits.
        //
        // Trace: val |= B[6:0]; check overflow; val <<= 7
        // After byte 0: val = B0[6:0]            (max 7 bits)
        // After byte 1: val = B0[6:0]<<7 | B1[6:0] (max 14 bits)
        // After byte 2: val = ...<<7 | B2[6:0]      (max 21 bits)
        // After byte 3: val = ...<<7 | B3[6:0]      (max 28 bits)
        //   → check: val > U32_MASK25 (25 bits)? If B0=B1=B2=B3=0xFF:
        //     val = 0x7F<<21 | 0x7F<<14 | 0x7F<<7 | 0x7F = 0x0FFFFFFF
        //     0x0FFFFFFF = 268435455 > U32_MASK25 = 33554431 → overflow!
        let mut payload = Payload::new();
        payload.raw.extend_from_slice(&[
            0xFF, 0xFF, 0xFF, 0xFF, // 4 continuation bytes
            0x00,                    // final byte (break)
        ]);
        // After processing byte 3 (0xFF), val = 0x0FFFFFFF > U32_MASK25
        let result = payload.as_parser().pop_u32();
        assert!(result.is_err(), "expected overflow error, got {:?}", result);
        assert!(result.unwrap_err().msg.contains("too long"));
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
        let test_cases = [
            b"".as_slice(),
            b"hello",
            b"\x00\x01\x02",
            &[0u8; 50],
        ];
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

        payload.push_value(&ArgValue::UInt16(1234)).unwrap();
        payload.push_value(&ArgValue::Int16(-567)).unwrap();
        payload.push_value(&ArgValue::UInt32(12345)).unwrap();
        payload.push_value(&ArgValue::Int32(-12345)).unwrap();
        payload.push_value(&ArgValue::Str("hello".to_string())).unwrap();
        payload.push_value(&ArgValue::Bytes(vec![1, 2, 3])).unwrap();

        let mut parser = payload.as_parser();
        assert_eq!(parser.pop_value(ArgType::UInt16).unwrap(), ArgValue::UInt16(1234));
        assert_eq!(parser.pop_value(ArgType::Int16).unwrap(), ArgValue::Int16(-567));
        assert_eq!(parser.pop_value(ArgType::UInt32).unwrap(), ArgValue::UInt32(12345));
        assert_eq!(parser.pop_value(ArgType::Int32).unwrap(), ArgValue::Int32(-12345));
        assert_eq!(parser.pop_value(ArgType::Str).unwrap(), ArgValue::Str("hello".to_string()));
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
    fn test_try_convert_to_lossless_range_checks() {
        // In-range same-width conversions succeed.
        assert_eq!(
            ArgValue::Int16(100).try_convert_to(ArgType::UInt16).unwrap(),
            ArgValue::UInt16(100)
        );
        assert_eq!(
            ArgValue::UInt16(32767).try_convert_to(ArgType::Int16).unwrap(),
            ArgValue::Int16(32767)
        );
        assert_eq!(
            ArgValue::Int32(42).try_convert_to(ArgType::UInt32).unwrap(),
            ArgValue::UInt32(42)
        );

        // Out-of-range / sign-losing conversions are rejected instead of
        // wrapping.
        assert!(ArgValue::Int16(-1).try_convert_to(ArgType::UInt16).is_err());
        assert!(ArgValue::UInt16(40000).try_convert_to(ArgType::Int16).is_err());
        assert!(ArgValue::Int32(-1).try_convert_to(ArgType::UInt32).is_err());
        assert!(
            ArgValue::UInt32(i32::MAX as u32 + 1)
                .try_convert_to(ArgType::Int32)
                .is_err()
        );
        assert!(ArgValue::Int16(-1).try_convert_to(ArgType::UInt32).is_err());

        // Widening conversions remain lossless for negative values.
        assert_eq!(
            ArgValue::Int16(-1).try_convert_to(ArgType::Int32).unwrap(),
            ArgValue::Int32(-1)
        );

        // Narrowing conversions (Int32 → Int16, UInt32 → UInt16) are not
        // supported at all.
        assert!(ArgValue::Int32(-1).try_convert_to(ArgType::Int16).is_err());
        assert!(ArgValue::UInt32(100).try_convert_to(ArgType::UInt16).is_err());

        // Same type is a no-op; Str/Bytes never convert to or from numerics.
        assert_eq!(
            ArgValue::UInt32(7).try_convert_to(ArgType::UInt32).unwrap(),
            ArgValue::UInt32(7)
        );
        assert!(ArgValue::Str("x".to_string()).try_convert_to(ArgType::UInt32).is_err());
        assert!(ArgValue::Bytes(vec![]).try_convert_to(ArgType::Str).is_err());
        assert!(ArgValue::UInt32(1).try_convert_to(ArgType::Str).is_err());
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
        a.push_u32(100).unwrap(); // 1 byte (7-bit varint)
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
        assert_eq!(parser.pop_value(ArgType::UInt32).unwrap(), ArgValue::UInt32(1));
        assert_eq!(parser.pop_value(ArgType::Str).unwrap(), ArgValue::Str("test".to_string()));
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
