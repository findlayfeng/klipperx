//! Shared plumbing for the debug commands the bus sections expose.
//!
//! `[i2c_device]` and `[spi_device]` both offer a pair of port-only commands
//! that move bytes on the bus so the interface can be exercised on a real
//! board. The pieces they have in common live here: the synchronous-to-async
//! bridge the commands need, and hex parsing/formatting for the `DATA=`
//! parameter.
//!
//! Nothing here is part of the bus resources themselves; a real driver would
//! call `McuI2c` / `McuSpi` directly from its own async task.

use crate::core::klippy::gcode::CommandError;

/// Decode a hex string (`"01af"`, whitespace ignored) into bytes.
pub(super) fn hex_decode(text: &str) -> Result<Vec<u8>, CommandError> {
    let cleaned: String = text.chars().filter(|c| !c.is_whitespace()).collect();
    if !cleaned.len().is_multiple_of(2) || !cleaned.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(CommandError::new(format!("invalid hex string '{text}'")));
    }
    let mut out = Vec::with_capacity(cleaned.len() / 2);
    for index in (0..cleaned.len()).step_by(2) {
        out.push(u8::from_str_radix(&cleaned[index..index + 2], 16).expect("validated hex digits"));
    }
    Ok(out)
}

/// Encode bytes as lowercase hex, the form [`hex_decode`] reads back.
pub(super) fn hex_encode(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hex_round_trips() {
        assert_eq!(hex_decode("01af").unwrap(), vec![0x01, 0xaf]);
        assert_eq!(hex_decode("00 FF").unwrap(), vec![0x00, 0xff]);
        assert_eq!(hex_decode("").unwrap(), Vec::<u8>::new());
        assert!(hex_decode("0").is_err());
        assert!(hex_decode("zz").is_err());
        assert_eq!(hex_encode(&[0x00, 0x0f, 0xff]), "000fff");
    }
}
