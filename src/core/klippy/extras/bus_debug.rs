//! Shared plumbing for the debug commands the bus sections expose.
//!
//! `[i2c_device]` and `[spi_device]` both offer a pair of port-only commands
//! that move bytes on the bus so the interface can be exercised on a real
//! board. The pieces they have in common live here: the synchronous-to-async
//! bridge the commands need, hex parsing/formatting for the `DATA=` parameter,
//! and the config-option readers (`spi_speed`, `cs_active_high`, …).
//!
//! Nothing here is part of the bus resources themselves; a real driver would
//! call `McuI2c` / `McuSpi` directly from its own async task.

use crate::core::klippy::config::ConfigSection;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mcu::McuError;

/// Drive an asynchronous bus transfer from a synchronous G-Code handler.
///
/// The dispatcher's handlers are synchronous and cannot await (`gcode.rs`), and
/// a bus transfer is a request/response exchange, so the future is run to
/// completion on the current runtime. `block_in_place` is what makes that legal
/// on a runtime worker; the server's runtimes are multi-threaded
/// (`src/klippy.rs`), and this bridge exists only for the debug commands — a
/// real driver would drive the transfer from its own async task.
///
/// # Errors
/// Returns the transfer's error, or the fact that this thread's runtime cannot
/// block (a single-threaded runtime).
pub(super) fn block_on<T>(
    future: impl std::future::Future<Output = Result<T, McuError>>,
) -> Result<T, CommandError> {
    let handle = tokio::runtime::Handle::try_current()
        .map_err(|_| CommandError::new("bus commands need the async runtime"))?;
    if handle.runtime_flavor() != tokio::runtime::RuntimeFlavor::MultiThread {
        return Err(CommandError::new(
            "bus commands need the multi-threaded runtime",
        ));
    }
    tokio::task::block_in_place(|| handle.block_on(future))
        .map_err(|err| CommandError::new(err.to_string()))
}

/// A required-or-default integer option, as upstream's `getint`.
pub(super) fn parse_int(section: &ConfigSection, name: &str) -> Result<Option<i64>, String> {
    let Some(text) = section.get_str(name) else {
        return Ok(None);
    };
    text.trim().parse::<i64>().map(Some).map_err(|_| {
        format!(
            "Unable to parse option '{name}' in section '{}'",
            section.identifier()
        )
    })
}

/// Read a boolean option the way upstream's `getboolean` does.
pub(super) fn get_bool(section: &ConfigSection, name: &str) -> Result<Option<bool>, String> {
    let Some(text) = section.get_str(name) else {
        return Ok(None);
    };
    match text.trim().to_ascii_lowercase().as_str() {
        "1" | "yes" | "true" | "on" => Ok(Some(true)),
        "0" | "no" | "false" | "off" => Ok(Some(false)),
        _ => Err(format!(
            "Unable to parse option '{name}' in section '{}'",
            section.identifier()
        )),
    }
}

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
