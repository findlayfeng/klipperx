//! Debug commands — `debug_read`, `debug_write`, `debug_ping`, and `debug_nop`.
//!
//! Host view of the debugging commands in the firmware's `debugcmds.c`. These are
//! used for low-level memory access and firmware testing:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `debug_read order=%c addr=%u` |
//! | MCU → host | `debug_result val=%u` |
//! | host → MCU | `debug_write order=%c addr=%u val=%u` |
//! | host → MCU | `debug_ping data=%*s` |
//! | MCU → host | `pong data=%*s` |
//! | host → MCU | `debug_nop` |

use crate::core::klippy::cmd::{McuCommand, McuResponse};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::{ArgType, ArgValue};

/// `debug_read order=%c addr=%u` — read a value from MCU memory.
///
/// - `order`: access size (0=8-bit, 1=16-bit, 2=32-bit)
/// - `addr`: decoded pointer address
///
/// Response: `debug_result val=%u`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugRead {
    /// Access size (0=8-bit, 1=16-bit, 2=32-bit).
    pub order: u8,
    /// Decoded pointer address.
    pub addr: u32,
}

impl McuCommand for DebugRead {
    const NAME: &'static str = "debug_read";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.order), ArgValue::UInt32(self.addr)]
    }
}

/// `debug_write order=%c addr=%u val=%u` — write a value to MCU memory.
///
/// - `order`: access size (0=8-bit, 1=16-bit, 2=32-bit)
/// - `addr`: decoded pointer address
/// - `val`: value to write
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugWrite {
    /// Access size (0=8-bit, 1=16-bit, 2=32-bit).
    pub order: u8,
    /// Decoded pointer address.
    pub addr: u32,
    /// Value to write.
    pub val: u32,
}

impl McuCommand for DebugWrite {
    const NAME: &'static str = "debug_write";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.order),
            ArgValue::UInt32(self.addr),
            ArgValue::UInt32(self.val),
        ]
    }
}

/// `debug_result val=%u` — response to `debug_read`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugResult {
    /// The value read from MCU memory.
    pub val: u32,
}

/// `McuResponse` for `debug_result val=%u`.
///
/// # Errors
/// Returns [`McuError::Decode`] if the `val` parameter is missing or not a
/// 32-bit unsigned integer.
impl McuResponse for DebugResult {
    const NAME: &'static str = "debug_result";

    fn decode(params: &crate::core::klippy::cmd::Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            val: params.get_u32("val")?,
        })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::Params;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::Msg;
    use serde_json::json;
    use std::sync::Arc;

    /// The messages this module needs, as the firmware would publish them.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "debug_read order=%c addr=%u": 12,
                "debug_write order=%c addr=%u val=%u": 11,
                "debug_ping data=%*s": 10
            },
            "responses": {
                "debug_result val=%u": -7
            }
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn test_debug_read_matches_the_firmware_format() {
        let command = DebugRead {
            order: 2,
            addr: 0x2000_0000,
        };

        let encoded = parser().encode(DebugRead::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();

        assert_eq!(decoded[0].0.name, "debug_read");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_debug_write_matches_the_firmware_format() {
        let command = DebugWrite {
            order: 2,
            addr: 0x2000_0000,
            val: 0x1234_5678,
        };

        let encoded = parser().encode(DebugWrite::NAME, &command.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();

        assert_eq!(decoded[0].0.name, "debug_write");
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_debug_result_decode() {
        let msg = Msg::parse(-7, "debug_result val=%u").unwrap();
        let params = Params::new(std::sync::Arc::new(msg), &[ArgValue::UInt32(0x1234_5678)]);

        let result = DebugResult::decode(&params).unwrap();
        assert_eq!(result.val, 0x1234_5678);
    }

    #[test]
    fn test_debug_result_decode_missing_param() {
        let msg = Msg::parse(-7, "debug_result val=%u").unwrap();
        let params = Params::new(Arc::new(msg), &[]);
        let result = DebugResult::decode(&params);
        assert!(result.is_err());
    }
}
