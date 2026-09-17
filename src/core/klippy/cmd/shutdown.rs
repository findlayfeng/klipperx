//! `emergency_stop` / `clear_shutdown` — stop the MCU and leave the stopped state.
//!
//! Host view of the "Misc commands" section of the firmware's `basecmd.c`. They
//! are the two halves of the firmware's shutdown latch:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `emergency_stop` |
//! | host → MCU | `clear_shutdown` |
//!
//! Neither has a response, and both are `HF_IN_SHUTDOWN`. `emergency_stop` makes
//! the firmware record a shutdown and stop executing queued work; the shutdown
//! handler clears timers, the move queue, and any command that is not
//! `HF_IN_SHUTDOWN`. `clear_shutdown` releases the latch only — it does not
//! restore the configuration, so the host still has to reconfigure and restart
//! afterwards.
//!
//! Because the firmware keeps answering `HF_IN_SHUTDOWN` commands while stopped,
//! the host can send these two either side of a shutdown and still read
//! [`get_config`](super::config::GetConfig) or
//! [`get_uptime`](super::uptime::GetUptime).

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `emergency_stop` — tell the firmware to shut down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmergencyStop;

impl McuCommand for EmergencyStop {
    const NAME: &'static str = "emergency_stop";

    fn args(&self) -> Vec<ArgValue> {
        // No parameters.
        Vec::new()
    }
}

/// `clear_shutdown` — release the firmware's shutdown latch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClearShutdown;

impl McuCommand for ClearShutdown {
    const NAME: &'static str = "clear_shutdown";

    fn args(&self) -> Vec<ArgValue> {
        // No parameters.
        Vec::new()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::Payload;
    use serde_json::json;

    /// The two messages this module needs, as the firmware would publish them.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "emergency_stop": 3,
                "clear_shutdown": 2
            }
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    /// Encode a parameterless command and spell out the expected wire bytes.
    fn assert_encodes_to_id(parser: &Parser, name: &str, args: &[ArgValue], id: i16) {
        let encoded = parser.encode(name, args).unwrap();
        let mut expected = Payload::new();
        expected.push_i16(id).unwrap();
        assert_eq!(encoded.payload(), expected.payload());
    }

    #[test]
    fn test_emergency_stop_matches_the_firmware_format() {
        let parser = parser();

        // Just the id: the message declares no parameters.
        assert_encodes_to_id(&parser, EmergencyStop::NAME, &EmergencyStop.args(), 3);
    }

    #[test]
    fn test_clear_shutdown_matches_the_firmware_format() {
        let parser = parser();

        assert_encodes_to_id(&parser, ClearShutdown::NAME, &ClearShutdown.args(), 2);
    }
}
