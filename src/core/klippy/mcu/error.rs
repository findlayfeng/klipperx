// ===========================================================================
// Error types for MCU-level operations.
// ===========================================================================

use crate::core::klippy::msg::error::MsgError;

/// Error returned by [`Mcu::call`](super::Mcu::call).
#[derive(Debug)]
pub enum McuCallError {
    /// The command name is not registered in the message parser.
    CommandNotFound(String),
    /// The command already has a callback bound — `call` is only for
    /// synchronous request/response pairs.
    CommandHasCallback(String),
    /// The send buffer is full and the command could not be queued.
    SendFailed(String),
    /// The expected response did not arrive before `timeout` elapsed.
    Timeout(String),
}

impl std::fmt::Display for McuCallError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            McuCallError::CommandNotFound(name) => {
                write!(f, "command not found: {}", name)
            }
            McuCallError::CommandHasCallback(name) => {
                write!(f, "command already has callback: {}", name)
            }
            McuCallError::SendFailed(msg) => {
                write!(f, "send failed: {}", msg)
            }
            McuCallError::Timeout(msg) => {
                write!(f, "timeout: {}", msg)
            }
        }
    }
}

impl std::error::Error for McuCallError {}

/// Error returned by MCU-level operations.
///
/// This is the umbrella error for everything that can go wrong above the raw
/// message codec: identifying an MCU, installing its data dictionary, and
/// running typed commands against it.
#[derive(Debug)]
pub enum McuError {
    /// The MCU data dictionary is malformed, or a message could not be
    /// registered from it.
    Dictionary(String),
    /// A command or response name is not in the installed dictionary.
    ///
    /// Either the firmware does not implement the message, or the identify
    /// handshake has not run yet.
    UnknownMessage(String),
    /// A response parameter is missing, or does not hold the type the host
    /// expects. This means the host and the firmware disagree about the
    /// message — a protocol mismatch, not a transient error.
    Decode(String),
    /// The MCU has not completed the identify handshake yet, so no dictionary
    /// is installed and no command can be resolved.
    NotIdentified,
    /// The firmware was in a session that came **before** this connection, and the
    /// handshake did not finish even after the connection took it over: the
    /// firmware's sequence does not start over, so it was already talking to
    /// somebody when the port was opened — it never rebooted.
    ///
    /// An `rpi_usb` reset leaves a board like this when switching the port's power
    /// disconnects the device without resetting it (a hub that does not really
    /// switch power, or a board powered from its own supply). The message is what
    /// the handshake reported under it.
    OldSession(String),
    /// The identify exchange broke protocol: a chunk arrived out of order, or
    /// the payload exceeded the size limit.
    IdentifyProtocol(String),
    /// The identify payload is not valid zlib data.
    IdentifyCompression(String),
    /// The decompressed identify payload is not valid JSON.
    IdentifyJson(String),
    /// A message could not be registered, encoded, or decoded.
    Msg(MsgError),
    /// A synchronous request/response call failed.
    Call(McuCallError),
    /// The firmware must be reset before this configuration can be sent, and
    /// its only way to do that is the `reset` command — which reboots the MCU
    /// and so drops the connection the handshake is using.
    ///
    /// A firmware that has `config_reset` is reset in place instead, and one
    /// with neither command reports [`McuError::Config`]. The caller of the
    /// handshake handles this by sending `reset`, reconnecting, and retrying
    /// (`mcu/object.rs`).
    ResetRequired,
    /// The MCU configuration phase failed: the configuration is internally
    /// inconsistent (`CrcMismatch`, an exhausted oid range), the MCU is in a
    /// state that cannot be configured, or the build was asked for at the
    /// wrong time (before identify, or twice).
    Config(String),
    /// An I2C bus error: the device reported NACK, timeout, etc.
    I2cBus {
        /// The I2C device oid.
        oid: u8,
        /// The bus status reported by the firmware.
        status: crate::core::klippy::cmd::i2c::I2cBusStatus,
    },
}

impl std::fmt::Display for McuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            McuError::Dictionary(msg) => write!(f, "invalid data dictionary: {}", msg),
            McuError::UnknownMessage(name) => {
                write!(f, "message not in MCU dictionary: {}", name)
            }
            McuError::Decode(msg) => write!(f, "cannot decode response: {}", msg),
            McuError::NotIdentified => {
                write!(f, "MCU has not completed the identify handshake")
            }
            McuError::OldSession(msg) => {
                write!(
                    f,
                    "the firmware was still in an earlier session, which the connection took \
                     over, and the handshake failed ({msg})"
                )
            }
            McuError::IdentifyProtocol(msg) => write!(f, "identify protocol error: {}", msg),
            McuError::IdentifyCompression(msg) => {
                write!(f, "cannot decompress identify payload: {}", msg)
            }
            McuError::IdentifyJson(msg) => write!(f, "cannot parse identify payload: {}", msg),
            McuError::Msg(e) => write!(f, "{}", e),
            McuError::Call(e) => write!(f, "{}", e),
            McuError::ResetRequired => {
                write!(f, "the firmware must be reset with the 'reset' command")
            }
            McuError::Config(msg) => write!(f, "cannot configure MCU: {}", msg),
            McuError::I2cBus { oid, status } => {
                write!(f, "I2C bus error on oid {oid}: {}", status.name())
            }
        }
    }
}

impl std::error::Error for McuError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            McuError::Msg(e) => Some(e),
            McuError::Call(e) => Some(e),
            McuError::Dictionary(_)
            | McuError::UnknownMessage(_)
            | McuError::Decode(_)
            | McuError::NotIdentified
            | McuError::OldSession(_)
            | McuError::IdentifyProtocol(_)
            | McuError::IdentifyCompression(_)
            | McuError::IdentifyJson(_)
            | McuError::ResetRequired
            | McuError::Config(_)
            | McuError::I2cBus { .. } => None,
        }
    }
}

impl From<MsgError> for McuError {
    fn from(e: MsgError) -> Self {
        McuError::Msg(e)
    }
}

impl From<McuCallError> for McuError {
    fn from(e: McuCallError) -> Self {
        McuError::Call(e)
    }
}

impl From<crate::core::klippy::pins::PinError> for McuError {
    /// A pin mistake is a configuration mistake: the resource could not be
    /// built from what the config file said. It surfaces through `build`, which
    /// already reports [`McuError::Config`].
    fn from(e: crate::core::klippy::pins::PinError) -> Self {
        McuError::Config(e.to_string())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mcu_call_error_display() {
        assert_eq!(
            McuCallError::CommandNotFound("get_clock".to_string()).to_string(),
            "command not found: get_clock"
        );
        assert_eq!(
            McuCallError::Timeout("no response".to_string()).to_string(),
            "timeout: no response"
        );
    }

    #[test]
    fn test_mcu_error_from_msg_error() {
        let err: McuError = MsgError::new("unknown message name: nope").into();
        assert!(matches!(err, McuError::Msg(_)));
        assert_eq!(err.to_string(), "unknown message name: nope");
    }

    #[test]
    fn test_mcu_error_from_call_error() {
        let err: McuError = McuCallError::CommandNotFound("x".to_string()).into();
        assert!(matches!(err, McuError::Call(_)));
        assert_eq!(err.to_string(), "command not found: x");
    }

    #[test]
    fn test_mcu_error_dictionary_display() {
        let err = McuError::Dictionary("'commands' must be an object".to_string());
        assert_eq!(
            err.to_string(),
            "invalid data dictionary: 'commands' must be an object"
        );
        assert!(std::error::Error::source(&err).is_none());
    }

    #[test]
    fn test_mcu_error_source_chain() {
        let err: McuError = MsgError::new("boom").into();
        assert!(std::error::Error::source(&err).is_some());
    }

    #[test]
    fn test_mcu_error_typed_message_variants() {
        assert_eq!(
            McuError::UnknownMessage("get_clock".to_string()).to_string(),
            "message not in MCU dictionary: get_clock"
        );
        assert_eq!(
            McuError::Decode("missing parameter 'clock'".to_string()).to_string(),
            "cannot decode response: missing parameter 'clock'"
        );
        assert_eq!(
            McuError::NotIdentified.to_string(),
            "MCU has not completed the identify handshake"
        );
        assert!(McuError::OldSession("timeout: no response".to_string())
            .to_string()
            .starts_with("the firmware was still in an earlier session"),);
        assert!(std::error::Error::source(&McuError::OldSession(String::new())).is_none());
    }

    #[test]
    fn test_mcu_error_identify_variants() {
        assert_eq!(
            McuError::IdentifyProtocol("offset mismatch".to_string()).to_string(),
            "identify protocol error: offset mismatch"
        );
        assert_eq!(
            McuError::IdentifyCompression("bad header".to_string()).to_string(),
            "cannot decompress identify payload: bad header"
        );
        assert_eq!(
            McuError::IdentifyJson("expected value".to_string()).to_string(),
            "cannot parse identify payload: expected value"
        );
    }
}
