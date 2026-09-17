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
            McuError::IdentifyProtocol(msg) => write!(f, "identify protocol error: {}", msg),
            McuError::IdentifyCompression(msg) => {
                write!(f, "cannot decompress identify payload: {}", msg)
            }
            McuError::IdentifyJson(msg) => write!(f, "cannot parse identify payload: {}", msg),
            McuError::Msg(e) => write!(f, "{}", e),
            McuError::Call(e) => write!(f, "{}", e),
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
            | McuError::IdentifyProtocol(_)
            | McuError::IdentifyCompression(_)
            | McuError::IdentifyJson(_) => None,
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
