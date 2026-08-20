// Klipper device communication errors
//
// This module defines error types used throughout the Klippy device
// communication system.



/// Klipper device communication errors
#[derive(Debug)]
pub enum KlippyError {
    /// Connection error
    Connection(String),
    /// Protocol error
    Protocol(String),
    /// Request error
    Request(String),
    /// Response parsing error
    Parse(String),
    /// Internal error
    Internal(String),
}

impl std::fmt::Display for KlippyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KlippyError::Connection(msg) => write!(f, "Connection: {}", msg),
            KlippyError::Protocol(msg) => write!(f, "Protocol: {}", msg),
            KlippyError::Request(msg) => write!(f, "Request: {}", msg),
            KlippyError::Parse(msg) => write!(f, "Parse: {}", msg),
            KlippyError::Internal(msg) => write!(f, "Internal: {}", msg),
        }
    }
}

impl std::error::Error for KlippyError {}

impl From<serde_json::Error> for KlippyError {
    fn from(e: serde_json::Error) -> Self {
        KlippyError::Parse(e.to_string())
    }
}
