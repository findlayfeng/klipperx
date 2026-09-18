/// Error type for interface receive operations.
///
/// Represents various failure modes that can occur when receiving
/// data from a Klipper device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InterfaceError {
    /// Connection is not established or has been lost
    ConnectionLost,
    /// Timeout waiting for a response
    Timeout,
    /// Invalid or malformed data received
    InvalidData,
    /// Generic error with an optional message
    SendError(String),
    Other(String),
}

impl std::fmt::Display for InterfaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InterfaceError::ConnectionLost => write!(f, "Connection lost"),
            InterfaceError::Timeout => write!(f, "Receive timeout"),
            InterfaceError::InvalidData => write!(f, "Invalid data received"),
            InterfaceError::SendError(msg) => write!(f, "Send error: {}", msg),
            InterfaceError::Other(msg) => write!(f, "{}", msg),
        }
    }
}

impl std::error::Error for InterfaceError {}
