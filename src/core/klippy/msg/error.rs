// ===========================================================================
// Error & Result types for the Klipper message protocol.
// ===========================================================================

/// Error raised by command / protocol operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MsgError {
    pub msg: String,
}

impl MsgError {
    pub fn new(msg: impl Into<String>) -> Self {
        Self { msg: msg.into() }
    }
}

impl std::fmt::Display for MsgError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.msg)
    }
}

impl std::error::Error for MsgError {}

impl From<serde_json::Error> for MsgError {
    fn from(e: serde_json::Error) -> Self {
        MsgError::new(e.to_string())
    }
}

/// Result type used for command / protocol operations.
pub type MsgResult<T> = Result<T, MsgError>;
