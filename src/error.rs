use std::fmt;

/// Application-level error type
#[derive(Debug)]
pub enum KlipperXError {
    /// Configuration error
    Config(String),
    /// I/O error
    Io(std::io::Error),
    /// Connection error
    Connection(String),
    /// Protocol error
    Protocol(String),
    /// Custom error
    Custom(String),
}

impl fmt::Display for KlipperXError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            KlipperXError::Config(msg) => write!(f, "Config: {}", msg),
            KlipperXError::Io(err) => write!(f, "IO: {}", err),
            KlipperXError::Connection(msg) => write!(f, "Connection: {}", msg),
            KlipperXError::Protocol(msg) => write!(f, "Protocol: {}", msg),
            KlipperXError::Custom(msg) => write!(f, "{}", msg),
        }
    }
}

impl std::error::Error for KlipperXError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            KlipperXError::Io(err) => Some(err),
            _ => None,
        }
    }
}

impl From<std::io::Error> for KlipperXError {
    fn from(err: std::io::Error) -> Self {
        KlipperXError::Io(err)
    }
}
