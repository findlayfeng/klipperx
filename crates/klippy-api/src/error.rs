//! Errors from the socket, rather than from a request.
//!
//! These are the failures that happen *around* the protocol: a listener that
//! could not be created, a server that could not be reached, a connection that
//! went away. A request-level failure is a different thing entirely — it is
//! [`ApiError`](crate::ApiError), which the protocol has a wire format for and
//! which a client is entitled to see. Nothing here is ever sent to a client:
//! when one of these happens there is nobody left to tell.
//!
//! The messages are built where the context is — the address that could not be
//! bound, the `io::Error` that came out of the syscall — so the variants exist
//! for a caller to branch on, not for them to compose a sentence out of.

use std::fmt;

/// A failure of the socket itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransportError {
    /// The listener could not be created, with the address and the cause.
    ///
    /// Reported where the address is chosen, so a server that cannot start says
    /// why before anything is served.
    Bind(String),
    /// The API server could not be reached, with the address and the cause.
    Connect(String),
    /// The peer closed the connection.
    ///
    /// Not an error on either side's part: it is how a server reports a
    /// shutdown, and how a client reports that its server is gone.
    Closed,
    /// A read or write on an established connection failed.
    Io(String),
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransportError::Bind(message) | TransportError::Connect(message) => {
                f.write_str(message)
            }
            TransportError::Closed => f.write_str("the API server closed the connection"),
            TransportError::Io(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for TransportError {}
