//! Where the API server is.
//!
//! Upstream klippy takes one option, `-a/--api-server`, holding the filename of
//! a Unix Domain Socket — and it has **no default**: without the option, no
//! server is created at all. That stays the default here, because a host that
//! silently listens somewhere is a host that surprises its operator.
//!
//! This host also accepts a TCP address, so a client on another machine can
//! reach the API without a socket tunnel. Both forms live in one option rather
//! than two, because a target is either one or the other and never both.
//!
//! | Value | The API server is at |
//! |---|---|
//! | `/tmp/klippy_uds` | a Unix Domain Socket at that path (upstream's form) |
//! | `unix:/tmp/klippy_uds` | the same, written explicitly |
//! | `tcp:127.0.0.1:7125` | TCP |
//! | `tcp://[::1]:7125` | TCP, IPv6 |
//! | `127.0.0.1:7125` | TCP (bare `host:port` shorthand) |
//!
//! A bare value is a socket path, as upstream has it. The two exceptions are a
//! bare `host:port` — a filename containing `:` is not something anyone wants,
//! and a host and port is unambiguous — and a `scheme://` prefix this module
//! does not know, which is rejected rather than turned into a filename that
//! would then fail to bind with a confusing message. `http://127.0.0.1:7125`
//! is the case that matters: that is Moonraker's own port, not this socket, and
//! it used to be the default here.
//!
//! Both directions of the API meet here: [`Server`](super::Server) binds the
//! target, and [`ApiTarget::connect`] dials it. Everything above the socket is
//! written against [`Transport`], which is the one thing a listener and a
//! dialer have in common.

use std::fmt;
use std::path::{Path, PathBuf};
use std::str::FromStr;

use tokio::io::{AsyncRead, AsyncWrite};
use tokio::net::{TcpStream, UnixStream};

use crate::core::klippy::error::KlippyError;

/// A read/write API socket, whichever transport it arrived on.
///
/// The two socket types have nothing in common but their traits, so everything
/// above the socket — framing, requests, dispatch, a connection's read and write
/// halves — is written against this instead of matching on the transport over
/// and over. The server gets one from accepting a connection; a client gets one
/// from [`ApiTarget::connect`].
pub trait Transport: AsyncRead + AsyncWrite + Unpin + Send {}

impl<T: AsyncRead + AsyncWrite + Unpin + Send> Transport for T {}

/// Where the API server is, as parsed from the `--api-server` option.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ApiTarget {
    /// A Unix Domain Socket at this filesystem path.
    Unix(PathBuf),
    /// A TCP listen address, still unresolved: `host:port`, so that a name
    /// like `localhost` is resolved when the listener is bound, not when the
    /// option is parsed.
    Tcp(String),
}

impl ApiTarget {
    /// Whether this target is a Unix Domain Socket.
    pub fn is_unix(&self) -> bool {
        matches!(self, ApiTarget::Unix(_))
    }

    /// The socket path, if this is a Unix Domain Socket target.
    pub fn as_unix(&self) -> Option<&Path> {
        match self {
            ApiTarget::Unix(path) => Some(path),
            ApiTarget::Tcp(_) => None,
        }
    }

    /// The unresolved TCP address, if this is a TCP target.
    pub fn as_tcp(&self) -> Option<&str> {
        match self {
            ApiTarget::Tcp(address) => Some(address),
            ApiTarget::Unix(_) => None,
        }
    }

    /// Dial the API server at this target.
    ///
    /// # Errors
    /// Returns [`KlippyError::Connection`] if the socket cannot be reached —
    /// the usual cause being that no API server is listening there.
    pub async fn connect(&self) -> Result<Box<dyn Transport>, KlippyError> {
        match self {
            ApiTarget::Unix(path) => {
                let stream = UnixStream::connect(path).await.map_err(|err| {
                    KlippyError::Connection(format!(
                        "cannot connect to unix socket {}: {err}",
                        path.display()
                    ))
                })?;
                Ok(Box::new(stream))
            }
            ApiTarget::Tcp(address) => {
                let stream = TcpStream::connect(address).await.map_err(|err| {
                    KlippyError::Connection(format!("cannot connect to tcp {address}: {err}"))
                })?;
                // Small requests, sent as soon as they exist: Nagle would only
                // delay them behind the acknowledgment of the previous one.
                let _ = stream.set_nodelay(true);
                Ok(Box::new(stream))
            }
        }
    }
}

impl fmt::Display for ApiTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ApiTarget::Unix(path) => write!(f, "unix:{}", path.display()),
            ApiTarget::Tcp(address) => write!(f, "tcp:{address}"),
        }
    }
}

/// An `--api-server` value that does not name a place to listen.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AddressError {
    /// The value was empty.
    Empty,
    /// A `unix:` prefix was used with no path after it.
    EmptySocketPath,
    /// A TCP address has no `:port`.
    MissingPort(String),
    /// A TCP address has a `:port` that is not a port number.
    InvalidPort(String),
    /// A `scheme://` prefix other than `unix` or `tcp`.
    UnknownScheme(String),
}

impl fmt::Display for AddressError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AddressError::Empty => f.write_str("--api-server needs an address"),
            AddressError::EmptySocketPath => {
                f.write_str("--api-server: 'unix:' needs a socket path after it")
            }
            AddressError::MissingPort(address) => write!(
                f,
                "--api-server: '{address}' needs a port, as in '{address}:7125'"
            ),
            AddressError::InvalidPort(address) => {
                write!(f, "--api-server: '{address}' does not end in a port number")
            }
            AddressError::UnknownScheme(scheme) => write!(
                f,
                "--api-server: unknown scheme '{scheme}://' (use a socket path, or 'tcp:host:port')"
            ),
        }
    }
}

impl std::error::Error for AddressError {}

impl FromStr for ApiTarget {
    type Err = AddressError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        let value = value.trim();
        if value.is_empty() {
            return Err(AddressError::Empty);
        }

        if let Some(rest) = strip_scheme(value, "unix") {
            if rest.is_empty() {
                return Err(AddressError::EmptySocketPath);
            }
            return Ok(ApiTarget::Unix(PathBuf::from(rest)));
        }
        if let Some(rest) = strip_scheme(value, "tcp") {
            return Ok(ApiTarget::Tcp(check_tcp(rest)?));
        }

        // A scheme we do not know is a mistake worth naming, not a filename.
        if let Some((scheme, _)) = value.split_once("://") {
            return Err(AddressError::UnknownScheme(scheme.to_string()));
        }

        // A bare `host:port` is TCP; anything else is a socket path, as upstream.
        if !value.starts_with('/') && value.contains(':') {
            return Ok(ApiTarget::Tcp(check_tcp(value)?));
        }
        Ok(ApiTarget::Unix(PathBuf::from(value)))
    }
}

/// Strip `scheme:` or `scheme://` from `value`.
fn strip_scheme<'a>(value: &'a str, scheme: &str) -> Option<&'a str> {
    let rest = value.strip_prefix(scheme)?;
    rest.strip_prefix("://").or_else(|| rest.strip_prefix(':'))
}

/// Check that `address` ends in a port number.
///
/// Only the port is checked. Whether the host part resolves is the resolver's
/// business, and it is also what lets `localhost:7125` stay unresolved here and
/// be looked up when the listener is bound.
fn check_tcp(address: &str) -> Result<String, AddressError> {
    if address.is_empty() {
        return Err(AddressError::Empty);
    }
    let (_, port) = address
        .rsplit_once(':')
        .ok_or_else(|| AddressError::MissingPort(address.to_string()))?;
    if port.is_empty() {
        return Err(AddressError::MissingPort(address.to_string()));
    }
    port.parse::<u16>()
        .map(|_| address.to_string())
        .map_err(|_| AddressError::InvalidPort(address.to_string()))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(value: &str) -> ApiTarget {
        value.parse().expect("test value parses")
    }

    #[test]
    fn test_a_bare_value_is_a_socket_path() {
        // Upstream's form, and the reason a bare value must stay a path.
        assert_eq!(
            parse("/tmp/klippy_uds"),
            ApiTarget::Unix(PathBuf::from("/tmp/klippy_uds"))
        );
        assert_eq!(
            parse("klippy_uds"),
            ApiTarget::Unix(PathBuf::from("klippy_uds"))
        );
        assert!(parse("/tmp/klippy_uds").is_unix());
        assert_eq!(
            parse("/tmp/klippy_uds").as_unix(),
            Some(Path::new("/tmp/klippy_uds"))
        );
        assert_eq!(parse("/tmp/klippy_uds").as_tcp(), None);
    }

    #[test]
    fn test_the_unix_scheme_may_be_written_explicitly() {
        for value in ["unix:/tmp/klippy_uds", "unix:///tmp/klippy_uds"] {
            assert_eq!(
                parse(value),
                ApiTarget::Unix(PathBuf::from("/tmp/klippy_uds")),
                "{value}"
            );
        }
    }

    #[test]
    fn test_a_tcp_target_is_recognised_however_it_is_written() {
        for value in [
            "tcp:127.0.0.1:7125",
            "tcp://127.0.0.1:7125",
            "127.0.0.1:7125",
        ] {
            assert_eq!(
                parse(value),
                ApiTarget::Tcp("127.0.0.1:7125".to_string()),
                "{value}"
            );
        }
        // A name is kept unresolved so the listener resolves it.
        assert_eq!(
            parse("tcp:localhost:7125"),
            ApiTarget::Tcp("localhost:7125".to_string())
        );
        assert_eq!(
            parse("[::1]:7125"),
            ApiTarget::Tcp("[::1]:7125".to_string())
        );
        assert_eq!(parse(":7125"), ApiTarget::Tcp(":7125".to_string()));
    }

    #[test]
    fn test_display_names_the_transport() {
        // Log lines have to say which kind of listener is up.
        assert_eq!(parse("/tmp/klippy_uds").to_string(), "unix:/tmp/klippy_uds");
        assert_eq!(
            parse("tcp:127.0.0.1:7125").to_string(),
            "tcp:127.0.0.1:7125"
        );
    }

    #[test]
    fn test_an_empty_value_is_rejected() {
        assert_eq!("".parse::<ApiTarget>().unwrap_err(), AddressError::Empty);
        assert_eq!("   ".parse::<ApiTarget>().unwrap_err(), AddressError::Empty);
        assert_eq!(
            "unix:".parse::<ApiTarget>().unwrap_err(),
            AddressError::EmptySocketPath
        );
    }

    #[test]
    fn test_a_tcp_target_must_end_in_a_port() {
        // `tcp:` is explicit, so it cannot fall back to being a filename.
        assert_eq!(
            "tcp:127.0.0.1".parse::<ApiTarget>().unwrap_err(),
            AddressError::MissingPort("127.0.0.1".to_string())
        );
        assert_eq!(
            "tcp:127.0.0.1:http".parse::<ApiTarget>().unwrap_err(),
            AddressError::InvalidPort("127.0.0.1:http".to_string())
        );
        assert_eq!(
            "tcp:127.0.0.1:".parse::<ApiTarget>().unwrap_err(),
            AddressError::MissingPort("127.0.0.1:".to_string())
        );
    }

    #[test]
    fn test_an_unknown_scheme_is_named_rather_than_taken_as_a_path() {
        // The old default was Moonraker's HTTP address; catching it is the point.
        assert_eq!(
            "http://127.0.0.1:7125".parse::<ApiTarget>().unwrap_err(),
            AddressError::UnknownScheme("http".to_string())
        );
        assert!(AddressError::UnknownScheme("http".to_string())
            .to_string()
            .contains("unknown scheme 'http://'"));
    }

    #[test]
    fn test_error_text_says_what_to_write_instead() {
        assert_eq!(
            AddressError::MissingPort("127.0.0.1".to_string()).to_string(),
            "--api-server: '127.0.0.1' needs a port, as in '127.0.0.1:7125'"
        );
        assert_eq!(
            AddressError::Empty.to_string(),
            "--api-server needs an address"
        );
    }
}
