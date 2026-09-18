//! Where the API server listens.
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
//! | Value | Listens on |
//! |---|---|
//! | `/tmp/klippy_uds` | Unix Domain Socket at that path (upstream's form) |
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

use std::fmt;
use std::path::{Path, PathBuf};
use std::str::FromStr;

/// Where to listen, as parsed from the `--api-server` option.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ListenTarget {
    /// A Unix Domain Socket at this filesystem path.
    Unix(PathBuf),
    /// A TCP listen address, still unresolved: `host:port`, so that a name
    /// like `localhost` is resolved when the listener is bound, not when the
    /// option is parsed.
    Tcp(String),
}

impl ListenTarget {
    /// Whether this target is a Unix Domain Socket.
    pub fn is_unix(&self) -> bool {
        matches!(self, ListenTarget::Unix(_))
    }

    /// The socket path, if this is a Unix Domain Socket target.
    pub fn as_unix(&self) -> Option<&Path> {
        match self {
            ListenTarget::Unix(path) => Some(path),
            ListenTarget::Tcp(_) => None,
        }
    }

    /// The unresolved TCP address, if this is a TCP target.
    pub fn as_tcp(&self) -> Option<&str> {
        match self {
            ListenTarget::Tcp(address) => Some(address),
            ListenTarget::Unix(_) => None,
        }
    }
}

impl fmt::Display for ListenTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ListenTarget::Unix(path) => write!(f, "unix:{}", path.display()),
            ListenTarget::Tcp(address) => write!(f, "tcp:{address}"),
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

impl FromStr for ListenTarget {
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
            return Ok(ListenTarget::Unix(PathBuf::from(rest)));
        }
        if let Some(rest) = strip_scheme(value, "tcp") {
            return Ok(ListenTarget::Tcp(check_tcp(rest)?));
        }

        // A scheme we do not know is a mistake worth naming, not a filename.
        if let Some((scheme, _)) = value.split_once("://") {
            return Err(AddressError::UnknownScheme(scheme.to_string()));
        }

        // A bare `host:port` is TCP; anything else is a socket path, as upstream.
        if !value.starts_with('/') && value.contains(':') {
            return Ok(ListenTarget::Tcp(check_tcp(value)?));
        }
        Ok(ListenTarget::Unix(PathBuf::from(value)))
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

    fn parse(value: &str) -> ListenTarget {
        value.parse().expect("test value parses")
    }

    #[test]
    fn test_a_bare_value_is_a_socket_path() {
        // Upstream's form, and the reason a bare value must stay a path.
        assert_eq!(
            parse("/tmp/klippy_uds"),
            ListenTarget::Unix(PathBuf::from("/tmp/klippy_uds"))
        );
        assert_eq!(
            parse("klippy_uds"),
            ListenTarget::Unix(PathBuf::from("klippy_uds"))
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
                ListenTarget::Unix(PathBuf::from("/tmp/klippy_uds")),
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
                ListenTarget::Tcp("127.0.0.1:7125".to_string()),
                "{value}"
            );
        }
        // A name is kept unresolved so the listener resolves it.
        assert_eq!(
            parse("tcp:localhost:7125"),
            ListenTarget::Tcp("localhost:7125".to_string())
        );
        assert_eq!(
            parse("[::1]:7125"),
            ListenTarget::Tcp("[::1]:7125".to_string())
        );
        assert_eq!(parse(":7125"), ListenTarget::Tcp(":7125".to_string()));
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
        assert_eq!("".parse::<ListenTarget>().unwrap_err(), AddressError::Empty);
        assert_eq!(
            "   ".parse::<ListenTarget>().unwrap_err(),
            AddressError::Empty
        );
        assert_eq!(
            "unix:".parse::<ListenTarget>().unwrap_err(),
            AddressError::EmptySocketPath
        );
    }

    #[test]
    fn test_a_tcp_target_must_end_in_a_port() {
        // `tcp:` is explicit, so it cannot fall back to being a filename.
        assert_eq!(
            "tcp:127.0.0.1".parse::<ListenTarget>().unwrap_err(),
            AddressError::MissingPort("127.0.0.1".to_string())
        );
        assert_eq!(
            "tcp:127.0.0.1:http".parse::<ListenTarget>().unwrap_err(),
            AddressError::InvalidPort("127.0.0.1:http".to_string())
        );
        assert_eq!(
            "tcp:127.0.0.1:".parse::<ListenTarget>().unwrap_err(),
            AddressError::MissingPort("127.0.0.1:".to_string())
        );
    }

    #[test]
    fn test_an_unknown_scheme_is_named_rather_than_taken_as_a_path() {
        // The old default was Moonraker's HTTP address; catching it is the point.
        assert_eq!(
            "http://127.0.0.1:7125".parse::<ListenTarget>().unwrap_err(),
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
