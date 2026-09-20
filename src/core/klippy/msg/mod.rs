pub mod error;
pub mod param;
pub mod parser;
pub mod proto;

use std::sync::{Arc, Mutex};

// Re-export commonly-used items for convenience
pub use error::{MsgError, MsgResult};
pub use param::Param;
pub use proto::{ArgType, ArgValue};

// ===========================================================================
// Msg — Klipper message definition with optional callback.
// ===========================================================================

/// Callback type for message handlers.
pub type MsgCallback = Arc<Mutex<Box<dyn FnMut(&[ArgValue]) + Send>>>;

/// A complete message definition with id, name, parameters, and optional callback.
#[derive(Clone)]
pub struct Msg {
    /// Message ID (host-side id from the firmware identify response).
    ///
    /// Klipper encodes ids as a signed VLQ, so ids for encoded values in
    /// `0x60..=0x7f` and `>= 0x3000` appear here as negative numbers; `i16`
    /// covers the full range (`-4096..=12287`).
    pub id: i16,
    /// Message name (string identifier).
    pub name: String,
    /// Parameter list in declaration order: (parameter_name, parameter_type).
    pub params: Vec<(String, ArgType)>,
    /// Optional callback invoked when this command is matched.
    pub callback: Option<MsgCallback>,
}

impl std::fmt::Debug for Msg {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Msg")
            .field("id", &self.id)
            .field("name", &self.name)
            .field("params", &self.params)
            .field("callback", &self.callback.is_some())
            .finish()
    }
}

impl PartialEq for Msg {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id && self.name == other.name && self.params == other.params
    }
}

impl Eq for Msg {}

impl std::hash::Hash for Msg {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.id.hash(state);
        self.name.hash(state);
        for (name, atype) in &self.params {
            name.hash(state);
            std::mem::discriminant(atype).hash(state);
        }
    }
}

impl Msg {
    /// Parse a Klipper message format string into a `Msg`.
    ///
    /// Format strings are space-separated tokens where the first token is the
    /// command name and remaining tokens are `name=type` pairs.
    /// The `id` defaults to 0 and `callback` is `None`.
    ///
    /// Returns `MsgError` if the format string is empty or malformed.
    pub fn parse(id: i16, fmt: &str) -> MsgResult<Self> {
        let trimmed = fmt.trim();
        if trimmed.is_empty() {
            return Err(MsgError::new("empty format string"));
        }
        let mut parts = trimmed.split_whitespace();
        let name = parts
            .next()
            .ok_or_else(|| MsgError::new("missing command name"))?
            .to_string();
        let mut params = Vec::new();
        for part in parts {
            let eq = part
                .find('=')
                .ok_or_else(|| MsgError::new(format!("invalid parameter format: {}", part)))?;
            let param_name = part[..eq].to_string();
            let typ = &part[eq + 1..];
            let arg_type = ArgType::parse_format(typ)
                .ok_or_else(|| MsgError::new(format!("unknown type specifier: {}", typ)))?;
            params.push((param_name, arg_type));
        }
        Ok(Self {
            id,
            name,
            params,
            callback: None,
        })
    }

    /// Create a `Msg` from components.
    pub fn new(id: i16, name: impl Into<String>, params: Vec<(String, ArgType)>) -> Self {
        Self {
            id,
            name: name.into(),
            params,
            callback: None,
        }
    }

    /// Returns the parameter list.
    pub fn params(&self) -> &[(String, ArgType)] {
        &self.params
    }

    /// Returns the format string by reconstructing it from name and params.
    ///
    /// Note: `%*s` normalizes to `%s`, since [`ArgType`] does not distinguish
    /// between the two string variants. Command names with no parameters are
    /// returned as just the name.
    pub fn format(&self) -> String {
        let params = self
            .params
            .iter()
            .map(|(name, atype)| format!("{}={}", name, atype.format_str()))
            .collect::<Vec<_>>()
            .join(" ");
        if params.is_empty() {
            self.name.clone()
        } else {
            format!("{} {}", self.name, params)
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Msg::parse
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_all_arg_types() {
        let msg = Msg::parse(1, "test_cmd a=%u b=%i c=%hu d=%hi e=%s f=%c g=%*s h=%.*s").unwrap();
        assert_eq!(msg.name, "test_cmd");
        assert_eq!(msg.params.len(), 8);
        assert_eq!(msg.params[0].1, ArgType::UInt32);
        assert_eq!(msg.params[1].1, ArgType::Int32);
        assert_eq!(msg.params[2].1, ArgType::UInt16);
        assert_eq!(msg.params[3].1, ArgType::Int16);
        assert_eq!(msg.params[4].1, ArgType::Str);
        assert_eq!(msg.params[5].1, ArgType::UInt8);
        assert_eq!(msg.params[6].1, ArgType::Str);
        assert_eq!(msg.params[7].1, ArgType::Bytes);
    }

    #[test]
    fn test_parse_error_empty_string() {
        let result = Msg::parse(1, "");
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().msg, "empty format string");
    }

    #[test]
    fn test_parse_error_invalid_param() {
        let result = Msg::parse(1, "CMD badparam");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("invalid parameter format"));
    }

    #[test]
    fn test_parse_error_unknown_type() {
        let result = Msg::parse(1, "CMD x=%x");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("unknown type specifier"));
    }

    #[test]
    fn test_format_all_types() {
        let params = vec![
            ("a".to_string(), ArgType::UInt32),
            ("b".to_string(), ArgType::Int32),
            ("c".to_string(), ArgType::UInt16),
            ("d".to_string(), ArgType::Int16),
            ("e".to_string(), ArgType::Str),
            ("f".to_string(), ArgType::Bytes),
        ];
        let msg = Msg::new(1, "test", params);
        assert_eq!(msg.format(), "test a=%u b=%i c=%hu d=%hi e=%s f=%.*s");
    }

    #[test]
    fn test_format_roundtrip() {
        for fmt in [
            "NOP",
            "CMD x=%u",
            "COMBINED a=%u b=%i c=%hu d=%hi e=%s f=%.*s",
        ] {
            let msg = Msg::parse(1, fmt).unwrap();
            assert_eq!(msg.format(), fmt);
        }
    }

    // -----------------------------------------------------------------------
    // Msg Hash (for use in collections)
    // -----------------------------------------------------------------------

    // `Msg`'s only interior mutability is the callback, which the manual
    // `Eq`/`Hash` ignore, so using it as a key is well defined.
    #[allow(clippy::mutable_key_type)]
    #[test]
    fn test_command_hash() {
        use std::collections::HashSet;

        let msg1 = Msg::new(1, "test", vec![("x".to_string(), ArgType::Int32)]);
        let msg2 = Msg::new(1, "test", vec![("x".to_string(), ArgType::Int32)]);

        let mut set = HashSet::new();
        set.insert(msg1);
        assert!(set.contains(&msg2));
    }
}
