pub mod error;
pub mod param;
pub mod parser;
pub mod proto;

// Re-export commonly-used items for convenience
pub use error::{MsgError, MsgResult};
pub use param::Param;
pub use proto::{ArgType, ArgValue};

// ===========================================================================
// MsgDef handling for Klipper message protocol.
//
// Provides the `MsgDef` struct — a parsed command containing the name
// and parameter list. Encoding/decoding logic is in `proto.rs`.
// ===========================================================================



/// A parsed command format containing only the name and parameter list.
///
/// Encoding/decoding methods are provided as standalone functions in `proto.rs`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct MsgDef {
    /// Parameter list in declaration order: (parameter_name, parameter_type).
    pub params: Vec<(String, ArgType)>,
}

impl MsgDef {
    /// Parse a Klipper message format string into a `MsgDef`.
    ///
    /// Format strings are space-separated tokens where the first token is the
    /// command name and remaining tokens are `name=type` pairs.
    ///
    /// Returns `MsgError` if the format string is empty or malformed.
    pub fn parse(fmt: &str) -> MsgResult<(String, Self)> {
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
            let arg_type = ArgType::parse_format(typ).map_err(|_| {
                MsgError::new(format!("unknown type specifier: {}", typ))
            })?;
            params.push((param_name, arg_type));
        }
        Ok((name, Self { params }))
    }

    /// Create a `MsgDef` from a parameter list.
    pub fn new(params: Vec<(String, ArgType)>) -> Self {
        Self { params }
    }

    /// Returns the parameter list.
    pub fn params(&self) -> &[(String, ArgType)] {
        &self.params
    }

    /// Returns the format string by reconstructing it from params.
    ///
    /// Note: `%*s` normalizes to `%s`, and `%.*s` normalizes to `%c` here,
    /// since [`ArgType`] does not distinguish between these variants.
    pub fn format(&self) -> String {
        self.params
            .iter()
            .map(|(name, atype)| format!("{}={}", name, atype.format_str()))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // MsgDef::parse
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_all_arg_types() {
        let fmt = "test_cmd a=%u b=%i c=%hu d=%hi e=%s f=%c g=%*s h=%.*s";
        let (name, cmd) = MsgDef::parse(fmt).unwrap();
        assert_eq!(name, "test_cmd");
        assert_eq!(cmd.params.len(), 8);
        assert_eq!(cmd.params[0].1, ArgType::UInt32);
        assert_eq!(cmd.params[1].1, ArgType::Int32);
        assert_eq!(cmd.params[2].1, ArgType::UInt16);
        assert_eq!(cmd.params[3].1, ArgType::Int16);
        assert_eq!(cmd.params[4].1, ArgType::Str);
        assert_eq!(cmd.params[5].1, ArgType::UInt8);
        assert_eq!(cmd.params[6].1, ArgType::Str);
        assert_eq!(cmd.params[7].1, ArgType::Bytes);
    }

    #[test]
    fn test_parse_error_empty_string() {
        let result = MsgDef::parse("");
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().msg, "empty format string");
    }

    #[test]
    fn test_parse_error_invalid_param() {
        let result = MsgDef::parse("CMD badparam");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("invalid parameter format"));
    }

    #[test]
    fn test_parse_error_unknown_type() {
        let result = MsgDef::parse("CMD x=%x");
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
        let cmd = MsgDef::new(params);
        assert_eq!(cmd.format(), "a=%u b=%i c=%hu d=%hi e=%s f=%.*s");
    }

    // -----------------------------------------------------------------------
    // MsgDef Hash (for use in collections)
    // -----------------------------------------------------------------------

    #[test]
    fn test_command_def_hash() {
        use std::collections::HashSet;

        let cmd1 = MsgDef::new(vec![("x".to_string(), ArgType::Int32)]);
        let cmd2 = MsgDef::new(vec![("x".to_string(), ArgType::Int32)]);

        let mut set = HashSet::new();
        set.insert(cmd1);
        assert!(set.contains(&cmd2));
    }
}
