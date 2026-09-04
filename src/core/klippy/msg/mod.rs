pub mod error;
pub mod param;
pub mod parser;
pub mod proto;

// Re-export commonly-used items for convenience
pub use error::{MsgError, MsgResult};
pub use param::Param;
pub use proto::{ArgType, ArgValue};

// ===========================================================================
// MsgBase handling for Klipper message protocol.
//
// Provides the `MsgBase` struct — a parsed command containing the name
// and parameter list. Encoding/decoding logic is in `proto.rs`.
// Also provides `MsgHandler` — a `MsgBase` with an optional callback
// for handling matched messages.
// ===========================================================================



// ===========================================================================
// MsgBase
// ===========================================================================

/// A parsed command format containing only the name and parameter list.
///
/// Encoding/decoding methods are provided as standalone functions in `proto.rs`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct MsgBase {
    /// Parameter list in declaration order: (parameter_name, parameter_type).
    pub params: Vec<(String, ArgType)>,
}

impl MsgBase {
    /// Parse a Klipper message format string into a `MsgBase`.
    ///
    /// Format strings are space-separated tokens where the first token is the
    /// command name and remaining tokens are `name=type` pairs.
    ///
    /// Returns `MsgError` if the format string is empty or malformed.
    ///
    /// # Examples
    /// ```
    /// # use klipperx::core::klippy::msg::MsgBase;
    /// let (name, cmd) = MsgBase::parse("config_digital_out oid=%u pin=%s").unwrap();
    /// assert_eq!(name, "config_digital_out");
    /// assert_eq!(cmd.params().len(), 2);
    /// ```
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
            let arg_type = match typ {
                "%u" => ArgType::UInt32,
                "%i" => ArgType::Int32,
                "%hu" => ArgType::UInt16,
                "%hi" => ArgType::Int16,
                "%c" => ArgType::Bytes,
                "%s" | "%*s" | "%.*s" => ArgType::Str,
                _ => {
                    return Err(MsgError::new(format!(
                        "unknown type specifier: {}",
                        typ
                    )))
                }
            };
            params.push((param_name, arg_type));
        }
        Ok((name, Self { params }))
    }

    /// Create a `MsgBase` from components.
    pub fn new(params: Vec<(String, ArgType)>) -> Self {
        Self { params }
    }

    /// Returns the parameter list.
    pub fn params(&self) -> &[(String, ArgType)] {
        &self.params
    }

    /// Returns the format string by reconstructing it from name and params.
    ///
    /// Note: the Str variants `%*s` and `%.*s` normalize to `%s` here, since
    /// [`ArgType`] does not distinguish them.
    pub fn format(&self) -> String {
        let mut parts = Vec::new();
        for (name, atype) in &self.params {
            let typ = match atype {
                ArgType::UInt32 => "%u",
                ArgType::Int32 => "%i",
                ArgType::UInt16 => "%hu",
                ArgType::Int16 => "%hi",
                ArgType::Str => "%s",
                ArgType::Bytes => "%c",
            };
            parts.push(format!("{}={}", name, typ));
        }
        parts.join(" ")
    }
}

// ===========================================================================
// MsgHandler
// ===========================================================================

/// A `MsgBase` with a callback for handling matched messages.
///
/// `MsgHandler` wraps a `MsgBase` and provides callback functionality.
/// It implements `Deref<Target = MsgBase>` so all `MsgBase` methods are
/// directly accessible.
///
/// # Example
/// ```
/// # use klipperx::core::klippy::msg::{MsgBase, MsgHandler, ArgValue};
/// let (name, command) = MsgBase::parse("config_digital_out oid=%u pin=%s").unwrap();
/// let handler = MsgHandler::new(command, |values| {
///     println!("command received with {} params", values.len());
/// });
/// assert_eq!(name, "config_digital_out");
/// ```
pub struct MsgHandler {
    /// The underlying command definition.
    msg: MsgBase,
    /// Callback invoked when this command is matched.
    callback: Box<dyn FnMut(&[ArgValue]) + Send>,
}

impl std::fmt::Debug for MsgHandler {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MsgHandler")
            .field("msg", &self.msg)
            .finish()
    }
}

impl MsgHandler {
    /// Create a new `MsgHandler` from a `MsgBase` and a callback.
    ///
    /// The callback receives a slice of `ArgValue` containing all decoded
    /// parameter values in command definition order. Use
    /// `values[i].clone()` or pattern matching to extract values.
    pub fn new(
        msg: MsgBase,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> Self {
        Self { msg,
            callback: Box::new(callback),
        }
    }

    /// Returns a reference to the underlying `MsgBase`.
    pub fn msg(&self) -> &MsgBase {
        &self.msg
    }

    /// Set or replace the callback function.
    ///
    /// The callback receives a slice of `ArgValue` containing all decoded
    /// parameter values in command definition order.
    pub fn set_callback(&mut self, callback: impl FnMut(&[ArgValue]) + Send + 'static) {
        self.callback = Box::new(callback);
    }

    /// Invoke the callback with the given parameters.
    pub fn invoke_callback(&mut self, params: &[ArgValue]) {
        (self.callback)(params);
    }
}

impl std::ops::Deref for MsgHandler {
    type Target = MsgBase;

    fn deref(&self) -> &Self::Target {
        &self.msg
    }
}

/// A command entry that wraps a parsed command format.
#[derive(Debug)]
pub enum MsgEntry {
    /// A base command format (no callback).
    Base(MsgBase),
    /// A command handler with a callback.
    Handler(MsgHandler),
}

impl From<MsgHandler> for MsgEntry {
    fn from(handler: MsgHandler) -> Self {
        Self::Handler(handler)
    }
}

impl From<MsgBase> for MsgEntry {
    fn from(base: MsgBase) -> Self {
        Self::Base(base)
    }
}

impl MsgEntry {
    /// Attach a callback to this entry.
    ///
    /// A `Base` entry is converted into a `Handler`. If the entry is already
    /// a `Handler`, the existing callback is replaced with the new one.
    pub fn with_callback(
        self,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> Self {
        match self {
            MsgEntry::Base(base) => MsgHandler::new(base, callback).into(),
            MsgEntry::Handler(mut handler) => {
                handler.set_callback(callback);
                MsgEntry::Handler(handler)
            }
        }
    }

    /// Convert a `Handler` entry to a `Base` by discarding the callback.
    ///
    /// Returns `self` unchanged if already a `Base`.
    pub fn into_base(self) -> Self {
        match self {
            MsgEntry::Base(_) => self,
            MsgEntry::Handler(handler) => handler.msg().clone().into(),
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
    // MsgBase::parse
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_all_arg_types() {
        let fmt = "test_cmd a=%u b=%i c=%hu d=%hi e=%s f=%c g=%*s h=%.*s";
        let (name, cmd) = MsgBase::parse(fmt).unwrap();
        assert_eq!(name, "test_cmd");
        assert_eq!(cmd.params.len(), 8);
        assert_eq!(cmd.params[0].1, ArgType::UInt32);
        assert_eq!(cmd.params[1].1, ArgType::Int32);
        assert_eq!(cmd.params[2].1, ArgType::UInt16);
        assert_eq!(cmd.params[3].1, ArgType::Int16);
        assert_eq!(cmd.params[4].1, ArgType::Str);
        assert_eq!(cmd.params[5].1, ArgType::Bytes);
        assert_eq!(cmd.params[6].1, ArgType::Str);
        assert_eq!(cmd.params[7].1, ArgType::Str);
    }

    #[test]
    fn test_parse_error_empty_string() {
        let result = MsgBase::parse("");
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().msg, "empty format string");
    }

    #[test]
    fn test_parse_error_invalid_param() {
        let result = MsgBase::parse("CMD badparam");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("invalid parameter format"));
    }

    #[test]
    fn test_parse_error_unknown_type() {
        let result = MsgBase::parse("CMD x=%x");
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
        let cmd = MsgBase::new(params);
        assert_eq!(cmd.format(), "a=%u b=%i c=%hu d=%hi e=%s f=%c");
    }

    // -----------------------------------------------------------------------
    // MsgHandler callback lifecycle
    // -----------------------------------------------------------------------

    #[test]
    fn test_set_callback() {
        let mut handler = MsgHandler::new(MsgBase::new(vec![]), |_| {});
        handler.set_callback(|_: &[ArgValue]| {});
    }

    #[test]
    fn test_invoke_callback_success() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;

        let invoked = Arc::new(AtomicBool::new(false));
        let invoked_clone = invoked.clone();
        let mut handler = MsgHandler::new(MsgBase::new(vec![]), move |_| {
            invoked_clone.store(true, Ordering::SeqCst);
        });
        let values = vec![ArgValue::Str("test".to_string())];
        handler.invoke_callback(&values);
        assert!(invoked.load(Ordering::SeqCst));
    }

    // -----------------------------------------------------------------------
    // MsgHandler Deref behavior
    // -----------------------------------------------------------------------

    #[test]
    fn test_deref_access_params() {
        let (name, command) = MsgBase::parse("G1 X=%u Y=%u").unwrap();
        let handler = MsgHandler::new(command, |_: &[ArgValue]| {});
        assert_eq!(name, "G1");
        // Access params() via Deref<Target=MsgBase>
        assert_eq!(handler.params().len(), 2);
    }

    #[test]
    fn test_deref_access_format() {
        let (name, command) = MsgBase::parse("G1 X=%u Y=%u").unwrap();
        let handler = MsgHandler::new(command, |_: &[ArgValue]| {});
        assert_eq!(name, "G1");
        // Access format() via Deref<Target=MsgBase>
        assert_eq!(handler.format(), "X=%u Y=%u");
    }

    // -----------------------------------------------------------------------
    // MsgHandler Debug
    // -----------------------------------------------------------------------

    #[test]
    fn test_handler_debug() {
        let handler = MsgHandler::new(MsgBase::new(vec![]), |_: &[ArgValue]| {});
        let debug_str = format!("{:?}", handler);
        assert!(debug_str.contains("MsgHandler"));
        assert!(!debug_str.contains("has_callback"));
    }

    // -----------------------------------------------------------------------
    // MsgBase Hash (for use in collections)
    // -----------------------------------------------------------------------

    #[test]
    fn test_command_base_hash() {
        use std::collections::HashSet;

        let cmd1 = MsgBase::new(vec![("x".to_string(), ArgType::Int32)]);
        let cmd2 = MsgBase::new(vec![("x".to_string(), ArgType::Int32)]);

        let mut set = HashSet::new();
        set.insert(cmd1);
        assert!(set.contains(&cmd2));
    }
}
