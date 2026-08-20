// Command handling for Klipper message protocol.
//
// Provides the `Command` struct — a parsed command containing the name
// and parameter list. Encoding/decoding logic is in `proto.rs`.
// Also provides `CommandHandler` — a `Command` with an optional callback
// for handling matched messages.

use super::proto::{ArgType, ArgValue};

// ===========================================================================
// Function Call Parameters
// ===========================================================================

/// Function call parameters.
///
/// - `Positional(ArgValue)`: Positional parameters, must be passed in the
///   order defined by the command.
/// - `Named(String, ArgValue)`: Named parameters, can be passed in any order.
#[derive(Debug, Clone, PartialEq)]
pub enum Param {
    Positional(ArgValue),
    Named(String, ArgValue),
}

impl Param {
    /// Get the parameter value.
    pub fn value(&self) -> &ArgValue {
        match self {
            Param::Positional(v) => v,
            Param::Named(_, v) => v,
        }
    }

    /// Get the parameter name (if named).
    pub fn name(&self) -> Option<&str> {
        match self {
            Param::Positional(_) => None,
            Param::Named(name, _) => Some(name),
        }
    }
}

// ===========================================================================
// Self-defined CommandError / CommandResult
// ===========================================================================

/// Error raised by command operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommandError {
    pub msg: String,
}

impl CommandError {
    pub fn new(msg: impl Into<String>) -> Self {
        Self { msg: msg.into() }
    }
}

impl std::fmt::Display for CommandError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.msg)
    }
}

impl std::error::Error for CommandError {}

/// Result type used for command operations.
pub type CommandResult<T> = Result<T, CommandError>;

// ===========================================================================
// Command
// ===========================================================================

/// A parsed command format containing only the name and parameter list.
///
/// Encoding/decoding methods are provided as standalone functions in `proto.rs`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct CommandBase {
    /// Parameter list in declaration order: (parameter_name, parameter_type).
    pub params: Vec<(String, ArgType)>,
}

impl CommandBase {
    /// Parse a Klipper message format string into a `Command`.
    ///
    /// Format strings are space-separated tokens where the first token is the
    /// command name and remaining tokens are `name=type` pairs.
    ///
    /// Returns `CommandError` if the format string is empty or malformed.
    ///
    /// # Examples
    /// ```
    /// # use klipperx::core::klippy::msg::CommandBase;
    /// let (name, cmd) = CommandBase::parse("config_digital_out oid=%u pin=%s").unwrap();
    /// assert_eq!(name, "config_digital_out");
    /// assert_eq!(cmd.params().len(), 2);
    /// ```
    pub fn parse(fmt: &str) -> CommandResult<(String, Self)> {
        let trimmed = fmt.trim();
        if trimmed.is_empty() {
            return Err(CommandError::new("empty format string"));
        }
        let mut parts = trimmed.split_whitespace();
        let name = parts
            .next()
            .ok_or_else(|| CommandError::new("missing command name"))?
            .to_string();
        let mut params = Vec::new();
        for part in parts {
            let eq = part
                .find('=')
                .ok_or_else(|| CommandError::new(format!("invalid parameter format: {}", part)))?;
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
                    return Err(CommandError::new(format!(
                        "unknown type specifier: {}",
                        typ
                    )))
                }
            };
            params.push((param_name, arg_type));
        }
        Ok((name, Self { params }))
    }

    /// Create a `Command` from components.
    pub fn new(params: Vec<(String, ArgType)>) -> Self {
        Self { params }
    }

    /// Returns the parameter list.
    pub fn params(&self) -> &[(String, ArgType)] {
        &self.params
    }

    /// Returns the format string by reconstructing it from name and params.
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
// CommandHandler
// ===========================================================================

/// A `Command` with an optional callback for handling matched messages.
///
/// `CommandHandler` wraps a `Command` and provides callback functionality.
/// It implements `Deref<Target = Command>` so all `Command` methods are
/// directly accessible.
///
/// # Example
/// ```
/// # use klipperx::core::klippy::msg::{CommandHandler, ArgValue};
/// let (name, mut handler) = CommandHandler::parse("config_digital_out oid=%u pin=%s").unwrap();
/// assert_eq!(name, "config_digital_out");
/// handler.set_callback(|_params| {
///     println!("command received");
/// });
/// ```
pub struct CommandHandler {
    /// The underlying command definition.
    command: CommandBase,
    /// Optional callback invoked when this command is matched.
    callback: Option<Box<dyn FnMut(&ArgValue)>>,
}

impl std::fmt::Debug for CommandHandler {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CommandHandler")
            .field("command", &self.command)
            .field("has_callback", &self.callback.is_some())
            .finish()
    }
}

impl CommandHandler {
    /// Create a new `CommandHandler` from a `Command`.
    pub fn new(command: CommandBase) -> Self {
        Self {
            command,
            callback: None,
        }
    }

    /// Create a `CommandHandler` by parsing a format string.
    pub fn parse(fmt: &str) -> CommandResult<(String, Self)> {
        let (name, command) = CommandBase::parse(fmt)?;

        Ok((
            name,
            Self {
                command,
                callback: None,
            },
        ))
    }

    /// Create a `CommandHandler` with a callback.
    pub fn with_callback(command: CommandBase, callback: impl FnMut(&ArgValue) + 'static) -> Self {
        Self {
            command,
            callback: Some(Box::new(callback)),
        }
    }

    /// Returns a reference to the underlying `Command`.
    pub fn command(&self) -> &CommandBase {
        &self.command
    }

    /// Set or replace the callback function.
    ///
    /// The callback receives a reference to `ArgValue` which contains the
    /// decoded parameter values. Use `params.get_int()`, `params.get_str()`,
    /// `params.get_float()`, or `params.get_bytes()` to extract values.
    pub fn set_callback(&mut self, callback: impl FnMut(&ArgValue) + 'static) {
        self.callback = Some(Box::new(callback));
    }

    /// Remove the callback, if any.
    pub fn clear_callback(&mut self) {
        self.callback = None;
    }

    /// Check if this handler has a callback set.
    pub fn has_callback(&self) -> bool {
        self.callback.is_some()
    }

    /// Invoke the callback with the given parameters, if set.
    ///
    /// Returns `Ok(())` if the callback was invoked, or `Err(CommandError)`
    /// if no callback is set.
    pub fn invoke_callback(&mut self, params: &ArgValue) -> CommandResult<()> {
        if let Some(ref mut callback) = self.callback {
            callback(params);
            Ok(())
        } else {
            Err(CommandError::new("no callback set for command"))
        }
    }
}

impl From<CommandBase> for CommandHandler {
    fn from(command: CommandBase) -> Self {
        Self::new(command)
    }
}

impl std::ops::Deref for CommandHandler {
    type Target = CommandBase;

    fn deref(&self) -> &Self::Target {
        &self.command
    }
}

/// A command entry that can be either a regular message format or a
/// request handler with an optional callback.
#[derive(Debug)]
pub enum CommandEntry {
    /// A regular message format (no callback).
    Regular(CommandBase),
    /// A request handler with an optional callback.
    Request(CommandHandler),
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // CommandBase::parse
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_all_arg_types() {
        let fmt = "test_cmd a=%u b=%i c=%hu d=%hi e=%s f=%c g=%*s h=%.*s";
        let (name, cmd) = CommandBase::parse(fmt).unwrap();
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
        let result = CommandBase::parse("");
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().msg, "empty format string");
    }

    #[test]
    fn test_parse_error_invalid_param() {
        let result = CommandBase::parse("CMD badparam");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("invalid parameter format"));
    }

    #[test]
    fn test_parse_error_unknown_type() {
        let result = CommandBase::parse("CMD x=%x");
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
        let cmd = CommandBase::new(params);
        assert_eq!(cmd.format(), "a=%u b=%i c=%hu d=%hi e=%s f=%c");
    }

    // -----------------------------------------------------------------------
    // CommandHandler callback lifecycle
    // -----------------------------------------------------------------------

    #[test]
    fn test_set_and_clear_callback() {
        let mut handler = CommandHandler::new(CommandBase::new(vec![]));
        assert!(!handler.has_callback());
        handler.set_callback(|_: &ArgValue| {});
        assert!(handler.has_callback());
        handler.clear_callback();
        assert!(!handler.has_callback());
    }

    #[test]
    fn test_invoke_callback_success() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;

        let mut handler = CommandHandler::new(CommandBase::new(vec![]));
        let invoked = Arc::new(AtomicBool::new(false));
        let invoked_clone = invoked.clone();
        handler.set_callback(move |_| {
            invoked_clone.store(true, Ordering::SeqCst);
        });
        let value = ArgValue::Str("test".to_string());
        assert!(handler.invoke_callback(&value).is_ok());
        assert!(invoked.load(Ordering::SeqCst));
    }

    #[test]
    fn test_invoke_callback_no_handler() {
        let mut handler = CommandHandler::new(CommandBase::new(vec![]));
        let result = handler.invoke_callback(&ArgValue::Str("test".to_string()));
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().msg, "no callback set for command");
    }

    // -----------------------------------------------------------------------
    // CommandHandler Deref behavior
    // -----------------------------------------------------------------------

    #[test]
    fn test_deref_access_params() {
        let (name, handler) = CommandHandler::parse("G1 X=%u Y=%u").unwrap();
        assert_eq!(name, "G1");
        // Access params() via Deref<Target=CommandBase>
        assert_eq!(handler.params().len(), 2);
    }

    #[test]
    fn test_deref_access_format() {
        let (name, handler) = CommandHandler::parse("G1 X=%u Y=%u").unwrap();
        assert_eq!(name, "G1");
        // Access format() via Deref<Target=CommandBase>
        assert_eq!(handler.format(), "X=%u Y=%u");
    }

    // -----------------------------------------------------------------------
    // CommandHandler Debug
    // -----------------------------------------------------------------------

    #[test]
    fn test_handler_debug_no_callback() {
        let handler = CommandHandler::new(CommandBase::new(vec![]));
        let debug_str = format!("{:?}", handler);
        assert!(debug_str.contains("CommandHandler"));
        assert!(debug_str.contains("has_callback: false"));
    }

    #[test]
    fn test_handler_debug_with_callback() {
        let mut handler = CommandHandler::new(CommandBase::new(vec![]));
        handler.set_callback(|_: &ArgValue| {});
        let debug_str = format!("{:?}", handler);
        assert!(debug_str.contains("has_callback: true"));
    }

    // -----------------------------------------------------------------------
    // CommandBase Hash (for use in collections)
    // -----------------------------------------------------------------------

    #[test]
    fn test_command_base_hash() {
        use std::collections::HashSet;

        let cmd1 = CommandBase::new(vec![("x".to_string(), ArgType::Int32)]);
        let cmd2 = CommandBase::new(vec![("x".to_string(), ArgType::Int32)]);

        let mut set = HashSet::new();
        set.insert(cmd1);
        assert!(set.contains(&cmd2));
    }
}
