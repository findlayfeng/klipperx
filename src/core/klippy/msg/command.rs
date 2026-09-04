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

/// A `Command` with a callback for handling matched messages.
///
/// `CommandHandler` wraps a `Command` and provides callback functionality.
/// It implements `Deref<Target = Command>` so all `Command` methods are
/// directly accessible.
///
/// # Example
/// ```
/// # use klipperx::core::klippy::msg::{CommandBase, CommandHandler, ArgValue};
/// let (name, command) = CommandBase::parse("config_digital_out oid=%u pin=%s").unwrap();
/// let mut handler = CommandHandler::new(command, |values| {
///     println!("command received with {} params", values.len());
/// });
/// assert_eq!(name, "config_digital_out");
/// ```
pub struct CommandHandler {
    /// The underlying command definition.
    command: CommandBase,
    /// Callback invoked when this command is matched.
    callback: Box<dyn FnMut(&[ArgValue]) + Send>,
}

impl std::fmt::Debug for CommandHandler {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CommandHandler")
            .field("command", &self.command)
            .finish()
    }
}

impl CommandHandler {
    /// Create a new `CommandHandler` from a `Command` and a callback.
    ///
    /// The callback receives a slice of `ArgValue` containing all decoded
    /// parameter values in command definition order. Use
    /// `values[i].clone()` or pattern matching to extract values.
    pub fn new(
        command: CommandBase,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> Self {
        Self {
            command,
            callback: Box::new(callback),
        }
    }

    /// Returns a reference to the underlying `Command`.
    pub fn command(&self) -> &CommandBase {
        &self.command
    }

    /// Set or replace the callback function.
    ///
    /// The callback receives a slice of `ArgValue` containing all decoded
    /// parameter values in command definition order.
    pub fn set_callback(&mut self, callback: impl FnMut(&[ArgValue]) + Send + 'static) {
        self.callback = Box::new(callback);
    }

    /// Invoke the callback with the given parameters.
    pub fn invoke_callback(&mut self, params: &[ArgValue]) -> CommandResult<()> {
        (self.callback)(params);
        Ok(())
    }
}

impl std::ops::Deref for CommandHandler {
    type Target = CommandBase;

    fn deref(&self) -> &Self::Target {
        &self.command
    }
}

/// A command entry that wraps a parsed command format.
#[derive(Debug)]
pub enum CommandEntry {
    /// A base command format (no callback).
    Base(CommandBase),
    /// A command handler with a callback.
    Handler(CommandHandler),
}

impl From<CommandHandler> for CommandEntry {
    fn from(handler: CommandHandler) -> Self {
        Self::Handler(handler)
    }
}

impl From<CommandBase> for CommandEntry {
    fn from(base: CommandBase) -> Self {
        Self::Base(base)
    }
}

impl CommandEntry {
    /// Convert a `Base` entry to a `Handler` by providing a callback.
    ///
    /// The callback receives a slice of `ArgValue` containing all decoded
    /// parameter values in command definition order.
    ///
    /// Returns `self` unchanged if already a `Handler`.
    pub fn with_callback(
        self,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> Self {
        match self {
            CommandEntry::Base(base) => CommandHandler::new(base, callback).into(),
            CommandEntry::Handler(ref _handler) => self,
        }
    }

    /// Convert a `Handler` entry to a `Base` by discarding the callback.
    ///
    /// Returns `self` unchanged if already a `Base`.
    pub fn into_base(self) -> Self {
        match self {
            CommandEntry::Base(_) => self,
            CommandEntry::Handler(handler) => handler.command().clone().into(),
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
    fn test_set_callback() {
        let mut handler = CommandHandler::new(CommandBase::new(vec![]), |_| {});
        handler.set_callback(|_: &[ArgValue]| {});
    }

    #[test]
    fn test_invoke_callback_success() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;

        let invoked = Arc::new(AtomicBool::new(false));
        let invoked_clone = invoked.clone();
        let mut handler = CommandHandler::new(CommandBase::new(vec![]), move |_| {
            invoked_clone.store(true, Ordering::SeqCst);
        });
        let values = vec![ArgValue::Str("test".to_string())];
        assert!(handler.invoke_callback(&values).is_ok());
        assert!(invoked.load(Ordering::SeqCst));
    }

    // -----------------------------------------------------------------------
    // CommandHandler Deref behavior
    // -----------------------------------------------------------------------

    #[test]
    fn test_deref_access_params() {
        let (name, command) = CommandBase::parse("G1 X=%u Y=%u").unwrap();
        let handler = CommandHandler::new(command, |_: &[ArgValue]| {});
        assert_eq!(name, "G1");
        // Access params() via Deref<Target=CommandBase>
        assert_eq!(handler.params().len(), 2);
    }

    #[test]
    fn test_deref_access_format() {
        let (name, command) = CommandBase::parse("G1 X=%u Y=%u").unwrap();
        let handler = CommandHandler::new(command, |_: &[ArgValue]| {});
        assert_eq!(name, "G1");
        // Access format() via Deref<Target=CommandBase>
        assert_eq!(handler.format(), "X=%u Y=%u");
    }

    // -----------------------------------------------------------------------
    // CommandHandler Debug
    // -----------------------------------------------------------------------

    #[test]
    fn test_handler_debug() {
        let handler = CommandHandler::new(CommandBase::new(vec![]), |_: &[ArgValue]| {});
        let debug_str = format!("{:?}", handler);
        assert!(debug_str.contains("CommandHandler"));
        assert!(!debug_str.contains("has_callback"));
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
