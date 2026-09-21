// Klipper error vocabulary
//
// The errors a host reports fall into four classes, and which one an error is
// decides what happens to the machine:
//
// | class | who caused it | what happens |
// |---|---|---|
// | [`CommandError`](crate::core::klippy::gcode::CommandError) | the client's G-code | the line is refused (`!! …`); the printer keeps running |
// | [`ConfigError`] | the configuration file | the printer reports an `error` state and can be restarted |
// | [`KlippyError`] communication variants | the MCU or the link | the printer reports an `error` state (an MCU may be analysed first) |
// | [`KlippyError::Internal`] | klippy itself | the printer is shut down, because its state can no longer be trusted |
//
// `ConfigError` is separate from the rest because it is raised while reading
// the config file and by module constructors, before there is any device to
// talk to.

/// A configuration file problem.
///
/// Upstream spells this `configfile.error` (`configparser.Error`) and raises it
/// from every `ConfigWrapper` getter, from `Printer.add_object` and from the
/// undefined-option check. Its message is what the user sees, so the wording is
/// kept verbatim ("Option 'x' in section 'y' must be specified", …).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError {
    message: String,
}

impl ConfigError {
    /// A config error with the message the user will see.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// The message.
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ConfigError {}

impl From<String> for ConfigError {
    fn from(message: String) -> Self {
        Self { message }
    }
}

/// Klipper device communication errors
#[derive(Debug)]
pub enum KlippyError {
    /// Connection error
    Connection(String),
    /// Protocol error
    Protocol(String),
    /// Request error
    Request(String),
    /// Response parsing error
    Parse(String),
    /// A configuration problem found after the config was loaded.
    ///
    /// Most config errors are raised by the loader ([`ConfigError`]); this
    /// variant carries the ones a part finds while it connects (an MCU section
    /// re-read at connect time, a resource whose pins cannot be reserved).
    Config(ConfigError),
    /// Internal error
    Internal(String),
}

impl std::fmt::Display for KlippyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KlippyError::Connection(msg) => write!(f, "Connection: {}", msg),
            KlippyError::Protocol(msg) => write!(f, "Protocol: {}", msg),
            KlippyError::Request(msg) => write!(f, "Request: {}", msg),
            KlippyError::Parse(msg) => write!(f, "Parse: {}", msg),
            KlippyError::Config(err) => write!(f, "{}", err),
            KlippyError::Internal(msg) => write!(f, "Internal: {}", msg),
        }
    }
}

impl std::error::Error for KlippyError {}

impl From<serde_json::Error> for KlippyError {
    fn from(e: serde_json::Error) -> Self {
        KlippyError::Parse(e.to_string())
    }
}

impl From<ConfigError> for KlippyError {
    fn from(e: ConfigError) -> Self {
        KlippyError::Config(e)
    }
}
