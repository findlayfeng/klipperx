// ===========================================================================
// Function Call Parameters
// ===========================================================================

use super::proto::ArgValue;

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
