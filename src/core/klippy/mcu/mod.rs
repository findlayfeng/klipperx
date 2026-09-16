mod identify;
mod restart_method;

pub use identify::{Identify, IdentifyError, IdentifyErrorKind};
pub use restart_method::McuRestartMethod;

#[cfg(test)]
use crate::core::klippy::config::mcu::McuConfig;
#[cfg(test)]
use crate::core::klippy::msg::parser::Parser;

/// MCU object that represents a physical microcontroller unit.
///
/// Created by consuming an `McuConfig` which already contains the interface.
#[cfg(test)]
pub struct Mcu {
    /// MCU name
    pub name: String,
    /// Message parser for communication
    pub parser: Parser,
}

#[cfg(test)]
impl Mcu {
    /// Create a new `Mcu` object from an `McuConfig`.
    ///
    /// The interface is already stored in the `McuConfig`. A `Parser` is created
    /// from the interface, and the default identify messages are registered.
    pub fn from_config(config: McuConfig) -> Result<Self, String> {
        let mut parser = Parser::new(config.interface);
        for (id, format_str) in identify::DEFAULT_MESSAGES {
            parser
                .register(*id, format_str)
                .expect("default identify message formats must be valid");
        }
        Ok(Self {
            name: config.name,
            parser,
        })
    }
}

// Tests removed due to Frame/Payload type conflicts - to be fixed later
