use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::msg::parser::Parser;

/// MCU object that represents a physical microcontroller unit.
///
/// Created by consuming an `McuConfig` which already contains the interface.
pub struct Mcu {
    /// MCU name
    pub name: String,
    /// Message parser for communication
    pub parser: Parser,
}

impl Mcu {
    /// Create a new `Mcu` object from an `McuConfig`.
    ///
    /// The interface is already stored in the `McuConfig`. A `Parser` is created
    /// from the interface and stored in the struct.
    pub fn from_config(config: McuConfig) -> Self {
        Self {
            name: config.name,
            parser: Parser::new(config.interface.into()),
        }
    }
}
