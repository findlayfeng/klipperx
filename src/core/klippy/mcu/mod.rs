mod identify;
mod restart_method;

pub use identify::{Identify, IdentifyError, IdentifyErrorKind};
pub use restart_method::McuRestartMethod;

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
    /// from the interface, and the default identify messages are registered.
    pub fn from_config(config: McuConfig) -> Self {
        let mut parser = Parser::new(config.interface.into());
        for (id, format_str) in identify::DEFAULT_MESSAGES {
            parser
                .register(*id, format_str)
                .expect("default identify message formats must be valid");
        }
        Self {
            name: config.name,
            parser,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::test::TestInterface;

    #[test]
    fn test_mcu_from_config() {
        let interface = Box::new(TestInterface::new(vec![]));
        let config = McuConfig {
            name: "zboard".to_string(),
            restart_method: McuRestartMethod::Arduino,
            interface,
        };

        let mcu = Mcu::from_config(config);
        assert_eq!(mcu.name, "zboard");
        // Parser should have the default messages registered (identify_request/response)
    }

    #[test]
    fn test_mcu_from_config_empty_name() {
        let interface = Box::new(TestInterface::new(vec![]));
        let config = McuConfig {
            name: String::new(),
            restart_method: McuRestartMethod::Command,
            interface,
        };

        let mcu = Mcu::from_config(config);
        assert_eq!(mcu.name, "");
    }
}
