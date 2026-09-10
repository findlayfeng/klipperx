mod identify;
mod restart_method;

pub use identify::{Identify, IdentifyError, IdentifyErrorKind};
pub use restart_method::McuRestartMethod;

#[cfg(test)]
use crate::core::klippy::config::mcu::{McuConfig, McuInterface};
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
    pub parser: Parser<crate::core::klippy::interface::test::TestInterface>,
}

#[cfg(test)]
impl Mcu {
    /// Create a new `Mcu` object from an `McuConfig`.
    ///
    /// The interface is already stored in the `McuConfig`. A `Parser` is created
    /// from the interface, and the default identify messages are registered.
    pub fn from_config(config: McuConfig) -> Result<Self, String> {
        let interface = match config.interface {
            McuInterface::Test(interface) => interface,
        };

        let mut parser = Parser::new(interface);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mcu_from_config() {
        let config_str = r#"
[mcu zboard]

test:
    01 02
"#;
        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu zboard").unwrap();
        let mcu_config = McuConfig::from_section(mcu_section).unwrap();

        let mcu = Mcu::from_config(mcu_config).unwrap();
        assert_eq!(mcu.name, "zboard");
    }

    #[test]
    fn test_mcu_from_config_empty_name() {
        let config_str = r#"
[mcu]
restart_method: command

test:
    01 02
"#;
        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu").unwrap();
        let mcu_config = McuConfig::from_section(mcu_section).unwrap();

        let mcu = Mcu::from_config(mcu_config).unwrap();
        assert_eq!(mcu.name, "");
    }
}
