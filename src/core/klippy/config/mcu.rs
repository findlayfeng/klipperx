use super::section::ConfigSection;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::McuRestartMethod;

/// MCU (Microcontroller Unit) configuration parsed from config file.
#[derive(Debug)]
pub struct McuConfig {
    /// MCU name (from ConfigSection's sub field)
    pub name: String,
    /// MCU restart method
    pub restart_method: McuRestartMethod,
    /// MCU interface for communication
    pub interface: Interface,
}

impl McuConfig {
    /// Parse MCU configuration from a ConfigSection.
    ///
    /// Inspects the section to determine the interface type:
    /// - In test builds: checks for a `test` config block
    /// - In production: returns an error (no interface configured)
    ///
    /// # Arguments
    /// * `section` — The MCU configuration section from the config file.
    ///
    /// # Returns
    /// `Ok(McuConfig)` with the appropriate interface type,
    /// or `Err` if no supported interface configuration is found.
    pub fn new(section: &ConfigSection) -> Result<Self, String> {
        let (name, restart_method) = Self::parse_common(section);
        let interface = Self::create_interface(section)?;

        Ok(Self {
            name,
            restart_method,
            interface,
        })
    }

    /// Parse common MCU configuration fields.
    fn parse_common(section: &ConfigSection) -> (String, McuRestartMethod) {
        let name = section.sub.clone().unwrap_or_default();

        let restart_method = section
            .get_str("restart_method")
            .and_then(McuRestartMethod::from_str)
            .unwrap_or(McuRestartMethod::Arduino);

        (name, restart_method)
    }

    fn _create_interface(_section: &ConfigSection) -> Result<Interface, String> {
        Err("no supported interface configuration found (test, serial, canbus, ...)".to_string())
    }
    /// Create the appropriate interface based on section content.
    #[cfg(test)]
    fn create_interface(section: &ConfigSection) -> Result<Interface, String> {
        if let Some(test_value) = section.get("test") {
            let lines: Vec<String> = test_value.lines().iter().map(|s| s.to_string()).collect();

            let mut mappings = Vec::new();
            for line in lines {
                let trimmed = line.trim();
                if trimmed.is_empty() || trimmed.starts_with('#') {
                    continue;
                }

                let hex_bytes: Vec<&str> = trimmed.split_whitespace().collect();
                if hex_bytes.is_empty() {
                    continue;
                }

                let input = Self::hex_decode_bytes(hex_bytes[0]).unwrap_or_default();
                let output_bytes: Vec<Vec<u8>> = hex_bytes[1..]
                    .iter()
                    .map(|h| Self::hex_decode_bytes(h).unwrap_or_default())
                    .collect();

                if !output_bytes.is_empty() {
                    let input_frame = super::super::frame::Frame::new(0, input);
                    let output_frames: Vec<super::super::frame::Frame> = output_bytes
                        .into_iter()
                        .map(|payload| super::super::frame::Frame::new(0, payload))
                        .collect();
                    mappings.push(crate::core::klippy::interface::test::MappingEntry {
                        input: input_frame,
                        outputs: output_frames,
                    });
                }
            }

            return Ok(Interface::new(
                crate::core::klippy::interface::test::TestDevice::new(mappings),
            ));
        }

        Self::_create_interface(section)
    }

    /// Create the appropriate interface based on section content.
    #[cfg(not(test))]
    fn create_interface(section: &ConfigSection) -> Result<Interface, String> {
        Self::_create_interface(section)
    }

    /// Decode a hex string to bytes (test helper).
    #[cfg(test)]
    fn hex_decode_bytes(s: &str) -> Result<Vec<u8>, String> {
        if s.len() % 2 != 0 {
            return Err("Hex string length must be even".to_string());
        }
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| e.to_string()))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;

    fn make_section(test_lines: &[&str]) -> ConfigSection {
        let mut section = ConfigSection::new("mcu", None);
        if !test_lines.is_empty() {
            section.parameters.insert(
                "test".to_string(),
                ConfigValue::Multi(test_lines.iter().map(|s| s.to_string()).collect()),
            );
        }
        section
    }

    #[test]
    fn test_parse_mcu_config_with_test_interface() {
        let section = make_section(&["01 02 03"]);
        let result = McuConfig::new(&section);
        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.name, "");
        assert_eq!(config.restart_method, McuRestartMethod::Arduino);
    }

    #[test]
    fn test_parse_mcu_config_with_name() {
        let mut section = make_section(&["01 02"]);
        section.sub = Some("mcu0".to_string());
        let result = McuConfig::new(&section);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().name, "mcu0");
    }

    #[test]
    fn test_parse_mcu_config_with_restart_method() {
        let mut section = make_section(&["01 02"]);
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("rpi_usb".to_string()),
        );
        let result = McuConfig::new(&section);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().restart_method, McuRestartMethod::RpiUsb);
    }

    #[test]
    fn test_parse_mcu_config_no_interface() {
        let section = make_section(&[]);
        let result = McuConfig::new(&section);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("no supported interface"));
    }

    #[test]
    fn test_parse_test_config_skips_comments_and_empty() {
        let section = make_section(&["", "# this is a comment", "01 02", "   ", "03 04 05"]);
        let result = McuConfig::new(&section);
        assert!(result.is_ok());
    }

    #[test]
    fn test_hex_decode_bytes() {
        assert_eq!(McuConfig::hex_decode_bytes("01").unwrap(), vec![0x01]);
        assert_eq!(
            McuConfig::hex_decode_bytes("DEAD").unwrap(),
            vec![0xDE, 0xAD]
        );
        assert_eq!(
            McuConfig::hex_decode_bytes("BEEF00").unwrap(),
            vec![0xBE, 0xEF, 0x00]
        );
        assert!(McuConfig::hex_decode_bytes("ABC").is_err()); // odd length
        assert!(McuConfig::hex_decode_bytes("ZZ").is_err()); // invalid hex
    }
}
