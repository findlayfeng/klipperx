use crate::core::klippy::mcu::McuRestartMethod;
use crate::core::klippy::interface::Interface;
use super::section::ConfigSection;

/// MCU (Microcontroller Unit) configuration parsed from config file.
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
    /// - If `test` config block exists and building for tests → creates `TestInterface`
    /// - Otherwise → returns an error (no interface configured)
    ///
    /// # Arguments
    /// * `section` — The MCU configuration section from the config file.
    ///
    /// # Returns
    /// `Ok(McuConfig)` with the appropriate interface type,
    /// or `Err` if no supported interface configuration is found.
    pub fn from_section(section: &ConfigSection) -> Result<Self, String> {
        // Parse common fields
        let (name, restart_method) = Self::parse_common(section);

        // Determine interface type from section content
        let interface = Self::create_interface(section)?;

        Ok(Self {
            name,
            restart_method,
            interface,
        })
    }

    /// Parse common MCU configuration fields.
    fn parse_common(section: &ConfigSection) -> (String, McuRestartMethod) {
        // Get name from sub field
        let name = section.sub.clone().unwrap_or_default();

        // Parse restart_method
        let restart_method = section
            .get_str("restart_method")
            .and_then(McuRestartMethod::from_str)
            .unwrap_or(McuRestartMethod::Arduino);

        (name, restart_method)
    }

    /// Create the appropriate interface based on section content.
    #[allow(dead_code, unused_variables)]
    fn create_interface(section: &ConfigSection) -> Result<Interface, String> {
        // Check for test configuration (only available in test builds)
        #[cfg(test)]
        if section.get("test").is_some() {
            let test_value = section.get("test").unwrap();
            let mappings = Self::parse_test_config(test_value)?;
            return Ok(Interface::test_new(mappings));
        }

        Err("no supported interface configuration found (test, serial, canbus, ...)".to_string())
    }

    /// Parse test config into mapping entries.
    #[cfg(test)]
    fn parse_test_config(
        test_value: &super::value::ConfigValue,
    ) -> Result<Vec<crate::core::klippy::interface::test::MappingEntry>, String> {
        let lines: Vec<String> = test_value
            .lines()
            .iter()
            .map(|s| s.to_string())
            .collect();

        let mut mappings = Vec::new();

        for line in lines {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }

            // Parse hex bytes: first is input, rest are outputs
            let hex_bytes: Vec<&str> = trimmed.split_whitespace().collect();
            if hex_bytes.is_empty() {
                continue;
            }

            let input = hex_decode(hex_bytes[0]).unwrap_or_default();
            let output_bytes: Vec<Vec<u8>> = hex_bytes[1..]
                .iter()
                .map(|h| hex_decode(h).unwrap_or_default())
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

        Ok(mappings)
    }
}

/// Decode a hex string to bytes.
#[cfg(test)]
fn hex_decode(s: &str) -> Result<Vec<u8>, String> {
    if s.len() % 2 != 0 {
        return Err("Hex string length must be even".to_string());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::InterfaceDevice;

    #[test]
    fn test_parse_mcu_config_no_test() {
        let config_str = r#"
[mcu]
serial: /dev/ttyACM0
baud: 250000
"#;

        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu").unwrap();
        let result = McuConfig::from_section(mcu_section);
        assert!(result.is_err(), "expected error when no 'test' config exists");
    }

    #[test]
    fn test_parse_mcu_config_with_test() {
        let config_str = r#"
[mcu]
serial: /dev/ttyACM0
baud: 250000

test:
    01 02 03
"#;

        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu").unwrap();
        let mcu_config = McuConfig::from_section(mcu_section).unwrap();
        assert_eq!(mcu_config.name, "");
        assert_eq!(mcu_config.restart_method, McuRestartMethod::Arduino);
        assert!(matches!(*mcu_config.interface.device.lock().unwrap(), InterfaceDevice::Test(_)));
    }

    #[test]
    fn test_parse_mcu_config_single_line_test() {
        let config_str = r#"
[mcu]
serial: /dev/ttyACM0
baud: 250000

test: 01 02 03
"#;

        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu").unwrap();
        let mcu_config = McuConfig::from_section(mcu_section).unwrap();
        assert_eq!(mcu_config.name, "");
        assert_eq!(mcu_config.restart_method, McuRestartMethod::Arduino);
        assert!(matches!(*mcu_config.interface.device.lock().unwrap(), InterfaceDevice::Test(_)));
    }

    #[test]
    fn test_parse_mcu_config_with_sub() {
        let config_str = r#"
[mcu zboard]
serial: /dev/ttyACM1
baud: 115200

test:
    aa bb cc
"#;

        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu zboard").unwrap();
        let mcu_config = McuConfig::from_section(mcu_section).unwrap();
        assert_eq!(mcu_config.name, "zboard");
        assert!(matches!(*mcu_config.interface.device.lock().unwrap(), InterfaceDevice::Test(_)));
    }

    #[test]
    fn test_parse_mcu_config_restart_method() {
        let config_str = r#"
[mcu]
serial: /dev/ttyACM0
restart_method: command

test:
    01 02
"#;

        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu").unwrap();
        let mcu_config = McuConfig::from_section(mcu_section).unwrap();
        assert_eq!(mcu_config.restart_method, McuRestartMethod::Command);
        assert!(matches!(*mcu_config.interface.device.lock().unwrap(), InterfaceDevice::Test(_)));
    }

    #[test]
    fn test_parse_mcu_config_default_restart_method() {
        let config_str = r#"
[mcu]
serial: /dev/ttyACM0

test:
    aa bb
"#;

        let (config, _) = crate::core::klippy::config::Config::from_str(config_str).unwrap();
        let mcu_section = config.get_section("mcu").unwrap();
        let mcu_config = McuConfig::from_section(mcu_section).unwrap();
        assert_eq!(mcu_config.restart_method, McuRestartMethod::Arduino);
        assert!(matches!(*mcu_config.interface.device.lock().unwrap(), InterfaceDevice::Test(_)));
    }
}
