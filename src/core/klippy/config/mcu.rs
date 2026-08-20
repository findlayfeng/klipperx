use super::section::ConfigSection;
use crate::core::klippy::mcu::McuRestartMethod;
use crate::core::klippy::traits::KlippyInterface;

/// MCU (Microcontroller Unit) configuration parsed from config file.
pub struct McuConfig {
    /// MCU name (from ConfigSection's sub field)
    pub name: String,
    /// MCU restart method
    pub restart_method: McuRestartMethod,
    /// MCU interface for communication
    pub interface: Box<dyn KlippyInterface>,
}

impl McuConfig {
    /// Parse MCU configuration from a section.
    ///
    /// The name is taken from the section's sub field (e.g., "mcu zboard" → "zboard").
    /// The restart_method is parsed from the section's restart_method parameter.
    /// If a 'test' config exists, a TestInterface is created automatically.
    pub fn from_section(section: &ConfigSection) -> Result<Self, String> {
        // Get name from sub field
        let name = section.sub.clone().unwrap_or_default();

        // Parse restart_method
        let restart_method = section
            .get_str("restart_method")
            .and_then(McuRestartMethod::from_str)
            .unwrap_or(McuRestartMethod::Arduino);

        // Create interface
        let interface = Self::create_interface(section)?;

        Ok(Self {
            name,
            restart_method,
            interface,
        })
    }

    /// Create interface based on config.
    fn create_interface(section: &ConfigSection) -> Result<Box<dyn KlippyInterface>, String> {
        // Test mode: create TestInterface if test config exists
        #[cfg(test)]
        if let Some(test_value) = section.get("test") {
            let mappings = Self::parse_test_config(test_value)?;
            return Ok(Box::new(
                crate::core::klippy::interface::test::TestInterface::new(mappings),
            ));
        }

        // Non-test mode: normal interface creation (to be implemented)
        #[cfg(not(test))]
        {
            let _ = section;
        }

        Err("interface not created".to_string())
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
        assert!(result.is_err());
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
    }
}
