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
    /// - If `test` config block exists and building for tests → creates `Interface::Test`
    /// - Otherwise → returns an error (no interface configured)
    ///
    /// # Arguments
    /// * `section` — The MCU configuration section from the config file.
    ///
    /// # Returns
    /// `Ok(McuConfig)` with the appropriate interface type,
    /// or `Err` if no supported interface configuration is found.
    pub fn new(section: &ConfigSection) -> Result<Self, String> {
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
    fn create_interface(section: &ConfigSection) -> Result<Interface, String> {
        #[cfg(test)]
        if section.get("test").is_some() {
            let test_value = section.get("test").unwrap();
            let mappings = Self::parse_test_config(test_value)?;
            return Ok(Interface::new(
                crate::core::klippy::interface::test::TestDevice::new(mappings),
            ));
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
