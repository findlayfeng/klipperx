use super::section::ConfigSection;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::McuRestartMethod;

/// MCU (Microcontroller Unit) configuration parsed from config file.
#[derive(Debug)]
pub struct McuConfig {
    /// MCU name (from ConfigSection's sub field)
    pub name: String,
    /// MCU restart method.
    ///
    /// Parsed and stored, but **not consumed yet** — restarting the firmware is a
    /// planned feature (see [`McuRestartMethod`]). Kept so a config that sets
    /// `restart_method` parses the same way it will once the restart path exists.
    pub restart_method: McuRestartMethod,
    /// MCU interface for communication
    pub interface: Interface,
}

/// Parse Klipper's `canbus_uuid`: six bytes as twelve hex digits.
fn parse_canbus_uuid(text: &str) -> Result<[u8; 6], String> {
    let text = text.trim();
    if text.len() != 12 {
        return Err(format!("expected 12 hex digits, got {}", text.len()));
    }
    let mut uuid = [0u8; 6];
    for (i, byte) in uuid.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[i * 2..i * 2 + 2], 16)
            .map_err(|_| format!("'{text}' is not hexadecimal"))?;
    }
    Ok(uuid)
}

/// The connection keys a `[mcu]` section may carry, in the order errors list
/// them. Exactly one of them names the transport, the way Klipper's own `[mcu]`
/// works (`serial` or `canbus_uuid`): see [`McuConfig::create_interface`].
///
/// `test` exists in test builds only — it is how the unit tests script a device.
fn interface_keys() -> &'static [&'static str] {
    #[cfg(test)]
    return &["host_library", "serial", "canbus_uuid", "test"];
    #[cfg(not(test))]
    return &["host_library", "serial", "canbus_uuid"];
}

/// Klipper's node-id range: ids are mapped to `0x100 + 2 * nodeid`, and the MCU
/// answers on the next arbitration id, so the whole 11-bit id space has to fit.
const MAX_CANBUS_NODEID: u32 = 0x37f;

impl McuConfig {
    /// Parse MCU configuration from a ConfigSection.
    ///
    /// The name comes from the section's sub (`[mcu zboard]` → `zboard`), and the
    /// interface from one of the connection keys (see `interface_keys`):
    ///
    /// ```ini
    /// [mcu]
    /// serial: /dev/ttyACM0
    /// baud: 250000
    ///
    /// [mcu simulated]
    /// host_library: /path/to/libklipper_host.so
    /// ```
    ///
    /// A CAN-connected MCU names its node id instead of a device path, and the
    /// CAN interface it is on (Klipper's default is `can0`):
    ///
    /// ```ini
    /// [mcu]
    /// canbus_uuid: 11aa22bb33cc
    /// canbus_interface: can0
    /// canbus_nodeid: 2
    /// ```
    ///
    /// `baud` only applies to `serial`, and defaults to Klipper's 250000.
    ///
    /// # Arguments
    /// * `section` — The MCU configuration section from the config file.
    ///
    /// # Returns
    /// `Ok(McuConfig)` with the appropriate interface type, or `Err` when the
    /// section names no interface, names several, or names one that cannot be
    /// brought up.
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
    ///
    /// `restart_method` defaults to [`McuRestartMethod::Arduino`] (Klipper's
    /// default). The value is deliberately parsed even though nothing reads it
    /// yet: see the field documentation on [`McuConfig::restart_method`].
    fn parse_common(section: &ConfigSection) -> (String, McuRestartMethod) {
        // Upstream names an MCU by its config section with the `mcu ` prefix
        // stripped (`klippy/mcu.py:1151-1153`): the main `[mcu]` is "mcu", and
        // `[mcu zboard]` is "zboard". Not an empty string for the main one.
        let name = section.sub.clone().unwrap_or_else(|| section.id.clone());

        let restart_method = section
            .get_str("restart_method")
            .and_then(McuRestartMethod::from_str)
            .unwrap_or(McuRestartMethod::Arduino);

        (name, restart_method)
    }

    /// Create the interface the section asks for.
    ///
    /// One connection key selects it. Two of them is a configuration mistake, not
    /// a preference order, so it is reported rather than resolved silently.
    fn create_interface(section: &ConfigSection) -> Result<Interface, String> {
        let requested: Vec<&str> = interface_keys()
            .iter()
            .copied()
            .filter(|key| section.has(key))
            .collect();
        if requested.len() > 1 {
            return Err(format!(
                "MCU '{}' sets more than one interface: {}",
                section.identifier(),
                requested.join(", ")
            ));
        }

        if let Some(text) = section.get_str("canbus_uuid") {
            let uuid = parse_canbus_uuid(text).map_err(|e| {
                format!(
                    "MCU '{}' has an invalid canbus_uuid: {e}",
                    section.identifier()
                )
            })?;
            let interface = section
                .get_str("canbus_interface")
                .unwrap_or("can0")
                .to_string();
            // Klipper hands out node ids from its `[canbus_ids]` section; klipperx
            // has no such allocator yet, so the section states the id itself.
            let nodeid = match section.get_str("canbus_nodeid") {
                Some(text) => match text.parse::<u32>() {
                    Ok(nodeid) if (1..=MAX_CANBUS_NODEID).contains(&nodeid) => nodeid,
                    _ => {
                        return Err(format!(
                            "MCU '{}' has an invalid canbus_nodeid: '{text}' \
                             (expected 1..={MAX_CANBUS_NODEID})",
                            section.identifier()
                        ))
                    }
                },
                None => {
                    return Err(format!(
                        "MCU '{}' is on a CAN bus, so it needs a canbus_nodeid \
                         (klipperx does not allocate one yet)",
                        section.identifier()
                    ))
                }
            };
            return Interface::canserial(&interface, uuid, nodeid)
                .map_err(|e| format!("canbus: {e}"));
        }

        if section.has("canbus_nodeid") || section.has("canbus_interface") {
            return Err(format!(
                "MCU '{}' needs a canbus_uuid to go with its CAN settings",
                section.identifier()
            ));
        }

        if let Some(path) = section.get_str("host_library") {
            return Interface::host(path).map_err(|e| format!("host_library: {e}"));
        }

        if let Some(path) = section.get_str("serial") {
            let baud = match section.get_str("baud") {
                // A rate of zero would ask the kernel to hang the line up.
                Some(text) => match text.parse::<u32>() {
                    Ok(baud) if baud > 0 => baud,
                    _ => {
                        return Err(format!(
                            "MCU '{}' has an invalid baud: '{text}'",
                            section.identifier()
                        ))
                    }
                },
                None => crate::core::klippy::interface::serial::DEFAULT_BAUD,
            };
            return Interface::serial(path, baud).map_err(|e| format!("serial: {e}"));
        }

        #[cfg(test)]
        if let Some(test_value) = section.get("test") {
            return Ok(Interface::new(Self::test_device(test_value)));
        }

        let how = if cfg!(test) {
            "set host_library: <libklipper_host.so> (or test: <frame mappings>)"
        } else {
            "set host_library: <libklipper_host.so>"
        };
        Err(format!(
            "MCU '{}' needs an interface: {how}",
            section.identifier()
        ))
    }

    /// Build a scripted device from a `test:` block of hex frame mappings
    /// (test builds only).
    #[cfg(test)]
    fn test_device(
        test_value: &super::value::ConfigValue,
    ) -> crate::core::klippy::interface::test::TestDevice {
        let mut mappings = Vec::new();
        for line in test_value.lines() {
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

        crate::core::klippy::interface::test::TestDevice::new(mappings)
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

    /// A section with one `key: value` parameter.
    fn section_with(key: &str, value: &str) -> ConfigSection {
        let mut section = ConfigSection::new("mcu", None);
        section
            .parameters
            .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        section
    }

    #[test]
    fn test_parse_mcu_config_with_test_interface() {
        let section = make_section(&["01 02 03"]);
        let result = McuConfig::new(&section);
        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.name, "mcu");
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
        let err = McuConfig::new(&section).unwrap_err();
        // The error has to say how to fix the section, not just that it is wrong.
        assert!(err.contains("needs an interface"), "{err}");
        assert!(err.contains("host_library"), "{err}");
    }

    #[test]
    fn test_host_library_key_becomes_the_host_interface() {
        // A path that cannot be loaded still proves the routing: the error is the
        // library's, not the "needs an interface" one.
        let section = section_with("host_library", "/nonexistent/libklipper_host.so");
        let err = McuConfig::new(&section).unwrap_err();
        assert!(err.starts_with("host_library: "), "{err}");
        assert!(err.contains("/nonexistent/libklipper_host.so"), "{err}");
    }

    #[test]
    fn test_two_interface_keys_are_rejected() {
        let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/dev/ttyACM0".to_string()),
        );

        let err = McuConfig::new(&section).unwrap_err();
        assert!(err.contains("more than one interface"), "{err}");
        assert!(err.contains("host_library, serial"), "{err}");
    }

    #[test]
    fn test_serial_key_becomes_the_serial_interface() {
        // A port that cannot be opened still proves the routing: the error names
        // the port, which only the serial device's own error does.
        let section = section_with("serial", "/dev/not-a-serial-port");
        let err = McuConfig::new(&section).unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");
    }

    /// A CAN section, with `overrides` on top of a complete one.
    fn can_section(overrides: &[(&str, &str)]) -> ConfigSection {
        let mut section = section_with("canbus_uuid", "11aa22bb33cc");
        section.parameters.insert(
            "canbus_nodeid".to_string(),
            ConfigValue::Single("2".to_string()),
        );
        for (key, value) in overrides {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        section
    }

    #[test]
    fn test_canbus_keys_become_the_can_interface() {
        // No CAN interface in the test environment, so the routing shows up as the
        // socket's error naming the interface we asked for.
        let section = can_section(&[("canbus_interface", "can99")]);
        let err = McuConfig::new(&section).unwrap_err();
        assert!(err.starts_with("canbus: "), "{err}");
        assert!(err.contains("can99"), "{err}");
    }

    #[test]
    fn test_canbus_uuid_is_parsed_as_klipper_writes_it() {
        assert_eq!(
            parse_canbus_uuid("11aa22bb33cc").unwrap(),
            [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        );
        for bad in ["11aa22bb33c", "11aa22bb33ccdd", "11aa22bb33cg", ""] {
            let section = can_section(&[("canbus_uuid", bad)]);
            let err = McuConfig::new(&section).unwrap_err();
            assert!(err.contains("invalid canbus_uuid"), "{bad}: {err}");
        }
    }

    #[test]
    fn test_canbus_nodeid_is_validated() {
        for bad in ["0", "fast", "900"] {
            let section = can_section(&[("canbus_nodeid", bad)]);
            let err = McuConfig::new(&section).unwrap_err();
            assert!(err.contains("invalid canbus_nodeid"), "{bad}: {err}");
        }

        // A valid node id gets as far as the socket, which is where it fails here.
        let err = McuConfig::new(&can_section(&[])).unwrap_err();
        assert!(err.contains("no CAN interface named 'can0'"), "{err}");
    }

    #[test]
    fn test_canbus_settings_without_a_uuid_are_reported() {
        let section = section_with("canbus_nodeid", "2");
        let err = McuConfig::new(&section).unwrap_err();
        assert!(err.contains("needs a canbus_uuid"), "{err}");

        let section = section_with("canbus_interface", "can0");
        let err = McuConfig::new(&section).unwrap_err();
        assert!(err.contains("needs a canbus_uuid"), "{err}");
    }

    #[test]
    fn test_invalid_baud_is_rejected_before_opening_the_port() {
        let mut section = section_with("serial", "/dev/not-a-serial-port");
        section
            .parameters
            .insert("baud".to_string(), ConfigValue::Single("fast".to_string()));
        let err = McuConfig::new(&section).unwrap_err();
        assert!(err.contains("invalid baud"), "{err}");
        assert!(err.contains("fast"), "{err}");
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
