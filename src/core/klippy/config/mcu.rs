use super::wrapper::ConfigWrapper;
use crate::core::klippy::error::ConfigError;
use crate::core::klippy::interface::usb::UsbPowerMethod;
#[cfg(test)]
use crate::core::klippy::interface::SimulatorDevice;
use crate::core::klippy::interface::{Interface, SerialDevice};
use crate::core::klippy::mcu::McuRestartMethod;
use tracing::warn;

/// MCU (Microcontroller Unit) configuration parsed from config file.
///
/// Parsing does not touch a device: [`McuConfig::open`] is what opens the
/// transport. The split lets a firmware restart reset the board on its **closed**
/// port, between the two (`mcu/object.rs`, `mcu/restart.rs`).
#[derive(Debug)]
pub struct McuConfig {
    /// MCU name (from ConfigSection's sub field)
    pub name: String,
    /// MCU restart method.
    ///
    /// Resolved to the effective method: `command` for a non-serial MCU, the
    /// config's own value (default `arduino`) for a serial one. The physical
    /// restart is still pending for `cheetah`/`rpi_usb` — only `command` and
    /// `arduino` have a path today ([`McuRestartMethod`]).
    pub restart_method: McuRestartMethod,
    /// What to open, described but not opened.
    pub transport: Transport,
    /// Which mechanism `rpi_usb` uses to switch the port's power.
    ///
    /// Only meaningful with `restart_method: rpi_usb`; see
    /// [`UsbPowerMethod`] and `interface/usb.rs`.
    pub usb_power: UsbPowerMethod,
}

/// The transport a section asks for, described before anything is opened.
///
/// Keeping the description separate from the opened [`Interface`] is what lets
/// the host reset a firmware on its **closed** port — a reset needs the port
/// path, and the device must not be open — between parsing the section and
/// opening the device (see [`McuConfig::open`]).
///
/// `TestDevice` is **not** constructed from config — tests build it directly in
/// code and pass it through `Interface::new()`.  `SimulatorDevice` is
/// constructed from `test: dict=<path>` in the `upstream` harness.
#[derive(Debug, Clone, PartialEq)]
pub enum Transport {
    /// A tty at a line speed.
    Serial { path: String, baud: u32 },
    /// A Klipper can-serial link (a CAN interface and node id).
    Can {
        interface: String,
        uuid: [u8; 6],
        nodeid: u32,
    },
    /// Klipper's host library, loaded from a shared object.
    Host { library: String },
    /// A dictionary-driven fake MCU, in test builds (`test: dict=<path>`).
    #[cfg(test)]
    Simulator(String),
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
/// `test` exists in test builds only — the `upstream` harness uses
/// `test: dict=<path>` to inject a dictionary-driven fake MCU.
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
    pub fn new(section: &ConfigWrapper) -> Result<Self, ConfigError> {
        // Upstream names an MCU by its config section with the `mcu ` prefix
        // stripped (`klippy/mcu.py:1151-1153`): the main `[mcu]` is "mcu", and
        // `[mcu zboard]` is "zboard". Not an empty string for the main one.
        let name = section
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| section.section().id.clone());
        let transport = Self::transport_for(section)?;
        let serial = matches!(transport, Transport::Serial { .. });

        // The option only means something on a serial port. Say so rather than
        // dropping it silently, then report what the MCU will actually use.
        if !serial && section.has("restart_method") {
            warn!(
                "MCU '{name}' sets restart_method, which only applies to a serial MCU; \
                 resetting with command"
            );
        }
        let restart_method = Self::parse_restart_method(section, serial)?;
        let usb_power = Self::parse_usb_power(section, &restart_method)?;

        Ok(Self {
            name,
            restart_method,
            transport,
            usb_power,
        })
    }

    /// Resolve the `usb_power` option: which mechanism `rpi_usb` uses.
    ///
    /// Only meaningful with `restart_method: rpi_usb`; a value given for another
    /// method is reported and ignored, the way `restart_method` itself is off a
    /// serial MCU.
    fn parse_usb_power(
        section: &ConfigWrapper,
        restart_method: &McuRestartMethod,
    ) -> Result<UsbPowerMethod, ConfigError> {
        let Some(text) = section.get_str("usb_power") else {
            return Ok(UsbPowerMethod::default());
        };
        let method = UsbPowerMethod::parse(&text).ok_or_else(|| {
            ConfigError::new(format!(
                "MCU '{}' has an invalid usb_power: '{text}' (expected one of {})",
                section.identifier(),
                UsbPowerMethod::CHOICES.join(", ")
            ))
        })?;
        if *restart_method != McuRestartMethod::RpiUsb {
            warn!(
                "MCU '{}' sets usb_power, which only applies to restart_method 'rpi_usb'; \
                 ignored",
                section.identifier()
            );
        }
        Ok(method)
    }

    /// Open the transport this config describes.
    ///
    /// # Errors
    /// Returns the transport's own message (`serial: …`, `canbus: …`,
    /// `host_library: …`) when it cannot be opened.
    pub fn open(&self) -> Result<Interface, String> {
        // Cheetah boards need RTS deasserted for the **whole** connection, not
        // just while resetting, or the board drops into its bootloader
        // (`klippy/mcu.py:703-705`). Every other method wants the line left as
        // the driver opened it.
        let rts = self.restart_method != McuRestartMethod::Cheetah;
        self.transport.open(rts)
    }

    /// Resolve `restart_method` for an MCU whose transport is (or is not) serial.
    ///
    /// Upstream reads the option **only** when the MCU is on a serial port
    /// (`klippy/mcu.py:668-671`); every other transport resets with `command`.
    /// An unknown value is a config error rather than silently becoming the
    /// default — upstream's `getchoice` refuses it too (`:666-671`).
    ///
    /// The `serial` flag is a parameter rather than a look at the interface so
    /// the rule can be exercised without a real serial port.
    fn parse_restart_method(
        section: &ConfigWrapper,
        serial: bool,
    ) -> Result<McuRestartMethod, ConfigError> {
        if !serial {
            return Ok(McuRestartMethod::Command);
        }
        match section.get_str("restart_method") {
            None => Ok(McuRestartMethod::Arduino),
            Some(text) => McuRestartMethod::parse(&text).ok_or_else(|| {
                ConfigError::new(format!(
                    "MCU '{}' has an invalid restart_method: '{text}' (expected one of {})",
                    section.identifier(),
                    McuRestartMethod::CHOICES.join(", ")
                ))
            }),
        }
    }

    /// Describe the transport the section asks for, without opening it.
    ///
    /// One connection key selects it. Two of them is a configuration mistake, not
    /// a preference order, so it is reported rather than resolved silently. Every
    /// check lives here; [`Transport::open`] only performs the side effects.
    fn transport_for(section: &ConfigWrapper) -> Result<Transport, ConfigError> {
        let requested: Vec<&str> = interface_keys()
            .iter()
            .copied()
            .filter(|key| section.has(key))
            .collect();
        if requested.len() > 1 {
            return Err(ConfigError::new(format!(
                "MCU '{}' sets more than one interface: {}",
                section.identifier(),
                requested.join(", ")
            )));
        }

        if let Some(text) = section.get_str("canbus_uuid") {
            let uuid = parse_canbus_uuid(&text).map_err(|e| {
                ConfigError::new(format!(
                    "MCU '{}' has an invalid canbus_uuid: {e}",
                    section.identifier()
                ))
            })?;
            let interface = section
                .get_str("canbus_interface")
                .unwrap_or_else(|| "can0".to_string());
            // Klipper hands out node ids from its `[canbus_ids]` section; klipperx
            // has no such allocator yet, so the section states the id itself.
            let nodeid = match section.get_str("canbus_nodeid") {
                Some(text) => match text.parse::<u32>() {
                    Ok(nodeid) if (1..=MAX_CANBUS_NODEID).contains(&nodeid) => nodeid,
                    _ => {
                        return Err(ConfigError::new(format!(
                            "MCU '{}' has an invalid canbus_nodeid: '{text}' \
                             (expected 1..={MAX_CANBUS_NODEID})",
                            section.identifier()
                        )))
                    }
                },
                None => {
                    return Err(ConfigError::new(format!(
                        "MCU '{}' is on a CAN bus, so it needs a canbus_nodeid \
                         (klipperx does not allocate one yet)",
                        section.identifier()
                    )))
                }
            };
            return Ok(Transport::Can {
                interface,
                uuid,
                nodeid,
            });
        }

        if section.has("canbus_nodeid") || section.has("canbus_interface") {
            return Err(ConfigError::new(format!(
                "MCU '{}' needs a canbus_uuid to go with its CAN settings",
                section.identifier()
            )));
        }

        if let Some(path) = section.get_str("host_library") {
            return Ok(Transport::Host { library: path });
        }

        if let Some(path) = section.get_str("serial") {
            let baud = match section.get_str("baud") {
                // A rate of zero would ask the kernel to hang the line up.
                Some(text) => match text.parse::<u32>() {
                    Ok(baud) if baud > 0 => baud,
                    _ => {
                        return Err(ConfigError::new(format!(
                            "MCU '{}' has an invalid baud: '{text}'",
                            section.identifier()
                        )))
                    }
                },
                None => crate::core::klippy::interface::devices::serial::DEFAULT_BAUD,
            };
            return Ok(Transport::Serial { path, baud });
        }

        #[cfg(test)]
        if let Some(test_value) = section.get_str("test") {
            if let Some(path) = test_value.trim().strip_prefix("dict=") {
                return Ok(Transport::Simulator(path.trim().to_string()));
            }
        }

        let how = if cfg!(test) {
            "set host_library: <libklipper_host.so>, serial: <tty>, canbus_uuid: <hex>, \
             or test: dict=<dictionary>"
        } else {
            "set host_library: <libklipper_host.so>, serial: <tty>, or canbus_uuid: <hex>"
        };
        Err(ConfigError::new(format!(
            "MCU '{}' needs an interface: {how}",
            section.identifier()
        )))
    }
}

impl Transport {
    /// Open the described transport.
    ///
    /// This is where the side effects — opening a tty, a CAN socket, or the host
    /// library — and the errors that name them live. Parsing, and every config
    /// check, happened in [`McuConfig::new`].
    ///
    /// `rts` is the state a serial port is left in after opening; only the
    /// [`Transport::Serial`] arm looks at it. The other transports ignore it.
    ///
    /// # Errors
    /// Returns a message prefixed with the transport that failed.
    pub fn open(&self, rts: bool) -> Result<Interface, String> {
        match self {
            Transport::Serial { path, baud } => {
                let device = SerialDevice::open(path, *baud).map_err(|e| format!("serial: {e}"))?;
                if !rts {
                    device.set_rts(false).map_err(|e| format!("serial: {e}"))?;
                }
                Ok(Interface::from_serial(device))
            }
            Transport::Can {
                interface,
                uuid,
                nodeid,
            } => {
                Interface::canserial(interface, *uuid, *nodeid).map_err(|e| format!("canbus: {e}"))
            }
            Transport::Host { library } => {
                Interface::host(library).map_err(|e| format!("host_library: {e}"))
            }
            #[cfg(test)]
            Transport::Simulator(path) => SimulatorDevice::new(path)
                .map(Interface::simulator)
                .map_err(|e| format!("test: {e}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;

    /// Helper that creates a `[mcu]` section with `serial: /fake/tty`.
    fn make_section(_test_lines: &[&str]) -> ConfigSection {
        let mut section = ConfigSection::new("mcu", None);
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/fake/tty".to_string()),
        );
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

    /// Wrap a hand-built section the way the loader does.
    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    #[test]
    fn test_parse_mcu_config_with_name() {
        let mut section = make_section(&["01 02"]);
        section.sub = Some("mcu0".to_string());
        let result = McuConfig::new(&wrap(&section));
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
        // A serial MCU reads the option; the value is taken as written.
        let method = McuConfig::parse_restart_method(&wrap(&section), true).unwrap();
        assert_eq!(method, McuRestartMethod::RpiUsb);
    }

    #[test]
    fn test_a_serial_mcu_defaults_to_arduino() {
        let section = make_section(&["01 02"]);
        let method = McuConfig::parse_restart_method(&wrap(&section), true).unwrap();
        assert_eq!(method, McuRestartMethod::Arduino);
    }

    #[test]
    fn test_an_unknown_restart_method_is_a_config_error() {
        let mut section = make_section(&["01 02"]);
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("bogus".to_string()),
        );
        let err = McuConfig::parse_restart_method(&wrap(&section), true).unwrap_err();
        assert!(err.to_string().contains("restart_method"), "{err}");
        assert!(err.to_string().contains("bogus"), "{err}");
        // The error names the valid choices, so the config can be fixed.
        assert!(err.to_string().contains("cheetah"), "{err}");
    }

    #[test]
    fn test_restart_method_spellings_round_trip() {
        // Every advertised choice parses back to a method that spells itself the
        // same way the config option did.
        for spelling in McuRestartMethod::CHOICES {
            let method = McuRestartMethod::parse(spelling).unwrap();
            assert_eq!(method.as_str(), *spelling);
        }
    }

    #[test]
    fn test_a_non_serial_mcu_resets_with_command_and_ignores_the_option() {
        let mut section = make_section(&["01 02"]);
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("cheetah".to_string()),
        );
        // Upstream does not read the option off serial, so a value that is
        // meaningless there is not an error — it simply does not apply.
        let method = McuConfig::parse_restart_method(&wrap(&section), false).unwrap();
        assert_eq!(method, McuRestartMethod::Command);
    }

    #[test]
    fn test_usb_power_defaults_to_auto_and_is_validated() {
        // Unset: `auto`.
        let section = make_section(&["01 02"]);
        assert_eq!(
            McuConfig::parse_usb_power(&wrap(&section), &McuRestartMethod::RpiUsb).unwrap(),
            UsbPowerMethod::Auto
        );

        // A valid value is taken as written.
        let with = |value: &str| {
            let mut section = make_section(&["01 02"]);
            section.parameters.insert(
                "usb_power".to_string(),
                ConfigValue::Single(value.to_string()),
            );
            section
        };
        assert_eq!(
            McuConfig::parse_usb_power(&wrap(&with("sysfs")), &McuRestartMethod::RpiUsb).unwrap(),
            UsbPowerMethod::Sysfs
        );

        // A typo is a config error, not a silent default.
        let err = McuConfig::parse_usb_power(&wrap(&with("bogus")), &McuRestartMethod::RpiUsb)
            .unwrap_err();
        assert!(err.to_string().contains("usb_power"), "{err}");
        assert!(err.to_string().contains("bogus"), "{err}");

        // With another restart method the option has no effect; it is reported
        // and ignored rather than refused.
        assert_eq!(
            McuConfig::parse_usb_power(&wrap(&with("sysfs")), &McuRestartMethod::Arduino).unwrap(),
            UsbPowerMethod::Sysfs
        );
    }

    #[test]
    fn test_parse_mcu_config_no_interface() {
        let section = ConfigSection::new("mcu", None);
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        // The error has to say how to fix the section, not just that it is wrong.
        assert!(err.to_string().contains("needs an interface"), "{err}");
        assert!(err.to_string().contains("host_library"), "{err}");
    }

    #[test]
    fn test_host_library_key_becomes_the_host_transport() {
        // Parsing routes it to the host transport; opening is what touches the
        // library, so the failure is the library's, not "needs an interface".
        let section = section_with("host_library", "/nonexistent/libklipper_host.so");
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(&config.transport, Transport::Host { .. }));

        let err = config.open().unwrap_err();
        assert!(err.starts_with("host_library: "), "{err}");
        assert!(
            err.to_string().contains("/nonexistent/libklipper_host.so"),
            "{err}"
        );
    }

    #[test]
    fn test_two_interface_keys_are_rejected() {
        let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/dev/ttyACM0".to_string()),
        );

        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("more than one interface"), "{err}");
        assert!(err.to_string().contains("host_library, serial"), "{err}");
    }

    #[test]
    fn test_serial_key_becomes_the_serial_transport() {
        let section = section_with("serial", "/dev/not-a-serial-port");
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            config.transport,
            Transport::Serial { ref path, .. } if path == "/dev/not-a-serial-port"
        ));

        // A port that cannot be opened proves the routing: the error names the
        // port, which only the serial device's own error does.
        let err = config.open().unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
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
    fn test_canbus_keys_become_the_can_transport() {
        let section = can_section(&[("canbus_interface", "can99")]);
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            config.transport,
            Transport::Can { ref interface, .. } if interface == "can99"
        ));

        // No CAN interface in the test environment, so opening shows up as the
        // socket's error naming the interface we asked for.
        let err = config.open().unwrap_err();
        assert!(err.starts_with("canbus: "), "{err}");
        assert!(err.to_string().contains("can99"), "{err}");
    }

    #[test]
    fn test_canbus_uuid_is_parsed_as_klipper_writes_it() {
        assert_eq!(
            parse_canbus_uuid("11aa22bb33cc").unwrap(),
            [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        );
        for bad in ["11aa22bb33c", "11aa22bb33ccdd", "11aa22bb33cg", ""] {
            let section = can_section(&[("canbus_uuid", bad)]);
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(
                err.to_string().contains("invalid canbus_uuid"),
                "{bad}: {err}"
            );
        }
    }

    #[test]
    fn test_canbus_nodeid_is_validated() {
        for bad in ["0", "fast", "900"] {
            let section = can_section(&[("canbus_nodeid", bad)]);
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(
                err.to_string().contains("invalid canbus_nodeid"),
                "{bad}: {err}"
            );
        }

        // A valid node id gets as far as the socket, which is where it fails here.
        let err = McuConfig::new(&wrap(&can_section(&[])))
            .unwrap()
            .open()
            .unwrap_err();
        assert!(
            err.to_string().contains("no CAN interface named 'can0'"),
            "{err}"
        );
    }

    #[test]
    fn test_canbus_settings_without_a_uuid_are_reported() {
        let section = section_with("canbus_nodeid", "2");
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("needs a canbus_uuid"), "{err}");

        let section = section_with("canbus_interface", "can0");
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("needs a canbus_uuid"), "{err}");
    }

    #[test]
    fn test_invalid_baud_is_rejected_before_opening_the_port() {
        let mut section = section_with("serial", "/dev/not-a-serial-port");
        section
            .parameters
            .insert("baud".to_string(), ConfigValue::Single("fast".to_string()));
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("invalid baud"), "{err}");
        assert!(err.to_string().contains("fast"), "{err}");
    }

    #[test]
    fn test_parse_mcu_config_with_dict_interface() {
        let mut section = ConfigSection::new("mcu", None);
        section.parameters.insert(
            "test".to_string(),
            ConfigValue::Single("dict=/fake/path.dict".to_string()),
        );
        let result = McuConfig::new(&wrap(&section));
        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.name, "mcu");
        assert!(matches!(config.transport, Transport::Simulator(_)));
        // A `test: dict=` MCU is not serial, so it resets with `command` like
        // every non-serial transport.
        assert_eq!(config.restart_method, McuRestartMethod::Command);
    }
}
