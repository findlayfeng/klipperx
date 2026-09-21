//! `[i2c_device <name>]` — a raw I2C device a client can read and write.
//!
//! The first *consumer* of the I2C stack (`mcu/resource/i2c.rs`): it reads the
//! `i2c_*` options, asks its MCU for an [`McuI2c`], and registers two debug
//! commands so the bus can be exercised on a real board before a real sensor
//! exists:
//!
//! | command | meaning |
//! |---|---|
//! | `IIC_WRITE DEVICE=<name> DATA=<hex>` | write bytes |
//! | `IIC_READ DEVICE=<name> WRITE=<hex> READ_LEN=<n>` | write, then read `n` bytes |
//!
//! The `IIC_` prefix (not `I2C_`) is forced by the G-Code grammar: an extended
//! command name may not have a digit in its second character, so `I2C_READ`
//! would be rejected at registration (`gcode::is_valid_extended_name`), exactly
//! as upstream Klipper rejects it.
//!
//! Upstream has no generic `[i2c_device]` section: a sensor reads the same
//! options through `MCU_I2C_from_config` (`klippy/extras/bus.py:316`) and adds
//! the protocol on top. This section is that constructor plus a test interface,
//! so F7 can be verified without a sensor driver.
//!
//! # Options
//!
//! | option | meaning |
//! |---|---|
//! | `i2c_mcu` | the MCU to use (default `mcu`) |
//! | `i2c_address` | the 7-bit device address, required (`0..=127`) |
//! | `i2c_speed` | clock in Hz (default 100000, minimum 100000) |
//! | `i2c_bus` | hardware bus name (default: the bus the firmware names `0`) |
//! | `i2c_software_scl_pin` / `i2c_software_sda_pin` | bit-bang on these pins instead |
//!
//! Setting one software pin but not the other is a config error, as upstream
//! requires both or neither.
//!
//! # What is not here
//!
//! A device protocol, and any retry policy. The section only moves bytes; a
//! sensor declares what they mean, and the `*_from_config` default flags
//! (`default_addr`, `async_write_only`) are not options because nothing uses
//! them yet.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigSection;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{I2cMode, McuI2c, McuObject, DEFAULT_SPEED};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::bus_debug::{block_on, hex_decode, hex_encode, parse_int};

// Loaded after `[board_pins]` (order 30), because a software bus may name its
// pins through an alias.
section!("i2c_device", order = 40, prefix = load_config_prefix);

/// The lowest clock upstream accepts (its `minval` for `i2c_speed`).
const MIN_SPEED: u32 = 100_000;

/// One configured `[i2c_device <name>]`.
pub struct I2cDevice {
    /// The name `IIC_*` addresses this device by: the section's sub.
    name: String,
    /// The bus resource, shared with the two command handlers.
    device: Arc<McuI2c>,
    /// The 7-bit device address, for `get_status`.
    address: u8,
    /// The configured clock, for `get_status`.
    speed: u32,
}

impl I2cDevice {
    /// Build the device from its section and register the debug commands.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when an option is
    /// missing, unparseable, or names an MCU or pin this machine does not have.
    pub fn new(section: &ConfigSection, printer: &Printer) -> Result<Self, String> {
        let identifier = section.identifier();
        let name = section.sub.clone().ok_or_else(|| {
            format!("Section '{identifier}' must be a '[i2c_device <name>]' section")
        })?;

        let address = parse_int(section, "i2c_address")?.ok_or_else(|| {
            format!("Option 'i2c_address' in section '{identifier}' is not specified")
        })?;
        if !(0..=127).contains(&address) {
            return Err(format!(
                "Option 'i2c_address' in section '{identifier}' must be between 0 and 127"
            ));
        }
        let address = address as u8;

        let speed = parse_int(section, "i2c_speed")?.unwrap_or(i64::from(DEFAULT_SPEED));
        if !(i64::from(MIN_SPEED)..=i64::from(u32::MAX)).contains(&speed) {
            return Err(format!(
                "Option 'i2c_speed' in section '{identifier}' must be at least {MIN_SPEED}"
            ));
        }
        let speed = speed as u32;

        let mcu_name = section
            .get_str("i2c_mcu")
            .map(str::trim)
            .unwrap_or("mcu")
            .to_string();
        let object_name = mcu_object_name(&mcu_name);
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&object_name)
            .ok_or_else(|| format!("Section '{identifier}': unknown MCU '{mcu_name}'"))?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        let mode = match (
            section.get_str("i2c_software_scl_pin"),
            section.get_str("i2c_software_sda_pin"),
        ) {
            (Some(scl), Some(sda)) => {
                // Validate (and reserve) the pins now; the numbers are filled
                // in at build time, when the firmware dictionary exists.
                let scl_params = pins
                    .lookup_pin(scl, false, false, Some("scl"))
                    .map_err(|err| format!("{identifier}: {err}"))?;
                let sda_params = pins
                    .lookup_pin(sda, false, false, Some("sda"))
                    .map_err(|err| format!("{identifier}: {err}"))?;
                if scl_params.chip_name != mcu_name || sda_params.chip_name != mcu_name {
                    return Err(format!(
                        "Section '{identifier}': i2c pins must be on the same mcu '{mcu_name}'"
                    ));
                }
                I2cMode::Software {
                    scl_pin: scl_params.pin,
                    sda_pin: sda_params.pin,
                    speed,
                }
            }
            (None, None) => I2cMode::Hardware {
                bus: section.get_str("i2c_bus").map(str::to_string),
                speed,
            },
            _ => {
                return Err(format!(
                    "Section '{identifier}': both 'i2c_software_scl_pin' and \
                     'i2c_software_sda_pin' must be set"
                ));
            }
        };

        let device = mcu_object.setup_i2c(mode, address);

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let write_handler: CommandHandler = {
            let device = Arc::clone(&device);
            Arc::new(move |gcmd| cmd_i2c_write(&device, gcmd))
        };
        gcode
            .register_mux_command(
                "IIC_WRITE",
                "DEVICE",
                Some(&name),
                write_handler,
                Some("Write bytes to an I2C device (debug)"),
            )
            .map_err(|err| format!("{identifier}: {err}"))?;
        let read_handler: CommandHandler = {
            let device = Arc::clone(&device);
            Arc::new(move |gcmd| cmd_i2c_read(&device, gcmd))
        };
        gcode
            .register_mux_command(
                "IIC_READ",
                "DEVICE",
                Some(&name),
                read_handler,
                Some("Write then read bytes from an I2C device (debug)"),
            )
            .map_err(|err| format!("{identifier}: {err}"))?;

        Ok(Self {
            name,
            device,
            address,
            speed,
        })
    }

    /// The name `IIC_*` addresses this device by.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The bus resource, for a sensor extra that looks this section up.
    pub fn device(&self) -> &Arc<McuI2c> {
        &self.device
    }
}

impl PrinterObject for I2cDevice {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "address": self.address, "speed": self.speed })
    }
}

impl std::fmt::Debug for I2cDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("I2cDevice")
            .field("name", &self.name)
            .field("address", &self.address)
            .field("speed", &self.speed)
            .finish_non_exhaustive()
    }
}

/// The printer-object name of an MCU: `mcu`, or `mcu <name>`.
///
/// Upstream's `get_printer_mcu` (`klippy/mcu.py:1251`), which is how every
/// `*_from_config` finds its bus.
fn mcu_object_name(mcu_name: &str) -> String {
    if mcu_name == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {mcu_name}")
    }
}

/// `IIC_WRITE DEVICE=<name> DATA=<hex>` — send bytes, no read.
fn cmd_i2c_write(device: &Arc<McuI2c>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let data = hex_decode(&gcmd.get_str("DATA")?)?;
    // Bring-up probing: a NACK is a result to report, not a reason to stop the
    // machine (`McuI2c::write` is the driver-facing form).
    block_on(device.write_without_shutdown(&data))?;
    gcmd.respond_info("i2c write ok");
    Ok(())
}

/// `IIC_READ DEVICE=<name> WRITE=<hex> READ_LEN=<n>` — write, then read.
fn cmd_i2c_read(device: &Arc<McuI2c>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let write = match gcmd.parameters().get("WRITE") {
        Some(text) => hex_decode(text)?,
        None => Vec::new(),
    };
    let read_len = gcmd.get_int("READ_LEN")?;
    if !(0..=255).contains(&read_len) {
        return Err(CommandError::new("READ_LEN must be between 0 and 255"));
    }
    let data = block_on(device.transfer_without_shutdown(&write, read_len as u32))?;
    gcmd.respond_info(&format!("i2c read: {}", hex_encode(&data)));
    Ok(())
}

/// Upstream's `load_config_prefix` for `[i2c_device <name>]`.
pub fn load_config_prefix(
    section: &ConfigSection,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, String> {
    Ok(Arc::new(I2cDevice::new(section, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::ConfigValue;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("i2c_device", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A ready printer with `gcode`, `pins`, and one registered `[mcu]`.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mcu = McuObject::new(ConfigSection::new("mcu", None), &printer).unwrap();
        printer.add_object("mcu", Arc::new(mcu)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    #[test]
    fn test_a_hardware_device_needs_an_address() {
        let printer = printer();

        let err = I2cDevice::new(&section("accel", &[]), &printer).unwrap_err();

        assert_eq!(
            err,
            "Option 'i2c_address' in section 'i2c_device accel' is not specified"
        );
    }

    #[test]
    fn test_an_address_out_of_range_is_refused() {
        let printer = printer();

        let err =
            I2cDevice::new(&section("accel", &[("i2c_address", "128")]), &printer).unwrap_err();

        assert_eq!(
            err,
            "Option 'i2c_address' in section 'i2c_device accel' must be between 0 and 127"
        );
    }

    #[test]
    fn test_a_device_registers_both_debug_commands() {
        let printer = printer();
        let device = I2cDevice::new(&section("accel", &[("i2c_address", "0x68")]), &printer);

        // `0x68` is not decimal; the option is an integer, like upstream's.
        assert!(device.is_err());

        I2cDevice::new(&section("accel", &[("i2c_address", "104")]), &printer).unwrap();
        let commands = gcode(&printer).command_help();
        assert!(commands.contains_key("IIC_WRITE"), "{commands:?}");
        assert!(commands.contains_key("IIC_READ"), "{commands:?}");
    }

    #[test]
    fn test_only_one_software_pin_is_refused() {
        let printer = printer();

        let err = I2cDevice::new(
            &section(
                "accel",
                &[("i2c_address", "104"), ("i2c_software_scl_pin", "PA0")],
            ),
            &printer,
        )
        .unwrap_err();

        assert!(err.contains("both 'i2c_software_scl_pin'"), "{err}");
    }

    #[test]
    fn test_an_unknown_mcu_names_the_section() {
        let printer = printer();

        let err = I2cDevice::new(
            &section("accel", &[("i2c_address", "104"), ("i2c_mcu", "zboard")]),
            &printer,
        )
        .unwrap_err();

        assert_eq!(err, "Section 'i2c_device accel': unknown MCU 'zboard'");
    }

    #[test]
    fn test_a_software_device_accepts_pins_on_its_mcu() {
        let printer = printer();
        let device = I2cDevice::new(
            &section(
                "accel",
                &[
                    ("i2c_address", "104"),
                    ("i2c_software_scl_pin", "PA0"),
                    ("i2c_software_sda_pin", "PA1"),
                ],
            ),
            &printer,
        )
        .unwrap();

        assert_eq!(device.name(), "accel");
    }

    #[test]
    fn test_the_software_pins_must_share_the_mcu() {
        let printer = printer();
        // Register a second MCU so the pin can name a different chip.
        let other = McuObject::new(ConfigSection::new("mcu", Some("other")), &printer).unwrap();
        printer.add_object("mcu other", Arc::new(other)).unwrap();

        let err = I2cDevice::new(
            &section(
                "accel",
                &[
                    ("i2c_address", "104"),
                    ("i2c_software_scl_pin", "PA0"),
                    ("i2c_software_sda_pin", "other:PA1"),
                ],
            ),
            &printer,
        )
        .unwrap_err();

        assert!(err.contains("must be on the same mcu 'mcu'"), "{err}");
    }

    #[test]
    fn test_a_ready_device_reports_its_address_and_speed() {
        let printer = printer();
        let device = I2cDevice::new(
            &section("accel", &[("i2c_address", "104"), ("i2c_speed", "400000")]),
            &printer,
        )
        .unwrap();

        assert_eq!(device.name(), "accel");
        let status = device.get_status(0.0);
        assert_eq!(status["address"], 104);
        assert_eq!(status["speed"], 400_000);
    }
}
