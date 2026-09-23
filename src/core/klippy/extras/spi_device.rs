//! `[spi_device <name>]` — a raw SPI device a client can shift bytes through.
//!
//! The first *consumer* of the SPI stack (`mcu/resource/spi.rs`): it reads the
//! `spi_*` options, asks its MCU for an [`McuSpi`], and registers two debug
//! commands so the bus can be exercised on a real board:
//!
//! | command | meaning |
//! |---|---|
//! | `SPI_TRANSFER DEVICE=<name> DATA=<hex>` | full-duplex transfer, reply is the bytes clocked in |
//! | `SPI_SEND DEVICE=<name> DATA=<hex>` | shift bytes out, ignore the reply |
//!
//! A single `SPI_TRANSFER` is one chip-select pulse, so a device protocol is
//! written into one buffer: a JEDEC-ID read on a W25 flash is
//! `SPI_TRANSFER DEVICE=flash DATA=9f000000`, and the reply's last three bytes
//! are the ID (`ef 40 17` for a W25Q64).
//!
//! Upstream has no generic `[spi_device]` section: a device reads the same
//! options through `MCU_SPI_from_config` (`klippy/extras/bus.py:124`) and adds
//! the protocol on top. This section is that constructor plus a test interface,
//! so F6 can be verified without a device driver.
//!
//! # Options
//!
//! | option | meaning |
//! |---|---|
//! | `spi_mcu` | the MCU to use (default `mcu`) |
//! | `cs_pin` | chip-select pin, or `None` for `config_spi_without_cs` |
//! | `cs_active_high` | whether CS is active high (default false) |
//! | `spi_mode` | SPI mode 0..=3 (CPOL/CPHA, default 0) |
//! | `spi_speed` | clock in Hz (default 100000, minimum 100000) |
//! | `spi_bus` | hardware bus name (default: the bus the firmware names `0`) |
//! | `spi_software_{miso,mosi,sclk}_pin` | bit-bang on these pins instead |
//!
//! # What is not here
//!
//! A device protocol, a shutdown message (`config_spi_shutdown`), and the
//! `spi_transfer_with_preface` optimisation. The section only moves bytes; a
//! driver declares what they mean.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuObject, McuSpi, SpiMode};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::bus_debug::{hex_decode, hex_encode};

// Loaded after `[board_pins]` (order 30), because a software bus or the CS pin
// may be named through an alias.
section!("spi_device", order = 50, prefix = load_config_prefix);

/// Default SPI clock speed in Hz (`klippy/extras/bus.py:127`).
const DEFAULT_SPEED: u32 = 100_000;

/// The lowest clock upstream accepts (its `minval` for `spi_speed`).
const MIN_SPEED: u32 = 100_000;

/// Default SPI mode when a section does not set `spi_mode`.
const DEFAULT_SPI_MODE: u8 = 0;

/// An [`McuSpi`] plus the options a `[spi_device]` reports in its status.
pub(crate) struct SpiSetup {
    /// The configured bus resource.
    pub device: Arc<McuSpi>,
    /// The configured clock, in Hz.
    pub speed: u32,
    /// Whether chip select is active high.
    pub cs_active_high: bool,
}

/// Build an [`McuSpi`] from the `spi_*` options in `config`.
///
/// Upstream's `MCU_SPI_from_config` (`klippy/extras/bus.py:124`): `pin_option`
/// names the chip-select option (`cs_pin` for `[spi_device]`, `sensor_pin` for
/// the SPI temperature chips), and `default_mode` / `default_speed` apply when
/// `spi_mode` / `spi_speed` are absent.
pub(crate) fn mcu_spi_from_config(
    config: &ConfigWrapper,
    printer: &Printer,
    default_mode: u8,
    pin_option: &str,
    default_speed: u32,
) -> Result<SpiSetup, ConfigError> {
    let identifier = config.identifier();
    let mcu_name = config
        .get_str("spi_mcu")
        .map(|text| text.trim().to_string())
        .unwrap_or_else(|| "mcu".to_string());
    let object_name = mcu_object_name(&mcu_name);
    let mcu_object = printer
        .lookup_object_as::<McuObject>(&object_name)
        .ok_or_else(|| {
            ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
        })?;

    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");

    // Chip select. `None` means the firmware does not drive one:
    // `config_spi_without_cs`.
    let cs_pin = match config
        .get_str(pin_option)
        .map(|text| text.trim().to_string())
    {
        None => None,
        Some(description) if description == "None" => None,
        Some(description) => {
            let params = pins
                .lookup_pin(&description, false, false, Some("cs"))
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
            if params.chip_name != mcu_name {
                return Err(ConfigError::new(format!(
                    "Section '{identifier}': {pin_option} must be on mcu '{mcu_name}'"
                )));
            }
            Some(params)
        }
    };
    let cs_active_high = config.get_bool("cs_active_high", Some(false))?;

    let speed = config.get_int("spi_speed", Some(i64::from(default_speed)))?;
    if !(i64::from(MIN_SPEED)..=i64::from(u32::MAX)).contains(&speed) {
        return Err(ConfigError::new(format!(
            "Option 'spi_speed' in section '{identifier}' must be at least {MIN_SPEED}"
        )));
    }
    let speed = speed as u32;

    let spi_mode = config.get_int("spi_mode", Some(i64::from(default_mode)))?;
    if !(0..=3).contains(&spi_mode) {
        return Err(ConfigError::new(format!(
            "Option 'spi_mode' in section '{identifier}' must be between 0 and 3"
        )));
    }
    let spi_mode = spi_mode as u8;

    let mode = match (
        config.get_str("spi_software_miso_pin"),
        config.get_str("spi_software_mosi_pin"),
        config.get_str("spi_software_sclk_pin"),
    ) {
        (Some(miso), Some(mosi), Some(sclk)) => {
            let software = [("miso", miso), ("mosi", mosi), ("sclk", sclk)];
            let mut pins_out = Vec::with_capacity(3);
            for (role, description) in software {
                let params = pins
                    .lookup_pin(&description, false, false, Some(role))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                if params.chip_name != mcu_name {
                    return Err(ConfigError::new(format!(
                        "Section '{identifier}': spi_software_{role}_pin must be on mcu \
                         '{mcu_name}'"
                    )));
                }
                pins_out.push(params.pin);
            }
            SpiMode::Software {
                miso_pin: pins_out[0].clone(),
                mosi_pin: pins_out[1].clone(),
                sclk_pin: pins_out[2].clone(),
                speed,
                mode: spi_mode,
            }
        }
        (None, None, None) => SpiMode::Hardware {
            bus: config.get_str("spi_bus"),
            speed,
            mode: spi_mode,
        },
        _ => {
            return Err(ConfigError::new(format!(
                "Section '{identifier}': all three of 'spi_software_miso_pin', \
                 'spi_software_mosi_pin' and 'spi_software_sclk_pin' must be set"
            )));
        }
    };

    let device = mcu_object.setup_spi(mode, cs_pin, cs_active_high);
    Ok(SpiSetup {
        device,
        speed,
        cs_active_high,
    })
}

/// One configured `[spi_device <name>]`.
pub struct SpiDevice {
    /// The name `SPI_*` addresses this device by: the section's sub.
    name: String,
    /// The bus resource, shared with the two command handlers.
    device: Arc<McuSpi>,
    /// Whether CS is active high, for `get_status`.
    cs_active_high: bool,
    /// The configured clock, for `get_status`.
    speed: u32,
}

impl SpiDevice {
    /// Build the device from its section and register the debug commands.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when an option is
    /// missing, unparseable, or names an MCU or pin this machine does not have.
    pub fn new(config: &ConfigWrapper, printer: &Printer) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[spi_device <name>]' section"
            ))
        })?;

        let setup =
            mcu_spi_from_config(config, printer, DEFAULT_SPI_MODE, "cs_pin", DEFAULT_SPEED)?;
        let device = setup.device;
        let cs_active_high = setup.cs_active_high;
        let speed = setup.speed;

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let transfer_handler: CommandHandler = {
            let device = Arc::clone(&device);
            Arc::new(move |gcmd: &GcodeCommand| {
                let device = Arc::clone(&device);
                Box::pin(async move { cmd_spi_transfer(&device, gcmd).await })
            })
        };
        gcode
            .register_mux_command(
                "SPI_TRANSFER",
                "DEVICE",
                Some(&name),
                transfer_handler,
                Some("Full-duplex SPI transfer (debug)"),
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        let send_handler: CommandHandler = {
            let device = Arc::clone(&device);
            Arc::new(move |gcmd: &GcodeCommand| {
                let device = Arc::clone(&device);
                Box::pin(async move { cmd_spi_send(&device, gcmd).await })
            })
        };
        gcode
            .register_mux_command(
                "SPI_SEND",
                "DEVICE",
                Some(&name),
                send_handler,
                Some("Shift bytes out on an SPI device (debug)"),
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self {
            name,
            device,
            cs_active_high,
            speed,
        })
    }

    /// The name `SPI_*` addresses this device by.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The bus resource, for a driver extra that looks this section up.
    pub fn device(&self) -> &Arc<McuSpi> {
        &self.device
    }
}

impl PrinterObject for SpiDevice {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "cs_active_high": self.cs_active_high, "speed": self.speed })
    }
}

impl std::fmt::Debug for SpiDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SpiDevice")
            .field("name", &self.name)
            .field("cs_active_high", &self.cs_active_high)
            .field("speed", &self.speed)
            .finish_non_exhaustive()
    }
}

/// The printer-object name of an MCU: `mcu`, or `mcu <name>`.
///
/// Upstream's `get_printer_mcu` (`klippy/mcu.py:1251`), which is how every
/// `*_from_config` finds its bus.
pub(crate) fn mcu_object_name(mcu_name: &str) -> String {
    if mcu_name == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {mcu_name}")
    }
}

/// `SPI_TRANSFER DEVICE=<name> DATA=<hex>` — full duplex; reply is what came in.
async fn cmd_spi_transfer(device: &Arc<McuSpi>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let data = hex_decode(&gcmd.get_str("DATA")?)?;
    let response = device
        .transfer(&data)
        .await
        .map_err(|err| CommandError::new(err.to_string()))?;
    gcmd.respond_info(&format!("spi transfer: {}", hex_encode(&response)));
    Ok(())
}

/// `SPI_SEND DEVICE=<name> DATA=<hex>` — shift bytes out, no read.
async fn cmd_spi_send(device: &Arc<McuSpi>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let data = hex_decode(&gcmd.get_str("DATA")?)?;
    device
        .send(&data)
        .map_err(|err| CommandError::new(err.to_string()))?;
    gcmd.respond_info("spi send ok");
    Ok(())
}

/// Upstream's `load_config_prefix` for `[spi_device <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(SpiDevice::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("spi_device", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A section and a tracked wrapper for it, leaked so the borrow outlives
    /// the call (tests only).
    fn wrap(name: &str, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
        ConfigWrapper::new(
            Box::leak(Box::new(section(name, options))),
            AccessTracking::shared(),
        )
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
    fn test_a_device_registers_both_debug_commands() {
        let printer = printer();
        SpiDevice::new(
            &wrap("flash", &[("cs_pin", "PA15"), ("spi_bus", "spi1a")]),
            &printer,
        )
        .unwrap();

        let commands = gcode(&printer).command_help();
        assert!(commands.contains_key("SPI_TRANSFER"), "{commands:?}");
        assert!(commands.contains_key("SPI_SEND"), "{commands:?}");
    }

    #[test]
    fn test_a_device_without_cs_is_allowed() {
        let printer = printer();
        let device = SpiDevice::new(
            &wrap("flash", &[("cs_pin", "None"), ("spi_bus", "spi1a")]),
            &printer,
        )
        .unwrap();

        assert_eq!(device.name(), "flash");
    }

    #[test]
    fn test_an_out_of_range_mode_is_refused() {
        let printer = printer();
        let err = SpiDevice::new(
            &wrap(
                "flash",
                &[("cs_pin", "PA15"), ("spi_bus", "spi1a"), ("spi_mode", "4")],
            ),
            &printer,
        )
        .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'spi_mode' in section 'spi_device flash' must be between 0 and 3"
        );
    }

    #[test]
    fn test_a_speed_below_the_minimum_is_refused() {
        let printer = printer();
        let err = SpiDevice::new(
            &wrap(
                "flash",
                &[
                    ("cs_pin", "PA15"),
                    ("spi_bus", "spi1a"),
                    ("spi_speed", "10000"),
                ],
            ),
            &printer,
        )
        .unwrap_err();

        assert!(err.to_string().contains("must be at least 100000"), "{err}");
    }

    #[test]
    fn test_only_some_software_pins_is_refused() {
        let printer = printer();
        let err = SpiDevice::new(
            &wrap(
                "flash",
                &[
                    ("cs_pin", "PA15"),
                    ("spi_software_miso_pin", "PB4"),
                    ("spi_software_mosi_pin", "PB5"),
                ],
            ),
            &printer,
        )
        .unwrap_err();

        assert!(err.to_string().contains("must be set"), "{err}");
    }

    #[test]
    fn test_the_cs_pin_must_be_on_the_named_mcu() {
        let printer = printer();
        let other = McuObject::new(ConfigSection::new("mcu", Some("other")), &printer).unwrap();
        printer.add_object("mcu other", Arc::new(other)).unwrap();

        let err = SpiDevice::new(
            &wrap(
                "flash",
                &[
                    ("cs_pin", "other:PA15"),
                    ("spi_mcu", "mcu"),
                    ("spi_bus", "spi1a"),
                ],
            ),
            &printer,
        )
        .unwrap_err();

        assert!(
            err.to_string().contains("cs_pin must be on mcu 'mcu'"),
            "{err}"
        );
    }

    #[test]
    fn test_an_unknown_mcu_names_the_section() {
        let printer = printer();
        let err = SpiDevice::new(
            &wrap("flash", &[("cs_pin", "PA15"), ("spi_mcu", "zboard")]),
            &printer,
        )
        .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Section 'spi_device flash': unknown MCU 'zboard'"
        );
    }

    #[test]
    fn test_a_ready_device_reports_its_configuration() {
        let printer = printer();
        let device = SpiDevice::new(
            &wrap(
                "flash",
                &[
                    ("cs_pin", "PA15"),
                    ("cs_active_high", "true"),
                    ("spi_bus", "spi1a"),
                    ("spi_speed", "1000000"),
                ],
            ),
            &printer,
        )
        .unwrap();

        let status = device.get_status(0.0);
        assert_eq!(status["cs_active_high"], true);
        assert_eq!(status["speed"], 1_000_000);
    }

    #[test]
    fn test_a_software_device_accepts_pins_on_its_mcu() {
        let printer = printer();
        let device = SpiDevice::new(
            &wrap(
                "flash",
                &[
                    ("cs_pin", "PA15"),
                    ("spi_software_miso_pin", "PB4"),
                    ("spi_software_mosi_pin", "PB5"),
                    ("spi_software_sclk_pin", "PB3"),
                ],
            ),
            &printer,
        )
        .unwrap();

        assert_eq!(device.name(), "flash");
    }
}
