//! `[mcp4018 <name>]` — the MCP4018 single I2C digital potentiometer (upstream's
//! `klippy/extras/mcp4018.py`).
//!
//! One wiper with 128 taps, set over I2C in a single byte. Boards that carry one
//! use a section per axis to trim that axis' stepper current, so the config
//! states the value in amps and `scale` maps it onto the wiper
//! (`scale: 0.773` makes 0.773 A full scale), exactly as `[mcp4451]` does for
//! its four wipers.
//!
//! | option | meaning |
//! |---|---|
//! | `i2c_address` | the device address, **default `0x2f`**; unlike `[mcp4451]`, this section may leave it out |
//! | `i2c_mcu` | the MCU to use (default `mcu`) |
//! | `i2c_speed` | clock in Hz (default 100000, minimum 100000) |
//! | `i2c_bus` / `i2c_software_scl_pin` / `i2c_software_sda_pin` | the hardware bus, or a bit-banged one |
//! | `scale` | full-scale value, above 0, default 1 |
//! | `wiper` | the start value, `0..=scale`, **required** |
//!
//! The wiper is one byte: `int(value * 127. / scale + .5)` (`mcp4018.py:22`),
//! rounded half up, and the write carries that byte alone (`i2c_write([val])`,
//! no register address). Upstream sends the start value from its
//! `klippy:connect` handler, so the wire carries it once the firmware is up;
//! `SET_DIGIPOT DIGIPOT=<name> [WIPER=<value>]` sends a later one through the
//! same single-byte write.
//!
//! # What is not here
//!
//! A `get_status`: upstream's `mcp4018` defines none, so the section is not
//! client-visible.

use std::sync::{Arc, Mutex};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{I2cMode, McuError, McuI2c, McuObject, DEFAULT_SPEED};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// The bus may name its pins through a `[board_pins]` alias, so this loads
// alongside the other order-40 I2C devices.
section!("mcp4018", order = 40, prefix = load_config_prefix);

/// Upstream's `default_addr=0x2f` for `MCU_I2C_from_config`
/// (`mcp4018.py:11`): this section's address may be left out.
const DEFAULT_ADDRESS: u8 = 0x2f;

/// Upstream's `getfloat('scale', 1., above=0.)` default (`mcp4018.py:12`).
const DEFAULT_SCALE: f64 = 1.0;

/// The lowest clock upstream accepts (its `minval` for `i2c_speed`).
const MIN_SPEED: u32 = 100_000;

/// Upstream's `cmd_SET_DIGIPOT_help` (`mcp4018.py:25`).
const CMD_SET_DIGIPOT_HELP: &str = "Set digipot value";

/// One configured `[mcp4018 <name>]`.
pub struct Mcp4018 {
    /// The name `SET_DIGIPOT` addresses this section by: the section's sub.
    name: String,
    /// The bus resource the wiper byte goes to.
    device: Arc<McuI2c>,
    /// Full scale for `wiper`/`WIPER`, and the divisor of the tap conversion.
    scale: f64,
    /// The `wiper` option: what `klippy:connect` sends.
    start_value: f64,
    /// The taps handed to the bus so far, in order. Tests assert these instead
    /// of a real transfer, the same shape as `[mcp4451]`'s record.
    writes: Mutex<Vec<u8>>,
}

impl Mcp4018 {
    /// Read the options, then wire up the section.
    ///
    /// Upstream's `mcp4018.__init__` (`mcp4018.py:9-20`): the bus first (with
    /// its address default), then `scale`, then the required `wiper`, then the
    /// command.
    ///
    /// # Errors
    /// A bad bus option or pin, an `i2c_address` outside `0..=127`, a `scale`
    /// at or below 0, a missing `wiper`, or a `wiper` outside `0..=scale`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[mcp4018 <name>]' section"
            ))
        })?;

        // `MCU_I2C_from_config(config, default_addr=0x2f)`: the address has a
        // default here, unlike `[mcp4451]`'s. This host has no shared helper,
        // so the option reading is the same as `[i2c_device]`'s
        // (`extras/i2c_device.rs`).
        let address = config.get_int("i2c_address", Some(i64::from(DEFAULT_ADDRESS)))?;
        if !(0..=127).contains(&address) {
            return Err(ConfigError::new(format!(
                "Option 'i2c_address' in section '{identifier}' must be between 0 and 127"
            )));
        }
        let address = address as u8;

        let speed = config.get_int("i2c_speed", Some(i64::from(DEFAULT_SPEED)))?;
        if !(i64::from(MIN_SPEED)..=i64::from(u32::MAX)).contains(&speed) {
            return Err(ConfigError::new(format!(
                "Option 'i2c_speed' in section '{identifier}' must be at least {MIN_SPEED}"
            )));
        }
        let speed = speed as u32;

        let mcu_name = config
            .get_str("i2c_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&mcu_name))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
            })?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        let mode = match (
            config.get_str("i2c_software_scl_pin"),
            config.get_str("i2c_software_sda_pin"),
        ) {
            (Some(scl), Some(sda)) => {
                // Validate (and reserve) the pins now; the numbers are filled
                // in at build time, when the firmware dictionary exists.
                let scl_params = pins
                    .lookup_pin(&scl, false, false, Some("scl"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                let sda_params = pins
                    .lookup_pin(&sda, false, false, Some("sda"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                if scl_params.chip_name != mcu_name || sda_params.chip_name != mcu_name {
                    return Err(ConfigError::new(format!(
                        "Section '{identifier}': i2c pins must be on the same mcu '{mcu_name}'"
                    )));
                }
                I2cMode::Software {
                    scl_pin: scl_params.pin,
                    sda_pin: sda_params.pin,
                    speed,
                }
            }
            (None, None) => I2cMode::Hardware {
                bus: config.get_str("i2c_bus"),
                speed,
            },
            _ => {
                return Err(ConfigError::new(format!(
                    "Section '{identifier}': both 'i2c_software_scl_pin' and \
                     'i2c_software_sda_pin' must be set"
                )));
            }
        };

        let device = mcu_object.setup_i2c(mode, address);

        let scale =
            config.get_float_bounded("scale", Some(DEFAULT_SCALE), None, None, Some(0.0), None)?;
        // Required, and bounded by `scale`: upstream's
        // `getfloat('wiper', minval=0., maxval=self.scale)` (`mcp4018.py:13-14`).
        let start_value =
            config.get_float_bounded("wiper", None, Some(0.0), Some(scale), None, None)?;

        Ok(Self {
            name,
            device,
            scale,
            start_value,
            writes: Mutex::new(Vec::new()),
        })
    }
    /// Register `SET_DIGIPOT DIGIPOT=<name>` (`mcp4018.py:17-20`).
    ///
    /// The mux value is the **short** name (`config.get_name().split()[1]`), not
    /// the full section name; the handler upgrades a `Weak` back to the object
    /// so the dispatcher does not keep it (and, through it, the machine) alive.
    ///
    /// # Errors
    /// A duplicate mux value or command name.
    fn register_command(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let weak = Arc::downgrade(self);
        let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
            let object = weak.upgrade();
            Box::pin(async move {
                let object = object.ok_or_else(|| CommandError::new("The digipot is gone"))?;
                cmd_set_digipot(&object, gcmd).await
            })
        });
        gcode
            .register_mux_command(
                "SET_DIGIPOT",
                "DIGIPOT",
                Some(&self.name),
                handler,
                Some(CMD_SET_DIGIPOT_HELP),
            )
            .map_err(|err| ConfigError::new(format!("{}: {err}", self.name)))?;
        Ok(())
    }

    /// The section's name, as `SET_DIGIPOT` addresses it.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The taps handed to the bus so far, in order.
    pub fn writes(&self) -> Vec<u8> {
        self.writes
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// The events upstream's section subscribes to (`mcp4018.py:15-16`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::downgrade(self);
                move |_| {
                    if let Some(object) = object.upgrade() {
                        object.handle_connect();
                    }
                }
            }),
        );
    }

    /// Upstream's `handle_connect`: the start value goes out once the firmware
    /// is up.
    ///
    /// The event carries no runtime of its own, so the send is spawned onto the
    /// one driving the connect; without one the write cannot be queued and only
    /// a warning is logged, leaving the wiper where it was.
    fn handle_connect(self: &Arc<Self>) {
        let object = Arc::clone(self);
        let name = self.name.clone();
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(err) = object.set_dac(object.start_value).await {
                        warn!("mcp4018 '{name}': could not set the wiper at connect: {err}");
                    }
                });
            }
            Err(_) => {
                warn!("mcp4018 '{name}': setting the wiper at connect needs an async runtime")
            }
        }
    }

    /// Upstream's `set_dac` (`mcp4018.py:22-24`): one tap, one byte.
    ///
    /// # Errors
    /// The bus error [`McuI2c::write`] reports; a NACK stops the machine, as
    /// upstream's `i2c_write` does.
    async fn set_dac(&self, value: f64) -> Result<(), McuError> {
        let tap = tap_value(value, self.scale);
        self.writes
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push(tap);
        self.device.write(&[tap]).await
    }
}

/// One wiper's tap: `int(value * 127. / scale + .5)` (`mcp4018.py:22`), rounded
/// half up.
///
/// The value is bounded by `scale`, where the arithmetic gives 127 — so the byte
/// is already at its top.
fn tap_value(value: f64, scale: f64) -> u8 {
    (value * 127.0 / scale + 0.5) as u8
}

/// The `WIPER` parameter, or `None` when the command did not give one
/// (`mcp4018.py:28`).
///
/// Upstream reads it with no default, so an absent `WIPER` is not a value and
/// not an error: nothing is written and nothing is reported.
///
/// # Errors
/// A `WIPER` that is not a number, below 0, or above `scale`.
fn read_wiper(gcmd: &GcodeCommand, scale: f64) -> Result<Option<f64>, CommandError> {
    if !gcmd.get_command_parameters().contains_key("WIPER") {
        return Ok(None);
    }
    Ok(Some(gcmd.get_float_range("WIPER", 0.0, scale)?))
}

/// The line `SET_DIGIPOT` reports after a write (`mcp4018.py:30-31`).
fn wiper_response(name: &str, wiper: f64) -> String {
    format!("New value for DIGIPOT = {name}, wiper = {wiper:.2}")
}

/// `SET_DIGIPOT DIGIPOT=<name> [WIPER=<0..scale>]`: set the wiper (`mcp4018.py:26-31`).
async fn cmd_set_digipot(chip: &Arc<Mcp4018>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let Some(wiper) = read_wiper(gcmd, chip.scale)? else {
        return Ok(());
    };
    chip.set_dac(wiper)
        .await
        .map_err(|err| CommandError::new(err.to_string()))?;
    gcmd.respond_info(&wiper_response(&chip.name, wiper));
    Ok(())
}

impl PrinterObject for Mcp4018 {
    /// Upstream's `mcp4018` defines no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Mcp4018 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Mcp4018")
            .field("name", &self.name)
            .field("scale", &self.scale)
            .field("start_value", &self.start_value)
            .finish_non_exhaustive()
    }
}

/// The section factory: one `[mcp4018 <name>]` section.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(Mcp4018::new(config, printer)?);
    object.register_command(printer)?;
    object.register_handlers(printer);
    Ok(object)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgValue;
    use crate::core::klippy::reactor::ManualReactor;

    /// An `[mcp4018 x_axis_pot]` section's options, as the loader would hand
    /// them over.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("mcp4018", Some("x_axis_pot"));
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// A printer with `gcode`, `pins` and one MCU, as the loader builds them
    /// before any section runs.
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
        let object = Arc::new(
            McuObject::new(ConfigSection::new("mcu", None), &printer)
                .expect("the MCU registers its chip"),
        );
        printer
            .add_object("mcu", object as Arc<dyn PrinterObject>)
            .unwrap();
        // Ready only so a test can run a command through the dispatcher; the
        // connect handler under test is fired by hand.
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    /// One section, built and wired the way [`load_config_prefix`] does it.
    fn load(printer: &Arc<Printer>, section: &ConfigSection) -> Arc<Mcp4018> {
        let object = Arc::new(Mcp4018::new(&wrap(section), printer).expect("the section loads"));
        object
            .register_command(printer)
            .expect("the command registers");
        object.register_handlers(printer);
        object
    }

    /// Let the write `klippy:connect` spawns run (`ManualReactor` runs no
    /// tasks, so the test drives the runtime itself).
    async fn settle() {
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    /// An identified test MCU over the corpus dictionary, for build tests.
    fn test_mcu() -> Arc<Mcu> {
        let mcu = Arc::new(Mcu::for_test(
            "mcu",
            Interface::new(FrameMock::new(Vec::new())),
        ));
        let path = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let raw = std::fs::read(&path).expect("the corpus dictionary");
        let value: serde_json::Value = serde_json::from_slice(&raw).expect("valid JSON");
        mcu.install_dictionary(Dictionary::from_json(value).unwrap())
            .unwrap();
        mcu
    }

    /// Decode an encoded config command list into `(name, args)`.
    fn decoded(
        mcu: &Mcu,
        payloads: &[crate::core::klippy::Payload],
    ) -> Vec<(String, Vec<ArgValue>)> {
        let mut parser = Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        payloads
            .iter()
            .map(|payload| {
                let frame = crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).unwrap();
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    // -- the corpus bytes --------------------------------------------------

    /// `generic-mightyboard.cfg:91-95`: 0.50 A at a 0.773 A full scale is
    /// `int(0.50 * 127 / 0.773 + .5) = 82`, sent once at connect.
    #[tokio::test]
    async fn test_the_mightyboard_start_value_goes_out_at_connect() {
        let printer = printer();
        let section = section(&[
            ("i2c_software_scl_pin", "PJ5"),
            ("i2c_software_sda_pin", "PF3"),
            ("wiper", "0.50"),
            ("scale", "0.773"),
        ]);
        let chip = load(&printer, &section);

        printer.send_event(&KlippyEvent::KlippyConnect);
        settle().await;

        assert_eq!(chip.writes(), [82]);
    }

    /// `printer-flashforge-creator-pro-2018.cfg:129-133`: 118 of 127 is
    /// `int(118 * 127 / 127 + .5) = 118` — one byte, no register.
    #[tokio::test]
    async fn test_the_flashforge_start_value_goes_out_at_connect() {
        let printer = printer();
        let section = section(&[
            ("i2c_software_scl_pin", "PJ5"),
            ("i2c_software_sda_pin", "PF3"),
            ("wiper", "118"),
            ("scale", "127"),
        ]);
        let chip = load(&printer, &section);

        printer.send_event(&KlippyEvent::KlippyConnect);
        settle().await;

        assert_eq!(chip.writes(), [118]);
    }

    /// Before connect, nothing has been sent.
    #[test]
    fn test_nothing_is_written_before_connect() {
        let printer = printer();
        let chip = load(&printer, &section(&[("wiper", "0.5"), ("scale", "1")]));

        assert!(chip.writes().is_empty());
    }

    #[test]
    fn test_the_value_converts_to_a_tap_rounded_half_up() {
        // `int(value * 127. / scale + .5)` (`mcp4018.py:22`) — 128 taps.
        assert_eq!(tap_value(0.0, 1.0), 0); // 0.5 -> 0
        assert_eq!(tap_value(0.5, 1.0), 64); // 63.5 + .5 = 64.0
        assert_eq!(tap_value(0.5, 2.0), 32); // 31.75 + .5 = 32.25
        assert_eq!(tap_value(0.50, 0.773), 82); // 82.14 -> 82
        assert_eq!(tap_value(118.0, 127.0), 118); // 118.0 + .5 = 118.5
        assert_eq!(tap_value(40.0, 127.0), 40);
    }

    #[test]
    fn test_the_top_of_the_scale_is_the_top_tap() {
        // `wiper` is bounded by `scale`, where the byte is already at its top.
        assert_eq!(tap_value(1.0, 1.0), 127);
        assert_eq!(tap_value(0.773, 0.773), 127);
        assert_eq!(tap_value(127.0, 127.0), 127);
    }

    // -- options -----------------------------------------------------------

    #[tokio::test]
    async fn test_the_address_defaults_to_0x2f() {
        let printer = printer();
        load(&printer, &section(&[("wiper", "0")]));

        let mcu = test_mcu();
        let mcu_object = printer.lookup_object_as::<McuObject>("mcu").unwrap();
        let built = mcu_object.config().build(&mcu).expect("the config builds");
        let commands = decoded(&mcu, &built.config);
        // `i2c_set_bus oid=%c i2c_bus=%u rate=%u address=%u`.
        let set_bus = commands
            .iter()
            .find(|(name, _)| name == "i2c_set_bus")
            .expect("the bus is configured");
        assert_eq!(set_bus.1[3], ArgValue::UInt32(u32::from(DEFAULT_ADDRESS)));
    }

    #[tokio::test]
    async fn test_an_explicit_address_overrides_the_default() {
        let printer = printer();
        load(&printer, &section(&[("i2c_address", "44"), ("wiper", "0")]));

        let mcu = test_mcu();
        let mcu_object = printer.lookup_object_as::<McuObject>("mcu").unwrap();
        let built = mcu_object.config().build(&mcu).expect("the config builds");
        let commands = decoded(&mcu, &built.config);
        let set_bus = commands
            .iter()
            .find(|(name, _)| name == "i2c_set_bus")
            .expect("the bus is configured");
        assert_eq!(set_bus.1[3], ArgValue::UInt32(44));
    }

    #[test]
    fn test_an_address_above_the_bus_range_is_refused() {
        let err = Mcp4018::new(
            &wrap(&section(&[("i2c_address", "128"), ("wiper", "0")])),
            &printer(),
        )
        .map(|_| ())
        .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'mcp4018 x_axis_pot' must be between 0 and 127"
        );
    }

    #[test]
    fn test_scale_defaults_to_one_and_must_be_above_zero() {
        // `getfloat('scale', 1., above=0.)` (`mcp4018.py:12`): with no `scale`,
        // the default 1 is the ceiling `wiper` is measured against.
        let too_high = section(&[("wiper", "1.5")]);
        let err = Mcp4018::new(&wrap(&too_high), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'wiper' in section 'mcp4018 x_axis_pot' must have maximum of 1"
        );

        let zero_scale = section(&[("scale", "0"), ("wiper", "0")]);
        let err = Mcp4018::new(&wrap(&zero_scale), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'scale' in section 'mcp4018 x_axis_pot' must be above 0"
        );
    }

    #[test]
    fn test_the_start_value_is_required() {
        // `getfloat('wiper', minval=0., maxval=scale)` has no default
        // (`mcp4018.py:13-14`).
        let err = Mcp4018::new(&wrap(&section(&[("scale", "0.773")])), &printer())
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'wiper' in section 'mcp4018 x_axis_pot' must be specified"
        );
    }

    #[test]
    fn test_the_start_value_is_bounded_the_way_upstream_bounds_it() {
        let low = section(&[("scale", "1"), ("wiper", "-1")]);
        let err = Mcp4018::new(&wrap(&low), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'wiper' in section 'mcp4018 x_axis_pot' must have minimum of 0"
        );

        let high = section(&[("scale", "0.773"), ("wiper", "0.8")]);
        let err = Mcp4018::new(&wrap(&high), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'wiper' in section 'mcp4018 x_axis_pot' must have maximum of 0.773"
        );
    }

    // -- SET_DIGIPOT -------------------------------------------------------

    #[test]
    fn test_set_digipot_help_is_upstreams_help_text() {
        let printer = printer();
        let _chip = load(&printer, &section(&[("wiper", "0.5")]));

        assert_eq!(
            gcode(&printer).command_help()["SET_DIGIPOT"],
            CMD_SET_DIGIPOT_HELP
        );
    }

    #[test]
    fn test_set_digipot_routes_by_the_sections_short_name() {
        let printer = printer();
        let _chip = load(&printer, &section(&[("wiper", "0.5")]));

        let err = gcode(&printer)
            .run_script_sync("SET_DIGIPOT DIGIPOT=unknown POT=1")
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "The value 'unknown' is not valid for DIGIPOT. Options: 'x_axis_pot'"
        );
    }

    #[test]
    fn test_set_digipot_without_a_wiper_writes_nothing() {
        let printer = printer();
        let chip = load(&printer, &section(&[("wiper", "0.5")]));

        // Upstream's `get_float` returns `None` for an absent parameter, and
        // the command returns without writing or reporting.
        gcode(&printer)
            .run_script_sync("SET_DIGIPOT DIGIPOT=x_axis_pot")
            .unwrap();

        assert!(chip.writes().is_empty());
    }

    #[test]
    fn test_set_digipot_bounds_the_wiper_by_the_sections_scale() {
        let printer = printer();
        let _chip = load(&printer, &section(&[("wiper", "0.5")]));

        let err = gcode(&printer)
            .run_script_sync("SET_DIGIPOT DIGIPOT=x_axis_pot WIPER=1.5")
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Error on 'SET_DIGIPOT DIGIPOT=x_axis_pot WIPER=1.5': WIPER must have maximum of 1"
        );

        let err = gcode(&printer)
            .run_script_sync("SET_DIGIPOT DIGIPOT=x_axis_pot WIPER=-0.1")
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Error on 'SET_DIGIPOT DIGIPOT=x_axis_pot WIPER=-0.1': WIPER must have minimum of 0"
        );
    }

    /// A hit goes through the handler and hands the tap to the bus. The send
    /// itself then fails: a unit test's printer has no connected firmware, so
    /// the machine's own error comes back instead of the client's line. The tap
    /// is recorded before the send, which is what this asserts.
    #[test]
    fn test_set_digipot_writes_the_tap_it_computed() {
        let printer = printer();
        let chip = load(&printer, &section(&[("scale", "0.773"), ("wiper", "0")]));

        let result = gcode(&printer).run_script_sync("SET_DIGIPOT DIGIPOT=x_axis_pot WIPER=0.50");

        assert!(result.is_err(), "no firmware is connected in a unit test");
        assert_eq!(chip.writes(), [82]);
    }

    #[test]
    fn test_the_reply_names_the_section_and_the_value() {
        // `"New value for DIGIPOT = %s, wiper = %.2f"` (`mcp4018.py:30-31`):
        // two decimals.
        assert_eq!(
            wiper_response("x_axis_pot", 0.50),
            "New value for DIGIPOT = x_axis_pot, wiper = 0.50"
        );
        assert_eq!(
            wiper_response("y_axis_pot", 0.773),
            "New value for DIGIPOT = y_axis_pot, wiper = 0.77"
        );
        assert_eq!(
            wiper_response("z_axis_pot", 40.0),
            "New value for DIGIPOT = z_axis_pot, wiper = 40.00"
        );
    }
}
