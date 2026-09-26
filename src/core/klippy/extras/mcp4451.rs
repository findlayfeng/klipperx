//! `[mcp4451 <name>]` — the MCP4451 quad digital potentiometer (upstream's
//! `klippy/extras/mcp4451.py`).
//!
//! Boards that carry one of these use it as a set of stepper current
//! "digipots": each wiper is a tap between 0 and 255, and the config states the
//! value in amps, scaled by `scale` so the number matches the driver's current
//! sense (`scale: 2.25` gives 2.25 A full scale).
//!
//! | option | meaning |
//! |---|---|
//! | `i2c_address` | the device address, required; this part answers only on 44..47 |
//! | `i2c_mcu` | the MCU to use (default `mcu`) |
//! | `i2c_speed` | clock in Hz (default 100000, minimum 100000) |
//! | `i2c_bus` / `i2c_software_scl_pin` / `i2c_software_sda_pin` | the hardware bus, or a bit-banged one |
//! | `scale` | full-scale value, above 0, default 1 |
//! | `wiper_0`..`wiper_3` | the value for one wiper, `0..=scale`; absent wipers are left alone |
//!
//! The registers are written once at startup. Unlike [ad5206](super::ad5206),
//! which queues two bytes per channel as firmware init commands, this section
//! sends the writes from a post-init callback: at config-load time the I2C
//! device has no oid yet, and the write itself is asynchronous. The callback
//! spawns one task that sends each two-byte register write in order.
//!
//! # What is not here
//!
//! `SET_DIGIPOT`-style runtime commands: upstream's `mcp4451` only sets the
//! wipers at load and exposes no G-Code surface (the runtime setter lives in
//! `[mcp4018]`, which this section is not).

use std::sync::Arc;

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{I2cMode, McuObject, DEFAULT_SPEED};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// The bus may name its pins through a `[board_pins]` alias, so this loads
// alongside the other order-40 I2C devices.
section!("mcp4451", order = 40, prefix = load_config_prefix);

/// Upstream's `getfloat('scale', 1., above=0.)` default (`mcp4451.py:16`).
const DEFAULT_SCALE: f64 = 1.0;

/// Upstream's `WiperRegisters` (`mcp4451.py:8`): wiper `i` goes to register
/// `WIPER_REGISTERS[i]`.
const WIPER_REGISTERS: [u8; 4] = [0x00, 0x01, 0x06, 0x07];

/// The wiper count, `wiper_0`..`wiper_3` (`mcp4451.py:20`).
const WIPER_COUNT: usize = 4;

/// This part answers only on addresses 44..47 (`mcp4451.py:13-15`).
const ADDRESS_RANGE: std::ops::RangeInclusive<u8> = 44..=47;

/// The lowest clock upstream accepts (its `minval` for `i2c_speed`).
const MIN_SPEED: u32 = 100_000;

/// One configured `[mcp4451 <name>]`.
pub struct Mcp4451 {
    /// The two-byte register writes this section asks for, in order: the two
    /// fixed writes, then each named wiper. These are the bytes handed to the
    /// bus at startup; tests assert them instead of a real transfer.
    writes: Vec<[u8; 2]>,
}

impl Mcp4451 {
    /// Read the options, then queue the register writes for bring-up.
    ///
    /// Upstream's `mcp4451.__init__` (`mcp4451.py:11-24`): the bus first, then
    /// the address check, then `scale`, then the wipers.
    ///
    /// # Errors
    /// A missing or out-of-range `i2c_address`, an address outside 44..47, a bad
    /// bus option or pin, a `scale` at or below 0, or a wiper outside
    /// `0..=scale`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();

        // `MCU_I2C_from_config` reads `i2c_address` with no default; this host
        // has no shared helper, so the option reading is the same as
        // `[i2c_device]`'s (`extras/i2c_device.rs`).
        let address = config.get_int("i2c_address", None)?;
        if !(0..=127).contains(&address) {
            return Err(ConfigError::new(format!(
                "Option 'i2c_address' in section '{identifier}' must be between 0 and 127"
            )));
        }
        let address = address as u8;
        // The part's own range, checked after the bus reads the address
        // (`mcp4451.py:13-15`); the message carries no section prefix.
        if !ADDRESS_RANGE.contains(&address) {
            return Err(ConfigError::new(format!(
                "mcp4451 address must be between {} and {}",
                ADDRESS_RANGE.start(),
                ADDRESS_RANGE.end()
            )));
        }

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
        let writes = register_writes(config, scale)?;

        // The writes need an oid (which only exists at build time) and an
        // asynchronous bus, so they run from a post-init callback, once the
        // firmware has accepted the configuration. The callback is synchronous,
        // so the send is spawned onto the runtime that is driving the connect.
        let post_device = Arc::clone(&device);
        let post_writes = writes.clone();
        mcu_object
            .config()
            .register_post_init_callback(Box::new(move |mcu| {
                let device = Arc::clone(&post_device);
                let writes = post_writes.clone();
                let mcu_name = mcu.name().to_string();
                match tokio::runtime::Handle::try_current() {
                    Ok(handle) => {
                        handle.spawn(async move {
                            for bytes in &writes {
                                if let Err(err) = device.write(bytes).await {
                                    warn!(
                                        "MCU '{mcu_name}': could not set an mcp4451 wiper: {err}"
                                    );
                                    return;
                                }
                            }
                        });
                    }
                    Err(_) => {
                        warn!("MCU '{mcu_name}': setting the mcp4451 wipers needs an async runtime")
                    }
                }
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        Ok(Self { writes })
    }

    /// The register writes this section asks for, in order.
    pub fn writes(&self) -> &[[u8; 2]] {
        &self.writes
    }
}

/// The register writes for one section (`mcp4451.py:17-24`): the two fixed
/// writes first, then each wiper the config names, in wiper order.
fn register_writes(config: &ConfigWrapper, scale: f64) -> Result<Vec<[u8; 2]>, ConfigError> {
    // `set_register(0x04, 0xff)` and `set_register(0x0a, 0xff)`, before any
    // wiper and regardless of what the config names (`mcp4451.py:17-19`).
    let mut writes = vec![register_bytes(0x04, 0xff), register_bytes(0x0a, 0xff)];
    for (index, register) in WIPER_REGISTERS.iter().enumerate().take(WIPER_COUNT) {
        let option = format!("wiper_{index}");
        // Upstream reads every wiper with `default=None`; an absent one is not
        // parsed, not bounds-checked, and not written.
        if !config.has(&option) {
            continue;
        }
        let value = config.get_float_bounded(&option, None, Some(0.0), Some(scale), None, None)?;
        writes.push(register_bytes(*register, wiper_value(value, scale)));
    }
    Ok(writes)
}

/// One wiper's tap: `int(value * 255. / scale + .5)` (`mcp4451.py:24`), rounded
/// half up.
///
/// The value is bounded by `scale`, where the arithmetic gives 255 — so the
/// byte is already at its top.
fn wiper_value(value: f64, scale: f64) -> u8 {
    (value * 255.0 / scale + 0.5) as u8
}

/// Upstream's `set_register(reg, value)` (`mcp4451.py:26-27`): the register in
/// the high nibble, the value in the low byte.
///
/// The high byte is `(value >> 8) & 0x03`, which is always 0 here because a
/// wiper value is at most 255.
fn register_bytes(register: u8, value: u8) -> [u8; 2] {
    [register << 4, value]
}

impl PrinterObject for Mcp4451 {
    /// Upstream's `mcp4451` defines no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Mcp4451 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Mcp4451")
            .field("writes", &self.writes)
            .finish_non_exhaustive()
    }
}

/// The section factory: one `[mcp4451 <name>]` section.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Mcp4451::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::reactor::ManualReactor;

    /// An `[mcp4451 stepper_digipot1]` section's options, as the loader would
    /// hand them over.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("mcp4451", Some("stepper_digipot1"));
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `pins` and one MCU, as the loader builds them before any
    /// section runs.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
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
        printer
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    // -- the corpus bytes --------------------------------------------------

    /// `generic-smoothieboard.cfg:91-105`, whose four wipers are 1.0 A at a
    /// 2.25 A full scale: `int(1.0 * 255 / 2.25 + .5) = 113`. The two leading
    /// 0xff writes are unconditional (`mcp4451.py:17-19`).
    #[test]
    fn test_the_smoothieboard_wipers_write_the_corpus_bytes() {
        let section = section(&[
            ("i2c_address", "44"),
            ("scale", "2.25"),
            ("wiper_0", "1.0"),
            ("wiper_1", "1.0"),
            ("wiper_2", "1.0"),
            ("wiper_3", "1.0"),
        ]);
        let chip = Mcp4451::new(&wrap(&section), &printer()).expect("the digipot loads");

        assert_eq!(
            chip.writes(),
            [
                [0x40, 0xff],
                [0xa0, 0xff],
                [0x00, 113],
                [0x10, 113],
                [0x60, 113],
                [0x70, 113],
            ]
        );
    }

    /// `generic-azteeg-x5-mini-v3.cfg:78-87`, whose four wipers are 1.0 A at a
    /// 2.0 A full scale: `int(1.0 * 255 / 2 + .5) = 128`.
    #[test]
    fn test_the_azteeg_wipers_write_the_corpus_bytes() {
        let section = section(&[
            ("i2c_address", "44"),
            ("scale", "2"),
            ("wiper_0", "1.0"),
            ("wiper_1", "1.0"),
            ("wiper_2", "1.0"),
            ("wiper_3", "1.0"),
        ]);
        let chip = Mcp4451::new(&wrap(&section), &printer()).expect("the digipot loads");

        assert_eq!(
            chip.writes(),
            [
                [0x40, 0xff],
                [0xa0, 0xff],
                [0x00, 128],
                [0x10, 128],
                [0x60, 128],
                [0x70, 128],
            ]
        );
    }

    #[test]
    fn test_a_section_with_no_wipers_still_writes_the_fixed_registers() {
        // `stepper_digipot2` (`generic-smoothieboard.cfg:99-105`) names only
        // `wiper_0`; the two 0xff writes are unconditional.
        let section = section(&[("i2c_address", "45"), ("scale", "2.25"), ("wiper_0", "1.0")]);
        let chip = Mcp4451::new(&wrap(&section), &printer()).expect("the digipot loads");

        assert_eq!(chip.writes(), [[0x40, 0xff], [0xa0, 0xff], [0x00, 113]]);
    }

    #[test]
    fn test_a_wiper_that_is_not_written_is_not_written() {
        let section = section(&[("i2c_address", "44"), ("scale", "1"), ("wiper_2", "0.5")]);
        let chip = Mcp4451::new(&wrap(&section), &printer()).expect("the digipot loads");

        // `wiper_2` -> `WiperRegisters[2]` = 0x06; `int(0.5 * 255 + .5)` = 128.
        assert_eq!(chip.writes(), [[0x40, 0xff], [0xa0, 0xff], [0x60, 128]]);
    }

    #[test]
    fn test_the_value_converts_to_a_tap_rounded_half_up() {
        // `int(value * 255. / scale + .5)` (`mcp4451.py:24`) — unlike
        // `[dac084S085]`'s truncation.
        assert_eq!(wiper_value(0.0, 1.0), 0);
        assert_eq!(wiper_value(0.5, 1.0), 128); // 128.0
        assert_eq!(wiper_value(0.5, 2.0), 64); // 64.25 -> 64
        assert_eq!(wiper_value(1.0, 2.25), 113); // 113.83 -> 113
        assert_eq!(wiper_value(1.0, 2.0), 128); // 128.0
                                                // The value is bounded by `scale`, where the byte is already at its top.
        assert_eq!(wiper_value(1.0, 1.0), 255);
    }

    #[test]
    fn test_the_register_byte_packs_the_register_in_the_high_nibble() {
        // `set_register` (`mcp4451.py:26-27`); the high byte is 0 for these
        // values.
        assert_eq!(register_bytes(0x00, 113), [0x00, 113]);
        assert_eq!(register_bytes(0x01, 113), [0x10, 113]);
        assert_eq!(register_bytes(0x06, 255), [0x60, 255]);
        assert_eq!(register_bytes(0x07, 0), [0x70, 0]);
        assert_eq!(register_bytes(0x04, 0xff), [0x40, 0xff]);
        assert_eq!(register_bytes(0x0a, 0xff), [0xa0, 0xff]);
    }

    // -- options -----------------------------------------------------------

    #[test]
    fn test_the_address_is_required() {
        let err = Mcp4451::new(&wrap(&section(&[("scale", "1")])), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'mcp4451 stepper_digipot1' must be specified"
        );
    }

    #[test]
    fn test_an_address_outside_this_parts_range_is_refused() {
        // `MCU_I2C_from_config` accepts 0..127, but this part answers only on
        // 44..47 (`mcp4451.py:13-15`); the message has no section prefix.
        let err = Mcp4451::new(&wrap(&section(&[("i2c_address", "43")])), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(err.to_string(), "mcp4451 address must be between 44 and 47");
    }

    #[test]
    fn test_an_address_above_the_bus_range_is_refused() {
        let err = Mcp4451::new(&wrap(&section(&[("i2c_address", "128")])), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'mcp4451 stepper_digipot1' must be between 0 and 127"
        );
    }

    #[test]
    fn test_scale_defaults_to_one_and_must_be_above_zero() {
        let chip = Mcp4451::new(
            &wrap(&section(&[("i2c_address", "44"), ("wiper_0", "0.5")])),
            &printer(),
        )
        .expect("the digipot loads without scale");
        assert_eq!(chip.writes()[2], [0x00, 128]);

        let section = section(&[("i2c_address", "44"), ("scale", "0"), ("wiper_0", "0")]);
        let err = Mcp4451::new(&wrap(&section), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'scale' in section 'mcp4451 stepper_digipot1' must be above 0"
        );
    }

    #[test]
    fn test_each_wiper_is_bounded_the_way_upstream_bounds_it() {
        // `getfloat('wiper_N', None, minval=0., maxval=scale)`
        // (`mcp4451.py:20-22`).
        let low = section(&[("i2c_address", "44"), ("scale", "1"), ("wiper_1", "-1")]);
        let err = Mcp4451::new(&wrap(&low), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'wiper_1' in section 'mcp4451 stepper_digipot1' must have minimum of 0"
        );

        let high = section(&[("i2c_address", "44"), ("scale", "1"), ("wiper_1", "1.5")]);
        let err = Mcp4451::new(&wrap(&high), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'wiper_1' in section 'mcp4451 stepper_digipot1' must have maximum of 1"
        );
    }
}
