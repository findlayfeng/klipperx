//! `[dac084S085 <name>]` — the TI DAC084S085 four-channel SPI DAC (upstream's
//! `klippy/extras/dac084S085.py`).
//!
//! A board with one of these uses it as a set of stepper current references:
//! each of the four channels is an 8-bit value, and the config states the value
//! in amps, scaled by `scale` so the number matches the driver's sense resistor
//! (`scale: 2.50` gives 2.50 A full scale). The channels are A..D; A/B/C/D map
//! onto X/Y/Z/extruder on the boards that use it.
//!
//! | option | meaning |
//! |---|---|
//! | `enable_pin` | the SPI chip select (upstream's `pin_option="enable_pin"`), required |
//! | `scale` | full-scale value, above 0, default 1 |
//! | `channel_A`..`channel_D` | the value for one channel, `0..=scale`; absent channels are left alone |
//!
//! The registers are written once at startup: channel `n` (0 for A, 3 for D)
//! goes out as the two bytes `[(n << 6) | 0x10 | ((value >> 4) & 0x0f),
//! (value << 4) & 0xf0]`, where `value = int(val * 255. / scale)`
//! (`dac084S085.py:19-22`). Note the value is **truncated**, not rounded like
//! `[ad5206]`/`[mcp4451]` (`int()` with no `+ .5`). Upstream queues these bytes
//! as init config commands (`bus.py:105-110`); here they are sent from a
//! post-init callback instead, because at config-load time the SPI device has
//! no oid yet ([`McuSpi::send`] would report "not configured").
//!
//! The bus is SPI **mode 1** at 10 MHz (`dac084S085.py:11-12`): upstream's
//! `MCU_SPI_from_config(config, 1, pin_option="enable_pin",
//! default_speed=10000000)`.
//!
//! # What is not here
//!
//! Runtime commands and a `get_status`: upstream's `dac084S085` only sets the
//! channels at load and exposes no G-Code surface.

use std::sync::Arc;

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuObject, McuSpi};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Upstream builds the bus from `[board_pins]` aliases, so it loads alongside
// the other order-40 SPI devices. The section id keeps the chip's casing
// (`dac084S085`): the factory table matches section ids case-sensitively, while
// the module/file name stays lowercase.
section!("dac084S085", order = 40, prefix = load_config_prefix);

/// Upstream's `MCU_SPI_from_config(config, 1, ...)` (`dac084S085.py:11`): SPI
/// mode 1, fixed (upstream never reads a `spi_mode` option).
const DEFAULT_SPI_MODE: u8 = 1;
/// Upstream's `default_speed=10000000` (`dac084S085.py:12`).
const DEFAULT_SPI_SPEED: u32 = 10_000_000;

/// Upstream's `getfloat('scale', 1., above=0.)` default (`dac084S085.py:13`).
const DEFAULT_SCALE: f64 = 1.0;

/// The DAC084S085 has four channels, `channel_A`..`channel_D`
/// (`dac084S085.py:14-16`).
const CHANNEL_NAMES: [&str; 4] = ["A", "B", "C", "D"];

/// One configured `[dac084S085 <name>]`.
pub struct Dac084S085 {
    /// The SPI bus (`enable_pin` plus the bus), kept for diagnostics.
    spi: Arc<McuSpi>,
    /// The two-byte writes this section asks for, in channel order (A..D).
    /// These are the bytes handed to the bus at startup; tests assert them
    /// instead of a real transfer.
    writes: Vec<[u8; 2]>,
}

impl Dac084S085 {
    /// Read the options, then queue the channel writes for bring-up.
    ///
    /// Upstream's `dac084S085.__init__` (`dac084S085.py:10-16`): the bus first,
    /// then `scale`, then the channels, so the first bad option is reported in
    /// that order.
    ///
    /// # Errors
    /// A missing `enable_pin` (unlike `[spi_device]`, this section always has a
    /// chip select), a bad SPI option or pin, a `scale` at or below 0, or a
    /// channel outside `0..=scale`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        // `enable_pin` is required: `MCU_SPI_from_config` reads it with a plain
        // `config.get`, while this host's port treats an absent pin option as
        // "no chip select". Read it here to keep upstream's error
        // (`dac084S085.py:11-12`).
        config.get("enable_pin", None)?;
        let setup = mcu_spi_from_config(
            config,
            printer,
            DEFAULT_SPI_MODE,
            "enable_pin",
            DEFAULT_SPI_SPEED,
        )?;
        let spi = setup.device;

        let scale =
            config.get_float_bounded("scale", Some(DEFAULT_SCALE), None, None, Some(0.0), None)?;
        let writes = channel_writes(config, scale)?;

        // The MCU the bus belongs to, so the writes run once it is up.
        let mcu_name = config
            .get_str("spi_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&mcu_name))
            .ok_or_else(|| {
                ConfigError::new(format!(
                    "Section '{}': unknown MCU '{mcu_name}'",
                    config.identifier()
                ))
            })?;
        let post_spi = Arc::clone(&spi);
        let post_writes = writes.clone();
        mcu_object
            .config()
            .register_post_init_callback(Box::new(move |mcu| {
                for payload in &post_writes {
                    if let Err(err) = post_spi.send(payload) {
                        warn!(
                            "MCU '{}': could not set a dac084S085 channel: {err}",
                            mcu.name()
                        );
                        return;
                    }
                }
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        Ok(Self { spi, writes })
    }

    /// The bus, so a caller (a test, or a future runtime setter) can reach it.
    pub fn spi(&self) -> &Arc<McuSpi> {
        &self.spi
    }

    /// The register writes this section asks for, in channel order (A..D).
    pub fn writes(&self) -> &[[u8; 2]] {
        &self.writes
    }
}

/// The channel writes for one section, in channel order (`dac084S085.py:14-16`):
/// A..D, and a channel the config does not write is skipped.
fn channel_writes(config: &ConfigWrapper, scale: f64) -> Result<Vec<[u8; 2]>, ConfigError> {
    let mut writes = Vec::new();
    for (chan, name) in CHANNEL_NAMES.iter().enumerate() {
        let option = format!("channel_{name}");
        // Upstream reads every channel with `default=None`; an absent one is
        // not parsed, not bounds-checked, and not written.
        if !config.has(&option) {
            continue;
        }
        let value = config.get_float_bounded(&option, None, Some(0.0), Some(scale), None, None)?;
        writes.push(register_bytes(chan as u8, value, scale));
    }
    Ok(writes)
}

/// One channel's byte value: `int(value * 255. / scale)` (`dac084S085.py:19`),
/// **truncated** toward zero — unlike `[ad5206]`'s `int(... + .5)`.
///
/// The value is bounded by `scale`, where the arithmetic gives 255 — the top of
/// the 8-bit register.
fn channel_value(value: f64, scale: f64) -> u8 {
    (value * 255.0 / scale) as u8
}

/// The two bytes for one channel (`dac084S085.py:20-22`): `chan` is 0..=3 and
/// `[b1, b2]` is what goes on the wire.
fn register_bytes(chan: u8, value: f64, scale: f64) -> [u8; 2] {
    let value = channel_value(value, scale);
    let b1 = (chan << 6) | (1 << 4) | ((value >> 4) & 0x0f);
    let b2 = (value << 4) & 0xf0;
    [b1, b2]
}

impl PrinterObject for Dac084S085 {
    /// Upstream's `dac084S085` defines no `get_status`, so it is not
    /// client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Dac084S085 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Dac084S085")
            .field("writes", &self.writes)
            .finish_non_exhaustive()
    }
}

/// The section factory: one `[dac084S085 <name>]` section.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Dac084S085::new(config, printer)?))
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
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[dac084S085 stepper_digipot]` section's options, as the loader would
    /// hand them over.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("dac084S085", Some("stepper_digipot"));
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

    /// The corpus's shape (`generic-alligator-r2.cfg:19-29`, `r3` alike):
    /// `scale: 2.50`, A/B/C `1.5` A, D `1.0` A.
    const ALLIGATOR: &[(&str, &str)] = &[
        ("enable_pin", "PB14"),
        ("spi_bus", "spi0"),
        ("scale", "2.50"),
        ("channel_A", "1.5"),
        ("channel_B", "1.5"),
        ("channel_C", "1.5"),
        ("channel_D", "1.0"),
    ];

    // -- options -----------------------------------------------------------

    #[test]
    fn test_the_section_writes_the_channels_it_names() {
        let chip = Dac084S085::new(&wrap(&section(ALLIGATOR)), &printer()).expect("the DAC loads");

        // A..D -> channels 0..3; each value is `int(val * 255 / 2.50)`.
        assert_eq!(
            chip.writes(),
            [[0x19, 0x90], [0x59, 0x90], [0x99, 0x90], [0xd6, 0x60]]
        );
        // `enable_pin` became the bus's chip select.
        assert!(chip.spi().cs_pin().is_some());
    }

    #[test]
    fn test_the_enable_pin_is_required() {
        // Unlike `[spi_device]`, this section always has a chip select
        // (`dac084S085.py:11`).
        let err = Dac084S085::new(&wrap(&section(&[("scale", "1")])), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'enable_pin' in section 'dac084S085 stepper_digipot' must be specified"
        );
    }

    #[test]
    fn test_scale_defaults_to_one_and_must_be_above_zero() {
        let chip = Dac084S085::new(&wrap(&section(&[("enable_pin", "PB14")])), &printer())
            .expect("the DAC loads without scale");
        assert!(chip.writes().is_empty());

        // `getfloat('scale', 1., above=0.)` (`dac084S085.py:13`).
        let section = section(&[("enable_pin", "PB14"), ("scale", "0"), ("channel_A", "0")]);
        let err = Dac084S085::new(&wrap(&section), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'scale' in section 'dac084S085 stepper_digipot' must be above 0"
        );
    }

    #[test]
    fn test_each_channel_is_bounded_the_way_upstream_bounds_it() {
        // `getfloat('channel_%s', None, minval=0., maxval=scale)`
        // (`dac084S085.py:14-15`).
        let low = section(&[("enable_pin", "PB14"), ("scale", "1"), ("channel_A", "-1")]);
        let err = Dac084S085::new(&wrap(&low), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'channel_A' in section 'dac084S085 stepper_digipot' must have minimum of 0"
        );

        let high = section(&[("enable_pin", "PB14"), ("scale", "1"), ("channel_B", "1.5")]);
        let err = Dac084S085::new(&wrap(&high), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'channel_B' in section 'dac084S085 stepper_digipot' must have maximum of 1"
        );
    }

    // -- the register conversion -------------------------------------------

    #[test]
    fn test_the_value_is_truncated_not_rounded() {
        // `int(value * 255. / scale)` with no `+ .5` (`dac084S085.py:19`) —
        // unlike `[ad5206]`, which rounds half up.
        assert_eq!(channel_value(1.0, 1.0), 255);
        assert_eq!(channel_value(0.0, 1.0), 0);
        // 1.0 * 255 / 2.0 = 127.5: truncation gives 127, a round-half-up port
        // would give 128.
        assert_eq!(channel_value(1.0, 2.0), 127);
        // 1.0 * 255 / 3.0 = 85.0 exactly.
        assert_eq!(channel_value(1.0, 3.0), 85);
        // The corpus's 2.50 A full scale.
        assert_eq!(channel_value(1.5, 2.5), 153);
        assert_eq!(channel_value(1.0, 2.5), 102);
    }

    #[test]
    fn test_the_two_bytes_carry_the_channel_and_the_value() {
        // `dac084S085.py:20-22`: `(chan << 6) | 0x10 | (value >> 4)`,
        // `(value << 4) & 0xf0`.
        assert_eq!(register_bytes(0, 1.5, 2.5), [0x19, 0x90]);
        assert_eq!(register_bytes(1, 1.5, 2.5), [0x59, 0x90]);
        assert_eq!(register_bytes(2, 1.5, 2.5), [0x99, 0x90]);
        assert_eq!(register_bytes(3, 1.0, 2.5), [0xd6, 0x60]);
    }

    #[test]
    fn test_a_channel_that_is_not_written_is_not_written() {
        let section = section(&[("enable_pin", "PB14"), ("scale", "1"), ("channel_C", "0.5")]);
        let chip = Dac084S085::new(&wrap(&section), &printer()).expect("the DAC loads");
        // Only channel C (chan 2): int(0.5 * 255) = 127.
        assert_eq!(
            chip.writes(),
            [[0x90 | (127 >> 4) as u8, (127 << 4) & 0xf0]]
        );
    }

    // -- the bus defaults --------------------------------------------------

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

    // `Mcu::for_test` needs the test's Tokio runtime (the interface spawns).
    #[tokio::test]
    async fn test_the_bus_defaults_to_mode_one_at_ten_megahertz() {
        // `MCU_SPI_from_config(config, 1, pin_option="enable_pin",
        // default_speed=10000000)` (`dac084S085.py:11-12`): no `spi_mode` /
        // `spi_speed` in the section still yields mode 1 at 10 MHz.
        let printer = printer();
        Dac084S085::new(&wrap(&section(&[("enable_pin", "PB0")])), &printer)
            .expect("the DAC loads");

        let mcu = test_mcu();
        let mcu_object = printer.lookup_object_as::<McuObject>("mcu").unwrap();
        let built = mcu_object.config().build(&mcu).expect("the config builds");
        let commands = decoded(&mcu, &built.config);
        // `spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u`.
        let set_bus = commands
            .iter()
            .find(|(name, _)| name == "spi_set_bus")
            .expect("the bus is configured");
        assert_eq!(set_bus.1[2], ArgValue::UInt32(DEFAULT_SPI_MODE as u32));
        assert_eq!(set_bus.1[3], ArgValue::UInt32(DEFAULT_SPI_SPEED));
    }
}
