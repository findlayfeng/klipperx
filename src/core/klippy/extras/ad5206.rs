//! `[ad5206 <name>]` — the AD5206 six-channel digital potentiometer (upstream's
//! `klippy/extras/ad5206.py`).
//!
//! A board with one of these uses it as a set of stepper current "digipots":
//! each of the six channels is a tap between 0 and 255, and the config states
//! the value in amps, scaled by `scale` so the number matches the driver's
//! current sense (`scale: 2.08` gives 2.08 A full scale).
//!
//! | option | meaning |
//! |---|---|
//! | `enable_pin` | the SPI chip select (upstream's `pin_option="enable_pin"`) |
//! | `scale` | full-scale value, above 0, default 1 |
//! | `channel_1`..`channel_6` | the value for one channel, `0..=scale`; absent channels are left alone |
//!
//! The registers are written once at startup: channel `n` goes to register
//! `n - 1` as the two bytes `[register, int(value * 256 / scale + .5)]`
//! (`ad5206.py:15-20`). Upstream queues exactly these bytes as init config
//! commands (`bus.py:105-110`); here they are sent from a post-init callback
//! instead, because at config-load time the SPI device has no oid yet
//! ([`McuSpi::send`] would report "not configured").
//!
//! # What is not here
//!
//! `SET_DIGIPOT`-style runtime commands: upstream's `ad5206` only sets the
//! channels at load and exposes no G-Code surface (the runtime setter lives in
//! `[mcp4018]`, which this section is not).

use std::sync::Arc;

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuObject, McuSpi};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Upstream builds the bus from `[board_pins]` aliases, so it loads alongside
// the other order-40 SPI devices.
section!("ad5206", order = 40, prefix = load_config_prefix);

/// Upstream's `MCU_SPI_from_config(config, 0, default_speed=25000000)`
/// (`ad5206.py:12-13`): SPI mode 0 at 25 MHz.
const DEFAULT_SPI_MODE: u8 = 0;
const DEFAULT_SPI_SPEED: u32 = 25_000_000;

/// Upstream's `getfloat('scale', 1., above=0.)` default (`ad5206.py:14`).
const DEFAULT_SCALE: f64 = 1.0;

/// The AD5206 has six channels, `channel_1`..`channel_6` (`ad5206.py:16`).
const CHANNEL_COUNT: u8 = 6;

/// One configured `[ad5206 <name>]`.
pub struct Ad5206 {
    /// The SPI bus (`enable_pin` plus the bus), kept for diagnostics.
    spi: Arc<McuSpi>,
    /// The two-byte register writes this section asks for, in channel order.
    /// These are the bytes handed to the bus at startup; tests assert them
    /// instead of a real transfer.
    writes: Vec<[u8; 2]>,
}

impl Ad5206 {
    /// Read the options, then queue the channel writes for bring-up.
    ///
    /// Upstream's `ad5206.__init__` (`ad5206.py:11-20`): the bus first, then
    /// `scale`, then the channels, so the first bad option is reported in that
    /// order.
    ///
    /// # Errors
    /// A missing `enable_pin` (unlike `[spi_device]`, this section always has a
    /// chip select), a bad SPI option or pin, a `scale` at or below 0, or a
    /// channel outside `0..=scale`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        // `enable_pin` is required: `MCU_SPI_from_config` reads it with a plain
        // `config.get`, while this host's port treats an absent pin option as
        // "no chip select". Read it here to keep upstream's error
        // (`ad5206.py:12-13`).
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
                            "MCU '{}': could not set an ad5206 channel: {err}",
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

    /// The register writes this section asks for, in channel order.
    pub fn writes(&self) -> &[[u8; 2]] {
        &self.writes
    }
}

/// The channel writes for one section, in channel order (`ad5206.py:15-19`):
/// channel `n` goes to register `n - 1`, and a channel the config does not
/// write is skipped.
fn channel_writes(config: &ConfigWrapper, scale: f64) -> Result<Vec<[u8; 2]>, ConfigError> {
    let mut writes = Vec::new();
    for channel in 1..=CHANNEL_COUNT {
        let option = format!("channel_{channel}");
        // Upstream reads every channel with `default=None`; an absent one is
        // not parsed, not bounds-checked, and not written.
        if !config.has(&option) {
            continue;
        }
        let value = config.get_float_bounded(&option, None, Some(0.0), Some(scale), None, None)?;
        writes.push([channel - 1, register_value(value, scale)]);
    }
    Ok(writes)
}

/// One channel's tap: `int(value * 256. / scale + .5)` (`ad5206.py:18`),
/// rounded half up.
///
/// The value is bounded by `scale`, where the arithmetic gives 256 — one past
/// the 8-bit register — so the byte saturates to 255 there.
fn register_value(value: f64, scale: f64) -> u8 {
    (value * 256.0 / scale + 0.5) as u8
}

impl PrinterObject for Ad5206 {
    /// Upstream's `ad5206` defines no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Ad5206 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ad5206")
            .field("writes", &self.writes)
            .finish_non_exhaustive()
    }
}

/// The section factory: one `[ad5206 <name>]` section.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Ad5206::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    /// An `[ad5206 stepper_digipot]` section's options, as the loader would hand
    /// them over.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("ad5206", Some("stepper_digipot"));
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

    /// The corpus's shape (`generic-rambo.cfg:91-99`).
    const RAMBO: &[(&str, &str)] = &[
        ("enable_pin", "PD7"),
        ("scale", "2.08"),
        ("channel_1", "1.34"),
        ("channel_2", "1.0"),
        ("channel_4", "1.1"),
    ];

    // -- options -----------------------------------------------------------

    #[test]
    fn test_the_section_writes_the_channels_it_names() {
        let chip = Ad5206::new(&wrap(&section(RAMBO)), &printer()).expect("the digipot loads");

        // channel 1 -> register 0, 2 -> 1, 4 -> 3; each value is
        // `int(val * 256 / 2.08 + .5)`.
        assert_eq!(chip.writes(), [[0, 165], [1, 123], [3, 135]]);
        // `enable_pin` became the bus's chip select.
        assert!(chip.spi().cs_pin().is_some());
    }

    #[test]
    fn test_the_enable_pin_is_required() {
        // Unlike `[spi_device]`, this section always has a chip select
        // (`ad5206.py:12`).
        let err = Ad5206::new(&wrap(&section(&[("scale", "1")])), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'enable_pin' in section 'ad5206 stepper_digipot' must be specified"
        );
    }

    #[test]
    fn test_scale_defaults_to_one_and_must_be_above_zero() {
        let chip = Ad5206::new(&wrap(&section(&[("enable_pin", "PD7")])), &printer())
            .expect("the digipot loads without scale");
        assert!(chip.writes().is_empty());

        // `getfloat('scale', 1., above=0.)` (`ad5206.py:14`).
        let section = section(&[("enable_pin", "PD7"), ("scale", "0"), ("channel_1", "0")]);
        let err = Ad5206::new(&wrap(&section), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'scale' in section 'ad5206 stepper_digipot' must be above 0"
        );
    }

    #[test]
    fn test_each_channel_is_bounded_the_way_upstream_bounds_it() {
        // `getfloat('channel_N', None, minval=0., maxval=scale)`
        // (`ad5206.py:16-17`).
        let low = section(&[("enable_pin", "PD7"), ("scale", "1"), ("channel_1", "-1")]);
        let err = Ad5206::new(&wrap(&low), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'channel_1' in section 'ad5206 stepper_digipot' must have minimum of 0"
        );

        let high = section(&[("enable_pin", "PD7"), ("scale", "1"), ("channel_1", "1.5")]);
        let err = Ad5206::new(&wrap(&high), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'channel_1' in section 'ad5206 stepper_digipot' must have maximum of 1"
        );
    }

    // -- the register conversion -------------------------------------------

    #[test]
    fn test_the_value_converts_to_a_tap_rounded_half_up() {
        // `int(value * 256. / scale + .5)` (`ad5206.py:18`).
        assert_eq!(register_value(0.0, 1.0), 0);
        assert_eq!(register_value(0.25, 1.0), 64); // 64.5 -> 64
        assert_eq!(register_value(0.5, 1.0), 128); // 128.5 -> 128
        assert_eq!(register_value(0.75, 1.0), 192); // 192.5 -> 192

        // The top of the range: the arithmetic gives 256, one past the 8-bit
        // register, so the byte saturates to 255.
        assert_eq!(register_value(1.0, 1.0), 255);
        // The corpus's 2.08 A full scale.
        assert_eq!(register_value(1.34, 2.08), 165);
    }

    #[test]
    fn test_a_channel_that_is_not_written_is_not_written() {
        let section = section(&[("enable_pin", "PD7"), ("scale", "1"), ("channel_3", "0.5")]);
        let chip = Ad5206::new(&wrap(&section), &printer()).expect("the digipot loads");
        assert_eq!(chip.writes(), [[2, 128]]);
    }

    #[test]
    fn test_the_writes_follow_channel_order_not_config_order() {
        // Options arrive sorted (`channel_1` before `channel_6`), and the
        // registers are written 0..5 in channel order.
        let section = section(&[
            ("enable_pin", "PD7"),
            ("scale", "1"),
            ("channel_1", "0.5"),
            ("channel_6", "0.1"),
        ]);
        let chip = Ad5206::new(&wrap(&section), &printer()).expect("the digipot loads");
        assert_eq!(chip.writes(), [[0, 128], [5, 26]]);
    }
}
