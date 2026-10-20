//! `[static_pwm_clock <name>]` — a pin driven as a fixed-frequency 50% clock.
//!
//! Upstream is `klippy/extras/static_pwm_clock.py` (41 lines):
//!
//! ```python
//! self.mcu_pin = ppins.setup_pin('pwm', config.get('pin'))
//! self.mcu = self.mcu_pin.get_mcu()
//! self.frequency = config.getfloat('frequency', 100, above=(1/0.3),
//!                                  maxval=520000000)
//! self.mcu_pin.setup_cycle_time(1. / self.frequency, True)
//! self.mcu_pin.setup_max_duration(0.)
//! self.mcu_pin.setup_start_value(0.5, 0.5)
//! self.mcu.register_config_callback(self._build_config)
//! ```
//!
//! It is a **section**, not a pin chip: it consumes one pin, builds a hardware
//! PWM on it (`hardware_pwm=True`), leaves it running at 50% duty and never
//! talks to it again — no `get_status`, no gcode command, so `objects/list`
//! leaves it out.
//!
//! The section is what gives a device's clock input a driver: `[load_cell]
//! pwm_clock` only *reads* the `frequency` of a `[static_pwm_clock]` section
//! (`extras/ads131m0x`), so without this section the frequency had an owner but
//! nothing drove the pin.
//!
//! # The remainder check
//!
//! The firmware counts the period in whole clock ticks (`config_pwm_out
//! cycle_ticks=%u`), and the resource derives them with
//! [`Mcu::seconds_to_clock`](crate::core::klippy::mcu::Mcu::seconds_to_clock),
//! which truncates. A frequency that does not divide the MCU clock would
//! therefore produce a silently wrong clock, so the build refuses it:
//! `mcu_freq // frequency`, multiplied back by `frequency`, must reproduce
//! `mcu_freq` exactly.
//!
//! # What differs from upstream, and why
//!
//! * **Reaching the MCU.** Upstream's `mcu_pin.get_mcu()` has no counterpart:
//!   [`PwmOut`] is a resource handle with no back-reference to its MCU.
//!   [`PrinterPins::parse_pin`] supplies the chip name (parse only — it
//!   reserves nothing), [`PrinterPins::chip_as`] fetches the concrete
//!   `McuChip`, and [`PrinterPins::setup_pwm`] then builds the resource. A pin
//!   whose chip is not an MCU (a `multi_pin` alias, …) is refused, where
//!   upstream would have followed `get_mcu` to the first sub-pin's MCU.
//! * **When the division is checked.** Upstream raises `printer.config_error`
//!   from its config callback; a [`ConfigCallback`] here can only report
//!   [`McuError::Config`], so a bad frequency fails the MCU configuration
//!   (connect time) rather than the config load. The message text is upstream's,
//!   indentation included.
//!
//! # The pulse-width warnings
//!
//! Upstream warns when `int(0.5 * cycle_ticks) / cycle_ticks` leaves the
//! `0.4..=0.6` band. With the duty fixed at 50% that can only happen for the
//! smallest periods (`cycle_ticks` of 1 or 3), which is exactly when the
//! rounding matters; the check is kept verbatim.
//!
//! [`ConfigCallback`]: crate::core::klippy::mcu::ConfigCallback

use std::sync::Arc;

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuChip, McuError};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[static_pwm_clock <name>]`) exists upstream. Order 35
// puts it after `[board_pins]` (30), so an alias may name the pin.
section!("static_pwm_clock", order = 35, prefix = load_config_prefix);

/// Default `frequency` in Hz (upstream's `getfloat('frequency', 100, …)`).
const DEFAULT_FREQUENCY: f64 = 100.0;

/// Upstream's `above=(1/0.3)`: the frequency must be **above** this (Hz).
const MINIMUM_FREQUENCY: f64 = 1.0 / 0.3;

/// Upstream's `maxval=520000000` (Hz).
const MAXIMUM_FREQUENCY: f64 = 520_000_000.0;

/// One configured `[static_pwm_clock <name>]`.
pub struct StaticPwmClock {
    /// The section's identifier, for logging and the build-time messages.
    identifier: String,
    /// The clock frequency in Hz.
    frequency: f64,
}

impl StaticPwmClock {
    /// Build the hardware PWM and register the build-time checks.
    ///
    /// # Errors
    /// Returns a config error when `pin` is missing, `frequency` is out of
    /// bounds, the pin cannot be built, or it does not name a pin on an MCU.
    pub fn new(config: &ConfigWrapper, printer: &Printer) -> Result<Self, ConfigError> {
        let identifier = config.identifier();

        // Upstream's read order (`static_pwm_clock.py:13-19`): `pin`, then the
        // resource, then `frequency`.
        let pin_desc = config.get("pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        // The same decorations `setup_pwm` accepts (`PinType::Pwm`): invert yes,
        // pull-up no.
        let params = pins
            .parse_pin(&pin_desc, true, false)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        let pwm = pins
            .setup_pwm(&pin_desc, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let frequency = config.get_float_bounded(
            "frequency",
            Some(DEFAULT_FREQUENCY),
            None,
            Some(MAXIMUM_FREQUENCY),
            Some(MINIMUM_FREQUENCY),
            None,
        )?;

        // Upstream's three setup calls (`static_pwm_clock.py:21-23`): a hardware
        // PWM (the firmware toggles the pin, not the host), no
        // return-to-shutdown limit, and 50% duty both at start and on shutdown.
        pwm.setup_cycle_time(1.0 / frequency, true);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.5, 0.5);

        // Upstream's `self.mcu = self.mcu_pin.get_mcu()`: the chip the pin
        // belongs to has to be an MCU now, because the build callback below
        // needs its clock frequency.
        let chip = pins.chip_as::<McuChip>(&params.chip_name).ok_or_else(|| {
            ConfigError::new(format!("[{identifier}] pin '{pin_desc}' is not on an MCU"))
        })?;

        let build_identifier = identifier.clone();
        chip.config()
            .register_config_callback(Box::new(move |_builder, mcu| {
                let mcu_freq = mcu.seconds_to_clock(1.0)?;
                let cycle_ticks = cycle_ticks_for(mcu_freq, frequency, &build_identifier)
                    .map_err(McuError::Config)?;
                // `value == int(0.5 * cycle_ticks)` by construction, so this
                // only trips for the smallest periods (see the module docs).
                let value = (0.5 * cycle_ticks as f64) as u64;
                let ratio = value as f64 / cycle_ticks as f64;
                if ratio < 0.4 {
                    warn!("[{build_identifier}] pulse width < 40%");
                }
                if ratio > 0.6 {
                    warn!("[{build_identifier}] pulse width > 60%");
                }
                Ok(())
            }))
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self {
            identifier,
            frequency,
        })
    }

    /// The section identifier.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }

    /// The clock frequency in Hz.
    pub fn frequency(&self) -> f64 {
        self.frequency
    }
}

impl PrinterObject for StaticPwmClock {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for StaticPwmClock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StaticPwmClock")
            .field("identifier", &self.identifier)
            .field("frequency", &self.frequency)
            .finish()
    }
}

/// Upstream's `_build_config` division check (`static_pwm_clock.py:27-36`).
///
/// Returns the whole number of clock ticks one period spans, or upstream's
/// message when `frequency` does not divide `mcu_freq` exactly. The two `as
/// u64` casts are upstream's two `int()` calls: the comparison is between
/// integers, so a fractional tick is what makes the check fail.
fn cycle_ticks_for(mcu_freq: u64, frequency: f64, identifier: &str) -> Result<u64, String> {
    let cycle_ticks = (mcu_freq as f64 / frequency) as u64;
    let mcu_freq_rev = (cycle_ticks as f64 * frequency) as u64;
    if mcu_freq_rev != mcu_freq {
        // Upstream's triple-quoted message, indentation and all: the leading
        // newline and the twelve spaces are what a Klipper user sees.
        return Err(format!(
            "\n            # Frequency output must be without remainder, {mcu_freq} != {mcu_freq_rev}\n            [{identifier}]\n            frequency = {frequency:.6}\n            "
        ));
    }
    Ok(cycle_ticks)
}

/// Upstream's `load_config_prefix` for `[static_pwm_clock <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(StaticPwmClock::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgValue;
    use crate::core::klippy::pins::{DigitalOut, PinChip, PinError, PinParams, PwmOut};
    use crate::core::klippy::reactor::ManualReactor;

    /// The clock of the test MCU; `128 Hz` divides it exactly, so `1/frequency`
    /// is a power of two and the period is 562500 ticks with no rounding.
    const CLOCK_FREQ: u64 = 72_000_000;

    /// A dictionary with the pin enumeration and the hardware PWM commands.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu max_duration=%u": 20,
                "queue_pwm_out oid=%c clock=%u value=%hu": 21
            },
            "enumerations": {"pin": {"PA0": 0, "PA1": 1}},
            "config": {"CLOCK_FREQ": CLOCK_FREQ, "PWM_MAX": 4095}
        }))
        .expect("the test dictionary parses")
    }

    fn mcu() -> Arc<Mcu> {
        let mcu = Arc::new(Mcu::for_test(
            "static_pwm_clock_mcu",
            Interface::new(FrameMock::new(Vec::new())),
        ));
        mcu.install_dictionary(dictionary()).expect("it installs");
        mcu
    }

    /// A machine whose `mcu` chip can build a hardware PWM.
    fn machine() -> (Arc<Printer>, McuChip) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        pins.register_chip("mcu", Arc::new(chip.clone()))
            .expect("the name is free");
        printer.add_object(PINS_OBJECT, pins).unwrap();
        (printer, chip)
    }

    /// An `[static_pwm_clock <name>]` section with the given options.
    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("static_pwm_clock", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// The decoded `(name, args)` of every config command of one build.
    fn config_commands(chip: &McuChip, mcu: &Mcu) -> Vec<(String, Vec<ArgValue>)> {
        let built = chip.config().build(mcu).expect("the configuration builds");
        let mut parser = Parser::new();
        mcu.dictionary()
            .unwrap()
            .install(&mut parser)
            .expect("the dictionary installs");
        built
            .config
            .iter()
            .map(|payload| {
                let frame = Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).expect("it decodes");
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    #[tokio::test]
    async fn test_a_clock_builds_a_hardware_pwm_at_half_duty() {
        let (printer, chip) = machine();
        let section = section("fast", &[("pin", "PA1"), ("frequency", "128")]);

        let object = StaticPwmClock::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        assert_eq!(object.frequency(), 128.0);
        let commands = config_commands(&chip, &mcu());
        assert_eq!(
            commands
                .iter()
                .map(|(name, _)| name.as_str())
                .collect::<Vec<_>>(),
            ["allocate_oids", "config_pwm_out", "finalize_config"]
        );
        assert_eq!(
            commands[1].1,
            [
                ArgValue::UInt8(0),
                ArgValue::UInt32(1),       // PA1
                ArgValue::UInt32(562_500), // 1/128 s at 72 MHz
                ArgValue::UInt16(2048),    // 0.5 * 4095, rounded
                ArgValue::UInt16(2048),    // shutdown is 50% too
                ArgValue::UInt32(0),       // max_duration
            ]
        );
    }

    #[test]
    fn test_the_frequency_defaults_to_100() {
        let (printer, _chip) = machine();
        let section = section("slow", &[("pin", "PA0")]);

        let object = StaticPwmClock::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        assert_eq!(object.frequency(), 100.0);
    }

    #[test]
    fn test_the_frequency_bounds_are_upstreams() {
        // One machine per case: a pin may only be reserved once.
        let (printer, _chip) = machine();
        let low = section("low", &[("pin", "PA0"), ("frequency", "3.3")]);
        let err = StaticPwmClock::new(&ConfigWrapper::untracked(&low), &printer).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'frequency' in section 'static_pwm_clock low' must be above 3.3333333333333335"
        );

        let (printer, _chip) = machine();
        let high = section("high", &[("pin", "PA0"), ("frequency", "520000001")]);
        let err = StaticPwmClock::new(&ConfigWrapper::untracked(&high), &printer).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'frequency' in section 'static_pwm_clock high' must have maximum of 520000000"
        );

        // Just inside the bound is accepted (and then only the build can
        // complain about the remainder).
        let (printer, _chip) = machine();
        let edge = section("edge", &[("pin", "PA0"), ("frequency", "520000000")]);
        assert!(StaticPwmClock::new(&ConfigWrapper::untracked(&edge), &printer).is_ok());
    }

    #[test]
    fn test_a_missing_pin_option_is_reported() {
        let (printer, _chip) = machine();
        let section = section("clk", &[("frequency", "128")]);

        let err = StaticPwmClock::new(&ConfigWrapper::untracked(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'static_pwm_clock clk' must be specified"
        );
    }

    /// A PWM resource for the non-MCU chip below: the section configures it but
    /// must not reach a build.
    struct VirtualPwm;

    impl PwmOut for VirtualPwm {
        fn setup_max_duration(&self, _max_duration: f64) {}

        fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: bool) {}

        fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}

        fn set_pwm(&self, _clock: u32, _value: f64) -> Result<(), McuError> {
            unreachable!("a static clock never changes its duty")
        }

        fn update_pwm(&self, _value: f64) -> Result<(), McuError> {
            unreachable!("a static clock never changes its duty")
        }

        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    /// A chip that builds PWMs but is not an MCU (a `multi_pin` in miniature).
    struct VirtualChip;

    impl PinChip for VirtualChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            Ok(Arc::new(VirtualPwm))
        }
    }

    #[test]
    fn test_a_pin_that_is_not_on_an_mcu_is_refused() {
        let (printer, _chip) = machine();
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .unwrap();
        pins.register_chip("virtual", Arc::new(VirtualChip))
            .expect("the name is free");
        let section = section("clk", &[("pin", "virtual:PA0")]);

        let err = StaticPwmClock::new(&ConfigWrapper::untracked(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "[static_pwm_clock clk] pin 'virtual:PA0' is not on an MCU"
        );
    }

    #[test]
    fn test_an_exact_division_is_the_tick_count() {
        // 72 MHz over 128 Hz is 562500 ticks, no remainder.
        assert_eq!(
            cycle_ticks_for(CLOCK_FREQ, 128.0, "static_pwm_clock clk"),
            Ok(562_500)
        );
    }

    #[test]
    fn test_the_remainder_message_is_upstreams() {
        // 24 MHz does not divide 64 MHz: two whole ticks are 48 MHz, not 64.
        let err = cycle_ticks_for(64_000_000, 24_000_000.0, "static_pwm_clock clk").unwrap_err();

        assert_eq!(
            err,
            "\n            # Frequency output must be without remainder, 64000000 != 48000000\n            [static_pwm_clock clk]\n            frequency = 24000000.000000\n            "
        );
    }
}
