//! The MCU as a pin chip: `McuChip`, and the resources it builds.
//!
//! [`McuChip`] is what the `pins` object dispatches to (`PinChip`): it turns a
//! validated [`PinParams`] into a concrete resource. [`McuDigitalOut`] is the
//! first one, upstream's `MCU_digital_out` (`klippy/mcu.py:408-449`).
//!
//! # Lifecycle
//!
//! A resource is built while the config file is loaded, so it cannot hold the
//! device — there is none yet. It holds the MCU's [`ConfigBuilder`] (to add its
//! `config_*` command), the `pins` object (to resolve aliases) and a **shared
//! slot** that [`McuChip::attach`] fills when the MCU connects. Sending a
//! runtime command goes through that slot.
//!
//! The configuration command itself is added from a **config callback**, which
//! runs at build time (`mcu/config.rs`): that is the first moment the firmware
//! dictionary exists, and therefore the first moment a pin *name* can become a
//! pin *number*. Upstream does the same resolution at send time, in its message
//! parser (`klippy/msgproto.py`); here it happens in the callback because this
//! host's encoder takes [`ArgValue`]s.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use super::adc::{AdcRegistry, McuAdc};
use super::pwm::McuPwm;
use crate::core::klippy::cmd::gpio::{ConfigDigitalOut, QueueDigitalOut, UpdateDigitalOut};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::pins::{
    Adc, DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut,
};

/// The furthest a scheduled change may be from now, in clock ticks. Upstream's
/// `MAX_SCHEDULE_TICKS` (`klippy/mcu.py:16`), used to reject a `max_duration`
/// the firmware's scheduler cannot represent.
pub(crate) const MAX_SCHEDULE_TICKS: u64 = (1 << 31) - 1;

/// The default `max_duration`, in seconds (`klippy/mcu.py:415`).
const DEFAULT_MAX_DURATION: f64 = 2.0;

/// One MCU, as far as the pin layer is concerned.
///
/// Cheap to clone: the configuration and the device slot are shared, so the
/// handle a resource keeps sees the device the moment the MCU connects.
#[derive(Clone)]
pub struct McuChip {
    name: String,
    config: Arc<ConfigBuilder>,
    /// The shared pin registry, held **weakly**.
    ///
    /// The registry owns its chips (`PrinterPins::register_chip` stores an
    /// `Arc<dyn PinChip>`), so a strong handle back would be a reference cycle
    /// that keeps the chip — and through it the connected `Mcu` and its device —
    /// alive after a restart drops the machine's parts. The registry outlives
    /// every chip it owns, so [`McuChip::pins`] can always upgrade.
    pins: Weak<PrinterPins>,
    /// The connected transport, once [`McuChip::attach`] has run.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
    /// Routes `analog_in_state` reports to the input each `oid` belongs to.
    /// Shared by every ADC on this chip.
    adc_registry: Arc<AdcRegistry>,
}

impl McuChip {
    /// A chip for `name`, not yet connected.
    pub fn new(name: String, config: Arc<ConfigBuilder>, pins: Arc<PrinterPins>) -> Self {
        Self {
            name,
            config,
            pins: Arc::downgrade(&pins),
            mcu: Arc::new(Mutex::new(None)),
            adc_registry: Arc::new(AdcRegistry::new()),
        }
    }

    /// The MCU's own name (`mcu`, or the sub of `[mcu <name>]`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The configuration builder resources add their commands to.
    pub fn config(&self) -> Arc<ConfigBuilder> {
        Arc::clone(&self.config)
    }

    /// The shared pin registry, so the object can reserve `RESERVE_PINS_*`.
    ///
    /// Upgrades the weak handle; the registry outlives the chips it owns, so it
    /// is only absent while the machine is being torn down.
    pub fn pins(&self) -> Arc<PrinterPins> {
        self.pins
            .upgrade()
            .expect("the pins registry outlives its chips")
    }

    /// Make the connected device reachable by the resources built on this chip.
    pub fn attach(&self, mcu: Arc<Mcu>) {
        *self.lock() = Some(mcu);
    }

    /// The connected device, or `None` before connect.
    pub fn mcu(&self) -> Option<Arc<Mcu>> {
        self.lock().clone()
    }

    /// Resolve a pin alias or reservation on this chip.
    ///
    /// # Errors
    /// As [`PrinterPins::resolve_pin`].
    pub fn resolve_pin(&self, name: &str) -> Result<String, PinError> {
        self.pins().resolve_pin(&self.name, name)
    }

    /// Resolve a bus name against the firmware's enumeration and reserve the
    /// pins the firmware speaks for (`BUS_PINS_<bus>`).
    ///
    /// Upstream's `resolve_bus_name` (`klippy/extras/bus.py:9-32`). `param` is
    /// the enumeration the caller asked for (`spi_bus`, `i2c_bus`); the firmware
    /// may instead publish a single generic `bus` enumeration. `bus` is what the
    /// config said, or `None` to use the bus the firmware names `0`.
    ///
    /// A firmware with no such enumeration has no bus names to check: the
    /// config's value is returned unchanged, or `"0"` when it left the option
    /// out. This is the SPI/I2C layer's helper for F6/F7 — it is here because
    /// reservations are the pin layer's business.
    ///
    /// # Errors
    /// Returns [`PinError::MustSpecifyBus`] when the bus was left out but the
    /// firmware does not name bus 0, and [`PinError::UnknownBus`] for a name the
    /// enumeration does not have.
    pub fn resolve_bus_name(
        &self,
        mcu: &Mcu,
        param: &str,
        bus: Option<&str>,
    ) -> Result<String, PinError> {
        let dictionary = mcu.dictionary();
        let enums = dictionary
            .as_ref()
            .and_then(|dictionary| dictionary.enumeration(param))
            .or_else(|| {
                dictionary
                    .as_ref()
                    .and_then(|dictionary| dictionary.enumeration("bus"))
            });
        let Some(enums) = enums else {
            // No bus enumeration: nothing to validate against, so pass the
            // caller's choice through (upstream returns the value as-is).
            return Ok(bus.unwrap_or("0").to_string());
        };

        let bus = match bus {
            Some(bus) => {
                if enums.value(bus).is_none() {
                    return Err(PinError::UnknownBus {
                        param: param.to_string(),
                        bus: bus.to_string(),
                    });
                }
                bus.to_string()
            }
            None => enums
                .name(0)
                .map(str::to_string)
                .ok_or_else(|| PinError::MustSpecifyBus {
                    param: param.to_string(),
                    chip: self.name.clone(),
                })?,
        };

        // The firmware marks the pins a bus owns; reserve them so a config that
        // also drives one fails instead of silently stealing it.
        if let Some(pins) = dictionary
            .as_ref()
            .and_then(|dictionary| dictionary.constant(&format!("BUS_PINS_{bus}")))
            .and_then(|value| value.as_str())
        {
            for pin in pins.split(',') {
                self.pins().reserve_pin(&self.name, pin, &bus)?;
            }
        }
        Ok(bus)
    }

    fn lock(&self) -> MutexGuard<'_, Option<Arc<Mcu>>> {
        self.mcu.lock().unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PinChip for McuChip {
    fn setup_digital_out(&self, params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        Ok(Arc::new(McuDigitalOut::new(
            Arc::clone(&self.config),
            self.pins(),
            self.name.clone(),
            Arc::clone(&self.mcu),
            params.clone(),
        )))
    }

    fn setup_pwm(&self, params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
        Ok(Arc::new(McuPwm::new(
            Arc::clone(&self.config),
            self.pins(),
            self.name.clone(),
            Arc::clone(&self.mcu),
            params.clone(),
        )))
    }

    fn setup_adc(&self, params: &PinParams) -> Result<Arc<dyn Adc>, PinError> {
        Ok(Arc::new(McuAdc::new(
            Arc::clone(&self.config),
            self.pins(),
            self.name.clone(),
            Arc::clone(&self.adc_registry),
            params.clone(),
        )))
    }
}

// ===========================================================================
// McuDigitalOut
// ===========================================================================

/// The state a digital output's config callback reads and write.
///
/// Shared with the callback so the resource itself does not have to be captured
/// by it (which would be a reference cycle through the builder).
struct DigitalOutState {
    /// Longest a scheduled change may be outstanding, seconds; `0.0` = no limit.
    max_duration: Mutex<f64>,
    /// Level to drive at startup, already XORed with the pin's inversion.
    start_value: Mutex<bool>,
    /// Level the firmware falls back to on shutdown, also XORed.
    shutdown_value: Mutex<bool>,
    /// The oid the firmware assigned, set by the config callback.
    oid: Mutex<Option<u8>>,
}

/// One `[output_pin]`-style digital output on an MCU.
pub struct McuDigitalOut {
    state: Arc<DigitalOutState>,
    /// Shared with the chip, so runtime sends reach the connected device.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
    pin: PinParams,
}

impl McuDigitalOut {
    /// Build the resource and register its config callback.
    ///
    /// The callback cannot be registered after the configuration is built;
    /// resources are always built while the config file is loaded, so the
    /// panic is a wiring invariant rather than something a client can cause.
    fn new(
        config: Arc<ConfigBuilder>,
        pins: Arc<PrinterPins>,
        chip_name: String,
        mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
        pin: PinParams,
    ) -> Self {
        let state = Arc::new(DigitalOutState {
            max_duration: Mutex::new(DEFAULT_MAX_DURATION),
            // Defaults are "off"; with an inverting pin, off is a high level.
            start_value: Mutex::new(pin.invert),
            shutdown_value: Mutex::new(pin.invert),
            oid: Mutex::new(None),
        });

        let callback_state = Arc::clone(&state);
        // Weak, not Arc: the registry owns this chip (`PrinterPins::chip_impls`),
        // and the chip owns the `ConfigBuilder` this callback is stored in. A
        // strong handle here is a cycle that keeps the registry — and through
        // `McuChip::mcu`, the connected device and its receive task — alive after
        // a restart drops the machine's parts (`mcu/restart.rs`, `object.rs`).
        let callback_pins = Arc::downgrade(&pins);
        let callback_pin = pin.clone();
        config
            .register_config_callback(Box::new(move |builder, mcu| {
                let pins = callback_pins
                    .upgrade()
                    .expect("the pins registry outlives the resources it built");
                callback_state.build(builder, mcu, &pins, &chip_name, &callback_pin)
            }))
            .expect("a resource is always built before the configuration is");

        Self { state, mcu, pin }
    }

    /// The oid the config callback assigned.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] before the configuration has been built.
    fn oid(&self) -> Result<u8, McuError> {
        self.state
            .oid
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .ok_or_else(|| McuError::Config("digital output is not configured yet".to_string()))
    }

    /// Send a runtime command to the connected device.
    fn send<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        let mcu = self
            .mcu
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        mcu.send_msg(cmd)
    }
}

impl DigitalOut for McuDigitalOut {
    fn setup_max_duration(&self, max_duration: f64) {
        *self
            .state
            .max_duration
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = max_duration;
    }

    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        let invert = self.pin.invert;
        *self
            .state
            .start_value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = start_value ^ invert;
        *self
            .state
            .shutdown_value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = shutdown_value ^ invert;
    }

    fn queue_digital_out(&self, clock: u32, value: bool) -> Result<(), McuError> {
        let oid = self.oid()?;
        // For a plain output `on_ticks` carries the level (`gpiocmds.c:174`).
        let on_ticks = u32::from(value ^ self.pin.invert);
        self.send(&QueueDigitalOut {
            oid,
            clock,
            on_ticks,
        })
    }

    fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
        let oid = self.oid()?;
        self.send(&UpdateDigitalOut {
            oid,
            value: u8::from(value ^ self.pin.invert),
        })
    }
}

impl DigitalOutState {
    /// The build-time half: resolve the pin and add the configuration.
    ///
    /// Upstream's `MCU_digital_out._build_config` (`klippy/mcu.py:426-449`).
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
        chip_name: &str,
        pin: &PinParams,
    ) -> Result<(), McuError> {
        let max_duration = *self
            .max_duration
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let start_value = *self
            .start_value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let shutdown_value = *self
            .shutdown_value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The firmware falls back to the shutdown level when a scheduled change
        // outlives `max_duration`, so the two levels have to agree unless the
        // limit is off.
        if max_duration != 0.0 && start_value != shutdown_value {
            return Err(PinError::MaxDurationMismatch.into());
        }
        let max_duration_ticks = mcu.seconds_to_clock(max_duration)?;
        if max_duration_ticks > MAX_SCHEDULE_TICKS {
            return Err(PinError::MaxDurationTooLarge.into());
        }

        builder.request_move_queue_slot()?;
        let oid = builder.create_oid()?;
        *self.oid.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(oid);

        // Aliases and reservations first (no dictionary needed), then the
        // firmware's pin enumeration.
        let canonical = pins.resolve_pin(chip_name, &pin.pin)?;
        let number = pin_number(mcu, &canonical, chip_name)?;

        builder.add_config_cmd(&ConfigDigitalOut {
            oid,
            pin: number,
            value: u8::from(start_value),
            default_value: u8::from(shutdown_value),
            max_duration: max_duration_ticks as u32,
        })?;
        builder.add_restart_cmd(&UpdateDigitalOut {
            oid,
            value: u8::from(start_value),
        })?;
        Ok(())
    }
}

/// Look a pin name up in the firmware's `pin` enumeration.
///
/// Upstream reports this from the message parser and the caller turns it into
/// `Pin '%s' is not a valid pin name on mcu '%s'` (`klippy/mcu.py:1032-1038`).
pub(crate) fn pin_number(mcu: &Mcu, name: &str, chip_name: &str) -> Result<u32, PinError> {
    let invalid = || PinError::InvalidName {
        pin: name.to_string(),
        chip: chip_name.to_string(),
    };
    let dictionary = mcu.dictionary().ok_or_else(invalid)?;
    let number = dictionary
        .enumeration("pin")
        .and_then(|enumeration| enumeration.value(name))
        .ok_or_else(invalid)?;
    u32::try_from(number).map_err(|_| invalid())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::devices::test::TestDevice;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;
    use serde_json::json;

    /// A dictionary with the pin enumeration and the GPIO commands.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 10,
                "update_digital_out oid=%c value=%c": 11,
                "queue_digital_out oid=%c clock=%u on_ticks=%u": 12
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "enumerations": {
                "pin": {"PA0": 0, "PA1": 1, "PB2": 2}
            },
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
    }

    /// An identified MCU that sends nowhere.
    fn mcu() -> Mcu {
        let mcu = Mcu::for_test("mcu", Interface::new(TestDevice::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu
    }

    /// A chip with the main MCU registered under `mcu`.
    fn chip() -> (McuChip, Arc<PrinterPins>) {
        let pins = Arc::new(PrinterPins::new());
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        pins.register_chip("mcu", Arc::new(chip.clone())).unwrap();
        (chip, pins)
    }

    #[test]
    fn test_a_chip_does_not_keep_the_pin_registry_alive() {
        // The registry owns its chips. If a chip reached back strongly, the
        // pair would outlive a restart — and with the chip, the connected MCU
        // and its device would stay open.
        let (chip, pins) = chip();

        let weak = Arc::downgrade(&pins);
        drop(chip);
        drop(pins);

        assert!(
            weak.upgrade().is_none(),
            "the pin registry was kept alive by its own chip"
        );
    }

    #[test]
    fn test_a_resource_does_not_keep_the_pin_registry_alive() {
        // A resource's config callback needs the registry at build time, but it
        // must hold it weakly: a strong handle cycles through the chip's
        // `ConfigBuilder` (`registry -> chip -> config -> callback -> registry`)
        // and keeps the registry — with the connected MCU behind the chip —
        // alive after a restart drops the machine's parts.
        let (chip, pins) = chip();
        let weak = Arc::downgrade(&pins);

        let params = PinParams {
            chip_name: "mcu".to_string(),
            pin: "PA0".to_string(),
            invert: false,
            pullup: 0,
            share_type: None,
        };
        let resource = McuDigitalOut::new(
            chip.config(),
            chip.pins(),
            "mcu".to_string(),
            Arc::clone(&chip.mcu),
            params,
        );

        drop(resource);
        drop(chip);
        drop(pins);

        assert!(
            weak.upgrade().is_none(),
            "a resource's config callback kept the pin registry alive"
        );
    }

    /// The decoded `(name, args)` of every config command the builder produced.
    fn built_commands(
        chip: &McuChip,
        mcu: &Mcu,
    ) -> Vec<(String, Vec<crate::core::klippy::msg::proto::ArgValue>)> {
        let built = chip.config().build(mcu).unwrap();
        let mut parser = crate::core::klippy::msg::parser::Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        built
            .config
            .iter()
            .map(|payload| {
                let frame = crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).unwrap();
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    #[tokio::test]
    async fn test_a_digital_out_builds_the_expected_config() {
        use crate::core::klippy::msg::proto::ArgValue;

        let (chip, pins) = chip();
        pins.setup_digital_out("PA1", None).unwrap();
        let mcu = mcu();

        let commands = built_commands(&chip, &mcu);

        // allocate_oids, config_digital_out, finalize_config.
        assert_eq!(commands.len(), 3);
        assert_eq!(commands[0].0, "allocate_oids");
        assert_eq!(commands[0].1, vec![ArgValue::UInt8(1)]);
        assert_eq!(commands[1].0, "config_digital_out");
        assert_eq!(
            commands[1].1,
            vec![
                ArgValue::UInt8(0),           // oid
                ArgValue::UInt32(1),          // PA1
                ArgValue::UInt8(0),           // value
                ArgValue::UInt8(0),           // default_value
                ArgValue::UInt32(40_000_000), // 2 s at 20 MHz
            ]
        );
        assert_eq!(commands[2].0, "finalize_config");
    }

    #[tokio::test]
    async fn test_the_start_value_is_replayed_on_restart() {
        use crate::core::klippy::msg::proto::ArgValue;

        let (chip, pins) = chip();
        let out = pins.setup_digital_out("PB2", None).unwrap();
        out.setup_start_value(true, false);
        out.setup_max_duration(0.0);
        let mcu = mcu();

        let built = chip.config().build(&mcu).unwrap();

        // `update_digital_out value=1` goes in the restart list.
        let mut parser = crate::core::klippy::msg::parser::Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        let decoded = parser
            .decode(
                crate::core::klippy::frame::Frame::new(0, built.restart[0].payload().to_vec())
                    .into(),
            )
            .unwrap();
        assert_eq!(decoded[0].0.name, "update_digital_out");
        assert_eq!(decoded[0].1, vec![ArgValue::UInt8(0), ArgValue::UInt8(1)]);
    }

    #[tokio::test]
    async fn test_an_inverting_pin_flips_the_level() {
        use crate::core::klippy::msg::proto::ArgValue;

        let (chip, pins) = chip();
        let out = pins.setup_digital_out("!PA0", None).unwrap();
        // Logically off, but the pin is active low, so the level is high.
        out.setup_start_value(false, false);
        let mcu = mcu();

        let commands = built_commands(&chip, &mcu);

        assert_eq!(
            commands[1].1,
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(1), // value
                ArgValue::UInt8(1), // default_value
                ArgValue::UInt32(40_000_000),
            ]
        );
    }

    #[tokio::test]
    async fn test_a_max_duration_mismatch_fails_the_build() {
        let (chip, pins) = chip();
        let out = pins.setup_digital_out("PA1", None).unwrap();
        out.setup_start_value(true, false);
        // max_duration stays at its 2 s default, so start != shutdown is invalid.
        let mcu = mcu();

        let err = chip.config().build(&mcu).unwrap_err();

        assert!(matches!(err, McuError::Config(_)), "{err:?}");
        assert!(
            err.to_string().contains("start value equal to shutdown"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_a_pin_not_in_the_enumeration_names_the_mcu() {
        let (chip, pins) = chip();
        pins.setup_digital_out("PA9", None).unwrap();
        let mcu = mcu();

        let err = chip.config().build(&mcu).unwrap_err();

        assert!(
            err.to_string()
                .contains("Pin 'PA9' is not a valid pin name on mcu 'mcu'"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_a_reserved_pin_fails_the_build() {
        let (chip, pins) = chip();
        pins.reserve_pin("mcu", "PA1", "uart0").unwrap();
        pins.setup_digital_out("PA1", None).unwrap();
        let mcu = mcu();

        let err = chip.config().build(&mcu).unwrap_err();

        assert!(err.to_string().contains("reserved for uart0"), "{err}");
    }

    #[test]
    fn test_a_runtime_send_before_the_build_is_reported() {
        // No config callback has run, so there is no oid yet.
        let (_chip, pins) = chip();
        let out = pins.setup_digital_out("PA1", None).unwrap();

        let err = out.update_digital_out(true).unwrap_err();

        assert!(matches!(err, McuError::Config(_)), "{err:?}");
        assert!(err.to_string().contains("not configured"), "{err}");
    }

    #[tokio::test]
    async fn test_a_runtime_send_reaches_the_attached_mcu() {
        let (chip, pins) = chip();
        let out = pins.setup_digital_out("PA1", None).unwrap();
        let mcu = Arc::new(mcu());
        chip.config().build(mcu.as_ref()).unwrap();
        chip.attach(Arc::clone(&mcu));

        // Both commands are in the dictionary and their arguments encode; a
        // missing message or a wrong argument list would error here.
        out.update_digital_out(true).unwrap();
        out.queue_digital_out(1_000, false).unwrap();
    }

    #[tokio::test]
    async fn test_a_runtime_send_before_connect_is_reported() {
        let (chip, pins) = chip();
        let out = pins.setup_digital_out("PA1", None).unwrap();
        chip.config().build(&mcu()).unwrap();
        // The chip was never attached to a device.

        let err = out.update_digital_out(true).unwrap_err();

        assert!(err.to_string().contains("not connected"), "{err}");
    }

    // -----------------------------------------------------------------------
    // resolve_bus_name
    // -----------------------------------------------------------------------

    /// A dictionary that publishes bus names and the pins of one bus.
    fn bus_dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
            },
            "enumerations": {
                "pin": {"PA0": 0, "PA1": 1, "PB2": 2},
                "spi_bus": {"spi1": 0, "spi2": 1}
            },
            "config": {"CLOCK_FREQ": 20000000, "BUS_PINS_spi1": "PB2"}
        }))
        .unwrap()
    }

    fn mcu_with(dictionary: Dictionary) -> Mcu {
        let mcu = Mcu::for_test("mcu", Interface::new(TestDevice::new(Vec::new())));
        mcu.install_dictionary(dictionary).unwrap();
        mcu
    }

    #[tokio::test]
    async fn test_resolve_bus_name_reserves_the_pins_the_firmware_owns() {
        let (chip, pins) = chip();
        let mcu = mcu_with(bus_dictionary());

        let bus = chip
            .resolve_bus_name(&mcu, "spi_bus", Some("spi1"))
            .unwrap();

        assert_eq!(bus, "spi1");
        // The firmware's `BUS_PINS_spi1` names PB2, so it cannot be driven.
        let err = pins.resolve_pin("mcu", "PB2").unwrap_err();
        assert!(err.to_string().contains("reserved for spi1"), "{err}");
    }

    #[tokio::test]
    async fn test_resolve_bus_name_defaults_to_the_bus_named_zero() {
        let (chip, _pins) = chip();
        let mcu = mcu_with(bus_dictionary());

        assert_eq!(
            chip.resolve_bus_name(&mcu, "spi_bus", None).unwrap(),
            "spi1"
        );
    }

    #[tokio::test]
    async fn test_resolve_bus_name_falls_back_to_the_generic_bus_enumeration() {
        let dictionary = Dictionary::from_json(json!({
            "commands": {},
            "enumerations": {"bus": {"i2c1": 0}},
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap();
        let (chip, _pins) = chip();
        let mcu = mcu_with(dictionary);

        assert_eq!(
            chip.resolve_bus_name(&mcu, "i2c_bus", Some("i2c1"))
                .unwrap(),
            "i2c1"
        );
    }

    #[tokio::test]
    async fn test_resolve_bus_name_rejects_an_unknown_bus() {
        let (chip, _pins) = chip();
        let mcu = mcu_with(bus_dictionary());

        let err = chip
            .resolve_bus_name(&mcu, "spi_bus", Some("spi9"))
            .unwrap_err();

        assert!(matches!(&err, PinError::UnknownBus { .. }), "{err:?}");
        assert_eq!(err.to_string(), "Unknown spi_bus 'spi9'");
    }

    #[tokio::test]
    async fn test_resolve_bus_name_requires_a_bus_when_zero_is_unnamed() {
        let dictionary = Dictionary::from_json(json!({
            "commands": {},
            "enumerations": {"spi_bus": {"spi1": 1}},
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap();
        let (chip, _pins) = chip();
        let mcu = mcu_with(dictionary);

        let err = chip.resolve_bus_name(&mcu, "spi_bus", None).unwrap_err();

        assert!(matches!(&err, PinError::MustSpecifyBus { .. }), "{err:?}");
        assert_eq!(err.to_string(), "Must specify spi_bus on mcu 'mcu'");
    }

    #[tokio::test]
    async fn test_resolve_bus_name_without_an_enumeration_passes_the_choice_through() {
        // The plain dictionary has a `pin` enumeration but no bus one.
        let (chip, _pins) = chip();
        let mcu = mcu();

        assert_eq!(
            chip.resolve_bus_name(&mcu, "spi_bus", Some("spi3"))
                .unwrap(),
            "spi3"
        );
        assert_eq!(chip.resolve_bus_name(&mcu, "spi_bus", None).unwrap(), "0");
    }
}
