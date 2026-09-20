//! `[mcu]` as a printer object: the section, and the connection it needs.
//!
//! Upstream's `MCU` is one printer object per `[mcu]` / `[mcu <name>]` section
//! (`klippy/mcu.py:1147`), registered by `mcu.add_printer_objects` under the
//! section's name (`klippy/mcu.py:1239`). Its status is the identify snapshot
//! `MCUStatsHelper` takes once the handshake is done (`klippy/mcu.py:938-975`).
//!
//! Two-phase construction is why this type is separate from [`Mcu`]: the loader
//! builds it from the section without touching the device, and
//! [`PrinterObject::connect`] parses the section and performs the identify
//! handshake when the machine comes up. That keeps a config parse free of side
//! effects — no serial port is opened just to read the file — which is also why
//! the whole [`ConfigSection`] is kept rather than a parsed [`McuConfig`].
//!
//! The object also owns the MCU's [`ConfigBuilder`], created here rather than at
//! connect: resources add their oids and `config_*` commands while the config
//! file is loaded, and connect is what finally sends them
//! (`builder.configure`), after identify has made the dictionary available.
//!
//! The reported fields are the three identify ones. `last_stats` is upstream's
//! fourth (`klippy/mcu.py:975`) and is **not** reported yet: it is accumulated
//! from the `stats` event, which today is only logged
//! ([`register_stats_logging`](crate::core::klippy::event::stats::register_stats_logging)).
//! It comes back with the statistics consumer.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Map, Value};
use tracing::warn;

use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::config::ConfigSection;
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::event::{IsShutdown, McuEvent, Shutdown, Starting};
use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu, McuChip, McuError};
use crate::core::klippy::pins::{PinError, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

/// The printer object for one `[mcu]` / `[mcu <name>]` section.
pub struct McuObject {
    /// The section as parsed, kept whole so the device is only opened at
    /// connect time.
    section: ConfigSection,
    /// This MCU as a pin chip: its name, its configuration builder, and the
    /// slot resources use to reach the device once it connects (`mcu/pin.rs`).
    ///
    /// Built at construction, not at connect, because resources add their
    /// `config_*` commands while the config file is being loaded — long before
    /// the device is opened.
    chip: McuChip,
    /// What `objects/query` reports; `{}` until the handshake fills it, which is
    /// what upstream's `_get_status_info` starts as.
    status: Mutex<Value>,
    /// The machine, for reporting a firmware shutdown. `Weak` because the
    /// printer's registry owns this object: a strong handle would be a cycle
    /// that keeps the printer (and its device) alive forever.
    printer: Weak<Printer>,
}

impl McuObject {
    /// Build the object for `section`, without touching the device.
    ///
    /// Also declares this MCU as a chip on the shared `pins` object, as
    /// upstream's `MCUConfigHelper.__init__` does
    /// (`klippy/mcu.py:996-997`): a pin description may name this MCU as its
    /// chip from the moment the config file mentions it.
    ///
    /// # Errors
    /// Returns [`PinError::DuplicateChip`] if another `[mcu]` section already
    /// claimed this name (`[mcu]` and `[mcu mcu]` would collide).
    pub fn new(section: ConfigSection, printer: &Arc<Printer>) -> Result<Self, PinError> {
        let name = section.sub.clone().unwrap_or_else(|| section.id.clone());
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let chip = McuChip::new(name, Arc::new(ConfigBuilder::new()), Arc::clone(&pins));
        // The chip is registered by value-shared handle, so the slot the
        // resources read is the same one connect fills.
        pins.register_chip(chip.name(), Arc::new(chip.clone()))?;
        Ok(Self {
            section,
            chip,
            status: Mutex::new(json!({})),
            printer: Arc::downgrade(printer),
        })
    }

    /// The MCU's own name, as upstream's `MCU.get_name` reports it.
    pub fn name(&self) -> &str {
        self.chip.name()
    }

    /// The configuration builder for this MCU.
    ///
    /// What a resource (a pin, a bus, a sensor) uses to reserve an oid and add
    /// its `config_*` command. It exists before the device does, which is the
    /// point: the config file is loaded before anything connects.
    pub fn config(&self) -> Arc<ConfigBuilder> {
        self.chip.config()
    }

    /// Snapshot a connected MCU's identify status for `objects/query`.
    fn set_status(&self, mcu: &Mcu) {
        let status = mcu
            .dictionary()
            .map(|dictionary| status_from(&dictionary))
            .unwrap_or_else(|| json!({}));
        *self.status.lock().unwrap_or_else(|p| p.into_inner()) = status;
    }

    /// Reserve the pins the firmware says belong to it.
    ///
    /// The dictionary carries `RESERVE_PINS_<name>` constants as
    /// comma-separated pin lists (UART, USB, …); upstream reserves them at
    /// `klippy:mcu_identify` (`klippy/mcu.py:1091-1100`), which is just after
    /// identify here. A pin already reserved for something else is a config
    /// error, because the resource that wants it would silently fight the
    /// firmware.
    fn reserve_pins(&self, mcu: &Mcu) -> Result<(), PinError> {
        let Some(dictionary) = mcu.dictionary() else {
            return Ok(());
        };
        let pins = self.chip.pins();
        for (key, value) in dictionary.constants() {
            let Some(reserve_name) = key.strip_prefix("RESERVE_PINS_") else {
                continue;
            };
            let Some(pin_list) = value.as_str() else {
                continue;
            };
            for pin in pin_list.split(',') {
                let pin = pin.trim();
                if !pin.is_empty() {
                    pins.reserve_pin(self.chip.name(), pin, reserve_name)?;
                }
            }
        }
        Ok(())
    }

    /// Whether this bring-up follows a `firmware_restart`.
    ///
    /// Only then is the firmware itself reset; a first start and a plain
    /// `restart` reconnect without touching it (upstream keys the same decision
    /// on `start_reason`, `klippy/mcu.py:678-680`).
    fn is_firmware_restart(&self) -> bool {
        self.printer
            .upgrade()
            .and_then(|printer| printer.start_reason())
            .as_deref()
            == Some("firmware_restart")
    }

    /// Report a firmware shutdown, restart, or already-stopped state.
    ///
    /// The events carry the reason; the machine is what knows what a stop
    /// means, so the handler only hands it over. Bound **after** the
    /// configuration handshake, so that a reset this host performs while
    /// configuring does not look like a spontaneous stop (see
    /// `mcu/config.rs`, which clears a stopped or differently-configured
    /// firmware before sending the configuration).
    fn bind_shutdown(&self, mcu: &Mcu) -> Result<(), McuError> {
        let name = self.chip.name().to_string();

        if !mcu.has_message(Shutdown::NAME) {
            warn!(
                "MCU '{name}' does not report `shutdown`; a firmware stop will not be \
                 noticed, and a configuration reset falls back to a delay"
            );
        }

        if mcu.has_message(Shutdown::NAME) {
            let printer = self.printer.clone();
            let name = name.clone();
            mcu.bind_event::<Shutdown, _>(move |event| {
                let msg = match event.clock {
                    Some(clock) => {
                        format!("MCU '{name}' shutdown: {} (clock {clock})", event.reason)
                    }
                    None => format!("MCU '{name}' shutdown: {}", event.reason),
                };
                report_shutdown(&printer, &msg);
            })?;
        }
        if mcu.has_message(IsShutdown::NAME) {
            let printer = self.printer.clone();
            let name = name.clone();
            mcu.bind_event::<IsShutdown, _>(move |event| {
                report_shutdown(
                    &printer,
                    &format!("MCU '{name}' is shutdown: {}", event.reason),
                );
            })?;
        }
        if mcu.has_message(Starting::NAME) {
            let printer = self.printer.clone();
            mcu.bind_event::<Starting, _>(move |_| {
                report_shutdown(&printer, &format!("MCU '{name}' restarted"));
            })?;
        }
        Ok(())
    }
}

impl PrinterObject for McuObject {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.status
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            // The device is opened here, not at construction: that is the point
            // of two-phase construction, and upstream parses the section at the
            // same moment (`klippy/mcu.py:1147`). Opening a serial port or
            // dlopen-ing the host library blocks, briefly, on this task.
            let config = McuConfig::new(&self.section).map_err(KlippyError::Internal)?;
            // A `firmware_restart` is the one bring-up that resets the firmware
            // itself, and it has to happen while the transport is still closed.
            if self.is_firmware_restart() {
                super::restart::reset_firmware(&config)
                    .await
                    .map_err(KlippyError::Internal)?;
            }
            let interface = config.open().map_err(KlippyError::Internal)?;
            let mcu = Mcu::connect(config.name, interface)
                .await
                .map_err(|err| KlippyError::Connection(err.to_string()))?;
            // Make the device reachable by resources before the configuration
            // is built; a resource's runtime methods need it.
            self.chip.attach(Arc::clone(&mcu));
            // Identify installed the dictionary; reserve the pins the firmware
            // owns before anything resolves one.
            self.reserve_pins(&mcu)
                .map_err(|err| KlippyError::Internal(err.to_string()))?;
            // Now the accumulated configuration can be encoded and sent, and the
            // firmware either adopts it or confirms it already has it
            // (`mcu/config.rs`).
            self.chip
                .config()
                .configure(&mcu)
                .await
                .map_err(|err| KlippyError::Connection(err.to_string()))?;
            // Only now does a firmware shutdown mean something the machine
            // should report: the configuration handshake is done, so nothing
            // this host sent is still in flight.
            self.bind_shutdown(&mcu)
                .map_err(|err| KlippyError::Internal(err.to_string()))?;
            self.set_status(&mcu);
            Ok(())
        })
    }
}

/// Log a firmware stop and put the machine into its shutdown state.
fn report_shutdown(printer: &Weak<Printer>, msg: &str) {
    warn!("{msg}");
    if let Some(printer) = printer.upgrade() {
        printer.invoke_shutdown(msg);
    }
}

/// The status upstream's `MCUStatsHelper._mcu_identify` fills
/// (`klippy/mcu.py:938-948`).
fn status_from(dictionary: &Dictionary) -> Value {
    let raw = dictionary.raw();
    let constants: Map<String, Value> = dictionary
        .constants()
        .iter()
        .map(|(name, value)| (name.clone(), value.clone()))
        .collect();
    json!({
        "mcu_version": raw.get("version").cloned().unwrap_or(Value::Null),
        "mcu_build_versions": raw.get("build_versions").cloned().unwrap_or(Value::Null),
        "mcu_constants": constants,
    })
}

/// Upstream's `load_config` for the `mcu` section.
///
/// Returns the object; the loader registers it under the section identifier.
/// The printer is part of the factory signature for object types that wire
/// themselves up as they are built (they register event handlers); an MCU
/// connects through [`PrinterObject::connect`] instead, so it does not use it.
pub fn load_config(
    section: &ConfigSection,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, String> {
    let object = McuObject::new(section.clone(), _printer).map_err(|err| err.to_string())?;
    Ok(Arc::new(object))
}

/// Upstream's `load_config_prefix` for `[mcu <name>]`.
///
/// The same object today. The two entry points differ upstream only by the clock
/// a secondary MCU synchronizes to (`klippy/mcu.py:1245`), which arrives with the
/// clock layer.
pub fn load_config_prefix(
    section: &ConfigSection,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, String> {
    load_config(section, printer)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::msg::proto::Payload;
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::ManualReactor;
    use tokio::time::Duration;

    fn section(sub: Option<&str>) -> ConfigSection {
        ConfigSection::new("mcu", sub)
    }

    /// A printer with the `pins` object, as the loader leaves it before any
    /// section is loaded.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        printer
    }

    /// An MCU object for `sub`, over such a printer.
    fn object(sub: Option<&str>) -> McuObject {
        McuObject::new(section(sub), &printer()).unwrap()
    }

    #[test]
    fn test_an_mcu_registers_itself_as_a_chip() {
        // A pin description may name this MCU from the moment the config file
        // mentions it, so the chip is declared when the object is built.
        let printer = printer();
        McuObject::new(section(Some("zboard")), &printer).unwrap();

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .unwrap();
        assert_eq!(pins.chips(), ["zboard"]);
    }

    #[test]
    fn test_two_mcus_cannot_claim_the_same_chip_name() {
        let printer = printer();
        McuObject::new(section(None), &printer).unwrap();

        let err = match McuObject::new(section(None), &printer) {
            Ok(_) => panic!("a second MCU claimed the same chip name"),
            Err(err) => err,
        };

        assert!(matches!(err, PinError::DuplicateChip(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_reserve_pins_marks_what_the_firmware_owns() {
        // The printer is held for the test: `McuChip` reaches the pin registry
        // weakly, and the registry is the printer's.
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        let mcu = Mcu::for_test("mcu", Interface::new(TestDevice::new(vec![])));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "config": {
                    "CLOCK_FREQ": 20000000,
                    "RESERVE_PINS_uart0": "PA0,PA1",
                }
            }))
            .unwrap(),
        )
        .unwrap();

        object.reserve_pins(&mcu).unwrap();

        // The reserved pins cannot be resolved; a free one can.
        assert!(object.chip.pins().resolve_pin("mcu", "PA0").is_err());
        assert!(object.chip.pins().resolve_pin("mcu", "PA2").is_ok());
    }

    #[tokio::test]
    async fn test_a_firmware_shutdown_stops_the_printer() {
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();

        // The firmware answers the request with an unsolicited `shutdown`, the
        // way it does when one of its shutdown handlers runs.
        let mut request = Payload::new();
        request.push_i16(4).unwrap(); // get_uptime
        let mut shutdown = Payload::new();
        shutdown.push_i16(20).unwrap(); // shutdown
        shutdown.push_u32(1234).unwrap(); // clock
        shutdown.push_u16(0).unwrap(); // static_string_id 0

        let device = TestDevice::new(vec![MappingEntry {
            input: Frame::new(0, request.into_raw()),
            outputs: vec![Frame::new(0, shutdown.into_raw())],
        }]);
        let mcu = Mcu::for_test("mcu", Interface::new(device));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "commands": {"get_uptime": 4},
                "responses": {"shutdown clock=%u static_string_id=%hu": 20},
                "enumerations": {"static_string_id": {"Move queue overflow": 0}}
            }))
            .unwrap(),
        )
        .unwrap();

        object.bind_shutdown(&mcu).unwrap();
        mcu.send("get_uptime", &[]).unwrap();
        // Let the receive task decode and dispatch the shutdown frame.
        tokio::time::sleep(Duration::from_millis(20)).await;

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Shutdown);
        assert!(
            state.message.contains("Move queue overflow"),
            "{}",
            state.message
        );
    }

    #[test]
    fn test_the_config_builder_exists_before_the_device_does() {
        // Resources add their `config_*` commands while the config file is
        // loaded — long before anything connects — so the builder has to be
        // usable from the object as it is built.
        let object = object(None);

        assert_eq!(object.config().create_oid().unwrap(), 0);
        assert!(!object.config().is_finalized());
    }

    #[test]
    fn test_the_mcus_own_name_is_the_section_without_the_id() {
        // `[mcu]` is "mcu"; `[mcu zboard]` is "zboard" — the registry key keeps
        // the id (`mcu zboard`), the name does not (`klippy/mcu.py:1151-1153`).
        assert_eq!(object(None).name(), "mcu");
        assert_eq!(object(Some("zboard")).name(), "zboard");
    }

    #[test]
    fn test_status_is_empty_until_connected() {
        let object = object(None);

        assert_eq!(object.get_status(0.0), json!({}));
    }

    #[tokio::test]
    async fn test_a_connected_mcu_reports_its_identify_snapshot() {
        let object = object(None);
        let mcu = Arc::new(Mcu::for_test(
            "mcu",
            Interface::new(TestDevice::new(vec![])),
        ));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "version": "v0.12.0-1-g1234567",
                "build_versions": "gcc: 12.3.1",
                "config": {"CLOCK_FREQ": 20000000},
            }))
            .unwrap(),
        )
        .unwrap();

        object.set_status(&mcu);

        assert_eq!(
            object.get_status(0.0),
            json!({
                "mcu_version": "v0.12.0-1-g1234567",
                "mcu_build_versions": "gcc: 12.3.1",
                "mcu_constants": {"CLOCK_FREQ": 20000000},
            })
        );
    }

    #[tokio::test]
    async fn test_connecting_a_section_with_no_usable_interface_fails() {
        let mut section = section(None);
        section.parameters.insert(
            "serial".to_string(),
            crate::core::klippy::config::ConfigValue::Single("/dev/not-a-serial-port".to_string()),
        );
        let object = McuObject::new(section, &printer()).unwrap();

        let err = object.connect().await.unwrap_err();

        // The section's own parser names the port; the object does not have to.
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
    }

    #[test]
    fn test_the_loader_builds_an_object_from_the_section() {
        let printer = printer();

        let object = load_config(&section(Some("zboard")), &printer).unwrap();

        assert_eq!(object.get_status(0.0), json!({}));
    }
}
