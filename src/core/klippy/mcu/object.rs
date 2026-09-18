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

use std::sync::{Arc, Mutex};

use serde_json::{json, Map, Value};

use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::config::ConfigSection;
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

/// The printer object for one `[mcu]` / `[mcu <name>]` section.
pub struct McuObject {
    /// The section as parsed, kept whole so the device is only opened at
    /// connect time.
    section: ConfigSection,
    /// The MCU's own name: `mcu`, or the sub for `[mcu zboard]`
    /// (`klippy/mcu.py:1151-1153`). Not the section identifier — that is what
    /// the registry key is, and the two differ for every secondary MCU.
    name: String,
    /// The configuration this MCU's resources build up, and the handshake that
    /// sends it on connect (`mcu/config.rs`).
    ///
    /// Built at construction, not at connect, because resources add their
    /// `config_*` commands while the config file is being loaded — long before
    /// the device is opened.
    config: Arc<ConfigBuilder>,
    /// What `objects/query` reports; `{}` until the handshake fills it, which is
    /// what upstream's `_get_status_info` starts as.
    status: Mutex<Value>,
    /// The connected transport, kept so it outlives this handle: dropping the
    /// last [`Arc<Mcu>`] shuts the interface down.
    mcu: Mutex<Option<Arc<Mcu>>>,
}

impl McuObject {
    /// Build the object for `section`, without touching the device.
    pub fn new(section: ConfigSection) -> Self {
        let name = section.sub.clone().unwrap_or_else(|| section.id.clone());
        Self {
            section,
            name,
            config: Arc::new(ConfigBuilder::new()),
            status: Mutex::new(json!({})),
            mcu: Mutex::new(None),
        }
    }

    /// The MCU's own name, as upstream's `MCU.get_name` reports it.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The configuration builder for this MCU.
    ///
    /// What a resource (a pin, a bus, a sensor) uses to reserve an oid and add
    /// its `config_*` command. It exists before the device does, which is the
    /// point: the config file is loaded before anything connects.
    pub fn config(&self) -> Arc<ConfigBuilder> {
        Arc::clone(&self.config)
    }

    /// Take ownership of a connected MCU and snapshot its identify status.
    fn attach(&self, mcu: Arc<Mcu>) {
        let status = mcu
            .dictionary()
            .map(|dictionary| status_from(&dictionary))
            .unwrap_or_else(|| json!({}));
        *self.status.lock().unwrap_or_else(|p| p.into_inner()) = status;
        *self.mcu.lock().unwrap_or_else(|p| p.into_inner()) = Some(mcu);
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
            let mcu = Mcu::connect(config)
                .await
                .map_err(|err| KlippyError::Connection(err.to_string()))?;
            // Identify installed the dictionary; now the accumulated
            // configuration can be encoded and sent, and the firmware either
            // adopts it or confirms it already has it (`mcu/config.rs`).
            self.config
                .configure(&mcu)
                .await
                .map_err(|err| KlippyError::Connection(err.to_string()))?;
            self.attach(mcu);
            Ok(())
        })
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
    Ok(Arc::new(McuObject::new(section.clone())))
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
    use crate::core::klippy::interface::test::TestDevice;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::reactor::ManualReactor;

    fn section(sub: Option<&str>) -> ConfigSection {
        ConfigSection::new("mcu", sub)
    }

    #[test]
    fn test_the_config_builder_exists_before_the_device_does() {
        // Resources add their `config_*` commands while the config file is
        // loaded — long before anything connects — so the builder has to be
        // usable from the object as it is built.
        let object = McuObject::new(section(None));

        assert_eq!(object.config().create_oid().unwrap(), 0);
        assert!(!object.config().is_finalized());
    }

    #[test]
    fn test_the_mcus_own_name_is_the_section_without_the_id() {
        // `[mcu]` is "mcu"; `[mcu zboard]` is "zboard" — the registry key keeps
        // the id (`mcu zboard`), the name does not (`klippy/mcu.py:1151-1153`).
        assert_eq!(McuObject::new(section(None)).name(), "mcu");
        assert_eq!(McuObject::new(section(Some("zboard"))).name(), "zboard");
    }

    #[test]
    fn test_status_is_empty_until_connected() {
        let object = McuObject::new(section(None));

        assert_eq!(object.get_status(0.0), json!({}));
    }

    #[tokio::test]
    async fn test_a_connected_mcu_reports_its_identify_snapshot() {
        let object = McuObject::new(section(None));
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

        object.attach(mcu);

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
        let object = McuObject::new(section);

        let err = object.connect().await.unwrap_err();

        // The section's own parser names the port; the object does not have to.
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
    }

    #[test]
    fn test_the_loader_builds_an_object_from_the_section() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        let object = load_config(&section(Some("zboard")), &printer).unwrap();

        assert_eq!(object.get_status(0.0), json!({}));
    }
}
