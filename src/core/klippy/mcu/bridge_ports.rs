//! `[uart_device <name>]` / `[can_device <name>]` — a bridge downstream port.
//!
//! M1's configuration face (see the work-log plan): a downstream port is a
//! named config section that later passes look up by section name, instead of
//! the topology being learned from the wire. `*_mcu` says which `[mcu]` the
//! port's hardware sits on (the bridge). For a UART port, `uart_oid` is the
//! name the host knows the port by, and `uart_baud` the rate the host opens its
//! stream at — the firmware reports neither, because the host creates the port
//! at configuration time instead: [`load_uart_device`] queues the
//! `config_uart` / `config_bridge_port` pair on the bridge's builder and takes
//! the port's **index** (its declaration order among the bridge's ports) from
//! the bridge — the same number `config_uart` travels with, and the same one
//! the host names the port's tty with. Grouping ports with the `[mcu]` sections that reference them, and
//! everything transport-shaped, is later work — this module only parses each
//! port section once and holds the answers.
//!
//! # Options
//!
//! | option | meaning |
//! |---|---|
//! | `uart_mcu` / `can_mcu` | the `[mcu]` this port sits on; a section name, `mcu` for the bare `[mcu]` (required) |
//! | `uart_oid` / `can_oid` | the name the host knows this port by (required, non-empty) |
//! | `uart_baud` | the port's line rate, a positive integer (optional; `250000` when absent) |
//! | `uart_path` | the tty the bridge's **host library** opens for this port, a non-empty string (optional) |
//! | `uart_restart_method` | the **line-level** reset the bridge gives a legacy device on this port — `arduino` (DTR) or `cheetah` (RTS/DTR); any other value is a config error (optional) |
//!
//! `uart_path` is only meaningful on a port whose bridge is a host library
//! (`[mcu] host_library:`): that library is what opens the tty, at run time and
//! per port, with the path the host names before the firmware comes up
//! (`third_party/klipper/src/host/klipper_host.h`, `klipper_host_port`). It is
//! the deployment fact a host moves between machines without rebuilding the
//! library; a bridge that is not a host library opens no such stream, so writing
//! one there is a config error ([`wire_bridge_port`]). A port that writes none
//! keeps the path the library was built with.
//!
//! The `*_mcu` target is a **config-section** existence check
//! ([`ConfigWrapper::has_sibling`]), not a printer-object lookup: the section
//! is what the loader hands later phases, and checking the config keeps this
//! pass independent of load order. `has_sibling` records nothing — the
//! `[mcu]` section's own factory is what marks *its* options read — while this
//! section's own options go through the tracked getters, so `check_unused`
//! sees exactly what this factory read and nothing else.
//!
//! Options this module does not know are not read here: they are left for
//! `check_unused` to report as invalid, the same contract every factory has.
//!
//! Each section registers a light description object under its own identifier
//! (`uart_device uart0`): no status, not queryable, just the parsed values for
//! the passes that follow — the same shape `serial_ports` and `mcu_ids` set.

use std::collections::BTreeSet;
use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::cmd::uart::{ConfigBridgePort, ConfigUart, DEFAULT_UART_BAUD};
use crate::core::klippy::config::mcu::Transport;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::{McuObject, McuRestartMethod};

// The port sections describe hardware on an `[mcu]`, so their order only has to
// sit after `[mcu]`'s 10 — the factory reads the bridge's `McuObject` (already
// registered by then) to queue its configuration commands and take its port
// index. 19 is the first free slot before the extras block at 20 (15/16/18 are
// taken by `multi_pin`/`sx1509`/`force_move`).
section!("uart_device", order = 19, prefix = load_uart_device);
section!("can_device", order = 19, prefix = load_can_device);

/// One configured `[uart_device <name>]`: a bridge downstream UART port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UartPort {
    /// The section's sub — the name other sections reference the port by.
    name: String,
    /// The `[mcu]` section the port sits on: `mcu` for the bare section, the
    /// sub otherwise (the same naming `get_printer_mcu` uses upstream).
    mcu: String,
    /// The oid name the firmware reports for this port.
    oid: String,
    /// The port's line rate, when the config writes one.
    baud: Option<u32>,
    /// The tty the bridge's host library opens for this port, when the config
    /// writes one. Meaningless on a bridge that is not a host library, which
    /// the loader refuses ([`wire_bridge_port`]).
    path: Option<String>,
    /// The line-level reset the bridge gives a legacy device on this port, when
    /// the config writes one.
    restart_method: Option<McuRestartMethod>,
    /// The port's index on its bridge: its declaration order among the bridge's
    /// ports, which is what the port-level `SET_PORT_NODEID` travels with — and
    /// the slot `config_uart` carries.
    ///
    /// `None` until the loader's factory has wired the port to its bridge
    /// ([`load_uart_device`], which queues the `config_uart` /
    /// `config_bridge_port` pair and takes the next index from the bridge); a
    /// section parsed on its own ([`UartPort::new`], in tests) has none.
    index: Option<u8>,
}

impl UartPort {
    /// The line-level resets this option may ask for.
    ///
    /// Only `arduino` (DTR) and `cheetah` (RTS/DTR) act on the port's own
    /// lines. `command` resets a firmware the host has a session with — a port
    /// has no session of its own — `rpi_usb` cuts a USB power rail a port does
    /// not own, and `admin` needs an admin channel: the device this option
    /// describes is the **legacy** one, which has none.
    const LINE_RESETS: &'static [&'static str] = &["arduino", "cheetah"];

    /// Parse `uart_restart_method`, refusing everything that is not line-level.
    ///
    /// # Errors
    /// A value that is not a line-level method — including a misspelling, which
    /// is reported with the two that are accepted.
    fn parse_restart_method(
        config: &ConfigWrapper,
        text: &str,
    ) -> Result<McuRestartMethod, ConfigError> {
        McuRestartMethod::parse(text)
            .filter(|method| {
                matches!(
                    method,
                    McuRestartMethod::Arduino | McuRestartMethod::Cheetah
                )
            })
            .ok_or_else(|| {
                ConfigError::new(format!(
                    "Section '{}' has an invalid uart_restart_method: '{text}' (expected one of {})",
                    config.identifier(),
                    Self::LINE_RESETS.join(", ")
                ))
            })
    }

    /// Parse one `[uart_device <name>]` section.
    ///
    /// # Errors
    /// A missing or empty required option, an `uart_mcu` that names no `[mcu]`
    /// section, a non-positive or unparsable `uart_baud`, an empty
    /// `uart_path`, or a `uart_restart_method` that is not a line-level method —
    /// each message names the section (and the target, for an unknown MCU).
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let name = section_name(config, "uart_device")?;
        let mcu = require_mcu(config, "uart_mcu")?;
        let oid = require_text(config, "uart_oid")?;
        let baud = parse_baud(config, "uart_baud")?;
        let path = parse_path(config)?;
        let restart_method = match config.get_str("uart_restart_method") {
            None => None,
            Some(text) => Some(Self::parse_restart_method(config, &text)?),
        };
        Ok(Self {
            name,
            mcu,
            oid,
            baud,
            path,
            restart_method,
            index: None,
        })
    }

    /// The name other sections reference this port by (the section's sub).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The `[mcu]` section this port sits on: `mcu` for the bare section, the
    /// sub otherwise.
    pub fn mcu(&self) -> &str {
        &self.mcu
    }

    /// The oid name the firmware reports for this port.
    pub fn oid(&self) -> &str {
        &self.oid
    }

    /// The port's line rate, when the config writes one.
    pub fn baud(&self) -> Option<u32> {
        self.baud
    }

    /// The tty the bridge's host library opens for this port, when the config
    /// writes `uart_path:`.
    pub fn path(&self) -> Option<&str> {
        self.path.as_deref()
    }

    /// The line-level reset the bridge gives a legacy device on this port,
    /// when the config writes one.
    pub fn restart_method(&self) -> Option<&McuRestartMethod> {
        self.restart_method.as_ref()
    }

    /// The port's index on its bridge — its declaration order among the
    /// bridge's ports, which is what the port-level `SET_PORT_NODEID` carries.
    ///
    /// `None` for a section parsed outside the loader (see the field); a port
    /// the loader built always has one.
    pub fn index(&self) -> Option<u8> {
        self.index
    }

    /// Record the index the bridge's port counter handed out.
    pub(crate) fn set_index(&mut self, index: u8) {
        self.index = Some(index);
    }
}

impl PrinterObject for UartPort {
    /// No status: this object carries parsed config, not runtime state.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Not client-visible, like `serial_ports` and `mcu_ids`.
    fn is_queryable(&self) -> bool {
        false
    }
}

/// One configured `[can_device <name>]`: a bridge downstream CAN port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanPort {
    /// The section's sub — the name other sections reference the port by.
    name: String,
    /// The `[mcu]` section the port sits on ([`UartPort::mcu`]'s naming).
    mcu: String,
    /// The oid name the firmware reports for this port.
    oid: String,
}

impl CanPort {
    /// Parse one `[can_device <name>]` section.
    ///
    /// # Errors
    /// A missing or empty required option, or a `can_mcu` that names no `[mcu]`
    /// section — each message names the section (and the target, for an
    /// unknown MCU).
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let name = section_name(config, "can_device")?;
        let mcu = require_mcu(config, "can_mcu")?;
        let oid = require_text(config, "can_oid")?;
        Ok(Self { name, mcu, oid })
    }

    /// The name other sections reference this port by (the section's sub).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The `[mcu]` section this port sits on: `mcu` for the bare section, the
    /// sub otherwise.
    pub fn mcu(&self) -> &str {
        &self.mcu
    }

    /// The oid name the firmware reports for this port.
    pub fn oid(&self) -> &str {
        &self.oid
    }
}

impl PrinterObject for CanPort {
    /// No status: this object carries parsed config, not runtime state.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Not client-visible, like `serial_ports` and `mcu_ids`.
    fn is_queryable(&self) -> bool {
        false
    }
}

/// Upstream's `load_config_prefix` shape for `[uart_device <name>]`.
///
/// # Errors
/// As [`UartPort::new`], plus the bridge's own configuration failing to take
/// this port ([`wire_bridge_port`]).
pub fn load_uart_device(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let mut port = UartPort::new(config)?;
    wire_bridge_port(config, printer, &mut port)?;
    Ok(Arc::new(port))
}

/// Upstream's `load_config_prefix` shape for `[can_device <name>]`.
///
/// # Errors
/// As [`CanPort::new`].
pub fn load_can_device(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(CanPort::new(config)?))
}

/// Queue one UART port's firmware configuration on its bridge, and take the
/// port's index from the bridge.
///
/// The firmware has no port of its own to report: the host creates it. So this
/// queues the pair `cmd::uart` defines on the **bridge's** builder, in order —
/// `config_uart` creates the byte stream at the section's rate (`250000` when
/// the section wrote none) on the port's slot, and `config_bridge_port` hands it
/// to the router as a downstream port — and takes the next port index from the
/// bridge. Both commands and the index have to agree: the bridge numbers ports
/// by the order its `config_bridge_port` commands arrive, so the
/// section-declaration order the loader calls this in is the port's index.
///
/// The index is also the **slot** the port's tty was named with, so the two are
/// one number rather than two counters: `config_uart` carries it, and a
/// host-library bridge is told `(slot, uart_path)` before the library comes up
/// ([`McuObject::add_host_port_path`], read back by
/// `McuObject::connect`).
///
/// The `[mcu]` a port named is a config-section existence check in
/// [`require_mcu`]; the object lookup here is the next step — the builder the
/// commands go on — and can only fail if the section is not there, which the
/// loader guarantees for a config it accepted.
///
/// # Errors
/// [`ConfigError`] when the bridge's object cannot be found, when the port
/// writes a `uart_path:` its bridge cannot open (a bridge that is not a host
/// library), or when the bridge's builder refuses another command or port (oid
/// exhaustion, or a bridge already at its port limit).
fn wire_bridge_port(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
    port: &mut UartPort,
) -> Result<(), ConfigError> {
    let bridge_name = port.mcu().to_string();
    let identifier = if bridge_name == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {bridge_name}")
    };
    let bridge = printer
        .lookup_object_as::<McuObject>(&identifier)
        .ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{}': unknown MCU '{bridge_name}'",
                config.identifier()
            ))
        })?;
    // `uart_path:` names the tty the bridge's host library opens for this port,
    // and only a host-library bridge has a library that opens one. Anywhere else
    // the option would sit there unfollowed, so it is refused by name instead.
    if port.path().is_some() && !bridge_opens_its_ports(&bridge) {
        return Err(ConfigError::new(format!(
            "Section '{}': uart_path: is only meaningful for a port whose bridge is a host \
             library ([mcu] host_library:), because only that library opens the port's tty; \
             '{bridge_name}' is not one",
            config.identifier()
        )));
    }
    let builder = bridge.config();
    let on = |err: crate::core::klippy::mcu::McuError| {
        ConfigError::new(format!(
            "Section '{}': MCU '{bridge_name}' cannot take the port: {err}",
            config.identifier()
        ))
    };
    let uart_oid = builder.create_oid().map_err(on)?;
    let port_oid = builder.create_oid().map_err(on)?;
    // The port's index is its declaration order among the bridge's ports
    // (`McuObject::next_bridge_port_index`); it is taken before the commands are
    // queued because `config_uart` carries it as the port's slot.
    let index = bridge.next_bridge_port_index(&config.identifier())?;
    port.set_index(index);
    if let Some(path) = port.path() {
        bridge.add_host_port_path(index, path.to_string());
    }
    let baud = port.baud().unwrap_or(DEFAULT_UART_BAUD);
    builder
        .add_config_cmd(&ConfigUart {
            oid: uart_oid,
            baud,
            slot: index,
        })
        .map_err(on)?;
    builder
        .add_config_cmd(&ConfigBridgePort {
            oid: port_oid,
            uart: uart_oid,
        })
        .map_err(on)?;
    Ok(())
}

/// Whether `bridge` opens its ports itself: its own transport is a host library,
/// the only bridge that reads a port's `uart_path:` at run time.
///
/// A section parsed outside the loader carries no transport description
/// ([`McuObject::parsed_config`]), and such a bridge cannot be a host library
/// either — the loader never built it — so it answers `false`.
fn bridge_opens_its_ports(bridge: &McuObject) -> bool {
    matches!(
        bridge.parsed_config().map(|config| config.transport),
        Some(Transport::Host { .. })
    )
}

/// The `[mcu]` **names** the config makes a bridge: every port section's
/// `*_mcu` value, trimmed (`mcu` for the bare section, the sub otherwise).
///
/// A bridge is the upstream side of a port, so its own link carries the
/// downstream devices' frames too; the link passes ask this before deciding
/// whether that link is addressed by id (`serial_ports`, `host_libraries`).
/// Like every pre-scan here it reads raw option text (`section().get_text`,
/// never a tracked getter), so it records no access and does not exempt a
/// `[mcu]` option from `check_unused` on its own account. Whether a `*_mcu`
/// names an existing `[mcu]` section is the port factory's check; an unknown
/// name is still reported here, so a bridge the config never declares cannot
/// make a path id-addressed by accident.
pub(crate) fn bridge_mcus(config: &ConfigWrapper) -> BTreeSet<String> {
    let mut bridges = BTreeSet::new();
    for (prefix, option) in [("uart_device", "uart_mcu"), ("can_device", "can_mcu")] {
        for section in config.sibling_prefix_sections(prefix) {
            if let Some(name) = section.section().get_text(option) {
                let name = name.trim();
                if !name.is_empty() {
                    bridges.insert(name.to_string());
                }
            }
        }
    }
    bridges
}

/// The section's sub, required: a port is `[<type> <name>]`, never bare.
///
/// The loader only calls a prefix factory for a section that has a sub, so
/// this guards direct calls (tests, later passes reading a stored section).
fn section_name(config: &ConfigWrapper, kind: &str) -> Result<String, ConfigError> {
    config.section().sub.clone().ok_or_else(|| {
        ConfigError::new(format!(
            "Section '{}' must be a '[{kind} <name>]' section",
            config.identifier()
        ))
    })
}

/// `uart_path`, the tty this port's bridge opens for it, when the config writes
/// one.
///
/// Optional: a port on a bridge that is not a host library has no such stream to
/// name (and writing one is a config error, [`wire_bridge_port`]), and a
/// host-library port that writes none keeps the path the library was built with.
///
/// # Errors
/// An `uart_path` that is written but empty — a path is named or the option is
/// left out, never blank.
fn parse_path(config: &ConfigWrapper) -> Result<Option<String>, ConfigError> {
    let Some(text) = config.get_str("uart_path") else {
        return Ok(None);
    };
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Err(ConfigError::new(format!(
            "Option 'uart_path' in section '{}' must not be empty",
            config.identifier()
        )));
    }
    Ok(Some(trimmed.to_string()))
}

/// A required, non-empty string option, read through the tracked getter.
fn require_text(config: &ConfigWrapper, option: &str) -> Result<String, ConfigError> {
    let text = config.get(option, None)?;
    let trimmed = text.trim().to_string();
    if trimmed.is_empty() {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{}' must be specified",
            config.identifier()
        )));
    }
    Ok(trimmed)
}

/// A required `*_mcu` option naming an existing `[mcu]` section.
///
/// The bare `[mcu]` is written `mcu` (upstream names it that way,
/// `klippy/mcu.py:1151-1153`); a `[mcu <name>]` by its sub. Existence is a
/// config-section check (`has_sibling`, which records nothing) — the section
/// is the contract later passes consume, and the `[mcu]` factories own marking
/// their own options read.
fn require_mcu(config: &ConfigWrapper, option: &str) -> Result<String, ConfigError> {
    let mcu = require_text(config, option)?;
    let identifier = if mcu == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {mcu}")
    };
    if !config.has_sibling(&identifier) {
        return Err(ConfigError::new(format!(
            "Section '{}': unknown MCU '{mcu}'",
            config.identifier()
        )));
    }
    Ok(mcu)
}

/// An optional positive-integer option (`uart_baud`).
///
/// Absent is `None` (the port inherits whatever the later transport pass
/// decides); written must parse as a positive integer, the same rule
/// `config/mcu.rs` applies to an MCU's `baud`.
fn parse_baud(config: &ConfigWrapper, option: &str) -> Result<Option<u32>, ConfigError> {
    let Some(text) = config.get_str(option) else {
        return Ok(None);
    };
    match text.trim().parse::<u32>() {
        Ok(baud) if baud > 0 => Ok(Some(baud)),
        _ => Err(ConfigError::new(format!(
            "Section '{}' has an invalid {option}: '{text}' (expected a positive integer)",
            config.identifier()
        ))),
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    /// Parse `text` into a config the wrappers can reach siblings through.
    fn parse(text: &str) -> Config {
        Config::from_text(text).expect("the test config parses").0
    }

    /// A tracked wrapper over `identifier`, with the whole config behind it so
    /// `has_sibling` answers — the same shape the loader hands a factory.
    fn wrap<'a>(config: &'a Config, identifier: &str) -> ConfigWrapper<'a> {
        let section = config
            .get_section(identifier)
            .unwrap_or_else(|| panic!("the config has [{identifier}]"));
        ConfigWrapper::with_config(section, AccessTracking::shared(), None, config)
    }

    /// A config with a bare `[mcu]` plus `section_text` as a port section.
    fn with_mcu(section_text: &str) -> Config {
        parse(&format!("[mcu]\nserial: /dev/a\n{section_text}"))
    }

    #[test]
    fn a_uart_port_parses_every_option() {
        let config = with_mcu(
            "[uart_device uart0]\n\
             uart_mcu: mcu\n\
             uart_oid: uart0\n\
             uart_baud: 250000\n\
             uart_restart_method: arduino\n",
        );
        let port = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap();

        assert_eq!(port.name(), "uart0");
        assert_eq!(port.mcu(), "mcu");
        assert_eq!(port.oid(), "uart0");
        assert_eq!(port.baud(), Some(250_000));
        assert_eq!(port.restart_method(), Some(&McuRestartMethod::Arduino));
    }

    #[test]
    fn a_uart_port_without_optional_options_parses() {
        let config = with_mcu("[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\n");
        let port = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap();

        assert_eq!(port.baud(), None);
        assert_eq!(port.restart_method(), None);
    }

    #[test]
    fn a_uart_port_may_name_a_prefixed_mcu() {
        let config = parse(
            "[mcu bridge]\nserial: /dev/a\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        );
        let port = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap();

        assert_eq!(port.mcu(), "bridge");
    }

    #[test]
    fn a_uart_port_without_uart_mcu_is_refused() {
        let config = with_mcu("[uart_device uart0]\nuart_oid: uart0\n");

        let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'uart_mcu' in section 'uart_device uart0' must be specified"
        );
    }

    #[test]
    fn a_uart_port_naming_an_unknown_mcu_is_refused() {
        let config = with_mcu("[uart_device uart0]\nuart_mcu: zboard\nuart_oid: uart0\n");

        let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Section 'uart_device uart0': unknown MCU 'zboard'"
        );
    }

    #[test]
    fn a_uart_port_without_uart_oid_is_refused() {
        let config = with_mcu("[uart_device uart0]\nuart_mcu: mcu\n");

        let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'uart_oid' in section 'uart_device uart0' must be specified"
        );
    }

    #[test]
    fn a_uart_port_with_an_invalid_baud_is_refused() {
        let config =
            with_mcu("[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\nuart_baud: fast\n");
        let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();
        assert!(err.to_string().contains("invalid uart_baud"), "{err}");
        assert!(err.to_string().contains("fast"), "{err}");

        // A rate that parses but is not positive is the same refusal.
        let config =
            with_mcu("[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\nuart_baud: 0\n");
        let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();
        assert!(err.to_string().contains("invalid uart_baud"), "{err}");
    }

    #[test]
    fn a_uart_port_with_an_invalid_restart_method_is_refused() {
        let config = with_mcu(
            "[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\n\
             uart_restart_method: wibble\n",
        );

        let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();

        assert!(
            err.to_string().contains("invalid uart_restart_method"),
            "{err}"
        );
        assert!(err.to_string().contains("wibble"), "{err}");
    }

    /// Only the port's own lines can be driven: `command` resets a firmware the
    /// host has a session with (a port has none), `rpi_usb` cuts a USB power
    /// rail, and `admin` needs an admin channel a legacy device does not have.
    #[test]
    fn a_uart_port_refuses_a_restart_method_that_is_not_line_level() {
        for value in ["command", "admin", "rpi_usb"] {
            let config = with_mcu(&format!(
                "[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\n\
                 uart_restart_method: {value}\n"
            ));

            let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();

            assert!(
                err.to_string().contains("invalid uart_restart_method"),
                "{value}: {err}"
            );
            assert!(err.to_string().contains(value), "{value}: {err}");
            // The refusal lists what is accepted, so the config can be fixed.
            assert!(
                err.to_string().contains("arduino, cheetah"),
                "{value}: {err}"
            );
        }
    }

    /// The other line-level method is accepted: `cheetah` is the RTS/DTR
    /// sequence, the second thing a bridge can do to a port's lines.
    #[test]
    fn a_uart_port_takes_a_cheetah_restart_method() {
        let config = with_mcu(
            "[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\n\
             uart_restart_method: cheetah\n",
        );
        let port = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap();

        assert_eq!(port.restart_method(), Some(&McuRestartMethod::Cheetah));
    }

    #[test]
    fn a_can_port_parses_and_its_options_are_required() {
        let config = with_mcu("[can_device can0]\ncan_mcu: mcu\ncan_oid: can0\n");
        let port = CanPort::new(&wrap(&config, "can_device can0")).unwrap();
        assert_eq!(port.name(), "can0");
        assert_eq!(port.mcu(), "mcu");
        assert_eq!(port.oid(), "can0");

        let config = with_mcu("[can_device can0]\ncan_oid: can0\n");
        let err = CanPort::new(&wrap(&config, "can_device can0")).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'can_mcu' in section 'can_device can0' must be specified"
        );

        let config = with_mcu("[can_device can0]\ncan_mcu: mcu\n");
        let err = CanPort::new(&wrap(&config, "can_device can0")).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'can_oid' in section 'can_device can0' must be specified"
        );

        let config = with_mcu("[can_device can0]\ncan_mcu: zboard\ncan_oid: can0\n");
        let err = CanPort::new(&wrap(&config, "can_device can0")).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Section 'can_device can0': unknown MCU 'zboard'"
        );
    }

    #[test]
    fn the_port_sections_load_and_hold_no_unused_options() {
        // The end-to-end contract: a config with an `[mcu]` and both port
        // sections loads clean — every port option this factory reads is
        // recorded, so `check_unused` accepts the sections, and each section
        // registers its description object under its own identifier.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = parse(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\nuart_baud: 250000\n\
             [can_device can0]\ncan_mcu: mcu\ncan_oid: can0\n",
        );

        printer.load_config(&config).unwrap();

        let uart = printer
            .lookup_object_as::<UartPort>("uart_device uart0")
            .expect("the uart port registered");
        assert_eq!(uart.oid(), "uart0");
        assert_eq!(
            uart.index(),
            Some(0),
            "the loader assigned the port its declaration order"
        );
        let can = printer
            .lookup_object_as::<CanPort>("can_device can0")
            .expect("the can port registered");
        assert_eq!(can.oid(), "can0");
    }

    /// Load `text` through the real loader — the same walk a config file takes,
    /// so the port factory's bridge wiring runs.
    fn load(text: &str) -> Result<Arc<Printer>, ConfigError> {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config)?;
        Ok(printer)
    }

    /// The `(slot, path)` pairs the `[mcu <name>]` bridge collected from its
    /// ports, in declaration order.
    fn host_port_paths(printer: &Arc<Printer>, name: &str) -> Vec<(u8, String)> {
        printer
            .lookup_object_as::<McuObject>(name)
            .expect("the bridge registered")
            .host_port_paths()
    }

    #[test]
    fn a_uart_port_takes_a_path() {
        let config = with_mcu(
            "[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\n\
             uart_path:  /dev/serial/by-id/usb-board\n",
        );

        let port = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap();

        assert_eq!(port.path(), Some("/dev/serial/by-id/usb-board"));
    }

    #[test]
    fn a_uart_port_without_a_path_has_none() {
        let config = with_mcu("[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\n");

        let port = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap();

        assert_eq!(port.path(), None);
    }

    #[test]
    fn an_empty_uart_path_is_refused() {
        let config = with_mcu("[uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\nuart_path: \n");

        let err = UartPort::new(&wrap(&config, "uart_device uart0")).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'uart_path' in section 'uart_device uart0' must not be empty"
        );
    }

    /// `uart_path:` names the tty the bridge's **host library** opens, so a
    /// bridge that is not one has nothing to open it with: the load fails,
    /// naming the section and the bridge, instead of leaving the option read and
    /// unfollowed.
    #[test]
    fn uart_path_on_a_bridge_that_is_not_a_host_library_is_refused() {
        let err = load(
            "[mcu bridge]\nserial: /dev/not-opened\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             uart_path: /dev/serial/by-id/usb-board\n",
        )
        .err()
        .expect("a serial bridge cannot open a port's tty from the host's word");

        assert_eq!(
            err.to_string(),
            "Section 'uart_device uart0': uart_path: is only meaningful for a port whose \
             bridge is a host library ([mcu] host_library:), because only that library opens \
             the port's tty; 'bridge' is not one"
        );
    }

    /// The port index is the number both the port's tty and `config_uart` travel
    /// with: the loader hands the bridge one pair per `uart_path:` port, keyed by
    /// the index its port was given — the second port's slot is 1, not 0, and a
    /// port that writes no path contributes nothing.
    #[test]
    fn a_host_library_bridge_registers_its_ports_paths_under_their_index() {
        let printer = load(
            "[mcu bridge]\nhost_library: /lib/libklipper_host.so\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [uart_device uart1]\nuart_mcu: bridge\nuart_oid: uart1\n\
             uart_path: /dev/serial/by-id/usb-board\n",
        )
        .expect("a host-library bridge takes the port's path");

        assert_eq!(
            host_port_paths(&printer, "mcu bridge"),
            vec![(1, "/dev/serial/by-id/usb-board".to_string())]
        );
        // The two counters are one: the port's index — which the port-level
        // `SET_PORT_NODEID` carries — is what the path is keyed by.
        assert_eq!(
            printer
                .lookup_object_as::<UartPort>("uart_device uart1")
                .expect("the port registered")
                .index(),
            Some(1)
        );
    }

    /// A host-library bridge with no `uart_path:` on any port collects nothing:
    /// every port keeps the tty the library was built with.
    #[test]
    fn a_host_library_bridge_without_port_paths_registers_none() {
        let printer = load(
            "[mcu bridge]\nhost_library: /lib/libklipper_host.so\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .expect("a host-library bridge loads without a path");

        assert!(host_port_paths(&printer, "mcu bridge").is_empty());
    }
}
