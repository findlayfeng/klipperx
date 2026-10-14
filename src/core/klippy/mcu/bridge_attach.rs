//! Attach a device that hangs on a **bridge downstream port**: a `[mcu]` whose
//! transport is `uart:` / `can:` naming a `[uart_device]` / `[can_device]`
//! section (M1's port sections, [`bridge_ports`](super::bridge_ports)).
//!
//! Such a device is not reached by opening a transport of its own. Its frames
//! travel the **bridge's** link, addressed by the device's own mcu id, and the
//! bridge is told which port that id belongs to with the port-level
//! `SET_PORT_NODEID` unit (`interface/devices::serial::set_port_node_id`). The
//! three steps are:
//!
//! 1. [`resolve`] — from the child's parsed transport to the bridge's link: who
//!    the bridge is, which of its firmware ports the section names, the port's
//!    `index` on that bridge, and the `flags` byte the port's shape asks for.
//!    Everything that needs the firmware's identify answer — the port table — is
//!    checked here, after the bridge has connected (`mcu/object.rs` awaits
//!    objects in registration order, so the bridge is up when the child is).
//! 2. [`open`] — open a **downstream** handle on the bridge's own link, with no
//!    handshake (a device behind a port has no admin channel of its own), and
//!    send `SET_PORT_NODEID(index, id, flags)` on it before anything else.
//! 3. The caller (`Mcu::connect`) then speaks to the handle exactly as it would
//!    to any MCU's transport.
//!
//! Only the **legacy** shape lands here: one `[mcu]` on a uart port writing no
//! `uuid:`, spoken to in raw frames. A named device (a `uuid:` on the child, or
//! any CAN port) needs the downstream handshake, which lands next; [`resolve`]
//! refuses it rather than silently treating it as legacy.
//!
//! The bridge's own transport decides how the downstream handle is opened, and
//! the two kinds it can be are exactly the two mux links: a shared serial port
//! ([`serial::attach_downstream`]) or a host library
//! ([`HostDevice::load_downstream`]). A bridge reached over CAN carries its
//! downstream frames inside the CAN payload (M5/P5), so it is refused here.

use super::bridge_ports::{CanPort, UartPort};
use super::host_libraries::{HostLibraries, HOST_LIBRARIES_OBJECT};
use super::port_grouping::{PortGroups, PortKind, PORT_GROUPS_OBJECT};
use super::{Dictionary, McuObject};
use crate::core::klippy::config::mcu::{McuConfig, Transport};
use crate::core::klippy::config::ConfigError;
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::interface::devices::host::{HostDevice, HostMux};
use crate::core::klippy::interface::devices::serial::{self, MUX_PORT_UNITS};
use crate::core::klippy::interface::Interface;
use crate::core::klippy::printer::Printer;
use tracing::debug;

/// Where a downstream device attaches, resolved from the child's config and the
/// bridge's identify answer.
///
/// Built by [`resolve`]; [`open`] turns it into a live [`Interface`]. The
/// `transport` is the **bridge's**, not the child's: the child has none of its
/// own to open.
#[derive(Debug)]
pub(crate) struct BridgePortLink {
    /// The bridge's own transport, as its section describes it (with the
    /// load-time shared-baud / host-mux stamps covered back on, `McuObject::mcu_config`).
    pub transport: Transport,
    /// The port's index in the bridge's firmware port table — what the
    /// port-level command travels with.
    pub index: u8,
    /// The port-level `SET_PORT_NODEID` flags byte: `0` for a legacy port, whose
    /// device is spoken to in raw frames; [`MUX_PORT_UNITS`] for a port whose
    /// device speaks units.
    pub flags: u8,
    /// Whether the port is legacy ([`PortGroups::is_legacy`]): one uart-port
    /// device with no `uuid:`. The same shape [`BridgePortLink::flags`] reports
    /// as `0`, kept as its own answer because it is the shape this pass supports.
    pub legacy: bool,
}

/// Resolve the child's `uart:`/`can:` transport to the bridge port it names.
///
/// The port table is the **bridge's** identify answer, so this only succeeds
/// once the bridge has connected. Bring-up connects objects in registration
/// order, which is config-file order within a phase, so a bridge declared before
/// its child is up by the time the child asks; a bridge declared after its child
/// is not waited for — the error says to reorder the file.
///
/// # Errors
/// [`KlippyError::Config`] for everything here: a named (uuid-bearing) device
/// this pass does not attach, a bridge that has not connected, and the three
/// port-table failures — no table, no port with that oid, or the port's type not
/// matching the section's.
pub(crate) fn resolve(
    printer: &Printer,
    config: &McuConfig,
) -> Result<BridgePortLink, KlippyError> {
    let Transport::BridgePort { port, uuid, kind } = &config.transport else {
        return Err(config_error(format!(
            "MCU '{}': not a bridge-port transport to resolve",
            config.name
        )));
    };
    // Only the legacy shape attaches here: a device with no admin channel, whose
    // frames the bridge strips and heads. A named device answers the mux itself
    // (SET_KLIPPER_NODEID + re-confirm), and CAN addressing is by uuid, so the
    // handshake is not optional for either.
    if uuid.is_some() {
        return Err(config_error(format!(
            "MCU '{}': a 'uuid:' names the device behind port '{port}'; the downstream handshake \
             lands next, so this build attaches a legacy (uuid-less) port only",
            config.name
        )));
    }

    // The port section object: the oid the firmware names it by, and the `[mcu]`
    // it says is the bridge.
    let (oid, bridge) = match kind {
        PortKind::Uart => {
            let section = printer.lookup_object_as::<UartPort>(port).ok_or_else(|| {
                config_error(format!("MCU '{}': no port section '{port}'", config.name))
            })?;
            (section.oid().to_string(), section.mcu().to_string())
        }
        PortKind::Can => {
            let section = printer.lookup_object_as::<CanPort>(port).ok_or_else(|| {
                config_error(format!("MCU '{}': no port section '{port}'", config.name))
            })?;
            (section.oid().to_string(), section.mcu().to_string())
        }
    };

    // The bridge object, registered under the same spelling the port section
    // writes: `mcu` for the bare section, `mcu <sub>` otherwise.
    let bridge_name = if bridge == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {bridge}")
    };
    let not_up = || {
        config_error(format!(
            "MCU '{}' hangs on a port of '{bridge}'; declare the bridge before it in the config \
             file",
            config.name
        ))
    };
    let bridge_object = printer
        .lookup_object_as::<McuObject>(&bridge_name)
        .ok_or_else(not_up)?;
    let Some(bridge_mcu) = bridge_object.mcu() else {
        return Err(not_up());
    };
    // The bridge's transport, for `open` to reach the downstream device through.
    let bridge_config = bridge_object.mcu_config()?;
    let dictionary = bridge_mcu.dictionary().ok_or_else(|| {
        config_error(format!(
            "MCU '{}': bridge '{bridge}' reports no port table (old firmware, or not a bridge \
             build)",
            config.name
        ))
    })?;
    let index = bridge_port_index(&dictionary, &oid, *kind, &config.name, &bridge)?;

    let groups = printer
        .lookup_object_as::<PortGroups>(PORT_GROUPS_OBJECT)
        .ok_or_else(|| {
            config_error(format!(
                "MCU '{}': the port grouping is not loaded",
                config.name
            ))
        })?;
    let (flags, legacy) = port_flags(&groups, port);
    Ok(BridgePortLink {
        transport: bridge_config.transport,
        index,
        flags,
        legacy,
    })
}

/// Open a downstream handle on `link`'s bridge link and establish its route.
///
/// The handle is the bridge's own transport reopened for **one downstream
/// device**: a second reference on the same shared serial port or host library,
/// addressed by `mcu_id`, with no handshake. The route is sent before anything
/// else the caller writes, because until the bridge has it, `mcu_id` names no
/// port and frames under it go nowhere.
///
/// # Errors
/// [`KlippyError::Config`] for a bridge link that cannot carry a downstream
/// device (a CAN upstream, or a transport with no mux); [`KlippyError::Connection`]
/// for a failed open or a failed `SET_PORT_NODEID`.
pub(crate) fn open(
    printer: &Printer,
    link: &BridgePortLink,
    mcu_id: u32,
) -> Result<Interface, KlippyError> {
    let interface = match &link.transport {
        Transport::Host { library, .. } => {
            // The downstream reference is addressed by the path's mux identity,
            // the same one the bridge itself attached with; the named uuid is
            // not used for a device behind a port (`HostDevice::load_downstream`).
            let libraries = printer
                .lookup_object_as::<HostLibraries>(HOST_LIBRARIES_OBJECT)
                .ok_or_else(|| {
                    config_error(format!(
                        "host_library {library}: a downstream device needs the path grouping, \
                         which is not loaded"
                    ))
                })?;
            let identity = libraries.identity(library).ok_or_else(|| {
                config_error(format!(
                    "host_library {library}: no [mcu] names this path, so it has no mux identity"
                ))
            })?;
            let device = HostDevice::load_downstream(
                library,
                HostMux {
                    identity,
                    named: identity,
                },
                mcu_id,
            )
            .map_err(|err| {
                KlippyError::Connection(format!(
                    "host_library {library}: downstream attach failed: {err}"
                ))
            })?;
            Interface::from_host(device)
        }
        Transport::Serial { path, baud, .. } => {
            let handle = serial::attach_downstream(path, *baud, mcu_id).map_err(|err| {
                KlippyError::Connection(format!("serial {path}: downstream attach failed: {err}"))
            })?;
            Interface::shared_serial(handle)
        }
        Transport::Can { .. } => {
            return Err(config_error(
                "a bridge reached over CAN cannot carry a downstream device yet: the CAN upstream \
                 attach is M5/P5"
                    .to_string(),
            ));
        }
        other => {
            return Err(config_error(format!(
                "a bridge on {other:?} cannot carry a downstream device"
            )));
        }
    };
    debug!(
        "bridge port {}: attaching downstream device id {mcu_id} ({})",
        link.index,
        if link.legacy { "legacy" } else { "units" }
    );
    let id = u8::try_from(mcu_id).map_err(|_| {
        KlippyError::Connection(format!(
            "MCU id {mcu_id} is outside the range a bridge port can address (1..=255)"
        ))
    })?;
    interface
        .send_set_port_node_id(link.index, id, link.flags)
        .map_err(|err| {
            KlippyError::Connection(format!(
                "sending SET_PORT_NODEID({}, {id}, {}) to the bridge failed: {err}",
                link.index, link.flags
            ))
        })?;
    Ok(interface)
}

/// Look up the child's oid in the bridge's firmware port table, returning its
/// `index`.
///
/// The table is the four identifying keys the bridge build reports
/// `BRIDGE_PORT_COUNT`, then `BRIDGE_PORT<i>_NAME` / `_TYPE` / `_PATH` per port
/// (`third_party/klipper/src/bridge.c`). A firmware without the count is not a
/// bridge build (or predates the table); a table without the oid names what it
/// does have; and the type must match the section's kind — an oid reached over
/// the wrong port kind would carry the wrong wire format.
///
/// # Errors
/// [`KlippyError::Config`] naming the child, the bridge and the mismatch.
fn bridge_port_index(
    dictionary: &Dictionary,
    oid: &str,
    kind: PortKind,
    child: &str,
    bridge_name: &str,
) -> Result<u8, KlippyError> {
    let Some(count) = dictionary.constant_f64("BRIDGE_PORT_COUNT") else {
        return Err(config_error(format!(
            "MCU '{child}': bridge '{bridge_name}' reports no port table (old firmware, or not a \
             bridge build)"
        )));
    };
    let mut names: Vec<String> = Vec::new();
    let mut found: Option<(usize, Option<String>)> = None;
    for index in 0..(count as usize) {
        let Some(name) = dictionary
            .constant(&format!("BRIDGE_PORT{index}_NAME"))
            .and_then(|value| value.as_str())
            .map(str::to_string)
        else {
            continue;
        };
        let kind_text = dictionary
            .constant(&format!("BRIDGE_PORT{index}_TYPE"))
            .and_then(|value| value.as_str())
            .map(str::to_string);
        if name == oid {
            found = Some((index, kind_text));
        }
        names.push(name);
    }
    let Some((index, kind_text)) = found else {
        let listed = if names.is_empty() {
            "the table is empty".to_string()
        } else {
            names
                .iter()
                .map(|name| format!("'{name}'"))
                .collect::<Vec<_>>()
                .join(", ")
        };
        return Err(config_error(format!(
            "MCU '{child}': bridge '{bridge_name}' has no port named '{oid}' (its ports are \
             {listed})"
        )));
    };
    let expected = kind.reference_option();
    let got = kind_text.as_deref().unwrap_or("none");
    if got != expected {
        return Err(config_error(format!(
            "MCU '{child}': bridge '{bridge_name}' port '{oid}' has the wrong type: expected \
             '{expected}', got '{got}'"
        )));
    }
    Ok(index as u8)
}

/// The port's `SET_PORT_NODEID` flags byte and whether it is legacy.
///
/// A legacy port's device does not speak the mux, so the bridge strips the
/// header going down and heads a raw frame coming up (`0`); every other port's
/// device speaks units, which the bridge must know ([`MUX_PORT_UNITS`]). The
/// judgment is the port grouping's ([`PortGroups::is_legacy`]) — the same one
/// that refused a legacy device on a shared port at load.
fn port_flags(groups: &PortGroups, port: &str) -> (u8, bool) {
    let legacy = groups.is_legacy(port);
    (if legacy { 0 } else { MUX_PORT_UNITS }, legacy)
}

/// Wrap `message` as the config error this pass reports.
fn config_error(message: String) -> KlippyError {
    KlippyError::Config(ConfigError::new(message))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{Config, ConfigWrapper};
    use crate::core::klippy::frame::{Frame, MESSAGE_MAX, MESSAGE_MIN};
    use crate::core::klippy::interface::devices::host::HOST_LIBRARY_TEST_LOCK;
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::printer::Printer;
    use crate::core::klippy::reactor::{ManualReactor, TokioReactor};
    use serde_json::json;
    use std::io::Read;
    use std::os::fd::AsRawFd;
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    /// A printer with every object a test config's `[mcu]` sections register.
    fn printer_with(text: &str) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = Config::from_text(text).expect("the test config parses").0;
        printer.load_config(&config).expect("the config loads");
        printer
    }

    /// The parsed transport of the `[mcu <name>]` section, read back through the
    /// object `connect` itself parses.
    fn child_config(printer: &Printer, name: &str) -> McuConfig {
        printer
            .lookup_object_as::<McuObject>(&format!("mcu {name}"))
            .unwrap_or_else(|| panic!("the printer has [mcu {name}]"))
            .mcu_config()
            .expect("the child's section parses")
    }

    /// A `PortGroups` classification of `text`, reached through its first
    /// `[mcu]`-family section.
    fn port_groups(text: &str) -> PortGroups {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        PortGroups::classify(&wrapper).expect("the topology is sound")
    }

    // -----------------------------------------------------------------------
    // The three port-table failures and the not-yet-connected bridge.
    // -----------------------------------------------------------------------

    /// A bridge that has not connected yet cannot say which port the child sits
    /// on: the error names both sections and says to reorder the config file.
    /// There is no waiting mechanism — bring-up connects in registration order.
    #[test]
    fn resolve_needs_the_bridge_connected_first() {
        let printer = printer_with(
            "[mcu bridge]\ntest: dict=/does/not/matter\n\
             [mcu child]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        );
        let config = child_config(&printer, "child");

        let err = resolve(&printer, &config).unwrap_err();

        let text = err.to_string();
        assert!(
            text.contains("MCU 'child' hangs on a port of 'bridge'"),
            "{text}"
        );
        assert!(
            text.contains("declare the bridge before it in the config file"),
            "{text}"
        );
    }

    /// A `[mcu]`'s identify answer built in code: the `config` object of the
    /// dictionary holds the compile-time constants (the port table).
    fn dictionary(value: serde_json::Value) -> Dictionary {
        Dictionary::from_json(value).expect("the dictionary parses")
    }

    /// A bridge whose firmware reports no port table (an old firmware, or one
    /// that is not a bridge build) is refused: there is no index to name.
    #[test]
    fn a_bridge_without_a_port_table_is_refused() {
        let dictionary = dictionary(json!({"config": {"CLOCK_FREQ": 1}}));

        let err =
            bridge_port_index(&dictionary, "uart0", PortKind::Uart, "child", "bridge").unwrap_err();

        let text = err.to_string();
        assert!(text.contains("MCU 'child'"), "{text}");
        assert!(
            text.contains(
                "bridge 'bridge' reports no port table (old firmware, or not a bridge build)"
            ),
            "{text}"
        );
    }

    /// A port table without the child's oid is refused, and the error lists the
    /// ports the bridge does report.
    #[test]
    fn a_port_table_without_the_oid_is_refused_and_lists_its_ports() {
        let dictionary = dictionary(json!({"config": {
            "BRIDGE_PORT_COUNT": 2,
            "BRIDGE_PORT0_NAME": "uart0",
            "BRIDGE_PORT0_TYPE": "uart",
            "BRIDGE_PORT1_NAME": "uart1",
            "BRIDGE_PORT1_TYPE": "uart",
        }}));

        let err =
            bridge_port_index(&dictionary, "uart2", PortKind::Uart, "child", "bridge").unwrap_err();

        let text = err.to_string();
        assert!(
            text.contains("bridge 'bridge' has no port named 'uart2'"),
            "{text}"
        );
        assert!(text.contains("its ports are 'uart0', 'uart1'"), "{text}");
    }

    /// A port whose firmware type does not match the section's kind is refused:
    /// an oid reached over the wrong kind would carry the wrong wire format.
    #[test]
    fn a_port_of_the_wrong_type_is_refused() {
        let dictionary = dictionary(json!({"config": {
            "BRIDGE_PORT_COUNT": 1,
            "BRIDGE_PORT0_NAME": "uart0",
            "BRIDGE_PORT0_TYPE": "can",
        }}));

        let err =
            bridge_port_index(&dictionary, "uart0", PortKind::Uart, "child", "bridge").unwrap_err();

        let text = err.to_string();
        assert!(
            text.contains("port 'uart0' has the wrong type: expected 'uart', got 'can'"),
            "{text}"
        );
    }

    // -----------------------------------------------------------------------
    // The flags byte (`SET_PORT_NODEID`'s fourth field).
    // -----------------------------------------------------------------------

    /// A legacy port — one uart-port device with no `uuid:` — is spoken to in
    /// raw frames, so its flags byte is `0`.
    #[test]
    fn a_legacy_port_sends_zero_flags() {
        let groups = port_groups(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        );

        let (flags, legacy) = port_flags(&groups, "uart_device uart0");

        assert_eq!(flags, 0);
        assert!(legacy);
    }

    /// A shared port (two or more members, each with its own `uuid:`) is a port
    /// whose devices speak units, so its flags byte says so.
    #[test]
    fn a_shared_port_sends_the_units_flag() {
        let groups = port_groups(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu a]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [mcu b]\nuart: uart0\nuuid: 22bb33cc44dd\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        );

        let (flags, legacy) = port_flags(&groups, "uart_device uart0");

        assert_eq!(flags, MUX_PORT_UNITS);
        assert!(!legacy);
    }

    // -----------------------------------------------------------------------
    // The config-driven route, over the real bridge library and a real pty.
    // -----------------------------------------------------------------------

    /// Read from the pty's master end for at most `window`: everything the
    /// bridge wrote to the port, handed over as soon as `at_least` bytes are in
    /// hand — empty when the port stays silent the whole window. The `poll` is
    /// bounded, so no step can hang on a port that never writes.
    fn read_pty(master: &std::fs::File, at_least: usize, window: Duration) -> Vec<u8> {
        let deadline = Instant::now() + window;
        let mut out = Vec::new();
        let mut buf = [0u8; 256];
        let mut reader = master;
        while out.len() < at_least {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            let wait = deadline.saturating_duration_since(now).as_millis() as i32;
            let mut poll = libc::pollfd {
                fd: master.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let rc = unsafe { libc::poll(&mut poll, 1, wait.max(1)) };
            if rc < 0 {
                let error = std::io::Error::last_os_error();
                if error.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                panic!("polling the pty master: {error}");
            }
            if rc == 0 {
                break;
            }
            match reader.read(&mut buf) {
                Ok(0) => break,
                Ok(read) => out.extend_from_slice(&buf[..read]),
                Err(error) => panic!("reading the pty master: {error}"),
            }
        }
        out
    }

    /// Block (bounded) until the process-wide library on `path` is unloaded, so
    /// the next test on it meets a fresh firmware.
    ///
    /// A load in a mode/identity the live library cannot serve fails while it is
    /// still up; a successful one means it was already unloaded, and dropping
    /// that one reference unloads it again.
    fn wait_for_mux_library_unload(path: &std::path::Path) {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match HostDevice::load_bytes(path, Some([0x5a; 6])) {
                Ok(probe) => {
                    drop(probe);
                    return;
                }
                Err(_) => {
                    assert!(
                        Instant::now() < deadline,
                        "the mux host library never unloaded"
                    );
                    std::thread::sleep(Duration::from_millis(10));
                }
            }
        }
    }

    /// Every whole, CRC-checked frame in `bytes`, scanning past whatever mux
    /// headers or partial writes are mixed in.
    ///
    /// A mux header's first byte has `0x80` set, which is outside a frame
    /// length (`0x05..=0x40`), so a bare frame is found by the length byte and
    /// proven whole by [`Frame::parse`] (the CRC and the `MESSAGE_DEST` bits).
    fn bare_frames(bytes: &[u8]) -> Vec<&[u8]> {
        let mut frames = Vec::new();
        let mut offset = 0;
        while offset < bytes.len() {
            let len = bytes[offset] as usize;
            if (MESSAGE_MIN..=MESSAGE_MAX).contains(&len)
                && offset + len <= bytes.len()
                && Frame::parse(&bytes[offset..offset + len]).is_some()
            {
                frames.push(&bytes[offset..offset + len]);
                offset += len;
            } else {
                offset += 1;
            }
        }
        frames
    }

    /// The whole config-driven path, over the real bridge library and a real
    /// pty: `[mcu bridge] host_library:` (a bridge, so the path is addressed by
    /// id: `host_libraries::path_is_mux`) + `[uart_device uart0]` + `[mcu child]
    /// uart: uart0` (no `uuid:`) comes up far enough that the child's **own**
    /// frames reach the bridge's downstream port.
    ///
    /// The child then blocks on identify — nothing answers the pty — so the
    /// bring-up is bounded and its outcome is not the observable; the port is.
    /// What the pty must show is the child's **bare** frame: `SET_PORT_NODEID`
    /// gave the bridge the route, the bridge stripped the mux header off the
    /// host's unit, and a frame — not a two-byte mux header — came out. A bare
    /// frame's first byte is its length (`0x05..=0x40`); a mux header's first
    /// byte always has `0x80` set. The bridge fans its own admin handshake out to
    /// the port, so the bytes carry header-led units too; the bare frame is found
    /// among them, and its payload starting with the host-defined `identify`
    /// message id (`0x01`, `identify.rs`) is what makes it the child's request.
    #[tokio::test(flavor = "multi_thread")]
    async fn the_config_driven_legacy_route_reaches_the_bridge_port() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge opens its port during init, so the tty has to exist first.
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                pty.path.display()
            )
        });

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             [mcu child]\nuart: uart0\n",
            library.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        printer.load_config(&config).expect("the config loads");

        // Bounded: the child's identify never answers, so the bring-up is
        // interrupted once the child has certainly spoken.
        let _ = tokio::time::timeout(Duration::from_secs(6), printer.bring_up()).await;

        let bytes = read_pty(&pty.master, 1, Duration::from_secs(3));
        assert!(
            !bytes.is_empty(),
            "the bridge port received nothing; SET_PORT_NODEID did not route the child"
        );
        let frames = bare_frames(&bytes);
        let request = frames.iter().find(|frame| frame[2] == 0x01);
        assert!(
            request.is_some(),
            "the port must receive the child's bare identify request (a frame with the mux \
             header stripped, its payload led by the identify message id); it got {bytes:02x?}"
        );
        let frame = Frame::parse(request.unwrap()).expect("the bytes are one whole frame");
        assert!(
            !frame.payload().is_empty(),
            "the identify request carries a payload"
        );

        // `teardown` drops the parts the config loaded and the event handlers
        // they registered; without it the `SAVE_CONFIG` handler's strong
        // `Arc<Printer>` keeps the machine (and so the bridge's open library)
        // alive, and the next test on this path meets a live firmware.
        printer.teardown();
        drop(printer);
        wait_for_mux_library_unload(&library);
        let _ = std::fs::remove_file(&bridge_path);
    }
}
