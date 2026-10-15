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
//!    the bridge is, the port's `index` on that bridge (the index the host
//!    assigned the port when it created it at configuration time, so its
//!    declaration order among the bridge's ports — `mcu/bridge_ports.rs`), and
//!    the `flags` byte the port's shape asks for. The bridge must be up first:
//!    what remains to check here is that its object has connected
//!    (`mcu/object.rs` awaits objects in registration order, so the bridge is up
//!    when the child is).
//! 2. [`open`] — open a **downstream** handle on the bridge's own link (with no
//!    handshake of the device's own), send `SET_PORT_NODEID(index, id, flags)` to
//!    build the bridge's route, and then — for a **named** device — run its own
//!    naming handshake over that route. The order is the point: see the two
//!    shapes below.
//! 3. The caller (`Mcu::connect`) then speaks to the handle exactly as it would
//!    to any MCU's transport.
//!
//! Two shapes land here, and the second sends **two** commands:
//!
//! - **Legacy** — one `[mcu]` on a uart port writing no `uuid:`. It has no admin
//!   channel of its own, so [`open`] sends only `SET_PORT_NODEID`: the bridge
//!   owns the mcu id and strips/heads the raw frames its device speaks.
//! - **Named** — a `uuid:` on the child (a uart port's every other shape). The
//!   device answers the mux itself, so it needs **both** commands: the bridge's
//!   route (`SET_PORT_NODEID`) *and* its own naming handshake
//!   (`SET_KLIPPER_NODEID` + re-confirm). [`open`] sends them in that order —
//!   the route **first**, because the handshake's units travel the route the
//!   bridge must own before they reach the device; sent ahead of it, the bridge
//!   drops them and the handshake times out with nothing downstream.
//!
//! A CAN port (`[can_device]`) has no configuration-time object yet (the
//! firmware's CAN *port backend* - `BRIDGE_PORT_CAN`, a `bridge_port_ops` that
//! drives a CAN transceiver - does not exist; the CAN mux carrier `MUX_CAN`
//! itself does), so a `can:` device is refused by name instead of being given a
//! route to a port the bridge does not have.
//!
//! The bridge's own transport decides how the downstream handle is opened, and
//! the two kinds it can be are exactly the two mux links: a shared serial port
//! ([`serial::attach_downstream`]) or a host library
//! ([`HostDevice::load_downstream`]). A bridge reached over CAN carries its
//! downstream frames inside the CAN payload: the firmware carrier (`MUX_CAN`)
//! and the port-command units exist, but the host-side attach and the CAN
//! *port* backend (`BRIDGE_PORT_CAN`) do not, so it is refused here.

use super::bridge_ports::{CanPort, UartPort};
use super::host_libraries::{HostLibraries, HOST_LIBRARIES_OBJECT};
use super::port_grouping::{PortGroups, PortKind, PORT_GROUPS_OBJECT};
use super::McuObject;
use crate::core::klippy::config::mcu::{McuConfig, Transport};
use crate::core::klippy::config::ConfigError;
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::interface::devices::host::HostDevice;
use crate::core::klippy::interface::devices::serial::{self, MUX_PORT_UNITS};
use crate::core::klippy::interface::Interface;
use crate::core::klippy::printer::Printer;
use std::sync::Arc;
use tracing::debug;

/// Where a downstream device attaches, resolved from the child's config and the
/// bridge's identify answer.
///
/// Built by [`resolve`]; [`open`] turns it into a live [`Interface`]. The
/// `transport` is the **bridge's**, not the child's: the child has none of its
/// own to open.
#[derive(Debug)]
pub(crate) struct BridgePortLink {
    /// The bridge's own transport, from the bridge's own section parse
    /// (`McuObject::mcu_config`). The load-time shared-baud / host-mux stamps
    /// are [`McuConfig`] fields, not part of [`Transport`].
    pub transport: Transport,
    /// The port's index on the bridge: the number the host gave it when it
    /// created the port at configuration time (its declaration order among the
    /// bridge's ports). It is what the port-level command travels with.
    pub index: u8,
    /// The port-level `SET_PORT_NODEID` flags byte: `0` for a legacy port
    /// ([`PortGroups::is_legacy`], one uart-port device with no `uuid:`), whose
    /// device is spoken to in raw frames; [`MUX_PORT_UNITS`] for a port whose
    /// device speaks units.
    pub flags: u8,
    /// The child's own `uuid:`, or `None` for the legacy shape. It is what
    /// [`open`]'s named branch names the device with (`SET_KLIPPER_NODEID`):
    /// a device behind a port answers the mux itself, unlike a legacy one.
    pub named: Option<[u8; 6]>,
}

/// Resolve the child's `uart:`/`can:` transport to the bridge port it names.
///
/// The port's index comes from the **port section** the host built at
/// configuration time (its declaration order among the bridge's ports), so this
/// only succeeds once the bridge object has connected. Bring-up connects objects
/// in registration order, which is config-file order within a phase, so a named
/// bridge declared before its child is up by the time the child asks; one
/// declared after its child is not waited for. A bare `[mcu]` child is the
/// exception the error names: the main section always registers before every
/// `[mcu <name>]`, so it can never be ordered after a named bridge and has to be
/// given a name.
///
/// # Errors
/// [`KlippyError::Config`] for everything here: a bridge that has not connected,
/// a port section that carries no index (a section built outside the loader), or
/// a CAN port, which has no configuration-time object yet.
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
    // A CAN downstream port has no configuration-time object yet: the firmware's
    // CAN port backend (`BRIDGE_PORT_CAN`) does not exist, so there is no port
    // the child could be routed to. (The CAN mux carrier `MUX_CAN` does exist -
    // it carries units over a CAN *upstream*; a CAN *downstream* port is a
    // separate `bridge_port_ops`.) Refuse it by name instead of sending a route
    // to a port the bridge does not have (which would drop it in silence).
    if *kind == PortKind::Can {
        let section = printer.lookup_object_as::<CanPort>(port).ok_or_else(|| {
            config_error(format!("MCU '{}': no port section '{port}'", config.name))
        })?;
        return Err(config_error(format!(
            "MCU '{}': bridge '{}' cannot host port '{}' (a [can_device]): a CAN downstream \
             port is not created at configuration time yet (the firmware's CAN port backend, \
             BRIDGE_PORT_CAN, does not exist)",
            config.name,
            section.mcu(),
            section.name()
        )));
    }
    // The port section object and the `[mcu]` it says is the bridge. A UART
    // port's index is the one the host assigned it when it created the port at
    // configuration time (`mcu/bridge_ports.rs`): its declaration order among
    // the bridge's ports, which is the index the firmware's router gave it.
    let (bridge, index) = {
        let section = printer.lookup_object_as::<UartPort>(port).ok_or_else(|| {
            config_error(format!("MCU '{}': no port section '{port}'", config.name))
        })?;
        let index = section.index().ok_or_else(|| {
            config_error(format!(
                "MCU '{}': port section '{port}' was not configured for its bridge",
                config.name
            ))
        })?;
        (section.mcu().to_string(), index)
    };

    // The bridge object, registered under the same spelling the port section
    // writes: `mcu` for the bare section, `mcu <sub>` otherwise.
    let bridge_name = if bridge == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {bridge}")
    };
    let not_up = || {
        // A bare `[mcu]` child can never be ordered after a named bridge: the
        // main section always registers (and so connects) before every
        // `[mcu <name>]`, which makes the file-order advice unsatisfiable for
        // it — the only way out is to name it.
        let bare_child = if config.name == "mcu" {
            "; a bare [mcu] always loads before [mcu <name>], so give this device an \
             [mcu <name>] section of its own"
        } else {
            ""
        };
        config_error(format!(
            "MCU '{}' hangs on a port of '{bridge}'; declare the bridge before it in the config \
             file{bare_child}",
            config.name
        ))
    };
    let bridge_object = printer
        .lookup_object_as::<McuObject>(&bridge_name)
        .ok_or_else(not_up)?;
    if bridge_object.mcu().is_none() {
        return Err(not_up());
    }
    // The bridge's transport, for `open` to reach the downstream device through.
    let bridge_config = bridge_object.mcu_config()?;

    let groups = printer
        .lookup_object_as::<PortGroups>(PORT_GROUPS_OBJECT)
        .ok_or_else(|| {
            config_error(format!(
                "MCU '{}': the port grouping is not loaded",
                config.name
            ))
        })?;
    let flags = port_flags(&groups, port);
    Ok(BridgePortLink {
        transport: bridge_config.transport,
        index,
        flags,
        named: *uuid,
    })
}

/// Open a downstream handle on `link`'s bridge link, **without** a route.
///
/// This is the bridge's own transport reopened for one device — a second
/// reference on the same shared serial port or host library, addressed by
/// `mcu_id`, with no handshake of its own. [`open`] layers the device's route
/// and naming on top; a caller that only needs the bridge's **admin** channel
/// (`restart.rs::open_bridge_link`, a port-level reset) opens it bare.
///
/// It is the one place a bridge link can refuse a downstream device (a CAN
/// upstream, a transport with no mux), so both callers report the same words.
///
/// # Errors
/// [`KlippyError::Config`] for a bridge link that cannot carry a downstream
/// device (a CAN upstream, or a transport with no mux); [`KlippyError::Connection`]
/// for a failed open.
pub(crate) fn open_link(
    printer: &Printer,
    link: &BridgePortLink,
    mcu_id: u32,
) -> Result<Interface, KlippyError> {
    let (interface, _handle) = open_link_handle(printer, link, mcu_id)?;
    Ok(interface)
}

/// The downstream handle behind [`open_link`], alongside the interface.
///
/// The concrete device is what a named device's handshake needs
/// ([`HostDevice::attach`], [`serial::SharedSerialHandle::handshake`]) and what
/// [`Interface`] does not expose; the open itself is identical either way.
enum LinkHandle {
    /// A second reference on the bridge's muxed host library.
    Host {
        device: Arc<HostDevice>,
        library: String,
    },
    /// A second slot on the bridge's shared serial port.
    Serial {
        handle: Arc<serial::SharedSerialHandle>,
        path: String,
    },
}

impl LinkHandle {
    /// Run the named device's own handshake over the handle [`open`] just
    /// opened: `SET_KLIPPER_NODEID` plus a re-confirm request, each answered by
    /// the device. The route must already be in ([`open`] sends it first), or
    /// the units reach no port and the handshake times out.
    fn handshake(&self, named: [u8; 6], mcu_id: u32, index: u8) -> Result<(), KlippyError> {
        match self {
            Self::Host { device, library } => device.attach(named, mcu_id).map_err(|err| {
                KlippyError::Connection(format!(
                    "host_library {library}: SET_KLIPPER_NODEID handshake on port {index} failed: \
                     {err}"
                ))
            }),
            Self::Serial { handle, path } => handle
                .handshake(named, serial::ASSIGN_ATTACH_TIMEOUT)
                .map_err(|err| {
                    KlippyError::Connection(format!(
                        "serial {path}: SET_KLIPPER_NODEID handshake on port {index} failed: {err}"
                    ))
                }),
        }
    }
}

/// Open the downstream handle and keep it, for [`open`]'s handshake.
///
/// The single match over the bridge's transport, and so the single refusal for a
/// link that cannot carry a downstream device; [`open_link`] drops the handle
/// again.
fn open_link_handle(
    printer: &Printer,
    link: &BridgePortLink,
    mcu_id: u32,
) -> Result<(Interface, LinkHandle), KlippyError> {
    match &link.transport {
        Transport::Host { library, .. } => {
            // The downstream reference is addressed by the path's mux identity,
            // the same one the bridge itself attached with. `named` is what a
            // later `attach` names the device with; `load_downstream` itself
            // only reads the identity.
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
            let device = HostDevice::load_downstream(library, identity, mcu_id).map_err(|err| {
                KlippyError::Connection(format!(
                    "host_library {library}: downstream attach failed: {err}"
                ))
            })?;
            let interface = Interface::from_host(Arc::clone(&device));
            Ok((
                interface,
                LinkHandle::Host {
                    device,
                    library: library.clone(),
                },
            ))
        }
        Transport::Serial { path, baud, .. } => {
            let handle = serial::attach_downstream(path, *baud, mcu_id).map_err(|err| {
                KlippyError::Connection(format!("serial {path}: downstream attach failed: {err}"))
            })?;
            let interface = Interface::shared_serial(Arc::clone(&handle));
            Ok((
                interface,
                LinkHandle::Serial {
                    handle,
                    path: path.clone(),
                },
            ))
        }
        Transport::Can { .. } => Err(config_error(
            "a bridge reached over CAN cannot carry a downstream device yet: the CAN mux \
             carrier and the port-command units exist, but the host-side attach is not \
             wired yet"
                .to_string(),
        )),
        other => Err(config_error(format!(
            "a bridge on {other:?} cannot carry a downstream device"
        ))),
    }
}

/// Turn a resolved [`BridgePortLink`] into the live [`Interface`] the caller
/// speaks to the device through: [`open_link`] the downstream handle, then — in
/// this order — the bridge's **route** and, for a named device, the device's own
/// naming handshake.
///
/// Two commands travel the link, in this order:
///
/// 1. `SET_PORT_NODEID(index, mcu_id, flags)` — the bridge's **route**. It is
///    sent before anything else the caller writes, because until the bridge has
///    it, `mcu_id` names no port and frames under it go nowhere.
/// 2. For a **named** device ([`BridgePortLink::named`]), its own naming
///    handshake: `SET_KLIPPER_NODEID(uuid, mcu_id)` plus a re-confirm request
///    (a legacy device has no admin channel and gets only step 1). It must come
///    **after** the route: the handshake's units are addressed to `mcu_id`, so
///    with no route the bridge drops them and the device never hears a word.
///
/// # Errors
/// [`KlippyError::Config`] for a bridge link that cannot carry a downstream
/// device (a CAN upstream, or a transport with no mux); [`KlippyError::Connection`]
/// for a failed open, a failed `SET_PORT_NODEID`, or a named device's handshake
/// that never completes.
pub(crate) fn open(
    printer: &Printer,
    link: &BridgePortLink,
    mcu_id: u32,
) -> Result<Interface, KlippyError> {
    let id = u8::try_from(mcu_id).map_err(|_| {
        KlippyError::Connection(format!(
            "MCU id {mcu_id} is outside the range a bridge port can address (1..=255)"
        ))
    })?;
    let (interface, handle) = open_link_handle(printer, link, mcu_id)?;
    // The route first, then the device's own naming — see the function doc; the
    // handshake's units are dropped by the bridge without it.
    interface
        .send_set_port_node_id(link.index, id, link.flags)
        .map_err(|err| route_error(link.index, id, link.flags, err))?;
    if let Some(named) = link.named {
        handle.handshake(named, mcu_id, link.index)?;
    }
    debug!(
        "bridge port {}: attaching downstream device id {mcu_id} ({}, {})",
        link.index,
        if link.flags == 0 { "legacy" } else { "units" },
        if link.named.is_some() {
            "named"
        } else {
            "unnamed"
        }
    );
    Ok(interface)
}

/// The port's `SET_PORT_NODEID` flags byte.
///
/// A legacy port's device does not speak the mux, so the bridge strips the
/// header going down and heads a raw frame coming up (`0`); every other port's
/// device speaks units, which the bridge must know ([`MUX_PORT_UNITS`]). The
/// judgment is the port grouping's ([`PortGroups::is_legacy`]) — the same one
/// that refused a legacy device on a shared port at load — and `0` is exactly
/// its byte, so the caller has no second answer to keep.
fn port_flags(groups: &PortGroups, port: &str) -> u8 {
    if groups.is_legacy(port) {
        0
    } else {
        MUX_PORT_UNITS
    }
}

/// Wrap a failed `SET_PORT_NODEID` as the connection error [`open`] reports,
/// naming the port and the flags byte the failed send carried.
fn route_error(index: u8, id: u8, flags: u8, err: impl std::fmt::Display) -> KlippyError {
    KlippyError::Connection(format!(
        "sending SET_PORT_NODEID({index}, {id}, {flags}) to the bridge failed: {err}"
    ))
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
        assert!(
            !text.contains("a bare [mcu] always loads before"),
            "a named child gets no bare-[mcu] note: {text}"
        );
    }

    /// A bare `[mcu]` child on a named bridge's port can never be ordered after
    /// the bridge: the main section always registers (and so connects) before
    /// `[mcu <name>]`. The error says so and points at the only way out — give
    /// the device a name.
    #[test]
    fn resolve_names_the_bare_mcu_order_trap() {
        let printer = printer_with(
            "[mcu bridge]\ntest: dict=/does/not/matter\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [mcu]\nuart: uart0\n",
        );
        let config = printer
            .lookup_object_as::<McuObject>("mcu")
            .expect("the printer has the bare [mcu]")
            .mcu_config()
            .expect("the bare section parses");

        let err = resolve(&printer, &config).unwrap_err();

        let text = err.to_string();
        assert!(
            text.contains("MCU 'mcu' hangs on a port of 'bridge'"),
            "{text}"
        );
        assert!(
            text.contains("a bare [mcu] always loads before [mcu <name>]"),
            "{text}"
        );
        assert!(
            text.contains("give this device an [mcu <name>] section of its own"),
            "{text}"
        );
    }

    // -----------------------------------------------------------------------
    // The port index: the declaration order the host assigned the port, which
    // is the index the firmware's router gave it.
    // -----------------------------------------------------------------------

    /// Two port sections on one bridge take the indices 0 and 1 in the order
    /// the config declares them: the firmware numbers ports by the order the
    /// host's `config_bridge_port` commands arrive, and the loader queues them
    /// in declaration order (`bridge_ports::load_uart_device`).
    #[test]
    fn port_sections_take_their_declaration_order() {
        let printer = printer_with(
            "[mcu bridge]\nserial: /dev/a\n\
             [uart_device second]\nuart_mcu: bridge\nuart_oid: uart1\n\
             [uart_device first]\nuart_mcu: bridge\nuart_oid: uart0\n",
        );
        let last = printer
            .lookup_object_as::<UartPort>("uart_device second")
            .expect("the printer has [uart_device second]");
        let first = printer
            .lookup_object_as::<UartPort>("uart_device first")
            .expect("the printer has [uart_device first]");

        assert_eq!(
            last.index(),
            Some(0),
            "the first declared port gets index 0"
        );
        assert_eq!(first.index(), Some(1), "the next gets index 1");
    }

    /// A bridge's port count is what the declaration order can address: the
    /// port index travels in one byte, so the 257th port is refused rather than
    /// wrapping onto an existing port's route.
    #[test]
    fn a_port_past_the_index_range_is_refused() {
        let printer = printer_with("[mcu bridge]\nserial: /dev/a\n");
        let bridge = printer
            .lookup_object_as::<McuObject>("mcu bridge")
            .expect("the printer has [mcu bridge]");

        for index in 0..=u8::MAX {
            assert_eq!(
                bridge.next_bridge_port_index("uart_device p").unwrap(),
                index,
                "indices 0..=255 are addressable"
            );
        }
        let err = bridge
            .next_bridge_port_index("uart_device p")
            .expect_err("the 257th port has no address");

        let text = err.to_string();
        assert!(text.contains("uart_device p"), "{text}");
        assert!(text.contains("256 ports"), "{text}");
        assert!(text.contains("0..=255"), "{text}");
    }

    /// A `can:` device is refused by name: a CAN downstream port has no
    /// configuration-time object yet, so there is no route for it to travel.
    ///
    /// The refusal is the port section's own — its `can_mcu` names the bridge —
    /// and comes before the not-yet-connected check, because no bridge state
    /// can make a CAN port exist.
    #[test]
    fn a_can_port_is_refused_by_name() {
        let printer = printer_with(
            "[mcu bridge]\ntest: dict=/does/not/matter\n\
             [mcu child]\ncan: can0\nuuid: 11aa22bb33cc\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
        );
        let config = child_config(&printer, "child");

        let err = resolve(&printer, &config).unwrap_err();

        let text = err.to_string();
        assert!(text.contains("MCU 'child'"), "{text}");
        assert!(
            text.contains("bridge 'bridge' cannot host port 'can0'"),
            "{text}"
        );
        assert!(text.contains("BRIDGE_PORT_CAN"), "{text}");
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

        let flags = port_flags(&groups, "uart_device uart0");

        assert_eq!(flags, 0);
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

        let flags = port_flags(&groups, "uart_device uart0");

        assert_eq!(flags, MUX_PORT_UNITS);
    }

    /// A bridge reached over CAN is refused with the one message the attach pass
    /// and the reset path share, and never through `{:?}`: its host-side attach
    /// is not wired yet.
    #[test]
    fn a_can_bridge_is_refused_with_the_shared_message() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let link = BridgePortLink {
            transport: Transport::Can {
                interface: "can0".to_string(),
                uuid: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
            },
            index: 0,
            flags: 0,
            named: None,
        };

        let err = open_link(&printer, &link, 7).unwrap_err();

        assert_eq!(
            err.to_string(),
            "a bridge reached over CAN cannot carry a downstream device yet: the CAN mux \
             carrier and the port-command units exist, but the host-side attach is not \
             wired yet"
        );
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
    /// that one reference unloads it again. A timeout **warns and returns**
    /// instead of asserting: this runs from `BridgeTeardown::drop`, and a panic
    /// there while the test is already unwinding would abort the whole test
    /// binary — the next test meeting the live library fails loudly instead.
    fn wait_for_mux_library_unload(path: &std::path::Path) {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match HostDevice::load_bytes(path, Some([0x5a; 6])) {
                Ok(probe) => {
                    drop(probe);
                    return;
                }
                Err(_) => {
                    if Instant::now() >= deadline {
                        eprintln!(
                            "teardown: mux host library still loaded after 10s: {} \
                             (continuing; the next test on it will fail loudly)",
                            path.display()
                        );
                        return;
                    }
                    std::thread::sleep(Duration::from_millis(10));
                }
            }
        }
    }

    /// Tears a host-library bring-up down when it drops, so a test that panics
    /// before its own cleanup leaves neither the machine (and so the library)
    /// alive for the next test on the path nor the bridge's tty symlink behind.
    ///
    /// `teardown` drops the parts the config loaded and the event handlers they
    /// registered; without it the `SAVE_CONFIG` handler's strong `Arc<Printer>`
    /// keeps the machine — and so the bridge's open library — alive, and the
    /// next test on this path meets a live firmware rather than a fresh one.
    ///
    /// The guard **owns** the printer and is declared **before** the test's own
    /// handle to it, so it drops after that handle has gone: the unload below
    /// needs the guard's release to be the last reference. A test therefore
    /// takes its handle with [`BridgeTeardown::printer`].
    struct BridgeTeardown {
        printer: Option<Arc<Printer>>,
        library: std::path::PathBuf,
        bridge_path: std::path::PathBuf,
    }

    impl BridgeTeardown {
        fn new(
            printer: Arc<Printer>,
            library: std::path::PathBuf,
            bridge_path: std::path::PathBuf,
        ) -> Self {
            Self {
                printer: Some(printer),
                library,
                bridge_path,
            }
        }

        /// The test's own handle to the printer the guard owns.
        fn printer(&self) -> Arc<Printer> {
            Arc::clone(self.printer.as_ref().expect("the guard holds the printer"))
        }
    }

    impl Drop for BridgeTeardown {
        fn drop(&mut self) {
            if let Some(printer) = self.printer.take() {
                printer.teardown();
                // Release the guard's reference, the last one: the library
                // unloads with it, which the wait below checks (and warns
                // about — never asserts — since this is a drop path).
                drop(printer);
            }
            wait_for_mux_library_unload(&self.library);
            let _ = std::fs::remove_file(&self.bridge_path);
        }
    }

    /// The child's bare `identify` request in `bytes`, if it is there: the
    /// whole frame with the mux header stripped and its payload led by the
    /// host-defined `identify` message id (`0x01`, `identify.rs`) — the traffic
    /// a bridge port is proven to carry with.
    fn identify_request(bytes: &[u8]) -> Option<&[u8]> {
        bare_frames(bytes)
            .into_iter()
            .find(|frame| frame[2] == 0x01)
    }

    /// Redirect the process's stderr into a file for as long as this lives, so
    /// a test can see what the host library reported **there**.
    ///
    /// A tty the bridge cannot open is named by the library's own `bridge_errno`
    /// (`third_party/klipper/src/host/bridge.c`), written straight to file
    /// descriptor 2: the firmware's shutdown message is a compile-time string
    /// and cannot carry a path, so stderr is the only place the tty that was
    /// tried appears. `std`'s printing is captured per test thread by libtest,
    /// before it reaches the descriptor, so this takes the library's writes and
    /// little else; everything it collected is handed back to the real stderr on
    /// drop, so nothing is lost.
    ///
    /// The swap is process-wide, so it is only built while
    /// [`HOST_LIBRARY_TEST_LOCK`] is held — the one lock every test that loads a
    /// host library takes, and the reason no other library is writing to the
    /// descriptor while it is redirected. Taking the guard as a parameter is
    /// what makes that structural rather than a comment.
    struct StderrCapture {
        saved: libc::c_int,
        path: std::path::PathBuf,
    }

    impl StderrCapture {
        fn start(_lock: &std::sync::MutexGuard<'_, ()>, tag: &str) -> Self {
            let path = std::env::temp_dir().join(format!(
                "klipperx-{tag}-{}-{:?}.stderr",
                std::process::id(),
                std::thread::current().id()
            ));
            let file = std::fs::OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open(&path)
                .expect("the capture file opens");
            let saved = unsafe { libc::dup(2) };
            assert!(saved >= 0, "dup(2): {}", std::io::Error::last_os_error());
            assert_eq!(
                unsafe { libc::dup2(file.as_raw_fd(), 2) },
                2,
                "dup2: {}",
                std::io::Error::last_os_error()
            );
            Self { saved, path }
        }

        /// Everything written to stderr since `start`.
        fn text(&self) -> String {
            std::fs::read_to_string(&self.path).unwrap_or_default()
        }
    }

    impl Drop for StderrCapture {
        fn drop(&mut self) {
            let text = std::fs::read(&self.path).unwrap_or_default();
            unsafe {
                libc::dup2(self.saved, 2);
                libc::close(self.saved);
            }
            use std::io::Write;
            let _ = std::io::stderr().write_all(&text);
            let _ = std::fs::remove_file(&self.path);
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
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(reactor)),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();
        printer.load_config(&config).expect("the config loads");

        // Bounded: the child's identify never answers, so the bring-up is
        // interrupted once the child has certainly spoken.
        let _ = tokio::time::timeout(Duration::from_secs(6), printer.bring_up()).await;

        let bytes = read_pty(&pty.master, 1, Duration::from_secs(3));
        assert!(
            !bytes.is_empty(),
            "the bridge port received nothing; SET_PORT_NODEID did not route the child"
        );
        let request = identify_request(&bytes).unwrap_or_else(|| {
            panic!(
                "the port must receive the child's bare identify request (a frame with the mux \
                 header stripped, its payload led by the identify message id); it got {bytes:02x?}"
            )
        });
        let frame = Frame::parse(request).expect("the bytes are one whole frame");
        assert!(
            !frame.payload().is_empty(),
            "the identify request carries a payload"
        );
    }

    /// The whole config-driven chain against the **real** linux process
    /// firmware as the legacy downstream: `[mcu bridge]` over the host mux
    /// library, `[uart_device uart0]` with `uart_path:` on the firmware's own
    /// pty, and a uuidless `[mcu child]` — the port groups as legacy, so the
    /// attach sends only the bridge's route and the firmware itself answers
    /// identify and `get_clock` (the chain of
    /// `the_config_driven_legacy_route_reaches_the_bridge_port`, with the
    /// hand-built answer replaced by a real firmware's).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_real_linux_firmware_completes_identify_and_clock_behind_the_bridge_port() {
        use crate::core::klippy::cmd::{ClockSync, McuClock};
        use crate::core::klippy::printer::PrinterState;

        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The real downstream: its pty slave (hung at the `-I` path) is the
        // tty the port opens through `uart_path:`. Never written to directly.
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn().expect("the firmware process starts");

        // The bridge keeps its build-baked tty as a live pty, so the only
        // difference from the E1 chain is the port's own path.
        let baked = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&baked.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                baked.path.display()
            )
        });

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             uart_path: {}\n\
             [mcu child]\nuart: uart0\n",
            library.display(),
            firmware.serial_path().display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor: Arc<dyn crate::core::klippy::reactor::Reactor> =
            Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(Arc::clone(&reactor))),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();
        printer.load_config(&config).expect("the config loads");

        // Everything answers here: unlike the E1 chain, bring-up runs to
        // completion instead of being interrupted at the silent child.
        tokio::time::timeout(Duration::from_secs(30), printer.bring_up())
            .await
            .expect("the real firmware answers every step of the chain");
        let state = printer.get_state_message();
        assert_eq!(
            state.category,
            PrinterState::Ready,
            "the whole chain came up: {}",
            state.message
        );

        let child = printer
            .lookup_object_as::<McuObject>("mcu child")
            .expect("the printer has [mcu child]")
            .mcu()
            .expect("the child connected");
        assert!(child.is_identified(), "the child completed identify");

        // The identify payload is the firmware's own: the version string it
        // reports, and the CLOCK_FREQ only this linux build carries (the mux
        // host library is built at 20 MHz).
        let status = printer
            .status_of("mcu child", 0.0)
            .expect("the child has a status snapshot");
        let version = status
            .get("mcu_version")
            .and_then(|value| value.as_str())
            .unwrap_or_default();
        assert!(
            !version.is_empty(),
            "the firmware's own version came through identify: {status}"
        );
        assert_eq!(
            status["mcu_constants"]["CLOCK_FREQ"].as_f64(),
            Some(50_000_000.0),
            "CLOCK_FREQ is the linux firmware's own constant: {status}"
        );

        // One `get_clock` round trip over the port, and a second to see the
        // counter advance at CLOCK_FREQ. The value is the low 32 bits (it
        // wraps at ~86 s at 50 MHz), so the pair is checked by its delta
        // rather than against an absolute bound.
        let clock = McuClock::new(Arc::clone(&child), reactor);
        let first = tokio::time::timeout(Duration::from_secs(5), clock.get_clock())
            .await
            .expect("get_clock is answered over the bridge port")
            .expect("get_clock after identify");
        assert!(first.clock > 0, "the firmware counter is running");
        tokio::time::sleep(Duration::from_millis(20)).await;
        let second = tokio::time::timeout(Duration::from_secs(5), clock.get_clock())
            .await
            .expect("the second get_clock is answered")
            .expect("get_clock twice");
        let advanced = second.clock.wrapping_sub(first.clock);
        // The 5s `get_clock` timeout plus the 20ms sleep bound any possible
        // stall: ≈ (5s + 20ms) × 50 MHz ≈ 251M ticks, so a delta under that
        // is only "the counter advanced".
        assert!(
            advanced > 0 && advanced < 251_000_000,
            "the counter advances by a plausible tick delta: {first:?} → {second:?}"
        );

        // No firmware process outlives the test: the fixture kills and reaps
        // on drop, after the printer and the mux library have let go. The
        // clock and the mcu handle hold the child's interface — and through
        // it the mux library — so they go first, or the unload wait below
        // still finds the library loaded.
        let pid = firmware.pid();
        drop(clock);
        drop(child);
        drop(printer);
        drop(teardown);
        drop(firmware);
        let deadline = Instant::now() + Duration::from_secs(5);
        while std::path::Path::new(&format!("/proc/{pid}")).exists() {
            assert!(
                Instant::now() < deadline,
                "firmware {pid} is still running after drop"
            );
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }

    /// The whole config-driven chain against the **real** linux process
    /// firmware as a **named** downstream: `[mcu bridge]` over the host mux
    /// library, `[uart_device uart0]` with `uart_path:` on the firmware's own
    /// pty, and a `[mcu child]` whose `uuid:` is the identity the firmware was
    /// started with (`-U`). The port groups as named, so the host scans it, the
    /// firmware answers with that identity, and the two complete the mux
    /// handshake — the named counterpart of
    /// `a_real_linux_firmware_completes_identify_and_clock_behind_the_bridge_port`,
    /// and the software-side equivalent of a named device that is real
    /// hardware.
    ///
    /// One identity constant feeds both the firmware's `-U` and the child's
    /// `uuid:`: the bytes have to be the same, or the scan finds no match and
    /// the handshake times out instead of failing clearly.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_named_real_linux_firmware_completes_handshake_behind_the_bridge_port() {
        use crate::core::klippy::cmd::{ClockSync, McuClock};
        use crate::core::klippy::printer::PrinterState;

        // One definition, two consumers, so the firmware's identity and the
        // child's `uuid:` cannot drift apart.
        const NAMED_DOWNSTREAM_UUID: [u8; 6] = [0x0d, 0x1e, 0x2f, 0x3a, 0x4b, 0x5c];
        let uuid_hex: String = NAMED_DOWNSTREAM_UUID
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();

        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The real named downstream: started with the identity, its pty slave
        // (hung at the `-I` path) is the tty the port opens through
        // `uart_path:`. Never written to directly.
        let firmware = klipperx_test_support::LinuxFirmware::spawn_named(NAMED_DOWNSTREAM_UUID)
            .expect("the mux firmware process starts");

        // The bridge keeps its build-baked tty as a live pty, so the only
        // difference from the legacy chain is the port's own path.
        let baked = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&baked.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                baked.path.display()
            )
        });

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             uart_path: {}\n\
             [mcu child]\nuart: uart0\nuuid: {uuid_hex}\n",
            library.display(),
            firmware.serial_path().display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor: Arc<dyn crate::core::klippy::reactor::Reactor> =
            Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(Arc::clone(&reactor))),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();

        // The library's own stderr is where the downstream port reports the tty
        // it opened (`bridge.c`'s `open at … (fd N, slot 0)`) — the route's far
        // end. Captured from before the config loads through the bring-up,
        // because the port may open either side of that call.
        let capture = StderrCapture::start(&_serial, "named-real-firmware");
        printer.load_config(&config).expect("the config loads");

        // Everything answers here: the named handshake completes instead of
        // being interrupted at a silent child.
        tokio::time::timeout(Duration::from_secs(30), printer.bring_up())
            .await
            .expect("the named real firmware answers every step of the chain");
        let stderr = capture.text();
        drop(capture);

        let state = printer.get_state_message();
        assert_eq!(
            state.category,
            PrinterState::Ready,
            "the whole chain came up: {}",
            state.message
        );
        // The port handle as well as `Ready`: the bridge opened a downstream
        // tty on slot 0, which is the routing end the handshake travelled to.
        assert!(
            stderr.contains("open at") && stderr.contains("slot 0"),
            "the bridge opened its route's downstream port on the firmware's tty; \
             library stderr:\n{stderr}"
        );

        let child = printer
            .lookup_object_as::<McuObject>("mcu child")
            .expect("the printer has [mcu child]")
            .mcu()
            .expect("the child connected");
        assert!(
            child.is_identified(),
            "the child completed identify, so the scan matched the firmware's identity"
        );

        // The identify payload is the firmware's own: the version string it
        // reports, and the CLOCK_FREQ only this linux build carries (the mux
        // host library is built at 20 MHz).
        let status = printer
            .status_of("mcu child", 0.0)
            .expect("the child has a status snapshot");
        let version = status
            .get("mcu_version")
            .and_then(|value| value.as_str())
            .unwrap_or_default();
        assert!(
            !version.is_empty(),
            "the firmware's own version came through identify: {status}"
        );
        assert_eq!(
            status["mcu_constants"]["CLOCK_FREQ"].as_f64(),
            Some(50_000_000.0),
            "CLOCK_FREQ is the linux firmware's own constant: {status}"
        );

        // One `get_clock` round trip over the port, and a second to see the
        // counter advance at CLOCK_FREQ. The value is the low 32 bits (it wraps
        // at ~86 s at 50 MHz), so the pair is checked by its delta rather than
        // against an absolute bound (the 5s timeout plus the 20ms sleep bounds
        // any possible stall at ≈ 251M ticks).
        let clock = McuClock::new(Arc::clone(&child), reactor);
        let first = tokio::time::timeout(Duration::from_secs(5), clock.get_clock())
            .await
            .expect("get_clock is answered over the bridge port")
            .expect("get_clock after identify");
        assert!(first.clock > 0, "the firmware counter is running");
        tokio::time::sleep(Duration::from_millis(20)).await;
        let second = tokio::time::timeout(Duration::from_secs(5), clock.get_clock())
            .await
            .expect("the second get_clock is answered")
            .expect("get_clock twice");
        let advanced = second.clock.wrapping_sub(first.clock);
        assert!(
            advanced > 0 && advanced < 251_000_000,
            "the counter advances by a plausible tick delta: {first:?} → {second:?}"
        );

        // No firmware process outlives the test: the fixture kills and reaps on
        // drop, after the printer and the mux library have let go. The clock
        // and the mcu handle hold the child's interface — and through it the mux
        // library — so they go first, or the unload wait still finds the library
        // loaded.
        let pid = firmware.pid();
        drop(clock);
        drop(child);
        drop(printer);
        drop(teardown);
        drop(firmware);
        let deadline = Instant::now() + Duration::from_secs(5);
        while std::path::Path::new(&format!("/proc/{pid}")).exists() {
            assert!(
                Instant::now() < deadline,
                "firmware {pid} is still running after drop"
            );
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }

    /// One bridge with **two** ports online at once, each hung on its own real
    /// linux firmware process: `[uart_device uart0]` on the first firmware's
    /// tty, `[uart_device uart1]` on the second's, and a uuidless `[mcu child]`
    /// per port. Both children identify and read their clock over the one mux
    /// library at the same time — the config-driven end-to-end of the
    /// firmware's per-slot multi-stream.
    ///
    /// The port sections' **order** fixes their slots (slot = index), and the
    /// backend dedupes by slot, not by path — so both ports must name distinct
    /// ttys, or both slots would open the same one and prove nothing. The main
    /// proof that the multi-stream firmware is live: the old single-stream
    /// backend refused the second `config_uart` with `UART_OPEN_BUSY` and never
    /// reached `Ready`, so **both** ports coming up together is the point.
    #[tokio::test(flavor = "multi_thread")]
    async fn two_real_linux_firmware_ports_come_up_on_one_bridge() {
        use crate::core::klippy::cmd::{ClockSync, McuClock};
        use crate::core::klippy::printer::PrinterState;

        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // Two independent firmware processes, each with its own temporary pty
        // path — the distinct ttys the two slots need.
        let firmware0 =
            klipperx_test_support::LinuxFirmware::spawn().expect("the first firmware starts");
        let firmware1 =
            klipperx_test_support::LinuxFirmware::spawn().expect("the second firmware starts");
        assert_ne!(
            firmware0.serial_path(),
            firmware1.serial_path(),
            "the two ports need distinct ttys"
        );

        // The bridge keeps its build-baked tty as a live pty, as in the
        // single-port chain above.
        let baked = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&baked.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                baked.path.display()
            )
        });

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             uart_path: {}\n\
             [uart_device uart1]\nuart_mcu: bridge\nuart_oid: uart1\nuart_baud: 250000\n\
             uart_path: {}\n\
             [mcu child0]\nuart: uart0\n\
             [mcu child1]\nuart: uart1\n",
            library.display(),
            firmware0.serial_path().display(),
            firmware1.serial_path().display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor: Arc<dyn crate::core::klippy::reactor::Reactor> =
            Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(Arc::clone(&reactor))),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();
        printer.load_config(&config).expect("the config loads");

        // Both ports answer over the one library: the second `config_uart` the
        // old backend refused with UART_OPEN_BUSY is exactly what now succeeds.
        let capture = StderrCapture::start(&_serial, "two-ports");
        tokio::time::timeout(Duration::from_secs(30), printer.bring_up())
            .await
            .expect("both real firmwares answer every step of the chain");
        let stderr = capture.text();
        drop(capture);
        let state = printer.get_state_message();
        assert_eq!(
            state.category,
            PrinterState::Ready,
            "one bridge with both ports came up: {}",
            state.message
        );

        // Each port's own open line (the library's own stderr,
        // `third_party/klipper/src/host/bridge.c`) names its tty and its slot:
        // the section order fixed `uart0` to slot 0 and `uart1` to slot 1.
        for (firmware, slot) in [(&firmware0, 0), (&firmware1, 1)] {
            let prefix = format!(
                "bridge serial: {} open at 250000 baud (fd ",
                firmware.serial_path().display()
            );
            let suffix = format!(", slot {slot})");
            assert!(
                stderr
                    .lines()
                    .any(|line| line.starts_with(&prefix) && line.ends_with(&suffix)),
                "the port on {} opened at slot {slot}: {stderr}",
                firmware.serial_path().display()
            );
        }

        for name in ["mcu child0", "mcu child1"] {
            let child = printer
                .lookup_object_as::<McuObject>(name)
                .unwrap_or_else(|| panic!("the printer has [{name}]"))
                .mcu()
                .unwrap_or_else(|| panic!("{name} connected"));
            assert!(child.is_identified(), "{name} completed identify");

            // The snapshot is its own firmware's: the identify payload
            // travelled over that port (the linux build's 50 MHz constant;
            // the mux host library is built at 20 MHz).
            let status = printer
                .status_of(name, 0.0)
                .unwrap_or_else(|| panic!("{name} has a status snapshot"));
            let version = status
                .get("mcu_version")
                .and_then(|value| value.as_str())
                .unwrap_or_default();
            assert!(
                !version.is_empty(),
                "{name}'s own version came through identify: {status}"
            );
            assert_eq!(
                status["mcu_constants"]["CLOCK_FREQ"].as_f64(),
                Some(50_000_000.0),
                "{name}'s CLOCK_FREQ is the linux firmware's own constant: {status}"
            );

            // One `get_clock` round trip per port, and a second to see that
            // port's counter advance. The value is the low 32 bits (it wraps
            // at ~86 s at 50 MHz), so the pair is checked by its delta.
            let clock = McuClock::new(Arc::clone(&child), Arc::clone(&reactor));
            let first = tokio::time::timeout(Duration::from_secs(5), clock.get_clock())
                .await
                .unwrap_or_else(|_| panic!("{name}'s get_clock is answered over its port"))
                .unwrap_or_else(|e| panic!("get_clock after identify for {name}: {e}"));
            assert!(first.clock > 0, "{name}'s firmware counter is running");
            tokio::time::sleep(Duration::from_millis(20)).await;
            let second = tokio::time::timeout(Duration::from_secs(5), clock.get_clock())
                .await
                .unwrap_or_else(|_| panic!("{name}'s second get_clock is answered"))
                .unwrap_or_else(|e| panic!("get_clock twice for {name}: {e}"));
            let advanced = second.clock.wrapping_sub(first.clock);
            // The 5s `get_clock` timeout plus the 20ms sleep bound any
            // possible stall: ≈ (5s + 20ms) × 50 MHz ≈ 251M ticks.
            assert!(
                advanced > 0 && advanced < 251_000_000,
                "{name}'s counter advances by a plausible tick delta: {first:?} → {second:?}"
            );

            // The clock and the mcu handle hold the child's interface — and
            // through it the mux library — so they go before the teardown, or
            // its unload wait still finds the library loaded.
            drop(clock);
            drop(child);
        }

        // No firmware process outlives the test: the fixture kills and reaps
        // on drop, after the printer and the mux library have let go.
        let pids = [firmware0.pid(), firmware1.pid()];
        drop(printer);
        drop(teardown);
        drop(firmware0);
        drop(firmware1);
        let deadline = Instant::now() + Duration::from_secs(5);
        for pid in pids {
            while std::path::Path::new(&format!("/proc/{pid}")).exists() {
                assert!(
                    Instant::now() < deadline,
                    "firmware {pid} is still running after drop"
                );
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        }
    }

    /// A port whose tty is not there fails the **configuration**, naming the MCU
    /// and the reason — not the library init, and not a bridge that comes up
    /// without its port.
    ///
    /// The host library opens nothing while it loads; the stream is opened when
    /// the host configures the UART object (`third_party/klipper/src/uart.c`),
    /// which is this bring-up. So a tty that is missing surfaces as the
    /// firmware's own `shutdown`, and the bring-up reports it with the reason
    /// the firmware gave (`McuObject::connect_failure`).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_missing_port_tty_fails_the_configuration() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // No tty at the path this build baked in: the port cannot be opened.
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             [mcu child]\nuart: uart0\n",
            library.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(reactor)),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();
        printer.load_config(&config).expect("the config loads");

        // Bounded: the firmware fails the configuration on the first run, and a
        // stopped firmware is retried a fixed number of times before the
        // bring-up reports it.
        let _ = tokio::time::timeout(Duration::from_secs(60), printer.bring_up()).await;

        let state = printer.get_state_message();
        assert!(
            state.message.contains("MCU 'bridge'"),
            "the failure names the bridge: {}",
            state.message
        );
        assert!(
            state.message.contains("config_uart"),
            "the failure carries the firmware's reason: {}",
            state.message
        );
    }

    // -----------------------------------------------------------------------
    // The named shape: `resolve` no longer refuses it, and `open` sends both
    // commands in the one order that works.
    // -----------------------------------------------------------------------

    /// First position at which `needle` appears whole in `haystack` — how the
    /// pty assertions below find one handshake unit among the bridge's own
    /// admin fan-out.
    fn find_subsequence(haystack: &[u8], needle: &[u8]) -> Option<usize> {
        haystack
            .windows(needle.len())
            .position(|window| window == needle)
    }

    /// A named device (a `uuid:` on the child) is no longer refused by
    /// `resolve`: with the bridge not yet connected the error is the
    /// config-order one, which is only reached **after** the old uuid rule — so
    /// this pins that the refusal is gone and the rest of `resolve` still runs.
    ///
    /// The flags a named port resolves to are [`port_flags`]' answer, exercised
    /// by `a_named_single_port_sends_the_units_flag` below (a connected bridge is
    /// needed to see them through `resolve` itself).
    #[test]
    fn a_named_device_resolves_past_the_uuid_rule() {
        let printer = printer_with(
            "[mcu bridge]\ntest: dict=/does/not/matter\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n\
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
            !text.contains("uuid:"),
            "the named-device refusal must be gone: {text}"
        );
    }

    /// A single uart-port device that writes a `uuid:` answers the mux itself, so
    /// its port is not legacy: the flags byte says the device speaks units.
    #[test]
    fn a_named_single_port_sends_the_units_flag() {
        let groups = port_groups(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        );

        let flags = port_flags(&groups, "uart_device uart0");

        assert_eq!(flags, MUX_PORT_UNITS);
    }

    /// The order `open` exists for, over a real serial link on a pty: the
    /// bridge's route (`SET_PORT_NODEID`) first, then the named device's own
    /// handshake (`SET_KLIPPER_NODEID` and an assignment request per attempt).
    ///
    /// A named device behind a port is a **serial** link with no firmware on the
    /// far end, so every byte the pass writes is on the pty as it goes out —
    /// unlike the host-library path, where the bridge consumes the route command.
    /// Nothing answers, so `open` ends in its handshake timeout; the observable
    /// is the byte order, not a completed handshake (the end-to-end one is the
    /// fixture pass's).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_named_downstream_route_precedes_its_handshake_over_a_pty() {
        let pty = Pty::new();
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let named = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44];
        let mcu_id = 7u32;
        let link = BridgePortLink {
            transport: Transport::Serial {
                path: pty.path.display().to_string(),
                baud: serial::DEFAULT_BAUD,
                uuid: None,
            },
            index: 0,
            flags: MUX_PORT_UNITS,
            named: Some(named),
        };

        let result = open(&printer, &link, mcu_id);
        assert!(
            result.is_err(),
            "the pty never answers the assignment, so the handshake times out"
        );

        let mut expected = serial::set_port_node_id(0, mcu_id as u8, MUX_PORT_UNITS);
        expected.extend_from_slice(
            &serial::set_node_id_for(named, mcu_id).expect("id 7 is addressable"),
        );
        let request = serial::assignment_request(mcu_id).expect("id 7 is addressable");
        for _ in 0..serial::ASSIGN_ATTACH_ATTEMPTS {
            expected.extend_from_slice(&request);
        }

        let seen = read_pty(&pty.master, expected.len(), Duration::from_secs(3));
        assert_eq!(
            seen, expected,
            "the route unit first, then the named device's SET_KLIPPER_NODEID and a routed \
             request per attempt"
        );
    }

    /// The whole config-driven path for a **named** child: over the real bridge
    /// library and a real pty, `[mcu child] uart: uart0 uuid: …` comes up far
    /// enough that the child's naming handshake is carried to the bridge's port.
    ///
    /// The route command is the bridge's own business (`SET_PORT_NODEID` is
    /// consumed, never forwarded), so the pty does not show it — what it shows is
    /// its effect: the handshake's units, which the bridge can route **only**
    /// because the route was built first. Sent the other way round, the naming
    /// unit is still broadcast but the request is dropped with no port to reach,
    /// and the request assertion below is red.
    ///
    /// Nothing behind the pty answers, so the child's `open` ends in its
    /// handshake timeout and the bring-up with it; the port's bytes are the
    /// observable.
    #[tokio::test(flavor = "multi_thread")]
    async fn the_config_driven_named_route_carries_the_handshake_to_the_bridge_port() {
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
        let named = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44];
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             [mcu child]\nuart: uart0\nuuid: 998877665544\n",
            library.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(reactor)),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();
        printer.load_config(&config).expect("the config loads");

        // Bounded: the child's handshake times out (nothing answers the pty),
        // which fails its `open` and the bring-up with it.
        let _ = tokio::time::timeout(Duration::from_secs(6), printer.bring_up()).await;

        let child_id = child_config(&printer, "child")
            .id
            .expect("the loader allocated the child an id");
        let naming =
            serial::set_node_id_for(named, child_id).expect("the child's id is addressable");
        let request = serial::assignment_request(child_id).expect("the child's id is addressable");

        let bytes = read_pty(
            &pty.master,
            naming.len() + request.len(),
            Duration::from_secs(3),
        );
        let naming_at = find_subsequence(&bytes, &naming).unwrap_or_else(|| {
            panic!(
                "the port must receive the child's SET_KLIPPER_NODEID unit (the bridge broadcasts \
                 it with its admin header); it got {bytes:02x?}"
            )
        });
        assert!(
            find_subsequence(&bytes[naming_at..], &request).is_some(),
            "the routed assignment request must follow the naming unit — and it only reaches the \
             port because SET_PORT_NODEID built the route first; it got {bytes:02x?}"
        );
    }

    // -----------------------------------------------------------------------
    // The tty a port opens is the host's to say at run time
    // (`[uart_device <name>] uart_path:`, third_party/klipper/src/host/).
    // -----------------------------------------------------------------------

    /// A port opens the tty its `uart_path:` names, not the one the library was
    /// built with.
    ///
    /// Two ptys say which one it was: `runtime` is the port's `uart_path:`, and
    /// the path the library baked in (`CONFIG_HOST_BRIDGE_SERIAL_PATH`) is linked
    /// to the second. The bridge comes up over the library exactly as in
    /// `the_config_driven_legacy_route_reaches_the_bridge_port`, so the child's
    /// bare identify request has to arrive on the **runtime** pty while the other
    /// one stays silent. A host that ignored `uart_path:` would open the baked
    /// path, which turns both assertions red.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_ports_uart_path_is_the_tty_the_bridge_opens() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        let runtime = Pty::new();
        let baked = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&baked.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                baked.path.display()
            )
        });

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             uart_path: {}\n\
             [mcu child]\nuart: uart0\n",
            library.display(),
            runtime.path.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(reactor)),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();
        printer.load_config(&config).expect("the config loads");

        // Bounded: the child's identify never answers, so the bring-up is
        // interrupted once the child has certainly spoken.
        let _ = tokio::time::timeout(Duration::from_secs(6), printer.bring_up()).await;

        let bytes = read_pty(&runtime.master, 1, Duration::from_secs(3));
        assert!(
            identify_request(&bytes).is_some(),
            "the port's uart_path is the tty it opens, so the child's bare identify request \
             arrives there; the pty at {} got {bytes:02x?}",
            runtime.path.display()
        );
        // Enough time for the task loop to have opened anything it was going to.
        assert!(
            read_pty(&baked.master, 1, Duration::from_millis(500)).is_empty(),
            "the tty the library was built with must not be opened at all once the port names \
             its own"
        );
    }

    /// A port whose `uart_path:` is not there fails the **configuration**, and
    /// fails it because of the path the **host** named: the library's stderr line
    /// names that tty, and the firmware's own `config_uart` message is what the
    /// bring-up reports.
    ///
    /// The path the library was built with is linked to a live pty here, so a
    /// host that fell back to it would open that one and come up. The failure
    /// itself is therefore the evidence that the run-time path was adopted, and
    /// the stderr line says which tty was tried.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_missing_uart_path_tty_fails_the_configuration() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The build's own path is a live tty, so it is not the reason for the
        // failure below.
        let baked = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&baked.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                baked.path.display()
            )
        });

        let missing = "/nonexistent/klipperx-missing-port-tty";
        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n\
             uart_path: {missing}\n",
            library.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let teardown = BridgeTeardown::new(
            Arc::new(Printer::new(reactor)),
            library.clone(),
            bridge_path.clone(),
        );
        let printer = teardown.printer();
        printer.load_config(&config).expect("the config loads");

        // Bounded: the firmware fails the configuration on the first run, and a
        // stopped firmware is retried a fixed number of times before the
        // bring-up reports it.
        let capture = StderrCapture::start(&_serial, "uart-path");
        let _ = tokio::time::timeout(Duration::from_secs(60), printer.bring_up()).await;
        let stderr = capture.text();
        drop(capture);

        let state = printer.get_state_message();
        assert!(
            state.message.contains("MCU 'bridge'"),
            "the failure names the bridge: {}",
            state.message
        );
        assert!(
            state.message.contains("config_uart"),
            "the failure carries the firmware's reason: {}",
            state.message
        );
        assert!(
            stderr.contains(missing),
            "the library names the tty it could not open ({missing}), which is the host's path \
             and not the build's: {stderr}"
        );
    }
}
