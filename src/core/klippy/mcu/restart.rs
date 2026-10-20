//! Resetting a firmware before it is reconnected.
//!
//! A restart reconnects the MCU, and some boards need more than a reconnect:
//! `restart_method` names how the firmware itself is reset (upstream's
//! `MCURestartHelper`, `klippy/mcu.py:656`). The reset acts on the **transport**,
//! so it has to run while the device is closed — between parsing the config and
//! opening it, which is exactly where [`McuObject::connect`] calls this.
//!
//! | method | here | elsewhere |
//! |---|---|---|
//! | `command` | nothing before the port opens | `reset` on the **live** connection, from `McuObject::before_firmware_restart`, before the parts come down; `config_reset` from the handshake (`mcu/object.rs`, `mcu/config.rs`) |
//! | `admin` | on a **serial** MCU the bridge restarts **itself**: its own `RESTART_DEVICE` goes out on the mux admin channel ([`bridge_admin_restart`]); on a bridge-port device nothing here — the port-level `RESTART_DEVICE` the bridge forwards ([`reset_bridge_firmware`]) | — |
//! | `arduino` | toggle DTR at 2400 baud — **untested on hardware**, and reported | — |
//! | `cheetah` | RTS/DTR sequence at 2400 baud — **untested on hardware**, and reported; the connection itself also needs RTS deasserted (`McuConfig::open`) | — |
//! | `rpi_usb` | cut and restore the USB port's power — **untested on hardware**, and reported | `interface/usb.rs` |
//!
//! A device behind a bridge downstream port is a fourth shape: it has no
//! transport of its own, so its reset travels the **bridge's** link and is chosen
//! by [`McuConfig::bridge_restart`](crate::core::klippy::config::mcu::McuConfig)
//! ([`reset_bridge_firmware`]). The physical methods above act on a transport; a
//! bridge-port device's does not exist until the attach pass opens it.
//!
//! The reset paths cannot be exercised in a test — a pty does not emulate the
//! modem lines, and there is no USB port to cut — so each one reports that it is
//! untested when it runs. That warning comes out once someone has confirmed the
//! sequence on a board.
//!
//! [`McuObject::connect`]: super::object::McuObject::connect

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use tokio::time::Duration;
use tracing::{debug, info, warn};

use crate::core::klippy::config::mcu::{BridgeRestart, McuConfig, Transport};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::interface::devices::serial::{self, ModemLines};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::usb;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::bridge_attach::{self, BridgePortLink};
use crate::core::klippy::mcu::McuRestartMethod;
use crate::core::klippy::printer::Printer;

/// How long to leave the board between the steps of a reset. Upstream pauses
/// `0.100` s between each (`klippy/serialhdl.py:365-402`).
const RESET_SETTLE: Duration = Duration::from_millis(100);

/// The line speed Klipper opens at for a reset. A different rate is part of what
/// the board's USB-serial bridge notices (`serialhdl.py:369` `:394`).
const RESET_BAUD: u32 = 2400;

/// How long the USB port stays unpowered. Upstream pauses two seconds
/// (`klippy/mcu.py:752`).
const USB_POWER_OFF: Duration = Duration::from_secs(2);

/// How long to wait for the device to re-enumerate after the port is powered
/// back on.
const USB_PORT_RETURN_TIMEOUT: Duration = Duration::from_secs(10);

/// How long to give re-enumeration when the device's node number could not be
/// read beforehand, so there is nothing to compare against.
const RE_ENUMERATE_SETTLE: Duration = Duration::from_millis(1000);

/// How long to wait for the bridge to answer the attach handshake, per attempt.
/// The serial layer's own budget for the same handshake.
const BRIDGE_ATTACH_TIMEOUT: Duration = serial::ASSIGN_ATTACH_TIMEOUT;

/// The index that never names a port, so the admin unit carrying it is the
/// **bridge's own**: the firmware's port table has at most eight entries, and an
/// index outside it falls through to the device's own handler
/// (`src/mux.c::mux_admin_unit`), whose `RESTART_DEVICE` arm calls
/// `mcu_restart()` on that device. `src/flash` spells the same convention as
/// `RESTART_DEVICE_TO_BRIDGE`.
const BRIDGE_ITSELF: u8 = 0xFF;

/// Reset the firmware on the transport that is about to be opened.
///
/// Called by `McuObject::connect` when the bring-up follows a
/// `firmware_restart`, before [`McuConfig::open`].
///
/// # Errors
/// Returns a message prefixed with the transport when the reset itself fails, so
/// it reads the same way the open errors do.
pub async fn reset_firmware(
    config: &McuConfig,
    handle: &tokio::runtime::Handle,
) -> Result<(), String> {
    match config.restart_method {
        // Nothing before the connection: `config_reset` clears the firmware once
        // it is up (`mcu/config.rs`).
        McuRestartMethod::Command => Ok(()),
        // The device resets itself, and the mux admin channel is how it is told
        // — which channel that is depends on the transport. A **bridge-port**
        // device's restart travels the bridge's link, before this device is
        // reached (`McuObject::connect`, `reset_bridge_firmware`), so there is
        // nothing to do on a transport of its own. A **serial** MCU is the
        // bridge itself: its admin channel is its own link, reached out of band
        // because this device's session does not exist yet.
        McuRestartMethod::Admin => match &config.transport {
            Transport::Serial { path, baud, uuid } => {
                let mcu_id = config.id.ok_or_else(|| {
                    format!(
                        "serial {path}: restart_method 'admin' needs the MCU id, which has not \
                         been assigned"
                    )
                })?;
                bridge_admin_restart(path, *baud, *uuid, mcu_id, handle).await
            }
            Transport::BridgePort { .. } => Ok(()),
            // `parse_restart_method` only yields `admin` for the two shapes above.
            _ => Ok(()),
        },
        McuRestartMethod::Arduino => match &config.transport {
            Transport::Serial { path, .. } => {
                warn_untested(config);
                arduino_reset(path).await
            }
            // `parse_restart_method` only yields a serial method for a serial MCU.
            _ => Ok(()),
        },
        McuRestartMethod::Cheetah => match &config.transport {
            Transport::Serial { path, .. } => {
                warn_untested(config);
                cheetah_reset(path).await
            }
            _ => Ok(()),
        },
        // Verified on hardware: the hub reports `PORT_POWER` off and the board
        // re-enumerates under a new node, so no "untested" warning here.
        McuRestartMethod::RpiUsb => match &config.transport {
            Transport::Serial { path, .. } => rpi_usb_reset(path, config.usb_power, handle).await,
            _ => Ok(()),
        },
    }
}

/// Reset a device that hangs on a bridge downstream port.
///
/// Such a device has no transport of its own: it is reached on the bridge's link
/// (`mcu/bridge_attach.rs`), and its reset travels the same way. Which reset is
/// [`McuConfig::bridge_restart`]'s answer — `Firmware` has the device restart
/// its own firmware (`RESTART_DEVICE`, which the bridge forwards to the port),
/// `Port` asks the bridge to pulse the port's lines (`RESET_PORT`). Both need
/// the port's `index`, which only the bridge's firmware port table has:
/// [`bridge_attach::resolve`]
/// is where that is reconciled, so an old firmware (no port table), or a
/// section whose oid the bridge does not report, is refused there before
/// anything is sent.
///
/// The command goes out on a **temporary** downstream handle
/// ([`open_bridge_link`]) — the bridge's admin channel, with no route sent and
/// no handshake awaited. The handle is dropped right after and leaves the link
/// as it found it; the attach pass opens the device's real reference when it
/// connects.
///
/// Called by `McuObject::connect` on the bring-up that follows a
/// `FIRMWARE_RESTART`, before the transport opens — the same place
/// [`reset_firmware`] runs, and the only place a bridge-port device's reset can
/// go out before its route is established.
///
/// # Errors
/// [`KlippyError::Config`] from [`bridge_attach::resolve`] — the section is not
/// a bridge port, the bridge has not connected, its firmware reports no port
/// table (an old firmware, or not a bridge build), or the port is missing or of
/// another type; [`KlippyError::Connection`] when the downstream handle cannot
/// be opened or the command cannot be written.
pub(crate) fn reset_bridge_firmware(
    printer: &Printer,
    config: &McuConfig,
    restart: BridgeRestart,
) -> Result<(), KlippyError> {
    let link = bridge_attach::resolve(printer, config)?;
    let mcu_id = config.id.ok_or_else(|| {
        KlippyError::Connection(format!(
            "MCU '{}' has no id assigned; cannot reset its bridge port",
            config.name
        ))
    })?;
    let interface = open_bridge_link(printer, &link, mcu_id)?;
    let sent = match restart {
        BridgeRestart::Firmware => device_reset(&interface, &link, mcu_id),
        BridgeRestart::Port => interface.send_reset_port(link.index),
    };
    sent.map_err(|err| {
        KlippyError::Connection(format!(
            "MCU '{}': sending the bridge port reset failed: {err}",
            config.name
        ))
    })?;
    info!(
        "MCU '{}': resetting bridge port {} with {}",
        config.name,
        link.index,
        match restart {
            BridgeRestart::Firmware => "RESTART_DEVICE",
            BridgeRestart::Port => "RESET_PORT",
        }
    );
    Ok(())
}

/// The two commands a **named** child's restart needs, in order.
///
/// A restart reaches the device only when the bridge forwards the unit
/// **whole**, and whether it does depends on the port's flags — bridge-side
/// state the host cannot assume: a bridge that has not been told where this id
/// lives still has the port in its default legacy mode and would strip the unit
/// header off, leaving the device a bare KXID frame its parser rejects, so the
/// restart would never happen. The route command is idempotent ([`mux.h`]: no
/// receipt, no copy downstream — the bridge consumes it), so saying it again is
/// the cheap way to be sure.
///
/// [`mux.h`]: https://github.com/Klipper3d/klipper
fn device_reset(
    interface: &Interface,
    link: &BridgePortLink,
    mcu_id: u32,
) -> Result<(), InterfaceError> {
    interface.send_set_port_node_id(link.index, mcu_id as u8, link.flags)?;
    interface.send_restart_device(link.index)
}

/// Open a temporary downstream handle on `link`'s bridge link, without a route.
///
/// This is the bridge's own transport reopened for one device — a second slot on
/// the shared serial port, or a second reference on the live host library — and
/// it is only here to reach the bridge's **admin** channel. The route the attach
/// pass sends (`SET_PORT_NODEID`) is deliberately left out: the device is about
/// to reboot, and the reference that will carry it is opened, and routed, when
/// it connects. Dropping this handle leaves both link kinds in place.
///
/// The opening itself is [`bridge_attach::open_link`]'s, so a link that cannot
/// carry a downstream device (a CAN upstream, or a transport with no mux) is
/// refused here with the attach pass's own words.
///
/// # Errors
/// [`KlippyError::Config`] for a bridge link that cannot carry a downstream
/// device (a CAN upstream, or a transport with no mux);
/// [`KlippyError::Connection`] for a failed open.
fn open_bridge_link(
    printer: &Printer,
    link: &BridgePortLink,
    mcu_id: u32,
) -> Result<Interface, KlippyError> {
    bridge_attach::open_link(printer, link, mcu_id)
}

/// Whether an `rpi_usb` board has to be brought up with a firmware restart
/// first, and the reason to log.
///
/// Upstream gates `rpi_usb` at two points (`klippy/mcu.py:690-700`):
///
/// - `check_restart_on_attach` — a port that is not even present is powered on
///   first;
/// - `check_restart_on_send_config` — an `rpi_usb` board is only ever
///   **configured** after a power cycle in this session, so a normal start
///   restarts it before sending the configuration.
///
/// The point of `rpi_usb` is a power cycle, and a board that answers without
/// having been power-cycled this session may still be running an old
/// configuration — so the host asks for the restart instead of configuring it.
///
/// Returns `None` when the bring-up should just proceed: not `rpi_usb`, already
/// in the restart the gate asked for (`restarting`), or (per
/// [`check_usb_power`]) a hub that cannot switch power at all — the caller
/// downgrades that to `command` before calling this.
pub fn restart_before_bringup(config: &McuConfig, restarting: bool) -> Option<&'static str> {
    if restarting || config.restart_method != McuRestartMethod::RpiUsb {
        return None;
    }
    match &config.transport {
        Transport::Serial { path, .. } if !Path::new(path).exists() => Some("enable power"),
        _ => Some("full reset before config"),
    }
}

/// Check, at connect, that `rpi_usb` will be able to switch this port's power.
///
/// Runs on every connect rather than only when a restart is requested, so a
/// missing udev rule is reported at startup — with the rules to install —
/// instead of at the first `FIRMWARE_RESTART`.
///
/// The reading happens off the runtime ([`hub_report`]): the sysfs walk, the
/// hub's class descriptor and the mechanism probe are `nusb`'s blocking path, and
/// this runs on the connect path. A port that resolves is also
/// [recorded](HubPorts) for [`rpi_usb_reset`], which needs those coordinates in
/// the case this method exists for.
///
/// # Returns
/// The reason this port's power **cannot be switched at all**, when the hub says
/// so itself ([`usb::PortPower::Unsupported`]): switching it would only disconnect
/// the board, which no mechanism can turn into a firmware reset, so the caller
/// drops `rpi_usb` for `command` instead of trying (`mcu/object.rs`). `None` when
/// the mechanism is usable, the method is not `rpi_usb`, or nothing could be found
/// out about the hub.
pub async fn check_usb_power(
    config: &McuConfig,
    handle: &tokio::runtime::Handle,
) -> Option<String> {
    if config.restart_method != McuRestartMethod::RpiUsb {
        return None;
    }
    let Transport::Serial { path, .. } = &config.transport else {
        return None;
    };

    let tty = PathBuf::from(path);
    let method = config.usb_power;
    let report = match handle
        .spawn_blocking(move || hub_report(&tty, method))
        .await
    {
        Ok(report) => report,
        Err(err) => Err(format!("usb: {err}")),
    };
    let report = match report {
        Ok(report) => report,
        Err(error) => {
            warn_power_may_not_switch(&config.name, &error);
            return None;
        }
    };
    // The coordinates are only readable while the board is: keep the ones this
    // connect saw, for the reset that has to power it on when it is not.
    remember_hub_port(path, &report.port);

    // Ask the hub before anything else: a hub without power switching makes both
    // mechanisms pointless, so neither the rule below nor the switch is worth
    // reporting or attempting.
    match report.power {
        Some(usb::PortPower::Unsupported) => {
            return Some(format!(
                "hub {}:{} reports no port power switching (wHubCharacteristics)",
                report.port.bus, report.port.device
            ));
        }
        Some(usb::PortPower::Ganged) => warn!(
            "MCU '{}': hub {}:{} switches the power of all its ports together \
             (ganged), so anything else on that hub goes down with this port",
            config.name, report.port.bus, report.port.device
        ),
        _ => (),
    }
    match report.probe {
        Ok(power) => {
            debug!(
                "MCU '{}' will switch USB port power via {power:?}",
                config.name
            );
            None
        }
        Err(error) => {
            warn_power_may_not_switch(&config.name, &error);
            None
        }
    }
}

/// What [`check_usb_power`] needs to know about the port, all of it read in the
/// one blocking block.
struct HubReport {
    /// The hub port the tty resolved to.
    port: usb::UsbPort,
    /// The hub's own answer about switching this port's power.
    power: Option<usb::PortPower>,
    /// The mechanism, or why neither is usable — with the rules that would make
    /// one usable.
    probe: Result<usb::UsbPower, String>,
}

/// Resolve the tty's hub port, ask the hub what it can do, and probe the
/// mechanism.
///
/// The three steps are one blocking block rather than three: each is a sysfs walk
/// or a `nusb` blocking call ([`usb::resolve_tty_port`], [`usb::port_power`],
/// [`usb::probe`], and the descriptor reads behind [`usb::recommended_rules`]), so
/// the thread is parked once and the caller's worker is never the one parked. The
/// error text is completed here too, where the rules can still be read off the
/// port.
fn hub_report(tty: &Path, method: usb::UsbPowerMethod) -> Result<HubReport, String> {
    let port = usb::resolve_tty_port(tty)?;
    let power = usb::port_power(&port);
    let probe = usb::probe(&port, method).map_err(|err| {
        format!(
            "{err}\n  install a udev rule, for example:\n    {}",
            usb::recommended_rules(&port).replace('\n', "\n    ")
        )
    });
    Ok(HubReport { port, power, probe })
}

/// The warning an `rpi_usb` port that cannot be switched gets: the method stays,
/// and the next connect reports again.
fn warn_power_may_not_switch(name: &str, error: &str) {
    warn!(
        "MCU '{name}' may not be able to switch the USB port power (restart_method 'rpi_usb'): \
         {error}"
    );
}

// ===========================================================================
// The hub port a configured serial path was last resolved to
// ===========================================================================

/// The hub coordinates each configured `serial:` path was last resolved to.
///
/// [`rpi_usb_reset`] needs a [`usb::UsbPort`] to switch, and resolving it from
/// the tty is the one step that cannot work when the method is most needed: a
/// board whose tty is gone — upstream's `enable power` gate,
/// [`restart_before_bringup`] — has no `/sys/class/tty` entry to walk, so a reset
/// that only resolved would never power it back on, and the board would stay off.
/// The coordinates are therefore remembered whenever they *can* be read (every
/// connect, [`check_usb_power`], and every reset that resolves) and used as the
/// fallback when the tty is missing.
///
/// Keyed by the configured path — the one name both sides have: the config's
/// `serial:` string is what [`reset_firmware`] was handed, and it goes on naming
/// the device while the kernel node comes and goes (`/dev/serial/by-id/…`
/// especially). A path never resolved has no entry, and the reset then reports the
/// missing coordinates instead of promising a power switch it cannot make.
///
/// Process-wide: the reset runs far from the connect that recorded it, and there
/// is one of each per process. Stale coordinates are only ever used when the tty
/// resolves to nothing — a hub that moved, or a board replugged onto another port,
/// is picked up by the resolution whenever there is one.
struct HubPorts {
    /// The coordinates, by configured `serial:` path.
    ports: BTreeMap<String, usb::UsbPort>,
}

impl HubPorts {
    /// An empty table.
    const fn new() -> Self {
        Self {
            ports: BTreeMap::new(),
        }
    }

    /// The coordinates recorded for `path`, if any.
    fn recall(&self, path: &str) -> Option<usb::UsbPort> {
        self.ports.get(path).cloned()
    }

    /// Record `port` as the coordinates `path` last resolved to.
    fn remember(&mut self, path: &str, port: &usb::UsbPort) {
        self.ports.insert(path.to_string(), port.clone());
    }
}

/// The process-wide [`HubPorts`].
static HUB_PORTS: Mutex<HubPorts> = Mutex::new(HubPorts::new());

/// Run `with` on [`HUB_PORTS`], poison notwithstanding: a panic elsewhere must not
/// cost an `rpi_usb` reset the coordinates it needs to power a board back on.
fn with_hub_ports<T>(with: impl FnOnce(&mut HubPorts) -> T) -> T {
    let mut ports = HUB_PORTS
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    with(&mut ports)
}

/// Record `port` as the coordinates `path` resolved to.
fn remember_hub_port(path: &str, port: &usb::UsbPort) {
    with_hub_ports(|ports| ports.remember(path, port));
}

/// The coordinates `path` was last seen to resolve to, if any were recorded.
fn recorded_hub_port(path: &str) -> Option<usb::UsbPort> {
    with_hub_ports(|ports| ports.recall(path))
}

/// Report that a reset path has never run against hardware.
///
/// A pty does not emulate the modem lines (`TIOCMBIS` is `ENOTTY` there), so
/// neither sequence can be exercised in a test. Say so on every run until
/// someone confirms one on a board.
fn warn_untested(config: &McuConfig) {
    warn!(
        "MCU '{}' restart_method '{}' has not been tested on real hardware; resetting anyway",
        config.name,
        config.restart_method.as_str()
    );
}

/// A transport error as the reset paths report it: prefixed like an open error,
/// so a failed reset reads the same way a failed open does.
fn serial_error(error: InterfaceError) -> String {
    format!("serial: {error}")
}

/// Restart the **bridge itself** over its mux admin channel.
///
/// The bridge is this MCU, and its firmware's own `RESTART_DEVICE` handler calls
/// `mcu_restart()` — so the frame that asks for it *is* the reset. There is no
/// session to send a `reset` command on and no identify to run: the admin id is
/// where a device with no id listens (`src/mux.c::mux_unit`).
///
/// Off the runtime: the attach blocks on the mux handshake, and that is a wait,
/// not work — the same reason `usb_set_power` leaves the runtime.
async fn bridge_admin_restart(
    path: &str,
    baud: u32,
    uuid: Option<[u8; 6]>,
    mcu_id: u32,
    handle: &tokio::runtime::Handle,
) -> Result<(), String> {
    let path = PathBuf::from(path);
    handle
        .spawn_blocking(move || send_bridge_restart(&path, baud, uuid, mcu_id))
        .await
        .map_err(|e| format!("serial: {e}"))?
}

/// Open the bridge's link, send its own restart, and let the link go.
///
/// The handle comes from [`serial::attach`] with the section's `uuid`: that is
/// the opening the shared-serial transport does, so a `uuid:` names the bridge
/// as `mcu_id` (`SET_NODEID`) before the assignment request and the request is
/// answerable whatever id the firmware was left holding.
///
/// A bridge whose firmware holds **no** node id answers nothing to that request:
/// the mux's admin handler replies to it with the id it already has, and an
/// unassigned one has none to reply with (`src/mux.c::mux_admin_unit`). That is
/// the state an **exclusive** path leaves the mux in — the host never named it
/// there — so an unanswered handshake falls back to
/// [`serial::attach_downstream`]: the same port, registered without a handshake
/// and writing nothing, and the admin unit goes out all the same, because the
/// admin id is exactly where a device with no id is listening. Every other
/// attach failure is the open's own and is reported.
///
/// Nothing waits for the bridge to come back: the bring-up that follows
/// ([`McuConfig::open`], retried on a firmware restart) is what meets it.
///
/// Dropping the handle — the last reference — unregisters its slot and closes
/// the port (`Drop for SharedSerialHandle` / `Drop for SharedSerialPort` stop the
/// reader on the way out), so the open that follows finds the tty as it was.
fn send_bridge_restart(
    path: &Path,
    baud: u32,
    uuid: Option<[u8; 6]>,
    mcu_id: u32,
) -> Result<(), String> {
    let handle = match serial::attach(path, baud, mcu_id, BRIDGE_ATTACH_TIMEOUT, uuid) {
        Ok(handle) => handle,
        Err(InterfaceError::Timeout) => {
            serial::attach_downstream(path, baud, mcu_id).map_err(serial_error)?
        }
        Err(err) => return Err(serial_error(err)),
    };
    handle
        .send_restart_device(BRIDGE_ITSELF)
        .map_err(serial_error)?;
    info!(
        "bridge {} restarted over its admin channel (index {BRIDGE_ITSELF:#04x})",
        path.display()
    );
    Ok(())
}

/// Toggle DTR at another rate — Klipper's `arduino_reset`
/// (`klippy/serialhdl.py:392`).
///
/// The port is opened here and dropped at the end, so it must not be open
/// anywhere else; `connect` calls this before [`McuConfig::open`].
async fn arduino_reset(path: &str) -> Result<(), String> {
    let port = ModemLines::open(path, RESET_BAUD).map_err(serial_error)?;
    // Drain first, the way upstream does: a byte already in flight must not be
    // left to be read as the board's answer to the reset.
    port.drain();
    tokio::time::sleep(RESET_SETTLE).await;
    port.set_dtr(true).map_err(serial_error)?;
    tokio::time::sleep(RESET_SETTLE).await;
    port.set_dtr(false).map_err(serial_error)?;
    tokio::time::sleep(RESET_SETTLE).await;
    info!("serial port {path} reset by DTR toggle");
    Ok(())
}

/// The Fysetc Cheetah sequence — Klipper's `cheetah_reset`
/// (`klippy/serialhdl.py:365`).
///
/// Those boards have stateful bootloader circuitry, and this RTS/DTR dance is
/// what disarms it: open at another rate **with RTS asserted**, then toggle DTR
/// twice with RTS deasserted between the toggles. The connection that follows
/// also needs RTS deasserted (`McuConfig::open`).
async fn cheetah_reset(path: &str) -> Result<(), String> {
    let port = ModemLines::open(path, RESET_BAUD).map_err(serial_error)?;
    // Upstream opens with RTS already asserted (`serialhdl.py:374`).
    port.set_rts(true).map_err(serial_error)?;
    port.drain();
    tokio::time::sleep(RESET_SETTLE).await;

    // Toggle DTR.
    port.set_dtr(true).map_err(serial_error)?;
    tokio::time::sleep(RESET_SETTLE).await;
    port.set_dtr(false).map_err(serial_error)?;
    tokio::time::sleep(RESET_SETTLE).await;

    // Deassert RTS.
    port.set_rts(false).map_err(serial_error)?;
    tokio::time::sleep(RESET_SETTLE).await;

    // Toggle DTR again.
    port.set_dtr(true).map_err(serial_error)?;
    tokio::time::sleep(RESET_SETTLE).await;
    port.set_dtr(false).map_err(serial_error)?;
    tokio::time::sleep(RESET_SETTLE).await;

    info!("serial port {path} reset Cheetah-style");
    Ok(())
}

/// Cut and restore the USB port's power — Klipper's `rpi_usb` restart
/// (`klippy/mcu.py:748`).
///
/// Upstream shells out to a compiled `hub-ctrl` with `sudo`; here the port is
/// read from sysfs and switched the way `usb_power` asks (`interface/usb.rs`).
/// The port must be closed, and the caller opens it afterwards.
///
/// The hub coordinates come from the tty when it is there — the normal case, and
/// the only one that follows a hub that moved — and from [`HubPorts`] when it is
/// not: a board with no tty is exactly the one this method is asked to power on
/// ([`restart_before_bringup`]), and its coordinates were recorded by the connect
/// that still had a node to read. With neither, the reset says so — the board has
/// to be powered on by hand once, which the next connect then records.
async fn rpi_usb_reset(
    serial_path: &str,
    method: usb::UsbPowerMethod,
    handle: &tokio::runtime::Handle,
) -> Result<(), String> {
    let tty = PathBuf::from(serial_path);
    let resolved = usb_resolve_tty_port(&tty, handle).await;
    let port = reset_port(serial_path, resolved, recorded_hub_port(serial_path))?;
    remember_hub_port(serial_path, &port);
    // Resolve the mechanism per reset, so a hub that was replugged (or a udev
    // rule that only now applies) is picked up again.
    let power = usb_probe(&port, method, handle).await?;
    // The device's own node number, to tell the old instance from the one that
    // comes back.
    let before = device_node(&tty);

    usb_set_power(&port, &power, false, handle).await?;
    tokio::time::sleep(USB_POWER_OFF).await;
    power_on(&port, &power, handle).await?;

    // The caller is about to open the port, so wait for the board to be back
    // rather than opening the instance that is still on its way out.
    wait_for_new_device(&tty, before, USB_PORT_RETURN_TIMEOUT).await
}

/// The hub port an `rpi_usb` reset switches: the tty's, or the coordinates
/// recorded for this path at an earlier connect.
///
/// Split out because the choice is the whole point of [`HubPorts`] and needs no
/// hub to test. The tty wins whenever it resolves — it is the fresher answer, and
/// the only one that can see a board moved to another port — and the recorded
/// coordinates are used when it does not, because a board whose node is gone is
/// precisely the one a power switch is worth trying.
fn reset_port(
    path: &str,
    resolved: Result<usb::UsbPort, String>,
    recorded: Option<usb::UsbPort>,
) -> Result<usb::UsbPort, String> {
    match resolved {
        Ok(port) => Ok(port),
        Err(error) => match recorded {
            Some(port) => {
                warn!(
                    "usb: {error}; using the hub port recorded for {path} at the last connect \
                     (hub {}:{} port {}), so this reset switches that one",
                    port.bus, port.device, port.port
                );
                Ok(port)
            }
            None => Err(format!(
                "{error}\n  no hub port is recorded for that path either, so its power cannot be \
                 switched: it has to be powered on by hand once, and the next connect will \
                 record where it is"
            )),
        },
    }
}

/// How many times the port is switched back on before the reset gives up.
///
/// Switching a port's power is idempotent (there is nothing to advance, only a
/// level to set), and what fails is transient — a hub still settling after the
/// port went down. Giving up here leaves the board **powered off**, so the few
/// retries are worth more than the failure they avoid.
const POWER_ON_ATTEMPTS: usize = 3;

/// How long to wait between those attempts.
const POWER_ON_RETRY: Duration = Duration::from_millis(100);

/// Switch the port back on, retrying a transient failure ([`POWER_ON_ATTEMPTS`]).
async fn power_on(
    port: &usb::UsbPort,
    power: &usb::UsbPower,
    handle: &tokio::runtime::Handle,
) -> Result<(), String> {
    retrying_power_on(|| usb_set_power(port, power, true, handle)).await
}

/// [`power_on`]'s retry loop, with the switch passed in.
///
/// Split out so the attempts and the message can be tested without a hub. The
/// last failure is what comes back, with the state it left the board in.
async fn retrying_power_on<F, Fut>(mut switch: F) -> Result<(), String>
where
    F: FnMut() -> Fut,
    Fut: std::future::Future<Output = Result<(), String>>,
{
    let mut last = String::new();
    for attempt in 1..=POWER_ON_ATTEMPTS {
        match switch().await {
            Ok(()) => {
                if attempt > 1 {
                    info!("usb: port powered back on at attempt {attempt}/{POWER_ON_ATTEMPTS}");
                }
                return Ok(());
            }
            Err(error) => {
                warn!(
                    "usb: powering the port back on failed \
                     (attempt {attempt}/{POWER_ON_ATTEMPTS}): {error}"
                );
                last = error;
                if attempt < POWER_ON_ATTEMPTS {
                    tokio::time::sleep(POWER_ON_RETRY).await;
                }
            }
        }
    }
    Err(format!(
        "usb: the port did not power back on in {POWER_ON_ATTEMPTS} attempts: {last}; the board \
         is left powered off, so it has to be powered on by hand"
    ))
}

/// [`usb::resolve_tty_port`] off the runtime: the sysfs walk parks the caller's
/// worker for as long as the kernel takes to answer, which on a connect path is
/// not the caller's to spend.
async fn usb_resolve_tty_port(
    tty: &Path,
    handle: &tokio::runtime::Handle,
) -> Result<usb::UsbPort, String> {
    let tty = tty.to_path_buf();
    handle
        .spawn_blocking(move || usb::resolve_tty_port(&tty))
        .await
        .map_err(|e| format!("usb: {e}"))?
}

/// [`usb::probe`] off the runtime: the control route opens the hub and reads its
/// bits, and `nusb`'s blocking path is a wait, not work — the same reason
/// [`usb_set_power`] leaves the runtime.
async fn usb_probe(
    port: &usb::UsbPort,
    method: usb::UsbPowerMethod,
    handle: &tokio::runtime::Handle,
) -> Result<usb::UsbPower, String> {
    let port = port.clone();
    handle
        .spawn_blocking(move || usb::probe(&port, method))
        .await
        .map_err(|e| format!("usb: {e}"))?
}

/// Switch one hub port, off the runtime: `nusb`'s blocking path does the syscalls.
async fn usb_set_power(
    port: &usb::UsbPort,
    power: &usb::UsbPower,
    on: bool,
    handle: &tokio::runtime::Handle,
) -> Result<(), String> {
    let port = port.clone();
    let power = power.clone();
    handle
        .spawn_blocking(move || usb::set_port_power(&port, &power, on))
        .await
        .map_err(|e| format!("usb: {e}"))?
}

/// The USB device node (`major:minor`) behind a tty path, or `None` when the tty
/// or its device is not there.
///
/// This is the only signal that changes when a board re-enumerates. Neither the
/// `/dev` node nor the tty's sysfs link does: Linux keeps a device around when its
/// port is switched off through the hub, and only tears it down once the port is
/// powered again (measured: the node number went `189:40` → `189:41` across a
/// cycle, while both of the obvious signals stayed "present" the whole time).
///
/// The path is resolved on **every** call. A `by-id` path keeps naming the device
/// while it comes back under another tty, whereas `/sys/class/tty` is keyed by the
/// tty's own name — so resolving once, before the reset, would lose a device that
/// returned as `ttyACM1`.
fn device_node(tty: &Path) -> Option<String> {
    let real = fs::canonicalize(tty).ok()?;
    let name = real.file_name()?.to_str()?;
    let interface = fs::canonicalize(Path::new("/sys/class/tty").join(name).join("device")).ok()?;
    let usb = interface.parent()?;
    fs::read_to_string(usb.join("dev"))
        .ok()
        .map(|value| value.trim().to_string())
}

/// Wait until the device behind tty `name` has re-enumerated.
///
/// `before` is [`device_node`] read before the port went off; a device whose
/// number differs is the fresh instance, and the caller may open it. When
/// `before` could not be read, a fixed settle time is used instead of waiting
/// forever for a change that cannot be told.
///
/// This is what upstream's `check_restart_on_attach` achieves by restarting once
/// more while the port is missing (`klippy/mcu.py:693`); waiting here keeps it in
/// one place.
async fn wait_for_new_device(
    tty: &Path,
    before: Option<String>,
    timeout: Duration,
) -> Result<(), String> {
    let Some(before) = before else {
        tokio::time::sleep(RE_ENUMERATE_SETTLE).await;
        return Ok(());
    };
    let mut seen = Reenumeration::default();
    let deadline = tokio::time::Instant::now() + timeout;
    loop {
        if seen.done(device_node(tty).as_deref(), &before) {
            return Ok(());
        }
        if tokio::time::Instant::now() >= deadline {
            return Err(format!(
                "usb: {} did not come back within {timeout:?}",
                tty.display()
            ));
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

/// Tracks whether the device came back, from successive [`device_node`] reads.
///
/// A node number that differs is a fresh instance. But the old number can come
/// back too — the kernel may hand the freed number straight out again — so a
/// device that was **seen to disappear** and then reappear counts as well. The
/// wait starts after the port was powered on, so a disappearance can only belong
/// to this cycle.
#[derive(Default)]
struct Reenumeration {
    /// Whether the device was absent at some point since the wait began.
    gone: bool,
}

impl Reenumeration {
    /// Read `now`, and say whether the device is back.
    fn done(&mut self, now: Option<&str>, before: &str) -> bool {
        match now {
            Some(now) if self.gone || now != before => true,
            Some(_) => false,
            None => {
                self.gone = true;
                false
            }
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::port_grouping::PortKind;

    /// A config whose transport is never opened by these tests.
    fn no_transport(method: McuRestartMethod) -> McuConfig {
        McuConfig {
            name: "mcu".to_string(),
            restart_method: method,
            bridge_restart: None,
            transport: Transport::Host {
                library: String::new(),
                uuid: None,
            },
            id: None,
            shared_baud: None,
            host_mux: None,
            host_port_paths: Vec::new(),
            usb_power: usb::UsbPowerMethod::default(),
        }
    }

    /// A config that names a serial port, so the reset dispatch runs but the
    /// open fails — which is where the routing shows.
    fn serial(method: McuRestartMethod) -> McuConfig {
        McuConfig {
            name: "mcu".to_string(),
            restart_method: method,
            bridge_restart: None,
            transport: Transport::Serial {
                path: "/dev/not-a-serial-port".to_string(),
                baud: 250_000,
                uuid: None,
            },
            id: None,
            shared_baud: None,
            host_mux: None,
            host_port_paths: Vec::new(),
            usb_power: usb::UsbPowerMethod::default(),
        }
    }

    /// A config naming one serial path, for the `rpi_usb` gating test.
    fn serial_at(path: &str, method: McuRestartMethod) -> McuConfig {
        McuConfig {
            name: "mcu".to_string(),
            restart_method: method,
            bridge_restart: None,
            transport: Transport::Serial {
                path: path.to_string(),
                baud: 250_000,
                uuid: None,
            },
            id: None,
            shared_baud: None,
            host_mux: None,
            host_port_paths: Vec::new(),
            usb_power: usb::UsbPowerMethod::default(),
        }
    }

    #[test]
    fn test_rpi_usb_needs_a_restart_before_it_is_configured() {
        // The port is there, so the board is powered on: it still has to be
        // power-cycled before this session configures it (upstream's
        // `check_restart_on_send_config`).
        assert_eq!(
            restart_before_bringup(&serial_at("/dev/null", McuRestartMethod::RpiUsb), false),
            Some("full reset before config")
        );
        // A port that is not there at all is powered on first
        // (`check_restart_on_attach`).
        assert_eq!(
            restart_before_bringup(
                &serial_at("/dev/no-such-tty", McuRestartMethod::RpiUsb),
                false
            ),
            Some("enable power")
        );
    }

    #[test]
    fn test_the_restart_gate_does_not_ask_twice() {
        // Already in the restart it asked for: configure instead of looping.
        assert_eq!(
            restart_before_bringup(&serial_at("/dev/null", McuRestartMethod::RpiUsb), true),
            None
        );
        // Only `rpi_usb` has the gate.
        for method in [
            McuRestartMethod::Command,
            McuRestartMethod::Arduino,
            McuRestartMethod::Cheetah,
        ] {
            assert_eq!(
                restart_before_bringup(&serial_at("/dev/null", method), false),
                None
            );
        }
    }

    #[tokio::test]
    async fn test_command_leaves_the_transport_alone() {
        // `command` is reset after the connection is up, so nothing happens here.
        reset_firmware(
            &no_transport(McuRestartMethod::Command),
            &tokio::runtime::Handle::current(),
        )
        .await
        .unwrap();
    }

    #[test]
    fn test_the_reenumeration_state_machine() {
        // The number changed: back, without ever seeing it go.
        assert!(Reenumeration::default().done(Some("189:41"), "189:40"));

        // Still the old instance: not back.
        assert!(!Reenumeration::default().done(Some("189:40"), "189:40"));

        // Gone, then back with the *same* number: the kernel handed the freed one
        // out again, which is a re-enumeration all the same.
        let mut seen = Reenumeration::default();
        assert!(!seen.done(None, "189:40"));
        assert!(seen.done(Some("189:40"), "189:40"));

        // Gone, then back with a new number: also back, and the disappearance was
        // enough on its own.
        let mut seen = Reenumeration::default();
        assert!(!seen.done(None, "189:40"));
        assert!(seen.done(Some("189:42"), "189:40"));
    }

    #[tokio::test]
    async fn test_a_serial_reset_method_needs_a_serial_transport() {
        // Without a serial port there is nothing to toggle or power-cycle, so
        // the dispatch is a no-op rather than an error.
        for method in [
            McuRestartMethod::Arduino,
            McuRestartMethod::Cheetah,
            McuRestartMethod::RpiUsb,
        ] {
            reset_firmware(&no_transport(method), &tokio::runtime::Handle::current())
                .await
                .unwrap();
        }
    }

    #[tokio::test]
    async fn test_arduino_routes_to_the_serial_reset() {
        // The reset opens the port itself, so a path that cannot be opened is
        // where the dispatch shows: the error is the serial one, wrapped.
        let err = reset_firmware(
            &serial(McuRestartMethod::Arduino),
            &tokio::runtime::Handle::current(),
        )
        .await
        .unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");
    }

    #[tokio::test]
    async fn test_rpi_usb_needs_a_usb_tty() {
        // The port has to be a USB one for its power to be switched; the error
        // from the sysfs lookup is what comes back.
        let err = reset_firmware(
            &serial(McuRestartMethod::RpiUsb),
            &tokio::runtime::Handle::current(),
        )
        .await
        .unwrap_err();
        assert!(err.starts_with("usb: "), "{err}");
    }

    #[tokio::test]
    async fn test_the_startup_check_only_answers_for_rpi_usb_on_a_serial_port() {
        let handle = tokio::runtime::Handle::current();
        // Everything else has no port switch to judge, so nothing is reported and
        // nothing is taken away from the caller.
        assert_eq!(
            check_usb_power(&no_transport(McuRestartMethod::Command), &handle).await,
            None
        );
        assert_eq!(
            check_usb_power(&no_transport(McuRestartMethod::RpiUsb), &handle).await,
            None
        );
        // A path that is not a USB tty cannot say anything about a hub either:
        // the lookup reports itself and the caller keeps `rpi_usb`.
        assert_eq!(
            check_usb_power(&serial(McuRestartMethod::RpiUsb), &handle).await,
            None
        );
    }

    /// A resolved port, for the tests that need coordinates without a hub.
    fn hub_port(port: u8) -> usb::UsbPort {
        usb::UsbPort {
            bus: 1,
            device: 2,
            port,
            hub_dir: PathBuf::from(format!("/sys/devices/fake-hub/port{port}")),
        }
    }

    #[test]
    fn test_the_hub_port_table_is_keyed_by_the_configured_path() {
        let mut ports = HubPorts::new();
        // Nothing recorded yet: no fallback, which is what the reset reports.
        assert_eq!(ports.recall("/dev/ttyACM0"), None);

        ports.remember("/dev/ttyACM0", &hub_port(1));
        ports.remember("/dev/serial/by-id/board", &hub_port(2));
        assert_eq!(ports.recall("/dev/ttyACM0"), Some(hub_port(1)));
        assert_eq!(ports.recall("/dev/serial/by-id/board"), Some(hub_port(2)));
        // A path of its own, and an unknown one, are not answered by someone
        // else's coordinates.
        assert_eq!(ports.recall("/dev/ttyACM1"), None);

        // Recording again replaces, so the reset switches the last place seen.
        ports.remember("/dev/ttyACM0", &hub_port(3));
        assert_eq!(ports.recall("/dev/ttyACM0"), Some(hub_port(3)));
    }

    #[test]
    fn test_a_reset_falls_back_to_the_recorded_hub_port() {
        // The tty resolves: its answer is used, recorded or not (it is the only
        // one that can see a board that moved).
        assert_eq!(
            reset_port("/dev/ttyACM0", Ok(hub_port(1)), Some(hub_port(2))),
            Ok(hub_port(1))
        );
        // No tty — the `enable power` case this fallback exists for: the connect
        // that still had a node recorded where the port is.
        assert_eq!(
            reset_port(
                "/dev/ttyACM0",
                Err("usb: no node".to_string()),
                Some(hub_port(2))
            ),
            Ok(hub_port(2))
        );
        // Neither: the error says the coordinates are missing instead of
        // promising a power switch that cannot be made.
        let err = reset_port("/dev/ttyACM0", Err("usb: no node".to_string()), None).unwrap_err();
        assert!(err.starts_with("usb: no node"), "{err}");
        assert!(err.contains("no hub port is recorded"), "{err}");
        assert!(err.contains("powered on by hand"), "{err}");
    }

    #[tokio::test]
    async fn test_powering_the_port_back_on_is_retried() {
        // A failure part way through is transient: the next attempt is the one
        // that leaves the board powered on.
        let mut calls = 0u8;
        let result = tokio::time::timeout(Duration::from_secs(5), async {
            retrying_power_on(|| {
                calls += 1;
                std::future::ready(if calls < 3 {
                    Err(format!("usb: attempt {calls} failed"))
                } else {
                    Ok(())
                })
            })
            .await
        })
        .await
        .expect("the retries must not hang");
        assert_eq!(result, Ok(()));
        assert_eq!(calls, 3);

        // Every attempt failing: the board is left off, and that is said.
        let mut calls = 0u8;
        let err = tokio::time::timeout(Duration::from_secs(5), async {
            retrying_power_on(|| {
                calls += 1;
                std::future::ready(Err("usb: switch refused".to_string()))
            })
            .await
        })
        .await
        .expect("the retries must not hang")
        .unwrap_err();
        assert_eq!(calls as usize, POWER_ON_ATTEMPTS);
        assert!(err.contains("did not power back on in 3 attempts"), "{err}");
        assert!(err.contains("usb: switch refused"), "{err}");
        assert!(err.contains("left powered off"), "{err}");
    }

    #[tokio::test]
    async fn test_cheetah_routes_to_the_serial_reset() {
        let err = reset_firmware(
            &serial(McuRestartMethod::Cheetah),
            &tokio::runtime::Handle::current(),
        )
        .await
        .unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");
    }

    /// `admin` on a serial MCU resets the **bridge itself**, so the dispatch
    /// opens its port instead of doing nothing: a path that cannot be opened is
    /// reported, never passed over as a no-op (the bridge would then come up
    /// unrestarted).
    #[tokio::test]
    async fn test_admin_on_a_serial_mcu_resets_the_bridge_itself() {
        let mut config = serial(McuRestartMethod::Admin);
        config.id = Some(1);
        let err = reset_firmware(&config, &tokio::runtime::Handle::current())
            .await
            .unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");

        // The admin unit is addressed to an id, so the dispatch needs one: the
        // bring-up assigns it before this runs, and a config without it says so
        // rather than sending a frame nobody can answer on.
        let err = reset_firmware(
            &serial(McuRestartMethod::Admin),
            &tokio::runtime::Handle::current(),
        )
        .await
        .unwrap_err();
        assert!(err.contains("has not been assigned"), "{err}");
    }

    // -----------------------------------------------------------------------
    // The bridge-port reset, on the wire.
    // -----------------------------------------------------------------------

    use crate::core::klippy::config::Config;
    use crate::core::klippy::interface::devices::host::HOST_LIBRARY_TEST_LOCK;
    use crate::core::klippy::interface::devices::serial::{
        admin_unit, set_port_node_id, CMD_RESET_PORT, CMD_RESTART_DEVICE, MUX_PORT_UNITS,
    };
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::mcu::bridge_attach::wait_for_mux_library_unload;
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::reactor::{ManualReactor, TokioReactor};
    use std::io::Read;
    use std::os::fd::AsRawFd;
    use std::sync::Arc;
    use std::time::Instant;

    /// Read from the pty's master end for at most `window`: everything the other
    /// end wrote, handed over as soon as `at_least` bytes are in hand — empty
    /// when the port stays silent the whole window. The `poll` is bounded, so no
    /// step can hang on a port that never writes.
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

    /// Eat everything the port has written until it has been quiet for `quiet`,
    /// or `cap` expires — so an earlier step's bytes are never mistaken for the
    /// reset's.
    fn drain_pty(master: &std::fs::File, quiet: Duration, cap: Duration) {
        let deadline = Instant::now() + cap;
        while Instant::now() < deadline && !read_pty(master, 1, quiet).is_empty() {}
    }

    /// A pty standing in for the bridge's own link, with a hand-built
    /// [`BridgePortLink`] and no firmware on the other end: the host's bytes are
    /// all this asserts.
    #[tokio::test]
    async fn the_reset_commands_are_the_port_level_admin_units() {
        let pty = Pty::new();
        let printer = Printer::new(ManualReactor::shared());
        let link = BridgePortLink {
            transport: Transport::Serial {
                path: pty.path.to_str().expect("a utf-8 pty path").to_string(),
                baud: 250_000,
                uuid: None,
            },
            index: 3,
            flags: 0,
            // The port command path does not read it: this link is hand-built
            // for the reset family, which never names a device.
            named: None,
            // Nor this: the reset family names no device, so no naming runs.
            kind: PortKind::Uart,
        };

        // One temporary downstream handle, one command: the reset unit goes out
        // whole — `e8 3f <len> 20 4b 58 49 44 04 03 <crc> 7e` — and nothing
        // else does: the handle this path opens sends no route.
        let reset = admin_unit(&[CMD_RESET_PORT, link.index]);
        let interface = open_bridge_link(&printer, &link, 1).expect("the bridge link opens");
        interface
            .send_reset_port(link.index)
            .expect("the port reset goes out");
        assert_eq!(
            read_pty(&pty.master, reset.len(), Duration::from_secs(3)),
            reset,
            "the port reset is exactly the admin unit the firmware's mux.h spells"
        );

        // The device restart is the same shape, one command byte apart.
        let reset = admin_unit(&[CMD_RESTART_DEVICE, link.index]);
        let interface = open_bridge_link(&printer, &link, 2).expect("the bridge link opens");
        interface
            .send_restart_device(link.index)
            .expect("the device restart goes out");
        assert_eq!(
            read_pty(&pty.master, reset.len(), Duration::from_secs(3)),
            reset,
            "the device restart is exactly the admin unit the firmware's mux.h spells"
        );
    }

    /// A bridge reached over CAN is refused here with the attach pass's own
    /// words — not through `{:?}` — so the reset path can never hand the user a
    /// `Can { interface: …, uuid: […] }` dump.
    #[test]
    fn a_can_bridge_reset_is_refused_with_the_attach_messages_words() {
        let printer = Printer::new(ManualReactor::shared());
        let link = BridgePortLink {
            transport: Transport::Can {
                interface: "can0".to_string(),
                uuid: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
            },
            index: 0,
            flags: 0,
            named: None,
            kind: PortKind::Uart,
        };

        let err = open_bridge_link(&printer, &link, 7).unwrap_err();

        assert_eq!(
            err.to_string(),
            "a bridge reached over CAN cannot carry a downstream device yet: the CAN mux \
             carrier and the port-command units exist, but the host-side attach is not \
             wired yet"
        );
    }

    /// The whole bridge-port reset, over the **real** bridge library and a pty.
    ///
    /// The legacy child's port declares `uart_restart_method: arduino`, so its
    /// reset is the bridge's own `RESET_PORT`. The call returning `Ok` means
    /// `resolve` found the port index from the bridge's firmware table and the
    /// command was written to the bridge; what the port itself shows is the
    /// firmware's contract: `RESET_PORT` is consumed by the bridge (it pulses
    /// the lines, which a pty does not have) and **nothing** is written
    /// downstream (`src/bridge.c` `bridge_admin_unit`).
    #[tokio::test(flavor = "multi_thread")]
    async fn the_port_reset_reaches_the_real_bridge_and_stays_off_the_port() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge opens its port during init, so the tty has to exist first —
        // the port's own `uart_path:` below, named for its slot when the library
        // comes up.
        let pty = Pty::new();

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             uart_path: {}\n\
             uart_restart_method: arduino\n\
             [mcu child]\nuart: uart0\n",
            library.display(),
            pty.path.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        printer.load_config(&config).expect("the config loads");

        // Bounded: the child's identify never answers, so the bring-up is
        // interrupted once the child has certainly spoken and the bridge is up.
        let _ = tokio::time::timeout(Duration::from_secs(6), printer.bring_up()).await;
        drain_pty(
            &pty.master,
            Duration::from_millis(500),
            Duration::from_secs(3),
        );

        // The child's own parse: a legacy device on a port that declares a
        // line-level reset takes that port's reset, while its own method stays
        // `command` — the reset is not the device's.
        let child = printer
            .lookup_object_as::<McuObject>("mcu child")
            .expect("the child registered")
            .mcu_config()
            .expect("the child's section parses");
        assert_eq!(child.bridge_restart, Some(BridgeRestart::Port));
        assert_eq!(child.restart_method, McuRestartMethod::Command);

        reset_bridge_firmware(&printer, &child, BridgeRestart::Port)
            .expect("the port reset goes out through the real bridge");

        assert!(
            read_pty(&pty.master, 1, Duration::from_millis(500)).is_empty(),
            "RESET_PORT is the bridge's own business: it must write nothing to the port"
        );

        // `teardown` drops the parts the config loaded and the event handlers
        // they registered; without it the `SAVE_CONFIG` handler's strong
        // `Arc<Printer>` keeps the machine (and so the bridge's open library)
        // alive, and the next test on this path meets a live firmware.
        printer.teardown();
        drop(printer);
        wait_for_mux_library_unload(&library);
    }

    /// The device reset restates the route first, on the bridge's own link.
    ///
    /// `SET_PORT_NODEID` is consumed by the bridge and never forwarded, so the
    /// only place these two units exist side by side is the bridge's link — a
    /// pty standing in for it here.
    #[tokio::test]
    async fn the_device_reset_restates_the_route_first() {
        let pty = Pty::new();
        let printer = Printer::new(ManualReactor::shared());
        let link = BridgePortLink {
            transport: Transport::Serial {
                path: pty.path.to_str().expect("a utf-8 pty path").to_string(),
                baud: 250_000,
                uuid: None,
            },
            index: 3,
            flags: MUX_PORT_UNITS,
            named: Some([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]),
            kind: PortKind::Uart,
        };

        let interface = open_bridge_link(&printer, &link, 9).expect("the bridge link opens");
        device_reset(&interface, &link, 9).expect("the device restart goes out");

        let mut expected = set_port_node_id(link.index, 9, MUX_PORT_UNITS);
        expected.extend_from_slice(&admin_unit(&[CMD_RESTART_DEVICE, link.index]));
        assert_eq!(
            read_pty(&pty.master, expected.len(), Duration::from_secs(3)),
            expected,
            "the route is restated, then the device restart"
        );
    }

    /// Read from the pty until `needle` is the tail of what came, within
    /// `window` — for a stream whose earlier units (a handshake the bridge was
    /// offered, say) are expected but not part of the assertion.
    fn read_until(master: &std::fs::File, needle: &[u8], window: Duration) -> Vec<u8> {
        let deadline = Instant::now() + window;
        let mut out = Vec::new();
        while Instant::now() < deadline {
            out.extend_from_slice(&read_pty(master, 1, Duration::from_millis(100)));
            if out.ends_with(needle) {
                break;
            }
        }
        out
    }

    /// The bridge's **own** restart goes out even when its firmware holds no
    /// node id.
    ///
    /// A silent pty stands in for that bridge: it answers the attach handshake
    /// with nothing — only an assigned mux replies to it (`src/mux.c`) — so the
    /// fallback to `attach_downstream` is what runs, and the admin unit naming
    /// the index no port can have is on the wire all the same.
    #[tokio::test]
    async fn a_bridge_with_no_id_is_restarted_over_its_admin_channel() {
        let pty = Pty::new();
        let config = McuConfig {
            name: "bridge".to_string(),
            restart_method: McuRestartMethod::Admin,
            bridge_restart: None,
            transport: Transport::Serial {
                path: pty.path.to_str().expect("a utf-8 pty path").to_string(),
                baud: 250_000,
                uuid: None,
            },
            id: Some(1),
            shared_baud: None,
            host_mux: None,
            host_port_paths: Vec::new(),
            usb_power: usb::UsbPowerMethod::default(),
        };

        reset_firmware(&config, &tokio::runtime::Handle::current())
            .await
            .expect("the bridge restart goes out");

        let restart = admin_unit(&[CMD_RESTART_DEVICE, 0xFF]);
        let written = read_until(&pty.master, &restart, Duration::from_secs(5));
        assert!(
            written.ends_with(&restart),
            "the bridge's own restart is the last unit on the wire: {written:?}"
        );
    }

    /// A **named** child's restart is forwarded to the device, whole.
    ///
    /// The deferred half of the restart family, and the reason the host says the
    /// route again before it: whether the bridge forwards the unit whole or
    /// strips its header depends on the port's flags, which live in the bridge
    /// and may still be the default legacy ones when the host asks for a
    /// restart. The port therefore shows both commands, exactly as `mux.h`
    /// spells them — the route ([`set_port_node_id`], idempotent) and then
    /// [`CMD_RESTART_DEVICE`] — and the bridge's own line pulse is never asked
    /// for.
    ///
    /// **Multi-thread runtime, deliberately** (as production builds:
    /// `klippy.rs`'s `new_multi_thread`, and the legacy sibling below). The
    /// bridge's two transport tasks each hold a clone of its `Interface`
    /// (`Mcu::from_parts`), and so of the downstream `Arc<HostDevice>`; only
    /// the last reference's drop unloads the library. `teardown` aborts those
    /// tasks, but an aborted task is dropped by the runtime, not by `abort` —
    /// on a current-thread runtime the bounded, *blocking* wait here never
    /// lets the runtime run them again, so the library would look permanently
    /// loaded. Nothing about the named path is special: the aborted-task
    /// release is the same one the legacy test relies on.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_named_childs_reset_is_forwarded_to_its_port_whole() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        let pty = Pty::new();

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let text = format!(
            "[mcu bridge]\nhost_library: {}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_baud: 250000\n\
             uart_path: {}\n\
             [mcu child]\nuart: uart0\nuuid: 1234567890ab\nrestart_method: admin\n",
            library.display(),
            pty.path.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        printer.load_config(&config).expect("the config loads");

        // Bounded: nothing answers behind the port, so the child's identify
        // never completes and the bring-up is interrupted once the bridge is
        // certainly up.
        let _ = tokio::time::timeout(Duration::from_secs(6), printer.bring_up()).await;
        drain_pty(
            &pty.master,
            Duration::from_millis(500),
            Duration::from_secs(3),
        );

        let child = printer
            .lookup_object_as::<McuObject>("mcu child")
            .expect("the child registered")
            .mcu_config()
            .expect("the child's section parses");
        assert_eq!(child.bridge_restart, Some(BridgeRestart::Firmware));
        assert_eq!(child.restart_method, McuRestartMethod::Admin);
        let _child_id = child.id.expect("the child has an id") as u8;

        reset_bridge_firmware(&printer, &child, BridgeRestart::Firmware)
            .expect("the device restart goes out through the real bridge");

        let mut expected = admin_unit(&[CMD_RESTART_DEVICE, 0]);
        let _ = &mut expected;
        assert_eq!(
            read_pty(&pty.master, expected.len(), Duration::from_secs(3)),
            expected,
            "the device restart is forwarded to the port whole, not stripped"
        );
        // The route command the host restates first is the bridge's own: it is
        // consumed there and never appears on the port (which is why the flag
        // write can only be observed on the bridge's link — see
        // `the_device_reset_restates_the_route_first`).
        assert!(
            read_pty(&pty.master, 1, Duration::from_millis(300)).is_empty(),
            "only the forwarded restart reaches the port"
        );

        printer.teardown();
        drop(printer);
        wait_for_mux_library_unload(&library);
    }
}
