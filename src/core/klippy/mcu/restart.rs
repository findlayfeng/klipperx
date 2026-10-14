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
//! | `admin` | nothing on this device's transport: the port-level `RESET_DEVICE` the bridge forwards ([`reset_bridge_firmware`]) | — |
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

use std::fs;
use std::path::{Path, PathBuf};

use tokio::time::Duration;
use tracing::{debug, info, warn};

use crate::core::klippy::config::mcu::{BridgeRestart, McuConfig, Transport};
use crate::core::klippy::error::{ConfigError, KlippyError};
use crate::core::klippy::interface::devices::host::{HostDevice, HostMux};
use crate::core::klippy::interface::devices::serial::{self, ModemLines};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::usb;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::bridge_attach::{self, BridgePortLink};
use crate::core::klippy::mcu::host_libraries::{HostLibraries, HOST_LIBRARIES_OBJECT};
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
        // The device resets itself, and the bridge forwards the command: that
        // travels the bridge's link, before this device is reached
        // (`McuObject::connect`, `reset_bridge_firmware`), so there is nothing to
        // do on a transport of its own.
        McuRestartMethod::Admin => Ok(()),
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
/// [`McuConfig::bridge_restart`]'s answer — `Device` lets the device reset
/// itself (`RESET_DEVICE`, which the bridge forwards to the port), `Port` asks
/// the bridge to pulse the port's lines (`RESET_PORT`). Both need the port's
/// `index`, which only the bridge's firmware port table has: [`bridge_attach::resolve`]
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
        BridgeRestart::Device => interface.send_reset_device(link.index),
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
            BridgeRestart::Device => "RESET_DEVICE",
            BridgeRestart::Port => "RESET_PORT",
        }
    );
    Ok(())
}

/// Open a temporary downstream handle on `link`'s bridge link, without a route.
///
/// This is the bridge's own transport reopened for one device — a second slot on
/// the shared serial port, or a second reference on the live host library — and
/// it is only here to reach the bridge's **admin** channel
/// ([`serial::attach_downstream`], [`HostDevice::load_downstream`]). The route
/// the attach pass sends (`SET_PORT_NODEID`) is deliberately left out: the
/// device is about to reboot, and the reference that will carry it is opened,
/// and routed, when it connects. Dropping this handle leaves both link kinds in
/// place.
///
/// # Errors
/// [`KlippyError::Config`] for a bridge link that cannot carry a downstream
/// device (a CAN upstream, or a path with no mux identity);
/// [`KlippyError::Connection`] for a failed open.
fn open_bridge_link(
    printer: &Printer,
    link: &BridgePortLink,
    mcu_id: u32,
) -> Result<Interface, KlippyError> {
    match &link.transport {
        Transport::Serial { path, baud, .. } => {
            let handle = serial::attach_downstream(path, *baud, mcu_id).map_err(|err| {
                KlippyError::Connection(format!("serial {path}: downstream attach failed: {err}"))
            })?;
            Ok(Interface::shared_serial(handle))
        }
        Transport::Host { library, .. } => {
            // The reference has to carry the path's own mux identity: the
            // library is already live on it under that identity, and a second
            // one would be refused (or would address another board).
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
            Ok(Interface::from_host(device))
        }
        other => Err(config_error(format!(
            "a bridge on {other:?} cannot carry a downstream device"
        ))),
    }
}

/// Wrap `message` as the config error the bridge-resolution paths report.
fn config_error(message: String) -> KlippyError {
    KlippyError::Config(ConfigError::new(message))
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
/// # Returns
/// The reason this port's power **cannot be switched at all**, when the hub says
/// so itself ([`usb::PortPower::Unsupported`]): switching it would only disconnect
/// the board, which no mechanism can turn into a firmware reset, so the caller
/// drops `rpi_usb` for `command` instead of trying (`mcu/object.rs`). `None` when
/// the mechanism is usable, the method is not `rpi_usb`, or nothing could be found
/// out about the hub.
pub fn check_usb_power(config: &McuConfig) -> Option<String> {
    if config.restart_method != McuRestartMethod::RpiUsb {
        return None;
    }
    let Transport::Serial { path, .. } = &config.transport else {
        return None;
    };

    let error = match usb::resolve_tty_port(Path::new(path)) {
        Err(err) => err,
        Ok(port) => {
            // Ask the hub before anything else: a hub without power switching
            // makes both mechanisms pointless, so neither the rule below nor the
            // switch is worth reporting or attempting.
            match usb::port_power(&port) {
                Some(usb::PortPower::Unsupported) => {
                    return Some(format!(
                        "hub {}:{} reports no port power switching (wHubCharacteristics)",
                        port.bus, port.device
                    ));
                }
                Some(usb::PortPower::Ganged) => warn!(
                    "MCU '{}': hub {}:{} switches the power of all its ports together \
                     (ganged), so anything else on that hub goes down with this port",
                    config.name, port.bus, port.device
                ),
                _ => (),
            }
            match usb::probe(&port, config.usb_power) {
                Ok(power) => {
                    debug!(
                        "MCU '{}' will switch USB port power via {power:?}",
                        config.name
                    );
                    return None;
                }
                Err(err) => format!(
                    "{err}\n  install a udev rule, for example:\n    {}",
                    usb::recommended_rules(&port).replace('\n', "\n    ")
                ),
            }
        }
    };
    warn!(
        "MCU '{}' may not be able to switch the USB port power (restart_method 'rpi_usb'): {error}",
        config.name
    );
    None
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
async fn rpi_usb_reset(
    serial_path: &str,
    method: usb::UsbPowerMethod,
    handle: &tokio::runtime::Handle,
) -> Result<(), String> {
    let tty = PathBuf::from(serial_path);
    let port = usb::resolve_tty_port(&tty)?;
    // Resolve the mechanism per reset, so a hub that was replugged (or a udev
    // rule that only now applies) is picked up again.
    let power = usb::probe(&port, method)?;
    // The device's own node number, to tell the old instance from the one that
    // comes back.
    let before = device_node(&tty);

    usb_set_power(&port, &power, false, handle).await?;
    tokio::time::sleep(USB_POWER_OFF).await;
    usb_set_power(&port, &power, true, handle).await?;

    // The caller is about to open the port, so wait for the board to be back
    // rather than opening the instance that is still on its way out.
    wait_for_new_device(&tty, before, USB_PORT_RETURN_TIMEOUT).await
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

    #[test]
    fn test_the_startup_check_only_answers_for_rpi_usb_on_a_serial_port() {
        // Everything else has no port switch to judge, so nothing is reported and
        // nothing is taken away from the caller.
        assert_eq!(
            check_usb_power(&no_transport(McuRestartMethod::Command)),
            None
        );
        assert_eq!(
            check_usb_power(&no_transport(McuRestartMethod::RpiUsb)),
            None
        );
        // A path that is not a USB tty cannot say anything about a hub either:
        // the lookup reports itself and the caller keeps `rpi_usb`.
        assert_eq!(check_usb_power(&serial(McuRestartMethod::RpiUsb)), None);
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

    // -----------------------------------------------------------------------
    // The bridge-port reset, on the wire.
    // -----------------------------------------------------------------------

    use crate::core::klippy::config::Config;
    use crate::core::klippy::interface::devices::host::HOST_LIBRARY_TEST_LOCK;
    use crate::core::klippy::interface::devices::serial::{
        admin_unit, CMD_RESET_DEVICE, CMD_RESET_PORT,
    };
    use crate::core::klippy::interface::pty::Pty;
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

    /// Block (bounded) until the process-wide library on `path` is unloaded, so
    /// the next test on it meets a fresh firmware.
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
            legacy: true,
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

        // The device reset is the same shape, one command byte apart.
        let reset = admin_unit(&[CMD_RESET_DEVICE, link.index]);
        let interface = open_bridge_link(&printer, &link, 2).expect("the bridge link opens");
        interface
            .send_reset_device(link.index)
            .expect("the device reset goes out");
        assert_eq!(
            read_pty(&pty.master, reset.len(), Duration::from_secs(3)),
            reset,
            "the device reset is exactly the admin unit the firmware's mux.h spells"
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
             uart_restart_method: arduino\n\
             [mcu child]\nuart: uart0\n",
            library.display()
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
        let _ = std::fs::remove_file(&bridge_path);
    }
}
