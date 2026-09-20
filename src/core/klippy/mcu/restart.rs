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
//! | `command` | nothing | `config_reset` after the connection is up (`mcu/config.rs`) |
//! | `arduino` | toggle DTR at 2400 baud — **untested on hardware**, and reported | — |
//! | `cheetah` | RTS/DTR sequence at 2400 baud — **untested on hardware**, and reported; the connection itself also needs RTS deasserted (`McuConfig::open`) | — |
//! | `rpi_usb` | cut and restore the USB port's power — **untested on hardware**, and reported | `interface/usb.rs` |
//!
//! The reset paths cannot be exercised in a test — a pty does not emulate the
//! modem lines, and there is no USB port to cut — so each one reports that it is
//! untested when it runs. That warning comes out once someone has confirmed the
//! sequence on a board.
//!
//! [`McuObject::connect`]: super::object::McuObject::connect

use std::path::{Path, PathBuf};

use tokio::time::Duration;
use tracing::{info, warn};

use crate::core::klippy::config::mcu::{McuConfig, Transport};
use crate::core::klippy::interface::devices::serial::ModemLines;
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::usb;
use crate::core::klippy::mcu::McuRestartMethod;

/// How long to leave the board between the steps of a reset. Upstream pauses
/// `0.100` s between each (`klippy/serialhdl.py:365-405`).
const RESET_SETTLE: Duration = Duration::from_millis(100);

/// The line speed Klipper opens at for a reset. A different rate is part of what
/// the board's USB-serial bridge notices (`serialhdl.py:369` `:394`).
const RESET_BAUD: u32 = 2400;

/// How long the USB port stays unpowered. Upstream pauses two seconds
/// (`klippy/mcu.py:752`).
const USB_POWER_OFF: Duration = Duration::from_secs(2);

/// How long to wait for the tty to reappear after the port is powered back on.
const USB_PORT_RETURN_TIMEOUT: Duration = Duration::from_secs(10);

/// Reset the firmware on the transport that is about to be opened.
///
/// Called by `McuObject::connect` when the bring-up follows a
/// `firmware_restart`, before [`McuConfig::open`].
///
/// # Errors
/// Returns a message prefixed with the transport when the reset itself fails, so
/// it reads the same way the open errors do.
pub async fn reset_firmware(config: &McuConfig) -> Result<(), String> {
    match config.restart_method {
        // Nothing before the connection: `config_reset` clears the firmware once
        // it is up (`mcu/config.rs`).
        McuRestartMethod::Command => Ok(()),
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
        McuRestartMethod::RpiUsb => match &config.transport {
            Transport::Serial { path, .. } => {
                warn_untested(config);
                rpi_usb_reset(path, config.usb_power).await
            }
            _ => Ok(()),
        },
    }
}

/// Check, at connect, that `rpi_usb` will be able to switch this port's power.
///
/// Runs on every connect rather than only when a restart is requested, so a
/// missing udev rule is reported at startup — with the rules to install —
/// instead of at the first `FIRMWARE_RESTART`.
pub fn check_usb_power(config: &McuConfig) {
    if config.restart_method != McuRestartMethod::RpiUsb {
        return;
    }
    let Transport::Serial { path, .. } = &config.transport else {
        return;
    };

    let error = match usb::resolve_tty_port(Path::new(path)) {
        Err(err) => err,
        Ok(port) => match usb::probe(&port, config.usb_power) {
            Ok(power) => {
                tracing::debug!(
                    "MCU '{}' will switch USB port power via {power:?}",
                    config.name
                );
                return;
            }
            Err(err) => format!(
                "{err}\n  install a udev rule, for example:\n    {}",
                usb::recommended_rules(&port).replace('\n', "\n    ")
            ),
        },
    };
    warn!(
        "MCU '{}' may not be able to switch the USB port power (restart_method 'rpi_usb'): {error}",
        config.name
    );
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
async fn rpi_usb_reset(serial_path: &str, method: usb::UsbPowerMethod) -> Result<(), String> {
    let tty = PathBuf::from(serial_path);
    let port = usb::resolve_tty_port(&tty)?;
    // Resolve the mechanism per reset, so a hub that was replugged (or a udev
    // rule that only now applies) is picked up again.
    let power = usb::probe(&port, method)?;

    usb_set_power(&port, &power, false).await?;
    tokio::time::sleep(USB_POWER_OFF).await;
    usb_set_power(&port, &power, true).await?;

    // The kernel needs a moment to enumerate the board again, and the caller is
    // about to open the port.
    wait_for_tty(&tty, USB_PORT_RETURN_TIMEOUT).await
}

/// Switch one hub port, off the runtime: `nusb`'s blocking path does the syscalls.
async fn usb_set_power(port: &usb::UsbPort, power: &usb::UsbPower, on: bool) -> Result<(), String> {
    let port = port.clone();
    let power = power.clone();
    tokio::task::spawn_blocking(move || usb::set_port_power(&port, &power, on))
        .await
        .map_err(|e| format!("usb: {e}"))?
}

/// Wait for a tty to (re)appear after its port was powered back on.
///
/// This is what upstream's `check_restart_on_attach` achieves by restarting once
/// more while the port is missing (`klippy/mcu.py:693`); waiting here keeps it in
/// one place.
async fn wait_for_tty(tty: &Path, timeout: Duration) -> Result<(), String> {
    let deadline = tokio::time::Instant::now() + timeout;
    while !tty.exists() {
        if tokio::time::Instant::now() >= deadline {
            return Err(format!(
                "usb: {} did not come back within {timeout:?}",
                tty.display()
            ));
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::value::ConfigValue;

    /// A config whose transport is never opened by these tests.
    fn no_transport(method: McuRestartMethod) -> McuConfig {
        McuConfig {
            name: "mcu".to_string(),
            restart_method: method,
            transport: Transport::Test(ConfigValue::Multi(Vec::new())),
            usb_power: usb::UsbPowerMethod::default(),
        }
    }

    /// A config that names a serial port, so the reset dispatch runs but the
    /// open fails — which is where the routing shows.
    fn serial(method: McuRestartMethod) -> McuConfig {
        McuConfig {
            name: "mcu".to_string(),
            restart_method: method,
            transport: Transport::Serial {
                path: "/dev/not-a-serial-port".to_string(),
                baud: 250_000,
            },
            usb_power: usb::UsbPowerMethod::default(),
        }
    }

    #[tokio::test]
    async fn test_command_leaves_the_transport_alone() {
        // `command` is reset after the connection is up, so nothing happens here.
        reset_firmware(&no_transport(McuRestartMethod::Command))
            .await
            .unwrap();
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
            reset_firmware(&no_transport(method)).await.unwrap();
        }
    }

    #[tokio::test]
    async fn test_arduino_routes_to_the_serial_reset() {
        // The reset opens the port itself, so a path that cannot be opened is
        // where the dispatch shows: the error is the serial one, wrapped.
        let err = reset_firmware(&serial(McuRestartMethod::Arduino))
            .await
            .unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");
    }

    #[tokio::test]
    async fn test_rpi_usb_needs_a_usb_tty() {
        // The port has to be a USB one for its power to be switched; the error
        // from the sysfs lookup is what comes back.
        let err = reset_firmware(&serial(McuRestartMethod::RpiUsb))
            .await
            .unwrap_err();
        assert!(err.starts_with("usb: "), "{err}");
    }

    #[tokio::test]
    async fn test_cheetah_routes_to_the_serial_reset() {
        let err = reset_firmware(&serial(McuRestartMethod::Cheetah))
            .await
            .unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");
    }
}
