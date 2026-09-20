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
//! | `rpi_usb` | *(not implemented)* | — |
//!
//! The reset paths cannot be exercised in a test — a pty does not emulate the
//! modem lines — so each one reports that it is untested when it runs. That
//! warning comes out once someone has confirmed the sequence on a board.
//!
//! `rpi_usb` is reported and the connection proceeds without a physical reset.
//! That still recovers a board whose firmware restarted or whose configuration
//! changed, because `configure` clears it with `config_reset` when it has to —
//! a physical reset only matters for a board that is wedged before the host can
//! talk to it.
//!
//! [`McuObject::connect`]: super::object::McuObject::connect

use tokio::time::Duration;
use tracing::{info, warn};

use crate::core::klippy::config::mcu::{McuConfig, Transport};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::serial::ModemLines;
use crate::core::klippy::mcu::McuRestartMethod;

/// How long to leave the board between the steps of a reset. Upstream pauses
/// `0.100` s between each (`klippy/serialhdl.py:365-405`).
const RESET_SETTLE: Duration = Duration::from_millis(100);

/// The line speed Klipper opens at for a reset. A different rate is part of what
/// the board's USB-serial bridge notices (`serialhdl.py:369` `:394`).
const RESET_BAUD: u32 = 2400;

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
        McuRestartMethod::RpiUsb => {
            warn!(
                "MCU '{}' restart_method 'rpi_usb' is not implemented; reconnecting without a \
                 physical reset",
                config.name
            );
            Ok(())
        }
    }
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
    async fn test_rpi_usb_is_not_implemented_but_does_not_fail() {
        // No physical reset, but the connection has to be allowed to go on:
        // `config_reset` still clears a wedged or re-configured firmware once
        // the link is up.
        reset_firmware(&no_transport(McuRestartMethod::RpiUsb))
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn test_a_serial_reset_method_needs_a_serial_transport() {
        // Without a serial port there is nothing to toggle, so the dispatch is a
        // no-op rather than an error.
        for method in [McuRestartMethod::Arduino, McuRestartMethod::Cheetah] {
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
    async fn test_cheetah_routes_to_the_serial_reset() {
        let err = reset_firmware(&serial(McuRestartMethod::Cheetah))
            .await
            .unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");
    }
}
