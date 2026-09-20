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
//! | `cheetah` | *(not implemented)* | — |
//! | `rpi_usb` | *(not implemented)* | — |
//!
//! The two unimplemented methods are reported and the connection proceeds
//! without a physical reset. That still recovers a board whose firmware
//! restarted or whose configuration changed, because `configure` clears it with
//! `config_reset` when it has to — the physical reset only matters for a board
//! that is wedged before the host can talk to it.
//!
//! [`McuObject::connect`]: super::object::McuObject::connect

use tokio::time::Duration;
use tracing::{info, warn};

use crate::core::klippy::config::mcu::{McuConfig, Transport};
use crate::core::klippy::interface::serial::ModemLines;
use crate::core::klippy::mcu::McuRestartMethod;

/// How long to leave the board between the steps of a reset. Upstream pauses
/// `0.100` s between each (`klippy/serialhdl.py:392-405`).
const RESET_SETTLE: Duration = Duration::from_millis(100);

/// The line speed Klipper opens at for an Arduino reset. A different rate is
/// part of what the board's USB-serial bridge notices (`serialhdl.py:394`).
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
                // Say so out loud: the DTR toggle cannot be exercised without a real
                // board (a pty does not emulate the modem lines), so this path has
                // never been run against hardware.
                warn!(
                    "MCU '{}' restart_method 'arduino' has not been tested on real hardware; \
                     resetting by DTR toggle anyway",
                    config.name
                );
                arduino_reset(path).await
            }
            // `parse_restart_method` only yields `arduino` for a serial MCU.
            _ => Ok(()),
        },
        McuRestartMethod::Cheetah | McuRestartMethod::RpiUsb => {
            warn!(
                "MCU '{}' restart_method '{}' is not implemented; reconnecting without a \
                 physical reset",
                config.name,
                config.restart_method.as_str()
            );
            Ok(())
        }
    }
}

/// Toggle DTR at another rate — Klipper's `arduino_reset`
/// (`klippy/serialhdl.py:392`).
///
/// The port is opened here and dropped at the end, so it must not be open
/// anywhere else; `connect` calls this before [`McuConfig::open`].
async fn arduino_reset(path: &str) -> Result<(), String> {
    let port = ModemLines::open(path, RESET_BAUD).map_err(|e| format!("serial: {e}"))?;
    // Drain first, the way upstream does: a byte already in flight must not be
    // left to be read as the board's answer to the reset.
    port.drain();
    tokio::time::sleep(RESET_SETTLE).await;
    port.set_dtr(true).map_err(|e| format!("serial: {e}"))?;
    tokio::time::sleep(RESET_SETTLE).await;
    port.set_dtr(false).map_err(|e| format!("serial: {e}"))?;
    tokio::time::sleep(RESET_SETTLE).await;
    info!("serial port {path} reset by DTR toggle");
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::value::ConfigValue;

    /// A config whose transport is never opened by these tests: `command`,
    /// `cheetah` and `rpi_usb` do not touch it here.
    fn no_transport(method: McuRestartMethod) -> McuConfig {
        McuConfig {
            name: "mcu".to_string(),
            restart_method: method,
            transport: Transport::Test(ConfigValue::Multi(Vec::new())),
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
    async fn test_unimplemented_methods_do_not_fail_the_connect() {
        for method in [McuRestartMethod::Cheetah, McuRestartMethod::RpiUsb] {
            // No physical reset, but the connection has to be allowed to go on:
            // `config_reset` still clears a wedged or re-configured firmware
            // once the link is up.
            reset_firmware(&no_transport(method)).await.unwrap();
        }
    }

    #[tokio::test]
    async fn test_arduino_routes_to_the_serial_reset() {
        // The reset opens the port itself, so a path that cannot be opened is
        // where the dispatch shows: the error is the serial one, wrapped.
        let config = McuConfig {
            name: "mcu".to_string(),
            restart_method: McuRestartMethod::Arduino,
            transport: Transport::Serial {
                path: "/dev/not-a-serial-port".to_string(),
                baud: 250_000,
            },
        };

        let err = reset_firmware(&config).await.unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.contains("/dev/not-a-serial-port"), "{err}");
    }
}
