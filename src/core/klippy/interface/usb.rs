//! Power-cycling the USB port a serial MCU sits on.
//!
//! `restart_method: rpi_usb` resets a board by cutting and restoring the power of
//! the USB port it hangs off — upstream shells out to a compiled `hub-ctrl` with
//! `sudo` for this (`klippy/mcu.py:748`, `chelper/__init__.py:339`). Here the port
//! is found by reading sysfs and the power is switched with a USB control
//! transfer, through `nusb` (pure Rust, no `libusb`).
//!
//! The operation itself is one class/other control request — `SET_FEATURE` /
//! `CLEAR_FEATURE` for `USB_PORT_FEAT_POWER` — sent to the hub (`hub-ctrl.c:396`).
//!
//! # Privileges
//!
//! Opening a hub's `/dev/bus/usb/...` node and claiming it needs root, which is
//! why upstream runs `sudo`. Use a **udev rule for that hub** instead, so the host
//! itself stays unprivileged:
//!
//! ```text
//! SUBSYSTEM=="usb", ATTRS{idVendor}=="1d6b", ATTRS{idProduct}=="0003", TAG+="uaccess"
//! ```
//!
//! (match the hub's own ids; `TAG+="uaccess"` grants the logged-in user, or use
//! `MODE="0660", GROUP="..."`). Without such a rule the transfer fails with a
//! permission error, which is reported as-is.
//!
//! # What this cannot do
//!
//! There is no standard sysfs switch for port power — that is exactly why
//! `uhubctl` uses `libusb` — so this is a real power cut, not the
//! deauthorize/unbind "soft reset" sysfs offers.

use std::fs;
use std::path::Path;
use std::time::Duration;

use nusb::transfer::{ControlOut, ControlType, Recipient};
use nusb::MaybeFuture;
use tracing::debug;

/// `USB_PORT_FEAT_POWER`: the hub port feature that switches the port's power
/// (`hub-ctrl.c:18`).
const USB_PORT_FEAT_POWER: u16 = 8;
/// `SET_FEATURE` / `CLEAR_FEATURE` (`linux/usb/ch9.h`).
const SET_FEATURE: u8 = 3;
const CLEAR_FEATURE: u8 = 1;
/// How long the port-power control transfer may take.
const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);

/// The hub port a USB device sits on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsbPort {
    /// The hub's bus number, as sysfs and usbfs report it.
    pub bus: u8,
    /// The hub's device address on that bus.
    pub device: u8,
    /// The 1-based port on that hub.
    pub port: u8,
}

/// Resolve the hub port a tty (e.g. `/dev/ttyACM0`) is attached to.
///
/// # Errors
/// Returns a message when the tty is not a USB device, or its topology cannot be
/// read from sysfs.
pub fn resolve_tty_port(tty: &Path) -> Result<UsbPort, String> {
    resolve_tty_port_in(Path::new("/sys"), tty)
}

/// Switch the power of `port` on or off.
///
/// # Errors
/// Returns a message when the hub cannot be found, opened (see the module
/// documentation on privileges), or the transfer fails.
pub fn set_port_power(port: UsbPort, on: bool) -> Result<(), String> {
    let hub = nusb::list_devices()
        .wait()
        .map_err(|e| format!("usb: cannot list devices: {e}"))?
        .find(|info| info.busnum() == port.bus && info.device_address() == port.device)
        .ok_or_else(|| format!("usb: hub {}:{} not found", port.bus, port.device))?;
    let handle = hub.open().wait().map_err(|e| {
        format!(
            "usb: cannot open hub {}:{} ({e}); a udev rule for that hub may be needed",
            port.bus, port.device
        )
    })?;

    handle
        .control_out(
            ControlOut {
                control_type: ControlType::Class,
                recipient: Recipient::Other,
                request: if on { SET_FEATURE } else { CLEAR_FEATURE },
                value: USB_PORT_FEAT_POWER,
                index: u16::from(port.port),
                data: &[],
            },
            CONTROL_TIMEOUT,
        )
        .wait()
        .map_err(|e| format!("usb: cannot switch port {} power: {e}", port.port))?;

    debug!(
        "usb port {} of hub {}:{} powered {}",
        port.port,
        port.bus,
        port.device,
        if on { "on" } else { "off" }
    );
    Ok(())
}

/// The sysfs-reading half of [`resolve_tty_port`], with a replaceable root.
///
/// Split out so a test can point it at a fake tree: everything below the root is
/// read the same way.
fn resolve_tty_port_in(sysfs: &Path, tty: &Path) -> Result<UsbPort, String> {
    let name = tty
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| format!("usb: not a tty path: {}", tty.display()))?;

    // `/sys/class/tty/<name>/device` links to the tty's USB interface, or to the
    // device itself.
    let link = sysfs.join("class/tty").join(name).join("device");
    let node = fs::canonicalize(&link)
        .map_err(|e| format!("usb: {} is not a USB tty ({e})", tty.display()))?;

    // The nearest ancestor carrying `busnum` is the USB device; its parent is the
    // hub whose port powers it.
    let device_dir = node
        .ancestors()
        .find(|dir| dir.join("busnum").is_file())
        .ok_or_else(|| format!("usb: no USB device above {}", node.display()))?;
    let hub_dir = device_dir
        .parent()
        .ok_or_else(|| "usb: USB device has no parent hub".to_string())?;

    let device_name = file_name(device_dir)?;
    let hub_name = file_name(hub_dir)?;
    let port = port_number(device_name, hub_name)
        .ok_or_else(|| format!("usb: cannot tell the port of '{device_name}' on '{hub_name}'"))?;

    Ok(UsbPort {
        bus: read_u8(hub_dir, "busnum")?,
        device: read_u8(hub_dir, "devnum")?,
        port,
    })
}

/// A sysfs directory's own name.
fn file_name(dir: &Path) -> Result<&str, String> {
    dir.file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| format!("usb: cannot read the name of {}", dir.display()))
}

/// A one-byte sysfs value (`busnum`, `devnum`).
fn read_u8(dir: &Path, file: &str) -> Result<u8, String> {
    let path = dir.join(file);
    fs::read_to_string(&path)
        .map_err(|e| format!("usb: cannot read {}: {e}", path.display()))?
        .trim()
        .parse()
        .map_err(|e| format!("usb: {} is not a number: {e}", path.display()))
}

/// The port number a device's sysfs name encodes, given its hub's name.
///
/// A device on a hub is `<hub>.<port>` (`1-1.2` under `1-1`); a device on a root
/// hub is `<bus>-<port>` (`1-1` under `usb1`).
fn port_number(device: &str, hub: &str) -> Option<u8> {
    let suffix = match device.strip_prefix(hub) {
        Some(rest) => rest.strip_prefix('.')?,
        None => {
            // The root hub is named `usb<bus>`; its children are `<bus>-<port>`.
            hub.strip_prefix("usb")?;
            device.split_once('-')?.1
        }
    };
    suffix.parse().ok()
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::symlink;
    use std::path::PathBuf;

    /// A throwaway sysfs stand-in.
    struct Sysfs(PathBuf);

    impl Sysfs {
        fn new(name: &str) -> Self {
            let dir =
                std::env::temp_dir().join(format!("klipperx-sysfs-{}-{name}", std::process::id()));
            let _ = fs::remove_dir_all(&dir);
            fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }

        fn root(&self) -> &Path {
            &self.0
        }

        /// A USB device directory at `<root>/devices/<rel>`, with its
        /// `busnum`/`devnum`.
        fn node(&self, rel: &str, bus: u8, dev: u8) -> PathBuf {
            let dir = self.0.join("devices").join(rel);
            fs::create_dir_all(&dir).unwrap();
            fs::write(dir.join("busnum"), format!("{bus}\n")).unwrap();
            fs::write(dir.join("devnum"), format!("{dev}\n")).unwrap();
            dir
        }

        /// Point `/sys/class/tty/<tty>/device` at a node, as the kernel does.
        fn tty(&self, tty: &str, node: PathBuf) {
            let dir = self.0.join("class/tty").join(tty);
            fs::create_dir_all(&dir).unwrap();
            symlink(node, dir.join("device")).unwrap();
        }
    }

    impl Drop for Sysfs {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn test_port_number_from_the_sysfs_names() {
        // On a hub: `<hub>.<port>`.
        assert_eq!(port_number("1-1.2", "1-1"), Some(2));
        assert_eq!(port_number("1-1.4", "1-1"), Some(4));
        // On a root hub: `<bus>-<port>`.
        assert_eq!(port_number("1-1", "usb1"), Some(1));
        assert_eq!(port_number("2-3", "usb2"), Some(3));
        // Anything else is not a device name.
        assert_eq!(port_number("weird", "1-1"), None);
        assert_eq!(port_number("1-1:x", "1-1"), None);
    }

    #[test]
    fn test_a_tty_resolves_to_the_hub_port_above_it() {
        let sysfs = Sysfs::new("nested");
        sysfs.node("usb1/1-1", 1, 2); // the hub
        let mcu = sysfs.node("usb1/1-1/1-1.2", 1, 3); // the MCU on port 2 of it
        let iface = mcu.join("1-1.2:1.0"); // its interface, what the tty links to
        fs::create_dir_all(&iface).unwrap();
        sysfs.tty("ttyACM0", iface);

        assert_eq!(
            resolve_tty_port_in(sysfs.root(), Path::new("/dev/ttyACM0")),
            Ok(UsbPort {
                bus: 1,
                device: 2,
                port: 2
            })
        );
    }

    #[test]
    fn test_a_device_on_the_root_hub_resolves_to_port_one() {
        let sysfs = Sysfs::new("root");
        sysfs.node("usb1", 1, 1); // the root hub itself
        let mcu = sysfs.node("usb1/1-1", 1, 2); // the MCU, directly on it
        sysfs.tty("ttyUSB0", mcu);

        assert_eq!(
            resolve_tty_port_in(sysfs.root(), Path::new("/dev/ttyUSB0")),
            Ok(UsbPort {
                bus: 1,
                device: 1,
                port: 1
            })
        );
    }

    #[test]
    fn test_a_tty_without_a_usb_device_is_reported() {
        let sysfs = Sysfs::new("nosuch");

        let err = resolve_tty_port_in(sysfs.root(), Path::new("/dev/ttyS0")).unwrap_err();
        assert!(err.contains("not a USB tty"), "{err}");
    }
}
