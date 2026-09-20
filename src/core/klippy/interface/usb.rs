//! Power-cycling the USB port a serial MCU sits on.
//!
//! `restart_method: rpi_usb` resets a board by cutting and restoring the power of
//! the USB port it hangs off — upstream shells out to a compiled `hub-ctrl` with
//! `sudo` for this (`klippy/mcu.py:748`, `chelper/__init__.py:339`). Here the port
//! is found by reading sysfs, and there are two ways to switch it:
//!
//! | mechanism | how | needs |
//! |---|---|---|
//! | [`UsbPower::Sysfs`] | write `1`/`0` to the port's sysfs `disable` file | Linux 6.0+, a writable attribute (a udev `RUN+=` chmod) |
//! | [`UsbPower::Control`] | a class/other `SET_FEATURE`/`CLEAR_FEATURE` for `USB_PORT_FEAT_POWER`, via `nusb` | access to `/dev/bus/usb/...` (a udev rule or root) |
//!
//! [`UsbPowerMethod`] — the `usb_power` config option — picks between them:
//! `auto` (the default) uses sysfs when it is usable and falls back to the
//! control request; `sysfs` and `libusb` force one. Forcing `libusb` is the
//! escape hatch for a hub whose port switch stalls.
//!
//! [`probe`] decides (and checks permission for) the mechanism without touching
//! power, so a missing udev rule is reported at startup rather than at the first
//! firmware restart; [`recommended_rules`] renders the rules to install.
//!
//! The control request is what `hub-ctrl` does (`hub-ctrl.c:396`); the sysfs file
//! is Linux's own port switch (`Documentation/ABI/testing/sysfs-bus-usb`, June
//! 2022). The user manual explains both, and why the **hub** — not the MCU — is
//! what a rule has to grant.

use std::fs;
use std::path::{Path, PathBuf};
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

/// How `rpi_usb` switches a port: the `usb_power` config option.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UsbPowerMethod {
    /// Use sysfs when it is usable, otherwise the hub control request.
    #[default]
    Auto,
    /// Only the sysfs port switch (Linux 6.0+).
    Sysfs,
    /// Only the hub control request — what `hub-ctrl` does.
    Libusb,
}

impl UsbPowerMethod {
    /// The spellings a config file may use, for error messages.
    pub const CHOICES: &'static [&'static str] = &["auto", "sysfs", "libusb"];

    /// The config-file spelling of this method.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Sysfs => "sysfs",
            Self::Libusb => "libusb",
        }
    }

    /// Parse a `usb_power` value from the config file.
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "auto" => Some(Self::Auto),
            "sysfs" => Some(Self::Sysfs),
            // `control` is the mechanism's own name, kept as an alias so the
            // option is readable either way.
            "libusb" | "control" => Some(Self::Libusb),
            _ => None,
        }
    }
}

/// The hub port a USB device sits on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbPort {
    /// The hub's bus number, as sysfs and usbfs report it.
    pub bus: u8,
    /// The hub's device address on that bus.
    pub device: u8,
    /// The 1-based port on that hub.
    pub port: u8,
    /// The hub's sysfs directory (`/sys/devices/.../<hub>`), where the sysfs
    /// port switch lives.
    pub hub_dir: PathBuf,
}

/// Which mechanism a reset will use, as [`probe`] resolved it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UsbPower {
    /// Write this port's sysfs `disable` file.
    Sysfs(PathBuf),
    /// Send the hub a port-power control request.
    Control,
}

/// Resolve the hub port a tty (e.g. `/dev/ttyACM0`) is attached to.
///
/// The path is resolved first: a config may name `/dev/serial/by-id/…`, whose
/// basename is not the tty's, and `/sys/class/tty` is keyed by the tty's own
/// name.
///
/// # Errors
/// Returns a message when the tty cannot be resolved, is not a USB device, or its
/// topology cannot be read from sysfs.
pub fn resolve_tty_port(tty: &Path) -> Result<UsbPort, String> {
    let real =
        fs::canonicalize(tty).map_err(|e| format!("usb: cannot resolve {}: {e}", tty.display()))?;
    resolve_tty_port_in(Path::new("/sys"), &real)
}

/// Decide how `port` will be switched, and check that this process may do it.
///
/// Nothing gets switched: this is the startup probe, so a missing udev rule is
/// reported before the first firmware restart.
///
/// # Errors
/// Returns the reason the chosen method is unusable. With
/// [`UsbPowerMethod::Auto`] the control request is tried when sysfs is unusable,
/// so an error here means neither works.
pub fn probe(port: &UsbPort, method: UsbPowerMethod) -> Result<UsbPower, String> {
    match method {
        UsbPowerMethod::Sysfs => probe_sysfs(port),
        UsbPowerMethod::Libusb => probe_control(port),
        UsbPowerMethod::Auto => probe_sysfs(port).or_else(|_| probe_control(port)),
    }
}

/// Switch the power of `port` on or off, the way [`probe`] resolved it.
///
/// # Errors
/// Returns a message when the sysfs write or the control transfer fails.
pub fn set_port_power(port: &UsbPort, power: &UsbPower, on: bool) -> Result<(), String> {
    match power {
        UsbPower::Sysfs(path) => set_sysfs(path, on),
        UsbPower::Control => set_control(port, on),
    }
}

/// The udev rules that make `port` switchable, as a warning prints them.
///
/// The first covers the control request, naming the hub by the ids it reports in
/// sysfs; the second makes the sysfs attribute writable, which udev can only do
/// with `RUN+=` — `MODE=` applies to `/dev` nodes, not to sysfs.
pub fn recommended_rules(port: &UsbPort) -> String {
    let (vendor, product) = hub_ids(&port.hub_dir);
    format!(
        "SUBSYSTEM==\"usb\", ATTR{{idVendor}}==\"{vendor}\", ATTR{{idProduct}}==\"{product}\", TAG+=\"uaccess\"\n\
         SUBSYSTEM==\"usb\", DRIVER==\"hub|usb\", \\\n\
         \x20   RUN+=\"/bin/sh -c \\\"chmod -f 660 $sys$devpath/*port*/disable || true\\\"\""
    )
}

// ===========================================================================
// The two mechanisms
// ===========================================================================

/// The sysfs route: the port's `disable` file, if it exists and we may write it.
fn probe_sysfs(port: &UsbPort) -> Result<UsbPower, String> {
    let path = disable_path(&port.hub_dir, port.port)
        .ok_or_else(|| "no sysfs port `disable` file (kernel < 6.0?)".to_string())?;
    if !writable(&path) {
        return Err(format!("{} is not writable", path.display()));
    }
    Ok(UsbPower::Sysfs(path))
}

/// The control route: opening the hub is the part that needs permission.
fn probe_control(port: &UsbPort) -> Result<UsbPower, String> {
    open_hub(port).map_err(|e| format!("hub {}:{}: {e}", port.bus, port.device))?;
    Ok(UsbPower::Control)
}

/// Write the sysfs port switch. Its sense is inverted: `1` is off, `0` is on.
fn set_sysfs(path: &Path, on: bool) -> Result<(), String> {
    fs::write(path, if on { "0" } else { "1" })
        .map_err(|e| format!("usb: cannot write {}: {e}", path.display()))?;
    debug!(
        "usb port switch {} set {}",
        path.display(),
        if on { "on" } else { "off" }
    );
    Ok(())
}

/// Send the hub the port-power control request.
fn set_control(port: &UsbPort, on: bool) -> Result<(), String> {
    let handle = open_hub(port)?;
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

// ===========================================================================
// sysfs discovery
// ===========================================================================

/// Find the sysfs `disable` file for `port` on the hub at `hub_dir`.
///
/// The port device's directory name has changed across kernels: the ABI document
/// says `port<X>`, while 7.x uses `<hub>-port<X>`. Both are accepted, under any
/// of the hub's interface directories (`<hub>:<cfg>.<if>`).
fn disable_path(hub_dir: &Path, port: u8) -> Option<PathBuf> {
    let plain = format!("port{port}");
    let suffixed = format!("-{plain}");
    for iface in fs::read_dir(hub_dir).ok()?.flatten() {
        let iface = iface.path();
        if !iface.is_dir() {
            continue;
        }
        for entry in fs::read_dir(&iface).ok()?.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if name != plain && !name.ends_with(&suffixed) {
                continue;
            }
            let disable = entry.path().join("disable");
            if disable.is_file() {
                return Some(disable);
            }
        }
    }
    None
}

/// Whether this process may write `path`. Opening a sysfs attribute for writing
/// changes nothing, so it is a safe probe.
fn writable(path: &Path) -> bool {
    fs::OpenOptions::new().write(true).open(path).is_ok()
}

/// The hub's own USB ids, as sysfs reports them, for the suggested udev rule.
fn hub_ids(hub_dir: &Path) -> (String, String) {
    let read = |file: &str| {
        fs::read_to_string(hub_dir.join(file))
            .map(|value| value.trim().to_string())
            .unwrap_or_else(|_| "????".to_string())
    };
    (read("idVendor"), read("idProduct"))
}

/// Open the hub device by the bus/address sysfs reports.
fn open_hub(port: &UsbPort) -> Result<nusb::Device, String> {
    let info = nusb::list_devices()
        .wait()
        .map_err(|e| format!("cannot list devices: {e}"))?
        .find(|info| info.busnum() == port.bus && info.device_address() == port.device)
        .ok_or_else(|| "not found".to_string())?;
    info.open()
        .wait()
        .map_err(|e| format!("{e}; a udev rule for that hub may be needed"))
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
        hub_dir: hub_dir.to_path_buf(),
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

        /// A USB device directory at `<root>/devices/<rel>`, with `busnum`,
        /// `devnum` and (for hubs) the ids a rule names it by.
        fn node(&self, rel: &str, bus: u8, dev: u8) -> PathBuf {
            let dir = self.0.join("devices").join(rel);
            fs::create_dir_all(&dir).unwrap();
            fs::write(dir.join("busnum"), format!("{bus}\n")).unwrap();
            fs::write(dir.join("devnum"), format!("{dev}\n")).unwrap();
            dir
        }

        /// Give a hub its ids, and a port device with the sysfs `disable` file.
        fn hub(
            &self,
            node: &Path,
            name: &str,
            vendor: &str,
            product: &str,
            port: u8,
            dir_name: &str,
        ) {
            fs::write(node.join("idVendor"), format!("{vendor}\n")).unwrap();
            fs::write(node.join("idProduct"), format!("{product}\n")).unwrap();
            let iface = node.join(format!("{name}:1.0"));
            let port_dir = iface.join(dir_name.replace("{n}", &port.to_string()));
            fs::create_dir_all(&port_dir).unwrap();
            fs::write(port_dir.join("disable"), "0").unwrap();
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
    fn test_the_option_spellings_round_trip() {
        for spelling in UsbPowerMethod::CHOICES {
            let method = UsbPowerMethod::parse(spelling).unwrap();
            assert_eq!(method.as_str(), *spelling);
        }
        // The mechanism's own name is accepted too.
        assert_eq!(
            UsbPowerMethod::parse("control"),
            Some(UsbPowerMethod::Libusb)
        );
        assert_eq!(UsbPowerMethod::parse("bogus"), None);
    }

    #[test]
    fn test_a_tty_resolves_to_the_hub_port_above_it() {
        let sysfs = Sysfs::new("nested");
        let hub = sysfs.node("usb1/1-1", 1, 2); // the hub
        let mcu = sysfs.node("usb1/1-1/1-1.2", 1, 3); // the MCU on port 2 of it
        let iface = mcu.join("1-1.2:1.0"); // its interface, what the tty links to
        fs::create_dir_all(&iface).unwrap();
        sysfs.tty("ttyACM0", iface);

        let port = resolve_tty_port_in(sysfs.root(), Path::new("/dev/ttyACM0")).unwrap();
        assert_eq!(port.bus, 1);
        assert_eq!(port.device, 2);
        assert_eq!(port.port, 2);
        // The hub directory is the real one, which is where the port switch lives.
        assert_eq!(port.hub_dir, fs::canonicalize(&hub).unwrap());
    }

    #[test]
    fn test_a_device_on_the_root_hub_resolves_to_port_one() {
        let sysfs = Sysfs::new("root");
        sysfs.node("usb1", 1, 1); // the root hub itself
        let mcu = sysfs.node("usb1/1-1", 1, 2); // the MCU, directly on it
        sysfs.tty("ttyUSB0", mcu);

        let port = resolve_tty_port_in(sysfs.root(), Path::new("/dev/ttyUSB0")).unwrap();
        assert_eq!((port.bus, port.device, port.port), (1, 1, 1));
    }

    #[test]
    fn test_a_tty_without_a_usb_device_is_reported() {
        let sysfs = Sysfs::new("nosuch");

        let err = resolve_tty_port_in(sysfs.root(), Path::new("/dev/ttyS0")).unwrap_err();
        assert!(err.contains("not a USB tty"), "{err}");
    }

    #[test]
    fn test_the_port_switch_is_found_under_either_name() {
        // The ABI document's `port<N>`, and the `<hub>-port<N>` 7.x uses.
        for (dir_name, name) in [("port{n}", "plain"), ("1-1-port{n}", "prefixed")] {
            let sysfs = Sysfs::new(name);
            let hub = sysfs.node("usb1/1-1", 1, 2);
            sysfs.hub(&hub, "1-1", "1d6b", "0002", 2, dir_name);

            let port = UsbPort {
                bus: 1,
                device: 2,
                port: 2,
                hub_dir: fs::canonicalize(&hub).unwrap(),
            };
            let found = disable_path(&port.hub_dir, 2).unwrap();
            assert!(found.is_file(), "{dir_name}: {}", found.display());

            // And the whole probe resolves it, writing enabled because the test
            // owns the file.
            assert_eq!(
                probe(&port, UsbPowerMethod::Sysfs),
                Ok(UsbPower::Sysfs(found))
            );
        }
    }

    #[test]
    fn test_the_recommended_rule_names_the_hub_by_its_ids() {
        let sysfs = Sysfs::new("rule");
        let hub = sysfs.node("usb1/1-1", 1, 2);
        sysfs.hub(&hub, "1-1", "1d6b", "0002", 2, "1-1-port{n}");

        let port = UsbPort {
            bus: 1,
            device: 2,
            port: 2,
            hub_dir: fs::canonicalize(&hub).unwrap(),
        };
        let rule = recommended_rules(&port);
        assert!(rule.contains("idVendor") && rule.contains("1d6b"), "{rule}");
        assert!(rule.contains("1d6b") && rule.contains("0002"), "{rule}");
        assert!(rule.contains("*port*/disable"), "{rule}");
    }
}
