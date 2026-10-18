/// How the host resets an MCU's firmware: the `restart_method` config option.
///
/// Upstream's `MCURestartHelper` (`klippy/mcu.py:656`) dispatches on this when it
/// acts on a `klippy:firmware_restart`. Four of the methods are upstream's, all
/// with a path (`mcu/restart.rs`: DTR toggles, the Cheetah RTS/DTR sequence,
/// cutting the USB port's power, and `config_reset` for `command`); the three
/// physical ones are **untested on hardware** and say so when they run; the
/// value itself is resolved and validated here, at config time.
///
/// Upstream reads the option only for a serial MCU; every other transport
/// resets with `command` (`klippy/mcu.py:668-671`). klipperx adds
/// [`McuRestartMethod::Admin`], which needs a mux **admin channel** — and two
/// shapes have one: a serial MCU, which *is* the bridge and restarts itself over
/// its own link, and a device behind a bridge port that answers the mux itself
/// (a `uuid:` on the child), whose restart the bridge forwards to its port. A
/// legacy (uuid-less) bridge-port device takes no method of its own — its
/// **port's** `uart_restart_method` decides (`config/mcu.rs`,
/// `mcu/bridge_ports.rs`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum McuRestartMethod {
    /// Toggle DTR (common on Arduino boards and clones)
    Arduino,
    /// Special method for Fysetc Cheetah boards
    Cheetah,
    /// Disable power to all USB ports (Raspberry Pi)
    RpiUsb,
    /// Send a Klipper command to reset itself
    Command,
    /// Have the device reset itself over its mux **admin** channel. Two shapes
    /// reach one: a **serial** MCU is the bridge itself, whose own `mcu_restart`
    /// the admin unit triggers (`mcu/restart.rs`), and a bridge-port child that
    /// writes a `uuid:`, whose restart the bridge forwards to its port
    /// (`RESET_DEVICE`).
    Admin,
}

impl McuRestartMethod {
    /// The spellings a config file may use, for error messages. Upstream's
    /// `getchoice` list without its `None` (`klippy/mcu.py:666`), plus klipperx's
    /// `admin` (`config/mcu.rs`).
    pub const CHOICES: &'static [&'static str] =
        &["arduino", "cheetah", "command", "rpi_usb", "admin"];

    /// The config-file spelling of this method, for logs and errors.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Arduino => "arduino",
            Self::Cheetah => "cheetah",
            Self::RpiUsb => "rpi_usb",
            Self::Command => "command",
            Self::Admin => "admin",
        }
    }

    /// Parse a `restart_method` value from the config file.
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "arduino" => Some(Self::Arduino),
            "cheetah" => Some(Self::Cheetah),
            "rpi_usb" => Some(Self::RpiUsb),
            "command" => Some(Self::Command),
            "admin" => Some(Self::Admin),
            _ => None,
        }
    }
}
