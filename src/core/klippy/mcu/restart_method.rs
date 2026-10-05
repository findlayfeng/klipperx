/// How the host resets an MCU's firmware: the `restart_method` config option.
///
/// Upstream's `MCURestartHelper` (`klippy/mcu.py:656`) dispatches on this when it
/// acts on a `klippy:firmware_restart`. All four methods have a path
/// (`mcu/restart.rs`: DTR toggles, the Cheetah RTS/DTR sequence, cutting the USB
/// port's power, and `config_reset` for `command`), but the three physical ones
/// are **untested on hardware** and say so when they run; the value itself is
/// resolved and validated here, at config time.
///
/// The option only applies to a serial MCU; the other transports reset with
/// `command` (`klippy/mcu.py:668-671`).
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
}

impl McuRestartMethod {
    /// The spellings a config file may use, for error messages. Upstream's
    /// `getchoice` list without its `None` (`klippy/mcu.py:666`).
    pub const CHOICES: &'static [&'static str] = &["arduino", "cheetah", "command", "rpi_usb"];

    /// The config-file spelling of this method, for logs and errors.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Arduino => "arduino",
            Self::Cheetah => "cheetah",
            Self::RpiUsb => "rpi_usb",
            Self::Command => "command",
        }
    }

    /// Parse a `restart_method` value from the config file.
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "arduino" => Some(Self::Arduino),
            "cheetah" => Some(Self::Cheetah),
            "rpi_usb" => Some(Self::RpiUsb),
            "command" => Some(Self::Command),
            _ => None,
        }
    }
}
