/// How the host resets an MCU's firmware: the `restart_method` config option.
///
/// Upstream's `MCURestartHelper` (`klippy/mcu.py:656`) dispatches on this when it
/// acts on a `klippy:firmware_restart`. The physical resets (DTR toggles, USB
/// power) are not implemented yet — only `command` has a path today, through
/// `config_reset` in `mcu/config.rs` — but the value is resolved and validated
/// now, so a config that sets it behaves the same once the rest lands.
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
