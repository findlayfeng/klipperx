/// MCU restart method
///
/// **Not implemented yet.** This is a planned feature: the value is parsed from the
/// config file and carried in [`McuConfig`](crate::core::klippy::config::mcu::McuConfig)
/// so that a printer config using `restart_method` keeps loading, but nothing reads
/// it — no firmware restart path exists yet.
///
/// When that path is implemented (Klipper's `mcu.py` restart / `config_reset`),
/// this enum is what it will switch on. Do not remove it as dead code in the
/// meantime.
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
