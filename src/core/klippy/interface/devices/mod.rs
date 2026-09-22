//! The transport implementations: one [`Device`](super::Device) per way an MCU
//! can be reached.
//!
//! | module | transport | `[mcu]` key |
//! |---|---|---|
//! | [`serial`] | a tty | `serial:` |
//! | [`canserial`] | Klipper's can-serial link | `canbus_uuid:` |
//! | [`host`] | Klipper's host library | `host_library:` |
//! | [`test`] | a scripted device (test builds only) | `test:` |
//! | `simulator` | a dictionary-driven fake MCU (test builds only) | `test: dict=` |
//!
//! The module they share — the [`Device`](super::Device) trait, the
//! [`Interface`](super::Interface) enum that dispatches to them, and the frame
//! formatting they log with — stays one level up, in [`super`].

pub mod canserial;
pub mod host;
pub mod serial;
#[cfg(test)]
pub mod simulator;
#[cfg(test)]
pub mod test;
