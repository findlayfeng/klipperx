//! The transport implementations: one [`Device`](super::Device) per way an MCU
//! can be reached.
//!
//! | module | transport | `[mcu]` key |
//! |---|---|---|
//! | [`serial`] | a tty | `serial:` |
//! | [`canserial`] | Klipper's can-serial link | `canbus_uuid:` |
//! | [`host`] | Klipper's host library | `host_library:` |
//! | `frame_mock` | a frame-level mock (test builds only) | — |
//! | `simulator` | a dictionary-driven fake MCU (test builds only) | `test: dict=` |
//! | `responder_mcu` | the test-side handle for **several** fake MCUs in one printer (test builds only) | — |
//!
//! The module they share — the [`Device`](super::Device) trait, the
//! [`Interface`](super::Interface) enum that dispatches to them, and the frame
//! formatting they log with — stays one level up, in [`super`].

pub mod canserial;
#[cfg(test)]
pub mod frame_mock;
pub mod host;
#[cfg(test)]
pub mod responder_mcu;
pub mod serial;
#[cfg(test)]
pub mod simulator;
