//! Feature traits — the layer that actually speaks the MCU protocol.
//!
//! A *feature* wraps one capability of the firmware (a clock query, a digital
//! output, an endstop, …) behind a trait, and implements it by issuing typed
//! commands through [`Mcu`](crate::core::klippy::mcu::Mcu).
//!
//! # Why a separate layer
//!
//! Three responsibilities are kept apart on purpose:
//!
//! | Layer | Owns | Knows about |
//! |---|---|---|
//! | `msg` | format strings ↔ bytes | nothing but the codec |
//! | `mcu` | frames, the identify handshake, the data dictionary | only the identify pair |
//! | `feature` | *which* messages exist and what they mean | the firmware protocol |
//!
//! Because the host learns every format from the firmware, a feature never
//! hard-codes a format string or a wire id: it names the message and its
//! parameters, and the dictionary supplies the rest.
//!
//! # Implementing a feature
//!
//! Traits here return `impl Future<…> + Send` rather than using `async fn`, so
//! the returned future is usable from spawned tasks and the `Send` bound is part
//! of the signature instead of an implicit, lint-flagged assumption.
//!
//! Features are constructed from an `Arc<Mcu>` — the shared handle returned by
//! [`identify::connect`] — so several features can use one MCU, and dropping the
//! last handle shuts the device down.
//!
//! [`identify`] is the bootstrap feature: it is the one exchange that runs before
//! a data dictionary exists, and it is what produces the handle the others need.

pub mod clock;
pub mod identify;

pub use clock::{ClockState, ClockSync, GetClock, McuClock};
pub use identify::{Identify, McuIdentify};
