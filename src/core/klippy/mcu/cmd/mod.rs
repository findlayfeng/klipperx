//! Command modules — the layer that actually speaks the MCU protocol.
//!
//! Each module in here owns one group of firmware commands and exposes them as a
//! capability: `clock` wraps `get_clock` / `clock`, and the next one will wrap
//! `set_digital_out`, and so on. The trait in a module is what callers depend on;
//! the `Mcu*` type beside it is the implementation that issues typed commands
//! through [`Mcu`](super::Mcu).
//!
//! # Why a separate layer
//!
//! Three responsibilities are kept apart on purpose. Command modules live
//! *inside* `mcu` as its outermost sublayer, but the split is the same one that
//! separates `mcu` from `msg`:
//!
//! | Layer | Owns | Knows about |
//! |---|---|---|
//! | `msg` | format strings ↔ bytes | nothing but the codec |
//! | `mcu` | frames, the data dictionary, typed message access | only the identify pair |
//! | `mcu::cmd` | *which* messages exist and what they mean | the firmware protocol |
//!
//! The dependency still points one way: `cmd` uses `Mcu`, and the transport and
//! dictionary code never mention a command module. `mcu` only declares the
//! module.
//!
//! Because the host learns every format from the firmware, a command module
//! never hard-codes a format string or a wire id: it names the message and its
//! parameters, and the dictionary supplies the rest.
//!
//! Identify is deliberately absent: it is the bootstrap exchange that *produces*
//! the dictionary, its formats are host-owned, and it runs before
//! [`Mcu::call_msg`](super::Mcu::call_msg) is usable. It therefore lives with the
//! transport, in `mcu::identify`, driven by [`Mcu::connect`](super::Mcu::connect).
//!
//! # Adding a command module
//!
//! Traits here return `impl Future<…> + Send` rather than using `async fn`, so
//! the returned future is usable from spawned tasks and the `Send` bound is part
//! of the signature instead of an implicit, lint-flagged assumption.
//!
//! Modules are constructed from an `Arc<Mcu>` — the shared handle returned by
//! [`Mcu::connect`](super::Mcu::connect) — so several modules can use one MCU, and
//! dropping the last handle shuts the device down.

pub mod clock;

pub use clock::{ClockState, ClockSync, GetClock, McuClock};
