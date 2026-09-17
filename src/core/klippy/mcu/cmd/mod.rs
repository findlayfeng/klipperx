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
//! The dependency runs one way for everything that is a *capability*: a command
//! module uses `Mcu`, while the frame, dictionary, and codec code never mention
//! one. The single edge back is identify — `mcu::identify` drives the pair defined
//! in `cmd::identify`, because that exchange belongs to the transport (it is the
//! one command whose formats the host owns, and it runs before any dictionary
//! exists).
//!
//! Because the host learns every format from the firmware, a command module
//! never hard-codes a format string or a wire id: it names the message and its
//! parameters, and the dictionary supplies the rest.
//!
//! Identify is here even though its *transfer* is not: the pair
//! (`identify` / `identify_response`) is the bootstrap exchange that produces the
//! dictionary, so its formats are host-owned and its chunk loop lives with the
//! transport in `mcu::identify`. What stays in the command layer is the part
//! every command has — the typed view and the arguments it sends.
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
pub mod identify;

pub use clock::{ClockState, ClockSync, GetClock, McuClock};
