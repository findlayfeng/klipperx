//! The printer-level event vocabulary: [`KlippyEvent`].
//!
//! This is the machine's own event bus, distinct from the MCU event layer in
//! [`super`]. An MCU event is pushed by firmware and bound to a response name
//! ([`McuEvent`](super::McuEvent)); a `KlippyEvent` is fired by the host itself
//! — a lifecycle step, a restart, or a part reporting a state change — and is
//! named after the same wire names upstream passes to
//! `Printer.send_event`/`register_event_handler`.
//!
//! The enum is generated at build time from the declarations under `decl`, so a
//! part adds its events beside itself instead of editing a central list.
//! `build.rs` writes the definition to `$OUT_DIR/klippy_events.rs` and this
//! module includes it.

include!(concat!(env!("OUT_DIR"), "/klippy_events.rs"));
