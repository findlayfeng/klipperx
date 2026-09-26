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

// The generated enum names the payload types bare, so they are imported here:
// an event with a non-primitive payload (`homing:home_rails_end` carries the
// run's [`HomingHandle`](crate::core::klippy::motion::HomingHandle)) reads it
// from this scope.
use std::sync::{Arc, Mutex};

use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::motion::HomingHandle;

include!(concat!(env!("OUT_DIR"), "/klippy_events.rs"));

/// A shared handle to the results a probing move is about to report.
///
/// The probe fires `probe:update_results` with it so a consumer can edit the
/// reported result in place (`axis_twist_compensation`, `probe.py:329`), and
/// the sender reads back whatever the handlers left. It compares by pointer
/// identity — the list behind it changes as the handlers run — which is what
/// lets the generated [`KlippyEvent`] keep deriving `PartialEq`, the same way
/// [`HomingHandle`](crate::core::klippy::motion::HomingHandle) does.
#[derive(Debug, Clone)]
pub struct ProbeResultsHandle(Arc<Mutex<Vec<Coord>>>);

impl ProbeResultsHandle {
    /// Wrap one probing move's results.
    pub fn new(results: Vec<Coord>) -> Self {
        Self(Arc::new(Mutex::new(results)))
    }

    /// The shared list, for a consumer to read or edit in place.
    pub fn shared(&self) -> Arc<Mutex<Vec<Coord>>> {
        Arc::clone(&self.0)
    }

    /// A copy of the results after the handlers ran.
    pub fn to_vec(&self) -> Vec<Coord> {
        self.0.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }
}

impl PartialEq for ProbeResultsHandle {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}
