//! Printer-level event declarations, one file per namespace.
//!
//! Each `event!(...)` call names a wire event and, when it carries one, its
//! payload. The macro expands to nothing: the declarations exist so that
//! `build.rs` can read them and generate [`KlippyEvent`](super::KlippyEvent),
//! and so that a declaration is still ordinary Rust that an editor can parse.
//!
//! Add an event by adding a line to the file for its namespace; a new namespace
//! gets a new file and a line in the module list below. The generated enum and
//! `name()` update on the next build.

/// Declare one printer-level event. Expands to nothing; `build.rs` scans it.
///
/// Defined before the declaration modules are declared so that its textual
/// scope reaches them.
macro_rules! event {
    ($($tokens:tt)*) => {};
}

pub mod dual_carriage;
pub mod extruder;
pub mod gcode;
pub mod homing;
pub mod idle_timeout;
pub mod klippy;
pub mod load_cell;
pub mod menu;
pub mod probe;
pub mod stepper;
pub mod stepper_enable;
pub mod toolhead;
pub mod virtual_sdcard;
