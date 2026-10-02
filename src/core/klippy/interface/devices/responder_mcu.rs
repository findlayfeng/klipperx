//! 响应器式假 MCU（可多实例）: one fake board per `[mcu …]` section.
//!
//! [`SimulatorDevice`] is the responder itself — dictionary-driven, one
//! instance per connection. This module is the test-side handle that puts
//! *several* of them into one printer and finds each one again after the
//! handshake, which is what a multi-MCU test needs: two boards identify
//! against their own dictionaries, answer their own queries, and — once joined
//! with [`SimulatorDevice::link_machine`] — carry one machine's moves between
//! them so a stepper on one board can trip an endstop on another.
//!
//! Nothing here is a process-wide singleton: an instance is reached through
//! the `[mcu …]` section it serves, so two tests sharing one dictionary never
//! see each other's boards.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::core::klippy::interface::SimulatorDevice;
use crate::core::klippy::mcu::{Mcu, McuObject};
use crate::core::klippy::printer::Printer;

/// One fake MCU: the `[mcu]` / `[mcu <name>]` section served by its own
/// [`SimulatorDevice`] instance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResponderMcu {
    /// The section's sub — `"mcu"` for the bare `[mcu]`, the board's name for
    /// `[mcu <name>]`, and the key the printer registers its [`McuObject`]
    /// under.
    name: String,
    /// The data dictionary this instance serves.
    dict: PathBuf,
}

impl ResponderMcu {
    /// A fake board named `name` serving the built dictionary `dict_file`
    /// (e.g. `"atmega2560.dict"`).
    ///
    /// `None` when `build.rs` did not build that dictionary — `KLIPPERX_ARCHES`
    /// decides which exist — so a test skips instead of failing on a build
    /// product it never got.
    pub fn new(name: &str, dict_file: &str) -> Option<Self> {
        let dict = klipperx_test_support::test_dicts_dir().join(dict_file);
        dict.is_file().then(|| Self {
            name: name.to_string(),
            dict,
        })
    }

    /// The name this board's section and printer object are keyed by.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The dictionary file this board serves.
    pub fn dict(&self) -> &Path {
        &self.dict
    }

    /// This board's config block: `[mcu]\ntest: dict=<path>\n` for the bare
    /// `[mcu]`, `[mcu <name>]\n…` for a secondary board.
    pub fn section(&self) -> String {
        if self.name == "mcu" {
            format!("[mcu]\ntest: dict={}\n", self.dict.display())
        } else {
            format!("[mcu {}]\ntest: dict={}\n", self.name, self.dict.display())
        }
    }

    /// The `[mcu …]` blocks of every board in `boards`, in order: the whole
    /// MCU section list of a config.
    pub fn sections(boards: &[Self]) -> String {
        boards.iter().map(Self::section).collect()
    }

    /// The printer-object key this board registers under: `mcu` for the bare
    /// `[mcu]`, `mcu <name>` for a secondary board — the section's id, which
    /// is what the loader keys every prefix section by (`load.rs`).
    pub fn object_id(&self) -> String {
        if self.name == "mcu" {
            "mcu".to_string()
        } else {
            format!("mcu {}", self.name)
        }
    }

    /// The connected [`Mcu`] this board answers through, or `None` before
    /// `bring_up` (or when the section was opened on another transport).
    ///
    /// The fallible shape is deliberate: a multi-MCU test reads this from
    /// inside the phase that brings the machine up, and a panic there would
    /// skip the teardown that releases the devices.
    pub fn mcu(&self, printer: &Printer) -> Option<Arc<Mcu>> {
        printer
            .lookup_object_as::<McuObject>(&self.object_id())
            .and_then(|object| object.mcu())
    }

    /// This board's own responder instance — the one serving *its* identify,
    /// configuration handshake and queries — or `None` when this board was
    /// opened on a transport other than `test: dict=`.
    pub fn device(&self, printer: &Printer) -> Option<Arc<SimulatorDevice>> {
        self.mcu(printer).and_then(|mcu| mcu.simulator_device())
    }

    /// Join every named board into one machine, so a move started on any of
    /// them trips the armed checks on all of them
    /// ([`SimulatorDevice::link_machine`]). Call it after `bring_up`, before
    /// whatever exercises the machine's motion.
    ///
    /// # Errors
    /// Names the board that is not connected (or not a `test: dict=` fake).
    pub fn link_machine(boards: &[Self], printer: &Printer) -> Result<(), String> {
        let mut devices = Vec::with_capacity(boards.len());
        for board in boards {
            let device = board.device(printer).ok_or_else(|| {
                format!(
                    "[mcu {}] is not connected to a `test: dict=` fake",
                    board.name
                )
            })?;
            devices.push(device);
        }
        SimulatorDevice::link_machine(&devices);
        Ok(())
    }
}
