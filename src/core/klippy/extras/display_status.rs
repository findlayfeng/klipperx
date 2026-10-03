//! `[display_status]` — the `M73`/`M117` progress and message state
//! (upstream `klippy/extras/display_status.py`).
//!
//! | option | default | role |
//! |---|---|---|
//! | — | — | the section carries no options of its own |
//!
//! The object is what a panel draws: `progress` from `M73` (or from
//! `virtual_sdcard`), `message` from `M117`/`SET_DISPLAY_TEXT`. Upstream
//! registers the three commands in the object's constructor
//! (`display_status.py:19-25`), so they exist whenever the object does —
//! whether the config names `[display_status]` or `[display]` creates it on
//! demand ([`ensure`], which is upstream's `printer.load_object(config,
//! "display_status")` in `display/display.py:190`).
//!
//! # What is not here
//!
//! `idle_timeout`. Upstream clears a progress that is older than `M73_TIMEOUT`
//! only while `idle_timeout` reports something other than `Printing`
//! (`display_status.py:24-28`); this host's `[idle_timeout]` object exists
//! (批 #21) but is not consulted here yet, so an expired progress is cleared
//! unconditionally. `virtual_sdcard`'s progress is used as
//! the fallback, as upstream does, and is `0.` when there is no
//! `virtual_sdcard`.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::Reactor;

section!("display_status", order = 30, load = load_config);

/// The name the object is registered under, and the name a consumer looks it up
/// by (`printer.load_object(config, "display_status")`).
pub const DISPLAY_STATUS_OBJECT: &str = "display_status";

/// How long an `M73` progress stands on its own (`display_status.py:9`).
const M73_TIMEOUT: f64 = 5.0;

/// `SET_DISPLAY_TEXT`'s help text (`display_status.py:43`).
const SET_DISPLAY_TEXT_HELP: &str = "Set or clear the display message";

/// The `[display_status]` section: the progress percentage and the message
/// (`display_status.py:12-16`).
pub struct DisplayStatus {
    /// The machine, for the `virtual_sdcard` fallback.
    printer: Weak<Printer>,
    /// The machine's clock, for `M73`'s expiry.
    reactor: Arc<dyn Reactor>,
    /// The clamped `M73` progress (`display_status.py:14`).
    progress: Mutex<Option<f64>>,
    /// When the progress stops counting (`display_status.py:13-14`).
    expire_progress: Mutex<f64>,
    /// The `M117`/`SET_DISPLAY_TEXT` message (`display_status.py:15`).
    message: Mutex<Option<String>>,
}

impl DisplayStatus {
    /// Build the object and register its three commands
    /// (`display_status.py:12-25`).
    ///
    /// # Errors
    /// A printer without a g-code dispatcher, or a command name already taken.
    fn new(printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| {
                ConfigError::new("the g-code dispatcher is not registered".to_string())
            })?;
        let object = Arc::new(Self {
            printer: Arc::downgrade(printer),
            reactor: printer.reactor(),
            progress: Mutex::new(None),
            expire_progress: Mutex::new(0.0),
            message: Mutex::new(None),
        });
        for (name, command, desc, params) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(&object);
                sync(move |gcmd| command(&object, gcmd))
            };
            gcode
                .register_command_with_params(name, handler, *desc, params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(object)
    }

    /// The `virtual_sdcard` progress, or `0.` when there is none
    /// (`display_status.py:33-37`).
    fn sdcard_progress(&self, eventtime: f64) -> f64 {
        let Some(printer) = self.printer.upgrade() else {
            return 0.0;
        };
        match printer.lookup_object("virtual_sdcard") {
            Some(sdcard) => sdcard.get_status(eventtime)["progress"]
                .as_f64()
                .unwrap_or(0.0),
            None => 0.0,
        }
    }
}

impl PrinterObject for DisplayStatus {
    /// Upstream's `DisplayStatus.get_status` (`display_status.py:22-34`).
    fn get_status(&self, eventtime: f64) -> Value {
        let mut progress = *self.progress.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(value) = progress {
            let expires = *self
                .expire_progress
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if eventtime > expires {
                // Upstream consults `idle_timeout` here and keeps the progress
                // while it reports `Printing`; the object exists (批 #21) but is
                // not read here yet, so the timeout stands on its own.
                *self.progress.lock().unwrap_or_else(|p| p.into_inner()) = None;
                progress = None;
            }
            let _ = value;
        }
        let progress = match progress {
            Some(value) => value,
            None => self.sdcard_progress(eventtime),
        };
        let message = self
            .message
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        json!({ "progress": progress, "message": message })
    }
}

impl std::fmt::Debug for DisplayStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DisplayStatus")
            .field(
                "progress",
                &self.progress.lock().unwrap_or_else(|p| p.into_inner()),
            )
            .field(
                "message",
                &self.message.lock().unwrap_or_else(|p| p.into_inner()),
            )
            .finish()
    }
}

/// The three commands, with upstream's registrations
/// (`display_status.py:19-25`).
///
/// `M117` takes the rest of the line, not a named parameter, so it declares
/// none.
type Command = fn(&Arc<DisplayStatus>, &GcodeCommand) -> Result<(), CommandError>;
const COMMANDS: &[(&str, Command, Option<&str>, &[&str])] = &[
    ("M73", cmd_m73, None, &["P"]),
    ("M117", cmd_m117, None, &[]),
    (
        "SET_DISPLAY_TEXT",
        cmd_set_display_text,
        Some(SET_DISPLAY_TEXT_HELP),
        &["MSG"],
    ),
];

/// `M73 P<percent>` (`display_status.py:36-41`).
fn cmd_m73(object: &Arc<DisplayStatus>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    // Upstream reads `P` as an *optional* float (`gcmd.get_float('P', None)`),
    // clamps it afterwards, and leaves the progress alone when it is absent —
    // so a bare `M73` only re-arms nothing at all.
    let Some(raw) = gcmd.get_command_parameters().get("P") else {
        return Ok(());
    };
    let percent = parse_float(raw).ok_or_else(|| {
        CommandError::new(format!(
            "Error on '{}': unable to parse {raw}",
            gcmd.commandline()
        ))
    })?;
    *object.progress.lock().unwrap_or_else(|p| p.into_inner()) =
        Some((percent / 100.0).clamp(0.0, 1.0));
    *object
        .expire_progress
        .lock()
        .unwrap_or_else(|p| p.into_inner()) = object.reactor.monotonic() + M73_TIMEOUT;
    Ok(())
}

/// `M117 <message>` (`display_status.py:39-41`): the rest of the line, or
/// "clear" when there is nothing after the command.
fn cmd_m117(object: &Arc<DisplayStatus>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let message = gcmd.get_raw_command_parameters();
    *object.message.lock().unwrap_or_else(|p| p.into_inner()) = if message.is_empty() {
        None
    } else {
        Some(message)
    };
    Ok(())
}

/// `SET_DISPLAY_TEXT MSG=<message>` (`display_status.py:44-46`).
fn cmd_set_display_text(
    object: &Arc<DisplayStatus>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    *object.message.lock().unwrap_or_else(|p| p.into_inner()) = gcmd.get_str("MSG").ok();
    Ok(())
}

/// The `display_status` object, creating it when the config has no
/// `[display_status]` section.
///
/// Upstream's `printer.load_object(config, "display_status")`
/// (`display/display.py:190`): a display always has one.
///
/// # Errors
/// As [`DisplayStatus::new`], plus a name that is already taken.
pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<DisplayStatus>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<DisplayStatus>(DISPLAY_STATUS_OBJECT) {
        return Ok(existing);
    }
    let object = DisplayStatus::new(printer)?;
    printer.add_object(
        DISPLAY_STATUS_OBJECT,
        Arc::clone(&object) as Arc<dyn PrinterObject>,
    )?;
    Ok(object)
}

/// The factory `section!` names (`display_status.py:49-50 def load_config`).
///
/// # Errors
/// As [`DisplayStatus::new`].
pub fn load_config(
    _config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(DisplayStatus::new(printer)?)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{check_unused, AccessTracking, Config};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::reactor::ManualReactor;

    /// Load a config and bring the dispatcher to the ready state, so the
    /// commands registered for a ready printer can run.
    fn loaded(text: &str) -> Arc<Printer> {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    /// The bare `[display_status]` section loads with no options to read —
    /// upstream's class reads none of its own either (`display_status.py:12-16`)
    /// — and reports the rest state (`display_status.py:22-34`).
    #[test]
    fn the_bare_section_loads_and_leaves_no_option_unread() {
        let text = "[mcu]\nserial: /dev/a\n[display_status]\n";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let printer = loaded(text);
        check_unused(
            &config,
            printer.access_tracking().as_ref(),
            &["mcu".to_string(), "display_status".to_string()],
        )
        .expect("no option is left unread");

        let status = printer
            .lookup_object_as::<DisplayStatus>(DISPLAY_STATUS_OBJECT)
            .expect("the section registered the object");
        assert_eq!(
            status.get_status(0.0),
            json!({ "progress": 0., "message": Value::Null })
        );
    }

    /// A `[display]` that creates the object on demand still gets the three
    /// commands: upstream's `load_object` builds `DisplayStatus` wherever it is
    /// asked for (`display/display.py:190`).
    #[test]
    fn the_object_is_created_on_demand() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();

        let first = ensure(&printer).expect("the object is created");
        let second = ensure(&printer).expect("the object is reused");
        assert!(Arc::ptr_eq(&first, &second));

        // And the three commands are registered on it, visible once the
        // printer is ready (they are not `when_not_ready` commands).
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        let commands = gcode.get_status(0.0)["commands"].clone();
        for name in ["M73", "M117", "SET_DISPLAY_TEXT"] {
            assert!(commands.get(name).is_some(), "command {name}");
        }
        assert_eq!(
            commands["SET_DISPLAY_TEXT"]["help"],
            json!("Set or clear the display message")
        );
    }

    /// `M73` sets the progress and clamps it; `M117` and `SET_DISPLAY_TEXT`
    /// set and clear the message (`display_status.py:36-46`).
    #[tokio::test(flavor = "multi_thread")]
    async fn the_three_commands_drive_the_status() {
        let printer = loaded("[mcu]\nserial: /dev/a\n[display_status]\n");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");
        let status = printer
            .lookup_object_as::<DisplayStatus>(DISPLAY_STATUS_OBJECT)
            .expect("the object");

        gcode.run_script("M73 P50").await.unwrap();
        assert_eq!(status.get_status(0.0)["progress"], json!(0.5));

        // Out-of-range percentages are clamped, not refused
        // (`display_status.py:38-39`).
        gcode.run_script("M73 P200").await.unwrap();
        assert_eq!(status.get_status(0.0)["progress"], json!(1.0));
        gcode.run_script("M73 P-10").await.unwrap();
        assert_eq!(status.get_status(0.0)["progress"], json!(0.0));

        gcode.run_script("M117 Printing now").await.unwrap();
        assert_eq!(status.get_status(0.0)["message"], json!("Printing now"));

        gcode
            .run_script("SET_DISPLAY_TEXT MSG=Hello")
            .await
            .unwrap();
        assert_eq!(status.get_status(0.0)["message"], json!("Hello"));
        gcode.run_script("SET_DISPLAY_TEXT").await.unwrap();
        assert_eq!(status.get_status(0.0)["message"], Value::Null);

        // A bare `M117` clears the message (`display_status.py:40-41`).
        gcode.run_script("M117 Again").await.unwrap();
        gcode.run_script("M117").await.unwrap();
        assert_eq!(status.get_status(0.0)["message"], Value::Null);
    }

    /// An `M73` progress expires after `M73_TIMEOUT` (`display_status.py:24-28`).
    #[tokio::test(flavor = "multi_thread")]
    async fn an_expired_progress_is_dropped() {
        let printer = loaded("[mcu]\nserial: /dev/a\n[display_status]\n");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");
        let status = printer
            .lookup_object_as::<DisplayStatus>(DISPLAY_STATUS_OBJECT)
            .expect("the object");

        gcode.run_script("M73 P25").await.unwrap();
        let now = printer.eventtime();
        assert_eq!(status.get_status(now)["progress"], json!(0.25));
        assert_eq!(
            status.get_status(now + M73_TIMEOUT + 1.0)["progress"],
            json!(0.0)
        );
    }

    /// A bad `P` is refused with the dispatcher's own wording.
    #[tokio::test(flavor = "multi_thread")]
    async fn an_unparseable_percentage_is_refused() {
        let printer = loaded("[mcu]\nserial: /dev/a\n[display_status]\n");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");

        // `P=abc` keeps `P` a parameter (the parser uppercases the line and
        // splits on letters, so a bare `Pabc` is one parameter named `PABC` —
        // upstream reads that as "no `P`" and ignores it).
        let err = gcode.run_script("M73 P=abc").await.unwrap_err();

        assert!(err.to_string().contains("unable to parse"), "{err}");
    }

    /// The section's own options are the only thing to read, so an untouched
    /// tracker is not an error for it.
    #[test]
    fn no_option_is_read_by_the_section() {
        let access = AccessTracking::shared();
        let (config, _) = Config::from_text("[display_status]\n").unwrap();
        let section = config.get_section("display_status").unwrap();
        let wrapper = ConfigWrapper::new(section, Arc::clone(&access));

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let _ = load_config(&wrapper, &printer).expect("the section loads");

        assert_eq!(access.sections().len(), 0);
    }
}
