//! `[delayed_gcode <name>]` — run a g-code script after a delay
//! (upstream `klippy/extras/delayed_gcode.py`).
//!
//! | option | meaning |
//! |---|---|
//! | `gcode` | the script to run, as a template, required |
//! | `initial_duration` | seconds after `klippy:ready` before it runs (default 0, `≥ 0`; 0 means "do not run") |
//!
//! `UPDATE_DELAYED_GCODE ID=<name> DURATION=<seconds>` re-arms the timer:
//! `DURATION=0` disarms it, anything else wakes it that many seconds from now
//! (upstream's `minval=0.` bound rejects a negative value). The command is the
//! section's mux command, keyed by the section suffix.
//!
//! The object defines no `get_status` upstream, so it is not client-visible
//! ([`PrinterObject::is_queryable`] is `false`).
//!
//! # What is not here
//!
//! Upstream's `inside_timer` / `repeat` handshake is dropped. It exists because
//! `delayed_gcode.py:32` calls `gcode.run_script` **synchronously** inside the
//! timer callback: a `UPDATE_DELAYED_GCODE` issued *by the delayed script
//! itself* runs while `inside_timer` is set, so upstream records `repeat` and
//! wakes again at `eventtime + duration` (`:37-41`).
//!
//! Here a timer callback is synchronous and `GCodeDispatch::run_script` is not
//! (`gcode.rs`), so the script is run detached — `tokio::spawn`, the same
//! pattern as `extras/idle_timeout.rs` and `extras/gcode_button.rs` — and the
//! callback returns before the script runs. A `UPDATE_DELAYED_GCODE` from
//! inside the script therefore takes the ordinary arm path and registers a
//! fresh timer at `monotonic() + duration` (`:45-50`): the same next wake time
//! upstream's `repeat` produces, without the flag. The callback itself always
//! retires after firing, exactly as upstream's does when nothing repeats.
//!
//! A host with no async runtime to spawn on cannot run the script; the callback
//! logs a warning instead, as `idle_timeout.rs` does.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_macro::PrinterGCodeMacro;
use crate::core::klippy::extras::template::{
    Builtin, Context, PrinterView, Rt, Template, TemplateError,
};
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

// Only the prefix form (`[delayed_gcode <name>]`) exists upstream
// (`delayed_gcode.py:52`).
section!("delayed_gcode", order = 30, prefix = load_config_prefix);

/// Upstream's `reactor.NEVER` (`klippy/reactor.py:11`), the wake time that
/// means "registered but not scheduled".
const NEVER: f64 = 9_999_999_999_999_999.0;

/// `UPDATE_DELAYED_GCODE`'s help text, verbatim
/// (`delayed_gcode.py:43`).
const UPDATE_DELAYED_GCODE_HELP: &str = "Update the duration of a delayed_gcode";

/// One `[delayed_gcode <name>]`.
pub struct DelayedGcode {
    /// The section suffix, the mux value of `UPDATE_DELAYED_GCODE`
    /// (`delayed_gcode.py:17`).
    name: String,
    /// The machine, for its reactor; weak so holding the object does not keep
    /// the machine alive.
    printer: Weak<Printer>,
    /// The dispatcher the rendered script is run on (`delayed_gcode.py:18`).
    gcode: Arc<GCodeDispatch>,
    /// The compiled `gcode` option (`delayed_gcode.py:21`).
    timer_gcode: Template,
    /// `initial_duration` / `UPDATE_DELAYED_GCODE DURATION`
    /// (`delayed_gcode.py:22, :46`).
    duration: Mutex<f64>,
    /// The timer the `klippy:ready` handler creates and the command re-arms
    /// (upstream's `timer_handler`); cancelled on drop.
    timer: Mutex<Option<TimerHandle>>,
    /// The `Arc` the timer callback upgrades: the registry owns this object,
    /// and the reactor's callback must not.
    self_ref: Weak<DelayedGcode>,
}

impl DelayedGcode {
    /// Read the section, load the template and register the command
    /// (`delayed_gcode.py:16-33`).
    ///
    /// # Errors
    /// A missing `gcode`, an `initial_duration` below 0, a template
    /// [`template`](crate::core::klippy::extras::template) refuses to compile, a
    /// `gcode_macro` object that cannot be created, or a taken command name.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| {
                ConfigError::new("the gcode dispatcher is not registered".to_string())
            })?;
        let gcode_macro = PrinterGCodeMacro::ensure(printer)?;
        let timer_gcode = gcode_macro.load_template(config, "gcode", None)?;
        // `delayed_gcode.py:22`: `getfloat('initial_duration', 0., minval=0.)`.
        let duration =
            config.get_float_bounded("initial_duration", Some(0.0), Some(0.0), None, None, None)?;

        let object = Arc::new_cyclic(|weak| Self {
            name,
            printer: Arc::downgrade(printer),
            gcode,
            timer_gcode,
            duration: Mutex::new(duration),
            timer: Mutex::new(None),
            self_ref: weak.clone(),
        });

        let weak = Arc::downgrade(&object);
        object
            .gcode
            .register_mux_command(
                "UPDATE_DELAYED_GCODE",
                "ID",
                Some(&object.name),
                sync(move |gcmd: &GcodeCommand| {
                    let this = weak
                        .upgrade()
                        .ok_or_else(|| CommandError::new("the delayed_gcode object is gone"))?;
                    this.cmd_update_delayed_gcode(gcmd)
                }),
                Some(UPDATE_DELAYED_GCODE_HELP),
            )
            .map_err(ConfigError::new)?;
        Ok(object)
    }

    /// The configured duration.
    fn duration(&self) -> f64 {
        *self.duration.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Register the timer (`DelayedGcode._handle_ready`, `delayed_gcode.py:34-39`):
    /// armed for `initial_duration` seconds, or retired at [`NEVER`] when the
    /// duration is 0.
    fn handle_ready(&self) {
        let duration = self.duration();
        let waketime = if duration != 0.0 {
            match self.printer.upgrade() {
                Some(printer) => printer.reactor().monotonic() + duration,
                None => return,
            }
        } else {
            NEVER
        };
        self.arm(waketime);
    }

    /// (Re-)register the timer for `waketime`.
    ///
    /// The stand-in for upstream's `reactor.update_timer(timer, waketime)`
    /// (`delayed_gcode.py:50`), which this host's reactor has no equivalent
    /// for (see `idle_timeout.rs`): the old handle is cancelled and a fresh
    /// timer is registered with the same callback.
    fn arm(&self, waketime: f64) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "delayed_gcode",
            Box::new(move |eventtime| {
                let this = weak.upgrade()?;
                this.tick(eventtime)
            }),
            waketime,
        );
        let mut timer = self.timer.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(previous) = timer.replace(handle) {
            previous.cancel();
        }
    }

    /// One timer callback (`DelayedGcode._gcode_timer_event`,
    /// `delayed_gcode.py:40-49`): render the template, run it, and retire.
    ///
    /// The returned `None` is upstream's `NEVER` (`:44-47`); the command
    /// re-arms a retired timer through [`DelayedGcode::arm`] (module docs).
    fn tick(&self, _eventtime: f64) -> Option<f64> {
        let Some(printer) = self.printer.upgrade() else {
            return None;
        };
        match self.render(&printer) {
            Ok(script) => {
                // Upstream's `gcode.run_script(script)`, which is `async` here
                // while a timer callback is not: the script runs on the host's
                // runtime and the callback does not wait for it (module docs).
                let gcode = Arc::clone(&self.gcode);
                match tokio::runtime::Handle::try_current() {
                    Ok(handle) => {
                        handle.spawn(async move {
                            if let Err(error) = gcode.run_script(&script).await {
                                tracing::error!("Script running error: {error}");
                            }
                        });
                    }
                    Err(_) => tracing::warn!(
                        "delayed_gcode '{}': running the script needs an async runtime",
                        self.name
                    ),
                }
            }
            // Upstream logs the exception and retires the timer all the same.
            Err(error) => tracing::error!("Script running error: {error}"),
        }
        None
    }

    /// Render the `gcode` template the way upstream's `TemplateWrapper.render`
    /// does with no context (`gcode_macro.py:61-68`): the `printer` view plus
    /// the actions a loaded template may call.
    ///
    /// # Errors
    /// The first template error, as upstream's `render` raises it.
    fn render(&self, printer: &Arc<Printer>) -> Result<String, TemplateError> {
        let mut context = Context::new();
        context.insert(
            "printer",
            Rt::Printer(PrinterView::new(Arc::clone(printer))),
        );
        context.insert(
            "action_respond_info",
            Rt::Builtin(Builtin::RespondInfo(Arc::clone(printer))),
        );
        context.insert("action_raise_error", Rt::Builtin(Builtin::RaiseError));
        context.insert("range", Rt::Builtin(Builtin::Range));
        self.timer_gcode.render(&mut context)
    }

    /// `UPDATE_DELAYED_GCODE ID=<name> DURATION=<seconds>`
    /// (`delayed_gcode.py:45-50`).
    ///
    /// # Errors
    /// A missing `DURATION`, a value that is not a float, or one below 0.
    fn cmd_update_delayed_gcode(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let duration =
            gcmd.get::<f64>("DURATION", None, parse_float, Some(0.0), None, None, None)?;
        let waketime = if duration != 0.0 {
            match self.printer.upgrade() {
                Some(printer) => printer.reactor().monotonic() + duration,
                None => return Ok(()),
            }
        } else {
            NEVER
        };
        *self.duration.lock().unwrap_or_else(|p| p.into_inner()) = duration;
        self.arm(waketime);
        Ok(())
    }
}

impl PrinterObject for DelayedGcode {
    /// Upstream's `delayed_gcode` defines no `get_status`, so it is not
    /// client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for DelayedGcode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DelayedGcode")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

impl Drop for DelayedGcode {
    fn drop(&mut self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

/// Wire the built object into the printer's `klippy:ready` event.
///
/// Upstream registers that handler inside `__init__`; here the `Arc` exists
/// only after construction, so the handler is attached in `load_config_prefix`.
fn on_ready(printer: &Arc<Printer>, delayed: &Arc<DelayedGcode>) {
    let weak = Arc::downgrade(delayed);
    printer.register_event_handler(
        KlippyEvent::KlippyReady,
        Box::new(move |_| {
            if let Some(this) = weak.upgrade() {
                this.handle_ready();
            }
        }),
    );
}

/// Upstream's `load_config_prefix` for `[delayed_gcode <name>]`
/// (`delayed_gcode.py:52-53`).
///
/// # Errors
/// A missing `gcode`, an invalid `initial_duration`, or a template that does
/// not compile.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let delayed = DelayedGcode::new(config, printer)?;
    on_ready(printer, &delayed);
    Ok(delayed)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    /// A machine with `text` loaded and the ready lamp lit, as the loader
    /// leaves it, over a clock the test drives.
    fn loaded(text: &str) -> (Arc<Printer>, Arc<ManualReactor>) {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(
            Arc::clone(&reactor) as Arc<dyn crate::core::klippy::reactor::Reactor>
        ));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, reactor)
    }

    /// A machine with `text` loaded but the ready lamp still dark.
    fn loaded_before_ready(text: &str) -> (Arc<Printer>, Arc<ManualReactor>) {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(
            Arc::clone(&reactor) as Arc<dyn crate::core::klippy::reactor::Reactor>
        ));
        printer.load_config(&config).expect("the config loads");
        (printer, reactor)
    }

    /// The section's object.
    fn delayed(printer: &Arc<Printer>) -> Arc<DelayedGcode> {
        printer
            .lookup_object_as::<DelayedGcode>("delayed_gcode welcome")
            .expect("the section registered the object")
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered")
    }

    /// Register `RECORD VALUE=<text>` and return what it was told, in order.
    fn record(printer: &Arc<Printer>) -> Arc<Mutex<Vec<String>>> {
        let seen = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&seen);
        let handler = sync(move |gcmd: &GcodeCommand| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(gcmd.get_str("VALUE").unwrap_or_default());
            Ok(())
        });
        gcode(printer)
            .register_command("RECORD", handler, None, false)
            .expect("the receiver registers");
        seen
    }

    fn recorded(seen: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        seen.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// Let the script a timer callback spawns run (`ManualReactor` runs no
    /// tasks, so the test drives the runtime itself).
    async fn settle() {
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    /// The template is rendered through the real loader and the result is what
    /// reaches the dispatcher, `initial_duration` seconds after
    /// `klippy:ready` (`delayed_gcode.py:21, :25, :34-49`).
    #[tokio::test]
    async fn test_the_initial_duration_runs_the_rendered_gcode_after_ready() {
        let (printer, reactor) = loaded_before_ready(
            "[delayed_gcode welcome]\ninitial_duration: 5\n\
             gcode:\n    {% if 'gcode' in printer %}RECORD VALUE=rendered{% endif %}\n",
        );
        let seen = record(&printer);

        // The timer is registered at `klippy:ready`, so nothing runs before it.
        reactor.advance(10.0);
        settle().await;
        assert_eq!(recorded(&seen), Vec::<String>::new());

        printer.send_event(&KlippyEvent::KlippyReady);
        reactor.advance(4.9);
        settle().await;
        assert_eq!(recorded(&seen), Vec::<String>::new());

        reactor.advance(0.2);
        settle().await;
        assert_eq!(recorded(&seen), ["rendered"]);
    }

    /// A callback fires once and retires: upstream returns `NEVER` from
    /// `_gcode_timer_event` (`delayed_gcode.py:44-47`), so a later clock has
    /// nothing left to run.
    #[tokio::test]
    async fn test_the_timer_retires_after_it_fires() {
        let (printer, reactor) = loaded(
            "[delayed_gcode welcome]\ninitial_duration: 1\n\
             gcode:\n    RECORD VALUE=once\n",
        );
        let seen = record(&printer);

        reactor.advance(1.1);
        settle().await;
        assert_eq!(recorded(&seen), ["once"]);

        reactor.advance(60.0);
        settle().await;
        assert_eq!(recorded(&seen), ["once"]);
    }

    /// `UPDATE_DELAYED_GCODE DURATION=<seconds>` arms a retired timer
    /// (`delayed_gcode.py:45-50`).
    #[tokio::test]
    async fn test_update_delayed_gcode_arms_the_timer() {
        let (printer, reactor) =
            loaded("[delayed_gcode welcome]\ngcode:\n    RECORD VALUE=armed\n");
        let seen = record(&printer);

        // `initial_duration` defaults to 0: nothing is scheduled.
        reactor.advance(1.0);
        settle().await;
        assert_eq!(recorded(&seen), Vec::<String>::new());

        gcode(&printer)
            .run_script("UPDATE_DELAYED_GCODE ID=welcome DURATION=2")
            .await
            .expect("the command runs");
        reactor.advance(1.9);
        settle().await;
        assert_eq!(recorded(&seen), Vec::<String>::new());

        reactor.advance(0.2);
        settle().await;
        assert_eq!(recorded(&seen), ["armed"]);
        assert_eq!(delayed(&printer).duration(), 2.0);
    }

    /// `DURATION=0` disarms the timer (`delayed_gcode.py:47-49`: the wake time
    /// becomes `NEVER`).
    #[tokio::test]
    async fn test_duration_zero_cancels_the_timer() {
        let (printer, reactor) = loaded(
            "[delayed_gcode welcome]\ninitial_duration: 2\n\
             gcode:\n    RECORD VALUE=late\n",
        );
        let seen = record(&printer);

        gcode(&printer)
            .run_script("UPDATE_DELAYED_GCODE ID=welcome DURATION=0")
            .await
            .expect("the command runs");
        reactor.advance(10.0);
        settle().await;
        assert_eq!(recorded(&seen), Vec::<String>::new());
        assert_eq!(delayed(&printer).duration(), 0.0);
    }

    /// `DURATION` is required and has upstream's `minval=0.` bound
    /// (`delayed_gcode.py:46`).
    #[test]
    fn test_update_delayed_gcode_refuses_a_missing_or_negative_duration() {
        let (printer, _reactor) = loaded("[delayed_gcode welcome]\ngcode:\n    RECORD VALUE=x\n");

        let error = gcode(&printer)
            .run_script_sync("UPDATE_DELAYED_GCODE ID=welcome")
            .expect_err("DURATION is required");
        assert_eq!(
            error.to_string(),
            "Error on 'UPDATE_DELAYED_GCODE ID=welcome': missing DURATION"
        );

        let error = gcode(&printer)
            .run_script_sync("UPDATE_DELAYED_GCODE ID=welcome DURATION=-1")
            .expect_err("a negative duration is refused");
        assert_eq!(
            error.to_string(),
            "Error on 'UPDATE_DELAYED_GCODE ID=welcome DURATION=-1': DURATION must have minimum of 0"
        );
    }

    /// The command is registered with upstream's help text and keyed by the
    /// section suffix (`delayed_gcode.py:27-32, :43`).
    #[test]
    fn test_update_delayed_gcode_is_registered_with_upstreams_help() {
        let (printer, _reactor) = loaded("[delayed_gcode welcome]\ngcode:\n    RECORD VALUE=x\n");

        assert_eq!(
            gcode(&printer).command_help().get("UPDATE_DELAYED_GCODE"),
            Some(&UPDATE_DELAYED_GCODE_HELP.to_string())
        );

        // Another section's name is not this registration's mux value.
        let error = gcode(&printer)
            .run_script_sync("UPDATE_DELAYED_GCODE ID=other DURATION=1")
            .expect_err("only the registered value is dispatched");
        assert!(error.to_string().contains("other"), "{error}");
    }

    /// Upstream defines no `get_status` for the section, so it is not
    /// client-visible.
    #[test]
    fn test_the_status_is_empty_and_not_queryable() {
        let (printer, _reactor) = loaded("[delayed_gcode welcome]\ngcode:\n    RECORD VALUE=x\n");
        let object = delayed(&printer);

        assert_eq!(object.get_status(0.0), json!({}));
        assert!(!object.is_queryable());
    }

    /// Every option the section is given is read — the corpus section
    /// (`printer-biqu-bx-2021.cfg:208-210`) sets exactly `initial_duration`
    /// and `gcode` (`delayed_gcode.py:21-22`).
    #[test]
    fn test_every_option_the_section_is_given_is_read() {
        let (printer, _reactor) = loaded(
            "[delayed_gcode welcome]\ninitial_duration: 1\n\
             gcode:\n    RECORD VALUE=x\n",
        );
        let access = printer.access_tracking();
        for option in ["gcode", "initial_duration"] {
            assert!(
                access.contains("delayed_gcode welcome", option),
                "unread option '{option}'"
            );
        }
    }

    /// A missing `gcode` is refused with the loader's wording
    /// (`delayed_gcode.py:21`).
    #[test]
    fn test_a_missing_gcode_is_refused() {
        let (config, _) =
            Config::from_text("[delayed_gcode welcome]\ninitial_duration: 1\n").expect("parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let error = printer.load_config(&config).expect_err("gcode is required");
        assert_eq!(
            error.to_string(),
            "Option 'gcode' in section 'delayed_gcode welcome' must be specified"
        );
    }

    /// `initial_duration` has upstream's `minval=0.` bound
    /// (`delayed_gcode.py:22`).
    #[test]
    fn test_a_negative_initial_duration_is_refused() {
        let (config, _) = Config::from_text(
            "[delayed_gcode welcome]\ninitial_duration: -1\n\
             gcode:\n    RECORD VALUE=x\n",
        )
        .expect("parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let error = printer
            .load_config(&config)
            .expect_err("a negative duration is refused");
        assert_eq!(
            error.to_string(),
            "Option 'initial_duration' in section 'delayed_gcode welcome' must have minimum of 0"
        );
    }
}
