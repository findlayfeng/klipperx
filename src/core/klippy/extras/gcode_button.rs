//! `[gcode_button <name>]` — run g-code when a hardware button is pressed or
//! released (upstream `klippy/extras/gcode_button.py`).
//!
//! The section is prefix-only (`load_config_prefix`, `gcode_button.py:42`). It
//! reads its options as upstream does:
//!
//! | option | default | role |
//! |---|---|---|
//! | `pin` | — (required) | the button's pin (`gcode_button.py:13`) |
//! | `press_gcode` | — (required) | the template rendered on press (`:19`) |
//! | `release_gcode` | `""` | the template rendered on release (`:20-21`) |
//! | `debounce_delay` | `0.`, minimum `0.` | read by the button registration (`buttons.py:255`) |
//! | `analog_range` | — | parsed, then refused — see below |
//! | `analog_pullup_resistor` | `4700.`, above `0.` | read only alongside `analog_range`, then refused |
//!
//! [`GCodeButton::button_callback`] is upstream's `button_callback`
//! (`gcode_button.py:29-39`): the new state is recorded, then the press template
//! (state set) or the release template (state clear) is rendered and, when it
//! is not blank, run through the dispatcher. [`get_status`] reports `PRESSED`
//! / `RELEASED` (`:36-39`), and `QUERY_BUTTON BUTTON=<name>` reports the same
//! (`:27-28`).
//!
//! # Gaps this port does not close yet
//!
//! * **The button event.** Upstream registers a debounced button with the
//!   `buttons` module (`:16`), whose firmware query (`buttons.py`'s
//!   `MCU_buttons`) fires the callback on a pin change. This host has no button
//!   query (see [`buttons`](crate::core::klippy::extras::buttons)), so the
//!   callback is recorded but never reached by the firmware. The corpus only
//!   *loads* `[gcode_button lcd_button]` (`printer-biqu-bx-2021.cfg:185`) and
//!   never presses it, so that is faithful on the corpus.
//! * **`analog_range`.** Upstream's analog branch (`gcode_button.py:17-21`)
//!   registers through `buttons.register_debounce_adc_button`, which rests on
//!   the `query_adc` object — absent from this host, the same reason
//!   [`adc_scaled`](crate::core::klippy::extras::adc_scaled) skips it. The two
//!   options are still read and validated (so their wording and the section's
//!   option check match upstream), but a section that sets `analog_range` is
//!   then refused with [`ANALOG_UNSUPPORTED`] rather than silently registered
//!   as a digital button.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::buttons::PrinterButtons;
use crate::core::klippy::extras::gcode_macro::PrinterGCodeMacro;
use crate::core::klippy::extras::template::{
    Builtin, Context, PrinterView, Rt, Template, TemplateError,
};
use crate::core::klippy::gcode::{sync, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form exists upstream (`gcode_button.py:42`).
section!("gcode_button", order = 30, prefix = load_config_prefix);

/// `cmd_QUERY_BUTTON_help` (`gcode_button.py:27`).
const QUERY_BUTTON_HELP: &str = "Report on the state of a button";

/// Why a section that sets `analog_range` is refused: the analog button path
/// needs `query_adc` / ADC-button infrastructure this host does not have (see
/// the module docs).
const ANALOG_UNSUPPORTED: &str =
    "is not supported: this host has no query_adc / ADC-button infrastructure";

/// One `[gcode_button <name>]` (upstream `GCodeButton`).
pub struct GCodeButton {
    /// The section suffix (`lcd_button`), the `QUERY_BUTTON` mux value
    /// (`gcode_button.py:11`).
    name: String,
    /// The `pin` option, kept for the button registration in
    /// [`GCodeButton::attach`].
    pin: String,
    /// `last_state` (`gcode_button.py:12`): the last state the button reported.
    last_state: Mutex<bool>,
    /// The compiled `press_gcode` (`:19`).
    press_template: Template,
    /// The compiled `release_gcode`, defaulting to empty (`:20-21`).
    release_template: Template,
    /// The machine, for the render context's `printer` view; weak so an object
    /// holding its button does not keep the machine alive.
    printer: Weak<Printer>,
    /// The dispatcher a button script is run through (`:22`).
    gcode: Arc<GCodeDispatch>,
}

impl GCodeButton {
    /// Read the section, load the templates, and take the dispatcher
    /// (`gcode_button.py:9-25`).
    ///
    /// # Errors
    /// A missing `pin` / `press_gcode`, a template
    /// [`template`](crate::core::klippy::extras::template) refuses to compile, or a
    /// section that sets `analog_range` (read and validated first, then
    /// refused — see the module docs).
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        let pin = config.get("pin", None)?;
        // Upstream loads the `buttons` module even before it branches on
        // `analog_range` (`gcode_button.py:14-16`).
        PrinterButtons::ensure(printer)?;
        if config.get_str("analog_range").is_some() {
            // Read both values the way upstream reads them (`:17-18`), so their
            // wording and the section's option check match, then report the gap.
            parse_float_list(config, "analog_range", 2)?;
            config.get_float_bounded(
                "analog_pullup_resistor",
                Some(4700.0),
                None,
                None,
                Some(0.0),
                None,
            )?;
            return Err(ConfigError::new(format!(
                "Option 'analog_range' in section '{}' {ANALOG_UNSUPPORTED}",
                config.identifier()
            )));
        }
        let gcode_macro = PrinterGCodeMacro::ensure(printer)?;
        let press_template = gcode_macro.load_template(config, "press_gcode", None)?;
        let release_template = gcode_macro.load_template(config, "release_gcode", Some(""))?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        Ok(Self {
            name,
            pin,
            last_state: Mutex::new(false),
            press_template,
            release_template,
            printer: Arc::downgrade(printer),
            gcode,
        })
    }

    /// Register the debounced button and the `QUERY_BUTTON` command
    /// (`gcode_button.py:16, :23-25`).
    ///
    /// Called after the `Arc` exists, so the handler can hold this button.
    ///
    /// # Errors
    /// A negative `debounce_delay` (read here, as upstream's `DebounceButton`
    /// reads it), or a taken command name.
    fn attach(
        self: &Arc<Self>,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<(), ConfigError> {
        let buttons = PrinterButtons::ensure(printer)?;
        let button = Arc::clone(self);
        buttons.register_debounce_button(
            config,
            &self.pin,
            Box::new(move |eventtime, state| {
                button.button_callback(eventtime, state);
            }),
        )?;

        let button = Arc::clone(self);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            gcmd.respond_info(&format!("{}: {}", button.name, button.state_name()));
            Ok(())
        });
        self.gcode
            .register_mux_command_with_params(
                "QUERY_BUTTON",
                "BUTTON",
                Some(&self.name),
                handler,
                Some(QUERY_BUTTON_HELP),
                &[],
            )
            .map_err(ConfigError::new)?;
        Ok(())
    }

    /// Upstream's `button_callback` (`gcode_button.py:29-39`): record the state,
    /// render the matching template, and run it when it is not blank.
    fn button_callback(&self, _eventtime: f64, state: bool) {
        *self.lock() = state;
        let template = if state {
            &self.press_template
        } else {
            &self.release_template
        };
        let Some(printer) = self.printer.upgrade() else {
            tracing::warn!("gcode_button '{}': the machine is gone", self.name);
            return;
        };
        let commands = match self.render(&printer, template) {
            Ok(commands) => commands,
            Err(error) => {
                tracing::error!("gcode_button '{}': {error}", self.name);
                return;
            }
        };
        if commands.trim().is_empty() {
            return;
        }
        let gcode = Arc::clone(&self.gcode);
        let name = self.name.clone();
        // Upstream's `self.gcode.run_script(commands)`, which is `async` here
        // while a button callback is not: the script runs on the host's runtime
        // and the callback does not wait for it.
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(error) = gcode.run_script(&commands).await {
                        tracing::error!("gcode_button '{name}': script error: {error}");
                    }
                });
            }
            Err(_) => {
                tracing::warn!(
                    "gcode_button '{name}': running the button script needs an async runtime"
                )
            }
        }
    }

    /// Render a template the way upstream's `TemplateWrapper.render` does with
    /// no context (`gcode_macro.py:61-68`): the `printer` view plus the two
    /// actions a loaded template may call.
    fn render(&self, printer: &Arc<Printer>, template: &Template) -> Result<String, TemplateError> {
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
        template.render(&mut context)
    }

    /// Upstream's `get_status` state name (`gcode_button.py:36-39`).
    fn state_name(&self) -> &'static str {
        if *self.lock() {
            "PRESSED"
        } else {
            "RELEASED"
        }
    }

    fn lock(&self) -> MutexGuard<'_, bool> {
        self.last_state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl std::fmt::Debug for GCodeButton {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GCodeButton")
            .field("name", &self.name)
            .field("pin", &self.pin)
            .finish_non_exhaustive()
    }
}

impl PrinterObject for GCodeButton {
    /// Upstream's `get_status` (`gcode_button.py:36-39`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "state": self.state_name() })
    }
}

/// Parse an option as a comma-separated float list of exactly `count` values.
///
/// The wrapper has no float-list getter; this is `getfloatlist(option,
/// count=2)` (`klippy/configfile.py:115`) with its two error wordings, the same
/// helper the display framework keeps (`display/display.rs`).
fn parse_float_list(
    config: &ConfigWrapper,
    option: &str,
    count: usize,
) -> Result<Vec<f64>, ConfigError> {
    let identifier = config.identifier();
    let text = config.get(option, None)?;
    let mut values = Vec::new();
    for part in text.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let value = part.parse::<f64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{identifier}'"
            ))
        })?;
        values.push(value);
    }
    if values.len() != count {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{identifier}' must have {count} elements"
        )));
    }
    Ok(values)
}

/// Upstream's `load_config_prefix` for `[gcode_button <name>]`
/// (`gcode_button.py:42-43`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(GCodeButton::new(config, printer)?);
    object.attach(config, printer)?;
    Ok(object)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::reactor::ManualReactor;

    /// A printer with `gcode` registered, as the loader builds it.
    ///
    /// The dispatcher refuses scripts before ready (`gcode.rs` state check);
    /// `gcode_macro.rs`'s machine lights the ready lamp the same way.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode registers");
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered")
    }

    /// Load the one `[gcode_button lcd_button]` section of `text` through the
    /// real factory, returning the button and the access tracker it recorded
    /// into.
    fn build(
        printer: &Arc<Printer>,
        text: &str,
    ) -> (Result<Arc<GCodeButton>, ConfigError>, Arc<AccessTracking>) {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let section = config
            .get_section("gcode_button lcd_button")
            .expect("the section exists");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(section, Arc::clone(&access));
        let result = GCodeButton::new(&wrapper, printer).and_then(|button| {
            let button = Arc::new(button);
            button.attach(&wrapper, printer).map(|()| button)
        });
        (result, access)
    }

    fn load_ok(printer: &Arc<Printer>, text: &str) -> Arc<GCodeButton> {
        build(printer, text).0.expect("the section loads")
    }

    /// Let the script a button callback spawns run (`ManualReactor` runs no
    /// tasks, so the test drives the runtime itself).
    async fn settle() {
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    /// The corpus section reads with upstream's defaults: `release_gcode`
    /// defaults to empty and nothing else is set.
    #[test]
    fn test_the_corpus_button_reads_every_option() {
        let printer = printer();
        let (result, access) = build(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    M117 pressed\n",
        );
        let button = result.expect("the section loads");
        assert_eq!(button.name, "lcd_button");
        assert_eq!(button.pin, "PH8");
        // Every option the digital path reads is recorded; `analog_range` is
        // absent and so read nothing.
        for option in ["pin", "press_gcode", "release_gcode", "debounce_delay"] {
            assert!(
                access.contains("gcode_button lcd_button", option),
                "unread option '{option}'"
            );
        }
        assert!(!access.contains("gcode_button lcd_button", "analog_range"));
        assert!(!access.contains("gcode_button lcd_button", "analog_pullup_resistor"));
    }

    /// Press and release each render their template and reach the dispatcher,
    /// and `get_status` follows the last state (`gcode_button.py:29-39`).
    #[tokio::test]
    async fn test_press_and_release_render_and_dispatch_their_templates() {
        let printer = printer();
        let seen = Arc::new(Mutex::new(Vec::<String>::new()));
        {
            let seen = Arc::clone(&seen);
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                seen.lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(gcmd.get_str("VALUE").unwrap_or_default());
                Ok(())
            });
            gcode(&printer)
                .register_command("RECORD", handler, None, false)
                .expect("the receiver registers");
        }
        let button = load_ok(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    RECORD VALUE=down\n\
             release_gcode:\n    RECORD VALUE=up\n",
        );
        assert_eq!(button.get_status(0.0), json!({ "state": "RELEASED" }));

        button.button_callback(0.0, true);
        settle().await;
        assert_eq!(
            seen.lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone(),
            vec!["down"]
        );
        assert_eq!(button.get_status(0.0), json!({ "state": "PRESSED" }));

        button.button_callback(1.0, false);
        settle().await;
        assert_eq!(
            seen.lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone(),
            vec!["down", "up"]
        );
        assert_eq!(button.get_status(0.0), json!({ "state": "RELEASED" }));
    }

    /// A release whose template is the empty default is not run
    /// (`gcode_button.py:20-21, :34-35`).
    #[tokio::test]
    async fn test_an_empty_release_template_is_not_run() {
        let printer = printer();
        let seen = Arc::new(Mutex::new(0usize));
        {
            let seen = Arc::clone(&seen);
            let handler: CommandHandler = sync(move |_| {
                *seen.lock().unwrap_or_else(|poison| poison.into_inner()) += 1;
                Ok(())
            });
            gcode(&printer)
                .register_command("RECORD", handler, None, false)
                .expect("the receiver registers");
        }
        let button = load_ok(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    RECORD\n",
        );

        button.button_callback(0.0, true);
        settle().await;
        assert_eq!(*seen.lock().unwrap_or_else(|poison| poison.into_inner()), 1);

        // The release template defaults to empty; the trimmed render is blank,
        // so nothing runs.
        button.button_callback(1.0, false);
        settle().await;
        assert_eq!(*seen.lock().unwrap_or_else(|poison| poison.into_inner()), 1);
        assert_eq!(button.get_status(0.0), json!({ "state": "RELEASED" }));
    }

    /// `QUERY_BUTTON BUTTON=<name>` is registered with upstream's help and
    /// reports the state `get_status` holds (`gcode_button.py:23-28`).
    #[test]
    fn test_query_button_is_registered_and_reports_the_state() {
        let printer = printer();
        let button = load_ok(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    M117 x\n",
        );
        assert_eq!(
            gcode(&printer).command_help().get("QUERY_BUTTON"),
            Some(&QUERY_BUTTON_HELP.to_string())
        );

        *button.lock() = true;
        gcode(&printer)
            .run_script_sync("QUERY_BUTTON BUTTON=lcd_button")
            .expect("the command runs");
    }

    /// A missing `pin` or `press_gcode` is refused with the loader's wording
    /// (`gcode_button.py:13, :19`).
    #[test]
    fn test_a_missing_required_option_is_refused() {
        let printer = printer();

        let (result, _) = build(
            &printer,
            "[gcode_button lcd_button]\npress_gcode:\n    M117 x\n",
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Option 'pin' in section 'gcode_button lcd_button' must be specified"),
            "{err}"
        );

        let (result, _) = build(&printer, "[gcode_button lcd_button]\npin: PH8\n");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains(
                "Option 'press_gcode' in section 'gcode_button lcd_button' must be specified"
            ),
            "{err}"
        );
    }

    /// `debounce_delay` is read by the registration with upstream's bound
    /// (`buttons.py:255`, `minval=0.`).
    #[test]
    fn test_a_negative_debounce_delay_is_refused() {
        let printer = printer();
        let (result, _) = build(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    M117 x\n\
             debounce_delay: -1\n",
        );
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must have minimum of 0"), "{err}");
    }

    /// A section that sets `analog_range` has both analog options read and
    /// validated first, then is refused with the gap message (see the module
    /// docs).
    #[test]
    fn test_analog_range_is_read_then_refused() {
        let printer = printer();

        let (result, access) = build(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    M117 x\n\
             analog_range: 1, 2\nanalog_pullup_resistor: 4700\n",
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains(
                "Option 'analog_range' in section 'gcode_button lcd_button' is not supported: \
                 this host has no query_adc / ADC-button infrastructure"
            ),
            "{err}"
        );
        assert!(access.contains("gcode_button lcd_button", "analog_range"));
        assert!(access.contains("gcode_button lcd_button", "analog_pullup_resistor"));
    }

    /// The two analog options keep upstream's parse and bound wording, reported
    /// before the gap message (`configfile.py:88-102`, `gcode_button.py:17-18`).
    #[test]
    fn test_analog_options_keep_upstream_wording() {
        let printer = printer();

        let (result, _) = build(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    M117 x\n\
             analog_range: 1, 2, 3\n",
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains(
                "Option 'analog_range' in section 'gcode_button lcd_button' must have 2 elements"
            ),
            "{err}"
        );

        let (result, _) = build(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    M117 x\n\
             analog_range: 1, two\n",
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains(
                "Unable to parse option 'analog_range' in section 'gcode_button lcd_button'"
            ),
            "{err}"
        );

        let (result, _) = build(
            &printer,
            "[gcode_button lcd_button]\npin: PH8\npress_gcode:\n    M117 x\n\
             analog_range: 1, 2\nanalog_pullup_resistor: 0\n",
        );
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must be above 0"), "{err}");
    }
}
