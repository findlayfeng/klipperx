//! `[respond]` — the host's echo commands (upstream `klippy/extras/respond.py`).
//!
//! The bare section reads two options and registers two commands, both of them
//! available **before** the printer is ready (upstream's third
//! `register_command` argument is `True`, `respond.py:26-29`), so a slicer's
//! `M118` works while the config is still being read:
//!
//! | command | role (`respond.py`) |
//! |---|---|
//! | `M118` | echo the **raw** parameter text behind the default prefix (`:30-32`) |
//! | `RESPOND` | echo `MSG` behind `TYPE`'s or `PREFIX`'s prefix (`:34-51`) |
//!
//! | option | meaning |
//! |---|---|
//! | `default_type` | which prefix `M118` and a `RESPOND` without `TYPE` use: `echo` → `echo:`, `command` → `//`, `error` → `!!` (default `echo`) |
//! | `default_prefix` | a literal prefix that overrides whatever `default_type` picks |
//!
//! `RESPOND`'s `TYPE` additionally accepts `echo_no_space`: the same `echo:`
//! prefix but **no** separating space (`respond_types_no_space`,
//! `respond.py:11-13`). `PREFIX` overrides the prefix the type selected, and
//! `MSG` defaults to the empty string — so a bare `RESPOND` emits the default
//! prefix and a trailing space, exactly as upstream does.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("respond", order = 30, load = load_config);

/// The prefix each `default_type` / `TYPE` name picks (upstream's
/// `respond_types`, `respond.py:7-10`).
const RESPOND_TYPES: [(&str, &str); 3] = [("echo", "echo:"), ("command", "//"), ("error", "!!")];

/// The `default_type` choices, in the order upstream's table spells them.
///
/// Kept beside [`RESPOND_TYPES`] rather than derived from it so the choice list
/// is the argument `get_choice` takes; a test pins the two together.
const DEFAULT_TYPE_CHOICES: [&str; 3] = ["echo", "command", "error"];

/// The `TYPE` that keeps the `echo:` prefix but drops the separating space
/// (`respond_types_no_space`, `respond.py:11-13`).
const ECHO_NO_SPACE: &str = "echo_no_space";

/// The help text upstream registers `RESPOND` with (`respond.py:33`).
const RESPOND_HELP: &str = "Echo the message prepended with a prefix";

/// The prefix a type name maps to, or `None` when it is not one of
/// [`RESPOND_TYPES`].
fn prefix_of(respond_type: &str) -> Option<&'static str> {
    RESPOND_TYPES
        .iter()
        .find(|(name, _)| *name == respond_type)
        .map(|(_, prefix)| *prefix)
}

/// The `[respond]` module object (upstream's `HostResponder`).
///
/// Its only state is the resolved `default_prefix`, which the two registered
/// handlers capture; upstream keeps the same two attributes
/// (`respond.py:17-19`).
pub struct HostResponder {
    /// The prefix a command uses when the client names none: `default_prefix`,
    /// or the one `default_type` selected.
    default_prefix: String,
}

impl HostResponder {
    /// Read the section and register `M118` and `RESPOND`.
    ///
    /// # Errors
    /// Returns a config error naming the section when `default_type` is not a
    /// choice, or when the dispatcher rejects a registration.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let default_type =
            config.get_choice("default_type", &DEFAULT_TYPE_CHOICES, Some("echo"))?;
        let default_prefix = config.get(
            "default_prefix",
            Some(prefix_of(&default_type).expect("get_choice accepts only RESPOND_TYPES names")),
        )?;

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        // Both commands are `when_not_ready`: a slicer may send `M118` (or a
        // macro may `RESPOND`) before the config finishes loading.
        let m118_prefix = default_prefix.clone();
        let handler: CommandHandler = sync(move |gcmd| cmd_m118(&m118_prefix, gcmd));
        gcode
            .register_command("M118", handler, None, true)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let respond_prefix = default_prefix.clone();
        let handler: CommandHandler = sync(move |gcmd| cmd_respond(&respond_prefix, gcmd));
        gcode
            .register_command("RESPOND", handler, Some(RESPOND_HELP), true)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self { default_prefix })
    }

    /// The resolved prefix, the value upstream keeps in `default_prefix`.
    pub fn default_prefix(&self) -> &str {
        &self.default_prefix
    }
}

impl PrinterObject for HostResponder {
    /// Upstream's `HostResponder` defines no `get_status`, so it is not
    /// client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for HostResponder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HostResponder")
            .field("default_prefix", &self.default_prefix)
            .finish()
    }
}

/// `M118 <raw text>`: echo the text after the command verbatim, behind the
/// default prefix (`respond.py:30-32`).
///
/// The text is the **raw** remainder (`get_raw_command_parameters`), so quotes
/// and their contents survive: the command never goes through the
/// `KEY=VALUE` parser.
fn cmd_m118(default_prefix: &str, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let msg = gcmd.get_raw_command_parameters();
    gcmd.respond_raw(&format!("{default_prefix} {msg}"));
    Ok(())
}

/// `RESPOND [TYPE=<t>] [PREFIX=<p>] [MSG=<m>]`: echo `MSG` behind a prefix
/// (`respond.py:34-51`).
///
/// `TYPE` picks the prefix and, for `echo_no_space`, suppresses the separating
/// space; an unknown `TYPE` is upstream's error. `PREFIX` overrides the
/// selected prefix, and `MSG` defaults to the empty string.
fn cmd_respond(default_prefix: &str, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let mut no_space = false;
    let mut prefix = default_prefix.to_string();

    if let Some(respond_type) = gcmd.get_command_parameters().get("TYPE") {
        let respond_type = respond_type.to_lowercase();
        if let Some(selected) = prefix_of(&respond_type) {
            prefix = selected.to_string();
        } else if respond_type == ECHO_NO_SPACE {
            prefix = prefix_of("echo")
                .expect("`echo` is a RESPOND_TYPES name")
                .to_string();
            no_space = true;
        } else {
            return Err(CommandError::new(format!(
                "RESPOND TYPE '{respond_type}' is invalid. Must be one of 'echo', \
                 'command', or 'error'"
            )));
        }
    }

    if let Some(override_prefix) = gcmd.get_command_parameters().get("PREFIX") {
        prefix = override_prefix.clone();
    }
    let msg = gcmd.get_str_default("MSG", "");
    if no_space {
        gcmd.respond_raw(&format!("{prefix}{msg}"));
    } else {
        gcmd.respond_raw(&format!("{prefix} {msg}"));
    }
    Ok(())
}

/// The factory `section!` names (`respond.py:54-55`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(HostResponder::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::Mutex;

    /// A printer with `[respond]` and the options in `options`, and a sink for
    /// every line the dispatcher emits. `ready` lights the ready lamp.
    fn loaded(options: &str, ready: bool) -> (Arc<Printer>, Arc<GCodeDispatch>, Lines) {
        let (config, _) =
            Config::from_text(&format!("[respond]\n{options}")).expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        if ready {
            printer.send_event(&KlippyEvent::KlippyReady);
        }
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let lines = Lines::capture(&gcode);
        (printer, gcode, lines)
    }

    /// A printer that is ready, with a sink for the emitted lines.
    fn machine(options: &str) -> (Arc<Printer>, Arc<GCodeDispatch>, Lines) {
        loaded(options, true)
    }

    /// Every line the dispatcher emitted, in order.
    #[derive(Clone, Default)]
    struct Lines(Arc<Mutex<Vec<String>>>);

    impl Lines {
        fn capture(gcode: &Arc<GCodeDispatch>) -> Self {
            let lines = Self::default();
            let sink = lines.clone();
            gcode.register_output_handler(Arc::new(move |line: &str| {
                sink.0
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(line.to_string());
            }));
            lines
        }

        fn emitted(&self) -> Vec<String> {
            self.0
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone()
        }
    }

    /// Assemble `[respond]` from `options` and return the load error message.
    fn load_error(options: &str) -> String {
        let (config, _) =
            Config::from_text(&format!("[respond]\n{options}")).expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .load_config(&config)
            .expect_err("the section is refused")
            .to_string()
    }

    /// The two tables the choices come from name the same three types.
    #[test]
    fn test_the_choice_list_matches_the_respond_types_table() {
        let names: Vec<&str> = RESPOND_TYPES.iter().map(|(name, _)| *name).collect();
        assert_eq!(names, DEFAULT_TYPE_CHOICES);
        assert_eq!(prefix_of("echo"), Some("echo:"));
        assert_eq!(prefix_of("command"), Some("//"));
        assert_eq!(prefix_of("error"), Some("!!"));
        assert_eq!(prefix_of(ECHO_NO_SPACE), None);
    }

    /// Each `default_type` selects its own prefix and is the value kept on the
    /// object (`respond.py:18-19`).
    #[test]
    fn test_the_default_type_selects_the_default_prefix() {
        for (default_type, prefix) in RESPOND_TYPES {
            let (printer, gcode, lines) = machine(&format!("default_type: {default_type}\n"));
            let object = printer
                .lookup_object_as::<HostResponder>("respond")
                .expect("the section registered the object");
            assert_eq!(object.default_prefix(), prefix);
            gcode.run_script_sync("M118 hi").unwrap();
            assert_eq!(lines.emitted(), [format!("{prefix} hi")]);
        }
    }

    /// `default_prefix` overrides the prefix `default_type` picked
    /// (`respond.py:19`).
    #[test]
    fn test_default_prefix_overrides_the_default_type() {
        let (_printer, gcode, lines) = machine("default_type: command\ndefault_prefix: RESP\n");

        gcode.run_script_sync("M118 hi").unwrap();

        assert_eq!(lines.emitted(), ["RESP hi"]);
    }

    /// An unknown `default_type` is the choice error, naming the section.
    #[test]
    fn test_an_unknown_default_type_is_a_choice_error() {
        assert_eq!(
            load_error("default_type: shout\n"),
            "Choice 'shout' for option 'default_type' in section 'respond' is not a valid choice"
        );
    }

    /// The default `default_type` is `echo` (`respond.py:18`).
    #[test]
    fn test_the_default_type_defaults_to_echo() {
        let (_printer, gcode, lines) = machine("");

        gcode.run_script_sync("M118 hi").unwrap();

        assert_eq!(lines.emitted(), ["echo: hi"]);
    }

    /// `M118` echoes the raw remainder: quotes, spaces and an empty tail are
    /// passed through untouched (`respond.py:30-32`).
    #[test]
    fn test_m118_passes_the_raw_parameters_through() {
        let (_printer, gcode, lines) = machine("");

        gcode.run_script_sync("M118 \"hello world\"  x").unwrap();
        gcode.run_script_sync("M118").unwrap();

        assert_eq!(lines.emitted(), ["echo: \"hello world\"  x", "echo: "]);
    }

    /// The three valid `TYPE`s select their prefixes (`respond.py:39-43`).
    #[test]
    fn test_respond_type_selects_the_prefix() {
        let (_printer, gcode, lines) = machine("");

        for respond_type in ["echo", "command", "error"] {
            gcode
                .run_script_sync(&format!("RESPOND TYPE={respond_type} MSG=x"))
                .unwrap();
        }

        assert_eq!(lines.emitted(), ["echo: x", "// x", "!! x"]);
    }

    /// `TYPE` is matched case-insensitively (`respond.py:40`).
    #[test]
    fn test_respond_type_is_case_insensitive() {
        let (_printer, gcode, lines) = machine("");

        gcode.run_script_sync("RESPOND TYPE=COMMAND MSG=x").unwrap();

        assert_eq!(lines.emitted(), ["// x"]);
    }

    /// `echo_no_space` keeps the `echo:` prefix but drops the separating space
    /// (`respond.py:11-13,46-49`).
    #[test]
    fn test_respond_echo_no_space_drops_the_separator() {
        let (_printer, gcode, lines) = machine("");

        gcode
            .run_script_sync("RESPOND TYPE=echo_no_space MSG=hi")
            .unwrap();

        assert_eq!(lines.emitted(), ["echo:hi"]);
    }

    /// `PREFIX` overrides the prefix the type selected (`respond.py:44`).
    #[test]
    fn test_respond_prefix_overrides_the_type() {
        let (_printer, gcode, lines) = machine("");

        gcode
            .run_script_sync("RESPOND TYPE=command PREFIX=XX MSG=x")
            .unwrap();

        assert_eq!(lines.emitted(), ["XX x"]);
    }

    /// A `RESPOND` without `MSG` uses the empty string, so the line ends after
    /// the separator (`respond.py:45`).
    #[test]
    fn test_respond_msg_defaults_to_empty() {
        let (_printer, gcode, lines) = machine("");

        gcode.run_script_sync("RESPOND").unwrap();

        assert_eq!(lines.emitted(), ["echo: "]);
    }

    /// An unknown `TYPE` is upstream's error, with the type lowercased the way
    /// the handler does (`respond.py:42-43`).
    #[test]
    fn test_an_unknown_respond_type_is_reported() {
        let (_printer, gcode, lines) = machine("");

        let err = gcode
            .run_script_sync("RESPOND TYPE=Bogus MSG=x")
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "RESPOND TYPE 'bogus' is invalid. Must be one of 'echo', 'command', or 'error'"
        );
        assert_eq!(
            lines.emitted(),
            ["!! RESPOND TYPE 'bogus' is invalid. Must be one of 'echo', 'command', or 'error'"]
        );
    }

    /// Both commands run before the ready lamp is lit (`respond.py:26-29`).
    #[test]
    fn test_both_commands_are_available_before_ready() {
        let (_printer, gcode, lines) = loaded("", false);

        gcode.run_script_sync("M118 hi").unwrap();
        gcode.run_script_sync("RESPOND TYPE=command MSG=x").unwrap();

        assert_eq!(lines.emitted(), ["echo: hi", "// x"]);
    }

    /// The object is registered under the section id and is not
    /// client-visible, as upstream's `HostResponder` is not.
    #[test]
    fn test_the_object_is_registered_but_not_queryable() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text("[respond]\n").expect("the config parses");
        printer.load_config(&config).expect("the config loads");

        let object = printer
            .lookup_object_as::<HostResponder>("respond")
            .expect("the section registered the object");
        assert_eq!(object.get_status(0.0), json!({}));
        assert!(!object.is_queryable());
    }
}
