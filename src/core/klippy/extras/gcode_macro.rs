//! `[gcode_macro]` — G-Code macros, each one a command
//! (upstream `klippy/extras/gcode_macro.py`).
//!
//! A `[gcode_macro <name>]` section *is* a command: its uppercased name is
//! registered with the dispatcher under the section's `description`, the way
//! `GCodeMacro.__init__` does (`gcode_macro.py:124-147`). The bare
//! `[gcode_macro]` section is the shared template holder upstream creates for
//! `load_template` (`gcode_macro.py:81-115`); it reads no options of its own,
//! so its factory only claims the section.
//!
//! | option | default | role |
//! |---|---|---|
//! | `gcode` | — (required) | the macro body |
//! | `description` | `G-Code macro` | the command's help text |
//! | `rename_existing` | — | the command this one renames (`:135-142`) |
//! | `variable_<name>` | — | a literal reported in `get_status` (`:153-162`) |
//!
//! # How the body runs
//!
//! The section's `gcode` body is compiled into a [`Template`] when the macro
//! loads — the point upstream's `load_template` compiles it (`gcode_macro.py:
//! 132`) — and the registered command renders it per invocation: the macro's
//! `variable_*` values, the template context (`printer`, `action_*`,
//! `range`), then `params` and `rawparams` make up upstream's `kwparams`
//! (`:186-190`); the rendered text is fed back through
//! `run_script_from_command`, exactly `TemplateWrapper.run_gcode_from_command`
//! (`:79-80`). The supported template subset — and every construct refused
//! explicitly outside it — is listed in [`template`]'s module docs.
//!
//! `SET_GCODE_VARIABLE` is a mux command keyed by the section's own name, one
//! value per macro (`gcode_macro.py:148-150`). Its `VALUE` parses as JSON,
//! the literal rule this port's `variable_*` reader already applies
//! (`:158-162`), where upstream uses Python's `ast.literal_eval`.
//!
//! # Gaps this port does not close yet
//!
//! - **`rename_existing` stops at the load-time checks** (`:135-142`): the
//!   option is read and the same-type rule enforced, but upstream's swap at
//!   `klippy:connect` (`handle_connect`, `:163-171`) is not implemented, so a
//!   renaming macro does not register at all — matching upstream's *load-time*
//!   behaviour, minus the deferred half.

use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::template::{Builtin, Context, PrinterView, Rt, Template};
use crate::core::klippy::gcode::{
    is_traditional_gcode, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand,
    GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// `cmd_SET_GCODE_VARIABLE_help` (`gcode_macro.py:163`).
const SET_GCODE_VARIABLE_HELP: &str = "Set the value of a G-Code macro variable";

// Both `[gcode_macro]` (the shared template holder) and every
// `[gcode_macro <name>]` (a command) are valid.
section!(
    "gcode_macro",
    order = 30,
    load = load_config,
    prefix = load_config_prefix
);

/// The name the bare `[gcode_macro]` object registers under, and the name a
/// module that loads it by hand looks it up by (`load_object(config,
/// 'gcode_macro')`).
pub const GCODE_MACRO_OBJECT: &str = "gcode_macro";

/// The bare `[gcode_macro]` section: upstream's `PrinterGCodeMacro`, the
/// object every macro loads its `gcode` option through (`gcode_macro.py:81`).
///
/// It reads no options — the option check accepts a bare section that carries
/// none, and rejects one that carries any, as upstream's does.
pub struct PrinterGCodeMacro;

impl PrinterGCodeMacro {
    /// The single template holder; the first caller creates it, as upstream's
    /// `printer.load_object(config, 'gcode_macro')` does when no
    /// `[gcode_macro]` section exists.
    ///
    /// # Errors
    /// A duplicate registration (a name already taken).
    pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrinterGCodeMacro>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<PrinterGCodeMacro>(GCODE_MACRO_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(PrinterGCodeMacro);
        printer.add_object(
            GCODE_MACRO_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
    }

    /// Upstream's `PrinterGCodeMacro.load_template` (`gcode_macro.py:81-88`):
    /// read `option` from `config` (or its `default` when given) and compile it.
    ///
    /// A module that carries a g-code option of its own (the filament sensors'
    /// `runout_gcode` / `insert_gcode`) loads it this way instead of running a
    /// `[gcode_macro]`.
    ///
    /// # Errors
    /// A missing option when no `default` is given, or an unparsable template.
    pub fn load_template(
        &self,
        config: &ConfigWrapper,
        option: &str,
        default: Option<&str>,
    ) -> Result<Template, ConfigError> {
        let name = format!("{}:{}", config.identifier(), option);
        let script = config.get(option, default)?;
        Template::parse(&name, &script).map_err(|error| ConfigError::new(error.to_string()))
    }
}

impl PrinterObject for PrinterGCodeMacro {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// One `[gcode_macro <name>]`: the compiled body, the `variable_*` values its
/// `get_status` reports (`gcode_macro.py:172-173`), and the recursion flag
/// the command holds while it runs. The uppercased section name lives on the
/// state as well, so both registered commands share it.
#[derive(Debug)]
pub struct GCodeMacro {
    /// Shared with the `SET_GCODE_VARIABLE` handler and the macro's own
    /// command, which are registered before this object is returned.
    state: Arc<MacroState>,
}

/// What a macro *is*, shared by its command, its variable setter and its
/// `get_status` (`gcode_macro.py:124-173`).
#[derive(Debug)]
struct MacroState {
    /// The command's uppercased name (`gcode_macro.py:130`).
    alias: String,
    /// The compiled `gcode` body.
    template: Template,
    /// The `variable_*` values, keyed without the prefix
    /// (`gcode_macro.py:153-158`); `SET_GCODE_VARIABLE` writes them.
    variables: Mutex<BTreeMap<String, Value>>,
    /// Upstream's `in_script` flag (`gcode_macro.py:183`): set while this
    /// macro runs, so reaching itself is refused instead of recursing.
    in_script: AtomicBool,
}

impl MacroState {
    /// Claim the run, or report the recursion upstream reports
    /// (`gcode_macro.py:183-184`).
    fn enter(&self) -> Result<MacroGuard<'_>, CommandError> {
        if self.in_script.swap(true, Ordering::SeqCst) {
            return Err(CommandError::new(format!(
                "Macro {} called recursively",
                self.alias
            )));
        }
        Ok(MacroGuard(&self.in_script))
    }

    /// The render context upstream's `cmd` builds (`gcode_macro.py:186-190`):
    /// the macro's variables first, then the globals that override them, then
    /// `params` / `rawparams`.
    fn context(
        &self,
        printer: &Arc<Printer>,
        params: &HashMap<String, String>,
        rawparams: &str,
    ) -> Context {
        let mut context = Context::new();
        {
            let variables = self
                .variables
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            for (name, value) in variables.iter() {
                context.insert(name.clone(), Rt::Json(value.clone()));
            }
        }
        // `create_template_context` (`gcode_macro.py:101-108`): the `printer`
        // status view, the two actions the corpus' bodies call, and `range`
        // behind `{% for %}` (`exclude_object.cfg:92`).
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
        let map: serde_json::Map<String, Value> = params
            .iter()
            .map(|(key, value)| (key.clone(), json!(value)))
            .collect();
        context.insert("params", Rt::Json(Value::Object(map)));
        context.insert("rawparams", Rt::Json(Value::String(rawparams.to_string())));
        context
    }
}

/// A macro run in progress: clears `in_script` however the run ends, the way
/// upstream's `try/finally` does (`gcode_macro.py:186-190`).
struct MacroGuard<'a>(&'a AtomicBool);

impl Drop for MacroGuard<'_> {
    fn drop(&mut self) {
        self.0.store(false, Ordering::SeqCst);
    }
}

impl GCodeMacro {
    /// Read the section, enforce `rename_existing`'s load-time rules, compile
    /// the body, and register the macro as its command plus
    /// `SET_GCODE_VARIABLE` (`gcode_macro.py:124-162`).
    ///
    /// # Errors
    /// A section name with more than one name token, a missing `gcode` body, a
    /// template outside [`template`]'s subset, a `rename_existing` that names
    /// another command type, a `variable_*` value that is not a literal, or a
    /// command name that is taken — upstream's wordings, except the two
    /// literal errors, whose tail is this port's JSON parser rather than
    /// Python's `ast.literal_eval`.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let identifier = config.identifier();
        let tokens: Vec<&str> = identifier.split_whitespace().collect();
        // The prefix factory only ever sees `[gcode_macro <name>]` (the loader
        // sends the bare section to `load_config`), so two tokens is the
        // healthy form; more is upstream's illegal-whitespace error
        // (`gcode_macro.py:125-128`), and fewer cannot name a macro.
        if tokens.len() != 2 {
            return Err(ConfigError::new(format!(
                "Name of section '{identifier}' contains illegal whitespace"
            )));
        }
        let name = tokens[1];
        let alias = name.to_uppercase();

        // The body, compiled here the way `load_template` compiles it
        // upstream (`gcode_macro.py:132`), before the options below are read.
        let body = config.get("gcode", None)?;
        let template = Template::parse(&format!("{identifier}:gcode"), &body)
            .map_err(|error| ConfigError::new(error.to_string()))?;

        let rename_existing = config.get_str("rename_existing");
        let description = config.get("description", Some("G-Code macro"))?;

        if let Some(rename) = rename_existing.as_deref() {
            // Upstream refuses to swap commands of different types
            // (`gcode_macro.py:137-142`); the swap itself runs at
            // `klippy:connect` and is not implemented here (module docs).
            if is_traditional_gcode(&alias) != is_traditional_gcode(rename) {
                return Err(ConfigError::new(format!(
                    "G-Code macro rename of different types ('{alias}' vs '{rename}')"
                )));
            }
        }

        // `variable_*`: read every prefixed option, keep its literal
        // (`gcode_macro.py:153-162`).
        let mut variables = BTreeMap::new();
        for option in config.prefix_options("variable_") {
            let raw = config.get(&option, None)?;
            let value: Value = serde_json::from_str(&raw).map_err(|error| {
                ConfigError::new(format!(
                    "Option '{option}' in section '{identifier}' is not a valid literal: {error}"
                ))
            })?;
            let name = option["variable_".len()..].to_string();
            variables.insert(name, value);
        }

        let state = Arc::new(MacroState {
            alias: alias.clone(),
            template,
            variables: Mutex::new(variables),
            in_script: AtomicBool::new(false),
        });

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");

        if rename_existing.is_none() {
            // The macro is its command (`gcode_macro.py:143-147`): render the
            // body and hand the text back to the dispatcher
            // (`TemplateWrapper.run_gcode_from_command`, `:79-80`).
            let weak = Arc::downgrade(printer);
            let state = Arc::clone(&state);
            let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                let weak = weak.clone();
                let state = Arc::clone(&state);
                let params = gcmd.get_command_parameters().clone();
                let rawparams = gcmd.get_raw_command_parameters();
                Box::pin(async move {
                    let _guard = state.enter()?;
                    let printer = weak
                        .upgrade()
                        .ok_or_else(|| CommandError::new("printer is gone"))?;
                    let mut context = state.context(&printer, &params, &rawparams);
                    let script = state
                        .template
                        .render(&mut context)
                        .map_err(|error| CommandError::new(error.to_string()))?;
                    let gcode = printer
                        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                        .ok_or_else(|| CommandError::new("the gcode dispatcher is gone"))?;
                    gcode.run_script_from_command(&script).await
                })
            });
            gcode
                .register_command(&alias, handler, Some(&description), false)
                .map_err(ConfigError::new)?;
        }

        // `SET_GCODE_VARIABLE MACRO=<this section's name>` — a mux value per
        // macro, registered whether or not the macro renames (`:148-150`).
        let setter_state = Arc::clone(&state);
        let section_name = name.to_string();
        let setter: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let variable = gcmd.get_str("VARIABLE")?;
            let value = gcmd.get_str("VALUE")?;
            let mut variables = setter_state
                .variables
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            if !variables.contains_key(&variable) {
                return Err(CommandError::new(format!(
                    "Unknown gcode_macro variable '{variable}'"
                )));
            }
            let literal: Value = serde_json::from_str(&value).map_err(|error| {
                CommandError::new(format!(
                    "Unable to parse '{value}' as a literal: {error} in '{}'",
                    gcmd.commandline()
                ))
            })?;
            variables.insert(variable, literal);
            Ok(())
        });
        gcode
            .register_mux_command(
                "SET_GCODE_VARIABLE",
                "MACRO",
                Some(&section_name),
                setter,
                Some(SET_GCODE_VARIABLE_HELP),
            )
            .map_err(ConfigError::new)?;

        Ok(Arc::new(Self { state }))
    }

    /// The `variable_*` values (`gcode_macro.py:172-173`).
    fn variables_status(&self) -> Value {
        let variables = self
            .state
            .variables
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        Value::Object(
            variables
                .iter()
                .map(|(name, value)| (name.clone(), value.clone()))
                .collect(),
        )
    }
}

impl PrinterObject for GCodeMacro {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.variables_status()
    }
}

/// The factory `section!` names for the bare `[gcode_macro]`
/// (`gcode_macro.py:115 def load_config`).
pub fn load_config(
    _config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterGCodeMacro))
}

/// The factory `section!` names for each `[gcode_macro <name>]`
/// (`gcode_macro.py:202 def load_config_prefix`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = GCodeMacro::new(config, printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::reactor::ManualReactor;

    /// One `[gcode_macro <name>]` section with the given options, as the
    /// parser would build them (option names already lowercased).
    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("gcode_macro", Some(name));
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `gcode` registered, as the loader builds it.
    ///
    /// The dispatcher refuses scripts before ready (`gcode.rs` state check);
    /// `exclude_object.rs`'s machine lights the ready lamp the same way.
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

    /// Every option of every `macros.cfg` instance is read, and every macro
    /// name is registered as a command — the section's whole contract
    /// (`gcode_macro.py:124-162`), against the real corpus file.
    #[test]
    fn the_eight_macro_instances_read_every_option_and_register_their_commands() {
        let path = klipperx_test_support::klipper_dir().join("test/klippy/macros.cfg");
        let text = std::fs::read_to_string(&path).expect("macros.cfg is readable");
        let config = Config::from_text(&text).expect("macros.cfg parses").0;

        let access = AccessTracking::shared();
        let printer = printer();
        let mut loaded = 0;
        for sect in config.sections() {
            if sect.id != "gcode_macro" {
                continue;
            }
            let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));
            load_config_prefix(&wrapper, &printer)
                .unwrap_or_else(|err| panic!("{} loads: {err}", sect.identifier()));
            loaded += 1;

            // The section's contract with the option check: every option the
            // file gives it was read (`config/validate.rs:44-51`).
            for option in sect.parameters.keys() {
                assert!(
                    access.contains(&sect.identifier(), option),
                    "unread option '{}' in '{}'",
                    option,
                    sect.identifier()
                );
            }
        }
        assert_eq!(loaded, 8, "macros.cfg defines eight macros");

        // Macro-as-command: the uppercased names, each with its description.
        let help = gcode(&printer).command_help();
        for (name, description) in [
            ("TEST_SAVE_RESTORE", "G-Code macro"),
            ("TEST_EXPRESSION", "G-Code macro"),
            ("TEST_VARIABLE", "G-Code macro"),
            ("TEST_VARIABLE_PART2", "G-Code macro"),
            ("TEST_PARAM", "G-Code macro"),
            ("TEST_IN", "G-Code macro"),
            ("TEST_UNICODE", "A unicode test °"),
            ("TESTIT", "G-Code macro"),
        ] {
            assert_eq!(
                help.get(name).map(String::as_str),
                Some(description),
                "command {name}"
            );
        }
        // Nothing else registered under a macro-like name: `command_help`
        // also carries the dispatcher's built-ins, so count only `TEST*`.
        let mut macros: Vec<&str> = help
            .keys()
            .map(String::as_str)
            .filter(|name| name.starts_with("TEST"))
            .collect();
        macros.sort_unstable();
        let mut expected = [
            "TESTIT",
            "TEST_EXPRESSION",
            "TEST_IN",
            "TEST_PARAM",
            "TEST_SAVE_RESTORE",
            "TEST_UNICODE",
            "TEST_VARIABLE",
            "TEST_VARIABLE_PART2",
        ];
        expected.sort_unstable();
        assert_eq!(macros, expected, "exactly the eight macros register");
    }

    /// A macro reports its `variable_*` literals in `get_status`
    /// (`gcode_macro.py:172-173`), keyed without the prefix.
    #[test]
    fn a_macro_reports_its_variables_as_status() {
        let printer = printer();
        let sect = section(
            "TEST_variable",
            &[
                ("gcode", "{ action_respond_info(\"x\") }"),
                ("variable_t", "12.0"),
            ],
        );
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(&sect, Arc::clone(&access));
        let object = load_config_prefix(&config, &printer).expect("the macro loads");

        assert_eq!(object.get_status(0.0), json!({ "t": 12.0 }));
        assert!(access.contains("gcode_macro TEST_variable", "gcode"));
        assert!(access.contains("gcode_macro TEST_variable", "variable_t"));
        // The default description is recorded the way `config.get` records a
        // used default (`config/wrapper.rs:179-190`).
        assert!(access.contains("gcode_macro TEST_variable", "description"));
        // `rename_existing` is absent and reads record nothing when absent.
        assert!(!access.contains("gcode_macro TEST_variable", "rename_existing"));
        assert_eq!(
            gcode(&printer).command_help().get("TEST_VARIABLE"),
            Some(&"G-Code macro".to_string())
        );
    }

    /// A section name with more than one name token is refused with upstream's
    /// wording (`gcode_macro.py:125-128`), a missing body with the loader's
    /// (`config/wrapper.rs:483-487`), and a non-literal variable with
    /// upstream's (`gcode_macro.py:159-162`).
    #[test]
    fn a_malformed_section_is_refused_with_upstream_wording() {
        let printer = printer();

        let sect = section("A B", &[("gcode", "G28")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            GCodeMacro::new(&config, &printer).unwrap_err().to_string(),
            "Name of section 'gcode_macro A B' contains illegal whitespace"
        );

        let sect = section("TESTIT", &[("description", "x")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            GCodeMacro::new(&config, &printer).unwrap_err().to_string(),
            "Option 'gcode' in section 'gcode_macro TESTIT' must be specified"
        );

        let sect = section("TEST_bad", &[("gcode", "G28"), ("variable_x", "[1, two]")]);
        let config = ConfigWrapper::untracked(&sect);
        let err = GCodeMacro::new(&config, &printer).unwrap_err().to_string();
        assert!(
            err.starts_with(
                "Option 'variable_x' in section 'gcode_macro TEST_bad' \
                 is not a valid literal: "
            ),
            "{err}"
        );
    }

    /// `rename_existing` may only rename the same command type
    /// (`gcode_macro.py:137-142`); a same-type rename loads without
    /// registering, because upstream's registration then happens at
    /// `klippy:connect` (`handle_connect`, `:163-171`) — which this port does
    /// not implement yet (module docs).
    #[test]
    fn rename_existing_is_type_checked_and_defers_registration() {
        let printer = printer();

        let sect = section(
            "my_macro",
            &[("gcode", "M117 hello"), ("rename_existing", "G28")],
        );
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            GCodeMacro::new(&config, &printer).unwrap_err().to_string(),
            "G-Code macro rename of different types ('MY_MACRO' vs 'G28')"
        );
        assert!(gcode(&printer).command_help().get("MY_MACRO").is_none());

        // Both traditional: the type check passes, and — as at load time
        // upstream — neither name is registered yet.
        let sect = section("G29", &[("gcode", "G28"), ("rename_existing", "G28")]);
        let config = ConfigWrapper::untracked(&sect);
        GCodeMacro::new(&config, &printer).expect("a same-type rename loads");
        let help = gcode(&printer).command_help();
        assert!(help.get("G29").is_none(), "registration waits for connect");
        assert!(help.get("G28").is_none(), "the builtin keeps its help");
    }

    /// The bare `[gcode_macro]` section is claimed and reads nothing
    /// (`gcode_macro.py:81-115`).
    #[test]
    fn the_bare_section_is_claimed_and_carries_no_status() {
        let printer = printer();
        let sect = ConfigSection::new("gcode_macro", None);
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(&sect, Arc::clone(&access));

        let object = load_config(&config, &printer).expect("the section loads");
        assert_eq!(object.get_status(0.0), json!({}));
        assert!(access.sections().is_empty(), "no option to read");
    }

    /// The seam upstream calls `run_gcode_from_command`: a macro renders its
    /// body against `params` / `rawparams` / its own `variable_*` status and
    /// the rendered lines reach the dispatcher (`gcode_macro.py:186-190` +
    /// `:79-80`). The receiving command is a fake this test registers.
    #[test]
    fn a_macro_body_renders_and_dispatches_its_lines() {
        let printer = printer();
        let dispatch = gcode(&printer);
        let seen = Arc::new(Mutex::new(Vec::new()));
        {
            let seen = Arc::clone(&seen);
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                seen.lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(gcmd.get_str("VALUE").unwrap_or_default());
                Ok(())
            });
            dispatch
                .register_command("ECHO_LINE", handler, None, false)
                .expect("the fake receiver registers");
        }

        let sect = section(
            "probe",
            &[
                (
                    "gcode",
                    "ECHO_LINE VALUE={params.N}\nECHO_LINE VALUE={rawparams}\n\
                     ECHO_LINE VALUE={t}\nECHO_LINE VALUE={printer[\"gcode_macro probe\"].t}",
                ),
                ("variable_t", "12.0"),
            ],
        );
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(&sect, Arc::clone(&access));
        let object = load_config_prefix(&config, &printer).expect("the macro loads");
        // The loader registers the object under its section name, which is
        // what `printer["…"]` then reads (`load.rs`).
        printer
            .add_object(&sect.identifier(), object)
            .expect("the object registers");

        dispatch
            .run_script_sync("PROBE N=7")
            .expect("the run is clean");
        assert_eq!(
            *seen.lock().unwrap_or_else(|poison| poison.into_inner()),
            vec!["7", "N=7", "12.0", "12.0"],
            "params, rawparams, the bare variable and its get_status read back"
        );
    }

    /// Upstream's `in_script` guard (`gcode_macro.py:183-184`): a macro whose
    /// body reaches itself is refused instead of recursing, and the refusal
    /// leaves the macro usable — the flag clears with the run.
    #[test]
    fn a_macro_that_calls_itself_is_refused() {
        let printer = printer();
        let sect = section("loop_", &[("gcode", "LOOP_")]);
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(&sect, Arc::clone(&access));
        load_config_prefix(&config, &printer).expect("the macro loads");

        let dispatch = gcode(&printer);
        let error = dispatch
            .run_script_sync("LOOP_")
            .expect_err("the recursion is refused");
        assert!(
            error.to_string().contains("Macro LOOP_ called recursively"),
            "{error}"
        );

        // The flag was cleared on the way out: a macro that stops recursing
        // runs its body to the end.
        let sect = section("oncemap", &[("gcode", "ECHO_")]);
        let config = ConfigWrapper::untracked(&sect);
        load_config_prefix(&config, &printer).expect("the macro loads");
        let seen = Arc::new(Mutex::new(0usize));
        {
            let seen = Arc::clone(&seen);
            let handler: CommandHandler = sync(move |_| {
                *seen.lock().unwrap_or_else(|poison| poison.into_inner()) += 1;
                Ok(())
            });
            dispatch
                .register_command("ECHO_", handler, None, false)
                .expect("the fake receiver registers");
        }
        dispatch.run_script_sync("ONCEMAP").expect("a clean run");
        dispatch.run_script_sync("ONCEMAP").expect("still clean");
        assert_eq!(
            *seen.lock().unwrap_or_else(|poison| poison.into_inner()),
            2,
            "both runs reached the receiver"
        );
    }

    /// `SET_GCODE_VARIABLE` is mux-keyed by the section's name and writes the
    /// value `get_status` then reports (`gcode_macro.py:148-150, :164-173`);
    /// an unknown variable and a non-literal value are refused with upstream's
    /// wording (the parse tail is this port's JSON parser).
    #[test]
    fn set_gcode_variable_writes_the_macro_status() {
        let printer = printer();
        let sect = section(
            "TEST_variable",
            &[
                ("gcode", "{ action_respond_info(\"x\") }"),
                ("variable_t", "12.0"),
            ],
        );
        let access = AccessTracking::shared();
        let config = ConfigWrapper::new(&sect, Arc::clone(&access));
        let object = load_config_prefix(&config, &printer).expect("the macro loads");

        let dispatch = gcode(&printer);
        assert_eq!(
            dispatch.command_help().get("SET_GCODE_VARIABLE"),
            Some(&"Set the value of a G-Code macro variable".to_string())
        );

        dispatch
            .run_script_sync("SET_GCODE_VARIABLE MACRO=TEST_variable VARIABLE=t VALUE=17")
            .expect("the write succeeds");
        assert_eq!(object.get_status(0.0), json!({ "t": 17 }));

        let error = dispatch
            .run_script_sync("SET_GCODE_VARIABLE MACRO=TEST_variable VARIABLE=nope VALUE=1")
            .expect_err("unknown variable");
        assert!(
            error
                .to_string()
                .contains("Unknown gcode_macro variable 'nope'"),
            "{error}"
        );

        let error = dispatch
            .run_script_sync("SET_GCODE_VARIABLE MACRO=TEST_variable VARIABLE=t VALUE=oops")
            .expect_err("not a literal");
        assert!(
            error
                .to_string()
                .contains("Unable to parse 'oops' as a literal: "),
            "{error}"
        );
    }

    /// A body outside [`template`]'s subset is refused when the section
    /// loads, with upstream's `Error loading template` frame
    /// (`gcode_macro.py:61-66`).
    #[test]
    fn an_unsupported_template_construct_is_a_load_error() {
        let printer = printer();
        let sect = section("SETTY", &[("gcode", "{% set x = 1 %}")]);
        let config = ConfigWrapper::untracked(&sect);
        let error = GCodeMacro::new(&config, &printer)
            .expect_err("the subset does not carry `set`")
            .to_string();
        assert!(
            error.starts_with(
                "Error loading template 'gcode_macro SETTY:gcode'\nline 1: \
                 unsupported statement 'set'"
            ),
            "{error}"
        );
    }
}
