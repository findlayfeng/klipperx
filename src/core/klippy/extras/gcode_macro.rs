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
//! # Gaps this port does not close yet
//!
//! - **The body is not rendered.** Running the command answers with a
//!   `respond_info` line and returns: expanding the template
//!   ([`TemplateWrapper`-style, `gcode_macro.py:46`]) and evaluating its
//!   expressions against `printer` / `params` (`GetStatusWrapper`, `:15`) is a
//!   later unit. Upstream's regression needs only the section to load and the
//!   command to exist — an invoked macro must simply not fail the run.
//! - **`SET_GCODE_VARIABLE` is not registered** (`gcode_macro.py:148-150`).
//!   The `variable_*` options are read (so the option check accepts them) and
//!   reported by `get_status`, but nothing writes them yet.
//! - **`rename_existing` stops at the load-time checks** (`:135-142`): the
//!   option is read and the same-type rule enforced, but upstream's swap at
//!   `klippy:connect` (`handle_connect`, `:163-171`) is not implemented, so a
//!   renaming macro does not register at all — matching upstream's *load-time*
//!   behaviour, minus the deferred half.

use std::collections::BTreeMap;
use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    is_traditional_gcode, CommandHandler, GCodeDispatch, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Both `[gcode_macro]` (the shared template holder) and every
// `[gcode_macro <name>]` (a command) are valid.
section!(
    "gcode_macro",
    order = 30,
    load = load_config,
    prefix = load_config_prefix
);

/// The bare `[gcode_macro]` section: upstream's `PrinterGCodeMacro`, the
/// object every macro loads its `gcode` option through (`gcode_macro.py:81`).
///
/// It reads no options — the option check accepts a bare section that carries
/// none, and rejects one that carries any, as upstream's does.
pub struct PrinterGCodeMacro;

impl PrinterObject for PrinterGCodeMacro {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// One `[gcode_macro <name>]`: the `variable_*` values its `get_status`
/// reports (`gcode_macro.py:172-173`). The uppercased section name lives on
/// the registered command, not here — the dispatcher already holds it.
#[derive(Debug)]
pub struct GCodeMacro {
    /// The `variable_*` options, keyed without the prefix and lowercased like
    /// the parser's own option names (`gcode_macro.py:153-158`).
    variables: BTreeMap<String, Value>,
}

impl GCodeMacro {
    /// Read the section, enforce `rename_existing`'s load-time rules, and
    /// register the macro as its command (`gcode_macro.py:124-162`).
    ///
    /// # Errors
    /// A section name with more than one name token, a missing `gcode` body, a
    /// `rename_existing` that names another command type, a `variable_*` value
    /// that is not a literal, or a command name that is taken — upstream's
    /// wordings, except the literal error, whose tail is this port's JSON
    /// parser rather than Python's `ast.literal_eval`.
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

        // The body: upstream compiles it into a template here
        // (`load_template(config, 'gcode')`, `gcode_macro.py:132`); this port
        // reads it for the option check and does not render it (see the module
        // docs).
        config.get("gcode", None)?;
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
        } else {
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode` first");
            let weak = Arc::downgrade(printer);
            let handler: CommandHandler = Arc::new({
                let alias = alias.clone();
                move |_gcmd| {
                    let weak = weak.clone();
                    let alias = alias.clone();
                    Box::pin(async move {
                        // The body is not rendered yet (module docs): say so
                        // where users look instead of silently doing nothing.
                        if let Some(printer) = weak.upgrade() {
                            if let Some(gcode) =
                                printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                            {
                                gcode.respond_info(
                                    &format!(
                                        "gcode_macro {alias}: the macro body was not run; \
                                         template rendering is not implemented"
                                    ),
                                    true,
                                );
                            }
                        }
                        Ok(())
                    })
                }
            });
            gcode
                .register_command(&alias, handler, Some(&description), false)
                .map_err(ConfigError::new)?;
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

        Ok(Arc::new(Self { variables }))
    }

    /// The `variable_*` values (`gcode_macro.py:172-173`).
    fn variables_status(&self) -> Value {
        Value::Object(
            self.variables
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
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode registers");
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
}
