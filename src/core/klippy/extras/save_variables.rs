//! `[save_variables]` — variables that survive a restart
//! (upstream `klippy/extras/save_variables.py`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `filename` | — (required) | the file the variables live in; a leading `~` is expanded |
//!
//! `SAVE_VARIABLE VARIABLE=<name> VALUE=<literal>` adds or replaces one variable
//! (`save_variables.py:15-21`). The **whole** set is then rewritten to the file,
//! sorted by name and one `name = <literal>` line each, and read back, so
//! `get_status` always reports exactly what the file holds. The name must be
//! lowercase — an upper-case letter is refused before anything is written.
//!
//! `get_status` is `{"variables": {...}}` (`save_variables.py:24-25`), which is
//! what a macro or a template reads as
//! `printer.save_variables.variables.<name>`; the object is registered under the
//! section id, so no extra wiring is needed.
//!
//! # The file
//!
//! The file is a `configparser` file with a single `[Variables]` section
//! (`save_variables.py:9-20`), written with the section header and one
//! `name = <literal>` line per variable, sorted by name. It is read back with
//! this port's own reader of that format: `#`/`;` lines are comments, `=` and
//! `:` both separate a name from its value, a name is lowercased, an indented
//! line continues the previous value with a newline, and only the `[Variables]`
//! section is looked at. Unlike the main config parser this reader keeps a `#`
//! or `;` **inside** a value (upstream's `configparser` does not strip inline
//! comments either), so a saved string survives a round trip.
//!
//! A missing file is created empty; anything that cannot be read or parsed —
//! including a value that is not a literal — is reported as
//! `Unable to parse existing variable file` (`save_variables.py:16-22`).
//!
//! # The literal dialect
//!
//! Upstream writes each value with Python's `repr` and reads it with
//! `ast.literal_eval` (`save_variables.py:13,20,29`). This port has neither, so
//! it spells the same subset itself:
//!
//! | value | written | read |
//! |---|---|---|
//! | `None` | `None` | `None` |
//! | bool | `True` / `False` | `True` / `False` |
//! | int | `1`, `-2` | decimal, optional sign |
//! | float | `2.5`, `1e20` | decimal with `.` and/or an exponent |
//! | str | `'text'` | `'text'` or `"text"`, with `\\ \' \" \n \r \t \xNN` escapes |
//! | list | `[1, 'x']` | the same, `, `-separated |
//! | dict | `{'a': 1}` | the same, **string keys only** |
//!
//! Tuples, sets, bytes, complex numbers and non-string dict keys are outside the
//! subset and are refused as an unparseable literal. A string is always written
//! with single quotes (Python's `repr` switches to double quotes for a string
//! that holds a single quote and no double quote) and only ASCII control
//! characters are escaped; both read back identically.
//!
//! # Known deviations from upstream
//!
//! * **No interpolation on read.** Upstream's `configparser` interpolates by
//!   default, so a stored value holding a bare `%` makes the load fail with
//!   `Unable to parse existing variable file`; this reader and writer leave
//!   `%` alone (`save_variables.py:16-22`).
//! * **File I/O is synchronous.** Upstream hands reads and writes to
//!   `aio_executor.allocate_executor("save_variables")`; this port has no such
//!   executor, so the command thread does the I/O itself.
//! * **`get_status` is sorted by name** (`BTreeMap`), where upstream reports the
//!   file's insertion order.
//! * Integers outside `i64`, and the tuple/set/bytes/complex literals above,
//!   are refused rather than round-tripped.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use serde_json::{json, Map, Number, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{sync, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("save_variables", order = 30, load = load_config);

/// The help text upstream registers `SAVE_VARIABLE` with
/// (`save_variables.py:19`).
const SAVE_VARIABLE_HELP: &str = "Save arbitrary variables to disk";

/// The one message every read/parse failure is reported with
/// (`save_variables.py:21`).
const PARSE_ERROR: &str = "Unable to parse existing variable file";

/// The `[save_variables]` module object (upstream's `SaveVariables`).
///
/// The filename and the variable set are shared with the command handler, which
/// lives on the dispatcher rather than on this object, so the object holds them
/// behind an [`Arc`]; the set is a `Mutex` because `get_status` runs on any
/// thread while a command rewrites it.
struct Shared {
    /// The `filename` option, with `~` expanded (`save_variables.py:9`).
    filename: PathBuf,
    /// `allVariables` (`save_variables.py:10`) — the file's contents.
    variables: Mutex<BTreeMap<String, Value>>,
}

/// The section object the loader registers under `save_variables`.
pub struct SaveVariables {
    shared: Arc<Shared>,
}

impl PrinterObject for SaveVariables {
    /// `{"variables": {...}}` (`save_variables.py:24-25`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let variables = self
            .shared
            .variables
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let map: Map<String, Value> = variables
            .iter()
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();
        json!({ "variables": map })
    }
}

/// Read `filename`, create it when missing, load it, then register
/// `SAVE_VARIABLE` (`save_variables.py:6-19`).
///
/// # Errors
/// A config error when `filename` is absent (`config.get`'s wording), when the
/// file cannot be created, when it cannot be parsed, or when the dispatcher
/// rejects the registration.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let identifier = config.identifier();
    let filename = expanduser(&config.get("filename", None)?);
    // Upstream creates the file when it is missing; a failure there is not a
    // command error it catches, so it surfaces as a config error here.
    if !filename.exists() {
        fs::write(&filename, "").map_err(|error| {
            ConfigError::new(format!(
                "{identifier}: unable to create variable file '{}': {error}",
                filename.display()
            ))
        })?;
    }
    let shared = Arc::new(Shared {
        filename,
        variables: Mutex::new(BTreeMap::new()),
    });
    load_variables(&shared).map_err(ConfigError::new)?;

    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` before any section");
    let handler_shared = Arc::clone(&shared);
    let handler = sync(move |gcmd| cmd_save_variable(&handler_shared, gcmd));
    // Upstream registers with the default `when_not_ready` (`False`). The two
    // names the handler reads are declared for client completion, which is
    // additive (`register_command_with_params`, `gcode.rs`).
    gcode
        .register_command_with_params(
            "SAVE_VARIABLE",
            handler,
            Some(SAVE_VARIABLE_HELP),
            &["VARIABLE", "VALUE"],
            false,
        )
        .map_err(|error| ConfigError::new(format!("{identifier}: {error}")))?;

    Ok(Arc::new(SaveVariables { shared }))
}

/// `SAVE_VARIABLE VARIABLE=<name> VALUE=<literal>` (`save_variables.py:15-38`).
fn cmd_save_variable(shared: &Shared, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let varname = gcmd.get_str("VARIABLE")?;
    if varname.to_lowercase() != varname {
        return Err(CommandError::new("VARIABLE must not contain upper case"));
    }
    let raw = gcmd.get_str("VALUE")?;
    let value = parse_literal(&raw)
        .ok_or_else(|| CommandError::new(format!("Unable to parse '{raw}' as a literal")))?;

    // Merge into the current set and write the whole file (upstream writes
    // `newvars`, a copy of `allVariables` plus this one, sorted).
    let newvars = {
        let variables = shared
            .variables
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mut newvars = variables.clone();
        newvars.insert(varname, value);
        newvars
    };
    write_variables(&shared.filename, &newvars)
        .map_err(|_| CommandError::new("Unable to save variable"))?;
    // Re-read so the in-memory set is exactly what the file holds
    // (`self.loadVariables()`, `save_variables.py:37`).
    load_variables(shared).map_err(CommandError::new)?;
    Ok(())
}

/// Load every variable `shared.filename` holds, replacing the in-memory set.
///
/// The `Err` is always [`PARSE_ERROR`] (`save_variables.py:16-22`).
fn load_variables(shared: &Shared) -> Result<(), String> {
    let text = fs::read_to_string(&shared.filename).map_err(|_| PARSE_ERROR.to_string())?;
    let entries = read_variables(&text).map_err(|()| PARSE_ERROR.to_string())?;
    let mut variables = BTreeMap::new();
    for (name, raw) in entries {
        let value = parse_literal(&raw).ok_or_else(|| PARSE_ERROR.to_string())?;
        variables.insert(name, value);
    }
    *shared
        .variables
        .lock()
        .unwrap_or_else(|poison| poison.into_inner()) = variables;
    Ok(())
}

/// Write the whole set to `filename`, one `name = <literal>` line per variable,
/// sorted by name.
fn write_variables(filename: &Path, variables: &BTreeMap<String, Value>) -> Result<(), ()> {
    let mut body = String::from("[Variables]\n");
    for (name, value) in variables {
        body.push_str(name);
        body.push_str(" = ");
        body.push_str(&format_literal(value));
        body.push('\n');
    }
    fs::write(filename, body).map_err(|_| ())
}

/// Read the `[Variables]` section's `(name, value)` pairs, in file order.
///
/// The `configparser` subset `save_variables.py` relies on: `#`/`;` lines are
/// comments, `[name]` opens a section, `name = value` / `name: value` is an
/// option with its name lowercased, and an indented line continues the previous
/// value with a newline. Comments are **only** full-line — a `#` inside a value
/// stays there, as it does in upstream's `configparser`. `Err(())` is a line
/// that cannot be read (a non-section line before any header, a header without
/// `]`, or an empty name).
fn read_variables(text: &str) -> Result<Vec<(String, String)>, ()> {
    // The open section, once a header has been seen; a line before any header
    // is `configparser`'s `MissingSectionHeaderError`.
    let mut section: Option<String> = None;
    let mut options: Vec<(String, String)> = Vec::new();
    let mut option_indent = 0usize;
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') || trimmed.starts_with(';') {
            continue;
        }
        let indent = line.len() - line.trim_start().len();
        if trimmed.starts_with('[') {
            // `SECTCRE` matches up to the last `]` and ignores what follows.
            let end = trimmed.rfind(']').ok_or(())?;
            section = Some(trimmed[1..end].trim().to_string());
            option_indent = 0;
            continue;
        }
        match section.as_deref() {
            // Another section's options are not ours to read.
            Some(name) if name != "Variables" => continue,
            Some(_) => {}
            None => return Err(()),
        }
        if !options.is_empty() && indent > option_indent {
            let (_, value) = options.last_mut().expect("non-empty");
            value.push('\n');
            value.push_str(trimmed);
            continue;
        }
        let (name, value) = match trimmed.find(['=', ':']) {
            Some(separator) => (trimmed[..separator].trim(), trimmed[separator + 1..].trim()),
            None => (trimmed, ""),
        };
        if name.is_empty() {
            return Err(());
        }
        options.push((name.to_lowercase(), value.to_string()));
        option_indent = indent;
    }
    Ok(options)
}

/// `os.path.expanduser` for the two spellings that do not need `pwd`
/// (`save_variables.py:9`): a leading `~` or `~/` becomes `$HOME`. `~user` is
/// left as written.
fn expanduser(path: &str) -> PathBuf {
    let Some(rest) = path.strip_prefix('~') else {
        return PathBuf::from(path);
    };
    if !(rest.is_empty() || rest.starts_with('/')) {
        return PathBuf::from(path);
    }
    let Some(home) = std::env::var_os("HOME").filter(|value| !value.is_empty()) else {
        return PathBuf::from(path);
    };
    let mut expanded = PathBuf::from(home);
    if !rest.is_empty() {
        expanded.push(rest.trim_start_matches('/'));
    }
    expanded
}

/// One value in upstream's `repr` spelling, over the subset of the module docs.
fn format_literal(value: &Value) -> String {
    match value {
        Value::Null => "None".to_string(),
        Value::Bool(true) => "True".to_string(),
        Value::Bool(false) => "False".to_string(),
        Value::Number(number) => format_number(number),
        Value::String(text) => format_string(text),
        Value::Array(items) => {
            let parts: Vec<String> = items.iter().map(format_literal).collect();
            format!("[{}]", parts.join(", "))
        }
        Value::Object(map) => {
            let parts: Vec<String> = map
                .iter()
                .map(|(key, item)| format!("{}: {}", format_string(key), format_literal(item)))
                .collect();
            format!("{{{}}}", parts.join(", "))
        }
    }
}

/// A number the way `repr` shows it: an integer as written, a float with its
/// decimal point or exponent kept (`2.0`, not `2`, so it reads back a float).
fn format_number(number: &Number) -> String {
    if let Some(value) = number.as_i64() {
        value.to_string()
    } else if let Some(value) = number.as_u64() {
        value.to_string()
    } else if let Some(value) = number.as_f64() {
        format!("{value:?}")
    } else {
        number.to_string()
    }
}

/// A single-quoted string with the escapes [`parse_literal`] reads back.
fn format_string(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('\'');
    for c in text.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 || c == '\u{7f}' => {
                out.push_str(&format!("\\x{:02x}", c as u32));
            }
            c => out.push(c),
        }
    }
    out.push('\'');
    out
}

/// Parse one value the way upstream's `ast.literal_eval` accepts it, over the
/// subset of the module docs. Whitespace around the value is allowed; anything
/// else — a bare name, a partial literal, trailing text — is `None`.
fn parse_literal(text: &str) -> Option<Value> {
    let mut parser = LiteralParser::new(text);
    parser.skip_ws();
    let value = parser.parse_value()?;
    parser.skip_ws();
    if parser.at_end() {
        Some(value)
    } else {
        None
    }
}

/// A recursive-descent reader for the literal subset of the module docs.
struct LiteralParser {
    chars: Vec<char>,
    pos: usize,
}

impl LiteralParser {
    fn new(text: &str) -> Self {
        Self {
            chars: text.chars().collect(),
            pos: 0,
        }
    }

    fn at_end(&self) -> bool {
        self.pos >= self.chars.len()
    }

    fn peek(&self) -> Option<char> {
        self.chars.get(self.pos).copied()
    }

    fn bump(&mut self) -> Option<char> {
        let c = self.peek();
        if c.is_some() {
            self.pos += 1;
        }
        c
    }

    fn skip_ws(&mut self) {
        while self.peek().is_some_and(char::is_whitespace) {
            self.pos += 1;
        }
    }

    fn parse_value(&mut self) -> Option<Value> {
        self.skip_ws();
        let value = match self.peek()? {
            '\'' | '"' => {
                let quote = self.bump().expect("peeked");
                Value::String(self.parse_string(quote)?)
            }
            '[' => self.parse_list()?,
            '{' => self.parse_dict()?,
            'T' | 'F' | 'N' => self.parse_name()?,
            c if c == '-' || c == '+' || c.is_ascii_digit() => self.parse_number()?,
            _ => return None,
        };
        self.skip_ws();
        Some(value)
    }

    /// `True`, `False` or `None` — an identifier of any other spelling is
    /// rejected, as `ast.literal_eval` rejects a name (including JSON's
    /// lowercase `true`/`false`/`null`).
    fn parse_name(&mut self) -> Option<Value> {
        let start = self.pos;
        while self.peek().is_some_and(|c| c.is_alphabetic() || c == '_') {
            self.pos += 1;
        }
        let name: String = self.chars[start..self.pos].iter().collect();
        match name.as_str() {
            "True" => Some(Value::Bool(true)),
            "False" => Some(Value::Bool(false)),
            "None" => Some(Value::Null),
            _ => None,
        }
    }

    fn parse_number(&mut self) -> Option<Value> {
        let start = self.pos;
        if matches!(self.peek(), Some('-') | Some('+')) {
            self.pos += 1;
        }
        let mut digits = 0usize;
        while self.peek().is_some_and(|c| c.is_ascii_digit()) {
            self.pos += 1;
            digits += 1;
        }
        let mut is_float = false;
        if self.peek() == Some('.') {
            is_float = true;
            self.pos += 1;
            while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                self.pos += 1;
                digits += 1;
            }
        }
        if matches!(self.peek(), Some('e') | Some('E')) {
            let save = self.pos;
            self.pos += 1;
            if matches!(self.peek(), Some('-') | Some('+')) {
                self.pos += 1;
            }
            let exponent_start = self.pos;
            while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                self.pos += 1;
            }
            if self.pos > exponent_start {
                is_float = true;
            } else {
                self.pos = save;
            }
        }
        // A bare sign, or a `.` with no digit on either side, is not a number.
        if digits == 0 {
            return None;
        }
        let text: String = self.chars[start..self.pos].iter().collect();
        if is_float {
            text.parse::<f64>().ok().map(|value| json!(value))
        } else {
            text.parse::<i64>().ok().map(|value| json!(value))
        }
    }

    fn parse_string(&mut self, quote: char) -> Option<String> {
        let mut out = String::new();
        loop {
            match self.bump()? {
                c if c == quote => return Some(out),
                '\\' => out.push(self.parse_escape()?),
                c => out.push(c),
            }
        }
    }

    fn parse_escape(&mut self) -> Option<char> {
        match self.bump()? {
            '\\' => Some('\\'),
            '\'' => Some('\''),
            '"' => Some('"'),
            'n' => Some('\n'),
            'r' => Some('\r'),
            't' => Some('\t'),
            '0' => Some('\0'),
            'x' => self.parse_hex(2),
            'u' => self.parse_hex(4),
            'U' => self.parse_hex(8),
            // A spelling `repr` never emits: keep the character, as Python's
            // literal reader would with a deprecation warning.
            other => Some(other),
        }
    }

    fn parse_hex(&mut self, digits: usize) -> Option<char> {
        let mut value = 0u32;
        for _ in 0..digits {
            value = value * 16 + self.bump()?.to_digit(16)?;
        }
        char::from_u32(value)
    }

    fn parse_list(&mut self) -> Option<Value> {
        self.bump(); // '['
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek() == Some(']') {
            self.bump();
            return Some(Value::Array(items));
        }
        loop {
            items.push(self.parse_value()?);
            self.skip_ws();
            match self.bump()? {
                ',' => {
                    self.skip_ws();
                    if self.peek() == Some(']') {
                        self.bump();
                        break;
                    }
                }
                ']' => break,
                _ => return None,
            }
        }
        Some(Value::Array(items))
    }

    fn parse_dict(&mut self) -> Option<Value> {
        self.bump(); // '{'
        let mut map = Map::new();
        self.skip_ws();
        if self.peek() == Some('}') {
            self.bump();
            return Some(Value::Object(map));
        }
        loop {
            self.skip_ws();
            // Only string keys are representable in a JSON object; a key of any
            // other type is outside the subset.
            let key = match self.peek()? {
                '\'' | '"' => {
                    let quote = self.bump().expect("peeked");
                    self.parse_string(quote)?
                }
                _ => return None,
            };
            self.skip_ws();
            if self.bump()? != ':' {
                return None;
            }
            let value = self.parse_value()?;
            map.insert(key, value);
            self.skip_ws();
            match self.bump()? {
                ',' => {
                    self.skip_ws();
                    if self.peek() == Some('}') {
                        self.bump();
                        break;
                    }
                }
                '}' => break,
                _ => return None,
            }
        }
        Some(Value::Object(map))
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::template::{Context, PrinterView, Rt, Template};
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// A temporary directory that removes itself on drop.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "klipperx-savevars-{}-{}-{}",
                std::process::id(),
                name,
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&path).expect("cannot create the test directory");
            Self(path)
        }

        fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A ready printer with `[save_variables]` pointing at `filename`.
    fn machine(filename: &Path) -> (Arc<Printer>, Arc<GCodeDispatch>) {
        let text = format!("[save_variables]\nfilename: {}\n", filename.display());
        let (config, _) = Config::from_text(&text).expect("the section parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        (printer, gcode)
    }

    /// `filename` is required (`config.get`'s wording).
    #[test]
    fn test_the_filename_option_is_required() {
        let (config, _) = Config::from_text("[save_variables]\n").expect("the section parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        let error = printer
            .load_config(&config)
            .expect_err("the section is refused");

        assert_eq!(
            error.to_string(),
            "Option 'filename' in section 'save_variables' must be specified"
        );
    }

    /// A missing file is created empty and loads as no variables.
    #[test]
    fn test_a_missing_file_is_created_empty() {
        let dir = TempDir::new("create");
        let file = dir.path().join("variables.cfg");
        assert!(!file.exists());

        let (printer, _gcode) = machine(&file);

        assert!(file.exists(), "the file is created");
        assert_eq!(std::fs::read_to_string(&file).unwrap(), "");
        assert_eq!(
            printer.status_of("save_variables", 0.0),
            Some(json!({ "variables": {} }))
        );
    }

    /// A saved value reaches the file (sorted, `repr`-spelled) and comes back
    /// through `get_status`, which reads the reloaded file.
    #[test]
    fn test_a_saved_value_round_trips_through_the_file() {
        let dir = TempDir::new("roundtrip");
        let file = dir.path().join("variables.cfg");
        let (printer, gcode) = machine(&file);

        gcode
            .run_script_sync("SAVE_VARIABLE VARIABLE=counter VALUE=7")
            .expect("the save succeeds");

        assert_eq!(
            std::fs::read_to_string(&file).unwrap(),
            "[Variables]\ncounter = 7\n"
        );
        assert_eq!(
            printer.status_of("save_variables", 0.0),
            Some(json!({ "variables": { "counter": 7 } }))
        );
    }

    /// Saving a new variable keeps the ones the file already held, and the
    /// file is rewritten sorted by name.
    #[test]
    fn test_a_save_keeps_the_variables_already_in_the_file() {
        let dir = TempDir::new("merge");
        let file = dir.path().join("variables.cfg");
        std::fs::write(&file, "[Variables]\na = 1\nb = 'two'\n").unwrap();
        let (printer, gcode) = machine(&file);
        assert_eq!(
            printer.status_of("save_variables", 0.0),
            Some(json!({ "variables": { "a": 1, "b": "two" } }))
        );

        gcode
            .run_script_sync("SAVE_VARIABLE VARIABLE=c VALUE=3.0")
            .expect("the save succeeds");

        assert_eq!(
            std::fs::read_to_string(&file).unwrap(),
            "[Variables]\na = 1\nb = 'two'\nc = 3.0\n"
        );
        assert_eq!(
            printer.status_of("save_variables", 0.0),
            Some(json!({ "variables": { "a": 1, "b": "two", "c": 3.0 } }))
        );
    }

    /// An upper-case variable name is refused before anything is written.
    #[test]
    fn test_an_uppercase_variable_is_refused() {
        let dir = TempDir::new("uppercase");
        let file = dir.path().join("variables.cfg");
        let (_printer, gcode) = machine(&file);

        let error = gcode
            .run_script_sync("SAVE_VARIABLE VARIABLE=Camel VALUE=1")
            .expect_err("the name is refused");

        assert_eq!(error.to_string(), "VARIABLE must not contain upper case");
        assert_eq!(std::fs::read_to_string(&file).unwrap(), "");
    }

    /// A value that is not a literal is reported with upstream's wording.
    #[test]
    fn test_an_unparseable_value_is_reported() {
        let dir = TempDir::new("badvalue");
        let file = dir.path().join("variables.cfg");
        let (_printer, gcode) = machine(&file);

        let error = gcode
            .run_script_sync("SAVE_VARIABLE VARIABLE=x VALUE=hello")
            .expect_err("the value is refused");

        assert_eq!(error.to_string(), "Unable to parse 'hello' as a literal");
        assert_eq!(std::fs::read_to_string(&file).unwrap(), "");
    }

    /// A file that cannot be parsed fails the load with upstream's message.
    #[test]
    fn test_a_corrupt_file_is_reported() {
        for contents in ["not a section\n", "[Variables]\nx = nonsense\n"] {
            let dir = TempDir::new("corrupt");
            let file = dir.path().join("variables.cfg");
            std::fs::write(&file, contents).unwrap();
            let text = format!("[save_variables]\nfilename: {}\n", file.display());
            let (config, _) = Config::from_text(&text).expect("the section parses");
            let printer = Arc::new(Printer::new(ManualReactor::shared()));

            let error = printer
                .load_config(&config)
                .expect_err("the load is refused");

            assert_eq!(error.to_string(), PARSE_ERROR, "contents: {contents:?}");
        }
    }

    /// A write that cannot reach the file is reported with upstream's wording.
    #[test]
    fn test_a_failed_write_is_reported() {
        let dir = TempDir::new("writefail");
        let file = dir.path().join("variables.cfg");
        let (_printer, gcode) = machine(&file);
        // Replace the file with a directory so the rewrite fails.
        std::fs::remove_file(&file).unwrap();
        std::fs::create_dir(&file).unwrap();

        let error = gcode
            .run_script_sync("SAVE_VARIABLE VARIABLE=x VALUE=1")
            .expect_err("the write is refused");

        assert_eq!(error.to_string(), "Unable to save variable");
    }

    /// Every value in the dialect survives `format_literal` → `parse_literal`.
    #[test]
    fn test_every_supported_literal_round_trips() {
        for value in [
            json!(1),
            json!(-2),
            json!(2.5),
            json!(0.0),
            json!(true),
            json!(false),
            Value::Null,
            json!("hello"),
            json!("it's"),
            json!("a\nb\tc"),
            json!([1, 2.5, true, "x"]),
            json!({ "a": 1, "b": [true, null] }),
        ] {
            let text = format_literal(&value);
            assert_eq!(parse_literal(&text), Some(value.clone()), "text: {text:?}");
        }
    }

    /// The Python `repr` spellings are read, and JSON's lowercase names are
    /// rejected the way `ast.literal_eval` rejects a name.
    #[test]
    fn test_python_repr_spellings_are_read() {
        assert_eq!(parse_literal("'hello'"), Some(json!("hello")));
        assert_eq!(parse_literal("\"hello\""), Some(json!("hello")));
        assert_eq!(parse_literal("True"), Some(json!(true)));
        assert_eq!(parse_literal("None"), Some(Value::Null));
        assert_eq!(parse_literal("[1, 2]"), Some(json!([1, 2])));
        assert_eq!(parse_literal("{'a': 1}"), Some(json!({ "a": 1 })));
        assert_eq!(parse_literal("true"), None);
        assert_eq!(parse_literal("null"), None);
        assert_eq!(parse_literal("hello"), None);
        assert_eq!(parse_literal(""), None);
    }

    /// A leading `~` expands to `$HOME`; a bare relative or absolute path is
    /// left as written.
    #[test]
    fn test_a_tilde_filename_expands_to_the_home_directory() {
        if let Some(home) = std::env::var_os("HOME").filter(|value| !value.is_empty()) {
            let home = PathBuf::from(home);
            assert_eq!(expanduser("~"), home);
            assert_eq!(expanduser("~/variables.cfg"), home.join("variables.cfg"));
        }
        assert_eq!(
            expanduser("/tmp/variables.cfg"),
            PathBuf::from("/tmp/variables.cfg")
        );
        // `~user` needs `pwd` and is left as written.
        assert_eq!(
            expanduser("~nobody/variables.cfg"),
            PathBuf::from("~nobody/variables.cfg")
        );
    }

    /// A macro's template reads the saved variables through
    /// `printer.save_variables.variables`.
    #[test]
    fn test_a_template_reads_the_saved_variables() {
        let dir = TempDir::new("template");
        let file = dir.path().join("variables.cfg");
        let (printer, gcode) = machine(&file);
        gcode
            .run_script_sync("SAVE_VARIABLE VARIABLE=greeting VALUE=\"'hi'\"")
            .expect("the save succeeds");

        let mut context = Context::new();
        context.insert(
            "printer",
            Rt::Printer(PrinterView::new(Arc::clone(&printer))),
        );
        let template =
            Template::parse("test", "{printer.save_variables.variables.greeting}").expect("parses");

        assert_eq!(template.render(&mut context).expect("renders"), "hi");
    }
}
