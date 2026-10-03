//! A `minijinja`-backed adapter for the macro bodies of `gcode_macro`.
//!
//! Upstream renders every macro body with `jinja2.Environment('{%', '%}', '{',
//! '}')` (`klippy/extras/gcode_macro.py:83`) — the variable delimiters are
//! **single braces**, which is why the corpus writes `{params.P}` rather than
//! `{{ params.P }}` — and `TemplateWrapper` (`:46-77`) compiles the body at
//! load and renders it against `create_template_context` (`:106-113`) before
//! `gcode.run_script_from_command` feeds the text back to the dispatcher.
//!
//! This module adapts `minijinja` 2.24 (`Cargo.toml`) to that environment.
//! [`Template::parse`] and [`Template::render`] keep this port's contract: a
//! body that does not parse is a **config-load** error, a body that does not
//! evaluate is a **command** error, the text between tags survives verbatim,
//! and the error frames are upstream's (`gcode_macro.py:46-77`).
//!
//! # The environment
//!
//! | setting | value | why |
//! |---|---|---|
//! | delimiters | `{% %}` / `{ }` / `{# #}` | upstream's `jinja2.Environment('{%', '%}', '{', '}')` (`gcode_macro.py:83`) |
//! | undefined behavior | `UndefinedBehavior::Strict` | a missing status key must not render a blank `PARK_` line — the rule the hand-written subset had |
//! | auto escape | `AutoEscape::None` | a macro body is g-code, not HTML (the engine's default callback switches on the template name's extension) |
//! | trailing newline | kept | a corpus body ends with a newline and the rendered text is run line by line; Jinja2/minijinja drop the last one by default (see the deviation table) |
//! | filters | the engine's, with `int`, `float`, `min`, `max` replaced | the corpus spells its coercions `params.S\|default(1000.0)\|float` and folds lists with `\|min`/`\|max` |
//! | globals | the engine's, with `range` and `namespace` bound explicitly | `range(...)` (`exclude_object.cfg:92`) and `namespace(...)`, which `gcode_macro` bodies call — the corpus spells `range`, this module's stepper-macro test binds `namespace` |
//!
//! Compiling and evaluating are **two phases**: [`Template::parse`] compiles the
//! body into the environment (`Environment::add_template_owned`),
//! [`Template::render`] evaluates the compiled template. `Environment::render_str`
//! is deliberately not used — it does both in one call, which is exactly the
//! distinction the two error frames draw. One [`Environment`] is built per
//! template, because a `minijinja::Template` borrows the environment it was
//! looked up in and the two cannot live in one struct; the environment's
//! defaults are `Arc`-shared, so a body's compile cost is the body's own.
//!
//! # What a render sees
//!
//! [`Context::insert`] binds [`Rt`] values the way `MacroState::context`
//! (`gcode_macro.rs`) builds them — `printer`, the `action_*` builtins,
//! `range`, `params`, `rawparams`, and the macro's `variable_*` values — and
//! each one becomes a minijinja value:
//!
//! - `Rt::Json` and `Rt::List` become minijinja containers, and their arrays
//!   **stay plain lists**: a list literal, `params`, or a `variable_*` list is
//!   indexed and iterated like any other sequence.
//! - `Rt::Printer` becomes the [`PrinterView`] object: `printer.<name>` and
//!   `printer["<name>"]` are one registered object's status, cached for the
//!   render the way `GetStatusWrapper.cache` caches it (`gcode_macro.py:20-33`),
//!   and `'<name>' in printer` asks whether that object is registered
//!   (`GetStatusWrapper.__contains__`, `:33-37`) — the engine's containment
//!   check on a map object *is* its lookup, so an unregistered name is absent,
//!   not an error. The view is not enumerable, the way upstream's wrapper is
//!   not iterable; a `{% for %}` over `printer` is refused rather than
//!   silently walking every object the machine registered.
//! - **Only a `printer` status array** becomes a [`Coord`]: klippy reports
//!   `Coord` namedtuples (`klippy/gcode.py`) whose fields are `x y z e`
//!   (`mathutil.rs`), this host's statuses are JSON arrays, and a body reads
//!   `printer.toolhead.position.x` (`macros.cfg:34`). A coordinate object also
//!   takes integer subscripts and `{% for v in printer.toolhead.position %}`;
//!   `.w` — a field `Coord` does not have — is undefined, exactly as it is on
//!   the namedtuple. Arrays **outside** a status keep their JSON meaning.
//! - `Rt::Builtin` becomes a callable: `range`, `action_respond_info`
//!   (`gcode_macro.py:94-96`) and `action_raise_error` (`:97-98`), the last
//!   failing the render with its own message.
//!
//! # Deviations
//!
//! The engine is minijinja's, so anything it accepts is accepted here. Most of
//! what the hand-written subset refused is Jinja2 too, and is now simply
//! available: `{% block %}`/`{% include %}`/`{% macro %}`/`{% with %}`/
//! `{% raw %}`/`{% filter %}`, `//`, `**`, `~`, `{% if %}` expressions, keyword
//! arguments (`default(0, boolean=True)`), chained comparisons, `is none`,
//! `|abs`, `|replace`, `|length`, … A body outside the engine's own syntax is
//! still a loud load error.
//!
//! Where this port still differs, deliberately:
//!
//! | case | Jinja2 (upstream) | this port |
//! |---|---|---|
//! | `//` and `%` on a negative operand | floor division / floor remainder (`-7 % 3 == 2`, `-7 // 3 == -3`) | Euclidean (`math_binop!(rem, checked_rem_euclid, %)`, `int_div`'s `div_euclid`); the corpus and this module's stepper-macro test use only non-negative operands, where the two agree |
//! | undefined in a *lookup* | `Undefined` value, error only when used | same: `UndefinedBehavior::Strict` fails at **print / iterate / test**, not when the name or key is read, so `{% if x is defined %}` and `\|default(…)` still probe quietly |
//! | `\|default` | catches `Undefined` only | same (the old subset probed its whole base expression quietly, so it also swallowed real errors) |
//! | `\|int`, `\|float` with an unusable value and **no** default | `0` / `0.0` | an error (`invalid literal for int()`), the old subset's rule — a macro that reads a missing number should not silently drive a pin with `0`; with a default (`\|int(0)`, `\|float(0.25)`) the default is returned, Jinja2's own shape |
//! | `\|min`, `\|max` on an empty sequence | `Undefined` (renders blank) | an error (`min() arg is an empty sequence`); there is no `Undefined` to hand back and a blank line would be silent |
//! | `range(a, b, step)` | 1–3 arguments | same (the old subset took exactly one) |
//! | a missing status object (`printer.nope`) | `Undefined` | same: undefined, which Strict then fails on if it is used |
//! | trailing newline | one newline dropped | kept (`set_keep_trailing_newline(true)`), because `idle_timeout`'s default script and the corpus' bodies are written to keep theirs |
//! | `{-3.5}` | `3.5`: `-` right after the opening brace is the whitespace-control marker (`{{-`'s, with single-brace delimiters), which swallows the sign — measured against `jinja2.Environment('{%','%}','{','}')` | same; a negative literal after `{` needs the space (`{ -3.5}`), and no corpus body writes `{-` |
//!
//! Detail wording differs wherever the engine owns the wording. The frames
//! (`Error loading template '<name>'\nline <n>: …`, `Error evaluating
//! '<name>': line <n>: …`) are this port's; what follows `line <n>: ` is the
//! engine's `detail` (or its error kind when it has none):
//!
//! | old subset's detail | this port's detail |
//! |---|---|
//! | `'nope' is undefined` | `undefined value` |
//! | `position has no attribute 'w'` | `undefined value` |
//! | `printer has no object 'nope'` | `undefined value` |
//! | `unsupported statement 'block' (this port implements …)` | `unknown statement foo` (for a tag no Jinja2 has) |
//! | `unknown filter 'abs' (this port implements …)` | `filter nosuch is unknown` (`abs` is a filter now, as in Jinja2) |
//! | `the 'float' filter takes at most 1 argument here, got 2 (this port's gap)` | `too many arguments` |
//! | `invalid literal for int(): "oops" (jinja's 'int' filter)` | `invalid literal for int(): oops` |
//! | `cannot iterate over number` | unchanged (this port's own `min`/`max`) |
//! | `unorderable types: str and number (<)` | unchanged (this port's own `min`/`max`) |
//! | `min() arg is an empty sequence (jinja2 leaves it undefined; …)` | `min() arg is an empty sequence` |
//!
//! A failure the engine reports without a location (`error.line()` is `None`)
//! reads as line 1.

use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::sync::{Arc, Mutex, MutexGuard};

use minijinja::syntax::SyntaxConfig;
use minijinja::value::{Enumerator, Object, ObjectRepr, Value as MjValue, ValueKind};
use minijinja::{AutoEscape, Environment, Error as MjError, ErrorKind, UndefinedBehavior::Strict};
use serde_json::Value;

use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::printer::Printer;

// ===========================================================================
// Errors
// ===========================================================================

/// When a template failed: at load (`Template::parse`) or at render
/// (`Template::render`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Load,
    Evaluate,
}

/// A template failure, worded after upstream's `TemplateWrapper`
/// (`gcode_macro.py:46-77`): a load error is
/// `Error loading template '<name>'\nline <n>: <detail>`; a render error is
/// `Error evaluating '<name>': line <n>: <detail>` — the line is this port's
/// addition, because a rendered macro body spans many of them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TemplateError {
    name: String,
    line: usize,
    phase: Phase,
    detail: String,
}

impl TemplateError {
    /// Frame an engine failure: its own detail when it has one, its error kind
    /// otherwise (a strict undefined fails as `undefined value`), and its line
    /// when it recorded one.
    fn of(name: &str, phase: Phase, error: &MjError) -> Self {
        Self {
            name: name.to_string(),
            line: error.line().unwrap_or(1),
            phase,
            detail: error
                .detail()
                .map(str::to_string)
                .unwrap_or_else(|| error.kind().to_string()),
        }
    }
}

impl fmt::Display for TemplateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.phase {
            Phase::Load => write!(
                f,
                "Error loading template '{}'\nline {}: {}",
                self.name, self.line, self.detail
            ),
            Phase::Evaluate => write!(
                f,
                "Error evaluating '{}': line {}: {}",
                self.name, self.line, self.detail
            ),
        }
    }
}

impl std::error::Error for TemplateError {}

// ===========================================================================
// Runtime values
// ===========================================================================

/// One name bound in a [`Context`]: a JSON value, the `printer` view, or a
/// builtin the corpus' templates call.
#[derive(Debug, Clone)]
pub enum Rt {
    /// Status data, `params`, a `variable_*` literal — the JSON world.
    Json(Value),
    /// A list of [`Rt`] values, kept from the hand-written engine's API — a
    /// render turns it into an engine list of the same elements.
    List(Vec<Rt>),
    /// `printer`: upstream's `GetStatusWrapper` (`gcode_macro.py:15-43`).
    Printer(PrinterView),
    /// `range` / `action_*`.
    Builtin(Builtin),
}

/// The callables a template context binds.
#[derive(Clone)]
pub enum Builtin {
    /// `range(...)` — `exclude_object.cfg:92`.
    Range,
    /// `action_respond_info(msg)` (`gcode_macro.py:94-96`): logs the line and
    /// renders nothing.
    RespondInfo(Arc<Printer>),
    /// `action_raise_error(msg)` (`gcode_macro.py:97-98`): fails the render,
    /// which fails the command.
    RaiseError,
}

impl fmt::Debug for Builtin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `Printer` has no `Debug`; name the builtin, not what it closes over.
        match self {
            Builtin::Range => f.write_str("Builtin::Range"),
            Builtin::RespondInfo(_) => f.write_str("Builtin::RespondInfo"),
            Builtin::RaiseError => f.write_str("Builtin::RaiseError"),
        }
    }
}

/// `printer.<name>` / `printer["<name>"]` — one object's status, cached for
/// the render the way `GetStatusWrapper.cache` caches it (`gcode_macro.py:20-33`).
///
/// The cache is a `Mutex` because the engine requires its objects to be `Sync`;
/// the view is `Clone` so a [`Rt`] can be cloned into a render.
#[derive(Clone)]
pub struct PrinterView {
    printer: Arc<Printer>,
    cache: Arc<Mutex<HashMap<String, MjValue>>>,
}

impl fmt::Debug for PrinterView {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `Printer` has no `Debug`; the cache is the interesting part.
        f.debug_struct("PrinterView")
            .field("cached", &self.lock().keys().collect::<Vec<_>>())
            .finish()
    }
}

impl PrinterView {
    /// View `printer` through `eventtime`-stamped statuses.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            cache: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// One object's status (`GetStatusWrapper.__getitem__`, `:19-32`), as the
    /// templates read it: `None` for a name nobody registered, which is what
    /// makes `'name' in printer` a registration test.
    fn status(&self, name: &str) -> Option<MjValue> {
        if let Some(cached) = self.lock().get(name) {
            return Some(cached.clone());
        }
        let status = status_value(&self.printer.status_of(name, self.printer.eventtime())?);
        self.lock().insert(name.to_string(), status.clone());
        Some(status)
    }

    fn lock(&self) -> MutexGuard<'_, HashMap<String, MjValue>> {
        self.cache
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl Object for PrinterView {
    fn repr(self: &Arc<Self>) -> ObjectRepr {
        ObjectRepr::Map
    }

    fn get_value(self: &Arc<Self>, key: &MjValue) -> Option<MjValue> {
        self.status(key.as_str()?)
    }

    /// Not enumerable: upstream's wrapper is a `__getitem__`-only mapping, so
    /// `{% for %}` over `printer` is refused, while containment (a map lookup)
    /// and truthiness still work.
    fn enumerate(self: &Arc<Self>) -> Enumerator {
        Enumerator::NonEnumerable
    }
}

/// A `printer` status array, read the way klippy's `Coord` namedtuple is
/// (`klippy/gcode.py`): `.x`/`.y`/`.z`/`.e` are the first four elements
/// (`mathutil.rs`'s field order), integer subscripts index it, and `{% for %}`
/// walks it.
#[derive(Debug)]
struct Coord {
    items: Vec<MjValue>,
}

impl Object for Coord {
    fn repr(self: &Arc<Self>) -> ObjectRepr {
        ObjectRepr::Seq
    }

    fn get_value(self: &Arc<Self>, key: &MjValue) -> Option<MjValue> {
        if let Some(index) = key.as_usize() {
            return self.items.get(index).cloned();
        }
        let index = match key.as_str()? {
            "x" => 0,
            "y" => 1,
            "z" => 2,
            "e" => 3,
            _ => return None,
        };
        self.items.get(index).cloned()
    }

    fn enumerate(self: &Arc<Self>) -> Enumerator {
        Enumerator::Seq(self.items.len())
    }

    fn enumerator_len(self: &Arc<Self>) -> Option<usize> {
        Some(self.items.len())
    }
}

/// One status value as the templates read it: an array is a [`Coord`] — and so
/// are the arrays nested inside it — an object is a map of the same, and a
/// scalar is itself.
fn status_value(value: &Value) -> MjValue {
    match value {
        Value::Array(items) => MjValue::from_object(Coord {
            items: items.iter().map(status_value).collect(),
        }),
        Value::Object(map) => {
            let entries: BTreeMap<String, MjValue> = map
                .iter()
                .map(|(key, item)| (key.clone(), status_value(item)))
                .collect();
            MjValue::from_serialize(&entries)
        }
        other => MjValue::from_serialize(other),
    }
}

/// One binding as the engine sees it.
fn rt_value(value: Rt) -> MjValue {
    match value {
        // JSON and literal lists become the engine's own containers: a
        // `params` value or a `variable_*` list is a plain list, **not** a
        // coordinate array — only a `printer` status is (`status_value`).
        Rt::Json(json) => MjValue::from_serialize(&json),
        Rt::List(items) => {
            MjValue::from_serialize(items.into_iter().map(rt_value).collect::<Vec<_>>())
        }
        Rt::Printer(view) => MjValue::from_object(view),
        Rt::Builtin(Builtin::Range) => MjValue::from_function(minijinja::functions::range),
        Rt::Builtin(Builtin::RespondInfo(printer)) => {
            MjValue::from_function(move |message: MjValue| -> Result<MjValue, MjError> {
                if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                    gcode.respond_info(&message.to_string(), true);
                }
                Ok(MjValue::from(""))
            })
        }
        Rt::Builtin(Builtin::RaiseError) => {
            MjValue::from_function(|message: MjValue| -> Result<MjValue, MjError> {
                Err(MjError::new(
                    ErrorKind::InvalidOperation,
                    message.to_string(),
                ))
            })
        }
    }
}

// ===========================================================================
// Context
// ===========================================================================

/// The names a render resolves against: the globals a macro builds (printer,
/// actions, `params`, `rawparams`, the macro's `variable_*` values), plus
/// whatever `{% set %}` and `{% for %}` scope while the body runs.
#[derive(Debug, Default)]
pub struct Context {
    globals: HashMap<String, Rt>,
}

impl Context {
    /// An empty context; the caller binds what the template may see.
    pub fn new() -> Self {
        Self::default()
    }

    /// Bind a global (`create_template_context` plus `kwparams`,
    /// `gcode_macro.py:106-113`).
    pub fn insert(&mut self, name: impl Into<String>, value: Rt) {
        self.globals.insert(name.into(), value);
    }

    /// The bindings as the engine takes them: one map of minijinja values,
    /// which is the render's root scope — `{% set %}` inside an `{% if %}`
    /// assigns into it, `{% for %}` pushes its own frame per iteration.
    fn bindings(&self) -> BTreeMap<String, MjValue> {
        self.globals
            .iter()
            .map(|(name, value)| (name.clone(), rt_value(value.clone())))
            .collect()
    }
}

// ===========================================================================
// Template
// ===========================================================================

/// A compiled template: its name (`gcode_macro M486:gcode`, the upstream
/// `TemplateWrapper` name, `gcode_macro.py:47-65`) and the environment it was
/// compiled into.
pub struct Template {
    name: String,
    env: Environment<'static>,
}

impl fmt::Debug for Template {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Template")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

impl Template {
    /// Compile `source`, reporting an unparsable body with upstream's
    /// load-error frame (`gcode_macro.py:57-60`).
    ///
    /// # Errors
    /// A syntax error in the body: an unclosed tag, a statement the engine
    /// does not know, an unbalanced block, a malformed expression.
    pub fn parse(name: &str, source: &str) -> Result<Self, TemplateError> {
        let mut env = environment();
        env.add_template_owned(name.to_string(), source.to_string())
            .map_err(|error| TemplateError::of(name, Phase::Load, &error))?;
        Ok(Self {
            name: name.to_string(),
            env,
        })
    }

    /// The template's upstream-style name (`<section>:<option>`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Render against `context`, or report the first failing expression with
    /// upstream's `Error evaluating` frame (`gcode_macro.py:70-79`).
    ///
    /// The context's bindings are the render's root frame, so a top-level
    /// `{% set %}` lands there and a later node sees it, an `{% if %}` body
    /// shares it, and a `{% for %}` body does not — the scoping Jinja2 gives
    /// those three, and what `MacroState` relies on.
    pub fn render(&self, context: &mut Context) -> Result<String, TemplateError> {
        let template = self
            .env
            .get_template(&self.name)
            .map_err(|error| TemplateError::of(&self.name, Phase::Evaluate, &error))?;
        template
            .render(&context.bindings())
            .map_err(|error| TemplateError::of(&self.name, Phase::Evaluate, &error))
    }
}

/// The environment every body is compiled into: upstream's delimiters,
/// undefined behavior and escaping, this port's trailing-newline rule, the four
/// filters the corpus spells differently, and the two Jinja2 globals the
/// corpus (`range`) and this module's stepper-macro test (`namespace`) call.
fn environment() -> Environment<'static> {
    let mut env = Environment::new();
    env.set_syntax(
        SyntaxConfig::builder()
            .block_delimiters("{%", "%}")
            .variable_delimiters("{", "}")
            .comment_delimiters("{#", "#}")
            .build()
            .expect("'{%' and '{' are distinct start delimiters"),
    );
    env.set_auto_escape_callback(|_| AutoEscape::None);
    env.set_undefined_behavior(Strict);
    env.set_keep_trailing_newline(true);
    env.add_filter("int", filter_int);
    env.add_filter("float", filter_float);
    env.add_filter("min", filter_min);
    env.add_filter("max", filter_max);
    env.add_function("range", minijinja::functions::range);
    env.add_function("namespace", minijinja::functions::namespace);
    env
}

// ===========================================================================
// Filters
// ===========================================================================

/// `| int` and `| int(default)`.
///
/// `int(value)` with Jinja2's optional default. A value the conversion cannot
/// take falls back to the default — `|int(0)` is Jinja2's spelling of the
/// corpus' `|default(0)|int` — and **without** a default it fails, which is
/// this port's rule: a body that meant `|int(0)` and wrote `|int` should not
/// drive a pin with a number nobody supplied.
///
/// An undefined value is always an error, default or not: that is what Jinja2
/// does (`Undefined.__int__` raises) and what the old subset did (it evaluated
/// the filter's base strictly).
fn filter_int(value: &MjValue, default: Option<MjValue>) -> Result<MjValue, MjError> {
    if value.is_undefined() {
        return Err(MjError::from(ErrorKind::UndefinedError));
    }
    let parsed = match value.kind() {
        ValueKind::Number => value
            .as_i64()
            .or_else(|| f64::try_from(value.clone()).ok().map(|f| f.trunc() as i64)),
        ValueKind::Bool => Some(i64::from(value.is_true())),
        ValueKind::String => {
            let text = value.as_str().unwrap_or_default().trim();
            text.parse::<i64>()
                .ok()
                .or_else(|| text.parse::<f64>().ok().map(|f| f.trunc() as i64))
        }
        _ => None,
    };
    match parsed {
        Some(int) => Ok(MjValue::from(int)),
        None => match default {
            Some(fallback) => Ok(fallback),
            None => Err(MjError::new(
                ErrorKind::InvalidOperation,
                format!("invalid literal for int(): {}", value),
            )),
        },
    }
}

/// `| float` and `| float(default)`.
///
/// `float(value)` with Jinja2's optional default: a value Python's `float()`
/// would reject — `None`, a container, a string that is not a number — falls
/// back to the argument, or to `0.0` when the filter has none
/// (`sample-macros.cfg:285`). `1e3` and surrounding whitespace are Python's
/// own spellings and are accepted.
///
/// As above, an undefined value is an error with or without a default.
fn filter_float(value: &MjValue, default: Option<MjValue>) -> Result<MjValue, MjError> {
    if value.is_undefined() {
        return Err(MjError::from(ErrorKind::UndefinedError));
    }
    let fallback = || default.clone().unwrap_or_else(|| MjValue::from(0.0));
    let parsed = match value.kind() {
        ValueKind::Number => f64::try_from(value.clone()).ok(),
        ValueKind::Bool => Some(if value.is_true() { 1.0 } else { 0.0 }),
        ValueKind::String => value
            .as_str()
            .unwrap_or_default()
            .trim()
            .parse::<f64>()
            .ok(),
        _ => None,
    };
    match parsed {
        Some(float) => Ok(MjValue::from(float)),
        None => Ok(fallback()),
    }
}

/// `| min` — the smallest item of a sequence
/// (`generic_cartesian_iqex.cfg:286-287`).
///
/// Jinja's `_min_or_max` with its default `case_sensitive=False`: strings
/// compare folded to lower case while the *original* item comes back
/// (`min(["B", "a"])` is `"a"`), and everything else is its own key.
fn filter_min(value: MjValue) -> Result<MjValue, MjError> {
    extreme("min", value, true)
}

/// `| max` — the largest item of a sequence, as `| min` reads it.
fn filter_max(value: MjValue) -> Result<MjValue, MjError> {
    extreme("max", value, false)
}

fn extreme(filter: &str, value: MjValue, least: bool) -> Result<MjValue, MjError> {
    let mut items = sequence(&value)?.into_iter();
    let Some(mut best) = items.next() else {
        return Err(MjError::new(
            ErrorKind::InvalidOperation,
            format!("{filter}() arg is an empty sequence"),
        ));
    };
    let op = if least { "<" } else { ">" };
    for item in items {
        let ordering = key_order(&item, &best).ok_or_else(|| {
            MjError::new(
                ErrorKind::InvalidOperation,
                format!(
                    "unorderable types: {} and {} ({op})",
                    ordering_name(&item),
                    ordering_name(&best)
                ),
            )
        })?;
        let replaces = if least {
            ordering.is_lt()
        } else {
            ordering.is_gt()
        };
        if replaces {
            best = item;
        }
    }
    Ok(best)
}

/// The values `| min`/`| max` walk, with a non-sequence named the way CPython
/// names it (`cannot iterate over number`).
fn sequence(value: &MjValue) -> Result<Vec<MjValue>, MjError> {
    match value.kind() {
        ValueKind::String | ValueKind::Seq | ValueKind::Iterable | ValueKind::Map => value
            .try_iter()
            .map(|items| items.collect())
            .map_err(|_| uniterable(value)),
        _ => Err(uniterable(value)),
    }
}

fn uniterable(value: &MjValue) -> MjError {
    MjError::new(
        ErrorKind::InvalidOperation,
        format!("cannot iterate over {}", value.kind()),
    )
}

/// How `| min`/`| max` order two items: numbers (booleans count as `0`/`1`, as
/// Python compares them) numerically, strings folded to lower case, and
/// anything else unorderable — CPython's own refusal, which this port keeps
/// loudly rather than falling back on a cross-type ordering.
fn key_order(left: &MjValue, right: &MjValue) -> Option<std::cmp::Ordering> {
    match (number_key(left), number_key(right)) {
        (Some(a), Some(b)) => return Some(a.cmp(&b)),
        (None, None) => {}
        _ => return None,
    }
    let (a, b) = (left.as_str()?, right.as_str()?);
    Some(a.to_lowercase().cmp(&b.to_lowercase()))
}

/// A number or a boolean as the number it compares as.
fn number_key(value: &MjValue) -> Option<MjValue> {
    match value.kind() {
        ValueKind::Number => Some(value.clone()),
        ValueKind::Bool => Some(MjValue::from(i64::from(value.is_true()))),
        _ => None,
    }
}

/// The Python type name an `unorderable types` message uses.
fn ordering_name(value: &MjValue) -> &'static str {
    match value.kind() {
        ValueKind::Number | ValueKind::Bool => "number",
        ValueKind::String => "str",
        ValueKind::None | ValueKind::Undefined => "NoneType",
        ValueKind::Seq => "list",
        ValueKind::Map => "dict",
        _ => "object",
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    use crate::core::klippy::printer::PrinterObject;
    use crate::core::klippy::reactor::ManualReactor;

    /// A context with the corpus' `params` / `rawparams` and one macro
    /// variable, the way `MacroState::context` builds them
    /// (`gcode_macro.rs`), minus the printer.
    fn context(params: &[(&str, &str)], rawparams: &str) -> Context {
        let mut context = Context::new();
        let map: serde_json::Map<String, Value> = params
            .iter()
            .map(|(key, value)| ((*key).to_string(), json!(value)))
            .collect();
        context.insert("params", Rt::Json(Value::Object(map)));
        context.insert("rawparams", Rt::Json(Value::String(rawparams.to_string())));
        context.insert("t", Rt::Json(json!(12.0)));
        // The production context binds this (`MacroState::context`).
        context.insert("range", Rt::Builtin(Builtin::Range));
        context
    }

    fn render(source: &str, context: &mut Context) -> Result<String, TemplateError> {
        Template::parse("gcode_macro TEST:gcode", source)?.render(context)
    }

    fn ok(source: &str) -> String {
        render(source, &mut context(&[], "")).unwrap_or_else(|error| panic!("{error}"))
    }

    /// A machine with one object registered under `name` whose status is
    /// `status`, for the tests that read `printer.<name>`.
    fn printer_with(name: &str, status: Value) -> Arc<Printer> {
        struct Fixed(Value);
        impl PrinterObject for Fixed {
            fn get_status(&self, _eventtime: f64) -> Value {
                self.0.clone()
            }
        }
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(name, Arc::new(Fixed(status)))
            .expect("the name is free");
        printer
    }

    /// The scalar spellings are Python's `str`, because the body is rendered
    /// through the engine's own formatter — `3.0`, not `3`; `True`, not `true`.
    #[test]
    fn expressions_render_python_spelling() {
        assert_eq!(ok("PARK_{3}"), "PARK_3");
        assert_eq!(ok("{3.0}"), "3.0");
        assert_eq!(ok("{12.0 - 12.0}"), "0.0");
        assert_eq!(ok("{1 + 2}"), "3");
        assert_eq!(ok("{True}/{False}/{None}"), "True/False/None");
        assert_eq!(ok("{ 17 * 2 + 1 % 4 }"), "35");
        // `-` right after the opening brace is the whitespace-control marker,
        // as `{{-` is with the default delimiters: upstream's
        // `jinja2.Environment('{%','%}','{','}')` renders `{-3.5}` as `3.5`
        // too, so the space is what upstream requires as well.
        assert_eq!(ok("{-3.5}"), "3.5");
        assert_eq!(ok("{ -3.5}"), "-3.5");
        assert_eq!(ok("{ 0 - 3.5 }"), "-3.5");
    }

    /// `if` / `elif` / `else` and `for` over `range(… | int)`, the shapes
    /// `exclude_object.cfg:85-113` uses.
    #[test]
    fn branches_and_loops_follow_the_corpus_shapes() {
        let source = "{% if 'T' in params %}RESET{% for i in range(params.T | int) %}NAME={i} \
                      {% endfor %}{% elif 'C' in params %}CANCEL{% else %}none{% endif %}";
        assert_eq!(
            render(source, &mut context(&[("T", "3")], "")).expect("renders"),
            "RESETNAME=0 NAME=1 NAME=2 "
        );
        assert_eq!(
            render(source, &mut context(&[("C", "1")], "")).expect("renders"),
            "CANCEL"
        );
        assert_eq!(
            render(source, &mut context(&[], "")).expect("renders"),
            "none"
        );
        // One tag line per line, as the corpus writes them: the text between
        // tags survives verbatim, tags themselves vanish.
        let multiline = "{% if params.S == '-1' %}\n  EXCLUDE_OBJECT_END\n{% else %}\n  \
                         EXCLUDE_OBJECT_START NAME={params.S}\n{% endif %}";
        assert_eq!(
            render(multiline, &mut context(&[("S", "0")], "")).expect("renders"),
            "\n  EXCLUDE_OBJECT_START NAME=0\n"
        );
    }

    /// `in` / `not in` / `or`, and the `is defined` tests `sdcard_loop.cfg:90`
    /// leans on — undefined is *false*, never an error.
    #[test]
    fn membership_and_defined_tests_behave_like_jinja() {
        let source = "{% if 'abc' in params or 'nope' not in params %}M112{% endif %}";
        // Absent keys: `'abc' in params` is false, `'nope' not in params` true.
        assert_eq!(
            render(source, &mut context(&[], "")).expect("renders"),
            "M112"
        );
        let present = "{% if 'abc' in params %}M112{% endif %}";
        assert_eq!(
            render(present, &mut context(&[("T", "3")], "")).expect("renders"),
            ""
        );

        let sdcard = "{% if params.K is not defined and params.L is defined %}\
                      SDCARD_LOOP_BEGIN COUNT={params.L|int}{% endif %}";
        assert_eq!(
            render(sdcard, &mut context(&[("L", "5")], "")).expect("renders"),
            "SDCARD_LOOP_BEGIN COUNT=5"
        );
        assert_eq!(
            render(sdcard, &mut context(&[("K", "1")], "")).expect("renders"),
            ""
        );
    }

    /// A **status** array is read the way klippy's `Coord` namedtuple is
    /// (`klippy/gcode.py`): `.x`/`.y`/`.z`/`.e`, integer subscripts, and
    /// iteration — the bridge `macros.cfg:34` needs. `printer` itself answers
    /// containment by registration, and is not iterable.
    #[test]
    fn printer_status_arrays_expose_coordinate_fields() {
        let printer = printer_with(
            "toolhead",
            json!({"position": [1.5, 2.5, 3.5, 4.5], "extruder": "extruder"}),
        );
        let mut context = Context::new();
        context.insert("printer", Rt::Printer(PrinterView::new(printer)));

        assert_eq!(
            render("{printer.toolhead.position.x} {printer.toolhead.position.y} {printer.toolhead.position.z} {printer.toolhead.position.e}", &mut context)
                .expect("renders"),
            "1.5 2.5 3.5 4.5"
        );
        // Integer subscripts and `{% for %}` read the same array.
        assert_eq!(
            render("{printer.toolhead.position[2]}", &mut context).expect("renders"),
            "3.5"
        );
        assert_eq!(
            render(
                "{% for v in printer.toolhead.position %}{v} {% endfor %}",
                &mut context
            )
            .expect("renders"),
            "1.5 2.5 3.5 4.5 "
        );
        // `'name' in printer` is registration (`GetStatusWrapper.__contains__`).
        assert_eq!(
            render(
                "{% if 'toolhead' in printer %}yes{% else %}no{% endif %}",
                &mut context
            )
            .expect("renders"),
            "yes"
        );
        assert_eq!(
            render(
                "{% if 'nope' in printer %}yes{% else %}no{% endif %}",
                &mut context
            )
            .expect("renders"),
            "no"
        );
        // The view itself is truthy, as upstream's wrapper is.
        assert_eq!(
            render("{% if printer %}truthy{% endif %}", &mut context).expect("renders"),
            "truthy"
        );
        // Outside the four Coord fields an array attribute is undefined, as it
        // is on the namedtuple.
        let error = render("{printer.toolhead.position.w}", &mut context).expect_err("no field");
        assert!(error.to_string().contains("undefined"), "{error}");
    }

    /// An array outside `printer` keeps its JSON meaning: `params`,
    /// `variable_*`, `Rt::List` and list literals are plain lists, without
    /// `.x`.
    #[test]
    fn arrays_outside_printer_status_stay_plain_lists() {
        let mut context = context(&[], "");
        context.insert("points", Rt::Json(json!([1.5, 2.5])));
        context.insert(
            "variables",
            Rt::List(vec![
                Rt::Json(json!(1)),
                Rt::Json(Value::String("a".to_string())),
            ]),
        );
        assert_eq!(
            render("{points[1]} { [3, 4][0] } {variables[1]}", &mut context).expect("renders"),
            "2.5 3 a"
        );
        assert_eq!(
            render("{% for v in variables %}{v} {% endfor %}", &mut context).expect("renders"),
            "1 a "
        );
        let error = render("{points.x}", &mut context).expect_err("not a Coord");
        assert!(error.to_string().contains("undefined"), "{error}");
    }

    /// `rawparams` is the line's tail, verbatim (`gcode_macro.py:195`).
    #[test]
    fn rawparams_renders_the_command_tail() {
        assert_eq!(ok("{rawparams}"), "");
        assert_eq!(
            render("{rawparams}", &mut context(&[], "T=3 EXCLUDE=1")).expect("renders"),
            "T=3 EXCLUDE=1"
        );
    }

    /// A statement the engine does not know fails the **load** with upstream's
    /// frame (`gcode_macro.py:57-60`), naming the line and the statement.
    /// `{% block %}` is no longer one of them: Jinja2 parses it, and so does
    /// this engine — the old subset was the stricter one.
    #[test]
    fn an_unknown_statement_is_a_load_error_with_upstream_frame() {
        assert_eq!(ok("{% set x = 1 %}{ x }"), "1");

        let error = Template::parse("gcode_macro SETTY:gcode", "{% foo %}")
            .expect_err("foo is not a statement");
        assert_eq!(
            error.to_string(),
            "Error loading template 'gcode_macro SETTY:gcode'\nline 1: unknown statement foo"
        );

        assert!(
            Template::parse("gcode_macro SETTY:gcode", "{% block body %}{% endblock %}").is_ok(),
            "block is upstream Jinja2, and parses here"
        );

        // An unbalanced block reads the same way — the engine names the tag
        // it wanted, not the one it found.
        let error = Template::parse("gcode_macro BAD:gcode", "{% if 1 %}").expect_err("no endif");
        assert!(
            error.to_string().contains("expected end of block"),
            "{error}"
        );
    }

    /// `{% set %}` scoping, checked against jinja2 3.1.6 with upstream's
    /// delimiters: at the top level a later node sees the binding, an `if`
    /// body leaks outward, a `for` body does not (its frame dies with the
    /// iteration) but the same iteration and nested bodies do.
    #[test]
    fn set_statements_scope_like_jinja2() {
        // Top level, and a chain of top-level assignments.
        assert_eq!(ok("{% set x = 12 %}{ x }"), "12");
        assert_eq!(ok("{% set a = 2 %}{% set b = a + 1 %}{ b }"), "3");

        // `if` does not scope: the binding is visible after `endif`.
        assert_eq!(ok("{% if 1 %}{% set y = 5 %}{% endif %}{ y }"), "5");

        // `for` scopes: `z` reads inside the body and dies with the iteration.
        assert_eq!(
            ok(
                "{% for i in range(2) %}{% set z = i %}{ z }{% endfor %}{% if z is defined %}\
                LEAK{% endif %}"
            ),
            "01"
        );

        // The corpus sentence (`generic_cartesian_iqex.cfg:288`), rendered
        // after a later expression reads it back. Its `x_max`/`x_min` come
        // from the context here; the lines that set them (286-287) are the
        // list literals `set_copy_mode_templates_load_and_render` covers, and
        // line 285 reads `printer.*`.
        let mut context = context(&[], "");
        context.insert("x_max", Rt::Json(json!(300.0)));
        context.insert("x_min", Rt::Json(json!(0.0)));
        let source = "{% set x_center = 0.5 * (x_max + x_min) %}{ x_center }";
        assert_eq!(render(source, &mut context).expect("renders"), "150.0");
    }

    /// A name or filter outside the context fails the **render**, and a
    /// Strict undefined fails at the point the value is *used* — printed,
    /// iterated, or asked whether it is true — never at the lookup itself, so
    /// `is defined` and `\|default(…)` still probe quietly.
    #[test]
    fn an_unknown_name_or_filter_is_a_render_error() {
        let error = render("{nope}", &mut context(&[], "")).expect_err("undefined");
        assert_eq!(
            error.to_string(),
            "Error evaluating 'gcode_macro TEST:gcode': line 1: undefined value"
        );
        for source in [
            "{nope}",
            "{% if nope %}x{% endif %}",
            "{% for x in nope %}x{% endfor %}",
        ] {
            let error = render(source, &mut context(&[], "")).expect_err("undefined");
            assert!(error.to_string().contains("undefined value"), "{error}");
        }
        // …while the two quiet probes still answer.
        assert_eq!(ok("{% if nope is defined %}x{% else %}y{% endif %}"), "y");
        assert_eq!(ok("{nope|default(1)}"), "1");

        let error =
            render("{params.L | nosuch}", &mut context(&[("L", "-1")], "")).expect_err("no filter");
        assert_eq!(
            error.to_string(),
            "Error evaluating 'gcode_macro TEST:gcode': line 1: filter nosuch is unknown"
        );

        // `abs` is the engine's filter, as it is Jinja2's — the old subset
        // refused it. A string operand is what both refuse.
        let error =
            render("{params.L | abs}", &mut context(&[("L", "-1")], "")).expect_err("string");
        assert!(
            error.to_string().contains("cannot get absolute value"),
            "{error}"
        );

        // `action_raise_error` is the corpus's own escape hatch
        // (`exclude_object.cfg:86`) and fails the render with its message.
        let mut context = context(&[], "");
        context.insert("action_raise_error", Rt::Builtin(Builtin::RaiseError));
        let error = render(
            "{action_raise_error(\"[exclude_object] is not enabled\")}",
            &mut context,
        )
        .expect_err("raises");
        assert!(
            error
                .to_string()
                .contains("[exclude_object] is not enabled"),
            "{error}"
        );
    }

    /// `action_respond_info` is callable and renders nothing
    /// (`gcode_macro.py:94-96`): the body's surrounding text is all that
    /// reaches the command, and the line goes to the client through the
    /// dispatcher.
    #[test]
    fn action_respond_info_renders_nothing() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode registers");
        let mut context = context(&[], "");
        context.insert(
            "action_respond_info",
            Rt::Builtin(Builtin::RespondInfo(Arc::clone(&printer))),
        );
        assert_eq!(
            render(
                "before {action_respond_info(\"a line\")} after",
                &mut context
            )
            .expect("renders"),
            "before  after"
        );
    }

    /// The filter shapes the corpus writes, one per literal form: a parameter
    /// with a fallback (`|default(` appears 31 times, `|float` 29 times across
    /// `config/*.cfg` + `test/klippy/*.cfg`), always chained into a coercion.
    #[test]
    fn filter_arguments_follow_the_corpus_forms() {
        // `{% set S = params.S|default(1000.0)|float %}` — float literal
        // fallback (`printer-velleman-k8800-2017.cfg:125`, `sample-pwm-tool.cfg:21`,
        // `sample-macros.cfg:101-103` with `|default(0)|float`).
        let float_default = "{% set S = params.S|default(1000.0)|float %}{ S }";
        assert_eq!(
            render(float_default, &mut context(&[], "")).expect("S omitted"),
            "1000.0"
        );
        assert_eq!(
            render(float_default, &mut context(&[("S", "25")], "")).expect("S given"),
            "25.0"
        );

        // `{% set P = params.P|default(100)|int %}` — int literal fallback
        // (`printer-velleman-k8800-2017.cfg:127`, `sample-macros.cfg:79-81`,
        // `printer-geeetech-A10T-A20T-2021.cfg:175` with `|default(0)| int`).
        let int_default = "{% set P = params.P|default(100)|int %}{ P }";
        assert_eq!(
            render(int_default, &mut context(&[], "")).expect("P omitted"),
            "100"
        );
        assert_eq!(
            render(int_default, &mut context(&[("P", "300")], "")).expect("P given"),
            "300"
        );

        // `{% set X = params['X']|float %}` — a bare `|float` on an index base
        // (`sample-macros.cfg:285-306`).
        let bare = "{% set X = params['X']|float %}{ X }";
        assert_eq!(
            render(bare, &mut context(&[("X", "1.5")], "")).expect("X given"),
            "1.5"
        );

        // A `default` that is not chained (`display/menu.cfg:359`): the
        // fallback for an absent key, the key's own value whatever its type
        // when it is present.
        assert_eq!(
            render("{params.Q|default(0)}", &mut context(&[], "")).expect("Q omitted"),
            "0"
        );
        assert_eq!(
            render("{params.Q|default(0)}", &mut context(&[("Q", "x")], "")).expect("Q given"),
            "x"
        );
        // An undefined *name* falls back the same way, and a `None` is a value
        // `default` passes through (Jinja's `default` only catches undefined).
        assert_eq!(
            render("{nope|default(7)}", &mut context(&[], "")).expect("undefined name"),
            "7"
        );
        assert_eq!(
            render("{None|default(7)}", &mut context(&[], "")).expect("None is defined"),
            "None"
        );
    }

    /// `|float` without an argument is Jinja's `0.0` fallback
    /// (`sample-macros.cfg:285`), `|float(d)` its own default, and a chained
    /// `|float|default(d)` tests the value that resolves.
    #[test]
    fn float_falls_back_to_zero_or_to_its_argument() {
        assert_eq!(
            render("{None|float}", &mut context(&[], "")).expect("None"),
            "0.0"
        );
        assert_eq!(
            render("{params.S|float}", &mut context(&[("S", "oops")], "")).expect("not a number"),
            "0.0"
        );
        assert_eq!(
            render("{params.S|float(0.25)}", &mut context(&[("S", "oops")], "")).expect("default"),
            "0.25"
        );
        assert_eq!(
            render(
                "{params.S|float|default(0)}",
                &mut context(&[("S", "2.5")], "")
            )
            .expect("chain"),
            "2.5"
        );
        // A string `float()` accepts keeps Python's spellings (`float("1e3")`).
        assert_eq!(
            render("{params.S|float}", &mut context(&[("S", "1e3")], "")).expect("exponent"),
            "1000.0"
        );
        // `|float` on an undefined name stays an error: the corpus spells the
        // fallback as `|default(0.0)|float` when it wants one
        // (`sample-pwm-tool.cfg:21`).
        let error = render("{nope|float}", &mut context(&[], "")).expect_err("undefined");
        assert!(error.to_string().contains("undefined"), "{error}");
    }

    /// The blocks the corpus puts a filter in: an `{% if %}` condition
    /// (`printer-anycubic-4maxpro-2.0-2021.cfg:171`) and a `{% for %}`
    /// iterable (`exclude_object.cfg:92`'s shape).
    #[test]
    fn filters_work_inside_if_and_for_blocks() {
        let branch = "{% if params.S|default(0)|int > 0 %}HOT{% else %}COLD{% endif %}";
        assert_eq!(
            render(branch, &mut context(&[], "")).expect("S omitted"),
            "COLD"
        );
        assert_eq!(
            render(branch, &mut context(&[("S", "5")], "")).expect("S given"),
            "HOT"
        );

        let loop_source = "{% for i in range(params.T|default(3)|int) %}NAME={i} {% endfor %}";
        assert_eq!(
            render(loop_source, &mut context(&[], "")).expect("T omitted"),
            "NAME=0 NAME=1 NAME=2 "
        );
    }

    /// `|float` against the operand shapes the corpus writes it on: a
    /// parenthesized sum (`printer-geeetech-A10T-A20T-2021.cfg:210`), a bare
    /// name inside arithmetic (`printer-anycubic-4maxpro-2.0-2021.cfg:159`),
    /// a binary expression (`…geeetech…:200`, `e0 / (…) | float`), and an
    /// attribute (`sample-macros.cfg:286`). The filter binds tighter than
    /// `*`/`/`, as in Jinja, so it takes the name, not the quotient.
    #[test]
    fn float_binds_like_jinja_inside_arithmetic() {
        let mut bound = context(&[], "");
        bound.insert("e0", Rt::Json(json!(1.0)));
        assert_eq!(
            render("{(e0+0.000001)|float}", &mut bound).expect("renders"),
            "1.000001"
        );
        assert_eq!(
            render("{e0 / (e0 + 1) | float}", &mut bound).expect("renders"),
            "0.5"
        );
        let quotient = "{% set S = params.S|default(2)|float %}{ 1.0 / S | float }";
        assert_eq!(
            render(quotient, &mut context(&[], "")).expect("renders"),
            "0.5"
        );

        bound.insert("pot", Rt::Json(json!({ "scale": 0.5 })));
        assert_eq!(
            render("{pot.scale|float}", &mut bound).expect("renders"),
            "0.5"
        );
    }

    /// The argument shapes the filters take: Jinja2's optional default
    /// (`|int(0)`, `|float(0.25)` — the short spelling of the corpus'
    /// `|default(0)|int`), and `default`'s own signature
    /// (`default(value, default_value='', boolean=False)`) including the
    /// keyword spelling the old tokenizer refused for want of `=`. What stays
    /// refused is an argument the filter has no room for.
    #[test]
    fn filter_arguments_are_jinjas_own_shapes() {
        // `|int(7)` holds a value the conversion cannot take, while the same
        // body without the default fails loudly (last assertion).
        assert_eq!(
            render("{params.S|int(7)}", &mut context(&[("S", "oops")], "")).expect("default"),
            "7"
        );

        // More than the filter's own arguments is still refused.
        let error = render("{params.S|float(1, 2)}", &mut context(&[], "")).expect_err("arity");
        assert!(error.to_string().contains("too many arguments"), "{error}");
        let error = render("{ [1, 5]|max(1) }", &mut context(&[], "")).expect_err("arity");
        assert!(error.to_string().contains("too many arguments"), "{error}");

        // `default`'s own signature, as Jinja2 writes it: without an argument
        // the fallback is `''`, a second positional argument is the lax flag,
        // and both the keyword spelling and the falsy-counts-as-undefined rule
        // hold.
        assert_eq!(
            render("{params.S|default}", &mut context(&[], "")).expect("no argument"),
            ""
        );
        assert_eq!(
            render("{params.S|default(1, 2)}", &mut context(&[], "")).expect("second argument"),
            "1"
        );
        assert_eq!(
            render("{params.S|default(0, boolean=True)}", &mut context(&[], ""))
                .expect("keyword argument"),
            "0"
        );
        assert_eq!(
            render("{''|default('fallback', true)}", &mut context(&[], ""))
                .expect("falsy counts as undefined"),
            "fallback"
        );

        // `int`'s type refusal is unchanged, and names the filter's own rule.
        let error = render("{params.S|int}", &mut context(&[("S", "oops")], "")).expect_err("type");
        assert!(
            error.to_string().contains("invalid literal for int()"),
            "{error}"
        );
    }

    /// The corpus macros these filters come from, loaded cell-for-cell from
    /// their config files: the `M300` tones of
    /// `printer-velleman-k8800-2017.cfg:124-130` and
    /// `printer-sunlu-t3-2022.cfg:185-197`.
    #[test]
    fn the_corpus_m300_templates_load_and_render() {
        let velleman = "    # Use a default 1kHz tone if S is omitted.\n    \
                        {% set S = params.S|default(1000.0)|float %}\n    \
                        # Use a 10ms duration is P is omitted.\n    \
                        {% set P = params.P|default(100)|int %}\n    \
                        SET_PIN PIN=BEEPER VALUE=50 CYCLE_TIME={ 1.0 / S }\n    \
                        G4 P{P}\n    SET_PIN PIN=BEEPER VALUE=0\n";
        let rendered = render(velleman, &mut context(&[], "")).expect("the macro loads");
        assert!(
            rendered.contains("SET_PIN PIN=BEEPER VALUE=50 CYCLE_TIME=0.001"),
            "{rendered}"
        );
        assert!(rendered.contains("G4 P100"), "{rendered}");

        let sunlu = "  {% set S = params.S|default(1000)|int %} ; S sets the tone frequency\n  \
                     {% set P = params.P|default(100)|int %} ; P sets the tone duration\n  \
                     {% set L = 0.5 %} ; L varies the PWM on time\n  \
                     {% if S <= 0 %} ; dont divide through zero\n  \
                     {% set F = 1 %}\n  {% set L = 0 %}\n  \
                     {% elif S >= 10000 %} ;max frequency set to 10kHz\n  \
                     {% set F = 0 %}\n  {% else %}\n  \
                     {% set F = 1/S %} ;convert frequency to seconds\n  {% endif %}\n    \
                     SET_PIN PIN=beeper VALUE={L} CYCLE_TIME={F} ;Play tone\n  \
                     G4 P{P} ;tone duration\n";
        let rendered = render(sunlu, &mut context(&[], "")).expect("the macro loads");
        assert!(
            rendered.contains("SET_PIN PIN=beeper VALUE=0.5 CYCLE_TIME=0.001"),
            "{rendered}"
        );
        // `S=1000` takes the `else` branch; a `S` above the cap takes `elif`.
        let loud = "{% set S = params.S|default(1000)|int %}\
                    {% if S <= 0 %}zero{% elif S >= 10000 %}capped{% else %}{F}{% endif %}";
        assert_eq!(
            render(loud, &mut context(&[("S", "20000")], "")).expect("renders"),
            "capped"
        );
    }

    /// List literals and `|min`/`|max` (`generic_cartesian_iqex.cfg:286-287`,
    /// `generic_cartesian_itex.cfg:231,257`), in each position the corpus and
    /// its neighbours put them: the right-hand side of a `{% set %}`, a
    /// rendered expression, an `in` membership test, a subscript, a `{% for %}`
    /// iterable, and chains into the filters this port already has.
    #[test]
    fn list_literals_and_min_max_follow_the_corpus_forms() {
        // `{% set x_max = [a, b]|min %}` — the corpus sentence, its elements
        // read from the context.
        let mut bound = context(&[], "");
        bound.insert("a", Rt::Json(json!(300.0)));
        bound.insert("b", Rt::Json(json!(120.0)));
        assert_eq!(
            render("{% set x_max = [a, b]|min %}{ x_max }", &mut bound).expect("renders"),
            "120.0"
        );

        // A rendered literal reaches the formatter the way Python's list repr
        // does; filtered, it is the extreme item — floats keep their spelling.
        assert_eq!(ok("[1, 5, 3]"), "[1, 5, 3]");
        assert_eq!(ok("{ [1, 5, 3]|max }"), "5");
        assert_eq!(ok("{ [1.0, 2]|min }"), "1.0");
        assert_eq!(ok("{ [3, 1]|max }"), "3");

        // Elements are whole expressions — filters included — and an empty
        // literal is an empty list.
        assert_eq!(
            render("{ [params.L|int, 4]|max }", &mut context(&[("L", "7")], "")).expect("renders"),
            "7"
        );
        assert_eq!(ok("{ [] }"), "[]");

        // What this port already had, now taking a list literal: `in`, a
        // subscript (the postfix chain after `[a, b]`), and `for`.
        assert_eq!(ok("{% if 3 in [1, 2, 3] %}YES{% endif %}"), "YES");
        assert_eq!(ok("{ [1, 2, 3][1] }"), "2");
        assert_eq!(ok("{% for x in [3, 4] %}{ x }{% endfor %}"), "34");

        // Chains: a `|min` value feeds arithmetic and the filters this port
        // already has, and `is defined` sees a filtered literal.
        assert_eq!(ok("{ [1, 2]|min + 1 }"), "2");
        assert_eq!(ok("{ [1, 2]|min|int }"), "1");
        assert_eq!(ok("{% if [1, 2]|max is defined %}Y{% endif %}"), "Y");

        // Jinja's default `case_sensitive=False`: the comparison folds case,
        // and the item itself is what comes back.
        assert_eq!(ok("{ [\"B\", \"a\"]|min }"), "a");
        assert_eq!(ok("{ [\"B\", \"a\"]|max }"), "B");
    }

    /// The error paths list literals and `min`/`max` add: an operand that is
    /// not a sequence, a sequence CPython cannot order, and the empty
    /// sequence. `|default` no longer catches the last one — it probes for
    /// undefined, the way Jinja2's does, not for errors.
    #[test]
    fn list_and_extreme_errors_name_what_failed() {
        // A non-sequence operand is named the way CPython names it.
        let error = render("{ 5|min }", &mut context(&[], "")).expect_err("not iterable");
        assert!(
            error.to_string().contains("cannot iterate over number"),
            "{error}"
        );

        // Mixed element types are unorderable (`min([1, "a"])` in CPython),
        // and so is a nesting the corpus never writes: this port's ordering is
        // numbers and strings.
        let error = render("{ [1, \"a\"]|min }", &mut context(&[], "")).expect_err("mixed");
        assert!(
            error
                .to_string()
                .contains("unorderable types: str and number (<)"),
            "{error}"
        );
        let error = render("{ [[1, 2], [3, 4]]|min }", &mut context(&[], "")).expect_err("nested");
        assert!(
            error
                .to_string()
                .contains("unorderable types: list and list (<)"),
            "{error}"
        );

        // The empty sequence is where this port parts with Jinja2 3.1.6:
        // `_min_or_max` returns an `Undefined` there (blank, `is defined`
        // false) and this port has no `Undefined` to return, so it fails
        // loudly — and `|default`, which probes for undefined, cannot catch
        // that.
        let error = render("{ []|min }", &mut context(&[], "")).expect_err("empty");
        assert!(
            error.to_string().contains("min() arg is an empty sequence"),
            "{error}"
        );
        let error = render("{ []|max|default(7) }", &mut context(&[], "")).expect_err("empty");
        assert!(
            error.to_string().contains("max() arg is an empty sequence"),
            "{error}"
        );

        // A literal's own load errors: an unclosed bracket.
        let error = Template::parse("gcode_macro TEST:gcode", "{ [1, 2 }").expect_err("unclosed");
        assert_eq!(
            error.to_string(),
            "Error loading template 'gcode_macro TEST:gcode'\nline 1: unexpected `}`, expected `,`"
        );
    }

    /// The two `SET_COPY_MODE` templates (`generic_cartesian_iqex.cfg:282-299`,
    /// `generic_cartesian_itex.cfg:226-256`), whole bodies, cell for cell: the
    /// list literals their `|min`/`|max` reduce, with the rest of the macro
    /// rendered off the same bindings (`x_center` feeding both `G1 X…` lines).
    #[test]
    fn set_copy_mode_templates_load_and_render() {
        let iqex = "    G90\n    \
                    {% set y_center = 0.5 * (printer.configfile.settings[\"dual_carriage carriage_gantry1_left\"].position_max + printer.configfile.settings[\"carriage carriage_gantry0_left\"].position_min) %}\n    \
                    {% set x_max = [printer.configfile.settings[\"dual_carriage carriage_t3\"].position_max, printer.configfile.settings[\"dual_carriage carriage_t1\"].position_max]|min %}\n    \
                    {% set x_min = [printer.configfile.settings[\"dual_carriage carriage_t2\"].position_min, printer.configfile.settings[\"carriage carriage_t0\"].position_min]|max %}\n    \
                    {% set x_center = 0.5 * (x_max + x_min) %}\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry0_left\n    \
                    G1 Y{printer.configfile.settings[\"carriage carriage_gantry0_left\"].position_min} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry1_left\n    \
                    G1 Y{y_center} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t2\n    \
                    G1 X{printer.configfile.settings[\"dual_carriage carriage_t2\"].position_min} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t0\n    \
                    G1 X{printer.configfile.settings[\"carriage carriage_t0\"].position_min} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t3\n    \
                    G1 X{x_center} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t1\n    \
                    G1 X{x_center} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t0 MODE=PRIMARY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t1 MODE=COPY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t2 MODE=COPY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t3 MODE=COPY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry0_left MODE=PRIMARY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry1_left MODE=COPY\n    \
                    ACTIVATE_EXTRUDER EXTRUDER=extruder\n    \
                    SYNC_EXTRUDER_MOTION EXTRUDER=extruder1 MOTION_QUEUE=extruder\n    \
                    SYNC_EXTRUDER_MOTION EXTRUDER=extruder2 MOTION_QUEUE=extruder\n    \
                    SYNC_EXTRUDER_MOTION EXTRUDER=extruder3 MOTION_QUEUE=extruder\n";
        let rendered = render(iqex, &mut copy_mode_context()).expect("the macro loads");
        // `y_center` = 0.5*(200 + 0), `x_max` = min(300, 120), `x_min` =
        // max(10, 5), so `x_center` = 0.5*(120 + 10) on both `G1 X…` lines.
        assert!(rendered.contains("G1 Y0.0 F12000"), "{rendered}");
        assert!(rendered.contains("G1 Y100.0 F12000"), "{rendered}");
        assert!(rendered.contains("G1 X10.0 F12000"), "{rendered}");
        assert!(rendered.contains("G1 X5.0 F12000"), "{rendered}");
        assert_eq!(rendered.matches("G1 X65.0 F12000").count(), 2, "{rendered}");
        assert!(
            rendered.contains("SYNC_EXTRUDER_MOTION EXTRUDER=extruder3 MOTION_QUEUE=extruder"),
            "{rendered}"
        );

        let itex = "    G90\n    \
                    {% set y_center = 0.5 * (printer.configfile.settings[\"dual_carriage carriage_gantry1\"].position_max + printer.configfile.settings[\"carriage carriage_gantry0_left\"].position_min) %}\n    \
                    {% set x_max = printer.configfile.settings[\"dual_carriage carriage_t1\"].position_max %}\n    \
                    {% set x_min = [printer.configfile.settings[\"dual_carriage carriage_t2\"].position_min, printer.configfile.settings[\"carriage carriage_t0\"].position_min]|max %}\n    \
                    {% set x_center = 0.5 * (x_max + x_min) %}\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry0_left\n    \
                    G1 Y{printer.configfile.settings[\"carriage carriage_gantry0_left\"].position_min} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry1\n    \
                    G1 Y{y_center} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t2\n    \
                    G1 X{printer.configfile.settings[\"dual_carriage carriage_t2\"].position_min} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t0\n    \
                    G1 X{printer.configfile.settings[\"carriage carriage_t0\"].position_min} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t1\n    \
                    G1 X{x_center} F12000\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t0 MODE=PRIMARY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t1 MODE=COPY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_t2 MODE=COPY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry0_left MODE=PRIMARY\n    \
                    SET_DUAL_CARRIAGE CARRIAGE=carriage_gantry1 MODE=COPY\n    \
                    ACTIVATE_EXTRUDER EXTRUDER=extruder\n    \
                    SYNC_EXTRUDER_MOTION EXTRUDER=extruder1 MOTION_QUEUE=extruder\n    \
                    SYNC_EXTRUDER_MOTION EXTRUDER=extruder2 MOTION_QUEUE=extruder\n";
        let rendered = render(itex, &mut copy_mode_context()).expect("the macro loads");
        assert!(rendered.contains("G1 Y100.0 F12000"), "{rendered}");
        assert_eq!(rendered.matches("G1 X65.0 F12000").count(), 1, "{rendered}");
        assert!(
            rendered.contains("SYNC_EXTRUDER_MOTION EXTRUDER=extruder2 MOTION_QUEUE=extruder"),
            "{rendered}"
        );
    }

    /// The `printer.configfile.settings` view both `SET_COPY_MODE` macros read,
    /// one `position_min`/`position_max` per carriage (floats, as the config
    /// parser hands them over).
    fn copy_mode_context() -> Context {
        let mut context = context(&[], "");
        context.insert(
            "printer",
            Rt::Json(json!({"configfile": {"settings": {
                "dual_carriage carriage_gantry1_left": {"position_max": 200.0},
                "dual_carriage carriage_gantry1": {"position_max": 200.0},
                "carriage carriage_gantry0_left": {"position_min": 0.0},
                "dual_carriage carriage_t3": {"position_max": 300.0},
                "dual_carriage carriage_t1": {"position_max": 120.0},
                "dual_carriage carriage_t2": {"position_min": 10.0},
                "carriage carriage_t0": {"position_min": 5.0},
            }}})),
        );
        context
    }

    /// An error names the line it happened on, counting from 1, for both
    /// frames.
    #[test]
    fn errors_name_the_line_they_happened_on() {
        let source = "line one\n{% if 1 %}\n{ nope }\n{% endif %}\n";
        let error = render(source, &mut context(&[], "")).expect_err("undefined");
        assert_eq!(
            error.to_string(),
            "Error evaluating 'gcode_macro TEST:gcode': line 3: undefined value"
        );

        let error = Template::parse("gcode_macro LINES:gcode", "one\n{% foo %}\n")
            .expect_err("unknown statement");
        assert_eq!(
            error.to_string(),
            "Error loading template 'gcode_macro LINES:gcode'\nline 2: unknown statement foo"
        );
    }

    /// Three stepper macro bodies as literal test cases: `phases` (an
    /// `if`/`elif` chain over `params`), `release`, and `stepper_move`, which
    /// needs `namespace(...)` and `{% set count.phase = … %}` — the shape
    /// whose absence from the old subset is what moved this module onto
    /// minijinja. Between them they cover `namespace(phase=0)`,
    /// `{% set count.phase = … %}`, the `if`/`elif` chain, `for` + `range`,
    /// the rendered `G4 P` delay values, and both `DIR` directions.
    ///
    /// The bodies carry no `#` comment lines: the config parser strips them
    /// before a `gcode:` value reaches this engine (`config/mod.rs`), and a
    /// `#` line that did reach the gcode dispatcher would answer
    /// `Unknown command` (`gcode.rs`'s `parse_line` strips `;` only).
    #[test]
    fn the_stepper_config_macros_load_and_render() {
        let phases = "\
{% set phase = params.PHASE|default(0)|int %}
{% if phase == 0 %}
SET_PIN PIN=motor_in1 VALUE=1
SET_PIN PIN=motor_in2 VALUE=0
SET_PIN PIN=motor_in3 VALUE=0
SET_PIN PIN=motor_in4 VALUE=0
{% elif phase == 1 %}
SET_PIN PIN=motor_in1 VALUE=1
SET_PIN PIN=motor_in2 VALUE=1
SET_PIN PIN=motor_in3 VALUE=0
SET_PIN PIN=motor_in4 VALUE=0
{% elif phase == 2 %}
SET_PIN PIN=motor_in1 VALUE=0
SET_PIN PIN=motor_in2 VALUE=1
SET_PIN PIN=motor_in3 VALUE=0
SET_PIN PIN=motor_in4 VALUE=0
{% elif phase == 3 %}
SET_PIN PIN=motor_in1 VALUE=0
SET_PIN PIN=motor_in2 VALUE=1
SET_PIN PIN=motor_in3 VALUE=1
SET_PIN PIN=motor_in4 VALUE=0
{% elif phase == 4 %}
SET_PIN PIN=motor_in1 VALUE=0
SET_PIN PIN=motor_in2 VALUE=0
SET_PIN PIN=motor_in3 VALUE=1
SET_PIN PIN=motor_in4 VALUE=0
{% elif phase == 5 %}
SET_PIN PIN=motor_in1 VALUE=0
SET_PIN PIN=motor_in2 VALUE=0
SET_PIN PIN=motor_in3 VALUE=1
SET_PIN PIN=motor_in4 VALUE=1
{% elif phase == 6 %}
SET_PIN PIN=motor_in1 VALUE=0
SET_PIN PIN=motor_in2 VALUE=0
SET_PIN PIN=motor_in3 VALUE=0
SET_PIN PIN=motor_in4 VALUE=1
{% elif phase == 7 %}
SET_PIN PIN=motor_in1 VALUE=1
SET_PIN PIN=motor_in2 VALUE=0
SET_PIN PIN=motor_in3 VALUE=0
SET_PIN PIN=motor_in4 VALUE=1
{% endif %}";
        // The text between the tags survives: the two newlines after
        // `{% set %}` and `{% if %}` open the branch.
        assert_eq!(
            render(phases, &mut context(&[("PHASE", "1")], "")).expect("phase 1"),
            "\n\nSET_PIN PIN=motor_in1 VALUE=1\nSET_PIN PIN=motor_in2 VALUE=1\n\
             SET_PIN PIN=motor_in3 VALUE=0\nSET_PIN PIN=motor_in4 VALUE=0\n"
        );
        assert_eq!(
            render(phases, &mut context(&[("PHASE", "7")], "")).expect("phase 7"),
            "\n\nSET_PIN PIN=motor_in1 VALUE=1\nSET_PIN PIN=motor_in2 VALUE=0\n\
             SET_PIN PIN=motor_in3 VALUE=0\nSET_PIN PIN=motor_in4 VALUE=1\n"
        );
        // `default(0)` takes the first branch, and a phase no branch names
        // renders only the lead-in the `{% set %}` left.
        assert_eq!(
            render(phases, &mut context(&[], "")).expect("phase 0 by default"),
            "\n\nSET_PIN PIN=motor_in1 VALUE=1\nSET_PIN PIN=motor_in2 VALUE=0\n\
             SET_PIN PIN=motor_in3 VALUE=0\nSET_PIN PIN=motor_in4 VALUE=0\n"
        );
        assert_eq!(
            render(phases, &mut context(&[("PHASE", "9")], "")).expect("no branch"),
            "\n"
        );

        let release = "\
SET_PIN PIN=motor_in1 VALUE=0
SET_PIN PIN=motor_in2 VALUE=0
SET_PIN PIN=motor_in3 VALUE=0
SET_PIN PIN=motor_in4 VALUE=0";
        assert_eq!(
            render(release, &mut context(&[], "")).expect("release"),
            release
        );

        let stepper_move = "\
{% set steps = params.STEPS|default(100)|int %}
{% set dir = params.DIR|default(1)|int %}
{% set delay = params.DELAY|default(0.002)|float %}

{% set count = namespace(phase=0) %}

{% for i in range(steps) %}
{% if dir == 1 %}
{% set count.phase = (i % 8) %}
{% else %}
{% set count.phase = (7 - (i % 8)) %}
{% endif %}

_STEPPER_SET_PHASE PHASE={count.phase}

G4 P{ (delay * 1000)|int }
{% endfor %}

STEPPER_RELEASE";
        let forward = render(
            stepper_move,
            &mut context(&[("STEPS", "8"), ("DIR", "1"), ("DELAY", "0.002")], ""),
        )
        .expect("the clockwise move renders");
        assert!(forward.contains("_STEPPER_SET_PHASE PHASE=0"), "{forward}");
        assert!(forward.contains("_STEPPER_SET_PHASE PHASE=7"), "{forward}");
        assert!(
            forward.find("_STEPPER_SET_PHASE PHASE=0").expect("first")
                < forward.find("_STEPPER_SET_PHASE PHASE=1").expect("then"),
            "{forward}"
        );
        assert_eq!(forward.matches("G4 P2").count(), 8, "{forward}");
        assert!(forward.ends_with("STEPPER_RELEASE"), "{forward}");

        let backward = render(
            stepper_move,
            &mut context(&[("STEPS", "8"), ("DIR", "0"), ("DELAY", "0.005")], ""),
        )
        .expect("the counter-clockwise move renders");
        assert!(
            backward.contains("_STEPPER_SET_PHASE PHASE=7"),
            "{backward}"
        );
        assert!(
            backward.find("_STEPPER_SET_PHASE PHASE=7").expect("first")
                < backward.find("_STEPPER_SET_PHASE PHASE=6").expect("then"),
            "{backward}"
        );
        assert_eq!(backward.matches("G4 P5").count(), 8, "{backward}");
        assert_eq!(backward.matches("G4 P5\n").count(), 8, "{backward}");
    }
}
