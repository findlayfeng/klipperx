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
//! | output formatter | Python's `str` ([`format_output`]) | a tag renders `str(value)`, so a container prints its elements' `repr` (`{'call': 'PAUSE'}`, `['a', 1]`); the engine's own formatter prints them with Rust's `Debug` (`{call: PAUSE}`) |
//! | filters | the engine's, with `int`, `float`, `min`, `max` replaced | the corpus spells its coercions `params.S\|default(1000.0)\|float` and folds lists with `\|min`/`\|max` |
//! | globals | the engine's, with `range` and `namespace` bound explicitly | `range(...)` (`exclude_object.cfg:92`) and `namespace(...)`, which `gcode_macro` bodies call — the corpus spells `range`, this module's stepper-macro test binds `namespace` |
//! | unknown method callback | Python's `str` and `list` methods ([`unknown_method`]) | the engine has no method table for a plain string or list, so `client.runout_sensor.split(" ")[1]` (`mainsail.cfg`'s `RESUME`, on every resume) would fail as `unknown method` — the callback answers it; see the deviation table's `append`/`extend` row |
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
//!   and `'<name>' in printer` asks whether that object is registered **and**
//!   reports a status (`GetStatusWrapper.__contains__`, `:33-37`, through a
//!   `__getitem__` that raises `KeyError` unless `hasattr(po, 'get_status')`,
//!   `:24-26`) — the engine's containment check on a map object *is* its
//!   lookup, so a name that fails either test is absent, not an error.
//!   Iterating the view walks the object names, as upstream's wrapper does
//!   ([`PrinterView::enumerate`], `GetStatusWrapper.__iter__`,
//!   `gcode_macro.py:40-43`): `{% for name in printer %}` yields the names
//!   `objects/list` reports, in registration order, and reads no status.
//! - `printer.objects` is this port's map-shaped spelling of that same
//!   reflection: an enumerable map from object name to status, holding the
//!   objects `objects/list` reports ([`Printer::queryable_objects`]) in
//!   registration order. Each lookup is the cached status `printer['<name>']`
//!   returns, and a name that is unregistered or reports no status is absent —
//!   Strict then fails the render if the body uses it. It is an extension
//!   (upstream's `GetStatusWrapper` has no `objects` key; a body ported from
//!   upstream writes `{% for name in printer %}`), the name the TODO tracks the
//!   capability under.
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
//! | `printer.objects` | `Undefined`: the template's `printer` is a `GetStatusWrapper`, which has no `objects` key (the reflection upstream is iteration, `{% for name in printer %}`) | this port adds the key as the same reflection in map shape: the names `{% for %}` walks, with each object's cached status under it, so a body may read `printer.objects['mcu'].<status key>` or hold the view; `\|length` counts them on both spellings (upstream's wrapper has no `__len__`) |
//! | trailing newline | one newline dropped | kept (`set_keep_trailing_newline(true)`), because `idle_timeout`'s default script and the corpus' bodies are written to keep theirs |
//! | `{-3.5}` | `3.5`: `-` right after the opening brace is the whitespace-control marker (`{{-`'s, with single-brace delimiters), which swallows the sign — measured against `jinja2.Environment('{%','%}','{','}')` | same; a negative literal after `{` needs the space (`{ -3.5}`), and no corpus body writes `{-` |
//! | a container joined with `~` | `str()` of it, elements `repr`-quoted: `'x' ~ {'call': 'PAUSE'}` is `x{'call': 'PAUSE'}` | the engine's `Display`, elements unquoted: `'x' ~ {'call': 'PAUSE'}` is `x{call: PAUSE}` — `~` joins through `Display` before the formatter sees a string, so only a whole tag (`{ … }`) gets Python's spelling |
//! | printing a *silent* undefined (`{x if c}`, which has no `else`) | blank | an `undefined value` error: the engine's own formatter lets *that* one print nothing (it tells the two kinds of undefined apart), a custom formatter does not — `ValueKind::Undefined` is one kind — so the strict check [`format_output`] repeats covers both; a *default* undefined (`{nope}`) is the error it always was |
//! | `"…%s…" % value` (a **string literal** directly left of `%`) | Python's printf formatting | rewritten at compile time into the engine's `\|format` filter, which is the same printf formatting with its arguments passed separately: `{"Restoring \"%s\"" % x}` → `{"Restoring \"%s\""\|format(x)}` ([`rewrite_printf_formatting`]). Both tag kinds carry expressions, so `{ … }` and `{% … %}` are both rewritten; a `{# … #}` comment is not. Narrow on purpose: the literal must be alone on the left and its `%` must end an operand position — any other `%` stays the remainder operator, and a formatting this rewrite cannot read fails the load the way it did before rather than mis-rendering |
//! | `"%(key)s" % {'key': v}` (a mapping operand) | formatted from the mapping | unsupported: the operand right of `%` is a dict literal this rewrite does not read, so the tag keeps its Python spelling and fails the load (`%` is the remainder, which a string left operand cannot take); `\|format` reads a mapping only when the whole operand is one map value |
//! | a method that mutates a **list** (`append`, `extend`, `pop`) | appends/extends/pops in place | `None`, and the list is unchanged: the engine's lists are **values**, so nothing can write back to the list a body holds. A body that collects into one (`{% set _d = prompt_txt.append(…) %}`, `mainsail.cfg`'s abort prompt) therefore reads an **empty** list afterwards — one `debug!` line per process records that the degradation happened. The read-only methods (`index`, `count`) are Python's own |
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

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use minijinja::syntax::SyntaxConfig;
use minijinja::value::{Enumerator, Object, ObjectRepr, Rest, Value as MjValue, ValueKind};
use minijinja::{
    escape_formatter, AutoEscape, Environment, Error as MjError, ErrorKind, Output, State,
    UndefinedBehavior::Strict,
};
use serde_json::Value;
use tracing::debug;

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
/// Iterating the view yields the names `objects/list` reports
/// (`GetStatusWrapper.__iter__`, `gcode_macro.py:40-43`), and
/// [`ObjectsView`] — `printer.objects` — is the same list as a map.
///
/// The cache is a `Mutex` because the engine requires its objects to be `Sync`;
/// the view is `Clone` so a [`Rt`] can be cloned into a render.
#[derive(Clone)]
pub struct PrinterView {
    printer: Arc<Printer>,
    cache: Arc<Mutex<HashMap<String, MjValue>>>,
    /// The names iteration walks, snapshotted once per view so `printer` and
    /// `printer.objects` hand out the *same* list.
    names: Arc<OnceLock<Arc<Vec<String>>>>,
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
            names: Arc::new(OnceLock::new()),
        }
    }

    /// One object's status (`GetStatusWrapper.__getitem__`, `:19-32`), as the
    /// templates read it: `None` for a name nobody registered **and** for a
    /// part that reports no status, which is what makes `'name' in printer`
    /// upstream's `<name> is queryable` test — its `__getitem__` raises
    /// `KeyError` unless `hasattr(po, 'get_status')` (`:24-26`). The object is
    /// asked for status after the registry lock is released, as
    /// [`Printer::status_of`] does.
    fn status(&self, name: &str) -> Option<MjValue> {
        if let Some(cached) = self.lock().get(name) {
            return Some(cached.clone());
        }
        let object = self.printer.lookup_object(name)?;
        if !object.is_queryable() {
            return None;
        }
        let status = status_value(&object.get_status(self.printer.eventtime()));
        self.lock().insert(name.to_string(), status.clone());
        Some(status)
    }

    fn lock(&self) -> MutexGuard<'_, HashMap<String, MjValue>> {
        self.cache
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The queryable names, in registration order — the list `objects/list`
    /// reports ([`Printer::queryable_objects`]), snapshotted on first use so
    /// every spelling of the reflection walks one list.
    fn names(&self) -> Arc<Vec<String>> {
        Arc::clone(
            self.names
                .get_or_init(|| Arc::new(self.printer.queryable_objects())),
        )
    }

    /// The enumerator `{% for name in printer %}` and `printer.objects` both
    /// walk: the names as the engine's values, so a loop yields strings.
    fn names_enumerator(&self) -> Enumerator {
        Enumerator::Values(self.names().iter().cloned().map(MjValue::from).collect())
    }

    /// The reflection view `printer.objects` reads: this view again (one
    /// cache and one name list for both spellings).
    fn objects(&self) -> ObjectsView {
        ObjectsView {
            printer: self.clone(),
            names: self.names(),
        }
    }
}

impl Object for PrinterView {
    fn repr(self: &Arc<Self>) -> ObjectRepr {
        ObjectRepr::Map
    }

    fn get_value(self: &Arc<Self>, key: &MjValue) -> Option<MjValue> {
        let name = key.as_str()?;
        // The reflection view `printer.objects` — this port's convenience
        // spelling of what iteration already yields. Reserved on purpose: no
        // machine registers a part under this name, and a body that reads it
        // wants the reflection, not a lookup.
        if name == OBJECTS_KEY {
            return Some(MjValue::from_object(self.objects()));
        }
        self.status(name)
    }

    /// Enumerable the way upstream's wrapper is: `GetStatusWrapper.__iter__`
    /// (`gcode_macro.py:40-43`) yields every registered name it can look up a
    /// status for, in registration order — `{% for name in printer %}`. Here
    /// that is [`Printer::queryable_objects`], the same list `objects/list`
    /// reports and [`ObjectsView`] holds. Iterating reads no status.
    fn enumerate(self: &Arc<Self>) -> Enumerator {
        self.names_enumerator()
    }

    /// Always true, as upstream's wrapper is: it defines no `__len__`/`__bool__`,
    /// so a machine that has registered nothing queryable yet is still truthy
    /// (the enumerable view's own length would say `0`).
    fn is_true(self: &Arc<Self>) -> bool {
        true
    }
}

/// The key a body reads the reflection view under: `printer.objects`.
const OBJECTS_KEY: &str = "objects";

/// `printer.objects` — this port's map-shaped view of the reflection.
///
/// Upstream spells the reflection as iteration over the template's `printer`
/// itself (`GetStatusWrapper.__iter__`, `gcode_macro.py:40-43`), and
/// `objects/list` reports only the parts that answer a status
/// ([`Printer::queryable_objects`], upstream's `hasattr(o, 'get_status')`
/// filter). This view is the same list under a name a body can hold: a map
/// from object name to status, in **registration order** — the order
/// `objects/list`, upstream's `OrderedDict` and `printer`'s own iteration use.
///
/// The names are the very list [`PrinterView`] iterates, snapshotted once per
/// render view, and the statuses go through the render's [`PrinterView`]
/// cache: `printer.objects['mcu']`, `printer['mcu']` and `printer.mcu` answer
/// the same value, and `get_status` runs at most once per object per render.
/// Both are what keeps a render off the reactor: nothing here blocks, and
/// nothing is recomputed per read. Iterating either spelling reads no status
/// at all.
#[derive(Clone)]
pub struct ObjectsView {
    /// The view the lookups go through, shared with the body's `printer` so
    /// both spellings hit one cache.
    printer: PrinterView,
    /// The names this view exposes, in registration order.
    names: Arc<Vec<String>>,
}

impl fmt::Debug for ObjectsView {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `Printer` has no `Debug`; the names are the interesting part.
        f.debug_struct("ObjectsView")
            .field("names", &self.names)
            .finish()
    }
}

impl Object for ObjectsView {
    fn repr(self: &Arc<Self>) -> ObjectRepr {
        ObjectRepr::Map
    }

    /// One object's status. `None` for a name that is unregistered or reports
    /// no status: Strict turns that into an `undefined value` error when the
    /// body prints, iterates or tests it, and `'<name>' in printer.objects`
    /// is `false` — the same quiet probe `printer` itself answers.
    fn get_value(self: &Arc<Self>, key: &MjValue) -> Option<MjValue> {
        let name = key.as_str()?;
        self.names
            .iter()
            .any(|known| known == name)
            .then(|| self.printer.status(name))
            .flatten()
    }

    /// `{% for name in printer.objects %}` walks the names, as a Jinja2 dict
    /// iterates its keys; `|items` / `|dictsort` read the pairs through
    /// [`ObjectsView::get_value`]. The length is known, so `|length` works.
    fn enumerate(self: &Arc<Self>) -> Enumerator {
        self.printer.names_enumerator()
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
    /// Python's string `%` formatting is rewritten into the engine's
    /// `\|format` filter first ([`rewrite_printf_formatting`]) — in a `{ … }`
    /// expression tag and in a `{% … %}` statement tag alike. The rewrite adds
    /// and removes no newline, so a load error still names the line the body
    /// wrote.
    ///
    /// # Errors
    /// A syntax error in the body: an unclosed tag, a statement the engine
    /// does not know, an unbalanced block, a malformed expression.
    pub fn parse(name: &str, source: &str) -> Result<Self, TemplateError> {
        let mut env = environment();
        let source = rewrite_printf_formatting(source);
        env.add_template_owned(name.to_string(), source.into_owned())
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

// ===========================================================================
// Python's `%` formatting
// ===========================================================================

/// Rewrite Python's string formatting into the engine's `\|format` filter.
///
/// Upstream runs Jinja2, where a string on the left of `%` is Python's printf
/// formatting (`"Restoring \"%s\"" % printer.toolhead.extruder`); this engine's
/// `%` is the remainder operator and refuses a string operand outright. Its
/// `format` filter is printf formatting with the arguments passed separately
/// (`"%s, %s!"\|format(a, b)`), so such a formatting is rewritten into that call
/// — the eleven in `mainsail.cfg`, eight inside a `{ … }` tag and three inside
/// a `{% set %}` (`145`, `152`, `163`).
///
/// Both tag kinds carry expressions, so both are rewritten; the rewrite is
/// narrow on purpose, so every other `%` keeps the engine's own meaning:
///
/// - `{ … }` expression tags and `{% … %}` statement tags are rewritten; a
///   `{# … #}` comment and everything between tags are copied byte for byte.
///   The `%` of a `{%` opener and of a `%}` closer is a delimiter, never an
///   operator, because a body starts after the one and ends before the other;
/// - the operand left of `%` must be **exactly one string literal** (single or
///   double quoted, escapes included) with only whitespace between it and the
///   `%`, and it must not itself be the right operand of something else;
///   otherwise the `%` stays the remainder operator — `{% set x = 100 %}` and
///   `{% if n % 2 == 0 %}` come through untouched;
/// - the operand right of it must end an operand position (the body's end, a
///   `,` `)` `]` `}` or `|`); a formatting this rewrite cannot read (an
///   unbalanced bracket, a dict operand, an operator binding looser than a
///   filter) is left as it is and fails the load the way it does today — never
///   silently mis-rendered;
/// - that operand becomes the filter's arguments: one parenthesized group
///   whose top level holds commas is a tuple (`(a, b)` → `format(a, b)`), one
///   without is a single argument (`(a)` → `format(a)`), and any other operand
///   is taken whole, trailers included (`(xs.split(" "))[1]` is one argument);
/// - no newline is added or removed, so an error still reports the line number
///   the body wrote.
fn rewrite_printf_formatting(source: &str) -> Cow<'_, str> {
    let bytes = source.as_bytes();
    let mut rewritten = String::with_capacity(source.len());
    let mut kept = 0;
    let mut changed = false;
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] != b'{' {
            i += 1;
            continue;
        }
        let (body_start, body_end, after) = match bytes.get(i + 1).copied() {
            // A statement tag: its body is what `{%` and `%}` enclose.
            Some(b'%') => {
                let Some(close) = find_bytes(bytes, b"%}", i + 2) else {
                    break;
                };
                (i + 2, close, close + 2)
            }
            // A comment tag: copied verbatim, end or no end.
            Some(b'#') => {
                let Some(close) = find_bytes(bytes, b"#}", i + 2) else {
                    break;
                };
                i = close + 2;
                continue;
            }
            // An expression tag, whose `}` a dict literal or a `split("}")`
            // inside it does not end early.
            _ => {
                let Some(close) = tag_end(bytes, i + 1) else {
                    break;
                };
                (i + 1, close, close + 1)
            }
        };
        if let Some(body) = rewrite_tag_body(&source[body_start..body_end]) {
            rewritten.push_str(&source[kept..body_start]);
            rewritten.push_str(&body);
            kept = body_end;
            changed = true;
        }
        i = after;
    }
    if !changed {
        return Cow::Borrowed(source);
    }
    rewritten.push_str(&source[kept..]);
    Cow::Owned(rewritten)
}

/// One tag body with every `"…" % operand` in it rewritten, or `None` when it
/// holds no formatting this rewrite reads.
fn rewrite_tag_body(body: &str) -> Option<String> {
    let bytes = body.as_bytes();
    let mut rewritten: Option<String> = None;
    let mut kept = 0;
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            // A string literal's own characters are never structure: not its
            // brackets, and not its `%`.
            b'\'' | b'"' => i = string_close(bytes, i)?,
            b'%' => {
                // A `%` this rewrite cannot read stays the remainder it is.
                let Some((start, call, end)) = printf_call(body, i) else {
                    i += 1;
                    continue;
                };
                let out = rewritten.get_or_insert_with(String::new);
                out.push_str(&body[kept..start]);
                out.push_str(&call);
                kept = end;
                i = end;
                continue;
            }
            _ => {}
        }
        i += 1;
    }
    let mut rewritten = rewritten?;
    rewritten.push_str(&body[kept..]);
    Some(rewritten)
}

/// The `"…"\|format(…)` call replacing the formatting whose `%` sits at `pct`,
/// the index of the literal it starts at, and the index just past the operand
/// it replaces.
fn printf_call(body: &str, pct: usize) -> Option<(usize, String, usize)> {
    let bytes = body.as_bytes();
    let literal = string_before(bytes, pct)?;
    // The literal has to start an operand itself: right after a value or a
    // closing bracket it is the *right* operand of something, where a filter
    // would bind tighter than the operator there and change the meaning.
    if let Some(before) = bytes[..literal]
        .iter()
        .rfind(|byte| !byte.is_ascii_whitespace())
    {
        if before.is_ascii_alphanumeric()
            || matches!(before, b'_' | b'\'' | b'"' | b')' | b']' | b'}')
        {
            return None;
        }
    }
    let (arguments, end) = printf_arguments(body, pct + 1)?;
    // ... and what follows it has to end an operand position; an operator there
    // binds looser than a filter and would be rebound by the rewrite.
    if !matches!(
        bytes.get(skip_whitespace(bytes, end)).copied(),
        None | Some(b',') | Some(b')') | Some(b']') | Some(b'}') | Some(b'|')
    ) {
        return None;
    }
    Some((
        literal,
        // The literal without the whitespace the `%` was padded with.
        format!(
            "{}|format({})",
            &body[literal..string_close(bytes, literal)? + 1],
            arguments
        ),
        end,
    ))
}

/// The filter arguments of the formatting whose `%` sits just before `from`,
/// and the index just past the operand they were read from.
fn printf_arguments(body: &str, from: usize) -> Option<(String, usize)> {
    let bytes = body.as_bytes();
    let start = skip_whitespace(bytes, from);
    if bytes.get(start) == Some(&b'(') {
        let close = bracket_close(bytes, start)?;
        // A group with a trailer — `(xs.split(" "))[1]` — is one operand, not a
        // tuple: the trailer belongs to it.
        let trailered = matches!(bytes.get(close + 1).copied(), Some(b'[' | b'.' | b'('));
        if !trailered {
            let arguments = tuple_arguments(body, start + 1, close)?;
            return Some((arguments, close + 1));
        }
    }
    let end = simple_operand_end(bytes, start)?;
    Some((body[start..end].to_string(), end))
}

/// The arguments of a parenthesized group whose body is `body[from..to]`: its
/// top-level commas split them, and a group without one is one argument.
fn tuple_arguments(body: &str, from: usize, to: usize) -> Option<String> {
    let bytes = body.as_bytes();
    let mut arguments: Vec<&str> = Vec::new();
    let mut depth = 0usize;
    let mut item = from;
    let mut i = from;
    while i < to {
        match bytes[i] {
            b'\'' | b'"' => i = string_close(bytes, i)?,
            b'(' | b'[' | b'{' => depth += 1,
            b')' | b']' | b'}' => depth = depth.checked_sub(1)?,
            b',' if depth == 0 => {
                arguments.push(body[item..i].trim());
                item = i + 1;
            }
            _ => {}
        }
        i += 1;
    }
    let last = body[item..to].trim();
    // `(a,)` is Python's one-element tuple.
    if !last.is_empty() {
        arguments.push(last);
    }
    if arguments.is_empty() || arguments.iter().any(|argument| argument.is_empty()) {
        return None;
    }
    Some(arguments.join(", "))
}

/// The end of the single operand starting at `start`: a primary (a name, a
/// number, a string, a bracketed group) followed by its `.name` / `[…]` / `(…)`
/// trailers, so `(xs.split(" "))[1]` is one operand.
fn simple_operand_end(bytes: &[u8], start: usize) -> Option<usize> {
    let mut end = primary_end(bytes, start)?;
    loop {
        end = match bytes.get(end).copied() {
            Some(b'.') => name_end(bytes, end + 1)?,
            Some(b'[' | b'(') => bracket_close(bytes, end)? + 1,
            _ => return Some(end),
        };
    }
}

/// The end of a primary expression: a name, a number, a string literal or a
/// bracketed group.
fn primary_end(bytes: &[u8], start: usize) -> Option<usize> {
    match bytes.get(start).copied()? {
        b'\'' | b'"' => string_close(bytes, start).map(|close| close + 1),
        b'(' | b'[' => bracket_close(bytes, start).map(|close| close + 1),
        digit if digit.is_ascii_digit() => {
            let mut end = start;
            while bytes.get(end).is_some_and(u8::is_ascii_digit) {
                end += 1;
            }
            // A fraction, not the `.name` trailer of a name.
            if bytes.get(end) == Some(&b'.') && bytes.get(end + 1).is_some_and(u8::is_ascii_digit) {
                end += 1;
                while bytes.get(end).is_some_and(u8::is_ascii_digit) {
                    end += 1;
                }
            }
            Some(end)
        }
        name if name.is_ascii_alphabetic() || name == b'_' => name_end(bytes, start),
        _ => None,
    }
}

/// The end of the name starting at `start`.
fn name_end(bytes: &[u8], start: usize) -> Option<usize> {
    let mut end = start;
    while bytes
        .get(end)
        .is_some_and(|byte| byte.is_ascii_alphanumeric() || *byte == b'_')
    {
        end += 1;
    }
    (end > start).then_some(end)
}

/// The opening quote of the string literal that ends (whitespace aside) right
/// before `pct`, or `None` when the operand left of it is not a string literal.
fn string_before(bytes: &[u8], pct: usize) -> Option<usize> {
    let mut end = pct;
    while end > 0 && bytes[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    let quote = *bytes.get(end.checked_sub(1)?)?;
    if quote != b'\'' && quote != b'"' {
        return None;
    }
    // The closing quote is the one we landed on; the literal's opening one is
    // the nearest same quote before it that no backslash escapes.
    let mut open = end - 1;
    while open > 0 {
        open -= 1;
        if bytes[open] != quote {
            continue;
        }
        let mut escapes = 0;
        while open > escapes && bytes[open - escapes - 1] == b'\\' {
            escapes += 1;
        }
        if escapes % 2 == 0 {
            return Some(open);
        }
    }
    None
}

/// The index of the quote closing the string literal that opens at `open`, its
/// escapes counted, or `None` when it never closes.
fn string_close(bytes: &[u8], open: usize) -> Option<usize> {
    let quote = *bytes.get(open)?;
    let mut i = open + 1;
    while i < bytes.len() {
        if bytes[i] == b'\\' {
            i += 2;
        } else if bytes[i] == quote {
            return Some(i);
        } else {
            i += 1;
        }
    }
    None
}

/// The index of the `}` closing a `{ … }` tag whose body starts at `from`: the
/// first one at bracket depth zero and outside a string.
fn tag_end(bytes: &[u8], from: usize) -> Option<usize> {
    let mut depth = 0usize;
    let mut i = from;
    while i < bytes.len() {
        match bytes[i] {
            b'\'' | b'"' => i = string_close(bytes, i)?,
            b'(' | b'[' | b'{' => depth += 1,
            b')' | b']' => depth = depth.saturating_sub(1),
            b'}' if depth == 0 => return Some(i),
            b'}' => depth -= 1,
            _ => {}
        }
        i += 1;
    }
    None
}

/// The index of the bracket closing the one at `open`, strings and nesting
/// inside it accounted for.
fn bracket_close(bytes: &[u8], open: usize) -> Option<usize> {
    let mut depth = 0usize;
    let mut i = open;
    while i < bytes.len() {
        match bytes[i] {
            b'\'' | b'"' => i = string_close(bytes, i)?,
            b'(' | b'[' | b'{' => depth += 1,
            b')' | b']' | b'}' => {
                depth = depth.checked_sub(1)?;
                if depth == 0 {
                    return Some(i);
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// The first index of `needle` in `haystack` at or after `from`.
fn find_bytes(haystack: &[u8], needle: &[u8], from: usize) -> Option<usize> {
    let mut i = from;
    while i + needle.len() <= haystack.len() {
        if &haystack[i..i + needle.len()] == needle {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// The index of the first non-whitespace byte at or after `from`.
fn skip_whitespace(bytes: &[u8], from: usize) -> usize {
    let mut i = from;
    while bytes.get(i).is_some_and(u8::is_ascii_whitespace) {
        i += 1;
    }
    i
}

/// The environment every body is compiled into: upstream's delimiters,
/// undefined behavior and escaping, this port's trailing-newline rule, the four
/// filters the corpus spells differently, the two Jinja2 globals the corpus
/// (`range`) and this module's stepper-macro test (`namespace`) call, and the
/// unknown-method callback that answers `str`/`list` method calls
/// ([`unknown_method`]).
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
    env.add_filter("format", filter_format);
    env.add_function("range", minijinja::functions::range);
    env.add_function("namespace", minijinja::functions::namespace);
    env.set_formatter(format_output);
    env.set_unknown_method_callback(unknown_method);
    env
}

// ===========================================================================
// Output
// ===========================================================================

/// What a `{ … }` tag writes: Python's `str`, which is what a Jinja2 tag
/// renders (`gcode_macro.py:83`).
///
/// The engine prints a container by walking it with Rust's `Debug`
/// (`Object::render`), which leaves the elements unquoted — `{call: PAUSE}`.
/// Jinja2 prints a container with its elements' `repr`, so upstream's
/// `{params}` reads `{'call': 'PAUSE'}`, and a macro body that writes a
/// container into a g-code value (`mainsail.cfg:182`'s
/// `SET_GCODE_VARIABLE … VALUE="{…}"`) writes that text. Everything that is
/// not a container is already Python's spelling in the engine's own `Display`
/// (`True`, `None`, `3.0`), so it keeps the default formatter.
///
/// A custom formatter is also what the engine drops its strict-undefined check
/// for on printing (`Instruction::Emit` only applies it when
/// `is_default_formatter`), so the check moves here: printing an undefined
/// stays the `undefined value` error a body has always got.
fn format_output(
    out: &mut Output<'_>,
    state: &State<'_, '_>,
    value: &MjValue,
) -> Result<(), MjError> {
    if value.is_undefined() {
        return Err(MjError::from(ErrorKind::UndefinedError));
    }
    match value.kind() {
        ValueKind::Map | ValueKind::Seq => {
            out.write_str(&python_repr(value)).map_err(MjError::from)
        }
        _ => escape_formatter(out, state, value),
    }
}

/// One value the way Python's `repr` spells it — what a container's elements
/// are printed with. A number, a boolean or `None` keeps the engine's spelling
/// (already Python's, `Display`'s `3.0` / `True` / `None`); a container recurses
/// so nesting reads like Python's too.
fn python_repr(value: &MjValue) -> String {
    match value.kind() {
        ValueKind::String => python_repr_string(value.as_str().unwrap_or_default()),
        ValueKind::Seq => match value.try_iter() {
            Ok(items) => format!(
                "[{}]",
                items
                    .map(|item| python_repr(&item))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
            Err(_) => value.to_string(),
        },
        ValueKind::Map => match value.as_object().and_then(|object| object.try_iter_pairs()) {
            Some(entries) => format!(
                "{{{}}}",
                entries
                    .map(|(key, item)| format!("{}: {}", python_repr(&key), python_repr(&item)))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
            None => value.to_string(),
        },
        _ => value.to_string(),
    }
}

/// A string as Python's `repr` quotes it inside a container: single quotes,
/// with a backslash and the quote itself escaped.
fn python_repr_string(text: &str) -> String {
    let mut quoted = String::with_capacity(text.len() + 2);
    quoted.push('\'');
    for ch in text.chars() {
        if ch == '\'' || ch == '\\' {
            quoted.push('\\');
        }
        quoted.push(ch);
    }
    quoted.push('\'');
    quoted
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

/// `|format` with Python's integer conversions over floats.
///
/// The menu's input names are `{'%3d' % (menu.input*100)}%`
/// (`menu.cfg:118`): a **float** formatted with `%d`. Python truncates it
/// toward zero, while the engine refuses it outright (`'float' cannot be
/// formatted in decimal format ('d')`). Every such tag is rewritten into this
/// filter ([`rewrite_printf_formatting`]), so the coercion belongs here: the
/// argument each `%d`/`%i`/`%u`/`%o`/`%x`/`%X`/`%c` conversion consumes is
/// truncated to an integer first. An argument that is already an integer is
/// passed through untouched.
fn filter_format(
    state: &State,
    format_str: &MjValue,
    args: Rest<MjValue>,
) -> Result<MjValue, MjError> {
    let Some(text) = format_str.as_str() else {
        return minijinja::filters::format(state, format_str, args);
    };
    let mut coerced = args.0;
    for (index, conversion) in printf_conversions(text).into_iter().enumerate() {
        if !matches!(conversion, b'd' | b'i' | b'u' | b'o' | b'x' | b'X' | b'c') {
            continue;
        }
        let Some(argument) = coerced.get_mut(index) else {
            break;
        };
        // `is_integer` tells `42` from `42.0`; only the latter needs the
        // truncation Python's `%d` would have done.
        if !argument.is_number() || argument.is_integer() {
            continue;
        }
        let Ok(value) = f64::try_from(argument.clone()) else {
            continue;
        };
        *argument = MjValue::from(value.trunc() as i64);
    }
    minijinja::filters::format(state, format_str, Rest(coerced))
}

/// The conversion characters a printf format consumes arguments with, in order.
///
/// A `*` width or precision consumes one too, so it is reported as a `w` — an
/// entry that shifts the positions after it without ever being an integer
/// conversion.
fn printf_conversions(text: &str) -> Vec<u8> {
    const FLAGS_AND_WIDTH: &[u8] = b"-+ #0123456789.hlL";
    let bytes = text.as_bytes();
    let mut conversions = Vec::new();
    let mut index = 0;
    while index < bytes.len() {
        if bytes[index] != b'%' {
            index += 1;
            continue;
        }
        index += 1;
        if bytes.get(index) == Some(&b'%') {
            index += 1;
            continue;
        }
        if bytes.get(index) == Some(&b'(') {
            // A mapping operand (not a shape the rewrite produces): the
            // argument positions are not positional, so stop reading.
            break;
        }
        while let Some(&byte) = bytes.get(index) {
            if byte == b'*' {
                conversions.push(b'w');
                index += 1;
                continue;
            }
            if FLAGS_AND_WIDTH.contains(&byte) {
                index += 1;
                continue;
            }
            break;
        }
        let Some(&conversion) = bytes.get(index) else {
            break;
        };
        conversions.push(conversion);
        index += 1;
    }
    conversions
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

/// The Python type name a method's argument error uses (`replace() argument
/// 'old' must be str, not int`). Numbers are told apart as `int`/`float`,
/// which is `TypeError`'s wording — unlike [`ordering_name`], whose two-type
/// form keeps the old subset's `min`/`max` messages.
fn python_type(value: &MjValue) -> &'static str {
    match value.kind() {
        ValueKind::Number if value.as_i64().is_some() => "int",
        ValueKind::Number => "float",
        ValueKind::Bool => "bool",
        ValueKind::String => "str",
        ValueKind::None | ValueKind::Undefined => "NoneType",
        ValueKind::Seq => "list",
        ValueKind::Map => "dict",
        _ => "object",
    }
}

// ===========================================================================
// Methods
// ===========================================================================

/// The answer for a method call on a value the engine has no method table for
/// — every plain string and list, which is why a body's
/// `client.runout_sensor.split(" ")[1]` (`mainsail.cfg`'s `RESUME`, on every
/// resume) would otherwise fail as `unknown method`.
///
/// A `str` gets [`string_method`], a `list` gets [`list_method`]. Everything
/// else — another type, another name — stays the `UnknownMethod` error it was,
/// so a body still hears about a method nothing implements.
///
/// The one gap is **mutation**: the engine's lists are values, so
/// `list.append`/`extend`/`pop` cannot write back to the list the body holds
/// (see [`mixed_method`]); they answer `None` rather than fail, so the corpus'
/// `{% set _d = prompt_txt.append(…) %}` keeps rendering.
fn unknown_method(
    _state: &State,
    value: &MjValue,
    method: &str,
    args: &[MjValue],
) -> Result<MjValue, MjError> {
    match value.kind() {
        ValueKind::String => string_method(value.as_str().unwrap_or_default(), method, args),
        ValueKind::Seq => list_method(value, method, args),
        _ => Err(MjError::from(ErrorKind::UnknownMethod)),
    }
}

/// The method `method` on the string `text`, with Python's `str` semantics for
/// each name this port implements; an unimplemented name is `UnknownMethod`.
fn string_method(text: &str, method: &str, args: &[MjValue]) -> Result<MjValue, MjError> {
    match method {
        "split" => method_split(text, args),
        "strip" => strip(method, text, args, Side::Both),
        "lstrip" => strip(method, text, args, Side::Left),
        "rstrip" => strip(method, text, args, Side::Right),
        "lower" => {
            takes_no_arguments(method, args)?;
            Ok(MjValue::from(text.to_lowercase()))
        }
        "upper" => {
            takes_no_arguments(method, args)?;
            Ok(MjValue::from(text.to_uppercase()))
        }
        "replace" => method_replace(text, args),
        "startswith" => affix(method, text, args, Affix::Start),
        "endswith" => affix(method, text, args, Affix::End),
        "join" => method_join(text, args),
        "find" => method_find(text, args),
        "count" => method_count(text, args),
        _ => Err(MjError::from(ErrorKind::UnknownMethod)),
    }
}

/// The method `method` on the list `value`: the three that would mutate it are
/// answered by [`mixed_method`], the read-only ones are Python's own, and every
/// other name is `UnknownMethod`.
fn list_method(value: &MjValue, method: &str, args: &[MjValue]) -> Result<MjValue, MjError> {
    match method {
        "append" => {
            arg_count(method, args, 1, 1)?;
            Ok(mixed_method(method))
        }
        "extend" => {
            arg_count(method, args, 1, 1)?;
            // Python refuses a non-iterable argument instead of extending
            // with it, so the degradation never swallows that mistake.
            match args[0].kind() {
                ValueKind::String | ValueKind::Seq | ValueKind::Iterable | ValueKind::Map => {
                    Ok(mixed_method(method))
                }
                _ => Err(MjError::new(
                    ErrorKind::InvalidOperation,
                    format!(
                        "{method}() argument must be an iterable, not {}",
                        python_type(&args[0])
                    ),
                )),
            }
        }
        "pop" => {
            arg_count(method, args, 0, 1)?;
            Ok(mixed_method(method))
        }
        "index" => list_index(value, args),
        "count" => list_count(value, args),
        _ => Err(MjError::from(ErrorKind::UnknownMethod)),
    }
}

/// A method whose effect is a **mutation** of its receiver: this port's lists
/// are values, so there is no way to write back to the list a body holds.
///
/// `append` and `extend` return `None`, which is what Python returns, and the
/// list keeps its contents — a body that collects into a list this way
/// (`{% set _d = prompt_txt.append(…) %}`, `mainsail.cfg`'s abort prompt) gets
/// an always-empty list. `pop` returns `None` too instead of the item it could
/// not remove. One `debug!` line per process records that the degradation
/// happened, so a render is never silent about it.
fn mixed_method(method: &str) -> MjValue {
    static REPORTED: AtomicBool = AtomicBool::new(false);
    if !REPORTED.swap(true, Ordering::Relaxed) {
        debug!(
            "template: list.{}() returns None and leaves the list unchanged — this port's lists \
             are values, so a macro cannot collect into one",
            method
        );
    }
    MjValue::from(())
}

/// A method's argument count, checked the way Python's method errors spell it:
/// `takes at most N arguments (M given)` / `takes at least N positional
/// arguments (M given)`. Neither wording runs the method's body.
fn arg_count(method: &str, args: &[MjValue], min: usize, max: usize) -> Result<(), MjError> {
    if args.len() < min {
        return Err(MjError::new(
            ErrorKind::MissingArgument,
            format!(
                "{method}() takes at least {min} positional arguments ({} given)",
                args.len()
            ),
        ));
    }
    if args.len() > max {
        return Err(MjError::new(
            ErrorKind::TooManyArguments,
            format!(
                "{method}() takes at most {max} arguments ({} given)",
                args.len()
            ),
        ));
    }
    Ok(())
}

/// `str`'s own no-argument methods (`lower`, `upper`), worded as CPython words
/// them: `str.lower() takes no arguments (1 given)`.
fn takes_no_arguments(method: &str, args: &[MjValue]) -> Result<(), MjError> {
    if args.is_empty() {
        return Ok(());
    }
    Err(MjError::new(
        ErrorKind::TooManyArguments,
        format!("str.{method}() takes no arguments ({} given)", args.len()),
    ))
}

/// One argument as a string, or the `TypeError` Python raises for a value that
/// is not one.
fn as_text<'a>(method: &str, value: &'a MjValue, name: &str) -> Result<&'a str, MjError> {
    value.as_str().ok_or_else(|| {
        MjError::new(
            ErrorKind::InvalidOperation,
            format!(
                "{method}() argument '{name}' must be str, not {}",
                python_type(value)
            ),
        )
    })
}

/// One argument as an integer, or the `TypeError` Python raises for a value
/// that is not one.
fn as_int(method: &str, value: &MjValue, name: &str) -> Result<i64, MjError> {
    value.as_i64().ok_or_else(|| {
        MjError::new(
            ErrorKind::InvalidOperation,
            format!(
                "{method}() argument '{name}' must be int, not {}",
                python_type(value)
            ),
        )
    })
}

/// `str.split([sep[, maxsplit]])`.
///
/// Splitting on a separator keeps every empty piece (`' a  b '.split(' ')` is
/// `['', 'a', '', 'b', '']`) and, with a `maxsplit`, keeps the tail verbatim;
/// splitting on whitespace drops the leading run, treats any run of whitespace
/// as one separator, and drops nothing else. An empty separator is Python's
/// `ValueError`.
fn method_split(text: &str, args: &[MjValue]) -> Result<MjValue, MjError> {
    arg_count("split", args, 0, 2)?;
    let maxsplit = match args.get(1) {
        Some(arg) => Some(as_int("split", arg, "maxsplit")?),
        None => None,
    };
    let pieces: Vec<MjValue> = match args.first() {
        None => split_whitespace(text, maxsplit),
        Some(sep) if sep.kind() == ValueKind::None => split_whitespace(text, maxsplit),
        Some(sep) => {
            let sep = as_text("split", sep, "sep")?;
            if sep.is_empty() {
                return Err(MjError::new(ErrorKind::InvalidOperation, "empty separator"));
            }
            let limit = match maxsplit {
                Some(maxsplit) => usize::try_from(maxsplit)
                    .map_or(usize::MAX, |maxsplit| maxsplit.saturating_add(1)),
                None => usize::MAX,
            };
            text.splitn(limit, sep).map(MjValue::from).collect()
        }
    };
    Ok(MjValue::from(pieces))
}

/// `str.split()` with no separator: the leading run of whitespace is dropped,
/// a run separates two pieces, and with a `maxsplit` the tail after the last
/// split is kept verbatim — `' a  b   c  '.split(None, 1)` is
/// `['a', 'b   c  ']`, trailing whitespace and all.
fn split_whitespace(text: &str, maxsplit: Option<i64>) -> Vec<MjValue> {
    let mut rest = text.trim_start_matches(char::is_whitespace);
    let mut left = maxsplit.map(|maxsplit| maxsplit.max(0) as usize);
    let mut pieces = Vec::new();
    loop {
        if rest.is_empty() {
            break;
        }
        if left == Some(0) {
            pieces.push(MjValue::from(rest));
            break;
        }
        match rest.find(char::is_whitespace) {
            Some(index) => {
                pieces.push(MjValue::from(&rest[..index]));
                rest = rest[index..].trim_start_matches(char::is_whitespace);
                left = left.map(|left| left - 1);
            }
            None => {
                pieces.push(MjValue::from(rest));
                break;
            }
        }
    }
    pieces
}

/// Which end [`strip`] trims.
#[derive(Clone, Copy)]
enum Side {
    Both,
    Left,
    Right,
}

/// `str.strip([chars])` — and the one-sided `lstrip`/`rstrip`. No argument
/// trims whitespace (Unicode `White_Space`, which is Python's set minus the
/// `\x1c`–`\x1f` controls); an argument trims any of its characters.
fn strip(method: &str, text: &str, args: &[MjValue], side: Side) -> Result<MjValue, MjError> {
    arg_count(method, args, 0, 1)?;
    let trimmed = match args.first() {
        None => trim(text, side, char::is_whitespace),
        Some(arg) => {
            let chars = as_text(method, arg, "chars")?;
            trim(text, side, |ch: char| chars.contains(ch))
        }
    };
    Ok(MjValue::from(trimmed))
}

fn trim(text: &str, side: Side, stripped: impl Fn(char) -> bool) -> &str {
    match side {
        Side::Both => text.trim_matches(stripped),
        Side::Left => text.trim_start_matches(stripped),
        Side::Right => text.trim_end_matches(stripped),
    }
}

/// `str.replace(old, new[, count])`. A negative `count` is unlimited, as in
/// Python, and an empty `old` inserts `new` between every character (and at
/// both ends), which is Python's own reading of it.
fn method_replace(text: &str, args: &[MjValue]) -> Result<MjValue, MjError> {
    arg_count("replace", args, 2, 3)?;
    let old = as_text("replace", &args[0], "old")?;
    let new = as_text("replace", &args[1], "new")?;
    let replaced = match args.get(2) {
        None => text.replace(old, new),
        Some(arg) => match usize::try_from(as_int("replace", arg, "count")?) {
            Ok(count) => text.replacen(old, new, count),
            // A negative count is unlimited, as in Python.
            Err(_) => text.replace(old, new),
        },
    };
    Ok(MjValue::from(replaced))
}

/// Which end [`affix`] tests.
#[derive(Clone, Copy)]
enum Affix {
    Start,
    End,
}

/// `str.startswith(prefix[, start[, end]])` and `str.endswith`.
///
/// Python's first argument may be a tuple of prefixes; the engine has no
/// tuple, so a **list** of them stands in for it. `start`/`end` slice the
/// string the way Python's do ([`py_slice`]).
fn affix(method: &str, text: &str, args: &[MjValue], affix: Affix) -> Result<MjValue, MjError> {
    arg_count(method, args, 1, 3)?;
    let (_, slice) = py_slice(method, text, args.get(1), args.get(2))?;
    let candidates: Vec<MjValue> = match args[0].kind() {
        ValueKind::Seq => args[0]
            .try_iter()
            .map_err(|_| uniterable(&args[0]))?
            .collect(),
        _ => vec![args[0].clone()],
    };
    for candidate in &candidates {
        let candidate = as_text(method, candidate, "prefix")?;
        let matched = match affix {
            Affix::Start => slice.starts_with(candidate),
            Affix::End => slice.ends_with(candidate),
        };
        if matched {
            return Ok(MjValue::from(true));
        }
    }
    Ok(MjValue::from(false))
}

/// `str.join(iterable)` — every item must be a string, as Python's own
/// `TypeError` demands.
fn method_join(text: &str, args: &[MjValue]) -> Result<MjValue, MjError> {
    arg_count("join", args, 1, 1)?;
    let items: Vec<MjValue> = match args[0].kind() {
        ValueKind::String | ValueKind::Seq | ValueKind::Iterable | ValueKind::Map => args[0]
            .try_iter()
            .map_err(|_| uniterable(&args[0]))?
            .collect(),
        _ => {
            return Err(MjError::new(
                ErrorKind::InvalidOperation,
                "can only join an iterable",
            ))
        }
    };
    let mut joined = String::new();
    for (index, item) in items.iter().enumerate() {
        let piece = item.as_str().ok_or_else(|| {
            MjError::new(
                ErrorKind::InvalidOperation,
                format!(
                    "sequence item {index}: expected str instance, {} found",
                    python_type(item)
                ),
            )
        })?;
        if index > 0 {
            joined.push_str(text);
        }
        joined.push_str(piece);
    }
    Ok(MjValue::from(joined))
}

/// `str.find(sub[, start[, end]])`: the **character** index of the first
/// occurrence (Python counts code points, not bytes), `-1` when there is none.
fn method_find(text: &str, args: &[MjValue]) -> Result<MjValue, MjError> {
    arg_count("find", args, 1, 3)?;
    let sub = as_text("find", &args[0], "sub")?;
    let (begin, slice) = py_slice("find", text, args.get(1), args.get(2))?;
    let found = slice.find(sub).map_or(-1, |byte| {
        begin as i64 + slice[..byte].chars().count() as i64
    });
    Ok(MjValue::from(found))
}

/// `str.count(sub[, start[, end]])`: non-overlapping occurrences, as Python
/// counts them (`'aaa'.count('aa')` is `1`); an empty `sub` counts the gaps
/// between characters, `len + 1` of them.
fn method_count(text: &str, args: &[MjValue]) -> Result<MjValue, MjError> {
    arg_count("count", args, 1, 3)?;
    let sub = as_text("count", &args[0], "sub")?;
    let (_, slice) = py_slice("count", text, args.get(1), args.get(2))?;
    Ok(MjValue::from(slice.matches(sub).count() as i64))
}

/// `list.index(value)` — the first equal item's index; Python's `ValueError`
/// when there is none.
fn list_index(value: &MjValue, args: &[MjValue]) -> Result<MjValue, MjError> {
    arg_count("index", args, 1, 1)?;
    let found = value
        .try_iter()
        .map_err(|_| uniterable(value))?
        .position(|item| item == args[0]);
    match found {
        Some(index) => Ok(MjValue::from(index as i64)),
        None => Err(MjError::new(
            ErrorKind::InvalidOperation,
            format!("{} is not in list", python_repr(&args[0])),
        )),
    }
}

/// `list.count(value)` — how many items equal `value`.
fn list_count(value: &MjValue, args: &[MjValue]) -> Result<MjValue, MjError> {
    arg_count("count", args, 1, 1)?;
    let count = value
        .try_iter()
        .map_err(|_| uniterable(value))?
        .filter(|item| *item == args[0])
        .count();
    Ok(MjValue::from(count as i64))
}

/// The Python slice of `text` the `start`/`end` arguments of
/// `find`/`count`/`startswith`/`endswith` name: a negative index counts back
/// from the end, both are clamped to the string's length, and `start >= end`
/// is the empty string. Returns the slice with the **character** index it
/// starts at, so a found offset can be reported the way Python reports it.
fn py_slice<'a>(
    method: &str,
    text: &'a str,
    start: Option<&MjValue>,
    end: Option<&MjValue>,
) -> Result<(usize, &'a str), MjError> {
    let length = text.chars().count() as i64;
    let bound = |arg: Option<&MjValue>, default: i64, name: &str| -> Result<i64, MjError> {
        match arg {
            None => Ok(default),
            Some(value) => {
                let index = as_int(method, value, name)?;
                Ok(if index < 0 {
                    (length + index).max(0)
                } else {
                    index.min(length)
                })
            }
        }
    };
    let first = bound(start, 0, "start")?;
    let last = bound(end, length, "end")?;
    if first >= last {
        return Ok((first as usize, ""));
    }
    let from = char_offset(text, first as usize);
    let to = char_offset(text, last as usize);
    Ok((first as usize, &text[from..to]))
}

/// The byte offset character `index` starts at (`text.len()` past the end).
fn char_offset(text: &str, index: usize) -> usize {
    text.char_indices()
        .nth(index)
        .map_or(text.len(), |(offset, _)| offset)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::sync::atomic::AtomicUsize;

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

    /// A part that reports `status` and counts its `get_status` calls, for
    /// pinning that a render asks an object once.
    struct Counted(Value, Arc<AtomicUsize>);

    impl PrinterObject for Counted {
        fn get_status(&self, _eventtime: f64) -> Value {
            self.1.fetch_add(1, Ordering::SeqCst);
            self.0.clone()
        }
    }

    /// A part `objects/list` leaves out (upstream's `hasattr(o, 'get_status')`
    /// filter; `pins` on the real machine).
    struct NotQueryable;

    impl PrinterObject for NotQueryable {
        fn get_status(&self, _eventtime: f64) -> Value {
            json!({})
        }

        fn is_queryable(&self) -> bool {
            false
        }
    }

    /// The machine the `printer.objects` tests read: registered in an order
    /// that is **not** alphabetical (`toolhead` before `motion_report`), with
    /// one part that reports no status, so an assertion can tell
    /// "registration order, queryable only" from "sorted" and from "every
    /// registered object".
    fn printer_with_objects() -> (Arc<Printer>, Arc<AtomicUsize>) {
        let calls = Arc::new(AtomicUsize::new(0));
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                "toolhead",
                Arc::new(Counted(
                    json!({ "position": [1.5, 2.5, 3.5, 4.5] }),
                    Arc::clone(&calls),
                )),
            )
            .expect("the name is free");
        printer
            .add_object("pins", Arc::new(NotQueryable))
            .expect("the name is free");
        printer
            .add_object(
                "motion_report",
                Arc::new(Counted(
                    json!({
                        "live_position": [1.0, 2.0, 3.0, 4.0],
                        "live_velocity": 6.0,
                    }),
                    Arc::clone(&calls),
                )),
            )
            .expect("the name is free");
        (printer, calls)
    }

    /// A context over `printer`, the way `MacroState::context`
    /// (`gcode_macro.rs`) binds it.
    fn printer_context(printer: Arc<Printer>) -> Context {
        let mut context = Context::new();
        context.insert("printer", Rt::Printer(PrinterView::new(printer)));
        context
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

    /// A container prints the way Python's `str` prints it: its elements' own
    /// `repr` — strings quoted, `True`/`None`, floats with `.0`, nesting
    /// recursed — so `mainsail.cfg:184`'s
    /// `VALUE="{{ 'enable': ENABLE, 'call': MACRO }}"` writes a text a Python
    /// reader can evaluate back instead of the engine's `Debug` output.
    #[test]
    fn containers_render_python_repr() {
        let dict = |value: Value| {
            let mut context = context(&[], "");
            context.insert("v", Rt::Json(value));
            context
        };

        // Keys come out in the engine's map order (its map is ordered), which
        // is alphabetical, not Jinja2's literal order.
        assert_eq!(
            render(
                "{v}",
                &mut dict(json!({"layer": 1, "call": "PAUSE", "enable": true}))
            )
            .expect("renders"),
            "{'call': 'PAUSE', 'enable': True, 'layer': 1}"
        );
        assert_eq!(
            render(
                "{v}",
                &mut dict(json!({"outer": {"inner": [1, 2.0, Value::Null]}}))
            )
            .expect("renders"),
            "{'outer': {'inner': [1, 2.0, None]}}"
        );
        assert_eq!(
            render("{v}", &mut dict(json!(["a", 1, 2.0]))).expect("renders"),
            "['a', 1, 2.0]"
        );
        assert_eq!(render("{v}", &mut dict(json!({}))).expect("renders"), "{}");
        assert_eq!(render("{v}", &mut dict(json!([]))).expect("renders"), "[]");
        // A backslash and the quote itself are escaped, as `repr` escapes them.
        assert_eq!(
            render("{v}", &mut dict(json!({"it's": "a \\ b"}))).expect("renders"),
            r"{'it\'s': 'a \\ b'}"
        );
        // The scalar spellings inside a container are the same Python ones the
        // engine already prints: `True`, `None`, `2.0`.
        assert_eq!(
            render("{v}", &mut dict(json!([true, false, null, 2.0]))).expect("renders"),
            "[True, False, None, 2.0]"
        );
        // A dict *literal* in a body, which is how the corpus spells it.
        assert_eq!(
            ok("{ {'enable': True, 'call': 'PAUSE'} }"),
            "{'call': 'PAUSE', 'enable': True}"
        );
        // A **whole tag** is `str`: a top-level string stays unquoted, and so
        // does a container's key read out of it.
        assert_eq!(
            render("{v}", &mut dict(json!("PAUSE"))).expect("renders"),
            "PAUSE"
        );
        assert_eq!(
            render("{v.call}", &mut dict(json!({"call": "PAUSE"}))).expect("renders"),
            "PAUSE"
        );
    }

    /// The two kinds of `Undefined` the engine distinguishes are one kind to a
    /// formatter, so the strict check covers both: printing a *silent* one
    /// (`{1 if False}`, blank in Jinja2) fails the render instead of printing
    /// nothing — the trade the deviations table records. An undefined that
    /// `is defined` probes or `\|default` swallows never reaches the formatter,
    /// so those stay quiet.
    #[test]
    fn a_printed_undefined_is_a_render_error() {
        let error = render("{1 if False}", &mut context(&[], "")).expect_err("silent undefined");
        assert!(error.to_string().contains("undefined value"), "{error}");
        assert_eq!(ok("{nope|default('none')}"), "none");
    }

    /// `mainsail.cfg:176-182`'s `SET_PAUSE_NEXT_LAYER`, body for body: the
    /// value it hands `SET_GCODE_VARIABLE` is a dict, and upstream writes it
    /// with a tag (`VALUE="{{ … }}"`) — so its text must be Python's spelling
    /// (`klippy`'s `ast.literal_eval` reads it back).
    #[test]
    fn pause_next_layer_writes_its_value_as_python_repr() {
        let printer = printer_with(
            "gcode_macro SET_PRINT_STATS_INFO",
            json!({"pause_next_layer": {"enable": false, "call": "PAUSE"}}),
        );
        let mut context = Context::new();
        context.insert("printer", Rt::Printer(PrinterView::new(printer)));
        context.insert("params", Rt::Json(json!({"ENABLE": "1"})));
        let body = "{% set pause_next_layer = printer['gcode_macro SET_PRINT_STATS_INFO']\
                    .pause_next_layer %}\n\
                    {% set ENABLE = params.ENABLE|default(1)|int != 0 %}\n\
                    {% set MACRO = params.MACRO|default(pause_next_layer.call, True) %}\n\
                    SET_GCODE_VARIABLE MACRO=SET_PRINT_STATS_INFO VARIABLE=pause_next_layer \
                    VALUE=\"{{ 'enable': ENABLE, 'call': MACRO }}\"\n";
        assert_eq!(
            render(body, &mut context).expect("the macro loads"),
            // The three `{% set %}` lines leave their newlines; the value itself
            // is Python's spelling of the dict.
            "\n\n\nSET_GCODE_VARIABLE MACRO=SET_PRINT_STATS_INFO VARIABLE=pause_next_layer \
             VALUE=\"{'call': 'PAUSE', 'enable': True}\"\n"
        );
    }

    /// `mainsail.cfg`'s printf formatings in **expression** tags, body for
    /// body: upstream runs Jinja2, where a string on the left of `%` is
    /// Python's printf formatting, which this engine's `%` (the remainder)
    /// refuses; [`rewrite_printf_formatting`] turns each tag into the engine's
    /// `|format` filter before it is compiled, `%s` / `%d` / `%3.1f` / `%%`
    /// included.
    #[test]
    fn mainsail_printf_formatings_render() {
        let printer = printer_with("toolhead", json!({"extruder": "extruder"}));
        let mut context = Context::new();
        context.insert("printer", Rt::Printer(PrinterView::new(printer)));
        context.insert("last_extruder_temp", Rt::Json(json!({"temp": 205.0})));
        context.insert("element", Rt::Json(json!("M117 hello")));
        context.insert("pause_next_layer", Rt::Json(json!({"call": "PAUSE"})));
        context.insert(
            "pause_at_layer",
            Rt::Json(json!({"call": "PAUSE_AT_LAYER", "layer": 12})),
        );

        // A `(a, b)` tuple becomes two arguments, a `%3.1f` a printf float.
        assert_eq!(
            render(
                r#"{"Restoring \"%s\" temperature to %3.1f\u00B0C, this may take some time" % (printer.toolhead.extruder, last_extruder_temp.temp) }"#,
                &mut context
            )
            .expect("mainsail.cfg:138 renders"),
            "Restoring \"extruder\" temperature to 205.0\u{00B0}C, this may take some time"
        );
        // A single operand, no parentheses.
        assert_eq!(
            render(
                r#"{"Resume aborted !!! \"%s\" not hot enough, please heat up again and press RESUME" % printer.toolhead.extruder}"#,
                &mut context
            )
            .expect("mainsail.cfg:144 renders"),
            "Resume aborted !!! \"extruder\" not hot enough, please heat up again and press RESUME"
        );
        assert_eq!(
            render(r#"{"action:prompt_text %s" % element}"#, &mut context)
                .expect("mainsail.cfg:169 renders"),
            "action:prompt_text M117 hello"
        );
        assert_eq!(
            render(
                r#"{"%s, forced by pause_next_layer" % pause_next_layer.call}"#,
                &mut context
            )
            .expect("mainsail.cfg:203 renders"),
            "PAUSE, forced by pause_next_layer"
        );
        assert_eq!(
            render(
                r#"{"%s, forced by pause_at_layer [%d]" % (pause_at_layer.call, pause_at_layer.layer)}"#,
                &mut context
            )
            .expect("mainsail.cfg:207 renders"),
            "PAUSE_AT_LAYER, forced by pause_at_layer [12]"
        );
        // `%%` is one literal `%`, as in Python.
        assert_eq!(
            render(r#"{"%d%% of %s" % (12, element)}"#, &mut context).expect("a `%%` renders"),
            "12% of M117 hello"
        );
    }

    /// A statement tag carries expressions too, so `{% set %}` is rewritten
    /// the same way: `mainsail.cfg:145`/`:152` (one body) and `:163`, whose
    /// printf sits inside a `prompt_txt.append(…)` call — the exact source and
    /// the exact rewritten text, then a render of the same tag.
    #[test]
    fn mainsail_printf_formatings_in_set_tags_are_rewritten() {
        let not_hot_enough: &str = r#"{% set _d = prompt_txt.append("\"%s\" not hot enough, please heat up again and press RESUME" % printer.toolhead.extruder) %}"#;
        assert_eq!(
            &*rewrite_printf_formatting(not_hot_enough),
            r#"{% set _d = prompt_txt.append("\"%s\" not hot enough, please heat up again and press RESUME"|format(printer.toolhead.extruder)) %}"#
        );

        let no_filament: &str = r#"{% set _d = prompt_txt.append("\"%s\" detects no filament, please load filament and press RESUME" % (client.runout_sensor.split(" "))[1]) %}"#;
        assert_eq!(
            &*rewrite_printf_formatting(no_filament),
            r#"{% set _d = prompt_txt.append("\"%s\" detects no filament, please load filament and press RESUME"|format((client.runout_sensor.split(" "))[1])) %}"#
        );

        // `.append(…)` and `.split(…)` are Python **methods** this port's
        // engine does not have yet (the `|format` operand text is the part
        // this unit owns), so the render keeps the tag and the literal and
        // binds what the call would have produced.
        let printer = printer_with("toolhead", json!({"extruder": "extruder"}));
        let mut context = Context::new();
        context.insert("printer", Rt::Printer(PrinterView::new(printer)));
        context.insert("runout_name", Rt::Json(json!("Sensor")));
        assert_eq!(
            render(
                r##"{% set txt = "\"%s\" not hot enough, please heat up again and press RESUME" % printer.toolhead.extruder %}{txt}"##,
                &mut context
            )
            .expect("a set-tag formatting renders"),
            "\"extruder\" not hot enough, please heat up again and press RESUME"
        );
        assert_eq!(
            render(
                r##"{% set txt = "\"%s\" detects no filament, please load filament and press RESUME" % runout_name %}{txt}"##,
                &mut context
            )
            .expect("a set-tag formatting renders"),
            "\"Sensor\" detects no filament, please load filament and press RESUME"
        );
    }

    /// The expression-tag half of `mainsail.cfg:162`: a parenthesized group with
    /// a trailer is **one** argument, `(client.runout_sensor.split(" "))[1]`.
    #[test]
    fn a_trailered_group_is_one_format_argument() {
        let source: &str = r#"{"\"%s\" detects no filament, please load filament and press RESUME" % (client.runout_sensor.split(" "))[1]}"#;
        assert_eq!(
            &*rewrite_printf_formatting(source),
            r#"{"\"%s\" detects no filament, please load filament and press RESUME"|format((client.runout_sensor.split(" "))[1])}"#
        );
        // `.split(…)` is a Python method this port's engine does not have yet,
        // so the render binds what the call would have produced.
        let mut context = context(&[], "");
        context.insert("runout_name", Rt::Json(json!("Sensor")));
        assert_eq!(
            render(
                r#"{"\"%s\" detects no filament, please load filament and press RESUME" % runout_name}"#,
                &mut context
            )
            .expect("renders"),
            "\"Sensor\" detects no filament, please load filament and press RESUME"
        );
    }

    /// Every other `%` keeps the engine's meaning: a numeric left operand is
    /// untouched, a non-literal one is left to fail the way it does today, and
    /// a formatting the rewrite cannot read is not guessed at (no panic, the
    /// loud load error it always was).
    #[test]
    fn a_percent_that_is_not_a_string_formatting_is_left_alone() {
        assert_eq!(ok("{ 7 % 3 }"), "1");
        assert_eq!(ok("{ 17 * 2 + 1 % 4 }"), "35");

        let mut context = context(&[], "");
        context.insert("s", Rt::Json(json!("abc")));
        let error = render("{ s % 3 }", &mut context).expect_err("% of a string");
        assert!(
            error
                .to_string()
                .contains("tried to use % operator on unsupported types string and number"),
            "{error}"
        );
        assert!(Template::parse("t", r#"{"x %s" % (a}"#).is_err());
        let error = render(r#"{"%(key)s" % {'key': 1}}"#, &mut context).expect_err("a mapping");
        assert!(
            error
                .to_string()
                .contains("tried to use % operator on unsupported types string and map"),
            "{error}"
        );
    }

    /// A `{% set %}` or `{% if %}` whose `%` is an **operator**, a comment tag
    /// and the text between tags keep their bytes, and the rewrite adds no
    /// newline — an error still names the line the body wrote.
    #[test]
    fn the_printf_rewrite_leaves_non_formatings_and_comments_alone() {
        for source in [
            "{% set x = 100 %}",
            "{% if n % 2 == 0 %}even{% else %}odd{% endif %}",
            "{# \"%s\" % x #}",
            "{# \"%s\" % x #}\"%s\" % x",
        ] {
            assert_eq!(&*rewrite_printf_formatting(source), source);
        }
        assert_eq!(ok("{% set x = 100 %}{x}"), "100");
        let mut context = context(&[], "");
        context.insert("n", Rt::Json(json!(4)));
        assert_eq!(
            render(
                "{% if n % 2 == 0 %}even{% else %}odd{% endif %}",
                &mut context
            )
            .expect("`%` is still the remainder"),
            "even"
        );
        context.insert("t", Rt::Json(json!("v")));
        assert_eq!(
            render(
                "{# \"%s\" % t #}\"%s\" % t\n{% set x = 1 %}{x}",
                &mut context
            )
            .expect("text and tags survive"),
            "\"%s\" % t\n1"
        );
        let error = render("{\"%s\" % t}\n{nope}", &mut context).expect_err("undefined");
        assert!(error.to_string().contains("line 2"), "{error}");
    }

    /// The vendored client macro file itself: every one of its eleven printf
    /// formatings is rewritten — six in `{ … }` tags, five in `{% set %}` —
    /// and no newline moves, so its error frames still name its own lines.
    #[test]
    fn every_printf_in_the_vendored_mainsail_file_is_rewritten() {
        let source = include_str!("mainsail.cfg");
        let rewritten = rewrite_printf_formatting(source);
        assert_eq!(source.matches("|format(").count(), 0);
        assert_eq!(rewritten.matches("|format(").count(), 11);
        assert_eq!(rewritten.lines().count(), source.lines().count());
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
    /// containment by queryable registration (`GetStatusWrapper.__contains__`)
    /// and iterates the object names (`__iter__`), never a status.
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
        // `'name' in printer` is registration and a status
        // (`GetStatusWrapper.__contains__` through `__getitem__`).
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

    /// `printer.objects` is the reflection `objects/list` reports: every part
    /// that answers a status, in registration order — `{% for name in
    /// printer.objects %}` walks the names (a Jinja2 dict iterates its keys),
    /// `|length` counts them, and a part that reports no status (`pins`) or a
    /// name nobody registered is absent.
    #[test]
    fn printer_objects_iterates_the_queryable_objects() {
        let (printer, _calls) = printer_with_objects();
        let mut context = printer_context(printer);

        assert_eq!(
            render(
                "{% for name in printer.objects %}{name} {% endfor %}",
                &mut context
            )
            .expect("renders"),
            // Two queryable parts, `pins` left out: the N the machine has, not
            // the three it registered.
            "toolhead motion_report "
        );
        assert_eq!(
            render("{printer.objects|length}", &mut context).expect("renders"),
            "2"
        );
        assert_eq!(
            render(
                "{% if 'motion_report' in printer.objects %}yes{% else %}no{% endif %}",
                &mut context
            )
            .expect("renders"),
            "yes"
        );
        assert_eq!(
            render(
                "{% if 'pins' in printer.objects %}yes{% else %}no{% endif %}",
                &mut context
            )
            .expect("renders"),
            "no"
        );
    }

    /// The order is the registry's and stays put: `toolhead` precedes
    /// `motion_report` although it sorts after it, and three renders of one
    /// machine walk the same order. `|sort` is still there for a body that
    /// wants alphabetical.
    #[test]
    fn printer_objects_walk_the_registration_order() {
        let (printer, _calls) = printer_with_objects();
        let mut context = printer_context(printer);
        let body = "{% for name in printer.objects %}{name} {% endfor %}";

        for _ in 0..3 {
            assert_eq!(
                render(body, &mut context).expect("renders"),
                "toolhead motion_report "
            );
        }
        assert_eq!(
            render(
                "{% for name in printer.objects|sort %}{name} {% endfor %}",
                &mut context
            )
            .expect("renders"),
            "motion_report toolhead "
        );
    }

    /// One object's status through the reflection — the `motion_report`
    /// `live_position` a macro reads — and the render's cache: both spellings
    /// (`printer.objects['name']` and `printer['name']`) ask `get_status`
    /// **once** for that object, so a body loops over objects without paying a
    /// status read per use.
    #[test]
    fn printer_objects_lookup_reads_the_cached_status() {
        let (printer, calls) = printer_with_objects();
        let mut context = printer_context(printer);

        assert_eq!(
            render(
                "{printer.objects['motion_report'].live_position.z} \
                 {printer.objects.motion_report.live_velocity} \
                 {printer['motion_report'].live_position.x}",
                &mut context
            )
            .expect("renders"),
            "3.0 6.0 1.0"
        );
        assert_eq!(calls.load(Ordering::SeqCst), 1);

        // A second object is the second read, and its status is the cached
        // one too.
        assert_eq!(
            render(
                "{printer.objects['toolhead'].position.y} \
                 {printer.objects['toolhead'].position.e}",
                &mut context
            )
            .expect("renders"),
            "2.5 4.5"
        );
        assert_eq!(calls.load(Ordering::SeqCst), 2);
    }

    /// A name the reflection does not hold — unregistered, or registered
    /// without a status — is `Undefined`, which Strict fails the render on
    /// when the body uses it (the `printer.nope` rule, not a silent blank);
    /// the `in` probe stays quiet, as it does on `printer`.
    #[test]
    fn printer_objects_unknown_name_is_undefined() {
        let (printer, _calls) = printer_with_objects();
        let mut context = printer_context(printer);

        let error = render("{printer.objects['nope']}", &mut context).expect_err("unregistered");
        assert!(error.to_string().contains("undefined value"), "{error}");

        let error = render("{printer.objects.pins}", &mut context).expect_err("not queryable");
        assert!(error.to_string().contains("undefined value"), "{error}");

        assert_eq!(
            render(
                "{% if 'nope' in printer.objects %}yes{% else %}no{% endif %}",
                &mut context
            )
            .expect("renders"),
            "no"
        );
    }

    /// The upstream spelling of the same reflection: `{% for name in printer %}`
    /// walks the names `printer.objects` holds — same list, same registration
    /// order, same count (`GetStatusWrapper.__iter__`, `gcode_macro.py:40-43`).
    #[test]
    fn printer_iteration_walks_the_same_names_as_the_objects_view() {
        let (printer, _calls) = printer_with_objects();
        let mut context = printer_context(printer);

        let walked = render("{% for name in printer %}{name} {% endfor %}", &mut context)
            .expect("printer iterates");
        let mapped = render(
            "{% for name in printer.objects %}{name} {% endfor %}",
            &mut context,
        )
        .expect("renders");

        assert_eq!(walked, mapped);
        // Pinned, so "same sequence" cannot drift into "both empty": the
        // machine registered three parts and `pins` reports no status.
        assert_eq!(walked, "toolhead motion_report ");
        assert_eq!(
            render("{printer|length}", &mut context).expect("renders"),
            "2"
        );
    }

    /// Containment is upstream's `__contains__` (`gcode_macro.py:33-37`): the
    /// name has to be lookable up through `__getitem__`, which refuses an
    /// object without a status (`:24-26`) — so a registered-but-unqueryable
    /// part (`pins`) is absent, and both spellings agree about it.
    #[test]
    fn printer_containment_needs_a_status_not_just_registration() {
        let (printer, _calls) = printer_with_objects();
        let mut context = printer_context(printer);

        assert_eq!(
            render(
                "{% if 'pins' in printer %}yes{% else %}no{% endif %}",
                &mut context
            )
            .expect("renders"),
            "no"
        );
        assert_eq!(
            render(
                "{% if 'toolhead' in printer %}yes{% else %}no{% endif %}",
                &mut context
            )
            .expect("renders"),
            "yes"
        );
        // A lookup that misses is `Undefined`, not a status: `pins` still
        // registered, just not queryable.
        let error = render("{printer.pins}", &mut context).expect_err("not queryable");
        assert!(error.to_string().contains("undefined value"), "{error}");
        let error = render("{printer['pins']}", &mut context).expect_err("not queryable");
        assert!(error.to_string().contains("undefined value"), "{error}");
    }

    /// Walking the reflection reads no status: iteration only asks the
    /// registry for names, so a body that loops first and looks one object up
    /// pays exactly that one `get_status` (and its cache, as
    /// [`printer_objects_lookup_reads_the_cached_status`] pins).
    #[test]
    fn printer_iteration_reads_no_status() {
        let (printer, calls) = printer_with_objects();
        let mut context = printer_context(printer);

        for body in [
            "{% for name in printer %}{name} {% endfor %}",
            "{% for name in printer.objects %}{name} {% endfor %}",
        ] {
            render(body, &mut context).expect("renders");
        }
        assert_eq!(calls.load(Ordering::SeqCst), 0);

        render("{printer.motion_report.live_velocity}", &mut context).expect("renders");
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        render("{% for name in printer %}{name} {% endfor %}", &mut context).expect("renders");
        assert_eq!(calls.load(Ordering::SeqCst), 1);
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

    /// `str.split`, the method the Mainsail corpus' `RESUME` depends on:
    /// `client.runout_sensor.split(" ")[1]` runs on every resume
    /// (`mainsail.cfg`). Each shape is what
    /// `python3 -c "print(…split(…))"` prints.
    #[test]
    fn split_follows_python_for_whitespace_separators_and_maxsplit() {
        // No argument: a run of whitespace separates, nothing empty survives.
        assert_eq!(ok("{ 'a b  c'.split() }"), "['a', 'b', 'c']");
        assert_eq!(ok("{ '  '.split() }"), "[]");
        assert_eq!(ok("{ ''.split() }"), "[]");
        // A separator: every empty piece is kept, consecutive separators too.
        assert_eq!(ok("{ ' a  b '.split(' ') }"), "['', 'a', '', 'b', '']");
        assert_eq!(ok("{ 'a,b'.split(',') }"), "['a', 'b']");
        assert_eq!(ok("{ 'a'.split(',') }"), "['a']");
        // `maxsplit` leaves the tail alone, and the whitespace form keeps its
        // trailing blanks (`' a  b   c  '.split(None, 1)` in Python).
        assert_eq!(ok("{ 'a b c'.split(' ', 1) }"), "['a', 'b c']");
        assert_eq!(ok("{ 'a,b,,c'.split(',', 2) }"), "['a', 'b', ',c']");
        assert_eq!(ok("{ ' a  b   c  '.split(none, 1) }"), "['a', 'b   c  ']");
        assert_eq!(ok("{ ' a  b '.split(none, 0) }"), "['a  b ']");
        // The corpus' own line.
        assert_eq!(
            ok("{ 'runout_sensor: TRIGGERED'.split(' ')[1] }"),
            "TRIGGERED"
        );

        let error =
            render("{ 'a b'.split('') }", &mut context(&[], "")).expect_err("empty separator");
        assert!(error.to_string().contains("empty separator"), "{error}");
    }

    /// `str.strip`/`lstrip`/`rstrip`, `lower`/`upper` and `replace`.
    #[test]
    fn strip_case_and_replace_follow_python() {
        assert_eq!(ok("[{ '  x  '.strip() }]"), "[x]");
        assert_eq!(ok("[{ '  x  '.lstrip() }]"), "[x  ]");
        assert_eq!(ok("[{ '  x  '.rstrip() }]"), "[  x]");
        // An argument is a *set* of characters, not a substring.
        assert_eq!(ok("{ 'xxaxx'.strip('x') }"), "a");
        assert_eq!(ok("{ 'xyaxy'.strip('xy') }"), "a");
        assert_eq!(ok("{ 'AbC'.lower() }/{ 'AbC'.upper() }"), "abc/ABC");

        assert_eq!(ok("{ 'a-b-c'.replace('-', '+') }"), "a+b+c");
        assert_eq!(ok("{ 'a-b-c'.replace('-', '+', 1) }"), "a+b-c");
        assert_eq!(ok("{ 'a-b-c'.replace('-', '+', 0) }"), "a-b-c");
        // A negative count is unlimited, as in Python.
        assert_eq!(ok("{ 'a-b-c'.replace('-', '+', -1) }"), "a+b+c");
        // An empty `old` inserts between every character and at both ends.
        assert_eq!(ok("{ 'abc'.replace('', '-') }"), "-a-b-c-");
        assert_eq!(ok("{ 'abc'.replace('', '-', 2) }"), "-a-bc");
    }

    /// `str.startswith`/`endswith` with the optional `start`/`end` window.
    #[test]
    fn affixes_follow_python_including_the_slice_window() {
        assert_eq!(ok("{ 'setup.cfg'.startswith('setup') }"), "True");
        assert_eq!(ok("{ 'setup.cfg'.endswith('.cfg') }"), "True");
        assert_eq!(ok("{ 'setup.cfg'.startswith('cfg') }"), "False");
        assert_eq!(ok("{ 'abc'.startswith('b', 1) }"), "True");
        assert_eq!(ok("{ 'abc'.endswith('b', 0, 2) }"), "True");
        assert_eq!(ok("{ 'abc'.startswith('b', 0, 1) }"), "False");
        // Negative indexes count from the end.
        assert_eq!(ok("{ 'abc'.endswith('c', -1) }"), "True");
        // A list stands in for Python's tuple of prefixes.
        assert_eq!(ok("{ 'abc'.startswith(['x', 'a']) }"), "True");
        assert_eq!(ok("{ 'abc'.endswith(['x', 'a']) }"), "False");
    }

    /// `str.find` and `str.count`: Python's **character** indexes, its window
    /// arguments, and `find`'s `-1`.
    #[test]
    fn find_and_count_follow_python() {
        assert_eq!(ok("{ 'abcabc'.find('bc') }"), "1");
        assert_eq!(ok("{ 'abc'.find('z') }"), "-1");
        assert_eq!(ok("{ 'abcabc'.find('bc', 2) }"), "4");
        assert_eq!(ok("{ 'abc'.find('', 1) }"), "1");
        // Code points, not bytes, as Python counts them.
        assert_eq!(ok("{ 'héllo'.find('llo') }"), "2");

        assert_eq!(ok("{ 'aaa'.count('aa') }"), "1");
        assert_eq!(ok("{ 'abcabc'.count('bc', 2) }"), "1");
        assert_eq!(ok("{ 'abc'.count('') }"), "4");
        assert_eq!(ok("{ 'abc'.count('z') }"), "0");
    }

    /// `str.join`: the separator comes from the receiver, the items must be
    /// strings, and a string argument iterates as its characters.
    #[test]
    fn join_follows_python() {
        assert_eq!(ok("{ '-'.join(['a', 'b']) }"), "a-b");
        assert_eq!(ok("{ ''.join(['a', 'b']) }"), "ab");
        assert_eq!(ok("[{ '-'.join([]) }]"), "[]");
        assert_eq!(ok("{ '-'.join('abc') }"), "a-b-c");

        let error = render("{ '-'.join([1]) }", &mut context(&[], "")).expect_err("item");
        assert!(
            error
                .to_string()
                .contains("sequence item 0: expected str instance, int found"),
            "{error}"
        );
        let error = render("{ '-'.join(1) }", &mut context(&[], "")).expect_err("not iterable");
        assert!(
            error.to_string().contains("can only join an iterable"),
            "{error}"
        );
    }

    /// The list methods that read (`index`, `count`) are Python's own, and the
    /// three that would mutate are answered with `None`: the engine's lists are
    /// values, so nothing can write back to the list a body holds — the
    /// deviation table's row. What a body sees: `None` back, the list as it was.
    #[test]
    fn list_reads_work_and_list_mutation_degrades_to_none() {
        assert_eq!(ok("{% set n = [1, 2, 2] %}{ n.count(2) }"), "2");
        assert_eq!(ok("{% set n = [1, 2, 2] %}{ n.count(3) }"), "0");
        assert_eq!(ok("{% set n = [1, 2, 2] %}{ n.index(2) }"), "1");
        let error =
            render("{% set n = [1] %}{ n.index(3) }", &mut context(&[], "")).expect_err("absent");
        assert!(error.to_string().contains("3 is not in list"), "{error}");

        // `{% set _d = prompt_txt.append(…) %}` — `mainsail.cfg`'s abort
        // prompt — renders, answers `None`, and collects nothing.
        assert_eq!(
            ok("{% set n = [1, 2] %}{% set _d = n.append(3) %}{ n }/{ _d }"),
            "[1, 2]/None"
        );
        assert_eq!(
            ok("{% set n = [1] %}{% set _d = n.extend([2, 3]) %}{ n }"),
            "[1]"
        );
        assert_eq!(
            ok("{% set n = [1, 2] %}{% set _d = n.pop() %}{ n }/{ _d }"),
            "[1, 2]/None"
        );

        let error =
            render("{% set n = [1] %}{ n.extend(1) }", &mut context(&[], "")).expect_err("int");
        assert!(
            error
                .to_string()
                .contains("extend() argument must be an iterable, not int"),
            "{error}"
        );
    }

    /// A method nothing implements — another name, another type — stays the
    /// engine's `unknown method` error rather than a quiet `None`.
    #[test]
    fn unimplemented_methods_stay_unknown_method_errors() {
        for source in [
            "{ 'a'.nosuch() }",
            "{ 'a'.sort() }",
            "{ params.split('a') }",
            "{% set n = [1] %}{ n.sort() }",
            "{ (1).split() }",
        ] {
            let error = render(source, &mut context(&[], "")).expect_err(source);
            assert!(
                error.to_string().contains("has no method named"),
                "{source}: {error}"
            );
        }
    }

    /// A wrong argument is Python's `TypeError` shape — an argument count or a
    /// type — not a silently wrong answer.
    #[test]
    fn method_arguments_are_checked_like_pythons() {
        let error = render("{ 'a b'.split(' ', 1, 2) }", &mut context(&[], "")).expect_err("3");
        assert!(
            error
                .to_string()
                .contains("split() takes at most 2 arguments (3 given)"),
            "{error}"
        );
        let error = render("{ 'a b'.split(1) }", &mut context(&[], "")).expect_err("int sep");
        assert!(
            error
                .to_string()
                .contains("split() argument 'sep' must be str, not int"),
            "{error}"
        );
        let error = render("{ 'a'.replace('a') }", &mut context(&[], "")).expect_err("1 arg");
        assert!(
            error
                .to_string()
                .contains("replace() takes at least 2 positional arguments (1 given)"),
            "{error}"
        );
        let error = render("{ 'a'.lower(1) }", &mut context(&[], "")).expect_err("1 arg");
        assert!(
            error
                .to_string()
                .contains("str.lower() takes no arguments (1 given)"),
            "{error}"
        );
        let error = render("{ ' a '.strip(1) }", &mut context(&[], "")).expect_err("int chars");
        assert!(
            error
                .to_string()
                .contains("strip() argument 'chars' must be str, not int"),
            "{error}"
        );
        let error = render("{ 'a'.find('a', 'x') }", &mut context(&[], "")).expect_err("start");
        assert!(
            error
                .to_string()
                .contains("find() argument 'start' must be int, not str"),
            "{error}"
        );
    }
}
