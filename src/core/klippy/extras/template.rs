//! A Jinja2-subset template engine for `gcode_macro` bodies.
//!
//! Upstream renders every macro body with `jinja2.Environment('{%', '%}', '{',
//! '}')` (`klippy/extras/gcode_macro.py:82`) — the variable delimiters are
//! **single braces**, which is why the corpus writes `{params.P}` rather than
//! `{{ params.P }}`, and [`TemplateWrapper`] (`:46-79`) compiles the body at
//! load and renders it against `create_template_context` (`:101-108`) before
//! `gcode.run_script_from_command` feeds the text back to the dispatcher.
//!
//! This port implements the subset the corpus actually uses, one context at a
//! time, and fails **explicitly** outside it — a template that does not parse
//! is a config-load error, a template that does not evaluate is a command
//! error; nothing silently renders empty.
//!
//! # Supported
//!
//! | construct | example from the corpus |
//! |---|---|
//! | text | `PARK_{printer.toolhead.extruder}` (`dual_carriage.cfg:72`) |
//! | `{ … }` expressions | `{action_raise_error("…")}` (`exclude_object.cfg:86`) |
//! | `{% if %}` / `elif` / `else` / `endif` | `exclude_object.cfg:85-113` |
//! | `{% for x in … %}` / `endfor` | `exclude_object.cfg:92` |
//! | `{% set name = expr %}` | `{% set x_center = 0.5 * (x_max + x_min) %}` (`generic_cartesian_iqex.cfg:288`) |
//! | `{# … #}` comments | (none in the corpus; parsed and skipped) |
//! | literals | `0.0`, `'-1'`, `"abc"`, `True`/`False`/`None` |
//! | names, `a.b`, `a["k"]`, `a[0]`, calls | `printer["gcode_macro T"].t` |
//! | `and` `or` `not`, `in` `not in`, `==` `!=` `<` `>` `<=` `>=` | `macros.cfg:66` |
//! | `+ - * / %`, unary `-`, `t - 12.0` | `macros.cfg:37` |
//! | `x is defined` / `is not defined` | `sdcard_loop.cfg:90` |
//! | filters `\| int`, `\| float`, `\| default(x)` | `params.S \| default(1000.0) \| float` (`printer-velleman-k8800-2017.cfg:125`) |
//! | `range(n)` | `range(params.T \| int)` |
//! | `action_respond_info`, `action_raise_error` | `macros.cfg`, `exclude_object.cfg` |
//! | the macro's `variable_*` as bare names, `params`, `rawparams` | `macros.cfg:33` |
//!
//! # Deliberate gaps (explicit errors, not silent blanks)
//!
//! - Statements outside `if/elif/else/endif/for/endfor/set` — Jinja also has
//!   `{% block %}`, `{% include %}`, … — are refused at load
//!   (`unsupported statement 'block'`).
//! - Filters outside `int`/`float`/`default` (`min`/`max`/`abs`/`replace` in
//!   the same corpus) are refused at render; a filter's keyword arguments
//!   (`default(0, boolean=True)`) are outside the subset, so the tokenizer
//!   refuses the `=`.
//! - `range(n)` takes one argument; `action_emergency_stop` and
//!   `action_call_remote_method` are not bound, so a name error says so.
//! - Python literals in `ast.literal_eval` syntax (`None`, `'str'`, …) are
//!   JSON here, matching this port's `variable_*` reader (`gcode_macro.rs`).
//! - A missing printer object or status key is an **error**, where Jinja2's
//!   default `Undefined` would render an empty string. Upstream's *corpus*
//!   always names a key that exists; a port gap that hides behind a blank
//!   `PARK_` line would be silent, so it fails loudly instead.
//! - `\| default(x)` is the one construct that expects to miss: it probes its
//!   base quietly, the way `is defined` does, and falls back to `x`, because
//!   `params.S \| default(…)` is exactly how the corpus spells an omitted
//!   parameter. Every other lookup still fails loudly.
//!
//! Status coordinates deserve their own note: klippy reports `Coord`
//! namedtuples (`klippy/gcode.py`), which Jinja reads as `.x`/`.y`/`.z`/`.e`,
//! while this host's statuses are JSON arrays. An **array attribute** lookup
//! maps `x y z e` to indices `0..3` — the namedtuple's field order — and
//! nothing else (`Coord` is `x y z e` in `mathutil.rs` too).

use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;

use serde_json::{json, Value};

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
/// (`gcode_macro.py:47-79`, `:70-79`): a load error is
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
    fn load(name: &str, line: usize, detail: impl Into<String>) -> Self {
        Self {
            name: name.to_string(),
            line,
            phase: Phase::Load,
            detail: detail.into(),
        }
    }

    fn evaluate(name: &str, line: usize, detail: impl Into<String>) -> Self {
        Self {
            name: name.to_string(),
            line,
            phase: Phase::Evaluate,
            detail: detail.into(),
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
    /// A literal list (`[a, b]` is *not* supported; `range(n)` builds these).
    List(Vec<Rt>),
    /// `printer`: upstream's `GetStatusWrapper` (`gcode_macro.py:15-43`).
    Printer(PrinterView),
    /// `range` / `action_*`.
    Builtin(Builtin),
}

/// The callables a template context binds.
#[derive(Clone)]
pub enum Builtin {
    /// `range(n)` — one argument, `0..n` (`exclude_object.cfg:92`).
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
/// the render the way `GetStatusWrapper.cache` caches it (`gcode_macro.py:17`).
#[derive(Clone)]
pub struct PrinterView {
    printer: Arc<Printer>,
    cache: RefCell<HashMap<String, Value>>,
}

impl fmt::Debug for PrinterView {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `Printer` has no `Debug`; the cache is the interesting part.
        f.debug_struct("PrinterView")
            .field("cached", &self.cache.borrow().keys().collect::<Vec<_>>())
            .finish()
    }
}

impl PrinterView {
    /// View `printer` through `eventtime`-stamped statuses.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            cache: RefCell::new(HashMap::new()),
        }
    }

    /// One object's status (`GetStatusWrapper.__getitem__`, `:19-32`).
    fn status(&self, name: &str) -> Option<Value> {
        if let Some(cached) = self.cache.borrow().get(name) {
            return Some(cached.clone());
        }
        let status = self.printer.status_of(name, self.printer.eventtime())?;
        self.cache
            .borrow_mut()
            .insert(name.to_string(), status.clone());
        Some(status)
    }

    /// `'name' in printer` (`GetStatusWrapper.__contains__`, `:33-37`): an
    /// object nobody registered is absent, not an error.
    fn has(&self, name: &str) -> bool {
        self.printer.lookup_object(name).is_some()
    }
}

/// Python truthiness for the values templates branch on.
fn truthy(value: &Rt) -> bool {
    match value {
        Rt::Json(Value::Null) => false,
        Rt::Json(Value::Bool(flag)) => *flag,
        Rt::Json(Value::Number(number)) => number.as_f64().is_some_and(|n| n != 0.0),
        Rt::Json(Value::String(text)) => !text.is_empty(),
        Rt::Json(Value::Array(items)) => !items.is_empty(),
        Rt::Json(Value::Object(map)) => !map.is_empty(),
        Rt::List(items) => !items.is_empty(),
        Rt::Printer(_) | Rt::Builtin(_) => true,
    }
}

/// `str(value)` for a rendered `{ … }` (`gcode_macro.py` renders with Jinja's
/// `str`): Python's spelling for scalars, compact JSON for containers (Python
/// would print a list/dict repr — no corpus template renders one).
fn to_text(value: &Rt) -> Result<String, String> {
    match value {
        Rt::Json(Value::String(text)) => Ok(text.clone()),
        Rt::Json(Value::Bool(flag)) => Ok(if *flag { "True" } else { "False" }.to_string()),
        Rt::Json(Value::Null) => Ok("None".to_string()),
        Rt::Json(Value::Number(number)) => {
            if let Some(int) = number.as_i64() {
                Ok(int.to_string())
            } else {
                Ok(fmt_float(number.as_f64().unwrap_or(f64::NAN)))
            }
        }
        Rt::Json(Value::Array(_) | Value::Object(_)) => {
            serde_json::to_string(value_json(value)).map_err(|error| error.to_string())
        }
        Rt::List(items) => {
            let mut parts = Vec::with_capacity(items.len());
            for item in items {
                parts.push(to_text(item)?);
            }
            Ok(format!("[{}]", parts.join(", ")))
        }
        Rt::Printer(_) => Err("a printer status object cannot be rendered as text".to_string()),
        Rt::Builtin(_) => Err("a builtin function cannot be rendered as text".to_string()),
    }
}

/// A float the way Python's `str` writes it: `3.0`, not `3`.
fn fmt_float(value: f64) -> String {
    if value.is_finite() {
        format!("{value:?}")
    } else if value.is_nan() {
        "nan".to_string()
    } else if value.is_sign_positive() {
        "inf".to_string()
    } else {
        "-inf".to_string()
    }
}

fn value_json(value: &Rt) -> &Value {
    match value {
        Rt::Json(inner) => inner,
        _ => unreachable!("only JSON containers reach to_text's serializer"),
    }
}

/// `==` / `!=`, Python-flavoured: numbers compare across int/float, other
/// types compare within themselves.
fn equals(left: &Rt, right: &Rt) -> bool {
    match (left, right) {
        (Rt::Json(a), Rt::Json(b)) => json_equals(a, b),
        _ => false,
    }
}

fn json_equals(left: &Value, right: &Value) -> bool {
    match (left, right) {
        (Value::Number(a), Value::Number(b)) => {
            a.as_f64().is_some() && b.as_f64().is_some() && a.as_f64() == b.as_f64()
        }
        // Python compares `True == 1`; fold booleans into the number side.
        (Value::Bool(a), Value::Number(_)) => json_equals(&json!(i64::from(*a)), right),
        (Value::Number(_), Value::Bool(b)) => json_equals(left, &json!(i64::from(*b))),
        _ => left == right,
    }
}

/// Both operands as floats when they are numbers (or booleans, which Python
/// compares as `0`/`1`).
fn as_numbers(left: &Rt, right: &Rt) -> Option<(f64, f64)> {
    fn number(value: &Rt) -> Option<f64> {
        match value {
            Rt::Json(Value::Number(n)) => n.as_f64(),
            Rt::Json(Value::Bool(flag)) => Some(if *flag { 1.0 } else { 0.0 }),
            _ => None,
        }
    }
    Some((number(left)?, number(right)?))
}

/// Python's `+` on the types templates concatenate: numbers add, strings
/// join. Everything else is an explicit error.
fn add(left: &Rt, right: &Rt) -> Result<Rt, String> {
    if let Some((a, b)) = as_numbers(left, right) {
        return Ok(Rt::Json(json_number(left, right, a + b)));
    }
    if let (Rt::Json(Value::String(a)), Rt::Json(Value::String(b))) = (left, right) {
        return Ok(Rt::Json(Value::String(format!("{a}{b}"))));
    }
    Err(format!(
        "unsupported operand types for +: {} and {}",
        type_name(left),
        type_name(right)
    ))
}

/// Keep int arithmetic integral, the way Python does (`12.0 - 12.0` is a
/// float, `3 + 4` is an int).
fn json_number(left: &Rt, right: &Rt, value: f64) -> Value {
    let both_int = matches!(
        (left, right),
        (Rt::Json(Value::Number(a)), Rt::Json(Value::Number(b)))
            if a.as_f64().is_some_and(|n| n.fract() == 0.0)
                && b.as_f64().is_some_and(|n| n.fract() == 0.0)
                && a.is_i64()
                && b.is_i64()
    );
    if both_int {
        json!(value as i64)
    } else {
        json!(value)
    }
}

fn type_name(value: &Rt) -> &'static str {
    match value {
        Rt::Json(Value::String(_)) => "str",
        Rt::Json(Value::Bool(_)) | Rt::Json(Value::Number(_)) => "number",
        Rt::Json(Value::Null) => "NoneType",
        Rt::Json(Value::Array(_)) => "list",
        Rt::Json(Value::Object(_)) => "dict",
        Rt::List(_) => "list",
        Rt::Printer(_) => "printer",
        Rt::Builtin(_) => "builtin_function_or_method",
    }
}

// ===========================================================================
// Context
// ===========================================================================

/// The names a render resolves against: the globals a macro builds (printer,
/// actions, `params`, `rawparams`, the macro's `variable_*` values) plus the
/// frame each `{% for %}` iteration pushes.
#[derive(Debug, Default)]
pub struct Context {
    globals: HashMap<String, Rt>,
    frames: Vec<HashMap<String, Rt>>,
}

impl Context {
    /// An empty context; the caller binds what the template may see.
    pub fn new() -> Self {
        Self::default()
    }

    /// Bind a global (`create_template_context` plus `kwparams`,
    /// `gcode_macro.py:186-190`).
    pub fn insert(&mut self, name: impl Into<String>, value: Rt) {
        self.globals.insert(name.into(), value);
    }

    fn lookup(&self, name: &str) -> Option<&Rt> {
        self.frames
            .iter()
            .rev()
            .find_map(|frame| frame.get(name))
            .or_else(|| self.globals.get(name))
    }

    fn push_frame(&mut self) {
        self.frames.push(HashMap::new());
    }

    fn pop_frame(&mut self) {
        self.frames.pop();
    }

    fn bind(&mut self, name: &str, value: Rt) {
        if let Some(frame) = self.frames.last_mut() {
            frame.insert(name.to_string(), value);
        }
    }
}

// ===========================================================================
// Parse: source → nodes
// ===========================================================================

/// A compiled template: its name (`gcode_macro M486:gcode`, the upstream
/// `TemplateWrapper` name, `gcode_macro.py:87`) and its parsed nodes.
#[derive(Debug)]
pub struct Template {
    name: String,
    nodes: Vec<Node>,
}

impl Template {
    /// Compile `source`, reporting an unparsable construct with upstream's
    /// load-error frame (`gcode_macro.py:61-66`).
    ///
    /// # Errors
    /// An unclosed tag, a statement outside `if/elif/else/endif/for/endfor`,
    /// an unbalanced block, or an expression this port's grammar refuses.
    pub fn parse(name: &str, source: &str) -> Result<Self, TemplateError> {
        let mut parser = Parser {
            name,
            src: source,
            pos: 0,
        };
        let (nodes, terminator) = parser.parse_nodes(&[])?;
        if let Some(stmt) = terminator {
            let keyword = keyword_of(&stmt);
            return Err(TemplateError::load(
                name,
                parser.line_at(parser.src.len()),
                format!("unexpected '{keyword}', no block is open"),
            ));
        }
        Ok(Self {
            name: name.to_string(),
            nodes,
        })
    }

    /// The template's upstream-style name (`<section>:<option>`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Render against `context`, or report the first failing expression with
    /// upstream's `Error evaluating` frame (`gcode_macro.py:70-79`).
    ///
    /// The body runs inside one frame: [`Context::bind`] writes the innermost
    /// frame, so a top-level `{% set %}` needs somewhere to land (an empty
    /// `frames` would drop it silently). The frame is popped with the render,
    /// so an assignment never survives into the next render, while `{% for %}`
    /// still pushes its own frame on top — a loop-body `set` dies with the
    /// iteration, and an `if` at the top level shares the body frame, exactly
    /// where Jinja2 scopes those assignments.
    pub fn render(&self, context: &mut Context) -> Result<String, TemplateError> {
        let mut out = String::new();
        context.push_frame();
        let rendered = render_nodes(&self.nodes, context, &self.name, &mut out);
        context.pop_frame();
        rendered?;
        Ok(out)
    }
}

/// One node of a parsed template.
#[derive(Debug)]
enum Node {
    /// Literal text between tags.
    Text(String),
    /// `{ … }` — rendered as `str(value)`.
    Expr(Expr),
    /// `{% if %}` … (`branches`: condition + body) with its optional `else`.
    If {
        branches: Vec<(Expr, Vec<Node>)>,
        otherwise: Option<Vec<Node>>,
    },
    /// `{% for name in … %}` … `{% endfor %}`.
    For {
        var: String,
        iter: Expr,
        body: Vec<Node>,
        line: usize,
    },
    /// `{% set name = expr %}` — bound where Jinja2 scopes it: into the
    /// body frame at the top level (so a later node, or an `if` body, sees
    /// it), into the loop frame inside `{% for %}` (so it dies with the
    /// iteration).
    Set { name: String, value: Expr },
}

struct Parser<'a> {
    name: &'a str,
    src: &'a str,
    pos: usize,
}

impl<'a> Parser<'a> {
    /// The 1-based line `offset` falls on.
    fn line_at(&self, offset: usize) -> usize {
        self.src[..offset.min(self.src.len())].matches('\n').count() + 1
    }

    fn load_error(&self, offset: usize, detail: impl Into<String>) -> TemplateError {
        TemplateError::load(self.name, self.line_at(offset), detail)
    }

    /// Parse nodes until the template ends or one of `stop`'s statements
    /// appears; returns the nodes and the terminator's statement text (`else`,
    /// `elif …`, `endif`, `endfor`) when one did.
    fn parse_nodes(&mut self, stop: &[&str]) -> Result<(Vec<Node>, Option<String>), TemplateError> {
        let mut nodes = Vec::new();
        loop {
            let Some(brace) = self.src[self.pos..].find('{') else {
                if self.pos < self.src.len() {
                    nodes.push(Node::Text(self.src[self.pos..].to_string()));
                    self.pos = self.src.len();
                }
                return Ok((nodes, None));
            };
            let open = self.pos + brace;
            if open > self.pos {
                nodes.push(Node::Text(self.src[self.pos..open].to_string()));
            }
            let rest = &self.src[open..];

            if rest.starts_with("{%") {
                let close = rest[2..]
                    .find("%}")
                    .map(|offset| open + 2 + offset)
                    .ok_or_else(|| self.load_error(open, "unclosed block tag, '%}' expected"))?;
                let stmt = self.src[open + 2..close].trim().to_string();
                self.pos = close + 2;
                let keyword = keyword_of(&stmt);
                if stop.contains(&keyword.as_str()) {
                    return Ok((nodes, Some(stmt)));
                }
                let line = self.line_at(open);
                let node = match keyword.as_str() {
                    "if" => self.parse_if(&stmt, line)?,
                    "for" => self.parse_for(&stmt, line)?,
                    "set" => self.parse_set(&stmt, line)?,
                    "elif" | "else" | "endif" | "endfor" => {
                        return Err(self.load_error(
                            open,
                            format!("unexpected '{keyword}', no matching block is open"),
                        ));
                    }
                    other => {
                        return Err(self.load_error(
                            open,
                            format!(
                                "unsupported statement '{other}' \
                                 (this port implements if/elif/else/endif/for/endfor/set)"
                            ),
                        ));
                    }
                };
                nodes.push(node);
                continue;
            }

            if rest.starts_with("{#") {
                let close = rest[2..]
                    .find("#}")
                    .map(|offset| open + 2 + offset)
                    .ok_or_else(|| self.load_error(open, "unclosed comment, '#}' expected"))?;
                self.pos = close + 2;
                continue;
            }

            // `{ expression }` — the environment's variable tag
            // (`jinja2.Environment('{%', '%}', '{', '}')`).
            let close = find_expr_end(rest)
                .map(|offset| open + offset)
                .ok_or_else(|| self.load_error(open, "unclosed expression, '}' expected"))?;
            let text = &rest[1..close - open];
            let line = self.line_at(open);
            let expr = parse_expr(self.name, line, text)?;
            nodes.push(Node::Expr(expr));
            self.pos = close + 1;
        }
    }

    /// `{% if … %}` … with its `elif`/`else` chain.
    fn parse_if(&mut self, stmt: &str, line: usize) -> Result<Node, TemplateError> {
        let mut branches = Vec::new();
        let mut otherwise = None;
        let mut stmt = stmt.to_string();
        loop {
            let condition = stmt
                .trim_start()
                .strip_prefix("if")
                .or_else(|| stmt.trim_start().strip_prefix("elif"))
                .map(str::trim)
                .ok_or_else(|| {
                    self.load_error(self.pos, format!("malformed condition in '{stmt}'"))
                })?;
            let expr = parse_expr(self.name, line, condition)?;
            let (body, terminator) = self.parse_nodes(&["elif", "else", "endif"])?;
            branches.push((expr, body));
            let Some(terminator) = terminator else {
                return Err(TemplateError::load(
                    self.name,
                    line,
                    "unexpected end of template, 'endif' expected",
                ));
            };
            match keyword_of(&terminator).as_str() {
                "endif" => {
                    return Ok(Node::If {
                        branches,
                        otherwise,
                    })
                }
                "else" => {
                    let (body, terminator) = self.parse_nodes(&["endif"])?;
                    otherwise = Some(body);
                    match terminator {
                        Some(_) => {
                            return Ok(Node::If {
                                branches,
                                otherwise,
                            })
                        }
                        None => {
                            return Err(TemplateError::load(
                                self.name,
                                line,
                                "unexpected end of template, 'endif' expected",
                            ))
                        }
                    }
                }
                // `elif …`: loop with the new condition.
                "elif" => {
                    stmt = terminator;
                }
                other => {
                    return Err(TemplateError::load(
                        self.name,
                        line,
                        format!("unexpected '{other}', no block is open"),
                    ))
                }
            }
        }
    }

    /// `{% for name in … %}` … `{% endfor %}`.
    fn parse_for(&mut self, stmt: &str, line: usize) -> Result<Node, TemplateError> {
        let rest = stmt
            .trim_start()
            .strip_prefix("for")
            .map(str::trim)
            .ok_or_else(|| self.load_error(self.pos, format!("malformed loop in '{stmt}'")))?;
        let name_end = rest.find(char::is_whitespace).unwrap_or_else(|| rest.len());
        let var = &rest[..name_end];
        if var.is_empty() || !var.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            return Err(self.load_error(self.pos, format!("malformed loop variable in '{stmt}'")));
        }
        let after = rest[name_end..].trim_start();
        let Some(expr_text) = after.strip_prefix("in").map(str::trim) else {
            return Err(self.load_error(self.pos, format!("expected 'in' in loop '{stmt}'")));
        };
        if expr_text.is_empty() {
            return Err(self.load_error(self.pos, format!("missing loop iterable in '{stmt}'")));
        }
        let expr = parse_expr(self.name, line, expr_text)?;
        let (body, terminator) = self.parse_nodes(&["endfor"])?;
        if terminator.is_none() {
            return Err(TemplateError::load(
                self.name,
                line,
                "unexpected end of template, 'endfor' expected",
            ));
        }
        Ok(Node::For {
            var: var.to_string(),
            iter: expr,
            body,
            line,
        })
    }

    /// `{% set name = expr %}` — one name and one expression; tuple targets
    /// and `set` without `=` are refused the way a malformed loop is.
    fn parse_set(&mut self, stmt: &str, line: usize) -> Result<Node, TemplateError> {
        let rest = stmt
            .trim_start()
            .strip_prefix("set")
            .map(str::trim)
            .ok_or_else(|| {
                self.load_error(self.pos, format!("malformed assignment in '{stmt}'"))
            })?;
        let name_end = rest
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
            .unwrap_or_else(|| rest.len());
        let name = &rest[..name_end];
        if name.is_empty() || name.starts_with(|c: char| c.is_ascii_digit()) {
            return Err(
                self.load_error(self.pos, format!("malformed assignment target in '{stmt}'"))
            );
        }
        let after = rest[name_end..].trim_start();
        let Some(expr_text) = after.strip_prefix('=').map(str::trim) else {
            return Err(self.load_error(self.pos, format!("expected '=' in assignment '{stmt}'")));
        };
        if expr_text.is_empty() {
            return Err(self.load_error(self.pos, format!("missing value in assignment '{stmt}'")));
        }
        let value = parse_expr(self.name, line, expr_text)?;
        Ok(Node::Set {
            name: name.to_string(),
            value,
        })
    }
}

/// The first word of a statement text.
fn keyword_of(stmt: &str) -> String {
    stmt.split_whitespace()
        .next()
        .unwrap_or_default()
        .to_string()
}

/// The offset of the `}` that closes a `{ expression }`, skipping braces
/// inside string literals.
fn find_expr_end(text: &str) -> Option<usize> {
    let bytes = text.as_bytes();
    let mut quote: Option<u8> = None;
    let mut escaped = false;
    for (index, &byte) in bytes.iter().enumerate().skip(1) {
        match quote {
            Some(current) => {
                if escaped {
                    escaped = false;
                } else if byte == b'\\' {
                    escaped = true;
                } else if byte == current {
                    quote = None;
                }
            }
            None => match byte {
                b'\'' | b'"' => quote = Some(byte),
                b'}' => return Some(index),
                _ => {}
            },
        }
    }
    None
}

// ===========================================================================
// Parse: expressions
// ===========================================================================

/// One expression token and the line it starts on.
#[derive(Debug, Clone)]
enum Tok {
    /// A number: `is_int` distinguishes `3` from `3.0`.
    Num {
        text: String,
        is_int: bool,
    },
    Str(String),
    Name(String),
    Op(String),
}

/// Compile one expression's text (already stripped of its delimiters).
fn parse_expr(name: &str, line: usize, text: &str) -> Result<Expr, TemplateError> {
    let toks = tokenize(name, line, text)?;
    let mut expr = ExprParser {
        name,
        toks,
        pos: 0,
        line,
    };
    let parsed = expr.parse_or()?;
    if let Some((tok, tok_line)) = expr.peek() {
        return Err(TemplateError::load(
            name,
            *tok_line,
            format!("unexpected token '{}' in expression", describe_tok(tok)),
        ));
    }
    Ok(parsed)
}

fn describe_tok(tok: &Tok) -> String {
    match tok {
        Tok::Num { text, .. } => text.clone(),
        Tok::Str(text) => format!("'{text}'"),
        Tok::Name(text) => text.clone(),
        Tok::Op(text) => text.clone(),
    }
}

/// Split an expression into tokens, tracking each token's line.
fn tokenize(name: &str, line: usize, text: &str) -> Result<Vec<(Tok, usize)>, TemplateError> {
    let mut toks = Vec::new();
    let mut current_line = line;
    let mut rest = text;
    while !rest.is_empty() {
        let c = rest.chars().next().expect("non-empty");
        if c.is_whitespace() {
            let consumed = rest.len() - rest[c.len_utf8()..].len();
            current_line += rest[..consumed].matches('\n').count();
            rest = &rest[consumed..];
            continue;
        }

        let start_line = current_line;
        if c == '\'' || c == '"' {
            let quote = c;
            let mut value = String::new();
            let mut chars = rest[1..].chars();
            let mut closed = false;
            while let Some(ch) = chars.next() {
                if ch == '\\' {
                    match chars.next() {
                        Some('n') => value.push('\n'),
                        Some('t') => value.push('\t'),
                        Some('r') => value.push('\r'),
                        Some(other) => value.push(other),
                        None => {
                            return Err(TemplateError::load(
                                name,
                                start_line,
                                "unterminated string literal".to_string(),
                            ))
                        }
                    }
                } else if ch == quote {
                    closed = true;
                    break;
                } else {
                    value.push(ch);
                }
            }
            if !closed {
                return Err(TemplateError::load(
                    name,
                    start_line,
                    "unterminated string literal".to_string(),
                ));
            }
            // `consumed` counts the whole literal, quotes included.
            let consumed = rest.len() - chars.as_str().len();
            rest = &rest[consumed..];
            toks.push((Tok::Str(value), start_line));
            continue;
        }
        if c.is_ascii_digit() {
            let end = rest
                .find(|ch: char| !ch.is_ascii_digit() && ch != '.')
                .unwrap_or(rest.len());
            let text = &rest[..end];
            if text.matches('.').count() > 1 {
                return Err(TemplateError::load(
                    name,
                    start_line,
                    format!("malformed number '{text}'"),
                ));
            }
            toks.push((
                Tok::Num {
                    text: text.to_string(),
                    is_int: !text.contains('.'),
                },
                start_line,
            ));
            rest = &rest[end..];
            continue;
        }
        if c.is_ascii_alphabetic() || c == '_' {
            let end = rest
                .find(|ch: char| !(ch.is_ascii_alphanumeric() || ch == '_'))
                .unwrap_or(rest.len());
            toks.push((Tok::Name(rest[..end].to_string()), start_line));
            rest = &rest[end..];
            continue;
        }
        // Operators: the two-character forms first, then one character —
        // sliced by `len_utf8` so a multi-byte character is reported, never
        // cut in half.
        let op = match rest.get(..2) {
            Some(two @ ("==" | "!=" | "<=" | ">=")) => {
                rest = &rest[2..];
                two.to_string()
            }
            _ => match c {
                '<' | '>' | '+' | '-' | '*' | '/' | '%' | '|' | '.' | '(' | ')' | '[' | ']'
                | ',' => {
                    rest = &rest[c.len_utf8()..];
                    c.to_string()
                }
                _ => {
                    return Err(TemplateError::load(
                        name,
                        start_line,
                        format!("unexpected character '{c}' in expression"),
                    ))
                }
            },
        };
        toks.push((Tok::Op(op), start_line));
    }
    Ok(toks)
}

/// One parsed expression, with the line it started on (for render errors).
#[derive(Debug)]
pub struct Expr {
    kind: ExprKind,
    line: usize,
}

#[derive(Debug)]
enum ExprKind {
    Literal(Value),
    Name(String),
    Attr(Box<Expr>, String),
    Index(Box<Expr>, Box<Expr>),
    Call(Box<Expr>, Vec<Expr>),
    Filter(Box<Expr>, String, Vec<Expr>),
    /// `x is defined` / `x is not defined` (`sdcard_loop.cfg:90`).
    IsDefined {
        negated: bool,
        test: Box<Expr>,
    },
    Not(Box<Expr>),
    Neg(Box<Expr>),
    Arith(Arith, Box<Expr>, Box<Expr>),
    Compare(Cmp, Box<Expr>, Box<Expr>),
    Membership {
        negated: bool,
        needle: Box<Expr>,
        haystack: Box<Expr>,
    },
    Bool(BoolOp, Box<Expr>, Box<Expr>),
}

#[derive(Debug, Clone, Copy)]
enum Arith {
    Add,
    Sub,
    Mul,
    Div,
    Mod,
}

#[derive(Debug, Clone, Copy)]
enum Cmp {
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
}

#[derive(Debug, Clone, Copy)]
enum BoolOp {
    And,
    Or,
}

impl Expr {
    /// The expression's source shape, for error messages
    /// (`printer.toolhead has no attribute 'extruder'`).
    fn describe(&self) -> String {
        match &self.kind {
            ExprKind::Literal(value) => match value {
                Value::String(text) => format!("'{text}'"),
                Value::Bool(flag) => (if *flag { "True" } else { "False" }).to_string(),
                Value::Null => "None".to_string(),
                other => other.to_string(),
            },
            ExprKind::Name(name) => name.clone(),
            ExprKind::Attr(base, key) => format!("{}.{}", base.describe(), key),
            ExprKind::Index(base, index) => {
                format!("{}[{}]", base.describe(), index.describe())
            }
            ExprKind::Call(callee, args) => {
                let args: Vec<String> = args.iter().map(Expr::describe).collect();
                format!("{}({})", callee.describe(), args.join(", "))
            }
            ExprKind::Filter(base, filter, args) => {
                if args.is_empty() {
                    format!("{} | {}", base.describe(), filter)
                } else {
                    let args: Vec<String> = args.iter().map(Expr::describe).collect();
                    format!("{} | {}({})", base.describe(), filter, args.join(", "))
                }
            }
            ExprKind::IsDefined { negated, test } => {
                format!(
                    "{} is {}defined",
                    test.describe(),
                    if *negated { "not " } else { "" }
                )
            }
            ExprKind::Not(inner) => format!("not {}", inner.describe()),
            ExprKind::Neg(inner) => format!("-{}", inner.describe()),
            ExprKind::Arith(op, left, right) => format!(
                "{} {} {}",
                left.describe(),
                match op {
                    Arith::Add => "+",
                    Arith::Sub => "-",
                    Arith::Mul => "*",
                    Arith::Div => "/",
                    Arith::Mod => "%",
                },
                right.describe()
            ),
            ExprKind::Compare(op, left, right) => format!(
                "{} {} {}",
                left.describe(),
                match op {
                    Cmp::Eq => "==",
                    Cmp::Ne => "!=",
                    Cmp::Lt => "<",
                    Cmp::Le => "<=",
                    Cmp::Gt => ">",
                    Cmp::Ge => ">=",
                },
                right.describe()
            ),
            ExprKind::Membership {
                negated,
                needle,
                haystack,
            } => format!(
                "{} {}in {}",
                needle.describe(),
                if *negated { "not " } else { "" },
                haystack.describe()
            ),
            ExprKind::Bool(op, left, right) => format!(
                "{} {} {}",
                left.describe(),
                match op {
                    BoolOp::And => "and",
                    BoolOp::Or => "or",
                },
                right.describe()
            ),
        }
    }
}

/// Recursive-descent parser over one expression's tokens. Precedence follows
/// Jinja2: `or` < `not` < comparisons/`in`/`is` < `+ -` < `* / %` < unary `-`
/// < `|` < postfix (`a.b`, `a[i]`, `a(…)`).
struct ExprParser<'a> {
    name: &'a str,
    toks: Vec<(Tok, usize)>,
    pos: usize,
    line: usize,
}

impl<'a> ExprParser<'a> {
    fn peek(&self) -> Option<&(Tok, usize)> {
        self.toks.get(self.pos)
    }

    fn line_here(&self) -> usize {
        self.toks
            .get(self.pos)
            .or(self.toks.last())
            .map(|(_, line)| *line)
            .unwrap_or(self.line)
    }

    fn error(&self, detail: impl Into<String>) -> TemplateError {
        TemplateError::load(self.name, self.line_here(), detail)
    }

    fn is_op(&self, op: &str) -> bool {
        matches!(self.peek(), Some((Tok::Op(word), _)) if word == op)
    }

    fn is_name(&self, name: &str) -> bool {
        matches!(self.peek(), Some((Tok::Name(word), _)) if word == name)
    }

    fn bump(&mut self) -> Option<Tok> {
        let tok = self.toks.get(self.pos).map(|(tok, _)| tok.clone());
        if tok.is_some() {
            self.pos += 1;
        }
        tok
    }

    fn expect_op(&mut self, op: &str) -> Result<(), TemplateError> {
        if self.is_op(op) {
            self.bump();
            Ok(())
        } else {
            Err(self.error(format!("expected '{op}'")))
        }
    }

    fn parse_or(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_and()?;
        while self.is_name("or") {
            self.bump();
            let line = self.line_here();
            let right = self.parse_and()?;
            left = Expr {
                kind: ExprKind::Bool(BoolOp::Or, Box::new(left), Box::new(right)),
                line,
            };
        }
        Ok(left)
    }

    fn parse_and(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_not()?;
        while self.is_name("and") {
            self.bump();
            let line = self.line_here();
            let right = self.parse_not()?;
            left = Expr {
                kind: ExprKind::Bool(BoolOp::And, Box::new(left), Box::new(right)),
                line,
            };
        }
        Ok(left)
    }

    fn parse_not(&mut self) -> Result<Expr, TemplateError> {
        if self.is_name("not") {
            let line = self.line_here();
            self.bump();
            let inner = self.parse_not()?;
            return Ok(Expr {
                kind: ExprKind::Not(Box::new(inner)),
                line,
            });
        }
        self.parse_compare()
    }

    fn parse_compare(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_add()?;
        loop {
            let line = self.line_here();
            let comparison = match self.peek().cloned() {
                Some((Tok::Op(op), _)) if Cmp::from(&op).is_some() => {
                    let op = Cmp::from(&op).expect("checked");
                    self.bump();
                    let right = self.parse_add()?;
                    ExprKind::Compare(op, Box::new(left), Box::new(right))
                }
                Some((Tok::Name(word), _)) if word == "in" => {
                    self.bump();
                    let haystack = self.parse_add()?;
                    ExprKind::Membership {
                        negated: false,
                        needle: Box::new(left),
                        haystack: Box::new(haystack),
                    }
                }
                Some((Tok::Name(word), _)) if word == "not" => {
                    // `x not in y` — but `x not …` otherwise is a syntax gap.
                    if !matches!(self.toks.get(self.pos + 1), Some((Tok::Name(w), _)) if w == "in")
                    {
                        return Err(self.error(
                            "expected 'in' after 'not' \
                             (only 'not in' is supported here)",
                        ));
                    }
                    self.pos += 2;
                    let haystack = self.parse_add()?;
                    ExprKind::Membership {
                        negated: true,
                        needle: Box::new(left),
                        haystack: Box::new(haystack),
                    }
                }
                Some((Tok::Name(word), _)) if word == "is" => {
                    self.bump();
                    let negated = if self.is_name("not") {
                        self.bump();
                        true
                    } else {
                        false
                    };
                    let Some((Tok::Name(test), _)) = self.peek().cloned() else {
                        return Err(self.error("expected a test name after 'is'"));
                    };
                    self.bump();
                    if test != "defined" {
                        return Err(self.error(format!(
                            "unsupported test '{test}' (this port implements 'is defined')"
                        )));
                    }
                    ExprKind::IsDefined {
                        negated,
                        test: Box::new(left),
                    }
                }
                _ => break,
            };
            left = Expr {
                kind: comparison,
                line,
            };
            // Python's chained comparisons (`a < b < c`) are not implemented.
            let chained = match self.peek() {
                Some((Tok::Op(op), _)) => Cmp::from(op).is_some(),
                Some((Tok::Name(word), _)) => matches!(word.as_str(), "in" | "is"),
                _ => false,
            };
            if chained {
                return Err(self.error("chained comparisons are not supported"));
            }
        }
        Ok(left)
    }

    fn parse_add(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_mul()?;
        loop {
            let line = self.line_here();
            let op = match self.peek().cloned() {
                Some((Tok::Op(op), _)) if op == "+" => Arith::Add,
                Some((Tok::Op(op), _)) if op == "-" => Arith::Sub,
                _ => break,
            };
            self.bump();
            let right = self.parse_mul()?;
            left = Expr {
                kind: ExprKind::Arith(op, Box::new(left), Box::new(right)),
                line,
            };
        }
        Ok(left)
    }

    fn parse_mul(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_unary()?;
        loop {
            let line = self.line_here();
            let op = match self.peek().cloned() {
                Some((Tok::Op(op), _)) if op == "*" => Arith::Mul,
                Some((Tok::Op(op), _)) if op == "/" => Arith::Div,
                Some((Tok::Op(op), _)) if op == "%" => Arith::Mod,
                _ => break,
            };
            self.bump();
            let right = self.parse_unary()?;
            left = Expr {
                kind: ExprKind::Arith(op, Box::new(left), Box::new(right)),
                line,
            };
        }
        Ok(left)
    }

    fn parse_unary(&mut self) -> Result<Expr, TemplateError> {
        if self.is_op("-") {
            let line = self.line_here();
            self.bump();
            let inner = self.parse_unary()?;
            return Ok(Expr {
                kind: ExprKind::Neg(Box::new(inner)),
                line,
            });
        }
        self.parse_filter()
    }

    /// `expr | name` with Jinja's optional argument list, `expr | name(a, b)`
    /// — `params.S | default(1000.0) | float`
    /// (`printer-velleman-k8800-2017.cfg:125`).
    fn parse_filter(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_postfix()?;
        while self.is_op("|") {
            let line = self.line_here();
            self.bump();
            let Some((Tok::Name(filter), _)) = self.peek().cloned() else {
                return Err(self.error("expected a filter name after '|'"));
            };
            self.bump();
            let mut args = Vec::new();
            if self.is_op("(") {
                self.bump();
                if !self.is_op(")") {
                    loop {
                        args.push(self.parse_or()?);
                        if self.is_op(",") {
                            self.bump();
                            continue;
                        }
                        break;
                    }
                }
                self.expect_op(")")?;
            }
            left = Expr {
                kind: ExprKind::Filter(Box::new(left), filter, args),
                line,
            };
        }
        Ok(left)
    }

    fn parse_postfix(&mut self) -> Result<Expr, TemplateError> {
        let mut expr = self.parse_primary()?;
        loop {
            let line = self.line_here();
            if self.is_op(".") {
                self.bump();
                let Some((Tok::Name(key), _)) = self.peek().cloned() else {
                    return Err(self.error("expected an attribute name after '.'"));
                };
                self.bump();
                expr = Expr {
                    kind: ExprKind::Attr(Box::new(expr), key),
                    line,
                };
                continue;
            }
            if self.is_op("[") {
                self.bump();
                let index = self.parse_or()?;
                self.expect_op("]")?;
                expr = Expr {
                    kind: ExprKind::Index(Box::new(expr), Box::new(index)),
                    line,
                };
                continue;
            }
            if self.is_op("(") {
                self.bump();
                let mut args = Vec::new();
                if !self.is_op(")") {
                    loop {
                        args.push(self.parse_or()?);
                        if self.is_op(",") {
                            self.bump();
                            continue;
                        }
                        break;
                    }
                }
                self.expect_op(")")?;
                expr = Expr {
                    kind: ExprKind::Call(Box::new(expr), args),
                    line,
                };
                continue;
            }
            break;
        }
        Ok(expr)
    }

    fn parse_primary(&mut self) -> Result<Expr, TemplateError> {
        let line = self.line_here();
        let Some(tok) = self.peek().cloned() else {
            return Err(self.error("unexpected end of expression"));
        };
        match tok {
            (Tok::Num { text, is_int }, _) => {
                self.bump();
                let value = if is_int {
                    json!(text.parse::<i64>().map_err(|_| {
                        TemplateError::load(self.name, line, format!("malformed number '{text}'"))
                    })?)
                } else {
                    json!(text.parse::<f64>().map_err(|_| {
                        TemplateError::load(self.name, line, format!("malformed number '{text}'"))
                    })?)
                };
                Ok(Expr {
                    kind: ExprKind::Literal(value),
                    line,
                })
            }
            (Tok::Str(text), _) => {
                self.bump();
                Ok(Expr {
                    kind: ExprKind::Literal(Value::String(text)),
                    line,
                })
            }
            (Tok::Name(word), _) => {
                self.bump();
                match word.as_str() {
                    "True" => Ok(Expr {
                        kind: ExprKind::Literal(json!(true)),
                        line,
                    }),
                    "False" => Ok(Expr {
                        kind: ExprKind::Literal(json!(false)),
                        line,
                    }),
                    "None" => Ok(Expr {
                        kind: ExprKind::Literal(Value::Null),
                        line,
                    }),
                    "and" | "or" | "not" | "in" | "is" | "defined" => {
                        Err(self.error(format!("unexpected '{word}'")))
                    }
                    _ => Ok(Expr {
                        kind: ExprKind::Name(word),
                        line,
                    }),
                }
            }
            (Tok::Op(op), _) if op == "(" => {
                self.bump();
                let inner = self.parse_or()?;
                self.expect_op(")")?;
                Ok(inner)
            }
            other => Err(self.error(format!(
                "unexpected token '{}' in expression",
                describe_tok(&other.0)
            ))),
        }
    }
}

impl Cmp {
    fn from(op: &str) -> Option<Self> {
        Some(match op {
            "==" => Cmp::Eq,
            "!=" => Cmp::Ne,
            "<" => Cmp::Lt,
            "<=" => Cmp::Le,
            ">" => Cmp::Gt,
            ">=" => Cmp::Ge,
            _ => return None,
        })
    }
}

// ===========================================================================
// Render
// ===========================================================================

fn render_nodes(
    nodes: &[Node],
    context: &mut Context,
    name: &str,
    out: &mut String,
) -> Result<(), TemplateError> {
    for node in nodes {
        match node {
            Node::Text(text) => out.push_str(text),
            Node::Expr(expr) => {
                let value = eval(expr, context, name)?;
                out.push_str(
                    &to_text(&value)
                        .map_err(|detail| TemplateError::evaluate(name, expr.line, detail))?,
                );
            }
            Node::If {
                branches,
                otherwise,
            } => {
                let mut done = false;
                for (condition, body) in branches {
                    let value = eval(condition, context, name)?;
                    if truthy(&value) {
                        render_nodes(body, context, name, out)?;
                        done = true;
                        break;
                    }
                }
                if !done {
                    if let Some(body) = otherwise {
                        render_nodes(body, context, name, out)?;
                    }
                }
            }
            Node::For {
                var,
                iter,
                body,
                line,
            } => {
                let value = eval(iter, context, name)?;
                let items = iterate(value).map_err(|detail| {
                    TemplateError::evaluate(name, *line, format!("in loop: {detail}"))
                })?;
                for item in items {
                    context.push_frame();
                    context.bind(var, item);
                    render_nodes(body, context, name, out)?;
                    context.pop_frame();
                }
            }
            Node::Set {
                name: target,
                value,
            } => {
                let value = eval(value, context, name)?;
                context.bind(target, value);
            }
        }
    }
    Ok(())
}

/// The values a `{% for %}` walks: a `range(n)` result, a JSON list or
/// string, or a literal list.
fn iterate(value: Rt) -> Result<Vec<Rt>, String> {
    match value {
        Rt::List(items) => Ok(items),
        Rt::Json(Value::Array(items)) => Ok(items.into_iter().map(Rt::Json).collect()),
        Rt::Json(Value::String(text)) => Ok(text
            .chars()
            .map(|c| Rt::Json(Value::String(c.to_string())))
            .collect()),
        other => Err(format!("cannot iterate over {}", type_name(&other))),
    }
}

fn eval(expr: &Expr, context: &Context, name: &str) -> Result<Rt, TemplateError> {
    let error = |detail: String| TemplateError::evaluate(name, expr.line, detail);
    match &expr.kind {
        ExprKind::Literal(value) => Ok(Rt::Json(value.clone())),
        ExprKind::Name(binding) => match binding.as_str() {
            // Python's keywords are literals in Jinja too.
            "True" => Ok(Rt::Json(json!(true))),
            "False" => Ok(Rt::Json(json!(false))),
            "None" => Ok(Rt::Json(Value::Null)),
            _ => context
                .lookup(binding)
                .cloned()
                .ok_or_else(|| error(format!("'{binding}' is undefined"))),
        },
        ExprKind::Attr(base, key) => {
            let base_value = eval(base, context, name)?;
            attr(&base_value, key)
                .ok_or_else(|| error(format!("{} has no attribute '{key}'", base.describe())))
        }
        ExprKind::Index(base, index) => {
            let base_value = eval(base, context, name)?;
            let index_value = eval(index, context, name)?;
            index_into(&base_value, &index_value).map_err(|detail| error(detail))
        }
        ExprKind::Call(callee, args) => {
            let mut evaluated = Vec::with_capacity(args.len());
            for arg in args {
                evaluated.push(eval(arg, context, name)?);
            }
            call(callee, &evaluated, context, name).map_err(|detail| error(detail))
        }
        ExprKind::Filter(base, filter, args) => {
            // Arity is the filter's own property, so it is reported before the
            // base resolves — a missing `params.S` would mask it.
            check_filter_arity(filter, args.len()).map_err(error)?;
            // `default` must know whether its base *resolves*, so it probes it
            // quietly instead of failing the render the way every other
            // operand does (`params.S|default(…)` with `S` omitted).
            if filter == "default" {
                let fallback = eval(&args[0], context, name)?;
                return Ok(eval_quiet(base, context).unwrap_or(fallback));
            }
            let mut evaluated = Vec::with_capacity(args.len());
            for arg in args {
                evaluated.push(eval(arg, context, name)?);
            }
            let value = eval(base, context, name)?;
            match filter.as_str() {
                // Jinja's `int` filter: `int(value)`, then `int(float(value))`.
                "int" => filter_int(&value).map_err(error),
                // Jinja's `float` filter: `float(value)` or the filter's own
                // default, `0.0` when it has no argument.
                "float" => filter_float(&value, evaluated.first()).map_err(error),
                other => Err(error(format!(
                    "unknown filter '{other}' \
                     (this port implements 'int', 'float' and 'default')"
                ))),
            }
        }
        ExprKind::IsDefined { negated, test } => {
            let found = probe_defined(test, context);
            Ok(Rt::Json(json!(found != *negated)))
        }
        ExprKind::Not(inner) => {
            let value = eval(inner, context, name)?;
            Ok(Rt::Json(json!(!truthy(&value))))
        }
        ExprKind::Neg(inner) => {
            let value = eval(inner, context, name)?;
            match value {
                Rt::Json(Value::Number(number)) => Ok(Rt::Json(json!(-number
                    .as_f64()
                    .ok_or_else(|| error(format!("cannot negate {}", number)))?))),
                other => Err(error(format!(
                    "bad operand type for unary '-': {}",
                    type_name(&other)
                ))),
            }
        }
        ExprKind::Arith(op, left, right) => {
            let left = eval(left, context, name)?;
            let right = eval(right, context, name)?;
            arith(*op, &left, &right).map_err(error)
        }
        ExprKind::Compare(op, left, right) => {
            let left = eval(left, context, name)?;
            let right = eval(right, context, name)?;
            compare(*op, &left, &right).map_err(error)
        }
        ExprKind::Membership {
            negated,
            needle,
            haystack,
        } => {
            let needle = eval(needle, context, name)?;
            let haystack = eval(haystack, context, name)?;
            let found = contains(&needle, &haystack);
            Ok(Rt::Json(json!(found != *negated)))
        }
        ExprKind::Bool(op, left, right) => {
            let left = eval(left, context, name)?;
            match op {
                BoolOp::And => {
                    if !truthy(&left) {
                        return Ok(Rt::Json(json!(false)));
                    }
                    let right = eval(right, context, name)?;
                    Ok(Rt::Json(json!(truthy(&right))))
                }
                BoolOp::Or => {
                    if truthy(&left) {
                        return Ok(Rt::Json(json!(true)));
                    }
                    let right = eval(right, context, name)?;
                    Ok(Rt::Json(json!(truthy(&right))))
                }
            }
        }
    }
}

/// `Rt` clones carry the `printer` view's cache with them; a probe and the
/// render it feeds therefore see the same statuses.

/// `value.key`: status objects by key, JSON arrays by the `Coord` field order
/// (`x y z e`), the `printer` view by object name.
fn attr(base: &Rt, key: &str) -> Option<Rt> {
    match base {
        Rt::Json(Value::Object(map)) => map.get(key).cloned().map(Rt::Json),
        Rt::Json(Value::Array(items)) => coord(items, key).map(Rt::Json),
        Rt::Printer(view) => view.status(key).map(Rt::Json),
        Rt::Json(_) | Rt::List(_) | Rt::Builtin(_) => None,
    }
}

/// The `Coord` namedtuple's field order (`klippy/gcode.py`, `mathutil.rs`).
fn coord(items: &[Value], key: &str) -> Option<Value> {
    let index = match key {
        "x" => 0,
        "y" => 1,
        "z" => 2,
        "e" => 3,
        _ => return None,
    };
    items.get(index).cloned()
}

fn index_into(base: &Rt, index: &Rt) -> Result<Rt, String> {
    match base {
        Rt::Json(Value::Object(map)) => {
            let key = to_text(index)?;
            map.get(&key)
                .cloned()
                .map(Rt::Json)
                .ok_or_else(|| format!("dict has no key '{key}'"))
        }
        Rt::Json(Value::Array(items)) => {
            let offset = as_index(index)?;
            items
                .get(offset)
                .cloned()
                .map(Rt::Json)
                .ok_or_else(|| format!("list index {offset} is out of range"))
        }
        Rt::List(items) => {
            let offset = as_index(index)?;
            match items.get(offset) {
                Some(item) => Ok(item.clone()),
                None => Err(format!("list index {offset} is out of range")),
            }
        }
        Rt::Printer(view) => {
            let key = to_text(index)?;
            view.status(&key)
                .map(Rt::Json)
                .ok_or_else(|| format!("printer has no object '{key}'"))
        }
        Rt::Builtin(_) => Err("a builtin function cannot be indexed".to_string()),
        Rt::Json(_) => Err(format!("{} is not subscriptable", type_name(base))),
    }
}

fn as_index(index: &Rt) -> Result<usize, String> {
    match index {
        Rt::Json(Value::Number(number)) => {
            let value = number.as_f64().ok_or("index is not a number")?;
            if value < 0.0 || value.fract() != 0.0 {
                Err(format!("list index {value} is not an integer"))
            } else {
                Ok(value as usize)
            }
        }
        other => Err(format!(
            "list indices must be integers, not {}",
            type_name(other)
        )),
    }
}

/// `x is defined`: quiet probing, so an undefined name is *false*, never an
/// error (`sdcard_loop.cfg:90`).
fn probe_defined(expr: &Expr, context: &Context) -> bool {
    eval_quiet(expr, context).is_some()
}

/// Evaluate without reporting an error: anything that fails to produce a
/// value is "not defined".
fn eval_quiet(expr: &Expr, context: &Context) -> Option<Rt> {
    eval(expr, context, "").ok()
}

/// `callable(args)` — the context's builtins only.
fn call(callee: &Expr, args: &[Rt], context: &Context, name: &str) -> Result<Rt, String> {
    let ExprKind::Name(binding) = &callee.kind else {
        return Err(format!("{} is not callable", callee.describe()));
    };
    let resolved = context
        .lookup(binding)
        .ok_or_else(|| format!("'{binding}' is undefined"))?;
    match resolved {
        Rt::Builtin(Builtin::Range) => {
            if args.len() != 1 {
                return Err(format!(
                    "range() takes 1 argument here, got {} (this port's gap)",
                    args.len()
                ));
            }
            let stop = match &args[0] {
                Rt::Json(Value::Number(number)) => {
                    let value = number.as_f64().ok_or("range() needs a number")?;
                    if value.fract() != 0.0 {
                        return Err(format!(
                            "'{}' cannot be interpreted as an integer",
                            fmt_float(value)
                        ));
                    }
                    value as i64
                }
                Rt::Json(Value::Bool(flag)) => i64::from(*flag),
                other => {
                    return Err(format!(
                        "'{}' cannot be interpreted as an integer",
                        to_text(other)?
                    ))
                }
            };
            Ok(Rt::List(
                (0..stop.max(0)).map(|n| Rt::Json(json!(n))).collect(),
            ))
        }
        Rt::Builtin(Builtin::RespondInfo(printer)) => {
            let message = to_text(args.first().unwrap_or(&Rt::Json(Value::Null)))?;
            if let Some(gcode) = printer
                .lookup_object_as::<crate::core::klippy::gcode::GCodeDispatch>(
                    crate::core::klippy::gcode::GCODE_OBJECT,
                )
            {
                gcode.respond_info(&message, true);
            }
            let _ = name;
            Ok(Rt::Json(Value::String(String::new())))
        }
        Rt::Builtin(Builtin::RaiseError) => {
            let message = to_text(args.first().unwrap_or(&Rt::Json(Value::Null)))?;
            Err(message)
        }
        Rt::Json(_) | Rt::List(_) | Rt::Printer(_) => {
            Err(format!("{} is not callable", callee.describe()))
        }
    }
}

/// The argument count each implemented filter accepts; anything else is one of
/// this port's gaps, named as such.
fn check_filter_arity(filter: &str, count: usize) -> Result<(), String> {
    match filter {
        "int" if count > 0 => Err(format!(
            "the 'int' filter takes no arguments here, got {count} (this port's gap)"
        )),
        "float" if count > 1 => Err(format!(
            "the 'float' filter takes at most 1 argument here, got {count} (this port's gap)"
        )),
        "default" if count != 1 => Err(format!(
            "the 'default' filter takes 1 argument here, got {count} (this port's gap)"
        )),
        _ => Ok(()),
    }
}

/// `| int`: `int(value)`, falling back to `int(float(value))` as Jinja's
/// filter does.
fn filter_int(value: &Rt) -> Result<Rt, String> {
    match value {
        Rt::Json(Value::Number(number)) => {
            let float = number.as_f64().ok_or("not a number")?;
            Ok(Rt::Json(json!(float.trunc() as i64)))
        }
        Rt::Json(Value::Bool(flag)) => Ok(Rt::Json(json!(i64::from(*flag)))),
        Rt::Json(Value::String(text)) => {
            let trimmed = text.trim();
            if let Ok(int) = trimmed.parse::<i64>() {
                return Ok(Rt::Json(json!(int)));
            }
            if let Ok(float) = trimmed.parse::<f64>() {
                return Ok(Rt::Json(json!(float.trunc() as i64)));
            }
            Err(format!(
                "invalid literal for int(): {text:?} (jinja's 'int' filter)"
            ))
        }
        other => Err(format!("cannot convert {} to an integer", type_name(other))),
    }
}

/// `| float` (and `| float(d)`): `float(value)`, returning the filter's own
/// default — `d`, or `0.0` when it has none — for the types Python's `float()`
/// rejects (`TypeError`) and for strings it cannot parse (`ValueError`).
fn filter_float(value: &Rt, default: Option<&Rt>) -> Result<Rt, String> {
    fn fallback(default: Option<&Rt>) -> Result<Rt, String> {
        Ok(default.cloned().unwrap_or_else(|| Rt::Json(json!(0.0))))
    }
    match value {
        Rt::Json(Value::Number(number)) => match number.as_f64() {
            Some(float) => Ok(Rt::Json(json!(float))),
            None => fallback(default),
        },
        Rt::Json(Value::Bool(flag)) => Ok(Rt::Json(json!(if *flag { 1.0 } else { 0.0 }))),
        Rt::Json(Value::String(text)) => match text.trim().parse::<f64>() {
            Ok(float) => Ok(Rt::Json(json!(float))),
            Err(_) => fallback(default),
        },
        // `None`, a container, `printer`, a builtin: `float()` raises.
        Rt::Json(_) | Rt::List(_) | Rt::Printer(_) | Rt::Builtin(_) => fallback(default),
    }
}

fn contains(needle: &Rt, haystack: &Rt) -> bool {
    match haystack {
        Rt::Printer(view) => view.has(&needle_display(needle)),
        Rt::Json(Value::Object(map)) => map.contains_key(&needle_display(needle)),
        Rt::Json(Value::Array(items)) => items
            .iter()
            .any(|item| equals(needle, &Rt::Json(item.clone()))),
        Rt::Json(Value::String(text)) => text.contains(&needle_display(needle)),
        Rt::List(items) => items.iter().any(|item| equals(needle, item)),
        Rt::Json(_) | Rt::Builtin(_) => false,
    }
}

/// The membership key: `GetStatusWrapper` does `str(val).strip()`
/// (`gcode_macro.py:19-20`), so render the same way.
fn needle_display(needle: &Rt) -> String {
    to_text(needle).unwrap_or_default().trim().to_string()
}

fn arith(op: Arith, left: &Rt, right: &Rt) -> Result<Rt, String> {
    match op {
        Arith::Add => add(left, right),
        Arith::Sub | Arith::Mul | Arith::Div | Arith::Mod => {
            let (a, b) = as_numbers(left, right).ok_or_else(|| {
                format!(
                    "unsupported operand types for {}: {} and {}",
                    match op {
                        Arith::Add => "+",
                        Arith::Sub => "-",
                        Arith::Mul => "*",
                        Arith::Div => "/",
                        Arith::Mod => "%",
                    },
                    type_name(left),
                    type_name(right)
                )
            })?;
            match op {
                Arith::Sub => Ok(Rt::Json(json_number(left, right, a - b))),
                Arith::Mul => Ok(Rt::Json(json_number(left, right, a * b))),
                Arith::Div => {
                    if b == 0.0 {
                        return Err("division by zero".to_string());
                    }
                    Ok(Rt::Json(json!(a / b)))
                }
                Arith::Mod => {
                    if b == 0.0 {
                        return Err("integer modulo by zero".to_string());
                    }
                    // Python's `%` takes the divisor's sign.
                    let mut rem = a % b;
                    if rem != 0.0 && (rem.is_sign_negative() != b.is_sign_negative()) {
                        rem += b;
                    }
                    Ok(Rt::Json(json_number(left, right, rem)))
                }
                Arith::Add => unreachable!("handled in add"),
            }
        }
    }
}

fn compare(op: Cmp, left: &Rt, right: &Rt) -> Result<Rt, String> {
    if let Some((a, b)) = as_numbers(left, right) {
        let result = match op {
            Cmp::Eq => a == b,
            Cmp::Ne => a != b,
            Cmp::Lt => a < b,
            Cmp::Le => a <= b,
            Cmp::Gt => a > b,
            Cmp::Ge => a >= b,
        };
        return Ok(Rt::Json(json!(result)));
    }
    if let (Rt::Json(Value::String(a)), Rt::Json(Value::String(b))) = (left, right) {
        let ordering = a.cmp(b);
        let result = match op {
            Cmp::Eq => ordering.is_eq(),
            Cmp::Ne => ordering.is_ne(),
            Cmp::Lt => ordering.is_lt(),
            Cmp::Le => ordering.is_le(),
            Cmp::Gt => ordering.is_gt(),
            Cmp::Ge => ordering.is_ge(),
        };
        return Ok(Rt::Json(json!(result)));
    }
    match op {
        Cmp::Eq => Ok(Rt::Json(json!(equals(left, right)))),
        Cmp::Ne => Ok(Rt::Json(json!(!equals(left, right)))),
        ordering => Err(format!(
            "unorderable types: {} and {} ({})",
            type_name(left),
            type_name(right),
            match ordering {
                Cmp::Lt => "<",
                Cmp::Le => "<=",
                Cmp::Gt => ">",
                Cmp::Ge => ">=",
                Cmp::Eq | Cmp::Ne => unreachable!(),
            }
        )),
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

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

    /// The scalar spellings are Python's `str`, because Jinja renders through
    /// it — `3.0`, not `3`; `True`, not `true`.
    #[test]
    fn expressions_render_python_spelling() {
        assert_eq!(ok("PARK_{3}"), "PARK_3");
        assert_eq!(ok("{3.0}"), "3.0");
        assert_eq!(ok("{12.0 - 12.0}"), "0.0");
        assert_eq!(ok("{1 + 2}"), "3");
        assert_eq!(ok("{True}/{False}/{None}"), "True/False/None");
        assert_eq!(ok("{ 17 * 2 + 1 % 4 }"), "35");
        assert_eq!(ok("{-3.5}"), "-3.5");
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

    /// Status coordinates are JSON arrays, but klippy's `Coord` namedtuple
    /// exposes `.x`/`.y`/`.z`/`.e` (`mathutil.rs`) — the bridge the corpus
    /// needs (`macros.cfg:34`).
    #[test]
    fn coordinate_attributes_map_onto_json_arrays() {
        let mut context = context(&[], "");
        context.insert("position", Rt::Json(json!([1.5, 2.5, 3.5, 4.5])));
        assert_eq!(
            render(
                "{position.x} {position.y} {position.z} {position.e}",
                &mut context
            )
            .expect("renders"),
            "1.5 2.5 3.5 4.5"
        );
        // Outside the four Coord fields an array attribute is an error.
        let error = render("{position.w}", &mut context).expect_err("no such field");
        assert!(
            error.to_string().contains("position has no attribute 'w'"),
            "{error}"
        );
    }

    /// `rawparams` is the line's tail, verbatim (`gcode_macro.py:189`).
    #[test]
    fn rawparams_renders_the_command_tail() {
        assert_eq!(ok("{rawparams}"), "");
        assert_eq!(
            render("{rawparams}", &mut context(&[], "T=3 EXCLUDE=1")).expect("renders"),
            "T=3 EXCLUDE=1"
        );
    }

    /// A construct outside the subset fails the **load** with upstream's frame
    /// (`gcode_macro.py:61-66`), naming the line and the statement. `{% set %}`
    /// used to be refused here; it is inside the subset now.
    #[test]
    fn an_unknown_statement_is_a_load_error_with_upstream_frame() {
        assert_eq!(ok("{% set x = 1 %}{ x }"), "1");

        let error = Template::parse("gcode_macro SETTY:gcode", "{% block body %}")
            .expect_err("block is not implemented");
        assert_eq!(
            error.to_string(),
            "Error loading template 'gcode_macro SETTY:gcode'\n\
             line 1: unsupported statement 'block' \
             (this port implements if/elif/else/endif/for/endfor/set)"
        );

        // An unbalanced block reads the same way.
        let error = Template::parse("gcode_macro BAD:gcode", "{% if 1 %}").expect_err("no endif");
        assert!(error.to_string().contains("endif' expected"), "{error}");
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
        // from the context here: the lines that set them (286-287) need list
        // literals and `|min`/`|max`, which are unit C's work, as is line 285
        // (it reads `printer.*`).
        let mut context = context(&[], "");
        context.insert("x_max", Rt::Json(json!(300.0)));
        context.insert("x_min", Rt::Json(json!(0.0)));
        let source = "{% set x_center = 0.5 * (x_max + x_min) %}{ x_center }";
        assert_eq!(render(source, &mut context).expect("renders"), "150.0");
    }

    /// A name or filter outside the subset fails the **render**, with the
    /// expression's source in the message.
    #[test]
    fn an_unknown_name_or_filter_is_a_render_error() {
        let error = render("{nope}", &mut context(&[], "")).expect_err("undefined");
        assert_eq!(
            error.to_string(),
            "Error evaluating 'gcode_macro TEST:gcode': line 1: 'nope' is undefined"
        );

        let error =
            render("{params.L | min}", &mut context(&[("L", "1")], "")).expect_err("no filter");
        assert!(
            error.to_string().contains(
                "unknown filter 'min' (this port implements 'int', 'float' and 'default')"
            ),
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
        assert!(error.to_string().contains("'nope' is undefined"), "{error}");
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

    /// The argument counts this port refuses, spelled out rather than
    /// guessed at.
    #[test]
    fn filter_arity_errors_name_the_filter() {
        let error = render("{params.S|float(1, 2)}", &mut context(&[], "")).expect_err("arity");
        assert!(
            error.to_string().contains(
                "the 'float' filter takes at most 1 argument here, got 2 (this port's gap)"
            ),
            "{error}"
        );

        let error = render("{params.S|int(1)}", &mut context(&[], "")).expect_err("arity");
        assert!(
            error
                .to_string()
                .contains("the 'int' filter takes no arguments here, got 1 (this port's gap)"),
            "{error}"
        );

        let error = render("{params.S|default}", &mut context(&[], "")).expect_err("arity");
        assert!(
            error
                .to_string()
                .contains("the 'default' filter takes 1 argument here, got 0 (this port's gap)"),
            "{error}"
        );

        let error = render("{params.S|default(1, 2)}", &mut context(&[], "")).expect_err("arity");
        assert!(
            error
                .to_string()
                .contains("the 'default' filter takes 1 argument here, got 2 (this port's gap)"),
            "{error}"
        );

        // Jinja's other spelling is a keyword argument (`default(0, boolean=True)`);
        // the tokenizer has no `=`, so that stays a load error.
        let error = Template::parse(
            "gcode_macro TEST:gcode",
            "{params.S|default(0, boolean=True)}",
        )
        .expect_err("keyword argument");
        assert!(
            error
                .to_string()
                .contains("unexpected character '=' in expression"),
            "{error}"
        );

        // `int`'s type refusal is unchanged.
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

    /// The two `SET_COPY_MODE` templates (`generic_cartesian_iqex.cfg:282-299`,
    /// `generic_cartesian_itex.cfg:226-256`) carry no `|default`/`|float` at
    /// all: their first load error is the list literal `[a, b]` that a later
    /// unit owns. This pin records which side of the boundary they sit on, so
    /// the template gap is not mistaken for a filter one (when list literals
    /// land, the assertion flips to `is_ok()`).
    #[test]
    fn set_copy_mode_templates_stop_on_list_literals() {
        let iqex = "    G90\n    \
                    {% set y_center = 0.5 * (printer.configfile.settings[\"dual_carriage carriage_gantry1_left\"].position_max + printer.configfile.settings[\"carriage carriage_gantry0_left\"].position_min) %}\n    \
                    {% set x_max = [printer.configfile.settings[\"dual_carriage carriage_t3\"].position_max, printer.configfile.settings[\"dual_carriage carriage_t1\"].position_max]|min %}\n    \
                    {% set x_min = [printer.configfile.settings[\"dual_carriage carriage_t2\"].position_min, printer.configfile.settings[\"carriage carriage_t0\"].position_min]|max %}\n    \
                    {% set x_center = 0.5 * (x_max + x_min) %}\n";
        let error = Template::parse("gcode_macro SET_COPY_MODE:gcode", iqex).expect_err("list");
        assert_eq!(
            error.to_string(),
            "Error loading template 'gcode_macro SET_COPY_MODE:gcode'\n\
             line 3: unexpected token '[' in expression"
        );

        let itex = "    G90\n    \
                    {% set y_center = 0.5 * (printer.configfile.settings[\"dual_carriage carriage_gantry1\"].position_max + printer.configfile.settings[\"carriage carriage_gantry0_left\"].position_min) %}\n    \
                    {% set x_max = printer.configfile.settings[\"dual_carriage carriage_t1\"].position_max %}\n    \
                    {% set x_min = [printer.configfile.settings[\"dual_carriage carriage_t2\"].position_min, printer.configfile.settings[\"carriage carriage_t0\"].position_min]|max %}\n";
        let error = Template::parse("gcode_macro SET_COPY_MODE:gcode", itex).expect_err("list");
        assert_eq!(
            error.to_string(),
            "Error loading template 'gcode_macro SET_COPY_MODE:gcode'\n\
             line 4: unexpected token '[' in expression"
        );
    }
}
