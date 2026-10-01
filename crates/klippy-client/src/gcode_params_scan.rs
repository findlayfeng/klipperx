//! Scan the host's `src/core/klippy` tree for the parameter names each G-Code
//! command declares.
//!
//! The host records the parameter names at the registration point
//! ([`GCodeDispatch::register_command_with_params`] and
//! [`GCodeDispatch::register_mux_command_with_params`]) and reports them in
//! `status.gcode.commands[<name>]["parameters"]`. A client that reaches a host
//! reads them from there; a client that does not still needs the names, so this
//! module re-derives them from the source and [`crate::gcode_params::BUILTIN`]
//! carries the result.
//!
//! It is a source scan, not a Rust parser: it finds the registration calls, then
//! resolves each argument the way the host would — a string literal, a
//! same-file `const`/`static` array, or a `for` loop's array element. Anything
//! it cannot resolve is returned in [`Scan::unresolved`] rather than dropped;
//! `tests/gcode_params_table.rs` asserts that tally so a new registration cannot
//! vanish silently.
//!
//! `#[cfg(test)]` items are skipped: the registrations a test makes are fixtures
//! (`"MY_CMD"`), not commands the host actually answers.
//!
//! [`GCodeDispatch::register_command_with_params`]: crate

use std::collections::BTreeMap;
use std::ffi::OsStr;
use std::fmt;
use std::fs;
use std::ops::Range;
use std::path::{Path, PathBuf};

/// A registration call site the scanner could not turn into parameter names.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unresolved {
    /// Path relative to `src/core/klippy`, e.g. `extras/fan.rs`.
    pub file: String,
    /// 1-based line of the call.
    pub line: usize,
    /// Why the arguments could not be resolved.
    pub reason: String,
}

/// One command and the parameter names it declares, already de-duplicated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Command {
    pub name: String,
    pub params: Vec<String>,
}

/// The result of scanning the host tree.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Scan {
    /// Every `register_*_with_params` call site found in non-test code.
    pub call_sites: usize,
    /// How many of those resolved to at least one command.
    pub resolved_call_sites: usize,
    /// The call sites that did not resolve, one entry each.
    pub unresolved: Vec<Unresolved>,
    /// The merged table: commands sorted by name, parameters in first-seen order.
    pub commands: Vec<Command>,
}

impl Scan {
    /// The commands as the `BUILTIN` table shape, for the freshness test.
    pub fn table(&self) -> Vec<(String, Vec<String>)> {
        self.commands
            .iter()
            .map(|command| (command.name.clone(), command.params.clone()))
            .collect()
    }
}

/// A scan that could not run at all (a file could not be read, or its source is
/// not the Rust this scanner understands).
#[derive(Debug)]
pub struct ScanError {
    message: String,
}

impl ScanError {
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for ScanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ScanError {}

/// The host tree, relative to this crate.
///
/// The generator and the freshness test both run with the crate's
/// `CARGO_MANIFEST_DIR`, so the path is the same in a checkout and in a worktree
/// (where only the main checkout carries `third_party/klipper`, never `src`).
pub fn klippy_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../src/core/klippy")
}

/// Scan the host tree next to this crate.
pub fn scan() -> Result<Scan, ScanError> {
    scan_dir(&klippy_dir())
}

/// Scan `dir` (`src/core/klippy`) recursively.
pub fn scan_dir(dir: &Path) -> Result<Scan, ScanError> {
    let mut files = Vec::new();
    collect_rs_files(dir, &mut files)?;
    files.sort();

    let mut scan = Scan::default();
    let mut entries: Vec<(String, Vec<String>)> = Vec::new();
    for path in files {
        let rel = path
            .strip_prefix(dir)
            .unwrap_or(&path)
            .to_string_lossy()
            .replace('\\', "/");
        let file = scan_file(&path, rel)?;
        scan.call_sites += file.call_sites;
        scan.resolved_call_sites += file.resolved_call_sites;
        scan.unresolved.extend(file.unresolved);
        entries.extend(file.commands);
    }
    scan.commands = merge(entries);
    Ok(scan)
}

/// Merge registrations: same command unions its parameters in first-seen order;
/// the result is sorted by command name.
fn merge(entries: Vec<(String, Vec<String>)>) -> Vec<Command> {
    let mut merged: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (name, params) in entries {
        let slot = merged.entry(name).or_default();
        for param in params {
            if !slot.contains(&param) {
                slot.push(param);
            }
        }
    }
    merged
        .into_iter()
        .map(|(name, params)| Command { name, params })
        .collect()
}

fn collect_rs_files(dir: &Path, out: &mut Vec<PathBuf>) -> Result<(), ScanError> {
    let entries = fs::read_dir(dir)
        .map_err(|error| ScanError::new(format!("cannot read {}: {error}", dir.display())))?;
    for entry in entries {
        let entry = entry
            .map_err(|error| ScanError::new(format!("cannot read {}: {error}", dir.display())))?;
        let path = entry.path();
        let kind = entry
            .file_type()
            .map_err(|error| ScanError::new(format!("cannot stat {}: {error}", path.display())))?;
        if kind.is_dir() {
            collect_rs_files(&path, out)?;
        } else if path.extension() == Some(OsStr::new("rs")) {
            out.push(path);
        }
    }
    Ok(())
}

/// What one file contributed.
#[derive(Debug, Default)]
struct FileScan {
    call_sites: usize,
    resolved_call_sites: usize,
    unresolved: Vec<Unresolved>,
    commands: Vec<(String, Vec<String>)>,
}

fn scan_file(path: &Path, rel: String) -> Result<FileScan, ScanError> {
    let source = fs::read_to_string(path)
        .map_err(|error| ScanError::new(format!("cannot read {}: {error}", path.display())))?;
    let tokens = strip_cfg_test(tokenize(&source)?);
    let loops = find_loops(&tokens);

    let mut scan = FileScan::default();
    let mut i = 0;
    while i < tokens.len() {
        let kind = match &tokens[i].kind {
            Kind::Ident(name) => match name.as_str() {
                "register_command_with_params" => Some(CallKind::Command),
                "register_mux_command_with_params" => Some(CallKind::Mux),
                _ => None,
            },
            _ => None,
        };
        let Some(kind) = kind else {
            i += 1;
            continue;
        };

        // A definition (`fn register_command_with_params`) or a path reference
        // is not a call; the definition carries no literal command name.
        let defined = matches!(
            i.checked_sub(1).and_then(|previous| tokens.get(previous)).map(|token| &token.kind),
            Some(Kind::Ident(previous)) if previous == "fn"
        );
        let path_reference = matches!(
            i.checked_sub(1)
                .and_then(|previous| tokens.get(previous))
                .map(|token| &token.kind),
            Some(Kind::Punct(':'))
        );
        if defined || path_reference {
            i += 1;
            continue;
        }

        if !matches!(
            tokens.get(i + 1).map(|token| &token.kind),
            Some(Kind::Punct('('))
        ) {
            i += 1;
            continue;
        }

        scan.call_sites += 1;
        let line = tokens[i].line;
        let Some(close) = close_bracket(&tokens, i + 1) else {
            scan.unresolved.push(Unresolved {
                file: rel.clone(),
                line,
                reason: "the argument list has no closing `)`".to_string(),
            });
            i += 1;
            continue;
        };
        let args = split_top_level(&tokens[i + 2..close]);
        match resolve_call(&tokens, kind, &args, &loops, i) {
            Ok(commands) => {
                scan.resolved_call_sites += 1;
                scan.commands.extend(commands);
            }
            Err(reason) => scan.unresolved.push(Unresolved {
                file: rel.clone(),
                line,
                reason,
            }),
        }
        i += 1;
    }
    Ok(scan)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CallKind {
    Command,
    Mux,
}

/// Resolve one registration call to `(command, declared parameters)` pairs.
///
/// A loop-driven call yields one pair per array element; everything else yields
/// exactly one.
fn resolve_call(
    tokens: &[Token],
    kind: CallKind,
    args: &[&[Token]],
    loops: &[Loop],
    pos: usize,
) -> Result<Vec<(String, Vec<String>)>, String> {
    let (name_arg, params_arg, key_arg) = match kind {
        CallKind::Command => {
            if args.len() != 5 {
                return Err(format!("expected 5 arguments, found {}", args.len()));
            }
            (args[0], args[3], None)
        }
        CallKind::Mux => {
            if args.len() != 6 {
                return Err(format!("expected 6 arguments, found {}", args.len()));
            }
            (args[0], args[5], Some(args[1]))
        }
    };

    let enclosing = enclosing_loop(loops, pos);
    let name_binding = loop_binding(name_arg, enclosing);
    let params_binding = loop_binding(params_arg, enclosing);
    let key_binding = key_arg.and_then(|arg| loop_binding(arg, enclosing));

    // Only a loop-driven call needs the array rows.
    let rows = if name_binding.or(params_binding).or(key_binding).is_some() {
        let loop_ = enclosing.ok_or("a loop binding without an enclosing loop")?;
        iterable_rows(tokens, loop_)?
    } else {
        Vec::new()
    };

    let names = collect_names(tokens, &rows, name_binding, name_arg)?;
    let paramses = collect_params(tokens, &rows, params_binding, params_arg)?;
    let keys = collect_keys(tokens, &rows, key_binding, key_arg)?;

    let count = names.len().max(paramses.len()).max(keys.len());
    let consistent = |len: usize| len == 1 || len == count;
    if !consistent(names.len()) || !consistent(paramses.len()) || !consistent(keys.len()) {
        return Err("the loop's name, parameters and key have different lengths".to_string());
    }

    let mut out = Vec::with_capacity(count);
    for index in 0..count {
        let params = pick(&paramses, index).clone();
        let declared = match pick(&keys, index) {
            Some(key) => prepend_key(key, params),
            None => params,
        };
        out.push((pick(&names, index).clone(), declared));
    }
    Ok(out)
}

/// The array index a call argument binds to, when it is a `for` loop variable.
fn loop_binding(arg: &[Token], enclosing: Option<&Loop>) -> Option<usize> {
    let [Token {
        kind: Kind::Ident(name),
        ..
    }] = arg
    else {
        return None;
    };
    enclosing?
        .bindings
        .iter()
        .position(|binding| binding == name)
}

fn collect_names(
    tokens: &[Token],
    rows: &[Vec<Vec<Token>>],
    binding: Option<usize>,
    arg: &[Token],
) -> Result<Vec<String>, String> {
    let Some(index) = binding else {
        return Ok(vec![resolve_name_scalar(arg, tokens)?]);
    };
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let field = row
            .get(index)
            .ok_or_else(|| format!("the loop element has no field {index}"))?;
        out.push(resolve_name_scalar(field, tokens)?);
    }
    Ok(out)
}

fn collect_params(
    tokens: &[Token],
    rows: &[Vec<Vec<Token>>],
    binding: Option<usize>,
    arg: &[Token],
) -> Result<Vec<Vec<String>>, String> {
    let Some(index) = binding else {
        return Ok(vec![resolve_params_scalar(arg, tokens)?]);
    };
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let field = row
            .get(index)
            .ok_or_else(|| format!("the loop element has no field {index}"))?;
        out.push(resolve_params_scalar(field, tokens)?);
    }
    Ok(out)
}

fn collect_keys(
    tokens: &[Token],
    rows: &[Vec<Vec<Token>>],
    binding: Option<usize>,
    arg: Option<&[Token]>,
) -> Result<Vec<Option<String>>, String> {
    let Some(arg) = arg else {
        return Ok(vec![None]);
    };
    let Some(index) = binding else {
        return Ok(vec![Some(resolve_name_scalar(arg, tokens)?)]);
    };
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let field = row
            .get(index)
            .ok_or_else(|| format!("the loop element has no field {index}"))?;
        out.push(Some(resolve_name_scalar(field, tokens)?));
    }
    Ok(out)
}

/// Broadcast: a single value stands for every row; otherwise take row `index`.
fn pick<T>(values: &[T], index: usize) -> &T {
    if values.len() == 1 {
        &values[0]
    } else {
        &values[index]
    }
}

/// The mux key leads the declared list and appears once.
fn prepend_key(key: &str, params: Vec<String>) -> Vec<String> {
    let mut declared = Vec::with_capacity(params.len() + 1);
    declared.push(key.to_string());
    for param in params {
        if !declared.contains(&param) {
            declared.push(param);
        }
    }
    declared
}

/// A command name: a string literal, or a same-file `const`/`static` `&str`.
fn resolve_name_scalar(arg: &[Token], file: &[Token]) -> Result<String, String> {
    let arg = strip_reference(arg);
    if let [Token {
        kind: Kind::Str(value),
        ..
    }] = arg
    {
        return Ok(value.clone());
    }
    if let [Token {
        kind: Kind::Ident(name),
        ..
    }] = arg
    {
        return const_string(file, name, 0);
    }
    Err(format!(
        "the command name is not a string literal or a same-file constant: {}",
        render(arg)
    ))
}

fn const_string(file: &[Token], name: &str, depth: usize) -> Result<String, String> {
    if depth > 8 {
        return Err(format!("constant {name} is nested too deeply"));
    }
    let init = find_initializer(file, name, false)
        .ok_or_else(|| format!("no same-file `const {name}` declaration"))?;
    let init = strip_reference(init);
    if let [Token {
        kind: Kind::Str(value),
        ..
    }] = init
    {
        return Ok(value.clone());
    }
    if let [Token {
        kind: Kind::Ident(other),
        ..
    }] = init
    {
        return const_string(file, other, depth + 1);
    }
    Err(format!("constant {name} is not a string literal"))
}

/// A parameter list: an array literal, or a same-file `const`/`static` array.
fn resolve_params_scalar(arg: &[Token], file: &[Token]) -> Result<Vec<String>, String> {
    resolve_params_inner(arg, file, 0)
}

fn resolve_params_inner(
    arg: &[Token],
    file: &[Token],
    depth: usize,
) -> Result<Vec<String>, String> {
    if depth > 8 {
        return Err("the parameter list is nested too deeply".to_string());
    }
    let arg = strip_reference(arg);
    let Some(first) = arg.first() else {
        return Err("the parameter list is empty".to_string());
    };
    if matches!(first.kind, Kind::Punct('[')) {
        let close = close_bracket(arg, 0)
            .ok_or_else(|| format!("the parameter list has no closing `]`: {}", render(arg)))?;
        if close != arg.len() - 1 {
            return Err(format!(
                "unexpected tokens after the parameter list: {}",
                render(arg)
            ));
        }
        let mut out = Vec::new();
        for part in split_top_level(&arg[1..close]) {
            match part {
                [Token {
                    kind: Kind::Str(value),
                    ..
                }] => {
                    if !out.contains(value) {
                        out.push(value.clone());
                    }
                }
                _ => {
                    return Err(format!(
                        "the parameter list has a non-string element: {}",
                        render(part)
                    ))
                }
            }
        }
        return Ok(out);
    }
    if let [Token {
        kind: Kind::Ident(name),
        ..
    }] = arg
    {
        let init = find_initializer(file, name, false).ok_or_else(|| {
            format!("no same-file `const {name}` declaration for the parameter list")
        })?;
        return resolve_params_inner(init, file, depth + 1);
    }
    Err(format!(
        "the parameter list is neither an array literal nor a same-file constant: {}",
        render(arg)
    ))
}

/// The tokens an `const`/`static` (and, when `include_let`, `let`) binding of
/// `name` has after its `=`.
///
/// `include_let` is for a loop iterable, which a `let` often names (a local
/// array); a command name or parameter list must be a `const`/`static`, because
/// a `let` elsewhere in the file is a different binding in a different scope.
fn find_initializer<'a>(tokens: &'a [Token], name: &str, include_let: bool) -> Option<&'a [Token]> {
    for (i, token) in tokens.iter().enumerate() {
        let Kind::Ident(keyword) = &token.kind else {
            continue;
        };
        let matches_keyword =
            matches!(keyword.as_str(), "const" | "static") || (include_let && keyword == "let");
        if !matches_keyword {
            continue;
        }
        let mut j = i + 1;
        if matches!(tokens.get(j).map(|token| &token.kind), Some(Kind::Ident(word)) if word == "mut")
        {
            j += 1;
        }
        match tokens.get(j).map(|token| &token.kind) {
            Some(Kind::Ident(bound)) if bound == name => {}
            _ => continue,
        }

        // The `=` that binds it, at bracket depth 0 (a `;` inside a type such as
        // `[(&str, &str); 4]` is not the statement's end).
        let mut depth = 0i32;
        let mut equals = None;
        let mut k = j + 1;
        while k < tokens.len() {
            match tokens[k].kind {
                Kind::Punct('(') | Kind::Punct('[') | Kind::Punct('{') => depth += 1,
                Kind::Punct(')') | Kind::Punct(']') | Kind::Punct('}') => depth -= 1,
                Kind::Punct('=') if depth == 0 => {
                    equals = Some(k);
                    break;
                }
                Kind::Punct(';') if depth == 0 => break,
                _ => {}
            }
            k += 1;
        }
        let Some(equals) = equals else {
            continue;
        };

        let mut depth = 0i32;
        let mut end = tokens.len();
        let mut m = equals + 1;
        while m < tokens.len() {
            match tokens[m].kind {
                Kind::Punct('(') | Kind::Punct('[') | Kind::Punct('{') => depth += 1,
                Kind::Punct(')') | Kind::Punct(']') | Kind::Punct('}') => depth -= 1,
                Kind::Punct(';') if depth == 0 => {
                    end = m;
                    break;
                }
                _ => {}
            }
            m += 1;
        }
        return Some(&tokens[equals + 1..end]);
    }
    None
}

/// Expand a `for` loop's array into rows of fields. Each row is the tuple's
/// elements; a non-tuple element is a one-field row.
fn iterable_rows(tokens: &[Token], loop_: &Loop) -> Result<Vec<Vec<Vec<Token>>>, String> {
    let iterable = &tokens[loop_.iterable.clone()];
    let array = strip_reference(resolve_array(tokens, iterable)?);
    let Some(first) = array.first() else {
        return Err("the loop iterates over an empty array".to_string());
    };
    if !matches!(first.kind, Kind::Punct('[')) {
        return Err(format!(
            "the loop iterates over a non-array expression: {}",
            render(array)
        ));
    }
    let close = close_bracket(array, 0)
        .ok_or_else(|| format!("the loop array has no closing `]`: {}", render(array)))?;
    if close != array.len() - 1 {
        return Err(format!(
            "unexpected tokens after the loop array: {}",
            render(array)
        ));
    }

    let mut rows = Vec::new();
    for element in split_top_level(&array[1..close]) {
        let row = if matches!(
            element.first().map(|token| &token.kind),
            Some(Kind::Punct('('))
        ) {
            let Some(inner_close) = close_bracket(element, 0) else {
                return Err(format!(
                    "a loop element has no closing `)`: {}",
                    render(element)
                ));
            };
            if inner_close != element.len() - 1 {
                return Err(format!(
                    "unexpected tokens after a loop element: {}",
                    render(element)
                ));
            }
            split_top_level(&element[1..inner_close])
                .into_iter()
                .map(<[Token]>::to_vec)
                .collect()
        } else {
            vec![element.to_vec()]
        };
        rows.push(row);
    }
    Ok(rows)
}

/// Follow a loop's iterable back to its array: an inline `[...]`, or a
/// same-file `const`/`static`/`let` binding.
fn resolve_array<'a>(tokens: &'a [Token], iterable: &'a [Token]) -> Result<&'a [Token], String> {
    let iterable = strip_reference(iterable);
    match iterable {
        [Token {
            kind: Kind::Ident(name),
            ..
        }] => find_initializer(tokens, name, true)
            .ok_or_else(|| format!("no same-file declaration for the loop array `{name}`")),
        _ => Ok(iterable),
    }
}

/// A `for PATTERN in ITERABLE { BODY }` loop and what its pattern binds.
#[derive(Debug)]
struct Loop {
    bindings: Vec<String>,
    iterable: Range<usize>,
    body: Range<usize>,
}

fn find_loops(tokens: &[Token]) -> Vec<Loop> {
    let mut loops = Vec::new();
    for (i, token) in tokens.iter().enumerate() {
        if !matches!(&token.kind, Kind::Ident(keyword) if keyword == "for") {
            continue;
        }
        let Some(in_index) = find_keyword(tokens, i + 1, "in") else {
            continue;
        };
        let Some(brace) = find_body_brace(tokens, in_index + 1) else {
            continue;
        };
        let Some(close) = close_bracket(tokens, brace) else {
            continue;
        };
        loops.push(Loop {
            bindings: pattern_bindings(&tokens[i + 1..in_index]),
            iterable: (in_index + 1)..brace,
            body: brace..(close + 1),
        });
    }
    loops
}

/// The `in` keyword at bracket depth 0, ending a `for` pattern.
fn find_keyword(tokens: &[Token], from: usize, keyword: &str) -> Option<usize> {
    let mut depth = 0i32;
    let mut i = from;
    while i < tokens.len() {
        match &tokens[i].kind {
            Kind::Punct('(') | Kind::Punct('[') => depth += 1,
            Kind::Punct(')') | Kind::Punct(']') => depth -= 1,
            Kind::Ident(word) if word == keyword && depth == 0 => return Some(i),
            _ => {}
        }
        i += 1;
    }
    None
}

/// The `{` that opens a `for` loop's body, at bracket depth 0.
fn find_body_brace(tokens: &[Token], from: usize) -> Option<usize> {
    let mut depth = 0i32;
    let mut i = from;
    while i < tokens.len() {
        match tokens[i].kind {
            Kind::Punct('(') | Kind::Punct('[') => depth += 1,
            Kind::Punct(')') | Kind::Punct(']') => depth -= 1,
            Kind::Punct('{') if depth == 0 => return Some(i),
            _ => {}
        }
        i += 1;
    }
    None
}

fn pattern_bindings(pattern: &[Token]) -> Vec<String> {
    pattern
        .iter()
        .filter_map(|token| match &token.kind {
            Kind::Ident(name) if name != "mut" && name != "ref" => Some(name.clone()),
            _ => None,
        })
        .collect()
}

/// The innermost `for` loop whose body contains `pos`.
fn enclosing_loop(loops: &[Loop], pos: usize) -> Option<&Loop> {
    loops
        .iter()
        .filter(|loop_| loop_.body.contains(&pos))
        .max_by_key(|loop_| loop_.body.start)
}

/// Index of the bracket matching the one at `open`.
fn close_bracket(tokens: &[Token], open: usize) -> Option<usize> {
    let (open_char, close_char) = match tokens.get(open).map(|token| &token.kind) {
        Some(Kind::Punct('(')) => ('(', ')'),
        Some(Kind::Punct('[')) => ('[', ']'),
        Some(Kind::Punct('{')) => ('{', '}'),
        _ => return None,
    };
    let mut depth = 0i32;
    for (i, token) in tokens.iter().enumerate().skip(open) {
        if let Kind::Punct(c) = token.kind {
            if c == open_char {
                depth += 1;
            } else if c == close_char {
                depth -= 1;
                if depth == 0 {
                    return Some(i);
                }
            }
        }
    }
    None
}

/// Split on commas that sit outside any bracket, dropping empty parts.
fn split_top_level(tokens: &[Token]) -> Vec<&[Token]> {
    let mut parts = Vec::new();
    let mut depth = 0i32;
    let mut start = 0;
    for (i, token) in tokens.iter().enumerate() {
        match token.kind {
            Kind::Punct('(') | Kind::Punct('[') | Kind::Punct('{') => depth += 1,
            Kind::Punct(')') | Kind::Punct(']') | Kind::Punct('}') => depth -= 1,
            Kind::Punct(',') if depth == 0 => {
                if start < i {
                    parts.push(&tokens[start..i]);
                }
                start = i + 1;
            }
            _ => {}
        }
    }
    if start < tokens.len() {
        parts.push(&tokens[start..]);
    }
    parts
}

fn strip_reference(tokens: &[Token]) -> &[Token] {
    match tokens.first().map(|token| &token.kind) {
        Some(Kind::Punct('&')) => &tokens[1..],
        _ => tokens,
    }
}

/// Render tokens for an error message.
fn render(tokens: &[Token]) -> String {
    let mut text = String::new();
    for (i, token) in tokens.iter().enumerate() {
        if i > 0 {
            text.push(' ');
        }
        match &token.kind {
            Kind::Ident(name) => text.push_str(name),
            Kind::Str(value) => {
                text.push('"');
                text.push_str(value);
                text.push('"');
            }
            Kind::Punct(c) => text.push(*c),
            Kind::Other => text.push('_'),
        }
        if text.len() > 80 {
            text.push_str("...");
            break;
        }
    }
    text
}

// ===========================================================================
// Tokens
// ===========================================================================

#[derive(Debug, Clone, PartialEq, Eq)]
enum Kind {
    Ident(String),
    Str(String),
    Punct(char),
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Token {
    kind: Kind,
    line: usize,
}

/// Remove every `#[cfg(test)]` item, so a fixture registration is not mistaken
/// for a command the host answers.
fn strip_cfg_test(tokens: Vec<Token>) -> Vec<Token> {
    let mut out = Vec::with_capacity(tokens.len());
    let mut i = 0;
    while i < tokens.len() {
        if !is_cfg_test(&tokens, i) {
            out.push(tokens[i].clone());
            i += 1;
            continue;
        }
        // Skip the attribute, any further attributes, then the item itself
        // (a braced body, or a `;`-terminated item such as a `use`).
        let mut j = i + 7;
        while matches!(
            tokens.get(j).map(|token| &token.kind),
            Some(Kind::Punct('#'))
        ) && matches!(
            tokens.get(j + 1).map(|token| &token.kind),
            Some(Kind::Punct('['))
        ) {
            match close_bracket(&tokens, j + 1) {
                Some(close) => j = close + 1,
                None => break,
            }
        }
        let mut depth = 0i32;
        let mut k = j;
        let mut skip_to = None;
        while k < tokens.len() {
            match tokens[k].kind {
                Kind::Punct('(') | Kind::Punct('[') => depth += 1,
                Kind::Punct(')') | Kind::Punct(']') => depth -= 1,
                Kind::Punct('{') if depth == 0 => {
                    skip_to = close_bracket(&tokens, k).map(|close| close + 1);
                    break;
                }
                Kind::Punct(';') if depth == 0 => {
                    skip_to = Some(k + 1);
                    break;
                }
                _ => {}
            }
            k += 1;
        }
        i = skip_to.unwrap_or(tokens.len());
    }
    out
}

fn is_cfg_test(tokens: &[Token], i: usize) -> bool {
    let punct = |offset: usize, expected: char| {
        matches!(
            tokens.get(i + offset).map(|token| &token.kind),
            Some(Kind::Punct(c)) if *c == expected
        )
    };
    let ident = |offset: usize, expected: &str| {
        matches!(
            tokens.get(i + offset).map(|token| &token.kind),
            Some(Kind::Ident(word)) if word == expected
        )
    };
    punct(0, '#')
        && punct(1, '[')
        && ident(2, "cfg")
        && punct(3, '(')
        && ident(4, "test")
        && punct(5, ')')
        && punct(6, ']')
}

/// Tokenize enough Rust to find the registration calls and their arguments.
fn tokenize(source: &str) -> Result<Vec<Token>, ScanError> {
    let chars: Vec<char> = source.chars().collect();
    let mut tokens = Vec::new();
    let mut line = 1usize;
    let mut i = 0usize;
    while i < chars.len() {
        let c = chars[i];
        match c {
            '\n' => {
                line += 1;
                i += 1;
            }
            _ if c.is_whitespace() => i += 1,
            '/' if chars.get(i + 1) == Some(&'/') => {
                i += 2;
                while i < chars.len() && chars[i] != '\n' {
                    i += 1;
                }
            }
            '/' if chars.get(i + 1) == Some(&'*') => {
                i += 2;
                let mut depth = 1;
                while i < chars.len() && depth > 0 {
                    if chars[i] == '\n' {
                        line += 1;
                        i += 1;
                    } else if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                        depth += 1;
                        i += 2;
                    } else if chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
                        depth -= 1;
                        i += 2;
                    } else {
                        i += 1;
                    }
                }
            }
            '"' => {
                let (value, next) = read_string(&chars, i, &mut line);
                push_token(&mut tokens, Kind::Str(value), line);
                i = next;
            }
            'r' | 'b' | 'c' => {
                if let Some((value, next)) = read_prefixed_string(&chars, i, &mut line) {
                    push_token(&mut tokens, Kind::Str(value), line);
                    i = next;
                } else {
                    let start = i;
                    while i < chars.len() && is_ident_continue(chars[i]) {
                        i += 1;
                    }
                    push_token(&mut tokens, ident(&chars, start, i), line);
                }
            }
            '\'' => {
                if let Some(next) = read_char_literal(&chars, i, &mut line) {
                    push_token(&mut tokens, Kind::Other, line);
                    i = next;
                } else {
                    // A lifetime: `'` then the name; the `'` is not significant.
                    i += 1;
                    while i < chars.len() && is_ident_continue(chars[i]) {
                        i += 1;
                    }
                    push_token(&mut tokens, Kind::Other, line);
                }
            }
            _ if is_ident_start(c) => {
                let start = i;
                while i < chars.len() && is_ident_continue(chars[i]) {
                    i += 1;
                }
                push_token(&mut tokens, ident(&chars, start, i), line);
            }
            _ if c.is_ascii_digit() => {
                while i < chars.len()
                    && (chars[i].is_alphanumeric() || chars[i] == '_' || chars[i] == '.')
                {
                    i += 1;
                }
                push_token(&mut tokens, Kind::Other, line);
            }
            _ => {
                push_token(&mut tokens, Kind::Punct(c), line);
                i += 1;
            }
        }
    }
    Ok(tokens)
}

fn push_token(tokens: &mut Vec<Token>, kind: Kind, line: usize) {
    tokens.push(Token { kind, line });
}

fn ident(chars: &[char], start: usize, end: usize) -> Kind {
    if start == end {
        Kind::Other
    } else {
        Kind::Ident(chars[start..end].iter().collect())
    }
}

fn is_ident_start(c: char) -> bool {
    c.is_alphabetic() || c == '_'
}

fn is_ident_continue(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

/// Read a `"..."` literal starting at `start`, decoding escapes.
fn read_string(chars: &[char], start: usize, line: &mut usize) -> (String, usize) {
    let mut i = start + 1;
    let mut value = String::new();
    while i < chars.len() {
        match chars[i] {
            '"' => {
                i += 1;
                break;
            }
            '\\' => {
                let (decoded, next) = read_escape(chars, i, line);
                value.push_str(&decoded);
                i = next;
            }
            '\n' => {
                *line += 1;
                value.push('\n');
                i += 1;
            }
            c => {
                value.push(c);
                i += 1;
            }
        }
    }
    (value, i)
}

/// Read a raw string (`r"..."`, `r#"..."#`) or byte string (`b"..."`) at
/// `start`, when one begins there.
fn read_prefixed_string(chars: &[char], start: usize, line: &mut usize) -> Option<(String, usize)> {
    let mut j = start;
    if chars.get(j) == Some(&'b') || chars.get(j) == Some(&'c') {
        if chars.get(j + 1) == Some(&'r') {
            j += 1;
        } else if chars.get(j + 1) == Some(&'"') {
            return Some(read_string(chars, j + 1, line));
        } else if chars.get(j + 1) == Some(&'\'') {
            return None; // a byte char literal, handled by the caller
        } else {
            return None;
        }
    }
    if chars.get(j) != Some(&'r') {
        return None;
    }
    j += 1;
    let mut hashes = 0;
    while chars.get(j) == Some(&'#') {
        hashes += 1;
        j += 1;
    }
    if chars.get(j) != Some(&'"') {
        return None;
    }
    j += 1;

    let mut value = String::new();
    while j < chars.len() {
        if chars[j] == '"' {
            let mut k = j + 1;
            let mut closed = 0;
            while closed < hashes && chars.get(k) == Some(&'#') {
                closed += 1;
                k += 1;
            }
            if closed == hashes {
                return Some((value, k));
            }
        }
        if chars[j] == '\n' {
            *line += 1;
        }
        value.push(chars[j]);
        j += 1;
    }
    Some((value, j))
}

/// Read the char literal at `start` (`'x'`, `'\n'`, `'\''`), or `None` when it
/// is a lifetime (`'static`).
fn read_char_literal(chars: &[char], start: usize, line: &mut usize) -> Option<usize> {
    if chars.get(start + 1) == Some(&'\\') {
        let (_, next) = read_escape(chars, start + 1, line);
        if chars.get(next) == Some(&'\'') {
            return Some(next + 1);
        }
        return None;
    }
    if chars.get(start + 2) == Some(&'\'') {
        return Some(start + 3);
    }
    None
}

/// Decode one escape sequence; `i` is the `\`.
fn read_escape(chars: &[char], i: usize, line: &mut usize) -> (String, usize) {
    let Some(&c) = chars.get(i + 1) else {
        return (String::new(), i + 1);
    };
    match c {
        'n' => ("\n".to_string(), i + 2),
        't' => ("\t".to_string(), i + 2),
        'r' => ("\r".to_string(), i + 2),
        '0' => ("\0".to_string(), i + 2),
        '\\' => ("\\".to_string(), i + 2),
        '"' => ("\"".to_string(), i + 2),
        '\'' => ("'".to_string(), i + 2),
        '\n' => {
            *line += 1;
            ("\n".to_string(), i + 2)
        }
        // `\xNN`: two hex digits.
        'x' => {
            let mut value = String::new();
            let mut j = i + 2;
            let mut code = 0u32;
            let mut digits = 0;
            while digits < 2 {
                match chars.get(j).and_then(|c| c.to_digit(16)) {
                    Some(digit) => code = code * 16 + digit,
                    None => break,
                }
                digits += 1;
                j += 1;
            }
            if let Some(c) = char::from_u32(code) {
                value.push(c);
            }
            (value, j)
        }
        // `\u{...}`.
        'u' => {
            let mut value = String::new();
            let mut j = i + 2;
            if chars.get(j) == Some(&'{') {
                j += 1;
                let mut code = 0u32;
                while let Some(digit) = chars.get(j).and_then(|c| c.to_digit(16)) {
                    code = code * 16 + digit;
                    j += 1;
                }
                if chars.get(j) == Some(&'}') {
                    j += 1;
                }
                if let Some(c) = char::from_u32(code) {
                    value.push(c);
                }
            }
            (value, j)
        }
        other => (other.to_string(), i + 2),
    }
}
