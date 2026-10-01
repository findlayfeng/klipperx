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
//! `const`/`static` array, a `for` loop's array element, a zero-argument helper
//! that builds the list (`probe_points_params()`), or a constant or helper a
//! `use` names in a sibling module, followed through `crate::`/`super::` to the
//! file that defines it. A registration inside a closure that takes the command
//! name and its parameters (`extras/tmc.rs` registers four commands that way) is
//! resolved at the closure's call sites, where the literals are. Anything it
//! cannot resolve — a name only the caller knows — is returned in
//! [`Scan::unresolved`] rather than dropped; `tests/gcode_params_table.rs`
//! asserts that tally so a new registration cannot vanish silently.
//!
//! `#[cfg(test)]` items are skipped: the registrations a test makes are fixtures
//! (`"MY_CMD"`), not commands the host actually answers.
//!
//! [`GCodeDispatch::register_command_with_params`]: crate

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::ffi::OsStr;
use std::fmt;
use std::fs;
use std::ops::Range;
use std::path::{Path, PathBuf};
use std::rc::Rc;

/// How many `const`/`use`/helper hops one argument may take before the scanner
/// gives up. It bounds a definition cycle as well as a chain that is merely
/// deeper than this scanner understands.
const MAX_DEPTH: usize = 8;

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

    let resolver = Resolver::new(dir.to_path_buf());
    let mut scan = Scan::default();
    let mut entries: Vec<(String, Vec<String>)> = Vec::new();
    for path in files {
        let rel = path
            .strip_prefix(dir)
            .unwrap_or(&path)
            .to_string_lossy()
            .replace('\\', "/");
        let source = resolver.file(&rel)?;
        let file = scan_file(source, &resolver)?;
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

/// The scanned tree, with each file read and tokenized on demand: following a
/// name a `use` imports means reading a file the scan has not reached yet, and
/// only the files an argument actually names need to be in memory.
struct Resolver {
    /// The tree root, `src/core/klippy`, which a module path is relative to.
    root: PathBuf,
    files: RefCell<BTreeMap<String, Rc<SourceFile>>>,
}

/// One file: its tokens, and the names its `use` statements import.
struct SourceFile {
    /// Path relative to the root, e.g. `extras/probe.rs`.
    rel: String,
    tokens: Vec<Token>,
    /// Local name -> (the file that defines it, the name there).
    imports: BTreeMap<String, (String, String)>,
}

/// Where one registration argument is read from: the file it sits in, plus the
/// tree the other files can be read from.
#[derive(Clone)]
struct Ctx<'a> {
    resolver: &'a Resolver,
    file: Rc<SourceFile>,
}

impl Resolver {
    fn new(root: PathBuf) -> Self {
        Self {
            root,
            files: RefCell::new(BTreeMap::new()),
        }
    }

    /// The file at `rel`, tokenized and cached.
    fn file(&self, rel: &str) -> Result<Rc<SourceFile>, ScanError> {
        if let Some(file) = self.files.borrow().get(rel) {
            return Ok(Rc::clone(file));
        }
        let path = self.root.join(rel);
        let source = fs::read_to_string(&path)
            .map_err(|error| ScanError::new(format!("cannot read {}: {error}", path.display())))?;
        let tokens = strip_cfg_test(tokenize(&source)?);
        let imports = self.imports_of(&tokens, rel);
        let file = Rc::new(SourceFile {
            rel: rel.to_string(),
            tokens,
            imports,
        });
        self.files
            .borrow_mut()
            .insert(rel.to_string(), Rc::clone(&file));
        Ok(file)
    }

    /// What the `use` statements of the file at `rel` import: the local name,
    /// the file it is defined in, and the name it has there.
    ///
    /// A glob (`use a::*`) and a path outside the tree are not recorded: the
    /// scanner follows a name to its definition, it does not search a module
    /// for one.
    fn imports_of(&self, tokens: &[Token], rel: &str) -> BTreeMap<String, (String, String)> {
        let mut imports = BTreeMap::new();
        let mut i = 0;
        while i < tokens.len() {
            let is_use = matches!(&tokens[i].kind, Kind::Ident(word) if word == "use");
            if !is_use {
                i += 1;
                continue;
            }
            let Some(end) = statement_end(tokens, i + 1) else {
                break;
            };
            for (local, module, name) in use_paths(&tokens[i + 1..end]) {
                if let Some(file) = self.module_file(rel, &module) {
                    imports.insert(local, (file, name));
                }
            }
            i = end + 1;
        }
        imports
    }

    /// The file a module path names, as read from the file at `from`.
    ///
    /// `crate::core::klippy::…` is relative to the tree root, `self::…` to the
    /// module `from` declares and `super::…` to its parent, exactly as the
    /// source reads. A path outside the tree (an external crate) and one with
    /// no file behind it are not followed.
    fn module_file(&self, from: &str, path: &[String]) -> Option<String> {
        let dir = parent_dir(from);
        let stem = file_stem(from);
        // The directory this file's own module keeps its children in: beside a
        // `mod.rs`, under `<stem>/` otherwise.
        let own = if stem == "mod" {
            dir.to_string()
        } else if dir.is_empty() {
            stem.to_string()
        } else {
            format!("{dir}/{stem}")
        };
        let mut segments: Vec<&str> = path.iter().map(String::as_str).collect();
        let mut dir = dir.to_string();
        match segments.first().copied()? {
            "crate" => {
                if segments.get(1) != Some(&"core") || segments.get(2) != Some(&"klippy") {
                    return None;
                }
                // `crate::core::klippy` is the tree root itself, so the rest of
                // the path is relative to the root, not to this file's directory.
                dir.clear();
                segments.drain(..3);
            }
            "self" => {
                segments.remove(0);
                dir = own;
            }
            "super" => {
                // The first `super` is this file's own directory; each further
                // one steps out of a module directory.
                segments.remove(0);
                while segments.first() == Some(&"super") {
                    segments.remove(0);
                    dir = parent_dir(&dir).to_string();
                }
            }
            _ => return None,
        }
        if segments.is_empty() {
            return None;
        }
        let rel = if dir.is_empty() {
            segments.join("/")
        } else {
            format!("{dir}/{}", segments.join("/"))
        };
        [format!("{rel}.rs"), format!("{rel}/mod.rs")]
            .into_iter()
            .find(|candidate| self.root.join(candidate).is_file())
    }
}

impl<'a> Ctx<'a> {
    fn tokens(&self) -> &[Token] {
        &self.file.tokens
    }

    /// The same tree, read as `rel`.
    fn in_file(&self, rel: &str) -> Result<Self, String> {
        Ok(Self {
            resolver: self.resolver,
            file: self.resolver.file(rel).map_err(|error| error.to_string())?,
        })
    }
}

/// One `use` item's imports: the local name, the module path it comes from and
/// the name it has there.
///
/// `a::b::c` imports `c` from `a::b`; `a::b::{c, d as e}` imports both. A glob
/// and a nested tree are not followed.
fn use_paths(tokens: &[Token]) -> Vec<(String, Vec<String>, String)> {
    let mut out = Vec::new();
    let mut i = skip_visibility(tokens);
    let mut segments: Vec<String> = Vec::new();
    loop {
        let Some(Kind::Ident(segment)) = tokens.get(i).map(|token| &token.kind) else {
            return out;
        };
        segments.push(segment.clone());
        i += 1;
        match tokens.get(i).map(|token| &token.kind) {
            Some(Kind::Punct(':'))
                if matches!(
                    tokens.get(i + 1).map(|token| &token.kind),
                    Some(Kind::Punct(':'))
                ) =>
            {
                i += 2;
                if matches!(
                    tokens.get(i).map(|token| &token.kind),
                    Some(Kind::Punct('{'))
                ) {
                    let Some(close) = close_bracket(tokens, i) else {
                        return out;
                    };
                    for part in split_top_level(&tokens[i + 1..close]) {
                        match part {
                            [Token {
                                kind: Kind::Ident(name),
                                ..
                            }] if name != "self" => {
                                out.push((name.clone(), segments.clone(), name.clone()));
                            }
                            [Token {
                                kind: Kind::Ident(name),
                                ..
                            }, Token {
                                kind: Kind::Ident(word),
                                ..
                            }, Token {
                                kind: Kind::Ident(alias),
                                ..
                            }] if word == "as" => {
                                out.push((alias.clone(), segments.clone(), name.clone()));
                            }
                            _ => {}
                        }
                    }
                    return out;
                }
                if matches!(
                    tokens.get(i).map(|token| &token.kind),
                    Some(Kind::Punct('*'))
                ) {
                    return out;
                }
            }
            Some(Kind::Ident(word)) if word == "as" => {
                let Some(Kind::Ident(alias)) = tokens.get(i + 1).map(|token| &token.kind) else {
                    return out;
                };
                let Some(name) = segments.pop() else {
                    return out;
                };
                out.push((alias.clone(), segments, name));
                return out;
            }
            _ => {
                let Some(name) = segments.pop() else {
                    return out;
                };
                out.push((name.clone(), segments, name));
                return out;
            }
        }
    }
}

/// The index after a leading `pub` / `pub(crate)` on a `use` item.
fn skip_visibility(tokens: &[Token]) -> usize {
    let mut i = 0;
    if matches!(tokens.first().map(|token| &token.kind), Some(Kind::Ident(word)) if word == "pub") {
        i = 1;
        if matches!(
            tokens.get(i).map(|token| &token.kind),
            Some(Kind::Punct('('))
        ) {
            if let Some(close) = close_bracket(tokens, i) {
                i = close + 1;
            }
        }
    }
    i
}

/// What one file contributed.
#[derive(Debug, Default)]
struct FileScan {
    call_sites: usize,
    resolved_call_sites: usize,
    unresolved: Vec<Unresolved>,
    commands: Vec<(String, Vec<String>)>,
}

fn scan_file(source: Rc<SourceFile>, resolver: &Resolver) -> Result<FileScan, ScanError> {
    let ctx = Ctx {
        resolver,
        file: source,
    };
    let tokens = ctx.tokens();
    let rel = ctx.file.rel.clone();
    let loops = find_loops(tokens);

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
        let Some(close) = close_bracket(tokens, i + 1) else {
            scan.unresolved.push(Unresolved {
                file: rel.clone(),
                line,
                reason: "the argument list has no closing `)`".to_string(),
            });
            i += 1;
            continue;
        };
        let args = split_top_level(&tokens[i + 2..close]);
        match resolve_call(&ctx, kind, &args, &loops, i, true) {
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

/// One call's arguments, each argument as its own tokens.
struct Arguments(Vec<Vec<Token>>);

/// Resolve one registration call to `(command, declared parameters)` pairs.
///
/// A loop-driven call yields one pair per array element, a closure-driven one a
/// pair per call of the closure; everything else yields exactly one.
fn resolve_call(
    ctx: &Ctx,
    kind: CallKind,
    args: &[&[Token]],
    loops: &[Loop],
    pos: usize,
    expand_closure: bool,
) -> Result<Vec<(String, Vec<String>)>, String> {
    let attempt = resolve_arguments(ctx, kind, args, loops, pos);
    if attempt.is_ok() || !expand_closure {
        return attempt;
    }
    // A registration inside a `let register = |params| { … }` closure takes its
    // command name from that closure's parameters, which only the calls of
    // `register` supply (`extras/tmc.rs` registers four commands through one
    // closure). Re-resolve each call with its arguments substituted.
    let Some(calls) = closure_call_arguments(ctx, kind, args, pos)? else {
        return attempt;
    };
    let mut out = Vec::new();
    for call in calls {
        let refs: Vec<&[Token]> = call.0.iter().map(Vec::as_slice).collect();
        out.extend(resolve_call(ctx, kind, &refs, loops, pos, false)?);
    }
    Ok(out)
}

/// Resolve one registration call against its own arguments: the arguments of a
/// loop-driven call index the loop's array rows.
fn resolve_arguments(
    ctx: &Ctx,
    kind: CallKind,
    args: &[&[Token]],
    loops: &[Loop],
    pos: usize,
) -> Result<Vec<(String, Vec<String>)>, String> {
    let Declared {
        name: name_arg,
        params: params_arg,
        key: key_arg,
    } = split_arguments(kind, args)?;

    let enclosing = enclosing_loop(loops, pos);
    let name_binding = loop_binding(name_arg, enclosing);
    let params_binding = loop_binding(params_arg, enclosing);
    let key_binding = key_arg.and_then(|arg| loop_binding(arg, enclosing));

    // Only a loop-driven call needs the array rows.
    let rows = if name_binding.or(params_binding).or(key_binding).is_some() {
        let loop_ = enclosing.ok_or("a loop binding without an enclosing loop")?;
        iterable_rows(ctx.tokens(), loop_)?
    } else {
        Vec::new()
    };

    let names = collect_names(ctx, &rows, name_binding, name_arg)?;
    let paramses = collect_params(ctx, &rows, params_binding, params_arg)?;
    let keys = collect_keys(ctx, &rows, key_binding, key_arg)?;

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

/// The three arguments of a registration that carry a declaration.
struct Declared<'a> {
    /// The command name.
    name: &'a [Token],
    /// The parameter list.
    params: &'a [Token],
    /// The key, for a mux command.
    key: Option<&'a [Token]>,
}

/// The three arguments that carry a declaration: the command name, its
/// parameter list, and a mux command's key.
fn split_arguments<'a>(kind: CallKind, args: &[&'a [Token]]) -> Result<Declared<'a>, String> {
    match kind {
        CallKind::Command => {
            if args.len() != 5 {
                return Err(format!("expected 5 arguments, found {}", args.len()));
            }
            Ok(Declared {
                name: args[0],
                params: args[3],
                key: None,
            })
        }
        CallKind::Mux => {
            if args.len() != 6 {
                return Err(format!("expected 6 arguments, found {}", args.len()));
            }
            Ok(Declared {
                name: args[0],
                params: args[5],
                key: Some(args[1]),
            })
        }
    }
}

/// The re-resolved arguments for a registration whose command name or parameter
/// list is an enclosing closure's parameter: one entry per call of that closure,
/// with the call's arguments substituted for the parameters.
///
/// `None` when no such argument is a closure parameter, or when the closure is
/// not bound to a name this file calls.
fn closure_call_arguments(
    ctx: &Ctx,
    kind: CallKind,
    args: &[&[Token]],
    pos: usize,
) -> Result<Option<Vec<Arguments>>, String> {
    let Declared {
        name: name_arg,
        params: params_arg,
        key: key_arg,
    } = split_arguments(kind, args)?;
    let Some(closure) = enclosing_closure(ctx.tokens(), pos) else {
        return Ok(None);
    };
    let is_parameter = |arg: &[Token]| matches!(arg, [Token { kind: Kind::Ident(name), .. }] if closure.params.contains(name));
    if ![Some(name_arg), key_arg, Some(params_arg)]
        .into_iter()
        .flatten()
        .any(is_parameter)
    {
        return Ok(None);
    }
    let mut out = Vec::new();
    for call in closure_calls(ctx.tokens(), &closure.binding) {
        if call.0.len() != closure.params.len() {
            continue;
        }
        let mut substituted = Arguments(args.iter().map(|arg| arg.to_vec()).collect());
        for (index, arg) in args.iter().enumerate() {
            let [Token {
                kind: Kind::Ident(name),
                ..
            }] = *arg
            else {
                continue;
            };
            if let Some(position) = closure.params.iter().position(|param| param == name) {
                substituted.0[index] = call.0[position].clone();
            }
        }
        out.push(substituted);
    }
    if out.is_empty() {
        return Ok(None);
    }
    Ok(Some(out))
}

/// A closure a `let` binds, with the names its parameter pattern binds.
struct Closure {
    /// The name the `let` binds the closure to.
    binding: String,
    /// The parameter names, in order.
    params: Vec<String>,
    /// The body, so the innermost closure around a call is the one found.
    body: Range<usize>,
}

/// The innermost `let NAME = |params| { body }` whose body contains `pos`.
///
/// Only a closure bound to a name is followed: the calls of that name are where
/// a registration inside it gets its command name and parameters.
fn enclosing_closure(tokens: &[Token], pos: usize) -> Option<Closure> {
    let mut found: Option<Closure> = None;
    for (i, token) in tokens.iter().enumerate() {
        if !matches!(&token.kind, Kind::Ident(word) if word == "let") {
            continue;
        }
        let mut j = i + 1;
        if matches!(tokens.get(j).map(|token| &token.kind), Some(Kind::Ident(word)) if word == "mut")
        {
            j += 1;
        }
        let Some(Kind::Ident(binding)) = tokens.get(j).map(|token| &token.kind) else {
            continue;
        };
        j += 1;
        if !matches!(
            tokens.get(j).map(|token| &token.kind),
            Some(Kind::Punct('='))
        ) {
            continue;
        }
        j += 1;
        if matches!(tokens.get(j).map(|token| &token.kind), Some(Kind::Ident(word)) if word == "move")
        {
            j += 1;
        }
        if !matches!(
            tokens.get(j).map(|token| &token.kind),
            Some(Kind::Punct('|'))
        ) {
            continue;
        }
        let Some(params_end) = closure_params_end(tokens, j) else {
            continue;
        };
        let open = params_end + 1;
        if !matches!(
            tokens.get(open).map(|token| &token.kind),
            Some(Kind::Punct('{'))
        ) {
            continue;
        }
        let Some(end) = close_bracket(tokens, open) else {
            continue;
        };
        let body = (open + 1)..end;
        if !body.contains(&pos) {
            continue;
        }
        let closure = Closure {
            binding: binding.clone(),
            params: closure_params(&tokens[j + 1..params_end]),
            body,
        };
        if found
            .as_ref()
            .is_none_or(|current| current.body.start < closure.body.start)
        {
            found = Some(closure);
        }
    }
    found
}

/// The index of the `|` that closes a closure's parameter list, at bracket
/// depth 0.
fn closure_params_end(tokens: &[Token], open: usize) -> Option<usize> {
    let mut depth = 0i32;
    let mut i = open + 1;
    while i < tokens.len() {
        match tokens[i].kind {
            Kind::Punct('(') | Kind::Punct('[') | Kind::Punct('{') => depth += 1,
            Kind::Punct(')') | Kind::Punct(']') | Kind::Punct('}') => depth -= 1,
            Kind::Punct('|') if depth == 0 => return Some(i),
            _ => {}
        }
        i += 1;
    }
    None
}

/// The names a closure's parameter pattern binds: the first name of each
/// top-level parameter. `|cmd: &str, params: &[&str]|` binds `cmd` and
/// `params`; the names in the types are not bindings.
///
/// An empty result means the pattern is not the flat list this scanner maps to
/// a call's arguments one for one.
fn closure_params(pattern: &[Token]) -> Vec<String> {
    let mut out = Vec::new();
    for part in split_top_level(pattern) {
        let Some(Token {
            kind: Kind::Ident(name),
            ..
        }) = part.first()
        else {
            return Vec::new();
        };
        out.push(name.clone());
    }
    out
}

/// Every call of the closure `binding` names, as its argument list.
fn closure_calls(tokens: &[Token], binding: &str) -> Vec<Arguments> {
    let mut out = Vec::new();
    for (i, token) in tokens.iter().enumerate() {
        if !matches!(&token.kind, Kind::Ident(name) if name == binding) {
            continue;
        }
        let is_method = matches!(
            i.checked_sub(1)
                .and_then(|previous| tokens.get(previous))
                .map(|token| &token.kind),
            Some(Kind::Punct('.'))
        );
        if is_method {
            continue;
        }
        if !matches!(
            tokens.get(i + 1).map(|token| &token.kind),
            Some(Kind::Punct('('))
        ) {
            continue;
        }
        let Some(close) = close_bracket(tokens, i + 1) else {
            continue;
        };
        out.push(Arguments(
            split_top_level(&tokens[i + 2..close])
                .into_iter()
                .map(<[Token]>::to_vec)
                .collect(),
        ));
    }
    out
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
    ctx: &Ctx,
    rows: &[Vec<Vec<Token>>],
    binding: Option<usize>,
    arg: &[Token],
) -> Result<Vec<String>, String> {
    let Some(index) = binding else {
        return Ok(vec![resolve_name_scalar(arg, ctx)?]);
    };
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let field = row
            .get(index)
            .ok_or_else(|| format!("the loop element has no field {index}"))?;
        out.push(resolve_name_scalar(field, ctx)?);
    }
    Ok(out)
}

fn collect_params(
    ctx: &Ctx,
    rows: &[Vec<Vec<Token>>],
    binding: Option<usize>,
    arg: &[Token],
) -> Result<Vec<Vec<String>>, String> {
    let Some(index) = binding else {
        return Ok(vec![resolve_params_scalar(arg, ctx)?]);
    };
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let field = row
            .get(index)
            .ok_or_else(|| format!("the loop element has no field {index}"))?;
        out.push(resolve_params_scalar(field, ctx)?);
    }
    Ok(out)
}

fn collect_keys(
    ctx: &Ctx,
    rows: &[Vec<Vec<Token>>],
    binding: Option<usize>,
    arg: Option<&[Token]>,
) -> Result<Vec<Option<String>>, String> {
    let Some(arg) = arg else {
        return Ok(vec![None]);
    };
    let Some(index) = binding else {
        return Ok(vec![Some(resolve_name_scalar(arg, ctx)?)]);
    };
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let field = row
            .get(index)
            .ok_or_else(|| format!("the loop element has no field {index}"))?;
        out.push(Some(resolve_name_scalar(field, ctx)?));
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

/// A command name: a string literal, or a `const`/`static` `&str` — in this
/// file, or in a module a `use` names.
fn resolve_name_scalar(arg: &[Token], ctx: &Ctx) -> Result<String, String> {
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
        return const_string(ctx, name, 0);
    }
    Err(format!(
        "the command name is not a string literal or a constant: {}",
        render(arg)
    ))
}

fn const_string(ctx: &Ctx, name: &str, depth: usize) -> Result<String, String> {
    if depth > MAX_DEPTH {
        return Err(format!("constant {name} is nested too deeply"));
    }
    if let Some(init) = find_initializer(ctx.tokens(), name, false) {
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
            return const_string(ctx, other, depth + 1);
        }
        return Err(format!("constant {name} is not a string literal"));
    }
    let Some((file, target)) = ctx.file.imports.get(name) else {
        return Err(format!(
            "no `const {name}` declaration here or imported by a `use`"
        ));
    };
    let ctx = ctx.in_file(file)?;
    const_string(&ctx, target, depth + 1)
}

/// A parameter list: an array literal, a `const`/`static` array, or a
/// zero-argument helper that builds one.
fn resolve_params_scalar(arg: &[Token], ctx: &Ctx) -> Result<Vec<String>, String> {
    resolve_params_inner(arg, ctx, 0)
}

fn resolve_params_inner(arg: &[Token], ctx: &Ctx, depth: usize) -> Result<Vec<String>, String> {
    if depth > MAX_DEPTH {
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
    // `vec!["SAMPLE_COUNT", "AXIS"]`, the literal form of an array.
    if matches!(first.kind, Kind::Ident(ref word) if word == "vec")
        && matches!(arg.get(1).map(|token| &token.kind), Some(Kind::Punct('!')))
    {
        return resolve_params_inner(&arg[2..], ctx, depth + 1);
    }
    // `PROBE_POINTS_PARAMS.to_vec()`: a copy of a constant array.
    if let [Token {
        kind: Kind::Ident(name),
        ..
    }, Token {
        kind: Kind::Punct('.'),
        ..
    }, Token {
        kind: Kind::Ident(method),
        ..
    }, Token {
        kind: Kind::Punct('('),
        ..
    }, Token {
        kind: Kind::Punct(')'),
        ..
    }] = arg
    {
        if method == "to_vec" {
            return resolve_const_array(ctx, name, depth + 1);
        }
    }
    // `probe_points_params()`: a helper that builds the list.
    if let [Token {
        kind: Kind::Ident(name),
        ..
    }, Token {
        kind: Kind::Punct('('),
        ..
    }, Token {
        kind: Kind::Punct(')'),
        ..
    }] = arg
    {
        return resolve_params_fn(ctx, name, depth + 1);
    }
    if let [Token {
        kind: Kind::Ident(name),
        ..
    }] = arg
    {
        return resolve_const_array(ctx, name, depth + 1);
    }
    Err(format!(
        "the parameter list is neither an array literal, a constant, nor a helper: {}",
        render(arg)
    ))
}

/// The array `const`/`static` `name` holds: in this file, or in a module a `use`
/// names.
fn resolve_const_array(ctx: &Ctx, name: &str, depth: usize) -> Result<Vec<String>, String> {
    if let Some(init) = find_initializer(ctx.tokens(), name, false) {
        return resolve_params_inner(init, ctx, depth);
    }
    let Some((file, target)) = ctx.file.imports.get(name) else {
        return Err(format!(
            "no `const {name}` declaration here or imported by a `use` for the parameter list"
        ));
    };
    let ctx = ctx.in_file(file)?;
    resolve_const_array(&ctx, target, depth + 1)
}

/// The list the zero-argument helper `name` builds: in this file, or in a module
/// a `use` names.
fn resolve_params_fn(ctx: &Ctx, name: &str, depth: usize) -> Result<Vec<String>, String> {
    if let Some(body) = find_fn_body(ctx.tokens(), name) {
        return eval_params_fn(ctx, &body, name, depth);
    }
    let Some((file, target)) = ctx.file.imports.get(name) else {
        return Err(format!(
            "no `fn {name}()` here or imported by a `use` for the parameter list"
        ));
    };
    let ctx = ctx.in_file(file)?;
    resolve_params_fn(&ctx, target, depth + 1)
}

/// The body of a zero-argument `fn name` in this file: the tokens inside its
/// braces. A method of the same name takes arguments, so it is not the helper
/// this looks for.
fn find_fn_body(tokens: &[Token], name: &str) -> Option<Range<usize>> {
    for (i, token) in tokens.iter().enumerate() {
        if !matches!(&token.kind, Kind::Ident(word) if word == "fn") {
            continue;
        }
        if !matches!(tokens.get(i + 1).map(|token| &token.kind), Some(Kind::Ident(word)) if word == name)
        {
            continue;
        }
        if !matches!(
            tokens.get(i + 2).map(|token| &token.kind),
            Some(Kind::Punct('('))
        ) {
            continue;
        }
        let Some(close) = close_bracket(tokens, i + 2) else {
            continue;
        };
        if close != i + 3 {
            continue;
        }
        let Some(open) = find_body_brace(tokens, close + 1) else {
            continue;
        };
        let Some(end) = close_bracket(tokens, open) else {
            continue;
        };
        return Some((open + 1)..end);
    }
    None
}

/// The list a helper builds: its `let [mut] params = …` base, then every
/// `params.push` / `params.extend` / `params.extend_from_slice` in the order the
/// body runs them.
fn eval_params_fn(
    ctx: &Ctx,
    body: &Range<usize>,
    fn_name: &str,
    depth: usize,
) -> Result<Vec<String>, String> {
    if depth > MAX_DEPTH {
        return Err(format!(
            "the parameters `{fn_name}` builds are nested too deeply"
        ));
    }
    let tokens = &ctx.tokens()[body.clone()];
    let Some((binding, base)) = bound_list(tokens) else {
        return Err(format!("`fn {fn_name}` builds no `let` parameter list"));
    };
    let mut params = resolve_params_inner(base, ctx, depth + 1)?;
    for (method, argument) in appends(tokens, &binding)? {
        if method == "push" {
            let [Token {
                kind: Kind::Str(value),
                ..
            }] = strip_reference(argument)
            else {
                return Err(format!(
                    "`fn {fn_name}` pushes a parameter that is not a literal"
                ));
            };
            if !params.contains(value) {
                params.push(value.clone());
            }
            continue;
        }
        for param in resolve_params_inner(argument, ctx, depth + 1)? {
            if !params.contains(&param) {
                params.push(param);
            }
        }
    }
    Ok(params)
}

/// The `let [mut] NAME = …;` that starts a helper's list: the binding and the
/// tokens of its initializer.
fn bound_list(tokens: &[Token]) -> Option<(String, &[Token])> {
    for (i, token) in tokens.iter().enumerate() {
        if !matches!(&token.kind, Kind::Ident(word) if word == "let") {
            continue;
        }
        let mut j = i + 1;
        if matches!(tokens.get(j).map(|token| &token.kind), Some(Kind::Ident(word)) if word == "mut")
        {
            j += 1;
        }
        let Some(Kind::Ident(binding)) = tokens.get(j).map(|token| &token.kind) else {
            continue;
        };
        if !matches!(
            tokens.get(j + 1).map(|token| &token.kind),
            Some(Kind::Punct('='))
        ) {
            continue;
        }
        let Some(end) = statement_end(tokens, j + 2) else {
            continue;
        };
        return Some((binding.clone(), &tokens[j + 2..end]));
    }
    None
}

/// The calls that add to the list `binding` builds, in order: the method and its
/// argument. A method this scanner does not follow is an error, so a list built
/// in a way it does not understand is reported rather than half-read.
fn appends<'a>(tokens: &'a [Token], binding: &str) -> Result<Vec<(&'a str, &'a [Token])>, String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < tokens.len() {
        let is_binding = matches!(&tokens[i].kind, Kind::Ident(name) if name == binding);
        let is_method = matches!(
            i.checked_sub(1)
                .and_then(|previous| tokens.get(previous))
                .map(|token| &token.kind),
            Some(Kind::Punct('.'))
        );
        if !is_binding || is_method {
            i += 1;
            continue;
        }
        if !matches!(
            tokens.get(i + 1).map(|token| &token.kind),
            Some(Kind::Punct('.'))
        ) {
            i += 1;
            continue;
        }
        let Some(Kind::Ident(method)) = tokens.get(i + 2).map(|token| &token.kind) else {
            return Err(format!(
                "the parameter list is built with `{binding}.` and something that is not a method"
            ));
        };
        if !matches!(
            tokens.get(i + 3).map(|token| &token.kind),
            Some(Kind::Punct('('))
        ) {
            return Err(format!(
                "the parameter list is built with `{binding}.{method}`, which the scanner does not follow"
            ));
        }
        if !matches!(method.as_str(), "push" | "extend" | "extend_from_slice") {
            return Err(format!(
                "the parameter list is built with `{binding}.{method}(…)`, which the scanner does not follow"
            ));
        }
        let Some(close) = close_bracket(tokens, i + 3) else {
            return Err(format!("`{binding}.{method}(` has no closing `)`"));
        };
        out.push((method.as_str(), &tokens[i + 4..close]));
        i = close + 1;
    }
    Ok(out)
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

/// The `;` that ends the statement starting at `from`, at bracket depth 0 — a
/// `;` inside a `use` tree is not the end of the item.
fn statement_end(tokens: &[Token], from: usize) -> Option<usize> {
    let mut depth = 0i32;
    let mut i = from;
    while i < tokens.len() {
        match tokens[i].kind {
            Kind::Punct('(') | Kind::Punct('[') | Kind::Punct('{') => depth += 1,
            Kind::Punct(')') | Kind::Punct(']') | Kind::Punct('}') => depth -= 1,
            Kind::Punct(';') if depth == 0 => return Some(i),
            _ => {}
        }
        i += 1;
    }
    None
}

/// The directory of a path relative to the tree root; `""` at the root.
fn parent_dir(rel: &str) -> &str {
    match rel.rfind('/') {
        Some(index) => &rel[..index],
        None => "",
    }
}

/// The file name of a path relative to the tree root, without its extension.
fn file_stem(rel: &str) -> &str {
    let name = match rel.rfind('/') {
        Some(index) => &rel[index + 1..],
        None => rel,
    };
    name.strip_suffix(".rs").unwrap_or(name)
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
