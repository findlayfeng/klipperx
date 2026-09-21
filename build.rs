//! Generate the tables whose entries are owned by separate modules.
//!
//! Three vocabularies are collected from declarations and written into
//! `$OUT_DIR`, then `include!`d where they are used:
//!
//! | Declaration | Where it lives | Generated into | Included by |
//! |---|---|---|---|
//! | `event!(...)` | `event/decl/*.rs` | `klippy_events.rs` | `event/printer_bus.rs` |
//! | `section!(...)` | the module owning the section | `section_factories.rs` | `load.rs` |
//! | `endpoint!(...)` | the endpoint module | `endpoint_installers.rs` | `api/mod.rs` |
//!
//! A `section!`/`endpoint!` declaration names a local item; the full path is the
//! file's module path plus that name, so a declaration reads the same as any
//! other local reference (`load = load_config`). A wrong path is an unresolved
//! path at compile time, not a silently missing entry.
//!
//! The macros themselves expand to nothing (see `load.rs` and
//! `api/endpoints/mod.rs`): the declarations exist to be read.

use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("cargo sets this"));
    let src_dir = manifest_dir.join("src");
    println!("cargo:rerun-if-changed=build.rs");

    let mut sections: Vec<SectionDecl> = Vec::new();
    let mut endpoints: Vec<EndpointDecl> = Vec::new();
    for file in rust_files(&src_dir) {
        println!("cargo:rerun-if-changed={}", file.display());
        let source = fs::read_to_string(&file)
            .unwrap_or_else(|err| panic!("cannot read {}: {err}", file.display()));
        collect_sections(&source, &file, &src_dir, &mut sections);
        collect_endpoints(&source, &file, &src_dir, &mut endpoints);
    }

    generate_events(&manifest_dir);
    write_generated("section_factories.rs", &render_sections(&sections));
    write_generated("endpoint_installers.rs", &render_endpoints(&endpoints));
}

/// Write one generated file to `$OUT_DIR`.
fn write_generated(name: &str, content: &str) {
    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("cargo sets this"));
    let destination = out_dir.join(name);
    fs::write(&destination, content)
        .unwrap_or_else(|err| panic!("cannot write {}: {err}", destination.display()));
}

// ===========================================================================
// File and module paths
// ===========================================================================

/// Every `.rs` file under `dir`, sorted for a stable walk.
fn rust_files(dir: &Path) -> Vec<PathBuf> {
    let mut files = Vec::new();
    collect_rust_files(dir, &mut files);
    files.sort();
    files
}

fn collect_rust_files(dir: &Path, out: &mut Vec<PathBuf>) {
    // Watch the directory too: adding or removing a file changes its mtime,
    // which per-file tracking alone would miss.
    println!("cargo:rerun-if-changed={}", dir.display());
    let entries =
        fs::read_dir(dir).unwrap_or_else(|err| panic!("cannot read {}: {err}", dir.display()));
    for entry in entries.filter_map(|entry| entry.ok()) {
        let path = entry.path();
        if path.is_dir() {
            collect_rust_files(&path, out);
        } else if path.extension().is_some_and(|ext| ext == "rs") {
            out.push(path);
        }
    }
}

/// The module path of a source file, e.g. `crate::core::klippy::extras::output_pin`.
///
/// The crate root is `src/lib.rs`; a `mod.rs` names its directory.
fn module_path(file: &Path, src_dir: &Path) -> String {
    let relative = file
        .strip_prefix(src_dir)
        .unwrap_or_else(|_| panic!("{} is not under {}", file.display(), src_dir.display()));
    let mut parts: Vec<String> = relative
        .with_extension("")
        .components()
        .map(|part| part.as_os_str().to_string_lossy().into_owned())
        .collect();
    if parts.last().is_some_and(|part| part == "mod") {
        parts.pop();
    }
    format!("crate::{}", parts.join("::"))
}

// ===========================================================================
// Invocation scanning
// ===========================================================================

/// One `macro!( ... )` invocation found in a source file.
struct Invocation {
    inner: String,
    file: PathBuf,
    line: usize,
}

/// Find every `macro_name!( ... )` invocation in `source`.
///
/// The name must not be part of a longer identifier and must be followed by
/// `!` and `(`; the arguments may span lines. A macro definition
/// (`macro_rules! macro_name`) and a doc-comment mention without a following
/// `(` are not invocations, so a file may both define and use a macro.
fn collect_invocations(macro_name: &str, source: &str, file: &Path) -> Vec<Invocation> {
    let bytes = source.as_bytes();
    let mut found = Vec::new();
    let mut search = 0;
    while let Some(offset) = source[search..].find(macro_name) {
        let start = search + offset;
        let end = start + macro_name.len();
        search = end;
        let before_ok = start == 0 || !is_ident_byte(bytes[start - 1]);
        if !before_ok {
            continue;
        }
        let mut cursor = skip_whitespace(bytes, end);
        if bytes.get(cursor) != Some(&b'!') {
            continue;
        }
        cursor = skip_whitespace(bytes, cursor + 1);
        if bytes.get(cursor) != Some(&b'(') {
            continue;
        }
        let inner_start = cursor + 1;
        let mut depth = 1i32;
        let mut cursor = inner_start;
        while cursor < bytes.len() && depth > 0 {
            match bytes[cursor] {
                b'(' => depth += 1,
                b')' => depth -= 1,
                _ => {}
            }
            cursor += 1;
        }
        if depth != 0 {
            panic!(
                "unbalanced `{macro_name}!(` at {}:{}",
                file.display(),
                line_of(source, start)
            );
        }
        found.push(Invocation {
            inner: source[inner_start..cursor - 1].trim().to_string(),
            file: file.to_path_buf(),
            line: line_of(source, start),
        });
    }
    found
}

fn skip_whitespace(bytes: &[u8], mut cursor: usize) -> usize {
    while bytes
        .get(cursor)
        .is_some_and(|byte| byte.is_ascii_whitespace())
    {
        cursor += 1;
    }
    cursor
}

fn is_ident_byte(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || byte == b'_'
}

fn line_of(source: &str, offset: usize) -> usize {
    source[..offset]
        .bytes()
        .filter(|byte| *byte == b'\n')
        .count()
        + 1
}

// ===========================================================================
// Sections
// ===========================================================================

struct SectionDecl {
    name: String,
    order: i64,
    load: Option<String>,
    prefix: Option<String>,
    object: Option<String>,
    phase: String,
}

fn collect_sections(source: &str, file: &Path, src_dir: &Path, out: &mut Vec<SectionDecl>) {
    let module = module_path(file, src_dir);
    for invocation in collect_invocations("section", source, file) {
        out.push(parse_section(&invocation, &module));
    }
}

fn parse_section(invocation: &Invocation, module: &str) -> SectionDecl {
    let where_ = format!("{}:{}", invocation.file.display(), invocation.line);
    let parts = split_top_level(&invocation.inner);
    let name = parts
        .first()
        .and_then(|part| unquote(part))
        .unwrap_or_else(|| panic!("section! needs a \"name\" first ({where_})"));

    let mut order = None;
    let mut load = None;
    let mut prefix = None;
    let mut object = None;
    let mut phase = None;
    for part in &parts[1..] {
        let (key, value) = split_once_top_level(part, '=').unwrap_or_else(|| {
            panic!("section! option must be `key = value`, got `{part}` ({where_})")
        });
        match key.trim() {
            "order" => {
                order = Some(value.trim().parse::<i64>().unwrap_or_else(|_| {
                    panic!("section `{name}` order must be an integer ({where_})")
                }))
            }
            "load" => load = Some(resolve_item(value.trim(), module)),
            "prefix" => prefix = Some(resolve_item(value.trim(), module)),
            "object" => {
                object = Some(unquote(value.trim()).unwrap_or_else(|| {
                    panic!("section `{name}` object must be a quoted name ({where_})")
                }))
            }
            "phase" => {
                let value = value.trim();
                assert!(
                    matches!(value, "early" | "generic" | "late"),
                    "section `{name}` phase must be early|generic|late, got `{value}` ({where_})"
                );
                phase = Some(value.to_string());
            }
            other => panic!("section `{name}` has an unknown option `{other}` ({where_})"),
        }
    }

    if load.is_none() && prefix.is_none() {
        panic!("section `{name}` declares neither `load` nor `prefix` ({where_})");
    }
    let order = order.unwrap_or_else(|| panic!("section `{name}` needs an `order` ({where_})"));
    SectionDecl {
        name,
        order,
        load,
        prefix,
        object,
        phase: phase.unwrap_or_else(|| "generic".to_string()),
    }
}

fn render_sections(sections: &[SectionDecl]) -> String {
    let mut seen = HashSet::new();
    for section in sections {
        assert!(
            seen.insert(section.name.clone()),
            "duplicate section `{}` declared",
            section.name
        );
    }

    let mut ordered: Vec<&SectionDecl> = sections.iter().collect();
    ordered.sort_by(|a, b| a.order.cmp(&b.order).then_with(|| a.name.cmp(&b.name)));

    let mut out = String::new();
    out.push_str("// @generated by build.rs from the `section!` declarations. Do not edit.\n\n");
    out.push_str("const FACTORIES: &[(&str, Factories)] = &[\n");
    for section in ordered {
        out.push_str(&format!("    (\n        \"{}\",\n", section.name));
        out.push_str("        Factories {\n");
        out.push_str(&format!(
            "            load_config: {},\n",
            option_path(&section.load)
        ));
        out.push_str(&format!(
            "            load_config_prefix: {},\n",
            option_path(&section.prefix)
        ));
        out.push_str(&format!(
            "            object: {},\n",
            option_string(&section.object)
        ));
        out.push_str(&format!(
            "            phase: Phase::{},\n",
            capitalize(&section.phase)
        ));
        out.push_str("        },\n    ),\n");
    }
    out.push_str("];\n");
    out
}

fn option_path(path: &Option<String>) -> String {
    match path {
        Some(path) => format!("Some({path})"),
        None => "None".to_string(),
    }
}

/// A `Some("name")` for an `object = "name"` declaration.
fn option_string(value: &Option<String>) -> String {
    match value {
        Some(value) => format!("Some({value:?})"),
        None => "None".to_string(),
    }
}

// ===========================================================================
// Endpoints
// ===========================================================================

struct EndpointDecl {
    install: String,
}

fn collect_endpoints(source: &str, file: &Path, src_dir: &Path, out: &mut Vec<EndpointDecl>) {
    let module = module_path(file, src_dir);
    for invocation in collect_invocations("endpoint", source, file) {
        let install = resolve_item(invocation.inner.trim(), &module);
        out.push(EndpointDecl { install });
    }
}

fn render_endpoints(endpoints: &[EndpointDecl]) -> String {
    let mut seen = HashSet::new();
    for endpoint in endpoints {
        assert!(
            seen.insert(endpoint.install.clone()),
            "duplicate endpoint installer `{}` declared",
            endpoint.install
        );
    }

    let mut ordered: Vec<&str> = endpoints
        .iter()
        .map(|endpoint| endpoint.install.as_str())
        .collect();
    ordered.sort_unstable();

    let mut out = String::new();
    out.push_str("// @generated by build.rs from the `endpoint!` declarations. Do not edit.\n\n");
    out.push_str("const ENDPOINT_INSTALLERS: &[EndpointInstaller] = &[\n");
    for install in ordered {
        out.push_str(&format!("    {install},\n"));
    }
    out.push_str("];\n");
    out
}

// ===========================================================================
// Events
// ===========================================================================

/// One `event!` declaration.
struct EventDecl {
    /// Wire name, e.g. `klippy:mcu_identify`.
    name: String,
    /// Payload fields as `(field, type)`, empty for a payload-less event.
    fields: Vec<(String, String)>,
}

impl EventDecl {
    /// The enum variant name, e.g. `KlippyMcuIdentify`.
    fn variant(&self) -> String {
        variant_name(&self.name)
    }

    /// The `{ field: Type, ... }` suffix, or an empty string.
    fn fields_suffix(&self) -> String {
        if self.fields.is_empty() {
            return String::new();
        }
        let fields = self
            .fields
            .iter()
            .map(|(field, ty)| format!("{field}: {ty}"))
            .collect::<Vec<_>>()
            .join(", ");
        format!(" {{ {fields} }}")
    }

    /// The `match` pattern for this variant.
    fn pattern(&self) -> String {
        if self.fields.is_empty() {
            format!("KlippyEvent::{}", self.variant())
        } else {
            format!("KlippyEvent::{} {{ .. }}", self.variant())
        }
    }
}

/// Generate the printer event enum from `event/decl/`.
fn generate_events(manifest_dir: &Path) {
    let decl_dir = manifest_dir.join("src/core/klippy/event/decl");
    println!("cargo:rerun-if-changed={}", decl_dir.display());

    let mut decl_files = declaration_files(&decl_dir);
    decl_files.sort();

    let mut events = Vec::new();
    let mut names = HashSet::new();
    for file in &decl_files {
        println!("cargo:rerun-if-changed={}", file.display());
        let source = fs::read_to_string(file)
            .unwrap_or_else(|err| panic!("cannot read {}: {err}", file.display()));
        for (index, line) in source.lines().enumerate() {
            let trimmed = line.trim();
            let Some((name, fields)) = parse_event(trimmed) else {
                // A declaration must fit on one line. A wrapped one would
                // otherwise be skipped silently and the event would go missing.
                if trimmed.starts_with("event!(") {
                    panic!(
                        "unparsable event declaration at {}:{}; keep each event! on one \
                         line: {trimmed}",
                        file.display(),
                        index + 1
                    );
                }
                continue;
            };
            if !names.insert(name.clone()) {
                panic!(
                    "duplicate event `{name}` declared at {}:{}",
                    file.display(),
                    index + 1
                );
            }
            events.push(EventDecl { name, fields });
        }
    }
    events.sort_by(|a, b| a.name.cmp(&b.name));

    write_generated("klippy_events.rs", &render_events(&events));
}

/// The declaration files: `.rs` files under `decl/`, except `mod.rs`.
fn declaration_files(dir: &Path) -> Vec<PathBuf> {
    let entries =
        fs::read_dir(dir).unwrap_or_else(|err| panic!("cannot read {}: {err}", dir.display()));
    entries
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "rs"))
        .filter(|path| path.file_name().is_some_and(|name| name != "mod.rs"))
        .collect()
}

/// Parse one `event!("name");` or `event!("name", { field: Type, ... });` line.
///
/// The declaration must be on a single line; `None` means the line is not a
/// declaration (a comment, a `use`, an empty line) and is skipped.
fn parse_event(line: &str) -> Option<(String, Vec<(String, String)>)> {
    let rest = line.trim().strip_prefix("event!(")?.strip_suffix(");")?;
    let rest = rest.trim();
    let (name, fields) = match rest.split_once(',') {
        Some((name, fields)) => (name.trim(), Some(fields.trim())),
        None => (rest, None),
    };
    let name = name.strip_prefix('"')?.strip_suffix('"')?.to_string();
    let fields = match fields {
        None => Vec::new(),
        Some(fields) => parse_fields(fields),
    };
    Some((name, fields))
}

/// Parse `{ field: Type, ... }` into `(field, Type)` pairs.
///
/// Splitting respects `<>`, `()`, `[]`, and `{}`, so a type such as
/// `HashMap<String, Value>` stays one field.
fn parse_fields(fields: &str) -> Vec<(String, String)> {
    let inner = fields
        .strip_prefix('{')
        .and_then(|rest| rest.strip_suffix('}'))
        .unwrap_or_else(|| panic!("payload must be `{{ field: Type, ... }}`, got `{fields}`"));
    split_top_level(inner)
        .into_iter()
        .map(|field| {
            let (name, ty) = split_once_top_level(&field, ':')
                .unwrap_or_else(|| panic!("payload field must be `field: Type`, got `{field}`"));
            (name.trim().to_string(), ty.trim().to_string())
        })
        .collect()
}

fn render_events(events: &[EventDecl]) -> String {
    let mut out = String::new();
    out.push_str("// @generated by build.rs from src/core/klippy/event/decl/. Do not edit.\n\n");
    out.push_str("use serde_json::Value;\n");
    out.push_str("use std::collections::HashMap;\n\n");

    out.push_str("/// An event a printer fires at its handlers.\n");
    out.push_str("///\n");
    out.push_str("/// Each declared variant maps to a wire event name; [`KlippyEvent::name`]\n");
    out.push_str("/// returns it. Names not declared here arrive as\n");
    out.push_str("/// [`KlippyEvent::Unknown`], so an event added upstream still reaches the\n");
    out.push_str("/// handlers registered for it.\n");
    // `Eq` is deliberately absent: an event payload may be a float — a print
    // time — and `f64` is not `Eq`.
    out.push_str("#[derive(Debug, Clone, PartialEq)]\n");
    out.push_str("pub enum KlippyEvent {\n");
    for event in events {
        out.push_str(&format!("    /// `{}`\n", event.name));
        out.push_str(&format!(
            "    {}{},\n",
            event.variant(),
            event.fields_suffix()
        ));
    }
    out.push_str("    /// An event with no declared variant.\n");
    out.push_str("    Unknown {\n");
    out.push_str("        name: String,\n");
    out.push_str("        params: HashMap<String, Value>,\n");
    out.push_str("    },\n");
    out.push_str("}\n\n");

    out.push_str("impl KlippyEvent {\n");
    out.push_str("    /// The wire name of this event, as upstream sends it.\n");
    out.push_str("    pub fn name(&self) -> &str {\n");
    out.push_str("        match self {\n");
    for event in events {
        out.push_str(&format!(
            "            {} => \"{}\",\n",
            event.pattern(),
            event.name
        ));
    }
    out.push_str("            KlippyEvent::Unknown { name, .. } => name.as_str(),\n");
    out.push_str("        }\n");
    out.push_str("    }\n");
    out.push_str("}\n");
    out
}

// ===========================================================================
// Shared parsing
// ===========================================================================

/// Resolve an item named in a declaration to a full path.
///
/// A bare name (`load_config`) is a sibling of the declaring module; a path
/// (`crate::...`) is used as written.
fn resolve_item(value: &str, module: &str) -> String {
    let value = value.trim();
    if value.contains("::") {
        value.to_string()
    } else {
        format!("{module}::{value}")
    }
}

/// Strip the surrounding quotes from a string literal.
fn unquote(text: &str) -> Option<String> {
    let text = text.trim();
    Some(text.strip_prefix('"')?.strip_suffix('"')?.to_string())
}

/// Split on commas that are not inside `<>`, `()`, `[]`, or `{}`.
fn split_top_level(text: &str) -> Vec<String> {
    let mut parts = Vec::new();
    let mut current = String::new();
    let mut depth = 0i32;
    for ch in text.chars() {
        match ch {
            '<' | '(' | '[' | '{' => {
                depth += 1;
                current.push(ch);
            }
            '>' | ')' | ']' | '}' => {
                depth -= 1;
                current.push(ch);
            }
            ',' if depth == 0 => {
                parts.push(current.trim().to_string());
                current.clear();
            }
            _ => current.push(ch),
        }
    }
    if !current.trim().is_empty() {
        parts.push(current.trim().to_string());
    }
    parts
}

/// Split at the first separator that is not inside brackets.
fn split_once_top_level(text: &str, separator: char) -> Option<(String, String)> {
    let mut depth = 0i32;
    for (index, ch) in text.char_indices() {
        match ch {
            '<' | '(' | '[' | '{' => depth += 1,
            '>' | ')' | ']' | '}' => depth -= 1,
            _ if ch == separator && depth == 0 => {
                return Some((
                    text[..index].to_string(),
                    text[index + ch.len_utf8()..].to_string(),
                ));
            }
            _ => {}
        }
    }
    None
}

/// Turn a wire name into a variant name: `klippy:mcu_identify` becomes
/// `KlippyMcuIdentify`.
fn variant_name(name: &str) -> String {
    name.split(':')
        .flat_map(|namespace| namespace.split('_'))
        .map(capitalize)
        .collect()
}

/// Uppercase the first character and leave the rest as written.
fn capitalize(word: &str) -> String {
    let mut chars = word.chars();
    match chars.next() {
        None => String::new(),
        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
    }
}
