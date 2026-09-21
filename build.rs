//! Generate the printer-level event enum from the declarations under
//! `src/core/klippy/event/decl/`.
//!
//! Each declaration file is an ordinary module holding one `event!(...)` call
//! per event. The macro itself expands to nothing (see `decl/mod.rs`), so the
//! declarations exist only to be read: this script parses them and writes
//! `KlippyEvent` plus its `name()` method into `$OUT_DIR/klippy_events.rs`,
//! which `event/printer_bus.rs` includes.
//!
//! Keeping the events in per-namespace declaration files means a module adds its
//! events next to itself instead of editing one central enum. The script fails
//! the build on a duplicate event name so a copy-paste cannot silently drop a
//! declaration.

use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

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

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("cargo sets this"));
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
            let Some((name, fields)) = parse_declaration(trimmed) else {
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

    let generated = render(&events);
    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("cargo sets this"));
    let destination = out_dir.join("klippy_events.rs");
    fs::write(&destination, generated)
        .unwrap_or_else(|err| panic!("cannot write {}: {err}", destination.display()));
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
fn parse_declaration(line: &str) -> Option<(String, Vec<(String, String)>)> {
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

/// Render the generated module.
fn render(events: &[EventDecl]) -> String {
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
    out.push_str("#[derive(Debug, Clone, PartialEq, Eq)]\n");
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
