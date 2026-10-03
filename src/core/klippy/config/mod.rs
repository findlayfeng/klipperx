// Klipper configuration file parser module
//
// This module parses Klipper config files which are INI-like format with:
// - Sections: [id] or [id sub]
// - Parameters: key: value
// - Comments: #
// - Multiline values: indented lines continue previous value
// - Empty sections: [id] with no parameters

pub mod access;
pub mod mcu;
pub mod object;
pub mod save_config;
pub mod section;
pub mod source;
pub mod validate;
pub mod value;
pub mod wrapper;

use std::collections::HashSet;
use std::fs;
use std::io::Read;
use std::path::Path;

pub use access::AccessTracking;
pub use object::PrinterConfig;
pub use section::ConfigSection;
pub use source::ConfigSource;
pub use validate::check_unused;
pub use value::ConfigValue;
pub use wrapper::ConfigWrapper;

// Re-exported so a section module imports everything config-related from here.
pub use crate::core::klippy::error::ConfigError;

/// Lowercase an option name, mirroring upstream's `optionxform = str.lower`.
///
/// Upstream's `configparser` normalizes every option name to lowercase at storage
/// time and also lowercases lookups, so `[extruder] pid_kp: 1.0` and a query for
/// `pid_Kp` always resolve to the same entry. Section names and values are **not**
/// folded — only option names.
fn lower_option_name(name: &str) -> String {
    // Unicode-aware, matching Python's `str.lower` (which is what upstream's
    // default `optionxform` applies) — `AccessTracking` and `check_unused`
    // already fold with `to_lowercase`.
    name.to_lowercase()
}

/// Represents a complete Klipper configuration file
#[derive(Debug, Clone)]
pub struct Config {
    /// All sections, indexed by key (unique) and id (non-unique)
    sections: section::ConfigSectionMap,
    /// The `SAVE_CONFIG` block parsed on its own — after `_strip_duplicates`
    /// and before the merge — which is exactly upstream's
    /// `ConfigAutoSave.fileconfig` (`klippy/configfile.py:304`): the block a
    /// `SAVE_CONFIG` writes back. `None` when the file has no block. Boxed so
    /// the option stays a sized member of `Config`.
    autosave: Option<Box<Config>>,
}

impl Config {
    pub fn new() -> Self {
        Self {
            sections: section::ConfigSectionMap::default(),
            autosave: None,
        }
    }

    /// The `SAVE_CONFIG` block alone (upstream's `ConfigAutoSave.fileconfig`).
    pub fn autosave_block(&self) -> Option<&Config> {
        self.autosave.as_deref()
    }

    /// Remove a section by full identifier (`"mcu"` or `"mcu zboard"`);
    /// returns it when it was present. Used by `remove_section` to drop a
    /// section from the block fileconfig at the next `SAVE_CONFIG`.
    pub fn remove_identifier(&mut self, identifier: &str) -> Option<ConfigSection> {
        let parts: Vec<&str> = identifier.splitn(2, ' ').collect();
        let key = (parts[0].to_string(), parts.get(1).map(|s| s.to_string()));
        self.sections.remove(&key)
    }

    /// Get a section by full identifier (e.g., "mcu" or "mcu zboard")
    pub fn get_section(&self, identifier: &str) -> Option<&ConfigSection> {
        let parts: Vec<&str> = identifier.splitn(2, ' ').collect();
        let id = parts[0];
        let sub = parts.get(1).map(|s| s.to_string());
        self.sections.get_by_key(&(id.to_string(), sub))
    }

    pub fn sections(&self) -> impl Iterator<Item = &ConfigSection> + '_ {
        self.sections.iter().map(|(_, s)| s)
    }

    pub fn sections_vec(&self) -> Vec<&ConfigSection> {
        self.sections.iter().map(|(_, s)| s).collect()
    }

    pub fn has_section(&self, identifier: &str) -> bool {
        self.get_section(identifier).is_some()
    }

    pub fn add_section(&mut self, section: ConfigSection) {
        // Upstream parses with `RawConfigParser(strict=False)`: a repeated
        // section header joins the section already read — the option sets are
        // unioned, a later duplicate of the same option wins, and the section
        // keeps its first position (`klippy/configfile.py:172`). Merge here so
        // the factory sees one merged section and loads it exactly once.
        let Some(existing) = self.sections.get_by_key(&section.key) else {
            self.sections.insert(section);
            return;
        };
        let mut merged = existing.clone();
        // `BTreeMap::extend`: a key present in both halves takes the incoming
        // (later) value — the same last-write-wins an in-file duplicate
        // option has.
        merged.parameters.extend(section.parameters);
        // A repeated section joins option sets, so join their block provenance
        // too: the exemption must not be lost when two copies merge.
        merged.autosave_options.extend(section.autosave_options);
        self.sections.insert(merged);
    }

    /// Get all sections matching the given id
    pub fn get_sections_by_id(&self, id: &str) -> Vec<&ConfigSection> {
        self.sections.iter_by_id().filter(|s| s.id == id).collect()
    }

    /// Unified parsing entry point.
    ///
    /// A config file may carry a `SAVE_CONFIG` block below the header; it is
    /// split off here and merged back the way upstream does
    /// (`load_main_config`, `klippy/configfile.py:296-306`): the block is
    /// parsed on its own and its options are appended, minus every option the
    /// regular text already defines — **the body wins, the block only adds**.
    /// **A file without the header takes exactly the old path over exactly
    /// the same bytes** — no block, no change. Each option the block
    /// contributes is tagged on its section ([`ConfigSection::is_autosave_option`])
    /// so [`check_unused`] can exempt it, as upstream does.
    pub fn parse(source: ConfigSource) -> Result<(Self, Vec<ConfigSource>), String> {
        let content = Self::read_source(&source)?;
        let (regular, autosave) = split_autosave(&content);
        let mut visited = HashSet::new();
        visited.insert(source.clone());
        let (mut included_config, sources_list) =
            Self::parse_with_includes(regular, &source, &mut visited, true)?;
        let mut autosave_config: Option<Config> = None;
        if let Some(block) = autosave {
            // `_strip_duplicates` needs the body (with its includes) parsed
            // first; the block itself never resolves includes (upstream
            // appends it as plain text).
            let stripped = strip_autosave_duplicates(&block, &included_config);
            let (saved, _) =
                Self::parse_with_includes(&stripped, &source, &mut HashSet::new(), false)?;
            // The block, kept whole for the write-back side: a `SAVE_CONFIG`
            // re-serializes it, exactly the `ConfigAutoSave.fileconfig`
            // upstream keeps after `load_main_config`.
            autosave_config = Some(saved.clone());
            merge_autosave(&mut included_config, &saved);
        }
        let mut all_sources = Vec::new();
        all_sources.push(source);
        all_sources.extend(sources_list);

        let mut config = Self::new();
        for section in included_config.sections_vec() {
            config.add_section(section.clone());
        }
        config.autosave = autosave_config.map(Box::new);

        Ok((config, all_sources))
    }

    /// Create a Config from any source implementing `std::io::Read`.
    pub fn from_read<R: Read>(mut reader: R) -> Result<(Self, Vec<ConfigSource>), String> {
        let mut content = String::new();
        reader
            .read_to_string(&mut content)
            .map_err(|e| format!("Failed to read config: {}", e))?;
        let source = ConfigSource::None(content);
        Self::parse(source)
    }

    /// Parse a config from a string.
    ///
    /// Named `from_text` rather than `from_str` because it also returns the
    /// sources it read, which `std::str::FromStr`'s signature cannot carry.
    pub fn from_text(content: &str) -> Result<(Self, Vec<ConfigSource>), String> {
        let source = ConfigSource::None(content.to_string());
        Self::parse(source)
    }

    /// Parse a config from a file path.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<(Self, Vec<ConfigSource>), String> {
        let path = path.as_ref().to_path_buf();
        let source = ConfigSource::File(path);
        Self::parse(source)
    }

    /// Parse a config from a URL.
    pub fn from_url(url: &str) -> Result<(Self, Vec<ConfigSource>), String> {
        let client = reqwest::blocking::Client::new();
        let response = client
            .get(url)
            .send()
            .map_err(|e| format!("Failed to fetch config from URL '{}': {}", url, e))?;

        if !response.status().is_success() {
            return Err(format!(
                "Failed to fetch config from URL '{}': HTTP {}",
                url,
                response.status()
            ));
        }

        let source = ConfigSource::Url(url.to_string());
        Self::parse(source)
    }

    /// Internal parse logic with include support.
    ///
    /// `resolve_includes` is `false` for the `SAVE_CONFIG` block: upstream
    /// appends those lines as text (`append_fileconfig`), so an
    /// `[include …]` header inside one is an ordinary section, not a splice.
    fn parse_with_includes(
        content: &str,
        source: &ConfigSource,
        visited: &mut HashSet<ConfigSource>,
        resolve_includes: bool,
    ) -> Result<(Self, Vec<ConfigSource>), String> {
        let mut config = Self::new();
        let mut sources = Vec::new();
        let mut current_section: Option<ConfigSection> = None;
        let mut current_key: Option<String> = None;
        // Indentation of the line that opened the current option. A later line
        // indented deeper than this continues that option's value, which is
        // upstream `configparser`'s rule (`klippy/configfile.py:280`).
        let mut option_indent = 0usize;

        for (line_num, line) in content.lines().enumerate() {
            let line_num = line_num + 1;
            let indent = line.len() - line.trim_start().len();

            // Upstream strips the inline comment before it looks for a section
            // header, so `[fan]  # cooling fan` still opens `fan`
            // (`klippy/configfile.py:159-175`). `#` and `;` both start a
            // comment; `;` only at the start of the line or after whitespace.
            let code = remove_inline_comment(line).trim();
            if code.is_empty() {
                continue;
            }

            // A continuation is checked before a section header: an indented
            // `[b]` under an option is part of that option's value.
            if current_section.is_some() && current_key.is_some() && indent > option_indent {
                if let (Some(section), Some(key)) = (current_section.as_mut(), current_key.as_ref())
                {
                    if let Some(slot) = section.parameters.get_mut(key) {
                        match slot {
                            ConfigValue::Multi(lines) => lines.push(code.to_string()),
                            ConfigValue::Single(_) => {
                                let first = slot.as_str();
                                *slot = ConfigValue::Multi(vec![first, code.to_string()]);
                            }
                        }
                    }
                }
                continue;
            }

            if code.starts_with('[') && code.contains(']') {
                if let Some(section) = current_section.take() {
                    if section.id == "include" && resolve_includes {
                        let include_path_str = section.sub.as_deref()
                            .or_else(|| section.get_str("path"))
                            .ok_or_else(|| {
                                format!(
                                    "Line {}: [include] section requires a 'path' parameter or sub field",
                                    line_num
                                )
                            })?;

                        let include_source =
                            Self::resolve_include_source(include_path_str, source)?;

                        if visited.contains(&include_source) {
                            return Err(format!(
                                "Line {}: Circular include detected: {}",
                                line_num, include_source
                            ));
                        }

                        visited.insert(include_source.clone());
                        let included_content = Self::read_source(&include_source)?;
                        let (included_config, mut included_sources) = Self::parse_with_includes(
                            &included_content,
                            &include_source,
                            visited,
                            resolve_includes,
                        )?;
                        sources.push(include_source.clone());
                        sources.append(&mut included_sources);

                        for section in included_config.sections_vec() {
                            config.add_section(section.clone());
                        }
                    } else {
                        config.add_section(section);
                    }
                }
                current_key = None;
                option_indent = 0;

                // `SECTCRE` matches up to the last `]` and ignores what follows
                // it, so the header ends at `rfind`.
                let section_content = &code[1..code.rfind(']').unwrap_or(code.len() - 1)];
                let parts: Vec<&str> = section_content.splitn(2, ' ').collect();
                let id = parts[0].trim();
                let sub = parts.get(1).map(|s| s.trim());
                current_section = Some(ConfigSection::new(id, sub));
                continue;
            }

            let section = current_section
                .as_mut()
                .ok_or_else(|| format!("Line {}: Parameter outside of section", line_num))?;

            // `=` and `:` both separate an option from its value, and the first
            // of either wins (upstream `OPTCRE`, `klippy/configfile.py:173`).
            let separator = code.find(|c: char| c == ':' || c == '=').ok_or_else(|| {
                format!("Line {}: Invalid format, expected 'key: value'", line_num)
            })?;

            let key = code[..separator].trim().to_string();
            let value_str = code[separator + 1..].trim();

            // Upstream's `optionxform = str.lower`: store option names in lowercase.
            let key = lower_option_name(&key);
            current_key = Some(key.clone());
            option_indent = indent;
            if value_str.is_empty() {
                // The empty option line is the value's first — empty — line.
                // `configparser` appends continuation lines with `'\n'`, so
                // keeping it is what makes a value that starts with `'\n'`
                // (upstream `temperature_probe`'s `drift_calibration`,
                // `temperature_probe.py:646`) survive the round trip.
                section
                    .parameters
                    .insert(key.clone(), ConfigValue::Multi(vec![String::new()]));
            } else {
                section
                    .parameters
                    .insert(key, ConfigValue::Single(value_str.to_string()));
            }
        }

        if let Some(section) = current_section {
            if section.id == "include" && resolve_includes {
                let include_path_str = section
                    .sub
                    .as_deref()
                    .or_else(|| section.get_str("path"))
                    .ok_or_else(|| {
                    "[include] section requires a 'path' parameter or sub field".to_string()
                })?;

                let include_source = Self::resolve_include_source(include_path_str, source)?;
                if visited.contains(&include_source) {
                    return Err(format!("Circular include detected: {}", include_source));
                }
                visited.insert(include_source.clone());
                let included_content = Self::read_source(&include_source)?;
                let (included_config, mut included_sources) = Self::parse_with_includes(
                    &included_content,
                    &include_source,
                    visited,
                    resolve_includes,
                )?;
                sources.push(include_source.clone());
                sources.append(&mut included_sources);
                for section in included_config.sections_vec() {
                    config.add_section(section.clone());
                }
            } else {
                config.add_section(section);
            }
        }

        Ok((config, sources))
    }

    /// Read content from a ConfigSource
    fn read_source(source: &ConfigSource) -> Result<String, String> {
        match source {
            ConfigSource::File(path) => fs::read_to_string(path)
                .map_err(|e| format!("Failed to read file '{}': {}", path.display(), e)),
            ConfigSource::Url(url) => {
                let client = reqwest::blocking::Client::new();
                let response = client
                    .get(url)
                    .send()
                    .map_err(|e| format!("Failed to fetch URL '{}': {}", url, e))?;
                if !response.status().is_success() {
                    return Err(format!(
                        "Failed to fetch URL '{}': HTTP {}",
                        url,
                        response.status()
                    ));
                }
                response
                    .text()
                    .map_err(|e| format!("Failed to read URL '{}': {}", url, e))
            }
            ConfigSource::None(s) => Ok(s.clone()),
        }
    }

    /// Get the parent directory of a URL.
    pub(crate) fn url_parent(url: &str) -> Option<String> {
        let path_start = if let Some(pos) = url.find("://") {
            pos + 3
        } else {
            0
        };

        if let Some(last_slash) = url[path_start..].rfind('/') {
            let absolute_pos = path_start + last_slash;
            if absolute_pos > path_start {
                return Some(url[..absolute_pos].to_string());
            }
        }
        None
    }

    /// Normalize a URL path by resolving . and .. segments.
    pub(crate) fn normalize_url_path(url: &str) -> String {
        let protocol_end = if let Some(pos) = url.find("://") {
            pos + 3
        } else {
            0
        };

        let path_start = if protocol_end < url.len() {
            if let Some(pos) = url[protocol_end..].find('/') {
                protocol_end + pos
            } else {
                url.len()
            }
        } else {
            0
        };

        let (prefix, path) = if path_start < url.len() {
            (&url[..path_start], &url[path_start..])
        } else {
            (url, "")
        };

        let segments: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();
        let mut resolved = Vec::new();
        for segment in segments {
            match segment {
                "." => continue,
                ".." => {
                    resolved.pop();
                }
                _ => resolved.push(segment),
            }
        }

        if resolved.is_empty() {
            prefix.to_string()
        } else {
            let path_str = "/".to_owned() + &resolved.join("/");
            format!("{}{}", prefix, path_str)
        }
    }

    /// Resolve an include path relative to the current config's source.
    pub(crate) fn resolve_include_source(
        path_param: &str,
        source: &ConfigSource,
    ) -> Result<ConfigSource, String> {
        if path_param.starts_with("http://") || path_param.starts_with("https://") {
            return Ok(ConfigSource::Url(path_param.to_string()));
        }

        let path = Path::new(path_param);
        match source {
            ConfigSource::File(base_path) => {
                if path.is_absolute() {
                    Ok(ConfigSource::File(path.to_path_buf()))
                } else {
                    let base_dir = base_path.parent().unwrap_or(Path::new("."));
                    Ok(ConfigSource::File(base_dir.join(path)))
                }
            }
            ConfigSource::Url(base_url) => {
                if let Some(base_dir) = Self::url_parent(base_url) {
                    let resolved_url = format!("{}/{}", base_dir, path_param);
                    Ok(ConfigSource::Url(Self::normalize_url_path(&resolved_url)))
                } else {
                    Ok(ConfigSource::Url(path_param.to_string()))
                }
            }
            ConfigSource::None(_) => {
                let cwd = std::env::current_dir()
                    .map_err(|e| format!("Failed to get current directory: {}", e))?;
                let resolved_path = cwd.join(path);
                Ok(ConfigSource::File(resolved_path))
            }
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// Strip a trailing comment from a line, leaving quoted text alone.
///
/// `#` starts a comment anywhere: upstream truncates the line at the first `#`
/// (`klippy/configfile.py:159-167`). `;` starts one only at the start of the
/// line or after whitespace, which is `configparser`'s inline-prefix rule
/// (used via `inline_comment_prefixes`, `klippy/configfile.py:172-175`); a
/// value such as `foo;bar` is therefore left intact.
fn remove_inline_comment(value: &str) -> &str {
    let mut in_quote = false;
    let bytes = value.as_bytes();
    for (i, c) in value.char_indices() {
        match c {
            '"' => in_quote = !in_quote,
            '#' if !in_quote => return &value[..i],
            ';' if !in_quote => {
                let at_start = i == 0;
                let after_space = i > 0 && bytes[i - 1].is_ascii_whitespace();
                if at_start || after_space {
                    return &value[..i];
                }
            }
            _ => {}
        }
    }
    value
}

// ---------------------------------------------------------------------------
// Value text
// ---------------------------------------------------------------------------

/// A float spelled the way Python's `str()` spells it.
///
/// Upstream hands raw floats to `configfile.set()` (a `SAVE_CONFIG` value) and
/// to `save_variables`'s `repr`, so the text a saved value gets is Python's own
/// rendering. Rust's `{:?}` picks the same digits and switches to an exponent
/// at the same places (fixed from `1e-4` through `1e15`), and differs only in
/// two spellings, both fixed up here: the exponent loses its sign and leading
/// zero (`1e-5` for Python's `1e-05`) and NaN reads `NaN` for `nan`.
pub(crate) fn py_float_str(value: f64) -> String {
    if value.is_nan() {
        return "nan".to_string();
    }
    let text = format!("{value:?}");
    let Some((mantissa, exponent)) = text.split_once('e') else {
        return text;
    };
    let (sign, digits) = match exponent.strip_prefix('-') {
        Some(digits) => ('-', digits),
        None => ('+', exponent),
    };
    format!("{mantissa}e{sign}{digits:0>2}")
}

// ---------------------------------------------------------------------------
// The SAVE_CONFIG block (`klippy/configfile.py:233-294`)
// ---------------------------------------------------------------------------

/// The block header, byte for byte (`AUTOSAVE_HEADER`, `configfile.py:233-237`).
const AUTOSAVE_HEADER: &str = concat!(
    "\n#*# <---------------------- SAVE_CONFIG ---------------------->\n",
    "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n",
    "#*#\n",
);

/// Split the `SAVE_CONFIG` block off a config file's raw text
/// (`_find_autosave_data`, `configfile.py:248-272`).
///
/// Returns the regular text and the block's lines with each `#*# ` prefix
/// removed. A corrupted block is a warning and **no split**: the whole text
/// stays regular, which is exactly what a file without the block always was
/// (its `#*#` lines are comments). The upstream warnings keep their wording.
fn split_autosave(data: &str) -> (&str, Option<String>) {
    let Some(pos) = data.find(AUTOSAVE_HEADER) else {
        return (data, None);
    };
    let regular = &data[..pos];
    let autosave = data[pos + AUTOSAVE_HEADER.len()..].trim();
    if regular.contains("\n#*# ") {
        tracing::warn!("Can't read autosave from config file - autosave state corrupted");
        return (data, None);
    }
    let mut lines = Vec::new();
    for line in autosave.split('\n') {
        // Upstream refuses a line that is not `#*# `-prefixed — once there is
        // a block at all (an empty block has no lines to check).
        let malformed = (!line.starts_with("#*#")
            || (line.len() >= 4 && !line.starts_with("#*# ")))
            && !autosave.is_empty();
        if malformed {
            tracing::warn!("Can't read autosave from config file - modifications after header");
            return (data, None);
        }
        // `line[4:]`: `#*# ` is four bytes; a bare `#*#` has no tail.
        lines.push(if line.len() >= 4 { &line[4..] } else { "" });
    }
    (regular, Some(lines.join("\n")))
}

/// Comment out every block line whose option the regular text already defines
/// (`_strip_duplicates`, `configfile.py:273-294`): the body wins, the block
/// only contributes new options. Continuation lines of a commented field go
/// with it.
fn strip_autosave_duplicates(block: &str, regular: &Config) -> String {
    // Upstream's naive comment cut for this text (the block's lines carry no
    // quoted `#`/`;`); the main parser keeps its own quote-aware rule.
    fn cut_comment(line: &str) -> &str {
        match line.find(['#', ';']) {
            Some(index) => &line[..index],
            None => line,
        }
    }
    let mut section: Option<String> = None;
    let mut is_dup_field = false;
    let mut out = Vec::new();
    for line in block.split('\n') {
        let pruned = cut_comment(line).trim_end();
        if pruned.is_empty() {
            out.push(line.to_string());
            continue;
        }
        if pruned.starts_with(char::is_whitespace) {
            out.push(if is_dup_field {
                format!("#{line}")
            } else {
                line.to_string()
            });
            continue;
        }
        is_dup_field = false;
        if pruned.starts_with('[') {
            section = pruned
                .strip_prefix('[')
                .and_then(|rest| rest.strip_suffix(']'))
                .map(str::trim)
                .map(str::to_string);
            out.push(line.to_string());
            continue;
        }
        // The field: the leading `[A-Za-z0-9_]` run (`value_r`).
        let field: String = pruned
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        let defined = section
            .as_deref()
            .and_then(|name| regular.get_section(name))
            .is_some_and(|section| section.has(&field));
        if defined {
            is_dup_field = true;
            out.push(format!("#{line}"));
        } else {
            out.push(line.to_string());
        }
    }
    out.join("\n")
}

/// Append the parsed block onto the body's config, option by option —
/// upstream's `append_fileconfig(regular_fileconfig, autosave_data, …)`
/// (`configfile.py:305`): a body-defined option stays (its block twin was
/// already stripped), a new one is added to its section (created if the body
/// lacks it), and insertion order keeps the body's sections in place. Every
/// option the block contributes is tagged as an autosave option, which is what
/// `check_unused` exempts.
fn merge_autosave(body: &mut Config, saved: &Config) {
    for section in saved.sections_vec() {
        let key = (section.id.clone(), section.sub.clone());
        match body.sections.get_by_key(&key).cloned() {
            None => {
                // The whole section came from the block, so every option in it
                // is an autosave option.
                let mut added = section.clone();
                for option in added.parameters.keys() {
                    added.autosave_options.insert(option.clone());
                }
                body.add_section(added);
            }
            Some(mut existing) => {
                for (option, value) in &section.parameters {
                    if !existing.parameters.contains_key(option) {
                        existing.parameters.insert(option.clone(), value.clone());
                        existing.autosave_options.insert(option.clone());
                    }
                }
                // Replacing under the same key keeps the body's position in
                // the iteration order (`insert` only appends unknown keys).
                body.add_section(existing);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Serialization (the write half of the `SAVE_CONFIG` block)
// ---------------------------------------------------------------------------

/// Serialize a config's sections to plain ini text, upstream's
/// `build_config_string` (`configfile.py:152-155`): `[identifier]`, one
/// `option = value` line per parameter, and a blank line closing each
/// section. A multi-line value's continuation lines are indented with a tab,
/// exactly as `configparser.write` leaves them (`'\n'` → `'\n\t'`), so the
/// block reads back as one option instead of a run of invalid lines.
pub fn build_config_string(config: &Config) -> String {
    let mut out = String::new();
    for section in config.sections_vec() {
        out.push_str(&format!("[{}]\n", section.identifier()));
        for (option, value) in &section.parameters {
            let value = value.as_str().replace('\n', "\n\t");
            out.push_str(&format!("{option} = {value}\n"));
        }
        out.push('\n');
    }
    out
}

/// The text a `SAVE_CONFIG` appends below the regular config: the block
/// config, every line `#*# `-prefixed, with the header and a trailing blank in
/// exactly the shape upstream's `cmd_SAVE_CONFIG` builds
/// (`configfile.py:346-360`). The leading newline keeps the block off the last
/// regular line without adding a separator upstream does not.
pub fn build_autosave_block(config: &Config) -> String {
    let text = build_config_string(config);
    let mut lines: Vec<String> = text
        .split('\n')
        .map(|line| format!("#*# {line}").trim_end().to_string())
        .collect();
    lines.insert(0, format!("\n{}", AUTOSAVE_HEADER.trim_end()));
    lines.push(String::new());
    lines.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(text: &str) -> Config {
        Config::from_text(text).expect("the config parses").0
    }

    fn value(config: &Config, section: &str, option: &str) -> String {
        config
            .get_section(section)
            .unwrap_or_else(|| panic!("section {section}"))
            .get(option)
            .unwrap_or_else(|| panic!("option {option}"))
            .as_str()
    }

    /// The expected strings are CPython's `repr` output (`str` of a float is
    /// the same); `{:?}` alone misses the four exponent spellings.
    #[test]
    fn a_float_is_spelled_the_way_python_spells_it() {
        let cases: &[(f64, &str)] = &[
            (0.0, "0.0"),
            (-0.0, "-0.0"),
            (1.0, "1.0"),
            (12.0, "12.0"),
            (-1.0, "-1.0"),
            (0.5, "0.5"),
            (8400000.0, "8400000.0"),
            (2.675, "2.675"),
            (1.0 / 3.0, "0.3333333333333333"),
            // The fixed/scientific switch sits where Python's does.
            (0.0001, "0.0001"),
            (1e-5, "1e-05"),
            (1e15, "1000000000000000.0"),
            (1e16, "1e+16"),
            (1.5e16, "1.5e+16"),
            (1e-7, "1e-07"),
            (5e-324, "5e-324"),
            (1.7976931348623157e308, "1.7976931348623157e+308"),
            (f64::INFINITY, "inf"),
            (f64::NEG_INFINITY, "-inf"),
            (f64::NAN, "nan"),
        ];
        for (value, expected) in cases {
            assert_eq!(&py_float_str(*value), expected, "for {value}");
        }
    }

    #[test]
    fn a_section_header_may_carry_a_hash_comment() {
        // config/generic-duet2-duex.cfg:358, config/generic-remram.cfg:106
        let config = parse("[output_pin GPIO1] # broken out on the duex\npin: PA0\n");
        assert_eq!(
            config
                .get_section("output_pin GPIO1")
                .unwrap()
                .get_str("pin"),
            Some("PA0")
        );
    }

    #[test]
    fn a_section_header_may_carry_a_semicolon_comment() {
        // test/klippy/macros.cfg:119
        let config = parse("[gcode_macro TEST_unicode]  ; comment ( ° )\nvariable_ABC: 25\n");
        assert_eq!(
            config
                .get_section("gcode_macro TEST_unicode")
                .unwrap()
                .get_str("variable_ABC"),
            Some("25")
        );
    }

    #[test]
    fn equals_separates_an_option_like_a_colon() {
        // config/sample-mmu2s-diy.cfg:110, test/klippy/eddy.cfg:84,
        // test/klippy/extruders.cfg:71
        let config = parse("[gcode_macro X]\nvariable_colorselector = [71,57]\nswitch_pin: PD4\n");
        let section = config.get_section("gcode_macro X").unwrap();
        assert_eq!(section.get_str("variable_colorselector"), Some("[71,57]"));
        assert_eq!(section.get_str("switch_pin"), Some("PD4"));
    }

    #[test]
    fn the_first_separator_of_either_kind_wins() {
        let config = parse("[s]\na = x:y\nb: x=y\n");
        let section = config.get_section("s").unwrap();
        assert_eq!(section.get_str("a"), Some("x:y"));
        assert_eq!(section.get_str("b"), Some("x=y"));
    }

    #[test]
    fn an_indented_line_continues_a_non_empty_value() {
        // config/printer-lulzbot-*.cfg `[bed_tilt] points:`
        let config =
            parse("[bed_tilt]\npoints: -2, -6\n        156, -6\n        156, 158\nspeed: 75\n");
        assert_eq!(
            value(&config, "bed_tilt", "points"),
            "-2, -6\n156, -6\n156, 158"
        );
        assert_eq!(
            config.get_section("bed_tilt").unwrap().get_str("speed"),
            Some("75")
        );
    }

    #[test]
    fn an_equals_option_may_continue_from_an_empty_value() {
        // The empty option line is the value's first — empty — line, exactly
        // what upstream's `configparser` hands back (`get` on this text
        // returns `'\n0.05:3300,0.10:3200,\n0.20:2900'`, measured in Python),
        // so the continuation lines hang below it.
        let config = parse(
            "[probe_eddy_current eddy]\ncalibrate =\n    0.05:3300,0.10:3200,\n    0.20:2900\nspeed: 1\n",
        );
        let section = config.get_section("probe_eddy_current eddy").unwrap();
        assert_eq!(
            section.get("calibrate").unwrap().lines(),
            vec!["", "0.05:3300,0.10:3200,", "0.20:2900"]
        );
        assert_eq!(section.get_str("speed"), Some("1"));
    }

    #[test]
    fn an_indented_bracket_is_a_continuation_not_a_section() {
        // Continuations are checked before section headers, as upstream does.
        let config = parse("[s]\nkey: a\n  [b]\n");
        assert_eq!(value(&config, "s", "key"), "a\n[b]");
        assert!(config.get_section("b").is_none());
    }

    #[test]
    fn a_semicolon_comment_needs_leading_whitespace() {
        let config = parse("[s]\na: x;y\nb: x ; y\nc: x\t; y\n");
        let section = config.get_section("s").unwrap();
        assert_eq!(section.get_str("a"), Some("x;y"));
        assert_eq!(section.get_str("b"), Some("x"));
        assert_eq!(section.get_str("c"), Some("x"));
    }

    #[test]
    fn a_hash_inside_quotes_is_not_a_comment() {
        let config = parse("[s]\nkey: \"a#b\"\n");
        assert_eq!(value(&config, "s", "key"), "\"a#b\"");
    }

    #[test]
    fn a_repeated_section_header_merges_the_option_sets() {
        // test/klippy/eddy.cfg has two `[probe_eddy_current eddy]` sections;
        // upstream's RawConfigParser(strict=False) joins them into one.
        let config = parse(
            "[probe_eddy_current eddy]\ni2c_mcu: mcu\ni2c_address: 35\n\
             [probe_eddy_current eddy]\nsensor_type: ldc1612\nspeed: 5\n",
        );
        let section = config.get_section("probe_eddy_current eddy").unwrap();
        assert_eq!(section.get_str("i2c_mcu"), Some("mcu"));
        assert_eq!(section.get_str("i2c_address"), Some("35"));
        assert_eq!(section.get_str("sensor_type"), Some("ldc1612"));
        assert_eq!(section.get_str("speed"), Some("5"));
        // One merged section, not two entries.
        assert_eq!(config.get_sections_by_id("probe_eddy_current").len(), 1);
    }

    #[test]
    fn a_duplicate_option_in_a_repeated_section_takes_the_later_value() {
        let config = parse("[s]\na: first\nb: keep\n[s]\na: second\nc: new\n");
        let section = config.get_section("s").unwrap();
        assert_eq!(section.get_str("a"), Some("second"));
        assert_eq!(section.get_str("b"), Some("keep"));
        assert_eq!(section.get_str("c"), Some("new"));
    }

    #[test]
    fn a_merged_section_stays_at_its_first_position() {
        // ConfigSectionMap::insert only pushes the key when it is new, so
        // re-inserting the merged section must keep it at the first slot.
        let config = parse("[a]\nx: 1\n[sdup]\np: 1\n[b]\ny: 2\n[sdup]\nq: 2\n");
        let order: Vec<String> = config
            .sections_vec()
            .iter()
            .map(|s| s.identifier())
            .collect();
        assert_eq!(order, ["a", "sdup", "b"]);
        // ...and the merged options are all present there.
        let section = config.get_section("sdup").unwrap();
        assert_eq!(section.get_str("p"), Some("1"));
        assert_eq!(section.get_str("q"), Some("2"));
    }

    #[test]
    fn distinct_sections_are_unaffected_by_merging() {
        let config = parse("[mcu]\nserial: /dev/ttyUSB0\n[stepper_x]\nstep_pin: PA1\n[mcu secondary]\nserial: /dev/ttyUSB1\n");
        assert_eq!(config.sections_vec().len(), 3);
        assert_eq!(
            config.get_section("mcu").unwrap().get_str("serial"),
            Some("/dev/ttyUSB0")
        );
        assert_eq!(
            config.get_section("stepper_x").unwrap().get_str("step_pin"),
            Some("PA1")
        );
        assert_eq!(
            config
                .get_section("mcu secondary")
                .unwrap()
                .get_str("serial"),
            Some("/dev/ttyUSB1")
        );
    }
    // -----------------------------------------------------------------------
    // The SAVE_CONFIG block (`_find_autosave_data` + `_strip_duplicates`)
    // -----------------------------------------------------------------------

    /// The header as upstream writes it — pinned so a byte of drift fails.
    #[test]
    fn the_save_config_header_matches_upstream_bytes() {
        assert_eq!(
            AUTOSAVE_HEADER,
            "\n#*# <---------------------- SAVE_CONFIG ---------------------->\n\
             #*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n\
             #*#\n"
        );
    }

    /// The exact delta_calibrate.cfg shape: body, header, prefixed lines.
    fn with_block(body: &str, block: &str) -> String {
        format!(
            "{body}\n#*# <---------------------- SAVE_CONFIG ---------------------->\n\
             #*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n\
             #*#\n{block}"
        )
    }

    #[test]
    fn a_config_without_a_block_parses_exactly_as_before() {
        // No header anywhere: the parse input is byte-for-byte the old one —
        // a `#*#`-looking line stays an ordinary comment.
        let text = "[s]\na: 1\n#*# not a block = 2\nb: 3\n";
        let config = parse(text);
        let section = config.get_section("s").unwrap();
        assert_eq!(section.get_str("a"), Some("1"));
        assert_eq!(section.get_str("b"), Some("3"));
        assert!(!section.has("not"));
        assert!(!section.has("not a block"));
        // And the split helper declines the file outright.
        assert_eq!(split_autosave(text), (text, None));
    }

    #[test]
    fn the_block_is_split_and_its_prefixes_stripped() {
        let text = with_block(
            "[printer]\nkinematics: delta\n",
            "#*# [printer]\n#*# delta_radius = 174.750004\n",
        );
        let (regular, block) = split_autosave(&text);
        assert_eq!(regular, "[printer]\nkinematics: delta\n");
        let block = block.expect("the block splits");
        assert_eq!(block, "[printer]\ndelta_radius = 174.750004");

        let config = parse(&text);
        // The prefixed lines parse as ordinary options: `#*#` never reaches a
        // value.
        let printer = config.get_section("printer").unwrap();
        assert_eq!(printer.get_str("delta_radius"), Some("174.750004"));
        assert_eq!(printer.get_str("kinematics"), Some("delta"));
    }

    #[test]
    fn the_block_adds_new_options_but_never_overrides_the_body() {
        // `_strip_duplicates`: the body wins, the block appends what the body
        // does not define.
        let text = with_block(
            "[delta_calibrate]\nradius: 50\n",
            "#*# [delta_calibrate]\n#*# radius = 99\n#*# height0 = 0.0\n",
        );
        let config = parse(&text);
        let section = config.get_section("delta_calibrate").unwrap();
        assert_eq!(section.get_str("radius"), Some("50"), "the body wins");
        assert_eq!(section.get_str("height0"), Some("0.0"), "the block adds");
    }

    #[test]
    fn the_block_may_open_sections_the_body_lacks() {
        let text = with_block(
            "[printer]\nkinematics: delta\n",
            "#*# [stepper_a]\n#*# angle = 210.0\n",
        );
        let config = parse(&text);
        assert_eq!(
            config.get_section("stepper_a").unwrap().get_str("angle"),
            Some("210.0")
        );
    }

    // -----------------------------------------------------------------------
    // Multi-line values in the block — the SAVE_CONFIG round trip
    // -----------------------------------------------------------------------

    /// A pending value the way `temperature_probe`'s `finish_calibration`
    /// queues one: upstream's `"\n" + "\n".join(polys)`
    /// (`temperature_probe.py:646`) — an empty first line, then the curves.
    const MULTILINE_VALUE: &str = "\n300, 0, 0\n200, 0, 0\n100, 0, 0";

    /// A one-section block fileconfig holding `option = value`.
    fn block_with(option: &str, value: &str) -> Config {
        let mut section = ConfigSection::new("temperature_probe", Some("probe"));
        section
            .parameters
            .insert(option.to_string(), ConfigValue::Single(value.to_string()));
        let mut config = Config::new();
        config.add_section(section);
        config
    }

    /// `configparser` writes a multi-line value as `key = ` with every
    /// following line indented by a tab (`configfile.py:152-155` through
    /// `ConfigParser.write`); the `#*# ` prefix keeps that indentation, so the
    /// block stays a value the parser reads back as one option.
    #[test]
    fn a_multiline_block_value_is_written_with_indented_continuations() {
        let text = build_autosave_block(&block_with("drift_calibration", MULTILINE_VALUE));
        assert!(
            text.contains(
                "#*# drift_calibration =\n#*# \t300, 0, 0\n#*# \t200, 0, 0\n#*# \t100, 0, 0"
            ),
            "{text}"
        );
    }

    /// Write, split, parse, and the value must be byte-for-byte the one that
    /// went in — a multi-line value that only parses is not enough: a restart
    /// must load exactly what the calibration saved.
    #[test]
    fn a_multiline_block_value_round_trips_verbatim() {
        let text = format!(
            "[temperature_probe probe]\nsensor_type: Scripted\n{}",
            build_autosave_block(&block_with("drift_calibration", MULTILINE_VALUE))
        );
        let config = parse(&text);
        assert_eq!(
            value(&config, "temperature_probe probe", "drift_calibration"),
            MULTILINE_VALUE
        );
    }

    #[test]
    fn a_corrupted_block_leaves_the_file_untouched() {
        // A `#*# `-prefixed line *above* the header corrupts the split
        // (`configfile.py:256-258`): nothing splits, every `#*#` line is the
        // comment it would have been without the feature.
        let text = with_block(
            "[printer]\n#*# ghost = 1\nkinematics: delta\n",
            "#*# [printer]\n#*# delta_radius = 174.750004\n",
        );
        let (regular, block) = split_autosave(&text);
        assert_eq!(regular, text, "no split");
        assert_eq!(block, None);

        // A line inside the block without the prefix corrupts it too
        // (`modifications after header`).
        let text = with_block("[printer]\n", "#*# [printer]\ntampered = 1\n");
        let (regular, block) = split_autosave(&text);
        assert_eq!(regular, text, "no split");
        assert_eq!(block, None);
        // The tampered line is ordinary text in that case, and the block's
        // options stay comments.
        let config = parse(&text);
        let printer = config.get_section("printer").unwrap();
        assert_eq!(printer.get_str("tampered"), Some("1"));
        assert!(printer.get("delta_radius").is_none());
    }
}
