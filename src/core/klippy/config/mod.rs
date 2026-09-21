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

/// Represents a complete Klipper configuration file
#[derive(Debug, Clone)]
pub struct Config {
    /// All sections, indexed by key (unique) and id (non-unique)
    sections: section::ConfigSectionMap,
}

impl Config {
    pub fn new() -> Self {
        Self {
            sections: section::ConfigSectionMap::default(),
        }
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
        self.sections.insert(section);
    }

    /// Get all sections matching the given id
    pub fn get_sections_by_id(&self, id: &str) -> Vec<&ConfigSection> {
        self.sections.iter_by_id().filter(|s| s.id == id).collect()
    }

    /// Unified parsing entry point.
    pub fn parse(source: ConfigSource) -> Result<(Self, Vec<ConfigSource>), String> {
        let content = Self::read_source(&source)?;
        let mut visited = HashSet::new();
        visited.insert(source.clone());
        let (included_config, sources_list) =
            Self::parse_with_includes(&content, &source, &mut visited)?;
        let mut all_sources = Vec::new();
        all_sources.push(source);
        all_sources.extend(sources_list);

        let mut config = Self::new();
        for section in included_config.sections_vec() {
            config.add_section(section.clone());
        }

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
    fn parse_with_includes(
        content: &str,
        source: &ConfigSource,
        visited: &mut HashSet<ConfigSource>,
    ) -> Result<(Self, Vec<ConfigSource>), String> {
        let mut config = Self::new();
        let mut sources = Vec::new();
        let mut current_section: Option<ConfigSection> = None;
        let mut current_key: Option<String> = None;
        let mut is_multiline = false;

        for (line_num, line) in content.lines().enumerate() {
            let line_num = line_num + 1;
            let trimmed = line.trim();

            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }

            if trimmed.starts_with('[') && trimmed.ends_with(']') {
                if let Some(section) = current_section.take() {
                    if section.id == "include" {
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
                        let (included_config, mut included_sources) =
                            Self::parse_with_includes(&included_content, &include_source, visited)?;
                        sources.push(include_source.clone());
                        sources.append(&mut included_sources);

                        for section in included_config.sections_vec() {
                            config.add_section(section.clone());
                        }
                    } else {
                        config.add_section(section);
                    }
                }
                is_multiline = false;
                current_key = None;

                let section_content = &trimmed[1..trimmed.len() - 1];
                let parts: Vec<&str> = section_content.splitn(2, ' ').collect();
                let id = parts[0].trim();
                let sub = parts.get(1).map(|s| s.trim());
                current_section = Some(ConfigSection::new(id, sub));
                continue;
            }

            let section = current_section
                .as_mut()
                .ok_or_else(|| format!("Line {}: Parameter outside of section", line_num))?;

            if is_multiline && (line.starts_with(' ') || line.starts_with('\t')) {
                if let Some(key) = &current_key {
                    if let Some(ConfigValue::Multi(ref mut lines)) = section.parameters.get_mut(key)
                    {
                        lines.push(trimmed.to_string());
                        continue;
                    }
                }
            }

            is_multiline = false;

            let colon_pos = trimmed.find(':').ok_or_else(|| {
                format!("Line {}: Invalid format, expected 'key: value'", line_num)
            })?;

            let key = trimmed[..colon_pos].trim().to_string();
            let value_str = trimmed[colon_pos + 1..].trim();
            let value_str = remove_inline_comment(value_str).trim();

            current_key = Some(key.clone());

            if value_str.is_empty() {
                is_multiline = true;
                section
                    .parameters
                    .insert(key, ConfigValue::Multi(Vec::new()));
            } else {
                section
                    .parameters
                    .insert(key, ConfigValue::Single(value_str.to_string()));
            }
        }

        if let Some(section) = current_section {
            if section.id == "include" {
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
                let (included_config, mut included_sources) =
                    Self::parse_with_includes(&included_content, &include_source, visited)?;
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

fn remove_inline_comment(value: &str) -> &str {
    let mut in_quote = false;
    let quote_char = '"';
    for (i, c) in value.char_indices() {
        if c == quote_char {
            in_quote = !in_quote;
        } else if c == '#' && !in_quote {
            return &value[..i];
        }
    }
    value
}
