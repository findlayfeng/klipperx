use multi_index_map::MultiIndexMap;
use std::collections::HashMap;

use super::value::ConfigValue;

/// Represents a configuration section (e.g., `[mcu]`, `[stepper_x]`, `[printer]`)
/// The section header `[id sub]` — `sub` is optional, and its meaning is
/// determined by each specific section.
///
/// The unique key is `(id, sub)` — individually neither is unique,
/// but the combination is guaranteed to be unique within a config.
#[derive(MultiIndexMap, Debug, Clone)]
#[multi_index_derive(Clone, Debug)]
pub struct ConfigSection {
    /// Unique key combining (id, sub). Used for lookups by full identifier.
    /// Neither `id` nor `sub` alone is unique, but their combination is.
    #[multi_index(hashed_unique)]
    pub key: (String, Option<String>),
    /// Section id (e.g., "mcu", "stepper_x", "printer")
    /// Non-unique: multiple sections can share the same id.
    #[multi_index(hashed_non_unique)]
    pub id: String,
    /// Section sub (optional). Its meaning is defined by the specific section.
    pub sub: Option<String>,
    /// Parameters in this section.
    pub parameters: HashMap<String, ConfigValue>,
}

impl ConfigSection {
    /// Create a new empty section with the given id and optional sub.
    /// The key is automatically computed from id and sub.
    pub fn new(id: &str, sub: Option<&str>) -> Self {
        let sub = sub.map(String::from);
        Self {
            key: (id.to_string(), sub.clone()),
            id: id.to_string(),
            sub,
            parameters: HashMap::new(),
        }
    }

    /// Get the full section identifier (e.g., "mcu" or "mcu zboard")
    pub fn identifier(&self) -> String {
        match &self.sub {
            Some(sub) => format!("{} {}", self.id, sub),
            None => self.id.clone(),
        }
    }

    /// Get a parameter value by name
    pub fn get(&self, key: &str) -> Option<&ConfigValue> {
        self.parameters.get(key)
    }

    /// Get a parameter value as a string slice
    pub fn get_str(&self, key: &str) -> Option<&str> {
        self.parameters.get(key).and_then(|v| v.as_str_ref())
    }

    /// Check if section has a parameter
    pub fn has(&self, key: &str) -> bool {
        self.parameters.contains_key(key)
    }
}
