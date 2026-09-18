use std::collections::HashMap;

use super::value::ConfigValue;

/// Represents a configuration section (e.g., `[mcu]`, `[stepper_x]`, `[printer]`)
/// The section header `[id sub]` — `sub` is optional, and its meaning is
/// determined by each specific section.
///
/// The unique key is `(id, sub)` — individually neither is unique,
/// but the combination is guaranteed to be unique within a config.
#[derive(Debug, Clone)]
pub struct ConfigSection {
    /// Unique key combining (id, sub). Used for lookups by full identifier.
    /// Neither `id` nor `sub` alone is unique, but their combination is.
    pub key: (String, Option<String>),
    /// Section id (e.g., "mcu", "stepper_x", "printer")
    /// Non-unique: multiple sections can share the same id.
    pub id: String,
    /// Section sub (optional). Its meaning is defined by the specific section.
    pub sub: Option<String>,
    /// Parameters in this section.
    pub parameters: HashMap<String, ConfigValue>,
}

/// Map of configuration sections indexed by their unique `(id, sub)` key.
///
/// Iteration preserves insertion order. The `id` alone is not unique, so
/// [`ConfigSectionMap::iter_by_id`] yields every section and callers filter
/// by the id they are interested in.
#[derive(Debug, Clone, Default)]
pub struct ConfigSectionMap {
    /// Insertion order of keys, used to iterate sections deterministically.
    keys: Vec<(String, Option<String>)>,
    /// Sections indexed by their unique `(id, sub)` key.
    by_key: HashMap<(String, Option<String>), ConfigSection>,
}

impl ConfigSectionMap {
    /// Look up a section by its unique `(id, sub)` key.
    pub fn get_by_key(&self, key: &(String, Option<String>)) -> Option<&ConfigSection> {
        self.by_key.get(key)
    }

    /// Iterate over all sections as `(key, section)` pairs in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = (&(String, Option<String>), &ConfigSection)> + '_ {
        self.keys
            .iter()
            .filter_map(move |key| self.by_key.get(key).map(|section| (key, section)))
    }

    /// Insert a section, replacing any existing section with the same key.
    pub fn insert(&mut self, section: ConfigSection) {
        let key = section.key.clone();
        if self.by_key.insert(key.clone(), section).is_none() {
            self.keys.push(key);
        }
    }

    /// Iterate over all sections. See the type-level docs for why this does
    /// not filter by a specific id.
    pub fn iter_by_id(&self) -> impl Iterator<Item = &ConfigSection> + '_ {
        self.keys.iter().filter_map(move |key| self.by_key.get(key))
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn section(id: &str, sub: Option<&str>) -> ConfigSection {
        ConfigSection::new(id, sub)
    }

    #[test]
    fn get_by_key_distinguishes_id_and_sub() {
        let mut map = ConfigSectionMap::default();
        map.insert(section("mcu", None));
        map.insert(section("mcu", Some("zboard")));

        assert!(map.get_by_key(&("mcu".to_string(), None)).is_some());
        assert!(map
            .get_by_key(&("mcu".to_string(), Some("zboard".to_string())))
            .is_some());
        assert!(map
            .get_by_key(&("mcu".to_string(), Some("other".to_string())))
            .is_none());
    }

    #[test]
    fn insert_replaces_same_key_without_duplicating() {
        let mut map = ConfigSectionMap::default();
        map.insert(section("stepper_x", None));
        map.insert(section("stepper_x", None));

        assert_eq!(map.iter().count(), 1);
    }

    #[test]
    fn iter_preserves_insertion_order() {
        let mut map = ConfigSectionMap::default();
        map.insert(section("first", None));
        map.insert(section("second", None));
        map.insert(section("third", None));

        let ids: Vec<&str> = map.iter().map(|(_, s)| s.id.as_str()).collect();
        assert_eq!(ids, vec!["first", "second", "third"]);
    }

    #[test]
    fn iter_by_id_returns_all_sections_for_filtering() {
        let mut map = ConfigSectionMap::default();
        map.insert(section("stepper_x", None));
        map.insert(section("stepper_y", None));
        map.insert(section("stepper_x", Some("extra")));

        let stepper_x: Vec<&ConfigSection> =
            map.iter_by_id().filter(|s| s.id == "stepper_x").collect();
        assert_eq!(stepper_x.len(), 2);
        assert_eq!(map.iter_by_id().count(), 3);
    }
}
