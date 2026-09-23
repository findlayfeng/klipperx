use std::collections::{BTreeMap, HashMap};

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
    ///
    /// Ordered by name: the parser used to keep a hash map, whose iteration
    /// order made the option check report a different bad option on each run.
    /// Upstream iterates the file's order; a sorted map is deterministic and
    /// close enough until the parser preserves insertion order.
    pub parameters: BTreeMap<String, ConfigValue>,
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
            parameters: BTreeMap::new(),
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
    ///
    /// Option names are compared case-insensitively, matching upstream's
    /// `optionxform = str.lower` behavior.
    pub fn get(&self, key: &str) -> Option<&ConfigValue> {
        self.parameters.get(&key.to_lowercase())
    }

    /// Get a parameter value as a string slice
    ///
    /// Option names are compared case-insensitively.
    pub fn get_str(&self, key: &str) -> Option<&str> {
        self.parameters
            .get(&key.to_lowercase())
            .and_then(|v| v.as_str_ref())
    }

    /// Check if section has a parameter
    ///
    /// Option names are compared case-insensitively.
    pub fn has(&self, key: &str) -> bool {
        self.parameters.contains_key(&key.to_lowercase())
    }

    /// The option's text: a `Single` as written, a `Multi` joined with newlines.
    ///
    /// This is what upstream's `configparser` hands `getlist`/`getlists`: one
    /// string, with the newlines of an indented value still in it (the list
    /// splitter trims them away). Option names are compared case-insensitively.
    pub fn get_text(&self, key: &str) -> Option<String> {
        self.parameters
            .get(&key.to_lowercase())
            .map(ConfigValue::as_str)
    }

    /// Upstream's `getlist`: split on `sep`, trim each item, drop the empty ones.
    ///
    /// Returns `None` when the option is absent, which is how a caller tells
    /// "not set" from "set to an empty list".
    pub fn get_list(&self, option: &str, sep: char) -> Option<Vec<String>> {
        self.get_text(option).map(|text| split_list(&text, sep))
    }

    /// Upstream's `getlists` with two separators: groups separated by `outer`,
    /// each group split by `inner`, with exactly `count` items per group.
    ///
    /// This is the shape `[board_pins]` uses (`seps=('=', ',')`): `A=PA0,
    /// B=PA1` is two groups of two. A missing option yields an empty vector, so
    /// the caller can treat "not set" and "set to nothing" alike.
    ///
    /// # Errors
    /// Returns a config-error message when a group does not have `count`
    /// elements, mirroring upstream's `must have N elements`.
    pub fn get_list_of_lists(
        &self,
        option: &str,
        outer: char,
        inner: char,
        count: usize,
    ) -> Result<Vec<Vec<String>>, String> {
        let Some(text) = self.get_text(option) else {
            return Ok(Vec::new());
        };
        let mut groups = Vec::new();
        for group in split_list(&text, outer) {
            let items = split_list(&group, inner);
            if items.len() != count {
                return Err(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.identifier()
                ));
            }
            groups.push(items);
        }
        Ok(groups)
    }
}

/// Split `text` on `sep` into trimmed, non-empty items.
pub(crate) fn split_list(text: &str, sep: char) -> Vec<String> {
    text.split(sep)
        .map(str::trim)
        .filter(|item| !item.is_empty())
        .map(str::to_string)
        .collect()
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

    // -----------------------------------------------------------------------
    // case folding — upstream's optionxform = str.lower
    // -----------------------------------------------------------------------

    #[test]
    fn get_looks_up_lowercase_key_when_inserted_lowercase() {
        // The parser lowercases keys on write, so a lowercase key is always stored.
        let mut section = section("extruder", None);
        section
            .parameters
            .insert("pid_kp".to_string(), ConfigValue::Single("1.0".to_string()));

        // A query with different casing still finds it.
        assert_eq!(section.get("pid_Kp"), section.get("pid_kp"));
        assert_eq!(section.get_str("PID_KP"), Some("1.0"));
        assert!(section.has("Pid_Kp"));
    }

    #[test]
    fn get_looks_up_case_insensitively_regardless_of_query_case() {
        // The parser lowercases keys at storage, so "pid_kp" is stored.
        // Lookups with any casing find it.
        let mut section = section("extruder", None);
        section
            .parameters
            .insert("pid_kp".to_string(), ConfigValue::Single("1.0".to_string()));

        assert_eq!(section.get("pid_kp"), section.get("PID_KP"));
        assert_eq!(section.get_str("Pid_Kp"), Some("1.0"));
    }

    #[test]
    fn section_names_are_not_lowercased() {
        // Only option names are folded; section ids keep their case.
        let section = section("Output_Pin Fan", None);
        assert_eq!(section.id, "Output_Pin Fan");
        assert_eq!(section.identifier(), "Output_Pin Fan");
    }

    // -----------------------------------------------------------------------
    // get_list / get_list_of_lists
    // -----------------------------------------------------------------------

    fn with(mut section: ConfigSection, key: &str, value: &str) -> ConfigSection {
        section
            .parameters
            .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        section
    }

    #[test]
    fn get_list_trims_and_drops_empty_items() {
        let section = with(section("board_pins", None), "mcu", " mcu , zboard ,");

        assert_eq!(
            section.get_list("mcu", ','),
            Some(vec!["mcu".to_string(), "zboard".to_string()])
        );
    }

    #[test]
    fn get_list_joins_multiline_values() {
        let mut section = section("board_pins", None);
        section.parameters.insert(
            "mcu".to_string(),
            ConfigValue::Multi(vec!["mcu,".to_string(), "zboard".to_string()]),
        );

        assert_eq!(
            section.get_list("mcu", ','),
            Some(vec!["mcu".to_string(), "zboard".to_string()])
        );
    }

    #[test]
    fn get_list_reports_a_missing_option() {
        assert_eq!(section("board_pins", None).get_list("mcu", ','), None);
    }

    #[test]
    fn get_list_of_lists_parses_name_value_pairs() {
        let section = with(
            section("board_pins", None),
            "aliases",
            "EXP1_1=PA0, EXP1_2=PA1,\n  EXP1_3=<GND>",
        );

        assert_eq!(
            section.get_list_of_lists("aliases", ',', '=', 2).unwrap(),
            vec![
                vec!["EXP1_1".to_string(), "PA0".to_string()],
                vec!["EXP1_2".to_string(), "PA1".to_string()],
                vec!["EXP1_3".to_string(), "<GND>".to_string()],
            ]
        );
    }

    #[test]
    fn get_list_of_lists_reports_a_wrong_item_count() {
        let section = with(section("board_pins", None), "aliases", "EXP1_1=PA0=PB0");

        let err = section
            .get_list_of_lists("aliases", ',', '=', 2)
            .unwrap_err();

        assert_eq!(
            err,
            "Option 'aliases' in section 'board_pins' must have 2 elements"
        );
    }
}
