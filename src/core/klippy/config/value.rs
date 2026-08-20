/// Represents a configuration value, which can be a single line or multiple lines
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigValue {
    /// Single line value
    Single(String),
    /// Multiple lines (for parameters like `points:`)
    Multi(Vec<String>),
}

impl ConfigValue {
    pub fn as_str(&self) -> String {
        match self {
            ConfigValue::Single(s) => s.clone(),
            ConfigValue::Multi(lines) => lines.join("\n"),
        }
    }

    pub fn as_str_ref(&self) -> Option<&str> {
        match self {
            ConfigValue::Single(s) => Some(s.as_str()),
            ConfigValue::Multi(_) => None,
        }
    }

    pub fn lines(&self) -> Vec<&str> {
        match self {
            ConfigValue::Single(s) => vec![s.as_str()],
            ConfigValue::Multi(lines) => lines.iter().map(|l| l.as_str()).collect(),
        }
    }
}

impl std::fmt::Display for ConfigValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigValue::Single(s) => write!(f, "{}", s),
            ConfigValue::Multi(lines) => write!(f, "{}", lines.join("\n")),
        }
    }
}
