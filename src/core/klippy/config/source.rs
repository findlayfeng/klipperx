use std::path::PathBuf;

/// Represents the source of a configuration
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum ConfigSource {
    /// Configuration loaded from a file
    File(PathBuf),
    /// Configuration loaded from a URL
    Url(String),
    /// Configuration from an in-memory string or other non-file/URL source
    /// The inner String stores the original content for potential re-parsing.
    None(String),
}

impl std::fmt::Display for ConfigSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigSource::File(path) => write!(f, "{}", path.display()),
            ConfigSource::Url(url) => write!(f, "{}", url),
            ConfigSource::None(s) => {
                let preview = if s.is_empty() {
                    "empty".to_string()
                } else if s.len() > 50 {
                    format!("{}...", &s[..50])
                } else {
                    s.clone()
                };
                write!(f, "<in-memory:{}>", preview)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_source_display() {
        let file_source = ConfigSource::File(PathBuf::from("/path/to/printer.cfg"));
        assert_eq!(format!("{}", file_source), "/path/to/printer.cfg");

        let url_source = ConfigSource::Url("https://example.com/printer.cfg".to_string());
        assert_eq!(format!("{}", url_source), "https://example.com/printer.cfg");

        let none_source = ConfigSource::None(String::new());
        assert_eq!(format!("{}", none_source), "<in-memory:empty>");

        let none_source_with_content = ConfigSource::None("test content".to_string());
        assert_eq!(
            format!("{}", none_source_with_content),
            "<in-memory:test content>"
        );
    }
}
