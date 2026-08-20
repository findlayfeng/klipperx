#[allow(dead_code)]
/// Parse G-code commands from input.
pub fn parse_gcode(input: &str) -> Vec<String> {
    input
        .lines()
        .map(|line| line.trim().to_string())
        .filter(|line| !line.is_empty() && !line.starts_with(';'))
        .collect()
}

#[allow(dead_code)]
/// Parse a single G-code line into components.
pub fn parse_gcode_line(line: &str) -> Vec<(String, String)> {
    let mut result = Vec::new();
    for token in line.split_whitespace() {
        if let Some((letter, value)) = token.split_once('=') {
            result.push((letter.to_uppercase(), value.to_string()));
        }
    }
    result
}
