//! Python literal syntax for `[gcode_macro]` values — the port of
//! `ast.literal_eval`.
//!
//! Upstream reads `variable_*` options and `SET_GCODE_VARIABLE VALUE=` with
//! Python's `ast.literal_eval` (`gcode_macro.py:153-162`, `:186-191`), so a
//! client may write `False`, `'single-quoted'`, a tuple or a dict with
//! unquoted keys. This port used to read them as JSON, which refused all of
//! those — including the `variable_last_extruder_temp` dict Mainsail ships in
//! its own macro file. [`parse`] accepts the literal subset Klipper relies on
//! and returns the [`Value`] the parser reports through `get_status`.
//!
//! # Accepted
//!
//! - `None`, `True`, `False` (case sensitive, as in Python)
//! - integers: decimal, and the `0x` / `0o` / `0b` bases (plus `_` digit
//!   separators), with an optional leading `+` / `-`
//! - floats: `1.5`, `.5`, `1.`, `1e9`, `1.5E-3`, also signed
//! - strings: single or double quoted, with Python's escapes — `\\ \' \" \n
//!   \t \r \a \b \f \v`, `\xHH`, `\uHHHH`, `\UHHHHHHHH`, `\ooo` octal, and a
//!   backslash-newline continuation. An unknown escape keeps its backslash,
//!   which is what Python does for e.g. `'\d'`.
//! - sequences: `[a, b]` and `(a, b)`, both with a trailing comma and an empty
//!   form; a parenthesised single value `(a)` is that value, as in Python
//! - mappings: `{k: v}`, including a trailing comma
//!
//! # Refused
//!
//! Names beyond the three constants (so no lookup, no function call), any
//! arithmetic, and `{a, b}` set literals — a JSON document has no set, and
//! neither the config files nor the clients use one.
//!
//! # Python and JSON spellings in one grammar
//!
//! The grammar is a *union* rather than "try Python, then fall back to JSON":
//! `true` / `false` / `null` are read as `True` / `False` / `None` in the same
//! pass, so every value the old JSON reader accepted still parses, and an
//! error names the one literal that was bad instead of the failure of a second
//! attempt at it. The two spellings differ in exactly one corner: JSON's `"\/"`
//! escape means `/`, while Python keeps an unknown escape verbatim, so `"a\/b"`
//! now reads `a\/b`.
//!
//! # Mapping keys
//!
//! JSON object keys are strings only, and upstream's values reach the wire as
//! JSON (`json.dumps` in `:158`, `:190`), which stringifies a numeric or
//! boolean key of a Python dict. Keys are converted the same way here — `1` to
//! `"1"`, `1.5` to `"1.5"`, `True` to `"True"`, `None` to `"None"` — while a
//! list or dict key is refused, as Python's unhashable-key `TypeError` would.
//! A repeated key keeps the last value, which is Python's rule too.

use std::fmt;

use serde_json::{Number, Value};

/// Why a value is not a literal, with the byte offset it gave up at.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LiteralError {
    what: String,
    offset: usize,
}

impl fmt::Display for LiteralError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} at offset {}", self.what, self.offset)
    }
}

impl std::error::Error for LiteralError {}

/// Parse `input` as a Python literal, the way upstream's `ast.literal_eval`
/// does for the subset in the module docs. Leading and trailing whitespace is
/// ignored; anything else left over after the value is an error.
pub fn parse(input: &str) -> Result<Value, LiteralError> {
    let mut parser = Parser {
        input: input.as_bytes(),
        pos: 0,
    };
    parser.skip_whitespace();
    let value = parser.value()?;
    parser.skip_whitespace();
    if !parser.at_end() {
        return Err(parser.error("unexpected trailing characters"));
    }
    Ok(value)
}

struct Parser<'a> {
    input: &'a [u8],
    pos: usize,
}

impl Parser<'_> {
    fn error_at(&self, offset: usize, what: impl Into<String>) -> LiteralError {
        LiteralError {
            what: what.into(),
            offset,
        }
    }

    fn error(&self, what: impl Into<String>) -> LiteralError {
        self.error_at(self.pos, what)
    }

    fn peek(&self) -> Option<u8> {
        self.input.get(self.pos).copied()
    }

    fn at_end(&self) -> bool {
        self.pos >= self.input.len()
    }

    fn skip_whitespace(&mut self) {
        while matches!(self.peek(), Some(byte) if byte.is_ascii_whitespace()) {
            self.pos += 1;
        }
    }

    fn value(&mut self) -> Result<Value, LiteralError> {
        self.skip_whitespace();
        match self.peek() {
            None => Err(self.error("expected a literal")),
            Some(b'[') => {
                self.pos += 1;
                let mut items = Vec::new();
                self.elements(b']', &mut items)?;
                Ok(Value::Array(items))
            }
            Some(b'(') => self.paren(),
            Some(b'{') => self.brace(),
            Some(b'\'') | Some(b'"') => self.string().map(Value::String),
            Some(b'+') | Some(b'-') | Some(b'.') => self.number(),
            Some(byte) if byte.is_ascii_digit() => self.number(),
            Some(_) => self.constant(),
        }
    }

    /// `None` / `True` / `False`, plus the JSON spellings the previous reader
    /// accepted; every other bare name is a lookup, which is refused.
    fn constant(&mut self) -> Result<Value, LiteralError> {
        let start = self.pos;
        while matches!(self.peek(), Some(byte) if byte.is_ascii_alphanumeric() || byte == b'_') {
            self.pos += 1;
        }
        let name = String::from_utf8_lossy(&self.input[start..self.pos]).into_owned();
        match name.as_str() {
            "None" | "null" => Ok(Value::Null),
            "True" | "true" => Ok(Value::Bool(true)),
            "False" | "false" => Ok(Value::Bool(false)),
            _ => Err(self.error_at(start, format!("'{name}' is not a literal"))),
        }
    }

    fn number(&mut self) -> Result<Value, LiteralError> {
        let start = self.pos;
        let negative = match self.peek() {
            Some(b'+') => {
                self.pos += 1;
                false
            }
            Some(b'-') => {
                self.pos += 1;
                true
            }
            _ => false,
        };
        let body = self.pos;

        // `0x` / `0o` / `0b`: an integer in another base, never a float.
        let radix = match (self.peek(), self.input.get(self.pos + 1)) {
            (Some(b'0'), Some(b'x' | b'X')) => Some(16),
            (Some(b'0'), Some(b'o' | b'O')) => Some(8),
            (Some(b'0'), Some(b'b' | b'B')) => Some(2),
            _ => None,
        };
        if let Some(radix) = radix {
            self.pos += 2;
            self.digits(radix, "base prefix")?;
            let text = self.clean_text(body + 2, self.pos);
            let magnitude = u128::from_str_radix(&text, radix)
                .map_err(|_| self.error_at(start, "integer literal is too large"))?;
            return self.integer(start, magnitude, negative);
        }

        let mut counted = self.digits(10, "")?;
        let mut is_float = false;
        if self.peek() == Some(b'.') {
            self.pos += 1;
            counted += self.digits(10, "")?;
            if counted == 0 {
                return Err(self.error("expected digits in the number"));
            }
            is_float = true;
        }
        if counted == 0 {
            return Err(self.error("expected a number"));
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.pos += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            if self.digits(10, "exponent")? == 0 {
                return Err(self.error("expected digits in the exponent"));
            }
            is_float = true;
        }

        let text = self.clean_text(body, self.pos);
        if !is_float {
            let magnitude = text
                .parse::<u128>()
                .map_err(|_| self.error_at(start, "integer literal is too large"))?;
            return self.integer(start, magnitude, negative);
        }
        let magnitude = text
            .parse::<f64>()
            .map_err(|_| self.error_at(start, "malformed number"))?;
        Number::from_f64(if negative { -magnitude } else { magnitude })
            .map(Value::Number)
            .ok_or_else(|| self.error_at(start, "number out of range"))
    }

    /// The source text between two positions with the `_` separators dropped.
    fn clean_text(&self, from: usize, to: usize) -> String {
        self.input[from..to]
            .iter()
            .filter(|byte| **byte != b'_')
            .map(|byte| char::from(*byte))
            .collect()
    }

    /// Consume one run of `radix` digits (with `_` separators) and return how
    /// many digits it held. `what` names the position for the error when the
    /// run is empty and the caller requires one.
    fn digits(&mut self, radix: u32, what: &str) -> Result<usize, LiteralError> {
        let start = self.pos;
        let mut counted = 0;
        while let Some(byte) = self.peek() {
            if byte == b'_' {
                self.pos += 1;
            } else if char::from(byte).is_digit(radix) {
                self.pos += 1;
                counted += 1;
            } else {
                break;
            }
        }
        if counted == 0 && !what.is_empty() {
            return Err(self.error_at(start, format!("expected digits after the {what}")));
        }
        Ok(counted)
    }

    fn integer(
        &self,
        start: usize,
        magnitude: u128,
        negative: bool,
    ) -> Result<Value, LiteralError> {
        let number = if negative {
            if magnitude == i64::MAX as u128 + 1 {
                Number::from(i64::MIN)
            } else if magnitude <= i64::MAX as u128 {
                Number::from(-(magnitude as i64))
            } else {
                return Err(self.error_at(start, "integer literal is too large"));
            }
        } else if magnitude <= i64::MAX as u128 {
            Number::from(magnitude as i64)
        } else if magnitude <= u64::MAX as u128 {
            Number::from(magnitude as u64)
        } else {
            return Err(self.error_at(start, "integer literal is too large"));
        };
        Ok(Value::Number(number))
    }

    fn string(&mut self) -> Result<String, LiteralError> {
        let start = self.pos;
        let quote = self.input[self.pos];
        self.pos += 1;
        let mut out: Vec<u8> = Vec::new();
        loop {
            let Some(byte) = self.peek() else {
                return Err(self.error_at(start, "unterminated string literal"));
            };
            if byte == quote {
                self.pos += 1;
                break;
            }
            if byte == b'\n' {
                return Err(self.error_at(start, "unterminated string literal"));
            }
            if byte == b'\\' {
                self.pos += 1;
                self.escape(&mut out)?;
            } else {
                out.push(byte);
                self.pos += 1;
            }
        }
        String::from_utf8(out)
            .map_err(|_| self.error_at(start, "string literal is not valid UTF-8"))
    }

    /// Decode the escape whose backslash has been consumed into `out`. An
    /// unknown escape pushes the backslash back and leaves the character for
    /// the caller's loop to copy verbatim, as CPython does.
    fn escape(&mut self, out: &mut Vec<u8>) -> Result<(), LiteralError> {
        let Some(byte) = self.peek() else {
            return Err(self.error("unterminated string literal"));
        };
        match byte {
            b'\\' => {
                out.push(b'\\');
                self.pos += 1;
            }
            b'\'' => {
                out.push(b'\'');
                self.pos += 1;
            }
            b'"' => {
                out.push(b'"');
                self.pos += 1;
            }
            b'\n' => self.pos += 1,
            b'n' => {
                out.push(b'\n');
                self.pos += 1;
            }
            b't' => {
                out.push(b'\t');
                self.pos += 1;
            }
            b'r' => {
                out.push(b'\r');
                self.pos += 1;
            }
            b'a' => {
                out.push(0x07);
                self.pos += 1;
            }
            b'b' => {
                out.push(0x08);
                self.pos += 1;
            }
            b'f' => {
                out.push(0x0c);
                self.pos += 1;
            }
            b'v' => {
                out.push(0x0b);
                self.pos += 1;
            }
            b'x' => {
                self.pos += 1;
                out.push(self.hex_digits(2)? as u8);
            }
            b'u' => {
                self.pos += 1;
                self.unicode_escape(4, out)?;
            }
            b'U' => {
                self.pos += 1;
                self.unicode_escape(8, out)?;
            }
            b'0'..=b'7' => {
                let mut code = 0u32;
                for _ in 0..3 {
                    match self.peek() {
                        Some(digit @ b'0'..=b'7') => {
                            code = code * 8 + u32::from(digit - b'0');
                            self.pos += 1;
                        }
                        _ => break,
                    }
                }
                out.push(code as u8);
            }
            _ => out.push(b'\\'),
        }
        Ok(())
    }

    fn hex_digits(&mut self, count: usize) -> Result<u32, LiteralError> {
        let mut value = 0u32;
        for _ in 0..count {
            let digit = self
                .peek()
                .and_then(|byte| char::from(byte).to_digit(16))
                .ok_or_else(|| self.error("malformed escape sequence"))?;
            value = value * 16 + digit;
            self.pos += 1;
        }
        Ok(value)
    }

    fn unicode_escape(&mut self, count: usize, out: &mut Vec<u8>) -> Result<(), LiteralError> {
        let code = self.hex_digits(count)?;
        let Some(character) = char::from_u32(code) else {
            return Err(self.error("malformed escape sequence"));
        };
        let mut buffer = [0u8; 4];
        out.extend_from_slice(character.encode_utf8(&mut buffer).as_bytes());
        Ok(())
    }

    /// `(a)` is `a` itself and `(a,)` is a one-element tuple, as in Python;
    /// `()` is the empty tuple.
    fn paren(&mut self) -> Result<Value, LiteralError> {
        self.pos += 1;
        self.skip_whitespace();
        if self.peek() == Some(b')') {
            self.pos += 1;
            return Ok(Value::Array(Vec::new()));
        }
        let first = self.value()?;
        self.skip_whitespace();
        match self.peek() {
            Some(b',') => {
                self.pos += 1;
                let mut items = vec![first];
                self.elements(b')', &mut items)?;
                Ok(Value::Array(items))
            }
            Some(b')') => {
                self.pos += 1;
                Ok(first)
            }
            _ => Err(self.error("expected ',' or ')' in a tuple")),
        }
    }

    /// `{}` is the empty dict, `{k: v}` a mapping, and `{a, b}` a set — which
    /// JSON cannot carry and no config file or client writes.
    fn brace(&mut self) -> Result<Value, LiteralError> {
        self.pos += 1;
        self.skip_whitespace();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            return Ok(Value::Object(serde_json::Map::new()));
        }
        let key_at = self.pos;
        let key = self.value()?;
        self.skip_whitespace();
        match self.peek() {
            Some(b':') => self.mapping(key, key_at),
            Some(b',' | b'}') => Err(self.error("set literals are not supported")),
            _ => Err(self.error("expected ':' in a mapping")),
        }
    }

    /// The rest of a mapping whose first key is parsed and whose `:` is next;
    /// `key_at` is where that first key started, for a key-shape error.
    fn mapping(&mut self, first: Value, key_at: usize) -> Result<Value, LiteralError> {
        let mut entries = serde_json::Map::new();
        let mut key = first;
        let mut key_at = key_at;
        loop {
            self.pos += 1; // the ':' the caller left in place
            self.skip_whitespace();
            let value = self.value()?;
            entries.insert(self.key_name(&key, key_at)?, value);
            self.skip_whitespace();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                    self.skip_whitespace();
                    if self.peek() == Some(b'}') {
                        self.pos += 1;
                        return Ok(Value::Object(entries));
                    }
                    key_at = self.pos;
                    key = self.value()?;
                    self.skip_whitespace();
                    if self.peek() != Some(b':') {
                        return Err(self.error("expected ':' in a mapping"));
                    }
                }
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(Value::Object(entries));
                }
                _ => return Err(self.error("expected ',' or '}' in a mapping")),
            }
        }
    }

    /// A mapping key as the JSON object it has to become; see the module docs
    /// for why a number or a constant is stringified instead of refused.
    /// `at` is where the key started, which is where an unhashable key is
    /// reported.
    fn key_name(&self, key: &Value, at: usize) -> Result<String, LiteralError> {
        match key {
            Value::String(name) => Ok(name.clone()),
            Value::Bool(true) => Ok("True".to_string()),
            Value::Bool(false) => Ok("False".to_string()),
            Value::Null => Ok("None".to_string()),
            Value::Number(number) => {
                if let Some(integer) = number.as_i64() {
                    Ok(integer.to_string())
                } else if let Some(integer) = number.as_u64() {
                    Ok(integer.to_string())
                } else {
                    number
                        .as_f64()
                        .map(|real| format!("{real:?}"))
                        .ok_or_else(|| self.error_at(at, "a number key is out of range"))
                }
            }
            Value::Array(_) | Value::Object(_) => {
                Err(self.error_at(at, "a list or dict cannot be a mapping key"))
            }
        }
    }

    /// Fill `items` with the comma-separated values up to and including the
    /// `close` bracket, accepting the trailing comma Python allows.
    fn elements(&mut self, close: u8, items: &mut Vec<Value>) -> Result<(), LiteralError> {
        loop {
            self.skip_whitespace();
            if self.peek() == Some(close) {
                self.pos += 1;
                return Ok(());
            }
            items.push(self.value()?);
            self.skip_whitespace();
            match self.peek() {
                Some(b',') => self.pos += 1,
                Some(byte) if byte == close => {
                    self.pos += 1;
                    return Ok(());
                }
                _ => {
                    return Err(self.error(format!(
                        "expected ',' or '{}' in a sequence",
                        char::from(close)
                    )));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use serde_json::{json, Value};

    use super::parse;

    /// The four dict and constant literals Mainsail's own `mainsail.cfg`
    /// ships (`extras/mainsail.cfg:117-119`, `:199-200`), verbatim.
    #[test]
    fn the_mainsail_client_literals_parse() {
        assert_eq!(
            parse("{'restore': False, 'temp': 0}").unwrap(),
            json!({ "restore": false, "temp": 0 })
        );
        assert_eq!(parse("0").unwrap(), json!(0));
        assert_eq!(parse("False").unwrap(), json!(false));
        assert_eq!(
            parse("{ 'enable': False, 'call': \"PAUSE\" }").unwrap(),
            json!({ "enable": false, "call": "PAUSE" })
        );
        assert_eq!(
            parse("{ 'enable': False, 'layer': 0, 'call': \"PAUSE\" }").unwrap(),
            json!({ "enable": false, "layer": 0, "call": "PAUSE" })
        );
        // What the client sends at run time, after the template rendered the
        // body's `{{...}}`.
        assert_eq!(
            parse("{'call': 'PAUSE', 'enable': True, 'layer': 1}").unwrap(),
            json!({ "call": "PAUSE", "enable": true, "layer": 1 })
        );
    }

    /// Python's three constants and the JSON spellings the old reader took —
    /// one grammar, so neither is a fallback for the other.
    #[test]
    fn constants_parse_in_both_spellings() {
        assert_eq!(parse("None").unwrap(), Value::Null);
        assert_eq!(parse("True").unwrap(), Value::Bool(true));
        assert_eq!(parse("False").unwrap(), Value::Bool(false));
        assert_eq!(parse("null").unwrap(), Value::Null);
        assert_eq!(parse("true").unwrap(), Value::Bool(true));
        assert_eq!(parse("false").unwrap(), Value::Bool(false));
    }

    #[test]
    fn integers_parse_in_every_base_and_sign() {
        assert_eq!(parse("42").unwrap(), json!(42));
        assert_eq!(parse("0x1f").unwrap(), json!(31));
        assert_eq!(parse("0o17").unwrap(), json!(15));
        assert_eq!(parse("0b1011").unwrap(), json!(11));
        assert_eq!(parse("+7").unwrap(), json!(7));
        assert_eq!(parse("-7").unwrap(), json!(-7));
        assert_eq!(parse("-0x10").unwrap(), json!(-16));
        assert_eq!(parse("1_000").unwrap(), json!(1000));
        assert_eq!(parse("-9223372036854775808").unwrap(), json!(i64::MIN));
    }

    #[test]
    fn floats_parse_with_a_fraction_or_an_exponent() {
        assert_eq!(parse("1.5").unwrap(), json!(1.5));
        assert_eq!(parse(".5").unwrap(), json!(0.5));
        assert_eq!(parse("1.").unwrap(), json!(1.0));
        assert_eq!(parse("1e3").unwrap(), json!(1000.0));
        assert_eq!(parse("1.5E-3").unwrap(), json!(0.0015));
        assert_eq!(parse("-2.5").unwrap(), json!(-2.5));
    }

    #[test]
    fn strings_take_either_quote_and_pythons_escapes() {
        assert_eq!(parse("'PAUSE'").unwrap(), json!("PAUSE"));
        assert_eq!(parse("\"PAUSE\"").unwrap(), json!("PAUSE"));
        assert_eq!(parse(r#"'it\'s'"#).unwrap(), json!("it's"));
        assert_eq!(parse(r#""a\"b""#).unwrap(), json!("a\"b"));
        assert_eq!(parse(r"'a\\b'").unwrap(), json!("a\\b"));
        assert_eq!(parse(r"'a\nb\tc'").unwrap(), json!("a\nb\tc"));
        assert_eq!(parse(r"'a\rb\0c'").unwrap(), json!("a\rb\0c"));
        assert_eq!(parse(r"'\x41'").unwrap(), json!("A"));
        assert_eq!(parse(r"'\101'").unwrap(), json!("A"));
        assert_eq!(parse(r"'\u00e4'").unwrap(), json!("ä"));
        assert_eq!(parse(r"'\U0001F600'").unwrap(), json!("😀"));
        // An unknown escape keeps its backslash, as CPython does for `'\d'`.
        assert_eq!(parse(r"'\d'").unwrap(), json!("\\d"));
        // A backslash-newline continues the line inside the quotes.
        assert_eq!(parse("'a\\\nb'").unwrap(), json!("ab"));
    }

    #[test]
    fn sequences_parse_as_lists_and_tuples() {
        assert_eq!(parse("[]").unwrap(), json!([]));
        assert_eq!(parse("()").unwrap(), json!([]));
        assert_eq!(parse("[1, 2, 3]").unwrap(), json!([1, 2, 3]));
        assert_eq!(parse("[1, 2, 3,]").unwrap(), json!([1, 2, 3]));
        assert_eq!(parse("(1, 2, 3)").unwrap(), json!([1, 2, 3]));
        assert_eq!(parse("(1,)").unwrap(), json!([1]));
        // A one-element parenthesised value is the value, not a sequence.
        assert_eq!(parse("(1)").unwrap(), json!(1));
    }

    #[test]
    fn nested_literals_parse() {
        assert_eq!(
            parse("{'a': [1, (2, 3)], 'b': {'c': None}}").unwrap(),
            json!({ "a": [1, [2, 3]], "b": { "c": null } })
        );
    }

    #[test]
    fn an_empty_mapping_parses_as_an_object() {
        assert_eq!(parse("{}").unwrap(), json!({}));
        assert_eq!(parse("{'a': 1,}").unwrap(), json!({ "a": 1 }));
    }

    /// JSON object keys are strings, so a number or a constant key is
    /// stringified the way upstream's `json.dumps` does (`gcode_macro.py:158`).
    #[test]
    fn mapping_keys_are_stringified() {
        assert_eq!(
            parse("{1: 'a', 1.5: 'b', True: 'c', None: 'd'}").unwrap(),
            json!({ "1": "a", "1.5": "b", "True": "c", "None": "d" })
        );
        // A list is unhashable in Python, so it cannot be a key here either.
        let error = parse("{[1]: 'a'}").unwrap_err().to_string();
        assert_eq!(error, "a list or dict cannot be a mapping key at offset 1");
        let error = parse("{'a': 1, (2,): 'b'}").unwrap_err().to_string();
        assert_eq!(error, "a list or dict cannot be a mapping key at offset 9");
    }

    #[test]
    fn a_repeated_key_keeps_the_last_value() {
        assert_eq!(parse("{'a': 1, 'a': 2}").unwrap(), json!({ "a": 2 }));
    }

    #[test]
    fn whitespace_is_ignored_inside_and_around_the_literal() {
        assert_eq!(parse("  \n\t{'a':\n  1}  ").unwrap(), json!({ "a": 1 }));
        assert_eq!(parse("[\n1,\n2,\n]").unwrap(), json!([1, 2]));
    }

    /// A JSON document still parses: nothing the old reader accepted became
    /// an error.
    #[test]
    fn a_json_document_still_parses() {
        assert_eq!(
            parse(r#"{"a": [1, 2.5], "b": null, "c": true}"#).unwrap(),
            json!({ "a": [1, 2.5], "b": null, "c": true })
        );
        assert_eq!(parse(r#""a\nb""#).unwrap(), json!("a\nb"));
    }

    #[test]
    fn names_calls_and_arithmetic_are_refused() {
        assert_eq!(
            parse("[1, two]").unwrap_err().to_string(),
            "'two' is not a literal at offset 4"
        );
        assert_eq!(
            parse("PAUSE").unwrap_err().to_string(),
            "'PAUSE' is not a literal at offset 0"
        );
        assert_eq!(
            parse("1+2").unwrap_err().to_string(),
            "unexpected trailing characters at offset 1"
        );
        assert_eq!(
            parse("open('x')").unwrap_err().to_string(),
            "'open' is not a literal at offset 0"
        );
        assert_eq!(
            parse("-None").unwrap_err().to_string(),
            "expected a number at offset 1"
        );
    }

    #[test]
    fn a_set_literal_is_refused() {
        assert_eq!(
            parse("{1, 2}").unwrap_err().to_string(),
            "set literals are not supported at offset 2"
        );
        assert_eq!(
            parse("{1}").unwrap_err().to_string(),
            "set literals are not supported at offset 2"
        );
    }

    #[test]
    fn a_truncated_or_empty_literal_is_refused() {
        assert_eq!(
            parse("").unwrap_err().to_string(),
            "expected a literal at offset 0"
        );
        assert_eq!(
            parse("   ").unwrap_err().to_string(),
            "expected a literal at offset 3"
        );
        assert_eq!(
            parse("'oops").unwrap_err().to_string(),
            "unterminated string literal at offset 0"
        );
        assert_eq!(
            parse("[1, 2").unwrap_err().to_string(),
            "expected ',' or ']' in a sequence at offset 5"
        );
        assert_eq!(
            parse("{'a' 1}").unwrap_err().to_string(),
            "expected ':' in a mapping at offset 5"
        );
        assert_eq!(
            parse("0x").unwrap_err().to_string(),
            "expected digits after the base prefix at offset 2"
        );
    }
}
