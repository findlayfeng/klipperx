//! The MCU data dictionary — the firmware's description of its own protocol.
//!
//! The dictionary is the counterpart of the host-side handshake in
//! [`identify`](super::identify): the MCU sends it once, and from then on it is
//! the single source of truth for
//!
//! * which commands and responses exist ([`MessageDef`]),
//! * their wire ids and format strings (which the host never hard-codes),
//! * the enumerations used by parameters (`pin`, `static_string_id`, …), and
//! * the firmware's compile-time constants (`CLOCK_FREQ`, …).
//!
//! # Dictionary layout
//!
//! Every message table maps a format string to a wire id:
//!
//! ```json
//! {
//!   "commands":  {"get_clock": 5, "identify offset=%u count=%c": 1},
//!   "responses": {"clock clock=%u": 18},
//!   "output":    {"mpu9240 fifo_max=%u": 30},
//!   "enumerations": {"static_string_id": {"Timer too close": 3},
//!                    "pin": {"PL0": [0, 13]}},
//!   "config": {"CLOCK_FREQ": 20000000}
//! }
//! ```
//!
//! Note the asymmetry between the three tables:
//!
//! * `commands` / `responses` keys start with the message **name**
//!   (`"get_clock"`, `"clock clock=%u"`), so the name can be extracted.
//! * `output` keys are free-form, printf-like strings with **no** leading name
//!   — the firmware declares them as `_DECL_OUTPUT("mpu9240 fifo_max=%u")` —
//!   so they are kept verbatim and are *not* registered with the parser.
//!   Asynchronous output belongs to the event layer, which does not exist yet.
//!
//! # Ranges
//!
//! An enumeration value is either a single integer or an `[start, count]` pair.
//! Ranges are expanded at parse time exactly like Klipper's `fill_enumerations`
//! does: `"PL0": [0, 13]` becomes `PL0` → 0 … `PL12` → 12, using the trailing
//! digits of the declared name as the first index.

use super::error::McuError;
use super::identify::Identify;
use crate::core::klippy::msg::parser::Parser;
use serde_json::Value;
use std::collections::HashMap;

/// One command or response published by the firmware.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MessageDef {
    /// Message name — the first token of `format`.
    pub name: String,
    /// Wire id, as decoded from the firmware's signed VLQ.
    pub id: i16,
    /// Firmware-defined format string, e.g. `clock clock=%u`.
    pub format: String,
}

/// An asynchronous `output()` message.
///
/// The format string is free-form (it may contain arbitrary text followed by
/// `%` specifiers) and carries no message name, so it is kept verbatim.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutputDef {
    /// Firmware-defined output format, e.g. `mpu9240 fifo_max=%u`.
    pub format: String,
    /// Wire id.
    pub id: i16,
}

/// A flattened enumeration: names resolved to values, and back.
#[derive(Debug, Clone, Default)]
pub struct Enumeration {
    by_name: HashMap<String, i64>,
    by_value: HashMap<i64, String>,
}

impl Enumeration {
    /// Resolve an enumeration name (e.g. `"PL3"`) to its value.
    pub fn value(&self, name: &str) -> Option<i64> {
        self.by_name.get(name).copied()
    }

    /// Resolve a value back to its name.
    ///
    /// Returns `None` for values the firmware did not name; Klipper renders
    /// those as `?<value>` rather than failing, and callers that need the same
    /// behaviour must apply that fallback themselves.
    pub fn name(&self, value: i64) -> Option<&str> {
        self.by_value.get(&value).map(String::as_str)
    }

    /// Number of named values.
    pub fn len(&self) -> usize {
        self.by_name.len()
    }

    /// Whether this enumeration has no named values.
    pub fn is_empty(&self) -> bool {
        self.by_name.is_empty()
    }

    /// Iterate over all `(name, value)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (&str, i64)> + '_ {
        self.by_name
            .iter()
            .map(|(name, value)| (name.as_str(), *value))
    }

    /// Insert one declared entry, expanding `[start, count]` ranges.
    fn insert_spec(&mut self, name: &str, value: &Value) -> Result<(), McuError> {
        if let Some(single) = value.as_i64() {
            self.insert(name.to_string(), single);
            return Ok(());
        }

        if let Some(range) = value.as_array() {
            let start = range
                .first()
                .and_then(Value::as_i64)
                .ok_or_else(|| bad_enum(name, "range start must be an integer"))?;
            let count = range
                .get(1)
                .and_then(Value::as_i64)
                .ok_or_else(|| bad_enum(name, "range count must be an integer"))?;
            self.expand_range(name, start, count);
            return Ok(());
        }

        Err(bad_enum(name, "value must be an integer or [start, count]"))
    }

    /// Expand `"PL0": [0, 13]` into `PL0`…`PL12`.
    fn expand_range(&mut self, declared: &str, start: i64, count: i64) {
        let trailing_digits = declared
            .chars()
            .rev()
            .take_while(char::is_ascii_digit)
            .count();
        let split = declared.len() - trailing_digits;
        let (root, digits) = declared.split_at(split);
        let first_index: i64 = digits.parse().unwrap_or(0);

        for offset in 0..count {
            let name = format!("{}{}", root, first_index + offset);
            self.insert(name, start + offset);
        }
    }

    fn insert(&mut self, name: String, value: i64) {
        self.by_value.insert(value, name.clone());
        self.by_name.insert(name, value);
    }
}

/// The firmware's data dictionary, parsed and ready to use.
///
/// Keeps the raw JSON as well, so fields that are not modelled here (`version`,
/// `build_versions`, `app`, `license`, …) stay reachable.
#[derive(Debug, Clone, Default)]
pub struct Dictionary {
    raw: Value,
    commands: Vec<MessageDef>,
    responses: Vec<MessageDef>,
    output: Vec<OutputDef>,
    enumerations: HashMap<String, Enumeration>,
    constants: HashMap<String, Value>,
}

impl Dictionary {
    /// Parse the dictionary carried by an identify payload.
    pub fn parse(identify: &Identify) -> Result<Self, McuError> {
        Self::from_json(identify.data.clone())
    }

    /// Parse a decoded identify JSON body.
    ///
    /// # Errors
    /// Returns [`McuError::Dictionary`] if the body is not a JSON object, or if
    /// any message table, enumeration, or message id has an unexpected shape.
    pub fn from_json(data: Value) -> Result<Self, McuError> {
        if !data.is_object() {
            return Err(McuError::Dictionary(
                "identify payload is not a JSON object".to_string(),
            ));
        }

        let commands = parse_messages(data.get("commands"), "commands")?;
        let responses = parse_messages(data.get("responses"), "responses")?;
        let output = parse_output(data.get("output"))?;
        let enumerations = parse_enumerations(data.get("enumerations"))?;
        let constants = parse_constants(data.get("config"))?;

        Ok(Self {
            raw: data,
            commands,
            responses,
            output,
            enumerations,
            constants,
        })
    }

    /// Register every command and response with `parser`.
    ///
    /// Messages that are already registered are skipped. This matters because
    /// the two host-defined identify formats are repeated verbatim in the
    /// firmware's dictionary, so re-registering them would fail on a duplicate
    /// id and name.
    ///
    /// `output` entries are deliberately **not** installed: they are
    /// asynchronous event messages, which the parser cannot represent yet.
    ///
    /// Returns the number of messages newly registered.
    ///
    /// # Errors
    /// Returns [`McuError::Msg`] if a format string cannot be parsed, or if an
    /// id or name collides with a different, already-registered message.
    pub fn install(&self, parser: &mut Parser) -> Result<usize, McuError> {
        let mut installed = 0;
        for def in self.commands.iter().chain(self.responses.iter()) {
            if parser.is_registered(&def.name) {
                continue;
            }
            parser.register(def.id, &def.format)?;
            installed += 1;
        }
        Ok(installed)
    }

    /// The complete identify JSON body, verbatim.
    pub fn raw(&self) -> &Value {
        &self.raw
    }

    /// Commands the MCU accepts.
    pub fn commands(&self) -> &[MessageDef] {
        &self.commands
    }

    /// Responses the MCU sends after a command.
    pub fn responses(&self) -> &[MessageDef] {
        &self.responses
    }

    /// Asynchronous `output()` messages (not registered with the parser).
    pub fn output(&self) -> &[OutputDef] {
        &self.output
    }

    /// Every command and response, in declaration order.
    ///
    /// Commands come first, matching the order [`Dictionary::install`] uses.
    pub fn messages(&self) -> impl Iterator<Item = &MessageDef> {
        self.commands.iter().chain(self.responses.iter())
    }

    /// Look up a command or response by name.
    pub fn message(&self, name: &str) -> Option<&MessageDef> {
        self.messages().find(|def| def.name == name)
    }

    /// All enumerations, keyed by enumeration name (`pin`, `static_string_id`,
    /// …).
    pub fn enumerations(&self) -> &HashMap<String, Enumeration> {
        &self.enumerations
    }

    /// Look up one enumeration.
    pub fn enumeration(&self, name: &str) -> Option<&Enumeration> {
        self.enumerations.get(name)
    }

    /// Firmware compile-time constants (`CLOCK_FREQ`, `SERIAL_BAUD`, …).
    pub fn constants(&self) -> &HashMap<String, Value> {
        &self.constants
    }

    /// Look up one constant, as raw JSON.
    pub fn constant(&self, key: &str) -> Option<&Value> {
        self.constants.get(key)
    }

    /// Look up one numeric constant as `f64`.
    ///
    /// Integers are widened, so `"CLOCK_FREQ": 20000000` yields `2e7`.
    pub fn constant_f64(&self, key: &str) -> Option<f64> {
        self.constant(key)?.as_f64()
    }
}

/// Parse a `{format: id}` table into message definitions.
fn parse_messages(table: Option<&Value>, table_name: &str) -> Result<Vec<MessageDef>, McuError> {
    let Some(table) = table else {
        return Ok(Vec::new());
    };
    let object = table
        .as_object()
        .ok_or_else(|| McuError::Dictionary(format!("'{table_name}' must be an object")))?;

    object
        .iter()
        .map(|(format, id)| {
            let name = format
                .split_whitespace()
                .next()
                .filter(|name| !name.is_empty())
                .ok_or_else(|| {
                    McuError::Dictionary(format!("'{table_name}' has an empty format string"))
                })?;
            Ok(MessageDef {
                name: name.to_string(),
                id: parse_id(id, table_name, format)?,
                format: format.clone(),
            })
        })
        .collect()
}

/// Parse the `output` table, keeping the format strings verbatim.
fn parse_output(table: Option<&Value>) -> Result<Vec<OutputDef>, McuError> {
    let Some(table) = table else {
        return Ok(Vec::new());
    };
    let object = table
        .as_object()
        .ok_or_else(|| McuError::Dictionary("'output' must be an object".to_string()))?;

    object
        .iter()
        .map(|(format, id)| {
            Ok(OutputDef {
                format: format.clone(),
                id: parse_id(id, "output", format)?,
            })
        })
        .collect()
}

/// Parse the `enumerations` table, expanding ranges.
fn parse_enumerations(table: Option<&Value>) -> Result<HashMap<String, Enumeration>, McuError> {
    let Some(table) = table else {
        return Ok(HashMap::new());
    };
    let object = table
        .as_object()
        .ok_or_else(|| McuError::Dictionary("'enumerations' must be an object".to_string()))?;

    let mut out = HashMap::with_capacity(object.len());
    for (enum_name, entries) in object {
        let entries = entries.as_object().ok_or_else(|| {
            McuError::Dictionary(format!("enumeration '{enum_name}' must be an object"))
        })?;

        let mut enumeration = Enumeration::default();
        for (name, value) in entries {
            enumeration.insert_spec(name, value)?;
        }
        out.insert(enum_name.clone(), enumeration);
    }
    Ok(out)
}

/// Parse the `config` table of compile-time constants.
fn parse_constants(table: Option<&Value>) -> Result<HashMap<String, Value>, McuError> {
    let Some(table) = table else {
        return Ok(HashMap::new());
    };
    let object = table
        .as_object()
        .ok_or_else(|| McuError::Dictionary("'config' must be an object".to_string()))?;

    Ok(object
        .iter()
        .map(|(key, value)| (key.clone(), value.clone()))
        .collect())
}

/// Parse a wire id, which must fit the signed 16-bit range used by [`Msg`].
///
/// [`Msg`]: crate::core::klippy::msg::Msg
fn parse_id(value: &Value, table_name: &str, format: &str) -> Result<i16, McuError> {
    let id = value.as_i64().ok_or_else(|| {
        McuError::Dictionary(format!(
            "'{table_name}' id for '{format}' must be an integer"
        ))
    })?;
    i16::try_from(id).map_err(|_| {
        McuError::Dictionary(format!(
            "'{table_name}' id {id} for '{format}' is out of range"
        ))
    })
}

fn bad_enum(name: &str, reason: &str) -> McuError {
    McuError::Dictionary(format!("enumeration entry '{name}': {reason}"))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::identify::IDENTIFY_MESSAGES;
    use crate::core::klippy::msg::proto::{ArgValue, Payload};

    /// A dictionary with the same shape as a real firmware's, trimmed down.
    const DICT_JSON: &str = r#"{
        "app": "Klipper",
        "version": "v0.12.0-1-g1234567",
        "build_versions": "gcc: 12.3.1",
        "license": "GNU GPLv3",
        "commands": {
            "identify offset=%u count=%c": 1,
            "get_clock": 5,
            "get_uptime": 4,
            "get_config": 7,
            "clear_shutdown": 2
        },
        "responses": {
            "identify_response offset=%u data=%.*s": 0,
            "clock clock=%u": 18,
            "uptime high=%u clock=%u": 17,
            "shutdown clock=%u static_string_id=%hu": 15,
            "is_shutdown static_string_id=%hu": 14
        },
        "output": {
            "mpu9240 fifo_max=%u": 30
        },
        "enumerations": {
            "static_string_id": {
                "Timer too close": 3,
                "Command parser error": 7
            },
            "pin": {
                "PL0": [0, 13],
                "PB0": [32, 10],
                "LED": 99
            }
        },
        "config": {
            "CLOCK_FREQ": 20000000,
            "SERIAL_BAUD": 250000
        }
    }"#;

    fn dict() -> Dictionary {
        Dictionary::from_json(serde_json::from_str(DICT_JSON).unwrap()).unwrap()
    }

    fn identify_of(json: &str) -> Identify {
        Identify {
            data: serde_json::from_str(json).unwrap(),
        }
    }

    // -----------------------------------------------------------------------
    // Parsing
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_messages() {
        let dict = dict();

        assert_eq!(dict.commands().len(), 5);
        assert_eq!(dict.responses().len(), 5);
        assert_eq!(dict.output().len(), 1);
        assert_eq!(dict.messages().count(), 10);

        let get_clock = dict.message("get_clock").unwrap();
        assert_eq!(get_clock.id, 5);
        assert_eq!(get_clock.format, "get_clock");
        let clock = dict.message("clock").unwrap();
        assert_eq!(clock.id, 18);
        assert_eq!(clock.format, "clock clock=%u");
        // The identify pair is part of the firmware dictionary.
        assert_eq!(dict.message("identify").unwrap().id, 1);
        assert_eq!(dict.message("identify_response").unwrap().id, 0);

        assert!(dict.message("nonexistent").is_none());
    }

    #[test]
    fn test_parse_keeps_output_formats_verbatim() {
        let dict = dict();
        let output = &dict.output()[0];
        // Free-form: the first token is *not* a message name.
        assert_eq!(output.format, "mpu9240 fifo_max=%u");
        assert_eq!(output.id, 30);
    }

    #[test]
    fn test_parse_from_identify() {
        let dict = Dictionary::parse(&identify_of(DICT_JSON)).unwrap();
        assert_eq!(dict.message("get_clock").unwrap().id, 5);
        assert_eq!(dict.raw()["app"], "Klipper");
    }

    #[test]
    fn test_parse_constants() {
        let dict = dict();
        assert_eq!(dict.constant_f64("CLOCK_FREQ"), Some(20_000_000.0));
        assert_eq!(dict.constant_f64("SERIAL_BAUD"), Some(250_000.0));
        assert_eq!(dict.constant_f64("missing"), None);
        assert_eq!(
            dict.constant("CLOCK_FREQ").unwrap().as_i64(),
            Some(20_000_000)
        );
        assert_eq!(dict.constants().len(), 2);
    }

    #[test]
    fn test_parse_tolerates_absent_optional_tables() {
        let dict = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5}
        }))
        .unwrap();

        assert_eq!(dict.commands().len(), 1);
        assert!(dict.responses().is_empty());
        assert!(dict.output().is_empty());
        assert!(dict.enumerations().is_empty());
        assert!(dict.constants().is_empty());
    }

    #[test]
    fn test_parse_rejects_non_object() {
        let err = Dictionary::from_json(serde_json::json!([1, 2, 3])).unwrap_err();
        assert!(matches!(err, McuError::Dictionary(_)));
        assert!(err.to_string().contains("not a JSON object"));
    }

    #[test]
    fn test_parse_rejects_malformed_message_table() {
        let err = Dictionary::from_json(serde_json::json!({"commands": 5})).unwrap_err();
        assert!(err.to_string().contains("'commands' must be an object"));
    }

    #[test]
    fn test_parse_rejects_non_integer_id() {
        let err = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": "five"}
        }))
        .unwrap_err();
        assert!(err.to_string().contains("must be an integer"));
    }

    #[test]
    fn test_parse_rejects_out_of_range_id() {
        let err = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 100000}
        }))
        .unwrap_err();
        assert!(err.to_string().contains("out of range"));
    }

    #[test]
    fn test_parse_rejects_empty_format_string() {
        let err = Dictionary::from_json(serde_json::json!({
            "commands": {"   ": 5}
        }))
        .unwrap_err();
        assert!(err.to_string().contains("empty format string"));
    }

    // -----------------------------------------------------------------------
    // Enumerations
    // -----------------------------------------------------------------------

    #[test]
    fn test_enumerations_single_values() {
        let dict = dict();
        let strings = dict.enumeration("static_string_id").unwrap();

        assert_eq!(strings.value("Timer too close"), Some(3));
        assert_eq!(strings.value("Command parser error"), Some(7));
        assert_eq!(strings.name(3), Some("Timer too close"));
        assert_eq!(strings.name(7), Some("Command parser error"));
        assert_eq!(strings.value("unknown"), None);
        assert_eq!(strings.name(12345), None);
        assert_eq!(strings.len(), 2);
    }

    #[test]
    fn test_enumerations_expand_ranges() {
        let dict = dict();
        let pins = dict.enumeration("pin").unwrap();

        // "PL0": [0, 13] -> PL0..PL12 with values 0..12
        assert_eq!(pins.value("PL0"), Some(0));
        assert_eq!(pins.value("PL3"), Some(3));
        assert_eq!(pins.value("PL12"), Some(12));
        assert_eq!(pins.name(12), Some("PL12"));
        assert_eq!(pins.value("PL13"), None);

        // "PB0": [32, 10] -> PB0..PB9 with values 32..41
        assert_eq!(pins.value("PB0"), Some(32));
        assert_eq!(pins.value("PB9"), Some(41));
        assert_eq!(pins.name(41), Some("PB9"));

        // Single values live in the same enumeration.
        assert_eq!(pins.value("LED"), Some(99));
        assert_eq!(pins.len(), 13 + 10 + 1);
    }

    #[test]
    fn test_enumerations_reject_bad_spec() {
        let err = Dictionary::from_json(serde_json::json!({
            "enumerations": {"pin": {"PA0": "zero"}}
        }))
        .unwrap_err();
        assert!(err
            .to_string()
            .contains("must be an integer or [start, count]"));

        let err = Dictionary::from_json(serde_json::json!({
            "enumerations": {"pin": {"PA0": [0]}}
        }))
        .unwrap_err();
        assert!(err.to_string().contains("range count must be an integer"));
    }

    #[test]
    fn test_enumeration_iter() {
        let dict = Dictionary::from_json(serde_json::json!({
            "enumerations": {"e": {"A": 1, "B": 2}}
        }))
        .unwrap();
        let mut pairs: Vec<(String, i64)> = dict
            .enumeration("e")
            .unwrap()
            .iter()
            .map(|(name, value)| (name.to_string(), value))
            .collect();
        pairs.sort();
        assert_eq!(pairs, vec![("A".to_string(), 1), ("B".to_string(), 2)]);
    }

    // -----------------------------------------------------------------------
    // Installation
    // -----------------------------------------------------------------------

    #[test]
    fn test_install_registers_commands_and_responses() {
        let dict = dict();
        let mut parser = Parser::new();

        let installed = dict.install(&mut parser).unwrap();

        // 5 commands + 5 responses; the identify pair is included because a
        // fresh parser has nothing registered yet.
        assert_eq!(installed, 10);
        assert!(parser.is_registered("get_clock"));
        assert!(parser.is_registered("clock"));
        assert!(parser.is_registered("get_uptime"));
    }

    #[test]
    fn test_install_skips_already_registered_identify_messages() {
        let dict = dict();
        let mut parser = Parser::new();
        parser.register_all(IDENTIFY_MESSAGES).unwrap();

        // Must not fail on the duplicate id/name of the identify pair.
        assert_eq!(dict.install(&mut parser).unwrap(), 8);
        assert!(parser.is_registered("identify"));
        assert!(parser.is_registered("identify_response"));
    }

    #[test]
    fn test_install_does_not_register_output() {
        let dict = dict();
        let mut parser = Parser::new();
        dict.install(&mut parser).unwrap();

        // Output formats have no message name and are left to the event layer.
        let result = parser.encode("mpu9240", &[ArgValue::UInt32(1)]);
        assert!(result.is_err());
    }

    #[test]
    fn test_install_makes_firmware_formats_encodable() {
        let dict = dict();
        let mut parser = Parser::new();
        dict.install(&mut parser).unwrap();

        // `get_clock` takes no parameters and encodes to just its id (5).
        let payload = parser.encode("get_clock", &[]).unwrap();
        assert_eq!(payload.payload(), &[5]);

        // `get_uptime high=%u clock=%u` decodes using the firmware's format.
        let mut response = Payload::new();
        response.push_i16(17).unwrap();
        response.push_u32(1).unwrap();
        response.push_u32(0x1234).unwrap();

        let decoded = parser
            .decode(Payload::from_raw(response.into_raw()))
            .unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "uptime");
        assert_eq!(decoded[0].1[0], ArgValue::UInt32(1));
        assert_eq!(decoded[0].1[1], ArgValue::UInt32(0x1234));
    }

    #[test]
    fn test_install_reports_format_errors() {
        let dict = Dictionary::from_json(serde_json::json!({
            "commands": {"broken": 42}
        }))
        .unwrap();
        let mut parser = Parser::new();

        // "broken" is a valid single-token format with no params, so it
        // installs; a genuinely malformed parameter list must fail instead.
        assert!(dict.install(&mut parser).is_ok());

        let dict = Dictionary::from_json(serde_json::json!({
            "commands": {"broken x=%q": 42}
        }))
        .unwrap();
        let mut parser = Parser::new();
        let err = dict.install(&mut parser).unwrap_err();
        assert!(matches!(err, McuError::Msg(_)));
    }
}
