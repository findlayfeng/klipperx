//! Typed views over the firmware-defined protocol.
//!
//! The host does not own any message format: the dictionary installed after the
//! identify handshake does. This module adds the thin typing layer on top of
//! [`Mcu::send`](super::Mcu::send) / [`Mcu::call`](super::Mcu::call) so command
//! code can name a message and its parameters without repeating the wire
//! format.
//!
//! Two traits describe the two directions:
//!
//! * [`McuCommand`] — an outbound message: a name plus an argument list.
//! * [`McuResponse`] — an inbound message: a name plus a decoder that reads
//!   parameters *by name* through [`Params`].
//!
//! Reading parameters by name is the point of [`Params`]: the parameter order
//! is chosen by the firmware, so positional indexing would silently break
//! whenever a firmware revision reorders or inserts a parameter. The declared
//! types are still validated — [`Params`] converts each value to the requested
//! type and reports a mismatch instead of truncating.

use super::dictionary::{Dictionary, Enumeration};
use super::error::McuError;
use super::Mcu;
use crate::core::klippy::msg::proto::{ArgType, ArgValue};
use crate::core::klippy::msg::Msg;
use std::sync::Arc;
use tokio::time::Duration;

/// An outbound MCU message.
///
/// Implementors only declare the name published by the firmware and how to
/// build the argument list; the wire id and format string come from the
/// dictionary.
pub trait McuCommand {
    /// Command name, which must match a `commands` entry of the dictionary.
    const NAME: &'static str;

    /// Arguments in the dictionary's declaration order.
    fn args(&self) -> Vec<ArgValue>;
}

/// An inbound MCU message.
pub trait McuResponse: Sized {
    /// Response name, which must match a `responses` entry of the dictionary.
    const NAME: &'static str;

    /// Build the typed response from the decoded parameters.
    ///
    /// # Errors
    /// Returns [`McuError::Decode`] when a parameter is missing or holds an
    /// unexpected type.
    fn decode(params: &Params<'_>) -> Result<Self, McuError>;
}

/// A read-only view over the parameters of a decoded message.
///
/// Parameter names and declared types come from the message definition, which
/// in turn comes from the firmware dictionary.
pub struct Params<'a> {
    msg: Arc<Msg>,
    values: &'a [ArgValue],
    /// Attached by [`Mcu::call_msg`] so [`Params::get_enum`] can resolve names.
    dictionary: Option<Arc<Dictionary>>,
}

impl<'a> Params<'a> {
    /// Create a view over `values`, which must follow `msg`'s parameter order.
    pub fn new(msg: Arc<Msg>, values: &'a [ArgValue]) -> Self {
        Self {
            msg,
            values,
            dictionary: None,
        }
    }

    /// Attach a dictionary to enable enumeration lookups.
    pub fn with_dictionary(mut self, dictionary: Arc<Dictionary>) -> Self {
        self.dictionary = Some(dictionary);
        self
    }

    /// Name of the message these parameters belong to.
    pub fn message_name(&self) -> &str {
        &self.msg.name
    }

    /// Number of parameters the message declares.
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Whether the message declares no parameters.
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Names and declared types, in declaration order.
    pub fn declared(&self) -> &[(String, ArgType)] {
        &self.msg.params
    }

    /// Raw value of the parameter called `name`.
    pub fn value(&self, name: &str) -> Option<&ArgValue> {
        let index = self
            .msg
            .params
            .iter()
            .position(|(param, _)| param == name)?;
        self.values.get(index)
    }

    /// Whether the message declares a parameter called `name`.
    pub fn has(&self, name: &str) -> bool {
        self.msg.params.iter().any(|(param, _)| param == name)
    }

    /// Value of `name` converted to `target`.
    ///
    /// # Errors
    /// Returns [`McuError::Decode`] if the parameter is not declared, or if its
    /// value cannot be represented as `target`.
    pub fn get(&self, name: &str, target: ArgType) -> Result<ArgValue, McuError> {
        if !self.has(name) {
            return Err(self.undeclared(name));
        }
        let value = self.value(name).expect("declared parameter has a value");
        value.try_convert_to(target).map_err(|_| {
            McuError::Decode(format!(
                "'{}' parameter '{}' is {} but {} was expected",
                self.msg.name,
                name,
                value.arg_type().format_str(),
                target.format_str()
            ))
        })
    }

    /// Value of `name` as `u32`.
    pub fn get_u32(&self, name: &str) -> Result<u32, McuError> {
        match self.get(name, ArgType::UInt32)? {
            ArgValue::UInt32(v) => Ok(v),
            other => Err(unexpected(name, &other, ArgType::UInt32)),
        }
    }

    /// Value of `name` as `u8`.
    pub fn get_u8(&self, name: &str) -> Result<u8, McuError> {
        match self.get(name, ArgType::UInt8)? {
            ArgValue::UInt8(v) => Ok(v),
            other => Err(unexpected(name, &other, ArgType::UInt8)),
        }
    }

    /// Value of `name` as `u16`.
    pub fn get_u16(&self, name: &str) -> Result<u16, McuError> {
        match self.get(name, ArgType::UInt16)? {
            ArgValue::UInt16(v) => Ok(v),
            other => Err(unexpected(name, &other, ArgType::UInt16)),
        }
    }

    /// Value of `name` as `i16`.
    pub fn get_i16(&self, name: &str) -> Result<i16, McuError> {
        match self.get(name, ArgType::Int16)? {
            ArgValue::Int16(v) => Ok(v),
            other => Err(unexpected(name, &other, ArgType::Int16)),
        }
    }

    /// Value of `name` as `i32`.
    pub fn get_i32(&self, name: &str) -> Result<i32, McuError> {
        match self.get(name, ArgType::Int32)? {
            ArgValue::Int32(v) => Ok(v),
            other => Err(unexpected(name, &other, ArgType::Int32)),
        }
    }

    /// Value of `name` as a borrowed byte buffer.
    ///
    /// Works for both `%s` and `%.*s`/`%*s`, which share one wire encoding.
    pub fn get_bytes(&self, name: &str) -> Result<Vec<u8>, McuError> {
        match self.get(name, ArgType::Bytes)? {
            ArgValue::Bytes(v) => Ok(v),
            other => Err(unexpected(name, &other, ArgType::Bytes)),
        }
    }

    /// Value of `name` as a UTF-8 string.
    ///
    /// `%s` and `%.*s` share one wire encoding, so either is accepted. Bytes
    /// that are not valid UTF-8 get a specific error rather than a generic type
    /// mismatch.
    ///
    /// # Errors
    /// Returns [`McuError::Decode`] if the parameter is not declared, is not a
    /// string/buffer, or holds invalid UTF-8.
    pub fn get_str(&self, name: &str) -> Result<String, McuError> {
        if !self.has(name) {
            return Err(self.undeclared(name));
        }
        match self.value(name).expect("declared parameter has a value") {
            ArgValue::Str(text) => Ok(text.clone()),
            ArgValue::Bytes(bytes) => String::from_utf8(bytes.clone()).map_err(|_| {
                McuError::Decode(format!(
                    "'{}' parameter '{}' is not valid UTF-8",
                    self.msg.name, name
                ))
            }),
            other => Err(unexpected(name, other, ArgType::Str)),
        }
    }

    /// Resolve an enumerated parameter to the name the firmware gave it.
    ///
    /// `enumeration` is the enumeration to consult. Klipper infers it from the
    /// parameter name (`is_shutdown static_string_id=%hu` uses the
    /// `static_string_id` enumeration for its `static_string_id` parameter), so
    /// the two arguments are usually identical; they are separate here to keep
    /// the mapping explicit.
    ///
    /// Values the firmware did not name render as `?<value>`, matching Klipper,
    /// so an unrecognised value never fails a decode.
    ///
    /// # Errors
    /// Returns [`McuError::Decode`] if the parameter is not an integer, or if no
    /// dictionary is attached / the enumeration is unknown.
    pub fn get_enum(&self, enumeration: &str, name: &str) -> Result<String, McuError> {
        // Enumeration parameters are wire-encoded as plain integers, declared
        // either signed or unsigned. Preserve the real error when neither view
        // works, instead of reporting a generic type mismatch.
        let number = match self.get(name, ArgType::UInt32) {
            Ok(value) => as_i64(&value).expect("UInt32 converts to i64"),
            Err(first_error) => match self.get(name, ArgType::Int32) {
                Ok(value) => as_i64(&value).expect("Int32 converts to i64"),
                Err(_) => return Err(first_error),
            },
        };

        let dictionary = self.dictionary.as_ref().ok_or_else(|| {
            McuError::Decode(format!(
                "cannot resolve enumeration '{}' without a dictionary",
                enumeration
            ))
        })?;
        let entries: &Enumeration = dictionary.enumeration(enumeration).ok_or_else(|| {
            McuError::Decode(format!(
                "MCU dictionary has no enumeration '{}'",
                enumeration
            ))
        })?;

        Ok(match entries.name(number) {
            Some(name) => name.to_string(),
            None => format!("?{}", number),
        })
    }

    fn declared_names(&self) -> String {
        self.msg
            .params
            .iter()
            .map(|(name, atype)| format!("{}={}", name, atype.format_str()))
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// Error for a parameter the message does not declare.
    fn undeclared(&self, name: &str) -> McuError {
        McuError::Decode(format!(
            "'{}' has no parameter '{}' (declared: {})",
            self.msg.name,
            name,
            self.declared_names()
        ))
    }
}

impl Mcu {
    /// Send a typed command, without waiting for a response.
    ///
    /// # Errors
    /// Returns [`McuError::NotIdentified`] before the identify handshake,
    /// [`McuError::UnknownMessage`] when the name is absent from the
    /// dictionary, [`McuError::Msg`] when the arguments do not match the
    /// firmware's format string, and [`McuError::Call`] when the outbound buffer
    /// is full.
    pub fn send_msg<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        self.require_dictionary()?;
        self.require_message(C::NAME)?;
        self.send(C::NAME, &cmd.args())?;
        Ok(())
    }

    /// Send a typed command and decode its typed response.
    ///
    /// Both names are resolved **before** the command is sent, so a message the
    /// firmware does not implement fails immediately instead of waiting for
    /// `timeout`.
    ///
    /// # Errors
    /// Returns [`McuError::NotIdentified`] before the identify handshake,
    /// [`McuError::UnknownMessage`] when either name is absent from the
    /// dictionary, [`McuError::Msg`] when the arguments do not match the
    /// firmware's format string, [`McuError::Call`] when the exchange fails or
    /// times out, and [`McuError::Decode`] when the response cannot be decoded.
    pub async fn call_msg<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        timeout: Duration,
    ) -> Result<R, McuError> {
        let dictionary = self.require_dictionary()?;
        self.call_typed::<C, R>(cmd, timeout, Some(dictionary))
            .await
    }

    /// Typed call that also works **before** the identify handshake.
    ///
    /// The identify exchange is the only one that precedes the dictionary, so
    /// this exists for
    /// [`Identify::fetch`](super::identify::Identify::fetch) alone; every other
    /// caller must use [`Mcu::call_msg`] so that a missing handshake is reported
    /// instead of silently attempting a command the parser does not know yet.
    pub(crate) async fn call_msg_ungated<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        timeout: Duration,
    ) -> Result<R, McuError> {
        self.call_typed::<C, R>(cmd, timeout, None).await
    }

    /// Shared body of the typed calls.
    ///
    /// Both names are resolved before the command is sent, so a message the
    /// firmware does not implement fails immediately instead of waiting for
    /// `timeout`.
    async fn call_typed<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        timeout: Duration,
        dictionary: Option<Arc<Dictionary>>,
    ) -> Result<R, McuError> {
        self.require_message(C::NAME)?;
        let msg = self.require_message(R::NAME)?;

        let values = self.call(C::NAME, &cmd.args(), R::NAME, timeout).await?;

        let params = match dictionary {
            Some(dictionary) => Params::new(msg, &values).with_dictionary(dictionary),
            None => Params::new(msg, &values),
        };
        R::decode(&params)
    }

    /// Look up a message, failing fast when the dictionary does not define it.
    fn require_message(&self, name: &str) -> Result<Arc<Msg>, McuError> {
        self.parser
            .lookup(name)
            .ok_or_else(|| McuError::UnknownMessage(name.to_string()))
    }
}

/// Interpret an integer [`ArgValue`] as `i64`.
fn as_i64(value: &ArgValue) -> Option<i64> {
    match value {
        ArgValue::UInt8(v) => Some(i64::from(*v)),
        ArgValue::UInt16(v) => Some(i64::from(*v)),
        ArgValue::Int16(v) => Some(i64::from(*v)),
        ArgValue::UInt32(v) => Some(i64::from(*v)),
        ArgValue::Int32(v) => Some(i64::from(*v)),
        ArgValue::Str(_) | ArgValue::Bytes(_) => None,
    }
}

fn unexpected(name: &str, value: &ArgValue, target: ArgType) -> McuError {
    decode_mismatch(name, value.arg_type().format_str(), target.format_str())
}
fn decode_mismatch(parameter: &str, actual: &str, expected: &str) -> McuError {
    McuError::Decode(format!(
        "parameter '{}' is {} but {} was expected",
        parameter, actual, expected
    ))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::McuCallError;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::Payload;
    use serde_json::json;

    // -----------------------------------------------------------------------
    // Test dictionary and messages
    // -----------------------------------------------------------------------

    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "get_clock": 5,
                "get_uptime": 4,
                "clear_shutdown": 2
            },
            "responses": {
                "clock clock=%u": 18,
                "uptime high=%u clock=%u": 17,
                "is_shutdown static_string_id=%hu": 14
            },
            "enumerations": {
                "static_string_id": {"Timer too close": 3}
            },
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
    }

    /// `get_clock` — takes no arguments.
    struct GetClock;

    impl McuCommand for GetClock {
        const NAME: &'static str = "get_clock";
        fn args(&self) -> Vec<ArgValue> {
            Vec::new()
        }
    }

    /// `clock clock=%u`
    #[derive(Debug, PartialEq)]
    struct ClockState {
        clock: u32,
    }

    impl McuResponse for ClockState {
        const NAME: &'static str = "clock";
        fn decode(params: &Params<'_>) -> Result<Self, McuError> {
            Ok(Self {
                clock: params.get_u32("clock")?,
            })
        }
    }

    /// `get_uptime` / `uptime high=%u clock=%u`
    struct GetUptime;

    impl McuCommand for GetUptime {
        const NAME: &'static str = "get_uptime";
        fn args(&self) -> Vec<ArgValue> {
            Vec::new()
        }
    }

    #[derive(Debug, PartialEq)]
    struct Uptime {
        high: u32,
        clock: u32,
    }

    impl McuResponse for Uptime {
        const NAME: &'static str = "uptime";
        fn decode(params: &Params<'_>) -> Result<Self, McuError> {
            Ok(Self {
                high: params.get_u32("high")?,
                clock: params.get_u32("clock")?,
            })
        }
    }

    /// `is_shutdown static_string_id=%hu`
    #[derive(Debug)]
    struct IsShutdown;

    impl McuResponse for IsShutdown {
        const NAME: &'static str = "is_shutdown";
        fn decode(_params: &Params<'_>) -> Result<Self, McuError> {
            Ok(Self)
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// Payload of a message built from its firmware member order.
    fn payload(parts: &[ArgValue]) -> Vec<u8> {
        let mut out = Payload::new();
        for value in parts {
            out.push_value(value).unwrap();
        }
        out.into_raw()
    }

    fn mcu_with(dictionary: Dictionary, mappings: Vec<MappingEntry>) -> Mcu {
        let mcu = Mcu::from((
            "test_mcu".to_string(),
            Interface::new(TestDevice::new(mappings)),
        ));
        // A fresh `Mcu` registers the identify pair, so the dictionary entries
        // install cleanly on top.
        mcu.install_dictionary(dictionary).unwrap();
        mcu
    }

    // -----------------------------------------------------------------------
    // Params
    // -----------------------------------------------------------------------

    fn params_for(format: &str, id: i16, values: &[ArgValue]) -> (Parser, Vec<ArgValue>) {
        let mut parser = Parser::new();
        parser.register(id, format).unwrap();
        let encoded = parser
            .encode(format.split_whitespace().next().unwrap(), values)
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        (parser, decoded[0].1.clone())
    }

    #[test]
    fn test_params_read_by_name_not_position() {
        // The firmware declares `clock` first; the host asks by name.
        let (parser, values) = params_for(
            "state clock=%u high=%u",
            17,
            &[ArgValue::UInt32(7), ArgValue::UInt32(9)],
        );
        let params = Params::new(parser.lookup("state").unwrap(), &values);

        assert_eq!(params.message_name(), "state");
        assert_eq!(params.len(), 2);
        assert!(!params.is_empty());
        assert_eq!(params.get_u32("clock").unwrap(), 7);
        assert_eq!(params.get_u32("high").unwrap(), 9);
        assert!(params.has("clock"));
        assert!(!params.has("missing"));
    }

    #[test]
    fn test_params_convert_widening_and_range_checked() {
        let (parser, values) = params_for(
            "state a=%c b=%hi",
            1,
            &[ArgValue::UInt8(5), ArgValue::Int16(-2)],
        );
        let params = Params::new(parser.lookup("state").unwrap(), &values);

        // Widening is allowed.
        assert_eq!(params.get_u32("a").unwrap(), 5);
        assert_eq!(params.get_i32("b").unwrap(), -2);
        // Narrowing a negative value is refused, not wrapped.
        let err = params.get_u8("b").unwrap_err();
        assert!(matches!(err, McuError::Decode(_)));
    }

    #[test]
    fn test_params_undeclared_parameter_reports_declared_names() {
        let (parser, values) = params_for("state clock=%u", 1, &[ArgValue::UInt32(1)]);
        let params = Params::new(parser.lookup("state").unwrap(), &values);

        let err = params.get_u32("nope").unwrap_err();
        let text = err.to_string();
        assert!(text.contains("has no parameter 'nope'"), "{text}");
        assert!(text.contains("clock=%u"), "{text}");
    }

    #[test]
    fn test_params_mistyped_parameter_reports_both_types() {
        let (parser, values) = params_for("state name=%s", 1, &[ArgValue::Str("x".into())]);
        let params = Params::new(parser.lookup("state").unwrap(), &values);

        let err = params.get_u32("name").unwrap_err();
        let text = err.to_string();
        assert!(text.contains("is %s but %u was expected"), "{text}");
    }

    #[test]
    fn test_params_get_bytes_and_str_share_encoding() {
        let (parser, values) = params_for("state data=%.*s", 1, &[ArgValue::Bytes(b"ok".to_vec())]);
        let params = Params::new(parser.lookup("state").unwrap(), &values);

        assert_eq!(params.get_bytes("data").unwrap(), b"ok");
        assert_eq!(params.get_str("data").unwrap(), "ok");
    }

    #[test]
    fn test_params_get_str_rejects_invalid_utf8() {
        let (parser, values) =
            params_for("state data=%.*s", 1, &[ArgValue::Bytes(vec![0xff, 0xfe])]);
        let params = Params::new(parser.lookup("state").unwrap(), &values);

        let err = params.get_str("data").unwrap_err();
        assert!(err.to_string().contains("not valid UTF-8"));
    }

    #[test]
    fn test_params_get_enum_resolves_name_and_falls_back() {
        let dict = Arc::new(dictionary());
        let (parser, values) = params_for(
            "is_shutdown static_string_id=%hu",
            14,
            &[ArgValue::UInt16(3)],
        );
        let params =
            Params::new(parser.lookup("is_shutdown").unwrap(), &values).with_dictionary(dict);

        assert_eq!(
            params
                .get_enum("static_string_id", "static_string_id")
                .unwrap(),
            "Timer too close"
        );
    }

    #[test]
    fn test_params_get_enum_unnamed_value_renders_question_mark() {
        let dict = Arc::new(dictionary());
        let (parser, values) = params_for(
            "is_shutdown static_string_id=%hu",
            14,
            &[ArgValue::UInt16(99)],
        );
        let params =
            Params::new(parser.lookup("is_shutdown").unwrap(), &values).with_dictionary(dict);

        assert_eq!(
            params
                .get_enum("static_string_id", "static_string_id")
                .unwrap(),
            "?99"
        );
    }

    #[test]
    fn test_params_get_enum_without_dictionary_fails() {
        let (parser, values) = params_for(
            "is_shutdown static_string_id=%hu",
            14,
            &[ArgValue::UInt16(3)],
        );
        let params = Params::new(parser.lookup("is_shutdown").unwrap(), &values);

        let err = params
            .get_enum("static_string_id", "static_string_id")
            .unwrap_err();
        assert!(err.to_string().contains("without a dictionary"));
    }

    #[test]
    fn test_params_get_enum_unknown_enumeration_fails() {
        let dict = Arc::new(dictionary());
        let (parser, values) = params_for(
            "is_shutdown static_string_id=%hu",
            14,
            &[ArgValue::UInt16(3)],
        );
        let params =
            Params::new(parser.lookup("is_shutdown").unwrap(), &values).with_dictionary(dict);

        let err = params
            .get_enum("nonexistent", "static_string_id")
            .unwrap_err();
        assert!(err.to_string().contains("no enumeration 'nonexistent'"));
    }

    // -----------------------------------------------------------------------
    // Mcu::send_msg / Mcu::call_msg
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_send_msg_before_identify_fails() {
        let mcu = Mcu::from((
            "test_mcu".to_string(),
            Interface::new(TestDevice::new(Vec::new())),
        ));

        assert!(!mcu.is_identified());
        let err = mcu.send_msg(&GetClock).unwrap_err();
        assert!(matches!(err, McuError::NotIdentified));
    }

    #[tokio::test]
    async fn test_install_dictionary_reports_installed_count() {
        let mcu = Mcu::from((
            "test_mcu".to_string(),
            Interface::new(TestDevice::new(Vec::new())),
        ));

        // 3 commands + 3 responses; the dictionary has no identify entries.
        assert_eq!(mcu.install_dictionary(dictionary()).unwrap(), 6);
        assert!(mcu.is_identified());

        let installed = mcu.dictionary().expect("dictionary installed");
        assert_eq!(installed.constant_f64("CLOCK_FREQ"), Some(20_000_000.0));
    }

    #[tokio::test]
    async fn test_send_msg_rejects_command_absent_from_dictionary() {
        struct NotInFirmware;
        impl McuCommand for NotInFirmware {
            const NAME: &'static str = "no_such_command";
            fn args(&self) -> Vec<ArgValue> {
                Vec::new()
            }
        }

        let mcu = mcu_with(dictionary(), Vec::new());
        let err = mcu.send_msg(&NotInFirmware).unwrap_err();
        assert!(matches!(err, McuError::UnknownMessage(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_send_msg_rejects_wrong_argument_type() {
        struct BadArgs;
        impl McuCommand for BadArgs {
            const NAME: &'static str = "clear_shutdown";
            fn args(&self) -> Vec<ArgValue> {
                // `clear_shutdown` takes no parameters.
                vec![ArgValue::UInt32(1)]
            }
        }

        let mcu = mcu_with(dictionary(), Vec::new());
        assert!(matches!(
            mcu.send_msg(&BadArgs).unwrap_err(),
            McuError::Msg(_)
        ));
    }

    #[tokio::test]
    async fn test_call_msg_roundtrip() {
        // The command encodes to its firmware id (5); the response is
        // `clock clock=%u` (id 18) with the clock value 0x1234.
        let mappings = vec![MappingEntry {
            input: Frame::new(0, payload(&[ArgValue::UInt8(5)])),
            outputs: vec![Frame::new(
                0,
                payload(&[ArgValue::UInt8(18), ArgValue::UInt32(0x1234)]),
            )],
        }];
        let mcu = mcu_with(dictionary(), mappings);

        let response = mcu
            .call_msg::<GetClock, ClockState>(&GetClock, Duration::from_secs(1))
            .await
            .unwrap();

        assert_eq!(response, ClockState { clock: 0x1234 });
    }

    #[tokio::test]
    async fn test_call_msg_decodes_multiple_parameters_by_name() {
        let mappings = vec![MappingEntry {
            input: Frame::new(0, payload(&[ArgValue::UInt8(4)])),
            outputs: vec![Frame::new(
                0,
                payload(&[
                    ArgValue::UInt8(17),
                    ArgValue::UInt32(1),
                    ArgValue::UInt32(0xabcd),
                ]),
            )],
        }];
        let mcu = mcu_with(dictionary(), mappings);

        let response = mcu
            .call_msg::<GetUptime, Uptime>(&GetUptime, Duration::from_secs(1))
            .await
            .unwrap();

        assert_eq!(
            response,
            Uptime {
                high: 1,
                clock: 0xabcd
            }
        );
    }

    #[tokio::test]
    async fn test_call_msg_times_out_when_mcu_stays_silent() {
        // Nothing is mapped, so the device never answers. The response name is
        // known, so the call must reach the timeout path.
        let mcu = mcu_with(dictionary(), Vec::new());

        let err = mcu
            .call_msg::<GetClock, IsShutdown>(&GetClock, Duration::from_millis(50))
            .await
            .unwrap_err();

        match err {
            McuError::Call(McuCallError::Timeout(msg)) => {
                assert!(msg.contains("no response for is_shutdown"), "{msg}");
            }
            other => panic!("expected a timeout, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn test_call_msg_reports_message_absent_from_dictionary() {
        #[derive(Debug)]
        struct NoSuchResponse;
        impl McuResponse for NoSuchResponse {
            const NAME: &'static str = "no_such_response";
            fn decode(_params: &Params<'_>) -> Result<Self, McuError> {
                Ok(Self)
            }
        }

        let mcu = mcu_with(dictionary(), Vec::new());
        let err = mcu
            .call_msg::<GetClock, NoSuchResponse>(&GetClock, Duration::from_secs(30))
            .await
            .unwrap_err();

        assert!(matches!(err, McuError::UnknownMessage(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_call_msg_decode_failure_is_reported() {
        #[derive(Debug)]
        struct BadDecoder;
        impl McuResponse for BadDecoder {
            const NAME: &'static str = "clock";
            fn decode(params: &Params<'_>) -> Result<Self, McuError> {
                params.get_u32("nonexistent")?;
                Ok(Self)
            }
        }

        let mappings = vec![MappingEntry {
            input: Frame::new(0, payload(&[ArgValue::UInt8(5)])),
            outputs: vec![Frame::new(
                0,
                payload(&[ArgValue::UInt8(18), ArgValue::UInt32(1)]),
            )],
        }];
        let mcu = mcu_with(dictionary(), mappings);

        let err = mcu
            .call_msg::<GetClock, BadDecoder>(&GetClock, Duration::from_secs(1))
            .await
            .unwrap_err();

        assert!(matches!(err, McuError::Decode(_)), "{err:?}");
    }
}
