//! DS18B20 (1-wire) temperature commands.
//!
//! Host view of the "DS18B20" part of the firmware's `ds18b20.c`. The sensor is
//! configured once, then the firmware polls it on a timer and pushes each
//! reading:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_ds18b20 oid=%c serial=%s max_error_count=%c` |
//! | host → MCU | `query_ds18b20 oid=%c clock=%u rest_ticks=%u min_value=%i max_value=%i` |
//! | MCU → host | `ds18b20_result oid=%c next_clock=%u value=%i fault=%u` |
//!
//! The `query_...` goes in the **init** list: it arms a periodic timer rather
//! than describing a one-time configuration. `value` is millidegrees Celsius and
//! `fault` is a bit mask; a non-zero fault means the reading is not usable.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_ds18b20 oid=%c serial=%s max_error_count=%c`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigDs18b20 {
    /// The object id the firmware allocated for this sensor.
    pub oid: u8,
    /// The 1-wire serial number, as hex text (one byte per two characters).
    pub serial: String,
    /// How many consecutive errors the firmware tolerates before shutting down.
    pub max_error_count: u8,
}

impl McuCommand for ConfigDs18b20 {
    const NAME: &'static str = "config_ds18b20";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Str(self.serial.clone()),
            ArgValue::UInt8(self.max_error_count),
        ]
    }
}

/// `query_ds18b20 oid=%c clock=%u rest_ticks=%u min_value=%i max_value=%i`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryDs18b20 {
    /// The sensor's oid.
    pub oid: u8,
    /// Absolute firmware clock of the first reading.
    pub clock: u32,
    /// Time between readings, in clock ticks.
    pub rest_ticks: u32,
    /// Lowest accepted value, in millidegrees.
    pub min_value: i32,
    /// Highest accepted value, in millidegrees.
    pub max_value: i32,
}

impl McuCommand for QueryDs18b20 {
    const NAME: &'static str = "query_ds18b20";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.rest_ticks),
            ArgValue::Int32(self.min_value),
            ArgValue::Int32(self.max_value),
        ]
    }
}

/// `ds18b20_result oid=%c next_clock=%u value=%i fault=%u` — one reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ds18b20Result {
    /// The sensor's oid.
    pub oid: u8,
    /// Firmware clock of the reading after this one.
    pub next_clock: u32,
    /// The reading, in millidegrees Celsius.
    pub value: i32,
    /// A non-zero fault means the reading is not usable.
    pub fault: u32,
}

impl McuResponse for Ds18b20Result {
    const NAME: &'static str = "ds18b20_result";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            next_clock: params.get_u32("next_clock")?,
            value: params.get_i32("value")?,
            fault: params.get_u32("fault")?,
        })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use serde_json::json;

    fn parser() -> Parser {
        let dictionary = Dictionary::from_json(json!({
            "commands": {
                "config_ds18b20 oid=%c serial=%s max_error_count=%c": 40,
                "query_ds18b20 oid=%c clock=%u rest_ticks=%u min_value=%i max_value=%i": 41
            },
            "responses": {
                "ds18b20_result oid=%c next_clock=%u value=%i fault=%u": 42
            }
        }))
        .unwrap();
        let mut parser = Parser::new();
        dictionary.install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn test_the_commands_match_the_firmware_format() {
        let parser = parser();

        let config = ConfigDs18b20 {
            oid: 3,
            serial: "28ff00".to_string(),
            max_error_count: 4,
        };
        let encoded = parser.encode(ConfigDs18b20::NAME, &config.args()).unwrap();
        assert_eq!(parser.decode(encoded).unwrap()[0].1, config.args());

        let query = QueryDs18b20 {
            oid: 3,
            clock: 1_000,
            rest_ticks: 60_000_000,
            min_value: -273_150,
            max_value: 999_999,
        };
        let encoded = parser.encode(QueryDs18b20::NAME, &query.args()).unwrap();
        assert_eq!(parser.decode(encoded).unwrap()[0].1, query.args());
    }

    #[test]
    fn test_a_result_decodes() {
        let parser = parser();
        let encoded = parser
            .encode(
                Ds18b20Result::NAME,
                &[
                    ArgValue::UInt8(3),
                    ArgValue::UInt32(9_000),
                    ArgValue::Int32(24500),
                    ArgValue::UInt32(0),
                ],
            )
            .unwrap();
        let msg = parser.decode(encoded).unwrap().remove(0);
        let params = Params::new(msg.0, &msg.1);

        let result = Ds18b20Result::decode(&params).unwrap();

        assert_eq!(result.oid, 3);
        assert_eq!(result.value, 24500);
        assert_eq!(result.fault, 0);
    }
}
