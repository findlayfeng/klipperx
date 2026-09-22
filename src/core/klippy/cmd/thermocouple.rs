//! SPI thermocouple / RTD temperature commands.
//!
//! Host view of the firmware's `thermocouple.c`. One `config_thermocouple`
//! describes the chip and its SPI device, then the firmware polls it on a timer
//! and pushes each raw reading:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c` |
//! | host → MCU | `query_thermocouple oid=%c clock=%u rest_ticks=%u min_value=%u max_value=%u max_invalid_count=%c` |
//! | MCU → host | `thermocouple_result oid=%c next_clock=%u value=%u fault=%c` |
//!
//! `thermocouple_type` is an **enumeration** (`src/thermocouple.c:20-23`), so it
//! travels as `%c`; the numbers are in [`ThermocoupleType`]. `value` is the raw
//! register reading; how to turn it into degrees lives with each chip in
//! `extras::spi_temperature`.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// The firmware's `thermocouple_type` enumeration (`src/thermocouple.c:15-23`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThermocoupleType {
    /// `MAX31855` — thermocouple, mode 0.
    Max31855 = 0,
    /// `MAX31856` — thermocouple, mode 1, needs SPI init registers.
    Max31856 = 1,
    /// `MAX31865` — RTD, mode 1, needs an SPI init register.
    Max31865 = 2,
    /// `MAX6675` — thermocouple, mode 0.
    Max6675 = 3,
}

/// `config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigThermocouple {
    /// The object id the firmware allocated for this sensor.
    pub oid: u8,
    /// The oid of the SPI device this chip hangs off.
    pub spi_oid: u8,
    /// Which chip the firmware should read.
    pub thermocouple_type: ThermocoupleType,
}

impl McuCommand for ConfigThermocouple {
    const NAME: &'static str = "config_thermocouple";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.spi_oid),
            ArgValue::UInt8(self.thermocouple_type as u8),
        ]
    }
}

/// `query_thermocouple oid=%c clock=%u rest_ticks=%u min_value=%u max_value=%u max_invalid_count=%c`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryThermocouple {
    /// The sensor's oid.
    pub oid: u8,
    /// Absolute firmware clock of the first reading.
    pub clock: u32,
    /// Time between readings, in clock ticks.
    pub rest_ticks: u32,
    /// Lowest accepted raw value.
    pub min_value: u32,
    /// Highest accepted raw value.
    pub max_value: u32,
    /// How many bad readings the firmware tolerates before shutting down.
    pub max_invalid_count: u8,
}

impl McuCommand for QueryThermocouple {
    const NAME: &'static str = "query_thermocouple";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.rest_ticks),
            ArgValue::UInt32(self.min_value),
            ArgValue::UInt32(self.max_value),
            ArgValue::UInt8(self.max_invalid_count),
        ]
    }
}

/// `thermocouple_result oid=%c next_clock=%u value=%u fault=%c` — one reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThermocoupleResult {
    /// The sensor's oid.
    pub oid: u8,
    /// Firmware clock of the reading after this one.
    pub next_clock: u32,
    /// The chip's raw register value.
    pub value: u32,
    /// A non-zero fault means the reading is not usable.
    pub fault: u8,
}

impl McuResponse for ThermocoupleResult {
    const NAME: &'static str = "thermocouple_result";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            next_clock: params.get_u32("next_clock")?,
            value: params.get_u32("value")?,
            fault: params.get_u8("fault")?,
        })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::debug::DebugNop;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, MappingEntry};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::Payload;
    use crate::core::klippy::msg::Msg;
    use serde_json::json;
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c": 50,
                "query_thermocouple oid=%c clock=%u rest_ticks=%u min_value=%u \
                 max_value=%u max_invalid_count=%c": 51,
                "debug_nop": 9
            },
            "responses": {
                "thermocouple_result oid=%c next_clock=%u value=%u fault=%c": -9
            }
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    fn frame(seq: u8, parts: &[ArgValue]) -> Frame {
        let mut payload = Payload::new();
        for value in parts {
            payload.push_value(value).unwrap();
        }
        Frame::new(seq, payload.into_raw())
    }

    #[test]
    fn test_the_commands_match_the_firmware_format() {
        let parser = parser();

        let config = ConfigThermocouple {
            oid: 4,
            spi_oid: 1,
            thermocouple_type: ThermocoupleType::Max31855,
        };
        let encoded = parser
            .encode(ConfigThermocouple::NAME, &config.args())
            .unwrap();
        assert_eq!(parser.decode(encoded).unwrap()[0].1, config.args());

        let query = QueryThermocouple {
            oid: 4,
            clock: 1_000,
            rest_ticks: 60_000_000,
            min_value: 0,
            max_value: 16_383,
            max_invalid_count: 3,
        };
        let encoded = parser
            .encode(QueryThermocouple::NAME, &query.args())
            .unwrap();
        assert_eq!(parser.decode(encoded).unwrap()[0].1, query.args());
    }

    #[test]
    fn test_the_chip_type_numbers_match_the_firmware_enumeration() {
        // `src/thermocouple.c:15-23`.
        assert_eq!(ThermocoupleType::Max31855 as u8, 0);
        assert_eq!(ThermocoupleType::Max31856 as u8, 1);
        assert_eq!(ThermocoupleType::Max31865 as u8, 2);
        assert_eq!(ThermocoupleType::Max6675 as u8, 3);
    }

    #[test]
    fn test_a_result_decodes() {
        let parser = parser();
        let encoded = parser
            .encode(
                ThermocoupleResult::NAME,
                &[
                    ArgValue::UInt8(4),
                    ArgValue::UInt32(9_000),
                    ArgValue::UInt32(0x0001_9000),
                    ArgValue::UInt8(0),
                ],
            )
            .unwrap();
        let msg = parser.decode(encoded).unwrap().remove(0);
        let params = Params::new(msg.0, &msg.1);

        let result = ThermocoupleResult::decode(&params).unwrap();

        assert_eq!(result.oid, 4);
        assert_eq!(result.value, 0x0001_9000);
        assert_eq!(result.fault, 0);
    }

    #[tokio::test]
    async fn test_a_result_arrives_through_a_virtual_mcu() {
        // Feed one `thermocouple_result` from the fake device and decode it the
        // way the per-oid registry does, so the full receive path is exercised.
        let mappings = vec![MappingEntry {
            input: frame(0, &[ArgValue::Int16(9)]), // debug_nop
            outputs: vec![frame(
                0,
                &[
                    ArgValue::Int16(-9),
                    ArgValue::UInt8(4),
                    ArgValue::UInt32(9_000),
                    ArgValue::UInt32(0x0001_9000),
                    ArgValue::UInt8(0),
                ],
            )],
        }];
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(mappings)));
        mcu.install_dictionary(dictionary()).unwrap();

        let msg = mcu.require_message(ThermocoupleResult::NAME).unwrap();
        let declaration = Arc::new(Msg::new(msg.id, msg.name.clone(), msg.params.clone()));
        let dictionary = mcu.dictionary();
        let seen: Arc<Mutex<Option<ThermocoupleResult>>> = Arc::new(Mutex::new(None));
        let seen_for_callback = Arc::clone(&seen);
        mcu.bind_callback(ThermocoupleResult::NAME, move |values| {
            let params = Params::new(Arc::clone(&declaration), values);
            let params = match &dictionary {
                Some(dictionary) => params.with_dictionary(Arc::clone(dictionary)),
                None => params,
            };
            if let Ok(result) = ThermocoupleResult::decode(&params) {
                *seen_for_callback.lock().unwrap() = Some(result);
            }
        })
        .unwrap();

        mcu.send_msg(&DebugNop).unwrap();
        tokio::time::sleep(Duration::from_millis(20)).await;

        let result = seen.lock().unwrap().expect("the result arrived");
        assert_eq!(result.oid, 4);
        assert_eq!(result.next_clock, 9_000);
        assert_eq!(result.value, 0x0001_9000);
        assert_eq!(result.fault, 0);
    }
}
