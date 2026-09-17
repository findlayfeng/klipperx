//! `get_config` / `finalize_config` — the configuration CRC handshake.
//!
//! Host view of the "Config CRC" section of the firmware's `basecmd.c`. On
//! connect the host asks the MCU what it is configured with, and either the CRC
//! matches and the MCU is reused, or the host (re)sends the configuration and
//! finalizes it with the CRC it computed.
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `get_config` |
//! | MCU → host | `config is_config=%c crc=%u is_shutdown=%c move_count=%hu` |
//! | host → MCU | `finalize_config crc=%u` |
//!
//! Both commands are `HF_IN_SHUTDOWN`: the firmware answers and records even
//! while stopped, which is what lets the host inspect an MCU that shut down
//! instead of guessing why it did.
//!
//! `finalize_config` also ends the configuration phase: after it the firmware
//! refuses further `oid_alloc` and a second finalize.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `get_config` — ask whether the MCU is configured, and with which CRC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GetConfig;

impl McuCommand for GetConfig {
    const NAME: &'static str = "get_config";

    fn args(&self) -> Vec<ArgValue> {
        // No parameters.
        Vec::new()
    }
}

/// `config is_config=%c crc=%u is_shutdown=%c move_count=%hu` — the MCU's
/// configuration state.
///
/// `is_config` is false until a `finalize_config` has been accepted, so a fresh
/// or reset MCU answers `is_config=0`. `crc` is the value the firmware recorded
/// from that `finalize_config`; matching it means the MCU can keep its current
/// configuration. `move_count` is the size of the move queue the firmware
/// allocated, which the host's motion queue has to fit into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigState {
    /// Whether the firmware has finalized a configuration.
    pub is_config: bool,
    /// CRC of the finalized configuration (`finalize_config crc=%u`).
    pub crc: u32,
    /// Whether the firmware is in its shutdown state.
    pub is_shutdown: bool,
    /// Number of moves the firmware's move queue holds.
    pub move_count: u16,
}

impl McuResponse for ConfigState {
    const NAME: &'static str = "config";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            // The wire type is a byte; the firmware only ever sends 0 or 1.
            is_config: params.get_u8("is_config")? != 0,
            crc: params.get_u32("crc")?,
            is_shutdown: params.get_u8("is_shutdown")? != 0,
            move_count: params.get_u16("move_count")?,
        })
    }
}

/// `finalize_config crc=%u` — lock the configuration and record its CRC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FinalizeConfig {
    /// CRC over the configuration the host just sent.
    pub crc: u32,
}

impl McuCommand for FinalizeConfig {
    const NAME: &'static str = "finalize_config";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt32(self.crc)]
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
    use crate::core::klippy::msg::proto::Payload;
    use serde_json::json;
    use std::sync::Arc;

    /// The messages this module needs, as the firmware would publish them.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "get_config": 7,
                "finalize_config crc=%u": 6
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            }
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    /// Decode one `config` response from its wire values.
    fn decode_config(parser: &Parser, values: &[ArgValue]) -> Result<ConfigState, McuError> {
        let encoded = parser.encode(ConfigState::NAME, values).unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let (msg, params) = &decoded[0];
        ConfigState::decode(&Params::new(Arc::clone(msg), params))
    }

    #[test]
    fn test_get_config_matches_the_firmware_format() {
        let parser = parser();

        let encoded = parser.encode(GetConfig::NAME, &GetConfig.args()).unwrap();

        let mut expected = Payload::new();
        expected.push_i16(7).unwrap();
        assert_eq!(encoded.payload(), expected.payload());
    }

    #[test]
    fn test_finalize_config_matches_the_firmware_format() {
        let command = FinalizeConfig { crc: 0xdead_beef };
        let parser = parser();

        let encoded = parser
            .encode(FinalizeConfig::NAME, &command.args())
            .unwrap();

        let mut expected = Payload::new();
        expected.push_i16(6).unwrap();
        expected.push_u32(0xdead_beef).unwrap();
        assert_eq!(encoded.payload(), expected.payload());

        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_config_state_is_read_by_name_not_position() {
        // The firmware declares `is_config` first; the view asks by name.
        let parser = parser();
        let state = decode_config(
            &parser,
            &[
                ArgValue::UInt8(1),
                ArgValue::UInt32(0x1234_5678),
                ArgValue::UInt8(0),
                ArgValue::UInt16(1024),
            ],
        )
        .unwrap();

        assert!(state.is_config);
        assert_eq!(state.crc, 0x1234_5678);
        assert!(!state.is_shutdown);
        assert_eq!(state.move_count, 1024);
    }

    #[test]
    fn test_config_state_reports_a_shutdown_unconfigured_mcu() {
        // A fresh MCU on a stopped connection: not configured, shut down, and
        // with no move queue yet.
        let parser = parser();
        let state = decode_config(
            &parser,
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(1),
                ArgValue::UInt16(0),
            ],
        )
        .unwrap();

        assert!(!state.is_config);
        assert_eq!(state.crc, 0);
        assert!(state.is_shutdown);
        assert_eq!(state.move_count, 0);
    }

    #[test]
    fn test_config_state_decode_reports_parameter_mismatches() {
        // Same message, but `is_config` declared as a string: the view asks for
        // a byte and must refuse rather than reinterpret the text.
        let mut parser = Parser::new();
        parser
            .register(
                0,
                "config is_config=%s crc=%u is_shutdown=%c move_count=%hu",
            )
            .unwrap();
        let err = decode_config(
            &parser,
            &[
                ArgValue::Str("yes".into()),
                ArgValue::UInt32(0),
                ArgValue::UInt8(0),
                ArgValue::UInt16(0),
            ],
        )
        .unwrap_err();

        assert!(matches!(err, McuError::Decode(_)), "{err:?}");
    }
}
