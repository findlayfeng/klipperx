//! `get_uptime` — read the firmware's full 64-bit clock.
//!
//! Host view of the "Timing and load stats" section of the firmware's
//! `basecmd.c`. The firmware's clock counter is 32 bits wide and wraps, so the
//! firmware keeps a *high* word alongside it and `get_uptime` returns both:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `get_uptime` |
//! | MCU → host | `uptime high=%u clock=%u` |
//!
//! `clock` is the same low word `get_clock` returns (parked in `cmd/clock.rs`);
//! `high` is the number of times it has wrapped since boot. The pair
//! gives a monotonic value that can be ordered across a wrap, which is what
//! clock synchronisation needs at connect time. The per-message `stats`
//! counters `stats_update` also sends live in the same C section, but they are
//! pushed on a timer without a request, so they are not part of this module.
//!
//! `get_uptime` is `HF_IN_SHUTDOWN`, so it still answers after an emergency
//! stop — useful for reading the time an MCU stopped.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `get_uptime` — ask for the 64-bit clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GetUptime;

impl McuCommand for GetUptime {
    const NAME: &'static str = "get_uptime";

    fn args(&self) -> Vec<ArgValue> {
        // No parameters.
        Vec::new()
    }
}

/// `uptime high=%u clock=%u` — the firmware clock split into two 32-bit words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Uptime {
    /// Number of times the low word has wrapped since boot.
    pub high: u32,
    /// Low 32 bits of the firmware clock.
    pub clock: u32,
}

impl Uptime {
    /// The two words as one 64-bit tick count: `(high << 32) | clock`.
    ///
    /// The firmware sends the two words separately because the wire format has
    /// no 64-bit type; this is how Klipper's own host recombines them.
    pub fn clock64(&self) -> u64 {
        (u64::from(self.high) << 32) | u64::from(self.clock)
    }
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
            "commands": {"get_uptime": 4},
            "responses": {"uptime high=%u clock=%u": 17}
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    /// Decode one `uptime` response from its wire values.
    fn decode_uptime(parser: &Parser, values: &[ArgValue]) -> Result<Uptime, McuError> {
        let encoded = parser.encode(Uptime::NAME, values).unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let (msg, params) = &decoded[0];
        Uptime::decode(&Params::new(Arc::clone(msg), params))
    }

    #[test]
    fn test_get_uptime_matches_the_firmware_format() {
        let parser = parser();

        let encoded = parser.encode(GetUptime::NAME, &GetUptime.args()).unwrap();

        let mut expected = Payload::new();
        expected.push_i16(4).unwrap();
        assert_eq!(encoded.payload(), expected.payload());
    }

    #[test]
    fn test_uptime_recombines_into_a_64_bit_clock() {
        let parser = parser();
        let uptime =
            decode_uptime(&parser, &[ArgValue::UInt32(1), ArgValue::UInt32(0x1234)]).unwrap();

        assert_eq!(uptime.high, 1);
        assert_eq!(uptime.clock, 0x1234);
        assert_eq!(uptime.clock64(), 0x1_0000_1234);
    }

    #[test]
    fn test_uptime_orders_correctly_across_a_32_bit_wrap() {
        // Just before and just after the low word wraps: the raw `clock` values
        // compare the wrong way round, the recombined 64-bit value does not.
        let parser = parser();
        let before =
            decode_uptime(&parser, &[ArgValue::UInt32(0), ArgValue::UInt32(u32::MAX)]).unwrap();
        let after = decode_uptime(&parser, &[ArgValue::UInt32(1), ArgValue::UInt32(0)]).unwrap();

        assert!(before.clock > after.clock);
        assert!(before.clock64() < after.clock64());
    }

    #[test]
    fn test_uptime_decode_reports_parameter_mismatches() {
        // Same message name, but `high` declared as a string.
        let mut parser = Parser::new();
        parser.register(0, "uptime high=%s clock=%u").unwrap();
        let err =
            decode_uptime(&parser, &[ArgValue::Str("1".into()), ArgValue::UInt32(2)]).unwrap_err();

        assert!(matches!(err, McuError::Decode(_)), "{err:?}");
    }
}
