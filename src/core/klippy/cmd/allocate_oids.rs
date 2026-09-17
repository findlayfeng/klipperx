//! `allocate_oids` — reserve the object ids a configuration will use.
//!
//! Host view of the "Low level allocation" section of the firmware's
//! `basecmd.c`. The firmware hands out runtime storage by *object id* (`oid`),
//! and every later command that allocates one refers to it by number, so this
//! single command has to run first and has to cover the whole configuration:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `allocate_oids count=%c` |
//!
//! There is no response. `count` is one byte, matching the firmware's `%c`; the
//! firmware refuses a second `allocate_oids` once ids exist, and refuses any
//! `oid_alloc` after [`finalize_config`](super::config::FinalizeConfig).

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `allocate_oids count=%c` — reserve `count` object ids.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AllocateOids {
    /// Number of ids to reserve.
    pub count: u8,
}

impl McuCommand for AllocateOids {
    const NAME: &'static str = "allocate_oids";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.count)]
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

    /// The one message this module needs, as the firmware would publish it.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {"allocate_oids count=%c": 2}
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn test_allocate_oids_matches_the_firmware_format() {
        let command = AllocateOids { count: 3 };
        let parser = parser();

        let encoded = parser.encode(AllocateOids::NAME, &command.args()).unwrap();

        // Spelled out against the wire: id 2 as a VLQ, then the count as a
        // single byte (`%c`).
        let mut expected = Payload::new();
        expected.push_i16(2).unwrap();
        expected.push_u8(3).unwrap();
        assert_eq!(encoded.payload(), expected.payload());

        // Decoding what was just encoded yields the arguments again, so the view
        // and the firmware format cannot drift apart unnoticed.
        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, AllocateOids::NAME);
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn test_allocate_oids_count_spans_the_whole_byte() {
        // `%c` is one byte, so the largest id count the firmware can be asked
        // for round-trips instead of wrapping.
        let command = AllocateOids { count: u8::MAX };
        let parser = parser();

        let encoded = parser.encode(AllocateOids::NAME, &command.args()).unwrap();
        let decoded = parser.decode(encoded).unwrap();

        assert_eq!(decoded[0].1, vec![ArgValue::UInt8(u8::MAX)]);
    }
}
