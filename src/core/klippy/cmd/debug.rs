//! Debug commands — `debug_read`, `debug_write`, `debug_ping`, and `debug_nop`.
//!
//! Host view of the commands in the firmware's `debugcmds.c`. These are used for
//! low-level memory access and firmware testing:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `debug_read order=%c addr=%u` |
//! | MCU → host | `debug_result val=%u` |
//! | host → MCU | `debug_write order=%c addr=%u val=%u` |
//! | host → MCU | `debug_ping data=%*s` |
//! | MCU → host | `pong data=%*s` |
//! | host → MCU | `debug_nop` |
//!
//! All four firmware commands are declared `HF_IN_SHUTDOWN`
//! (`src/debugcmds.c:26,43,53,59`), so they answer even on a stopped board —
//! which is what lets `temperature_mcu` read factory calibration before the
//! configuration is built.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `debug_read order=%c addr=%u` — read a value from MCU memory.
///
/// - `order`: access size (0=8-bit, 1=16-bit, 2=32-bit)
/// - `addr`: decoded pointer address
///
/// Response: `debug_result val=%u`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugRead {
    /// Access size (0=8-bit, 1=16-bit, 2=32-bit).
    pub order: u8,
    /// Decoded pointer address.
    pub addr: u32,
}

impl McuCommand for DebugRead {
    const NAME: &'static str = "debug_read";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.order), ArgValue::UInt32(self.addr)]
    }
}

/// `debug_write order=%c addr=%u val=%u` — write a value to MCU memory.
///
/// - `order`: access size (0=8-bit, 1=16-bit, 2=32-bit)
/// - `addr`: decoded pointer address
/// - `val`: value to write
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugWrite {
    /// Access size (0=8-bit, 1=16-bit, 2=32-bit).
    pub order: u8,
    /// Decoded pointer address.
    pub addr: u32,
    /// Value to write.
    pub val: u32,
}

impl McuCommand for DebugWrite {
    const NAME: &'static str = "debug_write";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.order),
            ArgValue::UInt32(self.addr),
            ArgValue::UInt32(self.val),
        ]
    }
}

/// `debug_ping data=%*s` — echo a payload back as `pong data=%*s`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DebugPing {
    /// The bytes to echo.
    pub data: Vec<u8>,
}

impl McuCommand for DebugPing {
    const NAME: &'static str = "debug_ping";

    fn args(&self) -> Vec<ArgValue> {
        // `%*s` is binary-safe: `ArgType::Bytes`, not `Str`.
        vec![ArgValue::Bytes(self.data.clone())]
    }
}

/// `pong data=%*s` — the firmware's echo of a `debug_ping`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pong {
    /// The echoed bytes.
    pub data: Vec<u8>,
}

/// `McuResponse` for `pong data=%*s`.
///
/// # Errors
/// Returns [`McuError::Decode`] if `data` is missing or not a byte string.
impl McuResponse for Pong {
    const NAME: &'static str = "pong";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            data: params.get_bytes("data")?,
        })
    }
}

/// `debug_nop` — a command with no arguments and no response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugNop;

impl McuCommand for DebugNop {
    const NAME: &'static str = "debug_nop";

    fn args(&self) -> Vec<ArgValue> {
        Vec::new()
    }
}

/// `debug_result val=%u` — response to `debug_read`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugResult {
    /// The value read from MCU memory.
    pub val: u32,
}

/// `McuResponse` for `debug_result val=%u`.
///
/// # Errors
/// Returns [`McuError::Decode`] if the `val` parameter is missing or not a
/// 32-bit unsigned integer.
impl McuResponse for DebugResult {
    const NAME: &'static str = "debug_result";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            val: params.get_u32("val")?,
        })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, MappingEntry};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::Payload;
    use crate::core::klippy::msg::Msg;
    use serde_json::json;
    use std::sync::Arc;
    use std::time::Duration;

    // -----------------------------------------------------------------------
    // A dictionary and a virtual MCU
    // -----------------------------------------------------------------------

    /// The four commands and two responses, as the firmware declares them
    /// (`src/debugcmds.c`), with made-up ids.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "debug_read order=%c addr=%u": 12,
                "debug_write order=%c addr=%u val=%u": 11,
                "debug_ping data=%*s": 10,
                "debug_nop": 9
            },
            "responses": {
                "debug_result val=%u": -7,
                "pong data=%*s": -8
            }
        }))
        .unwrap()
    }

    fn parser() -> Parser {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser
    }

    /// One frame carrying the message id followed by its arguments.
    fn frame(seq: u8, parts: &[ArgValue]) -> Frame {
        let mut payload = Payload::new();
        for value in parts {
            payload.push_value(value).unwrap();
        }
        Frame::new(seq, payload.into_raw())
    }

    fn mcu_with(mappings: Vec<MappingEntry>) -> Mcu {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(mappings)));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu
    }

    // -----------------------------------------------------------------------
    // Wire shape
    // -----------------------------------------------------------------------

    #[test]
    fn test_debug_read_and_write_match_the_firmware_format() {
        let read = DebugRead {
            order: 2,
            addr: 0x2000_0000,
        };
        let encoded = parser().encode(DebugRead::NAME, &read.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "debug_read");
        assert_eq!(decoded[0].1, read.args());

        let write = DebugWrite {
            order: 1,
            addr: 0x2000_0000,
            val: 0x1234_5678,
        };
        let encoded = parser().encode(DebugWrite::NAME, &write.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "debug_write");
        assert_eq!(decoded[0].1, write.args());
    }

    #[test]
    fn test_debug_ping_carries_bytes_and_nop_is_empty() {
        let ping = DebugPing {
            data: vec![0x00, 0xff, 0x7e],
        };
        let encoded = parser().encode(DebugPing::NAME, &ping.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "debug_ping");
        assert_eq!(decoded[0].1, vec![ArgValue::Bytes(ping.data.clone())]);

        let encoded = parser().encode(DebugNop::NAME, &DebugNop.args()).unwrap();
        let decoded = parser().decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, "debug_nop");
        assert!(decoded[0].1.is_empty());
    }

    #[test]
    fn test_debug_result_and_pong_decode() {
        let msg = Msg::parse(-7, "debug_result val=%u").unwrap();
        let params = Params::new(Arc::new(msg), &[ArgValue::UInt32(0x1234_5678)]);
        assert_eq!(DebugResult::decode(&params).unwrap().val, 0x1234_5678);

        let msg = Msg::parse(-8, "pong data=%*s").unwrap();
        let values = [ArgValue::Bytes(vec![1, 2, 3])];
        let params = Params::new(Arc::new(msg), &values);
        assert_eq!(Pong::decode(&params).unwrap().data, vec![1, 2, 3]);
    }

    #[test]
    fn test_debug_result_and_pong_decode_missing_param() {
        let msg = Msg::parse(-7, "debug_result val=%u").unwrap();
        assert!(DebugResult::decode(&Params::new(Arc::new(msg), &[])).is_err());

        let msg = Msg::parse(-8, "pong data=%*s").unwrap();
        assert!(Pong::decode(&Params::new(Arc::new(msg), &[])).is_err());
    }

    // -----------------------------------------------------------------------
    // Through a virtual MCU
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_debug_read_roundtrips_through_a_virtual_mcu() {
        let mappings = vec![MappingEntry {
            input: frame(
                0,
                &[
                    ArgValue::Int16(12),
                    ArgValue::UInt8(2),
                    ArgValue::UInt32(0x2000_0000),
                ],
            ),
            outputs: vec![frame(
                0,
                &[ArgValue::Int16(-7), ArgValue::UInt32(0xdead_beef)],
            )],
        }];
        let mcu = mcu_with(mappings);

        let result = mcu
            .call_msg::<DebugRead, DebugResult>(
                &DebugRead {
                    order: 2,
                    addr: 0x2000_0000,
                },
                Duration::from_secs(1),
            )
            .await
            .unwrap();

        assert_eq!(result.val, 0xdead_beef);
    }

    #[tokio::test]
    async fn test_debug_ping_roundtrips_through_a_virtual_mcu() {
        // Long enough to leave as its own frame, so the payload is exercised
        // across the frame boundary rather than merged with a neighbour.
        let data = b"ping payload long enough to not be merged!".to_vec();
        let mappings = vec![MappingEntry {
            input: frame(0, &[ArgValue::Int16(10), ArgValue::Bytes(data.clone())]),
            outputs: vec![frame(
                0,
                &[ArgValue::Int16(-8), ArgValue::Bytes(data.clone())],
            )],
        }];
        let mcu = mcu_with(mappings);

        let pong = mcu
            .call_msg::<DebugPing, Pong>(&DebugPing { data: data.clone() }, Duration::from_secs(1))
            .await
            .unwrap();

        assert_eq!(pong.data, data);
    }

    #[tokio::test]
    async fn test_debug_write_reaches_the_wire() {
        // The write has no response, so its arrival is proven by the read that
        // follows: the device compares frames in FIFO order.
        let mappings = vec![
            MappingEntry {
                input: frame(
                    0,
                    &[
                        ArgValue::Int16(11),
                        ArgValue::UInt8(2),
                        ArgValue::UInt32(0x2000_0000),
                        ArgValue::UInt32(0xdead_beef),
                    ],
                ),
                outputs: Vec::new(),
            },
            MappingEntry {
                input: frame(
                    1,
                    &[
                        ArgValue::Int16(12),
                        ArgValue::UInt8(2),
                        ArgValue::UInt32(0x2000_0000),
                    ],
                ),
                outputs: vec![frame(
                    0,
                    &[ArgValue::Int16(-7), ArgValue::UInt32(0xdead_beef)],
                )],
            },
        ];
        let mcu = mcu_with(mappings);

        mcu.send_msg(&DebugWrite {
            order: 2,
            addr: 0x2000_0000,
            val: 0xdead_beef,
        })
        .unwrap();
        // Let the send task flush it as its own frame before the read.
        tokio::time::sleep(Duration::from_millis(5)).await;

        let result = mcu
            .call_msg::<DebugRead, DebugResult>(
                &DebugRead {
                    order: 2,
                    addr: 0x2000_0000,
                },
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(result.val, 0xdead_beef);
    }

    #[tokio::test]
    async fn test_debug_nop_reaches_the_wire() {
        let mappings = vec![
            MappingEntry {
                input: frame(0, &[ArgValue::Int16(9)]),
                outputs: Vec::new(),
            },
            MappingEntry {
                input: frame(
                    1,
                    &[ArgValue::Int16(12), ArgValue::UInt8(0), ArgValue::UInt32(4)],
                ),
                outputs: vec![frame(0, &[ArgValue::Int16(-7), ArgValue::UInt32(0x2a)])],
            },
        ];
        let mcu = mcu_with(mappings);

        mcu.send_msg(&DebugNop).unwrap();
        tokio::time::sleep(Duration::from_millis(5)).await;

        let result = mcu
            .call_msg::<DebugRead, DebugResult>(
                &DebugRead { order: 0, addr: 4 },
                Duration::from_secs(1),
            )
            .await
            .unwrap();
        assert_eq!(result.val, 0x2a);
    }
}
