//! Shared fixtures for the event module's tests.
//!
//! Compiled under `cfg(test)` only. The format strings here are a *fixture* —
//! what the firmware would publish for these messages — not host constants; the
//! production code never names a format.

use crate::core::klippy::frame::Frame;
use crate::core::klippy::interface::devices::frame_mock::{FrameMock, MappingEntry};
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::{Dictionary, Mcu};
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use serde_json::json;
use std::sync::Arc;

/// The messages the event tests need, as the firmware would publish them.
///
/// `get_uptime` is only here to be a registered command the tests can send; the
/// reply the fake device queues for it is a `stats` report, so the event path
/// is exercised without a request that pairs with it.
pub(super) fn dictionary() -> Dictionary {
    Dictionary::from_json(json!({
        "commands": {"get_uptime": 4},
        "responses": {"stats count=%u sum=%u sumsq=%u": 12}
    }))
    .unwrap()
}

/// A parser with the fixture dictionary installed.
pub(super) fn parser() -> Parser {
    let mut parser = Parser::new();
    dictionary().install(&mut parser).unwrap();
    parser
}

/// Payload of a message built from its firmware member order.
pub(super) fn payload(parts: &[ArgValue]) -> Vec<u8> {
    let mut out = Payload::new();
    for value in parts {
        out.push_value(value).unwrap();
    }
    out.into_raw()
}

/// Frame carrying `parts` as the message id followed by its arguments.
pub(super) fn frame(seq: u8, parts: &[ArgValue]) -> Frame {
    Frame::new(seq, payload(parts))
}

/// An identified MCU whose device answers from `mappings`.
pub(super) fn mcu(mappings: Vec<MappingEntry>) -> Arc<Mcu> {
    let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(mappings)));
    mcu.install_dictionary(dictionary()).unwrap();
    Arc::new(mcu)
}
