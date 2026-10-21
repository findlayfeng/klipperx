//! Button query commands — upstream `klippy/extras/buttons.py`'s `MCU_buttons`.
//!
//! The firmware polls up to eight pins of one object id on a fixed period and
//! reports the levels that changed. The host configures the object once, arms
//! the polling with an absolute clock, and acknowledges each report:
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_buttons oid=%c button_count=%c` |
//! | host → MCU | `buttons_add oid=%c pos=%c pin=%u pull_up=%c` |
//! | host → MCU | `buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c` |
//! | host → MCU | `buttons_ack oid=%c count=%c` |
//! | MCU → host | `buttons_state oid=%c ack_count=%c state=%*s` |
//!
//! The firmware's side is `src/buttons.c`. `config_buttons` allocates the oid
//! and its pin slots (at most eight, `src/buttons.c:63-64`); `buttons_add` fills
//! one slot with a pin number and its pull-up (`int8_t`: `1` up, `-1` down,
//! `0` none — sent here as the same byte); `buttons_query` starts the polling
//! timer at `clock`, every `rest_ticks`, and seeds the debounce with `invert` so
//! the first report is compared against the right idle level
//! (`src/buttons.c:101-115`).
//!
//! A report carries one byte per **debounced** change, each byte a level per
//! pin, and the firmware's own 8-bit message counter in `ack_count`. The host
//! answers each report with `buttons_ack` and that many bytes; the counter is
//! what tells a retransmission (the firmware repeats an unreported change every
//! `retransmit_count` polls, `src/buttons.c:45-53`) from new data.

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_buttons oid=%c button_count=%c` — allocate the object and its pin
/// slots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigButtons {
    /// The object id the firmware allocated for this query.
    pub oid: u8,
    /// How many pins the object polls; at most eight (`src/buttons.c:63`).
    pub button_count: u8,
}

impl McuCommand for ConfigButtons {
    const NAME: &'static str = "config_buttons";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.button_count),
        ]
    }
}

/// `buttons_add oid=%c pos=%c pin=%u pull_up=%c` — one pin of the object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ButtonsAdd {
    /// The object id.
    pub oid: u8,
    /// The pin's slot in the report byte, `0` being the least significant bit.
    pub pos: u8,
    /// Numeric pin (the `pin` enumeration value).
    pub pin: u32,
    /// Pull-up as `gpio_in_setup`'s `int8_t` takes it: `1` up, `-1` down (`0xff`
    /// on the wire), `0` none.
    pub pull_up: u8,
}

impl McuCommand for ButtonsAdd {
    const NAME: &'static str = "buttons_add";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.pos),
            ArgValue::UInt32(self.pin),
            ArgValue::UInt8(self.pull_up),
        ]
    }
}

/// `buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c` —
/// arm the polling timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ButtonsQuery {
    /// The object id.
    pub oid: u8,
    /// Absolute firmware clock of the first poll.
    pub clock: u32,
    /// Time between polls, in clock ticks.
    pub rest_ticks: u32,
    /// How many polls an unacknowledged change is repeated for.
    pub retransmit_count: u8,
    /// The idle levels, bit per pin, so the firmware's debounce starts from the
    /// state the host expects (`!`-inverted pins included).
    pub invert: u8,
}

impl McuCommand for ButtonsQuery {
    const NAME: &'static str = "buttons_query";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.clock),
            ArgValue::UInt32(self.rest_ticks),
            ArgValue::UInt8(self.retransmit_count),
            ArgValue::UInt8(self.invert),
        ]
    }
}

/// `buttons_ack oid=%c count=%c` — the host consumed `count` report bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ButtonsAck {
    /// The object id.
    pub oid: u8,
    /// How many bytes of the last report were consumed.
    pub count: u8,
}

impl McuCommand for ButtonsAck {
    const NAME: &'static str = "buttons_ack";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.count)]
    }
}

/// `buttons_state oid=%c ack_count=%c state=%*s` — one report.
///
/// `ack_count` is the firmware's 8-bit count of the bytes the host has
/// acknowledged, which is how a retransmission is told from new data; `state`
/// holds one byte per debounced change, least significant bit first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ButtonsState {
    /// The object id.
    pub oid: u8,
    /// The firmware's 8-bit acknowledgement counter at the time of the report.
    pub ack_count: u8,
    /// The reported states, oldest first.
    pub state: Vec<u8>,
}

impl McuResponse for ButtonsState {
    const NAME: &'static str = "buttons_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            ack_count: params.get_u8("ack_count")?,
            state: params.get_bytes("state")?,
        })
    }
}

impl crate::core::klippy::event::McuEvent for ButtonsState {
    const NAME: &'static str = "buttons_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        <Self as McuResponse>::decode(params)
    }
}
