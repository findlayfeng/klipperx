//! Command modules — the layer that actually speaks the MCU protocol.
//!
//! This module is two things:
//!
//! * the vocabulary every command is written in — [`McuCommand`] and
//!   [`McuResponse`] for the two directions, [`Params`] for reading response
//!   parameters by name, and the typed calls [`Mcu::send_msg`] /
//!   [`Mcu::call_msg`] that run them;
//! * the command modules themselves: the bootstrap pair in `identify`, the
//!   base infrastructure commands in `allocate_oids` / `config` / `uptime` /
//!   `shutdown`, and `clock` (`get_clock` ↔ `clock`).
//!
//! The transport in [`mcu`](crate::core::klippy::mcu) owns frames, the data
//! dictionary, and the *bare* pair [`Mcu::send`](crate::core::klippy::mcu::Mcu::send) /
//! [`Mcu::call`](crate::core::klippy::mcu::Mcu::call), which take message names as
//! strings. Everything that names a message in the type system — its definition,
//! its parameters, its typed call — lives here.
//!
//! # Reading parameters by name
//!
//! [`Params`] is why this layer is worth having: the parameter order is chosen by
//! the firmware, so positional indexing would silently break whenever a firmware
//! revision reorders or inserts a parameter. The declared types are still
//! validated — [`Params`] converts each value to the requested type and reports a
//! mismatch instead of truncating.
//!
//! # Why a separate layer
//!
//! Three responsibilities are kept apart on purpose, one module each:
//!
//! | Layer | Owns | Knows about |
//! |---|---|---|
//! | [`msg`](crate::core::klippy::msg) | format strings ↔ bytes | nothing but the codec |
//! | [`mcu`](crate::core::klippy::mcu) | frames, the data dictionary, bare named access | only the identify pair |
//! | `cmd` | *which* messages exist and what they mean | the firmware protocol |
//!
//! `cmd` uses `mcu`; the frame, dictionary, and bare-call code over there mentions
//! no command module. The bootstrap in [`identify`](crate::core::klippy::identify)
//! is the one exchange that points the other way, and only because it has to: it
//! is the one command whose formats the host owns and it runs before any
//! dictionary exists (see below).
//!
//! Identify's transfer is therefore not here, but its definition is: the
//! `identify` / `identify_response` pair is the bootstrap exchange that produces
//! the dictionary, so its formats are host-owned and its chunk loop lives in
//! [`identify`](crate::core::klippy::identify). What stays in the command layer is
//! the part every command has — the typed view and the arguments it sends.
//!
//! Because the host learns every format from the firmware, a command never
//! hard-codes a format string or a wire id: it names the message and its
//! parameters, and the dictionary supplies the rest.
//!
//! # Adding a command module
//!
//! Traits here return `impl Future<…> + Send` rather than using `async fn`, so
//! the returned future is usable from spawned tasks and the `Send` bound is part
//! of the signature instead of an implicit, lint-flagged assumption.
//!
//! Modules are constructed from an `Arc<Mcu>` — the shared handle returned by
//! [`Mcu::connect`](crate::core::klippy::mcu::Mcu::connect) — so several modules
//! can use one MCU, and dropping the last handle shuts the device down.

pub mod adc;
pub mod ads1220;
pub mod ads131m0x;
pub mod adxl345;
pub mod allocate_oids;
pub mod bmi160;
pub mod buttons;
pub mod clock;
pub mod config;
pub mod debug;
pub mod ds18b20;
pub mod endstop;
pub mod gpio;
pub mod hx71x;
pub mod i2c;
pub mod icm20948;
pub mod identify;
pub mod ldc1612;
pub mod mpu9250;
pub mod pwm;
pub mod shutdown;
pub mod sos_filter;
pub mod spi;
pub mod stepper;
pub mod thermocouple;
pub mod trigger_analog;
pub mod trsync;
pub mod uart;
pub mod uptime;

pub use adc::{AnalogInState, AnalogInStateOld, ConfigAnalogIn, QueryAnalogIn, QueryAnalogInOld};
pub use bmi160::{ConfigBmi160, QueryBmi160, QUERY_BMI160_STATUS};
pub use buttons::{ButtonsAck, ButtonsAdd, ButtonsQuery, ButtonsState, ConfigButtons};
pub use clock::{ClockState, ClockSync, GetClock, McuClock, SecondarySync};
pub use debug::{DebugRead, DebugResult, DebugWrite};
pub use endstop::{ConfigEndstop, EndstopHome, EndstopQueryState, EndstopState};
pub use gpio::{ConfigDigitalOut, QueueDigitalOut, SetDigitalOutPwmCycle, UpdateDigitalOut};
pub use i2c::{
    ConfigI2c, I2cBusStatus, I2cRead, I2cReadResponse, I2cResponse, I2cSetBus, I2cSetSoftwareBus,
    I2cSetSwBus, I2cTransfer, I2cWrite, SoftwareI2cBus,
};
pub use icm20948::{ConfigIcm20948, QueryIcm20948, QUERY_ICM20948_STATUS};
pub use ldc1612::{
    ConfigLdc1612, ConfigLdc1612WithIntb, Ldc1612AttachTriggerAnalog, QueryLdc1612,
    QueryStatusLdc1612, SensorBulkData, SensorBulkStatus,
};
pub use mpu9250::{ConfigMpu9250, QueryMpu9250, QUERY_MPU9250_STATUS};
pub use pwm::{ConfigPwmOut, QueuePwmOut};
pub use sos_filter::{
    ConfigSosFilter, SosFilterSetActive, SosFilterSetOffsetScale, SosFilterSetSection,
    SosFilterSetState,
};
pub use spi::{
    ConfigSpi, ConfigSpiShutdown, ConfigSpiWithoutCs, SoftwareSpiBus, SpiSend, SpiSetBus,
    SpiSetSoftwareBus, SpiSetSwBus, SpiTransfer, SpiTransferResponse,
};
pub use stepper::{
    ConfigStepper, QueueStep, ResetStepClock, SetNextStepDir, StepperGetPosition, StepperPosition,
    StepperStopOnTrigger,
};
pub use thermocouple::{
    ConfigThermocouple, QueryThermocouple, ThermocoupleResult, ThermocoupleType,
};
pub use trigger_analog::{
    ConfigTriggerAnalog, TriggerAnalogHome, TriggerAnalogQueryState, TriggerAnalogSetRawRange,
    TriggerAnalogSetTrigger, TriggerAnalogState, TriggerAnalogType, REASON_TRIGGER_ANALOG,
};
pub use trsync::{
    ConfigTrsync, TriggerReason, TrsyncSetTimeout, TrsyncStart, TrsyncState, TrsyncTrigger,
};
pub use uart::{ConfigBridgePort, ConfigCan, ConfigUart, PortConfig, DEFAULT_UART_BAUD};

use crate::core::klippy::mcu::{Dictionary, Enumeration, Mcu, McuError, SendClocks};
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
        let value = self.value(name).ok_or_else(|| {
            McuError::Decode(format!(
                "'{}' parameter '{}' is missing",
                self.msg.name, name
            ))
        })?;
        value.try_convert_to(target).ok_or_else(|| {
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

    /// [`Mcu::send_msg`] with scheduling gates ([`SendClocks`]).
    ///
    /// The gates are the homing paths' own: `endstop_home` carries upstream's
    /// `reqclock` (`klippy/mcu.py:385`), so the arm lands behind the queue_step
    /// messages that must reach the board first (commit `6bd5f4e4`).
    /// (`cmd/mod.rs` is where the typed entry points live; putting the clocked
    /// twin here keeps one contract for both instead of splitting the decode
    /// and dictionary checks across two files.)
    ///
    /// # Errors
    /// As [`Mcu::send_msg`].
    pub(crate) fn send_msg_clocked<C: McuCommand>(
        &self,
        cmd: &C,
        clocks: SendClocks,
    ) -> Result<(), McuError> {
        self.require_dictionary()?;
        self.require_message(C::NAME)?;
        self.send_with_clocks(C::NAME, &cmd.args(), clocks)?;
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
        self.call_typed::<C, R>(cmd, None, timeout, Some(dictionary), SendClocks::default())
            .await
    }

    /// [`Mcu::call_msg`] whose response must carry `oid` — upstream's
    /// `serial.register_response(cb, name, oid)`.
    ///
    /// The TMC UART and SPI layers query several chips over one MCU; every
    /// answer arrives under the same response name and the `oid` is what says
    /// which chip it belongs to. A response carrying another `oid` does not
    /// resolve this call (it is dropped), so concurrent queries on one
    /// connection cannot take each other's data.
    ///
    /// # Errors
    /// As [`Mcu::call_msg`].
    pub async fn call_msg_oid<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        oid: u8,
        timeout: Duration,
    ) -> Result<R, McuError> {
        let dictionary = self.require_dictionary()?;
        self.call_typed::<C, R>(
            cmd,
            Some(oid),
            timeout,
            Some(dictionary),
            SendClocks::default(),
        )
        .await
    }

    /// Two commands in **one** wire block, one acknowledgement, and the second
    /// command's typed answer: upstream's `CommandQueryWrapper.send_with_preface`
    /// (`klippy/mcu.py:104-111`).
    ///
    /// Upstream's SPI form is `MCU_SPI.spi_transfer_with_preface`
    /// (`klippy/extras/bus.py:117-122`): the preface goes to `spi_send` and the
    /// read to `spi_transfer`, and both are encoded into one send — so the pair
    /// is one block, one ack, and the messages are still dispatched one at a
    /// time by the firmware (`src/spicmds.c:113-132`), one chip-select pulse
    /// each.
    ///
    /// The preface has to be a command the firmware answers **nothing** to, or
    /// its answer would arrive first and be the one this call decodes.
    ///
    /// One block is one frame, so the two messages must together fit
    /// `MESSAGE_PAYLOAD_MAX` ([`Payload::try_merge`]); a longer pair is refused
    /// instead of being split into two blocks with two acknowledgements.
    ///
    /// # Errors
    /// As [`Mcu::call_msg`]: [`McuError::NotIdentified`] before the handshake,
    /// [`McuError::UnknownMessage`] for a name the firmware does not declare,
    /// [`McuError::Msg`] when either message or the merged block cannot be
    /// encoded, [`McuError::Call`] when the exchange fails or times out, and
    /// [`McuError::Decode`] when the answer cannot be decoded.
    pub(crate) async fn call_two<A: McuCommand, B: McuCommand, R: McuResponse>(
        &self,
        first: &A,
        second: &B,
        oid: u8,
        timeout: Duration,
    ) -> Result<R, McuError> {
        let dictionary = self.require_dictionary()?;
        self.require_message(A::NAME)?;
        self.require_message(B::NAME)?;
        let msg = self.require_message(R::NAME)?;

        let mut block = self.encode(A::NAME, &first.args())?;
        block.try_merge(&self.encode(B::NAME, &second.args())?)?;
        let describe = format!(
            "{}, {}",
            self.describe_command(A::NAME, &first.args()),
            self.describe_command(B::NAME, &second.args())
        );

        let values = self
            .call_gated_block(
                block,
                &describe,
                Some(oid),
                R::NAME,
                timeout,
                SendClocks::default(),
            )
            .await?;

        let params = Params::new(msg, &values).with_dictionary(dictionary);
        R::decode(&params)
    }

    /// [`Mcu::call_msg`] with scheduling gates on the request ([`SendClocks`]).
    ///
    /// `endstop_query_state` carries upstream's `minclock`
    /// (`klippy/mcu.py:401-405`): the query waits until the queued motion has
    /// run, so the answer is the pin at the requested print time rather than
    /// whenever the message happened to arrive.
    ///
    /// # Errors
    /// As [`Mcu::call_msg`].
    pub(crate) async fn call_msg_clocked<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        clocks: SendClocks,
        timeout: Duration,
    ) -> Result<R, McuError> {
        let dictionary = self.require_dictionary()?;
        self.call_typed::<C, R>(cmd, None, timeout, Some(dictionary), clocks)
            .await
    }

    /// [`Mcu::call_msg_clocked`] whose response must also carry `oid`.
    ///
    /// The TMC UART needs both at once: several chips share one MCU, and a
    /// write carries the `minclock` gate of the print time it was asked for.
    ///
    /// # Errors
    /// As [`Mcu::call_msg`].
    pub(crate) async fn call_msg_oid_clocked<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        oid: u8,
        clocks: SendClocks,
        timeout: Duration,
    ) -> Result<R, McuError> {
        let dictionary = self.require_dictionary()?;
        self.call_typed::<C, R>(cmd, Some(oid), timeout, Some(dictionary), clocks)
            .await
    }

    /// Typed call that also works **before** the identify handshake.
    ///
    /// The identify exchange is the only one that precedes the dictionary, so
    /// this exists for
    /// [`Identify::fetch`](crate::core::klippy::identify::Identify::fetch) alone; every other
    /// caller must use [`Mcu::call_msg`] so that a missing handshake is reported
    /// instead of silently attempting a command the parser does not know yet.
    pub(crate) async fn call_msg_ungated<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        timeout: Duration,
    ) -> Result<R, McuError> {
        self.call_typed::<C, R>(cmd, None, timeout, None, SendClocks::default())
            .await
    }

    /// Shared body of the typed calls.
    ///
    /// Both names are resolved before the command is sent, so a message the
    /// firmware does not implement fails immediately instead of waiting for
    /// `timeout`. `oid` is `Some` only for [`Mcu::call_msg_oid`].
    async fn call_typed<C: McuCommand, R: McuResponse>(
        &self,
        cmd: &C,
        oid: Option<u8>,
        timeout: Duration,
        dictionary: Option<Arc<Dictionary>>,
        clocks: SendClocks,
    ) -> Result<R, McuError> {
        self.require_message(C::NAME)?;
        let msg = self.require_message(R::NAME)?;

        let values = self
            .call_gated_oid(C::NAME, &cmd.args(), oid, R::NAME, timeout, clocks)
            .await?;

        let params = match dictionary {
            Some(dictionary) => Params::new(msg, &values).with_dictionary(dictionary),
            None => Params::new(msg, &values),
        };
        R::decode(&params)
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
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, MappingEntry};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::McuCallError;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::Payload;
    use serde_json::json;

    // -----------------------------------------------------------------------
    // Test dictionary and messages
    // -----------------------------------------------------------------------

    /// The subset of a firmware dictionary these tests need.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "get_clock": 5,
                "get_uptime": 4,
                "clear_shutdown": 2,
                "debug_ping data=%*s": 10
            },
            "responses": {
                "clock clock=%u": 18,
                "uptime high=%u clock=%u": 17
            },
            "enumerations": {
                "static_string_id": {"Timer too close": 3}
            }
        }))
        .unwrap()
    }

    /// `get_uptime` / `uptime high=%u clock=%u` — two parameters, so the
    /// round-trip exercises more than a single name lookup.
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

    /// `debug_ping data=%*s` — only used to send a payload big enough that the
    /// send task cannot merge it with the next frame.
    struct DebugPing(String);

    impl McuCommand for DebugPing {
        const NAME: &'static str = "debug_ping";
        fn args(&self) -> Vec<ArgValue> {
            vec![ArgValue::Str(self.0.clone())]
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

    /// Frame carrying `parts` as the message id followed by its arguments.
    fn frame(seq: u8, parts: &[ArgValue]) -> Frame {
        Frame::new(seq, payload(parts))
    }

    fn mcu_with(dictionary: Dictionary, mappings: Vec<MappingEntry>) -> Mcu {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(mappings)));
        // A fresh `Mcu` registers the identify pair, so the dictionary entries
        // install cleanly on top.
        mcu.install_dictionary(dictionary).unwrap();
        mcu
    }

    /// Decoded parameters of a one-off message, for `Params` tests that need no
    /// device.
    fn params_for(format: &str, id: i16, values: &[ArgValue]) -> (Parser, Vec<ArgValue>) {
        let mut parser = Parser::new();
        parser.register(id, format).unwrap();
        let encoded = parser
            .encode(format.split_whitespace().next().unwrap(), values)
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        (parser, decoded[0].1.clone())
    }

    /// `Params` over `is_shutdown static_string_id=%hu` values, with the fixture
    /// enumeration attached unless `with_dictionary` is false.
    fn shutdown_params(values: &[ArgValue], with_dictionary: bool) -> Params<'_> {
        let mut parser = Parser::new();
        parser
            .register(14, "is_shutdown static_string_id=%hu")
            .unwrap();
        let params = Params::new(parser.lookup("is_shutdown").unwrap(), values);
        if with_dictionary {
            params.with_dictionary(Arc::new(dictionary()))
        } else {
            params
        }
    }

    // -----------------------------------------------------------------------
    // Params
    // -----------------------------------------------------------------------

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

        // The declaration is readable as the firmware gave it.
        let declared: Vec<(&str, ArgType)> = params
            .declared()
            .iter()
            .map(|(name, atype)| (name.as_str(), *atype))
            .collect();
        assert_eq!(
            declared,
            vec![("clock", ArgType::UInt32), ("high", ArgType::UInt32)]
        );
    }

    #[test]
    fn test_params_of_a_message_without_parameters() {
        let (parser, values) = params_for("clear_shutdown", 2, &[]);
        let params = Params::new(parser.lookup("clear_shutdown").unwrap(), &values);

        assert_eq!(params.message_name(), "clear_shutdown");
        assert_eq!(params.len(), 0);
        assert!(params.is_empty());
        assert!(params.declared().is_empty());
        assert!(!params.has("anything"));
    }

    #[test]
    fn test_params_conversions_widen_and_range_check() {
        let (parser, values) = params_for(
            "state a=%c b=%hi c=%hu",
            1,
            &[
                ArgValue::UInt8(5),
                ArgValue::Int16(-2),
                ArgValue::UInt16(4000),
            ],
        );
        let params = Params::new(parser.lookup("state").unwrap(), &values);

        // Widening and exact reads both go through the typed accessors.
        assert_eq!(params.get_u32("a").unwrap(), 5);
        assert_eq!(params.get_i32("b").unwrap(), -2);
        assert_eq!(params.get_i16("b").unwrap(), -2);
        assert_eq!(params.get_u16("c").unwrap(), 4000);

        // Narrowing a negative value is refused, not wrapped.
        assert!(matches!(params.get_u8("b"), Err(McuError::Decode(_))));
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
    fn test_params_get_enum_names_values_and_falls_back() {
        let values = vec![ArgValue::UInt16(3)];
        let named = shutdown_params(&values, true);
        assert_eq!(
            named
                .get_enum("static_string_id", "static_string_id")
                .unwrap(),
            "Timer too close"
        );

        // A value the firmware did not name renders as `?<value>`, like Klipper.
        let values = vec![ArgValue::UInt16(99)];
        let unnamed = shutdown_params(&values, true);
        assert_eq!(
            unnamed
                .get_enum("static_string_id", "static_string_id")
                .unwrap(),
            "?99"
        );
    }

    #[test]
    fn test_params_get_enum_reports_missing_dictionary_and_enumeration() {
        let values = vec![ArgValue::UInt16(3)];

        let without = shutdown_params(&values, false);
        let err = without
            .get_enum("static_string_id", "static_string_id")
            .unwrap_err();
        assert!(err.to_string().contains("without a dictionary"), "{err}");

        let unknown = shutdown_params(&values, true);
        let err = unknown
            .get_enum("nonexistent", "static_string_id")
            .unwrap_err();
        assert!(
            err.to_string().contains("no enumeration 'nonexistent'"),
            "{err}"
        );
    }

    // -----------------------------------------------------------------------
    // Mcu::send_msg / Mcu::call_msg
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_send_msg_before_identify_fails() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(Vec::new())));

        assert!(!mcu.is_identified());
        let err = mcu.send_msg(&GetClock).unwrap_err();
        assert!(matches!(err, McuError::NotIdentified));
    }

    #[tokio::test]
    async fn test_send_msg_puts_the_command_on_the_wire() {
        // 45 bytes of data push the payload past the batching threshold, so it
        // leaves as its own frame instead of being merged into the next one.
        let data = "x".repeat(45);
        let mappings = vec![
            MappingEntry {
                input: frame(0, &[ArgValue::UInt8(10), ArgValue::Str(data.clone())]),
                outputs: Vec::new(),
            },
            MappingEntry {
                input: frame(1, &[ArgValue::UInt8(5)]),
                // The first command has no response, so this is the first frame
                // the receive task ever sees: it counts received frames, so the
                // seq has to be 0 even though the request went out as seq 1.
                outputs: vec![frame(0, &[ArgValue::UInt8(18), ArgValue::UInt32(0x1234)])],
            },
        ];
        let mcu = mcu_with(dictionary(), mappings);

        mcu.send_msg(&DebugPing(data)).unwrap();

        // The device compares frames in FIFO order, so this response can only
        // arrive if the typed send really put `debug_ping` on the wire first.
        let state = mcu
            .call_msg::<GetClock, ClockState>(&GetClock, Duration::from_secs(1))
            .await
            .unwrap();
        assert_eq!(state.clock, 0x1234);
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
        // `get_uptime` encodes to its firmware id (4); the response is
        // `uptime high=%u clock=%u` (id 17), decoded by name.
        let mappings = vec![MappingEntry {
            input: frame(0, &[ArgValue::UInt8(4)]),
            outputs: vec![frame(
                0,
                &[
                    ArgValue::UInt8(17),
                    ArgValue::UInt32(1),
                    ArgValue::UInt32(0x1234),
                ],
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
                clock: 0x1234
            }
        );
    }

    #[tokio::test]
    async fn test_call_msg_times_out_when_mcu_stays_silent() {
        // Nothing is mapped, so the device never answers. Both names are known,
        // so the call must reach the timeout path.
        let mcu = mcu_with(dictionary(), Vec::new());

        let err = mcu
            .call_msg::<GetClock, ClockState>(&GetClock, Duration::from_millis(50))
            .await
            .unwrap_err();

        match err {
            McuError::Call(McuCallError::Timeout(msg)) => {
                assert!(msg.contains("no response for clock"), "{msg}");
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
            input: frame(0, &[ArgValue::UInt8(5)]),
            outputs: vec![frame(0, &[ArgValue::UInt8(18), ArgValue::UInt32(1)])],
        }];
        let mcu = mcu_with(dictionary(), mappings);

        let err = mcu
            .call_msg::<GetClock, BadDecoder>(&GetClock, Duration::from_secs(1))
            .await
            .unwrap_err();

        assert!(matches!(err, McuError::Decode(_)), "{err:?}");
    }

    // -----------------------------------------------------------------------
    // Mcu::call_msg_oid
    // -----------------------------------------------------------------------

    /// `tmc_uart_read oid=%c read_len=%u` / `tmc_uart_read_response oid=%c
    /// data=%*s` — one command and one answer name for every chip on the bus.
    fn tmc_dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {"tmc_uart_read oid=%c read_len=%u": 70},
            "responses": {"tmc_uart_read_response oid=%c data=%*s": 71}
        }))
        .unwrap()
    }

    struct TmcUartRead {
        oid: u8,
        read_len: u32,
    }

    impl McuCommand for TmcUartRead {
        const NAME: &'static str = "tmc_uart_read";
        fn args(&self) -> Vec<ArgValue> {
            vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.read_len)]
        }
    }

    #[derive(Debug, PartialEq)]
    struct TmcUartReadResponse {
        oid: u8,
        data: Vec<u8>,
    }

    impl McuResponse for TmcUartReadResponse {
        const NAME: &'static str = "tmc_uart_read_response";
        fn decode(params: &Params<'_>) -> Result<Self, McuError> {
            Ok(Self {
                oid: params.get_u8("oid")?,
                data: params.get_bytes("data")?,
            })
        }
    }

    #[tokio::test]
    async fn test_call_msg_oid_roundtrip() {
        let mappings = vec![MappingEntry {
            input: frame(
                0,
                &[ArgValue::UInt8(70), ArgValue::UInt8(1), ArgValue::UInt32(4)],
            ),
            outputs: vec![frame(
                0,
                &[
                    ArgValue::UInt8(71),
                    ArgValue::UInt8(1),
                    ArgValue::Bytes(b"one".to_vec()),
                ],
            )],
        }];
        let mcu = mcu_with(tmc_dictionary(), mappings);

        let response = mcu
            .call_msg_oid::<TmcUartRead, TmcUartReadResponse>(
                &TmcUartRead {
                    oid: 1,
                    read_len: 4,
                },
                1,
                Duration::from_secs(1),
            )
            .await
            .unwrap();

        assert_eq!(
            response,
            TmcUartReadResponse {
                oid: 1,
                data: b"one".to_vec()
            }
        );
    }

    #[tokio::test]
    async fn test_call_msg_oid_does_not_decode_another_oid_s_answer() {
        // oid 2's data answers oid 1's query: wrong chip, so the typed call must
        // not decode it — it waits out its own timeout instead.
        let mappings = vec![MappingEntry {
            input: frame(
                0,
                &[ArgValue::UInt8(70), ArgValue::UInt8(1), ArgValue::UInt32(4)],
            ),
            outputs: vec![frame(
                0,
                &[
                    ArgValue::UInt8(71),
                    ArgValue::UInt8(2),
                    ArgValue::Bytes(b"two".to_vec()),
                ],
            )],
        }];
        let mcu = mcu_with(tmc_dictionary(), mappings);

        let err = mcu
            .call_msg_oid::<TmcUartRead, TmcUartReadResponse>(
                &TmcUartRead {
                    oid: 1,
                    read_len: 4,
                },
                1,
                Duration::from_millis(50),
            )
            .await
            .unwrap_err();

        match err {
            McuError::Call(McuCallError::Timeout(msg)) => {
                assert!(
                    msg.contains("no response for tmc_uart_read_response"),
                    "{msg}"
                );
            }
            other => panic!("expected a timeout, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn test_call_msg_oid_times_out_when_mcu_stays_silent() {
        let mcu = mcu_with(tmc_dictionary(), Vec::new());

        let err = mcu
            .call_msg_oid::<TmcUartRead, TmcUartReadResponse>(
                &TmcUartRead {
                    oid: 5,
                    read_len: 1,
                },
                5,
                Duration::from_millis(50),
            )
            .await
            .unwrap_err();

        assert!(
            matches!(err, McuError::Call(McuCallError::Timeout(_))),
            "{err:?}"
        );
    }

    // -----------------------------------------------------------------------
    // Mcu::call_two
    // -----------------------------------------------------------------------

    /// `spi_send` (answers nothing) and `spi_transfer` (answers once) — the pair
    /// upstream's `spi_transfer_with_preface` encodes into one block
    /// (`klippy/extras/bus.py:117-122`, `klippy/mcu.py:104-111`).
    fn spi_pair_dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "spi_send oid=%c data=%*s": 54,
                "spi_transfer oid=%c data=%*s": 55
            },
            "responses": {"spi_transfer_response oid=%c response=%*s": 56}
        }))
        .unwrap()
    }

    struct SpiSendStub {
        oid: u8,
        data: Vec<u8>,
    }

    impl McuCommand for SpiSendStub {
        const NAME: &'static str = "spi_send";
        fn args(&self) -> Vec<ArgValue> {
            vec![
                ArgValue::UInt8(self.oid),
                ArgValue::Bytes(self.data.clone()),
            ]
        }
    }

    struct SpiTransferStub {
        oid: u8,
        data: Vec<u8>,
    }

    impl McuCommand for SpiTransferStub {
        const NAME: &'static str = "spi_transfer";
        fn args(&self) -> Vec<ArgValue> {
            vec![
                ArgValue::UInt8(self.oid),
                ArgValue::Bytes(self.data.clone()),
            ]
        }
    }

    #[derive(Debug, PartialEq)]
    struct SpiTransferStubResponse {
        oid: u8,
        response: Vec<u8>,
    }

    impl McuResponse for SpiTransferStubResponse {
        const NAME: &'static str = "spi_transfer_response";
        fn decode(params: &Params<'_>) -> Result<Self, McuError> {
            Ok(Self {
                oid: params.get_u8("oid")?,
                response: params.get_bytes("response")?,
            })
        }
    }

    /// ## One block, one ack
    ///
    /// The mapping's `input` is the **whole block** — both message ids, both
    /// sets of arguments, in one frame — and `FrameMock` refuses a frame that is
    /// not byte for byte that block. A second send would also find no mapping
    /// left, so one acknowledgement is all the exchange can consume.
    #[tokio::test]
    async fn test_call_two_sends_both_messages_as_one_block() {
        let preface = vec![0x9f];
        let read = vec![0x00, 0x00, 0x00];
        let answer = vec![0xef, 0x40, 0x17];
        let mappings = vec![MappingEntry {
            input: frame(
                0,
                &[
                    // `spi_send oid=%c data=%*s` (id 54).
                    ArgValue::UInt8(54),
                    ArgValue::UInt8(1),
                    ArgValue::Bytes(preface.clone()),
                    // `spi_transfer oid=%c data=%*s` (id 55) in the same frame.
                    ArgValue::UInt8(55),
                    ArgValue::UInt8(1),
                    ArgValue::Bytes(read.clone()),
                ],
            ),
            // Only the second message answers, so one response frame.
            outputs: vec![frame(
                0,
                &[
                    ArgValue::UInt8(56),
                    ArgValue::UInt8(1),
                    ArgValue::Bytes(answer.clone()),
                ],
            )],
        }];
        let mcu = mcu_with(spi_pair_dictionary(), mappings);

        let response = mcu
            .call_two::<SpiSendStub, SpiTransferStub, SpiTransferStubResponse>(
                &SpiSendStub {
                    oid: 1,
                    data: preface,
                },
                &SpiTransferStub { oid: 1, data: read },
                1,
                Duration::from_secs(1),
            )
            .await
            .unwrap();

        assert_eq!(
            response,
            SpiTransferStubResponse {
                oid: 1,
                response: answer
            }
        );
        assert_eq!(
            mcu.pending_call_count().await,
            0,
            "the one answer consumed the one registration"
        );
    }

    /// A block nobody answers times out like a single call, and hands its
    /// registration back — a leftover would take the *next* answer for that
    /// response name.
    #[tokio::test]
    async fn test_call_two_times_out_and_leaves_no_registration() {
        let mcu = mcu_with(spi_pair_dictionary(), Vec::new());

        let err = mcu
            .call_two::<SpiSendStub, SpiTransferStub, SpiTransferStubResponse>(
                &SpiSendStub {
                    oid: 1,
                    data: vec![0x00],
                },
                &SpiTransferStub {
                    oid: 1,
                    data: vec![0x00, 0x00],
                },
                1,
                Duration::from_millis(50),
            )
            .await
            .unwrap_err();

        assert!(
            matches!(err, McuError::Call(McuCallError::Timeout(_))),
            "{err:?}"
        );
        assert_eq!(
            mcu.pending_call_count().await,
            0,
            "a timed-out call takes its registration with it"
        );
    }

    /// A block that never leaves the host cancels its registration too: this is
    /// the path where the registration exists before the send is attempted.
    #[tokio::test]
    async fn test_call_two_that_cannot_be_sent_leaves_no_registration() {
        let mcu = mcu_with(spi_pair_dictionary(), Vec::new());
        mcu.close();

        let err = mcu
            .call_two::<SpiSendStub, SpiTransferStub, SpiTransferStubResponse>(
                &SpiSendStub {
                    oid: 1,
                    data: vec![0x00],
                },
                &SpiTransferStub {
                    oid: 1,
                    data: vec![0x00, 0x00],
                },
                1,
                Duration::from_millis(50),
            )
            .await
            .unwrap_err();

        match &err {
            McuError::Call(McuCallError::SendFailed(msg)) => {
                assert!(msg.contains("the connection is closed"), "{msg}");
            }
            other => panic!("expected a send failure, got {other:?}"),
        }
        assert_eq!(
            mcu.pending_call_count().await,
            0,
            "a send that failed cancels the registration it made first"
        );
    }
}
