//! `shutdown` / `is_shutdown` / `starting` — the firmware stopping or restarting.
//!
//! These are the messages that explain why a real MCU stopped. Without them a
//! host that sent a bad configuration, or whose MCU restarted underneath it,
//! looks exactly like a host that is working: the printer stays `ready` and the
//! client sees nothing. Upstream registers all three in `MCUConnectHelper`
//! (`klippy/mcu.py:880-881` `:875`) and turns each into a printer shutdown.
//!
//! | Direction | Message | When |
//! |---|---|---|
//! | MCU → host | `shutdown clock=%u static_string_id=%hu` | the firmware ran a shutdown handler (`src/sched.c:310`) |
//! | MCU → host | `is_shutdown static_string_id=%hu` | a stopped firmware reports its reason (`src/sched.c:318`) |
//! | MCU → host | `starting` | the firmware rebooted on its own (`src/sched.c:351`) |
//!
//! `static_string_id` is an enumeration: the firmware sends a number and the
//! dictionary maps it to the message text, which is why these are events from
//! the dictionary's `responses` table rather than host-side constants.
//!
//! # What is reported
//!
//! The event carries the decoded reason; turning it into a printer shutdown is
//! the caller's (`mcu/object.rs`), because a resource module does not own the
//! machine. Upstream keeps the reason in the shutdown *details* and fires
//! `klippy:analyze_shutdown`; this host has no payload-carrying events yet (TODO
//! Q3), so the reason goes into the state message instead — the same visibility,
//! a different place.

use crate::core::klippy::cmd::Params;
use crate::core::klippy::event::McuEvent;
use crate::core::klippy::mcu::McuError;

/// `shutdown clock=%u static_string_id=%hu` — the firmware stopped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Shutdown {
    /// The shutdown reason, resolved through the `static_string_id` enumeration.
    pub reason: String,
    /// The firmware clock at the moment of the shutdown, when the message
    /// carried one.
    pub clock: Option<u32>,
}

impl McuEvent for Shutdown {
    const NAME: &'static str = "shutdown";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            reason: params.get_enum("static_string_id", "static_string_id")?,
            clock: optional_u32(params, "clock")?,
        })
    }
}

/// `is_shutdown static_string_id=%hu` — a firmware that was already stopped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IsShutdown {
    /// The shutdown reason, as [`Shutdown::reason`].
    pub reason: String,
}

impl McuEvent for IsShutdown {
    const NAME: &'static str = "is_shutdown";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            reason: params.get_enum("static_string_id", "static_string_id")?,
        })
    }
}

/// `starting` — the firmware restarted on its own.
///
/// A restart drops whatever it was configured with, so the host has to treat it
/// as a stop: the configuration it believes is loaded is gone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Starting;

impl McuEvent for Starting {
    const NAME: &'static str = "starting";

    fn decode(_params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self)
    }
}

/// Read an optional `%u` parameter.
fn optional_u32(params: &Params<'_>, name: &str) -> Result<Option<u32>, McuError> {
    match params.value(name) {
        Some(_) => Ok(Some(params.get_u32(name)?)),
        None => Ok(None),
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
    use crate::core::klippy::msg::proto::ArgValue;
    use serde_json::json;
    use std::sync::Arc;

    /// A dictionary shaped like real firmware's: the three messages plus the
    /// `static_string_id` enumeration that turns their numbers into text.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "responses": {
                "shutdown clock=%u static_string_id=%hu": 20,
                "is_shutdown static_string_id=%hu": 21,
                "starting": 22
            },
            "enumerations": {
                "static_string_id": {
                    "Move queue overflow": 0,
                    "Invalid pin": 1
                }
            }
        }))
        .unwrap()
    }

    /// Decode one message from its wire values.
    fn decode<E: McuEvent>(name: &str, values: &[ArgValue]) -> Result<E, McuError> {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        let encoded = parser.encode(name, values).unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let (msg, params) = &decoded[0];
        E::decode(&Params::new(Arc::clone(msg), params).with_dictionary(Arc::new(dictionary())))
    }

    #[test]
    fn test_shutdown_resolves_its_reason_and_clock() {
        let event: Shutdown =
            decode("shutdown", &[ArgValue::UInt32(1234), ArgValue::UInt16(1)]).unwrap();

        assert_eq!(
            event,
            Shutdown {
                reason: "Invalid pin".to_string(),
                clock: Some(1234),
            }
        );
    }

    #[test]
    fn test_shutdown_without_a_clock_is_still_readable() {
        // `is_shutdown` has no clock; the same shape must tolerate it.
        let event: Shutdown = decode("is_shutdown", &[ArgValue::UInt16(0)]).unwrap();

        assert_eq!(
            event,
            Shutdown {
                reason: "Move queue overflow".to_string(),
                clock: None,
            }
        );
    }

    #[test]
    fn test_is_shutdown_and_starting_decode() {
        let event: IsShutdown = decode("is_shutdown", &[ArgValue::UInt16(1)]).unwrap();
        assert_eq!(event.reason, "Invalid pin");

        let event: Starting = decode("starting", &[]).unwrap();
        assert_eq!(event, Starting);
    }

    #[test]
    fn test_an_unknown_reason_is_reported_not_guessed() {
        // The enumeration has no entry for 7; the message says so rather than
        // inventing text.
        let event: IsShutdown = decode("is_shutdown", &[ArgValue::UInt16(7)]).unwrap();

        assert_eq!(event.reason, "?7");
    }
}
