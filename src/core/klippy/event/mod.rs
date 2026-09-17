//! Event messages — firmware-pushed messages with no request.
//!
//! Most of the protocol is request/response: the host sends a command and
//! [`call_msg`](crate::core::klippy::mcu::Mcu::call_msg) waits for the answer.
//! An event has no outbound half. The firmware sends it when something happens,
//! and the host has to be listening before it does. On the wire it is an
//! ordinary message — a `sendf` encoder, so it appears in the dictionary's
//! `responses` table and its format, id, and parameter names still come from
//! firmware — but it is consumed through the parser's per-message callback
//! rather than a pending call.
//!
//! This module is a peer of [`cmd`](crate::core::klippy::cmd), not a part of it:
//! a command answers the host, an event does not, and the two are registered and
//! delivered differently. The event layer builds on the command layer's
//! vocabulary ([`Params`], and the typed-call style of [`Mcu`]) the same way
//! `cmd` builds on `mcu`:
//!
//! ```text
//! msg  ←──  mcu  ←──  cmd  ←──  event
//! ```
//!
//! # Registering a handler
//!
//! [`McuEvent`] is the inbound counterpart of
//! [`McuResponse`](crate::core::klippy::cmd::McuResponse), and
//! [`Mcu::bind_event`] is the counterpart of `Mcu::call_msg`: it resolves the
//! name against the dictionary and installs a typed handler. Nothing is sent,
//! so there is no timeout and no pending call to leak.
//!
//! Events that the firmware declares with `output()` instead of `sendf()` live
//! in the dictionary's separate `output` table, which is parsed but not
//! registered, so those still cannot be delivered.
//!
//! # Modules
//!
//! | Module | Events |
//! |---|---|
//! | [`stats`] | `stats` — periodic scheduler timing from `stats_update` |

pub mod stats;

#[cfg(test)]
mod test_support;

pub use stats::Stats;

use crate::core::klippy::cmd::Params;
use crate::core::klippy::mcu::{Mcu, McuError};
use crate::core::klippy::msg::Msg;
use std::sync::Arc;
use tracing::error;

/// An inbound MCU message the firmware sends on its own.
///
/// An event has no outbound half: the firmware pushes it when something happens
/// — a timer fires, a pin changes — rather than answering a request. On the wire
/// it is an ordinary message (a `sendf` encoder, so it appears in the
/// dictionary's `responses` table), but it is consumed through the parser's
/// per-message callback instead of [`Mcu::call_msg`]. That way a handler never
/// has to own a request, and a missed event cannot leave a pending call behind.
///
/// Register a handler with [`Mcu::bind_event`].
pub trait McuEvent: Sized {
    /// Event name, which must match a `responses` entry of the dictionary.
    const NAME: &'static str;

    /// Build the typed event from the decoded parameters.
    ///
    /// # Errors
    /// Returns [`McuError::Decode`] when a parameter is missing or holds an
    /// unexpected type.
    fn decode(params: &Params<'_>) -> Result<Self, McuError>;
}

impl Mcu {
    /// Register a handler for a firmware-pushed event.
    ///
    /// The message must be in the installed dictionary. The handler runs on the
    /// receive task when the event arrives; it must not block. A pending
    /// [`Mcu::call_msg`] for the same response name takes priority, so the two
    /// cannot both consume one message, but the handler does run on the same
    /// task the synchronous calls are delivered from.
    ///
    /// A decode failure is logged and the event is dropped; it does not stop the
    /// receive task or unbind the handler.
    ///
    /// # Errors
    /// Returns [`McuError::NotIdentified`] before the identify handshake,
    /// [`McuError::UnknownMessage`] when the name is absent from the dictionary,
    /// and [`McuError::Msg`] when the callback cannot be bound.
    pub fn bind_event<E, F>(&self, mut handler: F) -> Result<(), McuError>
    where
        E: McuEvent,
        F: FnMut(E) + Send + 'static,
    {
        self.require_dictionary()?;
        let msg = self.require_message(E::NAME)?;

        // The callback is stored inside the registry's `Msg`, so capturing that
        // same `Arc` would make the message reference itself and never be freed.
        // A standalone copy of the declaration keeps the callback able to read
        // parameters by name without keeping the registry entry alive.
        let declaration = Arc::new(Msg::new(msg.id, msg.name.clone(), msg.params.clone()));
        let dictionary = self.dictionary();

        self.bind_callback(E::NAME, move |values| {
            let params = Params::new(Arc::clone(&declaration), values);
            let params = match &dictionary {
                Some(dictionary) => params.with_dictionary(Arc::clone(dictionary)),
                None => params,
            };
            match E::decode(&params) {
                Ok(event) => handler(event),
                Err(e) => error!("Failed to decode {} event: {e}", E::NAME),
            }
        })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::uptime::GetUptime;
    use crate::core::klippy::event::test_support::{frame, mcu};
    use crate::core::klippy::interface::test::TestDevice;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::msg::proto::ArgValue;
    use tokio::sync::mpsc;
    use tokio::time::{timeout, Duration};

    #[tokio::test]
    async fn test_bound_handler_receives_the_event() {
        // A `stats` report is unsolicited on the wire; the test device emits one
        // in reply to an unrelated command so there is a frame to receive.
        let mappings = vec![crate::core::klippy::interface::test::MappingEntry {
            input: frame(0, &[ArgValue::UInt8(4)]),
            outputs: vec![frame(
                0,
                &[
                    ArgValue::UInt8(12),
                    ArgValue::UInt32(5),
                    ArgValue::UInt32(100),
                    ArgValue::UInt32(2500),
                ],
            )],
        }];
        let mcu = mcu(mappings);
        let (tx, mut rx) = mpsc::unbounded_channel();
        mcu.bind_event::<Stats, _>(move |stats| {
            tx.send(stats).expect("receiver still open");
        })
        .unwrap();

        mcu.send_msg(&GetUptime).unwrap();

        let stats = timeout(Duration::from_secs(1), rx.recv())
            .await
            .expect("event must arrive")
            .expect("handler must run");
        assert_eq!(
            stats,
            Stats {
                count: 5,
                sum: 100,
                sumsq: 2500
            }
        );
    }

    #[tokio::test]
    async fn test_bind_event_before_identify_fails() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(Vec::new())));

        let err = mcu.bind_event::<Stats, _>(|_| {}).unwrap_err();

        assert!(matches!(err, McuError::NotIdentified), "{err:?}");
    }

    #[tokio::test]
    async fn test_bind_event_rejects_a_message_absent_from_the_dictionary() {
        /// An event the firmware does not implement.
        struct Unknown;
        impl McuEvent for Unknown {
            const NAME: &'static str = "no_such_event";
            fn decode(_params: &Params<'_>) -> Result<Self, McuError> {
                Ok(Self)
            }
        }

        let mcu = mcu(Vec::new());

        let err = mcu.bind_event::<Unknown, _>(|_| {}).unwrap_err();

        assert!(matches!(err, McuError::UnknownMessage(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_register_stats_logging_binds_a_handler() {
        let mcu = mcu(Vec::new());

        stats::register_stats_logging(&mcu).unwrap();
    }
}
