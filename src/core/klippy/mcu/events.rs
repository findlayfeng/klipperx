//! The per-MCU table of inbound message callbacks.
//!
//! Kept beside the [`Parser`] rather than inside it. The parser is the codec —
//! it maps message names and ids to encoders/decoders and is shared by the send
//! and receive paths; callbacks belong to whatever object bound them. Separating
//! the two is what lets the MCU's transport be held by resources without the
//! callbacks pulling the owning `Mcu` back in.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::msg::{MsgCallback, MsgError, MsgResult};

/// Inbound callbacks, keyed by the message id the firmware assigned.
///
/// The receive task looks a callback up by `msg.id`, so dispatch needs no name
/// comparison. Binding by name resolves through the [`Parser`], so a callback
/// can only be bound to a message the dictionary actually declared.
#[derive(Default)]
pub(crate) struct McuEvents {
    callbacks: Mutex<HashMap<i16, MsgCallback>>,
}

impl McuEvents {
    /// An empty table.
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Bind `callback` to the message `name`, which must be registered in
    /// `parser`. A later bind replaces an earlier one.
    ///
    /// # Errors
    /// [`MsgError`] when `name` is not a registered message.
    pub(crate) fn bind(
        &self,
        parser: &Parser,
        name: &str,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> MsgResult<()> {
        let msg = parser
            .lookup(name)
            .ok_or_else(|| MsgError::new(format!("Unknown command: {name}")))?;
        let callback: MsgCallback = Arc::new(Mutex::new(Box::new(callback)));
        self.callbacks
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert(msg.id, callback);
        Ok(())
    }

    /// The callback bound to `id`, if any.
    pub(crate) fn callback(&self, id: i16) -> Option<MsgCallback> {
        self.callbacks
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .get(&id)
            .cloned()
    }

    /// Whether the registered message `name` has a callback.
    pub(crate) fn has_callback(&self, parser: &Parser, name: &str) -> bool {
        parser
            .lookup(name)
            .map(|msg| {
                self.callbacks
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .contains_key(&msg.id)
            })
            .unwrap_or(false)
    }

    /// Drop every callback.
    ///
    /// Called when the machine's parts are torn down. A callback can hold a
    /// resource that itself holds the `Mcu` (for the transport), so leaving the
    /// table populated makes `Mcu → events → resource → Mcu` a strong cycle:
    /// `Mcu::Drop` never runs and its blocking device read parks forever,
    /// hanging runtime shutdown. Clearing here breaks that cycle.
    pub(crate) fn clear(&self) {
        self.callbacks
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clear();
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn parser() -> Parser {
        let mut parser = Parser::new();
        parser.register(1, "thing value=%u").unwrap();
        parser
    }

    #[test]
    fn test_a_callback_is_looked_up_by_id() {
        let parser = parser();
        let events = McuEvents::new();
        events.bind(&parser, "thing", |_| {}).unwrap();

        let id = parser.lookup("thing").unwrap().id;
        assert!(events.callback(id).is_some());
        assert!(events.callback(id + 1).is_none());
        assert!(events.has_callback(&parser, "thing"));
        assert!(!events.has_callback(&parser, "absent"));
    }

    #[test]
    fn test_binding_an_unregistered_message_fails() {
        let parser = parser();
        let events = McuEvents::new();

        let err = events.bind(&parser, "absent", |_| {}).unwrap_err();
        assert!(err.to_string().contains("Unknown command"), "{err}");
    }

    #[test]
    fn test_a_later_bind_replaces_the_callback() {
        let parser = parser();
        let events = McuEvents::new();
        let calls = Arc::new(Mutex::new(Vec::new()));
        let first = Arc::clone(&calls);
        events
            .bind(&parser, "thing", move |_| first.lock().unwrap().push(1))
            .unwrap();
        let second = Arc::clone(&calls);
        events
            .bind(&parser, "thing", move |_| second.lock().unwrap().push(2))
            .unwrap();

        let id = parser.lookup("thing").unwrap().id;
        let callback = events.callback(id).unwrap();
        callback.lock().unwrap()(&[ArgValue::UInt32(0)]);
        assert_eq!(*calls.lock().unwrap(), [2]);
    }
}
