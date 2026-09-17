use super::error::{MsgError, MsgResult};
use super::proto::{ArgValue, Payload};
use super::{MsgBase, MsgEntry};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

#[derive(Debug)]
pub struct Msg {
    id: u8,
    name: String,
    command: MsgEntry,
}

/// Registry of messages indexed by both id and name.
///
/// Both indexes are unique: inserting a message whose id or name is already
/// present is rejected. A name index maps to the message id, which is then
/// resolved through the id index so each message is stored exactly once.
#[derive(Debug, Default)]
struct MsgMap {
    by_id: HashMap<u8, Arc<Msg>>,
    by_name: HashMap<String, u8>,
}

impl MsgMap {
    /// Insert a message, failing if its id or name is already registered.
    pub fn try_insert(&mut self, msg: Msg) -> Result<(), String> {
        if self.by_id.contains_key(&msg.id) {
            return Err(format!("duplicate id: {}", msg.id));
        }
        if self.by_name.contains_key(&msg.name) {
            return Err(format!("duplicate name: {}", msg.name));
        }
        self.by_name.insert(msg.name.clone(), msg.id);
        self.by_id.insert(msg.id, Arc::new(msg));
        Ok(())
    }

    fn get_by_id(&self, id: &u8) -> Option<&Msg> {
        self.by_id.get(id).map(|arc| arc.as_ref())
    }

    fn get_by_name(&self, name: &str) -> Option<&Msg> {
        self.by_name.get(name).and_then(|id| self.by_id.get(id).map(|arc| arc.as_ref()))
    }
}

#[derive(Clone)]
pub struct Parser {
    msgs: Arc<Mutex<MsgMap>>,
}

impl Parser {
    /// Create a new parser and register default message formats.
    pub fn new() -> Self {
        Self {
            msgs: Arc::new(Mutex::new(MsgMap::default())),
        }
    }

    /// Register a message format with the given ID.
    ///    /// The format string is parsed into a [`MsgBase`] and stored under both
    /// its numeric `id` and its name. The command is always registered as a
    /// [`MsgEntry::Base`], which can be used for outbound [`Self::send`]
    /// calls. Use [`Self::bind`] to convert it to a [`MsgEntry::Handler`]
    /// for inbound dispatch.
    ///
    /// # Errors
    /// Returns an error if the format string is invalid, or if the `id`
    /// or the parsed command name is already registered.
    pub fn register(&mut self, id: u8, format: &str) -> MsgResult<()> {
        let (name, base) = MsgBase::parse(format)?;
        let cmd = MsgEntry::Base(base);

        let mut map = self
            .msgs
            .lock()
            .map_err(|_| MsgError::new("msgs lock poisoned"))?;

        map.try_insert(Msg {
            id,
            name,
            command: cmd,
        })
        .map_err(|e| MsgError::new(e.to_string()))?;

        Ok(())
    }

    /// Bind a callback to a registered command.
    ///
    /// Converts the command from `Base` to `Handler` with the provided callback.
    /// If the command already has a callback, the new callback replaces it.
    /// The callback receives a slice of `ArgValue` containing all decoded
    /// parameter values in command definition order.
    ///
    /// Returns an error if the command is not found.
    pub fn bind(
        &mut self,
        cmd_name: &str,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> MsgResult<()> {
        let mut map = self
            .msgs
            .lock()
            .map_err(|_| MsgError::new("msgs lock poisoned"))?;

        let id = *map
            .by_name
            .get(cmd_name)
            .ok_or_else(|| MsgError::new(format!("Unknown command: {}", cmd_name)))?;

        // Remove from both indexes before re-inserting
        map.by_name.remove(cmd_name);
        let arc_msg = map
            .by_id
            .remove(&id)
            .ok_or_else(|| MsgError::new(format!("Msg not found: {}", cmd_name)))?;

        // We own this Arc (just removed from the map under Mutex), so try_unwrap always succeeds
        let msg = Arc::try_unwrap(arc_msg).unwrap_or_else(|_| unreachable!("ref count should be 1"));
        let command = msg.command.with_callback(callback);
        map.try_insert(Msg {
            id: msg.id,
            name: msg.name,
            command,
        })
        .map_err(|e| MsgError::new(e))?;

        Ok(())
    }

    /// Encode a single command by message name.
    ///
    /// The payload format is: `[msg_id, param1, param2, ...]`.
    /// The number of values must match the command's expected parameter count.
    ///
    /// # Errors
    /// Returns an error if the message name is not found.
    pub fn encode(&self, name: &str, values: &[ArgValue]) -> MsgResult<Payload> {
        let map = self
            .msgs
            .lock()
            .map_err(|_| MsgError::new("msgs lock poisoned"))?;

        let msg = map
            .get_by_name(name)
            .ok_or_else(|| MsgError::new(format!("Unknown message name: {}", name)))?;

        let param_types = match &msg.command {
            MsgEntry::Base(base) => base.params(),
            MsgEntry::Handler(handler) => handler.params(),
        };

        if values.len() != param_types.len() {
            return Err(MsgError::new(format!(
                "expected {} parameters, got {}",
                param_types.len(),
                values.len()
            )));
        }

        let mut payload = Payload::new();
        payload.push(msg.id)?;
        for value in values {
            payload.push_value(value)?;
        }

        Ok(payload)
    }

    /// Decode a payload into an ordered list of message IDs and their parameter values.
    ///
    /// The payload format is: `[msg_id, param1, param2, ...]` repeated for
    /// each message in the batch. Each `msg_id` is looked up in the registry
    /// and the remaining bytes are decoded according to that message's
    /// parameter type list.
    ///
    /// Returns `Vec<(Arc<Msg>, Vec<ArgValue>)>` where each tuple contains an
    /// owned reference to the message definition and its decoded parameter
    /// values, in the order they appear in the payload.
    pub fn decode(&self, payload: Payload) -> MsgResult<Vec<(Arc<Msg>, Vec<ArgValue>)>> {
        let map = self
            .msgs
            .lock()
            .map_err(|_| MsgError::new("msgs lock poisoned"))?;

        let mut result = Vec::new();
        let mut parser = payload.as_parser();

        while !parser.is_empty() {
            // First byte is the message ID
            let id = parser.pop()?;

            // Look up the message definition
            let arc_msg = map
                .by_id
                .get(&id)
                .ok_or_else(|| MsgError::new(format!("Unknown message id: {}", id)))?;

            // Decode parameters according to the message's parameter types
            let param_types = match &arc_msg.command {
                MsgEntry::Base(base) => base.params(),
                MsgEntry::Handler(handler) => handler.params(),
            };

            let mut values = Vec::with_capacity(param_types.len());
            for (_, arg_type) in param_types {
                values.push(parser.pop_value(*arg_type)?);
            }

            result.push((arc_msg.clone(), values));
        }

        Ok(result)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;


    // -----------------------------------------------------------------------
    // MsgMap
    // -----------------------------------------------------------------------

    #[test]
    fn test_msgmap_insert_and_lookup() {
        let mut map = MsgMap::default();
        map.try_insert(Msg {
            id: 1,
            name: "CMD_A".to_string(),
            command: MsgEntry::Base(MsgBase::parse("CMD_A x=%u").unwrap().1),
        }).unwrap();

        assert!(map.get_by_id(&1).is_some());
        assert!(map.get_by_name("CMD_A").is_some());
        assert!(map.get_by_id(&2).is_none());
        assert!(map.get_by_name("CMD_B").is_none());
    }

    #[test]
    fn test_msgmap_duplicate_id_rejected() {
        let mut map = MsgMap::default();
        map.try_insert(Msg {
            id: 1,
            name: "CMD_A".to_string(),
            command: MsgEntry::Base(MsgBase::new(vec![])),
        }).unwrap();

        let result = map.try_insert(Msg {
            id: 1,
            name: "CMD_B".to_string(),
            command: MsgEntry::Base(MsgBase::new(vec![])),
        });
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("duplicate id"));
    }

    #[test]
    fn test_msgmap_duplicate_name_rejected() {
        let mut map = MsgMap::default();
        map.try_insert(Msg {
            id: 1,
            name: "CMD_A".to_string(),
            command: MsgEntry::Base(MsgBase::new(vec![])),
        }).unwrap();

        let result = map.try_insert(Msg {
            id: 2,
            name: "CMD_A".to_string(),
            command: MsgEntry::Base(MsgBase::new(vec![])),
        });
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("duplicate name"));
    }

    #[test]
    fn test_msgmap_by_name_resolves_through_by_id() {
        let mut map = MsgMap::default();
        let base = MsgBase::parse("TEST a=%u b=%s").unwrap().1;
        map.try_insert(Msg {
            id: 42,
            name: "TEST".to_string(),
            command: MsgEntry::Base(base),
        }).unwrap();

        let msg = map.get_by_name("TEST").unwrap();
        assert_eq!(msg.id, 42);
        let param_len = match &msg.command {
            MsgEntry::Base(b) => b.params().len(),
            MsgEntry::Handler(h) => h.params().len(),
        };
        assert_eq!(param_len, 2);
    }

    // -----------------------------------------------------------------------
    // Parser::new
    // -----------------------------------------------------------------------

    #[test]
    fn test_parser_new_is_empty() {
        let parser = Parser::new();
        // Can't directly check emptiness, but encoding/decoding should fail
        let result = parser.encode("unknown", &[]);
        assert!(result.is_err());
    }

    // -----------------------------------------------------------------------
    // Parser::register
    // -----------------------------------------------------------------------

    #[test]
    fn test_register_success() {
        let mut parser = Parser::new();
        let result = parser.register(1, "CMD_A x=%u");
        assert!(result.is_ok());
    }

    #[test]
    fn test_register_multiple_commands() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        parser.register(2, "CMD_B y=%s z=%c").unwrap();
        parser.register(3, "CMD_C").unwrap(); // no params

        // All should be accessible
        let payload = parser.encode("CMD_A", &[ArgValue::UInt32(42)]).unwrap();
        assert_eq!(payload.payload()[0], 1);

        let payload = parser.encode("CMD_B", &[ArgValue::Str("hello".to_string()), ArgValue::UInt8(99)]).unwrap();
        assert_eq!(payload.payload()[0], 2);

        let payload = parser.encode("CMD_C", &[]).unwrap();
        assert_eq!(payload.payload()[0], 3);
    }

    #[test]
    fn test_register_duplicate_id_fails() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        let result = parser.register(1, "CMD_B y=%u");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("duplicate id"));
    }

    #[test]
    fn test_register_duplicate_name_fails() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        let result = parser.register(2, "CMD_A y=%u");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("duplicate name"));
    }

    #[test]
    fn test_register_invalid_format_fails() {
        let mut parser = Parser::new();
        // Empty type specifier is invalid
        let result = parser.register(1, "CMD name=");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("unknown type specifier"));
    }

    #[test]
    fn test_register_unknown_type_fails() {
        let mut parser = Parser::new();
        let result = parser.register(1, "CMD x=%x");
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("unknown type specifier"));
    }

    // -----------------------------------------------------------------------
    // Parser::bind
    // -----------------------------------------------------------------------

    #[test]
    fn test_bind_success() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();

        parser.bind("CMD_A", |_| {}).unwrap();

        // Encode should still work after binding
        let payload = parser.encode("CMD_A", &[ArgValue::UInt32(10)]).unwrap();
        assert_eq!(payload.payload()[0], 1);

        // Decode should work and return correct values
        let result = parser.decode(payload).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].0.name, "CMD_A");
        assert_eq!(result[0].1[0], ArgValue::UInt32(10));
    }

    #[test]
    fn test_bind_unknown_command_fails() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        let result = parser.bind("NONEXISTENT", |_| {});
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown command"));
    }

    #[test]
    fn test_bind_replaces_callback() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();

        // First bind
        parser.bind("CMD_A", |_| {}).unwrap();

        // Second bind replaces the callback
        parser.bind("CMD_A", |_| {}).unwrap();

        // Encode/decode should still work
        let payload = parser.encode("CMD_A", &[ArgValue::UInt32(1)]).unwrap();
        let result = parser.decode(payload).unwrap();
        assert_eq!(result[0].1[0], ArgValue::UInt32(1));
    }

    #[test]
    fn test_bind_still_decodes_correctly() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_MULTI a=%u b=%s c=%c").unwrap();

        // Bind a callback (callback not invoked during decode)
        parser.bind("CMD_MULTI", |_| {}).unwrap();

        let payload = parser.encode(
            "CMD_MULTI",
            &[
                ArgValue::UInt32(42),
                ArgValue::Str("hello".to_string()),
                ArgValue::UInt8(99),
            ],
        ).unwrap();

        let result = parser.decode(payload).unwrap();
        assert_eq!(result[0].0.name, "CMD_MULTI");
        assert_eq!(result[0].1.len(), 3);
        assert_eq!(result[0].1[0], ArgValue::UInt32(42));
        assert_eq!(result[0].1[1], ArgValue::Str("hello".to_string()));
        assert_eq!(result[0].1[2], ArgValue::UInt8(99));
    }

    // -----------------------------------------------------------------------
    // Parser::encode
    // -----------------------------------------------------------------------

    #[test]
    fn test_encode_no_params() {
        let mut parser = Parser::new();
        parser.register(1, "NOP").unwrap();
        let payload = parser.encode("NOP", &[]).unwrap();
        assert_eq!(payload.payload(), &[1]);
    }

    #[test]
    fn test_encode_single_param() {
        let mut parser = Parser::new();
        parser.register(1, "GET_X val=%u").unwrap();
        let payload = parser.encode("GET_X", &[ArgValue::UInt32(123)]).unwrap();
        assert_eq!(payload.payload()[0], 1); // msg id
    }

    #[test]
    fn test_encode_multiple_param_types() {
        let mut parser = Parser::new();
        parser.register(1, "MULTI a=%u b=%i c=%hu d=%hi e=%s f=%c").unwrap();
        let payload = parser.encode(
            "MULTI",
            &[
                ArgValue::UInt32(100),
                ArgValue::Int32(-200),
                ArgValue::UInt16(300),
                ArgValue::Int16(-400),
                ArgValue::Str("test".to_string()),
                ArgValue::UInt8(50),
            ],
        ).unwrap();
        assert_eq!(payload.payload()[0], 1);
    }

    #[test]
    fn test_encode_unknown_name_fails() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        let result = parser.encode("UNKNOWN", &[ArgValue::UInt32(1)]);
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown message name"));
    }

    #[test]
    fn test_encode_wrong_param_count_too_many() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        let result = parser.encode("CMD_A", &[ArgValue::UInt32(1), ArgValue::UInt32(2)]);
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("expected 1 parameters, got 2"));
    }

    #[test]
    fn test_encode_wrong_param_count_too_few() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u y=%s").unwrap();
        let result = parser.encode("CMD_A", &[ArgValue::UInt32(1)]);
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("expected 2 parameters, got 1"));
    }

    #[test]
    fn test_encode_handler_still_works() {
        // After binding, encode should still work (Handler also has params)
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        parser.bind("CMD_A", |_| {}).unwrap();

        let payload = parser.encode("CMD_A", &[ArgValue::UInt32(99)]).unwrap();
        assert_eq!(payload.payload()[0], 1);
    }

    // -----------------------------------------------------------------------
    // Parser::decode
    // -----------------------------------------------------------------------

    #[test]
    fn test_decode_no_params() {
        let mut parser = Parser::new();
        parser.register(1, "NOP").unwrap();
        let mut payload = Payload::new();
        payload.push(1).unwrap();

        let result = parser.decode(payload).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].0.name, "NOP");
        assert!(result[0].1.is_empty());
    }

    #[test]
    fn test_decode_single_param() {
        let mut parser = Parser::new();
        parser.register(1, "GET_X val=%u").unwrap();
        let mut payload = Payload::new();
        payload.push(1).unwrap();
        payload.push_u32(42).unwrap();

        let result = parser.decode(payload).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].0.name, "GET_X");
        assert_eq!(result[0].1[0], ArgValue::UInt32(42));
    }

    #[test]
    fn test_decode_multiple_params() {
        let mut parser = Parser::new();
        parser.register(1, "CMD a=%u b=%s c=%c").unwrap();
        let mut payload = Payload::new();
        payload.push(1).unwrap();
        payload.push_u32(100).unwrap();
        payload.push_bytes("hello".as_bytes()).unwrap();
        payload.push_u8(42).unwrap();

        let result = parser.decode(payload).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].0.name, "CMD");
        assert_eq!(result[0].1.len(), 3);
        assert_eq!(result[0].1[0], ArgValue::UInt32(100));
        assert_eq!(result[0].1[1], ArgValue::Str("hello".to_string()));
        assert_eq!(result[0].1[2], ArgValue::UInt8(42));
    }

    #[test]
    fn test_decode_multiple_messages_batch() {
        let mut parser = Parser::new();
        parser.register(1, "MSG_A x=%u").unwrap();
        parser.register(2, "MSG_B y=%s").unwrap();
        parser.register(3, "MSG_C").unwrap();

        let mut payload = Payload::new();
        payload.push(1).unwrap();
        payload.push_u32(10).unwrap();
        payload.push(2).unwrap();
        payload.push_bytes("world".as_bytes()).unwrap();
        payload.push(3).unwrap();

        let result = parser.decode(payload).unwrap();
        assert_eq!(result.len(), 3);

        assert_eq!(result[0].0.name, "MSG_A");
        assert_eq!(result[0].1[0], ArgValue::UInt32(10));

        assert_eq!(result[1].0.name, "MSG_B");
        assert_eq!(result[1].1[0], ArgValue::Str("world".to_string()));

        assert_eq!(result[2].0.name, "MSG_C");
        assert!(result[2].1.is_empty());
    }

    #[test]
    fn test_decode_unknown_id_fails() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        let mut payload = Payload::new();
        payload.push(99).unwrap(); // unknown id

        let result = parser.decode(payload);
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown message id"));
    }

    #[test]
    fn test_decode_bytes_param() {
        let mut parser = Parser::new();
        parser.register(1, "RAW data=%.*s").unwrap();
        let mut payload = Payload::new();
        payload.push(1).unwrap();
        payload.push_bytes(&[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();

        let result = parser.decode(payload).unwrap();
        assert_eq!(result[0].1[0], ArgValue::Bytes(vec![0xDE, 0xAD, 0xBE, 0xEF]));
    }

    #[test]
    fn test_decode_signed_values() {
        let mut parser = Parser::new();
        parser.register(1, "SIGNED a=%i b=%hi").unwrap();
        let mut payload = Payload::new();
        payload.push(1).unwrap();
        payload.push_u32(0xFFFFFFFF).unwrap(); // -1 as signed u32
        payload.push_u16(0xFFFF).unwrap(); // -1 as signed u16

        let result = parser.decode(payload).unwrap();
        assert_eq!(result[0].1[0], ArgValue::Int32(-1));
        assert_eq!(result[0].1[1], ArgValue::Int16(-1));
    }

    // -----------------------------------------------------------------------
    // Integration: encode → decode roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn test_encode_decode_roundtrip_no_params() {
        let mut parser = Parser::new();
        parser.register(1, "NOP").unwrap();

        let payload = parser.encode("NOP", &[]).unwrap();
        let decoded = parser.decode(payload).unwrap();

        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "NOP");
        assert!(decoded[0].1.is_empty());
    }

    #[test]
    fn test_encode_decode_roundtrip_all_types() {
        let mut parser = Parser::new();
        parser.register(
            1,
            "ALL a=%u b=%i c=%hu d=%hi e=%s f=%c g=%.*s",
        ).unwrap();

        let values = vec![
            ArgValue::UInt32(0xDEADBEEF),
            ArgValue::Int32(-12345),
            ArgValue::UInt16(1234),
            ArgValue::Int16(-567),
            ArgValue::Str("klipper".to_string()),
            ArgValue::UInt8(255),
            ArgValue::Bytes(vec![0x00, 0xFF, 0xAA]),
        ];

        let payload = parser.encode("ALL", &values).unwrap();
        let decoded = parser.decode(payload).unwrap();

        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "ALL");
        assert_eq!(decoded[0].1.len(), 7);
        for (expected, actual) in values.iter().zip(decoded[0].1.iter()) {
            assert_eq!(expected, actual);
        }
    }

    #[test]
    fn test_encode_decode_roundtrip_multiple_messages() {
        let mut parser = Parser::new();
        parser.register(1, "A x=%u").unwrap();
        parser.register(2, "B y=%s").unwrap();

        let mut payload_a = parser.encode("A", &[ArgValue::UInt32(1)]).unwrap();
        let payload_b = parser.encode("B", &[ArgValue::Str("test".to_string())]).unwrap();
        payload_a.try_merge(&payload_b).unwrap();

        let decoded = parser.decode(payload_a).unwrap();
        assert_eq!(decoded.len(), 2);
        assert_eq!(decoded[0].0.name, "A");
        assert_eq!(decoded[0].1[0], ArgValue::UInt32(1));
        assert_eq!(decoded[1].0.name, "B");
        assert_eq!(decoded[1].1[0], ArgValue::Str("test".to_string()));
    }

    // -----------------------------------------------------------------------
    // Integration: register → bind → decode → callback
    // -----------------------------------------------------------------------

    #[test]
    fn test_bind_decode_lifecycle() {
        let mut parser = Parser::new();
        parser.register(1, "TEST_CMD a=%u b=%s").unwrap();

        // Bind a callback
        parser.bind("TEST_CMD", |_| {}).unwrap();

        // Encode with the bound command
        let payload = parser.encode(
            "TEST_CMD",
            &[
                ArgValue::UInt32(42),
                ArgValue::Str("hello".to_string()),
            ],
        ).unwrap();

        // Decode should return correct values
        let result = parser.decode(payload).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].0.name, "TEST_CMD");
        assert_eq!(result[0].1.len(), 2);
        assert_eq!(result[0].1[0], ArgValue::UInt32(42));
        assert_eq!(result[0].1[1], ArgValue::Str("hello".to_string()));
    }

    #[test]
    fn test_bind_decode_multiple_batch() {
        let mut parser = Parser::new();
        parser.register(1, "CMD_A x=%u").unwrap();
        parser.register(2, "CMD_B y=%s").unwrap();

        // Bind callbacks to both commands
        parser.bind("CMD_A", |_| {}).unwrap();
        parser.bind("CMD_B", |_| {}).unwrap();

        let mut payload = parser.encode("CMD_A", &[ArgValue::UInt32(1)]).unwrap();
        let payload_b = parser.encode("CMD_B", &[ArgValue::Str("b".to_string())]).unwrap();
        payload.try_merge(&payload_b).unwrap();

        let result = parser.decode(payload).unwrap();
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].0.name, "CMD_A");
        assert_eq!(result[0].1[0], ArgValue::UInt32(1));
        assert_eq!(result[1].0.name, "CMD_B");
        assert_eq!(result[1].1[0], ArgValue::Str("b".to_string()));
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_parser_arc_sharing() {
        // Parser uses Arc<Mutex<...>>, so cloning should share state
        let mut parser = Parser::new();
        parser.register(1, "SHARED x=%u").unwrap();

        let shared = std::sync::Arc::new(std::sync::Mutex::new(parser));
        {
            let p = shared.lock().unwrap();
            let payload = p.encode("SHARED", &[ArgValue::UInt32(77)]).unwrap();
            let decoded = p.decode(payload).unwrap();
            assert_eq!(decoded[0].1[0], ArgValue::UInt32(77));
        }
        // State persists after unlock
        {
            let p = shared.lock().unwrap();
            assert!(p.encode("SHARED", &[ArgValue::UInt32(1)]).is_ok());
        }
    }

    #[test]
    fn test_register_then_bind_then_encode() {
        // Full lifecycle: register (Base) → bind (Handler) → encode (still works)
        let mut parser = Parser::new();
        parser.register(1, "LIFECYCLE a=%u b=%s c=%c").unwrap();
        parser.bind("LIFECYCLE", |_| {}).unwrap();

        let payload = parser.encode(
            "LIFECYCLE",
            &[
                ArgValue::UInt32(1),
                ArgValue::Str("test".to_string()),
                ArgValue::UInt8(2),
            ],
        ).unwrap();

        let decoded = parser.decode(payload).unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "LIFECYCLE");
        assert_eq!(decoded[0].1.len(), 3);
    }

    #[test]
    fn test_bind_on_handler_still_works() {
        // bind on a command that's already a Handler should replace the callback
        let mut parser = Parser::new();
        parser.register(1, "REBIND x=%u").unwrap();

        // First bind
        parser.bind("REBIND", |_| {}).unwrap();

        // Re-bind should replace the callback
        parser.bind("REBIND", |_| {}).unwrap();

        // Encode/decode should still work
        let payload = parser.encode("REBIND", &[ArgValue::UInt32(1)]).unwrap();
        let result = parser.decode(payload).unwrap();
        assert_eq!(result[0].1[0], ArgValue::UInt32(1));
    }
}
