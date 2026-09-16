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
    by_id: HashMap<u8, Msg>,
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
        self.by_id.insert(msg.id, msg);
        Ok(())
    }

    fn get_by_id(&self, id: &u8) -> Option<&Msg> {
        self.by_id.get(id)
    }

    fn get_by_name(&self, name: &str) -> Option<&Msg> {
        self.by_name.get(name).and_then(|id| self.by_id.get(id))
    }

    /// Remove a message by name, keeping both indexes in sync.
    fn remove_by_name(&mut self, name: &str) -> Option<Msg> {
        let id = self.by_name.remove(name)?;
        self.by_id.remove(&id)
    }
}

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

        let (id, name) = map
            .get_by_name(cmd_name)
            .map(|c| (c.id, c.name.clone()))
            .ok_or_else(|| MsgError::new(format!("Unknown command: {}", cmd_name)))?;

        let entry = map
            .remove_by_name(&name)
            .ok_or_else(|| MsgError::new(format!("Msg not found: {}", cmd_name)))?;

        let command = entry.command.with_callback(callback);

        map.try_insert(Msg { id, name, command })
            .map_err(|e| MsgError::new(e.to_string()))?;

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
    /// Returns `Vec<(String, Vec<ArgValue>)>` where each tuple contains a message
    /// Name and its decoded parameter values, in the order they appear in the
    /// payload.
    pub fn decode(&self, payload: Payload) -> MsgResult<Vec<(String, Vec<ArgValue>)>> {
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
            let msg = map
                .get_by_id(&id)
                .ok_or_else(|| MsgError::new(format!("Unknown message id: {}", id)))?;

            // Decode parameters according to the message's parameter types
            let param_types = match &msg.command {
                MsgEntry::Base(base) => base.params(),
                MsgEntry::Handler(handler) => handler.params(),
            };

            let mut values = Vec::with_capacity(param_types.len());
            for (_, arg_type) in param_types {
                values.push(parser.pop_value(*arg_type)?);
            }

            result.push((msg.name.clone(), values));
        }

        Ok(result)
    }
}
