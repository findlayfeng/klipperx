use super::command::{CommandBase, CommandEntry, Param};
use super::proto::{ArgType, Payload, ProtoError, ProtoResult, ArgValue};
use super::super::frame::MESSAGE_PAYLOAD_MAX;
use super::super::traits::KlippyInterface;
use multi_index_map::MultiIndexMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{mpsc, oneshot};
use tokio::sync::Mutex as AsyncMutex;
use tokio::time::Instant;

/// Coalescing window for outbound payloads.
const SEND_COALESCE_WINDOW: Duration = Duration::from_millis(1);

/// Payloads at least this large are sent immediately without coalescing.
/// This is 2/3 of the maximum payload length.
const SEND_COALESCE_THRESHOLD: usize = MESSAGE_PAYLOAD_MAX * 2 / 3;

/// Default Klipper message formats for identify request/response.
#[allow(dead_code)]
const DEFAULT_MESSAGES: &[(u8, &str)] = &[
    (0, "identify_response offset=%u data=%.*s"),
    (1, "identify offset=%c count=%c"),
];

#[derive(MultiIndexMap, Debug)]
#[multi_index_derive(Debug)]
// #[multi_index_hash(rustc_hash::FxBuildHasher)]
pub struct Command {
    #[multi_index(hashed_unique)]
    id: u8,
    #[multi_index(hashed_unique)]
    name: String,
    command: CommandEntry,
}

pub struct Parser {
    commands: MultiIndexCommandMap,
    interface: Arc<dyn KlippyInterface>,
    /// Serializes outbound traffic and performs payload coalescing.
    outbox: AsyncMutex<Option<mpsc::Sender<OutItem>>>,
}

impl Parser {
    /// Create a new parser and register default message formats.
    pub fn new(interface: Arc<dyn KlippyInterface>) -> Self {
        let mut parser = Self {
            commands: MultiIndexCommandMap::default(),
            interface,
            outbox: AsyncMutex::new(None),
        };

        for (id, format_str) in DEFAULT_MESSAGES {
            let _ = parser.register(*id, format_str);
        }
        parser
    }

    /// Register a message format with the given ID.
    ///
    /// - If `id & 1 == 1`, the message is treated as a regular (response) format.
    /// - If `id & 1 == 0`, the message is treated as a request format.
    fn register(&mut self, id: u8, format: &str) -> ProtoResult<()> {
        let (name, base) =
            CommandBase::parse(format).map_err(|e| ProtoError::new(e.to_string()))?;
        let cmd = match id & 0x1 {
            0 => CommandEntry::Request(base.into()),
            _ => CommandEntry::Regular(base),
        };

        self.commands
            .try_insert(Command {
                id,
                name,
                command: cmd,
            })
            .map_err(|e| ProtoError::new(e.to_string()))?;

        Ok(())
    }

    /// Send a command with the given name and parameter values.
    ///
    /// Only `Regular` type commands can be sent. `Request` type commands
    /// (registered with even ID) cannot be sent via this method.
    ///
    /// Supports both positional and named parameters:
    /// - Positional parameters (`Param::Positional`) must come first and follow
    ///   the command's parameter order
    /// - Named parameters (`Param::Named`) can come after positional params and
    ///   can be in any order
    ///
    /// Parameters are encoded in the order defined by the command, regardless
    /// of the order they were provided.
    ///
    /// # Arguments
    /// * `cmd_name` - The command name (e.g. `"G1"`, `"M105"`)
    /// * `params` - List of `Param` (positional followed by named)
    ///
    /// # Errors
    /// Returns `ProtoError` if:
    /// - The command name is not found in the registry
    /// - The command is a `Request` type (only `Regular` commands can be sent)
    /// - Positional params don't match the expected parameter count or order
    /// - Named params reference unknown parameter names
    /// - Parameter types don't match the command definition
    /// - The interface send fails
    ///
    /// # Example
    /// ```ignore
    /// // All positional (must be in order)
    /// let params = vec![
    ///     Param::Positional(ArgValue::UInt32(100)),   // X
    ///     Param::Positional(ArgValue::UInt32(200)),   // Y
    /// ];
    /// parser.send("G1", &params).await?;
    ///
    /// // Mixed positional and named
    /// let params = vec![
    ///     Param::Positional(ArgValue::UInt32(100)),   // X (positional)
    ///     Param::Named("Y".to_string(), ArgValue::UInt32(200)),  // Y (named)
    /// ];
    /// parser.send("G1", &params).await?;
    /// ```
    pub async fn send(&self, cmd_name: &str, params: &[Param]) -> ProtoResult<()> {
        let cmd = self
            .commands
            .get_by_name(cmd_name)
            .ok_or_else(|| ProtoError::new(format!("Unknown command: {}", cmd_name)))?;

        // Only Regular type commands can be sent
        let param_defs: Vec<(String, ArgType)> = match &cmd.command {
            CommandEntry::Regular(base) => base.params().to_vec(),
            CommandEntry::Request(_) => {
                return Err(ProtoError::new(format!(
                    "Cannot send Request type command: {}",
                    cmd_name
                )));
            }
        };

        // Separate positional and named parameters
        let mut positional_count = 0;
        let mut named_params: Vec<(&str, &ArgValue)> = Vec::new();

        for param in params {
            match param {
                Param::Positional(_) => {
                    positional_count += 1;
                }
                Param::Named(name, _) => {
                    named_params.push((name.as_str(), param.value()));
                }
            }
        }

        // Validate positional params count
        if positional_count > param_defs.len() {
            return Err(ProtoError::new(format!(
                "Too many positional params for '{}': expected at most {}, got {}",
                cmd_name,
                param_defs.len(),
                positional_count
            )));
        }

        // Build a map of named params for quick lookup
        let named_map: std::collections::HashMap<&str, &ArgValue> = named_params.into_iter().collect();

        // Build the final parameter list in command definition order
        let mut final_params: Vec<ArgValue> = Vec::with_capacity(param_defs.len());

        for (i, (param_name, expected_type)) in param_defs.iter().enumerate() {
            let value = if i < positional_count {
                // Positional param - get from params list
                params[i].value().clone()
            } else if let Some(&named_value) = named_map.get(param_name.as_str()) {
                // Named param - look up by name
                named_value.clone()
            } else {
                return Err(ProtoError::new(format!(
                    "Missing required param '{}' for '{}'",
                    param_name, cmd_name
                )));
            };

            // Validate type — attempt conversion if types don't match
            if value.arg_type() != *expected_type {
                if let Ok(converted) = value.try_convert_to(*expected_type) {
                    eprintln!(
                        "[WARNING] Param type conversion for '{}' param '{}': {:?} -> {:?}",
                        cmd_name,
                        param_name,
                        value.arg_type(),
                        expected_type
                    );
                    final_params.push(converted);
                } else {
                    return Err(ProtoError::new(format!(
                        "Param type mismatch for '{}' param '{}': expected {:?}, got {:?}",
                        cmd_name,
                        param_name,
                        expected_type,
                        value.arg_type()
                    )));
                }
            } else {
                final_params.push(value);
            }
        }

        let mut payload = Payload::new();
        payload.push(cmd.id)?;
        for value in final_params {
            payload.push_value(&value)?;
        }

        // All outbound payloads go through the outbox task so that the send
        // order is preserved while small payloads may be coalesced into one.
        let (ack_tx, ack_rx) = oneshot::channel();
        let outbox = self.ensure_outbox().await;
        outbox
            .send(OutItem {
                payload,
                ack: ack_tx,
            })
            .await
            .map_err(|_| ProtoError::new("outbox closed"))?;
        ack_rx
            .await
            .map_err(|_| ProtoError::new("outbox task terminated"))??;

        Ok(())
    }

    /// Lazily start the outbox task and return its sender.
    ///
    /// The outbox task is the single writer to the interface, so outbound
    /// payloads are always transmitted in call order even when coalescing
    /// batches multiple commands into one frame.
    async fn ensure_outbox(&self) -> mpsc::Sender<OutItem> {
        let mut guard = self.outbox.lock().await;
        if let Some(sender) = guard.as_ref() {
            return sender.clone();
        }
        let (sender, receiver) = mpsc::channel::<OutItem>(64);
        let interface = Arc::clone(&self.interface);
        tokio::spawn(async move { run_sender(interface, receiver).await });
        let cloned = sender.clone();
        *guard = Some(sender);
        cloned
    }
}

/// One outbound payload together with the completion signal for the caller.
struct OutItem {
    payload: Payload,
    ack: oneshot::Sender<ProtoResult<()>>,
}

/// Send `payload` through `interface` and resolve every `ack` with the result.
async fn send_and_ack(
    interface: &Arc<dyn KlippyInterface>,
    payload: Payload,
    acks: Vec<oneshot::Sender<ProtoResult<()>>>,
) {
    let result = match interface.send(&payload).await {
        Ok(()) => Ok(()),
        Err(e) => Err(ProtoError::new(e.to_string())),
    };
    for ack in acks {
        let _ = ack.send(result.clone());
    }
}

/// Serialize outbound payloads and coalesce small ones.
///
/// Payloads arrive on a single FIFO channel, so transmission order matches the
/// order the calls to [`Parser::send`] were made.
///
/// A payload smaller than 2/3 of the maximum length opens a coalescing window
/// of [`SEND_COALESCE_WINDOW`]. Payloads that arrive during the window are
/// merged into the current batch via [`Payload::try_merge`] as long as the
/// combined length stays within [`MESSAGE_PAYLOAD_MAX`].
///
/// If a merge fails, the pending batch is flushed. If the new payload is large
/// (≥ threshold) it is sent immediately; otherwise it opens a fresh window.
async fn run_sender(interface: Arc<dyn KlippyInterface>, mut rx: mpsc::Receiver<OutItem>) {
    while let Some(OutItem { payload, ack }) = rx.recv().await {
        // Large payloads are sent immediately without coalescing.
        if payload.len() >= SEND_COALESCE_THRESHOLD {
            send_and_ack(&interface, payload, vec![ack]).await;
            continue;
        }

        // Open a coalescing window and gather everything that fits.
        let mut batch = payload;
        let mut acks = vec![ack];
        let mut deadline = Instant::now() + SEND_COALESCE_WINDOW;

        loop {
            let timer = tokio::time::sleep_until(deadline);
            tokio::pin!(timer);
            tokio::select! {
                _ = &mut timer => break,
                next = rx.recv() => match next {
                    None => break,
                    Some(OutItem { payload: next_payload, ack: next_ack }) => {
                        // First try to merge the new payload.
                        if batch.try_merge(&next_payload).is_ok() {
                            acks.push(next_ack);
                        } else {
                            // Merge failed (size limit reached): save whether
                            // the new payload is large before moving it into
                            // batch via replace.
                            let is_large = next_payload.len() >= SEND_COALESCE_THRESHOLD;
                            let pending = std::mem::replace(&mut batch, next_payload);
                            let pending_acks = std::mem::replace(&mut acks, vec![]);
                            send_and_ack(&interface, pending, pending_acks).await;

                            if is_large {
                                // Send the large payload immediately.
                                send_and_ack(&interface, batch, vec![next_ack]).await;
                                batch = Payload::new();
                                acks.clear();
                            } else {
                                // New payload is small: it opens a fresh window.
                                acks = vec![next_ack];
                                deadline = Instant::now() + SEND_COALESCE_WINDOW;
                            }
                        }
                    }
                },
            }
        }

        if !batch.is_empty() {
            send_and_ack(&interface, batch, acks).await;
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::test::{TestInterface, MappingEntry};
    use crate::core::klippy::frame::Frame;

    /// Helper to build expected payload using Payload methods.
    fn build_g1_payload(x: u32, y: u32) -> Payload {
        let mut p = Payload::new();
        p.push(3).unwrap(); // G1 cmd id
        p.push_u32(x).unwrap();
        p.push_u32(y).unwrap();
        p
    }

    fn register_g1_cmd(parser: &mut Parser) {
        // Register "G1 X=%u Y=%u" with id=3 (regular message, odd id)
        let _ = parser.register(3, "G1 X=%u Y=%u");
    }

    fn register_m105_cmd(parser: &mut Parser) {
        // Register "M105" with no params (id=5, regular message)
        let _ = parser.register(5, "M105");
    }

    fn register_request_cmd(parser: &mut Parser) {
        // Register "REQ" as a request type command (even id)
        let _ = parser.register(2, "REQ param=%u");
    }

    #[tokio::test]
    async fn test_send_g1_with_params() {
        let expected = build_g1_payload(100, 200);
        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_g1_cmd(&mut parser);

        let params = vec![
            Param::Positional(ArgValue::UInt32(100)),
            Param::Positional(ArgValue::UInt32(200)),
        ];
        parser.send("G1", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_unknown_command() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let parser = Parser::new(Arc::new(interface));

        let params = vec![Param::Positional(ArgValue::UInt32(1))];
        let result = parser.send("UNKNOWN_CMD", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown command"));
    }

    #[tokio::test]
    async fn test_send_param_count_mismatch() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_g1_cmd(&mut parser);

        // G1 expects 2 params, send only 1
        let params = vec![Param::Positional(ArgValue::UInt32(100))];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Missing required param"));
    }

    #[tokio::test]
    async fn test_send_param_type_mismatch() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        // Register a command with string param (odd ID = Regular type)
        let _ = parser.register(7, "CMD label=%s");

        // Send UInt32 instead of Str
        let params = vec![Param::Positional(ArgValue::UInt32(42))];
        let result = parser.send("CMD", &params).await;
        assert!(result.is_err(), "Expected error, got Ok");
        let err_msg = result.unwrap_err().msg;
        assert!(err_msg.contains("Param type mismatch"), "Error message: {}", err_msg);
    }

    #[tokio::test]
    async fn test_send_command_no_params() {
        let mut expected = Payload::new();
        expected.push(5).unwrap(); // M105 cmd id
        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_m105_cmd(&mut parser);

        let params: Vec<Param> = vec![];
        parser.send("M105", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_string_param() {
        let mut expected = Payload::new();
        expected.push(9).unwrap(); // TEST cmd id
        expected.push_bytes(b"hello").unwrap();
        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        // Register a command with a string param
        let _ = parser.register(9, "TEST name=%s");

        let params = vec![Param::Positional(ArgValue::Str("hello".to_string()))];
        parser.send("TEST", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_bytes_param() {
        let mut expected = Payload::new();
        expected.push(11).unwrap(); // TEST cmd id
        expected.push_bytes(&[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        // Register a command with a bytes param
        let _ = parser.register(11, "TEST data=%c");

        let params = vec![Param::Positional(ArgValue::Bytes(vec![0xDE, 0xAD, 0xBE, 0xEF]))];
        parser.send("TEST", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_mixed_params() {
        let mut expected = Payload::new();
        expected.push(13).unwrap(); // CMD cmd id
        expected.push_u32(42).unwrap();
        expected.push_bytes(b"test_label").unwrap();
        expected.push_bytes(&[0x01, 0x02]).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        // Register a command with mixed types: uint32, string, bytes
        let _ = parser.register(13, "CMD oid=%u label=%s raw=%c");

        let params = vec![
            Param::Positional(ArgValue::UInt32(42)),
            Param::Positional(ArgValue::Str("test_label".to_string())),
            Param::Positional(ArgValue::Bytes(vec![0x01, 0x02])),
        ];
        parser.send("CMD", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_named_params() {
        let mut expected = Payload::new();
        expected.push(3).unwrap(); // G1 cmd id
        expected.push_u32(100).unwrap();
        expected.push_u32(200).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_g1_cmd(&mut parser);

        // Named params in reverse order (Y before X)
        let params = vec![
            Param::Named("Y".to_string(), ArgValue::UInt32(200)),
            Param::Named("X".to_string(), ArgValue::UInt32(100)),
        ];
        parser.send("G1", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_mixed_positional_named() {
        let mut expected = Payload::new();
        expected.push(3).unwrap(); // G1 cmd id
        expected.push_u32(100).unwrap();
        expected.push_u32(200).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_g1_cmd(&mut parser);

        // X as positional, Y as named
        let params = vec![
            Param::Positional(ArgValue::UInt32(100)),
            Param::Named("Y".to_string(), ArgValue::UInt32(200)),
        ];
        parser.send("G1", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_named_param_not_found() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_g1_cmd(&mut parser);

        // Use unknown param name
        let params = vec![Param::Named("Z".to_string(), ArgValue::UInt32(100))];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Missing required param"));
    }

    #[tokio::test]
    async fn test_send_too_many_positional() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_g1_cmd(&mut parser);

        // G1 expects 2 params, send 3 positional
        let params = vec![
            Param::Positional(ArgValue::UInt32(100)),
            Param::Positional(ArgValue::UInt32(200)),
            Param::Positional(ArgValue::UInt32(300)),
        ];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Too many positional params"));
    }

    #[tokio::test]
    async fn test_send_request_type_not_allowed() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_request_cmd(&mut parser);

        // Request type commands (even id) cannot be sent
        let params = vec![Param::Positional(ArgValue::UInt32(1))];
        let result = parser.send("REQ", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Cannot send Request type command"));
    }

    #[tokio::test]
    async fn test_send_multiple_commands() {
        let g1_payload = build_g1_payload(100, 200);
        let mut m105_payload = Payload::new();
        m105_payload.push(5).unwrap(); // M105 cmd id

        let mapping = vec![
            MappingEntry {
                input: Frame::new(0, g1_payload.payload().to_vec()),
                outputs: vec![],
            },
            MappingEntry {
                input: Frame::new(1, m105_payload.payload().to_vec()),
                outputs: vec![],
            },
        ];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));
        register_g1_cmd(&mut parser);
        register_m105_cmd(&mut parser);

        // Send G1
        let params_g1 = vec![
            Param::Positional(ArgValue::UInt32(100)),
            Param::Positional(ArgValue::UInt32(200)),
        ];
        parser.send("G1", &params_g1).await.unwrap();

        // Send M105
        let params_m105: Vec<Param> = vec![];
        parser.send("M105", &params_m105).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_with_type_conversion_int32_to_uint32() {
        // Register command expecting UInt32
        let mut expected = Payload::new();
        expected.push(15).unwrap(); // CMD cmd id
        expected.push_u32(42).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        let _ = parser.register(15, "CMD val=%u");

        // Send Int32 instead of UInt32 — should warn and convert
        let params = vec![Param::Positional(ArgValue::Int32(42))];
        parser.send("CMD", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_with_type_conversion_int16_to_uint16() {
        let mut expected = Payload::new();
        expected.push(17).unwrap(); // CMD cmd id
        expected.push_u16(100).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        let _ = parser.register(17, "CMD val=%hu");

        // Send Int16 instead of UInt16 — should warn and convert
        let params = vec![Param::Positional(ArgValue::Int16(100))];
        parser.send("CMD", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_with_type_conversion_uint16_to_int32() {
        let mut expected = Payload::new();
        expected.push(19).unwrap(); // CMD cmd id
        expected.push_u32(100).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        let _ = parser.register(19, "CMD val=%i");

        // Send UInt16 instead of Int32 — should warn and convert
        let params = vec![Param::Positional(ArgValue::UInt16(100))];
        parser.send("CMD", &params).await.unwrap();
    }

    #[tokio::test]
    async fn test_send_type_conversion_not_possible() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(Arc::new(interface));

        let _ = parser.register(21, "CMD val=%s");

        // Send UInt32 instead of Str — conversion not possible
        let params = vec![Param::Positional(ArgValue::UInt32(42))];
        let result = parser.send("CMD", &params).await;
        assert!(result.is_err());
        let err_msg = result.unwrap_err().msg;
        assert!(err_msg.contains("Param type mismatch"), "Error message: {}", err_msg);
    }
}
