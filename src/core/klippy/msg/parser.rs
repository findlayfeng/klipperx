use super::error::{MsgError, MsgResult};
use super::param::Param;
use super::proto::{ArgType, Payload, ArgValue};
use super::{MsgBase, MsgEntry};
use super::super::frame::MESSAGE_PAYLOAD_MAX;
use super::super::traits::KlippyInterface;
use multi_index_map::MultiIndexMap;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{mpsc, oneshot};
use tokio::sync::Mutex as AsyncMutex;
use tokio::time::Instant;

/// Shared command registry wrapped in `Arc<Mutex>` for thread-safe sharing.
type MsgRegistry = Arc<Mutex<MultiIndexMsgMap>>;

/// Coalescing window for outbound payloads.
const SEND_COALESCE_WINDOW: Duration = Duration::from_millis(1);

/// Payloads at least this large are sent immediately without coalescing.
/// This is 2/3 of the maximum payload length.
const SEND_COALESCE_THRESHOLD: usize = MESSAGE_PAYLOAD_MAX * 2 / 3;

#[derive(MultiIndexMap, Debug)]
#[multi_index_derive(Debug)]
// #[multi_index_hash(rustc_hash::FxBuildHasher)]
pub struct Msg {
    #[multi_index(hashed_unique)]
    id: u8,
    #[multi_index(hashed_unique)]
    name: String,
    command: MsgEntry,
}

pub struct Parser<I: KlippyInterface> {
    msgs: MsgRegistry,
    interface: I,
    /// Serializes outbound traffic and performs payload coalescing.
    outbox: AsyncMutex<Option<mpsc::Sender<OutItem>>>,
    /// Sender for inbound messages (set when inbox is started).
    inbound_tx: AsyncMutex<Option<mpsc::Sender<InboundMessage>>>,
    /// Inbound messages whose command has a registered callback.
    callback_queue: Arc<Mutex<VecDeque<InboundMessage>>>,
    /// Pending one-shot waiters registered by [`Self::send_and_wait`].
    waiters: Arc<Mutex<Vec<PendingWaiter>>>,
}

/// Parsed inbound message from the interface.
#[derive(Debug, Clone)]
pub struct InboundMessage {
    /// Msg id (use `Parser::id_to_name` to resolve to a human-readable name).
    pub id: u8,
    /// Decoded parameter values in command definition order.
    pub params: Vec<ArgValue>,
}

/// A one-shot waiter registered by [`Parser::send_and_wait`].
struct PendingWaiter {
    /// Unique id used to remove exactly this waiter — several concurrent
    /// waiters may share the same message id.
    id: u64,
    /// Id of the awaited inbound message.
    msg_id: u8,
    /// Channel used to deliver the decoded parameter values.
    tx: oneshot::Sender<Vec<ArgValue>>,
}

/// Monotonic counter assigning a unique id to every pending waiter.
static NEXT_WAITER_ID: AtomicU64 = AtomicU64::new(0);

impl<I: KlippyInterface + 'static> Parser<I> {
    /// Create a new parser and register default message formats.
    pub fn new(interface: I) -> Self {
        let msgs: MsgRegistry = Arc::new(Mutex::new(MultiIndexMsgMap::default()));
        let parser = Self {
            msgs,
            interface,
            outbox: AsyncMutex::new(None),
            inbound_tx: AsyncMutex::new(None),
            callback_queue: Arc::new(Mutex::new(VecDeque::new())),
            waiters: Arc::new(Mutex::new(Vec::new())),
        };
        parser
    }

    /// Register a message format with the given ID.
    ///
    /// The format string is parsed into a [`MsgBase`] and stored under both
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

        let mut map = self.msgs.lock()
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
        let mut map = self.msgs.lock()
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

    /// Send a command with the given name and parameter values.
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
    /// Returns `MsgError` if:
    /// - The command name is not found in the registry
    /// - The command is a `Handler` type (only `Base` msgs can be sent)
    /// - A positional param appears after a named param
    /// - The same named param is provided twice, or one param is provided
    ///   both positionally and by name
    /// - A named param does not match any parameter defined by the command
    /// - Positional params exceed the command's parameter count, or a
    ///   required param is missing
    /// - Parameter types don't match the command definition and no lossless
    ///   conversion is possible
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
    pub async fn send(&self, cmd_name: &str, params: &[Param]) -> MsgResult<()> {
        // Look up the command and extract the id and parameter definitions.
        // The lock is scoped so the guard is dropped before any `.await` —
        // holding a std Mutex guard across an await could block a
        // single-threaded executor once the inbox task also locks `msgs`.
        let (cmd_id, param_defs): (u8, Vec<(String, ArgType)>) = {
            let guard = self
                .msgs
                .lock()
                .map_err(|_| MsgError::new("msgs lock poisoned"))?;
            let cmd = guard
                .get_by_name(cmd_name)
                .ok_or_else(|| MsgError::new(format!("Unknown command: {}", cmd_name)))?;

            match &cmd.command {
                MsgEntry::Base(base) => (cmd.id, base.params().to_vec()),
                MsgEntry::Handler(_) => {
                    return Err(MsgError::new(format!(
                        "Cannot send Handler type command: {}",
                        cmd_name
                    )));
                }
            }
        }; // guard dropped here

        // Split params into positional values and a name→value map,
        // rejecting ordering violations and duplicates up front.
        let mut positional: Vec<&ArgValue> = Vec::with_capacity(params.len());
        let mut named_map: HashMap<&str, &ArgValue> = HashMap::new();
        let mut named_started = false;

        for param in params {
            match param {
                Param::Positional(v) => {
                    if named_started {
                        return Err(MsgError::new(format!(
                            "Positional param after named param for '{}': positional params must come first",
                            cmd_name
                        )));
                    }
                    positional.push(v);
                }
                Param::Named(name, v) => {
                    named_started = true;
                    if named_map.insert(name.as_str(), v).is_some() {
                        return Err(MsgError::new(format!(
                            "Duplicate named param '{}' for '{}'",
                            name, cmd_name
                        )));
                    }
                }
            }
        }

        // Validate positional params count
        if positional.len() > param_defs.len() {
            return Err(MsgError::new(format!(
                "Too many positional params for '{}': expected at most {}, got {}",
                cmd_name,
                param_defs.len(),
                positional.len()
            )));
        }

        // Every named param must reference a parameter defined by the command.
        for name in named_map.keys() {
            if !param_defs.iter().any(|(def_name, _)| def_name.as_str() == *name) {
                return Err(MsgError::new(format!(
                    "Unknown param '{}' for '{}'",
                    name, cmd_name
                )));
            }
        }

        // Build the final parameter list in command definition order
        let mut final_params: Vec<ArgValue> = Vec::with_capacity(param_defs.len());

        for (i, (param_name, expected_type)) in param_defs.iter().enumerate() {
            let value = if i < positional.len() {
                if named_map.contains_key(param_name.as_str()) {
                    return Err(MsgError::new(format!(
                        "Param '{}' for '{}' provided both positionally and by name",
                        param_name, cmd_name
                    )));
                }
                positional[i].clone()
            } else if let Some(&named_value) = named_map.get(param_name.as_str()) {
                // Named param - look up by name
                named_value.clone()
            } else {
                return Err(MsgError::new(format!(
                    "Missing required param '{}' for '{}'",
                    param_name, cmd_name
                )));
            };

            // Validate type — attempt conversion if types don't match
            if value.arg_type() != *expected_type {
                if let Ok(converted) = value.try_convert_to(*expected_type) {
                    tracing::debug!(
                        "param type conversion for '{}' param '{}': {:?} -> {:?}",
                        cmd_name,
                        param_name,
                        value.arg_type(),
                        expected_type
                    );
                    final_params.push(converted);
                } else {
                    return Err(MsgError::new(format!(
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
        payload.push(cmd_id)?;
        for value in final_params {
            payload.push_value(&value)?;
        }

        tracing::debug!(
            "[MSG] SEND {} (id={}) {} bytes: {}",
            cmd_name,
            cmd_id,
            payload.len(),
            hex::encode(payload.payload())
        );

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
            .map_err(|_| MsgError::new("outbox closed"))?;
        ack_rx
            .await
            .map_err(|_| MsgError::new("outbox task terminated"))??;

        Ok(())
    }

    /// Synchronously drain the receive queue of messages that have a
    /// registered callback.
    ///
    /// While the inbox is running (started automatically via [`Self::ensure_inbox`]),
    /// every inbound message whose command was bound to a callback via
    /// [`Self::bind`] is queued internally instead of being forwarded to the
    /// inbox channel. This method returns all currently queued messages and
    /// removes them from the queue.
    ///
    /// The call is synchronous and never waits: it returns immediately with
    /// whatever is queued at the moment of the call (possibly an empty vec).
    pub fn take_callback_msgs(&self) -> MsgResult<Vec<InboundMessage>> {
        let mut queue = self
            .callback_queue
            .lock()
            .map_err(|_| MsgError::new("callback queue lock poisoned"))?;
        Ok(queue.drain(..).collect())
    }

    /// Send a command and wait for a specific inbound message, returning the
    /// parameter values attached to that message.
    ///
    /// A one-shot waiter for `wait_name` is registered *before* the command
    /// is sent, so a fast response cannot be missed. When the inbox task
    /// receives a message named `wait_name`, its decoded parameter values —
    /// in the order defined by the message format — are delivered to the
    /// waiter and become the return value of this method.
    ///
    /// A waited message takes precedence over a registered callback: if the
    /// message was also bound via [`Self::bind`], it is delivered here and
    /// is not queued for [`Self::take_callback_msgs`].
    ///
    /// # Arguments
    /// * `cmd_name` - The command to send (same semantics as [`Self::send`]).
    /// * `params` - Parameters for the command.
    /// * `wait_name` - Name of the inbound message to wait for.
    /// * `timeout` - Optional maximum wait duration; `None` waits indefinitely.
    ///
    /// # Errors
    /// Returns `MsgError` if:
    /// - The send fails (same errors as [`Self::send`]); the waiter is removed
    /// - The timeout expires; the waiter is removed
    /// - The inbox task terminated before the message arrived
    /// - The `wait_name` is not a registered message
    ///
    /// # Example
    /// ```ignore
    /// let params = parser
    ///     .send_and_wait("M105", &[], "temperature", Some(Duration::from_secs(1)))
    ///     .await?;
    /// ```
    pub async fn send_and_wait(
        &self,
        cmd_name: &str,
        params: &[Param],
        wait_name: &str,
        timeout: Option<Duration>,
    ) -> MsgResult<Vec<ArgValue>> {
        self.ensure_inbox().await?;

        // Resolve wait_name → id so the waiter can be matched against the
        // inbound cmd_id without re-doing a name comparison for every payload.
        let wait_id: u8 = {
            let guard = self
                .msgs
                .lock()
                .map_err(|_| MsgError::new("msgs lock poisoned"))?;
            guard
                .get_by_name(wait_name)
                .map(|c| c.id)
                .ok_or_else(|| MsgError::new(format!("Unknown wait message: {}", wait_name)))?
        };

        // Register the waiter before sending so a fast response cannot
        // arrive between the send and the registration.
        let (tx, rx) = oneshot::channel();
        let waiter_id = NEXT_WAITER_ID.fetch_add(1, Ordering::Relaxed);
        {
            let mut waiters = self
                .waiters
                .lock()
                .map_err(|_| MsgError::new("waiters lock poisoned"))?;
            waiters.push(PendingWaiter {
                id: waiter_id,
                msg_id: wait_id,
                tx,
            });
        }

        // Send the command; on failure the waiter must not linger.
        if let Err(e) = self.send(cmd_name, params).await {
            self.remove_waiter(waiter_id);
            return Err(e);
        }

        let received = match timeout {
            Some(duration) => match tokio::time::timeout(duration, rx).await {
                Ok(result) => result,
                Err(_) => {
                    self.remove_waiter(waiter_id);
                    return Err(MsgError::new(format!(
                        "timeout waiting for message '{}'",
                        wait_name
                    )));
                }
            },
            None => rx.await,
        };

        received.map_err(|_| MsgError::new("inbox task terminated"))
    }

    /// Remove a single pending waiter by id (no-op if absent).
    fn remove_waiter(&self, id: u64) {
        if let Ok(mut waiters) = self.waiters.lock() {
            if let Some(pos) = waiters.iter().position(|w| w.id == id) {
                waiters.remove(pos);
            }
        }
    }

    /// Lazily start the outbox task and return its sender.
    ///
    /// The outbox task is the single writer to the interface, so outbound
    /// payloads are always transmitted in call order even when coalescing
    /// batches multiple msgs into one frame.
    async fn ensure_outbox(&self) -> mpsc::Sender<OutItem> {
        let mut guard = self.outbox.lock().await;
        if let Some(sender) = guard.as_ref() {
            return sender.clone();
        }
        let (sender, receiver) = mpsc::channel::<OutItem>(64);
        let interface = self.interface.clone();
        tokio::spawn(async move { run_sender(interface, receiver).await });
        let cloned = sender.clone();
        *guard = Some(sender);
        cloned
    }

    /// Lazily start the inbox task if not already running.
    ///
    /// The inbox task continuously receives payloads from the interface,
    /// parses them into [`InboundMessage`]s, and dispatches them to
    /// callbacks and waiters.
    ///
    /// This method is automatically called by [`Self::send_and_wait`].
    /// For tests that need to trigger inbox processing without `send_and_wait`,
    /// call this method explicitly before sending messages.
    pub async fn ensure_inbox(&self) -> MsgResult<()> {
        if self.inbound_tx.lock().await.is_some() {
            return Ok(());
        }
        let (tx, _rx) = mpsc::channel(64);
        let interface = self.interface.clone();
        let msgs = self.msgs.clone();
        let callback_queue = Arc::clone(&self.callback_queue);
        let waiters = Arc::clone(&self.waiters);
        let tx_clone = tx.clone();
        tokio::spawn(async move {
            Self::run_inbox(interface, msgs, callback_queue, waiters, tx_clone).await
        });
        *self.inbound_tx.lock().await = Some(tx);
        Ok(())
    }

    /// Background task that continuously receives and parses inbound messages.
    async fn run_inbox(
        interface: I,
        msgs: MsgRegistry,
        callback_queue: Arc<Mutex<VecDeque<InboundMessage>>>,
        waiters: Arc<Mutex<Vec<PendingWaiter>>>,
        tx: mpsc::Sender<InboundMessage>,
    ) {
        loop {
            match interface.receive().await {
                Ok(payload) => {
                    tracing::debug!(
                        "[INBOX] Received {} bytes: {}",
                        payload.len(),
                        hex::encode(payload.payload())
                    );
                    if let Err(e) = Self::process_single(
                        &msgs,
                        &callback_queue,
                        &waiters,
                        &payload,
                        &tx,
                    )
                    .await
                    {
                        // The inbox channel is this task's only output: once
                        // its receiver is dropped there is no reason to keep
                        // draining the interface.
                        if tx.is_closed() {
                            break;
                        }
                        tracing::error!("[inbox] error processing payload: {}", e);
                    }
                }
                Err(e) => {
                    tracing::error!("[inbox] receive error: {}", e);
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }
        }
    }

    /// Parse a single inbound payload and route the result.
    ///
    /// Routing order:
    /// 1. If [`Parser::send_and_wait`] is waiting for this message name,
    ///    deliver the decoded params to that waiter.
    /// 2. Otherwise, if the command has a registered callback (bound via
    ///    [`Parser::bind`]), enqueue the message on the callback queue for
    ///    [`Parser::take_callback_msgs`].
    /// 3. Otherwise, forward the message to the inbox channel.
    async fn process_single(
        msgs: &MsgRegistry,
        callback_queue: &Arc<Mutex<VecDeque<InboundMessage>>>,
        waiters: &Arc<Mutex<Vec<PendingWaiter>>>,
        payload: &Payload,
        tx: &mpsc::Sender<InboundMessage>,
    ) -> MsgResult<()> {
        let mut parser = payload.as_parser();
        let cmd_id = parser.pop()?;

        // Look up command by id and extract data (lock held briefly)
        let (param_defs, has_callback) = {
            let guard = msgs.lock()
                .map_err(|_| MsgError::new("msgs lock poisoned"))?;
            let cmd = guard
                .get_by_id(&cmd_id)
                .ok_or_else(|| MsgError::new(format!("Unknown command id: {}", cmd_id)))?;

            let (param_defs, has_callback): (Vec<(String, ArgType)>, bool) =
                match &cmd.command {
                    MsgEntry::Base(base) => (base.params().to_vec(), false),
                    MsgEntry::Handler(handler) => (handler.msg().params().to_vec(), true),
                };
            (param_defs, has_callback)
        }; // guard dropped here

        let param_types: Vec<ArgType> = param_defs.iter().map(|(_, t)| *t).collect();
        let params: Vec<ArgValue> = parser.pop_values(&param_types)?;

        // Log the parsed inbound message
        tracing::debug!(
            "[INBOX] Parsed cmd_id={} params={:?}",
            cmd_id, params
        );

        // 1. A pending send_and_wait() waiter takes precedence (FIFO).
        let waiter = {
            let mut guard = waiters.lock()
                .map_err(|_| MsgError::new("waiters lock poisoned"))?;
            guard
                .iter()
                .position(|w| w.msg_id == cmd_id)
                .map(|pos| guard.remove(pos))
        };
        if let Some(waiter) = waiter {
            let _ = waiter.tx.send(params);
            return Ok(());
        }

        // 2. Messages with a registered callback go to the callback queue.
        if has_callback {
            let mut queue = callback_queue.lock()
                .map_err(|_| MsgError::new("callback queue lock poisoned"))?;
            queue.push_back(InboundMessage { id: cmd_id, params });
            return Ok(());
        }

        // 3. Everything else is forwarded to the inbox channel.
        tx.send(InboundMessage { id: cmd_id, params })
            .await
            .map_err(|_| MsgError::new("inbox receiver dropped"))?;

        Ok(())
    }
}

/// One outbound payload together with the completion signal for the caller.
struct OutItem {
    payload: Payload,
    ack: oneshot::Sender<MsgResult<()>>,
}

/// Send `payload` through `interface` and resolve every `ack` with the result.
async fn send_and_ack<I: KlippyInterface>(
    interface: &I,
    payload: Payload,
    acks: Vec<oneshot::Sender<MsgResult<()>>>,
) {
    tracing::debug!(
        "[OUTBOX] Sending {} bytes: {}",
        payload.len(),
        hex::encode(payload.payload())
    );
    let result = match interface.send(&payload).await {
        Ok(()) => Ok(()),
        Err(e) => Err(MsgError::new(e.to_string())),
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
async fn run_sender<I: KlippyInterface>(interface: I, mut rx: mpsc::Receiver<OutItem>) {
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
                            // Merge failed (size limit reached): flush the
                            // pending batch, then handle the payload that
                            // triggered the flush.
                            let pending = std::mem::take(&mut batch);
                            let pending_acks = std::mem::take(&mut acks);
                            send_and_ack(&interface, pending, pending_acks).await;

                            if next_payload.len() >= SEND_COALESCE_THRESHOLD {
                                // Large payloads are sent immediately.
                                send_and_ack(&interface, next_payload, vec![next_ack]).await;
                            } else {
                                // New payload is small: it opens a fresh window.
                                batch = next_payload;
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

    fn register_g1_cmd<I: KlippyInterface + 'static>(parser: &mut Parser<I>) {
        // Register "G1 X=%u Y=%u" with id=3
        let _ = parser.register(3, "G1 X=%u Y=%u");
    }

    fn register_m105_cmd<I: KlippyInterface + 'static>(parser: &mut Parser<I>) {
        // Register "M105" with no params
        let _ = parser.register(5, "M105");
    }

    #[tokio::test]
    async fn test_send_g1_with_params() {
        let expected = build_g1_payload(100, 200);
        let mapping = vec![MappingEntry {
            input: Frame::new(0, expected.payload().to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
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
        let parser = Parser::new(interface);

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
        let mut parser = Parser::new(interface);
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
        let mut parser = Parser::new(interface);

        // Register a command with a string param
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
        let mut parser = Parser::new(interface);
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
        let mut parser = Parser::new(interface);

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
        let mut parser = Parser::new(interface);

        // Register a command with a bytes param
        let _ = parser.register(11, "TEST data=%.*s");

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
        let mut parser = Parser::new(interface);

        // Register a command with mixed types: uint32, string, bytes
        let _ = parser.register(13, "CMD oid=%u label=%s raw=%.*s");

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
        let mut parser = Parser::new(interface);
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
        let mut parser = Parser::new(interface);
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
        let mut parser = Parser::new(interface);
        register_g1_cmd(&mut parser);

        // Use unknown param name
        let params = vec![Param::Named("Z".to_string(), ArgValue::UInt32(100))];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown param 'Z'"));
    }

    #[tokio::test]
    async fn test_send_unknown_named_param_with_complete_positional() {
        // All required params are covered positionally, but an extra named
        // param must still be rejected instead of being silently dropped.
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_g1_cmd(&mut parser);

        let params = vec![
            Param::Positional(ArgValue::UInt32(100)),
            Param::Positional(ArgValue::UInt32(200)),
            Param::Named("Z".to_string(), ArgValue::UInt32(300)),
        ];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown param 'Z'"));
    }

    #[tokio::test]
    async fn test_send_positional_after_named() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_g1_cmd(&mut parser);

        // Positional params must come before named params.
        let params = vec![
            Param::Named("X".to_string(), ArgValue::UInt32(100)),
            Param::Positional(ArgValue::UInt32(200)),
        ];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        let err_msg = result.unwrap_err().msg;
        assert!(err_msg.contains("positional params must come first"), "Error message: {}", err_msg);
    }

    #[tokio::test]
    async fn test_send_duplicate_named_param() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_g1_cmd(&mut parser);

        let params = vec![
            Param::Named("X".to_string(), ArgValue::UInt32(100)),
            Param::Named("X".to_string(), ArgValue::UInt32(200)),
            Param::Named("Y".to_string(), ArgValue::UInt32(300)),
        ];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Duplicate named param 'X'"));
    }

    #[tokio::test]
    async fn test_send_param_both_positional_and_named() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_g1_cmd(&mut parser);

        let params = vec![
            Param::Positional(ArgValue::UInt32(100)), // X
            Param::Named("X".to_string(), ArgValue::UInt32(999)),
            Param::Named("Y".to_string(), ArgValue::UInt32(200)),
        ];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        let err_msg = result.unwrap_err().msg;
        assert!(
            err_msg.contains("both positionally and by name"),
            "Error message: {}",
            err_msg
        );
    }

    #[tokio::test]
    async fn test_send_conversion_out_of_range_rejected() {
        // u32::MAX cannot convert to Int32 losslessly: must be a type
        // mismatch error, not a wrapped negative value.
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);

        let _ = parser.register(23, "CMD val=%i");

        let params = vec![Param::Positional(ArgValue::UInt32(u32::MAX))];
        let result = parser.send("CMD", &params).await;
        assert!(result.is_err());
        let err_msg = result.unwrap_err().msg;
        assert!(err_msg.contains("Param type mismatch"), "Error message: {}", err_msg);
    }

    #[tokio::test]
    async fn test_send_negative_to_unsigned_rejected() {
        // A negative value must not silently wrap into a huge unsigned one.
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);

        let _ = parser.register(25, "CMD val=%u");

        let params = vec![Param::Positional(ArgValue::Int32(-5))];
        let result = parser.send("CMD", &params).await;
        assert!(result.is_err());
        let err_msg = result.unwrap_err().msg;
        assert!(err_msg.contains("Param type mismatch"), "Error message: {}", err_msg);
    }

    #[test]
    fn test_remove_waiter_by_id_only_removes_target() {
        // Two waiters share the same message name; removing one by id must
        // leave the other untouched.
        let interface = TestInterface::new(vec![]);
        let parser = Parser::new(interface);

        let (tx1, _rx1) = oneshot::channel();
        let (tx2, _rx2) = oneshot::channel();
        let id1 = NEXT_WAITER_ID.fetch_add(1, Ordering::Relaxed);
        let id2 = NEXT_WAITER_ID.fetch_add(1, Ordering::Relaxed);

        parser.waiters.lock().unwrap().push(PendingWaiter {
            id: id1,
            msg_id: 32,
            tx: tx1,
        });
        parser.waiters.lock().unwrap().push(PendingWaiter {
            id: id2,
            msg_id: 32,
            tx: tx2,
        });

        // The second waiter is removed; the first must remain.
        parser.remove_waiter(id2);
        let waiters = parser.waiters.lock().unwrap();
        assert_eq!(waiters.len(), 1);
        assert_eq!(waiters[0].id, id1);
    }

    #[tokio::test]
    async fn test_send_too_many_positional() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
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
    async fn test_send_multiple_msgs() {
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
        let mut parser = Parser::new(interface);
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
        let mut parser = Parser::new(interface);

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
        let mut parser = Parser::new(interface);

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
        let mut parser = Parser::new(interface);

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
        let mut parser = Parser::new(interface);

        let _ = parser.register(21, "CMD val=%s");

        // Send UInt32 instead of Str — conversion not possible
        let params = vec![Param::Positional(ArgValue::UInt32(42))];
        let result = parser.send("CMD", &params).await;
        assert!(result.is_err());
        let err_msg = result.unwrap_err().msg;
        assert!(err_msg.contains("Param type mismatch"), "Error message: {}", err_msg);
    }

    #[tokio::test]
    async fn test_bind_command() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_g1_cmd(&mut parser);

        // Bind a callback to the G1 command.
        let called = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let called_clone = Arc::clone(&called);
        parser
            .bind("G1", move |_values| {
                called_clone.store(true, std::sync::atomic::Ordering::SeqCst);
            })
            .unwrap();

        // The command is now a Handler. Keep the guard in its own scope so it
        // is released before `send()` locks the same (non-reentrant) mutex.
        {
            let guard = parser.msgs.lock().unwrap();
            let cmd = guard.get_by_name("G1").unwrap();
            assert!(matches!(cmd.command, MsgEntry::Handler(_)));
        }

        // A bound command can no longer be sent.
        let params = vec![
            Param::Positional(ArgValue::UInt32(100)),
            Param::Positional(ArgValue::UInt32(200)),
        ];
        let result = parser.send("G1", &params).await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Handler"));

        // The callback was only stored, not invoked (no message received).
        assert!(!called.load(std::sync::atomic::Ordering::SeqCst));
    }

    #[tokio::test]
    async fn test_bind_unknown_command() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, Vec::new()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);

        let result = parser.bind("UNKNOWN", |_values| {});
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown command"));
    }

    // -----------------------------------------------------------------------
    // take_callback_msgs / send_and_wait
    // -----------------------------------------------------------------------

    /// Poll `take_callback_msgs` until `count` messages have been collected.
    async fn collect_callback_msgs<I: KlippyInterface + 'static>(parser: &Parser<I>, count: usize) -> Vec<InboundMessage> {
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        let mut collected = Vec::new();
        while collected.len() < count {
            collected.extend(parser.take_callback_msgs().unwrap());
            if collected.len() >= count {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "timed out waiting for callback messages"
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        collected
    }

    #[tokio::test]
    async fn test_take_callback_msgs_empty() {
        let interface = TestInterface::new(vec![]);
        let parser = Parser::new(interface);
        assert!(parser.take_callback_msgs().unwrap().is_empty());
    }

    #[tokio::test]
    async fn test_callback_msg_routing() {
        // One send triggers two inbound frames: a bound message (goes to the
        // callback queue) and an unbound message (forwarded to the inbox
        // channel).
        let mut m105_payload = Payload::new();
        m105_payload.push(5).unwrap();

        let mut temp_report = Payload::new();
        temp_report.push(31).unwrap();
        temp_report.push_u32(250).unwrap();

        let mut test_resp = Payload::new();
        test_resp.push(99).unwrap();
        test_resp.push_u32(4).unwrap();
        test_resp.push_bytes(b"abcd").unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, m105_payload.payload().to_vec()),
            outputs: vec![
                Frame::new(0, temp_report.payload().to_vec()),
                Frame::new(1, test_resp.payload().to_vec()),
            ],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_m105_cmd(&mut parser);
        // Register test_response so inbox task can process id=99 messages
        parser.register(99, "test_response value=%u data=%.*s").unwrap();

        // Bind a callback to the inbound message.
        let _ = parser.register(31, "temp_report value=%u");
        parser.bind("temp_report", |_values| {}).unwrap();

        // Ensure inbox is running for callback processing.
        parser.ensure_inbox().await.unwrap();

        // Trigger the two inbound frames.
        parser.send("M105", &[]).await.unwrap();

        // The bound message lands in the callback queue.
        let msgs = collect_callback_msgs(&parser, 1).await;
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].id, 31);
        assert_eq!(msgs[0].params, vec![ArgValue::UInt32(250)]);

        // Nothing else was queued.
        assert!(parser.take_callback_msgs().unwrap().is_empty());
    }

    #[tokio::test]
    async fn test_send_and_wait_returns_params() {
        let mut m105_payload = Payload::new();
        m105_payload.push(5).unwrap();

        let mut temperature = Payload::new();
        temperature.push(32).unwrap();
        temperature.push_u32(77).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, m105_payload.payload().to_vec()),
            outputs: vec![Frame::new(0, temperature.payload().to_vec())],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_m105_cmd(&mut parser);
        let _ = parser.register(32, "temperature value=%u");

        let params = parser
            .send_and_wait("M105", &[], "temperature", Some(Duration::from_secs(1)))
            .await
            .unwrap();
        assert_eq!(params, vec![ArgValue::UInt32(77)]);

        // The waiter was consumed.
        assert!(parser.waiters.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn test_send_and_wait_precedence_over_callback() {
        // Same as above, but the waited message is also bound to a callback:
        // the waiter wins and the callback queue stays empty.
        let mut m105_payload = Payload::new();
        m105_payload.push(5).unwrap();

        let mut temperature = Payload::new();
        temperature.push(32).unwrap();
        temperature.push_u32(77).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, m105_payload.payload().to_vec()),
            outputs: vec![Frame::new(0, temperature.payload().to_vec())],
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_m105_cmd(&mut parser);
        let _ = parser.register(32, "temperature value=%u");
        parser.bind("temperature", |_values| {}).unwrap();

        let params = parser
            .send_and_wait("M105", &[], "temperature", Some(Duration::from_secs(1)))
            .await
            .unwrap();
        assert_eq!(params, vec![ArgValue::UInt32(77)]);
        assert!(parser.take_callback_msgs().unwrap().is_empty());
    }

    #[tokio::test]
    async fn test_send_and_wait_timeout() {
        let mut m105_payload = Payload::new();
        m105_payload.push(5).unwrap();

        let mapping = vec![MappingEntry {
            input: Frame::new(0, m105_payload.payload().to_vec()),
            outputs: vec![], // no response
        }];
        let interface = TestInterface::new(mapping);
        let mut parser = Parser::new(interface);
        register_m105_cmd(&mut parser);
        let _ = parser.register(32, "temperature value=%u");

        let result = parser
            .send_and_wait("M105", &[], "temperature", Some(Duration::from_millis(100)))
            .await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("timeout waiting"));

        // The waiter was removed after the timeout.
        assert!(parser.waiters.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn test_send_and_wait_send_failure_removes_waiter() {
        // The send fails because the command is unknown; the registered
        // waiter must not linger.
        let interface = TestInterface::new(vec![]);
        let mut parser = Parser::new(interface);
        // Register a test response so send_and_wait can resolve the wait target
        parser.register(99, "test_response value=%u").unwrap();

        // Use "test_response" (id=99) — registered above.
        // The send fails with "Unknown command" (UNKNOWN cmd), not the wait lookup.
        let result = parser
            .send_and_wait("UNKNOWN", &[], "test_response", Some(Duration::from_secs(1)))
            .await;
        assert!(result.is_err());
        assert!(result.unwrap_err().msg.contains("Unknown command"));
        assert!(parser.waiters.lock().unwrap().is_empty());
    }
}
