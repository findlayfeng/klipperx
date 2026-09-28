//! Bookkeeping for synchronous request/response calls ([`Mcu::call`](super::Mcu::call)).
//!
//! A synchronous call registers a [`PendingCall`] before the command is sent and
//! waits on the returned [`oneshot`] channel. The receive loop resolves the
//! registration when a message with the expected response name arrives. All
//! registry locking is encapsulated here so the MCU orchestration code never
//! touches the mutex directly.

use crate::core::klippy::msg::proto::ArgValue;
use std::sync::Arc;
use tokio::sync::{oneshot, Mutex};

/// A single synchronous call waiting for a response from the MCU.
struct PendingCall {
    /// Name of the response message to match against.
    response_name: String,
    /// Sender used to deliver the decoded parameters (or drop on timeout).
    response_tx: oneshot::Sender<Vec<ArgValue>>,
}

/// Registry of in-flight synchronous calls.
///
/// Cheap to clone: every clone shares the same underlying registry, which lets
/// the receive task and the caller observe the same set of pending calls.
#[derive(Clone, Default)]
pub(crate) struct PendingCalls {
    calls: Arc<Mutex<Vec<PendingCall>>>,
}

impl PendingCalls {
    /// Create an empty registry.
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Register a call that is waiting for `response_name`.
    pub(crate) async fn register(
        &self,
        response_name: String,
        response_tx: oneshot::Sender<Vec<ArgValue>>,
    ) {
        self.calls.lock().await.push(PendingCall {
            response_name,
            response_tx,
        });
    }

    /// Deliver `params` to the oldest call waiting for `response_name`.
    ///
    /// Returns `true` when a call was matched and consumed, in which case the
    /// caller must **not** fall back to the regular callback dispatch. Returns
    /// `false` when nobody is waiting, which is the common case.
    ///
    /// The parameters are only cloned on a match, so the unmatched path — every
    /// asynchronous message — pays nothing.
    ///
    /// # Ordering
    /// Matching is by response name only. Two concurrent calls waiting on the
    /// same response name are therefore resolved first-come-first-served; callers
    /// that need them told apart must discriminate on a tag parameter (e.g.
    /// `oid`) themselves.
    pub(crate) async fn resolve(&self, response_name: &str, params: &[ArgValue]) -> bool {
        let mut calls = self.calls.lock().await;
        let Some(idx) = calls
            .iter()
            .position(|call| call.response_name == response_name)
        else {
            return false;
        };
        let call = calls.remove(idx);
        // A send failure means the receiver is gone already; that is not an error.
        let _ = call.response_tx.send(params.to_vec());
        true
    }

    /// Drop **every** in-flight call, so each waiter returns at once.
    ///
    /// Dropping a registration drops its `oneshot::Sender`, which resolves the
    /// waiter with `Err(RecvError)` — the "receiver dropped" outcome
    /// [`Mcu::call`](super::Mcu::call) already reports as a send failure, not
    /// the timeout it would otherwise sit out. Used when the connection itself
    /// has made every waiting call moot: a firmware that reported a stop during
    /// the connect handshake will not answer any of them.
    pub(crate) async fn abort_all(&self) {
        let mut calls = self.calls.lock().await;
        calls.clear();
    }

    /// Drop the oldest call registered for `response_name`, if any.
    ///
    /// Used to clean up after a send failure, a dropped receiver, or a timeout,
    /// so that a late response is not mis-delivered to a stale registration.
    pub(crate) async fn cancel(&self, response_name: &str) {
        let mut calls = self.calls.lock().await;
        if let Some(idx) = calls
            .iter()
            .position(|call| call.response_name == response_name)
        {
            calls.remove(idx);
        }
    }

    /// Number of in-flight calls. Intended for tests and diagnostics.
    #[cfg(test)]
    pub(crate) async fn len(&self) -> usize {
        self.calls.lock().await.len()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> Vec<ArgValue> {
        vec![ArgValue::UInt32(42)]
    }

    #[tokio::test]
    async fn test_register_and_resolve() {
        let calls = PendingCalls::new();
        let (tx, rx) = oneshot::channel();

        calls.register("clock".to_string(), tx).await;
        assert!(calls.resolve("clock", &params()).await);
        assert_eq!(rx.await.unwrap(), params());
    }

    #[tokio::test]
    async fn test_resolve_unknown_name_does_not_consume() {
        let calls = PendingCalls::new();
        let (tx, rx) = oneshot::channel();

        calls.register("clock".to_string(), tx).await;
        assert!(!calls.resolve("uptime", &params()).await);
        assert_eq!(calls.len().await, 1);

        // The registration is still resolvable afterwards.
        assert!(calls.resolve("clock", &params()).await);
        assert_eq!(rx.await.unwrap(), params());
    }

    #[tokio::test]
    async fn test_resolve_is_first_come_first_served() {
        let calls = PendingCalls::new();
        let (tx1, rx1) = oneshot::channel();
        let (tx2, rx2) = oneshot::channel();

        calls.register("clock".to_string(), tx1).await;
        calls.register("clock".to_string(), tx2).await;

        assert!(calls.resolve("clock", &[ArgValue::UInt32(1)]).await);
        assert!(calls.resolve("clock", &[ArgValue::UInt32(2)]).await);

        assert_eq!(rx1.await.unwrap(), vec![ArgValue::UInt32(1)]);
        assert_eq!(rx2.await.unwrap(), vec![ArgValue::UInt32(2)]);
        assert_eq!(calls.len().await, 0);
    }

    #[tokio::test]
    async fn test_abort_all_wakes_every_waiter_at_once() {
        // The registrations go, and with them their senders: each waiter
        // resolves with the dropped-receiver outcome instead of waiting out its
        // own timeout.
        let calls = PendingCalls::new();
        let (config_tx, config_rx) = oneshot::channel();
        let (clock_tx, clock_rx) = oneshot::channel();

        calls.register("config".to_string(), config_tx).await;
        calls.register("clock".to_string(), clock_tx).await;
        calls.abort_all().await;

        assert_eq!(calls.len().await, 0);
        for (response, rx) in [("config", config_rx), ("clock", clock_rx)] {
            let outcome = tokio::time::timeout(std::time::Duration::from_millis(50), rx)
                .await
                .expect("each waiter returns at once, not after a timeout");
            assert!(outcome.is_err(), "'{response}': the sender was dropped");
        }
        // Nothing is left for a late response to be mis-delivered to.
        assert!(!calls.resolve("config", &params()).await);
    }

    #[tokio::test]
    async fn test_cancel_removes_registration() {
        let calls = PendingCalls::new();
        let (tx, _rx) = oneshot::channel();

        calls.register("clock".to_string(), tx).await;
        calls.cancel("clock").await;

        assert_eq!(calls.len().await, 0);
        assert!(!calls.resolve("clock", &params()).await);
    }

    #[tokio::test]
    async fn test_cancel_only_removes_one_registration() {
        let calls = PendingCalls::new();
        let (tx1, _rx1) = oneshot::channel();
        let (tx2, _rx2) = oneshot::channel();

        calls.register("clock".to_string(), tx1).await;
        calls.register("clock".to_string(), tx2).await;
        calls.cancel("clock").await;

        assert_eq!(calls.len().await, 1);
    }

    #[tokio::test]
    async fn test_resolve_after_receiver_dropped_is_not_an_error() {
        let calls = PendingCalls::new();
        let (tx, rx) = oneshot::channel::<Vec<ArgValue>>();
        drop(rx);

        calls.register("clock".to_string(), tx).await;
        assert!(calls.resolve("clock", &params()).await);
        assert_eq!(calls.len().await, 0);
    }
}
