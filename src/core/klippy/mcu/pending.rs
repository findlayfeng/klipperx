//! Bookkeeping for synchronous request/response calls ([`Mcu::call`](super::Mcu::call)).
//!
//! A synchronous call registers a [`PendingCall`] before the command is sent and
//! waits on the returned [`oneshot`] channel. The receive loop resolves the
//! registration when a message with the expected response name arrives — and,
//! for an `oid`-filtered call
//! ([`Mcu::call_oid`](super::Mcu::call_oid)), only when that message carries the
//! call's `oid`. All registry locking is encapsulated here so the MCU
//! orchestration code never touches the mutex directly.

use crate::core::klippy::msg::proto::ArgValue;
use std::sync::Arc;
use tokio::sync::{oneshot, Mutex};
use tracing::debug;

/// A single synchronous call waiting for a response from the MCU.
struct PendingCall {
    /// Name of the response message to match against.
    response_name: String,
    /// The `oid` this call's response must carry, when the answer is told
    /// apart from other traffic of the same name (upstream
    /// `serial.register_response(cb, name, oid)`).
    oid: Option<u8>,
    /// Sender used to deliver the decoded parameters (or drop on timeout).
    response_tx: oneshot::Sender<Vec<ArgValue>>,
}

/// Whether `params` start with `oid` — the position the firmware declares it
/// in, e.g. `tmc_uart_read_response oid=%c data=%*s`.
fn carries_oid(params: &[ArgValue], oid: u8) -> bool {
    matches!(params.first(), Some(ArgValue::UInt8(value)) if *value == oid)
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

    /// Register a call that is waiting for `response_name`, optionally told
    /// apart from its siblings by `oid`.
    pub(crate) async fn register(
        &self,
        response_name: String,
        oid: Option<u8>,
        response_tx: oneshot::Sender<Vec<ArgValue>>,
    ) {
        self.calls.lock().await.push(PendingCall {
            response_name,
            oid,
            response_tx,
        });
    }

    /// Deliver `params` to the call waiting for `response_name` it belongs to.
    ///
    /// Returns `true` when the message is spent, in which case the caller must
    /// **not** fall back to the regular callback dispatch. Returns `false` when
    /// nobody is waiting, which is the common case.
    ///
    /// The parameters are only cloned on a match, so the unmatched path — every
    /// asynchronous message — pays nothing.
    ///
    /// # Ordering
    /// An `oid`-filtered call only accepts a message whose first parameter is
    /// that `oid`; a plain call accepts anything, which keeps the
    /// first-come-first-served rule it always had. Same-name calls are searched
    /// in registration order, so a plain call waiting alongside filtered ones
    /// takes the next message whatever its `oid`.
    ///
    /// A message whose `oid` no waiting call claims is **dropped**: upstream
    /// dispatches `oid`-registered callbacks by exact `oid`
    /// (`serial.register_response(cb, name, oid)`), so an answer nobody on this
    /// connection asked for is not handed to the next caller in line — that
    /// would be another peripheral's data.
    ///
    /// A registration whose receiver is gone goes first: a [`Mcu::call`] whose
    /// future was dropped — the caller was cancelled, its task aborted — leaves
    /// its registration behind, and nobody will ever read the answer. Matching
    /// it would spend the next caller's answer on a waiter that is not there
    /// any more, so such registrations are dropped here, before anything is
    /// matched. Their message then takes the ordinary path (nobody is waiting
    /// for it), which is what the caller that left asked for by leaving.
    ///
    /// [`Mcu::call`]: super::Mcu::call
    pub(crate) async fn resolve(&self, response_name: &str, params: &[ArgValue]) -> bool {
        let mut calls = self.calls.lock().await;
        calls.retain(|call| !call.response_tx.is_closed());
        let mut waiting = false;
        let mut matched = None;
        for (idx, call) in calls.iter().enumerate() {
            if call.response_name != response_name {
                continue;
            }
            waiting = true;
            if call.oid.is_none_or(|oid| carries_oid(params, oid)) {
                matched = Some(idx);
                break;
            }
        }
        let Some(idx) = matched else {
            if waiting {
                debug!(
                    "Discarding '{response_name}' with oid {:?}: no pending call carries it",
                    params.first()
                );
            }
            return waiting;
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

    /// Drop the oldest call registered for `response_name` and `oid`, if any.
    ///
    /// Used to clean up after a send failure, a dropped receiver, or a timeout,
    /// so that a late response is not mis-delivered to a stale registration.
    /// The `oid` is part of the key: a call cancelled for `oid` 1 leaves the one
    /// waiting for `oid` 2 alone.
    pub(crate) async fn cancel(&self, response_name: &str, oid: Option<u8>) {
        let mut calls = self.calls.lock().await;
        if let Some(idx) = calls
            .iter()
            .position(|call| call.response_name == response_name && call.oid == oid)
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

        calls.register("clock".to_string(), None, tx).await;
        assert!(calls.resolve("clock", &params()).await);
        assert_eq!(rx.await.unwrap(), params());
    }

    #[tokio::test]
    async fn test_resolve_unknown_name_does_not_consume() {
        let calls = PendingCalls::new();
        let (tx, rx) = oneshot::channel();

        calls.register("clock".to_string(), None, tx).await;
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

        calls.register("clock".to_string(), None, tx1).await;
        calls.register("clock".to_string(), None, tx2).await;

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

        calls.register("config".to_string(), None, config_tx).await;
        calls.register("clock".to_string(), None, clock_tx).await;
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

        calls.register("clock".to_string(), None, tx).await;
        calls.cancel("clock", None).await;

        assert_eq!(calls.len().await, 0);
        assert!(!calls.resolve("clock", &params()).await);
    }

    #[tokio::test]
    async fn test_cancel_only_removes_one_registration() {
        let calls = PendingCalls::new();
        let (tx1, _rx1) = oneshot::channel();
        let (tx2, _rx2) = oneshot::channel();

        calls.register("clock".to_string(), None, tx1).await;
        calls.register("clock".to_string(), None, tx2).await;
        calls.cancel("clock", None).await;

        assert_eq!(calls.len().await, 1);
    }

    /// A registration whose receiver is gone is no waiter at all: it is
    /// dropped before anything is matched, so its message takes the ordinary
    /// path instead of being spent on a call that already left.
    #[tokio::test]
    async fn test_a_dropped_receiver_leaves_no_waiter_behind() {
        let calls = PendingCalls::new();
        let (tx, rx) = oneshot::channel::<Vec<ArgValue>>();
        drop(rx);

        calls.register("clock".to_string(), None, tx).await;
        assert!(
            !calls.resolve("clock", &params()).await,
            "nobody is waiting: the registration's caller left"
        );
        assert_eq!(calls.len().await, 0, "and it is not kept either");
    }

    /// The shape the rule guards: a [`Mcu::call`](super::Mcu::call) whose
    /// future was dropped (its task cancelled) leaves its registration behind,
    /// and the next caller's answer has to reach that caller — not the departed
    /// one, whose answer nobody will ever read.
    #[tokio::test]
    async fn test_the_next_call_takes_the_answer_past_a_dropped_one() {
        let calls = PendingCalls::new();
        let (dropped_tx, dropped_rx) = oneshot::channel();
        drop(dropped_rx);
        let (live_tx, live_rx) = oneshot::channel();

        calls.register("clock".to_string(), None, dropped_tx).await;
        calls.register("clock".to_string(), None, live_tx).await;

        assert!(calls.resolve("clock", &params()).await);
        let answer = tokio::time::timeout(std::time::Duration::from_millis(50), live_rx)
            .await
            .expect("the caller still waiting is the one that gets it")
            .unwrap();
        assert_eq!(answer, params());
        assert_eq!(calls.len().await, 0);
    }

    // -----------------------------------------------------------------------
    // `oid`-filtered calls
    // -----------------------------------------------------------------------

    /// What `tmc_uart_read_response oid=%c data=%*s` decodes to.
    fn tmc_response(oid: u8) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(oid), ArgValue::Bytes(vec![oid; 2])]
    }

    #[tokio::test]
    async fn test_two_oid_calls_each_take_their_own_response() {
        let calls = PendingCalls::new();
        let (tx1, rx1) = oneshot::channel();
        let (tx2, rx2) = oneshot::channel();

        calls
            .register("tmc_uart_read_response".to_string(), Some(1), tx1)
            .await;
        calls
            .register("tmc_uart_read_response".to_string(), Some(2), tx2)
            .await;

        // The answers arrive in the opposite order to the registrations: the
        // `oid` decides, not who asked first.
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(2))
                .await
        );
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(1))
                .await
        );

        assert_eq!(rx1.await.unwrap(), tmc_response(1));
        assert_eq!(rx2.await.unwrap(), tmc_response(2));
        assert_eq!(calls.len().await, 0);
    }

    #[tokio::test]
    async fn test_an_answer_for_another_oid_reaches_no_pending_call() {
        let calls = PendingCalls::new();
        let (tx, mut rx) = oneshot::channel();

        calls
            .register("tmc_uart_read_response".to_string(), Some(1), tx)
            .await;

        // oid 2's answer is nobody's on this connection. Handing it to the call
        // waiting for oid 1 would be the wrong peripheral's data, so the
        // registration stays put and the answer is dropped.
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(2))
                .await
        );
        assert_eq!(calls.len().await, 1);
        assert!(rx.try_recv().is_err(), "nothing was delivered");

        // Its own answer still resolves it afterwards.
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(1))
                .await
        );
        assert_eq!(rx.await.unwrap(), tmc_response(1));
    }

    #[tokio::test]
    async fn test_an_unanswered_oid_call_still_times_out() {
        let calls = PendingCalls::new();
        let (tx, rx) = oneshot::channel();

        calls
            .register("tmc_uart_read_response".to_string(), Some(1), tx)
            .await;
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(2))
                .await
        );

        // Only a mismatched answer ever arrived, so the call sits out its own
        // deadline — the drop must not resolve it with the wrong data.
        assert!(
            tokio::time::timeout(std::time::Duration::from_millis(20), rx)
                .await
                .is_err()
        );
        calls.cancel("tmc_uart_read_response", Some(1)).await;
        assert_eq!(calls.len().await, 0);
    }

    #[tokio::test]
    async fn test_a_plain_call_takes_the_next_response_whatever_its_oid() {
        // The old path: a registration without `oid` claims the next message of
        // its name, tag parameter or not.
        let calls = PendingCalls::new();
        let (tx, rx) = oneshot::channel();

        calls
            .register("tmc_uart_read_response".to_string(), None, tx)
            .await;
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(7))
                .await
        );

        assert_eq!(rx.await.unwrap(), tmc_response(7));
    }

    #[tokio::test]
    async fn test_cancel_keeps_the_other_oid_registration() {
        let calls = PendingCalls::new();
        let (tx1, _rx1) = oneshot::channel();
        let (tx2, _rx2) = oneshot::channel();

        calls
            .register("tmc_uart_read_response".to_string(), Some(1), tx1)
            .await;
        calls
            .register("tmc_uart_read_response".to_string(), Some(2), tx2)
            .await;

        calls.cancel("tmc_uart_read_response", Some(1)).await;

        assert_eq!(calls.len().await, 1);
        // oid 1's answer now has no claimant: it is dropped, and oid 2's call is
        // left untouched by both the cancel and the drop.
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(1))
                .await
        );
        assert_eq!(calls.len().await, 1);
        assert!(
            calls
                .resolve("tmc_uart_read_response", &tmc_response(2))
                .await
        );
    }
}
