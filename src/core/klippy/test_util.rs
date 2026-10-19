// Test-only waiting helpers shared by the klippy tests.
//
// Tests that wait for a background task (send task, receive task, transport
// shutdown) used to sleep a fixed window and then assert positively. A window
// the scheduler misses under the full gate's load fails a test that should
// pass. Polling to a deadline keeps the assertion as strong (the state still
// has to appear) and spends the window only when the state is actually slow.

use std::time::{Duration, Instant};

/// Wait until `pred` holds, re-checking every `interval` for at most `deadline`.
///
/// Panics with `what` when the budget runs out, so a genuine regression fails
/// the test with the same message the fixed sleep produced.
pub(crate) async fn wait_until<F: FnMut() -> bool>(
    deadline: Duration,
    interval: Duration,
    mut pred: F,
    what: &str,
) {
    let deadline = Instant::now() + deadline;
    while !pred() {
        assert!(Instant::now() < deadline, "{}", what);
        tokio::time::sleep(interval).await;
    }
}
