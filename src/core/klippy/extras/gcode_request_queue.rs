//! `GCodeRequestQueue` — the print-time request queue behind a pin-like output.
//!
//! Upstream is the helper class at the top of `klippy/extras/output_pin.py`
//! (lines 15-90): g-code handlers queue `(print_time, value)` requests, and a
//! flush callback drains them towards the MCU one request at a time, skipping
//! requests a later one has overridden. This port is **pure logic** — it owns
//! only the queue and its schedule floor, nothing that registers callbacks.
//!
//! | upstream | here |
//! |---|---|
//! | `_queue_request(print_time, value)` (`output_pin.py:61-64`) | [`GCodeRequestQueue::push`] |
//! | `_flush_notification(must_flush_time, …)` (`output_pin.py:28-60`) | [`GCodeRequestQueue::flush`] |
//! | `send_async_request(value, print_time=None)` (`output_pin.py:68-90`) | [`GCodeRequestQueue::send_async_request`] |
//! | `self.callback(next_time, req_val)` returning `(action, next_min_time)` | [`RequestSink::set_at`] returning [`FlushAction`] + floor |
//!
//! # What is not here (the caller's job)
//!
//! * **Registration and MCU knowledge.** Upstream's constructor loads
//!   `motion_queuing`, registers `_flush_notification` as the flush callback
//!   and a `klippy:connect` handler, and asks `mcu.min_schedule_time()` on
//!   every flush (`output_pin.py:16-29, 69`). Here `min_schedule_time` is
//!   fixed once in [`GCodeRequestQueue::new`], and the caller registers
//!   [`GCodeRequestQueue::flush`] with the motion layer and supplies the
//!   estimated print time for [`GCodeRequestQueue::send_async_request`]
//!   (upstream computes it from the reactor and MCU at `output_pin.py:70-72`).
//! * **`note_mcu_movequeue_activity`** (`output_pin.py:59-60, 63-64`): this
//!   repo's flush is driven by the 10 ms tick, so there is no "wake the flush
//!   loop" call to make. `_flush_notification`'s unused `max_step_gen_time`
//!   argument is dropped too.
//!
//! # Concurrency
//!
//! Upstream runs single-threaded in the reactor: `rqueue` and
//! `next_min_flush_time` are plain fields and the callback runs unlocked,
//! re-entering the queue freely. Here the g-code thread (`push`,
//! `send_async_request`) and the flush thread (`flush`) touch the queue at the
//! same time, so the queue and the floor live behind one [`Mutex`]. The sink
//! is therefore called **outside** the lock: each call first picks the request
//! to send while holding the lock, drops the guard, invokes
//! [`RequestSink::set_at`], then re-locks to commit the queue and floor
//! updates. A slow or re-entering sink can neither deadlock on the queue nor
//! hold it while the g-code thread pushes.

use std::sync::{Mutex, MutexGuard};

/// What the sink asks the queue to do with the request it was just handed
/// (upstream's `action` strings at `output_pin.py:47-54, 83-89`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlushAction {
    /// `"normal"` — or a [`RequestSink::set_at`] `None` return: the request
    /// stands; pop it and raise the floor by `next_time + min_schedule_time`
    /// (`output_pin.py:55-57`).
    Normal,
    /// `"discard"`: drop the covered prefix *and* the sent request, move on
    /// to the next one (`output_pin.py:47-49`); no `min_schedule_time` floor.
    Discard,
    /// `"reschedule"`: drop only the requests the sent one covered and retry
    /// `rqueue[pos]` itself, with no forced floor advance — the floor moves
    /// only by the returned `next_min_time` (`output_pin.py:50-52`).
    Reschedule,
    /// `"repeat"`: same queue edit as [`FlushAction::Reschedule`] (upstream
    /// decrements `pos` before the pop, `output_pin.py:53-55`) but falls
    /// through to the `next_time + min_schedule_time` floor advance, so the
    /// retry happens at a later aligned time (`output_pin.py:56-57`).
    Repeat,
}

/// The downstream end of the queue: where a request lands at its print time.
pub trait RequestSink: Send + Sync + 'static {
    /// Deliver `value` at `print_time`. `None` means "normal": the request
    /// stands. A returned [`FlushAction`] plus `next_min_time` — a floor the
    /// queue merges into `next_min_flush_time` before acting on the action
    /// (`output_pin.py:44-46`) — steers what happens next.
    fn set_at(&self, print_time: f64, value: f64) -> Option<(FlushAction, f64)>;
}

/// The queue (`rqueue`) and its schedule floor (`next_min_flush_time`,
/// `output_pin.py:20-21`) behind the one lock.
struct QueueState {
    /// `(print_time, value)` in arrival order; never sorted (upstream
    /// doesn't sort either — `output_pin.py:62`).
    rqueue: Vec<(f64, f64)>,
    /// No request is sent before this time (`output_pin.py:21`).
    next_min_flush_time: f64,
}

/// Port of upstream's `GCodeRequestQueue` (`output_pin.py:15-90`): a queue of
/// `(print_time, value)` requests drained one at a time by [`flush`](Self::flush),
/// with covered requests collapsed into the one that overrides them. See the
/// module docs for the upstream mapping, what the caller must wire, and how
/// locking replaces upstream's single-threaded reactor.
pub struct GCodeRequestQueue<S: RequestSink> {
    /// Where a picked request is delivered (upstream's `self.callback`).
    sink: S,
    /// Upstream reads `mcu.min_schedule_time()` per call
    /// (`output_pin.py:29, 69`); here it is fixed at construction.
    min_schedule_time: f64,
    /// `rqueue` + `next_min_flush_time`; see the module docs on locking.
    state: Mutex<QueueState>,
}

impl<S: RequestSink> GCodeRequestQueue<S> {
    /// A queue delivering to `sink`, spacing sends by `min_schedule_time`
    /// seconds (upstream's `mcu.min_schedule_time()`).
    pub fn new(sink: S, min_schedule_time: f64) -> Self {
        Self {
            sink,
            min_schedule_time,
            state: Mutex::new(QueueState {
                rqueue: Vec::new(),
                next_min_flush_time: 0.,
            }),
        }
    }

    /// Queue one request for `print_time` (upstream `_queue_request`,
    /// `output_pin.py:61-62`, minus the `note_mcu_movequeue_activity` call the
    /// caller's flush tick covers). May run on the g-code thread while
    /// [`flush`](Self::flush) runs on the flush thread.
    pub fn push(&self, print_time: f64, value: f64) {
        self.lock().rqueue.push((print_time, value));
    }

    /// Drain the queue up to `must_flush_time` (upstream `_flush_notification`,
    /// `output_pin.py:28-60`). Each turn:
    ///
    /// 1. `next_time = max(rqueue[0].time, next_min_flush_time)`; if it is
    ///    past `must_flush_time`, return with the queue intact
    ///    (`output_pin.py:32-34`);
    /// 2. skip requests a following one overrides (`output_pin.py:35-38`) and
    ///    hand `rqueue[pos]`'s value to the sink **at** `next_time`
    ///    (`output_pin.py:39-41`) — outside the lock, see module docs;
    /// 3. apply the sink's [`FlushAction`] (`output_pin.py:42-57`).
    ///
    /// Stops when the queue runs dry. Like upstream, a sink that always
    /// answers [`FlushAction::Reschedule`] without raising the floor keeps a
    /// single call spinning — the sink is expected to make progress.
    pub fn flush(&self, must_flush_time: f64) {
        loop {
            // Pick what to send while holding the lock (`output_pin.py:31-39`).
            let (next_time, value, pos) = {
                let state = self.lock();
                let Some(&(first_time, _)) = state.rqueue.first() else {
                    return;
                };
                let next_time = first_time.max(state.next_min_flush_time);
                if next_time > must_flush_time {
                    // Not yet due: the whole queue stays (`output_pin.py:33-34`).
                    return;
                }
                // Skip requests overridden with a following request
                // (`output_pin.py:35-38`).
                let mut pos = 0;
                while pos + 1 < state.rqueue.len() && state.rqueue[pos + 1].0 <= next_time {
                    pos += 1;
                }
                (next_time, state.rqueue[pos].1, pos)
            };

            // Sink runs outside the lock: we only bring the picked entry out.
            let ret = self.sink.set_at(next_time, value);

            let mut state = self.lock();
            let action = match ret {
                Some((action, next_min_time)) => {
                    // Every returned action raises the floor first
                    // (`output_pin.py:44-46`).
                    state.next_min_flush_time = state.next_min_flush_time.max(next_min_time);
                    action
                }
                None => FlushAction::Normal,
            };
            match action {
                // Discard the covered prefix *and* the sent request, move on
                // (`output_pin.py:47-49`); no schedule-floor advance.
                FlushAction::Discard => {
                    state.rqueue.drain(..=pos);
                    continue;
                }
                // Drop only the covered prefix and retry `rqueue[pos]`
                // (`output_pin.py:50-52`); the floor moved only by the
                // returned `next_min_time` above.
                FlushAction::Reschedule => {
                    state.rqueue.drain(..pos);
                    continue;
                }
                // Upstream's `pos -= 1` before `del rqueue[:pos+1]`
                // (`output_pin.py:53-55`) is the same slice as reschedule's
                // `del rqueue[:pos]`: `rqueue[pos]` stays queued. Unlike
                // reschedule it falls through to the floor advance below.
                FlushAction::Repeat => {
                    state.rqueue.drain(..pos);
                }
                // Normal (or an action upstream does not special-case): pop
                // the covered prefix plus the sent request (`output_pin.py:55`).
                FlushAction::Normal => {
                    state.rqueue.drain(..=pos);
                }
            }
            // Space the next send behind this one (`output_pin.py:56-57`).
            state.next_min_flush_time = state
                .next_min_flush_time
                .max(next_time + self.min_schedule_time);
        }
    }

    /// Send `value` at `print_time` without touching the queue (upstream
    /// `send_async_request`, `output_pin.py:68-90`; the caller supplies the
    /// estimated print time upstream derives at `output_pin.py:70-72`).
    ///
    /// Each turn sends at `max(print_time, next_min_flush_time)` and then:
    /// [`FlushAction::Discard`] breaks, [`FlushAction::Reschedule`] retries
    /// against the raised floor, [`FlushAction::Repeat`] retries after the
    /// `next_time + min_schedule_time` advance, and normal breaks
    /// (`output_pin.py:73-90`).
    pub fn send_async_request(&self, value: f64, print_time: f64) {
        loop {
            // Floor re-read every turn: reschedule raised it (`output_pin.py:74`).
            let next_time = {
                let state = self.lock();
                print_time.max(state.next_min_flush_time)
            };

            // Sink runs outside the lock, as in `flush`.
            let ret = self.sink.set_at(next_time, value);

            let mut state = self.lock();
            let mut action = FlushAction::Normal;
            if let Some((returned, next_min_time)) = ret {
                action = returned;
                // Every returned action raises the floor first
                // (`output_pin.py:80-82`).
                state.next_min_flush_time = state.next_min_flush_time.max(next_min_time);
                if action == FlushAction::Discard {
                    // Breaks before the schedule-floor advance (`output_pin.py:83-84`).
                    break;
                }
                if action == FlushAction::Reschedule {
                    // Retries against the raised floor (`output_pin.py:85-86`).
                    continue;
                }
            }
            state.next_min_flush_time = state
                .next_min_flush_time
                .max(next_time + self.min_schedule_time);
            if action != FlushAction::Repeat {
                break;
            }
        }
    }

    /// The queue state; a poisoned lock is recovered from so one panicking
    /// caller cannot wedge the queue permanently (repo pattern,
    /// `adc_scaled.rs`).
    fn lock(&self) -> MutexGuard<'_, QueueState> {
        self.state.lock().unwrap_or_else(|p| p.into_inner())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::thread;

    /// A sink that records every `(print_time, value)` it is handed and
    /// answers from a script (one entry per call; once the script runs out it
    /// answers `None`, i.e. normal). `Clone` shares the recordings.
    #[derive(Clone, Default)]
    struct MockSink {
        calls: Arc<Mutex<Vec<(f64, f64)>>>,
        script: Arc<Mutex<VecDeque<Option<(FlushAction, f64)>>>>,
    }

    impl MockSink {
        /// A sink whose first calls answer `replies` in order, then `None`.
        fn scripted(replies: &[Option<(FlushAction, f64)>]) -> Self {
            let sink = Self::default();
            *sink.script.lock().unwrap_or_else(|p| p.into_inner()) =
                replies.iter().copied().collect();
            sink
        }

        /// Everything `set_at` received, in order.
        fn calls(&self) -> Vec<(f64, f64)> {
            self.calls.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    impl RequestSink for MockSink {
        fn set_at(&self, print_time: f64, value: f64) -> Option<(FlushAction, f64)> {
            self.calls
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push((print_time, value));
            self.script
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .pop_front()
                .flatten()
        }
    }

    /// Compare recorded calls pairwise; times are float sums, so compare with
    /// a tolerance (`assert_close` below for single figures).
    fn assert_calls(got: &[(f64, f64)], want: &[(f64, f64)]) {
        assert_eq!(got.len(), want.len(), "calls: got {got:?}, want {want:?}");
        for (g, w) in got.iter().zip(want) {
            assert!(
                (g.0 - w.0).abs() < 1e-9,
                "print_time: got {g:?}, want {w:?}"
            );
            assert!((g.1 - w.1).abs() < 1e-9, "value: got {g:?}, want {w:?}");
        }
    }

    fn assert_close(got: f64, want: f64) {
        assert!((got - want).abs() < 1e-9, "got {got}, want {want}");
    }

    /// Override compression (`output_pin.py:35-38`): three requests at the
    /// same time collapse into the last one — only the value that covers the
    /// others is handed to the sink, at the aligned time.
    #[test]
    fn override_compression_sends_only_the_covering_request() {
        let sink = MockSink::default();
        let q = GCodeRequestQueue::new(sink.clone(), 0.1);
        q.push(1.0, 0.1);
        q.push(1.0, 0.2);
        q.push(1.0, 0.3);

        q.flush(f64::MAX);

        assert_calls(&sink.calls(), &[(1.0, 0.3)]);
        assert!(q.lock().rqueue.is_empty());
    }

    /// The `min_schedule_time` floor (`output_pin.py:56-57`) both spaces the
    /// sends and re-times the next request: the entry pushed at 1.05 goes out
    /// at 1.0 + 0.2, not at its own time.
    #[test]
    fn min_schedule_time_spaces_and_aligns_consecutive_sends() {
        let sink = MockSink::default();
        let q = GCodeRequestQueue::new(sink.clone(), 0.2);
        q.push(1.0, 0.1);
        q.push(1.05, 0.2);

        q.flush(f64::MAX);

        assert_calls(&sink.calls(), &[(1.0, 0.1), (1.2, 0.2)]);
        assert_close(q.lock().next_min_flush_time, 1.4);
    }

    /// A request whose aligned time passes `must_flush_time` is not sent and
    /// nothing behind it is dequeued (`output_pin.py:32-34`) — neither before
    /// the first send nor mid-loop once the floor pushed the aligned time past
    /// the must-flush point.
    #[test]
    fn must_flush_time_before_the_aligned_time_stops_the_queue() {
        let sink = MockSink::default();
        let q = GCodeRequestQueue::new(sink.clone(), 0.2);
        q.push(1.0, 0.1);
        q.push(1.05, 0.2);

        // Nothing is due yet.
        q.flush(0.5);
        assert!(sink.calls().is_empty());
        assert_eq!(q.lock().rqueue.len(), 2);

        // The first goes out; the second aligns to 1.2 > 1.0, so the loop
        // returns with it still queued.
        q.flush(1.0);
        assert_calls(&sink.calls(), &[(1.0, 0.1)]);
        assert_eq!(q.lock().rqueue.len(), 1);

        // Still due only at the aligned time.
        q.flush(1.19);
        assert_calls(&sink.calls(), &[(1.0, 0.1)]);
        q.flush(1.2);
        assert_calls(&sink.calls(), &[(1.0, 0.1), (1.2, 0.2)]);
        assert!(q.lock().rqueue.is_empty());
    }

    /// discard (`output_pin.py:47-49`): the compressed prefix and the sent
    /// request are dropped together, and the floor the sink returned (2.5)
    /// re-times the survivor — `max(2.0, 2.5)`, not 2.0.
    #[test]
    fn discard_drops_the_covered_prefix_and_takes_the_returned_floor() {
        let sink = MockSink::scripted(&[Some((FlushAction::Discard, 2.5))]);
        let q = GCodeRequestQueue::new(sink.clone(), 0.1);
        q.push(1.0, 0.1);
        q.push(1.0, 0.2);
        q.push(2.0, 0.3);

        q.flush(f64::MAX);

        assert_calls(&sink.calls(), &[(1.0, 0.2), (2.5, 0.3)]);
        assert!(q.lock().rqueue.is_empty());
    }

    /// reschedule (`output_pin.py:50-52`): the covered request and its
    /// override both collapse — only `rqueue[pos]` stays and is retried — and
    /// there is *no* `next_time + min_schedule_time` floor: with a 0.5 s
    /// schedule time and a sink floor of 0., the retry is at 1.0 again, not at
    /// 1.5.
    #[test]
    fn reschedule_retries_the_covered_request_without_a_forced_floor() {
        let sink = MockSink::scripted(&[Some((FlushAction::Reschedule, 0.0))]);
        let q = GCodeRequestQueue::new(sink.clone(), 0.5);
        q.push(1.0, 0.1);
        q.push(1.0, 0.2);

        q.flush(f64::MAX);

        assert_calls(&sink.calls(), &[(1.0, 0.2), (1.0, 0.2)]);
        assert!(q.lock().rqueue.is_empty());
    }

    /// repeat (`output_pin.py:53-57`): same queue edit as reschedule (the
    /// covered request is dropped, `rqueue[pos]` retried) but it falls through
    /// to the floor advance, so with a 0.5 s schedule time the retry lands at
    /// 1.0 + 0.5 = 1.5 — the contrast with the reschedule test above.
    #[test]
    fn repeat_retries_at_the_advanced_floor_and_drops_the_covered_prefix() {
        let sink = MockSink::scripted(&[Some((FlushAction::Repeat, 0.0))]);
        let q = GCodeRequestQueue::new(sink.clone(), 0.5);
        q.push(1.0, 0.1);
        q.push(1.0, 0.2);

        q.flush(f64::MAX);

        assert_calls(&sink.calls(), &[(1.0, 0.2), (1.5, 0.2)]);
        assert!(q.lock().rqueue.is_empty());
    }

    /// Out-of-order pushes are not sorted: the queue is arrival-ordered and
    /// the override rule (`output_pin.py:35-38`) only compares times against
    /// `next_time`. The (2.0, 0.2) pushed *after* (3.0, 0.3) therefore covers
    /// it, and its value goes out at the aligned 3.0.
    #[test]
    fn out_of_order_pushes_override_in_arrival_order_at_the_aligned_time() {
        let sink = MockSink::default();
        let q = GCodeRequestQueue::new(sink.clone(), 0.1);
        q.push(1.0, 0.1);
        q.push(3.0, 0.3);
        q.push(2.0, 0.2);

        q.flush(f64::MAX);

        assert_calls(&sink.calls(), &[(1.0, 0.1), (3.0, 0.2)]);
        assert!(q.lock().rqueue.is_empty());
    }

    /// send_async_request goes straight to the sink and never touches the
    /// queue (`output_pin.py:73-90`): the pushed request stays put, and the
    /// second async call at the same print time aligns to the floor the first
    /// one raised (5.0 + 0.1).
    #[test]
    fn send_async_request_goes_straight_to_the_sink() {
        let sink = MockSink::default();
        let q = GCodeRequestQueue::new(sink.clone(), 0.1);
        q.push(9.0, 0.9);

        q.send_async_request(0.7, 5.0);
        q.send_async_request(0.8, 5.0);

        assert_calls(&sink.calls(), &[(5.0, 0.7), (5.1, 0.8)]);
        assert_eq!(q.lock().rqueue, vec![(9.0, 0.9)]);

        // The queued request flushes afterwards, untouched by the async sends.
        q.flush(f64::MAX);
        assert_calls(&sink.calls(), &[(5.0, 0.7), (5.1, 0.8), (9.0, 0.9)]);
    }

    /// Async reschedule (`output_pin.py:85-86`): retries against the sink's
    /// raised floor, and only the accepted send applies
    /// `next_time + min_schedule_time` (6.0 + 0.5).
    #[test]
    fn send_async_request_reschedules_until_accepted() {
        let sink = MockSink::scripted(&[Some((FlushAction::Reschedule, 6.0)), None]);
        let q = GCodeRequestQueue::new(sink.clone(), 0.5);

        q.send_async_request(1.0, 5.0);

        assert_calls(&sink.calls(), &[(5.0, 1.0), (6.0, 1.0)]);
        assert_close(q.lock().next_min_flush_time, 6.5);
    }

    /// Async repeat (`output_pin.py:87-90`): falls through to the floor
    /// advance and does not break, so the retry lands at 5.0 + 0.5 = 5.5.
    #[test]
    fn send_async_request_repeats_after_the_schedule_floor() {
        let sink = MockSink::scripted(&[Some((FlushAction::Repeat, 0.0)), None]);
        let q = GCodeRequestQueue::new(sink.clone(), 0.5);

        q.send_async_request(1.0, 5.0);

        assert_calls(&sink.calls(), &[(5.0, 1.0), (5.5, 1.0)]);
        assert_close(q.lock().next_min_flush_time, 6.0);
    }

    /// Async discard (`output_pin.py:83-84`) breaks *before* the schedule
    /// floor: the floor is only the returned 4.0, never 5.0 + 0.5.
    #[test]
    fn send_async_request_discard_breaks_without_the_schedule_floor() {
        let sink = MockSink::scripted(&[Some((FlushAction::Discard, 4.0))]);
        let q = GCodeRequestQueue::new(sink.clone(), 0.5);

        q.send_async_request(1.0, 5.0);

        assert_calls(&sink.calls(), &[(5.0, 1.0)]);
        assert_close(q.lock().next_min_flush_time, 4.0);
    }

    /// Concurrency smoke: two g-code threads pushing interleaved (mutually
    /// out-of-order) times while a flush thread drains concurrently must
    /// finish without deadlock or panic, empty the queue, and only ever send a
    /// pushed value at an aligned time at or after its own.
    #[test]
    fn two_push_threads_and_a_flush_thread_drain_the_queue() {
        const PER_THREAD: usize = 2_000;
        const PUSHERS: usize = 2;
        let sink = MockSink::default();
        let q = Arc::new(GCodeRequestQueue::new(sink.clone(), 0.0));
        let joined = Arc::new(AtomicUsize::new(0));

        let pushers: Vec<_> = [0.0f64, 0.0005]
            .into_iter()
            .map(|offset| {
                let q = Arc::clone(&q);
                let joined = Arc::clone(&joined);
                thread::spawn(move || {
                    for i in 0..PER_THREAD {
                        let t = offset + i as f64 * 1e-3;
                        q.push(t, t);
                    }
                    joined.fetch_add(1, Ordering::SeqCst);
                })
            })
            .collect();

        let flusher_queue = Arc::clone(&q);
        let flusher_joined = Arc::clone(&joined);
        let flusher = thread::spawn(move || {
            while flusher_joined.load(Ordering::SeqCst) < PUSHERS
                || !flusher_queue.lock().rqueue.is_empty()
            {
                flusher_queue.flush(f64::MAX);
                thread::yield_now();
            }
        });

        for pusher in pushers {
            pusher.join().expect("a pusher thread panics");
        }
        flusher.join().expect("the flusher thread panics");

        let calls = sink.calls();
        assert!(!calls.is_empty());
        assert!(q.lock().rqueue.is_empty());
        // Each call carries a pushed value at its aligned time (>= the entry's
        // own time, since next_time = max(entry, floor) and compression only
        // skips entries at or before it).
        for &(print_time, value) in &calls {
            assert!(
                value <= print_time,
                "value {value} sent at earlier {print_time}"
            );
        }
        // Compression can collapse entries but nothing is sent twice.
        assert!(calls.len() <= PUSHERS * PER_THREAD);
    }
}
