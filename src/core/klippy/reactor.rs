//! The reactor: the machine's clock, and the timers it wakes on.
//!
//! A printer is not a runtime. Upstream gives every printer a reactor
//! (`Printer.get_reactor()`, 119 call sites) and everything asynchronous goes
//! through it: a module schedules a callback for a later moment, or parks the
//! current greenlet until something completes. That reactor *is* upstream's
//! event loop — a `select()`/`poll()` around a timer list, with greenlets
//! papering over the waiting so the code reads synchronously
//! (`klippy/reactor.py:111` `monotonic`, `:145` `register_timer`, `:187`
//! `register_callback`, `:227` `pause`).
//!
//! Here the waiting half is `async`/`await` and belongs to the executor, so
//! this module keeps only the half that does not: **a monotonic clock and a
//! timer list**. A module that wants to wait writes `future.await`; a module
//! that wants to be *called back* later calls [`Reactor::register_timer`]. The
//! two big pieces upstream needs for its greenlet scheduling — `pause` and
//! `completion`/`wait` — have no counterpart here on purpose: they are what
//! `async` already means.
//!
//! # Why a trait
//!
//! So the machine does not have to own a runtime. [`Printer`] holds an
//! `Arc<dyn Reactor>` and never names tokio; whoever builds the printer decides
//! what drives time. The host passes a [`TokioReactor`] over the runtime it
//! already runs; tests pass a [`ManualReactor`] and step the clock by hand,
//! which is deterministic and needs no runtime at all.
//!
//! # The timer contract
//!
//! [`Reactor::register_timer`] follows upstream's shape: the callback receives
//! the event time and returns the *next* wake time, or `None` to stop. A
//! callback that returns `Some(now)` therefore runs again immediately — that is
//! upstream's `NOW` — and one that returns a later time stays registered until
//! it says `None`. Cancelling is [`Reactor::unregister_timer`].
//!
//! # What a callback may do
//!
//! A callback runs on the reactor's **one dispatcher thread**, one callback at
//! a time, so it may touch printer state without coordinating with other
//! callbacks. That serialization is the whole point — and it comes with a
//! matching duty: a callback must **not block and must not do heavy work**,
//! because everything else waits behind it. Upstream enforces the same rule with
//! `assert_no_pause` in its shutdown / ready callbacks (`klippy/reactor.py:265`);
//! here there is no `pause` to forbid, but "no waiting, no long work" still
//! holds — there is nowhere to `await` from a [`TimerCallback`], and a callback
//! that sleeps, locks across unrelated work, or runs for long delays every
//! other timer and the rest of the machine behind it. Waiting belongs in
//! `async` code that `.await`s; a callback is for work that is already ready to
//! do.
//!
//! # Latency
//!
//! Because the dispatcher runs callbacks one at a time, a slow one is directly
//! visible: it makes every later timer late. [`Reactor::set_latency_notifier`]
//! is how that is seen — a callback can be told whenever a dispatch round takes
//! longer than a threshold, along with the names and durations of the callbacks
//! in it ([`LatencyReport`]). It is upstream's `set_latency_notifier`
//! (`klippy/reactor.py:316`), which `extras/garbage_collection.py` uses to log a
//! `Reactor busy for …` warning; we have no garbage collector, so here it is
//! purely a diagnostic. The default does nothing; [`TokioReactor`] implements it.
//!
//! [`Printer`]: crate::core::klippy::printer::Printer

use std::cmp::Ordering as CmpOrdering;
use std::collections::BinaryHeap;
use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// A timer callback: given the event time, return the next wake time.
///
/// `None` stops the timer, which is how a one-shot ends and how a periodic one
/// is retired. The signature is upstream's
/// `double callback(double eventtime)` returning `NEVER`
/// (`klippy/reactor.py:166`), with `Option` in place of the sentinel.
///
/// It runs on the reactor's single dispatcher thread, one callback at a time,
/// and must not block or do heavy work — see the module docs.
pub type TimerCallback = Box<dyn FnMut(f64) -> Option<f64> + Send>;

/// A one-shot callback handed to [`Reactor::call_later`].
pub type OneShot = Box<dyn FnOnce(f64) + Send>;

/// One callback's part of a [`LatencyReport`].
#[derive(Debug, Clone)]
pub struct CallbackRun {
    /// The name the callback was registered with.
    pub name: &'static str,
    /// How long the callback ran, in seconds.
    pub duration: f64,
    /// How late it started: its event time minus the wake time it was due at.
    /// Non-zero means the dispatcher was already busy when the timer came due.
    pub lateness: f64,
}

/// What the reactor was doing when it ran late.
///
/// Reported by [`Reactor::set_latency_notifier`] after a dispatch round that
/// took longer than the threshold: the round's callbacks and how long the round
/// lasted. This is a diagnostic — the machine is soft real-time, and the point
/// is to find the callback that made the reactor late, not to act on every
/// warning.
#[derive(Debug, Clone)]
pub struct LatencyReport {
    /// The event time the report is dated with.
    pub eventtime: f64,
    /// How long the round was busy, in seconds — measured from the earliest
    /// wake time in it, so a late wake counts the same as a slow callback.
    pub busy: f64,
    /// The callbacks that ran in the round, in order.
    pub callbacks: Vec<CallbackRun>,
}

/// Called when a dispatch round runs past the latency threshold.
///
/// Shared (`Arc`) so the dispatcher can clone it out and call it without
/// holding the lock that holds it.
pub type LatencyCallback = Arc<dyn Fn(LatencyReport) + Send + Sync>;

/// The machine's clock and its timers.
///
/// Implemented by the host over its runtime ([`TokioReactor`]) and by tests
/// with a clock they control ([`ManualReactor`]). The machine holds it as
/// `Arc<dyn Reactor>`, so the two are interchangeable.
pub trait Reactor: Send + Sync {
    /// Monotonic seconds, near zero when the reactor was built.
    ///
    /// This is the clock every `get_status(eventtime)` is dated with and the
    /// one [`Printer::eventtime`](crate::core::klippy::printer::Printer::eventtime)
    /// reports. It never goes backwards and is unaffected by wall-clock
    /// changes. Upstream reads the reactor's `monotonic` here too
    /// (`klippy/reactor.py:111`); the only difference is the origin — ours is
    /// reactor construction rather than boot, which is what a client wants when
    /// it tells one report from the next.
    fn monotonic(&self) -> f64;

    /// Call `callback` no earlier than `waketime`, naming it for latency
    /// reports.
    ///
    /// The name is what [`Reactor::set_latency_notifier`] shows. Upstream reads
    /// it off the callback (`get_function_owner`,
    /// `klippy/extras/garbage_collection.py:13`); a Rust closure has no name, so
    /// a caller gives one.
    ///
    /// The callback may be called again whenever it asks to be, by returning
    /// the next wake time; returning `None` retires it. The returned handle is
    /// how it is cancelled ([`Reactor::unregister_timer`]).
    ///
    /// A `waketime` already in the past runs as soon as the implementer gets to
    /// it, which is upstream's `NOW` (`0.`).
    ///
    /// The callback runs on the reactor's dispatcher and must not block or do
    /// heavy work (see the module docs); it must not wait for anything, because
    /// every other timer waits behind it.
    fn register_timer_named(
        &self,
        name: &'static str,
        callback: TimerCallback,
        waketime: f64,
    ) -> TimerHandle;

    /// Call `callback` no earlier than `waketime`.
    ///
    /// The unnamed form of [`Reactor::register_timer_named`]; a latency report
    /// shows it as `"<timer>"`. Prefer the named form where the report should
    /// be able to say which timer was slow.
    fn register_timer(&self, callback: TimerCallback, waketime: f64) -> TimerHandle {
        self.register_timer_named("<timer>", callback, waketime)
    }

    /// Ask to be told when a dispatch round runs longer than `latency` seconds.
    ///
    /// Upstream's `set_latency_notifier` (`klippy/reactor.py:316`), used there
    /// by `extras/garbage_collection.py` to log a `Reactor busy for …` warning.
    /// We have no garbage collector, so this is a diagnostic: the default does
    /// nothing, and only [`TokioReactor`] implements it. The callback runs on
    /// the dispatcher, right after the slow round — it must not block either.
    fn set_latency_notifier(&self, _latency: f64, _callback: LatencyCallback) {}

    /// Cancel a timer.
    ///
    /// Idempotent, and safe after the timer has already retired itself. The
    /// default hands the request to the handle, which every implementation
    /// builds; an implementation that needs to do more (wake a sleeping task,
    /// say) does it there.
    fn unregister_timer(&self, handle: TimerHandle) {
        handle.cancel();
    }

    /// Call `callback` once, `delay` seconds from now.
    ///
    /// The one-shot counterpart of [`Reactor::register_timer`], and our stand-in
    /// for upstream's `register_callback` (`klippy/reactor.py:187`). Upstream
    /// returns a completion a greenlet can `wait()` on; here anything that
    /// *waits* is `async`, and this is only for a callback that does not.
    fn call_later(&self, delay: f64, callback: OneShot) -> TimerHandle {
        let mut callback = Some(callback);
        self.register_timer_named(
            "call_later",
            Box::new(move |eventtime| {
                if let Some(callback) = callback.take() {
                    callback(eventtime);
                }
                None
            }),
            self.monotonic() + delay,
        )
    }
}

// ===========================================================================
// TimerHandle
// ===========================================================================

/// A registered timer, for cancelling it.
///
/// Cloning gives another handle to the same timer; the timer ends when the
/// callback returns `None` or when any handle cancels it. There is no reference
/// counting of runs: a clone is not a subscription.
#[derive(Clone)]
pub struct TimerHandle(Arc<dyn TimerCancel>);

impl TimerHandle {
    /// Build a handle from the implementation's cancellation.
    fn new(cancel: impl TimerCancel + 'static) -> Self {
        Self(Arc::new(cancel))
    }

    /// Cancel the timer. Idempotent.
    pub fn cancel(&self) {
        self.0.cancel();
    }
}

impl fmt::Debug for TimerHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("TimerHandle").finish_non_exhaustive()
    }
}

/// What a [`TimerHandle`] does when cancelled.
///
/// Implemented for any `Fn()`, because that is all most handles are: a flag to
/// set, a message to send.
trait TimerCancel: Send + Sync {
    fn cancel(&self);
}

impl<F: Fn() + Send + Sync + 'static> TimerCancel for F {
    fn cancel(&self) {
        self()
    }
}

// ===========================================================================
// TokioReactor
// ===========================================================================

/// The reactor the host runs on: one dispatcher task over a min-heap of timers.
///
/// Built over the runtime the host already has, so it creates nothing of its
/// own — no thread, no event loop. `monotonic` is the runtime's clock, which
/// means a paused test clock (`#[tokio::test(start_paused = true)]`) moves it
/// too.
///
/// Timers are **not** one task each. They all live in one min-heap behind the
/// single dispatcher task spawned here, which sleeps until the earliest wake
/// time, pops every timer that is due, and runs the callbacks **one at a
/// time**, in wake-time order — upstream `_check_timers`' contract
/// (`klippy/reactor.py:157-172`). Two timer callbacks can therefore never run
/// at once, which is what lets one of them touch printer state without
/// coordinating with the other.
///
/// Registering or cancelling a timer wakes the dispatcher so it can shorten or
/// drop its sleep; a cancelled timer is skipped when it comes due. Dropping a
/// handle never cancels.
pub struct TokioReactor {
    /// What `monotonic` counts from.
    origin: tokio::time::Instant,
    /// The timers, shared with the one dispatcher task that runs them.
    dispatcher: Arc<Dispatcher>,
}

impl TokioReactor {
    /// Build a reactor over `handle`.
    ///
    /// The handle may be any runtime handle — the future that registered a
    /// timer runs on that runtime, so it must outlive the printer. The caller
    /// is expected to be inside the runtime already (`Handle::current()`), or
    /// to hold a handle from one it built.
    ///
    /// The dispatcher task is spawned here and ends when this reactor is
    /// dropped.
    pub fn new(handle: tokio::runtime::Handle) -> Self {
        let origin = tokio::time::Instant::now();
        let dispatcher = Arc::new(Dispatcher::new());
        handle.spawn(run_dispatcher(Arc::clone(&dispatcher), origin));
        Self { origin, dispatcher }
    }
}

impl Drop for TokioReactor {
    fn drop(&mut self) {
        // Wake the dispatcher so it sees the reactor is gone and its task can
        // end, rather than sleeping for a timer nobody can register again.
        self.dispatcher.closed.store(true, Ordering::Release);
        self.dispatcher.notify.notify_one();
    }
}

impl Reactor for TokioReactor {
    fn monotonic(&self) -> f64 {
        self.origin.elapsed().as_secs_f64()
    }

    fn register_timer_named(
        &self,
        name: &'static str,
        callback: TimerCallback,
        waketime: f64,
    ) -> TimerHandle {
        // Cancellation is a flag the dispatcher checks when the timer comes
        // due, plus a notification to wake it early. A `Notify` rather than a
        // channel, because dropping a handle must not cancel: the dispatcher
        // owns the entries, so a timer stays registered whether or not anyone
        // still holds its handle.
        let cancelled = Arc::new(AtomicBool::new(false));
        self.dispatcher.push(TimerEntry {
            waketime,
            seq: self.dispatcher.next_seq(),
            name,
            callback,
            cancelled: Arc::clone(&cancelled),
        });
        // The new timer may be earlier than whatever the dispatcher is sleeping
        // for, so wake it to re-evaluate. The entry is in the heap before the
        // notification, and a stale wakeup only costs a re-scan.
        self.dispatcher.notify.notify_one();
        let dispatcher = Arc::clone(&self.dispatcher);
        TimerHandle::new(move || {
            cancelled.store(true, Ordering::Release);
            dispatcher.notify.notify_one();
        })
    }

    fn set_latency_notifier(&self, latency: f64, callback: LatencyCallback) {
        *self
            .dispatcher
            .latency
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) =
            Some(LatencyNotifier { latency, callback });
    }
}

/// The timers, shared between the reactor and its one dispatcher task.
struct Dispatcher {
    /// A min-heap: the earliest wake time is always on top.
    timers: Mutex<BinaryHeap<TimerEntry>>,
    /// Wakes the dispatcher when a timer is added or cancelled, or when the
    /// reactor is dropped.
    notify: tokio::sync::Notify,
    /// Tie-breaker so timers due at the same moment run in registration order,
    /// like upstream's list walk and `ManualReactor`'s stable sort.
    seq: AtomicU64,
    /// Set by `TokioReactor::drop`, so the task ends with the reactor.
    closed: AtomicBool,
    /// The latency notifier, once [`Reactor::set_latency_notifier`] was called.
    latency: Mutex<Option<LatencyNotifier>>,
}

/// Told when a dispatch round ran past its threshold.
struct LatencyNotifier {
    latency: f64,
    callback: LatencyCallback,
}

impl Dispatcher {
    fn new() -> Self {
        Self {
            timers: Mutex::new(BinaryHeap::new()),
            notify: tokio::sync::Notify::new(),
            seq: AtomicU64::new(0),
            closed: AtomicBool::new(false),
            latency: Mutex::new(None),
        }
    }

    fn next_seq(&self) -> u64 {
        self.seq.fetch_add(1, Ordering::Relaxed)
    }

    fn push(&self, entry: TimerEntry) {
        self.timers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push(entry);
    }

    /// Call the latency notifier if a round ran past its threshold.
    ///
    /// The callback is cloned out of the lock first, so it may itself call
    /// `set_latency_notifier` (or do anything else) without deadlocking.
    fn report_if_late(&self, busy: f64, eventtime: f64, callbacks: Vec<CallbackRun>) {
        let callback = {
            let guard = self
                .latency
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            match guard.as_ref() {
                Some(notifier) if busy >= notifier.latency => Some(Arc::clone(&notifier.callback)),
                _ => None,
            }
        };
        if let Some(callback) = callback {
            callback(LatencyReport {
                eventtime,
                busy,
                callbacks,
            });
        }
    }
}

/// One registered timer.
struct TimerEntry {
    waketime: f64,
    /// Registration order, used to order timers due at the same moment.
    seq: u64,
    /// Shown in a [`LatencyReport`].
    name: &'static str,
    callback: TimerCallback,
    cancelled: Arc<AtomicBool>,
}

impl PartialEq for TimerEntry {
    fn eq(&self, other: &Self) -> bool {
        self.seq == other.seq && self.waketime.total_cmp(&other.waketime) == CmpOrdering::Equal
    }
}

impl Eq for TimerEntry {}

impl PartialOrd for TimerEntry {
    fn partial_cmp(&self, other: &Self) -> Option<CmpOrdering> {
        Some(self.cmp(other))
    }
}

impl Ord for TimerEntry {
    fn cmp(&self, other: &Self) -> CmpOrdering {
        // `BinaryHeap` is a max-heap, so compare in reverse: the earliest wake
        // time wins, and among equal wake times the lowest `seq` (registered
        // first) wins.
        other
            .waketime
            .total_cmp(&self.waketime)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

/// The longest the dispatcher sleeps in one go.
///
/// A wake time is an `f64` and may be arbitrarily far away — upstream's
/// `NEVER` is `9999999999999999.`. Sleeping that long in one call would
/// overflow a monotonic instant, so the dispatcher sleeps a day at a time and
/// re-checks. Nothing in the host asks for more than a few seconds.
const MAX_TIMER_SLEEP: Duration = Duration::from_secs(24 * 60 * 60);

/// The one task that runs every timer.
///
/// Each pass takes every timer that is due now — earliest first — and runs its
/// callback **before** looking at the next one, so callbacks never overlap.
/// Between passes it sleeps until the earliest remaining timer or a wakeup.
async fn run_dispatcher(dispatcher: Arc<Dispatcher>, origin: tokio::time::Instant) {
    loop {
        if dispatcher.closed.load(Ordering::Acquire) {
            return;
        }

        // Take the due timers out from under the lock: a callback may register
        // or cancel another timer, and it must not do that while we hold the
        // heap. The heap pops the earliest wake time first — and, among equal
        // wake times, the lowest `seq` — so `due` is already in the order the
        // callbacks should run.
        let mut due = Vec::new();
        {
            let mut timers = dispatcher
                .timers
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            let now = origin.elapsed().as_secs_f64();
            loop {
                let is_due = timers
                    .peek()
                    .map(|entry| entry.waketime <= now)
                    .unwrap_or(false);
                if !is_due {
                    break;
                }
                let entry = timers.pop().expect("peeked as due");
                if entry.cancelled.load(Ordering::Acquire) {
                    continue;
                }
                due.push(entry);
            }
        }

        if due.is_empty() {
            // Nothing to run: sleep until the earliest timer, or until a timer
            // is added or cancelled. The notification is armed before the heap
            // is read again, so a registration in between is not lost.
            let notified = dispatcher.notify.notified();
            let sleep = {
                let timers = dispatcher
                    .timers
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner());
                timers.peek().map(|entry| {
                    let remaining = entry.waketime - origin.elapsed().as_secs_f64();
                    Duration::from_secs_f64(remaining.max(0.0)).min(MAX_TIMER_SLEEP)
                })
            };
            match sleep {
                // No timer registered: wait for one, or for the reactor to go.
                None => notified.await,
                Some(sleep) => {
                    tokio::select! {
                        _ = tokio::time::sleep(sleep) => {}
                        _ = notified => {}
                    }
                }
            }
            continue;
        }

        // Run the round, recording each callback for a latency report. The
        // round's busy time is measured from the earliest wake time in it, so a
        // late wake shows up the same way a slow callback does.
        let first_due = due.first().map(|entry| entry.waketime);
        let mut runs = Vec::with_capacity(due.len());
        for mut entry in due {
            if entry.cancelled.load(Ordering::Acquire) {
                continue;
            }
            let name = entry.name;
            // The time the callback is woken for, read per callback so a long
            // one is visible in the next callback's `eventtime`.
            let started = origin.elapsed().as_secs_f64();
            let lateness = started - entry.waketime;
            if let Some(next) = (entry.callback)(started) {
                entry.waketime = next;
                entry.seq = dispatcher.next_seq();
                dispatcher.push(entry);
            }
            runs.push(CallbackRun {
                name,
                duration: origin.elapsed().as_secs_f64() - started,
                lateness,
            });
        }
        if let Some(first_due) = first_due {
            let eventtime = origin.elapsed().as_secs_f64();
            dispatcher.report_if_late(eventtime - first_due, eventtime, runs);
        }
    }
}

// ===========================================================================
// ManualReactor
// ===========================================================================

/// A reactor whose clock only moves when it is told to.
///
/// Time and timer firing are both explicit ([`ManualReactor::advance`]), which
/// is what makes it the reactor for tests: a test that would otherwise wait
/// 0.25 s for a subscription refresh steps 0.25 s instead, and gets the same
/// order every run. It also needs no runtime, so the machine's own tests stay
/// free of tokio.
///
/// Callbacks run on whatever thread calls `advance`.
pub struct ManualReactor {
    now: Mutex<f64>,
    timers: Mutex<Vec<ManualTimer>>,
}

struct ManualTimer {
    waketime: f64,
    callback: TimerCallback,
    /// Shared with the handle whose cancellation sets it.
    cancelled: Arc<AtomicBool>,
}

impl ManualReactor {
    /// Build a reactor whose clock reads `0.0`.
    pub fn new() -> Self {
        Self {
            now: Mutex::new(0.0),
            timers: Mutex::new(Vec::new()),
        }
    }

    /// A manual reactor behind the `Arc<dyn Reactor>` a `Printer` takes.
    ///
    /// For a caller that only needs the machine to have *a* clock and does not
    /// intend to step it — the lifecycle tests. A test that moves time keeps its
    /// own `Arc<ManualReactor>` instead, so it can call [`ManualReactor::advance`].
    pub fn shared() -> Arc<dyn Reactor> {
        Arc::new(Self::new())
    }

    /// Move the clock forward and run every timer that comes due on the way.
    ///
    /// The clock steps to each timer's wake time in turn, up to `delta` seconds
    /// from now, so a periodic timer fires once per period and its callback sees
    /// the time it was actually woken for — the same order a real reactor would
    /// produce. Timers due at the same moment fire in registration order (the
    /// sort is stable).
    ///
    /// Returns how many callbacks ran, which is what a test usually wants to
    /// assert on. A callback that keeps scheduling itself at the current time
    /// will keep firing, exactly as it would on a real reactor.
    pub fn advance(&self, delta: f64) -> usize {
        assert!(
            delta >= 0.0 && delta.is_finite(),
            "the reactor clock only moves forward: {delta}"
        );
        let target = self.monotonic() + delta;
        let mut fired = 0;

        while let Some(waketime) = self.next_wake_time(target) {
            {
                let mut now = self.now.lock().unwrap_or_else(|p| p.into_inner());
                // A timer registered with a wake time already in the past runs
                // now, not in the past: the clock never moves backwards.
                if waketime > *now {
                    *now = waketime;
                }
            }
            fired += self.run_due();
        }

        // No timer left to run: settle the clock exactly on the target, so a
        // quiet `advance` still moves time.
        let mut now = self.now.lock().unwrap_or_else(|p| p.into_inner());
        if *now < target {
            *now = target;
        }
        fired
    }

    /// The earliest live timer due at or before `limit`.
    fn next_wake_time(&self, limit: f64) -> Option<f64> {
        self.timers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .filter(|timer| !timer.cancelled.load(Ordering::Acquire))
            .map(|timer| timer.waketime)
            .filter(|waketime| *waketime <= limit)
            .min_by(f64::total_cmp)
    }

    /// Run every timer due at the current time, without moving the clock.
    ///
    /// This is what `advance` calls after stepping. It is also what a test uses
    /// to run a timer registered at `NOW` (`0.`): register it, then `run_due`.
    pub fn run_due(&self) -> usize {
        let mut fired = 0;
        loop {
            let now = *self.now.lock().unwrap_or_else(|p| p.into_inner());

            // Take the due timers out from under the lock: a callback may
            // register or cancel another timer, and it must not do that while
            // we hold the list.
            let mut due = Vec::new();
            {
                let mut timers = self.timers.lock().unwrap_or_else(|p| p.into_inner());
                let mut kept = Vec::with_capacity(timers.len());
                for timer in timers.drain(..) {
                    if timer.cancelled.load(Ordering::Acquire) {
                        continue;
                    }
                    if timer.waketime <= now {
                        due.push(timer);
                    } else {
                        kept.push(timer);
                    }
                }
                *timers = kept;
            }
            if due.is_empty() {
                return fired;
            }
            due.sort_by(|a, b| a.waketime.total_cmp(&b.waketime));

            for mut timer in due {
                if timer.cancelled.load(Ordering::Acquire) {
                    continue;
                }
                fired += 1;
                if let Some(waketime) = (timer.callback)(now) {
                    timer.waketime = waketime;
                    self.timers
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .push(timer);
                }
            }
        }
    }
}

impl Default for ManualReactor {
    fn default() -> Self {
        Self::new()
    }
}

impl Reactor for ManualReactor {
    fn monotonic(&self) -> f64 {
        *self.now.lock().unwrap_or_else(|p| p.into_inner())
    }

    fn register_timer_named(
        &self,
        _name: &'static str,
        callback: TimerCallback,
        waketime: f64,
    ) -> TimerHandle {
        let cancelled = Arc::new(AtomicBool::new(false));
        self.timers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(ManualTimer {
                waketime,
                callback,
                cancelled: Arc::clone(&cancelled),
            });
        TimerHandle::new(move || cancelled.store(true, Ordering::Release))
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;
    use tokio::sync::mpsc;

    // ---------------------------------------------------------------------
    // ManualReactor — the deterministic driver
    // ---------------------------------------------------------------------

    #[test]
    fn test_a_new_manual_reactor_starts_at_zero() {
        let reactor = ManualReactor::new();

        assert_eq!(reactor.monotonic(), 0.0);
    }

    #[test]
    fn test_advance_moves_the_clock_and_fires_when_due() {
        let reactor = ManualReactor::new();
        let count = Arc::new(AtomicUsize::new(0));
        let seen = Arc::clone(&count);
        reactor.register_timer(
            Box::new(move |_| {
                seen.fetch_add(1, Ordering::SeqCst);
                None
            }),
            1.0,
        );

        assert_eq!(reactor.advance(0.5), 0);
        assert_eq!(reactor.monotonic(), 0.5, "a quiet advance still moves time");
        assert_eq!(count.load(Ordering::SeqCst), 0, "fired early");

        assert_eq!(reactor.advance(0.5), 1);
        assert_eq!(count.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_a_callback_sees_the_time_it_was_woken_for() {
        let reactor = ManualReactor::new();
        let seen = Arc::new(Mutex::new(Vec::new()));
        let log = Arc::clone(&seen);
        reactor.register_timer(
            Box::new(move |eventtime| {
                log.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(eventtime);
                None
            }),
            0.75,
        );

        // The clock jumps over the wake time; the callback still sees 0.75, not
        // the 1.0 the clock ends on.
        reactor.advance(1.0);

        assert_eq!(*seen.lock().unwrap(), [0.75]);
        assert_eq!(reactor.monotonic(), 1.0);
    }

    #[test]
    fn test_a_timer_reschedules_with_its_return_value() {
        let reactor = ManualReactor::new();
        let count = Arc::new(AtomicUsize::new(0));
        let seen = Arc::clone(&count);
        reactor.register_timer(
            Box::new(move |eventtime| {
                seen.fetch_add(1, Ordering::SeqCst);
                Some(eventtime + 0.25)
            }),
            0.25,
        );

        assert_eq!(reactor.advance(1.0), 4);
        assert_eq!(count.load(Ordering::SeqCst), 4);
    }

    #[test]
    fn test_returning_none_retires_the_timer() {
        let reactor = ManualReactor::new();
        reactor.register_timer(Box::new(|_| None), 0.0);

        assert_eq!(reactor.run_due(), 1);
        assert_eq!(reactor.advance(10.0), 0);
    }

    #[test]
    fn test_a_timer_registered_for_now_runs_without_moving_the_clock() {
        let reactor = ManualReactor::new();
        reactor.register_timer(Box::new(|_| None), reactor.monotonic());

        assert_eq!(reactor.run_due(), 1);
    }

    #[test]
    fn test_a_cancelled_timer_never_runs() {
        let reactor = ManualReactor::new();
        let handle = reactor.register_timer(
            Box::new(|_| {
                panic!("cancelled");
            }),
            1.0,
        );

        reactor.unregister_timer(handle);

        assert_eq!(reactor.advance(10.0), 0);
    }

    #[test]
    fn test_cancelling_is_idempotent_and_safe_after_retirement() {
        let reactor = ManualReactor::new();
        let handle = reactor.register_timer(Box::new(|_| None), 0.0);
        assert_eq!(reactor.run_due(), 1);

        reactor.unregister_timer(handle.clone());
        reactor.unregister_timer(handle);
    }

    #[test]
    fn test_due_timers_fire_in_wake_time_order() {
        let reactor = ManualReactor::new();
        let order = Arc::new(Mutex::new(Vec::new()));
        for (name, waketime) in [("second", 2.0), ("first", 1.0), ("third", 3.0)] {
            let order = Arc::clone(&order);
            reactor.register_timer(
                Box::new(move |_| {
                    order.lock().unwrap().push(name);
                    None
                }),
                waketime,
            );
        }

        reactor.advance(3.0);

        assert_eq!(*order.lock().unwrap(), ["first", "second", "third"]);
    }

    #[test]
    fn test_a_callback_may_register_another_timer() {
        // The callback keeps a handle on the reactor so it can register into it
        // while it runs — the same shape a subscription uses to reschedule
        // itself. That reference cycle is the test's own; a real one is broken
        // by unregistering.
        let reactor = Arc::new(ManualReactor::new());
        let count = Arc::new(AtomicUsize::new(0));
        {
            let reactor = Arc::clone(&reactor);
            let count = Arc::clone(&count);
            let register_into = Arc::clone(&reactor);
            reactor.register_timer(
                Box::new(move |_| {
                    count.fetch_add(1, Ordering::SeqCst);
                    register_into.register_timer(Box::new(|_| None), register_into.monotonic());
                    None
                }),
                0.0,
            );
        }

        // The freshly registered timer is due immediately and runs after.
        assert_eq!(reactor.run_due(), 2);
        assert_eq!(count.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_call_later_runs_once_after_the_delay() {
        let reactor = ManualReactor::new();
        let count = Arc::new(AtomicUsize::new(0));
        let seen = Arc::clone(&count);
        reactor.call_later(
            0.5,
            Box::new(move |_| {
                seen.fetch_add(1, Ordering::SeqCst);
            }),
        );

        assert_eq!(reactor.advance(0.25), 0);
        assert_eq!(reactor.advance(0.25), 1);
        assert_eq!(reactor.advance(10.0), 0);
        assert_eq!(count.load(Ordering::SeqCst), 1);
    }

    // ---------------------------------------------------------------------
    // TokioReactor — the host's driver
    // ---------------------------------------------------------------------

    #[tokio::test(start_paused = true)]
    async fn test_a_tokio_timer_runs_when_its_waketime_arrives() {
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let (tx, mut rx) = mpsc::unbounded_channel();
        let waketime = reactor.monotonic() + 1.0;
        reactor.register_timer(
            Box::new(move |eventtime| {
                tx.send(eventtime).ok();
                None
            }),
            waketime,
        );

        tokio::time::advance(Duration::from_millis(999)).await;
        tokio::task::yield_now().await;
        assert!(rx.try_recv().is_err(), "the timer ran early");

        tokio::time::advance(Duration::from_millis(1)).await;
        let eventtime = rx.recv().await.expect("the timer must run");
        assert!(eventtime >= waketime, "{eventtime} < {waketime}");
    }

    #[tokio::test(start_paused = true)]
    async fn test_a_tokio_timer_reschedules_with_its_return_value() {
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let (tx, mut rx) = mpsc::unbounded_channel();
        reactor.register_timer(
            Box::new(move |eventtime| {
                tx.send(eventtime).ok();
                Some(eventtime + 0.25)
            }),
            0.25,
        );

        // The paused clock is stepped in periods rather than in one jump: a
        // timer that reschedules itself sleeps again from wherever the clock
        // now is, so each step is one period, hence one firing.
        for _ in 0..4 {
            tokio::time::advance(Duration::from_millis(250)).await;
            tokio::task::yield_now().await;
        }

        let mut fired = 0;
        while rx.try_recv().is_ok() {
            fired += 1;
        }
        assert_eq!(fired, 4, "four periods");
    }

    #[tokio::test(start_paused = true)]
    async fn test_a_cancelled_tokio_timer_wakes_at_once_and_does_not_run() {
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let (tx, mut rx) = mpsc::unbounded_channel();
        let handle = reactor.register_timer(
            Box::new(move |eventtime| {
                tx.send(eventtime).ok();
                None
            }),
            3600.0,
        );

        reactor.unregister_timer(handle);
        tokio::task::yield_now().await;
        tokio::time::advance(Duration::from_secs(7200)).await;
        tokio::task::yield_now().await;

        assert!(rx.try_recv().is_err(), "a cancelled timer ran");
    }

    #[tokio::test(start_paused = true)]
    async fn test_a_tokio_timer_may_be_retired_by_unregistering_after_it_stopped() {
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let handle = reactor.register_timer(Box::new(|_| None), 0.0);

        tokio::time::advance(Duration::from_millis(1)).await;
        tokio::task::yield_now().await;

        reactor.unregister_timer(handle.clone());
        reactor.unregister_timer(handle);
    }

    #[tokio::test(start_paused = true)]
    async fn test_tokio_timers_run_one_at_a_time_in_wake_time_order() {
        // The property the serial dispatcher exists for: timers due at the same
        // moment run in registration order, and no two callbacks are ever
        // active at once. A regression to one-task-per-timer would let them
        // overlap (and race the shared flag) instead of failing cleanly.
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let log = Arc::new(Mutex::new(Vec::new()));
        let running = Arc::new(AtomicBool::new(false));
        for name in ["first", "second", "third"] {
            let log = Arc::clone(&log);
            let running = Arc::clone(&running);
            reactor.register_timer(
                Box::new(move |_| {
                    assert!(
                        !running.swap(true, Ordering::SeqCst),
                        "two callbacks ran at once"
                    );
                    log.lock().unwrap().push(name);
                    running.store(false, Ordering::SeqCst);
                    None
                }),
                1.0,
            );
        }

        tokio::time::advance(Duration::from_millis(1000)).await;
        // The whole due batch runs in one poll of the dispatcher, but yield a
        // few times so the assertion sees it either way.
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }

        assert_eq!(*log.lock().unwrap(), ["first", "second", "third"]);
    }

    #[tokio::test(start_paused = true)]
    async fn test_a_tokio_callback_may_register_another_timer() {
        // The dispatcher must not hold its lock while a callback runs, or a
        // callback that registers a timer would deadlock. The callback holds an
        // `Arc` back to the reactor, which is the test's own reference cycle.
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let (tx, mut rx) = mpsc::unbounded_channel();
        {
            let for_callback = Arc::clone(&reactor);
            let tx = tx.clone();
            reactor.register_timer(
                Box::new(move |_| {
                    let reactor = Arc::clone(&for_callback);
                    let tx = tx.clone();
                    reactor.register_timer(
                        Box::new(move |_| {
                            tx.send(()).ok();
                            None
                        }),
                        reactor.monotonic(),
                    );
                    None
                }),
                0.5,
            );
        }

        tokio::time::advance(Duration::from_millis(500)).await;
        rx.recv().await.expect("the nested timer must run");
    }

    #[tokio::test(start_paused = true)]
    async fn test_tokio_call_later_runs_once() {
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let (tx, mut rx) = mpsc::unbounded_channel();
        reactor.call_later(
            0.5,
            Box::new(move |_| {
                tx.send(()).ok();
            }),
        );

        tokio::time::advance(Duration::from_millis(500)).await;
        rx.recv().await.expect("the callback must run");
        tokio::time::advance(Duration::from_secs(60)).await;
        tokio::task::yield_now().await;
        assert!(rx.try_recv().is_err(), "call_later ran twice");
    }

    // ---------------------------------------------------------------------
    // Latency notifier — real time, since the paused clock cannot see a
    // callback that blocks the dispatcher
    // ---------------------------------------------------------------------

    #[tokio::test]
    async fn test_a_slow_round_is_reported_by_the_latency_notifier() {
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let (tx, mut rx) = mpsc::unbounded_channel();
        reactor.set_latency_notifier(
            0.05,
            Arc::new(move |report| {
                tx.send(report).ok();
            }),
        );

        // Registered at NOW: the dispatcher runs it as its first round, and it
        // holds the dispatcher for 120 ms.
        reactor.register_timer_named(
            "slow",
            Box::new(|_| {
                std::thread::sleep(Duration::from_millis(120));
                None
            }),
            reactor.monotonic(),
        );

        let report = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .expect("the report must arrive")
            .expect("the channel must be open");
        assert!(report.busy >= 0.05, "busy was {}", report.busy);
        let slow = report
            .callbacks
            .iter()
            .find(|run| run.name == "slow")
            .expect("the slow callback must be named");
        assert!(slow.duration >= 0.05, "duration was {}", slow.duration);
    }

    #[tokio::test]
    async fn test_a_fast_round_is_not_reported() {
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let (tx, mut rx) = mpsc::unbounded_channel();
        reactor.set_latency_notifier(
            0.05,
            Arc::new(move |report| {
                tx.send(report).ok();
            }),
        );
        reactor.register_timer_named("fast", Box::new(|_| None), reactor.monotonic());

        // Long enough for the dispatcher to run the (fast) round and decide not
        // to report; the channel staying empty is the assertion.
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert!(rx.try_recv().is_err(), "a fast round must not be reported");
    }

    #[tokio::test]
    async fn test_a_timer_that_waited_behind_a_slow_callback_is_reported_late() {
        // The wake latency A1b is about: `late` is due at +20 ms, but the
        // dispatcher is stuck in `blocker`, so it starts ~100 ms late.
        let reactor = TokioReactor::new(tokio::runtime::Handle::current());
        let (tx, mut rx) = mpsc::unbounded_channel();
        reactor.set_latency_notifier(
            0.05,
            Arc::new(move |report| {
                tx.send(report).ok();
            }),
        );
        reactor.register_timer_named(
            "blocker",
            Box::new(|_| {
                std::thread::sleep(Duration::from_millis(120));
                None
            }),
            reactor.monotonic(),
        );
        reactor.register_timer_named("late", Box::new(|_| None), reactor.monotonic() + 0.02);

        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            let report = tokio::time::timeout(remaining, rx.recv())
                .await
                .expect("a report must arrive")
                .expect("the channel must be open");
            if let Some(late) = report.callbacks.iter().find(|run| run.name == "late") {
                assert!(late.lateness >= 0.05, "lateness was {}", late.lateness);
                return;
            }
        }
    }
}
