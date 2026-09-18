//! The reactor: the machine's clock, and the timers it wakes on.
//!
//! A printer is not a runtime. Upstream gives every printer a reactor
//! (`Printer.get_reactor()`, 121 call sites) and everything asynchronous goes
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
//! [`Printer`]: crate::core::klippy::printer::Printer

use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// A timer callback: given the event time, return the next wake time.
///
/// `None` stops the timer, which is how a one-shot ends and how a periodic one
/// is retired. The signature is upstream's
/// `double callback(double eventtime)` returning `NEVER`
/// (`klippy/reactor.py:166`), with `Option` in place of the sentinel.
pub type TimerCallback = Box<dyn FnMut(f64) -> Option<f64> + Send>;

/// A one-shot callback handed to [`Reactor::call_later`].
pub type OneShot = Box<dyn FnOnce(f64) + Send>;

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

    /// Call `callback` no earlier than `waketime` (absolute monotonic seconds).
    ///
    /// The callback may be called again whenever it asks to be, by returning
    /// the next wake time; returning `None` retires it. The returned handle is
    /// how it is cancelled ([`Reactor::unregister_timer`]).
    ///
    /// A `waketime` already in the past runs as soon as the implementer gets to
    /// it, which is upstream's `NOW` (`0.`).
    fn register_timer(&self, callback: TimerCallback, waketime: f64) -> TimerHandle;

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
        self.register_timer(
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

/// The reactor the host runs on: timers are tokio tasks.
///
/// Built over the runtime the host already has, so it creates nothing of its
/// own — no thread, no event loop. `monotonic` is the runtime's clock, which
/// means a paused test clock (`#[tokio::test(start_paused = true)]`) moves it
/// too.
///
/// A registered timer is one spawned task that sleeps until its wake time and
/// runs the callback there. Cancelling sets a flag and wakes the task, so a
/// timer does not have to wait out a long sleep to notice.
pub struct TokioReactor {
    handle: tokio::runtime::Handle,
    /// What `monotonic` counts from.
    origin: tokio::time::Instant,
}

impl TokioReactor {
    /// Build a reactor over `handle`.
    ///
    /// The handle may be any runtime handle — the future that registered a
    /// timer runs on that runtime, so it must outlive the printer. The caller
    /// is expected to be inside the runtime already (`Handle::current()`), or
    /// to hold a handle from one it built.
    pub fn new(handle: tokio::runtime::Handle) -> Self {
        Self {
            handle,
            origin: tokio::time::Instant::now(),
        }
    }
}

impl Reactor for TokioReactor {
    fn monotonic(&self) -> f64 {
        self.origin.elapsed().as_secs_f64()
    }

    fn register_timer(&self, callback: TimerCallback, waketime: f64) -> TimerHandle {
        // Cancellation is a flag the task checks plus a notification to wake it
        // from a long sleep. A `Notify` rather than a channel, because dropping
        // a handle must not cancel: the task owns its own copy of the state, so
        // it stays alive whether or not anyone still holds the handle.
        let cancel = Arc::new(CancelState {
            cancelled: AtomicBool::new(false),
            notify: tokio::sync::Notify::new(),
        });
        let origin = self.origin;
        self.handle
            .spawn(run_timer(origin, callback, waketime, Arc::clone(&cancel)));
        TimerHandle::new(move || {
            cancel.cancelled.store(true, Ordering::Release);
            cancel.notify.notify_one();
        })
    }
}

/// How a running timer is told to stop.
///
/// Shared between the handle and the task, so it outlives either.
struct CancelState {
    cancelled: AtomicBool,
    notify: tokio::sync::Notify,
}

/// The longest a timer task sleeps in one go.
///
/// A wake time is an `f64` and may be arbitrarily far away — upstream's
/// `NEVER` is `9999999999999999.`. Sleeping that long in one call would
/// overflow a monotonic instant, so the task sleeps a day at a time and
/// re-checks. Nothing in the host asks for more than a few seconds.
const MAX_TIMER_SLEEP: Duration = Duration::from_secs(24 * 60 * 60);

/// One spawned timer: sleep until due, run the callback, reschedule or stop.
async fn run_timer(
    origin: tokio::time::Instant,
    mut callback: TimerCallback,
    mut waketime: f64,
    cancel: Arc<CancelState>,
) {
    loop {
        if cancel.cancelled.load(Ordering::Acquire) {
            return;
        }
        // Sleep until due, in bounded slices, waking at once if cancelled.
        loop {
            let remaining = waketime - origin.elapsed().as_secs_f64();
            if remaining <= 0.0 {
                break;
            }
            let slice = Duration::from_secs_f64(remaining).min(MAX_TIMER_SLEEP);
            // Interest is registered before the flag is read again, so a
            // cancel in between is seen rather than lost.
            let notified = cancel.notify.notified();
            if cancel.cancelled.load(Ordering::Acquire) {
                return;
            }
            tokio::select! {
                _ = tokio::time::sleep(slice) => {}
                _ = notified => {
                    if cancel.cancelled.load(Ordering::Acquire) {
                        return;
                    }
                }
            }
        }
        let eventtime = origin.elapsed().as_secs_f64();
        match callback(eventtime) {
            Some(next) => waketime = next,
            None => return,
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

    fn register_timer(&self, callback: TimerCallback, waketime: f64) -> TimerHandle {
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
}
