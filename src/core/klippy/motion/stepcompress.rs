//! Turning step times into firmware commands.
//!
//! Upstream's `chelper/stepcompress.c`: the step solver hands this module the
//! times at which a stepper must step, and it turns them into `queue_step`
//! commands. Upstream compresses a run of evenly spaced steps into one command
//! (`interval`, `count`, `add`), which is what keeps the MCU's move queue from
//! overflowing.
//!
//! # This is the simplified implementation
//!
//! Every step becomes its own `queue_step` with `count=1, add=0`. It is enough
//! to make the motion path work end to end, but a real print will flood the
//! move queue. The full compressor — the `(interval, count, add)` search with
//! its quadratic deviation bound — is FW5f.
//!
//! Because that failure is not obvious from the outside, [`warn_and_wait`]
//! announces it once, and gives the operator a window to abort before motion
//! starts.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use tokio::time::sleep;
use tracing::warn;

/// The minimum spacing between a step, a direction change and the next step, in
/// seconds (`SDS_FILTER_TIME`, `chelper/stepcompress.c:505`).
const SDS_FILTER_TIME: f64 = 0.000_750;

/// One command the compressor produced.
///
/// `count` and `add` are always `1` and `0` here; they exist for the full
/// compressor to fill in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepCommand {
    /// `set_next_step_dir oid=%c dir=%c` — set the direction pin.
    SetNextStepDir { oid: u32, direction: bool },
    /// `queue_step oid=%c interval=%u count=%u add=%i` — take `count` steps,
    /// the first `interval` ticks after the previous one and each later one
    /// `add` ticks further apart.
    QueueStep {
        oid: u32,
        interval: u32,
        count: u32,
        add: i32,
    },
}

/// A step waiting to be emitted.
///
/// Upstream holds one step back so that a step immediately followed by a
/// direction change and another step can be rolled back
/// (`stepcompress_append`, `chelper/stepcompress.c:508-537`).
#[derive(Debug, Clone, Copy)]
struct PendingStep {
    clock: i64,
    direction: bool,
}

/// The simplified step compressor for one stepper.
///
/// "One stepper" is upstream's `struct stepcompress`: a `syncemitter` owns one,
/// and it knows only the oid and the clock mapping.
#[derive(Debug)]
pub struct StepCompressor {
    oid: u32,
    mcu_freq: f64,
    /// The print time the clock anchor corresponds to
    /// (`stepcompress_set_time`'s offset).
    last_step_print_time: f64,
    /// The clock anchor: the clock that `last_step_print_time` maps to
    /// (upstream's `last_step_clock`). It does not move as steps are emitted.
    last_step_clock: i64,
    /// The clock of the last emitted step, for the interval arithmetic.
    prev_step_clock: i64,
    /// The direction the firmware is set to, or `None` before any step has
    /// been emitted (upstream starts with `sdir = -1`, so the first step always
    /// sends `set_next_step_dir`).
    step_dir: Option<bool>,
    pending: Option<PendingStep>,
    out: Vec<StepCommand>,
}

impl StepCompressor {
    /// A compressor for `oid`, converting with `mcu_freq` ticks per second.
    pub fn new(oid: u32, mcu_freq: f64) -> Self {
        Self {
            oid,
            mcu_freq,
            last_step_print_time: 0.0,
            last_step_clock: 0,
            prev_step_clock: 0,
            step_dir: None,
            pending: None,
            out: Vec::new(),
        }
    }

    /// The stepper's oid.
    pub fn oid(&self) -> u32 {
        self.oid
    }

    /// The frequency used to turn print seconds into clock ticks.
    pub fn mcu_freq(&self) -> f64 {
        self.mcu_freq
    }

    /// Set the clock mapping (`stepcompress_set_time`).
    pub fn set_time(&mut self, time_offset: f64, mcu_freq: f64) {
        self.last_step_print_time = time_offset;
        self.mcu_freq = mcu_freq;
    }

    /// The clock of the last emitted step.
    pub fn prev_step_clock(&self) -> i64 {
        self.prev_step_clock
    }

    /// The direction a solver should assume.
    ///
    /// Before any step has been emitted the direction is unknown
    /// (`stepcompress_alloc` sets `sdir = -1`); positive is what the solver
    /// wants, since the first target is one half step forward.
    pub fn step_dir(&self) -> bool {
        self.step_dir.unwrap_or(true)
    }

    /// Note where the firmware's step counter is
    /// (`stepcompress_set_last_position`).
    pub fn set_last_position(&mut self, clock: i64) {
        self.last_step_clock = clock;
        self.prev_step_clock = clock;
    }

    /// Add the next step time (`stepcompress_append`).
    ///
    /// `sdir` is the direction of this step, `print_time` the print time of the
    /// move it belongs to, and `step_time` the time into that move.
    pub fn append(&mut self, sdir: bool, print_time: f64, step_time: f64) {
        let offset = print_time - self.last_step_print_time;
        let relative = (step_time + offset) * self.mcu_freq;
        let step_clock = self.last_step_clock + relative as i64;
        if let Some(pending) = self.pending.take() {
            if sdir != pending.direction {
                let diff = step_clock - pending.clock;
                if diff < (SDS_FILTER_TIME * self.mcu_freq) as i64 {
                    // Too close to turn around: drop the earlier step rather
                    // than emit a step+dir+step burst.
                    self.pending = Some(PendingStep {
                        clock: step_clock,
                        direction: sdir,
                    });
                    return;
                }
            }
            self.emit_step(pending.clock, pending.direction);
        }
        self.pending = Some(PendingStep {
            clock: step_clock,
            direction: sdir,
        });
    }

    /// Emit the pending step even though it can no longer be rolled back
    /// (`stepcompress_commit`).
    pub fn commit(&mut self) {
        if let Some(pending) = self.pending.take() {
            self.emit_step(pending.clock, pending.direction);
        }
    }

    /// Emit the pending step once `move_clock` has reached it
    /// (`stepcompress_flush`).
    pub fn flush(&mut self, move_clock: i64) {
        if let Some(pending) = self.pending {
            if move_clock >= pending.clock {
                self.pending = None;
                self.emit_step(pending.clock, pending.direction);
            }
        }
    }

    /// Take the commands produced since the last call.
    pub fn take_commands(&mut self) -> Vec<StepCommand> {
        std::mem::take(&mut self.out)
    }

    fn emit_step(&mut self, clock: i64, direction: bool) {
        if Some(direction) != self.step_dir {
            self.out.push(StepCommand::SetNextStepDir {
                oid: self.oid,
                direction,
            });
            self.step_dir = Some(direction);
        }
        // The simplified compressor: one step per command, so the interval is
        // the gap since the previous step, and the first step's gap is from
        // the clock anchor (upstream's `minmax_point` measures from
        // `last_step_clock` too).
        let interval = (clock - self.prev_step_clock).max(0) as u32;
        self.out.push(StepCommand::QueueStep {
            oid: self.oid,
            interval,
            count: 1,
            add: 0,
        });
        self.prev_step_clock = clock;
    }
}

/// How the simplified compressor announces itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GracePolicy {
    /// How many warning lines to print.
    pub warnings: u32,
    /// The pause between them.
    pub interval: Duration,
    /// The pause after the last one, before motion may start.
    pub grace: Duration,
}

impl GracePolicy {
    /// Three warnings a second apart, then five seconds to abort.
    pub const DEFAULT: Self = Self {
        warnings: 3,
        interval: Duration::from_secs(1),
        grace: Duration::from_secs(5),
    };

    /// No warnings and no wait, for tests.
    pub const QUIET: Self = Self {
        warnings: 0,
        interval: Duration::ZERO,
        grace: Duration::ZERO,
    };
}

/// Set once the simplified compressor has announced itself.
static ANNOUNCED: AtomicBool = AtomicBool::new(false);

/// Announce the simplified compressor and give the operator a window to abort.
///
/// The body runs **once per process**: the first caller prints the warning
/// [`GracePolicy::warnings`] times, [`GracePolicy::interval`] apart, then waits
/// [`GracePolicy::grace`] before returning. Later callers return immediately.
/// Returns whether this call was the one that announced.
///
/// The wait is `async` on purpose: the API and `M112` must stay responsive
/// during the grace period, because they are how an operator may abort.
pub async fn warn_and_wait(policy: GracePolicy) -> bool {
    warn_and_wait_once(policy, &ANNOUNCED).await
}

/// [`warn_and_wait`] with an injectable latch, so tests do not share state.
async fn warn_and_wait_once(policy: GracePolicy, announced: &AtomicBool) -> bool {
    if announced.swap(true, Ordering::SeqCst) {
        return false;
    }
    const MESSAGE: &str = "stepcompress is the simplified implementation: one queue_step per \
                           step, which will overflow the MCU move queue on a real print; the \
                           full compressor is FW5f";
    for _ in 0..policy.warnings {
        warn!("{MESSAGE}");
        if !policy.interval.is_zero() {
            sleep(policy.interval).await;
        }
    }
    if !policy.grace.is_zero() {
        warn!(
            "stepcompress: starting motion in {:.1}s; interrupt now (Ctrl-C or M112) to abort",
            policy.grace.as_secs_f64()
        );
        sleep(policy.grace).await;
    }
    true
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// A compressor at 1 MHz, so one millisecond is 1000 ticks.
    fn compressor() -> StepCompressor {
        StepCompressor::new(3, 1_000_000.0)
    }

    /// The `queue_step` commands, in order.
    fn steps(commands: &[StepCommand]) -> Vec<(u32, u32, i32)> {
        commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep {
                    interval,
                    count,
                    add,
                    ..
                } => Some((*interval, *count, *add)),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .collect()
    }

    #[test]
    fn test_every_step_becomes_one_queue_step() {
        let mut sc = compressor();
        // One step a millisecond, five of them (the first at 1 ms, so the
        // anchor gap is a real interval).
        for i in 1..=5 {
            sc.append(true, 0.0, i as f64 * 0.001);
        }
        sc.commit();

        let commands = sc.take_commands();
        assert_eq!(
            commands[0],
            StepCommand::SetNextStepDir {
                oid: 3,
                direction: true
            }
        );
        let steps = steps(&commands);
        assert_eq!(steps.len(), 5);
        assert!(steps.iter().all(|(_, count, add)| *count == 1 && *add == 0));
        // Every gap is 1 ms, including the first (from the clock anchor at 0).
        assert!(steps.iter().all(|(interval, ..)| *interval == 1000));
    }

    #[test]
    fn test_a_direction_change_emits_set_next_step_dir() {
        let mut sc = compressor();
        sc.append(true, 0.0, 0.0);
        sc.append(true, 0.0, 0.001);
        sc.append(false, 0.0, 0.002);
        sc.commit();

        let commands = sc.take_commands();
        let dirs: Vec<_> = commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::SetNextStepDir { direction, .. } => Some(*direction),
                StepCommand::QueueStep { .. } => None,
            })
            .collect();
        // true once at the start, false once when the direction reverses.
        assert_eq!(dirs, [true, false]);
    }

    #[test]
    fn test_a_step_too_close_to_a_reversal_is_dropped() {
        let mut sc = compressor();
        sc.append(true, 0.0, 0.0);
        sc.append(true, 0.0, 0.001);
        // 0.0001 s after the last step: inside SDS_FILTER_TIME, so the earlier
        // pending step is dropped instead of emitting step+dir+step.
        sc.append(false, 0.0, 0.0011);
        sc.commit();

        let steps = steps(&sc.take_commands());
        assert_eq!(steps.len(), 2);
    }

    #[test]
    fn test_flush_waits_for_the_move_clock() {
        let mut sc = compressor();
        sc.append(true, 0.0, 0.005);

        // Not yet: the pending step is at 5000 ticks.
        sc.flush(4999);
        assert!(sc.take_commands().is_empty());

        sc.flush(5000);
        assert_eq!(steps(&sc.take_commands()).len(), 1);
    }

    #[test]
    fn test_set_last_position_moves_the_anchor() {
        let mut sc = compressor();
        sc.set_last_position(10_000);
        sc.append(true, 0.0, 0.001);
        sc.commit();

        // 1 ms past the new anchor at 10000 ticks.
        assert_eq!(steps(&sc.take_commands())[0].0, 1_000);
    }

    #[tokio::test]
    async fn test_the_announcement_runs_once() {
        let announced = AtomicBool::new(false);

        assert!(warn_and_wait_once(GracePolicy::QUIET, &announced).await);
        // The second caller does not repeat it.
        assert!(!warn_and_wait_once(GracePolicy::QUIET, &announced).await);
    }

    #[tokio::test]
    async fn test_a_quiet_policy_does_not_wait() {
        let announced = AtomicBool::new(false);
        let started = std::time::Instant::now();

        warn_and_wait_once(GracePolicy::QUIET, &announced).await;

        // No warnings and no grace means no sleeping at all.
        assert!(started.elapsed() < Duration::from_millis(50));
    }
}
