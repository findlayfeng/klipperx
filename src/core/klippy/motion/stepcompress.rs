//! Turning step times into firmware commands.
//!
//! Upstream's `chelper/stepcompress.c`: the step solver hands this module the
//! times at which a stepper must step, and it turns them into `queue_step`
//! commands. Upstream compresses a run of steps into one command
//! (`interval`, `count`, `add`), which is what keeps the MCU's move queue from
//! overflowing.
//!
//! This is the **full** compressor (FW5f). It is a direct port of
//! `compress_bisect_add` (`chelper/stepcompress.c`): the step times are kept in
//! a small queue, and for each command an `add` value is guessed and the longest
//! run of steps that still fits the firmware's `interval + add` schedule is
//! taken, bounded by `max_error` (the schedule may be off by at most that many
//! clock ticks). A quadratic-deviation bound (`QUADRATIC_DEV`) keeps two
//! different `add` sequences from being confused, and `add = 0` wins ties.
//!
//! The early FW5c simplification (one `queue_step` per step) is gone; the
//! announcement and grace window it needed went with it.
//!
//! # The internal queue
//!
//! Upstream stores the lower 32 bits of each step's clock (`struct qstep`) to
//! cut memory, and keeps `last_step_clock` as 64 bits. That is kept here: the
//! queue holds `u32` clocks and all the compressor arithmetic is modulo 2^32,
//! while `last_step_clock` and the emitted step times are 64-bit. A step far
//! from the last one (more than `CLOCK_DIFF_MAX`) is sent as its own one-step
//! command rather than being queued, which is also what re-anchors the clock
//! after connect.
//!
//! # Errors
//!
//! `check_line` verifies every emitted sequence against the requested step
//! times, exactly as upstream's always-on `CHECK_LINES`. A mismatch is an
//! internal error: it means the compressor produced a schedule the requested
//! steps do not fit, which upstream reports as `Internal error in stepcompress`
//! and shuts the printer down for. The error travels up through the solver and
//! the motion queue to the flush task.

use std::collections::VecDeque;
use std::fmt;

/// The maximum schedule error the compressor may introduce, in seconds
/// (`MAX_STEPCOMPRESS_ERROR`, `klippy/stepper.py:19`).
pub const MAX_STEPCOMPRESS_ERROR: f64 = 0.000_025;

/// The minimum spacing between a step, a direction change and the next step, in
/// seconds (`SDS_FILTER_TIME`, `chelper/stepcompress.c:505`).
const SDS_FILTER_TIME: f64 = 0.000_750;

/// The largest clock delta the internal queue handles directly
/// (`CLOCK_DIFF_MAX`, `chelper/stepcompress.c:344`).
const CLOCK_DIFF_MAX: u64 = 3 << 28;

/// The quadratic-deviation constant (`QUADRATIC_DEV`,
/// `chelper/stepcompress.c:110`).
const QUADRATIC_DEV: i64 = 11;

/// One command the compressor produced.
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

/// The compressor produced a schedule the requested step times do not fit.
///
/// This is an internal error: it means the port of `compress_bisect_add` is
/// wrong or its arithmetic overflowed. Upstream shuts the printer down for it
/// (`Internal error in stepcompress`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepCompressError {
    message: String,
}

impl StepCompressError {
    /// An error with `message`.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// The message.
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for StepCompressError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for StepCompressError {}

/// Parameters of one `queue_step` command (`struct step_move`,
/// `chelper/stepcompress.c:52`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StepMove {
    interval: u32,
    count: u16,
    add: i16,
}

/// The minimum and maximum acceptable clock for one queued step
/// (`struct points`).
#[derive(Debug, Clone, Copy)]
struct Points {
    minp: i64,
    maxp: i64,
}

/// Integer ceiling division that keeps upstream's sign handling
/// (`idiv_up`, `chelper/stepcompress.c:69`).
fn idiv_up(n: i64, d: i64) -> i64 {
    if n >= 0 {
        (n + d - 1) / d
    } else {
        n / d
    }
}

/// Integer floor division that keeps upstream's sign handling
/// (`idiv_down`, `chelper/stepcompress.c:75`).
fn idiv_down(n: i64, d: i64) -> i64 {
    if n >= 0 {
        n / d
    } else {
        (n - d + 1) / d
    }
}

/// One recently sent `queue_step`, for finding a past position
/// (`struct history_steps`, `chelper/stepcompress.c:57`).
///
/// `step_count` is signed by direction; `interval`/`add` are the command's, so a
/// clock inside the run can be solved for the step offset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoryStep {
    /// The first step's clock.
    pub first_clock: u64,
    /// The last step's clock.
    pub last_clock: u64,
    /// The step position before this run.
    pub start_position: i64,
    /// Steps taken by this run, signed.
    pub step_count: i64,
    /// The command's `interval`.
    pub interval: u32,
    /// The command's `add`.
    pub add: i32,
}

/// The full step compressor for one stepper.
///
/// "One stepper" is upstream's `struct stepcompress`: a `syncemitter` owns one,
/// and it knows only the oid and the clock mapping.
#[derive(Debug)]
pub struct StepCompressor {
    oid: u32,
    mcu_freq: f64,
    /// The print time clock 0 maps to (`stepcompress_set_time`'s offset).
    mcu_time_offset: f64,
    /// The print time of `last_step_clock`
    /// (`calc_last_step_print_time`).
    last_step_print_time: f64,
    /// The largest schedule error allowed, in clock ticks.
    max_error: u32,
    /// The clock of the last scheduled step, 64-bit.
    last_step_clock: u64,
    /// The queued step clocks (lower 32 bits each).
    queue: VecDeque<u32>,
    /// The first queued step still needing compression.
    queue_pos: usize,
    /// One past the last queued step.
    queue_next: usize,
    /// The direction the firmware has been set to, or `-1` before any step has
    /// been emitted (upstream's `sdir`).
    sdir: i8,
    /// `invert_sdir` (upstream applies it when the dir command is sent; here the
    /// wire layer does, so it stays for parity and tests).
    invert_sdir: bool,
    /// The pending step's clock, `0` when there is none
    /// (upstream's `next_step_clock` sentinel).
    next_step_clock: u64,
    /// The pending step's direction.
    next_step_dir: bool,
    /// The last step position, for history/future use.
    last_position: i64,
    /// Recently sent runs, newest first (`history_list`).
    history: VecDeque<HistoryStep>,
    out: Vec<StepCommand>,
}

impl StepCompressor {
    /// A compressor for `oid`, converting with `mcu_freq` ticks per second.
    pub fn new(oid: u32, mcu_freq: f64) -> Self {
        Self {
            oid,
            mcu_freq,
            mcu_time_offset: 0.0,
            last_step_print_time: 0.0,
            max_error: (MAX_STEPCOMPRESS_ERROR * mcu_freq).max(0.0) as u32,
            last_step_clock: 0,
            queue: VecDeque::new(),
            queue_pos: 0,
            queue_next: 0,
            sdir: -1,
            invert_sdir: false,
            next_step_clock: 0,
            next_step_dir: false,
            last_position: 0,
            history: VecDeque::new(),
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

    /// The print-time-to-clock offset this compressor was set to
    /// (`stepcompress_set_time`'s `time_offset`): the print time that clock 0
    /// corresponds to.
    ///
    /// It is `0.0` for the primary MCU and the `SecondarySync` alignment for a
    /// secondary one.
    pub fn time_offset(&self) -> f64 {
        self.mcu_time_offset
    }

    /// Convert an absolute print time to this stepper's firmware clock
    /// (`MCU.print_time_to_clock`).
    ///
    /// This is the bound the step compressor flushes up to; each MCU has its
    /// own offset and frequency, so the caller asks the stepper rather than
    /// assuming one MCU's clock.
    pub fn print_time_to_clock(&self, print_time: f64) -> u64 {
        ((print_time - self.mcu_time_offset) * self.mcu_freq).max(0.0) as u64
    }

    /// The clock of the last scheduled step, 64-bit.
    pub fn last_step_clock(&self) -> u64 {
        self.last_step_clock
    }

    /// The direction of the pending (or last) step, as the solver should assume.
    pub fn step_dir(&self) -> bool {
        self.next_step_dir
    }

    /// Invert the stepper direction (`stepcompress_set_invert_sdir`).
    ///
    /// Only meaningful before steps are queued; upstream flips the tracked
    /// direction if one was already set.
    pub fn set_invert_sdir(&mut self, invert_sdir: bool) {
        if invert_sdir != self.invert_sdir {
            self.invert_sdir = invert_sdir;
            if self.sdir >= 0 {
                self.sdir ^= 1;
            }
        }
    }

    /// Set the largest schedule error allowed, in clock ticks
    /// (`stepcompress_fill`'s `max_error`).
    ///
    /// [`StepCompressor::new`] derives the production value from `mcu_freq`; a
    /// test that wants a specific compressor geometry sets it directly.
    pub fn set_max_error(&mut self, max_error: u32) {
        self.max_error = max_error;
    }

    /// Set the print-time-to-clock mapping (`stepcompress_set_time`).
    pub fn set_time(&mut self, time_offset: f64, mcu_freq: f64) {
        self.mcu_time_offset = time_offset;
        self.mcu_freq = mcu_freq;
        self.calc_last_step_print_time();
    }

    /// Note where the firmware's step counter is
    /// (`stepcompress_set_last_position`).
    ///
    /// A history marker is recorded at `clock`, so a position lookup afterwards
    /// knows the counter was `position` there.
    ///
    /// # Errors
    /// As [`StepCompressor::flush`] for any pending step it has to emit first.
    pub fn set_last_position(
        &mut self,
        clock: u64,
        position: i64,
    ) -> Result<(), StepCompressError> {
        self.flush(u64::MAX)?;
        self.last_position = position;
        self.history.push_front(HistoryStep {
            first_clock: clock,
            last_clock: clock,
            start_position: position,
            step_count: 0,
            interval: 0,
            add: 0,
        });
        Ok(())
    }

    /// The last step position noted by [`StepCompressor::set_last_position`].
    pub fn last_position(&self) -> i64 {
        self.last_position
    }

    /// The step position at a past clock (`stepcompress_find_past_position`).
    ///
    /// The history is walked newest first: before a run the counter is its
    /// `start_position`, after it the fully-counted position, and inside it the
    /// offset is solved from the run's `interval`/`add` (a quadratic when `add`
    /// is non-zero).
    pub fn find_past_position(&self, clock: u64) -> i64 {
        let mut last_position = self.last_position;
        for entry in &self.history {
            if clock < entry.first_clock {
                last_position = entry.start_position;
                continue;
            }
            if clock >= entry.last_clock {
                return entry.start_position + entry.step_count;
            }
            let interval = i64::from(entry.interval);
            let add = i64::from(entry.add);
            let ticks = (clock.wrapping_sub(entry.first_clock) as u32 as i32) as i64 + interval;
            let offset = if add == 0 {
                ticks / interval
            } else {
                // Solve `add*count*(count-1)/2 + interval*count = ticks`.
                let a = 0.5 * add as f64;
                let b = interval as f64 - 0.5 * add as f64;
                let c = -(ticks as f64);
                (((b * b - 4.0 * a * c).sqrt() - b) / (2.0 * a)) as i64
            };
            if entry.step_count < 0 {
                return entry.start_position - offset;
            }
            return entry.start_position + offset;
        }
        last_position
    }

    /// The recently sent runs overlapping `start_clock..end_clock`, newest first
    /// (`stepcompress_extract_old`).
    pub fn extract_old(&self, max: usize, start_clock: u64, end_clock: u64) -> Vec<HistoryStep> {
        let mut out = Vec::new();
        for entry in &self.history {
            if start_clock >= entry.last_clock || out.len() >= max {
                break;
            }
            if end_clock <= entry.first_clock {
                continue;
            }
            out.push(*entry);
        }
        out
    }

    /// Drop history that ended at or before `end_clock`
    /// (`stepcompress_history_expire`).
    pub fn history_expire(&mut self, end_clock: u64) {
        while let Some(oldest) = self.history.back() {
            if oldest.last_clock > end_clock {
                break;
            }
            self.history.pop_back();
        }
    }

    /// Reset the compressor's clock and direction
    /// (`stepcompress_reset`, used after homing).
    ///
    /// # Errors
    /// As [`StepCompressor::flush`].
    pub fn reset(&mut self, last_step_clock: u64) -> Result<(), StepCompressError> {
        self.flush(u64::MAX)?;
        self.last_step_clock = last_step_clock;
        self.sdir = -1;
        self.calc_last_step_print_time();
        Ok(())
    }

    /// Add the next step time (`stepcompress_append`).
    ///
    /// `sdir` is the direction of this step, `print_time` the print time of the
    /// move it belongs to, and `step_time` the time into that move.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from the compression.
    pub fn append(
        &mut self,
        sdir: bool,
        print_time: f64,
        step_time: f64,
    ) -> Result<(), StepCompressError> {
        let offset = print_time - self.last_step_print_time;
        let rel_sc = (step_time + offset) * self.mcu_freq;
        let step_clock = self.last_step_clock.wrapping_add(rel_sc as u64);
        if self.next_step_clock != 0 {
            if sdir != self.next_step_dir {
                let diff = step_clock.wrapping_sub(self.next_step_clock) as i64 as f64;
                if diff < SDS_FILTER_TIME * self.mcu_freq {
                    // Rollback last step to avoid rapid step+dir+step.
                    self.next_step_clock = 0;
                    self.next_step_dir = sdir;
                    return Ok(());
                }
            }
            self.queue_append()?;
        }
        self.next_step_clock = step_clock;
        self.next_step_dir = sdir;
        Ok(())
    }

    /// Emit the pending step even though it can no longer be rolled back
    /// (`stepcompress_commit`).
    ///
    /// # Errors
    /// As [`StepCompressor::append`].
    pub fn commit(&mut self) -> Result<(), StepCompressError> {
        if self.next_step_clock != 0 {
            self.queue_append()?;
        }
        Ok(())
    }

    /// Emit the pending step once `move_clock` has reached it, then compress the
    /// queued steps up to `move_clock` (`stepcompress_flush`).
    ///
    /// # Errors
    /// As [`StepCompressor::append`].
    pub fn flush(&mut self, move_clock: u64) -> Result<(), StepCompressError> {
        if self.next_step_clock != 0 && move_clock >= self.next_step_clock {
            self.queue_append()?;
        }
        self.queue_flush(move_clock)
    }

    /// Take the commands produced since the last call.
    pub fn take_commands(&mut self) -> Vec<StepCommand> {
        std::mem::take(&mut self.out)
    }

    /// The print time of the last scheduled step
    /// (`calc_last_step_print_time`).
    fn calc_last_step_print_time(&mut self) {
        let last = self.last_step_clock as f64;
        self.last_step_print_time = self.mcu_time_offset + (last - 0.5) / self.mcu_freq;
    }

    /// The acceptable clock range for the queued step at `index`
    /// (`minmax_point`).
    fn minmax_point(&self, index: usize) -> Points {
        let lsc = self.last_step_clock as u32;
        let point = self.queue[index].wrapping_sub(lsc);
        let prevpoint = if index > self.queue_pos {
            self.queue[index - 1].wrapping_sub(lsc)
        } else {
            0
        };
        let mut max_error = point.wrapping_sub(prevpoint) / 2;
        if max_error > self.max_error {
            max_error = self.max_error;
        }
        Points {
            minp: i64::from(point - max_error),
            maxp: i64::from(point),
        }
    }

    /// Find a `step_move` that covers a series of step times
    /// (`compress_bisect_add`).
    fn compress_bisect_add(&self) -> StepMove {
        let qlast = (self.queue_pos + 65535).min(self.queue_next);
        let point = self.minmax_point(self.queue_pos);
        let mut outer_mininterval = point.minp;
        let mut outer_maxinterval = point.maxp;
        let mut add: i64 = 0;
        let mut minadd: i64 = -0x8000;
        let mut maxadd: i64 = 0x7fff;
        let mut bestinterval: i64 = 0;
        let mut bestcount: i64 = 1;
        let mut bestadd: i64 = 1;
        let mut bestreach: i64 = i64::from(i32::MIN);
        let mut zerointerval: i64 = 0;
        let mut zerocount: i64 = 0;
        // Assigned by the inner loop before it can break; declared uninitialised
        // to mirror upstream's `struct points nextpoint;`.
        let mut nextpoint: Points;

        loop {
            // Find the longest valid sequence with the given `add`.
            let mut nextmininterval = outer_mininterval;
            let mut nextmaxinterval = outer_maxinterval;
            let mut interval = nextmaxinterval;
            let mut nextcount: i64 = 1;
            loop {
                nextcount += 1;
                if self.queue_pos + (nextcount as usize) > qlast {
                    let count = nextcount - 1;
                    return StepMove {
                        interval: interval as u32,
                        count: count as u16,
                        add: add as i16,
                    };
                }
                nextpoint = self.minmax_point(self.queue_pos + (nextcount as usize) - 1);
                let nextaddfactor = nextcount * (nextcount - 1) / 2;
                let c = add * nextaddfactor;
                if nextmininterval * nextcount < nextpoint.minp - c {
                    nextmininterval = idiv_up(nextpoint.minp - c, nextcount);
                }
                if nextmaxinterval * nextcount > nextpoint.maxp - c {
                    nextmaxinterval = idiv_down(nextpoint.maxp - c, nextcount);
                }
                if nextmininterval > nextmaxinterval {
                    break;
                }
                interval = nextmaxinterval;
            }

            // Check whether this is the best sequence found so far.
            let count = nextcount - 1;
            let addfactor = count * (count - 1) / 2;
            let reach = add * addfactor + interval * count;
            if reach > bestreach || (reach == bestreach && interval > bestinterval) {
                bestinterval = interval;
                bestcount = count;
                bestadd = add;
                bestreach = reach;
                if add == 0 {
                    zerointerval = interval;
                    zerocount = count;
                }
                if count > 0x200 {
                    // No `add` will improve the sequence; avoid overflow.
                    break;
                }
            }

            // Check whether a greater or lesser `add` could extend it.
            let nextaddfactor = nextcount * (nextcount - 1) / 2;
            let nextreach = add * nextaddfactor + interval * nextcount;
            if nextreach < nextpoint.minp {
                minadd = add + 1;
                outer_maxinterval = nextmaxinterval;
            } else {
                maxadd = add - 1;
                outer_mininterval = nextmininterval;
            }

            // The maximum valid deviation between two quadratic sequences.
            if count > 1 {
                let errdelta = i64::from(self.max_error) * QUADRATIC_DEV / (count * count);
                if minadd < add - errdelta {
                    minadd = add - errdelta;
                }
                if maxadd > add + errdelta {
                    maxadd = add + errdelta;
                }
            }

            // See whether the next point would further limit the `add` range.
            let c = outer_maxinterval * nextcount;
            if minadd * nextaddfactor < nextpoint.minp - c {
                minadd = idiv_up(nextpoint.minp - c, nextaddfactor);
            }
            let c = outer_mininterval * nextcount;
            if maxadd * nextaddfactor > nextpoint.maxp - c {
                maxadd = idiv_down(nextpoint.maxp - c, nextaddfactor);
            }

            // Bisect the valid `add` range and try again.
            if minadd > maxadd {
                break;
            }
            add = maxadd - (maxadd - minadd) / 4;
        }

        if zerocount + zerocount / 16 >= bestcount {
            // Prefer `add = 0` when it is similar to the best found sequence.
            return StepMove {
                interval: zerointerval as u32,
                count: zerocount as u16,
                add: 0,
            };
        }
        StepMove {
            interval: bestinterval as u32,
            count: bestcount as u16,
            add: bestadd as i16,
        }
    }

    /// Verify that a `step_move` matches the actual step times (`check_line`).
    fn check_line(&self, move_: StepMove) -> Result<(), StepCompressError> {
        if move_.count == 0
            || (move_.interval == 0 && move_.add == 0 && move_.count > 1)
            || move_.interval >= 0x8000_0000
        {
            return Err(self.invalid(&move_, "Invalid sequence"));
        }
        let mut interval = move_.interval;
        let mut p: u32 = 0;
        for i in 0..move_.count as usize {
            let point = self.minmax_point(self.queue_pos + i);
            p = p.wrapping_add(interval);
            if p < point.minp as u32 || p > point.maxp as u32 {
                return Err(self.invalid(
                    &move_,
                    &format!("Point {}: {p} not in {}:{}", i + 1, point.minp, point.maxp),
                ));
            }
            if interval >= 0x8000_0000 {
                return Err(self.invalid(&move_, &format!("Point {}: interval overflow", i + 1)));
            }
            interval = interval.wrapping_add(move_.add as u32);
        }
        Ok(())
    }

    fn invalid(&self, move_: &StepMove, detail: &str) -> StepCompressError {
        StepCompressError::new(format!(
            "stepcompress o={} i={} c={} a={}: {detail}",
            self.oid, move_.interval, move_.count, move_.add
        ))
    }

    /// Queue one `step_move` (`add_move`).
    fn add_move(&mut self, first_clock: u64, move_: StepMove) {
        let addfactor = i64::from(move_.count) * (i64::from(move_.count) - 1) / 2;
        let ticks = (i64::from(move_.add) * addfactor
            + i64::from(move_.interval) * (i64::from(move_.count) - 1)) as u32;
        let last_clock = first_clock + u64::from(ticks);
        self.out.push(StepCommand::QueueStep {
            oid: self.oid,
            interval: move_.interval,
            count: u32::from(move_.count),
            add: i32::from(move_.add),
        });
        // Record the run so a past position can be found later.
        let step_count = if self.sdir == 1 {
            i64::from(move_.count)
        } else {
            -i64::from(move_.count)
        };
        self.history.push_front(HistoryStep {
            first_clock,
            last_clock,
            start_position: self.last_position,
            step_count,
            interval: move_.interval,
            add: i32::from(move_.add),
        });
        self.last_position += step_count;
        self.last_step_clock = last_clock;
    }

    /// Compress the queued steps up to `move_clock` (`queue_flush`).
    fn queue_flush(&mut self, move_clock: u64) -> Result<(), StepCompressError> {
        if self.queue_pos >= self.queue_next {
            return Ok(());
        }
        while self.last_step_clock < move_clock {
            let move_ = self.compress_bisect_add();
            self.check_line(move_)?;
            self.add_move(self.last_step_clock + u64::from(move_.interval), move_);
            if self.queue_pos + move_.count as usize >= self.queue_next {
                self.queue.clear();
                self.queue_pos = 0;
                self.queue_next = 0;
                break;
            }
            self.queue_pos += move_.count as usize;
        }
        self.calc_last_step_print_time();
        Ok(())
    }

    /// Emit a one-step command for a step far from the last one
    /// (`stepcompress_flush_far`).
    fn flush_far(&mut self, abs_step_clock: u64) {
        let move_ = StepMove {
            interval: abs_step_clock.wrapping_sub(self.last_step_clock) as u32,
            count: 1,
            add: 0,
        };
        self.add_move(abs_step_clock, move_);
        self.calc_last_step_print_time();
    }

    /// Slow path for a step far in the future (`queue_append_far`).
    fn queue_append_far(&mut self) -> Result<(), StepCompressError> {
        let step_clock = self.next_step_clock;
        self.next_step_clock = 0;
        self.queue_flush(step_clock.wrapping_sub(CLOCK_DIFF_MAX).wrapping_add(1))?;
        if step_clock >= self.last_step_clock + CLOCK_DIFF_MAX {
            self.flush_far(step_clock);
            return Ok(());
        }
        self.push_step(step_clock);
        Ok(())
    }

    /// Store one step clock, compacting the queue as needed
    /// (`queue_append_extend` plus the storage half of `queue_append`).
    fn push_step(&mut self, step_clock: u64) {
        if self.queue_next - self.queue_pos > 65535 + 2000 {
            // No point in keeping more than 64K steps in memory.
            let index = self.queue_next - 65535;
            let flush = u64::from(self.queue[index].wrapping_sub(self.last_step_clock as u32));
            // The flush below cannot fail on an empty pending step; it only
            // compresses what is already queued.
            let _ = self.queue_flush(self.last_step_clock + flush);
        }
        if self.queue_pos > 65536 {
            self.queue.drain(..self.queue_pos);
            self.queue_next -= self.queue_pos;
            self.queue_pos = 0;
        }
        self.queue.push_back(step_clock as u32);
        self.queue_next += 1;
    }

    /// Add a step time to the queue (`queue_append`).
    fn queue_append(&mut self) -> Result<(), StepCompressError> {
        if self.next_step_dir as i8 != self.sdir {
            self.set_next_step_dir(self.next_step_dir)?;
        }
        if self.next_step_clock >= self.last_step_clock + CLOCK_DIFF_MAX {
            return self.queue_append_far();
        }
        let step_clock = self.next_step_clock;
        self.next_step_clock = 0;
        self.push_step(step_clock);
        Ok(())
    }

    /// Send the `set_next_step_dir` command (`set_next_step_dir`).
    fn set_next_step_dir(&mut self, sdir: bool) -> Result<(), StepCompressError> {
        let sdir = i8::from(sdir);
        if self.sdir == sdir {
            return Ok(());
        }
        self.queue_flush(u64::MAX)?;
        self.sdir = sdir;
        self.out.push(StepCommand::SetNextStepDir {
            oid: self.oid,
            direction: sdir == 1,
        });
        Ok(())
    }
}

/// How the simplified compressor announced itself.
///
/// The compressor is full now, so nothing calls this; it stays because the
/// policy is part of the FW5 API and tests exercise it, and because a future
/// fallback would use it again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GracePolicy {
    /// How many warning lines to print.
    pub warnings: u32,
    /// The pause between them.
    pub interval: std::time::Duration,
    /// The pause after the last one, before motion may start.
    pub grace: std::time::Duration,
}

impl GracePolicy {
    /// Three warnings a second apart, then five seconds to abort.
    pub const DEFAULT: Self = Self {
        warnings: 3,
        interval: std::time::Duration::from_secs(1),
        grace: std::time::Duration::from_secs(5),
    };

    /// No warnings and no wait, for tests.
    pub const QUIET: Self = Self {
        warnings: 0,
        interval: std::time::Duration::ZERO,
        grace: std::time::Duration::ZERO,
    };
}

/// Set once the simplified compressor has announced itself.
static ANNOUNCED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Announce a fallback compressor and give the operator a window to abort.
///
/// The full compressor does not need this; the function is kept for the policy
/// API and its tests (see [`GracePolicy`]).
pub async fn warn_and_wait(policy: GracePolicy) -> bool {
    warn_and_wait_once(policy, &ANNOUNCED).await
}

/// [`warn_and_wait`] with an injectable latch, so tests do not share state.
async fn warn_and_wait_once(
    policy: GracePolicy,
    announced: &std::sync::atomic::AtomicBool,
) -> bool {
    use std::sync::atomic::Ordering;
    if announced.swap(true, Ordering::SeqCst) {
        return false;
    }
    const MESSAGE: &str = "stepcompress fallback: one queue_step per step, which will overflow \
                           the MCU move queue on a real print";
    for _ in 0..policy.warnings {
        tracing::warn!("{MESSAGE}");
        if !policy.interval.is_zero() {
            tokio::time::sleep(policy.interval).await;
        }
    }
    if !policy.grace.is_zero() {
        tracing::warn!(
            "stepcompress: starting motion in {:.1}s; interrupt now (Ctrl-C or M112) to abort",
            policy.grace.as_secs_f64()
        );
        tokio::time::sleep(policy.grace).await;
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

    /// The `queue_step` commands, in order, as `(interval, count, add)`.
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

    /// Feed raw step clocks through the real `append` path.
    ///
    /// At `mcu_freq = 1` with `set_time(0.5, 1.0)`, `last_step_print_time` is
    /// exactly `0`, so `append(sdir, 0.0, clock)` queues exactly `clock`. This is
    /// the same trick the upstream vector harness uses, so the two see identical
    /// step clocks.
    fn append_clocks(sc: &mut StepCompressor, sdir: bool, clocks: &[u64]) {
        for clock in clocks {
            sc.append(sdir, 0.0, *clock as f64).unwrap();
        }
        sc.flush(u64::MAX).unwrap();
    }

    fn raw_compressor() -> StepCompressor {
        let mut sc = StepCompressor::new(7, 1.0);
        sc.set_time(0.5, 1.0);
        // The upstream harness uses `max_error = 40`; `new` would derive 0 from a
        // 1 Hz frequency, which is a degenerate compressor.
        sc.set_max_error(40);
        sc
    }

    #[test]
    fn test_a_constant_interval_is_one_move() {
        let mut sc = raw_compressor();
        let clocks: Vec<u64> = (1..=10).map(|i| i * 1000).collect();

        append_clocks(&mut sc, true, &clocks);

        // Generated by the upstream harness (`CASE constant_positive`).
        assert_eq!(sc.take_commands(), {
            let mut expected = vec![StepCommand::SetNextStepDir {
                oid: 7,
                direction: true,
            }];
            expected.push(StepCommand::QueueStep {
                oid: 7,
                interval: 1000,
                count: 10,
                add: 0,
            });
            expected
        });
    }

    #[test]
    fn test_an_accelerating_run_matches_upstream() {
        let mut sc = raw_compressor();
        let mut clocks = Vec::new();
        let mut t = 0i64;
        let mut gap = 3000i64;
        for _ in 0..12 {
            t += gap;
            clocks.push(t as u64);
            gap -= 200;
        }

        append_clocks(&mut sc, true, &clocks);

        // Upstream harness `CASE accelerate`: add = -198.
        assert_eq!(steps(&sc.take_commands()), vec![(2989, 12, -198)]);
    }

    #[test]
    fn test_a_decelerating_run_matches_upstream() {
        let mut sc = raw_compressor();
        let mut clocks = Vec::new();
        let mut t = 0i64;
        let mut gap = 1000i64;
        for _ in 0..12 {
            t += gap;
            clocks.push(t as u64);
            gap += 250;
        }

        append_clocks(&mut sc, true, &clocks);

        // Upstream harness `CASE decelerate`: add = 252.
        assert_eq!(steps(&sc.take_commands()), vec![(989, 12, 252)]);
    }

    #[test]
    fn test_a_quadratic_run_matches_upstream() {
        let mut sc = raw_compressor();
        let mut clocks = Vec::new();
        let mut t = 0i64;
        let mut gap = 2000i64;
        for _ in 0..20 {
            t += gap;
            clocks.push(t as u64);
            gap += 50;
        }

        append_clocks(&mut sc, true, &clocks);

        // Upstream harness `CASE quadratic`.
        assert_eq!(steps(&sc.take_commands()), vec![(2000, 20, 50)]);
    }

    #[test]
    fn test_a_direction_change_emits_a_dir_command_between_moves() {
        let mut sc = raw_compressor();
        let mut commands = Vec::new();

        let up: Vec<u64> = (1..=8).map(|i| i * 1000).collect();
        append_clocks(&mut sc, true, &up);
        commands.extend(sc.take_commands());
        let down: Vec<u64> = (9..=16).map(|i| i * 1000).collect();
        append_clocks(&mut sc, false, &down);
        commands.extend(sc.take_commands());

        // Upstream harness `CASE direction_change`.
        let dirs: Vec<bool> = commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::SetNextStepDir { direction, .. } => Some(*direction),
                StepCommand::QueueStep { .. } => None,
            })
            .collect();
        assert_eq!(dirs, [true, false], "{commands:?}");
        assert_eq!(steps(&commands), vec![(1000, 8, 0), (1000, 8, 0)]);
    }

    #[test]
    fn test_a_far_gap_becomes_a_one_step_reanchor() {
        let mut sc = raw_compressor();
        append_clocks(&mut sc, true, &[900_000_000, 900_001_000, 900_002_000]);

        // Upstream harness `CASE far_gap`.
        assert_eq!(
            steps(&sc.take_commands()),
            vec![(900_000_000, 1, 0), (1000, 2, 0)]
        );
    }

    #[test]
    fn test_a_single_step_matches_upstream() {
        let mut sc = raw_compressor();
        append_clocks(&mut sc, true, &[1234]);

        // Upstream harness `CASE single`.
        assert_eq!(steps(&sc.take_commands()), vec![(1234, 1, 0)]);
    }

    #[test]
    fn test_the_sds_filter_drops_a_step_before_a_reversal() {
        // At 1 MHz, SDS_FILTER_TIME is 750 ticks. The three step times are fed
        // through the real `append` path, as the upstream harness does.
        let mut sc = StepCompressor::new(7, 1_000_000.0);
        sc.set_max_error(40);
        sc.set_time(0.0, 1_000_000.0);
        sc.append(true, 0.0, 0.001).unwrap();
        sc.append(true, 0.0, 0.002).unwrap();
        // 50 µs after the last step: inside the filter, so both the 0.002 step
        // and the reversal are dropped in favour of the earlier step.
        sc.append(false, 0.0, 0.002_05).unwrap();
        sc.commit().unwrap();
        sc.flush(u64::MAX).unwrap();

        // Upstream harness `CASE sds_filter_mhz`: only the first step survives.
        let commands = sc.take_commands();
        let dirs: Vec<bool> = commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::SetNextStepDir { direction, .. } => Some(*direction),
                StepCommand::QueueStep { .. } => None,
            })
            .collect();
        assert_eq!(dirs, [true], "{commands:?}");
        assert_eq!(steps(&commands), vec![(1000, 1, 0)]);
    }

    #[test]
    fn test_flush_waits_for_the_move_clock() {
        let mut sc = compressor();
        sc.append(true, 0.0, 0.005).unwrap();

        // Not yet: the pending step is at 5000 ticks.
        sc.flush(4999).unwrap();
        assert!(sc.take_commands().is_empty());

        sc.flush(5000).unwrap();
        assert_eq!(steps(&sc.take_commands()).len(), 1);
    }

    #[test]
    fn test_set_last_position_flushes_and_records() {
        let mut sc = raw_compressor();
        sc.append(true, 0.0, 500.0).unwrap();

        sc.set_last_position(10_000, 3).unwrap();

        assert_eq!(sc.last_position(), 3);
        // The pending step was flushed by the call.
        assert_eq!(steps(&sc.take_commands()), vec![(500, 1, 0)]);
    }

    #[test]
    fn test_find_past_position_matches_the_emitted_schedule() {
        // An accelerating run; walk the emitted `queue_step` schedule and check
        // the history's position lookup at every step clock.
        let mut sc = raw_compressor();
        let mut clocks = Vec::new();
        let mut t = 0i64;
        let mut gap = 3000i64;
        for _ in 0..12 {
            t += gap;
            clocks.push(t as u64);
            gap -= 200;
        }
        append_clocks(&mut sc, true, &clocks);
        let commands = sc.take_commands();

        let mut schedule = Vec::new();
        let mut clock: i64 = 0;
        let mut position: i64 = 0;
        for command in &commands {
            if let StepCommand::QueueStep {
                interval,
                count,
                add,
                ..
            } = command
            {
                let mut interval = *interval as i64;
                for _ in 0..*count {
                    clock += interval;
                    position += 1;
                    schedule.push((clock, position));
                    interval += *add as i64;
                }
            }
        }
        assert_eq!(schedule.len(), clocks.len());
        assert_eq!(sc.find_past_position(0), 0);
        for (clock, position) in &schedule {
            assert_eq!(
                sc.find_past_position(*clock as u64),
                *position,
                "clock {clock}"
            );
        }
        assert_eq!(sc.find_past_position(u64::MAX), 12);
    }

    #[test]
    fn test_set_last_position_records_a_history_marker() {
        let mut sc = raw_compressor();
        append_clocks(&mut sc, true, &[1000, 2000, 3000]);
        sc.take_commands();

        sc.set_last_position(50_000, 7).unwrap();

        assert_eq!(sc.find_past_position(50_000), 7);
        // Before the marker the older run's end still stands (upstream walks the
        // history newest-first and an older run containing the clock wins).
        assert_eq!(sc.find_past_position(49_999), 3);
        // A later run continues from the marker's position.
        append_clocks(&mut sc, true, &[51_000, 52_000, 53_000]);
        assert_eq!(sc.find_past_position(51_000), 8);
        assert_eq!(sc.find_past_position(53_000), 10);
    }

    #[test]
    fn test_extract_old_and_history_expire() {
        let mut sc = raw_compressor();
        append_clocks(&mut sc, true, &[1000, 2000, 3000]);

        let old = sc.extract_old(10, 0, u64::MAX);
        assert_eq!(old.len(), 1);
        assert_eq!(old[0].step_count, 3);
        assert_eq!(old[0].first_clock, 1000);
        assert_eq!(old[0].last_clock, 3000);

        // Nothing ended at or before 500, so the run survives an expiry there.
        sc.history_expire(500);
        assert_eq!(sc.extract_old(10, 0, u64::MAX).len(), 1);
        sc.history_expire(3000);
        assert!(sc.extract_old(10, 0, u64::MAX).is_empty());
    }

    #[test]
    fn test_a_jittery_run_matches_upstream() {
        // A 40-step sequence whose gaps vary pseudo-randomly, which exercises the
        // `add` bisection much more than the smooth cases.
        let mut sc = raw_compressor();
        let mut clocks = Vec::new();
        let mut t: i64 = 0;
        for i in 0..40i64 {
            t += 1000 + (i * i * 7) % 500;
            clocks.push(t as u64);
        }

        append_clocks(&mut sc, true, &clocks);

        // Upstream harness `CASE jitter` (the u32 à i32 conversions included).
        assert_eq!(
            steps(&sc.take_commands()),
            vec![
                (964, 6, 39),
                (1228, 3, 125),
                (1040, 3, 164),
                (983, 3, 205),
                (1051, 2, 266),
                (998, 2, 295),
                (1002, 2, 323),
                (1062, 2, 351),
                (1178, 2, -121),
                (1348, 3, -112),
                (1456, 6, -71),
                (1076, 6, 7),
            ]
        );
    }

    #[test]
    fn test_print_time_to_clock_follows_the_offset() {
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        // The primary MCU: print-time zero is clock zero.
        sc.set_time(0.0, 1_000_000.0);
        assert_eq!(sc.print_time_to_clock(2.0), 2_000_000);

        // A secondary MCU whose clock zero is at print time 1.5.
        sc.set_time(1.5, 1_000_000.0);
        assert_eq!(sc.time_offset(), 1.5);
        assert_eq!(sc.print_time_to_clock(2.0), 500_000);
        // Before the offset there is no clock: the bound clamps to zero.
        assert_eq!(sc.print_time_to_clock(1.0), 0);
    }

    #[test]
    fn test_a_compressed_run_reconstructs_every_requested_step() {
        // The property `check_line` enforces: whatever the compressor emits, the
        // requested clocks must fall inside the schedule's error window.
        let mut sc = raw_compressor();
        let clocks: Vec<u64> = (1..=200u64).map(|i| i * 251).collect();
        append_clocks(&mut sc, true, &clocks);
        let commands = sc.take_commands();

        // Reconstruct the step clocks the commands describe.
        let mut clock = 0u64;
        let mut reconstructed = Vec::new();
        for command in &commands {
            if let StepCommand::QueueStep {
                interval,
                count,
                add,
                ..
            } = command
            {
                let mut interval = *interval;
                for _ in 0..*count {
                    clock += u64::from(interval);
                    reconstructed.push(clock);
                    interval = (i64::from(interval) + i64::from(*add)) as u32;
                }
            }
        }
        assert_eq!(reconstructed.len(), clocks.len());
        // Every requested clock is within `max_error` of the schedule.
        for (requested, actual) in clocks.iter().zip(&reconstructed) {
            let diff = (*requested as i64 - *actual as i64).abs();
            assert!(diff <= 25, "{requested} vs {actual}: {diff}");
        }
        // And the compression is drastic: 200 steps in a handful of commands.
        assert!(steps(&commands).len() <= 8, "{:?}", steps(&commands));
    }

    #[tokio::test]
    async fn test_the_announcement_runs_once() {
        let announced = std::sync::atomic::AtomicBool::new(false);

        assert!(warn_and_wait_once(GracePolicy::QUIET, &announced).await);
        assert!(!warn_and_wait_once(GracePolicy::QUIET, &announced).await);
    }

    #[tokio::test]
    async fn test_a_quiet_policy_does_not_wait() {
        let announced = std::sync::atomic::AtomicBool::new(false);
        let started = std::time::Instant::now();

        warn_and_wait_once(GracePolicy::QUIET, &announced).await;

        assert!(started.elapsed() < std::time::Duration::from_millis(50));
    }
}
