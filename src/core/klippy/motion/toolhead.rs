//! The toolhead: where moves are planned and print time is tracked.
//!
//! Upstream's `klippy/toolhead.py` `ToolHead`: it holds the look-ahead queue,
//! the commanded position and the print time, and feeds the motion queue. The
//! kinematics — which positions are legal, how to home — is FW5e; this is the
//! skeleton a `G1` drives.

use std::sync::Arc;

use super::extra::ExtraAxis;
use super::kinematics::{Kinematics, MoveContext};
use super::plan::{LookAheadQueue, Move, MoveLimits};
use super::queuing::MotionQueuing;
use super::stepcompress::{StepCommand, StepCompressError};
use super::stepper::Stepper;
use super::trapq::Trapq;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, Xyz, E_AXIS};

/// How far ahead of the MCU the planner starts, in seconds
/// (`BUFFER_TIME_START`, `klippy/toolhead.py:196`).
pub const BUFFER_TIME_START: f64 = 0.250;

/// The minimum lead a planned move keeps ahead of the estimated clock, in
/// seconds (`MIN_KIN_TIME`, `klippy/extras/motion_queuing.py:16`).
///
/// It is the `est` half of `calc_step_gen_restart`: generation restarts no
/// sooner than this after the estimate, so a move queued at the floor still
/// has time to be generated and reach the wire before its own clock.
///
/// **In this host the term is dominated** — `BUFFER_TIME_START` (0.250) is
/// already larger than `MIN_KIN_TIME + SDS_CHECK_TIME` (0.101) — so folding it
/// into the floor is an alignment with upstream's formula, not a behavioural
/// change on its own (it only ever wins when `last_step_gen_time` is further
/// ahead than `est + 0.101`); a raised `kin_flush_delay` narrows that.
const MIN_KIN_TIME: f64 = 0.100;

/// The step+dir+step filter window, in seconds (`SDS_CHECK_TIME`,
/// `klippy/extras/motion_queuing.py:17`, `chelper/stepcompress.c:504`).
///
/// It is the floor of [`ToolHead::kin_flush_delay`], which upstream starts here
/// and only raises for a stepper with a wider step generation window
/// (`motion_queuing.py:123-138`): an extruder's half smooth time
/// (`kin_extruder.c:144-145`) or an input shaper's pulse span
/// (`kin_shaper.c:202-218`). `recompute_scan_windows` runs the same scan over
/// this toolhead's steppers; with no stepper, or none with a window, the filter
/// alone is in force.
const SDS_CHECK_TIME: f64 = 0.001;

/// Where the toolhead reads the MCU's estimated print time from
/// (`MCU.estimated_print_time`).
///
/// Upstream asks its `self.mcu` every time it floors the print time
/// (`_calc_print_time`, `klippy/toolhead.py:260-268`), so the floor follows the
/// clock. This layer has no MCU, so the object that has one injects a getter at
/// connect — a **value** rather than a snapshot: a connect-time reading goes
/// stale by exactly the idle time since (C5 measured it: a machine idle 33 s
/// planned its next motion 33 s in the past, so every step batch expired at
/// once and the boards got the whole motion in one dump).
#[derive(Clone)]
pub struct EstimatedPrintTime(Arc<dyn Fn() -> f64 + Send + Sync>);

impl std::fmt::Debug for EstimatedPrintTime {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("EstimatedPrintTime(live)")
    }
}

impl Default for EstimatedPrintTime {
    /// A source that always reads `0.0` — the toolhead before anything
    /// injected one (a test, or a machine that has not connected yet), for
    /// which the floor is the buffer alone.
    fn default() -> Self {
        Self(Arc::new(|| 0.0))
    }
}

impl EstimatedPrintTime {
    /// A source reading through `source`, called fresh on every floor.
    pub fn new(source: impl Fn() -> f64 + Send + Sync + 'static) -> Self {
        Self(Arc::new(source))
    }

    /// The estimate right now.
    pub fn get(&self) -> f64 {
        (self.0)()
    }
}

/// Notified when [`ToolHead::calc_print_time`] raises the print time, with the
/// estimate and the new print time (`_calc_print_time`'s
/// `send_event("toolhead:sync_print_time", curtime, est_print_time,
/// self.print_time)`, `klippy/toolhead.py:264-268`).
///
/// Upstream sends the event right there; this layer only calls a hook, because
/// every path into `calc_print_time` runs under the connected state lock
/// (`ToolHeadObject`) and a handler that reaches back into the planner would
/// deadlock on it. The object that owns the printer injects a recording hook
/// at connect and sends the event once that lock is released — the same shape
/// as [`EstimatedPrintTime`]: a value handed in, not a clock this layer owns
/// (the hook dates the event, `toolhead.py:261`).
#[derive(Clone)]
pub struct SyncPrintTimeNotify(Arc<dyn Fn(f64, f64) + Send + Sync>);

impl std::fmt::Debug for SyncPrintTimeNotify {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("SyncPrintTimeNotify")
    }
}

impl Default for SyncPrintTimeNotify {
    /// A hook that drops the notification — the toolhead before anything
    /// injected one (a test, or a machine that has not connected yet).
    fn default() -> Self {
        Self(Arc::new(|_, _| {}))
    }
}

impl SyncPrintTimeNotify {
    /// A hook calling `notify(est_print_time, print_time)` on every raise.
    pub fn new(notify: impl Fn(f64, f64) + Send + Sync + 'static) -> Self {
        Self(Arc::new(notify))
    }

    /// Report one raise.
    fn notify(&self, est_print_time: f64, print_time: f64) {
        (self.0)(est_print_time, print_time)
    }
}

/// One iteration of the wait `M400` drives on (`ToolHead.wait_moves`,
/// `klippy/toolhead.py:422-428`), as a single snapshot: the caller holds the
/// planner for one short read and then sleeps with it released.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaitMovesState {
    /// Upstream's loop condition — whether there is still something to wait
    /// for: `not special_queuing_state or print_time >=
    /// estimated_print_time(eventtime)`.
    pub waiting: bool,
    /// The planner horizon the estimate was judged against
    /// ([`ToolHead::print_time`]).
    pub print_time: f64,
    /// The MCU's estimate at the instant of this read, never cached
    /// ([`ToolHead::estimated_print_time`]).
    pub estimated_print_time: f64,
}

/// A lookahead callback parked on the move it was registered against: the
/// index of that move in the batch that will flush it
/// (`ToolHead::register_lookahead_callback`).
struct ParkedLookahead {
    /// Where the target move sits in the batch that flushes it (0-based).
    index: usize,
    callback: Box<dyn FnOnce(f64) + Send>,
}

impl std::fmt::Debug for ParkedLookahead {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParkedLookahead")
            .field("index", &self.index)
            .finish_non_exhaustive()
    }
}

/// The toolhead: commanded position, print time, and the motion queue.
#[derive(Debug)]
pub struct ToolHead {
    limits: MoveLimits,
    lookahead: LookAheadQueue,
    commanded_pos: Coord,
    print_time: f64,
    /// The live source of the MCU's estimated print time (see
    /// [`EstimatedPrintTime`]): read on every prime, never cached.
    estimated_print_time: EstimatedPrintTime,
    /// Where a raised print time is reported (see [`SyncPrintTimeNotify`]):
    /// upstream's `toolhead:sync_print_time`, which `calc_print_time` cannot
    /// send from under the state lock.
    sync_print_time: SyncPrintTimeNotify,
    /// The step-generation horizon the planner has actually reached: the
    /// `step_gen_time` of the last successful `flush_step_generation`.
    /// Upstream's `motion_queuing.last_step_gen_time`
    /// (`motion_queuing.py:145-146`), the `kin_time` half of
    /// `_calc_print_time`'s floor (`toolhead.py:263` → `motion_queuing.py:191`).
    last_step_gen_time: f64,
    /// The widest step generation window among the steppers, in seconds —
    /// upstream's `motion_queuing.kin_flush_delay` (`motion_queuing.py:126-138`),
    /// kept in step by [`Self::recompute_scan_windows`].
    ///
    /// It is added to the print-time floor (`calc_step_gen_restart`,
    /// `motion_queuing.py:190-192`) and subtracted from the trapq free time
    /// before history is expired (`_advance_flush_time`,
    /// `motion_queuing.py:152,155`): steps inside it may still be rewritten by
    /// a change to the smoothing, so nothing inside it can be treated as
    /// finished.
    kin_flush_delay: f64,
    special_queuing_state: bool,
    /// How many moves sit in `lookahead`: one is added with every `move_to`
    /// and leaves with the batch `process_lookahead` flushes. It is the queue
    /// order a [`Self::register_lookahead_callback`] callback was parked at.
    lookahead_depth: usize,
    /// Lookahead callbacks registered while their move was still queued, each
    /// at the index that move will flush at (`register_lookahead_callback`).
    parked_lookahead: Vec<ParkedLookahead>,
    motion_queuing: MotionQueuing,
    kinematics: Option<Box<dyn Kinematics>>,
    /// The trapq the kinematic move is appended to; each extra axis has its
    /// own (`MotionQueuing::allocate_trapq`).
    main_trapq: usize,
    /// The non-kinematic axes (extruders), on position index `E_AXIS` and up.
    extra_axes: Vec<Arc<dyn ExtraAxis>>,
    /// Where the extra axes **past [`E_AXIS`]** are, index 0 == the first one
    /// after the four modelled axes.
    ///
    /// Upstream keeps these in the tail of `commanded_pos`, which grows a slot
    /// per `add_extra_axis` (`toolhead.py:378-385`) and is what a `Move` reads
    /// its start and end from (`:20-24`). This port's [`Coord`] is four axes,
    /// so the tail lives here and travels into `Move::extra_start`/
    /// `extra_end`.
    extra_pos: Vec<f64>,
}

impl ToolHead {
    /// A toolhead with `limits`.
    pub fn new(limits: MoveLimits) -> Self {
        let mut motion_queuing = MotionQueuing::new();
        let main_trapq = motion_queuing.allocate_trapq();
        Self {
            limits,
            lookahead: LookAheadQueue::new(),
            commanded_pos: Coord::default(),
            print_time: 0.0,
            estimated_print_time: EstimatedPrintTime::default(),
            sync_print_time: SyncPrintTimeNotify::default(),
            last_step_gen_time: 0.0,
            kin_flush_delay: SDS_CHECK_TIME,
            // Upstream starts in "NeedPrime" and resyncs the print time on the
            // first planned move (`klippy/toolhead.py:227`).
            special_queuing_state: true,
            lookahead_depth: 0,
            parked_lookahead: Vec::new(),
            motion_queuing,
            kinematics: None,
            main_trapq,
            extra_axes: Vec::new(),
            extra_pos: Vec::new(),
        }
    }

    /// Add a stepper to drive. It reads the main trapq.
    pub fn add_stepper(&mut self, mut stepper: Stepper) {
        stepper.set_trapq(self.main_trapq);
        self.motion_queuing.add_stepper(stepper);
    }

    /// Add a stepper that drives its own trapq (an extra axis' motor) without
    /// re-pointing it at the main one — the half of
    /// [`ToolHeadObject::add_extra_axis_stepper`](crate::core::klippy::extras::toolhead::ToolHeadObject::add_extra_axis_stepper)
    /// that runs with the connection lock already held.
    ///
    /// [`ToolHead::add_stepper`] is for the kinematic motors, which all read the
    /// main trapq; an extra axis' stepper was already pointed at the trapq its
    /// axis queues into.
    pub fn add_extra_axis_stepper(&mut self, stepper: Stepper) {
        self.motion_queuing.add_stepper(stepper);
    }

    /// Add a non-kinematic axis (the extruder).
    ///
    /// The caller owns the axis' trapq and has already pointed its stepper at
    /// it (`PrinterExtruder.stepper.set_trapq`).
    ///
    /// Upstream flushes the look-ahead first (`toolhead.py:442`: a queued move
    /// has as many axis slots as the toolhead had then) and appends the axis'
    /// position to `commanded_pos`. Here the position of the **first** extra
    /// axis is the `e` of the toolhead's [`Coord`] (it is the extruder slot,
    /// position [`E_AXIS`]); every later one gets a tail slot in
    /// [`Self::extra_pos`], seeded the way upstream's `axis_pos` argument does.
    pub fn add_extra_axis(&mut self, axis: Arc<dyn ExtraAxis>) {
        self.process_lookahead();
        let position = axis.find_past_position(self.print_time);
        self.extra_axes.push(axis);
        if self.extra_axes.len() > 1 {
            self.extra_pos.push(position);
        }
    }

    /// The non-kinematic axes.
    pub fn extra_axes(&self) -> &[Arc<dyn ExtraAxis>] {
        &self.extra_axes
    }

    /// Remove a non-kinematic axis by identity (`ToolHead.remove_extra_axis`).
    ///
    /// A `GCODE_AXIS=` request takes the manual stepper back off the axis list;
    /// upstream compares by object identity (`ea not in self.extra_axes`), so
    /// this matches on the shared allocation. Like [`Self::add_extra_axis`], it
    /// flushes the look-ahead first and drops the axis' position slot
    /// (`toolhead.py:446-452`).
    /// [`Self::extra_pos`] holds one position per axis **past** [`E_AXIS`], so
    /// the first extra axis (the extruder) has none.
    pub fn remove_extra_axis(&mut self, axis: &Arc<dyn ExtraAxis>) {
        self.process_lookahead();
        if let Some(index) = self
            .extra_axes
            .iter()
            .position(|candidate| Arc::ptr_eq(candidate, axis))
        {
            self.extra_axes.remove(index);
            if index > 0 {
                self.extra_pos.remove(index - 1);
            }
        }
    }

    /// The id of the main trapq, for an extra axis that needs one of its own.
    pub fn main_trapq(&self) -> usize {
        self.main_trapq
    }

    /// Allocate a trapq for an extra axis.
    pub fn allocate_trapq(&mut self) -> usize {
        self.motion_queuing.allocate_trapq()
    }

    /// Install the kinematics (upstream loads it from `[printer] kinematics`).
    pub fn set_kinematics(&mut self, kinematics: Box<dyn Kinematics>) {
        self.kinematics = Some(kinematics);
    }

    /// The kinematics, for reporting homing state and limits.
    pub fn kinematics(&self) -> Option<&dyn Kinematics> {
        self.kinematics.as_deref()
    }

    /// The kinematics, to change its state (`SET_KINEMATIC_POSITION`'s
    /// `CLEAR_HOMED`).
    pub fn kinematics_mut(&mut self) -> Option<&mut (dyn Kinematics + 'static)> {
        match &mut self.kinematics {
            Some(kinematics) => Some(kinematics.as_mut()),
            None => None,
        }
    }

    /// Where the toolhead has been commanded to.
    pub fn commanded_pos(&self) -> Coord {
        self.commanded_pos
    }

    /// The print time the planner has reached.
    pub fn print_time(&self) -> f64 {
        self.print_time
    }

    /// The live estimate of the print time *now* — `_flush_handler`'s
    /// `est_print_time` (`extras/motion_queuing.py:193,195`), read fresh from
    /// the injected source (see [`EstimatedPrintTime`]), never cached.
    pub fn estimated_print_time(&self) -> f64 {
        self.estimated_print_time.get()
    }

    /// The trapezoid queue the kinematic move is appended to.
    pub fn trapq(&self) -> &Trapq {
        self.motion_queuing.trapq(self.main_trapq)
    }

    /// The motion queue, for setting stepper positions.
    pub fn motion_queuing_mut(&mut self) -> &mut MotionQueuing {
        &mut self.motion_queuing
    }

    /// Where the toolhead reads the MCU's estimated print time from
    /// (`ToolHeadObject::connect` installs it: the primary MCU's own estimate,
    /// read fresh — [`EstimatedPrintTime`]).
    pub fn set_estimated_print_time_source(&mut self, source: EstimatedPrintTime) {
        self.estimated_print_time = source;
    }

    /// Where a raised print time is reported (`ToolHeadObject::connect`
    /// installs it: a hook that parks the event for the flush task — see
    /// [`SyncPrintTimeNotify`]).
    pub fn set_sync_print_time_notify(&mut self, notify: SyncPrintTimeNotify) {
        self.sync_print_time = notify;
    }

    /// Register a lookahead callback (`ToolHead.register_lookahead_callback`,
    /// `klippy/toolhead.py:526-531`).
    ///
    /// With the look-ahead empty it fires at once with the last move time;
    /// with moves queued it rides the queue's last move and fires with that
    /// move's end time when the look-ahead is flushed
    /// (`Move.timing_callbacks`, drained in [`Self::process_lookahead`]).
    pub fn register_lookahead_callback(&mut self, callback: Box<dyn FnOnce(f64) + Send + 'static>) {
        if self.lookahead.is_empty() {
            let last_move_time = self.get_last_move_time();
            callback(last_move_time);
            return;
        }
        // The move was queued after everything ahead of it and before
        // everything behind it (`lookahead_depth` counts with the queue), so
        // it flushes at this index.
        let index = self.lookahead_depth - 1;
        self.parked_lookahead
            .push(ParkedLookahead { index, callback });
    }

    /// Register a flush callback (`MotionQueuing::register_flush_callback`):
    /// it fires with the flush time on every step generation, in registration
    /// order — including generations that produce no steps at all.
    pub fn register_flush_callback(&mut self, callback: Box<dyn Fn(f64) + Send + 'static>) {
        self.motion_queuing.register_flush_callback(callback);
    }

    /// `ToolHead.set_max_velocities` (`klippy/toolhead.py:538-550`): override
    /// the named velocity/acceleration limits, rebuild the derived junction
    /// geometry, and return the four current values.
    ///
    /// The next [`Self::move_to`] profiles from whatever this leaves in
    /// `self.limits`, so the caller that keeps the matching copy
    /// (`SET_VELOCITY_LIMIT`) calls this to keep the two in step.
    ///
    /// `square_corner_velocity` and `min_cruise_ratio` are **full** values:
    /// this layer keeps only their derived geometry, so the caller — which
    /// stores the ratio itself — resolves them and passes them in. The
    /// velocity and acceleration are optional overrides against the limits
    /// already stored here, as upstream's `None` arguments are.
    pub fn set_max_velocities(
        &mut self,
        max_velocity: Option<f64>,
        max_accel: Option<f64>,
        square_corner_velocity: f64,
        min_cruise_ratio: f64,
    ) -> (f64, f64, f64, f64) {
        if let Some(velocity) = max_velocity {
            self.limits.max_velocity = velocity;
        }
        if let Some(accel) = max_accel {
            self.limits.max_accel = accel;
        }
        self.limits = MoveLimits::from_velocity_limits(
            self.limits.max_velocity,
            self.limits.max_accel,
            square_corner_velocity,
            min_cruise_ratio,
        );
        (
            self.limits.max_velocity,
            self.limits.max_accel,
            square_corner_velocity,
            min_cruise_ratio,
        )
    }

    /// Queue a move (`ToolHead.move`, `klippy/toolhead.py:395-409`).
    ///
    /// # Errors
    /// A zero-length move is ignored; the kinematics' own `check_move`
    /// (bounds, per-axis speed limits) is FW5e.
    pub fn move_to(&mut self, newpos: Coord, speed: f64) -> Result<(), CommandError> {
        self.move_to_extended(newpos, &[], speed)
    }

    /// Queue a move that also names the extra axes.
    ///
    /// `extras` are the new positions of the non-kinematic axes on position
    /// index `E_AXIS + 1` and up; an axis the caller leaves out keeps where it
    /// is (upstream's `G1` writes only the letters it is given into a position
    /// list that carries every axis).
    ///
    /// # Errors
    /// As [`Self::move_to`].
    pub fn move_to_extended(
        &mut self,
        newpos: Coord,
        extras: &[f64],
        speed: f64,
    ) -> Result<(), CommandError> {
        let extra_start = self.extra_pos.clone();
        let extra_end: Vec<f64> = extra_start
            .iter()
            .enumerate()
            .map(|(index, start)| extras.get(index).copied().unwrap_or(*start))
            .collect();
        let mut move_ = Move::new_extended(
            self.commanded_pos,
            &extra_start,
            newpos,
            &extra_end,
            speed,
            &self.limits,
        );
        if move_.move_d == 0.0 {
            return Ok(());
        }
        if move_.is_kinematic_move {
            if let Some(kinematics) = &self.kinematics {
                let mut ctx = MoveContext::new(&mut move_);
                kinematics.check_move(&mut ctx)?;
            }
        }
        for (index, axis) in self.extra_axes.iter().enumerate() {
            let ea_index = index + E_AXIS;
            if move_.axes_d[ea_index] != 0.0 {
                let mut ctx = MoveContext::new(&mut move_);
                axis.check_move(&mut ctx, ea_index)?;
            }
        }
        self.commanded_pos = move_.end_pos;
        self.extra_pos = move_.extra_end.clone();
        // Every non-zero move adds exactly one queue entry (`add_move` pushes
        // unconditionally; the zero-length move left above), so the depth
        // counter mirrors the queue's length.
        self.lookahead_depth += 1;
        let want_flush = self.lookahead.add_move(move_, &self.extra_axes);
        if want_flush {
            self.process_lookahead();
        }
        Ok(())
    }

    /// Floor `print_time` at the live estimate plus the start buffer, never
    /// below the horizon already generated (`_calc_print_time`,
    /// `klippy/toolhead.py:260-268`).
    ///
    /// The floor is upstream's: `max(est + BUFFER_TIME_START, kin_time)` where
    /// `kin_time = calc_step_gen_restart(est)` is
    /// `max(est + MIN_KIN_TIME, last_step_gen_time) + kin_flush_delay`
    /// (`motion_queuing.py:190-192`). The `MIN_KIN_TIME` half keeps a move
    /// queued at the floor from starting so close to the estimate that its
    /// steps cannot be generated and sent before their own clock; the
    /// `kin_flush_delay` half keeps it clear of the step filter the generated
    /// horizon already passed.
    ///
    /// Folding both terms in is an **alignment with the upstream formula, not a
    /// behavioural change on its own**: `BUFFER_TIME_START` (0.250) already
    /// exceeds `MIN_KIN_TIME + SDS_CHECK_TIME` (0.101), so the buffer term wins
    /// everywhere the generated horizon does not, and the rest differs only by
    /// the step filter. The step-lead fix a live wait needs is elsewhere (the
    /// fake firmware's step-chain model, `interface/devices/simulator.rs`).
    ///
    /// `kin_flush_delay` is the widest stepper window the scan pass found
    /// ([`Self::recompute_scan_windows`]): a smoothing change inside it may
    /// still rewrite already-generated steps, so a new move is held back by it
    /// rather than by the 1 ms filter alone.
    ///
    /// Raise-only: a horizon the machine has not caught up with is left alone,
    /// so re-priming after an idle moves the next move **forward** — onto
    /// "now plus a buffer", never onto steps already generated. That is the
    /// invariant C5's reproduction broke: with a connect-time snapshot the
    /// floor stood still while the clock advanced, and after `N` seconds idle
    /// every move was planned `N` seconds in the past (see
    /// [`EstimatedPrintTime`]).
    fn calc_print_time(&mut self) {
        let est = self.estimated_print_time.get();
        // `calc_step_gen_restart` (`motion_queuing.py:190-192`), with the
        // `kin_flush_delay` the scan-window pass last computed.
        let kin_time = (est + MIN_KIN_TIME).max(self.last_step_gen_time) + self.kin_flush_delay;
        let min_print_time = (est + BUFFER_TIME_START).max(kin_time);
        if min_print_time > self.print_time {
            self.print_time = min_print_time;
            // Raise-only, as upstream's `send_event` is: a print time the
            // machine has not caught up with was not re-synced, so there is
            // nothing to tell the handlers.
            self.sync_print_time.notify(est, self.print_time);
        }
    }

    /// Flush the look-ahead into the trapq and advance the print time
    /// (`ToolHead._process_lookahead`, `klippy/toolhead.py:269-299`).
    fn process_lookahead(&mut self) {
        let mut moves = self.lookahead.flush(false);
        if moves.is_empty() {
            return;
        }
        // Parked callbacks ride the move they were registered against: moves
        // flush in queue order, so that move sits at its recorded index in
        // this batch. Should a flush ever take only part of the queue, the
        // moves it leaves behind carry their callbacks over, shifted by what
        // this batch took.
        let mut carried = Vec::new();
        for parked in std::mem::take(&mut self.parked_lookahead) {
            if let Some(move_) = moves.get_mut(parked.index) {
                move_.timing_callbacks.push(parked.callback);
            } else {
                carried.push(ParkedLookahead {
                    index: parked.index - moves.len(),
                    callback: parked.callback,
                });
            }
        }
        self.parked_lookahead = carried;
        self.lookahead_depth -= moves.len();
        if self.special_queuing_state {
            // Leaving "NeedPrime": start the print time a buffer ahead of the
            // MCU so the queue is never empty when motion starts — upstream's
            // `_calc_print_time` (`klippy/toolhead.py:260-268`), which reads
            // the estimate **now** rather than from a connect-time snapshot
            // (see [`EstimatedPrintTime`]), and floors further at `kin_time` —
            // `max(est + MIN_KIN_TIME, last_step_gen_time) + kin_flush_delay`
            // (`toolhead.py:263` → `motion_queuing.py:190-192`) — where
            // `last_step_gen_time` is the floor that keeps a move queued after
            // a drip from starting behind steps that drip already generated.
            self.special_queuing_state = false;
            self.calc_print_time();
        }
        let mut next_move_time = self.print_time;
        for mut move_ in moves {
            if move_.is_kinematic_move {
                append_move(
                    self.motion_queuing.trapq_mut(self.main_trapq),
                    next_move_time,
                    &move_,
                );
            }
            // The extra axes queue their own trapezoid, on their own trapq
            // (`klippy/toolhead.py:290-292`). Every registered axis has a slot
            // in the move (`Move::new_extended`).
            for (index, axis) in self.extra_axes.iter().enumerate() {
                let ea_index = index + E_AXIS;
                if move_.axes_d[ea_index] != 0.0 {
                    axis.process_move(&mut self.motion_queuing, next_move_time, &move_, ea_index);
                }
            }
            next_move_time += move_.accel_t + move_.cruise_t + move_.decel_t;
            for callback in move_.timing_callbacks.drain(..) {
                callback(next_move_time);
            }
        }
        self.print_time = next_move_time;
    }

    /// Flush the look-ahead and generate steps up to `step_gen_time`.
    ///
    /// Returns one entry per stepper that produced commands.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from a stepper's compressor.
    pub fn flush_step_generation(
        &mut self,
        step_gen_time: f64,
    ) -> Result<Vec<(String, Vec<StepCommand>)>, StepCompressError> {
        self.process_lookahead();
        // Upstream's `flush_step_generation` opens with `_flush_lookahead()`,
        // which leaves the toolhead in "NeedPrime" (`klippy/toolhead.py:317-319`
        // → `:300-309`), and `_handle_step_flush` returns to it whenever
        // generation reaches the planner horizon (`:310-316`). Either way the
        // next planned move re-runs `_calc_print_time` against the **live**
        // estimate — without this the state is consumed by the first move ever
        // planned and the print time never re-syncs after an idle (C5: a
        // machine idle since connect planned its motion in the past).
        self.special_queuing_state = true;
        let batches = self.motion_queuing.generate(step_gen_time)?;
        // Generation is what advances the horizon: a flush that produced
        // nothing for a stepper still generated *to* `step_gen_time`.
        self.last_step_gen_time = self.last_step_gen_time.max(step_gen_time);
        Ok(batches)
    }

    /// The step-generation horizon this planner has reached
    /// (`last_step_gen_time`).
    ///
    /// It is the floor a later pass must generate **from**, never below: the
    /// solvers and the compressors have already scheduled their steps up to it.
    /// Upstream reads the same value wherever it bounds a flush
    /// (`motion_queuing.py:143-146`), so a caller that keeps its own copy of the
    /// horizon — this host's connected-state flush pass does — has to take the
    /// max of the two.
    pub fn last_step_gen_time(&self) -> f64 {
        self.last_step_gen_time
    }

    /// The widest step generation window among this toolhead's steppers
    /// (`motion_queuing.get_kin_flush_delay`, `motion_queuing.py:123-124`).
    ///
    /// Never below the step filter ([`SDS_CHECK_TIME`]), because with no
    /// stepper, or none whose position function reaches past its own move, the
    /// filter alone is what the floor and the history expiry account for.
    pub fn kin_flush_delay(&self) -> f64 {
        self.kin_flush_delay
    }

    /// Rescan every stepper's step generation window and keep the widest
    /// (`motion_queuing.check_step_generation_scan_windows`,
    /// `motion_queuing.py:126-138`).
    ///
    /// A stepper's window is how far, in seconds, its steps reach past the
    /// first and last move it is active in: an extruder's half smooth time, an
    /// input shaper's pulse span. Steps inside that window are not final — a
    /// change to the smoothing may rewrite them — so both the print-time floor
    /// ([`Self::calc_print_time`]) and the history expiry are held back by the
    /// widest one.
    ///
    /// Call it **whenever a window changes**, as upstream does
    /// (`kinematics/extruder.py:57,66,82`): this is a scan, not a per-flush
    /// cost. A detached stepper (`set_trapq(None)`) is skipped, matching
    /// upstream's NULL-trapq check: it generates no steps, so its window cannot
    /// be rewritten.
    pub fn recompute_scan_windows(&mut self) {
        let mut kin_flush_delay = SDS_CHECK_TIME;
        for stepper in self.motion_queuing.steppers() {
            if stepper.trapq_id().is_none() {
                continue;
            }
            let (pre_active, post_active) = stepper.kinematics().effective_generation_window();
            kin_flush_delay = kin_flush_delay.max(pre_active).max(post_active);
        }
        self.kin_flush_delay = kin_flush_delay;
    }

    /// Plan everything queued so far — upstream's `_flush_lookahead`, the
    /// first half of `ToolHead.wait_moves` (`klippy/toolhead.py:422-429`,
    /// `:300-309`): the toolhead is left in "NeedPrime", so the next planned
    /// move floors `print_time` at the estimate **of that moment** instead of
    /// chaining from a horizon the clock has long passed (`M400` is where a
    /// replay pauses between segments).
    ///
    /// The second half — waiting for the MCU to catch up — needs the clock
    /// estimate *live* and a reactor to sleep on, so it is driven by the
    /// caller: read [`Self::wait_moves_state`] once per iteration and sleep
    /// while it says `waiting` (the `[printer]` object's `M400` does).
    pub fn wait_moves(&mut self) {
        self.process_lookahead();
        self.special_queuing_state = true;
    }

    /// Read what upstream's `wait_moves` loop tests each iteration
    /// (`klippy/toolhead.py:425-427`):
    ///
    /// ```text
    /// while (not self.special_queuing_state
    ///        or self.print_time >= self.mcu.estimated_print_time(eventtime)):
    /// ```
    ///
    /// [`Self::wait_moves`] leaves the toolhead in "NeedPrime", where
    /// `special_queuing_state` is truthy, so while nothing plans new motion the
    /// wait runs until the estimate reaches the horizon; a `process_lookahead`
    /// that moves the planner on takes the state back to "main" (falsy) and
    /// ends the wait — the planner is ahead of the estimate on its own then.
    ///
    /// The estimate is read **fresh** ([`Self::estimated_print_time`]), so the
    /// answer is only good for the iteration that asked: a caller that loops on
    /// this must call it again after every sleep, and must not hold the
    /// planner's lock across that sleep.
    pub fn wait_moves_state(&self) -> WaitMovesState {
        let estimated_print_time = self.estimated_print_time.get();
        WaitMovesState {
            waiting: !self.special_queuing_state || self.print_time >= estimated_print_time,
            print_time: self.print_time,
            estimated_print_time,
        }
    }

    /// The print time the planner has reached (`ToolHead.get_last_move_time`,
    /// `klippy/toolhead.py:320-326`): flush the look-ahead into the trapq
    /// first, and while the toolhead is still priming run `_calc_print_time`
    /// as well — a caller that schedules from this value (the probe callback,
    /// the restart hooks) must not get a horizon the clock has already passed.
    /// Updates based on this value are what the MCU is executing.
    pub fn get_last_move_time(&mut self) -> f64 {
        self.process_lookahead();
        if self.special_queuing_state {
            self.calc_print_time();
        }
        self.print_time
    }

    /// Drop every queued move table entry (`motion_quuing.wipe_trapq`), as a
    /// homing move does once it has stopped.
    pub fn wipe_trapq(&mut self) {
        self.motion_queuing
            .trapq_mut(self.main_trapq)
            .finalize_moves(f64::MAX, 0.0);
    }

    /// Load one move straight into the trapq, bypassing the look-ahead
    /// (`ToolHead.drip_move` / `_drip_load_trapq`, `klippy/toolhead.py:459-492`).
    ///
    /// A homing move must not be joined to a previous move — it has to stop on
    /// its own endstop — so this sets its junction speeds to zero (start and end
    /// at rest) and appends its trapezoid at the current print time. As
    /// upstream's `_drip_load_trapq` (`toolhead.py:459-476`), **loading does
    /// not advance `print_time`**: the move's end is returned as the drip's
    /// boundary, and step generation advances with the drip itself
    /// (`motion_queuing.py:265-290`, segment by segment, wherever the drip
    /// stops). Advancing `print_time` to the end here would leave the planner
    /// claiming a horizon the machine never reached when the drip aborts at its
    /// endstop — the next queued move would inherit it as its start time.
    ///
    /// Returns the move's start and end print times.
    ///
    /// A drip carries the four modelled axes only: its move starts from
    /// `commanded_pos` and the extra axes are neither planned nor queued here
    /// (upstream's `_drip_load_trapq` reaches the main trapq too, and a manual
    /// stepper has no drip path in this port yet).
    ///
    /// # Errors
    /// The kinematics' `check_move` refusal.
    pub fn drip_move(&mut self, newpos: Coord, speed: f64) -> Result<(f64, f64), CommandError> {
        let mut move_ = Move::new(self.commanded_pos, newpos, speed, &self.limits);
        if move_.move_d == 0.0 {
            self.process_lookahead();
            return Ok((self.print_time, self.print_time));
        }
        if move_.is_kinematic_move {
            if let Some(kinematics) = &self.kinematics {
                let mut ctx = MoveContext::new(&mut move_);
                kinematics.check_move(&mut ctx)?;
            }
        }
        self.process_lookahead();
        move_.set_junction(0.0, move_.max_cruise_v2, 0.0);
        // `_drip_load_trapq` calls `_calc_print_time()` before taking the start
        // (`toolhead.py:465` → `:260-268`): `print_time` is raised — never
        // lowered — to `max(estimated_print_time + BUFFER_TIME_START, kin_time)`,
        // with `kin_time = max(est + MIN_KIN_TIME, last_step_gen_time) +
        // kin_flush_delay` (`motion_queuing.py:190-192`). Both floors are
        // modelled (`calc_print_time`): the buffer, read live, and the generated
        // horizon, which is what keeps the drip's start from landing behind
        // steps a previous drip already generated (an "Invalid sequence" in the
        // step solver).
        self.calc_print_time();
        let start_time = self.print_time;
        append_move(
            self.motion_queuing.trapq_mut(self.main_trapq),
            start_time,
            &move_,
        );
        let end_time = start_time + move_.accel_t + move_.cruise_t + move_.decel_t;
        self.commanded_pos = move_.end_pos;
        Ok((start_time, end_time))
    }

    /// Wait `delay` seconds without moving (`ToolHead.dwell`,
    /// `klippy/toolhead.py:417-421`).
    pub fn dwell(&mut self, delay: f64) {
        self.process_lookahead();
        // `_flush_lookahead` re-enters "NeedPrime" (`klippy/toolhead.py:300-309`),
        // which is what lets the next move re-floor `print_time` against the
        // live estimate; the delay itself is added on top, as upstream does
        // (`toolhead.py:417-421`).
        self.special_queuing_state = true;
        self.print_time += delay.max(0.0);
    }

    /// Append a move's trapezoid directly, for tests and `drip_move`.
    pub fn append_move(&mut self, print_time: f64, move_: &Move) {
        append_move(
            self.motion_queuing.trapq_mut(self.main_trapq),
            print_time,
            move_,
        );
    }

    /// Force the toolhead to `newpos`, marking `homing_axes` as homed
    /// (`ToolHead.set_position`, `klippy/toolhead.py:383-390`).
    ///
    /// This is `G92`'s low-level half and `SET_KINEMATIC_POSITION`: the print
    /// time does not move and no steps are generated, but the solver and the
    /// kinematics are told where the toolhead is so the next move starts here.
    /// The trapq's current position is rewritten at the same time, which drops
    /// or truncates any history the old position recorded.
    pub fn set_position(&mut self, newpos: Coord, homing_axes: &[usize]) {
        self.process_lookahead();
        // Upstream's `set_position` flushes step generation first
        // (`toolhead.py:384` → `flush_step_generation`, `:317-319`), whose
        // `_flush_lookahead` re-enters "NeedPrime" (`:300-304`) — so the next
        // planned move re-runs `_calc_print_time` and floors `print_time` at
        // the generated horizon. Re-prime the state here for the same reason:
        // without it the position is re-anchored at a `print_time` that a
        // completed drip's generated steps may already be past, and the next
        // move solves out of order ("Invalid sequence").
        self.special_queuing_state = true;
        self.motion_queuing
            .trapq_mut(self.main_trapq)
            .set_position(self.print_time, Xyz::from(newpos));
        self.commanded_pos = newpos;
        if let Some(kinematics) = &mut self.kinematics {
            kinematics.set_position(newpos, homing_axes);
        }
        // Each stepper that reads the **main** trapq is told where the toolhead
        // is. One that reads another queue is left alone: it tracks that
        // queue's own coordinates (an extra axis' stepper follows the extrusion
        // amount, an `[extruder_stepper]` follows the extruder it is bound to),
        // and forcing the toolhead's coordinate into it would make it step the
        // whole gap on the next range. Upstream's `ToolHead.set_position`
        // (`toolhead.py:383-390`) likewise only rewrites the main trapq
        // (`trapq_set_position`).
        let main_trapq = self.main_trapq;
        for stepper in self.motion_queuing.steppers_mut() {
            if stepper.trapq_id() != Some(main_trapq) {
                continue;
            }
            stepper.set_position(newpos.into());
        }
    }

    /// Drop finished moves from the trapq once the solvers are past them.
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        self.motion_queuing
            .finalize_moves(print_time, clear_history_time);
    }
}

/// Append one planned move's trapezoid to `trapq`
/// (`trapq_append` via `ToolHead._process_lookahead`).
fn append_move(trapq: &mut Trapq, print_time: f64, move_: &Move) {
    trapq.append(
        print_time,
        move_.accel_t,
        move_.cruise_t,
        move_.decel_t,
        Xyz::new(
            move_.start_pos.x(),
            move_.start_pos.y(),
            move_.start_pos.z(),
        ),
        Xyz::new(move_.axes_r[0], move_.axes_r[1], move_.axes_r[2]),
        move_.start_v,
        move_.cruise_v,
        move_.accel,
    );
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::AXES;
    use crate::core::klippy::motion::itersolve::Axis;
    use crate::core::klippy::motion::pressure_advance::ExtruderPressureAdvance;
    use crate::core::klippy::motion::stepcompress::StepCommand;
    use crate::core::klippy::motion::stepper::Stepper;
    use std::sync::Mutex;

    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    /// A toolhead with one X stepper, 1 mm per step.
    fn toolhead() -> ToolHead {
        let mut toolhead = ToolHead::new(limits());
        toolhead.add_stepper(Stepper::cartesian(
            "stepper_x",
            0,
            1.0,
            Axis::X,
            1_000_000.0,
        ));
        toolhead
    }

    /// The number of steps across all steppers (the commands are compressed).
    fn step_count(batches: &[(String, Vec<StepCommand>)]) -> u32 {
        batches
            .iter()
            .flat_map(|(_, commands)| commands)
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum()
    }

    /// A fake extra axis: records its calls and queues into its own trapq.
    #[derive(Debug)]
    struct FakeExtraAxis {
        name: String,
        trapq: usize,
        checked: Mutex<Vec<usize>>,
        junction_calls: Mutex<usize>,
        queued: Mutex<Vec<f64>>,
    }

    impl FakeExtraAxis {
        fn new(trapq: usize) -> Self {
            Self {
                name: "extruder".to_string(),
                trapq,
                checked: Mutex::new(Vec::new()),
                junction_calls: Mutex::new(0),
                queued: Mutex::new(Vec::new()),
            }
        }
    }

    impl ExtraAxis for FakeExtraAxis {
        fn name(&self) -> &str {
            &self.name
        }

        fn check_move(
            &self,
            _ctx: &mut MoveContext<'_>,
            ea_index: usize,
        ) -> Result<(), CommandError> {
            self.checked.lock().unwrap().push(ea_index);
            Ok(())
        }

        fn calc_junction(&self, _prev: &Move, _cur: &Move, _ea_index: usize) -> f64 {
            *self.junction_calls.lock().unwrap() += 1;
            1234.0
        }

        fn process_move(
            &self,
            queuing: &mut MotionQueuing,
            print_time: f64,
            move_: &Move,
            ea_index: usize,
        ) {
            self.queued.lock().unwrap().push(move_.end_axis(ea_index));
            queuing.trapq_mut(self.trapq).append(
                print_time,
                move_.accel_t,
                move_.cruise_t,
                move_.decel_t,
                Xyz::new(move_.start_axis(ea_index), 0.0, 0.0),
                Xyz::new(1.0, 0.0, 0.0),
                move_.start_v,
                move_.cruise_v,
                move_.accel,
            );
        }

        fn find_past_position(&self, _print_time: f64) -> f64 {
            0.0
        }

        fn get_status(&self) -> serde_json::Value {
            serde_json::json!({})
        }
    }

    #[test]
    fn test_an_extra_axis_is_checked_and_queued_in_its_own_trapq() {
        let mut toolhead = toolhead();
        let trapq = toolhead.allocate_trapq();
        let axis = Arc::new(FakeExtraAxis::new(trapq));
        toolhead.add_extra_axis(axis.clone());

        // A pure extrusion: the kinematic steppers stay still, the extra axis
        // is checked and queued.
        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 5.0), 10.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0).unwrap();

        assert_eq!(*axis.checked.lock().unwrap(), [3]);
        assert_eq!(*axis.queued.lock().unwrap(), [5.0]);
        // The trapezoid lands in the extra axis' trapq (three phases).
        assert!(!toolhead
            .motion_queuing_mut()
            .trapq(trapq)
            .moves()
            .is_empty());
        // The X stepper did not move, and the fake axis is not a `Stepper`.
        assert!(batches.is_empty());
    }

    #[test]
    fn test_an_extra_axis_at_index_four_is_checked_and_queued() {
        let mut toolhead = toolhead();
        // The extruder's slot, then the A axis'.
        let extruder = toolhead.allocate_trapq();
        toolhead.add_extra_axis(Arc::new(FakeExtraAxis::new(extruder)) as Arc<dyn ExtraAxis>);
        let trapq = toolhead.allocate_trapq();
        let axis = Arc::new(FakeExtraAxis::new(trapq));
        toolhead.add_extra_axis(Arc::clone(&axis) as Arc<dyn ExtraAxis>);

        // A pure extra-axis move: the kinematic steppers stay still.
        toolhead
            .move_to_extended(Coord::new(0.0, 0.0, 0.0, 0.0), &[10.0], 10.0)
            .unwrap();
        {
            let move_ = toolhead.lookahead.last().unwrap();
            // The move reaches past the four modelled axes and the extra axis'
            // distance is in it (`G1 A10`'s `axes_r[4]`).
            assert_eq!(move_.axes_r.len(), AXES + 1);
            assert_eq!(move_.axes_r[AXES], 1.0);
            assert_eq!(move_.extra_end, [10.0]);
            assert_eq!(move_.end_axis(AXES), 10.0);
        }
        assert_eq!(*axis.checked.lock().unwrap(), [AXES]);

        let batches = toolhead.flush_step_generation(1.0).unwrap();

        assert_eq!(*axis.queued.lock().unwrap(), [10.0]);
        assert!(!toolhead
            .motion_queuing_mut()
            .trapq(trapq)
            .moves()
            .is_empty());
        assert!(batches.is_empty(), "the extra axis is not a `Stepper`");
    }

    /// The main trapq stays three axes: the extra slots live beside it, one
    /// trapq per extra axis (`toolhead.py:284-292`).
    #[test]
    fn test_the_main_trapq_stays_three_axes_with_an_extra_axis_present() {
        let mut toolhead = toolhead();
        let trapq = toolhead.allocate_trapq();
        toolhead.add_extra_axis(Arc::new(FakeExtraAxis::new(trapq)) as Arc<dyn ExtraAxis>);

        toolhead
            .move_to_extended(Coord::new(10.0, 0.0, 0.0, 1.0), &[1.0], 100.0)
            .unwrap();
        toolhead.flush_step_generation(1.0).unwrap();

        // The extra axis' ratio must not leak into the main trapq's three
        // slots: every segment is a plain X move, filler included.
        let ratios: Vec<Xyz> = toolhead.trapq().moves().iter().map(|s| s.axes_r).collect();
        assert!(
            ratios.contains(&Xyz::new(1.0, 0.0, 0.0)),
            "the kinematic direction: {ratios:?}"
        );
        assert!(
            ratios
                .iter()
                .all(|ratio| ratio.y() == 0.0 && ratio.z() == 0.0),
            "{ratios:?}"
        );
    }

    #[test]
    fn test_an_extra_axis_limits_the_junction() {
        let mut toolhead = toolhead();
        let trapq = toolhead.allocate_trapq();
        let axis = Arc::new(FakeExtraAxis::new(trapq));
        toolhead.add_extra_axis(axis.clone());

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead
            .move_to(Coord::new(20.0, 0.0, 0.0, 1.0), 100.0)
            .unwrap();

        // The second move's junction folds in the extra axis' limit.
        assert_eq!(*axis.junction_calls.lock().unwrap(), 1);
        let last = toolhead.lookahead.last().unwrap();
        assert_eq!(last.max_start_v2, 1234.0);
    }

    #[test]
    fn test_a_move_reaches_the_trapq_and_generates_queue_steps() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0).unwrap();

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, "stepper_x");
        // 10 mm of travel at 1 mm per step.
        assert_eq!(step_count(&batches), 10);
        assert_eq!(toolhead.commanded_pos(), Coord::new(10.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn test_two_collinear_moves_keep_their_junction_speed() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead
            .move_to(Coord::new(20.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0).unwrap();

        // No stop between the moves: 20 mm at 1 mm per step.
        assert_eq!(step_count(&batches), 20);
    }

    #[test]
    fn test_dwell_advances_print_time() {
        let mut toolhead = toolhead();
        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead.flush_step_generation(1.0).unwrap();
        let before = toolhead.print_time();

        toolhead.dwell(0.5);

        assert!((toolhead.print_time() - (before + 0.5)).abs() < 1e-9);
    }

    /// A toolhead whose estimate the test drives: the prime floor has to read
    /// it at plan time (C5), so a test moves it between two plans.
    fn toolhead_with_estimate(est: &Arc<Mutex<f64>>) -> ToolHead {
        let mut toolhead = toolhead();
        let estimate = Arc::clone(est);
        toolhead.set_estimated_print_time_source(EstimatedPrintTime::new(move || {
            *estimate.lock().unwrap()
        }));
        toolhead
    }

    #[test]
    fn test_the_prime_floor_reads_the_estimate_at_plan_time_not_at_connect() {
        let est = Arc::new(Mutex::new(0.0));
        let mut toolhead = toolhead_with_estimate(&est);

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let first = toolhead.get_last_move_time();
        assert!(
            first >= BUFFER_TIME_START - 1e-9,
            "the first prime floors at the buffer above the estimate: {first}"
        );

        // The clock runs on while the planner idles: the next prime must read
        // the estimate **of that moment**. A connect-time reading would keep
        // flooring here at 0.25 for ever, and every later move would be
        // planned in the past (C5: idle 301 s → every step batch's
        // `completion` sat 301 s behind `est` and the whole motion was dumped
        // at once).
        *est.lock().unwrap() = 301.0;
        // `set_position` re-enters "NeedPrime" (the re-arm this host already
        // had, `klippy/toolhead.py:383-390`), so this is a fresh prime — the
        // only thing under test is *which* estimate it reads.
        toolhead.set_position(Coord::new(10.0, 0.0, 0.0, 0.0), &[0]);
        toolhead
            .move_to(Coord::new(20.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let horizon = toolhead.get_last_move_time();

        assert!(
            horizon >= 301.0 + BUFFER_TIME_START - 1e-9,
            "the floor follows the estimate: {horizon}"
        );
    }

    /// The floor is upstream's `_calc_print_time`
    /// (`klippy/toolhead.py:260-268` → `motion_queuing.py:190-192`):
    /// `max(est + BUFFER_TIME_START,
    ///      max(est + MIN_KIN_TIME, last_step_gen_time) + kin_flush_delay)`.
    ///
    /// The **boundary the kin terms own** is the one where the generated
    /// horizon is further ahead than the buffer: `last_step_gen_time >
    /// est + MIN_KIN_TIME`. Then the floor is that horizon plus the filter, not
    /// the bare horizon — a move planned on the raw horizon would start steps
    /// the filter window still covers.
    ///
    /// This host's `BUFFER_TIME_START` (0.250) already exceeds `MIN_KIN_TIME`
    /// (0.100) + `SDS_CHECK_TIME` (0.001), so the boundary is the only place
    /// the kin terms can ever bind — folding them in is an alignment with the
    /// upstream formula, and this pins exactly what it changes.
    #[test]
    fn test_the_prime_floor_adds_the_step_filter_on_the_generated_horizon() {
        let est = Arc::new(Mutex::new(0.0));
        let mut toolhead = toolhead_with_estimate(&est);
        // Generation reaches a horizon the estimate will lag far behind.
        toolhead.flush_step_generation(5.0).unwrap();
        assert_eq!(toolhead.last_step_gen_time, 5.0);

        // (a) The generated horizon binds: the floor is that horizon plus the
        // step filter — not the bare horizon, and not the buffer.
        *est.lock().unwrap() = 1.0;
        assert!(1.0 + MIN_KIN_TIME + SDS_CHECK_TIME < 5.0);
        toolhead.set_position(Coord::default(), &[0]);
        let floor = toolhead.get_last_move_time();
        assert!(
            (floor - (5.0 + SDS_CHECK_TIME)).abs() < 1e-9,
            "the floor is the generated horizon plus the filter: {floor}"
        );

        // (b) The buffer binds instead: the estimate has caught up past the
        // horizon, so `est + BUFFER_TIME_START` wins.
        assert!(10.0 + MIN_KIN_TIME + SDS_CHECK_TIME < 10.0 + BUFFER_TIME_START);
        *est.lock().unwrap() = 10.0;
        toolhead.set_position(Coord::default(), &[0]);
        let floor = toolhead.get_last_move_time();
        assert!(
            (floor - (10.0 + BUFFER_TIME_START)).abs() < 1e-9,
            "the buffer floor binds once the estimate passes the horizon: {floor}"
        );
    }

    /// A pressure-advance stepper with `smooth_time`, on the main trapq: the
    /// window steps are generated over is half of it (`kin_extruder.c:144-145`).
    fn pressure_advance_stepper(
        name: &str,
        oid: u32,
        pressure_advance: f64,
        smooth_time: f64,
    ) -> Stepper {
        let mut stepper = Stepper::cartesian(name, oid, 1.0, Axis::X, 1_000_000.0);
        stepper.set_pressure_advance(ExtruderPressureAdvance::new(pressure_advance, smooth_time));
        stepper
    }

    /// The flush delay starts at the step filter and only the scan pass raises
    /// it (`check_step_generation_scan_windows`, `motion_queuing.py:126-138`):
    /// with no stepper — and with steppers whose position function reads only
    /// the move it is asked about — it stays at `SDS_CHECK_TIME`.
    #[test]
    fn test_the_flush_delay_starts_at_the_step_filter() {
        let mut bare = ToolHead::new(limits());
        bare.recompute_scan_windows();
        assert_eq!(bare.kin_flush_delay(), SDS_CHECK_TIME);

        let mut toolhead = toolhead();
        toolhead.recompute_scan_windows();
        assert_eq!(toolhead.kin_flush_delay(), SDS_CHECK_TIME);
    }

    /// An extruder with pressure advance reaches half a smooth time past the
    /// moves it is active in, and that window is the flush delay
    /// (`kin_extruder.c:144-145` → `motion_queuing.py:126-138`).
    #[test]
    fn test_a_pressure_advance_stepper_raises_the_flush_delay_to_half_its_smooth_time() {
        let mut toolhead = toolhead();
        toolhead.add_stepper(pressure_advance_stepper("stepper_e", 1, 0.05, 0.04));

        toolhead.recompute_scan_windows();

        assert!(
            (toolhead.kin_flush_delay() - 0.02).abs() < 1e-12,
            "half the smooth time: {}",
            toolhead.kin_flush_delay()
        );
    }

    /// The pass keeps the **widest** window: the delay has to cover the stepper
    /// that reaches furthest past its own moves, whichever one that is.
    #[test]
    fn test_the_flush_delay_is_the_widest_stepper_window() {
        let mut toolhead = toolhead();
        toolhead.add_stepper(pressure_advance_stepper("stepper_e", 1, 0.05, 0.04));
        let mut shaper = Stepper::cartesian("stepper_y", 2, 1.0, Axis::Y, 1_000_000.0);
        // An input shaper's pulse span, as `input_shaper` reports it.
        shaper.kinematics_mut().set_generation_window(0.03, 0.035);
        toolhead.add_stepper(shaper);

        toolhead.recompute_scan_windows();

        assert!(
            (toolhead.kin_flush_delay() - 0.035).abs() < 1e-12,
            "the widest window wins over the extruder's half smooth time: {}",
            toolhead.kin_flush_delay()
        );
    }

    /// A `SET_PRESSURE_ADVANCE` that changes the smooth time changes the window
    /// (`kinematics/extruder.py:66,82`): the rescan reads the live values, so
    /// the delay follows without the stepper being reinstalled.
    #[test]
    fn test_a_changed_smooth_time_moves_the_flush_delay() {
        let mut toolhead = toolhead();
        let stepper = pressure_advance_stepper("stepper_e", 1, 0.05, 0.04);
        let pressure_advance = Arc::clone(stepper.kinematics().pressure_advance().unwrap());
        toolhead.add_stepper(stepper);
        toolhead.recompute_scan_windows();
        assert!((toolhead.kin_flush_delay() - 0.02).abs() < 1e-12);

        pressure_advance.set(1.5, 0.05, 0.08);
        toolhead.recompute_scan_windows();

        assert!(
            (toolhead.kin_flush_delay() - 0.04).abs() < 1e-12,
            "the wider smooth time raises the delay: {}",
            toolhead.kin_flush_delay()
        );
    }

    /// The floor is read from the **runtime** delay, not the filter constant:
    /// with a wide window the generated horizon binds by that window
    /// (`calc_step_gen_restart`, `motion_queuing.py:190-192`).
    #[test]
    fn test_the_prime_floor_adds_the_runtime_flush_delay_on_the_generated_horizon() {
        let est = Arc::new(Mutex::new(0.0));
        let mut toolhead = toolhead_with_estimate(&est);
        toolhead.add_stepper(pressure_advance_stepper("stepper_e", 1, 0.05, 0.2));
        toolhead.recompute_scan_windows();
        // Generation reaches a horizon the estimate lags far behind, so the
        // kin term — not the buffer — is what the floor is built from.
        toolhead.flush_step_generation(5.0).unwrap();
        *est.lock().unwrap() = 1.0;
        assert!(1.0 + MIN_KIN_TIME + toolhead.kin_flush_delay() < 5.0);

        toolhead.set_position(Coord::default(), &[0]);
        let floor = toolhead.get_last_move_time();

        assert!(
            (floor - (5.0 + 0.1)).abs() < 1e-9,
            "the floor is the horizon plus the extruder's half smooth time: {floor}"
        );
    }

    /// Upstream sends `toolhead:sync_print_time` only inside the branch that
    /// raises the print time (`klippy/toolhead.py:264-268`), and this layer's
    /// hook fires from the same branch: a floor that does not move the print
    /// time reports nothing.
    #[test]
    fn test_the_sync_print_time_notify_fires_only_when_the_print_time_is_raised() {
        let est = Arc::new(Mutex::new(0.0));
        let mut toolhead = toolhead_with_estimate(&est);
        let raised = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&raised);
        toolhead.set_sync_print_time_notify(SyncPrintTimeNotify::new(move |est, print_time| {
            sink.lock().unwrap().push((est, print_time));
        }));

        // The first prime floors the print time at the buffer: one report.
        toolhead.set_position(Coord::default(), &[0]);
        let first = toolhead.get_last_move_time();
        assert!((first - BUFFER_TIME_START).abs() < 1e-9);
        assert_eq!(*raised.lock().unwrap(), [(0.0, BUFFER_TIME_START)]);

        // A second prime reads the same estimate: nothing raised, nothing
        // reported.
        toolhead.set_position(Coord::default(), &[0]);
        let second = toolhead.get_last_move_time();
        assert!((second - BUFFER_TIME_START).abs() < 1e-9);
        assert_eq!(raised.lock().unwrap().len(), 1);

        // The estimate runs on and the next prime raises again: the report
        // carries that raise's estimate and print time.
        *est.lock().unwrap() = 10.0;
        toolhead.set_position(Coord::default(), &[0]);
        let third = toolhead.get_last_move_time();
        assert!((third - (10.0 + BUFFER_TIME_START)).abs() < 1e-9);
        assert_eq!(
            *raised.lock().unwrap(),
            [(0.0, BUFFER_TIME_START), (10.0, 10.0 + BUFFER_TIME_START)]
        );
    }

    /// The three re-arm paths upstream's `_flush_lookahead` covers
    /// (`klippy/toolhead.py:300-309`) — each has to leave the toolhead primed
    /// so the next move re-floors against the live estimate. One test each,
    /// because removing any one of the three re-arms must turn exactly its own
    /// test red.
    mod need_prime_rearm {
        use super::*;
        /// Idle the planner, exercise one re-arm path, then plan again and
        /// report the horizon that path's re-prime produced.
        fn horizon_after_idle(toolhead: &mut ToolHead, est: &Arc<Mutex<f64>>, rearm: ReArm) -> f64 {
            toolhead
                .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
                .unwrap();
            let _ = toolhead.get_last_move_time(); // plans: the first prime is spent
            *est.lock().unwrap() = 60.0; // the clock runs on while the planner idles
            match rearm {
                ReArm::WaitMoves => toolhead.wait_moves(), // `M400`
                ReArm::Dwell => toolhead.dwell(0.5),       // `G4`
                ReArm::Flush => {
                    toolhead.flush_step_generation(1.0).unwrap(); // generation
                }
            }
            toolhead
                .move_to(Coord::new(20.0, 0.0, 0.0, 0.0), 100.0)
                .unwrap();
            toolhead.get_last_move_time()
        }

        /// Which `_flush_lookahead` equivalent the path under test runs.
        enum ReArm {
            WaitMoves,
            Dwell,
            Flush,
        }

        fn assert_reprimed(horizon: f64, path: &str) {
            assert!(
                horizon >= 60.0 + BUFFER_TIME_START - 1e-9,
                "{path} left the toolhead primed, so the next move floors at the live \
                 estimate: {horizon}"
            );
        }

        #[test]
        fn test_wait_moves_re_primes_for_the_next_move() {
            let est = Arc::new(Mutex::new(0.0));
            let mut toolhead = toolhead_with_estimate(&est);

            let horizon = horizon_after_idle(&mut toolhead, &est, ReArm::WaitMoves);

            assert_reprimed(horizon, "wait_moves");
        }

        #[test]
        fn test_dwell_re_primes_for_the_next_move() {
            let est = Arc::new(Mutex::new(0.0));
            let mut toolhead = toolhead_with_estimate(&est);

            let horizon = horizon_after_idle(&mut toolhead, &est, ReArm::Dwell);

            assert_reprimed(horizon, "dwell");
        }

        #[test]
        fn test_flush_step_generation_re_primes_for_the_next_move() {
            let est = Arc::new(Mutex::new(0.0));
            let mut toolhead = toolhead_with_estimate(&est);

            let horizon = horizon_after_idle(&mut toolhead, &est, ReArm::Flush);

            assert_reprimed(horizon, "flush_step_generation");
        }
    }

    #[test]
    fn test_a_batch_planned_after_an_idle_lands_in_the_current_clock_domain() {
        let est = Arc::new(Mutex::new(0.0));
        let mut toolhead = toolhead_with_estimate(&est);

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        // What the next batch carries as `start` — and so as its `req_clock`
        // (`StepBatchClocks`, `extras/toolhead.rs`).
        let req_horizon = toolhead.get_last_move_time();

        *est.lock().unwrap() = 301.0;
        toolhead.set_position(Coord::new(10.0, 0.0, 0.0, 0.0), &[0]);
        toolhead
            .move_to(Coord::new(20.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        // The batch's `completion` — the clock that frees its slots.
        let completion_horizon = toolhead.get_last_move_time();
        let est_now = *est.lock().unwrap();

        // `start`/`completion` are print times converted to this MCU's clock
        // linearly (`stepcompress.rs:254-256`), so this ordering *is* the
        // clocks' ordering, and the gates judge `req_clock` against
        // `estimated_clock` with `MIN_REQTIME_DELTA` of lead (`mcu/mod.rs`):
        // a `req` at or behind the estimate releases on sight, a `completion`
        // at or after it is a batch the firmware can still schedule.
        assert!(
            req_horizon <= est_now + 0.100,
            "req inside the release window, not queued into the future: {req_horizon} vs {est_now}"
        );
        assert!(
            completion_horizon >= est_now,
            "completion not before the estimate — an expired batch is the C5 dump: \
             {completion_horizon} vs {est_now}"
        );
    }

    #[test]
    fn test_drip_move_loads_the_trapq_directly() {
        let mut toolhead = toolhead();

        let (start, end) = toolhead
            .drip_move(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();

        assert!(end > start);
        assert_eq!(toolhead.commanded_pos().x(), 10.0);
        // Loading does not advance `print_time`: upstream's
        // `_drip_load_trapq` (`toolhead.py:459-476`) returns start/end without
        // touching it, so a drip aborted at its endstop cannot leave the
        // planner claiming a horizon the machine never reached — `end` is only
        // the drip's boundary.
        assert_eq!(toolhead.get_last_move_time(), start);
        // The move is in the trapq and generates its steps.
        let batches = toolhead.flush_step_generation(end).unwrap();
        assert_eq!(step_count(&batches), 10);
        // Wiping the trapq leaves nothing queued.
        toolhead.wipe_trapq();
        assert!(toolhead.trapq().moves().is_empty());
    }

    #[test]
    fn test_a_zero_length_drip_move_does_nothing() {
        let mut toolhead = toolhead();

        let (start, end) = toolhead.drip_move(Coord::default(), 100.0).unwrap();

        assert_eq!(start, end);
        assert!(toolhead.trapq().moves().is_empty());
    }

    #[test]
    fn test_an_unhomed_axis_refuses_a_move() {
        use crate::core::klippy::motion::kinematics::CartesianKinematics;

        let mut toolhead = toolhead();
        toolhead.set_kinematics(Box::new(CartesianKinematics::new(
            ["stepper_x".into(), "stepper_y".into(), "stepper_z".into()],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            crate::core::klippy::motion::kinematics::CartesianTransform::Standard,
        )));

        let err = toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap_err();

        assert!(err.to_string().contains("Must home axis first"), "{err}");
    }

    #[test]
    fn test_a_zero_length_move_is_ignored() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();

        assert!(toolhead.trapq().moves().is_empty());
    }
}
