//! The iterative step solver.
//!
//! Upstream's `chelper/itersolve.c`, with the per-kinematics position functions
//! in `chelper/kin_*.c`: for each stepper it walks the [`Trapq`] and asks a
//! position function where the stepper is at a given time, then finds the times
//! at which it has moved a full `step_dist`.
//!
//! The search is a secant method with a bisection fallback
//! (`itersolve_gen_steps_range`). Upstream uses it because the position
//! function can be any nonlinear function of the move (delta, polar, …);
//! cartesian is the trivial case where it reads one axis.

use std::sync::Arc;

use crate::core::klippy::mathutil::Xyz;

use super::pressure_advance::{ExtruderPressureAdvance, PressureAdvanceEntry};
use super::shaper::KinematicShaper;
use super::stepcompress::{StepCompressError, StepCompressor};
use super::trapq::{MoveSegment, Trapq};

/// How far the solver may look ahead when re-seeking a step time, in seconds
/// (`SEEK_TIME_RESET`, `chelper/itersolve.c:24`).
const SEEK_TIME_RESET: f64 = 0.000_100;

/// The position function of one stepper: where it is `move_time` seconds into a
/// segment, in millimetres, with the function's parameters bound.
///
/// Upstream's cartesian callback reads only the move
/// (`cart_stepper_x_calc_position`, `chelper/kin_cartesian.c:14-20`), but
/// `delta_stepper_alloc` (`chelper/kin_delta.c:16-41`) allocates its solver
/// with `arm2`, `tower_x` and `tower_y` from the config. A plain function
/// pointer cannot carry those, so the bound function keeps a small
/// parameter block beside it. The value stays `Copy`, so `SolverSpec` and
/// `StepKinematics` are unchanged; the stateless solvers bind zeros.
///
/// Six floats is what the widest solver needs: `rotary_delta_stepper_alloc`
/// (`chelper/kin_rotary_delta.c:56-72`) carries shoulder radius, shoulder
/// height, the tower's `cos`/`sin`, and the squared upper/lower arm lengths.
#[derive(Debug, Clone, Copy)]
pub struct PositionFn {
    /// The solver, reading its parameters as `params` (`[arm2, tower_x,
    /// tower_y, …]` for delta, the six rotary values for `rotary_delta`,
    /// unused for the stateless ones).
    f: fn(&MoveSegment, f64, &[f64; 6]) -> f64,
    /// The parameters bound into `f`.
    params: [f64; 6],
}

impl PositionFn {
    /// Bind a solver to its parameters (`delta_stepper_alloc(arm2, tower_x,
    /// tower_y)`).
    pub fn bind(f: fn(&MoveSegment, f64, &[f64; 6]) -> f64, params: [f64; 6]) -> Self {
        Self { f, params }
    }

    /// Where the stepper is `move_time` seconds into `segment`.
    #[inline]
    pub fn call(&self, segment: &MoveSegment, move_time: f64) -> f64 {
        (self.f)(segment, move_time, &self.params)
    }
}

/// The axes a stepper moves (upstream's `AF_X`/`AF_Y`/`AF_Z`,
/// `chelper/itersolve.h:6-9`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AxisFlags(u8);

impl AxisFlags {
    /// No axis.
    pub const NONE: Self = Self(0);
    /// The X axis.
    pub const X: Self = Self(1);
    /// The Y axis.
    pub const Y: Self = Self(2);
    /// The Z axis.
    pub const Z: Self = Self(4);

    /// The union of two flag sets.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Whether any of `other`'s axes is set here.
    pub const fn intersects(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }

    /// Whether no axis is set.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

/// A position unwrap: correct one raw position-fn result against the
/// commanded position (`kin_polar.c`'s angle callback reads
/// `sk->commanded_pos` and shifts the wrapped `atan2` result by ±2π).
///
/// Both hooks are `None` for every solver upstream runs without a
/// `post_cb`/stateful callback (cartesian, corexy, extruder): no behaviour
/// changes for them.
pub type PositionUnwrap = fn(raw: f64, commanded: f64) -> f64;

/// A commanded-position fixup run after each generated range
/// (`kin_polar.c`'s `polar_stepper_angle_post_fixup`, called from
/// `itersolve_gen_steps_range` as `sk->post_cb`).
pub type PositionPost = fn(commanded: &mut f64);

/// The segments a position function may read while generating steps: the
/// trapq's finished history (oldest first) followed by its queued segments
/// (`chelper/trapq.c` keeps both in one linked list), the index of the segment
/// being generated, and the pressure-advance schedule snapshot the extruder
/// solver reads.
///
/// A solver that reads only the segment it is asked about gets a one-segment
/// window ([`SegmentWindow::single`]); the pressure-advance extruder needs its
/// neighbours for the smoothing window (`kin_extruder.c`'s
/// `pa_range_integrate` walks the list both ways), and so does an input shaper,
/// whose impulse times reach into the segments either side of the one being
/// generated (`kin_shaper.c`'s `calc_position`).
#[derive(Debug, Clone, Copy)]
pub struct SegmentWindow<'a> {
    cuts: &'a [MoveSegment],
    index: usize,
    schedule: &'a [PressureAdvanceEntry],
}

impl<'a> SegmentWindow<'a> {
    /// A window holding only `segment`, for a solver with no pressure advance.
    pub fn single(segment: &'a MoveSegment) -> Self {
        Self {
            cuts: std::slice::from_ref(segment),
            index: 0,
            schedule: &[],
        }
    }

    /// A window over `cuts`, oldest first, with `index` being the segment being
    /// generated.
    pub fn new(cuts: &'a [MoveSegment], index: usize) -> Self {
        Self {
            cuts,
            index,
            schedule: &[],
        }
    }

    /// The segments around the one being generated, oldest first.
    pub fn cuts(&self) -> &'a [MoveSegment] {
        self.cuts
    }

    /// Where the segment being generated sits in [`Self::cuts`].
    pub fn index(&self) -> usize {
        self.index
    }

    /// The segment being generated.
    pub fn segment(&self) -> &'a MoveSegment {
        &self.cuts[self.index]
    }
}

/// One stepper's position function and solver state.
///
/// Upstream's `struct stepper_kinematics` (`chelper/itersolve.h:10-22`), plus
/// the extruder's pressure-advance state (`struct extruder_stepper` wraps the
/// base struct in `chelper/kin_extruder.c:118-122`).
#[derive(Debug, Clone)]
pub struct StepKinematics {
    /// Millimetres per step.
    pub step_dist: f64,
    /// The stepper position the solver has reached, in millimetres.
    pub commanded_pos: f64,
    /// The print time the solver last generated up to.
    pub last_flush_time: f64,
    /// The print time the solver last finished a segment at.
    pub last_move_time: f64,
    /// Which axes this stepper moves.
    pub active_flags: AxisFlags,
    position: PositionFn,
    /// The unwrap applied to each raw position-fn result, against
    /// `commanded_pos` at evaluation time (`itersolve.h`'s stateful callback:
    /// polar's angle solver reads `sk->commanded_pos` while evaluating).
    unwrap: Option<PositionUnwrap>,
    /// The fixup applied to `commanded_pos` after each generated range
    /// (`itersolve.c:124-126`'s `post_cb`).
    post: Option<PositionPost>,
    /// The pressure-advance parameters, when this is an extruder stepper
    /// (`struct extruder_stepper`'s `pa_list`): the position function becomes
    /// the smoothed `pa_position` (`kin_extruder.c:124-136`).
    pressure_advance: Option<Arc<ExtruderPressureAdvance>>,
    /// The input shaper, when this stepper is shaped: the position function
    /// becomes the shaper's convolution of the coordinate it reads
    /// (`struct input_shaper`, `kin_shaper.c:17-23`).
    shaper: Option<Arc<KinematicShaper>>,
    /// The time before the first move this stepper is active in that it also
    /// generates steps in (`gen_steps_pre_active`).
    gen_steps_pre_active: f64,
    /// The time after the last move this stepper is active in that it also
    /// generates steps in (`gen_steps_post_active`).
    gen_steps_post_active: f64,
}

impl StepKinematics {
    /// A kinematic stepper with `step_dist` millimetres per step.
    pub fn new(step_dist: f64, position: PositionFn, active_flags: AxisFlags) -> Self {
        Self {
            step_dist,
            commanded_pos: 0.0,
            last_flush_time: 0.0,
            last_move_time: 0.0,
            active_flags,
            position,
            unwrap: None,
            post: None,
            pressure_advance: None,
            shaper: None,
            gen_steps_pre_active: 0.0,
            gen_steps_post_active: 0.0,
        }
    }

    /// Widen step generation to the `pre_active` seconds before the first move
    /// this stepper is active in and the `post_active` seconds after the last
    /// (`gen_steps_pre_active`/`gen_steps_post_active`).
    ///
    /// A solver whose position function reads only the move it is asked about
    /// (every kinematic solver) leaves both at zero. The pressure-advance
    /// extruder does not set them: its window is the live `hst`, which changes
    /// with `SET_PRESSURE_ADVANCE`, so [`StepKinematics::generate_steps`] derives
    /// it per call instead.
    pub fn set_generation_window(&mut self, pre_active: f64, post_active: f64) {
        self.gen_steps_pre_active = pre_active;
        self.gen_steps_post_active = post_active;
    }

    /// The generation window this stepper was given, in seconds before the
    /// first and after the last move it is active in
    /// (`gen_steps_pre_active`, `gen_steps_post_active`).
    pub fn generation_window(&self) -> (f64, f64) {
        (self.gen_steps_pre_active, self.gen_steps_post_active)
    }

    /// The window this stepper's steps are generated over right now, as
    /// [`StepKinematics::generate_steps`] uses it: the live half smooth time
    /// for a pressure-advance stepper — upstream rewrites both
    /// `gen_steps_pre_active`/`gen_steps_post_active` from it on every
    /// `SET_PRESSURE_ADVANCE` (`kin_extruder.c:144-145`) — else the stored
    /// [`StepKinematics::generation_window`].
    ///
    /// The motion queue's scan pass reads it
    /// ([`ToolHead::recompute_scan_windows`](super::toolhead::ToolHead::recompute_scan_windows)),
    /// so the flush delay is computed from the window generation actually
    /// rewrites.
    pub fn effective_generation_window(&self) -> (f64, f64) {
        match &self.pressure_advance {
            Some(pressure_advance) => {
                let hst = 0.5 * pressure_advance.values().1;
                (hst, hst)
            }
            None => (self.gen_steps_pre_active, self.gen_steps_post_active),
        }
    }

    /// Drive this stepper's position function with pressure advance
    /// (`extruder_set_pressure_advance`, `kin_extruder.c:138-139`): the solver
    /// reads the shared parameters on every evaluation, so a
    /// `SET_PRESSURE_ADVANCE` is seen without reinstalling anything.
    pub fn set_pressure_advance(&mut self, pressure_advance: Arc<ExtruderPressureAdvance>) {
        self.pressure_advance = Some(pressure_advance);
    }

    /// The pressure-advance parameters driving this stepper, when it is an
    /// extruder stepper (`struct extruder_stepper`'s `pa_list` and the shared
    /// parameter block `SET_PRESSURE_ADVANCE` writes).
    pub fn pressure_advance(&self) -> Option<&Arc<ExtruderPressureAdvance>> {
        self.pressure_advance.as_ref()
    }

    /// Shape this stepper's position function (`input_shaper_set_sk` plus
    /// `input_shaper_update_sk`, `kin_shaper.c:226-279`), or take the shaper off
    /// again with `None`.
    ///
    /// The step generation window follows the impulses, as upstream's
    /// `shaper_note_generation_time` recomputes it on every change: the shaped
    /// motion reaches the outermost impulse times past the first and last move
    /// the stepper is active in, and steps there are not final
    /// (`gen_steps_pre_active`/`gen_steps_post_active`). The caller rescans the
    /// queue's windows afterwards
    /// ([`ToolHead::recompute_scan_windows`](super::toolhead::ToolHead::recompute_scan_windows)).
    pub fn set_input_shaper(&mut self, shaper: Option<Arc<KinematicShaper>>) {
        let (pre_active, post_active) = match &shaper {
            Some(shaper) => shaper.generation_window(self.active_flags),
            None => (0.0, 0.0),
        };
        self.set_generation_window(pre_active, post_active);
        self.shaper = shaper;
    }

    /// The shaper driving this stepper, when it is shaped.
    pub fn input_shaper(&self) -> Option<&Arc<KinematicShaper>> {
        self.shaper.as_ref()
    }

    /// Install the solver's position hooks (upstream: the `calc_position_cb`
    /// that reads `sk->commanded_pos` and the `post_cb`; `kin_polar.c` is the
    /// only solver in the corpus that has them).
    pub fn set_hooks(&mut self, unwrap: Option<PositionUnwrap>, post: Option<PositionPost>) {
        self.unwrap = unwrap;
        self.post = post;
    }

    /// One position-fn evaluation: the raw result, unwrapped against the
    /// commanded position (`polar_stepper_angle_calc_position`).
    fn eval(&self, window: SegmentWindow<'_>, move_time: f64) -> f64 {
        let raw = if let Some(pressure_advance) = &self.pressure_advance {
            pressure_advance.position(window.cuts, window.index, move_time, window.schedule)
        } else if let Some(shaper) = &self.shaper {
            shaper.position(window, move_time, self.active_flags, &self.position)
        } else {
            self.position.call(window.segment(), move_time)
        };
        match self.unwrap {
            Some(unwrap) => unwrap(raw, self.commanded_pos),
            None => raw,
        }
    }

    /// Whether this stepper moves during `segment`
    /// (`check_active`, `chelper/itersolve.c:138-145`).
    pub fn is_active(&self, segment: &MoveSegment) -> bool {
        (self.active_flags.intersects(AxisFlags::X) && segment.axes_r.x() != 0.0)
            || (self.active_flags.intersects(AxisFlags::Y) && segment.axes_r.y() != 0.0)
            || (self.active_flags.intersects(AxisFlags::Z) && segment.axes_r.z() != 0.0)
    }

    /// The print time of the first queued move this stepper moves during, or
    /// `None` while there is no such move before `flush_time`
    /// (`itersolve_check_active`, `chelper/itersolve.c:216-231`).
    ///
    /// Only moves the solver has **not** generated yet are considered (its
    /// `last_flush_time` is the horizon it reached): a move that is already
    /// done can no longer need a motor energised, and its steps are on the MCU's
    /// timeline already. The move's **start** time is what upstream reports —
    /// it is the print time the motor's enable is scheduled at, so the driver is
    /// on before the first step of that move is due.
    pub fn check_active(&self, trapq: &Trapq, flush_time: f64) -> Option<f64> {
        for segment in trapq.moves() {
            if self.last_flush_time >= segment.end_time() {
                continue;
            }
            if self.is_active(segment) {
                return Some(segment.print_time);
            }
            if flush_time <= segment.end_time() {
                return None;
            }
        }
        None
    }

    /// The stepper position for a toolhead position
    /// (`itersolve_calc_position_from_coord`): a zero-acceleration move at
    /// `pos`, evaluated in its middle.
    pub fn calc_position_from_coord(&self, pos: Xyz) -> f64 {
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1000.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: pos,
            axes_r: Xyz::default(),
        };
        // `itersolve_calc_position_from_coord` calls the same callback, so it
        // unwraps against `commanded_pos` the same way (no `post_cb` there,
        // as upstream). A one-segment window clips any smoothing request, as
        // the synthetic move has no neighbours to read.
        self.eval(SegmentWindow::single(&segment), 500.0)
    }

    /// Set the stepper position from a toolhead position
    /// (`itersolve_set_position`).
    pub fn set_position(&mut self, pos: Xyz) {
        self.commanded_pos = self.calc_position_from_coord(pos);
    }

    /// The stepper position the solver has reached
    /// (`itersolve_get_commanded_pos`).
    pub fn commanded_pos(&self) -> f64 {
        self.commanded_pos
    }

    /// Generate the steps for `trapq` up to `flush_time`
    /// (`itersolve_generate_steps`, `chelper/itersolve.c:147-208`).
    ///
    /// A stepper only steps while it is active, but its position function may
    /// read the moves around the one it is asked about: pressure advance
    /// averages over `2 * hst = smooth_time` of the trapq (`pa_range_integrate`,
    /// `kin_extruder.c:89-116`), and an input shaper's impulses reach into the
    /// segments either side of the one being generated (`calc_position`,
    /// `kin_shaper.c:120-152`). So the `hst` before its first active move and
    /// after its last one really does move the extruder, and the shaped motion
    /// starts before and ends after the moves an axis is active in. Generating
    /// only inside the active moves would squeeze that motion into the move's
    /// own span — a jump at the start and at the end of every extrusion, and at
    /// both ends of every shaped move. The two windows
    /// (`gen_steps_pre_active`, `gen_steps_post_active`) make the generator
    /// visit the silent moves inside that reach as well.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from the compressor.
    pub fn generate_steps(
        &mut self,
        trapq: &Trapq,
        sc: &mut StepCompressor,
        flush_time: f64,
    ) -> Result<(), StepCompressError> {
        let last_flush_time = self.last_flush_time;
        self.last_flush_time = flush_time;
        if self.step_dist == 0.0 {
            return Ok(());
        }
        // The pressure-advance and shaper position functions read the segments
        // around the one they are asked about, including the ones already
        // finalized into the history (`kin_extruder.c` and `kin_shaper.c` walk
        // the trapq list both ways). Copy them into one slice, oldest first,
        // only when this stepper needs them — and take one snapshot of the
        // pressure-advance parameters, so the position function itself needs no
        // lock.
        let schedule: Vec<PressureAdvanceEntry> = match &self.pressure_advance {
            Some(pressure_advance) => pressure_advance.schedule(),
            None => Vec::new(),
        };
        let reads_neighbouring_moves = self.pressure_advance.is_some() || self.shaper.is_some();
        let cuts: Vec<MoveSegment> = if reads_neighbouring_moves {
            let mut cuts: Vec<MoveSegment> = trapq.history().iter().rev().copied().collect();
            cuts.extend(trapq.moves().iter().copied());
            cuts
        } else {
            Vec::new()
        };
        let first_queued = cuts.len().saturating_sub(trapq.moves().len());
        // Upstream rewrites both fields from the live `hst` on every
        // `SET_PRESSURE_ADVANCE` (`kin_extruder.c:144-145`), so the extruder's
        // window is derived from the live values here rather than cached in the
        // stepper: a change is then in force for the very next flush, and the
        // queue's scan pass reads the same window
        // ([`StepKinematics::effective_generation_window`]).
        let (pre_active, post_active) = self.effective_generation_window();
        let moves = trapq.moves();
        let mut force_steps_time = self.last_move_time + post_active;
        let mut skip_count = 0;
        let mut index = 0;
        while index < moves.len() {
            let segment = &moves[index];
            let move_start = segment.print_time;
            let move_end = segment.end_time();
            if move_end <= last_flush_time {
                // Already generated past this segment (upstream advances `m`
                // over these before the loop).
                index += 1;
                continue;
            }
            if self.is_active(segment) {
                if skip_count > 0 && pre_active > 0.0 {
                    // Steps leading up to stepper activity: from the silent
                    // move that covers `move_start - pre_active` (but no further
                    // back than the moves actually skipped, and never over time
                    // already generated or inside the previous post window).
                    let abs_start = (move_start - pre_active)
                        .max(last_flush_time)
                        .max(force_steps_time);
                    // `skip_count` silent moves precede this one, so `index` is
                    // at least that large and the walk stays inside the queue.
                    let mut first = index - 1;
                    let mut remaining = skip_count - 1;
                    while remaining > 0 && moves[first].print_time > abs_start {
                        first -= 1;
                        remaining -= 1;
                    }
                    for queued in first..index {
                        let window =
                            segment_window(&cuts, &schedule, first_queued, queued, &moves[queued]);
                        self.gen_steps_range(sc, window, abs_start, flush_time)?;
                    }
                }
                // Steps for this move.
                let window = segment_window(&cuts, &schedule, first_queued, index, segment);
                self.gen_steps_range(sc, window, last_flush_time, flush_time)?;
                if move_end >= flush_time {
                    self.last_move_time = flush_time;
                    return Ok(());
                }
                skip_count = 0;
                self.last_move_time = move_end;
                force_steps_time = self.last_move_time + post_active;
            } else {
                if move_start < force_steps_time {
                    // Steps just past stepper activity: the tail of the last
                    // active move's smoothing window.
                    let abs_end = force_steps_time.min(flush_time);
                    let window = segment_window(&cuts, &schedule, first_queued, index, segment);
                    self.gen_steps_range(sc, window, last_flush_time, abs_end)?;
                    skip_count = 1;
                } else {
                    // This move does not concern this stepper.
                    skip_count += 1;
                }
                if flush_time + pre_active <= move_end {
                    return Ok(());
                }
            }
            index += 1;
        }
        self.last_move_time = flush_time;
        Ok(())
    }

    /// Generate the steps inside one segment's `abs_start..abs_end`
    /// (`itersolve_gen_steps_range`, `chelper/itersolve.c:27-124`).
    ///
    /// The time of each step is found with the secant method, falling back to
    /// bisection when a guess lands outside the bracket. `abs_start` and
    /// `abs_end` are absolute print times.
    pub fn gen_steps_range(
        &mut self,
        sc: &mut StepCompressor,
        window: SegmentWindow<'_>,
        abs_start: f64,
        abs_end: f64,
    ) -> Result<(), StepCompressError> {
        let segment = window.segment();
        let half_step = 0.5 * self.step_dist;
        let start = (abs_start - segment.print_time).max(0.0);
        let end = (abs_end - segment.print_time).min(segment.move_t);
        let mut old_guess = TimePosition {
            time: start,
            position: self.commanded_pos,
        };
        let mut guess = old_guess;
        let mut sdir = sc.step_dir();
        let mut is_dir_change = false;
        let mut have_bracket = false;
        let mut check_oscillate = false;
        let mut target = self.commanded_pos + if sdir { half_step } else { -half_step };
        let mut last_time = start;
        let mut low_time = start;
        let mut high_time = (start + SEEK_TIME_RESET).min(end);
        loop {
            // The secant method: extrapolate from the two previous guesses.
            let guess_dist = guess.position - target;
            let old_dist = old_guess.position - target;
            let mut next_time =
                (old_guess.time * guess_dist - guess.time * old_dist) / (guess_dist - old_dist);
            if !(next_time > low_time && next_time < high_time) {
                // The guess left the bracket (or is NaN): validate it.
                if have_bracket {
                    // A poor guess — fall back to bisection.
                    next_time = (low_time + high_time) * 0.5;
                    check_oscillate = false;
                } else if guess.time >= end {
                    // No more steps in the requested range.
                    break;
                } else {
                    // Might be a poor guess — limit to an exponential search.
                    next_time = high_time;
                    high_time = (2.0 * high_time - last_time).min(end);
                }
            }
            old_guess = guess;
            guess = TimePosition {
                time: next_time,
                position: self.eval(window, next_time),
            };
            let guess_dist = guess.position - target;
            if guess_dist.abs() > 0.000_000_001 {
                // Not close enough: update the bracket.
                let rel_dist = if sdir { guess_dist } else { -guess_dist };
                if rel_dist > 0.0 {
                    // Past the target, so this step is definitely present.
                    if have_bracket && old_guess.time <= low_time {
                        if check_oscillate {
                            // Force a bisection next time to stop oscillating.
                            old_guess = guess;
                        }
                        check_oscillate = true;
                    }
                    high_time = guess.time;
                    have_bracket = true;
                } else if rel_dist < -(half_step + half_step + 0.000_000_010) {
                    // A direction change.
                    sdir = !sdir;
                    target = if sdir {
                        target + half_step + half_step
                    } else {
                        target - half_step - half_step
                    };
                    low_time = last_time;
                    high_time = guess.time;
                    is_dir_change = true;
                    have_bracket = true;
                    check_oscillate = false;
                } else {
                    low_time = guess.time;
                }
                if !have_bracket || high_time - low_time > 0.000_000_001 {
                    if !is_dir_change && rel_dist >= -half_step {
                        // The stepper fully reaches the step position, so the
                        // step can no longer be rolled back.
                        sc.commit()?;
                    }
                    continue;
                }
            }
            // Found the next step.
            sc.append(sdir, segment.print_time, guess.time)?;
            target = if sdir {
                target + half_step + half_step
            } else {
                target - half_step - half_step
            };
            let mut seek_time_delta = 1.5 * (guess.time - last_time);
            if seek_time_delta < 0.000_000_001 {
                seek_time_delta = 0.000_000_001;
            }
            if is_dir_change && seek_time_delta > SEEK_TIME_RESET {
                seek_time_delta = SEEK_TIME_RESET;
            }
            last_time = guess.time;
            low_time = guess.time;
            high_time = (guess.time + seek_time_delta).min(end);
            is_dir_change = false;
            have_bracket = false;
            check_oscillate = false;
        }
        self.commanded_pos = target - if sdir { half_step } else { -half_step };
        // Upstream calls `sk->post_cb` right after this assignment
        // (`itersolve.c:124-126`): polar renormalizes the bed angle into
        // [-π, π] so the next range's single ±2π unwrap stays sufficient.
        if let Some(post) = self.post {
            post(&mut self.commanded_pos);
        }
        Ok(())
    }
}

/// The window a stepper reads while generating the range of queued move
/// `index` (an index into [`Trapq::moves`]).
///
/// `cuts` is the trapq neighbourhood the pressure-advance integrator needs,
/// oldest first, with `first_queued` being where the queued moves start in it —
/// it is empty for a solver that has no pressure advance, which reads only the
/// segment itself.
fn segment_window<'a>(
    cuts: &'a [MoveSegment],
    schedule: &'a [PressureAdvanceEntry],
    first_queued: usize,
    index: usize,
    segment: &'a MoveSegment,
) -> SegmentWindow<'a> {
    if cuts.is_empty() {
        SegmentWindow::single(segment)
    } else {
        SegmentWindow {
            cuts,
            index: first_queued + index,
            schedule,
        }
    }
}

/// A time and the stepper position there (upstream's `struct timepos`).
#[derive(Debug, Clone, Copy)]
struct TimePosition {
    time: f64,
    position: f64,
}

/// The three cartesian axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    /// X.
    X,
    /// Y.
    Y,
    /// Z.
    Z,
}

/// The position function for one cartesian axis
/// (`cartesian_stepper_alloc`, `chelper/kin_cartesian.c:39-56`): read that
/// axis' coordinate.
pub fn cartesian_position_fn(axis: Axis) -> PositionFn {
    const NO_PARAMS: [f64; 6] = [0.0; 6];
    match axis {
        Axis::X => PositionFn::bind(
            |segment, move_time, _| segment.coord(move_time).x(),
            NO_PARAMS,
        ),
        Axis::Y => PositionFn::bind(
            |segment, move_time, _| segment.coord(move_time).y(),
            NO_PARAMS,
        ),
        Axis::Z => PositionFn::bind(
            |segment, move_time, _| segment.coord(move_time).z(),
            NO_PARAMS,
        ),
    }
}

/// The active-axis flags for one cartesian axis.
pub fn cartesian_active_flags(axis: Axis) -> AxisFlags {
    match axis {
        Axis::X => AxisFlags::X,
        Axis::Y => AxisFlags::Y,
        Axis::Z => AxisFlags::Z,
    }
}

/// The position function for one CoreXY motor (`corexy_stepper_alloc`,
/// `chelper/kin_corexy.c`): `+` follows `x + y`, `-` follows `x - y`.
pub fn corexy_position_fn(plus: bool) -> PositionFn {
    const NO_PARAMS: [f64; 6] = [0.0; 6];
    if plus {
        PositionFn::bind(
            |segment, move_time, _| {
                let c = segment.coord(move_time);
                c.x() + c.y()
            },
            NO_PARAMS,
        )
    } else {
        PositionFn::bind(
            |segment, move_time, _| {
                let c = segment.coord(move_time);
                c.x() - c.y()
            },
            NO_PARAMS,
        )
    }
}

/// The active flags for a CoreXY motor: it moves when X or Y does
/// (`corexy_stepper_alloc`'s `AF_X | AF_Y`).
pub fn corexy_active_flags() -> AxisFlags {
    AxisFlags::X.union(AxisFlags::Y)
}

/// The position function for one CoreXZ motor (`corexz_stepper_alloc`,
/// `chelper/kin_corexz.c`): `+` follows `x + z`, `-` follows `x - z`.
pub fn corexz_position_fn(plus: bool) -> PositionFn {
    const NO_PARAMS: [f64; 6] = [0.0; 6];
    if plus {
        PositionFn::bind(
            |segment, move_time, _| {
                let c = segment.coord(move_time);
                c.x() + c.z()
            },
            NO_PARAMS,
        )
    } else {
        PositionFn::bind(
            |segment, move_time, _| {
                let c = segment.coord(move_time);
                c.x() - c.z()
            },
            NO_PARAMS,
        )
    }
}

/// The active flags for a CoreXZ motor (`AF_X | AF_Z`).
pub fn corexz_active_flags() -> AxisFlags {
    AxisFlags::X.union(AxisFlags::Z)
}

/// The position function for an extruder motor (`extruder_stepper_alloc`,
/// `chelper/kin_extruder.c`): read the trapq's x, which the extruder's trapq
/// holds the extrusion amount in.
pub fn extruder_position_fn() -> PositionFn {
    PositionFn::bind(
        |segment, move_time, _| segment.coord(move_time).x(),
        [0.0; 6],
    )
}

/// The active flags for an extruder motor (`AF_X`, because it reads x).
pub fn extruder_active_flags() -> AxisFlags {
    AxisFlags::X
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::motion::shaper::ShaperPulses;
    use crate::core::klippy::motion::stepcompress::StepCommand;

    /// A constant-velocity move along X covering `distance` at `speed`.
    fn x_move(speed: f64, distance: f64) -> MoveSegment {
        MoveSegment {
            print_time: 0.0,
            move_t: distance / speed,
            start_v: speed,
            half_accel: 0.0,
            start_pos: Xyz::new(0.0, 0.0, 0.0),
            axes_r: Xyz::new(1.0, 0.0, 0.0),
        }
    }

    /// The total number of steps the commands describe.
    fn total_steps(commands: &[StepCommand]) -> u32 {
        commands
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum()
    }

    #[test]
    fn test_cartesian_reads_its_axis() {
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1.0,
            start_v: 10.0,
            half_accel: 0.0,
            start_pos: Xyz::new(1.0, 2.0, 3.0),
            axes_r: Xyz::new(1.0, 0.0, 0.0),
        };

        // 0.5 s at 10 mm/s along X, starting at (1, 2, 3).
        assert_eq!(cartesian_position_fn(Axis::X).call(&segment, 0.5), 6.0);
        assert_eq!(cartesian_position_fn(Axis::Y).call(&segment, 0.5), 2.0);
        assert_eq!(cartesian_position_fn(Axis::Z).call(&segment, 0.5), 3.0);
    }

    #[test]
    fn test_corexy_reads_the_x_y_sum_and_difference() {
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1.0,
            start_v: 10.0,
            half_accel: 0.0,
            start_pos: Xyz::new(1.0, 2.0, 3.0),
            axes_r: Xyz::new(1.0, 0.0, 0.0),
        };
        // x is 6 and y is 2 at 0.5 s, so the two motors read 8 and 4.
        assert_eq!(corexy_position_fn(true).call(&segment, 0.5), 8.0);
        assert_eq!(corexy_position_fn(false).call(&segment, 0.5), 4.0);
        assert_eq!(corexy_active_flags(), AxisFlags::X.union(AxisFlags::Y));
    }

    #[test]
    fn test_corexz_reads_the_x_z_sum_and_difference() {
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1.0,
            start_v: 10.0,
            half_accel: 0.0,
            start_pos: Xyz::new(1.0, 2.0, 3.0),
            axes_r: Xyz::new(0.0, 0.0, 1.0),
        };
        // x is 1 and z is 8 at 0.5 s, so the two motors read 9 and -7.
        assert_eq!(corexz_position_fn(true).call(&segment, 0.5), 9.0);
        assert_eq!(corexz_position_fn(false).call(&segment, 0.5), -7.0);
        assert_eq!(corexz_active_flags(), AxisFlags::X.union(AxisFlags::Z));
    }

    #[test]
    fn test_a_stepper_that_does_not_move_on_an_axis_is_inactive() {
        let sk = StepKinematics::new(1.0, cartesian_position_fn(Axis::Z), AxisFlags::Z);

        // The move is along X only.
        assert!(!sk.is_active(&x_move(100.0, 10.0)));
    }

    #[test]
    fn test_steps_are_generated_at_the_step_distance() {
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        let mut sk = StepKinematics::new(1.0, cartesian_position_fn(Axis::X), AxisFlags::X);
        let segment = x_move(100.0, 100.0);

        sk.gen_steps_range(
            &mut sc,
            SegmentWindow::single(&segment),
            0.0,
            segment.move_t,
        )
        .unwrap();
        // The compressor holds the steps in its queue until a flush.
        sc.flush(u64::MAX).unwrap();

        // Steps at 0.5, 1.5, … 99.5 mm — 100 of them, compressed into a few
        // `queue_step` commands.
        let commands = sc.take_commands();
        assert_eq!(total_steps(&commands), 100);
        assert!(
            commands.len() <= 8,
            "the full compressor should compress: {commands:?}"
        );
        // The solver ends having commanded the whole distance.
        assert!((sk.commanded_pos() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_generate_steps_walks_the_trapq() {
        let mut trapq = Trapq::new();
        // Two 10 mm moves at 100 mm/s, back to back.
        trapq.append(
            0.0,
            0.0,
            0.1,
            0.0,
            Xyz::new(0.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            100.0,
            100.0,
            0.0,
        );
        trapq.append(
            0.1,
            0.0,
            0.1,
            0.0,
            Xyz::new(10.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            100.0,
            100.0,
            0.0,
        );
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        let mut sk = StepKinematics::new(1.0, cartesian_position_fn(Axis::X), AxisFlags::X);

        sk.generate_steps(&trapq, &mut sc, 0.2).unwrap();
        sc.flush(u64::MAX).unwrap();

        // 20 mm of travel at 1 mm per step.
        assert_eq!(total_steps(&sc.take_commands()), 20);
    }

    /// A shaper on the x axis, built as the `[input_shaper]` layer builds it.
    fn x_shaper(a: &[f64], t: &[f64]) -> Arc<KinematicShaper> {
        Arc::new(KinematicShaper::new([
            Some(ShaperPulses::new(a, t).expect("impulses")),
            None,
            None,
        ]))
    }

    /// Two x moves: 10 mm at 100 mm/s, then 10 mm at 50 mm/s. The velocity step
    /// in the middle is what a shaper acts on.
    fn x_trapq() -> Trapq {
        let mut trapq = Trapq::new();
        trapq.append(
            0.0,
            0.0,
            0.1,
            0.0,
            Xyz::new(0.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            100.0,
            100.0,
            0.0,
        );
        trapq.append(
            0.1,
            0.0,
            0.2,
            0.0,
            Xyz::new(10.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            50.0,
            50.0,
            0.0,
        );
        trapq
    }

    /// The step commands `sk` generates for all of `trapq`.
    fn steps_of(sk: &mut StepKinematics, trapq: &Trapq) -> Vec<StepCommand> {
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        sk.generate_steps(trapq, &mut sc, 0.3).unwrap();
        sc.flush(u64::MAX).unwrap();
        sc.take_commands()
    }

    /// The plain x solver.
    fn plain_x_solver() -> StepKinematics {
        StepKinematics::new(0.01, cartesian_position_fn(Axis::X), AxisFlags::X)
    }

    /// No shaping is no change: a stepper without a shaper reads its plain
    /// position function point for point, and a stepper a shaper was taken off
    /// generates the same steps as one that never carried one. This is the
    /// regression guard for `shaper_type: none` and for an axis whose
    /// `shaper_freq` is 0 — the extras layer installs nothing there.
    #[test]
    fn test_without_a_shaper_nothing_changes() {
        let trapq = x_trapq();
        let cuts: Vec<MoveSegment> = trapq.moves().iter().copied().collect();
        let plain = plain_x_solver();
        let mut cleared = plain_x_solver();
        cleared.set_input_shaper(Some(x_shaper(&[0.5, 0.5], &[0.0, 0.02])));
        cleared.set_input_shaper(None);
        assert_eq!(cleared.generation_window(), (0.0, 0.0));

        for (index, segment) in cuts.iter().enumerate() {
            for sample in 0..=20 {
                let move_time = segment.move_t * f64::from(sample) / 20.0;
                let window = SegmentWindow::new(&cuts, index);
                assert_eq!(
                    plain.eval(window, move_time),
                    segment.coord(move_time).x(),
                    "the plain solver reads its axis: segment {index} at {move_time}"
                );
                assert_eq!(
                    cleared.eval(window, move_time),
                    plain.eval(window, move_time),
                    "a cleared shaper leaves no trace: segment {index} at {move_time}"
                );
            }
        }

        assert_eq!(
            steps_of(&mut plain_x_solver(), &trapq),
            steps_of(&mut cleared, &trapq)
        );
    }

    /// A single impulse weighting 1 at time 0 is the identity transformation
    /// (`shift_pulses` leaves it where it is): the shaped position is the plain
    /// one point for point, and so are the steps.
    #[test]
    fn test_a_single_impulse_at_time_zero_is_the_identity() {
        let trapq = x_trapq();
        let cuts: Vec<MoveSegment> = trapq.moves().iter().copied().collect();
        let mut shaped = plain_x_solver();
        shaped.set_input_shaper(Some(x_shaper(&[1.0], &[0.0])));
        assert_eq!(shaped.generation_window(), (0.0, 0.0));

        for (index, segment) in cuts.iter().enumerate() {
            for sample in 0..=20 {
                let move_time = segment.move_t * f64::from(sample) / 20.0;
                let window = SegmentWindow::new(&cuts, index);
                assert!(
                    (shaped.eval(window, move_time) - segment.coord(move_time).x()).abs() < 1e-9,
                    "segment {index} at {move_time}"
                );
            }
        }

        assert_eq!(
            steps_of(&mut shaped, &trapq),
            steps_of(&mut plain_x_solver(), &trapq)
        );
    }

    /// A two-impulse shaper does reach the steps: at the velocity step the
    /// position is the weighted sum of two delayed positions, so the generated
    /// steps cannot be the plain ones.
    #[test]
    fn test_a_two_impulse_shaper_moves_the_steps() {
        let trapq = x_trapq();
        let cuts: Vec<MoveSegment> = trapq.moves().iter().copied().collect();
        let mut shaped = plain_x_solver();
        shaped.set_input_shaper(Some(x_shaper(&[0.8, 0.2], &[0.02, 0.05])));
        // The impulses reach 6 ms ahead and 24 ms back of the shaped moment.
        let (pre_active, post_active) = shaped.generation_window();
        assert!((pre_active - 0.006).abs() < 1e-12, "{pre_active}");
        assert!((post_active - 0.024).abs() < 1e-12, "{post_active}");

        let shaped_steps = steps_of(&mut shaped, &trapq);
        let plain_steps = steps_of(&mut plain_x_solver(), &trapq);

        assert_ne!(shaped_steps, plain_steps);
        // The solver really stepped the shaped positions: it starts where the
        // shaper reaches (the impulses that would sample before the trapq are
        // clipped to its first position) and stops where it ends, so it travels
        // less than the plain 20 mm.
        // `cuts[0]` is the stationary filler the trapq puts before the first
        // move; the motion starts on `cuts[1]`.
        let start = shaped.eval(SegmentWindow::new(&cuts, 1), 0.0);
        let last = &cuts[cuts.len() - 1];
        let end = shaped.eval(SegmentWindow::new(&cuts, cuts.len() - 1), last.move_t);
        assert!((start - 0.48).abs() < 1e-9, "the shaped start is {start}");
        assert!((end - 19.76).abs() < 1e-9, "the shaped end is {end}");
        let travel = f64::from(total_steps(&shaped_steps));
        assert!(
            (travel - (end - start) / 0.01).abs() <= 1.0,
            "{travel} steps is not the shaped travel"
        );
        assert!(f64::from(total_steps(&plain_steps)) - travel > 20.0);
    }

    #[test]
    fn test_position_round_trips_through_a_coordinate() {
        let mut sk = StepKinematics::new(1.0, cartesian_position_fn(Axis::Y), AxisFlags::Y);

        sk.set_position(Xyz::new(1.0, 2.0, 3.0));

        assert_eq!(sk.commanded_pos(), 2.0);
    }

    /// An extruder trapq: `from..to` seconds of constant `speed` mm/s
    /// extrusion, in segments so a `SET_PRESSURE_ADVANCE` can land between
    /// them. The `y` direction cosine marks a segment as able to advance
    /// (`can_pressure_advance`, `extruder.py:221-232`); a trapq has to start
    /// with one that cannot, or the pressure advance would be on at the very
    /// first step — a jump no solver could step smoothly.
    fn extruder_trapq(segments: &[(f64, f64, bool)]) -> Trapq {
        let mut trapq = Trapq::new();
        let mut position = 0.0;
        for (print_time, duration, can_advance) in segments {
            trapq.append(
                *print_time,
                0.0,
                *duration,
                0.0,
                Xyz::new(position, 0.0, 0.0),
                Xyz::new(1.0, f64::from(u8::from(*can_advance)), 0.0),
                10.0,
                10.0,
                0.0,
            );
            position += 10.0 * duration;
        }
        trapq
    }

    /// The extruder solver with pressure advance drives the steps: at a
    /// constant extrusion rate it leads the nominal position by
    /// `pressure_advance * velocity`, both from the configured value and from
    /// one `SET_PRESSURE_ADVANCE` writes after the solver was built.
    #[test]
    fn test_pressure_advance_drives_the_extruder_steps() {
        // Constant 10 mm/s for 0.3 s: a 0.1 s stretch that cannot advance,
        // then two that can.
        let trapq = extruder_trapq(&[(0.0, 0.1, false), (0.1, 0.1, true), (0.2, 0.1, true)]);
        let solver = || StepKinematics::new(0.01, extruder_position_fn(), extruder_active_flags());

        // No pressure advance gets the nominal millimetres.
        let mut sk = solver();
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        sk.set_pressure_advance(ExtruderPressureAdvance::new(0.0, 0.040));
        sk.generate_steps(&trapq, &mut sc, 0.1).unwrap();
        sc.flush(u64::MAX).unwrap();
        assert!(
            (sk.commanded_pos() - 1.0).abs() < 0.02,
            "expected the nominal millimetre, got {}",
            sk.commanded_pos()
        );

        // pa 0.05 at 10 mm/s leads by 0.5 mm.
        let mut sk = solver();
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        sk.set_pressure_advance(ExtruderPressureAdvance::new(0.05, 0.040));
        sk.generate_steps(&trapq, &mut sc, 0.2).unwrap();
        sc.flush(u64::MAX).unwrap();
        assert!(
            (sk.commanded_pos() - 2.5).abs() < 0.02,
            "expected 2 mm nominal + the 0.5 mm lead, got {}",
            sk.commanded_pos()
        );

        // The same solver with the value changed after the first flush: the
        // segment that starts after the written print time uses it
        // (`STATE` = 0.15 s lands between two segments).
        let mut sk = solver();
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        let state = ExtruderPressureAdvance::new(0.0, 0.040);
        sk.set_pressure_advance(Arc::clone(&state));
        sk.generate_steps(&trapq, &mut sc, 0.1).unwrap();
        assert!((sk.commanded_pos() - 1.0).abs() < 0.02);
        state.set(0.15, 0.05, 0.040);
        sk.generate_steps(&trapq, &mut sc, 0.25).unwrap();
        sc.flush(u64::MAX).unwrap();
        assert!(
            (sk.commanded_pos() - 3.0).abs() < 0.02,
            "expected 2.5 mm nominal + the 0.5 mm lead, got {}",
            sk.commanded_pos()
        );
    }

    /// The absolute print times of the steps in `commands`, reconstructed from
    /// the compressor's `queue_step` runs at `mcu_freq` ticks per second.
    ///
    /// `clock` carries the last step clock from batch to batch (each run's first
    /// step is `interval` ticks after the previous step), and the compressor
    /// starts at clock zero for print time zero, so clocks are print times times
    /// `mcu_freq` (within the compressor's `MAX_STEPCOMPRESS_ERROR`).
    fn step_print_times(commands: &[StepCommand], mcu_freq: f64, clock: &mut u64) -> Vec<f64> {
        let mut times = Vec::new();
        for command in commands {
            let StepCommand::QueueStep {
                interval,
                count,
                add,
                ..
            } = command
            else {
                continue;
            };
            let mut ticks = i64::from(*interval);
            for _ in 0..*count {
                *clock += ticks as u64;
                times.push(*clock as f64 / mcu_freq);
                ticks += i64::from(*add);
            }
        }
        times
    }

    /// An extruder trapq of raw segments `(print_time, duration, active)`,
    /// oldest first: an active one extrudes at 10 mm/s and carries both
    /// `axes_r` components (so pressure advance applies to it), an inactive one
    /// holds the extrusion amount with `axes_r` all zero — the shape of the null
    /// move a time gap gets (`trapq_add_move`, `chelper/trapq.c:100-117`), in
    /// which the extruder stepper is not active.
    fn window_trapq(segments: &[(f64, f64, bool)]) -> Trapq {
        let mut trapq = Trapq::new();
        let mut position = 0.0;
        for (print_time, duration, active) in segments {
            let speed = if *active { 10.0 } else { 0.0 };
            let axis = if *active { 1.0 } else { 0.0 };
            trapq.append(
                *print_time,
                0.0,
                *duration,
                0.0,
                Xyz::new(position, 0.0, 0.0),
                Xyz::new(axis, axis, 0.0),
                speed,
                speed,
                0.0,
            );
            position += speed * duration;
        }
        trapq
    }

    /// The step print times of an extruder solver over `trapq`, with `pa` and
    /// `smooth_time` in force from the start, generated up to `flush_time`.
    fn extruder_step_times(
        trapq: &Trapq,
        pressure_advance: f64,
        smooth_time: f64,
        flush_time: f64,
    ) -> Vec<f64> {
        let mut sk = StepKinematics::new(0.01, extruder_position_fn(), extruder_active_flags());
        sk.set_pressure_advance(ExtruderPressureAdvance::new(pressure_advance, smooth_time));
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        sk.generate_steps(trapq, &mut sc, flush_time).unwrap();
        sc.flush(u64::MAX).unwrap();
        let mut clock = 0;
        step_print_times(&sc.take_commands(), 1_000_000.0, &mut clock)
    }

    /// The `post_active` window: the smoothing window keeps moving the extruder
    /// for `hst` after the last move it is active in, so steps are due there too.
    /// Without the window that tail is never generated — the extruder would jump
    /// back to the nominal position at the end of the extrusion.
    #[test]
    fn test_pressure_advance_generates_the_post_active_window() {
        // 0.1 s of extrusion at 10 mm/s, then 0.1 s of a silent gap.
        // hst = 0.02 s, so steps are due up to 0.12 s.
        let trapq = window_trapq(&[(0.0, 0.1, true), (0.1, 0.1, false)]);

        let times = extruder_step_times(&trapq, 0.05, 0.040, 0.2);
        assert!(
            times.iter().any(|time| *time > 0.1),
            "the post window must carry steps past the last active move: {times:?}"
        );

        // Pressure advance off: `hst` is zero, so there is no window and the
        // gap after 0.1 s must stay silent.
        let times = extruder_step_times(&trapq, 0.05, 0.0, 0.2);
        assert!(
            times.iter().all(|time| *time <= 0.1),
            "no window means no steps after the last active move: {times:?}"
        );
    }

    /// The `pre_active` window: the smoothing window already moves the extruder
    /// in the `hst` before the first move it is active in, so the first step is
    /// due there rather than at the move's start.
    #[test]
    fn test_pressure_advance_generates_the_pre_active_window() {
        // 0.1 s of silence, then 0.1 s of extrusion at 10 mm/s. hst = 0.02 s, so
        // the smoothing window reaches back to 0.08 s.
        let trapq = window_trapq(&[(0.0, 0.1, false), (0.1, 0.1, true)]);

        let times = extruder_step_times(&trapq, 0.05, 0.040, 0.15);
        let first = *times.first().expect("the window must generate steps");
        assert!(
            first < 0.1,
            "the pre window must start the steps before the move at 0.1 s: {first}"
        );
        assert!(
            first > 0.08 - 0.01,
            "the window is hst wide, so the first step cannot be earlier: {first}"
        );

        // Pressure advance off: no window, so nothing before the move.
        let times = extruder_step_times(&trapq, 0.05, 0.0, 0.15);
        let first = *times.first().expect("the move itself must generate steps");
        assert!(
            first >= 0.1,
            "no window means no steps before the first active move: {first}"
        );
    }

    /// Two flushes of one solver carry on where the previous one stopped: the
    /// windows must not re-generate the range already sent, and no step may land
    /// before an earlier one.
    #[test]
    fn test_generate_steps_windows_are_idempotent() {
        let trapq = window_trapq(&[(0.0, 0.1, true), (0.1, 0.1, false)]);
        let mut sk = StepKinematics::new(0.01, extruder_position_fn(), extruder_active_flags());
        sk.set_pressure_advance(ExtruderPressureAdvance::new(0.05, 0.040));
        let mut sc = StepCompressor::new(0, 1_000_000.0);
        let mut clock = 0;

        sk.generate_steps(&trapq, &mut sc, 0.1).unwrap();
        sc.flush(u64::MAX).unwrap();
        let mut times = step_print_times(&sc.take_commands(), 1_000_000.0, &mut clock);
        let first_batch = times.len();
        assert!(first_batch > 0, "the first flush must generate steps");

        sk.generate_steps(&trapq, &mut sc, 0.2).unwrap();
        sc.flush(u64::MAX).unwrap();
        let second = step_print_times(&sc.take_commands(), 1_000_000.0, &mut clock);
        assert!(!second.is_empty(), "the second flush must add the tail");
        assert!(
            second[0] >= 0.1 - 1e-3,
            "the second flush must not repeat the first: {:?}",
            &second[..second.len().min(5)]
        );
        times.extend(second);
        assert!(
            times.windows(2).all(|pair| pair[1] > pair[0]),
            "steps must neither repeat nor go backwards: {times:?}"
        );
    }
}
