//! The host-side motion planner.
//!
//! Upstream puts this in `klippy/toolhead.py`: a [`Move`] carries the
//! trapezoidal velocity profile of one requested move, and a
//! [`LookAheadQueue`] looks across consecutive moves to decide how fast each
//! junction may be crossed. Nothing here talks to an MCU — the result is a
//! sequence of moves with `accel_t`/`cruise_t`/`decel_t`, which the
//! [`trapq`](super::trapq) turns into step times.
//!
//! The formulas follow upstream (`klippy/toolhead.py:14-193`) line for line, so
//! a move planned here has the same timing as one planned by Klipper. The
//! extra-axes hooks (`Move.calc_junction`'s `extra_axes`, the extruder) are not
//! wired in yet — they arrive with `[extruder]` in FW5e.

use std::sync::Arc;

use super::extra::ExtraAxis;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, E_AXIS};

/// How long the look-ahead accumulates moves before flushing, in seconds
/// (`LOOKAHEAD_FLUSH_TIME`, `klippy/toolhead.py:116`).
pub const LOOKAHEAD_FLUSH_TIME: f64 = 0.150;

/// The toolhead limits a planner needs to profile one move.
///
/// Upstream reads these off the toolhead object (`klippy/toolhead.py:209-216`);
/// they are passed in here so a [`Move`] can be planned — and tested — without
/// a toolhead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveLimits {
    /// Maximum velocity, mm/s.
    pub max_velocity: f64,
    /// Maximum acceleration, mm/s^2.
    pub max_accel: f64,
    /// The corner deviation beyond which junction speed is limited, mm.
    pub junction_deviation: f64,
    /// The pseudo-acceleration `minimum_cruise_ratio` is enforced with, mm/s^2.
    pub mcr_pseudo_accel: f64,
}

impl MoveLimits {
    /// The planner limits the four values upstream's `ToolHead` stores derive
    /// (`ToolHead._calc_junction_deviation`, `klippy/toolhead.py:534-537`):
    /// `junction_deviation` is the corner deviation the junction speed is
    /// limited by, `mcr_pseudo_accel` the pseudo-acceleration
    /// `minimum_cruise_ratio` is enforced with.
    ///
    /// Every producer of a `MoveLimits` goes through here, so the formula has
    /// one home — next to the fields the planner reads it into ([`Move::new`])
    /// and this setter rebuilds at runtime. `max_accel` is an `above=0.`
    /// option, so the division is safe.
    pub fn from_velocity_limits(
        max_velocity: f64,
        max_accel: f64,
        square_corner_velocity: f64,
        min_cruise_ratio: f64,
    ) -> Self {
        Self {
            max_velocity,
            max_accel,
            junction_deviation: square_corner_velocity.powi(2) * (std::f64::consts::SQRT_2 - 1.0)
                / max_accel,
            mcr_pseudo_accel: max_accel * (1.0 - min_cruise_ratio),
        }
    }
}

/// One requested move and its trapezoidal velocity profile.
///
/// Distances are `_d`, velocities `_v`, velocities squared `_v2`, times `_t`
/// and ratios `_r`, as upstream's naming convention
/// (`klippy/toolhead.py:9-11`) has it.
pub struct Move {
    /// Where the move starts.
    pub start_pos: Coord,
    /// Where it ends (after the extrude-only adjustment).
    pub end_pos: Coord,
    /// Per-axis distance.
    pub axes_d: [f64; 4],
    /// Per-axis distance over the total distance (the direction cosines).
    pub axes_r: [f64; 4],
    /// The total distance.
    pub move_d: f64,
    /// The acceleration used for this move, mm/s^2.
    pub accel: f64,
    /// The corner deviation this move is limited by.
    pub junction_deviation: f64,
    /// Whether this move moves the toolhead (an extrude-only move does not).
    pub is_kinematic_move: bool,
    /// The highest speed the junction *before* this move may be entered at.
    pub max_start_v2: f64,
    /// The highest speed this move may cruise at.
    pub max_cruise_v2: f64,
    /// The shortest time this move can take (at `max_cruise_v2`).
    pub min_move_t: f64,
    /// How much squared velocity this move can add.
    pub delta_v2: f64,
    /// An explicit cap on the junction after this move.
    pub next_junction_v2: f64,
    /// The junction cap `minimum_cruise_ratio` imposes.
    pub max_mcr_start_v2: f64,
    /// The squared-velocity budget `minimum_cruise_ratio` allows.
    pub mcr_delta_v2: f64,
    /// The velocity the move starts at (filled by [`Move::set_junction`]).
    pub start_v: f64,
    /// The velocity it cruises at.
    pub cruise_v: f64,
    /// The velocity it ends at.
    pub end_v: f64,
    /// Time spent accelerating.
    pub accel_t: f64,
    /// Time spent cruising.
    pub cruise_t: f64,
    /// Time spent decelerating.
    pub decel_t: f64,
    /// Callbacks to run when the move is queued, with its end print time.
    pub timing_callbacks: Vec<Box<dyn FnOnce(f64) + Send>>,
}

impl Move {
    /// Profile one requested move.
    ///
    /// `start_pos`/`end_pos` are four-axis toolhead positions; `speed` is the
    /// requested speed in mm/s, capped at [`MoveLimits::max_velocity`]
    /// (`klippy/toolhead.py:15-51`).
    pub fn new(start_pos: Coord, end_pos: Coord, speed: f64, limits: &MoveLimits) -> Self {
        let mut axes_d = [0.0; 4];
        for (axis, slot) in axes_d.iter_mut().enumerate() {
            *slot = end_pos[axis] - start_pos[axis];
        }
        let mut end_pos = end_pos;
        let mut accel = limits.max_accel;
        let mut velocity = speed.min(limits.max_velocity);
        let mut is_kinematic_move = true;
        let mut move_d =
            (axes_d[0] * axes_d[0] + axes_d[1] * axes_d[1] + axes_d[2] * axes_d[2]).sqrt();
        if move_d < 0.000_000_001 {
            // An extrude-only move: nothing moves in space, so it gets the
            // extruder's own distance and an effectively unbounded
            // acceleration (an extruder has no kinematic limits).
            end_pos = Coord::new(start_pos.x(), start_pos.y(), start_pos.z(), end_pos.e());
            axes_d[0] = 0.0;
            axes_d[1] = 0.0;
            axes_d[2] = 0.0;
            move_d = axes_d[E_AXIS].abs();
            accel = 99999999.9;
            velocity = speed;
            is_kinematic_move = false;
        }
        let inv_move_d = if move_d != 0.0 { 1.0 / move_d } else { 0.0 };
        let mut axes_r = [0.0; 4];
        for (axis, slot) in axes_r.iter_mut().enumerate() {
            *slot = axes_d[axis] * inv_move_d;
        }
        Self {
            start_pos,
            end_pos,
            axes_d,
            axes_r,
            move_d,
            accel,
            junction_deviation: limits.junction_deviation,
            is_kinematic_move,
            max_start_v2: 0.0,
            max_cruise_v2: velocity * velocity,
            min_move_t: move_d / velocity,
            // Squared-velocity deltas: v2 = 2*a*d.
            delta_v2: 2.0 * move_d * accel,
            next_junction_v2: 999999999.9,
            max_mcr_start_v2: 0.0,
            mcr_delta_v2: 2.0 * move_d * limits.mcr_pseudo_accel,
            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
            accel_t: 0.0,
            cruise_t: 0.0,
            decel_t: 0.0,
            timing_callbacks: Vec::new(),
        }
    }

    /// Lower this move's speed and/or acceleration
    /// (`klippy/toolhead.py:52-59`). A kinematics uses this to respect a slow
    /// axis, the way cartesian slows Z.
    pub fn limit_speed(&mut self, speed: f64, accel: f64) {
        let speed2 = speed * speed;
        if speed2 < self.max_cruise_v2 {
            self.max_cruise_v2 = speed2;
            self.min_move_t = self.move_d / speed;
        }
        self.accel = self.accel.min(accel);
        self.delta_v2 = 2.0 * self.move_d * self.accel;
        self.mcr_delta_v2 = self.mcr_delta_v2.min(self.delta_v2);
    }

    /// Cap the speed the junction *after* this move may be crossed at
    /// (`klippy/toolhead.py:60`).
    pub fn limit_next_junction_speed(&mut self, speed: f64) {
        self.next_junction_v2 = self.next_junction_v2.min(speed * speed);
    }

    /// The error to report when the move is rejected
    /// (`klippy/toolhead.py:62-65`).
    pub fn move_error(&self, msg: &str) -> CommandError {
        let ep = self.end_pos;
        CommandError::new(format!(
            "{}: {:.3} {:.3} {:.3} [{:.3}]",
            msg,
            ep.x(),
            ep.y(),
            ep.z(),
            ep.e()
        ))
    }

    /// Work out how fast the junction before this move may be crossed.
    ///
    /// `prev` is the move immediately before it (`klippy/toolhead.py:66-99`).
    /// The junction speed is limited by both moves' cruise caps, by how much
    /// squared velocity the previous move can add, and by the "approximated
    /// centripetal velocity" the corner deviation allows. `extra_v2` is the
    /// extra axes' own limits (the extruder's `calc_junction`), each already
    /// squared.
    pub fn calc_junction(&mut self, prev: &Move, extra_v2: &[f64]) {
        if !self.is_kinematic_move || !prev.is_kinematic_move {
            return;
        }
        let mut max_start_v2 = [
            self.max_cruise_v2,
            prev.max_cruise_v2,
            prev.next_junction_v2,
            prev.max_start_v2 + prev.delta_v2,
        ]
        .into_iter()
        .fold(f64::INFINITY, f64::min);
        for v2 in extra_v2 {
            max_start_v2 = max_start_v2.min(*v2);
        }
        let axes_r = &self.axes_r;
        let prev_axes_r = &prev.axes_r;
        let junction_cos_theta =
            -(axes_r[0] * prev_axes_r[0] + axes_r[1] * prev_axes_r[1] + axes_r[2] * prev_axes_r[2]);
        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).max(0.0).sqrt();
        let cos_theta_d2 = (0.5 * (1.0 + junction_cos_theta)).max(0.0).sqrt();
        let one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;
        if one_minus_sin_theta_d2 > 0.0 && cos_theta_d2 > 0.0 {
            let r_jd = sin_theta_d2 / one_minus_sin_theta_d2;
            let move_jd_v2 = r_jd * self.junction_deviation * self.accel;
            let pmove_jd_v2 = r_jd * prev.junction_deviation * prev.accel;
            // The approximated circle must contact the moves no further than
            // mid-move: centripetal_v2 = .5 * move_d * accel * tan(theta/2).
            let quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
            let move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2;
            let pmove_centripetal_v2 = prev.delta_v2 * quarter_tan_theta_d2;
            max_start_v2 = max_start_v2
                .min(move_jd_v2)
                .min(pmove_jd_v2)
                .min(move_centripetal_v2)
                .min(pmove_centripetal_v2);
        }
        self.max_start_v2 = max_start_v2;
        self.max_mcr_start_v2 = max_start_v2.min(prev.max_mcr_start_v2 + prev.mcr_delta_v2);
    }

    /// Fix this move's profile from the junction speeds around it
    /// (`klippy/toolhead.py:100-114`).
    ///
    /// `start_v2`/`cruise_v2`/`end_v2` are squared velocities. The move is
    /// divided into an accelerating, a cruising and a decelerating part, each
    /// with its own time.
    pub fn set_junction(&mut self, start_v2: f64, cruise_v2: f64, end_v2: f64) {
        let half_inv_accel = 0.5 / self.accel;
        let accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        let decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        let cruise_d = self.move_d - accel_d - decel_d;
        self.start_v = start_v2.sqrt();
        self.cruise_v = cruise_v2.sqrt();
        self.end_v = end_v2.sqrt();
        self.accel_t = divide(accel_d, (self.start_v + self.cruise_v) * 0.5);
        self.cruise_t = divide(cruise_d, self.cruise_v);
        self.decel_t = divide(decel_d, (self.end_v + self.cruise_v) * 0.5);
    }
}

impl std::fmt::Debug for Move {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Move")
            .field("start_pos", &self.start_pos)
            .field("end_pos", &self.end_pos)
            .field("move_d", &self.move_d)
            .field("start_v", &self.start_v)
            .field("cruise_v", &self.cruise_v)
            .field("end_v", &self.end_v)
            .field("accel_t", &self.accel_t)
            .field("cruise_t", &self.cruise_t)
            .field("decel_t", &self.decel_t)
            .finish_non_exhaustive()
    }
}

/// `numerator / denominator`, or zero when the denominator is zero.
///
/// A part with zero distance also has zero average velocity; upstream relies
/// on this not happening, but a zero here would be a `NaN` time that spreads
/// through the queue, so it is pinned to zero instead.
fn divide(numerator: f64, denominator: f64) -> f64 {
    if denominator != 0.0 {
        numerator / denominator
    } else {
        0.0
    }
}

/// A queue of moves that decides junction speeds across them.
///
/// Moves are added as they are requested and flushed once enough time has
/// accumulated (or when the caller asks), which is what lets the planner look
/// *ahead* and not decelerate to a stop at every move boundary
/// (`klippy/toolhead.py:120-193`).
#[derive(Debug, Default)]
pub struct LookAheadQueue {
    queue: Vec<Move>,
    junction_flush: f64,
}

impl LookAheadQueue {
    /// An empty queue with upstream's default flush time.
    pub fn new() -> Self {
        Self {
            queue: Vec::new(),
            junction_flush: LOOKAHEAD_FLUSH_TIME,
        }
    }

    /// Drop every queued move (`klippy/toolhead.py:124-126`).
    pub fn reset(&mut self) {
        self.queue.clear();
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
    }

    /// Change the flush time (`klippy/toolhead.py:127-128`).
    pub fn set_flush_time(&mut self, flush_time: f64) {
        self.junction_flush = flush_time;
    }

    /// Whether there is nothing queued.
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    /// The most recently added move.
    pub fn last(&self) -> Option<&Move> {
        self.queue.last()
    }

    /// Add a move; returns whether the queue wants flushing now
    /// (`klippy/toolhead.py:186-193`).
    ///
    /// `extra_axes` are the non-kinematic axes (the extruder); each contributes
    /// a junction limit, as upstream's `Move.calc_junction` folds in
    /// (`klippy/toolhead.py:66-99`).
    pub fn add_move(&mut self, move_: Move, extra_axes: &[Arc<dyn ExtraAxis>]) -> bool {
        let len = self.queue.len();
        self.queue.push(move_);
        if len == 0 {
            return false;
        }
        // The extra axes' junction limits need both moves borrowed at once.
        let extra_v2: Vec<f64> = {
            let (front, back) = self.queue.split_at(len);
            let prev = &front[len - 1];
            let cur = &back[0];
            extra_axes
                .iter()
                .enumerate()
                // Slots past the move's four are skipped, as `ToolHead::move_to`
                // skips the axis itself: no slot, no junction limit to fold in.
                .filter(|(index, _)| *index + E_AXIS < cur.axes_d.len())
                .map(|(index, axis)| axis.calc_junction(prev, cur, index + E_AXIS))
                .collect()
        };
        // The move just pushed and the one before it, without borrowing the
        // whole queue twice.
        let (front, back) = self.queue.split_at_mut(len);
        let last = &mut back[0];
        last.calc_junction(&front[len - 1], &extra_v2);
        self.junction_flush -= last.min_move_t;
        self.junction_flush <= 0.0
    }

    /// Flush the queue, returning the moves whose junctions are now decided.
    ///
    /// The reverse pass walks from the end of the queue back to the front
    /// assuming the toolhead must come to a stop after the last queued move;
    /// the forward pass then propagates the cruise speeds it found. `lazy`
    /// leaves the trailing moves in the queue when it can
    /// (`klippy/toolhead.py:135-185`).
    pub fn flush(&mut self, lazy: bool) -> Vec<Move> {
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
        let mut update_flush_count = lazy;
        let queue_len = self.queue.len();
        let mut flush_count = queue_len;
        // Reverse pass: maximum junction speed for each move, assuming a stop
        // after the last one.
        let mut junction_info: Vec<(f64, Option<f64>, f64)> = vec![(0.0, None, 0.0); queue_len];
        let mut next_start_v2 = 0.0;
        let mut next_mcr_start_v2 = 0.0;
        let mut peak_cruise_v2 = 0.0;
        let mut pending_cv2_assign: usize = 0;
        for i in (0..queue_len).rev() {
            let move_ = &self.queue[i];
            let reachable_start_v2 = next_start_v2 + move_.delta_v2;
            let start_v2 = move_.max_start_v2.min(reachable_start_v2);
            let mut cruise_v2 = None;
            pending_cv2_assign += 1;
            let reach_mcr_start_v2 = next_mcr_start_v2 + move_.mcr_delta_v2;
            let mcr_start_v2 = move_.max_mcr_start_v2.min(reach_mcr_start_v2);
            if mcr_start_v2 < reach_mcr_start_v2 {
                // This move can accelerate.
                if mcr_start_v2 + move_.mcr_delta_v2 > next_mcr_start_v2 || pending_cv2_assign > 1 {
                    // It can both accelerate and decelerate, or it is a full
                    // acceleration followed by a full deceleration.
                    if update_flush_count && peak_cruise_v2 != 0.0 {
                        flush_count = i + pending_cv2_assign;
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = (mcr_start_v2 + reach_mcr_start_v2) * 0.5;
                }
                cruise_v2 = Some(
                    ((start_v2 + reachable_start_v2) * 0.5)
                        .min(move_.max_cruise_v2)
                        .min(peak_cruise_v2),
                );
                pending_cv2_assign = 0;
            }
            junction_info[i] = (start_v2, cruise_v2, next_start_v2);
            next_start_v2 = start_v2;
            next_mcr_start_v2 = mcr_start_v2;
        }
        if update_flush_count || flush_count == 0 {
            return Vec::new();
        }
        // Forward pass: propagate the cruise speed through the moves that
        // cannot accelerate on their own.
        let mut prev_cruise_v2 = 0.0f64;
        for (i, info) in junction_info.iter().enumerate().take(flush_count) {
            let (start_v2, cruise_v2, next_start_v2) = *info;
            let cruise_v2 = match cruise_v2 {
                Some(cruise_v2) => cruise_v2,
                None => prev_cruise_v2.min(start_v2),
            };
            let move_ = &mut self.queue[i];
            move_.set_junction(
                start_v2.min(cruise_v2),
                cruise_v2,
                next_start_v2.min(cruise_v2),
            );
            prev_cruise_v2 = cruise_v2;
        }
        let rest = self.queue.split_off(flush_count);
        std::mem::replace(&mut self.queue, rest)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// The limits a default `[printer]` produces: 200 mm/s, 1000 mm/s^2,
    /// 5 mm/s square corner, 0.5 minimum cruise ratio
    /// (`klippy/toolhead.py:209-216` `:534-537`).
    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    fn planar_move(dx: f64, dy: f64, speed: f64) -> Move {
        let limits = limits();
        Move::new(
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(dx, dy, 0.0, 0.0),
            speed,
            &limits,
        )
    }

    #[test]
    fn test_a_move_profiles_into_accel_cruise_decel() {
        let mut move_ = planar_move(10.0, 0.0, 100.0);

        assert!((move_.move_d - 10.0).abs() < 1e-12);
        assert_eq!(move_.axes_d[0], 10.0);
        assert_eq!(move_.axes_r[0], 1.0);
        assert!(move_.is_kinematic_move);
        assert_eq!(move_.max_cruise_v2, 10_000.0);
        // 2 * move_d * accel
        assert_eq!(move_.delta_v2, 20_000.0);

        // Start and end at rest: symmetric accel and decel, no cruise.
        move_.set_junction(0.0, 10_000.0, 0.0);

        assert_eq!(move_.start_v, 0.0);
        assert_eq!(move_.cruise_v, 100.0);
        assert_eq!(move_.end_v, 0.0);
        assert!((move_.accel_t - 0.1).abs() < 1e-12);
        assert_eq!(move_.cruise_t, 0.0);
        assert!((move_.decel_t - 0.1).abs() < 1e-12);
    }

    #[test]
    fn test_an_extrude_only_move_has_no_kinematic_distance() {
        let limits = limits();
        let move_ = Move::new(
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(0.0, 0.0, 0.0, 5.0),
            30.0,
            &limits,
        );

        // Nothing moves in space, so it is not a kinematic move: it takes the
        // extruder's own distance and an effectively unbounded acceleration.
        assert!(!move_.is_kinematic_move);
        assert_eq!(move_.move_d, 5.0);
        assert_eq!(move_.accel, 99999999.9);
        assert_eq!(move_.end_pos, Coord::new(0.0, 0.0, 0.0, 5.0));
    }

    #[test]
    fn test_junction_speed_is_limited_at_a_corner() {
        let limits = limits();
        let prev = Move::new(
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(10.0, 0.0, 0.0, 0.0),
            100.0,
            &limits,
        );
        let mut next = Move::new(
            Coord::new(10.0, 0.0, 0.0, 0.0),
            Coord::new(10.0, 10.0, 0.0, 0.0),
            100.0,
            &limits,
        );

        next.calc_junction(&prev, &[]);

        // A 90-degree corner is slower than the 100 mm/s cruise, and still
        // faster than a stop.
        assert!(next.max_start_v2 > 0.0);
        assert!(next.max_start_v2 < 10_000.0, "{}", next.max_start_v2);
    }

    #[test]
    fn test_a_straight_line_keeps_its_cruise_speed() {
        let limits = limits();
        let prev = Move::new(
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(10.0, 0.0, 0.0, 0.0),
            100.0,
            &limits,
        );
        let mut next = Move::new(
            Coord::new(10.0, 0.0, 0.0, 0.0),
            Coord::new(20.0, 0.0, 0.0, 0.0),
            100.0,
            &limits,
        );

        next.calc_junction(&prev, &[]);

        // No corner to slow down for: only the cruise cap applies.
        assert_eq!(next.max_start_v2, 10_000.0);
    }

    #[test]
    fn test_the_lookahead_flushes_when_enough_time_is_queued() {
        let mut queue = LookAheadQueue::new();

        // The first move never flushes on its own.
        assert!(!queue.add_move(planar_move(10.0, 0.0, 100.0), &[]));

        // Each move takes 0.1 s; the 0.15 s budget is passed on the second.
        assert!(!queue.add_move(planar_move(10.0, 0.0, 100.0), &[]));
        assert!(queue.add_move(planar_move(10.0, 0.0, 100.0), &[]));

        let moves = queue.flush(false);
        assert_eq!(moves.len(), 3);
        // Every flushed move has a profile.
        assert!(moves.iter().all(|m| m.cruise_v > 0.0));
    }

    #[test]
    fn test_a_lazy_flush_keeps_short_queues_intact() {
        let mut queue = LookAheadQueue::new();
        for _ in 0..3 {
            queue.add_move(planar_move(10.0, 0.0, 100.0), &[]);
        }

        // Too few moves to know the peak cruise speed: a lazy flush returns
        // nothing, and the moves are still there for a real flush.
        assert!(queue.flush(true).is_empty());
        assert_eq!(queue.flush(false).len(), 3);
    }
}
