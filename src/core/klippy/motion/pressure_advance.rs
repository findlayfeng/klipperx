//! Extruder pressure advance: the extruder solver's position function.
//!
//! Upstream's `chelper/kin_extruder.c`. Without pressure advance the extruder
//! stepper position is the nominal extrusion amount:
//!
//! ```text
//! extruder_position(t) = nominal_position(t)
//! ```
//!
//! With pressure advance, extra filament is pushed during acceleration (and
//! retracted during deceleration) by `pressure_advance * velocity`:
//!
//! ```text
//! pa_position(t) = nominal_position(t) + pressure_advance * nominal_velocity(t)
//! ```
//!
//! which is then "smoothed" over `smooth_time` with a triangular window
//! (`kin_extruder.c:18-25`):
//!
//! ```text
//! smooth_position(t) = definitive_integral(pa_position(x) * (smooth_time/2 - |t-x|) * dx,
//!                                          from=t-smooth_time/2, to=t+smooth_time/2)
//!                      / ((smooth_time/2)**2)
//! ```
//!
//! The window is integrated exactly, not sampled: every trapq segment is a
//! quadratic `base + t*(start_v + t*half_accel)`, so the weighted integral is
//! closed-form per segment and the window is clipped at segment boundaries,
//! walking into the neighbouring segments (`pa_range_integrate`,
//! `kin_extruder.c:90-116`). That keeps the result independent of the solver's
//! evaluation times, which is what lets the secant search ask for positions at
//! arbitrary, non-monotonic times.
//!
//! The two numbers `SET_PRESSURE_ADVANCE` writes live in atomics so the step
//! generator reads them without a lock and a change is seen at the next
//! evaluation (`kin_extruder.c:147-155` keeps a timestamped list instead; the
//! command path here applies the new value immediately, see
//! `extras::extruder::set_pressure_advance`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use super::trapq::MoveSegment;

/// The extruder's pressure-advance parameters, shared between the `[extruder]`
/// object (which `SET_PRESSURE_ADVANCE` writes) and its stepper's solver
/// (which reads them on every evaluation).
#[derive(Debug)]
pub struct ExtruderPressureAdvance {
    /// `pressure_advance`, as `f64` bits.
    pressure_advance: AtomicU64,
    /// `pressure_advance_smooth_time` in seconds, as `f64` bits.
    smooth_time: AtomicU64,
}

impl ExtruderPressureAdvance {
    /// Parameters for an extruder configured with `pressure_advance` and
    /// `pressure_advance_smooth_time`.
    pub fn new(pressure_advance: f64, smooth_time: f64) -> Arc<Self> {
        Arc::new(Self {
            pressure_advance: AtomicU64::new(pressure_advance.to_bits()),
            smooth_time: AtomicU64::new(smooth_time.to_bits()),
        })
    }

    /// The configured `(pressure_advance, smooth_time)`.
    ///
    /// Two relaxed loads: a reader may see one of them from before a
    /// simultaneous `set` (see [`ExtruderPressureAdvance::set`]).
    pub fn values(&self) -> (f64, f64) {
        (
            f64::from_bits(self.pressure_advance.load(Ordering::Relaxed)),
            f64::from_bits(self.smooth_time.load(Ordering::Relaxed)),
        )
    }

    /// Record new values for the next evaluation of the position function.
    ///
    /// Upstream applies a `pressure_advance` change at the print time of the
    /// next queued move (`register_lookahead_callback`) and flushes step
    /// generation when `smooth_time` changes (`kinematics/extruder.py:67-88`).
    /// Here both land in the solver directly: the next segment stepped uses
    /// them. A `smooth_time` change mid-queue therefore reaches the step
    /// generator without a flush (documented in `upstream-deviations`).
    pub fn set(&self, pressure_advance: f64, smooth_time: f64) {
        self.pressure_advance
            .store(pressure_advance.to_bits(), Ordering::Relaxed);
        self.smooth_time
            .store(smooth_time.to_bits(), Ordering::Relaxed);
    }

    /// Where the extruder stepper is `move_time` seconds into `cuts[index]`
    /// (`extruder_calc_position`, `kin_extruder.c:118-135`).
    ///
    /// `cuts` is the trapq's segments around the one being generated, oldest
    /// first (`itersolve::SegmentWindow`); the smoothing window reads its
    /// neighbours. Outside `cuts` the window is clipped, so a lone segment (a
    /// coordinate round-trip) behaves like the midpoint of a standing move.
    pub fn position(&self, cuts: &[MoveSegment], index: usize, move_time: f64) -> f64 {
        let segment = &cuts[index];
        let (pressure_advance, smooth_time) = self.values();
        let hst = 0.5 * smooth_time;
        if pressure_advance == 0.0 || hst == 0.0 {
            // Pressure advance not enabled (`kin_extruder.c:122-125`).
            return nominal_position(segment, move_time);
        }
        let area = range_integrate(cuts, index, move_time, hst, pressure_advance);
        segment.start_pos.x() + area / (hst * hst)
    }
}

/// The nominal extruder position `move_time` seconds into `segment`: the
/// extrusion amount, `m->start_pos.x + move_get_distance(m, move_time)`
/// (`kin_extruder.c:134`). The extruder trapq's x direction cosine is 1
/// (`extruder::PrinterExtruder::process_move` appends `(1, can_pa, 0)`), so
/// this is also what the axis-reading position function returns.
fn nominal_position(segment: &MoveSegment, move_time: f64) -> f64 {
    segment.start_pos.x() + segment.distance(move_time)
}

/// The definitive integral of the motion formula
/// (`extruder_integrate`, `kin_extruder.c:29-38`):
///
/// ```text
/// ∫ position(t) dt  for  position(t) = base + t * (start_v + t * half_accel)
/// ```
fn integrate(base: f64, start_v: f64, half_accel: f64, start: f64, end: f64) -> f64 {
    let half_v = 0.5 * start_v;
    let sixth_a = (1.0 / 3.0) * half_accel;
    let si = start * (base + start * (half_v + start * sixth_a));
    let ei = end * (base + end * (half_v + end * sixth_a));
    ei - si
}

/// The definitive integral of time-weighted position
/// (`extruder_integrate_time`, `kin_extruder.c:40-50`):
///
/// ```text
/// ∫ t * position(t) dt  for  position(t) = base + t * (start_v + t * half_accel)
/// ```
fn integrate_time(base: f64, start_v: f64, half_accel: f64, start: f64, end: f64) -> f64 {
    let half_b = 0.5 * base;
    let third_v = (1.0 / 3.0) * start_v;
    let eighth_a = 0.25 * half_accel;
    let si = start * start * (half_b + start * (third_v + start * eighth_a));
    let ei = end * end * (half_b + end * (third_v + end * eighth_a));
    ei - si
}

/// The window-weighted integral of one segment's pressure-advanced position
/// over `start..end`, measured against `time_offset`
/// (`pa_move_integrate`, `kin_extruder.c:52-79`):
///
/// ```text
/// ∫ (t - time_offset) * pa_position(t) dt
/// ```
///
/// `pa_position` adds `pressure_advance * (start_v + 2*half_accel*t)`, which
/// folds into the quadratic as `base + pressure_advance*start_v` and
/// `start_v + pressure_advance*2*half_accel`. The pressure advance of the
/// segment is zero when it cannot advance: the extruder trapq marks the moves
/// that extrude during an XY move in `axes_r.y` (`can_pressure_advance`,
/// `kinematics/extruder.py:221-232`).
fn move_integrate(
    segment: &MoveSegment,
    pressure_advance: f64,
    base: f64,
    start: f64,
    end: f64,
    time_offset: f64,
) -> f64 {
    let start = start.max(0.0);
    let end = end.min(segment.move_t);
    let pressure_advance = if segment.axes_r.y() != 0.0 {
        pressure_advance
    } else {
        0.0
    };
    let base = base + pressure_advance * segment.start_v;
    let start_v = segment.start_v + pressure_advance * 2.0 * segment.half_accel;
    let half_accel = segment.half_accel;
    let iext = integrate(base, start_v, half_accel, start, end);
    let wgt_ext = integrate_time(base, start_v, half_accel, start, end);
    wgt_ext - time_offset * iext
}

/// The window-weighted integral around `move_time` over a range of segments
/// (`pa_range_integrate`, `kin_extruder.c:81-116`).
///
/// The current segment contributes `∫(x - start)*pos(x) - ∫(x - end)*pos(x)`,
/// which is `∫(hst - |t-x|)*pos(x) dx` over the window; neighbours continue the
/// integration with their own polynomial, based relative to the current
/// segment's start position. A window running off either end of `cuts` is
/// clipped there (upstream walks into the trapq's sentinels).
fn range_integrate(
    cuts: &[MoveSegment],
    index: usize,
    move_time: f64,
    hst: f64,
    pressure_advance: f64,
) -> f64 {
    let segment = &cuts[index];
    let start_base = segment.start_pos.x();
    let mut res = 0.0;
    res += move_integrate(
        segment,
        pressure_advance,
        0.0,
        move_time - hst,
        move_time,
        move_time - hst,
    );
    res -= move_integrate(
        segment,
        pressure_advance,
        0.0,
        move_time,
        move_time + hst,
        move_time + hst,
    );
    // Integrate over the previous segments.
    let mut start = move_time - hst;
    let mut prev = index;
    while start < 0.0 {
        if prev == 0 {
            break;
        }
        prev -= 1;
        let previous = &cuts[prev];
        start += previous.move_t;
        let base = previous.start_pos.x() - start_base;
        res += move_integrate(
            previous,
            pressure_advance,
            base,
            start,
            previous.move_t,
            start,
        );
    }
    // Integrate over the following segments.
    let mut end = move_time + hst;
    let mut next = index;
    while end > cuts[next].move_t {
        end -= cuts[next].move_t;
        if next + 1 >= cuts.len() {
            break;
        }
        next += 1;
        let following = &cuts[next];
        let base = following.start_pos.x() - start_base;
        res -= move_integrate(following, pressure_advance, base, 0.0, end, end);
    }
    res
}
