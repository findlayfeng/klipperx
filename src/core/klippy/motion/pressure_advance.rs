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

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::Xyz;

    /// One extruder trapq segment: `start_x` is the extrusion amount, the
    /// segment travels `start_v` millimetres of filament per second and
    /// accelerates at `2 * half_accel`. `axes_r.y` marks it as a move that may
    /// use pressure advance (`can_pressure_advance`, `extruder.py:221-232`).
    fn segment(
        print_time: f64,
        move_t: f64,
        start_x: f64,
        start_v: f64,
        half_accel: f64,
    ) -> MoveSegment {
        MoveSegment {
            print_time,
            move_t,
            start_v,
            half_accel,
            start_pos: Xyz::new(start_x, 0.0, 0.0),
            axes_r: Xyz::new(1.0, 1.0, 0.0),
        }
    }

    /// The smoothed position at absolute print time `print_time`, evaluated on
    /// the segment that contains it (as the solver does).
    fn position_at(state: &ExtruderPressureAdvance, cuts: &[MoveSegment], print_time: f64) -> f64 {
        let index = cuts
            .iter()
            .position(|segment| print_time < segment.end_time())
            .unwrap_or(cuts.len() - 1);
        state.position(cuts, index, print_time - cuts[index].print_time)
    }

    /// The old position function read the trapq's x
    /// (`extruder_position_fn`, `chelper/kin_extruder.c:134`'s nominal path).
    /// With pressure advance off, point for point the same.
    #[test]
    fn test_pressure_advance_off_keeps_the_nominal_position() {
        let cuts = [
            segment(0.0, 0.1, 0.0, 5.0, 20.0),
            segment(0.1, 0.2, 0.6, 9.0, 0.0),
            segment(0.3, 0.1, 2.4, 9.0, -20.0),
        ];
        // `smooth_time` alone does nothing: upstream disables the filter when
        // `pressure_advance` is zero (`extruder.py:68-73`).
        let state = ExtruderPressureAdvance::new(0.0, 0.040);

        for (index, segment) in cuts.iter().enumerate() {
            for sample in 0..=20 {
                let move_time = segment.move_t * f64::from(sample) / 20.0;
                assert_eq!(
                    state.position(&cuts, index, move_time),
                    segment.coord(move_time).x(),
                    "segment {index} at {move_time}"
                );
            }
        }
    }

    /// During a constant-velocity stretch the smoothed position is the nominal
    /// one plus `pressure_advance * velocity`: the triangular window is
    /// symmetric, so a straight line passes through it unchanged.
    #[test]
    fn test_pressure_advance_leads_by_pa_times_velocity() {
        // Accelerate from rest to 100 mm/s over 0.1 s (half_accel 500), then
        // cruise.
        let cuts = [
            segment(0.0, 0.1, 0.0, 0.0, 500.0),
            segment(0.1, 0.2, 5.0, 100.0, 0.0),
        ];
        let state = ExtruderPressureAdvance::new(0.05, 0.020);

        // 0.05 s into the cruise: the whole smoothing window (0.11..0.19 s) is
        // inside the constant-velocity segment.
        let nominal = cuts[1].coord(0.05).x();
        let smoothed = state.position(&cuts, 1, 0.05);
        assert_eq!(nominal, 10.0);
        assert!(
            (smoothed - (nominal + 0.05 * 100.0)).abs() < 1e-9,
            "expected {nominal} + pa*velocity, got {smoothed}"
        );

        // The offset disappears again when pressure advance is zeroed, and the
        // new value is what the next evaluation reads.
        state.set(0.0, 0.020);
        assert!((state.position(&cuts, 1, 0.05) - nominal).abs() < 1e-12);
    }

    /// A feed-rate step is spread over `smooth_time`: the longer the time, the
    /// gentler the smoothed position moves (checking the shape, not one
    /// number).
    #[test]
    fn test_a_longer_smooth_time_makes_the_transition_gentler() {
        // Standing still, then extruding flat out at 20 mm/s: without
        // smoothing the pressure-advance term steps by `0.05 * 20 = 1 mm`.
        let cuts = [
            segment(0.0, 0.5, 0.0, 0.0, 0.0),
            segment(0.5, 0.5, 0.0, 20.0, 0.0),
        ];

        // The largest position change over 0.5 ms, around the feed-rate step.
        let steepest = |smooth_time: f64| {
            let state = ExtruderPressureAdvance::new(0.05, smooth_time);
            let step = 0.000_5;
            let mut previous = position_at(&state, &cuts, 0.46);
            let mut steepest = 0.0_f64;
            let mut print_time = 0.46 + step;
            while print_time <= 0.54 {
                let position = position_at(&state, &cuts, print_time);
                steepest = steepest.max(((position - previous) / step).abs());
                previous = position;
                print_time += step;
            }
            steepest
        };

        let sharp = steepest(0.005);
        let gentle = steepest(0.100);
        assert!(
            gentle < sharp * 0.5,
            "a 0.1 s smooth time should be much gentler than 0.005 s: {gentle} vs {sharp}"
        );
    }

    /// `SET_PRESSURE_ADVANCE` writes the shared parameters: the next evaluation
    /// uses them, with no solver reinstall (`kinematics/extruder.py:67-88`).
    #[test]
    fn test_set_pressure_advance_takes_effect_on_the_next_evaluation() {
        let cuts = [segment(0.0, 0.2, 0.0, 10.0, 0.0)];
        let state = ExtruderPressureAdvance::new(0.0, 0.040);
        let nominal = cuts[0].coord(0.1).x();
        assert_eq!(state.position(&cuts, 0, 0.1), nominal);

        state.set(0.06, 0.040);
        assert_eq!(state.values(), (0.06, 0.040));
        let smoothed = state.position(&cuts, 0, 0.1);
        assert!(
            (smoothed - (nominal + 0.06 * 10.0)).abs() < 1e-9,
            "expected the new pa in the next evaluation, got {smoothed}"
        );
    }

    /// The smoothed position is the documented weighted average of the
    /// pressure-advanced position (`kin_extruder.c:18-25`), checked against a
    /// numerical integration of that formula.
    #[test]
    fn test_the_smoothing_is_the_documented_weighted_average() {
        let cuts = [
            segment(0.0, 0.1, 0.0, 5.0, 0.0),
            segment(0.1, 0.1, 0.5, 5.0, 10.0),
            segment(0.2, 0.2, 1.05, 7.0, 0.0),
        ];
        let pressure_advance = 0.04;
        let state = ExtruderPressureAdvance::new(pressure_advance, 0.030);
        let hst = 0.015;
        // `pa_position(x)`, from the segment that holds `x`.
        let pa_position = |x: f64| {
            let segment = cuts
                .iter()
                .rev()
                .find(|segment| x >= segment.print_time)
                .expect("a segment starts at or before the sample");
            let move_time = x - segment.print_time;
            let pa = if segment.axes_r.y() != 0.0 {
                pressure_advance
            } else {
                0.0
            };
            segment.start_pos.x()
                + segment.distance(move_time)
                + pa * (segment.start_v + 2.0 * segment.half_accel * move_time)
        };

        // Every window below (0.015 s either side) sits inside 0..0.4 s.
        for print_time in [0.15, 0.18, 0.25, 0.30] {
            let samples = 20_000;
            let width = 2.0 * hst;
            let mut area = 0.0;
            for sample in 0..samples {
                let offset = width * (f64::from(sample) + 0.5) / f64::from(samples);
                let x = print_time - hst + offset;
                area += (hst - (print_time - x).abs()) * pa_position(x);
            }
            let expected = area * width / f64::from(samples) / (hst * hst);
            let got = position_at(&state, &cuts, print_time);
            assert!(
                (got - expected).abs() < 1e-6,
                "at {print_time}: expected {expected}, got {got}"
            );
        }
    }
}
