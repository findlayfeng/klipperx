//! Input shaping: the shaper's position function.
//!
//! Upstream's `chelper/kin_shaper.c`. An input shaper replaces a stepper's
//! position function with a convolution of the axis motion with its impulses:
//! the shaped position at time `t` is a weighted sum of the positions the axis
//! held at `t + t[i]`,
//!
//! ```text
//! axis_position(t)      = start_pos.axis + axes_r.axis * move_get_distance(t)
//! shaped_axis(t)        = Σ a[i] * axis_position(t + t[i])
//! stepper_position(t)   = inner_solver(shaped_x(t), shaped_y(t), shaped_z(t))
//! ```
//!
//! (`calc_position`, `kin_shaper.c:120-152`). The impulse times are relative to
//! the shaped moment and may be either sign: a shaper reaches both before and
//! after `t` (`mzv`'s impulses do), so a sample can land in the segment before
//! or after the one being generated. The samples are taken from the trapq
//! itself — the neighbourhood the solver was given as a [`SegmentWindow`],
//! which is how a shifted time finds its segment.
//!
//! The shaped coordinate then goes into the solver that was decorated, as a
//! synthetic move that does not travel (`is->m` with `DUMMY_T`,
//! `kin_shaper.c:124-190`): a delta's tower length or a polar's angle is
//! computed from the shaped position exactly as it would be from a commanded
//! one.
//!
//! The impulses are normalized and shifted so that constant-velocity motion
//! passes through the convolution unchanged ([`ShaperPulses::new`],
//! `shift_pulses`, `kin_shaper.c:28-52`) — without that a shaper would also
//! move the toolhead.

use super::itersolve::{Axis, AxisFlags, PositionFn, SegmentWindow};
use super::trapq::MoveSegment;
use crate::core::klippy::mathutil::Xyz;

/// The most impulses a shaper may carry (`shaper_pulses.pulses[10]`,
/// `kin_shaper.c:22-27`).
///
/// The limit is enforced where a shaper's coefficients are built
/// ([`crate::core::klippy::extras::input_shaper`]), as upstream's
/// `input_shaper_set_shaper_params` enforces it
/// (`kin_shaper.c:54-59`).
pub const MAX_PULSES: usize = 10;

/// The time the shaped coordinate is handed to the decorated solver at
/// (`DUMMY_T`, `kin_shaper.c:114`). The synthetic move is twice as long and
/// does not travel, so any time inside it reads the same position.
const DUMMY_T: f64 = 500.0;

/// The three axes, in the order [`KinematicShaper`] keeps their impulses.
const AXES: [(Axis, AxisFlags); 3] = [
    (Axis::X, AxisFlags::X),
    (Axis::Y, AxisFlags::Y),
    (Axis::Z, AxisFlags::Z),
];

/// One impulse: when it acts and how much it weighs (`struct { double t, a; }`,
/// `kin_shaper.c:22-27`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShaperPulse {
    /// Seconds relative to the shaped moment; a negative impulse acts ahead of
    /// it.
    pub t: f64,
    /// The impulse's weight. The weights of one axis sum to 1.
    pub a: f64,
}

/// One axis' impulses as the solver applies them (upstream's `struct
/// shaper_pulses`, `kin_shaper.c:22-27`).
///
/// The impulses are normalized (weights summing to 1) and shifted so that
/// their time-weighted sum is 0 (`init_shaper` + `shift_pulses`,
/// `kin_shaper.c:54-89`), which is what makes the shaper transparent to
/// constant-velocity motion.
#[derive(Debug, Clone, PartialEq)]
pub struct ShaperPulses {
    /// The impulses, in increasing time order. `shift_pulses` keeps them
    /// sorted: reversing the shaper's own order negates its times, and the
    /// shift is the same for every impulse.
    pulses: Vec<ShaperPulse>,
}

impl ShaperPulses {
    /// The solver's impulses for a shaper's coefficients (`init_shaper`,
    /// `kin_shaper.c:54-80`): the weights are normalized, the impulse order is
    /// reversed (upstream's `pulses[n-i-1]`, so the list runs in increasing
    /// time), and every time is moved by the weighted mean time.
    ///
    /// `None` when there is nothing to apply — no impulses at all, which is how
    /// `get_none_shaper()` describes an axis without shaping
    /// (`input_shaper.py:49-54`) and what upstream checks as `!num_pulses`
    /// (`kin_shaper.c:130,144,158`).
    ///
    /// # Panics
    /// If `a` and `t` are not one weight per impulse. The two lists come from
    /// one shaper and always pair up.
    pub fn new(a: &[f64], t: &[f64]) -> Option<Self> {
        assert_eq!(a.len(), t.len(), "one weight per impulse");
        if a.is_empty() {
            return None;
        }
        let inv_a = 1.0 / a.iter().sum::<f64>();
        // Reversed as it is built: upstream writes `a[i]` into `pulses[n-i-1]`.
        let mut pulses: Vec<ShaperPulse> = a
            .iter()
            .zip(t)
            .rev()
            .map(|(a, t)| ShaperPulse {
                t: -t,
                a: a * inv_a,
            })
            .collect();
        // Shift the impulses around their weighted mean time, so that the shaper
        // is an identity for constant-speed motion (`shift_pulses`,
        // `kin_shaper.c:28-38`).
        let shift: f64 = pulses.iter().map(|pulse| pulse.a * pulse.t).sum();
        for pulse in &mut pulses {
            pulse.t -= shift;
        }
        Some(Self { pulses })
    }

    /// The impulses, in increasing time order.
    pub fn pulses(&self) -> &[ShaperPulse] {
        &self.pulses
    }

    /// The latest time an impulse acts at: how far the shaped motion reaches
    /// after the move it is active in (`pre_active`, `kin_shaper.c:199-201`).
    pub fn pre_active(&self) -> f64 {
        self.pulses.last().map_or(0.0, |pulse| pulse.t)
    }

    /// The earliest time an impulse acts at, negated: how far the shaped motion
    /// reaches before the move it is active in (`post_active`,
    /// `kin_shaper.c:200-202`).
    pub fn post_active(&self) -> f64 {
        self.pulses.first().map_or(0.0, |pulse| -pulse.t)
    }

    /// Where `axis` is `move_time` seconds into `cuts[index]`, after the shaper
    /// (`calc_position`, `kin_shaper.c:120-152`):
    ///
    /// ```text
    /// Σ_i a[i] * axis_position(seg_at(t + t[i]), t + t[i])
    /// ```
    ///
    /// `axis_position` is the trapq's own coordinate of that axis
    /// (`get_axis_position`, `kin_shaper.c:97-103`). Upstream walks the trapq's
    /// move list from the segment being generated, in both directions, because
    /// a pulse time is either sign; `cuts` is that neighbourhood and `index` the
    /// segment being generated. A time running off either end of `cuts` is
    /// clipped there, where upstream would walk into the trapq's sentinels.
    fn position(&self, cuts: &[MoveSegment], index: usize, move_time: f64, axis: Axis) -> f64 {
        let shaped_time = cuts[index].print_time + move_time;
        let mut position = 0.0;
        for pulse in &self.pulses {
            let (segment, move_time) = locate(cuts, index, shaped_time + pulse.t);
            position += pulse.a * axis_position(segment, move_time, axis);
        }
        position
    }
}

/// The shaper one stepper applies: the impulses of each axis, as one snapshot
/// (the `sp[3]` of upstream's `struct input_shaper`, `kin_shaper.c:17-23`).
///
/// A `[input_shaper]` builds one and hands the same snapshot to every stepper
/// ([`crate::core::klippy::extras::input_shaper`]'s
/// `_update_input_shaping`, `input_shaper.py:169-190`); each stepper applies the
/// axes its own `active_flags` name, as `input_shaper_update_sk` picks between
/// the per-axis callbacks (`kin_shaper.c:226-241`).
#[derive(Debug, Clone, PartialEq)]
pub struct KinematicShaper {
    /// One entry per axis in [`AXES`] order — x, y, z. `None` is an axis that
    /// is not shaped: `shaper_freq` 0, upstream's `num_pulses == 0`.
    axes: [Option<ShaperPulses>; 3],
}

impl KinematicShaper {
    /// A shaper from the three axes' impulses.
    pub fn new(axes: [Option<ShaperPulses>; 3]) -> Self {
        Self { axes }
    }

    /// Whether no axis is shaped. Such a shaper cannot move anything: the
    /// stepper reads its plain position function, so it is never installed.
    pub fn is_empty(&self) -> bool {
        self.axes.iter().all(Option::is_none)
    }

    /// How far, in seconds, the shaped motion of a stepper that moves
    /// `active_flags` reaches before the first and after the last move it is
    /// active in (`shaper_note_generation_time`, `kin_shaper.c:197-224`): the
    /// largest impulse time of the axes it moves, and the largest negated one.
    ///
    /// Only the axes the stepper moves count (upstream's `active_flags & AF_X`
    /// tests): a shaper's z impulses do not widen an x motor's window.
    pub fn generation_window(&self, active_flags: AxisFlags) -> (f64, f64) {
        let mut pre_active: f64 = 0.0;
        let mut post_active: f64 = 0.0;
        for (pulses, (_, flag)) in self.axes.iter().zip(AXES) {
            if !active_flags.intersects(flag) {
                continue;
            }
            let Some(pulses) = pulses else {
                continue;
            };
            pre_active = pre_active.max(pulses.pre_active());
            post_active = post_active.max(pulses.post_active());
        }
        (pre_active, post_active)
    }

    /// Where a stepper is `move_time` seconds into the segment being generated
    /// (`shaper_x_calc_position` and friends, `kin_shaper.c:124-190`).
    ///
    /// `inner` is the position function being decorated and `active_flags` the
    /// axes the stepper moves: each of those with impulses is convolved, the
    /// others are read at `move_time` unshaped — upstream's
    /// `is->m.start_pos = move_get_coord(m, move_time)` followed by the
    /// per-axis overrides. The shaped coordinate then goes into `inner` as a
    /// synthetic move that does not travel, at [`DUMMY_T`].
    pub fn position(
        &self,
        window: SegmentWindow<'_>,
        move_time: f64,
        active_flags: AxisFlags,
        inner: &PositionFn,
    ) -> f64 {
        let segment = window.segment();
        let mut start_pos = segment.coord(move_time);
        for (pulses, (axis, flag)) in self.axes.iter().zip(AXES) {
            if !active_flags.intersects(flag) {
                continue;
            }
            let Some(pulses) = pulses else {
                continue;
            };
            let shaped = pulses.position(window.cuts(), window.index(), move_time, axis);
            start_pos = match axis {
                Axis::X => Xyz::new(shaped, start_pos.y(), start_pos.z()),
                Axis::Y => Xyz::new(start_pos.x(), shaped, start_pos.z()),
                Axis::Z => Xyz::new(start_pos.x(), start_pos.y(), shaped),
            };
        }
        let synthetic = MoveSegment {
            print_time: segment.print_time,
            move_t: 2.0 * DUMMY_T,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos,
            axes_r: Xyz::default(),
        };
        inner.call(&synthetic, DUMMY_T)
    }
}

/// The coordinate `axis` has `move_time` seconds into `segment`
/// (`get_axis_position`, `kin_shaper.c:97-103`).
fn axis_position(segment: &MoveSegment, move_time: f64, axis: Axis) -> f64 {
    let distance = segment.distance(move_time);
    let (start, direction) = match axis {
        Axis::X => (segment.start_pos.x(), segment.axes_r.x()),
        Axis::Y => (segment.start_pos.y(), segment.axes_r.y()),
        Axis::Z => (segment.start_pos.z(), segment.axes_r.z()),
    };
    start + direction * distance
}

/// The segment of `cuts` that `abs_time` falls in, with the time relative to
/// it (`calc_position`'s two walks, `kin_shaper.c:120-152`).
///
/// The walk starts at `index`, the segment being generated, because a shifted
/// time is only a few milliseconds away. A time before the first segment or
/// after the last one is clipped to it.
fn locate(cuts: &[MoveSegment], index: usize, abs_time: f64) -> (&MoveSegment, f64) {
    let mut index = index;
    while abs_time < cuts[index].print_time {
        if index == 0 {
            return (&cuts[0], 0.0);
        }
        index -= 1;
    }
    loop {
        let move_time = abs_time - cuts[index].print_time;
        if move_time <= cuts[index].move_t {
            return (&cuts[index], move_time);
        }
        if index + 1 == cuts.len() {
            return (&cuts[index], cuts[index].move_t);
        }
        index += 1;
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::motion::itersolve::cartesian_position_fn;

    /// A segment: `start_v` mm/s along +X from `start_x`, accelerating at
    /// `2 * half_accel`, lasting `move_t` seconds from `print_time`.
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
            axes_r: Xyz::new(1.0, 0.0, 0.0),
        }
    }

    /// The coordinate the trapq holds at absolute print time `time`, from the
    /// segment that contains it — the sampled function, evaluated independently
    /// of [`locate`].
    fn coordinate_at(cuts: &[MoveSegment], time: f64) -> f64 {
        for segment in cuts {
            if time < segment.end_time() {
                return segment.coord(time - segment.print_time).x();
            }
        }
        let last = &cuts[cuts.len() - 1];
        last.coord(last.move_t).x()
    }

    /// The impulses are normalized and shifted around their weighted mean time,
    /// which is what makes the shaper transparent to constant-velocity motion
    /// (`init_shaper` + `shift_pulses`, `kin_shaper.c:28-89`).
    #[test]
    fn the_impulses_normalize_and_shift_to_a_weighted_mean_of_zero() {
        // Weights 0.8 and 0.2 at 0.02 s and 0.05 s, from a shaper that sums to
        // one already; the shifted times are -0.024 s and +0.006 s:
        // Σ a t = -0.8*0.024 + 0.2*0.006 = -0.018, and the raw mean time is
        // 0.8*0.02 + 0.2*0.05 = 0.026, so the shifted pair is (-0.05, -0.02) +
        // 0.026.
        let pulses = ShaperPulses::new(&[0.8, 0.2], &[0.02, 0.05]).expect("two impulses");

        let shifted = [(-0.024, 0.2), (0.006, 0.8)];
        assert_eq!(pulses.pulses().len(), shifted.len());
        for (pulse, (t, a)) in pulses.pulses().iter().zip(shifted) {
            assert!((pulse.t - t).abs() < 1e-12, "{} != {t}", pulse.t);
            assert!((pulse.a - a).abs() < 1e-12, "{} != {a}", pulse.a);
        }
        let weight: f64 = pulses.pulses().iter().map(|pulse| pulse.a).sum();
        assert!((weight - 1.0).abs() < 1e-12, "weights sum to {weight}");
        let mean: f64 = pulses.pulses().iter().map(|pulse| pulse.a * pulse.t).sum();
        assert!(mean.abs() < 1e-12, "weighted mean time is {mean}");

        // Unnormalized coefficients normalize to the same impulses.
        let scaled = ShaperPulses::new(&[4.0, 1.0], &[0.02, 0.05]).expect("two impulses");
        for (pulse, scaled) in pulses.pulses().iter().zip(scaled.pulses()) {
            assert!((pulse.t - scaled.t).abs() < 1e-12);
            assert!((pulse.a - scaled.a).abs() < 1e-12);
        }
        // No impulses at all is an axis that is not shaped.
        assert!(ShaperPulses::new(&[], &[]).is_none());
    }

    /// The generation window comes from the outermost impulse times, and only
    /// from the axes the stepper moves (`shaper_note_generation_time`,
    /// `kin_shaper.c:197-224`).
    #[test]
    fn the_generation_window_spans_the_pulse_times() {
        let assert_window = |got: (f64, f64), want: (f64, f64)| {
            assert!(
                (got.0 - want.0).abs() < 1e-12 && (got.1 - want.1).abs() < 1e-12,
                "{got:?} != {want:?}"
            );
        };
        // (0.8, 0.2) at 0.02 s/0.05 s shifts to -0.024 s and +0.006 s: the
        // motion reaches 6 ms after the shaped moment and 24 ms before it.
        let asymmetric = KinematicShaper::new([
            Some(ShaperPulses::new(&[0.8, 0.2], &[0.02, 0.05]).unwrap()),
            None,
            None,
        ]);
        assert_window(asymmetric.generation_window(AxisFlags::X), (0.006, 0.024));

        // The other way round the window swaps, as the impulses do.
        let mirrored = KinematicShaper::new([
            Some(ShaperPulses::new(&[0.2, 0.8], &[0.02, 0.05]).unwrap()),
            None,
            None,
        ]);
        assert_window(mirrored.generation_window(AxisFlags::X), (0.024, 0.006));

        // A stepper that only moves Y has no window from x's impulses, and no
        // axes at all means no window at all.
        assert_window(asymmetric.generation_window(AxisFlags::Y), (0.0, 0.0));
        assert_window(
            asymmetric.generation_window(AxisFlags::X.union(AxisFlags::Y)),
            (0.006, 0.024),
        );
        assert_window(asymmetric.generation_window(AxisFlags::Z), (0.0, 0.0));

        // A single impulse at time zero is a window of zero, as `none` is.
        let single =
            KinematicShaper::new([Some(ShaperPulses::new(&[1.0], &[0.0]).unwrap()), None, None]);
        assert_window(single.generation_window(AxisFlags::X), (0.0, 0.0));
        assert!(KinematicShaper::new([None, None, None]).is_empty());
        assert!(!single.is_empty());
    }

    /// A straight line at constant speed passes through the shaper unchanged —
    /// the property [`ShaperPulses::new`]'s shift exists for.
    #[test]
    fn a_constant_velocity_move_is_left_alone() {
        let cuts = [segment(0.0, 0.5, 0.0, 100.0, 0.0)];
        let shaper = KinematicShaper::new([
            Some(ShaperPulses::new(&[0.8, 0.2], &[0.02, 0.05]).unwrap()),
            None,
            None,
        ]);
        let inner = cartesian_position_fn(Axis::X);

        for sample in 0..=10 {
            let move_time = 0.05 + 0.04 * f64::from(sample);
            let unshaped = cuts[0].coord(move_time).x();
            let shaped = shaper.position(
                SegmentWindow::new(&cuts, 0),
                move_time,
                AxisFlags::X,
                &inner,
            );
            assert!(
                (shaped - unshaped).abs() < 1e-9,
                "at {move_time}: {shaped} != {unshaped}"
            );
        }
    }

    /// The shaped position is the weighted sum of the axis positions at the
    /// delayed times (`calc_position`, `kin_shaper.c:120-152`), checked against
    /// that formula evaluated independently, across a velocity step so the
    /// impulses really sample different segments.
    #[test]
    fn the_shaped_position_is_the_weighted_sum_of_the_delayed_positions() {
        // Stand still for 0.1 s, then cruise at 100 mm/s: the impulses that
        // reach back before 0.1 s sample the standing position.
        let cuts = [
            segment(0.0, 0.1, 0.0, 0.0, 0.0),
            segment(0.1, 0.4, 0.0, 100.0, 0.0),
        ];
        let pulses = ShaperPulses::new(&[0.8, 0.2], &[0.02, 0.05]).unwrap();
        let shaper = KinematicShaper::new([Some(pulses.clone()), None, None]);
        let inner = cartesian_position_fn(Axis::X);

        for index in 0..cuts.len() {
            for sample in 0..=20 {
                let move_time = cuts[index].move_t * f64::from(sample) / 20.0;
                // Skip the samples whose impulses would run off `cuts`.
                let shaped_time = cuts[index].print_time + move_time;
                let earliest = shaped_time + pulses.pulses()[0].t;
                let latest = shaped_time + pulses.pulses()[1].t;
                if earliest < 0.0 || latest > cuts[1].end_time() {
                    continue;
                }
                let expected: f64 = pulses
                    .pulses()
                    .iter()
                    .map(|pulse| pulse.a * coordinate_at(&cuts, shaped_time + pulse.t))
                    .sum();
                let shaped = shaper.position(
                    SegmentWindow::new(&cuts, index),
                    move_time,
                    AxisFlags::X,
                    &inner,
                );
                assert!(
                    (shaped - expected).abs() < 1e-9,
                    "segment {index} at {move_time}: {shaped} != {expected}"
                );
            }
        }

        // The shaper really moves this motion: the heavier impulse reaches
        // 6 ms ahead, so just after the velocity step the shaped position is
        // already out at the moving one.
        let unshaped = cuts[1].coord(0.004).x();
        let shaped = shaper.position(SegmentWindow::new(&cuts, 1), 0.004, AxisFlags::X, &inner);
        assert!(shaped > unshaped + 0.1, "{shaped} should lead {unshaped}");
    }

    /// The shaped coordinate goes into the decorated solver as a position, not
    /// as a motion: a delta's arm length is computed from it
    /// (`shaper_xyz_calc_position`, `kin_shaper.c:162-190`).
    #[test]
    fn the_decorated_solver_sees_the_shaped_coordinate() {
        let cuts = [segment(0.0, 0.2, 0.0, 50.0, 0.0)];
        let pulses = ShaperPulses::new(&[0.8, 0.2], &[0.02, 0.05]).unwrap();
        // The shaper reaches ±(the impulses' span) around 0.05 s, inside the
        // segment.
        let move_time = 0.05;
        let shaper = KinematicShaper::new([Some(pulses.clone()), None, None]);

        // A solver that reads both the coordinate and its own move time: the
        // decorated call must read the shaped x at the synthetic move's time,
        // where nothing travels.
        let probe = PositionFn::bind(
            |segment: &MoveSegment, move_time: f64, _: &[f64; 6]| {
                1000.0 * segment.start_pos.x() + move_time
            },
            [0.0; 6],
        );
        let shaped_x: f64 = pulses
            .pulses()
            .iter()
            .map(|pulse| pulse.a * coordinate_at(&cuts, 0.05 + pulse.t))
            .sum();
        let position = shaper.position(
            SegmentWindow::new(&cuts, 0),
            move_time,
            AxisFlags::X,
            &probe,
        );
        assert!(
            (position - (1000.0 * shaped_x + DUMMY_T)).abs() < 1e-6,
            "{position} did not see {shaped_x} at {DUMMY_T}"
        );
    }
}
