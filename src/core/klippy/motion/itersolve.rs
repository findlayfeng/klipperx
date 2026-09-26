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

use crate::core::klippy::mathutil::Xyz;

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

/// One stepper's position function and solver state.
///
/// Upstream's `struct stepper_kinematics` (`chelper/itersolve.h:10-22`).
#[derive(Debug, Clone, Copy)]
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
        }
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
    fn eval(&self, segment: &MoveSegment, move_time: f64) -> f64 {
        let raw = self.position.call(segment, move_time);
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
        // as upstream).
        self.eval(&segment, 500.0)
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
    /// (`itersolve_generate_steps`, `chelper/itersolve.c:147-204`).
    ///
    /// Upstream's `gen_steps_pre_active`/`gen_steps_post_active` window (which
    /// keeps a shaper fed before and after a stepper is active) is not
    /// implemented: cartesian needs neither, and they arrive with the input
    /// shaper (H6).
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
        for segment in trapq.moves() {
            let move_end = segment.end_time();
            if move_end <= last_flush_time {
                // Already generated past this segment.
                continue;
            }
            if !self.is_active(segment) {
                if segment.print_time >= flush_time {
                    return Ok(());
                }
                continue;
            }
            let start = last_flush_time.max(segment.print_time);
            let end = flush_time.min(move_end);
            if start < end {
                self.gen_steps_range(sc, segment, start, end)?;
            }
            if move_end >= flush_time {
                self.last_move_time = flush_time;
                return Ok(());
            }
            self.last_move_time = move_end;
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
        segment: &MoveSegment,
        abs_start: f64,
        abs_end: f64,
    ) -> Result<(), StepCompressError> {
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
                position: self.eval(segment, next_time),
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

        sk.gen_steps_range(&mut sc, &segment, 0.0, segment.move_t)
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

    #[test]
    fn test_position_round_trips_through_a_coordinate() {
        let mut sk = StepKinematics::new(1.0, cartesian_position_fn(Axis::Y), AxisFlags::Y);

        sk.set_position(Xyz::new(1.0, 2.0, 3.0));

        assert_eq!(sk.commanded_pos(), 2.0);
    }
}
