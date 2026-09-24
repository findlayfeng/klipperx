//! The kinematics interface.
//!
//! Upstream loads a kinematics from `[printer] kinematics` and lets it say what
//! a move may do (`check_move`) and what the toolhead position is
//! (`calc_position`). The trait here is the narrow version FW5e needs: the
//! homing protocol (`home_rails`, `homing_state`) arrives with endstop/trsync
//! in FW6.

use std::collections::HashMap;

use serde_json::{json, Value};

use super::plan::Move;
use super::trapq::MoveSegment;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, AXES, X_AXIS, Y_AXIS, Z_AXIS};

/// What a kinematics may inspect and change about one move.
///
/// Upstream hands the kinematics the whole `Move` and lets it call
/// `limit_speed` (`klippy/kinematics/cartesian.py:113-117`). This is the narrow
/// version: the kinematics sees the geometry and may lower the speed, but not
/// touch the planner's junction state.
pub struct MoveContext<'a> {
    move_: &'a mut Move,
}

impl<'a> MoveContext<'a> {
    /// Wrap a move for a kinematics to inspect.
    pub fn new(move_: &'a mut Move) -> Self {
        Self { move_ }
    }

    /// Where the move ends.
    pub fn end_pos(&self) -> &Coord {
        &self.move_.end_pos
    }

    /// Where the move starts (polar's near-center slowdown measures the
    /// segment's closest approach to the bed center from both ends).
    pub fn start_pos(&self) -> &Coord {
        &self.move_.start_pos
    }

    /// The highest speed this move may cruise at, squared
    /// (`move.max_cruise_v2`; polar takes `sqrt` of it as the move's angular
    /// speed at a given radius).
    pub fn max_cruise_v2(&self) -> f64 {
        self.move_.max_cruise_v2
    }

    /// The per-axis distance.
    pub fn axes_d(&self) -> &[f64; 4] {
        &self.move_.axes_d
    }

    /// The per-axis distance over the total distance (the direction cosines).
    ///
    /// The extruder's `check_move` reads the E direction cosine
    /// (`kinematics/extruder.py:180-205`).
    pub fn axes_r(&self) -> &[f64; 4] {
        &self.move_.axes_r
    }

    /// The total distance.
    pub fn move_d(&self) -> f64 {
        self.move_.move_d
    }

    /// Lower the speed and/or acceleration of this move.
    pub fn limit_speed(&mut self, speed: f64, accel: f64) {
        self.move_.limit_speed(speed, accel);
    }

    /// The error for a move outside the printable area.
    pub fn out_of_range(&self) -> CommandError {
        self.move_.move_error("Move out of range")
    }

    /// A move error carrying `msg`, with the move's end position
    /// (`Move.move_error`).
    pub fn move_error(&self, msg: &str) -> CommandError {
        self.move_.move_error(msg)
    }

    /// The error for a move before the axis is homed.
    pub fn must_home(&self) -> CommandError {
        self.move_.move_error("Must home axis first")
    }
}

/// Where an axis' endstop is and how to home it.
///
/// Upstream's `GenericPrinterRail.get_homing_info()` (`klippy/stepper.py:475`),
/// which the kinematics reads to compute a homing move's endpoints. It lives
/// here rather than beside the rail so the homing protocol below does not have
/// to reach up into the extras.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct HomingInfo {
    /// The speed of the first homing move, mm/s.
    pub speed: f64,
    /// Where the endstop sits, in axis coordinates.
    pub position_endstop: f64,
    /// The speed of the retract move.
    pub retract_speed: f64,
    /// How far to retract before the second home.
    pub retract_dist: f64,
    /// Whether homing moves toward increasing coordinates.
    pub positive_dir: bool,
    /// The speed of the second homing move.
    pub second_homing_speed: f64,
}

/// A homing endpoint: `None` means "keep the current value", as upstream's
/// `Coord`-of-`None` does (`Homing._fill_coord`).
pub type HomeCoord = [Option<f64>; AXES];

/// What a kinematics may ask the homing driver to do.
///
/// Upstream's `Homing` (`klippy/extras/homing.py`); the narrow version a
/// kinematics needs: which axes to home, where each axis' endstop is, and the
/// call that actually drives a rail's endstop.
pub trait HomingState {
    /// The axes to home, as `mathutil` indices.
    fn axes(&self) -> Vec<usize>;

    /// The homing parameters of an axis' rail.
    fn homing_info(&self, axis: usize) -> HomingInfo;

    /// Home `rails` (axis indices): move from `forcepos` to `movepos`, stopping
    /// on the rails' endstops.
    ///
    /// `forcepos` is where the toolhead is *pretended* to be before the move
    /// (it can be outside the travel); `movepos` is where the axis is being
    /// homed to.
    fn home_rails(&mut self, rails: &[usize], forcepos: HomeCoord, movepos: HomeCoord);
}

/// What the toolhead needs from its kinematics.
pub trait Kinematics: Send + Sync + std::fmt::Debug {
    /// The toolhead position from the stepper positions.
    ///
    /// An axis is `None` when it cannot be determined (not homed, or a
    /// non-invertible solver), which is what upstream reports
    /// (`extras/homing.py:245`).
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3];

    /// Whether the move is allowed, changing its limits if needed.
    ///
    /// # Errors
    /// The error the client sees when the move is refused.
    fn check_move(&self, ctx: &mut MoveContext<'_>) -> Result<(), CommandError>;

    /// Note that the toolhead is at `newpos`, with `homing_axes` now homed.
    fn set_position(&mut self, newpos: Coord, homing_axes: &[usize]);

    /// Update an axis' limits once it is homed.
    fn update_limits(&mut self, axis: usize, range: Option<(f64, f64)>);

    /// Mark axes as no longer homed.
    fn clear_homing_state(&mut self, axes: &[usize]);

    /// The kinematics' `get_status`.
    fn get_status(&self) -> Value;

    /// Home the requested axes (`CartKinematics.home`).
    ///
    /// Each axis is homed independently and in order; the kinematics computes
    /// the homing move's endpoints from the rail's range and `HomingInfo` and
    /// hands them to the driver.
    fn home(&mut self, homing: &mut dyn HomingState);
}

/// The homing move endpoints for one axis (`CartKinematics.home_axis`).
///
/// `homepos` is the endstop position; `forcepos` is pushed 1.5 axis-lengths past
/// the far end so the move always starts on the correct side of the endstop and
/// has room to accelerate.
pub fn home_move(
    axis: usize,
    info: &HomingInfo,
    position_min: f64,
    position_max: f64,
) -> (HomeCoord, HomeCoord) {
    let mut homepos: HomeCoord = [None; AXES];
    homepos[axis] = Some(info.position_endstop);
    let mut forcepos = homepos;
    forcepos[axis] = Some(if info.positive_dir {
        info.position_endstop - 1.5 * (info.position_endstop - position_min)
    } else {
        info.position_endstop + 1.5 * (position_max - info.position_endstop)
    });
    (forcepos, homepos)
}

/// The `none` kinematics: no steppers, no limits.
///
/// Upstream's `kinematics/none.py`, the developer/testing machine from
/// `[printer] kinematics: none`. It accepts every move and reports a fixed
/// position, so a config that only needs the toolhead's plumbing (dwells, output
/// scheduling) can run without axes.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoneKinematics;

impl Kinematics for NoneKinematics {
    fn calc_position(&self, _stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        [Some(0.0), Some(0.0), Some(0.0)]
    }

    fn check_move(&self, _ctx: &mut MoveContext<'_>) -> Result<(), CommandError> {
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, _homing_axes: &[usize]) {}

    fn update_limits(&mut self, _axis: usize, _range: Option<(f64, f64)>) {}

    fn clear_homing_state(&mut self, _axes: &[usize]) {}

    fn get_status(&self) -> Value {
        let zero = Coord::new(0.0, 0.0, 0.0, 0.0);
        json!({
            "homed_axes": "",
            "axis_minimum": zero.as_array(),
            "axis_maximum": zero.as_array(),
        })
    }

    fn home(&mut self, _homing: &mut dyn HomingState) {}
}

/// How a cartesian-family kinematics maps its three rail positions to the
/// carriage axes.
///
/// The families differ only in this mapping (plus how the motors are driven,
/// which the rails' solvers carry). Upstream writes each as its own class
/// (`kinematics/corexy.py`, `corexz.py`, `hybrid_corexy.py`, `hybrid_corexz.py`)
/// around the same limits/homing code; here the mapping is a value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CartesianTransform {
    /// `stepper_x` = x, `stepper_y` = y, `stepper_z` = z.
    Standard,
    /// CoreXY: `x = (ax + ay) / 2`, `y = (ax - ay) / 2`.
    CoreXy,
    /// CoreXZ: `x = (ax + az) / 2`, `z = (ax - az) / 2`.
    CoreXz,
    /// Hybrid CoreXY (Markforged): `x = ax + ay`, `y = ay`.
    HybridCoreXy,
    /// Hybrid CoreXZ: `x = ax + az`, `z = az`.
    HybridCoreXz,
}

/// Cartesian kinematics: one stepper per axis, straight-line limits.
///
/// Upstream's `CartKinematics` (`klippy/kinematics/cartesian.py`) and the
/// corexy/corexz/hybrid variants, which share its limits, homing and check
/// code and differ only by [`CartesianTransform`].
#[derive(Debug, Clone)]
pub struct CartesianKinematics {
    /// The stepper name of each of the three axes.
    axes: [String; 3],
    /// The physical travel of each axis.
    axes_min: Coord,
    axes_max: Coord,
    /// The homed range of each axis, or `None` while it is not homed.
    limits: [Option<(f64, f64)>; 3],
    max_z_velocity: f64,
    max_z_accel: f64,
    /// How rail positions map to the carriage's x/y/z.
    transform: CartesianTransform,
}

impl CartesianKinematics {
    /// A cartesian kinematics over `axes` (`stepper_x` …), travelling between
    /// `axes_min` and `axes_max`, with the Z axis limited to `max_z_velocity`
    /// and `max_z_accel`, and the given rail→carriage mapping.
    pub fn new(
        axes: [String; 3],
        axes_min: Coord,
        axes_max: Coord,
        max_z_velocity: f64,
        max_z_accel: f64,
        transform: CartesianTransform,
    ) -> Self {
        Self {
            axes,
            axes_min,
            axes_max,
            // Upstream starts with `(1.0, -1.0)`, an empty range that reads as
            // "not homed" (`klippy/kinematics/cartesian.py:44`).
            limits: [None; 3],
            max_z_velocity,
            max_z_accel,
            transform,
        }
    }

    /// The homed range of an axis, or `None`.
    pub fn limits(&self, axis: usize) -> Option<(f64, f64)> {
        self.limits[axis]
    }

    /// The error for a move past the end of a homed axis.
    fn endstop_error(&self, ctx: &MoveContext<'_>) -> CommandError {
        ctx.out_of_range()
    }

    /// Home one axis (`CartKinematics.home_axis`).
    ///
    /// `homepos` is the endstop position; `forcepos` is pushed 1.5 axis-lengths
    /// past the far end so the move always starts on the correct side of the
    /// endstop and has room to accelerate.
    fn home_axis(&self, homing: &mut dyn HomingState, axis: usize) {
        let info = homing.homing_info(axis);
        let (forcepos, homepos) = home_move(axis, &info, self.axes_min[axis], self.axes_max[axis]);
        homing.home_rails(&[axis], forcepos, homepos);
    }
}

impl Kinematics for CartesianKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        let rail = |axis: usize| stepper_positions.get(&self.axes[axis]).copied();
        let (Some(ax), Some(ay), Some(az)) = (rail(X_AXIS), rail(Y_AXIS), rail(Z_AXIS)) else {
            // At least one rail position is unknown, so the carriage cannot be
            // located; keep the per-axis `None` shape the caller expects.
            return [rail(X_AXIS), rail(Y_AXIS), rail(Z_AXIS)];
        };
        match self.transform {
            CartesianTransform::Standard => [Some(ax), Some(ay), Some(az)],
            CartesianTransform::CoreXy => [Some(0.5 * (ax + ay)), Some(0.5 * (ax - ay)), Some(az)],
            CartesianTransform::CoreXz => [Some(0.5 * (ax + az)), Some(ay), Some(0.5 * (ax - az))],
            CartesianTransform::HybridCoreXy => [Some(ax + ay), Some(ay), Some(az)],
            CartesianTransform::HybridCoreXz => [Some(ax + az), Some(ay), Some(az)],
        }
    }

    fn check_move(&self, ctx: &mut MoveContext<'_>) -> Result<(), CommandError> {
        let end = *ctx.end_pos();
        let axes_d = *ctx.axes_d();
        // XY first, as upstream does (`klippy/kinematics/cartesian.py:104-112`).
        for axis in [X_AXIS, Y_AXIS] {
            if axes_d[axis] == 0.0 {
                continue;
            }
            match self.limits[axis] {
                Some((low, high)) => {
                    if end[axis] < low || end[axis] > high {
                        return Err(self.endstop_error(ctx));
                    }
                }
                // Not homed: a move on this axis must be refused.
                None => return Err(ctx.must_home()),
            }
        }
        if axes_d[Z_AXIS] == 0.0 {
            return Ok(());
        }
        match self.limits[Z_AXIS] {
            Some((low, high)) => {
                if end[Z_AXIS] < low || end[Z_AXIS] > high {
                    return Err(self.endstop_error(ctx));
                }
            }
            None => return Err(ctx.must_home()),
        }
        // A move with a Z component is slowed so Z itself does not exceed its
        // own limits.
        let z_ratio = ctx.move_d() / axes_d[Z_AXIS].abs();
        ctx.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio);
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, homing_axes: &[usize]) {
        for axis in homing_axes {
            self.limits[*axis] = Some((self.axes_min[*axis], self.axes_max[*axis]));
        }
    }

    fn update_limits(&mut self, axis: usize, range: Option<(f64, f64)>) {
        if let Some(range) = range {
            // Only a homed axis gets new limits, as upstream does
            // (`klippy/kinematics/cartesian.py:63-68`).
            if self.limits[axis].is_some() {
                self.limits[axis] = Some(range);
            }
        }
    }

    fn clear_homing_state(&mut self, axes: &[usize]) {
        for axis in axes {
            self.limits[*axis] = None;
        }
    }

    fn get_status(&self) -> Value {
        let homed: String = ["x", "y", "z"]
            .iter()
            .enumerate()
            .filter(|(axis, _)| self.limits[*axis].is_some())
            .map(|(_, name)| *name)
            .collect();
        json!({
            "homed_axes": homed,
            "axis_minimum": self.axes_min.as_array(),
            "axis_maximum": self.axes_max.as_array(),
        })
    }

    fn home(&mut self, homing: &mut dyn HomingState) {
        // Each axis independently, in the order the driver asks for.
        for axis in homing.axes() {
            self.home_axis(homing, axis);
        }
    }
}

// ===========================================================================
// Polar (`kinematics/polar.py`, `chelper/kin_polar.c`)
// ===========================================================================

/// The radius solver for one arm stepper (`polar_stepper_alloc('r')`,
/// `kin_polar.c:9-15`): the toolhead's distance from the bed center.
pub fn polar_radius_position(segment: &MoveSegment, move_time: f64) -> f64 {
    let c = segment.coord(move_time);
    (c.x() * c.x() + c.y() * c.y()).sqrt()
}

/// The angle solver for the bed stepper (`polar_stepper_alloc('a')`,
/// `kin_polar.c:17-24`): the toolhead's bearing, **raw** `atan2` — the ±2π
/// unwrap against `commanded_pos` happens through the solver's `unwrap`
/// hook (upstream's callback reads `sk->commanded_pos` itself).
pub fn polar_angle_position(segment: &MoveSegment, move_time: f64) -> f64 {
    let c = segment.coord(move_time);
    c.y().atan2(c.x())
}

/// Both polar steppers follow X and Y (`kin_polar.c:46` sets `AF_X | AF_Y`
/// for the angle **and** the radius solver).
pub const fn polar_active_flags() -> crate::core::klippy::motion::itersolve::AxisFlags {
    crate::core::klippy::motion::itersolve::AxisFlags::X
        .union(crate::core::klippy::motion::itersolve::AxisFlags::Y)
}

/// The bed angle unwrap (`polar_stepper_angle_calc_position`,
/// `kin_polar.c:17-24`): shift the wrapped `atan2` result by ±2π so it stays
/// continuous with the commanded position — this is the solver's `unwrap`
/// hook (`StepKinematics::set_hooks`), evaluated against `commanded_pos`
/// exactly as upstream's callback reads `sk->commanded_pos`.
pub fn polar_angle_unwrap(raw: f64, commanded: f64) -> f64 {
    if raw - commanded > std::f64::consts::PI {
        raw - 2.0 * std::f64::consts::PI
    } else if raw - commanded < -std::f64::consts::PI {
        raw + 2.0 * std::f64::consts::PI
    } else {
        raw
    }
}

/// Renormalize the commanded bed angle into [-π, π] after each generated
/// range (`polar_stepper_angle_post_fixup` called as `post_cb` from
/// `itersolve.c:124-126`): the solver's `post` hook. Without it the
/// commanded position would grow past ±3π over several rotations and a
/// single ±2π unwrap would no longer reach it.
pub fn polar_angle_normalize(commanded: &mut f64) {
    if *commanded < -std::f64::consts::PI {
        *commanded += 2.0 * std::f64::consts::PI;
    } else if *commanded > std::f64::consts::PI {
        *commanded -= 2.0 * std::f64::consts::PI;
    }
}

/// The closest distance from the bed center (origin) to the segment
/// `p1 → p2` (`distance_to_center`, `kinematics/polar.py:5-21`): which part
/// of the segment a slowdown applies to — the segment start, its end, or
/// the perpendicular foot.
pub fn distance_to_center(p1: (f64, f64), p2: (f64, f64)) -> f64 {
    let ab_x = p2.0 - p1.0;
    let ab_y = p2.1 - p1.1;
    let ap_x = -p1.0;
    let ap_y = -p1.1;
    let dot = ab_x * ap_x + ab_y * ap_y;
    let ab_length_sq = ab_x * ab_x + ab_y * ab_y;
    if dot <= 0.0 {
        // The projection of the center falls before `p1`: closest is `p1`.
        (ap_x * ap_x + ap_y * ap_y).sqrt()
    } else if dot >= ab_length_sq {
        // …or beyond `p2`: closest is `p2` itself.
        (p2.0 * p2.0 + p2.1 * p2.1).sqrt()
    } else {
        // Otherwise the perpendicular foot lies on the segment.
        (ab_x * ap_y - ab_y * ap_x).abs() / ab_length_sq.sqrt()
    }
}

/// The homing endpoints for one polar axis (`_home_axis`,
/// `kinematics/polar.py:64-79`): 1.0× push (not the cartesian 1.5× —
/// overshooting the arm's `position_min` would put the toolhead *behind*
/// the center, where the angle flips), and when homing the arm (axis 0)
/// **Y is pinned to 0** so the drip move stays on the +X radius.
pub fn polar_home_move(
    axis: usize,
    info: &HomingInfo,
    position_min: f64,
    position_max: f64,
) -> (HomeCoord, HomeCoord) {
    let mut homepos: HomeCoord = [None; AXES];
    homepos[axis] = Some(info.position_endstop);
    if axis == X_AXIS {
        homepos[Y_AXIS] = Some(0.0);
    }
    let mut forcepos = homepos;
    forcepos[axis] = Some(if info.positive_dir {
        info.position_endstop - (info.position_endstop - position_min)
    } else {
        info.position_endstop + (position_max - info.position_endstop)
    });
    (forcepos, homepos)
}

/// The polar kinematics: a rotating bed (angle stepper), a sliding arm
/// (radius stepper) and a cartesian Z rail.
///
/// Upstream's `PolarKinematics` (`klippy/kinematics/polar.py`). The carriage
/// position is `(cos(bed) · arm, sin(bed) · arm, z)`; X/Y travel is the
/// square `[-max_xy, max_xy]²` gated through `limit_xy2` (one squared
/// radius, set only once **both** axes are homed — X and Y cannot be
/// homed or cleared separately), and the Z range through `limit_z`.
#[derive(Debug, Clone)]
pub struct PolarKinematics {
    /// The stepper names `calc_position` reads: bed angle, arm radius, z.
    bed: String,
    arm: String,
    z: String,
    /// The arm rail's radius range and the Z rail's range.
    arm_range: (f64, f64),
    z_range: (f64, f64),
    /// The printable square and height, for `get_status`.
    axes_min: Coord,
    axes_max: Coord,
    /// The machine's velocity limits; the near-center slowdown scales both.
    limits: super::plan::MoveLimits,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// `[printer] max_angular_velocity`: the cap near the bed center
    /// (`0` = uncapped, upstream's default).
    v_rad_max: f64,
    /// The homed Z range, or an inverted range while unhomed
    /// (upstream's `(1.0, -1.0)` sentinel).
    limit_z: (f64, f64),
    /// The squared radius X/Y may reach, or `-1.0` while unhomed.
    limit_xy2: f64,
}

impl PolarKinematics {
    /// The polar kinematics over its two rails and the bed stepper
    /// (`PolarKinematics.__init__`, `kinematics/polar.py:14-59`).
    ///
    /// `names` are the stepper names for [`calc_position`](Self::calc_position)
    /// (bed, arm, z), `arm_range`/`z_range` the rail ranges.
    pub fn new(
        names: [String; 3],
        arm_range: (f64, f64),
        z_range: (f64, f64),
        limits: super::plan::MoveLimits,
        max_z_velocity: f64,
        max_z_accel: f64,
        v_rad_max: f64,
    ) -> Self {
        let max_xy = arm_range.1;
        let [bed, arm, z] = names;
        Self {
            bed,
            arm,
            z,
            arm_range,
            z_range,
            axes_min: Coord::new(-max_xy, -max_xy, z_range.0, 0.0),
            axes_max: Coord::new(max_xy, max_xy, z_range.1, 0.0),
            limits,
            max_z_velocity,
            max_z_accel,
            v_rad_max,
            // Upstream starts every limit unhomed: an inverted z range and a
            // negative squared radius that no move can pass
            // (`kinematics/polar.py:57-58`).
            limit_z: (1.0, -1.0),
            limit_xy2: -1.0,
        }
    }
}

impl Kinematics for PolarKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        // `calc_position` of `kinematics/polar.py:61-65`: the carriage is
        // the bed angle and arm radius in polar coordinates. An axis is
        // `None` when a stepper it needs is missing (the port's convention;
        // upstream would raise a `KeyError`).
        let (Some(bed), Some(arm)) = (
            stepper_positions.get(&self.bed).copied(),
            stepper_positions.get(&self.arm).copied(),
        ) else {
            return [None, None, stepper_positions.get(&self.z).copied()];
        };
        [
            Some(bed.cos() * arm),
            Some(bed.sin() * arm),
            stepper_positions.get(&self.z).copied(),
        ]
    }

    fn check_move(&self, ctx: &mut MoveContext<'_>) -> Result<(), CommandError> {
        // `check_move` of `kinematics/polar.py:104-139`, in upstream order:
        // the gated XY radius, then Z's range and speed, then the
        // near-center angular slowdown.
        let end = *ctx.end_pos();
        let axes_d = *ctx.axes_d();
        let xy2 = end.x() * end.x() + end.y() * end.y();
        if xy2 > self.limit_xy2 {
            if self.limit_xy2 < 0.0 {
                return Err(ctx.must_home());
            }
            return Err(ctx.out_of_range());
        }
        if axes_d[Z_AXIS] != 0.0 {
            if end.z() < self.limit_z.0 || end.z() > self.limit_z.1 {
                if self.limit_z.0 > self.limit_z.1 {
                    return Err(ctx.must_home());
                }
                return Err(ctx.out_of_range());
            }
            // A move with Z is slowed so Z itself stays within its own
            // speed, by the share of the move it covers.
            let z_ratio = ctx.move_d() / axes_d[Z_AXIS].abs();
            ctx.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio);
        }
        if axes_d[X_AXIS] != 0.0 || axes_d[Y_AXIS] != 0.0 {
            if self.v_rad_max == 0.0 {
                return Ok(());
            }
            let min_dist = distance_to_center(
                (ctx.start_pos().x(), ctx.start_pos().y()),
                (end.x(), end.y()),
            );
            if min_dist == 0.0 {
                return Ok(());
            }
            // Angular speed at the closest point: linear speed over radius.
            let v_angular = ctx.max_cruise_v2().sqrt() / min_dist;
            if self.v_rad_max < v_angular {
                let scale_radius = self.v_rad_max / v_angular;
                ctx.limit_speed(
                    self.limits.max_velocity * scale_radius,
                    self.limits.max_accel * scale_radius,
                );
            }
        }
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, homing_axes: &[usize]) {
        // `set_position` of `kinematics/polar.py:81-86`: the stepper
        // positions are set centrally by the toolhead; only the homed
        // limits live here. X **and** Y together open the whole square
        // (a single axis would leave `limit_xy2` untouched).
        if homing_axes.contains(&Z_AXIS) {
            self.limit_z = self.z_range;
        }
        if homing_axes.contains(&X_AXIS) && homing_axes.contains(&Y_AXIS) {
            self.limit_xy2 = self.arm_range.1 * self.arm_range.1;
        }
    }

    fn update_limits(&mut self, _axis: usize, _range: Option<(f64, f64)>) {
        // Upstream's polar kinematics defines no `update_limits` — only the
        // cartesian dual-carriage swap calls it, and polar has no dual
        // carriage.
    }

    fn clear_homing_state(&mut self, axes: &[usize]) {
        // `clear_homing_state` of `kinematics/polar.py:88-93`: "X and Y
        // cannot be cleared separately", so either clears both.
        if axes.contains(&X_AXIS) || axes.contains(&Y_AXIS) {
            self.limit_xy2 = -1.0;
        }
        if axes.contains(&Z_AXIS) {
            self.limit_z = (1.0, -1.0);
        }
    }

    fn get_status(&self) -> Value {
        // `get_status` of `kinematics/polar.py:141-151`.
        let xy_home = if self.limit_xy2 >= 0.0 { "xy" } else { "" };
        let z_home = if self.limit_z.0 <= self.limit_z.1 {
            "z"
        } else {
            ""
        };
        json!({
            "homed_axes": format!("{xy_home}{z_home}"),
            "axis_minimum": self.axes_min.as_array(),
            "axis_maximum": self.axes_max.as_array(),
        })
    }

    fn home(&mut self, homing: &mut dyn HomingState) {
        // `home` of `kinematics/polar.py:95-108`: X and Y are always homed
        // together on the arm rail (axis 0, Y pinned to 0 by
        // [`polar_home_move`]), then Z on its own rail.
        let requested = homing.axes();
        let home_xy = requested.contains(&X_AXIS) || requested.contains(&Y_AXIS);
        if home_xy {
            let info = homing.homing_info(X_AXIS);
            let (forcepos, movepos) =
                polar_home_move(X_AXIS, &info, self.arm_range.0, self.arm_range.1);
            homing.home_rails(&[X_AXIS], forcepos, movepos);
        }
        if requested.contains(&Z_AXIS) {
            let info = homing.homing_info(Z_AXIS);
            let (forcepos, movepos) =
                polar_home_move(Z_AXIS, &info, self.z_range.0, self.z_range.1);
            homing.home_rails(&[Z_AXIS], forcepos, movepos);
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::E_AXIS;
    use crate::core::klippy::motion::plan::MoveLimits;

    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    fn kinematics() -> CartesianKinematics {
        CartesianKinematics::new(
            [
                "stepper_x".to_string(),
                "stepper_y".to_string(),
                "stepper_z".to_string(),
            ],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            CartesianTransform::Standard,
        )
    }

    fn move_(start: Coord, end: Coord) -> Move {
        Move::new(start, end, 100.0, &limits())
    }

    #[test]
    fn test_a_move_before_homing_is_refused() {
        let kin = kinematics();
        let mut move_ = move_(Coord::default(), Coord::new(10.0, 0.0, 0.0, 0.0));

        let mut ctx = MoveContext::new(&mut move_);
        let err = kin.check_move(&mut ctx).unwrap_err();

        assert!(err.to_string().contains("Must home axis first"), "{err}");
    }

    #[test]
    fn test_a_homed_axis_accepts_a_move_inside_its_range() {
        let mut kin = kinematics();
        kin.set_position(Coord::default(), &[X_AXIS]);
        let mut move_ = move_(Coord::default(), Coord::new(10.0, 0.0, 0.0, 0.0));

        let mut ctx = MoveContext::new(&mut move_);
        assert!(kin.check_move(&mut ctx).is_ok());
    }

    #[test]
    fn test_a_homed_axis_refuses_a_move_past_its_range() {
        let mut kin = kinematics();
        kin.set_position(Coord::default(), &[X_AXIS]);
        let mut move_ = move_(Coord::default(), Coord::new(250.0, 0.0, 0.0, 0.0));

        let mut ctx = MoveContext::new(&mut move_);
        let err = kin.check_move(&mut ctx).unwrap_err();

        assert!(err.to_string().contains("Move out of range"), "{err}");
    }

    #[test]
    fn test_a_diagonal_move_is_slowed_for_the_z_axis() {
        let mut kin = kinematics();
        kin.set_position(Coord::default(), &[X_AXIS, Y_AXIS, Z_AXIS]);
        // Mostly Z: the move would otherwise drive Z far faster than 15 mm/s.
        let mut move_ = move_(Coord::default(), Coord::new(1.0, 0.0, 10.0, 0.0));

        let mut ctx = MoveContext::new(&mut move_);
        kin.check_move(&mut ctx).unwrap();

        // Z's 15 mm/s becomes a cap of 15 * (move_d / |axes_d[z]|).
        let expected = 15.0 * (move_.move_d / 10.0);
        assert!(
            (move_.max_cruise_v2 - expected * expected).abs() < 1e-6,
            "{}",
            move_.max_cruise_v2
        );
    }

    #[test]
    fn test_corexy_maps_rail_positions_to_the_carriage() {
        let kin = CartesianKinematics::new(
            [
                "stepper_x".to_string(),
                "stepper_y".to_string(),
                "stepper_z".to_string(),
            ],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            CartesianTransform::CoreXy,
        );
        let positions: HashMap<String, f64> = [
            ("stepper_x".to_string(), 12.0),
            ("stepper_y".to_string(), 4.0),
            ("stepper_z".to_string(), 3.0),
        ]
        .into_iter()
        .collect();

        assert_eq!(
            kin.calc_position(&positions),
            [Some(8.0), Some(4.0), Some(3.0)]
        );
    }

    #[test]
    fn test_corexz_maps_rail_positions_to_the_carriage() {
        let kin = CartesianKinematics::new(
            [
                "stepper_x".to_string(),
                "stepper_y".to_string(),
                "stepper_z".to_string(),
            ],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            CartesianTransform::CoreXz,
        );
        let positions: HashMap<String, f64> = [
            ("stepper_x".to_string(), 12.0),
            ("stepper_y".to_string(), 7.0),
            ("stepper_z".to_string(), 4.0),
        ]
        .into_iter()
        .collect();

        assert_eq!(
            kin.calc_position(&positions),
            [Some(8.0), Some(7.0), Some(4.0)]
        );
    }

    #[test]
    fn test_the_hybrid_families_map_only_x() {
        let positions: HashMap<String, f64> = [
            ("stepper_x".to_string(), 12.0),
            ("stepper_y".to_string(), 4.0),
            ("stepper_z".to_string(), 3.0),
        ]
        .into_iter()
        .collect();
        let axes = [
            "stepper_x".to_string(),
            "stepper_y".to_string(),
            "stepper_z".to_string(),
        ];

        // Hybrid CoreXY: x = ax + ay, y = ay.
        let kin = CartesianKinematics::new(
            axes.clone(),
            Coord::default(),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            CartesianTransform::HybridCoreXy,
        );
        assert_eq!(
            kin.calc_position(&positions),
            [Some(16.0), Some(4.0), Some(3.0)]
        );

        // Hybrid CoreXZ: x = ax + az, z = az.
        let kin = CartesianKinematics::new(
            axes,
            Coord::default(),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            CartesianTransform::HybridCoreXz,
        );
        assert_eq!(
            kin.calc_position(&positions),
            [Some(15.0), Some(4.0), Some(3.0)]
        );
    }

    #[test]
    fn test_an_extrude_only_move_is_not_the_kinematics_business() {
        let kin = kinematics();
        let mut move_ = move_(Coord::default(), Coord::new(0.0, 0.0, 0.0, 5.0));

        // The caller only runs `check_move` for kinematic moves, but the
        // kinematics itself must not refuse it either.
        assert!(!move_.is_kinematic_move);
        let mut ctx = MoveContext::new(&mut move_);
        assert!(kin.check_move(&mut ctx).is_ok());
        assert_eq!(move_.axes_d[E_AXIS], 5.0);
    }

    #[test]
    fn test_status_reports_the_homed_axes() {
        let mut kin = kinematics();
        assert_eq!(kin.get_status()["homed_axes"], "");
        kin.set_position(Coord::default(), &[X_AXIS, Z_AXIS]);
        assert_eq!(kin.get_status()["homed_axes"], "xz");
        kin.clear_homing_state(&[X_AXIS]);
        assert_eq!(kin.get_status()["homed_axes"], "z");
    }

    #[test]
    fn test_calc_position_reads_the_axis_steppers() {
        let kin = kinematics();
        let positions = HashMap::from([
            ("stepper_x".to_string(), 1.0),
            ("stepper_y".to_string(), 2.0),
        ]);

        let out = kin.calc_position(&positions);

        assert_eq!(out, [Some(1.0), Some(2.0), None]);
    }

    #[test]
    fn test_none_kinematics_accepts_every_move() {
        let kin = NoneKinematics;
        let mut move_ = move_(Coord::default(), Coord::new(10.0, 10.0, 10.0, 5.0));

        let mut ctx = MoveContext::new(&mut move_);
        kin.check_move(&mut ctx).unwrap();
        assert_eq!(kin.get_status()["homed_axes"], "");
        assert_eq!(kin.calc_position(&HashMap::new()), [Some(0.0); 3]);
    }

    /// A `HomingState` that records the calls a kinematics makes.
    struct FakeHoming {
        axes: Vec<usize>,
        info: [HomingInfo; 3],
        calls: Vec<(Vec<usize>, HomeCoord, HomeCoord)>,
    }

    impl HomingState for FakeHoming {
        fn axes(&self) -> Vec<usize> {
            self.axes.clone()
        }
        fn homing_info(&self, axis: usize) -> HomingInfo {
            self.info[axis]
        }
        fn home_rails(&mut self, rails: &[usize], forcepos: HomeCoord, movepos: HomeCoord) {
            self.calls.push((rails.to_vec(), forcepos, movepos));
        }
    }

    fn homing_info(position_endstop: f64, positive_dir: bool) -> HomingInfo {
        HomingInfo {
            speed: 5.0,
            position_endstop,
            retract_speed: 5.0,
            retract_dist: 5.0,
            positive_dir,
            second_homing_speed: 2.5,
        }
    }

    #[test]
    fn test_home_computes_the_force_and_move_positions() {
        let mut kin = kinematics();
        let mut homing = FakeHoming {
            // Home X then Y.
            axes: vec![X_AXIS, Y_AXIS],
            info: [
                homing_info(0.0, false),
                homing_info(200.0, true),
                homing_info(0.0, false),
            ],
            calls: Vec::new(),
        };

        kin.home(&mut homing);

        assert_eq!(homing.calls.len(), 2, "one call per axis");
        // X homes negative to 0; forcepos is pushed 1.5 axis-lengths past 200.
        let (rails, forcepos, movepos) = &homing.calls[0];
        assert_eq!(rails, &[X_AXIS]);
        assert_eq!(forcepos[X_AXIS], Some(300.0));
        assert_eq!(movepos[X_AXIS], Some(0.0));
        // Y homes positive to 200; forcepos is pushed below 0.
        let (rails, forcepos, movepos) = &homing.calls[1];
        assert_eq!(rails, &[Y_AXIS]);
        assert_eq!(forcepos[Y_AXIS], Some(-100.0));
        assert_eq!(movepos[Y_AXIS], Some(200.0));
        // The other axes are left `None` for the driver to fill in.
        assert_eq!(forcepos[X_AXIS], None);
        assert_eq!(movepos[X_AXIS], None);
    }
}
