//! `kinematics: deltesian` — the deltesian kinematics and its solver.
//!
//! Upstream spreads this over two places, which is why this file carries both:
//!
//! | upstream | here |
//! |---|---|
//! | `deltesian_stepper_alloc` (`chelper/kin_deltesian.c:17-40`) | [`deltesian_position_fn`] / [`deltesian_active_flags`] |
//! | `DeltesianKinematics` (`klippy/kinematics/deltesian.py:11-136`) | [`DeltesianKinematics`] |
//!
//! **Placement.** Like [`super::delta`] and [`super::rotary_delta`], this is its
//! own file so the cartesian family's code stays untouched. Deltesian is a
//! hybrid: two rotating arm towers (`stepper_left`/`stepper_right`) place the
//! carriage in X and Z, and one straight rail (`stepper_y`) is the Y axis. There
//! is no calibration half — upstream's `DeltesianKinematics` exposes no
//! `get_calibration` — so this file is the kinematics alone.
//!
//! # The geometry
//!
//! Each arm's stepper position is the actuator distance `sqrt(arm² − (x −
//! arm_x)²) + z` for the carriage at `(x, z)` (the C solver); the two signed
//! `arm_x` offsets are `−arm_x_length` on the left and `+arm_x_length` on the
//! right. Going the other way, the two arms' endstop heights are two spheres
//! whose intersection is the carriage position ([`DeltesianKinematics::actuator_to_cartesian`]).
//! `arm_length` and `arm_x_length` are options of the arm sections, not of
//! `[printer]`, which is why the toolhead's builder reads them from the sibling
//! sections.

use std::collections::HashMap;

use serde_json::{json, Value};
use tracing::info;

use super::itersolve::{AxisFlags, PositionFn};
use super::kinematics::{DeltesianHome, HomingState, Kinematics, MoveContext};
use super::trapq::MoveSegment;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, AXES, X_AXIS, Y_AXIS, Z_AXIS};

/// Slow moves once the ratio of arm to XY movement exceeds this
/// (`SLOW_RATIO`, `deltesian.py:9`).
pub const SLOW_RATIO: f64 = 3.;

/// The minimum angle with the horizontal for the arm, in degrees
/// (`MIN_ANGLE`, `deltesian.py:12`).
pub const MIN_ANGLE: f64 = 5.;

/// The stepper names the three rails answer to (`deltesian.py:15`), in rail
/// order: the two arms, then the Y rail.
pub const DELTESIAN_RAIL_NAMES: [&str; 3] = ["stepper_left", "stepper_right", "stepper_y"];

// ===========================================================================
// deltesian_stepper_alloc
// ===========================================================================

/// `deltesian_stepper_alloc(arm2, arm_x)` (`chelper/kin_deltesian.c:17-40`):
/// where the arm's actuator is for the carriage at the segment's position.
///
/// Parameters ride the bound [`PositionFn`]: `[arm2, arm_x]`. The left arm is
/// allocated with `−arm_x_length` and the right with `+arm_x_length`
/// (`deltesian.py:30-34`), so the single signed value carries the side.
pub fn deltesian_position_fn(arm2: f64, arm_x: f64) -> PositionFn {
    PositionFn::bind(deltesian_calc_position, [arm2, arm_x, 0.0, 0.0, 0.0, 0.0])
}

/// The solver body: `sqrt(arm2 − (x − arm_x)²) + z`
/// (`deltesian_stepper_calc_position`, `kin_deltesian.c:22-31`).
fn deltesian_calc_position(segment: &MoveSegment, move_time: f64, params: &[f64; 6]) -> f64 {
    let c = segment.coord(move_time);
    let dx = c.x() - params[1];
    (params[0] - dx * dx).sqrt() + c.z()
}

/// Each arm stepper moves on X and Z (`AF_X | AF_Z`, `kin_deltesian.c:38`).
pub fn deltesian_active_flags() -> AxisFlags {
    AxisFlags::X.union(AxisFlags::Z)
}

// ===========================================================================
// Geometry helpers (`DeltesianKinematics.__init__` derived values)
// ===========================================================================

/// The two arms' endstop heights (`self._abs_endstop`, `deltesian.py:72-73`):
/// each arm rail's `position_endstop` plus the vertical reach
/// `sqrt(arm2 − arm_x²)`.
pub fn arm_abs_endstops(arm_endstops: [f64; 2], arm_x: [f64; 2], arm2: [f64; 2]) -> [f64; 2] {
    [
        arm_endstops[0] + (arm2[0] - arm_x[0] * arm_x[0]).sqrt(),
        arm_endstops[1] + (arm2[1] - arm_x[1] * arm_x[1]).sqrt(),
    ]
}

/// The X travel the `min_angle` limit allows (`x_kin_min`/`x_kin_max`,
/// `deltesian.py:59-62`).
pub fn x_kin_limits(min_angle: f64, arm_x: [f64; 2], arm: [f64; 2]) -> (f64, f64) {
    let cos_angle = min_angle.to_radians().cos();
    let x_kin_min = -(arm_x[0].min(cos_angle * arm[1] - arm_x[1])).ceil();
    let x_kin_max = (arm_x[1].min(cos_angle * arm[0] - arm_x[0])).floor();
    (x_kin_min, x_kin_max)
}

/// The highest Z the arms allow at `x` (`_pillars_z_max`, `deltesian.py:78-82`).
pub fn pillars_z_max(arm_x: [f64; 2], arm2: [f64; 2], abs_endstop: [f64; 2], x: f64) -> f64 {
    let dz = [
        (arm2[0] - (arm_x[0] + x) * (arm_x[0] + x)).sqrt(),
        (arm2[1] - (arm_x[1] - x) * (arm_x[1] - x)).sqrt(),
    ];
    (abs_endstop[0] - dz[0]).min(abs_endstop[1] - dz[1])
}

// ===========================================================================
// DeltesianConfig → DeltesianKinematics
// ===========================================================================

/// What `[printer]` and its `[stepper_left]`/`[stepper_right]`/`[stepper_y]`
/// sections say about a deltesian machine, as plain numbers the config readers
/// already bounds-checked (`kinematics/deltesian.py:11-45` reads the same
/// options).
#[derive(Debug, Clone)]
pub struct DeltesianConfig {
    /// `arm_x_length` per arm: `stepper_left` required, `stepper_right`
    /// defaults to it; all above 0.
    pub arm_x: [f64; 2],
    /// `arm_length` per arm: `stepper_left` required and above its
    /// `arm_x_length`, `stepper_right` defaults to it.
    pub arm2: [f64; 2],
    /// Each arm rail's `position_endstop` (`stepper_right` defaults to
    /// `stepper_left`'s).
    pub arm_endstops: [f64; 2],
    /// The Y rail's travel range (`stepper_y`'s `position_min`/`position_max`).
    pub y_range: (f64, f64),
    /// `min_angle`, default [`MIN_ANGLE`], between 0 and 90.
    pub min_angle: f64,
    /// `print_width`, default `None`, between 0 and the arms' X range; a
    /// non-zero value centres the X limits on the origin.
    pub print_width: Option<f64>,
    /// `minimum_z_position`, default 0, at most the arms' highest Z.
    pub minimum_z_position: f64,
    /// `slow_ratio`, default [`SLOW_RATIO`], at least 0 (`0` disables the
    /// edge slowdown).
    pub slow_ratio: f64,
    /// `[printer] max_velocity`.
    pub max_velocity: f64,
    /// `[printer] max_accel`.
    pub max_accel: f64,
    /// `max_z_velocity`, default `max_velocity`.
    pub max_z_velocity: f64,
    /// `max_z_accel`, default `max_accel`.
    pub max_z_accel: f64,
}

/// Deltesian kinematics: two arms placing X/Z, one straight Y rail,
/// simultaneous arm homing, and an X-dependent Z ceiling
/// (`DeltesianKinematics`, `kinematics/deltesian.py:11-136`).
#[derive(Debug)]
pub struct DeltesianKinematics {
    /// Each arm's signed `arm_x` offset (`−left`, `+right`).
    arm_x: [f64; 2],
    /// Each arm's squared length.
    arm2: [f64; 2],
    /// The Y rail's travel range (`deltesian.py:70`).
    y_range: (f64, f64),
    /// X, Y and Z limits (`deltesian.py:67-76`).
    limits: [(f64, f64); 3],
    /// The printable box, for `get_status`.
    axes_min: Coord,
    axes_max: Coord,
    /// Each arm's endstop height (`self._abs_endstop`).
    abs_endstop: [f64; 2],
    /// The carriage Z at home (`self.home_z`).
    home_z: f64,
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// The squared X past which moves slow, or `None` when `slow_ratio` is 0
    /// (`self.slow_x2`/`self.very_slow_x2`).
    slow_x2: Option<f64>,
    very_slow_x2: Option<f64>,
    /// Which axes are homed (`self.homed_axis`).
    homed_axis: [bool; 3],
}

impl DeltesianKinematics {
    /// Build the kinematics from the config's numbers
    /// (`DeltesianKinematics.__init__`, `deltesian.py:11-86`), mirroring its
    /// bounds, derived envelope and log line.
    pub fn new(config: DeltesianConfig) -> Self {
        let arm_x = config.arm_x;
        let arm2 = config.arm2;
        let arm = [arm2[0].sqrt(), arm2[1].sqrt()];

        // X axis limits (`deltesian.py:57-69`): the arms' reach at the
        // `min_angle` limit, optionally centred by `print_width`.
        let (x_kin_min, x_kin_max) = x_kin_limits(config.min_angle, arm_x, arm);
        let mut limits = [(1.0, -1.0); 3];
        limits[0] = match config.print_width {
            Some(width) if width != 0.0 => (-width * 0.5, width * 0.5),
            _ => (x_kin_min, x_kin_max),
        };
        // Y axis limits: the straight rail's own range (`deltesian.py:70`).
        limits[1] = config.y_range;
        // Z axis limits (`deltesian.py:71-76`): the carriage Z where both arm
        // endstops sit, and the arms' highest Z over the X range.
        let abs_endstop = arm_abs_endstops(config.arm_endstops, arm_x, arm2);
        let home_z = Self::actuator_to_cartesian_of(arm_x, arm2, abs_endstop)[1];
        let z_max = pillars_z_max(arm_x, arm2, abs_endstop, limits[0].0).min(pillars_z_max(
            arm_x,
            arm2,
            abs_endstop,
            limits[0].1,
        ));
        limits[2] = (config.minimum_z_position, z_max);

        // The X edge slowdown (`deltesian.py:78-86`).
        let (slow_x2, very_slow_x2) = if config.slow_ratio > 0.0 {
            let sr2 = config.slow_ratio * config.slow_ratio;
            let reach = |scale: f64| {
                (0..2)
                    .map(|i| ((scale * sr2 * arm2[i]) / (scale * sr2 + 1.0)).sqrt() - arm_x[i])
                    .fold(f64::INFINITY, f64::min)
                    .powi(2)
            };
            let slow_x2 = reach(1.0);
            let very_slow_x2 = reach(2.0);
            info!(
                "Deltesian kinematics: moves slowed past {:.2}mm and {:.2}mm",
                slow_x2.sqrt(),
                very_slow_x2.sqrt()
            );
            (Some(slow_x2), Some(very_slow_x2))
        } else {
            (None, None)
        };

        let axes_min = Coord::from_axes(limits.iter().map(|limit| limit.0));
        let axes_max = Coord::from_axes(limits.iter().map(|limit| limit.1));
        Self {
            arm_x,
            arm2,
            y_range: config.y_range,
            limits,
            axes_min,
            axes_max,
            abs_endstop,
            home_z,
            max_velocity: config.max_velocity,
            max_accel: config.max_accel,
            max_z_velocity: config.max_z_velocity,
            max_z_accel: config.max_z_accel,
            slow_x2,
            very_slow_x2,
            homed_axis: [false; 3],
        }
    }

    /// The parameters each arm rail's solver is bound with, in rail order
    /// (`setup_itersolve('deltesian_stepper_alloc', arm2[i], ±arm_x[i])`,
    /// `deltesian.py:30-34`): `(arm2, signed arm_x)`.
    pub fn arm_geometry(&self) -> [(f64, f64); 2] {
        [
            (self.arm2[0], -self.arm_x[0]),
            (self.arm2[1], self.arm_x[1]),
        ]
    }

    /// The carriage `(x, z)` for the two arm actuator positions
    /// (`_actuator_to_cartesian`, `deltesian.py:45-56`).
    ///
    /// The two arms are trilaterated in the frame along the left-to-right
    /// pivots, then rotated and shifted back.
    pub fn actuator_to_cartesian(&self, sp: [f64; 2]) -> [f64; 2] {
        Self::actuator_to_cartesian_of(self.arm_x, self.arm2, sp)
    }

    /// [`Self::actuator_to_cartesian`] over plain numbers, so the constructor
    /// and [`Self::deltesian_home`] can share it.
    fn actuator_to_cartesian_of(arm_x: [f64; 2], arm2: [f64; 2], sp: [f64; 2]) -> [f64; 2] {
        let dx = arm_x[0] + arm_x[1];
        let dz = sp[1] - sp[0];
        let pivots = (dx * dx + dz * dz).sqrt();
        // Trilateration with the reference frame along left to right pivots.
        let xt = (pivots * pivots + arm2[0] - arm2[1]) / (2.0 * pivots);
        let zt = (arm2[0] - xt * xt).sqrt();
        // Rotation and translation of the reference frame.
        let x = xt * dx / pivots + zt * dz / pivots - arm_x[0];
        let z = xt * dz / pivots - zt * dx / pivots + sp[0];
        [x, z]
    }

    /// The arm group's start Z: below every endstop sphere so the move starts
    /// on the correct side (`forcepos[2]`, `deltesian.py:99`).
    fn arm_force_z(&self) -> f64 {
        let dz2 = [
            self.arm2[0] - self.arm_x[0] * self.arm_x[0],
            self.arm2[1] - self.arm_x[1] * self.arm_x[1],
        ];
        -1.5 * dz2[0].max(dz2[1]).sqrt()
    }

    /// The two arm rails' home move (`deltesian.py:96-102`): start at `x = 0`
    /// below the bed, end at `x = 0`, `z = home_z`, both arms travelling
    /// together.
    ///
    /// The Y coordinate is untouched (upstream's `None` homepos entry); the
    /// driver fills it from the current position. Each arm's travel is the
    /// endstop's Z change, identical for both arms because the arms' vertical
    /// reach `sqrt(arm2 − arm_x²)` cancels.
    fn home_move(&self) -> DeltesianHome {
        let force_z = self.arm_force_z();
        let travel = self.home_z - force_z;
        DeltesianHome {
            arm_force_z: force_z,
            arm_target_z: self.home_z,
            arm_travel: [travel, travel],
        }
    }

    /// `_pillars_z_max`: the arms' highest Z at `x`.
    fn pillars_z_max(&self, x: f64) -> f64 {
        pillars_z_max(self.arm_x, self.arm2, self.abs_endstop, x)
    }
}

impl Kinematics for DeltesianKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        let left = stepper_positions.get(DELTESIAN_RAIL_NAMES[0]).copied();
        let right = stepper_positions.get(DELTESIAN_RAIL_NAMES[1]).copied();
        let (Some(left), Some(right)) = (left, right) else {
            // Without both arms the carriage cannot be located at all;
            // upstream would raise `KeyError`.
            return [None, None, None];
        };
        let [x, z] = self.actuator_to_cartesian([left, right]);
        // Y is the straight rail's own actuator position (`deltesian.py:45-47`).
        let y = stepper_positions.get(DELTESIAN_RAIL_NAMES[2]).copied();
        [Some(x), y, Some(z)]
    }

    fn check_move(&self, ctx: &mut MoveContext<'_>) -> Result<(), CommandError> {
        let mut limits = self.limits;
        let spos = *ctx.start_pos();
        let epos = *ctx.end_pos();
        let axes_d = *ctx.axes_d();
        // Recognize the arm home's final position (`deltesian.py:113-121`).
        let mut homing_move = false;
        if epos.x() == 0.0 && epos.z() == self.home_z && axes_d[Y_AXIS] == 0.0 {
            homing_move = true;
        } else if epos.z() > limits[Z_AXIS].1 {
            // Moves at the very top adapt the Z ceiling to the X position.
            limits[Z_AXIS].1 = self.pillars_z_max(epos.x());
        }
        for axis in [X_AXIS, Y_AXIS, Z_AXIS] {
            if axes_d[axis] == 0.0 {
                continue;
            }
            if !self.homed_axis[axis] {
                return Err(ctx.move_error("Must home axis first"));
            }
            if epos[axis] < limits[axis].0 || epos[axis] > limits[axis].1 {
                if !homing_move {
                    return Err(ctx.out_of_range());
                }
            }
        }
        if axes_d[Z_AXIS] != 0.0 {
            // A move with a Z component is slowed so Z keeps its own limit.
            let z_ratio = ctx.move_d() / axes_d[Z_AXIS].abs();
            ctx.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio);
        }
        // Slow at the extreme ends of X (`deltesian.py:130-135`).
        if axes_d[X_AXIS] != 0.0 {
            if let (Some(slow_x2), Some(very_slow_x2)) = (self.slow_x2, self.very_slow_x2) {
                let move_x2 = (spos.x() * spos.x()).max(epos.x() * epos.x());
                if move_x2 > very_slow_x2 {
                    ctx.limit_speed(self.max_velocity * 0.25, self.max_accel * 0.25);
                } else if move_x2 > slow_x2 {
                    ctx.limit_speed(self.max_velocity * 0.50, self.max_accel * 0.50);
                }
            }
        }
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, homing_axes: &[usize]) {
        // The steppers' positions are set centrally by the toolhead; only the
        // homed flags live here (`deltesian.py:58-62`).
        for axis in homing_axes {
            self.homed_axis[*axis] = true;
        }
    }

    fn update_limits(&mut self, _axis: usize, _range: Option<(f64, f64)>) {
        // Deltesian has no per-axis cartesian range hook upstream
        // (`kinematics/deltesian.py` defines no `update_limits`).
    }

    fn clear_homing_state(&mut self, axes: &[usize]) {
        // Per-axis, unlike the deltas (`deltesian.py:63-66`).
        for axis in axes {
            self.homed_axis[*axis] = false;
        }
    }

    fn get_status(&self) -> Value {
        let homed: String = ["x", "y", "z"]
            .iter()
            .enumerate()
            .filter(|(axis, _)| self.homed_axis[*axis])
            .map(|(_, name)| *name)
            .collect();
        json!({
            "homed_axes": homed,
            "axis_minimum": self.axes_min.as_array(),
            "axis_maximum": self.axes_max.as_array(),
        })
    }

    fn home(&mut self, homing: &mut dyn HomingState) {
        // The two arms home together, then Y (`deltesian.py:88-110`). The
        // upstream `set_axes` calls only feed `Homing.changed_axes`, which
        // nothing reads after `home`, so the shape is carried by the
        // `forcepos` entries the driver derives its homed axes from.
        let requested = homing.axes();
        let home_xz = requested.contains(&X_AXIS) || requested.contains(&Z_AXIS);
        let home_y = requested.contains(&Y_AXIS);
        let mut homepos: [Option<f64>; AXES] = [None; AXES];
        if home_xz {
            homepos[X_AXIS] = Some(0.0);
            homepos[Z_AXIS] = Some(self.home_z);
            let mut forcepos = homepos;
            forcepos[Z_AXIS] = Some(self.arm_force_z());
            homing.home_rails(&[0, 1], forcepos, homepos);
        }
        if home_y {
            let info = homing.homing_info(Y_AXIS);
            let (mut forcepos, mut movepos) =
                super::kinematics::home_move(Y_AXIS, &info, self.y_range.0, self.y_range.1);
            // The Y move keeps the arm-homed X/Z (`deltesian.py:102-107`).
            if home_xz {
                forcepos[X_AXIS] = Some(0.0);
                forcepos[Z_AXIS] = Some(self.home_z);
                movepos[X_AXIS] = Some(0.0);
                movepos[Z_AXIS] = Some(self.home_z);
            }
            homing.home_rails(&[2], forcepos, movepos);
        }
    }

    fn deltesian_home(&self) -> Option<DeltesianHome> {
        Some(self.home_move())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::Xyz;
    use crate::core::klippy::motion::kinematics::HomingInfo;

    /// `config/example-deltesian.cfg`'s machine: `arm_x_length` and
    /// `arm_length` on `stepper_left` (inherited by `stepper_right`), endstop
    /// 268, the straight Y rail 0..200.
    fn example_deltesian() -> DeltesianKinematics {
        DeltesianKinematics::new(DeltesianConfig {
            arm_x: [160.0, 160.0],
            arm2: [217.0 * 217.0, 217.0 * 217.0],
            arm_endstops: [268.0, 268.0],
            y_range: (0.0, 200.0),
            min_angle: MIN_ANGLE,
            print_width: None,
            minimum_z_position: 0.0,
            slow_ratio: SLOW_RATIO,
            max_velocity: 500.0,
            max_accel: 3000.0,
            max_z_velocity: 150.0,
            max_z_accel: 3000.0,
        })
    }

    /// A segment standing still at `position`.
    fn at(position: [f64; 3]) -> MoveSegment {
        MoveSegment {
            print_time: 0.0,
            move_t: 1000.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Xyz::new(position[0], position[1], position[2]),
            axes_r: Xyz::default(),
        }
    }

    fn limits() -> crate::core::klippy::motion::plan::MoveLimits {
        crate::core::klippy::motion::plan::MoveLimits {
            max_velocity: 500.0,
            max_accel: 3000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    #[test]
    fn test_the_solver_is_the_c_formula() {
        // Recompute the C body by hand: sqrt(arm2 − (x − arm_x)²) + z.
        let (arm2, signed) = (217.0f64 * 217.0, -160.0f64);
        let solver = deltesian_position_fn(arm2, signed);
        let pos = [10.0, -5.0, 40.0];
        let dx = pos[0] - signed;
        let expected = (arm2 - dx * dx).sqrt() + pos[2];
        assert!((solver.call(&at(pos), 0.5) - expected).abs() < 1e-12);
        assert_eq!(deltesian_active_flags(), AxisFlags::X.union(AxisFlags::Z));
    }

    #[test]
    fn test_the_arm_geometry_matches_the_upstream_allocation() {
        // `setup_itersolve('deltesian_stepper_alloc', arm2[0], -arm_x[0])` and
        // `(..., arm2[1], arm_x[1])`.
        let kin = example_deltesian();
        let [(left_arm2, left_x), (right_arm2, right_x)] = kin.arm_geometry();
        assert_eq!(left_arm2, 217.0 * 217.0);
        assert_eq!(right_arm2, 217.0 * 217.0);
        assert_eq!(left_x, -160.0);
        assert_eq!(right_x, 160.0);
    }

    #[test]
    fn test_the_inverse_is_the_upstream_hand_computed_home() {
        // `_actuator_to_cartesian` at the two endstop heights lands on the
        // origin, `z = 268` (hand-computed from `deltesian.py:45-56`).
        let kin = example_deltesian();
        let abs = arm_abs_endstops([268.0, 268.0], [160.0, 160.0], [217.0 * 217.0; 2]);
        assert!((abs[0] - 268.0 - 146.591_268).abs() < 1e-4, "{abs:?}");
        let [x, z] = kin.actuator_to_cartesian(abs);
        assert!(x.abs() < 1e-9, "{x}");
        assert!((z - 268.0).abs() < 1e-9, "{z}");
    }

    #[test]
    fn test_the_forward_and_inverse_solvers_round_trip() {
        // For a cartesian point, the two arm solver outputs fed back through
        // `actuator_to_cartesian` must return it.
        let kin = example_deltesian();
        let [(arm2_l, x_l), (arm2_r, x_r)] = kin.arm_geometry();
        let left = deltesian_position_fn(arm2_l, x_l);
        let right = deltesian_position_fn(arm2_r, x_r);
        for (x, z) in [(0.0, 100.0), (12.5, 40.0), (-30.0, 200.0), (5.0, 250.0)] {
            let pos = [x, 7.0, z];
            let back =
                kin.actuator_to_cartesian([left.call(&at(pos), 0.5), right.call(&at(pos), 0.5)]);
            assert!((back[0] - x).abs() < 1e-9, "x {back:?}");
            assert!((back[1] - z).abs() < 1e-9, "z {back:?}");
        }
    }

    #[test]
    fn test_calc_position_reads_the_stepper_names_it_knows() {
        let kin = example_deltesian();
        let [(arm2_l, x_l), (arm2_r, x_r)] = kin.arm_geometry();
        let left = deltesian_position_fn(arm2_l, x_l);
        let right = deltesian_position_fn(arm2_r, x_r);
        let target = [5.0, 13.0, 75.0];
        let steppers = HashMap::from([
            (
                DELTESIAN_RAIL_NAMES[0].to_string(),
                left.call(&at(target), 0.5),
            ),
            (
                DELTESIAN_RAIL_NAMES[1].to_string(),
                right.call(&at(target), 0.5),
            ),
            (DELTESIAN_RAIL_NAMES[2].to_string(), target[1]),
        ]);
        let got = kin.calc_position(&steppers);
        for axis in 0..3 {
            assert!((got[axis].unwrap() - target[axis]).abs() < 1e-9, "{got:?}");
        }
        // A missing arm means no position at all.
        let mut missing = steppers.clone();
        missing.remove(DELTESIAN_RAIL_NAMES[1]);
        assert_eq!(kin.calc_position(&missing), [None, None, None]);
    }

    #[test]
    fn test_get_status_reports_the_homed_axes() {
        let mut kin = example_deltesian();
        assert_eq!(kin.get_status()["homed_axes"], "");
        kin.set_position(Coord::default(), &[X_AXIS, Z_AXIS]);
        assert_eq!(kin.get_status()["homed_axes"], "xz");
        kin.set_position(Coord::default(), &[Y_AXIS]);
        assert_eq!(kin.get_status()["homed_axes"], "xyz");
        kin.clear_homing_state(&[Y_AXIS]);
        assert_eq!(kin.get_status()["homed_axes"], "xz");
        // The reported box is the derived one: X at the arms' min_angle reach,
        // Y the straight rail's, Z the `minimum_z_position` and the arms' top.
        assert_eq!(kin.get_status()["axis_minimum"][0], -57.0);
        assert_eq!(kin.get_status()["axis_maximum"][0], 56.0);
        assert_eq!(kin.get_status()["axis_minimum"][1], 0.0);
        assert_eq!(kin.get_status()["axis_maximum"][1], 200.0);
    }

    #[test]
    fn test_the_derived_envelope_is_the_upstream_geometry() {
        let kin = example_deltesian();
        // x_kin_min/max: ceil/floor of the min_angle reach (±56.17).
        assert_eq!(kin.limits[0], (-57.0, 56.0));
        assert_eq!(kin.limits[1], (0.0, 200.0));
        // z_max = min(pillars_z_max(-57), pillars_z_max(56)) = 223.5939.
        assert!(kin.limits[2].0 == 0.0);
        assert!(
            (kin.limits[2].1 - 223.593_886).abs() < 1e-3,
            "{}",
            kin.limits[2].1
        );
        // The home Z is the arms' endstop Z, above that ceiling.
        assert_eq!(kin.home_z, 268.0);
        assert!((kin.arm_force_z() - -219.886_903).abs() < 1e-4);
    }

    #[test]
    fn test_check_move_refuses_moves_before_homing_and_outside_the_envelope() {
        let mut kin = example_deltesian();
        // Before homing, any movement is refused.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(0.0, 0.0, 15.0, 0.0),
            50.0,
            &limits(),
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            let err = kin.check_move(&mut ctx).expect_err("must home first");
            assert!(err.to_string().contains("Must home axis first"), "{err}");
        }
        kin.set_position(Coord::new(0.0, 0.0, 268.0, 0.0), &[0, 1, 2]);
        // A move inside the envelope passes, and a Z move is slowed.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 268.0, 0.0),
            Coord::new(0.0, 0.0, 15.0, 0.0),
            500.0,
            &limits(),
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            kin.check_move(&mut ctx)
                .expect("a Z move inside the envelope");
        }
        assert!((move_.max_cruise_v2.sqrt() - 150.0 * (move_.move_d / 253.0)).abs() < 1e-6);
        // Past the X reach is refused, even homed.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 15.0, 0.0),
            Coord::new(250.0, 0.0, 15.0, 0.0),
            50.0,
            &limits(),
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            let err = kin.check_move(&mut ctx).expect_err("outside the X reach");
            assert!(err.to_string().contains("Move out of range"), "{err}");
        }
        // Past the Y rail's range is refused too.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 15.0, 0.0),
            Coord::new(0.0, 250.0, 15.0, 0.0),
            50.0,
            &limits(),
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            kin.check_move(&mut ctx).expect_err("outside the Y rail");
        }
    }

    #[test]
    fn test_check_move_accepts_the_arm_home_and_the_x_ceiling() {
        let mut kin = example_deltesian();
        // The arm home ends at (0, current_y, home_z), above the Z ceiling at
        // x=0 — recognized as the homing move (`deltesian.py:117-118`). The
        // force position the driver sets first marks X and Z homed.
        kin.set_position(Coord::new(0.0, 0.0, -219.886_903, 0.0), &[X_AXIS, Z_AXIS]);
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, -219.886_903, 0.0),
            Coord::new(0.0, 0.0, 268.0, 0.0),
            50.0,
            &limits(),
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            kin.check_move(&mut ctx).expect("the arm home is allowed");
        }
        // With the arms homed but Y not, a move at the top adapts the ceiling
        // to the X position; it is refused only because Y is not homed.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 260.0, 0.0),
            Coord::new(0.0, 5.0, 268.0, 0.0),
            50.0,
            &limits(),
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            let err = kin.check_move(&mut ctx).expect_err("Y is not homed");
            assert!(err.to_string().contains("Must home axis first"), "{err}");
        }
    }

    #[test]
    fn test_the_arm_home_group_ends_at_zero_home_z() {
        let kin = example_deltesian();
        let home = kin.home_move();
        assert_eq!(home.arm_target_z, 268.0);
        assert!(home.arm_force_z < 0.0);
        // Both arms travel the same Z distance on a symmetric machine.
        assert!((home.arm_travel[0] - home.arm_travel[1]).abs() < 1e-9);
        assert!((home.arm_travel[0] - (268.0 - home.arm_force_z)).abs() < 1e-9);
    }

    #[test]
    fn test_home_splits_into_an_arm_group_and_a_y_rail() {
        struct Recording {
            axes: Vec<usize>,
            calls: Vec<(Vec<usize>, [Option<f64>; AXES], [Option<f64>; AXES])>,
        }
        impl HomingState for Recording {
            fn axes(&self) -> Vec<usize> {
                self.axes.clone()
            }
            fn homing_info(&self, _axis: usize) -> HomingInfo {
                HomingInfo {
                    speed: 50.0,
                    position_endstop: 0.0,
                    retract_speed: 50.0,
                    retract_dist: 5.0,
                    positive_dir: false,
                    second_homing_speed: 25.0,
                }
            }
            fn home_rails(
                &mut self,
                rails: &[usize],
                forcepos: [Option<f64>; AXES],
                movepos: [Option<f64>; AXES],
            ) {
                self.calls.push((rails.to_vec(), forcepos, movepos));
            }
            fn get_trigger_position(&self, _stepper_name: &str) -> f64 {
                0.0
            }
            fn set_stepper_adjustment(&mut self, _stepper_name: &str, _adjustment: f64) {}
        }

        let mut kin = example_deltesian();
        let mut homing = Recording {
            axes: vec![X_AXIS, Y_AXIS, Z_AXIS],
            calls: Vec::new(),
        };
        kin.home(&mut homing);
        assert_eq!(homing.calls.len(), 2);
        // The arm group: rails 0 and 1, x=0, z=home_z, force below.
        let (rails, force, target) = &homing.calls[0];
        assert_eq!(rails, &[0, 1]);
        assert_eq!(force[X_AXIS], Some(0.0));
        assert_eq!(force[Y_AXIS], None);
        assert_eq!(force[Z_AXIS], Some(kin.arm_force_z()));
        assert_eq!(target[X_AXIS], Some(0.0));
        assert_eq!(target[Z_AXIS], Some(268.0));
        // The Y rail alone, keeping the arm-homed x/z, endstop below.
        let (rails, force, target) = &homing.calls[1];
        assert_eq!(rails, &[2]);
        assert_eq!(force[X_AXIS], Some(0.0));
        assert_eq!(force[Z_AXIS], Some(268.0));
        assert_eq!(target[X_AXIS], Some(0.0));
        assert_eq!(target[Z_AXIS], Some(268.0));
        assert_eq!(target[Y_AXIS], Some(0.0));
        assert!(force[Y_AXIS].unwrap() > 200.0);
    }

    #[test]
    fn test_zero_slow_ratio_disables_the_edge_slowdown() {
        let kin = DeltesianKinematics::new(DeltesianConfig {
            arm_x: [160.0, 160.0],
            arm2: [217.0 * 217.0; 2],
            arm_endstops: [268.0, 268.0],
            y_range: (0.0, 200.0),
            min_angle: MIN_ANGLE,
            print_width: None,
            minimum_z_position: 0.0,
            slow_ratio: 0.0,
            max_velocity: 500.0,
            max_accel: 3000.0,
            max_z_velocity: 150.0,
            max_z_accel: 3000.0,
        });
        assert!(kin.slow_x2.is_none() && kin.very_slow_x2.is_none());
    }
}
