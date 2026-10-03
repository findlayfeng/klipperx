//! `kinematics: rotary_delta` — the rotary-delta kinematics and its
//! calibration math.
//!
//! Upstream spreads this over three places, which is why this file carries all
//! three:
//!
//! | upstream | here |
//! |---|---|
//! | `rotary_delta_stepper_alloc` (`chelper/kin_rotary_delta.c:44-72`) | [`rotary_delta_position_fn`] / [`rotary_delta_active_flags`] |
//! | `RotaryDeltaKinematics` (`klippy/kinematics/rotary_delta.py:9-130`) | [`RotaryDeltaKinematics`] |
//! | `RotaryDeltaCalibration` (`rotary_delta.py:133-224`) | [`RotaryDeltaCalibration`] |
//!
//! **Placement.** Like [`super::delta`], this is its own file so the cartesian
//! family's code stays untouched and so `[delta_calibrate]`'s calibration half
//! (stable positions, coordinate-descent parameters) lives beside the geometry
//! that consumes it. The two share only the private [`trilateration`]
//! (`super::delta`), which upstream keeps in `mathutil`.
//!
//! # The geometry
//!
//! Each tower's upper arm rotates in a plane through the shoulder joint; the
//! solver returns that arm's angle so the lower arm reaches the carriage
//! (`rotary_two_arm_calc`, `kin_rotary_delta.c:16-31`). Going the other way,
//! the three lower-arm spheres centred on the elbows intersect at the carriage
//! position ([`trilateration`]). A "stable position" is steps taken since the
//! endstop hit — independent of the software parameters, which is what
//! `[delta_calibrate]` stores.

use std::collections::HashMap;
use std::sync::Mutex;

use serde_json::{json, Value};
use tracing::info;

use super::delta::trilateration;
use super::itersolve::{AxisFlags, PositionFn};
use super::kinematics::{HomingState, Kinematics, KinematicsCalibration, MoveContext, UnifiedHome};
use super::trapq::MoveSegment;
use crate::core::klippy::config::ConfigError;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, Z_AXIS};

/// The stepper names the three rails answer to (`'stepper_' + a` for
/// `a in 'abc'`, `rotary_delta.py:12`), in rail order.
pub const ROTARY_DELTA_RAIL_NAMES: [&str; 3] = ["stepper_a", "stepper_b", "stepper_c"];

/// Each tower's default mounting angle in degrees (`rotary_delta.py:38-39`,
/// `[30., 150., 270.]`).
pub const ROTARY_DELTA_DEFAULT_ANGLES: [f64; 3] = [30., 150., 270.];

// ===========================================================================
// rotary_delta_stepper_alloc
// ===========================================================================

/// `rotary_delta_stepper_alloc(shoulder_radius, shoulder_height, angle,
/// upper_arm, lower_arm)` (`chelper/kin_rotary_delta.c:44-72`): the tower's
/// arm angle for the carriage at the segment's position.
///
/// Parameters ride the bound [`PositionFn`]:
/// `[shoulder_radius, shoulder_height, cos, sin, upper_arm2, lower_arm2]`.
pub fn rotary_delta_position_fn(
    shoulder_radius: f64,
    shoulder_height: f64,
    angle: f64,
    upper_arm: f64,
    lower_arm: f64,
) -> PositionFn {
    PositionFn::bind(
        rotary_calc_position,
        [
            shoulder_radius,
            shoulder_height,
            angle.cos(),
            angle.sin(),
            upper_arm * upper_arm,
            lower_arm * lower_arm,
        ],
    )
}

/// The solver body (`rotary_stepper_calc_position`, `kin_rotary_delta.c:33-42`):
/// rotate/shift to the shoulder joint's frame, then the two-arm angle.
fn rotary_calc_position(segment: &MoveSegment, move_time: f64, params: &[f64; 6]) -> f64 {
    let c = segment.coord(move_time);
    let (cos, sin) = (params[2], params[3]);
    let shoulder_radius = params[0];
    let shoulder_height = params[1];
    // Rotate and shift axes to an origin at the shoulder joint with the upper
    // arm constrained to the xy plane, x aligned to the shoulder platform.
    let sjz = c.y() * cos - c.x() * sin;
    let sjx = c.x() * cos + c.y() * sin - shoulder_radius;
    let sjy = c.z() - shoulder_height;
    rotary_two_arm_calc(sjx, sjy, params[4], params[5] - sjz * sjz)
}

/// `rotary_two_arm_calc` (`kin_rotary_delta.c:16-31`): the upper-arm angle so
/// that elbow→effector is `sqrt(lower_arm2)`.
///
/// The formulas upstream starts from:
/// `elbow_x² + elbow_y² = upper_arm²` and
/// `(effector_x − elbow_x)² + (effector_y − elbow_y)² = lower_arm²`.
fn rotary_two_arm_calc(dx: f64, dy: f64, upper_arm2: f64, lower_arm2: f64) -> f64 {
    // Constants such that `elbow_y = c1 - c2*elbow_x`.
    let inv_dy = 1. / dy;
    let c1 = 0.5 * inv_dy * (dx * dx + dy * dy + upper_arm2 - lower_arm2);
    let c2 = dx * inv_dy;
    // Scaled elbow coordinates via the quadratic equation.
    let scale = c2 * c2 + 1.0;
    let scaled_elbow_x = c1 * c2 + (scale * upper_arm2 - c1 * c1).sqrt();
    let scaled_elbow_y = c1 * scale - c2 * scaled_elbow_x;
    scaled_elbow_y.atan2(scaled_elbow_x)
}

/// The rotary stepper moves on every axis (`AF_X | AF_Y | AF_Z`,
/// `kin_rotary_delta.c:70`).
pub fn rotary_delta_active_flags() -> AxisFlags {
    AxisFlags::X.union(AxisFlags::Y).union(AxisFlags::Z)
}

// ===========================================================================
// RotaryDeltaCalibration (rotary_delta.py:133-224)
// ===========================================================================

/// The rotary delta's calibration parameters (`RotaryDeltaCalibration`,
/// `rotary_delta.py:133-224`).
///
/// The mounting angles are held in **degrees** (the config's unit), while the
/// solver and [`Self::elbow_coord`] convert to radians, exactly as upstream
/// keeps `self.angles` in degrees and calls `math.radians` where needed.
#[derive(Debug, Clone)]
pub struct RotaryDeltaCalibration {
    /// `shoulder_radius`, millimetres.
    pub shoulder_radius: f64,
    /// `shoulder_height`, millimetres.
    pub shoulder_height: f64,
    /// Each tower's mounting angle, degrees.
    pub angles: [f64; 3],
    /// Each tower's upper-arm length, millimetres.
    pub upper_arms: [f64; 3],
    /// Each tower's lower-arm length, millimetres.
    pub lower_arms: [f64; 3],
    /// Each rail's `position_endstop` (arm angle in radians).
    pub endstops: [f64; 3],
    /// Each rail's radians per step (`rail.get_steppers()[0].get_step_dist`).
    pub stepdists: [f64; 3],
}

impl RotaryDeltaCalibration {
    /// The absolute angle of each endstop (`self.abs_endstops`,
    /// `rotary_delta.py:149-151`): the solver at `(0, 0, position_endstop)`.
    pub fn abs_endstops(&self) -> [f64; 3] {
        let mut out = [0.0; 3];
        for i in 0..3 {
            out[i] = self.solve_tower(i, [0.0, 0.0, self.endstops[i]]);
        }
        out
    }

    /// One tower's arm angle for a cartesian coordinate (the C solver, bound
    /// from this tower's parameters).
    fn solve_tower(&self, tower: usize, coord: [f64; 3]) -> f64 {
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1000.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: crate::core::klippy::mathutil::Xyz::new(coord[0], coord[1], coord[2]),
            axes_r: crate::core::klippy::mathutil::Xyz::default(),
        };
        rotary_delta_position_fn(
            self.shoulder_radius,
            self.shoulder_height,
            self.angles[tower].to_radians(),
            self.upper_arms[tower],
            self.lower_arms[tower],
        )
        .call(&segment, 500.0)
    }

    /// The elbow position in the main cartesian frame for one tower
    /// (`elbow_coord`, `rotary_delta.py:178-187`).
    pub fn elbow_coord(&self, elbow_id: usize, spos: f64) -> [f64; 3] {
        // Elbow position in the shoulder joint's coordinate system.
        let sj_elbow_x = self.upper_arms[elbow_id] * spos.cos();
        let sj_elbow_y = self.upper_arms[elbow_id] * spos.sin();
        // Shift and rotate to the main cartesian coordinate system.
        let angle = self.angles[elbow_id].to_radians();
        [
            (sj_elbow_x + self.shoulder_radius) * angle.cos(),
            (sj_elbow_x + self.shoulder_radius) * angle.sin(),
            sj_elbow_y + self.shoulder_height,
        ]
    }

    /// The carriage position for the three arm angles
    /// (`actuator_to_cartesian`, `rotary_delta.py:188-191`).
    ///
    /// `None` when the three spheres do not intersect (upstream's `ValueError`
    /// out of `trilateration`).
    pub fn actuator_to_cartesian(&self, spos: [f64; 3]) -> Option<[f64; 3]> {
        let sphere_coords = [
            self.elbow_coord(0, spos[0]),
            self.elbow_coord(1, spos[1]),
            self.elbow_coord(2, spos[2]),
        ];
        let lower_arm2 = [
            self.lower_arms[0] * self.lower_arms[0],
            self.lower_arms[1] * self.lower_arms[1],
            self.lower_arms[2] * self.lower_arms[2],
        ];
        trilateration(sphere_coords, lower_arm2)
    }

    /// The cartesian coordinate a stable position describes
    /// (`get_position_from_stable`, `rotary_delta.py:192-197`).
    pub fn get_position_from_stable(&self, stable_position: [f64; 3]) -> Option<[f64; 3]> {
        let abs_endstops = self.abs_endstops();
        let mut spos = [0.0; 3];
        for i in 0..3 {
            spos[i] = abs_endstops[i] - stable_position[i] * self.stepdists[i];
        }
        self.actuator_to_cartesian(spos)
    }

    /// The stable position (steps since each endstop hit) for a cartesian
    /// coordinate (`calc_stable_position`, `rotary_delta.py:198-204`).
    pub fn calc_stable_position(&self, coord: [f64; 3]) -> [f64; 3] {
        let abs_endstops = self.abs_endstops();
        let mut stable = [0.0; 3];
        for i in 0..3 {
            let spos = self.solve_tower(i, coord);
            stable[i] = (abs_endstops[i] - spos) / self.stepdists[i];
        }
        stable
    }

    /// The adjustable parameters in upstream's `adj_params` order
    /// (`coordinate_descent_params`, `rotary_delta.py:152-165`):
    /// `shoulder_height`, `endstop_a/b/c`, and — for an extended fit —
    /// `shoulder_radius`, `angle_a`, `angle_b`.
    ///
    /// Upstream takes `is_extended` as truthy; here the boolean is explicit.
    pub fn descent_params(&self, extended: bool) -> Vec<f64> {
        let mut params = vec![
            self.shoulder_height,
            self.endstops[0],
            self.endstops[1],
            self.endstops[2],
        ];
        if extended {
            params.extend([self.shoulder_radius, self.angles[0], self.angles[1]]);
        }
        params
    }

    /// Rebuild a calibration from [`Self::descent_params`] values
    /// (`new_calibration`, `rotary_delta.py:166-177`): everything the
    /// adjustable set does not cover carries over from `base`.
    pub fn from_descent_params(base: &Self, values: &[f64], extended: bool) -> Self {
        let mut out = base.clone();
        out.shoulder_height = values[0];
        out.endstops = [values[1], values[2], values[3]];
        if extended {
            out.shoulder_radius = values[4];
            out.angles[0] = values[5];
            out.angles[1] = values[6];
        }
        out
    }

    /// The config options [`Self::save_state`] writes
    /// (`RotaryDeltaCalibration.save_state`, `rotary_delta.py:205-224`):
    /// `shoulder_radius`/`shoulder_height` on `[printer]` and each tower's
    /// `angle`/`position_endstop`.
    ///
    /// Returned as `(section, option, value)` triples so the caller can write
    /// them; the write order matches upstream's.
    pub fn save_state_values(&self) -> Vec<(String, String, String)> {
        let mut out = vec![
            (
                "printer".to_string(),
                "shoulder_radius".to_string(),
                format!("{:.6}", self.shoulder_radius),
            ),
            (
                "printer".to_string(),
                "shoulder_height".to_string(),
                format!("{:.6}", self.shoulder_height),
            ),
        ];
        for (index, axis) in ['a', 'b', 'c'].into_iter().enumerate() {
            let section = format!("stepper_{axis}");
            out.push((
                section.clone(),
                "angle".to_string(),
                format!("{:.6}", self.angles[index]),
            ));
            out.push((
                section,
                "position_endstop".to_string(),
                format!("{:.6}", self.endstops[index]),
            ));
        }
        out
    }

    /// The report `save_state` prints (`rotary_delta.py:216-224`).
    pub fn save_state_report(&self) -> String {
        format!(
            "stepper_a: position_endstop: {:.6} angle: {:.6}\n\
             stepper_b: position_endstop: {:.6} angle: {:.6}\n\
             stepper_c: position_endstop: {:.6} angle: {:.6}\n\
             shoulder_radius: {:.6} shoulder_height: {:.6}",
            self.endstops[0],
            self.angles[0],
            self.endstops[1],
            self.angles[1],
            self.endstops[2],
            self.angles[2],
            self.shoulder_radius,
            self.shoulder_height,
        )
    }
}

// ===========================================================================
// RotaryDeltaConfig → RotaryDeltaKinematics
// ===========================================================================

/// What `[printer]` and its three `[stepper_a/b/c]` sections say about a
/// rotary-delta machine, as plain numbers the config readers already
/// bounds-checked (`kinematics/rotary_delta.py:10-76` reads the same options).
#[derive(Debug, Clone)]
pub struct RotaryDeltaConfig {
    /// `shoulder_radius`, above 0.
    pub shoulder_radius: f64,
    /// `shoulder_height`, above 0.
    pub shoulder_height: f64,
    /// `angle` per tower, default `[30., 150., 270.]`.
    pub angles: [f64; 3],
    /// `upper_arm_length` per tower: `stepper_a` required, the rest default to
    /// it; all above 0.
    pub upper_arms: [f64; 3],
    /// `lower_arm_length` per tower: as the upper arms.
    pub lower_arms: [f64; 3],
    /// Each rail's `position_endstop` (arm angle in radians).
    pub endstops: [f64; 3],
    /// Each rail's radians per step.
    pub step_dists: [f64; 3],
    /// `minimum_z_position`, default 0, at most the lowest endstop.
    pub minimum_z_position: f64,
    /// `[printer] max_velocity`.
    pub max_velocity: f64,
    /// `[printer] max_accel`.
    pub max_accel: f64,
    /// `max_z_velocity`, default `max_velocity`.
    pub max_z_velocity: f64,
    /// `max_z_accel`, default `max_accel`.
    pub max_z_accel: f64,
}

/// Rotary-delta kinematics: three shoulder joints whose arm angles place the
/// carriage, simultaneous homing, and a cylindrical envelope
/// (`RotaryDeltaKinematics`, `rotary_delta.py:9-130`).
#[derive(Debug)]
pub struct RotaryDeltaKinematics {
    /// The calibration parameters (also `[delta_calibrate]`'s view).
    cal: RotaryDeltaCalibration,
    /// The cartesian position homing drives to (`rotary_delta.py:58-61`).
    home_position: Coord,
    axes_min: Coord,
    axes_max: Coord,
    /// The lowest endstop (`rotary_delta.py:62`).
    max_z: f64,
    /// `minimum_z_position`.
    min_z: f64,
    /// Above this Z the cylinder tapers (`rotary_delta.py:69`).
    limit_z: f64,
    /// The squared radius the cylinder allows (`rotary_delta.py:66`).
    max_xy2: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// The squared radius a completed check cached (`rotary_delta.py:57`,
    /// `101-122`). Interior-mutable because `check_move` takes `&self`.
    limit_xy2: Mutex<f64>,
    /// Whether `G28` still has to run (`rotary_delta.py:56`).
    need_home: bool,
}

impl RotaryDeltaKinematics {
    /// Build the kinematics from the config's numbers
    /// (`RotaryDeltaKinematics.__init__`, `rotary_delta.py:10-76`), mirroring
    /// its bounds, derived envelope and log lines.
    ///
    /// # Errors
    /// When the three endstop angles have no common sphere intersection — the
    /// home position cannot be computed.
    pub fn new(config: RotaryDeltaConfig) -> Result<Self, ConfigError> {
        let cal = RotaryDeltaCalibration {
            shoulder_radius: config.shoulder_radius,
            shoulder_height: config.shoulder_height,
            angles: config.angles,
            upper_arms: config.upper_arms,
            lower_arms: config.lower_arms,
            endstops: config.endstops,
            stepdists: config.step_dists,
        };
        let eangles = cal.abs_endstops();

        // Where the machine believes home is: the endstop angles read back as
        // one cartesian point (`rotary_delta.py:58-61`).
        let home_xyz = cal.actuator_to_cartesian(eangles).ok_or_else(|| {
            ConfigError::new(
                "Unable to compute the rotary delta home position in section 'printer'".to_string(),
            )
        })?;
        let home_position = Coord::new(home_xyz[0], home_xyz[1], home_xyz[2], 0.0);

        let max_z = config
            .endstops
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let min_z = config.minimum_z_position;
        // The radius the arms allow (`rotary_delta.py:64-66`).
        let min_ua = config
            .upper_arms
            .iter()
            .map(|arm| config.shoulder_radius + arm)
            .fold(f64::INFINITY, f64::min);
        let min_la = config
            .lower_arms
            .iter()
            .map(|arm| arm - config.shoulder_radius)
            .fold(f64::INFINITY, f64::min);
        let max_xy2 = min_ua.min(min_la).powi(2);
        let max_xy = max_xy2.sqrt();
        // The height above which the cylinder tapers: the lowest elbow's
        // height minus its lower arm (`rotary_delta.py:67-69`).
        let limit_z = (0..3)
            .map(|i| cal.elbow_coord(i, eangles[i])[2] - config.lower_arms[i])
            .fold(f64::INFINITY, f64::min);

        info!(
            "Delta max build height {:.2}mm (radius tapered above {:.2}mm)",
            max_z, limit_z
        );

        Ok(Self {
            cal,
            home_position,
            axes_min: Coord::new(-max_xy, -max_xy, min_z, 0.0),
            axes_max: Coord::new(max_xy, max_xy, max_z, 0.0),
            max_z,
            min_z,
            limit_z,
            max_xy2,
            max_z_velocity: config.max_z_velocity,
            max_z_accel: config.max_z_accel,
            limit_xy2: Mutex::new(-1.),
            need_home: true,
        })
    }

    /// The parameters each rail's solver is bound with, in rail order
    /// (`setup_itersolve('rotary_delta_stepper_alloc', shoulder_radius,
    /// shoulder_height, math.radians(a), ua, la)`, `rotary_delta.py:49-52`):
    /// `(shoulder_radius, shoulder_height, angle_degrees, upper_arm,
    /// lower_arm)`.
    pub fn tower_geometry(&self) -> [(f64, f64, f64, f64, f64); 3] {
        let mut out = [(0.0, 0.0, 0.0, 0.0, 0.0); 3];
        for i in 0..3 {
            out[i] = (
                self.cal.shoulder_radius,
                self.cal.shoulder_height,
                self.cal.angles[i],
                self.cal.upper_arms[i],
                self.cal.lower_arms[i],
            );
        }
        out
    }

    /// The calibration view `[delta_calibrate]` takes
    /// (`get_calibration`, `rotary_delta.py:129-130`).
    pub fn calibration(&self) -> RotaryDeltaCalibration {
        self.cal.clone()
    }

    /// The one-piece homing move (`home`, `rotary_delta.py:93-100`): start at
    /// the home XY but below the bed (`forcepos[2] = -1`), end at the home
    /// position, all three towers travelling together.
    fn home_move(&self) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let target = [
            self.home_position.x(),
            self.home_position.y(),
            self.home_position.z(),
        ];
        // Upstream's `forcepos = list(self.home_position); forcepos[2] = -1.`
        let force = [target[0], target[1], -1.0];
        let mut travel = [0.0; 3];
        for i in 0..3 {
            let at = |pos: [f64; 3]| self.cal.solve_tower(i, pos);
            travel[i] = (at(target) - at(force)).abs();
        }
        (force, target, travel)
    }
}

impl Kinematics for RotaryDeltaKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        let mut spos = [0.0; 3];
        for (index, name) in ROTARY_DELTA_RAIL_NAMES.iter().enumerate() {
            match stepper_positions.get(*name) {
                Some(position) => spos[index] = *position,
                // One tower missing means the carriage cannot be located at
                // all; upstream would raise `KeyError`.
                None => return [None, None, None],
            }
        }
        match self.cal.actuator_to_cartesian(spos) {
            Some(point) => [Some(point[0]), Some(point[1]), Some(point[2])],
            None => [None, None, None],
        }
    }

    fn check_move(&self, ctx: &mut MoveContext<'_>) -> Result<(), CommandError> {
        let end = *ctx.end_pos();
        let axes_d = *ctx.axes_d();
        let end_xy2 = end.x() * end.x() + end.y() * end.y();
        if end_xy2 <= *self.limit_xy2.lock().unwrap_or_else(|p| p.into_inner())
            && axes_d[Z_AXIS] == 0.0
        {
            // A normal XY move inside the cached bound.
            return Ok(());
        }
        if self.need_home {
            return Err(ctx.move_error("Must home first"));
        }
        let end_z = end.z();
        let mut limit_xy2 = self.max_xy2;
        if end_z > self.limit_z {
            // Higher up, the arms reach less far out (`rotary_delta.py:111-112`).
            limit_xy2 = limit_xy2.min((self.max_z - end_z).powi(2));
        }
        if end_xy2 > limit_xy2 || end_z > self.max_z || end_z < self.min_z {
            // Out of range — unless this is the homing move finishing at the
            // home XY/Z (`rotary_delta.py:113-118`).
            if end.x() != self.home_position.x()
                || end.y() != self.home_position.y()
                || end_z < self.min_z
                || end_z > self.home_position.z()
            {
                return Err(ctx.out_of_range());
            }
            limit_xy2 = -1.;
        }
        if axes_d[Z_AXIS] != 0.0 {
            // A move with a Z component is slowed so Z keeps its own limit.
            let z_ratio = ctx.move_d() / axes_d[Z_AXIS].abs();
            ctx.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio);
            limit_xy2 = -1.;
        }
        *self.limit_xy2.lock().unwrap_or_else(|p| p.into_inner()) = limit_xy2;
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, homing_axes: &[usize]) {
        *self.limit_xy2.lock().unwrap_or_else(|p| p.into_inner()) = -1.;
        // Upstream keys off `homing_axes == "xyz"`; all three axes is the only
        // three-element set there is (`rotary_delta.py:82-87`).
        if homing_axes.len() == 3 {
            self.need_home = false;
        }
    }

    fn update_limits(&mut self, _axis: usize, _range: Option<(f64, f64)>) {
        // Rotary delta's rails carry no per-axis cartesian range; upstream has
        // no equivalent hook in `kinematics/rotary_delta.py`.
    }

    fn clear_homing_state(&mut self, clear_axes: &[usize]) {
        // Clearing homing state per axis is not implemented upstream either —
        // any axis drops the whole machine (`rotary_delta.py:88-92`).
        if !clear_axes.is_empty() {
            *self.limit_xy2.lock().unwrap_or_else(|p| p.into_inner()) = -1.;
            self.need_home = true;
        }
    }

    fn get_status(&self) -> Value {
        json!({
            "homed_axes": if self.need_home { "" } else { "xyz" },
            "axis_minimum": self.axes_min.as_array(),
            "axis_maximum": self.axes_max.as_array(),
        })
    }

    fn home(&mut self, homing: &mut dyn HomingState) {
        // All axes are homed simultaneously (`rotary_delta.py:93-100`).
        let (force, target, _) = self.home_move();
        let to_home_coord = |pos: [f64; 3]| -> [Option<f64>; 4] {
            [Some(pos[0]), Some(pos[1]), Some(pos[2]), None]
        };
        homing.home_rails(&[0, 1, 2], to_home_coord(force), to_home_coord(target));
    }

    fn unified_home(&self) -> Option<UnifiedHome> {
        let (force, target, actuator_travel) = self.home_move();
        Some(UnifiedHome {
            force,
            target,
            actuator_travel,
        })
    }

    fn delta_calibration(&self) -> Option<KinematicsCalibration> {
        Some(KinematicsCalibration::Rotary(self.cal.clone()))
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::Xyz;

    /// `test/klippy/rotary_delta_calibrate.cfg`'s machine: shoulder radius and
    /// height from the SAVE_CONFIG block, arms from `[stepper_a]`, the default
    /// tower angles, endstop 252 (radians-mode steps: 2π / (200·16·(107/16·60/16))).
    fn example_rotary() -> RotaryDeltaKinematics {
        let gear_ratio = (107.0 / 16.0) * (60.0 / 16.0);
        let step_dist = std::f64::consts::TAU / (200. * 16. * gear_ratio);
        RotaryDeltaKinematics::new(RotaryDeltaConfig {
            shoulder_radius: 33.9,
            shoulder_height: 412.9,
            angles: ROTARY_DELTA_DEFAULT_ANGLES,
            upper_arms: [170.0, 170.0, 170.0],
            lower_arms: [320.0, 320.0, 320.0],
            endstops: [252.0, 252.0, 252.0],
            step_dists: [step_dist, step_dist, step_dist],
            minimum_z_position: 0.0,
            max_velocity: 300.0,
            max_accel: 3000.0,
            max_z_velocity: 50.0,
            max_z_accel: 3000.0,
        })
        .expect("the example geometry computes")
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

    #[test]
    fn test_rotary_solver_matches_the_position_the_calibration_reads_back() {
        // The C solver's angle, fed back through `elbow_coord` +
        // trilateration, has to return the same cartesian point — the
        // invariant `[delta_calibrate]` depends on.
        let cal = example_rotary().calibration();
        for coord in [[0.0, 0.0, 100.0], [12.5, -7.25, 40.0], [-30.0, 30.0, 150.0]] {
            let angled = [
                cal.calc_stable_position(coord),
                cal.calc_stable_position(coord),
                cal.calc_stable_position(coord),
            ];
            // Rebuild absolute angles from the stable positions the solver
            // produced: spos = abs_endstop - stable * stepdist.
            let abs_endstops = cal.abs_endstops();
            let spos = [
                abs_endstops[0] - angled[0][0] * cal.stepdists[0],
                abs_endstops[1] - angled[1][1] * cal.stepdists[1],
                abs_endstops[2] - angled[2][2] * cal.stepdists[2],
            ];
            let back = cal
                .actuator_to_cartesian(spos)
                .expect("the spheres intersect");
            for axis in 0..3 {
                assert!(
                    (back[axis] - coord[axis]).abs() < 1e-6,
                    "round trip {coord:?}: {back:?}"
                );
            }
        }
    }

    #[test]
    fn test_the_solver_is_the_c_formula() {
        // Recompute the C body by hand: rotate/shift to the shoulder joint,
        // then the two-arm angle.
        let (sr, sh, angle, ua, la) = (33.9, 412.9, 30.0f64.to_radians(), 170.0, 320.0);
        let solver = rotary_delta_position_fn(sr, sh, angle, ua, la);
        let pos = [10.0, -5.0, 40.0];
        let c = pos;
        let sjz = c[1] * angle.cos() - c[0] * angle.sin();
        let sjx = c[0] * angle.cos() + c[1] * angle.sin() - sr;
        let sjy = c[2] - sh;
        let expected = rotary_two_arm_calc(sjx, sjy, ua * ua, la * la - sjz * sjz);
        assert!((solver.call(&at(pos), 0.5) - expected).abs() < 1e-12);
        assert_eq!(
            rotary_delta_active_flags(),
            AxisFlags::X.union(AxisFlags::Y).union(AxisFlags::Z)
        );
    }

    #[test]
    fn test_stable_positions_round_trip_through_trilateration() {
        let cal = example_rotary().calibration();
        for coord in [[0.0, 0.0, 100.0], [12.5, -7.25, 30.0], [-20.0, 20.0, 0.5]] {
            let stable = cal.calc_stable_position(coord);
            let back = cal
                .get_position_from_stable(stable)
                .expect("the spheres intersect");
            for axis in 0..3 {
                assert!(
                    (back[axis] - coord[axis]).abs() < 1e-6,
                    "round trip {coord:?}: {back:?}"
                );
            }
        }
    }

    #[test]
    fn test_abs_endstops_are_the_solver_at_the_endstop_height() {
        let cal = example_rotary().calibration();
        let abs = cal.abs_endstops();
        // The solver at (0, 0, position_endstop) for tower A.
        assert!((abs[0]).is_finite());
        // The home position is those angles read back as one point, centred.
        let home = cal
            .actuator_to_cartesian(abs)
            .expect("the endstop angles intersect");
        assert!(home[0].abs() < 1e-6, "{home:?}");
        assert!(home[1].abs() < 1e-6, "{home:?}");
    }

    #[test]
    fn test_unified_home_starts_below_the_bed_and_ends_at_home() {
        let mut kin = example_rotary();
        let home = kin.unified_home().expect("rotary delta homes as one move");
        // Upstream's `forcepos[2] = -1`.
        assert_eq!(home.force[2], -1.0);
        assert!((home.target[2] - 252.0).abs() < 1e-6, "{}", home.target[2]);
        // Every tower travels the same distance on a symmetric machine.
        assert!((home.actuator_travel[0] - home.actuator_travel[1]).abs() < 1e-9);
        assert!((home.actuator_travel[1] - home.actuator_travel[2]).abs() < 1e-9);
        // Homing leaves the machine homed.
        kin.set_position(
            Coord::new(home.target[0], home.target[1], home.target[2], 0.0),
            &[0, 1, 2],
        );
        assert_eq!(kin.get_status()["homed_axes"], "xyz");
    }

    #[test]
    fn test_check_move_refuses_moves_before_homing_and_outside_the_envelope() {
        let mut kin = example_rotary();
        let limits = crate::core::klippy::motion::plan::MoveLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        };
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(0.0, 0.0, 15.0, 0.0),
            50.0,
            &limits,
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            let err = kin.check_move(&mut ctx).expect_err("must home first");
            assert!(err.to_string().contains("Must home first"), "{err}");
        }
        kin.set_position(Coord::new(0.0, 0.0, 252.0, 0.0), &[0, 1, 2]);
        // A Z move inside the envelope passes.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 252.0, 0.0),
            Coord::new(0.0, 0.0, 15.0, 0.0),
            50.0,
            &limits,
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            kin.check_move(&mut ctx)
                .expect("a Z move inside the envelope");
        }
        // Too far out in XY is refused, even homed.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 15.0, 0.0),
            Coord::new(250.0, 0.0, 15.0, 0.0),
            50.0,
            &limits,
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            let err = kin.check_move(&mut ctx).expect_err("outside the envelope");
            assert!(err.to_string().contains("Move out of range"), "{err}");
        }
        // Above the lowest endstop is refused too.
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 15.0, 0.0),
            Coord::new(0.0, 0.0, 400.0, 0.0),
            50.0,
            &limits,
        );
        {
            let mut ctx = MoveContext::new(&mut move_);
            kin.check_move(&mut ctx).expect_err("above max_z");
        }
    }

    #[test]
    fn test_calc_position_reads_the_stepper_names_it_knows() {
        let kin = example_rotary();
        let cal = kin.calibration();
        let target = [5.0, 13.0, 75.0];
        let abs_endstops = cal.abs_endstops();
        let stable = cal.calc_stable_position(target);
        let mut steppers = HashMap::new();
        for (index, name) in ROTARY_DELTA_RAIL_NAMES.iter().enumerate() {
            let actuator = abs_endstops[index] - stable[index] * cal.stepdists[index];
            steppers.insert(name.to_string(), actuator);
        }
        let got = kin.calc_position(&steppers);
        for axis in 0..3 {
            assert!((got[axis].unwrap() - target[axis]).abs() < 1e-6, "{got:?}");
        }
        // A missing tower means no position at all.
        steppers.remove("stepper_b");
        assert_eq!(kin.calc_position(&steppers), [None, None, None]);
    }

    #[test]
    fn test_descent_params_follow_the_upstream_adj_order() {
        let cal = example_rotary().calibration();
        let basic = cal.descent_params(false);
        assert_eq!(basic.len(), 4);
        assert_eq!(basic[0], cal.shoulder_height);
        assert_eq!(&basic[1..], &cal.endstops);
        let extended = cal.descent_params(true);
        assert_eq!(extended.len(), 7);
        assert_eq!(extended[4], cal.shoulder_radius);
        assert_eq!(extended[5], cal.angles[0]);
        assert_eq!(extended[6], cal.angles[1]);
        // Rebuilding with changed values keeps what the set does not adjust
        // (`new_calibration`): the C angle and the arm lengths.
        let mut values = extended.clone();
        values[0] = 413.5;
        values[4] = 34.0;
        let rebuilt = RotaryDeltaCalibration::from_descent_params(&cal, &values, true);
        assert_eq!(rebuilt.shoulder_height, 413.5);
        assert_eq!(rebuilt.shoulder_radius, 34.0);
        assert_eq!(rebuilt.angles[2], cal.angles[2]);
        assert_eq!(rebuilt.upper_arms, cal.upper_arms);
    }

    #[test]
    fn test_tower_geometry_feeds_the_solver_in_the_upstream_order() {
        let kin = example_rotary();
        let [(sr, sh, angle, ua, la), ..] = kin.tower_geometry();
        assert_eq!(sr, 33.9);
        assert_eq!(sh, 412.9);
        assert_eq!(angle, 30.0);
        assert_eq!(ua, 170.0);
        assert_eq!(la, 320.0);
    }

    #[test]
    fn test_save_state_report_matches_the_upstream_wording() {
        let cal = example_rotary().calibration();
        let expected = format!(
            "stepper_a: position_endstop: {:.6} angle: {:.6}\n\
             stepper_b: position_endstop: {:.6} angle: {:.6}\n\
             stepper_c: position_endstop: {:.6} angle: {:.6}\n\
             shoulder_radius: {:.6} shoulder_height: {:.6}",
            252.0, 30.0, 252.0, 150.0, 252.0, 270.0, 33.9, 412.9,
        );
        assert_eq!(cal.save_state_report(), expected);
        // The writes mirror upstream's order: `[printer]` first, then a/b/c.
        let writes = cal.save_state_values();
        assert_eq!(writes[0].0, "printer");
        assert_eq!(writes[0].1, "shoulder_radius");
        assert_eq!(writes[0].2, "33.900000");
        assert_eq!(writes[1].1, "shoulder_height");
        assert_eq!(writes[2].0, "stepper_a");
        assert_eq!(writes[2].1, "angle");
        assert_eq!(writes[3].1, "position_endstop");
    }
}
