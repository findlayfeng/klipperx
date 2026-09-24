//! `kinematics: delta` — the linear-delta kinematics and its calibration math.
//!
//! Upstream spreads this over three places, which is why this file carries all
//! three:
//!
//! | upstream | here |
//! |---|---|
//! | `delta_stepper_alloc` (`chelper/kin_delta.c:25-41`) | [`delta_position_fn`] / [`delta_active_flags`] |
//! | `DeltaKinematics` (`klippy/kinematics/delta.py:11-160`) | [`DeltaKinematics`] |
//! | `DeltaCalibration` (`delta.py:163-241`) + `mathutil.trilateration` | [`DeltaCalibration`] and the private [`trilateration`] |
//!
//! **Placement.** The cartesian family lives in [`super::kinematics`]; delta is
//! a separate file so that family's code stays untouched (the task's
//! "禁动既有运动学语义"), and because the calibration half (`DeltaCalibration`,
//! stable positions, coordinate-descent parameters) is consumed by
//! `[delta_calibrate]` and would otherwise double that module's size. The
//! glue that *does* touch shared code is kept minimal: the [`Kinematics`]
//! impl, two default methods on the trait, and the parameterised
//! [`PositionFn`](super::itersolve::PositionFn) hook upstream's
//! `setup_itersolve('delta_stepper_alloc', …)` (`delta.py:52`) needs.
//!
//! # The geometry
//!
//! Each tower's stepper position is the actuator distance `sqrt(arm² - dx² -
//! dy²) + z` for the carriage at `(x, y, z)` (the C solver's
//! `delta_stepper_calc_position`); going the other way, three spheres centred
//! on the towers intersect at the carriage position (`mathutil.trilateration`).
//! A "stable position" is steps taken since the endstop hit — a coordinate
//! independent of the software parameters, which is what `[delta_calibrate]`
//! stores (`delta_calibrate.py:7-11`).

use std::collections::HashMap;
use std::sync::Mutex;

use serde_json::{json, Value};
use tracing::info;

use super::itersolve::{AxisFlags, PositionFn};
use super::kinematics::{HomingState, Kinematics, MoveContext, UnifiedHome};
use super::trapq::MoveSegment;
use crate::core::klippy::config::ConfigError;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, Z_AXIS};

/// Slow moves once the ratio of tower to XY movement exceeds this
/// (`SLOW_RATIO`, `delta.py:9`).
const SLOW_RATIO: f64 = 3.;

/// The stepper names the three rails answer to (`'stepper_' + a` for
/// `a in 'abc'`, `delta.py:15`), in rail order.
pub const DELTA_RAIL_NAMES: [&str; 3] = ["stepper_a", "stepper_b", "stepper_c"];

// ===========================================================================
// delta_stepper_alloc
// ===========================================================================

/// `delta_stepper_alloc(arm2, tower_x, tower_y)` (`chelper/kin_delta.c:25-41`):
/// where the tower's actuator is for the carriage at the segment's position.
///
/// Parameters ride the bound [`PositionFn`]: `[arm2, tower_x, tower_y]`.
pub fn delta_position_fn(arm2: f64, tower_x: f64, tower_y: f64) -> PositionFn {
    PositionFn::bind(delta_calc_position, [arm2, tower_x, tower_y])
}

/// The solver body: `sqrt(arm2 - dx² - dy²) + z` (`kin_delta.c:17-24`).
///
/// A position outside the arm's reach takes the square root of a negative
/// number and yields `NaN`, which the step search treats as "no solution";
/// upstream's `check_move` keeps moves inside the envelope so the two agree.
fn delta_calc_position(segment: &MoveSegment, move_time: f64, params: &[f64; 3]) -> f64 {
    let coord = segment.coord(move_time);
    let dx = params[1] - coord.x();
    let dy = params[2] - coord.y();
    (params[0] - dx * dx - dy * dy).sqrt() + coord.z()
}

/// The delta stepper moves on every axis (`AF_X | AF_Y | AF_Z`,
/// `kin_delta.c:37`).
pub fn delta_active_flags() -> AxisFlags {
    AxisFlags::X.union(AxisFlags::Y).union(AxisFlags::Z)
}

// ===========================================================================
// Trilateration (mathutil.py:93-113)
// ===========================================================================

/// The intersection of three spheres — the branch below the sphere centres
/// (upstream's `z = -sqrt(radius2[0] - x² - y²)`).
///
/// `None` when the geometry has no real intersection (a square root of a
/// negative number, where upstream raises `ValueError`); every caller maps
/// that to its own "cannot compute" answer.
fn trilateration(sphere_coords: [[f64; 3]; 3], radius2: [f64; 3]) -> Option<[f64; 3]> {
    let [c1, c2, c3] = sphere_coords;
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let mul = |a: [f64; 3], k: f64| [a[0] * k, a[1] * k, a[2] * k];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let magsq = |a: [f64; 3]| dot(a, a);
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };

    let s21 = sub(c2, c1);
    let s31 = sub(c3, c1);
    let d = magsq(s21).sqrt();
    let ex = mul(s21, 1. / d);
    let i = dot(ex, s31);
    let ey = mul(
        sub(s31, mul(ex, i)),
        1. / magsq(sub(s31, mul(ex, i))).sqrt(),
    );
    let ez = cross(ex, ey);
    let j = dot(ey, s31);

    let x = (radius2[0] - radius2[1] + d * d) / (2. * d);
    let y = (radius2[0] - radius2[2] - x * x + (x - i) * (x - i) + j * j) / (2. * j);
    let z = -(radius2[0] - x * x - y * y).sqrt();

    let point = [
        c1[0] + ex[0] * x + ey[0] * y + ez[0] * z,
        c1[1] + ex[1] * x + ey[1] * y + ez[1] * z,
        c1[2] + ex[2] * x + ey[2] * y + ez[2] * z,
    ];
    point.iter().all(|v| v.is_finite()).then_some(point)
}

// ===========================================================================
// DeltaCalibration (delta.py:163-241)
// ===========================================================================

/// The five parameters `DELTA_CALIBRATE` fits: radius, tower angles, arm
/// lengths, endstop heights, and the steppers' step distances
/// (`DeltaCalibration`, `delta.py:163-241`).
#[derive(Debug, Clone)]
pub struct DeltaCalibration {
    /// The delta radius, millimetres (`delta_radius`).
    pub radius: f64,
    /// The tower angles in degrees, `[210., 330., 90.]` by default.
    pub angles: [f64; 3],
    /// The arm length per tower, millimetres.
    pub arms: [f64; 3],
    /// Where each tower's endstop sits, in actuator coordinates.
    pub endstops: [f64; 3],
    /// Millimetres per step per tower (`rail.get_steppers()[0].get_step_dist`).
    pub stepdists: [f64; 3],
}

impl DeltaCalibration {
    /// The towers' XY positions (`delta.py:44-46`).
    pub fn towers(&self) -> [(f64, f64); 3] {
        let mut towers = [(0.0, 0.0); 3];
        for (i, angle) in self.angles.iter().enumerate() {
            let radians = angle.to_radians();
            towers[i] = (radians.cos() * self.radius, radians.sin() * self.radius);
        }
        towers
    }

    /// The absolute Z height of each tower's endstop (`delta.py:47-49`).
    pub fn abs_endstops(&self) -> [f64; 3] {
        let radius2 = self.radius * self.radius;
        let mut out = [0.0; 3];
        for i in 0..3 {
            out[i] = self.endstops[i] + (self.arms[i] * self.arms[i] - radius2).sqrt();
        }
        out
    }

    /// The carriage position for actuator (stable-position *plus* endstop)
    /// coordinates — `_actuator_to_cartesian` (`delta.py:81-84`).
    pub fn actuator_to_cartesian(&self, actuator: [f64; 3]) -> Option<[f64; 3]> {
        let towers = self.towers();
        let sphere_coords = [
            [towers[0].0, towers[0].1, actuator[0]],
            [towers[1].0, towers[1].1, actuator[1]],
            [towers[2].0, towers[2].1, actuator[2]],
        ];
        trilateration(sphere_coords, self.arm2())
    }

    /// The three squared arm lengths.
    pub fn arm2(&self) -> [f64; 3] {
        [
            self.arms[0] * self.arms[0],
            self.arms[1] * self.arms[1],
            self.arms[2] * self.arms[2],
        ]
    }

    /// The stable position (steps since each endstop hit) for a cartesian
    /// coordinate (`calc_stable_position`, `delta.py:213-222`).
    pub fn calc_stable_position(&self, coord: [f64; 3]) -> [f64; 3] {
        let towers = self.towers();
        let abs_endstops = self.abs_endstops();
        let arm2 = self.arm2();
        let mut stable = [0.0; 3];
        for i in 0..3 {
            let dx = towers[i].0 - coord[0];
            let dy = towers[i].1 - coord[1];
            let steppos = (arm2[i] - dx * dx - dy * dy).sqrt() + coord[2];
            stable[i] = (abs_endstops[i] - steppos) / self.stepdists[i];
        }
        stable
    }

    /// The cartesian coordinate a stable position describes
    /// (`get_position_from_stable`, `delta.py:202-211`).
    ///
    /// `None` when the three spheres do not intersect (upstream's
    /// `ValueError` out of `trilateration`).
    pub fn get_position_from_stable(&self, stable: [f64; 3]) -> Option<[f64; 3]> {
        let towers = self.towers();
        let abs_endstops = self.abs_endstops();
        let sphere_coords = [
            [
                towers[0].0,
                towers[0].1,
                abs_endstops[0] - stable[0] * self.stepdists[0],
            ],
            [
                towers[1].0,
                towers[1].1,
                abs_endstops[1] - stable[1] * self.stepdists[1],
            ],
            [
                towers[2].0,
                towers[2].1,
                abs_endstops[2] - stable[2] * self.stepdists[2],
            ],
        ];
        trilateration(sphere_coords, self.arm2())
    }

    /// The adjustable parameters in upstream's `adj_params` order
    /// (`coordinate_descent_params`, `delta.py:187-199`): `radius`,
    /// `angle_a`, `angle_b`, `endstop_a/b/c`, and — for an extended
    /// (distance-measured) fit — `arm_a/b/c`.
    ///
    /// Upstream takes `is_extended` as truthy `distances` (its callers pass the
    /// list itself); here the boolean is explicit.
    pub fn descent_params(&self, extended: bool) -> Vec<f64> {
        let mut params = vec![
            self.radius,
            self.angles[0],
            self.angles[1],
            self.endstops[0],
            self.endstops[1],
            self.endstops[2],
        ];
        if extended {
            params.extend([self.arms[0], self.arms[1], self.arms[2]]);
        }
        params
    }

    /// Rebuild a calibration from [`Self::descent_params`] values
    /// (`new_calibration`, `delta.py:200-211`): everything the adjustable set
    /// does not cover carries over from `base`.
    pub fn from_descent_params(base: &Self, values: &[f64], extended: bool) -> Self {
        let mut out = base.clone();
        out.radius = values[0];
        out.angles[0] = values[1];
        out.angles[1] = values[2];
        out.endstops[0] = values[3];
        out.endstops[1] = values[4];
        out.endstops[2] = values[5];
        if extended {
            out.arms = [values[6], values[7], values[8]];
        }
        out
    }
}

// ===========================================================================
// DeltaConfig → DeltaKinematics
// ===========================================================================

/// What `[printer]` and its three `[stepper_a/b/c]` sections say about a
/// delta machine, as plain numbers the config readers already bounds-checked
/// (`kinematics/delta.py:11-77` reads the same options).
#[derive(Debug, Clone)]
pub struct DeltaConfig {
    /// `delta_radius`, above 0.
    pub radius: f64,
    /// `print_radius`, default `radius`.
    pub print_radius: f64,
    /// `minimum_z_position`, default 0, at most the lowest endstop.
    pub minimum_z_position: f64,
    /// `angle` per tower, default `[210., 330., 90.]`.
    pub angles: [f64; 3],
    /// `arm_length` per tower: `stepper_a` required, the rest default to it;
    /// all above `radius`.
    pub arm_lengths: [f64; 3],
    /// Each rail's `position_endstop` (`stepper_b/c` default to `stepper_a`).
    pub endstops: [f64; 3],
    /// Each rail's millimetres per step.
    pub step_dists: [f64; 3],
    /// `[printer] max_velocity`.
    pub max_velocity: f64,
    /// `[printer] max_accel`.
    pub max_accel: f64,
    /// `max_z_velocity`, default `max_velocity`.
    pub max_z_velocity: f64,
    /// `max_z_accel`, default `max_accel`.
    pub max_z_accel: f64,
}

/// Linear-delta kinematics: three towers, simultaneous homing, an envelope
/// that tapers with height (`DeltaKinematics`, `delta.py:11-160`).
#[derive(Debug)]
pub struct DeltaKinematics {
    /// The calibration parameters (also `[delta_calibrate]`'s view).
    cal: DeltaCalibration,
    /// The squared arm lengths (the `arm2` the solver and bounds read).
    arm2: [f64; 3],
    /// The towers' XY positions.
    towers: [(f64, f64); 3],
    /// The cartesian position homing drives to (`delta.py:78-80`).
    home_position: Coord,
    axes_min: Coord,
    axes_max: Coord,
    /// The lowest endstop: the build height (`delta.py:75`).
    max_z: f64,
    /// `minimum_z_position`.
    min_z: f64,
    /// Above this Z the envelope tapers with radius (`delta.py:76`).
    limit_z: f64,
    /// The shortest arm and its square, for the tapered bound.
    min_arm_length: f64,
    min_arm2: f64,
    /// `delta_radius`.
    radius: f64,
    /// Squared radii where moves slow, slow further, and stop being legal.
    max_xy2: f64,
    slow_xy2: f64,
    very_slow_xy2: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// `[printer] max_velocity` / `max_accel`: the envelope-slowdown step
    /// (`delta.py:150-157`).
    max_velocity: f64,
    max_accel: f64,
    /// The XY bound a completed check cached (`delta.py:66/120-159`).
    /// Interior-mutable because `check_move` takes `&self`.
    limit_xy2: Mutex<f64>,
    /// Whether `G28` still has to run (`delta.py:77`).
    need_home: bool,
}

impl DeltaKinematics {
    /// Build the kinematics from the config's numbers (`DeltaKinematics.__init__`,
    /// `delta.py:11-160`), mirroring its bounds, derived envelope and log lines.
    ///
    /// # Errors
    /// When the three endstops have no common sphere intersection — the home
    /// position cannot be computed (unreachable for a config whose arms are
    /// above its radius, which the config readers enforce).
    pub fn new(config: DeltaConfig) -> Result<Self, ConfigError> {
        let cal = DeltaCalibration {
            radius: config.radius,
            angles: config.angles,
            arms: config.arm_lengths,
            endstops: config.endstops,
            stepdists: config.step_dists,
        };
        let towers = cal.towers();
        let arm2 = cal.arm2();
        let abs_endstops = cal.abs_endstops();

        // Where the machine believes home is: the actuator positions at the
        // endstops read back as one cartesian point (`delta.py:78-80`).
        let home_xyz = cal.actuator_to_cartesian(abs_endstops).ok_or_else(|| {
            ConfigError::new(format!(
                "Unable to compute the delta home position in section 'printer'"
            ))
        })?;
        let home_position = Coord::new(home_xyz[0], home_xyz[1], home_xyz[2], 0.0);

        let max_z = config
            .endstops
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let min_z = config.minimum_z_position;
        let limit_z = abs_endstops
            .iter()
            .zip(config.arm_lengths.iter())
            .map(|(endstop, arm)| endstop - arm)
            .fold(f64::INFINITY, f64::min);

        // The point where an XY move could move a tower too fast
        // (`delta.py:58-73`).
        let half_min_step_dist = config
            .step_dists
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
            * 0.5;
        let min_arm_length = config
            .arm_lengths
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let min_arm2 = min_arm_length * min_arm_length;
        let radius = config.radius;
        let ratio_to_xy = |ratio: f64| {
            ratio
                * (min_arm2 / (ratio * ratio + 1.) - half_min_step_dist * half_min_step_dist).sqrt()
                + half_min_step_dist
                - radius
        };
        let slow_xy2 = ratio_to_xy(SLOW_RATIO).powi(2);
        let very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO).powi(2);
        let max_xy2 = config
            .print_radius
            .min(min_arm_length - radius)
            .min(ratio_to_xy(4. * SLOW_RATIO))
            .powi(2);
        let max_xy = max_xy2.sqrt();
        let axes_min = Coord::new(-max_xy, -max_xy, min_z, 0.0);
        let axes_max = Coord::new(max_xy, max_xy, max_z, 0.0);

        info!(
            "Delta max build height {:.2}mm (radius tapered above {:.2}mm)",
            max_z, limit_z
        );
        info!(
            "Delta max build radius {:.2}mm (moves slowed past {:.2}mm and {:.2}mm)",
            max_xy,
            slow_xy2.sqrt(),
            very_slow_xy2.sqrt()
        );

        Ok(Self {
            cal,
            arm2,
            towers,
            home_position,
            axes_min,
            axes_max,
            max_z,
            min_z,
            limit_z,
            min_arm_length,
            min_arm2,
            radius,
            max_xy2,
            slow_xy2,
            very_slow_xy2,
            max_z_velocity: config.max_z_velocity,
            max_z_accel: config.max_z_accel,
            max_velocity: config.max_velocity,
            max_accel: config.max_accel,
            limit_xy2: Mutex::new(-1.),
            need_home: true,
        })
    }

    /// The parameters each rail's solver is bound with, in rail order
    /// (`setup_itersolve('delta_stepper_alloc', arm2, tower_x, tower_y)`,
    /// `delta.py:50-52`).
    pub fn tower_geometry(&self) -> [(f64, f64, f64); 3] {
        let mut out = [(0.0, 0.0, 0.0); 3];
        for i in 0..3 {
            out[i] = (self.arm2[i], self.towers[i].0, self.towers[i].1);
        }
        out
    }

    /// The calibration view `[delta_calibrate]` takes
    /// (`get_calibration`, `delta.py:153-160`).
    pub fn calibration(&self) -> DeltaCalibration {
        self.cal.clone()
    }

    /// The carriage position for the three actuator positions.
    fn actuator_to_cartesian(&self, actuator: [f64; 3]) -> Option<[f64; 3]> {
        let sphere_coords = [
            [self.towers[0].0, self.towers[0].1, actuator[0]],
            [self.towers[1].0, self.towers[1].1, actuator[1]],
            [self.towers[2].0, self.towers[2].1, actuator[2]],
        ];
        trilateration(sphere_coords, self.arm2)
    }

    /// The one-piece homing move (`home`, `delta.py:104-110`): start below
    /// every sphere, end at the home position, all three towers travelling
    /// together.
    fn home_move(&self) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let target = [
            self.home_position.x(),
            self.home_position.y(),
            self.home_position.z(),
        ];
        let force_z = -1.5 * (self.arm2.iter().copied().fold(0.0, f64::max) - self.max_xy2).sqrt();
        let force = [target[0], target[1], force_z];
        let mut travel = [0.0; 3];
        for i in 0..3 {
            let at = |pos: [f64; 3]| {
                let dx = self.towers[i].0 - pos[0];
                let dy = self.towers[i].1 - pos[1];
                (self.arm2[i] - dx * dx - dy * dy).sqrt() + pos[2]
            };
            travel[i] = (at(target) - at(force)).abs();
        }
        (force, target, travel)
    }
}

impl Kinematics for DeltaKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        let mut actuator = [0.0; 3];
        for (index, name) in DELTA_RAIL_NAMES.iter().enumerate() {
            match stepper_positions.get(*name) {
                Some(position) => actuator[index] = *position,
                // One tower missing means the carriage cannot be located at
                // all; upstream would raise `KeyError`.
                None => return [None, None, None],
            }
        }
        match self.actuator_to_cartesian(actuator) {
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
            // Higher up, the arm reaches less far out (`delta.py:129-135`).
            let above_z_limit = end_z - self.limit_z;
            let allowed_radius = self.radius
                - (self.min_arm2 - (self.min_arm_length - above_z_limit).powi(2)).sqrt();
            limit_xy2 = limit_xy2.min(allowed_radius * allowed_radius);
        }
        if end_xy2 > limit_xy2 || end_z > self.max_z || end_z < self.min_z {
            // Out of range — unless this is the homing move finishing at the
            // home XY (`delta.py:137-146`).
            let start = *ctx.start_pos();
            if start.x() != self.home_position.x()
                || start.y() != self.home_position.y()
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
        // Slow down at the extreme edge of the envelope (`delta.py:150-157`).
        let start = *ctx.start_pos();
        let extreme_xy2 = end_xy2.max(start.x() * start.x() + start.y() * start.y());
        if extreme_xy2 > self.slow_xy2 {
            let ratio = if extreme_xy2 > self.very_slow_xy2 {
                0.25
            } else {
                0.5
            };
            ctx.limit_speed(self.max_velocity * ratio, self.max_accel * ratio);
            limit_xy2 = -1.;
        }
        *self.limit_xy2.lock().unwrap_or_else(|p| p.into_inner()) = limit_xy2.min(self.slow_xy2);
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, homing_axes: &[usize]) {
        *self.limit_xy2.lock().unwrap_or_else(|p| p.into_inner()) = -1.;
        // Upstream keys off `homing_axes == "xyz"`; all three axes is the only
        // three-element set there is (`delta.py:96-102`).
        if homing_axes.len() == 3 {
            self.need_home = false;
        }
    }

    fn update_limits(&mut self, _axis: usize, _range: Option<(f64, f64)>) {
        // Delta rails carry no per-axis cartesian range; upstream has no
        // equivalent hook in `kinematics/delta.py`.
    }

    fn clear_homing_state(&mut self, clear_axes: &[usize]) {
        // Clearing homing state is not implemented per axis upstream either —
        // it drops the whole machine (`delta.py:103-107`).
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
            "cone_start_z": self.limit_z,
        })
    }

    fn home(&mut self, homing: &mut dyn HomingState) {
        // All axes are homed simultaneously (`delta.py:104-110`).
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

    fn delta_calibration(&self) -> Option<DeltaCalibration> {
        Some(self.cal.clone())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// `config/example-delta.cfg`'s geometry: radius 174.75, arms 333,
    /// endstops 297.05, `rotation_distance: 40` at 16 microsteps → 0.0125 mm
    /// per step.
    fn example_delta() -> DeltaKinematics {
        DeltaKinematics::new(DeltaConfig {
            radius: 174.75,
            print_radius: 174.75,
            minimum_z_position: 0.0,
            angles: [210., 330., 90.],
            arm_lengths: [333., 333., 333.],
            endstops: [297.05, 297.05, 297.05],
            step_dists: [0.0125, 0.0125, 0.0125],
            max_velocity: 300.,
            max_accel: 3000.,
            max_z_velocity: 150.,
            max_z_accel: 1500.,
        })
        .expect("the example-delta geometry computes")
    }

    /// A segment standing still at `position`, for calling the position fns.
    fn at(position: [f64; 3]) -> MoveSegment {
        MoveSegment {
            print_time: 0.0,
            move_t: 1000.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: crate::core::klippy::mathutil::Xyz::new(
                position[0],
                position[1],
                position[2],
            ),
            axes_r: crate::core::klippy::mathutil::Xyz::default(),
        }
    }

    #[test]
    fn test_delta_stepper_position_matches_the_tower_formula() {
        // The C solver: sqrt(arm2 - (tower_x - x)^2 - (tower_y - y)^2) + z.
        let kin = example_delta();
        let [(arm2, tx, ty), ..] = kin.tower_geometry();
        let fn_a = delta_position_fn(arm2, tx, ty);
        let pos = [10.0, -5.0, 40.0];
        let expected = (arm2 - (tx - pos[0]).powi(2) - (ty - pos[1]).powi(2)).sqrt() + pos[2];
        assert!((fn_a.call(&at(pos), 0.5) - expected).abs() < 1e-9);
        // The same formula via the calibration object's stable position:
        // stable = (abs_endstop - steppos) / stepdist.
        let cal = kin.calibration();
        let steppos = expected;
        let stable = (cal.abs_endstops()[0] - steppos) / cal.stepdists[0];
        assert_eq!(kin.calibration().calc_stable_position(pos)[0], stable);
        assert_eq!(
            delta_active_flags(),
            AxisFlags::X.union(AxisFlags::Y).union(AxisFlags::Z)
        );
    }

    #[test]
    fn test_stable_positions_round_trip_through_trilateration() {
        let cal = example_delta().calibration();
        for coord in [[0.0, 0.0, 100.0], [12.5, -7.25, 30.0], [-40.0, 40.0, 0.5]] {
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
    fn test_calc_position_reads_the_stepper_names_it_knows() {
        let kin = example_delta();
        let cal = kin.calibration();
        let target = [5.0, 13.0, 75.0];
        let mut steppers = HashMap::new();
        for (index, name) in DELTA_RAIL_NAMES.iter().enumerate() {
            // The map holds stepper positions in actuator millimetres (what
            // the delta solver reports), not stable steps.
            let stable = cal.calc_stable_position(target)[index];
            let actuator = cal.abs_endstops()[index] - stable * cal.stepdists[index];
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
    fn test_unified_home_starts_below_the_envelope_and_ends_at_home() {
        let mut kin = example_delta();
        let home = kin.unified_home().expect("delta homes as one move");
        assert!(
            home.force[2] < 0.0,
            "starts below the bed: {}",
            home.force[2]
        );
        // Home is at the endstop height on the centre axis.
        assert!((home.target[0]).abs() < 1e-6);
        assert!((home.target[1]).abs() < 1e-6);
        assert!((home.target[2] - 297.05).abs() < 1e-6, "{}", home.target[2]);
        // Every tower travels the same distance on a symmetric machine, and
        // the travel spans force → home Z.
        let expected = 297.05 - home.force[2];
        for travel in home.actuator_travel {
            assert!((travel - expected).abs() < 1e-6, "{travel} vs {expected}");
        }
        // The homing move leaves the machine homed (`set_position` with all
        // three axes), and its Z is in range for the following moves.
        kin.set_position(
            Coord::new(home.target[0], home.target[1], home.target[2], 0.0),
            &[0, 1, 2],
        );
        assert_eq!(kin.get_status()["homed_axes"], "xyz");
        assert_eq!(kin.unified_home().is_some(), true);
    }

    #[test]
    fn test_check_move_refuses_moves_before_homing_and_outside_the_envelope() {
        let mut kin = example_delta();
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
        // Home it, then a normal Z move passes.
        kin.set_position(Coord::new(0.0, 0.0, 297.05, 0.0), &[0, 1, 2]);
        let mut move_ = crate::core::klippy::motion::plan::Move::new(
            Coord::new(0.0, 0.0, 297.05, 0.0),
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
        // Above the tallest tower is refused too.
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
    fn test_descent_params_follow_the_upstream_adj_order() {
        let cal = example_delta().calibration();
        let basic = cal.descent_params(false);
        assert_eq!(basic.len(), 6);
        assert_eq!(basic[0], cal.radius);
        assert_eq!(basic[1], cal.angles[0]);
        assert_eq!(basic[2], cal.angles[1]);
        assert_eq!(&basic[3..], &cal.endstops);
        let extended = cal.descent_params(true);
        assert_eq!(extended.len(), 9);
        assert_eq!(&extended[6..], &cal.arms);
        // Rebuilding with changed values keeps what the set does not adjust
        // (`new_calibration`): the C angle and the step distances.
        let mut values = extended.clone();
        values[0] = 175.5;
        values[6] = 334.0;
        let rebuilt = DeltaCalibration::from_descent_params(&cal, &values, true);
        assert_eq!(rebuilt.radius, 175.5);
        assert_eq!(rebuilt.arms[0], 334.0);
        assert_eq!(rebuilt.angles[2], cal.angles[2]);
        assert_eq!(rebuilt.stepdists, cal.stepdists);
    }

    #[test]
    fn test_tower_geometry_feeds_the_solver_in_the_upstream_order() {
        let kin = example_delta();
        let [(arm2, tower_x, tower_y), ..] = kin.tower_geometry();
        assert_eq!(arm2, 333.0 * 333.0);
        // Tower A at 210°: (cos·r, sin·r).
        assert!((tower_x - 210.0f64.to_radians().cos() * 174.75).abs() < 1e-9);
        assert!((tower_y - 210.0f64.to_radians().sin() * 174.75).abs() < 1e-9);
    }
}
