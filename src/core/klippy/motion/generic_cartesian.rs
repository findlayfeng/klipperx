//! Generic cartesian kinematics (`klippy/kinematics/generic_cartesian.py`).
//!
//! Unlike the cartesian family, where one rail maps to one carriage axis, a
//! generic-cartesian printer declares `[carriage]` sections and `[stepper]`
//! sections whose `carriages` expressions couple several carriages onto one
//! motor. Each stepper therefore follows a **linear combination** of the
//! carriage axes (`chelper/kin_generic.c`):
//!
//! ```text
//! stepper position = c0·x + c1·y + c2·z
//! ```
//!
//! The carriage position is recovered by the pseudo-inverse of the coefficient
//! matrix (`GenericCartesianKinematics.calc_position`), and the configuration is
//! rejected when that matrix is singular — a set of motors that cannot move the
//! axes independently (`_check_kinematics`).
//!
//! This module is the math: the coefficient solver, the linear-combination
//! position function, the matrix check, and the limits/homing code shared with
//! the cartesian family. The section parsing and the carriage/stepper wiring
//! live in [`extras::carriage`](crate::core::klippy::extras::carriage).

use std::collections::HashMap;

use serde_json::{json, Value};

use super::itersolve::{AxisFlags, PositionFn};
use super::kinematics::{home_move, HomingInfo, HomingState, Kinematics, MoveContext};
use super::trapq::MoveSegment;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{mat_transp, pseudo_inverse, Coord, X_AXIS, Y_AXIS, Z_AXIS};

/// One stepper's position function for `generic_cartesian_stepper_alloc`
/// (`kin_generic.c:24-31`): the dot product of its coefficient vector and the
/// move's carriage coordinate.
pub fn generic_cartesian_position(segment: &MoveSegment, move_time: f64, params: &[f64; 6]) -> f64 {
    let c = segment.coord(move_time);
    params[0] * c.x() + params[1] * c.y() + params[2] * c.z()
}

/// The bound solver for one `[stepper]` section with coefficients `coeffs`.
pub fn generic_position_fn(coeffs: [f64; 3]) -> PositionFn {
    PositionFn::bind(
        generic_cartesian_position,
        [coeffs[0], coeffs[1], coeffs[2], 0.0, 0.0, 0.0],
    )
}

/// The axes a generic-cartesian stepper follows
/// (`generic_cartesian_stepper_set_coeffs`, `kin_generic.c:33-40`): the axes
/// whose coefficient is non-zero.
pub fn generic_active_flags(coeffs: [f64; 3]) -> AxisFlags {
    let mut flags = AxisFlags::NONE;
    if coeffs[0] != 0.0 {
        flags = flags.union(AxisFlags::X);
    }
    if coeffs[1] != 0.0 {
        flags = flags.union(AxisFlags::Y);
    }
    if coeffs[2] != 0.0 {
        flags = flags.union(AxisFlags::Z);
    }
    flags
}

/// The generic-cartesian kinematics: a set of kinematic steppers, each
/// following a linear combination of the carriage axes.
///
/// Upstream's `GenericCartesianKinematics` (`kinematics/generic_cartesian.py`),
/// minus the dual-carriage transform (the `[dual_carriage]` state is carried by
/// [`extras::carriage`](crate::core::klippy::extras::carriage) as a coordinate
/// handover rather than a live matrix transform).
#[derive(Debug, Clone)]
pub struct GenericCartesianKinematics {
    /// Each stepper's name and coefficient vector, in config order.
    steppers: Vec<(String, [f64; 3])>,
    /// The homed range of each carriage axis, or `None` while it is unhomed.
    limits: [Option<(f64, f64)>; 3],
    /// The range the **active** carriage of each axis advertises, used when an
    /// axis is homed (`set_position`, `:287-294`).
    active_ranges: [(f64, f64); 3],
    /// The overall carriage span of each axis, for `get_status`.
    axis_min: Coord,
    axis_max: Coord,
    max_z_velocity: f64,
    max_z_accel: f64,
}

impl GenericCartesianKinematics {
    /// Build the kinematics from the stepper coefficient rows, the active
    /// carriage range per axis, and the overall carriage span per axis.
    pub fn new(
        steppers: Vec<(String, [f64; 3])>,
        active_ranges: [(f64, f64); 3],
        axis_min: Coord,
        axis_max: Coord,
        max_z_velocity: f64,
        max_z_accel: f64,
    ) -> Self {
        Self {
            steppers,
            // Upstream starts with `(1.0, -1.0)`, the empty range that reads as
            // "not homed" (`klippy/kinematics/cartesian.py:44`).
            limits: [None; 3],
            active_ranges,
            axis_min,
            axis_max,
            max_z_velocity,
            max_z_accel,
        }
    }

    /// The coefficient matrix, one row per stepper
    /// (`_get_kinematics_coeffs`).
    pub fn matrix(&self) -> Vec<Vec<f64>> {
        self.steppers
            .iter()
            .map(|(_, coeffs)| coeffs.to_vec())
            .collect()
    }

    /// Whether the configuration lets the motors move the axes independently
    /// (`_check_kinematics`, `:255-263`): the normal matrix `MᵀM` must be
    /// non-singular, or a carriage cannot be driven on its own.
    pub fn check_kinematics(&self) -> bool {
        let matrix = self.matrix();
        if matrix.is_empty() {
            return false;
        }
        let Some(mtm) = crate::core::klippy::mathutil::mat_mat_mul(&mat_transp(&matrix), &matrix)
        else {
            return false;
        };
        // Upstream passes an empty right-hand side: it only wants the pivots
        // (`gaussian_solve` returns `None` for a singular system).
        let rhs = vec![Vec::new(); mtm.len()];
        crate::core::klippy::mathutil::gaussian_solve(&mtm, &rhs, false).is_some()
    }

    /// The error for a move past the end of a homed axis.
    fn endstop_error(&self, ctx: &MoveContext<'_>) -> CommandError {
        ctx.out_of_range()
    }

    fn range_for_axis(&self, axis: usize) -> (f64, f64) {
        let (low, high) = self.active_ranges[axis];
        if low <= high {
            (low, high)
        } else {
            let span = (self.axis_min[axis], self.axis_max[axis]);
            if span.0 <= span.1 {
                span
            } else {
                (0.0, 0.0)
            }
        }
    }
}

impl Kinematics for GenericCartesianKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        let mut matrix: Vec<Vec<f64>> = Vec::with_capacity(self.steppers.len());
        let mut spos: Vec<f64> = Vec::with_capacity(self.steppers.len());
        for (name, coeffs) in &self.steppers {
            let Some(position) = stepper_positions.get(name).copied() else {
                return [None, None, None];
            };
            matrix.push(coeffs.to_vec());
            spos.push(position);
        }
        if matrix.is_empty() {
            return [None, None, None];
        }
        let Some(pinv) = pseudo_inverse(&matrix) else {
            return [None, None, None];
        };
        // `pos = [spos] · pinvᵀ` (`mat_mat_mul([[sp-o]], mat_transp(pinv))`).
        let mut out = [None; 3];
        for (axis, row) in pinv.iter().enumerate() {
            // A row of zeros cannot place an axis (`:270-271`).
            if row.iter().any(|v| *v != 0.0) {
                let dot: f64 = row.iter().zip(spos.iter()).map(|(c, s)| c * s).sum();
                out[axis] = Some(dot);
            }
        }
        out
    }

    fn check_move(&self, ctx: &mut MoveContext<'_>) -> Result<(), CommandError> {
        let end = *ctx.end_pos();
        let axes_d = *ctx.axes_d();
        // XY first, as `GenericCartesianKinematics.check_move` does
        // (`generic_cartesian.py:328-350`).
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
        let z_ratio = ctx.move_d() / axes_d[Z_AXIS].abs();
        ctx.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio);
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, homing_axes: &[usize]) {
        for axis in homing_axes {
            self.limits[*axis] = Some(self.range_for_axis(*axis));
        }
    }

    fn update_limits(&mut self, axis: usize, range: Option<(f64, f64)>) {
        if let Some(range) = range {
            // Only an already-homed axis takes new limits
            // (`generic_cartesian.py:274-279`).
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
            "axis_minimum": self.axis_min.as_array(),
            "axis_maximum": self.axis_max.as_array(),
        })
    }

    fn home(&mut self, homing: &mut dyn HomingState) {
        // Each axis is homed independently, in the order the driver asks for
        // (`GenericCartesianKinematics.home`, `:306-315`).
        for axis in homing.axes() {
            let info: HomingInfo = homing.homing_info(axis);
            let (low, high) = self.range_for_axis(axis);
            let (forcepos, movepos) = home_move(axis, &info, low, high);
            homing.home_rails(&[axis], forcepos, movepos);
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::motion::plan::{Move, MoveLimits};

    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 1500.0,
        }
    }

    fn move_(start: Coord, end: Coord) -> Move {
        Move::new(start, end, 100.0, &limits())
    }

    /// The `corexyuv.cfg` coefficient rows (`[stepper a/b/c/d/z/z1]`).
    fn corexyuv_kinematics() -> GenericCartesianKinematics {
        GenericCartesianKinematics::new(
            vec![
                ("stepper a".to_string(), [1.0, 1.0, 0.0]),
                ("stepper b".to_string(), [1.0, -1.0, 0.0]),
                ("stepper c".to_string(), [1.0, -1.0, 0.0]),
                ("stepper d".to_string(), [1.0, 1.0, 0.0]),
                ("stepper z".to_string(), [0.0, 0.0, 1.0]),
                ("stepper z1".to_string(), [0.0, 0.0, 1.0]),
            ],
            [(0.0, 300.0), (0.0, 200.0), (0.0, 100.0)],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(300.0, 200.0, 100.0, 0.0),
            15.0,
            10.0,
        )
    }

    #[test]
    fn test_the_position_function_is_the_linear_combination() {
        use crate::core::klippy::mathutil::Xyz;

        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Xyz::new(3.0, 4.0, 5.0),
            axes_r: Xyz::default(),
        };
        // `c0·x + c1·y + c2·z`.
        assert_eq!(
            generic_cartesian_position(&segment, 0.5, &[1.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            7.0
        );
        assert_eq!(
            generic_cartesian_position(&segment, 0.5, &[1.0, -1.0, 0.0, 0.0, 0.0, 0.0]),
            -1.0
        );
        assert_eq!(
            generic_cartesian_position(&segment, 0.5, &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
            5.0
        );
    }

    #[test]
    fn test_active_flags_follow_the_nonzero_coefficients() {
        assert_eq!(
            generic_active_flags([1.0, 1.0, 0.0]),
            AxisFlags::X.union(AxisFlags::Y)
        );
        assert_eq!(
            generic_active_flags([1.0, -1.0, 0.0]),
            AxisFlags::X.union(AxisFlags::Y)
        );
        assert_eq!(generic_active_flags([0.0, 0.0, 1.0]), AxisFlags::Z);
        assert_eq!(generic_active_flags([0.0, 0.0, 0.0]), AxisFlags::NONE);
    }

    #[test]
    fn test_corexyuv_matrix_is_non_singular() {
        assert!(corexyuv_kinematics().check_kinematics());
    }

    #[test]
    fn test_a_single_coupled_stepper_is_singular() {
        // One motor for `carriage_x+carriage_y` leaves x and y tied together.
        let kin = GenericCartesianKinematics::new(
            vec![("stepper a".to_string(), [1.0, 1.0, 0.0])],
            [(0.0, 300.0), (0.0, 200.0), (0.0, 100.0)],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(300.0, 200.0, 100.0, 0.0),
            15.0,
            10.0,
        );
        assert!(!kin.check_kinematics());
    }

    #[test]
    fn test_calc_position_inverts_a_known_stepper_configuration() {
        let kin = corexyuv_kinematics();
        // A carriage at (10, 20, 30) drives the steppers to a=30, b=-10,
        // c=-10, d=30, z=30, z1=30.
        let positions: HashMap<String, f64> = [
            ("stepper a".to_string(), 30.0),
            ("stepper b".to_string(), -10.0),
            ("stepper c".to_string(), -10.0),
            ("stepper d".to_string(), 30.0),
            ("stepper z".to_string(), 30.0),
            ("stepper z1".to_string(), 30.0),
        ]
        .into_iter()
        .collect();
        let pos = kin.calc_position(&positions);
        assert!((pos[0].unwrap() - 10.0).abs() < 1e-9, "{:?}", pos[0]);
        assert!((pos[1].unwrap() - 20.0).abs() < 1e-9, "{:?}", pos[1]);
        assert!((pos[2].unwrap() - 30.0).abs() < 1e-9, "{:?}", pos[2]);

        // A missing stepper leaves every axis undeterminable.
        assert_eq!(kin.calc_position(&HashMap::new()), [None, None, None]);
    }

    #[test]
    fn test_a_move_before_homing_is_refused() {
        let kin = corexyuv_kinematics();
        let mut before = move_(Coord::default(), Coord::new(10.0, 0.0, 0.0, 0.0));
        let err = kin
            .check_move(&mut MoveContext::new(&mut before))
            .unwrap_err();
        assert!(err.to_string().contains("Must home axis first"), "{err}");
    }

    #[test]
    fn test_a_homed_corexyuv_accepts_the_corpus_moves() {
        let mut kin = corexyuv_kinematics();
        kin.set_position(Coord::default(), &[X_AXIS, Y_AXIS, Z_AXIS]);

        // `G1 X170 Y190 F6000` from `corexyuv.test`: inside `[0,300]×[0,200]`.
        let mut inside = move_(Coord::default(), Coord::new(170.0, 190.0, 0.0, 0.0));
        assert!(kin.check_move(&mut MoveContext::new(&mut inside)).is_ok());

        // The T1 park's exact corner `X300 Y200` is inclusive.
        let mut corner = move_(
            Coord::new(170.0, 190.0, 0.0, 0.0),
            Coord::new(300.0, 200.0, 0.0, 0.0),
        );
        assert!(kin.check_move(&mut MoveContext::new(&mut corner)).is_ok());

        // One millimetre past the far corner is refused.
        let mut past = move_(Coord::default(), Coord::new(301.0, 0.0, 0.0, 0.0));
        let err = kin
            .check_move(&mut MoveContext::new(&mut past))
            .unwrap_err();
        assert!(err.to_string().contains("Move out of range"), "{err}");
    }

    #[test]
    fn test_status_reports_the_homed_axes_and_the_carriage_span() {
        let mut kin = corexyuv_kinematics();
        assert_eq!(kin.get_status()["homed_axes"], "");
        kin.set_position(Coord::default(), &[X_AXIS, Y_AXIS, Z_AXIS]);
        assert_eq!(kin.get_status()["homed_axes"], "xyz");
        assert_eq!(
            kin.get_status()["axis_minimum"],
            json!([0.0, 0.0, 0.0, 0.0])
        );
        assert_eq!(
            kin.get_status()["axis_maximum"],
            json!([300.0, 200.0, 100.0, 0.0])
        );
        kin.clear_homing_state(&[Y_AXIS]);
        assert_eq!(kin.get_status()["homed_axes"], "xz");
    }

    #[test]
    fn test_update_limits_only_touches_a_homed_axis() {
        let mut kin = corexyuv_kinematics();
        // Unhomed: the new range is dropped.
        kin.update_limits(X_AXIS, Some((5.0, 50.0)));
        kin.set_position(Coord::default(), &[X_AXIS]);
        assert_eq!(kin.limits[X_AXIS], Some((0.0, 300.0)));

        // Homed: the new range lands.
        kin.update_limits(X_AXIS, Some((5.0, 50.0)));
        assert_eq!(kin.limits[X_AXIS], Some((5.0, 50.0)));
    }
}
