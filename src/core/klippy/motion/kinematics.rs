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
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, X_AXIS, Y_AXIS, Z_AXIS};

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

    /// The per-axis distance.
    pub fn axes_d(&self) -> &[f64; 4] {
        &self.move_.axes_d
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

    /// The error for a move before the axis is homed.
    pub fn must_home(&self) -> CommandError {
        self.move_.move_error("Must home axis first")
    }
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
}

/// Cartesian kinematics: one stepper per axis, straight-line limits.
///
/// Upstream's `CartKinematics` (`klippy/kinematics/cartesian.py`), without the
/// dual-carriage extension.
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
}

impl CartesianKinematics {
    /// A cartesian kinematics over `axes` (`stepper_x` …), travelling between
    /// `axes_min` and `axes_max`, with the Z axis limited to `max_z_velocity`
    /// and `max_z_accel`.
    pub fn new(
        axes: [String; 3],
        axes_min: Coord,
        axes_max: Coord,
        max_z_velocity: f64,
        max_z_accel: f64,
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
}

impl Kinematics for CartesianKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        let mut out = [None; 3];
        for (axis, name) in self.axes.iter().enumerate() {
            out[axis] = stepper_positions.get(name).copied();
        }
        out
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
}
