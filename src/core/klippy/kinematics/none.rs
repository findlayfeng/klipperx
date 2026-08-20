// Dummy "none" kinematics support (for developer testing)
//
// This module implements a no-op kinematics class for Klipper, mirroring
// `third_party/klipper/klippy/kinematics/none.py`.
//
// Unlike real kinematics (cartesian, delta, corexy, etc.), NoneKinematics:
// - Returns no steppers
// - Always reports position [0, 0, 0]
// - All operations (homing, move validation, position setting) are no-ops
//
// This is useful for testing Klipper's host-side logic without a real printer.

use std::collections::HashMap;

use super::kinematics::{
    Coord, HomingAxes, HomingState, Kinematics, KinematicsError, KinematicsStatus, Move,
    StepperHandle,
};

/// Dummy kinematics that performs no coordinate transformation.
///
/// Mirrors `NoneKinematics` in `third_party/klipper/klippy/kinematics/none.py`.
///
/// # Fields
/// - `axes_minmax`: The minimum/maximum axis bounds, always `(0.0, 0.0, 0.0)`.
///   Mirrors `self.axes_minmax = toolhead.Coord((0., 0., 0.))` in the original.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NoneKinematics {
    /// Axis minimum/maximum bounds (x, y, z).
    /// Always zero since no real axes exist.
    pub axes_minmax: Coord,
}

impl NoneKinematics {
    /// Create a new NoneKinematics instance.
    ///
    /// Mirrors `NoneKinematics.__init__(self, toolhead, config)` in klippy.py.
    /// The `toolhead` and `config` parameters are ignored since this is a no-op
    /// kinematics used only for testing.
    pub fn new() -> Self {
        Self {
            axes_minmax: Coord::new(0.0, 0.0, 0.0),
        }
    }
}

impl Default for NoneKinematics {
    fn default() -> Self {
        Self::new()
    }
}

impl Kinematics for NoneKinematics {
    /// Get the list of steppers for this kinematics.
    ///
    /// Mirrors `get_steppers(self)` in klippy.py.
    /// Returns an empty list since there are no real steppers.
    fn get_steppers(&self) -> Vec<StepperHandle> {
        Vec::new()
    }

    /// Calculate the toolhead position from stepper positions.
    ///
    /// Mirrors `calc_position(self, stepper_positions)` in klippy.py.
    /// Always returns `[0, 0, 0]` since no real axes exist.
    fn calc_position(&self, _stepper_positions: &HashMap<String, f64>) -> Coord {
        Coord::new(0.0, 0.0, 0.0)
    }

    /// Set the current toolhead position.
    ///
    /// Mirrors `set_position(self, newpos, homing_axes)` in klippy.py.
    /// No-op for dummy kinematics.
    fn set_position(&self, _newpos: Coord, _homing_axes: &HomingAxes) {
        // No-op
    }

    /// Clear the homing state for specified axes.
    ///
    /// Mirrors `clear_homing_state(self, clear_axes)` in klippy.py.
    /// No-op for dummy kinematics.
    fn clear_homing_state(&self, _clear_axes: &HomingAxes) {
        // No-op
    }

    /// Perform a homing operation.
    ///
    /// Mirrors `home(self, homing_state)` in klippy.py.
    /// No-op for dummy kinematics.
    fn home(&self, _homing_state: &mut HomingState) {
        // No-op
    }

    /// Validate a move for kinematic constraints.
    ///
    /// Mirrors `check_move(self, move)` in klippy.py.
    /// No-op for dummy kinematics — all moves are accepted.
    fn check_move(&self, _move: &Move) -> Result<(), KinematicsError> {
        Ok(())
    }

    /// Get the status information for this kinematics.
    ///
    /// Mirrors `get_status(self, eventtime)` in klippy.py.
    fn get_status(&self, _eventtime: Option<f64>) -> KinematicsStatus {
        KinematicsStatus::new("", self.axes_minmax, self.axes_minmax)
    }
}

/// Load function for creating a NoneKinematics instance.
///
/// Mirrors `load_kinematics(toolhead, config)` in klippy.py.
/// In a full implementation, this would be called by the config loader
/// when it encounters `kinematics: none` in the config file.
pub fn load_kinematics() -> NoneKinematics {
    NoneKinematics::new()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_kinematics() {
        let kin = NoneKinematics::new();
        assert_eq!(kin.axes_minmax, Coord::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_get_steppers() {
        let kin = NoneKinematics::new();
        let steppers = kin.get_steppers();
        assert!(steppers.is_empty());
    }

    #[test]
    fn test_calc_position() {
        let kin = NoneKinematics::new();
        let mut positions = HashMap::new();
        positions.insert("stepper_x".to_string(), 100.0);
        positions.insert("stepper_y".to_string(), 200.0);

        let pos = kin.calc_position(&positions);
        assert_eq!(pos, Coord::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_set_position_is_noop() {
        let kin = NoneKinematics::new();
        kin.set_position(
            Coord::new(10.0, 20.0, 30.0),
            &HomingAxes::from_string("xyz"),
        );
    }

    #[test]
    fn test_clear_homing_state_is_noop() {
        let kin = NoneKinematics::new();
        kin.clear_homing_state(&HomingAxes::from_string("xz"));
    }

    #[test]
    fn test_home_is_noop() {
        let kin = NoneKinematics::new();
        kin.home(&mut HomingState::new());
    }

    #[test]
    fn test_check_move_is_ok() {
        let kin = NoneKinematics::new();
        let start = Coord::new(0.0, 0.0, 0.0);
        let end = Coord::new(10.0, 10.0, 10.0);
        let move_obj = Move::new(start, end, 500.0, 3000.0);
        assert!(kin.check_move(&move_obj).is_ok());
    }

    #[test]
    fn test_get_status() {
        let kin = NoneKinematics::new();
        let status = kin.get_status(None);
        assert_eq!(status.homed_axes, "");
        assert_eq!(status.axis_minimum, Coord::new(0.0, 0.0, 0.0));
        assert_eq!(status.axis_maximum, Coord::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_load_kinematics() {
        let kin = load_kinematics();
        assert_eq!(kin.axes_minmax, Coord::default());
    }
}
