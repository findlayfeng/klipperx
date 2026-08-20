// Klipper kinematics trait
//
// This module defines the abstract interface for all Klipper kinematics
// implementations (cartesian, delta, corexy, winch, polar, etc.).
//
// Each kinematics type defines how stepper motor positions map to
// toolhead (x, y, z) coordinates. The trait captures the common interface
// shared by all kinematics implementations.
//
// Reference: `third_party/klipper/klippy/kinematics/`
//   cartesian.py  - CartKinematics
//   corexy.py     - CoreXYKinematics
//   corexz.py     - CoreXZKinematics
//   delta.py      - DeltaKinematics
//   deltesian.py  - DeltesianKinematics
//   polar.py      - PolarKinematics
//   rotary_delta.py - RotaryDeltaKinematics
//   winch.py      - WinchKinematics
//   none.py       - NoneKinematics

use std::collections::HashMap;

/// A 3D coordinate (x, y, z), mirroring Klipper's `Coord` tuple.
///
/// `Coord` is a simple tuple-like struct with named accessors for x, y, z components.
/// In Klipper's Python code, it's implemented as a `tuple` subclass with properties.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Coord {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Coord {
    /// Create a new Coord from x, y, z values.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Create a Coord from an array of 3 f64 values.
    pub fn from_array(arr: [f64; 3]) -> Self {
        Self {
            x: arr[0],
            y: arr[1],
            z: arr[2],
        }
    }

    /// Convert to an array of 3 f64 values.
    pub fn to_array(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Check if this coordinate is approximately zero.
    pub fn is_near_zero(&self, epsilon: f64) -> bool {
        self.x.abs() < epsilon
            && self.y.abs() < epsilon
            && self.z.abs() < epsilon
    }
}

/// Axis identifier: x, y, or z.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    /// Get the axis as a character ('x', 'y', 'z').
    pub fn as_char(&self) -> char {
        match self {
            Axis::X => 'x',
            Axis::Y => 'y',
            Axis::Z => 'z',
        }
    }

    /// Get the axis as a string ("x", "y", "z").
    pub fn as_str(&self) -> &'static str {
        match self {
            Axis::X => "x",
            Axis::Y => "y",
            Axis::Z => "z",
        }
    }

    /// Parse an axis from a character.
    pub fn from_char(c: char) -> Option<Self> {
        match c {
            'x' | 'X' => Some(Axis::X),
            'y' | 'Y' => Some(Axis::Y),
            'z' | 'Z' => Some(Axis::Z),
            _ => None,
        }
    }

    /// Get the index (0=x, 1=y, 2=z).
    pub fn index(&self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }
}

/// Axes being homed, represented as a bitmask or boolean array.
///
/// In Klipper, homing_axes is a string like "x", "xy", "xyz".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HomingAxes {
    pub x: bool,
    pub y: bool,
    pub z: bool,
}

impl HomingAxes {
    /// Create a new empty HomingAxes.
    pub const fn new() -> Self {
        Self {
            x: false,
            y: false,
            z: false,
        }
    }

    /// Create a HomingAxes from a boolean array [x, y, z].
    pub const fn from_array(arr: [bool; 3]) -> Self {
        Self {
            x: arr[0],
            y: arr[1],
            z: arr[2],
        }
    }

    /// Convert to a boolean array [x, y, z].
    pub fn to_array(&self) -> [bool; 3] {
        [self.x, self.y, self.z]
    }

    /// Check if any axis is homed.
    pub fn is_any(&self) -> bool {
        self.x || self.y || self.z
    }

    /// Check if all axes are homed.
    pub fn is_all(&self) -> bool {
        self.x && self.y && self.z
    }

    /// Add an axis to the set.
    pub fn set(&mut self, axis: Axis, value: bool) {
        match axis {
            Axis::X => self.x = value,
            Axis::Y => self.y = value,
            Axis::Z => self.z = value,
        }
    }

    /// Get the set of homed axes.
    pub fn homed_axes(&self) -> Vec<Axis> {
        let mut axes = Vec::new();
        if self.x {
            axes.push(Axis::X);
        }
        if self.y {
            axes.push(Axis::Y);
        }
        if self.z {
            axes.push(Axis::Z);
        }
        axes
    }

    /// Parse from a Klipper-style string like "x", "xy", "xyz".
    pub fn from_string(s: &str) -> Self {
        let mut result = Self::new();
        for c in s.chars() {
            match c {
                'x' | 'X' => result.x = true,
                'y' | 'Y' => result.y = true,
                'z' | 'Z' => result.z = true,
                _ => {}
            }
        }
        result
    }
}

impl std::fmt::Display for HomingAxes {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = String::new();
        if self.x {
            s.push('x');
        }
        if self.y {
            s.push('y');
        }
        if self.z {
            s.push('z');
        }
        write!(f, "{}", s)
    }
}

/// Homing state for tracking homing operations.
///
/// Mirrors the homing state concept from Klipper's `extras/homing.py`.
/// Used by `Kinematics::home()` for all kinematics types.
#[derive(Debug, Clone)]
pub struct HomingState {
    /// The axes being homed
    axes: HomingAxes,
}

impl HomingState {
    pub fn new() -> Self {
        Self {
            axes: HomingAxes::new(),
        }
    }

    /// Set the axes being homed.
    /// Mirrors `HomingMove.set_axes()` in klipper.py.
    pub fn set_axes(&mut self, axes: HomingAxes) {
        self.axes = axes;
    }

    /// Get the axes being homed.
    pub fn get_axes(&self) -> &HomingAxes {
        &self.axes
    }
}

impl Default for HomingState {
    fn default() -> Self {
        Self::new()
    }
}

/// A move to be validated by kinematics.
///
/// Mirrors the move concept from Klipper's `toolhead.py`.
/// Used by `Kinematics::check_move()` for move validation.
///
/// Key fields:
/// - `start_pos` / `end_pos`: start and end positions
/// - `axes_d`: delta (difference) between start and end
/// - `axes_r`: ratio of each axis delta to total move distance
/// - `move_d`: total move distance
#[derive(Debug, Clone)]
pub struct Move {
    /// Start position
    pub start_pos: Coord,
    /// End position
    pub end_pos: Coord,
    /// Delta (end - start) for each axis
    pub axes_d: [f64; 3],
    /// Total move distance
    pub move_d: f64,
    /// Speed of the move
    pub speed: f64,
    /// Acceleration limit
    pub accel: f64,
    /// Per-axis ratios (axes_d / move_d)
    pub axes_r: [f64; 3],
}

impl Move {
    /// Create a new Move from start and end positions.
    pub fn new(start_pos: Coord, end_pos: Coord, speed: f64, accel: f64) -> Self {
        let axes_d = [
            end_pos.x - start_pos.x,
            end_pos.y - start_pos.y,
            end_pos.z - start_pos.z,
        ];
        let move_d = (axes_d[0].powi(2)
            + axes_d[1].powi(2)
            + axes_d[2].powi(2))
        .sqrt();
        let inv_move_d = if move_d > 0.0 { 1.0 / move_d } else { 0.0 };
        let axes_r = [
            axes_d[0] * inv_move_d,
            axes_d[1] * inv_move_d,
            axes_d[2] * inv_move_d,
        ];
        Self {
            start_pos,
            end_pos,
            axes_d,
            move_d,
            speed,
            accel,
            axes_r,
        }
    }

    /// Check if this is a kinematic move (non-zero XYZ movement).
    pub fn is_kinematic_move(&self) -> bool {
        self.move_d > 1e-9
    }
}

/// Status information returned by `get_status()`.
///
/// Mirrors the return value of kinematics `get_status()` in klippy.py:
/// ```python
/// return {
///     'homed_axes': 'xyz',  # string of homed axes
///     'axis_minimum': Coord(...),
///     'axis_maximum': Coord(...),
/// }
/// ```
#[derive(Debug, Clone)]
pub struct KinematicsStatus {
    /// Compressed string of homed axes, e.g., "xyz" or ""
    pub homed_axes: String,
    /// Minimum axis bounds
    pub axis_minimum: Coord,
    /// Maximum axis bounds
    pub axis_maximum: Coord,
}

impl KinematicsStatus {
    pub fn new(
        homed_axes: &str,
        axis_minimum: Coord,
        axis_maximum: Coord,
    ) -> Self {
        Self {
            homed_axes: homed_axes.to_string(),
            axis_minimum,
            axis_maximum,
        }
    }
}

/// Abstract trait for Klipper kinematics implementations.
///
/// This trait captures the common interface shared by all kinematics
/// classes in `third_party/klipper/klippy/kinematics/`:
///   - `CartKinematics` (cartesian.py)
///   - `CoreXYKinematics` (corexy.py)
///   - `CoreXZKinematics` (corexz.py)
///   - `DeltaKinematics` (delta.py)
///   - `DeltesianKinematics` (deltesian.py)
///   - `PolarKinematics` (polar.py)
///   - `RotaryDeltaKinematics` (rotary_delta.py)
///   - `WinchKinematics` (winch.py)
///   - `NoneKinematics` (none.py)
///
/// Each kinematics type defines how stepper motor positions map to
/// toolhead (x, y, z) coordinates.
///
/// # Methods
/// - `get_steppers()`: Return the list of steppers for this kinematics
/// - `calc_position()`: Calculate (x, y, z) from stepper positions
/// - `set_position()`: Set internal position tracking
/// - `clear_homing_state()`: Clear homing state for specified axes
/// - `home()`: Perform a homing operation
/// - `check_move()`: Validate a move against kinematic constraints
/// - `get_status()`: Return current status for UI display
///
/// # Example
/// ```ignore
/// let kin: Box<dyn Kinematics> = Box::new(CartKinematics::new(config));
/// let steppers = kin.get_steppers();
/// let pos = kin.calc_position(&stepper_positions);
/// kin.set_position(pos, HomingAxes::from_string("xyz"));
/// ```
pub trait Kinematics: Send + Sync {
    /// Get the list of steppers for this kinematics.
    ///
    /// Mirrors `get_steppers(self)` in klippy.py.
    ///
    /// In Klipper, steppers are `PrinterStepper` objects. In Rust,
    /// this returns a list of stepper identifiers or handles.
    fn get_steppers(&self) -> Vec<StepperHandle>;

    /// Calculate the toolhead position from stepper positions.
    ///
    /// Mirrors `calc_position(self, stepper_positions)` in klippy.py.
    ///
    /// This is the core kinematic transformation: converting individual
    /// stepper positions into Cartesian (x, y, z) coordinates.
    ///
    /// Each kinematics type implements this differently:
    /// - **Cartesian**: direct mapping, each rail maps to one axis
    /// - **CoreXY**: [0.5*(pos[0]+pos[1]), 0.5*(pos[0]-pos[1]), pos[2]]
    /// - **Delta**: trilateration of three tower spheres
    ///
    /// # Arguments
    /// * `stepper_positions` - Map of stepper name to position
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> Coord;

    /// Set the internal position tracking.
    ///
    /// Mirrors `set_position(self, newpos, homing_axes)` in klippy.py.
    ///
    /// Called after homing or at startup to initialize position tracking.
    ///
    /// # Arguments
    /// * `newpos` - New (x, y, z) position
    /// * `homing_axes` - Which axes were homed (affects limit tracking)
    fn set_position(&self, newpos: Coord, homing_axes: &HomingAxes);

    /// Clear the homing state for specified axes.
    ///
    /// Mirrors `clear_homing_state(self, clear_axes)` in klippy.py.
    ///
    /// Resets limit tracking for the specified axes, putting them
    /// back into an "unhomed" state.
    ///
    /// # Arguments
    /// * `clear_axes` - Which axes to clear homing state for
    fn clear_homing_state(&self, clear_axes: &HomingAxes);

    /// Perform a homing operation.
    ///
    /// Mirrors `home(self, homing_state)` in klippy.py.
    ///
    /// Each kinematics type homes differently:
    /// - **Cartesian/CoreXY**: each axis homed independently in order
    /// - **Delta**: all axes homed simultaneously
    ///
    /// # Arguments
    /// * `homing_state` - The homing state object tracking progress
    fn home(&self, homing_state: &mut HomingState);

    /// Validate a move for kinematic constraints.
    ///
    /// Mirrors `check_move(self, move)` in klippy.py.
    ///
    /// Called before executing a move to ensure it's within bounds.
    /// May modify the move's speed/acceleration limits.
    ///
    /// # Arguments
    /// * `move_obj` - The move to validate
    ///
    /// # Errors
    /// Returns an error if the move is invalid (out of range, must home first, etc.)
    fn check_move(&self, move_obj: &Move) -> Result<(), KinematicsError>;

    /// Get the status information for this kinematics.
    ///
    /// Mirrors `get_status(self, eventtime)` in klippy.py.
    ///
    /// Returns data used by the UI to display homed axes and axis bounds.
    ///
    /// # Arguments
    /// * `eventtime` - Current event time (for async status updates)
    fn get_status(&self, _eventtime: Option<f64>) -> KinematicsStatus;
}

/// Error returned by kinematics operations.
#[derive(Debug, Clone, PartialEq)]
pub enum KinematicsError {
    /// Move is out of range
    OutOfRange,
    /// Axis must be homed first
    MustHome,
    /// Invalid move parameters
    InvalidMove(String),
}

impl std::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KinematicsError::OutOfRange => write!(f, "Move out of range"),
            KinematicsError::MustHome => write!(f, "Must home axis first"),
            KinematicsError::InvalidMove(msg) => write!(f, "{}", msg),
        }
    }
}

impl std::error::Error for KinematicsError {}

/// A handle to a stepper motor for this kinematics.
///
/// In Klipper, this is a `PrinterStepper` object. In Rust, it's an
/// identifier that can be used to query stepper state.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct StepperHandle {
    /// Stepper name (e.g., "stepper_x", "stepper_y", "stepper_z")
    pub name: String,
}

impl StepperHandle {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
        }
    }
}

impl std::fmt::Display for StepperHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name)
    }
}

/// Factory function type for creating kinematics instances.
///
/// Mirrors `load_kinematics(toolhead, config)` in klippy.py.
/// Each kinematics module exports this function.
pub type KinematicsFactory = fn() -> Box<dyn Kinematics>;

/// Load a kinematics implementation by name.
///
/// Given a kinematics type name (e.g., "cartesian", "delta", "corexy", "none"),
/// returns a boxed `Kinematics` trait object.
///
/// This is the Rust equivalent of Klipper's module import system:
/// ```python
/// mod = importlib.import_module('kinematics.' + module_name)
/// return mod.load_kinematics(toolhead, config)
/// ```
///
/// # Arguments
/// * `kinematics_type` - The kinematics type name (e.g., "none", "cartesian")
pub fn load_kinematics(kinematics_type: &str) -> Box<dyn Kinematics> {
    match kinematics_type {
        "none" => load_none_kinematics(),
        _ => {
            // Placeholder: other kinematics will be implemented later
            load_none_kinematics()
        }
    }
}

/// Factory for NoneKinematics.
fn load_none_kinematics() -> Box<dyn Kinematics> {
    use super::none::NoneKinematics;
    Box::new(NoneKinematics::new())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_coord() {
        let c = Coord::new(1.0, 2.0, 3.0);
        assert_eq!(c.x, 1.0);
        assert_eq!(c.y, 2.0);
        assert_eq!(c.z, 3.0);
        assert_eq!(c.to_array(), [1.0, 2.0, 3.0]);

        let c2 = Coord::from_array([4.0, 5.0, 6.0]);
        assert_eq!(c2.x, 4.0);
        assert_eq!(c2.to_array()[2], 6.0);

        assert!(Coord::new(0.0, 0.0, 0.0).is_near_zero(1e-9));
        assert!(!Coord::new(1e-10, 0.0, 0.0).is_near_zero(1e-11));
    }

    #[test]
    fn test_homing_axes() {
        let mut axes = HomingAxes::new();
        assert!(!axes.is_any());
        assert!(!axes.is_all());

        axes.set(Axis::X, true);
        axes.set(Axis::Y, true);
        assert!(axes.is_any());
        assert!(!axes.is_all());

        axes.set(Axis::Z, true);
        assert!(axes.is_all());

        assert_eq!(axes.to_array(), [true, true, true]);
        assert_eq!(axes.homed_axes(), vec![Axis::X, Axis::Y, Axis::Z]);
        assert_eq!(axes.to_string(), "xyz");

        let parsed = HomingAxes::from_string("xz");
        assert!(parsed.x);
        assert!(!parsed.y);
        assert!(parsed.z);
    }

    #[test]
    fn test_axis() {
        assert_eq!(Axis::X.as_str(), "x");
        assert_eq!(Axis::Y.as_char(), 'y');
        assert_eq!(Axis::Z.index(), 2);
        assert_eq!(Axis::from_char('x'), Some(Axis::X));
        assert_eq!(Axis::from_char('w'), None);
    }

    #[test]
    fn test_move() {
        let start = Coord::new(0.0, 0.0, 0.0);
        let end = Coord::new(3.0, 4.0, 0.0);
        let move_obj = Move::new(start, end, 100.0, 3000.0);

        assert!(move_obj.is_kinematic_move());
        assert!((move_obj.move_d - 5.0).abs() < 1e-9);
        assert!((move_obj.axes_d[0] - 3.0).abs() < 1e-9);
        assert!((move_obj.axes_d[1] - 4.0).abs() < 1e-9);
        assert!((move_obj.axes_d[2] - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_move_extrude_only() {
        let start = Coord::new(10.0, 10.0, 10.0);
        let end = Coord::new(10.0, 10.0, 10.0);
        let move_obj = Move::new(start, end, 5.0, 3000.0);

        assert!(!move_obj.is_kinematic_move());
        assert_eq!(move_obj.move_d, 0.0);
    }

    #[test]
    fn test_stepper_handle() {
        let h = StepperHandle::new("stepper_x");
        assert_eq!(h.name, "stepper_x");
        assert_eq!(h.to_string(), "stepper_x");
    }

    #[test]
    fn test_load_kinematics_none() {
        let kin = load_kinematics("none");
        let pos = kin.calc_position(&HashMap::new());
        assert_eq!(pos, Coord::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_kinematics_status() {
        let status = KinematicsStatus::new(
            "xyz",
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(300.0, 300.0, 200.0),
        );
        assert_eq!(status.homed_axes, "xyz");
        assert_eq!(status.axis_maximum.x, 300.0);
    }

    #[test]
    fn test_kinematics_error_display() {
        let err = KinematicsError::OutOfRange;
        assert_eq!(err.to_string(), "Move out of range");

        let err = KinematicsError::MustHome;
        assert_eq!(err.to_string(), "Must home axis first");

        let err = KinematicsError::InvalidMove("bad config".to_string());
        assert_eq!(err.to_string(), "bad config");
    }
}
