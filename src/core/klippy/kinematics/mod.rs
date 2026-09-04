// Kinematics module
//
// This module provides kinematics implementations for Klipper printers.
// Each kinematics type defines how stepper motor positions map to
// toolhead (x, y, z) coordinates.
//
// Available implementations:
// - `none`: Dummy no-op kinematics for testing

pub mod kinematics;
pub mod none;

// Re-export common types
pub use kinematics::{
    Coord, HomingAxes, HomingState, Kinematics, KinematicsError, KinematicsFactory,
    KinematicsStatus, Move, StepperHandle, load_kinematics,
};
pub use none::{NoneKinematics, load_kinematics as load_none_kinematics};
