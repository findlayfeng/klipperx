//! Extra axes: the non-kinematic axes a move can carry, i.e. the extruder.
//!
//! Upstream's `ToolHead.extra_axes` (`klippy/toolhead.py:238-241`) is a list of
//! `PrinterExtruder` objects sitting on axis index 3 and up. They take part in
//! the planner exactly like an axis (junction limit, per-move check) but do not
//! belong to the kinematics: they have **their own trapq**, and their steppers
//! read the extrusion amount out of it (`kinematics/extruder.py`).
//!
//! This trait is the narrow interface the toolhead and the look-ahead queue use;
//! the extruder in `extras` implements it.

use serde_json::Value;

use super::kinematics::MoveContext;
use super::plan::Move;
use super::queuing::MotionQueuing;
use crate::core::klippy::gcode::CommandError;

/// One non-kinematic axis (the extruder).
///
/// Methods take `&self`: the axis is shared (`Arc`) between the printer's
/// object registry and the toolhead, so anything mutable lives behind interior
/// mutability. `ea_index` is the axis' index in the toolhead position (`3` for
/// the first extra axis), as upstream passes `e_index + 3`.
pub trait ExtraAxis: Send + Sync + std::fmt::Debug {
    /// The name in `toolhead.extra_axes` status (`extruder`, `extruder1`).
    fn name(&self) -> &str;

    /// Check a move's extrusion (`PrinterExtruder.check_move`).
    ///
    /// # Errors
    /// The client-visible error when the move is refused (too much extrusion,
    /// a cold hotend, an extrude-only move that is too long).
    fn check_move(&self, ctx: &mut MoveContext<'_>, ea_index: usize) -> Result<(), CommandError>;

    /// The junction speed the axis allows between `prev` and `cur`, squared.
    ///
    /// `PrinterExtruder.calc_junction`: a sudden change in extrusion rate limits
    /// the corner speed by `instantaneous_corner_velocity`.
    fn calc_junction(&self, prev: &Move, cur: &Move, ea_index: usize) -> f64;

    /// Queue `move`'s extrusion into this axis' trapq
    /// (`PrinterExtruder.process_move`).
    fn process_move(
        &self,
        queuing: &mut MotionQueuing,
        print_time: f64,
        move_: &Move,
        ea_index: usize,
    );

    /// The axis' position at a past print time, for a motion report
    /// (`PrinterExtruder.find_past_position`).
    fn find_past_position(&self, print_time: f64) -> f64;

    /// The axis' `get_status` fields, folded into the toolhead's status.
    fn get_status(&self) -> Value;
}
