//! The toolhead: where moves are planned and print time is tracked.
//!
//! Upstream's `klippy/toolhead.py` `ToolHead`: it holds the look-ahead queue,
//! the commanded position and the print time, and feeds the motion queue. The
//! kinematics — which positions are legal, how to home — is FW5e; this is the
//! skeleton a `G1` drives.

use std::sync::Arc;

use super::extra::ExtraAxis;
use super::kinematics::{Kinematics, MoveContext};
use super::plan::{LookAheadQueue, Move, MoveLimits};
use super::queuing::MotionQueuing;
use super::stepcompress::{StepCommand, StepCompressError};
use super::stepper::Stepper;
use super::trapq::Trapq;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::{Coord, Xyz, E_AXIS};

/// How far ahead of the MCU the planner starts, in seconds
/// (`BUFFER_TIME_START`, `klippy/toolhead.py:196`).
pub const BUFFER_TIME_START: f64 = 0.250;

/// The toolhead: commanded position, print time, and the motion queue.
#[derive(Debug)]
pub struct ToolHead {
    limits: MoveLimits,
    lookahead: LookAheadQueue,
    commanded_pos: Coord,
    print_time: f64,
    estimated_print_time: f64,
    special_queuing_state: bool,
    motion_queuing: MotionQueuing,
    kinematics: Option<Box<dyn Kinematics>>,
    /// The trapq the kinematic move is appended to; each extra axis has its
    /// own (`MotionQueuing::allocate_trapq`).
    main_trapq: usize,
    /// The non-kinematic axes (extruders), on position index 3 and up.
    extra_axes: Vec<Arc<dyn ExtraAxis>>,
}

impl ToolHead {
    /// A toolhead with `limits`.
    pub fn new(limits: MoveLimits) -> Self {
        let mut motion_queuing = MotionQueuing::new();
        let main_trapq = motion_queuing.allocate_trapq();
        Self {
            limits,
            lookahead: LookAheadQueue::new(),
            commanded_pos: Coord::default(),
            print_time: 0.0,
            estimated_print_time: 0.0,
            // Upstream starts in "NeedPrime" and resyncs the print time on the
            // first planned move (`klippy/toolhead.py:224`).
            special_queuing_state: true,
            motion_queuing,
            kinematics: None,
            main_trapq,
            extra_axes: Vec::new(),
        }
    }

    /// Add a stepper to drive. It reads the main trapq.
    pub fn add_stepper(&mut self, mut stepper: Stepper) {
        stepper.set_trapq(self.main_trapq);
        self.motion_queuing.add_stepper(stepper);
    }

    /// Add a non-kinematic axis (the extruder).
    ///
    /// The caller owns the axis' trapq and has already pointed its stepper at
    /// it (`PrinterExtruder.stepper.set_trapq`).
    pub fn add_extra_axis(&mut self, axis: Arc<dyn ExtraAxis>) {
        self.extra_axes.push(axis);
    }

    /// The non-kinematic axes.
    pub fn extra_axes(&self) -> &[Arc<dyn ExtraAxis>] {
        &self.extra_axes
    }

    /// Remove a non-kinematic axis by identity (`ToolHead.remove_extra_axis`).
    ///
    /// A `GCODE_AXIS=` request takes the manual stepper back off the axis list;
    /// upstream compares by object identity (`ea not in self.extra_axes`), so
    /// this matches on the shared allocation.
    pub fn remove_extra_axis(&mut self, axis: &Arc<dyn ExtraAxis>) {
        if let Some(index) = self
            .extra_axes
            .iter()
            .position(|candidate| Arc::ptr_eq(candidate, axis))
        {
            self.extra_axes.remove(index);
        }
    }

    /// The id of the main trapq, for an extra axis that needs one of its own.
    pub fn main_trapq(&self) -> usize {
        self.main_trapq
    }

    /// Allocate a trapq for an extra axis.
    pub fn allocate_trapq(&mut self) -> usize {
        self.motion_queuing.allocate_trapq()
    }

    /// Install the kinematics (upstream loads it from `[printer] kinematics`).
    pub fn set_kinematics(&mut self, kinematics: Box<dyn Kinematics>) {
        self.kinematics = Some(kinematics);
    }

    /// The kinematics, for reporting homing state and limits.
    pub fn kinematics(&self) -> Option<&dyn Kinematics> {
        self.kinematics.as_deref()
    }

    /// The kinematics, to change its state (`SET_KINEMATIC_POSITION`'s
    /// `CLEAR_HOMED`).
    pub fn kinematics_mut(&mut self) -> Option<&mut (dyn Kinematics + 'static)> {
        match &mut self.kinematics {
            Some(kinematics) => Some(kinematics.as_mut()),
            None => None,
        }
    }

    /// Where the toolhead has been commanded to.
    pub fn commanded_pos(&self) -> Coord {
        self.commanded_pos
    }

    /// The print time the planner has reached.
    pub fn print_time(&self) -> f64 {
        self.print_time
    }

    /// The trapezoid queue the kinematic move is appended to.
    pub fn trapq(&self) -> &Trapq {
        self.motion_queuing.trapq(self.main_trapq)
    }

    /// The motion queue, for setting stepper positions.
    pub fn motion_queuing_mut(&mut self) -> &mut MotionQueuing {
        &mut self.motion_queuing
    }

    /// The MCU's estimated print time, used when planning starts
    /// (`MCU.estimated_print_time`).
    pub fn set_estimated_print_time(&mut self, print_time: f64) {
        self.estimated_print_time = print_time;
    }

    /// Queue a move (`ToolHead.move`, `klippy/toolhead.py:395-408`).
    ///
    /// # Errors
    /// A zero-length move is ignored; the kinematics' own `check_move`
    /// (bounds, per-axis speed limits) is FW5e.
    pub fn move_to(&mut self, newpos: Coord, speed: f64) -> Result<(), CommandError> {
        let mut move_ = Move::new(self.commanded_pos, newpos, speed, &self.limits);
        if move_.move_d == 0.0 {
            return Ok(());
        }
        if move_.is_kinematic_move {
            if let Some(kinematics) = &self.kinematics {
                let mut ctx = MoveContext::new(&mut move_);
                kinematics.check_move(&mut ctx)?;
            }
        }
        for (index, axis) in self.extra_axes.iter().enumerate() {
            let ea_index = index + E_AXIS;
            // A second and later extra axis would need a position slot past
            // the move's four; those slots are not modelled yet (the guarded
            // downgrade for multi-extruder configs), so skip rather than
            // index out of bounds.
            if ea_index >= move_.axes_d.len() {
                continue;
            }
            if move_.axes_d[ea_index] != 0.0 {
                let mut ctx = MoveContext::new(&mut move_);
                axis.check_move(&mut ctx, ea_index)?;
            }
        }
        self.commanded_pos = move_.end_pos;
        let want_flush = self.lookahead.add_move(move_, &self.extra_axes);
        if want_flush {
            self.process_lookahead();
        }
        Ok(())
    }

    /// Flush the look-ahead into the trapq and advance the print time
    /// (`ToolHead._process_lookahead`, `klippy/toolhead.py:269-298`).
    fn process_lookahead(&mut self) {
        let moves = self.lookahead.flush(false);
        if moves.is_empty() {
            return;
        }
        if self.special_queuing_state {
            // Leaving "NeedPrime": start the print time a buffer ahead of the
            // MCU so the queue is never empty when motion starts.
            self.special_queuing_state = false;
            let min_print_time = self.estimated_print_time + BUFFER_TIME_START;
            if min_print_time > self.print_time {
                self.print_time = min_print_time;
            }
        }
        let mut next_move_time = self.print_time;
        for mut move_ in moves {
            if move_.is_kinematic_move {
                append_move(
                    self.motion_queuing.trapq_mut(self.main_trapq),
                    next_move_time,
                    &move_,
                );
            }
            // The extra axes queue their own trapezoid, on their own trapq
            // (`klippy/toolhead.py:288-291`).
            for (index, axis) in self.extra_axes.iter().enumerate() {
                let ea_index = index + E_AXIS;
                // Slots past the move's four are skipped, as in `move_to`.
                if ea_index >= move_.axes_d.len() {
                    continue;
                }
                if move_.axes_d[ea_index] != 0.0 {
                    axis.process_move(&mut self.motion_queuing, next_move_time, &move_, ea_index);
                }
            }
            next_move_time += move_.accel_t + move_.cruise_t + move_.decel_t;
            for callback in move_.timing_callbacks.drain(..) {
                callback(next_move_time);
            }
        }
        self.print_time = next_move_time;
    }

    /// Flush the look-ahead and generate steps up to `step_gen_time`.
    ///
    /// Returns one entry per stepper that produced commands.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from a stepper's compressor.
    pub fn flush_step_generation(
        &mut self,
        step_gen_time: f64,
    ) -> Result<Vec<(String, Vec<StepCommand>)>, StepCompressError> {
        self.process_lookahead();
        self.motion_queuing.generate(step_gen_time)
    }

    /// Plan everything queued so far (`ToolHead.wait_moves`,
    /// `klippy/toolhead.py:422-428`): upstream then waits for the MCU to catch
    /// up, which needs the clock estimate and is FW5d's MCU side.
    pub fn wait_moves(&mut self) {
        self.process_lookahead();
    }

    /// The print time the planner has reached (`ToolHead.get_last_move_time`):
    /// flush the look-ahead into the trapq first, then report. Updates based on
    /// this value are what the MCU is executing.
    pub fn get_last_move_time(&mut self) -> f64 {
        self.process_lookahead();
        self.print_time
    }

    /// Drop every queued move table entry (`motion_quuing.wipe_trapq`), as a
    /// homing move does once it has stopped.
    pub fn wipe_trapq(&mut self) {
        self.motion_queuing
            .trapq_mut(self.main_trapq)
            .finalize_moves(f64::MAX, 0.0);
    }

    /// Load one move straight into the trapq, bypassing the look-ahead
    /// (`ToolHead.drip_move` / `_drip_load_trapq`, `klippy/toolhead.py:459-493`).
    ///
    /// A homing move must not be joined to a previous move — it has to stop on
    /// its own endstop — so this sets its junction speeds to zero (start and end
    /// at rest), appends its trapezoid at the current print time, and advances
    /// the print time. The caller runs the step generation while the firmware
    /// moves (the drip loop), so only a small window is ever queued.
    ///
    /// Returns the move's start and end print times.
    ///
    /// # Errors
    /// The kinematics' `check_move` refusal.
    pub fn drip_move(&mut self, newpos: Coord, speed: f64) -> Result<(f64, f64), CommandError> {
        let mut move_ = Move::new(self.commanded_pos, newpos, speed, &self.limits);
        if move_.move_d == 0.0 {
            self.process_lookahead();
            return Ok((self.print_time, self.print_time));
        }
        if move_.is_kinematic_move {
            if let Some(kinematics) = &self.kinematics {
                let mut ctx = MoveContext::new(&mut move_);
                kinematics.check_move(&mut ctx)?;
            }
        }
        self.process_lookahead();
        move_.set_junction(0.0, move_.max_cruise_v2, 0.0);
        let start_time = self.print_time;
        append_move(
            self.motion_queuing.trapq_mut(self.main_trapq),
            start_time,
            &move_,
        );
        self.print_time = start_time + move_.accel_t + move_.cruise_t + move_.decel_t;
        self.commanded_pos = move_.end_pos;
        Ok((start_time, self.print_time))
    }

    /// Wait `delay` seconds without moving (`ToolHead.dwell`,
    /// `klippy/toolhead.py:417-420`).
    pub fn dwell(&mut self, delay: f64) {
        self.process_lookahead();
        self.print_time += delay.max(0.0);
    }

    /// Append a move's trapezoid directly, for tests and `drip_move`.
    pub fn append_move(&mut self, print_time: f64, move_: &Move) {
        append_move(
            self.motion_queuing.trapq_mut(self.main_trapq),
            print_time,
            move_,
        );
    }

    /// Force the toolhead to `newpos`, marking `homing_axes` as homed
    /// (`ToolHead.set_position`, `klippy/toolhead.py:383-391`).
    ///
    /// This is `G92`'s low-level half and `SET_KINEMATIC_POSITION`: the print
    /// time does not move and no steps are generated, but the solver and the
    /// kinematics are told where the toolhead is so the next move starts here.
    /// The trapq's current position is rewritten at the same time, which drops
    /// or truncates any history the old position recorded.
    pub fn set_position(&mut self, newpos: Coord, homing_axes: &[usize]) {
        self.process_lookahead();
        self.motion_queuing
            .trapq_mut(self.main_trapq)
            .set_position(self.print_time, Xyz::from(newpos));
        self.commanded_pos = newpos;
        if let Some(kinematics) = &mut self.kinematics {
            kinematics.set_position(newpos, homing_axes);
        }
        for stepper in self.motion_queuing.steppers_mut() {
            stepper.set_position(newpos.into());
        }
    }

    /// Drop finished moves from the trapq once the solvers are past them.
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        self.motion_queuing
            .finalize_moves(print_time, clear_history_time);
    }
}

/// Append one planned move's trapezoid to `trapq`
/// (`trapq_append` via `ToolHead._process_lookahead`).
fn append_move(trapq: &mut Trapq, print_time: f64, move_: &Move) {
    trapq.append(
        print_time,
        move_.accel_t,
        move_.cruise_t,
        move_.decel_t,
        Xyz::new(
            move_.start_pos.x(),
            move_.start_pos.y(),
            move_.start_pos.z(),
        ),
        Xyz::new(move_.axes_r[0], move_.axes_r[1], move_.axes_r[2]),
        move_.start_v,
        move_.cruise_v,
        move_.accel,
    );
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::motion::itersolve::Axis;
    use crate::core::klippy::motion::stepcompress::StepCommand;
    use crate::core::klippy::motion::stepper::Stepper;
    use std::sync::Mutex;

    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    /// A toolhead with one X stepper, 1 mm per step.
    fn toolhead() -> ToolHead {
        let mut toolhead = ToolHead::new(limits());
        toolhead.add_stepper(Stepper::cartesian(
            "stepper_x",
            0,
            1.0,
            Axis::X,
            1_000_000.0,
        ));
        toolhead
    }

    /// The number of steps across all steppers (the commands are compressed).
    fn step_count(batches: &[(String, Vec<StepCommand>)]) -> u32 {
        batches
            .iter()
            .flat_map(|(_, commands)| commands)
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum()
    }

    /// A fake extra axis: records its calls and queues into its own trapq.
    #[derive(Debug)]
    struct FakeExtraAxis {
        name: String,
        trapq: usize,
        checked: Mutex<Vec<usize>>,
        junction_calls: Mutex<usize>,
        queued: Mutex<Vec<f64>>,
    }

    impl FakeExtraAxis {
        fn new(trapq: usize) -> Self {
            Self {
                name: "extruder".to_string(),
                trapq,
                checked: Mutex::new(Vec::new()),
                junction_calls: Mutex::new(0),
                queued: Mutex::new(Vec::new()),
            }
        }
    }

    impl ExtraAxis for FakeExtraAxis {
        fn name(&self) -> &str {
            &self.name
        }

        fn check_move(
            &self,
            _ctx: &mut MoveContext<'_>,
            ea_index: usize,
        ) -> Result<(), CommandError> {
            self.checked.lock().unwrap().push(ea_index);
            Ok(())
        }

        fn calc_junction(&self, _prev: &Move, _cur: &Move, _ea_index: usize) -> f64 {
            *self.junction_calls.lock().unwrap() += 1;
            1234.0
        }

        fn process_move(
            &self,
            queuing: &mut MotionQueuing,
            print_time: f64,
            move_: &Move,
            ea_index: usize,
        ) {
            self.queued
                .lock()
                .unwrap()
                .push(move_.end_pos.axis(ea_index));
            queuing.trapq_mut(self.trapq).append(
                print_time,
                move_.accel_t,
                move_.cruise_t,
                move_.decel_t,
                Xyz::new(move_.start_pos.axis(ea_index), 0.0, 0.0),
                Xyz::new(1.0, 0.0, 0.0),
                move_.start_v,
                move_.cruise_v,
                move_.accel,
            );
        }

        fn find_past_position(&self, _print_time: f64) -> f64 {
            0.0
        }

        fn get_status(&self) -> serde_json::Value {
            serde_json::json!({})
        }
    }

    #[test]
    fn test_an_extra_axis_is_checked_and_queued_in_its_own_trapq() {
        let mut toolhead = toolhead();
        let trapq = toolhead.allocate_trapq();
        let axis = Arc::new(FakeExtraAxis::new(trapq));
        toolhead.add_extra_axis(axis.clone());

        // A pure extrusion: the kinematic steppers stay still, the extra axis
        // is checked and queued.
        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 5.0), 10.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0).unwrap();

        assert_eq!(*axis.checked.lock().unwrap(), [3]);
        assert_eq!(*axis.queued.lock().unwrap(), [5.0]);
        // The trapezoid lands in the extra axis' trapq (three phases).
        assert!(!toolhead
            .motion_queuing_mut()
            .trapq(trapq)
            .moves()
            .is_empty());
        // The X stepper did not move, and the fake axis is not a `Stepper`.
        assert!(batches.is_empty());
    }

    #[test]
    fn test_an_extra_axis_limits_the_junction() {
        let mut toolhead = toolhead();
        let trapq = toolhead.allocate_trapq();
        let axis = Arc::new(FakeExtraAxis::new(trapq));
        toolhead.add_extra_axis(axis.clone());

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead
            .move_to(Coord::new(20.0, 0.0, 0.0, 1.0), 100.0)
            .unwrap();

        // The second move's junction folds in the extra axis' limit.
        assert_eq!(*axis.junction_calls.lock().unwrap(), 1);
        let last = toolhead.lookahead.last().unwrap();
        assert_eq!(last.max_start_v2, 1234.0);
    }

    #[test]
    fn test_a_move_reaches_the_trapq_and_generates_queue_steps() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0).unwrap();

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, "stepper_x");
        // 10 mm of travel at 1 mm per step.
        assert_eq!(step_count(&batches), 10);
        assert_eq!(toolhead.commanded_pos(), Coord::new(10.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn test_two_collinear_moves_keep_their_junction_speed() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead
            .move_to(Coord::new(20.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        let batches = toolhead.flush_step_generation(1.0).unwrap();

        // No stop between the moves: 20 mm at 1 mm per step.
        assert_eq!(step_count(&batches), 20);
    }

    #[test]
    fn test_dwell_advances_print_time() {
        let mut toolhead = toolhead();
        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();
        toolhead.flush_step_generation(1.0).unwrap();
        let before = toolhead.print_time();

        toolhead.dwell(0.5);

        assert!((toolhead.print_time() - (before + 0.5)).abs() < 1e-9);
    }

    #[test]
    fn test_drip_move_loads_the_trapq_directly() {
        let mut toolhead = toolhead();

        let (start, end) = toolhead
            .drip_move(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();

        assert!(end > start);
        assert_eq!(toolhead.commanded_pos().x(), 10.0);
        assert_eq!(toolhead.get_last_move_time(), end);
        // The move is in the trapq and generates its steps.
        let batches = toolhead.flush_step_generation(end).unwrap();
        assert_eq!(step_count(&batches), 10);
        // Wiping the trapq leaves nothing queued.
        toolhead.wipe_trapq();
        assert!(toolhead.trapq().moves().is_empty());
    }

    #[test]
    fn test_a_zero_length_drip_move_does_nothing() {
        let mut toolhead = toolhead();

        let (start, end) = toolhead.drip_move(Coord::default(), 100.0).unwrap();

        assert_eq!(start, end);
        assert!(toolhead.trapq().moves().is_empty());
    }

    #[test]
    fn test_an_unhomed_axis_refuses_a_move() {
        use crate::core::klippy::motion::kinematics::CartesianKinematics;

        let mut toolhead = toolhead();
        toolhead.set_kinematics(Box::new(CartesianKinematics::new(
            ["stepper_x".into(), "stepper_y".into(), "stepper_z".into()],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            crate::core::klippy::motion::kinematics::CartesianTransform::Standard,
        )));

        let err = toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap_err();

        assert!(err.to_string().contains("Must home axis first"), "{err}");
    }

    #[test]
    fn test_a_zero_length_move_is_ignored() {
        let mut toolhead = toolhead();

        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();

        assert!(toolhead.trapq().moves().is_empty());
    }
}
