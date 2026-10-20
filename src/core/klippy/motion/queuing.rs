//! The motion output queue.
//!
//! Upstream's `klippy/extras/motion_quuing.py`: it owns the trapq and the
//! steppers, and knows how to turn the queued moves into step commands. The
//! real one paces itself with a reactor timer and sends over the MCU; this one
//! is driven explicitly (`generate`) so the host logic can be tested without a
//! device, and the MCU side attaches on top of it.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use tracing::error;

use super::stepcompress::{StepCommand, StepCompressError};
use super::stepper::Stepper;
use super::trapq::{Trapq, NEVER_TIME};
use crate::core::klippy::mathutil::Xyz;
use crate::core::klippy::reactor::panic_message;

/// One callback fired when its stepper first has real motion queued.
///
/// Upstream's `MCU_stepper._active_callbacks` entry: the stepper's own list,
/// emptied into the callbacks the flush pass invokes.
type ActiveCallback = Box<dyn Fn(f64) + Send + 'static>;

/// The per-stepper "enable me when I really move" callbacks
/// (`MCU_stepper.add_active_callback`).
///
/// Upstream keys these on the stepper and installs `_check_active` on the
/// motion queue's flush callbacks when the first one arrives; the list lives
/// here because the flush pass is the only place with both the trapq and the
/// solver state the check reads. It is `Arc`-shared so `stepper_enable` can arm
/// a stepper from outside the toolhead — at config load, when no planner exists
/// yet, and again after every `M84` (`EnableTracking.motor_disable` re-arms).
#[derive(Default)]
pub struct ActiveCallbacks {
    /// Waiting callbacks per stepper name, in registration order.
    entries: Mutex<HashMap<String, Vec<ActiveCallback>>>,
}

impl ActiveCallbacks {
    /// An empty set of callbacks.
    pub fn new() -> Self {
        Self::default()
    }

    /// Arm `stepper`: the next queued motion fires `callback` **once** with the
    /// print time that motion starts at (`MCU_stepper.add_active_callback`).
    pub fn add(&self, stepper: &str, callback: ActiveCallback) {
        self.entries
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .entry(stepper.to_string())
            .or_default()
            .push(callback);
    }

    /// The steppers with callbacks still waiting; a name drops out once it has
    /// fired.
    fn waiting(&self) -> Vec<String> {
        let mut entries = self
            .entries
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        entries.retain(|_, callbacks| !callbacks.is_empty());
        entries.keys().cloned().collect()
    }

    /// Take `stepper`'s waiting callbacks (`_check_active` clears the list it
    /// just fired). A name with none answers empty.
    fn take(&self, stepper: &str) -> Vec<ActiveCallback> {
        self.entries
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .remove(stepper)
            .unwrap_or_default()
    }
}

/// The trapqs and the steppers reading them.
///
/// Upstream's `MotionQueuing` owns a list of trapqs (`allocate_trapq`,
/// `motion_queuing.py:63-67`): the toolhead has one, each extruder its own. A
/// stepper is bound to one of them (`MCU_stepper.set_trapq`), and only reads
/// that one when generating steps.
#[derive(Default)]
pub struct MotionQueuing {
    trapqs: Vec<Trapq>,
    steppers: Vec<Stepper>,
    /// The flush callbacks (`register_flush_callback`), invoked with the
    /// flush time at the start of every [`MotionQueuing::generate`]
    /// (`motion_queuing.py:147-150`).
    flush_callbacks: Vec<Box<dyn Fn(f64) + Send>>,
    /// The per-stepper active callbacks, checked before every
    /// [`MotionQueuing::generate`] (`MCU_stepper.add_active_callback`).
    active_callbacks: Arc<ActiveCallbacks>,
}

impl std::fmt::Debug for MotionQueuing {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotionQueuing")
            .field("trapqs", &self.trapqs)
            .field("steppers", &self.steppers)
            .finish_non_exhaustive()
    }
}

impl MotionQueuing {
    /// An empty queue.
    ///
    /// The queue holds no clock of its own: each [`Stepper`] carries its MCU's
    /// frequency and print-time offset, so a stepper on a secondary MCU flushes
    /// on that MCU's clock.
    pub fn new() -> Self {
        Self {
            trapqs: Vec::new(),
            steppers: Vec::new(),
            flush_callbacks: Vec::new(),
            active_callbacks: Arc::new(ActiveCallbacks::new()),
        }
    }

    /// Register a flush callback (`MotionQueuing.register_flush_callback`):
    /// it is called with the flush time at the start of every
    /// [`MotionQueuing::generate`], in registration order, and stays
    /// registered for the later ones. Upstream's `_advance_flush_time` calls
    /// each with `flush_time` and `step_gen_time` (`motion_queuing.py:147-150`);
    /// only the flush time crosses this seam.
    pub fn register_flush_callback(&mut self, callback: Box<dyn Fn(f64) + Send + 'static>) {
        self.flush_callbacks.push(callback);
    }

    /// The per-stepper active callbacks (`MCU_stepper.add_active_callback`).
    pub fn active_callbacks(&self) -> Arc<ActiveCallbacks> {
        Arc::clone(&self.active_callbacks)
    }

    /// Check `active_callbacks` instead of this queue's own, empty set.
    ///
    /// The toolhead installs `stepper_enable`'s set here at connect: that object
    /// is armed at config load, long before this queue exists
    /// (`klippy/extras/toolhead.rs`).
    pub fn set_active_callbacks(&mut self, callbacks: Arc<ActiveCallbacks>) {
        self.active_callbacks = callbacks;
    }

    /// Create a trapq and return its id (`MotionQueuing.allocate_trapq`).
    pub fn allocate_trapq(&mut self) -> usize {
        self.trapqs.push(Trapq::new());
        self.trapqs.len() - 1
    }

    /// A trapq by id.
    pub fn trapq(&self, id: usize) -> &Trapq {
        &self.trapqs[id]
    }

    /// A trapq by id, to append to or move the current position in it.
    pub fn trapq_mut(&mut self, id: usize) -> &mut Trapq {
        &mut self.trapqs[id]
    }

    /// Append one move's trapezoid to the trapq `handle`
    /// (`MotionQueuing.lookup_trapq_append` → `trapq_append`).
    ///
    /// A handle-based form of [`Trapq::append`], so a caller that owns a trapq
    /// id (the force-move queue) appends without holding a `&mut Trapq` across
    /// the toolhead borrow it also needs.
    #[allow(clippy::too_many_arguments)]
    pub fn append(
        &mut self,
        trapq: usize,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos: Xyz,
        axes_r: Xyz,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    ) {
        self.trapqs[trapq].append(
            print_time, accel_t, cruise_t, decel_t, start_pos, axes_r, start_v, cruise_v, accel,
        );
    }

    /// Expire every queued move of the trapq `handle` into its history
    /// (`MotionQueuing.wipe_trapq`, `motion_queuing.py:68-70`: force to the
    /// history list with no expiry).
    pub fn wipe_trapq(&mut self, trapq: usize) {
        self.trapqs[trapq].finalize_moves(NEVER_TIME, 0.0);
    }

    /// Drop finished segments from every live queue into its history
    /// (`trapq_finalize_moves`).
    ///
    /// `print_time` is how far the step solvers have generated: anything ending
    /// before it can never be read again. `clear_history_time` is how old a
    /// history entry may be before it is dropped.
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        for trapq in &mut self.trapqs {
            trapq.finalize_moves(print_time, clear_history_time);
        }
    }

    /// Add a stepper to generate for. Its trapq id defaults to 0 (the main
    /// trapq); use [`Stepper::set_trapq`] before adding when it differs.
    pub fn add_stepper(&mut self, stepper: Stepper) {
        self.steppers.push(stepper);
    }

    /// The steppers.
    pub fn steppers(&self) -> &[Stepper] {
        &self.steppers
    }

    /// The steppers, to set positions on.
    pub fn steppers_mut(&mut self) -> &mut [Stepper] {
        &mut self.steppers
    }

    /// Generate steps for every stepper up to `flush_time`.
    ///
    /// Each stepper reads its own trapq, so the toolhead's steppers and an
    /// extruder's generate from different queues. A detached stepper
    /// (`Stepper::set_trapq(None)`) is skipped: no steps are generated for
    /// it until it is attached again (`MCU_stepper.set_trapq`).
    ///
    /// Returns one entry per stepper that produced commands, in the order the
    /// steppers were added.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from a stepper's compressor.
    pub fn generate(
        &mut self,
        flush_time: f64,
    ) -> Result<Vec<(String, Vec<StepCommand>)>, StepCompressError> {
        let Self {
            trapqs,
            steppers,
            flush_callbacks,
            active_callbacks,
        } = self;
        // Active checks first (`_advance_flush_time` runs the flush callbacks —
        // which is where upstream installs `_check_active` — before generating):
        // a stepper with real motion queued has its enable scheduled at that
        // motion's print time, before this pass generates its steps.
        let waiting = active_callbacks.waiting();
        if !waiting.is_empty() {
            let mut fired: Vec<(String, Vec<ActiveCallback>, f64)> = Vec::new();
            for name in waiting {
                let Some(stepper) = steppers.iter().find(|stepper| stepper.name() == name) else {
                    continue;
                };
                // A detached stepper generates nothing, so nothing can move.
                let Some(trapq_id) = stepper.trapq_id() else {
                    continue;
                };
                if let Some(print_time) = stepper
                    .kinematics()
                    .check_active(&trapqs[trapq_id], flush_time)
                {
                    fired.push((name.clone(), active_callbacks.take(&name), print_time));
                }
            }
            // Outside the queue borrows: a callback drives an enable pin and may
            // defer work, but must not need the queue.
            for (stepper, callbacks, print_time) in fired {
                for callback in callbacks {
                    run_callback(
                        &format!("active callback on {stepper}"),
                        &*callback,
                        print_time,
                    );
                }
            }
        }
        // Flush callbacks run before generating, and they run even when
        // there is no stepper to generate for: a macro-driven dwell timeline
        // queues no steps, but its flush time still advances.
        for (index, callback) in flush_callbacks.iter().enumerate() {
            run_callback(&format!("flush callback #{index}"), &**callback, flush_time);
        }
        let mut out = Vec::new();
        for stepper in steppers.iter_mut() {
            // Detached (`set_trapq(None)`): generate nothing for it.
            let Some(trapq_id) = stepper.trapq_id() else {
                continue;
            };
            let trapq = &trapqs[trapq_id];
            let commands = stepper.generate(trapq, flush_time)?;
            if !commands.is_empty() {
                out.push((stepper.name().to_string(), commands));
            }
        }
        Ok(out)
    }

    /// Generate steps for one stepper only, up to `flush_time`.
    ///
    /// The force-move queue uses this: it moves a motor on its own trapq and
    /// must generate **only that motor**, to the move's end rather than the
    /// background horizon ([`Self::generate`]). Generating every stepper to a
    /// content end ahead of the horizon would advance their solvers past it,
    /// and the next horizon-bounded pass would rewind them onto moves it had
    /// already generated.
    ///
    /// Returns the commands to send; an unknown name, or a detached stepper,
    /// produces none.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from the stepper's compressor.
    pub fn generate_stepper(
        &mut self,
        name: &str,
        flush_time: f64,
    ) -> Result<Vec<StepCommand>, StepCompressError> {
        let Self {
            trapqs, steppers, ..
        } = self;
        for stepper in steppers.iter_mut() {
            if stepper.name() != name {
                continue;
            }
            let Some(trapq_id) = stepper.trapq_id() else {
                return Ok(Vec::new());
            };
            return stepper.generate(&trapqs[trapq_id], flush_time);
        }
        Ok(Vec::new())
    }
}

/// Run one callback the flush pass registered, catching a panic in it.
///
/// The pass is driven from the toolhead's flush timer, so it runs on the
/// reactor's single dispatcher: a panic left to escape would end that round's
/// dispatch and, before [`crate::core::klippy::reactor`] isolated timer
/// callbacks, the dispatcher itself. The panic is logged with where the callback
/// came from (`source`), and the rest of the pass still runs — the flush time
/// has advanced either way, so dropping the remaining callbacks would leave the
/// queue behind the MCU's clock.
fn run_callback(source: &str, callback: &(dyn Fn(f64) + Send), eventtime: f64) {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        callback(eventtime);
    }));
    if let Err(payload) = result {
        error!(
            callback = source,
            "callback panicked: {}",
            panic_message(&*payload)
        );
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::{Coord, Xyz};
    use crate::core::klippy::motion::itersolve::Axis;
    use crate::core::klippy::motion::plan::{Move, MoveLimits};
    use std::sync::{Arc, Mutex};
    use tracing_subscriber::layer::SubscriberExt;

    fn limits() -> MoveLimits {
        MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        }
    }

    /// A 10 mm move along X at 100 mm/s, already planned.
    fn move_(start: f64, end: f64) -> Move {
        Move::new(
            Coord::new(start, 0.0, 0.0, 0.0),
            Coord::new(end, 0.0, 0.0, 0.0),
            100.0,
            &limits(),
        )
    }

    /// Append a move's kinematic (xyz) trapezoid, as the toolhead does.
    fn append(queuing: &mut MotionQueuing, trapq: usize, move_: &Move) {
        queuing.trapq_mut(trapq).append(
            0.0,
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

    #[test]
    fn test_append_and_generate_use_the_same_trapq() {
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        queuing.add_stepper(Stepper::cartesian(
            "stepper_x",
            0,
            1.0,
            Axis::X,
            1_000_000.0,
        ));
        let mut move_ = move_(0.0, 10.0);
        // Give it a profile: accelerate and decelerate over 10 mm.
        move_.set_junction(0.0, 10_000.0, 0.0);
        append(&mut queuing, trapq, &move_);

        let batches = queuing.generate(0.2).unwrap();

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, "stepper_x");
        let steps: u32 = batches[0]
            .1
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum();
        assert_eq!(steps, 10);
    }

    #[test]
    fn test_a_detached_stepper_generates_no_steps_until_reattached() {
        // `ZAdjustHelper.adjust_steppers` takes every Z motor off the trapq,
        // moves with them detached, and attaches them one by one; a detached
        // stepper must produce no commands until it is attached again.
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        queuing.add_stepper(Stepper::cartesian(
            "stepper_z",
            0,
            1.0,
            Axis::Z,
            1_000_000.0,
        ));
        let mut move_ = move_(0.0, 10.0);
        move_.set_junction(0.0, 10_000.0, 0.0);
        // The same 10 mm, queued along Z (the `append` helper is X-only).
        queuing.trapq_mut(trapq).append(
            0.0,
            move_.accel_t,
            move_.cruise_t,
            move_.decel_t,
            Xyz::new(0.0, 0.0, move_.start_pos.x()),
            Xyz::new(0.0, 0.0, 1.0),
            move_.start_v,
            move_.cruise_v,
            move_.accel,
        );
        // Detach (`set_trapq(None)`): the queued move generates nothing.
        queuing.steppers_mut()[0].set_trapq(None);
        assert_eq!(queuing.steppers()[0].trapq_id(), None);
        assert!(queuing.generate(0.2).unwrap().is_empty());

        // Reattach: the same move now generates its steps.
        queuing.steppers_mut()[0].set_trapq(trapq);
        let batches = queuing.generate(0.2).unwrap();

        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, "stepper_z");
        let steps: u32 = batches[0]
            .1
            .iter()
            .filter_map(|command| match command {
                StepCommand::QueueStep { count, .. } => Some(*count),
                StepCommand::SetNextStepDir { .. } => None,
            })
            .sum();
        assert_eq!(steps, 10);
    }

    #[test]
    fn test_a_stepper_with_nothing_to_do_is_silent() {
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        queuing.add_stepper(Stepper::cartesian(
            "stepper_y",
            1,
            1.0,
            Axis::Y,
            1_000_000.0,
        ));
        let mut move_ = move_(0.0, 10.0); // X only
        move_.set_junction(0.0, 10_000.0, 0.0);
        append(&mut queuing, trapq, &move_);

        // The Y stepper produces no commands, so only the trapq had work.
        assert!(queuing.generate(0.2).unwrap().is_empty());
    }

    #[test]
    fn test_flush_callbacks_run_in_registration_order_even_without_steppers() {
        // A dwell-only timeline — no stepper reads the trapq, so `generate`
        // produces nothing — still advances the flush time, and the flush
        // callbacks registered on it still run (`motion_queuing.py:147-150`).
        let mut queuing = MotionQueuing::new();
        queuing.allocate_trapq();
        let calls = Arc::new(Mutex::new(Vec::new()));
        for name in ["first", "second"] {
            let calls = Arc::clone(&calls);
            queuing.register_flush_callback(Box::new(move |flush_time| {
                calls.lock().unwrap().push((name, flush_time));
            }));
        }

        assert!(queuing.generate(1.5).unwrap().is_empty());
        assert_eq!(
            *calls.lock().unwrap(),
            [("first", 1.5), ("second", 1.5)],
            "in registration order, each with the flush time"
        );

        // The callbacks stay registered: the next generate runs them again.
        queuing.generate(2.0).unwrap();
        assert_eq!(
            *calls.lock().unwrap(),
            [
                ("first", 1.5),
                ("second", 1.5),
                ("first", 2.0),
                ("second", 2.0)
            ]
        );
    }

    #[test]
    fn test_a_panicking_flush_callback_does_not_stop_the_flush_pass() {
        // The pass runs on the reactor's dispatcher (through the toolhead's flush
        // timer): a panic in one registered callback must neither end the pass
        // nor hide the panic — the callbacks behind it still run, the step
        // generation still happens, and the log names the source.
        let capture = LogCapture::default();
        let subscriber = tracing_subscriber::registry()
            .with(tracing_subscriber::filter::LevelFilter::ERROR)
            .with(
                tracing_subscriber::fmt::layer()
                    .with_writer(capture.clone())
                    .with_ansi(false),
            );
        let calls = Arc::new(Mutex::new(Vec::new()));
        tracing::subscriber::with_default(subscriber, || {
            let mut queuing = MotionQueuing::new();
            let trapq = queuing.allocate_trapq();
            queuing.add_stepper(Stepper::cartesian(
                "stepper_x",
                0,
                1.0,
                Axis::X,
                1_000_000.0,
            ));
            let mut move_ = move_(0.0, 10.0);
            move_.set_junction(0.0, 10_000.0, 0.0);
            append(&mut queuing, trapq, &move_);

            queuing.register_flush_callback(Box::new(|_| panic!("flush exploded")));
            let recorded = Arc::clone(&calls);
            queuing.register_flush_callback(Box::new(move |flush_time| {
                recorded.lock().unwrap().push(flush_time);
            }));

            let commands = queuing
                .generate(0.2)
                .expect("the pass still generates after a panic");
            assert!(
                !commands.is_empty(),
                "the stepper still generated its steps"
            );
            assert_eq!(
                *calls.lock().unwrap(),
                [0.2],
                "the callback behind the panicking one still ran"
            );

            // The registration survives: the next pass runs both again.
            queuing.generate(0.4).unwrap();
            assert_eq!(*calls.lock().unwrap(), [0.2, 0.4]);
        });

        let log = capture.text();
        assert!(
            log.contains("flush callback #0"),
            "the log must name the callback that panicked: {log}"
        );
        assert!(
            log.contains("flush exploded"),
            "the log must carry the panic message: {log}"
        );
    }

    /// A byte sink for the fmt layer above. `with_default` keeps the capture off
    /// the process-wide subscriber, the same shape `serial_ports.rs` uses.
    #[derive(Clone, Default)]
    struct LogCapture(Arc<Mutex<Vec<u8>>>);

    impl LogCapture {
        fn text(&self) -> String {
            let bytes = self
                .0
                .lock()
                .expect("the capture lock is not poisoned")
                .clone();
            String::from_utf8(bytes).expect("tracing writes UTF-8")
        }
    }

    impl std::io::Write for LogCapture {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0
                .lock()
                .expect("the capture lock is not poisoned")
                .extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for LogCapture {
        type Writer = LogCapture;

        fn make_writer(&'a self) -> Self::Writer {
            self.clone()
        }
    }

    #[test]
    fn test_a_queued_move_fires_the_active_callback_once_at_its_print_time() {
        // `MCU_stepper.add_active_callback` + `_check_active`: the callback runs
        // once, with the print time the stepper's first real move starts at, and
        // its registration is consumed — a later flush does not fire it again.
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        queuing.add_stepper(Stepper::cartesian(
            "stepper_x",
            0,
            1.0,
            Axis::X,
            1_000_000.0,
        ));
        let fired = Arc::new(Mutex::new(Vec::new()));
        let recorded = Arc::clone(&fired);
        queuing.active_callbacks().add(
            "stepper_x",
            Box::new(move |print_time| recorded.lock().unwrap().push(print_time)),
        );
        // A dwell first (a `G4`): it moves no axis, so the move after it is what
        // the callback has to report.
        let mut move_ = move_(0.0, 10.0);
        move_.set_junction(0.0, 10_000.0, 0.0);
        queuing.trapq_mut(trapq).append(
            0.0,
            0.1,
            0.0,
            0.0,
            Xyz::default(),
            Xyz::default(),
            0.0,
            0.0,
            0.0,
        );
        queuing.trapq_mut(trapq).append(
            0.1,
            move_.accel_t,
            move_.cruise_t,
            move_.decel_t,
            Xyz::default(),
            Xyz::new(10.0, 0.0, 0.0),
            move_.start_v,
            move_.cruise_v,
            move_.accel,
        );

        queuing.generate(0.5).unwrap();

        assert_eq!(
            *fired.lock().unwrap(),
            [0.1],
            "once, at the print time the move starts at"
        );

        // Already fired: the registration is gone, so a later flush is silent.
        queuing.generate(0.8).unwrap();
        assert_eq!(*fired.lock().unwrap(), [0.1]);
    }

    #[test]
    fn test_a_flush_with_no_motion_never_fires_the_active_callback() {
        // Only a dwell is queued (a `G4`): nothing that moves an axis is queued,
        // so the motor stays off and the callback keeps waiting for the next
        // flush.
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        queuing.add_stepper(Stepper::cartesian(
            "stepper_y",
            0,
            1.0,
            Axis::Y,
            1_000_000.0,
        ));
        let fired = Arc::new(Mutex::new(Vec::new()));
        let recorded = Arc::clone(&fired);
        queuing.active_callbacks().add(
            "stepper_y",
            Box::new(move |print_time| recorded.lock().unwrap().push(print_time)),
        );
        queuing.trapq_mut(trapq).append(
            0.0,
            0.0,
            1.1,
            0.0,
            Xyz::default(),
            Xyz::default(),
            0.0,
            0.0,
            0.0,
        );

        queuing.generate(1.05).unwrap();
        assert!(fired.lock().unwrap().is_empty(), "a dwell moves no axis");

        // The waiting callback is still armed for the real motion that follows.
        let mut move_ = move_(0.0, 10.0);
        move_.set_junction(0.0, 10_000.0, 0.0);
        queuing.trapq_mut(trapq).append(
            1.1,
            move_.accel_t,
            move_.cruise_t,
            move_.decel_t,
            Xyz::default(),
            Xyz::new(0.0, 10.0, 0.0),
            move_.start_v,
            move_.cruise_v,
            move_.accel,
        );

        queuing.generate(2.0).unwrap();
        assert_eq!(*fired.lock().unwrap(), [1.1]);
    }

    #[test]
    fn test_two_steppers_on_one_mcu_both_generate() {
        // The common case: one MCU drives several motors. The steppers share the
        // MCU's clock/offset but have independent compressors; a diagonal move
        // must produce commands for both.
        let mut queuing = MotionQueuing::new();
        let trapq = queuing.allocate_trapq();
        let mut x = Stepper::cartesian("stepper_x", 0, 1.0, Axis::X, 1_000_000.0);
        x.compressor_mut().set_time(0.0, 1_000_000.0);
        let mut y = Stepper::cartesian("stepper_y", 1, 1.0, Axis::Y, 1_000_000.0);
        y.compressor_mut().set_time(0.0, 1_000_000.0);
        queuing.add_stepper(x);
        queuing.add_stepper(y);
        let mut move_ = Move::new(
            Coord::default(),
            Coord::new(10.0, 10.0, 0.0, 0.0),
            100.0,
            &limits(),
        );
        move_.set_junction(0.0, 10_000.0, 0.0);
        append(&mut queuing, trapq, &move_);

        let batches = queuing.generate(0.2).unwrap();

        let names: Vec<&str> = batches.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(names, ["stepper_x", "stepper_y"]);
        let counts: Vec<u32> = batches
            .iter()
            .map(|(_, commands)| {
                commands
                    .iter()
                    .filter_map(|command| match command {
                        StepCommand::QueueStep { count, .. } => Some(*count),
                        StepCommand::SetNextStepDir { .. } => None,
                    })
                    .sum()
            })
            .collect();
        // The diagonal is symmetric, so both axes take the same number of
        // steps on the shared clock.
        assert_eq!(counts[0], counts[1]);
        assert!(counts[0] >= 9, "{counts:?}");
    }
}
