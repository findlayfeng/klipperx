//! `motion_report` — diagnostic streams of the motion queues (upstream's
//! `klippy/extras/motion_report.py`).
//!
//! Two `name`-muxed `*/dump_*` endpoints, each with the batch machinery of
//! [`bulk_sensor`](super::bulk_sensor):
//!
//! | instance | path | header |
//! |---|---|---|
//! | every MCU stepper | [`DUMP_STEPPER_PATH`] | `interval`, `count`, `add` |
//! | `toolhead`, each extruder | [`DUMP_TRAPQ_PATH`] | `time`, `duration`, `start_velocity`, `acceleration`, `start_position`, `direction` |
//!
//! Upstream builds one `DumpStepper` per `MCU_stepper` and one `DumpTrapQ` per
//! queue, registering the mux instances in `klippy:connect` once the trapqs
//! exist (`motion_report.py:165-186`). Here the mux table is filled from the
//! load path, so each instance is registered **by name** where its owner is
//! loaded and the trapq id is resolved when a batch runs.
//!
//! The data is read, never produced: [`Trapq::extract_old`] and the step
//! compressor's history are the two records this streams.
//!
//! # Status and shutdown diagnostics
//!
//! [`MotionReport`] answers `get_status` with upstream's five keys
//! (`motion_report.py:232-263`): `live_position` (three kinematics axes plus
//! one slot per extra axis — the extruder sits in the fourth), `live_velocity`,
//! `live_extruder_velocity`, and the two registered-name lists `steppers` and
//! `trapq`. The position comes from the queues themselves at the MCU's
//! estimated print time, not from the planner's commanded position, exactly as
//! upstream reads it (`DumpTrapQ.get_trapq_position`).
//!
//! On `klippy:analyze_shutdown` it dumps the queue windows around the stop into
//! the log (`motion_report.py:188-231`): the stopping MCU's steppers, every
//! registered trapq, and the requested toolhead position at that instant.
//!
//! # What is not ported yet
//!
//! * The `manual_stepper` trapq instances: this port gives `manual_stepper` no
//!   trapq instance here (its queue is allocated on first motion), so there is
//!   no queue to stream and no slot for it in `live_position` — upstream fills
//!   that slot from its instance.
//! * The `"Internal error in stepcompress"` shutdown dump
//!   (`motion_report.py:225-231`): this port reports that error as its own
//!   message with no `step_gen_time` detail, so there is no window to dump.
//! * The drip-timing guard (`motion_queuing.check_drip_timing`,
//!   `motion_report.py:125-132`), which withholds trapq entries that may still
//!   change while homing. This host does not expose the drip's end time, so a
//!   homing move's tail may appear in one batch and be corrected by the next.
//!
//! [`Trapq::extract_old`]: crate::core::klippy::motion::trapq::Trapq::extract_old

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::info;

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, LifecycleCb, BATCH_INTERVAL,
};
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::mathutil::E_AXIS;
use crate::core::klippy::mcu::McuObject;
use crate::core::klippy::motion::stepcompress::HistoryStep;
use crate::core::klippy::motion::trapq::MoveSegment;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The printer object the module registers (a library object like upstream's
/// `motion_queuing`: `stepper.py:282-285` loads it per MCU stepper, there is no
/// `[motion_report]` section).
pub const MOTION_REPORT_OBJECT: &str = "motion_report";

/// The stepper history dump (`motion_report.py:12-68`).
pub const DUMP_STEPPER_PATH: &str = "motion_report/dump_stepper";

/// The trapezoidal-queue dump (`motion_report.py:73-141`).
pub const DUMP_TRAPQ_PATH: &str = "motion_report/dump_trapq";

/// The mux key that selects an instance (both endpoints, upstream `key="name"`).
pub const DUMP_KEY: &str = "name";

/// The instance name of the toolhead's main queue (`motion_report.py:171`).
pub const TOOLHEAD_TRAPQ_NAME: &str = "toolhead";

/// The printer object the toolhead registers itself as.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The `dump_stepper` column header, verbatim upstream (`motion_report.py:19-21`).
pub const STEPPER_HEADER: [&str; 3] = ["interval", "count", "add"];

/// The `dump_trapq` column header, verbatim upstream (`motion_report.py:79-85`).
pub const TRAPQ_HEADER: [&str; 6] = [
    "time",
    "duration",
    "start_velocity",
    "acceleration",
    "start_position",
    "direction",
];

/// A time past any real move, so extraction never stops at the queue head
/// (`motion_report.NEVER_TIME`).
const NEVER_TIME: f64 = 9999999999999999.;

/// How far before the last sent move the next trapq batch re-reads
/// (`min(duration, 0.100)`, `motion_report.py:126`): the previous batch may have
/// been read while that move was still being extended.
const RESEND_WINDOW: f64 = 0.100;

/// The clock past any scheduled step (`1 << 63`, `motion_report.py:48`).
const NEVER_CLOCK: u64 = 1 << 63;

/// How long one `get_status` answer stands before it is computed again
/// (`STATUS_REFRESH_TIME`, `motion_report.py:143`).
const STATUS_REFRESH_TIME: f64 = 0.250;

/// How far either side of the shutdown time the diagnostics dump
/// (`_dump_shutdown`'s `duration=0.100`, `motion_report.py:194`).
const SHUTDOWN_DUMP_DURATION: f64 = 0.100;

/// One `dump_trapq` row, in header order: `(time, duration, start_velocity,
/// acceleration, start_position, direction)` (`motion_report.py:130-133`).
type TrapqRow = (f64, f64, f64, f64, [f64; 3], [f64; 3]);

/// One `dump_stepper` row, in header order: `(interval, count, add)`
/// (`motion_report.py:61`).
type StepperRow = (u32, i64, i32);

// ===========================================================================
// Pure batch builders (unit-tested directly)
// ===========================================================================

/// One trapq segment as an API row.
fn trapq_row(segment: &MoveSegment) -> TrapqRow {
    (
        segment.print_time,
        segment.move_t,
        segment.start_v,
        // The queue keeps half the acceleration; the report publishes it whole
        // (`motion_report.py:131`).
        2.0 * segment.half_accel,
        [
            segment.start_pos.x(),
            segment.start_pos.y(),
            segment.start_pos.z(),
        ],
        [segment.axes_r.x(), segment.axes_r.y(), segment.axes_r.z()],
    )
}

/// Where the next trapq batch starts reading (`motion_report.py:126`); `0.0`
/// before the first batch, like upstream's `(0., 0.)` placeholder.
fn trapq_resend_time(last_batch_msg: Option<&TrapqRow>) -> f64 {
    match last_batch_msg {
        Some(row) => row.0 + row.1.min(RESEND_WINDOW),
        None => 0.0,
    }
}

/// One `dump_trapq` batch from the segments overlapping the read window,
/// newest first (upstream's `DumpTrapQ._process_batch`, minus the drip guard).
///
/// Upstream reverses the ffi chunks into oldest-first rows
/// (`motion_report.py:98-100`), drops the row the previous batch already sent —
/// the window re-reads it on purpose — and records the newest sent row as the
/// next `last_batch_msg`. `None` when nothing is new.
fn trapq_batch(last_batch_msg: &mut Option<TrapqRow>, moves: &[MoveSegment]) -> Option<Value> {
    let mut rows: Vec<TrapqRow> = moves.iter().rev().map(trapq_row).collect();
    if rows
        .first()
        .is_some_and(|first| Some(*first) == *last_batch_msg)
    {
        rows.remove(0);
    }
    let newest = *rows.last()?;
    *last_batch_msg = Some(newest);
    Some(json!({ "data": rows }))
}

/// One `dump_stepper` batch (upstream's `DumpStepper._process_batch`).
///
/// `steps` are the compressor's runs overlapping the read window, **newest
/// first** (as the history walk returns them); the batch runs oldest first and
/// stops after the first `step_count == 0` entry — a `set_position` marker,
/// whose counter break would make the rows after it meaningless as a
/// continuation (`motion_report.py:49-53`). `last_batch_clock` advances to the
/// last row kept.
///
/// `None` when nothing is new.
fn stepper_batch(
    last_batch_clock: &mut u64,
    steps: &[HistoryStep],
    step_dist: f64,
    clock_to_print_time: impl Fn(u64) -> f64,
) -> Option<Value> {
    let mut data: Vec<HistoryStep> = steps.iter().rev().copied().collect();
    if let Some(marker) = data.iter().position(|step| step.step_count == 0) {
        data.truncate(marker + 1);
    }
    let first = *data.first()?;
    let last = *data.last()?;
    *last_batch_clock = last.last_clock;
    let rows: Vec<StepperRow> = data
        .iter()
        .map(|step| (step.interval, step.step_count, step.add))
        .collect();
    Some(json!({
        "data": rows,
        // Upstream's `mcu_to_commanded_position`: the counter times the step
        // distance (`stepper.py:183-184`).
        "start_position": first.start_position as f64 * step_dist,
        "start_mcu_position": first.start_position,
        "step_distance": step_dist,
        "first_clock": first.first_clock,
        "first_step_time": clock_to_print_time(first.first_clock),
        "last_clock": last.last_clock,
        "last_step_time": clock_to_print_time(last.last_clock),
    }))
}

/// A trapq's toolhead position at one print time (upstream's
/// `DumpTrapQ.get_trapq_position`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct TrapqPosition {
    /// The position along the queue's own three axes, mm.
    pos: [f64; 3],
    /// The path velocity, mm/s.
    velocity: f64,
}

/// Where the queue is at `print_time` (`motion_report.py:107-121`).
///
/// `moves` is what the read window `0. .. print_time` returns with `max = 1`:
/// the newest segment that had already started. `None` when the queue holds
/// nothing that far back — upstream's `(None, None)`.
fn trapq_position(moves: &[MoveSegment], print_time: f64) -> Option<TrapqPosition> {
    let segment = moves.first()?;
    // A read past the segment's end holds its end (upstream clamps the same
    // way, `motion_report.py:113`).
    let move_time = (print_time - segment.print_time).clamp(0.0, segment.move_t);
    let coord = segment.coord(move_time);
    Some(TrapqPosition {
        pos: [coord.x(), coord.y(), coord.z()],
        // `start_v + accel * t`, with the queue's halved acceleration doubled
        // back (`motion_report.py:120`).
        velocity: segment.start_v + 2.0 * segment.half_accel * move_time,
    })
}

/// One `get_status` answer from the quantities already read
/// (`motion_report.py:232-263`).
///
/// `live_position` is three kinematics axes plus one slot per extra axis; the
/// caller builds it, so its length is the machine's axis count.
fn live_status(
    live_position: &[f64],
    live_velocity: f64,
    live_extruder_velocity: f64,
    steppers: &[String],
    trapqs: &[String],
) -> Value {
    json!({
        "live_position": live_position,
        "live_velocity": live_velocity,
        "live_extruder_velocity": live_extruder_velocity,
        "steppers": steppers,
        "trapq": trapqs,
    })
}

// ===========================================================================
// Shutdown diagnostics (pure log formatting, unit-tested directly)
// ===========================================================================

/// One stepper's `queue_step` block (`DumpStepper.log_steps`), oldest first.
///
/// `None` when the window is empty, which logs nothing (upstream returns
/// before printing). Unlike the batch stream this does **not** stop at a
/// `set_position` marker: the log is read by a human, not continued from.
fn stepper_dump_block(stepper: &str, mcu: &str, steps: &[HistoryStep]) -> Option<String> {
    if steps.is_empty() {
        return None;
    }
    let mut out = vec![format!(
        "Dumping stepper '{stepper}' ({mcu}) {} queue_step:",
        steps.len()
    )];
    for (index, step) in steps.iter().enumerate() {
        out.push(format!(
            "queue_step {index}: t={} p={} i={} c={} a={}",
            step.first_clock, step.start_position, step.interval, step.step_count, step.add
        ));
    }
    Some(out.join("\n"))
}

/// One trapq's move block (`DumpTrapQ.log_trapq`), oldest first.
fn trapq_dump_block(name: &str, moves: &[MoveSegment]) -> Option<String> {
    if moves.is_empty() {
        return None;
    }
    let mut out = vec![format!("Dumping trapq '{name}' {} moves:", moves.len())];
    for (index, segment) in moves.iter().enumerate() {
        out.push(format!(
            "move {index}: pt={:.6} mt={:.6} sv={:.6} a={:.6} \
             sp=({:.6},{:.6},{:.6}) ar=({:.6},{:.6},{:.6})",
            segment.print_time,
            segment.move_t,
            segment.start_v,
            // The queue keeps half the acceleration; the log publishes it
            // whole, as the dump stream does.
            2.0 * segment.half_accel,
            segment.start_pos.x(),
            segment.start_pos.y(),
            segment.start_pos.z(),
            segment.axes_r.x(),
            segment.axes_r.y(),
            segment.axes_r.z(),
        ));
    }
    Some(out.join("\n"))
}

/// The requested toolhead position at the shutdown time
/// (`motion_report.py:227-231`) — Python's `"...: %s"` of the `pos` tuple.
fn shutdown_position_block(shutdown_time: f64, pos: [f64; 3]) -> String {
    format!(
        "Requested toolhead position at shutdown time {shutdown_time:.6}: ({:?}, {:?}, {:?})",
        pos[0], pos[1], pos[2]
    )
}

// ===========================================================================
// Per-instance batch state
// ===========================================================================
/// One `dump_stepper` instance: one MCU stepper's stream.
struct DumpStepper {
    printer: Weak<Printer>,
    name: String,
    last_batch_clock: Mutex<u64>,
}

impl DumpStepper {
    fn process_batch(&self) -> Option<Value> {
        let printer = self.printer.upgrade()?;
        let toolhead = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)?;
        let mut last_batch_clock = self
            .last_batch_clock
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let history = toolhead.stepper_step_history(
            &self.name,
            usize::MAX,
            *last_batch_clock,
            NEVER_CLOCK,
        )?;
        stepper_batch(
            &mut last_batch_clock,
            &history.steps,
            history.step_dist,
            |clock| history.clock_to_print_time(clock),
        )
    }
}

/// Where a `dump_trapq` instance's queue lives.
#[derive(Debug, Clone)]
enum TrapqSource {
    /// The toolhead's main queue.
    ToolHead,
    /// An extruder's own queue; the section name is the instance name too.
    Extruder(String),
}

/// One `dump_trapq` instance: one queue's stream.
struct DumpTrapQ {
    printer: Weak<Printer>,
    source: TrapqSource,
    /// The most recent row already sent (`last_batch_msg`); `None` until the
    /// first batch.
    last_batch_msg: Mutex<Option<TrapqRow>>,
}

impl DumpTrapQ {
    fn process_batch(&self) -> Option<Value> {
        let printer = self.printer.upgrade()?;
        let toolhead = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)?;
        let trapq = resolve_trapq(&printer, &toolhead, &self.source)?;
        let mut last_batch_msg = self
            .last_batch_msg
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let start_time = trapq_resend_time(last_batch_msg.as_ref());
        let moves = toolhead.trapq_extract(trapq, usize::MAX, start_time, NEVER_TIME)?;
        trapq_batch(&mut last_batch_msg, &moves)
    }
}

/// The trapq id of `source`, from the live machine: the toolhead has its main
/// queue only after connect, and an extruder's is attached then too.
fn resolve_trapq(
    printer: &Arc<Printer>,
    toolhead: &ToolHeadObject,
    source: &TrapqSource,
) -> Option<usize> {
    match source {
        TrapqSource::ToolHead => toolhead.main_trapq(),
        TrapqSource::Extruder(name) => printer.lookup_object_as::<PrinterExtruder>(name)?.trapq(),
    }
}

/// A trapq's position at `print_time`, read through the window that holds the
/// segment covering it (`DumpTrapQ.get_trapq_position`, `motion_report.py:107`).
fn read_trapq_position(
    toolhead: &ToolHeadObject,
    trapq: usize,
    print_time: f64,
) -> Option<TrapqPosition> {
    let moves = toolhead.trapq_extract(trapq, 1, 0.0, print_time)?;
    trapq_position(&moves, print_time)
}

/// A batch callback over one instance's `process_batch`.
fn instance_batch_cb<T: Send + Sync + 'static>(
    instance: Arc<T>,
    process_batch: fn(&T) -> Option<Value>,
) -> BatchCb {
    Arc::new(move |_eventtime: f64| {
        let message = process_batch(&instance);
        Box::pin(async move { Ok(message) })
    })
}

/// The start / stop callbacks: nothing to do on either side (upstream defaults
/// both to a no-op, `bulk_sensor.py:32-36`).
fn noop_lifecycle() -> LifecycleCb {
    Arc::new(|| Box::pin(async { Ok(()) }))
}

// ===========================================================================
// The module object
// ===========================================================================

/// The machine's motion-report endpoints (upstream's `PrinterMotionReport`).
pub struct MotionReport {
    printer: Weak<Printer>,
    /// Every registered `dump_stepper` instance's stepper name
    /// (upstream's `self.steppers` keys).
    steppers: Mutex<Vec<String>>,
    /// Every registered `dump_trapq` instance: its name and the queue it reads
    /// (upstream's `self.dtrapqs`).
    trapqs: Mutex<Vec<(String, TrapqSource)>>,
    /// The last `get_status` answer and the eventtime it stands until
    /// (upstream's `last_status` / `next_status_time`).
    status_cache: Mutex<StatusCache>,
}

/// The cached `get_status` answer (`motion_report.py:146-150`).
#[derive(Debug)]
struct StatusCache {
    next_status_time: f64,
    status: Value,
}

impl MotionReport {
    /// The machine's motion report, created on first use
    /// (`printer.load_object(config, 'motion_report')`, `stepper.py:282-285`).
    pub fn ensure(printer: &Arc<Printer>) -> Arc<Self> {
        if let Some(existing) = printer.lookup_object_as::<Self>(MOTION_REPORT_OBJECT) {
            return existing;
        }
        let object = Arc::new(Self {
            printer: Arc::downgrade(printer),
            steppers: Mutex::new(Vec::new()),
            trapqs: Mutex::new(Vec::new()),
            // Upstream's initial `last_status`: a three-axis zero position and
            // no velocities, until something is registered and read.
            status_cache: Mutex::new(StatusCache {
                next_status_time: 0.0,
                status: live_status(&[0.0; E_AXIS], 0.0, 0.0, &[], &[]),
            }),
        });
        printer
            .add_object(
                MOTION_REPORT_OBJECT,
                Arc::clone(&object) as Arc<dyn PrinterObject>,
            )
            .expect("`motion_report` is registered once per machine");
        // `klippy:analyze_shutdown` — the queue windows around a stop
        // (`motion_report.py:162-163`). The handler holds the object weakly, as
        // the object holds the printer weakly: a strong handle either way is a
        // cycle.
        {
            let weak = Arc::downgrade(&object);
            printer.register_event_handler(
                KlippyEvent::KlippyAnalyzeShutdown {
                    msg: String::new(),
                    details: HashMap::new(),
                },
                Box::new(move |event| {
                    if let KlippyEvent::KlippyAnalyzeShutdown { msg, details } = event {
                        if let Some(object) = weak.upgrade() {
                            object.handle_analyze_shutdown(msg, details);
                        }
                    }
                }),
            );
        }
        object
    }

    /// Register one MCU stepper's [`DUMP_STEPPER_PATH`] instance
    /// (`DumpStepper.__init__`, reached from upstream's `register_stepper`).
    ///
    /// # Errors
    /// A duplicate instance name (each stepper name is one mux value).
    pub fn register_stepper(self: &Arc<Self>, name: &str) -> Result<(), ConfigError> {
        let instance = Arc::new(DumpStepper {
            printer: self.printer.clone(),
            name: name.to_string(),
            last_batch_clock: Mutex::new(0),
        });
        self.add_dump_endpoint(
            DUMP_STEPPER_PATH,
            name,
            json!({ "header": STEPPER_HEADER }),
            instance_batch_cb(instance, DumpStepper::process_batch),
        )?;
        self.own_mutex(&self.steppers).push(name.to_string());
        Ok(())
    }

    /// Register the toolhead's main queue's [`DUMP_TRAPQ_PATH`] instance
    /// (`motion_report.py:169-172`).
    ///
    /// # Errors
    /// [`TOOLHEAD_TRAPQ_NAME`] is already taken.
    pub fn register_toolhead_trapq(self: &Arc<Self>) -> Result<(), ConfigError> {
        self.register_trapq(TOOLHEAD_TRAPQ_NAME, TrapqSource::ToolHead)
    }

    /// Register one extruder's [`DUMP_TRAPQ_PATH`] instance, named after its
    /// section (`motion_report.py:173-179`).
    ///
    /// # Errors
    /// A duplicate instance name.
    pub fn register_extruder_trapq(self: &Arc<Self>, name: &str) -> Result<(), ConfigError> {
        self.register_trapq(name, TrapqSource::Extruder(name.to_string()))
    }

    fn register_trapq(&self, name: &str, source: TrapqSource) -> Result<(), ConfigError> {
        let instance = Arc::new(DumpTrapQ {
            printer: self.printer.clone(),
            source: source.clone(),
            last_batch_msg: Mutex::new(None),
        });
        self.add_dump_endpoint(
            DUMP_TRAPQ_PATH,
            name,
            json!({ "header": TRAPQ_HEADER }),
            instance_batch_cb(instance, DumpTrapQ::process_batch),
        )?;
        self.own_mutex(&self.trapqs)
            .push((name.to_string(), source));
        Ok(())
    }

    /// Give one instance its own [`BatchBulkHelper`] and mux registration
    /// (`batch_bulk.add_mux_endpoint(path, "name", name, api_resp)`).
    fn add_dump_endpoint(
        &self,
        path: &str,
        name: &str,
        start_resp: Value,
        batch_cb: BatchCb,
    ) -> Result<(), ConfigError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| ConfigError::new("motion_report: the printer is gone"))?;
        let helper = BatchBulkHelper::new(
            &printer,
            batch_cb,
            noop_lifecycle(),
            noop_lifecycle(),
            BATCH_INTERVAL,
        );
        helper.add_mux_endpoint(path, DUMP_KEY, name, start_resp)
    }

    /// One of this object's own mutexes, surviving a poisoned lock: a panic in
    /// one status read must not turn every later one into a panic.
    fn own_mutex<'a, T>(&self, mutex: &'a Mutex<T>) -> std::sync::MutexGuard<'a, T> {
        mutex.lock().unwrap_or_else(|poison| poison.into_inner())
    }

    /// The `get_status` answer, read fresh (upstream's `get_status` body after
    /// the refresh gate, `motion_report.py:150-263`).
    ///
    /// The position is read from the queues at the MCU's estimated print time —
    /// not from the planner's commanded position — so a status query shows what
    /// the machine is doing now rather than where the look-ahead will take it.
    /// Before connect (no planner, no dated clock) every number is `0.0`.
    fn read_status(&self) -> Value {
        let mut steppers = self.own_mutex(&self.steppers).clone();
        steppers.sort();
        let registered = self.own_mutex(&self.trapqs).clone();
        let mut trapqs: Vec<String> = registered.iter().map(|(name, _)| name.clone()).collect();
        trapqs.sort();
        let Some(printer) = self.printer.upgrade() else {
            return live_status(&[0.0; E_AXIS], 0.0, 0.0, &steppers, &trapqs);
        };
        let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) else {
            return live_status(&[0.0; E_AXIS], 0.0, 0.0, &steppers, &trapqs);
        };
        // One slot per axis the toolhead carries: X, Y, Z, then an extra axis
        // each (`live_pos[:3]` plus `live_pos[ea_index]`, upstream keeps the
        // three kinematics slots in the list too).
        let extra_axes = toolhead.get_extra_axes();
        let mut live_position = vec![0.0; E_AXIS + extra_axes.len()];
        let Some(print_time) = toolhead.estimated_print_time() else {
            return live_status(&live_position, 0.0, 0.0, &steppers, &trapqs);
        };
        let mut live_velocity = 0.0;
        let mut live_extruder_velocity = 0.0;
        if let Some(main_trapq) = toolhead.main_trapq() {
            if let Some(position) = read_trapq_position(&toolhead, main_trapq, print_time) {
                live_position[..E_AXIS].copy_from_slice(&position.pos);
                live_velocity = position.velocity;
            }
        }
        for (index, axis) in extra_axes.iter().enumerate() {
            // Upstream looks the axis up in its own instance table; an axis
            // without one keeps its zero (and contributes no velocity).
            let Some(source) = registered
                .iter()
                .find(|(name, _)| name == axis.name())
                .map(|(_, source)| source)
            else {
                continue;
            };
            let Some(trapq) = resolve_trapq(&printer, &toolhead, source) else {
                continue;
            };
            let Some(position) = read_trapq_position(&toolhead, trapq, print_time) else {
                continue;
            };
            live_position[E_AXIS + index] = position.pos[0];
            // Upstream reports the first extra axis' velocity as the extruder's
            // (`motion_report.py:259-260`).
            if index == 0 {
                live_extruder_velocity = position.velocity;
            }
        }
        live_status(
            &live_position,
            live_velocity,
            live_extruder_velocity,
            &steppers,
            &trapqs,
        )
    }

    /// `_handle_analyze_shutdown` (`motion_report.py:225-243`): log the queue
    /// windows around the stop.
    fn handle_analyze_shutdown(&self, msg: &str, details: &HashMap<String, Value>) {
        for block in self.shutdown_report(msg, details) {
            info!("{block}");
        }
    }

    /// The log blocks `_dump_shutdown` emits, in upstream's order: the stopping
    /// MCU's steppers, then every registered trapq, then the requested toolhead
    /// position at that instant.
    ///
    /// Empty when there is nothing to dump — an unrelated shutdown message, a
    /// report carrying no `shutdown_clock` or naming an MCU this host does not
    /// have (upstream returns on each of those too).
    fn shutdown_report(&self, msg: &str, details: &HashMap<String, Value>) -> Vec<String> {
        // Upstream also dumps on `"Internal error in stepcompress"`; this port
        // reports that error as its own message without `step_gen_time`, so
        // there is no window to date.
        if msg != "MCU shutdown" {
            return Vec::new();
        }
        let Some(printer) = self.printer.upgrade() else {
            return Vec::new();
        };
        let Some(mcu_name) = details.get("mcu").and_then(Value::as_str) else {
            return Vec::new();
        };
        let Some(shutdown_clock) = details.get("shutdown_clock").and_then(Value::as_i64) else {
            return Vec::new();
        };
        let Some(mcu) = printer
            .lookup_objects_as::<McuObject>(Some("mcu"))
            .into_iter()
            .find(|(_, object)| object.name() == mcu_name)
            .map(|(_, object)| object)
        else {
            return Vec::new();
        };
        let Some(shutdown_time) = mcu.clock_to_print_time(shutdown_clock) else {
            return Vec::new();
        };
        let toolhead = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT);
        // Only the steppers driven by the MCU that stopped.
        let steppers: Vec<String> = self
            .own_mutex(&self.steppers)
            .iter()
            .filter(|name| {
                toolhead
                    .as_ref()
                    .and_then(|toolhead| toolhead.mcu_stepper(name))
                    .is_some_and(|stepper| stepper.chip().name() == mcu_name)
            })
            .cloned()
            .collect();
        self.dump_shutdown(
            &printer,
            toolhead.as_deref(),
            &mcu,
            &steppers,
            shutdown_time,
            SHUTDOWN_DUMP_DURATION,
        )
    }

    /// `_dump_shutdown` (`motion_report.py:195-223`): the stepper queue-step
    /// windows, every trapq's window and the toolhead's requested position, as
    /// the log blocks to emit.
    fn dump_shutdown(
        &self,
        printer: &Arc<Printer>,
        toolhead: Option<&ToolHeadObject>,
        mcu: &McuObject,
        stepper_names: &[String],
        shutdown_time: f64,
        duration: f64,
    ) -> Vec<String> {
        let start_time = shutdown_time - duration;
        let end_time = shutdown_time + duration;
        let mut blocks = Vec::new();
        let Some(toolhead) = toolhead else {
            return blocks;
        };
        for name in stepper_names {
            let (Some(start_clock), Some(end_clock)) = (
                mcu.print_time_to_clock(start_time),
                mcu.print_time_to_clock(end_time),
            ) else {
                continue;
            };
            let Some(history) =
                toolhead.stepper_step_history(name, usize::MAX, start_clock, end_clock)
            else {
                continue;
            };
            // The history walk returns the runs newest first; the log reads
            // oldest first.
            let mut steps = history.steps;
            steps.reverse();
            blocks.extend(stepper_dump_block(name, mcu.name(), &steps));
        }
        for (name, source) in self.own_mutex(&self.trapqs).clone() {
            let Some(trapq) = resolve_trapq(printer, toolhead, &source) else {
                continue;
            };
            let Some(mut moves) = toolhead.trapq_extract(trapq, usize::MAX, start_time, end_time)
            else {
                continue;
            };
            moves.reverse();
            blocks.extend(trapq_dump_block(&name, &moves));
        }
        // The toolhead's own position at the instant it stopped.
        let position = resolve_trapq(printer, toolhead, &TrapqSource::ToolHead)
            .and_then(|trapq| read_trapq_position(toolhead, trapq, shutdown_time));
        if let Some(position) = position {
            blocks.push(shutdown_position_block(shutdown_time, position.pos));
        }
        blocks
    }
}

impl PrinterObject for MotionReport {
    /// Upstream's `PrinterMotionReport.get_status` (`motion_report.py:244-263`),
    /// behind its `STATUS_REFRESH_TIME` cache: a client that polls faster than
    /// four times a second gets the last answer rather than another pair of
    /// queue reads.
    fn get_status(&self, eventtime: f64) -> Value {
        let mut cache = self.own_mutex(&self.status_cache);
        if eventtime < cache.next_status_time {
            return cache.status.clone();
        }
        cache.next_status_time = eventtime + STATUS_REFRESH_TIME;
        let status = self.read_status();
        cache.status = status.clone();
        status
    }

    fn is_queryable(&self) -> bool {
        true
    }
}

impl std::fmt::Debug for MotionReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotionReport").finish_non_exhaustive()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    use crate::core::klippy::api::protocol::{PushTarget, Request};
    use crate::core::klippy::api::registry::{Api, EndpointContext, MuxEndpoint};
    use crate::core::klippy::api::webhooks;
    use crate::core::klippy::mathutil::Xyz;
    use crate::core::klippy::reactor::ManualReactor;

    fn segment(
        print_time: f64,
        move_t: f64,
        start_v: f64,
        accel: f64,
        start_pos: [f64; 3],
        axes_r: [f64; 3],
    ) -> MoveSegment {
        MoveSegment {
            print_time,
            move_t,
            start_v,
            half_accel: 0.5 * accel,
            start_pos: Xyz::new(start_pos[0], start_pos[1], start_pos[2]),
            axes_r: Xyz::new(axes_r[0], axes_r[1], axes_r[2]),
        }
    }

    #[test]
    fn test_dump_trapq_row_shape_for_one_move() {
        let mut last_batch_msg = None;
        // One cruise segment: 2 mm/s for 0.5 s along +Z, from (10, 20, 30).
        let moves = [segment(
            1.0,
            0.5,
            2.0,
            1000.0,
            [10., 20., 30.],
            [0., 0., 1.],
        )];

        let message = trapq_batch(&mut last_batch_msg, &moves).expect("one row");

        assert_eq!(
            message,
            json!({"data": [[1.0, 0.5, 2.0, 1000.0, [10.0, 20.0, 30.0], [0.0, 0.0, 1.0]]]}),
            "time/duration/start_velocity/acceleration/start_position/direction, with the \
             acceleration published whole"
        );
        assert_eq!(
            last_batch_msg,
            Some(trapq_row(&moves[0])),
            "the last sent row is remembered"
        );
    }

    #[test]
    fn test_dump_trapq_batch_drops_the_reread_row_and_reports_nothing_new() {
        // The read window re-reads the last sent row; it is sent once.
        let last_row = trapq_row(&segment(1.0, 0.5, 2.0, 1000.0, [0., 0., 0.], [1., 0., 0.]));
        let mut last_batch_msg = Some(last_row);
        let moves = [
            // Newest first, as `extract_old` returns them; the batch goes out
            // oldest first, so the re-read row leads it.
            segment(2.0, 0.25, 1.0, 0.0, [0., 0., 0.], [1., 0., 0.]),
            segment(1.0, 0.5, 2.0, 1000.0, [0., 0., 0.], [1., 0., 0.]),
        ];

        let message = trapq_batch(&mut last_batch_msg, &moves).expect("the new row");
        let data = message["data"].as_array().expect("data is a list");
        assert_eq!(data.len(), 1, "the re-read row is not sent twice");
        assert_eq!(data[0][0], 2.0);
        assert_eq!(
            last_batch_msg,
            Some(trapq_row(&moves[0])),
            "the batch records its newest row"
        );

        assert_eq!(
            trapq_batch(&mut last_batch_msg, &[]),
            None,
            "nothing new means no message"
        );
    }

    #[test]
    fn test_dump_trapq_reads_from_the_resend_window() {
        let last_row = trapq_row(&segment(10.0, 0.5, 1.0, 0.0, [0., 0., 0.], [1., 0., 0.]));
        assert_eq!(
            trapq_resend_time(Some(&last_row)),
            10.1,
            "at most 0.100 s before the last move"
        );
        let short_row = trapq_row(&segment(10.0, 0.02, 1.0, 0.0, [0., 0., 0.], [1., 0., 0.]));
        assert_eq!(
            trapq_resend_time(Some(&short_row)),
            10.02,
            "a short move is re-read from its own start"
        );
        assert_eq!(trapq_resend_time(None), 0.0, "the first batch reads from 0");
    }

    #[test]
    fn test_dump_stepper_batch_stops_after_the_first_set_position_marker() {
        // Newest first, as the step compressor's history returns them: the
        // newest run, then a `set_position` marker, then the older runs.
        let newest_first = vec![
            HistoryStep {
                first_clock: 500,
                last_clock: 600,
                start_position: 7,
                step_count: 2,
                interval: 30,
                add: 0,
            },
            HistoryStep {
                first_clock: 400,
                last_clock: 400,
                start_position: 7,
                step_count: 0,
                interval: 0,
                add: 0,
            },
            HistoryStep {
                first_clock: 200,
                last_clock: 350,
                start_position: 2,
                step_count: 5,
                interval: 25,
                add: 3,
            },
            HistoryStep {
                first_clock: 100,
                last_clock: 150,
                start_position: 0,
                step_count: 2,
                interval: 20,
                add: 1,
            },
        ];
        let mut last_batch_clock = 0;

        let message = stepper_batch(&mut last_batch_clock, &newest_first, 0.01, |clock| {
            clock as f64 / 1000.0
        })
        .expect("one batch");

        assert_eq!(
            message["data"],
            json!([[20, 2, 1], [25, 5, 3], [0, 0, 0]]),
            "oldest first, ending on the set_position marker"
        );
        assert_eq!(
            last_batch_clock, 400,
            "the batch ends at the marker's clock; the rows past it come later"
        );
        assert_eq!(
            message["start_mcu_position"], 0,
            "the batch starts at the oldest run's counter"
        );
        assert_eq!(message["start_position"], 0.0);
        assert_eq!(message["step_distance"], 0.01);
        assert_eq!(message["first_clock"], 100);
        assert_eq!(message["first_step_time"], 0.1);
        assert_eq!(message["last_clock"], 400);
        assert_eq!(message["last_step_time"], 0.4);
    }

    #[test]
    fn test_dump_stepper_batch_of_nothing_is_no_message() {
        let mut last_batch_clock = 12;
        assert_eq!(
            stepper_batch(&mut last_batch_clock, &[], 0.01, |_| 0.0),
            None
        );
        assert_eq!(
            last_batch_clock, 12,
            "an empty batch does not move the clock"
        );
    }

    /// A recording push target for endpoint tests.
    #[derive(Default)]
    struct Recorder {
        closed: AtomicBool,
        messages: Mutex<Vec<Value>>,
    }

    impl PushTarget for Recorder {
        fn is_closed(&self) -> bool {
            self.closed.load(Ordering::SeqCst)
        }
        fn push(&self, message: Value) {
            self.messages.lock().unwrap().push(message);
        }
    }

    /// The instance the mux table routes `path` with `name` to.
    fn handler_for(
        registrations: &[webhooks::MuxRegistration],
        path: &str,
        name: &str,
    ) -> Arc<dyn MuxEndpoint> {
        registrations
            .iter()
            .find(|registration| {
                registration.path == path && registration.value.as_deref() == Some(name)
            })
            .map(|registration| Arc::clone(&registration.handler))
            .unwrap_or_else(|| panic!("no {path} instance named {name}"))
    }

    #[tokio::test]
    async fn test_both_dump_endpoints_are_muxed_by_name_with_upstream_headers() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let webhooks = webhooks::install(&printer).unwrap();
        let motion_report = MotionReport::ensure(&printer);
        // The two instances upstream registers in `_connect` plus one stepper.
        motion_report.register_toolhead_trapq().unwrap();
        motion_report.register_extruder_trapq("extruder").unwrap();
        motion_report.register_stepper("stepper_x").unwrap();

        let registrations = webhooks.take_mux_endpoints();
        assert_eq!(registrations.len(), 3);
        assert!(
            registrations
                .iter()
                .all(|registration| registration.key == DUMP_KEY),
            "both endpoints select their instance with `name`"
        );

        let api = Api::new();
        let client = Arc::new(Recorder::default());
        let context = EndpointContext {
            api: &api,
            client: Arc::clone(&client) as Arc<dyn PushTarget>,
        };

        let trapq_request = Request::parse(
            br#"{"method":"motion_report/dump_trapq","params":{"name":"extruder","response_template":{"method":"dump"}}}"#,
        )
        .unwrap();
        let reply = handler_for(&registrations, DUMP_TRAPQ_PATH, "extruder")
            .handle(&trapq_request, &context)
            .await
            .unwrap();
        assert_eq!(
            reply,
            json!({"header": ["time", "duration", "start_velocity", "acceleration", "start_position", "direction"]})
        );

        let stepper_request = Request::parse(
            br#"{"method":"motion_report/dump_stepper","params":{"name":"stepper_x","response_template":{"method":"dump"}}}"#,
        )
        .unwrap();
        let reply = handler_for(&registrations, DUMP_STEPPER_PATH, "stepper_x")
            .handle(&stepper_request, &context)
            .await
            .unwrap();
        assert_eq!(reply, json!({"header": ["interval", "count", "add"]}));
    }

    #[test]
    fn test_get_status_reports_the_five_keys_with_the_extruder_in_the_fourth_slot() {
        // One extra axis: the position is `x, y, z, e`, the extruder last.
        let steppers = vec!["stepper_x".to_string()];
        let trapqs = vec!["extruder".to_string(), "toolhead".to_string()];

        let status = live_status(&[10.0, 20.0, 30.0, 4.5], 25.0, 1.5, &steppers, &trapqs);

        assert_eq!(
            status["live_position"],
            json!([10.0, 20.0, 30.0, 4.5]),
            "three kinematics axes and one slot per extra axis"
        );
        assert_eq!(status["live_position"].as_array().unwrap().len(), 4);
        assert_eq!(status["live_velocity"], json!(25.0));
        assert_eq!(status["live_extruder_velocity"], json!(1.5));
        assert_eq!(status["steppers"], json!(["stepper_x"]));
        assert_eq!(status["trapq"], json!(["extruder", "toolhead"]));
        for key in ["live_velocity", "live_extruder_velocity"] {
            assert!(status[key].is_f64(), "{key} is a float");
        }
    }

    #[test]
    fn test_the_live_position_is_read_from_the_queue_at_the_given_time() {
        // 2 mm/s accelerating at 1000 mm/s^2 along +X from (10, 20, 30).
        let moves = [segment(
            1.0,
            0.5,
            2.0,
            1000.0,
            [10., 20., 30.],
            [1., 0., 0.],
        )];

        // 0.2 s in: distance = (2 + 500 * 0.2) * 0.2 = 20.4 mm.
        let position = trapq_position(&moves, 1.2).expect("a position");
        assert!((position.pos[0] - 30.4).abs() < 1e-9, "{position:?}");
        assert!((position.pos[1] - 20.0).abs() < 1e-9);
        assert!((position.pos[2] - 30.0).abs() < 1e-9);
        assert!(
            (position.velocity - 202.0).abs() < 1e-9,
            "start_v + accel * t = 2 + 1000 * 0.2"
        );

        // Before the segment starts the queue holds its start; past its end it
        // holds the end (upstream clamps the same way).
        let before = trapq_position(&moves, 0.5).expect("the segment start");
        assert_eq!(before.pos, [10.0, 20.0, 30.0]);
        assert_eq!(before.velocity, 2.0);
        let after = trapq_position(&moves, 5.0).expect("the segment end");
        // The whole 0.5 s: (2 + 500 * 0.5) * 0.5 = 126 mm.
        assert!((after.pos[0] - 136.0).abs() < 1e-9, "{after:?}");

        assert_eq!(
            trapq_position(&[], 1.0),
            None,
            "an empty queue has no position"
        );
    }

    #[test]
    fn test_shutdown_diagnostics_log_the_stepper_window_oldest_first() {
        let steps = vec![
            HistoryStep {
                first_clock: 100,
                last_clock: 160,
                start_position: 0,
                step_count: 3,
                interval: 20,
                add: 1,
            },
            HistoryStep {
                first_clock: 160,
                last_clock: 210,
                start_position: 3,
                step_count: 2,
                interval: 25,
                add: 0,
            },
        ];

        let block = stepper_dump_block("stepper_x", "mcu", &steps).expect("one block");

        assert_eq!(
            block,
            "Dumping stepper 'stepper_x' (mcu) 2 queue_step:\n\
             queue_step 0: t=100 p=0 i=20 c=3 a=1\n\
             queue_step 1: t=160 p=3 i=25 c=2 a=0",
            "upstream's `DumpStepper.log_steps` lines"
        );
        assert_eq!(
            stepper_dump_block("stepper_x", "mcu", &[]),
            None,
            "an empty window logs nothing"
        );
    }

    #[test]
    fn test_shutdown_diagnostics_log_the_trapq_window_oldest_first() {
        let moves = [
            segment(1.0, 0.5, 2.0, 1000.0, [10., 20., 30.], [0., 0., 1.]),
            segment(1.5, 0.25, 4.0, 0.0, [10., 20., 30.5], [0., 0., 1.]),
        ];

        let block = trapq_dump_block("toolhead", &moves).expect("one block");

        assert_eq!(
            block,
            "Dumping trapq 'toolhead' 2 moves:\n\
             move 0: pt=1.000000 mt=0.500000 sv=2.000000 a=1000.000000 \
             sp=(10.000000,20.000000,30.000000) ar=(0.000000,0.000000,1.000000)\n\
             move 1: pt=1.500000 mt=0.250000 sv=4.000000 a=0.000000 \
             sp=(10.000000,20.000000,30.500000) ar=(0.000000,0.000000,1.000000)",
            "upstream's `DumpTrapQ.log_trapq` lines, acceleration published whole"
        );
        assert_eq!(trapq_dump_block("toolhead", &[]), None);
    }

    #[test]
    fn test_shutdown_diagnostics_log_the_requested_position() {
        assert_eq!(
            shutdown_position_block(12.5, [1.5, -2.0, 3.25]),
            "Requested toolhead position at shutdown time 12.500000: (1.5, -2.0, 3.25)",
            "upstream's `%.6f` time and the tuple of the requested position"
        );
    }

    #[test]
    fn test_a_shutdown_without_an_mcu_clock_dumps_nothing() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        webhooks::install(&printer).unwrap();
        let motion_report = MotionReport::ensure(&printer);
        motion_report.register_toolhead_trapq().unwrap();

        // upstream's early returns: another message, no MCU named, no clock,
        // or an MCU this host does not have.
        assert_eq!(
            motion_report.shutdown_report("Internal error in stepcompress", &HashMap::new()),
            Vec::<String>::new()
        );
        assert_eq!(
            motion_report.shutdown_report("MCU shutdown", &HashMap::new()),
            Vec::<String>::new()
        );
        assert_eq!(
            motion_report.shutdown_report(
                "MCU shutdown",
                &HashMap::from([("mcu".to_string(), json!("mcu"))])
            ),
            Vec::<String>::new(),
            "no shutdown_clock: the window cannot be dated"
        );
        assert_eq!(
            motion_report.shutdown_report(
                "MCU shutdown",
                &HashMap::from([
                    ("mcu".to_string(), json!("mcu")),
                    ("shutdown_clock".to_string(), json!(1234)),
                ])
            ),
            Vec::<String>::new(),
            "no such MCU on this machine"
        );
    }

    #[test]
    fn test_motion_report_is_queryable() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        webhooks::install(&printer).unwrap();
        let motion_report = MotionReport::ensure(&printer);
        motion_report.register_toolhead_trapq().unwrap();
        motion_report.register_extruder_trapq("extruder").unwrap();
        motion_report.register_stepper("stepper_y").unwrap();
        motion_report.register_stepper("stepper_x").unwrap();

        assert!(
            printer
                .queryable_objects()
                .contains(&"motion_report".to_string()),
            "`objects/list` reports it"
        );
        let status = printer
            .statuses(0.0)
            .remove("motion_report")
            .expect("`objects/query` answers for it");

        // Nothing is connected: no planner, no dated clock, so every number is
        // zero and the position has only the three kinematics axes.
        assert_eq!(status["live_position"], json!([0.0, 0.0, 0.0]));
        assert_eq!(status["live_velocity"], json!(0.0));
        assert_eq!(status["live_extruder_velocity"], json!(0.0));
        assert_eq!(
            status["steppers"],
            json!(["stepper_x", "stepper_y"]),
            "the registered steppers, sorted as upstream sorts them"
        );
        assert_eq!(status["trapq"], json!(["extruder", "toolhead"]));
    }

    #[test]
    fn test_the_status_answer_stands_until_the_refresh_time() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        webhooks::install(&printer).unwrap();
        let motion_report = MotionReport::ensure(&printer);
        motion_report.register_toolhead_trapq().unwrap();

        let first = motion_report.get_status(0.0);
        assert_eq!(first["steppers"], json!([]));

        // Registered inside the 0.250 s window: the cached answer stands.
        motion_report.register_stepper("stepper_x").unwrap();
        let cached = motion_report.get_status(0.1);
        assert_eq!(cached["steppers"], json!([]), "the answer is reused");

        let refreshed = motion_report.get_status(0.3);
        assert_eq!(
            refreshed["steppers"],
            json!(["stepper_x"]),
            "past the refresh time the status is read again"
        );
    }
}
