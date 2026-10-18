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
//! # What is not ported yet
//!
//! * `get_status` (`live_position`, `live_velocity`, `live_extruder_velocity`,
//!   `steppers`, `trapq`) — `motion_report.py:232-263`.
//! * The shutdown diagnostics on `klippy:analyze_shutdown`
//!   (`motion_report.py:188-231`).
//! * The `manual_stepper` trapq instances: this port gives `manual_stepper` no
//!   trapq (its timeline is dwell-only), so there is no queue to stream.
//! * The drip-timing guard (`motion_queuing.check_drip_timing`,
//!   `motion_report.py:125-132`), which withholds trapq entries that may still
//!   change while homing. This host does not expose the drip's end time, so a
//!   homing move's tail may appear in one batch and be corrected by the next.
//!
//! [`Trapq::extract_old`]: crate::core::klippy::motion::trapq::Trapq::extract_old

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, LifecycleCb, BATCH_INTERVAL,
};
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
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
        let trapq = self.resolve_trapq(&printer, &toolhead)?;
        let mut last_batch_msg = self
            .last_batch_msg
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let start_time = trapq_resend_time(last_batch_msg.as_ref());
        let moves = toolhead.trapq_extract(trapq, usize::MAX, start_time, NEVER_TIME)?;
        trapq_batch(&mut last_batch_msg, &moves)
    }

    /// The trapq id to read, from the live machine: the toolhead has it only
    /// after connect, and an extruder's is attached then too.
    fn resolve_trapq(&self, printer: &Arc<Printer>, toolhead: &ToolHeadObject) -> Option<usize> {
        match &self.source {
            TrapqSource::ToolHead => toolhead.main_trapq(),
            TrapqSource::Extruder(name) => {
                printer.lookup_object_as::<PrinterExtruder>(name)?.trapq()
            }
        }
    }
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
        });
        printer
            .add_object(
                MOTION_REPORT_OBJECT,
                Arc::clone(&object) as Arc<dyn PrinterObject>,
            )
            .expect("`motion_report` is registered once per machine");
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
        )
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
            source,
            last_batch_msg: Mutex::new(None),
        });
        self.add_dump_endpoint(
            DUMP_TRAPQ_PATH,
            name,
            json!({ "header": TRAPQ_HEADER }),
            instance_batch_cb(instance, DumpTrapQ::process_batch),
        )
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
}

impl PrinterObject for MotionReport {
    /// Upstream's `PrinterMotionReport` has `get_status`; this port has only
    /// the two dump endpoints so far, so a status query answers empty (the same
    /// surface as `force_move`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
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
}
