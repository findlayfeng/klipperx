//! `load_cell` — load-cell force sensors (upstream's
//! `klippy/extras/load_cell.py`).
//!
//! The section is the whole sensor's: `[load_cell]` / `[load_cell <name>]`
//! carries `sensor_type` and the chip's own options, and the chip
//! ([`hx71x`](crate::core::klippy::extras::hx71x)) is built from that same
//! section (`load_config` → `sensor_class(config)`). This module owns:
//!
//! | piece | upstream |
//! |---|---|
//! | [`section!`](crate::core::klippy::load) + option reading | `load_config` / `LoadCell.__init__` |
//! | [`LoadCell`] force tracking (tare / grams) | `LoadCell` |
//! | `load_cell/dump_force` mux endpoint | `ApiClientHelper.add_mux_endpoint` |
//! | `LOAD_CELL_*` mux commands | `LoadCellCommandHelper` |
//! | `load_cell:calibrate` / `load_cell:tare` on ready | `_handle_do_ready` |
//!
//! # Known gaps
//!
//! * `sensor_type` accepts all five upstream chips, but only `hx711`,
//!   `hx717`, `ads131m02` and `ads131m04` are built here — `ads1220`
//!   reports *not implemented* (LC-3).
//! * The four `LOAD_CELL_*` commands are registered with upstream's help
//!   strings but answer *not implemented*: they need the sample collector
//!   (`LoadCellSampleCollector`) and, for `LOAD_CELL_CALIBRATE`, the
//!   interactive `LoadCellGuidedCalibrationHelper`.
//! * `[load_cell_probe]`, the guided calibration's `configfile.set` write-back
//!   and the `trigger_analog` attach are not wired.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::api::protocol::{ApiError, PushTarget, Request, ResponseTemplate};
use crate::core::klippy::api::registry::{EndpointContext, EndpointFuture, MuxEndpoint};
use crate::core::klippy::api::webhooks;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::ads1220::Ads1220;
use crate::core::klippy::extras::ads131m0x::{params_for as ads131m0x_params_for, Ads131M0x};
use crate::core::klippy::extras::bulk_sensor::ClientCb;
use crate::core::klippy::extras::hx71x::{params_for, Hx71x};
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

// Loaded after the buses and heaters (order 30) with the other devices.
section!(
    "load_cell",
    order = 40,
    load = load_config,
    prefix = load_config_prefix
);

/// The dump endpoint clients stream force from
/// (`load_cell/dump_force`).
pub const DUMP_ENDPOINT: &str = "load_cell/dump_force";

/// The mux key that selects a load cell (`"load_cell"`).
pub const DUMP_KEY: &str = "load_cell";

/// The G-Code mux key the four commands are selected by (`"LOAD_CELL"`).
pub const LOAD_CELL_KEY: &str = "LOAD_CELL";

/// The columns the dump endpoint's start response advertises
/// (`{"header": ["time", "force (g)", "counts", "tare_counts"]}`).
const DUMP_HEADER: [&str; 4] = ["time", "force (g)", "counts", "tare_counts"];

/// The smallest calibrated `counts_per_gram` (`MIN_COUNTS_PER_GRAM = 1.`).
pub const MIN_COUNTS_PER_GRAM: f64 = 1.;

/// Every `sensor_type` upstream's table knows (`HX71X_SENSOR_TYPES` ∪
/// `ADS1220_SENSOR_TYPE` ∪ `ADS131M0X_SENSOR_TYPES`).
const SENSOR_TYPES: [&str; 5] = ["hx711", "hx717", "ads1220", "ads131m02", "ads131m04"];

/// The `sensor_orientation` choices (`{'normal': 1., 'inverted': -1.}`).
const SENSOR_ORIENTATIONS: [&str; 2] = ["normal", "inverted"];

/// The `LOAD_CELL_*` commands with their upstream help strings
/// (`LoadCellCommandHelper`).
const LOAD_CELL_COMMANDS: [(&str, &str); 4] = [
    ("LOAD_CELL_TARE", "Set the Zero point of the load cell"),
    ("LOAD_CELL_CALIBRATE", "Start interactive calibration tool"),
    ("LOAD_CELL_READ", "Take a reading from the load cell"),
    ("LOAD_CELL_DIAGNOSTIC", "Check the health of the load cell"),
];

// ===========================================================================
// Pure helpers (unit-tested directly)
// ===========================================================================

/// `counts_to_grams`: the tared, scaled, orientation-corrected force — or
/// `None` until the cell is both calibrated and tared.
pub fn counts_to_grams(
    counts: i64,
    tare_counts: Option<i64>,
    counts_per_gram: Option<f64>,
    invert: f64,
) -> Option<f64> {
    let tare_counts = tare_counts?;
    let counts_per_gram = counts_per_gram?;
    Some(invert * ((counts - tare_counts) as f64 / counts_per_gram))
}

/// `counts_to_percent`: the count against the sensor's positive range
/// (`(float(counts) / float(range_max)) * 100.`).
pub fn counts_to_percent(counts: i64, range_max: i64) -> f64 {
    (counts as f64 / range_max as f64) * 100.
}

/// One converted sensor row (`[time, force (g), counts, tare_counts]`), or
/// `None` when the row cannot be read.
///
/// This is `_sensor_data_event`'s per-sample mapping; the force column is
/// `null` until the cell is calibrated and tared (upstream's `counts_to_grams`
/// returns `None`, which JSON carries as `null`).
pub fn convert_row(
    row: &[Value],
    tare_counts: Option<i64>,
    counts_per_gram: Option<f64>,
    invert: f64,
) -> Option<Value> {
    let time = row.first()?.as_f64()?;
    let counts = row.get(1)?.as_i64()?;
    let grams = counts_to_grams(counts, tare_counts, counts_per_gram, invert);
    Some(json!([time, grams, counts, tare_counts]))
}

/// Python's `str(value)` for a whole float: `1.` reads `1.0`
/// (`"... must have minimum of %s" % (minval,)` upstream).
fn py_float(value: f64) -> String {
    let text = format!("{value}");
    if text.contains(['.', 'e', 'E']) {
        text
    } else {
        format!("{text}.0")
    }
}

/// Python's `round(value, digits)` (the force columns round to one place).
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

// ===========================================================================
// The load cell
// ===========================================================================

/// One load cell's chip (`self.sensor` — the `BulkSensorAdc` upstream swaps
/// per `sensor_type`).
///
/// Each variant owns its own stream and implements the same interface:
/// samples per second, saturated range, status, error naming, and the batch
/// client registration [`LoadCellState::start_sensor_client`] uses.
pub enum LoadSensor {
    /// An HX711 or HX717 ([`hx71x`](crate::core::klippy::extras::hx71x)).
    Hx71x(Arc<Hx71x>),
    /// An ADS1220 ([`ads1220`](crate::core::klippy::extras::ads1220)).
    Ads1220(Arc<Ads1220>),
    /// An ADS131M02 or ADS131M04
    /// ([`ads131m0x`](crate::core::klippy::extras::ads131m0x)).
    Ads131M0x(Arc<Ads131M0x>),
}

impl LoadSensor {
    /// Samples per second the chip streams (`get_samples_per_second`).
    pub fn samples_per_second(&self) -> f64 {
        match self {
            Self::Hx71x(sensor) => sensor.samples_per_second() as f64,
            Self::Ads1220(sensor) => sensor.samples_per_second() as f64,
            Self::Ads131M0x(sensor) => sensor.samples_per_second(),
        }
    }

    /// The saturated bounds of the chip's samples (`get_range`).
    pub fn range(&self) -> (i64, i64) {
        match self {
            Self::Hx71x(sensor) => sensor.range(),
            Self::Ads1220(sensor) => sensor.range(),
            Self::Ads131M0x(sensor) => sensor.range(),
        }
    }

    /// The chip's own counters (`get_status`).
    pub fn status(&self, eventtime: f64) -> Value {
        match self {
            Self::Hx71x(sensor) => sensor.status(eventtime),
            Self::Ads1220(sensor) => sensor.status(eventtime),
            Self::Ads131M0x(sensor) => sensor.status(eventtime),
        }
    }

    /// A firmware error's name (`lookup_sensor_error`).
    pub fn lookup_sensor_error(&self, error_code: i64) -> String {
        match self {
            Self::Hx71x(sensor) => sensor.lookup_sensor_error(error_code),
            Self::Ads1220(sensor) => sensor.lookup_sensor_error(error_code),
            Self::Ads131M0x(sensor) => sensor.lookup_sensor_error(error_code),
        }
    }

    /// Register a converted-batch client with the chip (`add_client`, the
    /// first one starts the stream).
    pub fn add_client(&self, client: ClientCb) {
        match self {
            Self::Hx71x(sensor) => sensor.add_client(client),
            Self::Ads1220(sensor) => sensor.add_client(client),
            Self::Ads131M0x(sensor) => sensor.add_client(client),
        }
    }
}

/// One load cell: the sensor behind it, the tare/calibration state, the force
/// buffer, and the `dump_force` fan-out (`LoadCell`).
pub struct LoadCell {
    state: Arc<LoadCellState>,
}

/// Everything the callbacks and the interface share (`LoadCell`'s fields).
struct LoadCellState {
    /// The section's last name segment (`config.get_name().split()[-1]`).
    name: String,
    /// The machine, for the ready handler and the events.
    printer: Weak<Printer>,
    /// The chip (`self.sensor`, the `BulkSensorAdc`).
    sensor: LoadSensor,
    /// `1.` or `-1.` (`sensor_orientation`).
    invert: f64,
    /// `reference_tare_counts`, before a first tare.
    reference_tare_counts: Option<i64>,
    /// `counts_per_gram`, until calibrated.
    counts_per_gram: Option<f64>,
    /// The live tare (`tare_counts`, seeded from `reference_tare_counts`).
    tare_counts: Mutex<Option<i64>>,
    /// The recent force samples (`_force_buffer`, `int(sps / 2)` deep).
    force_buffer: Mutex<VecDeque<f64>>,
    /// The batch fan-out (`ApiClientHelper.client_cbs`): the force tracker
    /// plus every `dump_force` connection.
    clients: Mutex<Vec<ClientCb>>,
    /// Whether the sensor's client is attached (`_handle_do_ready`).
    sensor_client: Mutex<bool>,
}

impl LoadCell {
    /// Read the section and build the cell (`load_config` +
    /// `LoadCell.__init__`).
    ///
    /// # Errors
    /// An unknown or missing `sensor_type`, a chip option complaint, or any
    /// option of the cell itself.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let config_name = config.identifier().to_string();
        let name = config_name
            .split_whitespace()
            .last()
            .unwrap_or_default()
            .to_string();

        // `load_config` reads `sensor_type` first and hands the *same*
        // section to the chip.
        let sensor_type = config.get_choice("sensor_type", &SENSOR_TYPES, None)?;
        let sensor = match sensor_type.as_str() {
            "hx711" | "hx717" => {
                let params = params_for(&sensor_type)
                    .expect("the hx71x branch covers exactly its own sensor types");
                LoadSensor::Hx71x(Arc::new(Hx71x::new(config, printer, &params)?))
            }
            "ads1220" => LoadSensor::Ads1220(Arc::new(Ads1220::new(config, printer)?)),
            "ads131m02" | "ads131m04" => {
                let params = ads131m0x_params_for(&sensor_type)
                    .expect("the ads131m0x branch covers exactly its own sensor types");
                LoadSensor::Ads131M0x(Arc::new(Ads131M0x::new(config, printer, &params)?))
            }
            other => {
                return Err(ConfigError::new(format!(
                    "sensor_type '{other}' is not implemented in this host"
                )));
            }
        };

        // The cell's own options, in upstream's order.
        let reference_tare_counts = config.get_optional_int("reference_tare_counts")?;
        let counts_per_gram = match config.get_optional_float("counts_per_gram")? {
            None => None,
            Some(value) if value < MIN_COUNTS_PER_GRAM => {
                return Err(ConfigError::new(format!(
                    "Option 'counts_per_gram' in section '{config_name}' must have minimum of {}",
                    py_float(MIN_COUNTS_PER_GRAM)
                )));
            }
            Some(value) => Some(value),
        };
        let orientation =
            config.get_choice("sensor_orientation", &SENSOR_ORIENTATIONS, Some("normal"))?;
        let invert = if orientation == "inverted" { -1. } else { 1. };

        let tare_counts = reference_tare_counts;
        // `int(sps / 2)`, never below one slot.
        let buffer_size = (sensor.samples_per_second() / 2.0).max(0.0) as usize;
        let force_buffer = VecDeque::with_capacity(buffer_size.max(1));

        // `_track_force` runs as one of the fan-out's clients, so it sees the
        // converted rows like every other client.
        let state = Arc::new(LoadCellState {
            name,
            printer: Arc::downgrade(printer),
            sensor,
            invert,
            reference_tare_counts,
            counts_per_gram,
            tare_counts: Mutex::new(tare_counts),
            force_buffer: Mutex::new(force_buffer),
            clients: Mutex::new(Vec::new()),
            sensor_client: Mutex::new(false),
        });
        // `load_cell/dump_force`, keyed by the section's name.
        let webhooks = webhooks::install(printer)?;
        webhooks.register_mux_endpoint(
            DUMP_ENDPOINT,
            DUMP_KEY,
            Some(&state.name),
            Arc::new(DumpForceEndpoint {
                state: Arc::downgrade(&state),
                start_resp: json!({ "header": DUMP_HEADER }),
            }),
        )?;

        // The four mux commands (`LoadCellCommandHelper`).
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        for (command, help) in LOAD_CELL_COMMANDS {
            let handler: CommandHandler = Arc::new(move |_gcmd: &GcodeCommand| {
                let error = CommandError::new(format!("{command} is not implemented in this host"));
                Box::pin(async move { Err::<(), CommandError>(error) })
            });
            gcode
                .register_mux_command(
                    command,
                    LOAD_CELL_KEY,
                    Some(state.name.as_str()),
                    Arc::clone(&handler),
                    Some(help),
                )
                .map_err(|err| ConfigError::new(format!("{config_name}: {err}")))?;
            // A bare `[load_cell]` answers to the default instance too
            // (`if len(name_parts) == 1: self.register_commands(None)`).
            if !config_name.contains(' ') {
                gcode
                    .register_mux_command(command, LOAD_CELL_KEY, None, handler, Some(help))
                    .map_err(|err| ConfigError::new(format!("{config_name}: {err}")))?;
            }
        }

        // Announce the stored calibration on ready (`_handle_do_ready`).
        let ready_state = Arc::downgrade(&state);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_event| {
                if let Some(state) = ready_state.upgrade() {
                    state.announce_state();
                }
            }),
        );

        Ok(Self { state })
    }

    /// The load cell's name (the section's last segment).
    pub fn name(&self) -> &str {
        &self.state.name
    }

    /// The chip behind this cell (`get_sensor`).
    pub fn sensor(&self) -> &LoadSensor {
        &self.state.sensor
    }

    /// The stored `reference_tare_counts` (`get_reference_tare_counts`).
    pub fn reference_tare_counts(&self) -> Option<i64> {
        self.state.reference_tare_counts
    }

    /// The live tare (`get_tare_counts`).
    pub fn tare_counts(&self) -> Option<i64> {
        *self
            .state
            .tare_counts
            .lock()
            .unwrap_or_else(|p| p.into_inner())
    }

    /// The calibration scale (`get_counts_per_gram`).
    pub fn counts_per_gram(&self) -> Option<f64> {
        self.state.counts_per_gram
    }

    /// Whether both calibration values are present (`is_calibrated`).
    pub fn is_calibrated(&self) -> bool {
        self.state.counts_per_gram.is_some() && self.state.reference_tare_counts.is_some()
    }

    /// Whether a tare is stored (`is_tared`).
    pub fn is_tared(&self) -> bool {
        self.tare_counts().is_some()
    }

    /// Store a new tare (`tare`).
    pub fn tare(&self, tare_counts: i64) {
        self.state.tare(tare_counts);
    }

    /// Raw counts → grams (`counts_to_grams`).
    pub fn counts_to_grams(&self, counts: i64) -> Option<f64> {
        counts_to_grams(
            counts,
            self.tare_counts(),
            self.state.counts_per_gram,
            self.state.invert,
        )
    }

    /// Raw counts against the sensor's full scale (`counts_to_percent`).
    pub fn counts_to_percent(&self, counts: i64) -> f64 {
        let (_, range_max) = self.state.sensor.range();
        counts_to_percent(counts, range_max)
    }

    /// Add a converted-batch client (`ApiClientHelper.add_client`).
    pub fn add_client(&self, client: ClientCb) {
        self.state
            .clients
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(client);
    }

    /// How many converted-batch clients are registered (tests and
    /// bookkeeping).
    #[cfg(test)]
    pub(crate) fn client_count(&self) -> usize {
        self.state
            .clients
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .len()
    }
}

impl LoadCellState {
    /// Store a tare and announce it (`LoadCell.tare`).
    fn tare(&self, tare_counts: i64) {
        *self.tare_counts.lock().unwrap_or_else(|p| p.into_inner()) = Some(tare_counts);
        if let Some(printer) = self.printer.upgrade() {
            printer.send_event(&KlippyEvent::LoadCellTare);
        }
    }

    /// Fan a converted message out to the registered clients
    /// (`ApiClientHelper.send`), dropping clients that are gone.
    fn send(&self, message: &Value) {
        let mut clients = self.clients.lock().unwrap_or_else(|p| p.into_inner());
        clients.retain(|client| client(message));
    }

    /// `_sensor_data_event`: map `[time, counts, adc]` rows to
    /// `[time, force (g), counts, tare_counts]` and forward the message.
    fn convert(&self, message: &Value) -> Option<Value> {
        let rows = message.get("data")?.as_array()?;
        let tare_counts = *self.tare_counts.lock().unwrap_or_else(|p| p.into_inner());
        let mut converted = Vec::with_capacity(rows.len());
        for row in rows {
            let row = row.as_array()?;
            converted.push(convert_row(
                row,
                tare_counts,
                self.counts_per_gram,
                self.invert,
            )?);
        }
        Some(json!({
            "data": converted,
            "errors": message.get("errors").cloned().unwrap_or(Value::Null),
            "overflows": message.get("overflows").cloned().unwrap_or(Value::Null),
        }))
    }

    /// `_track_force`: remember recent grams while calibrated and tared.
    fn track_force(&self, message: &Value) {
        if !(self.counts_per_gram.is_some() && self.reference_tare_counts.is_some()) {
            return;
        }
        let Some(rows) = message.get("data").and_then(Value::as_array) else {
            return;
        };
        let mut buffer = self.force_buffer.lock().unwrap_or_else(|p| p.into_inner());
        for row in rows {
            if let Some(grams) = row.get(1).and_then(Value::as_f64) {
                buffer.push_back(grams);
            }
        }
    }

    /// `_force_g`: the average/min/max of the recent force, when calibrated
    /// and tared (`{}` otherwise).
    fn force_g(&self) -> Value {
        let buffer = self.force_buffer.lock().unwrap_or_else(|p| p.into_inner());
        if !self.is_calibrated() || !self.is_tared() || buffer.is_empty() {
            return json!({});
        }
        let sum: f64 = buffer.iter().sum();
        let average = sum / buffer.len() as f64;
        json!({
            "force_g": round(average, 1),
            "min_force_g": round(buffer.iter().copied().fold(f64::INFINITY, f64::min), 1),
            "max_force_g": round(buffer.iter().copied().fold(f64::NEG_INFINITY, f64::max), 1),
        })
    }

    /// Whether both calibration values are present (`is_calibrated`).
    fn is_calibrated(&self) -> bool {
        self.counts_per_gram.is_some() && self.reference_tare_counts.is_some()
    }

    /// Whether a tare is stored (`is_tared`).
    fn is_tared(&self) -> bool {
        self.tare_counts
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_some()
    }

    /// Announce the stored calibration and tare (`_handle_do_ready`).
    fn announce_state(&self) {
        if let Some(printer) = self.printer.upgrade() {
            if self.is_calibrated() {
                printer.send_event(&KlippyEvent::LoadCellCalibrate);
            }
            if self.is_tared() {
                printer.send_event(&KlippyEvent::LoadCellTare);
            }
        }
    }

    /// Attach the sensor client once (`_handle_do_ready`: `sensor.add_client`
    /// + the force tracker).
    ///
    /// The converted message goes to [`Self::send`], and the force tracker is
    /// the first client of that fan-out.
    fn start_sensor_client(self: &Arc<Self>) {
        let mut attached = self.sensor_client.lock().unwrap_or_else(|p| p.into_inner());
        if *attached {
            return;
        }
        *attached = true;
        drop(attached);

        let tracker = Arc::downgrade(self);
        self.clients
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(Arc::new(move |message: &Value| {
                if let Some(state) = tracker.upgrade() {
                    state.track_force(message);
                }
                true
            }));

        let weak = Arc::downgrade(self);
        self.sensor.add_client(Arc::new(move |message: &Value| {
            let Some(state) = weak.upgrade() else {
                return false;
            };
            match state.convert(message) {
                Some(converted) => {
                    state.send(&converted);
                    true
                }
                None => false,
            }
        }));
    }
}

/// `load_cell/dump_force`: one API connection, converting as it fans out
/// (`ApiClientHelper._add_webhooks_client`).
struct DumpForceEndpoint {
    /// The cell this instance belongs to.
    state: Weak<LoadCellState>,
    /// The header every connection is greeted with.
    start_resp: Value,
}

impl MuxEndpoint for DumpForceEndpoint {
    fn handle<'a>(
        &'a self,
        request: &'a Request,
        context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let state = self
                .state
                .upgrade()
                .ok_or_else(|| ApiError::Internal("the load cell is gone".to_string()))?;
            let template = ResponseTemplate::from_params(&request.params())?;
            let pusher = ForcePush {
                target: Arc::clone(&context.client),
                template,
            };
            state
                .clients
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(Arc::new(move |message: &Value| pusher.send(message)));
            Ok(self.start_resp.clone())
        })
    }
}

/// A batch pusher for one `dump_force` connection (`BatchWebhooksClient`).
struct ForcePush {
    /// The connection, kept so the subscription outlives its request.
    target: Arc<dyn PushTarget>,
    /// The reply envelope the batches are wrapped in.
    template: ResponseTemplate,
}

impl ForcePush {
    /// Push one batch; `false` when the connection is gone.
    fn send(&self, message: &Value) -> bool {
        if self.target.is_closed() {
            return false;
        }
        self.target.push(self.template.message(message.clone()));
        true
    }
}

impl PrinterObject for LoadCell {
    /// Attach to the sensor when the machine comes up (`klippy:ready` →
    /// `_handle_do_ready` → `sensor.add_client`), which starts the stream:
    /// the batch helper's first client runs the start callback.
    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        let state = Arc::clone(&self.state);
        Box::pin(async move {
            state.start_sensor_client();
            Ok::<(), KlippyError>(())
        })
    }

    /// The force fields plus the calibration state and the chip's counters
    /// (`LoadCell.get_status`).
    fn get_status(&self, eventtime: f64) -> Value {
        let mut status = self
            .state
            .force_g()
            .as_object()
            .cloned()
            .unwrap_or_default();
        status.insert("is_calibrated".to_string(), json!(self.is_calibrated()));
        status.insert(
            "counts_per_gram".to_string(),
            json!(self.state.counts_per_gram),
        );
        status.insert(
            "reference_tare_counts".to_string(),
            json!(self.state.reference_tare_counts),
        );
        status.insert("tare_counts".to_string(), json!(self.tare_counts()));
        if let Some(sensor) = self.state.sensor.status(eventtime).as_object() {
            status.extend(sensor.clone());
        }
        Value::Object(status)
    }
}

/// Upstream's `load_config` (the bare `[load_cell]`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(LoadCell::new(config, printer)?))
}

/// Upstream's `load_config_prefix` (`[load_cell <name>]`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    load_config(config, printer)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("load_cell", name);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A section and a tracked wrapper for it, leaked so the borrow outlives
    /// the call (tests only).
    fn wrap(name: Option<&str>, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
        ConfigWrapper::new(
            Box::leak(Box::new(section(name, options))),
            AccessTracking::shared(),
        )
    }

    /// A ready printer with `pins`, `gcode` and one registered `[mcu]`.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let mcu = McuObject::new(ConfigSection::new("mcu", None), &printer).unwrap();
        printer.add_object("mcu", Arc::new(mcu)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    /// A refused section's message (`unwrap_err` without `Debug` on the
    /// built object).
    fn refuse<T>(result: Result<T, ConfigError>) -> String {
        match result {
            Err(err) => err.to_string(),
            Ok(_) => panic!("expected the section to be refused"),
        }
    }

    /// A minimal hx711 section's chip options (the corpus's `my_hx711`).
    const HX711_CHIP: [(&str, &str); 3] = [
        ("sensor_type", "hx711"),
        ("sclk_pin", "PA3"),
        ("dout_pin", "PA5"),
    ];

    #[test]
    fn test_the_corpus_hx71x_sections_load_and_read_every_option() {
        let path = klipperx_test_support::klipper_dir().join("test/klippy/load_cell.cfg");
        let text = std::fs::read_to_string(&path).expect("load_cell.cfg is readable");
        let config = Config::from_text(&text).expect("load_cell.cfg parses").0;
        let access = AccessTracking::shared();
        let printer = printer();

        let mut loaded: Vec<String> = Vec::new();
        let mut refused: Vec<(String, String)> = Vec::new();
        for sect in config.sections() {
            if sect.id != "load_cell" {
                continue;
            }
            let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));
            match load_config_prefix(&wrapper, &printer) {
                Ok(_) => {
                    // The section's contract with the option check: every
                    // option the file gives it was read.
                    for option in sect.parameters.keys() {
                        assert!(
                            access.contains(&sect.identifier(), option),
                            "unread option '{}' in '{}'",
                            option,
                            sect.identifier()
                        );
                    }
                    loaded.push(sect.identifier());
                }
                Err(err) => refused.push((sect.identifier(), err.to_string())),
            }
        }
        // Every chip family in the corpus file loads now (hx71x + ads1220 +
        // ads131m0x): LC-3 closed the last ADS gap.
        let mut loaded = loaded;
        loaded.sort();
        assert_eq!(
            loaded,
            [
                "load_cell my_ads1220",
                "load_cell my_ads131m02",
                "load_cell my_hx711",
                "load_cell my_hx717"
            ],
            "the implemented cells of {path:?}"
        );
        let refused: Vec<&str> = refused.iter().map(|(_, err)| err.as_str()).collect();
        assert!(
            refused.is_empty(),
            "no chip gap remains (hx71x + ads1220 + ads131m0x): {refused:?}"
        );
    }

    #[tokio::test]
    async fn test_an_ads131m02_cell_attaches_its_sensor_to_the_bulk_stream() {
        // The corpus options for `[load_cell my_ads131m02]`.
        let printer = printer();
        let cell = LoadCell::new(&wrap(Some("my_ads131m02"), &ADS131M02_CHIP), &printer).unwrap();
        // 8.192 MHz / (2 * 8192) = 500 SPS.
        assert_eq!(cell.sensor().samples_per_second(), 500.0);
        assert_eq!(cell.sensor().range(), (-0x80_0000, 0x7F_FFFF));
        // `klippy:ready` → `_handle_do_ready` → `sensor.add_client`, which is
        // the chip's pass-through to `BatchBulkHelper` (the spawned batch loop
        // has not been polled yet, so the count is that registration).
        cell.connect().await.unwrap();
        let LoadSensor::Ads131M0x(sensor) = cell.sensor() else {
            panic!("the cell's chip is not an ads131m0x");
        };
        assert_eq!(sensor.client_count(), 1);
    }

    #[tokio::test]
    async fn test_an_ads1220_cell_attaches_its_sensor_to_the_bulk_stream() {
        // The corpus options for `[load_cell my_ads1220]`.
        let printer = printer();
        let cell = LoadCell::new(
            &wrap(
                Some("my_ads1220"),
                &[
                    ("sensor_type", "ads1220"),
                    ("cs_pin", "PA0"),
                    ("data_ready_pin", "PA1"),
                ],
            ),
            &printer,
        )
        .unwrap();
        assert_eq!(cell.sensor().samples_per_second(), 660.0);
        // `klippy:ready` → `_handle_do_ready` → `sensor.add_client`.
        cell.connect().await.unwrap();
        let LoadSensor::Ads1220(sensor) = cell.sensor() else {
            panic!("the cell's chip is not an ads1220");
        };
        assert_eq!(sensor.client_count(), 1);
        assert_eq!(cell.client_count(), 1, "the force tracker is a client too");
    }

    /// The corpus options of `[load_cell my_ads131m02]`.
    const ADS131M02_CHIP: [(&str, &str); 4] = [
        ("sensor_type", "ads131m02"),
        ("cs_pin", "PB5"),
        ("data_ready_pin", "PB6"),
        ("clock_freq", "8192000"),
    ];

    #[test]
    fn test_a_missing_sensor_type_is_refused_upstream_wording() {
        let printer = printer();
        let err = refuse(LoadCell::new(&wrap(None, &[("sclk_pin", "PA3")]), &printer));
        assert_eq!(
            err,
            "Option 'sensor_type' in section 'load_cell' must be specified"
        );
    }

    #[test]
    fn test_an_unknown_sensor_type_is_refused_upstream_wording() {
        let printer = printer();
        let err = refuse(LoadCell::new(
            &wrap(
                Some("x"),
                &[("sensor_type", "loadcell"), ("dout_pin", "PA5")],
            ),
            &printer,
        ));
        assert_eq!(
            err,
            "Choice 'loadcell' for option 'sensor_type' in section 'load_cell x' is not a valid choice"
        );
    }

    #[test]
    fn test_sensor_orientation_selects_the_sign_and_defaults_to_normal() {
        let printer = printer();
        let cell = LoadCell::new(&wrap(Some("x"), &HX711_CHIP), &printer).unwrap();
        assert_eq!(cell.counts_to_grams(1000), None); // not calibrated yet
        assert!(!cell.is_tared() && !cell.is_calibrated());

        let mut options = HX711_CHIP.to_vec();
        options.push(("sensor_orientation", "sideways"));
        // Different pins: the first cell already claimed PA5/PA3.
        let options: Vec<(&str, &str)> = options
            .into_iter()
            .map(|(key, value)| match key {
                "sclk_pin" => (key, "PA7"),
                "dout_pin" => (key, "PJ0"),
                other => (other, value),
            })
            .collect();
        let err = refuse(LoadCell::new(&wrap(Some("x"), &options), &printer));
        assert_eq!(
            err,
            "Choice 'sideways' for option 'sensor_orientation' in section 'load_cell x' is not a valid choice"
        );
    }

    #[test]
    fn test_counts_per_gram_below_the_minimum_is_refused() {
        let printer = printer();
        let mut options = HX711_CHIP.to_vec();
        options.push(("counts_per_gram", "0.5"));
        let err = refuse(LoadCell::new(&wrap(Some("x"), &options), &printer));
        assert_eq!(
            err,
            "Option 'counts_per_gram' in section 'load_cell x' must have minimum of 1.0"
        );
    }

    #[test]
    fn test_the_calibration_state_reaches_the_status() {
        let machine = printer();
        // `reference_tare_counts` + `counts_per_gram` = calibrated *and* tared.
        let mut options = HX711_CHIP.to_vec();
        options.push(("reference_tare_counts", "1000"));
        options.push(("counts_per_gram", "100.0"));
        let cell = load_config(&wrap(Some("x"), &options), &machine).unwrap();
        let status = cell.get_status(0.);
        assert_eq!(status["is_calibrated"], true);
        assert_eq!(status["counts_per_gram"], 100.0);
        assert_eq!(status["reference_tare_counts"], 1000);
        assert_eq!(status["tare_counts"], 1000);
        // The chip's own counters merge in (`HX71xBase.get_status`).
        assert_eq!(status["errors"], 0);
        assert_eq!(status["overflows"], 0);
        assert_eq!(status["sample_rate"], 80);
        // No force rows yet, so no force fields (`_force_g` needs the buffer).
        assert!(status.get("force_g").is_none());

        // An uncalibrated cell reports nulls and no force (its own printer:
        // PA5/PA3 belong to the first cell).
        let bare_printer = printer();
        let bare = load_config(&wrap(Some("y"), &HX711_CHIP), &bare_printer).unwrap();
        let status = bare.get_status(0.);
        assert_eq!(status["is_calibrated"], false);
        assert!(status["counts_per_gram"].is_null());
        assert!(status["reference_tare_counts"].is_null());
        assert!(status["tare_counts"].is_null());
        assert!(status.get("force_g").is_none());
    }

    #[test]
    fn test_the_dump_endpoint_is_registered_under_the_load_cell_key() {
        let printer = printer();
        load_config(&wrap(Some("x"), &HX711_CHIP), &printer).unwrap();

        let webhooks = webhooks::install(&printer).unwrap();
        let registrations = webhooks.take_mux_endpoints();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].path, DUMP_ENDPOINT);
        assert_eq!(registrations[0].key, DUMP_KEY);
        assert_eq!(registrations[0].value.as_deref(), Some("x"));
    }

    #[test]
    fn test_the_four_load_cell_commands_are_registered_with_upstream_help() {
        let printer = printer();
        load_config(&wrap(Some("x"), &HX711_CHIP), &printer).unwrap();

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let help = gcode.command_help();
        for (command, description) in LOAD_CELL_COMMANDS {
            assert_eq!(
                help.get(command).map(String::as_str),
                Some(description),
                "command {command}"
            );
        }
    }

    #[test]
    fn test_counts_to_grams_applies_tare_scale_and_orientation() {
        // 100 counts over a 100 counts/gram scale, normal orientation.
        assert_eq!(counts_to_grams(1100, Some(1000), Some(100.), 1.), Some(1.));
        // Inverted mounts read negative.
        assert_eq!(
            counts_to_grams(1100, Some(1000), Some(100.), -1.),
            Some(-1.)
        );
        // Not calibrated or not tared: no force (`None` → JSON `null`).
        assert_eq!(counts_to_grams(1100, None, Some(100.), 1.), None);
        assert_eq!(counts_to_grams(1100, Some(1000), None, 1.), None);

        // A dump row is `[time, force (g), counts, tare_counts]`.
        let row = vec![json!(1.5), json!(1042), json!(0.5)];
        let converted = convert_row(&row, Some(1000), Some(100.), 1.).unwrap();
        assert_eq!(converted, json!([1.5, 0.42, 1042, 1000]));
        // Uncalibrated: the force column is null, the counts still flow.
        let converted = convert_row(&row, None, None, 1.).unwrap();
        assert_eq!(converted, json!([1.5, null, 1042, null]));
    }

    #[test]
    fn test_counts_to_percent_uses_the_positive_range() {
        assert!((counts_to_percent(0x40_0000, 0x7F_FFFF) - 50.).abs() < 1e-5);
        assert_eq!(counts_to_percent(0x7F_FFFF, 0x7F_FFFF), 100.);
        assert_eq!(counts_to_percent(0, 0x7F_FFFF), 0.);
    }
}
