//! `bulk_sensor` — the bulk batch-read framework (upstream's
//! `klippy/extras/bulk_sensor.py`, ticket H6).
//!
//! A sensor that streams thousands of samples per second must not cost the
//! host a message per sample. This module is the shared machinery:
//!
//! | item | upstream | role |
//! |---|---|---|
//! | [`BatchBulkHelper`] | `BatchBulkHelper` | periodic batch processing + client fan-out + mux endpoint |
//! | [`FixedFreqReader`] | `FixedFreqReader` | clock-synchronized pull of `sensor_bulk_data` blocks |
//! | [`SampleFormat`] | `struct.Struct(unpack_fmt)` | sample byte order and size (`bytes_per_sample` → `samples_per_block`) |
//! | [`ClockSyncRegression`] | `ClockSyncRegression` | sample-rate / timestamp estimation by EMA regression |
//! | `BulkDataQueue` | `BulkDataQueue` | per-oid queue behind the one bound `sensor_bulk_data` callback |
//!
//! # What differs from upstream, and why
//!
//! Upstream's helper drives a reactor timer whose callback blocks on a query
//! round-trip; this host forbids blocking work on the reactor, so the timer
//! body became a spawned async task (`BatchBulkHelper::spawn_loop`). The
//! observable contract is unchanged: the loop starts with the first client,
//! processes one batch every `batch_interval`, calls every client with each
//! message, unregisters a client whose callback returns `false`, and runs the
//! stop callback when the last one goes.
//!
//! One callback can be bound per message name, so all sensors on an MCU share
//! one bound `sensor_bulk_data` closure and route by oid — the same thing
//! upstream's `register_serial_response(..., oid=…)` does per sensor.

use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex, OnceLock, Weak};
use std::time::Duration;

use serde_json::Value;

use crate::core::klippy::api::protocol::{ApiError, PushTarget, Request, ResponseTemplate};
use crate::core::klippy::api::registry::{EndpointContext, EndpointFuture, MuxEndpoint};
use crate::core::klippy::api::webhooks;
use crate::core::klippy::cmd::ldc1612::{QueryStatusLdc1612, SensorBulkData, SensorBulkStatus};
use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::config::ConfigError;
use crate::core::klippy::mcu::{Mcu, McuError, McuObject};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::printer::Printer;

/// Seconds between batch callbacks when none is given
/// (`bulk_sensor.BATCH_INTERVAL`).
pub const BATCH_INTERVAL: f64 = 0.500;

/// The largest `sensor_bulk_data` payload the firmware sends
/// (`MAX_BULK_MSG_SIZE`).
pub const MAX_BULK_MSG_SIZE: usize = 51;

/// Bytes one sample occupies under the **default** format: upstream's
/// `FixedFreqReader(mcu, …, ">I")` — a big-endian 32-bit value, which is what
/// ldc1612 (and [`FixedFreqReader::new`]) unpacks. A reader built with
/// [`FixedFreqReader::with_format`] derives its own size from the format
/// string instead of this constant.
pub const BYTES_PER_SAMPLE: usize = 4;

/// Samples one full `sensor_bulk_data` message carries under the default
/// format (`MAX_BULK_MSG_SIZE // bytes_per_sample` in upstream; a reader with
/// another format computes it as [`SampleFormat::samples_per_block`]).
pub const SAMPLES_PER_BLOCK: usize = MAX_BULK_MSG_SIZE / BYTES_PER_SAMPLE;

/// How long one `sensor_bulk_status` query (any chip's `query_*_status`)
/// round-trip may take.
const QUERY_TIMEOUT: Duration = Duration::from_secs(1);

/// One raw sample: `(print time, raw value)`.
///
/// The raw value is converted to the sensor's unit by the sensor itself
/// (`ldc1612._convert_samples`), exactly as upstream pulls `(ptime, raw)`
/// and converts afterwards.
pub type Sample = (f64, u32);

/// A batch client: receives each message, returns `false` to unregister.
pub type ClientCb = Arc<dyn Fn(&Value) -> bool + Send + Sync>;

/// The future a batch or lifecycle callback returns ('static: the closures
/// capture owned state).
pub type BulkFuture<T> = Pin<Box<dyn Future<Output = T> + Send>>;

/// One batch: the message to fan out, or `None` when there is nothing to
/// report (upstream's falsy `{}` return from `_process_batch`).
pub type BatchCb = Arc<dyn Fn(f64) -> BulkFuture<Result<Option<Value>, String>> + Send + Sync>;

/// The start / stop callbacks; an error is logged and stops the helper.
pub type LifecycleCb = Arc<dyn Fn() -> BulkFuture<Result<(), String>> + Send + Sync>;

// ===========================================================================
// BatchBulkHelper
// ===========================================================================

/// The state a batch loop and [`add_client`](Self::add_client) share.
#[derive(Default)]
struct BulkState {
    clients: Vec<ClientCb>,
    running: bool,
    /// Set by [`BatchBulkHelper::stop`]: the registration that owned this helper
    /// is gone, so it must not take a client again (see [`BatchBulkHelper::add_client`]).
    detached: bool,
}

/// Periodic batch processing with client fan-out
/// (`bulk_sensor.BatchBulkHelper`).
///
/// Construct with [`new`](Self::new), then register clients; the first client
/// starts the loop (running the start callback first), and the last departing
/// client stops it (running the stop callback). [`add_mux_endpoint`](Self::add_mux_endpoint)
/// exposes the batches to API clients over a `*/dump_*` mux path.
///
/// `add_client` spawns onto the Tokio runtime, so it must be called from
/// inside one (an endpoint handler or a G-Code handler both are).
pub struct BatchBulkHelper {
    printer: Arc<Printer>,
    batch_cb: BatchCb,
    start_cb: LifecycleCb,
    stop_cb: LifecycleCb,
    batch_interval: f64,
    state: Mutex<BulkState>,
}

impl BatchBulkHelper {
    /// Build the helper over the machine whose reactor times it.
    pub fn new(
        printer: &Arc<Printer>,
        batch_cb: BatchCb,
        start_cb: LifecycleCb,
        stop_cb: LifecycleCb,
        batch_interval: f64,
    ) -> Arc<Self> {
        Arc::new(Self {
            printer: Arc::clone(printer),
            batch_cb,
            start_cb,
            stop_cb,
            batch_interval,
            state: Mutex::new(BulkState::default()),
        })
    }

    /// Register a client; the first one starts the batch loop
    /// (`add_client` → `_start` upstream).
    ///
    /// Returns `false` when the helper was detached: its mux registration is
    /// gone (a configuration reload dropped it), so it must not take a client
    /// again. The caller reports the endpoint as unknown rather than leaving a
    /// client subscribed to a stream that will never run — and, worse, having
    /// `finish` restart the loop for a registration that no longer exists.
    ///
    /// # Panics
    /// Panics when called outside a Tokio runtime — every caller here runs
    /// inside one.
    pub fn add_client(self: &Arc<Self>, client: ClientCb) -> bool {
        let start = {
            let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
            if state.detached {
                return false;
            }
            state.clients.push(client);
            if state.running {
                false
            } else {
                state.running = true;
                true
            }
        };
        if start {
            self.spawn_loop();
        }
        true
    }

    /// Register one instance of a `*/dump_*` mux endpoint that streams this
    /// helper's batches (`add_mux_endpoint` upstream).
    ///
    /// `start_resp` is the immediate reply a connecting client receives
    /// (for `ldc1612/dump_ldc1612`: the column header).
    pub fn add_mux_endpoint(
        self: &Arc<Self>,
        path: &str,
        key: &str,
        value: &str,
        start_resp: Value,
    ) -> Result<(), ConfigError> {
        let webhooks = webhooks::install(&self.printer)?;
        webhooks.register_mux_endpoint(
            path,
            key,
            Some(value),
            Arc::new(MuxBatchEndpoint {
                bulk: Arc::clone(self),
                start_resp,
                key: key.to_string(),
                value: value.to_string(),
            }),
        )
    }

    /// Fan one message out, dropping clients that asked to unregister
    /// (the loop body of upstream's `_proc_batch`).
    pub(crate) fn process_one(&self, message: &Value) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        state.clients.retain(|client| client(message));
    }

    /// How many clients are registered (tests and loop bookkeeping).
    pub(crate) fn client_count(&self) -> usize {
        self.state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clients
            .len()
    }

    /// Stop streaming to every registered client and refuse new ones.
    ///
    /// The loop sees the empty list on its next tick, breaks, and goes through
    /// [`finish`](Self::finish) — which turns `running` off (and skips the stop
    /// callback, because this helper's configuration is gone). `running` is
    /// deliberately left alone here: [`finish`] is the single owner of that
    /// flag, and it restarts the loop when a client registered again while
    /// stopping.
    ///
    /// `detached` is set **under the same lock** as the client list, so a
    /// request that resolved this instance just before the reload cannot add a
    /// client after the list was emptied (see [`add_client`](Self::add_client)).
    ///
    /// Called by [`MuxBatchEndpoint::detach`] when the host drops the mux
    /// registration (a configuration reload), so the old instance's stream
    /// stops with the configuration that started it.
    pub(crate) fn stop(&self) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        state.detached = true;
        state.clients.clear();
    }

    /// Whether the batch loop is running.
    #[cfg(test)]
    fn is_running(&self) -> bool {
        self.state.lock().unwrap_or_else(|p| p.into_inner()).running
    }

    /// Spawn the loop if it is not already running (called under `running =
    /// true`, set by [`add_client`](Self::add_client)).
    fn spawn_loop(self: &Arc<Self>) {
        let helper = Arc::clone(self);
        tokio::spawn(async move {
            helper.run().await;
        });
    }

    /// The loop: start callback, then a batch every interval until the last
    /// client leaves or a callback errors, then the stop callback
    /// (upstream's `_start` / `_proc_batch` / `_stop` sequence).
    async fn run(self: Arc<Self>) {
        if let Err(err) = (self.start_cb)().await {
            tracing::error!("BatchBulkHelper start callback error: {err}");
            let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
            state.clients.clear();
            state.running = false;
            return;
        }
        loop {
            tokio::time::sleep(Duration::from_secs_f64(self.batch_interval)).await;
            if self.client_count() == 0 {
                break;
            }
            let eventtime = self.printer.reactor().monotonic();
            match (self.batch_cb)(eventtime).await {
                Ok(Some(message)) => self.process_one(&message),
                Ok(None) => {}
                Err(err) => {
                    tracing::error!("BatchBulkHelper batch callback error: {err}");
                    break;
                }
            }
            if self.client_count() == 0 {
                break;
            }
        }
        self.finish().await;
    }

    /// Stop: clear the clients, run the stop callback (unless the helper was
    /// detached), and restart if a new client arrived while stopping
    /// (upstream's `_stop`).
    async fn finish(self: &Arc<Self>) {
        let detached = {
            let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
            state.clients.clear();
            state.detached
        };
        // A detached helper's configuration is gone: the stop callback would
        // reach for objects `Printer::teardown` has already dropped (upstream
        // drops the whole printer instead of stopping it), so it is skipped
        // rather than reported as a failure.
        if !detached {
            if let Err(err) = (self.stop_cb)().await {
                tracing::error!("BatchBulkHelper stop callback error: {err}");
            }
        }
        let restart = {
            let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
            state.running = false;
            !state.detached && !state.clients.is_empty()
        };
        if restart {
            let start = {
                let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
                if state.running {
                    false
                } else {
                    state.running = true;
                    true
                }
            };
            if start {
                self.spawn_loop();
            }
        }
    }
}

// ===========================================================================
// Webhooks endpoint
// ===========================================================================

/// One instance of a `*/dump_*` mux endpoint over a [`BatchBulkHelper`].
pub struct MuxBatchEndpoint {
    bulk: Arc<BatchBulkHelper>,
    start_resp: Value,
    /// The mux key and instance value, so a request that arrives after
    /// [`detach`](Self::detach) can be answered as the unknown instance it now
    /// is.
    key: String,
    value: String,
}

impl MuxEndpoint for MuxBatchEndpoint {
    fn handle<'a>(
        &'a self,
        request: &'a Request,
        context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let template = ResponseTemplate::from_params(&request.params())?;
            let client = WebhooksBatchClient {
                target: Arc::clone(&context.client),
                template,
            };
            if !self
                .bulk
                .add_client(Arc::new(move |message: &Value| client.send(message)))
            {
                // The registration is gone: a reload detached this instance
                // after the request was routed to it. Answer like the table
                // would have — the client retries instead of waiting forever on
                // a stream that will never run.
                return Err(ApiError::UnknownMuxValue {
                    key: self.key.clone(),
                    value: self.value.clone(),
                });
            }
            Ok(self.start_resp.clone())
        })
    }

    /// Dropping this registration stops the helper's stream
    /// (`BatchBulkHelper::stop`), which the loop then finishes off.
    fn detach(&self) {
        self.bulk.stop();
    }
}

/// A batch client that pushes `template + {params: message}` to one API
/// connection (`BatchWebhooksClient` upstream).
struct WebhooksBatchClient {
    target: Arc<dyn PushTarget>,
    template: ResponseTemplate,
}

impl WebhooksBatchClient {
    /// Push one batch; `false` when the connection is gone (the helper then
    /// unregisters this client).
    fn send(&self, message: &Value) -> bool {
        if self.target.is_closed() {
            return false;
        }
        self.target.push(self.template.message(message.clone()));
        true
    }
}

// ===========================================================================
// ClockSyncRegression
// ===========================================================================

/// Sample-rate / timestamp estimation by exponentially-decayed linear
/// regression (`bulk_sensor.ClockSyncRegression`).
///
/// The MCU periodically reports `(clock, sample count)`; this tracks both in
/// averages and covarianes so it can answer "what is the print time of sample
/// N?" ([`get_time_translation`](Self::get_time_translation)).
pub struct ClockSyncRegression {
    /// MCU ticks expected between sample counters (the smoothing window).
    chip_clock_smooth: f64,
    decay: f64,
    last_chip_clock: f64,
    last_exp_mcu_clock: f64,
    mcu_clock_avg: f64,
    mcu_clock_variance: f64,
    chip_clock_avg: f64,
    chip_clock_covariance: f64,
}

impl ClockSyncRegression {
    /// Track a counter expected `chip_clock_smooth` MCU ticks apart.
    pub fn new(chip_clock_smooth: f64) -> Self {
        Self {
            chip_clock_smooth,
            decay: 1. / 20.,
            last_chip_clock: 0.,
            last_exp_mcu_clock: 0.,
            mcu_clock_avg: 0.,
            mcu_clock_variance: 0.,
            chip_clock_avg: 0.,
            chip_clock_covariance: 0.,
        }
    }

    /// Restart from one known-good pair (upstream `reset`).
    pub fn reset(&mut self, mcu_clock: f64, chip_clock: f64) {
        self.mcu_clock_avg = mcu_clock;
        self.chip_clock_avg = chip_clock;
        self.mcu_clock_variance = 0.;
        self.chip_clock_covariance = 0.;
        self.last_chip_clock = 0.;
        self.last_exp_mcu_clock = 0.;
    }

    /// Fold one observation into the regression (upstream `update`).
    pub fn update(&mut self, mcu_clock: f64, chip_clock: f64) {
        let decay = self.decay;
        let diff_mcu_clock = mcu_clock - self.mcu_clock_avg;
        self.mcu_clock_avg += decay * diff_mcu_clock;
        self.mcu_clock_variance =
            (1. - decay) * (self.mcu_clock_variance + diff_mcu_clock * diff_mcu_clock * decay);
        let diff_chip_clock = chip_clock - self.chip_clock_avg;
        self.chip_clock_avg += decay * diff_chip_clock;
        self.chip_clock_covariance =
            (1. - decay) * (self.chip_clock_covariance + diff_mcu_clock * diff_chip_clock * decay);
    }

    /// Anchor the translation at the newest sample counter (upstream
    /// `set_last_chip_clock`).
    pub fn set_last_chip_clock(&mut self, chip_clock: f64) {
        let (base_mcu, base_chip, inv_cfreq) = self.get_clock_translation();
        self.last_chip_clock = chip_clock;
        self.last_exp_mcu_clock = base_mcu + (chip_clock - base_chip) * inv_cfreq;
    }

    /// `(mcu clock, chip clock, MCU ticks per chip tick)` — either at the
    /// regression's centre, or projected to the newest anchor.
    pub fn get_clock_translation(&self) -> (f64, f64, f64) {
        let inv_chip_freq = self.mcu_clock_variance / self.chip_clock_covariance;
        if self.last_chip_clock == 0. {
            return (self.mcu_clock_avg, self.chip_clock_avg, inv_chip_freq);
        }
        // Find the mcu clock associated with the future chip clock.
        let s_chip_clock = self.last_chip_clock + self.chip_clock_smooth;
        let scdiff = s_chip_clock - self.chip_clock_avg;
        let s_mcu_clock = self.mcu_clock_avg + scdiff * inv_chip_freq;
        // Calculate the frequency to converge at the future point.
        let mdiff = s_mcu_clock - self.last_exp_mcu_clock;
        let s_inv_chip_freq = mdiff / self.chip_clock_smooth;
        (self.last_exp_mcu_clock, s_chip_clock, s_inv_chip_freq)
    }

    /// `(print time, chip clock, print seconds per chip tick)` — what turns
    /// sample counters into timestamps. `clock_to_print_time` maps MCU ticks
    /// to print time (the machine's synchronized clock).
    pub fn get_time_translation(
        &self,
        clock_to_print_time: impl Fn(i64) -> f64,
    ) -> (f64, f64, f64) {
        let (base_mcu, base_chip, inv_cfreq) = self.get_clock_translation();
        let base_time = clock_to_print_time(base_mcu.round() as i64);
        let inv_freq = clock_to_print_time((base_mcu + inv_cfreq).round() as i64) - base_time;
        (base_time, base_chip, inv_freq)
    }
}

// ===========================================================================
// BulkDataQueue: per-oid routing for sensor_bulk_data
// ===========================================================================

/// One sensor's inbound message queue.
type BulkQueue = Arc<Mutex<Vec<(u16, Vec<u8>)>>>;

/// Per-oid routing for `sensor_bulk_data`, one registry per MCU — the same
/// shape as `ds18b20`'s result registry (one callback per message name, so
/// the sensors share it and route by oid).
#[derive(Default)]
struct BulkDataRegistry {
    queues: Mutex<HashMap<u8, Weak<Mutex<Vec<(u16, Vec<u8>)>>>>>,
}

impl BulkDataRegistry {
    /// Route this MCU's `sensor_bulk_data` messages into `queue` for `oid`.
    fn bind(self: &Arc<Self>, mcu: &Mcu, oid: u8, queue: &BulkQueue) -> Result<(), McuError> {
        self.queues
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(oid, Arc::downgrade(queue));
        let registry = Arc::clone(self);
        mcu.bind_callback(SensorBulkData::NAME, move |values| {
            // `sensor_bulk_data oid=%c sequence=%hu data=%*s`, in that order.
            let (oid, sequence, data) = match (values.first(), values.get(1), values.get(2)) {
                (
                    Some(ArgValue::UInt8(oid)),
                    Some(ArgValue::UInt16(sequence)),
                    Some(ArgValue::Bytes(data)),
                ) => (*oid, *sequence, data.clone()),
                _ => return,
            };
            if let Some(queue) = registry
                .queues
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .get(&oid)
                .and_then(Weak::upgrade)
            {
                queue
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push((sequence, data));
            }
        })
    }
}

/// The registry for one MCU name, created on first use.
fn registry_for(name: &str) -> Arc<BulkDataRegistry> {
    static REGISTRIES: OnceLock<Mutex<HashMap<String, Arc<BulkDataRegistry>>>> = OnceLock::new();
    let registries = REGISTRIES.get_or_init(|| Mutex::new(HashMap::new()));
    let mut registries = registries.lock().unwrap_or_else(|p| p.into_inner());
    Arc::clone(
        registries
            .entry(name.to_string())
            .or_insert_with(|| Arc::new(BulkDataRegistry::default())),
    )
}

// ===========================================================================
// SampleFormat
// ===========================================================================

/// One sample's byte layout, parsed from an upstream `struct` format string
/// (`FixedFreqReader(mcu, chip_clock_smooth, unpack_fmt)`).
///
/// The format decides two things the reader needs: how many bytes one sample
/// occupies — and from it how many samples fit one 51-byte
/// [`MAX_BULK_MSG_SIZE`] message ([`samples_per_block`](Self::samples_per_block),
/// upstream's `MAX_BULK_MSG_SIZE // self.bytes_per_sample`) — and which end of
/// the byte run is the most significant when a sample is unpacked.
///
/// Sizes are summed field by field, packed with no alignment padding, which
/// is what Python's standard (`<`, `>`, `=`) and byte-only formats do; every
/// format the sensors in this host pass (`">I"`, `"<i"`, `"BBBBB"`, `"<hhh"`,
/// `">hhh"`) measures the same either way.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SampleFormat {
    /// The format string as given (for diagnostics).
    spec: String,
    /// Bytes one sample occupies (upstream's `unpack.size`).
    bytes_per_sample: usize,
    /// Fields the format unpacks (upstream's tuple arity).
    field_count: usize,
    /// Whether the sample's bytes are little-endian.
    little_endian: bool,
}

impl SampleFormat {
    /// Parse an upstream `struct` format string such as `">I"`, `"<i"` or
    /// `"BBBBB"`.
    ///
    /// A leading `<` selects little-endian and `>`/`!` big-endian; `=`, `@`
    /// and a missing prefix mean native order — little-endian on every target
    /// this host builds for.
    ///
    /// # Errors
    /// [`ConfigError`] for an empty format or a field code outside the integer
    /// codes (`b B ? h H i I l L q Q`), which is everything the bulk sensors
    /// pass.
    pub fn parse(spec: &str) -> Result<Self, ConfigError> {
        let mut chars = spec.chars();
        let (little_endian, fields) = match chars.next() {
            Some('<') => (true, chars.as_str()),
            Some('>') | Some('!') => (false, chars.as_str()),
            Some('=') | Some('@') => (cfg!(target_endian = "little"), chars.as_str()),
            Some(_) => (cfg!(target_endian = "little"), spec),
            None => {
                return Err(ConfigError::new(format!(
                    "sample format \"{spec}\" is empty"
                )))
            }
        };
        let mut bytes_per_sample = 0usize;
        let mut field_count = 0usize;
        for code in fields.chars() {
            bytes_per_sample += match code {
                'b' | 'B' | '?' => 1,
                'h' | 'H' => 2,
                'i' | 'I' | 'l' | 'L' => 4,
                'q' | 'Q' => 8,
                other => {
                    return Err(ConfigError::new(format!(
                        "sample format \"{spec}\": unsupported field code '{other}'"
                    )))
                }
            };
            field_count += 1;
        }
        if field_count == 0 {
            return Err(ConfigError::new(format!(
                "sample format \"{spec}\" has no fields"
            )));
        }
        Ok(Self {
            spec: spec.to_string(),
            bytes_per_sample,
            field_count,
            little_endian,
        })
    }

    /// Bytes one sample occupies (upstream's `unpack.size`).
    pub fn bytes_per_sample(&self) -> usize {
        self.bytes_per_sample
    }

    /// Samples one full `sensor_bulk_data` message carries
    /// (`MAX_BULK_MSG_SIZE // bytes_per_sample` upstream).
    pub fn samples_per_block(&self) -> usize {
        MAX_BULK_MSG_SIZE / self.bytes_per_sample
    }

    /// Unpack one sample's bytes as the raw value of a [`Sample`].
    ///
    /// # Errors
    /// The format cannot be carried by `Sample = (f64, u32)` — more than one
    /// field or wider than four bytes (adxl345's `"BBBBB"`, mpu9250's
    /// `">hhh"`; wiring those needs a wider sample type) — or `bytes` is not
    /// exactly one sample long.
    pub fn decode(&self, bytes: &[u8]) -> Result<u32, McuError> {
        if self.field_count != 1 || self.bytes_per_sample > 4 {
            return Err(McuError::Decode(format!(
                "sample format \"{}\" unpacks {} byte(s) in {} field(s); Sample = (f64, u32) \
                 cannot carry it",
                self.spec, self.bytes_per_sample, self.field_count
            )));
        }
        if bytes.len() != self.bytes_per_sample {
            return Err(McuError::Decode(format!(
                "sample format \"{}\" needs {} byte(s), got {}",
                self.spec,
                self.bytes_per_sample,
                bytes.len()
            )));
        }
        let mut value = 0_u32;
        if self.little_endian {
            for (index, byte) in bytes.iter().enumerate() {
                value |= u32::from(*byte) << (8 * index);
            }
        } else {
            for byte in bytes {
                value = (value << 8) | u32::from(*byte);
            }
        }
        Ok(value)
    }
}

impl Default for SampleFormat {
    /// Upstream's ldc1612 default: `">I"` — one big-endian 32-bit value.
    fn default() -> Self {
        Self {
            spec: ">I".to_string(),
            bytes_per_sample: 4,
            field_count: 1,
            little_endian: false,
        }
    }
}

// ===========================================================================
// FixedFreqReader
// ===========================================================================

/// Read `sensor_bulk_data` and timestamp fixed-rate samples
/// (`bulk_sensor.FixedFreqReader`).
///
/// Each pull first queries `sensor_bulk_status` for the firmware's clock and
/// sequence counters, folds that into a [`ClockSyncRegression`], then decodes
/// the queued messages into `(print time, raw)` samples.
///
/// The sample layout and the status-query command are parameters — upstream's
/// `unpack_fmt` and the `msgformat` of `setup_query_command` — with
/// [`new`](Self::new) keeping ldc1612's defaults.
pub struct FixedFreqReader {
    /// The machine this sensor is on, resolved when it binds.
    mcu_object: Mutex<Weak<McuObject>>,
    oid: Mutex<Option<u8>>,
    queue: BulkQueue,
    clock_sync: Mutex<ClockSyncRegression>,
    /// Messages reported so far (upstream's `last_sequence`, an int that
    /// accumulates across 16-bit wraps).
    last_sequence: Mutex<u64>,
    /// The longest status query tolerated before the measurement is skipped
    /// (upstream's duration filter).
    max_query_duration: Mutex<u32>,
    /// Messages the firmware could not deliver, accumulated across wraps.
    last_overflows: Mutex<u64>,
    /// The sample layout this sensor streams (upstream's `unpack_fmt`): byte
    /// order and [`bytes_per_sample`](SampleFormat::bytes_per_sample).
    format: SampleFormat,
    /// This chip's status-query command name — the first token of upstream's
    /// `setup_query_command(msgformat, …)`; its argument is always the
    /// sensor's `oid` (`oid=%c`).
    query_cmd: String,
}

impl FixedFreqReader {
    /// Track samples expected `chip_clock_smooth` MCU ticks apart — upstream
    /// passes `data_rate * BATCH_UPDATES * 2` — with ldc1612's defaults: the
    /// `">I"` sample format and the `query_status_ldc1612` status query
    /// (upstream's `FixedFreqReader(mcu, …, ">I")` +
    /// `setup_query_command("query_status_ldc1612 oid=%c", …)`).
    pub fn new(chip_clock_smooth: f64) -> Self {
        Self::build(
            chip_clock_smooth,
            SampleFormat::default(),
            QueryStatusLdc1612::NAME.to_string(),
        )
    }

    /// The parameterized constructor — upstream's `__init__(mcu,
    /// chip_clock_smooth, unpack_fmt)` together with the `msgformat` half of
    /// `setup_query_command(msgformat, oid, cq)`.
    ///
    /// `unpack_fmt` is a Python `struct` format (`"<i"`, `">I"`, `"BBBBB"`)
    /// that fixes the sample's byte order and size — hence
    /// [`samples_per_block`](SampleFormat::samples_per_block) =
    /// `51 // bytes_per_sample` — and `query_msgformat` is the chip's status
    /// query (e.g. `"query_hx71x_status oid=%c"`), whose first token names
    /// the command.
    ///
    /// # Errors
    /// [`ConfigError`] when either string is unusable (see
    /// [`SampleFormat::parse`]).
    pub fn with_format(
        chip_clock_smooth: f64,
        unpack_fmt: &str,
        query_msgformat: &str,
    ) -> Result<Self, ConfigError> {
        let format = SampleFormat::parse(unpack_fmt)?;
        let query_cmd = query_msgformat
            .split_whitespace()
            .next()
            .ok_or_else(|| {
                ConfigError::new(format!(
                    "the bulk status query message format \"{query_msgformat}\" names no command"
                ))
            })?
            .to_string();
        Ok(Self::build(chip_clock_smooth, format, query_cmd))
    }

    /// The shared constructor behind [`new`](Self::new) and
    /// [`with_format`](Self::with_format).
    fn build(chip_clock_smooth: f64, format: SampleFormat, query_cmd: String) -> Self {
        Self {
            mcu_object: Mutex::new(Weak::new()),
            oid: Mutex::new(None),
            queue: Arc::new(Mutex::new(Vec::new())),
            clock_sync: Mutex::new(ClockSyncRegression::new(chip_clock_smooth)),
            last_sequence: Mutex::new(0),
            max_query_duration: Mutex::new(0),
            last_overflows: Mutex::new(0),
            format,
            query_cmd,
        }
    }

    /// Route this sensor's `sensor_bulk_data` messages (upstream's
    /// `setup_query_command` queues half: the `BulkDataQueue`).
    pub fn bind(&self, mcu: &Mcu, mcu_object: &Arc<McuObject>, oid: u8) -> Result<(), McuError> {
        *self.oid.lock().unwrap_or_else(|p| p.into_inner()) = Some(oid);
        *self.mcu_object.lock().unwrap_or_else(|p| p.into_inner()) = Arc::downgrade(mcu_object);
        registry_for(mcu.name()).bind(mcu, oid, &self.queue)
    }

    /// Overflow messages reported since the last start
    /// (`get_last_overflows`).
    pub fn get_last_overflows(&self) -> u64 {
        *self
            .last_overflows
            .lock()
            .unwrap_or_else(|p| p.into_inner())
    }

    /// Start of a measurement session: reset counters and the queue, then
    /// take the first clock sample (upstream `note_start`).
    pub async fn note_start(&self) -> Result<(), McuError> {
        *self.last_sequence.lock().unwrap_or_else(|p| p.into_inner()) = 0;
        *self
            .last_overflows
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 0;
        self.queue.lock().unwrap_or_else(|p| p.into_inner()).clear();
        *self
            .max_query_duration
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 1 << 31;
        self.query_and_apply(true).await?;
        *self
            .max_query_duration
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = 1 << 31;
        Ok(())
    }

    /// End of a measurement session: drop the queued samples (upstream
    /// `note_end`).
    pub fn note_end(&self) {
        self.queue.lock().unwrap_or_else(|p| p.into_inner()).clear();
    }

    /// Connected pieces, or an error before the machine is up.
    fn connected(&self) -> Result<(Arc<Mcu>, Arc<McuObject>), McuError> {
        let object = self
            .mcu_object
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .upgrade()
            .ok_or_else(|| McuError::Config("the sensor's MCU is gone".to_string()))?;
        let mcu = object
            .mcu()
            .ok_or_else(|| McuError::Config("the sensor's MCU is not connected".to_string()))?;
        Ok((mcu, object))
    }

    fn oid(&self) -> Result<u8, McuError> {
        self.oid
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .ok_or_else(|| McuError::Config("the sensor is not configured yet".to_string()))
    }

    /// Query `sensor_bulk_status` through this chip's query command and fold
    /// it into the clock regression.
    async fn query_and_apply(&self, reset: bool) -> Result<(), McuError> {
        let (mcu, object) = self.connected()?;
        let oid = self.oid()?;
        let status = self.query_status(&mcu, oid).await?;
        let clock64 = object.clock32_to_clock64(status.clock).ok_or_else(|| {
            McuError::Config("the sensor's MCU clock is not synchronized".to_string())
        })?;
        let five_us = mcu.seconds_to_clock(0.000005)?.min(u64::from(u32::MAX)) as u32;
        self.apply_status(clock64, five_us, &status, reset);
        Ok(())
    }

    /// One status exchange through this chip's query command — upstream's
    /// `setup_query_command(msgformat, oid, cq)` looks the command up by its
    /// format string and `_update_clock` sends `[oid]` through it. Both names
    /// resolve before the command goes out, exactly as [`Mcu::call_msg`] does,
    /// so a firmware without the message fails fast instead of waiting out
    /// [`QUERY_TIMEOUT`].
    async fn query_status(&self, mcu: &Mcu, oid: u8) -> Result<SensorBulkStatus, McuError> {
        let dictionary = mcu.require_dictionary()?;
        mcu.require_message(&self.query_cmd)?;
        let response = mcu.require_message(SensorBulkStatus::NAME)?;
        let values = mcu
            .call(
                &self.query_cmd,
                &[ArgValue::UInt8(oid)],
                SensorBulkStatus::NAME,
                QUERY_TIMEOUT,
            )
            .await?;
        SensorBulkStatus::decode(&Params::new(response, &values).with_dictionary(dictionary))
    }

    /// One status observation: advance the counters, apply the duration
    /// filter, and reset or update the regression (upstream `_update_clock`).
    ///
    /// `mcu_clock64` is `status.clock` widened to 64 bits and `five_us` is
    /// `seconds_to_clock(.000005)` — both resolved by the caller so this core
    /// stays synchronous and testable.
    pub(crate) fn apply_status(
        &self,
        mcu_clock64: i64,
        five_us: u32,
        status: &SensorBulkStatus,
        reset: bool,
    ) {
        let mut last_sequence = self.last_sequence.lock().unwrap_or_else(|p| p.into_inner());
        let seq_diff = u16::wrapping_sub(status.next_sequence, *last_sequence as u16);
        *last_sequence += u64::from(seq_diff);

        let mut last_overflows = self
            .last_overflows
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let po_diff = u16::wrapping_sub(status.possible_overflows, *last_overflows as u16);
        *last_overflows += u64::from(po_diff);

        let mut max_query_duration = self
            .max_query_duration
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let duration = status.query_ticks;
        if duration > *max_query_duration {
            // Skip: a long query could skew the clock tracking.
            *max_query_duration = (2_u32.saturating_mul(*max_query_duration)).max(five_us);
            return;
        }
        *max_query_duration = 2_u32.saturating_mul(duration);

        let msg_count = *last_sequence * self.format.samples_per_block() as u64
            + u64::from(status.buffered) / self.format.bytes_per_sample() as u64;
        // +1 for the average query-response offset and assumed hardware
        // processing time (upstream's chip clock).
        let chip_clock = (msg_count + 1) as f64;
        let avg_mcu_clock = mcu_clock64 + i64::from(duration) / 2;
        let mut clock_sync = self.clock_sync.lock().unwrap_or_else(|p| p.into_inner());
        if reset {
            clock_sync.reset(avg_mcu_clock as f64, chip_clock);
        } else {
            clock_sync.update(avg_mcu_clock as f64, chip_clock);
        }
    }

    /// Query the clock, pull every queued message, and timestamp the samples
    /// (`pull_samples` upstream).
    pub async fn pull_samples(&self) -> Result<Vec<Sample>, McuError> {
        self.query_and_apply(false).await?;
        let raw: Vec<(u16, Vec<u8>)> = {
            let mut queue = self.queue.lock().unwrap_or_else(|p| p.into_inner());
            std::mem::take(&mut *queue)
        };
        if raw.is_empty() {
            return Ok(Vec::new());
        }
        let (_mcu, object) = self.connected()?;
        let clock = object.clock().ok_or_else(|| {
            McuError::Config("the sensor's MCU clock is not synchronized".to_string())
        })?;
        let last_sequence = *self.last_sequence.lock().unwrap_or_else(|p| p.into_inner());
        let (time_base, chip_base, inv_freq) = self
            .clock_sync
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get_time_translation(|ticks| clock.clock_to_print_time(ticks));
        let (samples, last_chip_clock) = decode_blocks(
            &raw,
            &self.format,
            last_sequence as i64,
            time_base,
            chip_base,
            inv_freq,
        )?;
        self.clock_sync
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .set_last_chip_clock(last_chip_clock);
        Ok(samples)
    }

    /// The current clock translation, for tests and diagnostics.
    #[cfg(test)]
    fn clock_translation(&self) -> (f64, f64, f64) {
        self.clock_sync
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get_clock_translation()
    }

    /// Messages reported so far, for tests.
    #[cfg(test)]
    fn last_sequence(&self) -> u64 {
        *self.last_sequence.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The active duration filter, for tests.
    #[cfg(test)]
    fn max_query_duration(&self) -> u32 {
        *self
            .max_query_duration
            .lock()
            .unwrap_or_else(|p| p.into_inner())
    }

    /// Seed the duration filter, for tests that skip `note_start` (which
    /// needs a connected MCU).
    #[cfg(test)]
    fn force_max_query_duration(&self, value: u32) {
        *self
            .max_query_duration
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = value;
    }
}

/// Decode queued `sensor_bulk_data` messages into timestamped samples
/// (the inner loop of upstream's `pull_samples`).
///
/// `start_last_sequence` is the sequence the status query established; each
/// message's sequence is taken relative to it with 16-bit wrap and signed
/// extension. Returns the samples and the chip-clock position of the last
/// sample (for [`ClockSyncRegression::set_last_chip_clock`]).
fn decode_blocks(
    raw: &[(u16, Vec<u8>)],
    format: &SampleFormat,
    start_last_sequence: i64,
    time_base: f64,
    chip_base: f64,
    inv_freq: f64,
) -> Result<(Vec<Sample>, f64), McuError> {
    let samples_per_block = format.samples_per_block();
    let bytes_per_sample = format.bytes_per_sample();
    let mut samples = Vec::with_capacity(raw.len() * samples_per_block);
    let mut last_sequence = start_last_sequence;
    let mut last_chip_clock = start_last_sequence as f64 * samples_per_block as f64;
    for (sequence, data) in raw {
        let seq_diff = u16::wrapping_sub(*sequence, last_sequence as u16);
        let signed = i64::from(seq_diff) - i64::from(seq_diff & 0x8000) * 2;
        let seq = last_sequence + signed;
        last_sequence = seq;
        let msg_cdiff = seq as f64 * samples_per_block as f64 - chip_base;
        let count = data.len() / bytes_per_sample;
        for index in 0..count {
            let ptime = time_base + (msg_cdiff + index as f64) * inv_freq;
            let start = index * bytes_per_sample;
            let raw_value = format.decode(&data[start..start + bytes_per_sample])?;
            samples.push((ptime, raw_value));
        }
        if count > 0 {
            last_chip_clock = seq as f64 * samples_per_block as f64 + (count - 1) as f64;
        }
    }
    Ok((samples, last_chip_clock))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn test_samples_per_block_matches_the_firmware_message_size() {
        // 51 bytes per message, 4 bytes per sample → 12 samples per block,
        // exactly upstream's integer division.
        assert_eq!(MAX_BULK_MSG_SIZE, 51);
        assert_eq!(BYTES_PER_SAMPLE, 4);
        assert_eq!(SAMPLES_PER_BLOCK, 12);
        // The default reader carries that same format.
        let reader = FixedFreqReader::new(80.);
        assert_eq!(reader.format.bytes_per_sample(), BYTES_PER_SAMPLE);
        assert_eq!(reader.format.samples_per_block(), SAMPLES_PER_BLOCK);
        assert_eq!(reader.query_cmd, QueryStatusLdc1612::NAME);
    }

    #[test]
    fn test_sample_format_parses_sizes_byte_order_and_rejections() {
        // "<i": little-endian, four bytes → 51 // 4 = 12 per block.
        let le = SampleFormat::parse("<i").expect("valid format");
        assert_eq!(le.bytes_per_sample(), 4);
        assert_eq!(le.samples_per_block(), 12);

        // "BBBBB": five one-byte fields → 51 // 5 = 10 per block
        // (upstream adxl345's format; the old constant 12 was wrong here).
        let five = SampleFormat::parse("BBBBB").expect("valid format");
        assert_eq!(five.bytes_per_sample(), 5);
        assert_eq!(five.samples_per_block(), MAX_BULK_MSG_SIZE / 5);
        assert_eq!(five.samples_per_block(), 10);

        // The ldc1612 default parses to the same layout as `default()`.
        assert_eq!(
            SampleFormat::parse(">I").expect("valid"),
            SampleFormat::default()
        );
        // "hhh": three 2-byte fields, both byte orders.
        assert_eq!(
            SampleFormat::parse(">hhh")
                .expect("valid")
                .bytes_per_sample(),
            6
        );
        assert_eq!(
            SampleFormat::parse("<hhh")
                .expect("valid")
                .bytes_per_sample(),
            6
        );

        // Rejected: empty, no fields, unknown code.
        assert!(SampleFormat::parse("").is_err());
        assert!(SampleFormat::parse(">").is_err());
        assert!(SampleFormat::parse(">z").is_err());
    }

    #[test]
    fn test_little_endian_i32_block_decodes_correctly() {
        // "<i": four-byte little-endian i32 samples (upstream hx71x/ads1220).
        let format = SampleFormat::parse("<i").expect("valid format");
        let mut block = Vec::new();
        for value in [0x0010_0000_i32, -1, 0x1234_5678] {
            block.extend_from_slice(&value.to_le_bytes());
        }
        let raw = vec![(1_u16, block)];
        let (samples, _) = decode_blocks(&raw, &format, 1, 10.0, 13.0, 0.5).expect("decodes");
        assert_eq!(samples.len(), 3);
        assert_eq!(samples[0].1, 0x0010_0000);
        // -1 as i32 is all bits set; little-endian must not byte-swap it.
        assert_eq!(samples[1].1, 0xFFFF_FFFF);
        // 0x12345678 LE is bytes 78 56 34 12; big-endian would read 0x78563412.
        assert_eq!(samples[2].1, 0x1234_5678);
        assert_ne!(samples[2].1, 0x7856_3412);
    }

    #[test]
    fn test_default_reader_decodes_byte_like_the_fixed_be_path() {
        // Byte equivalence with the pre-parameterization reader: the same
        // ldc1612 samples decoded by `u32::from_be_bytes` over 4-byte windows.
        let mut block = Vec::new();
        for value in 0..12_u32 {
            block.extend_from_slice(&(0x0010_0000 + value).to_be_bytes());
        }
        let expected: Vec<u32> = block
            .chunks_exact(4)
            .map(|window| u32::from_be_bytes(window.try_into().expect("4 bytes")))
            .collect();

        let reader = FixedFreqReader::new(80.);
        assert_eq!(reader.format, SampleFormat::default());
        let raw = vec![(1_u16, block)];
        let (samples, _) =
            decode_blocks(&raw, &reader.format, 1, 10.0, 13.0, 0.5).expect("decodes");
        let decoded: Vec<u32> = samples.iter().map(|(_t, value)| *value).collect();
        assert_eq!(decoded, expected);

        // The default construction still names ldc1612's query command.
        assert_eq!(reader.query_cmd, "query_status_ldc1612");
        // …and an injected format reuses the same decode path.
        let injected = FixedFreqReader::with_format(80., ">I", "query_status_ldc1612 oid=%c")
            .expect("valid args");
        assert_eq!(injected.format, reader.format);
        assert_eq!(injected.query_cmd, reader.query_cmd);
    }

    #[test]
    fn test_with_format_injects_query_command_and_rejects_bad_input() {
        let reader = FixedFreqReader::with_format(80., "<i", "query_hx71x_status oid=%c")
            .expect("valid args");
        assert_eq!(reader.query_cmd, "query_hx71x_status");
        assert_eq!(reader.format.bytes_per_sample(), 4);
        assert_eq!(reader.format.samples_per_block(), 12);
        // The msgformat's first token names the command (upstream's
        // `lookup_query_command` splits it the same way).
        assert!(FixedFreqReader::with_format(80., "<i", "   ").is_err());
        assert!(FixedFreqReader::with_format(80., "zzz", "query_x_status oid=%c").is_err());
    }

    #[test]
    fn test_sample_format_decode_rejects_what_sample_cannot_carry() {
        // "BBBBB" sizes the block (10) but its 5-byte tuple cannot be one
        // (f64, u32) raw value — the honest error instead of a silent pack.
        let five = SampleFormat::parse("BBBBB").expect("valid format");
        assert!(five.decode(&[0_u8; 5]).is_err());
        // Width mismatch is refused too.
        let be = SampleFormat::default();
        assert!(be.decode(&[0_u8; 3]).is_err());
        assert_eq!(
            be.decode(&[0x12, 0x34, 0x56, 0x78]).expect("4 bytes"),
            0x1234_5678
        );
    }

    #[test]
    fn test_clock_sync_recovers_a_linear_sample_rate() {
        // 16 MHz MCU, one sample per 40000 ticks (400 Hz).
        let mut sync = ClockSyncRegression::new(80.);
        sync.reset(1_000_000., 1.);
        sync.update(1_000_000. + 40_000., 2.);
        let (_base_mcu, base_chip, inv_cfreq) = sync.get_clock_translation();
        // One update from a zero-variance reset: the translation is exactly
        // the observation's slope, and the centre moves 5% toward it.
        assert!((inv_cfreq - 40_000.).abs() < 1e-9);
        assert_eq!(base_chip, 1.05);

        sync.set_last_chip_clock(13.);
        let (base_mcu, base_chip, inv_cfreq) = sync.get_clock_translation();
        // The anchor is projected one smoothing window into the future.
        assert_eq!(base_chip, 93.);
        assert!((inv_cfreq - 40_000.).abs() < 1e-6);

        let mcu_freq = 16_000_000.;
        let (base_time, chip, inv_freq) =
            sync.get_time_translation(|ticks| ticks as f64 / mcu_freq);
        assert_eq!(chip, 93.);
        // One sample every 40000 / 16 MHz = 0.0025 s.
        assert!((inv_freq - 0.0025).abs() < 1e-9);
        assert_eq!(base_time, (base_mcu.round() as i64) as f64 / mcu_freq);
    }

    #[test]
    fn test_decode_blocks_slices_batches_and_times_them() {
        // Two full messages: 12 samples each, big-endian 32-bit values.
        let mut first = Vec::new();
        for value in 0..12_u32 {
            first.extend_from_slice(&(0x0010_0000 + value).to_be_bytes());
        }
        let mut second = Vec::new();
        for value in 12..24_u32 {
            second.extend_from_slice(&(0x0010_0000 + value).to_be_bytes());
        }
        let raw = vec![(1_u16, first), (2_u16, second)];
        let (samples, last_chip_clock) =
            decode_blocks(&raw, &SampleFormat::default(), 1, 10.0, 13.0, 0.5).expect("decodes");
        // 24 samples; message 1 is seq 1 (baseline), message 2 follows.
        assert_eq!(samples.len(), 24);
        assert_eq!(samples[0], (10.0 + (12.0 - 13.0) * 0.5, 0x0010_0000));
        assert_eq!(samples[11].0, 10.0 + (12.0 - 13.0 + 11.0) * 0.5);
        assert_eq!(samples[12].0, 10.0 + (24.0 - 13.0) * 0.5);
        assert_eq!(samples[23].1, 0x0010_0000 + 23);
        // The last sample's chip position: seq 2 * 12 + 11.
        assert_eq!(last_chip_clock, 35.0);

        // An empty pull decodes to nothing.
        let (empty, _) =
            decode_blocks(&[], &SampleFormat::default(), 0, 0., 0., 1.).expect("decodes");
        assert!(empty.is_empty());
    }

    #[test]
    fn test_decode_blocks_handles_sequence_wrap_and_back_jump() {
        let block = vec![0_u8; BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK];
        // Forward across the 16-bit wrap: 65534 → 1 is +3 messages.
        let raw = vec![(65534_u16, block.clone()), (1_u16, block.clone())];
        let (samples, last_chip) =
            decode_blocks(&raw, &SampleFormat::default(), 65534, 0., 0., 1.).expect("decodes");
        assert_eq!(samples.len(), 2 * SAMPLES_PER_BLOCK);
        // chip positions: 65534*12+12 → 65537*12+11 (baseline arithmetic).
        assert_eq!(last_chip, 65537.0 * 12.0 + 11.0);

        // A backwards jump: 0x8000 is -32768 signed (upstream's extension).
        let raw = vec![(0x8000_u16, block)];
        let (samples, last_chip) =
            decode_blocks(&raw, &SampleFormat::default(), 0, 0., 0., 1.).expect("decodes");
        assert_eq!(last_chip, -32768.0 * 12.0 + 11.0);
        assert_eq!(samples.len(), SAMPLES_PER_BLOCK);
    }

    fn status(next_sequence: u16, query_ticks: u32, buffered: u32) -> SensorBulkStatus {
        SensorBulkStatus {
            oid: 0,
            clock: 1_000,
            query_ticks,
            next_sequence,
            buffered,
            possible_overflows: 0,
        }
    }

    #[test]
    fn test_apply_status_accumulates_sequences_across_wraps() {
        let reader = FixedFreqReader::new(80.);
        // A fresh reader's duration filter is 0 (upstream clears it in
        // `note_start`, which needs a connected MCU); seed it so the first
        // status is not skipped.
        reader.force_max_query_duration(1_000);
        // 65534 messages reported: msg_count = 65534*12 + 24/4, chip +1.
        reader.apply_status(1_000_000, 5, &status(65_534, 1_000, 24), true);
        assert_eq!(reader.last_sequence(), 65_534);
        let (_, chip, _) = reader.clock_translation();
        assert_eq!(chip, 65_534.0 * 12.0 + 6.0 + 1.0);

        // The 16-bit sequence wraps: 65534 → 2 is +4 messages.
        reader.apply_status(1_040_000, 5, &status(2, 1_000, 0), false);
        assert_eq!(reader.last_sequence(), 65_538);
        let (_, _, inv) = reader.clock_translation();
        // One observation folded in from a clean reset: a real slope.
        assert!(inv > 0., "the regression has a slope: {inv}");
    }

    #[test]
    fn test_apply_status_duration_filter_skews_not_the_clock() {
        let reader = FixedFreqReader::new(80.);
        reader.force_max_query_duration(1_000);
        reader.apply_status(1_000_000, 80, &status(1, 1_000, 0), true);
        assert_eq!(reader.max_query_duration(), 2_000);
        let (_, chip_before, _) = reader.clock_translation();

        // A query longer than the filter: the sequence counters still
        // advance, but the regression is untouched.
        reader.apply_status(1_500_000, 80, &status(9, 5_000, 0), false);
        assert_eq!(reader.last_sequence(), 9);
        assert_eq!(reader.max_query_duration(), 4_000);
        let (_, chip_after, _) = reader.clock_translation();
        assert_eq!(chip_before, chip_after);
    }

    #[tokio::test(start_paused = true)]
    async fn test_batch_helper_starts_with_the_first_client_and_reports_batches() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let starts = Arc::new(AtomicUsize::new(0));
        let stops = Arc::new(AtomicUsize::new(0));
        let batches = Arc::new(AtomicUsize::new(0));

        let count = Arc::clone(&batches);
        let batch_cb: BatchCb = Arc::new(move |_eventtime| {
            let count = Arc::clone(&count);
            Box::pin(async move {
                count.fetch_add(1, Ordering::SeqCst);
                Ok(Some(json!({ "n": count.load(Ordering::SeqCst) })))
            })
        });
        let start_count = Arc::clone(&starts);
        let start_cb: LifecycleCb = Arc::new(move || {
            let start_count = Arc::clone(&start_count);
            Box::pin(async move {
                start_count.fetch_add(1, Ordering::SeqCst);
                Ok(())
            })
        });
        let stop_count = Arc::clone(&stops);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_count = Arc::clone(&stop_count);
            Box::pin(async move {
                stop_count.fetch_add(1, Ordering::SeqCst);
                Ok(())
            })
        });
        let helper = BatchBulkHelper::new(&printer, batch_cb, start_cb, stop_cb, 0.05);

        let received = Arc::new(Mutex::new(Vec::<Value>::new()));
        let got = Arc::clone(&received);
        helper.add_client(Arc::new(move |message: &Value| {
            got.lock().unwrap().push(message.clone());
            true
        }));
        // Auto-advancing time: several intervals pass.
        tokio::time::sleep(Duration::from_secs_f64(0.16)).await;

        assert_eq!(starts.load(Ordering::SeqCst), 1, "started once");
        assert!(batches.load(Ordering::SeqCst) >= 2, "batches on schedule");
        let received = received.lock().unwrap();
        assert_eq!(received.len(), batches.load(Ordering::SeqCst));
        assert_eq!(received[0]["n"], 1);
        assert_eq!(stops.load(Ordering::SeqCst), 0, "still running");
        assert!(helper.is_running());
    }

    #[tokio::test(start_paused = true)]
    async fn test_batch_helper_stops_with_the_last_client() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let stops = Arc::new(AtomicUsize::new(0));
        let batch_cb: BatchCb = Arc::new(|_| Box::pin(async { Ok(Some(json!({ "x": 1 }))) }));
        let start_cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        let stop_count = Arc::clone(&stops);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_count = Arc::clone(&stop_count);
            Box::pin(async move {
                stop_count.fetch_add(1, Ordering::SeqCst);
                Ok(())
            })
        });
        let helper = BatchBulkHelper::new(&printer, batch_cb, start_cb, stop_cb, 0.05);

        let seen = Arc::new(AtomicUsize::new(0));
        let seen_client = Arc::clone(&seen);
        // Unregister after the second batch, which empties the list and stops
        // the helper (upstream's client returning false).
        helper.add_client(Arc::new(move |_message: &Value| {
            seen_client.fetch_add(1, Ordering::SeqCst) + 1 < 2
        }));
        tokio::time::sleep(Duration::from_secs_f64(0.3)).await;

        assert_eq!(seen.load(Ordering::SeqCst), 2, "exactly two batches");
        assert_eq!(stops.load(Ordering::SeqCst), 1, "stopped once");
        assert_eq!(helper.client_count(), 0);
        assert!(!helper.is_running());
    }

    #[tokio::test(start_paused = true)]
    async fn test_batch_helper_stop_detaches_the_clients_and_the_loop_finishes() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let stops = Arc::new(AtomicUsize::new(0));
        let batch_cb: BatchCb = Arc::new(|_| Box::pin(async { Ok(Some(json!({ "x": 1 }))) }));
        let start_cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        let stop_count = Arc::clone(&stops);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_count = Arc::clone(&stop_count);
            Box::pin(async move {
                stop_count.fetch_add(1, Ordering::SeqCst);
                Ok(())
            })
        });
        let helper = BatchBulkHelper::new(&printer, batch_cb, start_cb, stop_cb, 0.05);

        helper.add_client(Arc::new(|_message: &Value| true));
        assert_eq!(helper.client_count(), 1);
        // `stop` marks the helper detached and empties the client list
        // synchronously; `running` stays true until the loop's next tick hands
        // over to `finish`.
        helper.stop();
        assert_eq!(helper.client_count(), 0);
        assert!(helper.is_running(), "`stop` does not touch `running`");
        // A detached helper takes no new client: a request routed to it just
        // before the reload is answered as an unknown endpoint instead.
        assert!(
            !helper.add_client(Arc::new(|_message: &Value| true)),
            "a detached helper refuses clients"
        );
        assert_eq!(helper.client_count(), 0);

        // A few intervals: the loop wakes, sees no client, and finishes.
        tokio::time::sleep(Duration::from_secs_f64(0.2)).await;
        assert!(!helper.is_running());
        // The stop callback is skipped — the configuration it would stop is gone.
        assert_eq!(
            stops.load(Ordering::SeqCst),
            0,
            "detach skips the stop callback"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn test_batch_mux_endpoint_detach_stops_the_helpers_clients() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let stops = Arc::new(AtomicUsize::new(0));
        let batch_cb: BatchCb = Arc::new(|_| Box::pin(async { Ok(Some(json!({ "x": 1 }))) }));
        let start_cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        let stop_count = Arc::clone(&stops);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_count = Arc::clone(&stop_count);
            Box::pin(async move {
                stop_count.fetch_add(1, Ordering::SeqCst);
                Ok(())
            })
        });
        let helper = BatchBulkHelper::new(&printer, batch_cb, start_cb, stop_cb, 0.05);
        let endpoint = MuxBatchEndpoint {
            bulk: Arc::clone(&helper),
            start_resp: json!({ "header": "#x" }),
            key: "sensor".to_string(),
            value: "a".to_string(),
        };

        helper.add_client(Arc::new(|_message: &Value| true));
        assert_eq!(helper.client_count(), 1);
        endpoint.detach();
        assert_eq!(helper.client_count(), 0, "detach drops the registration");

        tokio::time::sleep(Duration::from_secs_f64(0.2)).await;
        assert_eq!(
            stops.load(Ordering::SeqCst),
            0,
            "a detached helper does not run the stop callback"
        );
        assert!(!helper.is_running());
    }

    #[tokio::test(start_paused = true)]
    async fn test_a_detached_mux_endpoint_answers_unknown_mux_value() {
        // The window the `detached` flag closes: dispatch resolved this instance
        // just before a reload dropped its registration, so the request lands on
        // a helper that must not take a client. Answering as the emptied table
        // would makes the client retry instead of waiting on a stream that will
        // never run.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let batch_cb: BatchCb = Arc::new(|_| Box::pin(async { Ok(Some(json!({ "x": 1 }))) }));
        let start_cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        let stop_cb: LifecycleCb = Arc::new(|| Box::pin(async { Ok(()) }));
        let helper = BatchBulkHelper::new(&printer, batch_cb, start_cb, stop_cb, 0.05);
        let endpoint = MuxBatchEndpoint {
            bulk: Arc::clone(&helper),
            start_resp: json!({ "header": "#x" }),
            key: "sensor".to_string(),
            value: "a".to_string(),
        };
        endpoint.detach();

        let api = crate::core::klippy::api::registry::Api::new();
        let request = Request::parse(br#"{"method":"sensors/dump","params":{"sensor":"a"}}"#)
            .expect("test body is a valid request");
        let context = EndpointContext {
            api: &api,
            client: crate::core::klippy::api::test_support::silent_target(),
        };
        let err = endpoint
            .handle(&request, &context)
            .await
            .expect_err("a detached instance takes no client");
        assert_eq!(
            err,
            ApiError::UnknownMuxValue {
                key: "sensor".to_string(),
                value: "a".to_string(),
            }
        );
        assert_eq!(helper.client_count(), 0);
    }
}
