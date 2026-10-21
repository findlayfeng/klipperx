//! `[angle <name>]` — SPI magnetic angle sensors (upstream's
//! `klippy/extras/angle.py`, with `src/sensor_angle.c`).
//!
//! | piece | upstream |
//! |---|---|
//! | [`Angle`] — `sensor_type`, `sample_period`, SPI via `MCU_SPI_from_config` | `Angle.__init__` (`angle.py:610-651`) |
//! | [`ChipType`] — the five `spi_angle_type` chips, their SPI mode/speed/static delay | the `Helper*` classes' `SPI_MODE`/`SPI_SPEED`/`get_static_delay` |
//! | [`start_frames`] — what `sensor_helper.start()` clocks out before arming | `HelperA1333.start`, `HelperAS5047D.start` |
//! | [`ExtractState::extract_samples`] — the sample decode | `Angle._extract_samples` (`:663-717`) |
//! | [`time_shift_for`] — the time-code shift | `Angle._build_config` (`:653-657`) |
//! | `angle/dump_angle` mux endpoint (key `sensor`) | `add_mux_endpoint` (`:648-650`) |
//! | `config_spi_angle` / `query_spi_angle` | [`crate::core::klippy::cmd::angle`] |
//!
//! # The sample path
//!
//! The firmware queries the chip itself and reports three bytes per sample
//! (`angle_add`, `sensor_angle.c:82-89`): a **time code**, then the 16-bit angle
//! little-endian. A time code of `0xff` marks an error sample (`SE_*`); any
//! other value is the SPI transaction's lateness in `1 << time_shift` firmware
//! ticks, which [`ExtractState`] folds into the sample's timestamp.
//!
//! That decode is why this sensor reads the raw `sensor_bulk_data` blocks
//! through [`BulkDataQueue`] instead of the shared
//! [`FixedFreqReader`](super::bulk_sensor::FixedFreqReader): the reader
//! timestamps samples from a bulk *status* clock, and an angle sensor has no
//! status query command at all (`src/sensor_angle.c` declares only
//! `config_spi_angle`, `query_spi_angle` and `spi_angle_transfer`). Upstream
//! does the same (`angle.py:641`).
//!
//! As upstream, the angle itself is reported as a running value: each sample's
//! 16-bit word is turned into a signed difference against the previous sample
//! and accumulated (`last_angle`), so a chip that reports a discontinuity does
//! not tear the stream.
//!
//! # Not implemented
//!
//! - `tle5012b` and `mt6826s` (two of the five `sensor_type` chips). The
//!   `tle5012b` helper needs its CRC-8/J1850 SPI register traffic, a
//!   chip-clock estimate that replaces the time code
//!   (`HelperTLE5012B.update_clock`) and `ANGLE_DEBUG_READ`/`ANGLE_DEBUG_WRITE`;
//!   `mt6826s` needs its own CRC-8 register reads and the
//!   `ANGLE_CHIP_CALIBRATE` motor sequence. Selecting either is a config error
//!   that names what is missing.
//! - `AngleCalibration` (`angle.py:19-292`): `ANGLE_CALIBRATE`, the numpy
//!   least-squares bucket fit, the TMC phase lookup and the calibration moves.
//!   A section that sets `stepper` says so instead of being silently ignored;
//!   without it the batch message's `position_offset` is `null`, which is what
//!   upstream reports for an uncalibrated sensor too.
//! - `get_status`' `temperature` is `null` here: only the `tle5012b` helper ever
//!   sets `last_temperature` (`angle.py:373`), and that chip is deferred.
//!   Upstream would raise `AttributeError` on a helper without the attribute;
//!   a status read must not fail, so this host reports the absent channel as
//!   `null` instead.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::angle::{ConfigSpiAngle, QuerySpiAngle};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::bulk_sensor::{
    BatchBulkHelper, BatchCb, BulkDataQueue, LifecycleCb, MAX_BULK_MSG_SIZE,
};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError, McuObject, McuSpi, SendClocks};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Loaded after `[board_pins]` (order 40) so an alias or a software bus can name
// the CS pin, alongside the other SPI devices.
section!("angle", order = 40, prefix = load_config_prefix);

/// The endpoint clients stream samples from (`angle/dump_angle`).
pub const DUMP_ENDPOINT: &str = "angle/dump_angle";

/// The mux key that selects a sensor instance.
pub const DUMP_KEY: &str = "sensor";

/// The columns the dump endpoint's start response advertises
/// (`{'header': ('time', 'angle')}`, `angle.py:647`).
pub const DUMP_HEADER: [&str; 2] = ["time", "angle"];

/// The time code value that marks an error sample (`TCODE_ERROR`,
/// `angle.py:9` / `sensor_angle.c:26`).
const TCODE_ERROR: u8 = 0xff;

/// Bytes one sample occupies in a bulk message (`angle.py:604`).
const BYTES_PER_SAMPLE: usize = 3;

/// Samples one full bulk message carries (`angle.py:605`).
const SAMPLES_PER_BLOCK: usize = MAX_BULK_MSG_SIZE / BYTES_PER_SAMPLE;

/// Default seconds between samples (`SAMPLE_PERIOD`, `angle.py:606`).
const DEFAULT_SAMPLE_PERIOD: f64 = 0.000400;

/// Seconds between batch callbacks (`BATCH_UPDATES`, `angle.py:607`).
const ANGLE_BATCH_UPDATES: f64 = 0.100;

/// How far ahead of the estimated print time the first sample is scheduled
/// (`MIN_MSG_TIME`, `angle.py:8`).
const MIN_MSG_TIME: f64 = 0.100;

/// The time code must be able to express this much lateness
/// (`_build_config`'s `0.002`).
const MIN_TIME_CODE_SPAN: f64 = 0.002;

// ===========================================================================
// Chip types
// ===========================================================================

/// The SPI angle chips the firmware can query (`src/sensor_angle.c`'s
/// `SA_CHIP_*` / `spi_angle_type` enumeration).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipType {
    /// `a1333` — `HelperA1333`.
    A1333,
    /// `as5047d` — `HelperAS5047D`.
    As5047d,
    /// `tle5012b` — `HelperTLE5012B` (deferred, see the module docs).
    Tle5012b,
    /// `mt6816` — `HelperMT6816`.
    Mt6816,
    /// `mt6826s` — `HelperMT6826S` (deferred, see the module docs).
    Mt6826s,
}

impl ChipType {
    /// Every `sensor_type` the firmware's enumeration accepts, in its order
    /// (`angle.py:619-624`'s `sensors` table).
    pub const ALL: [ChipType; 5] = [
        ChipType::A1333,
        ChipType::As5047d,
        ChipType::Tle5012b,
        ChipType::Mt6816,
        ChipType::Mt6826s,
    ];

    /// The `sensor_type` name, which is also the enumeration's name.
    pub fn name(self) -> &'static str {
        match self {
            ChipType::A1333 => "a1333",
            ChipType::As5047d => "as5047d",
            ChipType::Tle5012b => "tle5012b",
            ChipType::Mt6816 => "mt6816",
            ChipType::Mt6826s => "mt6826s",
        }
    }

    /// The chip by its `sensor_type` name.
    pub fn from_name(name: &str) -> Option<ChipType> {
        ChipType::ALL.into_iter().find(|chip| chip.name() == name)
    }

    /// The `spi_angle_type` ordinal `config_spi_angle` carries
    /// (`src/sensor_angle.c`'s `SA_CHIP_*` order).
    pub fn firmware_value(self) -> u8 {
        match self {
            ChipType::A1333 => 0,
            ChipType::As5047d => 1,
            ChipType::Tle5012b => 2,
            ChipType::Mt6816 => 3,
            ChipType::Mt6826s => 4,
        }
    }

    /// The helper class's `SPI_MODE` (`MCU_SPI_from_config`'s default mode).
    fn spi_mode(self) -> u8 {
        match self {
            ChipType::A1333 => 3,
            ChipType::As5047d => 1,
            ChipType::Tle5012b => 1,
            ChipType::Mt6816 => 3,
            ChipType::Mt6826s => 3,
        }
    }

    /// The helper class's `SPI_SPEED`, as the default `spi_speed`.
    fn default_spi_speed(self) -> u32 {
        match self {
            // `int(1. / .000000350)`.
            ChipType::As5047d => 2_857_142,
            ChipType::Tle5012b => 4_000_000,
            ChipType::A1333 | ChipType::Mt6816 | ChipType::Mt6826s => 10_000_000,
        }
    }

    /// The helper class's `get_static_delay`: how long after the SPI transfer a
    /// sample's timestamp is (`_extract_samples`'s `static_delay`).
    fn static_delay(self) -> f64 {
        match self {
            ChipType::As5047d => 0.000100,
            ChipType::Mt6826s => 0.00001,
            ChipType::A1333 | ChipType::Tle5012b | ChipType::Mt6816 => 0.000001,
        }
    }

    /// What this build still owes for a chip, or `None` when it is implemented.
    fn deferred_because(self) -> Option<&'static str> {
        match self {
            ChipType::A1333 | ChipType::As5047d | ChipType::Mt6816 => None,
            ChipType::Tle5012b => Some(
                "its CRC-8/J1850 SPI register traffic, the chip-clock estimate that replaces \
                 the time code, and ANGLE_DEBUG_READ/ANGLE_DEBUG_WRITE",
            ),
            ChipType::Mt6826s => {
                Some("its CRC-8 register reads and the ANGLE_CHIP_CALIBRATE motor sequence")
            }
        }
    }
}

/// The transfers `sensor_helper.start()` clocks out before the bulk stream is
/// armed: `HelperAS5047D` clears any errors from the device, `HelperA1333`
/// sets the chip up for angle queries, and `HelperMT6816.start` is a no-op
/// (`angle.py:296-300`, `:307-312`, `:486-487`).
fn start_frames(chip: ChipType) -> &'static [&'static [u8]] {
    match chip {
        ChipType::A1333 => &[&[0x32, 0x00]],
        // Read DIAAGC, read ERRFL, read NOP.
        ChipType::As5047d => &[&[0xff, 0xfc], &[0x40, 0x01], &[0xc0, 0x00]],
        ChipType::Tle5012b | ChipType::Mt6816 | ChipType::Mt6826s => &[],
    }
}

/// The smallest `time_shift` whose time code spans at least
/// [`MIN_TIME_CODE_SPAN`] seconds (`_build_config`'s loop).
fn time_shift_for(mcu_freq: f64) -> u8 {
    let mut shift = 0u8;
    while f64::from(TCODE_ERROR) * f64::powi(2., i32::from(shift)) / mcu_freq < MIN_TIME_CODE_SPAN {
        shift += 1;
    }
    shift
}

// ===========================================================================
// Sample decoding
// ===========================================================================

/// One decoded sample of a batch message: `[time, angle]`
/// (`(ptime, last_angle)` upstream).
pub type AngleSample = [f64; 2];

/// The bulk stream's rolling decode state: where the firmware started sampling,
/// how far apart the samples are, and where the last angle left off
/// (`Angle.start_clock` / `sample_ticks` / `time_shift` / `last_sequence` /
/// `last_angle`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExtractState {
    /// Firmware clock of the first scheduled sample (`start_clock`).
    start_clock: i64,
    /// Firmware ticks between samples (`sample_ticks`).
    sample_ticks: i64,
    /// How many bits of a sample's time code are its clock offset
    /// (`time_shift`).
    time_shift: u8,
    /// Blocks seen so far, accumulated across 16-bit wraps (`last_sequence`).
    last_sequence: u64,
    /// The running angle (`last_angle`).
    last_angle: i64,
}

impl ExtractState {
    /// A state for a stream sampled every `sample_ticks` firmware ticks with a
    /// time code of `time_shift` bits.
    pub fn new(sample_ticks: i64, time_shift: u8) -> Self {
        Self {
            start_clock: 0,
            sample_ticks,
            time_shift,
            last_sequence: 0,
            last_angle: 0,
        }
    }

    /// Begin a measurement session at `start_clock` (`_start_measurements`'
    /// `last_sequence = 0`; the running angle carries over, as upstream's
    /// does).
    pub fn note_start(&mut self, start_clock: i64) {
        self.start_clock = start_clock;
        self.last_sequence = 0;
    }

    /// The running angle the next sample continues from (`last_angle`).
    pub fn last_angle(&self) -> i64 {
        self.last_angle
    }

    /// Decode one batch of raw blocks (`Angle._extract_samples`).
    ///
    /// Each block is `(sequence, data)`; `data` is a whole number of
    /// [`BYTES_PER_SAMPLE`] samples. A sample whose time code is
    /// [`TCODE_ERROR`] is dropped and counted. A surviving sample's timestamp is
    /// its scheduled clock plus the time code's offset, converted with
    /// `clock_to_print_time` and shifted back by `static_delay`.
    ///
    /// Returns the samples and how many error samples were dropped.
    pub fn extract_samples<F>(
        &mut self,
        raw: &[(u16, Vec<u8>)],
        clock_to_print_time: F,
        static_delay: f64,
    ) -> (Vec<AngleSample>, u64)
    where
        F: Fn(i64) -> f64,
    {
        let mut samples = Vec::with_capacity(raw.len() * SAMPLES_PER_BLOCK);
        let mut error_count = 0u64;
        for (sequence, data) in raw {
            // The block's place in the stream, taken relative to the previous
            // one with 16-bit wrap (`seq_diff = (sequence - last_sequence) &
            // 0xffff`).
            let seq_diff = (i64::from(*sequence) - self.last_sequence as i64) & 0xffff;
            self.last_sequence += seq_diff as u64;
            let msg_mclock = self.start_clock
                + self.last_sequence as i64 * SAMPLES_PER_BLOCK as i64 * self.sample_ticks;
            let count = data.len() / BYTES_PER_SAMPLE;
            for index in 0..count {
                let base = index * BYTES_PER_SAMPLE;
                let tcode = data[base];
                if tcode == TCODE_ERROR {
                    error_count += 1;
                    continue;
                }
                let raw_angle = i64::from(data[base + 1]) | (i64::from(data[base + 2]) << 8);
                // The chip's 16-bit word is a signed difference against the
                // running angle, so a wrap is not a jump.
                let mut angle_diff = (raw_angle - self.last_angle) & 0xffff;
                angle_diff -= (angle_diff & 0x8000) << 1;
                self.last_angle += angle_diff;
                let mclock = msg_mclock + index as i64 * self.sample_ticks;
                let sclock = mclock + (i64::from(tcode) << self.time_shift);
                let ptime = round(clock_to_print_time(sclock) - static_delay, 6);
                samples.push([ptime, self.last_angle as f64]);
            }
        }
        (samples, error_count)
    }
}

/// Python's `round(value, digits)` for the sample timestamps.
fn round(value: f64, digits: i32) -> f64 {
    let scale = 10_f64.powi(digits);
    (value * scale).round() / scale
}

/// The start response the dump endpoint advertises
/// (`api_resp = {'header': ('time', 'angle')}`).
fn dump_start_response() -> Value {
    json!({ "header": DUMP_HEADER })
}

// ===========================================================================
// The sensor
// ===========================================================================

/// One configured angle sensor (an `[angle <name>]` section).
pub struct Angle {
    state: Arc<AngleState>,
}

impl Angle {
    /// Read the section and wire the sensor up (`Angle.__init__`).
    ///
    /// # Errors
    /// An unknown or deferred `sensor_type`, a non-positive `sample_period`, a
    /// `stepper` (calibration is deferred), a missing MCU or CS pin, an oid
    /// shortage, or a mux endpoint collision.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| identifier.clone());

        let choices: Vec<&str> = ChipType::ALL.iter().map(|chip| chip.name()).collect();
        let sensor_type = config.get_choice("sensor_type", &choices, Some("as5047d"))?;
        let chip = ChipType::from_name(&sensor_type).ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}': unknown angle sensor_type '{sensor_type}'"
            ))
        })?;
        if let Some(missing) = chip.deferred_because() {
            return Err(ConfigError::new(format!(
                "Section '{identifier}': angle sensor_type '{}' is not implemented: {missing}",
                chip.name()
            )));
        }
        let sample_period = config.get_float_bounded(
            "sample_period",
            Some(DEFAULT_SAMPLE_PERIOD),
            None,
            None,
            Some(0.),
            None,
        )?;

        // `AngleCalibration` would take this over; without it the option has no
        // meaning, so say so rather than leaving it unread.
        if config.get_str("stepper").is_some() {
            return Err(ConfigError::new(format!(
                "Section '{identifier}': 'stepper' (ANGLE_CALIBRATE) is not implemented"
            )));
        }

        let setup = mcu_spi_from_config(
            config,
            printer,
            chip.spi_mode(),
            "cs_pin",
            chip.default_spi_speed(),
        )?;
        let spi = setup.device;

        let mcu_name = config
            .get_str("spi_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&mcu_name))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
            })?;
        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(AngleState {
            name,
            chip,
            sample_period,
            oid,
            spi,
            mcu_object: Arc::clone(&mcu_object),
            queue: BulkDataQueue::new(),
            extract: Mutex::new(ExtractState::new(0, 0)),
            printer: Arc::downgrade(printer),
            batch_bulk: Mutex::new(None),
        });

        // The build callback must follow the SPI resource's own callback: the
        // device oid only exists once that has run.
        let build_state = Arc::downgrade(&state);
        let build_identifier = identifier.clone();
        builder
            .register_config_callback(Box::new(move |builder, mcu| {
                if let Some(state) = build_state.upgrade() {
                    state.build(builder, mcu)?;
                }
                Ok(())
            }))
            .map_err(|err| ConfigError::new(format!("{build_identifier}: {err}")))?;

        let start_state = Arc::downgrade(&state);
        let start_cb: LifecycleCb = Arc::new(move || {
            let start_state = start_state.clone();
            Box::pin(async move {
                let state = start_state
                    .upgrade()
                    .ok_or_else(|| "angle is gone".to_string())?;
                state.start_measurements().await
            })
        });
        let stop_state = Arc::downgrade(&state);
        let stop_cb: LifecycleCb = Arc::new(move || {
            let stop_state = stop_state.clone();
            Box::pin(async move {
                let state = stop_state
                    .upgrade()
                    .ok_or_else(|| "angle is gone".to_string())?;
                state.finish_measurements().await
            })
        });
        let batch_state = Arc::downgrade(&state);
        let batch_cb: BatchCb = Arc::new(move |eventtime| {
            let batch_state = batch_state.clone();
            Box::pin(async move {
                let state = batch_state
                    .upgrade()
                    .ok_or_else(|| "angle is gone".to_string())?;
                state.process_batch(eventtime).await
            })
        });
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, start_cb, stop_cb, ANGLE_BATCH_UPDATES);
        batch_bulk.add_mux_endpoint(DUMP_ENDPOINT, DUMP_KEY, &state.name, dump_start_response())?;
        *state.batch_bulk.lock().unwrap_or_else(|p| p.into_inner()) = Some(batch_bulk);

        Ok(Self { state })
    }

    /// The sensor's name: the section's sub (`config.get_name().split()[1]`).
    pub fn name(&self) -> &str {
        &self.state.name
    }

    /// The configured chip.
    pub fn chip(&self) -> ChipType {
        self.state.chip
    }

    /// The configured seconds between samples.
    pub fn sample_period(&self) -> f64 {
        self.state.sample_period
    }

    /// The decode state, for the calibration path and for tests.
    pub fn extract_state(&self) -> ExtractState {
        *self.state.extract.lock().unwrap_or_else(|p| p.into_inner())
    }
}

impl PrinterObject for Angle {
    /// The sensor's status: only the temperature channel exists upstream, and
    /// only the deferred `tle5012b` helper ever fills it (see the module docs).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "temperature": Value::Null })
    }
}

impl std::fmt::Debug for Angle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Angle")
            .field("name", &self.state.name)
            .field("chip", &self.state.chip)
            .field("sample_period", &self.state.sample_period)
            .finish_non_exhaustive()
    }
}

/// Everything the callbacks and the driver share.
struct AngleState {
    /// The sensor's name (the section's sub).
    name: String,
    /// The chip the firmware should query.
    chip: ChipType,
    /// Seconds between samples.
    sample_period: f64,
    /// The oid the firmware assigned.
    oid: u8,
    /// The SPI bus, shared with the chip's `start()` traffic.
    spi: Arc<McuSpi>,
    /// The MCU the chip is on (the SPI bus's).
    mcu_object: Arc<McuObject>,
    /// The raw `sensor_bulk_data` blocks.
    queue: BulkDataQueue,
    /// The rolling decode state.
    extract: Mutex<ExtractState>,
    /// The machine, for the clock and for the SPI bus.
    printer: Weak<Printer>,
    /// The batch helper that owns the mux endpoint.
    batch_bulk: Mutex<Option<Arc<BatchBulkHelper>>>,
}

impl AngleState {
    /// The build-time half: the chip's config command, its disarmed-at-restart
    /// query, the time-code shift and the block routing (`_build_config`).
    fn build(&self, builder: &ConfigBuilder, mcu: &Mcu) -> Result<(), McuError> {
        let time_shift = time_shift_for(mcu.clock_freq()?);
        {
            let mut extract = self.extract.lock().unwrap_or_else(|p| p.into_inner());
            *extract = ExtractState::new(0, time_shift);
        }
        builder.add_config_cmd(&ConfigSpiAngle {
            oid: self.oid,
            spi_oid: self.spi.oid()?,
            chip_type: self.chip.firmware_value(),
        })?;
        // Upstream arms nothing at config time: `query_spi_angle` is added
        // `on_restart`, so a firmware restart leaves the sensor disarmed.
        builder.add_restart_cmd(&QuerySpiAngle {
            oid: self.oid,
            clock: 0,
            rest_ticks: 0,
            time_shift: 0,
        })?;
        self.queue.bind(mcu, self.oid)?;
        Ok(())
    }

    /// The connected MCU, or why the sensor cannot reach it.
    fn connected_mcu(&self) -> Result<Arc<Mcu>, String> {
        self.mcu_object
            .mcu()
            .ok_or_else(|| "angle's MCU is not connected".to_string())
    }

    /// Bring the chip up and arm the bulk stream (`_start_measurements`).
    async fn start_measurements(&self) -> Result<(), String> {
        for frame in start_frames(self.chip) {
            self.spi
                .transfer(frame)
                .await
                .map_err(|err| err.to_string())?;
        }
        self.queue.clear_queue();
        let mcu = self.connected_mcu()?;
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| "angle's printer is gone".to_string())?;
        let print_time = self
            .mcu_object
            .estimated_print_time(printer.reactor().monotonic())
            .ok_or_else(|| "angle's MCU clock is not synchronized".to_string())?
            + MIN_MSG_TIME;
        let start_clock = self
            .mcu_object
            .print_time_to_clock(print_time)
            .ok_or_else(|| "angle's MCU clock is not synchronized".to_string())?;
        let rest_ticks = mcu
            .seconds_to_clock(self.sample_period)
            .map_err(|err| err.to_string())? as u32;

        let time_shift = {
            let mut extract = self.extract.lock().unwrap_or_else(|p| p.into_inner());
            extract.sample_ticks = i64::from(rest_ticks);
            extract.note_start(start_clock as i64);
            extract.time_shift
        };

        // Upstream sends this with `reqclock=reqclock`: the arm must not reach
        // the board before the clock it names.
        mcu.send_msg_clocked(
            &QuerySpiAngle {
                oid: self.oid,
                clock: start_clock as u32,
                rest_ticks,
                time_shift,
            },
            SendClocks {
                min_clock: None,
                req_clock: Some(start_clock),
            },
        )
        .map_err(|err| err.to_string())?;
        tracing::info!("Starting angle '{}' measurements", self.name);
        Ok(())
    }

    /// Halt the bulk stream (`_finish_measurements`).
    async fn finish_measurements(&self) -> Result<(), String> {
        let mcu = self.connected_mcu()?;
        // Upstream's `send_wait_ack` blocks for the ack; the queued messages
        // ahead of this one preserve the order, so the fire-and-forget send is
        // enough here.
        mcu.send_msg(&QuerySpiAngle {
            oid: self.oid,
            clock: 0,
            rest_ticks: 0,
            time_shift: 0,
        })
        .map_err(|err| err.to_string())?;
        self.queue.clear_queue();
        tracing::info!("Stopped angle '{}' measurements", self.name);
        Ok(())
    }

    /// One batch: pull, decode, report (`_process_batch`).
    async fn process_batch(&self, _eventtime: f64) -> Result<Option<Value>, String> {
        let raw = self.queue.pull_queue();
        if raw.is_empty() {
            return Ok(None);
        }
        let mcu_object = &self.mcu_object;
        let (samples, errors) = {
            let mut extract = self.extract.lock().unwrap_or_else(|p| p.into_inner());
            extract.extract_samples(
                &raw,
                |clock| mcu_object.clock_to_print_time(clock).unwrap_or(0.),
                self.chip.static_delay(),
            )
        };
        if samples.is_empty() {
            return Ok(None);
        }
        Ok(Some(json!({
            "data": samples,
            "errors": errors,
            // No `AngleCalibration` in this build; upstream reports `None` for
            // an uncalibrated sensor the same way.
            "position_offset": Value::Null,
        })))
    }
}

/// Upstream's `load_config_prefix` for `[angle <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(Angle::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::webhooks;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    fn section(name: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("angle", name);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A section and a tracked wrapper for it, leaked so the borrow outlives the
    /// call (tests only).
    fn wrap(name: Option<&str>, options: &[(&str, &str)]) -> ConfigWrapper<'static> {
        ConfigWrapper::new(
            Box::leak(Box::new(section(name, options))),
            AccessTracking::shared(),
        )
    }

    /// A ready printer with `pins` and one registered `[mcu]`.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mcu = McuObject::new(ConfigSection::new("mcu", None), &printer).unwrap();
        printer.add_object("mcu", Arc::new(mcu)).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    #[test]
    fn test_a_named_section_reads_its_chip_and_sample_period() {
        let printer = printer();
        let angle = Angle::new(
            &wrap(
                Some("magnet"),
                &[("cs_pin", "PK7"), ("sensor_type", "mt6816")],
            ),
            &printer,
        )
        .unwrap();
        assert_eq!(angle.name(), "magnet");
        assert_eq!(angle.chip(), ChipType::Mt6816);
        assert_eq!(angle.sample_period(), DEFAULT_SAMPLE_PERIOD);

        let as5047d = Angle::new(
            &wrap(
                Some("second"),
                &[("cs_pin", "PK7"), ("sensor_type", "as5047d")],
            ),
            &printer,
        )
        .unwrap();
        assert_eq!(as5047d.chip(), ChipType::As5047d);
        // `sample_period` overrides the 400us default.
        let slow = Angle::new(
            &wrap(
                Some("third"),
                &[
                    ("cs_pin", "PK7"),
                    ("sensor_type", "a1333"),
                    ("sample_period", "0.001"),
                ],
            ),
            &printer,
        )
        .unwrap();
        assert_eq!(slow.sample_period(), 0.001);
    }

    #[test]
    fn test_the_chip_names_are_the_firmwares_enumeration() {
        let names: Vec<&str> = ChipType::ALL.iter().map(|chip| chip.name()).collect();
        assert_eq!(names, ["a1333", "as5047d", "tle5012b", "mt6816", "mt6826s"]);
        for (index, chip) in ChipType::ALL.iter().enumerate() {
            assert_eq!(chip.firmware_value() as usize, index);
            assert_eq!(ChipType::from_name(chip.name()), Some(*chip));
        }
        assert_eq!(ChipType::from_name("as5311"), None);
    }

    #[test]
    fn test_an_unknown_sensor_type_is_refused() {
        let printer = printer();
        let err = Angle::new(
            &wrap(
                Some("magnet"),
                &[("cs_pin", "PK7"), ("sensor_type", "as5311")],
            ),
            &printer,
        )
        .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice 'as5311' for option 'sensor_type' in section 'angle magnet' is not a valid \
             choice"
        );
    }

    #[test]
    fn test_the_deferred_chips_name_what_is_missing() {
        let printer = printer();
        let err = Angle::new(
            &wrap(
                Some("magnet"),
                &[("cs_pin", "PK7"), ("sensor_type", "tle5012b")],
            ),
            &printer,
        )
        .unwrap_err();
        let message = err.to_string();
        assert!(
            message.contains("angle sensor_type 'tle5012b' is not implemented"),
            "{message}"
        );
        assert!(message.contains("CRC-8/J1850"), "{message}");

        let err = Angle::new(
            &wrap(
                Some("magnet"),
                &[("cs_pin", "PK7"), ("sensor_type", "mt6826s")],
            ),
            &printer,
        )
        .unwrap_err();
        let message = err.to_string();
        assert!(
            message.contains("angle sensor_type 'mt6826s' is not implemented"),
            "{message}"
        );
        assert!(message.contains("ANGLE_CHIP_CALIBRATE"), "{message}");
    }

    #[test]
    fn test_a_stepper_section_says_calibration_is_missing() {
        let printer = printer();
        let err = Angle::new(
            &wrap(
                Some("magnet"),
                &[
                    ("cs_pin", "PK7"),
                    ("sensor_type", "mt6816"),
                    ("stepper", "stepper_x"),
                ],
            ),
            &printer,
        )
        .unwrap_err();
        assert!(
            err.to_string()
                .contains("'stepper' (ANGLE_CALIBRATE) is not implemented"),
            "{err}"
        );
    }

    #[test]
    fn test_a_non_positive_sample_period_is_refused() {
        let printer = printer();
        let err = Angle::new(
            &wrap(
                Some("magnet"),
                &[
                    ("cs_pin", "PK7"),
                    ("sensor_type", "mt6816"),
                    ("sample_period", "0"),
                ],
            ),
            &printer,
        )
        .unwrap_err();
        assert!(
            err.to_string().contains("sample_period"),
            "the bound names the option: {err}"
        );
    }

    #[test]
    fn test_extract_samples_accumulates_angles_and_folds_in_the_time_code() {
        // 1000 ticks per sample, a time code in whole ticks: the sample's
        // timestamp is `start_clock + sequence*17*ticks + i*ticks +
        // (tcode << shift)`, converted at 1 tick = 1us.
        let mut state = ExtractState::new(1000, 0);
        state.note_start(10_000);
        let to_time = |clock: i64| clock as f64 / 1_000_000.;
        let raw = vec![
            // Sequence 1 → the block starts 17 samples after `start_clock`.
            (
                1u16,
                vec![
                    // tcode 3, angle 0x0800 → 2048.
                    0x03, 0x00, 0x08, // tcode 0, angle 0x0c00 → 3072.
                    0x00, 0x00, 0x0c,
                ],
            ),
            // Sequence 2 → the next block, 17 more samples later.
            (2u16, vec![0x00, 0x10, 0x08]),
        ];
        let (samples, errors) = state.extract_samples(&raw, to_time, 0.);
        assert_eq!(errors, 0);
        let block_start = 10_000 + 17 * 1000;
        assert_eq!(
            samples,
            vec![
                [(block_start + 3) as f64 / 1_000_000., 2048.],
                [(block_start + 1000) as f64 / 1_000_000., 3072.],
                [(34 * 1000 + 10_000) as f64 / 1_000_000., 2064.],
            ]
        );
        assert_eq!(state.last_angle(), 2064);
    }

    #[test]
    fn test_extract_samples_drops_error_samples_and_counts_them() {
        let mut state = ExtractState::new(1000, 0);
        state.note_start(0);
        let to_time = |clock: i64| clock as f64;
        let raw = vec![(
            1u16,
            vec![
                // tcode 0xff is the firmware's error marker (`SE_*`).
                TCODE_ERROR,
                0x11,
                0x22, // then a good one.
                0x00,
                0x01,
                0x00,
            ],
        )];
        let (samples, errors) = state.extract_samples(&raw, to_time, 0.);
        assert_eq!(errors, 1);
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0][1], 1.);
    }

    #[test]
    fn test_extract_samples_wraps_the_angle_like_a_signed_difference() {
        let mut state = ExtractState::new(1, 0);
        state.note_start(0);
        let to_time = |clock: i64| clock as f64;
        // 0xffff is -1 against a zero running angle, so the running value goes
        // down instead of jumping to 65535 (the same asymmetry upstream's
        // `angle_diff -= (angle_diff & 0x8000) << 1` applies).
        let raw = vec![(1u16, vec![0x00, 0xff, 0xff, 0x00, 0x01, 0x00])];
        let (samples, _) = state.extract_samples(&raw, to_time, 0.);
        let angles: Vec<f64> = samples.iter().map(|sample| sample[1]).collect();
        assert_eq!(angles, vec![-1., 1.]);
    }

    #[test]
    fn test_extract_samples_shifts_off_the_static_delay() {
        // as5047d's 100us static delay is subtracted from the timestamp and the
        // result is rounded to six decimals, as upstream's
        // `round(clock_to_print_time(sclock) - static_delay, 6)`.
        let mut state = ExtractState::new(1, 0);
        state.note_start(1_000_000);
        let to_time = |clock: i64| clock as f64 / 1_000_000.;
        // Sequence 1 puts the block 17 ticks after `start_clock`, so the first
        // sample lands at 1 s + 17 us, less the chip's 100 us static delay.
        let raw = vec![(1u16, vec![0x00, 0x01, 0x00])];
        let (samples, _) = state.extract_samples(&raw, to_time, ChipType::As5047d.static_delay());
        assert_eq!(samples[0][0], 0.999917);
    }

    #[test]
    fn test_the_time_shift_grows_until_the_time_code_spans_two_milliseconds() {
        // 255 << shift must cover 2ms of firmware clocks: 32000 ticks at
        // 16MHz, so shifts 0..6 are too narrow (255<<6 = 16320) and 7 fits.
        assert_eq!(time_shift_for(16_000_000.), 7);
        // A faster clock needs more room.
        assert_eq!(time_shift_for(48_000_000.), 9);
        assert_eq!(time_shift_for(1_000_000.), 3);
    }

    #[test]
    fn test_the_start_frames_are_the_helpers_sequences() {
        assert_eq!(start_frames(ChipType::A1333), &[&[0x32, 0x00][..]]);
        assert_eq!(
            start_frames(ChipType::As5047d),
            &[&[0xff, 0xfc][..], &[0x40, 0x01][..], &[0xc0, 0x00][..]]
        );
        assert!(start_frames(ChipType::Mt6816).is_empty());
    }

    #[test]
    fn test_the_chip_transport_defaults_are_the_helpers() {
        assert_eq!(ChipType::A1333.spi_mode(), 3);
        assert_eq!(ChipType::As5047d.spi_mode(), 1);
        assert_eq!(ChipType::Mt6816.spi_mode(), 3);
        assert_eq!(ChipType::As5047d.default_spi_speed(), 2_857_142);
        assert_eq!(ChipType::Mt6816.default_spi_speed(), 10_000_000);
        assert_eq!(ChipType::A1333.static_delay(), 0.000001);
        assert_eq!(ChipType::As5047d.static_delay(), 0.000100);
    }

    #[test]
    fn test_get_status_reports_only_the_temperature_channel() {
        let printer = printer();
        let angle = Angle::new(
            &wrap(
                Some("magnet"),
                &[("cs_pin", "PK7"), ("sensor_type", "mt6816")],
            ),
            &printer,
        )
        .unwrap();
        assert_eq!(angle.get_status(0.), json!({ "temperature": Value::Null }));
    }

    #[test]
    fn test_the_dump_start_response_is_upstreams_header() {
        assert_eq!(
            dump_start_response(),
            json!({ "header": ["time", "angle"] })
        );
    }

    #[tokio::test]
    async fn test_the_dump_endpoint_is_registered_under_the_sensor_key() {
        let printer = printer();
        Angle::new(
            &wrap(
                Some("magnet"),
                &[("cs_pin", "PK7"), ("sensor_type", "mt6816")],
            ),
            &printer,
        )
        .unwrap();

        let webhooks = webhooks::install(&printer).unwrap();
        let registrations = webhooks.take_mux_endpoints();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].path, DUMP_ENDPOINT);
        assert_eq!(registrations[0].key, DUMP_KEY);
        assert_eq!(registrations[0].value.as_deref(), Some("magnet"));
    }
}
