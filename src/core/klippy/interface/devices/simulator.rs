//! A dictionary-driven fake MCU: the "responder" the upstream corpus runs
//! against (test builds only).
//!
//! Upstream's own regression tests run klippy with no firmware at all: `-d`
//! injects a data dictionary and `-o` points the "serial port" at a file, and
//! every call that would wait for a reply is short-circuited. This device takes
//! the other road — it *answers* — so the host runs its ordinary path (identify,
//! configuration handshake, clock reads, message sequencing) instead of a branch
//! that production never takes.
//!
//! The dictionary is the whole program. It tells the device how to decode what
//! the host sends and how to encode what it sends back, so one implementation
//! serves every `.test` case:
//!
//! * **identify**: request `offset`/`count`, answer with the zlib-compressed
//!   dictionary in chunks, then an empty chunk — the same exchange the firmware
//!   performs (`klippy/identify.py`), so the host installs the dictionary it was
//!   given.
//! * **configuration**: answer `get_config`, remember the CRC from
//!   `finalize_config`, and report the configuration as current afterwards.
//! * **clock**: answer `get_clock`/`get_uptime` from a monotonic counter.
//! * **sequencing**: every received block is acknowledged by echoing its sequence
//!   with an empty payload, which is how the host's send window advances.
//!
//! What it does not do is simulate hardware: endstop triggers, stepper motion and
//! shutdown reporting are added per case as the corpus needs them.
//!
//! # Endstop 口径 (fake)
//!
//! An armed check trips at the **first step of the move it was armed for**
//! (`reset_step_clock`/`queue_step`), which is what keeps a "triggered prior to
//! movement" probe honest. When several endstops are armed for the same move —
//! delta homes all three towers in one move
//! (`kinematics/delta.py:104-110`) — **every** armed check trips at that first
//! step, each reporting its own trsync. The fake deliberately does not tell
//! their trigger times apart (upstream's file mode likewise completes every
//! armed trsync at the drip move's end); revisit only if a case starts caring
//! about per-endstop trigger instants.

use std::collections::{HashMap, VecDeque};
use std::io::Write;
use std::path::Path;
use std::sync::{Arc, Condvar, Mutex, Weak};
use std::time::Instant;

use flate2::write::ZlibEncoder;
use flate2::Compression;
use serde_json::Value;
use tracing::debug;

use crate::core::klippy::frame::Frame;
use crate::core::klippy::identify;
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::Device;
use crate::core::klippy::mcu::Dictionary;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use crate::core::klippy::trace_enabled;

/// How many bytes one `identify` answer may carry. The host asks for a window of
/// its own size (`cmd::identify::IDENTIFY_CHUNK_SIZE`, `count=40`); this is the
/// fallback when the request's `count` cannot be read.
const IDENTIFY_CHUNK: usize = 40;

/// The move count a fresh fake firmware reports, as upstream's file mode does
/// (`klippy/mcu.py:1037`).
const MOVE_COUNT: u16 = 500;

/// Everything the device remembers between frames.
struct State {
    /// Decodes what the host sends and encodes what it is sent back. Starts with
    /// only the identify pair, gets the full dictionary after identify.
    parser: Parser,
    /// The dictionary, kept for the identify phase.
    dictionary: Option<Dictionary>,
    /// The dictionary file, for `identify_response` to serve.
    raw: Vec<u8>,
    /// `raw`, zlib-compressed, which is what the identify protocol carries.
    compressed: Vec<u8>,
    /// Whether the dictionary has been installed into `parser`.
    installed: bool,
    /// Whether `finalize_config` has been seen since the last reset.
    configured: bool,
    /// The CRC `finalize_config` carried.
    crc: u32,
    /// Ticks per second, from the dictionary's `CLOCK_FREQ`.
    freq: f64,
    /// When this device was created: `clock()` counts from here.
    started: Instant,
    /// Frames waiting for `receive()`.
    out: VecDeque<Frame>,
    /// Set by `shutdown()`: `receive()` returns `None` from then on.
    shutdown: bool,
    /// The clock/pin level of the most recent `endstop_home` activity: the
    /// move-clock floor `reset_step_clock` falls back to, and the answer for
    /// an oid with no [`State::endstop_reports`] entry.
    endstop_clock: u32,
    /// The pin level for that fallback answer.
    endstop_pin_value: u8,
    /// The `endstop_home` checks still armed, by endstop oid, with the
    /// trsync to report each trigger on.
    ///
    /// One slot per endstop: delta arms its three tower endstops together
    /// before one move (`kinematics/delta.py:104-110`), and each has to keep
    /// its own trsync (see the module docs' Endstop 口径 note).
    armed: HashMap<u32, ArmedEndstop>,
    /// What `endstop_query_state` answers per endstop oid: the clock the
    /// check was armed (then fired) at, and the level it stops on. Kept after
    /// the disable removes the check — the host queries after disabling.
    endstop_reports: HashMap<u32, (u32, u8)>,
    /// The `trigger_analog_home` armed for, with the monitor window that
    /// cancels it when the sensor goes quiet (see [`ArmedTriggerAnalog`]).
    trigger_analog: Option<ArmedTriggerAnalog>,
    /// What `trigger_analog_query_state` reports as `homing_clock`: the arm
    /// clock, replaced by the trigger clock when the check fires.
    ta_homing_clock: u32,
    /// The sequence of the host's most recent block, so a frame this device
    /// sends spontaneously (the monitor expiry) carries a sequence the receive
    /// loop has already seen instead of one it would drop.
    last_host_seq: u8,
    /// The step chains this firmware drives, by `config_stepper` oid — the
    /// firmware's `next_step_time` chain (`stepper.c`). Kept across host
    /// sessions: a host that reuses the configuration must re-anchor the chain
    /// with `reset_step_clock` (Q10 / the C5 case); a session that does not
    /// keeps the last session's tail, which is what lets a first step land in
    /// the past (`Timer too close`).
    steppers: HashMap<u8, StepChain>,
    /// Whether the attached sensor feeds the firmware its samples locally —
    /// `ldc1612_attach_trigger_analog` hands the chip to the trigger module,
    /// and from then on samples reach it without ever crossing the wire (the
    /// host pulls bulk data occasionally; `home_start` sizes the monitor
    /// window as one sample period, `MONITOR_MAX` 3 missed samples). The
    /// monitor slides while this is alive and only a genuinely silent sensor
    /// can expire it.
    ldc_sampling: bool,
    /// A shutdown the step-chain model raised, with its `static_string_id`
    /// name. Distinct from `shutdown` (the interface-level close): real
    /// firmware keeps answering `get_clock`/`stats` after a shutdown, and
    /// reports why through `get_config` plus one `shutdown` frame.
    firmware_shutdown: Option<&'static str>,
    /// The `static_string_id` values this dictionary gives the two shutdowns
    /// the model can raise, resolved once at construction; a dictionary
    /// without the string suppresses that report (an unnamed id renders as
    /// `?<value>`, which the host would not match either).
    shutdown_ids: ShutdownIds,
    /// The other fake MCUs this one runs beside in one machine
    /// ([`SimulatorDevice::link_machine`]). Weakly held: the instances of one
    /// test must not keep each other (and their receive threads) alive past
    /// teardown.
    peers: Vec<Weak<SimulatorDevice>>,
    /// A move started on this board while its host block was handled: the
    /// signal [`Device::send`] forwards to `peers` once this board's own lock
    /// is free (see [`SimulatorDevice::note_move`]).
    move_pending: bool,
}

/// The firmware's per-stepper chain state (`stepper.c`): where the next step
/// fires from (`base + interval`), when the armed batch ends (`end`, standing
/// in for `s->count > 0`), and how far apart steps run.
#[derive(Debug, Clone, Copy)]
struct StepChain {
    /// The firmware's `s->next_step_time`: the chain anchor. `config_stepper`
    /// zeroes it; a reused firmware keeps the last session's tail — the
    /// leftover the C5 case hinged on.
    base: u32,
    /// The step distance of the batch that armed this chain.
    interval: u32,
    /// When the armed batch's last step fires (wrapping); the chain is busy
    /// while `now` is before it. Approximates `count`/`add` bookkeeping
    /// (`stepper.c:96-123`) — enough to answer "is this stepper running".
    end: u32,
    /// Whether a batch is armed (the firmware's `s->count > 0`).
    armed: bool,
}

/// The `static_string_id` of the shutdowns [`State::firmware_shutdown`] can
/// raise, read from the dictionary once, at construction.
#[derive(Debug, Default, Clone, Copy)]
struct ShutdownIds {
    timer_too_close: Option<i64>,
    reset_active: Option<i64>,
}

/// An armed `trigger_analog_home` the fake firmware has not fired yet.
#[derive(Debug, Clone, Copy)]
struct ArmedTriggerAnalog {
    /// The trsync to report on.
    trsync_oid: u8,
    /// The reason a trigger match reports (upstream sends `ENDSTOP_HIT`).
    trigger_reason: u8,
    /// The base of a monitor-expiry report; the firmware adds `TE_MONITOR`
    /// (2), so `error_reason = 5` reports reason 7.
    error_reason: u8,
    /// The clock the check was armed at.
    arm_clock: u32,
    /// The clock the monitor window expires at, on the fake's own time base:
    /// now + (monitor_max + 1) missed windows, as `monitor_event` counts them
    /// (`trigger_analog.c:59-75`). The host's arm clock lives in a different
    /// epoch, so the window must not be derived from it. Sample activity
    /// (`i2c_transfer` / `query_status_ldc1612`) pushes it out again — the
    /// firmware's monitor counts *missed* samples, not silence since arming.
    deadline: u32,
    /// The span one window represents, for pushing the deadline out.
    window: u32,
    /// Whether either report has been made; one report ends the check.
    fired: bool,
}

/// An armed `endstop_home` the fake firmware has not triggered yet.
#[derive(Debug, Clone, Copy)]
struct ArmedEndstop {
    /// The trsync to report the trigger on.
    trsync_oid: u8,
    /// The pin level the host stops on.
    level: u8,
    /// The clock the endstop was armed at.
    arm_clock: u32,
    /// Whether the trigger has been reported.
    fired: bool,
}

/// A fake MCU built from a `.dict` file. See the module docs.
pub struct SimulatorDevice {
    state: Mutex<State>,
    signal: Condvar,
}

impl SimulatorDevice {
    /// Build a fake firmware from the data dictionary at `path`.
    ///
    /// # Errors
    /// A message when the file cannot be read, is not JSON, or is not a valid
    /// data dictionary.
    pub fn new(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let raw = std::fs::read(path)
            .map_err(|e| format!("failed to read data dictionary {}: {e}", path.display()))?;
        let value: Value = serde_json::from_slice(&raw)
            .map_err(|e| format!("{} is not a JSON data dictionary: {e}", path.display()))?;
        let dictionary = Dictionary::from_json(value).map_err(|e| e.to_string())?;
        let freq = dictionary.constant_f64("CLOCK_FREQ").unwrap_or(1_000_000.0);
        let mut encoder = ZlibEncoder::new(Vec::new(), Compression::default());
        encoder.write_all(&raw).map_err(|e| format!("zlib: {e}"))?;
        let compressed = encoder.finish().map_err(|e| format!("zlib: {e}"))?;
        let static_strings = dictionary.enumeration("static_string_id");
        let shutdown_ids = ShutdownIds {
            timer_too_close: static_strings.and_then(|e| e.value("Timer too close")),
            reset_active: static_strings
                .and_then(|e| e.value("Can't reset time when stepper active")),
        };
        Ok(Self {
            state: Mutex::new(State {
                parser: identify::new_parser(),
                dictionary: Some(dictionary),
                raw,
                compressed,
                installed: false,
                configured: false,
                crc: 0,
                freq,
                started: Instant::now(),
                out: VecDeque::new(),
                shutdown: false,
                endstop_clock: 0,
                endstop_pin_value: 0,
                armed: HashMap::new(),
                endstop_reports: HashMap::new(),
                trigger_analog: None,
                ta_homing_clock: 0,
                last_host_seq: 0,
                steppers: HashMap::new(),
                firmware_shutdown: None,
                ldc_sampling: false,
                shutdown_ids,
                peers: Vec::new(),
                move_pending: false,
            }),
            signal: Condvar::new(),
        })
    }

    /// The dictionary this device serves, for a test that wants to inspect it.
    pub fn dictionary_len(&self) -> usize {
        self.state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .raw
            .len()
    }

    /// Join `devices` into one machine: a move that starts on any of them trips
    /// the armed checks of **every** one of them.
    ///
    /// The fake reads "the carriage moved" off the steps it receives
    /// (`reset_step_clock` / `queue_step`), but a real machine's endstop may
    /// live on another board than the stepper it guards: the arm arrives on
    /// this board, the steps on that one. Forwarding the step signal is what
    /// lets a two-MCU homing move fire the check on the endstop's board in the
    /// same move it was armed for. Nothing else is shared — every instance
    /// still serves only its own dictionary and its own oids — and each call
    /// replaces the previous membership, so relinking a board is exact.
    pub fn link_machine(devices: &[Arc<Self>]) {
        for (index, device) in devices.iter().enumerate() {
            let peers = devices
                .iter()
                .enumerate()
                .filter(|(other, _)| *other != index)
                .map(|(_, peer)| Arc::downgrade(peer))
                .collect();
            device.state.lock().unwrap_or_else(|p| p.into_inner()).peers = peers;
        }
    }

    /// Note that a move started on this board ([`SimulatorDevice::link_machine`]):
    /// a board with peers leaves a pending signal for [`Device::send`] to carry
    /// to them, outside this board's own lock.
    fn note_move(state: &mut State) {
        if !state.peers.is_empty() {
            state.move_pending = true;
        }
    }

    /// Carry the move signal to every peer: their armed checks fire, each at
    /// the clock *it* was armed with, in its own domain — this board's own
    /// checks already fired where the block was handled
    /// ([`SimulatorDevice::note_move`]).
    ///
    /// Runs with this board's lock released (called from [`Device::send`]): two
    /// boards stepping at once must not end up holding one lock while asking
    /// for the other's. The peers fire only their own checks and never forward
    /// the signal again, so the fan-out is one hop wide.
    fn forward_move_to_peers(&self) {
        let peers: Vec<Arc<SimulatorDevice>> = self
            .state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .peers
            .iter()
            .filter_map(Weak::upgrade)
            .collect();
        for peer in peers {
            peer.fire_checks_armed_for_the_move();
        }
    }

    /// This board's armed checks, tripped by a move that started on a peer:
    /// the checks fire at their own arm clock (the floor `trigger_if_armed`
    /// already applies), which is this move's start in this board's domain.
    fn fire_checks_armed_for_the_move(&self) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        let armed_endstop = state.armed.values().any(|armed| !armed.fired);
        let armed_trigger_analog = state
            .trigger_analog
            .as_ref()
            .is_some_and(|armed| !armed.fired);
        if !armed_endstop && !armed_trigger_analog {
            return;
        }
        // The sequence the host's last block arrived with, so the frame this
        // device sends spontaneously carries one it has already seen
        // (`State::last_host_seq`, the same reason the monitor expiry uses it).
        let seq = state.last_host_seq;
        if armed_endstop {
            let clock = state.endstop_clock;
            Self::trigger_if_armed(&mut state, seq, clock);
        }
        if armed_trigger_analog {
            let clock = state.ta_homing_clock;
            Self::trigger_analog_if_armed(&mut state, seq, clock);
        }
        self.signal.notify_all();
    }

    /// The synthetic clock, in firmware ticks since construction.
    ///
    /// Pure wall time: the firmware's hardware counter does not jump, so
    /// neither does this one. (An "executed floor" that advanced time to a
    /// scheduled command's schedule was tried for Q10 and removed — it let
    /// one chain's far stamp pull the clock far enough ahead to expire an
    /// earlier-stamped chain, and it parked whole batches for seconds; the
    /// host's step-generation horizon now bounds stamps instead.)
    fn clock(state: &State) -> u64 {
        (state.started.elapsed().as_secs_f64() * state.freq) as u64
    }

    /// Encode a response and queue it.
    ///
    /// The frame is stamped with the firmware's counter, which advances to
    /// `seq + 1` the moment it accepts that block (`command.c:300-305`):
    /// everything the host sent up to that number is taken. Echoing the
    /// accepted block's own number instead would leave it in the host's
    /// in-flight queue (the sender drops only what is *below* what the
    /// firmware reports) and the host would retransmit — and this fake would
    /// re-run — that block forever whenever the host then falls silent.
    /// The firmware's timer comparison (`armcm_timer.c:26-30`): a signed
    /// compare over wrapping 32-bit ticks — only meaningful within ±2³¹
    /// ticks (29.8 s at 72 MHz), the edge the real `Timer too close` check
    /// false-positives at.
    fn timer_is_before(a: u32, b: u32) -> bool {
        (a.wrapping_sub(b) as i32) < 0
    }

    /// Steps in the batch a `queue_step` arms, from its first step (`k = 0`,
    /// `base + interval`) to its last (`k = count - 1`): the span the chain's
    /// `end` covers. Truncating `as u32` wraps like the firmware's ticks;
    /// `count = 0` is `Invalid count parameter` upstream and never arrives.
    fn chain_span(interval: u32, count: u16, add: i16) -> u32 {
        let n = i128::from(count.saturating_sub(1));
        let span = i128::from(interval) * n + i128::from(add) * n * (n - 1) / 2;
        span.max(0) as u32
    }

    /// Raise a firmware-style shutdown once: record why, and emit the
    /// `shutdown clock=%u static_string_id=%hu` frame the host renders as
    /// `MCU shutdown: <reason>` (the dictionary gives the id — an unnamed
    /// reason suppresses the frame instead of sending a bogus id).
    fn raise_firmware_shutdown(state: &mut State, seq: u8, reason: &'static str, id: Option<i64>) {
        if state.firmware_shutdown.is_some() {
            return;
        }
        state.firmware_shutdown = Some(reason);
        let Some(id) = id else {
            debug!("simulator: no static_string_id for '{reason}', suppressing the report");
            return;
        };
        let clock = Self::clock(state) as u32;
        Self::respond(
            state,
            seq,
            "shutdown",
            &[ArgValue::UInt32(clock), ArgValue::UInt16(id as u16)],
        );
    }

    fn respond(state: &mut State, seq: u8, name: &str, values: &[ArgValue]) {
        match state.parser.encode(name, values) {
            Ok(payload) => state
                .out
                .push_back(Frame::new((seq + 1) & 0xf, payload.into_raw())),
            Err(e) => debug!("simulator: cannot encode '{name}': {e}"),
        }
    }

    /// Handle the messages of one block.
    fn dispatch(state: &mut State, seq: u8, payload: Vec<u8>) {
        let decoded = match state.parser.decode(Payload::from_raw(payload)) {
            Ok(messages) => messages,
            Err(e) => {
                debug!("simulator: cannot decode a block: {e}");
                return;
            }
        };

        for (message, params) in decoded {
            // Sample activity feeds the monitor: the firmware's window
            // counts missed samples, so an i2c read or bulk-status query
            // while the check is armed starts a fresh window.
            if matches!(
                message.name.as_str(),
                "i2c_transfer" | "query_status_ldc1612"
            ) {
                let now = Self::clock(state) as u32;
                if let Some(armed) = state.trigger_analog.as_mut() {
                    if !armed.fired {
                        armed.deadline = now.wrapping_add(armed.window);
                    }
                }
            }
            match message.name.as_str() {
                "identify" => Self::identify(state, seq, &params),
                "get_config" => {
                    let is_config = u8::from(state.configured);
                    let is_shutdown = u8::from(state.firmware_shutdown.is_some());
                    Self::respond(
                        state,
                        seq,
                        "config",
                        &[
                            ArgValue::UInt8(is_config),
                            ArgValue::UInt32(state.crc),
                            ArgValue::UInt8(is_shutdown),
                            ArgValue::UInt16(MOVE_COUNT),
                        ],
                    );
                }
                "finalize_config" => {
                    if let Some(ArgValue::UInt32(crc)) = params.first() {
                        state.crc = *crc;
                        state.configured = true;
                    }
                }
                "config_reset" | "reset" => {
                    state.configured = false;
                    state.crc = 0;
                    // A real `reset` reboots: chains start over at zero
                    // (`config_stepper` will be resent) and a modelled
                    // shutdown is gone. The clock is *not* restarted here —
                    // construction anchors it, which only differs from a
                    // reboot in uptime, not in ordering.
                    state.steppers.clear();
                    state.firmware_shutdown = None;
                    state.ldc_sampling = false;
                }
                // `clear_shutdown`: the host reads a shutdown, then clears it
                // (upstream: only valid *while* shutdown — `Shutdown cleared
                // when not shutdown`). Idle here: clearing nothing is a no-op.
                "clear_shutdown" => {
                    state.firmware_shutdown = None;
                }
                "get_clock" => {
                    let clock = Self::clock(state) as u32;
                    Self::respond(state, seq, "clock", &[ArgValue::UInt32(clock)]);
                }
                "get_uptime" => {
                    let clock = Self::clock(state);
                    Self::respond(
                        state,
                        seq,
                        "uptime",
                        &[
                            ArgValue::UInt32((clock >> 32) as u32),
                            ArgValue::UInt32(clock as u32),
                        ],
                    );
                }
                "stepper_get_position" => {
                    // `stepper_get_position oid=%c` — the connect-time position
                    // read. No step model, so report zero.
                    let oid = match params.first() {
                        Some(ArgValue::UInt8(v)) => *v,
                        _ => {
                            debug!("simulator: stepper_get_position: missing oid");
                            return;
                        }
                    };
                    Self::respond(
                        state,
                        seq,
                        "stepper_position",
                        &[ArgValue::UInt8(oid), ArgValue::Int32(0)],
                    );
                }
                "endstop_home" => {
                    // `endstop_home oid=%c clock=%u sample_ticks=%u
                    // sample_count=%c rest_ticks=%u pin_value=%c
                    // trsync_oid=%c trigger_reason=%c`. A nonzero
                    // `sample_count` arms the check and the pin reads *open*
                    // (`1 - pin_value`) until the move starts; see
                    // [`ArmedEndstop`]. A zero count is the disable the host
                    // sends after waiting, which leaves the reported level
                    // alone — the query that follows it wants the trigger.
                    let sample_count = match params.get(3) {
                        Some(ArgValue::UInt8(v)) => *v,
                        _ => 0,
                    };
                    // Do not `return`: a block may batch the all-zero disable
                    // with a following command (e.g. `endstop_query_state`),
                    // and skipping the block's tail would drop it.
                    if sample_count != 0 {
                        let oid = match params.first() {
                            Some(ArgValue::UInt8(v)) => u32::from(*v),
                            _ => 0,
                        };
                        let clock = match params.get(1) {
                            Some(ArgValue::UInt32(v)) => *v,
                            _ => 0,
                        };
                        let pin_value = match params.get(5) {
                            Some(ArgValue::UInt8(v)) => *v,
                            _ => 0,
                        };
                        let trsync_oid = match params.get(6) {
                            Some(ArgValue::UInt8(v)) => *v,
                            _ => 0,
                        };
                        state.endstop_clock = clock;
                        state.endstop_pin_value = 1 - pin_value;
                        // A real endstop is *open* until the carriage reaches
                        // it, so the check must not fire at arming: it fires
                        // when the move it was armed for starts.
                        state.endstop_reports.insert(oid, (clock, 1 - pin_value));
                        state.armed.insert(
                            oid,
                            ArmedEndstop {
                                trsync_oid,
                                level: pin_value,
                                arm_clock: clock,
                                fired: false,
                            },
                        );
                        if trace_enabled() {
                            eprintln!(
                                "SIM-DIAG: arm trsync={trsync_oid} clock={clock} level={pin_value}"
                            );
                        }
                    } else {
                        // The disable the host sends after waiting: remove
                        // just this endstop's check (a following
                        // `endstop_query_state` still answers from
                        // `endstop_reports`).
                        let oid = match params.first() {
                            Some(ArgValue::UInt8(v)) => u32::from(*v),
                            _ => 0,
                        };
                        if state.armed.contains_key(&oid) {
                            if trace_enabled() {
                                eprintln!("SIM-DIAG: disarm (query follows)");
                            }
                        }
                        state.armed.remove(&oid);
                    }
                }
                "endstop_query_state" => {
                    let oid = match params.first() {
                        Some(ArgValue::UInt8(v)) => u32::from(*v),
                        _ => 0,
                    };
                    // The answer is this endstop's own arm/fire record, so
                    // three towers homing together each read their own clock.
                    let (clock, pin) = state
                        .endstop_reports
                        .get(&oid)
                        .copied()
                        .unwrap_or((state.endstop_clock, state.endstop_pin_value));
                    Self::respond(
                        state,
                        seq,
                        "endstop_state",
                        &[
                            ArgValue::UInt8(oid as u8),
                            ArgValue::UInt8(0), // homing
                            ArgValue::UInt32(clock),
                            ArgValue::UInt8(pin),
                        ],
                    );
                }
                "trigger_analog_home" => {
                    // `trigger_analog_home oid=%c trsync_oid=%c
                    // trigger_reason=%c error_reason=%c clock=%u
                    // monitor_ticks=%u monitor_max=%u`. A zero `monitor_ticks`
                    // is the disable the host sends after waiting; anything
                    // else arms the check with a monitor window of
                    // (monitor_max + 1) sample periods — and, because this
                    // fake delivers no sensor samples, the window is what
                    // eventually ends the check with `error_reason + MONITOR`
                    // unless a move triggers it first.
                    if let (
                        Some(ArgValue::UInt8(trsync_oid)),
                        Some(ArgValue::UInt32(monitor_ticks)),
                    ) = (params.get(1), params.get(5))
                    {
                        if *monitor_ticks == 0 {
                            state.trigger_analog = None;
                            if trace_enabled() {
                                eprintln!("SIM-DIAG: trigger_analog disarm");
                            }
                        } else {
                            let trigger_reason = match params.get(2) {
                                Some(ArgValue::UInt8(v)) => *v,
                                _ => 1,
                            };
                            let error_reason = match params.get(3) {
                                Some(ArgValue::UInt8(v)) => *v,
                                _ => 5,
                            };
                            let clock = match params.get(4) {
                                Some(ArgValue::UInt32(v)) => *v,
                                _ => 0,
                            };
                            let monitor_max = match params.get(6) {
                                Some(ArgValue::UInt32(v)) => *v,
                                _ => 3,
                            };
                            let window = monitor_max.wrapping_add(1).wrapping_mul(*monitor_ticks);
                            // The monitor is a firmware timer: it runs on the
                            // fake's own clock (`Self::clock`), not on the
                            // host's arm clock — the two epochs differ by
                            // more than `i32::MAX` once the host's print time
                            // runs ahead, and comparing across them reports a
                            // false `error_reason + MONITOR` in the arming
                            // frame before the move can ever trip.
                            let deadline = (Self::clock(state) as u32).wrapping_add(window);
                            state.ta_homing_clock = clock;
                            state.trigger_analog = Some(ArmedTriggerAnalog {
                                trsync_oid: *trsync_oid,
                                trigger_reason,
                                error_reason,
                                arm_clock: clock,
                                deadline,
                                window,
                                fired: false,
                            });
                            if trace_enabled() {
                                eprintln!(
                                    "SIM-DIAG: trigger_analog arm trsync={trsync_oid} clock={clock} \
                                     deadline={deadline} monitor={monitor_ticks}x{monitor_max}"
                                );
                            }
                        }
                    }
                }
                "trigger_analog_query_state" => {
                    // `trigger_analog_state oid=%c homing=%c homing_clock=%u`:
                    // `homing` mirrors `TA_CAN_TRIGGER` — set while armed,
                    // cleared by either report or a disable; `homing_clock` is
                    // the trigger clock once a match fired, else the arm clock.
                    let oid = match params.first() {
                        Some(ArgValue::UInt8(v)) => *v,
                        _ => 0,
                    };
                    let homing = u8::from(
                        state
                            .trigger_analog
                            .as_ref()
                            .is_some_and(|armed| !armed.fired),
                    );
                    Self::respond(
                        state,
                        seq,
                        "trigger_analog_state",
                        &[
                            ArgValue::UInt8(oid),
                            ArgValue::UInt8(homing),
                            ArgValue::UInt32(state.ta_homing_clock),
                        ],
                    );
                }
                "debug_read" => {
                    // `debug_read order=%c addr=%u` — return a simulated value.
                    // For simplicity, we return 0 for all reads (not fully
                    // simulated memory model).
                    let _order = match params.first() {
                        Some(ArgValue::UInt8(v)) => *v,
                        _ => {
                            debug!("simulator: debug_read: missing order");
                            return;
                        }
                    };
                    let _addr = match params.get(1) {
                        Some(ArgValue::UInt32(v)) => *v,
                        _ => {
                            debug!("simulator: debug_read: missing addr");
                            return;
                        }
                    };
                    // Return 0 for all reads (simplified simulation).
                    Self::respond(state, seq, "debug_result", &[ArgValue::UInt32(0)]);
                }
                "debug_write" => {
                    // `debug_write order=%c addr=%u val=%u` — accept but ignore.
                    // No memory model to update.
                    let _order = match params.first() {
                        Some(ArgValue::UInt8(v)) => *v,
                        _ => {
                            debug!("simulator: debug_write: missing order");
                            return;
                        }
                    };
                    let _addr = match params.get(1) {
                        Some(ArgValue::UInt32(v)) => *v,
                        _ => {
                            debug!("simulator: debug_write: missing addr");
                            return;
                        }
                    };
                    let _val = match params.get(2) {
                        Some(ArgValue::UInt32(v)) => *v,
                        _ => {
                            debug!("simulator: debug_write: missing val");
                            return;
                        }
                    };
                    // No response expected for debug_write.
                }
                // A stepper time base reset is the first command of a move:
                // the carriage has started, so an armed endstop now trips. The
                // clock it carries is what the trigger is reported at.
                "reset_step_clock" => {
                    let (Some(ArgValue::UInt8(oid)), Some(ArgValue::UInt32(new_base))) =
                        (params.first(), params.get(1))
                    else {
                        debug!("simulator: reset_step_clock with unexpected arguments");
                        continue;
                    };
                    let (oid, new_base) = (*oid, *new_base);
                    // The firmware refuses a *running* chain before touching
                    // the base (`stepper.c:310-313`); an idle one takes the
                    // new anchor — which is exactly what the restart list of a
                    // config-reusing host carries, and what a session without
                    // it fails to do (the C5 leftover, Q10).
                    let ids = state.shutdown_ids;
                    let now = Self::clock(state) as u32;
                    let busy = state
                        .steppers
                        .get(&oid)
                        .is_some_and(|chain| chain.armed && Self::timer_is_before(now, chain.end));
                    if busy {
                        Self::raise_firmware_shutdown(
                            state,
                            seq,
                            "Can't reset time when stepper active",
                            ids.reset_active,
                        );
                        continue;
                    }
                    if let Some(chain) = state.steppers.get_mut(&oid) {
                        chain.base = new_base;
                        chain.armed = false;
                    }
                    state.endstop_clock = state.endstop_clock.max(new_base);
                    Self::trigger_if_armed(state, seq, new_base);
                    Self::trigger_analog_if_armed(state, seq, new_base);
                    Self::note_move(state);
                }
                // `queue_step` is the move itself: also a trigger point.
                "queue_step" => {
                    let clock = state.endstop_clock;
                    Self::trigger_if_armed(state, seq, clock);
                    Self::trigger_analog_if_armed(state, seq, clock);
                    Self::note_move(state);
                    if state.firmware_shutdown.is_some() {
                        // The firmware refuses commands after a shutdown
                        // (`sched.c`); the block is still acked by the caller.
                        continue;
                    }
                    let (
                        Some(ArgValue::UInt8(oid)),
                        Some(ArgValue::UInt32(interval)),
                        Some(ArgValue::UInt16(count)),
                        Some(ArgValue::Int16(add)),
                    ) = (params.get(0), params.get(1), params.get(2), params.get(3))
                    else {
                        debug!("simulator: queue_step with unexpected arguments");
                        continue;
                    };
                    let (oid, interval, count, add) = (*oid, *interval, *count, *add);
                    if count == 0 {
                        // `Invalid count parameter` (`stepper.c:265-266`):
                        // the host never sends one; ignore like the firmware
                        // would refuse it.
                        continue;
                    }
                    let now = Self::clock(state) as u32;
                    let expired = {
                        let Some(chain) = state.steppers.get_mut(&oid) else {
                            debug!("simulator: queue_step for an oid without config_stepper");
                            continue;
                        };
                        if chain.armed && Self::timer_is_before(now, chain.end) {
                            // Running: the firmware only queues the move
                            // (`stepper.c:280-281`), chaining it behind the
                            // batch — no timer is armed, nothing can expire.
                            chain.end = chain
                                .end
                                .wrapping_add(Self::chain_span(interval, count, add));
                            false
                        } else {
                            // Idle: the first step fires at `base + interval`
                            // (`stepper.c:100-101`). With `base` at zero that
                            // makes the interval the first shot's absolute
                            // clock; with a leftover base it is wherever the
                            // last session left the chain.
                            if chain.armed {
                                // The batch ran out between commands: the
                                // firmware's `next_step_time` walked to the
                                // chain's tail, so the next batch chains off
                                // `end`, not off this batch's first shot.
                                chain.base = chain.end;
                                chain.armed = false;
                            }
                            let first = chain.base.wrapping_add(interval);
                            if Self::timer_is_before(first, now) {
                                true
                            } else {
                                chain.base = first;
                                chain.end =
                                    first.wrapping_add(Self::chain_span(interval, count, add));
                                chain.armed = true;
                                false
                            }
                        }
                    };
                    if expired {
                        let ids = state.shutdown_ids;
                        Self::raise_firmware_shutdown(
                            state,
                            seq,
                            "Timer too close",
                            ids.timer_too_close,
                        );
                    }
                }
                // A fresh `config_stepper` zeroes the chain — over an existing
                // oid it drops whatever the last session left (`oid_alloc`,
                // `stepper.c:220-226`).
                "config_stepper" => {
                    if let Some(ArgValue::UInt8(oid)) = params.first() {
                        state.steppers.insert(
                            *oid,
                            StepChain {
                                base: 0,
                                interval: 0,
                                end: 0,
                                armed: false,
                            },
                        );
                    }
                }
                // The ldc1612's register access over the shared I2C bus:
                // `i2c_transfer oid=%c write=%*s read_len=%u`. The chip answers
                // its two identification registers and reads back zero
                // elsewhere; a write (`read_len = 0`) is an empty success.
                "i2c_transfer" => {
                    let oid = match params.first() {
                        Some(ArgValue::UInt8(v)) => *v,
                        _ => 0,
                    };
                    let reg = match params.get(1) {
                        Some(ArgValue::Bytes(data)) => data.first().copied().unwrap_or(0),
                        _ => 0,
                    };
                    let read_len = match params.get(2) {
                        Some(ArgValue::UInt32(v)) => *v,
                        _ => 0,
                    };
                    // `REG_MANUFACTURER_ID` (0x7e) → `LDC1612_MANUF_ID`,
                    // `REG_DEVICE_ID` (0x7f) → `LDC1612_DEV_ID`; every other
                    // register reads as zero on this fake chip.
                    let value: u16 = match reg {
                        0x7e => 0x5449,
                        0x7f => 0x3055,
                        _ => 0x0000,
                    };
                    let mut response = value.to_be_bytes().to_vec();
                    response.truncate(read_len as usize);
                    Self::respond(
                        state,
                        seq,
                        "i2c_response",
                        &[
                            ArgValue::UInt8(oid),
                            // `i2c_bus_status`: `SUCCESS = 0`.
                            ArgValue::UInt8(0),
                            ArgValue::Bytes(response),
                        ],
                    );
                }
                // The bulk channel's clock status (`query_status_ldc1612`).
                // The stream never carries samples under file-output mode —
                // the host discards them there — so sequence and buffer stay
                // at zero: a deterministic answer every query agrees with,
                // beside whatever the armed `trigger_analog` does (this reply
                // never touches the arm/fire/monitor state).
                "query_status_ldc1612" => {
                    let oid = match params.first() {
                        Some(ArgValue::UInt8(v)) => *v,
                        _ => 0,
                    };
                    let clock = Self::clock(state) as u32;
                    Self::respond(
                        state,
                        seq,
                        "sensor_bulk_status",
                        &[
                            ArgValue::UInt8(oid),
                            ArgValue::UInt32(clock),
                            ArgValue::UInt32(0), // query_ticks
                            ArgValue::UInt16(0), // next_sequence
                            ArgValue::UInt32(0), // buffered
                            ArgValue::UInt16(0), // possible_overflows
                        ],
                    );
                }
                // Everything else is accepted and ignored: `allocate_oids`,
                // The sensor coming up arms its local sample feed: from the
                // attach on, samples are the firmware's own business
                // (`ldc1612_attach_trigger_analog` + `config_ldc1612*`).
                "config_ldc1612" | "config_ldc1612_with_intb" | "ldc1612_attach_trigger_analog" => {
                    state.ldc_sampling = true;
                }
                // `config_*` (`query_ldc1612`'s arm), `emergency_stop`, and any command this
                // fake firmware does not model yet.
                _ => {}
            }
        }
    }

    /// Report every armed endstop's trigger once the move it was armed for has
    /// started (`reset_step_clock` / `queue_step`), each exactly once.
    ///
    /// The clock is the arming clock until the move gives a later one, so the
    /// trigger time is never in the past and never zero. Several endstops may
    /// be armed for one move (delta's simultaneous tower homing): all of them
    /// trip at this first step, each on its own trsync (module docs' Endstop
    /// 口径 note).
    fn trigger_if_armed(state: &mut State, seq: u8, clock: u32) {
        // First mark every armed check fired and collect what to report —
        // `respond` needs the whole state, so nothing may borrow `armed`.
        let mut fired = Vec::new();
        for (oid, armed) in state.armed.iter_mut() {
            if armed.fired {
                continue;
            }
            armed.fired = true;
            fired.push((*oid, armed.trsync_oid, armed.arm_clock, armed.level));
        }
        for (oid, trsync_oid, arm_clock, level) in fired {
            let trigger_clock = clock.max(arm_clock + 1);
            state.endstop_reports.insert(oid, (trigger_clock, level));
            state.endstop_clock = trigger_clock;
            state.endstop_pin_value = level;
            if trace_enabled() {
                eprintln!("SIM-DIAG: fire at clock={clock} arm_clock={arm_clock}");
            }
            Self::respond(
                state,
                seq,
                "trsync_state",
                &[
                    ArgValue::UInt8(trsync_oid),
                    ArgValue::UInt8(0), // can_trigger
                    ArgValue::UInt8(1), // trigger_reason = EndstopHit
                    ArgValue::UInt32(trigger_clock),
                ],
            );
        }
    }

    /// Report an armed `trigger_analog` once the move it was armed for starts
    /// (`reset_step_clock` / `queue_step`), exactly once — the fake's answer to
    /// a matching sample. The monitor window has not expired yet in the normal
    /// flow, so this is the report the host sees first.
    fn trigger_analog_if_armed(state: &mut State, seq: u8, clock: u32) {
        let Some(armed) = state.trigger_analog.as_mut() else {
            return;
        };
        if armed.fired {
            return;
        }
        armed.fired = true;
        let trigger_clock = clock.max(armed.arm_clock.wrapping_add(1));
        let trsync_oid = armed.trsync_oid;
        let trigger_reason = armed.trigger_reason;
        state.ta_homing_clock = trigger_clock;
        if trace_enabled() {
            eprintln!(
                "SIM-DIAG: trigger_analog fire at clock={trigger_clock} arm_clock={}",
                armed.arm_clock
            );
        }
        Self::respond(
            state,
            seq,
            "trsync_state",
            &[
                ArgValue::UInt8(trsync_oid),
                ArgValue::UInt8(0), // can_trigger
                ArgValue::UInt8(trigger_reason),
                ArgValue::UInt32(trigger_clock),
            ],
        );
    }

    /// End an armed `trigger_analog` whose monitor window has passed without
    /// a sample, as `monitor_event` does (`trigger_analog.c:59-75`): report
    /// `error_reason + TE_MONITOR` on the trsync and leave `homing_clock` at
    /// the arm clock (`cancel_homing` does not touch it). Returns whether a
    /// report was made.
    fn fire_monitor_if_expired(state: &mut State, seq: u8) -> bool {
        let now = Self::clock(state) as u32;
        let ldc_sampling = state.ldc_sampling;
        let Some(armed) = state.trigger_analog.as_mut() else {
            return false;
        };
        if armed.fired {
            return false;
        }
        if ldc_sampling {
            // The attached sensor's samples reach the firmware locally —
            // the wire never carries them, so a window that waited on wire
            // traffic would false-expire a perfectly live sensor
            // (`monitor_ticks` = one sample period at `samples_per_second`,
            // `MONITOR_MAX` = 3 missed, `trigger_analog.py:374-385`; fed
            // from `ldc1612_attach_trigger_analog`, `sensor_ldc1612.c`).
            // The window slides until the feed itself goes quiet.
            armed.deadline = now.wrapping_add(armed.window);
            return false;
        }
        if (now.wrapping_sub(armed.deadline) as i32) < 0 {
            return false;
        }
        armed.fired = true;
        let trsync_oid = armed.trsync_oid;
        let reason = armed.error_reason.wrapping_add(2); // + TE_MONITOR
        if trace_enabled() {
            eprintln!(
                "SIM-DIAG: trigger_analog monitor expiry at clock={now} fires reason={reason}"
            );
        }
        Self::respond(
            state,
            seq,
            "trsync_state",
            &[
                ArgValue::UInt8(trsync_oid),
                ArgValue::UInt8(0), // can_trigger
                ArgValue::UInt8(reason),
                ArgValue::UInt32(now),
            ],
        );
        true
    }

    /// How long `receive()` may sleep before an armed monitor window expires;
    /// `None` while nothing can expire on its own.
    fn monitor_wait(state: &State) -> Option<std::time::Duration> {
        let armed = state.trigger_analog.as_ref()?;
        if armed.fired {
            return None;
        }
        let now = Self::clock(state) as u32;
        let remaining = armed.deadline.wrapping_sub(now);
        if (remaining as i32) < 0 {
            return Some(std::time::Duration::ZERO);
        }
        Some(std::time::Duration::from_secs_f64(
            remaining as f64 / state.freq,
        ))
    }

    /// Answer one chunk of the identify exchange.
    fn identify(state: &mut State, seq: u8, params: &[ArgValue]) {
        let offset = match params.first() {
            Some(ArgValue::UInt32(offset)) => *offset as usize,
            _ => 0,
        };
        let count = match params.get(1) {
            Some(ArgValue::UInt8(count)) => *count as usize,
            _ => IDENTIFY_CHUNK,
        };

        let length = state.compressed.len();
        let (data, last) = if offset >= length {
            (Vec::new(), true)
        } else {
            let end = (offset + count).min(length);
            (state.compressed[offset..end].to_vec(), false)
        };

        Self::respond(
            state,
            seq,
            "identify_response",
            &[ArgValue::UInt32(offset as u32), ArgValue::Bytes(data)],
        );

        if last && !state.installed {
            if let Some(dictionary) = state.dictionary.as_ref() {
                if let Err(e) = dictionary.install(&mut state.parser) {
                    debug!("simulator: cannot install the dictionary: {e}");
                }
            }
            state.installed = true;
        }
    }
}

impl std::fmt::Debug for SimulatorDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SimulatorDevice")
            .field("dictionary_len", &self.dictionary_len())
            .finish_non_exhaustive()
    }
}

impl Device for SimulatorDevice {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        let move_pending = {
            let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
            let seq = frame.seq();
            let payload = frame.payload().to_vec();
            state.last_host_seq = seq;
            if !payload.is_empty() {
                // A window that expired before this block wins over what the block
                // asks for (the firmware's monitor timer is not part of it), and
                // one that the block itself arms into is checked again below.
                Self::fire_monitor_if_expired(&mut state, seq);
                Self::dispatch(&mut state, seq, payload);
                Self::fire_monitor_if_expired(&mut state, seq);
            }
            // Every accepted block is acknowledged by echoing its sequence with an
            // empty payload; that is what advances the host's send window. The
            // number is the firmware's counter *after* taking the block
            // (`command.c:305`), i.e. `seq + 1` — see `respond`.
            state.out.push_back(Frame::new((seq + 1) & 0xf, Vec::new()));
            self.signal.notify_all();
            std::mem::take(&mut state.move_pending)
        };
        // A move that started here has to trip the armed checks of the peers
        // (`link_machine`) — after this board's lock is released, so two boards
        // stepping at once cannot deadlock on each other.
        if move_pending {
            self.forward_move_to_peers();
        }
        Ok(())
    }

    fn receive(&self) -> Option<Frame> {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        loop {
            if let Some(frame) = state.out.pop_front() {
                return Some(frame);
            }
            if state.shutdown {
                return None;
            }
            // An armed monitor window ends the check even when the host sends
            // nothing more (upstream `monitor_event` is a firmware timer).
            let seq = state.last_host_seq;
            if Self::fire_monitor_if_expired(&mut state, seq) {
                continue;
            }
            let Some(wait) = Self::monitor_wait(&state) else {
                state = self.signal.wait(state).unwrap_or_else(|p| p.into_inner());
                continue;
            };
            state = self
                .signal
                .wait_timeout(state, wait)
                .unwrap_or_else(|p| p.into_inner())
                .0;
        }
    }

    fn shutdown(&self) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        state.shutdown = true;
        self.signal.notify_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::clock::{ClockState, GetClock};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Mcu;
    use std::time::Duration;

    #[test]
    fn a_bad_dictionary_path_is_reported() {
        let err = SimulatorDevice::new("/nonexistent/klipper.dict").unwrap_err();
        assert!(err.contains("failed to read data dictionary"), "{err}");
    }

    #[tokio::test]
    async fn it_serves_identify_and_answers_a_clock_read() {
        let device =
            SimulatorDevice::new(klipperx_test_support::test_dicts_dir().join("linuxprocess.dict"))
                .expect("the linux-process dictionary");
        let mcu = Mcu::connect("mcu", Interface::simulator(device))
            .await
            .expect("identify against the fake firmware");

        assert!(mcu.is_identified());
        assert!(mcu.dictionary().unwrap().message("get_clock").is_some());

        // The reply must come back through the ordinary call path, which needs
        // the responder to have acknowledged the block it arrived in.
        let state = mcu
            .call_msg::<GetClock, ClockState>(&GetClock, Duration::from_secs(2))
            .await
            .expect("a clock read");
        let _ = state.clock;
    }

    // -----------------------------------------------------------------------
    // trigger_analog arm/fire
    // -----------------------------------------------------------------------

    /// Install the data dictionary into the parser, as the host's identify
    /// handshake would: from then on the device can answer any command the
    /// dictionary defines, without running the exchange.
    fn install_dictionary(device: &SimulatorDevice) {
        let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(dictionary) = state.dictionary.take() {
            dictionary
                .install(&mut state.parser)
                .expect("install the dictionary");
        }
    }

    /// A device whose parser already knows the corpus dictionary, for tests
    /// that drive `dispatch` directly instead of running the identify exchange.
    fn armed_device() -> SimulatorDevice {
        let device =
            SimulatorDevice::new(klipperx_test_support::test_dicts_dir().join("atmega2560.dict"))
                .expect("the corpus dictionary");
        install_dictionary(&device);
        device
    }

    /// [`installed_device`] over `dict`, ready to be linked as one of several
    /// instances of one machine.
    fn installed_device(dict: &Path) -> Arc<SimulatorDevice> {
        let device = Arc::new(SimulatorDevice::new(dict).expect("the dictionary"));
        install_dictionary(&device);
        device
    }

    /// Encode `name` and feed it to the dispatcher as one block.
    fn issue(device: &SimulatorDevice, name: &str, args: &[ArgValue]) {
        let payload = {
            let state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            state
                .parser
                .encode(name, args)
                .expect("encode the message")
                .into_raw()
        };
        let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
        SimulatorDevice::dispatch(&mut state, 0, payload);
    }

    /// The next queued message as `(name, arguments)`, if any.
    fn queued(device: &SimulatorDevice) -> Option<(String, Vec<ArgValue>)> {
        let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
        let frame = state.out.pop_front()?;
        let decoded = state
            .parser
            .decode(Payload::from_raw(frame.payload().to_vec()))
            .expect("decode the queued frame");
        Some((decoded[0].0.name.clone(), decoded[0].1.clone()))
    }

    /// Arm the fake check: oid 1, trsync 2, reasons 1/5, arm clock 1000,
    /// 40 000 ticks per sample, 3 missed windows.
    fn arm(device: &SimulatorDevice) {
        issue(
            device,
            "trigger_analog_home",
            &[
                ArgValue::UInt8(1),
                ArgValue::UInt8(2),
                ArgValue::UInt8(1),
                ArgValue::UInt8(5),
                ArgValue::UInt32(1000),
                ArgValue::UInt32(40_000),
                ArgValue::UInt32(3),
            ],
        );
    }

    #[test]
    fn trigger_analog_home_fires_on_the_first_move() {
        let device = armed_device();
        arm(&device);

        // Armed: the query reports homing at the arm clock.
        issue(&device, "trigger_analog_query_state", &[ArgValue::UInt8(1)]);
        let (name, args) = queued(&device).expect("an answer");
        assert_eq!(name, "trigger_analog_state");
        assert_eq!(
            args,
            vec![
                ArgValue::UInt8(1),
                ArgValue::UInt8(1),
                ArgValue::UInt32(1000)
            ]
        );

        // The first move after arming fires the trsync once, with the move's
        // clock and the armed trigger reason.
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(2000)],
        );
        let (name, args) = queued(&device).expect("a trigger");
        assert_eq!(name, "trsync_state");
        assert_eq!(
            args,
            vec![
                ArgValue::UInt8(2),
                ArgValue::UInt8(0),
                ArgValue::UInt8(1),
                ArgValue::UInt32(2000)
            ]
        );

        // Afterwards: no longer homing, and the clock is the trigger clock.
        issue(&device, "trigger_analog_query_state", &[ArgValue::UInt8(1)]);
        let (name, args) = queued(&device).expect("an answer");
        assert_eq!(name, "trigger_analog_state");
        assert_eq!(
            args,
            vec![
                ArgValue::UInt8(1),
                ArgValue::UInt8(0),
                ArgValue::UInt32(2000)
            ]
        );

        // A later move does not fire again.
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(1000),
                ArgValue::Int32(0),
                ArgValue::UInt32(1000),
            ],
        );
        assert!(queued(&device).is_none(), "one report ends the check");
    }

    #[test]
    fn an_attached_sensor_slides_the_monitor_window() {
        // `home_start` sizes the window as one sample period with three
        // missed allowed (`trigger_analog.py:374-385`); while the sensor is
        // attached those samples reach the firmware without crossing the
        // wire (`ldc1612_attach_trigger_analog`), so an armed monitor must
        // slide rather than expire on wire silence — the `eddy.test` shape
        // (the host's next batch lands well outside a sample-period window).
        let device = armed_device();
        issue(
            &device,
            "config_ldc1612",
            &[ArgValue::UInt8(1), ArgValue::UInt8(1)],
        );
        issue(
            &device,
            "trigger_analog_home",
            &[
                ArgValue::UInt8(1),
                ArgValue::UInt8(2),
                ArgValue::UInt8(1),
                ArgValue::UInt8(5),
                ArgValue::UInt32(0),
                ArgValue::UInt32(1),
                ArgValue::UInt32(3),
            ],
        );

        // Walk far past the window: the local feed keeps it open.
        advance_ticks(&device, 100_000);
        {
            let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            assert!(
                !SimulatorDevice::fire_monitor_if_expired(&mut state, 0),
                "the attached sensor's samples slide the window"
            );
            let armed = state.trigger_analog.as_ref().expect("still armed");
            assert!(!armed.fired, "no monitor report while sampling");
        }

        // Detach (reboot clears the feed with everything else): the same
        // silence then expires the window, as the untouched test below pins.
        issue(&device, "reset", &[]);
        assert_eq!(shutdown_reason(&device), None);
        {
            let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            assert!(!state.ldc_sampling, "a reboot drops the feed");
        }
    }

    #[test]
    fn trigger_analog_monitor_expiry_fires_the_monitor_error() {
        let device = armed_device();
        // One tick of window: the monitor is due as soon as the clock moves.
        issue(
            &device,
            "trigger_analog_home",
            &[
                ArgValue::UInt8(1),
                ArgValue::UInt8(2),
                ArgValue::UInt8(1),
                ArgValue::UInt8(5),
                ArgValue::UInt32(0),
                ArgValue::UInt32(1),
                ArgValue::UInt32(0),
            ],
        );

        // With no host traffic left, `receive()` itself must produce the
        // monitor report: error_reason (5) + TE_MONITOR (2) = reason 7.
        let frame = device.receive().expect("the monitor report");
        let state = device.state.lock().unwrap_or_else(|p| p.into_inner());
        let decoded = state
            .parser
            .decode(Payload::from_raw(frame.payload().to_vec()))
            .expect("decode the report");
        assert_eq!(decoded[0].0.name, "trsync_state");
        assert_eq!(
            decoded[0].1[..3],
            [ArgValue::UInt8(2), ArgValue::UInt8(0), ArgValue::UInt8(7)]
        );
        assert!(matches!(decoded[0].1[3], ArgValue::UInt32(_)));
        drop(state);

        // The expiry leaves `homing_clock` at the arm clock (`cancel_homing`
        // does not touch it) and clears the homing flag.
        issue(&device, "trigger_analog_query_state", &[ArgValue::UInt8(1)]);
        let (_, args) = queued(&device).expect("an answer");
        assert_eq!(
            args,
            vec![ArgValue::UInt8(1), ArgValue::UInt8(0), ArgValue::UInt32(0)]
        );

        // Exactly one report: after a shutdown, nothing more comes out.
        device.shutdown();
        assert!(device.receive().is_none(), "the check does not re-fire");
    }

    #[test]
    fn trigger_analog_sample_activity_pushes_the_monitor_deadline() {
        let device = armed_device();
        // One sample period of window (40 000 ticks), like the corpus' arm.
        issue(
            &device,
            "trigger_analog_home",
            &[
                ArgValue::UInt8(1),
                ArgValue::UInt8(2),
                ArgValue::UInt8(1),
                ArgValue::UInt8(5),
                ArgValue::UInt32(0),
                ArgValue::UInt32(40_000),
                ArgValue::UInt32(0),
            ],
        );
        let window = {
            let state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            Duration::from_secs_f64(40_000.0 / state.freq)
        };

        // Silence: once the window passes, the monitor is due immediately.
        std::thread::sleep(window * 4);
        {
            let state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            assert_eq!(
                SimulatorDevice::monitor_wait(&state),
                Some(Duration::ZERO),
                "an idle window expires the check"
            );
        }

        // A sample read is activity: the firmware counts *missed* samples,
        // so the read starts a fresh window (`monitor_event`).
        issue(
            &device,
            "i2c_transfer",
            &[
                ArgValue::UInt8(0),
                ArgValue::Bytes(vec![0x2e]),
                ArgValue::UInt32(2),
            ],
        );
        {
            let state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            let wait = SimulatorDevice::monitor_wait(&state).expect("still armed");
            assert!(
                wait > Duration::ZERO,
                "i2c activity pushed the deadline out ({wait:?})"
            );
        }

        // An ordinary query is not sample activity: it leaves the fresh
        // window alone, and the check expires again once that window passes.
        std::thread::sleep(window * 4);
        issue(&device, "trigger_analog_query_state", &[ArgValue::UInt8(1)]);
        {
            let state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            assert_eq!(
                SimulatorDevice::monitor_wait(&state),
                Some(Duration::ZERO),
                "a query does not feed the deadline"
            );
        }
    }

    #[test]
    fn trigger_analog_zero_monitor_ticks_disables_the_check() {
        let device = armed_device();
        arm(&device);
        // The all-zero disable the host sends after waiting.
        issue(
            &device,
            "trigger_analog_home",
            &[
                ArgValue::UInt8(1),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt32(0),
                ArgValue::UInt32(0),
            ],
        );

        issue(&device, "trigger_analog_query_state", &[ArgValue::UInt8(1)]);
        let (_, args) = queued(&device).expect("an answer");
        assert_eq!(args[1], ArgValue::UInt8(0), "homing cleared");
        // The arm clock is kept, as the firmware leaves `homing_clock` alone.
        assert_eq!(args[2], ArgValue::UInt32(1000));

        // A move after the disable does not fire.
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(2000)],
        );
        assert!(queued(&device).is_none());
    }

    // -----------------------------------------------------------------------
    // Multiple armed endstops (delta's simultaneous tower homing)
    // -----------------------------------------------------------------------

    /// Arm one endstop check: `endstop_home oid clock sample_ticks
    /// sample_count rest_ticks pin_value trsync_oid trigger_reason`
    /// (`pin_value` 1: reads open until the move, then stops on 1).
    fn arm_endstop(device: &SimulatorDevice, oid: u8, trsync: u8, clock: u32) {
        issue(
            device,
            "endstop_home",
            &[
                ArgValue::UInt8(oid),
                ArgValue::UInt32(clock),
                ArgValue::UInt32(40_000),
                ArgValue::UInt8(4),
                ArgValue::UInt32(0),
                ArgValue::UInt8(1),
                ArgValue::UInt8(trsync),
                ArgValue::UInt8(1),
            ],
        );
    }

    /// Disable the endstop's check (the all-zero-count disable).
    fn disarm_endstop(device: &SimulatorDevice, oid: u8) {
        issue(
            device,
            "endstop_home",
            &[
                ArgValue::UInt8(oid),
                ArgValue::UInt32(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
            ],
        );
    }

    #[test]
    fn three_endstops_armed_together_all_fire_at_the_first_move() {
        let device = armed_device();
        arm_endstop(&device, 1, 10, 1000);
        arm_endstop(&device, 3, 11, 1001);
        arm_endstop(&device, 5, 12, 1002);

        // The move's first step trips every armed check, each on its own
        // trsync (delta homes all three towers in one move).
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(2000)],
        );
        let mut triggers = Vec::new();
        while let Some((name, args)) = queued(&device) {
            assert_eq!(name, "trsync_state");
            triggers.push(args);
        }
        triggers.sort_by_key(|args| match &args[0] {
            ArgValue::UInt8(trsync) => *trsync,
            _ => u8::MAX,
        });
        assert_eq!(triggers.len(), 3, "every armed check reports");
        let expected: [u8; 3] = [10, 11, 12];
        for (index, args) in triggers.iter().enumerate() {
            assert_eq!(args[0], ArgValue::UInt8(expected[index]));
            assert_eq!(args[1], ArgValue::UInt8(0));
            assert_eq!(args[2], ArgValue::UInt8(1), "EndstopHit");
            assert_eq!(args[3], ArgValue::UInt32(2000));
        }

        // Each query answers its own record: armed at its own clock, fired at
        // the move's.
        for (oid, arm_clock) in [(1, 1000), (3, 1001), (5, 1002)] {
            assert!(arm_clock < 2000);
            issue(&device, "endstop_query_state", &[ArgValue::UInt8(oid)]);
            let (name, args) = queued(&device).expect("an answer");
            assert_eq!(name, "endstop_state");
            assert_eq!(args[0], ArgValue::UInt8(oid));
            assert_eq!(args[2], ArgValue::UInt32(2000));
            assert_eq!(args[3], ArgValue::UInt8(1), "tripped level");
        }

        // A later move does not re-fire any of them.
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(3000)],
        );
        assert!(queued(&device).is_none(), "one report per check");
    }

    #[test]
    fn a_single_endstop_still_arms_fires_and_answers_as_before() {
        let device = armed_device();
        arm_endstop(&device, 1, 2, 1000);

        // While armed the check reads open at its arm clock.
        issue(&device, "endstop_query_state", &[ArgValue::UInt8(1)]);
        let (name, args) = queued(&device).expect("an answer");
        assert_eq!(name, "endstop_state");
        assert_eq!(args[2], ArgValue::UInt32(1000));
        assert_eq!(args[3], ArgValue::UInt8(0), "open");

        // The move trips it exactly once.
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(2000)],
        );
        let (name, args) = queued(&device).expect("a trigger");
        assert_eq!(name, "trsync_state");
        assert_eq!(
            args,
            vec![
                ArgValue::UInt8(2),
                ArgValue::UInt8(0),
                ArgValue::UInt8(1),
                ArgValue::UInt32(2000)
            ]
        );

        // The disable clears the check but not its record: the query that
        // follows still reports the trigger.
        disarm_endstop(&device, 1);
        issue(&device, "endstop_query_state", &[ArgValue::UInt8(1)]);
        let (_, args) = queued(&device).expect("an answer");
        assert_eq!(args[2], ArgValue::UInt32(2000));
        assert_eq!(args[3], ArgValue::UInt8(1), "tripped level");

        // And a move after the disable does not fire.
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(3000)],
        );
        assert!(queued(&device).is_none());
    }

    #[test]
    fn disabling_one_endstop_leaves_the_others_armed() {
        let device = armed_device();
        arm_endstop(&device, 1, 10, 1000);
        arm_endstop(&device, 3, 11, 1001);

        // Only oid 1 is disabled; oid 3 keeps waiting for its move.
        disarm_endstop(&device, 1);
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(2000)],
        );
        let (name, args) = queued(&device).expect("oid 3's trigger");
        assert_eq!(name, "trsync_state");
        assert_eq!(args[0], ArgValue::UInt8(11));
        assert_eq!(args[3], ArgValue::UInt32(2000));
        assert!(queued(&device).is_none(), "the disabled check stays silent");

        // The disabled check still answers its own (never-firing) record.
        issue(&device, "endstop_query_state", &[ArgValue::UInt8(1)]);
        let (_, args) = queued(&device).expect("an answer");
        assert_eq!(args[2], ArgValue::UInt32(1000));
        assert_eq!(args[3], ArgValue::UInt8(0), "still open");
    }

    // -----------------------------------------------------------------------
    // linked instances: two fake boards, one machine (FW6a-2)
    // -----------------------------------------------------------------------

    /// Encode `name` and feed it to `device` as one host block over the whole
    /// [`Device::send`] path — unlike [`issue`], which drives the dispatcher
    /// directly and therefore never runs the peer forward that
    /// [`SimulatorDevice::link_machine`] hangs off a block.
    fn host_block(device: &SimulatorDevice, name: &str, args: &[ArgValue]) {
        let payload = {
            let state = device.state.lock().unwrap_or_else(|p| p.into_inner());
            state
                .parser
                .encode(name, args)
                .expect("encode the message")
                .into_raw()
        };
        Device::send(device, &Frame::new(0, payload)).expect("the fake accepts the block");
    }

    /// The next frame `device` would send, stepping over the empty block acks.
    fn next_answer(device: &SimulatorDevice) -> Option<(String, Vec<ArgValue>)> {
        let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
        while let Some(frame) = state.out.pop_front() {
            if frame.payload().is_empty() {
                continue;
            }
            let decoded = state
                .parser
                .decode(Payload::from_raw(frame.payload().to_vec()))
                .expect("decode the queued frame");
            return Some((decoded[0].0.name.clone(), decoded[0].1.clone()));
        }
        None
    }

    /// Two instances of one machine: each answers only its own traffic, and a
    /// move starting on one board trips the endstop armed on the other — the
    /// stepper-on-A / endstop-on-B shape a single instance cannot express.
    #[test]
    fn linked_fake_mcus_forward_the_move_but_nothing_else() {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let board_a = installed_device(&dict);
        let board_b = installed_device(&dict);
        assert!(
            !Arc::ptr_eq(&board_a, &board_b),
            "two instances, not one shared device"
        );
        SimulatorDevice::link_machine(&[Arc::clone(&board_a), Arc::clone(&board_b)]);

        // B arms its endstop (oid 1, trsync 2) for a move; the move starts on A.
        arm_endstop(&board_b, 1, 2, 1000);
        host_block(
            &board_a,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(2000)],
        );

        // B reports the trigger on its own trsync, at *its* arm clock — A's
        // step clock never crosses the instance boundary.
        let (name, args) = next_answer(&board_b).expect("B fires the armed check");
        assert_eq!(name, "trsync_state");
        assert_eq!(
            args,
            vec![
                ArgValue::UInt8(2),
                ArgValue::UInt8(0),
                ArgValue::UInt8(1),
                ArgValue::UInt32(1001)
            ]
        );
        // A had nothing armed, so it stays silent — only its block ack, which
        // `next_answer` skips.
        assert!(next_answer(&board_a).is_none(), "A's checks are its own");

        // Each instance still answers for itself: a query sent to A comes from
        // A, and B (never asked) says nothing.
        host_block(&board_a, "get_clock", &[]);
        let (name, args) = next_answer(&board_a).expect("A answers its own query");
        assert_eq!(name, "clock");
        assert!(matches!(args.as_slice(), [ArgValue::UInt32(_)]));
        assert!(next_answer(&board_b).is_none(), "B was not queried");

        // Unlinked, the same shape is silent again: the forward only ever runs
        // between instances a test explicitly joined.
        let solo = installed_device(&dict);
        let lonely = installed_device(&dict);
        arm_endstop(&lonely, 1, 5, 4000);
        host_block(
            &solo,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(2000)],
        );
        assert!(
            next_answer(&lonely).is_none(),
            "unlinked instances do not hear each other's moves"
        );
    }

    // -----------------------------------------------------------------------
    // step chain: the firmware's `next_step_time` across sessions (Q10 / C5)
    // -----------------------------------------------------------------------

    /// `config_stepper oid=0` with dummy pins: a chain at base zero.
    fn config_a_stepper(device: &SimulatorDevice) {
        issue(
            device,
            "config_stepper",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt8(16),
                ArgValue::UInt8(17),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
            ],
        );
    }

    /// Advance the fake's clock by `ticks` of this dictionary's `CLOCK_FREQ`
    /// (the clock counts from construction, anchored at `State::started`), so
    /// "now" lands where a test needs it without sleeping.
    ///
    /// The test dictionaries differ (16 MHz AVR, 72 MHz STM32…), so tick
    /// math has to follow the dict rather than a hard-coded rate — a fixed
    /// "500 ms" of wall time is 8 M ticks at 16 MHz, not the 36 M a 72 MHz
    /// reading would give.
    fn advance_ticks(device: &SimulatorDevice, ticks: u32) {
        let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
        let by = Duration::from_secs_f64(ticks as f64 / state.freq);
        if let Some(earlier) = state.started.checked_sub(by) {
            state.started = earlier;
        }
    }

    /// The recorded reason the model shut down for, if any.
    fn shutdown_reason(device: &SimulatorDevice) -> Option<&'static str> {
        device
            .state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .firmware_shutdown
    }

    #[test]
    fn timer_is_before_follows_the_firmware_wrap() {
        assert!(SimulatorDevice::timer_is_before(100, 200));
        assert!(!SimulatorDevice::timer_is_before(200, 100));
        // Across the wrap the signed compare keeps ordering: 0xffff_ff00 is
        // "now - 256", 0x0000_0100 is "now + 256".
        assert!(SimulatorDevice::timer_is_before(0xffff_ff00, 0x0000_0100));
        assert!(!SimulatorDevice::timer_is_before(0x0000_0100, 0xffff_ff00));
        // The 2³¹-tick edge (29.8 s at 72 MHz): exactly half a wrap ahead
        // compares as "before" — the false-positive side of `Timer too
        // close`, kept because the firmware's arithmetic has it too
        // (`armcm_timer.c:26-30`).
        assert!(!SimulatorDevice::timer_is_before(0x7fff_ffff, 0));
        assert!(SimulatorDevice::timer_is_before(0x8000_0000, 0));
    }

    #[test]
    fn an_expired_first_step_reports_timer_too_close() {
        let device = armed_device();
        config_a_stepper(&device);
        advance_ticks(&device, 72_000); // > 1_000 ticks at any dict rate

        // `base = 0`, a small interval: the first shot (`base + interval`)
        // lands behind the fake's now — the firmware arms it at :283-285 and
        // `sched_add_timer` trips over the past (`sched.c:91-94`).
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(1_000),
                ArgValue::UInt16(100),
                ArgValue::Int16(0),
            ],
        );
        let (name, args) = queued(&device).expect("the shutdown report");
        assert_eq!(name, "shutdown");
        let id = device
            .state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .shutdown_ids
            .timer_too_close
            .expect("the dictionary names 'Timer too close'");
        assert!(matches!(args[0], ArgValue::UInt32(_)), "clock is a u32");
        assert_eq!(args[1], ArgValue::UInt16(id as u16));
        assert_eq!(shutdown_reason(&device), Some("Timer too close"));

        // `get_config` reports the shutdown the way real firmware does …
        issue(&device, "get_config", &[]);
        let (_, cfg) = queued(&device).expect("a config answer");
        assert_eq!(cfg[2], ArgValue::UInt8(1));

        // … and refuses further steps (the block is still acked by `send`).
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(5_000),
                ArgValue::UInt16(10),
                ArgValue::Int16(0),
            ],
        );
        assert!(queued(&device).is_none(), "the firmware is shut down");
    }

    #[test]
    fn a_stale_chain_expires_until_reset_step_clock_reanchors() {
        // The C5 shape: a session leaves its chain tail behind; the next
        // session that reuses the configuration must re-anchor it — the
        // restart list's `reset_step_clock clock=0` (`stepper.py:117-118`).
        let device = armed_device();
        config_a_stepper(&device);
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(0x00ff_ffff), // first shot far in the future
                ArgValue::UInt16(1),
                ArgValue::Int16(0),
            ],
        );
        assert!(queued(&device).is_none(), "armed without complaint");
        advance_ticks(&device, 0x0100_1000); // past the 0x00ff_ffff tail
                                             // The batch ran out; the chain's base stays at its tail — the
                                             // leftover.

        // Without the re-anchor (the bug): a small interval chains onto the
        // leftover and the first shot is in the past.
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(64),
                ArgValue::UInt16(50),
                ArgValue::Int16(0),
            ],
        );
        let (name, _) = queued(&device).expect("the stale chain expires");
        assert_eq!(name, "shutdown");

        // With it (a fresh device, same history): `reset_step_clock clock=0`
        // re-anchors, and the interval becomes the first shot's absolute
        // clock again (base 0 ⇒ P4), so a future shot is accepted.
        let device = armed_device();
        config_a_stepper(&device);
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(0x00ff_ffff),
                ArgValue::UInt16(1),
                ArgValue::Int16(0),
            ],
        );
        assert!(queued(&device).is_none());
        advance_ticks(&device, 0x0100_1000); // now ≈ 0x0100_1000
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(0)],
        );
        assert!(queued(&device).is_none(), "no endstop is armed");
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(0x0400_0000), // 67 M ticks > now, < 2³¹
                ArgValue::UInt16(100),
                ArgValue::Int16(0),
            ],
        );
        assert!(queued(&device).is_none(), "the re-anchored shot is ahead");
        assert_eq!(
            shutdown_reason(&device),
            None,
            "no shutdown on the re-anchored path"
        );
    }

    #[test]
    fn reset_of_a_running_chain_reports_the_firmware_error() {
        let device = armed_device();
        config_a_stepper(&device);
        issue(
            &device,
            "queue_step",
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(0x0100_0000), // armed ~0.23 s out; busy
                ArgValue::UInt16(100),
                ArgValue::Int16(0),
            ],
        );
        assert!(queued(&device).is_none());

        // Reach `busy` over the wire: the clock walks into the armed batch's
        // span (past its first shot, before its end), and a re-anchor while
        // the chain runs is the firmware's refusal (`stepper.c:310-313`).
        advance_ticks(&device, 0x0100_0000 + 1_000);

        // `stepper.c:310-313`: a running chain refuses the re-anchor.
        issue(
            &device,
            "reset_step_clock",
            &[ArgValue::UInt8(0), ArgValue::UInt32(0)],
        );
        let (name, args) = queued(&device).expect("the shutdown report");
        assert_eq!(name, "shutdown");
        let id = device
            .state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .shutdown_ids
            .reset_active
            .expect("the dictionary names 'Can't reset time when stepper active'");
        assert_eq!(args[1], ArgValue::UInt16(id as u16));
        assert_eq!(
            shutdown_reason(&device),
            Some("Can't reset time when stepper active")
        );
    }
}
