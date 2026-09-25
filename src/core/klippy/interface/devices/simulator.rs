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
use std::sync::{Condvar, Mutex};
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

    /// The synthetic clock, in firmware ticks since construction.
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
                    Self::respond(
                        state,
                        seq,
                        "config",
                        &[
                            ArgValue::UInt8(is_config),
                            ArgValue::UInt32(state.crc),
                            ArgValue::UInt8(0),
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
                        eprintln!(
                            "SIM-DIAG: arm trsync={trsync_oid} clock={clock} level={pin_value}"
                        );
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
                            eprintln!("SIM-DIAG: disarm (query follows)");
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
                            eprintln!("SIM-DIAG: trigger_analog disarm");
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
                            eprintln!(
                                "SIM-DIAG: trigger_analog arm trsync={trsync_oid} clock={clock} \
                                 deadline={deadline} monitor={monitor_ticks}x{monitor_max}"
                            );
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
                    let clock = match params.get(1) {
                        Some(ArgValue::UInt32(v)) => *v,
                        _ => state.endstop_clock,
                    };
                    state.endstop_clock = state.endstop_clock.max(clock);
                    Self::trigger_if_armed(state, seq, clock);
                    Self::trigger_analog_if_armed(state, seq, clock);
                }
                // `queue_step` is the move itself: also a trigger point.
                "queue_step" => {
                    let clock = state.endstop_clock;
                    Self::trigger_if_armed(state, seq, clock);
                    Self::trigger_analog_if_armed(state, seq, clock);
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
                // `config_*` (`config_ldc1612*`, `ldc1612_attach_trigger_analog`,
                // `query_ldc1612`'s arm), `emergency_stop`, and any command this
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
            eprintln!("SIM-DIAG: fire at clock={clock} arm_clock={arm_clock}");
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
        eprintln!(
            "SIM-DIAG: trigger_analog fire at clock={trigger_clock} arm_clock={}",
            armed.arm_clock
        );
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
        let Some(armed) = state.trigger_analog.as_mut() else {
            return false;
        };
        if armed.fired {
            return false;
        }
        if (now.wrapping_sub(armed.deadline) as i32) < 0 {
            return false;
        }
        armed.fired = true;
        let trsync_oid = armed.trsync_oid;
        let reason = armed.error_reason.wrapping_add(2); // + TE_MONITOR
        eprintln!("SIM-DIAG: trigger_analog monitor expiry at clock={now} fires reason={reason}");
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

    /// A device whose parser already knows the corpus dictionary, for tests
    /// that drive `dispatch` directly instead of running the identify exchange.
    fn armed_device() -> SimulatorDevice {
        let device =
            SimulatorDevice::new(klipperx_test_support::test_dicts_dir().join("atmega2560.dict"))
                .expect("the corpus dictionary");
        let mut state = device.state.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(dictionary) = state.dictionary.take() {
            dictionary
                .install(&mut state.parser)
                .expect("install the dictionary");
        }
        drop(state);
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
}
