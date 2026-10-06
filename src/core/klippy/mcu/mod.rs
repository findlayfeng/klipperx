//! The transport layer: frames, the message parser, the firmware data dictionary,
//! and the bare `send` / `call` pair that take message names as strings.
//!
//! Construction ([`Mcu::new`]) only brings the transport up: the parser knows the
//! host's identify formats and nothing else, so typed commands answer
//! [`McuError::NotIdentified`] until the identify handshake installs a dictionary.
//! That handshake is [`Mcu::connect`], defined in [`identify`]
//! together with the transfer it drives — this module does not call it.
//!
//! Anything that names a message in the type system —
//! [`McuCommand`](super::cmd::McuCommand), [`McuResponse`](super::cmd::McuResponse),
//! [`Params`](super::cmd::Params), and the typed calls — lives beside this module
//! in [`cmd`](super::cmd), together with the command modules themselves. Identify
//! straddles both: its formats, its transfer, and its entry points are transport
//! work in [`identify`], while its two typed views are defined in
//! [`cmd::identify`](super::cmd::identify).
//!
//! The one thing the transport asks of the modules beside it is the parser it
//! starts with: [`Mcu::new`] calls `identify::new_parser`, because the two formats
//! it has to know before anything else are identify's.

mod config;
mod dictionary;
mod error;
mod events;
mod object;
mod pending;
mod resource;
mod restart;
mod restart_method;

pub use config::{
    query_slot, BuiltConfig, ConfigBuilder, ConfigCallback, Configured, PostInitCallback,
    PreBuildCallback,
};
pub use dictionary::{Dictionary, Enumeration, MessageDef, OutputDef};
pub use error::{McuCallError, McuError};
pub use object::{load_config, load_config_prefix, McuObject};
pub(crate) use resource::pin_number;
pub use resource::{
    Completion, I2cMode, McuAdc, McuChip, McuDigitalOut, McuEndstop, McuI2c, McuPwm, McuSpi,
    McuStepper, McuTriggerAnalog, McuTrsync, SosFilter, SosFilterDesign, SpiMode, TriggerDispatch,
    TrsyncRegistry, DEFAULT_SPEED, MONITOR_MAX, TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT,
};
pub use restart_method::McuRestartMethod;

use events::McuEvents;

/// How many outbound commands the send queue holds.
///
/// Upstream never refuses at all — its writer blocks on the port, so a host
/// burst is only ever slowed down. This queue is bounded, so the two kinds of
/// producer are told apart: paths that can queue a lot (step batches, the
/// configuration phase) use the awaiting [`Mcu::send_payload`], and the
/// synchronous [`Mcu::send`] waits out a burst for [`SYNC_SEND_WAIT`]. A
/// legitimate burst is large: a bed-mesh calibration probes a 7x7 grid (49
/// points), each with an endstop/trsync arm plus its step blocks, and a 20x4
/// HD44780 panel's refresh queues one `spi_send` per nibble byte — 480 of them
/// for one full screen (`extras/display/hd44780_spi.rs`). A sender that is
/// still without room after the wait is the runaway case this bound reports.
const SEND_QUEUE_CAPACITY: usize = 512;

/// Slots the awaiting producers ([`Mcu::send_payload`]) leave free for the
/// synchronous [`Mcu::send`].
///
/// `Mcu::send` cannot await room — it waits [`SYNC_SEND_WAIT`] at most — so a
/// burst of step batches that saturates the queue would starve every sync
/// sender behind it (an endstop arm during `PROBE` is the observed case:
/// `endstop_home` found the queue full and the g-code line failed). The
/// awaiting path therefore stops at this watermark and lets the wire drain;
/// the sync path gets the reserved slots.
const SYNC_SEND_HEADROOM: usize = 16;

/// How long a synchronous sender ([`Mcu::send`]) waits for room in the outbound
/// queue before it reports the queue as full.
///
/// The wait is what lets a large legitimate burst through: the panel refresh
/// above queues ~480 `spi_send`s back to back, which a full queue would refuse
/// for as long as the send task is behind it. Draining the 512 slots takes tens
/// of milliseconds on an idle host, so a second is generous; past that the
/// queue is not keeping up with a burst, which is exactly what the guard is for.
/// On a single-threaded runtime the wait cannot help (the task that drains the
/// queue is the one being waited on), but it is bounded, so the wait still ends
/// in the reported error.
const SYNC_SEND_WAIT: Duration = Duration::from_secs(1);

/// The first wait between two looks at the queue in [`try_send_bounded`].
const SYNC_SEND_POLL_START: Duration = Duration::from_micros(50);

/// The longest wait between two such looks: the backoff doubles up to this, so a
/// queue that is draining takes the sender with it without hammering
/// `capacity()`.
const SYNC_SEND_POLL_MAX: Duration = Duration::from_millis(10);

use crate::core::klippy::load::section;

// The `[mcu]` / `[mcu <name>]` sections. The factories are re-exported from
// `object` above; `build.rs` resolves them to this module's path.
//
// `phase = early`: upstream loads `pins` and `mcu` before the generic section
// walk (`klippy/klippy.py:120-121`), so an MCU is registered as a chip before
// any section that names it — including `[board_pins]` with `mcu: zboard`.
section!(
    "mcu",
    order = 10,
    phase = early,
    load = load_config,
    prefix = load_config_prefix
);

use crate::core::klippy::frame::{Frame, MESSAGE_PAYLOAD_MAX};
use crate::core::klippy::identify;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::pending::PendingCalls;
use crate::core::klippy::msg::error::MsgError;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use crate::core::klippy::msg::Msg;
use crate::core::klippy::trace_enabled;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::Instant;
use tokio::sync::mpsc::error::TrySendError;
use tokio::sync::{mpsc, oneshot, watch};
use tokio::time::{sleep, Duration};
use tracing::{debug, error, info, warn};
/// One item on the outbound queue: a message, or a barrier that asks the send
/// task to flush what is already queued.
///
/// The send task coalesces payloads queued close together into one wire block,
/// which is what most callers want — but some need two commands to land in
/// **separate** blocks, because the first makes the firmware leave the block it
/// is dispatching (its shutdown is a `longjmp`, `src/sched.c`) and drop the rest
/// of it. Such a caller asks for a boundary with [`Mcu::flush`].
#[derive(Debug)]
enum SendItem {
    /// A message to append to the current batch.
    Payload(Payload),
    /// A message carrying scheduling gates (see [`SendClocks`]).
    Clocked(Payload, SendClocks),
    /// Send whatever is queued so far, then signal completion.
    Flush(oneshot::Sender<()>),
    /// Adopt the sequence the firmware last reported and signal completion
    /// (see [`Mcu::renumber_to_firmware`]).
    Renumber(oneshot::Sender<()>),
}

/// The scheduling gates a message carries: upstream's `min_clock`/`req_clock`,
/// the two numbers `serialqueue_send` stamps on every queued message
/// (`serialqueue.c:939-944`). Both are firmware clock ticks of this MCU.
///
/// * `min_clock` — **not before**: the send task parks the message until the
///   estimated clock reaches it (`serialqueue.c:556`, where `ack_clock <
///   qm->min_clock` keeps a message out of the ready queues).
/// * `req_clock` — **not later than / priority**: the message wants to be on
///   the wire by `req_clock` with [`MIN_REQTIME_DELTA`] of lead. It is
///   released as soon as `req_clock <= ack_clock + MIN_REQTIME_DELTA`
///   (`PR_NOW`, `serialqueue.c:644-646`), and among the messages the gates
///   release the **lowest** `req_clock` goes first (`serialqueue.c:478-486`,
///   "highest priority message").
///
/// A message with neither gate — the configuration phase, the g-code class
/// sends — never waits: there is no gate to check, and its `req_clock` reads
/// as 0, upstream's default for `send(data)` (`mcu.py:101`). So among ungated
/// messages every key ties and the pick falls back to queue order, byte for
/// byte the batching this task did before the gates existed.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct SendClocks {
    /// Hold until the estimated clock is at or past this.
    pub min_clock: Option<u64>,
    /// Send by this clock — this window before it, and behind any lower one.
    pub req_clock: Option<u64>,
}

/// How long before its `req_clock` a gated message wants to be on the wire.
///
/// Upstream's `MIN_REQTIME_DELTA` (`serialqueue.c:110`): a message whose
/// `req_clock` is inside this window of the estimated clock sends now
/// (`PR_NOW`) instead of waiting for the clock to climb to it.
const MIN_REQTIME_DELTA: f64 = 0.100;

/// How far past a move's completion clock the host treats its slot in the
/// firmware's move queue as free ([`MoveSlots::free_at`]).
///
/// The pool judges with `estimated_clock()`, so this has to absorb how far
/// that estimate can run **ahead** of the firmware's clock: released early
/// means the board is handed a move while it still holds the previous one —
/// a "Move queue overflow" (`basecmd.c:85-90`). The estimate is re-anchored at
/// every clock round trip's midpoint (`ClockEstimate::record`), so its error
/// is bounded by one sample's placement and the fitted rate — a few ms, the
/// order of upstream's `TRANSMIT_EXTRA = .001` (`clocksync.py:10`); 20 ms
/// covers that with margin to spare. The other direction (estimate running
/// slow) only makes the host wait out this same 20 ms, and it has to stay an
/// order below upstream's `MIN_SCHEDULE_TIME = 0.100` (`mcu.py:13`) — the
/// lead a move still needs before its own clock — or preventing overflow
/// would turn into sending too late.
const SLOT_RELEASE_MARGIN: f64 = 0.020;

/// How long the send task sleeps at a time while a gated message is parked.
///
/// The gates are judged against an estimate that can be re-anchored
/// ([`Mcu::set_clock_base`], on connect and in tests) *while* a message waits;
/// one sleep computed from the old anchor would outlive the new one. Slicing
/// the wait costs nothing — the task is parked either way — and bounds how
/// long a re-anchored clock goes unnoticed.
const GATE_REPOLL_MAX: Duration = Duration::from_millis(10);

/// A message waiting on its gates in the send task, and what its park/release
/// lines read back (C5's observability): when it parked, and which gate was
/// holding it then.
struct ParkedMsg {
    payload: Payload,
    clocks: SendClocks,
    /// When it entered the deque — the release line reports how long it sat.
    parked_at: Instant,
    /// [`SendClocks::held_by`] answered at that moment: the gate that has to
    /// open again before this message can go out.
    held_by: &'static str,
}

/// A message waiting on its gates in the send task, or a flush barrier queued
/// behind such messages. The send task owns this list; nothing else sees it.
enum Parked {
    Payload(ParkedMsg),
    /// [`Mcu::flush`] behind parked messages: everything queued ahead of the
    /// barrier includes the parked messages, so the barrier waits with them.
    /// The `Instant` is when the barrier was queued (how long it waited).
    Flush(oneshot::Sender<()>, Instant),
}

impl Parked {
    fn is_flush(&self) -> bool {
        matches!(self, Self::Flush(..))
    }

    fn clocks(&self) -> SendClocks {
        match self {
            Self::Payload(msg) => msg.clocks,
            Self::Flush(..) => SendClocks::default(),
        }
    }
}

/// "Now" as the gates see it: the estimated firmware clock, plus
/// [`MIN_REQTIME_DELTA`] already converted to ticks of that clock.
struct GateNow {
    clock: u64,
    req_lead: u64,
}

impl SendClocks {
    /// Whether both gates allow this message onto the wire at `now`.
    ///
    /// `now = None` — no clock estimate or no `CLOCK_FREQ` yet — opens both
    /// gates: upstream does the same while its clock is still unknown
    /// (`serialqueue.c:612-618`, "Clock unknown during initial startup ...
    /// return PR_NOW").
    fn released(&self, now: Option<&GateNow>) -> bool {
        self.held_by_raw(now).is_none()
    }

    /// Which gate(s) hold this message at `now`: `"min"`, `"req"`,
    /// `"min+req"`, or `"none"` when nothing does (`now = None` reads as
    /// "clock unknown", which releases).
    ///
    /// This is the label the park line writes down and the release line reads
    /// back as its reason — the gates have no other trace.
    fn held_by(&self, now: Option<&GateNow>) -> &'static str {
        self.held_by_raw(now).unwrap_or("none")
    }

    /// [`SendClocks::held_by`] without the `"none"` label, so `released` can
    /// share the one judgement.
    fn held_by_raw(&self, now: Option<&GateNow>) -> Option<&'static str> {
        let now = now?;
        let held_min = self
            .min_clock
            .is_some_and(|min_clock| now.clock < min_clock);
        let held_req = self
            .req_clock
            .is_some_and(|req_clock| req_clock > now.clock + now.req_lead);
        match (held_min, held_req) {
            (true, true) => Some("min+req"),
            (true, false) => Some("min"),
            (false, true) => Some("req"),
            (false, false) => None,
        }
    }
}

/// The clock window one generated step batch covers, in ticks of its MCU —
/// what the step path hands the move pool (`McuStepper::send_steps_async`).
///
/// * `start` — when the batch's first step can run: the previous generation
///   horizon (`last_step_gen_time`), a lower bound on the batch's first step.
///   It is the batch's `req_clock`, so the message wants to be on the wire
///   [`MIN_REQTIME_DELTA`] before it — upstream stamps the same boundary on
///   every step message (`stepcompress.c:359`,
///   `min_clock = req_clock = last_step_clock`).
/// * `completion` — when its last step runs (`step_gen_time`): the clock that
///   frees this batch's slots in the firmware's move queue (`MoveSlots`).
///
/// A stepper with no print-time mapping yet reports clock 0 for both: the
/// gates read that as "send now" (unknown clock) and a 0 completion frees on
/// sight.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepBatchClocks {
    /// When the batch's first step can run — its `req_clock`.
    pub start: u64,
    /// When its last step runs — the slot-freeing clock.
    pub completion: u64,
}

/// The clock the send task's gates are judged against: this connection's
/// estimate and dictionary, shared with [`Mcu`] — the two halves
/// [`Mcu::estimated_clock`] reads, as one cloneable handle (the send task
/// cannot borrow the `Mcu` that is being built around it).
#[derive(Clone)]
struct GateClock {
    estimate: Arc<StdMutex<Option<ClockEstimate>>>,
    dictionary: Arc<StdMutex<Option<Arc<Dictionary>>>>,
}

impl GateClock {
    /// The installed dictionary's nominal `CLOCK_FREQ`, if a dictionary is in.
    fn freq(&self) -> Option<f64> {
        self.dictionary
            .lock()
            .expect("dictionary lock poisoned")
            .as_ref()?
            .constant_f64("CLOCK_FREQ")
    }

    /// The firmware clock, extrapolated — [`Mcu::estimated_clock`] answered
    /// from the shared state.
    fn estimated_clock(&self) -> Option<u64> {
        let freq = self.freq()?;
        let estimate = self
            .estimate
            .lock()
            .expect("clock estimate lock poisoned")
            .clone()?;
        Some(estimate.clock_at(Instant::now(), freq))
    }

    /// The gates' "now": the estimated clock and the req lead in ticks.
    fn now(&self) -> Option<GateNow> {
        let clock = self.estimated_clock()?;
        let req_lead = (MIN_REQTIME_DELTA * self.freq()?) as u64;
        Some(GateNow { clock, req_lead })
    }
}

/// A gate as a log line: its number, or `-` when that gate is not set.
fn show_gate(clock: Option<u64>) -> String {
    clock.map_or_else(|| "-".to_string(), |clock| clock.to_string())
}

/// The estimate as a log line: the clock, or `?` when there is none.
fn show_est(clock: Option<u64>) -> String {
    clock.map_or_else(|| "?".to_string(), |clock| clock.to_string())
}

/// Take the next message the gates allow onto the wire: the **lowest**
/// `req_clock` among the released ones, queue order on ties — this is
/// `build_and_send_command` picking the "highest priority message"
/// (`serialqueue.c:478-486`) — and never past a flush barrier.
///
/// A message that had been held reports its release (board, both gates, the
/// estimate it was judged against, and how long it sat): the gates are
/// otherwise invisible, and C5 needs to see which one opened.
fn take_released(
    parked: &mut VecDeque<Parked>,
    now: Option<&GateNow>,
    gates_open: bool,
    board: &str,
) -> Option<Payload> {
    // Only the run before the first barrier may be picked; a barrier is a
    // boundary for everything queued behind it.
    let selectable = parked.iter().take_while(|item| !item.is_flush()).count();
    let (index, _) = parked
        .iter()
        .enumerate()
        .take(selectable)
        .filter(|(_, item)| gates_open || item.clocks().released(now))
        .min_by_key(|(index, item)| (item.clocks().req_clock.unwrap_or(0), *index))?;
    match parked.remove(index) {
        Some(Parked::Payload(msg)) => {
            if msg.held_by != "none" {
                let reason = if gates_open {
                    "gates open"
                } else if now.is_none() {
                    "clock unknown"
                } else {
                    msg.held_by
                };
                debug!(
                    "[{board}] gate open: released after {:.0} ms (was held by {reason}) — \
                     min={} req={} est={}",
                    msg.parked_at.elapsed().as_secs_f64() * 1000.,
                    show_gate(msg.clocks.min_clock),
                    show_gate(msg.clocks.req_clock),
                    show_est(now.map(|now| now.clock)),
                );
            }
            Some(msg.payload)
        }
        // Unreachable: `index` came from the prefix before any barrier.
        Some(Parked::Flush(done, queued_at)) => {
            parked.insert(index, Parked::Flush(done, queued_at));
            None
        }
        None => None,
    }
}

/// Queue one message for the gates, and report the hold: a message the gates
/// keep back is the one worth a line — the rest releases on sight — and that
/// line is the park half of the park/release pair (C5's observability).
fn park_payload(
    parked: &mut VecDeque<Parked>,
    payload: Payload,
    clocks: SendClocks,
    now: Option<&GateNow>,
    gates_open: bool,
    board: &str,
) {
    let held_by = if gates_open {
        "none"
    } else {
        clocks.held_by(now)
    };
    if held_by != "none" {
        debug!(
            "[{board}] gate park: min={} req={} est={} held_by={held_by}",
            show_gate(clocks.min_clock),
            show_gate(clocks.req_clock),
            show_est(now.map(|now| now.clock)),
        );
    }
    parked.push_back(Parked::Payload(ParkedMsg {
        payload,
        clocks,
        parked_at: Instant::now(),
        held_by,
    }));
}

/// Queue a flush barrier behind what is parked, saying how far back it has to
/// wait (`Mcu::flush`'s half of the pair: a barrier covers the parked messages
/// and therefore waits with them).
fn park_flush(parked: &mut VecDeque<Parked>, done: oneshot::Sender<()>, board: &str) {
    debug!(
        "[{board}] gate flush: barrier queued behind {} parked message(s)",
        parked.len()
    );
    parked.push_back(Parked::Flush(done, Instant::now()));
}

/// When the send task should next look at the parked gates: the earliest
/// instant any parked message opens, sliced at [`GATE_REPOLL_MAX`].
///
/// A message opens at the **later** of its two gates (both have to pass), and
/// the list at the earliest of its messages. `None` when nothing is parked or
/// the clock cannot judge it — released messages are already picked by
/// [`take_released`] before this is asked.
fn gate_wake(
    parked: &VecDeque<Parked>,
    gate: &GateClock,
    gates_open: bool,
) -> Option<tokio::time::Instant> {
    if gates_open {
        // Every message releases on sight, so nothing is ever parked.
        return None;
    }
    let now = gate.now()?;
    let freq = gate.freq()?;
    let mut earliest: Option<u64> = None;
    for item in parked {
        let Parked::Payload(msg) = item else {
            continue;
        };
        let clocks = msg.clocks;
        let mut wait: Option<u64> = None;
        if let Some(min_clock) = clocks.min_clock {
            if now.clock < min_clock {
                wait = Some(min_clock - now.clock);
            }
        }
        if let Some(req_clock) = clocks.req_clock {
            let due = req_clock.saturating_sub(now.req_lead);
            if now.clock < due {
                let wait_req = due - now.clock;
                wait = Some(wait.map_or(wait_req, |wait| wait.max(wait_req)));
            }
        }
        if let Some(wait) = wait {
            earliest = Some(earliest.map_or(wait, |earliest| earliest.min(wait)));
        }
    }
    let ticks = earliest?;
    let until = Duration::from_secs_f64(ticks as f64 / freq);
    Some(tokio::time::Instant::now() + until.min(GATE_REPOLL_MAX))
}

/// One MCU's in-flight move pool: the completion clocks of move-class
/// commands sent but not yet executed, kept in completion order.
///
/// The firmware keeps one free list of move nodes **per board** and shuts the
/// board down with "Move queue overflow" when a command arrives and none is
/// free (`basecmd.c:85-90`, `move_alloc`, taken by `queue_step` /
/// `set_next_step_dir` in `stepper.c:262` and by `queue_digital_out` in
/// `gpiocmds.c:148`). The host holds the same count — capacity is the
/// firmware's `move_count`, which the config handshake already checks
/// against the reserved slots (`mcu/config.rs`) — so the next move whose
/// slot is not there yet gets the `min_clock` of the entry that will free
/// one. That is A-lite: no upstream `heap_replace` / `move_clocks` heap, just
/// the deque and its front.
#[derive(Debug, Default)]
struct MoveSlots {
    /// The firmware's `move_count`; `None` until the config handshake arms it
    /// ([`Mcu::set_move_slot_capacity`]), and then no message is held back.
    capacity: Option<usize>,
    /// Completion clocks of the moves still holding a slot, ascending.
    pending: VecDeque<u64>,
}

impl MoveSlots {
    /// The clock at which a move that completes at `completion` counts as
    /// having freed its slot: its completion plus [`SLOT_RELEASE_MARGIN`].
    fn free_at(completion: u64, freq: f64) -> u64 {
        completion + (SLOT_RELEASE_MARGIN * freq) as u64
    }

    /// Take one sent move into the pool and answer with the `min_clock` the
    /// **next** move needs — `None` while a slot is free for it.
    ///
    /// Freed entries drop off the front first (the list is kept ascending, so
    /// the front is the earliest release). The move is then recorded — it is
    /// held back by its floor, not cancelled, so from here it occupies a slot
    /// too — and the floor, once the pool is over capacity, is the completion
    /// clock of the entry `capacity - 1` places back from the end: the move
    /// whose freeing makes room for exactly this one. That is the
    /// `(len - capacity)`-th earliest completion of the whole list
    /// (`pending[len - capacity - 1]` once the new entry is in), which for
    /// in-order completions — steps — is simply the entry `capacity` deep.
    ///
    /// A clock the estimate cannot answer yet (`None`) frees nothing and
    /// imposes nothing — the same "clock unknown, send" the gates open for
    /// (`serialqueue.c:612-618`); the entries are judged as soon as the
    /// estimate exists.
    fn record(&mut self, completion: u64, gate: &GateClock) -> Option<u64> {
        let freq = gate.freq().unwrap_or(0.0);
        if let Some(estimated) = gate.estimated_clock() {
            while let Some(&front) = self.pending.front() {
                if estimated < Self::free_at(front, freq) {
                    break;
                }
                self.pending.pop_front();
            }
        }
        let index = self
            .pending
            .iter()
            .take_while(|&&known| known <= completion)
            .count();
        self.pending.insert(index, completion);
        let capacity = self.capacity?;
        (self.pending.len() > capacity)
            .then(|| self.pending[self.pending.len() - capacity - 1])
            .map(|completion| Self::free_at(completion, freq))
    }

    /// How full the pool is, as the enqueue line reports it: entries waiting
    /// for their completion clocks, and the firmware's `move_count` once the
    /// handshake armed it (`"unarmed"` until then).
    fn report(&self) -> (usize, String) {
        (
            self.pending.len(),
            self.capacity
                .map_or_else(|| "unarmed".to_string(), |capacity| capacity.to_string()),
        )
    }
}

/// Queue `item` for the send task, waiting a bounded time for room.
///
/// This is the synchronous senders' half of the queue's flow control (see
/// [`SYNC_SEND_WAIT`]): it polls the queue's free slots with a doubling backoff
/// instead of awaiting, because its callers cannot await. The payload comes
/// back in the error, which is what the caller reports.
fn try_send_bounded(
    send_buf_tx: &mpsc::Sender<SendItem>,
    mut item: SendItem,
) -> Result<(), TrySendError<SendItem>> {
    let deadline = Instant::now() + SYNC_SEND_WAIT;
    let mut backoff = SYNC_SEND_POLL_START;
    loop {
        match send_buf_tx.try_send(item) {
            Ok(()) => return Ok(()),
            // A closed queue is final: no wait can open it again.
            Err(TrySendError::Closed(returned)) => return Err(TrySendError::Closed(returned)),
            Err(TrySendError::Full(returned)) => {
                if Instant::now() >= deadline {
                    return Err(TrySendError::Full(returned));
                }
                item = returned;
            }
        }
        std::thread::sleep(backoff);
        backoff = (backoff * 2).min(SYNC_SEND_POLL_MAX);
    }
}

/// Whether an awaiting producer ([`Mcu::send_payload`]) may queue another
/// payload: at least [`SYNC_SEND_HEADROOM`] slots have to stay free for the
/// synchronous senders.
fn payload_has_room(send_buf_tx: &mpsc::Sender<SendItem>) -> bool {
    send_buf_tx.capacity() > SYNC_SEND_HEADROOM
}

/// MCU object that represents a physical microcontroller unit.
///
/// Two steps, in this order:
///
/// 1. [`Mcu::new`] brings up the transport. It is infallible, and the result is
///    not usable yet: only the host's identify messages are registered.
/// 2. [`Mcu::connect`] performs the identify handshake, which installs the
///    firmware's data dictionary and thereby makes every command the firmware
///    implements available.
///
/// [`Mcu::install_dictionary`] is the second half of step 2 on its own, for when
/// the dictionary does not come from this MCU.
///
/// The host owns no message formats beyond the identify pair, and the typed
/// command API ([`Mcu::send_msg`] / [`Mcu::call_msg`], defined in [`cmd`](super::cmd)
/// next to the vocabulary they use) refuses to run before the dictionary is in
/// place.
pub struct Mcu {
    /// MCU name
    name: String,
    /// Inbound callbacks, keyed by message id. Kept out of the parser so the
    /// codec can be shared without the callbacks owning the binding resources.
    events: Arc<McuEvents>,
    /// Message parser for communication
    parser: Parser,
    /// The firmware's data dictionary, installed after the identify handshake.
    ///
    /// Protected by a plain mutex rather than an async one: it is only read and
    /// written in short, non-awaiting critical sections, and `send_msg` needs to
    /// check it from a synchronous context. Shared with the send task
    /// ([`GateClock`]), whose gates need the dictionary's `CLOCK_FREQ`.
    dictionary: Arc<StdMutex<Option<Arc<Dictionary>>>>,
    /// Sender for the outbound queue: messages and flush barriers.
    send_buf_tx: mpsc::Sender<SendItem>,
    /// Pending synchronous calls waiting for responses.
    pending_calls: PendingCalls,
    /// Interface clone kept so the device can be shut down on drop.
    interface: Interface,
    /// The runtime this connection's transport tasks run on.
    ///
    /// Taken from the [`Interface`] at construction so the sender task, the
    /// receiver task and the device's own blocking I/O all share one runtime.
    /// It is the machine runtime; a future split from the API runtime (TODO A3)
    /// changes only where the interface picked it up.
    handle: tokio::runtime::Handle,
    /// The sequence state of this connection, shared with both of its tasks: the
    /// number the next block carries, and whether the connection had to take over a
    /// session that was already running (see [`Wire`]).
    wire: Arc<Wire>,
    /// Whether the dictionary is installed, shared with the receive task.
    ///
    /// The task needs it to tell an expected decode miss during identify — the
    /// running firmware's unsolicited `stats`/`shutdown`, which carry ids the
    /// host does not know yet — from a genuinely unknown message afterwards.
    identified: Arc<AtomicBool>,
    /// Whether the scheduling gates ([`SendClocks`]) still hold messages back.
    ///
    /// `true` everywhere a real link runs — and on the `test:` fake transport
    /// it is switched off ([`Mcu::disable_send_gates`]). Shared with the send
    /// task, which is the only reader.
    gates_open: Arc<AtomicBool>,
    /// The stateful estimate of the firmware's free-running clock: clock
    /// round-trip samples folded in, an anchor plus a fitted rate read out.
    /// See [`Mcu::estimated_clock`] and [`ClockEstimate`]; `None` until
    /// something seeds it (the MCU object does, right after identify). Shared
    /// with the send task ([`GateClock`]), whose gates read it.
    clock_estimate: Arc<StdMutex<Option<ClockEstimate>>>,
    /// This MCU's in-flight move pool: which move-class commands still hold a
    /// slot of the firmware's move queue, and therefore the `min_clock` the
    /// next one needs (`MoveSlots`).
    move_slots: StdMutex<MoveSlots>,
    /// Whether [`Mcu::close`] has run: the session is over and nothing may be
    /// queued for the wire any more.
    ///
    /// Set before the tasks are torn down, so a caller racing the abort gets a
    /// refused send rather than a payload queued for a task that will never
    /// read it.
    closed: AtomicBool,
    /// Handle to the send task, so [`Mcu::close`] can stop it directly instead
    /// of waiting for the queue or the ack stream to notice the session is
    /// gone.
    send_handle: StdMutex<Option<tokio::task::JoinHandle<()>>>,
    /// Handle to the receive task, used to abort it on close and on drop.
    recv_handle: StdMutex<Option<tokio::task::JoinHandle<()>>>,
}

impl std::fmt::Debug for Mcu {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The parser, interface, and channels have no useful representation, so
        // report the identifying facts only.
        f.debug_struct("Mcu")
            .field("name", &self.name)
            .field("identified", &self.is_identified())
            .finish_non_exhaustive()
    }
}

/// A message as a log line renders it: the definition's own words with the type
/// specifiers replaced by the values — `set_pin oid=3 value=1`.
///
/// This is Klipper's own debug format (`MessageFormat.format_params` in
/// `klippy/msgproto.py`): the dictionary's parameter names, in declaration
/// order, with the values spliced into the `name=%x` slots. Nothing has to be
/// invented for the log to read well — the definition already says how the
/// message is written.
fn describe_message(msg: &Msg, values: &[ArgValue]) -> String {
    let mut text = msg.name.clone();
    for (index, value) in values.iter().enumerate() {
        let name = msg
            .params
            .get(index)
            .map(|(name, _)| name.as_str())
            .unwrap_or("?");
        text.push_str(&format!(" {name}={}", describe_value(value)));
    }
    text
}

/// One parameter value, as the debug format shows it: integers in decimal, a
/// string quoted and escaped, bytes as a byte-string literal.
///
/// A dynamic string — the `%s`/`%*s`/`%.*s` family — can hold anything,
/// including a space or a newline, so it is quoted the way Klipper reprs it:
/// otherwise a value would run into the format around it and the line could not
/// be read back.
fn describe_value(value: &ArgValue) -> String {
    match value {
        ArgValue::UInt8(v) => v.to_string(),
        ArgValue::UInt16(v) => v.to_string(),
        ArgValue::Int16(v) => v.to_string(),
        ArgValue::UInt32(v) => v.to_string(),
        ArgValue::Int32(v) => v.to_string(),
        ArgValue::Str(v) => describe_str(v),
        ArgValue::Bytes(v) => describe_bytes(v),
    }
}

/// A string in a form that can be read and written back: quoted, with the
/// characters that would run into the format around it escaped.
fn describe_str(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for ch in text.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            ch if ch.is_control() => out.push_str(&format!("\\u{{{:x}}}", ch as u32)),
            ch => out.push(ch),
        }
    }
    out.push('"');
    out
}

/// The same for bytes, which need not be UTF-8: printable ASCII shows through,
/// and everything else becomes `\xNN`.
fn describe_bytes(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() + 3);
    out.push_str("b\"");
    for &byte in bytes {
        match byte {
            b'"' => out.push_str("\\\""),
            b'\\' => out.push_str("\\\\"),
            b'\n' => out.push_str("\\n"),
            b'\r' => out.push_str("\\r"),
            b'\t' => out.push_str("\\t"),
            0x20..=0x7e => out.push(byte as char),
            _ => out.push_str(&format!("\\x{byte:02x}")),
        }
    }
    out.push('"');
    out
}

/// How many blocks may be on the wire unanswered.
///
/// The bound upstream puts on its send window (`third_party/klipper/klippy/chelper/
/// serialqueue.c:109`, `MAX_PENDING_BLOCKS = 12`): an MCU that has stopped
/// answering has to back the host up rather than let the queue of unacknowledged
/// blocks grow without end.
const MAX_PENDING_BLOCKS: usize = 12;

/// The sequence state one connection shares with the firmware.
///
/// Klipper's firmware keeps one `next_sequence` (`third_party/klipper/src/command.c:16`):
/// every frame it sends is stamped with it, a block is accepted only when it
/// carries it, and accepting a block advances it **before** the block is
/// dispatched (`:301-305`). So the two ends share one number, and the firmware's
/// own frames are what say where it is: **the sequence of an empty (ack/nak) frame
/// is the number the next block has to carry** — the ack that follows block N
/// carries N+1, and the nak for a block the firmware would not take carries
/// whatever it is still waiting for.
///
/// The send task owns the number: it is the only writer to the wire. What comes
/// back is where the firmware is, and when that is not what this connection would
/// send next, the firmware is in a session this connection did not open — a board
/// that never rebooted — or a block went missing. Upstream reads the number the
/// same way, and adopts it for the same reason (`serialqueue.c:196-201`: "Got an
/// ack for a message not sent; must be connection init").
#[derive(Debug, Default)]
struct Wire {
    /// The sequence the **next** new block will carry: the firmware's
    /// `next_sequence`, as far as this connection knows.
    ///
    /// Unwrapped into a monotonic counter, because the 4-bit value on the wire
    /// only ever moves forward: a frame that looks behind is really ahead.
    next: AtomicU64,
    /// Set when the connection's first new sequence number showed a firmware
    /// that was already mid-session, i.e. one nothing had reset.
    took_over: AtomicBool,
    /// The firmware's counter as its own frames last reported it, in this
    /// connection's unwrapped numbering: maintained by the receive task, read by
    /// [`Sender::renumber_to_firmware`].
    ///
    /// Where the send side looks when silence turns out to have been a nak.
    /// `next` alone only says what this connection sent last — which is exactly
    /// what an ambiguous empty frame (ack *and* nak carry the same number) cannot
    /// confirm.
    ///
    /// Every frame carries the number, including one the window cannot place and
    /// drops: a frame past `next` is still the firmware saying where it is, and
    /// a renumber that could only read the last *accepted* frame would adopt a
    /// number the firmware has long left behind (see [`place_frame`]).
    seen: AtomicU64,
    /// Set when the window was just renumbered onto the firmware's number
    /// ([`Sender::renumber_to_firmware`]), cleared when the receive task places
    /// the next new number ([`place_frame`]).
    ///
    /// A renumber rewinds `next` to `seen`, so the firmware's next number may be
    /// past anything this connection has sent — not a frame from an earlier
    /// session but *this* connection re-initialising itself, which upstream
    /// adopts rather than drops (`serialqueue.c:196-201`: "Got an ack for a
    /// message not sent; must be connection init"). The exemption is the one the
    /// session's first new number already has (`serialqueue.c:261`,
    /// `receive_seq != 1`): without re-arming it, a window that was rewound can
    /// never place the firmware's answer again — every frame from it is judged
    /// "a block this connection never sent" and dropped, for good.
    connection_init: AtomicBool,
}

/// What one received frame's sequence byte decides for the receive window.
///
/// The pure half of the receive task's sequence handling, so the two rules that
/// decide whether a frame is *placed* or *dropped* can be tested without a
/// transport on the line.
#[derive(Debug, PartialEq, Eq)]
enum Placement {
    /// The frame repeats the number the window is at: no new number, nothing to
    /// adopt, and nothing spent — a firmware whose counter sits at a multiple of
    /// 16 looks exactly like this against a session that starts at 0.
    Repeat,
    /// A new number the window takes. `session_start` says this is the session's
    /// first new number (the takeover test); `ahead_of_window` says it is past
    /// `next`, which is the send task's cue to adopt it and resend
    /// ([`Sender::settle`]).
    Adopt {
        rseq: u64,
        session_start: bool,
        ahead_of_window: bool,
    },
    /// A number past anything this connection sent, with no connection-init
    /// exemption left: the frame is dropped (its payload never reaches the
    /// parser), but its number has already been recorded as where the firmware
    /// says it is.
    Ahead { rseq: u64, next: u64 },
}

/// Place a frame's 4-bit sequence in this connection's unwrapped window.
///
/// The firmware stamps everything it sends with its one counter, and that
/// counter only ever moves forward (`src/command.c:16,208,301-305`): a frame
/// that looks behind is really ahead, so the delta picks the representative in
/// `[seen, seen + 15]`.
///
/// Two exemptions let a number *past* `next` through, both connection init:
///
/// * the session's **first** new number (`seen == 0`, i.e. upstream's
///   `receive_seq == 1` at `serialqueue.c:261`) — how a firmware that never
///   rebooted is taken over;
/// * the first new number after a **renumber** (`wire.connection_init`) — the
///   window was just rewound onto the firmware's number, so its answer arriving
///   is this connection resynchronising, not an answer to a block never sent
///   (`serialqueue.c:196-201`).
///
/// A frame that has no exemption and is past `next` is dropped — and that is
/// where the firmware's number has to be kept anyway: `Wire::seen` is what
/// [`Sender::renumber_to_firmware`] adopts, so a drop that discarded the number
/// would leave every later renumber aiming at a stale window.
fn place_frame(seen: u64, wire: &Wire, frame_seq: u8) -> Placement {
    let delta = (frame_seq.wrapping_sub(seen as u8) & 0xf) as u64;
    if delta == 0 {
        return Placement::Repeat;
    }
    let rseq = seen + delta;
    let next = wire.next.load(Ordering::Relaxed);
    let session_start = seen == 0;
    let exempt = session_start || wire.connection_init.load(Ordering::Relaxed);
    if !exempt && rseq > next {
        // The frame goes, the report does not: the renumber adopts it.
        wire.seen.store(rseq, Ordering::Relaxed);
        return Placement::Ahead { rseq, next };
    }
    wire.seen.store(rseq, Ordering::Relaxed);
    // Any new number spends the exemption that let it through.
    wire.connection_init.store(false, Ordering::Relaxed);
    Placement::Adopt {
        rseq,
        session_start,
        ahead_of_window: rseq > next,
    }
}

/// How long the host waits for an answer before putting the unacknowledged
/// blocks back on the wire, and the ceiling the wait backs off to.
///
/// Upstream computes its retransmit timeout from round-trip samples
/// (`serialqueue.c:218-237`, clamped to these same 25 ms / 5 s). This host
/// does the same: [`Sender::rto`] starts at this floor and moves to the
/// estimate from the first sample on (see [`RttEstimator`], kept in step by
/// [`Sender::record_sample`]).
///
/// The two halves divide the work: **the estimate is the normal wait**, and
/// **doubling off the floor is the lost-packet fallback**. A timeout still
/// doubles the wait up to the ceiling (`:456-460`) so a line that has gone
/// quiet does not hammer the wire, and the next sample the line does yield
/// pulls the wait back to the estimate.
const MIN_RTO: Duration = Duration::from_millis(25);
const MAX_RTO: Duration = Duration::from_secs(5);

/// A round trip longer than this gets the log's attention.
///
/// 25 ms is [`MIN_RTO`], the floor the retransmit timeout is clamped to
/// (`serialqueue.c:233-234`): a line whose round trip outlasts that floor
/// expires the timer before the answer can arrive, so blocks that were never
/// lost go back on the wire.
///
/// One warning at the session's first crossing, then one per doubling of the
/// value that warned (see [`RttWarnState`]) — a line that sits just over the
/// threshold would otherwise log every sample.
const RTT_WARN_THRESHOLD: Duration = MIN_RTO;

/// The least the four deviations may add to the smoothed round trip when the
/// timeout is derived from the estimate (`serialqueue.c:229-231`,
/// `rttvar4 < 0.001` → `0.001`): a line that has settled has a near-zero
/// variance, and the timeout still has to cover the jitter that number cannot
/// see.
const RTTVAR_FLOOR: Duration = Duration::from_millis(1);

/// One block that went out and is waiting for its answer.
#[derive(Debug)]
struct InFlightBlock {
    /// The block's sequence number, unwrapped (see [`Wire`]).
    seq: u64,
    /// The frame exactly as it went out: a retransmit sends it again as it is,
    /// sequence and all (see [`Sender::resend_block`]).
    frame: Frame,
    /// When the write completed — the start of the round trip an ack covering
    /// this block closes (see [`RttEstimator`]).
    sent_at: Instant,
}

/// The connection's round-trip estimate, fed from the blocks above
/// (`third_party/klipper/klippy/chelper/serialqueue.c:218-237`).
///
/// A **sample** is one round trip: the moment a block finished going out to
/// the moment an ack covering it came back. Upstream measures exactly that
/// (`:525` stamps `receive_time` when the block is built, `:209` reads it back
/// for the ack that answers the block, `:218-219` decides a sample is due),
/// and throws the sample away if the block had to be retransmitted first
/// (`:463`, `rtt_sample_seq = 0`) — a resent block measures the line, not the
/// traffic on it.
///
/// The estimator itself is pure: [`rtt_step`] and [`rtt_rto`] do the
/// arithmetic, [`RttEstimator::record`] only stores what they return. The
/// retransmit wait ([`Sender::rto`]) is driven from it —
/// [`Sender::record_sample`] folds each sample in and brings the wait along —
/// while the warning keeps firing off the sample itself.
#[derive(Debug)]
struct RttEstimator {
    /// The most recent sample, `None` until the first one: the raw
    /// measurement, as opposed to the smoothed value below.
    last_sample: Option<Duration>,
    /// The smoothed round trip (`srtt`), `None` until the first sample.
    srtt: Option<Duration>,
    /// The mean deviation (`rttvar`), zero until the first sample.
    rttvar: Duration,
    /// What the estimate says a retransmit timeout should be, clamped to
    /// [`MIN_RTO`]..=[`MAX_RTO`] as upstream clamps its own.
    rto: Duration,
}

impl RttEstimator {
    fn new() -> Self {
        Self {
            last_sample: None,
            srtt: None,
            rttvar: Duration::ZERO,
            rto: MIN_RTO,
        }
    }

    /// Take one round trip into the estimate.
    fn record(&mut self, sample: Duration) {
        let (srtt, rttvar) = rtt_step(self.srtt, self.rttvar, sample);
        self.last_sample = Some(sample);
        self.srtt = Some(srtt);
        self.rttvar = rttvar;
        self.rto = rtt_rto(srtt, rttvar);
    }
}

/// One step of the estimate over a sample, returning the new `(srtt, rttvar)`
/// (`serialqueue.c:222-227`).
///
/// The first sample starts deliberately conservatively — upstream's
/// `srtt = delta * 10.0`, commented "use a higher start default" — so a single
/// lucky sample on a fresh connection cannot send the estimate straight to the
/// floor. Later samples smooth: the deviation keeps three quarters of itself
/// plus a quarter of the new error, the average seven eighths of itself plus
/// one eighth of the sample.
fn rtt_step(srtt: Option<Duration>, rttvar: Duration, sample: Duration) -> (Duration, Duration) {
    match srtt {
        None => (sample * 10, sample / 2),
        Some(srtt) => {
            // `|srtt − δ|`, without a subtraction that could go negative.
            let diff = if srtt >= sample {
                srtt - sample
            } else {
                sample - srtt
            };
            ((srtt * 7 + sample) / 8, (rttvar * 3 + diff) / 4)
        }
    }
}

/// The retransmit timeout an `(srtt, rttvar)` pair implies
/// (`serialqueue.c:229-236`): the smoothed round trip plus four deviations —
/// never less than [`RTTVAR_FLOOR`], never outside [`MIN_RTO`]..=[`MAX_RTO`].
fn rtt_rto(srtt: Duration, rttvar: Duration) -> Duration {
    (srtt + (rttvar * 4).max(RTTVAR_FLOOR)).clamp(MIN_RTO, MAX_RTO)
}

/// Whether a round trip still deserves the log's attention
/// (see [`RTT_WARN_THRESHOLD`]).
///
/// One warning at the first crossing, then one per doubling of the value that
/// warned: a line that stays just over the threshold logs once, and one that
/// keeps getting worse logs again only once it has got twice as bad.
#[derive(Debug, Default)]
struct RttWarnState {
    /// The round trip that last warned; twice it is the next bar.
    warned_at: Option<Duration>,
}

impl RttWarnState {
    /// Whether `new_rtt` crosses the bar. The first crossing always does; a
    /// later one only when it has doubled since the last warning. Dropping
    /// back under the threshold re-arms nothing, so coming back up short of a
    /// doubling stays quiet.
    fn should_warn(&mut self, new_rtt: Duration) -> bool {
        if new_rtt <= RTT_WARN_THRESHOLD {
            return false;
        }
        match self.warned_at {
            None => self.warned_at = Some(new_rtt),
            Some(warned) if new_rtt >= warned * 2 => self.warned_at = Some(new_rtt),
            Some(_) => return false,
        }
        true
    }
}

/// The warning text: the measured round trip and where to look first.
fn rtt_warn_message(sample: Duration) -> String {
    format!(
        "RTT {sample:?} 超过 {RTT_WARN_THRESHOLD:?} 阈值：检查串口桥延迟 latency_timer / 波特率 / 线缆"
    )
}

/// How many clock round-trip samples [`ClockEstimate::record`] fits the
/// frequency over — a plain sliding window where upstream weighs an endless
/// sample stream with an EWMA (`DECAY = 1/30`, `klippy/clocksync.py:9`, i.e.
/// about thirty samples of memory). The fit computes the same
/// covariance-over-variance ratio upstream derives at
/// `klippy/clocksync.py:128` (`clock_covariance / time_variance`); the window
/// only changes how the past is forgotten — the oldest sample is pushed out
/// rather than decayed.
const CLOCK_FIT_WINDOW: usize = 30;

/// One clock round trip feeding the estimate: when the request went out, when
/// the answer came back, and the 64-bit firmware clock the answer reported.
#[derive(Debug, Clone, Copy)]
struct ClockSample {
    sent: Instant,
    received: Instant,
    clock: u64,
}

impl ClockSample {
    /// When the reported reading was taken, in host time: the round trip's
    /// midpoint (`sent + ½ RTT`).
    ///
    /// The firmware stamps its clock while the request is in flight, so
    /// neither end of the trip is where the reading belongs: anchoring on
    /// `received` places it a full round trip late, on `sent` a full one
    /// early. Half is also the shift upstream applies to its anchor
    /// (`time_avg + min_half_rtt`, `klippy/clocksync.py:134-135`), though it
    /// takes the *smallest* half trip it has seen rather than this sample's.
    /// This estimator takes the midpoint of **its own** sample instead of
    /// borrowing the transport's RTT estimate for two reasons: that estimate
    /// measures ack round trips of arbitrary blocks — retransmits, coalesced
    /// batches, a different population from a clock query — and it lives in
    /// the send task, where the clock paths cannot reach it.
    fn midpoint(&self) -> Instant {
        self.sent + self.received.saturating_duration_since(self.sent) / 2
    }
}

/// The least-squares slope of `clock` against time over one window of
/// `(midpoint_seconds, clock)` pairs: upstream's
/// `clock_covariance / time_variance` (`klippy/clocksync.py:128`) read off a
/// plain window of samples instead of an EWMA over all of them.
///
/// `None` when the window cannot say: fewer than two samples, or every one of
/// them at the same instant (zero variance in time).
fn fit_freq(window: &[(f64, f64)]) -> Option<f64> {
    if window.len() < 2 {
        return None;
    }
    let count = window.len() as f64;
    let mean_time = window.iter().map(|(time, _)| time).sum::<f64>() / count;
    let mean_clock = window.iter().map(|(_, clock)| clock).sum::<f64>() / count;
    let mut covariance = 0.0;
    let mut variance = 0.0;
    for (time, clock) in window {
        let time_diff = time - mean_time;
        covariance += time_diff * (clock - mean_clock);
        variance += time_diff * time_diff;
    }
    if variance <= 0.0 {
        return None;
    }
    Some(covariance / variance)
}

/// The stateful host-side estimate of one MCU's free-running clock: round-trip
/// samples go in, an anchor plus a fitted rate come out, and
/// [`Mcu::estimated_clock`] extrapolates from those.
///
/// This replaces the single `(Instant, u64)` snapshot the estimate used to
/// be. The snapshot answered "roughly what clock is it now" only while the
/// firmware ticked at exactly the dictionary's nominal `CLOCK_FREQ`; the
/// samples let the host measure the crystal's real rate and place each
/// reading at its round trip's midpoint (see [`ClockSample::midpoint`]).
///
/// Not the same estimator as `cmd::clock::ClockEstimator`: that one maps the
/// reactor's time onto print time for the primary/secondary sync; this one is
/// the MCU's own 64-bit clock, the number [`Mcu::estimated_clock`] reports.
#[derive(Debug, Clone)]
struct ClockEstimate {
    /// The samples the fit runs over, oldest first, at most
    /// [`CLOCK_FIT_WINDOW`] of them.
    samples: VecDeque<ClockSample>,
    /// The anchor: firmware clock `anchor_clock` belongs to host instant
    /// `anchor_at`; every read extrapolates from there (upstream's
    /// `clock_est`, `klippy/clocksync.py:134-135`).
    anchor_at: Instant,
    anchor_clock: u64,
    /// The fitted rate in ticks per second, `None` until the window can say
    /// (see [`fit_freq`]); readers fall back to the dictionary's nominal
    /// frequency.
    freq: Option<f64>,
}

impl ClockEstimate {
    /// An estimate anchored by one reading taken at `at`, with no round trip
    /// of its own to place it by — what a seed (`Mcu::set_clock_base`) is:
    /// the caller read the clock after the answer came back, so the reading
    /// is taken exactly at that instant, as the old single-point snapshot
    /// did. The window starts empty; clock round trips fill it.
    fn seeded(at: Instant, clock: u64) -> Self {
        Self {
            samples: VecDeque::new(),
            anchor_at: at,
            anchor_clock: clock,
            freq: None,
        }
    }

    /// An estimate built from one clock round trip, before any seed: the
    /// sample anchors the estimate at its midpoint and starts the window.
    fn from_sample(sample: ClockSample) -> Self {
        let mut estimate = Self::seeded(sample.midpoint(), sample.clock);
        estimate.record(sample);
        estimate
    }

    /// Fold one clock round trip in: it joins the fit window (the oldest
    /// sample is pushed out past [`CLOCK_FIT_WINDOW`]), and it becomes the
    /// anchor, at its midpoint.
    fn record(&mut self, sample: ClockSample) {
        self.samples.push_back(sample);
        while self.samples.len() > CLOCK_FIT_WINDOW {
            self.samples.pop_front();
        }
        let origin = self.samples.front().expect("a sample was just pushed").sent;
        let window: Vec<(f64, f64)> = self
            .samples
            .iter()
            .map(|sample| {
                (
                    sample
                        .midpoint()
                        .saturating_duration_since(origin)
                        .as_secs_f64(),
                    sample.clock as f64,
                )
            })
            .collect();
        // A window that cannot say (every sample at one instant) keeps the
        // fit it had rather than dropping back to nominal.
        if let Some(freq) = fit_freq(&window) {
            self.freq = Some(freq);
        }
        let last = self.samples.back().expect("a sample was just pushed");
        self.anchor_at = last.midpoint();
        self.anchor_clock = last.clock;
    }

    /// The firmware clock at host time `at`: `anchor_clock + elapsed × freq`,
    /// with `nominal_freq` standing in until the window has fitted a rate.
    fn clock_at(&self, at: Instant, nominal_freq: f64) -> u64 {
        let freq = self.freq.unwrap_or(nominal_freq);
        let elapsed = at.saturating_duration_since(self.anchor_at).as_secs_f64();
        self.anchor_clock + (elapsed * freq) as u64
    }
}

/// The send task's side of a connection.
///
/// It is the only writer to the wire, so the sequence and the blocks waiting to be
/// acknowledged live here and nowhere else.
struct Sender {
    /// The sequence state, shared with the receive side (see [`Wire`]).
    wire: Arc<Wire>,
    /// Blocks that went out and have not been answered, oldest first. Their
    /// payloads are what a nak — or a number from an earlier session — has to put
    /// back on the wire, and their write times are where a round trip starts
    /// (see [`RttEstimator`]).
    in_flight: VecDeque<InFlightBlock>,
    /// The block whose ack closes the next round-trip sample: set to the first
    /// block written while no sample is pending, dropped when that ack arrives
    /// or as soon as anything is retransmitted. Upstream's `rtt_sample_seq`
    /// (`serialqueue.c:528-529`, consumed at `:237`, dropped at `:463`) — the
    /// drop is the point: a block that had to be sent again measures the
    /// retransmit, not the line.
    sample_seq: Option<u64>,
    /// The round-trip estimate built from the blocks above, and whether this
    /// session has warned about it yet (see [`RTT_WARN_THRESHOLD`]).
    rtt: RttEstimator,
    rtt_warn: RttWarnState,
    /// The last ack/nak this connection acted on, and the value a retransmit has
    /// already been done for: the firmware repeats its ack/nak for as long as it
    /// waits, and one retransmit per value is what upstream allows
    /// (`serialqueue.c:451-454`, `ignore_nak_seq`).
    acked: Option<u64>,
    retransmitted: Option<u64>,
    /// How long to wait for an answer before retransmitting: the estimate's
    /// value once a sample has landed (see [`Sender::record_sample`]), the
    /// [`MIN_RTO`] floor until then, and a timeout's doubling on top of
    /// either until the next sample arrives (see [`MIN_RTO`] for the split).
    rto: Duration,
    /// When the unanswered blocks should go out again, `None` when there are
    /// none. The send task's `select!` arms on it, so a block the firmware never
    /// answered does not wait for the firmware to speak first.
    retransmit_at: Option<tokio::time::Instant>,
}

impl Sender {
    fn new(wire: Arc<Wire>) -> Self {
        Self {
            wire,
            in_flight: VecDeque::new(),
            sample_seq: None,
            rtt: RttEstimator::new(),
            rtt_warn: RttWarnState::default(),
            acked: None,
            retransmitted: None,
            rto: MIN_RTO,
            retransmit_at: None,
        }
    }

    /// When the unacknowledged blocks should be sent again, if any are.
    fn retransmit_deadline(&self) -> Option<tokio::time::Instant> {
        self.retransmit_at
    }

    /// Arm the retransmit timer from now with the current wait: the
    /// estimate's value, or the backed-off one a timeout left behind (see
    /// [`Sender::rto`]).
    fn arm_retransmit(&mut self) {
        self.retransmit_at = Some(tokio::time::Instant::now() + self.rto);
    }

    /// Whether the window is full: the host has to wait for answers before it puts
    /// another block on the wire.
    fn is_full(&self) -> bool {
        self.in_flight.len() >= MAX_PENDING_BLOCKS
    }

    /// The most recent round trip measured on this connection, `None` until the
    /// first one lands.
    ///
    /// Read-only: the send task is the only writer. What the retransmit timer
    /// consumes is the *estimate*, brought in step by
    /// [`Sender::record_sample`]; the raw sample stays an observation.
    fn rtt(&self) -> Option<Duration> {
        self.rtt.last_sample
    }

    /// The smoothed round trip (`srtt`), `None` until the first sample lands.
    ///
    /// Read-only, like [`Sender::rtt`]: upstream derives its retransmit
    /// timeout from this and the deviation (`serialqueue.c:229-236`).
    fn srtt(&self) -> Option<Duration> {
        self.rtt.srtt
    }

    /// The retransmit timeout the estimate implies, already clamped to
    /// [`MIN_RTO`]..=[`MAX_RTO`].
    ///
    /// Read-only, like [`Sender::rtt`]. It is what [`Sender::rto`] waits out
    /// once a sample has landed; that wait differs from it only where a
    /// timeout has doubled the value, until the next sample pulls it back.
    fn estimated_rto(&self) -> Duration {
        self.rtt.rto
    }

    /// Take one round trip into the estimate and bring the retransmit wait
    /// along with it.
    ///
    /// This is the pull-back half of the split described in [`MIN_RTO`]: a
    /// wait a timeout backed off is the fallback, and the next sample the line
    /// yields sets it straight to the estimate again.
    fn record_sample(&mut self, sample: Duration) {
        self.rtt.record(sample);
        self.sync_rto_to_estimate();
    }

    /// Bring the retransmit wait back to what the estimate says: [`MIN_RTO`]
    /// while no sample has landed, the estimator's clamped value from then on.
    ///
    /// A success or a renumber starts the wait over **here**, not at the floor:
    /// how long the next unanswered block may take is what the line has
    /// measured, not 25 ms by default.
    fn sync_rto_to_estimate(&mut self) {
        self.rto = self.estimated_rto();
    }

    /// Put one block on the wire, carrying the sequence this connection is at, and
    /// remember it until the firmware answers it.
    async fn send_block(&mut self, interface: &Interface, payload: Vec<u8>) {
        // The counter is unwrapped here and only its low four bits go on the wire
        // (`Frame` keeps what it is given and masks when encoding), but the frame
        // this connection remembers has to carry those four bits too: that is what
        // a frame parsed off the wire carries, and the two are compared.
        let seq = self.wire.next.load(Ordering::Relaxed);
        let frame = Frame::new((seq & 0xf) as u8, payload);
        // Advance **before** writing. The firmware can answer while the write is
        // still in flight, and the receive side reads this number to tell an answer
        // to a block this connection sent from one that belongs to an earlier
        // session: with the number still one behind, a perfectly good response
        // would look like it answered a block that was never sent. Upstream orders
        // it the same way (`serialqueue.c`: `build_and_send_command` advances
        // `send_seq`, its caller writes the block).
        self.wire.next.store(seq + 1, Ordering::Relaxed);
        match interface.send(frame.clone()).await {
            Ok(()) => {
                // The round trip starts when the block is fully out
                // (`serialqueue.c:525`, `receive_time`).
                let sent_at = Instant::now();
                self.in_flight.push_back(InFlightBlock {
                    seq,
                    frame,
                    sent_at,
                });
                // The first block written while nothing is pinned is what the
                // next sample is measured against (`serialqueue.c:528-529`).
                if self.sample_seq.is_none() {
                    self.sample_seq = Some(seq);
                }
                self.arm_retransmit();
            }
            Err(e) => {
                // Nothing went out, so the number was not used after all.
                error!("Send failed (seq={}): {e}", seq & 0xf);
                self.wire.next.store(seq, Ordering::Relaxed);
            }
        }
    }

    /// Put one block back on the wire **as it was**, sequence and all.
    ///
    /// That is what the firmware's numbering asks for: it is waiting for exactly
    /// that block, and a block it had already taken would be answered with a nak
    /// instead of being run twice.
    async fn resend_block(&mut self, interface: &Interface, seq: u64, frame: Frame) {
        match interface.send(frame.clone()).await {
            Ok(()) => {
                debug!("Block {seq} sent again");
                self.in_flight.push_back(InFlightBlock {
                    seq,
                    frame,
                    sent_at: Instant::now(),
                });
                // Whatever round trip was in progress is over: a block that had
                // to be put back on the wire measures the retransmit, not the
                // line, so the pinned sample dies with it
                // (`serialqueue.c:463`, `rtt_sample_seq = 0`).
                self.sample_seq = None;
                self.arm_retransmit();
            }
            Err(e) => error!("Retransmit failed (seq={}): {e}", frame.seq()),
        }
    }

    /// Put every unacknowledged block back on the wire, as the retransmit timer
    /// asks for.
    ///
    /// Upstream resends the whole pending queue on one timeout
    /// (`serialqueue.c:441-446`) and doubles the wait (`:456-460`); the blocks
    /// keep their sequences so the firmware, which is waiting for the oldest,
    /// takes them in order.
    ///
    /// The doubling is the lost-packet fallback of the split in [`MIN_RTO`],
    /// not the way the wait is normally chosen: it runs off the current value
    /// — estimate-backed or already backed off — up to [`MAX_RTO`], and the
    /// next sample ([`Sender::record_sample`]) replaces it with the estimate.
    async fn retransmit(&mut self, interface: &Interface, board: &str) {
        let again: Vec<InFlightBlock> = self.in_flight.drain(..).collect();
        self.retransmit_at = None;
        if again.is_empty() {
            return;
        }
        warn!(
            "[{board}] No answer for {} in-flight block(s); retransmitting",
            again.len()
        );
        for block in again {
            self.resend_block(interface, block.seq, block.frame).await;
        }
        self.rto = (self.rto * 2).min(MAX_RTO);
    }

    /// Take in what the firmware's counter has been seen at.
    ///
    /// `seen` is that number in this connection's unwrapped sequence. Everything
    /// below it is a block the firmware accepted. What is left is one of two
    /// things, told apart by which side of `next` the number is on:
    ///
    /// * **ahead of `next`** — the number comes from a session that was already
    ///   running; only the connection's first frame can be (see [`Wire`]). Adopt
    ///   it, and renumber and resend everything unacknowledged: the nak that
    ///   carried it is proof the firmware never ran those blocks, while a block it
    ///   did run would come back as a nak rather than run twice.
    /// * **not newer than the last one** — the same ack/nak again, i.e. the
    ///   firmware is still waiting for the block this connection sent and it never
    ///   arrived. Put the unacknowledged blocks back **as they are**: their
    ///   sequences are the ones being waited for.
    ///
    /// The one number this deliberately never acts on by itself is the
    /// **ambiguity band**: `seen` landing exactly on `next` is how the ack of
    /// *every* block looks, and the nak of a block the firmware never took
    /// carries that very same number — one 5-byte empty frame cannot say which
    /// it is. Guessing here would either renumber a healthy session or read a
    /// nak as an ack, so the band is left to the caller that can watch the line
    /// fall silent after its request: identify's retry adopts the number through
    /// [`Mcu::renumber_to_firmware`] instead.
    async fn settle(&mut self, interface: &Interface, seen: u64) {
        let next = self.wire.next.load(Ordering::Relaxed);
        if seen > next {
            // First, because the numbering is not shared yet: nothing the firmware
            // has said about its own counter can say anything about the blocks this
            // connection numbered by itself — in particular it must not be read as
            // "those blocks were accepted", or the block that has to go out again
            // would be dropped instead.
            debug!("Firmware is at {seen}, this connection would send {next}: taking it over");
            self.wire.next.store(seen, Ordering::Relaxed);
            self.acked = None;
            self.retransmitted = None;
            // The pinned sample belongs to the window being abandoned; the blocks
            // go out again under the adopted numbers, and the first of those
            // writes pins the next one.
            self.sample_seq = None;
            let again: Vec<InFlightBlock> = self.in_flight.drain(..).collect();
            for block in again {
                self.send_block(interface, block.frame.payload().to_vec())
                    .await;
            }
            return;
        }

        // The numbering is shared, so the firmware's counter says which blocks it
        // took: everything below it. The last of them is the block this ack
        // directly answers, so its write time is where the round trip it closes
        // began (`serialqueue.c:209`, `last_receive_sent_time`).
        let mut acked_tx: Option<Instant> = None;
        while self.in_flight.front().is_some_and(|block| block.seq < seen) {
            let block = self.in_flight.pop_front().expect("checked just above");
            debug!("Block {} acknowledged", block.seq);
            acked_tx = Some(block.sent_at);
        }
        if self.in_flight.is_empty() {
            // Everything is answered, so there is nothing to retransmit, and
            // the wait starts over at the estimate — the floor only while no
            // sample has landed, never a reset back to it after a success.
            self.retransmit_at = None;
            self.sync_rto_to_estimate();
        }

        // One sample per pinned block, and only if the ack actually covers it
        // (`serialqueue.c:218-219`): a sample is a round trip, so it needs both
        // ends — the write just popped and the ack that came in now.
        if let (Some(anchor), Some(sent_at)) = (self.sample_seq, acked_tx) {
            if seen > anchor {
                self.sample_seq = None;
                let sample = Instant::now().saturating_duration_since(sent_at);
                self.record_sample(sample);
                if self.rtt_warn.should_warn(sample) {
                    warn!("{}", rtt_warn_message(sample));
                }
                debug!(
                    "Round trip {:?}: srtt {:?}, estimated rto {:?}",
                    self.rtt(),
                    self.srtt(),
                    self.estimated_rto()
                );
            }
        }

        match self.acked {
            Some(previous) if seen <= previous => {
                // The firmware saying the same thing again is a nak
                // (`serialqueue.c:291-293`): what it is waiting for never arrived.
                if self.retransmitted != Some(seen) {
                    self.retransmitted = Some(seen);
                    let again: Vec<InFlightBlock> = self.in_flight.drain(..).collect();
                    for block in again {
                        self.resend_block(interface, block.seq, block.frame).await;
                    }
                }
            }
            _ => {
                self.acked = Some(seen);
                self.retransmitted = None;
            }
        }
    }

    /// Adopt the sequence the firmware last reported, abandoning what is in flight.
    ///
    /// Connection init: an answer this connection's window cannot place says
    /// where the firmware really is (`serialqueue.c:196-201`, "Got an ack for a
    /// message not sent; must be connection init"), and the firmware's number is
    /// the one the next block has to carry. It is also where the receive side
    /// needs an exemption of its own: the window has just moved back to the
    /// firmware's number, so its answer arriving next can be past what this
    /// connection has sent — an answer the receive task would otherwise drop as
    /// one to a block never sent, leaving the retry answered by a window that can
    /// no longer hear it (see [`place_frame`], `connection_init`).
    ///
    /// [`Sender::settle`] deliberately does **not** do this on an empty frame
    /// alone: `seen == next` is both the ack of a healthy block and the nak of
    /// one the firmware never took. Only the caller that watched the line stay
    /// silent after its request — identify's retry (see
    /// [`Mcu::renumber_to_firmware`]) — is entitled to read the number as a nak.
    /// What is in flight goes: before the dictionary is installed the only blocks
    /// on the wire are earlier attempts at the very chunk being retried, and the
    /// retry re-issues it.
    fn renumber_to_firmware(&mut self) {
        let seen = self.wire.seen.load(Ordering::Relaxed);
        let next = self.wire.next.load(Ordering::Relaxed);
        if seen != next {
            debug!("Renumbering: firmware waits for {seen}, this connection would send {next}");
        }
        self.wire.next.store(seen, Ordering::Relaxed);
        // Rewinding the window is what the receive side has to be told about:
        // the firmware's next number may now be ahead of it, and that frame is
        // this connection initialising itself, not a block this connection never
        // sent (`serialqueue.c:196-201`). Spent by the first new number that
        // arrives (`place_frame`), so it cannot outlive this renumber.
        self.wire.connection_init.store(true, Ordering::Relaxed);
        self.in_flight.clear();
        self.retransmit_at = None;
        self.acked = None;
        self.retransmitted = None;
        // What was pinned belonged to the abandoned window.
        self.sample_seq = None;
        self.sync_rto_to_estimate();
    }
}

impl Mcu {
    /// Create a new MCU from a name and interface.
    fn from_parts(name: String, interface: Interface) -> Self {
        info!("Creating MCU: {name}");
        // The parser starts out knowing only the two formats the host owns; every
        // other format arrives with the firmware dictionary. It is built before the
        // receive task starts, and that task gets a clone of the same registry, so
        // installing a dictionary later needs no restart (see
        // [`Mcu::install_dictionary`]).
        let parser = identify::new_parser();
        let parser_for_task = parser.clone();
        let events = Arc::new(McuEvents::new());
        let events_for_task = Arc::clone(&events);
        let pending_calls = PendingCalls::new();
        let pending_calls_for_task = pending_calls.clone();
        // Both transport tasks run on the interface's runtime, not the ambient
        // one. Cloned once and shared, since `Handle::spawn` only borrows it.
        let handle = interface.handle().clone();

        let wire = Arc::new(Wire::default());
        let identified = Arc::new(AtomicBool::new(false));
        let identified_for_task = Arc::clone(&identified);
        // The gates start shut (holding): `gates_open` reads as "release on
        // sight", and only a fake transport opens them (`Mcu::open_send_gates`)
        // before any motion is queued.
        let gates_open = Arc::new(AtomicBool::new(false));
        let gates_open_for_task = Arc::clone(&gates_open);
        let (send_buf_tx, mut send_buf_rx) = mpsc::channel::<SendItem>(SEND_QUEUE_CAPACITY);
        // Where the firmware's counter has been seen at, one value per ack/nak
        // frame (see `Wire`). A watch channel: only the newest value matters, the
        // receive task must never be held up by the send task, and every change
        // wakes it — including a repeated value, which is a nak (`Sender::settle`).
        let (acks_tx, mut acks_rx) = watch::channel(0u64);
        let interface_for_send = interface.clone();
        let wire_for_send = Arc::clone(&wire);
        // Shared with the send task as one handle: its gates judge a message's
        // min/req clocks against the same estimate `Mcu::estimated_clock` reads
        // (`GateClock`). The dictionary is not installed yet when the task
        // starts — the gates treat that as "clock unknown", which is upstream's
        // `PR_NOW` (`serialqueue.c:612-618`).
        let dictionary = Arc::new(StdMutex::new(None));
        let clock_estimate = Arc::new(StdMutex::new(None));
        let gate_clock = GateClock {
            estimate: Arc::clone(&clock_estimate),
            dictionary: Arc::clone(&dictionary),
        };

        // The name both transport tasks put on their lines: two MCUs share one
        // log, and a frame that could not be placed or a message nobody bound
        // says nothing useful without its board.
        let board = name.clone();
        let board_for_recv = board.clone();
        let send_handle = handle.spawn(async move {
            let mut sender = Sender::new(Arc::clone(&wire_for_send));
            // What the two gates hold back: messages not released yet, and
            // flush barriers queued behind them (`Parked`).
            let mut parked: VecDeque<Parked> = VecDeque::new();
            let gates_open = Arc::clone(&gates_open_for_task);
            'outer: loop {
                // Wait for the first message the gates release — or for the
                // firmware's counter to move, which is what asks for a block to
                // go out again. A flush with nothing queued before it is
                // already satisfied; a flush behind parked messages waits with
                // them.
                let mut payload = loop {
                    if let Some(first) = take_released(
                        &mut parked,
                        gate_clock.now().as_ref(),
                        gates_open.load(Ordering::Relaxed),
                        &board,
                    ) {
                        break first;
                    }
                    if matches!(parked.front(), Some(Parked::Flush(..))) {
                        if let Some(Parked::Flush(done, queued_at)) = parked.pop_front() {
                            debug!(
                                "[{board}] gate flush: barrier satisfied after {:.0} ms with \
                                 nothing parked ahead",
                                queued_at.elapsed().as_secs_f64() * 1000.
                            );
                            let _ = done.send(());
                        }
                        continue;
                    }
                    let wake = gate_wake(&parked, &gate_clock, gates_open.load(Ordering::Relaxed));
                    let wake_at = wake.unwrap_or_else(tokio::time::Instant::now);
                    let deadline = sender.retransmit_deadline();
                    let when = deadline.unwrap_or_else(|| tokio::time::Instant::now() + MAX_RTO);
                    tokio::select! {
                        item = send_buf_rx.recv() => match item {
                            Some(SendItem::Payload(p)) => {
                                let now = gate_clock.now();
                                park_payload(
                                    &mut parked,
                                    p,
                                    SendClocks::default(),
                                    now.as_ref(),
                                    gates_open.load(Ordering::Relaxed),
                                    &board,
                                );
                            }
                            Some(SendItem::Clocked(p, clocks)) => {
                                let now = gate_clock.now();
                                park_payload(
                                    &mut parked,
                                    p,
                                    clocks,
                                    now.as_ref(),
                                    gates_open.load(Ordering::Relaxed),
                                    &board,
                                );
                            }
                            Some(SendItem::Flush(done)) => {
                                if parked.is_empty() {
                                    let _ = done.send(());
                                } else {
                                    park_flush(&mut parked, done, &board);
                                }
                            }
                            Some(SendItem::Renumber(applied)) => {
                                sender.renumber_to_firmware();
                                let _ = applied.send(());
                            }
                            None => break 'outer, // channel closed
                        },
                        changed = acks_rx.changed() => {
                            if changed.is_err() {
                                break 'outer; // receive task gone: the device is shutting down
                            }
                            let seen = *acks_rx.borrow_and_update();
                            sender.settle(&interface_for_send, seen).await;
                        }
                        // Unanswered blocks are put back on the wire rather than
                        // waiting for the firmware to speak: a block that never
                        // arrived leaves it waiting silently (`serialqueue.c`).
                        _ = tokio::time::sleep_until(when), if deadline.is_some() => {
                            sender.retransmit(&interface_for_send, &board).await;
                        }
                        // A parked gate coming due.
                        _ = tokio::time::sleep_until(wake_at), if wake.is_some() => {}
                    }
                };

                // Coalesce more payloads until the batch is full, a flush asks
                // for a boundary, or the line goes idle.
                let mut flush_done = None;
                loop {
                    // Check if buffer is sufficiently full
                    if payload.len() >= MESSAGE_PAYLOAD_MAX * 2 / 3 {
                        break;
                    }

                    // The next released message — lowest `req_clock` first,
                    // never past a flush barrier.
                    if let Some(next) = take_released(
                        &mut parked,
                        gate_clock.now().as_ref(),
                        gates_open.load(Ordering::Relaxed),
                        &board,
                    ) {
                        if payload.try_merge(&next).is_err() {
                            // Merge failed (would exceed max), send current batch first
                            sender
                                .send_block(&interface_for_send, payload.into_raw())
                                .await;
                            // Start new batch with next
                            payload = next;
                        }
                        continue;
                    }
                    // A barrier the gates have let through: send this batch,
                    // then signal it (`Mcu::flush`).
                    if matches!(parked.front(), Some(Parked::Flush(..))) {
                        if let Some(Parked::Flush(done, queued_at)) = parked.pop_front() {
                            debug!(
                                "[{board}] gate flush: barrier let go after {:.0} ms — the \
                                 messages ahead of it are out",
                                queued_at.elapsed().as_secs_f64() * 1000.
                            );
                            flush_done = Some((done, queued_at));
                        }
                        break;
                    }

                    // Wait for more data, a flush, an ack, a gate, or a short idle timeout.
                    let wake = gate_wake(&parked, &gate_clock, gates_open.load(Ordering::Relaxed));
                    let wake_at = wake.unwrap_or_else(tokio::time::Instant::now);
                    tokio::select! {
                        maybe_next = send_buf_rx.recv() => {
                            match maybe_next {
                                Some(SendItem::Payload(next_payload)) => {
                                    let now = gate_clock.now();
                                    park_payload(
                                        &mut parked,
                                        next_payload,
                                        SendClocks::default(),
                                        now.as_ref(),
                                        gates_open.load(Ordering::Relaxed),
                                        &board,
                                    );
                                }
                                Some(SendItem::Clocked(next_payload, clocks)) => {
                                    let now = gate_clock.now();
                                    park_payload(
                                        &mut parked,
                                        next_payload,
                                        clocks,
                                        now.as_ref(),
                                        gates_open.load(Ordering::Relaxed),
                                        &board,
                                    );
                                }
                                Some(SendItem::Flush(done)) => {
                                    if parked.is_empty() {
                                        // Boundary requested with nothing parked
                                        // ahead of it: send this batch now.
                                        flush_done = Some((done, Instant::now()));
                                        break;
                                    }
                                    park_flush(&mut parked, done, &board);
                                }
                                Some(SendItem::Renumber(applied)) => {
                                    // Applied to the window; a batch already being
                                    // coalesced goes out under the adopted number
                                    // (identify queues nothing while it waits).
                                    sender.renumber_to_firmware();
                                    let _ = applied.send(());
                                }
                                None => {
                                    // Channel closed
                                    break;
                                }
                            }
                        }
                        changed = acks_rx.changed() => {
                            if changed.is_err() {
                                break;
                            }
                            let seen = *acks_rx.borrow_and_update();
                            sender.settle(&interface_for_send, seen).await;
                        }
                        _ = sleep(Duration::from_millis(1)) => {
                            // Timeout, send accumulated batch
                            break;
                        }
                        _ = tokio::time::sleep_until(wake_at), if wake.is_some() => {}
                    }
                }

                // An MCU that has stopped answering has to back the host up
                // instead of growing the queue of unacknowledged blocks.
                while sender.is_full() {
                    let deadline = sender.retransmit_deadline();
                    let when = deadline.unwrap_or_else(|| tokio::time::Instant::now() + MAX_RTO);
                    tokio::select! {
                        changed = acks_rx.changed() => {
                            if changed.is_err() {
                                break; // receive task gone: the device is shutting down
                            }
                            let seen = *acks_rx.borrow_and_update();
                            sender.settle(&interface_for_send, seen).await;
                        }
                        _ = tokio::time::sleep_until(when), if deadline.is_some() => {
                            sender.retransmit(&interface_for_send, &board).await;
                        }
                    }
                }

                // send the batched payload to the MCU
                debug!("Sending batch: {} bytes", payload.len());
                sender
                    .send_block(&interface_for_send, payload.into_raw())
                    .await;
                if let Some((done, queued_at)) = flush_done {
                    debug!(
                        "[{board}] gate flush: satisfied after {:.0} ms — its batch is on the \
                         wire",
                        queued_at.elapsed().as_secs_f64() * 1000.
                    );
                    let _ = done.send(());
                }
            }
        });

        let interface_for_recv = interface.clone();
        let wire_for_recv = Arc::clone(&wire);
        let recv_handle = handle.spawn(async move {
            // The firmware's counter in this connection's unwrapped numbering:
            // `0` until a frame carries a new number, and a frame that merely
            // repeats the number so far leaves it at `0` — the same way
            // upstream's `receive_seq` only moves on a new sequence
            // (`serialqueue.c:254-268`). `0` is also this session's first-frame
            // test: no new number taken in yet is what says whether the firmware
            // was already running (`Wire::took_over`).
            let mut seen = 0u64;

            loop {
                let frame = match interface_for_recv.receive().await {
                    Some(frame) => frame,
                    // Device shut down: no more frames will arrive.
                    None => break,
                };

                // Whether this frame's number can be placed in the window, and
                // where — the two connection-init exemptions and the drop are
                // all decided there (`place_frame`).
                match place_frame(seen, &wire_for_recv, frame.seq()) {
                    Placement::Repeat => {}
                    Placement::Ahead { rseq, next } => {
                        // The number is kept as the firmware's last report (the
                        // renumber adopts it), but this frame's payload has no
                        // place in the window: it answers a block this
                        // connection never sent.
                        warn!(
                            "Frame with sequence {rseq} answers block {next} or later, which this \
                             connection never sent; dropping it"
                        );
                        continue;
                    }
                    Placement::Adopt {
                        rseq,
                        session_start,
                        ahead_of_window,
                    } => {
                        seen = rseq;
                        if session_start && rseq > 1 {
                            // A firmware that just booted answers this connection's
                            // first block with 0 or 1 (`src/command.c`, whose counter
                            // starts at 0). Anything else was already running when the
                            // port was opened — a board nothing reset.
                            wire_for_recv.took_over.store(true, Ordering::Relaxed);
                            info!(
                                "Firmware is still in an earlier session (sequence {rseq}); \
                                 taking it over"
                            );
                        }
                        // A data frame is a response, and the ack that follows it (in
                        // the normal flow) carries the same number — so a takeover is
                        // the only thing about a data frame the send task has to know.
                        if ahead_of_window {
                            let _ = acks_tx.send(seen);
                        }
                    }
                }

                // An empty frame is the MCU's ack of the block it took, or its nak
                // of one it would not take. Either way its number is where the
                // firmware is — the same number its response carried — and the send
                // task is the one that acts on it: an ack drops what it proves was
                // accepted, a nak puts it back on the wire.
                if frame.payload().is_empty() {
                    debug!("Ack for block {}", frame.seq());
                    let _ = acks_tx.send(seen);
                    continue;
                }

                let decoded = match parser_for_task.decode(frame.into()) {
                    Ok(msgs) => {
                        debug!("Decoded {} messages", msgs.len());
                        msgs
                    }
                    Err(e) => {
                        // Before the dictionary is installed, the only ids the
                        // parser knows are the identify pair; a firmware that
                        // was already running keeps sending `stats`/`shutdown`,
                        // whose ids are not known yet. That is expected, not an
                        // error — see `identified`.
                        if identified_for_task.load(Ordering::SeqCst) {
                            error!("[{board_for_recv}] Decode error: {e}");
                        } else {
                            debug!("[{board_for_recv}] Decode error before the dictionary: {e}");
                        }
                        continue;
                    }
                };

                for (msg, params) in decoded {
                    debug!(
                        "[{board_for_recv}] recv {}",
                        describe_message(&msg, &params)
                    );
                    // Pending call has priority — if matched, consume and skip callback.
                    if pending_calls_for_task.resolve(&msg.name, &params).await {
                        debug!(
                            "Pending call matched: {} (id={}), delivering {} params",
                            msg.name,
                            msg.id,
                            params.len()
                        );
                        continue;
                    }
                    // No pending call — fall back to the callback bound to
                    // this message id.
                    if let Some(callback) = events_for_task.callback(msg.id) {
                        debug!("Invoking callback for {} (id={})", msg.name, msg.id);
                        let mut cb = callback.lock().unwrap();
                        cb(params.as_slice());
                    } else {
                        // No callback and no pending call — discard with warning.
                        warn!(
                            "[{board_for_recv}] Unhandled message {} (id={}), discarding",
                            msg.name, msg.id
                        );
                    }
                }
            }
        });

        Self {
            name,
            parser,
            events,
            dictionary,
            send_buf_tx,
            pending_calls,
            interface,
            handle,
            wire,
            identified: Arc::clone(&identified),
            gates_open,
            clock_estimate,
            move_slots: StdMutex::new(MoveSlots::default()),
            closed: AtomicBool::new(false),
            send_handle: StdMutex::new(Some(send_handle)),
            recv_handle: StdMutex::new(Some(recv_handle)),
        }
    }

    /// Build the transport for an MCU called `name`: the parser with the host's
    /// identify formats, the send task, and the receive task.
    ///
    /// The result is **not identified**. Until a dictionary is installed, the
    /// only message that can be exchanged is identify itself, and the typed
    /// command API answers [`McuError::NotIdentified`]. [`Mcu::connect`] is the
    /// normal entry point; use this one when the interface has to be brought up
    /// before the handshake — to inspect it, to retry it with a custom timeout,
    /// or because the dictionary comes from somewhere else.
    ///
    /// The two background tasks outlive this call and are stopped by
    /// [`Mcu::close`] — which [`Drop`] calls too, as the backstop for a
    /// session that simply goes away.
    ///
    /// The interface arrives already open: a [`McuConfig`] describes a transport
    /// and `McuConfig::open` opens it, so that a firmware restart can reset the
    /// board on its **closed** port in between (`mcu/restart.rs`).
    pub fn new(name: impl Into<String>, interface: Interface) -> Self {
        Self::from_parts(name.into(), interface)
    }

    /// The runtime this connection's transport tasks run on.
    ///
    /// Taken from the [`Interface`] when it was opened, so the sender task, the
    /// receiver task and the device's own blocking I/O all share it. A caller
    /// that needs to schedule work alongside a connection uses this rather than
    /// the ambient runtime.
    pub fn handle(&self) -> &tokio::runtime::Handle {
        &self.handle
    }

    /// Whether this connection had to take over a session that was already
    /// running.
    ///
    /// A firmware that just booted answers the connection's first block with
    /// sequence 0 or 1, because its counter starts over (`src/command.c:16`).
    /// Anything else is a board that was already mid-session, and the transport
    /// adopts the number the firmware is waiting for and puts the unanswered
    /// blocks back on the wire (see `Wire`, the transport's shared sequence).
    ///
    /// That is the one thing a reconnect can observe to tell "the board rebooted"
    /// from "the board was only disconnected": an `rpi_usb` port switch that does
    /// not really switch power leaves the firmware running, and the caller that
    /// asked for the reset reads this to find out (`mcu/object.rs`).
    pub fn took_over_session(&self) -> bool {
        self.wire.took_over.load(Ordering::Relaxed)
    }

    /// Install the firmware's data dictionary.
    ///
    /// Called by [`Mcu::identify`] once the identify payload has been fetched; it
    /// is public because a dictionary can also arrive from elsewhere, such as a
    /// cached copy or a test fixture. Two things happen:
    ///
    /// * commands and responses are registered with the parser. The receive task
    ///   holds a clone of the same [`Parser`], so decoded messages become
    ///   available immediately — no task restart, no parser rebuild.
    /// * the dictionary itself is retained, for enumerations and constants.
    ///
    /// Returns the number of messages that were newly registered.
    ///
    /// # Errors
    /// Returns [`McuError`] if a format string cannot be parsed, or if an id or
    /// name collides with a different, already-registered message.
    pub fn install_dictionary(&self, dictionary: Dictionary) -> Result<usize, McuError> {
        // `Parser` is a thin handle over shared state, and `register` only needs
        // `&mut` on the handle, so cloning it is enough to install while `&self`
        // is borrowed. The clone shares the registry the receive task uses.
        let mut parser = self.parser.clone();
        let installed = dictionary.install(&mut parser)?;

        let mut slot = self.dictionary.lock().expect("dictionary lock poisoned");
        *slot = Some(Arc::new(dictionary));
        self.identified.store(true, Ordering::SeqCst);
        Ok(installed)
    }

    /// The installed data dictionary, or `None` before the identify handshake.
    pub fn dictionary(&self) -> Option<Arc<Dictionary>> {
        self.dictionary
            .lock()
            .expect("dictionary lock poisoned")
            .clone()
    }

    /// Whether the identify handshake has installed a dictionary.
    pub fn is_identified(&self) -> bool {
        self.dictionary
            .lock()
            .expect("dictionary lock poisoned")
            .is_some()
    }

    /// Require an installed dictionary before running a typed command.
    pub(crate) fn require_dictionary(&self) -> Result<Arc<Dictionary>, McuError> {
        self.dictionary().ok_or(McuError::NotIdentified)
    }

    /// Look up a registered message, failing fast when it is unknown.
    ///
    /// The typed calls in [`crate::core::klippy::cmd`] resolve both names through
    /// this before sending anything, which is what turns an unimplemented message
    /// into an immediate error instead of a timeout.
    pub(crate) fn require_message(&self, name: &str) -> Result<Arc<Msg>, McuError> {
        self.parser
            .lookup(name)
            .ok_or_else(|| McuError::UnknownMessage(name.to_string()))
    }

    /// Whether the firmware declares `name`.
    ///
    /// Older or trimmed firmware may not send every message. Callers that can
    /// degrade use this instead of [`Mcu::require_message`]'s hard failure.
    pub(crate) fn has_message(&self, name: &str) -> bool {
        self.parser.lookup(name).is_some()
    }

    /// Check whether the firmware implements `format`, without failing.
    ///
    /// Upstream's `MCU.try_lookup_command` (`klippy/mcu.py:1197`, wrapping
    /// `MsgParser.lookup_command`, `klippy/msgproto.py:309`): the format's first
    /// token names the message, and the **whole** string must then match the
    /// firmware's declaration byte for byte — parameter names and specifiers
    /// included (`i2c_transfer oid=%c write=%*s read_len=%u`). The dictionary is
    /// where that exact wording survives; the parser indexes by bare name, so it
    /// cannot answer this question.
    ///
    /// Returns `Some(())` on an exact match, `None` otherwise. This is how the
    /// I2C/SPI layers detect legacy vs new command styles at runtime.
    pub fn try_lookup_command(&self, format: &str) -> Option<()> {
        let name = format.split_whitespace().next()?;
        let dictionary = self.dictionary()?;
        match dictionary.message(name) {
            Some(def) if def.format == format => Some(()),
            _ => None,
        }
    }

    /// Get the MCU name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Drop every synchronous call this session has in flight.
    ///
    /// The calls will not be answered any more — the caller is the connect-time
    /// shutdown watcher (`mcu/object.rs`), and a firmware that reported a stop
    /// refuses every command but the handful that run while stopped
    /// (`src/command.c:346-349`). Dropping the registrations returns each waiter
    /// now instead of after its own timeout (see [`PendingCalls::abort_all`]).
    ///
    /// Handed to this session's runtime rather than awaited here: the caller is
    /// a message callback running on the receive task, which must not block
    /// ([`Mcu::bind_event`]).
    pub(crate) fn abort_pending_calls(&self) {
        let calls = self.pending_calls.clone();
        self.handle.spawn(async move { calls.abort_all().await });
    }

    /// The firmware's clock frequency (`CLOCK_FREQ`), in ticks per second.
    ///
    /// Read from the installed dictionary, so it is only available after the
    /// identify handshake. This is what turns host seconds into firmware clock
    /// ticks for every timed command (`queue_*`, `spi_send`, ADC sampling).
    ///
    /// # Errors
    /// Returns [`McuError::NotIdentified`] before the handshake and
    /// [`McuError::Config`] if the dictionary has no `CLOCK_FREQ`.
    pub fn clock_freq(&self) -> Result<f64, McuError> {
        let dictionary = self.require_dictionary()?;
        dictionary
            .constant_f64("CLOCK_FREQ")
            .ok_or_else(|| McuError::Config("dictionary has no CLOCK_FREQ".to_string()))
    }

    /// Convert seconds to firmware clock ticks, as upstream's
    /// `MCU.seconds_to_clock` (`klippy/mcu.py:1184-1185`).
    ///
    /// # Errors
    /// As [`Mcu::clock_freq`].
    pub fn seconds_to_clock(&self, seconds: f64) -> Result<u64, McuError> {
        let freq = self.clock_freq()?;
        Ok((seconds * freq).max(0.0) as u64)
    }

    /// Minimum time the host needs to get scheduled events queued into the
    /// MCU — upstream's `MCU.min_schedule_time` (`klippy/mcu.py:1187-1188`),
    /// which returns `MIN_SCHEDULE_TIME = 0.100` (`mcu.py:13`).
    ///
    /// In this repo that number already exists as [`MIN_REQTIME_DELTA`]
    /// (upstream `serialqueue.c:110`, the lead a gated message wants before
    /// its `req_clock` — the same 0.100 s), so this accessor hands out that
    /// constant rather than keeping a second copy. It exists for upper layers
    /// that need upstream's scheduling-lead definition — e.g. `GCodeRequestQueue`'s
    /// `next_min_flush_time` alignment — without reaching into this module's
    /// internals.
    pub fn min_schedule_time(&self) -> f64 {
        MIN_REQTIME_DELTA
    }

    /// Record the firmware clock read at this moment, so [`Mcu::estimated_clock`]
    /// can extrapolate from it.
    ///
    /// This is the **seed**: one reading at connect, no round trip of its own,
    /// so the estimate is anchored exactly where the reading was taken — the
    /// old single-point behaviour, kept as the starting state. From there
    /// `estimated_clock` extrapolates at the dictionary's nominal frequency
    /// until clock round trips (`Mcu::record_clock_sample`) let the windowed
    /// fit measure the crystal's real rate.
    ///
    /// A seed starts the estimate over: whatever was folded in before is
    /// dropped (a reconnect reads the clock again anyway). Print time is a
    /// separate estimate built on top of this one
    /// (`McuObject::estimated_print_time`).
    pub fn set_clock_base(&self, clock64: u64) {
        *self
            .clock_estimate
            .lock()
            .expect("clock estimate lock poisoned") =
            Some(ClockEstimate::seeded(Instant::now(), clock64));
    }

    /// Fold one clock round trip into the estimate behind
    /// [`Mcu::estimated_clock`]: the host instants that bracket the exchange
    /// and the 64-bit clock the answer reported — the three numbers upstream's
    /// serial queue stamps on every message and its clock sync consumes
    /// (`klippy/clocksync.py:68-100`).
    ///
    /// The sample joins the fit window (its oldest member is pushed out past
    /// `CLOCK_FIT_WINDOW`) and becomes the extrapolation anchor, at its round
    /// trip's midpoint rather than at either end (see `ClockSample::midpoint`).
    /// A seed carries no round trip and goes through `Mcu::set_clock_base`
    /// instead; a sample that arrives before any seed simply becomes the
    /// estimate.
    ///
    /// Called by the clock-read path (`McuClock::get_clock` in `cmd::clock`),
    /// which the `mcu_clock_poll` timer (`McuObject`) drives about once a second
    /// per MCU after the seed.
    pub(crate) fn record_clock_sample(&self, sent: Instant, received: Instant, clock64: u64) {
        let mut slot = self
            .clock_estimate
            .lock()
            .expect("clock estimate lock poisoned");
        let sample = ClockSample {
            sent,
            received,
            clock: clock64,
        };
        match slot.as_mut() {
            Some(estimate) => estimate.record(sample),
            None => *slot = Some(ClockEstimate::from_sample(sample)),
        }
    }

    /// The firmware clock, extrapolated from the estimate's anchor.
    ///
    /// Returns `None` before a base has been recorded, or if the firmware
    /// frequency is unknown. The value is 64-bit; a clocked command carries its
    /// low word. Clocked commands do not advance it (this is not a scheduler), so
    /// two calls close together agree.
    ///
    /// The anchor is the last sample's midpoint (`sent + ½ RTT`, see
    /// `ClockSample::midpoint`), and the rate is the windowed fit when the
    /// window can produce one — the dictionary's nominal `CLOCK_FREQ` before
    /// that, which is exactly what the old single-point snapshot answered
    /// after a seed with no round trips behind it.
    pub fn estimated_clock(&self) -> Option<u64> {
        self.gate_clock().estimated_clock()
    }

    /// The clock handle the send task judges its gates with: this MCU's
    /// estimate and dictionary, shared (`GateClock`).
    fn gate_clock(&self) -> GateClock {
        GateClock {
            estimate: Arc::clone(&self.clock_estimate),
            dictionary: Arc::clone(&self.dictionary),
        }
    }

    /// Open the scheduling gates on this connection: every message releases on
    /// sight instead of waiting for its `min_clock`/`req_clock`
    /// ([`SendClocks::released`] short-circuits, the send task never parks).
    /// Pool accounting keeps running, so the floors are still computed — they
    /// are simply never enforced here.
    ///
    /// **Fake transports only** (`test:` → `interface/devices/simulator.rs`),
    /// called by `McuObject`'s connect path. The simulator's clock only moves
    /// with wall time while the corpus' motion is virtual, so a gate there
    /// degenerates into wall-clock serialisation: the estimate can never lead
    /// the stream it is waiting on (the C4 probes measured it — released
    /// batches only extend the clock by their own span, so `estimated_clock`
    /// stays pinned to the wall and messages wait out the print horizon in
    /// real seconds). Upstream likewise never validates message scheduling
    /// against the corpus: its file output short-circuits those waits
    /// (`is_fileoutput`, e.g. `klippy/mcu.py:403-404`). Production links —
    /// and the `FrameMock` unit tests — keep the gates; nothing sets the flag
    /// there.
    pub(crate) fn open_send_gates(&self) {
        self.gates_open.store(true, Ordering::Relaxed);
    }

    /// Encode a command without sending it.
    ///
    /// The configuration phase needs the bytes before they are queued: it
    /// hashes them into the configuration CRC. Everything else goes through
    /// [`Mcu::send`], which encodes and queues in one step.
    ///
    /// # Errors
    /// Returns [`McuError::Msg`] if the name is unknown or the arguments do not
    /// match the firmware's format string.
    pub(crate) fn encode(&self, name: &str, args: &[ArgValue]) -> Result<Payload, McuError> {
        Ok(self.parser.encode(name, args)?)
    }

    /// Queue an already-encoded payload, waiting for room.
    ///
    /// The send channel is bounded, so a configuration of hundreds of commands
    /// would overflow [`Mcu::send`]'s bounded wait. This is the
    /// blocking-in-the-async-sense counterpart the configuration phase uses.
    ///
    /// # Errors
    /// Returns [`McuError::Msg`] if the send task has gone away.
    pub(crate) async fn send_payload(&self, payload: Payload) -> Result<(), McuError> {
        // Hold back at the headroom so the sync `Mcu::send` keeps slots to
        // land in (see `SYNC_SEND_HEADROOM`); give up waiting only if the
        // channel closed, so the error below still surfaces.
        while !payload_has_room(&self.send_buf_tx) && !self.send_buf_tx.is_closed() {
            sleep(Duration::from_micros(100)).await;
        }
        self.send_buf_tx
            .send(SendItem::Payload(payload))
            .await
            .map_err(|e| McuError::Msg(MsgError::new(e.to_string())))
    }

    /// Encode and send a command to the MCU.
    ///
    /// Converts the command name and arguments to a [`Payload`] using the
    /// registered message format, then queues it for sending. The queue is
    /// bounded and this path cannot await, so it waits [`SYNC_SEND_WAIT`] for
    /// room before it reports the queue as full.
    ///
    /// # Errors
    /// Returns [`MsgError`] if the message name is unknown, the arguments
    /// don't match the expected parameter count, the send task has gone away,
    /// or the send buffer stayed full for the whole wait.
    pub fn send(&self, name: &str, args: &[ArgValue]) -> Result<(), MsgError> {
        self.enqueue(name, args, None, SendClocks::default())
    }

    /// [`Mcu::send`] with scheduling gates ([`SendClocks`]).
    ///
    /// The gates are enforced by the send task, not here: this only stamps
    /// them on the message so `min`/`req` ordering and release happen where the
    /// wire is (`serialqueue.c`'s two gate checks). A message with no gates is
    /// indistinguishable from [`Mcu::send`].
    ///
    /// # Errors
    /// As [`Mcu::send`].
    pub(crate) fn send_with_clocks(
        &self,
        name: &str,
        args: &[ArgValue],
        clocks: SendClocks,
    ) -> Result<(), MsgError> {
        self.enqueue(name, args, None, clocks)
    }

    /// Arm this MCU's move pool at the firmware's `move_count`.
    ///
    /// The capacity comes from the config handshake, where the firmware's
    /// answer has just been checked against the reserved slots
    /// (`mcu/config.rs`, the value reported as `Configured::move_count`).
    /// Until then the pool holds no message back ([`MoveSlots::capacity`]).
    pub(crate) fn set_move_slot_capacity(&self, move_count: u16) {
        self.move_slots
            .lock()
            .expect("move slots lock poisoned")
            .capacity = Some(move_count as usize);
    }

    /// Send a **move-class** command: one whose firmware handler takes a slot
    /// from the board's move free list (`queue_step`, `set_next_step_dir`,
    /// `queue_digital_out` — each `move_alloc()`s, `stepper.c:262` /
    /// `gpiocmds.c:148`).
    ///
    /// The command enters this MCU's pool with `completion_clock` — when its
    /// last step (or its own scheduled instant) runs — which is what later
    /// moves' `min_clock` floors are cut from. Its `req_clock` is
    /// `start_clock`: when the command's own work begins, so it goes out
    /// [`MIN_REQTIME_DELTA`] before that and behind any lower `req_clock` —
    /// upstream stamps both on every step message
    /// (`stepcompress.c:359`, `min_clock = req_clock = last_step_clock`).
    ///
    /// # Errors
    /// As [`Mcu::send`].
    pub(crate) fn send_move(
        &self,
        name: &str,
        args: &[ArgValue],
        start_clock: u64,
        completion_clock: u64,
    ) -> Result<(), MsgError> {
        self.enqueue(
            name,
            args,
            None,
            self.move_clocks(start_clock, completion_clock),
        )
    }

    /// [`Mcu::send_move`] for an already-encoded payload — the stepper's flush
    /// path, where a batch is encoded once and sent with its window
    /// (`McuStepper::send_steps_async`).
    ///
    /// The pool entry is taken here, before the payload waits for room, so the
    /// slot accounting is in step with the queue.
    ///
    /// # Errors
    /// As [`Mcu::send_payload`].
    pub(crate) async fn send_move_payload(
        &self,
        payload: Payload,
        start_clock: u64,
        completion_clock: u64,
    ) -> Result<(), McuError> {
        let clocks = self.move_clocks(start_clock, completion_clock);
        // The gates' inputs as the message enters the queue: this is the only
        // place the step path is visible (it bypasses `Mcu::enqueue`'s `send`
        // line), and the pair it prints — `completion` here, `est` there — is
        // what the release comparison runs on. C5 measured that pair before
        // the print-time floor went live: `completion` sat 27 s behind `est`
        // after 33 s idle and 294 s behind it after 301 s idle — both gates
        // opened on sight and the whole motion went out in one dump.
        let (pending, capacity) = self
            .move_slots
            .lock()
            .expect("move slots lock poisoned")
            .report();
        debug!(
            "[{}] move in: {} bytes, start={} completion={} min={} req={} est={} \
             pool={pending}/{capacity}",
            self.name,
            payload.len(),
            start_clock,
            completion_clock,
            show_gate(clocks.min_clock),
            show_gate(clocks.req_clock),
            show_est(self.gate_clock().estimated_clock()),
        );
        // Same headroom wait as `send_payload`: a long step batch must not
        // starve the synchronous senders (`SYNC_SEND_HEADROOM`).
        while !payload_has_room(&self.send_buf_tx) && !self.send_buf_tx.is_closed() {
            sleep(Duration::from_micros(100)).await;
        }
        self.send_buf_tx
            .send(SendItem::Clocked(payload, clocks))
            .await
            .map_err(|e| McuError::Msg(MsgError::new(e.to_string())))
    }

    /// The gates for one move-class command: the pool's floor for its
    /// `min_clock`, its start clock for `req_clock` (`Mcu::send_move`).
    fn move_clocks(&self, start_clock: u64, completion_clock: u64) -> SendClocks {
        let min_clock = self
            .move_slots
            .lock()
            .expect("move slots lock poisoned")
            .record(completion_clock, &self.gate_clock());
        SendClocks {
            min_clock,
            req_clock: Some(start_clock),
        }
    }

    /// [`Mcu::send_move`] for a typed command — the move-class counterpart of
    /// `Mcu::send_msg` (which lives with the other typed sends in [`cmd`](crate::core::klippy::cmd)).
    ///
    /// # Errors
    /// As [`Mcu::send`], plus [`McuError::NotIdentified`] before the identify
    /// handshake and [`McuError::UnknownMessage`] when the name is absent from
    /// the dictionary.
    pub(crate) fn send_move_msg<C: crate::core::klippy::cmd::McuCommand>(
        &self,
        cmd: &C,
        start_clock: u64,
        completion_clock: u64,
    ) -> Result<(), McuError> {
        self.require_dictionary()?;
        self.require_message(C::NAME)?;
        self.send_move(C::NAME, &cmd.args(), start_clock, completion_clock)?;
        Ok(())
    }

    /// The body of [`Mcu::send`], plus what the caller will be waiting for.
    ///
    /// A call is one round trip, so it gets one line: `[mcu] send identify offset=0
    /// count=40 (waiting for identify_response)` says what went out and what is
    /// expected back, where two lines said half of that each.
    ///
    /// A queue that is still full when [`try_send_bounded`] gives up is
    /// reported with the command and the level it was full at: the bare
    /// `no available capacity` says nothing about which burst spent the queue.
    ///
    /// `clocks` are the message's scheduling gates ([`SendClocks`]); callers
    /// that do not gate use [`Mcu::send`] and never attach them.
    fn enqueue(
        &self,
        name: &str,
        args: &[ArgValue],
        waiting_for: Option<&str>,
        clocks: SendClocks,
    ) -> Result<(), MsgError> {
        // A closed session refuses at once: the send task is already gone, and
        // a message that merely queued would wait for a reader that never comes.
        if self.is_closed() {
            return Err(MsgError::new(format!(
                "{}: the connection is closed",
                self.describe_command(name, args)
            )));
        }
        let payload = self.parser.encode(name, args)?;
        match waiting_for {
            Some(response) => debug!(
                "[{}] send {} (waiting for {response})",
                self.name,
                self.describe_command(name, args)
            ),
            None => debug!("[{}] send {}", self.name, self.describe_command(name, args)),
        }
        // A gated message says what holds it: `min`/`req` against the estimate
        // at the moment it joins the queue (the park line reports the same
        // three numbers once the send task has it).
        if clocks != SendClocks::default() {
            debug!(
                "[{}] gate in: {} min={} req={} est={}",
                self.name,
                self.describe_command(name, args),
                show_gate(clocks.min_clock),
                show_gate(clocks.req_clock),
                show_est(self.gate_clock().estimated_clock()),
            );
        }
        // An ungated message stays on the plain variant: the gate path treats
        // the two the same (`SendClocks::default()` is released on sight), and
        // keeping the existing variant keeps the existing behaviour literal.
        let item = if clocks == SendClocks::default() {
            SendItem::Payload(payload)
        } else {
            SendItem::Clocked(payload, clocks)
        };
        try_send_bounded(&self.send_buf_tx, item).map_err(|e| {
            let level = match &e {
                TrySendError::Full(_) => format!(
                    " (send queue {} of {SEND_QUEUE_CAPACITY} slots in use)",
                    SEND_QUEUE_CAPACITY - self.send_buf_tx.capacity()
                ),
                // The channel's own words are all there is to say about a gone
                // send task; a level would read as a full queue.
                TrySendError::Closed(_) => String::new(),
            };
            MsgError::new(format!("{}: {e}{level}", self.describe_command(name, args)))
        })?;
        Ok(())
    }

    /// Wait until everything queued before this call has been written to the
    /// transport.
    ///
    /// The send task coalesces payloads that are queued close together into one
    /// wire block. That is usually what a caller wants, but not always: a command
    /// that makes the firmware leave the block it is dispatching has to be alone
    /// in its block. Queueing a barrier and waiting here forces that boundary —
    /// payloads queued **after** the barrier are not covered, which is the point.
    ///
    /// This says the bytes reached the transport, **not** that the firmware has
    /// read or acted on them. It is a flush, not an acknowledgement; waiting for
    /// an effect needs the message that reports it (see `mcu/config.rs`).
    ///
    /// # Errors
    /// Returns [`McuError::Call`] if the send task is gone, or if the barrier was
    /// not reached within `timeout` (a wedged transport).
    pub async fn flush(&self, timeout: Duration) -> Result<(), McuError> {
        if self.is_closed() {
            return Err(McuCallError::SendFailed("the connection is closed".to_string()).into());
        }
        let (done_tx, done_rx) = oneshot::channel();
        self.send_buf_tx
            .send(SendItem::Flush(done_tx))
            .await
            .map_err(|e| McuError::Call(McuCallError::SendFailed(e.to_string())))?;
        match tokio::time::timeout(timeout, done_rx).await {
            Ok(Ok(())) => Ok(()),
            Ok(Err(_)) => {
                Err(McuCallError::SendFailed("the send task dropped the flush".to_string()).into())
            }
            Err(_) => {
                Err(McuCallError::Timeout(format!("flush not reached within {timeout:?}")).into())
            }
        }
    }

    /// Renumber the send window onto the sequence the firmware last reported.
    ///
    /// The identify handshake's answer to silence. An empty ack/nak frame and
    /// the ack of a healthy block carry the same number, so when a request is
    /// met with nothing at all, the number that empty frame carried is read as
    /// the nak it probably was: the window adopts it and the request goes out
    /// again under that number (connection init, `serialqueue.c:196-201`).
    /// Blocks still in flight are abandoned — before the dictionary is
    /// installed they are only ever earlier attempts at the same identify
    /// chunk, which the caller re-issues
    /// ([`Identify::fetch`](crate::core::klippy::identify::Identify::fetch)).
    ///
    /// Only the send task writes the wire, so this is a message to it
    /// ([`SendItem::Renumber`]); it resolves once the renumber has been applied.
    ///
    /// # Errors
    /// Returns [`McuError::Call`] if the send task is gone.
    pub(crate) async fn renumber_to_firmware(&self) -> Result<(), McuError> {
        let (applied_tx, applied_rx) = oneshot::channel();
        self.send_buf_tx
            .send(SendItem::Renumber(applied_tx))
            .await
            .map_err(|e| McuError::Call(McuCallError::SendFailed(e.to_string())))?;
        applied_rx.await.map_err(|_| {
            McuError::Call(McuCallError::SendFailed(
                "the send task dropped the renumber".to_string(),
            ))
        })?;
        Ok(())
    }

    /// A command on its way out, as its DEBUG line shows it: the dictionary's
    /// own parameter names when it has the message, the name alone otherwise.
    ///
    /// Called as a field of the log line, so neither the lookup nor the
    /// formatting happens unless DEBUG is on.
    fn describe_command(&self, name: &str, args: &[ArgValue]) -> String {
        match self.parser.lookup(name) {
            Some(msg) => describe_message(&msg, args),
            None => name.to_string(),
        }
    }

    /// Send a command and wait for the response message.
    ///
    /// This is a synchronous request/response pattern: the command is sent,
    /// then the caller blocks (async) until the response message arrives or
    /// `timeout` elapses.
    ///
    /// The response does not have to be a direct answer to `command`: as long as
    /// the name matches, this also waits for a message the firmware pushes in
    /// reaction. That is how the configuration phase waits for `shutdown` after
    /// sending `emergency_stop` (`mcu/config.rs`): registration happens before
    /// the command goes out, so a fast report cannot race ahead of it.
    ///
    /// # Requirements
    /// - `command` must be registered in the message parser.
    /// - `command` must **not** have a callback bound (callback-registered
    ///   messages are for asynchronous notification, not request/response).
    /// - The response message identified by `response_name` must also be
    ///   registered (with or without a callback).
    ///
    /// # Errors
    /// Returns [`McuCallError`] if the command is unknown, already has a
    /// callback, the send buffer is full, or the response does not arrive
    /// within `timeout`.
    pub async fn call(
        &self,
        command: &str,
        args: &[ArgValue],
        response_name: &str,
        timeout: Duration,
    ) -> Result<Vec<ArgValue>, McuCallError> {
        self.call_gated(command, args, response_name, timeout, SendClocks::default())
            .await
    }

    /// [`Mcu::call`] with scheduling gates on the request ([`SendClocks`]).
    ///
    /// Used by the typed clocked call (`Mcu::call_msg_clocked`, `cmd`) — the
    /// homing query paths are the callers that gate (`endstop_query_state`'s
    /// `minclock`, upstream `klippy/mcu.py:401-405`).
    pub(crate) async fn call_gated(
        &self,
        command: &str,
        args: &[ArgValue],
        response_name: &str,
        timeout: Duration,
        clocks: SendClocks,
    ) -> Result<Vec<ArgValue>, McuCallError> {
        // 1. Verify command is registered; warn if it has a callback.
        if !self.parser.is_registered(command) {
            return Err(McuCallError::CommandNotFound(command.to_string()));
        }
        if self.events.has_callback(&self.parser, command) {
            warn!(
                "Command '{}' already has a callback, call may not work as expected",
                command
            );
        }

        // 2. Create a oneshot channel for the response.
        let (tx, rx) = oneshot::channel::<Vec<ArgValue>>();

        // 3. Register the pending call.
        self.pending_calls
            .register(response_name.to_string(), tx)
            .await;

        // 4. Send the command. Its line names the response it is waiting for.
        if let Err(e) = self.enqueue(command, args, Some(response_name), clocks) {
            warn!("Failed to send command '{}': {e}", command);
            // Clean up the pending call on send failure.
            self.pending_calls.cancel(response_name).await;
            return Err(McuCallError::SendFailed(e.msg));
        }

        // 5. Wait for the response. A successful resolve already consumed the
        // registration, so only the failure paths need to clean up.
        match tokio::time::timeout(timeout, rx).await {
            // The response itself was logged where it was decoded, with its
            // parameters; there is nothing left to say here.
            Ok(Ok(params)) => Ok(params),
            Ok(Err(_recv)) => {
                // Receiver dropped (shouldn't happen in normal flow).
                self.pending_calls.cancel(response_name).await;
                error!(
                    "[{}] Response receiver dropped for '{}'",
                    self.name, response_name
                );
                Err(McuCallError::SendFailed(
                    "response receiver dropped".to_string(),
                ))
            }
            Err(_) => {
                // Timeout — clean up the pending call.
                self.pending_calls.cancel(response_name).await;
                warn!(
                    "[{}] Timeout waiting for response '{}'",
                    self.name, response_name
                );
                Err(McuCallError::Timeout(format!(
                    "no response for {} within {:?}",
                    response_name, timeout
                )))
            }
        }
    }

    /// Bind a callback to a registered message, replacing any existing one.
    ///
    /// This is the callback counterpart of [`Mcu::send`] / [`Mcu::call`]: the
    /// typed entry point is `Mcu::bind_event` in
    /// [`event`](crate::core::klippy::event), which decodes the values by name
    /// before handing them to the handler. The callback receives the decoded
    /// parameter values in message declaration order and runs on the receive
    /// task.
    ///
    /// # Errors
    /// Returns [`McuError::Msg`] if no message called `name` is registered.
    /// Drop every inbound callback (see [`McuEvents::clear`]).
    ///
    /// The machine's parts call this as they are torn down, so the callbacks
    /// stop keeping their resources — and, through the resources' transport
    /// handle, the `Mcu` itself — alive after the machine is gone.
    pub(crate) fn clear_events(&self) {
        self.events.clear();
    }

    pub(crate) fn bind_callback(
        &self,
        name: &str,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> Result<(), McuError> {
        // The table is separate from the parser, so binding needs no `&mut` on
        // the shared registry; the receive task sees it immediately.
        self.events
            .bind(&self.parser, name, callback)
            .map_err(McuError::Msg)
    }
}

impl Mcu {
    /// Shut this session down **explicitly**, without waiting for the last
    /// [`Arc`] to go.
    ///
    /// Upstream ends a restart's old connection the same way:
    /// `MCURestartHelper._restart_via_command` sends `reset`, pauses 15 ms,
    /// then calls `self._disconnect()` (`klippy/mcu.py:730-747`) — the
    /// disconnect is a step of its own, not a consequence of reference
    /// counting. This host needs the same because handles outlive the session
    /// that used them: the chip's device slot and the clock estimate's
    /// `McuClock` still hold their `Arc<Mcu>`, so without this `Drop` never
    /// runs and both transport tasks keep working a port that has been
    /// reopened behind them — an old write end failing with EIO against a
    /// re-enumerated USB CDC, an old read end still stealing a UART's frames.
    ///
    /// The steps are `Drop`'s, taken here while the handles still exist:
    /// release the blocking device read first, then abort the tasks.
    /// Everything is idempotent — a session may be closed and then dropped.
    pub(crate) fn close(&self) {
        self.closed.store(true, Ordering::SeqCst);
        // The receive task's device read runs inside `spawn_blocking` and
        // cannot be cancelled by aborting the task; shutting the device down
        // releases that thread, and the aborts below then tear both tasks down
        // promptly (the send task would otherwise only notice when the queue
        // or the ack stream closes).
        self.interface.shutdown();
        for handles in [&self.send_handle, &self.recv_handle] {
            if let Some(handle) = handles
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .as_ref()
            {
                handle.abort();
            }
        }
    }

    /// Whether [`Mcu::close`] (or `Drop`, which calls it) has ended this
    /// session.
    pub(crate) fn is_closed(&self) -> bool {
        self.closed.load(Ordering::SeqCst)
    }
}

impl Drop for Mcu {
    /// Shut the session down when the last handle goes — [`Mcu::close`],
    /// which this calls as the backstop for every path that ends a session by
    /// letting it go rather than by closing it (a failed `Mcu::connect`, the
    /// machine's teardown, an ordinary restart).
    ///
    /// The blocking-read rationale lives there: the device read runs inside
    /// `spawn_blocking` and cannot be cancelled by aborting the async task, so
    /// the interface is shut down before the tasks are aborted — otherwise the
    /// blocked thread stays parked forever and the runtime hangs during
    /// shutdown.
    fn drop(&mut self) {
        if trace_enabled() {
            eprintln!("MCU DROP {}", self.name);
        }
        self.close();
    }
}

#[cfg(test)]
impl Mcu {
    /// Build a transport over a bare interface, without a config.
    ///
    /// Tests talk to a [`FrameMock`](crate::core::klippy::interface::devices::frame_mock::FrameMock)
    /// rather than a real `McuConfig`, and most of them never identify.
    pub(crate) fn for_test(name: impl Into<String>, interface: Interface) -> Self {
        Self::from_parts(name.into(), interface)
    }

    /// The dictionary-driven fake MCU behind this connection, when the section
    /// asked for one (`test: dict=`); `None` on every other transport.
    ///
    /// How a multi-instance test reaches the exact responder that answers its
    /// `[mcu …]` section (see
    /// [`crate::core::klippy::interface::devices::responder_mcu`]).
    pub(crate) fn simulator_device(
        &self,
    ) -> Option<Arc<crate::core::klippy::interface::SimulatorDevice>> {
        self.interface.simulator_device()
    }

    /// Whether both transport tasks have finished — the test seam behind
    /// [`Mcu::close`]'s promise that a session's tasks stop without the last
    /// `Arc` going away.
    pub(crate) fn transport_tasks_finished(&self) -> bool {
        [&self.send_handle, &self.recv_handle]
            .iter()
            .all(|handles| {
                handles
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .as_ref()
                    .map(|handle| handle.is_finished())
                    .unwrap_or(true)
            })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::clock::{ClockSync, McuClock};
    use crate::core::klippy::cmd::config::Reset;
    use crate::core::klippy::cmd::identify::IDENTIFY_CHUNK_SIZE;
    use crate::core::klippy::cmd::McuCommand;
    use crate::core::klippy::identify::Identify;
    use crate::core::klippy::interface::devices::frame_mock::{
        FrameMock, FrameRecorder, MappingEntry, RecordingWire,
    };
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::reactor::ManualReactor;
    use flate2::write::ZlibEncoder;
    use flate2::Compression;
    use std::io::Write;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    /// A block the way [`Sender::in_flight`] keeps it: written out, unanswered.
    fn in_flight_block(seq: u64, frame: Frame) -> InFlightBlock {
        InFlightBlock {
            seq,
            frame,
            sent_at: Instant::now(),
        }
    }

    // -----------------------------------------------------------------------
    // Mcu creation
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_new_starts_unidentified() {
        let mcu = Mcu::new("test_mcu", Interface::new(FrameMock::new(vec![])));

        assert_eq!(mcu.name(), "test_mcu");

        // Construction is transport only: the parser knows the host's identify
        // pair and nothing else. Everything else arrives with `Mcu::connect`.
        assert!(!mcu.is_identified());
        assert!(mcu.parser.is_registered("identify"));
        assert!(mcu.parser.is_registered("identify_response"));
    }

    /// `try_lookup_command` matches the firmware's declaration exactly, which is
    /// how the I2C layer tells legacy (`i2c_transfer`) from new
    /// (`i2c_write`/`i2c_read`) transfer styles.
    #[tokio::test]
    async fn test_try_lookup_command_requires_the_exact_format() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(vec![])));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {
                "i2c_transfer oid=%c write=%*s read_len=%u": 43,
                "i2c_write oid=%c data=%*s": 44
            }
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        // The firmware's own wording matches.
        assert!(mcu
            .try_lookup_command("i2c_transfer oid=%c write=%*s read_len=%u")
            .is_some());
        // A different parameter name, specifier, or unknown message does not.
        // The parser would still find `i2c_transfer` by its bare name, so these
        // are exactly the cases a name-only lookup gets wrong.
        assert!(mcu
            .try_lookup_command("i2c_transfer oid=%c data=%*s read_len=%u")
            .is_none());
        assert!(mcu
            .try_lookup_command("i2c_transfer oid=%c write=%s read_len=%u")
            .is_none());
        assert!(mcu.try_lookup_command("i2c_read oid=%c reg=%*s").is_none());
    }

    /// The shapes a real MCU produces for one request: the response, and the ack
    /// that carries no payload at all.
    #[tokio::test]
    async fn test_acks_and_repeated_sequences_are_accepted() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();

        let request = make_frame(0, &[5]);
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(0x1234).unwrap();

        // The firmware stamps everything it sends while handling a block with that
        // block's sequence, so the ack repeats the response's sequence. A transport
        // that expected a new number per frame would drop the ack, advance its
        // counter, and then reject the next exchange's answer.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: request.clone(),
                outputs: vec![
                    make_frame(0, &answer.clone().into_raw()),
                    make_frame(0, &[]), // ack: no payload
                ],
            },
            MappingEntry {
                input: make_frame(1, &[5]),
                outputs: vec![make_frame(1, &answer.into_raw())],
            },
        ]);
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        // Both exchanges have to complete: the first proves the ack was ignored
        // rather than counted, the second proves it did not desynchronise the
        // stream.
        for _ in 0..2 {
            let params = mcu
                .call("get_clock", &[], "clock", Duration::from_millis(500))
                .await
                .expect("both exchanges must complete");
            assert_eq!(params, vec![ArgValue::UInt32(0x1234)]);
        }
    }

    #[tokio::test]
    async fn test_a_running_firmware_is_taken_over() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // A board that never rebooted is still at some number from the session
        // before this one, and naks the block it cannot place — with a frame
        // carrying that number (`src/command.c:331`, an empty ack/nak frame). The
        // transport has to adopt it, put the request back on the wire under it, and
        // carry on: the second mapping is the firmware accepting *that* block.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![make_frame(9, &[])],
            },
            MappingEntry {
                input: make_frame(9, &[5]),
                outputs: vec![make_frame(9, &answer.clone().into_raw())],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        let params = mcu
            .call("get_clock", &[], "clock", Duration::from_millis(200))
            .await
            .expect("the request is answered once the session is taken over");
        assert_eq!(params.len(), 1);
        assert!(
            mcu.took_over_session(),
            "the firmware was mid-session: the caller has to be able to tell"
        );

        // The request went out twice: once at the number this connection started
        // with, and once at the firmware's own.
        let sent = recorder.frames();
        let seqs: Vec<u8> = sent.iter().map(|frame| frame.seq()).collect();
        assert_eq!(seqs, [0, 9], "the block is renumbered, not just repeated");
    }

    #[tokio::test]
    async fn test_a_fresh_firmware_needs_no_taking_over() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // What a board that just booted does: it answers block 0 with 1 (the number
        // its counter moved to), and then acks it with the same number. Nothing is
        // sent a second time, and no takeover is reported.
        let device = FrameMock::new(vec![MappingEntry {
            input: make_frame(0, &[5]),
            outputs: vec![
                make_frame(1, &answer.clone().into_raw()),
                make_frame(1, &[]),
            ],
        }]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        mcu.call("get_clock", &[], "clock", Duration::from_millis(200))
            .await
            .expect("the request is answered");

        assert!(!mcu.took_over_session());
        let sent = recorder.frames();
        assert_eq!(sent.len(), 1, "an ack is not a nak: nothing goes out again");
    }

    #[tokio::test]
    async fn test_a_frame_answering_a_block_we_never_sent_is_dropped() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // The answer, and then a frame numbered past anything this connection sent.
        // Only the *first* frame of a connection may be that (it is a session to
        // take over); later ones are dropped, and an exchange that follows must not
        // be disturbed by them.
        let device = FrameMock::new(vec![MappingEntry {
            input: make_frame(0, &[5]),
            outputs: vec![
                make_frame(1, &answer.clone().into_raw()),
                make_frame(5, &[]),
            ],
        }]);
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        mcu.call("get_clock", &[], "clock", Duration::from_millis(200))
            .await
            .expect("the request is answered");

        assert!(
            !mcu.took_over_session(),
            "a stray frame after the first one is not a session to take over"
        );
    }

    /// The session's first **new** sequence number is adopted even when an
    /// earlier frame repeated the number this connection starts at.
    ///
    /// A board that never rebooted may repeat this connection's own initial
    /// number first — a leftover frame from the session before — and only then
    /// show the number it is really waiting for. Only the first new number
    /// belongs to the takeover (`receive_seq == 1` upstream,
    /// `serialqueue.c:261`): the frame carrying it must not be dropped for
    /// answering a block this connection never sent, or the firmware replays it
    /// forever and whatever is queued behind it never goes out.
    #[tokio::test]
    async fn test_the_first_new_sequence_is_adopted_after_a_repeated_frame() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // Two frames answer the first block: one that repeats the number this
        // connection starts at, then the first new number — high, and for a
        // block this connection never sent. The second mapping is the firmware
        // accepting the request once its number is adopted.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![make_frame(0, &[]), make_frame(9, &[])],
            },
            MappingEntry {
                input: make_frame(9, &[5]),
                outputs: vec![make_frame(9, &answer.into_raw())],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        let params = mcu
            .call("get_clock", &[], "clock", Duration::from_millis(200))
            .await
            .expect("the first new sequence is adopted, so the request is answered");
        assert_eq!(params.len(), 1);
        assert!(
            mcu.took_over_session(),
            "a firmware waiting past this connection's number never rebooted"
        );

        // The block went out once at this connection's number and once at the
        // adopted one: renumbered, not merely repeated.
        let sent = recorder.frames();
        let seqs: Vec<u8> = sent.iter().map(|frame| frame.seq()).collect();
        assert_eq!(
            seqs,
            [0, 9],
            "the block is renumbered onto the adopted sequence"
        );
    }

    /// Once the session has taken in a new number, a frame answering past what
    /// this connection sent is still dropped, not adopted.
    ///
    /// Mid-session such a frame cannot be where the firmware is — the firmware
    /// moves in lockstep with this connection's window — so adopting it would
    /// rename the session around a stray (`serialqueue.c:261-265` drops it
    /// there too).
    #[tokio::test]
    async fn test_a_mid_session_frame_past_our_next_is_dropped_not_adopted() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // First exchange: the normal answer (the session's first new number is
        // 1, nothing to take over). Second exchange: a stray numbered past
        // anything this connection sent arrives *before* the real answer.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![
                    make_frame(1, &answer.clone().into_raw()),
                    make_frame(1, &[]),
                ],
            },
            MappingEntry {
                input: make_frame(1, &[5]),
                outputs: vec![make_frame(9, &[]), make_frame(2, &answer.into_raw())],
            },
        ]);
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        for round in 0..2 {
            mcu.call("get_clock", &[], "clock", Duration::from_millis(200))
                .await
                .unwrap_or_else(|e| panic!("exchange {round} must complete: {e}"));
        }

        assert!(
            !mcu.took_over_session(),
            "a mid-session stray is not a session to take over"
        );
        assert_eq!(
            mcu.wire.next.load(Ordering::Relaxed),
            2,
            "the stray never renames the session"
        );
    }

    /// A firmware with no leftover frame: the first frame is the normal answer
    /// to the first block, and two round trips are exactly one block each, at
    /// their own numbers — no takeover, no renumbering, no second send.
    #[tokio::test]
    async fn test_a_normal_first_frame_exchange_is_unchanged() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![
                    make_frame(1, &answer.clone().into_raw()),
                    make_frame(1, &[]),
                ],
            },
            MappingEntry {
                input: make_frame(1, &[5]),
                outputs: vec![
                    make_frame(2, &answer.clone().into_raw()),
                    make_frame(2, &[]),
                ],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        for round in 0..2 {
            mcu.call("get_clock", &[], "clock", Duration::from_millis(200))
                .await
                .unwrap_or_else(|e| panic!("exchange {round} must complete: {e}"));
        }

        assert!(
            !mcu.took_over_session(),
            "a freshly booted firmware has nothing to take over"
        );
        assert_eq!(
            mcu.wire.next.load(Ordering::Relaxed),
            2,
            "both blocks accepted, none renumbered"
        );
        let sent = recorder.frames();
        let seqs: Vec<u8> = sent.iter().map(|frame| frame.seq()).collect();
        assert_eq!(seqs, [0, 1], "one block per exchange, at its own number");
    }

    // -----------------------------------------------------------------------
    // Decode before the dictionary is installed
    // -----------------------------------------------------------------------

    /// Payload of the periodic `stats` a running firmware keeps sending while
    /// the handshake is still transferring the dictionary: message id -12 as a
    /// signed VLQ, then 8 parameter bytes. Before the dictionary arrives the
    /// parser knows only the identify pair, so this id cannot decode.
    fn stats_payload() -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(-12).unwrap();
        payload.extend(&[0; 8]).unwrap();
        payload.into_raw()
    }

    /// Payload of an `identify offset=%u count=%c` request — the bytes the
    /// handshake puts on the wire for `offset` (the same bytes the fixture in
    /// `identify.rs` builds for its own exchanges).
    fn identify_request_payload(offset: u32) -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(1).unwrap();
        payload.push_u32(offset).unwrap();
        payload.push_u8(IDENTIFY_CHUNK_SIZE).unwrap();
        payload.into_raw()
    }

    /// Payload of an `identify_response offset=%u data=%.*s` answer.
    fn identify_response_payload(offset: u32, data: &[u8]) -> Vec<u8> {
        let mut payload = Payload::new();
        payload.push_i16(0).unwrap();
        payload.push_u32(offset).unwrap();
        payload.push_bytes(data).unwrap();
        payload.into_raw()
    }

    /// The body handed over by the exchange below, compressed the way the
    /// firmware stores it (`zlib.compress(...)` output).
    fn compress(body: &[u8]) -> Vec<u8> {
        let mut encoder = ZlibEncoder::new(Vec::new(), Compression::default());
        encoder.write_all(body).unwrap();
        encoder.finish().unwrap()
    }

    /// What the receive task owes a frame it cannot decode while identify is
    /// still running: nothing. A firmware that was already alive keeps sending
    /// `stats` between the request and its answer, and before the dictionary is
    /// installed its id is unknown to the host-owned parser. The frame has to
    /// be dropped without failing the call chain and without touching the
    /// pending call, so the `identify_response` right behind it still resolves
    /// and the handshake completes — with no retransmit or renumber on account
    /// of the noise.
    #[tokio::test]
    async fn test_an_unknown_id_frame_before_the_dictionary_is_skipped() {
        let compressed = compress(br#"{"app": "Klipper"}"#);
        let device = FrameMock::new(vec![
            MappingEntry {
                input: Frame::new(0, identify_request_payload(0)),
                outputs: vec![
                    // The noise first, stamped with the sequence the firmware
                    // took the block under — the same number the answer and
                    // the ack behind it carry.
                    Frame::new(1, stats_payload()),
                    Frame::new(1, identify_response_payload(0, &compressed)),
                    Frame::new(1, Vec::new()),
                ],
            },
            MappingEntry {
                // The terminating request: the payload so far, all of it.
                input: Frame::new(1, identify_request_payload(compressed.len() as u32)),
                outputs: vec![
                    Frame::new(2, identify_response_payload(compressed.len() as u32, &[])),
                    Frame::new(2, Vec::new()),
                ],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        let identify = Identify::fetch(&mcu, Duration::from_secs(1))
            .await
            .expect("the unknown-id frame is skipped and the identify response still resolves");

        assert_eq!(identify.data["app"], "Klipper");
        assert!(
            !mcu.took_over_session(),
            "the noise frame names no session to take over"
        );
        // One request per chunk and nothing else: the frame the parser refused
        // cost the exchange neither a retransmit nor a renumber, and the
        // pending call was consumed by the response, not by the noise.
        let sent = recorder.frames();
        let seqs: Vec<u8> = sent.iter().map(|frame| frame.seq()).collect();
        assert_eq!(
            seqs,
            [0, 1],
            "one request per chunk at its own number: the skipped frame adds no send"
        );
    }

    /// The renumber adopts the sequence the firmware reported and abandons what
    /// is in flight: those blocks are earlier attempts at the chunk being
    /// retried, which goes out again under the adopted number (see
    /// [`Mcu::renumber_to_firmware`]). The state that belongs to the old
    /// numbering — the ack bookkeeping and the retransmit wait — starts over.
    #[test]
    fn test_renumber_adopts_the_firmware_sequence_and_clears_the_window() {
        let wire = Arc::new(Wire::default());
        wire.next.store(2, Ordering::Relaxed);
        wire.seen.store(1, Ordering::Relaxed);
        let mut sender = Sender::new(Arc::clone(&wire));
        sender
            .in_flight
            .push_back(in_flight_block(0, make_frame(0, &[5])));
        sender.acked = Some(2);
        sender.retransmitted = Some(2);
        sender.rto = Duration::from_secs(1);
        sender.sample_seq = Some(0);
        sender.arm_retransmit();

        sender.renumber_to_firmware();

        assert_eq!(
            wire.next.load(Ordering::Relaxed),
            1,
            "the firmware's number is the one the next block carries"
        );
        assert!(
            sender.in_flight.is_empty(),
            "what waited under the old numbering is abandoned; the retry re-issues its chunk"
        );
        assert!(
            sender.retransmit_at.is_none(),
            "nothing waits to be resent under the old numbering"
        );
        assert_eq!(sender.rto, MIN_RTO, "the wait starts over");
        assert!(sender.acked.is_none());
        assert!(sender.retransmitted.is_none());
        assert!(
            sender.sample_seq.is_none(),
            "the pinned sample belonged to the abandoned window"
        );
    }

    // -----------------------------------------------------------------------
    // The receive window: a dropped frame still reports, and a renumber
    // re-arms connection init (B5 — reset → reconnect against an arbitrary
    // residue, firmware rebooted or not).
    //
    // Both rules are what keeps a rewound window from going deaf: the receive
    // side decides on its own whether a frame is placed or dropped
    // (`place_frame`), and the two halves of that decision are what the sender
    // then acts on.
    // -----------------------------------------------------------------------

    /// A frame past the window is dropped — but it is still the firmware saying
    /// where it is, and `Wire::seen` is exactly what `renumber_to_firmware`
    /// adopts. A drop that discarded the number would leave every later renumber
    /// aiming at a window the firmware has already left, and the receive side
    /// would then drop every answer against a `next` that never moves.
    ///
    /// The real shape: `/tmp/vA.log` dropped 396 frames, all against the same
    /// `answers block 110` — the recorded number never moved with what the
    /// firmware reported.
    #[test]
    fn test_a_frame_past_the_window_still_reports_where_the_firmware_is() {
        // Both sides at 110, then the firmware's counter moves on without this
        // connection having sent the blocks: frame sequence 0 is two past 110 in
        // this window's numbering.
        let wire = Wire::default();
        wire.next.store(110, Ordering::Relaxed);
        wire.seen.store(110, Ordering::Relaxed);

        assert_eq!(
            place_frame(110, &wire, 0),
            Placement::Ahead {
                rseq: 112,
                next: 110
            },
            "the frame is dropped: it answers a block this connection never sent"
        );
        assert_eq!(
            wire.seen.load(Ordering::Relaxed),
            112,
            "the drop keeps the firmware's report — that is what the next renumber adopts"
        );
    }

    /// A renumber rewinds the window, so the firmware's answer arriving next can
    /// be ahead of it. That frame is this connection initialising itself
    /// (`serialqueue.c:196-201`), not an answer to a block never sent: without
    /// the re-armed exemption the receive side drops it and the retry is never
    /// answered.
    #[test]
    fn test_a_renumber_rearms_connection_init_for_the_answer_behind_it() {
        let wire = Arc::new(Wire::default());
        wire.next.store(6, Ordering::Relaxed);
        wire.seen.store(5, Ordering::Relaxed);
        let mut sender = Sender::new(Arc::clone(&wire));

        sender.renumber_to_firmware();

        assert_eq!(
            wire.next.load(Ordering::Relaxed),
            5,
            "the window is rewound onto the firmware's number"
        );
        // The firmware took the block this connection sent before the silence,
        // so it is past the rewound window when its answer arrives.
        assert_eq!(
            place_frame(5, &wire, 6),
            Placement::Adopt {
                rseq: 6,
                session_start: false,
                ahead_of_window: true,
            },
            "the answer behind a renumber is placed, and the send task is told"
        );
        // One exemption, one frame: spent by the number it let through, so a
        // later frame past the window is dropped as it was before. The send task
        // has adopted 6 in between (`Sender::settle`), which is what the window
        // reads here.
        wire.next.store(6, Ordering::Relaxed);
        assert_eq!(
            place_frame(6, &wire, 8),
            Placement::Ahead { rseq: 8, next: 6 },
            "the exemption does not outlive the renumber it belongs to"
        );
    }

    /// A congruent frame is no decision at all: no number moves and no
    /// exemption is spent, so a firmware whose counter sits at a multiple of 16
    /// opens the session exactly like one at 0 — same window, no takeover.
    #[test]
    fn test_a_congruent_frame_is_no_decision_at_all() {
        let wire = Wire::default(); // next 0, seen 0: a session that just opened

        assert_eq!(
            place_frame(0, &wire, 0),
            Placement::Repeat,
            "a frame congruent with `seen` carries no new number"
        );
        assert_eq!(wire.seen.load(Ordering::Relaxed), 0, "and moves nothing");
        assert_eq!(
            place_frame(0, &wire, 5),
            Placement::Adopt {
                rseq: 5,
                session_start: true,
                ahead_of_window: true,
            },
            "the first-frame exemption is still armed for the next real number"
        );
    }

    // -----------------------------------------------------------------------
    // RTT estimation and the over-threshold warning
    //
    // `serialqueue.c:218-237`: one sample per acknowledged block, an
    // RFC6298-style smoothing that starts conservatively, and a timeout
    // derived from both and clamped to [MIN_RTO, MAX_RTO] — which is also the
    // wait the retransmit timer arms itself with (see [`Sender::rto`]).
    // -----------------------------------------------------------------------

    /// The first sample starts the way upstream starts it
    /// (`serialqueue.c:222-224`, "use a higher start default"): `srtt` at ten
    /// times the sample, `rttvar` at half of it, and a timeout that is the sum
    /// of the smoothed value and four deviations.
    #[test]
    fn test_rtt_first_sample_starts_conservatively() {
        let (srtt, rttvar) = rtt_step(None, Duration::ZERO, Duration::from_millis(4));

        assert_eq!(srtt, Duration::from_millis(40), "srtt = δ × 10");
        assert_eq!(rttvar, Duration::from_millis(2), "rttvar = δ / 2");
        assert_eq!(
            rtt_rto(srtt, rttvar),
            Duration::from_millis(48),
            "rto = 40 ms + 4 × 2 ms"
        );
    }

    /// Every later sample smooths (`serialqueue.c:226-227`): the deviation
    /// keeps three quarters of itself plus a quarter of the new error, the
    /// smoothed value seven eighths of itself plus one eighth of the sample —
    /// from either side of it.
    #[test]
    fn test_rtt_later_samples_smooth_from_both_sides() {
        // δ below srtt: rttvar = (3 × 5 ms + 80 ms) / 4, srtt = (7 × 100 ms + 20 ms) / 8.
        let (srtt, rttvar) = rtt_step(
            Some(Duration::from_millis(100)),
            Duration::from_millis(5),
            Duration::from_millis(20),
        );
        assert_eq!(srtt, Duration::from_millis(90));
        assert_eq!(rttvar, Duration::from_micros(23_750), "23.75 ms");
        assert_eq!(
            rtt_rto(srtt, rttvar),
            Duration::from_millis(185),
            "90 ms + 4 × 23.75 ms"
        );

        // δ above srtt: the error is |10 ms − 30 ms|, and srtt rises.
        let (srtt, rttvar) = rtt_step(
            Some(Duration::from_millis(10)),
            Duration::from_millis(4),
            Duration::from_millis(30),
        );
        assert_eq!(srtt, Duration::from_micros(12_500), "12.5 ms");
        assert_eq!(rttvar, Duration::from_millis(8), "(3 × 4 ms + 20 ms) / 4");
    }

    /// Four deviations may never add less than a millisecond
    /// (`serialqueue.c:229-231`), and above that they are counted as they are.
    #[test]
    fn test_rtt_rto_keeps_the_1ms_variance_floor() {
        assert_eq!(
            rtt_rto(Duration::from_millis(30), Duration::from_micros(100)),
            Duration::from_millis(31),
            "4 × 100 µs is under the floor, so the floor applies"
        );
        assert_eq!(
            rtt_rto(Duration::from_millis(30), Duration::from_millis(2)),
            Duration::from_millis(38),
            "4 × 2 ms is over the floor, so it counts"
        );
    }

    /// The estimate's timeout stays inside the two constants — which are the
    /// same 25 ms / 5 s upstream clamps to (`serialqueue.c:107-108`), and are
    /// not moved by any of this.
    #[test]
    fn test_rtt_rto_is_clamped_to_the_bounds() {
        assert_eq!(MIN_RTO, Duration::from_millis(25));
        assert_eq!(MAX_RTO, Duration::from_secs(5));
        assert_eq!(
            rtt_rto(Duration::from_millis(10), Duration::ZERO),
            MIN_RTO,
            "an estimate below the floor is raised to it"
        );
        assert_eq!(
            rtt_rto(Duration::from_secs(10), Duration::from_secs(5)),
            MAX_RTO,
            "an estimate above the ceiling is lowered to it"
        );
    }

    /// Recording keeps the sample as well as the smoothed value, and derives
    /// the clamped timeout from each pair — what [`Sender::rtt`],
    /// [`Sender::srtt`] and [`Sender::estimated_rto`] hand out.
    #[test]
    fn test_rtt_estimator_records_samples_and_derives_the_timeout() {
        let mut estimator = RttEstimator::new();
        assert_eq!(estimator.last_sample, None, "no sample to begin with");
        assert_eq!(estimator.srtt, None, "no estimate to begin with");
        assert_eq!(estimator.rto, MIN_RTO, "the wait starts at the floor");

        estimator.record(Duration::from_millis(5));
        assert_eq!(estimator.last_sample, Some(Duration::from_millis(5)));
        assert_eq!(estimator.srtt, Some(Duration::from_millis(50)));
        assert_eq!(estimator.rttvar, Duration::from_micros(2_500));

        estimator.record(Duration::from_millis(7));
        // srtt = (7 × 50 ms + 7 ms) / 8 = 44.625 ms,
        // rttvar = (3 × 2.5 ms + |50 ms − 7 ms|) / 4 = 12.625 ms,
        // rto = 44.625 ms + 50.5 ms = 95.125 ms.
        assert_eq!(estimator.last_sample, Some(Duration::from_millis(7)));
        assert_eq!(estimator.srtt, Some(Duration::from_micros(44_625)));
        assert_eq!(estimator.rttvar, Duration::from_micros(12_625));
        assert_eq!(estimator.rto, Duration::from_micros(95_125));
    }

    /// The first round trip over the threshold warns once, and only a doubling
    /// of the value that warned warns again.
    #[test]
    fn test_rtt_warn_fires_once_per_crossing_and_once_per_doubling() {
        let mut state = RttWarnState::default();
        assert!(
            !state.should_warn(Duration::from_millis(25)),
            "the threshold itself is not over it"
        );
        assert!(!state.should_warn(Duration::from_millis(24)), "under it");
        assert!(
            state.should_warn(Duration::from_millis(26)),
            "the first crossing warns"
        );
        assert!(
            !state.should_warn(Duration::from_millis(30)),
            "the same crossing does not warn again"
        );
        assert!(
            state.should_warn(Duration::from_millis(52)),
            "twice the value that warned"
        );
        assert!(
            !state.should_warn(Duration::from_millis(53)),
            "and then quiet again"
        );
    }

    /// Falling back under the threshold re-arms nothing: coming back up short
    /// of a doubling of the value that warned stays quiet.
    #[test]
    fn test_rtt_warn_stays_quiet_after_a_drop_below_a_doubling() {
        let mut state = RttWarnState::default();
        assert!(state.should_warn(Duration::from_millis(30)));
        assert!(!state.should_warn(Duration::from_millis(2)), "back under");
        assert!(
            !state.should_warn(Duration::from_millis(40)),
            "40 ms is not twice 30 ms"
        );
        assert!(!state.should_warn(Duration::from_millis(59)));
        assert!(
            state.should_warn(Duration::from_millis(60)),
            "60 ms is twice 30 ms"
        );
    }

    /// The warning carries what was measured and where to look first.
    #[test]
    fn test_rtt_warn_message_carries_the_value_and_the_hint() {
        let message = rtt_warn_message(Duration::from_millis(31));

        assert!(message.contains("RTT 31ms"), "{message}");
        assert!(
            message.contains(&format!("{RTT_WARN_THRESHOLD:?}")),
            "the threshold itself ({RTT_WARN_THRESHOLD:?}): {message}"
        );
        assert!(message.contains("latency_timer"), "{message}");
    }

    /// An ack that covers a block nobody had to resend closes a round trip:
    /// the sample lands in the estimate and the accessors hand it out.
    #[tokio::test]
    async fn test_an_ack_without_a_retransmit_yields_a_sample() {
        let device = FrameMock::new(vec![MappingEntry {
            input: make_frame(0, &[5]),
            outputs: vec![],
        }]);
        let interface = Interface::new(device);
        let mut sender = Sender::new(Arc::new(Wire::default()));

        sender.send_block(&interface, vec![5]).await;
        assert_eq!(
            sender.sample_seq,
            Some(0),
            "the block on the wire pins the next sample"
        );
        assert_eq!(sender.rtt(), None, "nothing measured yet");

        sender.settle(&interface, 1).await;

        assert!(
            sender.rtt().is_some(),
            "the ack closes the round trip the block started"
        );
        assert!(sender.srtt().is_some());
        assert!(
            sender.sample_seq.is_none(),
            "one sample per pinned block, as upstream consumes it"
        );
    }

    /// A block that had to be retransmitted measures the retransmit, not the
    /// line: the pin dies with it (`serialqueue.c:463`), so the ack that
    /// finally arrives yields no sample.
    #[tokio::test]
    async fn test_a_retransmit_invalidates_the_pinned_sample() {
        // Two mappings for the same frame: the original and the retransmit.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
        ]);
        let interface = Interface::new(device);
        let mut sender = Sender::new(Arc::new(Wire::default()));

        sender.send_block(&interface, vec![5]).await;
        sender.retransmit(&interface, "test_mcu").await;
        assert!(
            sender.sample_seq.is_none(),
            "a retransmit throws the pinned sample away"
        );

        sender.settle(&interface, 1).await;

        assert!(
            sender.rtt().is_none(),
            "no sample from a block that went out twice"
        );
        assert!(sender.srtt().is_none());
    }

    // -----------------------------------------------------------------------
    // the retransmit wait the estimate drives (`Sender::rto`)
    //
    // The split named in [`MIN_RTO`]: the estimate is the normal wait, a
    // timeout's doubling off the floor is the lost-packet fallback, and the
    // next sample pulls the wait back to the estimate.
    // -----------------------------------------------------------------------

    /// With no sample the wait is the floor it has always been, and a timeout
    /// doubles it — the fallback path a line nothing has measured yet runs on.
    #[tokio::test]
    async fn test_without_a_sample_the_wait_is_the_floor_and_doubles() {
        // Original send and two retransmits of the same block.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
        ]);
        let interface = Interface::new(device);
        let mut sender = Sender::new(Arc::new(Wire::default()));

        sender.send_block(&interface, vec![5]).await;
        assert_eq!(
            sender.rto, MIN_RTO,
            "no sample: the wait starts at the floor"
        );
        assert_eq!(
            sender.estimated_rto(),
            MIN_RTO,
            "and the estimate says the same"
        );

        sender.retransmit(&interface, "test_mcu").await;
        assert_eq!(sender.rto, MIN_RTO * 2, "a timeout still doubles the wait");
        sender.retransmit(&interface, "test_mcu").await;
        assert_eq!(sender.rto, MIN_RTO * 4, "and doubles again");
    }

    /// The first sample moves the wait off the floor to the estimate, to the
    /// microsecond — the values the estimator itself derives for the same
    /// sequence, not a rounded stand-in.
    #[test]
    fn test_a_sample_moves_the_wait_to_the_estimate() {
        let mut sender = Sender::new(Arc::new(Wire::default()));
        assert_eq!(
            sender.rto, MIN_RTO,
            "before any sample the wait is the floor"
        );

        sender.record_sample(Duration::from_millis(5));
        assert_eq!(
            sender.estimated_rto(),
            Duration::from_millis(60),
            "srtt 50 ms + 4 × 2.5 ms"
        );
        assert_eq!(
            sender.rto,
            Duration::from_millis(60),
            "the wait is the estimate from the first sample on"
        );

        sender.record_sample(Duration::from_millis(7));
        assert_eq!(
            sender.rto,
            Duration::from_micros(95_125),
            "44.625 ms + 50.5 ms — the same value the estimator derives"
        );
        assert_eq!(
            sender.rto,
            sender.estimated_rto(),
            "the wait and the estimate stay together"
        );
    }

    /// A success starts the wait over **at the estimate**, not at the floor:
    /// what the line measured before still says how long the next unanswered
    /// block may take.
    #[tokio::test]
    async fn test_a_success_returns_the_wait_to_the_estimate_not_the_floor() {
        // The original send and the retransmit of the same block: the ack that
        // finally lands closes the window without yielding a new sample (the
        // retransmit threw the pin away), so only the success line below can
        // choose the wait.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
        ]);
        let interface = Interface::new(device);
        let mut sender = Sender::new(Arc::new(Wire::default()));
        sender.record_sample(Duration::from_millis(5));
        assert_eq!(
            sender.rto,
            Duration::from_millis(60),
            "the estimate the success should return to"
        );

        sender.send_block(&interface, vec![5]).await;
        sender.retransmit(&interface, "test_mcu").await;
        assert_eq!(
            sender.rto,
            Duration::from_millis(120),
            "the timeout backed the wait off the estimate"
        );

        sender.settle(&interface, 1).await;

        assert_eq!(
            sender.rto,
            Duration::from_millis(60),
            "back to the estimate, not to the 25 ms floor"
        );
        assert_eq!(sender.rto, sender.estimated_rto());
        assert_ne!(sender.rto, MIN_RTO);
    }

    /// A timeout backs the wait off; the next sample the line actually yields
    /// pulls it straight back to the estimate.
    #[tokio::test]
    async fn test_a_new_sample_pulls_a_doubled_wait_back_to_the_estimate() {
        // seq 0: original and retransmit; seq 1: the block that yields the
        // sample pulling the wait back.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(1, &[5]),
                outputs: vec![],
            },
        ]);
        let interface = Interface::new(device);
        let mut sender = Sender::new(Arc::new(Wire::default()));
        sender.record_sample(Duration::from_millis(5));

        sender.send_block(&interface, vec![5]).await;
        sender.retransmit(&interface, "test_mcu").await;
        assert_eq!(
            sender.rto,
            Duration::from_millis(120),
            "the doubled wait the sample has to pull back"
        );

        sender.send_block(&interface, vec![5]).await;
        // The round trip of the block just sent, measured at 5 ms.
        let block = sender.in_flight.back_mut().expect("the block on the wire");
        block.sent_at = Instant::now()
            .checked_sub(Duration::from_millis(5))
            .expect("the process has been up for longer than the sample");

        sender.settle(&interface, 2).await;

        assert_eq!(
            sender.rto,
            sender.estimated_rto(),
            "the new sample pulls the wait back to the estimate"
        );
        assert!(
            sender.rto >= Duration::from_millis(60),
            "the estimate for a 5 ms line is above the floor, got {:?}",
            sender.rto
        );
        assert!(
            sender.rto < Duration::from_millis(120),
            "the backoff is gone again, got {:?}",
            sender.rto
        );
    }

    // -----------------------------------------------------------------------
    // the outbound queue's bounds
    //
    // `Mcu::send` cannot await room, so it polls the queue for `SYNC_SEND_WAIT`
    // and reports what it was sending if the queue is still full. These tests
    // pin both ends of that: the wait outlasts a drain, and it gives up on a
    // queue that never drains.
    // -----------------------------------------------------------------------

    /// A queue that drains within the wait takes the sync sender with it.
    #[test]
    fn test_a_sync_send_lands_once_the_queue_drains() {
        let (tx, mut rx) = mpsc::channel::<SendItem>(2);
        for _ in 0..2 {
            tx.try_send(SendItem::Payload(Payload::new()))
                .expect("the queue starts empty");
        }
        assert_eq!(tx.capacity(), 0, "the queue is full to begin with");

        // The sender runs on its own thread, so the test thread is free to
        // drain the queue under it. `rx` stays here: dropping it would close
        // the queue, which is a different outcome than the one under test.
        let sender_tx = tx.clone();
        let sender = std::thread::spawn(move || {
            try_send_bounded(&sender_tx, SendItem::Payload(Payload::new()))
        });

        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(tx.capacity(), 0, "the sender waits instead of refusing");
        let _ = rx
            .try_recv()
            .expect("the two payloads that filled the queue");

        let result = sender.join().expect("the sender thread");
        assert!(result.is_ok(), "the payload lands once there is room");
    }

    /// A queue that never drains is reported after the wait, not waited on
    /// forever.
    #[test]
    fn test_a_full_queue_that_never_drains_is_reported_after_the_wait() {
        let (tx, _rx) = mpsc::channel::<SendItem>(2);
        for _ in 0..2 {
            tx.try_send(SendItem::Payload(Payload::new()))
                .expect("the queue starts empty");
        }

        let started = Instant::now();
        let result = try_send_bounded(&tx, SendItem::Payload(Payload::new()));
        let waited = started.elapsed();

        assert!(
            matches!(result, Err(TrySendError::Full(_))),
            "a full queue with no drain is Full, got {result:?}"
        );
        assert!(waited >= SYNC_SEND_WAIT, "it looked for room: {waited:?}");
        assert!(
            waited < SYNC_SEND_WAIT + Duration::from_millis(500),
            "the wait is bounded: {waited:?}"
        );
    }

    /// The reserved slots the awaiting producer stops at are what the sync path
    /// spends first — the two halves share `payload_has_room`.
    #[test]
    fn test_the_reserved_slots_reach_the_sync_path_first() {
        let (tx, _rx) = mpsc::channel::<SendItem>(SEND_QUEUE_CAPACITY);
        // What `Mcu::send_payload` does: queue until only the headroom is left.
        while payload_has_room(&tx) {
            tx.try_send(SendItem::Payload(Payload::new()))
                .expect("room was just checked");
        }
        assert_eq!(tx.capacity(), SYNC_SEND_HEADROOM);

        let started = Instant::now();
        for slot in 0..SYNC_SEND_HEADROOM {
            try_send_bounded(&tx, SendItem::Payload(Payload::new()))
                .unwrap_or_else(|e| panic!("reserved slot {slot} is the sync path's: {e:?}"));
        }
        assert_eq!(tx.capacity(), 0, "the reserved slots are gone");
        assert!(
            started.elapsed() < Duration::from_millis(250),
            "the sync path does not wait for its own slots"
        );
    }

    /// A sync `Mcu::send` at the watermark still reaches the queue, and reports
    /// the command and the level once the reserved slots are spent too.
    ///
    /// The test's own runtime is single-threaded and never yields while it
    /// fills the queue, so the send task cannot drain a slot: the queue stays
    /// exactly as the sync sends left it.
    #[tokio::test]
    async fn test_a_full_queue_names_the_command_and_the_level() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(vec![])));
        let dictionary =
            Dictionary::from_json(serde_json::json!({"commands": {"get_clock": 5}})).unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        // The awaits the test makes are what would let the send task run, so
        // the queue is filled without one.
        while mcu.send_buf_tx.capacity() > SYNC_SEND_HEADROOM {
            mcu.send("get_clock", &[]).expect("a free slot");
        }
        for slot in 0..SYNC_SEND_HEADROOM {
            mcu.send("get_clock", &[])
                .unwrap_or_else(|e| panic!("reserved slot {slot} is the sync path's: {e}"));
        }

        let error = mcu
            .send("get_clock", &[])
            .expect_err("the queue is full and nothing drains it");
        let message = error.to_string();
        assert!(
            message.contains("get_clock"),
            "names the command: {message}"
        );
        assert!(
            message.contains("no available capacity"),
            "keeps the queue's own error: {message}"
        );
        assert!(
            message.contains(&format!("{SEND_QUEUE_CAPACITY} of {SEND_QUEUE_CAPACITY}")),
            "reports the level it was full at: {message}"
        );
    }

    /// `send_payload` never spends the slots it reserves for the sync path.
    #[tokio::test]
    async fn test_send_payload_leaves_the_sync_headroom_free() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(vec![])));
        let dictionary =
            Dictionary::from_json(serde_json::json!({"commands": {"get_clock": 5}})).unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        let payload = || mcu.encode("get_clock", &[]).expect("a known command");
        // Fill to the watermark without awaiting: the send task stays parked,
        // so the level under test is the one left here.
        while mcu.send_buf_tx.capacity() > SYNC_SEND_HEADROOM {
            mcu.send("get_clock", &[]).expect("a free slot");
        }

        // At the watermark the awaiting producer parks instead of taking a
        // reserved slot. One look at it says so without yielding: a poll that
        // awaited a timer would let the send task free room first.
        let mut queued = Box::pin(mcu.send_payload(payload()));
        let first = std::future::poll_fn(|cx| {
            use std::future::Future;
            std::task::Poll::Ready(queued.as_mut().poll(cx))
        })
        .await;
        assert!(
            first.is_pending(),
            "it waited for room rather than spending the headroom"
        );
        assert_eq!(mcu.send_buf_tx.capacity(), SYNC_SEND_HEADROOM);
        drop(queued);

        // Once it runs, the payload goes out and the headroom is intact.
        mcu.send_payload(payload())
            .await
            .expect("the send task drains the queue");
        assert!(mcu.send_buf_tx.capacity() >= SYNC_SEND_HEADROOM);
    }

    // -----------------------------------------------------------------------
    // flush — forcing a block boundary
    // -----------------------------------------------------------------------

    /// `flush` separates what is queued before it from what comes after: the two
    /// commands reach the wire as two frames instead of one coalesced frame.
    #[tokio::test]
    async fn test_flush_forces_a_block_boundary() {
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(1, &[6]),
                outputs: vec![],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5, "get_uptime": 6}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        // Queued close together these would coalesce; the barrier keeps them
        // apart. A merged frame would match neither mapping, so it would not be
        // recorded and the count would not reach two.
        mcu.send("get_clock", &[]).unwrap();
        mcu.flush(Duration::from_millis(500)).await.unwrap();
        mcu.send("get_uptime", &[]).unwrap();
        mcu.flush(Duration::from_millis(500)).await.unwrap();

        let frames = recorder.frames();
        assert_eq!(frames.len(), 2, "expected two frames, got {frames:?}");
        assert_eq!(frames[0].payload(), &[5]);
        assert_eq!(frames[1].payload(), &[6]);
    }

    /// A flush with nothing queued before it is already satisfied.
    #[tokio::test]
    async fn test_flush_with_nothing_queued_completes() {
        let device = FrameMock::new(vec![]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        mcu.flush(Duration::from_millis(500)).await.unwrap();

        assert!(recorder.frames().is_empty());
    }

    // -----------------------------------------------------------------------
    // Scheduling gates: min_clock / req_clock (`SendClocks`)
    // -----------------------------------------------------------------------

    /// A dictionary the gate tests run against: four no-argument commands
    /// whose id is the whole message on the wire — so counting id bytes in
    /// the recorded frames counts messages, whatever block they landed in —
    /// plus the `CLOCK_FREQ` the gates judge time by.
    fn gate_dictionary() -> Dictionary {
        Dictionary::from_json(serde_json::json!({
            "commands": {
                "get_clock": 5,
                "config_reset": 6,
                "emergency_stop": 7,
                "debug_nop": 8,
                "queue_digital_out oid=%c clock=%u on_ticks=%u": 12,
                "queue_step oid=%c interval=%u count=%u add=%i": 9
            },
            "config": {"CLOCK_FREQ": 16000000}
        }))
        .unwrap()
    }

    /// One block as the test expects it: the frame at sequence `seq`, and the
    /// empty ack that reports the **next** sequence — the number a firmware
    /// says when it took the block, and the one `Sender::settle` drains on
    /// (`block.seq < seen`).
    fn gate_block(seq: u8, payload: &[u8]) -> MappingEntry {
        MappingEntry {
            input: make_frame(seq, payload),
            outputs: vec![make_frame((seq + 1) & 0x0f, &[])],
        }
    }

    /// A transport with the gate dictionary installed and the estimate seeded
    /// at `base` — the clock every gate test starts from.
    fn gate_mcu(entries: Vec<MappingEntry>, base: u64) -> (Mcu, FrameRecorder) {
        let device = FrameMock::new(entries);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        mcu.install_dictionary(gate_dictionary()).unwrap();
        mcu.set_clock_base(base);
        (mcu, recorder)
    }

    /// How often `id` went out — the gate messages carry their id as their
    /// whole payload, so this counts messages rather than blocks.
    fn sent_count(recorder: &FrameRecorder, id: u8) -> usize {
        recorder
            .frames()
            .iter()
            .map(|frame| frame.payload().iter().filter(|&&byte| byte == id).count())
            .sum()
    }

    /// `min_schedule_time` answers upstream's `MCU.min_schedule_time`
    /// (`mcu.py:1187-1188` → `MIN_SCHEDULE_TIME = 0.100`), and it is the
    /// same number as [`MIN_REQTIME_DELTA`] — asserting both pins the two
    /// names to one value so they cannot drift apart.
    #[tokio::test]
    async fn test_min_schedule_time_is_the_upstream_0_100_schedule_lead() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(vec![])));

        assert_eq!(mcu.min_schedule_time(), 0.100);
        assert_eq!(mcu.min_schedule_time(), MIN_REQTIME_DELTA);
    }

    /// ① The `min_clock` floor: a message parked on a release clock in the
    /// future does not reach the wire before it, and does once the clock gets
    /// there (`serialqueue.c:556`, `ack_clock < min_clock` holds it back).
    #[tokio::test]
    async fn test_min_clock_holds_a_message_until_its_release() {
        const BASE: u64 = 1_000_000;
        const FREQ: f64 = 16_000_000.0;
        let release = BASE + (10.0 * FREQ) as u64;
        // The one block this test allows: `get_clock` (id 5).
        let (mcu, recorder) = gate_mcu(vec![gate_block(0, &[5])], BASE);

        mcu.send_with_clocks(
            "get_clock",
            &[],
            SendClocks {
                min_clock: Some(release),
                req_clock: None,
            },
        )
        .unwrap();

        // The estimate sits 10 s short of the release clock.
        sleep(Duration::from_millis(60)).await;
        assert!(
            recorder.frames().is_empty(),
            "sent before the min_clock: {:?}",
            recorder.frames()
        );

        // Give the clock the 10 s it was short by: `set_clock_base`
        // re-anchors the estimate, and the parked gate re-polls within
        // `GATE_REPOLL_MAX`.
        mcu.set_clock_base(release + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(recorder.frames().len(), 1, "not sent at the release clock");
        assert_eq!(recorder.frames()[0].payload(), &[5]);
    }

    /// ② `req_clock` priority and the lead window: both messages stay parked
    /// until the estimate is within `MIN_REQTIME_DELTA` of their `req_clock`
    /// (`PR_NOW`, `serialqueue.c:644-646`), and then the **lower** `req_clock`
    /// leads the block (`serialqueue.c:478-486`, "highest priority message").
    #[tokio::test]
    async fn test_req_clock_orders_messages_inside_the_lead_window() {
        const BASE: u64 = 1_000_000;
        const FREQ: f64 = 16_000_000.0;
        let req_low = BASE + (10.02 * FREQ) as u64;
        let req_high = BASE + (10.05 * FREQ) as u64;
        // One block: `emergency_stop` (7) was queued **second** but carries the
        // lower req_clock, so it leads the `config_reset` (6) behind it.
        let (mcu, recorder) = gate_mcu(vec![gate_block(0, &[7, 6])], BASE);

        mcu.send_with_clocks(
            "config_reset",
            &[],
            SendClocks {
                min_clock: None,
                req_clock: Some(req_high),
            },
        )
        .unwrap();
        mcu.send_with_clocks(
            "emergency_stop",
            &[],
            SendClocks {
                min_clock: None,
                req_clock: Some(req_low),
            },
        )
        .unwrap();

        // Neither req_clock is within 100 ms of the estimate yet.
        sleep(Duration::from_millis(60)).await;
        assert!(
            recorder.frames().is_empty(),
            "sent outside the lead window: {:?}",
            recorder.frames()
        );

        // Inside the window for both: PR_NOW, lowest req_clock first.
        mcu.set_clock_base(BASE + (10.1 * FREQ) as u64);
        sleep(Duration::from_millis(60)).await;
        let frames = recorder.frames();
        assert_eq!(frames.len(), 1, "expected one coalesced block: {frames:?}");
        assert_eq!(
            frames[0].payload(),
            &[7, 6],
            "the higher req_clock went first"
        );
    }

    /// ④ Capacity cut-back: a burst larger than the firmware's move queue is
    /// cut back to one release per freed slot, instead of all of it going at
    /// once — and `queue_digital_out` draws from the same pool, so a later
    /// move's floor is **its** completion clock.
    ///
    /// Capacity 2 over five move-class commands (`m1`..`m4` plus the digital
    /// out `d`): completions 10/20/30/40 s out, `d`'s own clock at 25 s. Each
    /// step walks `set_clock_base` over the next floor, so every boundary
    /// below is a clock, not a sleep.
    #[tokio::test]
    async fn test_a_full_move_capacity_releases_one_slot_at_a_time() {
        const BASE: u64 = 1_000_000;
        const FREQ: f64 = 16_000_000.0;
        let clock = |seconds: f64| BASE + (seconds * FREQ) as u64;
        let (c1, c2, c3, c4) = (clock(10.0), clock(20.0), clock(30.0), clock(40.0));
        let own_clock = clock(25.0);
        let free_at = |completion: u64| MoveSlots::free_at(completion, FREQ);

        // The digital out's bytes as the firmware sees them, so the block it
        // lands in can be matched exactly — built with the same parser the
        // transport encodes with.
        let mut parser = Parser::new();
        parser
            .register(12, "queue_digital_out oid=%c clock=%u on_ticks=%u")
            .unwrap();
        let d_args = [
            ArgValue::UInt8(0),
            ArgValue::UInt32(own_clock as u32),
            ArgValue::UInt32(1),
        ];
        let d_payload = parser
            .encode("queue_digital_out", &d_args)
            .unwrap()
            .into_raw();

        // Four blocks: the pair with room, one per released slot, and the
        // digital out on its own (it carries arguments).
        let (mcu, recorder) = gate_mcu(
            vec![
                gate_block(0, &[5, 6]),
                gate_block(1, &[7]),
                gate_block(2, &d_payload),
                gate_block(3, &[8]),
            ],
            BASE,
        );
        mcu.set_move_slot_capacity(2);

        // `m1`/`m2` take the two slots; `m3` queues behind `c1`, the digital
        // out behind `c2` and its own clock, `m4` behind the digital out.
        // The moves start in the past, so their `req_clock` never holds them.
        mcu.send_move("get_clock", &[], BASE, c1).unwrap();
        mcu.send_move("config_reset", &[], BASE, c2).unwrap();
        mcu.send_move("emergency_stop", &[], BASE, c3).unwrap();
        mcu.send_move("queue_digital_out", &d_args, own_clock, own_clock)
            .unwrap();
        mcu.send_move("debug_nop", &[], BASE, c4).unwrap();

        let counts = || [5, 6, 7, 8].map(|id| sent_count(&recorder, id));
        let d_sent = || {
            recorder
                .frames()
                .iter()
                .filter(|frame| frame.payload() == d_payload.as_slice())
                .count()
        };

        // At the base clock only the two with a free slot go.
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            counts(),
            [1, 1, 0, 0],
            "the over-capacity moves went with the burst"
        );
        assert_eq!(d_sent(), 0, "the digital out went before its own clock");

        // The first slot's completion (+ margin): exactly one move lands.
        mcu.set_clock_base(free_at(c1) + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            counts(),
            [1, 1, 1, 0],
            "one slot did not release exactly one move"
        );
        assert_eq!(d_sent(), 0);

        // The second slot's floor: `m4` still waits — its floor is the digital
        // out's completion (25 s), not `c2`, because `queue_digital_out` is in
        // the pool too. (Without it, `m4` would go right here.)
        mcu.set_clock_base(free_at(c2) + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            counts(),
            [1, 1, 1, 0],
            "m4's floor does not account for queue_digital_out's slot"
        );
        assert_eq!(d_sent(), 0);

        // The digital out releases on its own clock's lead window, ahead of
        // the pool floor that would otherwise still hold it.
        mcu.set_clock_base(own_clock - (MIN_REQTIME_DELTA * FREQ) as u64 + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(d_sent(), 1, "the digital out missed its req_clock window");
        assert_eq!(
            counts(),
            [1, 1, 1, 0],
            "m4 went before the digital out's slot freed"
        );

        // The slot the digital out held: `m4` goes at its completion (+ margin).
        mcu.set_clock_base(free_at(own_clock) + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            counts(),
            [1, 1, 1, 1],
            "moves did not release one slot at a time"
        );
        assert_eq!(d_sent(), 1);
    }

    /// The pool's arithmetic on its own: no floor while a slot is free, the
    /// floor is the completion that frees a slot, entries sort in even when
    /// they arrive out of order, an unarmed pool holds nothing back, and a
    /// live estimate drops what has already freed.
    #[test]
    fn test_move_slots_floor_is_the_slot_freeing_completion() {
        // No estimate, no dictionary: nothing frees, the release margin is 0,
        // and the assertions are the raw completion clocks.
        let gate = GateClock {
            estimate: Arc::new(StdMutex::new(None)),
            dictionary: Arc::new(StdMutex::new(None)),
        };
        let mut slots = MoveSlots {
            capacity: Some(2),
            pending: VecDeque::new(),
        };

        assert_eq!(slots.record(100, &gate), None, "first slot");
        assert_eq!(slots.record(200, &gate), None, "second slot");
        assert_eq!(
            slots.record(300, &gate),
            Some(100),
            "over capacity: the move waits for the entry that frees a slot"
        );
        // An out-of-order completion sorts in, and the floor recomputes over
        // the whole list: two slots free once the second-earliest (150) has
        // completed.
        assert_eq!(slots.record(150, &gate), Some(150));

        // Before the config handshake reports `move_count` the pool is
        // unarmed and holds nothing back.
        let mut unarmed = MoveSlots::default();
        assert_eq!(unarmed.record(100, &gate), None);

        // With a live estimate, entries whose free-at has passed drop off the
        // front instead of keeping the floor down: at an estimated 10 s, both
        // earlier completions (free at completion + 20 ms) are gone before the
        // third move is recorded.
        let dictionary = Arc::new(StdMutex::new(Some(Arc::new(gate_dictionary()))));
        let estimate = Arc::new(StdMutex::new(Some(ClockEstimate::seeded(
            Instant::now(),
            10_000_000,
        ))));
        let live = GateClock {
            estimate,
            dictionary,
        };
        let mut slots = MoveSlots {
            capacity: Some(2),
            pending: VecDeque::new(),
        };
        assert_eq!(slots.record(1_000_000, &live), None);
        assert_eq!(slots.record(2_000_000, &live), None);
        assert_eq!(
            slots.record(3_000_000, &live),
            None,
            "the freed entries were pruned, so the pool has room again"
        );
    }

    /// ③ The safety window around a slot-starved send: with the pool full, the
    /// move is released by its floor **and** goes no later than its own start
    /// clock minus `MIN_SCHEDULE_TIME` — the upper edge is asserted against the
    /// **start clock** (the batch's first step, per Q2(b)), and it is the
    /// property that keeps a full pool from turning overflow prevention into a
    /// "Timer too close" (`src/sched.c:94`). The lower edge (never *before* the
    /// release) is test ①'s single-gate job: with a future `req_clock` also on
    /// the message, the two gates only ever combine (release = max of both), so
    /// one message cannot show `min` alone — see the paired configs in ①/②.
    #[tokio::test]
    async fn test_slot_release_and_start_clock_bracket_the_send() {
        const BASE: u64 = 1_000_000;
        const FREQ: f64 = 16_000_000.0;
        let completion = BASE + 16_000_000; // 1.0 s: the predecessor in the pool
        let floor = MoveSlots::free_at(completion, FREQ); // +20 ms margin
        let start = completion + 3_200_000; // 0.2 s later: this move's own clock
        let window = start - (MIN_REQTIME_DELTA * FREQ) as u64; // opens at +0.1 s
        assert!(floor < window, "the floor must sit inside the lead window");

        // One slot: the predecessor takes it, the move queues behind it.
        let (mcu, recorder) = gate_mcu(vec![gate_block(0, &[5]), gate_block(1, &[6])], BASE);
        mcu.set_move_slot_capacity(1);
        mcu.send_move("get_clock", &[], BASE, completion).unwrap();
        sleep(Duration::from_millis(60)).await;
        assert_eq!(sent_count(&recorder, 5), 1, "the predecessor goes");

        // The move: floor = free-at(predecessor completion), req = start.
        mcu.send_move("config_reset", &[], start, start + 1_600_000)
            .unwrap();
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            sent_count(&recorder, 6),
            0,
            "before the slot releases, the move does not go"
        );

        // The slot releases (floor passed) but the req window has not opened:
        // both gates have to pass, so it still waits (② pins the window itself).
        mcu.set_clock_base(floor + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            sent_count(&recorder, 6),
            0,
            "slot released, but its own start window has not opened"
        );

        // Window opens → out it goes: `sent_clock <= start - MIN_SCHEDULE_TIME`
        // (+1 tick of re-poll). If a (buggy) floor ever sat *past* this point,
        // this step would stay red — the Timer-too-close detector.
        mcu.set_clock_base(window + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            sent_count(&recorder, 6),
            1,
            "not sent by its start clock minus MIN_SCHEDULE_TIME"
        );
    }

    /// ⑤ Step-path integration: `send_move_payload` carries the batch's
    /// `(start, completion)` window into the gates — the completion clock
    /// becomes the pool floor the next batch waits on, and the frame only
    /// leaves once that floor frees.
    #[tokio::test]
    async fn test_step_batches_carry_their_window_into_the_gates() {
        const BASE: u64 = 1_000_000;
        const FREQ: f64 = 16_000_000.0;
        let completion = BASE + 16_000_000;
        let floor = MoveSlots::free_at(completion, FREQ);

        let first = mcu_encode_step(1_000_000); // interval is incidental; the window does the gating
        let second = mcu_encode_step(2_000);

        let (mcu, recorder) = gate_mcu(vec![gate_block(0, &first), gate_block(1, &second)], BASE);
        mcu.set_move_slot_capacity(1);

        // First batch: pool has room, goes at once (start in the past → req open).
        mcu.send_move_payload(payload_from(&first), BASE, completion)
            .await
            .unwrap();
        sleep(Duration::from_millis(60)).await;
        assert_eq!(recorder.frames().len(), 1, "first batch on the wire");

        // Second batch: its completion entered the pool, so its floor is the
        // first batch's completion + margin — held until then.
        mcu.send_move_payload(payload_from(&second), BASE, completion + 8_000_000)
            .await
            .unwrap();
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            recorder.frames().len(),
            1,
            "completion clock did not reach the pool: no floor held it"
        );

        mcu.set_clock_base(floor + 1);
        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            recorder.frames().len(),
            2,
            "released at its predecessor's slot-free clock"
        );
    }

    /// Encode one `queue_step` for the step-path test (bytes the wire expects).
    fn mcu_encode_step(interval: u32) -> Vec<u8> {
        let mut parser = Parser::new();
        parser
            .register(9, "queue_step oid=%c interval=%u count=%u add=%i")
            .unwrap();
        parser
            .encode(
                "queue_step",
                &[
                    ArgValue::UInt8(0),
                    ArgValue::UInt32(interval),
                    ArgValue::UInt32(1),
                    ArgValue::Int32(0),
                ],
            )
            .unwrap()
            .into_raw()
    }

    /// Raw payload → a [`Payload`] to hand the send path.
    fn payload_from(raw: &[u8]) -> Payload {
        Payload::from_raw(raw.to_vec())
    }

    /// ⑥ An unknown clock never blocks: no estimate seed means
    /// [`SendClocks::released`] opens both gates (`serialqueue.c:612-618`,
    /// "Clock unknown during initial startup … return PR_NOW") — the send chain
    /// must not be able to park on a clock the host cannot judge.
    #[tokio::test]
    async fn test_an_unknown_clock_never_blocks_a_gated_message() {
        // Deliberately **no** `set_clock_base`: `estimated_clock()` is `None`.
        let device = FrameMock::new(vec![gate_block(0, &[5])]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        mcu.install_dictionary(gate_dictionary()).unwrap();
        assert!(
            mcu.estimated_clock().is_none(),
            "clock must be unknown here"
        );

        mcu.send_with_clocks(
            "get_clock",
            &[],
            SendClocks {
                min_clock: Some(u64::MAX - 1),
                req_clock: Some(u64::MAX - 1),
            },
        )
        .unwrap();

        sleep(Duration::from_millis(60)).await;
        assert_eq!(
            recorder.frames().len(),
            1,
            "a gated message parked on an unjudgeable clock"
        );
    }

    /// An unanswered block goes out again on the retransmit timer, without
    /// waiting for the firmware to speak first (`serialqueue.c:422-446`).
    #[tokio::test]
    async fn test_an_unanswered_block_is_retransmitted() {
        // Two mappings for the same frame: the original and the retransmit. A
        // third attempt would find no mapping and record nothing.
        let device = FrameMock::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary =
            Dictionary::from_json(serde_json::json!({"commands": {"get_clock": 5}})).unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        mcu.send("get_clock", &[]).unwrap();
        // The first send is immediate; the retransmit waits out MIN_RTO (25 ms).
        tokio::time::sleep(Duration::from_millis(120)).await;

        let frames = recorder.frames();
        assert!(
            frames.len() >= 2,
            "expected the block to be sent again, got {frames:?}"
        );
        // A retransmit keeps the sequence: the firmware is waiting for exactly
        // that block (see `Sender::resend_block`).
        assert_eq!(frames[0].seq(), 0);
        assert_eq!(frames[1].seq(), 0);
    }

    // -----------------------------------------------------------------------
    // The clock estimate: half-RTT anchoring, the frequency fit, the window
    // -----------------------------------------------------------------------

    /// The reading a `(sent, received, clock)` triple reports belongs to the
    /// round trip's **midpoint** — half a trip after `sent`, half before
    /// `received` — so the anchor, and with it the offset every read
    /// extrapolates from, counts half the trip rather than the whole one.
    #[test]
    fn test_the_offset_counts_half_the_round_trip() {
        let sent = Instant::now();
        let received = sent + Duration::from_millis(10);
        let sample = ClockSample {
            sent,
            received,
            clock: 40_000_000,
        };
        let freq = 20_000_000.0;

        assert_eq!(sample.midpoint(), sent + Duration::from_millis(5));

        let estimate = ClockEstimate::from_sample(sample);
        // Half of the 10 ms trip at 20 MHz is 5 ms × 20 MHz = 100 000 ticks:
        // by `received` the firmware has ticked that far past the reading.
        assert_eq!(
            estimate.clock_at(received, freq),
            40_100_000,
            "the second half of the trip is counted forward"
        );
        assert_eq!(
            estimate.clock_at(sample.midpoint() + Duration::from_secs(1), freq),
            40_000_000 + 20_000_000,
            "and the anchor then advances at the fitted rate"
        );
    }

    /// The frequency comes from a least-squares fit over the sample window —
    /// upstream's `clock_covariance / time_variance` (`clocksync.py:128`) — so
    /// a firmware ticking 1000 ppm fast is measured as ticking 1000 ppm fast.
    #[test]
    fn test_the_frequency_is_fitted_from_the_sample_window() {
        let nominal = 20_000_000.0;
        let actual = nominal * 1.001;
        let origin = Instant::now();
        let mut estimate = ClockEstimate::seeded(origin, 0);
        // One sample every 0.5 s, each reading the true clock at its own
        // midpoint (2 ms trip) — a perfectly straight line at `actual`.
        for step in 1..=CLOCK_FIT_WINDOW {
            let sent = origin + Duration::from_micros(step as u64 * 500_000);
            let received = sent + Duration::from_millis(2);
            let midpoint_offset = sent.duration_since(origin).as_secs_f64() + 0.001;
            let clock = (midpoint_offset * actual) as u64;
            estimate.record(ClockSample {
                sent,
                received,
                clock,
            });
        }

        let freq = estimate.freq.expect("the window fits a rate");
        assert!(
            (freq - actual).abs() < 100.0,
            "fitted {freq}, want {actual}"
        );
        assert!((freq - nominal).abs() > 10_000.0, "not the nominal rate");
    }

    /// The window is a window: once it is full of newer samples the old ones
    /// are pushed out entirely, and the fit follows the new rate alone.
    #[test]
    fn test_old_samples_are_pushed_out_of_the_fit_window() {
        let nominal = 20_000_000.0;
        let actual = nominal * 1.001;
        let origin = Instant::now();
        let mut estimate = ClockEstimate::seeded(origin, 0);
        let push = |estimate: &mut ClockEstimate, step: usize, rate: f64| {
            let sent = origin + Duration::from_micros(step as u64 * 500_000);
            let received = sent + Duration::from_millis(2);
            let midpoint_offset = sent.duration_since(origin).as_secs_f64() + 0.001;
            let clock = (midpoint_offset * rate) as u64;
            estimate.record(ClockSample {
                sent,
                received,
                clock,
            });
        };

        // A full window of samples from a firmware at the nominal rate …
        for step in 1..=CLOCK_FIT_WINDOW {
            push(&mut estimate, step, nominal);
        }
        let before = estimate.freq.expect("the first window fits a rate");
        assert!((before - nominal).abs() < 100.0, "fitted {before}");

        // … then a full window from one 1000 ppm fast. The window holds
        // `CLOCK_FIT_WINDOW` samples, so not one of the old ones survives.
        for step in (CLOCK_FIT_WINDOW + 1)..=(2 * CLOCK_FIT_WINDOW) {
            push(&mut estimate, step, actual);
        }
        assert_eq!(
            estimate.samples.len(),
            CLOCK_FIT_WINDOW,
            "nothing accumulates"
        );
        let after = estimate.freq.expect("the second window fits a rate");
        assert!(
            (after - actual).abs() < 100.0,
            "the old samples are gone: fitted {after}, want {actual}"
        );
    }

    /// Round trips from a firmware ticking at exactly the nominal rate leave
    /// the estimate where the single-point snapshot would have put it: the
    /// midpoint anchor and the fitted rate agree with `seed + elapsed × freq`.
    #[test]
    fn test_no_drift_samples_agree_with_the_snapshot_formula() {
        let freq = 20_000_000.0;
        let origin = Instant::now();
        let mut estimate = ClockEstimate::seeded(origin, 1_000_000);
        // A sample one second in: the reading is the true clock at the round
        // trip's midpoint (10 ms trip), i.e. on the very timeline the seed
        // started.
        let sent = origin + Duration::from_secs(1);
        let received = sent + Duration::from_millis(10);
        // The true clock at the midpoint: 1.005 s × 20 MHz = 20 100 000 ticks.
        let clock = 1_000_000 + 20_100_000;
        estimate.record(ClockSample {
            sent,
            received,
            clock,
        });

        let got = estimate.clock_at(received, freq);
        let snapshot = 1_000_000 + (received.duration_since(origin).as_secs_f64() * freq) as u64;
        assert_eq!(got, snapshot, "no drift: the sample moves nothing");
    }

    /// The seed path end to end: no round trips behind it, so
    /// `estimated_clock` answers the old snapshot's
    /// `base + elapsed × nominal` — pinned from both sides, because the seed
    /// instant is only known to sit between the instants around the call.
    #[tokio::test]
    async fn test_a_seeded_clock_estimates_like_the_old_snapshot() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(vec![])));
        mcu.install_dictionary(
            Dictionary::from_json(serde_json::json!({
                "commands": {"get_clock": 5},
                "config": {"CLOCK_FREQ": 20000000}
            }))
            .unwrap(),
        )
        .unwrap();
        assert_eq!(mcu.estimated_clock(), None, "nothing seeded yet");

        let before = Instant::now();
        mcu.set_clock_base(1_000_000);
        let after = Instant::now();
        let read_from = Instant::now();
        let got = mcu.estimated_clock().unwrap();
        let read_to = Instant::now();

        let freq = 20_000_000.0;
        let low =
            1_000_000 + (read_from.saturating_duration_since(after).as_secs_f64() * freq) as u64;
        let high = 1_000_000 + (read_to.duration_since(before).as_secs_f64() * freq) as u64;
        assert!(
            (low..=high).contains(&got),
            "{got} outside the snapshot's [{low}, {high}]"
        );
    }

    // -----------------------------------------------------------------------
    // Real board (ignored by default)
    // -----------------------------------------------------------------------

    /// Frame-sequence synchronisation against a real MCU.
    ///
    /// The fake-device tests above pin each rule of the transport; this one asks
    /// the firmware to behave the way they assume.
    ///
    /// It is a configuration-driven hardware test: it declares that it needs the
    /// main MCU, and `hardware_test` derives everything else from the printer
    /// config named by `KLIPPERX_HW_CONFIG`. Run it explicitly:
    ///
    /// ```text
    /// KLIPPERX_HW_CONFIG=~/printer_data/config/printer.cfg \
    ///   cargo test -p klipperx --lib test_frame_sequence_sync_against_a_real_board \
    ///   -- --ignored --nocapture
    /// ```
    ///
    /// See `docs/klippy/developer-manual/testing.md` for the full story; the
    /// `hardware_test` module is the reference. A plain `cargo test` never runs
    /// it (`#[ignore]`), and asking for it explicitly on a machine with no
    /// `KLIPPERX_HW_CONFIG` prints `HW-IGNORED: …` and passes: a config that is
    /// not there is a skip, reported rather than hidden.
    ///
    /// Once it runs it owns the board exclusively — `hardware_test` serialises
    /// hardware tests — and it deliberately leaves that board running: a
    /// firmware is only reset by a power cycle, never by a port reopen, so the
    /// second connection below is exactly the "board that never rebooted" the
    /// takeover path exists for.
    #[tokio::test]
    #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
    async fn test_frame_sequence_sync_against_a_real_board() {
        let Some(machine) = crate::hardware_test::acquire(
            "test_frame_sequence_sync_against_a_real_board",
            &crate::hardware_test::Requires::new().mcu(),
        ) else {
            return;
        };
        let open = || machine.open_mcu().expect("the board's transport must open");
        let seq = |mcu: &Mcu| mcu.wire.next.load(Ordering::Relaxed) & 0xf;

        // 1. Whichever session the board is in — just booted, or still running
        //    from a previous host — identify has to complete. A connection that
        //    could only ever start at sequence 0 would time out here.
        let mcu = Mcu::connect("mcu", open())
            .await
            .expect("identify must complete (fresh firmware, or a session to take over)");
        println!(
            "connect #1: took_over={} at sequence {}",
            mcu.took_over_session(),
            seq(&mcu),
        );

        // 2. Round trips across the 4-bit wraparound. Identify alone already
        //    sent roughly ninety blocks (the dictionary is that long), so a
        //    counter that never wrapped would have stalled long before this.
        let clock = McuClock::new(Arc::clone(&mcu), ManualReactor::shared());
        for round in 0..40 {
            clock
                .get_clock()
                .await
                .unwrap_or_else(|e| panic!("get_clock round {round} failed: {e}"));
        }

        // 3. Leave the firmware unambiguously past the "just booted" range. The
        //    takeover check deliberately accepts 0 or 1 (a freshly booted board
        //    answers the first block with one of them), so a connection can only
        //    be *guaranteed* to report a takeover once the counter is past that.
        while seq(&mcu) < 2 {
            clock
                .get_clock()
                .await
                .expect("get_clock while nudging the counter past the fresh range");
        }
        let running_at = seq(&mcu);
        drop(clock);
        drop(mcu);
        // Let the receive task finish and the port close before reopening it.
        sleep(Duration::from_millis(300)).await;

        // 4. The board was never reset, so its counter carried on from where
        //    the first connection left it. The new connection numbers its first
        //    block 0, which the firmware cannot place: it naks with its own
        //    number, and the transport has to adopt it and put the request back
        //    on the wire under that number.
        let mcu = Mcu::connect("mcu", open())
            .await
            .expect("the second identify must complete by taking the session over");
        assert!(
            mcu.took_over_session(),
            "the board was still running at sequence {running_at}; \
             the new connection had to take it over"
        );
        println!(
            "connect #2: took_over={} adopted sequence {}",
            mcu.took_over_session(),
            seq(&mcu),
        );

        // 5. And the taken-over session is genuinely usable, not just identified.
        let clock = McuClock::new(Arc::clone(&mcu), ManualReactor::shared());
        for round in 0..40 {
            clock
                .get_clock()
                .await
                .unwrap_or_else(|e| panic!("get_clock after takeover (round {round}) failed: {e}"));
        }
    }

    /// How many resets the R2 case performs.
    const RESET_ROUNDS: usize = 3;

    /// How long to wait before reopening a board that was told to `reset`: the
    /// firmware has to reboot, and on a native-USB board re-enumerate, before the
    /// port can be opened again.
    const RESET_RECONNECT_DELAY: Duration = Duration::from_millis(250);

    /// How many times to reopen before calling the board lost. Bounded: 40 ×
    /// 250 ms is ten seconds for one reboot, and a board that is still not back
    /// by then did not come back.
    const RESET_RECONNECT_ATTEMPTS: usize = 40;

    /// How long to give the `reset` command's flush before closing the port.
    const RESET_FLUSH_TIMEOUT: Duration = Duration::from_secs(1);

    /// The pause between the flushed `reset` and closing the port, so the bytes
    /// reach the firmware first — upstream pauses the same 15 ms between `reset`
    /// and `_disconnect()` (`klippy/mcu.py:730-747`).
    const RESET_DISCONNECT_DELAY: Duration = Duration::from_millis(15);

    /// Reopen the board after it was told to `reset`, retrying while it reboots.
    ///
    /// The board is gone the moment the port is closed, so an open failure is the
    /// expected first answer; the last one is reported when it never comes back —
    /// a serial node that moved out from under `serial:` is exactly this failure.
    async fn reopen_after_reset(
        machine: &crate::hardware_test::Machine,
    ) -> Result<Arc<Mcu>, String> {
        let mut last = String::new();
        for _ in 0..RESET_RECONNECT_ATTEMPTS {
            sleep(RESET_RECONNECT_DELAY).await;
            let interface = match machine.open_mcu() {
                Ok(interface) => interface,
                Err(err) => {
                    last = err;
                    continue;
                }
            };
            // A failed `Mcu::connect` drops the half-built session on the way out,
            // which closes the port again — so the next attempt opens it cleanly.
            match Mcu::connect("mcu", interface).await {
                Ok(mcu) => return Ok(mcu),
                Err(err) => last = err.to_string(),
            }
        }
        Err(last)
    }

    /// The board half of `TESTING.md`'s R2, on a real board: three firmware
    /// resets in a row, and the board comes back every time.
    ///
    /// R2 is “软复位与会话恢复” (`TESTING.md:39-41`): the real test is three
    /// `FIRMWARE_RESTART`s in a row, and its judgment sentence is “板子每次都能重启
    /// 回来 … 不出现串口打不开或设备节点漂移”. Each round here does the three things
    /// that sentence is about:
    ///
    /// 1. **reboot the firmware** — the `reset` command on the live connection,
    ///    which is the `restart_method: command` path (`mcu/object.rs`'s
    ///    `before_firmware_restart`, `klippy/mcu.py:730-747`: send `reset`, flush,
    ///    pause, disconnect);
    /// 2. **reconnect** — reopen the transport while the board is rebooting,
    ///    retrying within a bounded budget, so a port that will not open or a node
    ///    that drifted is a failure and not a hang;
    /// 3. **identify and use the session** — `Mcu::connect` runs the handshake, and
    ///    one `get_clock` round trip proves the session is usable rather than
    ///    merely identified.
    ///
    /// What it asserts, round by round: the reconnect identifies within the budget,
    /// the firmware that answers is a **freshly booted** one
    /// (`!took_over_session()` — a board that never rebooted answers the new
    /// connection with its old sequence, and the transport takes that session over,
    /// `Mcu::took_over_session`), and the new session answers `get_clock`. Three
    /// rounds with no failure is the pass; the timeout of the identify handshake,
    /// an open failure that outlives the budget, a taken-over session, or a failed
    /// `get_clock` is the failure.
    ///
    /// # Dangerous action
    ///
    /// **This makes the board's firmware reboot for real, three times.** Any host
    /// session on that board dies with the first reset. A reboot drops whatever the
    /// firmware was driving, so **do not run this with a hot or busy printer**:
    /// power the heaters down and let the host release the board before running it.
    ///
    /// # What it touches
    ///
    /// Nothing physical. There is no assumed wiring beyond `[mcu]`'s own transport,
    /// no stepper is moved, no pin is written, no heater is commanded, and no
    /// endstop is read: the only messages on the wire are the identify handshake
    /// (read), `reset` (write), and `get_clock` (write and read). The only bound it
    /// needs is the reconnect budget (40 × 250 ms = 10 s per round) — there is no
    /// motion to bound.
    ///
    /// # What it does not cover
    ///
    /// R2's other halves. The configuration handshake (“配置”) needs the assembled
    /// `ConfigBuilder`, and the API subscription (“订阅”, whose continuity the
    /// judgment sentence also asks for) needs a running `Printer`; `hardware_test`
    /// deliberately hands a test the parsed config and the transports, never a
    /// running machine, so both belong to the host layer rather than here.
    ///
    /// # Why the declaration is only `.mcu()`
    ///
    /// The declaration says the board is reachable. It is deliberately **not**
    /// `.option("mcu", "restart_method")`: the key is optional and its default is
    /// `command` (`mcu/restart_method.rs`), so the configs this case targets are
    /// exactly the ones that do not write it — declaring the option would skip
    /// them. What the case really needs is the firmware's `reset` command, which is
    /// a property of the firmware's dictionary, not of the config file; a firmware
    /// without it is detected after connecting and reported as `HW-IGNORED`, not
    /// declared.
    ///
    /// # Running it
    ///
    /// ```text
    /// KLIPPERX_HW_CONFIG=~/printer_data/config/printer.cfg \
    ///   cargo test -p klipperx --lib test_firmware_reset_comes_back_three_times_on_a_real_board \
    ///   -- --ignored --nocapture
    /// ```
    ///
    /// On a machine with no `KLIPPERX_HW_CONFIG` it prints `HW-IGNORED: …` and
    /// passes. It leaves the board freshly rebooted and unconfigured.
    #[tokio::test]
    #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
    async fn test_firmware_reset_comes_back_three_times_on_a_real_board() {
        let Some(machine) = crate::hardware_test::acquire(
            "test_firmware_reset_comes_back_three_times_on_a_real_board",
            &crate::hardware_test::Requires::new().mcu(),
        ) else {
            return;
        };

        let mut mcu = Mcu::connect(
            "mcu",
            machine.open_mcu().expect("the board's transport must open"),
        )
        .await
        .expect("identify must complete on the board as it is now");
        // The reboot is the firmware's own `reset` command. A firmware that does
        // not declare it cannot be rebooted from the host at all — R2's reboot half
        // has nothing to drive there (a `config_reset` board is cleared in place,
        // not restarted) — so report the skip in the same shape the framework uses
        // instead of pretending to have tested it.
        if !mcu.has_message(Reset::NAME) {
            println!(
                "HW-IGNORED: test_firmware_reset_comes_back_three_times_on_a_real_board: the \
                 firmware declares no 'reset' command, so the host cannot reboot it — R2's \
                 reboot half has nothing to drive on this board"
            );
            return;
        }
        println!(
            "connected: took_over={} (the board as it is now, before any reset)",
            mcu.took_over_session(),
        );

        for round in 1..=RESET_ROUNDS {
            let started = Instant::now();

            // 1. Reboot the firmware on the live connection, the way
            //    `before_firmware_restart` does for `restart_method: command`:
            //    `reset`, flushed so the bytes leave the host before the port is
            //    closed. The flush often reports the transport going away as the
            //    board reboots; the command was written first, so that is not an
            //    error.
            mcu.send_msg(&Reset).unwrap_or_else(|err| {
                panic!("round {round}: the firmware rejected 'reset': {err}")
            });
            let _ = mcu.flush(RESET_FLUSH_TIMEOUT).await;
            // Let the flushed bytes reach the firmware: closing the port
            // immediately could cut a write the tty driver still holds, and a
            // firmware that never saw `reset` never comes back.
            sleep(RESET_DISCONNECT_DELAY).await;
            // The old session must be the port's only owner, or dropping it closes
            // nothing and its tasks keep reading the tty into the next session.
            assert_eq!(
                Arc::strong_count(&mcu),
                1,
                "round {round}: another handle still holds the session; the old tasks \
                 would stay on the port"
            );
            drop(mcu);

            // 2. Reopen while the firmware reboots; identify runs in
            //    `Mcu::connect`.
            let reconnected = reopen_after_reset(&machine).await.unwrap_or_else(|last| {
                panic!("round {round}: the board did not come back after 'reset': {last}")
            });

            // 3. A booted firmware is what answers: a board that never rebooted
            //    answers the new connection's first block with its old sequence,
            //    and the transport takes that session over (`took_over_session`).
            assert!(
                !reconnected.took_over_session(),
                "round {round}: the reconnected firmware was still the pre-reset \
                 session — it did not reboot"
            );

            // 4. The new session is usable, not just identified.
            let clock = McuClock::new(Arc::clone(&reconnected), ManualReactor::shared());
            clock.get_clock().await.unwrap_or_else(|err| {
                panic!("round {round}: get_clock failed on the reconnected board: {err}")
            });
            drop(clock);

            println!(
                "reset round {round}/{RESET_ROUNDS}: rebooted, identified and answered \
                 get_clock in {:?}",
                started.elapsed(),
            );
            mcu = reconnected;
        }
    }

    // -----------------------------------------------------------------------
    // The reset case's declaration (no board needed)
    // -----------------------------------------------------------------------

    /// The declaration of [`test_firmware_reset_comes_back_three_times_on_a_real_board`]
    /// is the right one: a config with a reachable `[mcu]` satisfies it, and one
    /// without — commented out, absent, or carrying no interface — does not.
    ///
    /// `hardware_test` decides that from the parsed config alone, so this can be
    /// checked without a board. It is what keeps the declaration from silently
    /// skipping the case on the very configs it is for: the parser decides
    /// “present”, so a commented-out `[mcu]` is simply absent.
    #[test]
    fn test_the_reset_case_declaration_matches_a_config() {
        use crate::core::klippy::config::Config;
        use crate::hardware_test::{check, Missing, Requires};

        let requires = Requires::new().mcu();
        let parse = |text: &str| Config::from_text(text).expect("the fixture parses").0;
        let missing_main_mcu = || Err(vec![Missing::Mcu("mcu".to_string())]);

        // Reachable over serial: the case runs …
        assert_eq!(
            check(&parse("[mcu]\nserial: /dev/ttyACM0\n"), &requires),
            Ok(())
        );
        // … and over CAN. Either interface is a reachable MCU.
        assert_eq!(
            check(&parse("[mcu]\ncanbus_uuid: abc123\n"), &requires),
            Ok(())
        );

        // No `[mcu]` section at all.
        assert_eq!(
            check(&parse("[stepper_x]\nendstop_pin: PA0\n"), &requires),
            missing_main_mcu()
        );
        // Commented out is absent — the real parser decides “present”.
        assert_eq!(
            check(&parse("# [mcu]\n# serial: /dev/ttyACM0\n"), &requires),
            missing_main_mcu()
        );
        // Present, but with nothing to open: not a reachable MCU.
        assert_eq!(
            check(&parse("[mcu]\nrestart_method: command\n"), &requires),
            missing_main_mcu()
        );
    }

    // -----------------------------------------------------------------------
    // Parser helpers (no background tasks needed)
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_send_invalid_command() {
        let device = FrameMock::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("test_mcu", interface);

        let result = mcu.send("nonexistent_cmd", &[ArgValue::UInt32(0)]);
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_send_wrong_param_count() {
        let device = FrameMock::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("test_mcu", interface);

        // %u requires one param
        let result = mcu.send("test_cmd", &[]);
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_send_wrong_param_type() {
        let device = FrameMock::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("test_mcu", interface);

        // test_cmd expects x=%u but we pass a string
        let result = mcu.send("test_cmd", &[ArgValue::Str("bad".to_string())]);
        assert!(result.is_err());
    }

    // -----------------------------------------------------------------------
    // Parser encode / decode roundtrip (no background tasks)
    // -----------------------------------------------------------------------

    #[test]
    fn test_parser_encode_decode_roundtrip() {
        let mut parser = Parser::new();
        parser.register(100, "test_cmd x=%u").unwrap();
        parser.register(101, "test_resp val=%u data=%.*s").unwrap();

        // Encode
        let payload = parser.encode("test_cmd", &[ArgValue::UInt32(42)]).unwrap();
        let raw = payload.into_raw();

        // Decode
        let frame = make_frame(0, &raw);
        let decoded = parser.decode(frame.into()).unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "test_cmd");
        assert_eq!(decoded[0].0.id, 100);
        assert_eq!(decoded[0].1.len(), 1);
        assert_eq!(decoded[0].1[0], ArgValue::UInt32(42));
    }

    #[test]
    fn test_parser_decode_response_message() {
        let mut parser = Parser::new();
        parser.register(101, "test_resp val=%u data=%.*s").unwrap();

        let mut payload = Payload::new();
        payload.push_i16(101).unwrap();
        payload.push_u32(99).unwrap();
        payload.push_bytes(b"ok").unwrap();

        let frame = make_frame(0x10, &payload.into_raw());
        let decoded = parser.decode(frame.into()).unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "test_resp");
        assert_eq!(decoded[0].1.len(), 2);
        assert_eq!(decoded[0].1[0], ArgValue::UInt32(99));
        assert_eq!(decoded[0].1[1], ArgValue::Bytes(b"ok".to_vec()));
    }

    // -----------------------------------------------------------------------
    // Drop / shutdown
    // -----------------------------------------------------------------------

    /// Dropping the `Mcu` must shut down cleanly even when the underlying
    /// device keeps its receive channel open forever.
    ///
    /// The `FrameMock` is filled with extra (unconsumed) mappings on purpose:
    /// `FrameMock::send` only drops the last `buf_tx` sender once all mappings
    /// have been consumed, so with mappings still queued the frame channel stays
    /// open and `receive()` blocks indefinitely. This guarantees the receive
    /// task does not end on its own, so the test truly exercises the shutdown
    /// path added by `impl Drop for Mcu`.
    ///
    /// The test verifies two things:
    /// 1. the async receive task is aborted, and
    /// 2. the blocking device read is released (via `Interface::shutdown`),
    ///    so the runtime shuts down instead of hanging on a parked
    ///    `spawn_blocking` thread.
    #[tokio::test]
    async fn test_drop_aborts_receive_task_with_open_interface() {
        let mappings: Vec<MappingEntry> = (0..4)
            .map(|i| MappingEntry {
                input: make_frame(i, b"cmd"),
                outputs: vec![make_frame(i + 16, b"resp")],
            })
            .collect();

        let device = FrameMock::new(mappings);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("drop_test", interface);

        // `JoinHandle` is not `Clone`; an `AbortHandle` lets us observe the
        // receive task after the `Mcu` (and its `JoinHandle`) is gone.
        let abort_handle = mcu
            .recv_handle
            .lock()
            .unwrap()
            .as_ref()
            .expect("receive task handle must be present")
            .abort_handle();

        // Let the receive task start and block inside `interface.receive()`.
        sleep(Duration::from_millis(20)).await;
        assert!(
            !abort_handle.is_finished(),
            "receive task should still be running while the interface is open"
        );

        // Actively drop the `Mcu`; this must abort the receive task.
        drop(mcu);

        // Give the runtime a chance to process the cancellation.
        sleep(Duration::from_millis(20)).await;
        assert!(
            abort_handle.is_finished(),
            "receive task should be aborted after the Mcu is dropped"
        );
    }

    // -----------------------------------------------------------------------
    // Explicit session close (B6 — host reset → reconnect teardown)
    // -----------------------------------------------------------------------

    /// `close` ends the session even though the `Arc` is still shared: both
    /// transport tasks stop, sends are refused, and not one more frame reaches
    /// the transport.
    ///
    /// This is what makes a reconnect's teardown independent of reference
    /// counting — on the real host the clock estimate and the chip's device
    /// slot keep their handles, so `Drop` never runs and the old write end
    /// retransmits against a dead fd (r8host's `Send failed (seq=9) … EIO`
    /// once a second, forever). The second `Arc` here stands in for those
    /// holders.
    #[tokio::test]
    async fn test_close_stops_the_session_while_the_arc_is_still_shared() {
        let wire = RecordingWire::new();
        let sent = wire.sent();
        let mcu = Arc::new(Mcu::for_test("closing", Interface::recording(wire.clone())));
        mcu.install_dictionary(
            Dictionary::from_json(serde_json::json!({"commands": {"ping": 7}})).unwrap(),
        )
        .unwrap();
        // One command on the wire, so the send task has real work behind it.
        mcu.send("ping", &[]).unwrap();
        mcu.flush(Duration::from_secs(1)).await.unwrap();
        assert!(!sent.lock().unwrap().is_empty(), "the session was live");
        let keeper = Arc::clone(&mcu); // the holder that outlives the close

        mcu.close();

        for _ in 0..100 {
            if mcu.transport_tasks_finished() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(
            mcu.transport_tasks_finished(),
            "both transport tasks stop while the Arc is still shared"
        );
        assert_eq!(
            Arc::strong_count(&mcu),
            2,
            "the close did not wait for the reference count"
        );
        assert!(mcu.is_closed());
        // Not one frame more, whatever a caller tries after the close.
        let frozen = sent.lock().unwrap().len();
        assert!(
            mcu.send("ping", &[]).is_err(),
            "a closed session refuses sends"
        );
        assert!(
            mcu.flush(Duration::from_millis(50)).await.is_err(),
            "and refuses barriers too"
        );
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(
            sent.lock().unwrap().len(),
            frozen,
            "zero frames on the transport after close"
        );
        drop(keeper);
    }

    /// The real-machine reconnect sequence (r8host 08:22:45–08:23:30): a
    /// session runs at a high sequence, the firmware resets, the host reopens
    /// the port — and on a UART the old session's tasks share the reopened
    /// line with the new one, because a tty never re-enumerates. With the old
    /// session closed first, the reopened connection owns the line: identify
    /// completes, and no frame numbered by the old session reaches the new
    /// window — in the log those frames were the old session's own drops
    /// (`Frame with sequence 112 … answers block 107 …`) and the new
    /// session's identify timed out forever.
    #[tokio::test]
    async fn test_a_closed_session_lets_the_reopened_identify_complete() {
        // The dictionary the handshake transfers: small enough that the
        // chunk loop below scripts each chunk and the terminator explicitly.
        let body = br#"{"commands":{"get_clock":5},"config":{"CLOCK_FREQ":20000000}}"#;
        let compressed = compress(body);
        // One shared wire: both sessions' receive tasks draw from the same
        // queue, exactly like the two fds on one UART in the log.
        let wire = RecordingWire::new();
        let mut offset = 0usize;
        loop {
            let end = (offset + IDENTIFY_CHUNK_SIZE as usize).min(compressed.len());
            let data = &compressed[offset..end];
            wire.reply_to(
                identify_request_payload(offset as u32),
                identify_response_payload(offset as u32, data),
            );
            if data.is_empty() {
                break;
            }
            offset = end;
        }

        // The old session, mid-session numbering: its window sits past 107
        // while the reopened connection starts from scratch.
        let previous = Arc::new(Mcu::for_test("old", Interface::recording(wire.clone())));
        previous.wire.next.store(107, Ordering::Relaxed);
        previous.wire.seen.store(106, Ordering::Relaxed);

        // `reset` + the reconnect's first step: the old session goes down
        // before the port comes back up — and the `Arc` is held here the
        // whole time, so only `close` can have stopped those tasks.
        previous.close();
        for _ in 0..100 {
            if previous.transport_tasks_finished() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(
            previous.transport_tasks_finished(),
            "the old session is gone while its Arc is still held"
        );

        // The reopened port: identify completes against the same wire, with
        // nobody left to steal its answers.
        let reopened = Mcu::connect("reopened", Interface::recording(wire.clone()))
            .await
            .expect("identify completes once the old session is closed");
        assert!(
            !reopened.took_over_session(),
            "a reset firmware counts from zero; the old numbering is not adopted"
        );
        assert!(
            reopened.wire.seen.load(Ordering::Relaxed) < 107,
            "no frame from the old session's numbering reached the new window"
        );
    }

    #[test]
    fn test_describe_message_names_the_parameters() {
        // The shape is the definition's own: `test_resp val=%u data=%.*s` with
        // the specifiers replaced by values, dynamic strings quoted.
        let response = Msg::parse(101, "test_resp val=%u data=%.*s").unwrap();
        assert_eq!(
            describe_message(
                &response,
                &[ArgValue::UInt32(7), ArgValue::Bytes(vec![0xaa, 0xbb])]
            ),
            r#"test_resp val=7 data=b"\xaa\xbb""#
        );

        // A string is quoted, so a value with a space cannot run into the
        // format around it.
        let ping = Msg::parse(10, "debug_ping data=%*s").unwrap();
        assert_eq!(
            describe_message(&ping, &[ArgValue::Str("two words".into())]),
            r#"debug_ping data="two words""#
        );

        // A message with no parameters is just its name.
        let bare = Msg::parse(5, "get_clock").unwrap();
        assert_eq!(describe_message(&bare, &[]), "get_clock");
    }

    #[test]
    fn test_describe_value_shows_each_type_readably() {
        assert_eq!(describe_value(&ArgValue::UInt8(1)), "1");
        assert_eq!(describe_value(&ArgValue::UInt16(2)), "2");
        assert_eq!(describe_value(&ArgValue::Int16(-3)), "-3");
        assert_eq!(describe_value(&ArgValue::UInt32(4)), "4");
        assert_eq!(describe_value(&ArgValue::Int32(-5)), "-5");

        // Strings are quoted and escaped; non-ASCII text stays itself.
        assert_eq!(describe_value(&ArgValue::Str("abc".into())), r#""abc""#);
        assert_eq!(describe_value(&ArgValue::Str("a\nb".into())), r#""a\nb""#);
        assert_eq!(describe_value(&ArgValue::Str("温度".into())), "\"温度\"");

        // Bytes are the same idea, but every non-printable byte is escaped.
        assert_eq!(
            describe_value(&ArgValue::Bytes(b"hi there".to_vec())),
            r#"b"hi there""#
        );
        assert_eq!(
            describe_value(&ArgValue::Bytes(vec![1, 2, 0xff])),
            r#"b"\x01\x02\xff""#
        );
    }
}
