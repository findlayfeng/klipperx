//! `[idle_timeout]` — put the machine into an idle state after a stretch with
//! no motion (upstream `klippy/extras/idle_timeout.py`).
//!
//! | option | meaning |
//! |---|---|
//! | `timeout` | seconds of idle before the idle gcode runs (default 600, `> 0`) |
//! | `gcode` | the script run once the timeout expires, as a template (default `DEFAULT_IDLE_GCODE`) |
//!
//! A timer registered at `klippy:ready` walks the machine through upstream's
//! three states, and `get_status` reports them as `state` / `printing_time` /
//! `idle_timeout`:
//!
//! * `Idle` — nothing has moved since the machine came up, or the idle gcode
//!   has run;
//! * `Printing` — the toolhead's print time is moving;
//! * `Ready` — idle, with the timeout armed.
//!
//! The section is **optional**, as it is upstream: `toolhead.add_printer_objects`
//! loads the module by name (`toolhead.py:603-611`) and `PrinterConfig.getsection`
//! hands an absent section an empty config, so the defaults above are the whole
//! configuration. `idle_timeout` is therefore in
//! [`DEFAULT_MODULES`](crate::core::klippy::load), and a `printer.cfg` that never
//! mentions the section still has `printer.idle_timeout` and
//! `settings.idle_timeout.timeout` — the two a client like `mainsail.cfg` reads
//! unconditionally.
//!
//! `idle_timeout:ready` and `idle_timeout:printing` carry the print time the
//! transition is dated with (`+ PIN_MIN_TIME`), and `idle_timeout:idle` the
//! print time the idle gcode ran at. `SET_IDLE_TIMEOUT TIMEOUT=<seconds>`
//! changes the timeout, and re-arms the timer when the machine is `Ready`.
//!
//! # What is not here
//!
//! Upstream's timing model leans on two things this host does not have, so the
//! state machine is approximated rather than copied:
//!
//! * **No `Reactor::update_timer`.** Upstream re-arms the timeout from outside
//!   its callback in three places (`idle_timeout.py:105,115` and the retired
//!   timer a print start re-arms). `IdleTimeout::arm` is the stand-in — it
//!   cancels the handle and registers a fresh timer — and the timer is never
//!   retired: `Machine::armed` is what upstream's retired timer means, and the
//!   registered timer is how this port keeps polling a machine nothing else
//!   wakes (see the next bullet).
//! * **The print-start source has a fallback.** `toolhead:sync_print_time` is
//!   the moment the toolhead raises its print time for new motion
//!   (`toolhead.py:264-268`), and `IdleTimeout::handle_sync_print_time` is what
//!   upstream's `handle_sync_print_time` (`idle_timeout.py:98-107`) does with
//!   it: `Printing`, an `idle_timeout:printing` event, and a re-armed timeout
//!   for when the buffer should have drained. Because that handler only runs
//!   once something sends the event, the tick still *observes* the print time
//!   as a fallback (`IdleTimeout::observe_print_time`): a print time that moved
//!   since the last tick is a print start — but only until the first event
//!   arrives (`Machine::seen_sync`), whereupon the observer retires so a print
//!   start is never counted twice. The fallback's cost is that a `Ready`
//!   machine is woken every `READY_TIMEOUT` rather than only at the timeout
//!   (upstream's own re-arm granularity); a machine that sends the event still
//!   wakes at that rate, because the timer stays registered here (previous
//!   bullet).
//!
//! The `idle_timeout:printing` payload is the toolhead's print time plus
//! `PIN_MIN_TIME`, which is what upstream's handler sends once the argument
//! order of its `(curtime, print_time, est_print_time)` signature against
//! `toolhead.py:267-268`'s `(curtime, est_print_time, print_time)` is taken into
//! account — the handler is ported with that same order (see
//! `IdleTimeout::handle_sync_print_time`).
//!
//! Three smaller pieces of upstream's checks are dropped rather than
//! approximated:
//!
//! * **The lookahead check.** Upstream's `toolhead.check_busy` reports whether
//!   the look-ahead queue is empty (`toolhead.py:500-502`); the motion layer
//!   here exposes no such read, and this module does not reach into it, so only
//!   the buffer arithmetic is kept. A toolhead with moves queued still looks
//!   busy through `print_time - est_print_time`.
//! * **The gcode-busy backoff.** Upstream retries in a second while
//!   `gcode.get_mutex()` says a script is running (`idle_timeout.py:70-72`).
//!   This host has no reactor mutex, so the check is gone; the state machine's
//!   result is the same, only one back-off is skipped.
//! * **`printer.is_shutdown()`** does not exist here; the tick reads
//!   `PrinterState` instead ([`Printer::get_state_message`]).
//!
//! The idle gcode itself is run detached (`tokio::spawn`, as
//! `extras/bulk_sensor.rs` does): a timer callback is synchronous and
//! `GCodeDispatch::run_script` is not. Upstream reaches `state = "Idle"` and the
//! `idle_timeout:idle` event *after* the script returns, and goes back to
//! `Ready` (to retry in a second) when it fails; here the state is settled to
//! `Idle` and the event sent before the script finishes, and a script failure is
//! logged instead. A render failure is still a retry at `eventtime + 1.`, as
//! upstream's is. The template is rendered with the same context a
//! `[gcode_macro]` body gets (`gcode_macro.py:106-113`), minus `params` /
//! `rawparams`, which only exist for a macro being run.
//!
//! Upstream's default `gcode` calls `TURN_OFF_HEATERS`, which this host does not
//! implement: that line reaches the dispatcher's unknown-command path (an info
//! line, not an error), so the idle gcode still runs.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_macro::PrinterGCodeMacro;
use crate::core::klippy::extras::template::{
    Builtin, Context, PrinterView, Rt, Template, TemplateError,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuObject;
use crate::core::klippy::printer::{Printer, PrinterObject, PrinterState};
use crate::core::klippy::reactor::TimerHandle;

// Only the bare section exists upstream (`idle_timeout.py:117-118`), and
// upstream builds it without one too — it is in the loader's `DEFAULT_MODULES`
// (`toolhead.py:603-611`; see the module docs).
section!("idle_timeout", order = 30, load = load_config);

/// Upstream's default idle script (`idle_timeout.py:8-13`); the leading newline
/// is part of it.
const DEFAULT_IDLE_GCODE: &str =
    "\n{% if 'heaters' in printer %}\n   TURN_OFF_HEATERS\n{% endif %}\nM84\n";

/// Seconds added to the print time the `ready` and `printing` events carry
/// (`PIN_MIN_TIME`, `idle_timeout.py:15`).
const PIN_MIN_TIME: f64 = 0.100;

/// How long the toolhead has to stand still before the machine counts as ready
/// (`READY_TIMEOUT`), and the granularity at which this port looks for the
/// print start upstream is told about (see the module docs).
const READY_TIMEOUT: f64 = 0.500;

/// The objects upstream looks up (`idle_timeout.py:18-19`, `:42`).
const TOOLHEAD_OBJECT: &str = "toolhead";
const MCU_OBJECT: &str = "mcu";

/// `SET_IDLE_TIMEOUT`'s help text, verbatim (`idle_timeout.py:108`).
const SET_IDLE_TIMEOUT_HELP: &str = "Set the idle timeout in seconds";

/// Upstream's `state` (`idle_timeout.py:24,32-40`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Ready,
    Printing,
}

impl State {
    /// The string `get_status` reports.
    fn as_str(self) -> &'static str {
        match self {
            State::Idle => "Idle",
            State::Ready => "Ready",
            State::Printing => "Printing",
        }
    }
}

/// What the tick changes: upstream's `state`, `last_print_start_systime` and
/// the arm state of its timeout timer.
///
/// The whole machine state is one mutex so a tick is one consistent read: the
/// tick's transitions are what other threads (a status query, the
/// `SET_IDLE_TIMEOUT` handler) read.
struct Machine {
    state: State,
    /// Whether the timeout is armed. Upstream retires the timer by returning
    /// `NEVER` from its callback (`idle_timeout.py:57`) and re-arms it from
    /// `toolhead:sync_print_time`; this port's timer stays registered, so this
    /// flag is what "retired" means (module docs).
    armed: bool,
    /// Whether a `toolhead:sync_print_time` has arrived: from then on the event
    /// is the print-start source, and the tick's print-time observation — the
    /// fallback for a machine nothing sends it from — retires
    /// (`IdleTimeout::observe_print_time`).
    seen_sync: bool,
    /// The print time the last tick saw, read only while [`Self::seen_sync`] is
    /// unset: the fallback's stand-in for `toolhead:sync_print_time` (see
    /// `IdleTimeout::observe_print_time`).
    last_print_time: f64,
    /// When the current `Printing` state began (`printing_time` counts from
    /// here, `idle_timeout.py:34-40`).
    last_print_start_systime: f64,
}

impl Default for Machine {
    fn default() -> Self {
        Self {
            // Upstream's initial state and `last_print_start_systime`
            // (`idle_timeout.py:24-25`).
            state: State::Idle,
            armed: true,
            seen_sync: false,
            last_print_time: 0.0,
            last_print_start_systime: 0.0,
        }
    }
}

/// What the timeout state machine asks of the toolhead: upstream's
/// `toolhead.check_busy` (`toolhead.py:500-502`) and
/// `toolhead.get_last_move_time` (`toolhead.py:320-326`).
///
/// The trait is the seam: the production implementor is `MotionToolhead`,
/// and a test drives the state machine with a stand-in instead of a motion
/// stack.
trait Toolhead: Send + Sync {
    /// The print time the planner has reached (`check_busy`'s `print_time`).
    fn print_time(&self) -> f64;
    /// Flush the planner and return the print time it reached
    /// (`get_last_move_time`, upstream's `idle_timeout.py:55`).
    fn get_last_move_time(&self) -> f64;
    /// The MCU's estimate of the print time now (`check_busy`'s
    /// `est_print_time`).
    fn estimated_print_time(&self, eventtime: f64) -> f64;
}

/// The production `Toolhead`: the toolhead object plus the primary MCU whose
/// clock dates its estimate (`toolhead.py:500-502`).
struct MotionToolhead {
    toolhead: Arc<ToolHeadObject>,
    mcu: Option<Arc<McuObject>>,
}

impl Toolhead for MotionToolhead {
    fn print_time(&self) -> f64 {
        self.toolhead.print_time()
    }

    fn get_last_move_time(&self) -> f64 {
        self.toolhead.get_last_move_time()
    }

    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        // Before the MCU is connected there is no estimate; the monotonic clock
        // is the same fallback `filament_motion_sensor` uses.
        self.mcu
            .as_ref()
            .and_then(|mcu| mcu.estimated_print_time(eventtime))
            .unwrap_or(eventtime)
    }
}

/// One `[idle_timeout]`.
pub struct IdleTimeout {
    /// The machine, for its reactor and its shutdown state.
    printer: Weak<Printer>,
    /// The dispatcher the idle gcode is run on and the commands are registered
    /// with (`idle_timeout.py:18`).
    gcode: Arc<GCodeDispatch>,
    /// The `timeout` option (`idle_timeout.py:25`).
    idle_timeout: Mutex<f64>,
    /// The compiled `gcode` option (`idle_timeout.py:26-28`).
    idle_gcode: Template,
    /// The toolhead, resolved at `klippy:ready` (module docs).
    toolhead: Mutex<Option<Arc<dyn Toolhead>>>,
    /// The timeout timer, for re-arming and for `Drop`.
    timer: Mutex<Option<TimerHandle>>,
    /// The state the tick walks.
    machine: Mutex<Machine>,
    /// The `Arc` the timer callback upgrades: the registry owns this object,
    /// and the reactor's callback must not.
    self_ref: Weak<IdleTimeout>,
}

impl IdleTimeout {
    /// Build the section: read its options, load the idle template, register
    /// `SET_IDLE_TIMEOUT` (`idle_timeout.py:18-31`).
    ///
    /// # Errors
    /// A `timeout` that is not above 0, a `gcode` template that does not
    /// compile, or a `gcode_macro` object that cannot be created.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| {
                ConfigError::new("the gcode dispatcher is not registered".to_string())
            })?;
        let idle_timeout =
            config.get_float_bounded("timeout", Some(600.0), None, None, Some(0.0), None)?;
        let gcode_macro = PrinterGCodeMacro::ensure(printer)?;
        let idle_gcode = gcode_macro.load_template(config, "gcode", Some(DEFAULT_IDLE_GCODE))?;

        let object = Arc::new_cyclic(|weak| Self {
            printer: Arc::downgrade(printer),
            gcode,
            idle_timeout: Mutex::new(idle_timeout),
            idle_gcode,
            toolhead: Mutex::new(None),
            timer: Mutex::new(None),
            machine: Mutex::new(Machine::default()),
            self_ref: weak.clone(),
        });

        let weak = Arc::downgrade(&object);
        object
            .gcode
            .register_command_with_params(
                "SET_IDLE_TIMEOUT",
                sync(move |gcmd: &GcodeCommand| {
                    let this = weak
                        .upgrade()
                        .ok_or_else(|| CommandError::new("the idle_timeout object is gone"))?;
                    this.cmd_set_idle_timeout(gcmd)
                }),
                Some(SET_IDLE_TIMEOUT_HELP),
                // The one word `cmd_set_idle_timeout` reads
                // (`idle_timeout.py:108-115`).
                &["TIMEOUT"],
                false,
            )
            .map_err(ConfigError::new)?;
        Ok(object)
    }

    fn lock_machine(&self) -> MutexGuard<'_, Machine> {
        self.machine.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The `timeout` option as configured.
    fn timeout(&self) -> f64 {
        *self.idle_timeout.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Resolve the toolhead and arm the timeout (`IdleTimeout.handle_ready`,
    /// `idle_timeout.py:41-45`).
    ///
    /// A config with `[idle_timeout]` and no toolhead never reaches here in
    /// practice (every configuration has a `[printer]`); the timeout is left
    /// unarmed rather than failing the machine.
    fn handle_ready(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) else {
            tracing::warn!("idle_timeout: the toolhead did not resolve");
            return;
        };
        let mcu = printer.lookup_object_as::<McuObject>(MCU_OBJECT);
        self.start(Arc::new(MotionToolhead { toolhead, mcu }));
    }

    /// Point the state machine at `toolhead` and register the timeout
    /// (`IdleTimeout.handle_ready`).
    ///
    /// Split out of `IdleTimeout::handle_ready` so a test can drive the timer
    /// with a stand-in toolhead through the same path production uses.
    fn start(&self, toolhead: Arc<dyn Toolhead>) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        // The print time the machine is at when the timeout arms: motion past
        // this point is what the fallback observation reads as a print (see
        // `IdleTimeout::observe_print_time`). Taking it here keeps a print time
        // that is already non-zero at boot from looking like one.
        self.lock_machine().last_print_time = toolhead.print_time();
        *self.toolhead.lock().unwrap_or_else(|p| p.into_inner()) = Some(toolhead);
        // Upstream registers its timer at `reactor.NOW` (`idle_timeout.py:43`).
        self.arm(printer.reactor().monotonic());
    }

    /// (Re-)register the timeout timer for `waketime`.
    ///
    /// The stand-in for upstream's `reactor.update_timer(timer, waketime)`,
    /// which this host's reactor has no equivalent for: the old handle is
    /// cancelled and a fresh timer is registered with the same callback.
    fn arm(&self, waketime: f64) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "idle_timeout",
            Box::new(move |eventtime| {
                let Some(this) = weak.upgrade() else {
                    return None;
                };
                this.tick(eventtime)
            }),
            waketime,
        );
        let mut timer = self.timer.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(previous) = timer.replace(handle) {
            previous.cancel();
        }
    }

    /// One timer callback: upstream's `timeout_handler`
    /// (`idle_timeout.py:75-97`) plus the print-time observation that stands in
    /// for `handle_sync_print_time` while nothing sends
    /// `toolhead:sync_print_time` (`:98-107`).
    ///
    /// The returned wake time is the next time this runs; `None` retires the
    /// timer, which is what upstream does on shutdown (it also retires it after
    /// the idle gcode; this port keeps it registered there — module docs).
    fn tick(&self, eventtime: f64) -> Option<f64> {
        let Some(printer) = self.printer.upgrade() else {
            return None;
        };
        if matches!(
            printer.get_state_message().category,
            PrinterState::Shutdown | PrinterState::Error
        ) {
            // Upstream's `printer.is_shutdown()` -> `reactor.NEVER`.
            return None;
        }
        let toolhead = self
            .toolhead
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()?;
        let print_time = toolhead.print_time();
        let idle_timeout = self.timeout();

        let (state, armed, print_started) = {
            let mut machine = self.lock_machine();
            let started = Self::observe_print_time(&mut machine, print_time, eventtime);
            (machine.state, machine.armed, started)
        };
        if print_started {
            // Upstream sends this from `handle_sync_print_time`; this is the
            // fallback path, for a machine whose toolhead does not send the
            // event (module docs). The payload is what its handler's
            // `est_print_time + PIN_MIN_TIME` comes to.
            printer.send_event(&KlippyEvent::IdleTimeoutPrinting {
                print_time: print_time + PIN_MIN_TIME,
            });
        }

        if state == State::Ready {
            // Upstream's `check_idle_timeout` (`idle_timeout.py:59-74`).
            let est_print_time = toolhead.estimated_print_time(eventtime);
            let idle_time = est_print_time - print_time;
            if idle_time >= idle_timeout {
                // The idle timeout has elapsed.
                return self.transition_idle_state(&printer, &toolhead, eventtime);
            }
            // `idle_time < 1.` is upstream's "the toolhead is busy" (the
            // lookahead check is dropped, module docs); `ready_wake` keeps that
            // case's wake time, capped so a print start is seen (module docs).
            return Some(ready_wake(eventtime, idle_time, idle_timeout));
        }

        if state == State::Idle && !armed {
            // The idle gcode has run: upstream's timer is retired here, and a
            // print start re-arms it. The registered timer is what observes
            // that print start, so this polls (module docs).
            return Some(eventtime + READY_TIMEOUT);
        }

        // Upstream's `timeout_handler` (`idle_timeout.py:75-97`).
        let est_print_time = toolhead.estimated_print_time(eventtime);
        let buffer_time = (print_time - est_print_time).min(2.0);
        if buffer_time > -READY_TIMEOUT {
            // The toolhead has not stood still long enough yet.
            return Some(eventtime + READY_TIMEOUT + buffer_time);
        }
        // Transition to "ready" state.
        {
            let mut machine = self.lock_machine();
            machine.state = State::Ready;
            machine.armed = true;
        }
        printer.send_event(&KlippyEvent::IdleTimeoutReady {
            print_time: est_print_time + PIN_MIN_TIME,
        });
        Some(ready_wake(
            eventtime,
            est_print_time - print_time,
            idle_timeout,
        ))
    }

    /// Upstream's `handle_sync_print_time` (`idle_timeout.py:98-107`): the
    /// toolhead raised its print time, so a print is starting — go `Printing`,
    /// send `idle_timeout:printing`, and re-arm the timeout for when the
    /// buffer should have drained.
    ///
    /// **The two print times are named in upstream's order, which is not the
    /// order they are sent in.** `toolhead.py:267-268` sends
    /// `(curtime, est_print_time, print_time)`; the handler signature is
    /// `(curtime, print_time, est_print_time)` (`idle_timeout.py:98`), so the
    /// `print_time` received here is upstream's **estimate** and
    /// `est_print_time` is the raised print time. That quirk is upstream's own
    /// and is kept: the body below binds the parameters as upstream does, so
    /// the timeout is armed at `READY_TIMEOUT + est - print` and the `printing`
    /// payload is the print time plus `PIN_MIN_TIME` (module docs).
    ///
    /// The handler reads the payload and this module's state only. It must not
    /// call back into the toolhead: the sender has just released the toolhead's
    /// state lock, and a read here would be a second lock on that path.
    fn handle_sync_print_time(&self, curtime: f64, print_time: f64, est_print_time: f64) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        {
            let mut machine = self.lock_machine();
            // From now on the event is the print-start source and the tick's
            // print-time observation is the fallback
            // (`IdleTimeout::observe_print_time`).
            machine.seen_sync = true;
            if machine.state == State::Printing {
                // Upstream returns here: `Printing` means this print's buffer
                // is still draining and the timer is already armed for it.
                return;
            }
            machine.state = State::Printing;
            machine.armed = true;
            machine.last_print_start_systime = curtime;
        }
        // Upstream's `update_timer(timer, curtime + check_time)`
        // (`idle_timeout.py:104-105`): `arm` replaces the registered timer, so
        // the tick's own renewal cannot run alongside this one.
        self.arm(curtime + READY_TIMEOUT + print_time - est_print_time);
        printer.send_event(&KlippyEvent::IdleTimeoutPrinting {
            print_time: est_print_time + PIN_MIN_TIME,
        });
    }

    /// The fallback print-start source: a print time that moved since the last
    /// tick means the toolhead is printing.
    ///
    /// Upstream learns this from the event `ToolHead._calc_print_time` sends
    /// when it raises the print time for new motion (`toolhead.py:264-268`) and
    /// answers it in [`IdleTimeout::handle_sync_print_time`]. That event is the
    /// source once one has been handled (`Machine::seen_sync`); this observation
    /// remains for the configuration where nothing sends it, and it must not
    /// report a start a second time — `last_print_start_systime` would be
    /// overwritten and `printing_time` would jump. Returns whether the state
    /// just became `Printing`.
    fn observe_print_time(machine: &mut Machine, print_time: f64, eventtime: f64) -> bool {
        if machine.seen_sync {
            return false;
        }
        if print_time <= machine.last_print_time {
            return false;
        }
        machine.last_print_time = print_time;
        if machine.state == State::Printing {
            return false;
        }
        machine.state = State::Printing;
        machine.armed = true;
        machine.last_print_start_systime = eventtime;
        true
    }

    /// Upstream's `transition_idle_state` (`idle_timeout.py:46-58`): run the
    /// idle gcode and settle into `Idle`.
    fn transition_idle_state(
        &self,
        printer: &Arc<Printer>,
        toolhead: &Arc<dyn Toolhead>,
        eventtime: f64,
    ) -> Option<f64> {
        // Upstream sets `Printing` before the script so a `sync_print_time` the
        // script itself provokes does not re-enter; the render below happens
        // with the state consistent with a script in flight.
        self.lock_machine().state = State::Printing;

        let script = match self.render(printer) {
            Ok(script) => script,
            Err(error) => {
                // Upstream: log, go back to `Ready` and retry in a second.
                tracing::error!("idle timeout gcode execution: {error}");
                self.lock_machine().state = State::Ready;
                return Some(eventtime + 1.0);
            }
        };

        // Upstream's `gcode.run_script(script)`, which is `async` here while a
        // timer callback is not: the script runs on the host's runtime and the
        // callback does not wait for it (module docs).
        let gcode = Arc::clone(&self.gcode);
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(error) = gcode.run_script(&script).await {
                        tracing::error!("idle timeout gcode execution: {error}");
                    }
                });
            }
            Err(_) => {
                tracing::warn!("idle_timeout: the idle gcode needs a runtime to run");
            }
        }

        // Upstream reaches these after the script returns; here they settle the
        // state the script left behind (module docs).
        let print_time = toolhead.get_last_move_time();
        {
            let mut machine = self.lock_machine();
            machine.state = State::Idle;
            machine.armed = false;
            // The idle script may have queued motion of its own; that is not a
            // print starting.
            machine.last_print_time = print_time;
        }
        printer.send_event(&KlippyEvent::IdleTimeoutIdle { print_time });
        Some(eventtime + READY_TIMEOUT)
    }

    /// Render the idle gcode (`idle_timeout.py:49`).
    ///
    /// # Errors
    /// The first template error, as upstream's `render` raises it.
    fn render(&self, printer: &Arc<Printer>) -> Result<String, TemplateError> {
        let mut context = Context::new();
        // The context a `[gcode_macro]` body gets (`gcode_macro.py:106-113`),
        // minus `params` / `rawparams`: those belong to a macro being run.
        context.insert(
            "printer",
            Rt::Printer(PrinterView::new(Arc::clone(printer))),
        );
        context.insert(
            "action_respond_info",
            Rt::Builtin(Builtin::RespondInfo(Arc::clone(printer))),
        );
        context.insert("action_raise_error", Rt::Builtin(Builtin::RaiseError));
        context.insert("range", Rt::Builtin(Builtin::Range));
        self.idle_gcode.render(&mut context)
    }

    /// `SET_IDLE_TIMEOUT TIMEOUT=<seconds>` (`idle_timeout.py:108-115`).
    fn cmd_set_idle_timeout(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let timeout = gcmd.get(
            "TIMEOUT",
            Some(self.timeout()),
            parse_float,
            None,
            None,
            Some(0.0),
            None,
        )?;
        *self.idle_timeout.lock().unwrap_or_else(|p| p.into_inner()) = timeout;
        gcmd.respond_info(&format!("idle_timeout: Timeout set to {timeout:.2} s"));
        let ready = self.lock_machine().state == State::Ready;
        if ready {
            // Upstream re-arms at `monotonic() + timeout` so the new value
            // takes effect now rather than at the old wake time.
            if let Some(printer) = self.printer.upgrade() {
                self.arm(printer.reactor().monotonic() + timeout);
            }
        }
        Ok(())
    }
}

/// The next wake time for a machine in `Ready`: upstream's
/// `eventtime + idle_timeout - idle_time`, but never later than
/// `READY_TIMEOUT` past the check, which is when this port looks for the
/// print start upstream is told about (module docs).
fn ready_wake(eventtime: f64, idle_time: f64, idle_timeout: f64) -> f64 {
    let upstream = if idle_time < 1.0 {
        eventtime + idle_timeout
    } else {
        eventtime + idle_timeout - idle_time
    };
    upstream.min(eventtime + READY_TIMEOUT)
}

impl PrinterObject for IdleTimeout {
    /// Upstream's `get_status` (`idle_timeout.py:32-40`).
    fn get_status(&self, eventtime: f64) -> Value {
        let machine = self.lock_machine();
        let printing_time = if machine.state == State::Printing {
            eventtime - machine.last_print_start_systime
        } else {
            0.0
        };
        let idle_timeout = *self.idle_timeout.lock().unwrap_or_else(|p| p.into_inner());
        json!({
            "state": machine.state.as_str(),
            "printing_time": printing_time,
            "idle_timeout": idle_timeout,
        })
    }
}

impl Drop for IdleTimeout {
    fn drop(&mut self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

/// Wire the built object into the printer's `klippy:ready` event.
///
/// Upstream registers that handler inside `__init__`; here the `Arc` exists
/// only after construction, so the handler is attached in `load_config`.
fn on_ready(printer: &Arc<Printer>, idle: &Arc<IdleTimeout>) {
    let weak = Arc::downgrade(idle);
    printer.register_event_handler(
        KlippyEvent::KlippyReady,
        Box::new(move |_| {
            if let Some(this) = weak.upgrade() {
                this.handle_ready();
            }
        }),
    );
}

/// Wire the built object into `toolhead:sync_print_time`, its print-start
/// source (`idle_timeout.py:44-45`).
///
/// The payload is forwarded positionally, as Python binds it: the handler's
/// second parameter is the event's `est_print_time` and its third is the
/// event's `print_time`, because its signature names them the other way round
/// (`idle_timeout.py:98` against `toolhead.py:267-268`; see
/// `IdleTimeout::handle_sync_print_time`). The registration's payload values are
/// placeholders: handlers are keyed by event name.
fn on_sync_print_time(printer: &Arc<Printer>, idle: &Arc<IdleTimeout>) {
    let weak = Arc::downgrade(idle);
    printer.register_event_handler(
        KlippyEvent::ToolheadSyncPrintTime {
            curtime: 0.0,
            est_print_time: 0.0,
            print_time: 0.0,
        },
        Box::new(move |event| {
            let KlippyEvent::ToolheadSyncPrintTime {
                curtime,
                est_print_time,
                print_time,
            } = event
            else {
                return;
            };
            if let Some(this) = weak.upgrade() {
                this.handle_sync_print_time(*curtime, *est_print_time, *print_time);
            }
        }),
    );
}

/// Upstream's `load_config` for `[idle_timeout]` (`idle_timeout.py:117-118`).
///
/// # Errors
/// A missing or invalid option, or an idle gcode template that does not
/// compile.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let idle = IdleTimeout::new(config, printer)?;
    on_ready(printer, &idle);
    on_sync_print_time(printer, &idle);
    Ok(idle)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    /// A toolhead the test positions: the MCU reaches `print_time` at
    /// `finished_at` and stands still after that.
    ///
    /// That is the shape `check_busy`'s numbers have on a machine that has
    /// stopped — `est_print_time` catches up to `print_time` and then keeps
    /// going with the clock — without a planner, an MCU or a trapq.
    struct FakeToolhead {
        print_time: Mutex<f64>,
        finished_at: Mutex<f64>,
    }

    impl FakeToolhead {
        fn new(print_time: f64, finished_at: f64) -> Self {
            Self {
                print_time: Mutex::new(print_time),
                finished_at: Mutex::new(finished_at),
            }
        }

        /// Move the planner to `print_time`, finishing it at `finished_at`.
        fn move_to(&self, print_time: f64, finished_at: f64) {
            *self.print_time.lock().unwrap_or_else(|p| p.into_inner()) = print_time;
            *self.finished_at.lock().unwrap_or_else(|p| p.into_inner()) = finished_at;
        }
    }

    impl Toolhead for FakeToolhead {
        fn print_time(&self) -> f64 {
            *self.print_time.lock().unwrap_or_else(|p| p.into_inner())
        }

        fn get_last_move_time(&self) -> f64 {
            self.print_time()
        }

        fn estimated_print_time(&self, eventtime: f64) -> f64 {
            let finished_at = *self.finished_at.lock().unwrap_or_else(|p| p.into_inner());
            self.print_time() + (eventtime - finished_at).max(0.0)
        }
    }

    /// A printer with `text` loaded and the ready lamp lit, as the loader
    /// leaves it, over a clock the test drives.
    fn loaded(text: &str) -> (Arc<Printer>, Arc<ManualReactor>) {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(
            Arc::clone(&reactor) as Arc<dyn crate::core::klippy::reactor::Reactor>
        ));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, reactor)
    }

    /// The object the section registered.
    fn idle(printer: &Arc<Printer>) -> Arc<IdleTimeout> {
        printer
            .lookup_object_as::<IdleTimeout>("idle_timeout")
            .expect("the section registered the object")
    }

    /// `printer.configfile.settings`, the map a client reads its configuration
    /// from.
    fn settings(printer: &Arc<Printer>) -> Value {
        printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("configfile is registered")
            .get_status(0.0)["settings"]
            .clone()
    }

    /// The dispatcher, for running commands.
    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered")
    }

    /// Everything `gcode` reported, one entry per line (`// …` prefixes
    /// included, as a client sees them).
    fn captured_lines(printer: &Arc<Printer>) -> Arc<Mutex<Vec<String>>> {
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        gcode(printer).register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        lines
    }

    fn emitted(lines: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        lines.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// The `idle_timeout` events, in the order they were sent.
    ///
    /// The registry keys handlers by the event's name, so the payloads a
    /// registration is built with do not matter.
    fn captured_events(printer: &Arc<Printer>) -> Arc<Mutex<Vec<KlippyEvent>>> {
        let sent = Arc::new(Mutex::new(Vec::new()));
        for event in [
            KlippyEvent::IdleTimeoutPrinting { print_time: 0.0 },
            KlippyEvent::IdleTimeoutReady { print_time: 0.0 },
            KlippyEvent::IdleTimeoutIdle { print_time: 0.0 },
        ] {
            let sink = Arc::clone(&sent);
            printer.register_event_handler(
                event,
                Box::new(move |event| {
                    sink.lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .push(event.clone())
                }),
            );
        }
        sent
    }

    fn sent_events(sent: &Arc<Mutex<Vec<KlippyEvent>>>) -> Vec<KlippyEvent> {
        sent.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// The `idle_timeout:printing` payloads, in the order they were sent.
    fn printing_times(sent: &Arc<Mutex<Vec<KlippyEvent>>>) -> Vec<f64> {
        sent_events(sent)
            .into_iter()
            .filter_map(|event| match event {
                KlippyEvent::IdleTimeoutPrinting { print_time } => Some(print_time),
                _ => None,
            })
            .collect()
    }

    /// The minimal machine the requirement names: `kinematics: none`, no
    /// extruder, no `[virtual_sdcard]` / `[print_stats]`. `max_velocity` /
    /// `max_accel` are the two options `[printer]` cannot default.
    const MINIMAL_PRINTER: &str =
        "[printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n";

    /// The section's status starts at upstream's initial values
    /// (`idle_timeout.py:24,32-40`).
    #[test]
    fn test_the_status_starts_idle_with_the_default_timeout() {
        let (printer, _reactor) = loaded("[idle_timeout]\n");
        assert_eq!(
            idle(&printer).get_status(0.0),
            json!({ "state": "Idle", "printing_time": 0.0, "idle_timeout": 600.0 })
        );
    }

    /// Upstream loads this module from `toolhead.add_printer_objects`
    /// (`toolhead.py:603-611`), so a config without `[idle_timeout]` still has
    /// the object, its default timeout and the command — what `mainsail.cfg`
    /// reads unconditionally (`printer.idle_timeout.state`,
    /// `printer.configfile.settings.idle_timeout.timeout`).
    ///
    /// The machine is the minimal one the requirement names: no extruder and no
    /// `[virtual_sdcard]` / `[print_stats]` to lean on, `kinematics: none`.
    #[test]
    fn test_the_module_loads_without_a_config_section() {
        let (printer, _reactor) = loaded(MINIMAL_PRINTER);

        // The object exists and reports the defaults a client reads.
        assert_eq!(
            idle(&printer).get_status(0.0),
            json!({ "state": "Idle", "printing_time": 0.0, "idle_timeout": 600.0 })
        );
        // A default that was used is recorded in `settings`, as upstream's is
        // (`klippy/configfile.py:33-36`).
        assert_eq!(settings(&printer)["idle_timeout"]["timeout"], json!(600.0));

        // And the command the same client sends is registered.
        gcode(&printer)
            .run_script_sync("SET_IDLE_TIMEOUT TIMEOUT=30")
            .expect("the command runs without a section");
        assert_eq!(idle(&printer).get_status(0.0)["idle_timeout"], 30.0);

        // The other read `mainsail.cfg` makes (`RESUME`, mainsail.cfg:134) is a
        // template one: `printer.idle_timeout.state` has to resolve in the
        // context a macro body gets, not fail with `undefined value`.
        let mut context = Context::new();
        context.insert(
            "printer",
            Rt::Printer(PrinterView::new(Arc::clone(&printer))),
        );
        let template = Template::parse(
            "idle_state",
            "{% if printer.idle_timeout.state|upper == 'IDLE' %}idle{% endif %}",
        )
        .expect("the template compiles");
        assert_eq!(template.render(&mut context).expect("it renders"), "idle");
    }

    /// A config that names the section still runs with its own timeout, in
    /// `status` and in `settings` alike.
    #[test]
    fn test_a_configured_section_keeps_its_timeout() {
        let (printer, _reactor) =
            loaded(&format!("{MINIMAL_PRINTER}[idle_timeout]\ntimeout: 1200\n"));
        assert_eq!(idle(&printer).get_status(0.0)["idle_timeout"], 1200.0);
        assert_eq!(settings(&printer)["idle_timeout"]["timeout"], json!(1200.0));
    }

    /// `timeout` is read through the loader's bound wording
    /// (`idle_timeout.py:25`).
    #[test]
    fn test_the_timeout_option_is_read_and_zero_is_refused() {
        let (printer, _reactor) = loaded("[idle_timeout]\ntimeout: 360\n");
        assert_eq!(idle(&printer).get_status(0.0)["idle_timeout"], 360.0);

        let (config, _) = Config::from_text("[idle_timeout]\ntimeout: 0\n").expect("parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let error = printer.load_config(&config).expect_err("zero is refused");
        assert_eq!(
            error.to_string(),
            "Option 'timeout' in section 'idle_timeout' must be above 0"
        );
    }

    /// The command reports the new value with upstream's wording and keeps the
    /// old one when no parameter is given (`idle_timeout.py:108-112`).
    #[test]
    fn test_set_idle_timeout_reports_the_value_it_uses() {
        let (printer, _reactor) = loaded("[idle_timeout]\n");
        let lines = captured_lines(&printer);
        let gcode = gcode(&printer);

        // The help text is upstream's, verbatim (`idle_timeout.py:108`).
        assert_eq!(
            gcode
                .command_help()
                .get("SET_IDLE_TIMEOUT")
                .map(String::as_str),
            Some("Set the idle timeout in seconds")
        );

        gcode
            .run_script_sync("SET_IDLE_TIMEOUT TIMEOUT=60")
            .expect("the command runs");
        assert_eq!(emitted(&lines), ["// idle_timeout: Timeout set to 60.00 s"]);
        assert_eq!(idle(&printer).get_status(0.0)["idle_timeout"], 60.0);

        gcode
            .run_script_sync("SET_IDLE_TIMEOUT")
            .expect("the command runs without a parameter");
        assert_eq!(
            emitted(&lines),
            [
                "// idle_timeout: Timeout set to 60.00 s",
                "// idle_timeout: Timeout set to 60.00 s"
            ]
        );
        assert_eq!(idle(&printer).get_status(0.0)["idle_timeout"], 60.0);
    }

    /// `TIMEOUT` has the current value as its default but still has to be above
    /// 0 (`idle_timeout.py:110`).
    #[test]
    fn test_set_idle_timeout_refuses_zero() {
        let (printer, _reactor) = loaded("[idle_timeout]\n");
        let error = gcode(&printer)
            .run_script_sync("SET_IDLE_TIMEOUT TIMEOUT=0")
            .expect_err("zero is refused");
        assert_eq!(
            error.to_string(),
            "Error on 'SET_IDLE_TIMEOUT TIMEOUT=0': TIMEOUT must be above 0"
        );
        assert_eq!(idle(&printer).get_status(0.0)["idle_timeout"], 600.0);
    }

    /// The default idle gcode renders to upstream's text, with and without a
    /// `heaters` object to take the `{% if %}` branch (`idle_timeout.py:8-13`).
    ///
    /// The tag lines keep the newlines around them, as Jinja2 renders them
    /// (`jinja2` of the same template gives `"\n\n   TURN_OFF_HEATERS\n\nM84"`).
    /// The trailing newline is the one difference: this host's template engine
    /// has no `keep_trailing_newline`, so the template's last newline survives.
    /// The script is run line by line, where a trailing empty line is nothing.
    #[test]
    fn test_the_default_idle_gcode_renders() {
        let (printer, _reactor) = loaded("[idle_timeout]\n");
        assert_eq!(idle(&printer).render(&printer).unwrap(), "\n\nM84\n");

        /// An object for `'heaters' in printer` to be true about.
        struct Present;
        impl PrinterObject for Present {
            fn get_status(&self, _eventtime: f64) -> Value {
                json!({})
            }
        }
        printer
            .add_object("heaters", Arc::new(Present))
            .expect("the name is free");
        assert_eq!(
            idle(&printer).render(&printer).unwrap(),
            "\n\n   TURN_OFF_HEATERS\n\nM84\n"
        );
    }

    /// A `gcode` option replaces the default script (`idle_timeout.py:26-28`),
    /// rendered as a template. The value keeps the empty first line its
    /// option line carries — upstream's `configparser` does the same
    /// (`get` returns `'\n…'`) — so the render starts with a newline; the
    /// script runs line by line and an empty line is nothing.
    #[test]
    fn test_a_configured_idle_gcode_is_rendered() {
        let (printer, _reactor) = loaded(
            "[idle_timeout]\ngcode:\n    {% if 'heaters' in printer %}TURN_OFF_HEATERS\n    {% endif %}M84\n",
        );
        assert_eq!(idle(&printer).render(&printer).unwrap(), "\nM84");

        struct Present;
        impl PrinterObject for Present {
            fn get_status(&self, _eventtime: f64) -> Value {
                json!({})
            }
        }
        printer
            .add_object("heaters", Arc::new(Present))
            .expect("the name is free");
        assert_eq!(
            idle(&printer).render(&printer).unwrap(),
            "\nTURN_OFF_HEATERS\nM84"
        );
    }

    /// The first tick waits for the toolhead to settle, as upstream's
    /// `timeout_handler` does, and the transition to `Ready` reports the print
    /// time it is dated with (`idle_timeout.py:92-96`).
    #[test]
    fn test_the_timeout_goes_ready_after_the_toolhead_settles() {
        let (printer, reactor) = loaded("[idle_timeout]\ntimeout: 10\n");
        let sent = captured_events(&printer);
        let object = idle(&printer);
        object.start(Arc::new(FakeToolhead::new(0.0, 0.0)));

        reactor.run_due();
        // `buffer_time` is 0, so upstream waits out `READY_TIMEOUT`.
        assert_eq!(object.get_status(0.0)["state"], "Idle");

        reactor.advance(0.6);
        assert_eq!(object.get_status(0.0)["state"], "Ready");
        // `est_print_time` is 0.5 here, so the payload is that plus
        // `PIN_MIN_TIME`.
        assert_eq!(
            sent_events(&sent),
            [KlippyEvent::IdleTimeoutReady { print_time: 0.600 }]
        );
    }

    /// The timeout expires `timeout` seconds after the machine went idle
    /// (`idle_timeout.py:59-74`), and the `idle` event carries
    /// `get_last_move_time` (`:55-57`).
    #[test]
    fn test_the_timeout_goes_idle_at_the_timeout() {
        let (printer, reactor) = loaded("[idle_timeout]\ntimeout: 10\n");
        let sent = captured_events(&printer);
        let object = idle(&printer);
        // The planner reached 12.5 when the machine came up and has not moved
        // since: the print time the status reports and the idle event carries.
        object.start(Arc::new(FakeToolhead::new(12.5, 0.0)));

        reactor.advance(9.9);
        assert_eq!(object.get_status(0.0)["state"], "Ready");
        reactor.advance(0.05);
        assert_eq!(object.get_status(0.0)["state"], "Ready");

        // `Ready` one tick before the timeout, `Idle` at it.
        reactor.advance(0.10);
        assert_eq!(object.get_status(0.0)["state"], "Idle");
        assert_eq!(
            sent_events(&sent),
            [
                // `est_print_time` is 13.0 at the transition (the MCU has been
                // standing still since 0, and the machine went `Ready` at 0.5).
                KlippyEvent::IdleTimeoutReady { print_time: 13.100 },
                KlippyEvent::IdleTimeoutIdle { print_time: 12.500 },
            ]
        );
    }

    /// A print time that moves puts the machine in `Printing`, and it returns
    /// to `Ready` once the toolhead has stood still (`idle_timeout.py:75-97`).
    #[test]
    fn test_a_moving_print_time_is_printing_until_the_buffer_drains() {
        let (printer, reactor) = loaded("[idle_timeout]\ntimeout: 10\n");
        let sent = captured_events(&printer);
        let object = idle(&printer);
        let toolhead = Arc::new(FakeToolhead::new(0.0, 0.0));
        object.start(Arc::clone(&toolhead) as Arc<dyn Toolhead>);

        reactor.run_due();
        reactor.advance(0.6);
        assert_eq!(object.get_status(0.0)["state"], "Ready");

        // A move: the planner is at 5.0 and the MCU will not reach it until
        // 3.0, so the toolhead is busy.
        toolhead.move_to(5.0, 3.0);
        reactor.advance(0.4);
        assert_eq!(object.get_status(0.0)["state"], "Printing");
        // `printing_time` counts from the tick that saw the move.
        assert_eq!(object.get_status(1.5)["printing_time"], 0.5);
        assert_eq!(
            sent_events(&sent),
            [
                KlippyEvent::IdleTimeoutReady { print_time: 0.600 },
                // The payload is the print time the move re-synced at.
                KlippyEvent::IdleTimeoutPrinting { print_time: 5.100 },
            ]
        );

        // The MCU reached the move at 3.0 and has stood still since: the
        // machine goes back to `Ready` half a second later.
        reactor.advance(2.6);
        assert_eq!(object.get_status(0.0)["state"], "Ready");
    }

    /// The event names the print start once: after a `toolhead:sync_print_time`
    /// has been handled, a print time that moves further must not send
    /// `idle_timeout:printing` a second time, nor re-date the print start
    /// (`Machine::seen_sync` retires the fallback observation).
    #[test]
    fn test_a_seen_sync_event_retires_the_print_time_observer() {
        let (printer, reactor) = loaded("[idle_timeout]\ntimeout: 10\n");
        let sent = captured_events(&printer);
        let object = idle(&printer);
        let toolhead = Arc::new(FakeToolhead::new(0.0, 0.0));
        reactor.advance(1.0);
        object.start(Arc::clone(&toolhead) as Arc<dyn Toolhead>);

        // A print starts: the toolhead raised its print time to 3.25 while the
        // estimate was 3.0 (`toolhead.py:264-268`), reported at 1.0.
        toolhead.move_to(3.25, 3.0);
        printer.send_event(&KlippyEvent::ToolheadSyncPrintTime {
            curtime: 1.0,
            est_print_time: 3.0,
            print_time: 3.25,
        });
        // Upstream's payload is the raised print time plus `PIN_MIN_TIME`
        // (the `est_print_time` of its handler's signature).
        assert_eq!(printing_times(&sent), [3.35]);
        let status = object.get_status(1.0);
        assert_eq!(status["state"], "Printing");
        assert_eq!(status["printing_time"], 0.0);

        // The planner moves on: the tick sees the print time advance, but it is
        // no longer the source and must not report a second start.
        toolhead.move_to(4.0, 5.0);
        reactor.advance(0.5);
        assert_eq!(printing_times(&sent), [3.35]);
        assert_eq!(object.get_status(1.5)["state"], "Printing");
        // `printing_time` counts from the event's `curtime`, not from the tick.
        assert_eq!(object.get_status(1.5)["printing_time"], 0.5);
    }

    /// The handler keeps upstream's argument order, so the timeout it arms is
    /// `curtime + READY_TIMEOUT + est - print` (`idle_timeout.py:104-105` with
    /// the swapped names): 0.25 s after the event here, not the 0.75 s the
    /// names read as.
    #[test]
    fn test_the_sync_handler_arms_the_timeout_with_upstreams_argument_order() {
        let (printer, reactor) = loaded("[idle_timeout]\ntimeout: 10\n");
        let object = idle(&printer);
        // The MCU is ahead of the planner, so the first wake drains the buffer
        // and settles the machine in `Ready`.
        reactor.advance(1.0);
        object.start(Arc::new(FakeToolhead::new(3.25, 0.0)));
        printer.send_event(&KlippyEvent::ToolheadSyncPrintTime {
            curtime: 1.0,
            est_print_time: 3.0,
            print_time: 3.25,
        });
        assert_eq!(object.get_status(1.0)["state"], "Printing");

        reactor.advance(0.2);
        assert_eq!(
            object.get_status(1.2)["state"],
            "Printing",
            "the re-armed timeout is not due before 1.25"
        );
        reactor.advance(0.1);
        assert_eq!(
            object.get_status(1.3)["state"],
            "Ready",
            "the re-armed timeout fired at 1.25 and drained the buffer"
        );
    }

    /// A print start puts the machine in `Printing`, the buffer draining puts it
    /// back in `Ready`, and the next start puts it in `Printing` again — the
    /// timer the event arms runs those transitions, and the tick's own renewal
    /// does not fight it.
    #[test]
    fn test_the_sync_event_round_trips_through_printing_and_ready() {
        let (printer, reactor) = loaded("[idle_timeout]\ntimeout: 10\n");
        let sent = captured_events(&printer);
        let object = idle(&printer);
        // The planner stands at 3.25; the MCU reaches it at 5.0 and its estimate
        // runs on from there, so the buffer drains between 5.0 and 5.5.
        let toolhead = Arc::new(FakeToolhead::new(3.25, 5.0));
        reactor.advance(1.0);
        object.start(Arc::clone(&toolhead) as Arc<dyn Toolhead>);

        printer.send_event(&KlippyEvent::ToolheadSyncPrintTime {
            curtime: 1.0,
            est_print_time: 3.0,
            print_time: 3.25,
        });
        assert_eq!(object.get_status(1.0)["state"], "Printing");

        // The event's timer keeps the machine `Printing` while the buffer is
        // still ahead of the MCU.
        reactor.advance(4.0);
        assert_eq!(object.get_status(5.0)["state"], "Printing");
        reactor.advance(1.0);
        assert_eq!(object.get_status(6.0)["state"], "Ready");

        // A second start, on a machine in `Ready`: `Printing` again, with its
        // own payload and its own re-armed timeout.
        printer.send_event(&KlippyEvent::ToolheadSyncPrintTime {
            curtime: 6.0,
            est_print_time: 6.0,
            print_time: 6.25,
        });
        assert_eq!(object.get_status(6.0)["state"], "Printing");
        assert_eq!(printing_times(&sent), [3.35, 6.35]);
        // The second start's timeout runs the machine back to `Ready` once its
        // buffer is behind the estimate.
        reactor.advance(1.0);
        assert_eq!(object.get_status(7.0)["state"], "Ready");
    }

    /// `SET_IDLE_TIMEOUT` in `Ready` moves the timeout the machine already has
    /// armed (`idle_timeout.py:113-115`).
    #[test]
    fn test_set_idle_timeout_re_arms_the_timeout_in_ready() {
        let (printer, reactor) = loaded("[idle_timeout]\ntimeout: 10\n");
        let object = idle(&printer);
        object.start(Arc::new(FakeToolhead::new(0.0, 0.0)));

        reactor.run_due();
        reactor.advance(0.6);
        assert_eq!(object.get_status(0.0)["state"], "Ready");

        // The command runs at 0.5, so the re-armed timer is due at 3.5.
        gcode(&printer)
            .run_script_sync("SET_IDLE_TIMEOUT TIMEOUT=3")
            .expect("the command runs");
        reactor.advance(2.5);
        assert_eq!(object.get_status(0.0)["state"], "Ready");

        // The re-armed timeout fires at 3.5 s; the value the machine was armed
        // with at `Ready` (10 s) is still in the future.
        reactor.advance(0.6);
        assert_eq!(object.get_status(0.0)["state"], "Idle");
    }
}
