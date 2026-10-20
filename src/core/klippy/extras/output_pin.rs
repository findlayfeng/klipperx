//! `[output_pin <name>]` — a pin a client can set with `SET_PIN`.
//!
//! The first *consumer* of the pin stack: it reads a `pin` description, asks
//! `pins` for a digital output or a PWM (`mcu/resource/pin.rs`, `mcu/resource/pwm.rs`), tells it
//! the start and shutdown values, and registers `SET_PIN PIN=<name> VALUE=<0..1>`
//! with the G-Code dispatcher.
//!
//! Upstream is `klippy/extras/output_pin.py`. This port covers the **output**
//! subset:
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the pin description, required |
//! | `value` | value to drive at startup (default 0) |
//! | `shutdown_value` | value to fall back to on shutdown (default 0) |
//! | `pwm` | use a PWM rather than a plain digital output (default false) |
//! | `scale` | PWM full-scale figure (default 1, `above=0`); `value` / `shutdown_value`
//!   are bounded by it |
//! | `cycle_time` | PWM period in seconds (default 0.1) |
//! | `hardware_pwm` | use the firmware's hardware PWM (default false, software PWM) |
//!
//! `maximum_mcu_duration` is deliberately **not** an option: upstream's
//! `PrinterOutputPin` calls `setup_max_duration(0.)` unconditionally
//! (`klippy/extras/output_pin.py:217`), so the firmware's "return to the
//! shutdown value" limit is off and `value` and `shutdown_value` may differ.
//!
//! `static_value` was a **startup-value** option until upstream removed it
//! (`Config_Changes.md` 20240912) and it is now silently ignored there
//! (`Config_Reference.md` still says "deprecated and should no longer be
//! specified"). This port reads it only so a config migrated from an older
//! klipper still loads, and reports it as deprecated; it drives nothing.
//!
//! # `TEMPLATE=`
//!
//! `SET_PIN PIN=<name> TEMPLATE=<display_template> [PARAM_*=<literal>...]`
//! subscribes the pin to the shared
//! [`template_evaluator`]: the template is rendered every 0.5 s, each result is
//! parsed as a number and driven through `send_async_request` — **immediately**,
//! dated with the *estimated* print time, exactly as upstream's
//! `_template_update` does (`output_pin.py:68-90, 245-250`). A rendered value is
//! **not** divided by `scale` and not bounded by it: upstream divides only the
//! `VALUE=` path (`:255, 265`). A render that does not parse as a number, or a
//! template body that fails, drives `0.` with a logged error.
//!
//! `TEMPLATE=` with an **empty** value cancels the subscription (upstream reads
//! the word's truthiness, `output_pin.py:165-167`), and `VALUE=`/`TEMPLATE=` are
//! mutually exclusive either way around: giving both or neither is
//! `SET_PIN command must specify VALUE or TEMPLATE` (`:256-257`).
//!
//! Two upstream behaviours are reproduced rather than fixed: a `VALUE=` command
//! does **not** cancel an existing `TEMPLATE=` subscription (the timer keeps
//! overwriting it), and a digital output treats the rendered number with this
//! port's `>= 0.5` rule rather than upstream's `bool()`.
//!
//! # Scheduling
//!
//! `SET_PIN` schedules like upstream (`output_pin.py:15-90, 196-269`): the
//! value rides this repo's `GCodeRequestQueue`, dated by a
//! `toolhead.register_lookahead_callback`, and a flush callback registered
//! once on the toolhead drains the queue — the sink (`RequestSink::set_at`,
//! upstream's `_set_pin`) turns each due request into a `queue_digital_out` /
//! `set_pwm` at the pin's MCU clock.
//!
//! Two forks keep the **immediate** path (`update_digital_out` /
//! `update_pwm`), with no error and no panic:
//!
//! * **No `toolhead` object** — this port allows a config without a
//!   `[printer]` section. With nothing to date the change against, `SET_PIN`
//!   drives the pin at once.
//! * **The resource cannot schedule yet** — its MCU is not connected, so the
//!   queue's schedule floor (upstream's `mcu_pin.get_mcu()
//!   .min_schedule_time()`) or the print-time-to-clock mapping does not exist.
//!   A `SET_PIN` before connect then behaves exactly as before (the immediate
//!   path reports "MCU is not connected").
//!
//! The queue arms lazily at the first `SET_PIN` that finds both a toolhead
//! and a schedulable resource, so the section keeps its `order = 20` and
//! never races the toolhead's later load.
//!
//! # What is not here
//!
//! * **`pwm_cycle_time`** is not on the queue because upstream's is not
//!   either (see that module). `output_pin` / `fan` / `servo` / `pwm_tool`
//!   all ride this queue: the schedule state and the lookahead wiring here
//!   (`PinSchedule`, `queue_at_lookahead`) are shared with them — `servo`
//!   keeps its own sink because upstream aligns it to the pulse cycle
//!   (`servo.py:48-56`, `RESCHEDULE_SLACK`).

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::gcode_request_queue::{
    FlushAction, GCodeRequestQueue, RequestSink,
};
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::extras::template_evaluator::{
    lookup_template_eval, PrinterTemplateEvaluator,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{sync, CommandError, CommandHandler, GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuError, McuObject};
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[output_pin <name>]`) exists upstream.
section!("output_pin", order = 20, prefix = load_config_prefix);

/// One configured `[output_pin <name>]`.
///
/// The resource lives in the scheduling state the `SET_PIN` handler drives;
/// this object shares it, so `get_status` reads the value last driven.
pub struct OutputPin {
    /// The scheduling state the `SET_PIN` handler and the request queue share.
    schedule: Arc<PinSchedule>,
}

/// What `SET_PIN` drives: a plain output or a PWM.
///
/// The two have different trait objects but the same `0..=1` client interface,
/// so the section keeps the choice behind one enum and the handlers match on it.
#[derive(Clone)]
enum PinHandle {
    Digital(Arc<dyn DigitalOut>),
    Pwm(Arc<dyn PwmOut>),
}

impl OutputPin {
    /// Build the pin from its section and register `SET_PIN`.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when an option is
    /// missing, unparseable, or asks for something this port does not do yet.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[output_pin <name>]' section"
            ))
        })?;

        let pin_desc = config.get("pin", None)?;

        // Upstream reads `scale` only on the PWM path (`output_pin.py:207-214`);
        // a digital output has an implicit scale of 1. `value` and
        // `shutdown_value` are bounded by `scale` and stored divided by it, so
        // only a PWM accepts values above 1.
        let is_pwm = config.get_bool("pwm", Some(false))?;
        let scale = if is_pwm {
            config.get_float_bounded("scale", Some(1.0), None, None, Some(0.0), None)?
        } else {
            1.0
        };
        let value =
            config.get_float_bounded("value", Some(0.0), Some(0.0), Some(scale), None, None)?
                / scale;
        let shutdown_value = config.get_float_bounded(
            "shutdown_value",
            Some(0.0),
            Some(0.0),
            Some(scale),
            None,
            None,
        )? / scale;

        // `static_value` was removed upstream (`Config_Changes.md` 20240912)
        // and is silently ignored there. This host's undefined-option check
        // would refuse the section, so read it as a deprecated option and drop
        // the value: a config written for an older klipper still loads.
        config.deprecate("static_value", None);
        let _ = config.get_str("static_value");

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        // Upstream disables the firmware's max-duration limit for an
        // `output_pin` unconditionally, which is what lets `value` and
        // `shutdown_value` differ.
        let handle = if is_pwm {
            let pwm = pins
                .setup_pwm(&pin_desc, None)
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
            let cycle_time =
                config.get_float_bounded("cycle_time", Some(0.100), None, None, Some(0.0), None)?;
            let hardware_pwm = config.get_bool("hardware_pwm", Some(false))?;
            pwm.setup_cycle_time(cycle_time, hardware_pwm);
            pwm.setup_max_duration(0.0);
            pwm.setup_start_value(value, shutdown_value);
            PinHandle::Pwm(pwm)
        } else {
            let pin = pins
                .setup_digital_out(&pin_desc, None)
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
            pin.setup_max_duration(0.0);
            pin.setup_start_value(value >= 0.5, shutdown_value >= 0.5);
            PinHandle::Digital(pin)
        };

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let template_eval = lookup_template_eval(printer)?;
        // The `[mcu …]` object the pin's clock lives on, which the template
        // path dates its sends with. Upstream reads it off the pin
        // (`mcu_pin.get_mcu()`); here the description's chip prefix names it,
        // the way `fan` resolves its own. Resolved by parsing the description
        // again — pure, no resource — and looked up lazily, because the MCU
        // object may register after this section.
        let mcu_object = pins
            .parse_pin(&pin_desc, true, true)
            .ok()
            .map(|params| mcu_object_name(&params.chip_name));
        let value_slot = Arc::new(Mutex::new(value));
        let schedule = Arc::new(PinSchedule {
            name: name.clone(),
            value: value_slot,
            handle,
            printer: Arc::downgrade(printer),
            mcu_object,
            armed: Mutex::new(None),
        });
        let handler: CommandHandler = {
            let schedule = Arc::clone(&schedule);
            sync(move |gcmd| cmd_set_pin(&schedule, scale, &template_eval, gcmd))
        };
        gcode
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some(&name),
                handler,
                Some("Set the value of a pin"),
                // `cmd_set_pin` reads the level or the `TEMPLATE` subscription;
                // the mux key `PIN` is prepended by the registration.
                &["VALUE", "TEMPLATE"],
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self { schedule })
    }

    /// The name `SET_PIN` addresses this pin by.
    pub fn name(&self) -> &str {
        &self.schedule.name
    }

    fn lock(&self) -> MutexGuard<'_, f64> {
        self.schedule
            .value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for OutputPin {
    /// The value last driven, as upstream's `PrinterOutputPin.get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "value": *self.lock() })
    }
}

impl std::fmt::Debug for OutputPin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OutputPin")
            .field("name", &self.schedule.name)
            .finish_non_exhaustive()
    }
}

/// The scheduling state behind a pin-like `SET_PIN`: the shared value slot,
/// the pin handle, and the request queue once the first schedulable command
/// arms it.
///
/// Shared by the section object (status), the command handler (pushes) and
/// the queue's sink (drives) — all three see the same last driven value.
/// `output_pin` builds it with either handle; `pwm_tool` with
/// [`PinSchedule::new_pwm`] (its upstream `SET_PIN` rides the same queue per
/// this repo's wiring, `pwm_tool.py:177-184`); `servo` carries its own state
/// because its sink differs.
pub(crate) struct PinSchedule {
    /// The section's sub: names the pin in a send-failure log line.
    name: String,
    /// The value last **driven** (upstream `last_value`, updated when the
    /// change lands: at flush time on the queued path, at once on the
    /// immediate one), for `get_status`.
    value: Arc<Mutex<f64>>,
    /// What `SET_PIN` queues or drives.
    handle: PinHandle,
    /// The printer, so a command can find the toolhead after config load.
    /// Weak: the printer owns the g-code handlers, a strong handle would be a
    /// `printer → objects → gcode → handler → printer` cycle.
    printer: Weak<Printer>,
    /// The `[mcu …]` object the pin's clock lives on, for the template path's
    /// estimated print time; `None` when the description could not be
    /// re-parsed (an alias the pins registry resolves later) or when the
    /// section has no template consumer at all (`pwm_tool`'s
    /// [`Self::new_pwm`]).
    mcu_object: Option<String>,
    /// The queue, built and armed (its flush callback registered with the
    /// toolhead) exactly once — by the first command that finds both a
    /// toolhead and a schedulable resource. See the module docs for the two
    /// fallbacks.
    armed: Mutex<Option<Arc<GCodeRequestQueue<PinSink>>>>,
}

impl PinSchedule {
    /// The scheduling state for a PWM section that shares this queue:
    /// `pwm_tool`'s `SET_PIN` (`pwm_tool.py:177-184` pins it to the lookahead
    /// time just like this module's own command does).
    pub(crate) fn new_pwm(
        name: String,
        pwm: Arc<dyn PwmOut>,
        initial_value: f64,
        printer: &Arc<Printer>,
    ) -> Self {
        Self::with_handle(name, PinHandle::Pwm(pwm), initial_value, printer)
    }

    /// The scheduling state around an already-built handle.
    fn with_handle(
        name: String,
        handle: PinHandle,
        initial_value: f64,
        printer: &Arc<Printer>,
    ) -> Self {
        Self {
            name,
            value: Arc::new(Mutex::new(initial_value)),
            handle,
            printer: Arc::downgrade(printer),
            // `pwm_tool`'s section has no template consumer; `output_pin`
            // fills this in after building its handle.
            mcu_object: None,
            armed: Mutex::new(None),
        }
    }

    /// The section's sub: the name commands address the pin by.
    pub(crate) fn name(&self) -> &str {
        &self.name
    }

    /// The value last **driven** (at once on the immediate path, at its print
    /// time on the queued one) — what `get_status` reports and what a repeat
    /// is compared against on the immediate path.
    pub(crate) fn value(&self) -> f64 {
        *self
            .value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Pin `value` to the toolhead's lookahead time through the shared
    /// request queue (upstream `GCodeRequestQueue.queue_gcode_request`,
    /// `output_pin.py:65-67`), arming the queue on the first call.
    ///
    /// `false` when there is no timeline to date the change against — no
    /// `toolhead` object, or the resource cannot schedule yet
    /// (`min_schedule_time()` is `None`) — so the command keeps its
    /// immediate path, as the module docs describe.
    pub(crate) fn queue(&self, value: f64) -> bool {
        queue_at_lookahead(
            &self.printer,
            &self.armed,
            self.handle.min_schedule_time(),
            self.sink(),
            value,
        )
    }

    /// This schedule's end of the queue: the sink sees the same handle and
    /// value slot the immediate path drives and records.
    fn sink(&self) -> PinSink {
        PinSink {
            name: self.name.clone(),
            handle: self.handle.clone(),
            value: Arc::clone(&self.value),
        }
    }

    /// Send `value` **now**, dated with the estimated print time: upstream's
    /// `_template_update` → `send_async_request` (`output_pin.py:68-90,
    /// 245-250`), what every rendered template lands through.
    ///
    /// Unlike [`Self::queue`] this does not wait for a lookahead callback — a
    /// template render is a timer tick, not a move — so the date is the
    /// estimate `mcu.estimated_print_time(now + min_schedule_time)` upstream
    /// computes when its caller gives none (`:70-72`). The module docs' two
    /// fallbacks apply: with no timeline to date the change against, the value
    /// is driven at once, and a failed drive is logged rather than raised (the
    /// timer callback has no error channel, as the sink's warn notes).
    pub(crate) fn send_async(&self, value: f64) {
        let request = self.arm().and_then(|(_toolhead, queue)| {
            let print_time = self.estimated_print_time()?;
            Some((queue, print_time))
        });
        if let Some((queue, print_time)) = request {
            queue.send_async_request(value, print_time);
            return;
        }
        if let Err(err) = self.drive_now(value) {
            warn!("SET_PIN PIN={}: {err}", self.name);
        }
    }

    /// The queue, arming it on the first request that can use it — the flush
    /// callback is registered **before** the queue is published, so a push can
    /// never sit in a queue nothing drains. `None` means no timeline could date
    /// the change: no `toolhead` object, or `min_schedule_time()` is `None`
    /// (the resource's MCU is not connected).
    fn arm(&self) -> Option<(Arc<ToolHeadObject>, Arc<GCodeRequestQueue<PinSink>>)> {
        arm_queue(
            &self.printer,
            &self.armed,
            self.handle.min_schedule_time(),
            self.sink(),
        )
    }

    /// The date upstream's `send_async_request` computes when none is given
    /// (`output_pin.py:70-72`):
    /// `mcu.estimated_print_time(reactor.monotonic() + min_schedule_time)`.
    /// `None` while the pin's `[mcu …]` object or its schedule floor is not
    /// there yet — no date, so the caller keeps the immediate path.
    fn estimated_print_time(&self) -> Option<f64> {
        let printer = self.printer.upgrade()?;
        let mcu = printer.lookup_object_as::<McuObject>(self.mcu_object.as_deref()?)?;
        let min_schedule_time = self.handle.min_schedule_time()?;
        let now = printer.reactor().monotonic();
        mcu.estimated_print_time(now + min_schedule_time)
    }

    /// The immediate path: drive the pin now and record the value
    /// (`update_digital_out` / `update_pwm`).
    ///
    /// # Errors
    /// Whatever the resource reports — typically "MCU is not connected"
    /// before connect.
    pub(crate) fn drive_now(&self, value: f64) -> Result<(), McuError> {
        self.handle.drive_now(value)?;
        *self
            .value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = value;
        Ok(())
    }
}

/// Pin one command's `value` to the toolhead's lookahead time through `armed`
///'s request queue, building and arming that queue on the first call
/// (upstream `queue_gcode_request`, `output_pin.py:65-67` — the shared wiring
/// behind `output_pin`, `pwm_tool` and `servo`).
///
/// The flush callback is registered **before** the queue is published, so a
/// push can never sit in a queue nothing drains (`register_flush_callback` is
/// connect-safe: before the toolhead connects both wait in its pending lists
/// and are installed together). `false` means no timeline could date the
/// change — no `toolhead` object, or `min_schedule_time()` is `None` (the
/// resource's MCU is not connected) — and the caller keeps its immediate
/// path.
pub(crate) fn queue_at_lookahead<S: RequestSink>(
    printer: &Weak<Printer>,
    armed: &Mutex<Option<Arc<GCodeRequestQueue<S>>>>,
    min_schedule_time: Option<f64>,
    sink: S,
    value: f64,
) -> bool {
    let Some((toolhead, queue)) = arm_queue(printer, armed, min_schedule_time, sink) else {
        return false;
    };
    toolhead.register_lookahead_callback(Box::new(move |print_time| {
        queue.push(print_time, value);
    }));
    true
}

/// The queue's arm step, shared by [`queue_at_lookahead`] and
/// [`PinSchedule::send_async`]: find a timeline, then build (and build the flush
/// callback for) the queue once.
///
/// `None` means no timeline could date the change — no `toolhead` object, or
/// `min_schedule_time()` is `None` (the resource's MCU is not connected) — and
/// the caller keeps its immediate path.
pub(crate) fn arm_queue<S: RequestSink>(
    printer: &Weak<Printer>,
    armed: &Mutex<Option<Arc<GCodeRequestQueue<S>>>>,
    min_schedule_time: Option<f64>,
    sink: S,
) -> Option<(Arc<ToolHeadObject>, Arc<GCodeRequestQueue<S>>)> {
    // The toolhead (`phase = late, order = 60`) loads after these sections
    // (`order = 20`), so it is looked up per command, never at config load.
    let toolhead = printer
        .upgrade()
        .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))?;
    // The resource must be able to schedule: its MCU is connected, so the
    // queue's schedule floor exists.
    let min_schedule_time = min_schedule_time?;
    let queue = {
        let mut armed = armed.lock().unwrap_or_else(|poison| poison.into_inner());
        if let Some(queue) = armed.as_ref() {
            Arc::clone(queue)
        } else {
            let queue = Arc::new(GCodeRequestQueue::new(sink, min_schedule_time));
            let flush_queue = Arc::clone(&queue);
            toolhead.register_flush_callback(Box::new(move |flush_time| {
                flush_queue.flush(flush_time);
            }));
            *armed = Some(Arc::clone(&queue));
            queue
        }
    };
    Some((toolhead, queue))
}

/// The queue's downstream end: where a due request lands at its print time
/// (upstream's `_set_pin`, `output_pin.py:237-244`).
struct PinSink {
    /// Names the pin in a send-failure log line.
    name: String,
    /// What to drive.
    handle: PinHandle,
    /// The last driven value — upstream's `last_value`: a repeat of it is
    /// discarded before a frame is built, and `get_status` reports it.
    value: Arc<Mutex<f64>>,
}

impl RequestSink for PinSink {
    fn set_at(&self, print_time: f64, value: f64) -> Option<(FlushAction, f64)> {
        {
            let mut last = self
                .value
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            if value == *last {
                // Upstream answers "discard", 0. and sends nothing
                // (`output_pin.py:197-198`).
                return Some((FlushAction::Discard, 0.0));
            }
            // Upstream updates `last_value` before driving (`:199`).
            *last = value;
        }
        if let Err(err) = self.handle.drive_at(print_time, value) {
            // The flush callback has no error channel; upstream would raise
            // into the reactor. The frame is lost either way, so say so —
            // quietly dropping it would hide a dead pin.
            warn!("SET_PIN PIN={}: {err}", self.name);
        }
        None
    }
}

impl PinHandle {
    /// The schedule floor the queue spaces its sends by, or `None` while the
    /// resource's MCU is not connected (upstream reads it from
    /// `mcu_pin.get_mcu().min_schedule_time()`).
    fn min_schedule_time(&self) -> Option<f64> {
        match self {
            PinHandle::Digital(pin) => pin.min_schedule_time(),
            PinHandle::Pwm(pin) => pin.min_schedule_time(),
        }
    }

    /// The immediate path: drive without a date (`update_digital_out` /
    /// `update_pwm`). A digital output keeps this port's `>= 0.5` is "on"
    /// semantics.
    fn drive_now(&self, value: f64) -> Result<(), McuError> {
        match self {
            PinHandle::Digital(pin) => pin.update_digital_out(value >= 0.5),
            PinHandle::Pwm(pin) => pin.update_pwm(value),
        }
    }

    /// Land the change at `print_time` on the pin's MCU clock: upstream's
    /// `set_digital` / `set_pwm` (`mcu.py:445-449, 545-553`).
    ///
    /// A resource whose clock is not there yet (MCU not connected) falls back
    /// to the immediate form, which reports the missing connection itself.
    ///
    /// # Errors
    /// Whatever the resource reports: a software PWM's alignment needing a
    /// firmware frequency, or a failed send.
    fn drive_at(&self, print_time: f64, value: f64) -> Result<(), McuError> {
        match self {
            PinHandle::Digital(pin) => match pin.print_time_to_clock(print_time) {
                Some(clock) => pin.queue_digital_out(clock as u32, value >= 0.5),
                None => pin.update_digital_out(value >= 0.5),
            },
            PinHandle::Pwm(pin) => match pin.print_time_to_clock(print_time) {
                Some(clock) => {
                    // A software PWM may only change on a cycle boundary:
                    // `next_aligned_clock` rounds up to one (a hardware PWM
                    // needs none and returns the clock untouched). Landing
                    // early is not wanted, so `allow_early` is 0 — the figure
                    // the immediate path aligns with too.
                    let clock = pin.next_aligned_clock(clock as u32, 0.0)?;
                    pin.set_pwm(clock, value)
                }
                None => pin.update_pwm(value),
            },
        }
    }
}

/// `SET_PIN PIN=<name> VALUE=<0..scale>` or `... TEMPLATE=<display_template>`:
/// set the pin's level, or subscribe it to a template (upstream `cmd_SET_PIN`,
/// `output_pin.py:249-269`).
///
/// `VALUE` is bounded by the pin's `scale` and divided by it first; a digital
/// output treats the result `>= 0.5` as "on", a PWM takes it as a duty. The
/// `TEMPLATE` path does neither — see the module docs. `VALUE` and `TEMPLATE`
/// are mutually exclusive in both directions, and an empty `TEMPLATE=` cancels
/// the subscription.
fn cmd_set_pin(
    schedule: &Arc<PinSchedule>,
    scale: f64,
    template_eval: &Arc<PrinterTemplateEvaluator>,
    gcmd: &crate::core::klippy::gcode::GcodeCommand,
) -> Result<(), CommandError> {
    let value = value_word(gcmd, scale)?;
    let template = gcmd.get_command_parameters().get("TEMPLATE").cloned();
    if value.is_none() == template.is_none() {
        return Err(CommandError::new(
            "SET_PIN command must specify VALUE or TEMPLATE",
        ));
    }
    let Some(value) = value else {
        // `TEMPLATE=` (possibly empty: an unsubscribe, which moves nothing).
        let schedule = Arc::clone(schedule);
        return template_eval.set_template(
            &format!("output_pin {}", schedule.name()),
            gcmd,
            Arc::new(move |text| schedule.send_async(render_value(text))),
        );
    };
    // The client's `0..=scale` figure as the resource's `0..=1` duty.
    let value = value / scale;
    if schedule.queue(value) {
        return Ok(());
    }
    schedule
        .drive_now(value)
        .map_err(|err| CommandError::new(err.to_string()))?;
    Ok(())
}

/// `gcmd.get_float('VALUE', None, minval=0., maxval=scale)`: an absent word is
/// `None`, a present one is parsed and bounded (`output_pin.py:255`).
fn value_word(
    gcmd: &crate::core::klippy::gcode::GcodeCommand,
    scale: f64,
) -> Result<Option<f64>, CommandError> {
    if !gcmd.get_command_parameters().contains_key("VALUE") {
        return Ok(None);
    }
    Ok(Some(gcmd.get_float_range("VALUE", 0.0, scale)?))
}

/// A rendered template's text as a pin level: upstream's `_template_update`
/// parses `float(text)` and falls back to `0.` with a logged error
/// (`output_pin.py:245-250`).
///
/// The value is **not** divided by `scale` and not bounded — upstream has
/// neither check on this path (`:262-265` is the `VALUE` path).
fn render_value(text: &str) -> f64 {
    match text.trim().parse::<f64>() {
        Ok(value) if value.is_finite() => value,
        _ => {
            warn!("output_pin template render error: '{text}' is not a number");
            0.0
        }
    }
}

/// Upstream's `load_config_prefix` for `[output_pin <name>]`.
///
/// The loader registers the object under the section identifier
/// (`output_pin fan`); `SET_PIN` addresses it by the sub (`fan`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(OutputPin::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{PinChip, PinError, PinParams, PwmOut};
    use crate::core::klippy::reactor::ManualReactor;

    /// The clock the fake resources map print time through, in Hz.
    const TEST_CLOCK_HZ: f64 = 1_000_000.0;

    /// The schedule floor the fake resources report (the real one is 0.100).
    const TEST_MIN_SCHEDULE_TIME: f64 = 0.1;

    /// A digital output that records what it was told: clocked changes and
    /// immediate ones separately.
    #[derive(Default)]
    struct FakeDigitalOut {
        max_duration: Mutex<f64>,
        start_value: Mutex<(bool, bool)>,
        updates: Mutex<Vec<bool>>,
        queued: Mutex<Vec<(u32, bool)>>,
        /// Whether the fake models a connected MCU — a clock to convert print
        /// times with and a schedule floor to queue against.
        schedulable: bool,
    }

    impl DigitalOut for FakeDigitalOut {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
            *self.start_value.lock().unwrap() = (start_value, shutdown_value);
        }
        fn queue_digital_out(&self, clock: u32, value: bool) -> Result<(), McuError> {
            self.queued.lock().unwrap().push((clock, value));
            Ok(())
        }
        fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn print_time_to_clock(&self, print_time: f64) -> Option<u64> {
            self.schedulable
                .then_some((print_time * TEST_CLOCK_HZ) as u64)
        }
        fn min_schedule_time(&self) -> Option<f64> {
            self.schedulable.then_some(TEST_MIN_SCHEDULE_TIME)
        }
    }

    /// A PWM that records what it was told: clocked changes (with the clock
    /// they went out at) and immediate ones separately.
    #[derive(Default)]
    struct FakePwm {
        max_duration: Mutex<f64>,
        cycle_time: Mutex<(f64, bool)>,
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
        queued: Mutex<Vec<(u32, f64)>>,
        /// The last duty set, for [`FakePwm::next_aligned_clock`] — `McuPwm`
        /// tracks the same figure to decide whether a duty has a cycle to
        /// land on.
        last_value: Mutex<f64>,
        /// See [`FakeDigitalOut::schedulable`].
        schedulable: bool,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
            *self.cycle_time.lock().unwrap() = (cycle_time, hardware_pwm);
        }
        fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
            *self.start_value.lock().unwrap() = (start_value, shutdown_value);
            *self.last_value.lock().unwrap() = start_value;
        }
        fn set_pwm(&self, clock: u32, value: f64) -> Result<(), McuError> {
            self.queued.lock().unwrap().push((clock, value));
            *self.last_value.lock().unwrap() = value;
            Ok(())
        }
        fn update_pwm(&self, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            *self.last_value.lock().unwrap() = value;
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            // Mirrors `McuPwm`: a hardware PWM needs no alignment, and a duty
            // fully on or off has no cycle to land on.
            let (cycle_time, hardware) = *self.cycle_time.lock().unwrap();
            if hardware {
                return Ok(clock);
            }
            let last_value = *self.last_value.lock().unwrap();
            if last_value == 0.0 || last_value == 1.0 {
                return Ok(clock);
            }
            // Round up to the next cycle boundary (in fake-clock ticks).
            let cycle = (cycle_time * TEST_CLOCK_HZ) as u32;
            if cycle == 0 {
                return Ok(clock);
            }
            Ok((clock + cycle - 1) / cycle * cycle)
        }
        fn print_time_to_clock(&self, print_time: f64) -> Option<u64> {
            self.schedulable
                .then_some((print_time * TEST_CLOCK_HZ) as u64)
        }
        fn min_schedule_time(&self) -> Option<f64> {
            self.schedulable.then_some(TEST_MIN_SCHEDULE_TIME)
        }
    }

    /// A chip that hands out a [`FakeDigitalOut`] or [`FakePwm`] per setup.
    struct FakeChip {
        created: Mutex<Vec<Arc<FakeDigitalOut>>>,
        pwms: Mutex<Vec<Arc<FakePwm>>>,
        /// Whether its resources model a connected MCU (the fixtures' default).
        schedulable: bool,
    }

    impl Default for FakeChip {
        fn default() -> Self {
            Self {
                created: Mutex::new(Vec::new()),
                pwms: Mutex::new(Vec::new()),
                schedulable: true,
            }
        }
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            let out = Arc::new(FakeDigitalOut {
                schedulable: self.schedulable,
                ..Default::default()
            });
            self.created.lock().unwrap().push(Arc::clone(&out));
            Ok(out)
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm {
                schedulable: self.schedulable,
                ..Default::default()
            });
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A ready printer with `gcode` and `pins` over `chip`.
    fn printer_with(chip: FakeChip) -> (Arc<Printer>, Arc<FakeChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(chip);
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, chip)
    }

    /// A ready printer with `gcode` and `pins` over the default fake chip.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>) {
        printer_with(FakeChip::default())
    }

    /// The same printer with a connected `toolhead` registered —
    /// `kinematics: none`, the dwell-only timeline whose flush callbacks must
    /// still run (`toolhead`'s own fixture).
    async fn add_toolhead(printer: &Arc<Printer>) -> Arc<ToolHeadObject> {
        let mut section = ConfigSection::new("printer", None);
        for (key, value) in [
            ("kinematics", "none"),
            ("max_velocity", "300"),
            ("max_accel", "3000"),
        ] {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        let object =
            ToolHeadObject::new(&wrap(&section), printer).expect("kinematics: none builds");
        printer.add_object("toolhead", Arc::new(object)).unwrap();
        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .unwrap();
        object.connect().await.expect("the toolhead connects");
        object
    }

    /// An `[output_pin <name>]` section with `pin: <pin>` plus `options`.
    fn section(name: &str, pin: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("output_pin", Some(name));
        section
            .parameters
            .insert("pin".to_string(), ConfigValue::Single(pin.to_string()));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// Wrap a hand-built section the way the loader does.
    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    fn created(chip: &FakeChip, index: usize) -> Arc<FakeDigitalOut> {
        chip.created.lock().unwrap()[index].clone()
    }

    fn pwm(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    /// The duties a PWM was handed by the immediate path.
    fn pwm_updates(chip: &FakeChip, index: usize) -> Vec<f64> {
        pwm(chip, index).updates.lock().unwrap().clone()
    }

    /// File a `[display_template <name>]` section into the shared registry.
    fn add_display_template(printer: &Arc<Printer>, name: &str, options: &[(&str, &str)]) {
        let mut section = ConfigSection::new("display_template", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        crate::core::klippy::extras::led::load_display_template(&wrap(&section), printer)
            .expect("the template loads");
    }

    /// The shared evaluator a `TEMPLATE=` subscription lands in.
    fn evaluator(printer: &Arc<Printer>) -> Arc<PrinterTemplateEvaluator> {
        crate::core::klippy::extras::template_evaluator::lookup_template_eval(printer)
            .expect("the evaluator is created on first use")
    }

    /// A printer object whose status counts reads, so a template that reads it
    /// renders a different number on every render pass.
    #[derive(Default)]
    struct CountingStatus {
        reads: Mutex<u32>,
    }

    impl PrinterObject for CountingStatus {
        fn get_status(&self, _eventtime: f64) -> Value {
            let mut reads = self
                .reads
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            *reads += 1;
            json!({ "value": *reads })
        }
    }

    #[test]
    fn test_a_digital_output_is_configured_with_its_levels() {
        let (printer, chip) = printer();
        let section = section("fan", "PA1", &[("value", "1"), ("shutdown_value", "0")]);

        OutputPin::new(&wrap(&section), &printer).unwrap();

        let out = created(&chip, 0);
        assert_eq!(*out.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*out.start_value.lock().unwrap(), (true, false));
    }

    #[test]
    fn test_set_pin_drives_the_output() {
        let (printer, chip) = printer();
        let pin = OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan VALUE=1")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [true]);
        assert_eq!(pin.get_status(0.0)["value"], 1.0);

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan VALUE=0")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [true, false]);
        assert_eq!(pin.get_status(0.0)["value"], 0.0);
    }

    #[test]
    fn test_set_pin_treats_a_half_as_on() {
        let (printer, chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan VALUE=0.5")
            .unwrap();

        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [true]);
    }

    #[test]
    fn test_set_pin_requires_exactly_one_of_value_and_template() {
        let (printer, _chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();
        add_display_template(&printer, "hot", &[("text", "0.5")]);

        // Neither word, and both words: upstream answers the same error
        // (`output_pin.py:256-257`).
        for line in [
            "SET_PIN PIN=fan",
            "SET_PIN PIN=fan VALUE=1 TEMPLATE=hot",
            "SET_PIN PIN=fan TEMPLATE=hot VALUE=1",
        ] {
            let err = gcode(&printer).run_script_sync(line).unwrap_err();
            assert_eq!(
                err.to_string(),
                "SET_PIN command must specify VALUE or TEMPLATE",
                "{line}"
            );
        }
    }

    #[test]
    fn test_a_pin_template_must_name_a_declared_template_and_declared_params() {
        let (printer, _chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();
        add_display_template(
            &printer,
            "hot",
            &[("param_myvar", "0.25"), ("text", "{param_myvar}")],
        );

        // The three refusals upstream's `set_template` makes, with its wording
        // (`output_pin.py:168-182`).
        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=nosuch")
            .unwrap_err();
        assert_eq!(err.to_string(), "Unknown display_template 'nosuch'");

        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=hot PARAM_NOPE=1")
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Invalid display_template parameter: param_nope"
        );

        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=hot PARAM_MYVAR=abc")
            .unwrap_err();
        assert_eq!(err.to_string(), "Unable to parse 'abc' as a literal");
    }

    #[test]
    fn test_a_pin_template_re_renders_and_lands_on_the_pin() {
        let (printer, chip) = printer();
        // A template that reads a counter, so two render passes differ.
        add_display_template(&printer, "hot", &[("text", "{printer.test_counter.value}")]);
        printer
            .add_object(
                "test_counter",
                Arc::new(CountingStatus::default()) as Arc<dyn PrinterObject>,
            )
            .unwrap();
        OutputPin::new(&wrap(&section("fan", "PA1", &[("pwm", "true")])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=hot")
            .unwrap();

        // One timer period's worth of rendering, twice: each render hands its
        // text to the pin, which parses it and drives it.
        let eval = evaluator(&printer);
        assert_eq!(eval.render_once(0.0), Some(0.5));
        assert_eq!(pwm_updates(&chip, 0), [1.0]);
        assert_eq!(eval.render_once(0.5), Some(1.0));
        assert_eq!(pwm_updates(&chip, 0), [1.0, 2.0]);
    }

    #[test]
    fn test_a_command_parameter_overrides_the_template_default() {
        let (printer, chip) = printer();
        add_display_template(
            &printer,
            "hot",
            &[("param_myvar", "0.25"), ("text", "{param_myvar}")],
        );
        OutputPin::new(&wrap(&section("fan", "PA1", &[("pwm", "true")])), &printer).unwrap();
        let eval = evaluator(&printer);

        // The section's own `param_*` value is the default...
        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=hot")
            .unwrap();
        assert_eq!(eval.render_once(0.0), Some(0.5));
        assert_eq!(pwm_updates(&chip, 0), [0.25]);

        // ...and `PARAM_MYVAR=` is bound over it, as upstream's `params.update`
        // does (`display/display.py:44-46`).
        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=hot PARAM_MYVAR=0.75")
            .unwrap();
        assert_eq!(eval.render_once(0.5), Some(1.0));
        assert_eq!(pwm_updates(&chip, 0), [0.25, 0.75]);
    }

    #[test]
    fn test_an_empty_template_argument_unsubscribes() {
        let (printer, chip) = printer();
        add_display_template(&printer, "half", &[("text", "0.5")]);
        OutputPin::new(&wrap(&section("fan", "PA1", &[("pwm", "true")])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=half")
            .unwrap();
        let eval = evaluator(&printer);
        assert_eq!(eval.render_once(0.0), Some(0.5));
        assert_eq!(pwm_updates(&chip, 0), [0.5]);

        // `TEMPLATE=` cancels the subscription, and with nothing left the timer
        // retires itself (`output_pin.py:165-167`, `:131-136`).
        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=")
            .unwrap();
        assert_eq!(eval.render_once(0.5), None);
        assert_eq!(pwm_updates(&chip, 0), [0.5], "nothing more was driven");
    }

    #[test]
    fn test_a_failed_render_and_a_non_numeric_one_drive_zero() {
        let (printer, chip) = printer();
        // `{printer.no.such}` is a strict-undefined failure; `"ready"` renders
        // fine but is not a number. Both end up at `0.` with a logged error
        // (`output_pin.py:148-152, 245-250`).
        add_display_template(&printer, "broken", &[("text", "{printer.no.such}")]);
        add_display_template(&printer, "word", &[("text", "ready")]);
        OutputPin::new(&wrap(&section("fan", "PA1", &[("pwm", "true")])), &printer).unwrap();
        let eval = evaluator(&printer);

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=broken")
            .unwrap();
        assert_eq!(eval.render_once(0.0), Some(0.5));
        assert_eq!(pwm_updates(&chip, 0), [0.0]);

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan TEMPLATE=word")
            .unwrap();
        assert_eq!(eval.render_once(0.5), Some(1.0));
        assert_eq!(pwm_updates(&chip, 0), [0.0, 0.0]);
    }

    #[test]
    fn test_two_pins_are_driven_independently() {
        let (printer, chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();
        OutputPin::new(&wrap(&section("light", "PA2", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=light VALUE=1")
            .unwrap();

        assert!(created(&chip, 0).updates.lock().unwrap().is_empty());
        assert_eq!(*created(&chip, 1).updates.lock().unwrap(), [true]);
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (printer, _chip) = printer();
        let section = ConfigSection::new("output_pin", Some("fan"));

        let err = OutputPin::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'output_pin fan' must be specified"
        );
    }

    #[test]
    fn test_an_unparseable_value_is_reported() {
        let (printer, _chip) = printer();

        let err = OutputPin::new(&wrap(&section("fan", "PA1", &[("value", "abc")])), &printer)
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'value' in section 'output_pin fan'"
        );
    }

    #[test]
    fn test_a_pwm_output_is_configured_and_driven() {
        let (printer, chip) = printer();
        let section = section(
            "fan",
            "PA1",
            &[("pwm", "true"), ("cycle_time", "0.05"), ("value", "0.5")],
        );

        OutputPin::new(&wrap(&section), &printer).unwrap();

        let pwm = chip.pwms.lock().unwrap()[0].clone();
        assert_eq!(*pwm.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*pwm.cycle_time.lock().unwrap(), (0.05, false));
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.5, 0.0));

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan VALUE=0.25")
            .unwrap();
        assert_eq!(*pwm.updates.lock().unwrap(), [0.25]);
    }

    #[test]
    fn test_a_hardware_pwm_is_selected_by_its_option() {
        let (printer, chip) = printer();
        let section = section("fan", "PA1", &[("pwm", "true"), ("hardware_pwm", "true")]);

        OutputPin::new(&wrap(&section), &printer).unwrap();

        assert_eq!(
            *chip.pwms.lock().unwrap()[0].cycle_time.lock().unwrap(),
            (0.1, true)
        );
    }

    #[test]
    fn test_a_pwm_scale_raises_the_value_bound_and_divides_it() {
        let (printer, chip) = printer();
        let section = section(
            "current",
            "PA1",
            &[
                ("pwm", "true"),
                ("scale", "2.0"),
                ("value", "1.3"),
                ("shutdown_value", "0.4"),
            ],
        );

        let pin = OutputPin::new(&wrap(&section), &printer).unwrap();

        let pwm = chip.pwms.lock().unwrap()[0].clone();
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.65, 0.2));

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=current VALUE=1.0")
            .unwrap();
        assert_eq!(*pwm.updates.lock().unwrap(), [0.5]);
        assert_eq!(pin.get_status(0.0)["value"], 0.5);
    }

    #[test]
    fn test_a_digital_output_value_is_still_bounded_by_one() {
        let (printer, _chip) = printer();
        let section = section("fan", "PA1", &[("value", "1.3")]);

        let err = OutputPin::new(&wrap(&section), &printer).unwrap_err();

        assert!(err.to_string().contains("must have maximum of 1"), "{err}");
    }

    #[test]
    fn test_set_pin_is_bounded_by_the_scale() {
        let (printer, _chip) = printer();
        let section = section("current", "PA1", &[("pwm", "true"), ("scale", "2.0")]);
        OutputPin::new(&wrap(&section), &printer).unwrap();

        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=current VALUE=3")
            .unwrap_err();

        assert!(err.to_string().contains("maximum"), "{err}");
    }

    #[test]
    fn test_a_non_positive_cycle_time_is_reported() {
        let (printer, _chip) = printer();
        let section = section("fan", "PA1", &[("pwm", "true"), ("cycle_time", "0")]);

        let err = OutputPin::new(&wrap(&section), &printer).unwrap_err();

        assert!(err.to_string().contains("cycle_time"), "{err}");
        assert!(err.to_string().contains("above 0"), "{err}");
    }

    #[test]
    fn test_an_unparseable_boolean_is_reported() {
        let (printer, _chip) = printer();

        let err = OutputPin::new(&wrap(&section("fan", "PA1", &[("pwm", "maybe")])), &printer)
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'pwm' in section 'output_pin fan'"
        );
    }

    // -----------------------------------------------------------------------
    // The queued (print-time) path
    // -----------------------------------------------------------------------

    /// A queued `SET_PIN` lands as a **clocked** `queue_digital_out` dated by
    /// the toolhead's print time — never the immediate `update_digital_out` —
    /// and only then does the status report the new value (upstream
    /// `last_value` moves when the change lands).
    #[tokio::test]
    async fn test_a_queued_set_pin_sends_a_clocked_change() {
        let (printer, chip) = printer();
        let toolhead = add_toolhead(&printer).await;
        let pin = OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=1")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let out = created(&chip, 0);
        let print_time = toolhead.print_time();
        assert_eq!(
            *out.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, true)],
            "the frame carries the print time as a clock"
        );
        assert!(
            out.updates.lock().unwrap().is_empty(),
            "the immediate update_digital_out path is not taken"
        );
        assert_eq!(pin.get_status(0.0)["value"], 1.0);
    }

    /// Neighbouring changes are clocked exactly their print-time gap apart:
    /// a `dwell` of 0.25 s between two `SET_PIN`s shows up as a 0.25 s clock
    /// gap (0.25 × `TEST_CLOCK_HZ` ticks) on the wire.
    #[tokio::test]
    async fn test_the_clock_gap_between_two_changes_is_their_print_time_gap() {
        let (printer, chip) = printer();
        let toolhead = add_toolhead(&printer).await;
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=1")
            .await
            .unwrap();
        let first_time = toolhead.print_time();
        toolhead.flush_step_generation().await.unwrap();

        toolhead.dwell(0.25);
        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0")
            .await
            .unwrap();
        let second_time = toolhead.print_time();
        toolhead.flush_step_generation().await.unwrap();

        assert_eq!(
            second_time - first_time,
            0.25,
            "dwell advanced the timeline"
        );
        let out = created(&chip, 0);
        let queued = out.queued.lock().unwrap().clone();
        assert_eq!(queued.len(), 2, "{queued:?}");
        assert!(queued[0].1, "first change is on");
        assert!(!queued[1].1, "second change is off");
        let clock_gap = f64::from(queued[1].0) - f64::from(queued[0].0);
        let print_time_gap = (second_time - first_time) * TEST_CLOCK_HZ;
        assert!(
            (clock_gap - print_time_gap).abs() < 1e-6,
            "clock gap {clock_gap} vs print-time gap {print_time_gap}"
        );
        assert!(out.updates.lock().unwrap().is_empty());
    }

    /// Repeating the value that is already driven is discarded by the sink:
    /// the later request flushes (it is past the schedule floor) and sends
    /// nothing — upstream's `"discard"` (`output_pin.py:197-198`).
    #[tokio::test]
    async fn test_repeating_the_driven_value_sends_no_second_frame() {
        let (printer, chip) = printer();
        let toolhead = add_toolhead(&printer).await;
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=1")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();
        assert_eq!(created(&chip, 0).queued.lock().unwrap().len(), 1);

        // A later request for the same value — past the schedule floor of the
        // first send, so it reaches the sink and is discarded there.
        toolhead.dwell(0.25);
        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=1")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        assert_eq!(
            created(&chip, 0).queued.lock().unwrap().len(),
            1,
            "the repeat sent no second frame"
        );
        assert!(created(&chip, 0).updates.lock().unwrap().is_empty());
    }

    /// No `toolhead` object (a config without `[printer]`), even though the
    /// resource could schedule: `SET_PIN` keeps the immediate path — driven at
    /// once, nothing queued.
    #[test]
    fn test_without_a_toolhead_the_pin_is_still_set_immediately() {
        let (printer, chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=fan VALUE=1")
            .unwrap();

        let out = created(&chip, 0);
        assert_eq!(*out.updates.lock().unwrap(), [true]);
        assert!(out.queued.lock().unwrap().is_empty());
    }

    /// The second fork: a toolhead, but a resource whose MCU is not connected
    /// (no schedule floor to queue with) — immediate path, no error, no panic.
    #[tokio::test]
    async fn test_a_resource_that_cannot_schedule_is_set_immediately() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: false,
            ..FakeChip::default()
        });
        add_toolhead(&printer).await;
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=1")
            .await
            .unwrap();

        let out = created(&chip, 0);
        assert_eq!(*out.updates.lock().unwrap(), [true]);
        assert!(out.queued.lock().unwrap().is_empty());
    }

    /// The digital `0.5` is "on" on the queued path too: the level decision
    /// (`>= 0.5`) is made where the frame is built, unchanged.
    #[tokio::test]
    async fn test_a_queued_half_value_lands_as_on() {
        let (printer, chip) = printer();
        let toolhead = add_toolhead(&printer).await;
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0.5")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let queued = created(&chip, 0).queued.lock().unwrap().clone();
        assert_eq!(queued.len(), 1, "{queued:?}");
        assert!(queued[0].1, "0.5 queues as on");
        assert!(created(&chip, 0).updates.lock().unwrap().is_empty());
    }

    /// The queued PWM path: `set_pwm` at the converted clock, with a software
    /// PWM rounded up to its cycle boundary once the duty is mid-cycle (a
    /// change from a fully off duty is not aligned — there is no cycle to
    /// land on yet, as `McuPwm` itself decides).
    #[tokio::test]
    async fn test_a_queued_pwm_change_is_aligned_to_its_cycle() {
        let (printer, chip) = printer();
        let toolhead = add_toolhead(&printer).await;
        OutputPin::new(
            &wrap(&section(
                "fan",
                "PA1",
                &[("pwm", "true"), ("cycle_time", "0.1")],
            )),
            &printer,
        )
        .unwrap();

        // From the startup duty 0: no alignment yet.
        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0.25")
            .await
            .unwrap();
        let first_print_time = toolhead.print_time();
        toolhead.flush_step_generation().await.unwrap();

        // Now the duty is mid-cycle: the next change rounds up to the next
        // 0.1 s boundary of the fake's 1 MHz clock.
        toolhead.dwell(0.125);
        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0.5")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = chip.pwms.lock().unwrap()[0].clone();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [
                ((first_print_time * TEST_CLOCK_HZ) as u64 as u32, 0.25),
                (400_000, 0.5),
            ],
            "unaligned at the start, aligned to the cycle afterwards"
        );
        assert!(
            pwm.updates.lock().unwrap().is_empty(),
            "the immediate update_pwm path is not taken"
        );
    }
}
