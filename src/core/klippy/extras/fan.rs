//! `[fan]` — the printer cooling fan, driven by `M106`/`M107`.
//!
//! Upstream's `klippy/extras/fan.py`: it reads a `pin`, asks `pins` for a PWM,
//! and exposes the fan through `M106 S<0..255>` / `M107`. Half the file is the
//! **shared `Fan` core** — `fan_generic` (H2-2), `heater_fan` (H2-3) and
//! `controller_fan` (H2-4) build the same object and choose the speed some
//! other way — so the core lives here, next to the section that first needs it.
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the fan's PWM pin, required |
//! | `max_power` | duty ceiling, `0 < .. ≤ 1` (default 1) |
//! | `kick_start_time` | seconds at full power when starting from rest, or when stepping up by more than 0.5 (default 0.1, `≥ 0`) |
//! | `off_below` | a request under this runs the fan off, `0 ..= 1` (default 0) |
//! | `cycle_time` | PWM period in seconds (default 0.010, `> 0`) |
//! | `hardware_pwm` | the firmware's PWM rather than a software one (default false) |
//! | `shutdown_speed` | duty the firmware falls back to when klippy dies (default 0, `0 ..= 1`), capped by `max_power` |
//! | `enable_pin` | optional digital output powering the driver, flipped only on 0 ↔ non-zero |
//! | `tachometer_pin` | optional GPIO the tachometer counts edges on (`tachometer_ppr`, default 2; `tachometer_poll_interval`, default 0.0015) |
//!
//! `M106`'s `S` defaults to 255 and has no upper bound (only `minval=0.`), as
//! upstream's does; `max_power` is what caps the duty that results.
//!
//! # Print-time scheduling
//!
//! Every speed change rides this port's
//! [`GCodeRequestQueue`](crate::core::klippy::extras::gcode_request_queue)
//! (the helper `output_pin.py:15-90` ports), with the same wiring
//! [`output_pin`](crate::core::klippy::extras::output_pin) uses. Upstream's
//! split is kept one for one:
//!
//! * `set_speed_from_command` — `M106`/`M107`, `SET_FAN_SPEED` — is upstream's
//!   `queue_gcode_request` (`output_pin.py:61-66`): the request is dated by
//!   `toolhead.register_lookahead_callback` and pushed, so a speed lands at a
//!   print time, later requests override earlier ones, and the kick-start
//!   tail is the queued request **re-run** at `print_time + kick_start_time`
//!   (the queue's `"repeat"`, `fan.py:60-66`) — not a reactor timer.
//! * `set_speed` — the reactor-timer callers (`heater_fan`, `controller_fan`,
//!   `temperature_fan`) and the `gcode:request_restart` handler — is upstream's
//!   `send_async_request` (`fan.py:69-70`): the sink runs right away at
//!   `max(print_time, next_min_flush_time)`, dated by the pin MCU's print-time
//!   estimate or the restart event's own time (`output_pin.py:68-72`).
//!
//! The queue arms lazily at the first request that finds both a toolhead and
//! a schedulable pin, so the section keeps its `order = 20` and never races
//! the toolhead's later load. Two forks keep the **immediate** path
//! (`update_pwm` / `update_digital_out`), with no error and no panic:
//!
//! * **No `toolhead` object** — a config without `[printer]` has nothing to
//!   date a change against, so the pin is driven at once.
//! * **The request cannot be dated** — `min_schedule_time()` answers `None`
//!   (the pin's MCU is not connected yet), or the pin's `[mcu …]` object has
//!   no print-time estimate. A request before connect then behaves exactly as
//!   before: the immediate write reports "MCU is not connected" itself.
//!
//! # What is not here
//!
//! * **The kick-start tail on the immediate path**: with no queue to re-run
//!   the request, that fallback still gives the tail a reactor timer
//!   (`call_later`) and drops it when a newer request supersedes it
//!   (`FanState::kick_serial`) — the stand-in the queued path does not need.
//!
//! The tachometer is not one of the gaps: `tachometer_pin` builds a
//! [`pulse_counter`](crate::core::klippy::extras::pulse_counter) frequency
//! counter, and `get_status` reports `rpm` from it exactly as upstream's
//! `FanTachometer` does — `null` for a section that has no tachometer pin.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_request_queue::{
    FlushAction, GCodeRequestQueue, RequestSink,
};
use crate::core::klippy::extras::pulse_counter::FrequencyCounter;
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuError, McuObject};
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::Reactor;

// Only the bare form (`[fan]`) exists upstream.
section!("fan", order = 20, load = load_config);

/// What the fan is driving, and what was asked of it.
///
/// Upstream's `last_fan_value` / `last_req_value`: the first is the duty on the
/// pin (which during a kick start is the full power), the second is the duty the
/// last request wanted — the one `get_status` reports as `speed`.
#[derive(Debug, Default)]
struct FanState {
    last_fan_value: f64,
    last_req_value: f64,
    /// Generation counter for a kick start still in flight **on the
    /// immediate path** (module docs).
    ///
    /// Every request that actually changes the fan bumps it, so the tail of an
    /// older kick start finds a stale serial and does nothing — the effect the
    /// queued path gets from a later request overriding the pending one.
    kick_serial: u64,
}

/// Seconds between two tachometer samples (upstream's fixed `sample_time`).
const TACHOMETER_SAMPLE_TIME: f64 = 1.;

/// Upstream's `FanTachometer` (`fan.py:85-106`): the optional pulse counter
/// behind `tachometer_pin`, and the RPM its frequency becomes.
struct FanTachometer {
    /// Pulses per revolution (`tachometer_ppr`), upstream's `self.ppr`.
    ppr: f64,
    /// The frequency counter, when the section has a `tachometer_pin`.
    counter: Option<FrequencyCounter>,
}

impl FanTachometer {
    /// Read the tachometer options and build the counter (`FanTachometer.__init__`).
    ///
    /// # Errors
    /// `tachometer_ppr` below 1, `tachometer_poll_interval` at or below 0, or
    /// any complaint [`FrequencyCounter::new`] makes about the pin.
    fn new(
        config: &ConfigWrapper,
        identifier: &str,
        pins: &PrinterPins,
    ) -> Result<Self, ConfigError> {
        let Some(pin) = config.get_str("tachometer_pin") else {
            // No tachometer pin: upstream keeps the counter at `None` and
            // reports no RPM for this section.
            return Ok(Self {
                ppr: 2.,
                counter: None,
            });
        };
        let ppr = config.get_int_bounded("tachometer_ppr", Some(2), Some(1), None)?;
        let poll_time = config.get_float_bounded(
            "tachometer_poll_interval",
            Some(0.0015),
            None,
            None,
            Some(0.),
            None,
        )?;
        let counter = FrequencyCounter::new(pins, &pin, TACHOMETER_SAMPLE_TIME, poll_time)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        Ok(Self {
            ppr: ppr as f64,
            counter: Some(counter),
        })
    }

    /// Upstream's `FanTachometer.get_status`: no tachometer reads as `null`,
    /// one reads as the frequency scaled into RPM.
    fn rpm(&self) -> Value {
        match &self.counter {
            Some(counter) => json!(to_rpm(counter.get_frequency(), self.ppr)),
            None => Value::Null,
        }
    }
}

/// Upstream's `rpm = self._freq_counter.get_frequency() * 30. / self.ppr`
/// (`fan.py:98`): the frequency of a `tachometer_ppr`-pulse train in RPM.
fn to_rpm(frequency: f64, ppr: f64) -> f64 {
    frequency * 30. / ppr
}

/// What drives the fan's pins: the option set, the resources, and the
/// evaluation one request goes through (upstream's `Fan._apply_speed`,
/// `fan.py:49-68`).
///
/// The immediate fallback and the queue's [`FanSink`] both drive through
/// this — the split `output_pin` keeps between `PinSchedule` and `PinSink`.
struct FanDrive {
    reactor: Arc<dyn Reactor>,
    max_power: f64,
    kick_start_time: f64,
    off_below: f64,
    mcu_fan: Arc<dyn PwmOut>,
    enable_pin: Option<Arc<dyn DigitalOut>>,
    state: Arc<Mutex<FanState>>,
    /// Names the section in a send-failure log line.
    identifier: String,
}

impl FanDrive {
    fn lock(&self) -> MutexGuard<'_, FanState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Drive the pin for one request (upstream's `Fan._apply_speed`,
    /// `fan.py:49-68`), whose `print_time` arrives as `Some` on the queued
    /// path — land the change at that print time — and as `None` on the
    /// immediate fallback (drive now).
    ///
    /// `off_below` snaps small requests off, `max_power` caps them, an
    /// unchanged request is discarded, the enable line only moves on a 0 ↔
    /// non-zero transition, and a start from rest (or a step up of more than
    /// 0.5) runs at full power for `kick_start_time` first.
    ///
    /// The return is what the queue is told to do next:
    /// [`FlushAction::Discard`] when the request changes nothing,
    /// [`FlushAction::Repeat`] when the kick-start tail is this request
    /// re-run at `print_time + kick_start_time` (`fan.py:66`). The immediate
    /// path has no queue, ignores the action, and gives the tail a reactor
    /// timer instead (module docs).
    ///
    /// # Errors
    /// A failed PWM or enable-pin write.
    fn apply(
        &self,
        print_time: Option<f64>,
        requested: f64,
    ) -> Result<Option<(FlushAction, f64)>, McuError> {
        let mut state = self.lock();
        let requested = if requested < self.off_below {
            0.0
        } else {
            requested
        };
        let value = (requested * self.max_power).clamp(0.0, self.max_power);

        if value == state.last_fan_value {
            // Same as what is already driven: upstream answers "discard", 0.
            // (`fan.py:53-54`), and the queue sends nothing — including while
            // a kick start is still in flight, which keeps its tail and lands
            // the value then.
            return Ok(Some((FlushAction::Discard, 0.0)));
        }

        // This request supersedes a kick start whose tail has not run yet.
        state.kick_serial = state.kick_serial.wrapping_add(1);
        let serial = state.kick_serial;

        if let Some(pin) = &self.enable_pin {
            if value > 0.0 && state.last_fan_value == 0.0 {
                self.drive_enable(print_time, pin, true)?;
            } else if value == 0.0 && state.last_fan_value > 0.0 {
                self.drive_enable(print_time, pin, false)?;
            }
        }

        if value > 0.0
            && self.kick_start_time > 0.0
            && (state.last_fan_value == 0.0 || value - state.last_fan_value > 0.5)
        {
            // Full power now, the requested duty after `kick_start_time`.
            state.last_req_value = value;
            state.last_fan_value = self.max_power;
            self.drive_pwm(print_time, self.max_power)?;
            if let Some(print_time) = print_time {
                // The queued path: the tail is this very request re-run by
                // the queue at `print_time + kick_start_time` — upstream's
                // "repeat" (`fan.py:60-66`).
                return Ok(Some((
                    FlushAction::Repeat,
                    print_time + self.kick_start_time,
                )));
            }
            // The immediate fallback has no queue to re-run it: a reactor
            // timer stands in for the queue slot (module docs).
            let tail = KickTail {
                mcu_fan: Arc::clone(&self.mcu_fan),
                state: Arc::clone(&self.state),
            };
            let delay = self.kick_start_time;
            // Not holding the lock while registering: a dispatcher on another
            // thread could run the timer and wait on `state`.
            drop(state);
            self.reactor
                .call_later(delay, Box::new(move |_| tail.run(serial, value)));
            return Ok(None);
        }

        state.last_fan_value = value;
        state.last_req_value = value;
        self.drive_pwm(print_time, value)?;
        Ok(None)
    }

    /// Land the duty at `print_time` on the pin's MCU clock (upstream's
    /// `mcu_fan.set_pwm(print_time, value)`), or drive it now when the
    /// change carries no date. A clock the resource cannot produce yet (MCU
    /// not connected) falls back to `update_pwm`, which reports the missing
    /// connection itself — `output_pin`'s sink does the same.
    fn drive_pwm(&self, print_time: Option<f64>, value: f64) -> Result<(), McuError> {
        let Some(print_time) = print_time else {
            return self.mcu_fan.update_pwm(value);
        };
        let Some(clock) = self.mcu_fan.print_time_to_clock(print_time) else {
            return self.mcu_fan.update_pwm(value);
        };
        // A software PWM may only change on a cycle boundary; landing early is
        // not wanted, so `allow_early` is 0 — how `output_pin` aligns its
        // queued PWM changes too.
        let clock = self.mcu_fan.next_aligned_clock(clock as u32, 0.0)?;
        self.mcu_fan.set_pwm(clock, value)
    }

    /// Flip the enable line at `print_time` (upstream's
    /// `enable_pin.set_digital(print_time, …)`, `fan.py:56-59`) or now — the
    /// same rule as [`Self::drive_pwm`].
    fn drive_enable(
        &self,
        print_time: Option<f64>,
        pin: &Arc<dyn DigitalOut>,
        on: bool,
    ) -> Result<(), McuError> {
        match print_time.and_then(|print_time| pin.print_time_to_clock(print_time)) {
            Some(clock) => pin.queue_digital_out(clock as u32, on),
            None => pin.update_digital_out(on),
        }
    }
}

/// The fan core: one PWM pin, its optional enable line, and how to drive them.
///
/// Shared with everything that chooses a speed: the `[fan]` object and its
/// `M106`/`M107` handlers, the `gcode:request_restart` handler, and the
/// `fan_generic` / `heater_fan` / `controller_fan` / `temperature_fan`
/// sections that build this core and pick the speed another way. Requests are
/// dated through the module docs' queue wiring; `Fan` holds the queue once a
/// request arms it.
pub struct Fan {
    /// The pins, the option set and the request evaluation the queue's sink
    /// and the immediate path share.
    drive: Arc<FanDrive>,
    /// The optional tachometer behind `tachometer_pin`.
    tachometer: FanTachometer,
    /// The printer, so a command can find the toolhead and the pin's MCU
    /// after config load. Weak: the printer owns the g-code handlers, a
    /// strong handle would be a `printer → objects → gcode → handler →
    /// printer` cycle (as in `output_pin`).
    printer: Weak<Printer>,
    /// The `[mcu …]` object the pin belongs to — upstream reaches its timing
    /// through `mcu_fan.get_mcu()`, which `PwmOut` does not carry, so the pin
    /// description's chip prefix names it (the split `bltouch`'s control-pin
    /// lookup uses).
    mcu_object: String,
    /// The queue, built and armed (its flush callback registered with the
    /// toolhead) exactly once — by the first request that finds both a
    /// toolhead and a schedulable pin. See the module docs for the fallbacks.
    armed: Mutex<Option<Arc<GCodeRequestQueue<FanSink>>>>,
}

impl Fan {
    /// Build the fan from its section, with upstream's option set.
    ///
    /// `default_shutdown_speed` is what upstream passes in: `0.` for `[fan]`
    /// and `fan_generic`, `1.` for `heater_fan` (a hotend fan that keeps
    /// running when klippy dies).
    ///
    /// # Errors
    /// A missing option, one out of range, or a pin that cannot be set up.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        default_shutdown_speed: f64,
    ) -> Result<Arc<Self>, ConfigError> {
        let identifier = config.identifier();
        // Upstream: max_power above=0. maxval=1.; kick_start_time minval=0.;
        // off_below minval=0. maxval=1.; cycle_time above=0.
        let max_power =
            config.get_float_bounded("max_power", Some(1.0), None, Some(1.0), Some(0.0), None)?;
        let kick_start_time =
            config.get_float_bounded("kick_start_time", Some(0.1), Some(0.0), None, None, None)?;
        let off_below =
            config.get_float_bounded("off_below", Some(0.0), Some(0.0), Some(1.0), None, None)?;
        let cycle_time =
            config.get_float_bounded("cycle_time", Some(0.010), None, None, Some(0.0), None)?;
        let hardware_pwm = config.get_bool("hardware_pwm", Some(false))?;
        let shutdown_speed = config.get_float_bounded(
            "shutdown_speed",
            Some(default_shutdown_speed),
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;

        let pin_desc = config.get("pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let mcu_fan = pins
            .setup_pwm(&pin_desc, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        mcu_fan.setup_max_duration(0.0);
        mcu_fan.setup_cycle_time(cycle_time, hardware_pwm);
        // A fan starts at 0; the shutdown duty is what the firmware falls back
        // to, capped by max_power (`fan.py:26`).
        let shutdown_power = shutdown_speed.clamp(0.0, max_power);
        mcu_fan.setup_start_value(0.0, shutdown_power);

        let enable_pin = match config.get_str("enable_pin") {
            Some(desc) => {
                let pin = pins
                    .setup_digital_out(&desc, None)
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                pin.setup_max_duration(0.0);
                Some(pin)
            }
            None => None,
        };

        // Upstream builds the tachometer after the pins (`fan.py:41`).
        let tachometer = FanTachometer::new(config, &identifier, pins.as_ref())?;

        // The `[mcu …]` object the pin belongs to: upstream reaches its
        // timing through `mcu_fan.get_mcu()`, which `PwmOut` does not carry,
        // so the pin description's chip prefix names it (as `bltouch`'s
        // control-pin lookup does). Resolved here, looked up lazily — the MCU
        // object may register after this section.
        let description = pin_desc.trim_start_matches(|c| matches!(c, '!' | '^' | '~'));
        let chip = match description.split_once(':') {
            Some((chip, _)) if !chip.is_empty() => chip,
            _ => "mcu",
        };
        let drive = Arc::new(FanDrive {
            reactor: printer.reactor(),
            max_power,
            kick_start_time,
            off_below,
            mcu_fan,
            enable_pin,
            state: Arc::new(Mutex::new(FanState::default())),
            identifier,
        });
        let fan = Arc::new(Self {
            drive,
            tachometer,
            printer: Arc::downgrade(printer),
            mcu_object: mcu_object_name(chip),
            armed: Mutex::new(None),
        });

        // Upstream stops every fan when a restart is requested
        // (`fan.py:44-45`), before the new object graph is built, and hands
        // the event's print time to `set_speed` (`fan.py:73-74`).
        let restart = Arc::clone(&fan);
        printer.register_event_handler(
            KlippyEvent::GcodeRequestRestart { print_time: 0.0 },
            Box::new(move |event| {
                let print_time = match event {
                    KlippyEvent::GcodeRequestRestart { print_time } => *print_time,
                    _ => 0.0,
                };
                let _ = restart.set_speed_at(0.0, Some(print_time));
            }),
        );

        Ok(fan)
    }

    /// Choose the speed at the estimated time now: upstream's
    /// `Fan.set_speed(value)` (`fan.py:69-70`), an async request whose date
    /// the caller derives from the pin MCU (`output_pin.py:70-72`).
    ///
    /// The reactor-timer callers (`heater_fan`, `controller_fan`,
    /// `temperature_fan`) land here; a g-code line goes through
    /// [`set_speed_from_command`](Self::set_speed_from_command).
    ///
    /// # Errors
    /// A failed PWM or enable-pin write — only on the immediate fallback
    /// (module docs); the queued path logs a failed send at flush time, as
    /// `output_pin`'s does.
    pub fn set_speed(&self, value: f64) -> Result<(), CommandError> {
        self.set_speed_at(value, None)
    }

    /// `set_speed` with an explicit print time — upstream's `print_time`
    /// parameter (`fan.py:69`), which the restart handler fills from its
    /// event. `None` asks for the estimate [`Self::set_speed`] uses.
    ///
    /// # Errors
    /// As [`Self::set_speed`].
    fn set_speed_at(&self, value: f64, print_time: Option<f64>) -> Result<(), CommandError> {
        // Three things date a request — a toolhead to drain the queue with, a
        // pin that can be scheduled, and a date; without all three the change
        // is driven at once (module docs).
        let request = self.toolhead().and_then(|toolhead| {
            let queue = self.arm(&toolhead)?;
            let print_time = print_time.or_else(|| self.estimated_print_time())?;
            Some((queue, print_time))
        });
        if let Some((queue, print_time)) = request {
            // Upstream `send_async_request` (`output_pin.py:68-90`): the sink
            // runs right away at `max(print_time, next_min_flush_time)`, so
            // the queue's floor spaces this send behind earlier ones.
            queue.send_async_request(value, print_time);
            return Ok(());
        }
        self.drive_now(value)
    }

    /// Choose the speed from a g-code line: upstream's
    /// `Fan.set_speed_from_command`, which queues the request against the
    /// toolhead's lookahead (`queue_gcode_request`, `output_pin.py:61-66`).
    ///
    /// This is where `M106`/`M107` and `SET_FAN_SPEED` land, so a speed
    /// change takes effect at a print time, later requests override earlier
    /// ones, and a kick-start tail is the queue's re-run (module docs).
    ///
    /// # Errors
    /// A failed PWM or enable-pin write — only on the immediate fallback
    /// (module docs); the queued path defers failures to flush time.
    pub fn set_speed_from_command(&self, value: f64) -> Result<(), CommandError> {
        let Some(toolhead) = self.toolhead() else {
            return self.drive_now(value);
        };
        let Some(queue) = self.arm(&toolhead) else {
            return self.drive_now(value);
        };
        toolhead.register_lookahead_callback(Box::new(move |print_time| {
            queue.push(print_time, value);
        }));
        Ok(())
    }

    /// The queue, arming it on the first request that can use it —
    /// `output_pin`'s `PinSchedule::arm` one for one: the flush callback is
    /// registered **before** the queue is published, so a push can never sit
    /// in a queue nothing drains, and `None` when `min_schedule_time()`
    /// answers `None` (the pin's MCU is not connected; the request keeps the
    /// immediate path).
    fn arm(&self, toolhead: &ToolHeadObject) -> Option<Arc<GCodeRequestQueue<FanSink>>> {
        let mut armed = self
            .armed
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if let Some(queue) = armed.as_ref() {
            return Some(Arc::clone(queue));
        }
        let min_schedule_time = self.drive.mcu_fan.min_schedule_time()?;
        let sink = FanSink {
            drive: Arc::clone(&self.drive),
        };
        let queue = Arc::new(GCodeRequestQueue::new(sink, min_schedule_time));
        let flush_queue = Arc::clone(&queue);
        toolhead.register_flush_callback(Box::new(move |flush_time| {
            flush_queue.flush(flush_time);
        }));
        *armed = Some(Arc::clone(&queue));
        Some(queue)
    }

    /// The toolhead, looked up per command: it loads (`phase = late, order =
    /// 60`) after this section (`order = 20`), and a config without
    /// `[printer]` has none at all — that fork stands in the module docs.
    fn toolhead(&self) -> Option<Arc<ToolHeadObject>> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))
    }

    /// The date upstream's `send_async_request` computes when none is given
    /// (`output_pin.py:70-72`):
    /// `mcu.estimated_print_time(reactor.monotonic() + min_schedule_time)`.
    /// `None` while the pin's `[mcu …]` object or its schedule floor is not
    /// there yet — no date, so the caller keeps the immediate path.
    fn estimated_print_time(&self) -> Option<f64> {
        let printer = self.printer.upgrade()?;
        let mcu = printer.lookup_object_as::<McuObject>(&self.mcu_object)?;
        let min_schedule_time = self.drive.mcu_fan.min_schedule_time()?;
        let now = printer.reactor().monotonic();
        mcu.estimated_print_time(now + min_schedule_time)
    }

    /// The immediate fallback: evaluate the request with no date and drive
    /// the pin now (module docs).
    ///
    /// # Errors
    /// A failed PWM or enable-pin write — before connect, "MCU is not
    /// connected", exactly as before this port had a queue.
    fn drive_now(&self, value: f64) -> Result<(), CommandError> {
        self.drive.apply(None, value).map_err(mcu_error)?;
        Ok(())
    }

    /// Upstream's `Fan.get_status`.
    pub fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "speed": self.lock().last_req_value,
            // Upstream reports `None` for a section without `tachometer_pin`
            // (`fan.py:99-102`); one that has a counter reports its RPM.
            "rpm": self.tachometer.rpm(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, FanState> {
        self.drive.lock()
    }
}

impl std::fmt::Debug for Fan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Fan")
            .field("max_power", &self.drive.max_power)
            .field("kick_start_time", &self.drive.kick_start_time)
            .field("off_below", &self.drive.off_below)
            .finish_non_exhaustive()
    }
}

/// The kick-start tail: settle the fan from full power to the duty requested.
///
/// Held by the reactor timer alone, so it carries what it needs rather than a
/// reference back to the section.
struct KickTail {
    mcu_fan: Arc<dyn PwmOut>,
    state: Arc<Mutex<FanState>>,
}

impl KickTail {
    fn run(&self, serial: u64, value: f64) {
        let mut state = self
            .state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if state.kick_serial != serial {
            // A newer request took over while the kick was running.
            return;
        }
        // This tail has fired; anything after it is a fresh request.
        state.kick_serial = state.kick_serial.wrapping_add(1);
        if value == state.last_fan_value {
            // The requested duty is what full power already means (a fan with
            // `max_power: 1` asked for full speed): upstream discards here too.
            return;
        }
        state.last_fan_value = value;
        state.last_req_value = value;
        let _ = self.mcu_fan.update_pwm(value);
    }
}

/// The queue's downstream end: where a due request lands at its print time —
/// upstream passes `Fan._apply_speed` itself as the queue's callback
/// (`fan.py:37-38`).
struct FanSink {
    /// The shared drive: the same evaluation the immediate path runs.
    drive: Arc<FanDrive>,
}

impl RequestSink for FanSink {
    fn set_at(&self, print_time: f64, value: f64) -> Option<(FlushAction, f64)> {
        match self.drive.apply(Some(print_time), value) {
            Ok(action) => action,
            Err(err) => {
                // The flush callback has no error channel (as `output_pin`'s
                // sink): say the frame is lost rather than drop it silently.
                warn!("{}: {err}", self.drive.identifier);
                None
            }
        }
    }
}

/// An MCU write failure, as the command error it is reported through.
fn mcu_error(err: crate::core::klippy::mcu::McuError) -> CommandError {
    CommandError::new(err.to_string())
}

/// One `[fan]`, as upstream's `PrinterFan`.
pub struct PrinterFan {
    fan: Arc<Fan>,
}

impl PrinterFan {
    /// Build the fan and register `M106`/`M107`.
    ///
    /// # Errors
    /// As [`Fan::new`], or a g-code registration failure.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let fan = Fan::new(config, printer, 0.0)?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        // `M106 S<value>`: default 255, `minval=0.`, no upper bound — what
        // caps the duty is `max_power`, inside `_apply_speed` (`fan.py:117-120`).
        let speed = Arc::clone(&fan);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let value =
                gcmd.get("S", Some(255.0), parse_float, Some(0.0), None, None, None)? / 255.;
            speed.set_speed_from_command(value)
        });
        gcode
            .register_command_with_params("M106", handler, None, &["S"], false)
            .map_err(ConfigError::new)?;

        let off = Arc::clone(&fan);
        let handler: CommandHandler =
            sync(move |_gcmd: &GcodeCommand| off.set_speed_from_command(0.));
        gcode
            .register_command("M107", handler, None, false)
            .map_err(ConfigError::new)?;

        Ok(Self { fan })
    }

    /// The fan this section drives; its `M106`/`M107` handlers share it.
    pub fn fan(&self) -> &Arc<Fan> {
        &self.fan
    }
}

impl PrinterObject for PrinterFan {
    fn get_status(&self, eventtime: f64) -> Value {
        self.fan.get_status(eventtime)
    }
}

impl std::fmt::Debug for PrinterFan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterFan").finish_non_exhaustive()
    }
}

/// The factory `section!` names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterFan::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::mcu::{ConfigBuilder, McuChip, McuError};
    use crate::core::klippy::pins::{PinChip, PinError, PinParams};
    use crate::core::klippy::reactor::ManualReactor;

    /// The clock the fake resources map print time through, in Hz.
    const TEST_CLOCK_HZ: f64 = 1_000_000.0;

    /// The schedule floor the fake resources report (the real one is 0.100).
    const TEST_MIN_SCHEDULE_TIME: f64 = 0.1;

    /// A PWM that records what it was told: clocked changes (with the clock
    /// they went out at) and immediate ones separately.
    #[derive(Default)]
    struct FakePwm {
        max_duration: Mutex<f64>,
        cycle_time: Mutex<(f64, bool)>,
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
        /// What the queued path sent, as `(clock, duty)`.
        queued: Mutex<Vec<(u32, f64)>>,
        /// The last duty set, for [`FakePwm::next_aligned_clock`] — `McuPwm`
        /// tracks the same figure to decide whether a duty has a cycle to
        /// land on.
        last_value: Mutex<f64>,
        /// Whether the fake models a connected MCU — a clock to convert print
        /// times with and a schedule floor to queue against.
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

    /// A digital output that records what it was told: clocked changes and
    /// immediate ones separately.
    #[derive(Default)]
    struct FakeDigitalOut {
        max_duration: Mutex<f64>,
        updates: Mutex<Vec<bool>>,
        /// What the queued path sent, as `(clock, level)`.
        queued: Mutex<Vec<(u32, bool)>>,
        /// See [`FakePwm::schedulable`].
        schedulable: bool,
    }

    impl DigitalOut for FakeDigitalOut {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
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

    /// A chip that hands out a [`FakePwm`] or [`FakeDigitalOut`] per setup.
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
        digital: Mutex<Vec<Arc<FakeDigitalOut>>>,
        /// Whether its resources model a connected MCU (the fixtures'
        /// default).
        schedulable: bool,
    }

    impl Default for FakeChip {
        fn default() -> Self {
            Self {
                pwms: Mutex::new(Vec::new()),
                digital: Mutex::new(Vec::new()),
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
            self.digital.lock().unwrap().push(Arc::clone(&out));
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

    /// A ready printer with `gcode` and `pins` over `chip`, plus the reactor
    /// behind it — a test that runs the kick-start tail advances it.
    fn printer_with(chip: FakeChip) -> (Arc<Printer>, Arc<FakeChip>, Arc<ManualReactor>) {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(reactor.clone()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(chip);
        pins.register_chip("mcu", chip.clone()).unwrap();
        // A `tachometer_pin` needs a real MCU chip — the counter takes its oid
        // there (`pulse_counter`) — while the fan's own pins stay on the fake.
        pins.register_chip(
            "counter",
            Arc::new(McuChip::new(
                "counter".to_string(),
                Arc::new(ConfigBuilder::new()),
                Arc::clone(&pins),
            )),
        )
        .unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, chip, reactor)
    }

    /// A ready printer over the default fake chip (a connected MCU).
    fn printer() -> (Arc<Printer>, Arc<FakeChip>, Arc<ManualReactor>) {
        printer_with(FakeChip::default())
    }

    /// A printer with a connected `toolhead` registered — `kinematics: none`,
    /// the dwell-only timeline whose flush callbacks must still run (the
    /// fixture `output_pin`'s queued tests use).
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

    /// A `[fan]` section with `pin: <pin>` plus `options`.
    fn section(pin: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("fan", None);
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

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    fn pwm(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    fn updates(pwm: &FakePwm) -> Vec<f64> {
        pwm.updates.lock().unwrap().clone()
    }

    fn speed(fan: &PrinterFan) -> f64 {
        fan.get_status(0.0)["speed"]
            .as_f64()
            .expect("speed is a number")
    }

    #[test]
    fn test_a_fan_is_configured_with_upstream_defaults() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(&wrap(&section("PA1", &[])), &printer).unwrap();

        let fan = pwm(&chip, 0);
        assert_eq!(*fan.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*fan.cycle_time.lock().unwrap(), (0.010, false));
        // Off at start, and the firmware's shutdown duty is 0 for `[fan]`.
        assert_eq!(*fan.start_value.lock().unwrap(), (0.0, 0.0));
    }

    #[test]
    fn test_shutdown_speed_is_capped_by_max_power() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("max_power", "0.8"), ("shutdown_speed", "1.0")],
            )),
            &printer,
        )
        .unwrap();

        assert_eq!(*pwm(&chip, 0).start_value.lock().unwrap(), (0.0, 0.8));
    }

    #[test]
    fn test_m106_sets_the_speed_and_m107_turns_it_off() {
        let (printer, chip, _reactor) = printer();
        // `kick_start_time: 0` keeps this test about the plain path.
        let fan = PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [128.0 / 255.0]);
        assert!((speed(&fan) - 128.0 / 255.0).abs() < 1e-9);

        // M106 without S is full speed (default 255).
        gcode(&printer).run_script_sync("M106").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [128.0 / 255.0, 1.0]);
        assert_eq!(speed(&fan), 1.0);

        gcode(&printer).run_script_sync("M107").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [128.0 / 255.0, 1.0, 0.0]);
        assert_eq!(speed(&fan), 0.0);
        assert!(fan.get_status(0.0)["rpm"].is_null());
    }

    #[test]
    fn test_m106_rejects_a_negative_speed() {
        let (printer, _chip, _reactor) = printer();
        PrinterFan::new(&wrap(&section("PA1", &[])), &printer).unwrap();

        let err = gcode(&printer).run_script_sync("M106 S-1").unwrap_err();

        assert!(err.to_string().contains("minimum of 0"), "{err}");
    }

    #[test]
    fn test_kick_start_runs_at_full_power_then_settles() {
        let (printer, chip, reactor) = printer();
        // Half power, so the kick (full *configured* power) is distinguishable
        // from the duty that was asked for.
        let fan = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("max_power", "0.5"), ("kick_start_time", "0.1")],
            )),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        let target = 128.0 / 255.0 * 0.5;
        assert_eq!(updates(&pwm(&chip, 0)), [0.5], "starts at full power");
        // Until the kick runs out, the *requested* speed is what is reported.
        assert!((speed(&fan) - target).abs() < 1e-9);

        reactor.advance(0.1);
        let driven = updates(&pwm(&chip, 0));
        assert_eq!(
            driven.len(),
            2,
            "the tail writes the requested duty: {driven:?}"
        );
        assert!((driven[1] - target).abs() < 1e-9, "{driven:?}");
        assert!((speed(&fan) - target).abs() < 1e-9);
    }

    #[test]
    fn test_a_new_request_supersedes_a_pending_kick() {
        let (printer, chip, reactor) = printer();
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0.1")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [1.0], "kicking at full power");

        // A second request before the kick runs out owns the fan from here.
        gcode(&printer).run_script_sync("M106 S64").unwrap();
        reactor.advance(0.1);
        assert_eq!(
            updates(&pwm(&chip, 0)),
            [1.0, 64.0 / 255.0],
            "the stale tail must not write the first request's duty"
        );
    }

    #[test]
    fn test_off_below_snaps_a_small_request_to_zero() {
        let (printer, chip, _reactor) = printer();
        let fan = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("kick_start_time", "0"), ("off_below", "0.2")],
            )),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S5").unwrap();
        assert!(updates(&pwm(&chip, 0)).is_empty(), "5/255 is below 0.2");
        assert_eq!(speed(&fan), 0.0);

        gcode(&printer).run_script_sync("M106 S200").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [200.0 / 255.0]);
    }

    #[test]
    fn test_max_power_caps_the_speed() {
        let (printer, chip, _reactor) = printer();
        let fan = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("kick_start_time", "0"), ("max_power", "0.5")],
            )),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [0.5]);
        assert_eq!(speed(&fan), 0.5);
    }

    #[test]
    fn test_the_enable_pin_moves_only_on_off_to_on() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[
                    ("kick_start_time", "0"),
                    ("enable_pin", "PB0"),
                    ("max_power", "1.0"),
                ],
            )),
            &printer,
        )
        .unwrap();
        let enable = chip.digital.lock().unwrap()[0].clone();
        assert_eq!(*enable.max_duration.lock().unwrap(), 0.0);

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        gcode(&printer).run_script_sync("M106 S255").unwrap();
        gcode(&printer).run_script_sync("M107").unwrap();

        // On, then a change of speed (both non-zero: no edge), then off.
        assert_eq!(*enable.updates.lock().unwrap(), [true, false]);
        assert_eq!(
            updates(&pwm(&chip, 0)),
            [128.0 / 255.0, 1.0, 0.0],
            "the fan itself follows every request"
        );
    }

    #[test]
    fn test_a_restart_request_stops_the_fan() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106").unwrap();
        printer.send_event(&KlippyEvent::GcodeRequestRestart { print_time: 0.0 });

        assert_eq!(updates(&pwm(&chip, 0)), [1.0, 0.0]);
    }

    #[test]
    fn test_the_tachometer_pin_builds_a_counter_and_reports_zero_rpm() {
        let (printer, _chip, _reactor) = printer();

        let fan = PrinterFan::new(
            &wrap(&section("PA1", &[("tachometer_pin", "counter:PC0")])),
            &printer,
        )
        .unwrap();

        // No edge has been counted yet: 0 Hz scales to 0 RPM, which is a
        // number — upstream reports `None` only for a section with no
        // tachometer pin at all.
        assert_eq!(fan.get_status(0.0)["rpm"], json!(0.0));
    }

    #[test]
    fn test_the_tachometer_pin_may_carry_a_pull_up() {
        let (printer, _chip, _reactor) = printer();

        PrinterFan::new(
            &wrap(&section("PA1", &[("tachometer_pin", "^counter:PC0")])),
            &printer,
        )
        .unwrap();
    }

    #[test]
    fn test_a_tachometer_ppr_below_one_is_refused() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("tachometer_pin", "counter:PC0"), ("tachometer_ppr", "0")],
            )),
            &printer,
        )
        .unwrap_err();

        assert!(err.to_string().contains("tachometer_ppr"), "{err}");
    }

    #[test]
    fn test_a_tachometer_poll_interval_must_be_above_zero() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[
                    ("tachometer_pin", "counter:PC0"),
                    ("tachometer_poll_interval", "0"),
                ],
            )),
            &printer,
        )
        .unwrap_err();

        assert!(err.to_string().contains("must be above 0"), "{err}");
    }

    #[test]
    fn test_rpm_is_the_frequency_scaled_by_the_ppr() {
        // 60 Hz over two pulses per revolution is 30 revolutions per second.
        assert_eq!(to_rpm(60., 2.), 900.);
        assert_eq!(to_rpm(0., 4.), 0.);
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(&wrap(&ConfigSection::new("fan", None)), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'fan' must be specified"
        );
    }

    #[test]
    fn test_an_option_out_of_range_says_which_bound() {
        let (printer, _chip, _reactor) = printer();

        let err =
            PrinterFan::new(&wrap(&section("PA1", &[("max_power", "0")])), &printer).unwrap_err();
        assert!(err.to_string().contains("must be above 0"), "{err}");

        let err = PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "-0.1")])),
            &printer,
        )
        .unwrap_err();
        assert!(err.to_string().contains("minimum of 0"), "{err}");

        let err =
            PrinterFan::new(&wrap(&section("PA1", &[("cycle_time", "0")])), &printer).unwrap_err();
        assert!(err.to_string().contains("cycle_time"), "{err}");
    }

    #[test]
    fn test_an_unparseable_number_is_reported() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(&wrap(&section("PA1", &[("off_below", "half")])), &printer)
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'off_below' in section 'fan'"
        );
    }

    // -----------------------------------------------------------------------
    // The queued (print-time) path, and its two immediate fallbacks
    // -----------------------------------------------------------------------

    /// An `M106` is captured by `register_lookahead_callback`: nothing is
    /// driven at command time — the request waits in the queue — and the
    /// flush lands it at the lookahead's print time as a **clocked** change,
    /// never the immediate `update_pwm`. The status only moves when the
    /// request lands (upstream's `last_req_value` moves inside `_apply_speed`).
    #[tokio::test]
    async fn test_m106_is_queued_at_the_lookahead_print_time() {
        let (printer, chip, _reactor) = printer();
        let toolhead = add_toolhead(&printer).await;
        let fan = PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script("M106 S128").await.unwrap();

        let pwm = pwm(&chip, 0);
        assert!(
            pwm.queued.lock().unwrap().is_empty(),
            "nothing is driven before a flush: the request waits in the queue"
        );
        assert!(
            pwm.updates.lock().unwrap().is_empty(),
            "the immediate update_pwm path is not taken"
        );
        assert_eq!(speed(&fan), 0.0, "the status waits for the landing too");

        toolhead.flush_step_generation().await.unwrap();

        let print_time = toolhead.print_time();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, 128.0 / 255.0)],
            "flush lands the request at its lookahead print time"
        );
        assert!(pwm.updates.lock().unwrap().is_empty());
        assert!((speed(&fan) - 128.0 / 255.0).abs() < 1e-9);
    }

    /// Later request overrides earlier one (`output_pin.py:35-38` through the
    /// fan's queue): two `M106`s at the same lookahead time flush into a
    /// single frame carrying the second value.
    #[tokio::test]
    async fn test_a_later_queued_request_overrides_the_earlier_one() {
        let (printer, chip, _reactor) = printer();
        let toolhead = add_toolhead(&printer).await;
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script("M106 S128").await.unwrap();
        gcode(&printer).run_script("M106 S64").await.unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = pwm(&chip, 0);
        let print_time = toolhead.print_time();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, 64.0 / 255.0)],
            "only the covering request reaches the pin, at its own print time"
        );
        assert!(pwm.updates.lock().unwrap().is_empty());
    }

    /// The kick-start tail is the queued request **re-run** (`fan.py:60-66`,
    /// the queue's "repeat"): full power lands at the request's print time,
    /// the requested duty at `print_time + kick_start_time` — and stepping
    /// the reactor adds nothing, because no timer holds the tail.
    #[tokio::test]
    async fn test_the_kick_start_tail_is_a_queue_rerun() {
        let (printer, chip, reactor) = printer();
        let toolhead = add_toolhead(&printer).await;
        PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("max_power", "1.0"), ("kick_start_time", "0.1")],
            )),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script("M106 S128").await.unwrap();
        // Two passes: the retry is aligned to `print_time + 0.1`, which the
        // first generation's horizon may not reach.
        toolhead.flush_step_generation().await.unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = pwm(&chip, 0);
        let print_time = toolhead.print_time();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [
                ((print_time * TEST_CLOCK_HZ) as u64 as u32, 1.0),
                (
                    ((print_time + 0.1) * TEST_CLOCK_HZ) as u64 as u32,
                    128.0 / 255.0
                ),
            ],
            "full power, then the requested duty at the re-run's time"
        );
        assert!(
            pwm.updates.lock().unwrap().is_empty(),
            "the immediate path never runs"
        );

        // The tail is the queue's, not a reactor timer's: time moving on
        // produces no further write.
        reactor.advance(0.2);
        assert_eq!(pwm.queued.lock().unwrap().len(), 2);
    }

    /// Fork one: no `toolhead` (a config without `[printer]`), even though the
    /// pin could schedule — `M106` drives the pin immediately and queues
    /// nothing.
    #[test]
    fn test_without_a_toolhead_m106_drives_the_pin_immediately() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S128").unwrap();

        let pwm = pwm(&chip, 0);
        assert_eq!(*pwm.updates.lock().unwrap(), [128.0 / 255.0]);
        assert!(pwm.queued.lock().unwrap().is_empty());
    }

    /// Fork two: a toolhead, but a pin whose MCU is not connected (no
    /// schedule floor to queue with) — immediate path, no error, no panic.
    #[tokio::test]
    async fn test_a_fan_that_cannot_schedule_drives_immediately() {
        let (printer, chip, _reactor) = printer_with(FakeChip {
            schedulable: false,
            ..FakeChip::default()
        });
        add_toolhead(&printer).await;
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script("M106 S128").await.unwrap();

        let pwm = pwm(&chip, 0);
        assert_eq!(*pwm.updates.lock().unwrap(), [128.0 / 255.0]);
        assert!(pwm.queued.lock().unwrap().is_empty());
    }

    /// The restart handler keeps upstream's hand-off (`fan.py:73-74`): the
    /// event's print time dates an async request, which the queue drives at
    /// `max(print_time, next_min_flush_time)` — the pin sees a clocked 0.0,
    /// no immediate write.
    #[tokio::test]
    async fn test_a_restart_request_stops_the_fan_through_the_queue() {
        let (printer, chip, _reactor) = printer();
        let toolhead = add_toolhead(&printer).await;
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script("M106 S128").await.unwrap();
        toolhead.flush_step_generation().await.unwrap();
        let pwm = pwm(&chip, 0);
        assert_eq!(pwm.queued.lock().unwrap().len(), 1);

        // The event carries the last move time, as `request_restart` notes it.
        printer.send_event(&KlippyEvent::GcodeRequestRestart {
            print_time: toolhead.print_time(),
        });

        let queued = pwm.queued.lock().unwrap().clone();
        assert_eq!(queued.len(), 2, "{queued:?}");
        assert_eq!(queued[1].1, 0.0, "the fan is driven off at the event time");
        assert!(pwm.updates.lock().unwrap().is_empty());
    }
}
