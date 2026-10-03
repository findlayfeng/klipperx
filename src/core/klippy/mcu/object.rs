//! `[mcu]` as a printer object: the section, and the connection it needs.
//!
//! Upstream's `MCU` is one printer object per `[mcu]` / `[mcu <name>]` section
//! (`klippy/mcu.py:1146`), registered by `mcu.add_printer_objects` under the
//! section's name (`klippy/mcu.py:1239`). Its status is the identify snapshot
//! `MCUStatsHelper` takes once the handshake is done (`klippy/mcu.py:938-975`).
//!
//! Two-phase construction is why this type is separate from [`Mcu`]: the loader
//! builds it from the section without touching the device, and
//! [`PrinterObject::connect`] parses the section and performs the identify
//! handshake when the machine comes up. That keeps a config parse free of side
//! effects — no serial port is opened just to read the file — which is also why
//! the whole [`ConfigSection`] is kept rather than a parsed [`McuConfig`].
//!
//! The object also owns the MCU's [`ConfigBuilder`], created here rather than at
//! connect: resources add their oids and `config_*` commands while the config
//! file is loaded, and connect is what finally sends them
//! (`builder.configure`), after identify has made the dictionary available.
//!
//! The reported fields are the three identify ones. `last_stats` is upstream's
//! fourth (`klippy/mcu.py:975`) and is **not** reported yet: it is accumulated
//! from the `stats` event, which today is only logged
//! ([`register_stats`](crate::core::klippy::event::stats::register_stats)).
//! It comes back with the statistics consumer.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use serde_json::{json, Map, Value};
use tracing::{debug, info, warn};

use crate::core::klippy::cmd::clock::{ClockSync, McuClock, SecondarySync};
use crate::core::klippy::cmd::config::Reset;
use crate::core::klippy::cmd::shutdown::EmergencyStop;
use crate::core::klippy::cmd::uptime::{GetUptime, Uptime};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::config::value::ConfigValue;
use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigWrapper};
use crate::core::klippy::error::{ConfigError, KlippyError};
use crate::core::klippy::event::stats::{register_stats, LastStats};
use crate::core::klippy::event::{IsShutdown, KlippyEvent, McuEvent, Shutdown, Starting};
use crate::core::klippy::mcu::{
    BuiltConfig, ConfigBuilder, Configured, Dictionary, I2cMode, Mcu, McuChip, McuError, McuI2c,
    McuRestartMethod, McuSpi, McuStepper, SpiMode,
};
use crate::core::klippy::pins::{PinError, PinParams, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject, RestartFuture};
use crate::core::klippy::reactor::{Reactor, TimerHandle};

/// How long to wait between attempts to reopen a board that was just told to
/// reboot. A native-USB board re-enumerates, so the port is briefly gone.
const RECONNECT_DELAY: Duration = Duration::from_millis(250);

/// How many times to try reopening before giving up (a little over 5 s).
const RECONNECT_ATTEMPTS: usize = 20;

/// How long to wait for the `reset` command to leave the send queue before the
/// rebooted board takes the transport with it.
const RESET_FLUSH_TIMEOUT: Duration = Duration::from_secs(1);

/// How many times one bring-up may reset the firmware in place before it gives
/// up.
///
/// A firmware whose only way out of a configuration is `reset` drops its
/// connection when it reboots, so the handshake has to be re-run on the
/// connection that comes back — and on a real board that reboot sometimes comes
/// back stopped: the firmware reports a clock fault (`Timer too close`,
/// `Rescheduled timer in the past`) less than a second after it starts, and the
/// handshake can never finish on that session. Observed on a two-board setup
/// (an stm32f103 `mcu` beside a CP2102 `mcu2`): a fresh process was often ready
/// on its second or third start, so a bring-up that resets again usually
/// succeeds. Bounded because each round is a reboot and a re-identify; a board
/// that cannot come up in three tries is not going to on the fourth.
const MAX_IN_PLACE_RESETS: u32 = 3;

/// How long the firmware gets to start before the port is reopened after a
/// `reset`.
///
/// A rebooted board is not ready when the `reset` has flushed: the firmware
/// still has to come up, and reopening the port straight away races its startup
/// — on the real board that showed as an identify timeout, or as the firmware
/// tripping a clock fault of its own less than a second after it started.
const RESET_SETTLE: Duration = Duration::from_millis(500);

/// How long the one `get_uptime` that seeds the clock estimate may take.
///
/// The same order as the other connect-time reads; a firmware that does not
/// answer only loses the estimate, not the connection.
const CLOCK_BASE_TIMEOUT: Duration = Duration::from_secs(1);

/// How often a secondary MCU's clock alignment is recalibrated, in seconds
/// (upstream does it in the periodic `stats`, `klippy/extras/motion_queuing.py:100`).
const RECALIBRATE_INTERVAL: f64 = 1.0;

/// How often the host reads a connected MCU's clock to keep its estimate fed,
/// in seconds. Upstream's `get_clock` timer fires every ~0.9839 s — deliberately
/// off the round second so it does not resonate with other periodic events
/// (`klippy/clocksync.py:62-67`); one second here, the same order as
/// [`RECALIBRATE_INTERVAL`], whose precedent the registration follows.
const CLOCK_POLL_INTERVAL: f64 = 1.0;

/// The same read, for a **fake** transport (`test: dict=`): 150 ms.
///
/// A fake link answers instantly, so only cadence is given up: polling fast
/// keeps the estimate — and the send gates that judge against it — fresh for
/// a run that finishes in seconds rather than hours, and fills the
/// estimator's fit window early. A production link keeps
/// [`CLOCK_POLL_INTERVAL`] (C3b's ~1 s cadence, RTT sample density and
/// fit-window semantics unchanged — only transports the config names
/// `test:` poll fast).
const FAKE_CLOCK_POLL_INTERVAL: f64 = 0.15;

/// The printer object for one `[mcu]` / `[mcu <name>]` section.
pub struct McuObject {
    /// The section as parsed, kept whole so the device is only opened at
    /// connect time.
    section: ConfigSection,
    /// This MCU as a pin chip: its name, its configuration builder, and the
    /// slot resources use to reach the device once it connects (`mcu/resource/pin.rs`).
    ///
    /// Built at construction, not at connect, because resources add their
    /// `config_*` commands while the config file is being loaded — long before
    /// the device is opened.
    chip: McuChip,
    /// What `objects/query` reports; `{}` until the handshake fills it, which is
    /// what upstream's `_get_status_info` starts as.
    status: Mutex<Value>,
    /// The restart method of the connection currently up.
    ///
    /// [`McuObject::before_firmware_restart`] runs on the live connection and has
    /// to know whether this is one of the methods that resets there (`command`)
    /// or one that needs the port closed (`mcu/restart.rs`). Set by `connect`, so
    /// it describes the connection that is actually open.
    restart_method: Mutex<McuRestartMethod>,
    /// Whether this firmware is (or has been locally declared) stopped.
    ///
    /// Upstream's `_is_shutdown` (`klippy/mcu.py:794`): set when the firmware
    /// reports a stop, and by [`McuObject::force_local_shutdown`]. A host
    /// shutdown only sends `emergency_stop` when this is false — echoing the
    /// stop back to a firmware that just reported it would be pointless, and
    /// after a config reset the firmware is already stopped.
    ///
    /// `Arc` rather than a bare atomic because the `shutdown`/`is_shutdown`
    /// event handlers are `'static` and need their own handle to it.
    is_shutdown: Arc<AtomicBool>,
    /// The reason the firmware gave for a stop it reported **while this
    /// connection was still connecting**, first one wins.
    ///
    /// Separate from [`McuObject::is_shutdown`] on purpose: a bring-up clears a
    /// stopped or differently configured firmware itself (`mcu/config.rs`), so a
    /// stop reported here is not a spontaneous one and must not become a printer
    /// shutdown. What it does mean is that the handshake's remaining answers will
    /// never come — the firmware refuses every command but the few that run while
    /// stopped (`src/command.c:346-349`) — so the reason is what the bring-up
    /// fails with instead of its response timeout
    /// ([`McuObject::connect_failure`]). `Arc` because the recording handlers are
    /// `'static`.
    connect_shutdown: Arc<Mutex<Option<String>>>,
    /// The latest scheduler load from the firmware's `stats` reports, for
    /// `get_status`'s `last_stats` (`klippy/mcu.py:974-975`).
    ///
    /// `Arc` because the `'static` event handler owns its handle to it.
    last_stats: Arc<Mutex<Option<LastStats>>>,
    /// The secondary clock alignment, when this is not the primary MCU
    /// (`SecondarySync`); `None` for the primary.
    secondary_sync: Mutex<Option<SecondarySync>>,
    /// The periodic recalibration timer, cancelled on drop.
    recalibrate_timer: Mutex<Option<TimerHandle>>,
    /// The periodic clock poll timer (`mcu_clock_poll`), cancelled on drop.
    clock_poll_timer: Mutex<Option<TimerHandle>>,
    /// The machine, for reporting a firmware shutdown. `Weak` because the
    /// printer's registry owns this object: a strong handle would be a cycle
    /// that keeps the printer (and its device) alive forever.
    printer: Weak<Printer>,
}

impl McuObject {
    /// Build the object for `section`, without touching the device.
    ///
    /// Also declares this MCU as a chip on the shared `pins` object, as
    /// upstream's `MCUConfigHelper.__init__` does
    /// (`klippy/mcu.py:1000`): a pin description may name this MCU as its
    /// chip from the moment the config file mentions it.
    ///
    /// # Errors
    /// Returns [`PinError::DuplicateChip`] if another `[mcu]` section already
    /// claimed this name (`[mcu]` and `[mcu mcu]` would collide).
    pub fn new(section: ConfigSection, printer: &Arc<Printer>) -> Result<Self, PinError> {
        let name = section.sub.clone().unwrap_or_else(|| section.id.clone());
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let chip = McuChip::new(name, Arc::new(ConfigBuilder::new()), Arc::clone(&pins));
        // The chip is registered by value-shared handle, so the slot the
        // resources read is the same one connect fills.
        pins.register_chip(chip.name(), Arc::new(chip.clone()))?;
        Ok(Self {
            section,
            chip,
            status: Mutex::new(json!({})),
            restart_method: Mutex::new(McuRestartMethod::Command),
            is_shutdown: Arc::new(AtomicBool::new(false)),
            connect_shutdown: Arc::new(Mutex::new(None)),
            last_stats: Arc::new(Mutex::new(None)),
            secondary_sync: Mutex::new(None),
            recalibrate_timer: Mutex::new(None),
            clock_poll_timer: Mutex::new(None),
            printer: Arc::downgrade(printer),
        })
    }

    /// The MCU's own name, as upstream's `MCU.get_name` reports it.
    pub fn name(&self) -> &str {
        self.chip.name()
    }

    /// Whether this firmware is stopped (or the host has marked it so).
    ///
    /// Upstream's `MCU.is_shutdown()` (`klippy/mcu.py:906-907`).
    pub fn is_shutdown(&self) -> bool {
        self.is_shutdown.load(Ordering::SeqCst)
    }

    /// Mark the firmware stopped and tell it to stop, whatever it reports.
    ///
    /// Upstream's `force_local_shutdown` (`klippy/mcu.py:894-896`): used on a
    /// path that is about to reset the firmware in place, where the stop must
    /// not wait for `klippy:shutdown` and the firmware's own report must not be
    /// taken for a spontaneous stop.
    pub fn force_local_shutdown(&self) {
        self.is_shutdown.store(true, Ordering::SeqCst);
        self.send_emergency_stop();
    }

    /// Handle a host shutdown: stop the firmware unless it already stopped.
    ///
    /// Registered on `klippy:shutdown` by [`load_config`]. The swap makes this
    /// the first stop, so a second host shutdown (or a firmware report that
    /// raced it) does not send `emergency_stop` again.
    fn on_host_shutdown(&self) {
        if self.is_shutdown.swap(true, Ordering::SeqCst) {
            return;
        }
        self.send_emergency_stop();
    }

    /// Send `emergency_stop` on the live connection, if there is one.
    ///
    /// A part that is not connected has nothing to stop, and a send that fails
    /// (the queue is closed, the firmware has no such command) only costs a
    /// warning: the machine is already on its way down.
    fn send_emergency_stop(&self) {
        let Some(mcu) = self.chip.mcu() else {
            return;
        };
        if let Err(err) = mcu.send_msg(&EmergencyStop) {
            warn!(
                "MCU '{}': could not send emergency_stop: {err}",
                self.name()
            );
        }
    }

    /// The configuration builder for this MCU.
    ///
    /// What a resource (a pin, a bus, a sensor) uses to reserve an oid and add
    /// its `config_*` command. It exists before the device does, which is the
    /// point: the config file is loaded before anything connects.
    pub fn config(&self) -> Arc<ConfigBuilder> {
        self.chip.config()
    }

    /// The connected device, or `None` before connect.
    ///
    /// The clock/trigger layer asks the MCU for its frequency and clock; a
    /// resource reaches it through its own [`McuStepper`]/pin handle, but a
    /// consumer that found this object by name (the toolhead looking for the
    /// primary) needs the handle here.
    pub fn mcu(&self) -> Option<Arc<Mcu>> {
        self.chip.mcu()
    }

    /// Build an I2C device on this MCU.
    ///
    /// The bus counterpart of the pin resources `pins` builds: a section that
    /// owns an I2C device asks its MCU for it. Upstream's `MCU_I2C`
    /// (`klippy/extras/bus.py:161`).
    pub fn setup_i2c(&self, mode: I2cMode, address: u8) -> Arc<McuI2c> {
        self.chip.setup_i2c(mode, address, self.printer.clone())
    }

    /// Build an SPI device on this MCU.
    ///
    /// The SPI counterpart of [`McuObject::setup_i2c`]: a section that owns an
    /// SPI device asks its MCU for it. Upstream's `MCU_SPI`
    /// (`klippy/extras/bus.py:42`).
    pub fn setup_spi(
        &self,
        mode: SpiMode,
        cs_pin: Option<PinParams>,
        cs_active_high: bool,
    ) -> Arc<McuSpi> {
        self.chip.setup_spi(mode, cs_pin, cs_active_high)
    }

    /// Build a stepper on this MCU.
    ///
    /// The motion counterpart of the pin and bus resources: a `[stepper_*]`
    /// section asks its MCU for it (FW5e wires the section up). Upstream's
    /// `MCU_stepper` (`klippy/stepper.py:22`).
    pub fn setup_stepper(
        &self,
        step_pin: PinParams,
        dir_pin: PinParams,
        invert_step: i8,
        step_pulse_duration: f64,
        invert_dir: bool,
    ) -> Arc<McuStepper> {
        self.chip.setup_stepper(
            step_pin,
            dir_pin,
            invert_step,
            step_pulse_duration,
            invert_dir,
        )
    }

    /// The clock estimate for this MCU, once connected.
    ///
    /// The endstop/trsync layer uses it to turn print times into this MCU's
    /// clock and to extend 32-bit readings.
    pub fn clock(&self) -> Option<Arc<McuClock>> {
        self.chip.clock()
    }

    /// The print time this MCU's clock zero corresponds to (`0.0` for the
    /// primary).
    pub fn print_time_offset(&self) -> f64 {
        self.chip.print_time_offset()
    }

    /// Convert an absolute print time to this MCU's clock, once connected.
    pub fn print_time_to_clock(&self, print_time: f64) -> Option<u64> {
        self.chip.print_time_to_clock(print_time)
    }

    /// Extend a 32-bit clock reading into this MCU's 64-bit domain.
    pub fn clock32_to_clock64(&self, clock32: u32) -> Option<i64> {
        self.chip.clock32_to_clock64(clock32)
    }

    /// The estimated print time at a host instant, once connected.
    pub fn estimated_print_time(&self, eventtime: f64) -> Option<f64> {
        self.clock()
            .map(|clock| clock.estimated_print_time(eventtime))
    }

    /// The `(print-time offset, frequency)` this MCU's clock is mapped with.
    pub fn time_mapping(&self) -> (f64, f64) {
        self.chip.time_mapping()
    }

    /// Realign a secondary MCU's clock to the primary's (`SecondarySync`).
    ///
    /// Called by this MCU's recalibration timer. Nothing happens when this is
    /// the primary, or before both clocks are known.
    pub fn recalibrate(&self) {
        let mut guard = self
            .secondary_sync
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let Some(sync) = guard.as_mut() else {
            return;
        };
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let Some(primary) = printer.lookup_object_as::<McuObject>("mcu") else {
            return;
        };
        let (Some(primary_clock), Some(local_clock)) = (primary.clock(), self.clock()) else {
            return;
        };
        let now = printer.reactor().monotonic();
        let print_time = primary_clock.estimated_print_time(now);
        sync.calibrate(
            &primary_clock.estimator(),
            &local_clock.estimator(),
            print_time,
            now,
        );
        self.chip.set_mapping(sync.offset, sync.freq);
    }

    /// Register the periodic clock read that feeds the estimate
    /// (`mcu_clock_poll`), following the `mcu_recalibrate` timer's shape for
    /// registration, cancellation and shutdown.
    ///
    /// Samples reach the estimators only through [`McuClock::get_clock`]
    /// (`cmd/clock.rs`), and upstream drives it from a timer that fires every
    /// ~0.9839 s (`klippy/clocksync.py:62-67`); without a periodic query the
    /// estimate would run on its connect seed alone forever. The timer is
    /// registered for **every** MCU: `mcu_recalibrate` only *reads* the
    /// estimators (`SecondarySync::calibrate` is pure math), so a primary and a
    /// secondary alike get their samples from this poll — one query stream per
    /// MCU, none fed twice.
    ///
    /// A failed round trip is logged at debug level and the next interval runs
    /// as usual: this timer feeds an estimate, it does not watch for a dead MCU.
    /// Cancellation is the precedent's — [`McuObject::release_cycles`] on
    /// teardown (`Drop`), so nothing polls after the machine comes down.
    ///
    /// One timer per object: [`McuObject::install_clock`] runs again when a
    /// reconnect replaces the session, and a second timer beside the first
    /// (whose handle the slot would then have lost) would poll twice over.
    fn register_clock_poll(&self, reactor: &dyn Reactor) {
        if self
            .clock_poll_timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .is_some()
        {
            return;
        }
        let printer = self.printer.clone();
        let identifier = self.section.identifier();
        // The fake's jumped clock is picked up by this poll alone, so it runs
        // at `test:` cadence; everything else keeps the production interval.
        let interval = if self.section.parameters.contains_key("test") {
            FAKE_CLOCK_POLL_INTERVAL
        } else {
            CLOCK_POLL_INTERVAL
        };
        let handle = reactor.register_timer_named(
            "mcu_clock_poll",
            Box::new(move |eventtime| {
                let Some(printer) = printer.upgrade() else {
                    return None;
                };
                if let Some(object) = printer.lookup_object_as::<McuObject>(&identifier) {
                    object.poll_clock();
                }
                Some(eventtime + interval)
            }),
            reactor.monotonic() + interval,
        );
        *self
            .clock_poll_timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(handle);
    }

    /// One `mcu_clock_poll` round: read the firmware clock.
    ///
    /// The round trip is awaited on the runtime, not here — a timer callback
    /// must not block the dispatcher. [`McuClock::get_clock`] folds the answer
    /// into its regression and, through it, into `Mcu::record_clock_sample`; a
    /// query that fails (a firmware without the message, a timeout) logs at
    /// debug level, feeds nothing and changes nothing — the timer keeps its
    /// schedule and the next round tries again.
    ///
    /// A **closed** session is not even asked: between a reconnect's
    /// [`Mcu::close`] and the new session's clock landing on the chip, the slot
    /// still names the old connection, and querying it would push a dead
    /// session onto the wire once an interval for nothing. The next fire finds
    /// the clock `reconnect` installed.
    fn poll_clock(&self) {
        let Some(clock) = self.clock() else {
            // No estimate, so no seed either: there is nothing to feed.
            return;
        };
        let name = self.chip.name().to_string();
        if clock.mcu().is_closed() {
            debug!("MCU '{name}': clock poll skipped: the session is closed");
            return;
        }
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(err) = clock.get_clock().await {
                        debug!("MCU '{name}': clock poll failed: {err}");
                    }
                });
            }
            Err(_) => debug!("MCU '{name}': the clock poll needs an async runtime"),
        }
    }

    /// Snapshot a connected MCU's identify status for `objects/query`.
    fn set_status(&self, mcu: &Mcu) {
        let status = mcu
            .dictionary()
            .map(|dictionary| status_from(&dictionary))
            .unwrap_or_else(|| json!({}));
        *self.status.lock().unwrap_or_else(|p| p.into_inner()) = status;
    }

    /// Reserve the pins the firmware says belong to it.
    ///
    /// The dictionary carries `RESERVE_PINS_<name>` constants as
    /// comma-separated pin lists (UART, USB, …); upstream reserves them at
    /// `klippy:mcu_identify` (`klippy/mcu.py:1091-1100`), which is just after
    /// identify here. A pin already reserved for something else is a config
    /// error, because the resource that wants it would silently fight the
    /// firmware.
    fn reserve_pins(&self, mcu: &Mcu) -> Result<(), PinError> {
        let Some(dictionary) = mcu.dictionary() else {
            return Ok(());
        };
        let pins = self.chip.pins();
        for (key, value) in dictionary.constants() {
            let Some(reserve_name) = key.strip_prefix("RESERVE_PINS_") else {
                continue;
            };
            let Some(pin_list) = value.as_str() else {
                continue;
            };
            for pin in pin_list.split(',') {
                let pin = pin.trim();
                if !pin.is_empty() {
                    pins.reserve_pin(self.chip.name(), pin, reserve_name)?;
                }
            }
        }
        Ok(())
    }

    /// Open the port and identify the firmware, retrying while a rebooted board
    /// comes back.
    ///
    /// `reset` restarts the MCU, and a native-USB board disappears from the bus
    /// and re-enumerates, so the port may not be openable for a moment
    /// (`mcu/restart.rs`). Only a bring-up that follows a reset uses this: a
    /// plain start opens the port once and reports the error, so a missing
    /// device is not hidden behind a retry.
    async fn open_and_connect(&self, config: &McuConfig) -> Result<Arc<Mcu>, KlippyError> {
        let mut last = String::new();
        for _ in 0..RECONNECT_ATTEMPTS {
            tokio::time::sleep(RECONNECT_DELAY).await;
            let interface = match config.open() {
                Ok(interface) => interface,
                Err(err) => {
                    last = err;
                    continue;
                }
            };
            match Mcu::connect(&config.name, interface).await {
                Ok(mcu) => return Ok(mcu),
                Err(err) => last = err.to_string(),
            }
        }
        Err(KlippyError::Connection(format!(
            "MCU '{}' did not come back after a reset: {last}",
            config.name
        )))
    }

    /// Reopen a firmware that was just told to reboot.
    ///
    /// The old session is **closed explicitly** before the port is reopened,
    /// the way upstream ends one: `_restart_via_command` sends `reset`, pauses
    /// 15 ms, then calls `self._disconnect()` (`klippy/mcu.py:730-747`) — the
    /// disconnect is a step of its own, not a consequence of letting go of a
    /// handle. Dropping this one reference would not end the session either:
    /// the chip's device slot and the `McuClock` behind the clock poll still
    /// hold their `Arc`s, so `Mcu::Drop` never runs and both transport tasks
    /// keep working a port that is about to be reopened behind them (an old
    /// write end failing with EIO against a re-enumerated USB CDC, an old read
    /// end still stealing a UART's frames — r8host's death spiral).
    async fn reconnect(
        &self,
        config: &McuConfig,
        previous: Arc<Mcu>,
    ) -> Result<Arc<Mcu>, KlippyError> {
        previous.close();
        // The session that comes up is a new one, and the old connection's last
        // words are not its: a `shutdown` the dying firmware reported while the
        // `reset` was taking hold would otherwise fail the bring-up that
        // replaces it.
        self.forget_connect_shutdown();
        let mcu = self
            .open_and_connect(config)
            .await
            .map_err(|err| self.connect_failure(err))?;
        self.chip.attach(Arc::clone(&mcu));
        // The new connection needs its own watcher as much as the first one did:
        // the handshake runs on it too (the caller re-runs it after this
        // returns).
        self.watch_connect_shutdown(&mcu)
            .map_err(|err| KlippyError::Internal(err.to_string()))?;
        // A fresh connection has a fresh clock: re-seed the estimate and put a
        // clock bound to *this* session on the chip. The clock the first
        // `connect` installed still queries the old `Mcu` — installing a new
        // one is what moves the periodic `mcu_clock_poll` off the closed
        // session and releases the `Arc<Mcu>` the old `McuClock` holds.
        let reactor = self.printer.upgrade().map(|printer| printer.reactor());
        let sent_time = reactor
            .as_ref()
            .map(|reactor| reactor.monotonic())
            .unwrap_or(0.0);
        let uptime = seed_clock_base(&mcu).await;
        if let Some(reactor) = &reactor {
            self.install_clock(&mcu, reactor, sent_time, uptime);
        }
        Ok(mcu)
    }

    /// Build this session's clock estimate and put it on the chip.
    ///
    /// The primary (the bare `[mcu]`) defines the print-time origin; a
    /// secondary is shifted so the same print time maps to its own clock
    /// (`SecondarySync`, `klippy/clocksync.py:177-231`). Resources that
    /// convert print time to this MCU's clock read it through the chip.
    ///
    /// Both bring-ups run it — `connect` for the first session of a section,
    /// `reconnect` for the one that replaces a reset firmware — because the
    /// chip's clock slot is how the periodic `mcu_clock_poll` reaches its
    /// session: a slot left holding the pre-reset `McuClock` keeps querying the
    /// old connection forever (that clock keeps its own `Arc<Mcu>`, so the old
    /// session is never dropped either).
    ///
    /// `sent_time` is the reactor time just before `uptime` was read, so the
    /// seed anchors the regression on that round trip (`seed_clock_base`).
    fn install_clock(
        &self,
        mcu: &Arc<Mcu>,
        reactor: &Arc<dyn Reactor>,
        sent_time: f64,
        uptime: Option<u64>,
    ) {
        let clock = Arc::new(McuClock::new(Arc::clone(mcu), Arc::clone(reactor)));
        let seeded = uptime.is_some();
        if let Some(clock64) = uptime {
            clock.seed(sent_time, clock64 as i64);
        }
        let now = reactor.monotonic();
        let offset = if self.name() == "mcu" {
            0.0
        } else {
            self.printer
                .upgrade()
                .and_then(|printer| printer.lookup_object_as::<McuObject>("mcu"))
                .and_then(|primary| primary.clock())
                .map(|main| main.estimated_print_time(now) - clock.estimated_print_time(now))
                .unwrap_or(0.0)
        };
        self.chip.set_clock(Arc::clone(&clock), offset);
        // A secondary's crystals drift against the primary's; upstream
        // realigns it every `stats` (`SecondarySync.calibrate_clock`),
        // so a timer does it here.
        if self.name() != "mcu" {
            *self
                .secondary_sync
                .lock()
                .unwrap_or_else(|p| p.into_inner()) =
                Some(SecondarySync::new(clock.estimator().mcu_freq()));
            // Registered once per object: a reconnect installs the next
            // session's clock on the same object, and a second timer would
            // double the recalibration next to the first (whose handle the
            // slot would then have lost).
            if self
                .recalibrate_timer
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .is_none()
            {
                let printer = self.printer.clone();
                let identifier = self.section.identifier();
                let handle = reactor.register_timer_named(
                    "mcu_recalibrate",
                    Box::new(move |eventtime| {
                        let printer = printer.upgrade()?;
                        if let Some(object) = printer.lookup_object_as::<McuObject>(&identifier) {
                            object.recalibrate();
                        }
                        Some(eventtime + RECALIBRATE_INTERVAL)
                    }),
                    reactor.monotonic() + RECALIBRATE_INTERVAL,
                );
                *self
                    .recalibrate_timer
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = Some(handle);
            }
        }
        // The periodic `get_clock` that keeps the estimate fed — for
        // every MCU, primary and secondary alike (`mcu_recalibrate`
        // only reads the estimators, it never queries). Registered
        // only once the seed is in, so a sample always corrects a base
        // instead of starting an unseeded regression; before identify
        // there is no clock to poll at all. `register_clock_poll` keeps
        // one timer per object, so the reconnect's install is a no-op for it.
        if seeded {
            self.register_clock_poll(reactor.as_ref());
        }
    }

    /// Whether this bring-up follows a `firmware_restart`.
    ///
    /// Only then is the firmware itself reset; a first start and a plain
    /// `restart` reconnect without touching it (upstream keys the same decision
    /// on `start_reason`, `klippy/mcu.py:682-683`).
    fn is_firmware_restart(&self) -> bool {
        self.printer
            .upgrade()
            .and_then(|printer| printer.start_reason())
            .as_deref()
            == Some("firmware_restart")
    }

    /// Report that `rpi_usb` cannot reset this MCU's firmware, and use `command`
    /// for it from now on.
    ///
    /// `observed` is what was seen, and it is one of four things: the hub reports
    /// no port power switching at all (found before anything is switched), the
    /// switch itself failed, the board answered from an older session, or it still
    /// carried its configuration — a board that rebooted does neither. The first
    /// needs no attempt and the rest are attempts that did not work; in every case
    /// repeating the switch would keep `firmware_restart` from ever working, so the
    /// method becomes `command` — upstream's generic path, which resets the
    /// firmware through its own `config_reset` (`mcu/config.rs`).
    ///
    /// The replacement is recorded on the printer, not here: a restart re-reads the
    /// config file and rebuilds this object, and the config file is not written
    /// to (see [`Printer::override_config`]). Printed when it happens — the
    /// fallback is what keeps it from happening again.
    fn usb_reset_unusable(&self, config: &McuConfig, observed: &str) {
        warn!(
            "MCU '{}': restart_method 'rpi_usb' will not reset this firmware ({}); \
             switching this MCU to 'command' in memory — the config file is \
             unchanged",
            config.name, observed
        );
        if let Some(printer) = self.printer.upgrade() {
            printer.override_config(
                &self.section.identifier(),
                "restart_method",
                ConfigValue::Single(McuRestartMethod::Command.as_str().to_string()),
            );
        }
    }

    /// Watch for a firmware stop **while the connection is still being brought
    /// up**, and keep the reason without reporting a stop.
    ///
    /// Upstream binds its shutdown handlers this early — in `_mcu_identify`
    /// (`klippy/mcu.py:880-882`), before it sends the configuration — so the
    /// reason a stopped board gives is read instead of discarded. What upstream
    /// does not have to split here is the reporting: this bring-up is the one
    /// that *clears* a stopped or differently configured firmware
    /// (`mcu/config.rs`), so a stop that arrives on the way through is not a
    /// spontaneous one and the handlers
    /// [`McuObject::bind_shutdown`] binds would report the host's own reset as a
    /// printer shutdown. Only the reason is kept; the stop is reported after the
    /// handshake, when a stop does mean the machine should go down.
    ///
    /// A firmware that reports a stop here answers nothing else: it refuses
    /// every command that is not marked as running while stopped
    /// (`src/command.c:346-349`), so the handshake would sit out its five-second
    /// response timeout for an answer that is never coming. Dropping the
    /// in-flight calls ([`Mcu::abort_pending_calls`]) is what returns them at
    /// once.
    ///
    /// [`McuObject::bind_shutdown`] replaces both handlers once the handshake is
    /// done; until then every start of a session binds this pair again.
    fn watch_connect_shutdown(&self, mcu: &Arc<Mcu>) -> Result<(), McuError> {
        // A `Weak` because the handler outlives this call and is held by the
        // session's own event table: a strong handle would make
        // `Mcu -> events -> handler -> Mcu` a cycle that only the teardown's
        // `clear_events` breaks (see `McuEvents::clear`).
        let session = Arc::downgrade(mcu);
        let board = self.name().to_string();

        if mcu.has_message(Shutdown::NAME) {
            let slot = Arc::clone(&self.connect_shutdown);
            let board = board.clone();
            let session = session.clone();
            mcu.bind_event::<Shutdown, _>(move |event| {
                note_connect_shutdown(&slot, &board, &event.reason);
                abort_connect_calls(&session);
            })?;
        }
        if mcu.has_message(IsShutdown::NAME) {
            let slot = Arc::clone(&self.connect_shutdown);
            mcu.bind_event::<IsShutdown, _>(move |event| {
                note_connect_shutdown(&slot, &board, &event.reason);
                abort_connect_calls(&session);
            })?;
        }
        Ok(())
    }

    /// Forget what the previous session reported about stopping.
    ///
    /// Called where a session starts (connect, and the reopen after a
    /// firmware reset): one session's reason is not the next one's, and the old
    /// connection's dying report must not make a healthy bring-up fail with it.
    fn forget_connect_shutdown(&self) {
        *self
            .connect_shutdown
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = None;
    }

    /// The reason the firmware stopped while this connection was connecting.
    fn connect_shutdown(&self) -> Option<String> {
        self.connect_shutdown
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// A failed bring-up, reported with the firmware's own reason when it gave
    /// one.
    ///
    /// Without this, a board that stopped during the handshake reads as
    /// `timeout: no response for config within 5s` — the wire's account of what
    /// happened, not why it happened. The recorded reason is the one thing the
    /// connection got out of that board, so it is what the bring-up fails with;
    /// with no reason recorded the error is passed through unchanged.
    fn connect_failure(&self, err: KlippyError) -> KlippyError {
        match self.connect_shutdown() {
            Some(reason) => KlippyError::Connection(format!(
                "MCU '{}' shutdown during connect: {reason}",
                self.name()
            )),
            None => err,
        }
    }

    /// Whether a failed handshake round is worth another in-place firmware
    /// reset, with `resets` of them already done.
    ///
    /// Two failures send the bring-up back to `reset`:
    ///
    /// - [`McuError::ResetRequired`]: the firmware's only way out of its
    ///   configuration is to reboot itself (`mcu/config.rs`).
    /// - a stop this session reported on the way through
    ///   ([`McuObject::connect_shutdown`]): the firmware came up stopped — a
    ///   clock fault it hit while starting, on the real board — so it refuses
    ///   every configuration command and no further handshake round on this
    ///   session will finish. A fresh reboot is the way out.
    ///
    /// Both are capped by [`MAX_IN_PLACE_RESETS`]; past that the bring-up
    /// reports the failure it holds instead of rebooting forever.
    fn should_reset_and_retry(&self, err: &McuError, resets: u32) -> bool {
        if resets >= MAX_IN_PLACE_RESETS {
            return false;
        }
        matches!(err, McuError::ResetRequired) || self.connect_shutdown().is_some()
    }

    /// The bring-up's failure once the handshake has run out of in-place
    /// resets.
    ///
    /// A `ResetRequired` that outlived every reset means the firmware kept
    /// carrying its configuration through them — say that, rather than the
    /// terse "must be reset" the retry already acted on. Everything else goes
    /// through [`McuObject::connect_failure`], so a session that came up
    /// stopped fails with the firmware's own reason instead of a response
    /// timeout.
    fn handshake_failure(&self, err: &McuError) -> KlippyError {
        let err = match err {
            McuError::ResetRequired => KlippyError::Connection(format!(
                "MCU '{}' still carries a configuration after a 'reset'",
                self.name()
            )),
            other => KlippyError::Connection(other.to_string()),
        };
        self.connect_failure(err)
    }

    /// Run the configuration handshake, resetting the firmware in place when it
    /// cannot accept the configuration yet.
    ///
    /// Split out of [`McuObject::connect`] so the retry is callable against a
    /// mock first session: a `test:` section's fake starts a *working* firmware,
    /// so the one thing `connect` cannot be scripted into is a handshake that
    /// fails and then succeeds on the connection a reset brings back. Handed the
    /// connection it starts from, this is the same loop either way.
    ///
    /// The point is `FIRMWARE_RESTART` on the `command` method, which already
    /// sent the firmware's own `reset` on the live connection before the parts
    /// came down (`before_firmware_restart`; upstream's `_restart_via_command`,
    /// `klippy/mcu.py:730-747`). What is left here is the firmware whose *only*
    /// reset is `reset` and that still carries a configuration: no `config_reset`
    /// to clear it in place, so it has to be rebooted and re-identified — with
    /// the in-place retries this loop owns.
    async fn handshake_with_in_place_resets(
        &self,
        config: &McuConfig,
        mut mcu: Arc<Mcu>,
        mut built: BuiltConfig,
    ) -> Result<(Arc<Mcu>, Configured), KlippyError> {
        let mut resets = 0u32;
        loop {
            // `test:` serves the corpus against the fake, whose clock only
            // moves with wall time while the corpus' motion is virtual: a
            // scheduling gate there degenerates into wall-clock serialisation
            // (the estimate can never lead the stream it waits on — measured in
            // C4), so the gates are opened for this transport only. Every
            // connection passes through here once — and again after
            // `reconnect` — so one call covers each `Mcu` this section ever
            // gets. Production links never take this branch, and `Mcu::for_test`
            // (FrameMock) never reaches this object outside the tests below.
            // (`object.rs` is touched because it is the only place that sees
            // both the section's interface key and every live `Mcu`.)
            // `KLIPPERX_KEEP_GATES=1` keeps the gates shut on the fake — a
            // debug hatch for reproducing gate-vs-generation hazards against one
            // corpus case (pair it with `KLIPPERX_UPSTREAM_FILTER=<name>`; the C4
            // iqex follow-up reproduces in ~1.5 s that way).
            if self.section.parameters.contains_key("test")
                && std::env::var_os("KLIPPERX_KEEP_GATES").is_none()
            {
                mcu.open_send_gates();
            }
            let err = match self
                .chip
                .config()
                .handshake(&mcu, &mut built, self.is_firmware_restart())
                .await
            {
                Ok(configured) => return Ok((mcu, configured)),
                Err(err) => err,
            };
            if !self.should_reset_and_retry(&err, resets) {
                return Err(self.handshake_failure(&err));
            }
            resets += 1;
            // The two ways in read differently on the console: a `ResetRequired`
            // is the firmware's own plan, while a stop is a reboot that did not
            // come up — worth a warning, because on the real board it is the
            // signal that the retry is doing its job.
            match self.connect_shutdown() {
                Some(reason) => warn!(
                    "MCU '{}': firmware is still shutdown after a reset ({reason}); \
                     resetting again",
                    config.name
                ),
                None => info!(
                    "MCU '{}': resetting the firmware with the 'reset' command",
                    config.name
                ),
            }
            // `reset_and_flush` reports what the old connection said on its way
            // out: a stop recorded there is still this bring-up's reason.
            if let Err(err) = reset_and_flush(&mcu).await {
                return Err(self.connect_failure(err));
            }
            // Let the rebooted firmware start before the port is reopened; the
            // reopen races its startup otherwise (`RESET_SETTLE`).
            tokio::time::sleep(RESET_SETTLE).await;
            mcu = self.reconnect(config, mcu).await?;
        }
    }

    /// Report a firmware shutdown, restart, or already-stopped state.
    ///
    /// The events carry the reason; the machine is what knows what a stop
    /// means, so the handler only hands it over. Bound **after** the
    /// configuration handshake, so that a reset this host performs while
    /// configuring does not look like a spontaneous stop (see
    /// `mcu/config.rs`, which clears a stopped or differently-configured
    /// firmware before sending the configuration).
    fn bind_shutdown(&self, mcu: &Mcu) -> Result<(), McuError> {
        let name = self.chip.name().to_string();

        if !mcu.has_message(Shutdown::NAME) {
            warn!(
                "MCU '{name}' does not report `shutdown`; a firmware stop will not be \
                 noticed, and a configuration reset falls back to a delay"
            );
        }

        if mcu.has_message(Shutdown::NAME) {
            let printer = self.printer.clone();
            let name = name.clone();
            let is_shutdown = Arc::clone(&self.is_shutdown);
            mcu.bind_event::<Shutdown, _>(move |event| {
                if is_shutdown.swap(true, Ordering::SeqCst) {
                    return;
                }
                let human = match event.clock {
                    Some(clock) => {
                        format!("MCU '{name}' shutdown: {} (clock {clock})", event.reason)
                    }
                    None => format!("MCU '{name}' shutdown: {}", event.reason),
                };
                report_mcu_shutdown(&printer, &name, &event.reason, "shutdown", &human);
            })?;
        }
        if mcu.has_message(IsShutdown::NAME) {
            let printer = self.printer.clone();
            let name = name.clone();
            let is_shutdown = Arc::clone(&self.is_shutdown);
            mcu.bind_event::<IsShutdown, _>(move |event| {
                is_shutdown.store(true, Ordering::SeqCst);
                let human = format!("MCU '{name}' is shutdown: {}", event.reason);
                report_mcu_shutdown(&printer, &name, &event.reason, "is_shutdown", &human);
            })?;
        }
        if mcu.has_message(Starting::NAME) {
            let printer = self.printer.clone();
            let name = name.clone();
            let is_shutdown = Arc::clone(&self.is_shutdown);
            mcu.bind_event::<Starting, _>(move |_| {
                // Upstream only treats a restart as fatal when the MCU was not
                // already stopped (`klippy/mcu.py:826-829`).
                if is_shutdown.load(Ordering::SeqCst) {
                    return;
                }
                report_shutdown(&printer, &format!("MCU '{name}' spontaneous restart"));
            })?;
        }
        Ok(())
    }
}

impl McuObject {
    /// Break the `Mcu → events → resource → Mcu` strong cycle.
    ///
    /// `Printer::teardown` calls this for every part *before* dropping them, so
    /// the cycle is broken even when the parts cannot be dropped (a cycle keeps
    /// `Drop` from ever running, and the leaked `Mcu` would park runtime
    /// shutdown); `Drop` calls it again as the backstop for other teardown
    /// paths.
    pub(crate) fn release_cycles(&self) {
        if let Some(handle) = self
            .recalibrate_timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .take()
        {
            handle.cancel();
        }
        if let Some(handle) = self
            .clock_poll_timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .take()
        {
            handle.cancel();
        }
        if let Some(mcu) = self.chip.mcu() {
            mcu.clear_events();
        }
    }
}

impl Drop for McuObject {
    fn drop(&mut self) {
        // Recycled with `Drop`: `Printer::teardown` calls it before the parts
        // are dropped (see `release_cycles`), which is what makes the release
        // unconditional when a cycle would keep this object alive.
        self.release_cycles();
    }
}

impl PrinterObject for McuObject {
    fn get_status(&self, _eventtime: f64) -> Value {
        let mut status = self
            .status
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        // `last_stats` appears only once a report has arrived, as upstream's
        // does (the key is added by its `stats()` collector).
        if let Some(last) = *self.last_stats.lock().unwrap_or_else(|p| p.into_inner()) {
            if let Value::Object(fields) = &mut status {
                fields.insert("last_stats".to_string(), last.to_json());
            }
        }
        status
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            // One session's shutdown is not the next one's: whatever a previous
            // bring-up recorded stays out of this one.
            self.forget_connect_shutdown();
            // The device is opened here, not at construction: that is the point
            // of two-phase construction. The section was already parsed once
            // (by the loader, which recorded its options for the undefined-option
            // check); parsing it again here is done on the printer's tracker so
            // a read that happens now is still recorded.
            let access = self
                .printer
                .upgrade()
                .map(|printer| printer.access_tracking())
                .unwrap_or_else(AccessTracking::shared);
            let wrapper = ConfigWrapper::new(&self.section, access);
            let mut config = McuConfig::new(&wrapper).map_err(KlippyError::Config)?;
            info!(
                "MCU '{}' restart method: {}",
                config.name,
                config.restart_method.as_str()
            );
            // Check at startup, not at the first restart, that an `rpi_usb` reset
            // will be able to switch this port's power (and say which udev rule
            // to install if it will not). A hub that reports no power switching
            // cannot switch it at all, so that check is also where `rpi_usb` is
            // dropped before it ever disconnects a board for nothing.
            if let Some(reason) = super::restart::check_usb_power(&config) {
                self.usb_reset_unusable(&config, &reason);
                config.restart_method = McuRestartMethod::Command;
            }
            // Remember what this connection resets with: `before_firmware_restart`
            // runs later, on the live connection, and only `command` resets there.
            *self
                .restart_method
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()) = config.restart_method.clone();
            // An `rpi_usb` board is only configured after a power cycle in this
            // session: ask for the restart instead of configuring a board that
            // may still be running an old configuration (upstream's
            // `check_restart_on_attach` / `check_restart_on_send_config`).
            if let Some(reason) =
                super::restart::restart_before_bringup(&config, self.is_firmware_restart())
            {
                info!(
                    "Attempting automated MCU '{}' restart: {reason}",
                    config.name
                );
                if let Some(printer) = self.printer.upgrade() {
                    printer.request_exit("firmware_restart");
                }
                return Err(KlippyError::Connection(format!(
                    "MCU '{}' needs a firmware restart before it can be configured",
                    config.name
                )));
            }
            // This bring-up is the one that resets the firmware itself, and
            // `rpi_usb` is the only method that does it by switching the port's
            // power — the one thing that can disconnect a board without
            // resetting it, and so the one thing worth checking afterwards.
            let usb_reset =
                self.is_firmware_restart() && config.restart_method == McuRestartMethod::RpiUsb;
            if self.is_firmware_restart() {
                if let Err(err) =
                    super::restart::reset_firmware(&config, &tokio::runtime::Handle::current())
                        .await
                {
                    if usb_reset {
                        self.usb_reset_unusable(&config, &err);
                    }
                    return Err(KlippyError::Internal(err));
                }
            }
            // A bring-up that follows a firmware restart finds the board
            // rebooting — `command` sent the reset on the live connection
            // (`before_firmware_restart`), and the physical methods just switched
            // it — so the port is retried until it comes back. A plain start
            // opens it once and reports the error, so a missing device is not
            // hidden behind a retry.
            let mcu = if self.is_firmware_restart() {
                match self.open_and_connect(&config).await {
                    Ok(mcu) => mcu,
                    Err(err) => {
                        if usb_reset {
                            self.usb_reset_unusable(&config, &err.to_string());
                        }
                        return Err(self.connect_failure(err));
                    }
                }
            } else {
                let interface = config.open().map_err(KlippyError::Connection)?;
                match Mcu::connect(&config.name, interface).await {
                    Ok(mcu) => mcu,
                    Err(err) => {
                        if usb_reset {
                            self.usb_reset_unusable(&config, &err.to_string());
                        }
                        return Err(self.connect_failure(KlippyError::Connection(err.to_string())));
                    }
                }
            };
            // Make the device reachable by resources before the configuration
            // is built; a resource's runtime methods need it.
            self.chip.attach(Arc::clone(&mcu));
            // The firmware may stop on the way through this connection — this is
            // the bring-up that clears a stopped board — so from here on its
            // `shutdown`/`is_shutdown` reports are read instead of discarded
            // (`watch_connect_shutdown`). They are not reported as a printer
            // shutdown until the handshake is done (`bind_shutdown`, below).
            self.watch_connect_shutdown(&mcu)
                .map_err(|err| KlippyError::Internal(err.to_string()))?;
            // One clock read, so an unclocked resource can estimate "now"
            // (`Mcu::estimated_clock`). A firmware without `get_uptime` simply
            // has no estimate.
            let reactor = self.printer.upgrade().map(|printer| printer.reactor());
            let sent_time = reactor
                .as_ref()
                .map(|reactor| reactor.monotonic())
                .unwrap_or(0.0);
            let uptime = seed_clock_base(&mcu).await;
            if let Some(reactor) = &reactor {
                self.install_clock(&mcu, reactor, sent_time, uptime);
            }
            // Identify installed the dictionary; reserve the pins the firmware
            // owns before anything resolves one. A conflict here is a
            // configuration problem (upstream's `pins.error`, caught as a
            // config error), not an internal failure.
            self.reserve_pins(&mcu)
                .map_err(|err| KlippyError::Config(ConfigError::new(err.to_string())))?;

            // `FIRMWARE_RESTART` on the `command` method sends the firmware's own
            // `reset` on the live connection, before the parts come down: it is
            // what lets this connection be the only one, instead of identifying
            // the running firmware just to tell it to reboot
            // (`before_firmware_restart`; upstream's `_restart_via_command`,
            // `klippy/mcu.py:730-747`). What is left for the handshake below is
            // the firmware whose *only* reset is `reset` and that still carries a
            // configuration: no `config_reset` to clear it in place, so it has to
            // be rebooted and re-identified here — with the in-place retries
            // [`McuObject::handshake_with_in_place_resets`] owns.

            // The accumulated configuration is encoded once, and the handshake
            // can then be retried on a fresh connection if the firmware has to
            // reboot to accept it (`mcu/config.rs`).
            // Work that needs the dictionary *and* a round-trip to the firmware
            // before the configuration is frozen (the `debug_read` calibration
            // reads in `temperature_mcu`). The dictionary is installed and the
            // device is attached by now, and this call is awaited, so the
            // callbacks may `call_msg` on the live connection.
            self.chip
                .config()
                .run_pre_build(&mcu)
                .await
                .map_err(|err| match err {
                    McuError::Config(message) => KlippyError::Config(ConfigError::new(message)),
                    other => KlippyError::Internal(other.to_string()),
                })?;
            let built = self.chip.config().build(&mcu).map_err(|err| match err {
                // A config callback resolves pins/buses against the
                // dictionary; a bad pin is a config problem, not klippy's.
                McuError::Config(message) => KlippyError::Config(ConfigError::new(message)),
                other => KlippyError::Internal(other.to_string()),
            })?;
            let (mcu, configured) = self
                .handshake_with_in_place_resets(&config, mcu, built)
                .await?;
            // A board that just rebooted has a sequence that starts over and no
            // configuration: the transport had nothing to take over
            // (`Mcu::took_over_session`), and the firmware was neither configured
            // nor stopped when it answered. Anything else was only disconnected,
            // and the port switch did not do its job.
            if usb_reset && (mcu.took_over_session() || configured.already_running) {
                self.usb_reset_unusable(
                    &config,
                    "the firmware was still in the session that came before this connection",
                );
            }
            // Only now does a firmware shutdown mean something the machine
            // should report: the configuration handshake is done, so nothing
            // this host sent is still in flight.
            self.bind_shutdown(&mcu)
                .map_err(|err| KlippyError::Internal(err.to_string()))?;
            // The firmware sends periodic `stats` reports (id=-12) with
            // scheduler timing; keep the latest as `last_stats` (and log it) so
            // `objects/query` can report the load.
            let freq = mcu
                .dictionary()
                .and_then(|dictionary| dictionary.constant_f64("CLOCK_FREQ"));
            let sumsq_base = mcu
                .dictionary()
                .and_then(|dictionary| dictionary.constant_f64("STATS_SUMSQ_BASE"));
            match (freq, sumsq_base) {
                (Some(freq), Some(sumsq_base)) => {
                    register_stats(&mcu, freq, sumsq_base, Arc::clone(&self.last_stats))
                        .map_err(|err| KlippyError::Internal(err.to_string()))?
                }
                _ => warn!(
                    "MCU '{}' has no clock statistics constants; last_stats stays empty",
                    self.chip.name()
                ),
            }
            self.set_status(&mcu);
            Ok(())
        })
    }

    fn before_firmware_restart<'a>(&'a self) -> RestartFuture<'a> {
        Box::pin(async move {
            // Only `command` resets on the live connection; the physical methods
            // need the port closed and run from `connect` (`mcu/restart.rs`).
            let command = {
                let method = self
                    .restart_method
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner());
                *method == McuRestartMethod::Command
            };
            if !command {
                return;
            }
            let Some(mcu) = self.chip.mcu() else {
                return;
            };
            if !mcu.has_message(Reset::NAME) {
                return;
            }
            info!(
                "MCU '{}': resetting the firmware with the 'reset' command",
                self.chip.name()
            );
            if let Err(err) = reset_and_flush(&mcu).await {
                warn!(
                    "MCU '{}': could not send the 'reset' command ({err})",
                    self.chip.name()
                );
            }
        })
    }
}

/// Send `reset` and wait for it to leave the send queue.
///
/// The firmware reboots on receipt, so the connection is about to die;
/// flushing first is what keeps the command from being dropped with the
/// transport. The flush itself often reports the transport closing as the board
/// goes away — the command was still written first, so that is not an error.
async fn reset_and_flush(mcu: &Mcu) -> Result<(), KlippyError> {
    mcu.send_msg(&Reset)
        .map_err(|err| KlippyError::Internal(err.to_string()))?;
    let _ = mcu.flush(RESET_FLUSH_TIMEOUT).await;
    Ok(())
}

/// Seed the firmware-clock estimate from one `get_uptime`.
///
/// Upstream's clock sync reads the clock at connect too
/// (`MCUConnectHelper._attach` → `clocksync.connect`); this host keeps only the
/// base point, for resources that need a "now" clock without consulting the
/// print-time estimate. The 64-bit `get_uptime` is used rather than `get_clock`
/// because the 32-bit counter wraps every few minutes at a typical
/// `CLOCK_FREQ`. A firmware that does not publish it simply has no estimate.
async fn seed_clock_base(mcu: &Mcu) -> Option<u64> {
    if !mcu.has_message(GetUptime::NAME) {
        return None;
    }
    match mcu
        .call_msg::<GetUptime, Uptime>(&GetUptime, CLOCK_BASE_TIMEOUT)
        .await
    {
        Ok(uptime) => {
            mcu.set_clock_base(uptime.clock64());
            Some(uptime.clock64())
        }
        Err(err) => {
            warn!(
                "MCU '{}': could not read the clock for a time estimate: {err}",
                mcu.name()
            );
            None
        }
    }
}

/// Keep the first stop a connection reported while it was connecting.
///
/// First one wins: a stopped firmware repeats its one reason in every
/// `is_shutdown` it sends, so a later report only says the same thing again. The
/// line is the stop's only trace until the handshake is over — the handlers that
/// report it to the machine are bound afterwards on purpose
/// ([`McuObject::bind_shutdown`]).
fn note_connect_shutdown(slot: &Mutex<Option<String>>, board: &str, reason: &str) {
    let mut slot = slot.lock().unwrap_or_else(|poison| poison.into_inner());
    if slot.is_some() {
        return;
    }
    *slot = Some(reason.to_string());
    warn!("MCU '{board}': firmware shutdown during connect: {reason}");
}

/// Drop whatever the stopped session is still being waited on for.
///
/// Called from the receive task's message callback, so the session is reached
/// weakly ([`McuObject::watch_connect_shutdown`]); a session that is already
/// gone has dropped those calls along with itself.
fn abort_connect_calls(session: &Weak<Mcu>) {
    if let Some(mcu) = session.upgrade() {
        mcu.abort_pending_calls();
    }
}

/// Log a firmware stop and put the machine into its shutdown state.
fn report_shutdown(printer: &Weak<Printer>, msg: &str) {
    warn!("{msg}");
    if let Some(printer) = printer.upgrade() {
        printer.invoke_shutdown(msg);
    }
}

/// Report an MCU-halted stop the way upstream's `_handle_shutdown` does
/// (`klippy/mcu.py:813-825`): the generic message `"MCU shutdown"` plus the
/// details `error_mcu` needs to build the user-facing text.
fn report_mcu_shutdown(
    printer: &Weak<Printer>,
    name: &str,
    reason: &str,
    event_type: &str,
    human: &str,
) {
    warn!("{human}");
    if let Some(printer) = printer.upgrade() {
        printer.invoke_shutdown_with(
            "MCU shutdown",
            HashMap::from([
                ("reason".to_string(), json!(reason)),
                ("mcu".to_string(), json!(name)),
                ("event_type".to_string(), json!(event_type)),
            ]),
        );
    }
}

/// The status upstream's `MCUStatsHelper._mcu_identify` fills
/// (`klippy/mcu.py:938-948`).
fn status_from(dictionary: &Dictionary) -> Value {
    let raw = dictionary.raw();
    let constants: Map<String, Value> = dictionary
        .constants()
        .iter()
        .map(|(name, value)| (name.clone(), value.clone()))
        .collect();
    json!({
        "mcu_version": raw.get("version").cloned().unwrap_or(Value::Null),
        "mcu_build_versions": raw.get("build_versions").cloned().unwrap_or(Value::Null),
        "mcu_constants": constants,
    })
}

/// Upstream's `load_config` for the `mcu` section.
///
/// Returns the object; the loader registers it under the section identifier.
/// The printer is part of the factory signature for object types that wire
/// themselves up as they are built (they register event handlers); an MCU
/// connects through [`PrinterObject::connect`] instead, so it does not use it.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    // Parse the section now so its options are recorded for the undefined-option
    // check: the check runs at the end of the load walk, before anything
    // connects, and an MCU reads its section at connect time. The parse is pure;
    // the device is still only opened by `connect`.
    McuConfig::new(config)?;
    // The first `[mcu]` section brings the `error_mcu` module with it, as
    // upstream's `MCU.__init__` does (`klippy/mcu.py:1159`); the rest find it
    // already there. It has to exist before any MCU can fail.
    crate::core::klippy::extras::error_mcu::ensure(printer)?;
    let object = Arc::new(
        McuObject::new(config.section().clone(), printer)
            .map_err(|err| ConfigError::new(err.to_string()))?,
    );
    // A host shutdown stops the firmware too (upstream registers the same
    // handler in `MCU.__init__`, `klippy/mcu.py:800`). Registered here
    // rather than in `connect` because the handler is `'static` and needs a
    // handle to the object the registry owns; a `Weak` avoids keeping it alive.
    let weak = Arc::downgrade(&object);
    printer.register_event_handler(
        KlippyEvent::KlippyShutdown,
        Box::new(move |_| {
            if let Some(object) = weak.upgrade() {
                object.on_host_shutdown();
            }
        }),
    );
    Ok(object)
}

/// Upstream's `load_config_prefix` for `[mcu <name>]`.
///
/// The same object today. The two entry points differ upstream only by the clock
/// a secondary MCU synchronizes to (`klippy/mcu.py:1245`), which arrives with the
/// clock layer.
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
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::{
        FrameMock, FrameRecorder, MappingEntry,
    };
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::msg::proto::Payload;
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::ManualReactor;
    use tokio::time::Duration;

    fn section(sub: Option<&str>) -> ConfigSection {
        ConfigSection::new("mcu", sub)
    }

    /// A printer with the `pins` object, as the loader leaves it before any
    /// section is loaded.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        printer
    }

    /// An MCU attached to `object`'s chip, with `reset` in its dictionary.
    fn attached(object: &McuObject, mappings: Vec<MappingEntry>) -> FrameRecorder {
        let device = FrameMock::new(mappings);
        let recorder = device.recorder();
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::new(device)));
        mcu.install_dictionary(Dictionary::from_json(json!({"commands": {"reset": 9}})).unwrap())
            .unwrap();
        object.chip.attach(Arc::clone(&mcu));
        recorder
    }

    #[tokio::test]
    async fn test_a_firmware_restart_resets_on_the_live_connection() {
        // The `command` reset has to go out on the connection that is already up:
        // by the time the parts are torn down there is no transport left to send
        // it on, and reconnecting only to reset is what made a restart identify
        // the firmware twice. `before_firmware_restart` is the hook the host calls
        // while the parts are still up (`klippy.rs`).
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        let recorder = attached(
            &object,
            vec![MappingEntry {
                input: Frame::new(0, vec![9]),
                outputs: vec![],
            }],
        );

        object.before_firmware_restart().await;

        let sent = recorder.frames();
        assert_eq!(sent.len(), 1, "one reset command, got {sent:?}");
        assert_eq!(sent[0].payload(), &[9]);
    }

    #[tokio::test]
    async fn test_a_physical_restart_method_does_not_reset_on_the_live_connection() {
        // `arduino` / `cheetah` / `rpi_usb` reset the firmware on the **closed**
        // port, from `connect` (`mcu/restart.rs`). Sending `reset` here as well
        // would reboot the board twice.
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        *object
            .restart_method
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = McuRestartMethod::RpiUsb;
        let recorder = attached(&object, vec![]);

        object.before_firmware_restart().await;

        assert!(recorder.frames().is_empty());
    }

    #[test]
    fn test_a_usb_reset_that_did_not_reset_the_firmware_switches_to_command() {
        // An `rpi_usb` reset that left the firmware running is worth acting on:
        // the switch is recorded as `command` for this MCU, in memory. The
        // section is the serial one, because that is the only transport the
        // option applies to.
        let printer = printer();
        let mut section = section(None);
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/dev/not-opened-yet".to_string()),
        );
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("rpi_usb".to_string()),
        );
        let object = McuObject::new(section, &printer).unwrap();
        let config = McuConfig::new(&ConfigWrapper::untracked(&object.section)).unwrap();
        assert_eq!(config.restart_method, McuRestartMethod::RpiUsb);
        assert!(printer.overrides_for("mcu").is_empty());

        object.usb_reset_unusable(
            &config,
            "the firmware answered from an older session (timeout: no response)",
        );

        assert_eq!(
            printer.overrides_for("mcu"),
            vec![(
                "restart_method".to_string(),
                ConfigValue::Single("command".to_string())
            )]
        );
        // Which is what the loader will hand back the next time this section is
        // read: the in-memory config now says `command`.
        let next = printer.overrides_for(&object.section.identifier());
        let mut section = object.section.clone();
        for (option, value) in next {
            section.parameters.insert(option, value);
        }
        assert_eq!(
            McuConfig::new(&ConfigWrapper::untracked(&section))
                .unwrap()
                .restart_method,
            McuRestartMethod::Command
        );
    }

    /// An MCU object for `sub`, over such a printer.
    fn object(sub: Option<&str>) -> McuObject {
        McuObject::new(section(sub), &printer()).unwrap()
    }

    #[test]
    fn test_an_mcu_registers_itself_as_a_chip() {
        // A pin description may name this MCU from the moment the config file
        // mentions it, so the chip is declared when the object is built.
        let printer = printer();
        McuObject::new(section(Some("zboard")), &printer).unwrap();

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .unwrap();
        assert_eq!(pins.chips(), ["zboard"]);
    }

    #[test]
    fn test_two_mcus_cannot_claim_the_same_chip_name() {
        let printer = printer();
        McuObject::new(section(None), &printer).unwrap();

        let err = match McuObject::new(section(None), &printer) {
            Ok(_) => panic!("a second MCU claimed the same chip name"),
            Err(err) => err,
        };

        assert!(matches!(err, PinError::DuplicateChip(_)), "{err:?}");
    }

    #[tokio::test]
    async fn test_reserve_pins_marks_what_the_firmware_owns() {
        // The printer is held for the test: `McuChip` reaches the pin registry
        // weakly, and the registry is the printer's.
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(vec![])));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "config": {
                    "CLOCK_FREQ": 20000000,
                    "RESERVE_PINS_uart0": "PA0,PA1",
                }
            }))
            .unwrap(),
        )
        .unwrap();

        object.reserve_pins(&mcu).unwrap();

        // The reserved pins cannot be resolved; a free one can.
        assert!(object.chip.pins().resolve_pin("mcu", "PA0").is_err());
        assert!(object.chip.pins().resolve_pin("mcu", "PA2").is_ok());
    }

    #[tokio::test]
    async fn test_a_firmware_shutdown_stops_the_printer() {
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();

        // The firmware answers the request with an unsolicited `shutdown`, the
        // way it does when one of its shutdown handlers runs.
        let mut request = Payload::new();
        request.push_i16(4).unwrap(); // get_uptime
        let mut shutdown = Payload::new();
        shutdown.push_i16(20).unwrap(); // shutdown
        shutdown.push_u32(1234).unwrap(); // clock
        shutdown.push_u16(0).unwrap(); // static_string_id 0

        let device = FrameMock::new(vec![MappingEntry {
            input: Frame::new(0, request.into_raw()),
            outputs: vec![Frame::new(0, shutdown.into_raw())],
        }]);
        let mcu = Mcu::for_test("mcu", Interface::new(device));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "commands": {"get_uptime": 4},
                "responses": {"shutdown clock=%u static_string_id=%hu": 20},
                "enumerations": {"static_string_id": {"Move queue overflow": 0}}
            }))
            .unwrap(),
        )
        .unwrap();

        object.bind_shutdown(&mcu).unwrap();
        // The `error_mcu` module enriches the terse "MCU shutdown" message; the
        // MCU factory brings it in, and this test does too.
        crate::core::klippy::extras::error_mcu::ensure(&printer).unwrap();
        mcu.send("get_uptime", &[]).unwrap();
        // Let the receive task decode and dispatch the shutdown frame.
        tokio::time::sleep(Duration::from_millis(20)).await;

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Shutdown);
        assert!(
            state
                .message
                .starts_with("MCU 'mcu' shutdown: Move queue overflow"),
            "{}",
            state.message
        );
        assert!(
            state.message.contains("Printer is shutdown"),
            "{}",
            state.message
        );
    }

    #[test]
    fn test_the_config_builder_exists_before_the_device_does() {
        // Resources add their `config_*` commands while the config file is
        // loaded — long before anything connects — so the builder has to be
        // usable from the object as it is built.
        let object = object(None);

        assert_eq!(object.config().create_oid().unwrap(), 0);
        assert!(!object.config().is_finalized());
    }

    #[test]
    fn test_the_mcus_own_name_is_the_section_without_the_id() {
        // `[mcu]` is "mcu"; `[mcu zboard]` is "zboard" — the registry key keeps
        // the id (`mcu zboard`), the name does not (`klippy/mcu.py:1151-1153`).
        assert_eq!(object(None).name(), "mcu");
        assert_eq!(object(Some("zboard")).name(), "zboard");
    }

    #[test]
    fn test_status_is_empty_until_connected() {
        let object = object(None);

        assert_eq!(object.get_status(0.0), json!({}));
    }

    #[tokio::test]
    async fn test_a_connected_mcu_reports_its_identify_snapshot() {
        let object = object(None);
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::new(FrameMock::new(vec![]))));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "version": "v0.12.0-1-g1234567",
                "build_versions": "gcc: 12.3.1",
                "config": {"CLOCK_FREQ": 20000000},
            }))
            .unwrap(),
        )
        .unwrap();

        object.set_status(&mcu);

        assert_eq!(
            object.get_status(0.0),
            json!({
                "mcu_version": "v0.12.0-1-g1234567",
                "mcu_build_versions": "gcc: 12.3.1",
                "mcu_constants": {"CLOCK_FREQ": 20000000},
            })
        );
    }

    #[tokio::test]
    async fn test_connecting_a_section_with_no_usable_interface_fails() {
        let mut section = section(None);
        section.parameters.insert(
            "serial".to_string(),
            crate::core::klippy::config::ConfigValue::Single("/dev/not-a-serial-port".to_string()),
        );
        let object = McuObject::new(section, &printer()).unwrap();

        let err = object.connect().await.unwrap_err();

        // The section's own parser names the port; the object does not have to.
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
    }

    #[test]
    fn test_the_loader_builds_an_object_from_the_section() {
        let printer = printer();
        // The loader parses the section (to record its options); the device is
        // still only opened at connect, so a serial path that does not exist is
        // fine here.
        let mut section = section(Some("zboard"));
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/dev/not-opened-yet".to_string()),
        );

        let object = load_config(&ConfigWrapper::untracked(&section), &printer).unwrap();

        assert_eq!(object.get_status(0.0), json!({}));
    }

    /// An MCU attached to `object`'s chip whose dictionary has `emergency_stop`.
    ///
    /// The device is scripted with `mappings` because [`FrameMock`] only
    /// records a frame it was told to expect: with no mapping, a sent frame is
    /// refused and never reaches the recorder.
    fn attached_with_estop(
        object: &McuObject,
        mappings: Vec<MappingEntry>,
    ) -> (Arc<Mcu>, FrameRecorder) {
        let device = FrameMock::new(mappings);
        let recorder = device.recorder();
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::new(device)));
        mcu.install_dictionary(
            Dictionary::from_json(json!({"commands": {"emergency_stop": 3}})).unwrap(),
        )
        .unwrap();
        object.chip.attach(Arc::clone(&mcu));
        (mcu, recorder)
    }

    /// The one frame a first `emergency_stop` puts on the wire: seq 0, id 3.
    fn estop_mapping() -> Vec<MappingEntry> {
        vec![MappingEntry {
            input: Frame::new(0, vec![3]),
            outputs: vec![],
        }]
    }

    #[tokio::test]
    async fn test_a_host_shutdown_sends_emergency_stop_to_the_firmware() {
        // Upstream's `MCU._shutdown` (`klippy/mcu.py:889-893`): a host shutdown
        // stops the firmware too, so it cannot keep executing queued work while
        // the host is gone.
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        let (mcu, recorder) = attached_with_estop(&object, estop_mapping());

        object.on_host_shutdown();
        mcu.flush(Duration::from_secs(1)).await.unwrap();

        let sent = recorder.frames();
        assert_eq!(sent.len(), 1, "one emergency_stop, got {sent:?}");
        assert_eq!(sent[0].payload(), &[3]);
        assert!(object.is_shutdown(), "the local flag stands after the stop");
    }

    #[tokio::test]
    async fn test_a_host_shutdown_after_a_firmware_stop_does_not_echo_it_back() {
        // The firmware reported the stop itself (or the host already forced
        // one); sending `emergency_stop` again would be pointless.
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        let (mcu, recorder) = attached_with_estop(&object, estop_mapping());
        object.is_shutdown.store(true, Ordering::SeqCst);

        object.on_host_shutdown();
        mcu.flush(Duration::from_secs(1)).await.unwrap();

        assert!(recorder.frames().is_empty(), "nothing to stop");
    }

    #[tokio::test]
    async fn test_load_config_registers_the_host_shutdown_handler() {
        let printer = printer();
        let mut section = section(None);
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/dev/not-opened-yet".to_string()),
        );
        let object = load_config(&ConfigWrapper::untracked(&section), &printer).unwrap();
        // The loader does this in production; the factory only returns the
        // object, and the handler's `Weak` needs the registry to hold it.
        printer.add_object("mcu", Arc::clone(&object)).unwrap();
        let mcu_object = printer
            .lookup_object_as::<McuObject>("mcu")
            .expect("the registered object");
        let (mcu, recorder) = attached_with_estop(&mcu_object, estop_mapping());

        printer.send_event(&KlippyEvent::KlippyShutdown);
        mcu.flush(Duration::from_secs(1)).await.unwrap();

        assert_eq!(recorder.frames().len(), 1);
    }

    #[test]
    fn test_last_stats_appears_in_the_status_once_reported() {
        // Upstream's `MCUStatsHelper` puts the running numbers in `last_stats`
        // (`klippy/mcu.py:974-975`); before any report the key is absent.
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        assert!(object.get_status(0.0).get("last_stats").is_none());

        *object.last_stats.lock().unwrap_or_else(|p| p.into_inner()) = Some(LastStats {
            mcu_tick_avg: 1.5e-6,
            mcu_tick_stddev: 2.5e-7,
            mcu_tick_awake: 0.25,
        });

        let status = object.get_status(0.0);
        assert_eq!(status["last_stats"]["mcu_tick_awake"], 0.25);
        assert_eq!(status["last_stats"]["mcu_tick_avg"], 1.5e-6);
    }

    // -----------------------------------------------------------------------
    // The periodic clock poll (`mcu_clock_poll`)
    // -----------------------------------------------------------------------

    /// The dictionary of a firmware that answers `get_clock`, as the clock
    /// module's own tests script it.
    fn polling_dictionary() -> Value {
        json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18},
            "config": {"CLOCK_FREQ": 20000000},
        })
    }

    /// The one scripted `get_clock` → `clock <value>` exchange.
    fn clock_exchange(clock: u32) -> Vec<MappingEntry> {
        let mut response = Payload::new();
        response.push_u8(18).unwrap(); // response id, `clock clock=%u`
        response.push_u32(clock).unwrap();
        vec![MappingEntry {
            input: Frame::new(0, vec![5]), // `get_clock`, no parameters
            outputs: vec![Frame::new(0, response.into_raw())],
        }]
    }

    /// Wait up to a second for `want` recorded frames, then hand them over.
    async fn wait_for_frames(recorder: &FrameRecorder, want: usize) -> Vec<Frame> {
        for _ in 0..200 {
            let frames = recorder.frames();
            if frames.len() >= want {
                return frames;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        recorder.frames()
    }

    /// The state `connect` leaves behind, assembled without the handshake: an
    /// `McuObject` registered on its printer, its chip holding a connected MCU
    /// (`mappings` scripted on its mock device), and a clock estimate seeded
    /// from one `get_uptime` reading — what the poll timer finds when it runs.
    struct PollRig {
        /// Stepped by the test to fire the timer.
        manual: Arc<ManualReactor>,
        /// Held because the timer callback only holds a `Weak`.
        printer: Arc<Printer>,
        object: Arc<McuObject>,
        mcu: Arc<Mcu>,
        recorder: FrameRecorder,
    }

    impl PollRig {
        fn new(dictionary: Value, mappings: Vec<MappingEntry>) -> Self {
            let manual = Arc::new(ManualReactor::new());
            let reactor: Arc<dyn Reactor> = manual.clone();
            let printer = Arc::new(Printer::new(Arc::clone(&reactor)));
            printer
                .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
                .unwrap();
            let object = Arc::new(McuObject::new(section(None), &printer).unwrap());
            printer
                .add_object(&object.section.identifier(), object.clone())
                .unwrap();

            let device = FrameMock::new(mappings);
            let recorder = device.recorder();
            let mcu = Arc::new(Mcu::for_test("mcu", Interface::new(device)));
            mcu.install_dictionary(Dictionary::from_json(dictionary).unwrap())
                .unwrap();
            object.chip.attach(Arc::clone(&mcu));

            // The seed `connect` takes from `get_uptime`: in the MCU's own
            // estimate, and as the regression's base point behind `McuClock`.
            mcu.set_clock_base(1_000_000);
            let clock = Arc::new(McuClock::new(Arc::clone(&mcu), reactor));
            clock.seed(0.0, 1_000_000);
            object.chip.set_clock(clock, 0.0);

            Self {
                manual,
                printer,
                object,
                mcu,
                recorder,
            }
        }
    }

    #[tokio::test]
    async fn test_the_clock_poll_fires_on_schedule_and_queries_get_clock() {
        // The poll has to arrive on the wire as `get_clock`: that query is the
        // only path that folds a sample into the estimator
        // (`McuClock::get_clock`), and it has to come after a full interval
        // rather than immediately.
        let rig = PollRig::new(polling_dictionary(), clock_exchange(21_000_500));
        // What the timer callback resolves: the object, by its section id.
        assert!(rig.printer.lookup_object_as::<McuObject>("mcu").is_some());
        rig.object.register_clock_poll(rig.manual.as_ref());

        assert_eq!(rig.manual.advance(0.5), 0, "no fire before the interval");
        assert_eq!(rig.manual.advance(0.5), 1, "one fire at the interval");
        let sent = wait_for_frames(&rig.recorder, 1).await;
        assert_eq!(sent.len(), 1, "one query, got {sent:?}");
        assert_eq!(sent[0].payload(), &[5], "the query is get_clock");
    }

    #[tokio::test]
    async fn test_one_clock_poll_feeds_the_estimator() {
        // What the poll is for: the sample moves the regression's clock onto
        // the reading and re-anchors `Mcu::estimated_clock` on it
        // (`Mcu::record_clock_sample`, behind `McuClock::get_clock`).
        let rig = PollRig::new(polling_dictionary(), clock_exchange(21_000_500));
        let before = rig.mcu.estimated_clock().unwrap();
        assert!(before < 21_000_500, "still on the connect seed: {before}");

        rig.object.register_clock_poll(rig.manual.as_ref());
        rig.manual.advance(1.0);
        wait_for_frames(&rig.recorder, 1).await;

        let clock = rig.object.clock().unwrap();
        assert_eq!(
            clock.estimator().last_clock(),
            21_000_500,
            "the sample folded into the regression"
        );
        let got = rig.mcu.estimated_clock().unwrap();
        assert!(got >= 21_000_500, "re-anchored on the sample: {got}");
    }

    #[tokio::test]
    async fn test_a_failed_clock_poll_feeds_nothing_and_keeps_its_schedule() {
        // A query that cannot go through (here `get_clock` is not even in the
        // firmware's dictionary, so it fails before anything is sent) must not
        // disturb anything: no sample, no panic, no state change — and the
        // next interval runs as usual. Watching for a dead MCU is a different
        // feature and deliberately not part of this timer.
        let rig = PollRig::new(json!({"config": {"CLOCK_FREQ": 20000000}}), vec![]);
        let seed = rig.object.clock().unwrap().estimator().last_clock();
        rig.object.register_clock_poll(rig.manual.as_ref());

        assert_eq!(rig.manual.advance(1.0), 1, "the first round fires");
        tokio::time::sleep(Duration::from_millis(50)).await; // let the query fail
        assert!(rig.recorder.frames().is_empty(), "nothing reached the wire");
        assert_eq!(
            rig.object.clock().unwrap().estimator().last_clock(),
            seed,
            "no sample folded in"
        );
        assert!(rig.mcu.estimated_clock().is_some(), "the seed stands");

        assert_eq!(rig.manual.advance(1.0), 1, "the next round fires again");
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(rig.recorder.frames().is_empty());
    }

    #[tokio::test]
    async fn test_releasing_the_object_stops_the_clock_poll() {
        // The `mcu_recalibrate` precedent's stop semantics: teardown cancels
        // the timer, so nothing polls after the machine comes down.
        let rig = PollRig::new(polling_dictionary(), clock_exchange(21_000_500));
        rig.object.register_clock_poll(rig.manual.as_ref());

        rig.object.release_cycles();

        assert_eq!(
            rig.manual.advance(10.0),
            0,
            "the cancelled timer never fires"
        );
        assert!(rig.recorder.frames().is_empty());
    }

    // -----------------------------------------------------------------------
    // reset → reconnect (B6 — the old session must go down explicitly)
    // -----------------------------------------------------------------------

    /// The host's reset → reconnect, end to end against the `test:` fake: the
    /// old session is closed **before** the reopen starts, and the new
    /// session's clock lands on the chip — the two things r8host's death
    /// spiral (08:22:45–08:23:30) proved were missing. There the drop of one
    /// reference was supposed to end the old session while the clock and the
    /// chip slot still held theirs, so the old tasks outlived it: the old
    /// write end retransmitted against a dead fd once a second, and the clock
    /// poll kept querying the closed session forever.
    #[tokio::test]
    async fn test_reconnect_closes_the_old_session_and_binds_the_new_sessions_clock() {
        // The object, its printer and a manual reactor — the poll timer is the
        // only thing driven by hand here.
        let manual = Arc::new(ManualReactor::new());
        let reactor: Arc<dyn Reactor> = manual.clone();
        let printer = Arc::new(Printer::new(Arc::clone(&reactor)));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mut section = section(None);
        section.parameters.insert(
            "test".to_string(),
            ConfigValue::Single(format!(
                "dict={}",
                klipperx_test_support::test_dicts_dir()
                    .join("linuxprocess.dict")
                    .display()
            )),
        );
        let object = Arc::new(McuObject::new(section, &printer).unwrap());
        printer
            .add_object(&object.section.identifier(), object.clone())
            .unwrap();
        let config = McuConfig::new(&ConfigWrapper::untracked(&object.section)).unwrap();

        // The old session, as `connect` leaves it at the handshake: attached,
        // clock installed, poll registered (`install_clock` registers it).
        let device = FrameMock::new(clock_exchange(21_000_500));
        let recorder = device.recorder();
        let previous = Arc::new(Mcu::for_test("old", Interface::new(device)));
        previous
            .install_dictionary(Dictionary::from_json(polling_dictionary()).unwrap())
            .unwrap();
        object.chip.attach(Arc::clone(&previous));
        object.install_clock(&previous, &reactor, 0.0, Some(1_000_000));
        let held = Arc::clone(&previous);

        let mut reconnect = Box::pin(object.reconnect(&config, previous));
        // One second into the reopen: the port sleeps `RECONNECT_DELAY` before
        // the first attempt, so this lands inside that window.
        let _ = tokio::time::timeout(Duration::from_millis(100), &mut reconnect).await;

        // (1) The old session is already down — before any reopen has
        // succeeded, and while two `Arc`s still reference it.
        for _ in 0..100 {
            if held.transport_tasks_finished() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(
            held.transport_tasks_finished(),
            "the old session's tasks stop at the start of the reconnect, not when the Arc goes"
        );
        assert!(
            Arc::strong_count(&held) >= 2,
            "the Arc is still shared: only the explicit close stopped those tasks"
        );
        assert!(
            held.send("get_clock", &[]).is_err(),
            "the closed session refuses sends"
        );

        // The poll keeps firing through the window; a closed session is not
        // queried (its mapping below would record the frame otherwise).
        let _ = manual.advance(1.0);
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(
            recorder.frames().is_empty(),
            "no frame on the closed session's transport during the reopen"
        );

        // Let the reopen run to success against the fake.
        tokio::time::timeout(Duration::from_secs(30), &mut reconnect)
            .await
            .expect("the reopen keeps trying until the port answers")
            .expect("identify against the fake succeeds");

        // (2) The chip names the new session — and so does its clock, which is
        // what the periodic poll queries from now on.
        let reopened = object.chip.mcu().expect("the new session is attached");
        assert!(
            !Arc::ptr_eq(&reopened, &held),
            "a new session, not the old one"
        );
        let clock = object.clock().expect("a clock is installed");
        assert!(
            Arc::ptr_eq(clock.mcu(), &reopened),
            "the clock poll is bound to the new session's wire"
        );
        drop(clock);
        assert_eq!(
            Arc::strong_count(&held),
            1,
            "the old clock released its Arc<Mcu>: nothing holds the closed session"
        );

        // The next fire's sample comes from the new session: the estimate
        // moves (the old session would feed nothing — it is closed), and the
        // old transport stays silent.
        let before = object.clock().unwrap().estimator().last_clock();
        let _ = manual.advance(0.35); // `test:` cadence is 150 ms
        tokio::time::sleep(Duration::from_millis(100)).await;
        let after = object.clock().unwrap().estimator().last_clock();
        assert_ne!(
            before, after,
            "a sample from the new session folded into the estimate: {before} -> {after}"
        );
        assert!(
            recorder.frames().is_empty(),
            "the old transport stayed quiet through the reconnect"
        );
    }

    // -----------------------------------------------------------------------
    // The connect handshake's stop watch
    // -----------------------------------------------------------------------

    /// A firmware dictionary with what the config phase needs, plus the two
    /// messages a stopped firmware reports itself with.
    ///
    /// `allocate_oids` / `finalize_config` / `get_config` use the ids the
    /// `test:` fake's dictionary has (`linuxprocess.dict`), so a configuration
    /// built on this one is accepted by the session a reopen brings up
    /// (`test_a_reset_that_comes_back_stopped_is_reset_again_and_connects`).
    fn stopping_dictionary() -> Value {
        json!({
            "commands": {
                "allocate_oids count=%c": 8,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "reset": 30
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9,
                "shutdown clock=%u static_string_id=%hu": 20,
                "is_shutdown static_string_id=%hu": 21
            },
            "enumerations": {
                "static_string_id": {"Move queue overflow": 0, "Timer too close": 3}
            },
            "config": {"CLOCK_FREQ": 20000000}
        })
    }

    /// An MCU attached to `object`'s chip whose firmware answers the handshake's
    /// first question — `get_config` — with `answer` instead of a configuration
    /// state.
    fn attached_answering_get_config(object: &McuObject, answer: Vec<u8>) -> Arc<Mcu> {
        let mut request = Payload::new();
        request.push_i16(7).unwrap(); // get_config
        let device = FrameMock::new(vec![MappingEntry {
            input: Frame::new(0, request.into_raw()),
            outputs: vec![Frame::new(0, answer)],
        }]);
        let mcu = Arc::new(Mcu::for_test("mcu2", Interface::new(device)));
        mcu.install_dictionary(Dictionary::from_json(stopping_dictionary()).unwrap())
            .unwrap();
        object.chip.attach(Arc::clone(&mcu));
        mcu
    }

    /// Take `object` through the handshake's first question, from the
    /// connect-time watcher on — the seam [`McuObject::connect`] drives around
    /// the handshake.
    ///
    /// `connect` itself cannot be driven here: it opens the port the section
    /// names, and a section can only name a real port or the `test:` fake, whose
    /// dictionary drives a *working* firmware. What `connect` adds around this —
    /// the watcher's binding and [`McuObject::connect_failure`] — is what the
    /// tests below call in the order it does.
    async fn handshake_against(object: &McuObject, answer: Vec<u8>) -> (McuError, Duration) {
        let mcu = attached_answering_get_config(object, answer);
        object.watch_connect_shutdown(&mcu).unwrap();
        let mut built = object.chip.config().build(&mcu).unwrap();
        let started = std::time::Instant::now();
        let err = object
            .chip
            .config()
            .handshake(&mcu, &mut built, false)
            .await
            .expect_err("a firmware that stopped accepts no configuration");
        (err, started.elapsed())
    }

    #[tokio::test]
    async fn test_a_stop_during_the_handshake_fails_the_connect_at_once_with_its_reason() {
        // What the real board does: a firmware that stopped refuses the commands
        // it cannot run and answers `is_shutdown` with its reason instead
        // (`src/command.c:346-349`), so the answer the handshake waits for never
        // comes. Without a handler bound before the handshake the report is
        // discarded as an unhandled message and the connect reports its own
        // five-second timeout for `config` — the misleading failure this pins
        // down (reproduced on a two-board setup, `mcu2`, "Timer too close").
        let object = McuObject::new(section(Some("mcu2")), &printer()).unwrap();
        let mut answer = Payload::new();
        answer.push_i16(21).unwrap(); // is_shutdown
        answer.push_u16(3).unwrap(); // static_string_id 3 = "Timer too close"

        let (err, elapsed) = handshake_against(&object, answer.into_raw()).await;

        assert!(
            elapsed < Duration::from_secs(1),
            "the handshake returns when the stop is reported, not after its 5 s \
             timeout (took {elapsed:?})"
        );
        assert!(matches!(err, McuError::Call(_)), "got {err:?}");
        assert_eq!(
            object.connect_shutdown().as_deref(),
            Some("Timer too close")
        );
        let failure = object
            .connect_failure(KlippyError::Connection(err.to_string()))
            .to_string();
        assert!(
            failure.contains("MCU 'mcu2' shutdown during connect: Timer too close"),
            "{failure}"
        );
    }

    #[tokio::test]
    async fn test_an_unsolicited_shutdown_during_the_handshake_fails_the_connect_at_once() {
        // The other shape: the firmware announces the stop itself, with the
        // clock it happened at (`src/sched.c:310-311`). Same consequence — the
        // handshake's answer is not coming — and the same reason reaches the
        // bring-up's failure.
        let object = McuObject::new(section(Some("mcu2")), &printer()).unwrap();
        let mut answer = Payload::new();
        answer.push_i16(20).unwrap(); // shutdown
        answer.push_u32(1234).unwrap(); // clock
        answer.push_u16(3).unwrap(); // static_string_id 3 = "Timer too close"

        let (_, elapsed) = handshake_against(&object, answer.into_raw()).await;

        assert!(
            elapsed < Duration::from_secs(1),
            "the handshake returns when the stop is reported (took {elapsed:?})"
        );
        assert_eq!(
            object.connect_shutdown().as_deref(),
            Some("Timer too close")
        );
    }

    #[test]
    fn test_the_first_stop_reason_is_the_one_kept() {
        // A stopped firmware repeats its one reason in every `is_shutdown`, so
        // only the first report has anything to say.
        let slot = Mutex::new(None);

        note_connect_shutdown(&slot, "mcu2", "Timer too close");
        note_connect_shutdown(&slot, "mcu2", "Move queue overflow");

        assert_eq!(slot.lock().unwrap().as_deref(), Some("Timer too close"));
    }

    #[tokio::test]
    async fn test_a_reopen_starts_with_a_clean_connect_shutdown_slot() {
        // The old connection's dying words are not the new session's: a `reset`
        // is sent to a board that may report a stop while it reboots, and the
        // bring-up that replaces that session must not fail with it.
        let manual = Arc::new(ManualReactor::new());
        let reactor: Arc<dyn Reactor> = manual.clone();
        let printer = Arc::new(Printer::new(Arc::clone(&reactor)));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mut section = section(None);
        section.parameters.insert(
            "test".to_string(),
            ConfigValue::Single(format!(
                "dict={}",
                klipperx_test_support::test_dicts_dir()
                    .join("linuxprocess.dict")
                    .display()
            )),
        );
        let object = Arc::new(McuObject::new(section, &printer).unwrap());
        printer
            .add_object(&object.section.identifier(), object.clone())
            .unwrap();
        let config = McuConfig::new(&ConfigWrapper::untracked(&object.section)).unwrap();

        // The dead session's last words, as its watcher would have left them.
        note_connect_shutdown(&object.connect_shutdown, "mcu", "Timer too close");
        let previous = Arc::new(Mcu::for_test("old", Interface::new(FrameMock::new(vec![]))));
        previous
            .install_dictionary(Dictionary::from_json(polling_dictionary()).unwrap())
            .unwrap();

        tokio::time::timeout(Duration::from_secs(30), object.reconnect(&config, previous))
            .await
            .expect("the reopen keeps trying until the port answers")
            .expect("identify against the fake succeeds");

        assert_eq!(
            object.connect_shutdown(),
            None,
            "the session that comes up starts with a clean slot"
        );
    }

    /// The retry that makes a real board's flaky reboot tolerable: a session the
    /// reopen brings back **stopped** (a clock fault while it started) refuses
    /// the configuration, so one reset is not enough — the bring-up has to
    /// reset again instead of handing the shutdown to the user.
    ///
    /// The first session is a mock whose `get_config` is answered with
    /// `is_shutdown`; the reopen brings up the `test:` fake, which accepts the
    /// configuration. Nothing here is scripted through `connect` itself — it
    /// opens the section's port — but the loop `connect` runs is the same one,
    /// handed the session it starts from.
    #[tokio::test]
    async fn test_a_reset_that_comes_back_stopped_is_reset_again_and_connects() {
        let manual = Arc::new(ManualReactor::new());
        let reactor: Arc<dyn Reactor> = manual.clone();
        let printer = Arc::new(Printer::new(Arc::clone(&reactor)));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mut section = section(Some("mcu2"));
        section.parameters.insert(
            "test".to_string(),
            ConfigValue::Single(format!(
                "dict={}",
                klipperx_test_support::test_dicts_dir()
                    .join("linuxprocess.dict")
                    .display()
            )),
        );
        let object = Arc::new(McuObject::new(section, &printer).unwrap());
        printer
            .add_object(&object.section.identifier(), object.clone())
            .unwrap();
        let config = McuConfig::new(&ConfigWrapper::untracked(&object.section)).unwrap();

        // The stopped session: `get_config` answered with `is_shutdown`, and
        // `reset` scripted so `reset_and_flush` has something to send.
        let mut get_config = Payload::new();
        get_config.push_i16(7).unwrap();
        let mut stopped = Payload::new();
        stopped.push_i16(21).unwrap(); // is_shutdown
        stopped.push_u16(3).unwrap(); // static_string_id 3 = "Timer too close"
        let mut reset = Payload::new();
        reset.push_i16(30).unwrap(); // reset
        let device = FrameMock::new(vec![
            MappingEntry {
                input: Frame::new(0, get_config.into_raw()),
                outputs: vec![Frame::new(0, stopped.into_raw())],
            },
            MappingEntry {
                input: Frame::new(1, reset.into_raw()),
                outputs: vec![],
            },
        ]);
        let recorder = device.recorder();
        let first = Arc::new(Mcu::for_test("mcu2", Interface::new(device)));
        first
            .install_dictionary(Dictionary::from_json(stopping_dictionary()).unwrap())
            .unwrap();
        object.chip.attach(Arc::clone(&first));
        object.watch_connect_shutdown(&first).unwrap();
        let built = object.chip.config().build(&first).unwrap();
        let held = Arc::clone(&first);

        let (mcu, _configured) = tokio::time::timeout(
            Duration::from_secs(30),
            object.handshake_with_in_place_resets(&config, first, built),
        )
        .await
        .expect("the retry does not hang")
        .expect("the connection the reset brings back accepts the configuration");

        // The stop was retried, not reported: the reset went out on the stopped
        // session, and the handshake finished on a new one.
        let resets: Vec<_> = recorder
            .frames()
            .into_iter()
            .filter(|frame| frame.payload() == [30u8].as_slice())
            .collect();
        assert_eq!(resets.len(), 1, "one reset on the stopped session");
        assert!(!Arc::ptr_eq(&mcu, &held), "a new session, not the old one");
    }

    /// The retry is bounded: each round is a reboot, and the bring-up has to
    /// hand the failure over at some point.
    #[test]
    fn test_a_come_back_stopped_is_retried_only_a_bounded_number_of_times() {
        let object = object(Some("mcu2"));

        // A clean slot: only `ResetRequired` — the firmware's own plan — is
        // worth a reset.
        assert!(object.should_reset_and_retry(&McuError::ResetRequired, 0));
        assert!(!object.should_reset_and_retry(&McuError::NotIdentified, 0));

        // With a stop recorded, any failed round is retried while resets are
        // left...
        note_connect_shutdown(&object.connect_shutdown, "mcu2", "Timer too close");
        assert!(object.should_reset_and_retry(&McuError::NotIdentified, 0));
        assert!(object.should_reset_and_retry(&McuError::NotIdentified, MAX_IN_PLACE_RESETS - 1));

        // ...and is not, once they are spent.
        assert!(!object.should_reset_and_retry(&McuError::NotIdentified, MAX_IN_PLACE_RESETS));
        assert!(!object.should_reset_and_retry(&McuError::ResetRequired, MAX_IN_PLACE_RESETS));
    }

    /// The failure a spent bring-up hands over names the MCU and quotes the
    /// firmware's own reason, not a response timeout.
    #[test]
    fn test_a_bring_up_that_runs_out_of_resets_reports_the_mcus_name_and_reason() {
        let stopped_object = object(Some("mcu2"));
        note_connect_shutdown(&stopped_object.connect_shutdown, "mcu2", "Timer too close");

        let stopped = stopped_object
            .handshake_failure(&McuError::NotIdentified)
            .to_string();
        assert!(stopped.contains("MCU 'mcu2'"), "{stopped}");
        assert!(stopped.contains("Timer too close"), "{stopped}");

        // A `ResetRequired` that outlived every reset keeps its own wording:
        // the firmware kept carrying its configuration through them.
        let carried_object = object(Some("mcu2"));
        let carried = carried_object
            .handshake_failure(&McuError::ResetRequired)
            .to_string();
        assert!(
            carried.contains("MCU 'mcu2' still carries a configuration after a 'reset'"),
            "{carried}"
        );
    }
}
