//! `[mcu]` as a printer object: the section, and the connection it needs.
//!
//! Upstream's `MCU` is one printer object per `[mcu]` / `[mcu <name>]` section
//! (`klippy/mcu.py:1147`), registered by `mcu.add_printer_objects` under the
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
use tracing::{info, warn};

use crate::core::klippy::cmd::clock::{McuClock, SecondarySync};
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
    ConfigBuilder, Dictionary, I2cMode, Mcu, McuChip, McuError, McuI2c, McuRestartMethod, McuSpi,
    McuStepper, SpiMode,
};
use crate::core::klippy::pins::{PinError, PinParams, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject, RestartFuture};
use crate::core::klippy::reactor::TimerHandle;

/// How long to wait between attempts to reopen a board that was just told to
/// reboot. A native-USB board re-enumerates, so the port is briefly gone.
const RECONNECT_DELAY: Duration = Duration::from_millis(250);

/// How many times to try reopening before giving up (a little over 5 s).
const RECONNECT_ATTEMPTS: usize = 20;

/// How long to wait for the `reset` command to leave the send queue before the
/// rebooted board takes the transport with it.
const RESET_FLUSH_TIMEOUT: Duration = Duration::from_secs(1);

/// How long the one `get_uptime` that seeds the clock estimate may take.
///
/// The same order as the other connect-time reads; a firmware that does not
/// answer only loses the estimate, not the connection.
const CLOCK_BASE_TIMEOUT: Duration = Duration::from_secs(1);

/// How often a secondary MCU's clock alignment is recalibrated, in seconds
/// (upstream does it in the periodic `stats`, `klippy/extras/motion_quuing.py:100`).
const RECALIBRATE_INTERVAL: f64 = 1.0;

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
    /// (`klippy/mcu.py:996-997`): a pin description may name this MCU as its
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
            last_stats: Arc::new(Mutex::new(None)),
            secondary_sync: Mutex::new(None),
            recalibrate_timer: Mutex::new(None),
            printer: Arc::downgrade(printer),
        })
    }

    /// The MCU's own name, as upstream's `MCU.get_name` reports it.
    pub fn name(&self) -> &str {
        self.chip.name()
    }

    /// Whether this firmware is stopped (or the host has marked it so).
    ///
    /// Upstream's `MCU.is_shutdown()` (`klippy/mcu.py:908`).
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
    /// `previous` is dropped first so its transport is closed before the port is
    /// reopened.
    async fn reconnect(
        &self,
        config: &McuConfig,
        previous: Arc<Mcu>,
    ) -> Result<Arc<Mcu>, KlippyError> {
        drop(previous);
        let mcu = self.open_and_connect(config).await?;
        self.chip.attach(Arc::clone(&mcu));
        // A fresh connection has a fresh clock; re-seed the estimate.
        seed_clock_base(&mcu).await;
        Ok(mcu)
    }

    /// Whether this bring-up follows a `firmware_restart`.
    ///
    /// Only then is the firmware itself reset; a first start and a plain
    /// `restart` reconnect without touching it (upstream keys the same decision
    /// on `start_reason`, `klippy/mcu.py:678-680`).
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
    /// The replacement is recorded on the printer, not here: a restart reloads the
    /// parsed config and rebuilds this object, and the config file is not written
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
            let mut mcu = if self.is_firmware_restart() {
                match self.open_and_connect(&config).await {
                    Ok(mcu) => mcu,
                    Err(err) => {
                        if usb_reset {
                            self.usb_reset_unusable(&config, &err.to_string());
                        }
                        return Err(err);
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
                        return Err(KlippyError::Connection(err.to_string()));
                    }
                }
            };
            // Make the device reachable by resources before the configuration
            // is built; a resource's runtime methods need it.
            self.chip.attach(Arc::clone(&mcu));
            // One clock read, so an unclocked resource can estimate "now"
            // (`Mcu::estimated_clock`). A firmware without `get_uptime` simply
            // has no estimate.
            let reactor = self.printer.upgrade().map(|printer| printer.reactor());
            let sent_time = reactor
                .as_ref()
                .map(|reactor| reactor.monotonic())
                .unwrap_or(0.0);
            let uptime = seed_clock_base(&mcu).await;
            // Build this MCU's clock estimate and its print-time alignment. The
            // primary (the bare `[mcu]`) defines the print-time origin; a
            // secondary is shifted so the same print time maps to its own clock
            // (`SecondarySync`, `klippy/clocksync.py:177-235`). Resources that
            // convert print time to this MCU's clock read it through the chip.
            if let Some(reactor) = &reactor {
                let clock = Arc::new(McuClock::new(Arc::clone(&mcu), Arc::clone(reactor)));
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
                        .map(|main| {
                            main.estimated_print_time(now) - clock.estimated_print_time(now)
                        })
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
                    let printer = self.printer.clone();
                    let identifier = self.section.identifier();
                    let handle = reactor.register_timer_named(
                        "mcu_recalibrate",
                        Box::new(move |eventtime| {
                            let printer = printer.upgrade()?;
                            if let Some(object) = printer.lookup_object_as::<McuObject>(&identifier)
                            {
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
            // `klippy/mcu.py:730-746`). What is left for this loop is the
            // firmware whose *only* reset is `reset` and that still carries a
            // configuration: no `config_reset` to clear it in place, so it has to
            // be rebooted and re-identified here.
            let mut reset_sent = false;

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
            let mut built = self.chip.config().build(&mcu).map_err(|err| match err {
                // A config callback resolves pins/buses against the
                // dictionary; a bad pin is a config problem, not klippy's.
                McuError::Config(message) => KlippyError::Config(ConfigError::new(message)),
                other => KlippyError::Internal(other.to_string()),
            })?;
            let configured = loop {
                match self
                    .chip
                    .config()
                    .handshake(&mcu, &mut built, self.is_firmware_restart())
                    .await
                {
                    Ok(configured) => break configured,
                    Err(McuError::ResetRequired) if !reset_sent => {
                        // No `config_reset`, but the firmware can reboot itself:
                        // send `reset` and re-run the handshake on the
                        // connection that comes back.
                        info!(
                            "MCU '{}': resetting the firmware with the 'reset' command",
                            config.name
                        );
                        reset_and_flush(&mcu).await?;
                        reset_sent = true;
                        mcu = self.reconnect(&config, mcu).await?;
                    }
                    Err(McuError::ResetRequired) => {
                        return Err(KlippyError::Connection(format!(
                            "MCU '{}' still carries a configuration after a 'reset'",
                            config.name
                        )));
                    }
                    Err(err) => {
                        return Err(KlippyError::Connection(err.to_string()));
                    }
                }
            };
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
    // handler in `MCU.__init__`, `klippy/mcu.py:798-799`). Registered here
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
        // Upstream's `MCU._shutdown` (`klippy/mcu.py:888-893`): a host shutdown
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
}
