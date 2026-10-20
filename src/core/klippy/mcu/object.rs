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
//! The reported fields are the four upstream has: the three identify ones and
//! `last_stats`, which is accumulated from the firmware's periodic `stats`
//! event ([`register_stats`](crate::core::klippy::event::stats::register_stats))
//! and dated by its companion `last_stats_time`. Both appear only once a report
//! has arrived, as upstream's `last_stats` does (its `stats()` collector is what
//! adds the key, `klippy/mcu.py:975`).

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use serde_json::{json, Map, Value};
use tracing::{debug, info, warn};

use super::bridge_attach;
use super::host_libraries;
use super::ids;
use super::port_grouping::{self, PortKind};
use super::serial_ports;
use crate::core::klippy::cmd::clock::{ClockSync, McuClock, SecondarySync};
use crate::core::klippy::cmd::config::Reset;
use crate::core::klippy::cmd::shutdown::EmergencyStop;
use crate::core::klippy::cmd::uptime::{GetUptime, Uptime};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::config::mcu::{McuConfig, Transport};
use crate::core::klippy::config::value::ConfigValue;
use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigWrapper};
use crate::core::klippy::error::{ConfigError, KlippyError};
use crate::core::klippy::event::stats::{register_stats, LastStats};
use crate::core::klippy::event::{IsShutdown, KlippyEvent, McuEvent, Shutdown, Starting};
use crate::core::klippy::interface::{HostMux, Interface};
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

/// How many times to try reopening before giving up.
///
/// The delay alone is 20 × [`RECONNECT_DELAY`] = 5 s, which is *not* the
/// budget: each round also runs a full `Mcu::connect`, whose identify is
/// bounded by [`IDENTIFY_TIMEOUT`](crate::core::klippy::identify::IDENTIFY_TIMEOUT)
/// (10 s) and shares it with everything else that round waits for. A board that
/// stays silent is therefore waited on for up to 20 × (250 ms + 10 s) ≈ 205 s.
/// That bound is what makes the exit check in `open_and_connect` necessary: a
/// shutdown must not sit on three and a half minutes of a board that is not
/// coming back.
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
/// The same order as the other connect-time reads. A firmware that does not
/// answer fails the connect (`seed_clock_base`): without this reading there is
/// no anchor for the print-time mapping at all.
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
    /// The transport description the loader parsed `section` into.
    ///
    /// `connect` cannot re-parse a `uart:`/`can:` section on its own: the parse
    /// checks that the port section it names exists, which needs the whole
    /// config, and a part holding only its section cannot see one
    /// (`ConfigWrapper::has_sibling` answers false without it). The loader
    /// parses where the config is in hand, and stores the answer here so
    /// `connect` reads it instead of re-parsing. `None` only for an object built
    /// outside the loader, which re-parses as before.
    parsed: Mutex<Option<McuConfig>>,
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
    /// The id this `[mcu]` was assigned at load, unique across every MCU
    /// whatever its transport (`mcu/ids.rs`).
    ///
    /// `Some` after the loader registered the section; `connect` refuses to
    /// open a transport without it (a CAN MCU cannot be addressed until it has
    /// the id it was assigned, and every transport takes one from the same
    /// sequence). `None` only for an object built outside the loader, which is
    /// never the one a working machine connects.
    id: Mutex<Option<u32>>,
    /// The rate this MCU's serial path is opened at, when the path is addressed
    /// by id (two or more `[mcu]` sections name it, or a bridge's link carries
    /// its downstream devices' frames), or `None` for an exclusive one.
    ///
    /// Stamped at load from the grouping pass (`mcu/serial_ports.rs`) and read
    /// back by `connect`, exactly as [`McuObject::id`] is: it is what tells the
    /// transport layer to open the shared port instead of a private device.
    shared_baud: Mutex<Option<u32>>,
    /// What this MCU's `host_library:` path attaches with when the config calls
    /// it muxed, or `None` for a path that is not muxed (and for every other
    /// transport).
    ///
    /// Stamped at load from the path grouping (`mcu/host_libraries.rs`) and read
    /// back by `connect`, exactly as [`McuObject::shared_baud`] is: it is what
    /// tells the transport to open the library through [`Interface::host_mux`]
    /// instead of the bare-frame mode.
    ///
    /// [`Interface::host_mux`]: crate::core::klippy::interface::Interface::host_mux
    host_mux: Mutex<Option<HostMux>>,
    /// The tty the host has to name for each of this bridge's ports before the
    /// firmware comes up, as `(slot, path)` — the slot being the port's index.
    ///
    /// Filled while the config loads, by the port sections' factory
    /// (`mcu/bridge_ports.rs`, which also assigns the index), and read back by
    /// `connect`: the host library is told every pair before
    /// `klipper_host_init` (`HostDevice::load_mux`), so the port's tty is a
    /// deployment fact rather than one baked into the library. Empty for every
    /// MCU that is not a host-library bridge, and for a port that writes no
    /// `uart_path:` (its port keeps the library's own default).
    host_port_paths: Mutex<Vec<(u8, String)>>,
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
    /// Whether this MCU has already been reported as lost.
    ///
    /// Upstream's `_is_timeout` (`klippy/mcu.py:801`): one report per MCU. The
    /// clock poll keeps running after the machine is shutdown, so without this
    /// every later round would find the same unanswered queries and ask for the
    /// same stop again.
    clock_timed_out: AtomicBool,
    /// The latest scheduler load from the firmware's `stats` reports, for
    /// `get_status`'s `last_stats` and `last_stats_time`
    /// (`klippy/mcu.py:974-975`).
    ///
    /// `Arc` because the `'static` event handler owns its handle to it.
    last_stats: Arc<Mutex<Option<LastStats>>>,
    /// The secondary clock alignment, when this is not the primary MCU
    /// (`SecondarySync`); `None` for the primary.
    secondary_sync: Mutex<Option<SecondarySync>>,
    /// How many downstream ports have claimed an index on this MCU as a bridge.
    ///
    /// A bridge's ports are created at configuration time, in the order the
    /// `[uart_device]` sections are declared, and that declaration order **is**
    /// the port's index: the firmware numbers ports by the order the host's
    /// `config_bridge_port` commands arrive
    /// (`third_party/klipper/src/uart.c`), and the host queues them in this
    /// order (`mcu/bridge_ports.rs`). So this counter is both "the next index"
    /// and "how many ports this bridge has". Only a bridge is ever asked; a
    /// plain `[mcu]` never is.
    bridge_ports: AtomicU16,
    /// How many UART ports have claimed a slot on this MCU as a bridge.
    ///
    /// A UART port's **slot** is a different number from its index: the index
    /// counts every port on the bridge, CAN ones included, while the firmware
    /// picks a USART by the slot `config_uart` carries and by nothing else
    /// (`third_party/klipper/src/stm32/bridge_port.c`: the UART slots are
    /// `0..bridge_serial_slot_count()-1`, and a CAN port names no slot at all).
    /// So the slot is "which UART of this bridge the port is" — the count of
    /// UART ports declared before it, whatever their indices are. Keeping the
    /// two apart is what stops a `[can_device]` declared before a
    /// `[uart_device]` from handing that UART port the wrong USART.
    ///
    /// Only a UART port takes from it; a CAN port takes an index only.
    bridge_uart_slots: AtomicU16,
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
            parsed: Mutex::new(None),
            chip,
            status: Mutex::new(json!({})),
            id: Mutex::new(None),
            shared_baud: Mutex::new(None),
            host_mux: Mutex::new(None),
            host_port_paths: Mutex::new(Vec::new()),
            restart_method: Mutex::new(McuRestartMethod::Command),
            is_shutdown: Arc::new(AtomicBool::new(false)),
            connect_shutdown: Arc::new(Mutex::new(None)),
            clock_timed_out: AtomicBool::new(false),
            last_stats: Arc::new(Mutex::new(None)),
            secondary_sync: Mutex::new(None),
            bridge_ports: AtomicU16::new(0),
            bridge_uart_slots: AtomicU16::new(0),
            recalibrate_timer: Mutex::new(None),
            clock_poll_timer: Mutex::new(None),
            printer: Arc::downgrade(printer),
        })
    }

    /// The MCU's own name, as upstream's `MCU.get_name` reports it.
    pub fn name(&self) -> &str {
        self.chip.name()
    }

    /// The id the loader assigned this MCU, if it was registered.
    ///
    /// `None` for an object built outside the loader (tests); a working machine
    /// only ever connects a registered object, which always holds one.
    pub fn id(&self) -> Option<u32> {
        *self.id.lock().unwrap_or_else(|poison| poison.into_inner())
    }

    /// Record the id the loader assigned this `[mcu]` (`mcu/ids.rs`).
    pub fn set_id(&self, id: u32) {
        *self.id.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(id);
    }

    /// The baud this MCU shares its serial port at, if it is on a shared one.
    ///
    /// `None` means the `serial:` path is this MCU's alone (or the transport is
    /// not serial at all).
    pub fn shared_baud(&self) -> Option<u32> {
        *self
            .shared_baud
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Record the shared-port baud the load-time grouping chose
    /// (`mcu/serial_ports.rs`).
    pub fn set_shared_baud(&self, baud: Option<u32>) {
        *self
            .shared_baud
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = baud;
    }

    /// Take the next downstream-port index for this MCU as a bridge.
    ///
    /// A port's index is its declaration order among the bridge's ports (see
    /// the field): the first port section on a bridge gets 0, the next 1, and
    /// so on. The port-level `SET_PORT_NODEID` carries the index in one byte, so
    /// a bridge can hold at most 256 ports; the 257th is refused here, naming
    /// the port and the index it would have had, instead of wrapping onto an
    /// existing port's route.
    ///
    /// # Errors
    /// [`ConfigError`] when this bridge already has every port a port command
    /// can address.
    pub fn next_bridge_port_index(&self, port: &str) -> Result<u8, ConfigError> {
        let index = self.bridge_ports.fetch_add(1, Ordering::Relaxed);
        if index > u16::from(u8::MAX) {
            return Err(ConfigError::new(format!(
                "Section '{port}': bridge '{}' already has the 256 ports a port command can \
                 address; port {index} cannot be named (0..=255)",
                self.name()
            )));
        }
        Ok(index as u8)
    }

    /// Take the next UART slot for this MCU as a bridge.
    ///
    /// A UART port's slot is not its index (see the field): it is how many UART
    /// ports this bridge already has, so a bridge whose `[can_device]` sections
    /// are declared first still gives its first UART port slot 0. That is the
    /// number `config_uart` carries and the number the port's tty is registered
    /// under, both of which the firmware resolves a USART by.
    ///
    /// The one-byte bound is the index counter's: every port takes an index
    /// before its slot, so a bridge still inside the index range is inside this
    /// one too.
    pub fn next_bridge_uart_slot(&self) -> u8 {
        let slot = self.bridge_uart_slots.fetch_add(1, Ordering::Relaxed);
        debug_assert!(
            slot <= u16::from(u8::MAX),
            "a UART slot past the one-byte range, without the index counter refusing first"
        );
        slot as u8
    }

    /// What this MCU's `host_library:` path attaches with, when the config
    /// calls it muxed.
    ///
    /// `None` means the path is not muxed (or the transport is not a host
    /// library): it opens with bare frames, today's mode.
    pub fn host_mux(&self) -> Option<HostMux> {
        *self
            .host_mux
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Record what the load-time grouping decided a muxed `host_library:` path
    /// attaches with (`mcu/host_libraries.rs`).
    pub fn set_host_mux(&self, mux: Option<HostMux>) {
        *self
            .host_mux
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = mux;
    }

    /// Record the tty this bridge's port `slot` opens at run time.
    ///
    /// Called by the port section's factory while the config loads
    /// (`mcu/bridge_ports.rs`), before anything connects: the library is told
    /// every pair before it comes up, so a port's tty has to be known by then.
    /// `slot` is the port's **UART slot** — the same number `config_uart`
    /// carries, which is how the firmware looks the tty up again
    /// (`klipper_host_port` and `port_backend_open` both key on it). It is not
    /// the port's index: a CAN port declared first takes no slot.
    pub fn add_host_port_path(&self, slot: u8, path: String) {
        self.host_port_paths
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push((slot, path));
    }

    /// The `(slot, path)` pairs this bridge's ports named, in declaration order.
    pub fn host_port_paths(&self) -> Vec<(u8, String)> {
        self.host_port_paths
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
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

    /// This MCU's section, parsed back into the transport description its
    /// connection opened.
    ///
    /// The bridge-attach pass (`mcu/bridge_attach.rs`) reads the **bridge's**
    /// transport to open a downstream device on the same link. The loader made
    /// this parse where the whole config was in hand; an object built outside
    /// the loader falls back to parsing its stored section alone, which serves
    /// every transport whose parse does not need a sibling.
    ///
    /// # Errors
    /// The section's own parse error, the way `connect` reports one.
    pub(crate) fn mcu_config(&self) -> Result<McuConfig, KlippyError> {
        if let Some(config) = self.parsed_config() {
            return Ok(config);
        }
        // Parsed on the printer's tracker, exactly as the loader parses it: a
        // read that happens now is still recorded for the undefined-option
        // check.
        let access = self
            .printer
            .upgrade()
            .map(|printer| printer.access_tracking())
            .unwrap_or_else(AccessTracking::shared);
        let wrapper = ConfigWrapper::new(&self.section, access);
        let mut config = McuConfig::new(&wrapper).map_err(KlippyError::Config)?;
        config.id = self.id();
        config.shared_baud = self.shared_baud();
        config.host_mux = self.host_mux();
        config.host_port_paths = self.host_port_paths();
        Ok(config)
    }

    /// The transport description the loader stored, if it parsed this section.
    pub(crate) fn parsed_config(&self) -> Option<McuConfig> {
        self.parsed
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// Record the transport the loader parsed this section into
    /// ([`McuObject::mcu_config`]).
    pub(crate) fn set_parsed_config(&self, config: McuConfig) {
        *self
            .parsed
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(config);
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
    /// as usual — the failure itself is not the verdict on the link. What is
    /// watched is the count of queries still waiting for an answer
    /// ([`McuObject::check_clock_timeout`]). Cancellation is the precedent's —
    /// [`McuObject::release_cycles`] on teardown (`Drop`), so nothing polls
    /// after the machine comes down.
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
    /// debug level, feeds nothing — the timer keeps its schedule and the next
    /// round tries again.
    ///
    /// What the failure does change is the estimator's outstanding-query count
    /// (`ClockEstimator::is_active` is what reads it): that count is what
    /// [`McuObject::check_clock_timeout`] judges the link by, and it is the
    /// only state a dead board leaves behind.
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
        self.check_clock_timeout(&clock, &name);
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                // Outstanding until an answer folds in (`ClockEstimator::update`
                // clears the count), which is exactly what a dead board never
                // does.
                clock.estimator().note_query_sent();
                handle.spawn(async move {
                    if let Err(err) = clock.get_clock().await {
                        debug!("MCU '{name}': clock poll failed: {err}");
                    }
                });
            }
            Err(_) => debug!("MCU '{name}': the clock poll needs an async runtime"),
        }
    }

    /// Report this MCU as lost once too many clock queries have gone
    /// unanswered (`MCUConnHelper.check_timeout`, `klippy/mcu.py:895-905`).
    ///
    /// The judge is upstream's own: `ClockEstimator::is_active` tolerates up
    /// to four outstanding queries and no more (`klippy/clocksync.py:156-157`),
    /// so a board that stops answering is declared lost on the sixth round —
    /// five unanswered intervals, at [`CLOCK_POLL_INTERVAL`]'s one second
    /// each. The message is upstream's word for word, because clients and the
    /// `error_mcu` hint table match on that text.
    ///
    /// A **file-output** run never gets here: it writes the protocol to a file
    /// instead of a port (`Printer::is_fileoutput`), so nothing answers by
    /// design and the count would be meaningless. Reported once
    /// ([`McuObject::clock_timed_out`]) and not again: the timer keeps firing
    /// after the machine is shutdown.
    fn check_clock_timeout(&self, clock: &McuClock, name: &str) {
        if clock.estimator().is_active() || self.clock_timed_out.load(Ordering::SeqCst) {
            return;
        }
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if printer.is_fileoutput() {
            debug!(
                "MCU '{name}': no answer to the clock poll; not reported from a file-output run"
            );
            return;
        }
        self.clock_timed_out.store(true, Ordering::SeqCst);
        report_shutdown(
            &self.printer,
            &format!("Lost communication with MCU '{name}'"),
        );
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

    /// Open this MCU's transport, attaching through a bridge when the transport
    /// is a downstream port.
    ///
    /// A `uart:`/`can:` transport's device sits behind a bridge port: it is
    /// reached on the bridge's own link and named by a port-level
    /// `SET_PORT_NODEID`, never by [`Transport::open`] (`mcu/bridge_attach.rs`).
    /// Every other transport opens as before. `config` must already carry the id
    /// `connect` covered onto it — a bridge port is addressed by id like every
    /// other MCU.
    ///
    /// `retrying` says whether this open belongs to a connect that may wait for
    /// the port: an exclusive tty is then retried until it appears
    /// ([`McuConfig::open_retrying`]) and the printer's exit request ends the
    /// wait. The reopen after a `reset` passes `false` — its own loop already
    /// retries, and nesting the two would multiply the two budgets.
    ///
    /// The wait runs on the blocking pool, not here: the retry sleeps between
    /// attempts, and the machine runtime has two workers carrying the reactor
    /// dispatcher and the send/receive tasks — a sleep of up to the retry budget
    /// on one of them would stall everything the reactor owns for the whole
    /// wait.
    async fn open_interface(
        &self,
        config: &McuConfig,
        retrying: bool,
    ) -> Result<Interface, KlippyError> {
        if matches!(config.transport, Transport::BridgePort { .. }) {
            let printer = self.printer.upgrade().ok_or_else(|| {
                KlippyError::Internal(format!(
                    "MCU '{}': the printer is gone; cannot resolve its bridge port",
                    config.name
                ))
            })?;
            let id = config.id.ok_or_else(|| {
                KlippyError::Connection(format!(
                    "MCU '{}' has no id assigned; cannot open its transport",
                    config.name
                ))
            })?;
            let link = bridge_attach::resolve(&printer, config)?;
            // The attach is blocking I/O throughout — the tty open, the bridge
            // `dlopen`, up to three 500 ms naming attempts and the CAN settle
            // — so it runs on the blocking pool like the retry below it: on a
            // reactor worker the whole reactor waits for it.
            return tokio::task::spawn_blocking(move || bridge_attach::open(&printer, &link, id))
                .await
                .map_err(|err| {
                    KlippyError::Internal(format!("the bridge attach task failed: {err}"))
                })?;
        }
        if !retrying {
            return config.open().map_err(KlippyError::Connection);
        }
        let config = config.clone();
        let printer = self.printer.upgrade();
        tokio::task::spawn_blocking(move || {
            let stop = move || {
                printer
                    .as_ref()
                    .is_none_or(|printer| printer.exit_requested())
            };
            config.open_retrying(&stop)
        })
        .await
        .map_err(|err| KlippyError::Internal(format!("the port retry task failed: {err}")))?
        .map_err(KlippyError::Connection)
    }

    /// Open the port and identify the firmware, retrying while a rebooted board
    /// comes back.
    ///
    /// `reset` restarts the MCU, and a native-USB board disappears from the bus
    /// and re-enumerates, so the port may not be openable for a moment
    /// (`mcu/restart.rs`). A bring-up that follows a reset retries the whole
    /// open-and-handshake conversation here; the plain start retries only the
    /// open (`McuConfig::open_retrying`), so a protocol error is never retried.
    ///
    /// The rounds are what a shutdown has to be able to end: one round can spend
    /// the whole identify budget on a board that never answers, so
    /// [`RECONNECT_ATTEMPTS`] of them is minutes, not the seconds
    /// [`RECONNECT_DELAY`] suggests. An exit request is therefore polled after
    /// every sleep — the same test `open_interface(config, true)` gives its own
    /// retry — and ends the loop with a failure that says so.
    async fn open_and_connect(&self, config: &McuConfig) -> Result<Arc<Mcu>, KlippyError> {
        let mut last = String::new();
        let mut aborted = false;
        for _ in 0..RECONNECT_ATTEMPTS {
            tokio::time::sleep(RECONNECT_DELAY).await;
            if self
                .printer
                .upgrade()
                .is_none_or(|printer| printer.exit_requested())
            {
                aborted = true;
                break;
            }
            let interface = match self.open_interface(config, false).await {
                Ok(interface) => interface,
                Err(err) => {
                    last = err.to_string();
                    continue;
                }
            };
            match Mcu::connect(&config.name, interface).await {
                Ok(mcu) => return Ok(mcu),
                Err(err) => last = err.to_string(),
            }
        }
        if aborted {
            let last = match last.is_empty() {
                true => "no attempt had failed yet",
                false => last.as_str(),
            };
            return Err(KlippyError::Connection(format!(
                "MCU '{}': the reopen after a reset was aborted by an exit request: {last}",
                config.name
            )));
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
        let uptime = seed_clock_base(&mcu)
            .await
            .map_err(|err| self.connect_failure(err))?;
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
    /// That reading is not optional: the bring-up fails before this is reached
    /// when the firmware cannot give one.
    fn install_clock(
        &self,
        mcu: &Arc<Mcu>,
        reactor: &Arc<dyn Reactor>,
        sent_time: f64,
        uptime: u64,
    ) {
        let clock = Arc::new(McuClock::new(Arc::clone(mcu), Arc::clone(reactor)));
        clock.seed(sent_time, uptime as i64);
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
        // only reads the estimators, it never queries). It goes in with the
        // seed it corrects: a poll on an unseeded regression would have
        // nothing to correct, and before identify there is no clock to poll
        // at all. `register_clock_poll` keeps one timer per object, so the
        // reconnect's install is a no-op for it.
        self.register_clock_poll(reactor.as_ref());
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

    /// Ask for a plain restart when a host-library bridge lost its port tty.
    ///
    /// A bridge port's tty is opened by the **library** when the firmware's
    /// `config_uart` arrives, and the library cannot be re-initialised in place:
    /// the failed open leaves the firmware shut down, and this host build's
    /// firmware has neither `reset` nor `config_reset` to clear it
    /// (`mcu/config.rs::reset_firmware` reports that as unrecoverable). What
    /// re-runs `init` is a fresh mapping, and that is what the printer's restart
    /// path gives: teardown drops every reference, the library unloads, and the
    /// next bring-up loads it again and re-names its ports (`host.rs`'s
    /// registry).
    ///
    /// So this is only the **decision**: when the failure is the port that could
    /// not be opened and every tty the config named is back within
    /// [`bridge_attach::BRIDGE_PORT_RETURN_TIMEOUT`], ask the printer for a
    /// **plain** restart (`request_exit("restart")`) — a `firmware_restart`
    /// would reset the downstream device again, and its re-enumeration is what
    /// lost the tty. A tty that does not come back is a configuration error: the
    /// bring-up reports it unchanged, with no restart.
    ///
    /// The restart is budgeted **once per incident**
    /// ([`Printer::claim_bridge_port_recovery`]): a tty that is there but cannot
    /// be opened (a directory, a node without permission) is back from the wait
    /// at once, and an unbudgeted restart would loop forever. The budget is the
    /// printer's, so it survives the restart; a session that comes up gives it
    /// back (`connect`'s success path), so the next real loss recovers once more.
    async fn recover_lost_bridge_port(&self, config: &McuConfig, err: KlippyError) -> KlippyError {
        // Only a host-library bridge names its ports; only a port that could not
        // be opened is worth waiting for; and only the reason this very session
        // recorded counts — anything else is reported as it is.
        if !matches!(config.transport, Transport::Host { .. }) || config.host_port_paths.is_empty()
        {
            return err;
        }
        let Some(reason) = self.connect_shutdown() else {
            return err;
        };
        if !bridge_attach::is_port_open_failure(&reason) {
            return err;
        }
        let paths: Vec<String> = config
            .host_port_paths
            .iter()
            .map(|(_, path)| path.clone())
            .collect();
        if !bridge_attach::wait_for_port_paths(&paths, bridge_attach::PORT_RETURN_BUDGET).await {
            return err;
        }
        let Some(printer) = self.printer.upgrade() else {
            return err;
        };
        self.restart_for_bridge_port(&printer, config, err)
    }

    /// Ask for a plain restart when a device behind a host-library bridge's port
    /// lost that tty while it was connecting.
    ///
    /// The downstream device is reset by `RESTART_DEVICE` during its **own**
    /// connect (`mcu/restart.rs`), and a native-USB board re-enumerates: the
    /// bridge is still up, but the fd it holds for the port is dead, so the
    /// frames the child sends fail (`bridge serial: write failed: (5)Input/output
    /// error`) or reach nobody and its naming handshake times out. Nothing in
    /// this session can mend that — the library would have to be re-mapped (see
    /// [`McuObject::recover_lost_bridge_port`]) — so the same bounded wait and
    /// the same budget are used, keyed by the **child's** own section.
    ///
    /// Only a port whose bridge is a host library has a tty the host can wait
    /// for ([`bridge_attach::host_bridge_port_path`]), which is what keeps this
    /// off `serial:`/CAN/`test:` devices.
    async fn recover_lost_child_port(&self, config: &McuConfig, err: KlippyError) -> KlippyError {
        let Some(printer) = self.printer.upgrade() else {
            return err;
        };
        let Some(path) = bridge_attach::host_bridge_port_path(&printer, config) else {
            return err;
        };
        if !bridge_attach::wait_for_port_paths(&[path], bridge_attach::PORT_RETURN_BUDGET).await {
            return err;
        }
        self.restart_for_bridge_port(&printer, config, err)
    }

    /// Spend this section's one bridge-port recovery on a **plain** restart, or
    /// report `err` unchanged once it is gone.
    ///
    /// The budget is per section and lives on the printer
    /// ([`Printer::claim_bridge_port_recovery`]), so it outlives the restart it
    /// buys. Both triggers — the bridge's own port that could not be opened, and
    /// a downstream device that lost the port while connecting — keep their own,
    /// and each is spent once per incident: a port that is there but cannot be
    /// opened (or whose device never answers) would otherwise restart forever. A
    /// session that comes up gives the budget back (`connect`'s success path), so
    /// the next real loss recovers once more.
    fn restart_for_bridge_port(
        &self,
        printer: &Printer,
        config: &McuConfig,
        err: KlippyError,
    ) -> KlippyError {
        if !printer.claim_bridge_port_recovery(&self.section.identifier()) {
            warn!(
                "MCU '{}': the bridge port has already had its restart; not restarting again",
                config.name
            );
            return err;
        }
        info!(
            "MCU '{}': the bridge port tty came back; restarting the printer to reopen it",
            config.name
        );
        printer.request_exit("restart");
        err
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
        // does (the key is added by its `stats()` collector). `last_stats_time`
        // is this host's stamp on that same report, so it arrives with it and
        // never before.
        if let Some(last) = *self.last_stats.lock().unwrap_or_else(|p| p.into_inner()) {
            if let Value::Object(fields) = &mut status {
                fields.insert("last_stats".to_string(), last.to_json());
                fields.insert("last_stats_time".to_string(), json!(last.eventtime));
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
            // check); that parse is reused when it is there, because a `uart:`/
            // `can:` section cannot be re-parsed without the whole config (its
            // port-section check needs a sibling, `McuObject::parsed`). An object
            // built outside the loader falls back to parsing its section on the
            // printer's tracker, which serves every transport whose parse does
            // not need a sibling.
            let mut config = match self.parsed_config() {
                Some(config) => config,
                None => {
                    let access = self
                        .printer
                        .upgrade()
                        .map(|printer| printer.access_tracking())
                        .unwrap_or_else(AccessTracking::shared);
                    let wrapper = ConfigWrapper::new(&self.section, access);
                    McuConfig::new(&wrapper).map_err(KlippyError::Config)?
                }
            };
            // An MCU must hold the id it was assigned at load before its
            // transport can be opened — a CAN MCU cannot be addressed without
            // its node id, and every transport's id is what this layer guards
            // (`mcu/ids.rs`). Parsing only re-derives an explicit
            // `canbus_nodeid`, so the definitive id is the one the object
            // registered with; a missing one is a precondition failure of the
            // bring-up, not an option to paper over.
            config.id = Some(self.id().ok_or_else(|| {
                KlippyError::Connection(format!(
                    "MCU '{}' has no id assigned; cannot open its transport",
                    config.name
                ))
            })?);
            // Same pattern as the id: the shared-port baud, the host-mux attach
            // and the tty each bridge port opens were decided for the whole
            // config at load, so connecting only reads them back rather than
            // reclassifying every `[mcu]` section again.
            config.shared_baud = self.shared_baud();
            config.host_mux = self.host_mux();
            config.host_port_paths = self.host_port_paths();
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
            if let Some(reason) =
                super::restart::check_usb_power(&config, &tokio::runtime::Handle::current()).await
            {
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
                // A device behind a bridge port has no transport of its own, so
                // its reset travels the bridge's link — the port-level command
                // the config chose (`config.bridge_restart`). It goes here,
                // before the attach pass opens the device: the board is about to
                // reboot, so the route that pass establishes would be stale, and
                // `reset_firmware` above is a no-op for it. Resolving the port
                // against the bridge's firmware port table is where an old
                // firmware (no port table) or a mismatched oid is refused — a
                // config error, with the downgrade being `restart_method:
                // command`.
                if let Some(restart) = config.bridge_restart {
                    let printer = self.printer.upgrade().ok_or_else(|| {
                        KlippyError::Internal(format!(
                            "MCU '{}': the printer is gone; cannot reset its bridge port",
                            config.name
                        ))
                    })?;
                    super::restart::reset_bridge_firmware(&printer, &config, restart)?;
                }
            }
            // A bring-up that follows a firmware restart finds the board
            // rebooting — `command` sent the reset on the live connection
            // (`before_firmware_restart`), and the physical methods just switched
            // it — so the port is retried until it comes back. A plain start
            // waits for the port the same way: a host that comes up before the
            // kernel has re-enumerated the tty, or a board that was power-cycled
            // under it, must not leave the printer in `error` after one failed
            // open (`McuConfig::open_retrying`). Handshake and configuration
            // stay single-shot in both cases, so a protocol error is reported as
            // one.
            let mcu = if self.is_firmware_restart() {
                match self.open_and_connect(&config).await {
                    Ok(mcu) => mcu,
                    Err(err) => {
                        if usb_reset {
                            self.usb_reset_unusable(&config, &err.to_string());
                        }
                        let err = self.connect_failure(err);
                        return Err(self.recover_lost_child_port(&config, err).await);
                    }
                }
            } else {
                let interface = match self.open_interface(&config, true).await {
                    Ok(interface) => interface,
                    Err(err) => return Err(self.recover_lost_child_port(&config, err).await),
                };
                match Mcu::connect(&config.name, interface).await {
                    Ok(mcu) => mcu,
                    Err(err) => {
                        if usb_reset {
                            self.usb_reset_unusable(&config, &err.to_string());
                        }
                        let err = self.connect_failure(KlippyError::Connection(err.to_string()));
                        return Err(self.recover_lost_child_port(&config, err).await);
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
            // (`Mcu::estimated_clock`). A firmware that cannot answer it has no
            // anchor at all, which upstream treats as a failed connect
            // (`seed_clock_base`), not as a session to carry on without.
            let reactor = self.printer.upgrade().map(|printer| printer.reactor());
            let sent_time = reactor
                .as_ref()
                .map(|reactor| reactor.monotonic())
                .unwrap_or(0.0);
            let uptime = seed_clock_base(&mcu)
                .await
                .map_err(|err| self.connect_failure(err))?;
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
            let (mcu, configured) = match self
                .handshake_with_in_place_resets(&config, mcu, built)
                .await
            {
                Ok(pair) => pair,
                // A host-library bridge whose port tty could not be opened is
                // the one failure a plain restart can mend: the library cannot
                // be re-initialised in place, so unloading and loading it again
                // is what re-runs `init` ([`McuObject::recover_lost_bridge_port`]).
                Err(err) => return Err(self.recover_lost_bridge_port(&config, err).await),
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
            // The firmware sends periodic `stats` reports with scheduler
            // timing; keep the latest as `last_stats` (and log it) so
            // `objects/query` can report the load. Binding is unconditional: the
            // report has to be consumed whether or not the dictionary publishes
            // the two constants that turn its counters into seconds (a missing
            // constant costs the tick numbers only, and is reported inside).
            register_stats(&mcu, Arc::clone(&self.last_stats), self.printer.clone())
                .map_err(|err| KlippyError::Internal(err.to_string()))?;
            self.set_status(&mcu);
            // This session is up: an MCU that had spent its one bridge-port
            // recovery has it back for the next time its port tty goes away.
            if let Some(printer) = self.printer.upgrade() {
                printer.clear_bridge_port_recovery(&self.section.identifier());
            }
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

/// Seed the firmware-clock estimate from one `get_uptime`, or fail the bring-up.
///
/// Upstream's clock sync reads the clock at connect too
/// (`MCUConnectHelper._attach` → `clocksync.connect`, `klippy/clocksync.py:33-36`);
/// this host keeps only the base point, for resources that need a "now" clock
/// without consulting the print-time estimate. The 64-bit `get_uptime` is used
/// rather than `get_clock` because the 32-bit counter wraps every few minutes at
/// a typical `CLOCK_FREQ`.
///
/// A failed reading is **not** a session to carry on without: the anchors of
/// the mapping (`ClockEstimator::clock_at_sample`, `clock_sample_time`) would
/// stay at zero, so print time 0 would map onto firmware clock 0 — the past, as
/// the firmware's scheduler reads it, and every move gated on it comes back
/// "Timer too close" (or fires at once). Upstream has no fallback for this
/// step either: `clocksync.connect` sends `get_uptime` and lets the exception
/// end the connect. The error names the MCU and the message that did not
/// answer, which is the only thing the user can act on.
async fn seed_clock_base(mcu: &Mcu) -> Result<u64, KlippyError> {
    if !mcu.has_message(GetUptime::NAME) {
        return Err(KlippyError::Connection(format!(
            "MCU '{}': the firmware does not publish 'get_uptime', so its clock base \
             cannot be read and the connect is refused",
            mcu.name()
        )));
    }
    let uptime = mcu
        .call_msg::<GetUptime, Uptime>(&GetUptime, CLOCK_BASE_TIMEOUT)
        .await
        .map_err(|err| {
            KlippyError::Connection(format!(
                "MCU '{}': 'get_uptime' went unanswered, so its clock base cannot be read: {err}",
                mcu.name()
            ))
        })?;
    mcu.set_clock_base(uptime.clock64());
    Ok(uptime.clock64())
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
    // the device is still only opened by `connect`. The result is kept on the
    // object because `connect` cannot re-parse a `uart:`/`can:` section — its
    // port-section check needs the whole config, which a stored section alone
    // does not have (`McuObject::parsed`, `McuObject::mcu_config`).
    let mut parsed = McuConfig::new(config)?;
    // Every `[mcu]` registers with the shared id allocator here, while the
    // config is loaded, so the whole sequence is known before any of them
    // connects (upstream registers a CAN MCU's uuid in `MCU.__init__`,
    // `klippy/mcu.py:784`; the id is generalized to every transport in
    // `mcu/ids.rs`). Before the first one takes an id, the allocator reserves
    // every explicit `canbus_nodeid` in the config, so the auto ids the later
    // sections receive step over them — an explicit claim is honoured wherever
    // it sits in the file. Every CAN family link brings its uuid and any
    // explicit `canbus_nodeid`: the wire (`canbus_fd`) shapes **how** the
    // blocks travel, not who is addressed, so a CAN FD MCU takes the same
    // allocation as a classical-wire one; another transport takes the lowest
    // free id from the same sequence, with no concrete allocation of its own
    // yet.
    let allocator = ids::ensure(printer)?;
    allocator.reserve_explicit(config);
    let id = match &parsed.transport {
        Transport::Can { uuid, .. } | Transport::CanFd { uuid, .. } => {
            allocator.register_can(&parsed.name, *uuid, parsed.id)?
        }
        // A CAN port's member is addressed by its uuid exactly as a direct
        // CAN link is, so it takes the same allocation. Rule 6 made one
        // mandatory for it at parse (`port_grouping::validate`, which the
        // transport parse runs), so the parse pinned it.
        Transport::BridgePort {
            uuid,
            kind: PortKind::Can,
            ..
        } => allocator.register_can(
            &parsed.name,
            uuid.expect("a CAN port member writes a uuid (checked at parse, rule 6)"),
            parsed.id,
        )?,
        _ => allocator.register(&parsed.name)?,
    };
    // The first `[mcu]` section brings the `error_mcu` module with it, as
    // upstream's `MCU.__init__` does (`klippy/mcu.py:1159`); the rest find it
    // already there. It has to exist before any MCU can fail.
    crate::core::klippy::extras::error_mcu::ensure(printer)?;
    let object = Arc::new(
        McuObject::new(config.section().clone(), printer)
            .map_err(|err| ConfigError::new(err.to_string()))?,
    );
    // The id assigned at load is the one the machine connects with; `connect`
    // refuses to open a transport without it.
    object.set_id(id);
    // Whether this MCU's `serial:` path is addressed by id is a property of the
    // whole config, resolved once for every `[mcu]` by the grouping pass — the
    // verdict the transport splits on, and the rate the path resolved to (the
    // lowest its sections write). A path two or more sections name is shared;
    // a bridge's path is addressed by id too, even though one section names it,
    // because its downstream devices' frames travel it
    // (`SerialPorts::is_id_addressed`). Both are stamped on the object so
    // `connect` can hand them to the transport without reclassifying, and the
    // transport never compares the rate against a section's own `baud`: doing
    // so sent a section that wrote the higher rate down the exclusive arm, a
    // second opener on a tty another MCU already held. A path neither shape
    // covers stays exclusive (`None`), and its transport opens exactly as
    // before — no assignment frame is sent.
    let ports = serial_ports::ensure(printer, config)?;
    let shared_baud = match &parsed.transport {
        Transport::Serial { path, .. } if ports.is_id_addressed(path) => ports.effective_baud(path),
        _ => None,
    };
    object.set_shared_baud(shared_baud);
    // Whether this MCU's `host_library:` path is muxed is likewise the whole
    // config's call, resolved once by the path grouping pass. A muxed path
    // brings the attach its identity pair: the library's own device gets the
    // path's own identity, and the attach names the section's `uuid` when it
    // wrote one (the board the bridge routes to) else that same identity. A path
    // the config does not call muxed stays `None` and opens with bare frames.
    //
    // Only a host-library section asks for the grouping (as `McuConfig::new`
    // only calls `host_libraries::validate` for one): a config with no
    // `host_library:` has no path to group and no identity to assign, so the
    // object is not brought in for it.
    let host_mux = match &parsed.transport {
        Transport::Host { library, uuid } => {
            let libraries = host_libraries::ensure(printer, config)?;
            libraries.is_mux(library).then(|| HostMux {
                identity: libraries
                    .identity(library)
                    .expect("a path a section names has an identity"),
                named: libraries
                    .named_uuid(library, *uuid)
                    .expect("a path a section names names a device"),
            })
        }
        _ => None,
    };
    object.set_host_mux(host_mux);
    // Whether this MCU hangs on a bridge downstream port — shared or
    // exclusive, legacy or named — is the whole config's call too
    // (`mcu/port_grouping.rs`). Only a `uart:`/`can:` section asks for the
    // grouping, as the parse only calls `port_grouping::validate` for one (the
    // precedent `host_libraries::ensure` above sets: a config with no port
    // reference has no port to group). The attach pass reads this grouping at
    // connect to derive the port's `flags` (`mcu/bridge_attach.rs`), which is
    // why it is registered here rather than built on demand there.
    if matches!(parsed.transport, Transport::BridgePort { .. }) {
        port_grouping::ensure(printer, config)?;
    }
    // Keep the parse for `connect`: the load-time stamps are part of what the
    // connection will use, so they are covered back onto it here too.
    parsed.id = Some(id);
    parsed.shared_baud = shared_baud;
    parsed.host_mux = host_mux;
    object.set_parsed_config(parsed);
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
    use crate::core::klippy::config::Config;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::{
        FrameMock, FrameRecorder, MappingEntry,
    };
    use crate::core::klippy::interface::devices::serial::{self, DEFAULT_BAUD};
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::msg::proto::Payload;
    use crate::core::klippy::printer::{Printer, PrinterState};
    use crate::core::klippy::reactor::{ManualReactor, Reactor};
    use crate::core::klippy::test_util::wait_until;
    use std::io::Write;
    use tokio::time::{timeout, Duration};

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
        // Wait for the receive task to decode and dispatch the shutdown frame,
        // on a bound rather than in a fixed window the scheduler can miss.
        wait_until(
            Duration::from_secs(1),
            Duration::from_millis(5),
            || printer.get_state_message().category == PrinterState::Shutdown,
            "the firmware's shutdown reached the printer",
        )
        .await;

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
        // The loader assigns every `[mcu]` an id before it can be opened; the
        // test builds the object by hand, so the id is set to let the connect
        // get past the precondition and fail where this test wants it to — at
        // the unusable port.
        object.set_id(4);

        let err = object.connect().await.unwrap_err();

        // The section's own parser names the port; the object does not have to.
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
    }

    /// A plain start waits for a port that is not there yet, and an exit request
    /// ends that wait instead of leaving the process on the budget.
    ///
    /// Both halves are the fix: one failed open used to leave the printer in
    /// `error` for a board that was still re-enumerating, and the wait must not
    /// outlive a shutdown either.
    #[tokio::test]
    async fn test_a_missing_port_is_retried_and_an_exit_request_ends_the_wait() {
        let start = std::time::Instant::now();
        let waited = connect_to_a_missing_port(&printer()).await;
        assert!(
            waited.to_string().contains("/dev/not-a-serial-port"),
            "{waited}"
        );
        assert!(waited.to_string().contains("gave up"), "{waited}");
        assert!(
            start.elapsed() >= serial::OPEN_RETRY_BUDGET,
            "one attempt would have returned at once, took {:?}",
            start.elapsed()
        );

        // Asked to leave before the open, the same connect ends with the retry's
        // abort rather than sitting on the budget.
        let printer = printer();
        printer.request_exit("exit");
        let start = std::time::Instant::now();
        let aborted = connect_to_a_missing_port(&printer).await;
        assert!(aborted.to_string().contains("aborted"), "{aborted}");
        assert!(
            start.elapsed() < serial::OPEN_RETRY_BUDGET,
            "an exit request must not wait the budget, took {:?}",
            start.elapsed()
        );
    }

    /// The reopen loop after a `reset` waits far longer than the single open
    /// above, so an exit request has to end it too.
    ///
    /// One round may spend the whole identify budget (10 s) on a board that
    /// never answers, and the loop runs [`RECONNECT_ATTEMPTS`] rounds: the wait a
    /// shutdown would sit on is ~205 s. The port here fails at once, so the
    /// loop's own delays are the only time it spends — and the check after the
    /// first sleep is enough to end it.
    #[tokio::test]
    async fn test_an_exit_request_ends_the_reopen_after_a_reset() {
        let printer = printer();
        printer.request_exit("exit");
        let mut section = section(None);
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/dev/not-a-serial-port".to_string()),
        );
        let object = McuObject::new(section, &printer).unwrap();
        let config = McuConfig::new(&ConfigWrapper::untracked(&object.section)).unwrap();

        let start = std::time::Instant::now();
        let err = tokio::time::timeout(Duration::from_secs(10), object.open_and_connect(&config))
            .await
            .expect("the exit request ends the loop instead of running its budget")
            .err()
            .expect("the board never comes back");

        // Which failure it is says why the wait ended: the loop's own verdict
        // names the MCU and the exit, not just the last open error.
        assert!(err.to_string().contains("exit request"), "{err}");
        assert!(err.to_string().contains("'mcu'"), "{err}");
        // The delays alone are 20 × 250 ms = 5 s; ending on the first check is
        // what keeps the shutdown off them.
        assert!(
            start.elapsed() < RECONNECT_DELAY * 4,
            "the loop ran on for {:?}",
            start.elapsed()
        );
    }

    /// The wait happens on the blocking pool, so a connect that is sitting on a
    /// port that is not there yet does not stop the runtime it was called from.
    ///
    /// The machine runtime runs two workers (`klippy.rs`), one of which carries
    /// the reactor dispatcher: a retry that slept on a worker would stall every
    /// timer, every send/receive task and the API for the whole budget. A
    /// current-thread runtime is the sharpest form of the test — nothing at all
    /// runs on it while a task blocks it.
    #[tokio::test]
    async fn test_waiting_for_a_port_does_not_block_the_runtime() {
        let ticks = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let ticker = {
            let ticks = Arc::clone(&ticks);
            tokio::spawn(async move {
                loop {
                    tokio::time::sleep(Duration::from_millis(20)).await;
                    ticks.fetch_add(1, Ordering::SeqCst);
                }
            })
        };

        let waited = connect_to_a_missing_port(&printer()).await;
        ticker.abort();

        assert!(waited.to_string().contains("gave up"), "{waited}");
        // The budget is 300 ms in a test build: a runtime that kept running
        // ticks ~15 times, one that was blocked ticks none.
        let ticks = ticks.load(Ordering::SeqCst);
        assert!(
            ticks >= 5,
            "the runtime was blocked for the wait ({ticks} ticks)"
        );
    }

    /// Connect a `[mcu]` on a serial port no device is behind, and return the
    /// error it reports.
    async fn connect_to_a_missing_port(printer: &Arc<Printer>) -> KlippyError {
        let mut section = section(None);
        section.parameters.insert(
            "serial".to_string(),
            crate::core::klippy::config::ConfigValue::Single("/dev/not-a-serial-port".to_string()),
        );
        let object = McuObject::new(section, printer).unwrap();
        object.set_id(4);
        object.connect().await.unwrap_err()
    }

    /// The id gate itself: an object built outside the loader holds no id, and
    /// `connect` refuses to open anything for it — the precondition the loader
    /// (and every `set_id`) satisfies. Deleting the gate makes this test fail;
    /// the port is never even opened.
    #[tokio::test]
    async fn test_connect_without_an_assigned_id_is_refused() {
        let mut section = section(None);
        section.parameters.insert(
            "serial".to_string(),
            crate::core::klippy::config::ConfigValue::Single("/dev/never-opened".to_string()),
        );
        let object = McuObject::new(section, &printer()).unwrap();
        // Deliberately no `set_id`: the hand-built object the gate exists for.

        let err = object.connect().await.unwrap_err();
        let text = err.to_string();
        assert!(text.contains("has no id assigned"), "{text}");
        assert!(text.contains("cannot open its transport"), "{text}");
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

    /// Load `text` through the real loader and return the printer, so its
    /// `[mcu]` objects can be looked up by section name.
    fn load_printer(text: &str) -> Arc<Printer> {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        printer
    }

    /// The shared-port baud stamped on the object `name` registered.
    fn shared_baud_of(printer: &Arc<Printer>, name: &str) -> Option<u32> {
        printer
            .lookup_object_as::<McuObject>(name)
            .expect("the section registered")
            .shared_baud()
    }

    /// A `serial:` path two `[mcu]` sections name is shared, and both objects
    /// carry the same rate: the grouping is resolved once at load.
    #[test]
    fn test_a_serial_path_two_mcus_name_is_stamped_shared() {
        let printer = load_printer(
            "[mcu]\nserial: /dev/not-opened\n\
             [mcu tool]\nserial: /dev/not-opened\n",
        );
        assert_eq!(shared_baud_of(&printer, "mcu"), Some(DEFAULT_BAUD));
        assert_eq!(shared_baud_of(&printer, "mcu tool"), Some(DEFAULT_BAUD));
    }

    /// A `serial:` path one `[mcu]` alone names stays exclusive (`None`), the
    /// shape that sends no assignment frame.
    #[test]
    fn test_a_serial_path_one_mcu_names_stays_exclusive() {
        let printer = load_printer(
            "[mcu]\nserial: /dev/not-opened\n\
             [mcu tool]\nserial: /dev/other\n",
        );
        assert_eq!(shared_baud_of(&printer, "mcu"), None);
        assert_eq!(shared_baud_of(&printer, "mcu tool"), None);
    }

    /// Sections on one path that write different rates share the **lowest**,
    /// and both objects carry it.
    #[test]
    fn test_a_shared_path_is_stamped_with_the_lowest_baud() {
        let printer = load_printer(
            "[mcu]\nserial: /dev/not-opened\nbaud: 250000\n\
             [mcu tool]\nserial: /dev/not-opened\nbaud: 115200\n",
        );
        assert_eq!(shared_baud_of(&printer, "mcu"), Some(115200));
        assert_eq!(shared_baud_of(&printer, "mcu tool"), Some(115200));
    }

    /// The id the loader's allocator handed the `[mcu]` section `name`.
    fn id_of(printer: &Arc<Printer>, name: &str) -> Option<u32> {
        printer
            .lookup_object_as::<McuObject>(name)
            .expect("the section registered")
            .id()
    }

    /// A CAN FD MCU is addressed the way a classical-wire one is: the load
    /// registers it with its uuid (`register_can`), so its id comes from the
    /// same sequence as a serial MCU's and interleaves with it in declaration
    /// order.
    #[test]
    fn test_the_can_family_links_register_with_their_uuid() {
        let printer = load_printer(
            "[mcu]\nuuid: 11aa22bb33cc\ncanbus_fd: true\n\
             [mcu tool]\nserial: /dev/not-opened\n\
             [mcu probe]\nuuid: 44dd55ee66ff\ncanbus_fd: false\n",
        );
        // `NODEID_FIRST` (4) upward, one sequence for every transport.
        assert_eq!(id_of(&printer, "mcu"), Some(4));
        assert_eq!(id_of(&printer, "mcu tool"), Some(5));
        assert_eq!(id_of(&printer, "mcu probe"), Some(6));
    }

    /// The allocation is `register_can`'s, which is the one place a uuid is
    /// remembered: two CAN family sections naming one identity are one board
    /// claimed twice, exactly as two classical-wire sections would be.
    #[test]
    fn test_a_uuid_is_claimed_once_across_the_can_family() {
        let (config, _) = Config::from_text(
            "[mcu a]\nuuid: 11aa22bb33cc\ncanbus_fd: true\n\
             [mcu b]\nuuid: 11aa22bb33cc\ncanbus_fd: false\n",
        )
        .expect("the test config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        let err = printer.load_config(&config).unwrap_err();
        assert!(err.to_string().contains("Duplicate uuid"), "{err}");
    }

    /// What the load-time host-library grouping stamped on the object `name`
    /// registered (`mcu/host_libraries.rs`).
    fn host_mux_of(printer: &Arc<Printer>, name: &str) -> Option<HostMux> {
        printer
            .lookup_object_as::<McuObject>(name)
            .expect("the section registered")
            .host_mux()
    }

    /// A `host_library:` path one section names without a `uuid` is not muxed:
    /// the attach is left `None`, so the transport opens bare frames.
    #[test]
    fn test_a_bare_host_library_path_is_not_stamped_mux() {
        let printer = load_printer("[mcu]\nhost_library: /lib/libklipper_host.so\n");
        assert_eq!(host_mux_of(&printer, "mcu"), None);
    }

    /// A `host_library:` path whose section writes a `uuid` is muxed: the attach
    /// carries the path's own identity and names the section's `uuid`.
    #[test]
    fn test_a_host_library_path_with_a_uuid_is_stamped_mux() {
        let printer =
            load_printer("[mcu]\nhost_library: /lib/libklipper_host.so\nuuid: 11aa22bb33cc\n");
        let mux = host_mux_of(&printer, "mcu").expect("the path is muxed");
        assert_eq!(mux.named, [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]);
        assert_ne!(mux.identity, mux.named, "the bridge is not the board");
        assert_ne!(mux.identity, [0; 6], "zero is the library's unnamed value");
    }

    /// Two sections on one path are muxed, sharing the path's identity: the one
    /// without a `uuid` names the bridge's own device, the one with a `uuid`
    /// names its board.
    #[test]
    fn test_two_host_sections_stamp_the_mux_naming_rule() {
        let printer = load_printer(
            "[mcu]\nhost_library: /lib/libklipper_host.so\n\
             [mcu tool]\nhost_library: /lib/libklipper_host.so\nuuid: 22bb33cc44dd\n",
        );
        let bridge = host_mux_of(&printer, "mcu").expect("muxed");
        let tool = host_mux_of(&printer, "mcu tool").expect("muxed");
        assert_eq!(bridge.identity, tool.identity, "the identity is the path's");
        assert_eq!(bridge.named, bridge.identity, "no uuid names the bridge");
        assert_eq!(tool.named, [0x22, 0xbb, 0x33, 0xcc, 0x44, 0xdd]);
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
        // (`klippy/mcu.py:974-975`); before any report both keys are absent.
        let printer = printer();
        let object = McuObject::new(section(None), &printer).unwrap();
        assert!(object.get_status(0.0).get("last_stats").is_none());
        assert!(object.get_status(0.0).get("last_stats_time").is_none());

        *object.last_stats.lock().unwrap_or_else(|p| p.into_inner()) = Some(LastStats {
            mcu_tick_avg: 1.5e-6,
            mcu_tick_stddev: 2.5e-7,
            mcu_tick_awake: 0.25,
            eventtime: 12.5,
            bytes_read: 1_024,
            bytes_write: 2_048,
        });

        let status = object.get_status(0.0);
        assert_eq!(status["last_stats"]["mcu_tick_awake"], 0.25);
        assert_eq!(status["last_stats"]["mcu_tick_avg"], 1.5e-6);
        assert_eq!(status["last_stats"]["bytes_read"], 1_024);
        assert_eq!(status["last_stats"]["bytes_write"], 2_048);
        assert_eq!(status["last_stats_time"], 12.5);
    }

    /// Frame carrying one `stats` report: id -1 (the id the firmware's own
    /// dictionary publishes, a signed VLQ byte), then `count`, `sum`, `sumsq`.
    fn stats_frame(seq: u8, count: u32, sum: u32, sumsq: u32) -> Frame {
        let mut payload = Payload::new();
        payload.push_i16(-1).unwrap();
        payload.push_u32(count).unwrap();
        payload.push_u32(sum).unwrap();
        payload.push_u32(sumsq).unwrap();
        Frame::new(seq, payload.into_raw())
    }

    /// A printer whose clock the test steps, for `last_stats_time`.
    fn printer_with_clock() -> (Arc<Printer>, Arc<ManualReactor>) {
        let clock = Arc::new(ManualReactor::new());
        let reactor: Arc<dyn Reactor> = Arc::clone(&clock) as Arc<dyn Reactor>;
        let printer = Arc::new(Printer::new(reactor));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        (printer, clock)
    }

    /// An MCU attached to `object`'s chip whose `get_uptime` answers are the
    /// given `stats` reports, in order, with [`register_stats`] keeping the
    /// latest in the object's slot.
    ///
    /// A `stats` report is unsolicited on the wire — the firmware sends it every
    /// five seconds (`src/basecmd.c:332-334`). Answering an unrelated command is
    /// only how a test gets one to arrive, the same trick `event/mod.rs` uses.
    fn attached_with_stats(
        object: &McuObject,
        printer: &Arc<Printer>,
        reports: &[(u32, u32, u32)],
    ) -> Arc<Mcu> {
        let mappings = reports
            .iter()
            .enumerate()
            .map(|(seq, &(count, sum, sumsq))| MappingEntry {
                input: Frame::new(seq as u8, vec![4]),
                outputs: vec![stats_frame(seq as u8, count, sum, sumsq)],
            })
            .collect();
        let mcu = Arc::new(Mcu::for_test(
            "mcu",
            Interface::new(FrameMock::new(mappings)),
        ));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "commands": {"get_uptime": 4},
                "responses": {"stats count=%u sum=%u sumsq=%u": -1},
                "config": {"CLOCK_FREQ": 72000000, "STATS_SUMSQ_BASE": 4},
            }))
            .unwrap(),
        )
        .unwrap();
        object.chip.attach(Arc::clone(&mcu));
        register_stats(
            &mcu,
            Arc::clone(&object.last_stats),
            Arc::downgrade(printer),
        )
        .unwrap();
        mcu
    }

    /// The object's report, waiting (bounded) until one stamped `eventtime` has
    /// arrived.
    ///
    /// Waiting on the stamp and not just on "a report arrived" is what tells a
    /// replacement from the report it replaced: the slot is already filled by
    /// the earlier one.
    async fn last_report_at(object: &McuObject, eventtime: f64) -> LastStats {
        let wait = wait_until(
            Duration::from_secs(1),
            Duration::from_millis(5),
            || {
                object
                    .last_stats
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .is_some_and(|last| last.eventtime == eventtime)
            },
            "a `stats` report must arrive",
        );
        timeout(Duration::from_secs(5), wait)
            .await
            .expect("the wait is bounded");
        object
            .last_stats
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .expect("just waited for it")
    }

    #[tokio::test]
    async fn test_a_stats_report_becomes_last_stats_with_its_arrival_time() {
        let (printer, clock) = printer_with_clock();
        let object = McuObject::new(section(None), &printer).unwrap();
        let mcu = attached_with_stats(&object, &printer, &[(100, 3000, 250_000)]);

        // The report arrives at 2.5 s, so the stamp cannot be mistaken for the
        // "no report yet" absence of the key.
        clock.advance(2.5);
        mcu.send_msg(&GetUptime).unwrap();
        let last = last_report_at(&object, 2.5).await;

        assert_eq!(last.eventtime, 2.5, "stamped with the machine's clock");
        assert!((last.mcu_tick_awake - 3000.0 / 72_000_000.0).abs() < 1e-18);
        let status = object.get_status(0.0);
        assert_eq!(status["last_stats_time"], 2.5);
        assert_eq!(
            status["last_stats"]["mcu_tick_awake"].as_f64().unwrap(),
            3000.0 / 72_000_000.0
        );
    }

    #[tokio::test]
    async fn test_a_later_stats_report_replaces_the_earlier_one() {
        let (printer, clock) = printer_with_clock();
        let object = McuObject::new(section(None), &printer).unwrap();
        let mcu = attached_with_stats(
            &object,
            &printer,
            &[(100, 3000, 250_000), (200, 8000, 400_000)],
        );

        clock.advance(2.5);
        mcu.send_msg(&GetUptime).unwrap();
        assert_eq!(last_report_at(&object, 2.5).await.eventtime, 2.5);

        clock.advance(5.0);
        mcu.send_msg(&GetUptime).unwrap();
        let second = last_report_at(&object, 7.5).await;

        assert_eq!(
            second.eventtime, 7.5,
            "the later report is the one reported"
        );
        assert!((second.mcu_tick_awake - 8000.0 / 72_000_000.0).abs() < 1e-18);
    }

    /// An MCU on `pty`'s line, with [`register_stats`] keeping its reports in
    /// `object`'s slot.
    ///
    /// Unlike [`attached_with_stats`] this transport has a real wire, so its
    /// byte counters move with what the test writes.
    fn attached_over_a_line(
        object: &McuObject,
        printer: &Arc<Printer>,
        pty: &Pty,
        name: &str,
    ) -> Arc<Mcu> {
        let device = serial::SerialDevice::open(&pty.path, DEFAULT_BAUD).unwrap();
        let mcu = Arc::new(Mcu::for_test(name, Interface::from_serial(device)));
        mcu.install_dictionary(
            Dictionary::from_json(json!({
                "commands": {"get_uptime": 4},
                "responses": {"stats count=%u sum=%u sumsq=%u": -1},
                "config": {"CLOCK_FREQ": 72000000, "STATS_SUMSQ_BASE": 4},
            }))
            .unwrap(),
        )
        .unwrap();
        object.chip.attach(Arc::clone(&mcu));
        register_stats(
            &mcu,
            Arc::clone(&object.last_stats),
            Arc::downgrade(printer),
        )
        .unwrap();
        mcu
    }

    #[tokio::test]
    async fn test_last_stats_carries_the_wire_bytes_of_its_own_line() {
        // The utilization a client computes is `(Δbytes_read + Δbytes_write)`
        // over the interval: the status has to name the line's own counters, not
        // another MCU's and not a frame count. The two reports here are of
        // different lengths, so a mix-up between the lines would also show.
        let (printer, clock) = printer_with_clock();
        let first = McuObject::new(section(None), &printer).unwrap();
        let second = McuObject::new(section(Some("extra")), &printer).unwrap();
        let first_pty = Pty::new();
        let second_pty = Pty::new();
        attached_over_a_line(&first, &printer, &first_pty, "mcu");
        attached_over_a_line(&second, &printer, &second_pty, "extra");

        let first_report = stats_frame(0, 100, 3000, 250_000).raw_bytes();
        let second_report = stats_frame(1, 5, 100, 2500).raw_bytes();
        assert_ne!(
            first_report.len(),
            second_report.len(),
            "the two lines must differ in traffic to tell them apart"
        );
        clock.advance(2.5);
        (&first_pty.master).write_all(&first_report).unwrap();
        (&second_pty.master).write_all(&second_report).unwrap();
        last_report_at(&first, 2.5).await;
        last_report_at(&second, 2.5).await;

        let first_status = first.get_status(0.0);
        let second_status = second.get_status(0.0);
        assert_eq!(
            first_status["last_stats"]["bytes_read"],
            first_report.len() as u64
        );
        assert_eq!(
            second_status["last_stats"]["bytes_read"],
            second_report.len() as u64
        );
        // Nothing has been sent on either line yet, so neither has written.
        assert_eq!(first_status["last_stats"]["bytes_write"], 0);
        assert_eq!(second_status["last_stats"]["bytes_write"], 0);
        // Each line's own load, from its own report.
        assert_eq!(
            first_status["last_stats"]["mcu_tick_awake"]
                .as_f64()
                .unwrap(),
            3000.0 / 72_000_000.0
        );
        assert_eq!(
            second_status["last_stats"]["mcu_tick_awake"]
                .as_f64()
                .unwrap(),
            100.0 / 72_000_000.0
        );
    }

    #[tokio::test]
    async fn test_one_stats_report_fills_every_tick_and_traffic_field() {
        // What a client reads out of one report: the three tick numbers its
        // `stats()` collector reports, the stamp this host dates them with, and
        // the line's own byte counters. The counters are the firmware's; the
        // other three are the host's arithmetic on them.
        let (printer, clock) = printer_with_clock();
        let object = McuObject::new(section(None), &printer).unwrap();
        let pty = Pty::new();
        let mcu = attached_over_a_line(&object, &printer, &pty, "mcu");

        // A block of this host's own, written out before the report: that is
        // what `bytes_write` counts, and what makes it distinguishable from a
        // number read off the wrong field.
        mcu.send_msg(&GetUptime).unwrap();
        mcu.flush(Duration::from_secs(1)).await.unwrap();

        let report = stats_frame(0, 100, 3000, 250_000).raw_bytes();
        clock.advance(2.5);
        (&pty.master).write_all(&report).unwrap();
        let last = last_report_at(&object, 2.5).await;

        let freq = 72_000_000.0;
        let c = 1.0 / (100.0 * freq);
        assert!((last.mcu_tick_avg - 3000.0 * c).abs() < 1e-18);
        let variance = 100.0 * 250_000.0 * 4.0 - 3000.0f64.powi(2);
        assert!((last.mcu_tick_stddev - c * variance.sqrt()).abs() < 1e-18);
        assert!((last.mcu_tick_awake - 3000.0 / freq).abs() < 1e-18);
        assert_eq!(last.eventtime, 2.5, "dated with the machine's clock");
        assert_eq!(last.bytes_read, report.len() as u64);
        assert!(
            last.bytes_write > 0,
            "the block sent above is on the line: {}",
            last.bytes_write
        );
        assert_eq!(object.get_status(0.0)["last_stats_time"], 2.5);
    }

    #[tokio::test]
    async fn test_each_mcu_keeps_its_own_last_stats() {
        // The slot is per object, so two MCUs report their own load — and the
        // one that has not spoken yet keeps reporting nothing.
        let (printer, clock) = printer_with_clock();
        let first = McuObject::new(section(None), &printer).unwrap();
        let second = McuObject::new(section(Some("extra")), &printer).unwrap();
        let first_mcu = attached_with_stats(&first, &printer, &[(100, 3000, 250_000)]);
        let second_mcu = attached_with_stats(&second, &printer, &[(50, 1000, 60_000)]);

        clock.advance(2.5);
        first_mcu.send_msg(&GetUptime).unwrap();
        assert_eq!(last_report_at(&first, 2.5).await.eventtime, 2.5);
        assert!(second.get_status(0.0).get("last_stats").is_none());

        clock.advance(5.0);
        second_mcu.send_msg(&GetUptime).unwrap();
        assert_eq!(last_report_at(&second, 7.5).await.eventtime, 7.5);

        let first_status = first.get_status(0.0);
        assert_eq!(first_status["last_stats_time"], 2.5);
        assert_eq!(
            first_status["last_stats"]["mcu_tick_awake"]
                .as_f64()
                .unwrap(),
            3000.0 / 72_000_000.0
        );
        let second_status = second.get_status(0.0);
        assert_eq!(second_status["last_stats_time"], 7.5);
        assert_eq!(
            second_status["last_stats"]["mcu_tick_awake"]
                .as_f64()
                .unwrap(),
            1000.0 / 72_000_000.0
        );
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

    /// Wait up to a second for the poll's sample to fold into the regression,
    /// then hand over the folded reading.
    ///
    /// [`wait_for_frames`] only proves the `get_clock` query reached the wire:
    /// the fold runs on the round trip's own task (`McuObject::poll_clock`
    /// spawns `McuClock::get_clock`), so a test that reads the estimate has to
    /// wait for the estimator to leave its seed, not for the frame.
    async fn wait_for_clock_fold(clock: &McuClock, seed: i64) -> i64 {
        for _ in 0..200 {
            let last = clock.estimator().last_clock();
            if last != seed {
                return last;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        clock.estimator().last_clock()
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

        /// Make this run a file-output one: the host's start arguments name a
        /// `--debugoutput` file, which is what [`Printer::is_fileoutput`]
        /// reads.
        fn as_fileoutput(&self) {
            let mut args = crate::core::klippy::api::StartArgs::collect(
                "/tmp/printer.cfg",
                None,
                crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
            );
            args.debug_output = Some("_test_output".to_string());
            self.printer.set_start_args(Arc::new(args));
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
        // (`Mcu::record_clock_sample`, behind `McuClock::get_clock`). The
        // assertions below read the estimator, so this waits for the fold
        // itself; that the query goes out is the sibling test's subject.
        let rig = PollRig::new(polling_dictionary(), clock_exchange(21_000_500));
        let before = rig.mcu.estimated_clock().unwrap();
        assert!(before < 21_000_500, "still on the connect seed: {before}");

        let clock = rig.object.clock().unwrap();
        let seed = clock.estimator().last_clock();
        rig.object.register_clock_poll(rig.manual.as_ref());
        rig.manual.advance(1.0);

        let folded = wait_for_clock_fold(&clock, seed).await;
        assert_eq!(folded, 21_000_500, "the sample folded into the regression");
        let got = rig.mcu.estimated_clock().unwrap();
        assert!(got >= 21_000_500, "re-anchored on the sample: {got}");
    }

    #[tokio::test]
    async fn test_a_failed_clock_poll_feeds_nothing_and_keeps_its_schedule() {
        // A query that cannot go through (here `get_clock` is not even in the
        // firmware's dictionary, so it fails before anything is sent) must not
        // disturb anything: no sample, no panic, no state change — and the
        // next interval runs as usual. A failed round is not yet a verdict on
        // the link either: declaring it lost takes more unanswered queries
        // than one round leaves outstanding
        // (`McuObject::check_clock_timeout`).
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

    // -----------------------------------------------------------------------
    // The connect seed: a firmware that cannot give one fails the bring-up
    // -----------------------------------------------------------------------

    /// A `get_uptime` dictionary and the one scripted exchange that answers it,
    /// in the firmware's own two-word form.
    fn uptime_dictionary() -> Value {
        json!({
            "commands": {"get_uptime": 4},
            "responses": {"uptime high=%u clock=%u": 17},
            "config": {"CLOCK_FREQ": 20000000},
        })
    }

    fn uptime_exchange(high: u32, clock: u32) -> Vec<MappingEntry> {
        let mut response = Payload::new();
        response.push_u8(17).unwrap(); // response id, `uptime high=%u clock=%u`
        response.push_u32(high).unwrap();
        response.push_u32(clock).unwrap();
        vec![MappingEntry {
            input: Frame::new(0, vec![4]), // `get_uptime`, no parameters
            outputs: vec![Frame::new(1, response.into_raw())],
        }]
    }

    #[tokio::test]
    async fn test_the_seed_reads_the_uptime_into_the_estimate() {
        // The happy path the bring-up depends on: one `get_uptime` becomes both
        // the `Mcu`'s own estimate (`Mcu::estimated_clock`, for resources that
        // need "now") and the value `connect` hands to `install_clock`.
        let mcu = Mcu::for_test(
            "mcu",
            Interface::new(FrameMock::new(uptime_exchange(1, 500_000))),
        );
        mcu.install_dictionary(Dictionary::from_json(uptime_dictionary()).unwrap())
            .unwrap();
        assert!(mcu.estimated_clock().is_none(), "nothing read yet");

        let seeded = seed_clock_base(&mcu).await.expect("the firmware answers");

        assert_eq!(seeded, (1u64 << 32) | 500_000, "both words are the clock");
        let now = mcu.estimated_clock().expect("the estimate is anchored");
        assert!(
            now >= 500_000,
            "the estimate starts at the reading, not at zero: {now}"
        );
    }

    #[tokio::test]
    async fn test_a_firmware_without_get_uptime_is_a_connect_error() {
        // No reading, no anchor: carrying on would leave the `McuClock` mapping
        // print time 0 onto firmware clock 0 — the past — and the first move
        // would come back "Timer too close". Upstream ends the connect here too
        // (`klippy/clocksync.py:33-36`).
        let mcu = Mcu::for_test("mcu2", Interface::new(FrameMock::new(vec![])));
        mcu.install_dictionary(Dictionary::from_json(polling_dictionary()).unwrap())
            .unwrap();

        let err = seed_clock_base(&mcu)
            .await
            .expect_err("a firmware with no `get_uptime` cannot be seeded");

        let text = err.to_string();
        assert!(text.contains("MCU 'mcu2'"), "names the MCU: {text}");
        assert!(text.contains("get_uptime"), "names the message: {text}");
        assert!(
            matches!(err, KlippyError::Connection(_)),
            "the bring-up reports a connection failure: {err:?}"
        );
        assert!(mcu.estimated_clock().is_none(), "no estimate was invented");
    }

    #[tokio::test]
    async fn test_a_get_uptime_that_never_answers_is_a_connect_error() {
        // The other shape of "no seed": the firmware publishes `get_uptime` but
        // does not answer it. The bound is the seed's own timeout
        // (`CLOCK_BASE_TIMEOUT`), so the bring-up ends by itself.
        let mcu = Mcu::for_test(
            "mcu2",
            Interface::new(FrameMock::new(vec![MappingEntry {
                input: Frame::new(0, vec![4]), // `get_uptime`
                outputs: vec![],
            }])),
        );
        mcu.install_dictionary(Dictionary::from_json(uptime_dictionary()).unwrap())
            .unwrap();

        let err = tokio::time::timeout(Duration::from_secs(10), seed_clock_base(&mcu))
            .await
            .expect("the seed is bounded by its own timeout")
            .expect_err("an unanswered `get_uptime` seeds nothing");

        let text = err.to_string();
        assert!(text.contains("MCU 'mcu2'"), "names the MCU: {text}");
        assert!(text.contains("get_uptime"), "names the message: {text}");
        assert!(matches!(err, KlippyError::Connection(_)), "got {err:?}");
    }

    /// The `test:` fake's own dictionary with `get_uptime` taken out, written to
    /// a temp file because the section names its dictionary by path.
    ///
    /// The fake firmware still answers the command; what the host loses is the
    /// knowledge that it exists (`Mcu::has_message`), which is the shape an old
    /// board reports — a dictionary without it.
    fn fake_dictionary_without_get_uptime() -> std::path::PathBuf {
        let source = klipperx_test_support::test_dicts_dir().join("linuxprocess.dict");
        let text = std::fs::read_to_string(&source).expect("the fake's dictionary is built");
        let mut dictionary: Value = serde_json::from_str(&text).expect("the dictionary is JSON");
        let removed = dictionary
            .get_mut("commands")
            .and_then(Value::as_object_mut)
            .and_then(|commands| commands.remove("get_uptime"));
        assert!(removed.is_some(), "the fake publishes get_uptime to remove");
        let path =
            std::env::temp_dir().join(format!("klipperx-no-uptime-{}.dict", std::process::id()));
        std::fs::write(&path, serde_json::to_string(&dictionary).unwrap())
            .expect("the temp dictionary is writable");
        path
    }

    #[tokio::test]
    async fn test_connect_refuses_a_firmware_that_cannot_seed_the_clock() {
        // The whole bring-up, not just the helper: against the `test:` fake the
        // connect gets through identify and then stops at the seed, which is
        // the order the real board fails in — instead of coming up with a clock
        // that points into the past.
        let dictionary = fake_dictionary_without_get_uptime();
        let mut section = section(None);
        section.parameters.insert(
            "test".to_string(),
            ConfigValue::Single(format!("dict={}", dictionary.display())),
        );
        let printer = printer();
        let object = Arc::new(McuObject::new(section, &printer).unwrap());
        printer
            .add_object(&object.section.identifier(), object.clone())
            .unwrap();
        // The loader assigns every `[mcu]` an id before its transport is opened.
        object.set_id(1);

        let err = tokio::time::timeout(Duration::from_secs(30), object.connect())
            .await
            .expect("the bring-up is bounded")
            .expect_err("a firmware with no clock base is not connected");

        let text = err.to_string();
        assert!(text.contains("MCU 'mcu'"), "names the MCU: {text}");
        assert!(text.contains("get_uptime"), "names the message: {text}");
        assert!(
            object.clock().is_none(),
            "no clock was installed for the refused session"
        );
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
    // Liveness: a board that stops answering its clock queries
    // -----------------------------------------------------------------------

    /// `count` scripted `get_clock` blocks that reach the wire and are **never
    /// answered** — a board that is still connected but no longer talking.
    ///
    /// The block sequence numbers are the ones the interface stamps on
    /// consecutive sends, so a silent round cannot be mistaken for a mismatched
    /// script entry. Two spare entries are appended: consuming the last entry
    /// closes the mock's receive channel, which would end the session instead
    /// of leaving it silent.
    fn silent_clock_exchanges(count: usize) -> Vec<MappingEntry> {
        (0..count + 2)
            .map(|i| MappingEntry {
                input: Frame::new((i & 0xf) as u8, vec![5]), // `get_clock`
                outputs: vec![],
            })
            .collect()
    }

    /// `count` scripted `get_clock` exchanges that are all answered, in the
    /// order the interface sends them.
    fn answered_clock_exchanges(count: usize, clock: u32) -> Vec<MappingEntry> {
        (0..count + 2)
            .map(|i| {
                let seq = (i & 0xf) as u8;
                let mut response = Payload::new();
                response.push_u8(18).unwrap(); // response id, `clock clock=%u`
                response.push_u32(clock).unwrap();
                MappingEntry {
                    input: Frame::new(seq, vec![5]),
                    outputs: vec![Frame::new(seq, response.into_raw())],
                }
            })
            .collect()
    }

    /// Fire `rounds` clock-poll intervals. The rig's section names no `test:`,
    /// so its interval is the production one second.
    fn fire_clock_polls(rig: &PollRig, rounds: usize) {
        for _ in 0..rounds {
            assert_eq!(rig.manual.advance(1.0), 1, "one poll per interval");
        }
    }

    /// Wait (bounded) for the outstanding clock query to be answered, so the
    /// next round is judged on a clean count.
    async fn wait_for_pending_clear(clock: &McuClock) {
        for _ in 0..200 {
            if clock.estimator().queries_pending() == 0 {
                return;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        panic!(
            "the clock query was never answered ({})",
            clock.estimator().queries_pending()
        );
    }

    /// A board whose clock queries stop being answered is reported lost with
    /// upstream's wording (`MCUConnHelper.check_timeout`, `klippy/mcu.py:895-905`).
    #[tokio::test]
    async fn test_an_mcu_that_stops_answering_the_clock_is_reported_lost() {
        tokio::time::timeout(Duration::from_secs(5), async {
            let rig = PollRig::new(polling_dictionary(), silent_clock_exchanges(8));
            let clock = rig.object.clock().unwrap();
            rig.object.register_clock_poll(rig.manual.as_ref());

            // The poll counts its own query as outstanding; only an answer
            // clears the count (`ClockEstimator::update`).
            fire_clock_polls(&rig, 1);
            assert_eq!(clock.estimator().queries_pending(), 1);

            // Up to four outstanding queries are still within upstream's
            // `is_active` bound.
            fire_clock_polls(&rig, 4);
            assert_eq!(clock.estimator().queries_pending(), 5);
            assert_ne!(
                rig.printer.get_state_message().category,
                PrinterState::Shutdown,
                "four unanswered queries are still tolerated"
            );

            // The fifth unanswered round makes it a dead link, and the state
            // message is upstream's word for word.
            fire_clock_polls(&rig, 1);
            let state = rig.printer.get_state_message();
            assert_eq!(state.category, PrinterState::Shutdown);
            assert_eq!(state.message, "Lost communication with MCU 'mcu'");
        })
        .await
        .expect("the liveness check is bounded");
    }

    /// The normal path is untouched: answered queries never add up to a lost
    /// link, however many intervals pass.
    #[tokio::test]
    async fn test_a_clock_that_keeps_answering_is_never_reported_lost() {
        tokio::time::timeout(Duration::from_secs(20), async {
            let rig = PollRig::new(
                polling_dictionary(),
                answered_clock_exchanges(12, 21_000_500),
            );
            let clock = rig.object.clock().unwrap();
            rig.object.register_clock_poll(rig.manual.as_ref());

            for _ in 0..12 {
                fire_clock_polls(&rig, 1);
                wait_for_pending_clear(&clock).await;
            }

            assert!(clock.estimator().is_active());
            assert_ne!(
                rig.printer.get_state_message().category,
                PrinterState::Shutdown,
                "an answering board is never declared lost"
            );
        })
        .await
        .expect("the answered path is bounded");
    }

    /// A file-output run never reports the MCU lost: the protocol is written to
    /// a file instead of a port, so nothing answers by design and the count of
    /// outstanding queries says nothing about a board (`MCU.is_fileoutput`,
    /// the mode every corpus case runs in).
    #[tokio::test]
    async fn test_a_fileoutput_run_never_reports_the_mcu_lost() {
        tokio::time::timeout(Duration::from_secs(5), async {
            let rig = PollRig::new(polling_dictionary(), silent_clock_exchanges(8));
            rig.as_fileoutput();
            let clock = rig.object.clock().unwrap();
            rig.object.register_clock_poll(rig.manual.as_ref());

            // Far past the five unanswered rounds the real mode stops at.
            fire_clock_polls(&rig, 8);

            assert_eq!(clock.estimator().queries_pending(), 8);
            assert_ne!(
                rig.printer.get_state_message().category,
                PrinterState::Shutdown
            );
        })
        .await
        .expect("the file-output path is bounded");
    }

    /// A lost MCU leaves the machine haltable and restartable: `M112` (which
    /// does exactly the `invoke_shutdown` below) still lands, the first
    /// message standing, and `RESTART`'s pair still brings the printer back to
    /// `startup` so the config can be loaded again.
    #[tokio::test]
    async fn test_a_lost_mcu_leaves_m112_and_restart_usable() {
        tokio::time::timeout(Duration::from_secs(5), async {
            let rig = PollRig::new(polling_dictionary(), silent_clock_exchanges(8));
            rig.object.register_clock_poll(rig.manual.as_ref());
            fire_clock_polls(&rig, 6);
            assert_eq!(
                rig.printer.get_state_message().category,
                PrinterState::Shutdown
            );

            // `M112`'s handler body (`gcode.rs`, `register_command("M112", ..)`).
            rig.printer.invoke_shutdown("Shutdown due to M112 command");
            assert_eq!(
                rig.printer.get_state_message().message,
                "Lost communication with MCU 'mcu'",
                "the first message stands"
            );

            // `RESTART`: the printer leaves its run loop and the next bring-up
            // starts from `startup`.
            rig.printer.request_exit("restart");
            rig.printer.reset_for_restart("restart");
            assert_eq!(
                rig.printer.get_state_message().category,
                PrinterState::Startup
            );
        })
        .await
        .expect("the halt and restart path is bounded");
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
        object.install_clock(&previous, &reactor, 0.0, 1_000_000);
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
                                      // Poll for the new session's sample instead of betting the round trip
                                      // and the fold both land inside a fixed 100 ms window.
        wait_until(
            Duration::from_secs(1),
            Duration::from_millis(5),
            || object.clock().unwrap().estimator().last_clock() != before,
            "a sample from the new session folded into the estimate",
        )
        .await;
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
