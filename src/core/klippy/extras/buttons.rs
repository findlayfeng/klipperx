//! `[buttons]` — the button/switch registry and its firmware query (upstream
//! `klippy/extras/buttons.py`).
//!
//! The module object a consumer loads by name (`printer.load_object(config,
//! 'buttons')`) rather than from a section of its own. Two entry points, one
//! per consumer shape: [`PrinterButtons::register_debounce_button`] (a single
//! debounced switch, the filament *switch* sensor and `gcode_button`) and
//! [`PrinterButtons::register_buttons`] (one or more pins sharing a callback,
//! the filament *motion* sensor's encoder).
//!
//! # The firmware side
//!
//! A registration does not poll anything itself. Upstream groups the
//! registrations of one MCU into an `MCU_buttons` (`buttons.py:16-83`), which
//! configures one firmware button object per eight pins and arms its polling
//! timer; the firmware then reports the *debounced* level of every pin whenever
//! one changes. That is what [`McuButtons`] here is: the same object, built on
//! the command layer ([`crate::core::klippy::cmd::buttons`]).
//!
//! | step | message | when |
//! |---|---|---|
//! | allocate the object | `config_buttons oid=%c button_count=%c` | config |
//! | fill a pin slot | `buttons_add oid=%c pos=%c pin=%u pull_up=%c` | init |
//! | arm the poll timer | `buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c` | after the firmware accepts the configuration |
//! | acknowledge a report | `buttons_ack oid=%c count=%c` | per report |
//! | report | `buttons_state oid=%c ack_count=%c state=%*s` | firmware → host |
//!
//! The query carries an **absolute** clock, so it is sent from a post-init
//! callback with the clock read on the connection that will report — a
//! waketime from before a firmware reset is off the new clock (`mcu/config.rs`,
//! the same reason the ADC query is armed there).
//!
//! # Reports: counters, inversion, debounce
//!
//! `buttons_state` is an event ([`ButtonsState`]), not an answer, so the handler
//! is bound before the query is sent. One MCU has one callback per message
//! name, so [`ButtonsRouter`] keeps the objects by oid and routes each report —
//! the same shape as the trsync and ADC registries.
//!
//! The firmware's `ack_count` is 8-bit and wraps; upstream expands it against
//! its own counter to tell a **retransmission** (the firmware repeats an
//! unacknowledged change every `retransmit_count` polls, `src/buttons.c:57-64`)
//! from new data, and acks the number of bytes it consumed
//! (`buttons.py:58-83`). Only the new bytes are turned into callbacks.
//!
//! Each pin's `!` (`invert`) is applied to the whole report byte before the
//! levels are read, as upstream does (`button ^= self.invert`), and the same
//! bits seed the firmware's debounce (`buttons_query`'s `invert`), so a pin
//! wired active-low idles at zero on both sides.
//!
//! Two debounces exist and both are here: the firmware's, which needs a level
//! to hold for one poll (`QUERY_TIME` = 2 ms, `src/buttons.c:28-70`), and the
//! host's [`DebounceButton`] (`debounce_delay`, default `0.`), which holds a
//! change for the configured time before it reaches the consumer.
//!
//! # What is not here
//!
//! The ADC button path (`register_adc_button`, `analog_range` sections): it is
//! a different object in upstream (`MCU_ADC_buttons`) on top of `query_adc`, not
//! a pin query. `[gcode_button]` refuses `analog_range` for that reason.
//!
//! A registration whose pin cannot be resolved (no such chip, a pin already in
//! use, or pins from two MCUs in one registration) is not sent to any firmware:
//! it is logged as an error and dropped. Upstream raises the same messages as
//! config errors; this entry point is infallible because its callers' signatures
//! keep it so (`filament_motion_sensor`'s encoder).

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::buttons::{
    ButtonsAck, ButtonsAdd, ButtonsQuery, ButtonsState, ConfigButtons,
};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::mcu::{pin_number, query_slot, ConfigBuilder, Mcu, McuChip, McuError};
use crate::core::klippy::pins::{PinParams, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::Reactor;
use tracing::{error, warn};

/// The name the sensors look the registry up by (`load_object(config,
/// 'buttons')`).
pub const BUTTONS_OBJECT: &str = "buttons";

/// A registered button state callback: `(eventtime, state)`, the signature
/// upstream's `_button_handler` / `encoder_event` take.
pub type ButtonCallback = Box<dyn Fn(f64, bool) + Send + Sync>;

/// How often the firmware polls the pins: upstream's `QUERY_TIME`
/// (`buttons.py:13`, `0.002` s).
const QUERY_TIME: f64 = 0.002;

/// Upstream's `RETRANSMIT_COUNT` (`buttons.py:14`): how many polls an
/// unacknowledged change is repeated for. The firmware refuses the value once it
/// reaches its "no retransmit" bit (`src/buttons.c:109-110`).
const RETRANSMIT_COUNT: u8 = 50;

/// The firmware's per-object pin limit (`src/buttons.c:76-78`): one report byte
/// is one bit per pin.
const MAX_BUTTONS: usize = 8;

/// The state one callback is interested in: which bits of a report belong to
/// it, and where its own pins start in the byte.
#[derive(Clone)]
struct GroupCallback {
    /// The bits this registration watches.
    mask: u8,
    /// The bit the registration's first pin sits at, so a multi-pin
    /// registration reads its own pins as one number (an encoder's two pins).
    shift: u8,
    callback: Arc<dyn Fn(f64, bool) + Send + Sync>,
}

/// One firmware button object — upstream's `MCU_buttons` (`buttons.py:16-83`).
///
/// Upstream makes a new one when a chip's eight pin slots are full
/// (`buttons.py:328-332`), so a chip may own several; each has its own oid and
/// its own report state.
struct McuButtons {
    chip: McuChip,
    /// The machine's clock: a report's eventtime is upstream's `#receive_time`,
    /// the reactor time the frame arrived at.
    printer: Weak<Printer>,
    /// The router that hands this object its reports, shared by every object on
    /// the chip.
    router: Arc<ButtonsRouter>,
    state: Mutex<GroupState>,
}

/// The buildable and reportable half of one [`McuButtons`].
#[derive(Default)]
struct GroupState {
    /// The pins in slot order: the name still to resolve, and the pull-up as
    /// `gpio_in_setup`'s `int8_t` (`1` up, `-1` down, `0` none).
    pins: Vec<(String, i8)>,
    callbacks: Vec<GroupCallback>,
    /// The idle levels, one bit per pin (`!` pins set), as `buttons_query`'s
    /// `invert` and the inversion of every report byte.
    invert: u8,
    /// The last reported level byte, after inversion.
    last_button: u8,
    /// Upstream's `ack_count`: the host's unbounded count of the bytes the
    /// firmware has reported, used to expand its 8-bit echo.
    ack_count: i64,
    /// The object id, assigned when the configuration is built.
    oid: Option<u8>,
}

impl McuButtons {
    /// Build the object for `chip` and register its config callback.
    ///
    /// The callback captures this object (it has to: it assigns the oid and
    /// arms the query), so it is the one strong handle the chip's builder holds
    /// — and `ConfigBuilder::build` takes the callbacks out as it runs them, so
    /// the cycle it would otherwise make ends there.
    ///
    /// # Errors
    /// Returns [`McuError`] if the callback cannot be registered (the
    /// configuration is already built).
    fn new(
        chip: McuChip,
        printer: Weak<Printer>,
        router: Arc<ButtonsRouter>,
    ) -> Result<Arc<Self>, McuError> {
        let group = Arc::new(Self {
            chip: chip.clone(),
            printer,
            router,
            state: Mutex::new(GroupState::default()),
        });
        let builder = chip.config();
        let callback_group = Arc::clone(&group);
        let callback_pins = Arc::downgrade(&chip.pins());
        builder.register_config_callback(Box::new(move |builder, mcu| {
            let pins = callback_pins
                .upgrade()
                .ok_or_else(|| McuError::Config("the pins registry is gone".to_string()))?;
            callback_group.build(builder, mcu, &pins)
        }))?;
        Ok(group)
    }

    fn lock(&self) -> MutexGuard<'_, GroupState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Add one registration's pins to this object — upstream's `setup_buttons`
    /// (`buttons.py:26-34`).
    fn setup_buttons(&self, pins: &[PinParams], callback: ButtonCallback) {
        let mut state = self.lock();
        let shift = state.pins.len() as u8;
        let mut mask = 0u8;
        for params in pins {
            let bit = state.pins.len() as u8;
            if params.invert {
                state.invert |= 1 << bit;
            }
            mask |= 1 << bit;
            state.pins.push((params.pin.clone(), params.pullup));
        }
        state.callbacks.push(GroupCallback {
            mask,
            shift,
            callback: Arc::from(callback),
        });
    }

    /// Whether `pins` fit in this object's remaining slots.
    fn has_room(&self, pins: usize) -> bool {
        self.lock().pins.len() + pins <= MAX_BUTTONS
    }

    /// The oid, once the configuration is built.
    fn oid(&self) -> Option<u8> {
        self.lock().oid
    }

    /// The build-time half: assign the oid, add the config and init commands,
    /// and arrange for the query to be armed on the connection that will
    /// report. Upstream's `MCU_buttons.build_config` (`buttons.py:35-57`).
    fn build(
        self: &Arc<Self>,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
    ) -> Result<(), McuError> {
        let (pin_list, invert) = {
            let state = self.lock();
            (state.pins.clone(), state.invert)
        };
        // Querying nothing is not an object: upstream returns before creating
        // the oid (`buttons.py:36-37`).
        if pin_list.is_empty() {
            return Ok(());
        }
        let button_count = u8::try_from(pin_list.len()).map_err(|_| {
            McuError::Config(format!("{} buttons do not fit a byte", pin_list.len()))
        })?;
        let oid = builder.create_oid()?;
        self.lock().oid = Some(oid);
        self.router.register(self);
        builder.add_config_cmd(&ConfigButtons { oid, button_count })?;
        let chip_name = self.chip.name().to_string();
        for (pos, (pin, pull_up)) in pin_list.iter().enumerate() {
            let canonical = pins.resolve_pin(&chip_name, pin)?;
            let number = pin_number(mcu, &canonical, &chip_name)?;
            builder.add_init_cmd(&ButtonsAdd {
                oid,
                pos: pos as u8,
                pin: number,
                pull_up: *pull_up as u8,
            })?;
        }
        let group = Arc::clone(self);
        builder.register_post_init_callback(Box::new(move |mcu| {
            if let Err(err) = group.arm(mcu, oid, invert) {
                warn!(
                    "MCU '{}': could not arm the button query: {err}",
                    mcu.name()
                );
            }
        }))?;
        Ok(())
    }

    /// Send `buttons_query` on `mcu` and bind the report handler.
    ///
    /// The handler is bound **before** the query goes out: the firmware reports
    /// as soon as its timer fires, and a report that arrives with nothing bound
    /// is dropped as unhandled.
    ///
    /// The clock is the one this connection is at, plus the query slot offset
    /// that keeps the first poll clear of the `init` commands
    /// ([`query_slot`]).
    fn arm(&self, mcu: &Mcu, oid: u8, invert: u8) -> Result<(), McuError> {
        self.router.bind(mcu)?;
        let clock = query_slot(mcu, oid)?;
        let rest_ticks = mcu.seconds_to_clock(QUERY_TIME)? as u32;
        mcu.send_msg(&ButtonsQuery {
            oid,
            clock,
            rest_ticks,
            retransmit_count: RETRANSMIT_COUNT,
            invert,
        })
    }

    /// Fold one report in: acknowledge what it delivered, then hand the new
    /// levels to the callbacks.
    ///
    /// Upstream's `MCU_buttons.handle_buttons_state` (`buttons.py:58-83`). The
    /// ack goes out first, before any consumer code runs, so a slow consumer
    /// cannot make the firmware retransmit what the host already has.
    ///
    /// The consumers themselves run from a reactor callback (`call_later(0.)`),
    /// which is where upstream's `register_async_callback` puts them
    /// (`buttons.py:81-82`): they are a runout check and user gcode templates,
    /// and this runs on the receive task — a consumer that asks the MCU
    /// anything could not be answered from there. The eventtime is captured
    /// first, at the receipt upstream dates the callback with (`#receive_time`,
    /// `buttons.py:74`).
    fn handle_report(&self, report: &ButtonsState) {
        let Some((count, calls)) = self.take_report(report) else {
            return;
        };
        match self.chip.mcu() {
            Some(mcu) => {
                if let Err(err) = mcu.send_msg(&ButtonsAck {
                    oid: report.oid,
                    count,
                }) {
                    warn!("could not acknowledge {} button reports: {err}", count);
                }
            }
            None => warn!(
                "could not acknowledge {} button reports: the MCU is gone",
                count
            ),
        }
        if calls.is_empty() {
            return;
        }
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let eventtime = printer.eventtime();
        printer.reactor().call_later(
            0.,
            Box::new(move |_| {
                for (callback, state) in calls {
                    callback(eventtime, state);
                }
            }),
        );
    }

    /// The state half of [`McuButtons::handle_report`]: how many bytes have to
    /// be acknowledged, and what the registrations' levels became.
    ///
    /// `None` when the report carried nothing new — a retransmission, as
    /// upstream's `if new_count <= 0: return` (`buttons.py:67-68`).
    #[allow(clippy::type_complexity)]
    fn take_report(
        &self,
        report: &ButtonsState,
    ) -> Option<(u8, Vec<(Arc<dyn Fn(f64, bool) + Send + Sync>, bool)>)> {
        let mut state = self.lock();
        // The firmware's counter is 8-bit; upstream reads the difference as a
        // signed byte (`buttons.py:60-63`), so a wrap between two reports is
        // the small step it looks like rather than a 256-byte jump.
        let ack_diff = (i64::from(report.ack_count) - state.ack_count) & 0xff;
        let ack_diff = if ack_diff & 0x80 != 0 {
            ack_diff - 0x100
        } else {
            ack_diff
        };
        let msg_ack_count = state.ack_count + ack_diff;
        let new_count = msg_ack_count + report.state.len() as i64 - state.ack_count;
        if new_count <= 0 {
            return None;
        }
        let new_count = new_count as usize;
        // The report may repeat bytes the host has already seen at its front.
        // Python's `buttons[-new_count:]` clamps a slice longer than the array
        // to the whole array, which a report after a counter wrap can ask for
        // (`buttons.py:69`).
        let start = report.state.len().saturating_sub(new_count);
        let new_bytes = report.state.get(start..)?;
        state.ack_count += new_count as i64;
        let count = u8::try_from(new_count).ok()?;

        let callbacks = state.callbacks.clone();
        let mut calls = Vec::new();
        for &button in new_bytes {
            let button = button ^ state.invert;
            let changed = button ^ state.last_button;
            state.last_button = button;
            for entry in &callbacks {
                if changed & entry.mask != 0 {
                    let level = (button & entry.mask) >> entry.shift;
                    calls.push((Arc::clone(&entry.callback), level != 0));
                }
            }
        }
        Some((count, calls))
    }
}

/// Routes `buttons_state` to the object its `oid` belongs to.
///
/// One per MCU: `Mcu::bind_event` keeps a single handler per message name, so
/// every button object on a chip shares this table. Weak entries, like
/// [`TrsyncRegistry`](crate::core::klippy::mcu::TrsyncRegistry): each object
/// holds the router (to bind the handler), so a strong entry would be a cycle —
/// and through the object's chip it would keep the connected `Mcu` alive past
/// teardown.
#[derive(Default)]
struct ButtonsRouter {
    by_oid: Mutex<HashMap<u8, Weak<McuButtons>>>,
}

impl ButtonsRouter {
    /// Bind this router as the chip's `buttons_state` handler, replacing one
    /// bound earlier.
    ///
    /// Rebinding is harmless — every object on the chip is in the same table by
    /// then — and it is what makes a reconnected `Mcu` (a firmware `reset`, a
    /// fresh parser with no callbacks) work again.
    ///
    /// # Errors
    /// Returns [`McuError`] if the session's dictionary has no `buttons_state`.
    fn bind(self: &Arc<Self>, mcu: &Mcu) -> Result<(), McuError> {
        let router = Arc::downgrade(self);
        mcu.bind_event::<ButtonsState, _>(move |report| {
            if let Some(router) = router.upgrade() {
                router.route(&report);
            }
        })
    }

    /// Hand one report to the object its oid names; an oid nobody owns is
    /// dropped, as upstream's per-oid handler drops a late report.
    fn route(&self, report: &ButtonsState) {
        let group = self
            .by_oid
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .get(&report.oid)
            .and_then(Weak::upgrade);
        if let Some(group) = group {
            group.handle_report(report);
        }
    }

    fn register(&self, group: &Arc<McuButtons>) {
        let Some(oid) = group.oid() else {
            return;
        };
        self.by_oid
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert(oid, Arc::downgrade(group));
    }
}

/// The host-side debounce between the firmware's levels and a consumer —
/// upstream's `DebounceButton` (`buttons.py:252-276`).
///
/// The firmware already requires a level to hold for one poll; this holds the
/// *transition* for `debounce_delay` on top of that. Two properties are
/// upstream's, and both matter for a filament switch: a bounce back before the
/// delay expires cancels the pending change (the states match again), and a
/// newer event supersedes a pending one rather than adding to it.
struct DebounceButton {
    reactor: Arc<dyn Reactor>,
    action: Arc<dyn Fn(f64, bool) + Send + Sync>,
    delay: f64,
    logical_state: Mutex<Option<bool>>,
    physical_state: Mutex<Option<bool>>,
    latest_eventtime: Mutex<Option<f64>>,
}

impl DebounceButton {
    fn new(reactor: Arc<dyn Reactor>, delay: f64, action: ButtonCallback) -> Arc<Self> {
        Arc::new(Self {
            reactor,
            action: Arc::from(action),
            delay,
            logical_state: Mutex::new(None),
            physical_state: Mutex::new(None),
            latest_eventtime: Mutex::new(None),
        })
    }

    /// The firmware callback: record the level and schedule the debounce timer.
    fn button_handler(self: &Arc<Self>, eventtime: f64, state: bool) {
        *self.lock(&self.physical_state) = Some(state);
        *self.lock(&self.latest_eventtime) = Some(eventtime);
        // No transition: nothing to debounce.
        if *self.lock(&self.logical_state) == Some(state) {
            return;
        }
        let trigger_time = eventtime + self.delay;
        let debounce = Arc::clone(self);
        self.reactor.register_timer_named(
            "buttons_debounce",
            Box::new(move |eventtime| {
                debounce.debounce_event(eventtime);
                None
            }),
            trigger_time,
        );
    }

    /// The timer callback: enact the transition if it is still the latest one.
    fn debounce_event(&self, eventtime: f64) {
        let physical = *self.lock(&self.physical_state);
        if *self.lock(&self.logical_state) == physical {
            return;
        }
        // A more recent event supersedes this one: its own timer will decide.
        if let Some(latest) = *self.lock(&self.latest_eventtime) {
            if (eventtime - self.delay) < latest {
                return;
            }
        }
        *self.lock(&self.logical_state) = physical;
        if let Some(physical) = physical {
            let eventtime = self.lock(&self.latest_eventtime).unwrap_or(eventtime);
            (self.action)(eventtime, physical);
        }
    }

    fn lock<'a, T>(&'a self, field: &'a Mutex<T>) -> MutexGuard<'a, T> {
        field.lock().unwrap_or_else(|poison| poison.into_inner())
    }
}

/// The `[buttons]` module object (upstream's `PrinterButtons`).
pub struct PrinterButtons {
    printer: Weak<Printer>,
    state: Mutex<ButtonsRegistry>,
}

/// What the registrations have built so far.
#[derive(Default)]
struct ButtonsRegistry {
    /// One `MCU_buttons` per firmware object, in creation order.
    groups: Vec<Arc<McuButtons>>,
    /// The report router of each chip, by chip name.
    routers: HashMap<String, Arc<ButtonsRouter>>,
    /// The pins of each registration, in registration order (tests and
    /// diagnostics).
    registrations: Vec<Vec<String>>,
}

impl PrinterButtons {
    /// The single `buttons` object; the first caller creates it, as upstream's
    /// `printer.load_object(config, 'buttons')` does.
    ///
    /// # Errors
    /// A duplicate registration (a name already taken).
    pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrinterButtons>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<PrinterButtons>(BUTTONS_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(PrinterButtons {
            printer: Arc::downgrade(printer),
            state: Mutex::new(ButtonsRegistry::default()),
        });
        printer.add_object(
            BUTTONS_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
    }

    /// Upstream's `register_debounce_button` (`buttons.py:296-298`): read the
    /// section's `debounce_delay` and register a single pin through a
    /// [`DebounceButton`].
    ///
    /// The debounce delay is read from `config` (the caller's section), as
    /// upstream's `DebounceButton` reads it (`buttons.py:257`).
    ///
    /// # Errors
    /// A negative or unparsable `debounce_delay`.
    pub fn register_debounce_button(
        &self,
        config: &ConfigWrapper,
        pin: &str,
        callback: ButtonCallback,
    ) -> Result<(), ConfigError> {
        let delay =
            config.get_float_bounded("debounce_delay", Some(0.0), Some(0.0), None, None, None)?;
        let reactor = self
            .printer
            .upgrade()
            .map(|printer| printer.reactor())
            .ok_or_else(|| ConfigError::new("the buttons registry outlived the printer"))?;
        let debounce = DebounceButton::new(reactor, delay, callback);
        let handler = Arc::clone(&debounce);
        self.register_buttons(
            vec![pin.to_string()],
            Box::new(move |eventtime, state| handler.button_handler(eventtime, state)),
        );
        Ok(())
    }

    /// Upstream's `register_buttons` (`buttons.py:309-336`): one or more pins
    /// that share a callback, on one MCU.
    ///
    /// The pins are resolved here, at load time, exactly as upstream's
    /// `ppins.lookup_pin(pin, can_invert=True, can_pullup=True)` does, so a
    /// `!`/`^` on the pin reaches the firmware (`buttons_query`'s `invert`,
    /// `buttons_add`'s `pull_up`) and a registration that cannot be resolved is
    /// reported rather than silently dropped.
    ///
    /// Infallible because its callers' signatures are: upstream raises these as
    /// config errors, here they are logged and the registration is not made.
    pub fn register_buttons(&self, pins: Vec<String>, callback: ButtonCallback) {
        match self.setup_buttons(&pins, callback) {
            Ok(()) => self
                .lock()
                .registrations
                .push(pins.iter().map(|pin| pin.to_string()).collect()),
            Err(err) => error!("buttons: {err}"),
        }
    }

    /// Resolve `pins`, find the firmware object that has room for them and
    /// register them — upstream's `register_buttons` body.
    fn setup_buttons(&self, pins: &[String], callback: ButtonCallback) -> Result<(), String> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| "the buttons registry outlived the printer".to_string())?;
        let ppins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| "the machine has no pins object".to_string())?;
        let mut params: Vec<PinParams> = Vec::with_capacity(pins.len());
        for pin in pins {
            let resolved = ppins
                .lookup_pin(pin, true, true, None)
                .map_err(|err| format!("{err}"))?;
            if let Some(first) = params.first() {
                if first.chip_name != resolved.chip_name {
                    return Err(format!("button pins must be on same mcu: {pin}"));
                }
            }
            params.push(resolved);
        }
        if params.is_empty() {
            return Ok(());
        }
        let chip_name = params[0].chip_name.clone();
        let chip = ppins
            .chip_as::<McuChip>(&chip_name)
            .ok_or_else(|| format!("chip '{chip_name}' is not an MCU"))?;
        let group = {
            let mut state = self.lock();
            let router = match state.routers.get(&chip_name) {
                Some(router) => Arc::clone(router),
                None => {
                    let router = Arc::new(ButtonsRouter::default());
                    state.routers.insert(chip_name.clone(), Arc::clone(&router));
                    router
                }
            };
            let existing = state
                .groups
                .iter()
                .rev()
                .find(|group| group.chip.name() == chip_name && group.has_room(params.len()))
                .cloned();
            match existing {
                Some(group) => group,
                None => {
                    let printer = self.printer.clone();
                    let group = McuButtons::new((*chip).clone(), printer, router)
                        .map_err(|err| format!("could not configure the button query: {err}"))?;
                    state.groups.push(Arc::clone(&group));
                    group
                }
            }
        };
        group.setup_buttons(&params, callback);
        Ok(())
    }

    /// How many registrations have been made (tests and diagnostics).
    pub fn registration_count(&self) -> usize {
        self.lock().registrations.len()
    }

    /// The registered pin groups, in registration order (tests and
    /// diagnostics); a debounced single button is one group of one pin.
    pub fn registered_pins(&self) -> Vec<Vec<String>> {
        self.lock().registrations.clone()
    }

    fn lock(&self) -> MutexGuard<'_, ButtonsRegistry> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl std::fmt::Debug for PrinterButtons {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.lock();
        f.debug_struct("PrinterButtons")
            .field("registrations", &state.registrations.len())
            .field("groups", &state.groups.len())
            .finish()
    }
}

impl PrinterObject for PrinterButtons {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::interface::devices::frame_mock::RecordingWire;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{BuiltConfig, ConfigBuilder, Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::{ArgValue, Payload};
    use crate::core::klippy::reactor::ManualReactor;
    use crate::core::klippy::test_util::wait_until;
    use serde_json::json;
    use std::time::Duration;

    // -----------------------------------------------------------------------
    // Fixtures
    // -----------------------------------------------------------------------

    /// The dictionary subset one button query needs: the configuration
    /// handshake's messages, the four button commands, and nine pins.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "get_clock": 5,
                "config_buttons oid=%c button_count=%c": 50,
                "buttons_add oid=%c pos=%c pin=%u pull_up=%c": 51,
                "buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c": 52,
                "buttons_ack oid=%c count=%c": 53
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9,
                "clock clock=%u": 18,
                "buttons_state oid=%c ack_count=%c state=%*s": 54
            },
            "enumerations": {
                "pin": {
                    "PA0": 0, "PA1": 1, "PA2": 2, "PA3": 3, "PA4": 4,
                    "PA5": 5, "PA6": 6, "PA7": 7, "PA8": 8
                }
            },
            "config": {"CLOCK_FREQ": 1_000_000}
        }))
        .unwrap()
    }

    /// The formats the tests encode against, by the id the fixture gives them.
    const CONFIG_BUTTONS: i16 = 50;
    const BUTTONS_ADD: i16 = 51;
    const BUTTONS_QUERY: i16 = 52;
    const BUTTONS_ACK: i16 = 53;
    const BUTTONS_STATE: i16 = 54;

    fn format_of(id: i16) -> &'static str {
        match id {
            CONFIG_BUTTONS => "config_buttons oid=%c button_count=%c",
            BUTTONS_ADD => "buttons_add oid=%c pos=%c pin=%u pull_up=%c",
            BUTTONS_QUERY => {
                "buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c"
            }
            BUTTONS_ACK => "buttons_ack oid=%c count=%c",
            BUTTONS_STATE => "buttons_state oid=%c ack_count=%c state=%*s",
            other => panic!("no format for message {other}"),
        }
    }

    /// One message's wire bytes: the firmware's id followed by its parameters.
    fn wire_bytes(id: i16, args: &[ArgValue]) -> Vec<u8> {
        let format = format_of(id);
        let mut parser = Parser::new();
        parser.register(id, format).unwrap();
        let name = format.split_whitespace().next().unwrap();
        parser.encode(name, args).unwrap().into_raw()
    }

    /// A machine with a `pins` object and one MCU chip, as the loader leaves it
    /// before anything connects.
    struct World {
        reactor: Arc<ManualReactor>,
        printer: Arc<Printer>,
        chip: McuChip,
        buttons: Arc<PrinterButtons>,
    }

    fn world() -> World {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let pins = Arc::new(PrinterPins::new());
        printer
            .add_object(PINS_OBJECT, Arc::clone(&pins) as Arc<dyn PrinterObject>)
            .expect("the pins registry is registered once");
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        pins.register_chip("mcu", Arc::new(chip.clone()))
            .expect("one chip per name");
        let buttons = PrinterButtons::ensure(&printer).expect("the object registers");
        World {
            reactor,
            printer,
            chip,
            buttons,
        }
    }

    /// A test MCU carrying the fixture dictionary, at 1 MHz.
    fn mcu(wire: RecordingWire) -> Arc<Mcu> {
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::recording(wire)));
        mcu.install_dictionary(dictionary()).unwrap();
        // The query slot needs a clock to place the first poll in the future.
        mcu.set_clock_base(1_000_000);
        mcu
    }

    /// Closes the fake session when a test ends — a session left open parks the
    /// wire's blocking read and hangs the test binary (see the endstop tests'
    /// `SessionGuard` for the full story).
    struct SessionGuard(Arc<Mcu>);

    impl Drop for SessionGuard {
        fn drop(&mut self) {
            self.0.clear_events();
            self.0.close();
        }
    }

    /// The decoded parameters of one encoded message.
    fn decode_payload(id: i16, payload: &[u8]) -> Vec<ArgValue> {
        let mut parser = Parser::new();
        parser.register(id, format_of(id)).unwrap();
        let frame = crate::core::klippy::frame::Frame::new(0, payload.to_vec());
        parser.decode(frame.into()).unwrap()[0].1.clone()
    }

    /// The decoded `(name, args)` of one built list.
    fn decode_list(mcu: &Mcu, payloads: &[Payload]) -> Vec<(String, Vec<ArgValue>)> {
        let mut parser = Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        payloads
            .iter()
            .map(|payload| {
                let frame = crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).unwrap();
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    /// The object the registration created, for the report-level tests.
    fn the_group(buttons: &Arc<PrinterButtons>) -> Arc<McuButtons> {
        let state = buttons.lock();
        assert_eq!(state.groups.len(), 1, "expected exactly one button object");
        Arc::clone(&state.groups[0])
    }

    /// Build the chip and run the post-init callbacks, as the handshake does —
    /// the production path that arms the query.
    fn configure(world: &World, mcu: &Arc<Mcu>) {
        world.chip.attach(Arc::clone(mcu));
        let mut built = world.chip.config().build(mcu).expect("the config builds");
        for callback in built.take_post_init() {
            callback(mcu);
        }
    }

    // -----------------------------------------------------------------------
    // The registry
    // -----------------------------------------------------------------------

    /// The first `ensure` creates the object, the second returns the same one —
    /// upstream's `load_object` caches by name.
    #[test]
    fn test_ensure_returns_one_shared_object() {
        let world = world();
        let second = PrinterButtons::ensure(&world.printer).expect("the object is reused");
        assert!(Arc::ptr_eq(&world.buttons, &second));
    }

    /// `register_debounce_button` reads the section's `debounce_delay` (so the
    /// option is accepted) and records the registration.
    #[test]
    fn test_register_debounce_button_reads_the_delay_and_records() {
        let world = world();
        let (config, _) =
            Config::from_text("[buttons_test]\nswitch_pin: PA0\ndebounce_delay: 0.01\n")
                .expect("the config parses");
        let section = config
            .get_section("buttons_test")
            .expect("the section exists");
        let wrapper = ConfigWrapper::untracked(section);

        world
            .buttons
            .register_debounce_button(&wrapper, "PA0", Box::new(|_, _| {}))
            .expect("the delay is accepted");
        assert_eq!(world.buttons.registration_count(), 1);
        assert_eq!(world.buttons.registered_pins(), vec![vec!["PA0"]]);
    }

    /// A negative debounce delay is refused with the config wording, as
    /// upstream's `getfloat(..., minval=0.)` does.
    #[test]
    fn test_a_negative_debounce_delay_is_refused() {
        let world = world();
        let (config, _) =
            Config::from_text("[buttons_test]\ndebounce_delay: -1\n").expect("the config parses");
        let section = config
            .get_section("buttons_test")
            .expect("the section exists");
        let wrapper = ConfigWrapper::untracked(section);

        let err = world
            .buttons
            .register_debounce_button(&wrapper, "PA0", Box::new(|_, _| {}))
            .unwrap_err()
            .to_string();
        assert!(err.contains("must have minimum of"), "{err}");
    }

    /// `register_buttons` records every pin of one registration, as upstream
    /// groups them.
    #[test]
    fn test_register_buttons_records_the_group() {
        let world = world();
        world.buttons.register_buttons(
            vec!["PA0".to_string(), "PA1".to_string()],
            Box::new(|_, _| {}),
        );
        assert_eq!(world.buttons.registration_count(), 1);
        assert_eq!(
            world.buttons.registered_pins(),
            vec![vec!["PA0".to_string(), "PA1".to_string()]]
        );
    }

    /// Pins from two MCUs in one registration are refused, as upstream's
    /// "button pins must be on same mcu" does — and, being unresolvable, the
    /// registration is not recorded.
    #[test]
    fn test_pins_from_two_mcus_are_refused() {
        let world = world();
        let pins = world
            .printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the pins registry");
        let other = McuChip::new(
            "toolboard".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        pins.register_chip("toolboard", Arc::new(other))
            .expect("a second chip name");

        world.buttons.register_buttons(
            vec!["PA0".to_string(), "toolboard:PA1".to_string()],
            Box::new(|_, _| {}),
        );
        assert_eq!(world.buttons.registration_count(), 0);
    }

    /// Nine pins do not fit one firmware object (its report is one byte), so
    /// the ninth starts a second oid — upstream's `buttons.py:328-332`.
    #[tokio::test]
    async fn test_a_ninth_pin_starts_a_second_firmware_object() {
        let world = world();
        for index in 0..9 {
            world.buttons.register_buttons(
                vec![format!("PA{index}")],
                Box::new(move |_, _| {
                    let _ = index;
                }),
            );
        }
        let mcu = mcu(RecordingWire::new());
        let _session = SessionGuard(Arc::clone(&mcu));
        let built: BuiltConfig = world.chip.config().build(&mcu).unwrap();
        let config = decode_list(&mcu, &built.config);

        let buttons: Vec<_> = config
            .iter()
            .filter(|(name, _)| name == "config_buttons")
            .collect();
        assert_eq!(buttons.len(), 2, "one object per eight pins");
        assert_eq!(buttons[0].1, vec![ArgValue::UInt8(0), ArgValue::UInt8(8)]);
        assert_eq!(buttons[1].1, vec![ArgValue::UInt8(1), ArgValue::UInt8(1)]);
    }

    // -----------------------------------------------------------------------
    // ① The configuration bytes
    // -----------------------------------------------------------------------

    /// ① `config_buttons` and `buttons_add` as they reach the firmware: the
    /// oid, the pin slot, the resolved pin number, and the pull-up — `^` as
    /// `1` and `~` as `-1` (`0xff`, `gpio_in_setup`'s `int8_t`).
    #[tokio::test]
    async fn test_the_config_and_init_lists_carry_the_button_bytes() {
        let world = world();
        world.buttons.register_buttons(
            vec!["PA1".to_string(), "~PA2".to_string()],
            Box::new(|_, _| {}),
        );
        let mcu = mcu(RecordingWire::new());
        let _session = SessionGuard(Arc::clone(&mcu));
        let built = world.chip.config().build(&mcu).unwrap();

        // allocate_oids, config_buttons, finalize_config.
        let config = decode_list(&mcu, &built.config);
        assert_eq!(config[1].0, "config_buttons");
        assert_eq!(config[1].1, vec![ArgValue::UInt8(0), ArgValue::UInt8(2)]);
        assert_eq!(built.config[1].payload(), &[50, 0, 2]);

        // One `buttons_add` per pin, in slot order.
        let init = decode_list(&mcu, &built.init);
        assert_eq!(init[0].0, "buttons_add");
        assert_eq!(
            init[0].1,
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(1),
                ArgValue::UInt8(0)
            ]
        );
        assert_eq!(built.init[0].payload(), &[51, 0, 0, 1, 0]);
        // `~PA2` is pin 2 with a pull-down, which is the byte 0xff.
        assert_eq!(
            init[1].1,
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt8(1),
                ArgValue::UInt32(2),
                ArgValue::UInt8(0xff)
            ]
        );
    }

    // -----------------------------------------------------------------------
    // ①②③ The armed query, the report, the acknowledgement
    // -----------------------------------------------------------------------

    /// ①②③ One report, end to end on the wire: the query is armed in the
    /// future with upstream's `rest_ticks`/`retransmit_count`/`invert`, the
    /// injected `buttons_state` reaches the callback as `(eventtime, state)`
    /// with the pin's `!` applied, and the host acknowledges the byte it
    /// consumed.
    #[tokio::test]
    async fn test_the_query_is_armed_and_a_report_calls_back_and_is_acknowledged() {
        let world = world();
        // A known eventtime: the reactor's clock is what a report is dated with
        // (upstream's `#receive_time`).
        world.reactor.advance(0.5);
        let calls: Arc<Mutex<Vec<(f64, bool)>>> = Arc::new(Mutex::new(Vec::new()));
        let record = Arc::clone(&calls);
        // `!PA1`: active low, so the report byte 0 means "pressed".
        world.buttons.register_buttons(
            vec!["!PA1".to_string()],
            Box::new(move |eventtime, state| {
                record.lock().unwrap().push((eventtime, state));
            }),
        );

        let wire = RecordingWire::new();
        let sent = wire.sent();
        let mcu = mcu(wire.clone());
        let _session = SessionGuard(Arc::clone(&mcu));
        // Assert the registration did become one firmware object.
        the_group(&world.buttons);

        let report = wire_bytes(
            BUTTONS_STATE,
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::Bytes(vec![0x00]),
            ],
        );

        // The handshake's post-init step arms the query; read it back before
        // answering, because its clock is the live connection's (1.5 s ahead of
        // the estimate, so it cannot be scripted in advance). The second arm
        // sends the same bytes and gets the report back.
        configure(&world, &mcu);
        wait_until(
            Duration::from_secs(1),
            Duration::from_millis(5),
            || !sent.lock().unwrap().is_empty(),
            "the query never left",
        )
        .await;
        let query = sent.lock().unwrap()[0].clone();
        let fields = decode_payload(BUTTONS_QUERY, &query);
        assert_eq!(fields[0], ArgValue::UInt8(0), "oid");
        let clock = match fields[1] {
            ArgValue::UInt32(clock) => clock,
            ref other => panic!("clock is {other:?}"),
        };
        // The first poll sits upstream's 1.5 s ahead of the estimate, plus the
        // oid's 10 ms slot (`mcu.py:1136`).
        assert!(
            (2_500_000..2_600_000).contains(&clock),
            "the first poll is placed 1.5 s ahead: {clock}"
        );
        // QUERY_TIME = 2 ms, upstream's retransmit count, and `!PA1`'s idle bit.
        assert_eq!(fields[2], ArgValue::UInt32(2_000));
        assert_eq!(fields[3], ArgValue::UInt8(RETRANSMIT_COUNT));
        assert_eq!(fields[4], ArgValue::UInt8(1));

        // A query whose bytes this test controls, answered by the report: the
        // production arm's clock is the live connection's, so it cannot be
        // scripted ahead. The firmware accepts a new query at any time.
        let answered = wire_bytes(
            BUTTONS_QUERY,
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt32(42),
                ArgValue::UInt32(2_000),
                ArgValue::UInt8(RETRANSMIT_COUNT),
                ArgValue::UInt8(1),
            ],
        );
        wire.reply_to(answered.clone(), report);
        // The send task coalesces whatever is queued within its 1 ms window, so
        // this send has to be a block of its own to be answered.
        tokio::time::sleep(Duration::from_millis(10)).await;
        mcu.send_msg(&ButtonsQuery {
            oid: 0,
            clock: 42,
            rest_ticks: 2_000,
            retransmit_count: RETRANSMIT_COUNT,
            invert: 1,
        })
        .expect("the query is queued");
        wait_until(
            Duration::from_secs(1),
            Duration::from_millis(5),
            || {
                sent.lock()
                    .unwrap()
                    .iter()
                    .any(|payload| *payload == answered)
            },
            "the answered query never left",
        )
        .await;

        let ack = wire_bytes(BUTTONS_ACK, &[ArgValue::UInt8(0), ArgValue::UInt8(1)]);
        // And the byte is acknowledged, which is what stops the retransmission.
        wait_until(
            Duration::from_secs(1),
            Duration::from_millis(5),
            || sent.lock().unwrap().iter().any(|payload| *payload == ack),
            "the report was never acknowledged",
        )
        .await;
        assert_eq!(the_group(&world.buttons).lock().ack_count, 1);

        // The consumers run from the reactor, not from the report path, so they
        // are still pending until it steps (upstream's
        // `register_async_callback`).
        assert!(
            calls.lock().unwrap().is_empty(),
            "the callback must not run on the receive task"
        );
        world.reactor.run_due();
        assert_eq!(
            *calls.lock().unwrap(),
            vec![(0.5, true)],
            "!(0) is pressed, dated at the reactor time the report arrived"
        );
    }

    /// ③ The same report again is a retransmission: the firmware's `ack_count`
    /// echo says so, nothing new is acknowledged and no callback runs.
    #[tokio::test]
    async fn test_a_retransmitted_report_is_not_acked_twice() {
        let world = world();
        world
            .buttons
            .register_buttons(vec!["PA1".to_string()], Box::new(|_, _| {}));
        let mcu = mcu(RecordingWire::new());
        let _session = SessionGuard(Arc::clone(&mcu));
        let group = the_group(&world.buttons);
        // Build assigns the oid; the report path needs only that.
        let built = world.chip.config().build(&mcu).unwrap();
        drop(built);

        let first = ButtonsState {
            oid: 0,
            ack_count: 0,
            state: vec![0x01, 0x03],
        };
        let (count, calls) = group.take_report(&first).expect("two new bytes");
        assert_eq!(count, 2);
        assert_eq!(calls.len(), 1, "one registration, one change");
        assert!(calls[0].1, "PA1 reports as pressed");

        // The firmware repeats the same bytes before it hears the ack.
        assert!(
            group.take_report(&first).is_none(),
            "a retransmission carries nothing new"
        );

        // Now the firmware has seen the ack: one newer byte follows.
        let second = ButtonsState {
            oid: 0,
            ack_count: 2,
            state: vec![0x03],
        };
        let (count, calls) = group.take_report(&second).expect("one new byte");
        assert_eq!(count, 1);
        assert!(calls.is_empty(), "the level did not change");
        assert_eq!(group.lock().ack_count, 3);
    }

    /// ③ The firmware's counter wraps: the 8-bit echo is read as a signed
    /// difference from the host's own unbounded count, so a byte that has
    /// wrapped reads as one step — not 255.
    #[tokio::test]
    async fn test_the_firmware_counter_wrap_is_read_as_a_small_step() {
        let world = world();
        world
            .buttons
            .register_buttons(vec!["PA1".to_string()], Box::new(|_, _| {}));
        let mcu = mcu(RecordingWire::new());
        let _session = SessionGuard(Arc::clone(&mcu));
        let group = the_group(&world.buttons);
        drop(world.chip.config().build(&mcu).unwrap());

        // The host has counted 300 bytes, so 300 & 0xff is what the firmware's
        // byte would be. It says 43 instead: one behind, which is the step a
        // wrap produces.
        group.lock().ack_count = 300;
        let behind = ButtonsState {
            oid: 0,
            ack_count: 43,
            state: vec![0x01, 0x03],
        };
        let (count, calls) = group.take_report(&behind).expect("one new byte");
        assert_eq!(count, 1, "the newest byte only");
        assert_eq!(calls.len(), 1, "and only its change");
        assert_eq!(group.lock().ack_count, 301);
    }

    /// A report for an oid this host never built is dropped, not applied to
    /// another object's state.
    #[tokio::test]
    async fn test_a_report_for_an_unknown_oid_is_dropped() {
        let world = world();
        world
            .buttons
            .register_buttons(vec!["PA1".to_string()], Box::new(|_, _| {}));
        let mcu = mcu(RecordingWire::new());
        let _session = SessionGuard(Arc::clone(&mcu));
        let group = the_group(&world.buttons);
        drop(world.chip.config().build(&mcu).unwrap());
        group.router.bind(&mcu).unwrap();

        group.router.route(&ButtonsState {
            oid: 7,
            ack_count: 0,
            state: vec![0x01],
        });
        assert_eq!(group.lock().ack_count, 0);
    }

    // -----------------------------------------------------------------------
    // ④ Debounce
    // -----------------------------------------------------------------------

    /// ④ The host-side debounce: a level has to hold for `debounce_delay`, and
    /// two edges inside that window are one (cancelled) transition, not two
    /// callbacks — upstream's `DebounceButton` (`buttons.py:252-276`).
    #[test]
    fn test_the_host_debounce_holds_a_change_and_drops_a_bounce() {
        let reactor = Arc::new(ManualReactor::new());
        let calls: Arc<Mutex<Vec<(f64, bool)>>> = Arc::new(Mutex::new(Vec::new()));
        let record = Arc::clone(&calls);
        let debounce = DebounceButton::new(
            Arc::clone(&reactor) as Arc<dyn Reactor>,
            0.01,
            Box::new(move |eventtime, state| {
                record.lock().unwrap().push((eventtime, state));
            }),
        );

        // The first level is acted on once it holds.
        debounce.button_handler(1.0, true);
        reactor.advance(1.1);
        assert_eq!(*calls.lock().unwrap(), vec![(1.0, true)]);

        // A bounce back inside the delay: the second edge cancels the first, so
        // the tick produces one callback — the one already made — and not two.
        debounce.button_handler(2.0, false);
        debounce.button_handler(2.005, true);
        reactor.advance(1.0);
        assert_eq!(
            *calls.lock().unwrap(),
            vec![(1.0, true)],
            "a bounce that returns to the logical state must not be acted on"
        );

        // A change that holds is dated at the event, not at the timer.
        debounce.button_handler(3.0, false);
        reactor.advance(1.0);
        assert_eq!(*calls.lock().unwrap(), vec![(1.0, true), (3.0, false)]);
    }

    /// ④ The firmware's own debounce is armed by the query: polls every
    /// `QUERY_TIME` (2 ms), repeats an unacknowledged change 50 times, and
    /// starts from the inverted idle level.
    #[tokio::test]
    async fn test_the_query_arms_the_firmware_debounce() {
        let world = world();
        world.buttons.register_buttons(
            vec!["!PA1".to_string(), "PA2".to_string()],
            Box::new(|_, _| {}),
        );
        let wire = RecordingWire::new();
        let sent = wire.sent();
        let mcu = mcu(wire);
        let _session = SessionGuard(Arc::clone(&mcu));
        let group = the_group(&world.buttons);
        drop(world.chip.config().build(&mcu).unwrap());

        group.arm(&mcu, 0, 1).expect("the query is armed");

        wait_until(
            Duration::from_secs(1),
            Duration::from_millis(5),
            || !sent.lock().unwrap().is_empty(),
            "the query never left",
        )
        .await;
        let query = sent.lock().unwrap()[0].clone();
        let fields = decode_payload(BUTTONS_QUERY, &query);
        // One of the two pins is inverted: only its bit is set.
        assert_eq!(fields[4], ArgValue::UInt8(1));
        // `rest_ticks` is QUERY_TIME, and the change is repeated 50 polls.
        assert_eq!(fields[2], ArgValue::UInt32(2_000));
        assert_eq!(fields[3], ArgValue::UInt8(RETRANSMIT_COUNT));
    }
}
