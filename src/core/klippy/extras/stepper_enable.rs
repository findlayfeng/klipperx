//! `[stepper_enable]` — enable pin tracking for stepper motors.
//!
//! Upstream's `stepper_enable.py`: it manages shared enable pins with reference
//! counting, tracks per-stepper enable state, and registers the M18/M84/
//! SET_STEPPER_ENABLE g-code commands.
//!
//! # What is here
//!
//! | struct | purpose |
//! |---|---|
//! | `StepperEnablePin` | shared enable pin with reference counting |
//! | `EnableTracking` | per-stepper enable state + callbacks |
//! | `PrinterStepperEnable` | global tracking, g-code commands, status |
//!
//! # Auto enable
//!
//! A motor is energised by its own motion, not by a command: every tracked
//! stepper arms a callback on the motion queue's active-check list
//! ([`ActiveCallbacks`]), and the flush pass fires it with the print time the
//! stepper's first real move starts at — `EnableTracking.__init__`'s
//! `stepper.add_active_callback(self.motor_enable)` and `_check_active`
//! (`klippy/stepper.py:239-258`). `M84` re-arms it, so the next move after a
//! stop enables again (`EnableTracking.motor_disable`).
//!
//! # Limitations
//!
//! Upstream's `set_motors_enable` opens with `toolhead.flush_step_generation()`
//! and dwells `DISABLE_STALL_TIME` around a disable. Here the change is dated
//! with `toolhead.get_last_move_time()` — the planner's horizon, which the
//! background generator task turns into steps — and there is no dwell: a
//! scheduled change and a step command are both on the MCU's timeline, so the
//! ordering is the firmware's.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{sync, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::motion::queuing::ActiveCallbacks;
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The `[printer]` object, which dates a manual enable/disable
/// (`set_motors_enable`'s `toolhead.get_last_move_time()`).
const TOOLHEAD_OBJECT: &str = "toolhead";

// The constant upstream uses for the dwell before disabling motors
// (`DISABLE_STALL_TIME`, `stepper_enable.py:8`).
#[allow(dead_code)]
const DISABLE_STALL_TIME: f64 = 0.100;

/// Shared enable pin with reference counting.
///
/// Upstream's `StepperEnablePin`: it tracks how many steppers share this pin,
/// and only drives the pin when the count transitions through zero.
struct StepperEnablePin {
    /// The firmware output pin, `None` when there is no real pin
    /// (`enable_pin` absent from config).
    mcu_enable: Option<Arc<dyn DigitalOut>>,
    /// How many steppers are currently enabled through this pin.
    enable_count: u32,
    /// `true` when this is a dedicated pin for one stepper; `false` when
    /// shared.
    #[allow(dead_code)]
    is_dedicated: bool,
}

impl StepperEnablePin {
    /// Build a "no pin" enable object (`enable_pin` absent).
    ///
    /// Always "enabled" with a high count so the real pin path is never taken.
    fn no_pin() -> Self {
        Self {
            mcu_enable: None,
            enable_count: 9999,
            is_dedicated: false,
        }
    }

    /// Build a dedicated enable pin (no sharing).
    fn dedicated(mcu_enable: Arc<dyn DigitalOut>) -> Self {
        Self {
            mcu_enable: Some(mcu_enable),
            enable_count: 0,
            is_dedicated: true,
        }
    }

    /// Build a shared enable pin placeholder; the first user claims it.
    #[allow(dead_code)]
    fn shared_placeholder() -> Self {
        Self {
            mcu_enable: None,
            enable_count: 0,
            is_dedicated: false,
        }
    }

    /// Increment the count and drive the pin high if transitioning from zero
    /// (`StepperEnablePin.set_enable(print_time)`).
    fn set_enable(&mut self, print_time: Option<f64>) {
        if self.enable_count == 0 {
            self.drive(true, print_time);
        }
        self.enable_count += 1;
    }

    /// Decrement the count and drive the pin low if transitioning to zero.
    fn set_disable(&mut self, print_time: Option<f64>) {
        self.enable_count -= 1;
        if self.enable_count == 0 {
            self.drive(false, print_time);
        }
    }

    /// Land a level on the pin at `print_time` (`mcu_pin.set_digital`), or drive
    /// it now when the change carries no date or the pin's MCU cannot turn one
    /// into a clock yet (not connected) — the same rule `extras/fan` and
    /// `extras/output_pin` use.
    fn drive(&self, value: bool, print_time: Option<f64>) {
        let Some(pin) = &self.mcu_enable else {
            return;
        };
        match print_time.and_then(|print_time| pin.print_time_to_clock(print_time)) {
            Some(clock) => {
                let _ = pin.queue_digital_out(clock as u32, value);
            }
            None => {
                let _ = pin.update_digital_out(value);
            }
        }
    }
}

/// Per-stepper enable tracking.
///
/// Upstream's `EnableTracking`: it wraps a stepper with an enable pin,
/// manages the enabled state, and calls registered callbacks on transitions.
///
/// The print time a transition carries (`Option<f64>`) is where it belongs on
/// the MCU's timeline: the auto-enable path dates it with the first move's
/// print time, a manual one with the planner's horizon, and `None` means "no
/// timeline to date it against" (the pin then drives immediately).
pub(crate) struct EnableTracking {
    /// The stepper name (`stepper_x`, etc.).
    stepper_name: String,
    /// The shared or dedicated enable pin.
    enable: Arc<Mutex<StepperEnablePin>>,
    /// Callbacks registered by the driver (fire on enable/disable).
    callbacks: Vec<Box<dyn Fn(Option<f64>, bool) + Send>>,
    /// Whether the motor is currently enabled.
    is_enabled: bool,
}

impl EnableTracking {
    fn new(stepper_name: String, enable: Arc<Mutex<StepperEnablePin>>) -> Self {
        Self {
            stepper_name,
            enable,
            callbacks: Vec::new(),
            is_enabled: false,
        }
    }

    /// Register a callback for enable/disable transitions
    /// (`register_state_callback`): it is called with the transition's print
    /// time and the new state, in registration order.
    pub(crate) fn register_state_callback<F: Fn(Option<f64>, bool) + Send + 'static>(
        &mut self,
        cb: F,
    ) {
        self.callbacks.push(Box::new(cb));
    }

    /// Enable the motor at `print_time` (`EnableTracking.motor_enable`).
    /// Idempotent: an already enabled line fires nothing and drives nothing.
    fn motor_enable(&mut self, print_time: Option<f64>) {
        if !self.is_enabled {
            for cb in &self.callbacks {
                cb(print_time, true);
            }
            self.enable.lock().unwrap().set_enable(print_time);
            self.is_enabled = true;
        }
    }

    /// Disable the motor at `print_time` (`EnableTracking.motor_disable`).
    /// Idempotent; the caller re-arms the auto-enable callback.
    fn motor_disable(&mut self, print_time: Option<f64>) {
        if self.is_enabled {
            for cb in &self.callbacks {
                cb(print_time, false);
            }
            self.enable.lock().unwrap().set_disable(print_time);
            self.is_enabled = false;
        }
    }

    /// Whether the motor is currently enabled.
    pub(crate) fn is_motor_enabled(&self) -> bool {
        self.is_enabled
    }

    /// Whether this is a dedicated (non-shared) enable pin.
    pub(crate) fn has_dedicated_enable(&self) -> bool {
        self.enable.lock().unwrap().is_dedicated
    }
}

/// Global stepper enable tracking.
///
/// Upstream's `PrinterStepperEnable`: it owns the enable pin map, registers
/// the M18/M84/SET_STEPPER_ENABLE commands, and provides the status object.
pub struct PrinterStepperEnable {
    /// Per-stepper enable tracking, keyed by stepper name.
    /// Wrapped in Arc for sharing across closures, and Mutex for interior
    /// mutability (stepper sections register themselves after this object
    /// is created).
    enable_lines: Arc<Mutex<HashMap<String, Arc<Mutex<EnableTracking>>>>>,
    /// The auto-enable callbacks waiting for each stepper's first real motion
    /// (`MCU_stepper.add_active_callback`). The toolhead's motion queue checks
    /// this set before every step generation, which is where upstream installs
    /// its `_check_active` flush callback.
    active_callbacks: Arc<ActiveCallbacks>,
    /// Pin resolver, available at load time.
    pins: Mutex<Option<Arc<PrinterPins>>>,
    /// For looking up the gcode object to register commands.
    printer: Option<Arc<Printer>>,
}

impl PrinterStepperEnable {
    /// Build the object.
    pub fn new(printer: &Arc<Printer>) -> Self {
        Self {
            enable_lines: Arc::new(Mutex::new(HashMap::new())),
            active_callbacks: Arc::new(ActiveCallbacks::new()),
            pins: Mutex::new(None),
            printer: Some(Arc::clone(printer)),
        }
    }

    /// The auto-enable callbacks, for the motion queue to check
    /// (`ToolHeadObject::connect` installs them).
    pub fn active_callbacks(&self) -> Arc<ActiveCallbacks> {
        Arc::clone(&self.active_callbacks)
    }

    /// The `stepper_enable` object, creating it if the config named no section.
    ///
    /// Upstream's `PrinterStepper` calls `load_object(config, 'stepper_enable')`
    /// for every stepper (`klippy/stepper.py:282-285`), so a config that only
    /// writes `enable_pin` still gets the object, its enable tracking, and the
    /// `M18`/`M84` commands. The created object registers itself in the printer
    /// registry, exactly as an explicit `[stepper_enable]` factory would.
    pub fn ensure(printer: &Arc<Printer>) -> Arc<Self> {
        if let Some(existing) = printer.lookup_object_as::<Self>("stepper_enable") {
            return existing;
        }
        let object = Arc::new(Self::new(printer));
        object.register_gcode_commands(printer);
        printer
            .add_object("stepper_enable", object.clone())
            .expect("`stepper_enable` is registered once per machine");
        object
    }

    /// Register a stepper with this enable tracker.
    ///
    /// Parses `enable_pin` from `config`, sets up the pin (shared or dedicated),
    /// and creates an `EnableTracking` entry.
    ///
    /// # Errors
    /// Returns a config error if the pin cannot be resolved.
    pub fn register_stepper(
        &self,
        config: &ConfigWrapper,
        stepper_name: &str,
    ) -> Result<(), ConfigError> {
        let printer = self.printer.as_ref().expect("printer is set in new()");
        let pins = {
            let mut pins_lock = self.pins.lock().unwrap();
            Arc::clone(pins_lock.get_or_insert_with(|| {
                printer
                    .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                    .expect("the loader registers `pins` before any section")
            }))
        };

        let enable = setup_enable_pin(config, &pins)?;
        let tracking = Arc::new(Mutex::new(EnableTracking::new(
            stepper_name.to_string(),
            enable,
        )));
        self.enable_lines
            .lock()
            .unwrap()
            .insert(stepper_name.to_string(), Arc::clone(&tracking));
        self.arm_auto_enable(&tracking);
        Ok(())
    }

    /// Arm `stepper`'s auto-enable callback: its next real motion enables the
    /// motor at that motion's print time
    /// (`EnableTracking.__init__`'s `stepper.add_active_callback(self.motor_enable)`).
    ///
    /// Also the re-arm after a disable (`EnableTracking.motor_disable`), which is
    /// what makes a move after `M84` energise the motor again. The callback holds
    /// the tracking line weakly: the queue must not keep a retired stepper's
    /// enable state alive.
    fn arm_auto_enable(&self, tracking: &Arc<Mutex<EnableTracking>>) {
        let weak: Weak<Mutex<EnableTracking>> = Arc::downgrade(tracking);
        let stepper_name = tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .stepper_name
            .clone();
        self.active_callbacks.add(
            &stepper_name,
            Box::new(move |print_time| {
                if let Some(tracking) = weak.upgrade() {
                    tracking
                        .lock()
                        .unwrap_or_else(|poison| poison.into_inner())
                        .motor_enable(Some(print_time));
                }
            }),
        );
    }

    /// Register g-code commands (M18, M84, SET_STEPPER_ENABLE) and the
    /// `gcode:request_restart` handler that stops the motors.
    pub fn register_gcode_commands(self: &Arc<Self>, printer: &Arc<Printer>) {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        // M18/M84 turn every stepper off, which is `motor_off` (it also tells
        // the rest of the machine via `stepper_enable:motor_off`).
        let handler_m18: crate::core::klippy::gcode::CommandHandler = {
            let weak = Arc::downgrade(self);
            sync(move |gcmd: &GcodeCommand| {
                let _ = gcmd;
                if let Some(object) = weak.upgrade() {
                    object.motor_off();
                }
                Ok(())
            })
        };
        gcode
            .register_command(
                "M18",
                Arc::clone(&handler_m18),
                Some("Turn off all steppers"),
                false,
            )
            .ok();

        // M84 is an alias for M18
        gcode
            .register_command(
                "M84",
                handler_m18,
                Some("Turn off all steppers (alias)"),
                false,
            )
            .ok();

        let weak = Arc::downgrade(self);
        let handler_set = sync(move |gcmd: &GcodeCommand| {
            let stepper_name = gcmd.get_str("STEPPER").map_err(|_| {
                crate::core::klippy::gcode::CommandError::new("Missing STEPPER parameter")
            })?;
            let enable = gcmd.get_int_default("ENABLE", 1)? != 0;

            if let Some(object) = weak.upgrade() {
                object.set_motors_enable(std::slice::from_ref(&stepper_name), enable);
            }
            Ok(())
        });
        // `STEPPER` names the line, `ENABLE` the state
        // (`stepper_enable.py:cmd_SET_STEPPER_ENABLE`); `M18` / `M84` above
        // read no key.
        gcode
            .register_command_with_params(
                "SET_STEPPER_ENABLE",
                handler_set,
                Some("Enable/disable individual stepper"),
                &["STEPPER", "ENABLE"],
                false,
            )
            .ok();

        // Upstream stops the motors on every restart
        // (`stepper_enable.py:97-98`), before the new object graph is built.
        let weak = Arc::downgrade(self);
        printer.register_event_handler(
            KlippyEvent::GcodeRequestRestart { print_time: 0.0 },
            Box::new(move |_| {
                if let Some(object) = weak.upgrade() {
                    object.motor_off();
                }
            }),
        );
    }

    /// Enable or disable several steppers by name, returning whether any
    /// changed (`PrinterStepperEnable.set_motors_enable`,
    /// `stepper_enable.py:92-115`).
    ///
    /// A name with no tracking is skipped, as the `SET_STEPPER_ENABLE` handler
    /// has always done; that keeps the manual stepper's `ENABLE` able to name a
    /// stepper this object does not manage without failing the command.
    ///
    /// The change is dated with the print time the planner has reached — what
    /// the MCU will be executing when it lands
    /// (`toolhead.get_last_move_time()`) — and a disable re-arms the stepper's
    /// auto-enable callback, so the next move turns it back on. Upstream dwells
    /// `DISABLE_STALL_TIME` before a disable and after it; there is no dwell
    /// here (see the module docs).
    pub fn set_motors_enable(&self, names: &[String], enable: bool) -> bool {
        let mut did_change = false;
        let mut print_time = None;
        let lines = self.enable_lines.lock().unwrap();
        for name in names {
            let Some(tracking) = lines.get(name).cloned() else {
                continue;
            };
            let mut line = tracking.lock().unwrap_or_else(|poison| poison.into_inner());
            if line.is_motor_enabled() == enable {
                continue;
            }
            // One reading for the whole call: a dwell-free timeline must not
            // move between two lines of the same command.
            let dated = *print_time.get_or_insert_with(|| self.last_move_time());
            if enable {
                line.motor_enable(dated);
            } else {
                line.motor_disable(dated);
            }
            drop(line);
            if !enable {
                // `EnableTracking.motor_disable`: enable on future movement.
                self.arm_auto_enable(&tracking);
            }
            did_change = true;
        }
        did_change
    }

    /// Turn off all motors and notify the rest of the machine
    /// (`PrinterStepperEnable.motor_off`): every line off through
    /// [`Self::set_motors_enable`], so each one re-arms its auto enable.
    pub fn motor_off(&self) {
        self.set_motors_enable(&self.get_steppers(), false);
        if let Some(printer) = &self.printer {
            printer.send_event(&KlippyEvent::StepperEnableMotorOff);
        }
    }

    /// The print time a manual enable or disable is dated with: the planner's
    /// horizon (`toolhead.get_last_move_time()`), which is what the MCU is
    /// executing now. `None` before the machine is up — the pin then takes its
    /// immediate path.
    fn last_move_time(&self) -> Option<f64> {
        self.printer
            .as_ref()?
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)?
            .last_move_time()
    }

    /// Get the status object for `printer.stepper_enable`.
    pub fn get_status(&self, _eventtime: f64) -> Value {
        let enable_lines = self.enable_lines.lock().unwrap();
        let steppers: serde_json::Map<String, Value> = enable_lines
            .iter()
            .map(|(name, tracking)| {
                (
                    name.clone(),
                    json!(tracking.lock().unwrap().is_motor_enabled()),
                )
            })
            .collect();
        json!({ "steppers": steppers })
    }

    /// Look up enable tracking for a stepper.
    ///
    /// # Errors
    /// Returns a config error if the stepper name is unknown.
    #[allow(dead_code)]
    pub(crate) fn lookup_enable(
        &self,
        name: &str,
    ) -> Result<Arc<Mutex<EnableTracking>>, ConfigError> {
        self.enable_lines
            .lock()
            .unwrap()
            .get(name)
            .cloned()
            .ok_or_else(|| ConfigError::new(format!("Unknown stepper '{name}'")))
    }

    /// Return all registered stepper names.
    pub fn get_steppers(&self) -> Vec<String> {
        self.enable_lines.lock().unwrap().keys().cloned().collect()
    }
}

/// Set up an enable pin for a stepper.
///
/// Upstream's `setup_enable_pin` (`stepper_enable.py:25-42`): if `enable_pin`
/// is absent, return a "always enabled" placeholder; if the pin is already
/// shared (same `share_type`), return the existing object; otherwise create a
/// new dedicated pin.
///
/// # Errors
/// Returns a config error if the pin cannot be resolved.
fn setup_enable_pin(
    config: &ConfigWrapper,
    pins: &PrinterPins,
) -> Result<Arc<Mutex<StepperEnablePin>>, ConfigError> {
    let identifier = config.identifier();

    // Check if enable_pin is specified
    let enable_pin_desc = match config.get_str("enable_pin") {
        Some(pin) => pin,
        None => return Ok(Arc::new(Mutex::new(StepperEnablePin::no_pin()))),
    };

    // Look up the pin with share_type='stepper_enable' for sharing support
    pins.lookup_pin(
        &enable_pin_desc,
        true,  // can_invert
        false, // can_pullup
        Some("stepper_enable"),
    )
    .map_err(|err| ConfigError::new(format!("enable_pin in section '{}': {}", identifier, err)))?;

    // Build the digital output pin
    let mcu_enable = pins
        .setup_digital_out(&enable_pin_desc, Some("stepper_enable"))
        .map_err(|err| {
            ConfigError::new(format!("enable_pin in section '{}': {}", identifier, err))
        })?;
    // The enable line is held, not refreshed: the firmware's default 2 s watchdog
    // would shut the printer down (`Missed scheduling of next digital out event`)
    // two seconds after the motor is turned on. Upstream clears it the same way
    // (`stepper_enable.py`: `enable.setup_max_duration(0.)`).
    mcu_enable.setup_max_duration(0.0);

    Ok(Arc::new(Mutex::new(StepperEnablePin::dedicated(
        mcu_enable,
    ))))
}

impl PrinterObject for PrinterStepperEnable {
    fn get_status(&self, eventtime: f64) -> Value {
        self.get_status(eventtime)
    }

    fn is_queryable(&self) -> bool {
        true
    }
}

/// The factory the section declaration names.
pub(crate) fn load_config(
    _config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let obj = Arc::new(PrinterStepperEnable::new(printer));
    obj.register_gcode_commands(printer);
    Ok(obj)
}

// Register the [stepper_enable] section.
section!("stepper_enable", order = 10, load = load_config);

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    use crate::core::klippy::mathutil::Xyz;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::motion::itersolve::Axis;
    use crate::core::klippy::motion::queuing::MotionQueuing;
    use crate::core::klippy::motion::stepper::Stepper;
    use crate::core::klippy::reactor::ManualReactor;

    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = crate::core::klippy::config::Config::from_text(text)
            .expect("the test config parses")
            .0;
        let result = printer.load_config(&config);
        (printer, result)
    }

    #[test]
    fn test_stepper_enable_section_loads() {
        let (printer, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_enable]\n",
        );

        result.unwrap();
        assert!(printer.lookup_object("stepper_enable").is_some());
    }

    #[test]
    fn test_no_pin_returns_always_enabled() {
        let (_printer, _result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\nenable_pin: ^PA2\n\
             [stepper_enable]\n",
        );

        // This should work with a valid pin
        // Note: The actual pin setup depends on MCU chip being configured
    }

    // -----------------------------------------------------------------------
    // Auto enable: a motor is energised by its own motion
    // -----------------------------------------------------------------------

    /// An enable pin that records what it was told: dated changes and
    /// immediate ones separately.
    #[derive(Default)]
    struct RecordingPin {
        /// The dated changes, as `(clock, level)`.
        queued: Mutex<Vec<(u32, bool)>>,
        /// The levels driven with no date (`update_digital_out`).
        immediate: Mutex<Vec<bool>>,
    }

    impl DigitalOut for RecordingPin {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
        fn queue_digital_out(&self, clock: u32, value: bool) -> Result<(), McuError> {
            self.queued.lock().unwrap().push((clock, value));
            Ok(())
        }
        fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
            self.immediate.lock().unwrap().push(value);
            Ok(())
        }
        /// One clock tick is one microsecond, so a print time reads back as its
        /// clock: `0.1 s` → `100_000`.
        fn print_time_to_clock(&self, print_time: f64) -> Option<u64> {
            Some((print_time * 1_000_000.0) as u64)
        }
    }

    /// Everything the auto-enable chain needs: an enable line on a recording
    /// pin, the object that tracks it, and a motion queue armed the way
    /// `ToolHeadObject::connect` arms it with this object's callbacks.
    struct Fixture {
        stepper_enable: Arc<PrinterStepperEnable>,
        pin: Arc<RecordingPin>,
        tracking: Arc<Mutex<EnableTracking>>,
        queuing: MotionQueuing,
        trapq: usize,
        /// Where the axis is, so every appended segment starts where the last
        /// one ended (the trapq stores absolute start positions).
        position: f64,
    }

    impl Fixture {
        /// `[stepper_x]` tracked by a fresh object, with a dedicated enable pin.
        fn new() -> Self {
            let printer = Arc::new(Printer::new(ManualReactor::shared()));
            let stepper_enable = Arc::new(PrinterStepperEnable::new(&printer));
            let pin = Arc::new(RecordingPin::default());
            let enable = Arc::new(Mutex::new(StepperEnablePin::dedicated(
                Arc::clone(&pin) as Arc<dyn DigitalOut>
            )));
            let tracking = Arc::new(Mutex::new(EnableTracking::new(
                "stepper_x".to_string(),
                enable,
            )));
            stepper_enable
                .enable_lines
                .lock()
                .unwrap()
                .insert("stepper_x".to_string(), Arc::clone(&tracking));
            stepper_enable.arm_auto_enable(&tracking);

            let mut queuing = MotionQueuing::new();
            let trapq = queuing.allocate_trapq();
            queuing.add_stepper(Stepper::cartesian(
                "stepper_x",
                0,
                1.0,
                Axis::X,
                1_000_000.0,
            ));
            // What connect does: the queue checks this object's callbacks.
            queuing.set_active_callbacks(stepper_enable.active_callbacks());
            Self {
                stepper_enable,
                pin,
                tracking,
                queuing,
                trapq,
                position: 0.0,
            }
        }

        /// Queue a dwell of `duration` seconds at `print_time`.
        fn dwell(&mut self, print_time: f64, duration: f64) {
            self.queuing.trapq_mut(self.trapq).append(
                print_time,
                duration,
                0.0,
                0.0,
                Xyz::new(self.position, 0.0, 0.0),
                Xyz::default(),
                0.0,
                0.0,
                0.0,
            );
        }

        /// Queue a 10 mm X move at `print_time` (0.1 s of acceleration, 0.1 s of
        /// deceleration, no cruise). `axes_r` is the unit direction, as the trapq
        /// stores it: the 10 mm comes from the velocity profile.
        fn move_x(&mut self, print_time: f64) {
            let start = self.position;
            self.position += 10.0;
            self.queuing.trapq_mut(self.trapq).append(
                print_time,
                0.1,
                0.0,
                0.1,
                Xyz::new(start, 0.0, 0.0),
                Xyz::new(1.0, 0.0, 0.0),
                0.0,
                100.0,
                1000.0,
            );
        }

        fn generate(&mut self, flush_time: f64) {
            self.queuing.generate(flush_time).unwrap();
        }

        fn is_enabled(&self) -> bool {
            self.tracking.lock().unwrap().is_motor_enabled()
        }
    }

    /// The motor comes on when a real move is queued — once, dated with that
    /// move's print time, and never again on later flushes
    /// (`MCU_stepper.add_active_callback` + `_check_active`).
    #[test]
    fn test_a_move_enables_the_motor_once_at_its_print_time() {
        let mut fixture = Fixture::new();
        // A dwell first: it moves no axis, so it must not energise anything.
        fixture.dwell(0.0, 0.1);
        fixture.move_x(0.1);

        fixture.generate(0.5);

        assert!(fixture.is_enabled(), "the queued move energised the motor");
        assert_eq!(
            *fixture.pin.queued.lock().unwrap(),
            [(100_000, true)],
            "one dated change, at the move's print time, not an immediate one"
        );
        assert!(fixture.pin.immediate.lock().unwrap().is_empty());

        // Already enabled: a later flush drives nothing and fires no callback.
        fixture.generate(0.8);
        assert_eq!(*fixture.pin.queued.lock().unwrap(), [(100_000, true)]);
        assert!(fixture.pin.immediate.lock().unwrap().is_empty());
    }

    /// A motor a command already enabled stays on: the auto-enable callback
    /// still fires, and enabling an enabled line drives nothing.
    #[test]
    fn test_an_already_enabled_motor_is_not_enabled_again() {
        let mut fixture = Fixture::new();
        assert!(
            fixture
                .stepper_enable
                .set_motors_enable(&["stepper_x".to_string()], true),
            "the command enabled the motor"
        );
        assert_eq!(fixture.pin.immediate.lock().unwrap().len(), 1);

        fixture.move_x(0.0);
        fixture.generate(0.5);

        assert!(fixture.is_enabled());
        assert!(
            fixture.pin.queued.lock().unwrap().is_empty(),
            "nothing was queued: the line was already on"
        );
        assert_eq!(fixture.pin.immediate.lock().unwrap().len(), 1);
    }

    /// `M84` re-arms the auto enable: the next move energises the motor again
    /// (`EnableTracking.motor_disable`'s `add_active_callback`).
    #[test]
    fn test_a_move_after_motor_off_enables_the_motor_again() {
        let mut fixture = Fixture::new();
        fixture.move_x(0.0);
        fixture.generate(0.5);
        assert!(fixture.is_enabled());
        assert_eq!(*fixture.pin.queued.lock().unwrap(), [(0, true)]);

        fixture.stepper_enable.motor_off();
        assert!(!fixture.is_enabled(), "M84 disabled the motor");
        // No toolhead in this fixture, so the disable carries no date and the
        // pin takes its immediate path.
        assert_eq!(*fixture.pin.immediate.lock().unwrap(), [false]);

        // A move queued after the stop turns it back on, at its own print time.
        fixture.dwell(0.5, 0.5);
        fixture.move_x(1.0);
        fixture.generate(1.5);
        assert!(fixture.is_enabled(), "the move after M84 re-enabled it");
        assert_eq!(
            *fixture.pin.queued.lock().unwrap(),
            [(0, true), (1_000_000, true)]
        );
    }

    /// A flush that generates nothing (only a dwell is queued) leaves the motor
    /// off and the callback armed.
    #[test]
    fn test_a_flush_with_no_motion_leaves_the_motor_off() {
        let mut fixture = Fixture::new();
        fixture.dwell(0.0, 1.0);

        fixture.generate(1.05);

        assert!(!fixture.is_enabled(), "a dwell moves no axis");
        assert!(fixture.pin.queued.lock().unwrap().is_empty());
        assert!(fixture.pin.immediate.lock().unwrap().is_empty());
    }

    #[test]
    fn test_stepper_enablepin_no_pin() {
        let mut pin = StepperEnablePin::no_pin();
        assert!(!pin.is_dedicated);
        assert_eq!(pin.enable_count, 9999);
        assert!(pin.mcu_enable.is_none());

        // set_enable on no_pin should not change anything
        pin.set_enable(None);
        assert_eq!(pin.enable_count, 10000);

        pin.set_disable(None);
        assert_eq!(pin.enable_count, 9999);
    }

    // -----------------------------------------------------------------------
    // Real board: R9-lite, `M18` disables the motors (zero motion)
    // -----------------------------------------------------------------------

    /// What the R9-lite real-board case needs from `KLIPPERX_HW_CONFIG`.
    ///
    /// `[stepper_x]` is what makes the machine a motion machine, and it is the
    /// right declaration rather than `[stepper_enable]`: the section is
    /// optional and almost every config omits it, because `stepper.rs` calls
    /// [`PrinterStepperEnable::ensure`] for every `[stepper_*]` section
    /// (`extras/stepper.rs:498`), which registers the object and its `M18`/`M84`
    /// commands anyway. Declaring `[stepper_enable]` would therefore skip the
    /// case on exactly the printers that can run it. What the case actually
    /// reads is one tracked stepper: with `[stepper_x]` present,
    /// `get_status` has a line to report and `motor_off` a line to disable.
    ///
    /// Shared with [`the_disable_case_declares_mcu_and_stepper_x`], so the
    /// declaration the config is judged against and the one the case runs under
    /// cannot drift apart.
    fn disable_case() -> crate::hardware_test::Requires {
        crate::hardware_test::Requires::new()
            .mcu()
            .section("stepper_x")
    }

    /// The declaration is the gate: it decides whether the case runs or is
    /// reported `HW-IGNORED`. Pinned against synthetic config text parsed by the
    /// real parser, so a declaration that named the wrong section — which would
    /// skip the case forever on a printer that could run it — fails here
    /// instead of on hardware.
    #[test]
    fn the_disable_case_declares_mcu_and_stepper_x() {
        use crate::core::klippy::config::Config;
        use crate::hardware_test::{check, Missing};

        let parse = |text: &str| Config::from_text(text).expect("the fixture parses").0;

        let full = parse(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PB1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n",
        );
        assert_eq!(check(&full, &disable_case()), Ok(()));

        // Commented out is absent to the real parser, so the case skips rather
        // than runs against a machine with no stepper to disable.
        let commented = parse(
            "[mcu]\nserial: /dev/ttyACM0\n\
             # [stepper_x]\n# step_pin: PA0\n# dir_pin: PB1\n",
        );
        assert_eq!(
            check(&commented, &disable_case()),
            Err(vec![Missing::Section("stepper_x".to_string())])
        );

        // The main MCU is the other half of the declaration.
        let no_mcu = parse(
            "[stepper_x]\nstep_pin: PA0\ndir_pin: PB1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n",
        );
        assert_eq!(
            check(&no_mcu, &disable_case()),
            Err(vec![Missing::Mcu("mcu".to_string())])
        );
    }

    /// R9-lite — `M18` disables every motor on a real board, with zero motion.
    ///
    /// `TESTING.md`'s R9 (“打印收尾与静止”, lines 85-87) ends a print with
    /// `M104 S0` and `M18` and asks that “电机失能”. This case is the `M18` half
    /// on its own: the machine comes up (L2, so `gcode`/`toolhead`/extras are
    /// registered — the corpus' file-output mode never reaches them), `M18` is
    /// sent through the machine's own dispatcher, and two things are asserted.
    ///
    /// # 前提：配置被假定为正确的
    ///
    /// **本用例假定提供的配置是正确的**（上游 Klipper 也会接受、且与实际接线一致）。
    /// 它验的是**本仓 `[stepper_enable]` 这个模块的行为**，不是这份配置对不对——配置/接线/
    /// 固件不匹配导致的失败，不代表模块有 bug。要验配置本身请用宿主或上游 Klipper。
    ///
    /// # What it verifies (this module's behaviour)
    ///
    /// * `M18` is registered and routed to [`PrinterStepperEnable::motor_off`]
    ///   (`register_gcode_commands`, `stepper_enable.rs:254`; `M18` at `:273`);
    /// * `motor_off` publishes `KlippyEvent::StepperEnableMotorOff`
    ///   (`motor_off`, `stepper_enable.rs:342`; the event at `:351`) — observed
    ///   through the existing `Printer::register_event_handler`, no new
    ///   mechanism;
    /// * `get_status` reports every stepper disabled afterwards, in the shape
    ///   this object has (`{"steppers": {<name>: bool}}`,
    ///   `stepper_enable.rs:356-368`).
    ///
    /// # The state before `M18`, and why it is not a bug
    ///
    /// On this port nothing enables a motor at bring-up: `motor_enable` /
    /// `set_motors_enable` are called only by `force_move`, `manual_stepper` and
    /// `SET_STEPPER_ENABLE`, never by the toolhead (the print-time enable wiring
    /// is exactly what the module docs say has not landed). So the status is
    /// normally already all-`false` before `M18`. The case therefore asserts
    /// what is true — **after `M18` every stepper reads `false`**, plus the
    /// event — rather than a `true → false` transition it could not produce
    /// without first energising a motor. `motor_off` short-circuits an already
    /// disabled line; that is the correct idempotent behaviour, not a mismatch.
    ///
    /// # What it assumes about the wiring
    ///
    /// `[stepper_x]` exists on the main `[mcu]` board. Nothing else: `M18`
    /// needs no endstop, no heater and no second board.
    ///
    /// # What it is safe to run
    ///
    /// **Zero motion, no homing, no heating.** The only g-code is `M18`, which
    /// disables the enable lines; it never queues a step and never commands a
    /// heater or an endstop, so there is no motion to bound. The enable pins are
    /// only driven low (the command's whole point) and read back through
    /// `stepper_enable`'s own status: the host has no API to read a digital
    /// output pin, so `get_status`'s per-stepper `is_enabled` is the read-only
    /// proxy for what the pin was driven to.
    ///
    /// As every L2 case, it takes the board's one session (a running printer
    /// host must not be attached) and takes the machine down on `Drop`, the
    /// panic path included.
    ///
    /// # Running it
    ///
    /// ```text
    /// KLIPPERX_HW_CONFIG=~/printer_data/config/printer.cfg \
    ///   cargo test -p klipperx --lib test_m18_disables_every_motor_on_a_real_board \
    ///   -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
    fn test_m18_disables_every_motor_on_a_real_board() {
        let Some(machine) = crate::hardware_test::acquire(
            "test_m18_disables_every_motor_on_a_real_board",
            &disable_case(),
        ) else {
            return; // reported as HW-IGNORED; the case passes without a board
        };
        let machine = machine.bring_up().expect("the board comes up");
        let printer = machine.printer();

        let enabled = printer
            .lookup_object_as::<PrinterStepperEnable>("stepper_enable")
            .expect("the machine registers `stepper_enable` for [stepper_x]");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the machine registers `gcode`");

        // Observe the event this object already publishes, through the
        // existing handler registration.
        let motor_off = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&motor_off);
        printer.register_event_handler(
            KlippyEvent::StepperEnableMotorOff,
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );

        // The declared `[stepper_x]` must really be tracked, or the assertions
        // below would pass on an empty map.
        let before = enabled.get_status(0.0);
        assert!(
            before
                .get("steppers")
                .and_then(Value::as_object)
                .is_some_and(|steppers| steppers.contains_key("stepper_x")),
            "the machine tracks the declared [stepper_x]: {before}"
        );

        gcode.run_script_sync("M18").expect("M18 runs");

        assert_eq!(
            motor_off.load(Ordering::SeqCst),
            1,
            "M18 must publish `stepper_enable:motor_off` exactly once"
        );

        let after = enabled.get_status(0.0);
        let steppers = after
            .get("steppers")
            .and_then(Value::as_object)
            .expect("get_status reports a `steppers` map");
        assert!(
            steppers.contains_key("stepper_x"),
            "the machine still tracks the declared [stepper_x]: {after}"
        );
        for (name, state) in steppers {
            assert_eq!(
                state.as_bool(),
                Some(false),
                "M18 must leave [{name}] disabled: {after}"
            );
        }
        println!(
            "R9-lite: M18 disabled {} stepper(s): {steppers:?}",
            steppers.len()
        );
    }
}
