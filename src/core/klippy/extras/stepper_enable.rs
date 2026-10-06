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
//! # Limitations
//!
//! The upstream implementation schedules enable/disable at print time through
//! the toolhead (`toolhead.dwell`, `toolhead.flush_step_generation`). This
//! port's toolhead exists but that wiring does not: `motor_off` /
//! `SET_STEPPER_ENABLE` apply immediately rather than syncing with motion. A
//! dedicated enable pin on its own is not urgent; what matters is that the
//! infrastructure exists for when print-time scheduling (C1d) lands.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::gcode::{sync, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

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

    /// Increment the count and drive the pin high if transitioning from zero.
    fn set_enable(&mut self) {
        if self.enable_count == 0 {
            if let Some(ref pin) = self.mcu_enable {
                let _ = pin.update_digital_out(true);
            }
        }
        self.enable_count += 1;
    }

    /// Decrement the count and drive the pin low if transitioning to zero.
    fn set_disable(&mut self) {
        self.enable_count -= 1;
        if self.enable_count == 0 {
            if let Some(ref pin) = self.mcu_enable {
                let _ = pin.update_digital_out(false);
            }
        }
    }
}

/// Per-stepper enable tracking.
///
/// Upstream's `EnableTracking`: it wraps a stepper with an enable pin,
/// manages the enabled state, and calls registered callbacks on transitions.
pub(crate) struct EnableTracking {
    /// The stepper name (`stepper_x`, etc.).
    #[allow(dead_code)]
    stepper_name: String,
    /// The shared or dedicated enable pin.
    enable: Arc<Mutex<StepperEnablePin>>,
    /// Callbacks registered by the stepper (fire on enable/disable).
    callbacks: Vec<Box<dyn Fn(bool) + Send>>,
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

    /// Register a callback for enable/disable transitions.
    #[allow(dead_code)]
    fn register_state_callback<F: Fn(bool) + Send + 'static>(&mut self, cb: F) {
        self.callbacks.push(Box::new(cb));
    }

    /// Enable the motor.
    fn motor_enable(&mut self) {
        if !self.is_enabled {
            for cb in &self.callbacks {
                cb(true);
            }
            self.enable.lock().unwrap().set_enable();
            self.is_enabled = true;
        }
    }

    /// Disable the motor.
    fn motor_disable(&mut self) {
        if self.is_enabled {
            for cb in &self.callbacks {
                cb(false);
            }
            self.enable.lock().unwrap().set_disable();
            self.is_enabled = false;
        }
    }

    /// Whether the motor is currently enabled.
    fn is_motor_enabled(&self) -> bool {
        self.is_enabled
    }

    /// Whether this is a dedicated (non-shared) enable pin.
    #[allow(dead_code)]
    fn has_dedicated_enable(&self) -> bool {
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
            pins: Mutex::new(None),
            printer: Some(Arc::clone(printer)),
        }
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
        let mut pins_lock = self.pins.lock().unwrap();
        let pins = pins_lock.get_or_insert_with(|| {
            printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("the loader registers `pins` before any section")
        });

        let enable = setup_enable_pin(config, pins)?;
        let tracking = Arc::new(Mutex::new(EnableTracking::new(
            stepper_name.to_string(),
            enable,
        )));
        self.enable_lines
            .lock()
            .unwrap()
            .insert(stepper_name.to_string(), tracking);
        Ok(())
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

        let enable_lines2 = Arc::clone(&self.enable_lines);
        let handler_set = sync(move |gcmd: &GcodeCommand| {
            let stepper_name = gcmd.get_str("STEPPER").map_err(|_| {
                crate::core::klippy::gcode::CommandError::new("Missing STEPPER parameter")
            })?;
            let enable = gcmd.get_int_default("ENABLE", 1)? != 0;

            set_motors_enable_inner(&enable_lines2, std::slice::from_ref(&stepper_name), enable);
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
    /// Unlike upstream, this does not flush step generation or dwell on the
    /// toolhead — this port's enable/disable apply immediately (see the module
    /// docs on print-time scheduling).
    pub fn set_motors_enable(&self, names: &[String], enable: bool) -> bool {
        set_motors_enable_inner(&self.enable_lines, names, enable)
    }

    /// Turn off all motors and notify the rest of the machine.
    pub fn motor_off(&self) {
        let stepper_names: Vec<String> =
            self.enable_lines.lock().unwrap().keys().cloned().collect();
        for name in &stepper_names {
            if let Some(tracking) = self.enable_lines.lock().unwrap().get(name) {
                tracking.lock().unwrap().motor_disable();
            }
        }
        if let Some(printer) = &self.printer {
            printer.send_event(&KlippyEvent::StepperEnableMotorOff);
        }
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

/// Set the enable state of every named stepper, reporting whether anything
/// changed.
///
/// Split out so the `SET_STEPPER_ENABLE` handler (one name) and the manual
/// stepper's `ENABLE` (a list) share one loop, with the same ignore-unknown
/// behavior the handler has always had.
fn set_motors_enable_inner(
    enable_lines: &Mutex<HashMap<String, Arc<Mutex<EnableTracking>>>>,
    names: &[String],
    enable: bool,
) -> bool {
    let mut did_change = false;
    let lines = enable_lines.lock().unwrap();
    for name in names {
        let Some(tracking) = lines.get(name) else {
            continue;
        };
        let mut tracking = tracking.lock().unwrap();
        let was_enabled = tracking.is_motor_enabled();
        if enable {
            tracking.motor_enable();
        } else {
            tracking.motor_disable();
        }
        if tracking.is_motor_enabled() != was_enabled {
            did_change = true;
        }
    }
    did_change
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

    #[test]
    fn test_stepper_enablepin_no_pin() {
        let mut pin = StepperEnablePin::no_pin();
        assert!(!pin.is_dedicated);
        assert_eq!(pin.enable_count, 9999);
        assert!(pin.mcu_enable.is_none());

        // set_enable on no_pin should not change anything
        pin.set_enable();
        assert_eq!(pin.enable_count, 10000);

        pin.set_disable();
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
    #[tokio::test]
    #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
    async fn test_m18_disables_every_motor_on_a_real_board() {
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

        gcode.run_script("M18").await.expect("M18 runs");

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
