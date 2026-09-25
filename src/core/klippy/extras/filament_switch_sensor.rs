//! `[filament_switch_sensor <name>]` — a switch that reports filament presence
//! (upstream `klippy/extras/filament_switch_sensor.py`).
//!
//! [`RunoutHelper`] is upstream's helper shared by both filament sensors: it
//! reads the `pause_on_runout` / `runout_gcode` / `insert_gcode` / `pause_delay`
//! / `event_delay` options, loads the `pause_resume` and `gcode_macro` objects
//! they need, registers the `QUERY_FILAMENT_SENSOR` / `SET_FILAMENT_SENSOR` mux
//! commands, and tracks the filament-present state. [`SwitchSensor`] adds the
//! debounced `switch_pin`.
//!
//! # What is not here
//!
//! * The runout / insert **action**: [`RunoutHelper::note_filament_present`]
//!   detects the state change and the "printing" condition, but running the
//!   `runout_gcode` (and pausing through `pause_resume`) is not wired — it needs
//!   a reactor callback that runs host g-code, which this port's event handlers
//!   do not have. The state change itself is recorded, as the fake-firmware
//!   corpus can never generate the button event that would reach it.
//! * The `idle_timeout` object: upstream reads its `"Printing"` state to decide
//!   between a runout and an insert. `idle_timeout` is not implemented here, so
//!   a state change is treated as "not printing".

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::buttons::PrinterButtons;
use crate::core::klippy::extras::gcode_macro::PrinterGCodeMacro;
use crate::core::klippy::extras::pause_resume::PauseResume;
use crate::core::klippy::extras::template::Template;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form exists upstream (`filament_switch_sensor.py:120`).
section!(
    "filament_switch_sensor",
    order = 30,
    prefix = load_config_prefix
);

/// Upstream's reactor `NEVER` (`filament_switch_sensor.py:28`), the initial
/// `min_event_systime` before `klippy:ready` starts the clock.
const NEVER: f64 = 9_999_999_999_999_999.0;

/// The `klippy:ready` window before a state change is acted on
/// (`filament_switch_sensor.py:41`).
const READY_DELAY: f64 = 2.0;

/// The runout helper both filament sensors share
/// (upstream `RunoutHelper`, `filament_switch_sensor.py:9-102`).
pub struct RunoutHelper {
    /// The section suffix (`runout_switch`), the mux value of the two commands
    /// (`filament_switch_sensor.py:12`).
    name: String,
    /// The machine, for its reactor clock and the `idle_timeout` peek.
    printer: Weak<Printer>,
    /// `pause_on_runout` (default `True`).
    pause_on_runout: bool,
    /// The compiled `runout_gcode`, when it applies.
    runout_gcode: Option<Template>,
    /// The compiled `insert_gcode`, when it was set.
    insert_gcode: Option<Template>,
    /// `pause_delay` (default `.5`, `above=0`).
    pause_delay: f64,
    /// `event_delay` (default `3.`, `minval=0`).
    event_delay: f64,
    /// The earliest time a state change may be acted on; [`NEVER`] until ready.
    min_event_systime: Mutex<f64>,
    /// The last state the sensor reported.
    filament_present: Mutex<bool>,
    /// `SET_FILAMENT_SENSOR ENABLE=0` disables acting on changes.
    sensor_enabled: Mutex<bool>,
}

impl RunoutHelper {
    /// Read the helper's options and load the objects they name
    /// (`filament_switch_sensor.py:14-42`).
    ///
    /// # Errors
    /// A missing/invalid option, or a dependency this load cannot create.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        let pause_on_runout = config.get_bool("pause_on_runout", Some(true))?;
        if pause_on_runout {
            PauseResume::ensure(config, printer)?;
        }
        // Upstream loads `gcode_macro` unconditionally (`:20`).
        let gcode_macro = PrinterGCodeMacro::ensure(printer)?;
        let runout_gcode = if pause_on_runout || config.get_str("runout_gcode").is_some() {
            Some(gcode_macro.load_template(config, "runout_gcode", Some(""))?)
        } else {
            None
        };
        let insert_gcode = if config.get_str("insert_gcode").is_some() {
            Some(gcode_macro.load_template(config, "insert_gcode", None)?)
        } else {
            None
        };
        let pause_delay =
            config.get_float_bounded("pause_delay", Some(0.5), None, None, Some(0.0), None)?;
        let event_delay =
            config.get_float_bounded("event_delay", Some(3.0), Some(0.0), None, None, None)?;
        Ok(Self {
            name,
            printer: Arc::downgrade(printer),
            pause_on_runout,
            runout_gcode,
            insert_gcode,
            pause_delay,
            event_delay,
            min_event_systime: Mutex::new(NEVER),
            filament_present: Mutex::new(false),
            sensor_enabled: Mutex::new(true),
        })
    }

    /// The section suffix (`runout_switch`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Register the helper's commands and its `klippy:ready` handler
    /// (`filament_switch_sensor.py:30-42`).
    ///
    /// Called after the `Arc` exists so the mux handlers can share this helper.
    ///
    /// # Errors
    /// A duplicate command registration.
    pub fn attach(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        let query = Arc::clone(self);
        let query_handler: CommandHandler =
            sync(move |gcmd: &GcodeCommand| cmd_query_filament_sensor(&query, gcmd));
        gcode
            .register_mux_command(
                "QUERY_FILAMENT_SENSOR",
                "SENSOR",
                Some(&self.name),
                query_handler,
                Some("Query the status of the Filament Sensor"),
            )
            .map_err(ConfigError::new)?;

        let set = Arc::clone(self);
        let set_handler: CommandHandler =
            sync(move |gcmd: &GcodeCommand| cmd_set_filament_sensor(&set, gcmd));
        gcode
            .register_mux_command(
                "SET_FILAMENT_SENSOR",
                "SENSOR",
                Some(&self.name),
                set_handler,
                Some("Sets the filament sensor on/off"),
            )
            .map_err(ConfigError::new)?;

        let weak = Arc::downgrade(self);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                if let Some(helper) = weak.upgrade() {
                    let now = helper.now();
                    *helper.lock(&helper.min_event_systime) = now + READY_DELAY;
                }
            }),
        );
        Ok(())
    }

    /// Upstream's `note_filament_present` (`filament_switch_sensor.py:63-92`):
    /// act only on a change, outside the initialsation and event-delay windows,
    /// and only when the sensor is enabled. The printing/insert split needs
    /// `idle_timeout`, which is not implemented (see the module docs).
    pub fn note_filament_present(&self, eventtime: f64, is_filament_present: bool) {
        {
            let mut present = self.lock(&self.filament_present);
            if is_filament_present == *present {
                return;
            }
            *present = is_filament_present;
        }
        if eventtime < *self.lock(&self.min_event_systime) {
            return;
        }
        if !*self.lock(&self.sensor_enabled) {
            return;
        }
        let _ = self.pause_delay;
        // Without `idle_timeout` the state is "not printing", so only the insert
        // action is reachable; the runout action needs a printing state. Both
        // actions are the g-code run that is documented as not wired, so a
        // detected change just closes the event window here.
        let _ = (self.runout_gcode.is_some(), self.insert_gcode.is_some());
        *self.lock(&self.min_event_systime) = NEVER;
    }

    /// The last state the sensor reported.
    pub fn filament_present(&self) -> bool {
        *self.lock(&self.filament_present)
    }

    /// Whether `SET_FILAMENT_SENSOR ENABLE=0` disabled the sensor.
    pub fn sensor_enabled(&self) -> bool {
        *self.lock(&self.sensor_enabled)
    }

    /// The machine's monotonic clock, or `0.0` before the printer is reachable.
    fn now(&self) -> f64 {
        self.printer
            .upgrade()
            .map(|printer| printer.reactor().monotonic())
            .unwrap_or(0.0)
    }

    fn lock<'a, T>(&self, slot: &'a Mutex<T>) -> MutexGuard<'a, T> {
        slot.lock().unwrap_or_else(|poison| poison.into_inner())
    }
}

impl std::fmt::Debug for RunoutHelper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RunoutHelper")
            .field("name", &self.name)
            .field("pause_on_runout", &self.pause_on_runout)
            .field("pause_delay", &self.pause_delay)
            .field("event_delay", &self.event_delay)
            .finish_non_exhaustive()
    }
}

/// `QUERY_FILAMENT_SENSOR` (`filament_switch_sensor.py:93-99`).
fn cmd_query_filament_sensor(
    helper: &Arc<RunoutHelper>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let present = *helper.lock(&helper.filament_present);
    let msg = if present {
        format!("Filament Sensor {}: filament detected", helper.name)
    } else {
        format!("Filament Sensor {}: filament not detected", helper.name)
    };
    gcmd.respond_info(&msg);
    Ok(())
}

/// `SET_FILAMENT_SENSOR` (`filament_switch_sensor.py:101-102`).
fn cmd_set_filament_sensor(
    helper: &Arc<RunoutHelper>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let enabled = gcmd.get_int_default("ENABLE", 1)? != 0;
    *helper.lock(&helper.sensor_enabled) = enabled;
    Ok(())
}

/// One `[filament_switch_sensor <name>]` (upstream `SwitchSensor`,
/// `filament_switch_sensor.py:108-118`).
pub struct SwitchSensor {
    /// The shared helper the button handler feeds.
    runout_helper: Arc<RunoutHelper>,
}

impl SwitchSensor {
    /// Build the section: resolve the `buttons` module, read `switch_pin`,
    /// register the debounced button, then build the runout helper
    /// (`filament_switch_sensor.py:109-115`).
    ///
    /// # Errors
    /// A missing `switch_pin`, or any option the runout helper refuses.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let buttons = PrinterButtons::ensure(printer)?;
        let switch_pin = config.get("switch_pin", None)?;
        let runout_helper = Arc::new(RunoutHelper::new(config, printer)?);
        runout_helper.attach(printer)?;
        let handler = Arc::clone(&runout_helper);
        buttons.register_debounce_button(
            config,
            &switch_pin,
            Box::new(move |eventtime, state| {
                handler.note_filament_present(eventtime, state);
            }),
        )?;
        Ok(Self { runout_helper })
    }

    /// The shared runout helper.
    pub fn runout_helper(&self) -> &Arc<RunoutHelper> {
        &self.runout_helper
    }
}

impl PrinterObject for SwitchSensor {
    /// Upstream aliases this to the helper's
    /// (`filament_switch_sensor.py:116`).
    fn get_status(&self, eventtime: f64) -> Value {
        let present = *self
            .runout_helper
            .lock(&self.runout_helper.filament_present);
        let enabled = *self.runout_helper.lock(&self.runout_helper.sensor_enabled);
        let _ = eventtime;
        json!({ "filament_detected": present, "enabled": enabled })
    }
}

impl std::fmt::Debug for SwitchSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SwitchSensor")
            .field("runout_helper", &self.runout_helper)
            .finish()
    }
}

/// The factory the section declaration names
/// (`filament_switch_sensor.py:120 def load_config_prefix`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(SwitchSensor::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    fn config(sensor: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [filament_switch_sensor runout_switch]\n{sensor}"
        )
    }

    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        let result = printer.load_config(&config);
        (printer, result)
    }

    fn load_ok(text: &str) -> Arc<Printer> {
        let (printer, result) = load(text);
        result.expect("the config loads");
        printer
    }

    fn the_sensor(printer: &Arc<Printer>) -> Arc<SwitchSensor> {
        printer
            .lookup_object_as::<SwitchSensor>("filament_switch_sensor runout_switch")
            .expect("[filament_switch_sensor runout_switch] is registered")
    }

    /// The corpus section (`test/klippy/extruders.cfg:66-67`) reads with every
    /// option defaulted as upstream defaults them.
    #[test]
    fn test_the_corpus_switch_section_reads_every_option() {
        let printer = load_ok(&config("switch_pin = PD4\n"));
        let helper = the_sensor(&printer).runout_helper().clone();
        assert_eq!(helper.name(), "runout_switch");
        assert!(helper.pause_on_runout);
        assert_eq!(helper.pause_delay, 0.5);
        assert_eq!(helper.event_delay, 3.0);
        assert!(helper.runout_gcode.is_some());
        assert!(helper.insert_gcode.is_none());
    }

    /// `pause_on_runout`, `pause_delay` and `event_delay` take explicit values.
    #[test]
    fn test_the_helper_options_take_explicit_values() {
        let printer = load_ok(&config(
            "switch_pin: PD4\npause_on_runout: False\npause_delay: 1.5\nevent_delay: 0\n",
        ));
        let helper = the_sensor(&printer).runout_helper().clone();
        assert!(!helper.pause_on_runout);
        assert_eq!(helper.pause_delay, 1.5);
        assert_eq!(helper.event_delay, 0.0);
        // With `pause_on_runout` false and no `runout_gcode`, upstream never
        // reads the option and leaves the template unset.
        assert!(helper.runout_gcode.is_none());
    }

    /// The bounds upstream puts on the two delays are enforced with its wording.
    #[test]
    fn test_the_delay_bounds_are_enforced() {
        let (_, result) = load(&config("switch_pin: PD4\npause_delay: 0\n"));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must be above"), "{err}");

        let (_, result) = load(&config("switch_pin: PD4\nevent_delay: -1\n"));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must have minimum of"), "{err}");
    }

    /// A missing `switch_pin` is refused, as upstream's `config.get` does.
    #[test]
    fn test_a_missing_switch_pin_is_refused() {
        let (_, result) = load(&config(""));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("switch_pin"), "{err}");
    }

    /// The helper's status and the `QUERY`/`SET` commands follow upstream.
    #[test]
    fn test_status_and_the_query_set_commands() {
        let printer = load_ok(&config("switch_pin: PD4\n"));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        let status = PrinterObject::get_status(&*the_sensor(&printer), 0.0);
        assert_eq!(status["filament_detected"], false);
        assert_eq!(status["enabled"], true);

        gcode
            .run_script_sync("SET_FILAMENT_SENSOR SENSOR=runout_switch ENABLE=0")
            .expect("the command runs");
        let status = PrinterObject::get_status(&*the_sensor(&printer), 0.0);
        assert_eq!(status["enabled"], false);

        gcode
            .run_script_sync("QUERY_FILAMENT_SENSOR SENSOR=runout_switch")
            .expect("the command runs");
    }

    /// A state change inside the ready window is ignored; after it, a change
    /// is recorded and the event window closes.
    #[test]
    fn test_a_state_change_outside_the_ready_window_is_acted_on() {
        let printer = load_ok(&config("switch_pin: PD4\n"));
        let helper = the_sensor(&printer).runout_helper().clone();
        // Before ready, `min_event_systime` is NEVER, so nothing is acted on.
        helper.note_filament_present(0.0, true);
        assert!(*helper.lock(&helper.filament_present));

        // Force the window open, then a change clears it again.
        *helper.lock(&helper.min_event_systime) = 0.0;
        helper.note_filament_present(1.0, false);
        assert!(!*helper.lock(&helper.filament_present));
        assert_eq!(*helper.lock(&helper.min_event_systime), NEVER);
    }
}
