//! `[filament_motion_sensor <name>]` — an encoder that watches extrusion motion
//! (upstream `klippy/extras/filament_motion_sensor.py`).
//!
//! [`EncoderSensor`] builds the shared [`RunoutHelper`] (the switch sensor's
//! module) and adds the encoder pin, the `extruder` it watches, and
//! `detection_length`. At `klippy:ready` it resolves that extruder and this
//! MCU's clock, as upstream's `_handle_ready` does.
//!
//! # What is not here
//!
//! * The periodic runout check (`_extruder_pos_update_event`, upstream's
//!   `CHECK_RUNOUT_TIMEOUT = .250` timer): the `idle_timeout:printing` event
//!   would point that timer at "now", but the timer itself and the position
//!   comparison against `filament_runout_pos` are not wired. On the corpus's
//!   fake firmware no encoder event is generated either, so the runout check is
//!   never reached.
//! * A bound `[extruder_stepper]`'s past position: the host stepper it wraps is
//!   owned by the toolhead after connect and exposes no position, so
//!   [`ExtruderSource::find_past_position`] returns `0.0` for that case — the
//!   same H10 motion-sync gap the extra stepper itself carries.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::buttons::PrinterButtons;
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::extruder_stepper::PrinterExtruderStepper;
use crate::core::klippy::extras::filament_switch_sensor::RunoutHelper;
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuObject;
use crate::core::klippy::motion::extra::ExtraAxis;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form exists upstream (`filament_motion_sensor.py:76`).
section!(
    "filament_motion_sensor",
    order = 30,
    prefix = load_config_prefix
);

/// The name of the MCU object whose clock dates a position lookup
/// (`filament_motion_sensor.py:41`).
const MCU_OBJECT: &str = "mcu";

/// The extruder a motion sensor watches, once resolved by name
/// (`printer.lookup_object(self.extruder_name)`, `filament_motion_sensor.py:40`).
enum ExtruderSource {
    /// A `[extruder]` (upstream's `PrinterExtruder`, an `ExtraAxis`).
    Extruder(Arc<PrinterExtruder>),
    /// A `[extruder_stepper <name>]` (upstream's `PrinterExtruderStepper`).
    ExtruderStepper(Arc<PrinterExtruderStepper>),
}

impl ExtruderSource {
    /// The position at a past print time (`find_past_position`). The extra
    /// stepper has no reachable host position after connect (see the module
    /// docs), so it reports `0.0`.
    fn find_past_position(&self, print_time: f64) -> f64 {
        match self {
            Self::Extruder(extruder) => extruder.find_past_position(print_time),
            Self::ExtruderStepper(stepper) => stepper.find_past_position(print_time),
        }
    }

    /// Which kind of extruder resolved, for diagnostics and tests.
    fn kind(&self) -> &'static str {
        match self {
            Self::Extruder(_) => "extruder",
            Self::ExtruderStepper(_) => "extruder_stepper",
        }
    }
}

/// One `[filament_motion_sensor <name>]` (upstream `EncoderSensor`,
/// `filament_motion_sensor.py:11-75`).
pub struct EncoderSensor {
    /// The section suffix (`runout_encoder`), upstream's object name.
    name: String,
    /// The shared runout helper the encoder event feeds.
    runout_helper: Arc<RunoutHelper>,
    /// The `extruder` option: a section identifier that may contain a space
    /// (`extruder_stepper my_extra_stepper`).
    extruder_name: String,
    /// `detection_length` (default `7.`, `above=0`).
    detection_length: f64,
    /// The machine, to resolve the extruder and its clock at ready.
    printer: Weak<Printer>,
    /// The resolved extruder, once ready.
    extruder: Mutex<Option<ExtruderSource>>,
    /// The MCU object whose clock dates a lookup, once ready.
    mcu: Mutex<Option<Arc<McuObject>>>,
    /// The extrusion position past which the filament is assumed to have run
    /// out (`filament_runout_pos`).
    filament_runout_pos: Mutex<Option<f64>>,
    /// Whether the print is running, from the `idle_timeout` events.
    printing: AtomicBool,
}

impl EncoderSensor {
    /// Build the section: read its options, register the encoder pin, build the
    /// runout helper (`filament_motion_sensor.py:12-37`).
    ///
    /// # Errors
    /// A missing `switch_pin` / `extruder`, an invalid `detection_length`, or
    /// any option the runout helper refuses.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let buttons = PrinterButtons::ensure(printer)?;
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        let switch_pin = config.get("switch_pin", None)?;
        let extruder_name = config.get("extruder", None)?;
        let detection_length =
            config.get_float_bounded("detection_length", Some(7.0), None, None, Some(0.0), None)?;

        let runout_helper = Arc::new(RunoutHelper::new(config, printer)?);
        runout_helper.attach(printer)?;
        let handler = Arc::clone(&runout_helper);
        buttons.register_buttons(
            vec![switch_pin],
            Box::new(move |eventtime, state| {
                // The encoder only reports a presence change; the runout check
                // that follows is not wired (module docs).
                if state {
                    handler.note_filament_present(eventtime, true);
                }
            }),
        );

        Ok(Self {
            name,
            runout_helper,
            extruder_name,
            detection_length,
            printer: Arc::downgrade(printer),
            extruder: Mutex::new(None),
            mcu: Mutex::new(None),
            filament_runout_pos: Mutex::new(None),
            printing: AtomicBool::new(false),
        })
    }

    /// The section suffix (`runout_encoder`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The `extruder` option as written.
    pub fn extruder_name(&self) -> &str {
        &self.extruder_name
    }

    /// `detection_length`.
    pub fn detection_length(&self) -> f64 {
        self.detection_length
    }

    /// The shared runout helper.
    pub fn runout_helper(&self) -> &Arc<RunoutHelper> {
        &self.runout_helper
    }

    /// Which kind of extruder resolved (`extruder` / `extruder_stepper`), or
    /// `None` before ready.
    pub fn resolved_extruder_kind(&self) -> Option<&'static str> {
        self.lock(&self.extruder).as_ref().map(ExtruderSource::kind)
    }

    /// Register the `klippy:ready` and `idle_timeout` handlers
    /// (`filament_motion_sensor.py:33-37`, `:39-43`).
    ///
    /// Called after the `Arc` exists.
    pub fn attach(self: &Arc<Self>, printer: &Arc<Printer>) {
        let weak = Arc::downgrade(self);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                if let Some(sensor) = weak.upgrade() {
                    sensor.handle_ready();
                }
            }),
        );
        let weak = Arc::downgrade(self);
        printer.register_event_handler(
            KlippyEvent::IdleTimeoutPrinting,
            Box::new(move |_| {
                if let Some(sensor) = weak.upgrade() {
                    sensor.printing.store(true, Ordering::SeqCst);
                }
            }),
        );
        for event in [KlippyEvent::IdleTimeoutReady, KlippyEvent::IdleTimeoutIdle] {
            let weak = Arc::downgrade(self);
            printer.register_event_handler(
                event,
                Box::new(move |_| {
                    if let Some(sensor) = weak.upgrade() {
                        sensor.printing.store(false, Ordering::SeqCst);
                    }
                }),
            );
        }
    }

    /// Upstream's `_handle_ready` (`filament_motion_sensor.py:39-45`): resolve
    /// the watched extruder and the MCU clock, then seed the runout position.
    ///
    /// A name that does not resolve is left unset (upstream raises); ready
    /// handlers here cannot fail the machine, so the sensor stays dormant.
    fn handle_ready(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let source = if let Some(extruder) =
            printer.lookup_object_as::<PrinterExtruder>(&self.extruder_name)
        {
            Some(ExtruderSource::Extruder(extruder))
        } else {
            printer
                .lookup_object_as::<PrinterExtruderStepper>(&self.extruder_name)
                .map(ExtruderSource::ExtruderStepper)
        };
        let Some(source) = source else {
            tracing::warn!(
                sensor = self.name.as_str(),
                extruder = self.extruder_name.as_str(),
                "filament motion sensor: the extruder did not resolve"
            );
            return;
        };
        *self.lock(&self.extruder) = Some(source);
        if let Some(mcu) = printer.lookup_object_as::<McuObject>(MCU_OBJECT) {
            *self.lock(&self.mcu) = Some(mcu);
        }
        self.update_filament_runout_pos(None);
    }

    /// Upstream's `_update_filament_runout_pos` (`filament_motion_sensor.py:31-36`).
    fn update_filament_runout_pos(&self, eventtime: Option<f64>) {
        let pos = self.extruder_pos(eventtime);
        *self.lock(&self.filament_runout_pos) = Some(pos + self.detection_length);
    }

    /// Upstream's `_get_extruder_pos` (`filament_motion_sensor.py:55-58`): the
    /// position at the print time the MCU clock implies.
    fn extruder_pos(&self, eventtime: Option<f64>) -> f64 {
        let now = eventtime.unwrap_or_else(|| self.now());
        let print_time = self
            .lock(&self.mcu)
            .as_ref()
            .and_then(|mcu| mcu.estimated_print_time(now))
            .unwrap_or(now);
        match self.lock(&self.extruder).as_ref() {
            Some(source) => source.find_past_position(print_time),
            None => 0.0,
        }
    }

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

impl PrinterObject for EncoderSensor {
    /// Upstream aliases this to the helper's
    /// (`filament_motion_sensor.py:25`).
    fn get_status(&self, eventtime: f64) -> Value {
        let present = self.runout_helper.filament_present();
        let enabled = self.runout_helper.sensor_enabled();
        let _ = eventtime;
        json!({ "filament_detected": present, "enabled": enabled })
    }
}

impl std::fmt::Debug for EncoderSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EncoderSensor")
            .field("name", &self.name)
            .field("extruder_name", &self.extruder_name)
            .field("detection_length", &self.detection_length)
            .finish_non_exhaustive()
    }
}

/// The factory the section declaration names
/// (`filament_motion_sensor.py:76 def load_config_prefix`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let sensor = Arc::new(EncoderSensor::new(config, printer)?);
    sensor.attach(printer);
    Ok(sensor as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::extras::buttons::BUTTONS_OBJECT;
    use crate::core::klippy::extras::gcode_macro::GCODE_MACRO_OBJECT;
    use crate::core::klippy::extras::pause_resume::PAUSE_RESUME_OBJECT;
    use crate::core::klippy::reactor::ManualReactor;

    /// A printer with an `[extruder]`, an `[extruder_stepper my_extra_stepper]`
    /// and a motion sensor watching `extruder_stepper my_extra_stepper` — the
    /// corpus's shape with the sensor named for clarity.
    const CONFIG: &str = "[mcu]\nserial: /dev/not-opened-yet\n\
         [extruder]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 33.5\nmicrosteps: 16\n\
         nozzle_diameter: 0.4\nfilament_diameter: 1.75\nheater_pin: PB0\n\
         sensor_type: temperature_mcu\ncontrol: pid\npid_Kp: 1\npid_Ki: 0.1\npid_Kd: 10\n\
         min_temp: 0\nmax_temp: 250\n\
         [extruder_stepper my_extra_stepper]\nextruder: extruder\n\
         step_pin: PH5\ndir_pin: PH6\nmicrosteps: 16\nrotation_distance: 28.2\n\
         [filament_motion_sensor runout_encoder1]\nswitch_pin = PL6\n\
         detection_length = 4\nextruder = extruder_stepper my_extra_stepper\n";

    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        let result = printer.load_config(&config);
        (printer, result)
    }

    fn the_sensor(printer: &Arc<Printer>) -> Arc<EncoderSensor> {
        printer
            .lookup_object_as::<EncoderSensor>("filament_motion_sensor runout_encoder1")
            .expect("[filament_motion_sensor runout_encoder1] is registered")
    }

    /// The corpus section reads every option, and the dependencies it names are
    /// created even though the config has no section for them.
    #[test]
    fn test_the_corpus_motion_section_reads_every_option() {
        let (printer, result) = load(CONFIG);
        result.expect("the config loads");
        let sensor = the_sensor(&printer);
        assert_eq!(sensor.name(), "runout_encoder1");
        assert_eq!(sensor.extruder_name(), "extruder_stepper my_extra_stepper");
        assert_eq!(sensor.detection_length(), 4.0);
        assert!(printer.lookup_object(BUTTONS_OBJECT).is_some());
        assert!(printer.lookup_object(GCODE_MACRO_OBJECT).is_some());
        assert!(printer.lookup_object(PAUSE_RESUME_OBJECT).is_some());
    }

    /// `detection_length` defaults to `7.` and is bounded `above=0`.
    #[test]
    fn test_detection_length_default_and_bound() {
        let (printer, result) = load(&CONFIG.replace("detection_length = 4\n", ""));
        result.expect("the config loads");
        assert_eq!(the_sensor(&printer).detection_length(), 7.0);

        let (_, result) = load(&CONFIG.replace("detection_length = 4", "detection_length = 0"));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must be above"), "{err}");
    }

    /// At ready the `extruder` option resolves **by section identifier** — the
    /// value contains a space, so it only resolves if the whole identifier is
    /// looked up (`printer.lookup_object("extruder_stepper my_extra_stepper")`).
    #[test]
    fn test_ready_resolves_the_extruder_by_its_section_identifier() {
        let (printer, result) = load(CONFIG);
        result.expect("the config loads");
        let sensor = the_sensor(&printer);
        assert_eq!(sensor.resolved_extruder_kind(), None);
        printer.send_event(&KlippyEvent::KlippyReady);
        assert_eq!(sensor.resolved_extruder_kind(), Some("extruder_stepper"));
    }

    /// A sensor watching the primary `[extruder]` resolves to it.
    #[test]
    fn test_ready_resolves_the_primary_extruder() {
        let (printer, result) = load(&CONFIG.replace(
            "extruder = extruder_stepper my_extra_stepper",
            "extruder = extruder",
        ));
        result.expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        assert_eq!(
            the_sensor(&printer).resolved_extruder_kind(),
            Some("extruder")
        );
    }

    /// A missing `extruder` option is refused, as upstream's `config.get` does.
    #[test]
    fn test_a_missing_extruder_is_refused() {
        let (_, result) =
            load(&CONFIG.replace("extruder = extruder_stepper my_extra_stepper\n", ""));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("extruder"), "{err}");
    }

    /// The `idle_timeout` events track the printing state.
    #[test]
    fn test_the_idle_timeout_events_track_printing() {
        let (printer, result) = load(CONFIG);
        result.expect("the config loads");
        let sensor = the_sensor(&printer);
        printer.send_event(&KlippyEvent::IdleTimeoutPrinting);
        assert!(sensor.printing.load(Ordering::SeqCst));
        printer.send_event(&KlippyEvent::IdleTimeoutIdle);
        assert!(!sensor.printing.load(Ordering::SeqCst));
    }
}
