//! `[heater_bed]` — the heated bed.
//!
//! Upstream's `klippy/extras/heater_bed.py`: it is `setup_heater` under the
//! `B` g-code id, plus `M140`/`M190`. The heater itself (sensor, control loop,
//! PWM) lives in [`heaters`](crate::core::klippy::extras::heaters); this section
//! is the object the bed registers with the printer.
//!
//! `M190` waits for the target in upstream (`heaters.set_temperature(wait=True)`
//! through `TEMPERATURE_WAIT`); the wait loop is not wired here yet, so `M190`
//! currently sets the target and returns, like `M140`.

use std::sync::Arc;

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{self, Heater};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("heater_bed", order = 20, load = load_config);

/// One `[heater_bed]`.
pub struct PrinterHeaterBed {
    heater: Arc<Heater>,
}

impl PrinterHeaterBed {
    /// Build the bed: set up its heater and register `M140`/`M190`.
    ///
    /// # Errors
    /// A missing or invalid heater option, or an unknown sensor.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let heater = heaters::ensure(printer)?.setup_heater(config, printer, Some("B"))?;
        let object = Self { heater };
        object.register_commands(printer)?;
        Ok(object)
    }

    /// The bed's heater.
    pub fn heater(&self) -> &Arc<Heater> {
        &self.heater
    }

    fn register_commands(&self, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        for (name, _wait) in [("M140", false), ("M190", true)] {
            let heater = Arc::clone(&self.heater);
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                let temp = gcmd.get_float_default("S", 0.0)?;
                set_bed_temperature(&heater, temp)
            });
            gcode
                .register_command(name, handler, Some("Set bed temperature"), false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }
}

impl PrinterObject for PrinterHeaterBed {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.heater.get_status()
    }
}

impl std::fmt::Debug for PrinterHeaterBed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterHeaterBed").finish_non_exhaustive()
    }
}

/// `M140`/`M190`: set the bed target (`PrinterHeaterBed.cmd_M140`).
fn set_bed_temperature(heater: &Heater, temp: f64) -> Result<(), CommandError> {
    heater.set_temp(temp)
}

/// The factory `section!` names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterHeaterBed::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    fn load_ok(text: &str) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer.load_config(&config).expect("the config loads");
        // `M140`/`M190` are ready-only commands.
        printer.send_event(&crate::core::klippy::event::KlippyEvent::KlippyReady);
        printer
    }

    const BED: &str = "[mcu]\nserial: /dev/not-opened-yet\n\
                       [heater_bed]\nheater_pin: PB1\nsensor_type: EPCOS 100K B57560G104F\n\
                       sensor_pin: PK6\ncontrol: watermark\nmin_temp: 0\nmax_temp: 130\n";

    #[test]
    fn test_a_heater_bed_loads_and_registers_m140() {
        let printer = load_ok(BED);

        let bed = printer
            .lookup_object_as::<PrinterHeaterBed>("heater_bed")
            .expect("the bed is registered");
        assert!(bed.heater().get_status()["temperature"].is_number());
        assert_eq!(
            printer
                .lookup_object_as::<crate::core::klippy::extras::heaters::PrinterHeaters>("heaters")
                .unwrap()
                .get_status(0.0)["available_heaters"],
            json!(["heater_bed"])
        );
    }

    #[test]
    fn test_m140_sets_the_target_and_m190_too() {
        let printer = load_ok(BED);
        let gcode = printer
            .lookup_object_as::<crate::core::klippy::gcode::GCodeDispatch>("gcode")
            .unwrap();
        let bed = printer
            .lookup_object_as::<PrinterHeaterBed>("heater_bed")
            .unwrap();

        gcode.run_script_sync("M140 S60").unwrap();
        assert_eq!(bed.heater().get_status()["target"], 60.0);
        gcode.run_script_sync("M190 S70").unwrap();
        assert_eq!(bed.heater().get_status()["target"], 70.0);
    }
}
