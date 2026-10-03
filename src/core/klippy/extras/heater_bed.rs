//! `[heater_bed]` — the heated bed.
//!
//! Upstream's `klippy/extras/heater_bed.py`: it is `setup_heater` under the
//! `B` g-code id, plus `M140`/`M190`. The heater itself (sensor, control loop,
//! PWM) lives in [`heaters`](crate::core::klippy::extras::heaters); this section
//! is the object the bed registers with the printer.
//!
//! `M190` waits for the target in upstream (`heaters.set_temperature(wait=True)`
//! through `TEMPERATURE_WAIT`). The wait loop lives in
//! [`PrinterHeaters::set_temperature`](crate::core::klippy::extras::heaters::PrinterHeaters::set_temperature),
//! and `M190` calls it with `wait=true`, while `M140` calls it with `wait=false`.

use std::sync::{Arc, Weak};

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{self, Heater, PrinterHeaters};
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
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
        for (name, wait) in [("M140", false), ("M190", true)] {
            let heater = Arc::clone(&self.heater);
            let printer_weak = Arc::downgrade(printer);
            let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                let heater = Arc::clone(&heater);
                let printer_weak = printer_weak.clone();
                Box::pin(async move {
                    let temp = gcmd.get_float_default("S", 0.0)?;
                    set_bed_temperature(&printer_weak, &heater, temp, wait).await
                })
            });
            gcode
                .register_command_with_params(
                    name,
                    handler,
                    Some("Set bed temperature"),
                    M140_M190_PARAMS,
                    false,
                )
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

/// `M140`/`M190` read one word, `S` (`heater_bed.py:20`), through the
/// inline handler both names share.
const M140_M190_PARAMS: &[&str] = &["S"];

/// `M140`/`M190`: set the bed target, waiting when asked
/// (`PrinterHeaterBed.cmd_M140` with `wait`, `heater_bed.py:18-25`).
///
/// Both names go through `PrinterHeaters::set_temperature`; `M190` passes
/// `wait=true` so the call blocks until the bed reaches the target.
async fn set_bed_temperature(
    printer: &Weak<Printer>,
    heater: &Arc<Heater>,
    temp: f64,
    wait: bool,
) -> Result<(), CommandError> {
    let pheaters = printer
        .upgrade()
        .and_then(|p| p.lookup_object_as::<PrinterHeaters>(heaters::HEATERS_OBJECT))
        .ok_or_else(|| CommandError::new("Unknown config object 'heaters'"))?;
    pheaters.set_temperature(heater, temp, wait).await
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
        // `M190` waits for the bed to reach the target.  Bring the smoothed
        // reading to 70 so `check_busy` is false and the wait returns at once.
        bed.heater().temperature_callback(1.0, 70.0);
        gcode.run_script_sync("M190 S70").unwrap();
        assert_eq!(bed.heater().get_status()["target"], 70.0);
    }

    /// `M190` with `wait=true` calls `PrinterHeaters::set_temperature(.., true)`,
    /// so it blocks until the bed reaches the target — unlike `M140`, which
    /// sets the target and returns.
    #[tokio::test(start_paused = true)]
    async fn test_m190_waits_for_the_target_temperature() {
        let printer = load_ok(BED);
        let gcode = printer
            .lookup_object_as::<crate::core::klippy::gcode::GCodeDispatch>("gcode")
            .unwrap();
        let bed = printer
            .lookup_object_as::<PrinterHeaterBed>("heater_bed")
            .unwrap();

        // The bed starts cold (smoothed_temp 0).  Feed simulated readings
        // while `M190` waits so the heater eventually reaches the target.
        let feed = Arc::clone(bed.heater());
        let feeder = tokio::spawn(async move {
            let mut temp = 20.0;
            let mut time = 0.0;
            loop {
                let power = feed.get_status()["power"].as_f64().unwrap();
                time += 0.1;
                temp += (power * 120.0 - (temp - 20.0) * 0.5) * 0.1;
                feed.temperature_callback(time, temp);
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            }
        });

        // `M140` returns at once: the target is set but the bed is still cold.
        gcode.run_script("M140 S60").await.unwrap();
        assert_eq!(bed.heater().get_status()["target"], 60.0);
        assert!(bed.heater().get_temp().0 < 58.0);

        // `M190` waits: the call does not return until the feeder has brought
        // the bed within `max_delta` of the target.
        gcode.run_script("M190 S60").await.unwrap();
        feeder.abort();

        assert_eq!(bed.heater().get_status()["target"], 60.0);
        // The wait loop ran: the bed heated to within `max_delta` (2 °C) of
        // the target.
        assert!(bed.heater().get_temp().0 >= 58.0);
    }
}
