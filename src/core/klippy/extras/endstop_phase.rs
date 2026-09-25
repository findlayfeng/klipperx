//! `[endstop_phase]` — endstop accuracy improvement via stepper phase tracking.
//!
//! Upstream `klippy/extras/endstop_phase.py`. A stepper's rotor settles at a
//! phase that depends on how far it turned before the endstop fired; with a
//! fixed number of microsteps per rotation that phase maps to a fraction of a
//! full step of trigger error. `[endstop_phase <stepper>]` records the phase a
//! stepper triggered at and moves the axis' post-home position to a chosen
//! phase, and `[endstop_phase]` (the bare section) supports
//! `ENDSTOP_PHASE_CALIBRATE`, which measures the phase distribution over
//! several homes and writes a `trigger_phase` back to config.
//!
//! | upstream | here |
//! |---|---|
//! | `PhaseCalc` | [`PhaseCalc`] |
//! | `EndstopPhase` (`[endstop_phase <stepper>]`) | the prefix section |
//! | `EndstopPhases` (`[endstop_phase]`) | [`EndstopPhases`] |
//! | `ENDSTOP_PHASE_CALIBRATE` | [`EndstopPhases::cmd_endstop_phase_calibrate`] |
//!
//! # Gaps
//!
//! The Traminic drivers (`tmc2130` … `tmc5160`) are not implemented in this
//! host, so `PhaseCalc.lookup_tmc` never finds a `<driver> <stepper>` object and
//! the MCU phase offset stays zero (`PhaseCalc.calc_phase`); `phases` comes from
//! the section's own `microsteps` instead. This does not stop the bare section
//! working, and upstream runs without a Traminic driver the same way.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::object::CONFIGFILE_OBJECT;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::motion::HomingHandle;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("endstop_phase", order = 30, load = load_config);

/// The Traminic drivers whose `get_phase_offset()` a `PhaseCalc` looks for
/// (`endstop_phase.py:9-10`).
const TRINAMIC_DRIVERS: [&str; 6] = [
    "tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160",
];

/// One stepper's trigger-phase tracking (upstream `PhaseCalc`).
struct PhaseCalc {
    /// The stepper's name.
    name: String,
    /// How many phases a rotation divides into (`phases`): `microsteps × 4`,
    /// or what a Traminic driver reports. `None` until one of those says so.
    phases: Option<usize>,
    /// How often each phase was seen (`phase_history`), `None` until `phases`
    /// is known.
    phase_history: Option<Vec<u64>>,
    /// The newest phase (`last_phase`).
    last_phase: Option<usize>,
    /// The newest trigger MCU position (`last_mcu_position`).
    last_mcu_position: Option<f64>,
    /// Whether this stepper is the primary of its rail (`is_primary`), the one
    /// whose phase `ENDSTOP_PHASE_CALIBRATE` may write back.
    is_primary: bool,
    /// Whether the tracker only counts phases and never adjusts an axis
    /// (`stats_only`) — a stepper with no `[endstop_phase <stepper>]` section.
    stats_only: bool,
}

impl PhaseCalc {
    /// A tracker for `name`, with `phases` known when the section supplies it.
    fn new(name: &str, phases: Option<usize>) -> Self {
        Self {
            name: name.to_string(),
            phases,
            phase_history: phases.map(|phases| vec![0; phases]),
            last_phase: None,
            last_mcu_position: None,
            is_primary: false,
            stats_only: false,
        }
    }

    /// Resolve the stepper's Traminic driver (`PhaseCalc.lookup_tmc`).
    ///
    /// None of [`TRINAMIC_DRIVERS`] is implemented here, so this only records
    /// that `phases` has to come from config; the search stays so a driver that
    /// lands later only has to register under its upstream name.
    fn lookup_tmc(&self, printer: &Printer) {
        for driver in TRINAMIC_DRIVERS {
            let driver_name = format!("{driver} {}", self.name);
            if printer.lookup_object(&driver_name).is_some() {
                // A driver here would answer `get_phase_offset()`; until one
                // exists the phase offset stays zero.
                break;
            }
        }
    }

    /// Record the phase a trigger position falls on
    /// (`PhaseCalc.calc_phase`).
    ///
    /// The MCU phase offset is zero — no Traminic driver in this host supplies
    /// one — so the phase is the trigger position modulo `phases`.
    fn calc_phase(&mut self, trig_mcu_pos: f64) -> usize {
        let phases = self
            .phases
            .expect("calc_phase is only called with phases known");
        let phase = (trig_mcu_pos % phases as f64) as usize;
        if let Some(history) = self.phase_history.as_mut() {
            history[phase] += 1;
        }
        self.last_phase = Some(phase);
        self.last_mcu_position = Some(trig_mcu_pos);
        phase
    }
}

/// The bare `[endstop_phase]` section (`EndstopPhases`).
///
/// It owns the `ENDSTOP_PHASE_CALIBRATE` command and the per-stepper phase
/// history. A stepper with an `[endstop_phase <stepper>]` section shares that
/// section's [`PhaseCalc`]; one without gets a `stats_only` tracker that only
/// counts phases.
pub struct EndstopPhases {
    /// The machine, to find `<driver> <stepper>` objects and the `configfile`.
    printer: Weak<Printer>,
    /// The G-code dispatcher, for `ENDSTOP_PHASE_CALIBRATE`.
    gcode: Arc<GCodeDispatch>,
    /// The per-stepper trackers (`EndstopPhases.tracking`).
    tracking: Mutex<HashMap<String, Arc<Mutex<PhaseCalc>>>>,
}

impl EndstopPhases {
    /// The bare section (`[endstop_phase]`).
    pub fn new(printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        Ok(Self {
            printer: Arc::downgrade(printer),
            gcode,
            tracking: Mutex::new(HashMap::new()),
        })
    }

    /// Register the command and the homing handler.
    ///
    /// # Errors
    /// When `ENDSTOP_PHASE_CALIBRATE` is already registered.
    pub fn register(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let handler: CommandHandler = Arc::new({
            let object = Arc::clone(self);
            move |gcmd: &GcodeCommand| {
                let object = Arc::clone(&object);
                Box::pin(async move { object.cmd_endstop_phase_calibrate(gcmd) })
            }
        });
        self.gcode
            .register_command(
                "ENDSTOP_PHASE_CALIBRATE",
                handler,
                Some(Self::CMD_ENDSTOP_PHASE_CALIBRATE_HELP),
                false,
            )
            .map_err(ConfigError::new)?;
        printer.register_event_handler(
            KlippyEvent::HomingHomeRailsEnd {
                axes: Vec::new(),
                homing: HomingHandle::new(),
            },
            Box::new({
                let object = Arc::clone(self);
                move |event| {
                    if let KlippyEvent::HomingHomeRailsEnd { homing, .. } = event {
                        object.handle_home_rails_end(homing);
                    }
                }
            }),
        );
        Ok(())
    }

    /// The task the command reports (`ENDSTOP_PHASE_CALIBRATE` help).
    const CMD_ENDSTOP_PHASE_CALIBRATE_HELP: &'static str = "Calibrate stepper phase";

    /// Note a stepper's trigger phase after a home
    /// (`EndstopPhases.update_stepper`).
    fn update_stepper(&self, stepper_name: &str, trig_mcu_pos: f64, is_primary: bool) {
        let phase_calc = {
            let mut tracking = self
                .tracking
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            match tracking.get(stepper_name) {
                Some(phase_calc) => Arc::clone(phase_calc),
                None => {
                    // A stepper with no `[endstop_phase <stepper>]` section
                    // gets a tracker that only counts phases.
                    let mut phase_calc = PhaseCalc::new(stepper_name, None);
                    phase_calc.stats_only = true;
                    if let Some(printer) = self.printer.upgrade() {
                        phase_calc.lookup_tmc(&printer);
                    }
                    let phase_calc = Arc::new(Mutex::new(phase_calc));
                    tracking.insert(stepper_name.to_string(), Arc::clone(&phase_calc));
                    phase_calc
                }
            }
        };
        let mut phase_calc = phase_calc
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        // No phases means no history to record into (`PhaseCalc` stays idle
        // until a driver or the section's config supplies them).
        if phase_calc.phase_history.is_none() {
            return;
        }
        if is_primary {
            phase_calc.is_primary = true;
        }
        if phase_calc.stats_only {
            phase_calc.calc_phase(trig_mcu_pos);
        }
    }

    /// The `homing:home_rails_end` handler: note every stepper that homed
    /// (`EndstopPhases.handle_home_rails_end`).
    ///
    /// Upstream walks the rails it is given; the port's payload carries the
    /// run's [`HomingHandle`] instead, which records each stepper's trigger
    /// position and primary flag as the rails home.
    fn handle_home_rails_end(&self, homing: &HomingHandle) {
        let state = homing.lock();
        let steppers: Vec<(String, f64, bool)> = state
            .trigger_positions()
            .map(|(name, position)| (name.to_string(), position, state.is_primary(name)))
            .collect();
        drop(state);
        for (name, position, is_primary) in steppers {
            self.update_stepper(&name, position, is_primary);
        }
    }

    /// `ENDSTOP_PHASE_CALIBRATE [STEPPER=<name>]`.
    fn cmd_endstop_phase_calibrate(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let stepper_name = gcmd.get_str_default("STEPPER", "");
        if stepper_name.is_empty() {
            self.report_stats();
            return Ok(());
        }
        let phase_calc = self
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .get(&stepper_name)
            .filter(|phase_calc| {
                phase_calc
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .phase_history
                    .is_some()
            })
            .cloned();
        let Some(phase_calc) = phase_calc else {
            return Err(CommandError::new(format!(
                "Stats not available for stepper {stepper_name}"
            )));
        };
        let (endstop_phase, phases) = self.generate_stats(&stepper_name, &phase_calc);
        if !phase_calc
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .is_primary
        {
            return Ok(());
        }
        if let Some(printer) = self.printer.upgrade() {
            let section = format!("endstop_phase {stepper_name}");
            if let Some(configfile) = printer
                .lookup_object_as::<crate::core::klippy::config::PrinterConfig>(CONFIGFILE_OBJECT)
            {
                configfile.remove_section(&section);
                configfile.set(
                    &section,
                    "trigger_phase",
                    &format!("{endstop_phase}/{phases}"),
                );
            }
        }
        self.gcode.respond_info(
            "The SAVE_CONFIG command will update the printer config\n\
             file with these parameters and restart the printer.",
            true,
        );
        Ok(())
    }

    /// Measure a stepper's best trigger phase from its history
    /// (`EndstopPhases.generate_stats`), reporting the range it saw.
    ///
    /// The phases wrap, so the history is doubled and each candidate phase
    /// scores the sum of the circular distances of the samples to it; the
    /// cheapest wins.
    fn generate_stats(
        &self,
        stepper_name: &str,
        phase_calc: &Arc<Mutex<PhaseCalc>>,
    ) -> (usize, usize) {
        let phase_calc = phase_calc
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let history = phase_calc
            .phase_history
            .as_ref()
            .expect("stats are only measured with phases known");
        let phases = history.len();
        let half_phases = phases / 2;
        let mut wrapped = history.clone();
        wrapped.extend_from_slice(history);
        let mut costs: Vec<(u64, usize)> = (0..phases)
            .map(|index| {
                let phase = index + half_phases;
                let cost: u64 = (index..index + phases)
                    .map(|sample| wrapped[sample] * (sample as i64 - phase as i64).unsigned_abs())
                    .sum();
                (cost, phase)
            })
            .collect();
        costs.sort_unstable();
        let best = costs[0].1;
        let found: Vec<usize> = (best - half_phases..best + half_phases)
            .filter(|&sample| wrapped[sample] > 0)
            .collect();
        let best_phase = best % phases;
        let (lo, hi) = match (found.first(), found.last()) {
            (Some(first), Some(last)) => (first % phases, last % phases),
            _ => (best_phase, best_phase),
        };
        self.gcode.respond_info(
            &format!("{stepper_name}: trigger_phase={best_phase}/{phases} (range {lo} to {hi})"),
            true,
        );
        (best_phase, phases)
    }

    /// Report every primary stepper's trigger phase
    /// (`EndstopPhases.report_stats`).
    fn report_stats(&self) {
        let tracking = self
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if tracking.is_empty() {
            self.gcode
                .respond_info("No steppers found. (Be sure to home at least once.)", true);
            return;
        }
        let mut names: Vec<String> = tracking.keys().cloned().collect();
        names.sort();
        for name in names {
            let phase_calc = Arc::clone(&tracking[&name]);
            if !phase_calc
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .is_primary
            {
                continue;
            }
            self.generate_stats(&name, &phase_calc);
        }
    }
}

impl PrinterObject for EndstopPhases {
    /// The `last_home` map upstream reports (`EndstopPhases.get_status`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let tracking = self
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mut last_home = serde_json::Map::new();
        for (name, phase_calc) in tracking.iter() {
            let phase_calc = phase_calc
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            if phase_calc.phase_history.is_none() {
                continue;
            }
            last_home.insert(
                name.clone(),
                json!({
                    "phase": phase_calc.last_phase,
                    "phases": phase_calc.phases,
                    "mcu_position": phase_calc.last_mcu_position,
                }),
            );
        }
        json!({ "last_home": Value::Object(last_home) })
    }
}

/// The bare `[endstop_phase]` factory.
pub fn load_config(
    _config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(EndstopPhases::new(printer)?);
    object.register(printer)?;
    Ok(object)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, PrinterConfig};
    use crate::core::klippy::reactor::ManualReactor;

    /// A printer with a `gcode` dispatcher and a `configfile`, plus a sink for
    /// the lines the dispatcher emits.
    fn machine() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<Mutex<Vec<String>>>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let gcode = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
        printer
            .add_object(GCODE_OBJECT, Arc::clone(&gcode) as Arc<dyn PrinterObject>)
            .unwrap();
        printer
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::new(PrinterConfig::new(
                    AccessTracking::shared(),
                    serde_json::Map::new(),
                )),
            )
            .unwrap();
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push(line.to_string());
        }));
        (printer, gcode, lines)
    }

    fn emitted(lines: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        lines
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// Build a `PhaseCalc` tracker with `counts` as its phase history.
    fn tracker(counts: &[u64], is_primary: bool) -> Arc<Mutex<PhaseCalc>> {
        Arc::new(Mutex::new(PhaseCalc {
            name: "stepper_x".to_string(),
            phases: Some(counts.len()),
            phase_history: Some(counts.to_vec()),
            last_phase: None,
            last_mcu_position: None,
            is_primary,
            stats_only: true,
        }))
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// The bare `[endstop_phase]` section loads with zero options and registers
    /// `ENDSTOP_PHASE_CALIBRATE` with upstream's help.
    #[test]
    fn test_the_bare_section_loads_and_registers_the_command() {
        let (printer, gcode, _lines) = machine();
        let section = ConfigSection::new("endstop_phase", None);

        load_config(&wrap(&section), &printer).unwrap();
        // Only the active table is reported, and a ready-only command enters
        // it when the printer becomes ready.
        printer.send_event(&KlippyEvent::KlippyReady);

        let status = gcode.get_status(0.0);
        assert_eq!(
            status["commands"]["ENDSTOP_PHASE_CALIBRATE"]["help"],
            json!("Calibrate stepper phase")
        );
    }

    /// With nothing tracked, the command reports upstream's message.
    #[test]
    fn test_endstop_phase_calibrate_without_a_home_reports_no_steppers() {
        let (printer, gcode, lines) = machine();
        let object = Arc::new(EndstopPhases::new(&printer).unwrap());
        object.register(&printer).unwrap();
        let gcmd = gcode.create_gcode_command(
            "ENDSTOP_PHASE_CALIBRATE",
            "ENDSTOP_PHASE_CALIBRATE",
            HashMap::new(),
        );

        object.cmd_endstop_phase_calibrate(&gcmd).unwrap();

        assert!(
            emitted(&lines)
                .iter()
                .any(|line| line.contains("No steppers found. (Be sure to home at least once.)")),
            "{:?}",
            emitted(&lines)
        );
    }

    /// An untracked stepper name is reported verbatim.
    #[test]
    fn test_endstop_phase_calibrate_names_an_untracked_stepper() {
        let (printer, gcode, _lines) = machine();
        let object = Arc::new(EndstopPhases::new(&printer).unwrap());
        object.register(&printer).unwrap();
        let gcmd = gcode.create_gcode_command(
            "ENDSTOP_PHASE_CALIBRATE",
            "ENDSTOP_PHASE_CALIBRATE STEPPER=nope",
            HashMap::from([("STEPPER".to_string(), "nope".to_string())]),
        );

        let err = object.cmd_endstop_phase_calibrate(&gcmd).unwrap_err();

        assert_eq!(err.message(), "Stats not available for stepper nope");
    }

    /// A single sample at phase 0 of 8 picks phase 0 and reports the range it
    /// saw (upstream's circular phase histogram).
    #[test]
    fn test_generate_stats_picks_the_best_phase_and_reports_the_range() {
        let (printer, _gcode, lines) = machine();
        let object = EndstopPhases::new(&printer).unwrap();
        let phase_calc = tracker(&[1, 0, 0, 0, 0, 0, 0, 0], true);

        let (best_phase, phases) = object.generate_stats("stepper_x", &phase_calc);

        assert_eq!((best_phase, phases), (0, 8));
        assert!(
            emitted(&lines)
                .iter()
                .any(|line| line.contains("stepper_x: trigger_phase=0/8 (range 0 to 0)")),
            "{:?}",
            emitted(&lines)
        );
    }

    /// A primary stepper writes `trigger_phase` for `SAVE_CONFIG`; a
    /// non-primary one measures but writes nothing (`endstop_phase.py:171-190`).
    #[test]
    fn test_a_non_primary_stepper_does_not_write_config() {
        let (printer, gcode, lines) = machine();
        let object = Arc::new(EndstopPhases::new(&printer).unwrap());
        object.register(&printer).unwrap();
        object
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert(
                "stepper_x".to_string(),
                tracker(&[1, 0, 0, 0, 0, 0, 0, 0], false),
            );
        let gcmd = gcode.create_gcode_command(
            "ENDSTOP_PHASE_CALIBRATE",
            "ENDSTOP_PHASE_CALIBRATE STEPPER=stepper_x",
            HashMap::from([("STEPPER".to_string(), "stepper_x".to_string())]),
        );

        object.cmd_endstop_phase_calibrate(&gcmd).unwrap();

        assert!(
            !emitted(&lines)
                .iter()
                .any(|line| line.contains("SAVE_CONFIG")),
            "non-primary must not ask for a write: {:?}",
            emitted(&lines)
        );
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .unwrap();
        assert_eq!(
            configfile.get_status(0.0)["save_config_pending"],
            json!(false)
        );
    }

    /// A primary stepper asks `SAVE_CONFIG` to write `trigger_phase` back.
    #[test]
    fn test_a_primary_stepper_writes_trigger_phase_for_save_config() {
        let (printer, gcode, lines) = machine();
        let object = Arc::new(EndstopPhases::new(&printer).unwrap());
        object.register(&printer).unwrap();
        object
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert(
                "stepper_x".to_string(),
                tracker(&[1, 0, 0, 0, 0, 0, 0, 0], true),
            );
        let gcmd = gcode.create_gcode_command(
            "ENDSTOP_PHASE_CALIBRATE",
            "ENDSTOP_PHASE_CALIBRATE STEPPER=stepper_x",
            HashMap::from([("STEPPER".to_string(), "stepper_x".to_string())]),
        );

        object.cmd_endstop_phase_calibrate(&gcmd).unwrap();

        assert!(
            emitted(&lines)
                .iter()
                .any(|line| line.contains("SAVE_CONFIG")),
            "{:?}",
            emitted(&lines)
        );
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .unwrap();
        let status = configfile.get_status(0.0);
        assert_eq!(status["save_config_pending"], json!(true));
        assert_eq!(
            status["save_config_pending_items"]["endstop_phase stepper_x"]["trigger_phase"],
            json!("0/8")
        );
    }

    /// A `stats_only` tracker counts the phase a home landed on and never asks
    /// for a position adjustment (`EndstopPhases.update_stepper`).
    #[test]
    fn test_a_stats_only_tracker_counts_and_never_adjusts() {
        let (printer, _gcode, _lines) = machine();
        let object = Arc::new(EndstopPhases::new(&printer).unwrap());
        object.register(&printer).unwrap();
        object
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert("stepper_x".to_string(), tracker(&[0; 8], true));

        let homing = HomingHandle::new();
        homing.lock().set_trigger_position("stepper_x", 10.0);
        homing.lock().set_primary("stepper_x", true);
        object.handle_home_rails_end(&homing);

        // 10 % 8 = 2: the tracker counted one sample at phase 2.
        let tracked = Arc::clone(
            &object
                .tracking
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())["stepper_x"],
        );
        let tracked = tracked.lock().unwrap_or_else(|poison| poison.into_inner());
        assert_eq!(tracked.phase_history.as_ref().unwrap()[2], 1);
        assert_eq!(tracked.last_phase, Some(2));
        assert_eq!(tracked.last_mcu_position, Some(10.0));
        // It only counts; the run state carries no adjustment.
        assert!(homing.lock().adjustments().is_empty());
    }

    /// `get_status` reports one entry per tracked stepper with phases
    /// (`EndstopPhases.get_status`).
    #[test]
    fn test_get_status_reports_last_home() {
        let (printer, _gcode, _lines) = machine();
        let object = Arc::new(EndstopPhases::new(&printer).unwrap());
        object.register(&printer).unwrap();
        let tracked = tracker(&[1, 0, 0, 0, 0, 0, 0, 0], true);
        {
            let mut tracked = tracked.lock().unwrap_or_else(|poison| poison.into_inner());
            tracked.last_phase = Some(3);
            tracked.last_mcu_position = Some(11.0);
        }
        object
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert("stepper_x".to_string(), tracked);

        let status = object.get_status(0.0);
        assert_eq!(status["last_home"]["stepper_x"]["phase"], json!(3));
        assert_eq!(status["last_home"]["stepper_x"]["phases"], json!(8));
        assert_eq!(
            status["last_home"]["stepper_x"]["mcu_position"],
            json!(11.0)
        );
    }
}
