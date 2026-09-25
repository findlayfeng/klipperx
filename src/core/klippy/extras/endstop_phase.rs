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
//!
//! Upstream widens `endstop_phase_accuracy` to `phases` under `debugoutput`
//! (its test mode); this host has no such start argument, so the computed
//! accuracy always stands.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::info;

use crate::core::klippy::config::object::CONFIGFILE_OBJECT;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::stepper::PrinterStepper;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::motion::HomingHandle;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!(
    "endstop_phase",
    order = 55,
    phase = late,
    load = load_config,
    prefix = load_config_prefix
);

/// The object name the bare section registers under, and the prefix of each
/// `[endstop_phase <stepper>]` object's name (`endstop_phase.py:73`).
const ENDSTOP_PHASE_OBJECT: &str = "endstop_phase";

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

    /// Convert a driver's phase count to this tracker's
    /// (`PhaseCalc.convert_phase`): `round(driver_phase / driver_phases ×
    /// phases)`, taken modulo `phases`.
    fn convert_phase(&self, driver_phase: f64, driver_phases: f64) -> usize {
        let phases = self
            .phases
            .expect("convert_phase is only called with phases known");
        ((driver_phase / driver_phases * phases as f64 + 0.5) as usize) % phases
    }
}

/// One `[endstop_phase <stepper>]` section (`EndstopPhase`).
///
/// It tracks the phase its stepper triggers at and, once the phase is known,
/// asks the driver to move the axis' post-home position to it.
/// `endstop_align_zero` additionally moves 0.0 onto a full microstep.
pub struct EndstopPhase {
    /// The stepper's name (`stepper_x`).
    name: String,
    /// The tracker the bare section also reads (upstream shares the object
    /// through `printer.load_object`).
    phase_calc: Arc<Mutex<PhaseCalc>>,
    /// Millimetres per step (`step_dist = rotation_dist / steps_per_rotation`).
    step_dist: f64,
    /// The phase count (`microsteps × 4`).
    phases: usize,
    /// The axis' endstop position, for `align_endstop`
    /// (`rail.get_homing_info().position_endstop`).
    position_endstop: f64,
    /// The phase the axis is aligned to (`endstop_phase`): config's
    /// `trigger_phase`, or the first home's phase when it is unset.
    endstop_phase: Mutex<Option<usize>>,
    /// `endstop_align_zero`: put 0.0 on a full microstep.
    endstop_align_zero: bool,
    /// The largest phase error accepted (`endstop_phase_accuracy`).
    endstop_phase_accuracy: usize,
}

impl EndstopPhase {
    /// Build the section (`EndstopPhase.__init__`).
    ///
    /// # Errors
    /// A missing `[<stepper>]` section, a malformed `trigger_phase`, an
    /// `endstop_accuracy` too coarse for the phase count, or an option outside
    /// its bounds.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{}' needs a stepper name",
                config.identifier()
            ))
        })?;
        let stepper = printer
            .lookup_object_as::<PrinterStepper>(&name)
            .ok_or_else(|| ConfigError::new(format!("Section '{name}' not found")))?;
        let step_dist = stepper.step_dist();
        let phases = (stepper.microsteps() * 4) as usize;
        let position_endstop = stepper.homing_info().position_endstop;
        let phase_calc = Arc::new(Mutex::new(PhaseCalc::new(&name, Some(phases))));

        let mut endstop_phase = None;
        if config.has("trigger_phase") {
            let text = config.get("trigger_phase", None)?;
            let items = config.get_list("trigger_phase", '/').unwrap_or_default();
            let (p, ps) = match items.as_slice() {
                [p, ps] => {
                    let parse = |value: &str| -> Result<i64, ConfigError> {
                        value.trim().parse::<i64>().map_err(|_| {
                            ConfigError::new(format!(
                                "Option 'trigger_phase' in section '{}' is not a list of 2 \
                                 integers",
                                config.identifier()
                            ))
                        })
                    };
                    (parse(p)?, parse(ps)?)
                }
                _ => {
                    return Err(ConfigError::new(format!(
                        "Option 'trigger_phase' in section '{}' is not a list of 2 integers",
                        config.identifier()
                    )));
                }
            };
            if p >= ps {
                return Err(ConfigError::new(format!("Invalid trigger_phase '{text}'")));
            }
            endstop_phase = Some(
                phase_calc
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .convert_phase(p as f64, ps as f64),
            );
        }
        let endstop_align_zero = config.get_bool("endstop_align_zero", Some(false))?;
        let endstop_accuracy = if config.has("endstop_accuracy") {
            Some(config.get_float_bounded("endstop_accuracy", None, None, None, Some(0.0), None)?)
        } else {
            None
        };
        let endstop_phase_accuracy = match (endstop_accuracy, endstop_phase) {
            (None, _) => phases / 2 - 1,
            // A trigger phase halves the tolerated error: it pins the phase to
            // within half a step (`endstop_phase.py:80-93`).
            (Some(accuracy), Some(_)) => (accuracy * 0.5 / step_dist).ceil() as usize,
            (Some(accuracy), None) => (accuracy / step_dist).ceil() as usize,
        };
        if endstop_phase_accuracy >= phases / 2 {
            return Err(ConfigError::new(format!(
                "Endstop for {name} is not accurate enough for stepper phase adjustment"
            )));
        }
        Ok(Self {
            name,
            phase_calc,
            step_dist,
            phases,
            position_endstop,
            endstop_phase: Mutex::new(endstop_phase),
            endstop_align_zero,
            endstop_phase_accuracy,
        })
    }

    /// The offset that puts 0.0 on a full microstep
    /// (`EndstopPhase.align_endstop`), or `0.0` when disabled.
    fn align_endstop(&self) -> f64 {
        let Some(endstop_phase) = *self
            .endstop_phase
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
        else {
            return 0.0;
        };
        if !self.endstop_align_zero {
            return 0.0;
        }
        let microsteps = self.phases / 4;
        let half_microsteps = microsteps / 2;
        let phase_offset = (((endstop_phase + half_microsteps) % microsteps) as i64
            - half_microsteps as i64) as f64
            * self.step_dist;
        let full_step = microsteps as f64 * self.step_dist;
        let pe = self.position_endstop;
        (pe / full_step + 0.5) as i64 as f64 * full_step - pe + phase_offset
    }

    /// The offset the axis should move by to reach the aligned phase
    /// (`EndstopPhase.get_homed_offset`).
    ///
    /// The first home only records the phase and returns `0.0`; later homes
    /// compare against it. A phase beyond the accuracy raises.
    ///
    /// # Errors
    /// When the phase differs from the recorded one by more than
    /// `endstop_phase_accuracy` steps.
    fn get_homed_offset(&self, trig_mcu_pos: f64) -> Result<f64, CommandError> {
        let phase = self
            .phase_calc
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .calc_phase(trig_mcu_pos);
        let mut endstop_phase = self
            .endstop_phase
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let Some(reference) = *endstop_phase else {
            info!("Setting {} endstop phase to {}", self.name, phase);
            *endstop_phase = Some(phase);
            return Ok(0.0);
        };
        let mut delta = (phase as i64 - reference as i64).rem_euclid(self.phases as i64);
        if delta >= self.phases as i64 - self.endstop_phase_accuracy as i64 {
            delta -= self.phases as i64;
        } else if delta > self.endstop_phase_accuracy as i64 {
            return Err(CommandError::new(format!(
                "Endstop {} incorrect phase (got {} vs {})",
                self.name, phase, reference
            )));
        }
        Ok(delta as f64 * self.step_dist)
    }

    /// The `homing:home_rails_end` handler: when this section's stepper homed,
    /// ask the driver to nudge its endstop position
    /// (`EndstopPhase.handle_home_rails_end`).
    fn handle_home_rails_end(&self, homing: &HomingHandle) {
        let mut state = homing.lock();
        if !state.has_trigger(&self.name) {
            return;
        }
        let trig_mcu_pos = state.get_trigger_position(&self.name);
        let align = self.align_endstop();
        match self.get_homed_offset(trig_mcu_pos) {
            Ok(offset) => state.set_stepper_adjustment(&self.name, align + offset),
            Err(error) => state.set_error(error.message().to_string()),
        }
    }
}

impl PrinterObject for EndstopPhase {
    /// Upstream's `EndstopPhase` has no `get_status`. It reports an empty
    /// object and stays out of `objects/list`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
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
                    // A stepper with an `[endstop_phase <stepper>]` section
                    // shares that section's tracker (its phases are known);
                    // one without gets a tracker that only counts phases
                    // (`EndstopPhases.update_stepper`).
                    let prefix = self.printer.upgrade().and_then(|printer| {
                        printer.lookup_object_as::<EndstopPhase>(&format!(
                            "{ENDSTOP_PHASE_OBJECT} {stepper_name}"
                        ))
                    });
                    let phase_calc = match prefix {
                        Some(prefix) => Arc::clone(&prefix.phase_calc),
                        None => {
                            let mut phase_calc = PhaseCalc::new(stepper_name, None);
                            phase_calc.stats_only = true;
                            if let Some(printer) = self.printer.upgrade() {
                                phase_calc.lookup_tmc(&printer);
                            }
                            Arc::new(Mutex::new(phase_calc))
                        }
                    };
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

    /// The single bare section, created on demand when the config has only
    /// `[endstop_phase <stepper>]` sections.
    ///
    /// Upstream's `EndstopPhase.__init__` calls
    /// `printer.load_object(config, "endstop_phase")` for exactly this reason;
    /// a config that also writes `[endstop_phase]` has already registered it
    /// (main sections load before prefix sections).
    ///
    /// # Errors
    /// A duplicate registration or a g-code name this dispatcher refuses.
    pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<Self>(ENDSTOP_PHASE_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(Self::new(printer)?);
        object.register(printer)?;
        printer.add_object(
            ENDSTOP_PHASE_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
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

/// The `[endstop_phase <stepper>]` factory.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(EndstopPhase::new(config, printer)?);
    // The prefix node registers its handler before the bare section is created,
    // so it runs first and the bare section sees the phase it recorded
    // (`endstop_phase.py:63-67`).
    printer.register_event_handler(
        KlippyEvent::HomingHomeRailsEnd {
            axes: Vec::new(),
            homing: HomingHandle::new(),
        },
        Box::new({
            let object = Arc::clone(&object);
            move |event| {
                if let KlippyEvent::HomingHomeRailsEnd { homing, .. } = event {
                    object.handle_home_rails_end(homing);
                }
            }
        }),
    );
    EndstopPhases::ensure(printer)?;
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

    // =======================================================================
    // `[endstop_phase <stepper>]`
    // =======================================================================

    /// Load a config text the way the host does.
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = crate::core::klippy::config::Config::from_text(text)
            .expect("the test config parses")
            .0;
        let result = printer.load_config(&config);
        (printer, result)
    }

    /// An `[mcu]`, the makergear `[stepper_x]` geometry (8 microsteps, 36 mm
    /// per rotation), and `[endstop_phase stepper_x]` with `extra` options.
    fn config_with_x(extra: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nenable_pin: !PA7\n\
             microsteps: 8\nrotation_distance: 36\nposition_endstop: 0.0\n\
             position_max: 200\n\
             [endstop_phase stepper_x]\n{extra}"
        )
    }

    fn prefix_object(printer: &Arc<Printer>) -> Arc<EndstopPhase> {
        printer
            .lookup_object_as::<EndstopPhase>("endstop_phase stepper_x")
            .expect("the prefix section loads")
    }

    /// The bare section with zero options loads through the real loader, so
    /// `check_unused` accepts it (tmc.cfg's `[endstop_phase]`).
    #[test]
    fn test_the_bare_section_passes_check_unused() {
        let (printer, result) = load("[mcu]\nserial: /dev/not-opened-yet\n[endstop_phase]\n");

        result.unwrap();
        assert!(printer
            .lookup_object_as::<EndstopPhases>(ENDSTOP_PHASE_OBJECT)
            .is_some());
    }

    /// The phase count is `microsteps × 4` (`endstop_phase.py:58`).
    #[test]
    fn test_the_prefix_section_uses_microsteps_times_four() {
        let (printer, result) = load(&config_with_x(""));

        result.unwrap();
        let object = prefix_object(&printer);
        assert_eq!(object.phases, 32, "8 microsteps × 4");
        assert_eq!(object.step_dist, 36.0 / (200.0 * 8.0));
        // The bare section is created on demand (upstream's `load_object`); it
        // keeps its `get_status`, while the prefix object is not queryable.
        let bare = printer
            .lookup_object_as::<EndstopPhases>(ENDSTOP_PHASE_OBJECT)
            .expect("the bare section is ensured");
        assert!(bare.is_queryable());
        assert!(!object.is_queryable());
    }

    /// A `trigger_phase` pins the phase and halves the tolerated error
    /// (`endstop_phase.py:80-93`).
    #[test]
    fn test_trigger_phase_selects_the_phase_and_halves_the_accuracy() {
        let (printer, result) = load(&config_with_x("trigger_phase: 1/4\nendstop_accuracy: .200"));

        result.unwrap();
        let object = prefix_object(&printer);
        // round(1/4 × 32) = 8.
        assert_eq!(
            *object
                .endstop_phase
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()),
            Some(8)
        );
        // ceil(0.200 × 0.5 / step_dist) = 5, not the 9 a plain
        // `endstop_accuracy` would give.
        assert_eq!(object.endstop_phase_accuracy, 5);
    }

    /// Without a `trigger_phase` the accuracy is not halved.
    #[test]
    fn test_endstop_accuracy_without_a_trigger_phase_is_not_halved() {
        let (printer, result) = load(&config_with_x("endstop_accuracy: .200"));

        result.unwrap();
        assert_eq!(prefix_object(&printer).endstop_phase_accuracy, 9);
    }

    /// `trigger_phase` with `p >= ps` is rejected verbatim.
    #[test]
    fn test_an_invalid_trigger_phase_is_rejected() {
        let (_printer, result) = load(&config_with_x("trigger_phase: 3/3"));

        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("Invalid trigger_phase '3/3'"),
            "{err}"
        );
    }

    /// An accuracy too coarse for the phase count is rejected.
    #[test]
    fn test_a_coarse_endstop_accuracy_is_rejected() {
        let (_printer, result) = load(&config_with_x("endstop_accuracy: 5.0"));

        let err = result.unwrap_err();
        assert!(
            err.to_string().contains(
                "Endstop for stepper_x is not accurate enough for stepper phase adjustment"
            ),
            "{err}"
        );
    }

    /// A directly built section with `phases` and a `step_dist` for the
    /// offset math.
    fn phase(
        phases: usize,
        step_dist: f64,
        reference: Option<usize>,
        accuracy: usize,
    ) -> EndstopPhase {
        EndstopPhase {
            name: "stepper_x".to_string(),
            phase_calc: Arc::new(Mutex::new(PhaseCalc::new("stepper_x", Some(phases)))),
            step_dist,
            phases,
            position_endstop: 0.0,
            endstop_phase: Mutex::new(reference),
            endstop_align_zero: false,
            endstop_phase_accuracy: accuracy,
        }
    }

    /// The first home records the phase and returns `0.0`
    /// (`endstop_phase.py:105-118`).
    #[test]
    fn test_the_first_home_records_the_phase_and_returns_zero() {
        let object = phase(8, 1.0, None, 2);

        assert_eq!(object.get_homed_offset(3.0).unwrap(), 0.0);
        assert_eq!(
            *object
                .endstop_phase
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()),
            Some(3)
        );
    }

    /// A sample within `accuracy` of the reference returns the signed
    /// distance; the caller applies its own sign.
    #[test]
    fn test_a_known_phase_returns_the_step_offset() {
        let object = phase(8, 0.5, Some(3), 2);

        // One step ahead: +0.5 mm.
        assert_eq!(object.get_homed_offset(4.0).unwrap(), 0.5);
        // One step behind wraps to the short way: -0.5 mm.
        assert_eq!(object.get_homed_offset(2.0).unwrap(), -0.5);
    }

    /// A phase near the wrap subtracts `phases` instead of taking a modulo
    /// (`endstop_phase.py:113-114`).
    #[test]
    fn test_a_phase_near_the_wrap_subtracts_the_phase_count() {
        // reference 1, phases 8, accuracy 2: the wrap threshold is 6.
        let object = phase(8, 1.0, Some(1), 2);

        // (7 - 1) % 8 = 6 >= 8 - 2: 6 - 8 = -2 steps, not +6.
        assert_eq!(object.get_homed_offset(7.0).unwrap(), -2.0);
    }

    /// A phase beyond the tolerance raises upstream's message.
    #[test]
    fn test_a_phase_beyond_the_tolerance_raises() {
        let object = phase(8, 1.0, Some(1), 2);

        let err = object.get_homed_offset(5.0).unwrap_err();
        assert_eq!(
            err.message(),
            "Endstop stepper_x incorrect phase (got 5 vs 1)"
        );
    }

    /// The homing handler nudges the axis by the offset the phase math gives.
    #[test]
    fn test_the_prefix_handler_sets_a_stepper_adjustment() {
        let object = phase(8, 1.0, Some(3), 2);
        let homing = HomingHandle::new();
        homing.lock().set_trigger_position("stepper_x", 4.0);

        object.handle_home_rails_end(&homing);

        assert_eq!(homing.lock().adjustments()["stepper_x"], 1.0);
    }

    /// A phase mismatch is left in the run state for the driver to raise
    /// (a port event handler cannot return an error).
    #[test]
    fn test_the_prefix_handler_records_a_phase_error() {
        let object = phase(8, 1.0, Some(1), 1);
        let homing = HomingHandle::new();
        homing.lock().set_trigger_position("stepper_x", 5.0);

        object.handle_home_rails_end(&homing);

        let mut state = homing.lock();
        assert_eq!(
            state.take_error(),
            Some("Endstop stepper_x incorrect phase (got 5 vs 1)".to_string())
        );
        assert!(state.adjustments().is_empty());
    }

    /// A stepper with a prefix section shares that section's tracker, so the
    /// bare section counts the same history (`endstop_phase.py:191-209`).
    #[test]
    fn test_the_bare_section_shares_the_prefix_tracker() {
        let (printer, result) = load(&config_with_x(""));
        result.unwrap();
        let bare = printer
            .lookup_object_as::<EndstopPhases>(ENDSTOP_PHASE_OBJECT)
            .unwrap();
        let prefix = prefix_object(&printer);

        bare.update_stepper("stepper_x", 3.0, true);

        let tracked = bare
            .tracking
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())["stepper_x"]
            .clone();
        assert!(Arc::ptr_eq(&tracked, &prefix.phase_calc));
        let tracked = tracked.lock().unwrap_or_else(|poison| poison.into_inner());
        assert!(!tracked.stats_only, "the prefix tracker adjusts the axis");
        assert_eq!(tracked.phase_history.as_ref().unwrap()[3], 0);
    }
}
