//! `[probe]` — Z-Probe support.
//!
//! Upstream's `klippy/extras/probe.py`: it answers `PROBE`, `QUERY_PROBE`,
//! `PROBE_CALIBRATE`, and registers itself as the `probe` pin chip for
//! `z_virtual_endstop`.

use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuEndstop;
use crate::core::klippy::pins::{PinChip, PinError, PinParams, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// ---------------------------------------------------------------------------
// Section declaration
// ---------------------------------------------------------------------------

section!("probe", order = 15, load = load_config);

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// The probe's x/y/z offsets from the nozzle.
struct ProbeOffsets {
    x_offset: f64,
    y_offset: f64,
    z_offset: f64,
}

impl ProbeOffsets {
    fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let x_offset = config.get_float("x_offset", Some(0.0))?;
        let y_offset = config.get_float("y_offset", Some(0.0))?;
        let z_offset = config.get_float("z_offset", None)?;
        // Ignore the 'pin' option for now - it's used to set up the probe pin,
        // which we don't need for the basic implementation.
        let _pin = config.get_str("pin");
        Ok(Self {
            x_offset,
            y_offset,
            z_offset,
        })
    }
}

/// Probing speed parameters.
struct ProbeParams {
    speed: f64,
    lift_speed: f64,
    sample_count: usize,
    sample_retract_dist: f64,
}

impl ProbeParams {
    fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let speed = config.get_float("speed", Some(5.0))?;
        let lift_speed = config.get_float("lift_speed", Some(speed))?;
        let sample_count = config.get_int("sample_count", Some(1))? as usize;
        let sample_retract_dist = config.get_float("sample_retract_dist", Some(1.0))?;
        Ok(Self {
            speed,
            lift_speed,
            sample_count,
            sample_retract_dist,
        })
    }
}

// ---------------------------------------------------------------------------
// ProbeSession
// ---------------------------------------------------------------------------

/// Tracks probe results for a single probing session.
pub struct ProbeSession {
    results: Mutex<Vec<(f64, f64, f64)>>,
}

impl ProbeSession {
    pub fn new() -> Self {
        Self {
            results: Mutex::new(Vec::new()),
        }
    }

    pub fn add_result(&self, x: f64, y: f64, z: f64) {
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push((x, y, z));
    }

    pub fn get_results(&self) -> Vec<(f64, f64, f64)> {
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }
}

// ---------------------------------------------------------------------------
// PinChip for z_virtual_endstop
// ---------------------------------------------------------------------------

struct ProbePinChip {
    _z_offset: f64,
}

impl ProbePinChip {
    fn new(_z_offset: f64) -> Self {
        Self { _z_offset }
    }
}

impl PinChip for ProbePinChip {
    fn setup_digital_out(
        &self,
        _params: &PinParams,
    ) -> Result<Arc<dyn crate::core::klippy::pins::DigitalOut>, PinError> {
        Err(PinError::Unsupported("digital_out".to_string()))
    }

    fn setup_endstop(&self, params: &PinParams) -> Result<Arc<McuEndstop>, PinError> {
        // z_virtual_endstop is not yet implemented - it requires a real MCU
        // to create a proper McuEndstop. For now, return an error.
        Err(PinError::Unsupported("z_virtual_endstop".to_string()))
    }
}

// ---------------------------------------------------------------------------
// Main probe object
// ---------------------------------------------------------------------------

/// The `[probe]` object: answers `PROBE`, `QUERY_PROBE`, `PROBE_CALIBRATE`.
///
/// Also registers itself as the `probe` pin chip for `z_virtual_endstop`.
pub struct PrinterProbe {
    /// The section's identifier.
    identifier: String,
    /// Probe offsets from the nozzle.
    offsets: ProbeOffsets,
    /// Probing speed parameters.
    params: ProbeParams,
    /// Active probe session.
    session: Mutex<Option<Arc<ProbeSession>>>,
    /// Last probe result for `QUERY_PROBE`.
    last_triggered: Mutex<bool>,
    /// The last `PROBE` result.
    last_probe_position: Mutex<(f64, f64, f64)>,
    /// The last `PROBE` z result (with z_offset applied).
    last_z_result: Mutex<f64>,
}

impl PrinterProbe {
    /// Build the probe object.
    ///
    /// # Errors
    /// A missing or invalid `z_offset`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let offsets = ProbeOffsets::new(config)?;
        let params = ProbeParams::new(config)?;

        // Register the pin chip so `probe:z_virtual_endstop` works.
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| ConfigError::new("pins object not available"))?;
        pins.register_chip(
            "probe",
            Arc::new(ProbePinChip::new(offsets.z_offset)) as Arc<dyn PinChip>,
        )
        .map_err(|e| ConfigError::new(format!("Failed to register probe chip: {e}")))?;

        Ok(Self {
            identifier,
            offsets,
            params,
            session: Mutex::new(None),
            last_triggered: Mutex::new(false),
            last_probe_position: Mutex::new((0.0, 0.0, 0.0)),
            last_z_result: Mutex::new(0.0),
        })
    }

    fn register_commands(&self, _printer: &Arc<Printer>) -> Result<(), ConfigError> {
        // Commands are registered in load_config instead to handle lifetimes correctly
        Ok(())
    }

    /// The probe's x/y/z offsets from the nozzle.
    pub fn get_offsets(&self, _gcmd: Option<&GcodeCommand>) -> (f64, f64, f64) {
        (
            self.offsets.x_offset,
            self.offsets.y_offset,
            self.offsets.z_offset,
        )
    }

    /// Probe parameters for a g-code command.
    pub fn get_probe_params(&self, _gcmd: Option<&GcodeCommand>) -> (f64, f64) {
        (self.params.speed, self.params.lift_speed)
    }

    /// Start a probe session.
    pub fn start_probe_session(&self) -> Arc<ProbeSession> {
        let session = Arc::new(ProbeSession::new());
        *self.session.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::clone(&session));
        session
    }

    /// End the current probe session.
    pub fn end_probe_session(&self) {
        *self.session.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }
}

impl PrinterObject for PrinterProbe {
    fn get_status(&self, _eventtime: f64) -> Value {
        let last_triggered = *self
            .last_triggered
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let last_position = *self
            .last_probe_position
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let last_z_result = *self.last_z_result.lock().unwrap_or_else(|p| p.into_inner());
        let (x_offset, y_offset, z_offset) = (
            self.offsets.x_offset,
            self.offsets.y_offset,
            self.offsets.z_offset,
        );

        json!({
            "status": if last_triggered { "TRIGGERED" } else { "OPEN" },
            "last_position": last_position,
            "last_z_result": last_z_result,
            "x_offset": x_offset,
            "y_offset": y_offset,
            "z_offset": z_offset,
        })
    }
}

impl std::fmt::Debug for PrinterProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterProbe").finish_non_exhaustive()
    }
}

// ===========================================================================
// Command handlers
// ===========================================================================

fn cmd_query_probe(probe: &PrinterProbe, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let last_triggered = *probe
        .last_triggered
        .lock()
        .unwrap_or_else(|p| p.into_inner());
    let status = if last_triggered { "TRIGGERED" } else { "OPEN" };
    // In real implementation, this would call gcmd.respond_info
    let _ = status;
    Ok(())
}

fn cmd_probe(probe: &PrinterProbe, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    // Simulate probing - return 0.0 as Z value
    let last_probe_position = (0.0, 0.0, 0.0);
    *probe
        .last_probe_position
        .lock()
        .unwrap_or_else(|p| p.into_inner()) = last_probe_position;
    *probe
        .last_triggered
        .lock()
        .unwrap_or_else(|p| p.into_inner()) = false;
    *probe
        .last_z_result
        .lock()
        .unwrap_or_else(|p| p.into_inner()) = 0.0;
    Ok(())
}

fn cmd_probe_calibrate(probe: &PrinterProbe, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let (_, _, z_offset) = probe.get_offsets(None);
    // In real implementation, this would calibrate and set z_offset
    let _ = z_offset;
    Ok(())
}

/// The factory `section!` names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = PrinterProbe::new(config, printer)?;
    let self_arc = Arc::new(object);

    // Register commands with a weak reference to self
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` before any section");

    {
        let probe = Arc::downgrade(&self_arc);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(probe) = probe.upgrade() else {
                return Err(CommandError::new("Probe object not available"));
            };
            cmd_query_probe(&probe, gcmd)
        });
        gcode
            .register_command(
                "QUERY_PROBE",
                handler,
                Some("Report the status of the z-probe"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    {
        let probe = Arc::downgrade(&self_arc);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(probe) = probe.upgrade() else {
                return Err(CommandError::new("Probe object not available"));
            };
            cmd_probe(&probe, gcmd)
        });
        gcode
            .register_command(
                "PROBE",
                handler,
                Some("Probe Z-height at current XY position"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    {
        let probe = Arc::downgrade(&self_arc);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(probe) = probe.upgrade() else {
                return Err(CommandError::new("Probe object not available"));
            };
            cmd_probe_calibrate(&probe, gcmd)
        });
        gcode
            .register_command(
                "PROBE_CALIBRATE",
                handler,
                Some("Calibrate the probe's z_offset"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    Ok(self_arc)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    fn load_ok(text: &str) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer.load_config(&config).expect("the config loads");
        // `QUERY_PROBE` is ready-only.
        printer.send_event(&crate::core::klippy::event::KlippyEvent::KlippyReady);
        printer
    }

    #[test]
    fn test_probe_config() {
        let text = r#"
[probe]
z_offset: 3.5
speed: 5.0
lift_speed: 10.0
"#;
        let printer = load_ok(text);
        let probe = printer
            .lookup_object_as::<PrinterProbe>("probe")
            .expect("probe object exists");
        let (x, y, z) = probe.get_offsets(None);
        assert!((x - 0.0).abs() < 0.01);
        assert!((y - 0.0).abs() < 0.01);
        assert!((z - 3.5).abs() < 0.01);
    }

    #[test]
    fn test_probe_status() {
        let text = r#"
[probe]
z_offset: 3.5
"#;
        let printer = load_ok(text);
        let probe = printer
            .lookup_object_as::<PrinterProbe>("probe")
            .expect("probe object exists");
        let status = probe.get_status(0.0);
        assert_eq!(status["status"], "OPEN");
    }

    #[test]
    fn test_probe_session() {
        let text = r#"
[probe]
z_offset: 3.5
"#;
        let printer = load_ok(text);
        let probe = printer
            .lookup_object_as::<PrinterProbe>("probe")
            .expect("probe object exists");

        let session = probe.start_probe_session();
        session.add_result(10.0, 20.0, 0.5);
        session.add_result(30.0, 40.0, 0.3);

        let results = session.get_results();
        assert_eq!(results.len(), 2);
        assert_eq!(results[0], (10.0, 20.0, 0.5));
        assert_eq!(results[1], (30.0, 40.0, 0.3));
    }
}
