//! `[quad_gantry_level]` — conform a moving, twistable gantry to a
//! stationary bed with four Z steppers (upstream
//! `klippy/extras/quad_gantry_level.py`).
//!
//! Everything the leveling family shares — [`RetryHelper`],
//! [`ZAdjustStatus`], [`ZAdjustHelper`] — comes from
//! [`z_tilt`](super::z_tilt), exactly as upstream's `quad_gantry_level.py`
//! imports them (`from . import probe, z_tilt`). What is unique here is the
//! math: no coordinate descent, but a two-point line fit along X, another
//! pair along Y, the four gantry corners' heights from them, and the
//! `max_adjust` abort when one corner would have to move too far.
//!
//! Two upstream quirks are kept on purpose: `probe_finalize` uses the
//! section's own `horizontal_move_z`, not the running command's
//! `HORIZONTAL_MOVE_Z` override (`quad_gantry_level.py:31` vs `:57`), and the
//! retry check compares the *gantry-relative* probe heights, while `z_tilt`
//! compares the raw probed Z.

use std::sync::{Arc, Mutex, OnceLock, Weak};

use serde_json::Value;
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::probe::{
    ProbeOffsets, ProbePointsFinalize, ProbePointsHelper, RETRY,
};
use crate::core::klippy::extras::z_tilt::{
    block_in_command, read_xy_option, RetryHelper, ZAdjustHelper, ZAdjustStatus,
};
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

section!("quad_gantry_level", order = 30, load = load_config);

/// The object this section is registered under (its finalize callback looks
/// itself up through the registry, as `z_tilt`'s does).
const QGL_OBJECT: &str = "quad_gantry_level";

/// The four actuator names the reports use (`quad_gantry_level.py:106`).
const ACTUATORS: [&str; 4] = ["z", "z1", "z2", "z3"];

/// The gantry-relative corner heights and the motor adjustments from them.
struct GantryFit {
    /// The corner heights, actuator order (`quad_gantry_level.py:87-92`).
    heights: [f64; 4],
    /// `average - height` per actuator (`quad_gantry_level.py:112-115`).
    adjustments: Vec<f64>,
}

/// Fit a straight line through two points (`quad_gantry_level.py:126-133`).
///
/// Equal Y is upstream's "straight line" shortcut: the horizontal fit
/// `(0, y)`. Equal X with unequal Y would divide by zero upstream; here it
/// yields an infinite slope, which the `max_adjust` check then refuses.
fn linefit(p1: (f64, f64), p2: (f64, f64)) -> (f64, f64) {
    if p1.1 == p2.1 {
        return (0.0, p1.1);
    }
    let slope = (p2.1 - p1.1) / (p2.0 - p1.0);
    let intercept = p1.1 - slope * p1.0;
    (slope, intercept)
}

/// Evaluate a fitted line (`quad_gantry_level.py:134-135`).
fn plot(fit: (f64, f64), x: f64) -> f64 {
    fit.0 * x + fit.1
}

/// The gantry-relative heights of the four gantry corners
/// (`quad_gantry_level.py:57-92`): mirror the probed Z into the gantry's
/// frame, fit the X slopes across probe pairs 0–3 and 1–2, take the Y
/// slopes at the two `gantry_corners` abscissas, and read each corner's
/// height off them. The probe X/Y carry the probe's own offsets, as
/// upstream's `positions[i] + offsets` do.
fn gantry_heights(
    offsets: &ProbeOffsets,
    positions: &[Coord],
    gantry_corners: &[(f64, f64)],
    horizontal_move_z: f64,
) -> [f64; 4] {
    // The gantry-relative probe points (`quad_gantry_level.py:59-63`).
    let z_positions: Vec<f64> = positions
        .iter()
        .map(|position| horizontal_move_z - position.z())
        .collect();

    // Slope along X between probe points 0 and 3, and between 1 and 2
    // (`quad_gantry_level.py:66-74`).
    let slope_x_pp03 = linefit(
        (positions[0].x() + offsets.x, z_positions[0]),
        (positions[3].x() + offsets.x, z_positions[3]),
    );
    let slope_x_pp12 = linefit(
        (positions[1].x() + offsets.x, z_positions[1]),
        (positions[2].x() + offsets.x, z_positions[2]),
    );

    // The gantry slopes along Y — evaluated at each gantry corner's
    // abscissa, from probe points 0 and 1 (`quad_gantry_level.py:77-85`).
    let slope_y_s01 = linefit(
        (
            positions[0].y() + offsets.y,
            plot(slope_x_pp03, gantry_corners[0].0),
        ),
        (
            positions[1].y() + offsets.y,
            plot(slope_x_pp12, gantry_corners[0].0),
        ),
    );
    let slope_y_s23 = linefit(
        (
            positions[0].y() + offsets.y,
            plot(slope_x_pp03, gantry_corners[1].0),
        ),
        (
            positions[1].y() + offsets.y,
            plot(slope_x_pp12, gantry_corners[1].0),
        ),
    );

    // The z height of each stepper (`quad_gantry_level.py:87-92`).
    [
        plot(slope_y_s01, gantry_corners[0].1),
        plot(slope_y_s01, gantry_corners[1].1),
        plot(slope_y_s23, gantry_corners[1].1),
        plot(slope_y_s23, gantry_corners[0].1),
    ]
}

/// The corner heights plus the motor adjustments, including the
/// `max_adjust` abort (`quad_gantry_level.py:94-116`): each actuator moves
/// by `average - height`.
///
/// # Errors
/// "Aborting quad_gantry_level required adjustment … is greater than
/// max_adjust …" (`quad_gantry_level.py:99-103`).
fn fit_gantry(
    offsets: &ProbeOffsets,
    positions: &[Coord],
    gantry_corners: &[(f64, f64)],
    horizontal_move_z: f64,
    max_adjust: f64,
) -> Result<GantryFit, CommandError> {
    let heights = gantry_heights(offsets, positions, gantry_corners, horizontal_move_z);
    let average = heights.iter().sum::<f64>() / heights.len() as f64;
    let adjustments: Vec<f64> = heights.iter().map(|height| average - height).collect();
    let adjust_max = adjustments
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    if adjust_max > max_adjust {
        return Err(CommandError::new(format!(
            "Aborting quad_gantry_level required adjustment {adjust_max:.6} is greater than max_adjust {max_adjust:.6}"
        )));
    }
    Ok(GantryFit {
        heights,
        adjustments,
    })
}

/// The rows of `points`, counted as upstream counts `probe_points`
/// (`quad_gantry_level.py:34-37`).
fn probe_helper_point_count(config: &ConfigWrapper) -> Result<usize, ConfigError> {
    Ok(config.get_list_of_lists("points", '\n', ',', 2)?.len())
}

/// The `[quad_gantry_level]` section (`quad_gantry_level.py:26-125`).
pub struct QuadGantryLevel {
    /// The printer, for the finalize reports (upstream keeps `self.gcode`).
    printer: Weak<Printer>,
    /// The section's own `horizontal_move_z` — deliberately *not* the
    /// running command's `HORIZONTAL_MOVE_Z` override (upstream's quirk,
    /// `quad_gantry_level.py:31`).
    horizontal_move_z: f64,
    max_adjust: f64,
    gantry_corners: Vec<(f64, f64)>,
    retry_helper: RetryHelper,
    /// See `z_tilt::ZTilt::probe_helper`.
    probe_helper: OnceLock<Arc<ProbePointsHelper>>,
    z_status: Arc<ZAdjustStatus>,
    z_helper: ZAdjustHelper,
    /// See `z_tilt::ZTilt::last_error`.
    last_error: Mutex<Option<CommandError>>,
}

impl QuadGantryLevel {
    /// Read the section, wire the probe helper's callback, and register
    /// `QUAD_GANTRY_LEVEL` (`quad_gantry_level.py:26-52`), in upstream's
    /// order: retries and limits, the probe points (and the exactly-four
    /// check), the status and motor helpers, `gantry_corners`, the command.
    ///
    /// # Errors
    /// A malformed option or one of upstream's load-time complaints: "Need
    /// exactly 4 probe points for quad_gantry_level", "quad_gantry_level
    /// requires at least two gantry_corners".
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let retry_helper =
            RetryHelper::new(config, printer, "Possibly Z motor numbering is wrong")?;
        let max_adjust =
            config.get_float_bounded("max_adjust", Some(4.0), None, None, Some(0.0), None)?;
        let horizontal_move_z = config.get_float("horizontal_move_z", Some(5.0))?;

        // As `z_tilt`: the callback reaches the section through the registry,
        // so it is built before the object that uses it.
        let printer_weak = Arc::downgrade(printer);
        let finalize: ProbePointsFinalize = Arc::new(move |offsets, positions| {
            let Some(printer) = printer_weak.upgrade() else {
                return None;
            };
            let Some(qgl) = printer.lookup_object_as::<QuadGantryLevel>(QGL_OBJECT) else {
                warn!("QUAD_GANTRY_LEVEL finalize: the quad_gantry_level object is gone");
                return None;
            };
            qgl.probe_finalize(offsets, positions)
        });
        let probe_helper = ProbePointsHelper::new(config, printer, finalize)?;
        if probe_helper_point_count(config)? != 4 {
            return Err(ConfigError::new(
                "Need exactly 4 probe points for quad_gantry_level",
            ));
        }

        let z_status = ZAdjustStatus::new(printer);
        let z_helper = ZAdjustHelper::new(config, printer, 4);

        let gantry_corners = read_xy_option(config, "gantry_corners")?;
        if gantry_corners.len() < 2 {
            return Err(ConfigError::new(
                "quad_gantry_level requires at least two gantry_corners",
            ));
        }

        let qgl = Arc::new(Self {
            printer: Arc::downgrade(printer),
            horizontal_move_z,
            max_adjust,
            gantry_corners,
            retry_helper,
            probe_helper: OnceLock::new(),
            z_status,
            z_helper,
            last_error: Mutex::new(None),
        });
        qgl.probe_helper
            .set(probe_helper)
            .unwrap_or_else(|_| unreachable!("the probe helper is wired once"));

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        let this = Arc::downgrade(&qgl);
        let handler: CommandHandler = Arc::new(move |gcmd| {
            let this = this.clone();
            Box::pin(async move {
                let this = this
                    .upgrade()
                    .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                this.cmd_quad_gantry_level(gcmd).await
            })
        });
        gcode
            .register_command(
                "QUAD_GANTRY_LEVEL",
                handler,
                Some("Conform a moving, twistable gantry to the shape of a stationary bed"),
                false,
            )
            .map_err(ConfigError::new)?;

        Ok(qgl)
    }

    /// `QUAD_GANTRY_LEVEL` (`quad_gantry_level.py:44-52`): clear the applied
    /// flag, arm the retries, probe every point.
    ///
    /// # Errors
    /// As `z_tilt::ZTilt::cmd_Z_TILT_ADJUST`.
    async fn cmd_quad_gantry_level(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.z_status.reset();
        self.retry_helper.start(gcmd)?;
        *self.last_error.lock().unwrap_or_else(|p| p.into_inner()) = None;
        let probe_helper = self
            .probe_helper
            .get()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        probe_helper.start_probe(gcmd).await?;
        if let Some(error) = self
            .last_error
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
        {
            return Err(error);
        }
        Ok(())
    }

    /// The probe helper's finalize callback; the error recording works as
    /// `z_tilt::ZTilt::probe_finalize` describes.
    fn probe_finalize(&self, offsets: ProbeOffsets, positions: &[Coord]) -> Option<&'static str> {
        match self.run_finalize(offsets, positions) {
            Ok(result) => (result == RETRY).then_some(RETRY),
            Err(error) => {
                warn!("QUAD_GANTRY_LEVEL: {error}");
                let mut slot = self.last_error.lock().unwrap_or_else(|p| p.into_inner());
                if slot.is_none() {
                    *slot = Some(error);
                }
                None
            }
        }
    }

    /// The finalize work (`quad_gantry_level.py:54-123`): report the
    /// gantry-relative probe points, fit the corners and their adjustments,
    /// move the motors, then run the retry check over the *gantry-relative*
    /// heights (unlike `z_tilt`, which uses the raw probed Z).
    fn run_finalize(
        &self,
        offsets: ProbeOffsets,
        positions: &[Coord],
    ) -> Result<&'static str, CommandError> {
        let probe_helper = self
            .probe_helper
            .get()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let gcode = self
            .printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;

        // The gantry-relative probe points (`quad_gantry_level.py:59-63`).
        let z_positions: Vec<f64> = positions
            .iter()
            .map(|position| self.horizontal_move_z - position.z())
            .collect();
        let points_message = format!(
            "Gantry-relative probe points:\n{}\n",
            z_positions
                .iter()
                .enumerate()
                .map(|(index, height)| format!("{index}: {height:.6}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
        gcode.respond_info(&points_message, true);

        let fit = fit_gantry(
            &offsets,
            positions,
            &self.gantry_corners,
            self.horizontal_move_z,
            self.max_adjust,
        )?;
        let apos = fit
            .heights
            .iter()
            .zip(ACTUATORS)
            .map(|(height, name)| format!("{name}: {height:.6}"))
            .collect::<Vec<_>>()
            .join(" ");
        gcode.respond_info(&format!("Actuator Positions:\n{apos}"), true);
        let average = fit.heights.iter().sum::<f64>() / fit.heights.len() as f64;
        gcode.respond_info(&format!("Average: {average:.6}"), true);

        let speed = probe_helper.get_lift_speed();
        block_in_command(self.z_helper.adjust_steppers(&fit.adjustments, speed))?;
        let result = self.retry_helper.check_retry(&z_positions)?;
        Ok(self.z_status.check_retry_result(result))
    }
}

impl PrinterObject for QuadGantryLevel {
    fn get_status(&self, eventtime: f64) -> Value {
        self.z_status.get_status(eventtime)
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move { self.z_helper.handle_connect() })
    }
}

/// The factory `section!` names (`quad_gantry_level.py:load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = QuadGantryLevel::new(config, printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}
