//! `bed_tilt` — level a tilted bed in the G-Code coordinate system.
//!
//! Upstream's `klippy/extras/bed_tilt.py`. Where `z_tilt` moves Z steppers,
//! this module never touches them: it claims the `gcode_move` move-transform
//! slot ([`MoveTarget`]) and shifts **Z** by a plane — `x_adjust`/`y_adjust`
//! the slopes, `z_adjust` the intercept — so every `G1`/`M114` speaks a
//! coordinate system that follows the bed's tilt:
//!
//! - [`BedTilt::position`] (upstream's `get_position`) **subtracts** the
//!   plane from the toolhead's Z;
//! - [`BedTilt::move_to`] (upstream's `move`) **adds** it back before the
//!   toolhead queues the move.
//!
//! The plane is data, not motion: the toolhead never moves a stepper — the
//! transform alone makes `G1` and `M114` follow the bed's tilt.
//!
//! | option | default | role |
//! |---|---|---|
//! | `x_adjust` / `y_adjust` / `z_adjust` | `0.` | the plane (the `SAVE_CONFIG` items) |
//! | `points` | — | probe points; **their presence** registers `BED_TILT_CALIBRATE` (`bed_tilt.py:18-19`) |
//! | `horizontal_move_z` / `speed` | `5.0` / `50.` | the calibration's travels (read by [`ProbePointsHelper`]) |
//!
//! [`ProbePointsHelper`]: crate::core::klippy::extras::probe::ProbePointsHelper

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_move::{self, MoveTarget};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, Z_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("bed_tilt", order = 30, load = load_config);

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The plane the transform applies: upstream's `x_adjust`/`y_adjust`/
/// `z_adjust` trio, copied as one value (`bed_tilt.py:36-42`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Adjust {
    /// The Z slope along X.
    x_adjust: f64,
    /// The Z slope along Y.
    y_adjust: f64,
    /// The plane's intercept (probe `z_offset` and XY offsets removed).
    z_adjust: f64,
}

impl Default for Adjust {
    fn default() -> Self {
        Self {
            x_adjust: 0.0,
            y_adjust: 0.0,
            z_adjust: 0.0,
        }
    }
}

/// One `[bed_tilt]` section: the plane and the move transform
/// (`bed_tilt.py:9-42`).
pub struct BedTilt {
    /// The machine, to find `toolhead`/`gcode_move`/`configfile` at run time.
    printer: Weak<Printer>,
    /// The compensation plane.
    adjust: Mutex<Adjust>,
    /// The toolhead the transform wraps — set at `klippy:connect`
    /// (upstream's `handle_connect`), the fake a test injects before that.
    target: Mutex<Option<Arc<dyn MoveTarget>>>,
}

impl BedTilt {
    /// Read the section's plane (`bed_tilt.py:14-17`).
    ///
    /// # Errors
    /// When an adjust option is malformed.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        Ok(Self {
            printer: Arc::downgrade(printer),
            adjust: Mutex::new(Adjust {
                x_adjust: config.get_float("x_adjust", Some(0.0))?,
                y_adjust: config.get_float("y_adjust", Some(0.0))?,
                z_adjust: config.get_float("z_adjust", Some(0.0))?,
            }),
            target: Mutex::new(None),
        })
    }

    /// The events upstream's `__init__` subscribes to (`bed_tilt.py:11-12`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.handle_connect()
            }),
        );
    }

    /// Upstream's `handle_connect`: the toolhead exists by `klippy:connect`.
    fn handle_connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
            *self.target.lock().unwrap_or_else(|p| p.into_inner()) =
                Some(Arc::new(ToolheadMove(toolhead)));
        }
    }

    /// The plane, copied out of its lock.
    fn adjust(&self) -> Adjust {
        *self.adjust.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The transform's underlying target, when there is one to ask.
    fn target(&self) -> Option<Arc<dyn MoveTarget>> {
        self.target
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }
}

impl MoveTarget for BedTilt {
    /// Upstream's `move` (`bed_tilt.py:29-32`): add the plane back — the
    /// caller speaks the tilted bed's coordinates, the toolhead the real ones.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let adjust = self.adjust();
        let mut toolhead_position = position;
        toolhead_position.set_axis(
            Z_AXIS,
            position.z()
                + position.x() * adjust.x_adjust
                + position.y() * adjust.y_adjust
                + adjust.z_adjust,
        );
        match self.target() {
            Some(target) => target.move_to(toolhead_position, speed),
            // No toolhead yet (`klippy:connect` has not run): upstream would
            // crash on `self.toolhead = None`; here the move is refused.
            None => Err(CommandError::new("Printer is not ready")),
        }
    }

    /// Upstream's `get_position` (`bed_tilt.py:26-28`): the toolhead's
    /// position with the plane **subtracted**, so the g-code space reads flat.
    fn position(&self) -> Coord {
        let Some(target) = self.target() else {
            // As above: before connect there is nothing to read.
            return Coord::default();
        };
        let toolhead_position = target.position();
        let adjust = self.adjust();
        let mut position = toolhead_position;
        position.set_axis(
            Z_AXIS,
            toolhead_position.z()
                - toolhead_position.x() * adjust.x_adjust
                - toolhead_position.y() * adjust.y_adjust
                - adjust.z_adjust,
        );
        position
    }
}

impl PrinterObject for BedTilt {
    /// Upstream's `BedTilt` defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Kept out of `objects/list`, as an object without `get_status`
    /// upstream.
    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for BedTilt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let adjust = self.adjust();
        f.debug_struct("BedTilt")
            .field("x_adjust", &adjust.x_adjust)
            .field("y_adjust", &adjust.y_adjust)
            .field("z_adjust", &adjust.z_adjust)
            .finish_non_exhaustive()
    }
}

/// The toolhead behind the transform: what `position` reads and `move_to`
/// feeds (upstream passes `self.toolhead` around directly).
struct ToolheadMove(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadMove {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn position(&self) -> Coord {
        self.0.position().unwrap_or_default()
    }
}

/// Upstream's `load_config` for `[bed_tilt]` (`bed_tilt.py:95`).
///
/// `gcode_move::ensure` stands in for upstream's
/// `load_object(config, 'gcode_move')`: the transform takes the slot before
/// ready, so `gcode_move._handle_ready` leaves it alone
/// (`gcode_move.py:51-58`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let bedtilt = Arc::new(BedTilt::new(config, printer)?);
    bedtilt.register_handlers(printer);
    let gcode_move = gcode_move::ensure(printer)?;
    gcode_move.set_move_transform(Arc::clone(&bedtilt) as Arc<dyn MoveTarget>, false)?;
    Ok(bedtilt)
}
