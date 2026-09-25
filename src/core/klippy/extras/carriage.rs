//! `[carriage <name>]` / `[extra_carriage <name>]` / `[dual_carriage <name>]` /
//! `[stepper <name>]` — the printer description of `kinematics:
//! generic_cartesian` (`klippy/kinematics/generic_cartesian.py`,
//! `klippy/kinematics/kinematic_stepper.py`).
//!
//! A generic-cartesian printer says what its printhead rides on and which motors
//! drive it, instead of naming one motor per axis:
//!
//! | section | what it is |
//! |---|---|
//! | `[carriage <name>]` | one carriage: the rail geometry (range, endstop, homing speeds) of a printhead, with **no motor of its own** |
//! | `[extra_carriage <name>]` | a second endstop on an existing carriage's rail (the rail-shared twin, like `carriage_z1` under `carriage_z`) |
//! | `[dual_carriage <name>]` | a second carriage on the same axis as its `primary_carriage`, with a `safe_distance` |
//! | `[stepper <name>]` | one motor, driving a **linear combination** of the carriage axes (`carriages: carriage_x+carriage_y`) |
//!
//! The section ids are split on whitespace (`[stepper a]` is id `stepper`, sub
//! `a`), so these are prefix sections: upstream's
//! `config.get_prefix_sections('carriage ')` and friends
//! (`generic_cartesian.py:173-212`), and here one factory per id.
//!
//! The one thing shared between the sections is their order: a `[carriage]`
//! declares the rail, `[dual_carriage]`/`[extra_carriage]` attach to it, and a
//! `[stepper]` names the carriages it drives — so a stepper can only resolve
//! its `carriages` expression once the carriages exist. Upstream loads them in
//! exactly that order, and the load order here is the same.
//!
//! The rail geometry is read the way upstream's `GenericPrinterRail`
//! (`klippy/stepper.py:327-392`) reads it for a **motor-less** rail: the
//! primary carriage's `position_min`/`position_max`/`position_endstop`, the
//! homing speeds, and its `endstop_pin`. The kinematics that consumes this is
//! [`motion::generic_cartesian`](crate::core::klippy::motion::generic_cartesian);
//! [`build`] is the bridge that turns the loaded sections into it, running the
//! checks upstream's `_load_kinematics` runs.

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::idex_modes;
use crate::core::klippy::extras::stepper::{
    axis_index, read_homing_info, PrinterStepper, RailGeometry, RailParams,
};
use crate::core::klippy::extras::toolhead::HomingEndstop;
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::motion::generic_cartesian::{
    generic_active_flags, generic_position_fn, GenericCartesianKinematics,
};
use crate::core::klippy::motion::{Axis, HomingInfo};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

// Upstream's `_load_kinematics` walks them in this order
// (`generic_cartesian.py:173-212`): the carriages first, then what attaches to
// them, then the steppers that name them. The kinematics itself is `[printer]`
// (order 60), which reads the result.
section!(
    "carriage",
    order = 52,
    phase = generic,
    prefix = load_main_carriage
);
// The bare `[dual_carriage]` (the cartesian IDEX module) and the prefix form
// (`[dual_carriage <name>]`, built by `load_dual_carriage` below) share one id,
// so their factories are declared together in `idex_modes` — build.rs allows one
// `section!` per id.
section!(
    "extra_carriage",
    order = 54,
    phase = generic,
    prefix = load_extra_carriage
);
section!(
    "stepper",
    order = 55,
    phase = generic,
    prefix = load_stepper
);

/// The name of the shared carriage registry (see [`CarriageModel`]).
///
/// It is upstream's `config.get_prefix_sections(...)` bookkeeping, here a
/// printer object because that is how a section factory hands data to the
/// kinematics that loads later. Not queryable: upstream has no such object.
pub const CARRIAGE_MODEL_OBJECT: &str = "carriage_model";

/// Upstream's `VALID_AXES` (`generic_cartesian.py:11`).
const VALID_AXES: [&str; 3] = ["x", "y", "z"];

/// Which section a [`Carriage`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarriageKind {
    /// `[carriage <name>]`: a primary carriage.
    Main,
    /// `[dual_carriage <name>]`: a second carriage on its primary's axis.
    Dual,
    /// `[extra_carriage <name>]`: a second endstop on its primary's rail.
    Extra,
}

/// One loaded carriage section: a rail with no motor of its own.
///
/// Upstream's `MainCarriage`/`DualCarriage`/`ExtraCarriage`
/// (`generic_cartesian.py:14-117`), which all answer the same three questions
/// the kinematics asks: the axis, the rail's range, and the endstop that stops
/// the motors driving it.
pub struct Carriage {
    /// The section's short name (`carriage_x`) — upstream's
    /// `rail.get_name(short=True)`, the name `SET_DUAL_CARRIAGE CARRIAGE=`
    /// takes and a `carriages` expression references
    /// (`stepper.py:388-393` drops the `carriage ` prefix).
    name: String,
    kind: CarriageKind,
    axis: Axis,
    params: RailParams,
    homing: HomingInfo,
    /// The carriage's own `endstop_pin`: the endstop a stepper driving this
    /// carriage arms (`GenericPrinterRail.add_stepper`).
    endstop: Arc<dyn HomingEndstop>,
    /// `primary_carriage` of a dual/extra carriage; `None` for a main one.
    primary: Option<String>,
    /// `safe_distance` of a dual carriage. A written value is kept as read; an
    /// absent one is filled in by [`build`] from both carriages' ranges
    /// (`idex_modes.py:20-26`).
    safe_distance: Mutex<Option<f64>>,
}

impl Carriage {
    /// The name a `carriages` expression and `SET_DUAL_CARRIAGE` use.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Which section this carriage came from.
    pub fn kind(&self) -> CarriageKind {
        self.kind
    }

    /// The carriage axis (`get_axis`).
    pub fn axis(&self) -> Axis {
        self.axis
    }

    /// The carriage's rail parameters (`get_rail().get_range()`): for an extra
    /// carriage these are its primary's.
    pub fn params(&self) -> RailParams {
        self.params
    }

    /// The carriage's travel range (`rail.get_range`).
    pub fn range(&self) -> (f64, f64) {
        (self.params.position_min, self.params.position_max)
    }

    /// The carriage's homing parameters (`rail.get_homing_info`).
    pub fn homing_info(&self) -> HomingInfo {
        self.homing
    }

    /// The endstop a motor driving this carriage stops on.
    pub fn endstop(&self) -> &Arc<dyn HomingEndstop> {
        &self.endstop
    }

    /// The primary this carriage attaches to, if it is a dual/extra one.
    pub fn primary_name(&self) -> Option<&str> {
        self.primary.as_deref()
    }

    /// The resolved `safe_distance`, or `None` for a carriage that has none.
    pub fn safe_distance(&self) -> Option<f64> {
        *self.lock()
    }

    fn lock(&self) -> MutexGuard<'_, Option<f64>> {
        self.safe_distance
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl std::fmt::Debug for Carriage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Carriage")
            .field("name", &self.name)
            .field("kind", &self.kind)
            .field("axis", &self.axis)
            .finish_non_exhaustive()
    }
}

impl PrinterObject for Carriage {
    /// A carriage is not a printer object upstream, so it is not in
    /// `objects/list` here either (it is only the loader's claim on the
    /// section).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// One `[stepper <name>]`: a motor driving a linear combination of carriages
/// (`KinematicStepper`, `kinematics/kinematic_stepper.py:44-85`).
pub struct KinematicStepper {
    /// The section identifier (`stepper a`) — upstream's
    /// `PrinterStepper.get_name()`, which the toolhead's stepper map is keyed by.
    name: String,
    /// The motor itself.
    stepper: Arc<PrinterStepper>,
    /// The coefficients of each axis in this motor's position
    /// (`get_kin_coeffs`).
    coeffs: [f64; 3],
    /// Which carriage carries each axis' coefficient (`get_carriages`), so the
    /// dual-carriage transform can be applied to exactly the right axis.
    refs: [Option<String>; 3],
}

impl KinematicStepper {
    /// The section identifier (`stepper a`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The motor.
    pub fn stepper(&self) -> &Arc<PrinterStepper> {
        &self.stepper
    }

    /// This motor's coefficient vector (`c0·x + c1·y + c2·z`).
    pub fn coeffs(&self) -> [f64; 3] {
        self.coeffs
    }

    /// The carriage that carries each axis' coefficient.
    pub fn carriage_for_axis(&self, axis: usize) -> Option<&str> {
        self.refs[axis].as_deref()
    }
}

impl PrinterObject for KinematicStepper {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }

    /// The motor connects itself: upstream's `stepper.PrinterStepper(config)`
    /// registers its own `klippy:connect` handler, and this section's object —
    /// the only one the loader walks — is the wrapper around it
    /// (`kinematics/kinematic_stepper.py:44-48`). Without this the host solver
    /// never exists and the toolhead refuses to bring the machine up.
    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        self.stepper.connect()
    }
}

/// The carriages and steppers of one generic-cartesian printer, collected as
/// the sections load.
///
/// The section factories each register themselves here; [`build`] reads the
/// whole picture once `[printer]` needs it. Order is the load order, which is
/// upstream's (`_load_kinematics`).
#[derive(Default)]
pub struct CarriageModel {
    carriages: Mutex<Vec<Arc<Carriage>>>,
    steppers: Mutex<Vec<Arc<KinematicStepper>>>,
}

impl CarriageModel {
    fn carriages(&self) -> Vec<Arc<Carriage>> {
        self.carriages
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// Every `[stepper <name>]`, in load order.
    pub fn steppers(&self) -> Vec<Arc<KinematicStepper>> {
        self.steppers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// The carriage that is active for `axis`: the primary carriage, which is
    /// what [`build`] activates (`idex_modes.py:27-34`). The homing driver
    /// homes this one carriage per axis.
    pub fn active_carriage(&self, axis: usize) -> Option<Arc<Carriage>> {
        self.carriages().into_iter().find(|carriage| {
            carriage.kind() == CarriageKind::Main && axis_index(carriage.axis()) == axis
        })
    }

    /// The step distance of the first motor driving `carriage_name` — the step
    /// size the homing driver polls the endstop at (`home_axis`).
    pub fn step_dist(&self, carriage_name: &str) -> Option<f64> {
        self.steppers()
            .iter()
            .find(|stepper| {
                [X_AXIS, Y_AXIS, Z_AXIS]
                    .iter()
                    .any(|axis| stepper.carriage_for_axis(*axis) == Some(carriage_name))
            })
            .map(|stepper| stepper.stepper().step_dist())
    }

    fn push_carriage(&self, carriage: Arc<Carriage>) {
        self.carriages
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push(carriage);
    }

    fn push_stepper(&self, stepper: Arc<KinematicStepper>) {
        self.steppers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push(stepper);
    }
}

impl PrinterObject for CarriageModel {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// The registry of a config that has any carriage section, created on first
/// use.
fn model(printer: &Arc<Printer>) -> Arc<CarriageModel> {
    if let Some(existing) = printer.lookup_object_as::<CarriageModel>(CARRIAGE_MODEL_OBJECT) {
        return existing;
    }
    let model = Arc::new(CarriageModel::default());
    printer
        .add_object(CARRIAGE_MODEL_OBJECT, model.clone())
        .expect("the carriage registry is registered once per machine");
    model
}

/// The registry, when the config has carriage sections.
pub fn lookup_model(printer: &Arc<Printer>) -> Option<Arc<CarriageModel>> {
    printer.lookup_object_as::<CarriageModel>(CARRIAGE_MODEL_OBJECT)
}

/// Read one carriage's rail geometry, the way upstream's `GenericPrinterRail`
/// does for a motor-less rail (`stepper.py:327-392`).
///
/// `position_min`/`position_max` are always read (upstream's default
/// `need_position_minmax=True`): a carriage is a printable range, never a bare
/// sibling.
fn read_rail(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<(RailParams, HomingInfo, Arc<dyn HomingEndstop>), ConfigError> {
    let identifier = config.identifier();
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");
    // `GenericPrinterRail.__init__` reads the pin through `config.get` — every
    // rail has an endstop (`stepper.py:336`).
    let pin = config.get("endstop_pin", None)?;
    let endstop = pins
        .setup_endstop_dyn(&pin, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    let position_min = config.get_float("position_min", Some(0.0))?;
    let position_max =
        config.get_float_bounded("position_max", None, None, None, Some(position_min), None)?;
    let position_endstop = match pins.virtual_endstop_position(&endstop) {
        Some(virtual_position) => virtual_position,
        None => config.get_float("position_endstop", None)?,
    };
    if position_endstop < position_min || position_endstop > position_max {
        return Err(ConfigError::new(format!(
            "position_endstop in section '{identifier}' must be between position_min and position_max"
        )));
    }
    let homing = read_homing_info(
        config,
        &identifier,
        position_min,
        position_max,
        position_endstop,
    )?;
    Ok((
        RailParams {
            position_min,
            position_max,
            position_endstop,
        },
        homing,
        endstop,
    ))
}

/// The carriage axis: `axis` is required unless the carriage is *named* for the
/// axis (`[carriage x]`), which is upstream's `MainCarriage.__init__`
/// (`generic_cartesian.py:16-21`).
fn read_axis(config: &ConfigWrapper, name: &str) -> Result<Axis, ConfigError> {
    let axis = if VALID_AXES.contains(&name) {
        config.get_choice("axis", &VALID_AXES, Some(name))?
    } else {
        config.get_choice("axis", &VALID_AXES, None)?
    };
    Ok(match axis.as_str() {
        "x" => Axis::X,
        "y" => Axis::Y,
        _ => Axis::Z,
    })
}

/// A section's short carriage name: the sub of `[carriage <name>]`
/// (`rail.get_name(short=True)`).
fn short_name(config: &ConfigWrapper) -> Result<String, ConfigError> {
    config.section().sub.clone().ok_or_else(|| {
        ConfigError::new(format!(
            "Section '{}' must name a carriage",
            config.identifier()
        ))
    })
}

/// The factory for `[carriage <name>]` (`MainCarriage`).
pub fn load_main_carriage(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let name = short_name(config)?;
    let axis = read_axis(config, &name)?;
    let (params, homing, endstop) = read_rail(config, printer)?;
    let carriage = Arc::new(Carriage {
        name,
        kind: CarriageKind::Main,
        axis,
        params,
        homing,
        endstop,
        primary: None,
        safe_distance: Mutex::new(None),
    });
    model(printer).push_carriage(Arc::clone(&carriage));
    Ok(carriage)
}

/// The factory for `[dual_carriage <name>]` (`DualCarriage`).
///
/// The bare `[dual_carriage]` is the cartesian IDEX module
/// ([`extras::idex_modes`](crate::core::klippy::extras::idex_modes)); this is
/// the generic-cartesian prefix form, which is a rail plus the name of the
/// primary carriage it shares its axis with.
pub fn load_dual_carriage(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let name = short_name(config)?;
    // `axis` is optional here: the real axis is the primary's, checked in
    // `build` (`resolve_primary_carriage`); only `safe_distance`'s floor is
    // read now (`generic_cartesian.py:66-71`).
    if config.section().has("axis") {
        config.get_choice("axis", &VALID_AXES, None)?;
    }
    let primary = config.get("primary_carriage", None)?;
    let (params, homing, endstop) = read_rail(config, printer)?;
    let safe_distance = if config.section().has("safe_distance") {
        Some(config.get_float_bounded("safe_distance", None, Some(0.), None, None, None)?)
    } else {
        None
    };
    let primary_axis = model(printer)
        .carriages()
        .iter()
        .find(|c| c.name() == primary)
        .map(|c| c.axis())
        .unwrap_or(Axis::X);
    let carriage = Arc::new(Carriage {
        name,
        kind: CarriageKind::Dual,
        // The primary's axis, when it has already loaded (it does: `[carriage]`
        // sections load first). A primary that never loads leaves the
        // placeholder, which `build` rejects with upstream's wording
        // (`resolve_primary_carriage`, `generic_carriages.py:89-116`).
        axis: primary_axis,
        params,
        homing,
        endstop,
        primary: Some(primary),
        safe_distance: Mutex::new(safe_distance),
    });
    model(printer).push_carriage(Arc::clone(&carriage));
    Ok(carriage)
}

/// The factory for `[extra_carriage <name>]` (`ExtraCarriage`).
///
/// An extra carriage owns no motor and no geometry of its own: it is a second
/// endstop on its primary's rail (`generic_cartesian.py:41-54`), whose range and
/// homing come from the primary.
pub fn load_extra_carriage(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let name = short_name(config)?;
    let identifier = config.identifier();
    let primary = config.get("primary_carriage", None)?;
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");
    let pin = config.get("endstop_pin", None)?;
    let endstop = pins
        .setup_endstop_dyn(&pin, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    // An extra carriage has neither geometry nor an axis of its own: both are
    // the primary's (`ExtraCarriage.get_axis`/`get_rail`).
    let primary_carriage = model(printer)
        .carriages()
        .iter()
        .find(|c| c.name() == primary)
        .cloned();
    let carriage = Arc::new(Carriage {
        name,
        kind: CarriageKind::Extra,
        axis: primary_carriage
            .as_ref()
            .map(|c| c.axis())
            .unwrap_or(Axis::X),
        params: primary_carriage
            .as_ref()
            .map(|c| c.params())
            .unwrap_or_default(),
        homing: primary_carriage
            .as_ref()
            .map(|c| c.homing_info())
            .unwrap_or_default(),
        endstop,
        primary: Some(primary),
        safe_distance: Mutex::new(None),
    });
    model(printer).push_carriage(Arc::clone(&carriage));
    Ok(carriage)
}

/// The factory for `[stepper <name>]` (`KinematicStepper`): the motor, plus the
/// carriage expression saying what it drives.
pub fn load_stepper(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let name = config.identifier();
    let expression = config.get("carriages", None)?;
    let model = model(printer);
    let carriages = model.carriages();
    let names: Vec<(String, Axis)> = carriages
        .iter()
        .map(|carriage| (carriage.name().to_string(), carriage.axis()))
        .collect();
    let (coeffs, referenced) =
        parse_carriages_string(&expression, &names).map_err(|message| ConfigError::new(message))?;
    // The axis is only the bare-motor fallback; the kinematics installs this
    // stepper's real solver (`setup_itersolve`) from the coefficients.
    let axis = [Axis::X, Axis::Y, Axis::Z]
        .into_iter()
        .zip(coeffs)
        .find(|(_, coeff)| *coeff != 0.0)
        .map(|(axis, _)| axis)
        .unwrap_or(Axis::X);
    let stepper = Arc::new(PrinterStepper::with_geometry(
        config,
        printer,
        axis,
        RailGeometry::BareMotor,
    )?);
    // The owning kinematics installs the solver; doing it here keeps the
    // generic position function and the axes it moves together with the
    // coefficients that produced them (`generic_cartesian_stepper_set_coeffs`).
    stepper.setup_itersolve(generic_position_fn(coeffs), generic_active_flags(coeffs));
    // A motor arms the endstop of every carriage it drives
    // (`GenericPrinterRail.add_stepper`), so a homing move stops it.
    let carriage_objects: Vec<Arc<Carriage>> = referenced
        .iter()
        .filter_map(|name| carriages.iter().find(|c| c.name() == name).cloned())
        .collect();
    for carriage in &carriage_objects {
        register_with_carriage_endstop(&stepper, carriage)?;
    }
    let mut refs: [Option<String>; 3] = [None, None, None];
    for carriage in carriage_objects {
        refs[axis_index(carriage.axis())] = Some(carriage.name().to_string());
    }
    let kinematic = Arc::new(KinematicStepper {
        name,
        stepper,
        coeffs,
        refs,
    });
    model.push_stepper(Arc::clone(&kinematic));
    Ok(kinematic)
}

/// Add a motor to the trigger dispatch of the endstop that guards its carriage
/// (`mcu_endstop.add_stepper`, `stepper.py:444`).
fn register_with_carriage_endstop(
    stepper: &Arc<PrinterStepper>,
    carriage: &Arc<Carriage>,
) -> Result<(), ConfigError> {
    let identifier = carriage.name().to_string();
    let endstop = carriage.endstop().clone();
    let dispatch = endstop.dispatch().ok_or_else(|| {
        ConfigError::new(format!(
            "carriage '{identifier}' must name an endstop that drives a trigger dispatch"
        ))
    })?;
    let mcu_stepper = stepper.mcu_stepper();
    dispatch
        .add_stepper(
            mcu_stepper.chip().clone(),
            Arc::downgrade(mcu_stepper),
            stepper.name(),
        )
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    Ok(())
}

/// Parse a `carriages` expression (`parse_carriages_string`,
/// `kinematics/kinematic_stepper.py:10-42`).
///
/// Terms are separated by `+`/`-` and may carry a `*coefficient`; each names a
/// carriage, and a carriage's axis may appear only once. Returns the coefficient
/// vector and the referenced carriages, in expression order.
fn parse_carriages_string(
    expression: &str,
    carriages: &[(String, Axis)],
) -> Result<([f64; 3], Vec<String>), String> {
    let chars: Vec<char> = expression.chars().collect();
    let mut coeffs = [0.0; 3];
    let mut referenced: Vec<String> = Vec::new();
    let mut next = 0usize;
    while next < chars.len() {
        // The first `+`/`-` **after** `next` ends the term: a leading sign is
        // part of the term (upstream searches from `nxt+1`).
        let end = chars
            .iter()
            .enumerate()
            .skip(next + 1)
            .find(|(_, c)| **c == '+' || **c == '-')
            .map(|(index, _)| index)
            .unwrap_or(chars.len());
        let term: String = chars[next..end].iter().collect();
        let term = term.trim();
        let parts: Vec<&str> = term.split('*').collect();
        if parts.len() != 1 && parts.len() != 2 {
            return Err(format!("Invalid term '{term}' in '{expression}'"));
        }
        let coeff = if parts.len() == 2 {
            parts[0]
                .trim()
                .parse::<f64>()
                .map_err(|_| format!("Invalid float '{}'", parts[0].trim()))?
        } else if parts[0].starts_with('-') {
            -1.0
        } else {
            1.0
        };
        let mut carriage_name = *parts.last().expect("a term has a carriage");
        if parts.len() == 1 && (carriage_name.starts_with('-') || carriage_name.starts_with('+')) {
            carriage_name = &carriage_name[1..];
        }
        let Some((_, axis)) = carriages.iter().find(|(name, _)| name == carriage_name) else {
            return Err(format!(
                "Invalid '{carriage_name}' carriage referenced in '{expression}'"
            ));
        };
        let axis = axis_index(*axis);
        if coeffs[axis] != 0.0 {
            return Err(format!(
                "Axis '{}' was referenced multiple times by carriages in '{expression}'",
                VALID_AXES[axis]
            ));
        }
        coeffs[axis] = coeff;
        referenced.push(carriage_name.to_string());
        next = end;
    }
    Ok((coeffs, referenced))
}

/// What [`build`] hands the toolhead: the kinematics and the motors it drives.
pub struct GenericCartesianConfig {
    /// The built kinematics.
    pub kinematics: GenericCartesianKinematics,
    /// The `[stepper <name>]` sections, in load order.
    pub steppers: Vec<Arc<KinematicStepper>>,
}

/// Turn the loaded carriage/stepper sections into the kinematics, running
/// upstream's `_load_kinematics` checks (`generic_cartesian.py:173-263`).
///
/// `max_z_velocity`/`max_z_accel` are `[printer]`'s Z caps, which upstream's
/// `GenericCartesianKinematics.__init__` reads from the same section
/// (`generic_cartesian.py:162-166`); its `check_move` limits every Z move by
/// them.
///
/// # Errors
/// Upstream's wording for a duplicated or missing primary carriage, a bad
/// `primary_carriage`, two duals on one carriage, a carriage no stepper drives,
/// and a coefficient matrix that cannot move the axes independently.
pub fn build(
    printer: &Arc<Printer>,
    max_z_velocity: f64,
    max_z_accel: f64,
) -> Result<GenericCartesianConfig, ConfigError> {
    let model = lookup_model(printer).ok_or_else(|| {
        ConfigError::new(
            "kinematics 'generic_cartesian' needs '[carriage <name>]' sections".to_string(),
        )
    })?;
    let carriages = model.carriages();
    let mains: Vec<&Arc<Carriage>> = carriages
        .iter()
        .filter(|c| c.kind() == CarriageKind::Main)
        .collect();

    // One primary carriage per axis, and no axis without one
    // (`generic_cartesian.py:174-185`).
    for (axis, axis_name) in VALID_AXES.iter().enumerate() {
        let dups: Vec<&str> = mains
            .iter()
            .filter(|c| axis_index(c.axis()) == axis)
            .map(|c| c.name())
            .collect();
        if dups.len() > 1 {
            return Err(ConfigError::new(format!(
                "Axis '{axis_name}' is set for multiple primary carriages ({})",
                dups.join(", ")
            )));
        }
        if dups.is_empty() {
            return Err(ConfigError::new(format!(
                "No carriage defined for axis '{axis_name}'"
            )));
        }
    }

    // The name → carriage map, primaries and duals first, then the extras
    // (`generic_carriages` + `carriages`), with upstream's redefinition check.
    let mut by_name: Vec<Arc<Carriage>> = Vec::new();
    for carriage in carriages.iter().filter(|c| c.kind() != CarriageKind::Extra) {
        if by_name.iter().any(|c| c.name() == carriage.name()) {
            return Err(ConfigError::new(format!(
                "Redefinition of carriage {}",
                carriage.name()
            )));
        }
        by_name.push(Arc::clone(carriage));
    }
    // A dual carriage resolves its primary, and its axis is the primary's
    // (`resolve_primary_carriage`).
    let duals: Vec<Arc<Carriage>> = by_name
        .iter()
        .filter(|c| c.kind() == CarriageKind::Dual)
        .cloned()
        .collect();
    let mut resolved_axes: Vec<(String, Axis)> = Vec::new();
    for dual in &duals {
        let primary_name = dual.primary_name().unwrap_or_default();
        let Some(primary) = by_name.iter().find(|c| c.name() == primary_name) else {
            return Err(ConfigError::new(format!(
                "primary_carriage = '{primary_name}' for '{}' is not a valid choice",
                dual.name()
            )));
        };
        if let Some((other, _)) = resolved_axes
            .iter()
            .find(|(_, axis)| *axis == primary.axis())
        {
            return Err(ConfigError::new(format!(
                "Multiple dual carriages ('{other}', '{}') for carriage '{}'",
                dual.name(),
                primary.name()
            )));
        }
        if axis_index(primary.axis()) > Y_AXIS {
            return Err(ConfigError::new(format!(
                "Invalid axis '{}' for dual_carriage '{}'",
                VALID_AXES[axis_index(primary.axis())],
                dual.name()
            )));
        }
        resolved_axes.push((dual.name().to_string(), primary.axis()));
    }
    // The extras hang off the same map, checked the same way.
    for carriage in carriages.iter().filter(|c| c.kind() == CarriageKind::Extra) {
        let primary_name = carriage.primary_name().unwrap_or_default();
        if !by_name.iter().any(|c| c.name() == primary_name) {
            return Err(ConfigError::new(format!(
                "primary_carriage = '{primary_name}' for '{}' is not a valid choice",
                carriage.name()
            )));
        }
        if by_name.iter().any(|c| c.name() == carriage.name()) {
            return Err(ConfigError::new(format!(
                "Redefinition of carriage {}",
                carriage.name()
            )));
        }
        by_name.push(Arc::clone(carriage));
    }

    let steppers = model.steppers();
    // Every carriage must be driven by some stepper
    // (`_check_carriages_references`, `:213-221`).
    let unreferenced: Vec<&str> = by_name
        .iter()
        .filter(|carriage| {
            !steppers.iter().any(|stepper| {
                [X_AXIS, Y_AXIS, Z_AXIS]
                    .iter()
                    .any(|axis| stepper.carriage_for_axis(*axis) == Some(carriage.name()))
            })
        })
        .map(|carriage| carriage.name())
        .collect();
    if !unreferenced.is_empty() {
        return Err(ConfigError::new(format!(
            "Carriage(s) {} must be referenced by some stepper(s)",
            unreferenced.join(", ")
        )));
    }

    // The load-time matrix: each axis follows the carriage that is active for
    // it, which is the primary carriage — upstream activates the first carriage
    // of each dual-carriage axis as it builds the module
    // (`generic_cartesian.py:143-168`, `idex_modes.py:27-34`). Our port has no
    // live dual-carriage transform (the `[dual_carriage]` handover is the
    // coordinate anchor of `extras::idex_modes`), so a coefficient that belongs
    // to an inactive carriage contributes nothing — which is the load-time
    // state upstream checks.
    let active: [&str; 3] = [
        mains
            .iter()
            .find(|c| axis_index(c.axis()) == X_AXIS)
            .map(|c| c.name())
            .unwrap_or(""),
        mains
            .iter()
            .find(|c| axis_index(c.axis()) == Y_AXIS)
            .map(|c| c.name())
            .unwrap_or(""),
        mains
            .iter()
            .find(|c| axis_index(c.axis()) == Z_AXIS)
            .map(|c| c.name())
            .unwrap_or(""),
    ];
    let mut rows: Vec<(String, [f64; 3])> = Vec::with_capacity(steppers.len());
    for stepper in &steppers {
        let mut coeffs = stepper.coeffs();
        for axis in [X_AXIS, Y_AXIS, Z_AXIS] {
            if stepper.carriage_for_axis(axis) != Some(active[axis]) {
                coeffs[axis] = 0.0;
            }
        }
        rows.push((stepper.name().to_string(), coeffs));
    }

    let ranges = |axis: usize| -> (f64, f64) {
        let lows = by_name
            .iter()
            .filter(|c| axis_index(c.axis()) == axis)
            .map(|c| c.range().0)
            .fold(f64::INFINITY, f64::min);
        let highs = by_name
            .iter()
            .filter(|c| axis_index(c.axis()) == axis)
            .map(|c| c.range().1)
            .fold(f64::NEG_INFINITY, f64::max);
        (lows, highs)
    };
    let active_ranges: [(f64, f64); 3] = [
        mains
            .iter()
            .find(|c| axis_index(c.axis()) == X_AXIS)
            .map(|c| c.range())
            .unwrap_or((0.0, 0.0)),
        mains
            .iter()
            .find(|c| axis_index(c.axis()) == Y_AXIS)
            .map(|c| c.range())
            .unwrap_or((0.0, 0.0)),
        mains
            .iter()
            .find(|c| axis_index(c.axis()) == Z_AXIS)
            .map(|c| c.range())
            .unwrap_or((0.0, 0.0)),
    ];
    let (x_min, x_max) = ranges(X_AXIS);
    let (y_min, y_max) = ranges(Y_AXIS);
    let (z_min, z_max) = ranges(Z_AXIS);

    // A written `safe_distance` stays; an absent one falls back to the closest
    // pair of carriage limits (`idex_modes.py:20-26`).
    for dual in &duals {
        if dual.safe_distance().is_none() {
            if let Some(primary) = by_name
                .iter()
                .find(|c| c.name() == dual.primary_name().unwrap_or_default())
            {
                let (p_min, p_max) = primary.range();
                let (d_min, d_max) = dual.range();
                *dual
                    .safe_distance
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner()) =
                    Some((p_min - d_min).abs().min((p_max - d_max).abs()));
            }
        }
    }

    let kinematics = GenericCartesianKinematics::new(
        rows,
        active_ranges,
        Coord::new(x_min, y_min, z_min, 0.0),
        Coord::new(x_max, y_max, z_max, 0.0),
        max_z_velocity,
        max_z_accel,
    );
    if !kinematics.check_kinematics() {
        return Err(ConfigError::new(
            "Verify configured stepper(s) and their 'carriages' specifications, the current \
             configuration does not allow independent movements of all printer axes."
                .to_string(),
        ));
    }

    // A machine with dual carriages builds the idex module the way upstream's
    // kinematics does: `GenericCartesianKinematics.__init__` constructs
    // `idex_modes.DualCarriages` when the config carries
    // `[dual_carriage <name>]` sections, and that constructor registers the
    // `dual_carriage` object and the three `SET_DUAL_CARRIAGE` /
    // `SAVE_DUAL_CARRIAGE_STATE` / `RESTORE_DUAL_CARRIAGE_STATE` commands
    // (`generic_cartesian.py:137-146`). The carriage order is upstream's
    // `dc_rails` (`idex_modes.py:37-45`): the primary carriage of every dual
    // axis first, then the dual carriages themselves.
    if !duals.is_empty() {
        let dc_axes: Vec<Axis> = duals.iter().map(|dual| dual.axis()).collect();
        let mut idex_carriages: Vec<(String, Axis)> = mains
            .iter()
            .filter(|main| dc_axes.contains(&main.axis()))
            .map(|main| (main.name().to_string(), main.axis()))
            .collect();
        idex_carriages.extend(
            duals
                .iter()
                .map(|dual| (dual.name().to_string(), dual.axis())),
        );
        idex_modes::register_generic(printer, &idex_carriages)?;
    }

    Ok(GenericCartesianConfig {
        kinematics,
        steppers,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tests_carriages() -> Vec<(String, Axis)> {
        vec![
            ("carriage_x".to_string(), Axis::X),
            ("carriage_y".to_string(), Axis::Y),
        ]
    }

    #[test]
    fn a_carriages_expression_is_a_linear_combination() {
        let carriages = tests_carriages();
        let (coeffs, referenced) =
            parse_carriages_string("carriage_x+carriage_y", &carriages).unwrap();
        assert_eq!(coeffs, [1.0, 1.0, 0.0]);
        assert_eq!(referenced, vec!["carriage_x", "carriage_y"]);

        let (coeffs, _) = parse_carriages_string("carriage_x-carriage_y", &carriages).unwrap();
        assert_eq!(coeffs, [1.0, -1.0, 0.0]);

        let (coeffs, _) =
            parse_carriages_string("2*carriage_x-0.5*carriage_y", &carriages).unwrap();
        assert_eq!(coeffs, [2.0, -0.5, 0.0]);
    }

    #[test]
    fn a_repeated_axis_in_the_expression_is_refused() {
        let carriages = tests_carriages();
        let err = parse_carriages_string("carriage_x+carriage_x", &carriages).unwrap_err();
        assert_eq!(
            err,
            "Axis 'x' was referenced multiple times by carriages in 'carriage_x+carriage_x'"
        );
    }

    #[test]
    fn an_unknown_carriage_name_is_refused() {
        let carriages = vec![("carriage_x".to_string(), Axis::X)];
        let err = parse_carriages_string("nope", &carriages).unwrap_err();
        assert_eq!(err, "Invalid 'nope' carriage referenced in 'nope'");

        let err = parse_carriages_string("1.0.0*carriage_x", &carriages).unwrap_err();
        assert_eq!(err, "Invalid float '1.0.0'");
    }

    /// The corpus' generic-cartesian machine (`corexyuv.cfg`) loaded through
    /// the real loader — every section (`[carriage]`, `[extra_carriage]`,
    /// `[dual_carriage]`, `[stepper a]`…) claimed, every option read
    /// (`check_unused`), with the MCU transport swapped for the fake
    /// firmware's dictionary.
    fn load_corexyuv() -> Arc<Printer> {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::printer::Printer;
        use crate::core::klippy::reactor::ManualReactor;

        let path = klipperx_test_support::klipper_dir().join("test/klippy/corexyuv.cfg");
        let text =
            std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let text = text.replace(
            "[mcu]\nserial: /dev/ttyACM0",
            &format!("[mcu]\ntest: dict={}", dict.display()),
        );
        let (config, _) = Config::from_text(&text).expect("corexyuv.cfg parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("corexyuv.cfg loads");
        printer
    }

    #[test]
    fn the_corexyuv_config_loads_and_passes_check_unused() {
        load_corexyuv();
    }

    /// The three idex commands are known commands on a generic-cartesian
    /// machine: upstream builds `idex_modes.DualCarriages` inside
    /// `GenericCartesianKinematics.__init__` (`generic_cartesian.py:137-146`)
    /// instead of loading a `[dual_carriage]` section, and that constructor
    /// registers them. The corpus drives them by carriage **name**
    /// (`corexyuv.test:14-30`), which is what lands here.
    #[test]
    fn the_generic_cartesian_kinematics_registers_the_idex_commands() {
        use crate::core::klippy::event::KlippyEvent;
        use crate::core::klippy::extras::idex_modes::{GenericDualCarriages, DUAL_CARRIAGE_OBJECT};
        use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};

        let printer = load_corexyuv();
        // A successful load fires `klippy:ready`, so the dispatcher runs
        // scripts (`gcode.rs:492-496`, as idex_modes' tests do).
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let help = gcode.command_help();
        for (name, desc) in [
            ("SET_DUAL_CARRIAGE", "Configure the dual carriages mode"),
            (
                "SAVE_DUAL_CARRIAGE_STATE",
                "Save dual carriages modes and positions",
            ),
            (
                "RESTORE_DUAL_CARRIAGE_STATE",
                "Restore dual carriages modes and positions",
            ),
        ] {
            assert_eq!(help.get(name).map(String::as_str), Some(desc), "{name}");
        }

        // `dc_rails` order (`generic_cartesian.py:138-142`): the primary
        // carriage of each dual axis, then the dual carriages.
        let module = printer
            .lookup_object_as::<GenericDualCarriages>(DUAL_CARRIAGE_OBJECT)
            .expect("the dual_carriage object is registered");
        assert_eq!(
            module.carriage_names(),
            ["carriage_x", "carriage_y", "carriage_u", "carriage_v"]
        );

        // The names the corpus passes pick their carriage
        // (`corexyuv.test:14-26`), and a four-carriage machine has no `0`/`1`
        // index fallback (`idex_modes.py:247-254`).
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=carriage_u")
            .unwrap();
        assert_eq!(module.active_carriage(), 2);
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=carriage_x")
            .unwrap();
        assert_eq!(module.active_carriage(), 0);
        let err = gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=0")
            .unwrap_err();
        assert_eq!(err.to_string(), "Invalid CARRIAGE=0 specified");

        // A save carries the active carriage across a switch
        // (`corexyuv.test:28-38`).
        gcode.run_script_sync("SAVE_DUAL_CARRIAGE_STATE").unwrap();
        gcode
            .run_script_sync("SET_DUAL_CARRIAGE CARRIAGE=carriage_v")
            .unwrap();
        assert_eq!(module.active_carriage(), 3);
        gcode
            .run_script_sync("RESTORE_DUAL_CARRIAGE_STATE")
            .unwrap();
        assert_eq!(module.active_carriage(), 0, "the saved index is restored");
    }

    /// A three-axis carriage printer, and nothing else — the tests below vary
    /// the carriage/stepper sections to reach the load-time checks.
    fn load_error(carriages: &str, steppers: &str) -> String {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::printer::Printer;
        use crate::core::klippy::reactor::ManualReactor;

        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let text = format!(
            "[mcu]\ntest: dict={}\n{carriages}{steppers}\n\
             [printer]\nkinematics: generic_cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
            dict.display()
        );
        let (config, _) = Config::from_text(&text).expect("the test config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        match printer.load_config(&config) {
            Ok(()) => panic!("the config loads, but a load error was expected"),
            Err(error) => error.to_string(),
        }
    }

    /// The three primary carriages the tests below share, and a stepper for
    /// each axis: `x+y`/`x-y` drive x and y independently, so the matrix is
    /// only singular when a test makes it so.
    const CARRIAGES: &str = "[carriage x]\naxis: x\nposition_endstop: 0\nposition_max: 300\n\
         endstop_pin: ^PE5\n\
         [carriage y]\naxis: y\nposition_endstop: 0\nposition_max: 200\nendstop_pin: ^PJ1\n\
         [carriage z]\naxis: z\nposition_endstop: 0\nposition_max: 100\nendstop_pin: ^PD3\n";
    const STEPPERS: &str =
        "[stepper a]\ncarriages: x+y\nstep_pin: PF0\ndir_pin: PF1\nmicrosteps: 16\n\
         rotation_distance: 40\n\
         [stepper c]\ncarriages: x-y\nstep_pin: PF6\ndir_pin: !PF7\nmicrosteps: 16\n\
         rotation_distance: 40\n\
         [stepper z]\ncarriages: z\nstep_pin: PL3\ndir_pin: PL1\nmicrosteps: 16\n\
         rotation_distance: 8\n";

    #[test]
    fn a_second_primary_carriage_on_an_axis_is_refused() {
        let carriages = format!(
            "{CARRIAGES}[carriage x2]\naxis: x\nposition_endstop: 0\nposition_max: 300\n\
             endstop_pin: ^PE6\n"
        );
        assert_eq!(
            load_error(&carriages, STEPPERS),
            "Axis 'x' is set for multiple primary carriages (x, x2)"
        );
    }

    #[test]
    fn an_axis_without_a_primary_carriage_is_refused() {
        let carriages = "[carriage x]\naxis: x\nposition_endstop: 0\nposition_max: 300\n\
             endstop_pin: ^PE5\n\
             [carriage y]\naxis: y\nposition_endstop: 0\nposition_max: 200\nendstop_pin: ^PJ1\n";
        assert_eq!(
            load_error(
                carriages,
                "[stepper a]\ncarriages: x+y\nstep_pin: PF0\ndir_pin: PF1\nmicrosteps: 16\n\
                 rotation_distance: 40\n\
                 [stepper c]\ncarriages: x-y\nstep_pin: PF6\ndir_pin: !PF7\nmicrosteps: 16\n\
                 rotation_distance: 40\n"
            ),
            "No carriage defined for axis 'z'"
        );
    }

    #[test]
    fn a_singular_coefficient_matrix_is_refused() {
        // Both motors drive the same combination, so x and y cannot move
        // independently.
        let steppers = "[stepper a]\ncarriages: x+y\nstep_pin: PF0\ndir_pin: PF1\nmicrosteps: 16\n\
             rotation_distance: 40\n\
             [stepper c]\ncarriages: x+y\nstep_pin: PF6\ndir_pin: !PF7\nmicrosteps: 16\n\
             rotation_distance: 40\n\
             [stepper z]\ncarriages: z\nstep_pin: PL3\ndir_pin: PL1\nmicrosteps: 16\n\
             rotation_distance: 8\n";
        assert_eq!(
            load_error(CARRIAGES, steppers),
            "Verify configured stepper(s) and their 'carriages' specifications, the current \
             configuration does not allow independent movements of all printer axes."
        );
    }

    #[test]
    fn a_carriage_no_stepper_drives_is_refused() {
        // A fourth carriage nothing references (`_check_carriages_references`).
        let carriages =
            format!("{CARRIAGES}[extra_carriage z1]\nprimary_carriage: z\nendstop_pin: ^PD2\n");
        assert_eq!(
            load_error(&carriages, STEPPERS),
            "Carriage(s) z1 must be referenced by some stepper(s)"
        );
    }
}
