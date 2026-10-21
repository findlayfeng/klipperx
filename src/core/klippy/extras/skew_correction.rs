//! `[skew_correction]` — correct a skewed XY frame with a three-plane
//! transform (upstream `klippy/extras/skew_correction.py`).
//!
//! The section reads **no options** (`skew_correction.py:23-47`); what it does
//! is keep three skew factors (one per plane) and claim `gcode_move`'s
//! move-transform slot ([`MoveTarget`]) so every `G1`/`M114` passes through the
//! matrix:
//!
//! - [`SkewCorrection::move_to`] (upstream's `move`) **skews** the g-code
//!   position before the chain below queues it;
//! - [`SkewCorrection::position`] (upstream's `get_position`) **un-skews** the
//!   toolhead's position, so the g-code space reads square.
//!
//! The matrix is upstream's (`skew_correction.py:62-71`), with `xy_factor`,
//! `xz_factor` and `yz_factor`:
//!
//! ```text
//! skewed_x = x - y*xy_factor - z*(xz_factor - xy_factor*yz_factor)
//! skewed_y = y - z*yz_factor
//! ```
//!
//! and the inverse adds the same terms back (`:72-77`).
//!
//! | command | role (`skew_correction.py`) |
//! |---|---|
//! | `GET_CURRENT_SKEW` | report the three factors in radians and degrees (`:83-95`) |
//! | `CALC_MEASURED_SKEW` | turn one measured plane's `AC`/`BD`/`AD` into a factor (`:96-101`) |
//! | `SET_SKEW` | set the factors from measured lengths per plane, or `CLEAR=1` (`:102-120`) |
//! | `SKEW_PROFILE` | `SAVE=`/`LOAD=`/`REMOVE=` a stored profile (`:121-165`) |
//!
//! A profile is a `[skew_correction <name>]` section holding `xy_skew`,
//! `xz_skew`, `yz_skew`; `SAVE=` queues it through the `configfile` object for
//! the next `SAVE_CONFIG`, the same way `BED_MESH_PROFILE` queues a mesh.
//!
//! # The transform chain
//!
//! Upstream takes the slot at `klippy:connect` with `force=True` and keeps what
//! it displaced as `next_transform` (`skew_correction.py:48-50`), so `bed_mesh`
//! or `bed_tilt` keep working underneath. This port chains the same way
//! (`exclude_object.rs` is the other user of that shape): a move is skewed and
//! then forwarded, a position is read from the chain below and un-skewed. When
//! the slot was still empty the chain's end is the toolhead captured at connect,
//! because an empty slot answers `None` here rather than upstream's toolhead.
//!
//! Two upstream details are kept as they are: `LOAD=` records the requested
//! name *before* it checks that the profile exists (`:123-125`), and `SET_SKEW`
//! writes the factors without re-anchoring `gcode_move`'s `last_position`
//! (`:111-120` — only `_update_skew`, `:78-82`, does that).

use std::f64::consts::PI;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_move::{self, GCodeMove, MoveTarget, GCODE_MOVE_OBJECT};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, X_AXIS, Y_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("skew_correction", order = 30, load = load_config);

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The section's own name — upstream's `config.get_name()`, which both the
/// profile sections (`"<name> <profile>"`) and nothing else derive from
/// (`skew_correction.py:51`).
const SECTION: &str = "skew_correction";

/// The planes a factor belongs to, in the order every report lists them
/// (`skew_correction.py:86`).
const PLANES: [&str; 3] = ["XY", "XZ", "YZ"];

/// The `SKEW_PROFILE` verbs, in the order the command tries them
/// (`skew_correction.py:122-157`).
const PROFILE_VERBS: [&str; 3] = ["LOAD", "SAVE", "REMOVE"];

/// One stored profile — a `[skew_correction <name>]` section's three floats
/// (`skew_correction.py:55-59`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct SkewProfile {
    /// The XY-plane factor.
    xy_skew: f64,
    /// The XZ-plane factor.
    xz_skew: f64,
    /// The YZ-plane factor.
    yz_skew: f64,
}

/// The live correction: the three factors and the profile last asked for
/// (`skew_correction.py:26-29`).
#[derive(Debug, Clone, PartialEq)]
struct SkewState {
    /// `xy_factor`.
    xy_factor: f64,
    /// `xz_factor`.
    xz_factor: f64,
    /// `yz_factor`.
    yz_factor: f64,
    /// `current_profile_name`, reported by `get_status`.
    current_profile_name: String,
}

impl Default for SkewState {
    fn default() -> Self {
        Self {
            xy_factor: 0.0,
            xz_factor: 0.0,
            yz_factor: 0.0,
            current_profile_name: String::new(),
        }
    }
}

/// The `[skew_correction]` section (`skew_correction.py:PrinterSkew`).
pub struct SkewCorrection {
    /// The machine, for `gcode_move`/`configfile` at run time.
    printer: Weak<Printer>,
    /// The factors and the profile name, behind one lock so a status query and
    /// a move cannot see a half-updated set.
    state: Mutex<SkewState>,
    /// The stored profiles, in config order — upstream's `skew_profiles` dict
    /// (`skew_correction.py:29`), which only ever gets looked up, replaced or
    /// removed.
    profiles: Mutex<Vec<(String, SkewProfile)>>,
    /// The toolhead captured at `klippy:connect`, the chain's end when the slot
    /// was empty (`skew_correction.py` keeps it as `self.toolhead`).
    toolhead: Mutex<Option<Arc<dyn MoveTarget>>>,
    /// The transform this one displaced from `gcode_move`'s slot (upstream
    /// `next_transform`, `skew_correction.py:50`).
    next_transform: Mutex<Option<Arc<dyn MoveTarget>>>,
}

impl std::fmt::Debug for SkewCorrection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.state();
        f.debug_struct("SkewCorrection")
            .field("xy_factor", &state.xy_factor)
            .field("xz_factor", &state.xz_factor)
            .field("yz_factor", &state.yz_factor)
            .field("current_profile_name", &state.current_profile_name)
            .field("profiles", &self.profile_names())
            .finish_non_exhaustive()
    }
}

impl SkewCorrection {
    /// Read the section: no options, but the `[skew_correction <name>]`
    /// profiles the last `SAVE_CONFIG` wrote (`skew_correction.py:23-32`,
    /// `:51-60`).
    ///
    /// # Errors
    /// A profile section whose `xy_skew`/`xz_skew`/`yz_skew` is absent or not
    /// a float.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let profiles = read_profiles(config)?;
        Ok(Self {
            printer: Arc::downgrade(printer),
            state: Mutex::new(SkewState::default()),
            profiles: Mutex::new(profiles),
            toolhead: Mutex::new(None),
            next_transform: Mutex::new(None),
        })
    }

    /// The state lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock(&self) -> MutexGuard<'_, SkewState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The state, copied out of its lock.
    fn state(&self) -> SkewState {
        self.lock().clone()
    }

    /// The stored profile names, in config order.
    fn profile_names(&self) -> Vec<String> {
        self.profiles
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .iter()
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// One stored profile, copied out of its lock.
    fn profile(&self, name: &str) -> Option<SkewProfile> {
        self.profiles
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .iter()
            .find(|(stored, _)| stored == name)
            .map(|(_, profile)| *profile)
    }

    /// Store a profile under `name`, replacing one of the same name
    /// (`skew_profiles[name] = {...}`, `skew_correction.py:139-143`).
    fn store_profile(&self, name: &str, profile: SkewProfile) {
        let mut profiles = self
            .profiles
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        match profiles.iter_mut().find(|(stored, _)| stored == name) {
            Some((_, stored)) => *stored = profile,
            None => profiles.push((name.to_string(), profile)),
        }
    }

    /// Forget a profile; whether one was there is the caller's answer
    /// (`skew_correction.py:151-152`).
    fn remove_profile(&self, name: &str) -> bool {
        let mut profiles = self
            .profiles
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        match profiles.iter().position(|(stored, _)| stored == name) {
            Some(index) => {
                profiles.remove(index);
                true
            }
            None => false,
        }
    }

    /// The events upstream's `__init__` subscribes to
    /// (`skew_correction.py:31-32`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.handle_connect()
            }),
        );
    }

    /// Upstream's `_handle_connect` (`skew_correction.py:48-50`): take the
    /// move-transform slot, keeping the occupant it displaces as the chain
    /// below this transform, and the toolhead as the chain's end.
    fn handle_connect(self: &Arc<Self>) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
            *self.toolhead.lock().unwrap_or_else(|p| p.into_inner()) =
                Some(Arc::new(ToolheadMove(toolhead)));
        }
        let Some(gcode_move) = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT) else {
            return;
        };
        // `set_move_transform(self, force=True)`: whatever held the slot
        // (`bed_mesh`/`bed_tilt`, when one of them registered first) becomes
        // the transform the skewed moves go through.
        let displaced =
            gcode_move.set_move_transform(Arc::clone(self) as Arc<dyn MoveTarget>, true);
        if let Ok(previous) = displaced {
            if let Some(previous) = previous {
                *self
                    .next_transform
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = Some(previous);
            }
        }
    }

    /// The transform chain below this one: the displaced occupant once
    /// connected, the connect-time toolhead before that.
    fn target(&self) -> Option<Arc<dyn MoveTarget>> {
        self.next_transform
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
            .or_else(|| {
                self.toolhead
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .clone()
            })
    }

    /// Upstream's `calc_skew` (`skew_correction.py:62-67`): the g-code position
    /// as the skewed machine frame sees it.
    fn calc_skew(&self, position: Coord) -> Coord {
        let state = self.state();
        skewed(position, &state)
    }

    /// Upstream's `calc_unskew` (`skew_correction.py:72-77`): the machine
    /// position as the square g-code frame sees it.
    fn calc_unskew(&self, position: Coord) -> Coord {
        let state = self.state();
        unskewed(position, &state)
    }

    /// Upstream's `_update_skew` (`skew_correction.py:78-82`): store the three
    /// factors and re-anchor `gcode_move`'s `last_position`, which was anchored
    /// to the old matrix.
    fn update_skew(&self, xy_factor: f64, xz_factor: f64, yz_factor: f64) {
        {
            let mut state = self.lock();
            state.xy_factor = xy_factor;
            state.xz_factor = xz_factor;
            state.yz_factor = yz_factor;
        }
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if let Some(gcode_move) = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT) {
            gcode_move.reset_last_position();
        }
    }

    /// The `configfile` object, when the machine has one — every module that
    /// queues a `SAVE_CONFIG` value treats an absent one as "session only".
    fn configfile(&self) -> Option<Arc<PrinterConfig>> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT))
    }

    /// `GET_CURRENT_SKEW` (`skew_correction.py:83-95`): the three factors in
    /// radians and degrees.
    fn cmd_get_current_skew(&self, gcmd: &GcodeCommand) {
        let state = self.state();
        let factors = [state.xy_factor, state.xz_factor, state.yz_factor];
        let mut out = String::from("Current Printer Skew:");
        for (plane, factor) in PLANES.iter().zip(factors) {
            out.push_str(&format!(
                "\n{plane} Skew: {factor:.6} radians, {:.2} degrees",
                factor.to_degrees()
            ));
        }
        gcmd.respond_info(&out);
    }

    /// `CALC_MEASURED_SKEW` (`skew_correction.py:96-101`): the factor one
    /// measured print implies, reported only — nothing is applied.
    ///
    /// # Errors
    /// An `AC`/`BD`/`AD` that is absent, not a float, or not above zero.
    fn cmd_calc_measured_skew(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let ac = gcmd.get_float_bounded("AC", Some(0.0), None)?;
        let bd = gcmd.get_float_bounded("BD", Some(0.0), None)?;
        let ad = gcmd.get_float_bounded("AD", Some(0.0), None)?;
        let factor = calc_skew_factor(ac, bd, ad);
        gcmd.respond_info(&format!(
            "Calculated Skew: {factor:.6} radians, {:.2} degrees",
            factor.to_degrees()
        ));
        Ok(())
    }

    /// `SET_SKEW` (`skew_correction.py:102-120`): `CLEAR=1` zeroes every
    /// factor, otherwise each `XY=`/`XZ=`/`YZ=` carries the three measured
    /// lengths `AC,BD,AD` that imply that plane's factor.
    ///
    /// Unlike [`SkewCorrection::update_skew`] this writes the factors without
    /// re-anchoring `last_position`, which is upstream's behaviour (`:111-120`).
    ///
    /// # Errors
    /// A plane entry that is not three comma-separated floats; the message
    /// quotes the command line, as upstream's `gcmd.error` does.
    fn cmd_set_skew(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        if gcmd.get_int_default("CLEAR", 0)? != 0 {
            self.update_skew(0.0, 0.0, 0.0);
            return Ok(());
        }
        let params = gcmd.get_command_parameters();
        let mut state = self.state();
        for plane in PLANES {
            let Some(text) = params.get(plane) else {
                continue;
            };
            let lengths = parse_lengths(text).ok_or_else(|| {
                CommandError::new(format!(
                    "skew_correction: improperly formatted entry for plane [{plane}]\n{}",
                    gcmd.commandline()
                ))
            })?;
            let factor = calc_skew_factor(lengths[0], lengths[1], lengths[2]);
            match plane {
                "XY" => state.xy_factor = factor,
                "XZ" => state.xz_factor = factor,
                _ => state.yz_factor = factor,
            }
        }
        *self.lock() = state;
        Ok(())
    }

    /// `SKEW_PROFILE` (`skew_correction.py:121-165`): `LOAD=`/`SAVE=`/`REMOVE=`,
    /// the first one the line carries.
    ///
    /// # Errors
    /// A `configfile` lookup never fails here — an absent object keeps the
    /// session copy only — so this only refuses what the parameter readers
    /// refuse.
    fn cmd_skew_profile(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let params = gcmd.get_command_parameters();
        for verb in PROFILE_VERBS {
            let Some(name) = params.get(verb) else {
                continue;
            };
            match verb {
                "LOAD" => self.load_profile(gcmd, name),
                "SAVE" => self.save_profile(gcmd, name),
                _ => self.remove_stored_profile(gcmd, name),
            }
            return Ok(());
        }
        gcmd.respond_info(&format!("Invalid syntax '{}'", gcmd.commandline()));
        Ok(())
    }

    /// `SKEW_PROFILE LOAD=` (`skew_correction.py:122-131`): remember the name,
    /// then apply the profile's factors.
    fn load_profile(&self, gcmd: &GcodeCommand, name: &str) {
        // Upstream records the name before it checks the profile exists
        // (`:123-125`).
        self.lock().current_profile_name = name.to_string();
        let Some(profile) = self.profile(name) else {
            gcmd.respond_info(&format!(
                "skew_correction:  Load failed, unknown profile [{name}]"
            ));
            return;
        };
        self.update_skew(profile.xy_skew, profile.xz_skew, profile.yz_skew);
    }

    /// `SKEW_PROFILE SAVE=` (`skew_correction.py:132-146`): queue a
    /// `[skew_correction <name>]` section for `SAVE_CONFIG` and keep the same
    /// values for this session.
    fn save_profile(&self, gcmd: &GcodeCommand, name: &str) {
        let state = self.state();
        let section = format!("{SECTION} {name}");
        if let Some(configfile) = self.configfile() {
            // Upstream hands `configfile.set` the floats and that writes
            // `str(value)` (`klippy/configfile.py:311-315`).
            configfile.set(&section, "xy_skew", &format!("{}", state.xy_factor));
            configfile.set(&section, "xz_skew", &format!("{}", state.xz_factor));
            configfile.set(&section, "yz_skew", &format!("{}", state.yz_factor));
        }
        self.store_profile(name, profile_of(&state));
        gcmd.respond_info(&format!(
            "Skew Correction state has been saved to profile [{name}]\n\
             for the current session.  The SAVE_CONFIG command will\n\
             update the printer config file and restart the printer."
        ));
    }

    /// `SKEW_PROFILE REMOVE=` (`skew_correction.py:147-165`): forget the
    /// profile and queue its section's removal.
    fn remove_stored_profile(&self, gcmd: &GcodeCommand, name: &str) {
        if !self.remove_profile(name) {
            gcmd.respond_info(&format!(
                "skew_correction: No profile named [{name}] to remove"
            ));
            return;
        }
        if let Some(configfile) = self.configfile() {
            configfile.remove_section(&format!("{SECTION} {name}"));
        }
        gcmd.respond_info(&format!(
            "Profile [{name}] removed from storage for this session.\n\
             The SAVE_CONFIG command will update the printer\n\
             configuration and restart the printer"
        ));
    }

    /// Register the four commands, capturing a handle to this object
    /// (`skew_correction.py:33-46`).
    ///
    /// # Errors
    /// A command name another module already registered.
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        const COMMANDS: &[(&str, &str, &[&str])] = &[
            ("GET_CURRENT_SKEW", "Report current printer skew", &[]),
            (
                "CALC_MEASURED_SKEW",
                "Calculate skew from measured print",
                &["AC", "BD", "AD"],
            ),
            (
                "SET_SKEW",
                "Set skew based on lengths of measured object",
                &["XY", "XZ", "YZ", "CLEAR"],
            ),
            (
                "SKEW_PROFILE",
                "Profile management for skew_correction",
                &["LOAD", "SAVE", "REMOVE"],
            ),
        ];
        for &(name, desc, params) in COMMANDS {
            let handler: CommandHandler = match name {
                "GET_CURRENT_SKEW" => {
                    let object = Arc::clone(self);
                    sync(move |gcmd| {
                        object.cmd_get_current_skew(gcmd);
                        Ok(())
                    })
                }
                "CALC_MEASURED_SKEW" => {
                    let object = Arc::clone(self);
                    sync(move |gcmd| object.cmd_calc_measured_skew(gcmd))
                }
                "SET_SKEW" => {
                    let object = Arc::clone(self);
                    sync(move |gcmd| object.cmd_set_skew(gcmd))
                }
                _ => {
                    let object = Arc::clone(self);
                    sync(move |gcmd| object.cmd_skew_profile(gcmd))
                }
            };
            gcode
                .register_command_with_params(name, handler, Some(desc), params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }
}

impl MoveTarget for SkewCorrection {
    /// Upstream's `move` (`skew_correction.py:79-81`): skew the g-code position
    /// and hand it to the chain below.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let corrected = self.calc_skew(position);
        match self.target() {
            Some(target) => target.move_to(corrected, speed),
            // No toolhead yet (`klippy:connect` has not run): nothing forwards.
            None => Err(CommandError::new("Printer is not ready")),
        }
    }

    /// Upstream's `get_position` (`skew_correction.py:73-75`): the chain's
    /// position un-skewed, so the g-code space reads square.
    fn position(&self) -> Coord {
        let Some(target) = self.target() else {
            return Coord::default();
        };
        self.calc_unskew(target.position())
    }
}

impl PrinterObject for SkewCorrection {
    /// Upstream's `get_status` (`skew_correction.py:166-169`): the profile name
    /// only.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "current_profile_name": self.lock().current_profile_name })
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

/// The g-code position as the skewed machine frame reads it — upstream's
/// `calc_skew` (`skew_correction.py:62-67`).
fn skewed(position: Coord, state: &SkewState) -> Coord {
    let mut skewed = position;
    skewed.set_axis(
        X_AXIS,
        position.x()
            - position.y() * state.xy_factor
            - position.z() * (state.xz_factor - state.xy_factor * state.yz_factor),
    );
    skewed.set_axis(Y_AXIS, position.y() - position.z() * state.yz_factor);
    skewed
}

/// The machine position as the square g-code frame reads it — upstream's
/// `calc_unskew` (`skew_correction.py:72-77`).
fn unskewed(position: Coord, state: &SkewState) -> Coord {
    let mut square = position;
    square.set_axis(
        X_AXIS,
        position.x() + position.y() * state.xy_factor + position.z() * state.xz_factor,
    );
    square.set_axis(Y_AXIS, position.y() + position.z() * state.yz_factor);
    square
}

/// The profile a state's factors make.
fn profile_of(state: &SkewState) -> SkewProfile {
    SkewProfile {
        xy_skew: state.xy_factor,
        xz_skew: state.xz_factor,
        yz_skew: state.yz_factor,
    }
}

/// Upstream's `calc_skew_factor` (`skew_correction.py:13-17`): the skew angle
/// implied by a square's two diagonals (`AC`, `BD`) and one side (`AD`).
fn calc_skew_factor(ac: f64, bd: f64, ad: f64) -> f64 {
    let side = (2.0 * ac * ac + 2.0 * bd * bd - 4.0 * ad * ad).sqrt() / 2.0;
    (PI / 2.0 - ((ac * ac - side * side - ad * ad) / (2.0 * side * ad)).acos()).tan()
}

/// The three lengths of one `SET_SKEW` plane entry (`XY=ac,bd,ad`), or `None`
/// when the entry is not exactly three floats (`skew_correction.py:108-118`).
fn parse_lengths(text: &str) -> Option<[f64; 3]> {
    let values: Vec<&str> = text.split(',').map(str::trim).collect();
    if values.len() != 3 {
        return None;
    }
    let mut lengths = [0.0; 3];
    for (slot, value) in lengths.iter_mut().zip(values) {
        *slot = value.parse::<f64>().ok()?;
    }
    Some(lengths)
}

/// The `[skew_correction <name>]` sections the config carries — upstream's
/// `_load_storage` (`skew_correction.py:51-60`).
///
/// # Errors
/// A profile section whose three factors cannot be read.
fn read_profiles(config: &ConfigWrapper) -> Result<Vec<(String, SkewProfile)>, ConfigError> {
    let mut profiles = Vec::new();
    for section in config.sibling_prefix_sections(SECTION) {
        // `[skew_correction <name>]`: the name is the identifier's second word.
        let name = section
            .identifier()
            .split_once(' ')
            .map(|(_, name)| name.to_string())
            .unwrap_or_default();
        profiles.push((
            name,
            SkewProfile {
                xy_skew: section.get_float("xy_skew", None)?,
                xz_skew: section.get_float("xz_skew", None)?,
                yz_skew: section.get_float("yz_skew", None)?,
            },
        ));
    }
    Ok(profiles)
}

/// Upstream's `load_config` for `[skew_correction]` (`skew_correction.py:171-172`).
///
/// # Errors
/// A malformed stored profile, or a command name already taken.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(SkewCorrection::new(config, printer)?);
    object.register_handlers(printer);
    object.register_commands(printer)?;
    // The transform takes the slot at `klippy:connect` (upstream's
    // `_handle_connect`); `gcode_move` only has to exist to be ready for it.
    gcode_move::ensure(printer)?;
    Ok(object)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    /// The section's object name, as the loader registers it.
    const OBJECT: &str = "skew_correction";

    /// A machine whose config was loaded from `text`: the loader registers
    /// `gcode`/`configfile`/`pins` itself, then the sections.
    fn loaded(text: &str) -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<SkewCorrection>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the test config parses");
        printer.load_config(&config).expect("the config loads");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");
        let object = printer
            .lookup_object_as::<SkewCorrection>(OBJECT)
            .expect("the section registered its object");
        (printer, gcode, object)
    }

    /// A loaded machine that is also ready to run non-built-in commands.
    fn machine(text: &str) -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<SkewCorrection>) {
        let (printer, gcode, object) = loaded(text);
        printer.send_event(&KlippyEvent::KlippyConnect);
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, gcode, object)
    }

    /// A move target that records what it was asked and stands where the last
    /// move left it — the toolhead, or another transform in the chain, as far
    /// as the coordinate math cares.
    struct FakeTarget {
        position: Mutex<Coord>,
        moves: Mutex<Vec<(Coord, f64)>>,
    }

    impl FakeTarget {
        fn new(position: Coord) -> Self {
            Self {
                position: Mutex::new(position),
                moves: Mutex::new(Vec::new()),
            }
        }

        fn moves(&self) -> Vec<(Coord, f64)> {
            self.moves.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    impl MoveTarget for FakeTarget {
        fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            self.moves
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push((position, speed));
            Ok(())
        }

        fn position(&self) -> Coord {
            *self.position.lock().unwrap_or_else(|p| p.into_inner())
        }
    }

    /// Point the transform's chain at a fake standing at `position` — what
    /// `klippy:connect` does with the connect-time toolhead.
    fn chain_at(object: &SkewCorrection, position: Coord) -> Arc<FakeTarget> {
        let fake = Arc::new(FakeTarget::new(position));
        *object.toolhead.lock().unwrap_or_else(|p| p.into_inner()) =
            Some(Arc::clone(&fake) as Arc<dyn MoveTarget>);
        fake
    }

    /// Capture everything the dispatcher sends to a client.
    fn capture_output(gcode: &Arc<GCodeDispatch>) -> Arc<Mutex<Vec<String>>> {
        let output = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&output);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        output
    }

    fn emitted(output: &Arc<Mutex<Vec<String>>>) -> String {
        output.lock().unwrap_or_else(|p| p.into_inner()).join("\n")
    }

    /// The transform this one displaced from `gcode_move`'s slot, if any.
    fn displaced(object: &SkewCorrection) -> Option<Arc<dyn MoveTarget>> {
        object
            .next_transform
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    #[test]
    fn test_the_bare_section_registers_the_object_and_its_four_commands() {
        let (printer, gcode, object) = machine("[skew_correction]\n");

        assert!(printer.lookup_object(OBJECT).is_some());
        assert!(object.is_queryable(), "upstream defines `get_status`");
        let commands = gcode.get_status(0.0)["commands"].clone();
        assert_eq!(
            commands["SKEW_PROFILE"]["parameters"],
            json!(["LOAD", "SAVE", "REMOVE"])
        );
        assert_eq!(
            commands["SKEW_PROFILE"]["help"],
            json!("Profile management for skew_correction")
        );
        assert_eq!(
            commands["SET_SKEW"]["parameters"],
            json!(["XY", "XZ", "YZ", "CLEAR"])
        );
        assert_eq!(
            commands["CALC_MEASURED_SKEW"]["parameters"],
            json!(["AC", "BD", "AD"])
        );
        assert_eq!(
            commands["GET_CURRENT_SKEW"]["help"],
            json!("Report current printer skew")
        );
    }

    /// A stored `[skew_correction <name>]` section is read at load and applied
    /// by `LOAD=`, which is how a profile survives a restart
    /// (`skew_correction.py:51-60`, `:122-131`).
    #[test]
    fn test_a_stored_profile_section_is_read_at_load_and_loaded_on_demand() {
        let (_printer, gcode, object) = machine(
            "[skew_correction]\n\
             [skew_correction shop]\n\
             xy_skew: 0.01\n\
             xz_skew: 0.02\n\
             yz_skew: 0.03\n",
        );

        // Known but not applied: the factors start at zero.
        assert_eq!(object.state().xy_factor, 0.0);
        assert_eq!(
            object.profile("shop"),
            Some(SkewProfile {
                xy_skew: 0.01,
                xz_skew: 0.02,
                yz_skew: 0.03,
            })
        );

        gcode.run_script_sync("SKEW_PROFILE LOAD=shop").unwrap();

        let state = object.state();
        assert_eq!(state.xy_factor, 0.01);
        assert_eq!(state.xz_factor, 0.02);
        assert_eq!(state.yz_factor, 0.03);
        assert_eq!(state.current_profile_name, "shop");
    }

    /// `SKEW_PROFILE LOAD=` of a name nothing stored reports upstream's
    /// wording, and still records the name (`skew_correction.py:123-130`).
    #[test]
    fn test_load_of_an_unknown_profile_reports_its_wording() {
        let (_printer, gcode, object) = machine("[skew_correction]\n");
        let output = capture_output(&gcode);

        gcode.run_script_sync("SKEW_PROFILE LOAD=nope").unwrap();

        assert_eq!(
            emitted(&output),
            "// skew_correction:  Load failed, unknown profile [nope]"
        );
        assert_eq!(object.state().current_profile_name, "nope");
    }

    /// `SKEW_PROFILE SAVE=` writes a `[skew_correction <name>]` block for
    /// `SAVE_CONFIG` and keeps the same values for the session
    /// (`skew_correction.py:132-146`).
    #[test]
    fn test_save_queues_a_section_and_keeps_the_session_copy() {
        let (printer, gcode, object) = machine("[skew_correction]\n");
        let output = capture_output(&gcode);

        gcode.run_script_sync("SET_SKEW XY=200,190,100").unwrap();
        let factor = object.state().xy_factor;
        assert!(
            (factor - 0.05831433841936399).abs() < 1e-12,
            "upstream's factor for AC=200 BD=190 AD=100, got {factor}"
        );

        gcode.run_script_sync("SKEW_PROFILE SAVE=shop").unwrap();

        assert_eq!(
            emitted(&output),
            "// Skew Correction state has been saved to profile [shop]\n\
             // for the current session.  The SAVE_CONFIG command will\n\
             // update the printer config file and restart the printer."
        );
        assert_eq!(
            object.profile("shop"),
            Some(SkewProfile {
                xy_skew: factor,
                xz_skew: 0.0,
                yz_skew: 0.0,
            })
        );
        // The write-back `SAVE_CONFIG` will flush, as the config stores floats.
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("the loader registers `configfile`");
        let status = configfile.get_status(0.0);
        assert_eq!(status["save_config_pending"], json!(true));
        assert_eq!(
            status["save_config_pending_items"]["skew_correction shop"],
            json!({
                "xy_skew": format!("{factor}"),
                "xz_skew": "0",
                "yz_skew": "0",
            })
        );
    }

    /// `SKEW_PROFILE REMOVE=` forgets a profile and queues its section's
    /// removal; an unknown name reports upstream's wording
    /// (`skew_correction.py:147-165`).
    #[test]
    fn test_remove_forgets_the_profile_and_queues_the_section_removal() {
        let (printer, gcode, object) = machine(
            "[skew_correction]\n\
             [skew_correction shop]\n\
             xy_skew: 0.01\n\
             xz_skew: 0.02\n\
             yz_skew: 0.03\n",
        );
        let output = capture_output(&gcode);
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("the loader registers `configfile`");

        gcode.run_script_sync("SKEW_PROFILE REMOVE=shop").unwrap();

        assert_eq!(object.profile("shop"), None);
        assert_eq!(
            emitted(&output),
            "// Profile [shop] removed from storage for this session.\n\
             // The SAVE_CONFIG command will update the printer\n\
             // configuration and restart the printer"
        );
        assert_eq!(
            configfile.get_status(0.0)["save_config_pending_items"]["skew_correction shop"],
            Value::Null
        );

        gcode.run_script_sync("SKEW_PROFILE REMOVE=shop").unwrap();
        assert_eq!(
            emitted(&output),
            "// Profile [shop] removed from storage for this session.\n\
             // The SAVE_CONFIG command will update the printer\n\
             // configuration and restart the printer\n\
             // skew_correction: No profile named [shop] to remove"
        );
    }

    /// A line carrying none of the three verbs reports upstream's generic
    /// syntax remark (`skew_correction.py:121-165`).
    #[test]
    fn test_a_profile_command_without_a_verb_reports_invalid_syntax() {
        let (_printer, gcode, _object) = machine("[skew_correction]\n");
        let output = capture_output(&gcode);

        gcode.run_script_sync("SKEW_PROFILE").unwrap();

        assert_eq!(emitted(&output), "// Invalid syntax 'SKEW_PROFILE'");
    }

    /// The matrix of `skew_correction.py:62-77`: a move is skewed on the way
    /// down, a position is un-skewed on the way up, and the two inverse.
    #[test]
    fn test_a_move_is_skewed_and_a_position_unskews_back() {
        let (_printer, _gcode, object) = machine("[skew_correction]\n");
        object.update_skew(0.01, 0.02, 0.03);
        let chain = chain_at(&object, Coord::new(10.0, 20.0, 5.0, 1.5));

        // `G1 X10 Y20 Z5` names the square frame; the chain below sees the
        // skewed one.
        object
            .move_to(Coord::new(10.0, 20.0, 5.0, 1.5), 25.0)
            .unwrap();
        let moves = chain.moves();
        assert_eq!(moves.len(), 1);
        assert_eq!(moves[0].1, 25.0);
        // x - y*0.01 - z*(0.02 - 0.01*0.03) = 10 - 0.2 - 0.0985
        assert!(
            (moves[0].0.x() - 9.7015).abs() < 1e-12,
            "got {}",
            moves[0].0.x()
        );
        // y - z*0.03 = 20 - 0.15
        assert!(
            (moves[0].0.y() - 19.85).abs() < 1e-12,
            "got {}",
            moves[0].0.y()
        );
        // Z and E are untouched, as upstream's `pos[2:]`.
        assert_eq!(moves[0].0.z(), 5.0);
        assert_eq!(moves[0].0.e(), 1.5);

        // Reading the same spot back through the transform lands in the square
        // frame: the toolhead stands at the skewed position.
        let position = object.position();
        assert!((position.x() - 10.0).abs() < 1e-12, "got {}", position.x());
        assert!((position.y() - 20.0).abs() < 1e-12, "got {}", position.y());
        assert_eq!(position.z(), 5.0);
    }

    /// Upstream's `klippy:connect` takes the move-transform slot with
    /// `force=True` and keeps what it displaced (`skew_correction.py:48-50`),
    /// so a `G1` reaches both transforms.
    #[test]
    fn test_the_transform_chains_over_the_occupant_it_displaces() {
        let (printer, gcode, object) = loaded("[skew_correction]\n");
        object.update_skew(0.01, 0.02, 0.03);
        let gcode_move = gcode_move::ensure(&printer).unwrap();
        // The occupancy `bed_mesh`/`bed_tilt` would leave behind before
        // `klippy:connect` runs.
        let below = Arc::new(FakeTarget::new(Coord::default()));
        gcode_move
            .set_move_transform(Arc::clone(&below) as Arc<dyn MoveTarget>, false)
            .unwrap();

        printer.send_event(&KlippyEvent::KlippyConnect);
        printer.send_event(&KlippyEvent::KlippyReady);

        // The displaced transform is the chain below this one …
        assert!(Arc::ptr_eq(
            &displaced(&object).expect("the displaced occupant"),
            &(Arc::clone(&below) as Arc<dyn MoveTarget>)
        ));
        // … and the slot is skew_correction's, so a plain registration is
        // refused.
        assert!(gcode_move
            .set_move_transform(Arc::new(FakeTarget::new(Coord::default())), false)
            .is_err());

        gcode.run_script_sync("G1 X10 Y20 Z5 F600").unwrap();

        // The displaced transform carried the move, skewed on the way down.
        let moves = below.moves();
        assert_eq!(moves.len(), 1);
        assert!((moves[0].0.x() - 9.7015).abs() < 1e-12, "got {moves:?}");
        assert!((moves[0].0.y() - 19.85).abs() < 1e-12, "got {moves:?}");
    }

    /// A real `[bed_mesh]` section holds the slot first (`bed_mesh.py:130-131`
    /// registers at load); the skew transform chains over it and both stay in
    /// the path.
    #[test]
    fn test_a_bed_mesh_registered_first_stays_in_the_chain() {
        let (printer, _gcode, object) = machine(
            "[bed_mesh]\n\
             mesh_min: 0,0\n\
             mesh_max: 100,100\n\
             probe_count: 3,3\n\
             [skew_correction]\n",
        );
        let bed_mesh = printer
            .lookup_object_as::<crate::core::klippy::extras::bed_mesh::BedMesh>(
                crate::core::klippy::extras::bed_mesh::BED_MESH_OBJECT,
            )
            .expect("the `[bed_mesh]` section registered");
        assert!(Arc::ptr_eq(
            &displaced(&object).expect("the mesh is the chain below"),
            &(bed_mesh as Arc<dyn MoveTarget>)
        ));
    }

    /// `get_status` reports the profile name and nothing else
    /// (`skew_correction.py:166-169`).
    #[test]
    fn test_get_status_reports_the_current_profile_name() {
        let (_printer, gcode, object) = machine(
            "[skew_correction]\n\
             [skew_correction shop]\n\
             xy_skew: 0.01\n\
             xz_skew: 0.02\n\
             yz_skew: 0.03\n",
        );

        assert_eq!(
            object.get_status(0.0),
            json!({ "current_profile_name": "" })
        );
        gcode.run_script_sync("SKEW_PROFILE LOAD=shop").unwrap();
        assert_eq!(
            object.get_status(0.0),
            json!({ "current_profile_name": "shop" })
        );
    }

    /// `GET_CURRENT_SKEW` reports the three planes in radians and degrees
    /// (`skew_correction.py:83-95`).
    #[test]
    fn test_get_current_skew_reports_radians_and_degrees() {
        let (_printer, gcode, object) = machine("[skew_correction]\n");
        let output = capture_output(&gcode);
        object.update_skew(0.01, 0.02, 0.03);

        gcode.run_script_sync("GET_CURRENT_SKEW").unwrap();

        assert_eq!(
            emitted(&output),
            "// Current Printer Skew:\n\
             // XY Skew: 0.010000 radians, 0.57 degrees\n\
             // XZ Skew: 0.020000 radians, 1.15 degrees\n\
             // YZ Skew: 0.030000 radians, 1.72 degrees"
        );
    }

    /// `CALC_MEASURED_SKEW` reports the factor one measured print implies and
    /// applies nothing (`skew_correction.py:96-101`).
    #[test]
    fn test_calc_measured_skew_reports_the_factor_without_applying_it() {
        let (_printer, gcode, object) = machine("[skew_correction]\n");
        let output = capture_output(&gcode);

        gcode
            .run_script_sync("CALC_MEASURED_SKEW AC=200 BD=190 AD=100")
            .unwrap();

        assert_eq!(
            emitted(&output),
            "// Calculated Skew: 0.058314 radians, 3.34 degrees"
        );
        assert_eq!(object.state().xy_factor, 0.0, "nothing is applied");
    }

    /// `SET_SKEW` takes the three lengths per plane and `CLEAR=1` zeroes every
    /// factor (`skew_correction.py:102-120`).
    #[test]
    fn test_set_skew_reads_measured_lengths_per_plane_and_clears() {
        let (_printer, gcode, object) = machine("[skew_correction]\n");

        gcode
            .run_script_sync("SET_SKEW XY=200,190,100 XZ=200,190,100")
            .unwrap();
        let state = object.state();
        assert!((state.xy_factor - 0.05831433841936399).abs() < 1e-12);
        assert!((state.xz_factor - 0.05831433841936399).abs() < 1e-12);
        assert_eq!(state.yz_factor, 0.0);

        gcode.run_script_sync("SET_SKEW CLEAR=1").unwrap();
        let state = object.state();
        assert_eq!(state.xy_factor, 0.0);
        assert_eq!(state.xz_factor, 0.0);
        assert_eq!(state.yz_factor, 0.0);
    }

    /// A plane entry that is not three floats is refused with upstream's
    /// wording, which quotes the command line (`skew_correction.py:112-118`).
    #[test]
    fn test_set_skew_refuses_a_malformed_plane_entry() {
        let (_printer, gcode, object) = machine("[skew_correction]\n");

        let error = gcode.run_script_sync("SET_SKEW XY=200,190").unwrap_err();

        assert_eq!(
            error.to_string(),
            "skew_correction: improperly formatted entry for plane [XY]\nSET_SKEW XY=200,190"
        );
        assert_eq!(object.state().xy_factor, 0.0);
    }
}
