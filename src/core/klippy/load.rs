//! Config-driven object loading: section id → factory.
//!
//! Upstream's `Printer._read_config` (`klippy/klippy.py:111`) turns the parsed
//! config into printer objects: it asks the modules that must exist before
//! anything else for their objects, then walks every prefix section, then
//! validates that no section was left unused (`klippy/configfile.py:425`). This
//! module is that step, with upstream's dynamic module lookup replaced by a
//! static table.
//!
//! The table **is** the schema: a section id is valid exactly when some factory
//! claims it. `[mcu]` is claimed by a `load_config`, `[mcu zboard]` by a
//! `load_config_prefix` — the same two entry points upstream looks up on the
//! module (`klippy/klippy.py:90-99`).
//!
//! The table is **generated** from the `section!` declarations in the modules
//! that own sections (see `build.rs`): a module is one declaration next to the
//! factory it names, so adding a section never edits a central list. Each
//! declaration carries an `order`, because load order is part of the contract,
//! plus two optional fields for the sections that are not a plain extras module:
//!
//! - `phase = early|generic|late` (default `generic`) places a section before or
//!   after the generic walk, the way upstream loads `mcu` up front and
//!   `toolhead` last (`klippy/klippy.py:120-125`);
//! - `object = "<name>"` registers the built object under a name other than the
//!   section's, because `[printer]`'s consumer is the `toolhead` object.
//!
//! The step itself is [`Printer::load_config`], defined here rather than in
//! `printer.rs` so that the machine's core does not import its parts — the same
//! split as `Mcu::connect` living in `identify.rs`.
//!
//! # Order
//!
//! Phase by phase (early, generic, late), and within a phase main sections
//! first (table order), then prefix sections (table order). Upstream loads its
//! up-front modules the same way (`mcu` before the generic prefix walk) and
//! keeps `toolhead` for last; the order matters because an object may look up
//! one an earlier entry registered.
//!
//! # Validation
//!
//! The undefined-option check runs at the end of [`Printer::load_config`] and
//! uses the access tracking in [`ConfigWrapper`] as the schema — see
//! [`check_unused`]. Every option read through a wrapper is recorded; a factory
//! that must read a section later keeps the printer's tracker
//! ([`Printer::access_tracking`]).

use std::sync::Arc;

use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::{
    check_unused, AccessTracking, Config, ConfigError, ConfigWrapper,
};
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// Builds one printer object from a config section.
///
/// The loader registers what a factory returns under the section's identifier
/// (`mcu`, `mcu zboard`) — or under the declaration's `object` name when it has
/// one — so a factory never names its own object, and two sections cannot
/// silently claim one name. The printer is passed because an object may wire
/// itself up as it is built.
///
/// `Err` is the factory's own complaint about the section, reported by the
/// loader as a config error ([`ConfigError`]).
pub type LoadConfig =
    fn(&ConfigWrapper, &Arc<Printer>) -> Result<Arc<dyn PrinterObject>, ConfigError>;

/// When a section is loaded relative to the generic walk.
///
/// Upstream's `_read_config` loads a few modules explicitly around the generic
/// prefix walk (`klippy/klippy.py:120-125`); this is that placement, spelled out
/// on the declaration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Before the generic walk: the modules everything else looks up (`mcu`).
    Early,
    /// The generic walk (the default).
    Generic,
    /// After the generic walk: modules that consume objects the walk built
    /// (`toolhead`, which owns `[printer]`).
    Late,
}

/// One section id's entry points and placement, as upstream's `load_config` /
/// `load_config_prefix` plus where the module is loaded.
#[derive(Clone, Copy)]
pub struct Factories {
    /// Builds the bare `[<id>]` section.
    pub load_config: Option<LoadConfig>,
    /// Builds each `[<id> <name>]` section.
    pub load_config_prefix: Option<LoadConfig>,
    /// The name to register the object under, when it differs from the section
    /// identifier (`[printer]` → `toolhead`).
    pub object: Option<&'static str>,
    /// When to load this section relative to the generic walk.
    pub phase: Phase,
}

// Every section id this host knows, in load order.
//
// Generated from the `section!` declarations (see the module docs).
include!(concat!(env!("OUT_DIR"), "/section_factories.rs"));

/// Every section id this host knows, in load order.
///
/// For tooling that has to tell "known section" from "gap" without loading a
/// config (the upstream regression gap report). The list is the generated
/// [`FACTORIES`] table, so it moves with the `section!` declarations.
#[cfg(test)]
pub(crate) fn known_section_ids() -> Vec<&'static str> {
    FACTORIES.iter().map(|(id, _)| *id).collect()
}

/// Declare one config section. Expands to nothing; `build.rs` scans it.
///
/// A declaration names the section, an `order`, and the factories it has: a
/// bare section (`load = load_config`), a prefix section
/// (`prefix = load_config_prefix`), or both. `order` decides the load order
/// among the entries that share a half.
///
/// Two optional fields place a section that is not a plain extras module:
/// `phase = early|generic|late` (default `generic`) and `object = "<name>"`, the
/// name to register the built object under when it differs from the section id.
///
/// The factories are named as siblings: a bare name resolves to
/// `<this module>::<name>`, a path is used as written.
macro_rules! section {
    ($($tokens:tt)*) => {};
}
pub(crate) use section;

impl Printer {
    /// Load every printer object the config describes into this machine.
    ///
    /// The receiver is `&Arc<Self>` rather than `&self` because a factory is
    /// handed the shared handle: an object may hold on to the machine it belongs
    /// to — to register handlers, or to look another object up as it is built.
    /// Defining this next to the table keeps the machine's core free of its
    /// parts.
    ///
    /// The printer is expected to be freshly built: loading twice would trip the
    /// duplicate-name check, which is the intent — a name is registered once.
    ///
    /// # Order
    ///
    /// This is not the first thing that happens to a fresh machine. The API
    /// server's own object (`webhooks`) is registered *before* the config is
    /// loaded, so that `objects/list` starts with it as upstream's does (see
    /// [`api::register`](crate::core::klippy::api::register)); a host that loads
    /// first and registers it afterwards reorders that list.
    ///
    /// [`GCodeDispatch`](crate::core::klippy::gcode::GCodeDispatch) comes first
    /// of all, because `pins` sections and resources register commands with it
    /// as they are built. Upstream registers the same object in
    /// `Printer.__init__` (`klippy/klippy.py:36-40`), before the config is read.
    ///
    /// [`PrinterConfig`](crate::core::klippy::config::PrinterConfig) comes next:
    /// it owns the access tracking the whole load records into, and upstream
    /// registers it before `pins` and `mcu` too (`klippy/klippy.py:115-121`).
    ///
    /// [`PrinterPins`](crate::core::klippy::pins::PrinterPins) is registered
    /// next, unconditionally, because every resource and `[board_pins]` reaches
    /// it while sections are being loaded — and the MCU objects register
    /// themselves as chips as they are built. Upstream loads the same objects up
    /// front (`pins` then `mcu`, `klippy/klippy.py:118-119`). `pins` is
    /// registered but never queryable, so it is not in `objects/list`.
    ///
    /// # Errors
    /// Returns [`ConfigError`] if a factory rejects a section, if a name is
    /// already taken, or if a section or option nothing read is left over — the
    /// last is upstream's `Section '%s' is not a valid config section` and
    /// `Option '%s' is not valid in section '%s'` (`klippy/configfile.py:431`,
    /// `:440`).
    pub fn load_config(self: &Arc<Self>, config: &Config) -> Result<(), ConfigError> {
        // Remember where the host's own parts end, so a restart can keep them
        // and drop only what the config loads (see `reset_for_restart`).
        self.mark_host_objects();

        // One access record for this load, shared by every wrapper, the
        // `configfile` object, and any part that reads its section later.
        let access = AccessTracking::shared();
        self.set_access_tracking(Arc::clone(&access));

        self.add_object(GCODE_OBJECT, Arc::new(GCodeDispatch::new(Arc::clone(self))))?;
        self.add_object(
            CONFIGFILE_OBJECT,
            Arc::new(PrinterConfig::new(
                Arc::clone(&access),
                PrinterConfig::raw_config(config),
            )),
        )?;
        self.add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))?;

        let claimed = self.load_sections(config, &access, FACTORIES)?;
        check_unused(config, &access, &claimed)?;
        Ok(())
    }

    /// Walk a factory table, registering what it claims.
    ///
    /// Split out from [`Printer::load_config`] so a test can drive the loader
    /// with a synthetic table and exercise the phase/name rules without adding a
    /// real section.
    fn load_sections(
        self: &Arc<Self>,
        config: &Config,
        access: &Arc<AccessTracking>,
        factories: &[(&str, Factories)],
    ) -> Result<Vec<String>, ConfigError> {
        let mut claimed: Vec<String> = Vec::new();

        // Early → generic → late; within a phase, main sections then prefixes.
        for phase in [Phase::Early, Phase::Generic, Phase::Late] {
            for (id, entry) in factories.iter().filter(|(_, entry)| entry.phase == phase) {
                let Some(load) = entry.load_config else {
                    continue;
                };
                let Some(section) = config.get_section(id) else {
                    continue;
                };
                self.register(load, entry, section, config, access, &mut claimed)?;
            }
            for (id, entry) in factories.iter().filter(|(_, entry)| entry.phase == phase) {
                let Some(load) = entry.load_config_prefix else {
                    continue;
                };
                for section in config.get_sections_by_id(id) {
                    // The bare `[id]` is the main section above; only `[id <name>]`
                    // is a prefix section, which is what upstream's
                    // `get_prefix_sections` returns.
                    if section.sub.is_none() {
                        continue;
                    }
                    self.register(load, entry, section, config, access, &mut claimed)?;
                }
            }
        }

        Ok(claimed)
    }

    /// Build and register one section, recording it as claimed.
    ///
    /// The section handed to the factory has the printer's in-memory overrides
    /// applied on top of the parsed config ([`Printer::override_config`]), so a
    /// part that found an option unworkable is read back with the replacement.
    fn register(
        self: &Arc<Self>,
        load: LoadConfig,
        entry: &Factories,
        section: &crate::core::klippy::config::ConfigSection,
        config: &Config,
        access: &Arc<AccessTracking>,
        claimed: &mut Vec<String>,
    ) -> Result<(), ConfigError> {
        let identifier = section.identifier();
        let overrides = self.overrides_for(&identifier);
        let overridden;
        let section = if overrides.is_empty() {
            section
        } else {
            overridden = {
                let mut section = section.clone();
                for (option, value) in overrides {
                    // Keys are folded at `override_config`, and again here so
                    // any future writer of this map is covered too.
                    section.parameters.insert(option.to_lowercase(), value);
                }
                section
            };
            &overridden
        };

        let wrapper = match self.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT) {
            Some(configfile) => {
                ConfigWrapper::with_config(section, Arc::clone(access), Some(configfile), config)
            }
            None => ConfigWrapper::with_config(section, Arc::clone(access), None, config),
        };
        let object = load(&wrapper, self)?;
        let name = entry.object.unwrap_or(identifier.as_str());
        self.add_object(name, object)?;
        claimed.push(identifier);
        Ok(())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::{json, Value};

    /// Parse a config from its text, as the host does from a file.
    fn config(text: &str) -> Config {
        Config::from_text(text).expect("the test config parses").0
    }

    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let result = printer.load_config(&config(text));
        (printer, result)
    }

    /// A printer object that reports nothing, for the synthetic-table test.
    struct Nothing;

    impl PrinterObject for Nothing {
        fn get_status(&self, _eventtime: f64) -> Value {
            json!({})
        }
    }

    fn nothing(
        _config: &ConfigWrapper,
        _printer: &Arc<Printer>,
    ) -> Result<Arc<dyn PrinterObject>, ConfigError> {
        Ok(Arc::new(Nothing))
    }

    #[test]
    fn test_the_factory_table_is_in_load_order() {
        // The table is generated from the `section!` declarations; this pins the
        // order those declarations ask for, which the loader depends on.
        let ids: Vec<&str> = FACTORIES.iter().map(|(id, _)| *id).collect();
        assert_eq!(
            ids,
            [
                "mcu",
                "stepper_enable",
                "extruder",
                "extruder_stepper",
                "fan",
                "heater_bed",
                "heater_generic",
                "manual_stepper",
                "output_pin",
                "pwm_cycle_time",
                "pwm_tool",
                "servo",
                "adc_temperature",
                "thermistor",
                "bed_mesh",
                "bed_screws",
                "bed_tilt",
                // Sorted within its `order` group by section id; the bytes put
                // `bltouch` before `board_pins` (`l` < `o`).
                "bltouch",
                "board_pins",
                "controller_fan",
                "delta_calibrate",
                "display_status",
                "exclude_object",
                "filament_motion_sensor",
                "filament_switch_sensor",
                "gcode_arcs",
                "gcode_macro",
                "heater_fan",
                "homing_override",
                "input_shaper",
                "manual_probe",
                "probe",
                "probe_eddy_current",
                "quad_gantry_level",
                "resonance_tester",
                "screws_tilt_adjust",
                "sdcard_loop",
                "smart_effector",
                "temperature_fan",
                "temperature_sensor",
                "virtual_sdcard",
                "z_tilt",
                "static_digital_output",
                "adxl345",
                "display_template",
                "dotstar",
                "i2c_device",
                "led",
                "mpu9250",
                "neopixel",
                "pca9533",
                "pca9632",
                "spi_device",
                "stepper_a",
                "stepper_arm",
                "stepper_b",
                "stepper_bed",
                "stepper_c",
                "stepper_x",
                "stepper_y",
                "stepper_z",
                "dual_carriage",
                "endstop_phase",
                "printer",
                // The G28 wrapper takes the toolhead's handler away, so it has
                // to load after `[printer]` (`order = 70` against `60`).
                "safe_z_home"
            ]
        );
        // `mcu` is the one up-front section (upstream loads `pins` and `mcu`
        // before the generic walk); `[stepper_*]` and `[printer]` are late
        // (upstream builds `toolhead` last, and `Rail::lookup` reads stepper
        // config at that point); the rest are plain generic sections.
        let by_id = |id: &str| {
            FACTORIES
                .iter()
                .find(|(name, _)| *name == id)
                .map(|(_, entry)| *entry)
                .unwrap_or_else(|| panic!("no section '{id}'"))
        };
        assert_eq!(by_id("mcu").phase, Phase::Early);
        assert_eq!(by_id("output_pin").phase, Phase::Generic);
        assert_eq!(by_id("stepper_x").phase, Phase::Late);
        assert_eq!(by_id("stepper_arm").phase, Phase::Late);
        assert_eq!(by_id("stepper_bed").phase, Phase::Late);
        // `[endstop_phase <stepper>]` reads its stepper section, so it loads
        // after the steppers and before `[printer]`.
        assert_eq!(by_id("endstop_phase").phase, Phase::Late);
        assert_eq!(by_id("printer").phase, Phase::Late);
        assert_eq!(by_id("printer").object, Some("toolhead"));
        assert!(FACTORIES
            .iter()
            .filter(|(id, _)| *id != "printer")
            .all(|(_, entry)| entry.object.is_none()));
    }

    #[test]
    fn test_a_late_section_loads_after_the_generic_walk() {
        // The tenant rule: a section declared `phase = late` is loaded after the
        // generic sections, and `object = "..."` registers it under a name other
        // than the section id (`[printer]` → `toolhead`).
        let factories: &[(&str, Factories)] = &[
            (
                "first",
                Factories {
                    load_config: Some(nothing),
                    load_config_prefix: None,
                    object: None,
                    phase: Phase::Generic,
                },
            ),
            (
                "printer",
                Factories {
                    load_config: Some(nothing),
                    load_config_prefix: None,
                    object: Some("toolhead"),
                    phase: Phase::Late,
                },
            ),
        ];
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = config("[printer]\nmax_velocity: 500\n[first]\nvalue: 1\n");
        let access = AccessTracking::shared();
        // The synthetic factories read nothing, so both options are unread; the
        // loader's own `load_sections` is what this test exercises.
        let claimed = printer.load_sections(&config, &access, factories).unwrap();

        assert_eq!(printer.objects(), ["first", "toolhead"]);
        assert_eq!(claimed, ["first", "printer"]);
    }

    #[test]
    fn test_a_factory_deprecate_reaches_the_configfile_object() {
        fn deprecating(
            config: &ConfigWrapper,
            printer: &Arc<Printer>,
        ) -> Result<Arc<dyn PrinterObject>, ConfigError> {
            config.deprecate("pin", None);
            nothing(config, printer)
        }

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = config("[legacy]\npin: PA0\n");
        printer
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::new(PrinterConfig::new(
                    AccessTracking::shared(),
                    PrinterConfig::raw_config(&config),
                )),
            )
            .unwrap();
        let factories: &[(&str, Factories)] = &[(
            "legacy",
            Factories {
                load_config: Some(deprecating),
                load_config_prefix: None,
                object: None,
                phase: Phase::Generic,
            },
        )];
        printer
            .load_sections(&config, &AccessTracking::shared(), factories)
            .unwrap();

        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .unwrap();
        let warnings = configfile.get_status(0.0)["warnings"].clone();
        assert_eq!(warnings.as_array().unwrap().len(), 1);
        assert_eq!(warnings[0]["option"], json!("pin"));
    }

    #[test]
    fn test_an_early_section_loads_before_the_generic_walk() {
        let factories: &[(&str, Factories)] = &[
            (
                "chip",
                Factories {
                    load_config: Some(nothing),
                    load_config_prefix: None,
                    object: None,
                    phase: Phase::Early,
                },
            ),
            (
                "first",
                Factories {
                    load_config: Some(nothing),
                    load_config_prefix: None,
                    object: None,
                    phase: Phase::Generic,
                },
            ),
        ];
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = config("[first]\nvalue: 1\n[chip]\nserial: /dev/a\n");
        let access = AccessTracking::shared();

        printer.load_sections(&config, &access, factories).unwrap();

        assert_eq!(printer.objects(), ["chip", "first"]);
    }

    #[test]
    fn test_the_main_mcu_section_becomes_the_mcu_object() {
        let (printer, result) = load("[mcu]\nserial: /dev/not-opened-yet\n");

        result.unwrap();
        // `configfile` is registered after `gcode`, `pins` after it (upstream
        // loads `pins` and `mcu` up front), then `error_mcu` — which the first
        // `[mcu]` section brings with it (`klippy/mcu.py:1159`) — and the
        // section's own object.
        assert_eq!(
            printer.objects(),
            ["gcode", "configfile", "pins", "error_mcu", "mcu"]
        );
    }

    #[test]
    fn test_an_mcu_prefix_loads_before_a_generic_section_that_names_it() {
        // `mcu` is an early section, so `[mcu zboard]` is a registered chip
        // before the generic walk reaches `[board_pins]`, which names it.
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [mcu zboard]\nserial: /dev/b\n\
             [board_pins]\nmcu: zboard\naliases: X=PA0\n",
        );

        result.unwrap();
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "error_mcu",
                "mcu",
                "mcu zboard",
                "board_pins"
            ]
        );
    }

    #[test]
    fn test_the_config_can_be_loaded_again_after_a_restart() {
        // A restart drops the config's parts and loads the same file again: the
        // registry ends up exactly as it started, with nothing left over.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = config("[mcu]\nserial: /dev/not-opened-yet\n");
        printer.load_config(&config).unwrap();
        assert_eq!(
            printer.objects(),
            ["gcode", "configfile", "pins", "error_mcu", "mcu"]
        );

        printer.reset_for_restart("restart");
        printer.load_config(&config).unwrap();

        assert_eq!(
            printer.objects(),
            ["gcode", "configfile", "pins", "error_mcu", "mcu"]
        );
    }

    #[test]
    fn test_a_cartesian_printer_loads_its_steppers_and_toolhead() {
        // The full chain the framework was waiting for: `[stepper_*]` are valid
        // sections, and `[printer]` is the late tenant that consumes them and
        // registers the `toolhead` object (`klippy.py:124`).
        let (printer, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
        );

        result.unwrap();
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "error_mcu",
                "mcu",
                "stepper_enable",
                "stepper_x",
                "stepper_y",
                "stepper_z",
                "query_endstops",
                // The toolhead's factory loads the default modules before the
                // loader registers the toolhead itself (`gcode_move::ensure`),
                // so `gcode_move` lands one slot earlier than upstream does
                // (`toolhead.py:610-613` adds the toolhead first). Nothing
                // resolves objects by position — `gcode_move` looks the
                // toolhead up by name at ready — so only the list order shows
                // it.
                "gcode_move",
                // `manual_probe` is on the same "default modules" list
                // (`toolhead.py:293`), so it lands here too — which is what
                // makes `PROBE_CALIBRATE` work without a `[manual_probe]`
                // section.
                "manual_probe",
                "toolhead"
            ]
        );
        // The steppers are registered but not client-visible; the toolhead is.
        let queryable = printer.queryable_objects();
        assert!(queryable.contains(&"toolhead".to_string()));
        assert!(!queryable.contains(&"stepper_x".to_string()));
        // `[printer]`'s consumer is the `toolhead` object, not `printer`.
        assert!(printer.lookup_object("printer").is_none());
        assert_eq!(
            printer.lookup_object("toolhead").unwrap().get_status(0.0),
            json!({})
        );
    }

    #[test]
    fn test_a_cartesian_printer_without_a_stepper_is_a_config_error() {
        let (_, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
        );

        let err = result.unwrap_err().to_string();
        assert!(err.contains("needs a '[stepper_x]'"), "{err}");
    }

    #[test]
    fn test_the_configfile_object_reports_the_config_and_its_reads() {
        let (printer, result) = load("[mcu]\nserial: /dev/a\n[output_pin fan]\npin: PA1\n");

        result.unwrap();
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("the loader registers `configfile`");
        let status = configfile.get_status(0.0);
        assert_eq!(status["config"]["output_pin fan"]["pin"], json!("PA1"));
        assert_eq!(status["settings"]["output_pin fan"]["pin"], json!("PA1"));
        // Queryable, so `objects/list` reports it as upstream's does.
        assert!(printer
            .queryable_objects()
            .contains(&CONFIGFILE_OBJECT.to_string()));
    }

    #[test]
    fn test_an_override_reaches_the_factory_that_reads_the_section() {
        // The loader hands the factory the parsed section **plus** what the
        // printer recorded: a part that found an option unworkable at run time is
        // read back with the replacement. The pin here resolves against the MCU's
        // chip, and the override names a chip that does not exist — so the
        // override is the only reason this section stops loading.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = config("[mcu]\nserial: /dev/not-opened-yet\n[output_pin fan]\npin: PA0\n");
        printer.load_config(&config).unwrap();
        printer.reset_for_restart("restart");

        printer.override_config(
            "output_pin fan",
            "pin",
            ConfigValue::Single("nope:PA0".to_string()),
        );
        let err = printer.load_config(&config).unwrap_err();
        assert!(err.to_string().contains("nope"), "{err}");
    }

    #[test]
    fn test_an_override_name_is_folded_like_every_other_option() {
        // `override_config` goes through the same option-name folding as the
        // parser: the replacement lands under the lowercase name, so the
        // factory's case-folding read finds it instead of the parsed value.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = config("[mcu]\nserial: /dev/not-opened-yet\n[output_pin fan]\npin: PA0\n");
        printer.load_config(&config).unwrap();
        printer.reset_for_restart("restart");

        printer.override_config(
            "output_pin fan",
            "PIN",
            ConfigValue::Single("nope:PA0".to_string()),
        );
        let err = printer.load_config(&config).unwrap_err();
        assert!(err.to_string().contains("nope"), "{err}");
    }

    #[test]
    fn test_prefix_sections_become_objects_of_their_own() {
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [mcu zboard]\nserial: /dev/b\n\
             [mcu toolhead]\nserial: /dev/c\n",
        );

        result.unwrap();
        // The main section first, then the prefix sections in config order —
        // upstream's `add_printer_objects` order (`klippy/mcu.py:1239-1246`).
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "error_mcu",
                "mcu",
                "mcu zboard",
                "mcu toolhead"
            ]
        );
    }

    #[test]
    fn test_an_unknown_section_is_rejected_the_way_upstream_rejects_it() {
        let (_printer, result) = load("[mcu]\nserial: /dev/a\n[made_up]\n");

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Section 'made_up' is not a valid config section"),
            "{err}"
        );
    }

    #[test]
    fn test_an_unknown_option_is_rejected_the_way_upstream_rejects_it() {
        // The option check is the schema by use: `pin` is read, `pinn` is not.
        let (_printer, result) =
            load("[mcu]\nserial: /dev/a\n[output_pin fan]\npin: PA0\npinn: PA1\n");

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Option 'pinn' is not valid in section 'output_pin fan'"),
            "{err}"
        );
    }

    #[test]
    fn test_an_unknown_option_in_the_mcu_section_is_rejected() {
        // The MCU section is parsed at load time so its options are recorded
        // before the check runs, even though the device opens at connect.
        let (_printer, result) = load("[mcu]\nserial: /dev/a\nserail: /dev/b\n");

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Option 'serail' is not valid in section 'mcu'"),
            "{err}"
        );
    }

    #[test]
    fn test_a_config_with_no_objects_loads_only_the_builtins() {
        let (printer, result) = load("");

        result.unwrap();
        // `gcode`, `configfile` and `pins` are unconditional; no section
        // contributed anything else.
        assert_eq!(printer.objects(), ["gcode", "configfile", "pins"]);
    }

    #[test]
    fn test_a_bad_interface_is_not_noticed_until_connect() {
        // Two-phase construction: loading only parses the section. The device is
        // opened — and the port found missing — by `McuObject::connect`, so a
        // config whose transport cannot open still *loads*, and the error is
        // reported when the printer comes up rather than when the file is read.
        let (printer, result) = load("[mcu]\nserial: /dev/not-a-serial-port\n");

        result.unwrap();
        assert_eq!(
            printer.objects(),
            ["gcode", "configfile", "pins", "error_mcu", "mcu"]
        );
    }

    #[test]
    fn test_an_output_pin_section_is_claimed_by_its_factory() {
        // The first real resource section: `[mcu]` registers the chip while it
        // is built, `[output_pin fan]` looks it up and builds a digital output.
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [output_pin fan]\npin: PA1\n",
        );

        result.unwrap();
        // Main sections first, then the prefix section.
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "error_mcu",
                "mcu",
                "output_pin fan"
            ]
        );
    }

    #[test]
    fn test_a_board_pins_section_is_claimed_by_its_factory() {
        // `[board_pins]` is both a main and a prefix section. It registers no
        // resource, only aliases the pin names used by later sections.
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [board_pins]\naliases:\n    EXP1=PA0, EXP2=PA1\n\
             [board_pins second]\nmcu: mcu\naliases: EXP3=PA2\n",
        );

        result.unwrap();
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "error_mcu",
                "mcu",
                "board_pins",
                "board_pins second"
            ]
        );
    }

    #[test]
    fn test_an_i2c_device_section_is_claimed_by_its_factory() {
        // The F7 consumer: `[mcu]` registers the chip while it is built,
        // `[i2c_device accel]` asks it for a bus and registers the debug
        // commands.
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [i2c_device accel]\ni2c_address: 104\n",
        );

        result.unwrap();
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "error_mcu",
                "mcu",
                "i2c_device accel"
            ]
        );
    }

    #[test]
    fn test_a_spi_device_section_is_claimed_by_its_factory() {
        // The F6 consumer: `[mcu]` registers the chip while it is built,
        // `[spi_device flash]` asks it for a bus and registers the debug
        // commands.
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [spi_device flash]\ncs_pin: PA15\nspi_bus: spi1a\n",
        );

        result.unwrap();
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "error_mcu",
                "mcu",
                "spi_device flash"
            ]
        );
    }

    // =========================================================================
    // Late-section chip resolution test
    // =========================================================================

    /// Prove that a late section can use a chip registered by a generic section.
    ///
    /// This is the regression guard for the stepper_x/y/z → late migration:
    /// when `[stepper_*]` was generic, `endstop_pin` resolution failed for chips
    /// registered by later sections (probe, TMC). Moving stepper to late means
    /// all generic sections (including probe/TMC) load **before** stepper's
    /// `setup_endstop` call, so the chip is already registered.
    ///
    /// The test uses `PrinterPins::chips()` (public) to verify the chip was
    /// registered by the time the late section runs. This mirrors the internal
    /// `PrinterPins::chip()` lookup that `setup_endstop` uses.
    #[test]
    fn test_a_late_section_can_use_chips_registered_by_generic_sections() {
        use crate::core::klippy::pins::{PinChip, PinParams, PrinterPins};
        use crate::core::klippy::printer::PrinterObject;

        // A minimal PinChip that satisfies the trait but does nothing.
        struct NoopChip;
        impl PinChip for NoopChip {
            fn setup_digital_out(
                &self,
                _params: &PinParams,
            ) -> Result<
                Arc<dyn crate::core::klippy::pins::DigitalOut>,
                crate::core::klippy::pins::PinError,
            > {
                Err(crate::core::klippy::pins::PinError::Unsupported(
                    "digital_out".into(),
                ))
            }
            fn setup_pwm(
                &self,
                _params: &PinParams,
            ) -> Result<
                Arc<dyn crate::core::klippy::pins::PwmOut>,
                crate::core::klippy::pins::PinError,
            > {
                Err(crate::core::klippy::pins::PinError::Unsupported(
                    "pwm".into(),
                ))
            }
            fn setup_adc(
                &self,
                _params: &PinParams,
            ) -> Result<Arc<dyn crate::core::klippy::pins::Adc>, crate::core::klippy::pins::PinError>
            {
                Err(crate::core::klippy::pins::PinError::Unsupported(
                    "adc".into(),
                ))
            }
            fn setup_stepper(
                &self,
                _step: &PinParams,
                _dir: &PinParams,
                _inv_step: i8,
                _spd: f64,
                _inv_dir: bool,
            ) -> Result<
                Arc<crate::core::klippy::mcu::McuStepper>,
                crate::core::klippy::pins::PinError,
            > {
                Err(crate::core::klippy::pins::PinError::Unsupported(
                    "stepper".into(),
                ))
            }
        }

        // A synthetic section that registers a chip in PrinterPins.
        fn register_test_chip(
            config: &ConfigWrapper,
            printer: &Arc<Printer>,
        ) -> Result<Arc<dyn PrinterObject>, ConfigError> {
            let chip_name = config.get("chip_name", Some("test_chip"))?;
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("pins object must be registered");
            pins.register_chip(&chip_name, Arc::new(NoopChip))
                .map_err(|e| ConfigError::new(format!("register_chip: {e}")))?;
            Ok(Arc::new(Nothing))
        }

        // A synthetic late section that verifies the chip is registered.
        fn check_chip_resolved(
            config: &ConfigWrapper,
            printer: &Arc<Printer>,
        ) -> Result<Arc<dyn PrinterObject>, ConfigError> {
            let chip_name = config.get("chip_name", Some("test_chip"))?;
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("pins object must be registered");
            // `chips()` returns the list of registered chip names. If the chip
            // isn't there, the late section loaded before the generic one —
            // which would be the bug we're guarding against.
            let registered = pins.chips();
            if !registered.iter().any(|n| *n == chip_name) {
                return Err(ConfigError::new(format!(
                    "chip '{chip_name}' not registered yet (registered: {registered:?})"
                )));
            }
            Ok(Arc::new(Nothing))
        }

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        // Set up the builtin objects that real sections depend on.
        use crate::core::klippy::config::PrinterConfig;
        use crate::core::klippy::pins::PINS_OBJECT;

        printer
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::new(PrinterConfig::new(
                    AccessTracking::shared(),
                    PrinterConfig::raw_config(&config("")),
                )),
            )
            .unwrap();
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::default()))
            .unwrap();

        // Factory table: generic `chip_provider` registers a chip, then late
        // `late_consumer` looks it up.
        let factories: &[(&str, Factories)] = &[
            (
                "chip_provider",
                Factories {
                    load_config: Some(register_test_chip),
                    load_config_prefix: None,
                    object: None,
                    phase: Phase::Generic,
                },
            ),
            (
                "late_consumer",
                Factories {
                    load_config: Some(check_chip_resolved),
                    load_config_prefix: None,
                    object: None,
                    phase: Phase::Late,
                },
            ),
        ];

        let config_text = "\
            [chip_provider]
            chip_name: my_probe_chip
            [late_consumer]
            chip_name: my_probe_chip
        ";
        let config = config(config_text);
        let access = AccessTracking::shared();

        // If the late section loads before the generic section, `chips()`
        // would not contain the chip and this call would error.
        // Because generic → late, the chip is registered first.
        let claimed = printer
            .load_sections(&config, &access, factories)
            .expect("late section should resolve the chip registered by generic");

        assert_eq!(claimed, ["chip_provider", "late_consumer"]);
    }
}
