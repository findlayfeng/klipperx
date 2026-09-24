//! `[bed_screws]` — the bed-screws helper's section
//! (upstream `klippy/extras/bed_screws.py`).
//!
//! The section reads `screw1`..`screw99`, stopping at the first missing one
//! (`bed_screws.py:15-18`); each screw carries an optional `screwN_name`
//! (default `screw at %.3f,%.3f` from its own coordinates) and an optional
//! `screwN_fine_adjust` coordinate recorded under the *coarse* screw's name
//! (`:19-26`). Fewer than three screws is a config error (`:27-28`), then the
//! travels are read: `speed` (`50.`), `probe_speed` (`5.`, upstream's
//! `lift_speed`), `horizontal_move_z` (`5.`) and `probe_height` (`0.`,
//! upstream's `probe_z`) — `speed`/`probe_speed` must be above `0`
//! (`:31-34`). Every option is read through the tracking wrapper, which is
//! what lets [`check_unused`](crate::core::klippy::config::check_unused)
//! accept the section.
//!
//! | option | default | role |
//! |---|---|---|
//! | `screw1`..`screw99` | — | two floats each; **stops at the first missing index** |
//! | `screwN_name` | `screw at %.3f,%.3f` | the screw's display name |
//! | `screwN_fine_adjust` | — | the fine pass's coordinate for that screw |
//! | `speed` / `probe_speed` | `50.` / `5.` | the travels, both above `0` |
//! | `horizontal_move_z` / `probe_height` | `5.` / `0.` | the lift and the descent |
//!
//! **Not ported yet — command fidelity gap (H9)**: upstream registers
//! `BED_SCREWS_ADJUST` at load (`bed_screws.py:37-39`) and, per session, the
//! `ACCEPT`/`ADJUSTED`/`ABORT` trio (`:60-73`), moving the toolhead between
//! screws and reporting `Adjust <name>. Then run ACCEPT, ADJUSTED, or ABORT`.
//! None of that exists here; the corpus still passes because an unknown
//! command is answered with `Unknown command:"…"` and is not an error
//! (`gcode.rs:1068`). [`BedScrews::get_status`] therefore only ever answers
//! the rest state below, and the parsed values sit with the object for that
//! behavior unit.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("bed_screws", order = 30, load = load_config);

/// The `[bed_screws]` section: the screws, their fine positions, and the
/// travels (`bed_screws.py:8-34`).
///
/// The parsed values stay with the object so the `BED_SCREWS_ADJUST` behavior
/// unit starts from them; nothing reads them at runtime while the command
/// family is unported (the rest-state [`PrinterObject::get_status`] answers
/// constants), hence the `allow`.
#[allow(dead_code)]
#[derive(Debug)]
pub struct BedScrews {
    /// `(coordinate, name)` per `screwN`, in order (`bed_screws.py:15-22`).
    screws: Vec<((f64, f64), String)>,
    /// `(coordinate, name)` per `screwN_fine_adjust`, in order; the name is
    /// the coarse screw's own (`bed_screws.py:23-26`).
    fine_adjust: Vec<((f64, f64), String)>,
    /// The `speed` option, default `50.`, above `0` (`bed_screws.py:31`).
    speed: f64,
    /// The `probe_speed` option — upstream's `lift_speed`, default `5.`,
    /// above `0` (`bed_screws.py:32`).
    probe_speed: f64,
    /// The `horizontal_move_z` option, default `5.` (`bed_screws.py:33`).
    horizontal_move_z: f64,
    /// The `probe_height` option — upstream's `probe_z`, default `0.`
    /// (`bed_screws.py:34`).
    probe_height: f64,
}

impl BedScrews {
    /// Read the section in upstream's order (`bed_screws.py:13-34`): the
    /// screws (each with its name and fine coordinate), the three-screws
    /// floor, then the travels.
    ///
    /// # Errors
    /// A coordinate that is not two numbers (upstream's `getfloatlist` with
    /// `count=2`), fewer than three screws, or a travel outside its bound —
    /// all with upstream's wording.
    fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let mut screws = Vec::new();
        let mut fine_adjust = Vec::new();
        for index in 1..=99 {
            let prefix = format!("screw{index}");
            // `config.get(prefix, None) is None` breaks the scan
            // (`bed_screws.py:17-18`): a gap stops it even when a higher
            // screw exists.
            let Some(raw) = config.get_str(&prefix) else {
                break;
            };
            let coord = parse_coord(&prefix, &identifier, &raw)?;
            let default_name = format!("screw at {:.3},{:.3}", coord.0, coord.1);
            let name = config.get(&format!("{prefix}_name"), Some(&default_name))?;
            let fine_option = format!("{prefix}_fine_adjust");
            if let Some(fine_raw) = config.get_str(&fine_option) {
                let fine = parse_coord(&fine_option, &identifier, &fine_raw)?;
                fine_adjust.push((fine, name.clone()));
            }
            screws.push((coord, name));
        }
        if screws.len() < 3 {
            return Err(ConfigError::new(
                "bed_screws: Must have at least three screws".to_string(),
            ));
        }
        Ok(Self {
            screws,
            fine_adjust,
            speed: config.get_float_bounded("speed", Some(50.), None, None, Some(0.), None)?,
            probe_speed: config.get_float_bounded(
                "probe_speed",
                Some(5.),
                None,
                None,
                Some(0.),
                None,
            )?,
            horizontal_move_z: config.get_float("horizontal_move_z", Some(5.))?,
            probe_height: config.get_float("probe_height", Some(0.))?,
        })
    }
}

/// One coordinate option: two comma-separated floats — upstream's
/// `getfloatlist(option, count=2)` (`bed_screws.py:19/25`), whose parse runs
/// before the count check (`configfile.py:88-102`).
///
/// # Errors
/// `Unable to parse option …` for a value that is not a number, `must have 2
/// elements` for any other length.
fn parse_coord(option: &str, identifier: &str, raw: &str) -> Result<(f64, f64), ConfigError> {
    let parts: Vec<&str> = raw.split(',').map(str::trim).collect();
    let parsed: Vec<f64> = parts
        .iter()
        .map(|part| {
            part.parse::<f64>().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option '{option}' in section '{identifier}'"
                ))
            })
        })
        .collect::<Result<_, _>>()?;
    if parsed.len() != 2 {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{identifier}' must have 2 elements"
        )));
    }
    Ok((parsed[0], parsed[1]))
}

impl PrinterObject for BedScrews {
    /// The rest state upstream's `get_status` answers (`bed_screws.py:74-79`):
    /// `reset()` leaves `state` as `None`, so `is_active` is `false`, and no
    /// session can start while `BED_SCREWS_ADJUST` is unported.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "is_active": false,
            "state": null,
            "current_screw": 0,
            "accepted_screws": 0,
        })
    }
}

/// The factory `section!` names (`bed_screws.py:122 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(BedScrews::read(config)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config, ConfigSection, ConfigValue};

    /// A `[bed_screws]` section with the given options, as the parser would
    /// build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("bed_screws", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    // --- Config parsing ---------------------------------------------------

    /// Every option the corpus writes is read back through the tracker, so
    /// `check_unused` accepts the section — the option-level half of the
    /// loader's validation (`config/validate.rs:48`).
    #[test]
    fn every_option_the_section_writes_is_recorded_as_read() {
        let text = "\
[bed_screws]
screw1: 100,50
screw1_name: Front right
screw1_fine_adjust: 200,50
screw2: 75,75
screw2_fine_adjust: 200,75
screw3: 75,75
screw3_name: Last
screw3_fine_adjust: 75,90
speed: 20.
probe_speed: 2.
horizontal_move_z: 2.
probe_height: 0.5
";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("bed_screws").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = BedScrews::read(&wrapper).expect("the section reads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(
            parsed.screws,
            [
                ((100., 50.), "Front right".to_string()),
                ((75., 75.), "screw at 75.000,75.000".to_string()),
                ((75., 75.), "Last".to_string()),
            ]
        );
        // The fine pass pairs each coordinate with the coarse screw's name.
        assert_eq!(
            parsed.fine_adjust,
            [
                ((200., 50.), "Front right".to_string()),
                ((200., 75.), "screw at 75.000,75.000".to_string()),
                ((75., 90.), "Last".to_string()),
            ]
        );
        assert_eq!(parsed.speed, 20.);
        assert_eq!(parsed.probe_speed, 2.);
        assert_eq!(parsed.horizontal_move_z, 2.);
        assert_eq!(parsed.probe_height, 0.5);
    }

    /// With nothing but the screws, the four travels answer upstream's
    /// defaults and the names fall back to `screw at %.3f,%.3f`
    /// (`bed_screws.py:20,31-34`).
    #[test]
    fn the_travels_fall_back_to_their_upstream_defaults() {
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let parsed = BedScrews::read(&config).expect("three screws read");

        assert_eq!(parsed.speed, 50.);
        assert_eq!(parsed.probe_speed, 5.);
        assert_eq!(parsed.horizontal_move_z, 5.);
        assert_eq!(parsed.probe_height, 0.);
        assert_eq!(parsed.screws[0].1, "screw at 10.000,30.000");
        assert!(parsed.fine_adjust.is_empty());
    }

    /// `screwN` is read from 1 up to the first missing index: a gap stops
    /// the scan even when a higher screw exists, and a `screwN_fine_adjust`
    /// behind the gap is never read (`bed_screws.py:15-18`).
    #[test]
    fn screws_stop_at_the_first_missing_index() {
        let access = AccessTracking::shared();
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
            ("screw5", "10,190"),
            ("screw5_fine_adjust", "12,192"),
        ]);
        let config = ConfigWrapper::new(&sect, Arc::clone(&access));
        let parsed = BedScrews::read(&config).expect("three screws read");

        assert_eq!(
            parsed.screws.len(),
            3,
            "screw5 must not be read past the gap"
        );
        let settings = access.settings();
        let options = &settings["bed_screws"];
        assert!(options.get("screw4").is_none());
        assert!(options.get("screw5").is_none());
        assert!(options.get("screw5_fine_adjust").is_none());
    }

    /// Fewer than three screws is config error with upstream's wording, a
    /// coordinate of any other length keeps the `getfloatlist` count wording,
    /// and a non-number keeps the parser's wording (`bed_screws.py:27-28`,
    /// `configfile.py:49/100-102`).
    #[test]
    fn malformed_screws_keep_upstream_wording() {
        let sect = section(&[("screw1", "10,30"), ("screw2", "155,30")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            BedScrews::read(&config).unwrap_err().to_string(),
            "bed_screws: Must have at least three screws"
        );

        let sect = section(&[
            ("screw1", "10"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            BedScrews::read(&config).unwrap_err().to_string(),
            "Option 'screw1' in section 'bed_screws' must have 2 elements"
        );

        let sect = section(&[
            ("screw1", "left,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            BedScrews::read(&config).unwrap_err().to_string(),
            "Unable to parse option 'screw1' in section 'bed_screws'"
        );

        // The fine coordinate is parsed the same way.
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
            ("screw2_fine_adjust", "200"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            BedScrews::read(&config).unwrap_err().to_string(),
            "Option 'screw2_fine_adjust' in section 'bed_screws' must have 2 elements"
        );
    }

    /// `speed`/`probe_speed` must be above `0` with upstream's bound wording,
    /// and an unparseable travel keeps the parser's wording
    /// (`bed_screws.py:31-32`, `configfile.py:49`).
    #[test]
    fn the_travels_are_bounded_the_way_upstream_bounds_them() {
        // The three-screws floor is checked first (`bed_screws.py:27-31`),
        // so every case keeps a full screw list and varies a travel.
        let screws: [(&str, &str); 3] = [
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ];
        let with = |travel: (&'static str, &'static str)| {
            section(&[screws[0], screws[1], screws[2], travel])
        };

        let sect = with(("speed", "0"));
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            BedScrews::read(&config).unwrap_err().to_string(),
            "Option 'speed' in section 'bed_screws' must be above 0"
        );

        let sect = with(("probe_speed", "-1"));
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            BedScrews::read(&config).unwrap_err().to_string(),
            "Option 'probe_speed' in section 'bed_screws' must be above 0"
        );

        let sect = with(("horizontal_move_z", "above"));
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            BedScrews::read(&config).unwrap_err().to_string(),
            "Unable to parse option 'horizontal_move_z' in section 'bed_screws'"
        );
    }

    // --- Status -----------------------------------------------------------

    /// The status is upstream's rest state (`bed_screws.py:74-79`) — no
    /// session can be active while `BED_SCREWS_ADJUST` is unported.
    #[test]
    fn the_status_answers_upstreams_rest_state() {
        let sect = section(&[
            ("screw1", "10,30"),
            ("screw2", "155,30"),
            ("screw3", "155,190"),
        ]);
        let config = ConfigWrapper::untracked(&sect);
        let parsed = BedScrews::read(&config).expect("three screws read");

        assert_eq!(
            parsed.get_status(0.),
            json!({
                "is_active": false,
                "state": null,
                "current_screw": 0,
                "accepted_screws": 0,
            })
        );
    }
}
