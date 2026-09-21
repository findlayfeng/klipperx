//! The undefined-option check: every section and option must have been read.
//!
//! Upstream's `ConfigValidate.check_unused` (`klippy/configfile.py:424-445`)
//! runs after the config is loaded and uses the access record as the schema:
//!
//! - a section is valid when a printer object was registered for it **or** at
//!   least one of its options was read — the second is how `[printer]` is
//!   accepted, since its consumer registers as `toolhead`, not `printer`;
//! - an option is valid exactly when someone read it.
//!
//! Names are compared lowercased because that is how `configparser` treats
//! them upstream; this crate's parser preserves case, so the lowercasing is
//! done here.

use crate::core::klippy::config::access::AccessTracking;
use crate::core::klippy::config::Config;
use crate::core::klippy::error::ConfigError;

/// Reject every section and option nobody read.
///
/// `claimed` is the identifiers of the sections the loader registered (an
/// object named `toolhead` for a `[printer]` section still claims `printer`).
///
/// # Errors
/// Returns upstream's wording:
/// `Section 'x' is not a valid config section`, `Option 'x' is not valid in
/// section 'y'`.
pub fn check_unused(
    config: &Config,
    access: &AccessTracking,
    claimed: &[String],
) -> Result<(), ConfigError> {
    let mut valid: std::collections::BTreeSet<String> =
        claimed.iter().map(|id| id.to_lowercase()).collect();
    valid.extend(access.sections());

    for section in config.sections() {
        let id = section.identifier().to_lowercase();
        if !valid.contains(&id) {
            return Err(ConfigError::new(format!(
                "Section '{id}' is not a valid config section"
            )));
        }
        for option in section.parameters.keys() {
            let option = option.to_lowercase();
            if !access.contains(&id, &option) {
                return Err(ConfigError::new(format!(
                    "Option '{option}' is not valid in section '{id}'"
                )));
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn config(text: &str) -> Config {
        Config::from_text(text).expect("the test config parses").0
    }

    #[test]
    fn a_section_nobody_claimed_or_read_is_rejected() {
        let config = config("[made_up]\nvalue: 1\n");
        let err = check_unused(&config, &AccessTracking::new(), &[]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Section 'made_up' is not a valid config section"
        );
    }

    #[test]
    fn a_read_only_section_is_valid_without_an_object() {
        // `[printer]` has no object named `printer`; its consumer reads the
        // options and registers as `toolhead`.
        let config = config("[printer]\nmax_velocity: 500\n");
        let access = AccessTracking::new();
        access.note("printer", "max_velocity", json!(500.0));

        check_unused(&config, &access, &[]).unwrap();
    }

    #[test]
    fn an_option_nobody_read_is_rejected() {
        let config = config("[output_pin fan]\npin: PA0\npinn: PA1\n");
        let access = AccessTracking::new();
        access.note("output_pin fan", "pin", json!("PA0"));

        let err = check_unused(&config, &access, &["output_pin fan".to_string()]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'pinn' is not valid in section 'output_pin fan'"
        );
    }

    #[test]
    fn a_claimed_section_with_all_options_read_passes() {
        let config = config("[mcu]\nserial: /dev/a\nbaud: 250000\n");
        let access = AccessTracking::new();
        access.note("mcu", "serial", json!("/dev/a"));
        access.note("mcu", "baud", json!(250000));

        check_unused(&config, &access, &["mcu".to_string()]).unwrap();
    }

    #[test]
    fn names_are_compared_case_insensitively() {
        let config = config("[MCU]\nSerial: /dev/a\n");
        let access = AccessTracking::new();
        access.note("mcu", "serial", json!("/dev/a"));

        check_unused(&config, &access, &["mcu".to_string()]).unwrap();
    }
}
