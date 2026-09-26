//! The undefined-option check: every section and option must have been read.
//!
//! Upstream's `ConfigValidate.check_unused` (`klippy/configfile.py:424-445`)
//! runs after the config is loaded and uses the access record as the schema:
//!
//! - a section is valid when a printer object was registered for it **or** at
//!   least one of its options was read — the second is how `[printer]` is
//!   accepted, since its consumer registers as `toolhead`, not `printer`;
//! - an option is valid exactly when someone read it **or** the `SAVE_CONFIG`
//!   block wrote it: `start_access_tracking` folds the autosave fileconfig into
//!   the access set (`configfile.py:416-422`), so `SAVE_CONFIG`-written options
//!   are never undefined. This is what lets a saved `[stepper_a] lower_arm`
//!   pass even though the kinematics reads `lower_arm_length`
//!   (`test/klippy/rotary_delta_calibrate.cfg`).
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
        // A section is valid when a printer object claimed it, someone read one
        // of its options, or the `SAVE_CONFIG` block wrote one — the autosave
        // options joined the access set upstream before this ran, so a section
        // they name is valid on their account.
        if !valid.contains(&id) && !section.has_autosave_options() {
            return Err(ConfigError::new(format!(
                "Section '{id}' is not a valid config section"
            )));
        }
        for option in section.parameters.keys() {
            let option = option.to_lowercase();
            if !access.contains(&id, &option) && !section.is_autosave_option(&option) {
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

    // -----------------------------------------------------------------------
    // `SAVE_CONFIG` options are exempt (`configfile.py:416-430`)
    // -----------------------------------------------------------------------

    /// The block header as upstream writes it (`configfile.py:233-237`).
    const AUTOSAVE_HEADER: &str = concat!(
        "\n#*# <---------------------- SAVE_CONFIG ---------------------->\n",
        "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n",
        "#*#\n",
    );

    fn autosave_config(body: &str, block: &str) -> Config {
        Config::from_text(&format!("{body}{AUTOSAVE_HEADER}{block}"))
            .expect("the test config parses")
            .0
    }

    #[test]
    fn an_option_only_the_autosave_block_wrote_is_accepted_unread() {
        // test/klippy/rotary_delta_calibrate.cfg: the block saves `lower_arm`
        // into `[stepper_a]`, but the kinematics reads `lower_arm_length` — no
        // section ever reads `lower_arm`. Upstream merges the block's options
        // into the access set, so it is not an undefined option.
        let config = autosave_config(
            "[stepper_a]\nupper_arm_length: 170\n",
            "#*# [stepper_a]\n#*# lower_arm = 320.000011\n",
        );
        let access = AccessTracking::new();
        access.note("stepper_a", "upper_arm_length", json!(170.0));

        check_unused(&config, &access, &["stepper_a".to_string()]).unwrap();
    }

    #[test]
    fn an_option_the_autosave_block_wrote_and_a_section_read_still_passes() {
        // No regression: a block option that *is* read (the usual case for a
        // saved calibration) passes exactly as before.
        let config = autosave_config(
            "[printer]\nkinematics: delta\n",
            "#*# [printer]\n#*# delta_radius = 174.750004\n",
        );
        let access = AccessTracking::new();
        access.note("printer", "kinematics", json!("delta"));
        access.note("printer", "delta_radius", json!(174.75));

        check_unused(&config, &access, &[]).unwrap();
    }

    #[test]
    fn a_plain_unread_option_is_still_rejected() {
        // The exemption is scoped to the block: the very same option written in
        // the body stays an undefined option, so `check_unused` keeps its
        // meaning for ordinary configs.
        let config = config("[stepper_a]\nupper_arm_length: 170\nlower_arm: 320\n");
        let access = AccessTracking::new();
        access.note("stepper_a", "upper_arm_length", json!(170.0));

        let err = check_unused(&config, &access, &["stepper_a".to_string()]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'lower_arm' is not valid in section 'stepper_a'"
        );
    }

    #[test]
    fn a_body_option_duplicated_in_the_block_is_not_exempt() {
        // `_strip_duplicates` drops the block's copy when the body defines the
        // option (`configfile.py:273-294`), so the body's copy is *not* in the
        // autosave set and must still be read — the exemption cannot be bought
        // by echoing an option into the block.
        let config = autosave_config(
            "[stepper_a]\nupper_arm_length: 170\nlower_arm: 320\n",
            "#*# [stepper_a]\n#*# lower_arm = 320.000011\n",
        );
        let access = AccessTracking::new();
        access.note("stepper_a", "upper_arm_length", json!(170.0));

        let err = check_unused(&config, &access, &["stepper_a".to_string()]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'lower_arm' is not valid in section 'stepper_a'"
        );
    }
}
