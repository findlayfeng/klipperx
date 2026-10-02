//! The write half of `SAVE_CONFIG`: turn the command into a rewritten config
//! file, backed up, that the next restart reads.
//!
//! Upstream's `ConfigAutoSave.cmd_SAVE_CONFIG` (`klippy/configfile.py:346-402`)
//! appends the block fileconfig (see [`super::Config::autosave_block`]) to the
//! regular body, comments out body copies of options the block also defines,
//! validates the result still parses, then swaps temp → backup → main with a
//! timestamped backup and requests a restart. This module holds that
//! write-back, kept free of the `gcode`/`printer` layers so it can be tested on
//! its own; the `SAVE_CONFIG` command registration and the restart live in the
//! config loader.

use crate::core::klippy::config::object::PrinterConfig;
use crate::core::klippy::config::{build_autosave_block, split_autosave, Config};

/// Write the block fileconfig back into `cfgname`, backing up the old file.
///
/// Mirrors upstream `cmd_SAVE_CONFIG` (`klippy/configfile.py:346-402`): read
/// the on-disk file, split off and replace its `SAVE_CONFIG` block, comment
/// out regular copies of the block's options, re-validate the block survived,
/// then swap via a temp file with a timestamped backup. `Ok(())` means the
/// file was rewritten; the caller then triggers the restart.
pub fn write_config(configfile: &PrinterConfig, cfgname: &str) -> Result<(), String> {
    // Upstream: `if not self.fileconfig.sections(): return` — nothing saved.
    if !configfile.has_pending_sections() {
        return Ok(());
    }
    let Some(fileconfig) = configfile.autosave_fileconfig() else {
        return Ok(());
    };
    let autosave_data = build_autosave_block(&fileconfig);

    let data = std::fs::read_to_string(cfgname)
        .map_err(|_| "Unable to read existing config on SAVE_CONFIG".to_string())?;
    let (regular_data, _old_autosave) = split_autosave(&data);
    let regular_data = strip_regular_duplicates(regular_data, &fileconfig);
    // `regular_data.rstrip() + autosave_data` — the block brings its own
    // leading newline, so no separator is added (upstream, `:366`).
    let data = format!("{}{}", regular_data.trim_end(), autosave_data);

    // A `SAVE_CONFIG` must leave a parseable block behind; otherwise the edit
    // is abandoned (upstream `:369-374`).
    let (_new_regular, new_autosave) = split_autosave(&data);
    if new_autosave.is_none() {
        return Err("Existing config autosave is corrupted. Can't complete SAVE_CONFIG".into());
    }

    let datestr = datestr();
    let (backup_name, temp_name) = if cfgname.ends_with(".cfg") {
        let base = &cfgname[..cfgname.len() - 4];
        (
            format!("{base}{datestr}.cfg"),
            format!("{base}_autosave.cfg"),
        )
    } else {
        (format!("{cfgname}{datestr}"), format!("{cfgname}_autosave"))
    };
    (|| -> std::io::Result<()> {
        std::fs::write(&temp_name, &data)?;
        std::fs::rename(cfgname, &backup_name)?;
        std::fs::rename(&temp_name, cfgname)?;
        Ok(())
    })()
    .map_err(|_| "Unable to write config file during SAVE_CONFIG".to_string())?;

    Ok(())
}

/// Comment out, in the regular body text, every option the block fileconfig
/// also defines — upstream's `_strip_duplicates(data, fileconfig)` run on the
/// body (`configfile.py:273-294`, called at `:366`). The block wins on re-read,
/// so the body's duplicate copy is commented out.
fn strip_regular_duplicates(regular: &str, fileconfig: &Config) -> String {
    fn cut_comment(line: &str) -> &str {
        match line.find(['#', ';']) {
            Some(index) => &line[..index],
            None => line,
        }
    }
    let mut section: Option<String> = None;
    let mut is_dup_field = false;
    let mut out = Vec::new();
    for line in regular.split('\n') {
        let pruned = cut_comment(line).trim_end();
        if pruned.is_empty() {
            out.push(line.to_string());
            continue;
        }
        if pruned.starts_with(char::is_whitespace) {
            out.push(if is_dup_field {
                format!("#{line}")
            } else {
                line.to_string()
            });
            continue;
        }
        is_dup_field = false;
        if pruned.starts_with('[') {
            section = pruned
                .strip_prefix('[')
                .and_then(|rest| rest.strip_suffix(']'))
                .map(str::trim)
                .map(str::to_string);
            out.push(line.to_string());
            continue;
        }
        let field: String = pruned
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        let defined = section
            .as_deref()
            .and_then(|name| fileconfig.get_section(name))
            .is_some_and(|s| s.has(&field));
        if defined {
            is_dup_field = true;
            out.push(format!("#{line}"));
        } else {
            out.push(line.to_string());
        }
    }
    out.join("\n")
}

/// A `-YYYYMMDD_HHMMSS` suffix for the backup name, in UTC (upstream uses
/// local time; the filename just needs to be readable and unique).
fn datestr() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let days = (secs / 86_400) as i64;
    let sod = secs % 86_400;
    // Civil-from-days (Howard Hinnant's algorithm), UTC.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let yy = if m <= 2 { y + 1 } else { y };
    let (h, mi, s) = (sod / 3600, (sod % 3600) / 60, sod % 60);
    format!("{yy:04}{m:02}{d:02}_{h:02}{mi:02}{s:02}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use serde_json::Map;
    use std::fs;

    const HEADER: &str = concat!(
        "\n#*# <---------------------- SAVE_CONFIG ---------------------->\n",
        "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n",
        "#*#\n",
    );

    /// Build a `PrinterConfig` whose block fileconfig is parsed from `text`.
    fn object(text: &str) -> PrinterConfig {
        let (config, _) = Config::from_text(text).expect("the config parses");
        PrinterConfig::new_with_autosave(
            AccessTracking::shared(),
            Map::new(),
            config.autosave_block().cloned(),
        )
    }

    fn temp_file(name: &str, content: &str) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!("savecfg_{}_{}", std::process::id(), name));
        fs::write(&path, content).expect("writes the temp config");
        path
    }

    #[test]
    fn write_config_appends_pending_values_and_backs_up() {
        let path = temp_file(
            "write_back.cfg",
            &format!(
                "[probe]\nz_offset: 1.0\ncalibrate: 0\n{HEADER}#*# [probe]\n#*# x_offset = 2.0\n"
            ),
        );
        let cfgname = path.to_str().unwrap().to_string();

        let configfile = object(&format!(
            "[probe]\nz_offset: 1.0\n{HEADER}#*# [probe]\n#*# x_offset = 2.0\n"
        ));
        configfile.set("probe", "z_offset", "3.5");

        write_config(&configfile, &cfgname).expect("SAVE_CONFIG writes");

        let written = fs::read_to_string(&cfgname).expect("rewritten file readable");
        // The body keeps its original z_offset; the block gains the new value
        // alongside the pre-existing x_offset.
        assert!(
            written.contains("z_offset: 1.0"),
            "body preserved: {written}"
        );
        assert!(
            written.contains("#*# x_offset = 2.0"),
            "old block value kept: {written}"
        );
        assert!(
            written.contains("#*# z_offset = 3.5"),
            "new value written: {written}"
        );
        assert!(written.contains("SAVE_CONFIG"), "header present: {written}");

        // A timestamped backup exists and holds the pre-write content.
        let backup = cfgname.trim_end_matches(".cfg").to_string() + &datestr() + ".cfg";
        assert!(
            std::path::Path::new(&backup).exists(),
            "backup {backup} exists"
        );
        let backup_body = fs::read_to_string(&backup).expect("backup readable");
        assert!(backup_body.contains("z_offset: 1.0"));

        let _ = fs::remove_file(&cfgname);
        let _ = fs::remove_file(&backup);
    }

    #[test]
    fn write_config_writes_a_block_when_the_file_had_none() {
        let path = temp_file("fresh_block.cfg", "[probe]\nz_offset: 1.0\n");
        let cfgname = path.to_str().unwrap().to_string();

        let configfile = object("[probe]\nz_offset: 1.0\n");
        configfile.set("probe", "z_offset", "2.5");

        write_config(&configfile, &cfgname).expect("SAVE_CONFIG writes a fresh block");

        let written = fs::read_to_string(&cfgname).expect("rewritten file readable");
        assert!(
            written.contains("#*# z_offset = 2.5"),
            "new block: {written}"
        );
        assert!(written.contains("z_offset: 1.0"), "body preserved");

        let backup = cfgname.trim_end_matches(".cfg").to_string() + &datestr() + ".cfg";
        let _ = fs::remove_file(&cfgname);
        let _ = fs::remove_file(&backup);
    }

    #[test]
    fn strip_regular_duplicates_comments_out_body_copies_of_block_options() {
        let (block_config, _) = Config::from_text("[probe]\nz_offset: 1\nx_offset: 2\n").unwrap();
        let body = "[probe]\nz_offset: 1\npin: PA0\n";

        let stripped = strip_regular_duplicates(body, &block_config);

        // `z_offset` also lives in the block → commented; `pin` does not → left.
        assert!(stripped.contains("#z_offset: 1"), "{stripped}");
        assert!(stripped.contains("pin: PA0"), "{stripped}");
    }

    #[test]
    fn a_save_config_with_nothing_pending_is_a_noop() {
        let path = temp_file("noop.cfg", "[probe]\nz_offset: 1.0\n");
        let cfgname = path.to_str().unwrap().to_string();

        let configfile = object("[probe]\nz_offset: 1.0\n");

        write_config(&configfile, &cfgname).expect("no-op succeeds");

        let written = fs::read_to_string(&cfgname).expect("file untouched");
        assert_eq!(written, "[probe]\nz_offset: 1.0\n");
        let _ = fs::remove_file(&cfgname);
    }
}
