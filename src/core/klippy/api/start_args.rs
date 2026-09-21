//! Host start arguments: what the host process was started with.
//!
//! Upstream keeps this dictionary on the printer (`klippy/klippy.py:283`) and
//! fills it in `main()` before the printer exists, so that any module can ask
//! `printer.get_start_args()` (29 call sites) for the config path, the log file,
//! the CPU description and the rest. It is host data, not machine state: it does
//! not change while the host runs, it is never reported as a printer object, and
//! it outlives any one printer. It therefore lives on the host's side of the
//! API, and the host builds it.
//!
//! Only the fields the `info` endpoint reports are here so far. Upstream's
//! dictionary also carries `apiserver`, `start_reason`, the debug input/output
//! and the per-MCU dictionary paths; they arrive with the modules that read
//! them.

/// What the host process was started with.
///
/// `info` reports `log_file`, `config_file`, `software_version` and `cpu_info`
/// straight from here — the four fields upstream reads out of `start_args`
/// rather than gathering in the request handler (`klippy/webhooks.py:395-397`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StartArgs {
    /// The config file the host was started with.
    pub config_file: String,
    /// The log file, or `None` when the host is not logging to a file.
    ///
    /// `None` until a `--logfile` exists: the host logs to stdout, and upstream
    /// reports `null` for exactly this case.
    pub log_file: Option<String>,
    /// Version of the host software.
    pub software_version: String,
    /// CPU description, e.g. `"4 core ARMv7 Processor rev 4 (v7l)"`.
    pub cpu_info: String,
}

impl StartArgs {
    /// Gather the start arguments a host knows at startup.
    ///
    /// `log_file` is the `--logfile` path, or `None` when the host logs to the
    /// terminal only.
    pub fn collect(config_file: impl Into<String>, log_file: Option<String>) -> Self {
        Self {
            config_file: config_file.into(),
            log_file,
            software_version: env!("CARGO_PKG_VERSION").to_string(),
            cpu_info: cpu_info(),
        }
    }
}

/// This machine's CPU description, as upstream's `util.get_cpu_info` writes it
/// (`klippy/util.py:116`): the count of `processor` entries and the `model name`,
/// as `"{n} core {model}"`.
///
/// `"?"` when `/proc/cpuinfo` cannot be read, which is upstream's answer too.
fn cpu_info() -> String {
    match std::fs::read_to_string("/proc/cpuinfo") {
        Ok(data) => parse_cpu_info(&data),
        Err(_) => "?".to_string(),
    }
}

/// The part of [`cpu_info`] that can be tested without a `/proc`.
fn parse_cpu_info(data: &str) -> String {
    let mut cores = 0usize;
    let mut model = None;
    for line in data.lines() {
        let Some((key, value)) = line.split_once(':') else {
            continue;
        };
        match key.trim() {
            "processor" => cores += 1,
            "model name" => model = Some(value.trim().to_string()),
            _ => {}
        }
    }
    format!("{cores} core {}", model.unwrap_or_else(|| "?".to_string()))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_the_cpu_description_is_the_processor_count_and_the_model() {
        let data = "\
processor\t: 0
model name\t: ARMv7 Processor rev 4 (v7l)
processor\t: 1
model name\t: ARMv7 Processor rev 4 (v7l)
";

        assert_eq!(parse_cpu_info(data), "2 core ARMv7 Processor rev 4 (v7l)");
    }

    #[test]
    fn test_a_proc_without_a_model_name_still_reports_the_count() {
        // What upstream does with the same input: `dict(lines).get(..., "?")`.
        assert_eq!(parse_cpu_info("processor\t: 0\n"), "1 core ?");
    }

    #[test]
    fn test_collected_start_args_carry_the_config_file_and_a_version() {
        let args = StartArgs::collect("/tmp/printer.cfg", None);

        assert_eq!(args.config_file, "/tmp/printer.cfg");
        assert_eq!(args.log_file, None);
        assert_eq!(args.software_version, env!("CARGO_PKG_VERSION"));
        // The CPU description depends on the machine, so only its shape is
        // pinned here; `parse_cpu_info` above covers the text.
        assert!(args.cpu_info.contains("core"), "{}", args.cpu_info);
    }

    #[test]
    fn test_a_log_file_reaches_the_start_args() {
        let args = StartArgs::collect("/tmp/printer.cfg", Some("/tmp/klippy.log".to_string()));

        assert_eq!(args.log_file.as_deref(), Some("/tmp/klippy.log"));
    }
}
