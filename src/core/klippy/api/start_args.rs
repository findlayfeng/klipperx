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
//! Compared to upstream's dictionary, `debuginput` (file input) and the per-MCU
//! dictionary paths are absent: this host implements neither (see
//! `docs/klippy/developer-manual/upstream-deviations.md`). `debugoutput` stays
//! even though the `-o` option does not exist, because the regression harness
//! fills it to put the machine into upstream's file-output mode.

/// What the host process was started with.
///
/// `info` reports `log_file`, `config_file`, `software_version` and `cpu_info`
/// straight from here — the four fields upstream reads out of `start_args`
/// rather than gathering in the request handler (`klippy/webhooks.py:395-397`).
/// The rest mirror the dictionary upstream's `main()` builds
/// (`klippy/klippy.py:288-338`), so that a module can ask the printer for the
/// start arguments instead of taking them as parameters.
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
    /// The `--api-server` value (upstream's `apiserver`).
    pub apiserver: Option<String>,
    /// Why this run started: `"startup"` for the first one, then the result
    /// the previous run ended with (upstream's `start_args['start_reason']`).
    pub start_reason: String,
    /// The `--debugoutput` file, when the host writes the MCU protocol to a
    /// file instead of the serial port (`debugoutput`).
    pub debug_output: Option<String>,
    /// The board description (upstream's `device`, `util.get_device_info`).
    pub device: String,
    /// The kernel version (upstream's `linux_version`, `util.get_linux_version`).
    pub linux_version: String,
}

impl StartArgs {
    /// Gather the start arguments a host knows at startup.
    ///
    /// `log_file` is the `--logfile` path, or `None` when the host logs to the
    /// terminal only. `debug_output` stays `None` until the parser carries
    /// `--debugoutput`, and the API address is filled in by the host, which is
    /// where the option lives. Upstream's `--debuginput` file-input mode is not
    /// implemented here, so there is no field for it (see
    /// `docs/klippy/developer-manual/upstream-deviations.md`).
    pub fn collect(config_file: impl Into<String>, log_file: Option<String>) -> Self {
        Self {
            config_file: config_file.into(),
            log_file,
            software_version: env!("CARGO_PKG_VERSION").to_string(),
            cpu_info: cpu_info(),
            apiserver: None,
            start_reason: "startup".to_string(),
            debug_output: None,
            device: device_info(),
            linux_version: linux_version(),
        }
    }
}

/// This machine's CPU description, as upstream's `util.get_cpu_info` writes it
/// (`klippy/util.py:116-124`): the count of `processor` entries and the `model name`,
/// as `"{n} core {model}"`.
///
/// `"?"` when `/proc/cpuinfo` cannot be read, which is upstream's answer too.
fn cpu_info() -> String {
    match std::fs::read_to_string("/proc/cpuinfo") {
        Ok(data) => parse_cpu_info(&data),
        Err(_) => "?".to_string(),
    }
}

/// The board description, as upstream's `util.get_device_info` writes it
/// (`klippy/util.py:126-132`): the device tree model, else the DMI product
/// name, else `"?"`.
fn device_info() -> String {
    for path in ["/proc/device-tree/model", "/sys/class/dmi/id/product_name"] {
        if let Ok(data) = std::fs::read_to_string(path) {
            return data
                .trim_matches(|c: char| c == ' ' || c == '\0')
                .trim()
                .to_string();
        }
    }
    "?".to_string()
}

/// The kernel version, as upstream's `util.get_linux_version` writes it
/// (`klippy/util.py:134-138`): `/proc/version` verbatim, else `"?"`.
fn linux_version() -> String {
    std::fs::read_to_string("/proc/version")
        .map(|data| data.trim().to_string())
        .unwrap_or_else(|_| "?".to_string())
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

    #[test]
    fn test_a_fresh_run_starts_for_the_first_reason() {
        // Upstream's `main()`: `start_args = {..., 'start_reason': 'startup'}`
        // (`klippy/klippy.py:288`).
        let args = StartArgs::collect("/tmp/printer.cfg", None);

        assert_eq!(args.start_reason, "startup");
        assert_eq!(args.apiserver, None);
        // Upstream's `--debuginput` file-input mode is not implemented here, so
        // there is no field for it
        // (`docs/klippy/developer-manual/upstream-deviations.md`); the host
        // reads G-Code from its pty. `--debugoutput` stays pending: the
        // protocol goes to the serial port for now.
        assert_eq!(args.debug_output, None);
        // Both are read from the running kernel, so only their shape is pinned.
        assert!(!args.device.is_empty());
        assert!(!args.linux_version.is_empty());
    }
}
