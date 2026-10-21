//! `system_stats` — the host process's load, as one printer object.
//!
//! Upstream's `extras/statistics.py` has two halves: `PrinterSysStats`, which
//! collects the numbers and registers itself as the `system_stats` object, and
//! `PrinterStats`, which logs them once a second. Both are here, in separate
//! modules: this one is the numbers, and
//! [`printer_stats`](crate::core::klippy::extras::printer_stats) is the 1 Hz
//! `Stats` line that publishes them (`PrinterObject::stats`). This object also
//! carries the reactor dispatcher's round totals (`DispatchStats`) as a
//! `dispatch` sub-object, which upstream's has no counterpart for.
//!
//! There is no `[system_stats]` section: upstream builds the object from
//! `load_config` as a library object (`statistics.py:58-60`), and here the host
//! registers it unconditionally at startup, the way it registers `webhooks`
//! (`klippy.rs`). It is a host object, so it survives a `RESTART` and its
//! dispatcher totals keep counting across one.
//!
//! # Why the reads are inline
//!
//! `get_status` is where the numbers are read. Upstream caches them in the
//! object: `stats` recomputes the load average and the CPU total every second
//! and `get_status` reports the cached values (`statistics.py:24-33`, `:47-50`).
//! Here every number is read where it is reported — `get_status` when a client
//! asks (a subscription is 1 Hz), [`PrinterObject::stats`] when the line is made
//! — and reading `/proc` is a few microseconds with no lock, so reading it is
//! cheaper than keeping a cache for it, and there is nothing to go stale. It is
//! not work that blocks the dispatcher: `get_status` runs on the thread of
//! whoever asked, never on the reactor's own thread.
//!
//! [`DispatchStats`]: crate::core::klippy::reactor::DispatchStats

use std::sync::{Arc, OnceLock};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::DispatchStats;

/// The printer object's name, upstream's `add_object('system_stats', …)`
/// (`statistics.py:59`).
pub const SYSTEM_STATS_OBJECT: &str = "system_stats";

/// The system load average, first field — upstream's `os.getloadavg()[0]`
/// (`statistics.py:29`).
const LOADAVG_PATH: &str = "/proc/loadavg";

/// The process's own CPU time, fields `utime` + `stime` — upstream's
/// `time.process_time()` (`statistics.py:26`).
const PROC_SELF_STAT_PATH: &str = "/proc/self/stat";

/// Where `MemAvailable` comes from — upstream reads the same file
/// (`statistics.py:12`, `:38`).
const MEMINFO_PATH: &str = "/proc/meminfo";

/// Nanoseconds in a second, for turning the dispatcher's totals into seconds.
const NS_PER_S: f64 = 1e9;

/// The host's own load, as a printer object.
pub struct SystemStats {
    /// The dispatcher's running totals, shared with the reactor.
    dispatch: Arc<DispatchStats>,
}

impl SystemStats {
    /// Register the object, or return the one already registered.
    ///
    /// Installed by the host at startup, before the config is read, so it is a
    /// host object: a restart keeps it (and the totals it publishes) rather
    /// than rebuilding it.
    ///
    /// # Errors
    /// The name is already taken by a different object.
    pub fn install(
        printer: &Arc<Printer>,
        dispatch: Arc<DispatchStats>,
    ) -> Result<Arc<Self>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<Self>(SYSTEM_STATS_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(Self { dispatch });
        printer.add_object(
            SYSTEM_STATS_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
    }

    /// The `dispatch` sub-object: the reactor dispatcher's round totals in
    /// seconds.
    ///
    /// The averages are `0.0` before the first round (no division by zero), and
    /// `threshold_s` is `0.0` while no latency notifier is installed — nothing
    /// is counted as over a threshold that does not exist yet.
    fn dispatch_status(&self) -> Value {
        let snapshot = self.dispatch.snapshot();
        let average = |sum_ns: u64| match snapshot.rounds {
            0 => 0.0,
            rounds => sum_ns as f64 / rounds as f64 / NS_PER_S,
        };
        json!({
            "rounds": snapshot.rounds,
            "busy_avg_s": average(snapshot.busy_sum_ns),
            "busy_max_s": snapshot.busy_max_ns as f64 / NS_PER_S,
            "lateness_avg_s": average(snapshot.lateness_sum_ns),
            "lateness_max_s": snapshot.lateness_max_ns as f64 / NS_PER_S,
            "over_threshold": snapshot.over_threshold,
            "threshold_s": if snapshot.threshold_ns == u64::MAX {
                0.0
            } else {
                snapshot.threshold_ns as f64 / NS_PER_S
            },
        })
    }
}

impl PrinterObject for SystemStats {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "sysload": sysload(),
            "cputime": cputime(),
            "memavail": memavail(),
            // Our own addition (upstream's `system_stats` has no log counters):
            // records dropped because the writer's queue was full. A card-sized
            // board writing to eMMC/SD is exactly where this can become
            // non-zero, and a silent drop is what would make a log look
            // complete while it is not.
            "dropped_logs": crate::logging::dropped_records(),
            "dispatch": self.dispatch_status(),
        })
    }

    /// Upstream's `PrinterSysStats.stats` (`statistics.py:24-46`): the three
    /// numbers as one `key=value` run, always flagged `false` — they are added
    /// to somebody else's `Stats` line but never make one
    /// (`statistics.py:67-69`).
    ///
    /// `cputime` is the process's CPU time so far, which is what upstream's
    /// running total comes to: it sums the positive diffs of
    /// `time.process_time()`, a clock that only moves forward, so the sum is the
    /// current value. `memavail` is always reported; upstream leaves the field
    /// out when `/proc/meminfo` cannot be read (`statistics.py:35-46`), and here
    /// that read's `0` default — the value `get_status` publishes too — takes
    /// its place.
    fn stats(&self) -> Option<(bool, String)> {
        Some((
            false,
            format!(
                "sysload={:.2} cputime={:.3} memavail={}",
                sysload(),
                cputime(),
                memavail()
            ),
        ))
    }
}

/// The system load average's first field, or `0.0` if `/proc` cannot be read —
/// upstream's `last_load_avg` starts at `0.` and only moves on a successful read
/// (`statistics.py:17`, `:29`).
fn sysload() -> f64 {
    std::fs::read_to_string(LOADAVG_PATH)
        .ok()
        .and_then(|data| first_loadavg_field(&data))
        .unwrap_or(0.0)
}

/// This process's CPU time in seconds, `utime` + `stime` from `/proc/self/stat`.
///
/// Upstream reports `time.process_time()`, the whole process's CPU time
/// (`statistics.py:26`); the kernel's two fields are the same number, and they
/// are clock ticks, so they are divided by `_SC_CLK_TCK`. Rounded to the
/// millisecond upstream prints it with (`"%.3f"`, `statistics.py:32`), so the
/// value a client sees does not wobble in the last digits.
fn cputime() -> f64 {
    let Ok(stat) = std::fs::read_to_string(PROC_SELF_STAT_PATH) else {
        return 0.0;
    };
    let Some(ticks) = process_ticks(&stat) else {
        return 0.0;
    };
    let seconds = ticks as f64 / clock_ticks_per_second() as f64;
    (seconds * 1000.0).round() / 1000.0
}

/// `MemAvailable` in kB, or `0` if it cannot be read (upstream keeps its `0`
/// default the same way, `statistics.py:19`).
fn memavail() -> u64 {
    std::fs::read_to_string(MEMINFO_PATH)
        .ok()
        .and_then(|data| mem_available_kb(&data))
        .unwrap_or(0)
}

/// The kernel's clock ticks per second (`_SC_CLK_TCK`), read once.
///
/// The only way to turn `/proc/self/stat`'s `utime`/`stime` into seconds.
/// `sysconf` cannot fail for this parameter on Linux; 100 is the value every
/// Linux port uses if it somehow does.
fn clock_ticks_per_second() -> u64 {
    static TICKS: OnceLock<u64> = OnceLock::new();
    *TICKS.get_or_init(|| {
        // SAFETY: `sysconf` reads a kernel-provided constant and takes one
        // argument; it cannot fail for `_SC_CLK_TCK`.
        let ticks = unsafe { libc::sysconf(libc::_SC_CLK_TCK) };
        if ticks > 0 {
            ticks as u64
        } else {
            100
        }
    })
}

/// The first field of `/proc/loadavg` (the 1-minute load average).
fn first_loadavg_field(contents: &str) -> Option<f64> {
    contents.split_whitespace().next()?.parse().ok()
}

/// The `MemAvailable` value, in kB, from a `/proc/meminfo` body.
fn mem_available_kb(contents: &str) -> Option<u64> {
    contents
        .lines()
        .find_map(|line| line.strip_prefix("MemAvailable:"))?
        .split_whitespace()
        .next()?
        .parse()
        .ok()
}

/// `utime` + `stime` from a `/proc/self/stat` body, in clock ticks.
///
/// Fields 14 and 15, counted from 1. Field 2 (`comm`) is the executable's name
/// in parentheses and may itself contain spaces, so the split starts after the
/// **last** `)` — everything before it is the pid and that name.
fn process_ticks(stat: &str) -> Option<u64> {
    let rest = &stat[stat.rfind(')')? + 1..];
    let mut fields = rest.split_whitespace();
    // `rest` starts at field 3 (`state`), so the two we want are the 12th and
    // 13th of what follows.
    let utime: u64 = fields.nth(11)?.parse().ok()?;
    let stime: u64 = fields.next()?.parse().ok()?;
    Some(utime + stime)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;

    /// A machine with nothing loaded: enough to hold and query the object.
    fn installed(dispatch: Arc<DispatchStats>) -> (Arc<Printer>, Arc<SystemStats>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let object = SystemStats::install(&printer, dispatch).expect("the name is free");
        (printer, object)
    }

    #[test]
    fn test_get_status_reports_the_host_and_the_dispatcher() {
        // One round over a 50 ms threshold: the reported numbers are that
        // round's, in seconds.
        let dispatch = Arc::new(DispatchStats::new());
        dispatch.set_threshold(0.05);
        dispatch.record_round(0.060, 0.010);
        let (printer, object) = installed(Arc::clone(&dispatch));

        // The object is the machine's, queryable by name like any other.
        let registered = printer
            .lookup_object_as::<SystemStats>(SYSTEM_STATS_OBJECT)
            .expect("`system_stats` is registered");
        assert!(Arc::ptr_eq(&registered, &object));

        let status = object.get_status(0.0);

        // Upstream's three keys, with upstream's units: a float load average,
        // seconds of CPU time, kB available.
        assert!(
            status["sysload"].is_f64(),
            "sysload must be a float, was {}",
            status["sysload"]
        );
        assert!(
            status["cputime"].is_f64(),
            "cputime was {}",
            status["cputime"]
        );
        assert!(
            status["cputime"].as_f64().unwrap() >= 0.0,
            "cputime was {}",
            status["cputime"]
        );
        assert!(
            status["memavail"].is_u64(),
            "memavail must be whole kB, was {}",
            status["memavail"]
        );

        let dispatch = &status["dispatch"];
        assert_eq!(dispatch["rounds"], json!(1));
        assert_eq!(dispatch["over_threshold"], json!(1));
        assert_eq!(dispatch["threshold_s"], json!(0.05));
        assert_eq!(dispatch["busy_avg_s"], json!(0.060));
        assert_eq!(dispatch["busy_max_s"], json!(0.060));
        assert_eq!(dispatch["lateness_avg_s"], json!(0.010));
        assert_eq!(dispatch["lateness_max_s"], json!(0.010));
        for key in [
            "busy_avg_s",
            "busy_max_s",
            "lateness_avg_s",
            "lateness_max_s",
            "threshold_s",
        ] {
            assert!(
                dispatch[key].is_f64(),
                "{key} must be seconds as a float, was {}",
                dispatch[key]
            );
        }
        for key in ["rounds", "over_threshold"] {
            assert!(
                dispatch[key].is_u64(),
                "{key} must be a counter, was {}",
                dispatch[key]
            );
        }

        // Our own addition: the log writer's drop counter. Monotonic and a
        // whole number - a board writing to eMMC/SD is where it moves.
        assert!(
            status["dropped_logs"].is_u64(),
            "dropped_logs must be a counter, was {}",
            status["dropped_logs"]
        );
    }

    #[test]
    fn test_stats_is_the_sysload_cputime_memavail_line() {
        let (_printer, object) = installed(Arc::new(DispatchStats::new()));

        let (flag, line) = object.stats().expect("`system_stats` always reports");

        // Upstream's `(False, "sysload=%.2f cputime=%.3f" + " memavail=%d")`
        // (`statistics.py:32-46`): three `key=value` words in that order, and the
        // flag never set — the numbers join a line, they never make one.
        assert!(!flag, "the numbers never make a `Stats` line on their own");
        let fields: Vec<&str> = line.split(' ').collect();
        assert_eq!(fields.len(), 3, "line was {line}");
        let value = |index: usize, key: &str| {
            fields[index]
                .strip_prefix(&format!("{key}="))
                .unwrap_or_else(|| panic!("field {index} of '{line}' is not '{key}='"))
        };
        // `%.2f` and `%.3f`: the decimals upstream prints.
        for (index, key, decimals) in [(0, "sysload", 2), (1, "cputime", 3)] {
            let (whole, fraction) = value(index, key).split_once('.').expect("a float");
            assert_eq!(fraction.len(), decimals, "{key} was {}", value(index, key));
            assert!(whole.parse::<f64>().is_ok(), "{key} was {whole}");
        }
        // `%d`: whole kB.
        assert!(
            value(2, "memavail").parse::<u64>().is_ok(),
            "memavail was {}",
            value(2, "memavail")
        );
    }

    #[test]
    fn test_get_status_before_the_first_round_reports_zeros() {
        let (_printer, object) = installed(Arc::new(DispatchStats::new()));

        let status = object.get_status(0.0);
        let dispatch = &status["dispatch"];
        // No rounds yet: the averages are 0, not a division by zero, and no
        // threshold was installed, so nothing has counted as over it.
        assert_eq!(dispatch["rounds"], json!(0));
        assert_eq!(dispatch["over_threshold"], json!(0));
        assert_eq!(dispatch["busy_avg_s"], json!(0.0));
        assert_eq!(dispatch["lateness_avg_s"], json!(0.0));
        assert_eq!(dispatch["threshold_s"], json!(0.0));
    }

    #[test]
    fn test_install_registers_the_object_only_once() {
        let dispatch = Arc::new(DispatchStats::new());
        let (printer, first) = installed(dispatch);
        let second =
            SystemStats::install(&printer, Arc::new(DispatchStats::new())).expect("already there");

        assert!(Arc::ptr_eq(&first, &second));
        let named = printer
            .objects()
            .into_iter()
            .filter(|name| name == SYSTEM_STATS_OBJECT)
            .count();
        assert_eq!(named, 1);
    }

    // ---------------------------------------------------------------------
    // The /proc parsing, on fixed text (the real files are machine state)
    // ---------------------------------------------------------------------

    #[test]
    fn test_first_loadavg_field_is_the_one_minute_average() {
        assert_eq!(
            first_loadavg_field("0.52 0.48 0.44 1/1234 5678\n"),
            Some(0.52)
        );
        assert_eq!(first_loadavg_field(""), None);
        assert_eq!(first_loadavg_field("not a number"), None);
    }

    #[test]
    fn test_mem_available_comes_from_the_mem_available_line() {
        let meminfo = "MemTotal:        8000000 kB\nMemFree:         1000000 kB\n\
                       MemAvailable:    4000000 kB\nBuffers:          500000 kB\n";
        assert_eq!(mem_available_kb(meminfo), Some(4_000_000));
        assert_eq!(mem_available_kb("MemTotal: 8000000 kB\n"), None);
    }

    #[test]
    fn test_process_ticks_reads_utime_and_stime_past_the_command_name() {
        // Field 2 is `(some (odd) name)` here: the split must start after the
        // last `)`, or every field would be off.
        let stat = "1234 (some (odd) name) S 1 1234 1234 0 -1 4194560 100 0 0 0 \
                    12 34 0 0 20 0 1 0 100 12345678 1234 18446744073709551615";
        assert_eq!(process_ticks(stat), Some(12 + 34));
        assert_eq!(process_ticks("not a stat line"), None);
    }
}
