//! Logging setup, shared by the binaries.
//!
//! Two layers, one subscriber:
//!
//! * the usual formatted output, on stdout — except while a window is up, when
//!   it would paint over it (see [`WindowGuard`]);
//! * a copy of every record that a window can show in its log pane, in the order
//!   the host produced it.
//!
//! The copy exists because `klipperx klippy --tui` puts a client window over the
//! host's own terminal: without it the host's lines — "Successfully parsed
//! config", "API server listening on …" — would be written to a screen the window
//! owns, and be lost. With it, the window's log holds both sides interleaved,
//! which is the only way the two are readable together.
//!
//! This module is an application concern that lives in the library because the
//! library is already where this crate's application entry points live.

use std::collections::BTreeMap;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::Path;
use std::sync::{Arc, Mutex};

use tokio::sync::mpsc::UnboundedSender;
use tracing::debug;
use tracing::field::{Field, Visit};
use tracing_subscriber::filter::LevelFilter;
use tracing_subscriber::layer::{Context, Layer, SubscriberExt};
use tracing_subscriber::registry::LookupSpan;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::EnvFilter;

/// How loud a record is, for whatever is showing them.
///
/// Written here rather than reusing the window's own level type: this module is
/// the host's, and the host must not have to know that a window exists — let
/// alone link a terminal library for one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// Line-by-line detail: the bytes on the wire, and the like.
    Trace,
    /// Development detail, shown with `--verbose`.
    Debug,
    /// Something happened.
    Info,
    /// Something is off but the host carries on.
    Warn,
    /// Something failed.
    Error,
}

/// One record, as a window receives it.
pub type Record = (Level, String);

/// Where a window, if one is up, receives this process's log records.
///
/// A global rather than a parameter because the subscriber is installed once,
/// before the command line has been looked at, and the window is opened later —
/// and because a log record can come from any thread. `None` means no window:
/// records go to stdout only.
static WINDOW: Mutex<Option<UnboundedSender<Record>>> = Mutex::new(None);

/// The level `--verbose` turns on.
const VERBOSE_LEVEL: &str = "debug";

/// The level used when neither source asks for one.
const DEFAULT_LEVEL: &str = "info";

/// The filter the host runs with.
///
/// Two sources can ask for a level: `--verbose` and `RUST_LOG`. Neither is
/// ranked above the other — a flag must not silently discard an environment
/// setting, and the other way round — so the **more detailed** of the two wins:
/// `RUST_LOG=trace` survives `--verbose`, and `--verbose` survives
/// `RUST_LOG=warn`. With the flag absent, `RUST_LOG` is used as it is, including
/// to quiet the host down; with neither, the host is `info`.
///
/// A `RUST_LOG` that does not parse is ignored rather than fatal, the way
/// `EnvFilter::try_from_default_env`'s fallback treated it.
fn filter_for(verbose: bool, rust_log: Option<&str>) -> EnvFilter {
    let rust_log = rust_log.and_then(|spec| EnvFilter::try_new(spec).ok());
    match (verbose, rust_log) {
        (false, Some(env)) => env,
        (false, None) => EnvFilter::new(DEFAULT_LEVEL),
        (true, env) => {
            let verbose = EnvFilter::new(VERBOSE_LEVEL);
            match env {
                Some(env) if detail(&env) > detail(&verbose) => env,
                _ => verbose,
            }
        }
    }
}

/// How detailed a filter is: the loudest level it lets through.
///
/// A filter's own `max_level_hint` is the answer, so a per-target `RUST_LOG`
/// compares by its loudest target rather than by a hand-parsed level name.
fn detail(filter: &EnvFilter) -> LevelFilter {
    filter.max_level_hint().unwrap_or(LevelFilter::TRACE)
}

/// Install the global tracing subscriber.
///
/// The level is the more detailed of what `--verbose` asks for and what
/// `RUST_LOG` asks for; see [`filter_for`].
///
/// `log_file` is `--logfile`: the same formatted lines are also appended to that
/// file (upstream's `--logfile`, `klippy/klippy.py:294-345`). A file that cannot
/// be opened is reported on stderr and the host keeps logging to the terminal,
/// rather than refusing to start over a log path.
///
/// Installing twice is not an error: the second attempt is ignored, which is
/// what a user of the library as a library will do.
pub fn init(verbose: bool, log_file: Option<&Path>) {
    let filter = filter_for(verbose, std::env::var("RUST_LOG").ok().as_deref());
    let output = LogOutput::new(log_file);

    tracing_subscriber::registry()
        .with(filter)
        .with(
            tracing_subscriber::fmt::layer()
                .with_target(false)
                .with_writer(output),
        )
        .with(ToWindow)
        .try_init()
        .ok();

    debug!("Debug mode enabled");
}

/// Open a window over this process's own output.
///
/// While the guard lives, log records are also sent to `entries` instead of
/// being written to stdout — the window shows them, so writing them twice would
/// mean writing them onto the window.
pub fn to_window(entries: UnboundedSender<Record>) -> WindowGuard {
    *WINDOW.lock().expect("the window slot is not poisoned") = Some(entries);
    WindowGuard
}

/// Puts this process's output back on stdout when dropped.
pub struct WindowGuard;

impl Drop for WindowGuard {
    fn drop(&mut self) {
        *WINDOW.lock().expect("the window slot is not poisoned") = None;
    }
}

/// Whether a window is currently showing this process's log.
fn window() -> Option<UnboundedSender<Record>> {
    WINDOW
        .lock()
        .expect("the window slot is not poisoned")
        .clone()
}

/// Where the formatted output goes: stdout (unless a window is up) and, when
/// `--logfile` was given, that file too.
#[derive(Clone)]
struct LogOutput {
    /// The open log file, shared with every writer the layer makes.
    file: Option<Arc<Mutex<File>>>,
}

impl LogOutput {
    /// Open `path` for appending, or report on stderr and log to stdout only.
    fn new(path: Option<&Path>) -> Self {
        let file =
            path.and_then(
                |path| match OpenOptions::new().create(true).append(true).open(path) {
                    Ok(file) => Some(Arc::new(Mutex::new(file))),
                    Err(err) => {
                        eprintln!("warning: cannot open log file {}: {err}", path.display());
                        None
                    }
                },
            );
        Self { file }
    }
}

impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for LogOutput {
    type Writer = Tee;

    fn make_writer(&'a self) -> Self::Writer {
        Tee {
            file: self.file.clone(),
        }
    }
}

/// One formatted record's destination: stdout (or nowhere while a window is up)
/// plus the log file.
struct Tee {
    file: Option<Arc<Mutex<File>>>,
}

impl Write for Tee {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        // A write to either destination failing must not lose the other, and
        // must not make the tracing layer panic: a host that cannot write its
        // log file still runs.
        if window().is_none() {
            let _ = std::io::stdout().write_all(buf);
        }
        if let Some(file) = &self.file {
            let _ = file
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .write_all(buf);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let _ = std::io::stdout().flush();
        if let Some(file) = &self.file {
            let _ = file
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .flush();
        }
        Ok(())
    }
}

/// Copies every record a window is up for into that window.
struct ToWindow;

impl<S> Layer<S> for ToWindow
where
    S: tracing::Subscriber + for<'a> LookupSpan<'a>,
{
    fn on_event(&self, event: &tracing::Event<'_>, _context: Context<'_, S>) {
        // Nothing to do — and nothing to format — when no window is up, which is
        // the normal case.
        let Some(entries) = window() else {
            return;
        };

        let mut message = Message::default();
        event.record(&mut message);

        let level = match *event.metadata().level() {
            tracing::Level::TRACE => Level::Trace,
            tracing::Level::DEBUG => Level::Debug,
            tracing::Level::INFO => Level::Info,
            tracing::Level::WARN => Level::Warn,
            tracing::Level::ERROR => Level::Error,
        };

        // The window may have gone away between the check above and here; a
        // dropped record is not worth reporting, since the only place to report
        // it is the window that is gone.
        let _ = entries.send((level, message.0));
    }
}

/// The `message` field of a record, which is all a log line needs.
#[derive(Default)]
struct Message(String);

impl Visit for Message {
    /// A message that is a plain string — `info!("starting")` — is recorded as
    /// one, not through `Debug`. Missing this hook is how a log pane ends up
    /// full of empty lines.
    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.0 = value.to_string();
        }
    }

    /// A message with arguments — `info!("{} of {}", …)` — arrives as `Debug`.
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.0 = format!("{value:?}");
        }
    }
}

// ===========================================================================
// Rollover information
// ===========================================================================

/// The named blocks the log carries across a rollover.
///
/// Upstream's `bglogger.rollover_info` (`klippy/queuelogger.py:31-53`): a few
/// named texts — the versions, the config, the connected clients — written at
/// the top of a rotated log so a bug report has them without the whole file.
/// Sorted by name, as upstream sorts them.
static ROLLOVER: Mutex<BTreeMap<String, String>> = Mutex::new(BTreeMap::new());

/// Set one named rollover block, or (`info` is `None`) remove it.
pub fn set_rollover_info(name: &str, info: Option<&str>) {
    let mut rollover = ROLLOVER.lock().unwrap_or_else(|poison| poison.into_inner());
    match info {
        Some(info) => {
            rollover.insert(name.to_string(), info.to_string());
        }
        None => {
            rollover.remove(name);
        }
    }
}

/// Forget every rollover block.
///
/// Upstream clears them at the top of each restart (`klippy/klippy.py:356`), so
/// a block a previous printer set does not survive into the next one.
pub fn clear_rollover_info() {
    ROLLOVER
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .clear();
}

/// Write the rollover blocks and the banner as one log record.
///
/// Upstream writes the same text when the log rotates
/// (`klippy/queuelogger.py:49-53`). A host that rolls its log by restarting (the
/// only rotation there is today) calls this at each start; the banner is what
/// makes the seam visible in the file.
pub fn write_rollover() {
    let info = ROLLOVER.lock().unwrap_or_else(|poison| poison.into_inner());
    if info.is_empty() {
        return;
    }
    let mut block: String = info
        .values()
        .map(String::as_str)
        .collect::<Vec<_>>()
        .join("\n");
    block.push_str(&format!(
        "\n=============== Log rollover at {} ===============",
        asctime()
    ));
    tracing::info!("{block}");
}

/// The current local time as `time.asctime()` spells it.
fn asctime() -> String {
    // SAFETY: `localtime_r` is given a null-terminated-ish pointer to a `time_t`
    // and a `tm` it may write; `strftime` writes into `buf` and reports how many
    // bytes it used, which is bounded by the buffer length. The format string is
    // a static C string.
    unsafe {
        let now = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        if libc::localtime_r(&now, &mut tm).is_null() {
            return "?".to_string();
        }
        let mut buf = [0 as libc::c_char; 64];
        let format = b"%a %b %e %H:%M:%S %Y\0";
        let written = libc::strftime(
            buf.as_mut_ptr(),
            buf.len(),
            format.as_ptr() as *const libc::c_char,
            &tm,
        );
        let bytes: Vec<u8> = buf[..written].iter().map(|byte| *byte as u8).collect();
        String::from_utf8_lossy(&bytes).into_owned()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::mpsc::unbounded_channel;

    /// The window slot is process-wide, so these two tests would otherwise race
    /// each other — one dropping the guard while the other is emitting.
    static SLOT: Mutex<()> = Mutex::new(());

    /// Hold the slot for the length of a test body.
    fn exclusive() -> std::sync::MutexGuard<'static, ()> {
        SLOT.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// The point of this module: a window sees the host's own records, with
    /// their levels, in the order they were made.
    ///
    /// A thread-local subscriber is installed rather than the global one, so the
    /// test neither needs nor disturbs the process-wide setup.
    #[test]
    fn test_a_window_receives_the_hosts_records() {
        let _slot = exclusive();
        let (entries, mut logs) = unbounded_channel();
        let guard = to_window(entries);

        tracing::subscriber::with_default(tracing_subscriber::registry().with(ToWindow), || {
            // A literal message and a formatted one are recorded differently,
            // and both have to arrive: a pane full of empty lines is what
            // happens when only one of the two is handled.
            tracing::info!("Successfully parsed config with 1 sections");
            tracing::warn!("dropping malformed request");
            tracing::debug!("a detail");
            tracing::trace!("tx frame: 0a11 | 01020304 05 | 31d87e");
            tracing::info!("API server listening on {}", "unix:/tmp/x");
        });

        let received: Vec<Record> = std::iter::from_fn(|| logs.try_recv().ok()).collect();

        assert_eq!(
            received,
            vec![
                (
                    Level::Info,
                    "Successfully parsed config with 1 sections".into()
                ),
                (Level::Warn, "dropping malformed request".into()),
                (Level::Debug, "a detail".into()),
                (Level::Trace, "tx frame: 0a11 | 01020304 05 | 31d87e".into()),
                (Level::Info, "API server listening on unix:/tmp/x".into()),
            ]
        );

        // Once the window is gone, the host's records stop being copied: they go
        // back to being printed, and there is nobody left to copy them for.
        drop(guard);
        tracing::subscriber::with_default(tracing_subscriber::registry().with(ToWindow), || {
            tracing::info!("after the window")
        });
        assert!(
            logs.try_recv().is_err(),
            "a closed window still received a record"
        );
    }

    /// `--verbose` and `RUST_LOG` are two ways to ask for a level. Neither is
    /// ranked above the other: the more detailed of the two is used.
    #[test]
    fn test_the_more_detailed_of_flag_and_environment_wins() {
        // `--verbose` is a floor, not a ceiling: a louder RUST_LOG is kept...
        assert_eq!(detail(&filter_for(true, Some("trace"))), LevelFilter::TRACE);
        // ...and a quieter one does not silence the flag.
        assert_eq!(detail(&filter_for(true, Some("warn"))), LevelFilter::DEBUG);
        assert_eq!(detail(&filter_for(true, Some("error"))), LevelFilter::DEBUG);
        assert_eq!(detail(&filter_for(true, None)), LevelFilter::DEBUG);
        // Without the flag, RUST_LOG is used as it is — including to quiet down.
        assert_eq!(
            detail(&filter_for(false, Some("trace"))),
            LevelFilter::TRACE
        );
        assert_eq!(detail(&filter_for(false, Some("warn"))), LevelFilter::WARN);
        assert_eq!(detail(&filter_for(false, None)), LevelFilter::INFO);
        // A RUST_LOG that does not parse is ignored, not fatal.
        assert_eq!(
            detail(&filter_for(true, Some("foo=notalevel"))),
            LevelFilter::DEBUG
        );
        assert_eq!(
            detail(&filter_for(false, Some("foo=notalevel"))),
            LevelFilter::INFO
        );
    }

    #[test]
    fn test_the_window_slot_is_where_the_layer_looks() {
        let _slot = exclusive();
        // `window()` is what both the layer and the stdout gate consult, so it
        // has to follow the guard.
        assert!(window().is_none());
        let (entries, _logs) = unbounded_channel();
        let guard = to_window(entries);
        assert!(window().is_some());
        drop(guard);
        assert!(window().is_none());
    }

    #[test]
    fn test_a_log_file_receives_the_formatted_bytes() {
        use tracing_subscriber::fmt::MakeWriter;

        let _slot = exclusive();
        let dir = std::env::temp_dir().join(format!("klipperx-log-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("test.log");
        let _ = std::fs::remove_file(&path);

        // A window is up so the tee skips stdout: this test asserts the file
        // side without echoing a line into the test output.
        let (entries, _logs) = unbounded_channel();
        let _guard = to_window(entries);
        let output = LogOutput::new(Some(&path));
        {
            let mut writer = output.make_writer();
            writer.write_all(b"hello\n").unwrap();
            writer.flush().unwrap();
        }

        assert_eq!(std::fs::read_to_string(&path).unwrap(), "hello\n");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_a_log_file_that_cannot_be_opened_degrades_to_stdout() {
        // A bad `--logfile` must not stop the host: the writer exists and just
        // has no file behind it.
        let output = LogOutput::new(Some(Path::new("/nonexistent/dir/klippy.log")));
        assert!(output.file.is_none());
    }

    #[test]
    fn test_rollover_info_is_sorted_and_cleared() {
        let _slot = exclusive();
        clear_rollover_info();
        set_rollover_info("versions", Some("the versions"));
        set_rollover_info("config", Some("the config"));

        let (entries, mut logs) = unbounded_channel();
        let guard = to_window(entries);
        tracing::subscriber::with_default(tracing_subscriber::registry().with(ToWindow), || {
            write_rollover();
            // Nothing to write once cleared: no second record.
            clear_rollover_info();
            write_rollover();
        });
        drop(guard);

        let received: Vec<Record> = std::iter::from_fn(|| logs.try_recv().ok()).collect();
        assert_eq!(received.len(), 1, "one rollover record: {received:?}");
        let (level, text) = &received[0];
        assert_eq!(*level, Level::Info);
        // Sorted by name: `config` before `versions`.
        assert!(text.starts_with("the config\nthe versions\n"), "{text}");
        assert!(text.contains("Log rollover at"), "{text}");
    }
}
