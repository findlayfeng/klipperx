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
//! Neither destination is written from the thread that logged: the formatted
//! bytes go into a bounded queue that one writer thread drains ([`LogSink`]). The
//! threads that log are the reactor loop and the MCU's receive task, and on a
//! card-sized board a write to eMMC/SD can take milliseconds — waiting for the
//! disk there is waiting in the middle of a move. A full queue drops the record
//! and counts it ([`dropped_records`]) rather than blocking the caller.
//!
//! This module is an application concern that lives in the library because the
//! library is already where this crate's application entry points live.

use std::collections::BTreeMap;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, Sender, SyncSender, TrySendError};
use std::sync::{Arc, Mutex, Once};
use std::time::{Duration, Instant};

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

/// How long a log file covers before it is rotated: a day.
///
/// Upstream's `bglogger` is a `TimedRotatingFileHandler` with
/// `when='midnight'`, `backupCount=5` (`klippy/queuelogger.py:24-31`): the file
/// in use is the one for the day it is written on, and the first record after
/// local midnight starts the next day's file.
const ROTATION_PERIOD: i64 = 24 * 60 * 60;

/// How many rotated files are kept (`backupCount`), the oldest going first.
const BACKUP_COUNT: usize = 5;

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
    static INSTALLED: Once = Once::new();
    INSTALLED.call_once(|| {
        let filter = filter_for(verbose, std::env::var("RUST_LOG").ok().as_deref());
        let output = LogOutput::new(log_file);

        let installed = tracing_subscriber::registry()
            .with(filter)
            .with(
                tracing_subscriber::fmt::layer()
                    .with_target(false)
                    .with_writer(output.clone()),
            )
            .with(ToWindow)
            .try_init()
            .is_ok();

        // Only a subscriber that was actually installed has records to write out;
        // otherwise the sink is dropped, and the writer thread with it, having
        // opened the log file for nothing.
        if installed {
            register_exit_drain(&output.sink);
        }

        debug!("Debug mode enabled");
    });
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

/// What the log file's clock reads: seconds since the epoch.
///
/// A seam rather than a direct `time()` call so that a test can cross a day
/// boundary without waiting for one — the same reason the reactor has a manual
/// clock. There is no production way to replace it.
type Clock = Arc<dyn Fn() -> i64 + Send + Sync>;

/// The wall clock, in seconds since the epoch.
fn wall_clock() -> i64 {
    // SAFETY: `time(NULL)` only reads the clock.
    unsafe { libc::time(std::ptr::null_mut()) as i64 }
}

/// How many formatted records may wait for the writer thread.
///
/// Bounded, because the caller is a caller in the middle of something else — the
/// queue is a handoff, not a buffer of last resort. 1024 records is a few hundred
/// kilobytes of ordinary log lines: enough to ride out a slow write burst on a
/// card, small enough that the queue itself never becomes the memory problem it
/// exists to prevent. What does not fit is dropped, not waited for.
const QUEUE_CAPACITY: usize = 1024;

/// How long the exit drain waits for the writer thread before giving up.
///
/// A process that is exiting has to exit: half a second is long enough to write
/// the queued records out on a card, and short enough that a stuck destination (a
/// terminal that stopped reading, a disk that went away) cannot hold the host
/// hostage. Whatever is still queued after that is lost — the alternative is a
/// host that will not stop.
const DRAIN_TIMEOUT: Duration = Duration::from_millis(500);

/// How many records were dropped because the writer could not keep up.
static DROPPED: AtomicU64 = AtomicU64::new(0);

/// How many records this process has dropped because the writer could not keep
/// up.
///
/// A record is dropped when the queue is full: the thread that logged carries on
/// instead of waiting for the disk. Never reset — it counts for the life of the
/// process, and it is the one number a log can offer about its own completeness.
pub fn dropped_records() -> u64 {
    DROPPED.load(Ordering::Relaxed)
}

/// What the thread that logs hands to the writer thread.
enum Command {
    /// One formatted record, already bytes.
    ///
    /// `to_stdout` is decided by the sender rather than by the writer: whether a
    /// window is up is a property of the moment the record was made, and the
    /// window may be gone by the time the writer gets to the record.
    Record { bytes: Vec<u8>, to_stdout: bool },
    /// Flush both destinations.
    Flush,
    /// Write what is queued, then stop.
    Shutdown,
    /// Answer on the channel once everything queued so far has been written.
    ///
    /// Tests need a moment at which the writer has caught up; nothing on the
    /// host's own path may wait for it.
    #[cfg(test)]
    Sync(Sender<()>),
}

/// Where the formatted output goes: stdout (unless a window is up) and, when
/// `--logfile` was given, that file too — through a queue one thread drains.
#[derive(Clone)]
struct LogOutput {
    sink: Arc<LogSink>,
}

impl LogOutput {
    /// Open `path` for appending, or report on stderr and log to stdout only.
    fn new(path: Option<&Path>) -> Self {
        Self::with_clock(path, Arc::new(wall_clock))
    }

    /// The same, reading "now" from `clock` instead of the wall clock.
    fn with_clock(path: Option<&Path>, clock: Clock) -> Self {
        let file = path.and_then(|path| match LogFile::open(path, clock) {
            Ok(file) => Some(file),
            Err(err) => {
                eprintln!("warning: cannot open log file {}: {err}", path.display());
                None
            }
        });
        Self {
            sink: LogSink::spawn(file),
        }
    }
}

/// The queue a log record is handed to, and the thread that drains it.
struct LogSink {
    /// The queue. Bounded ([`QUEUE_CAPACITY`]) and only ever filled without
    /// waiting.
    queue: SyncSender<Command>,
    /// Answered when the writer thread has drained and stopped; taken by
    /// [`LogSink::drain`], which is why it is an `Option`.
    finished: Mutex<Option<Receiver<()>>>,
}

impl LogSink {
    /// Start the thread that owns the log file and stdout, and the queue that
    /// feeds it.
    fn spawn(file: Option<LogFile>) -> Arc<Self> {
        let (queue, commands) = std::sync::mpsc::sync_channel(QUEUE_CAPACITY);
        let (finished, done) = std::sync::mpsc::channel();
        std::thread::Builder::new()
            .name("log-writer".to_string())
            .spawn(move || writer(commands, file, finished))
            .expect("the log writer thread starts");
        Arc::new(Self {
            queue,
            finished: Mutex::new(Some(done)),
        })
    }

    /// Hand one formatted record over, or count it as dropped.
    ///
    /// Never waits: a full queue — or a writer thread that is gone — is a lost
    /// record and an incremented counter, not a delay on the thread that logged.
    /// Neither is an error to report, because the only thing that could report it
    /// is the log; a host that cannot write its log still runs.
    fn submit(&self, bytes: &[u8], to_stdout: bool) {
        let record = Command::Record {
            bytes: bytes.to_vec(),
            to_stdout,
        };
        if let Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) =
            self.queue.try_send(record)
        {
            DROPPED.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Ask for a flush, without waiting for it to happen.
    ///
    /// A dropped request is harmless because a flush is idempotent: the log
    /// file's writes are unbuffered, and anything still owed to stdout is flushed
    /// by the exit drain — a host that asked to flush once has nothing more to say
    /// than a host that asked twice.
    fn flush(&self) {
        let _ = self.queue.try_send(Command::Flush);
    }

    /// Write what is queued, stop the writer thread, and wait at most `within`.
    ///
    /// This is the exit drain: the last records of a run are the interesting ones,
    /// and they are worth a short wait at the end of the process — and nothing
    /// more than a short one, because a destination that has stopped accepting
    /// bytes must not stop the host from exiting.
    ///
    /// Returns whether the writer thread finished; if it did not, whatever it still
    /// held is lost with the process.
    fn drain(&self, within: Duration) -> bool {
        let finished = self
            .finished
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .take();
        // Already drained, or (in a test) a sink that never had a writer thread.
        let Some(finished) = finished else {
            return false;
        };
        let deadline = Instant::now() + within;
        // Handing the shutdown over can need a wait of its own: the queue may be
        // full of records the thread has not written yet, and it can only make
        // room by writing them. The whole drain is still bounded by `within`.
        loop {
            match self.queue.try_send(Command::Shutdown) {
                Ok(()) => break,
                Err(TrySendError::Disconnected(_)) => return false,
                Err(TrySendError::Full(_)) => {
                    let left = deadline.saturating_duration_since(Instant::now());
                    if left.is_zero() {
                        return false;
                    }
                    std::thread::sleep(left.min(Duration::from_millis(1)));
                }
            }
        }
        finished
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
            .is_ok()
    }

    /// Wait for the writer thread to have written everything queued so far.
    ///
    /// A test helper, not a host path: the host never waits for its own log.
    #[cfg(test)]
    fn sync(&self) {
        let (done, written) = std::sync::mpsc::channel();
        if self.queue.send(Command::Sync(done)).is_ok() {
            let _ = written.recv_timeout(Duration::from_secs(10));
        }
    }
}

/// The writer thread: the only thing that touches stdout and the log file.
///
/// A thread of its own because the threads that log are the ones that must not
/// wait — the reactor loop and the MCU's receive task are not the place to spend
/// a card's write latency. It blocks on the queue when there is nothing to write,
/// which costs nothing.
fn writer(commands: Receiver<Command>, mut file: Option<LogFile>, finished: Sender<()>) {
    while let Ok(command) = commands.recv() {
        if let Command::Shutdown = command {
            // A shutdown is not a discarding stop: what is already queued is
            // written first, which is what the caller waits for.
            while let Ok(command) = commands.try_recv() {
                write_command(command, &mut file);
            }
            flush_destinations(&mut file);
            break;
        }
        write_command(command, &mut file);
    }
    let _ = finished.send(());
}

/// Do one queued command's work, on the writer thread.
fn write_command(command: Command, file: &mut Option<LogFile>) {
    match command {
        Command::Record { bytes, to_stdout } => {
            // A write to either destination failing must not lose the other, and
            // must not take the host down: a host that cannot write its log file
            // still runs.
            if to_stdout {
                let _ = std::io::stdout().write_all(&bytes);
            }
            if let Some(file) = file.as_mut() {
                let _ = file.write_all(&bytes);
            }
        }
        Command::Flush => flush_destinations(file),
        // Handled by the loop itself; a record queued after a shutdown is one the
        // process no longer owes.
        Command::Shutdown => {}
        #[cfg(test)]
        Command::Sync(done) => {
            let _ = done.send(());
        }
    }
}

/// Flush stdout and the log file, ignoring failures the way a record's writes do.
fn flush_destinations(file: &mut Option<LogFile>) {
    let _ = std::io::stdout().flush();
    if let Some(file) = file.as_mut() {
        let _ = file.flush();
    }
}

/// The sinks the process has to write out before it exits.
///
/// Only [`init`] registers here, so a test's own sinks are neither held alive by
/// this list nor written at the end of the test binary.
static SINKS: Mutex<Vec<Arc<LogSink>>> = Mutex::new(Vec::new());

/// Whether the exit handler below has been installed.
static EXIT_DRAIN: Once = Once::new();

/// Have the process write out `sink`'s queue when it exits.
///
/// The records at the end of a run are the ones that matter — the shutdown, the
/// error that ended it — and a log file or a pipe to `tail` can still hold them
/// when `main` returns. `atexit` is the hook this build has: the binaries call
/// [`init`] and exit as they please, and nothing in the host would call a drain of
/// its own.
fn register_exit_drain(sink: &Arc<LogSink>) {
    SINKS
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .push(Arc::clone(sink));
    EXIT_DRAIN.call_once(|| {
        // SAFETY: `atexit` only stores the function pointer. The handler reads
        // state this module owns — the sink list and the writers' queues — none of
        // which the C library tears down before it runs.
        unsafe { libc::atexit(drain_at_exit) };
    });
}

/// Write out every registered sink, waiting a bounded moment for each.
extern "C" fn drain_at_exit() {
    let sinks: Vec<Arc<LogSink>> = SINKS
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .drain(..)
        .collect();
    for sink in sinks {
        sink.drain(DRAIN_TIMEOUT);
    }
}

/// The log file in use, rotated at each local midnight.
///
/// One file covers one day: `klippy.log` holds today's records, and the day
/// that ends is renamed to `klippy.log.<date>` — the suffix stdlib derives from
/// `when='midnight'` (`%Y-%m-%d`). Keeping the rollover information in the new
/// file is upstream's whole point in rotating
/// (`QueueListener.doRollover`, `klippy/queuelogger.py:49-61`): a log attached
/// to a bug report names the build and the config even after it was rotated.
struct LogFile {
    /// The file in use, `klippy.log`.
    path: PathBuf,
    file: File,
    /// Seconds since the epoch at which the next rollover is due: the next
    /// local midnight.
    rollover_at: i64,
    /// Where "now" comes from.
    clock: Clock,
}

impl LogFile {
    /// Open `path` for appending, due to rotate at the next local midnight.
    fn open(path: &Path, clock: Clock) -> std::io::Result<Self> {
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(Self {
            path: path.to_path_buf(),
            file,
            rollover_at: next_midnight((clock)()),
            clock,
        })
    }

    /// Append one record, rotating first if the day it belongs to has ended.
    ///
    /// Upstream checks per record (`TimedRotatingFileHandler.emit` calls
    /// `shouldRollover` then `doRollover`), which is why a host left running
    /// overnight rotates on its next log line rather than at midnight itself.
    fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
        let now = (self.clock)();
        if now >= self.rollover_at {
            self.rollover(now)?;
        }
        self.file.write_all(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.file.flush()
    }

    /// Rotate the file in use and start the next one, `now` being the moment the
    /// day boundary was noticed.
    fn rollover(&mut self, now: i64) -> std::io::Result<()> {
        // A rotated file is named for the day it holds, not for the moment of
        // the rollover: stdlib's `doRollover` derives the suffix from
        // `rolloverAt - interval`.
        let rotated = rotated_name(&self.path, self.rollover_at - ROTATION_PERIOD);
        // The writes are unbuffered, so the handle being open across the rename
        // costs nothing; upstream's close-then-rename reaches the same files.
        self.file.flush()?;
        if rotated.exists() {
            std::fs::remove_file(&rotated)?;
        }
        std::fs::rename(&self.path, &rotated)?;
        prune_old_backups(&self.path);

        self.file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)?;
        self.rollover_at = next_midnight(now);

        // Upstream's `doRollover` emits the rollover information once the
        // rotation is done, so it lands in the new file rather than in the one
        // that was just closed.
        let header = rollover_text();
        if !header.is_empty() {
            self.file.write_all(header.as_bytes())?;
            self.file.write_all(b"\n")?;
        }
        self.file.flush()
    }
}

/// The name a rotated file takes: the base name plus the day it holds.
fn rotated_name(path: &Path, at: i64) -> PathBuf {
    let mut name = path.as_os_str().to_owned();
    name.push(format!(".{}", local_time(b"%Y-%m-%d\0", at)));
    PathBuf::from(name)
}

/// Whether `suffix` is the `%Y-%m-%d` a midnight rollover appends.
fn is_rotation_suffix(suffix: &str) -> bool {
    let bytes = suffix.as_bytes();
    bytes.len() == 10
        && bytes.iter().enumerate().all(|(index, byte)| match index {
            4 | 7 => *byte == b'-',
            _ => byte.is_ascii_digit(),
        })
}

/// Delete rotated files until only the newest [`BACKUP_COUNT`] are left.
///
/// `TimedRotatingFileHandler.getFilesToDelete`: the dated files of this base
/// name, sorted, everything past the newest `backupCount` removed.
fn prune_old_backups(path: &Path) {
    let (Some(dir), Some(base)) = (path.parent(), path.file_name()) else {
        return;
    };
    let dir = if dir.as_os_str().is_empty() {
        Path::new(".")
    } else {
        dir
    };
    let prefix = format!("{}.", base.to_string_lossy());
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    let mut dated: Vec<PathBuf> = entries
        .filter_map(|entry| entry.ok())
        .filter(|entry| {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            name.strip_prefix(&prefix).is_some_and(is_rotation_suffix)
        })
        .map(|entry| entry.path())
        .collect();
    if dated.len() < BACKUP_COUNT {
        return;
    }
    // A `%Y-%m-%d` suffix sorts chronologically.
    dated.sort();
    for old in &dated[..dated.len() - BACKUP_COUNT] {
        let _ = std::fs::remove_file(old);
    }
}

/// The local midnight after `now`, in seconds since the epoch.
///
/// Upstream's `_computeRollover` for `when='midnight'`: the boundary of the next
/// local day, not `now` plus a fixed interval — a day with a DST change is not
/// 24 hours long ([`ROTATION_PERIOD`] is the period as upstream measures it when
/// naming the rotated file, which is also why that arithmetic is copied as is).
fn next_midnight(now: i64) -> i64 {
    // SAFETY: `localtime_r` writes into a `tm` this frame owns; `mktime`
    // normalizes that struct and reports the instant, or -1.
    unsafe {
        let at: libc::time_t = now as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        if libc::localtime_r(&at, &mut tm).is_null() {
            return now + ROTATION_PERIOD;
        }
        tm.tm_hour = 0;
        tm.tm_min = 0;
        tm.tm_sec = 0;
        tm.tm_mday += 1;
        // Let `mktime` work out whether that day is in daylight time.
        tm.tm_isdst = -1;
        let midnight = libc::mktime(&mut tm);
        if midnight == -1 {
            now + ROTATION_PERIOD
        } else {
            midnight as i64
        }
    }
}

impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for LogOutput {
    type Writer = Tee;

    fn make_writer(&'a self) -> Self::Writer {
        Tee {
            sink: Arc::clone(&self.sink),
        }
    }
}

/// One formatted record's destination: the writer thread's queue.
///
/// The layer hands a whole record over in one `write_all`; nothing here waits for
/// it to be written — see [`LogSink::submit`] for what a full queue does.
struct Tee {
    sink: Arc<LogSink>,
}

impl Write for Tee {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        // The window gate stays on this side of the queue: whether these bytes go
        // to stdout is a property of the moment the record was made, not of the
        // moment the writer gets to it.
        self.sink.submit(buf, window().is_none());
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.sink.flush();
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

/// The rollover block: every named text, sorted by name, then the banner that
/// dates the seam — upstream's `QueueListener.doRollover`
/// (`klippy/queuelogger.py:50-57`). Empty when there is nothing to report.
///
/// The lock is released before the text is used, so a writer that holds the log
/// file while rotating a record cannot deadlock against a caller of this.
fn rollover_text() -> String {
    let info = ROLLOVER.lock().unwrap_or_else(|poison| poison.into_inner());
    if info.is_empty() {
        return String::new();
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
    block
}

/// Write the rollover blocks and the banner as one log record.
///
/// Upstream writes the same text when the log rotates
/// (`klippy/queuelogger.py:49-53`), and once at each start for the versions
/// block (`klippy/klippy.py:339-345`); a rotation writes it into the new file
/// itself (see [`LogFile::rollover`]). The banner is what makes the seam visible
/// in the file.
pub fn write_rollover() {
    let block = rollover_text();
    if block.is_empty() {
        return;
    }
    tracing::info!("{block}");
}

/// `at` (seconds since the epoch) in local time, in `format` — one of the C
/// formats the log uses, which must be null-terminated.
fn local_time(format: &[u8], at: i64) -> String {
    // SAFETY: `localtime_r` is given a pointer to a `time_t` and a `tm` it may
    // write; `strftime` writes into `buf` and reports how many bytes it used,
    // which is bounded by the buffer length. The format string is a static C
    // string, null-terminated by its callers.
    unsafe {
        let at: libc::time_t = at as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        if libc::localtime_r(&at, &mut tm).is_null() {
            return "?".to_string();
        }
        let mut buf = [0 as libc::c_char; 64];
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

/// The current local time as `time.asctime()` spells it.
fn asctime() -> String {
    local_time(b"%a %b %e %H:%M:%S %Y\0", wall_clock())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicI64, Ordering};
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
        // `write_all` only queues the bytes; wait for the writer thread to have
        // put them in the file before reading it back.
        output.sink.sync();

        assert_eq!(std::fs::read_to_string(&path).unwrap(), "hello\n");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_a_log_file_that_cannot_be_opened_degrades_to_stdout() {
        // A bad `--logfile` must not stop the host: the sink still takes a
        // record, and nothing is created at the path that could not be opened.
        let dir = std::env::temp_dir().join(format!("klipperx-log-missing-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("klippy.log");

        let output = LogOutput::new(Some(&path));
        // Not to stdout: what this test watches is what the sink does with it.
        output.sink.submit(b"no file behind it\n", false);
        output.sink.sync();

        assert!(!path.exists(), "a log file appeared at {}", path.display());
    }

    // =======================================================================
    // The queue
    // =======================================================================

    /// A sink whose writer thread never ran.
    ///
    /// The test holds both receiving ends and never answers them, so the queue
    /// fills up and the exit drain is never told the writer finished. Holding them
    /// (rather than dropping them) is what keeps the channels connected.
    struct StalledSink {
        sink: Arc<LogSink>,
        _commands: Receiver<Command>,
        _finished: Sender<()>,
    }

    impl StalledSink {
        fn new() -> Self {
            let (queue, commands) = std::sync::mpsc::sync_channel(QUEUE_CAPACITY);
            let (finished, done) = std::sync::mpsc::channel();
            Self {
                sink: Arc::new(LogSink {
                    queue,
                    finished: Mutex::new(Some(done)),
                }),
                _commands: commands,
                _finished: finished,
            }
        }
    }

    /// A full queue is a dropped record, not a wait: the thread that logged is in
    /// the middle of something else, so a writer that cannot keep up costs it the
    /// record and nothing else.
    #[test]
    fn test_a_full_queue_drops_a_record_instead_of_waiting() {
        let stalled = StalledSink::new();
        let before = dropped_records();

        let started = Instant::now();
        // One record more than the queue holds: the last one has nowhere to go.
        for _ in 0..=QUEUE_CAPACITY {
            stalled
                .sink
                .submit(b"a record the writer never takes\n", false);
        }
        let elapsed = started.elapsed();

        assert!(
            elapsed < Duration::from_millis(100),
            "submitting {QUEUE_CAPACITY} records took {elapsed:?}: the caller waited for the writer"
        );
        assert!(
            dropped_records() - before > 0,
            "a full queue dropped nothing"
        );
    }

    /// The host's own path: a record handed over reaches the file, and the exit
    /// drain does not lose the records still queued behind it.
    #[test]
    fn test_queued_records_are_written_and_the_exit_drain_keeps_them() {
        let dir = TempDir::new("drain");
        let path = dir.path("klippy.log");
        let output = LogOutput::new(Some(&path));

        // Queued, not written: nothing on this side waits for the disk.
        for line in ["first\n", "second\n", "third\n"] {
            output.sink.submit(line.as_bytes(), false);
        }

        assert!(
            output.sink.drain(DRAIN_TIMEOUT),
            "the writer did not finish within {DRAIN_TIMEOUT:?}"
        );
        assert_eq!(
            std::fs::read_to_string(&path).expect("the log file is readable"),
            "first\nsecond\nthird\n"
        );
    }

    /// The exit drain is bounded: a writer that never finishes costs the process
    /// one short wait rather than a hang, and the drain says it did not finish.
    #[test]
    fn test_a_drain_with_no_writer_gives_up_within_its_bound() {
        let stalled = StalledSink::new();
        stalled.sink.submit(b"never written\n", false);

        let bound = Duration::from_millis(50);
        let started = Instant::now();
        let drained = stalled.sink.drain(bound);
        let elapsed = started.elapsed();

        assert!(!drained, "a drain nobody served reported success");
        assert!(
            elapsed >= bound,
            "the drain gave up after {elapsed:?}, before its bound of {bound:?}"
        );
        assert!(
            elapsed < bound + Duration::from_secs(2),
            "the drain took {elapsed:?}, which is not bounded by {bound:?}"
        );
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

    // =======================================================================
    // Rotation
    // =======================================================================

    /// A directory of this test's own, removed when the test ends.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(tag: &str) -> Self {
            let dir = std::env::temp_dir().join(format!(
                "klipperx-log-rotation-{tag}-{}",
                std::process::id()
            ));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).expect("the test directory is created");
            Self(dir)
        }

        fn path(&self, name: &str) -> PathBuf {
            self.0.join(name)
        }

        /// The directory's entries, sorted, as bare names.
        fn names(&self) -> Vec<String> {
            let mut names: Vec<String> = std::fs::read_dir(&self.0)
                .expect("the test directory is readable")
                .map(|entry| {
                    entry
                        .expect("an entry")
                        .file_name()
                        .to_string_lossy()
                        .into_owned()
                })
                .collect();
            names.sort();
            names
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A log file whose clock the test moves, and the handle it is moved with.
    ///
    /// The clock is why these tests need no midnight of their own: a rollover is
    /// a comparison against "now", so moving "now" is the whole trigger.
    fn test_log(dir: &TempDir, start: i64) -> (LogOutput, Arc<AtomicI64>) {
        let now = Arc::new(AtomicI64::new(start));
        let clock: Clock = {
            let now = Arc::clone(&now);
            Arc::new(move || now.load(Ordering::SeqCst))
        };
        (
            LogOutput::with_clock(Some(&dir.path("klippy.log")), clock),
            now,
        )
    }

    /// A local wall-clock time in seconds since the epoch, so a test names a day
    /// without having to know its time zone.
    fn local(year: i32, month: i32, day: i32, hour: i32, minute: i32, second: i32) -> i64 {
        // SAFETY: `mktime` normalizes a `tm` this frame owns and reports the
        // instant it names.
        unsafe {
            let mut tm: libc::tm = std::mem::zeroed();
            tm.tm_year = year - 1900;
            tm.tm_mon = month - 1;
            tm.tm_mday = day;
            tm.tm_hour = hour;
            tm.tm_min = minute;
            tm.tm_sec = second;
            tm.tm_isdst = -1;
            libc::mktime(&mut tm) as i64
        }
    }

    /// The date a rotated file covering the day that starts at `at` is named
    /// after, spelled the way stdlib's midnight suffix spells it.
    fn day_name(at: i64) -> String {
        format!("klippy.log.{}", local_time(b"%Y-%m-%d\0", at))
    }

    /// Append one record through the production writer.
    ///
    /// The callers keep a window up, which is what stops the tee from also
    /// echoing these records into the test output.
    fn append(output: &LogOutput, text: &str) {
        use tracing_subscriber::fmt::MakeWriter;

        let mut writer = output.make_writer();
        writer
            .write_all(text.as_bytes())
            .expect("the record writes");
        writer.flush().expect("the record flushes");
        // The bytes are queued, not written: wait for the writer thread so that
        // the caller's next look at the file sees this record.
        output.sink.sync();
    }

    /// The day boundary is what rotates the log: the first record after local
    /// midnight goes to a new file, and the file that was left behind holds the
    /// day it was written on.
    #[test]
    fn test_the_first_record_after_midnight_rotates_the_log() {
        let _slot = exclusive();
        let (entries, _logs) = unbounded_channel();
        let _window = to_window(entries);

        let dir = TempDir::new("day");
        let (output, now) = test_log(&dir, local(2026, 6, 10, 23, 59, 59));
        append(&output, "before midnight\n");
        assert_eq!(
            dir.names(),
            vec!["klippy.log".to_string()],
            "no rotation yet"
        );

        now.store(local(2026, 6, 11, 0, 0, 1), Ordering::SeqCst);
        append(&output, "after midnight\n");

        let names = dir.names();
        assert_eq!(names.len(), 2, "the day that ended was rotated: {names:?}");
        let rotated = names
            .iter()
            .find(|name| *name != "klippy.log")
            .expect("the rotated file");
        assert_eq!(*rotated, day_name(local(2026, 6, 10, 0, 0, 0)), "{names:?}");
        // What was in the file in use is what the rotated file holds.
        assert_eq!(
            std::fs::read_to_string(dir.path(rotated)).unwrap(),
            "before midnight\n"
        );
        let current = std::fs::read_to_string(dir.path("klippy.log")).unwrap();
        assert!(current.contains("after midnight"), "{current}");
        assert!(!current.contains("before midnight"), "{current}");
    }

    /// A rotated log opens with the rollover information, so a file taken from
    /// the middle of a run still says what produced it — upstream's reason for
    /// writing the block on every rollover (`klippy/queuelogger.py:50-57`).
    #[test]
    fn test_a_rotated_log_starts_with_the_rollover_block() {
        let _slot = exclusive();
        let (entries, _logs) = unbounded_channel();
        let _window = to_window(entries);

        clear_rollover_info();
        set_rollover_info("versions", Some("Versions: klipperx test"));
        set_rollover_info(
            "config",
            Some("===== Config file =====\n[printer]\nkinematics = none\n======================="),
        );

        let dir = TempDir::new("header");
        let (output, now) = test_log(&dir, local(2026, 6, 10, 23, 59, 59));
        append(&output, "yesterday\n");
        now.store(local(2026, 6, 11, 0, 0, 1), Ordering::SeqCst);
        append(&output, "today\n");
        clear_rollover_info();

        let current = std::fs::read_to_string(dir.path("klippy.log")).unwrap();
        // Sorted by name — `config` before `versions` — and then the banner, all
        // of it ahead of the record that caused the rotation.
        assert!(
            current.starts_with(concat!(
                "===== Config file =====\n",
                "[printer]\n",
                "kinematics = none\n",
                "=======================\n",
                "Versions: klipperx test\n",
                "=============== Log rollover at ",
            )),
            "{current}"
        );
        assert!(
            current.contains("[printer]"),
            "the loaded config: {current}"
        );
        assert!(current.ends_with("today\n"), "{current}");
    }

    /// Five rotated files are kept, the oldest going as the newest arrives
    /// (`backupCount=5`), and the file in use is always `klippy.log`.
    #[test]
    fn test_every_rotation_keeps_five_dated_files() {
        let _slot = exclusive();
        let (entries, _logs) = unbounded_channel();
        let _window = to_window(entries);

        let dir = TempDir::new("backups");
        let (output, now) = test_log(&dir, local(2026, 6, 10, 12, 0, 0));
        // Eight days of use: the first write opens the file, each later one lands
        // on a new day and rotates what the day before left.
        for day in 10..=17 {
            now.store(local(2026, 6, day, 12, 0, 0), Ordering::SeqCst);
            append(&output, &format!("day {day}\n"));
        }

        let expected: Vec<String> = (12..=16)
            .map(|day| day_name(local(2026, 6, day, 0, 0, 0)))
            .collect();
        let mut with_current = expected.clone();
        with_current.push("klippy.log".to_string());
        with_current.sort();
        assert_eq!(
            dir.names(),
            with_current,
            "five rotated files plus the one in use"
        );
        assert_eq!(
            std::fs::read_to_string(dir.path("klippy.log")).unwrap(),
            "day 17\n",
            "the file in use is the newest one"
        );
    }

    /// Without `--logfile` there is no file: the day boundary passes with nothing
    /// to rotate and nothing written anywhere.
    #[test]
    fn test_no_logfile_means_nothing_to_rotate() {
        let _slot = exclusive();
        let (entries, _logs) = unbounded_channel();
        let _window = to_window(entries);

        let dir = TempDir::new("absent");
        let now = Arc::new(AtomicI64::new(local(2026, 6, 10, 23, 59, 59)));
        let clock: Clock = {
            let now = Arc::clone(&now);
            Arc::new(move || now.load(Ordering::SeqCst))
        };
        // No `--logfile`: there is no file behind the sink, so a day boundary has
        // nothing to rotate and nothing is written anywhere.
        let output = LogOutput::with_clock(None, clock);

        append(&output, "no file\n");
        now.store(local(2026, 6, 11, 0, 0, 1), Ordering::SeqCst);
        append(&output, "still no file\n");

        assert!(dir.names().is_empty(), "{:?}", dir.names());
    }

    /// The boundary a rotation waits for is local midnight, not a fixed day's
    /// worth of seconds after the file was opened.
    #[test]
    fn test_the_rotation_boundary_is_local_midnight() {
        let start = local(2026, 6, 10, 13, 45, 10);
        let boundary = next_midnight(start);
        assert!(boundary > start);
        assert_eq!(local_time(b"%H:%M:%S\0", boundary), "00:00:00");
        assert_eq!(
            local_time(b"%Y-%m-%d\0", boundary),
            local_time(b"%Y-%m-%d\0", local(2026, 6, 11, 0, 0, 0))
        );
        // A moment that is already midnight belongs to the day that ended, so the
        // next boundary is the following one.
        let midnight = local(2026, 6, 11, 0, 0, 0);
        assert_eq!(next_midnight(midnight), local(2026, 6, 12, 0, 0, 0));
    }
}
