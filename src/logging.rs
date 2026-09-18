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

use std::io::Write;
use std::sync::Mutex;

use tokio::sync::mpsc::UnboundedSender;
use tracing::debug;
use tracing::field::{Field, Visit};
use tracing_subscriber::layer::{Context, Layer, SubscriberExt};
use tracing_subscriber::registry::LookupSpan;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::EnvFilter;

use klippy_client::session::{Entry, LogLevel};

/// Where a window, if one is up, receives this process's log records.
///
/// A global rather than a parameter because the subscriber is installed once,
/// before the command line has been looked at, and the window is opened later —
/// and because a log record can come from any thread. `None` means no window:
/// records go to stdout only.
static WINDOW: Mutex<Option<UnboundedSender<Entry>>> = Mutex::new(None);

/// Install the global tracing subscriber.
///
/// `--verbose` wins over `RUST_LOG`, because it is the more explicit of the two.
///
/// Installing twice is not an error: the second attempt is ignored, which is
/// what a user of the library as a library will do.
pub fn init(verbose: bool) {
    let filter = if verbose {
        EnvFilter::try_new("debug").expect("'debug' is a valid filter")
    } else {
        EnvFilter::try_from_default_env()
            .or_else(|_| EnvFilter::try_new("info"))
            .expect("'info' is a valid filter")
    };

    tracing_subscriber::registry()
        .with(filter)
        .with(
            tracing_subscriber::fmt::layer()
                .with_target(false)
                .with_writer(HostOutput),
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
pub fn to_window(entries: UnboundedSender<Entry>) -> WindowGuard {
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
fn window() -> Option<UnboundedSender<Entry>> {
    WINDOW
        .lock()
        .expect("the window slot is not poisoned")
        .clone()
}

/// Where the formatted output goes: stdout, or nowhere while a window is up.
#[derive(Clone, Copy)]
struct HostOutput;

impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for HostOutput {
    type Writer = Box<dyn Write + 'a>;

    fn make_writer(&'a self) -> Self::Writer {
        if window().is_some() {
            Box::new(std::io::sink())
        } else {
            Box::new(std::io::stdout())
        }
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
            tracing::Level::TRACE | tracing::Level::DEBUG => LogLevel::Debug,
            tracing::Level::INFO => LogLevel::Info,
            tracing::Level::WARN => LogLevel::Warn,
            tracing::Level::ERROR => LogLevel::Error,
        };

        // The window may have gone away between the check above and here; a
        // dropped record is not worth reporting, since the only place to report
        // it is the window that is gone.
        let _ = entries.send(Entry::Log {
            level,
            text: message.0,
        });
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
            tracing::info!("API server listening on {}", "unix:/tmp/x");
        });

        let received: Vec<(LogLevel, String)> = std::iter::from_fn(|| logs.try_recv().ok())
            .map(|entry| match entry {
                Entry::Log { level, text } => (level, text),
                other => panic!("expected a log entry, got {other:?}"),
            })
            .collect();

        assert_eq!(
            received,
            vec![
                (
                    LogLevel::Info,
                    "Successfully parsed config with 1 sections".into()
                ),
                (LogLevel::Warn, "dropping malformed request".into()),
                (LogLevel::Debug, "a detail".into()),
                (LogLevel::Info, "API server listening on unix:/tmp/x".into()),
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
}
